# SPDX-License-Identifier: Apache-2.0
"""Packager integrity checks only; these do not qualify firmware behavior."""

import base64
import copy
import importlib.util
import io
import json
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest import mock

REPO = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("firmware_release", REPO / "scripts/release/build_firmware.py")
firmware = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(firmware)


class FirmwareReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.addCleanup(self.temporary.cleanup)

    def identity(self):
        return json.loads((REPO / "firmware/ardupilot/source.json").read_text())

    def write_identity(self, identity):
        path = self.root / "source.json"
        firmware.write_json(path, identity)
        return path

    def test_reviewed_identity_and_all_recursive_pins_are_required(self):
        identity = self.identity()
        self.assertEqual(len(firmware.load_identity(self.write_identity(identity))["submodules"]), 21)
        mutations = []
        for key, value in (
            ("revision", "a" * 40),
            ("board", "CubeOrange"),
            ("apj_board_id", True),
            ("scripting_enabled", True),
            ("patch_sha256", "bad"),
        ):
            modified = copy.deepcopy(identity)
            modified[key] = value
            mutations.append(modified)
        missing = copy.deepcopy(identity)
        del missing["repository"]
        mutations.append(missing)
        missing_pin = copy.deepcopy(identity)
        missing_pin["submodules"].pop()
        mutations.append(missing_pin)
        duplicate = copy.deepcopy(identity)
        duplicate["submodules"][0] = duplicate["submodules"][1]
        mutations.append(duplicate)
        escaping = copy.deepcopy(identity)
        escaping["submodules"][0]["path"] = "modules/../../outside"
        mutations.append(escaping)
        for value in mutations:
            with self.subTest(value=value), self.assertRaises(ValueError):
                firmware.load_identity(self.write_identity(value))

    def test_gitlink_inventory_must_match_every_pin(self):
        identity = {"revision": "a" * 40, "submodules": [{"path": "modules/a", "revision": "b" * 40}]}

        def command(path, *args):
            if args[0] == "submodule":
                return " " + "b" * 40 + " modules/a"
            return "b" * 40 if path.parts[-2:] == ("modules", "a") else "a" * 40

        with mock.patch.object(firmware, "git", side_effect=command):
            firmware.verify_revisions(self.root, identity)
            identity["submodules"][0]["revision"] = "c" * 40
            with self.assertRaisesRegex(ValueError, "pins differ"):
                firmware.verify_revisions(self.root, identity)

    def test_apj_requires_board_size_and_identical_decoded_image(self):
        binary, apj = self.root / "firmware.bin", self.root / "firmware.apj"
        binary.write_bytes(b"known firmware image")
        value = {
            "board_id": 1063,
            "image_size": binary.stat().st_size,
            "image": base64.b64encode(zlib.compress(binary.read_bytes())).decode(),
        }
        firmware.write_json(apj, value)
        self.assertEqual(firmware.validate_apj(apj, binary)["image_sha256"], firmware.digest(binary))
        for key, bad in (("board_id", 1062), ("image_size", 1)):
            modified = dict(value, **{key: bad})
            firmware.write_json(apj, modified)
            with self.assertRaises(ValueError):
                firmware.validate_apj(apj, binary)
        firmware.write_json(apj, value)
        binary.write_bytes(b"different image")
        with self.assertRaisesRegex(ValueError, "differs"):
            firmware.validate_apj(apj, binary)

    def test_feature_check_requires_afs_vehicle_and_disabled_scripting(self):
        common = "AP_ADVANCEDFAILSAFE_ENABLED\n!AP_SCRIPTING_ENABLED\n"
        firmware.validate_features(common + "AP_COPTER_ADVANCED_FAILSAFE_ENABLED\n", "arducopter")
        firmware.validate_features(common + "HAL_QUADPLANE_ENABLED\n", "arduplane")
        for features in (common, "HAL_QUADPLANE_ENABLED\nAP_SCRIPTING_ENABLED\n", "!HAL_QUADPLANE_ENABLED\n" + common):
            with self.assertRaises(ValueError):
                firmware.validate_features(features, "arduplane")

    def test_missing_running_failed_and_extra_gates_never_pass(self):
        gates = {name: {"status": "PASS"} for name in firmware.MANDATORY_GATES}
        self.assertTrue(firmware.overall_pass(gates))
        for status in ("FAIL", "RUNNING", "SKIPPED"):
            invalid = copy.deepcopy(gates)
            invalid["hardware"]["status"] = status
            self.assertFalse(firmware.overall_pass(invalid))
        gates.pop("hardware")
        self.assertFalse(firmware.overall_pass(gates))
        gates["unknown"] = {"status": "PASS"}
        self.assertFalse(firmware.overall_pass(gates))

    def test_existing_work_directory_is_refused(self):
        with self.assertRaisesRegex(ValueError, "already exist"):
            firmware.Pipeline(REPO, self.root, 1)

    def test_failure_writes_results_and_no_release_assets(self):
        pipeline = firmware.Pipeline(REPO, self.root / "work", 1)
        with mock.patch.object(pipeline, "inputs", side_effect=ValueError("invalid patch hash")):
            with self.assertRaises(ValueError):
                pipeline.run()
        result = json.loads((pipeline.work / "results.json").read_text())
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["gates"]["inputs"]["status"], "FAIL")
        self.assertFalse(pipeline.package.exists())
        with self.assertRaisesRegex(ValueError, "every qualification gate"):
            pipeline.package_assets()

    def packaging_fixture(self):
        pipeline = firmware.Pipeline(REPO, self.root / "work", 1)
        image, recipe = self.root / "image.apj", self.root / "recipe.txt"
        image.write_bytes(b"qualified image")
        recipe.write_text("qualified recipe")
        pipeline.archive = self.root / "source.tar.gz"
        pipeline.archive.write_bytes(b"qualified source archive")
        (pipeline.work / "source-content.json").write_text("{}")
        pipeline.identity = {"revision": firmware.REVISION}
        pipeline.hardware_files, pipeline.input_files = [(image, image.name)], [(recipe, recipe.name)]
        pipeline.hardware_hashes = {image.name: firmware.digest(image)}
        pipeline.input_hashes = {recipe.name: firmware.digest(recipe)}
        pipeline.details = {"source_archive_sha256": firmware.digest(pipeline.archive)}
        pipeline.gates = {name: {"status": "PASS"} for name in firmware.MANDATORY_GATES if name != "package"}
        return pipeline

    def test_packaging_rejects_inputs_changed_after_qualification(self):
        pipeline = self.packaging_fixture()
        pipeline.input_files[0][0].write_text("changed after stable gate")
        with self.assertRaisesRegex(ValueError, "asset changed after qualification"):
            pipeline.package_assets()
        self.assertFalse(pipeline.package.exists())

    def test_checksum_write_failure_never_exposes_qualified_package(self):
        pipeline = self.packaging_fixture()
        original_write = Path.write_text

        def failing_write(path, *args, **kwargs):
            if path.name == "SHA256SUMS":
                raise OSError("disk failure fixture")
            return original_write(path, *args, **kwargs)

        with mock.patch.object(Path, "write_text", failing_write), self.assertRaises(OSError):
            pipeline.package_assets()
        self.assertFalse(pipeline.package.exists())

    def test_successful_package_has_complete_checksums_and_pass_manifest(self):
        pipeline = self.packaging_fixture()
        pipeline.gate("package", pipeline.package_assets)
        manifest = json.loads((pipeline.package / "manifest.json").read_text())
        self.assertTrue(firmware.overall_pass(manifest["gates"]))
        checked = set()
        for line in (pipeline.package / "SHA256SUMS").read_text().splitlines():
            checksum, name = line.split("  ")
            self.assertEqual(checksum, firmware.digest(pipeline.package / name))
            checked.add(name)
        self.assertEqual(checked, {path.name for path in pipeline.package.iterdir()} - {"SHA256SUMS"})
        self.assertEqual(json.loads((pipeline.work / "results.json").read_text())["status"], "PASS")

    @unittest.skipUnless(sys.platform == "linux", "owned process groups are Linux only")
    def test_command_failure_timeout_and_missing_native_marker_are_failures(self):
        pipeline = firmware.Pipeline(REPO, self.root / "work", 1)
        for args, timeout, token in (
            ([sys.executable, "-c", "raise SystemExit(3)"], 5, None),
            ([sys.executable, "-c", "print('unrelated success')"], 5, "test.Copter.Terminate"),
            ([sys.executable, "-c", "import time; time.sleep(10)"], 0.1, None),
        ):
            with self.subTest(args=args), self.assertRaises((ValueError, subprocess.TimeoutExpired)):
                pipeline.command(args, timeout=timeout, token=token)
        self.assertEqual(len(list(pipeline.diagnostics.glob("command-*.json"))), 3)
        self.assertTrue(json.loads((pipeline.diagnostics / "command-003.json").read_text())["timed_out"])

    @unittest.skipUnless(sys.platform == "linux", "detached child cleanup uses Linux subreaper")
    def test_timeout_reaps_owned_child_in_separate_session(self):
        pipeline = firmware.Pipeline(REPO, self.root / "work", 1)
        pidfile = self.root / "child.pid"
        code = (
            "import subprocess,sys,time; from pathlib import Path; "
            "child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],start_new_session=True); "
            f"Path({str(pidfile)!r}).write_text(str(child.pid)); time.sleep(30)"
        )
        with self.assertRaises(subprocess.TimeoutExpired):
            pipeline.command([sys.executable, "-c", code], timeout=1)
        self.assertTrue(pidfile.exists())
        self.assertFalse(Path("/proc", pidfile.read_text()).exists(), "detached owned child survived timeout cleanup")

    @unittest.skipUnless(shutil.which("git") and os.name == "posix", "source fixture uses native Git and symlinks")
    def test_complete_source_roundtrip_keeps_new_modes_and_public_git_only(self):
        source = self.root / "public-source"
        source.mkdir()
        subprocess.run(["git", "init", "-q", str(source)], check=True)
        for key, value in (
            ("user.name", "Fixture"),
            ("user.email", "fixture@example.invalid"),
            ("remote.secret.url", "https://private.invalid/secret"),
        ):
            firmware.git(source, "config", key, value)
        (source / "COPYING.txt").write_text("Upstream license fixture\n")
        (source / "tracked.txt").write_text("known public content\n")
        firmware.git(source, "add", ".")
        firmware.git(source, "commit", "-qm", "Test fixture")
        child = self.root / "public-child"
        child.mkdir()
        firmware.git(child, "init", "-q")
        firmware.git(child, "config", "user.name", "Fixture")
        firmware.git(child, "config", "user.email", "fixture@example.invalid")
        (child / "child.txt").write_text("public recursive source\n")
        firmware.git(child, "add", ".")
        firmware.git(child, "commit", "-qm", "Child fixture")
        firmware.git(source, "-c", "protocol.file.allow=always", "submodule", "add", str(child), "modules/child")
        firmware.git(source, "commit", "-qam", "Pinned child")
        (source / "build").mkdir()
        (source / "build/generated.bin").write_bytes(b"generated must be excluded")
        (source / "raw-runtime.tlog").write_bytes(b"raw diagnostic must be excluded")
        (source / ".git/hooks/private-hook").write_text("private metadata must be excluded")
        for relative in firmware.NEW_MODES:
            path = source / relative
            path.parent.mkdir()
            path.write_text("new mode source\n")
        stage = self.root / "source"
        revision = firmware.git(source, "rev-parse", "HEAD")
        identity = {
            "revision": revision,
            "submodules": [{"path": "modules/child", "revision": firmware.git(child, "rev-parse", "HEAD")}],
        }
        inputs = [
            (REPO / path, Path(path).name)
            for path in (
                "scripts/release/build_firmware.py",
                "scripts/release/firmware-requirements.txt",
                ".github/workflows/firmware.yml",
                "firmware/ardupilot/source.json",
                "firmware/ardupilot/controlled-termination.patch",
                "firmware/ardupilot/COPYING.txt",
                "docs/native-termination.md",
                "LICENSE",
            )
        ]
        original = firmware.freeze_source(source, stage, identity, inputs)
        for _, name in inputs:
            self.assertEqual(original[name]["mode"], 0o644, "Windows input mode must be portable")
        self.assertTrue(all("build/" not in name and "raw-runtime" not in name for name in original))
        self.assertFalse(any("hooks" in name or "logs" in name or "alternates" in name for name in original))
        self.assertNotIn("private", (stage / "ardupilot/.git/config").read_text())
        for relative in firmware.NEW_MODES:
            self.assertIn("ardupilot/" + relative, original)
        archive = self.root / "source.tar.gz"
        with tarfile.open(archive, "w:gz") as stream:
            for path in stage.iterdir():
                stream.add(path, arcname=path.name, filter=firmware.archive_header)
        with tarfile.open(archive) as stream:
            for entry in stream.getmembers():
                self.assertEqual((entry.uid, entry.gid, entry.uname, entry.gname, entry.mtime), (0, 0, "", "", 0))
        target = self.root / "extracted"
        self.assertEqual(firmware.extract_verified(archive, target, firmware.digest(archive)), original)
        self.assertEqual(firmware.git(target / "ardupilot", "rev-parse", "HEAD"), revision)
        self.assertEqual(firmware.git(target / "ardupilot", "show", "HEAD:tracked.txt"), "known public content")
        firmware.verify_revisions(target / "ardupilot", identity)
        self.assertIn("ardupilot/modules/child/child.txt", original)
        unchanged = firmware.content_manifest(target)
        unchanged.pop("source-content.json")
        self.assertEqual(unchanged, original, "revision verification modified frozen content")
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            firmware.extract_verified(archive, self.root / "bad-hash", "0" * 64)
        (stage / "ardupilot/tracked.txt").write_text("corrupt source\n")
        with tarfile.open(archive, "w:gz") as stream:
            for path in stage.iterdir():
                stream.add(path, arcname=path.name)
        with self.assertRaisesRegex(ValueError, "content or permissions mismatch"):
            firmware.extract_verified(archive, self.root / "corrupt", firmware.digest(archive))

    def test_archive_path_traversal_and_links_are_rejected(self):
        for name, link in (("../escape", None), ("/absolute", None), ("innocent", "../../escape")):
            archive = self.root / "unsafe.tar.gz"
            with tarfile.open(archive, "w:gz") as stream:
                member = tarfile.TarInfo(name)
                if link:
                    member.type, member.linkname = tarfile.SYMTYPE, link
                else:
                    member.size = 1
                stream.addfile(member, None if link else io.BytesIO(b"x"))
            target = self.root / ("extract-" + str(len(list(self.root.iterdir()))))
            with self.assertRaisesRegex(ValueError, "unsafe archive"):
                firmware.extract_verified(archive, target, firmware.digest(archive))


if __name__ == "__main__":
    unittest.main()
