# SPDX-License-Identifier: Apache-2.0
"""Pure packaging guards; no downloads, compiler, firmware process or operator files."""

import importlib.util
import unittest
from pathlib import Path

SPEC = importlib.util.spec_from_file_location(
    "conference_sitl", Path(__file__).resolve().parents[1] / "conference/build_sitl.py"
)
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class ConferenceSitlTests(unittest.TestCase):
    def header(self, **changes):
        values = dict(builder.FEATURES, **changes)
        return "\n".join("#define " + key + " " + value for key, value in values.items())

    def test_requires_both_afs_features_and_native_mode_without_lua(self):
        symbols = "00000001401ae450 T ModeTerminate::run()"
        self.assertEqual(builder.verify_features(self.header(), symbols), builder.FEATURES)
        for change in (
            {"AP_ADVANCEDFAILSAFE_ENABLED": "0"},
            {"AP_COPTER_ADVANCED_FAILSAFE_ENABLED": "0"},
            {"AP_SCRIPTING_ENABLED": "1"},
        ):
            with self.assertRaises(ValueError):
                builder.verify_features(self.header(**change), symbols)
        with self.assertRaises(ValueError):
            builder.verify_features(self.header(), "0000 T ModeLand::run()")

    def test_collects_only_cygwin_runtime_and_deduplicates_recursive_output(self):
        text = (
            "C:\\work\\arducopter.exe\n  C:\\cygwin\\bin\\cygwin1.dll\n"
            "    C:\\Windows\\System32\\KERNEL32.dll\n  C:\\cygwin\\bin\\cygstdc++-6.dll\n"
            "  C:\\cygwin\\bin\\cygwin1.dll\n"
        )
        self.assertEqual(
            builder.cygwin_dependencies(text),
            {"cygwin1.dll": "C:\\cygwin\\bin\\cygwin1.dll", "cygstdc++-6.dll": "C:\\cygwin\\bin\\cygstdc++-6.dll"},
        )

    def test_missing_ambiguous_and_relative_dependencies_are_rejected(self):
        for text in (
            "cygcheck: could not find cygwin1.dll",
            "C:\\Windows\\System32\\KERNEL32.dll",
            "cygwin1.dll",
            "C:\\a\\cygwin1.dll\nC:\\b\\cygwin1.dll",
        ):
            with self.subTest(text=text), self.assertRaises(ValueError):
                builder.cygwin_dependencies(text)


if __name__ == "__main__":
    unittest.main()
