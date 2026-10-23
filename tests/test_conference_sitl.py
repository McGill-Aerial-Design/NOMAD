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
    def cache(self, **changes):
        values = dict(builder.FEATURES, **changes)
        flags = ["-O3"] + ["-D" + key + "=" + value for key, value in values.items() if key != "AP_SCRIPTING_ENABLED"]
        defines = ["CONFIG_HAL_BOARD=HAL_BOARD_SITL", "AP_SCRIPTING_ENABLED=" + values["AP_SCRIPTING_ENABLED"]]
        return "BOARD = 'sitl'\nCXXFLAGS = " + repr(flags) + "\nDEFINES = " + repr(defines) + "\n"

    def test_requires_both_afs_features_and_native_mode_without_lua(self):
        symbols = "00000001401ae450 T ModeTerminate::run()"
        self.assertEqual(builder.verify_features(self.cache(), symbols), builder.FEATURES)
        for change in (
            {"AP_ADVANCEDFAILSAFE_ENABLED": "0"},
            {"AP_COPTER_ADVANCED_FAILSAFE_ENABLED": "0"},
            {"AP_SCRIPTING_ENABLED": "1"},
        ):
            with self.assertRaises(ValueError):
                builder.verify_features(self.cache(**change), symbols)
        with self.assertRaises(ValueError):
            builder.verify_features(
                self.cache().replace("-DAP_ADVANCEDFAILSAFE_ENABLED=1", "-DSOME_OTHER_FEATURE=1"), symbols
            )
        with self.assertRaises(ValueError):
            builder.verify_features(self.cache(), "0000 T ModeLand::run()")

    def test_conflicting_required_definitions_are_rejected(self):
        cache = self.cache().replace("'CONFIG_HAL_BOARD=HAL_BOARD_SITL'", "'AP_ADVANCEDFAILSAFE_ENABLED=0'")
        with self.assertRaisesRegex(ValueError, "Conflicting"):
            builder.verify_features(cache, "T ModeTerminate::run()")

    def test_nonliteral_cache_code_and_wrong_definition_shapes_are_rejected(self):
        for cache in (
            "import os\n" + self.cache(),
            "run_some_code()\n" + self.cache(),
            "CXXFLAGS = load_flags()\nDEFINES = []\n",
            self.cache().replace("CXXFLAGS = ['-O3',", "CXXFLAGS = [None,"),
            self.cache().split("DEFINES =")[0] + "DEFINES = ('AP_SCRIPTING_ENABLED=0',)\n",
            self.cache().replace("CXXFLAGS =", "CXXDEFINES ="),
            self.cache() + "CXXFLAGS = []\n",
            self.cache() + "SIDE_EFFECT = open('operator-file', 'w')\n",
            self.cache() + "NOT_LITERAL = set()\n",
        ):
            with self.subTest(cache=cache), self.assertRaises((ValueError, SyntaxError)):
                builder.verify_features(cache, "T ModeTerminate::run()")

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
