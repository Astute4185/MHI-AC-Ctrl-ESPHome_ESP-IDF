#!/usr/bin/env python3

import re
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
TEST_SCRIPT = REPO_ROOT / "scripts" / "test.sh"
COMPONENT_DIR = REPO_ROOT / "components" / "MhiAcCtrl"
GENERATED_DIR_NAMES = {".esphome", ".test-build", ".pioenvs", "build"}

PATH_PATTERN = re.compile(r"^\s*((?:tests/unit|components/MhiAcCtrl)/[^\s]+\.(?:cpp|c))\s*$", re.MULTILINE)
OBSOLETE_CONTROLLER_SETTERS = (
    "set_rx_driver(",
    "set_tx_driver(",
    "set_sck_pin(",
    "set_mosi_pin(",
    "set_miso_pin(",
    "set_frame_start_idle_ms(",
    "set_rmt_spi_frame_gap_us(",
)



class RepositoryHygieneTests(unittest.TestCase):
    def test_host_test_manifest_only_references_existing_sources(self):
        source = TEST_SCRIPT.read_text(encoding="utf-8")
        listed = PATH_PATTERN.findall(source)

        self.assertGreater(len(listed), 20)
        self.assertEqual(len(listed), len(set(listed)), "Duplicate source in scripts/test.sh")

        missing = [relative for relative in listed if not (REPO_ROOT / relative).is_file()]
        self.assertEqual([], missing, f"Missing sources in scripts/test.sh: {missing}")

    def test_all_cpp_unit_tests_are_explicitly_listed(self):
        source = TEST_SCRIPT.read_text(encoding="utf-8")
        listed = set(PATH_PATTERN.findall(source))
        discovered = {
            str(path.relative_to(REPO_ROOT))
            for path in (REPO_ROOT / "tests" / "unit").glob("test_*.cpp")
        }

        self.assertEqual(
            discovered,
            {path for path in listed if path.startswith("tests/unit/test_")},
            "Host unit-test manifest and tests/unit/test_*.cpp are out of sync",
        )

    def test_removed_worker_policy_is_absent(self):
        self.assertFalse((COMPONENT_DIR / "mhi_worker_policy.h").exists())
        self.assertFalse((REPO_ROOT / "tests" / "unit" / "test_worker_policy.cpp").exists())

        matches = []
        for path in COMPONENT_DIR.rglob("*"):
            if not path.is_file() or any(part in GENERATED_DIR_NAMES for part in path.parts):
                continue
            if path.suffix not in {".h", ".hpp", ".cpp", ".c", ".py"}:
                continue
            text = path.read_text(encoding="utf-8", errors="replace")
            if "mhi_rx_driver_supports_classified_worker" in text:
                matches.append(str(path.relative_to(REPO_ROOT)))

        self.assertEqual([], matches)

    def test_obsolete_controller_transport_setters_are_absent(self):
        files = (
            COMPONENT_DIR / "__init__.py",
            COMPONENT_DIR / "mhi_ac_ctrl.h",
            COMPONENT_DIR / "mhi_ac_ctrl.cpp",
        )
        matches = []
        for path in files:
            text = path.read_text(encoding="utf-8", errors="replace")
            for token in OBSOLETE_CONTROLLER_SETTERS:
                if token in text:
                    matches.append(f"{path.relative_to(REPO_ROOT)}: {token}")

        self.assertEqual([], matches)

    def test_transport_implementation_files_remain_flat(self):
        nested_transport_sources = [
            path.relative_to(COMPONENT_DIR)
            for path in COMPONENT_DIR.rglob("mhi_*transport*.cpp")
            if path.parent != COMPONENT_DIR
        ]
        self.assertEqual([], nested_transport_sources)


if __name__ == "__main__":
    unittest.main()
