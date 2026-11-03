"""Standalone configuration tests (do not require the ESPHome Python package)."""

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "components/MhiAcCtrl/climate/setpoint_config.py"
SPEC = importlib.util.spec_from_file_location("mhi_setpoint_config", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
resolve_setpoint_step = MODULE.resolve_setpoint_step


class ClimateSetpointConfigTests(unittest.TestCase):
    def test_default_is_one_degree(self):
        self.assertEqual(resolve_setpoint_step({}), 1.0)
        self.assertEqual(
            resolve_setpoint_step({"visual": {"temperature_step": {"current_temperature": 0.1}}}), 1.0
        )

    def test_explicit_opt_in(self):
        self.assertEqual(resolve_setpoint_step({"setpoint_step": 0.5}), 0.5)

    def test_legacy_ui_settings_remain_supported(self):
        self.assertEqual(resolve_setpoint_step({"visual_temperature_step": 0.5}), 0.5)
        self.assertEqual(
            resolve_setpoint_step(
                {"visual": {"temperature_step": {"target_temperature": 0.5, "current_temperature": 0.1}}}
            ),
            0.5,
        )
        self.assertEqual(resolve_setpoint_step({"visual": {"temperature_step": 0.5}}), 0.5)

    def test_matching_explicit_and_visual_settings(self):
        self.assertEqual(
            resolve_setpoint_step(
                {"setpoint_step": 0.5, "visual": {"temperature_step": {"target_temperature": 0.5}}}
            ),
            0.5,
        )

    def test_conflicting_resolution_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Conflicting"):
            resolve_setpoint_step({"setpoint_step": 1.0, "visual_temperature_step": 0.5})

    def test_unsupported_resolution_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "must be either"):
            resolve_setpoint_step({"setpoint_step": 0.25})


if __name__ == "__main__":
    unittest.main()
