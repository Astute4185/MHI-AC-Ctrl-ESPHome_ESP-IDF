#!/usr/bin/env python3
"""Regression checks for the component logging severity policy."""

from pathlib import Path
import re
import unittest

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPONENT_DIR = REPO_ROOT / "components" / "MhiAcCtrl"


class LogSeverityPolicyTests(unittest.TestCase):
    def read(self, filename: str) -> str:
        return (COMPONENT_DIR / filename).read_text(encoding="utf-8")

    def test_periodic_runtime_telemetry_is_debug(self):
        sources = (
            "mhi_ac_ctrl.cpp",
            "mhi_rmt_cs_spi_transport.cpp",
            "mhi_rmt_spi_rx_driver.cpp",
            "mhi_external_clock_rx_driver.cpp",
            "mhi_null_tx_driver.cpp",
        )

        for filename in sources:
            with self.subTest(source=filename):
                source = self.read(filename)
                self.assertIsNone(
                    re.search(r'ESP_LOGI\([^;]*?"(?:runtime:|probe:)', source, re.DOTALL),
                    f"periodic telemetry must not use INFO in {filename}",
                )

        controller = self.read("mhi_ac_ctrl.cpp")
        self.assertIn('ESP_LOGD(DIAG_TAG,\n           "runtime: transport state=', controller)
        self.assertIn('ESP_LOGD(DIAG_TAG,\n             "runtime: rx_protocol_health healthy=YES', controller)
        self.assertIn('ESP_LOGW(DIAG_TAG,\n             "runtime: rx_protocol_health healthy=NO', controller)

    def test_command_lifecycle_is_visible_at_info(self):
        source = self.read("mhi_ac_ctrl.cpp")

        self.assertIn('ESP_LOGI(DIAG_TAG, "command: superseded pending confirmation', source)
        self.assertIn('ESP_LOGI(DIAG_TAG,\n             "command: staged=', source)
        self.assertIn('ESP_LOGI(DIAG_TAG, "command: confirmed mask=', source)
        self.assertIn('ESP_LOGW(DIAG_TAG, "command: rejected while Active Mode is off', source)
        self.assertIn('ESP_LOGW(DIAG_TAG, "command: confirmation timeout', source)
        self.assertIn('ESP_LOGW(DIAG_TAG, "command: confirmation exhausted', source)

    def test_normal_driver_startup_is_not_a_warning(self):
        rmt_cs_spi = self.read("mhi_rmt_cs_spi_transport.cpp")
        rmt_spi_rx = self.read("mhi_rmt_spi_rx_driver.cpp")
        external_clock = self.read("mhi_external_clock_rx_driver.cpp")

        self.assertIn('ESP_LOGI(TAG,\n           "RMT-CS SPI duplex enabled:', rmt_cs_spi)
        self.assertIn('ESP_LOGI(TAG, "Applied original ESP32 FIFO mode-3 edge correction")', rmt_cs_spi)
        self.assertIn('ESP_LOGI(TAG,\n           "RMT/SPI RX enabled:', rmt_spi_rx)
        self.assertIn('ESP_LOGI(EXTERNAL_CLOCK_RX_TAG,\n           "External-clock RX probe enabled:', external_clock)

    def test_intentional_tx_disable_remains_warning(self):
        source = self.read("mhi_null_tx_driver.cpp")
        self.assertIn('ESP_LOGW(NULL_TX_TAG, "TX disabled.', source)
        self.assertIn('ESP_LOGD(NULL_TX_TAG, "probe: dropped_tx_frames=', source)


if __name__ == "__main__":
    unittest.main()
