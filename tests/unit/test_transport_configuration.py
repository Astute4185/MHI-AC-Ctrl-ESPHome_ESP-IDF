#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "components" / "MhiAcCtrl"))

from mhi_transport_registry import (  # noqa: E402
    DEFAULT_FRAME_START_IDLE_MS,
    DEFAULT_RMT_SPI_FRAME_GAP_US,
    TRANSPORT_DEFINITIONS,
    TransportConfigurationError,
    resolve_transport_tuning,
    validate_driver_subsections,
)


class TransportConfigurationTests(unittest.TestCase):
    def test_registry_contains_all_current_rx_drivers(self):
        self.assertEqual(
            set(TRANSPORT_DEFINITIONS),
            {"fast_gpio_rx", "external_clock_rx", "rmt_spi_rx", "rmt_cs_spi"},
        )

    def test_defaults_preserve_current_runtime_values(self):
        tuning = resolve_transport_tuning({"rx_driver": "rmt_spi_rx"})

        self.assertEqual(tuning.frame_start_idle_ms, DEFAULT_FRAME_START_IDLE_MS)
        self.assertEqual(tuning.rmt_spi_frame_gap_us, DEFAULT_RMT_SPI_FRAME_GAP_US)

    def test_legacy_flat_rmt_gap_is_preserved(self):
        tuning = resolve_transport_tuning(
            {
                "rx_driver": "rmt_spi_rx",
                "rmt_spi_frame_gap_us": 1250,
            }
        )

        self.assertEqual(tuning.rmt_spi_frame_gap_us, 1250)

    def test_nested_rmt_spi_gap_overrides_default(self):
        tuning = resolve_transport_tuning(
            {
                "rx_driver": "rmt_spi_rx",
                "rmt_spi_rx": {"frame_gap_us": 1250},
            }
        )

        self.assertEqual(tuning.rmt_spi_frame_gap_us, 1250)

    def test_nested_rmt_cs_gap_overrides_default(self):
        tuning = resolve_transport_tuning(
            {
                "rx_driver": "rmt_cs_spi",
                "rmt_cs_spi": {"frame_gap_us": 1500},
            }
        )

        self.assertEqual(tuning.rmt_spi_frame_gap_us, 1500)

    def test_nested_fast_gpio_idle_overrides_default(self):
        tuning = resolve_transport_tuning(
            {
                "rx_driver": "fast_gpio_rx",
                "fast_gpio_rx": {"frame_start_idle_ms": 12},
            }
        )

        self.assertEqual(tuning.frame_start_idle_ms, 12)

    def test_unselected_driver_subsection_is_rejected(self):
        with self.assertRaisesRegex(TransportConfigurationError, "external_clock_rx.*rmt_spi_rx"):
            validate_driver_subsections(
                {
                    "rx_driver": "rmt_spi_rx",
                    "external_clock_rx": {},
                }
            )

    def test_duplicate_legacy_and_nested_rmt_gap_is_rejected(self):
        with self.assertRaisesRegex(TransportConfigurationError, "either as rmt_spi_frame_gap_us"):
            resolve_transport_tuning(
                {
                    "rx_driver": "rmt_spi_rx",
                    "rmt_spi_frame_gap_us": 1000,
                    "rmt_spi_rx": {"frame_gap_us": 1250},
                }
            )

    def test_duplicate_legacy_and_nested_fast_gpio_idle_is_rejected(self):
        with self.assertRaisesRegex(TransportConfigurationError, "either at MhiAcCtrl level"):
            resolve_transport_tuning(
                {
                    "rx_driver": "fast_gpio_rx",
                    "frame_start_idle_ms": 10,
                    "fast_gpio_rx": {"frame_start_idle_ms": 12},
                }
            )

    def test_hardware_transports_request_internal_fast_gpio_recovery(self):
        self.assertFalse(TRANSPORT_DEFINITIONS["fast_gpio_rx"].uses_internal_fast_gpio_recovery)
        self.assertTrue(TRANSPORT_DEFINITIONS["external_clock_rx"].uses_internal_fast_gpio_recovery)
        self.assertTrue(TRANSPORT_DEFINITIONS["rmt_spi_rx"].uses_internal_fast_gpio_recovery)
        self.assertTrue(TRANSPORT_DEFINITIONS["rmt_cs_spi"].uses_internal_fast_gpio_recovery)


if __name__ == "__main__":
    unittest.main()
