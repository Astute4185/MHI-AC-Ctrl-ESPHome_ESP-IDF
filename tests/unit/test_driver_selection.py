#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "components" / "MhiAcCtrl"))

from driver_selection import DriverSelectionError, resolve_tx_driver  # noqa: E402


class DriverSelectionTests(unittest.TestCase):
    def test_fast_gpio_rx_defaults_to_fast_gpio_tx(self):
        self.assertEqual(resolve_tx_driver("fast_gpio_rx"), "fast_gpio_tx")

    def test_external_clock_rx_defaults_to_fast_gpio_tx(self):
        self.assertEqual(resolve_tx_driver("external_clock_rx"), "fast_gpio_tx")

    def test_rmt_spi_rx_defaults_to_fast_gpio_tx(self):
        self.assertEqual(resolve_tx_driver("rmt_spi_rx"), "fast_gpio_tx")

    def test_explicit_split_tx_override_is_preserved(self):
        self.assertEqual(resolve_tx_driver("rmt_spi_rx", "none"), "none")

    def test_rmt_cs_spi_owns_both_directions(self):
        self.assertEqual(resolve_tx_driver("rmt_cs_spi"), "rmt_cs_spi")

    def test_rmt_cs_spi_rejects_tx_override(self):
        with self.assertRaisesRegex(DriverSelectionError, "remove the tx_driver override"):
            resolve_tx_driver("rmt_cs_spi", "fast_gpio_tx")

    def test_unknown_rx_has_no_implicit_tx(self):
        with self.assertRaisesRegex(DriverSelectionError, "No default TX driver"):
            resolve_tx_driver("unknown_rx")


if __name__ == "__main__":
    unittest.main()
