#!/usr/bin/env python3

import ast
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPONENT_DIR = REPO_ROOT / "components" / "MhiAcCtrl"
COMPONENT_INIT = COMPONENT_DIR / "__init__.py"
MANAGER_HEADER = COMPONENT_DIR / "mhi_transport_manager.h"
CONTROLLER_HEADER = COMPONENT_DIR / "mhi_ac_ctrl.h"
WORKER_POLICY_HEADER = COMPONENT_DIR / "mhi_worker_policy.h"
WORKER_POLICY_TEST = REPO_ROOT / "tests" / "unit" / "test_worker_policy.cpp"

TRANSPORT_SOURCES = {
    "mhi_fast_gpio_rx_driver.cpp": (
        "#ifdef MHI_USE_TRANSPORT_FAST_GPIO",
        "#endif  // MHI_USE_TRANSPORT_FAST_GPIO",
    ),
    "mhi_fast_gpio_tx_driver.cpp": (
        "#if defined(MHI_USE_TRANSPORT_FAST_GPIO) && defined(USE_ESP_IDF)",
        "#endif  // MHI_USE_TRANSPORT_FAST_GPIO and supported split-TX target",
    ),
    "mhi_null_tx_driver.cpp": (
        "#if defined(MHI_USE_TRANSPORT_FAST_GPIO) && defined(USE_ESP_IDF)",
        "#endif  // MHI_USE_TRANSPORT_FAST_GPIO and USE_ESP_IDF",
    ),
    "mhi_external_clock_rx_driver.cpp": (
        "#ifdef MHI_USE_TRANSPORT_EXTERNAL_CLOCK",
        "#endif  // MHI_USE_TRANSPORT_EXTERNAL_CLOCK",
    ),
    "mhi_rmt_spi_rx_driver.cpp": (
        "#ifdef MHI_USE_TRANSPORT_RMT_SPI",
        "#endif  // MHI_USE_TRANSPORT_RMT_SPI",
    ),
    "mhi_rmt_cs_spi_transport.cpp": (
        "#ifdef MHI_USE_TRANSPORT_RMT_CS_SPI",
        "#endif  // MHI_USE_TRANSPORT_RMT_CS_SPI",
    ),
}


class TransportCompileSelectionTests(unittest.TestCase):
    def test_root_codegen_emits_registry_selected_defines_and_dependencies(self):
        tree = ast.parse(COMPONENT_INIT.read_text(encoding="utf-8"))

        imported_names = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom)
            and node.module == "mhi_transport_registry"
            for alias in node.names
        }
        called_names = {
            node.func.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        called_attributes = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }

        self.assertIn("resolve_selected_compile_defines", imported_names)
        self.assertIn("resolve_selected_idf_components", imported_names)
        self.assertIn("resolve_selected_compile_defines", called_names)
        self.assertIn("resolve_selected_idf_components", called_names)
        self.assertIn("add_define", called_attributes)
        self.assertIn("include_builtin_idf_component", called_names)
        self.assertIn("build_selected_transports", imported_names)
        self.assertIn("build_selected_transports", called_names)
        self.assertIn("set_primary_transport", called_attributes)
        self.assertIn("set_recovery_transport", called_attributes)
        self.assertIn("set_opdata_freshness_timeout_ms", called_attributes)

        for obsolete_setter in (
            "set_sck_pin",
            "set_mosi_pin",
            "set_miso_pin",
            "set_rx_driver",
            "set_tx_driver",
            "set_frame_start_idle_ms",
            "set_rmt_spi_frame_gap_us",
        ):
            self.assertNotIn(obsolete_setter, called_attributes)

    def test_manager_is_non_owning_and_has_no_concrete_driver_selection(self):
        source = MANAGER_HEADER.read_text(encoding="utf-8")

        self.assertIn("IMhiTransport* primary_", source)
        self.assertIn("IMhiTransport* recovery_", source)
        self.assertIn("void set_primary(IMhiTransport* transport)", source)
        self.assertIn("void set_recovery(IMhiTransport* transport)", source)

        for forbidden in (
            "MHI_USE_TRANSPORT_FAST_GPIO",
            "MHI_USE_TRANSPORT_EXTERNAL_CLOCK",
            "MHI_USE_TRANSPORT_RMT_SPI",
            "MHI_USE_TRANSPORT_RMT_CS_SPI",
            "MhiFastGpioRxDriver",
            "MhiFastGpioTxDriver",
            "MhiExternalClockRxDriver",
            "MhiRmtSpiRxDriver",
            "MhiRmtCsSpiTransport",
            "MhiTransportPins pins_",
            "requested_rx_driver_name_",
        ):
            self.assertNotIn(forbidden, source)


    def test_controller_is_transport_agnostic(self):
        source = CONTROLLER_HEADER.read_text(encoding="utf-8")

        for forbidden in (
            "struct MhiPins",
            "MhiPins pins_",
            "rx_driver_",
            "tx_driver_",
            "frame_start_idle_ms_",
            "rmt_spi_frame_gap_us_",
            "void set_sck_pin",
            "void set_mosi_pin",
            "void set_miso_pin",
            "void set_rx_driver",
            "void set_tx_driver",
        ):
            self.assertNotIn(forbidden, source)

    def test_obsolete_string_worker_policy_is_removed(self):
        self.assertFalse(WORKER_POLICY_HEADER.exists())
        self.assertFalse(WORKER_POLICY_TEST.exists())

    def test_each_transport_implementation_has_a_whole_unit_guard(self):
        for filename, (opening_guard, closing_guard) in TRANSPORT_SOURCES.items():
            with self.subTest(source=filename):
                source = (COMPONENT_DIR / filename).read_text(encoding="utf-8")
                prefix = f'#include "esphome/core/defines.h"\n\n{opening_guard}'
                self.assertTrue(source.startswith(prefix))
                self.assertTrue(source.rstrip().endswith(closing_guard))


if __name__ == "__main__":
    unittest.main()
