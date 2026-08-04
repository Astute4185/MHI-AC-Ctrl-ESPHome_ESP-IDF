#!/usr/bin/env python3

import ast
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPONENT_DIR = REPO_ROOT / "components" / "MhiAcCtrl"
COMPONENT_INIT = COMPONENT_DIR / "__init__.py"
MANAGER_HEADER = COMPONENT_DIR / "mhi_transport_manager.h"

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
        "#endif  // MHI_USE_TRANSPORT_FAST_GPIO and supported split-TX target",
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

    def test_manager_enables_concrete_drivers_only_from_codegen_defines(self):
        source = MANAGER_HEADER.read_text(encoding="utf-8")

        self.assertIn("defined(MHI_USE_TRANSPORT_FAST_GPIO)", source)
        self.assertIn("defined(MHI_USE_TRANSPORT_EXTERNAL_CLOCK)", source)
        self.assertIn("defined(MHI_USE_TRANSPORT_RMT_SPI)", source)
        self.assertIn("defined(MHI_USE_TRANSPORT_RMT_CS_SPI)", source)
        self.assertIn("MHI_ENABLE_SPLIT_TX_DRIVER", source)
        self.assertIn(
            "defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)",
            source,
        )

    def test_each_transport_implementation_has_a_whole_unit_guard(self):
        for filename, (opening_guard, closing_guard) in TRANSPORT_SOURCES.items():
            with self.subTest(source=filename):
                source = (COMPONENT_DIR / filename).read_text(encoding="utf-8")
                prefix = f'#include "esphome/core/defines.h"\n\n{opening_guard}'
                self.assertTrue(source.startswith(prefix))
                self.assertTrue(source.rstrip().endswith(closing_guard))


if __name__ == "__main__":
    unittest.main()
