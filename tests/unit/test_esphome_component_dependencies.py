#!/usr/bin/env python3

import ast
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPONENT_DIR = REPO_ROOT / "components" / "MhiAcCtrl"
COMPONENT_INIT = COMPONENT_DIR / "__init__.py"
RMT_DRIVER_MODULES = (
    COMPONENT_DIR / "mhi_transport_rmt_spi.py",
    COMPONENT_DIR / "mhi_transport_rmt_cs_spi.py",
)


class EspHomeComponentDependencyTests(unittest.TestCase):
    def test_component_enables_selected_registry_idf_dependencies(self):
        tree = ast.parse(COMPONENT_INIT.read_text(encoding="utf-8"))

        imports_include_helper = False
        resolves_dependencies = False
        enables_resolved_component = False

        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module == "esphome.components.esp32":
                imports_include_helper = any(
                    alias.name == "include_builtin_idf_component" for alias in node.names
                )
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                if node.func.id == "resolve_selected_idf_components":
                    resolves_dependencies = True
                elif node.func.id == "include_builtin_idf_component" and node.args:
                    enables_resolved_component = isinstance(node.args[0], ast.Name)

        self.assertTrue(imports_include_helper)
        self.assertTrue(resolves_dependencies)
        self.assertTrue(enables_resolved_component)

    def test_rmt_driver_modules_declare_rmt_dependency(self):
        for module_path in RMT_DRIVER_MODULES:
            with self.subTest(module=module_path.name):
                tree = ast.parse(module_path.read_text(encoding="utf-8"))
                string_values = {
                    node.value
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Constant) and isinstance(node.value, str)
                }
                self.assertIn("esp_driver_rmt", string_values)

    def test_root_component_no_longer_uses_target_wide_dependency_resolution(self):
        source = COMPONENT_INIT.read_text(encoding="utf-8")
        self.assertNotIn("resolve_legacy_build_idf_components", source)
        self.assertNotIn('include_builtin_idf_component("esp_driver_rmt")', source)


if __name__ == "__main__":
    unittest.main()
