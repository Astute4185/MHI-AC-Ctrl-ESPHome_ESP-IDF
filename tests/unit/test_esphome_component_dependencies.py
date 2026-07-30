#!/usr/bin/env python3

import ast
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPONENT_INIT = REPO_ROOT / "components" / "MhiAcCtrl" / "__init__.py"


class EspHomeComponentDependencyTests(unittest.TestCase):
    def test_rmt_idf_component_is_explicitly_enabled(self):
        tree = ast.parse(COMPONENT_INIT.read_text(encoding="utf-8"))

        imported = False
        enabled_components = set()

        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                if node.module == "esphome.components.esp32":
                    imported = any(
                        alias.name == "include_builtin_idf_component"
                        for alias in node.names
                    )
            elif isinstance(node, ast.Call):
                if (
                    isinstance(node.func, ast.Name)
                    and node.func.id == "include_builtin_idf_component"
                    and node.args
                    and isinstance(node.args[0], ast.Constant)
                    and isinstance(node.args[0].value, str)
                ):
                    enabled_components.add(node.args[0].value)

        self.assertTrue(imported)
        self.assertIn("esp_driver_rmt", enabled_components)


if __name__ == "__main__":
    unittest.main()
