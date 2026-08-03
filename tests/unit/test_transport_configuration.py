#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "components" / "MhiAcCtrl"))

from mhi_transport_registry import (  # noqa: E402
    DEFAULT_FRAME_START_IDLE_MS,
    DEFAULT_RMT_SPI_FRAME_GAP_US,
    FRAMEWORK_ESP_IDF,
    PLATFORM_ESP32,
    TRANSPORT_DEFINITIONS,
    VARIANT_ESP32,
    VARIANT_ESP32C3,
    VARIANT_ESP32S3,
    TransportConfigurationError,
    resolve_legacy_build_idf_components,
    resolve_selected_compile_defines,
    resolve_selected_idf_components,
    resolve_transport_tuning,
    validate_driver_subsections,
    validate_selected_transport_target,
)


class TransportConfigurationTests(unittest.TestCase):
    def test_registry_contains_all_current_rx_drivers(self):
        self.assertEqual(
            set(TRANSPORT_DEFINITIONS),
            {"fast_gpio_rx", "external_clock_rx", "rmt_spi_rx", "rmt_cs_spi"},
        )

    def test_each_driver_owns_schema_and_compile_metadata(self):
        for name, definition in TRANSPORT_DEFINITIONS.items():
            with self.subTest(driver=name):
                self.assertEqual(definition.name, name)
                self.assertTrue(callable(definition.schema_factory))
                self.assertTrue(definition.compile_define.startswith("MHI_USE_TRANSPORT_"))
                self.assertEqual(definition.supported_platforms, frozenset({PLATFORM_ESP32}))
                self.assertEqual(definition.supported_frameworks, frozenset({FRAMEWORK_ESP_IDF}))

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

    def test_target_support_matches_current_native_drivers(self):
        expected = {
            "fast_gpio_rx": {VARIANT_ESP32, VARIANT_ESP32C3, VARIANT_ESP32S3},
            "external_clock_rx": {VARIANT_ESP32, VARIANT_ESP32S3},
            "rmt_spi_rx": {VARIANT_ESP32S3},
            "rmt_cs_spi": {VARIANT_ESP32, VARIANT_ESP32S3},
        }

        for name, variants in expected.items():
            with self.subTest(driver=name):
                self.assertEqual(TRANSPORT_DEFINITIONS[name].supported_variants, frozenset(variants))

    def test_supported_target_is_accepted(self):
        definition = validate_selected_transport_target(
            {"rx_driver": "rmt_spi_rx"},
            platform=PLATFORM_ESP32,
            framework=FRAMEWORK_ESP_IDF,
            variant=VARIANT_ESP32S3,
        )

        self.assertEqual(definition.name, "rmt_spi_rx")

    def test_unsupported_variant_is_rejected(self):
        with self.assertRaisesRegex(TransportConfigurationError, "rmt_spi_rx.*ESP32C3"):
            validate_selected_transport_target(
                {"rx_driver": "rmt_spi_rx"},
                platform=PLATFORM_ESP32,
                framework=FRAMEWORK_ESP_IDF,
                variant=VARIANT_ESP32C3,
            )

    def test_non_idf_framework_is_rejected(self):
        with self.assertRaisesRegex(TransportConfigurationError, "requires the ESP-IDF framework"):
            validate_selected_transport_target(
                {"rx_driver": "fast_gpio_rx"},
                platform=PLATFORM_ESP32,
                framework="arduino",
                variant=VARIANT_ESP32,
            )

    def test_hardware_transports_request_internal_fast_gpio_recovery(self):
        self.assertFalse(TRANSPORT_DEFINITIONS["fast_gpio_rx"].uses_internal_fast_gpio_recovery)
        self.assertTrue(TRANSPORT_DEFINITIONS["external_clock_rx"].uses_internal_fast_gpio_recovery)
        self.assertTrue(TRANSPORT_DEFINITIONS["rmt_spi_rx"].uses_internal_fast_gpio_recovery)
        self.assertTrue(TRANSPORT_DEFINITIONS["rmt_cs_spi"].uses_internal_fast_gpio_recovery)

    def test_future_compile_plan_adds_fast_gpio_recovery(self):
        self.assertEqual(
            set(resolve_selected_compile_defines({"rx_driver": "rmt_spi_rx"})),
            {
                "MHI_INTERNAL_FAST_GPIO_RECOVERY",
                "MHI_USE_TRANSPORT_FAST_GPIO",
                "MHI_USE_TRANSPORT_RMT_SPI",
            },
        )

    def test_fast_gpio_compile_plan_has_no_duplicate_recovery(self):
        self.assertEqual(
            resolve_selected_compile_defines({"rx_driver": "fast_gpio_rx"}),
            ("MHI_USE_TRANSPORT_FAST_GPIO",),
        )

    def test_rmt_dependency_is_driver_owned(self):
        self.assertEqual(resolve_selected_idf_components({"rx_driver": "rmt_spi_rx"}), ("esp_driver_rmt",))
        self.assertEqual(resolve_selected_idf_components({"rx_driver": "rmt_cs_spi"}), ("esp_driver_rmt",))
        self.assertEqual(resolve_selected_idf_components({"rx_driver": "fast_gpio_rx"}), ())

    def test_legacy_manager_dependencies_follow_target_enabled_drivers(self):
        self.assertEqual(
            resolve_legacy_build_idf_components(
                platform=PLATFORM_ESP32,
                framework=FRAMEWORK_ESP_IDF,
                variant=VARIANT_ESP32S3,
            ),
            ("esp_driver_rmt",),
        )
        self.assertEqual(
            resolve_legacy_build_idf_components(
                platform=PLATFORM_ESP32,
                framework=FRAMEWORK_ESP_IDF,
                variant=VARIANT_ESP32C3,
            ),
            (),
        )


if __name__ == "__main__":
    unittest.main()
