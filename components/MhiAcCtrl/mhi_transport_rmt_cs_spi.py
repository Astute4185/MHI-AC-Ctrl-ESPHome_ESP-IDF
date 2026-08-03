"""Integrated RMT-CS SPI transport schema and compile-time metadata."""

try:
    from .mhi_transport_registry import (
        FRAMEWORK_ESP_IDF,
        PLATFORM_ESP32,
        VARIANT_ESP32,
        VARIANT_ESP32S3,
        MhiTransportDefinition,
    )
except ImportError:
    from mhi_transport_registry import (
        FRAMEWORK_ESP_IDF,
        PLATFORM_ESP32,
        VARIANT_ESP32,
        VARIANT_ESP32S3,
        MhiTransportDefinition,
    )

CONF_FRAME_GAP_US = "frame_gap_us"


def build_config_schema():
    import esphome.config_validation as cv

    return cv.Schema(
        {
            cv.Optional(CONF_FRAME_GAP_US): cv.int_range(min=500, max=5000),
        }
    )


TRANSPORT_DEFINITION = MhiTransportDefinition(
    name="rmt_cs_spi",
    schema_factory=build_config_schema,
    compile_define="MHI_USE_TRANSPORT_RMT_CS_SPI",
    supported_platforms=frozenset({PLATFORM_ESP32}),
    supported_frameworks=frozenset({FRAMEWORK_ESP_IDF}),
    supported_variants=frozenset({VARIANT_ESP32, VARIANT_ESP32S3}),
    required_idf_components=("esp_driver_rmt",),
    uses_internal_fast_gpio_recovery=True,
)
