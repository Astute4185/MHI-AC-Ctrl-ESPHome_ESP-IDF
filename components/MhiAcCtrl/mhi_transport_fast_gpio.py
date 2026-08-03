"""FastGPIO transport schema and compile-time metadata."""

try:
    from .mhi_transport_registry import (
        FRAMEWORK_ESP_IDF,
        PLATFORM_ESP32,
        VARIANT_ESP32,
        VARIANT_ESP32C3,
        VARIANT_ESP32S3,
        MhiTransportDefinition,
    )
except ImportError:
    from mhi_transport_registry import (
        FRAMEWORK_ESP_IDF,
        PLATFORM_ESP32,
        VARIANT_ESP32,
        VARIANT_ESP32C3,
        VARIANT_ESP32S3,
        MhiTransportDefinition,
    )

CONF_FRAME_START_IDLE_MS = "frame_start_idle_ms"


def build_config_schema():
    import esphome.config_validation as cv

    return cv.Schema(
        {
            cv.Optional(CONF_FRAME_START_IDLE_MS): cv.int_range(min=1, max=50),
        }
    )


TRANSPORT_DEFINITION = MhiTransportDefinition(
    name="fast_gpio_rx",
    schema_factory=build_config_schema,
    compile_define="MHI_USE_TRANSPORT_FAST_GPIO",
    supported_platforms=frozenset({PLATFORM_ESP32}),
    supported_frameworks=frozenset({FRAMEWORK_ESP_IDF}),
    supported_variants=frozenset({VARIANT_ESP32, VARIANT_ESP32C3, VARIANT_ESP32S3}),
    required_idf_components=(),
    uses_internal_fast_gpio_recovery=False,
)
