"""External-clock transport schema and compile-time metadata."""

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


def build_config_schema():
    import esphome.config_validation as cv

    # The current implementation has no public driver-specific tuning. Keeping
    # an explicit schema gives the driver a stable namespace for future options.
    return cv.Schema({})


TRANSPORT_DEFINITION = MhiTransportDefinition(
    name="external_clock_rx",
    schema_factory=build_config_schema,
    compile_define="MHI_USE_TRANSPORT_EXTERNAL_CLOCK",
    supported_platforms=frozenset({PLATFORM_ESP32}),
    supported_frameworks=frozenset({FRAMEWORK_ESP_IDF}),
    supported_variants=frozenset({VARIANT_ESP32, VARIANT_ESP32S3}),
    required_idf_components=(),
    uses_internal_fast_gpio_recovery=True,
)
