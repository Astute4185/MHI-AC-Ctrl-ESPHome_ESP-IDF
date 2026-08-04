"""External-clock transport schema, codegen, and compile-time metadata."""

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

    return cv.Schema({})


async def build_transport(config, inputs, *, recovery=False):
    if recovery:
        raise ValueError("external_clock_rx cannot be constructed as the recovery transport")

    import esphome.codegen as cg

    from .mhi_transport_codegen import (
        CONF_PRIMARY_EXTERNAL_CLOCK_RX_ID,
        CONF_PRIMARY_SPLIT_TRANSPORT_ID,
        build_split_tx,
        configure_split_transport,
    )

    rx = cg.new_Pvariable(config[CONF_PRIMARY_EXTERNAL_CLOCK_RX_ID])
    cg.add(rx.set_frame_size_hint(inputs.frame_size))

    tx, uses_bus_marker = build_split_tx(config, inputs, recovery=False)
    transport = cg.new_Pvariable(config[CONF_PRIMARY_SPLIT_TRANSPORT_ID])
    return configure_split_transport(
        transport,
        inputs,
        rx,
        tx,
        classified_worker=True,
        uses_bus_marker=uses_bus_marker,
    )


TRANSPORT_DEFINITION = MhiTransportDefinition(
    name="external_clock_rx",
    schema_factory=build_config_schema,
    builder=build_transport,
    compile_define="MHI_USE_TRANSPORT_EXTERNAL_CLOCK",
    supported_platforms=frozenset({PLATFORM_ESP32}),
    supported_frameworks=frozenset({FRAMEWORK_ESP_IDF}),
    supported_variants=frozenset({VARIANT_ESP32, VARIANT_ESP32S3}),
    required_idf_components=(),
    uses_internal_fast_gpio_recovery=True,
)
