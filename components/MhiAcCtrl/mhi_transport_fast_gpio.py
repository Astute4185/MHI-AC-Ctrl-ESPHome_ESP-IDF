"""FastGPIO transport schema, codegen, and compile-time metadata."""

try:
    from .mhi_transport_registry import (
        FRAMEWORK_ESP_IDF,
        PLATFORM_ESP32,
        WIFI_ESP32_VARIANTS,
        MhiTransportDefinition,
    )
except ImportError:
    from mhi_transport_registry import (
        FRAMEWORK_ESP_IDF,
        PLATFORM_ESP32,
        WIFI_ESP32_VARIANTS,
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


async def build_transport(config, inputs, *, recovery=False):
    import esphome.codegen as cg

    from .mhi_transport_codegen import (
        CONF_PRIMARY_FAST_GPIO_RX_ID,
        CONF_PRIMARY_SPLIT_TRANSPORT_ID,
        CONF_RECOVERY_FAST_GPIO_RX_ID,
        CONF_RECOVERY_SPLIT_TRANSPORT_ID,
        build_split_tx,
        configure_split_transport,
    )

    rx_id = CONF_RECOVERY_FAST_GPIO_RX_ID if recovery else CONF_PRIMARY_FAST_GPIO_RX_ID
    split_id = CONF_RECOVERY_SPLIT_TRANSPORT_ID if recovery else CONF_PRIMARY_SPLIT_TRANSPORT_ID

    rx = cg.new_Pvariable(config[rx_id])
    cg.add(rx.set_frame_size_hint(inputs.frame_size))
    cg.add(rx.set_frame_start_idle_ms(inputs.frame_start_idle_ms))

    tx, uses_bus_marker = build_split_tx(config, inputs, recovery=recovery)
    transport = cg.new_Pvariable(config[split_id])
    return configure_split_transport(
        transport,
        inputs,
        rx,
        tx,
        classified_worker=False,
        uses_bus_marker=uses_bus_marker,
    )


TRANSPORT_DEFINITION = MhiTransportDefinition(
    name="fast_gpio_rx",
    schema_factory=build_config_schema,
    builder=build_transport,
    compile_define="MHI_USE_TRANSPORT_FAST_GPIO",
    supported_platforms=frozenset({PLATFORM_ESP32}),
    supported_frameworks=frozenset({FRAMEWORK_ESP_IDF}),
    supported_variants=WIFI_ESP32_VARIANTS,
    required_idf_components=(),
    uses_internal_fast_gpio_recovery=False,
)
