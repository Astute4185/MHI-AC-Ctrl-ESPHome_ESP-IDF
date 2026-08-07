"""ESPHome codegen primitives for non-owning MHI transport injection."""

from dataclasses import dataclass

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.esp32 import get_esp32_variant

mhi_ns = cg.esphome_ns.namespace("mhi_ac_ctrl")

IMhiTransport = mhi_ns.class_("IMhiTransport")
MhiSplitTransport = mhi_ns.class_("MhiSplitTransport", IMhiTransport)
MhiDuplexTransportAdapter = mhi_ns.class_("MhiDuplexTransportAdapter", IMhiTransport)
MhiFastGpioRxDriver = mhi_ns.class_("MhiFastGpioRxDriver")
MhiFastGpioTxDriver = mhi_ns.class_("MhiFastGpioTxDriver")
MhiNullTxDriver = mhi_ns.class_("MhiNullTxDriver")
MhiExternalClockRxDriver = mhi_ns.class_("MhiExternalClockRxDriver")
MhiRmtSpiRxDriver = mhi_ns.class_("MhiRmtSpiRxDriver")
MhiRmtCsSpiTransport = mhi_ns.class_("MhiRmtCsSpiTransport")

CONF_PRIMARY_SPLIT_TRANSPORT_ID = "primary_split_transport_id"
CONF_PRIMARY_DUPLEX_ADAPTER_ID = "primary_duplex_adapter_id"
CONF_PRIMARY_FAST_GPIO_RX_ID = "primary_fast_gpio_rx_id"
CONF_PRIMARY_FAST_GPIO_TX_ID = "primary_fast_gpio_tx_id"
CONF_PRIMARY_NULL_TX_ID = "primary_null_tx_id"
CONF_PRIMARY_EXTERNAL_CLOCK_RX_ID = "primary_external_clock_rx_id"
CONF_PRIMARY_RMT_SPI_RX_ID = "primary_rmt_spi_rx_id"
CONF_PRIMARY_RMT_CS_SPI_ID = "primary_rmt_cs_spi_id"

CONF_RECOVERY_SPLIT_TRANSPORT_ID = "recovery_split_transport_id"
CONF_RECOVERY_FAST_GPIO_RX_ID = "recovery_fast_gpio_rx_id"
CONF_RECOVERY_FAST_GPIO_TX_ID = "recovery_fast_gpio_tx_id"
CONF_RECOVERY_NULL_TX_ID = "recovery_null_tx_id"


@dataclass(frozen=True)
class MhiTransportBuildInputs:
    frame_size: int
    sck_pin: int
    mosi_pin: int
    miso_pin: int
    frame_start_idle_ms: int
    rmt_spi_frame_gap_us: int
    tx_driver: str


def build_internal_transport_schema():
    """Generated IDs for codegen-owned transport and driver objects."""

    return {
        cv.GenerateID(CONF_PRIMARY_SPLIT_TRANSPORT_ID): cv.declare_id(MhiSplitTransport),
        cv.GenerateID(CONF_PRIMARY_DUPLEX_ADAPTER_ID): cv.declare_id(MhiDuplexTransportAdapter),
        cv.GenerateID(CONF_PRIMARY_FAST_GPIO_RX_ID): cv.declare_id(MhiFastGpioRxDriver),
        cv.GenerateID(CONF_PRIMARY_FAST_GPIO_TX_ID): cv.declare_id(MhiFastGpioTxDriver),
        cv.GenerateID(CONF_PRIMARY_NULL_TX_ID): cv.declare_id(MhiNullTxDriver),
        cv.GenerateID(CONF_PRIMARY_EXTERNAL_CLOCK_RX_ID): cv.declare_id(MhiExternalClockRxDriver),
        cv.GenerateID(CONF_PRIMARY_RMT_SPI_RX_ID): cv.declare_id(MhiRmtSpiRxDriver),
        cv.GenerateID(CONF_PRIMARY_RMT_CS_SPI_ID): cv.declare_id(MhiRmtCsSpiTransport),
        cv.GenerateID(CONF_RECOVERY_SPLIT_TRANSPORT_ID): cv.declare_id(MhiSplitTransport),
        cv.GenerateID(CONF_RECOVERY_FAST_GPIO_RX_ID): cv.declare_id(MhiFastGpioRxDriver),
        cv.GenerateID(CONF_RECOVERY_FAST_GPIO_TX_ID): cv.declare_id(MhiFastGpioTxDriver),
        cv.GenerateID(CONF_RECOVERY_NULL_TX_ID): cv.declare_id(MhiNullTxDriver),
    }


def _split_tx_supported() -> bool:
    return get_esp32_variant() in {"ESP32", "ESP32S3"}


def build_split_tx(config, inputs: MhiTransportBuildInputs, *, recovery: bool):
    """Construct the TX backend paired with one split RX transport."""

    if recovery:
        tx_id = CONF_RECOVERY_FAST_GPIO_TX_ID
        tx_driver = "fast_gpio_tx"
    else:
        tx_driver = inputs.tx_driver
        tx_id = CONF_PRIMARY_FAST_GPIO_TX_ID

    if tx_driver == "fast_gpio_tx" and _split_tx_supported():
        tx = cg.new_Pvariable(config[tx_id])
        cg.add(tx.set_frame_size_hint(inputs.frame_size))
        cg.add(tx.set_frame_start_idle_ms(inputs.frame_start_idle_ms))
        return tx, True

    null_tx_id = CONF_RECOVERY_NULL_TX_ID if recovery else CONF_PRIMARY_NULL_TX_ID
    null_tx = cg.new_Pvariable(config[null_tx_id])
    return null_tx, False


def configure_split_transport(
    transport, inputs: MhiTransportBuildInputs, rx, tx, *, classified_worker: bool, uses_bus_marker: bool
):
    cg.add(transport.set_pins(inputs.sck_pin, inputs.mosi_pin, inputs.miso_pin))
    cg.add(transport.bind(rx, tx, classified_worker, uses_bus_marker))
    return transport
