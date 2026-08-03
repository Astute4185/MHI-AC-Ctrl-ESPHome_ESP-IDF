"""Transport metadata and configuration normalization.

This module deliberately has no ESPHome imports so the selection and
compatibility rules can be exercised by the host unit-test suite.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

CONF_RX_DRIVER = "rx_driver"
CONF_FRAME_START_IDLE_MS = "frame_start_idle_ms"
CONF_RMT_SPI_FRAME_GAP_US = "rmt_spi_frame_gap_us"
CONF_FRAME_GAP_US = "frame_gap_us"

DEFAULT_FRAME_START_IDLE_MS = 10
DEFAULT_RMT_SPI_FRAME_GAP_US = 1000


@dataclass(frozen=True)
class MhiTransportDefinition:
    """Compile-time metadata for one public RX driver selection."""

    name: str
    compile_define: str
    uses_internal_fast_gpio_recovery: bool


@dataclass(frozen=True)
class MhiTransportTuning:
    """Current common tuning values after legacy/nested normalization."""

    frame_start_idle_ms: int
    rmt_spi_frame_gap_us: int


TRANSPORT_DEFINITIONS = {
    "fast_gpio_rx": MhiTransportDefinition(
        name="fast_gpio_rx",
        compile_define="MHI_USE_TRANSPORT_FAST_GPIO",
        uses_internal_fast_gpio_recovery=False,
    ),
    "external_clock_rx": MhiTransportDefinition(
        name="external_clock_rx",
        compile_define="MHI_USE_TRANSPORT_EXTERNAL_CLOCK",
        uses_internal_fast_gpio_recovery=True,
    ),
    "rmt_spi_rx": MhiTransportDefinition(
        name="rmt_spi_rx",
        compile_define="MHI_USE_TRANSPORT_RMT_SPI",
        uses_internal_fast_gpio_recovery=True,
    ),
    "rmt_cs_spi": MhiTransportDefinition(
        name="rmt_cs_spi",
        compile_define="MHI_USE_TRANSPORT_RMT_CS_SPI",
        uses_internal_fast_gpio_recovery=True,
    ),
}

TRANSPORT_SUBSECTION_KEYS = frozenset(TRANSPORT_DEFINITIONS)


class TransportConfigurationError(ValueError):
    """Raised when driver-specific YAML is structurally inconsistent."""


def _selected_driver(config: Mapping[str, Any]) -> str:
    return str(config.get(CONF_RX_DRIVER, "fast_gpio_rx"))


def _driver_block(config: Mapping[str, Any], driver_name: str) -> Mapping[str, Any]:
    value = config.get(driver_name, {})
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise TransportConfigurationError(f"The '{driver_name}' configuration block must be a mapping")
    return value


def validate_driver_subsections(config: Mapping[str, Any]) -> None:
    """Validate that nested driver options match the selected RX driver."""

    selected = _selected_driver(config)
    if selected not in TRANSPORT_DEFINITIONS:
        raise TransportConfigurationError(f"Unknown rx_driver: {selected}")

    for driver_name in TRANSPORT_SUBSECTION_KEYS:
        if driver_name in config and driver_name != selected:
            raise TransportConfigurationError(
                f"The '{driver_name}' configuration block is present, but rx_driver is set to '{selected}'"
            )

    driver_config = _driver_block(config, selected)

    if selected == "fast_gpio_rx" and CONF_FRAME_START_IDLE_MS in config and CONF_FRAME_START_IDLE_MS in driver_config:
        raise TransportConfigurationError(
            "Configure frame_start_idle_ms either at MhiAcCtrl level or under fast_gpio_rx, not both"
        )

    if (
        selected in {"rmt_spi_rx", "rmt_cs_spi"}
        and CONF_RMT_SPI_FRAME_GAP_US in config
        and CONF_FRAME_GAP_US in driver_config
    ):
        raise TransportConfigurationError(
            f"Configure the frame gap either as {CONF_RMT_SPI_FRAME_GAP_US} or under {selected}, not both"
        )


def resolve_transport_tuning(config: Mapping[str, Any]) -> MhiTransportTuning:
    """Resolve current C++ tuning inputs without changing runtime behaviour."""

    validate_driver_subsections(config)

    selected = _selected_driver(config)
    driver_config = _driver_block(config, selected)

    frame_start_idle_ms = int(config.get(CONF_FRAME_START_IDLE_MS, DEFAULT_FRAME_START_IDLE_MS))
    rmt_spi_frame_gap_us = int(config.get(CONF_RMT_SPI_FRAME_GAP_US, DEFAULT_RMT_SPI_FRAME_GAP_US))

    if selected == "fast_gpio_rx":
        frame_start_idle_ms = int(driver_config.get(CONF_FRAME_START_IDLE_MS, frame_start_idle_ms))
    elif selected in {"rmt_spi_rx", "rmt_cs_spi"}:
        rmt_spi_frame_gap_us = int(driver_config.get(CONF_FRAME_GAP_US, rmt_spi_frame_gap_us))

    return MhiTransportTuning(
        frame_start_idle_ms=frame_start_idle_ms,
        rmt_spi_frame_gap_us=rmt_spi_frame_gap_us,
    )
