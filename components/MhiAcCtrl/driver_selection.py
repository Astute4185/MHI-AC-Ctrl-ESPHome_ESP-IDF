"""Driver selection rules shared by ESPHome configuration and tests."""

RX_DRIVERS = (
    "fast_gpio_rx",
    "external_clock_rx",
    "rmt_spi_rx",
    "rmt_cs_spi",
)

TX_DRIVERS = (
    "fast_gpio_tx",
    "none",
)

DUPLEX_RX_DRIVERS = frozenset({"rmt_cs_spi"})

DEFAULT_TX_BY_RX = {
    "fast_gpio_rx": "fast_gpio_tx",
    "external_clock_rx": "fast_gpio_tx",
    "rmt_spi_rx": "fast_gpio_tx",
}


class DriverSelectionError(ValueError):
    """Raised when an RX/TX driver combination is structurally invalid."""


def resolve_tx_driver(rx_driver: str, explicit_tx_driver: str | None = None) -> str:
    """Resolve the effective TX driver from the selected RX driver.

    Split RX drivers default to FastGPIO TX while preserving an explicit TX
    override. Full-duplex RX selections own both directions and reject a TX
    override.
    """

    if rx_driver in DUPLEX_RX_DRIVERS:
        if explicit_tx_driver is not None:
            raise DriverSelectionError(f"rx_driver: {rx_driver} owns RX and TX; remove the tx_driver override")
        return rx_driver

    if explicit_tx_driver is not None:
        return explicit_tx_driver

    try:
        return DEFAULT_TX_BY_RX[rx_driver]
    except KeyError as err:
        raise DriverSelectionError(f"No default TX driver is defined for rx_driver: {rx_driver}") from err
