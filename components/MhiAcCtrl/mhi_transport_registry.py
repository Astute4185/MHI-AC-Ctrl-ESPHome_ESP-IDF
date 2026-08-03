"""Generic MHI transport registry and configuration normalization.

The registry deliberately has no ESPHome imports. Driver modules provide lazy
schema factories so transport metadata can be unit-tested without installing
ESPHome in the host-test environment.
"""

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

CONF_RX_DRIVER = "rx_driver"
CONF_FRAME_START_IDLE_MS = "frame_start_idle_ms"
CONF_RMT_SPI_FRAME_GAP_US = "rmt_spi_frame_gap_us"
CONF_FRAME_GAP_US = "frame_gap_us"

PLATFORM_ESP32 = "esp32"
FRAMEWORK_ESP_IDF = "esp-idf"
VARIANT_ESP32 = "ESP32"
VARIANT_ESP32C3 = "ESP32C3"
VARIANT_ESP32S3 = "ESP32S3"

DEFAULT_FRAME_START_IDLE_MS = 10
DEFAULT_RMT_SPI_FRAME_GAP_US = 1000

SchemaFactory = Callable[[], Any]


@dataclass(frozen=True)
class MhiTransportDefinition:
    """Compile-time metadata owned by one public RX driver."""

    name: str
    schema_factory: SchemaFactory
    compile_define: str
    supported_platforms: frozenset[str]
    supported_frameworks: frozenset[str]
    supported_variants: frozenset[str]
    required_idf_components: tuple[str, ...]
    uses_internal_fast_gpio_recovery: bool

    def supports_target(self, platform: str, framework: str, variant: str | None) -> bool:
        if platform not in self.supported_platforms:
            return False
        if framework not in self.supported_frameworks:
            return False
        return variant in self.supported_variants


@dataclass(frozen=True)
class MhiTransportTuning:
    """Current common tuning values after legacy/nested normalization."""

    frame_start_idle_ms: int
    rmt_spi_frame_gap_us: int


class TransportConfigurationError(ValueError):
    """Raised when transport configuration is structurally invalid."""


def _load_transport_definitions() -> dict[str, MhiTransportDefinition]:
    try:
        from .mhi_transport_external_clock import TRANSPORT_DEFINITION as external_clock
        from .mhi_transport_fast_gpio import TRANSPORT_DEFINITION as fast_gpio
        from .mhi_transport_rmt_cs_spi import TRANSPORT_DEFINITION as rmt_cs_spi
        from .mhi_transport_rmt_spi import TRANSPORT_DEFINITION as rmt_spi
    except ImportError:
        from mhi_transport_external_clock import TRANSPORT_DEFINITION as external_clock
        from mhi_transport_fast_gpio import TRANSPORT_DEFINITION as fast_gpio
        from mhi_transport_rmt_cs_spi import TRANSPORT_DEFINITION as rmt_cs_spi
        from mhi_transport_rmt_spi import TRANSPORT_DEFINITION as rmt_spi

    definitions = (fast_gpio, external_clock, rmt_spi, rmt_cs_spi)
    registry: dict[str, MhiTransportDefinition] = {}

    for definition in definitions:
        if definition.name in registry:
            raise RuntimeError(f"Duplicate MHI transport definition: {definition.name}")
        registry[definition.name] = definition

    return registry


TRANSPORT_DEFINITIONS = _load_transport_definitions()
TRANSPORT_SUBSECTION_KEYS = frozenset(TRANSPORT_DEFINITIONS)


def build_transport_schemas() -> dict[str, Any]:
    """Build ESPHome schemas lazily from driver-owned schema factories."""

    return {name: definition.schema_factory() for name, definition in TRANSPORT_DEFINITIONS.items()}


def get_transport_definition(driver_name: str) -> MhiTransportDefinition:
    try:
        return TRANSPORT_DEFINITIONS[driver_name]
    except KeyError as err:
        raise TransportConfigurationError(f"Unknown rx_driver: {driver_name}") from err


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
    get_transport_definition(selected)

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


def validate_selected_transport_target(
    config: Mapping[str, Any],
    *,
    platform: str,
    framework: str,
    variant: str | None,
) -> MhiTransportDefinition:
    """Validate the selected transport against the resolved ESPHome target."""

    definition = get_transport_definition(_selected_driver(config))

    if platform not in definition.supported_platforms:
        raise TransportConfigurationError(f"rx_driver '{definition.name}' is only supported on ESP32")

    if framework not in definition.supported_frameworks:
        raise TransportConfigurationError(f"rx_driver '{definition.name}' requires the ESP-IDF framework")

    if variant not in definition.supported_variants:
        supported = ", ".join(sorted(definition.supported_variants))
        raise TransportConfigurationError(
            f"rx_driver '{definition.name}' is not supported on {variant or 'this ESP32 variant'}; "
            f"supported variants: {supported}"
        )

    return definition


def resolve_legacy_build_idf_components(
    *,
    platform: str,
    framework: str,
    variant: str | None,
) -> tuple[str, ...]:
    """Resolve dependencies required by the current target-gated C++ manager.

    Phase 2 still compiles every transport enabled by the manager's target
    macros. Phase 3 will switch this to the selected primary plus internal
    FastGPIO recovery only.
    """

    components: set[str] = set()
    for definition in TRANSPORT_DEFINITIONS.values():
        if definition.supports_target(platform, framework, variant):
            components.update(definition.required_idf_components)
    return tuple(sorted(components))


def resolve_selected_compile_defines(config: Mapping[str, Any]) -> tuple[str, ...]:
    """Return the future Phase 3 compile plan for the selected transport."""

    selected = get_transport_definition(_selected_driver(config))
    defines = {selected.compile_define}

    if selected.uses_internal_fast_gpio_recovery:
        defines.add(get_transport_definition("fast_gpio_rx").compile_define)
        defines.add("MHI_INTERNAL_FAST_GPIO_RECOVERY")

    return tuple(sorted(defines))


def resolve_selected_idf_components(config: Mapping[str, Any]) -> tuple[str, ...]:
    """Return dependencies for the future selected-only transport build."""

    selected = get_transport_definition(_selected_driver(config))
    components = set(selected.required_idf_components)

    if selected.uses_internal_fast_gpio_recovery:
        components.update(get_transport_definition("fast_gpio_rx").required_idf_components)

    return tuple(sorted(components))


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
