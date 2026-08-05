# Developing a new transport

A new transport should be implemented as a self-contained module and connected to the existing common interfaces. Do not add a driver name switch or concrete driver member to `MhiAcCtrl` or `MhiTransportManager`.

### 1. Choose the transport shape

Use a **split RX driver** when the new hardware only captures SCK/MOSI and can reuse an existing TX backend. Implement `IMhiRxDriver`, then let `MhiSplitTransport` provide the common `IMhiTransport` strategy.

Use an **integrated duplex backend** when RX and TX share one peripheral, transaction boundary, task, or buffer set. Implement `IMhiDuplexTransport`, then expose it through `MhiDuplexTransportAdapter`.

A split RX driver is the smallest useful example because it can reuse `fast_gpio_tx` and the existing completion, Active Mode, and command lifecycle.

### 2. Add the C++ backend

Create consistently prefixed files in the flat component directory, for example:

```text
components/MhiAcCtrl/mhi_example_rx_driver.h
components/MhiAcCtrl/mhi_example_rx_driver.cpp
```

The header implements the split RX contract:

```cpp
#pragma once

#include <cstddef>
#include <cstdint>

#include "mhi_rx_driver.h"

namespace esphome {
namespace mhi_ac_ctrl {

class MhiExampleRxDriver final : public IMhiRxDriver {
 public:
  void set_frame_size_hint(int frame_size) {
    frame_size_ = frame_size == 33 ? 33U : 20U;
  }

  void set_sample_depth(int sample_depth) {
    sample_depth_ = static_cast<uint8_t>(sample_depth);
  }

  bool setup(const MhiTransportPins& pins) override;
  void loop() override;
  void shutdown() override;
  std::size_t read(uint8_t* dst, std::size_t max_len) override;

  // Required when split TX is armed from a detected frame boundary.
  MhiBusMarker bus_marker() const override;

  const char* name() const override {
    return "example_rx";
  }

  bool ready() const override {
    return ready_;
  }

 private:
  MhiTransportPins pins_{};
  uint8_t frame_size_{20U};
  uint8_t sample_depth_{4U};
  bool ready_{false};
};

}  // namespace mhi_ac_ctrl
}  // namespace esphome
```

The complete implementation translation unit must be guarded so an unselected driver contributes no compiled implementation:

```cpp
#include "esphome/core/defines.h"

#ifdef MHI_USE_TRANSPORT_EXAMPLE_RX

#include "mhi_example_rx_driver.h"

namespace esphome {
namespace mhi_ac_ctrl {

// setup(), loop(), shutdown(), read(), and bus_marker() implementation.

}  // namespace mhi_ac_ctrl
}  // namespace esphome

#endif  // MHI_USE_TRANSPORT_EXAMPLE_RX
```

The backend must:

- own its peripheral handles, tasks, queues, DMA/FIFO buffers, and target workarounds;
- return copied, complete, bounded RX chunks without exposing peripheral-owned memory after `read()` returns;
- keep ISR and critical-section work bounded;
- expose a monotonic bus marker when paired TX depends on a frame boundary;
- make `shutdown()` safe after partial setup failure;
- report readiness and hardware errors through its transport wrapper and diagnostics;
- avoid frame decode, command construction, ESPHome publication, or Home Assistant state.

### 3. Add the driver-owned Python definition

Create `components/MhiAcCtrl/mhi_transport_example.py`. The module owns schema, target policy, dependencies, compile definition, and construction:

```python
"""Example split RX transport schema, codegen, and metadata."""

from .mhi_transport_registry import (
    FRAMEWORK_ESP_IDF,
    PLATFORM_ESP32,
    VARIANT_ESP32S3,
    MhiTransportDefinition,
)

CONF_SAMPLE_DEPTH = "sample_depth"


def build_config_schema():
    import esphome.config_validation as cv

    return cv.Schema(
        {
            cv.Optional(CONF_SAMPLE_DEPTH, default=4): cv.int_range(min=2, max=16),
        }
    )


async def build_transport(config, inputs, *, recovery=False):
    if recovery:
        raise ValueError("example_rx cannot be constructed as the recovery transport")

    import esphome.codegen as cg

    from .mhi_transport_codegen import (
        CONF_PRIMARY_EXAMPLE_RX_ID,
        CONF_PRIMARY_SPLIT_TRANSPORT_ID,
        build_split_tx,
        configure_split_transport,
    )

    driver_config = config.get("example_rx", {})
    sample_depth = driver_config.get(CONF_SAMPLE_DEPTH, 4)

    rx = cg.new_Pvariable(config[CONF_PRIMARY_EXAMPLE_RX_ID])
    cg.add(rx.set_frame_size_hint(inputs.frame_size))
    cg.add(rx.set_sample_depth(sample_depth))

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
    name="example_rx",
    schema_factory=build_config_schema,
    builder=build_transport,
    compile_define="MHI_USE_TRANSPORT_EXAMPLE_RX",
    supported_platforms=frozenset({PLATFORM_ESP32}),
    supported_frameworks=frozenset({FRAMEWORK_ESP_IDF}),
    supported_variants=frozenset({VARIANT_ESP32S3}),
    required_idf_components=(),  # List any required built-in IDF components.
    uses_internal_fast_gpio_recovery=True,
)
```

The corresponding YAML is driver-local:

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: example_rx

  example_rx:
    sample_depth: 4
```

### 4. Add the small registration and codegen wiring

The new module requires these wiring changes:

1. In `mhi_transport_registry.py`, import its `TRANSPORT_DEFINITION` and add it to the definitions tuple.
2. In `mhi_transport_codegen.py`, declare the generated C++ class, generated ID constant, and `cv.declare_id(...)` entry.
3. In `driver_selection.py`, add a default TX mapping for a split driver, or add the name to `DUPLEX_RX_DRIVERS` for an integrated backend.
4. Add the new whole-translation-unit compile guard to `test_transport_compile_selection.py`.

These are declarative construction changes. The new driver must not require edits to controller behaviour, transport-manager recovery logic, protocol decoding, command confirmation, or ESPHome entity code.

### 5. Declare capabilities accurately

For a split driver, pass accurate values to `configure_split_transport(...)`:

- `classified_worker=True` only when `read()` is safe from the command-worker context;
- `uses_bus_marker=True` only when `bus_marker()` provides the boundary required to arm paired TX.

For an integrated duplex backend, implement real TX completion after the bus transaction has finished. Queue acceptance is not TX completion. The backend must also keep RX running when Active Mode disables MISO participation and must clear staged TX so it cannot replay when Active Mode is restored.

### 6. Add validation before advertising support

At minimum, add:

- Python tests for registration, nested schema, target rejection, dependencies, compile definitions, recovery policy, and TX resolution;
- host C++ tests for queue, completion, Active Mode, error, and health behaviour that can be isolated from hardware;
- a representative ESPHome compile fixture for every materially different supported target;
- source-manifest and whole-unit guard coverage;
- hardware startup, RX, TX, command-confirmation, recovery, and soak evidence.

A hardware result should show the selected transport, frame size, worker setting, valid/invalid frame counters, checksum and synchronisation health, queue high-water marks, TX completions/failures, command confirmations/timeouts, loop timing, and final recovery/safe-mode state.

### 7. Review boundary

A transport contribution is correctly isolated when reviewers can remove its Python module and guarded C++ implementation without changing the protocol, state, command, entity, or manager layers. If adding the backend requires a concrete driver conditional outside registration/codegen or its own implementation, the common contract should be extended generically instead of adding a one-off exception.


## Documentation requirement

Every new public driver must add a page in this directory containing:

- the exact `rx_driver` or `tx_driver` value;
- supported platform, framework, and ESP32 variants;
- whether TX is integrated or automatically resolved;
- whether command-worker classified RX is supported;
- whether internal FastGPIO recovery is included;
- the smallest working YAML example;
- every nested tunable, including type, accepted range, default, and operational effect;
- invalid combinations and important target-specific behaviour;
- the hardware evidence used to claim runtime support.

Add the driver to the summary table in [`README.md`](README.md). Do not place the complete tunable reference back into the project root README.

## Related pages

- [Driver index](README.md)
- [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md)
- [`../../CONTRIBUTING.md`](../../CONTRIBUTING.md)
- [`../../DIAGNOSTICS.md`](../../DIAGNOSTICS.md)
