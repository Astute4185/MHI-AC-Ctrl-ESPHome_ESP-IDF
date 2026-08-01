# Driver Selection Guide

This guide explains the available MHI transport drivers, the supported combinations, and the evidence required before a hardware result is treated as validated.

The short operational recommendation remains in [`README.md`](README.md). Runtime counters and soak-test interpretation are documented in [`DIAGNOSTICS.md`](DIAGNOSTICS.md).

## Current position

The transport architecture is now largely complete:

- `fast_gpio_rx` + `fast_gpio_tx` remains the conservative default and fallback.
- `external_clock_rx` + `fast_gpio_tx` remains a validated split path for the original ESP32.
- `rmt_spi_rx` + `fast_gpio_tx` remains the validated ESP32-S3 split hardware path.
- `rmt_cs_spi` is the consolidated FIFO-backed full-duplex path for the original ESP32 and ESP32-S3.
- The temporary `rmt_cs_spi_nodma` label has been removed.
- `rmt_cs_spi` always uses the SPI CPU FIFO with `SPI_DMA_DISABLED`.
- Command staging, real TX completion, semantic confirmation, retries, and latest-intent supersession are shared above the transport layer.

The default configuration is equivalent to:

```yaml
MhiAcCtrl:
  frame_size: 20
  fan_profile: four_speed
  rx_driver: fast_gpio_rx
  command_worker: false
```

`fast_gpio_rx` automatically resolves to `fast_gpio_tx`.

## Supported selections

| `rx_driver` | Effective TX | Target availability | Runtime position |
|---|---|---|---|
| `fast_gpio_rx` | `fast_gpio_tx` | Original ESP32 and ESP32-S3 runtime; ESP32-C3 compile fixture | Stable baseline on validated runtime targets |
| `external_clock_rx` | `fast_gpio_tx` | Original ESP32 and ESP32-S3 | Hardware-validated on the original ESP32 |
| `rmt_spi_rx` | `fast_gpio_tx` | ESP32-S3 only | Hardware-validated split hardware RX path |
| `rmt_cs_spi` | Integrated `rmt_cs_spi` TX | Original ESP32 and ESP32-S3 | Hardware-validated FIFO full-duplex path |

ESP32-C3 currently has compile coverage through the FastGPIO configuration. Runtime RX/TX operation has not yet been validated, and the current target-specific transport build does not establish a supported control path on C3.

## Selection rules

### Split drivers

The normal split configuration only needs `rx_driver`:

```yaml
MhiAcCtrl:
  rx_driver: rmt_spi_rx
```

The component resolves the effective TX driver to `fast_gpio_tx`.

The explicit equivalent remains valid:

```yaml
MhiAcCtrl:
  rx_driver: rmt_spi_rx
  tx_driver: fast_gpio_tx
```

The same rule applies to `fast_gpio_rx` and `external_clock_rx`.

### RX-only diagnostics

The queue-backed split drivers can be operated without TX:

```yaml
MhiAcCtrl:
  rx_driver: rmt_spi_rx
  tx_driver: none
```

or:

```yaml
MhiAcCtrl:
  rx_driver: external_clock_rx
  tx_driver: none
```

Use this only to isolate RX behaviour. Climate commands, vane commands, 3D Auto changes, and opdata requests requiring TX will not work.

Do not use `tx_driver: none` as a normal Home Assistant configuration.

### Full-duplex `rmt_cs_spi`

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
```

`rmt_cs_spi` owns:

- RMT frame-boundary detection;
- SPI2 slave configuration;
- SCK, MOSI, and MISO;
- RX and TX buffers;
- transaction queue/result calls;
- the dedicated SPI owner task;
- command TX completion reporting.

It therefore rejects a separate TX override.

Invalid:

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
  tx_driver: fast_gpio_tx
```

Also invalid:

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
  tx_driver: none
```

No physical CS pin is required. RMT detects the inter-frame SCK idle gap and drives the SPI peripheral's internal CS signal.

The implementation always uses:

```text
SPI_DMA_DISABLED
```

The MHI frame sizes are 20 or 33 bytes. The rounded 36-byte working transaction fits within the SPI slave FIFO capacity, so DMA does not provide a frame-capacity benefit for this transport.

### Command worker

All transports use the same command contract. The selected transport still owns real-time bus activity.

```yaml
MhiAcCtrl:
  command_worker: false
```

uses the main-loop command coordinator.

```yaml
MhiAcCtrl:
  command_worker: true
```

enables the event-driven command worker.

When enabled:

- command envelopes are prepared outside the transport's real-time path;
- a command becomes pending confirmation only after the transport reports actual TX completion;
- queue-backed RX drivers are drained, synchronised, classified, and decoded in the worker;
- the main loop applies bounded decoded snapshots and remains the only context that publishes ESPHome state;
- `fast_gpio_rx` remains main-loop driven because its synchronous edge sampling cannot be moved safely into the classified worker.

Queue-backed classified-RX support applies to:

- `external_clock_rx`;
- `rmt_spi_rx`;
- `rmt_cs_spi`.

`command_worker: false` remains the fallback for diagnosis and compatibility comparison.

## Driver details

### `fast_gpio_rx`

Software GPIO receive path.

Strengths:

- conservative default;
- widest fallback coverage;
- straightforward bring-up and comparison path;
- paired with the established `fast_gpio_tx` implementation.

Trade-offs:

- CPU-intensive synchronous sampling;
- sensitive to scheduler, logging, Wi-Fi, API, and publication load;
- remains in the ESPHome main loop even when `command_worker: true`;
- can contribute to long loop sections.

Use it when:

- bringing up new hardware;
- testing ESP32-C3;
- comparing a hardware-assisted backend against the baseline;
- isolating whether a failure is transport-specific.

### `external_clock_rx`

Interrupt-driven external-clock MOSI sampler for the no-CS MHI bus.

It samples MOSI from AC-provided SCK edges, reconstructs LSB-first bytes, and emits signature-anchored frame chunks.

Strengths:

- validated on the original ESP32/M5Stack Atom configuration;
- lower RX pressure than the original synchronous FastGPIO path in tested runs;
- supports command-worker classified RX;
- retains `fast_gpio_tx` for split operation.

Trade-offs:

- TX remains software-driven;
- validation is hardware-specific;
- target support is limited to original ESP32 and ESP32-S3 in the current transport manager.

Recommended validated split configuration:

```yaml
MhiAcCtrl:
  rx_driver: external_clock_rx
  command_worker: true
```

### `rmt_spi_rx`

ESP32-S3 receive-only hardware backend.

RMT detects the inter-frame clock gap and the SPI slave peripheral captures each complete frame. This backend still uses a DMA-backed RX buffer internally. That DMA design is independent from the FIFO-only `rmt_cs_spi` transport.

Strengths:

- hardware-assisted complete-frame RX;
- clean 20-byte and 33-byte capture on the tested ESP32-S3;
- completed an approximately 47.5-hour soak with clean protocol health;
- supports command-worker classified RX;
- preserves the proven `fast_gpio_tx` command path.

Trade-offs:

- ESP32-S3 only;
- RX-only: TX remains `fast_gpio_tx`;
- FastGPIO TX can still dominate loop timing or miss background windows;
- uses the inferred internal-CS boundary because the MHI bus has no physical CS line.

Recommended split configuration:

```yaml
MhiAcCtrl:
  rx_driver: rmt_spi_rx
  rmt_spi_frame_gap_us: 1000
  command_worker: true
```

### `rmt_cs_spi`

FIFO-backed full-duplex transport for the original dual-core ESP32 and ESP32-S3.

RMT converts the inter-frame SCK idle gap into an internal CS pulse. The SPI slave peripheral then captures MOSI and shifts MISO as one mode-3, LSB-first transaction.

Strengths:

- hardware owns bit-level RX and TX timing;
- removes FastGPIO TX from the normal transaction path;
- one public driver and one FIFO architecture across ESP32 and ESP32-S3;
- no DMA allocation, alignment, descriptor, or channel lifecycle;
- command completion is reported only after the SPI owner task receives the completed transaction result;
- supports classified RX in the command worker.

Target-specific behaviour:

- original ESP32 applies a driver-local mode-3 receive-edge correction after SPI initialisation;
- ESP32-S3 uses the standard ESP-IDF mode-3 configuration.

Configuration:

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
  rmt_spi_frame_gap_us: 1000
  command_worker: true
```

Do not specify `tx_driver` and do not add a physical CS pin.

### `fast_gpio_tx`

Software GPIO transmit path used by all split RX selections.

Strengths:

- established and broadly exercised command path;
- supports `fast_gpio_rx`, `external_clock_rx`, and `rmt_spi_rx`;
- uses the common TX completion and command-confirmation contract.

Trade-offs:

- follows the AC-owned clock in software;
- can create long main-loop sections;
- background TX attempts may fail even when user commands continue to confirm.

### `none`

Diagnostic RX-only TX selection for supported queue-backed split drivers.

It is not a usable control configuration.

## Hardware guidance

| ESP chip | Validated hardware | Recommended selection | Position |
|---|---|---|---|
| Original ESP32 | M5Stack Atom and current original-ESP32 test path | `rmt_cs_spi` for full duplex; `external_clock_rx` + `fast_gpio_tx` as the validated split alternative | Both paths hardware-tested; original-ESP32 mode-3 correction is internal |
| ESP32-S3 | Current ESP32-S3 test unit | `rmt_cs_spi` for full duplex; `rmt_spi_rx` + `fast_gpio_tx` as the validated split alternative | Both paths hardware-tested; `rmt_cs_spi` uses the standard S3 mode-3 configuration |
| ESP32-C3 | No runtime-validated board | `fast_gpio_rx` + `fast_gpio_tx` | Compile coverage only |

Do not infer compatibility from the generic ESP32 family name. GPIO registers, core topology, RMT revisions, SPI routing, DMA support, and peripheral capabilities vary by target.

## Configuration and tuning

### `frame_size`

Default:

```yaml
frame_size: 20
```

Use `20` for short-frame units.

Use:

```yaml
frame_size: 33
```

for units that provide the extended frame, including horizontal vane and 3D Auto feedback.

An incorrect frame size normally presents as invalid frames, checksum failures, or missing extended features.

### `fan_profile`

Default:

```yaml
fan_profile: four_speed
```

Alternative:

```yaml
fan_profile: three_speed
```

The profile changes fan exposure and encoding. It does not select a different transport implementation.

### `rmt_spi_frame_gap_us`

Default:

```yaml
rmt_spi_frame_gap_us: 1000
```

Valid range: 500-5000 microseconds.

This value applies to both `rmt_spi_rx` and `rmt_cs_spi` and must be:

- greater than the normal inter-byte idle period;
- lower than the inter-frame idle period.

The documented bus has an approximately 250 microsecond inter-byte pause and roughly 40 millisecond inter-frame pause, making 1000 microseconds a practical starting point.

Only tune this when diagnostics show repeated invalid lengths, missed boundaries, or RMT re-arm failures.

### `frame_start_idle_ms`

Default:

```yaml
frame_start_idle_ms: 10
```

This primarily affects the synchronous FastGPIO timing path. Leave it at the default unless measured bus timing shows a different requirement.

### `tx_background_interval_ms`

Default:

```yaml
tx_background_interval_ms: 250
```

Command frames bypass this interval. Background requests wait while command confirmation is pending.

Increase it to reduce background bus pressure. Lower values increase TX activity and can expose split FastGPIO timing limits.

### Command-worker tuning

Defaults:

```yaml
command_worker: false
command_worker_start_delay_ms: 0
command_worker_stack_size: 6144
command_worker_priority: 4
command_worker_core_id: -1
```

Do not tune stack, priority, or core placement without evidence from runtime diagnostics. Functional confirmation, opdata flow, and transport health are more important than a lower loop-time number.

## Compile coverage

The representative compile matrix intentionally covers material chip and transport boundaries rather than every YAML permutation:

| Configuration | Coverage |
|---|---|
| ESP32-C3 + FastGPIO | `fast_gpio_rx`, `fast_gpio_tx`, 20-byte frame, three-speed fan profile |
| Original ESP32 + `rmt_cs_spi` + worker | FIFO full-duplex path and original-ESP32 mode-3 correction |
| ESP32-S3 + `rmt_cs_spi` + worker | FIFO full-duplex S3 path |
| ESP32-S3 + `rmt_spi_rx` + `fast_gpio_tx` + worker | Split hardware RX and legacy software TX path |

Run:

```bash
./scripts/compile-tests.sh
```

The default action is to compile all four representative configurations.

Validation-only remains available:

```bash
./scripts/compile-tests.sh validate
```

Frame-size, fan-profile, command-coordinator, confirmation, worker-policy, and driver-selection permutations belong primarily in unit and schema tests rather than duplicate firmware builds.

## Validation criteria

A driver result is not accepted solely because it compiles or receives frames.

Confirm:

- protocol counters remain clean;
- commands are physically applied and confirmed from returned MOSI state;
- retries recover or are explicitly explained;
- retry exhaustion remains zero;
- opdata continues to publish;
- Home Assistant state follows confirmed AC state;
- queue, overwrite, drop, SPI, and RMT counters remain bounded and understood;
- the same behaviour survives reconnects and extended runtime.

See [`DIAGNOSTICS.md`](DIAGNOSTICS.md) for the required evidence.

## Recording a hardware result

Record at least:

```text
ESP chip and revision
board/module
ESPHome version
ESP-IDF version
AC model
frame size
fan profile
SCK/MOSI/MISO pins
RX selection and effective TX
command-worker settings
tuning overrides
test duration
commands tested
opdata behaviour
end-of-test common diagnostics
end-of-test driver-specific diagnostics
known failures or limitations
```

## Related findings

- [`notes/FINDINGS_RMT_SPI_RX.md`](notes/FINDINGS_RMT_SPI_RX.md)
- [`notes/FINDINGS_fastGpio&ExternalClock.md`](notes/FINDINGS_fastGpio&ExternalClock.md)
- [`notes/FINDINGS_LOUVERS_3D_AUTO.md`](notes/FINDINGS_LOUVERS_3D_AUTO.md)
- [`notes/FINDINGS_FAN_PROFILES.md`](notes/FINDINGS_FAN_PROFILES.md)
