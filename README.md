# MHI-AC-Ctrl-ESPHome-Redux

ESPHome external component for controlling Mitsubishi Heavy Industries air conditioners over the MHI SPI-style bus using ESP32-class hardware.

This Redux version is an ESP-IDF-focused rewrite/refactor of the existing MHI-AC-Ctrl ESPHome work. It keeps the ESPHome/Home Assistant integration model, but splits the protocol, transport, state, publishing, diagnostics, command confirmation, and transport-driver paths so changes can be made in smaller, safer pieces.

This is not a clean-room protocol project. It builds on the original community MHI-AC-Ctrl work, upstream ESPHome component behaviour, and public MHI trace/capture knowledge.

## Current status

The current validated target remains **ESP32-S3 with ESP-IDF**.

Recommended/default path:

```yaml
rx_driver: fast_gpio_rx
tx_driver: fast_gpio_tx
rx_worker: false
tx_worker: false
```

Experimental path:

```yaml
rx_driver: external_clock_rx
tx_driver: fast_gpio_tx
```

Worker mode is currently recommended only for **external-clock RX experiments**. Do not enable workers on the normal FastGPIO backend unless you are actively debugging the worker path.

Implemented and runtime-tested:

- 20-byte and 33-byte MHI frame handling.
- Split RX/TX transport selection.
- FastGPIO RX and FastGPIO TX transport.
- Experimental external-clock RX with FastGPIO TX on ESP32 and ESP32-S3.
- Latest-slot frame cataloging for status, extended status, opdata, and unknown frames.
- Command confirmation and duplicate command suppression.
- Climate control with confirmed-state updates.
- Fan speed select.
- Vertical vane select.
- Horizontal vane select from 33-byte feedback.
- 3D Auto switch and read-only 3D Auto feedback sensor.
- Sensor parity for common status and opdata telemetry.
- Runtime diagnostics for RX, TX, workers, frame cataloging, commands, loop budget, and protocol health.

Still intentionally conservative:

- FastGPIO RX/TX remains the stable/default transport path.
- `external_clock_rx` remains experimental and should be validated with soak logs before being treated as supported.
- `rx_worker` and `tx_worker` are optional, experimental, and disabled by default.
- Workers are not currently recommended for `fast_gpio_rx`.

For detailed test findings and transport trade-offs, see [`FINDINGS.md`](FINDINGS.md).

## Hardware assumptions

The MHI bus used by this component has:

- SCK
- MOSI
- MISO
- no chip-select line

The air conditioner is the SPI-style bus master. The ESP device listens to MOSI/SCK and drives MISO at the correct time.

Known-good current development target:

- ESP32-S3
- ESP-IDF framework
- 33-byte frame mode
- `rx_driver: fast_gpio_rx`
- `tx_driver: fast_gpio_tx`
- workers disabled

Experimental transport targets:

- ESP32 / ESP32-S3 with `rx_driver: external_clock_rx` and `tx_driver: fast_gpio_tx`
- ESP32 / ESP32-S3 with `external_clock_rx` and optional workers
- ESP32-C3 compile coverage only; runtime behaviour is not validated

## ESP32-C3 status

ESP32-C3 support is **experimental**.

Current status:

- ESP32-C3 ESP-IDF compile coverage exists.
- The FastGPIO transport has target-specific register handling for ESP32-C3-class GPIO registers.
- The component compiles for ESP32-C3.
- Runtime behaviour has **not** been validated on real ESP32-C3 hardware.

Important limitations:

- ESP32-C3 is single-core.
- FastGPIO capture is timing-sensitive and software-heavy.
- Passing compile tests does not prove the C3 can reliably capture the MHI bus under Wi-Fi, logging, API, and Home Assistant load.
- ESP32-S3 remains the validated and recommended target.

Use ESP32-C3 only for development/testing until hardware soak logs show clean protocol health:

```text
valid_frames climbs
invalid_frames = 0
checksum_failures = 0
signature_misses = 0
sync_losses = 0
dropped_bytes = 0
commands confirm
opdata continues to publish when requested
```

## Transport drivers

Examples deliberately set the RX and TX drivers with concrete 1:1 driver names:

```yaml
rx_driver: fast_gpio_rx
tx_driver: fast_gpio_tx
```

There are no legacy aliases in the schema. `fast_gpio_rx` is an RX driver and `fast_gpio_tx` is a TX driver.

The transport layer treats RX and TX as separate responsibilities:

```text
mhi_fast_gpio_rx_driver
mhi_fast_gpio_tx_driver
mhi_external_clock_rx_driver
mhi_null_tx_driver
```

The stable/default path is:

```yaml
rx_driver: fast_gpio_rx
tx_driver: fast_gpio_tx
rx_worker: false
tx_worker: false
```

This uses the split FastGPIO RX and FastGPIO TX drivers. RX captures MOSI/SCK and publishes a frame-end bus marker. TX queues frames and attempts to drive MISO after a new RX bus marker is observed.

The experimental external-clock path is:

```yaml
rx_driver: external_clock_rx
tx_driver: fast_gpio_tx
```

`external_clock_rx` samples MOSI from the external SCK edge and emits signature-anchored frame chunks. It also publishes the same frame-end bus marker used by FastGPIO RX, so FastGPIO TX can respond from RX-observed bus timing instead of blindly polling for a transmit window.

RX-only validation is also available:

```yaml
rx_driver: external_clock_rx
tx_driver: none
```

The current bus has no chip-select line, so normal ESP-IDF SPI slave assumptions do not fit this hardware cleanly. FastGPIO remains the stable baseline while hardware-assisted receive work, such as LCD-CAM/I2S-style external-clock capture, remains experimental.

## Installation

Add this repository as an ESPHome external component.

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/Astute4185/MHI-AC-Ctrl-ESPHome_ESP-IDF
      ref: ESP-IDF-Redux
    components: [MhiAcCtrl]
```

## Minimal component configuration

Recommended FastGPIO configuration:

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: fast_gpio_rx
  tx_driver: fast_gpio_tx
  rx_worker: false
  tx_worker: false
  room_temp_timeout: 60
```

Required component-level fields:

```text
id
frame_size
sck_pin
mosi_pin
miso_pin
rx_driver
tx_driver
```

Optional fields:

```text
rx_worker
tx_worker
rx_worker_start_delay_ms
rx_worker_stack_size
rx_worker_priority
rx_worker_core_id
tx_worker_start_delay_ms
tx_worker_stack_size
tx_worker_priority
tx_worker_core_id
tx_background_interval_ms
room_temp_timeout
external_temperature_sensor
```

## Worker mode

Worker mode is experimental and disabled by default.

Current recommendation:

```text
FastGPIO RX/TX:
  keep workers disabled

External-clock RX experiments:
  workers may be enabled for testing
```

The worker path is currently intended for external-clock RX testing only:

```yaml
MhiAcCtrl:
  id: mhi_ac

  # Required bus/frame config.
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39

  # Experimental transport selection.
  rx_driver: external_clock_rx
  tx_driver: fast_gpio_tx

  # Optional worker mode.
  rx_worker: true
  tx_worker: true

  # Optional worker tuning.
  rx_worker_start_delay_ms: 5000
  rx_worker_stack_size: 6144
  rx_worker_priority: 4
  rx_worker_core_id: 0

  tx_worker_start_delay_ms: 5000
  tx_worker_stack_size: 6144
  tx_worker_priority: 4
  tx_worker_core_id: 1

  # Optional. Recommended for worker testing to reduce background TX pressure.
  tx_background_interval_ms: 1000
```

Worker responsibilities:

```text
rx_worker:
  capture/sync/classify/catalog RX frames only

 tx_worker:
  flush queued TX frames against RX bus markers

main loop:
  decode cataloged frames
  publish ESPHome entities
  manage command confirmation
  queue TX frames
```

Do not assume workers improve stability simply because the board is dual-core. On the FastGPIO backend, worker mode can improve loop timing counters while degrading real behaviour such as command confirmation or opdata flow.

A worker test should only be considered healthy if all of these remain true:

```text
valid_frames increases
checksum_failures stays at 0
signature_misses stays at 0
tx_failures stays low
command_confirmations increases when commands are sent
command_confirmation_timeouts stays at 0
opdata frames continue to appear when requested
Home Assistant state matches the physical AC state
```

### Worker queue policy

Do not use a general FIFO queue for status or extended-status frames. A bounded FIFO was considered during RX-worker testing, but rejected because it reintroduces queue/timing pressure and works against the latest-state design.

Preferred worker/catalog shape:

```text
status:
  latest slot only

extended status:
  latest slot only

opdata:
  keyed latest slots

command feedback:
  optional latest command-candidate side slot
```

The intent is to keep worker mode from building a backlog. Repeated steady-state frames should collapse to the latest value. Transient command feedback may have a dedicated latest side slot, but should not become a FIFO backlog.

## Frame size

Use `frame_size: 20` for older units that do not support the larger frame layout.

Use `frame_size: 33` for units that support the larger frame layout and need features such as horizontal vanes or 3D Auto feedback.

If you see repeated checksum or signature failures, verify `frame_size` before changing anything else.

## Climate

```yaml
climate:
  - platform: MhiAcCtrl
    name: Guest Bedroom AC
    mhi_ac_ctrl_id: mhi_ac
    visual_min_temperature: 16
    visual_max_temperature: 31
    visual_temperature_step: 1
```

Climate state is published from confirmed decoded MHI state.

## Selects

```yaml
select:
  - platform: MhiAcCtrl
    mhi_ac_ctrl_id: mhi_ac
    fan_speed:
      name: Fan Speed
    vertical_vanes:
      name: Fan Control Up Down
    horizontal_vanes:
      name: Fan Control Left Right
```

Supported fan options:

- Auto
- Low
- Medium
- High

Quiet mode is not currently exposed as a normal fan speed. Some ACs appear to have a separate quiet/silent mode, but it should not be assumed to be part of the internal fan-speed enum unless the feedback code is clearly identified.

Supported vertical vane options:

- Up
- Up/Center
- Center/Down
- Down
- Swing

Supported horizontal vane options:

- Left
- Left/Center
- Center
- Center/Right
- Right
- Wide
- Spot
- Swing

Horizontal vane feedback requires 33-byte status frames.

## Switches

```yaml
switch:
  - platform: MhiAcCtrl
    mhi_ac_ctrl_id: mhi_ac
    vanes_3d_auto:
      name: 3D Auto
```

3D Auto command state is confirmed from decoded feedback. The switch should settle to the state reported by the AC.

## Binary sensors

```yaml
binary_sensor:
  - platform: MhiAcCtrl
    mhi_ac_ctrl_id: mhi_ac
    power:
      name: Power
    defrost:
      name: Defrost
    vanes_3d_auto_enabled:
      name: 3D Auto enabled
```

`vanes_3d_auto_enabled` is read-only feedback from decoded 33-byte status frames.

## Text sensors

```yaml
text_sensor:
  - platform: MhiAcCtrl
    mhi_ac_ctrl_id: mhi_ac
    error_code:
      name: Error code
    protection_state:
      name: Protection state
```

`protection_state` is decoded from opdata when the unit provides it.

## Sensors

```yaml
sensor:
  - platform: MhiAcCtrl
    mhi_ac_ctrl_id: mhi_ac
    room_temperature:
      name: Room temperature
    target_temperature:
      name: Target temperature
    outdoor_temperature:
      name: Outdoor temperature
    return_air_temperature:
      name: Return air temperature
    compressor_frequency:
      name: Compressor frequency
    current_power:
      name: Current
    indoor_unit_fan_speed:
      name: Indoor unit fan speed
    outdoor_unit_fan_speed:
      name: Outdoor unit fan speed
    indoor_unit_total_run_time:
      name: Indoor unit total run time
    compressor_total_run_time:
      name: Compressor total run time
    energy_used:
      name: Energy used
    indoor_unit_thi_r1:
      name: Indoor unit THI R1
    indoor_unit_thi_r2:
      name: Indoor unit THI R2
    indoor_unit_thi_r3:
      name: Indoor unit THI R3
    outdoor_unit_tho_r1:
      name: Outdoor unit THO R1
    outdoor_unit_expansion_valve:
      name: Outdoor unit expansion valve
    outdoor_unit_discharge_pipe:
      name: Outdoor unit discharge pipe
    outdoor_unit_discharge_pipe_super_heat:
      name: Outdoor unit discharge pipe super heat
    protection_state_number:
      name: Protection state number
```

Opdata sensors are validity-gated. If the air conditioner does not provide a field, the entity should remain unavailable instead of publishing a bogus zero.

## External room temperature

An external ESPHome temperature sensor can be attached:

```yaml
MhiAcCtrl:
  id: mhi_ac
  external_temperature_sensor: external_room_temp
```

Manual action support also exists:

```yaml
on_...:
  then:
    - climate.mhi.set_external_room_temperature:
        mhi_ac_ctrl_id: mhi_ac
        temperature: 22.5
```

## Vane actions

Automation actions are available for direct vane control:

```yaml
on_...:
  then:
    - climate.mhi.set_vertical_vanes:
        mhi_ac_ctrl_id: mhi_ac
        position: 5
```

```yaml
on_...:
  then:
    - climate.mhi.set_horizontal_vanes:
        mhi_ac_ctrl_id: mhi_ac
        position: 8
```

## Runtime diagnostics

The component logs runtime diagnostics for transport, protocol health, frame cataloging, worker state, command confirmation, and loop budget.

Healthy runtime should look like:

```text
runtime: rx_bytes=... candidate_frames=... valid_frames=... invalid_frames=0 checksum_failures=0
runtime: signature_misses=0 sync_losses=0 dropped_bytes=0 tx_frames=... tx_failures=0
runtime: command_confirmations=... command_confirmation_timeouts=0 pending_confirmation_mask=0x00000000
runtime: catalog ingested=... status=... extended=... opdata=... unknown=... overwritten=... opdata_slots_full=0
runtime: rx_worker enabled=... running=... loops=... ingested=... idle_yields=...
runtime: tx_worker enabled=... running=... loops=... flush_attempts=... flush_successes=... idle_yields=...
runtime: loop_us last=... avg=... max=... over_budget=... budget=30000
```

Recommended use:

- Use detailed diagnostics while bringing up hardware or soak testing.
- Reduce log level once the device is stable.
- For worker testing, validate command confirmation and opdata flow, not just RX frame health.

Important counters:

- `valid_frames`
- `invalid_frames`
- `checksum_failures`
- `signature_misses`
- `sync_losses`
- `dropped_bytes`
- `tx_failures`
- `command_confirmation_timeouts`
- `catalog status`
- `catalog extended`
- `catalog opdata`
- `opdata_slots_full`
- `rx_worker running`
- `tx_worker running`
- `tx_worker flush_successes`
- `loop_us avg`
- `loop_us max`
- `over_budget`

Healthy target:

```text
valid_frames climbs
invalid_frames = 0
checksum_failures = 0
sync_losses = 0
tx_failures stays low
commands confirm
opdata continues to appear when requested
loop_us avg < 30000us
over_budget does not climb during normal operation
```

## Command confirmation and duplicate suppression

Commands are staged and then confirmed against decoded MOSI feedback.

Implemented command safety behaviour:

- Confirmed decoded state wins over requested state.
- Duplicate confirmable commands are suppressed while the same command is pending.
- Command timeouts are counted in diagnostics.

This prevents rapid Home Assistant/UI input changes from repeatedly restaging the same pending command.

## Validation checklist

Before submitting a merge request:

```bash
./scripts/lint.sh fix
./scripts/test.sh
./scripts/compile-tests.sh
```

For hardware validation:

```text
valid_frames climbs
invalid_frames = 0
checksum_failures = 0
sync_losses = 0
commands confirm
opdata sensors continue to publish when requested
Home Assistant state settles to confirmed AC feedback
```

For worker validation, also check:

```text
rx_worker/tx_worker state is what the YAML intended
command_confirmation_timeouts = 0
opdata does not stop flowing
loop_us and over_budget improve without breaking functional behaviour
```

## Troubleshooting

### Long ESPHome loop warnings

Synchronous FastGPIO RX/TX can block the ESPHome loop while waiting for the external MHI frame cadence. This can produce long-operation warnings even when RX is reliable.

Treat clean protocol counters as necessary but not sufficient. If opdata stops flowing or commands stop confirming, the run is not healthy even if `valid_frames` looks good.

### Worker mode looks faster but behaviour regresses

Workers can improve loop timing counters while still breaking command confirmation or opdata flow.

For normal FastGPIO operation, disable workers:

```yaml
rx_driver: fast_gpio_rx
tx_driver: fast_gpio_tx
rx_worker: false
tx_worker: false
```

Use workers only for external-clock RX experiments unless actively debugging worker internals.

### Sensor remains unavailable

Many opdata fields are model-dependent. A configured sensor only publishes after the AC returns a valid response for that field.

Unavailable is preferred over publishing bogus zero values.

### Fan or vane command appears to bounce

Confirmed AC feedback is authoritative. If the AC clamps or rejects a requested state, the UI will settle to the confirmed decoded state.

Repeated identical pending commands should be suppressed by the command debounce/duplicate suppression path.

## Development notes

The code is split into protocol, transport, state, publish, diagnostics, and platform glue:

```text
mhi_checksum.*
mhi_frame_sync.*
mhi_frame_classifier.*
mhi_frame_catalog.*
mhi_status_decoder.*
mhi_opdata_decoder.*
mhi_tx_builder.*
mhi_command_confirmation.*
mhi_publish_bridge.*
mhi_transport_manager.*
mhi_fast_gpio_rx_driver.*
mhi_fast_gpio_tx_driver.*
mhi_external_clock_rx_driver.*
mhi_null_tx_driver.*
mhi_diag.*
mhi_stats.*
```

Tests live under:

```text
tests/unit/
tests/fixtures/
tests/components/
```

Run host tests:

```bash
./scripts/test.sh
```

Run ESPHome compile tests:

```bash
./scripts/compile-tests.sh
```

Run lint:

```bash
./scripts/lint.sh fix
```

## Current design rules

- Keep sensor/opdata fields validity-gated.
- Keep confirmed decoded state authoritative.
- Keep FastGPIO as the supported/default transport path.
- Keep RX/TX driver selection explicit in examples.
- Drive split FastGPIO TX from RX frame-end bus markers, not blind polling.
- Keep workers opt-in and experimental.
- Recommend workers only for `external_clock_rx` testing.
- Do not recommend workers for `fast_gpio_rx` normal use.
- Do not add general status/extended-status FIFO queues; use latest slots to avoid queue/timing pressure.
- Use a dedicated latest command-candidate side slot only when transient command feedback needs protection from normal latest-slot overwrites.
- Do not combine transport experiments with sensor parity changes.

## Roadmap

Near term:

- Keep the default path conservative: `fast_gpio_rx` + `fast_gpio_tx`, workers disabled.
- Document `external_clock_rx` as the main experimental worker path.
- Continue soaking external-clock RX against FastGPIO TX.
- Continue fan/control validation on the stable FastGPIO path.
- Continue command confirmation hardening around rapid UI changes.

Later:

- Treat ESP32-C3/single-core support as experimental until validated on real hardware with clean soak logs.
- Re-investigate LCD-CAM/I2S-style external-clock RX capture for a future hardware-assisted backend.
- Keep TX on the stable FastGPIO path unless a clear reason appears to change it.

## Credits

This project builds on prior MHI reverse-engineering and ESPHome integration work.

- Upstream ESPHome project base: [ginkage/MHI-AC-Ctrl-ESPHome](https://github.com/ginkage/MHI-AC-Ctrl-ESPHome)
- Bus capture and trace reference: [absalom-muc/MHI-AC-Trace](https://github.com/absalom-muc/MHI-AC-Trace)
- FastGPIO inspiration/reference work: [RobertJansen1/MHI-AC-Ctrl-ESPHome esp32_errors branch](https://github.com/RobertJansen1/MHI-AC-Ctrl-ESPHome/tree/esp32_errors)
- Original reverse-engineering lineage and MHI protocol work from the wider `MHI-AC-Ctrl` community

## License

See `LICENSE`.