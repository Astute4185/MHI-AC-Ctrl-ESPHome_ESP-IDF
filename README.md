# MHI-AC-Ctrl-ESPHome-Redux

ESPHome external component for controlling Mitsubishi Heavy Industries air conditioners over the MHI SPI-style bus using ESP32-class hardware.

This Redux version is an ESP-IDF-focused rewrite/refactor of the existing MHI-AC-Ctrl ESPHome work. It keeps the ESPHome/Home Assistant integration model, but splits the protocol, transport, state, publishing, diagnostics, and command confirmation paths so changes can be made in smaller, safer pieces.

This is not a clean-room protocol project. It builds on the original community MHI-AC-Ctrl work, upstream ESPHome component behaviour, and public MHI trace/capture knowledge.

## Current status

The current validated target is **ESP32-S3 with ESP-IDF**.

Implemented and runtime-tested:

- 20-byte and 33-byte MHI frame handling.
- FastGPIO RX/TX transport.
- Optional RX worker path for reducing ESPHome loop blocking.
- `rx_worker: auto`, `true`, and `false` modes.
- Climate control with confirmed-state updates.
- Fan speed select.
- Vertical vane select.
- Horizontal vane select from 33-byte feedback.
- 3D Auto switch and read-only 3D Auto feedback sensor.
- Sensor parity for common status and opdata telemetry.
- Command confirmation and duplicate command suppression.
- Runtime diagnostics for RX, TX, commands, loop budget, and RX worker health.

Still intentionally conservative:

- `rx_worker` remains configurable and should be validated on each hardware target.
- Single-core chips are not the recommended target for RX worker mode.

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
- FastGPIO RX/TX
- `rx_worker: auto` or `rx_worker: true` after validation

Single-core chips should generally use `rx_worker: false` unless you are explicitly testing them.

## Why FastGPIO is specified explicitly

Examples deliberately set:

```yaml
rx_driver: fast_gpio
tx_driver: fast_gpio
```

FastGPIO is the currently validated transport backend. Naming it in YAML is intentional because the transport layer is modular: future backends can be added without changing the climate, sensor, select, switch, command, or publish logic.

This also makes runtime logs clearer. The component can report both the configured driver and the active driver, which is useful when testing new receive paths or falling back to the known-good implementation.

The current bus has no chip-select line, so normal ESP-IDF SPI slave assumptions do not fit this hardware cleanly. FastGPIO remains the stable baseline while future hardware-assisted receive work, such as LCD-CAM/I2S-style external-clock capture, remains experimental.

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

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: fast_gpio
  tx_driver: fast_gpio
  room_temp_timeout: 60
```

`rx_worker` is optional. If omitted, it behaves as `auto`.

## RX worker mode

`rx_worker` supports three behaviours:

```yaml
rx_worker: auto   # omitted/null/auto: enable on multi-core chips, disable on single-core chips
rx_worker: true   # force enabled
rx_worker: false  # force disabled
```

Recommended default for ESP32-S3 testing:

```yaml
rx_worker: auto
```

Recommended safe fallback:

```yaml
rx_worker: false
```

Boot logs should show the resolved mode:

```text
RX worker mode: auto
Chip cores: 2
RX worker configured: YES
RX worker resolved: YES
RX worker active: YES
```

Do not use an undeclared substitution such as this:

```yaml
rx_worker: ${rx_worker}
```

That is only valid if you also define the substitution:

```yaml
substitutions:
  rx_worker: auto
```

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
- Quiet
- Low
- Medium
- High

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

The component logs runtime diagnostics for transport, protocol health, command confirmation, loop budget, and RX worker health.

Healthy RX worker runtime should look like:

```text
runtime: rx_bytes=... candidate_frames=... valid_frames=... invalid_frames=0 checksum_failures=0
runtime: signature_misses=0 sync_losses=0 dropped_bytes=0 tx_frames=... tx_failures=0
runtime: command_confirmations=... command_confirmation_timeouts=0 pending_confirmation_mask=0x00000000
runtime: loop_us last=... avg=... max=... over_budget=0 budget=30000
runtime: rx_worker samples=... frames=... drained=... queue_depth=0 queue_max=... queue_overflows=0
runtime: rx_worker_health active=YES healthy=YES startup_grace_expired=YES ...
```

Recommended use:

- Use detailed diagnostics while bringing up hardware or soak testing.
- Reduce log level once the device is stable.
- Avoid enabling the RX worker by default on single-core chips unless that target has been separately validated.

Important counters:

- `valid_frames`
- `invalid_frames`
- `checksum_failures`
- `signature_misses`
- `sync_losses`
- `dropped_bytes`
- `tx_failures`
- `command_confirmation_timeouts`
- `queue_overflows`
- `loop_us avg`
- `rx_worker_health healthy`

Healthy target:

```text
valid_frames climbs
invalid_frames = 0
checksum_failures = 0
sync_losses = 0
queue_overflows = 0
loop_us avg < 30000us
rx_worker_health healthy=YES
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
bash scripts/test.sh
scripts/compile-tests.sh
scripts/lint.sh
```

For hardware validation:

```text
valid_frames climbs
invalid_frames = 0
checksum_failures = 0
sync_losses = 0
queue_overflows = 0
commands confirm
Home Assistant state settles to confirmed AC feedback
```

## Troubleshooting

### Long ESPHome loop warnings

If `rx_worker: false`, synchronous FastGPIO RX can block the ESPHome loop while waiting for the external MHI frame cadence. This can produce long-operation warnings even when RX is reliable.

Use `rx_worker: auto` or `rx_worker: true` on validated ESP32-S3 hardware to move frame capture out of the ESPHome loop.

### RX worker enabled but no frames

Check diagnostics:

```text
rx_worker: enabled but no frames after startup grace
rx_worker: stalled, no new frames for Xms
rx_worker: queue not draining for Xms
```

Fallback to:

```yaml
rx_worker: false
```

Then retest the synchronous path.

### `${rx_worker}` substitution error

This is a YAML/package problem, not a component problem.

This is invalid unless `rx_worker` is declared under `substitutions`:

```yaml
rx_worker: ${rx_worker}
```

Either remove the line entirely, set a direct value, or add:

```yaml
substitutions:
  rx_worker: auto
```

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
mhi_status_decoder.*
mhi_opdata_decoder.*
mhi_tx_builder.*
mhi_command_confirmation.*
mhi_publish_bridge.*
mhi_transport_manager.*
mhi_fast_gpio_driver.*
mhi_rx_worker.*
mhi_rx_worker_mode.*
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
bash scripts/test.sh
```

Run ESPHome compile tests:

```bash
scripts/compile-tests.sh
```

Run lint:

```bash
scripts/lint.sh
```

## Current design rules

- Keep sensor/opdata fields validity-gated.
- Keep confirmed decoded state authoritative.
- Keep RX worker optional and diagnosable.
- Keep synchronous FastGPIO as a fallback path.
- Keep transport backend selection explicit in examples.
- Do not combine transport experiments with sensor parity changes.

## Roadmap

Near term:

- Complete RX worker soak validation on ESP32-S3.
- Keep collecting worker health diagnostics across real installations.
- Keep `rx_worker` optional until more hardware targets are validated.

Later:

- Revisit single-core support only if someone is willing to validate it separately.
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