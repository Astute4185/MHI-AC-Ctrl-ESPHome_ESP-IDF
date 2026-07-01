# MHI-AC-Ctrl-ESPHome-Redux

ESPHome external component for controlling Mitsubishi Heavy Industries air conditioners over the MHI SPI-style bus using ESP32-class hardware.

This Redux version is an ESP-IDF-focused rewrite/refactor of the existing MHI-AC-Ctrl ESPHome work. It keeps the ESPHome/Home Assistant integration model, but splits the protocol, transport, state, publishing, diagnostics, and command confirmation paths so changes can be made in smaller, safer pieces.

This is not a clean-room protocol project. It builds on the original community MHI-AC-Ctrl work, upstream ESPHome component behaviour, and public MHI trace/capture knowledge.

## Current status

The current validated target is **ESP32-S3 with ESP-IDF**.

Implemented and runtime-tested:

- 20-byte and 33-byte MHI frame handling.
- FastGPIO RX/TX transport.
- Experimental external-clock RX transport on ESP32-S3.
- Hybrid transport mode using external-clock RX with FastGPIO TX.
- Climate control with confirmed-state updates.
- Fan speed select.
- Vertical vane select.
- Horizontal vane select from 33-byte feedback.
- 3D Auto switch and read-only 3D Auto feedback sensor.
- Sensor parity for common status and opdata telemetry.
- Command confirmation and duplicate command suppression.
- Runtime diagnostics for RX, TX, commands, loop budget, and protocol health.

Stable/default path:

- `rx_driver: fast_gpio`
- `tx_driver: fast_gpio`

Experimental path:

- `rx_driver: external_clock_rx`
- `tx_driver: fast_gpio` or `tx_driver: none` for RX-only validation

FastGPIO remains the safest fully working transport because RX and TX/commands are both validated. External-clock RX is currently the best CPU/scheduling improvement path. It has shown clean RX windows and a large reduction in ESPHome loop time, but should remain opt-in until longer hybrid RX/TX soak testing is complete.

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

## ESP32-C3 status

ESP32-C3 support is **experimental**.

Current status:

- ESP32-C3 ESP-IDF compile coverage exists.
- The FastGPIO transport has target-specific register handling for ESP32-C3-class GPIO registers.
- The component compiles for ESP32-C3.
- Runtime behaviour has **not** been validated on real ESP32-C3 hardware.

Important limitations:

- ESP32-C3 is single-core.
- The supported RX path is synchronous FastGPIO, which is timing-sensitive.
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
```

## Transport backends

Examples deliberately set the transport backends explicitly:

```yaml
rx_driver: fast_gpio
tx_driver: fast_gpio
```

FastGPIO is the stable baseline. It is synchronous and timing-sensitive, but it has validated RX and TX behaviour. Naming it in YAML is intentional because the transport layer is modular: receive and transmit paths can be tested independently without changing the climate, sensor, select, switch, command, or publish logic.

The current bus has no chip-select line, so normal ESP-IDF SPI slave assumptions do not fit this hardware cleanly. The component therefore supports explicit transport selection and reports both configured and active drivers at boot.

### CPU / scheduling comparison

| Backend | RX quality | TX / commands | Loop budget |
| --- | --- | --- | --- |
| FastGPIO RX/TX | 100% valid in clean runs | Working | Poor under ESPHome scheduling, around 49 ms average loop in observed runs |
| External-clock RX + FastGPIO TX | Around 99%+ valid, with clean windows during tuning | RX-only validation | Excellent, around 23 us average loop in observed runs |

External-clock RX is an ISR/software-shifter prototype. It samples MOSI from the external SCK edge, anchors frames on the MHI signature, emits complete frame chunks, and hands those chunks to the existing decoder path. This keeps the protocol, state, publishing, and command-confirmation layers unchanged.

Current best RX-only tuning on the tested ESP32-S3 unit:

```yaml
rx_driver: external_clock_rx
tx_driver: none
external_clock_edge: falling
```

The following external-clock settings are embedded defaults and normally do not need to be set:

```yaml
external_clock_sample_delay_nops: 0
external_clock_byte_gap_us: 80
external_clock_frame_gap_us: 5000
external_clock_min_edge_gap_us: 4
```

Only override these options while tuning a specific unit or debugging capture quality.

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

Stable FastGPIO configuration:

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


## Experimental external-clock RX

External-clock RX is opt-in and currently intended for ESP32-S3 testing only.

RX-only validation mode:

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: external_clock_rx
  tx_driver: none
  room_temp_timeout: 60
```

Hybrid validation mode with the existing TX path:

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: external_clock_rx
  tx_driver: fast_gpio
  room_temp_timeout: 60
```

Recommended external-clock defaults are embedded in the component. In normal testing, only select the backend and leave the timing options unset.

Optional tuning values:

```yaml
MhiAcCtrl:
  # ... normal config ...
  external_clock_edge: falling
  external_clock_sample_delay_nops: 0
  external_clock_byte_gap_us: 80
  external_clock_frame_gap_us: 5000
  external_clock_min_edge_gap_us: 4
```

Use the tuning values only when comparing capture behaviour. The current best tested setting is `external_clock_edge: falling` with `external_clock_sample_delay_nops: 0`.

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

The component logs runtime diagnostics for transport, protocol health, command confirmation, and loop budget.

Healthy runtime should look like:

```text
runtime: rx_bytes=... candidate_frames=... valid_frames=... invalid_frames=0 checksum_failures=0
runtime: signature_misses=0 sync_losses=0 dropped_bytes=0 tx_frames=... tx_failures=0
runtime: command_confirmations=... command_confirmation_timeouts=0 pending_confirmation_mask=0x00000000
runtime: loop_us last=... avg=... max=... over_budget=... budget=30000
```

Recommended use:

- Use detailed diagnostics while bringing up hardware or soak testing.
- Reduce log level once the device is stable.

Important counters:

- `valid_frames`
- `invalid_frames`
- `checksum_failures`
- `signature_misses`
- `sync_losses`
- `dropped_bytes`
- `tx_failures`
- `command_confirmation_timeouts`
- `loop_us avg`

External-clock RX also reports probe counters such as:

- `edge`
- `delay`
- `frame_chunks`
- `signature_starts`
- `discarded_presync_bytes`
- `discarded_partial_frames`
- `dropped_frame_chunks`
- `dropped`
- `ring_capacity`
- `peak_buffered`

Healthy target:

```text
valid_frames climbs
invalid_frames = 0
checksum_failures = 0
sync_losses = 0
loop_us avg < 30000us
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
commands confirm
Home Assistant state settles to confirmed AC feedback
```

## Troubleshooting

### Long ESPHome loop warnings

Synchronous FastGPIO RX can block the ESPHome loop while waiting for the external MHI frame cadence. This can produce long-operation warnings even when RX is reliable. Treat clean protocol counters as the source of truth.

External-clock RX was added to reduce this scheduling pressure. In RX-only validation it has shown much lower loop time because the receive path no longer waits synchronously for a complete frame in the main ESPHome loop.

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
- Keep FastGPIO RX/TX as the stable baseline.
- Keep external-clock RX opt-in until hybrid RX/TX soak testing is clean.
- Keep transport backend selection explicit in examples.
- Keep external-clock timing options embedded as defaults, with YAML overrides only for tuning/debugging.
- Do not combine transport experiments with sensor parity changes.

## Roadmap

Near term:

- Soak `external_clock_rx` with `tx_driver: fast_gpio` to check whether TX timing affects RX capture.
- Keep FastGPIO RX/TX as the default until hybrid mode has longer clean logs.

Later:

- Treat ESP32-C3/single-core support as experimental until validated on real hardware with clean soak logs.
- Keep TX on the stable FastGPIO path unless a clear reason appears to change it.

## Credits

This project builds on prior MHI reverse-engineering and ESPHome integration work.

- Upstream ESPHome project base: [ginkage/MHI-AC-Ctrl-ESPHome](https://github.com/ginkage/MHI-AC-Ctrl-ESPHome)
- Bus capture and trace reference: [absalom-muc/MHI-AC-Trace](https://github.com/absalom-muc/MHI-AC-Trace)
- FastGPIO inspiration/reference work: [RobertJansen1/MHI-AC-Ctrl-ESPHome esp32_errors branch](https://github.com/RobertJansen1/MHI-AC-Ctrl-ESPHome/tree/esp32_errors)
- Original reverse-engineering lineage and MHI protocol work from the wider `MHI-AC-Ctrl` community

## License

See `LICENSE`.