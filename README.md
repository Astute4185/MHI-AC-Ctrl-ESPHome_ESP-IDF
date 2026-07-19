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
```

Preferred experimental ESP32-S3 path:

```yaml
rx_driver: rmt_spi_rx
tx_driver: fast_gpio_tx
rx_worker: false
tx_worker: false
rmt_spi_frame_gap_us: 1000
```

`rmt_spi_rx` uses RMT to infer the missing chip-select boundary and the ESP32-S3 SPI slave peripheral with DMA to capture complete frames. A roughly 47.5-hour soak completed about 3.42 million transactions with zero checksum failures, signature misses, sync losses, backend queue errors, completed-frame overwrites, or completed-frame drops. Workers should remain disabled; current generic RX/TX workers are polling-based and regress the otherwise clean hardware-assisted path.

Implemented and runtime-tested:

- 20-byte and 33-byte MHI frame handling.
- Split RX/TX transport selection.
- FastGPIO RX and FastGPIO TX transport.
- Experimental external-clock RX with FastGPIO TX on ESP32 and ESP32-S3.
- Experimental RMT-generated-CS SPI/DMA RX with FastGPIO TX on ESP32-S3.
- Latest-slot frame cataloging for status, extended status, opdata, and unknown frames.
- Command confirmation and duplicate command suppression.
- Climate control with confirmed-state updates.
- Configurable three-speed or Quiet/four-speed fan profile.
- Vertical vane select.
- Horizontal vane select from 33-byte feedback.
- 3D Auto switch and read-only 3D Auto feedback sensor.
- Sensor parity for common status and opdata telemetry.
- Runtime diagnostics for RX, TX, workers, frame cataloging, commands, loop budget, and protocol health.

Still intentionally conservative:

- FastGPIO RX/TX remains the stable/default transport path.
- `rmt_spi_rx` remains experimental, although the ESP32-S3 path has completed a roughly 47.5-hour soak with clean RX protocol health.
- `external_clock_rx` remains experimental and available as a software external-clock comparison path.
- `rx_worker` and `tx_worker` are optional, experimental, and disabled by default.
- Workers are not currently recommended for `fast_gpio_rx` or `rmt_spi_rx`.

For the hardware-assisted RX findings, see [`notes/FINDINGS_RMT_SPI_RX.md`](notes/FINDINGS_RMT_SPI_RX.md). For earlier FastGPIO/external-clock findings, see [`notes/FINDINGS_fastGpio&ExternalClock.md`](notes/FINDINGS_fastGpio&ExternalClock.md).

## Hardware assumptions

The MHI bus used by this component has:

- SCK
- MOSI
- MISO
- no chip-select line

The air conditioner is the SPI-style bus master. The ESP device listens to MOSI/SCK and drives MISO at the correct time.

Known-good stable baseline:

- ESP32-S3
- ESP-IDF framework
- 33-byte frame mode
- `rx_driver: fast_gpio_rx`
- `tx_driver: fast_gpio_tx`
- workers disabled

Validated experimental development path:

- ESP32-S3 with `rx_driver: rmt_spi_rx` and `tx_driver: fast_gpio_tx`
- `rmt_spi_frame_gap_us: 1000`
- workers disabled
- clean 33-byte SPI/DMA capture, opdata flow, and command confirmation through a roughly 47.5-hour hardware soak

Other experimental transport targets:

- ESP32 / ESP32-S3 with `rx_driver: external_clock_rx` and `tx_driver: fast_gpio_tx`
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

There are no legacy aliases in the schema. `rx_driver` is the primary transport selection and `tx_driver` is an optional advanced override. Existing explicit split configurations remain valid.

The default TX mapping is:

```text
fast_gpio_rx     -> fast_gpio_tx
external_clock_rx -> fast_gpio_tx
rmt_spi_rx       -> fast_gpio_tx
rmt_cs_spi       -> full-duplex rmt_cs_spi ownership
```

The transport layer retains separate RX and TX drivers for split operation and a shared duplex backend where one peripheral owns both directions:

```text
mhi_fast_gpio_rx_driver
mhi_fast_gpio_tx_driver
mhi_external_clock_rx_driver
mhi_rmt_spi_rx_driver
mhi_null_tx_driver
mhi_rmt_cs_spi_transport
```

The stable/default path is now configured with the RX selection only:

```yaml
rx_driver: fast_gpio_rx
```

This automatically selects `fast_gpio_tx`. The previous explicit configuration remains valid:

```yaml
rx_driver: fast_gpio_rx
tx_driver: fast_gpio_tx
```

FastGPIO RX captures MOSI/SCK and publishes a frame-end bus marker. FastGPIO TX queues frames and attempts to drive MISO after a new RX bus marker is observed.

The experimental external-clock path is:

```yaml
rx_driver: external_clock_rx
```

This automatically selects `fast_gpio_tx`. `external_clock_rx` samples MOSI from the external SCK edge and emits signature-anchored frame chunks. It also publishes the same frame-end bus marker used by FastGPIO RX, so FastGPIO TX can respond from RX-observed bus timing instead of blindly polling for a transmit window.

RX-only validation remains available through an explicit override:

```yaml
rx_driver: external_clock_rx
tx_driver: none
```

The ESP32-S3 hardware-assisted path is:

```yaml
rx_driver: rmt_spi_rx
rmt_spi_frame_gap_us: 1000
```

This automatically selects `fast_gpio_tx`; an explicit `tx_driver: fast_gpio_tx` remains supported for existing configurations.

`rmt_spi_rx` observes SCK with RMT, treats the inter-frame idle gap as an internal chip-select boundary, and captures MOSI through the SPI slave peripheral using DMA. The application-facing handoff is intentionally limited to two complete frames and is aggressively drained into the existing latest-slot catalogue.

This path was investigated after reviewing [hberntsen/mhi-ac-ctrl-esp32](https://github.com/hberntsen/mhi-ac-ctrl-esp32), which demonstrates the key technique of deriving an internal SPI CS signal from RMT-observed clock gaps. Redux retains its own transport, synchronisation, catalogue, decoder, state, publishing, diagnostics, and command-confirmation architecture; only the transport technique informed this backend.

The current bus has no physical chip-select line, so normal ESP-IDF SPI slave assumptions do not fit directly. FastGPIO remains the stable baseline, while `rmt_spi_rx` is now the preferred experimental ESP32-S3 receive path.

An initial full-duplex transport is also available for ESP32-S3 development:

```yaml
rx_driver: rmt_cs_spi
rmt_spi_frame_gap_us: 1000
rx_worker: false
tx_worker: false
```

`rmt_cs_spi` gives one backend exclusive ownership of RMT, SPI2, SCK, MOSI, MISO, and the DMA transaction buffers. It captures MOSI and shifts MISO in the same mode-3, LSB-first SPI slave transaction. Because it owns both directions, configuring any `tx_driver` override with `rx_driver: rmt_cs_spi` is rejected. This path has compile coverage for 20-byte and 33-byte configurations but still requires hardware validation; the stable defaults remain unchanged.

Long-duration validation on the tested ESP32-S3 reached approximately:

```text
RMT/SPI completed frames:  3,421,459
Redux valid frames:        3,421,625
Invalid frames:            0
Checksum failures:         0
Signature misses:          0
Sync losses:               0
Completed-frame overwrite: 0
Completed-frame drops:     0
```

The same soak confirmed that the remaining transport risk is FastGPIO TX rather than RMT/SPI RX. Background TX recorded a small failure rate, while all tested command frames confirmed without timeout. The opdata catalogue also reported sustained slot-pressure, which now requires separate investigation. See the findings document for details.

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
```

Optional fields:

```text
tx_driver
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
rmt_spi_frame_gap_us
room_temp_timeout
external_temperature_sensor
fan_profile
```

## Worker mode

Worker mode is experimental and disabled by default.

Current recommendation:

```text
FastGPIO RX/TX:
  keep workers disabled

External-clock RX:
  keep workers disabled unless explicitly testing the worker path

RMT/SPI RX:
  keep workers disabled
```

The RMT/SPI backend already performs deterministic hardware capture. In initial testing, the no-worker path captured every 33-byte frame without checksum, synchronisation, queue, or RMT re-arm errors. Enabling both generic workers reduced main-loop blocking but introduced completed-frame overwrites/drops and FastGPIO TX marker misses.

The worker issue is not excessive bus throughput. The AC remains at roughly 20 frames per second. The problem is that the existing workers poll and yield instead of blocking on transport events. During scheduling gaps, the two-frame completed handoff can fill even though the main-loop path drains it reliably.

Observed worker shape:

```text
RX worker:
  tens of thousands of polling loops
  mostly idle yields
  completed-frame handoff can overwrite under contention

TX worker:
  tens of thousands of flush attempts
  very few actual sends
  bus-marker timing misses can occur
```

The future RTOS direction is event-driven:

```text
RMT/SPI completion notification
  -> drain completed DMA transactions
  -> validate/classify
  -> update latest catalogue slots
  -> notify main loop

TX command/boundary notification
  -> wake TX task only when work and a valid bus window exist
```

A worker experiment is only healthy if all of these remain true:

```text
valid_frames increases
checksum_failures stays at 0
signature_misses stays at 0
command confirmations work
command timeouts stay at 0
opdata continues to publish
RMT/SPI overwritten and dropped stay at 0
Home Assistant state matches the physical AC
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

Redux defaults to the four-speed profile. No `fan_profile` setting is required for normal use:

```yaml
MhiAcCtrl:
  id: mhi_ac
  # fan_profile defaults to four_speed
```

The default exposes:

- Auto
- Quiet
- Low
- Medium
- High

Mapping:

```text
MOSI 0 -> Quiet
MOSI 1 -> Low
MOSI 2 -> Medium
MOSI 6 -> High
MOSI 7 -> Auto
```

Hardware testing on both available AC models confirmed that protocol value `0` is accepted, returned in status, and distinct from Low. Quiet, Low, Medium, High, and Auto all completed command confirmation successfully.

For a model that genuinely does not support Quiet, explicitly select the compatibility profile:

```yaml
MhiAcCtrl:
  fan_profile: three_speed
```

The three-speed profile exposes Auto, Low, Medium, and High. It presents received protocol values `0` and `1` as Low, while outgoing Low commands continue to use protocol value `1`.


The selected profile controls climate traits, fan-select options, status publishing, TX command encoding, and command confirmation.

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

* Keep the default path conservative: `fast_gpio_rx` + `fast_gpio_tx`, with workers disabled.
* Continue long-duration validation of `rmt_spi_rx` on ESP32-S3, with `fast_gpio_tx` and workers disabled.
* Treat `rmt_spi_rx` as the preferred experimental S3 receive backend following the successful 47.5-hour soak test.
* Investigate and fix FastGPIO TX timing failures, missed bus markers, and main-loop blocking.
* Evaluate hardware-assisted SPI TX so RX and TX can share the same externally clocked transaction path.
* Replace polling RX/TX workers with event-driven RTOS tasks using SPI/RMT completion and command notifications.
* Investigate `opdata_slots_full`, including logging rejected opdata keys and reviewing catalogue slot allocation.
* Continue fan, mode, temperature, vane, and command-confirmation validation on the stable no-worker path.
* Continue hardening command confirmation around rapid and overlapping UI changes.


## Credits

This project builds on prior MHI reverse-engineering and ESPHome integration work.

- Upstream ESPHome project base: [ginkage/MHI-AC-Ctrl-ESPHome](https://github.com/ginkage/MHI-AC-Ctrl-ESPHome)
- Bus capture and trace reference: [absalom-muc/MHI-AC-Trace](https://github.com/absalom-muc/MHI-AC-Trace)
- FastGPIO inspiration/reference work: [RobertJansen1/MHI-AC-Ctrl-ESPHome esp32_errors branch](https://github.com/RobertJansen1/MHI-AC-Ctrl-ESPHome/tree/esp32_errors)
- Original reverse-engineering lineage and MHI protocol work from the wider `MHI-AC-Ctrl` community

## License

See `LICENSE`.