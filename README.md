# MHI-AC-Ctrl-ESPHome — ESP-IDF Branch

This branch is an **ESP-IDF-focused development branch** of [ginkage/MHI-AC-Ctrl-ESPHome](https://github.com/ginkage/MHI-AC-Ctrl-ESPHome), built to keep the original ESPHome integration model while improving transport reliability, diagnostics, and maintainability for Mitsubishi Heavy Industries air conditioners.

It is based on the upstream ESPHome component structure and native codegen approach already used in the main project, and it keeps the same broad goal: simple Home Assistant integration with OTA, API support, and no MQTT requirement for core control.

This branch also uses frame and bus-capture information from [absalom-muc/MHI-AC-Trace](https://github.com/absalom-muc/MHI-AC-Trace) to guide transport handling, frame validation, and diagnostics work.

## What this branch is

This is **not** a clean-room rewrite. It is a branch off the upstream project with targeted engineering work in four areas:

1. **ESP-IDF-first operation**
2. **MHI bus transport and frame-handling improvements**
3. **better logging and diagnostics for bad frames**
4. **core refactoring to make future maintenance safer and smaller in scope**

The current branch has been compiled and deployed in the provided test environment on **ESPHome 2026.3.3** using **ESP-IDF 5.5.3** on an **ESP32-S3** target.

## Key branch additions

### 1. ESP-IDF-oriented transport work

This branch is focused on running cleanly on ESP-IDF builds rather than carrying Arduino-era assumptions forward.

Current work in this area includes:
- transport configuration passed from YAML into the component at runtime
- frame-size hinting for 20-byte and 33-byte units
- pin configuration support for `sck`, `mosi`, and `miso`
- branch work toward simplifying transport ownership and reducing backend-switching complexity

### 2. Improved diagnostics and logging

The branch adds or is actively standardising more useful diagnostics around frame failures.

Goals of this work:
- keep the periodic summary line as the main health signal
- keep detailed bad-frame logs when a real failure occurs
- suppress or replace low-value numeric-only errors such as `mhi_ac_ctrl_core loop error: -2`
- make timeout, signature, and checksum failures easier to understand

Examples of the diagnostic direction for this branch:
- periodic `mhi.diag` summary counters
- explicit logging for extended checksum, base checksum, signature, and timeout failure classes
- branch work to map numeric loop errors to named diagnostic reasons

### 3. Modular core refactor

The original core flow was difficult to patch safely because transmit build, receive/validate, status decode, opdata decode, and state management were all tightly packed into the same hot path.

This branch has been progressively refactored so the pipeline is easier to reason about and easier to patch in isolation.

Current refactor direction:
- `mhi_frame_layout.h` for frame offsets and checksum helpers
- `mhi_core_state.h` for runtime/cache state
- `mhi_tx_builder.*` for outbound frame construction
- `mhi_rx_validator.*` for transport exchange and frame validation
- `mhi_status_decoder.*` for standard status decode
- `mhi_opdata_decoder.*` for `DB9` opdata / error-opdata handling
- optional helper layers such as `mhi_publish.*` and `mhi_core_reset.cpp` to reduce repeated boilerplate and keep the hot path small

### 4. Reliability work for 33-byte units

Upstream already supports frame-size selection, and this branch continues that direction with more emphasis on diagnosing and stabilising 33-byte operation.

The objective is not to hide failures. The objective is to:
- make frame problems visible
- keep logs readable
- make it easier to determine whether an issue is transport timing, capture integrity, checksum mismatch, or a unit/frame-size compatibility problem

## Features retained from upstream

This branch still builds on the upstream component feature set, including:
- Home Assistant / ESPHome integration without MQTT for core control
- climate entity support
- up/down vane control
- left/right vane support on units using larger frames
- external room temperature support
- low-temperature heating/cooling configuration support
- quiet / diffuse fan related upstream work
- YAML-based installation flow using example configs

## Hardware and target notes

This branch has been exercised on ESP32-S3 hardware in the provided logs.

From the supplied compile and runtime output:
- board target was `esp32-s3-devkitc-1`
- framework was ESP-IDF
- compile and OTA deployment completed successfully
- runtime logs show sustained successful frame processing with periodic `mhi.diag` summaries

That does **not** mean every board and every MHI indoor unit will behave the same. Older units may still require `frame_size: 20`, and newer units may benefit from `frame_size: 33` when the transport path is stable.

## Getting started

Create a new ESPHome device and combine your YAML with one of the examples from the `examples` directory in the upstream project.

At minimum you should configure:
- device name
- Wi-Fi
- OTA
- API key
- MHI pins (`sck`, `mosi`, `miso`) if your hardware requires them
- `frame_size` appropriate for your unit

Recommended starting points from upstream:
- `simple.yaml`
- `external_sensor.yaml`
- `full.yaml`
- `simple-energy-management.yaml`

## Branch-specific notes

### Frame size

If you are seeing repeated checksum or loop failures on a unit that does not support the larger frame layout, set:

```yaml
frame_size: 20
```

If your unit supports the larger frame layout and left/right vane or 3D auto features are required, test with:

```yaml
frame_size: 33
```

### Transport backend

The public backend selector is intentionally limited to one optional field:

```yaml
transport_backend: gpio
```

or:

```yaml
transport_backend: lcd_cam_rx
```

If omitted, the component uses `gpio`. Backend-specific recovery, gating, and raw capture diagnostics are internal implementation details and are not YAML tuning knobs.

### Logging

This branch is actively improving diagnostics. Expect log formats around transport and frame validation to evolve as the diagnostics layer is cleaned up.

The long-term intent is:
- concise summaries during normal operation
- named warnings for known failure classes
- less spam from redundant platform-level numeric errors

### Refactor status

This branch contains ongoing codebase modularisation work. The intent is to make future fixes smaller, more reviewable, and less likely to regress unrelated behaviour.

## FAQ

### I am getting checksum/signature failures or legacy `mhi_ac_ctrl_core loop error: -2`

In this branch, the most useful diagnostics are the `mhi.diag` warnings such as:
- `extended_checksum_fail`
- `base_checksum_fail`
- `invalid_signature`
- `short_capture`
- `timeout_low`
- `timeout_high`
- `timeout_other`

A raw `mhi_ac_ctrl_core loop error: -2` is a legacy/secondary symptom and should be read together with the surrounding `mhi.diag` output.

The usual causes are:
- the configured `frame_size` is wrong for the unit (`20` vs `33`)
- the transport path is unstable
- a bad frame was captured and rejected by signature/checksum validation

The first thing to verify is whether the unit should be using `frame_size: 20` or `frame_size: 33`.

### Why is this branch different from upstream?

Because the priority here is engineering work around:
- ESP-IDF compatibility
- transport stability
- more useful diagnostics
- making the core easier to maintain and patch

### Is this upstream?

No. This is a development branch built on top of the upstream project.

## Credits

- Upstream ESPHome project base: [ginkage/MHI-AC-Ctrl-ESPHome](https://github.com/ginkage/MHI-AC-Ctrl-ESPHome)
- Bus capture and trace reference: [absalom-muc/MHI-AC-Trace](https://github.com/absalom-muc/MHI-AC-Trace)
- Original reverse-engineering lineage and MHI protocol work from the wider community around `MHI-AC-Ctrl`

## License

This branch inherits the upstream licensing model. See the repository `LICENSE` file for details.
