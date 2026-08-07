# `external_clock_rx`

`external_clock_rx` is an interrupt-driven split RX backend for the MHI bus. It samples MOSI from the AC-provided SCK edges, reconstructs LSB-first bytes, and queues signature-anchored frame chunks for the shared synchroniser and classifier.

## Basic configuration

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: external_clock_rx
  command_worker: false
```

Effective TX is `fast_gpio_tx` unless `tx_driver: none` is selected for RX-only diagnostics.

## Supported targets

| Property | Value |
|---|---|
| Platform | ESP32 |
| Framework | ESP-IDF |
| Compile-supported variants | ESP32, ESP32-S2, ESP32-S3, ESP32-C2, ESP32-C3, ESP32-C5, ESP32-C6, ESP32-C61, ESP32-S31 |
| Hardware validation | **Experimental**; intermittent RX corruption reproduced on original ESP32 and ESP32-S3 |
| Command-worker classified RX | Available, but not recommended for normal use |
| Internal FastGPIO recovery | Yes |

## Driver tunables

`external_clock_rx` currently exposes no driver-specific tunables.

Use the driver without a nested block:

```yaml
MhiAcCtrl:
  rx_driver: external_clock_rx
```

Do not add tuning fields from another backend. For example, `frame_gap_us` belongs to the RMT-based drivers and is not accepted here.

## Behaviour

The backend reacts to the external SCK signal rather than polling it synchronously. Completed bytes and frame chunks are copied into bounded queues, allowing the common RX runtime to synchronise, classify, and decode them.

The queue-backed RX path can technically be drained and classified by `command_worker`, but current hardware evidence does not recommend that configuration. ESPHome state publication remains in the main loop.

TX remains software-driven through `fast_gpio_tx`. On both tested ESP32 families, worker mode increased pressure on this timing-sensitive TX path while leaving the separate RX-integrity problem unchanged.

## Current validation result

Hardware testing on both original ESP32 and ESP32-S3 found the same failure signature: occasional invalid/checksum frames followed by signature misses, synchronisation loss, and one-frame recovery drops. The backend continues receiving afterward, but the RX stream is not protocol-clean.

A long original-ESP32 worker run exceeded 700,000 valid frames and demonstrated that the backend can operate continuously, but it still accumulated RX corruption and substantially more FastGPIO TX misses. This is therefore a functional engineering backend, not a release-validated transport.

## Strengths

- Much lower average main-loop RX pressure than synchronous FastGPIO sampling.
- Preserves the established split TX implementation.
- Useful for isolating interrupt-driven external-clock capture behaviour.

## Trade-offs

- **Known intermittent RX-integrity failure on both tested ESP32 and ESP32-S3 hardware.**
- TX still follows the AC clock in software.
- Worker mode increases FastGPIO TX contention and does not solve the RX corruption.
- Interrupt timing and GPIO behaviour remain target-specific.

Until the frame-boundary/edge handling issue is resolved, do not treat `external_clock_rx` as a stable or recommended runtime transport.

## RX-only diagnostic configuration

```yaml
MhiAcCtrl:
  rx_driver: external_clock_rx
  tx_driver: none
```

This can isolate RX capture and decode. It disables climate commands, vane commands, 3D Auto changes, and operation-data requests that require TX.

## Related pages

- [`fast_gpio_tx`](fast_gpio_tx.md)
- [`none`](none.md)
- [Driver index](README.md)
- [`../../DIAGNOSTICS.md`](../../DIAGNOSTICS.md)
