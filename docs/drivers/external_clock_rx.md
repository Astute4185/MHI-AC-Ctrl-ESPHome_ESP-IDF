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
  command_worker: true
```

Effective TX is `fast_gpio_tx` unless `tx_driver: none` is selected for RX-only diagnostics.

## Supported targets

| Property | Value |
|---|---|
| Platform | ESP32 |
| Framework | ESP-IDF |
| Compile-supported variants | ESP32, ESP32-S2, ESP32-S3, ESP32-C2, ESP32-C3, ESP32-C5, ESP32-C6, ESP32-C61, ESP32-S31 |
| Hardware validation | Limited; compile support on a target is not a runtime claim |
| Command-worker classified RX | Yes |
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

With `command_worker: true`, the queue-backed RX path can be drained and classified outside the main ESPHome loop. ESPHome state publication still remains in the main loop.

TX remains software-driven through `fast_gpio_tx`.

## Strengths

- Lower RX pressure than the synchronous FastGPIO path in tested original-ESP32 runs.
- Preserves the established split TX implementation.
- Supports classified RX in the command worker.
- Useful when an integrated SPI path is unavailable or being isolated.

## Trade-offs

- TX still follows the AC clock in software.
- Interrupt timing and GPIO behaviour remain target-specific.
- Runtime behaviour remains target-specific and must be validated on physical hardware for each new variant.

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
