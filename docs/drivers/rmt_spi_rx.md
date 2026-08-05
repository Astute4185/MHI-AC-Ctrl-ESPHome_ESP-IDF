# `rmt_spi_rx`

`rmt_spi_rx` is the ESP32-S3 hardware-assisted split RX backend. RMT detects the inter-frame SCK idle gap and drives an internal transaction boundary while the SPI slave peripheral captures MOSI.

## Basic configuration

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: rmt_spi_rx
  command_worker: true
```

Effective TX is `fast_gpio_tx` unless `tx_driver: none` is selected for diagnostics.

## Supported targets

| Property | Value |
|---|---|
| Platform | ESP32 |
| Framework | ESP-IDF |
| Variants | ESP32-S3 only |
| Hardware validation | ESP32-S3 split hardware RX |
| Required IDF component | `esp_driver_rmt` |
| Command-worker classified RX | Yes |
| Internal FastGPIO recovery | Yes |

## Driver tunables

```yaml
MhiAcCtrl:
  rx_driver: rmt_spi_rx

  rmt_spi_rx:
    frame_gap_us: 1000
```

| Option | Type and range | Default | Purpose |
|---|---|---:|---|
| `frame_gap_us` | Integer, `500` to `5000` microseconds | `1000` µs | Sets the maximum SCK pulse interval used by RMT when detecting the idle gap that terminates one frame and defines the next SPI transaction boundary. |

The legacy top-level alias remains accepted:

```yaml
MhiAcCtrl:
  rx_driver: rmt_spi_rx
  rmt_spi_frame_gap_us: 1000
```

Do not configure both `rmt_spi_frame_gap_us` and `rmt_spi_rx.frame_gap_us`.

### When to tune it

Leave the default at `1000` µs unless captures or driver diagnostics show incorrect frame-boundary detection.

- A larger value tolerates a longer pause inside activity before declaring a completed frame.
- A smaller value closes the transaction sooner after clock activity stops.
- A value that is too small can split a valid frame.
- A value that is too large can delay completion or combine activity that should be treated as separate transactions.

Tuning should be based on measured SCK timing, not on checksum failures alone.

## Behaviour

RMT observes SCK and identifies the bus idle gap. The SPI slave peripheral captures complete 20-byte or 33-byte MOSI transactions into the RX backend. The shared frame pipeline then validates, classifies, and decodes the copied data.

This backend remains RX-only. MISO is driven by `fast_gpio_tx` through the split transport wrapper.

With `command_worker: true`, queued RX frames can be synchronised, classified, and decoded in the worker.

## Strengths

- Hardware-assisted complete-frame RX.
- Clean 20-byte and 33-byte capture on the validated ESP32-S3 path.
- Reduces RX timing pressure compared with synchronous FastGPIO sampling.
- Retains the established FastGPIO command path.

## Trade-offs

- ESP32-S3 only.
- TX remains software-driven.
- Depends on an inferred internal transaction boundary because the MHI connector has no physical CS line.
- Hardware timing and pin routing still require board-level validation.

## RX-only diagnostic configuration

```yaml
MhiAcCtrl:
  rx_driver: rmt_spi_rx
  tx_driver: none
```

Use this only to isolate RX. It is not a usable Home Assistant control configuration.

## Related pages

- [`fast_gpio_tx`](fast_gpio_tx.md)
- [`rmt_cs_spi`](rmt_cs_spi.md)
- [`none`](none.md)
- [Driver index](README.md)
- [`../../DIAGNOSTICS.md`](../../DIAGNOSTICS.md)
