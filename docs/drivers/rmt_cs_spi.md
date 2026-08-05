# `rmt_cs_spi`

`rmt_cs_spi` is the integrated FIFO-backed full-duplex transport for the original dual-core ESP32 and ESP32-S3. RMT detects the inter-frame SCK idle gap, drives the SPI peripheral's internal CS signal, and the SPI slave peripheral captures MOSI while shifting MISO in the same transaction.

## Basic configuration

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: rmt_cs_spi
  command_worker: true
```

Do not configure `tx_driver`. This transport owns both RX and TX.

No physical CS pin is required.

## Supported targets

| Property | Value |
|---|---|
| Platform | ESP32 |
| Framework | ESP-IDF |
| Variants | ESP32, ESP32-S3 |
| Hardware validation | Original ESP32 and ESP32-S3 full-duplex paths |
| Required IDF component | `esp_driver_rmt` |
| Command-worker classified RX | Yes |
| Internal FastGPIO recovery | Yes |

## Driver tunables

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi

  rmt_cs_spi:
    frame_gap_us: 1000
```

| Option | Type and range | Default | Purpose |
|---|---|---:|---|
| `frame_gap_us` | Integer, `500` to `5000` microseconds | `1000` µs | Sets the maximum SCK pulse interval used by RMT to recognise the idle gap that ends one full-duplex SPI transaction and arms the next one. |

The legacy top-level alias remains accepted:

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
  rmt_spi_frame_gap_us: 1000
```

Do not configure both `rmt_spi_frame_gap_us` and `rmt_cs_spi.frame_gap_us`.

### When to tune it

Keep the default unless measured SCK timing or transport diagnostics show an incorrect transaction boundary.

- Increasing the value allows a longer pause before the transaction is considered complete.
- Decreasing the value completes the transaction sooner after clock activity stops.
- Too small can split a valid frame.
- Too large can delay re-arming or merge activity that should be separate.

Because this value controls both RX completion and TX transaction ownership, change it conservatively and re-test commands as well as receive health.

## Behaviour

The transport owns:

- SCK, MOSI, and MISO;
- RMT boundary detection;
- SPI2 slave setup;
- FIFO-backed RX and TX buffers;
- transaction queue and result handling;
- the dedicated SPI owner task;
- real TX-completion reporting.

The implementation always uses `SPI_DMA_DISABLED`. The supported 20-byte and 33-byte frames fit within the SPI slave FIFO working transaction, so DMA does not provide a frame-capacity advantage here.

A command becomes eligible for semantic confirmation only after the SPI owner task reports that the transaction completed.

## Target-specific behaviour

- Original ESP32 applies a transport-local mode-3 receive-edge correction after SPI initialisation.
- ESP32-S3 uses the standard ESP-IDF mode-3 configuration.

These differences remain inside the driver and do not affect the shared controller or protocol code.

## Invalid configurations

A separate TX backend is invalid:

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
  tx_driver: fast_gpio_tx
```

RX-only mode is also invalid for this integrated backend:

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
  tx_driver: none
```

## Strengths

- Hardware owns bit-level RX and TX timing.
- Removes FastGPIO TX from the normal transaction path.
- One public FIFO architecture across ESP32 and ESP32-S3.
- No DMA descriptor, channel, allocation, or alignment lifecycle.
- Real TX completion is tied to the completed SPI transaction.

## Trade-offs

- Requires compatible RMT and SPI slave peripherals.
- Boundary tuning affects the complete transaction, not RX alone.
- Hardware validation is still required for each board, pin set, and AC model.

## Related pages

- [`rmt_spi_rx`](rmt_spi_rx.md)
- [Driver index](README.md)
- [`../../DIAGNOSTICS.md`](../../DIAGNOSTICS.md)
