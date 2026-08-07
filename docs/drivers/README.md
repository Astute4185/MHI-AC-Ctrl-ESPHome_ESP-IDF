# MHI Transport Drivers

This directory is the canonical configuration guide for MHI transport drivers.

The transport layer is modular: each backend owns its hardware setup, target restrictions, optional tuning, queues, timing, and diagnostics. The protocol decoder, command coordinator, state model, entities, recovery policy, and `MhiAcCtrl` controller continue to use the same common transport interfaces. Changing or adding a driver therefore does not require transport-specific branches throughout the codebase.

## Basic configuration

The conservative baseline is `fast_gpio_rx` with the automatically selected `fast_gpio_tx` backend:

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: fast_gpio_rx
```

For split RX drivers, `tx_driver` normally does not need to be specified. The component resolves it to `fast_gpio_tx` automatically.

To select another backend, change only `rx_driver`:

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: rmt_cs_spi
```

## Calling a driver tunable

Driver-specific tunables are nested under a block with the same name as the selected `rx_driver`:

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: rmt_cs_spi

  rmt_cs_spi:
    frame_gap_us: 1000
```

The nested block is optional. Leave it out when the documented default is suitable.

Only the selected driver's block may be present. For example, this is invalid because `rmt_spi_rx` is not selected:

```yaml
MhiAcCtrl:
  rx_driver: fast_gpio_rx

  rmt_spi_rx:
    frame_gap_us: 1000
```

Older top-level tuning aliases remain accepted for compatibility, but new configurations should use the driver-specific nested form. Do not configure the same option in both locations.

## Driver summary

| Selection | Effective TX | Supported ESP32 variants | Driver tunables | Position |
|---|---|---|---|---|
| [`fast_gpio_rx`](fast_gpio_rx.md) | [`fast_gpio_tx`](fast_gpio_tx.md) where supported, otherwise `none` | All Wi-Fi ESP32 variants at compile time | `frame_start_idle_ms` | Validated compatibility baseline and recovery implementation |
| [`external_clock_rx`](external_clock_rx.md) | [`fast_gpio_tx`](fast_gpio_tx.md) where supported, otherwise `none` | All Wi-Fi ESP32 variants at compile time | None | Experimental; intermittent RX corruption reproduced on ESP32 and ESP32-S3 |
| [`rmt_spi_rx`](rmt_spi_rx.md) | [`fast_gpio_tx`](fast_gpio_tx.md) | ESP32-S3 | `frame_gap_us` | Validated hardware-assisted split RX; no worker recommended |
| [`rmt_cs_spi`](rmt_cs_spi.md) | Integrated | ESP32, ESP32-S3 | `frame_gap_us` | Preferred validated FIFO-backed full-duplex transport |
| [`none`](none.md) | Disabled | Split RX drivers only | None | RX-only diagnostics; not a normal control configuration |

All current drivers require the ESP-IDF framework. Target support is validated during ESPHome configuration.

The portable RX paths have extended compile coverage across ESP32, S2, S3, C2, C3, C5, C6, C61, and S31. This is source/toolchain coverage, not a hardware-support claim. See [`../../TRANSPORT_COMPILE_MATRIX.md`](../../TRANSPORT_COMPILE_MATRIX.md).

## Selection rules

### Split drivers

These RX drivers automatically use `fast_gpio_tx`:

- `fast_gpio_rx`
- `external_clock_rx`
- `rmt_spi_rx`

The explicit equivalent is still valid:

```yaml
MhiAcCtrl:
  rx_driver: rmt_spi_rx
  tx_driver: fast_gpio_tx
```

### Full-duplex driver

`rmt_cs_spi` owns both RX and TX. Do not configure `tx_driver` with it:

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
```

There is no physical CS pin. RMT derives the transaction boundary from the idle gap on SCK and drives the SPI peripheral's internal CS signal.

### Internal recovery

Hardware-assisted primary drivers automatically include an internal FastGPIO recovery transport using the same pins. Users do not configure a separate fallback driver.

Recovery is included for:

- `external_clock_rx`
- `rmt_spi_rx`
- `rmt_cs_spi`

`fast_gpio_rx` is already the recovery implementation and therefore does not add another fallback layer.

### Command worker

`command_worker` is a shared component option rather than a driver tunable:

```yaml
MhiAcCtrl:
  rx_driver: rmt_cs_spi
  command_worker: true
```

Queue-backed transports can move frame draining, classification, and decode into the command worker, but hardware validation shows that this is not equally suitable for every backend.

- `rmt_cs_spi`: worker mode is validated and supported.
- `rmt_spi_rx`: keep the worker disabled for normal use because the split FastGPIO TX path becomes less reliable under worker polling.
- `external_clock_rx`: keep the worker disabled during engineering use; the backend is experimental because RX corruption occurs independently of worker mode.
- `fast_gpio_rx`: remains synchronous and main-loop driven even when the command worker is enabled.

Low-level worker and TX scheduling options are advanced shared settings. They are not driver-specific and should normally remain at their validated defaults.

## Driver pages

- [`fast_gpio_rx`](fast_gpio_rx.md) — synchronous software GPIO RX baseline
- [`external_clock_rx`](external_clock_rx.md) — interrupt-driven external-clock RX
- [`rmt_spi_rx`](rmt_spi_rx.md) — ESP32-S3 RMT boundary detection plus SPI RX
- [`rmt_cs_spi`](rmt_cs_spi.md) — integrated FIFO-backed RMT/SPI RX and TX
- [`fast_gpio_tx`](fast_gpio_tx.md) — software TX used by split RX drivers
- [`none`](none.md) — RX-only diagnostic TX selection
- [Developing a new driver](developing-drivers.md) — interfaces, registration, compile guards, tests, and hardware evidence

For runtime counters, recovery interpretation, and soak-test requirements, see [`../../DIAGNOSTICS.md`](../../DIAGNOSTICS.md).
