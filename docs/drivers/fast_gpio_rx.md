# `fast_gpio_rx`

`fast_gpio_rx` is the synchronous software GPIO receive backend. It is the conservative default and the internal recovery implementation used by hardware-assisted transports.

## Basic configuration

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: fast_gpio_rx
```

Effective TX is `fast_gpio_tx` unless `tx_driver: none` is explicitly selected for diagnostics.

## Supported targets

| Property | Value |
|---|---|
| Platform | ESP32 |
| Framework | ESP-IDF |
| Compile-supported variants | ESP32, ESP32-S2, ESP32-S3, ESP32-C2, ESP32-C3, ESP32-C5, ESP32-C6, ESP32-C61, ESP32-S31 |
| Runtime validation | ESP32 and ESP32-S3 baseline/recovery paths |
| Command-worker classified RX | No |
| Internal recovery layer | No; this is the recovery implementation |

Extended compile coverage is available across all listed Wi-Fi variants. Runtime timing and control remain hardware-validation requirements, especially on variants using the generic GPIO fallback.

## Driver tunables

```yaml
MhiAcCtrl:
  rx_driver: fast_gpio_rx

  fast_gpio_rx:
    frame_start_idle_ms: 10
```

| Option | Type and range | Default | Purpose |
|---|---|---:|---|
| `frame_start_idle_ms` | Integer, `1` to `50` ms | `10` ms | Requires SCK to remain idle-high for this period before the next low pulse is treated as the start of a fresh frame. The same resolved value is also used by `fast_gpio_tx` to align software TX with the next AC frame. |

The legacy top-level form is still accepted:

```yaml
MhiAcCtrl:
  rx_driver: fast_gpio_rx
  frame_start_idle_ms: 10
```

Do not configure `frame_start_idle_ms` both at component level and under `fast_gpio_rx`.

### When to tune it

Leave the default unchanged unless hardware traces or diagnostics show that the software path is entering a frame too early or failing to recognise the idle boundary.

- A larger value requires a longer idle-high interval before a new frame begins.
- A smaller value accepts a shorter idle interval.
- Excessive values can delay or time out frame acquisition.
- Values that are too small can make noise or intra-transaction gaps look like a new frame boundary.

## Behaviour

The backend waits for a clean idle-high SCK interval, then samples MOSI one clock edge at a time. Because it follows the external clock synchronously in software, it is sensitive to scheduler load, logging, Wi-Fi, API activity, and other long-running main-loop work.

Even with `command_worker: true`, this RX path remains main-loop driven. The command worker may still handle command coordination, but it does not move FastGPIO edge sampling into a background classified-RX path.

## Strengths

- Simple and broadly applicable baseline.
- Useful for initial hardware bring-up.
- Useful for comparing a hardware-assisted driver against known software behaviour.
- Used as the automatic recovery transport for hardware-assisted primaries.

## Trade-offs

- Highest CPU pressure of the available RX drivers.
- Can contribute to long loop sections.
- More sensitive to logging and publication load.
- Bit timing remains software-owned.

## Related pages

- [`fast_gpio_tx`](fast_gpio_tx.md)
- [Driver index](README.md)
- [`../../DIAGNOSTICS.md`](../../DIAGNOSTICS.md)
