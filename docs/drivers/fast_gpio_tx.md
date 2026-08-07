# `fast_gpio_tx`

`fast_gpio_tx` is the software GPIO transmit backend used by all split RX selections.

It is normally selected automatically and does not need its own YAML block.

## Automatic selection

Each split RX driver resolves to `fast_gpio_tx`:

```yaml
MhiAcCtrl:
  rx_driver: external_clock_rx
```

The explicit equivalent is:

```yaml
MhiAcCtrl:
  rx_driver: external_clock_rx
  tx_driver: fast_gpio_tx
```

Automatic selection is preferred unless a test needs to make the effective TX path explicit.

## Compatible RX drivers

- `fast_gpio_rx`
- `external_clock_rx`
- `rmt_spi_rx`

It must not be combined with `rmt_cs_spi`, because that backend owns RX and TX as one transaction.

## Driver tunables

There is currently no `fast_gpio_tx:` nested configuration block.

For split transports, TX uses the resolved `frame_start_idle_ms` value. That value is exposed under `fast_gpio_rx` because FastGPIO RX and TX share the same idle-high frame-alignment rule:

```yaml
MhiAcCtrl:
  rx_driver: fast_gpio_rx

  fast_gpio_rx:
    frame_start_idle_ms: 10
```

When another split RX backend is selected, the default FastGPIO TX idle value remains `10` ms. The legacy top-level `frame_start_idle_ms` setting can still override it, but new driver-specific tuning should only use documented nested fields.

## Related shared options

These options affect command scheduling but are not `fast_gpio_tx` driver tunables:

- `tx_background_interval_ms`
- `command_worker`
- `command_worker_start_delay_ms`
- `command_worker_stack_size`
- `command_worker_priority`
- `command_worker_core_id`

They should normally remain at their defaults. Change them only for targeted diagnostics or measured scheduling constraints.

## Behaviour

The backend waits for SCK to remain idle-high long enough to align with the next AC frame, then drives MISO one externally clocked bit at a time. It reports real completion through the shared TX contract so command confirmation does not begin merely because a frame was staged.

## Strengths

- Established command path.
- Reused across all split RX configurations.
- Supports the common completion, confirmation, retry, and supersession lifecycle.

## Trade-offs

- Bit timing is software-owned.
- Can create long main-loop sections.
- Background transmissions may be more sensitive to scheduler load than user commands.

## Related pages

- [`fast_gpio_rx`](fast_gpio_rx.md)
- [`external_clock_rx`](external_clock_rx.md)
- [`rmt_spi_rx`](rmt_spi_rx.md)
- [Driver index](README.md)
