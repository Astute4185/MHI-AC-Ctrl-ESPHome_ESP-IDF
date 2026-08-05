# `tx_driver: none`

`none` disables TX for supported split RX drivers. It is an RX-only diagnostic selection, not a normal climate-control configuration.

## Configuration

```yaml
MhiAcCtrl:
  id: mhi_ac
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: external_clock_rx
  tx_driver: none
```

It can also be used with `rmt_spi_rx` and other split RX drivers where target validation permits the RX backend.

It cannot be used with `rmt_cs_spi`, because that driver owns RX and TX as one integrated transport.

## What continues to work

- Passive RX capture.
- Frame synchronisation and classification.
- Status publication from frames sent by the AC.
- Transport and protocol diagnostics.

## What does not work

- Climate commands.
- Fan, vane, and 3D Auto commands.
- Operation-data requests that require transmitted frames.
- Command confirmation testing.

## Driver tunables

`none` has no tunables and no nested YAML block.

## When to use it

Use it briefly to determine whether an instability is caused by RX capture or by TX activity. Do not use it as a permanent Home Assistant configuration.

## Related pages

- [`external_clock_rx`](external_clock_rx.md)
- [`rmt_spi_rx`](rmt_spi_rx.md)
- [Driver index](README.md)
- [`../../DIAGNOSTICS.md`](../../DIAGNOSTICS.md)
