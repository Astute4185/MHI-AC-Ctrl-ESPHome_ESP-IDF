# SPI Transport Findings

## Scope

This document records the current findings for the two hardware-assisted SPI
transport architectures retained by the project:

- `rmt_spi_rx`: ESP32-S3 receive-only SPI capture paired with a separate TX driver;
- `rmt_cs_spi`: integrated FIFO-backed RX/TX transport for ESP32 and ESP32-S3.

The older worker experiments and pre-consolidation driver names are historical
context only. Current configuration uses `command_worker`; the temporary
`rmt_cs_spi_nodma` label has been removed.

## Protocol basis

The transport design follows the documented three-wire MHI bus:

```text
AC is master
SCK and MOSI are outputs from the AC
MISO is returned by the controller
no physical chip-select signal
CPOL=1, CPHA=1 (mode 3)
```

Representative SRK ZS-S timing is a 10 ms frame, 40 ms frame pause, 250 us
byte and inter-byte timing, and a 31.25 us bit period. Other units may differ.
The complete protocol baseline and project-specific extensions are consolidated
in [`FINDINGS_MHI_PROTOCOL.md`](FINDINGS_MHI_PROTOCOL.md).

## MHI bus constraints

The MHI bus exposes:

```text
SCK
MOSI
MISO
```

There is no physical chip-select signal. The AC is the clock master. Frame
boundaries must be inferred from the external clock idle gap and validated using
frame length, signature and checksum.

Representative timing from the tested units is:

```text
Frame duration:       about 10 ms
Inter-frame pause:    about 40 ms
Inter-byte pause:     about 250 us
Bit period:           about 31.25 us
Frame cadence:        about 20 frames/s
```

Both SPI transports use RMT observation of SCK to derive an internal transaction
boundary for the ESP-IDF SPI slave peripheral.

## Architecture comparison

| Property | `rmt_spi_rx` | `rmt_cs_spi` |
|---|---|---|
| Role | Receive-only | Integrated RX and TX |
| Target | ESP32-S3 | ESP32 and ESP32-S3 |
| SPI buffering | DMA-backed | FIFO-only, DMA disabled |
| TX path | Separate driver, normally `fast_gpio_tx` | Transport-owned |
| Boundary detection | RMT-derived internal CS | RMT-derived internal CS |
| Frame support | 20 and 33 bytes | 20 and 33 bytes |
| Worker use | Supports classified `command_worker` RX | Supports classified `command_worker` RX and TX lifecycle |
| Primary use | Split transport alternative | Preferred integrated hardware SPI path |

## Transport evolution findings

| Path | Primary finding | Current position |
|---|---|---|
| `fast_gpio_rx` + `fast_gpio_tx` | Functionally reliable but synchronous timing can consume substantial loop budget | Conservative fallback |
| `external_clock_rx` + `fast_gpio_tx` | RX pressure falls, but FastGPIO TX remains the dominant stall source | Experimental split path |
| `rmt_spi_rx` + `fast_gpio_tx` | Hardware RX remained clean over a long soak; TX still caused the remaining timing pressure | Retained ESP32-S3 split reference |
| `rmt_cs_spi` | Integrated RX/TX removes the synchronous FastGPIO TX bottleneck; FIFO capacity is sufficient | Preferred ESP32/ESP32-S3 SPI path |

The critical engineering lesson is that clean RX counters alone are not enough.
A transport is accepted only when command confirmation, opdata progression,
physical AC state, Home Assistant state, queues, and loop timing remain healthy
together.

## Shared design

The common transaction model is:

```text
RMT observes SCK
  -> inter-frame idle gap is detected
  -> an internal SPI CS boundary is generated
  -> one complete SPI slave transaction finishes
  -> the complete frame enters the project frame pipeline
```

The default frame-gap threshold used during validation is:

```yaml
MhiAcCtrl:
  rmt_spi_frame_gap_us: 1000
```

The threshold must remain above the normal inter-byte pause and below the
inter-frame pause for the target unit.

## `rmt_spi_rx`

### Architecture

`rmt_spi_rx` is an ESP32-S3 receive-only backend. It uses DMA-backed SPI capture
and must be paired with a separate TX driver when control is required.

Typical configuration:

```yaml
MhiAcCtrl:
  frame_size: 33
  rx_driver: rmt_spi_rx
  tx_driver: fast_gpio_tx
  command_worker: true
  rmt_spi_frame_gap_us: 1000
```

This split architecture isolates RX from the software GPIO capture cost, but the
separate FastGPIO TX path can still create long synchronous timing sections.

### Initial RX-only result

Representative ESP32-S3 result for 33-byte frames:

```text
RMT/SPI boundaries:       1202
SPI transactions:         1202
33-byte frames:           1202
Invalid transaction len:  0
SPI result errors:        0
SPI queue errors:         0
RMT re-arm errors:        0
Maximum buffered frames:  1
Overwritten frames:       0
Dropped frames:           0

Valid protocol frames:    1197
Invalid frames:           0
Checksum failures:        0
Signature misses:         0
Sync losses:              0
Dropped bytes:            0
```

Main-loop timing remained light during RX-only testing:

```text
loop average:    about 36 us
loop maximum:    about 2.5 ms
over-budget:     0
```

### Long-duration soak

The split `rmt_spi_rx` plus `fast_gpio_tx` configuration completed an
approximately 47.5-hour soak.

Representative end-of-soak RX counters:

```text
RMT/SPI completed frames:       3,421,459
33-byte frames:                 3,421,459
Invalid transaction lengths:    1
SPI result errors:              0
SPI queue errors:               0
RMT re-arm errors:              0
Completed-frame overwritten:    0
Completed-frame dropped:        0

Valid protocol frames:          3,421,625
Invalid frames:                 0
Checksum failures:              0
Signature misses:               0
Sync losses:                    0
Dropped bytes:                  0
```

This established the DMA-backed RX path as a clean hardware receive reference on
the tested ESP32-S3 hardware.

### Split-TX limitation

The same soak recorded:

```text
TX frames:                      340,345
TX failures:                    389
TX command frames:              8
TX command failures:            0
Command confirmations:          8
Command confirmation timeouts:  0
```

The command path remained reliable, but synchronous FastGPIO TX remained the
main source of loop-budget pressure. The near one-to-one relationship between TX
frames and loop overruns indicated that RX was no longer the dominant timing
risk.

### Current role

Retain `rmt_spi_rx` when an ESP32-S3 split transport is specifically required or
when comparison against the long-soak DMA receive reference is useful.

It is not the preferred full-duplex configuration because TX remains owned by a
separate driver.

## `rmt_cs_spi`

### Architecture

`rmt_cs_spi` is the integrated hardware SPI transport. It owns:

- RMT boundary detection;
- SPI slave configuration;
- complete-frame RX;
- MISO TX preparation and transmission;
- actual TX completion reporting;
- transport queues and driver diagnostics.

A separate `tx_driver` must not be configured.

Typical configuration:

```yaml
MhiAcCtrl:
  frame_size: 33
  rx_driver: rmt_cs_spi
  command_worker: true
  rmt_spi_frame_gap_us: 1000
```

### FIFO-only decision

The largest supported frame is 33 bytes. DMA is not required for transactions of
this size, so the integrated driver now always uses:

```text
SPI_DMA_DISABLED
internal SPI FIFO buffers
one shared implementation for ESP32 and ESP32-S3
```

Removing DMA eliminated:

- DMA-capable buffer allocation;
- DMA alignment requirements;
- DMA descriptor lifecycle;
- original-ESP32 DMA reset and CS-management complexity;
- separate DMA and non-DMA public driver labels.

The temporary `rmt_cs_spi_nodma` label was removed. Its validated FIFO behaviour
is now the sole implementation behind `rmt_cs_spi`.

### Original ESP32 mode-3 correction

The first original-ESP32 FIFO test produced correctly bounded transactions but
invalid MOSI bytes. Boundary detection, queueing and transaction length were
working; the issue was isolated to the original ESP32 non-DMA mode-3 receive
edge.

The original ESP32 path applies this internal correction after SPI
initialisation:

```text
ck_idle_edge = 1
ck_i_edge    = 1
```

ESP32-S3 uses the normal ESP-IDF mode-3 configuration.

### Initial original-ESP32 result

Representative result after the edge correction:

```text
Valid protocol frames:          1,199
Invalid frames:                 0
Checksum failures:              0
Signature misses:               0
Sync losses:                    0
Dropped bytes:                  0

TX frames:                      239
TX failures:                    0
TX command frames:              4
Command confirmations:          6
Command confirmation timeouts:  0
Retries:                        0
Retry exhaustions:              0

Completed transactions:         1,200
33-byte frames:                 1,200
Invalid transaction lengths:    2
SPI result errors:              0
SPI queue errors:               0
RMT re-arm errors:              0
RX overwritten:                 0
Dropped frames:                 0

Loop average:                   48 us
Loop maximum:                   4,622 us
Loop over-budget:               0
```

Power, mode and temperature commands were transmitted and confirmed from
returned MOSI state. Opdata continued to advance.

### Extended-louver validation

The FIFO-backed ESP32-S3 transport remained clean during the complete 80-case
vertical-vane, horizontal-vane and 3D Auto hardware matrix. The matrix completed
80/80 combinations with no retry exhaustion and no pending confirmation left at
completion.

The associated transport counters showed no protocol corruption, checksum
failure, queue error, RMT re-arm error, RX overwrite or completion drop.

Detailed command findings are recorded in
[`FINDINGS_LOUVERS_3D_AUTO.md`](FINDINGS_LOUVERS_3D_AUTO.md).

### Current role

Use `rmt_cs_spi` on supported ESP32 or ESP32-S3 hardware when an integrated
hardware-assisted full-duplex transport is required.

It is the preferred SPI architecture because it removes the synchronous
FastGPIO TX bottleneck and gives one transport clear ownership of RX, TX and
actual transmission completion.

## Queue and catalogue findings

The project deliberately separates hardware readiness from application state:

```text
SPI transaction queue
  keeps hardware transactions armed

Completed-frame handoff
  short bounded scheduling bridge

Frame catalogue
  latest status slot
  latest extended-status slot
  keyed latest opdata slots
  command-candidate slot
```

A large general-purpose frame FIFO is not the desired solution. Repeated status
frames should collapse into latest-value slots, while complete frames must never
be reduced to partial byte streams.

## Worker findings

The early generic `rx_worker` and `tx_worker` experiment improved main-loop
numbers but introduced frame overwrites, drops and excessive polling. That model
was superseded.

The current architecture uses one `command_worker` and event-driven transport
ownership:

```text
transport task
  owns real-time hardware activity

command worker
  drains queue-backed RX
  classifies and decodes frames
  coordinates commands and confirmation

ESPHome main loop
  applies decoded snapshots
  publishes entities
```

See [`../ARCHITECTURE.md`](../ARCHITECTURE.md) for the current ownership model.

## Current recommendation

| Requirement | Recommendation |
|---|---|
| Integrated ESP32/ESP32-S3 hardware SPI | `rmt_cs_spi` |
| ESP32-S3 receive-only or split comparison path | `rmt_spi_rx` + suitable TX driver |
| Conservative fallback | `fast_gpio_rx` + `fast_gpio_tx` |
| RX timing experiment where supported | `external_clock_rx` + suitable TX driver |

Driver selection must also account for the actual board, target, frame size and
hardware validation status. See [`../DRIVER_SELECTION.md`](../DRIVER_SELECTION.md).

## Acceptance criteria

For either SPI backend, hardware validation should review:

```text
invalid transaction lengths
SPI result errors
SPI queue errors
RMT re-arm errors
completed-frame overwrites and drops
checksum failures
signature misses
sync losses
TX failures
TX completion drops
command confirmation timeouts
retries and retry exhaustions
pending confirmation mask
opdata progression
Home Assistant state versus physical AC state
```

An isolated startup `invalid_len` event may be acceptable when it does not grow
and all complete protocol frames remain valid. Sustained growth or any command,
queue or protocol corruption requires investigation.

## Conclusion

The hardware evidence supports two distinct conclusions:

1. `rmt_spi_rx` provides a clean ESP32-S3 DMA-backed receive reference, but a
   separate FastGPIO TX path remains a timing bottleneck.
2. `rmt_cs_spi` provides the cleaner integrated architecture. FIFO capacity is
   sufficient for 20-byte and 33-byte MHI frames, allowing one non-DMA
   implementation to support both ESP32 and ESP32-S3.

The project should retain both architectures only while each has a clear use
case. New full-duplex SPI development should target `rmt_cs_spi`.
