# RMT-Generated-CS SPI Transport Findings

## Scope

This document records the implementation, hardware validation, worker experiments, and long-duration soak findings for the Redux RMT-generated-CS SPI transport.

The work progressed through two related implementations:

- the original ESP32-S3 `rmt_spi_rx` receive backend, using DMA-backed SPI capture with `fast_gpio_tx`;
- the integrated `rmt_cs_spi` full-duplex transport, initially proven on ESP32-S3 and subsequently consolidated onto the SPI CPU FIFO for both the original dual-core ESP32 and ESP32-S3.

The transport combines:

- RMT observation of the external SCK signal;
- inter-frame idle detection to synthesise an internal SPI chip-select boundary;
- ESP-IDF SPI slave capture in mode 3, LSB-first;
- complete-frame hardware transactions through the SPI CPU FIFO;
- the existing Redux frame synchroniser, latest-slot catalogue, decoders, state, publishing, diagnostics, command coordination, and confirmation paths.

The original ESP32-S3 DMA RX-only path completed a roughly 47.5-hour soak with clean protocol health. The FIFO-backed full-duplex path then passed hardware validation on the original dual-core ESP32 and ESP32-S3, establishing that DMA is not required for the integrated transport.

## Acknowledgement and design source

The transport path was investigated after reviewing:

- [hberntsen/mhi-ac-ctrl-esp32](https://github.com/hberntsen/mhi-ac-ctrl-esp32)

That repository demonstrates the key no-physical-CS technique used here: RMT observes the MHI clock and derives an internal SPI CS transition from the inter-frame idle gap, allowing the ESP32 hardware SPI peripheral to capture the bus.

Redux does not copy that repository's full application architecture. The Redux implementation keeps its own split RX/TX driver interfaces, frame synchronisation, catalogue, protocol decoding, command confirmation, publishing, diagnostics, tests, and ESPHome integration. The external repository provided the driver direction and evidence that the RMT-plus-SPI approach was viable.

The broader protocol and community lineage still includes the original MHI-AC-Ctrl and ESPHome implementations.

## Why this path was selected

The MHI bus exposes:

```text
SCK
MOSI
MISO
```

There is no physical chip-select line. The AC is the clock master. Representative timing is approximately:

```text
Frame duration:       10 ms
Inter-frame pause:    40 ms
Inter-byte pause:     250 us
Bit period:           31.25 us
Frame cadence:        20 frames/s
```

Software GPIO capture works, but it consumes CPU during every clock edge and remains sensitive to scheduling, logging, Wi-Fi, API, and TX load. A normal SPI slave setup cannot delimit transactions without CS.

The RMT/SPI design closes that gap:

```text
RMT detects SCK idle gap
  -> toggles internal SPI CS through the GPIO matrix
  -> SPI slave completes one hardware transaction
  -> Redux receives one complete 20-byte or 33-byte frame
```

The initial frame-gap setting is:

```yaml
rmt_spi_frame_gap_us: 1000
```

This is comfortably above the normal inter-byte pause and well below the inter-frame pause on the tested unit.

## Implementation boundaries

The backend is intentionally isolated behind the existing RX-driver abstraction. No significant decoder, state, publisher, climate, sensor, select, switch, or command-confirmation restructure was required.

Initial configuration:

```yaml
rx_driver: rmt_spi_rx
tx_driver: none
rx_worker: false
tx_worker: false
rmt_spi_frame_gap_us: 1000
```

Hybrid validation configuration:

```yaml
rx_driver: rmt_spi_rx
tx_driver: fast_gpio_tx
rx_worker: false
tx_worker: false
rmt_spi_frame_gap_us: 1000
```

The backend keeps two distinct queue concepts:

```text
SPI transaction queue depth: 4
  Required to keep DMA transactions armed.

Completed application-frame depth: 2
  Short handoff only; oldest complete frame is overwritten on overflow.
```

The application queue is not intended as a history FIFO. The component drains available frames aggressively into the existing latest-slot frame catalogue.

The integrated full-duplex transport exposes one driver selection:

```yaml
rx_driver: rmt_cs_spi
```

The driver owns RX and TX. A separate `tx_driver` is not configured. The same `sck_pin`, `mosi_pin`, `miso_pin`, and `rmt_spi_frame_gap_us` configuration is used on ESP32 and ESP32-S3.

## Initial RX-only hardware result

Target:

```text
ESP32-S3 rev 0.2
ESP-IDF 5.5.3
33-byte frames
SCK=8 MOSI=38 MISO=39
```

Representative result after approximately one minute:

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

Redux valid frames:       1197
Invalid frames:           0
Checksum failures:        0
Signature misses:         0
Sync losses:              0
Dropped bytes:            0
```

Main-loop timing remained light:

```text
loop average:    about 36 us
loop maximum:    about 2.5 ms
over-budget:     0
```

Conclusion:

```text
The RMT-generated-CS SPI/DMA RX path captures the tested 33-byte bus cleanly and with low main-loop cost.
```

## Hybrid RX plus FastGPIO TX result

With `tx_driver: fast_gpio_tx` and both workers disabled:

```text
RX remained checksum-clean.
Opdata frames flowed.
Mode, power, temperature, and fan commands confirmed.
RMT/SPI maximum buffered depth remained 1.
RMT/SPI overwritten and dropped remained 0.
```

Representative protocol state:

```text
valid_frames:       1797
invalid_frames:     0
checksum_failures:  0
signature_misses:   0
sync_losses:        0
TX command frames:  1+
command timeouts:   0
opdata frames:      increasing
```

The remaining performance issue is FastGPIO TX. Main-loop transport spikes still reached roughly 49-51 ms because the software TX path blocks while following the external clock.

Conclusion:

```text
The new backend removes RX as the dominant timing risk. FastGPIO TX becomes the remaining transport bottleneck.
```

## Long-duration soak result

The no-worker hybrid configuration was then left running for approximately 47.5 hours:

```yaml
rx_driver: rmt_spi_rx
tx_driver: fast_gpio_tx
rx_worker: false
tx_worker: false
rmt_spi_frame_gap_us: 1000
```

Representative end-of-soak counters:

```text
RMT/SPI boundaries:             3,421,460
RMT/SPI completed frames:       3,421,459
33-byte frames:                 3,421,459
Invalid transaction lengths:    1
SPI result errors:              0
SPI queue errors:               0
RMT re-arm errors:              0
Maximum buffered frames:        2
Completed-frame overwritten:    0
Completed-frame dropped:        0

Redux RX bytes:                 112,913,625
Redux RX chunks:                3,421,625
Candidate frames:               3,421,625
Valid frames:                   3,421,625
Invalid frames:                 0
Checksum failures:              0
Signature misses:               0
Sync losses:                    0
Dropped bytes:                  0
```

At the observed bus rate, this represents roughly 3.42 million clean 33-byte frames over about 47.5 hours.

### RX conclusion

The RX result is effectively clean:

- no checksum failures;
- no signature misses;
- no synchronisation losses;
- no SPI result or queue errors;
- no completed-frame overwrites or drops;
- no protocol-invalid frames.

The single invalid transaction length across more than 3.4 million completed transactions is not currently material. It may represent startup, shutdown, or one malformed boundary interval. It should remain visible in diagnostics and only becomes actionable if it starts increasing regularly.

This soak materially strengthens the conclusion that the RMT-generated-CS SPI/DMA receive path is stable on the tested ESP32-S3 hardware.

### TX result

The same soak recorded:

```text
TX frames:                    340,345
TX failures:                  389
TX command frames:            8
TX command failures:          0
Command confirmations:        8
Command confirmation timeouts: 0
```

The aggregate TX failure rate is approximately 0.11%. The command path remained reliable: every tested command confirmed and none timed out. The failures therefore appear to be background/opdata transmissions missing the FastGPIO timing window rather than user-command failures.

FastGPIO TX remains the weaker transport side and the next transport-level improvement target.

### Main-loop timing correlation

End-of-soak timing counters included:

```text
Loop average:                  36 us
Loop maximum:                  98,092 us
Loop over-budget count:        340,717
Transport maximum:             64,404 us
RX section maximum:            63,836 us
TX frames:                     340,345
```

The near one-to-one relationship between `loop over-budget` and `TX frames` strongly indicates that synchronous FastGPIO TX is causing almost every main-loop budget overrun.

The large RX-section maximum does not imply slow hardware capture. RMT/SPI capture is asynchronous and remained error-free. The section timer is measuring the component's downstream processing period around blocking TX activity and accumulated RX draining.

### Opdata catalogue pressure

The soak also exposed a separate catalogue issue:

```text
Catalog ingested:      3,421,625
Extended frames:       3,144,419
Opdata frames:         277,206
Catalog overwritten:   42
Opdata slots full:      50,832
```

`opdata_slots_full` is not raw transport loss. The RMT/SPI backend reported zero dropped or overwritten completed frames. It means the fixed opdata latest-slot catalogue could not allocate or match a slot for more than 50,000 opdata insert attempts.

This requires targeted diagnostics before changing capacity. The next step should log or count the rejected opdata key so it is possible to determine whether:

- the catalogue capacity is genuinely too small;
- more opdata keys exist than expected;
- variable or malformed keys are consuming slots;
- slot reuse or matching is incorrect;
- the counter represents repeated attempts against an already saturated catalogue.

Increasing the catalogue size without identifying the rejected keys may only hide the underlying classification or keying problem.

## 2026-07-30 — DMA is not required for MHI frames

The integrated transport was initially DMA-backed because the ESP32-S3 implementation was already working and provided complete-frame buffers with low CPU involvement.

Further review and hardware testing showed that DMA is not required for this protocol.

### Technical basis

The largest supported MHI frame is 33 bytes. The transport rounds its working buffer to 36 bytes, which remains within the SPI slave FIFO transaction capacity.

The hardware requirement is:

```text
external SCK sampling
full-duplex MOSI/MISO shifting
one complete 20-byte or 33-byte transaction
actual transaction-length reporting
```

DMA does not provide a functional requirement that the FIFO path cannot satisfy at this frame size.

Removing DMA removes:

- DMA-capable buffer allocation and alignment restrictions;
- DMA channel selection and reset handling;
- DMA-specific CS preparation behaviour;
- a second transport mode and public driver label.

The RMT-generated internal CS boundary and SPI transaction state machine remain unchanged.

### FIFO comparison and consolidation

The FIFO implementation was first validated independently against the established ESP32-S3 path. It was made available on both:

```text
original dual-core ESP32
ESP32-S3
```

After cross-chip functional validation, the FIFO implementation became the sole `rmt_cs_spi` implementation. The production driver now always uses `SPI_DMA_DISABLED` and internal 8-bit-capable buffers.

### Original ESP32 mode-3 edge correction

The first original-ESP32 FIFO test produced correctly bounded 33-byte transactions, but the captured MOSI bytes were invalid:

```text
candidate frames:       0
valid frames:           0
signature misses:       19,765
representative bytes:   ff ff e0
```

RMT boundaries, transaction completion, queueing, and transaction lengths were otherwise working. The failure was isolated to the original ESP32 SPI slave's CPU-FIFO mode-3 sampling edge.

The original ESP32 path therefore applies a target-specific correction after SPI initialisation:

```text
ck_idle_edge = 1
ck_i_edge    = 1
```

This correction is internal to the transport. It does not change YAML configuration, frame-gap settings, protocol decoding, or the ESP32-S3 path.

### Initial original-ESP32 FIFO result

Target:

```text
Original dual-core ESP32 rev 1.1
ESPHome 2026.7.2
ESP-IDF 5.5.x
33-byte frames
SCK=33 MOSI=25 MISO=21
rmt_cs_spi
classified command worker enabled
```

Representative result after the edge correction:

```text
Redux RX bytes:                  39,567
Redux RX chunks:                1,199
Candidate frames:               1,199
Valid frames:                   1,199
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

RMT/SPI boundaries:             1,202
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

Power, mode, and temperature changes were transmitted and confirmed by returned MOSI state. Opdata continued to advance.

Two TX mailbox overwrites occurred while commands were being changed rapidly. They did not correlate with a TX failure, confirmation timeout, retry, or stale pending command. The two invalid transaction lengths were not exposed to the protocol parser: all 1,199 candidate frames were valid.

### Finding

The evidence supports the following conclusion:

```text
DMA is not required for 20-byte or 33-byte MHI transactions.
```

The FIFO path:

- preserves hardware-assisted full-duplex capture and transmission;
- preserves RMT-derived transaction boundaries;
- supports both original ESP32 and ESP32-S3;
- avoids DMA-specific allocation and transaction-management complexity;
- has sufficient performance headroom for the observed bus rate;
- completed original-ESP32 and ESP32-S3 functional validation.

The resulting production position is one supported FIFO-backed `rmt_cs_spi` driver on both targets, with no DMA mode or alternate public driver label.

## Worker experiment

Both workers were then enabled as a stress experiment:

```yaml
rx_worker: true
tx_worker: true
```

The result was functionally alive but not acceptable as the default:

```text
RMT/SPI frames completed:     1202
RX worker frames ingested:    1100
Completed frames overwritten: 98
Completed frames dropped:     98
Maximum buffered depth:       2
TX failures:                  2
```

The workers also showed very high polling activity:

```text
RX worker loops:       55997
RX idle yields:        54897
TX worker loops:       52002
TX flush successes:    57
TX idle yields:        51945
```

Main-loop pressure improved substantially, but that improvement came with frame loss and TX bus-marker misses.

### Interpretation

The hardware backend is not receiving too many frames. The AC remains at approximately 20 frames per second, which the no-worker path processes without loss.

The failure is scheduling architecture:

```text
Hardware producer:
  deterministic SPI transaction completion

Generic workers:
  poll, yield, and compete for scheduling

Two-frame handoff:
  fills during worker scheduling gaps
```

Therefore the queue depth should not simply be increased. A deeper FIFO would retain stale repeated frames and delay multiplexed state.

## Queue findings

The required distinction is:

```text
SPI transaction queue:
  keep depth 4
  hardware readiness mechanism for the SPI transaction path

Completed-frame handoff:
  keep depth 2
  short scheduling bridge only

Frame catalogue:
  latest status slot
  latest extended-status slot
  keyed latest opdata slots
  command-candidate slot
```

A conventional application FIFO is not the desired architecture. Repeated steady-state frames should collapse into latest slots. Whole frames may be overwritten under exceptional pressure, but partial byte streams must never be produced.

## Recommended configuration

The historical ESP32-S3 RX-only configuration remains useful as the long-soak split-path reference:

```yaml
MhiAcCtrl:
  frame_size: 33
  sck_pin: 8
  mosi_pin: 38
  miso_pin: 39
  rx_driver: rmt_spi_rx
  tx_driver: fast_gpio_tx
  rmt_spi_frame_gap_us: 1000
  command_worker: false
```

The integrated cross-chip full-duplex configuration is:

```yaml
MhiAcCtrl:
  frame_size: 33
  sck_pin: 33
  mosi_pin: 25
  miso_pin: 21
  rx_driver: rmt_cs_spi
  rmt_spi_frame_gap_us: 1000
  command_worker: true
```

Use pins appropriate to the selected board. Do not configure a separate `tx_driver`; `rmt_cs_spi` owns both RX and TX. The implementation uses the SPI CPU FIFO with DMA disabled on both ESP32 and ESP32-S3.

## RTOS ownership direction

The earlier worker experiment established that larger queues and faster generic polling were not the correct solution. The integrated `rmt_cs_spi` transport now moves real-time transaction ownership into the transport and uses the classified command worker for downstream decode and command coordination.

The remaining direction should continue to favour event-driven ownership rather than polling.

Preferred RX shape:

```text
RMT/SPI completion notification
  -> backend task wakes
  -> drain all completed SPI transactions
  -> validate and classify
  -> update latest catalogue slots directly
  -> notify main loop that decoded work is available
  -> block again
```

Preferred TX shape:

```text
command queued or frame-boundary notification
  -> TX task wakes
  -> transmit only in a valid bus window
  -> block again
```

This should eliminate tens of thousands of idle worker loops and reduce the chance of missing short bus-marker windows.

## Acceptance criteria for further testing

Further soak and regression testing should retain:

```text
invalid transaction lengths remains zero or effectively isolated
SPI result errors = 0
SPI queue errors = 0
RMT re-arm errors = 0
completed-frame overwritten = 0
completed-frame dropped = 0
checksum failures = 0
signature misses = 0
sync losses = 0
command confirmation timeouts = 0
retry exhaustions = 0
completion drops = 0
TX mailbox overwrites remain explainable and do not affect commands
opdata continues to advance
opdata rejected-key diagnostics remain explainable
FastGPIO TX failure rate does not regress on the historical hybrid path
Home Assistant state matches the AC
```

Longer-term work should compare the same hardware and workload across:

```text
fast_gpio_rx + fast_gpio_tx
external_clock_rx + fast_gpio_tx
rmt_spi_rx + fast_gpio_tx
rmt_cs_spi on ESP32-S3
rmt_cs_spi on original ESP32
```

The historical evidence established RMT-generated-CS SPI as the cleanest receive path. The 2026-07-30 result extends that finding: the same hardware-assisted architecture operates full-duplex through the SPI FIFO on both supported chip families without DMA.
