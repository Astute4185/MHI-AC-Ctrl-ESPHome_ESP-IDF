# MHI Transport and Worker Findings

## Scope

This document records the current findings from testing:

- `fast_gpio_rx` versus `external_clock_rx`
- FastGPIO TX interaction with both RX backends
- optional RX/TX workers
- frame cataloging and opdata flow
- the working theory that the FastGPIO path is CPU/resource constrained under real ESPHome load

This is a working engineering note, not a final support guarantee.

## Protocol constraints

The MHI bus exposes only:

```text
SCK
MOSI
MISO
```

There is no chip-select line. The AC is the SPI-style master. Frame boundaries are inferred from timing gaps, signatures, frame length, and checksum validation.

Representative timing from the protocol notes:

```text
Frame time:       around 10 ms
Frame pause:      around 40 ms
Byte time:        around 250 us
Byte pause:       around 250 us
Bit time:         around 31.25 us
Nominal cadence:  around 20 frames per second
```

This makes the bus awkward for normal ESP-IDF SPI slave handling and explains why this project uses custom transport code.

## Executive summary

Current practical recommendation:

```text
Normal users:
  rx_driver: fast_gpio_rx
  tx_driver: fast_gpio_tx
  rx_worker: false
  tx_worker: false

Experimenters:
  rx_driver: external_clock_rx
  tx_driver: fast_gpio_tx
  rx_worker: true
  tx_worker: true
```

Do not recommend workers for `fast_gpio_rx` normal use.

The important finding is that **better timing counters do not automatically mean better AC control**. Worker mode can reduce ESPHome loop budget pressure while still degrading command confirmation or opdata flow.

## Backend comparison

| Backend | Current status | RX quality | TX/commands | Opdata | Loop budget | Recommendation |
|---|---:|---:|---:|---:|---:|---|
| `fast_gpio_rx` + `fast_gpio_tx`, workers off | Stable/default | Good | Good | Proven | Can spike | Recommended default |
| `external_clock_rx` + `fast_gpio_tx`, workers off | Experimental | Good to excellent after tuning | Good | Proven | Still spikes due FastGPIO TX | Useful test path |
| `external_clock_rx` + `fast_gpio_tx`, workers on | Experimental | Promising | Needs soak validation | Needs soak validation | Best improvement | Worker path worth testing |
| `fast_gpio_rx` + workers | Not recommended | RX can look clean | Regressions seen | Regressions seen | Looks better on paper | Do not recommend |


## Timing results from representative tests

These are engineering timing notes from the current test logs, not final benchmarks. They are still useful because they show where the load moves when changing RX backend or enabling workers.

| Test path | Representative result | Timing result | Functional result | Read |
|---|---|---|---|---|
| `external_clock_rx` + `tx_driver: none` on ESP32-S3 | RX-only validation, no control TX | `loop_us avg` around 23 us, `max` around 2.6-6.7 ms, `over_budget=0`; transport max around 4.1 ms in the cleaner falling-edge runs | RX only; useful for proving capture timing without TX cost | External-clock RX alone is light on the main loop once tuned. |
| `external_clock_rx` + `fast_gpio_tx`, workers off | Valid frames and opdata flow; `tx_background_interval_ms: 250` | Early run showed `loop_us max=50453`, `over_budget=236`, `transport max=50044` | Opdata did flow, but FastGPIO TX still caused large main-loop transport spikes | External-clock RX helps RX, but FastGPIO TX still dominates worst-case timing. |
| Long external-clock soak, workers off | `valid_frames=284555`, `invalid_frames=6`, `tx_frames=70640`, `tx_failures=233` | `loop_us max=57325`, large accumulated `over_budget`; transport max around 56 ms | Commands confirmed and opdata continued to publish | Functionally useful, but still budget-stressed because TX remained FastGPIO/main-loop owned. |
| `external_clock_rx` + `fast_gpio_tx`, RX/TX workers on | `valid_frames=1679`, `checksum_failures=0`, `flush_successes=89` | `loop_us avg=18`, `max=22236`, `over_budget=0`; transport max around 4.2 ms | `command_confirmations=3`, `command_confirmation_timeouts=0`, opdata present | Best timing result seen so far; still experimental and needs soak validation. |
| `fast_gpio_rx` + `fast_gpio_tx`, RX/TX workers on | Clean RX counters, `valid_frames=1996`, `checksum_failures=0` | `loop_us avg=20`, `max=7040`, `over_budget=0` | Bad functional result: `command_confirmations=0`, `command_confirmation_timeouts=3`, `opdata=0` | Good timing counters were misleading; worker mode degraded real FastGPIO behaviour. |
| `fast_gpio_rx` + `fast_gpio_tx`, RX worker only | Clean RX counters, TX still main-loop auto-flush | `loop_us max` around 65-69 ms; transport max around 50-64 ms | Opdata stayed at 0 and command timeouts appeared | RX worker does not remove FastGPIO TX cost; the system still looks resource constrained. |

### Timing interpretation

The timing results support three separate conclusions:

```text
1. external_clock_rx reduces RX-side timing pressure.
2. fast_gpio_tx can still create 40-60 ms class transport stalls when it is owned by the main loop.
3. moving work into workers can improve loop timing counters without proving functional correctness.
```

The most important example is `fast_gpio_rx` with workers. It produced the best-looking loop budget numbers, but command confirmation and opdata were worse. Therefore, timing counters must be treated as necessary but not sufficient.

A backend should only be considered healthier if the timing improvement happens together with:

```text
command confirmations working
opdata continuing to publish
checksum/signature health staying clean
Home Assistant state matching the physical AC
```

This is why the current worker recommendation is limited to `external_clock_rx` testing. External-clock plus workers produced the best timing shape while still keeping command confirmation and some opdata alive in the observed run. FastGPIO plus workers improved loop budget but failed the functional checks.

## FastGPIO findings

FastGPIO remains the stable/default path because it has the best functional behaviour today. Commands confirm, Home Assistant state generally settles correctly, and opdata has been proven to flow on the non-worker path.

The downside is loop budget pressure. FastGPIO RX/TX has blocking, timing-sensitive sections. Some runs show clean protocol health while the ESPHome loop still records long transport sections and over-budget counts.

Observed pattern:

```text
valid_frames climbs
checksum_failures can remain low or zero
commands can confirm
opdata can publish
but transport max can still hit tens of milliseconds
```

Working theory:

```text
FastGPIO is close to the practical CPU/scheduling budget.
When Wi-Fi/API/logging/publishing/TX all compete with the timing-critical GPIO path,
less frequent work such as opdata request/response handling is the first thing to suffer.
```

This does not mean FastGPIO is bad. It means FastGPIO should remain the default **without workers**, and experimental scheduling changes must be judged by functional behaviour, not just loop timing counters.

## External-clock RX findings

`external_clock_rx` was added to reduce CPU pressure on the RX side by sampling from the AC-provided external clock and producing signature-anchored frame chunks.

Observed positives:

- RX can be clean after tuning.
- RX loop cost can be much lower than synchronous FastGPIO sampling.
- It works with `fast_gpio_tx` via the same RX bus-marker concept.
- It is the best current candidate for worker-mode testing.

Observed limitations:

- TX is still FastGPIO.
- With workers disabled, FastGPIO TX can still cause main-loop transport spikes.
- External-clock RX is still experimental and must be soak-tested per board/unit.

Current conclusion:

```text
external_clock_rx is the right experimental direction for reducing RX-side CPU pressure.
It does not eliminate FastGPIO TX cost by itself.
```

## Worker findings

### RX worker

The RX worker can move capture/sync/classify/catalog work away from the main ESPHome loop.

Desired responsibility:

```text
RX worker:
  capture RX bytes
  frame sync
  classify frames
  write latest catalog slots
```

It should not publish ESPHome entities and should not trigger TX side effects.

Observed FastGPIO RX-worker issue:

```text
rx_worker: true
tx_worker: false
RX protocol health can look clean
commands can partially work
opdata may stop flowing or catalog as only extended/status-like frames
main loop can still spike because TX remains FastGPIO auto-flush
```

Conclusion:

```text
RX worker is not recommended for fast_gpio_rx normal use.
```

### TX worker

The TX worker moves FastGPIO TX flushing out of the ESPHome component loop.

Desired responsibility:

```text
TX worker:
  wait for queued TX frame
  wait for RX bus marker
  flush/send TX frame in the response window
```

Observed positive:

```text
Main loop budget improves significantly when TX worker owns blocking TX.
```

Observed FastGPIO issue:

```text
fast_gpio_rx + tx_worker can reduce loop spikes but degrade command confirmation.
```

Conclusion:

```text
TX worker is not recommended for fast_gpio_rx normal use.
```

### Current worker recommendation

Workers should be documented as:

```text
Experimental.
Opt-in.
Recommended only for external_clock_rx testing.
Not recommended for fast_gpio_rx normal use.
```

A worker run is only healthy if all of these are true:

```text
valid_frames increases
checksum_failures = 0 or remains acceptably low
signature_misses does not climb unexpectedly
tx_failures remains low
command_confirmations increases when commands are sent
command_confirmation_timeouts = 0
opdata continues to appear when requested
Home Assistant state matches the physical AC state
```

## Frame catalog findings

The frame catalog/latest-slot model remains the right design direction.

Do not go back to a general FIFO backlog. This includes small/bounded FIFOs for status or extended-status frames: they were considered during RX-worker testing and rejected because the problem is not just memory growth, it is queue/timing pressure and stale frame ordering under load.

Current preferred model:

```text
status:
  latest slot

extended status:
  latest slot

opdata:
  keyed latest slots

command feedback:
  separate latest command-candidate slot if needed
```

This keeps memory bounded and avoids stale queue pressure while still allowing special handling for transient command-confirmation frames.

Rejected direction:

```text
status FIFO
extended-status FIFO
```

Reason:

```text
A FIFO reintroduces queue/timing pressure.
A FIFO can preserve stale intermediate frames that the main loop then has to drain.
That competes with command confirmation, opdata decode, publishing, and TX scheduling.
It moves away from the latest-state design decision.
```

Rejected even if bounded:

```text
status FIFO[small N]
extended-status FIFO[small N]
```

The issue is not only unbounded memory. The issue is that FIFO processing asks the main loop to catch up with historical frames when the system is already timing constrained.

## Opdata findings

Opdata is model-dependent and validity-gated. A configured opdata sensor should remain unavailable until the AC actually returns a supported value.

Known practical finding:

```text
Opdata flowing is a stronger health signal than RX valid_frames alone.
```

A run with clean RX but no opdata is not fully healthy.

Observed worker-related concern:

```text
fast_gpio_rx + rx_worker can show clean RX and catalog growth while opdata remains at zero.
```

Theory:

```text
The system may be resource-constrained enough that opdata request/response timing suffers before basic status frames fail.
```

## Resource-constraint theory

The current theory is that FastGPIO is resource-constrained under realistic ESPHome load. This is also why FIFO was rejected: buffering more status frames does not remove CPU/timing pressure, it moves that pressure into decode and scheduling.

The current theory is:

```text
FastGPIO RX/TX is software-heavy and timing-sensitive.
The AC bus has a fixed external cadence.
ESPHome adds Wi-Fi, API, logging, publishing, sensors, and component loop work.
When FastGPIO TX/RX and ESPHome work overlap, the main loop can hit large transport spikes.
```

Implication:

```text
Basic status decode may survive.
Commands may partly survive.
Opdata, being request/response and lower-frequency, is more fragile.
```

This explains why a run can look good on RX counters but still be functionally degraded.

## Recommended user-facing support matrix

### Supported/default

```yaml
rx_driver: fast_gpio_rx
tx_driver: fast_gpio_tx
rx_worker: false
tx_worker: false
```

Use this for normal installs.

### Experimental external-clock

```yaml
rx_driver: external_clock_rx
tx_driver: fast_gpio_tx
rx_worker: false
tx_worker: false
```

Use this to validate external-clock RX without introducing worker variables.

### Experimental external-clock with workers

```yaml
rx_driver: external_clock_rx
tx_driver: fast_gpio_tx
rx_worker: true
tx_worker: true
tx_background_interval_ms: 1000
```

Use this only for testing. Validate commands and opdata, not just RX counters.

### Not recommended

```yaml
rx_driver: fast_gpio_rx
tx_driver: fast_gpio_tx
rx_worker: true
```

```yaml
rx_driver: fast_gpio_rx
tx_driver: fast_gpio_tx
tx_worker: true
```

Do not recommend these to users. They are debugging configurations only.

## Validation checklist

A backend should not be called healthy unless it passes this checklist:

```text
valid_frames climbs
invalid_frames remains zero or acceptably low
checksum_failures remains zero or acceptably low
signature_misses does not climb unexpectedly
sync_losses does not climb unexpectedly
tx_failures remains low
command confirmations work for known-good commands
command confirmation timeouts remain zero
opdata sensors continue to update
Home Assistant state matches the physical AC state
loop budget is acceptable for the selected backend
no safe-mode boot or reboot loop
```

## Current conclusion

The default path should remain conservative:

```text
fast_gpio_rx + fast_gpio_tx
workers disabled
```

The experimental path should focus on reducing RX CPU pressure:

```text
external_clock_rx + fast_gpio_tx
optional workers for external-clock testing only
```

The long-term direction remains a hardware-assisted external-clock sampler, ideally using an ESP32-S3-friendly peripheral/DMA path. FastGPIO should remain the fallback and stable baseline.
