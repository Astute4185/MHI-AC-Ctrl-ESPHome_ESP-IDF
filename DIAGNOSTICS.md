# Diagnostics and Hardware Validation

This guide explains the current MHI runtime diagnostics, how to distinguish recoverable events from real failures, and what evidence should accompany a hardware or driver result.

Driver availability and configuration rules are documented in [`DRIVER_SELECTION.md`](DRIVER_SELECTION.md).

## Logging strategy

Use DEBUG during initial bring-up and focused command testing:

```yaml
logger:
  level: DEBUG
```

For a longer soak, reduce unrelated log load while retaining MHI health lines:

```yaml
logger:
  level: WARN
  logs:
    mhi.diag: INFO
    mhi_rmt_cs_spi: INFO
    mhi_rmt_spi_rx: INFO
```

Only enable the driver tag that applies to the selected transport.

A clean startup log is not sufficient. Review diagnostics after command sequences, Home Assistant reconnects, opdata polling, Wi-Fi activity, and sustained runtime.

## Reading the periodic diagnostics

The component emits several periodic lines:

1. common RX/TX protocol health;
2. command confirmation and retry state;
3. frame-catalogue and worker-decoded-store activity;
4. TX scheduling and worker activity;
5. transport queue and loop timing;
6. a driver-specific line for hardware-assisted transports.

Interpret the lines together. A clean transport does not prove that commands confirmed or that opdata continued to publish.

## Common protocol health

Primary fields:

```text
rx_bytes
rx_chunks
candidate_frames
valid_frames
invalid_frames
checksum_failures
signature_misses
sync_losses
dropped_bytes
last_valid_frame_age_ms
last_rx_byte_age_ms
```

| Counter | Meaning | Healthy expectation |
|---|---|---|
| `rx_bytes` | Bytes delivered by the selected transport | Increases while the bus is active |
| `rx_chunks` | Transport chunks passed into frame processing | Increases with traffic |
| `candidate_frames` | Potential frames evaluated | Tracks frame activity |
| `valid_frames` | Frames accepted after structure and checksum validation | Increases continuously |
| `invalid_frames` | Candidate frames rejected | Zero during normal operation |
| `checksum_failures` | Frames rejected by checksum | Zero |
| `signature_misses` | Data discarded while searching for a valid signature | Zero on complete-frame hardware paths; bounded recovery activity may occur on software paths |
| `sync_losses` | Loss of frame synchronisation | Zero |
| `dropped_bytes` | Bytes discarded during overflow or recovery | Zero |
| `last_valid_frame_age_ms` | Age of the last accepted frame | Close to the normal bus cadence |
| `last_rx_byte_age_ms` | Age of the last received byte | Close to the normal bus cadence |

A single startup anomaly can be acceptable when the counter remains fixed. A counter that continues increasing represents an active fault.

The `rx_protocol_health healthy=YES/NO` line is a delta-based summary. Treat the underlying counters as authoritative when diagnosing an anomaly.

## TX health

Primary fields:

```text
tx_frames
tx_failures
tx_command_frames
tx_command_failures
unsupported_commands
last_tx_command_mask
last_tx_command_age_ms
```

| Counter | Meaning | Healthy expectation |
|---|---|---|
| `tx_frames` | Frames successfully accepted by the TX backend | Increases with background and command traffic |
| `tx_failures` | General TX failures | Zero or fully explained by a fixed startup event |
| `tx_command_frames` | Command-bearing frames actually transmitted | Increases when controls are used |
| `tx_command_failures` | Failed command-frame attempts | Zero |
| `unsupported_commands` | Commands the active path could not represent | Zero |
| `last_tx_command_mask` | Fields carried by the last command frame | Used for focused command tracing |
| `last_tx_command_age_ms` | Age of the last command transmission | Informational |

A TX frame count does not prove command success. Returned AC state is authoritative.

## Command confirmation, retries, and supersession

Commands move through three distinct stages:

```text
staged intent -> actual bus transmission -> semantic MOSI confirmation
```

Monitor:

```text
command_confirmations
confirmation_timeouts
retries
retry_exhaustions
staged_timeouts
pending_confirmation_mask
last_confirmed_mask
last_timeout_mask
last_retry_mask
last_exhausted_mask
last_staged_timeout_mask
```

| Counter | Meaning | Interpretation |
|---|---|---|
| `command_confirmations` | Command fields confirmed from returned MOSI state | Should increase after successful commands |
| `confirmation_timeouts` | Confirmation attempt exceeded its field-specific window | Ideally zero; recoverable when a retry subsequently confirms |
| `retries` | Timed-out fields queued for another attempt | Should correspond to recoverable confirmation timeouts |
| `retry_exhaustions` | All permitted attempts completed without confirmation | Hard command failure; expected zero |
| `staged_timeouts` | A command envelope remained staged without real TX completion for too long | Indicates transport or completion-path blockage; expected zero |
| `pending_confirmation_mask` | Fields currently awaiting semantic feedback | Returns to zero after confirmation, supersession, or exhaustion |

A non-zero `confirmation_timeouts` count is not automatically a failed run. The louver matrix demonstrated that a command may be ignored on the first attempt and confirm after one retry. Acceptance requires:

```text
retry_exhaustions = 0
staged_timeouts = 0
pending_confirmation_mask returns to 0
physical state and Home Assistant state match
```

### Extended-louver timing

Horizontal vane and 3D Auto confirmation use three-second windows. Other command fields use the normal ten-second confirmation window.

### Latest-intent supersession

Horizontal vane and 3D Auto share DB16/DB17. When newer intent changes the composite desired state, the old confirmation generation is superseded and a combined command can be transmitted immediately.

Typical trace:

```text
command: superseded pending confirmation mask=0x00000020
command: staged=YES mask=0x00000060 ...
command: confirmed mask=0x00000060 pending=0x00000000
```

This is expected behaviour, not a failure. The latest requested composite state must confirm without stale intent carrying into the next command.

### Command test scope

Exercise at least:

- power on and off;
- each supported climate mode;
- multiple temperature changes;
- every exposed fan level;
- all vertical vane positions and swing;
- all horizontal vane positions and swing on 33-byte units;
- 3D Auto on and off;
- commands while the unit is off;
- rapid replacement of pending commands;
- horizontal/3D changes issued close together.

## Frame catalogue and decoded snapshots

The frame catalogue separates standard status, extended status, opdata, unknown frames, and command-confirmation candidates.

Monitor:

| Counter | Meaning | Healthy expectation |
|---|---|---|
| `catalog ingested` | Valid frames accepted by the catalogue | Tracks `valid_frames` closely |
| `status` | Standard status frames classified | Model/frame dependent |
| `extended` | Extended status frames classified | Expected on 33-byte units |
| `opdata` | Operation-data responses classified | Increases while opdata polling is active |
| `unknown` | Valid frames not mapped to a known class | Zero or explicitly explained |
| `overwritten` | Latest-value catalogue slot replaced before consumption | Bounded latest-value behaviour; investigate continuous growth with missing publications |
| `opdata_slots_full` | Opdata could not be retained because all semantic slots were occupied | Zero |
| `command_candidates` | Feedback frames considered for confirmation | Increases around commands |

For worker decode, monitor:

```text
worker_decode status=writes/overwrites
worker_decode extended=writes/overwrites
worker_decode candidates=writes/overwrites
opdata_merges
opdata_field_overwrites
unknown=writes/overwrites
publish_batches
pending_high_water
unknown_high_water
```

Latest-value overwrites are not automatically data loss. Repeated status can replace an older pending status before the main loop applies it. The failure condition is missing or stale published state, lost opdata fields, unbounded queue pressure, or command-confirmation regression.

Opdata is model-dependent. A field that the AC never returns should remain unavailable rather than being fabricated as zero.

## TX scheduling and priority

Monitor:

```text
command_attempts
background_attempts
background_failures
interval_deferrals
confirmation_deferrals
```

Expected behaviour:

- command traffic bypasses the background interval;
- routine background traffic waits while confirmation is pending;
- `confirmation_deferrals` can increase during an active command and should stop afterward;
- repeated `background_failures` require investigation, particularly if opdata stops updating.

A split `fast_gpio_tx` path can show occasional background failures while user commands still confirm. Record the failure count and verify that it does not correlate with missing opdata or command failures.

## Command-worker diagnostics

`command_worker` is opt-in and production-capable, but the synchronous path remains available for comparison.

Monitor:

```text
command_worker enabled
command_worker running
command_worker classified_rx
wakes
service_runs
idle_polls
frames_staged
completions
rx_polls
rx_batches
rx_chunks
rx_frames
rx_max_batch
runtime_us=last/max
notify_max
stack_free_min
```

| Field | Meaning | Healthy expectation |
|---|---|---|
| `enabled` | YAML setting | Matches configuration |
| `running` | FreeRTOS worker task state | `YES` when enabled |
| `classified_rx` | Worker drains and decodes RX | `YES` for queue-backed drivers; `NO` for `fast_gpio_rx` |
| `wakes` | Explicit worker notifications consumed | Increases with command and completion activity |
| `service_runs` | Combined command/RX service passes | Increases continuously while active |
| `idle_polls` | Timed passes without an explicit notification | Can increase steadily without being a fault |
| `frames_staged` | TX envelopes accepted by the transport | Increases with background and command traffic |
| `completions` | Command frames reported complete after a real transaction | Increases only for command-bearing completions |
| `rx_polls` | Worker RX polling passes | Increases when `classified_rx=YES` |
| `rx_batches` | Polls that produced at least one valid frame | Increases on an active bus |
| `rx_chunks` | Transport chunks drained by the worker | Tracks queue handoff activity |
| `rx_frames` | Valid frames synchronised and catalogued by the worker | Broadly tracks `valid_frames` |
| `rx_max_batch` | Maximum valid frames processed in one poll | Normally small and bounded |
| `runtime_us` | Last and maximum worker service duration | Use to detect worker stalls |
| `notify_max` | Maximum notification batch consumed at once | Normally small |
| `stack_free_min` | Minimum remaining worker stack | Must remain comfortably above zero |

Queue-backed RX drivers that support classified RX:

- `external_clock_rx`;
- `rmt_spi_rx`;
- `rmt_cs_spi`.

`fast_gpio_rx` remains main-loop capture/synchronisation/decode even when the command worker is enabled.

A valid worker run requires:

- real TX completion before confirmation starts;
- successful command confirmation;
- no staged timeouts or retry exhaustion;
- continued opdata publication;
- bounded worker and transport queues;
- synchronised physical and Home Assistant state.

Lower loop timing alone is not acceptance evidence.

## Transport queues

Common queue line:

```text
transport_queues rx_depth=... rx_high_water=... rx_overwritten=...
completion_depth=... completion_high_water=... completion_dropped=...
```

| Counter | Healthy expectation |
|---|---|
| `rx_depth` | Normally zero or low after each service pass |
| `rx_high_water` | Low and bounded |
| `rx_overwritten` | Zero |
| `completion_depth` | Normally zero |
| `completion_high_water` | Low and bounded |
| `completion_dropped` | Zero |

A high-water mark is historical. It does not need to return to zero. The current depth should drain and overwrite/drop counters should remain fixed.

## Main-loop timing

The component reports:

```text
loop_us last / average / maximum
over_budget
section_us transport / tx / rx / publish / command
```

Use section timing to identify pressure:

- `transport`: selected transport service and completion draining;
- `tx`: command/background staging and context refresh;
- `rx`: main-loop RX work or worker-snapshot apply;
- `publish`: ESPHome state publication;
- `command`: timeout and retry housekeeping.

A single startup over-budget event can be non-material when the count remains fixed. Sustained growth requires investigation.

FastGPIO RX/TX can block while following the AC-owned clock. Hardware-assisted paths should materially reduce normal loop pressure, but functional behaviour remains the acceptance criterion.

## `rmt_spi_rx` diagnostics

This ESP32-S3 split backend uses RMT-derived frame boundaries and DMA-backed SPI RX. TX remains `fast_gpio_tx`.

Example fields:

```text
boundaries
completed
frame20
frame33
invalid_len
result_errors
queue_errors
rmt_rearm_errors
buffered_frames
max_buffered
overwritten
dropped
```

| Counter | Healthy expectation |
|---|---|
| `boundaries` | Increases with frame traffic |
| `completed` | Closely tracks boundaries after startup |
| `frame20` / `frame33` | Only the configured frame counter increases |
| `invalid_len` | Zero after startup |
| `result_errors` | Zero |
| `queue_errors` | Zero |
| `rmt_rearm_errors` | Zero |
| `buffered_frames` | Zero or low |
| `max_buffered` | Low and bounded |
| `overwritten` | Zero |
| `dropped` | Zero |

If RX remains clean but loop timing or background TX degrades, investigate `fast_gpio_tx` separately.

## `rmt_cs_spi` diagnostics

`rmt_cs_spi` is the FIFO-backed full-duplex transport. It owns the complete SPI transaction and its dedicated owner task.

Example line:

```text
boundaries=51611 completed=51610 tx_completed=10355 tx_failures=0
frame20=0 frame33=51610 invalid_len=1 result_errors=0
queue_errors=0 rmt_rearm_errors=0 buffered_frames=0 max_buffered=2
rx_overwritten=0 tx_overwritten=20 dropped=0 completion=0/1/0
task_running=YES
```

| Counter | Meaning | Healthy expectation |
|---|---|---|
| `boundaries` | Inter-frame gaps detected by RMT | Increases with bus traffic |
| `completed` | SPI transactions returned successfully | Closely tracks boundaries after startup |
| `tx_completed` | Transactions that carried a prepared TX snapshot | Increases with background and command traffic |
| `tx_failures` | Failed TX-bearing transactions | Zero |
| `frame20` / `frame33` | Completed transaction lengths | Only the configured frame counter increases |
| `invalid_len` | Unexpected transaction length | Zero after startup; one fixed attach-time event can be acceptable |
| `result_errors` | Errors retrieving transaction results | Zero |
| `queue_errors` | Errors queuing the next SPI transaction | Zero |
| `rmt_rearm_errors` | Failures restarting boundary detection | Zero |
| `buffered_frames` | RX frames waiting for consumption | Normally zero or low |
| `max_buffered` | Historical RX queue high-water mark | Low and bounded |
| `rx_overwritten` | Completed RX frames replaced before consumption | Zero |
| `tx_overwritten` | Pending latest TX snapshot replaced before use | Can increase during aggressive command/background churn; not a failure when commands confirm and no completions are dropped |
| `dropped` | Transport frames discarded | Zero |
| `completion=current/high-water/dropped` | TX completion queue state | Current normally zero, high-water low, dropped zero |
| `task_running` | SPI owner task state | `YES` |

`tx_overwritten` implements latest-value mailbox behaviour. Investigate it when growth correlates with command failure, missing opdata, staged timeouts, or completion drops. Do not classify a bounded increase during an aggressive matrix as RX corruption.

## Known validated louver-matrix example

The completed 80-case vertical/horizontal/3D matrix showed:

```text
all requested combinations matched
retry_exhaustions=0
pending_confirmation_mask=0x00000000
invalid_frames=0
checksum_failures=0
signature_misses=0
sync_losses=0
dropped_bytes=0
tx_failures=0
queue_errors=0
rmt_rearm_errors=0
rx_overwritten=0
completion_dropped=0
```

The run included recoverable 3D confirmation timeouts, successful retries, horizontal/3D supersession, and a fixed startup `invalid_len=1`. Those events were acceptable because counters stopped increasing, all combinations confirmed, and no command exhausted its retry budget.

Detailed protocol findings are recorded in [`notes/FINDINGS_LOUVERS_3D_AUTO.md`](notes/FINDINGS_LOUVERS_3D_AUTO.md).

## Hardware validation workflow

Before a pull request:

```bash
./scripts/lint.sh fix
./scripts/test.sh
./scripts/compile-tests.sh
```

`compile-tests.sh` compiles the representative four-target matrix by default.

Record:

```text
ESP chip and revision
board/module
ESPHome version
ESP-IDF version
AC model
frame size
fan profile
SCK/MOSI/MISO pins
RX selection and effective TX
command-worker settings
tuning overrides
```

Then verify:

```text
valid_frames increases
invalid_frames remains zero
checksum_failures remains zero
signature_misses and sync_losses remain zero on complete-frame hardware paths
dropped_bytes remains zero
commands physically apply and confirm
retry_exhaustions remains zero
staged_timeouts remains zero
pending_confirmation_mask returns to zero
opdata continues to publish
Home Assistant matches confirmed AC feedback
queue, overwrite, drop, SPI, and RMT counters remain bounded and understood
```

Complete a command matrix before beginning a long soak.

## Soak-test record

Include:

```text
start and end time
total duration
logging configuration
Wi-Fi/API reconnect events
commands issued
opdata fields observed
end-of-test common diagnostics
end-of-test driver-specific diagnostics
crashes, watchdog resets, safe-mode boots
all anomalies and whether their counters continued increasing
```

Recommended sequence:

1. DEBUG bring-up and complete command test.
2. Several hours with MHI diagnostics visible.
3. A 24-48 hour lower-noise soak.
4. Final reconnect with MHI INFO diagnostics enabled.

## Troubleshooting

### No valid frames

Check in order:

1. SCK, MOSI, and MISO pins;
2. common ground and electrical connection;
3. `frame_size`;
4. selected driver and chip support;
5. visible SCK activity;
6. active driver names and ready state in `dump_config`.

Do not tune frame-gap values before confirming wiring and frame size.

### Repeated checksum or signature failures

Likely causes:

- wrong frame size;
- incorrect GPIO mapping;
- software-path timing instability;
- electrical noise or level problems;
- incorrect frame-boundary detection.

Compare with `fast_gpio_rx` to separate wiring/protocol problems from a hardware-assisted backend.

### Repeated invalid transaction lengths

For `rmt_spi_rx` and `rmt_cs_spi`:

- confirm `rmt_spi_frame_gap_us` is above the inter-byte gap and below the inter-frame gap;
- confirm the configured frame size;
- determine whether the counter is one fixed startup event or continues increasing;
- review `rmt_rearm_errors`, `queue_errors`, and `result_errors`.

### Commands transmit but do not confirm

Review:

```text
tx_command_frames
tx_command_failures
pending_confirmation_mask
confirmation_timeouts
retries
retry_exhaustions
last_confirmed_mask
last_timeout_mask
physical AC state
```

A transmitted command without matching returned state is not successful.

### Staged timeout

A `staged_timeout` means the command did not reach real TX completion in the expected time. Investigate:

- transport readiness;
- TX completion queue drops;
- driver task state;
- queue/result errors;
- FastGPIO marker/window timing on split paths.

This is different from a confirmation timeout, which occurs after the frame was transmitted.

### Opdata stops updating

Check:

- catalogue `opdata` growth;
- `opdata_slots_full`;
- worker decode merges and field overwrites;
- background attempts and failures;
- confirmation deferrals;
- active pending confirmation;
- worker state.

If status remains healthy but opdata stops, investigate background TX scheduling and catalogue handling rather than RX synchronisation alone.

### Long loop warnings

Use `section_us` to identify the source. A FastGPIO split path can legitimately spend significant time following the external clock, but sustained overruns, missing commands, or stale publications require action.

### Worker-path regression

Temporarily compare with:

```yaml
MhiAcCtrl:
  command_worker: false
```

The worker path is acceptable only when TX completion, semantic confirmation, opdata, publications, and queue health remain correct.

### Sensor remains unavailable

Many opdata fields are model-dependent. A field only becomes available after the AC returns a valid response.

### Fan or vane selection bounces back

Confirmed AC feedback is authoritative. The published selection will return to the decoded state when the unit rejects, clamps, or remaps the request.

Review the command masks, retries, and confirmation state.

## Reporting a hardware result

Attach:

- the hardware/configuration record;
- the final common diagnostic lines;
- the relevant driver-specific line;
- command test results;
- opdata behaviour;
- every non-default tuning value;
- an explanation of any non-zero counters.

A result is repeatable only when the maturity statement is tied to a specific ESP chip, board, AC model, and configuration.
