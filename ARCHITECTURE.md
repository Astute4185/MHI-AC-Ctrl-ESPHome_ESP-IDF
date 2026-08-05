# Architecture

This document describes the current runtime architecture of the ESP-IDF rewrite of MHI-AC-Ctrl for ESPHome.

It is an implementation reference, not a migration plan. The major transport, command-worker, frame-classification, command-confirmation, and state-publication changes described here are already implemented.

For configuration guidance, see [`README.md`](README.md) and [`DRIVER_SELECTION.md`](DRIVER_SELECTION.md). For runtime counters and hardware validation, see [`DIAGNOSTICS.md`](DIAGNOSTICS.md).

## Design goals

The architecture is built around the following goals:

- keep hardware-specific bus timing inside transport drivers;
- pass only complete, validated frames into the protocol layer;
- separate transport, protocol, state, command, and publication responsibilities;
- prevent ESPHome publication and logging from blocking real-time bus activity;
- start command confirmation only after a frame was actually transmitted;
- confirm commands from returned MOSI state rather than assuming that TX succeeded;
- preserve the latest desired state when commands overlap or are superseded;
- tolerate high-rate status traffic without allowing stale frames to build an unbounded backlog;
- support both split RX/TX drivers and an integrated full-duplex transport through one command contract.

## Bus model

The air conditioner is the bus master and provides:

- SCK;
- MOSI;
- MISO.

There is no physical chip-select signal. Frame boundaries must therefore be derived from the clock idle gap or reconstructed in software.

The protocol supports two configured frame sizes:

- 20-byte base frames;
- 33-byte extended frames.

The 33-byte frame exposes the extended horizontal-vane and 3D Auto fields.

## High-level structure

```text
ESPHome entities
      │
      ▼
MhiCommandState
      │
      ▼
MhiCommandCoordinator ───────► MhiTxBuilder
      │                            │
      │                            ▼
      │                       MhiTxEnvelope
      │                            │
      ▼                            ▼
semantic confirmation ◄──── MhiTransportManager
      ▲                            │
      │                            ▼
MhiStatusState ◄──── decode/apply pipeline ◄──── validated MOSI frames
      │
      ▼
MhiPublishBridge
      │
      ▼
ESPHome climate, selects, switches and sensors
```

The component is divided into five functional layers:

| Layer | Primary responsibility |
|---|---|
| Transport | Capture MOSI, drive MISO, detect transaction boundaries, and report actual TX completion |
| Frame handling | Synchronise raw input, validate complete frames, classify traffic, and control backlog |
| Protocol | Decode status and opdata; build MISO command and background frames |
| Command lifecycle | Coordinate generations, retries, duplicate suppression, supersession, and semantic confirmation |
| ESPHome integration | Apply decoded state and publish entities from the main loop |

## Runtime contexts and ownership

The implementation uses up to three runtime contexts.

| Context | Owns | Must not own |
|---|---|---|
| Transport task or driver context | Peripheral access, clock-sensitive RX/TX, transaction completion, transport queues | ESPHome entity publication, command semantics |
| Command worker | Command preparation, classified RX draining and decode for queue-backed transports | Direct ESPHome publication, hardware-specific bit timing |
| ESPHome main loop | Applying decoded state, semantic confirmation updates, publication, diagnostics and lifecycle | Long-running hardware capture when a queue-backed driver is available |

### Main-loop-only publication

`MhiPublishBridge` is called only from the ESPHome main loop. Transport tasks and the command worker may prepare data, but they do not publish climate, select, switch, binary-sensor, text-sensor, or numeric-sensor state directly.

This keeps ESPHome API activity, callbacks, logging, and publication caches outside real-time transport execution.

## Core state model

`MhiStateStore` contains three state domains:

| Domain | Purpose |
|---|---|
| `MhiStatusState` | Latest accepted control/status state returned by the air conditioner |
| `MhiOpDataState` | Latest accepted operating-data fields |
| `MhiCommandState` | Desired command fields waiting to be encoded or retransmitted |

Status state includes power, mode, fan, temperatures, vertical vane, horizontal vane, 3D Auto, error code, and the raw DB16/DB17 extended-louver context when available.

Command state is intent, not confirmed state. A command field is removed from the active build queue when encoded, then restored if staging or transmission fails.

## RX architecture

The RX path depends on whether the selected transport can safely expose queued frame data to the command worker.

### Synchronous FastGPIO RX

`fast_gpio_rx` performs timing-critical synchronous sampling inside its read path. It therefore remains main-loop driven even when `command_worker: true` is configured.

```text
FastGPIO sampling
→ MhiFrameSync
→ MhiFrameCatalog
→ decode
→ apply state
→ publish
```

This is the conservative fallback path, but it places more timing pressure on the ESPHome main loop.

### Queue-backed RX

The following drivers support classified RX in the command worker:

- `external_clock_rx`;
- `rmt_spi_rx`;
- `rmt_cs_spi`.

Their flow is:

```text
transport queue
→ bounded worker drain
→ MhiFrameSync
→ MhiFrameCatalog
→ protocol decode
→ MhiWorkerDecodedStore
→ main-loop apply
→ publish
```

The worker is notified when command or RX work is available and also polls at a bounded interval so missed notifications do not stall progress.

### Frame synchronisation

`MhiFrameSync` converts incoming bytes or chunks into complete protocol frames. It is responsible for:

- signature detection;
- 20-byte or 33-byte framing;
- checksum validation;
- partial-frame handling;
- resynchronisation after malformed input;
- protocol-health statistics.

Only complete validated frames continue into the frame catalogue.

## Frame classification and backlog control

`MhiFrameClassifier` identifies each valid MOSI frame as:

- status;
- extended status;
- opdata;
- unknown.

`MhiFrameCatalog` then stores traffic according to its semantics rather than as one undifferentiated FIFO.

### Latest-value slots

Status traffic uses latest-value storage:

- latest status;
- latest extended status;
- latest unknown frame.

If a newer status frame arrives before the older one is consumed, the older snapshot may be overwritten. This is intentional because the latest complete state is normally more useful than a queue of stale states.

### Opdata slots

Opdata frames are stored in keyed slots. This prevents one frequently updated opdata topic from replacing unrelated operating data before it is decoded.

### Command-candidate side slot

While command confirmation is pending, the catalogue also preserves the latest status or extended-status frame in a command-candidate slot.

This prevents a short-lived confirmation frame from being overwritten by later high-rate status traffic before the confirmation path observes it.

## Worker decoded store

When classified RX runs in the command worker, decoded data is transferred to the main loop through `MhiWorkerDecodedStore`.

It contains:

- latest decoded status;
- latest decoded extended status;
- latest decoded command candidate;
- merged opdata fields;
- a small bounded ring for unknown frames.

The store uses latest-value semantics for status and field-merge semantics for opdata. The main loop drains a bounded set of snapshots, applies them to `MhiStateStore`, and then publishes changes.

## Status application and validation

Decoded protocol values are not published immediately. They are first applied to the state store.

The apply layer performs:

- sanity checks;
- range checks for opdata values;
- extended-louver feedback filtering;
- command-confirmation observation;
- suspicious-change diagnostics;
- update timestamps.

A decoded status frame can therefore be valid at the checksum level but still have individual values rejected if they are outside the accepted range or fail extended-feedback acceptance rules.

## TX architecture

All TX drivers use the same transport-independent contract.

### TX envelope

`MhiTxEnvelope` contains:

- a complete 20-byte or 33-byte frame;
- frame length;
- generation number;
- background or command kind;
- encoded command mask;
- semantic command intent.

The envelope preserves the information needed to restore a command after failure and to begin semantic confirmation after real transmission.

### TX completion

`MhiTxCompletion` reports:

- generation;
- command mask and intent;
- success or failure;
- completion timestamp.

A command completion is a lifecycle event. Completion queues reject overflow rather than overwriting an older event.

### Queue acceptance is not transmission

A successful call to `queue_tx()` means that the transport accepted the envelope. It does not mean that the frame was clocked onto the MHI bus.

The command coordinator starts confirmation only after the matching successful TX completion is received.

This distinction prevents queue delays, missed bus windows, or transport failures from being reported as successful commands.

## Command lifecycle

The normal command path is:

```text
entity request
→ stage desired field in MhiCommandState
→ suppress duplicates
→ build complete frame
→ assign command generation
→ queue transport envelope
→ wait for matching TX completion
→ begin semantic confirmation
→ observe returned MOSI state
→ confirm, retry, supersede, or exhaust
```

### Command coordinator

`MhiCommandCoordinator` owns:

- command generation numbers;
- one command in flight;
- one active confirmation generation;
- attempt counters;
- restoration after queue or TX failure;
- staged-transmission timeout reporting;
- semantic confirmation expiry;
- retry restoration;
- latest-intent supersession.

Only one command generation is prepared while a previous command is in flight or awaiting confirmation.

### Command priority

User commands bypass the normal background TX interval. Background opdata traffic is deferred while command confirmation is pending.

This prevents background polling from competing with a command that still needs returned-state confirmation.

### Duplicate suppression

Requests already represented by confirmed state or current pending intent are suppressed where the protocol state is sufficiently known.

This reduces unnecessary transmissions and avoids retries caused by repeatedly sending the same vane or control value.

## Semantic confirmation

Confirmation compares returned MOSI state with the semantic intent encoded in the command.

Examples include:

- power state;
- operating mode;
- fan level;
- target temperature;
- vertical-vane fixed position or swing;
- horizontal-vane fixed position or swing;
- 3D Auto state.

The implementation does not require unrelated bytes to match.

### Swing confirmation

When vertical or horizontal swing is active, the air conditioner retains the previous fixed-position bits.

Confirmation therefore checks the swing bit and ignores the retained fixed-position value.

### Extended-louver composite state

Horizontal vane and 3D Auto share DB16/DB17 and are treated as one encoded domain.

A horizontal-only request must preserve the current 3D bit. A 3D-only request must preserve the current horizontal fixed position or swing state.

The builder derives a complete desired composite state from:

```text
confirmed extended-louver state
+ pending horizontal/3D intent
+ newly staged horizontal/3D intent
= complete DB16/DB17 command target
```

Horizontal confirmation verifies its requested horizontal state and, when present in the encoded context, the preserved 3D companion state.

3D Auto confirmation checks DB17 bit `0x04` independently. Horizontal position is preserved context, not part of the semantic 3D request.

### Latest-intent supersession

A newer request may make an older pending or in-flight value obsolete.

For independent fields, the older semantic confirmation is settled and the newer value remains queued.

For horizontal vane and 3D Auto, the coordinator coalesces the complete latest composite target. For example:

```text
pending horizontal command
+ newer 3D request
→ supersede obsolete extended confirmation
→ retain latest horizontal + 3D state
→ transmit combined command generation
```

The coordinator does not wait for the old confirmation timeout when the desired state has already changed.

## Retry and timeout model

The command lifecycle distinguishes three different failure windows.

| Failure type | Meaning |
|---|---|
| Stage failure | The transport did not accept the envelope; encoded intent is restored immediately |
| Staged timeout | A command was accepted but no matching transport completion arrived within the lifecycle timeout |
| Confirmation timeout | The frame completed transmission but returned MOSI state did not confirm the intent within the semantic window |

Confirmation expiry restores the unconfirmed semantic intent for another attempt until the maximum attempt count is reached.

Extended horizontal and 3D Auto commands use shorter confirmation windows than general control commands because successful hardware feedback is normally returned quickly.

Retry exhaustion is recorded explicitly. A recovered timeout is still useful diagnostic evidence, but it does not leave the command mask pending indefinitely.

## Transport architecture

ESPHome Python code generation selects and constructs the configured transport before C++ setup runs. `MhiAcCtrl` receives transport interface pointers and orchestrates the protocol, command, publication, and diagnostic runtimes without storing driver names, pins, or driver tuning.

`MhiTransportManager` is a non-owning runtime coordinator over:

- one codegen-owned primary `IMhiTransport`;
- an internal FastGPIO recovery transport for hardware-assisted selections;
- generic RX, TX completion, health, recovery, and safe-mode contracts.

### Split transports

Split strategies compose an RX backend with `fast_gpio_tx` or diagnostic `none` TX behind `MhiSplitTransport`.

```text
RX backend ─┐
            ├─► MhiSplitTransport ─► IMhiTransport
TX backend ─┘
```

Active split RX drivers are:

| Driver | Target position | RX model |
|---|---|---|
| `fast_gpio_rx` | Conservative default and internal recovery | Synchronous software sampling |
| `external_clock_rx` | Original ESP32 and ESP32-S3 | Interrupt/external-clock queue-backed sampling |
| `rmt_spi_rx` | ESP32-S3 | RMT boundary detection plus DMA-backed SPI receive |

`fast_gpio_tx` remains the transmit implementation for split control configurations.

### Integrated full-duplex transport

`rmt_cs_spi` is adapted to the common interface through `MhiDuplexTransportAdapter`. The backend remains responsible for:

- RMT inter-frame gap detection;
- internally derived chip-select timing;
- SPI2 slave configuration;
- SCK, MOSI and MISO;
- RX/TX transaction buffers;
- the SPI owner task;
- completed RX transactions;
- TX completion reporting.

It always uses `SPI_DMA_DISABLED`. The supported MHI frame sizes fit within the SPI slave FIFO transaction capacity.

The original ESP32 applies a transport-local mode-3 receive-edge correction. ESP32-S3 uses the standard ESP-IDF mode-3 configuration.

A separate `tx_driver` is not valid when `rmt_cs_spi` is selected.

### Compile-time selection and recovery

The Python transport registry owns target validation, driver-specific schema, ESP-IDF dependencies, compile definitions, and object construction. Known-invalid target selections fail configuration instead of silently substituting another driver.

Hardware-assisted primary selections compile an internal FastGPIO recovery path automatically. Runtime setup failure, no traffic, invalid traffic, or a sustained traffic stall can activate that recovery path. Recovery is latched until reboot. If both primary and recovery fail, the component enters stable transport safe mode with TX and commands disabled while diagnostics remain available.

## Command-worker modes

### `command_worker: false`

The main loop owns command coordination and all RX decode work.

This mode is useful for:

- compatibility comparison;
- isolating worker-specific regressions;
- synchronous FastGPIO operation.

### `command_worker: true`

The worker owns command preparation for every transport.

For queue-backed RX drivers, it also owns:

- bounded RX queue draining;
- frame synchronisation;
- frame classification;
- protocol decode into the worker store.

The main loop still owns:

- applying decoded snapshots;
- updating command confirmation from applied status;
- publishing ESPHome state;
- lifecycle and diagnostic reporting.

## Concurrency and synchronisation

The architecture uses explicit ownership and bounded handoff structures rather than sharing peripheral state freely across contexts.

Key synchronisation mechanisms include:

- a command mutex around command state and coordinator transitions;
- critical sections around the frame catalogue;
- critical sections around the worker decoded store;
- transport-owned queue protection;
- task notifications for command-worker wake-up;
- atomic worker lifecycle and diagnostic counters.

Spinlock-protected sections copy or swap bounded data only. Logging, decoding, publication, and other potentially slow operations are performed outside critical sections.

## Backpressure policy

Different data classes use different loss policies.

| Data class | Policy | Rationale |
|---|---|---|
| Status snapshots | Overwrite with latest | Stale state has low value |
| Extended status | Overwrite with latest | Latest louver/control state is authoritative |
| Command candidate | Preserve latest while confirmation is pending | Avoid losing short-lived confirmation feedback |
| Opdata | Keyed slots or field merge | Preserve unrelated operating-data topics |
| Unknown frames | Small bounded latest/ring storage | Diagnostic only |
| TX completions | Never overwrite older events | Lifecycle events must remain ordered and visible |
| Pending transport TX frame | Transport-specific bounded latest/queue behaviour with diagnostics | Bus windows are finite; command intent is restored on failure |

All overwrite, drop, high-water, and queue-failure conditions are exposed through diagnostics.

## Startup and shutdown

During setup the component:

1. resets diagnostics, command lifecycle, frame synchronisation and catalogues;
2. creates the command mutex;
3. configures frame size and TX runtime state;
4. configures and starts the selected transport;
5. determines whether classified worker RX is supported;
6. registers external-temperature callbacks;
7. starts the command worker when enabled.

During shutdown:

1. new work is rejected;
2. the command worker is asked to stop;
3. teardown waits for the worker lifecycle to settle;
4. the transport is shut down;
5. the command mutex is deleted.

The transport is stopped only after worker ownership has been released.

## Diagnostics and observability

Diagnostics are part of the architecture rather than an external add-on.

The component records:

- valid and invalid frames;
- checksum, signature, synchronisation and dropped-byte health;
- transport TX frames and failures;
- command staging, completion, confirmation, retries and exhaustion;
- frame-catalogue writes and overwrites;
- worker drain, decode, runtime and stack metrics;
- queue depths, high-water marks, overwrites and drops;
- main-loop and section timing;
- driver-specific RMT/SPI or external-clock counters.

See [`DIAGNOSTICS.md`](DIAGNOSTICS.md) for interpretation and soak-test criteria.

## Design invariants

Changes to the component should preserve these rules:

1. Only complete checksum-valid frames cross from frame synchronisation into protocol classification.
2. Hardware-specific timing remains inside transport drivers.
3. ESPHome entity publication occurs only in the main loop.
4. Queue acceptance is not treated as completed TX.
5. Command confirmation starts only after matching successful transport completion.
6. Command confirmation is semantic and ignores unrelated retained protocol fields.
7. A failed queue or TX lifecycle restores command intent.
8. Newer desired state supersedes obsolete pending state.
9. Horizontal vane and 3D Auto are encoded as one complete DB16/DB17 target.
10. Background traffic does not overtake a command awaiting confirmation.
11. Status backlog remains bounded through latest-value storage.
12. TX completion events are not silently overwritten.
13. Critical sections remain bounded and do not contain logging, decode, or publication work.
14. Runtime fallback must be visible through logs and diagnostics.

## Adding or changing a transport

Transport implementations remain flat under `components/MhiAcCtrl/` because arbitrary internal source subfolders are not relied on for compilation discovery. Each backend is a self-contained module with consistently prefixed C++ files and a driver-owned Python definition.

The portability boundary is the common transport contract. A backend owns its schema, target policy, dependencies, construction, pins, peripherals, tasks, queues, buffers, timing, target workarounds, health, and hardware counters. The controller, manager, protocol, command, state, and entity layers consume only `IMhiTransport`, `IMhiRxDriver`, `IMhiTxDriver`, or `IMhiDuplexTransport` contracts.

Adding a transport may require declarative wiring in `mhi_transport_registry.py`, `mhi_transport_codegen.py`, and `driver_selection.py`. It must not require concrete-driver branches or members in `MhiAcCtrl`, `MhiTransportManager`, protocol decoders, the command coordinator, or ESPHome entity platforms.

A new transport should:

- choose a split `IMhiRxDriver`/`IMhiTxDriver` shape or an integrated `IMhiDuplexTransport` shape;
- register its public name, nested schema, target support, dependencies, compile definition, builder, and recovery policy in its Python transport definition;
- construct a complete `IMhiTransport` strategy through `MhiSplitTransport` or `MhiDuplexTransportAdapter`;
- own runtime pins, peripheral configuration, queues, buffers, and hardware-specific health;
- return copied, complete, bounded chunks without exposing peripheral-owned buffers after return;
- report real TX completion through `MhiTxCompletion`, never queue acceptance alone;
- keep RX active and clear staged TX when Active Mode is disabled;
- expose queue depth, overwrite, drop, completion, and hardware error counters;
- declare capabilities such as classified-worker safety and marker-owned TX accurately;
- use a whole-translation-unit compile guard so unselected implementations are absent from the build;
- avoid ESPHome publication, protocol decode, command building, retries, or semantic confirmation;
- preserve the common command envelope and generation contract;
- include target-specific compile coverage and source-manifest checks;
- include hardware validation showing clean protocol, TX completion, command confirmation, recovery, and soak behaviour.

Hardware support remains driver-specific. Portability does not bypass target constraints: unsupported chip/framework combinations are declared by the driver and rejected during ESPHome configuration.

See [`DRIVER_SELECTION.md#developing-a-new-transport`](DRIVER_SELECTION.md#developing-a-new-transport) for a concrete split-driver example and the required registration, codegen, testing, and hardware-validation steps.

## Validation model

Architecture changes are validated at several layers:

| Layer | Validation |
|---|---|
| Protocol and state logic | Native C++ unit tests |
| Configuration and driver policy | Python tests and ESPHome config validation |
| Target-specific code | Representative ESPHome compile matrix |
| Memory and undefined behaviour | Sanitizer-enabled unit tests |
| Transport correctness | Hardware command tests and soak runs |
| Extended louver behaviour | Full vertical/horizontal/3D Auto matrix |

Representative compilation currently covers:

- ESP32-C3 FastGPIO configuration;
- original ESP32 `rmt_cs_spi` with command worker;
- ESP32-S3 `rmt_cs_spi` with command worker;
- ESP32-S3 `rmt_spi_rx` with `fast_gpio_tx` and command worker.

Compilation proves target compatibility, not runtime bus correctness. Hardware transports require real-device testing.

## Current implementation status

The major architectural work is complete:

- transport interfaces are split from protocol and publication;
- queue-backed classified RX is implemented;
- command generations use real TX completion;
- semantic confirmation, retries, duplicate suppression and supersession are implemented;
- frame traffic uses bounded semantic storage;
- horizontal vane and 3D Auto composite handling is hardware-validated;
- FIFO-backed `rmt_cs_spi` supports the original ESP32 and ESP32-S3.

Remaining work is primarily wider hardware validation, protocol discovery, documentation maintenance, and incremental hardening rather than another architectural migration.

## Related documents

- [`README.md`](README.md) — configuration and user-facing project status
- [`DRIVER_SELECTION.md`](DRIVER_SELECTION.md) — driver combinations and target guidance
- [`DIAGNOSTICS.md`](DIAGNOSTICS.md) — runtime counters and hardware validation
- [`notes/FINDINGS_MHI_PROTOCOL.md`](notes/FINDINGS_MHI_PROTOCOL.md) — consolidated MHI bus and protocol findings
- [`notes/FINDINGS_LOUVERS_3D_AUTO.md`](notes/FINDINGS_LOUVERS_3D_AUTO.md) — extended-louver protocol findings
- [`notes/FINDINGS_SPI_TRANSPORTS.md`](notes/FINDINGS_SPI_TRANSPORTS.md) — current SPI transport findings
- [`notes/FINDINGS_FAN_PROFILES.md`](notes/FINDINGS_FAN_PROFILES.md) — fan-profile findings
- [`notes/history/FINDINGS_FASTGPIO_EXTERNAL_CLOCK.md`](notes/history/FINDINGS_FASTGPIO_EXTERNAL_CLOCK.md) — historical worker and transport experiments

## Runtime Active Mode

Active Mode is a controller-level transmit gate implemented through the generic
transport contract. Disabling it does not stop RX or transport health
processing. The controller first blocks command generation and clears command
coordinator state; the active transport then clears staged TX and completions.
Re-enabling performs another command-state reset before transmission is allowed,
which prevents stale command replay.

The transport manager preserves the selected Active Mode state across primary
to FastGPIO recovery transitions. Safe mode always forces Active Mode off.



## Operation-data freshness

`MhiOpDataFreshnessTracker` maintains one timestamp for each enabled opdata
request bit. Accepted decoder results refresh only their corresponding request;
other operation-data traffic cannot conceal a missing field. The tracker
reports observed, pending, and stale masks, the age of the oldest enabled
request, and a transition-based timeout counter.

The controller resets the observation window when the active transport changes
but retains the cumulative timeout counter. `MhiOpDataFreshnessPublisher` owns
the optional diagnostic entities and suppresses unchanged publication. Existing
opdata values remain available when stale; freshness is diagnostic metadata and
does not alter the request scheduler, decoder, state store, or entity values.

## Estimated power and energy

`MhiPowerEstimator` is a derived-data service fed only by accepted CT/current
opdata. It calculates instantaneous power from configurable nominal voltage and
power factor. An optional standby floor is applied only when normal status has
confirmed that the unit is powered off. No fixed standby assumption is embedded
in the protocol decoder.

Energy uses trapezoidal integration between consecutive current samples.
Intervals beyond the configured maximum are skipped instead of integrating a
stale value. Transport transitions reset only the sample window; the current
boot-session energy total and diagnostic counters remain intact.

Derived values are stored alongside opdata for publication, but remain distinct
from native `energy_used`. Enabling an estimated power or energy entity adds the
CT request bit to the opdata mask. The estimator is otherwise inactive and adds
no protocol request.

## Architecture enforcement

The transport ownership boundaries are enforced by host-side repository tests in `tests/unit/test_transport_compile_selection.py` and `tests/unit/test_repository_hygiene.py`.

The checks prevent:

- concrete transport ownership returning to `MhiTransportManager`;
- obsolete driver and pin setters returning to `MhiAcCtrl` codegen;
- transport implementation files moving into unsupported arbitrary source subfolders;
- stale deleted unit-test files remaining in the explicit host-test build manifest.

Run `./scripts/release-gate.sh validate` after architectural cleanup and `./scripts/release-gate.sh compile` before release integration.
