# TX Staged Command Replacement Spike

## Status

Experimental spike for `rmt_cs_spi` only.

This is not a replacement for the existing command confirmation and retry model. It narrows the period between command staging and hardware ownership so rapid user changes can be collapsed before transmission.

## Problem

The command coordinator currently treats a command as in flight immediately after `queue_tx()` accepts it. For the duplex transport, acceptance can mean either:

1. the command is still waiting in the software TX mailbox; or
2. the transport task has already copied it into a queued SPI transaction.

A newer user request arriving in state 1 cannot currently replace the staged command, even though the mailbox is still software-owned. The old value is transmitted first and the new value waits for completion or confirmation handling.

## Spike Behaviour

The spike adds an atomic replacement contract:

```text
latest user patch
    -> rebuild staged command plus latest desired state
    -> replace only if expected generation is still in mailbox
    -> commit replacement generation to coordinator
```

If the transport has already claimed the generation, replacement returns `NOT_PENDING`. The speculative build is discarded and the existing completion and confirmation lifecycle continues unchanged.

## Safety Invariants

- A queued SPI transaction buffer is never modified.
- Replacement requires an exact expected generation match.
- Replacement is command-to-command only.
- Routine background frames cannot displace a command.
- The command state and TX runtime are changed only after the transport confirms replacement.
- A failed replacement leaves the original in-flight generation and the newer queued command intact.
- Semantic confirmation still starts only after an actual TX completion.

## Runtime Handling

The coordinator stores the TX runtime snapshot from immediately before the original command build. Replacement rebuilds from that snapshot so:

- the same double-frame phase is used;
- operation-data counters are not advanced twice;
- the replacement represents one physical TX opportunity, not an additional background cycle.

## Diagnostics

The existing `runtime: tx_priority` line now includes:

- `staged_replace=<successes>/<attempts>`
- `claimed_miss=<count>`
- `unsupported=<count>`
- `rejected=<count>`

The `mhi_rmt_cs_spi` runtime line includes:

- `tx_replaced=<count>`

A successful replacement also logs:

```text
command: replaced staged generation=<old>-><new> mask=<mask> ...
```

A claimed race logs at debug level:

```text
command: staged replacement missed generation=<old>; transport already claimed frame
```

## Hardware Validation

Use the normal hardware command validation script with command gaps short enough to occasionally submit a second request before the next bus transaction.

Recommended cases:

1. fan `Quiet -> Low -> Medium -> High` with 50-150 ms request gaps;
2. mode plus target temperature submitted as separate actions;
3. horizontal vane followed immediately by 3D Auto;
4. repeated same-field changes before the next AC frame;
5. normal 10-second command gaps as a regression control.

Validate:

- successful replacement logs appear only for rapid changes;
- `tx_replaced` matches `staged_replace` successes;
- old replaced generations never produce accepted completions;
- claimed misses still complete and confirm normally;
- no increase in TX failures, completion drops, or retry exhaustion;
- final published state matches the latest requested value;
- opdata freshness remains stable.

## Merge Decision

Push beyond the spike only if hardware testing shows one of these outcomes:

- rapid user changes produce successful replacements and fewer stale intermediate transmissions; or
- replacement diagnostics provide useful proof that the mailbox ownership window is material.

Do not merge if almost every attempt reports `claimed_miss`. That would show the mailbox window is too short to justify the additional coordinator and transport complexity.
