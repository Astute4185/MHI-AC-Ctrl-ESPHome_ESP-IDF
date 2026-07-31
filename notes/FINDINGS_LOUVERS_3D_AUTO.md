# Vertical Vane, Horizontal Vane, and 3D Auto Findings

## Summary

Hardware testing confirmed that vertical vane state, horizontal vane state, and 3D Auto are represented across two protocol areas:

- vertical vane uses `DB0` and `DB1` in the base frame area;
- horizontal vane and 3D Auto use `DB16` and `DB17` in the 33-byte extended frame area;
- horizontal vane and 3D Auto form one composite `DB16`/`DB17` register domain;
- 3D Auto itself remains an independent semantic bit: `DB17[2]`, mask `0x04`.

The production implementation must therefore preserve the complete horizontal/3D state whenever either field changes. A horizontal-only command must not clear 3D Auto, and a 3D-only command must not alter horizontal position or swing.

The final hardware matrix completed all 80 combinations successfully:

```text
Vertical vane matches:              80/80
Horizontal vane matches:            80/80
3D Auto matches:                    80/80
Complete requested-state matches:   80/80
Retry exhaustions:                   0
Pending confirmation mask at end:   0x00000000
```

## Bus direction and frame terminology

The air conditioner is the SPI-like bus master:

- `MOSI` is sent by the air conditioner and is used as authoritative returned status/confirmation;
- `MISO` is sent by the controller and carries requested commands;
- the bus has no conventional chip-select signal, so frame boundaries are inferred by the transport.

The byte names are used consistently in both directions. For example, a horizontal command is encoded in MISO `DB16`/`DB17`, and its result is confirmed from MOSI `DB16`/`DB17`.

## 33-byte frame positions

The extended louver fields are not inside the original 20-byte data area. They appear after the original checksum in the 33-byte frame.

| Logical field | Raw byte index | Frame area | Louver meaning |
|---|---:|---|---|
| `SB0`–`SB2` | 0–2 | Signature | Frame signature |
| `DB0` | 3 | Base data | Vertical command/set and vertical swing |
| `DB1` | 4 | Base data | Vertical fixed position |
| `DB2`–`DB14` | 5–17 | Base data | Other status/command fields |
| `CBH` / `CBL` | 18–19 | Base checksum | Checksum for the original frame area |
| `DB15` | 20 | Extended data | Extended field |
| `DB16` | 21 | Extended data | Horizontal fixed position or retained position during swing |
| `DB17` | 22 | Extended data | Horizontal swing and 3D Auto state |
| `DB18`–`DB26` | 23–31 | Extended data | Other extended fields |
| `CBL2` | 32 | Extended checksum | Final checksum byte for the 33-byte frame |

This positioning matters because 20-byte units do not expose `DB16`/`DB17`. Horizontal vane and 3D Auto commands are therefore supported only on the extended 33-byte path.

## Confirmed vertical-vane mapping

| Vertical state | `DB0` | `DB1` | Semantic confirmation |
|---|---:|---:|---|
| Up | `0x80` | `0x80` | Swing off and position 1 |
| Up/Center | `0x80` | `0x90` | Swing off and position 2 |
| Center/Down | `0x80` | `0xA0` | Swing off and position 3 |
| Down | `0x80` | `0xB0` | Swing off and position 4 |
| Swing | `0xC0` | Previous fixed position retained in feedback | `DB0[6]` set; ignore retained `DB1` position bits |

Relevant bits:

- `DB0[7]` participates in the vertical command/set form;
- `DB0[6]` is vertical swing;
- `DB1[5:4]` contains fixed vertical position;
- `DB1[7]` participates in the fixed-position command/set form.

When vertical swing is active, the returned `DB1` position bits retain the previous fixed position. They do not describe the current vane mode and must not be compared during swing confirmation.

## Confirmed horizontal-vane and 3D Auto mapping

| Horizontal state | `DB16` | `DB17`, 3D OFF | `DB17`, 3D ON |
|---|---:|---:|---:|
| Left | `0x10` | `0x0A` | `0x0E` |
| Left/Center | `0x11` | `0x0A` | `0x0E` |
| Center | `0x12` | `0x0A` | `0x0E` |
| Center/Right | `0x13` | `0x0A` | `0x0E` |
| Right | `0x14` | `0x0A` | `0x0E` |
| Wide | `0x15` | `0x0A` | `0x0E` |
| Spot | `0x16` | `0x0A` | `0x0E` |
| Swing | `retained†` | `0x0B` | `0x0F` |

Relevant `DB17` bits:

- `DB17[0]`, mask `0x01`: horizontal swing;
- `DB17[2]`, mask `0x04`: 3D Auto;
- the observed fixed-position base value is `0x0A`;
- the observed swing base value is `0x0B`.

The semantics of every set bit in the invariant `0x0A` base have not been independently identified. The tested implementation preserves the complete observed command shape instead of assigning unsupported meanings to those bits.

When horizontal swing is active, MOSI feedback retains the previous fixed position in `DB16`. `DB17[0]` is the authoritative swing indicator. The builder preserves the known raw `DB16` value when transmitting swing, and confirmation deliberately ignores `DB16` while swing is requested.

## Complete 80-combination feedback matrix

The matrix below documents the expected MOSI feedback state after each requested combination has settled. It does not imply that all four bytes are written in one MISO command frame: vertical, horizontal, and 3D commands may be staged separately, while MOSI feedback reports the resulting persistent state.

Each row represents one vertical/horizontal pair and contains both 3D states. Forty vane pairs multiplied by two 3D states gives the complete 80-case matrix.

| Matrix cases OFF / ON | Vertical | `DB0` | `DB1` | Horizontal | `DB16` | `DB17` OFF | `DB17` ON |
|---:|---|---:|---:|---|---:|---:|---:|
| 1 / 2 | Up | `0x80` | `0x80` | Left | `0x10` | `0x0A` | `0x0E` |
| 3 / 4 | Up | `0x80` | `0x80` | Left/Center | `0x11` | `0x0A` | `0x0E` |
| 5 / 6 | Up | `0x80` | `0x80` | Center | `0x12` | `0x0A` | `0x0E` |
| 7 / 8 | Up | `0x80` | `0x80` | Center/Right | `0x13` | `0x0A` | `0x0E` |
| 9 / 10 | Up | `0x80` | `0x80` | Right | `0x14` | `0x0A` | `0x0E` |
| 11 / 12 | Up | `0x80` | `0x80` | Wide | `0x15` | `0x0A` | `0x0E` |
| 13 / 14 | Up | `0x80` | `0x80` | Spot | `0x16` | `0x0A` | `0x0E` |
| 15 / 16 | Up | `0x80` | `0x80` | Swing | `retained†` | `0x0B` | `0x0F` |
| 17 / 18 | Up/Center | `0x80` | `0x90` | Left | `0x10` | `0x0A` | `0x0E` |
| 19 / 20 | Up/Center | `0x80` | `0x90` | Left/Center | `0x11` | `0x0A` | `0x0E` |
| 21 / 22 | Up/Center | `0x80` | `0x90` | Center | `0x12` | `0x0A` | `0x0E` |
| 23 / 24 | Up/Center | `0x80` | `0x90` | Center/Right | `0x13` | `0x0A` | `0x0E` |
| 25 / 26 | Up/Center | `0x80` | `0x90` | Right | `0x14` | `0x0A` | `0x0E` |
| 27 / 28 | Up/Center | `0x80` | `0x90` | Wide | `0x15` | `0x0A` | `0x0E` |
| 29 / 30 | Up/Center | `0x80` | `0x90` | Spot | `0x16` | `0x0A` | `0x0E` |
| 31 / 32 | Up/Center | `0x80` | `0x90` | Swing | `retained†` | `0x0B` | `0x0F` |
| 33 / 34 | Center/Down | `0x80` | `0xA0` | Left | `0x10` | `0x0A` | `0x0E` |
| 35 / 36 | Center/Down | `0x80` | `0xA0` | Left/Center | `0x11` | `0x0A` | `0x0E` |
| 37 / 38 | Center/Down | `0x80` | `0xA0` | Center | `0x12` | `0x0A` | `0x0E` |
| 39 / 40 | Center/Down | `0x80` | `0xA0` | Center/Right | `0x13` | `0x0A` | `0x0E` |
| 41 / 42 | Center/Down | `0x80` | `0xA0` | Right | `0x14` | `0x0A` | `0x0E` |
| 43 / 44 | Center/Down | `0x80` | `0xA0` | Wide | `0x15` | `0x0A` | `0x0E` |
| 45 / 46 | Center/Down | `0x80` | `0xA0` | Spot | `0x16` | `0x0A` | `0x0E` |
| 47 / 48 | Center/Down | `0x80` | `0xA0` | Swing | `retained†` | `0x0B` | `0x0F` |
| 49 / 50 | Down | `0x80` | `0xB0` | Left | `0x10` | `0x0A` | `0x0E` |
| 51 / 52 | Down | `0x80` | `0xB0` | Left/Center | `0x11` | `0x0A` | `0x0E` |
| 53 / 54 | Down | `0x80` | `0xB0` | Center | `0x12` | `0x0A` | `0x0E` |
| 55 / 56 | Down | `0x80` | `0xB0` | Center/Right | `0x13` | `0x0A` | `0x0E` |
| 57 / 58 | Down | `0x80` | `0xB0` | Right | `0x14` | `0x0A` | `0x0E` |
| 59 / 60 | Down | `0x80` | `0xB0` | Wide | `0x15` | `0x0A` | `0x0E` |
| 61 / 62 | Down | `0x80` | `0xB0` | Spot | `0x16` | `0x0A` | `0x0E` |
| 63 / 64 | Down | `0x80` | `0xB0` | Swing | `retained†` | `0x0B` | `0x0F` |
| 65 / 66 | Swing | `0xC0` | `retained*` | Left | `0x10` | `0x0A` | `0x0E` |
| 67 / 68 | Swing | `0xC0` | `retained*` | Left/Center | `0x11` | `0x0A` | `0x0E` |
| 69 / 70 | Swing | `0xC0` | `retained*` | Center | `0x12` | `0x0A` | `0x0E` |
| 71 / 72 | Swing | `0xC0` | `retained*` | Center/Right | `0x13` | `0x0A` | `0x0E` |
| 73 / 74 | Swing | `0xC0` | `retained*` | Right | `0x14` | `0x0A` | `0x0E` |
| 75 / 76 | Swing | `0xC0` | `retained*` | Wide | `0x15` | `0x0A` | `0x0E` |
| 77 / 78 | Swing | `0xC0` | `retained*` | Spot | `0x16` | `0x0A` | `0x0E` |
| 79 / 80 | Swing | `0xC0` | `retained*` | Swing | `retained†` | `0x0B` | `0x0F` |

\* **Vertical swing:** returned `DB1` retains the previous fixed position. Confirmation checks `DB0[6]` and ignores the retained position.

† **Horizontal swing:** returned `DB16` retains the previous fixed position. The builder preserves the known value; before extended-louver context has been learned, the fallback command value may be `0x00`. Confirmation checks `DB17[0]` and ignores retained `DB16` position bits.

## Composite horizontal/3D finding

The original failure came from treating horizontal vane and 3D Auto as unrelated commands even though both are encoded through `DB16`/`DB17`.

A horizontal-only command previously generated `DB17=0x0A` or `0x0B` without preserving bit `0x04`. If 3D Auto was already enabled, the horizontal command therefore cleared it before the explicit 3D request was processed.

The correct command-building model is:

```text
desired horizontal state = last confirmed horizontal state
desired 3D state         = last confirmed 3D state

overlay newly requested horizontal state, when present
overlay newly requested 3D state, when present

encode the complete DB16/DB17 pair
```

This produces the required preservation behaviour:

- horizontal-only commands preserve 3D Auto;
- 3D-only commands preserve horizontal position or swing;
- a combined horizontal + 3D command encodes both latest values together.

## Asymmetric confirmation semantics

Horizontal and 3D share an encoded register domain, but they do not use identical semantic confirmation rules.

| Pending command | Confirmation rule |
|---|---|
| Vertical fixed position | Vertical swing is off and returned position matches |
| Vertical swing | `DB0[6]` is set; ignore retained `DB1` position |
| Horizontal fixed position | Horizontal swing is off, returned position matches, and the preserved 3D companion state still matches |
| Horizontal swing | `DB17[0]` is set, ignore retained `DB16`, and the preserved 3D companion state still matches |
| 3D Auto only | Returned `DB17[2]` matches; horizontal position is preserved context and does not block confirmation |

This asymmetry is intentional:

- a horizontal command encodes the full composite pair and must prove that it did not unintentionally change 3D Auto;
- a 3D-only request changes one independent bit and should confirm from that bit alone.

3D state must also remain decodable when `DB16` contains an unknown fixed-position value. Invalid or unrecognised horizontal position feedback must not suppress valid `DB17[2]` state.

## Pending-command supersession

A second failure mode occurred when a horizontal command was transmitted but had not yet confirmed before a newer 3D request arrived. Waiting for the old timeout allowed obsolete intent to cross the diagnostic matrix case boundary.

The coordinator now resolves the extended-louver domain using the latest desired state:

```text
confirmed state
+ pending intent
+ newly staged intent
= latest desired composite state
```

When the latest desired state differs from the currently pending expected state:

1. settle the obsolete confirmation generation;
2. merge the latest horizontal and 3D intent;
3. transmit a combined command using mask `0x60`;
4. start a new confirmation generation for the combined state.

The final matrix exercised this path three times:

| Case | Requested state | Superseded | Replacement | Result |
|---:|---|---:|---:|---|
| 23 | Up/Center + Center/Right + 3D OFF | `0x20` | `0x60` | Confirmed |
| 27 | Up/Center + Wide + 3D OFF | `0x20` | `0x60` | Confirmed |
| 61 | Down + Spot + 3D OFF | `0x20` | `0x60` | Confirmed |

All three combined replacements confirmed in approximately 140–154 ms and did not leak stale state into the next case.

## Duplicate suppression and retry policy

The matrix reuses the same vertical value across each group of horizontal/3D combinations. Reissuing those already-confirmed vertical values previously created unnecessary command traffic and confirmation timeouts.

Vertical requests now use the same duplicate guard as horizontal and 3D requests. An already-confirmed value is ignored when there is no conflicting pending vertical generation.

Successful hardware confirmations were normally reported within a few hundred milliseconds. An unmatched extended-louver command after several seconds behaved as an ignored command rather than a slowly settling mechanical state.

The resulting retry policy is:

- horizontal confirmation window: 3 seconds;
- 3D Auto confirmation window: 3 seconds;
- retry once using the latest coalesced intent;
- report retry exhaustion if the replacement also remains unconfirmed.

The complete matrix had three transient 3D timeouts. All three confirmed after one retry. There were no retry exhaustions.

## Hardware validation

The final 80-case run verified:

- all five vertical states;
- all eight horizontal states;
- both 3D Auto states for every vertical/horizontal pair;
- composite preservation for fixed horizontal positions and horizontal swing;
- semantic swing confirmation with retained position fields;
- duplicate vertical suppression;
- early 3D retry;
- immediate horizontal/3D supersession and `0x60` coalescing.

The transport remained clean during the matrix and subsequent soak:

```text
Invalid frames:       0
Checksum failures:    0
Signature misses:     0
Sync losses:          0
Dropped bytes:        0
TX failures:          0
Queue errors:         0
RMT re-arm errors:    0
RX overwrites:        0
Completion drops:     0
```

The single `invalid_len=1` counter was present from startup and did not increase. `tx_overwritten` increased under the deliberately aggressive test matrix without producing command failures or RX corruption.

## Implementation outcome

The production implementation now follows these rules:

1. Treat `DB16`/`DB17` as one composite horizontal/3D encoding domain.
2. Preserve confirmed companion state when only one field changes.
3. Confirm 3D Auto independently from `DB17[2]`.
4. Confirm horizontal commands against both requested horizontal state and preserved 3D context.
5. Confirm swing semantically and ignore retained fixed-position fields.
6. Supersede obsolete pending extended-louver generations immediately.
7. Coalesce latest horizontal and 3D intent into mask `0x60` when required.
8. Suppress already-confirmed duplicate vertical requests.
9. Retry unconfirmed horizontal/3D commands after three seconds rather than allowing stale intent to remain pending.

## Scope and limitations

These findings were validated on the tested Mitsubishi Heavy Industries unit using 33-byte extended status frames. Other indoor-unit models may expose different extended fields or timing behaviour.

The confirmed claims in this document are limited to:

- the observed vertical, horizontal, and 3D mappings;
- the returned status behaviour during fixed and swing modes;
- the tested command-confirmation and retry behaviour;
- the successful 80-case matrix and associated transport diagnostics.

The exact purpose of every invariant bit in the extended command bytes remains undocumented and should not be inferred beyond the observed mappings above.
