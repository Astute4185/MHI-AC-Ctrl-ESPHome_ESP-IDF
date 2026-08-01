# Fan Profile Findings

## Summary

The MHI protocol provides four fixed fan values plus Auto:

```text
0 = Quiet / lowest fixed speed
1 = Low
2 = Medium
6 = High
7 = Auto
```

Initial testing suggested that one AC exposed only three fixed speeds. Further hardware testing showed that both available AC models accept protocol value `0`, return it in MOSI status, and distinguish it from Low. Quiet, Low, Medium, High, and Auto all completed command confirmation successfully.

Redux therefore defaults to the four-speed profile.

## Protocol notation

The supplied protocol reference describes four fixed fan levels using
`DB1[1:0]` and `DB6[6]`, with a special MISO set form for the highest fixed
speed. The component's normalized values (`0`, `1`, `2`, `6`, `7`) are an
implementation-level code assembled from the relevant status fields; they are
not the same numbering convention as the legacy user-facing levels `1..4`.

Observed returned DB1 command/status bytes on the tested four-speed path were:

| Presentation state | Returned DB1 | Normalized code |
|---|---:|---:|
| Quiet | `0x08` | `0` |
| Low | `0x09` | `1` |
| Medium | `0x0A` | `2` |
| High | `0x0E` | `6` |
| Auto | `0x0F` | `7` |

See [`FINDINGS_MHI_PROTOCOL.md`](FINDINGS_MHI_PROTOCOL.md) for the base field
layout and direction terminology.

## Default four-speed profile

No explicit setting is required:

```yaml
MhiAcCtrl:
  id: mhi_ac
```

The equivalent explicit configuration is:

```yaml
MhiAcCtrl:
  fan_profile: four_speed
```

This exposes:

```text
Auto
Quiet
Low
Medium
High
```

Mapping:

```text
MOSI 0 -> Quiet
MOSI 1 -> Low
MOSI 2 -> Medium
MOSI 6 -> High
MOSI 7 -> Auto

TX Quiet  -> 0
TX Low    -> 1
TX Medium -> 2
TX High   -> 6
TX Auto   -> 7
```

## Three-speed compatibility profile

A model that genuinely does not support Quiet can opt out explicitly:

```yaml
MhiAcCtrl:
  fan_profile: three_speed
```

This exposes:

```text
Auto
Low
Medium
High
```

Mapping:

```text
MOSI 0 -> Low
MOSI 1 -> Low
MOSI 2 -> Medium
MOSI 6 -> High
MOSI 7 -> Auto

TX Low    -> 1
TX Medium -> 2
TX High   -> 6
TX Auto   -> 7
```

Protocol value `0` remains preserved by the status decoder but is collapsed to Low at the presentation layer.

## Validation coverage

Host tests cover:

- four-speed selection as the C++ fallback/default;
- canonical `four_speed` naming;
- value `0` exposing Quiet under `four_speed`;
- value `0` collapsing to Low under `three_speed`;
- Quiet command rejection under `three_speed`;
- Quiet command encoding as protocol value `0` under `four_speed`;
- preservation of raw fan value `0` by the status decoder;
- climate and select publishing for both profiles;
- TX frame encoding and checksum validity;
- Quiet command confirmation.

ESPHome compile coverage uses the representative matrix rather than separate builds for every profile:

- the ESP32-C3 FastGPIO fixture explicitly covers `fan_profile: three_speed` with 20-byte frames;
- the ESP32 and ESP32-S3 SPI fixtures cover the default or explicit four-speed path with 33-byte frames.

Profile encoding, decoding, command rejection, publishing and confirmation remain covered by host unit tests.

## Hardware validation

Hardware testing confirmed the following returned command values:

```text
Quiet  -> DB1 0x08
Low    -> DB1 0x09
Medium -> DB1 0x0A
High   -> DB1 0x0E
Auto   -> DB1 0x0F
```

Each mode was accepted, confirmed from returned MOSI state, and published correctly to both the climate entity and fan-speed select on the tested four-speed unit. Subsequent testing showed the unit initially believed to be three-speed also supported the distinct Quiet state.

New AC models should still be checked for command acceptance, returned status, climate/select synchronisation, and unintended state-publishing regressions.
