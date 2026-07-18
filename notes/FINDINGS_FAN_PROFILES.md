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

## Compatibility alias

The earlier configuration value remains accepted:

```yaml
MhiAcCtrl:
  fan_profile: four_speed
```

It maps internally to `four_speed`. New configurations should use `four_speed` or omit `fan_profile`.

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

ESPHome compile coverage includes:

- the standard fixture with `fan_profile` omitted, validating the default four-speed path;
- an explicit `three_speed` compatibility fixture;

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
