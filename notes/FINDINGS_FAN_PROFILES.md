# Fan Profile Findings

## Summary

MHI fan capabilities are model-dependent. Testing identified one unit with three fixed speeds and another with four fixed speeds, excluding Auto.

The protocol values used by the current implementation are:

```text
0 = lowest fixed speed
1 = Low
2 = Medium
6 = High
7 = Auto
```

Reference implementations preserve protocol value `0`, but name it differently:

- `ginkage/MHI-AC-Ctrl-ESPHome` exposes it as Quiet.
- `hberntsen/mhi-ac-ctrl-esp32` exposes the lowest level separately rather than collapsing it into Low.

Redux therefore uses an explicit profile rather than automatic detection.

## Default three-speed profile

```yaml
MhiAcCtrl:
  fan_profile: three_speed
```

This is the default and exposes:

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

Protocol value `0` is retained by the decoder but collapsed to Low at the presentation layer.

## Opt-in Quiet/four-speed profile

```yaml
MhiAcCtrl:
  fan_profile: quiet_four_speed
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

## Why it is opt-in

Automatic detection is unreliable. A three-speed unit may never report protocol value `0`, while an unsupported Quiet command may be ignored, aliased, or handled differently by another model. The user must therefore select the profile that matches the physical unit.

## Validation coverage

Host tests cover:

- default profile selection;
- value `0` collapsing to Low under `three_speed`;
- value `0` exposing Quiet under `quiet_four_speed`;
- Quiet command rejection under the default profile;
- Quiet command encoding as protocol value `0` when opted in;
- preservation of raw fan value `0` by the status decoder;
- climate and select publishing for both profiles;
- TX frame encoding and checksum validity;
- Quiet command confirmation.

ESPHome compile coverage includes a dedicated ESP32-S3 `quiet_four_speed` fixture containing both the climate and fan-select entities.

## Hardware test sequence

For a new AC model, record the model number and test:

```text
Auto
Quiet (four-speed profile only)
Low
Medium
High
```

Confirm that each command is accepted and that returned MOSI state settles to the expected protocol value. Indoor fan-speed opdata is useful supporting evidence but is not a replacement for command/status confirmation.
