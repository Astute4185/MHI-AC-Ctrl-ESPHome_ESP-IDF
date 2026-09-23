# Protocol frame-capture diagnostic branch

## Purpose

Keep a small, reusable diagnostic branch for protocol discovery instead of rebuilding one-off logging every time an unknown MHI field appears.

Suggested branch:

```text
diagnostic/frame-capture
```

The branch must remain passive: no changes to decode, command, confirmation or Home Assistant publication behaviour. Capture runs only after a valid MOSI frame has been synchronised/classified.

## Design

The capture implementation is split in two:

- `mhi_protocol_capture_core.*` — host-testable transition engine with no ESPHome logging dependency.
- `mhi_protocol_capture.*` — thin runtime logging adapter using the `mhi.capture` tag.

A baseline is maintained per `(frame kind, opdata key)`. The next valid frame for that identity is compared with the latest baseline. Payload changes are reported as byte transitions plus XOR masks; checksum-only changes are suppressed.

## Automated coverage

The branch adds C++ unit coverage for:

- first frame becomes a baseline;
- unchanged frames are suppressed;
- payload changes report before/after/xor correctly;
- reverse transitions are compared against the latest observed state;
- checksum-only changes are ignored for both 20-byte and 33-byte layouts;
- 33-byte `DB15..DB26` payload bytes remain observable despite the legacy `CBH/CBL` gap at raw bytes 18/19 and `CBL2` at raw byte 32;
- opdata keys have independent baselines;
- frame kinds have independent baselines;
- 20/33-byte length changes re-baseline safely;
- invalid frame lengths are ignored;
- large changes are bounded to the reporting capacity while preserving total count;
- reset clears capture state;
- full slot tables recycle the oldest identity.

The Python analyser is also unit-tested for parsing, log-noise rejection, kind filtering and repeated transition aggregation.

Validation gate after rebasing the branch:

```text
./scripts/lint.sh fix
./scripts/test.sh
./scripts/compile-tests.sh
```

## Required test-runner integration

Add these to the existing host test build:

```text
tests/unit/test_protocol_capture.cpp
components/MhiAcCtrl/mhi_protocol_capture_core.cpp
```

Run the analyser test after the C++ suite:

```text
python3 tests/unit/test_analyze_frame_capture.py
```

Declare/call the C++ test functions in `tests/unit/mhi_test_common.h` and `tests/unit/mhi_unit_test_main.cpp` as shown in `existing-file-edits.patch`.

## Logging

Recommended capture logging:

```yaml
logger:
  level: DEBUG
  logs:
    mhi.capture: INFO
    mhi.diag: WARN
```

Example:

```text
[I][mhi.capture]: baseline kind=status key=0x0000 seq=101 len=33 bytes=...
[I][mhi.capture]: change kind=status key=0x0000 seq=145 changed=DB13:0x00->0x08/xor=0x08
[I][mhi.capture]: frame kind=status key=0x0000 seq=145 len=33 bytes=...
```

## Capture procedure

For each candidate feature:

1. Hold a known stable state for 20-30 seconds.
2. Change exactly one feature using the physical/IR/wired controller.
3. Hold for 10-20 seconds.
4. Revert it and hold again.
5. Repeat each transition at least three times.
6. Compare repeatable DB/bit transitions, not one-off dynamic opdata.

Use `CAPTURE_TEST_MATRIX.md` as the hardware execution/result template.

Initial targets:

- requested feature from issue #48, if either local unit exposes it;
- Self Clean, if supported;
- any controller-only state that can be isolated from mode/temp/fan/vane changes.

Hardware support remains the primary limitation. No observed transition on the available units does not prove that another MHI model lacks the feature.

## Analysis

```bash
python3 scripts/analyze_frame_capture.py capture.log
python3 scripts/analyze_frame_capture.py capture.log --kind status
python3 scripts/analyze_frame_capture.py capture.log --min-count 3
```

`--min-count 3` is useful after repeated toggles because it suppresses one-off background transitions and brings repeatable candidate bits to the top.

## Promotion criteria

Do not promote a captured field into a normal entity until the evidence establishes:

- repeatable setter transition;
- repeatable returned/confirmed state, if one exists;
- persistence across normal background frames;
- power-off/on behaviour;
- interaction with HVAC mode;
- whether values are mutually exclusive or independent flags;
- 20-byte vs 33-byte behaviour where hardware allows.
