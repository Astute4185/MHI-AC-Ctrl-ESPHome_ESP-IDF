## Summary

Describe what changed and why.

## Scope

List the affected subsystem, driver, protocol field, entity, documentation, or tooling.

## Validation

```text
./scripts/lint.sh fix
./scripts/test.sh
./scripts/compile-tests.sh
```

Paste the relevant results or explain why a check is not applicable.

## Hardware validation

- ESP chip and board:
- Air-conditioner model:
- Frame size:
- RX/TX transport:
- `command_worker`:
- Test duration:
- Commands or states tested:
- Final protocol/transport counters:
- Hardware validation: completed / not applicable / still required

## Risk and compatibility

Describe timing, protocol, configuration, backward-compatibility, or model-specific risks.

## Checklist

- [ ] The change is focused and does not include unrelated refactoring.
- [ ] Host tests cover new protocol or state behaviour where practical.
- [ ] Representative ESPHome configurations compile.
- [ ] Documentation and examples match any configuration or behaviour changes.
- [ ] Hardware-dependent claims include logs, counters, and test conditions.
- [ ] No secrets, generated build output, `.esphome`, `.test-build`, or unnecessary logs are committed.
