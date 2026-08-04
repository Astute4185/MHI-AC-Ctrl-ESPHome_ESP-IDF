# Modular Transport Refactor Validation

This document records the release-readiness position for the modular transport and codebase-refinement work.

## Software gates

Run the standard validation gate:

```bash
./scripts/release-gate.sh validate
```

Run the full compile and footprint gate:

```bash
rm -rf tests/components/MhiAcCtrl/.esphome
./scripts/release-gate.sh compile \
  | tee transport-release-gate.log

python3 scripts/transport-footprint-report.py \
  | tee transport-footprint.csv
```

The release gate covers:

- C++ and Python formatting/lint checks;
- normal host unit tests;
- ASan/UBSan host unit tests;
- repository and host-test manifest hygiene;
- ESPHome configuration validation;
- representative ESP-IDF compilation when `compile` is selected.

## Architecture invariants

The automated checks enforce the following completed outcomes:

- `MhiTransportManager` is non-owning and transport-agnostic;
- codegen constructs and injects the selected primary transport;
- hardware-assisted builds include only their primary transport and internal FastGPIO recovery;
- obsolete controller transport setters are not emitted;
- transport implementation files remain in the flat component source layout;
- deleted host tests cannot remain referenced by `scripts/test.sh`;
- new C++ unit tests cannot be added without updating the explicit host-test manifest.

## Hardware validation status

Hardware validation is deferred because the device is remote. Software completion does not prove electrical or timing correctness.

Before merging the integration branch into `master`, complete or explicitly waive the following:

- preferred hardware-assisted transport startup and status reception;
- power, mode, setpoint, fan, vertical vane, and horizontal vane confirmation;
- operation-data continuity;
- Active Mode RX-only behaviour;
- forced primary failure and FastGPIO recovery;
- safe mode when both transports are unavailable;
- at least 24 hours on the preferred primary transport;
- at least 12 hours after forced FastGPIO recovery;
- no reboot, transport flapping, stale command replay, or queue growth.

Any waiver should be stated in the pull request and release notes as **hardware validation pending**.

## Release evidence

Attach or record:

- output from `./scripts/release-gate.sh compile`;
- `transport-footprint.csv`;
- ESPHome and ESP-IDF versions;
- compile targets covered;
- hardware test logs when available;
- known warnings and deferred work.
