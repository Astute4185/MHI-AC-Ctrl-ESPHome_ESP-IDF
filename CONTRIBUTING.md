# Contributing

Contributions are welcome, especially hardware validation on additional ESP32 boards and Mitsubishi Heavy Industries air-conditioner models.

This project controls a timing-sensitive, externally clocked bus. Keep changes focused, preserve the documented ownership boundaries, and separate compile success from hardware validation.

## Before starting

- Search existing issues and pull requests for the same board, air-conditioner model, driver, or protocol field.
- Read [`ARCHITECTURE.md`](ARCHITECTURE.md) before changing transport, worker, state, command, or publication code.
- Read [the driver documentation](docs/drivers/README.md) and [`DIAGNOSTICS.md`](DIAGNOSTICS.md) before changing or validating a transport.
- Keep unrelated cleanup out of behavioural changes so regressions remain easy to isolate.

For substantial protocol or transport changes, open an issue first with the observed frames, hardware, current configuration, and intended behaviour.

## Development setup

CI currently uses Python 3.13, a C++17 compiler, `clang-format`, `shellcheck`, Ruff, Yamllint, and the ESPHome version pinned in `requirements-ci.txt`.

Install the Python dependencies in a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install --requirement requirements-ci.txt
python -m pip install ruff yamllint pre-commit
```

Install `g++`, `clang-format`, and `shellcheck` using the operating-system package manager.

Optional pre-commit setup:

```bash
pre-commit install
pre-commit run --all-files
```

## Required checks

Run these before opening a pull request:

```bash
./scripts/lint.sh fix
./scripts/test.sh
./scripts/compile-tests.sh
```

`compile-tests.sh` builds four representative ESP-IDF configurations covering ESP32-C3 FastGPIO, original ESP32 `rmt_cs_spi`, ESP32-S3 `rmt_cs_spi`, and ESP32-S3 `rmt_spi_rx` with `fast_gpio_tx`.

A successful compile does not prove that a timing-sensitive transport works on hardware.

## Architecture rules

Changes must preserve these core rules:

- The transport owns real-time bus activity.
- Only complete validated frames cross the transport boundary.
- ESPHome entities are published from the main loop, not a transport task.
- Command confirmation begins after actual transmission, not queue acceptance.
- Returned MOSI state remains authoritative.
- Newer desired state supersedes stale pending intent.
- Horizontal vane and 3D Auto state remain one composite extended-frame context.
- Full-duplex transports exclusively own both RX and TX.
- `rmt_cs_spi` remains FIFO-backed on ESP32 and ESP32-S3 unless new hardware evidence justifies a design change.
- New drivers remain self-contained and declare their own schema, target support, dependencies, compile definition, construction, hardware state, and diagnostics.
- Adding a driver must not introduce concrete-driver conditionals into `MhiAcCtrl`, `MhiTransportManager`, protocol, command, state, or entity code.
- Unselected transport implementation files remain excluded through whole-translation-unit compile guards.

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the complete ownership and lifecycle model and [`docs/drivers/developing-drivers.md`](docs/drivers/developing-drivers.md) for the driver contribution example.

## Tests

Add or update host tests for changes to:

- frame parsing, synchronisation, checksums, or classification;
- command construction, coordination, retries, or confirmation;
- driver selection and invalid configuration handling;
- fan-profile or vane mappings;
- decoded state, publication gating, or diagnostics;
- worker policy, queues, catalogues, or completion contracts.

Compile fixtures should represent a material chip, transport, frame-size, or configuration boundary. Do not add a full compile fixture for every runtime permutation when host tests can cover the difference.

## Hardware validation

Hardware-dependent changes should include the following evidence where applicable:

- ESP chip and exact board;
- air-conditioner model;
- ESPHome version;
- frame size;
- RX driver and effective TX driver;
- `command_worker` setting;
- relevant YAML configuration;
- startup configuration log;
- final transport and protocol-health counters;
- command-confirmation counters;
- commands or state transitions tested;
- test or soak duration;
- whether opdata continued to update.

Use DEBUG logging for initial validation. Reduce logging after the path is stable so logging overhead does not distort timing results.

A recovered confirmation retry is not automatically a failure. Report whether the final state confirmed, whether retries accumulated continuously, and whether any retry exhausted.

Use the hardware-validation issue form for reports that do not require a code change.

## Protocol findings

Keep baseline protocol facts, project-observed extensions, and model-specific hypotheses distinct.

When proposing a new mapping, include:

- raw byte positions and masks;
- before/after frames or logs;
- the exact user action that produced the change;
- repeatability across multiple attempts;
- air-conditioner model and frame size;
- whether the field is command, status, opdata, or unknown;
- any conflicting observations.

Update the relevant document under `notes/` when the finding is sufficiently validated.

## Pull requests

A pull request should:

- explain the problem and the chosen design;
- list the files or subsystems affected;
- include test results;
- identify hardware validation as completed, not applicable, or still required;
- update examples and documentation when configuration or behaviour changes;
- avoid committing `.esphome`, `.test-build`, logs, secrets, or generated build output.

Prefer a small sequence of reviewable pull requests over one change that combines protocol behaviour, transport timing, broad refactoring, and documentation cleanup.

## Consolidated release gate

For transport or cross-cutting runtime changes, use the consolidated gate:

```bash
./scripts/release-gate.sh validate
```

Before a release or integration into `master`, run the full compile gate:

```bash
rm -rf tests/components/MhiAcCtrl/.esphome
./scripts/release-gate.sh compile
```

The host-test script uses an explicit source manifest and fails early when a listed file was deleted or a new `test_*.cpp` file was not added. This prevents stale test-runner and linker failures after cleanup phases.

See [`TRANSPORT_REFACTOR_VALIDATION.md`](TRANSPORT_REFACTOR_VALIDATION.md) for the final software and hardware evidence checklist.
