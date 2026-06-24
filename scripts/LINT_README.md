# Lint setup

## Local install

```bash
python3 -m pip install --upgrade ruff yamllint pre-commit
```

Install `clang-format` through your OS package manager.

## Run checks

```bash
scripts/lint.sh check
```

## Auto-fix formatting

```bash
scripts/lint.sh fix
```

## Optional pre-commit setup

```bash
pre-commit install
pre-commit run --all-files
```

## Scope

This lint setup checks:

- C++ headers/source under `components/`
- Python ESPHome component files under `components/`
- YAML files in the repo, excluding generated/build directories

The timing-critical FastGPIO path should stay simple and lint-clean. Lint is not a substitute for hardware timing validation.
