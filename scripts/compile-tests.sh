#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

# Fast CI gate: one representative build per material transport boundary.
REPRESENTATIVE_COMPILE_CONFIGS=(
  "tests/components/MhiAcCtrl/test.esp32-c3-idf-fast-gpio.yaml"
  "tests/components/MhiAcCtrl/test.esp32-idf-rmt-cs-spi-command-worker.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s3-idf-rmt-cs-spi-command-worker.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s3-idf-rmt-spi-rx-command-worker.yaml"
)

# Optional portability gate: compile the portable RX-only configuration on every
# Wi-Fi-capable ESP32 variant supported by ESPHome 2026.7.3. These builds prove
# source/toolchain compatibility only; they do not claim electrical or timing
# validation on physical hardware.
PORTABILITY_COMPILE_CONFIGS=(
  "tests/components/MhiAcCtrl/test.esp32-idf-portable-rx-only.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s2-idf-portable-rx-only.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s3-idf-portable-rx-only.yaml"
  "tests/components/MhiAcCtrl/test.esp32-c2-idf-portable-rx-only.yaml"
  "tests/components/MhiAcCtrl/test.esp32-c3-idf-portable-rx-only.yaml"
  "tests/components/MhiAcCtrl/test.esp32-c5-idf-portable-rx-only.yaml"
  "tests/components/MhiAcCtrl/test.esp32-c6-idf-portable-rx-only.yaml"
  "tests/components/MhiAcCtrl/test.esp32-c61-idf-portable-rx-only.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s31-idf-portable-rx-only.yaml"
)

usage() {
  cat >&2 <<'EOF'
Usage: ./scripts/compile-tests.sh [validate|compile|list] [representative|extended]

Examples:
  ./scripts/compile-tests.sh
  ./scripts/compile-tests.sh validate
  ./scripts/compile-tests.sh compile representative
  ./scripts/compile-tests.sh validate extended
  ./scripts/compile-tests.sh compile extended
  ./scripts/compile-tests.sh list extended
EOF
}

select_configs() {
  local scope="$1"

  case "${scope}" in
    representative)
      SELECTED_CONFIGS=("${REPRESENTATIVE_COMPILE_CONFIGS[@]}")
      ;;
    extended)
      SELECTED_CONFIGS=(
        "${REPRESENTATIVE_COMPILE_CONFIGS[@]}"
        "${PORTABILITY_COMPILE_CONFIGS[@]}"
      )
      ;;
    *)
      echo "Unknown compile-test scope: ${scope}" >&2
      usage
      exit 2
      ;;
  esac
}

check_configs_exist() {
  local config

  for config in "${SELECTED_CONFIGS[@]}"; do
    if [[ ! -f "${config}" ]]; then
      echo "Missing ESPHome compile-test configuration: ${config}" >&2
      exit 1
    fi
  done
}

validate_configs() {
  local config

  check_configs_exist

  for config in "${SELECTED_CONFIGS[@]}"; do
    echo "Validating ${config}"
    esphome config "${config}" >/dev/null
  done

  echo "ESPHome configuration validation passed (${SCOPE}, ${#SELECTED_CONFIGS[@]} configurations)"
}

compile_configs() {
  local config

  check_configs_exist

  for config in "${SELECTED_CONFIGS[@]}"; do
    echo "Compiling ${config}"
    esphome compile "${config}"
  done

  echo "ESPHome compile tests passed (${SCOPE}, ${#SELECTED_CONFIGS[@]} configurations)"
}

list_configs() {
  local config

  check_configs_exist
  printf '%s\n' "${SELECTED_CONFIGS[@]}"
}

MODE="${1:-compile}"
SCOPE="${2:-representative}"

# Convenience alias: `./scripts/compile-tests.sh extended`.
if [[ "${MODE}" == "extended" ]]; then
  MODE="compile"
  SCOPE="extended"
fi

select_configs "${SCOPE}"

case "${MODE}" in
  validate)
    validate_configs
    ;;
  compile)
    compile_configs
    ;;
  list)
    list_configs
    ;;
  *)
    usage
    exit 2
    ;;
esac
