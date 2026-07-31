#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

# One representative compile per material chip/transport boundary:
# - ESP32-C3 FastGPIO: legacy RX selection, 20-byte frames, three-speed fan
# - ESP32 RMT-CS SPI: original ESP32 FIFO full-duplex implementation
# - ESP32-S3 RMT-CS SPI: S3 FIFO full-duplex implementation
# - ESP32-S3 RMT/SPI RX: split hardware RX with legacy FastGPIO TX
COMPILE_CONFIGS=(
  "tests/components/MhiAcCtrl/test.esp32-c3-idf-fast-gpio.yaml"
  "tests/components/MhiAcCtrl/test.esp32-idf-rmt-cs-spi-command-worker.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s3-idf-rmt-cs-spi-command-worker.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s3-idf-rmt-spi-rx-command-worker.yaml"
)

check_configs_exist() {
  local config

  for config in "${COMPILE_CONFIGS[@]}"; do
    if [[ ! -f "${config}" ]]; then
      echo "Missing ESPHome compile-test configuration: ${config}" >&2
      exit 1
    fi
  done
}

validate_configs() {
  local config

  check_configs_exist

  for config in "${COMPILE_CONFIGS[@]}"; do
    echo "Validating ${config}"
    esphome config "${config}" >/dev/null
  done

  echo "All ESPHome compile-test configurations are valid"
}

compile_configs() {
  local config

  check_configs_exist

  for config in "${COMPILE_CONFIGS[@]}"; do
    echo "Compiling ${config}"
    esphome compile "${config}"
  done

  echo "ESPHome compile tests passed"
}

MODE="${1:-compile}"

case "${MODE}" in
  validate)
    validate_configs
    ;;

  compile)
    compile_configs
    ;;

  *)
    echo "Usage: $0 {validate|compile}" >&2
    exit 2
    ;;
esac
