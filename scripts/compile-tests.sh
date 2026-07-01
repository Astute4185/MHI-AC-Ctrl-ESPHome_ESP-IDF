#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

CONFIGS=(
  "tests/components/MhiAcCtrl/test.esp32-s3-idf.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s3-idf-frame33.yaml"
  "tests/components/MhiAcCtrl/test.esp32-s3-idf-external-clock-rx.yaml"
  "tests/components/MhiAcCtrl/test.esp32-c3-idf.yaml"
  "tests/components/MhiAcCtrl/test.esp32-c3-idf-external-clock-rx.yaml"
)

for config in "${CONFIGS[@]}"; do
  echo "Compiling ${config}"
  esphome compile "${config}"
done

echo "ESPHome compile tests passed"
