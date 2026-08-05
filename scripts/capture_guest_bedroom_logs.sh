#!/usr/bin/env bash
set -euo pipefail

CONFIG="${1:-guest-bedroom-ac-atom-diagnostic.yaml}"
OUT_DIR="${2:-diagnostic-logs}"
STAMP="$(date +%Y%m%d-%H%M%S)"
mkdir -p "${OUT_DIR}"
LOG_FILE="${OUT_DIR}/guest-bedroom-mhi-${STAMP}.log"

printf 'Capturing %s to %s\n' "${CONFIG}" "${LOG_FILE}"
printf 'Start the Home Assistant script: script.mhi_guest_bedroom_full_input_test\n'

esphome logs "${CONFIG}" 2>&1 | tee "${LOG_FILE}"
