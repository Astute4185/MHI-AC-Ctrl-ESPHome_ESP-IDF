#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

MODE="${1:-validate}"

case "${MODE}" in
  validate|compile)
    ;;
  *)
    echo "Usage: $0 [validate|compile]" >&2
    exit 2
    ;;
esac

./scripts/lint.sh check
./scripts/test.sh
SANITIZERS=1 ./scripts/test.sh
./scripts/compile-tests.sh validate

if [[ "${MODE}" == "compile" ]]; then
  ./scripts/compile-tests.sh compile
  python3 ./scripts/transport-footprint-report.py
fi

echo "Release gate passed: ${MODE}"
