#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-check}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ "$MODE" != "check" && "$MODE" != "fix" ]]; then
  echo "Usage: scripts/lint.sh [check|fix]" >&2
  exit 2
fi

missing=0
need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    missing=1
  fi
}

need_cmd clang-format
need_cmd python3
need_cmd yamllint

if ! python3 -m ruff --version >/dev/null 2>&1; then
  echo "Missing Python module: ruff" >&2
  missing=1
fi

if [[ "$missing" -ne 0 ]]; then
  cat >&2 <<'MSG'

Install local lint tools:
  python3 -m pip install --upgrade ruff yamllint pre-commit

Install clang-format through your OS/package manager, for example:
  brew install clang-format
  sudo apt-get install clang-format

MSG
  exit 127
fi

mapfile -t CPP_FILES < <(
  find components -type f \
    \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.c' \) \
    -not -path '*/.pioenvs/*' \
    -not -path '*/.esphome/*' \
    | sort
)

mapfile -t PY_FILES < <(
  find components -type f -name '*.py' \
    -not -path '*/.pioenvs/*' \
    -not -path '*/.esphome/*' \
    | sort
)

mapfile -t YAML_FILES < <(
  find . -type f \
    \( -name '*.yaml' -o -name '*.yml' \) \
    -not -path './.git/*' \
    -not -path './.pioenvs/*' \
    -not -path './.esphome/*' \
    -not -path './build/*' \
    -not -path './dist/*' \
    | sort
)

if [[ "$MODE" == "fix" ]]; then
  echo "Formatting C++..."
  if [[ "${#CPP_FILES[@]}" -gt 0 ]]; then
    clang-format -i "${CPP_FILES[@]}"
  fi

  echo "Formatting Python..."
  if [[ "${#PY_FILES[@]}" -gt 0 ]]; then
    python3 -m ruff check --fix "${PY_FILES[@]}"
    python3 -m ruff format "${PY_FILES[@]}"
  fi
else
  echo "Checking C++ format..."
  if [[ "${#CPP_FILES[@]}" -gt 0 ]]; then
    clang-format --dry-run --Werror "${CPP_FILES[@]}"
  fi

  echo "Checking Python lint/format..."
  if [[ "${#PY_FILES[@]}" -gt 0 ]]; then
    python3 -m ruff check "${PY_FILES[@]}"
    python3 -m ruff format --check "${PY_FILES[@]}"
  fi
fi

if [[ "${#YAML_FILES[@]}" -gt 0 ]]; then
  echo "Checking YAML..."
  yamllint "${YAML_FILES[@]}"
fi

echo "Lint complete: $MODE"
