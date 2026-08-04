#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_ROOT = REPO_ROOT / "tests" / "components" / "MhiAcCtrl" / ".esphome" / "build"
TRANSPORT_SYMBOLS = (
    "MhiFastGpio",
    "MhiExternalClock",
    "MhiRmtSpi",
    "MhiRmtCsSpi",
)


def first_matching(root: Path, pattern: str):
    return next(root.rglob(pattern), None)


def file_size(path):
    return path.stat().st_size if path is not None and path.is_file() else None


def map_transport_symbols(map_path):
    if map_path is None or not map_path.is_file():
        return ""
    text = map_path.read_text(encoding="utf-8", errors="replace")
    return ",".join(symbol for symbol in TRANSPORT_SYMBOLS if symbol in text)


def collect_rows():
    if not BUILD_ROOT.is_dir():
        return []

    rows = []
    for build_dir in sorted(path for path in BUILD_ROOT.iterdir() if path.is_dir()):
        firmware_bin = first_matching(build_dir, "firmware.bin")
        firmware_elf = first_matching(build_dir, "*.elf")
        map_file = first_matching(build_dir, "*.map")
        rows.append(
            {
                "build": build_dir.name,
                "firmware_bin_bytes": file_size(firmware_bin),
                "firmware_elf_bytes": file_size(firmware_elf),
                "map_bytes": file_size(map_file),
                "transport_symbols": map_transport_symbols(map_file),
            }
        )
    return rows


def main():
    rows = collect_rows()
    if not rows:
        print(
            "No generated ESPHome builds found. Run ./scripts/compile-tests.sh first.",
            file=sys.stderr,
        )
        return 1

    writer = csv.DictWriter(
        sys.stdout,
        fieldnames=(
            "build",
            "firmware_bin_bytes",
            "firmware_elf_bytes",
            "map_bytes",
            "transport_symbols",
        ),
    )
    writer.writeheader()
    writer.writerows(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
