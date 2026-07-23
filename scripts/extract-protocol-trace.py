#!/usr/bin/env python3
"""Extract bounded MHI protocol trace blocks from ESPHome logs."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path
from typing import Any, Iterable

KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]*)")
RECORD_RE = re.compile(r"trace\[(\d+)\]\s+(.*)$")
RAW_RE = re.compile(r"trace\[(\d+)\]\.raw\s+len=(\d+)\s+bytes=(.*?)\s+changed_bytes=(.*)$")
STATE_RE = re.compile(r"trace\[(\d+)\]\.state\s+(.*)$")


def _payload(line: str, marker: str) -> str | None:
    position = line.find(marker)
    if position < 0:
        return None
    return line[position:]


def _parse_key_values(text: str) -> dict[str, str]:
    return {match.group(1): match.group(2) for match in KEY_VALUE_RE.finditer(text)}


def parse_trace_lines(lines: Iterable[str]) -> list[dict[str, Any]]:
    captures: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None

    for raw_line in lines:
        line = raw_line.rstrip("\n")

        begin = _payload(line, "TRACE_BEGIN ")
        if begin is not None:
            if current is not None:
                captures.append(current)
            header = _parse_key_values(begin[len("TRACE_BEGIN ") :])
            current = {"header": header, "records": []}
            continue

        if current is None:
            continue

        summary = _payload(line, "TRACE_SUMMARY ")
        if summary is not None:
            current["summary"] = _parse_key_values(summary[len("TRACE_SUMMARY ") :])
            continue

        timing = _payload(line, "TRACE_TIMING ")
        if timing is not None:
            current["timing"] = _parse_key_values(timing[len("TRACE_TIMING ") :])
            continue

        end = _payload(line, "TRACE_END ")
        if end is not None:
            current["end"] = _parse_key_values(end[len("TRACE_END ") :])
            captures.append(current)
            current = None
            continue

        payload = _payload(line, "trace[")
        if payload is None:
            continue

        raw_match = RAW_RE.match(payload)
        if raw_match:
            index = int(raw_match.group(1))
            record = _ensure_record(current, index)
            record["frame_len"] = int(raw_match.group(2))
            record["raw_bytes"] = raw_match.group(3).strip()
            record["changed_bytes"] = raw_match.group(4).strip()
            continue

        state_match = STATE_RE.match(payload)
        if state_match:
            index = int(state_match.group(1))
            record = _ensure_record(current, index)
            record["state"] = _parse_key_values(state_match.group(2))
            continue

        record_match = RECORD_RE.match(payload)
        if record_match:
            index = int(record_match.group(1))
            record = _ensure_record(current, index)
            record.update(_parse_key_values(record_match.group(2)))

    if current is not None:
        captures.append(current)

    return captures


def _ensure_record(capture: dict[str, Any], index: int) -> dict[str, Any]:
    records: list[dict[str, Any]] = capture["records"]
    while len(records) <= index:
        records.append({"index": len(records)})
    return records[index]


def flatten_records(captures: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for capture in captures:
        header = capture.get("header", {})
        summary = capture.get("summary", {})
        timing = capture.get("timing", {})
        for record in capture.get("records", []):
            row: dict[str, Any] = {
                "capture_id": header.get("id", ""),
                "capture_label": header.get("label", ""),
                "capture_command_mask": header.get("command_mask", ""),
                "capture_generation": header.get("generation", ""),
                "capture_reason": header.get("reason", ""),
                "capture_assessment": header.get("assessment", ""),
                **{f"summary_{key}": value for key, value in summary.items()},
                **{f"timing_{key}": value for key, value in timing.items()},
                **{key: value for key, value in record.items() if key != "state"},
            }
            for key, value in record.get("state", {}).items():
                row[f"state_{key}"] = value
            rows.append(row)
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="ESPHome log file containing mhi.trace output")
    parser.add_argument("--json", dest="json_path", type=Path, help="Write structured captures as JSON")
    parser.add_argument("--csv", dest="csv_path", type=Path, help="Write one CSV row per trace record")
    args = parser.parse_args()

    captures = parse_trace_lines(args.log.read_text(encoding="utf-8", errors="replace").splitlines())
    rows = flatten_records(captures)

    if args.json_path:
        args.json_path.write_text(json.dumps(captures, indent=2) + "\n", encoding="utf-8")
    if args.csv_path:
        write_csv(args.csv_path, rows)
    if not args.json_path and not args.csv_path:
        print(json.dumps(captures, indent=2))

    print(f"Extracted {len(captures)} capture(s), {len(rows)} record(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
