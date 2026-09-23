#!/usr/bin/env python3
"""Fast unit tests for scripts/analyze_frame_capture.py."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "analyze_frame_capture.py"
spec = importlib.util.spec_from_file_location("analyze_frame_capture", SCRIPT)
assert spec is not None and spec.loader is not None
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def expect(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def test_parser_extracts_multiple_fields() -> None:
    lines = [
        "[I][mhi.capture]: change kind=status key=0x0000 seq=42 "
        "changed=DB10:0x00->0x01/xor=0x01,DB13:0x20->0x28/xor=0x08"
    ]
    transitions = module.parse_transitions(lines)
    expect(len(transitions) == 2, "expected two parsed transitions")
    expect(transitions[0].field == "DB10", "first field")
    expect(transitions[1].xor_mask == 0x08, "second xor")


def test_parser_ignores_non_capture_noise() -> None:
    transitions = module.parse_transitions(["normal ESPHome log", "[D][mhi.diag]: something"])
    expect(transitions == [], "noise should not parse")


def test_kind_filter_is_exact() -> None:
    lines = [
        "change kind=status key=0x0000 seq=1 changed=DB1:0x00->0x01/xor=0x01",
        "change kind=opdata key=0x8010 seq=2 changed=DB11:0x10->0x11/xor=0x01",
    ]
    transitions = module.parse_transitions(lines, kind_filter="status")
    expect(len(transitions) == 1 and transitions[0].kind == "status", "kind filter")


def test_summary_groups_repeated_xor_identity() -> None:
    lines = [
        "change kind=status key=0x0000 seq=1 changed=DB13:0x00->0x08/xor=0x08",
        "change kind=status key=0x0000 seq=2 changed=DB13:0x08->0x00/xor=0x08",
    ]
    counts, _ = module.build_summary(module.parse_transitions(lines))
    expect(counts[("status", 0, "DB13", 0x08)] == 2, "repeat count")


def main() -> int:
    test_parser_extracts_multiple_fields()
    test_parser_ignores_non_capture_noise()
    test_kind_filter_is_exact()
    test_summary_groups_repeated_xor_identity()
    print("Frame capture analyzer tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
