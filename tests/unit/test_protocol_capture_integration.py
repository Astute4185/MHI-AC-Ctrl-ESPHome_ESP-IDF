#!/usr/bin/env python3
"""Regression checks for frame-capture runtime wiring."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUNTIME = ROOT / "components" / "MhiAcCtrl" / "mhi_rx_runtime.cpp"
TEST_SCRIPT = ROOT / "scripts" / "test.sh"


def expect(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    source = RUNTIME.read_text(encoding="utf-8")
    test_script = TEST_SCRIPT.read_text(encoding="utf-8")

    expect('#include "mhi_protocol_capture.h"' in source, "RX runtime must include capture adapter")

    ingest_pos = source.find("frame_catalog_.ingest_mosi_frame(")
    unlock_pos = source.find("unlock_catalog_();", ingest_pos)
    capture_pos = source.find("mhi_protocol_capture_frame(", ingest_pos)
    dropped_pos = source.find("if (!result.stored)", ingest_pos)

    expect(ingest_pos >= 0, "catalog ingest call missing")
    expect(unlock_pos > ingest_pos, "catalog must unlock after ingest")
    expect(capture_pos > unlock_pos, "capture must run after leaving the catalog critical section")
    expect(dropped_pos < 0 or capture_pos < dropped_pos, "capture must observe classified frames before catalog-drop handling")

    expect("components/MhiAcCtrl/mhi_protocol_capture.cpp" in test_script, "capture adapter must be host-compiled")
    expect("components/MhiAcCtrl/mhi_protocol_capture_core.cpp" in test_script, "capture core must be host-compiled")
    expect("tests/unit/test_protocol_capture.cpp" in test_script, "capture unit tests must be host-compiled")

    print("Frame capture integration tests passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
