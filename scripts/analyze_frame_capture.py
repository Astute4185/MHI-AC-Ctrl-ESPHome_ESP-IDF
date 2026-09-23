#!/usr/bin/env python3
"""Summarise mhi.capture byte transitions from an ESPHome log."""

from __future__ import annotations

import argparse
import collections
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

CHANGE_LINE = re.compile(
    r"change kind=(?P<kind>\S+) key=0x(?P<key>[0-9a-fA-F]+) "
    r"seq=(?P<seq>\d+) changed=(?P<changes>.*)$"
)
FIELD = re.compile(
    r"(?P<field>(?:SB|DB)\d+|CBH|CBL):"
    r"0x(?P<before>[0-9a-fA-F]{2})->0x(?P<after>[0-9a-fA-F]{2})/"
    r"xor=0x(?P<xor>[0-9a-fA-F]{2})"
)


@dataclass(frozen=True)
class Transition:
    kind: str
    key: int
    field: str
    before: int
    after: int
    xor_mask: int
    sequence: int


def parse_transitions(lines: Iterable[str], *, kind_filter: str | None = None) -> list[Transition]:
    transitions: list[Transition] = []
    for raw in lines:
        match = CHANGE_LINE.search(raw)
        if not match:
            continue
        kind = match.group("kind")
        if kind_filter and kind != kind_filter:
            continue
        key = int(match.group("key"), 16)
        sequence = int(match.group("seq"))
        for field_match in FIELD.finditer(match.group("changes")):
            transitions.append(
                Transition(
                    kind=kind,
                    key=key,
                    field=field_match.group("field"),
                    before=int(field_match.group("before"), 16),
                    after=int(field_match.group("after"), 16),
                    xor_mask=int(field_match.group("xor"), 16),
                    sequence=sequence,
                )
            )
    return transitions


def build_summary(transitions: Iterable[Transition]):
    counts: collections.Counter[tuple[str, int, str, int]] = collections.Counter()
    examples: dict[tuple[str, int, str, int], Transition] = {}
    for transition in transitions:
        identity = (transition.kind, transition.key, transition.field, transition.xor_mask)
        counts[identity] += 1
        examples.setdefault(identity, transition)
    return counts, examples


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="ESPHome log containing mhi.capture lines")
    parser.add_argument("--kind", help="Only include one catalogue kind")
    parser.add_argument("--min-count", type=int, default=1, help="Hide transitions seen fewer than N times")
    args = parser.parse_args()

    transitions = parse_transitions(args.log.read_text(errors="replace").splitlines(), kind_filter=args.kind)
    counts, examples = build_summary(transitions)
    visible = [(identity, count) for identity, count in counts.most_common() if count >= args.min_count]

    if not visible:
        print("No parsed mhi.capture changes found.")
        return 1

    print(f"Parsed {len(transitions)} field transitions\n")
    print("count  kind             key     field  xor   example")
    print("-----  ---------------  ------  -----  ----  ------------------------")
    for (kind, key, field, xor_mask), count in visible:
        example = examples[(kind, key, field, xor_mask)]
        print(
            f"{count:5d}  {kind:15.15s}  0x{key:04x}  {field:5s}  0x{xor_mask:02x}  "
            f"0x{example.before:02x}->0x{example.after:02x} seq={example.sequence}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
