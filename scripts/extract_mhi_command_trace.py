#!/usr/bin/env python3
"""Extract and summarize MHI command-confirmation traces from an ESPHome log."""
from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass, field
from pathlib import Path

TS_RE = re.compile(r"^\[(?P<time>\d{2}:\d{2}:\d{2}\.\d{3})\]")
GEN_RE = re.compile(r"generation=(?P<generation>\d+)")
MASK_RE = re.compile(r"(?:mask|pending|command_mask)=(?P<mask>0x[0-9a-fA-F]+)")
ATTEMPT_RE = re.compile(r"attempt=(?P<attempt>\d+)")
SEQ_RE = re.compile(r"seq=(?P<seq>\d+)")
SUCCESS_RE = re.compile(r"success=(?P<success>YES|NO)")
CLEAR_REASON_RE = re.compile(r"reason=(?P<reason>[^ ]+)")
CAT_VALID_RE = re.compile(r"catalog\{valid=(?P<valid>YES|NO)")
WORKER_VALID_RE = re.compile(r"worker\{valid=(?P<valid>YES|NO)")
MARKER_RE = re.compile(r"HA_MARKER: (?P<marker>.*)$")
REPLACE_RE = re.compile(
    r"old\{generation=(?P<old_gen>\d+) kind=(?P<old_kind>[^ ]+) mask=(?P<old_mask>0x[0-9a-fA-F]+).*?"
    r"new\{generation=(?P<new_gen>\d+) kind=(?P<new_kind>[^ ]+) mask=(?P<new_mask>0x[0-9a-fA-F]+)"
)

EVENT_PATTERNS = (
    ("ha_marker", "HA_MARKER:"),
    ("request", "command_trace: request"),
    ("tx_stage", "command_trace: tx_stage"),
    ("tx_mailbox_replace", "command_trace: tx_mailbox_replace"),
    ("tx_complete", "command_trace: tx_complete"),
    ("candidate", "command_trace: candidate source="),
    ("candidate_clear", "command_trace: candidate_clear"),
    ("observe", "command_trace: observe"),
    ("confirmed", "command: confirmed"),
    ("timeout", "command: confirmation timeout"),
    ("exhausted", "command: confirmation exhausted"),
)


def classify(line: str) -> str | None:
    for name, marker in EVENT_PATTERNS:
        if marker in line:
            return name
    return None


def pick(pattern: re.Pattern[str], line: str, group: str) -> str:
    match = pattern.search(line)
    return match.group(group) if match else ""


def timestamp_ms(value: str) -> int | None:
    if not value:
        return None
    h, m, rest = value.split(":")
    s, ms = rest.split(".")
    return (((int(h) * 60 + int(m)) * 60) + int(s)) * 1000 + int(ms)


def elapsed_ms(start: str, end: str) -> str:
    a = timestamp_ms(start)
    b = timestamp_ms(end)
    if a is None or b is None:
        return ""
    delta = b - a
    if delta < 0:
        delta += 24 * 60 * 60 * 1000
    return str(delta)


@dataclass
class GenerationSummary:
    generation: str
    case: str = ""
    mask: str = ""
    attempt: str = ""
    stage_time: str = ""
    completion_time: str = ""
    completion_success: str = ""
    first_candidate_time: str = ""
    candidate_count: int = 0
    candidate_sequences: list[str] = field(default_factory=list)
    clear_reasons: list[str] = field(default_factory=list)
    clear_had_candidate: bool = False
    confirmation_time: str = ""
    confirmed_masks: list[str] = field(default_factory=list)
    timeout_count: int = 0
    exhausted: bool = False
    mailbox_replaced_old: bool = False
    mailbox_replaced_by: list[str] = field(default_factory=list)

    def status(self) -> str:
        if self.exhausted:
            return "EXHAUSTED"
        if self.timeout_count:
            return "RETRIED_CONFIRMED" if self.confirmation_time else "TIMEOUT_PENDING"
        if self.confirmation_time:
            return "CONFIRMED"
        if self.completion_success == "NO":
            return "TX_FAILED"
        return "INCOMPLETE"

    def suspicion(self) -> str:
        notes: list[str] = []
        if self.clear_had_candidate:
            notes.append("candidate_present_when_cleared")
        if self.mailbox_replaced_old:
            notes.append("mailbox_envelope_replaced")
        if self.exhausted and self.candidate_count == 0:
            notes.append("exhausted_without_candidate")
        if self.exhausted and self.candidate_count > 0:
            notes.append("candidate_seen_but_not_confirmed")
        return ";".join(notes)


def parse_log(path: Path) -> tuple[list[dict[str, str]], dict[str, GenerationSummary]]:
    rows: list[dict[str, str]] = []
    generations: dict[str, GenerationSummary] = {}
    active_generation = ""
    current_case = ""

    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        event = classify(raw)
        if event is None:
            continue

        time = pick(TS_RE, raw, "time")
        generation = pick(GEN_RE, raw, "generation")
        attempt = pick(ATTEMPT_RE, raw, "attempt")
        mask = pick(MASK_RE, raw, "mask")
        sequence = pick(SEQ_RE, raw, "seq")
        marker = pick(MARKER_RE, raw, "marker")

        if event == "ha_marker":
            if " COMMAND " in f" {marker} ":
                current_case = marker
            elif " PASS " in f" {marker} " or " FAIL " in f" {marker} ":
                # Keep the result attached to the events row, then clear for the next case.
                pass
        elif generation:
            active_generation = generation

        associated_generation = generation or active_generation
        row = {
            "time": time,
            "event": event,
            "generation": associated_generation,
            "attempt": attempt,
            "mask": mask,
            "sequence": sequence,
            "case": current_case,
            "line": raw,
        }
        rows.append(row)

        if event == "tx_mailbox_replace":
            match = REPLACE_RE.search(raw)
            if match:
                old_gen = match.group("old_gen")
                new_gen = match.group("new_gen")
                if old_gen != "0":
                    old = generations.setdefault(old_gen, GenerationSummary(old_gen))
                    old.mailbox_replaced_old = True
                    old.mailbox_replaced_by.append(
                        f"{new_gen}:{match.group('new_kind')}:{match.group('new_mask')}"
                    )
            continue

        if not associated_generation or associated_generation == "0":
            if event == "ha_marker" and (" PASS " in f" {marker} " or " FAIL " in f" {marker} "):
                current_case = ""
            continue

        summary = generations.setdefault(associated_generation, GenerationSummary(associated_generation))
        if current_case and not summary.case:
            summary.case = current_case
        if mask and not summary.mask:
            summary.mask = mask
        if attempt:
            summary.attempt = attempt

        if event == "tx_stage":
            summary.stage_time = summary.stage_time or time
        elif event == "tx_complete":
            summary.completion_time = time
            summary.completion_success = pick(SUCCESS_RE, raw, "success")
        elif event == "candidate":
            summary.candidate_count += 1
            summary.first_candidate_time = summary.first_candidate_time or time
            if sequence and sequence not in summary.candidate_sequences:
                summary.candidate_sequences.append(sequence)
        elif event == "candidate_clear":
            reason = pick(CLEAR_REASON_RE, raw, "reason")
            if reason:
                summary.clear_reasons.append(reason)
            cat = pick(CAT_VALID_RE, raw, "valid")
            worker = pick(WORKER_VALID_RE, raw, "valid")
            summary.clear_had_candidate |= cat == "YES" or worker == "YES"
        elif event == "confirmed":
            summary.confirmation_time = time
            if mask:
                summary.confirmed_masks.append(mask)
        elif event == "timeout":
            summary.timeout_count += 1
        elif event == "exhausted":
            summary.exhausted = True

        if event == "ha_marker" and (" PASS " in f" {marker} " or " FAIL " in f" {marker} "):
            current_case = ""

    return rows, generations


def write_events(path: Path, rows: list[dict[str, str]]) -> None:
    fields = ["time", "event", "generation", "attempt", "mask", "sequence", "case", "line"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_generations(path: Path, generations: dict[str, GenerationSummary]) -> None:
    fields = [
        "generation", "case", "mask", "attempt", "stage_time", "completion_time",
        "tx_latency_ms", "completion_success", "first_candidate_time", "candidate_latency_ms",
        "candidate_count", "candidate_sequences", "clear_reasons", "clear_had_candidate",
        "confirmation_time", "confirmation_latency_ms", "confirmed_masks", "timeout_count",
        "exhausted", "mailbox_replaced_old", "mailbox_replaced_by", "status", "suspicion",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for generation in sorted(generations, key=lambda value: int(value)):
            item = generations[generation]
            writer.writerow({
                "generation": item.generation,
                "case": item.case,
                "mask": item.mask,
                "attempt": item.attempt,
                "stage_time": item.stage_time,
                "completion_time": item.completion_time,
                "tx_latency_ms": elapsed_ms(item.stage_time, item.completion_time),
                "completion_success": item.completion_success,
                "first_candidate_time": item.first_candidate_time,
                "candidate_latency_ms": elapsed_ms(item.completion_time, item.first_candidate_time),
                "candidate_count": item.candidate_count,
                "candidate_sequences": ";".join(item.candidate_sequences),
                "clear_reasons": ";".join(item.clear_reasons),
                "clear_had_candidate": "YES" if item.clear_had_candidate else "NO",
                "confirmation_time": item.confirmation_time,
                "confirmation_latency_ms": elapsed_ms(item.completion_time, item.confirmation_time),
                "confirmed_masks": ";".join(item.confirmed_masks),
                "timeout_count": item.timeout_count,
                "exhausted": "YES" if item.exhausted else "NO",
                "mailbox_replaced_old": "YES" if item.mailbox_replaced_old else "NO",
                "mailbox_replaced_by": ";".join(item.mailbox_replaced_by),
                "status": item.status(),
                "suspicion": item.suspicion(),
            })


def write_report(path: Path, generations: dict[str, GenerationSummary]) -> None:
    items = [generations[key] for key in sorted(generations, key=lambda value: int(value))]
    confirmed = sum(item.status() in {"CONFIRMED", "RETRIED_CONFIRMED"} for item in items)
    exhausted = sum(item.exhausted for item in items)
    retried = sum(item.timeout_count > 0 for item in items)
    suspicious = [item for item in items if item.suspicion()]
    lines = [
        "# MHI Command Trace Summary",
        "",
        f"- Generations: {len(items)}",
        f"- Confirmed: {confirmed}",
        f"- Retried: {retried}",
        f"- Exhausted: {exhausted}",
        f"- Suspicious generations: {len(suspicious)}",
        "",
        "## Suspicious generations",
        "",
    ]
    if not suspicious:
        lines.append("No candidate-clear, mailbox-replacement, or exhaustion signatures were found.")
    else:
        lines.append("| Generation | Status | Mask | Candidates | Suspicion | Case |")
        lines.append("|---:|---|---|---:|---|---|")
        for item in suspicious:
            lines.append(
                f"| {item.generation} | {item.status()} | {item.mask} | {item.candidate_count} | "
                f"{item.suspicion()} | {item.case.replace('|', '/')} |"
            )
    lines.extend([
        "",
        "## Interpretation",
        "",
        "- `candidate_present_when_cleared`: a catalog/worker confirmation candidate existed when a clear ran.",
        "- `mailbox_envelope_replaced`: the pending TX envelope was replaced before it was consumed.",
        "- `exhausted_without_candidate`: RX remained active but no command candidate reached the confirmation path.",
        "- `candidate_seen_but_not_confirmed`: candidate frames reached the path but never satisfied the pending intent.",
        "",
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--output-prefix", type=Path, default=None)
    args = parser.parse_args()

    prefix = args.output_prefix or args.log.with_suffix("")
    events_path = prefix.with_name(prefix.name + ".events.csv")
    generations_path = prefix.with_name(prefix.name + ".generations.csv")
    report_path = prefix.with_name(prefix.name + ".summary.md")

    rows, generations = parse_log(args.log)
    write_events(events_path, rows)
    write_generations(generations_path, generations)
    write_report(report_path, generations)

    print(f"Wrote {len(rows)} lifecycle events to {events_path}")
    print(f"Wrote {len(generations)} generation summaries to {generations_path}")
    print(f"Wrote investigation summary to {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
