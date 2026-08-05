from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys
import tempfile

SCRIPT = Path(__file__).parents[1] / "scripts" / "extract_mhi_command_trace.py"
spec = spec_from_file_location("extract_mhi_command_trace", SCRIPT)
module = module_from_spec(spec)
sys.modules[spec.name] = module
assert spec.loader is not None
spec.loader.exec_module(module)


def test_generation_summary_detects_candidate_clear_and_exhaustion():
    log = """\
[16:00:00.000][I][mhi.test]: HA_MARKER: MHI-1 COMMAND mode=dry kind=hvac_mode requested=dry aux=
[16:00:00.010][D][mhi.diag]: command_trace: tx_stage generation=7 attempt=1 queued=YES mask=0x00000003 len=33
[16:00:00.020][D][mhi.diag]: command_trace: tx_complete generation=7 success=YES completed_at_ms=10 handled=YES command_mask=0x00000003 pending=0x00000003 attempt=1
[16:00:00.021][D][mhi.diag]: command_trace: candidate_clear reason=tx_completion_staged_confirmation generation=7 attempt=1 pending=0x00000003 catalog{valid=YES seq=55 capture_ms=11} worker{valid=NO seq=0 capture_ms=0}
[16:00:00.040][D][mhi.diag]: command_trace: candidate source=worker_store generation=7 attempt=1 seq=56 capture_ms=30 age_ms=10 pending=0x00000003
[16:00:10.020][W][mhi.diag]: command: confirmation timeout attempt=1 mask=0x00000003 retry=0x00000003 superseded=0x00000000
[16:00:30.020][W][mhi.diag]: command: confirmation exhausted after 3 attempts mask=0x00000003 superseded=0x00000000
"""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "test.log"
        path.write_text(log)
        rows, generations = module.parse_log(path)
    assert len(rows) == 7
    item = generations["7"]
    assert item.exhausted
    assert item.candidate_count == 1
    assert item.clear_had_candidate
    assert "candidate_present_when_cleared" in item.suspicion()
    assert "candidate_seen_but_not_confirmed" in item.suspicion()


def test_mailbox_replacement_is_attached_to_old_generation():
    log = """\
[16:00:00.010][D][mhi.diag]: command_trace: tx_stage generation=8 attempt=1 queued=YES mask=0x00000002 len=33
[16:00:00.012][D][mhi_rmt_cs_spi]: command_trace: tx_mailbox_replace old{generation=8 kind=command mask=0x00000002 len=33} new{generation=0 kind=background mask=0x00000000 len=33}
"""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "test.log"
        path.write_text(log)
        _, generations = module.parse_log(path)
    item = generations["8"]
    assert item.mailbox_replaced_old
    assert "mailbox_envelope_replaced" in item.suspicion()
