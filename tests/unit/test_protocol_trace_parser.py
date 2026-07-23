import importlib.util
import pathlib
import unittest

SCRIPT = pathlib.Path(__file__).parents[2] / "scripts" / "extract-protocol-trace.py"
SPEC = importlib.util.spec_from_file_location("extract_protocol_trace", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class ProtocolTraceParserTest(unittest.TestCase):
    def test_parses_capture_records_raw_and_state(self):
        lines = [
            "[I][mhi.trace]: TRACE_BEGIN id=3 label=fan_high command_mask=0x4 "
            "generation=9 reason=confirmed assessment=confirmed records=2",
            "[I][mhi.trace]: TRACE_SUMMARY tx_attempts=1 tx_successes=1 confirmations=1 same_transaction_rx=YES",
            "[I][mhi.trace]: TRACE_TIMING request_to_stage_us=100 stage_to_wire_us=200 tx_to_first_rx_us=500",
            "[I][mhi.trace]: trace[0] observed_us=10 event_us=10 elapsed_us=0 "
            "catalog_seq=1 bus_seq=1 event=COMMAND_REQUESTED dir=NONE gen=0 "
            "mask=0x4 pending=0x0 success=YES changed=0x0 label=component",
            "[I][mhi.trace]: trace[1] observed_us=20 event_us=20 elapsed_us=10 "
            "catalog_seq=2 bus_seq=2 event=MOSI_STABLE_STATUS "
            "dir=MOSI_AC_TO_CONTROLLER gen=9 mask=0x4 pending=0x0 "
            "success=YES changed=0x20 label=",
            "[I][mhi.trace]: trace[1].raw len=33 bytes=6d 80 04 changed_bytes=DB1",
            "[I][mhi.trace]: trace[1].state fields=0x7 power=1 mode=2 fan=6 target=22.0",
            "[I][mhi.trace]: TRACE_END id=3",
        ]
        captures = MODULE.parse_trace_lines(lines)
        self.assertEqual(len(captures), 1)
        self.assertEqual(captures[0]["header"]["label"], "fan_high")
        self.assertEqual(captures[0]["header"]["assessment"], "confirmed")
        self.assertEqual(captures[0]["summary"]["same_transaction_rx"], "YES")
        self.assertEqual(captures[0]["timing"]["tx_to_first_rx_us"], "500")
        record = captures[0]["records"][1]
        self.assertEqual(record["event"], "MOSI_STABLE_STATUS")
        self.assertEqual(record["changed_bytes"], "DB1")
        self.assertEqual(record["state"]["fan"], "6")

        rows = MODULE.flatten_records(captures)
        self.assertEqual(rows[1]["capture_id"], "3")
        self.assertEqual(rows[1]["capture_assessment"], "confirmed")
        self.assertEqual(rows[1]["summary_tx_attempts"], "1")
        self.assertEqual(rows[1]["timing_stage_to_wire_us"], "200")
        self.assertEqual(rows[1]["state_target"], "22.0")


if __name__ == "__main__":
    unittest.main()
