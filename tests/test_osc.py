"""OSC output tests (no scanner or python-osc needed): python -m unittest discover tests"""

import argparse
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lidar import osc_messages, run_osc  # noqa: E402
from lms200 import LMSError, Scan  # noqa: E402


def make_scan(ranges, start=0.0, step=1.0):
    angles = [start + i * step for i in range(len(ranges))]
    return Scan(timestamp=0.0, angles_deg=angles, ranges_m=ranges, raw=[0] * len(ranges),
                unit_mm=True, status=0x10)


class TestOscMessages(unittest.TestCase):
    def test_layout_and_values(self):
        nan = float("nan")
        scan = make_scan([2.0, nan, 1.0] + [3.0] * 178)  # 181 readings, 0°..180° at 1°
        msgs = dict(osc_messages(scan, "/lms200", hz=9.5))
        self.assertEqual(list(msgs), ["/lms200/info", "/lms200/ranges", "/lms200/x",
                                      "/lms200/y", "/lms200/nearest"])
        self.assertEqual(msgs["/lms200/info"], [181, 0.0, 1.0, 9.5])
        for key in ("/lms200/ranges", "/lms200/x", "/lms200/y"):
            self.assertEqual(len(msgs[key]), 181)  # fixed length: index = angle
            self.assertFalse(any(math.isnan(v) for v in msgs[key]))
        self.assertEqual(msgs["/lms200/ranges"][:3], [2.0, 0.0, 1.0])  # invalid -> 0
        self.assertAlmostEqual(msgs["/lms200/x"][0], 2.0)  # 0° is +x
        self.assertAlmostEqual(msgs["/lms200/y"][90], 3.0)  # 90° is +y, straight ahead
        a, r, x, y = msgs["/lms200/nearest"]
        self.assertEqual((a, r), (2.0, 1.0))
        self.assertAlmostEqual(x, math.cos(math.radians(2.0)))

    def test_no_valid_readings(self):
        msgs = dict(osc_messages(make_scan([float("nan")] * 5)))
        self.assertEqual(msgs["/lms200/nearest"], [0.0, 0.0, 0.0, 0.0])


class FakeLMS:
    angular_range, resolution = 180, 1.0

    def __init__(self, scans, then):
        self.scans, self.then, self.closed = scans, then, False

    def start_stream(self):
        pass

    def stop_stream(self):
        pass

    def close(self):
        self.closed = True

    def stream(self):
        yield from self.scans
        raise self.then


class TestReconnect(unittest.TestCase):
    def test_recovers_from_drops_and_reports_connection(self):
        scan = make_scan([1.0] * 181)
        dropped = FakeLMS([scan, scan], LMSError("stream stalled"))
        final = FakeLMS([scan], KeyboardInterrupt())
        attempts = iter([LMSError("no answer"), dropped, final])

        def open_fn(_args):
            item = next(attempts)
            if isinstance(item, Exception):
                raise item
            return item

        sent = []
        args = argparse.Namespace(prefix="lms200/")
        with self.assertLogs("lidar", level="INFO") as logs:
            run_osc(args, lambda a, v: sent.append((a, v)), open_fn=open_fn, sleep=lambda s: None)

        connected = [v for a, v in sent if a == "/lms200/connected"]
        self.assertEqual(connected, [0, 1, 0, 1, 0])  # fail, up, drop, up, Ctrl-C
        self.assertEqual(sum(a == "/lms200/ranges" for a, _ in sent), 3)
        self.assertTrue(dropped.closed and final.closed)
        warnings = [m for m in logs.output if m.startswith("WARNING")]
        self.assertEqual(len(warnings), 2)  # one per outage
        self.assertIn("no answer", warnings[0])
        self.assertIn("stream stalled", warnings[1])

    def test_long_outage_does_not_warn_on_every_retry(self):
        errors = [LMSError("no answer")] * 100
        attempts = iter(errors + [FakeLMS([], KeyboardInterrupt())])

        def open_fn(_args):
            item = next(attempts)
            if isinstance(item, Exception):
                raise item
            return item

        with self.assertLogs("lidar", level="INFO") as logs:
            run_osc(argparse.Namespace(prefix="/lms200"), lambda a, v: None,
                    open_fn=open_fn, sleep=lambda s: None, retry_s=2.0)
        warnings = [m for m in logs.output if m.startswith("WARNING")]
        self.assertEqual(len(warnings), 4)  # first failure, then about once a minute (every 30th)

    def test_send_errors_do_not_trigger_reconnect(self):
        scan = make_scan([1.0] * 181)
        lms = FakeLMS([scan, scan, scan], KeyboardInterrupt())
        opens = []

        def open_fn(_args):
            opens.append(1)
            return lms

        def send(address, values):
            raise OSError("network is unreachable")

        with self.assertLogs("lidar", level="WARNING") as logs:
            run_osc(argparse.Namespace(prefix="/lms200"), send, open_fn=open_fn, sleep=lambda s: None)
        self.assertEqual(len(opens), 1)
        self.assertEqual(len(logs.output), 1)  # reported once, not per message


if __name__ == "__main__":
    unittest.main()
