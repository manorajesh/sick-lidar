"""Offline protocol tests (no scanner needed): python -m unittest discover tests"""

import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lms200 import LMS200, build_telegram, crc16  # noqa: E402

# Example telegrams printed in the SICK manuals (quick manual C.2–C.9, telegram listing §7).
MANUAL_EXAMPLES = {
    "31": "02 00 01 00 31 15 12",  # status request
    "20 24": "02 00 02 00 20 24 34 08",  # start continuous output
    "20 25": "02 00 02 00 20 25 35 08",  # stop continuous output
    "20 40": "02 00 02 00 20 40 50 08",  # 38400 baud
    "30 01": "02 00 02 00 30 01 31 18",  # request one scan
    "74": "02 00 01 00 74 50 12",  # read configuration
    "3B B4 00 32 00": "02 00 05 00 3B B4 00 32 00 3B 1F",  # 180° / 0.5°
    "20 00 53 49 43 4B 5F 4C 4D 53": "02 00 0A 00 20 00 53 49 43 4B 5F 4C 4D 53 BE C5",  # setup mode
}


def offline_lms(buf: bytes = b"", fov: int = 180) -> LMS200:
    """An LMS200 with no serial port, for exercising the parser."""
    lms = LMS200.__new__(LMS200)
    lms.verbose = False
    lms._buf = bytearray(buf)
    lms.distance_bits = 13
    lms.angular_range = fov
    return lms


def scan_reply(values, unit_mm=True) -> bytes:
    info = len(values) | ((1 if unit_mm else 0) << 14)
    data = struct.pack("<H", info) + struct.pack(f"<{len(values)}H", *values)
    body = bytes([0x02, 0x80]) + struct.pack("<H", 1 + len(data) + 1) + b"\xB0" + data + b"\x10"
    return body + struct.pack("<H", crc16(body))


class TestTelegrams(unittest.TestCase):
    def test_build_matches_manual_examples(self):
        for payload, expected in MANUAL_EXAMPLES.items():
            with self.subTest(payload=payload):
                self.assertEqual(build_telegram(bytes.fromhex(payload)), bytes.fromhex(expected))

    def test_reply_checksums_from_manual(self):
        for reply in ("02 80 03 00 A0 00 10 16 0A", "02 81 03 00 A0 00 10 36 1A"):
            frame = bytes.fromhex(reply)
            self.assertEqual(crc16(frame[:-2]), frame[-2] | frame[-1] << 8)


class TestScanDecoding(unittest.TestCase):
    def test_full_scan_mm(self):
        values = [1000 + i for i in range(361)]
        values[10] = 0x1FFF  # "no stop signal" error code
        values[20] = 2500 | (1 << 13)  # flag bit set above the 13 distance bits
        frame = scan_reply(values)
        self.assertEqual(len(frame), 732)  # matches manual §7.5.2 Table 7-29
        self.assertEqual(frame[2:4], b"\xD6\x02")

        lms = offline_lms(b"\x06\xAA" + frame)  # leading ACK and junk must be skipped
        scan = lms._decode_scan(lms._parse_one())
        self.assertEqual(len(scan.ranges_m), 361)
        self.assertEqual((scan.angles_deg[0], scan.angles_deg[1], scan.angles_deg[-1]), (0.0, 0.5, 180.0))
        self.assertTrue(scan.unit_mm)
        self.assertAlmostEqual(scan.ranges_m[0], 1.0)
        self.assertTrue(math.isnan(scan.ranges_m[10]))
        self.assertAlmostEqual(scan.ranges_m[20], 2.5)
        self.assertEqual(scan.flags[20], 1)

    def test_cm_units(self):
        lms = offline_lms(scan_reply([123] * 181, unit_mm=False))
        scan = lms._decode_scan(lms._parse_one())
        self.assertFalse(scan.unit_mm)
        self.assertAlmostEqual(scan.ranges_m[0], 1.23)

    def test_100_degree_field_starts_at_40(self):
        lms = offline_lms(scan_reply([500] * 401), fov=100)
        scan = lms._decode_scan(lms._parse_one())
        self.assertEqual((scan.angles_deg[0], scan.angles_deg[-1]), (40.0, 140.0))

    def test_corrupt_frame_is_rejected(self):
        frame = bytearray(scan_reply([500] * 181))
        frame[100] ^= 0xFF
        self.assertIsNone(offline_lms(bytes(frame))._parse_one())


if __name__ == "__main__":
    unittest.main()
