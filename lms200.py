"""Driver for the SICK LMS200 (and LMS2xx family) over RS-232.

Protocol reference: SICK "Telegram listing" LMS2xx (8007954) and SICK "Quick
Manual for LMS communication setup". Section numbers in comments refer to the
telegram listing. See README.md for how each part maps to the manuals.

Telegram frame (little-endian):
    STX(0x02) ADDR LEN_L LEN_H CMD [DATA...] CRC_L CRC_H
LEN counts CMD + DATA. Replies from the LMS carry ADDR | 0x80, CMD | 0x80,
and a status byte just before the CRC. Each accepted request is preceded by
an ACK (0x06); a bad checksum gets a NAK (0x15).
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass, field

import serial
from serial.tools import list_ports

STX, ACK, NAK = 0x02, 0x06, 0x15
KEYSPAN_VID = 0x06CD  # Keyspan / InnoSys (USA-19HS reports PID 0x0121)


def find_port() -> str:
    """Locate the Keyspan adapter. Its /dev name embeds an IORegistry ID that
    changes on every re-plug or sleep/wake, so it can't be hardcoded."""
    ports = list_ports.comports()
    for p in ports:
        if p.vid == KEYSPAN_VID:
            return p.device
    usb = [p.device for p in ports if p.vid is not None]
    if len(usb) == 1:
        return usb[0]
    raise serial.SerialException(
        "Keyspan adapter not found. Is it plugged in? "
        f"Serial ports seen: {', '.join(p.device for p in ports) or 'none'}"
    )


# Command 20h sub-commands (section 7.4.1)
BAUD_SUBCMD = {38400: 0x40, 19200: 0x41, 9600: 0x42}  # 500k (0x48) needs RS-422
MODE_CONTINUOUS_ALL = 0x24  # stream every scan
MODE_ON_REQUEST = 0x25  # power-on default: only answer 30h requests

# Command 77h block D -> number of distance bits (section 7.46.1)
MEASURING_MODE_BITS = {0x00: 13, 0x01: 13, 0x02: 13, 0x03: 14, 0x04: 14, 0x05: 15, 0x06: 15, 0x0F: 15}


def crc16(data: bytes) -> int:
    """SICK LMS2xx CRC (section 9): poly 0x8005 over a sliding 2-byte window."""
    crc = 0
    prev = 0
    for b in data:
        window = (prev << 8) | b
        prev = b
        if crc & 0x8000:
            crc = ((crc & 0x7FFF) << 1) ^ 0x8005
        else:
            crc <<= 1
        crc ^= window
        crc &= 0xFFFF
    return crc


def build_telegram(payload: bytes, addr: int = 0x00) -> bytes:
    body = bytes([STX, addr]) + struct.pack("<H", len(payload)) + payload
    return body + struct.pack("<H", crc16(body))


class LMSError(RuntimeError):
    pass


@dataclass
class Telegram:
    addr: int
    cmd: int
    data: bytes  # between CMD and the status byte
    status: int


@dataclass
class Scan:
    timestamp: float
    angles_deg: list[float]
    ranges_m: list[float]  # NaN where the reading is an overflow/error code
    raw: list[int]  # raw 16-bit words, flags included
    unit_mm: bool
    status: int
    partial_index: int | None = None  # interlaced mode: 0..3 for x.00/.25/.50/.75
    flags: list[int] = field(default_factory=list)

    def points_xy(self) -> list[tuple[float, float]]:
        """Cartesian points in metres; 90° points straight ahead of the scanner (+y)."""
        return [
            (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
            for a, r in zip(self.angles_deg, self.ranges_m)
            if not math.isnan(r)
        ]


class LMS200:
    def __init__(self, port: str | None = None, verbose: bool = False):
        self.verbose = verbose
        port = port or find_port()
        self.log("using port", port)
        self.ser = serial.Serial(
            port, 9600, bytesize=8, parity="N", stopbits=1, timeout=0.05,
            xonxoff=False, rtscts=False, dsrdtr=False,
        )
        self._buf = bytearray()
        self.distance_bits = 13
        self.unit_mm = True
        self.angular_range = 180
        self.resolution = 0.5
        self.config: bytes | None = None

    # ---------- low-level I/O ----------

    def log(self, *args):
        if self.verbose:
            print("[lms]", *args, flush=True)

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _fill(self) -> bool:
        chunk = self.ser.read(max(1, self.ser.in_waiting))
        if chunk:
            self._buf += chunk
            return True
        return False

    def flush_input(self):
        self.ser.reset_input_buffer()
        self._buf.clear()

    def send(self, payload: bytes):
        tel = build_telegram(payload)
        self.log("TX", tel.hex(" "))
        self.ser.write(tel)
        self.ser.flush()

    def read_telegram(self, timeout: float = 1.0, expect_cmd: int | None = None) -> Telegram | None:
        """Return the next CRC-valid reply telegram, resyncing on STX as needed."""
        deadline = time.monotonic() + timeout
        while True:
            tel = self._parse_one()
            if tel is not None:
                if expect_cmd is None or tel.cmd == expect_cmd:
                    return tel
                self.log(f"skipping reply 0x{tel.cmd:02X} while waiting for 0x{expect_cmd:02X}")
                continue
            if time.monotonic() > deadline:
                return None
            self._fill()

    def _parse_one(self) -> Telegram | None:
        buf = self._buf
        while True:
            i = buf.find(STX)
            if i < 0:
                buf.clear()
                return None
            if i:
                del buf[:i]
            if len(buf) < 4:
                return None
            addr = buf[1]
            length = buf[2] | (buf[3] << 8)
            # Replies always carry the 0x80 bit in ADDR; max telegram is 812 bytes.
            if not (addr & 0x80) or length < 2 or length > 812:
                del buf[0]
                continue
            total = 4 + length + 2
            if len(buf) < total:
                return None
            frame = bytes(buf[:total])
            got = frame[-2] | (frame[-1] << 8)
            if crc16(frame[:-2]) != got:
                del buf[0]  # false STX inside data; resync
                continue
            del buf[:total]
            return Telegram(addr=addr, cmd=frame[4], data=frame[5:-3], status=frame[-3])

    def request(self, payload: bytes, reply_cmd: int, timeout: float = 2.0, retries: int = 3) -> Telegram:
        """Send a command, wait for ACK then the matching reply telegram."""
        last = "no response"
        for attempt in range(retries):
            self.flush_input()
            self.send(payload)
            ack = self._wait_ack(0.3)
            if ack is None:
                last = "no ACK/NAK"
                self.log(f"attempt {attempt + 1}: {last}")
                continue
            if ack == NAK:
                last = "NAK (checksum rejected)"
                self.log(f"attempt {attempt + 1}: {last}")
                time.sleep(0.05)
                continue
            tel = self.read_telegram(timeout, expect_cmd=reply_cmd)
            if tel is not None:
                self.log(f"RX 0x{tel.cmd:02X} data={tel.data.hex(' ')} status=0x{tel.status:02X}")
                return tel
            last = f"ACK but no 0x{reply_cmd:02X} reply"
        raise LMSError(f"command 0x{payload[0]:02X} failed: {last}")

    def _wait_ack(self, timeout: float) -> int | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            while self._buf:
                b = self._buf[0]
                if b in (ACK, NAK):
                    del self._buf[0]
                    return b
                if b == STX:
                    return ACK  # some firmware skips ACK before streaming; treat reply as ack
                del self._buf[0]
            self._fill()
        return None

    # ---------- connection ----------

    def connect(self, baud: int = 38400) -> None:
        """Find the LMS at whatever baud it is currently using, then switch to `baud`."""
        found = None
        for b in (9600, 38400, 19200):
            self.ser.baudrate = b
            time.sleep(0.05)
            # The LMS may be mid-stream from a previous session; stopping it is harmless.
            try:
                self.request(bytes([0x20, MODE_ON_REQUEST]), 0xA0, timeout=3.5, retries=2)
                found = b
                break
            except LMSError as e:
                self.log(f"no answer at {b} baud: {e}")
        if found is None:
            raise LMSError(
                "LMS did not respond at 9600/19200/38400 baud. Check power (green LED), "
                "the null-modem wiring (LMS pin 2<->PC pin 3, 3<->2, 5<->5) and that pins 7/8 "
                "are NOT bridged in the LMS connector (that selects RS-422)."
            )
        self.log(f"LMS answered at {found} baud")
        if baud != found:
            self.set_baud(baud)

    def set_baud(self, baud: int) -> None:
        tel = self.request(bytes([0x20, BAUD_SUBCMD[baud]]), 0xA0, timeout=3.5)
        if tel.data[:1] != b"\x00":
            raise LMSError(f"baud change refused: {tel.data.hex()}")
        self.ser.baudrate = baud
        time.sleep(0.1)
        self.flush_input()
        self.log(f"now at {baud} baud")

    # ---------- queries ----------

    def get_type(self) -> str:
        tel = self.request(b"\x3A", 0xBA)
        return tel.data.decode("ascii", "replace").strip("\x00 ")

    def get_status(self) -> Telegram:
        return self.request(b"\x31", 0xB1, timeout=2.0)

    def get_config(self) -> dict:
        """Read the stored configuration (74h)."""
        tel = self.request(b"\x74", 0xF4)
        d = tel.data
        self.config = d
        blanking, thresh = struct.unpack_from("<HH", d, 0)
        mode, unit = d[5], d[6]
        self.unit_mm = unit == 0x01
        self.distance_bits = MEASURING_MODE_BITS.get(mode, 13)
        return {
            "blanking_cm": blanking,
            "threshold_word": f"0x{thresh:04X}",
            "availability": d[4],
            "measuring_mode": f"0x{mode:02X}",
            "unit": "mm" if unit == 1 else "cm" if unit == 0 else f"0x{unit:02X}",
            "distance_bits": self.distance_bits,
            "raw": d.hex(" "),
        }

    def enter_setup(self, password: bytes = b"SICK_LMS") -> None:
        """Command 20h 00h: installation mode, required before writing configuration."""
        tel = self.request(b"\x20\x00" + password, 0xA0, timeout=4.0)
        if tel.data[:1] != b"\x00":
            raise LMSError("scanner refused installation mode (wrong password?)")

    def set_units(self, mm: bool) -> bool:
        """Switch distance units between mm (1 mm steps, 8.2 m max) and cm (1 cm steps, 81.9 m max).

        Writes the scanner's EEPROM (command 77h), so the setting survives power-off. Only the
        unit byte is changed; the rest of the stored configuration is written back unchanged.
        Returns False if the scanner was already in the requested unit.
        """
        self.get_config()
        current = bytearray(self.config)
        if len(current) < 7:
            raise LMSError(f"unexpected configuration length {len(current)}")
        want = 0x01 if mm else 0x00
        if current[6] == want:
            return False
        new = bytes(current[:6]) + bytes([want]) + bytes(current[7:])

        self.enter_setup()
        try:
            # A unit change can take up to 7 s before the F7 confirmation arrives.
            tel = self.request(b"\x77" + new, 0xF7, timeout=10.0, retries=2)
            if tel.data[:1] != b"\x01":
                raise LMSError(f"configuration rejected; scanner kept: {tel.data[1:].hex(' ')}")
            if tel.data[1:1 + len(new)] != new:
                raise LMSError(f"scanner echoed a different configuration: {tel.data[1:].hex(' ')}")
        finally:
            self.request(bytes([0x20, MODE_ON_REQUEST]), 0xA0, timeout=4.0)

        self.get_config()
        if self.config[6] != want:
            raise LMSError("unit change did not persist")
        return True

    def set_variant(self, angular_range: int = 180, resolution: float = 0.5) -> None:
        """Command 3Bh: select scan angle (100/180) and resolution (1/0.5/0.25). Not stored in EEPROM."""
        res = {1.0: 100, 0.5: 50, 0.25: 25}[resolution]
        tel = self.request(b"\x3B" + struct.pack("<HH", angular_range, res), 0xBB, timeout=3.0)
        if tel.data[:1] != b"\x01":
            raise LMSError(f"variant {angular_range}°/{resolution}° refused: {tel.data.hex(' ')}")
        self.angular_range, self.resolution = angular_range, resolution

    # ---------- measurements ----------

    def _decode_scan(self, tel: Telegram) -> Scan:
        d = tel.data
        info = d[0] | (d[1] << 8)
        n = info & 0x3FF
        unit_mm = ((info >> 14) & 0x3) == 0x1
        partial = (info >> 11) & 0x3 if info & (1 << 13) else None
        words = struct.unpack_from(f"<{n}H", d, 2)

        bits = self.distance_bits
        mask = (1 << bits) - 1
        scale = 0.001 if unit_mm else 0.01
        overflow_floor = mask - 8  # 0x1FF7 and up are error codes for 13-bit (section 10.8)
        ranges, flags = [], []
        for w in words:
            v = w & mask
            flags.append(w >> bits)
            ranges.append(float("nan") if v >= overflow_floor else v * scale)

        if partial is not None:
            # Interlaced: 1° raster offset by partial*0.25°.
            start = partial * 0.25
            angles = [start + i for i in range(n)]
        else:
            span = self.angular_range
            step = span / (n - 1) if n > 1 else 0
            start = 0.0 if span == 180 else 40.0  # 100° field spans 40°..140°
            angles = [start + i * step for i in range(n)]
        return Scan(
            timestamp=time.time(), angles_deg=angles, ranges_m=ranges, raw=list(words),
            unit_mm=unit_mm, status=tel.status, partial_index=partial, flags=flags,
        )

    def poll_scan(self) -> Scan:
        """Request a single complete scan (30h 01h)."""
        tel = self.request(b"\x30\x01", 0xB0, timeout=1.5)
        return self._decode_scan(tel)

    def start_stream(self) -> None:
        self.request(bytes([0x20, MODE_CONTINUOUS_ALL]), 0xA0, timeout=3.5)

    def stop_stream(self) -> None:
        self.request(bytes([0x20, MODE_ON_REQUEST]), 0xA0, timeout=3.5)

    def stream(self, timeout: float = 2.0):
        """Yield scans from continuous output mode (call start_stream first)."""
        while True:
            tel = self.read_telegram(timeout, expect_cmd=0xB0)
            if tel is None:
                raise LMSError("stream stalled: no B0 telegram within timeout")
            yield self._decode_scan(tel)
