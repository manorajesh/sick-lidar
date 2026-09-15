"""Geometry and file formats for turning LMS200 scan lines plus phone orientation
into a 3D point cloud. No hardware or network code, so it can be tested offline.

Frames (all right-handed):
- scanner: x = right, y = forward (the 90° beam), z = up (top of the housing).
  A reading at angle a and range r is (r cos a, r sin a, 0).
- phone: iOS device axes: x = right edge, y = top edge, z = out of the screen.
- world: the phone's reference frame (z vertical), turned about z so that the
  scanner's first forward direction points along +y.
"""

from __future__ import annotations

import bisect
import json
import math
import threading
from dataclasses import dataclass, field

import numpy as np

DIRECTIONS = {
    "right": (1.0, 0.0, 0.0), "left": (-1.0, 0.0, 0.0),
    "forward": (0.0, 1.0, 0.0), "back": (0.0, -1.0, 0.0),
    "up": (0.0, 0.0, 1.0), "down": (0.0, 0.0, -1.0),
}


def mount_matrix(top: str, screen: str) -> np.ndarray:
    """Matrix taking scanner-frame vectors to phone-frame vectors, given which way
    the phone's top edge and screen face, as directions of the scanner."""
    y = np.array(DIRECTIONS[top])
    z = np.array(DIRECTIONS[screen])
    if abs(y @ z) > 1e-9:
        raise ValueError(f"phone top ({top}) and screen ({screen}) must be perpendicular")
    x = np.cross(y, z)
    return np.vstack([x, y, z])  # rows: phone axes in scanner coordinates


def parse_quat(args, order: str = "wxyz") -> np.ndarray | None:
    """Unit quaternion (w, x, y, z) from 4 OSC arguments, or None if unusable."""
    if len(args) != 4:
        return None
    try:
        vals = [float(a) for a in args]
    except (TypeError, ValueError):
        return None
    if order == "xyzw":
        vals = [vals[3], vals[0], vals[1], vals[2]]
    q = np.array(vals)
    n = float(np.linalg.norm(q))
    if not math.isfinite(n) or n < 1e-6:
        return None
    return q / n


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Rotation matrix that rotates vectors by unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    d = float(q0 @ q1)
    if d < 0:  # take the short way round
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)


def world_from_phone(q: np.ndarray, invert: bool = False) -> np.ndarray:
    """Phone-to-world rotation. `invert` flips the quaternion's meaning, for apps that
    send the world-to-device rotation instead."""
    r = quat_to_matrix(q)
    return r.T if invert else r


class PoseBuffer:
    """Timestamped orientation samples, safe to fill from another thread."""

    def __init__(self, keep_s: float | None = 30.0):
        self.keep_s = keep_s
        self._t: list[float] = []
        self._q: list[np.ndarray] = []
        self._lock = threading.Lock()

    def add(self, t: float, q: np.ndarray) -> None:
        with self._lock:
            if self._t and t < self._t[-1]:
                i = bisect.bisect(self._t, t)
                self._t.insert(i, t)
                self._q.insert(i, q)
            else:
                self._t.append(t)
                self._q.append(q)
            if self.keep_s is not None:
                cut = bisect.bisect_left(self._t, self._t[-1] - self.keep_s)
                if cut:
                    del self._t[:cut], self._q[:cut]

    def at(self, t: float, max_gap: float = 0.25) -> np.ndarray | None:
        """Orientation at time t, interpolated between the samples around it.
        None if t is outside the samples or they are more than max_gap apart."""
        with self._lock:
            i = bisect.bisect_left(self._t, t)
            if i < len(self._t) and self._t[i] == t:
                return self._q[i]
            if i == 0 or i == len(self._t):
                return None
            t0, t1 = self._t[i - 1], self._t[i]
            if t1 - t0 > max_gap:
                return None
            return slerp(self._q[i - 1], self._q[i], (t - t0) / (t1 - t0))

    def latest(self) -> tuple[float, np.ndarray] | None:
        with self._lock:
            return (self._t[-1], self._q[-1]) if self._t else None

    def __len__(self) -> int:
        return len(self._t)


class Fuser:
    """Turns one scan line plus the phone orientation into world points."""

    def __init__(self, mount: np.ndarray, invert: bool = False):
        self.mount = mount
        self.invert = invert
        self.heading: np.ndarray | None = None  # set from the first pose

    def scanner_to_world(self, q: np.ndarray) -> np.ndarray:
        r = world_from_phone(q, self.invert) @ self.mount
        if self.heading is None:
            # Turn the world about vertical so the scanner starts out facing +y.
            f = r @ np.array([0.0, 1.0, 0.0])
            if math.hypot(f[0], f[1]) < 0.2:  # pointing almost straight up/down
                self.heading = np.eye(3)
            else:
                th = math.pi / 2 - math.atan2(f[1], f[0])
                c, s = math.cos(th), math.sin(th)
                self.heading = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return self.heading @ r

    def points(self, angles_deg, ranges_m, q: np.ndarray) -> np.ndarray:
        a = np.radians(np.asarray(angles_deg, dtype=float))
        r = np.asarray(ranges_m, dtype=float)
        ok = np.isfinite(r) & (r > 0)
        local = np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())])
        return local @ self.scanner_to_world(q).T


def telegram_seconds(n_values: int, baud: int) -> float:
    """Time to transmit one scan telegram (2 bytes per value + 10 framing bytes,
    10 bits per byte). A scan's measurement is this much older than its arrival."""
    return (2 * n_values + 10) * 10 / baud


def gravity_error_deg(q: np.ndarray, gravity_device, invert: bool) -> float:
    """Angle between measured gravity (phone frame) mapped to the world and straight down."""
    g = np.asarray(gravity_device, dtype=float)
    n = float(np.linalg.norm(g))
    if n < 1e-6:
        return float("nan")
    down = world_from_phone(q, invert) @ (g / n)
    return math.degrees(math.acos(max(-1.0, min(1.0, -down[2]))))


def diagnose_convention(raw_quat_args, gravity_device) -> dict[tuple[str, bool], float]:
    """Gravity error for each quaternion order / invert combination; the right one is near 0°."""
    out = {}
    for order in ("wxyz", "xyzw"):
        q = parse_quat(raw_quat_args, order)
        if q is None:
            continue
        for invert in (False, True):
            out[(order, invert)] = gravity_error_deg(q, gravity_device, invert)
    return out


def diagnose_window(pairs) -> dict[tuple[str, bool], float]:
    """90th-percentile gravity error per convention over (raw quaternion args, gravity)
    pairs. The right convention stays near 0° at every pose; a wrong one can fit some
    poses by coincidence (so a median is too forgiving) but not a spread of tilts."""
    errors: dict[tuple[str, bool], list[float]] = {}
    for raw, g in pairs:
        for combo, e in diagnose_convention(raw, g).items():
            if math.isfinite(e):
                errors.setdefault(combo, []).append(e)
    return {combo: float(np.percentile(v, 90)) for combo, v in errors.items() if v}


def gravity_spread_deg(pairs) -> float:
    """How far the phone has tilted across the pairs (largest angle from the first gravity)."""
    if not pairs:
        return 0.0
    g0 = np.asarray(pairs[0][1], dtype=float)
    g0 = g0 / (np.linalg.norm(g0) or 1.0)
    spread = 0.0
    for _, g in pairs:
        g = np.asarray(g, dtype=float)
        n = np.linalg.norm(g)
        if n > 1e-6:
            spread = max(spread, math.degrees(math.acos(max(-1.0, min(1.0, float(g0 @ g) / n)))))
    return spread


def is_quat_message(address: str, args, quat_address: str | None) -> bool:
    if quat_address:
        return address == quat_address
    return "quat" in address.lower() and len(args) == 4


def is_gravity_message(address: str, args) -> bool:
    return "grav" in address.lower() and len(args) == 3


# ---------- files ----------

class SessionWriter:
    """Raw capture log (JSON Lines): phone messages and scans with timestamps, so a
    point cloud can be rebuilt later with different settings."""

    def __init__(self, path: str, meta: dict):
        self._f = open(path, "w", encoding="utf-8")
        self._lock = threading.Lock()
        self._lines = 0
        self._write({"type": "meta", "version": 1, **meta})

    def _write(self, record: dict) -> None:
        line = json.dumps(record, separators=(",", ":"))
        with self._lock:
            self._f.write(line + "\n")
            self._lines += 1
            if self._lines % 200 == 0:
                self._f.flush()

    def osc(self, t: float, address: str, args) -> None:
        self._write({"type": "osc", "t": t, "addr": address, "args": list(args)})

    def scan(self, t: float, tx: float, angles_deg, ranges_m) -> None:
        step = angles_deg[1] - angles_deg[0] if len(angles_deg) > 1 else 0.0
        ranges = [None if not math.isfinite(r) else round(r, 4) for r in ranges_m]
        self._write({"type": "scan", "t": t, "tx": tx, "start": angles_deg[0], "step": step, "ranges": ranges})

    def close(self) -> None:
        with self._lock:
            self._f.close()


@dataclass
class Session:
    meta: dict
    osc: list = field(default_factory=list)  # (t, address, args)
    scans: list = field(default_factory=list)  # dicts as written by SessionWriter.scan


def read_session(path: str) -> Session:
    session = Session(meta={})
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            rec = json.loads(line)
            kind = rec.get("type")
            if kind == "meta":
                session.meta = rec
            elif kind == "osc":
                session.osc.append((rec["t"], rec["addr"], rec["args"]))
            elif kind == "scan":
                session.scans.append(rec)
    return session


@dataclass
class BuildStats:
    scans: int = 0
    used: int = 0
    no_pose: int = 0
    points: int = 0


def build_points(session: Session, mount: np.ndarray, quat_address: str | None = None,
                 order: str = "wxyz", invert: bool = False, offset_s: float = 0.0,
                 max_gap: float = 0.25) -> tuple[np.ndarray, np.ndarray, BuildStats]:
    """Rebuild a point cloud from a recorded session. offset_s moves scans later
    (+) or earlier (-) relative to the phone data."""
    poses = PoseBuffer(keep_s=None)
    for t, address, args in session.osc:
        if is_quat_message(address, args, quat_address):
            q = parse_quat(args, order)
            if q is not None:
                poses.add(t, q)
    fuser = Fuser(mount, invert)
    stats = BuildStats(scans=len(session.scans))
    chunks, ids = [], []
    for i, s in enumerate(session.scans):
        q = poses.at(s["t"] - s["tx"] + offset_s, max_gap)
        if q is None:
            stats.no_pose += 1
            continue
        n = len(s["ranges"])
        angles = [s["start"] + k * s["step"] for k in range(n)]
        ranges = [math.nan if r is None else r for r in s["ranges"]]
        pts = fuser.points(angles, ranges, q)
        chunks.append(pts)
        ids.append(np.full(len(pts), i, dtype=np.uint32))
        stats.used += 1
    points = np.vstack(chunks) if chunks else np.zeros((0, 3))
    scan_ids = np.concatenate(ids) if ids else np.zeros(0, dtype=np.uint32)
    stats.points = len(points)
    return points, scan_ids, stats


PLY_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("scan", "<u4")])


def write_ply(path: str, points: np.ndarray, scan_ids: np.ndarray) -> None:
    """Binary PLY point cloud (x, y, z in metres plus the scan index), readable by
    CloudCompare, MeshLab, Blender, Open3D and others."""
    data = np.empty(len(points), dtype=PLY_DTYPE)
    if len(points):
        data["x"], data["y"], data["z"] = points[:, 0], points[:, 1], points[:, 2]
        data["scan"] = scan_ids
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\nproperty uint scan\n"
        "end_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(data.tobytes())


def read_ply(path: str) -> np.ndarray:
    """Read a PLY written by write_ply (used by the tests)."""
    with open(path, "rb") as f:
        raw = f.read()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    return np.frombuffer(raw[end:], dtype=PLY_DTYPE)
