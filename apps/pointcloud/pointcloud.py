#!/usr/bin/env python3
"""Build a 3D point cloud from LMS200 scan lines and a phone's orientation sent
over OSC (e.g. GyrOSC on an iPhone mounted on the scanner).

    python pointcloud.py monitor                  # show what the phone is sending
    python pointcloud.py check                    # live check of orientation and mounting
    python pointcloud.py capture room.ply         # scan; writes room.ply and room.jsonl
    python pointcloud.py capture room.ply --preview --duration 60
    python pointcloud.py build room.jsonl room2.ply --scan-offset 0.03   # rebuild offline

Diagnostics go to stderr (and --log-file); results go to stdout.
"""

from __future__ import annotations

import argparse
import logging
import math
import queue
import socket
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path

try:
    import lms200
except ImportError:  # running from a checkout without `pip install -e .`
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    import lms200

try:
    import numpy as np
    from pythonosc.dispatcher import Dispatcher
    from pythonosc.osc_server import BlockingOSCUDPServer
except ImportError as e:
    sys.exit(f'error: {e.name} is not installed. From the repo root:  pip install -e ".[pointcloud]"')

from fusion import (
    BuildStats, Fuser, PoseBuffer, SessionWriter, build_points, diagnose_window, gravity_spread_deg,
    is_gravity_message, is_quat_message, mount_matrix, parse_quat, read_session,
    telegram_seconds, write_ply, DIRECTIONS,
)

log = logging.getLogger("pointcloud")

SETTLE_S = 0.3  # process scans this long after arrival, once the phone data around them is in


# ---------- phone ----------

class PhoneListener:
    """Receives the phone's OSC messages on a background thread."""

    def __init__(self, port: int, quat_address: str | None, order: str):
        self.quat_address = quat_address
        self.order = order
        self.poses = PoseBuffer()
        self.counts: Counter = Counter()
        self.last_args: dict = {}
        self.raw_quat = None  # (t, args) of the latest orientation message
        self.gravity_pairs: deque = deque(maxlen=300)  # (quat args, gravity args) sent together
        self.orientation_address = None
        self.session: SessionWriter | None = None
        dispatcher = Dispatcher()
        dispatcher.set_default_handler(self._handle)
        # A single-threaded server keeps messages in arrival order.
        self.server = BlockingOSCUDPServer(("0.0.0.0", port), dispatcher)
        self.thread = threading.Thread(target=self.server.serve_forever, name="osc", daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.server.shutdown()
        self.server.server_close()

    def _handle(self, address, *args):
        t = time.time()
        self.counts[address] += 1
        self.last_args[address] = args
        if is_quat_message(address, args, self.quat_address):
            if self.orientation_address is None:
                self.orientation_address = address
                log.info("orientation from %s", address)
            self.raw_quat = (t, args)
            q = parse_quat(args, self.order)
            if q is not None:
                self.poses.add(t, q)
            if self.session:
                self.session.osc(t, address, args)
        elif is_gravity_message(address, args):
            if self.raw_quat is not None and t - self.raw_quat[0] < 0.05:
                self.gravity_pairs.append((self.raw_quat[1], args))
            if self.session:
                self.session.osc(t, address, args)


def local_ips() -> list[str]:
    ips = set()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.0.2.1", 9))  # picks the outgoing interface; sends nothing
            ips.add(s.getsockname()[0])
    except OSError:
        pass
    try:
        ips.update(ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127."))
    except OSError:
        pass
    return sorted(ips)


def start_listener(args) -> PhoneListener:
    try:
        listener = PhoneListener(args.listen_port, args.quat_address, args.quat_order)
    except OSError as e:
        sys.exit(f"error: cannot listen on UDP port {args.listen_port}: {e}")
    ips = ", ".join(local_ips()) or "this computer's IP address"
    log.info("listening for OSC on UDP port %d. In the phone app, send to %s port %d",
             args.listen_port, ips, args.listen_port)
    listener.start()
    return listener


def wait_for_orientation(listener: PhoneListener, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    next_note = time.time() + 3
    while not len(listener.poses):
        if time.time() > deadline:
            return False
        if time.time() > next_note:
            seen = ", ".join(sorted(listener.counts)) or "nothing yet"
            log.info("waiting for orientation (quaternion) messages; received so far: %s", seen)
            next_note = time.time() + 3
        time.sleep(0.1)
    return True


def fmt_args(values, limit=6) -> str:
    shown = [f"{v:.3f}" if isinstance(v, float) else str(v) for v in values[:limit]]
    return " ".join(shown) + (" ..." if len(values) > limit else "")


def cmd_monitor(args):
    listener = start_listener(args)
    prev: Counter = Counter()
    try:
        while True:
            time.sleep(2.0)
            counts = Counter(listener.counts)
            if not counts:
                log.info("no OSC received yet on port %d", args.listen_port)
                continue
            print(f"\n{'address':28s} {'rate':>7s}  {'args':>4s}  last values")
            for address in sorted(counts):
                values = listener.last_args[address]
                rate = (counts[address] - prev[address]) / 2.0
                role = ""
                if is_quat_message(address, values, args.quat_address):
                    role = "  <- orientation"
                elif is_gravity_message(address, values):
                    role = "  <- gravity (for the convention check)"
                print(f"{address:28s} {rate:5.1f}/s  {len(values):4d}  {fmt_args(values)}{role}")
            sys.stdout.flush()
            prev = counts
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()


def cmd_check(args):
    listener = start_listener(args)
    fuser = Fuser(mount_matrix(args.phone_top, args.phone_screen), args.invert)
    try:
        if not wait_for_orientation(listener, args.wait_phone):
            log.error("no orientation messages. Run `monitor` to see what the phone is sending.")
            sys.exit(1)
        print("Tilt and turn the scanner and check the numbers follow. 0° heading = where it started.")
        print("Ctrl-C to stop.\n")
        prev_count, prev_t = 0, time.time()
        while True:
            time.sleep(0.25)
            _, q = listener.poses.latest()
            r = fuser.scanner_to_world(q)
            fwd, right = r @ [0.0, 1.0, 0.0], r @ [1.0, 0.0, 0.0]
            heading = math.degrees(math.atan2(fwd[0], fwd[1]))
            elevation = math.degrees(math.asin(max(-1.0, min(1.0, fwd[2]))))
            roll = math.degrees(math.asin(max(-1.0, min(1.0, right[2]))))
            count = listener.counts[listener.orientation_address]
            now = time.time()
            rate = (count - prev_count) / (now - prev_t)
            prev_count, prev_t = count, now
            print(f"\rheading {heading:+6.1f}° (+ = turned right)  nose {elevation:+6.1f}° (+ = up)  "
                  f"right side {roll:+6.1f}° (+ = up)  {rate:4.0f} Hz  {gravity_verdict(listener, args)}   ",
                  end="", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        listener.stop()


def gravity_verdict(listener: PhoneListener, args) -> str:
    pairs = list(listener.gravity_pairs)
    if not pairs:
        return "(enable Gravity in the phone app for an automatic check)"
    if len(pairs) < 20 or gravity_spread_deg(pairs) < 15:
        return "gravity check: tilt the scanner around to run it"
    errors = diagnose_window(pairs)
    current = errors.get((args.quat_order, args.invert), float("nan"))
    others = [e for combo, e in errors.items() if combo != (args.quat_order, args.invert)]
    if current < 8 and all(e > 15 for e in others):
        return f"gravity check OK ({current:.0f}°)"
    if current < 8:
        return "gravity check unclear: tilt the scanner further"
    order, invert = min(errors, key=errors.get)
    if errors[(order, invert)] < 8:
        fix = f"--quat-order {order}" + (" --invert" if invert else " without --invert")
        return f"gravity check FAILED ({current:.0f}°): use {fix}"
    return f"gravity check FAILED ({current:.0f}°): no setting fits; is the quaternion from this phone?"


# ---------- scanner ----------

def scanner_worker(args, out: queue.Queue, stop: threading.Event) -> None:
    """Streams scans into `out` until `stop`, reconnecting if the scanner drops out."""
    failures = 0
    while not stop.is_set():
        lms = None
        try:
            lms = lms200.LMS200(args.port)
            lms.connect(baud=args.baud)
            lms.get_config()
            lms.set_variant(args.fov, args.resolution)
            lms.start_stream()
            baud = lms.ser.baudrate
            log.info("scanner streaming %d° @ %s°", args.fov, args.resolution)
            failures = 0
            for scan in lms.stream():
                out.put((scan, telegram_seconds(len(scan.ranges_m), baud)))
                if stop.is_set():
                    break
        except (lms200.LMSError, OSError) as e:
            if not stop.is_set():
                failures += 1
                if failures == 1 or failures % 30 == 0:  # once per outage, then about once a minute
                    log.warning("scanner unavailable (%s); retrying every 2 s", e)
                stop.wait(2.0)
        finally:
            if lms is not None:
                try:
                    lms.stop_stream()
                except Exception:
                    pass
                lms.close()


# ---------- capture ----------

class Preview:
    """Live 3D view of the growing cloud (matplotlib), thinned to stay responsive."""

    def __init__(self, max_points: int = 40000):
        import matplotlib.pyplot as plt

        self.plt = plt
        self.max_points = max_points
        plt.ion()
        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(projection="3d")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m, start direction)")
        self.ax.set_zlabel("z (m, up)")
        self.scatter = None
        self.next_draw = 0.0

    @property
    def closed(self) -> bool:
        return not self.plt.fignum_exists(self.fig.number)

    def update(self, chunks: list) -> None:
        if time.time() >= self.next_draw and chunks:
            self.next_draw = time.time() + 0.5
            pts = np.vstack(chunks)
            if len(pts) > self.max_points:
                pts = pts[:: math.ceil(len(pts) / self.max_points)]
            if self.scatter is not None:
                self.scatter.remove()
            self.scatter = self.ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=pts[:, 2], s=1,
                                           cmap="viridis", depthshade=False)
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            mid, half = (lo + hi) / 2, max(float((hi - lo).max()) / 2, 0.5)
            self.ax.set_xlim(mid[0] - half, mid[0] + half)
            self.ax.set_ylim(mid[1] - half, mid[1] + half)
            self.ax.set_zlim(mid[2] - half, mid[2] + half)
            self.ax.set_box_aspect((1, 1, 1))
            self.ax.set_title(f"{sum(len(c) for c in chunks):,} points")
            self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def cmd_capture(args):
    out = Path(args.output)
    session_path = Path(args.session) if args.session else out.with_suffix(".jsonl")
    mount = mount_matrix(args.phone_top, args.phone_screen)
    listener = start_listener(args)
    if not wait_for_orientation(listener, args.wait_phone):
        listener.stop()
        log.error("no orientation messages from the phone. Run `monitor` to see what it is sending.")
        sys.exit(1)

    session = SessionWriter(str(session_path), {
        "fov": args.fov, "resolution": args.resolution, "phone_top": args.phone_top,
        "phone_screen": args.phone_screen, "quat_order": args.quat_order, "invert": args.invert,
        "quat_address": args.quat_address, "scan_offset": args.scan_offset,
    })
    listener.session = session
    stop = threading.Event()
    scans: queue.Queue = queue.Queue()
    worker = threading.Thread(target=scanner_worker, args=(args, scans, stop), name="scanner", daemon=True)
    worker.start()
    log.info("capturing; the scanner's starting direction becomes +y. Turn it slowly. Ctrl-C to finish.")

    fuser = Fuser(mount, args.invert)
    stats = BuildStats()
    chunks: list = []
    ids: list = []
    pending: deque = deque()
    preview = Preview() if args.preview else None
    t_start = last_status = last_gap_warning = time.time()

    def process(scan, tx):
        nonlocal last_gap_warning
        session.scan(scan.timestamp, tx, scan.angles_deg, scan.ranges_m)
        index = stats.scans
        stats.scans += 1
        q = listener.poses.at(scan.timestamp - tx + args.scan_offset, args.max_gap)
        if q is None:
            stats.no_pose += 1
            if time.time() - last_gap_warning > 5:
                log.warning("scan skipped: no phone orientation at that moment (phone paused or Wi-Fi gap?)")
                last_gap_warning = time.time()
            return
        pts = fuser.points(scan.angles_deg, scan.ranges_m, q)
        chunks.append(pts)
        ids.append(np.full(len(pts), index, dtype=np.uint32))
        stats.used += 1
        stats.points += len(pts)

    try:
        while True:
            try:
                pending.append(scans.get(timeout=0.1))
            except queue.Empty:
                pass
            now = time.time()
            while pending and pending[0][0].timestamp < now - SETTLE_S:
                process(*pending.popleft())
            if preview is not None:
                preview.update(chunks)
                if preview.closed:
                    break
            if now - last_status >= 10:
                log.info("%d scans, %s points, %d skipped", stats.scans, f"{stats.points:,}", stats.no_pose)
                last_status = now
            if args.duration and now - t_start >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        log.info("finishing")
        stop.set()
        worker.join(timeout=10)
        time.sleep(SETTLE_S)  # let the last phone samples arrive
        while pending:
            process(*pending.popleft())
        listener.stop()
        session.close()

    points = np.vstack(chunks) if chunks else np.zeros((0, 3))
    write_ply(str(out), points, np.concatenate(ids) if ids else np.zeros(0, dtype=np.uint32))
    report(out, points, stats, session_path)


def report(out: Path, points, stats: BuildStats, session_path: Path | None = None) -> None:
    print(f"wrote {stats.points:,} points from {stats.used} of {stats.scans} scans to {out}")
    if session_path:
        print(f"raw session saved to {session_path} (rebuild with: build {session_path} <out.ply>)")
    if stats.no_pose:
        print(f"{stats.no_pose} scans skipped for lack of phone orientation")
    if len(points):
        lo, hi = points.min(axis=0), points.max(axis=0)
        print("extent (m): " + "  ".join(f"{a} {lo[i]:+.2f}..{hi[i]:+.2f}" for i, a in enumerate("xyz")))


# ---------- build ----------

def cmd_build(args):
    session = read_session(args.session)
    meta = session.meta

    def pick(name, default):
        value = getattr(args, name)
        return value if value is not None else meta.get(name, default)

    mount = mount_matrix(pick("phone_top", "forward"), pick("phone_screen", "up"))
    points, ids, stats = build_points(
        session, mount, quat_address=pick("quat_address", None), order=pick("quat_order", "wxyz"),
        invert=pick("invert", False), offset_s=pick("scan_offset", 0.0), max_gap=args.max_gap,
    )
    if not stats.scans:
        log.error("no scans in %s", args.session)
        sys.exit(1)
    write_ply(args.output, points, ids)
    report(Path(args.output), points, stats)


# ---------- CLI ----------

def setup_logging(verbose: bool, quiet: bool, log_file: str | None) -> None:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    level = logging.DEBUG if verbose else logging.WARNING if quiet else logging.INFO
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    handlers[0].setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(message)s", "%H:%M:%S"))
    if log_file:
        handler = logging.FileHandler(log_file, encoding="utf-8")
        handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s"))
        handlers.append(handler)
    logging.basicConfig(level=level, handlers=handlers, force=True)


def main():
    directions = sorted(DIRECTIONS)
    phone = argparse.ArgumentParser(add_help=False)
    phone.add_argument("--listen-port", type=int, default=8000, help="UDP port the phone sends to (default 8000)")
    phone.add_argument("--quat-address", help="orientation address (default: any 4-value address containing 'quat')")
    phone.add_argument("--quat-order", choices=["wxyz", "xyzw"], default="wxyz",
                       help="quaternion argument order (GyrOSC documents w, x, y, z)")
    phone.add_argument("--invert", action="store_true", help="use if tilts come out mirrored (see `check`)")
    phone.add_argument("--wait-phone", type=float, default=15.0, help="seconds to wait for the phone (default 15)")

    mount = argparse.ArgumentParser(add_help=False)
    mount.add_argument("--phone-top", choices=directions, default="forward",
                       help="scanner direction the phone's top edge points to (default forward)")
    mount.add_argument("--phone-screen", choices=directions, default="up",
                       help="scanner direction the phone's screen faces (default up)")

    scanner = argparse.ArgumentParser(add_help=False)
    scanner.add_argument("--port", help="scanner serial port, e.g. COM3 or /dev/ttyUSB0 (default: auto-detect)")
    scanner.add_argument("--baud", type=int, default=38400, choices=[9600, 19200, 38400])
    scanner.add_argument("--fov", type=int, default=180, choices=[180, 100])
    scanner.add_argument("--resolution", type=float, default=1.0, choices=[1.0, 0.5, 0.25],
                         help="angular step (default 1.0: ~9.4 scans/s; 0.5: ~4.7 scans/s)")

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    p.add_argument("-q", "--quiet", action="store_true", help="only warnings and errors")
    p.add_argument("--log-file", help="also append timestamped logs to this file")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("monitor", parents=[phone], help="show incoming OSC messages")
    sub.add_parser("check", parents=[phone, mount], help="live orientation and mounting check")

    c = sub.add_parser("capture", parents=[phone, mount, scanner], help="scan and build a point cloud")
    c.add_argument("output", help="point cloud to write (.ply)")
    c.add_argument("--session", help="raw session log (default: next to the output, .jsonl)")
    c.add_argument("--duration", type=float, help="stop after this many seconds (default: Ctrl-C)")
    c.add_argument("--preview", action="store_true", help="live 3D view (needs matplotlib)")
    c.add_argument("--scan-offset", type=float, default=0.0,
                   help="shift scans in time relative to the phone, seconds (tune with `build`)")
    c.add_argument("--max-gap", type=float, default=0.25, help="max gap between phone samples, seconds")

    b = sub.add_parser("build", help="rebuild a point cloud from a saved session")
    b.add_argument("session", help="session .jsonl from capture")
    b.add_argument("output", help="point cloud to write (.ply)")
    b.add_argument("--phone-top", choices=directions)
    b.add_argument("--phone-screen", choices=directions)
    b.add_argument("--quat-address")
    b.add_argument("--quat-order", choices=["wxyz", "xyzw"])
    b.add_argument("--invert", action=argparse.BooleanOptionalAction, default=None)
    b.add_argument("--scan-offset", type=float, help="seconds; default: the value used when capturing")
    b.add_argument("--max-gap", type=float, default=0.25)

    args = p.parse_args()
    if getattr(args, "resolution", None) == 0.25 and args.fov != 100:
        p.error("0.25° resolution is only available with --fov 100")
    try:
        setup_logging(args.verbose, args.quiet, args.log_file)
    except OSError as e:
        sys.exit(f"error: cannot open log file: {e}")
    try:
        {"monitor": cmd_monitor, "check": cmd_check, "capture": cmd_capture, "build": cmd_build}[args.cmd](args)
    except (ValueError, OSError) as e:
        log.debug("details", exc_info=True)
        log.error("%s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
