#!/usr/bin/env python3
"""Command-line tool for the SICK LMS200.

    python lidar.py info                 # identify scanner, print status/config
    python lidar.py scan                 # one scan, printed as angle/range table
    python lidar.py record out.csv -n 50 # stream N scans to CSV (0 = until Ctrl-C)
    python lidar.py view                 # live top-down map (metres)
    python lidar.py --fov 100 --resolution 0.25 view   # finer angle, narrower field
    python lidar.py osc --osc-port 9000  # stream scans as OSC over UDP
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time

import serial

from lms200 import LMS200, LMSError


def open_scanner(args) -> LMS200:
    lms = LMS200(args.port, verbose=args.verbose)
    try:
        lms.connect(baud=args.baud)
        lms.get_config()
        lms.set_variant(args.fov, args.resolution)
    except BaseException:
        lms.close()
        raise
    return lms


def cmd_info(args):
    lms = LMS200(args.port, verbose=args.verbose)
    lms.connect(baud=args.baud)
    print("Connected at", lms.ser.baudrate, "baud")
    try:
        print("Type:", lms.get_type())
    except LMSError as e:
        print("Type: (unavailable)", e)
    st = lms.get_status()
    print("Status telegram:", st.data[:7].decode("ascii", "replace"), f"(status byte 0x{st.status:02X})")
    for k, v in lms.get_config().items():
        print(f"  {k}: {v}")
    lms.close()


def cmd_units(args):
    lms = LMS200(args.port, verbose=args.verbose)
    lms.connect(baud=args.baud)
    before = lms.get_config()
    print(f"current: {before['unit']}  (config {before['raw']})")
    print(f"writing {args.unit} to the scanner's permanent memory (can take ~7 s)...")
    changed = lms.set_units(mm=args.unit == "mm")
    after = lms.get_config()
    print(("changed" if changed else "already set") + f": now {after['unit']}  (config {after['raw']})")
    lms.close()


def cmd_scan(args):
    lms = open_scanner(args)
    s = lms.poll_scan()
    lms.close()
    valid = [r for r in s.ranges_m if not math.isnan(r)]
    print(f"{len(s.ranges_m)} readings, {len(valid)} valid, unit={'mm' if s.unit_mm else 'cm'}, "
          f"status=0x{s.status:02X}")
    if valid:
        print(f"min {min(valid):.3f} m  max {max(valid):.3f} m")
    for a, r, w in zip(s.angles_deg, s.ranges_m, s.raw):
        print(f"{a:6.2f}°  {'   ---  ' if math.isnan(r) else f'{r:7.3f} m'}  raw=0x{w:04X}")


def cmd_record(args):
    lms = open_scanner(args)
    lms.start_stream()
    count = 0
    t0 = time.time()
    try:
        with open(args.outfile, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["scan", "timestamp", "angle_deg", "range_m", "raw"])
            for s in lms.stream():
                for a, r, raw in zip(s.angles_deg, s.ranges_m, s.raw):
                    w.writerow([count, f"{s.timestamp:.4f}", f"{a:.2f}", "" if math.isnan(r) else f"{r:.3f}", raw])
                count += 1
                print(f"\rscans: {count}  rate: {count / (time.time() - t0):.1f} Hz", end="", flush=True)
                if args.n and count >= args.n:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        lms.stop_stream()
        lms.close()
    print(f"wrote {count} scans to {args.outfile}")


def cmd_view(args):
    import matplotlib.pyplot as plt

    lms = open_scanner(args)
    lms.start_stream()
    plt.ion()
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)  ← left | right →")
    ax.set_ylabel("y (m)  straight ahead")
    ax.plot(0, 0, "r^", markersize=10)
    (outline,) = ax.plot([], [], "-", linewidth=0.8, color="0.6")  # connects neighbouring points
    (dots,) = ax.plot([], [], ".", markersize=4)
    title = ax.set_title("")
    extent = args.rmax or 1.0
    t_prev = time.time()
    hz = 0.0
    try:
        for s in lms.stream():
            xs, ys, lx, ly = [], [], [], []
            prev = None
            for a, r in zip(s.angles_deg, s.ranges_m):
                if math.isnan(r):
                    prev = None
                    lx.append(math.nan)
                    ly.append(math.nan)
                    continue
                x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
                # Break the outline at depth jumps so separate objects aren't joined.
                if prev is not None and abs(r - prev) > max(0.1, 0.05 * r):
                    lx.append(math.nan)
                    ly.append(math.nan)
                xs.append(x)
                ys.append(y)
                lx.append(x)
                ly.append(y)
                prev = r
            dots.set_data(xs, ys)
            outline.set_data(lx, ly)

            if args.rmax is None and xs:
                # Fit the view to the scene: grow at once, shrink slowly to avoid jitter.
                far = sorted(math.hypot(x, y) for x, y in zip(xs, ys))[int(0.98 * (len(xs) - 1))]
                target = max(0.5, far * 1.1)
                extent = target if target > extent else 0.9 * extent + 0.1 * target
            ax.set_xlim(-extent, extent)
            ax.set_ylim(-0.05 * extent, extent)

            now = time.time()
            hz = 0.8 * hz + 0.2 / max(now - t_prev, 1e-3)
            t_prev = now
            title.set_text(
                f"LMS200  {lms.angular_range}° @ {lms.resolution}°  "
                f"{len(xs)}/{len(s.ranges_m)} valid  {hz:.1f} Hz  status 0x{s.status:02X}"
            )
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            if not plt.fignum_exists(fig.number):
                break
    except KeyboardInterrupt:
        pass
    finally:
        lms.stop_stream()
        lms.close()


def osc_messages(scan, prefix: str = "/lms200", hz: float = 0.0) -> list[tuple[str, list]]:
    """OSC (address, arguments) pairs for one scan.

    Arrays have a fixed length, so index i is always the same angle. Readings with
    no valid return are sent as range 0 with x = y = 0.
    """
    ranges, xs, ys = [], [], []
    nearest = None
    for a, r in zip(scan.angles_deg, scan.ranges_m):
        if math.isnan(r):
            ranges.append(0.0)
            xs.append(0.0)
            ys.append(0.0)
            continue
        x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        ranges.append(r)
        xs.append(x)
        ys.append(y)
        if nearest is None or r < nearest[1]:
            nearest = [a, r, x, y]
    angles = scan.angles_deg
    step = angles[1] - angles[0] if len(angles) > 1 else 0.0
    return [
        (f"{prefix}/info", [len(ranges), float(angles[0]), float(step), float(hz)]),
        (f"{prefix}/ranges", ranges),
        (f"{prefix}/x", xs),
        (f"{prefix}/y", ys),
        (f"{prefix}/nearest", nearest or [0.0, 0.0, 0.0, 0.0]),
    ]


def run_osc(args, send, open_fn=open_scanner, sleep=time.sleep, retry_s: float = 2.0):
    """Stream scans as OSC via send(address, args), reconnecting whenever the scanner
    or USB adapter drops out. Runs until Ctrl-C."""
    prefix = "/" + args.prefix.strip("/")
    send_failing = False

    def emit(address, values):
        # UDP is fire-and-forget: a network hiccup must not look like a scanner failure.
        nonlocal send_failing
        try:
            send(address, values)
            send_failing = False
        except OSError as e:
            if not send_failing:
                print(f"\nOSC send failed ({e}); continuing", flush=True)
            send_failing = True

    lms = None
    try:
        while True:
            try:
                lms = open_fn(args)
                lms.start_stream()
                emit(f"{prefix}/connected", 1)
                print(f"scanner connected: {lms.angular_range}° @ {lms.resolution}°", flush=True)
                t_prev, hz = None, 0.0
                for s in lms.stream():
                    now = time.time()
                    if t_prev is not None:
                        rate = 1.0 / max(now - t_prev, 1e-3)
                        hz = rate if hz == 0.0 else 0.8 * hz + 0.2 * rate
                    t_prev = now
                    msgs = osc_messages(s, prefix, hz)
                    for address, values in msgs:
                        emit(address, values)
                    a, r = msgs[-1][1][:2]
                    print(f"\r{hz:5.1f} Hz   nearest {r:6.3f} m at {a:6.2f}°", end="", flush=True)
            except (LMSError, serial.SerialException, OSError) as e:
                if lms is not None:
                    try:
                        lms.close()
                    except Exception:
                        pass
                    lms = None
                emit(f"{prefix}/connected", 0)
                print(f"\nscanner unavailable ({e}); retrying in {retry_s:g} s", flush=True)
                sleep(retry_s)
    except KeyboardInterrupt:
        print()
    finally:
        if lms is not None:
            try:
                lms.stop_stream()
            except Exception:
                pass
            lms.close()
        emit(f"{prefix}/connected", 0)


def cmd_osc(args):
    try:
        from pythonosc.udp_client import SimpleUDPClient
    except ImportError:
        sys.exit("error: OSC output needs python-osc:  pip install -e '.[osc]'")
    client = SimpleUDPClient(args.host, args.osc_port)
    print(f"sending OSC to {args.host}:{args.osc_port} (Ctrl-C to stop)")
    run_osc(args, client.send_message)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="serial device (default: auto-detect the Keyspan)")
    p.add_argument("--baud", type=int, default=38400, choices=[9600, 19200, 38400])
    p.add_argument("--fov", type=int, default=180, choices=[180, 100], help="scan angle in degrees")
    p.add_argument("--resolution", type=float, default=None, choices=[1.0, 0.5, 0.25],
                   help="angular step in degrees (default 0.5; 1.0 for osc; 0.25 needs --fov 100)")
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("scan")
    u = sub.add_parser("units", help="permanently set distance units (mm: 1 mm steps, 8.2 m max; cm: 81.9 m max)")
    u.add_argument("unit", choices=["mm", "cm"])
    r = sub.add_parser("record")
    r.add_argument("outfile")
    r.add_argument("-n", type=int, default=0, help="number of scans (0 = until Ctrl-C)")
    v = sub.add_parser("view")
    v.add_argument("--rmax", type=float, default=None, help="fixed view radius in metres (default: fit to scene)")
    o = sub.add_parser("osc", help="stream scans as OSC over UDP, reconnecting automatically")
    o.add_argument("--host", default="127.0.0.1", help="destination IP (default: this machine)")
    o.add_argument("--osc-port", type=int, default=9000, help="destination UDP port (default 9000)")
    o.add_argument("--prefix", default="/lms200", help="OSC address prefix (default /lms200)")
    args = p.parse_args()
    if args.resolution is None:
        # 1° doubles the scan rate at 38400 baud and keeps each OSC message in one UDP packet.
        args.resolution = 1.0 if args.cmd == "osc" else 0.5
    if args.resolution == 0.25 and args.fov != 100:
        p.error("0.25° resolution is only available with --fov 100")
    try:
        {"info": cmd_info, "scan": cmd_scan, "units": cmd_units, "record": cmd_record,
         "view": cmd_view, "osc": cmd_osc}[args.cmd](args)
    except (LMSError, serial.SerialException) as e:
        sys.exit(f"error: {e}")


if __name__ == "__main__":
    main()
