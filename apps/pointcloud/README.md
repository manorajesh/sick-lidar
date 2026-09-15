# 3D point clouds from the LMS200 and a phone

The LMS200 measures one flat slice of the world at a time. This app mounts a phone
on the scanner and uses the phone's orientation, sent over OSC by an app such as
[GyrOSC](https://apps.apple.com/us/app/gyrosc/id418751595), to place each slice
in 3D as you tilt or turn the scanner. The result is a point cloud (`.ply`) you
can open in CloudCompare, MeshLab, Blender or Open3D.

It's a separate app built on the driver (`lms200.py` in the repo root). It
doesn't change the driver or `lidar.py`.

## How it works

1. The phone sends its orientation (a quaternion) many times a second over Wi-Fi.
2. The scanner streams scan lines over serial.
3. Each scan is matched to the phone orientation at the moment it was measured.
   That's its arrival time minus the time the scan takes to cross the serial line,
   and the orientation is interpolated between the phone samples around it.
4. The scan's points are rotated from the scanner's frame into a level world frame
   and added to the cloud.

**Orientation only, no position.** A phone's motion sensors can't track position:
it drifts within seconds. So **rotate the scanner in place**, on a tripod or pan/tilt
head, rather than carrying it around. The phone's own offset from the scanner doesn't
matter, only that it's rigidly attached.

## Setup

```sh
# from the repo root
pip install -e ".[pointcloud,view]"      # view is only needed for --preview
```

**Mount the phone rigidly on the scanner.** The default assumes it lies flat on
top, screen up, with its top edge pointing where the scanner looks (the 90° beam,
straight out of the scan window). For other mountings, say which scanner
direction the phone's top edge and screen point to:

```sh
python pointcloud.py check --phone-top up --phone-screen back   # phone upright against the back
```

**Set up the phone app** (GyrOSC or any app that sends a quaternion over OSC):

- Target: the IP address that `monitor` prints, port 8000 (or your `--listen-port`).
- Turn on **Quaternion** (required) and **Gravity** (optional; enables the automatic check below).
- Choose the highest update rate available.
- Keep the app open with the screen on. iOS pauses apps in the background.
- The computer's firewall must allow incoming UDP on the port. macOS and Windows
  usually ask the first time.

## Workflow

Run these from this folder (`apps/pointcloud/`), or give the script's full path.

**1. See what the phone sends.** No scanner needed.

```sh
python pointcloud.py monitor
```

This lists every OSC address with its rate and latest values, and marks the ones
used for orientation and gravity. Any 4-value address containing `quat` is used
as orientation; if yours is named differently, pass `--quat-address /your/address`.

**2. Check orientation and mounting.** No scanner needed.

```sh
python pointcloud.py check
```

Tilt and turn the scanner and check the readout follows:

- **heading:** 0° where it started, + when turned right.
- **nose:** + when tilted up.
- **right side:** + when that side is raised.

If Gravity is on, tilt the scanner around and the app works out whether it's reading
the quaternion correctly. It shows *gravity check OK*, or tells you which
`--quat-order` / `--invert` setting to use.

**3. Capture.**

```sh
python pointcloud.py capture room.ply --preview
```

Hold the scanner still for a moment at the start; its facing direction becomes +y.
Then turn it **slowly**. At the default 1° resolution the scanner delivers about
9.4 lines a second, so 5–10°/s gives lines about 1° apart. Press Ctrl-C (or close
the preview) to finish, or use `--duration 60`.

This writes `room.ply` and the raw session `room.jsonl`.

Two ways to cover a space:

- **Nod:** scanner upright (slice horizontal); tilt it up and down. This covers
  the view in front of it, 180° wide.
- **Pan:** scanner on its side (slice vertical); turn it about the vertical axis.
  A full turn covers the whole room.

**4. Tune the timing (optional).** Wi-Fi and serial delays can put the phone data
slightly out of step with the scans, which shows up as doubled or smeared walls.
Rotate back and forth during the capture, then rebuild with a few offsets and keep
the sharpest:

```sh
python pointcloud.py build room.jsonl room_a.ply --scan-offset -0.05
python pointcloud.py build room.jsonl room_b.ply --scan-offset 0.05
```

`build` reuses the capture's settings unless you override them (`--phone-top`,
`--quat-order`, `--invert`/`--no-invert`, ...), so you can also fix mounting or
convention mistakes after the fact.

## Output

- **`.ply`**: binary point cloud. `x, y, z` in metres, with the origin at the
  scanner, z up and +y the scanner's starting direction. `scan` is each point's
  scan-line index, handy for colouring by time.
- **`.jsonl`**: the raw session, one JSON object per line: a `meta` header, every
  orientation/gravity message (`osc`), and every scan (`scan`). All are timestamped.

## Options

| Option | Commands | Meaning |
|---|---|---|
| `--listen-port 8000` | monitor, check, capture | UDP port the phone sends to |
| `--quat-address` | all | Orientation address, if auto-detection picks the wrong one |
| `--quat-order wxyz\|xyzw` | all | Quaternion argument order (GyrOSC documents w, x, y, z) |
| `--invert` | all | For apps that send the inverse rotation (`check` tells you) |
| `--phone-top`, `--phone-screen` | check, capture, build | How the phone is mounted |
| `--port`, `--fov`, `--resolution` | capture | Scanner settings, as in `lidar.py` |
| `--scan-offset` | capture, build | Shift scans in time relative to the phone (seconds) |
| `--duration`, `--preview` | capture | Stop after N seconds; live 3D view |

Logging works like `lidar.py`: `-v`, `-q` and `--log-file` go *before* the command,
for example `python pointcloud.py --log-file cap.log capture room.ply`.

## Limitations

- **No position tracking:** the scanner must rotate about (roughly) a fixed point.
- **Heading drift:** without a compass fix, the phone's heading drifts slowly, so
  keep captures to a few minutes.
- **Point density** follows the scan rate: about 9.4 lines/s at 1°, 4.7 lines/s at 0.5°.
- **Testing so far:** a simulated scanner and phone, end to end (wall flat to 4 mm).
  A real GyrOSC capture hasn't been tried yet. `monitor` and `check` are there to
  confirm the phone's message names and conventions before the first capture.

## Tests

```sh
python -m unittest discover apps/pointcloud/tests      # from the repo root
```

They cover the rotation maths, interpolation, mounting, the gravity-based convention
check, PLY output, and a simulated capture of a flat wall that must come back flat.
