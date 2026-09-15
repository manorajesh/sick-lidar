"""Offline tests for the point-cloud maths: python -m unittest discover apps/pointcloud/tests"""

import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fusion import (  # noqa: E402
    Fuser, PoseBuffer, SessionWriter, build_points, diagnose_convention, diagnose_window,
    gravity_spread_deg, mount_matrix,
    parse_quat, quat_to_matrix, read_ply, read_session, slerp, telegram_seconds, write_ply,
)


def axis_angle_quat(axis, deg):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return np.array([math.cos(h), *(math.sin(h) * axis)])


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


class TestRotations(unittest.TestCase):
    def test_quat_to_matrix(self):
        r = quat_to_matrix(axis_angle_quat([0, 0, 1], 90))
        np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)

    def test_parse_quat_orders(self):
        np.testing.assert_allclose(parse_quat([2, 0, 0, 0]), IDENTITY)
        np.testing.assert_allclose(parse_quat([0, 0, 0, 1], "xyzw"), IDENTITY)
        self.assertIsNone(parse_quat([0, 0, 0, 0]))
        self.assertIsNone(parse_quat([1, 2, 3]))

    def test_slerp_midpoint(self):
        mid = slerp(IDENTITY, axis_angle_quat([0, 0, 1], 90), 0.5)
        np.testing.assert_allclose(mid, axis_angle_quat([0, 0, 1], 45), atol=1e-12)

    def test_mount(self):
        np.testing.assert_allclose(mount_matrix("forward", "up"), np.eye(3))
        # Phone standing on its edge facing backwards, top edge up:
        m = mount_matrix("up", "back")
        np.testing.assert_allclose(m @ [0, 0, 1], [0, 1, 0])  # scanner up -> phone top
        np.testing.assert_allclose(m @ [0, -1, 0], [0, 0, 1])  # scanner back -> screen
        with self.assertRaises(ValueError):
            mount_matrix("up", "down")


class TestPoseBuffer(unittest.TestCase):
    def test_interpolation_and_limits(self):
        buf = PoseBuffer()
        buf.add(1.0, IDENTITY)
        buf.add(1.1, axis_angle_quat([0, 0, 1], 10))
        np.testing.assert_allclose(buf.at(1.05), axis_angle_quat([0, 0, 1], 5), atol=1e-9)
        self.assertIsNone(buf.at(0.9))  # before the first sample
        self.assertIsNone(buf.at(1.2))  # after the last
        buf.add(2.0, IDENTITY)
        self.assertIsNone(buf.at(1.5, max_gap=0.25))  # gap too large

    def test_out_of_order_samples(self):
        buf = PoseBuffer()
        buf.add(2.0, IDENTITY)
        buf.add(1.0, IDENTITY)
        self.assertIsNotNone(buf.at(1.5, max_gap=2))


class TestFuser(unittest.TestCase):
    def test_level_scanner(self):
        f = Fuser(mount_matrix("forward", "up"))
        pts = f.points([0, 90, 180], [1.0, 2.0, float("nan")], IDENTITY)
        np.testing.assert_allclose(pts, [[1, 0, 0], [0, 2, 0]], atol=1e-12)

    def test_nose_up_tilt(self):
        f = Fuser(mount_matrix("forward", "up"))
        f.heading = np.eye(3)
        pts = f.points([90], [2.0], axis_angle_quat([1, 0, 0], 30))  # pitch about the right axis
        np.testing.assert_allclose(pts[0], [0, 2 * math.cos(math.radians(30)), 1.0], atol=1e-12)

    def test_heading_is_zeroed_on_first_pose(self):
        f = Fuser(mount_matrix("forward", "up"))
        yawed = axis_angle_quat([0, 0, 1], 40)
        np.testing.assert_allclose(f.points([90], [1.0], yawed)[0], [0, 1, 0], atol=1e-12)
        # Later rotations are relative to that start: 90° more to the left.
        later = quat_mul(axis_angle_quat([0, 0, 1], 90), yawed)
        np.testing.assert_allclose(f.points([90], [1.0], later)[0], [-1, 0, 0], atol=1e-12)


class TestConvention(unittest.TestCase):
    def test_gravity_picks_the_right_convention(self):
        q = quat_mul(axis_angle_quat([0, 0, 1], 70), axis_angle_quat([1, 1, 0], 35))
        gravity_device = quat_to_matrix(q).T @ [0, 0, -1]  # what the phone would measure
        errors = diagnose_convention(list(q), gravity_device)
        self.assertEqual(min(errors, key=errors.get), ("wxyz", False))
        self.assertLess(errors[("wxyz", False)], 1e-6)
        xyzw = [q[1], q[2], q[3], q[0]]
        errors = diagnose_convention(xyzw, gravity_device)
        self.assertEqual(min(errors, key=errors.get), ("xyzw", False))

    def test_window_rejects_coincidental_matches(self):
        # From one pose a wrong convention can fit gravity by chance; over tilting it can't,
        # even when the scanner is only tilted one way (half a nod).
        for steps in (60, 15):
            with self.subTest(steps=steps):
                pairs = []
                for k in range(steps):
                    q = quat_mul(axis_angle_quat([0, 0, 1], 25),
                                 axis_angle_quat([1, 0, 0], 30 * math.sin(2 * math.pi * k / 60)))
                    pairs.append(([q[1], q[2], q[3], q[0]], quat_to_matrix(q).T @ [0, 0, -1]))
                errors = diagnose_window(pairs)
                self.assertLess(errors[("xyzw", False)], 1e-6)
                self.assertTrue(all(e > 15 for combo, e in errors.items() if combo != ("xyzw", False)))
                self.assertGreater(gravity_spread_deg(pairs), 15)


class TestEndToEnd(unittest.TestCase):
    """Simulate nodding the scanner in front of a flat wall 3 m ahead and check that
    every rebuilt point lands on the wall."""

    WALL_Y = 3.0

    def simulate(self, path, heading_deg=25.0):
        start = axis_angle_quat([0, 0, 1], heading_deg)
        pitch = lambda t: -30 + 12 * t  # noqa: E731  (-30° to +30° over 5 s)
        pose = lambda t: quat_mul(start, axis_angle_quat([1, 0, 0], pitch(t)))  # noqa: E731
        writer = SessionWriter(path, {"fov": 180, "resolution": 1.0})
        for k in range(501):  # phone at 100 Hz
            t = 100.0 + k * 0.01
            writer.osc(t, "/gyrosc/quat", list(pose(t - 100.0)))
            writer.osc(t, "/gyrosc/gyro", [0.0, 0.0, 0.0])  # ignored
        angles = [float(a) for a in range(181)]
        tx = telegram_seconds(181, 38400)
        for k in range(1, 49):  # scans at ~10 Hz, arriving tx after measurement
            t_meas = 100.0 + k * 0.1
            # Ground truth in the start-facing world frame (heading removed).
            r_ws = quat_to_matrix(axis_angle_quat([1, 0, 0], pitch(t_meas - 100.0)))
            ranges = []
            for a in angles:
                d = r_ws @ [math.cos(math.radians(a)), math.sin(math.radians(a)), 0.0]
                ranges.append(self.WALL_Y / d[1] if d[1] > 0.2 else float("nan"))
            writer.scan(t_meas + tx, tx, angles, ranges)
        writer.close()

    def test_wall_is_flat(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "session.jsonl")
            self.simulate(path)
            session = read_session(path)
            points, ids, stats = build_points(session, mount_matrix("forward", "up"))
            self.assertEqual((stats.scans, stats.used, stats.no_pose), (48, 48, 0))
            self.assertGreater(len(points), 3000)
            # Ranges are stored to 0.1 mm, so allow a little rounding.
            np.testing.assert_allclose(points[:, 1], self.WALL_Y, atol=2e-3)
            self.assertGreater(np.ptp(points[:, 2]), 3.0)  # the nod spread the lines vertically

            ply = str(Path(tmp) / "cloud.ply")
            write_ply(ply, points, ids)
            back = read_ply(ply)
            self.assertEqual(len(back), len(points))
            np.testing.assert_allclose(back["y"], points[:, 1], atol=1e-5)

    def test_timing_offset_matters(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = str(Path(tmp) / "session.jsonl")
            self.simulate(path)
            session = read_session(path)
            # 0.1 s late at 12°/s is a 1.2° pitch error: several cm at the wall's edges,
            # far beyond the 2 mm the correctly timed build achieves.
            points, _, _ = build_points(session, mount_matrix("forward", "up"), offset_s=0.1)
            self.assertGreater(np.abs(points[:, 1] - self.WALL_Y).max(), 0.02)


if __name__ == "__main__":
    unittest.main()
