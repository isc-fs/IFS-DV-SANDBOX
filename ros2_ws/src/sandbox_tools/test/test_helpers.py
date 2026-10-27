import math

import numpy as np

from sandbox_tools.geometry import Pose2D, body_to_world, quat_from_yaw, world_to_body, yaw_from_quat
from sandbox_tools.skew_model import add_skew, point_times, relative_motion


def test_world_body_round_trip():
    pose = Pose2D(3.0, -2.0, 0.7)
    pts = np.random.default_rng(0).uniform(-10, 10, (50, 2))
    assert np.allclose(body_to_world(world_to_body(pts, pose), pose), pts)


def test_point_ahead_is_positive_x():
    pose = Pose2D(1.0, 1.0, math.pi / 2)            # facing +y
    assert np.allclose(world_to_body([[1.0, 3.0]], pose), [[2.0, 0.0]])


def test_quaternion_yaw():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 3.0):
        assert math.isclose(yaw_from_quat(*quat_from_yaw(yaw)), yaw, abs_tol=1e-9)


def test_skew_times_and_static_car():
    xyz = np.array([[10.0, 5.0, 0.0], [10.0, -5.0, 0.0]])
    t = point_times(xyz)
    assert t[0] < t[1] <= 0.0                         # left side measured first
    assert np.allclose(add_skew(xyz, t, 0.0, 0.0, 0.0), xyz)   # no motion, no skew


def test_relative_motion_straight_and_turning():
    dx, dy, th = relative_motion(0.1, 10.0, 0.0, 0.0)
    assert math.isclose(dx, 1.0) and dy == 0.0
    dx, dy, th = relative_motion(math.pi, 1.0, 0.0, 1.0)      # half circle of radius 1
    assert math.isclose(dx, 0.0, abs_tol=1e-9) and math.isclose(dy, 2.0)
