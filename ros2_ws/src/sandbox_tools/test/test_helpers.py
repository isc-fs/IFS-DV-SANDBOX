import math

import numpy as np

from sandbox_msgs.msg import Cone

from sandbox_tools.geometry import (Pose2D, body_to_world, colour_by_side, quat_from_yaw, world_to_body,
                                    yaw_from_quat)
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


def test_colour_by_side_follows_the_edges_through_a_bend():
    # Left-hand bend, radius 9 m at the centre, track 3 m wide, a cone every 3 m.
    # Ahead in the bend the right edge crosses to y > 0, where "left = blue" fails.
    yellow, blue = [], []
    for r, out in ((10.5, yellow), (7.5, blue)):
        for a in np.arange(0.0, 2.0, 3.0 / r):
            out.append((r * math.sin(a), 9.0 - r * math.cos(a)))
    xy = np.array(yellow + blue)
    truth = [Cone.YELLOW] * len(yellow) + [Cone.BLUE] * len(blue)
    assert (xy[:len(yellow), 1] > 0).any()                       # the naive rule would fail here
    assert list(colour_by_side(xy)) == truth


def test_colour_by_side_in_a_hairpin_with_both_edges_on_one_side():
    # What perception:=ifs09 saw in IFSSIM track A's hairpin (car frame, rounded),
    # 1 s before the car hit a cone: a hard right turn, every cone on the right.
    blue = [(6, -1), (9, -3), (11, -5), (12, -7), (14, -10), (15, -13)]     # outer (left) edge
    yellow = [(5, -3), (7, -5), (9, -7), (10, -10), (12, -13)]             # inner (right) edge
    xy = np.array(blue + yellow, dtype=float)
    truth = [Cone.BLUE] * len(blue) + [Cone.YELLOW] * len(yellow)
    assert list(colour_by_side(xy)) == truth


def test_colour_by_side_doesnt_pair_two_cones_of_the_same_edge():
    # IFSSIM track A, entering the hairpin (car frame, rounded): the nearest cone
    # (5, -2) has a cone of its own edge 2.8 m away, (7, -4), and its real
    # partner across the track 3.2 m away, (6, 1).
    blue = [(6, 1), (8, 0), (10, -3), (11, -6), (11, -9), (10, -12), (9, -14)]
    yellow = [(5, -2), (7, -4), (8, -7), (7, -10)]
    xy = np.array(blue + yellow, dtype=float)
    truth = [Cone.BLUE] * len(blue) + [Cone.YELLOW] * len(yellow)
    assert list(colour_by_side(xy)) == truth
