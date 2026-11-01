"""The glue between the IFS09 perception code and the sandbox (sandbox_tools/ifs09.py)."""
import logging

import numpy as np
import pytest
from sandbox_msgs.msg import Cone

from sandbox_tools.ifs09 import Ifs09Perception, colour_by_intensity, cone_intensity

GROUND_Z = -1.10          # mock: the LiDAR is 1.10 m above the ground


def cone_points(x, y, height=0.33, radius=0.114, intensity=1.0, rng=None):
    """Points on the side of a cone facing a LiDAR at the origin."""
    rng = rng or np.random.default_rng(0)
    facing = np.arctan2(-y, -x)
    h = rng.uniform(0.0, height, 300)
    a = facing + rng.uniform(-np.pi / 2, np.pi / 2, 300)
    r = radius * (1.0 - h / height)
    return np.column_stack((x + r * np.cos(a), y + r * np.sin(a), GROUND_Z + h, np.full(300, intensity)))


def ground_points(n=6000, rng=None):
    rng = rng or np.random.default_rng(1)
    return np.column_stack((rng.uniform(1, 20, n), rng.uniform(-6, 6, n),
                            GROUND_Z + rng.normal(0, 0.005, n), rng.uniform(0.0, 0.1, n)))


def test_cone_intensity_ignores_the_ground_around_the_cone():
    pts = np.vstack((cone_points(5.0, 1.5, intensity=0.8), ground_points()))
    assert cone_intensity(np.array([[5.0, 1.5]]), pts)[0] == pytest.approx(0.8, abs=0.05)
    assert np.isnan(cone_intensity(np.array([[50.0, 0.0]]), pts)[0])     # no points there


def test_bright_cones_are_yellow_dark_cones_blue():
    xy = np.array([[5.0, 1.5], [9.0, 1.5], [5.0, -1.5], [9.0, -1.5]])
    bright = {(5.0, -1.5), (9.0, -1.5)}
    # Like the sims: intensity = reflectance * (5 m / range)^2
    pts = np.vstack([cone_points(x, y, intensity=(1.0 if (x, y) in bright else 0.2)
                                 * min(1.0, (5.0 / np.hypot(x, y)) ** 2)) for x, y in xy])
    assert list(colour_by_intensity(xy, pts)) == [Cone.BLUE, Cone.BLUE, Cone.YELLOW, Cone.YELLOW]


def test_no_colour_information_falls_back_to_the_side():
    xy = np.array([[5.0, 1.5], [5.0, -1.5]])
    pts = np.vstack([cone_points(x, y, intensity=0.5) for x, y in xy])
    assert list(colour_by_intensity(xy, pts)) == [Cone.BLUE, Cone.YELLOW]     # + y = left


def test_the_real_detector_finds_cones_in_a_sandbox_scan():
    pytest.importorskip("cone_detection")       # the vendored IFS09 code
    from cone_detection.config import ConeDetectionConfig
    from cone_detection.strategies import BaseConeDetection

    det = Ifs09Perception(BaseConeDetection, ConeDetectionConfig(), logging.getLogger("test"))
    truth = np.array([[4.0, 1.5], [8.0, 1.5], [12.0, 1.5], [4.0, -1.5], [8.0, -1.5], [12.0, -1.5]])
    pts = np.vstack([ground_points()] + [cone_points(x, y) for x, y in truth]).astype(np.float32)

    xy, colors = det.find_cones(pts)
    assert len(xy) == len(colors)
    found = [np.min(np.hypot(*(xy - t).T)) < 0.3 for t in truth] if len(xy) else [False]
    assert sum(found) >= 5, f"found {sum(found)} of {len(truth)} cones: {xy}"
    assert det.find_cones(np.empty((0, 4)))[0].shape == (0, 2)
