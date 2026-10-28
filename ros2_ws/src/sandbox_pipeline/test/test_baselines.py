import numpy as np
import pytest

from sandbox_pipeline.control_baseline import ControlBaseline
from sandbox_pipeline.perception_baseline import PerceptionBaseline


def test_clustering_finds_two_cones():
    rng = np.random.default_rng(0)
    cone = lambda x, y: np.column_stack((rng.normal(x, 0.03, 30), rng.normal(y, 0.03, 30),
                                         rng.uniform(-1.0, -0.8, 30), np.full(30, 0.5)))
    pts = np.vstack((cone(5.0, 1.5), cone(8.0, -1.5)))
    xy, _, _ = PerceptionBaseline.cluster_centers(pts, 0.15, 4, 0.6)
    assert len(xy) == 2
    assert np.allclose(sorted(xy[:, 0]), [5.0, 8.0], atol=0.1)


@pytest.mark.parametrize("ground_z", [-1.10, 0.0])   # mock / IFSSIM put z = 0 at different heights
def test_ground_removal_keeps_only_the_cone(ground_z):
    rng = np.random.default_rng(0)
    ground = np.column_stack((rng.uniform(1, 20, 2000), rng.uniform(-5, 5, 2000),
                              rng.normal(ground_z, 0.005, 2000), np.zeros(2000)))
    cone = np.column_stack((rng.normal(6, 0.03, 40), rng.normal(1, 0.03, 40),
                            ground_z + rng.uniform(0.1, 0.3, 40), np.zeros(40)))
    kept, found_z = PerceptionBaseline.remove_ground(np.vstack((ground, cone)), 20.0, 0.05, 0.55)
    assert abs(found_z - ground_z) < 0.01
    assert len(kept) == 40 and np.all(np.abs(kept[:, 0] - 6) < 0.2)


def test_lookahead_point_on_straight_path():
    path = np.array([[0.0, 0.0], [10.0, 0.0]])
    assert np.allclose(ControlBaseline.point_at_distance(path, 4.0), [4.0, 0.0], atol=1e-3)


def test_slam_compose_inverse_round_trip():
    from sandbox_pipeline.slam_baseline import compose, inverse
    from sandbox_tools.geometry import Pose2D
    a, b = Pose2D(1.0, 2.0, 0.5), Pose2D(-3.0, 0.5, -1.2)
    c = compose(inverse(a), compose(a, b))
    assert abs(c.x - b.x) < 1e-9 and abs(c.y - b.y) < 1e-9 and abs(c.yaw - b.yaw) < 1e-9
