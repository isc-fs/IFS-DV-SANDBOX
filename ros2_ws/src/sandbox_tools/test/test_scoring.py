import numpy as np

from sandbox_tools import scoring


def straight_track(length=50.0, half_width=1.5):
    x = np.arange(0.0, length, 2.5)
    blue = np.column_stack((x, np.full_like(x, half_width)))
    yellow = np.column_stack((x, np.full_like(x, -half_width)))
    return blue, yellow


def test_centre_offset_on_and_off_track():
    blue, yellow = straight_track()
    off = scoring.centre_offset([(10.0, 0.0), (10.0, 1.0), (10.0, 30.0)], blue, yellow)
    assert off[0] < 0.05                    # centred
    assert 0.8 < off[1] < 1.2               # 1 m to the left
    assert off[2] > 20.0                    # far away from the track is NOT "centred"


def test_match_is_one_to_one():
    a = np.array([[0.0, 0.0], [0.1, 0.0]])
    b = np.array([[0.05, 0.0]])
    assert len(scoring.match(a, b, 0.5)) == 1


def test_resample_path_spacing_and_cap():
    pts = scoring.resample_path(np.array([[0.0, 0.0], [30.0, 0.0]]), 1.0, 15.0)
    assert len(pts) == 16 and np.isclose(pts[-1, 0], 15.0)


def test_scores_are_0_to_100_and_monotonic():
    best = scoring.perception_score(1.0, 1.0, 0.0, 0.01)
    worse = scoring.perception_score(0.5, 1.0, 0.0, 0.01)
    assert best == 100.0 and worse < best
    assert scoring.control_score(0.0, 0, 0.0) == 100.0
    assert scoring.control_score(0.0, 10, 0.0) == 70.0          # cones part goes to 0
    assert scoring.planning_score(0.0, 0.0, 15.0) == 100.0
    assert scoring.slam_score(0.0, 1.0, 1.0, 0.0) == 100.0


def test_corrected_time_and_overall():
    assert scoring.corrected_time([50.0], 2, 1) == 54.0         # +2 s per cone
    assert scoring.corrected_time([], 0, 1) is None             # DNF
    assert scoring.overall_score(50.0, 50.0) == 100.0
    assert scoring.overall_score(None, 50.0) == 0.0


def test_fast_but_empty_perception_scores_nothing():
    assert scoring.perception_score(0.0, 0.0, 0.3, 0.01) == 0.0
    assert scoring.bag_perception_score(0.0, 0.0, 0.01, 1.0) == 0.0
    assert scoring.bag_perception_score(1.0, 1.0, 0.01, 1.0) == 100.0
    assert scoring.bag_perception_score(1.0, 1.0, 1.0, 0.0) == 50.0     # right but too slow
