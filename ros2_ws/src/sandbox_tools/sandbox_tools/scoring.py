"""Benchmark metrics and scores. Pure numpy, no ROS (see scorer.py for the node).

Every stage gets a score from 0 to 100, built from metrics measured
against the simulator's ground truth. The formulas are on purpose
simple; see docs/LEADERBOARD.md for what each one rewards.

Colour accuracy is reported but NOT part of any score: the real
pipeline is colour-blind by design (its planner doesn't need colours),
and it would be unfair to punish that.
"""
from __future__ import annotations

import numpy as np

BLUE, YELLOW = 1, 0
FS_CONE_PENALTY_S = 2.0      # Formula Student: +2 s per cone hit


def clamp01(x: float) -> float:
    return float(min(1.0, max(0.0, x)))


def match(a: np.ndarray, b: np.ndarray, max_dist: float):
    """Greedy one-to-one nearest matching of points a (N,2) to b (M,2).
    Returns list of (i, j, distance) with distance < max_dist."""
    if len(a) == 0 or len(b) == 0:
        return []
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    pairs = []
    used_a, used_b = set(), set()
    for flat in np.argsort(d, axis=None):
        i, j = divmod(int(flat), d.shape[1])
        if d[i, j] >= max_dist:
            break
        if i in used_a or j in used_b:
            continue
        used_a.add(i)
        used_b.add(j)
        pairs.append((i, j, float(d[i, j])))
    return pairs


def centre_offset(points: np.ndarray, blue: np.ndarray, yellow: np.ndarray) -> np.ndarray:
    """Approximate distance of each point from the track centre line:
    half the difference between the distance to the nearest blue and the
    nearest yellow cone (0 = perfectly centred). Far from every cone that
    difference is small again, so it's also at least "distance to the
    nearest cone - 2 m": leaving the track always gives a large offset."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) == 0 or len(blue) == 0 or len(yellow) == 0:
        return np.zeros(len(points))
    db = np.min(np.linalg.norm(points[:, None] - blue[None], axis=2), axis=1)
    dy = np.min(np.linalg.norm(points[:, None] - yellow[None], axis=2), axis=1)
    return np.maximum(np.abs(db - dy) / 2.0, np.minimum(db, dy) - 2.0)


def resample_path(xy: np.ndarray, step: float = 1.0, max_len: float = 15.0) -> np.ndarray:
    """Points every `step` metres along a polyline, up to `max_len`."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 2:
        return xy
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    targets = np.arange(0.0, min(s[-1], max_len) + 1e-9, step)
    return np.column_stack((np.interp(targets, s, xy[:, 0]), np.interp(targets, s, xy[:, 1])))


# ---------------------------------------------------------------- scores

def perception_score(recall, precision, pos_err, latency_p95):
    """35 % recall, 25 % precision, 20 % position (0 at 0.3 m), 20 % timing
    (full marks up to 50 ms, 0 at 150 ms). Timing and position only count as
    much as you detect (x recall): being fast at finding nothing earns nothing."""
    if recall is None:
        return None
    return round(100 * (0.35 * recall + 0.25 * precision
                        + 0.20 * recall * clamp01(1 - pos_err / 0.3)
                        + 0.20 * recall * clamp01((0.150 - latency_p95) / 0.100)), 1)


def bag_perception_score(agree_recall, agree_precision, latency_p95, processed):
    """Real car bag: quality (F1 of agreement with the real pipeline) times a
    speed factor between 0.5 and 1 (timing: full up to 50 ms, 0 at 150 ms;
    processed: fraction of scans answered)."""
    if agree_recall is None:
        return None
    p = agree_precision or 0.0
    f1 = 2 * agree_recall * p / max(agree_recall + p, 1e-9)
    timing = clamp01((0.150 - latency_p95) / 0.100) if latency_p95 is not None else 0.0
    return round(100 * f1 * (0.5 + 0.5 * (timing + processed) / 2), 1)


def odometry_score(drift_percent):
    """Drift = max position error / distance driven. 100 at 0 %, 0 at 3 %
    (a good EKF on a real car drifts around 1-2 %)."""
    if drift_percent is None:
        return None
    return round(100 * clamp01(1 - drift_percent / 3.0), 1)


def slam_score(pose_err_mean, map_recall, map_precision, map_err):
    """40 % pose (0 at 0.5 m mean error), 20 % map recall, 20 % map precision,
    20 % map position (0 at 0.3 m)."""
    if pose_err_mean is None:
        return None
    mr = map_recall or 0.0
    mp = map_precision or 0.0
    me = 0.3 if map_err is None else map_err
    return round(100 * (0.40 * clamp01(1 - pose_err_mean / 0.5) + 0.20 * mr + 0.20 * mp
                        + 0.20 * clamp01(1 - me / 0.3)), 1)


def planning_score(offset_mean, outside_frac, length_mean):
    """50 % centring (0 at 0.75 m mean offset), 30 % staying inside the track,
    20 % looking far enough ahead (full marks at 15 m)."""
    if offset_mean is None:
        return None
    return round(100 * (0.50 * clamp01(1 - offset_mean / 0.75) + 0.30 * (1 - outside_frac)
                        + 0.20 * clamp01(length_mean / 15.0)), 1)


def control_score(cte_mean, cones_hit, steer_rate_rms):
    """50 % tracking (0 at 0.75 m mean offset from the centre), 30 % cones
    (0 at 5 hits), 20 % smooth steering (0 at 3 /s RMS steering rate)."""
    if cte_mean is None:
        return None
    return round(100 * (0.50 * clamp01(1 - cte_mean / 0.75) + 0.30 * clamp01(1 - cones_hit / 5)
                        + 0.20 * clamp01(1 - steer_rate_rms / 3.0)), 1)


def corrected_time(lap_times, cones_hit, laps_required):
    """Total time of the required laps + FS cone penalty, or None if DNF."""
    if len(lap_times) < laps_required:
        return None
    return round(sum(lap_times[:laps_required]) + FS_CONE_PENALTY_S * cones_hit, 2)


def overall_score(time_s, reference_time_s):
    """100 = as fast as the reference (real pipeline if available), >100 = faster."""
    if time_s is None or reference_time_s is None:
        return 0.0 if time_s is None else None
    return round(100.0 * reference_time_s / time_s, 1)
