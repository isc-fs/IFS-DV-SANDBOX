"""Small 2D geometry + message conversion helpers. Plain numpy, no magic.

Conventions (REP-103): x forward, y left, z up, yaw counter-clockwise.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap_angle(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0      # body-frame forward speed (m/s)
    vy: float = 0.0      # body-frame lateral speed (m/s)
    yaw_rate: float = 0.0
    stamp: float = 0.0   # seconds

    @classmethod
    def from_odometry(cls, msg) -> "Pose2D":
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        return cls(
            x=p.x, y=p.y, yaw=yaw_from_quat(q.x, q.y, q.z, q.w),
            vx=t.linear.x, vy=t.linear.y, yaw_rate=t.angular.z,
            stamp=stamp_to_sec(msg.header.stamp),
        )

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)


def world_to_body(xy: np.ndarray, pose: Pose2D) -> np.ndarray:
    """(N,2) world points -> (N,2) points in the car frame."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    d = xy - (pose.x, pose.y)
    return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))


def body_to_world(xy: np.ndarray, pose: Pose2D) -> np.ndarray:
    """(N,2) car-frame points -> (N,2) world points."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.column_stack((c * xy[:, 0] - s * xy[:, 1] + pose.x,
                            s * xy[:, 0] + c * xy[:, 1] + pose.y))


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# --- ConeArray <-> numpy ------------------------------------------------

def cones_to_numpy(msg):
    """ConeArray -> (xy (N,2), colors (N,) uint8, confidence (N,) float)."""
    n = len(msg.cones)
    xy = np.empty((n, 2))
    colors = np.empty(n, dtype=np.uint8)
    conf = np.empty(n, dtype=np.float32)
    for i, c in enumerate(msg.cones):
        xy[i] = (c.x, c.y)
        colors[i] = c.color
        conf[i] = c.confidence
    return xy, colors, conf


def numpy_to_cones(header, xy, colors, confidence=None):
    """(N,2) points + (N,) colours -> ConeArray with the given header."""
    from sandbox_msgs.msg import Cone, ConeArray

    msg = ConeArray()
    msg.header = header
    if confidence is None:
        confidence = np.ones(len(xy), dtype=np.float32)
    for (x, y), col, cf in zip(np.asarray(xy).reshape(-1, 2), colors, confidence):
        msg.cones.append(Cone(x=float(x), y=float(y), color=int(col), confidence=float(cf)))
    return msg


def colour_by_side(xy, max_gap=7.0, max_turn_deg=45.0):
    """Blue / yellow for cones in the car frame, from geometry only.

    For a perception that can't see colours (IFSSIM's LiDAR has no colour
    information). "Left of the car = blue" goes wrong in every bend: the
    outer edge ahead swings over to the other side. So follow each edge
    instead:
      1. start from the nearest cone and its partner across the track
         (2-5 m away, the most side by side): the one further left starts the blue
         edge, the other the yellow edge. In a hairpin both can be on the
         same side of the car, which is why "nearest cone on the left" isn't
         enough;
      2. step each edge to the closest cone ahead that keeps its direction
         (turn < max_turn_deg, closer than max_gap: one missed cone is fine).
    Cones on neither edge: left of the car = blue, right = yellow.
    """
    from sandbox_msgs.msg import Cone

    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    colors = np.where(xy[:, 1] > 0.0, Cone.BLUE, Cone.YELLOW).astype(np.uint8)
    seeds = _edge_seeds(xy)
    if seeds is None:
        return colors
    left, right, forward = seeds
    free = np.ones(len(xy), dtype=bool)
    edges = []                                       # [colour, last cone, direction]
    for start, colour in ((left, Cone.BLUE), (right, Cone.YELLOW)):
        free[start] = False
        colors[start] = colour
        edges.append([colour, start, forward.copy()])
    cos_max = math.cos(math.radians(max_turn_deg))
    while any(e[2] is not None for e in edges):      # grow both edges, one cone at a time
        for e in edges:
            if e[2] is None:
                continue
            idx = np.flatnonzero(free)
            v = xy[idx] - xy[e[1]]
            d = np.hypot(v[:, 0], v[:, 1])
            cos_turn = (v @ e[2]) / np.maximum(d, 1e-9)
            ok = (d > 0.5) & (d < max_gap) & (cos_turn > cos_max)
            if not ok.any():
                e[2] = None                          # this edge ends here
                continue
            k = np.flatnonzero(ok)[np.argmin((d * (2.0 - cos_turn))[ok])]   # close and straight first
            free[idx[k]] = False
            colors[idx[k]] = e[0]
            e[1], e[2] = idx[k], v[k] / d[k]
    return colors


def _edge_seeds(xy):
    """(left cone, right cone, forward direction) to start the two edges, or None."""
    ahead = np.flatnonzero(xy[:, 0] > -2.0)
    if len(ahead) < 2:
        return None
    first = ahead[np.argmin(np.hypot(xy[ahead, 0], xy[ahead, 1]))]
    v = xy[ahead] - xy[first]
    d = np.hypot(v[:, 0], v[:, 1])
    along = np.abs(v[:, 0]) / np.maximum(d, 1e-9)    # 0 = straight across the car, 1 = one behind the other
    ok = (d > 2.0) & (d < 5.0) & (along < 0.8)
    if not ok.any():
        return None
    # The most "across" one: in a hairpin the next cone on the SAME edge can be
    # as close as the one across the track, but it's further ahead.
    partner = ahead[np.flatnonzero(ok)[np.argmin(along[ok])]]
    left, right = (first, partner) if xy[first, 1] > xy[partner, 1] else (partner, first)
    w = xy[left] - xy[right]
    forward = np.array([w[1], -w[0]]) / np.hypot(w[0], w[1])   # perpendicular to the pair
    return left, right, (forward if forward[0] >= 0 else -forward)
