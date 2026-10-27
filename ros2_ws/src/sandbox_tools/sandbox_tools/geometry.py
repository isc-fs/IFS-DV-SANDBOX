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
