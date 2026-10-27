"""Synthetic LiDAR motion distortion ("skew"). Pure numpy, no ROS.

Why this exists: the simulator renders each LiDAR scan as an instant
snapshot, so sim clouds have NO motion distortion. The real Hesai sweeps
its field of view over ~100 ms, and the car keeps moving meanwhile, so
points measured early in the sweep are expressed in a different car pose
than points measured at the end.

This module takes an undistorted scan plus the car's motion and produces
what a sweeping sensor would have measured. Because the original cloud
is kept, you have the exact ground truth to check a deskewing algorithm
against. Used by the `lidar_skewer` node and by challenges/lidar_deskew.

Model (deliberately simple, document your own if you improve it):
  * the sweep goes from the LEFT edge of the FOV (+60 deg) to the RIGHT
    edge (-60 deg) at constant angular speed;
  * the cloud's header.stamp is the END of the sweep, so each point gets
    a time offset t in [-scan_period, 0] seconds;
  * the car moves with a constant body twist (vx, vy, wz) during the sweep.
"""
from __future__ import annotations

import numpy as np


def point_times(xyz: np.ndarray, scan_period: float = 0.1,
                fov_deg: tuple[float, float] = (-60.0, 60.0)) -> np.ndarray:
    """Per-point time offset (s) relative to the end of the sweep, in [-scan_period, 0]."""
    az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
    lo, hi = fov_deg
    frac = np.clip((hi - az) / (hi - lo), 0.0, 1.0)   # 0 at left edge, 1 at right edge
    return (frac - 1.0) * scan_period


def relative_motion(tau, vx: float, vy: float, wz: float):
    """Pose reached after moving for `tau` seconds with constant body twist,
    expressed in the starting frame. Returns (dx, dy, dtheta), vectorised over tau."""
    tau = np.asarray(tau, dtype=float)
    th = wz * tau
    if abs(wz) < 1e-6:
        return vx * tau, vy * tau, th
    s, c = np.sin(th), np.cos(th)
    dx = (vx * s - vy * (1.0 - c)) / wz
    dy = (vx * (1.0 - c) + vy * s) / wz
    return dx, dy, th


def add_skew(xyz: np.ndarray, t: np.ndarray, vx: float, vy: float = 0.0,
             wz: float = 0.0) -> np.ndarray:
    """Distort an instantaneous scan.

    xyz: (N,3) points in the sensor frame at the END of the sweep (t = 0).
    t:   (N,) time offsets <= 0 of each point.
    Returns (N,3) points as a sweeping sensor would report them: each one
    in the sensor frame at its own time t.

    A point q (end frame) seen from the frame at time t is  M q,  where M
    is the motion from t to the end of the sweep expressed in the frame at
    t:  p = R(dtheta) q + d.
    """
    dx, dy, th = relative_motion(-np.asarray(t), vx, vy, wz)
    c, s = np.cos(th), np.sin(th)
    out = np.empty_like(xyz)
    out[:, 0] = c * xyz[:, 0] - s * xyz[:, 1] + dx
    out[:, 1] = s * xyz[:, 0] + c * xyz[:, 1] + dy
    out[:, 2] = xyz[:, 2]
    return out
