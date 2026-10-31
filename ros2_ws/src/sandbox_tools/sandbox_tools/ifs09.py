"""Run IFS09-DV-PIPELINE perception code on the sandbox contract.

The real pipeline's cone detection (vendor/ifs09_cone_detection, Python
package `cone_detection`) works on plain numpy: a strategy takes an (N, 3)
scan and returns cones. This module is the glue between that and the
sandbox:

  - builds a strategy with its ConeDetectionConfig, and runs the Numba
    warm-up once (10-20 s the very first time, ~2 s after that);
  - runs it on a sandbox scan (N, 4: x y z intensity);
  - adds what the sandbox needs and the real perception doesn't give:
    a colour per cone. The real perception is colour-blind (its SLAM and
    planner don't use colour); the sandbox planner pairs blue with yellow,
    so the colour is estimated here from intensity, the same way as the
    sandbox baseline. Big orange cones come from the strategy's height.

Used by sandbox_pipeline/perception_ifs09.py (perception:=ifs09) and by
the perception.py that `sb-new <you> --from ifs09` gives you.
"""
from __future__ import annotations

import dataclasses
import inspect
import typing

import numpy as np
from sandbox_msgs.msg import Cone

COLOUR_RADIUS = 0.25   # m: points this close (in xy) to a cone count for its colour


class Ifs09Perception:
    """One IFS09 strategy + config, ready to run on sandbox scans."""

    def __init__(self, strategy_cls, config, logger):
        self.strategy = strategy_cls(logger, config)
        self.strategy.configure()                 # Numba warm-up
        self.last_timings: dict[str, float] = {}
        # The base strategy fills per-stage timings; the abstract interface
        # doesn't require it, so a new strategy may not take the argument.
        params = inspect.signature(self.strategy.detect_cones).parameters
        self._timings_arg = "stage_timings" in params

    def find_cones(self, points: np.ndarray):
        """points: (N, 4) x, y, z, intensity in the LiDAR frame.
        Return (xy (M, 2), colors (M,)) in base_link, like every sandbox perception."""
        points = np.asarray(points, dtype=np.float32).reshape(-1, 4)
        if len(points) == 0:
            return np.empty((0, 2)), np.empty(0, dtype=np.uint8)
        timings: dict[str, float] = {}
        xyz = np.ascontiguousarray(points[:, :3])
        if self._timings_arg:
            result = self.strategy.detect_cones(xyz, stage_timings=timings)
        else:
            result = self.strategy.detect_cones(xyz)
        self.last_timings = timings
        if not result.cones:
            return np.empty((0, 2)), np.empty(0, dtype=np.uint8)
        xy = np.array([[c.x, c.y] for c in result.cones], dtype=float)
        height = np.array([c.height_m for c in result.cones], dtype=float)
        colors = colour_by_intensity(xy, points)
        colors[height > self.strategy.big_orange_height_threshold_m()] = Cone.ORANGE_BIG
        return xy, colors


def colour_by_intensity(xy: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Blue / yellow for each cone from the intensity of the points around it.

    Same rule as the sandbox baseline: yellow reflects more than blue, so
    split the cones into bright and dark (compensating for distance). If
    they all look alike, fall back to the side: left = blue, right = yellow.
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    by_side = np.where(xy[:, 1] > 0.0, Cone.BLUE, Cone.YELLOW).astype(np.uint8)
    if len(xy) < 2 or len(points) == 0:
        return by_side
    intensity = cone_intensity(xy, points)
    ok = np.isfinite(intensity)
    if ok.sum() < 2:
        return by_side
    r = np.hypot(xy[:, 0], xy[:, 1])
    value = intensity * np.maximum(r / 5.0, 1.0) ** 2   # intensity falls with distance
    lo, hi = np.percentile(value[ok], 10), np.percentile(value[ok], 90)
    if hi < 1.5 * max(lo, 1e-6):                        # no colour information
        return by_side
    colors = np.where(value > (lo + hi) / 2.0, Cone.YELLOW, Cone.BLUE).astype(np.uint8)
    colors[~ok] = by_side[~ok]
    return colors


def cone_intensity(xy: np.ndarray, points: np.ndarray, radius: float = COLOUR_RADIUS) -> np.ndarray:
    """Mean intensity of the cone's own points: within `radius` of it in xy and
    above the lowest of them (so the ground around the cone doesn't count).
    NaN for a cone with no points."""
    out = np.full(len(xy), np.nan)
    # Only points in the grid cells around a cone can count: pick those first
    # (a real scan has ~174 k points, a few cones have a few hundred).
    lo_xy = xy.min(axis=0) - 2 * radius
    cone_cell = np.floor((xy - lo_xy) / radius).astype(np.int64)
    grid = np.zeros(tuple(cone_cell.max(axis=0) + 3), dtype=bool)
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            grid[cone_cell[:, 0] + i, cone_cell[:, 1] + j] = True
    cell = np.floor((points[:, :2] - lo_xy) / radius).astype(np.int64)
    inside = (cell >= 0).all(axis=1) & (cell[:, 0] < grid.shape[0]) & (cell[:, 1] < grid.shape[1])
    keep = np.flatnonzero(inside)
    keep = keep[grid[cell[keep, 0], cell[keep, 1]]]
    points = points[keep]
    order = np.argsort(points[:, 0], kind="stable")
    px, py, pz, pi = (points[order, k] for k in range(4))
    lo = np.searchsorted(px, xy[:, 0] - radius)
    hi = np.searchsorted(px, xy[:, 0] + radius)
    for k, (x, y) in enumerate(xy):
        s = slice(lo[k], hi[k])
        near = (px[s] - x) ** 2 + (py[s] - y) ** 2 < radius ** 2
        if not near.any():
            continue
        z, i = pz[s][near], pi[s][near]
        body = z > z.min() + 0.05
        out[k] = float(i[body].mean() if body.any() else i.mean())
    return out


def declare_config(node, config_cls):
    """One ROS parameter per config field (same name and default), like the
    real node, so `ros2 param` and params YAML files work. Returns the config
    built from the parameters' values at start-up."""
    hints = typing.get_type_hints(config_cls)
    values = {}
    for f in dataclasses.fields(config_cls):
        default = f.default
        if isinstance(default, int) and not isinstance(default, bool) and hints.get(f.name) is float:
            default = float(default)
        node.declare_parameter(f.name, default)
        values[f.name] = node.get_parameter(f.name).value
    return config_cls(**values)
