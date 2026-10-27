"""PERCEPTION baseline: LiDAR points -> cones in the car frame.

  in : /lidar_points    sensor_msgs/PointCloud2 (frame hesai_lidar, ~10 Hz)
  out: /sandbox/cones   sandbox_msgs/ConeArray  (frame base_link, same stamp as the scan)

Steps (each one is deliberately naive, so there is plenty to improve):
  1. crop       keep points in front of the car, closer than max_range
  2. ground     drop points near the ground height (flat-ground assumption!)
  3. cluster    put the remaining points on a 2D grid and join touching cells
  4. filter     keep clusters with the size of a cone
  5. colour     yellow cones reflect more than blue ones: split the clusters
                into "bright" and "dark" by intensity. If they all look alike,
                fall back to "left of the car = blue, right = yellow".
                Clusters taller than 0.42 m are the big orange start/finish cones.

Real pipeline equivalent: IFS09-DV-PIPELINE/cone_detection
(RANSAC ground plane, DBSCAN clustering, colour from intensity, Numba).

The node logs how long each scan takes. A scan arrives every 100 ms, so
if processing takes longer, scans pile up and everything downstream gets
old data. Keep an eye on it.
"""
from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray
from scipy import ndimage
from sensor_msgs.msg import PointCloud2

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import numpy_to_cones
from sandbox_tools.pointcloud import read_fields


class PerceptionBaseline(Node):
    def __init__(self):
        super().__init__("perception")
        self.declare_parameter("input_topic", contract.LIDAR)
        self.declare_parameter("max_range", 20.0)
        self.declare_parameter("sensor_height", 1.10)     # LiDAR height above the ground (m)
        self.declare_parameter("min_height", 0.05)        # ignore points lower than this above ground
        self.declare_parameter("max_height", 0.55)        # cones are < 0.51 m tall
        self.declare_parameter("cell_size", 0.15)         # clustering grid (m)
        self.declare_parameter("min_points", 4)
        self.declare_parameter("max_extent", 0.6)         # cluster bigger than this is not a cone (m)
        self.declare_parameter("big_cone_height", 0.42)   # small cones 0.33 m, big orange 0.51 m

        self.times = []
        self.pub = self.create_publisher(ConeArray, contract.CONES, 10)
        topic = self.get_parameter("input_topic").value
        self.create_subscription(PointCloud2, topic, self.on_cloud, contract.SENSOR_QOS)
        self.get_logger().info(f"perception baseline listening on {topic}")

    def on_cloud(self, msg: PointCloud2):
        t0 = time.perf_counter()
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        pts = read_fields(msg, ("x", "y", "z", "intensity"))

        # 1. crop
        r = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(pts[:, 0] > 0.0) & (r < p("max_range"))]

        # 2. ground removal: height above a flat ground at -sensor_height
        h = pts[:, 2] + p("sensor_height")
        pts = pts[(h > p("min_height")) & (h < p("max_height"))]

        # 3. clustering on a 2D occupancy grid
        xy, intensity, top = self.cluster_centers(pts, p("cell_size"), p("min_points"), p("max_extent"),
                                                  p("sensor_height"))

        # 5. colour
        colors = self.classify(xy, intensity)
        colors[top > p("big_cone_height")] = Cone.ORANGE_BIG

        header = msg.header
        header.frame_id = contract.BASE_FRAME   # in sim the LiDAR frame = base_link (+ height)
        self.pub.publish(numpy_to_cones(header, xy, colors))
        self.report(time.perf_counter() - t0)

    @staticmethod
    def cluster_centers(pts: np.ndarray, cell: float, min_points: int, max_extent: float,
                        sensor_height: float = 1.10):
        """pts (N,4) x,y,z,intensity -> centres (M,2), mean intensity (M,), top height (M,)."""
        empty = np.empty((0, 2)), np.empty(0), np.empty(0)
        if len(pts) == 0:
            return empty
        xy = pts[:, :2]
        origin = xy.min(axis=0)
        idx = np.floor((xy - origin) / cell).astype(int)
        grid = np.zeros(idx.max(axis=0) + 1, dtype=bool)
        grid[idx[:, 0], idx[:, 1]] = True
        labels, n = ndimage.label(grid, structure=np.ones((3, 3)))   # 8-connected cells
        if n == 0:
            return empty
        lab = labels[idx[:, 0], idx[:, 1]]                             # label of each point
        count = np.bincount(lab, minlength=n + 1)
        cx = np.bincount(lab, xy[:, 0], minlength=n + 1) / np.maximum(count, 1)
        cy = np.bincount(lab, xy[:, 1], minlength=n + 1) / np.maximum(count, 1)
        ci = np.bincount(lab, pts[:, 3], minlength=n + 1) / np.maximum(count, 1)
        top = np.full(n + 1, -np.inf)
        np.maximum.at(top, lab, pts[:, 2] + sensor_height)            # highest point above ground
        # 4. size filter (extent of the cluster's cells)
        sl = ndimage.find_objects(labels)
        extent = np.array([0.0] + [max(s[0].stop - s[0].start, s[1].stop - s[1].start) * cell for s in sl])
        ok = (count >= min_points) & (extent <= max_extent)
        ok[0] = False                                                  # label 0 = empty cells
        return np.column_stack((cx[ok], cy[ok])), ci[ok], top[ok]

    @staticmethod
    def classify(xy: np.ndarray, intensity: np.ndarray) -> np.ndarray:
        by_side = np.where(xy[:, 1] > 0.0, Cone.BLUE, Cone.YELLOW).astype(np.uint8)
        if len(xy) < 2:
            return by_side
        # Intensity falls with distance squared beyond ~5 m: compensate.
        r = np.hypot(xy[:, 0], xy[:, 1])
        value = intensity * np.maximum(r / 5.0, 1.0) ** 2
        lo, hi = np.percentile(value, 10), np.percentile(value, 90)
        if hi < 1.5 * max(lo, 1e-6):          # everything looks the same: no colour info
            return by_side
        threshold = (lo + hi) / 2.0
        return np.where(value > threshold, Cone.YELLOW, Cone.BLUE).astype(np.uint8)

    def report(self, dt: float):
        self.times.append(dt)
        if len(self.times) >= 50:
            t = np.array(self.times) * 1000
            self.get_logger().info(f"processing: mean {t.mean():.1f} ms, max {t.max():.1f} ms per scan")
            self.times = []


def main():
    run(PerceptionBaseline)


if __name__ == "__main__":
    main()
