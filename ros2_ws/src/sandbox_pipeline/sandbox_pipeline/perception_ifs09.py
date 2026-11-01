"""PERCEPTION from IFS09-DV-PIPELINE: the real pipeline's cone detection, run in the sandbox.

    sb-launch perception:=ifs09

  in : /lidar_points    sensor_msgs/PointCloud2 (frame hesai_lidar)
  out: /sandbox/cones   sandbox_msgs/ConeArray  (frame base_link, same stamp as the scan)

The algorithm is the real one, unchanged: a pinned copy of the pipeline's
cone_detection package (vendor/ifs09_cone_detection, see vendor/README.md),
strategy "base": RANSAC ground plane -> DBSCAN clustering -> cone template fit.
Only its ROS node is replaced by this one, which speaks the sandbox topics.

Parameters: one per ConeDetectionConfig field, with the same names and defaults
as the real node (= bringup/config/params.yaml of the pipeline). Read at start-up.

Colour: the real perception is colour-blind (the real SLAM and planner don't
need colour), but the sandbox planner pairs blue with yellow. So the sandbox
adds a colour per cone from intensity, the same way as the baseline
(sandbox_tools/ifs09.py). Big orange cones come from the fitted height.

Want to change it? `sb-new <you> --from ifs09` gives you your own copy of this
code in your package (docs/LEVELS.md, Level 3).
"""
from __future__ import annotations

import time

import numpy as np
from cone_detection.config import ConeDetectionConfig
from cone_detection.strategies import BaseConeDetection
from rclpy.node import Node
from sandbox_msgs.msg import ConeArray
from sensor_msgs.msg import PointCloud2

from sandbox_tools import contract
from sandbox_tools.geometry import numpy_to_cones
from sandbox_tools.ifs09 import Ifs09Perception, declare_config
from sandbox_tools.node_utils import run
from sandbox_tools.pointcloud import read_fields


class PerceptionIfs09(Node):
    def __init__(self):
        super().__init__("perception")
        self.declare_parameter("input_topic", contract.LIDAR)
        config = declare_config(self, ConeDetectionConfig)
        self.get_logger().info("IFS09 cone detection: Numba warm-up (10-20 s the very first time, ~2 s after)")
        self.detector = Ifs09Perception(BaseConeDetection, config, self.get_logger())

        self.times = []
        self.pub = self.create_publisher(ConeArray, contract.CONES, 10)
        topic = self.get_parameter("input_topic").value
        self.create_subscription(PointCloud2, topic, self.on_cloud, contract.SENSOR_QOS)
        self.get_logger().info(f"IFS09 cone detection listening on {topic}")

    def on_cloud(self, msg: PointCloud2):
        t0 = time.perf_counter()
        xy, colors = self.detector.find_cones(read_fields(msg, ("x", "y", "z", "intensity")))
        header = msg.header
        header.frame_id = contract.BASE_FRAME   # like the real node: cones in the sensor's xy
        self.pub.publish(numpy_to_cones(header, xy, colors))
        self.report(time.perf_counter() - t0)

    def report(self, dt: float):
        self.times.append(dt)
        if len(self.times) >= 50:
            t = np.array(self.times) * 1000
            self.get_logger().info(f"processing: mean {t.mean():.1f} ms, max {t.max():.1f} ms per scan")
            self.times = []


def main():
    run(PerceptionIfs09)


if __name__ == "__main__":
    main()
