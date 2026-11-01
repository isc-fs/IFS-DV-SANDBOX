"""My PERCEPTION node, starting from the IFS09 pipeline's cone detection.

    sb-launch perception:=my_pipeline/perception
    sb-score perception:=my_pipeline/perception     # vs perception:=ifs09 (the original)

  in : /lidar_points   PointCloud2 (x, y, z, intensity), frame hesai_lidar
  out: /sandbox/cones  ConeArray in base_link, header.stamp = stamp of the scan

Your copy of the real code is in my_cone_detection/ (made by `sb-new --from ifs09`
from vendor/ifs09_cone_detection, see vendor/README.md for the pipeline commit).
It's yours now: change anything. The main files:

  config.py               every parameter, with what it does (one ROS parameter each)
  cone_detection.py       the detector: crop -> ground -> clustering -> cone fit
  ground_methods.py       ground removal (RANSAC and the alternatives being tested)
  clustering.py           DBSCAN implementations
  cone_fit.py             fitting a cone template to each cluster
  strategies/             the algorithm this node runs (ConeDetectionStrategy)

Where your idea goes:
  - it changes one part (ground, clustering, fit...): edit that file, or add an
    option to config.py so you can switch between the old and the new way;
  - it's a new algorithm: write it in my_cone_detection/strategies/my_strategy.py
    (started for you). STRATEGY below picks what runs.
  - it needs more than x, y, z (e.g. `ring`, only on the real bag): read the
    field in on_cloud() and pass it to your strategy.

Colour: the real perception is colour-blind, so the sandbox adds a colour per
cone (sandbox_tools/ifs09.py), as for perception:=ifs09: from intensity when it
tells blue from yellow (the mock), otherwise by following the two track edges
(IFSSIM, sandbox_tools/geometry.py colour_by_side).
"""
import logging

import numpy as np
from rclpy.node import Node
from sandbox_msgs.msg import ConeArray
from sensor_msgs.msg import PointCloud2

from my_cone_detection.config import ConeDetectionConfig
from my_cone_detection.strategies.my_strategy import MyConeDetection
from sandbox_tools import contract
from sandbox_tools.geometry import numpy_to_cones
from sandbox_tools.ifs09 import Ifs09Perception, declare_config
from sandbox_tools.node_utils import run
from sandbox_tools.pointcloud import read_fields

# What runs. MyConeDetection starts as the real algorithm, unchanged. To run the
# original next to yours, import BaseConeDetection from my_cone_detection.strategies.
STRATEGY = MyConeDetection

_default = None


def find_cones(points: np.ndarray, detector: Ifs09Perception | None = None):
    """points: (N, 4) x, y, z, intensity.  Return (xy (M, 2), colors (M,))."""
    global _default
    if len(points) == 0:
        return np.empty((0, 2)), np.empty(0, dtype=np.uint8)
    if detector is None:                  # tests: default config, built once
        if _default is None:
            _default = Ifs09Perception(STRATEGY, ConeDetectionConfig(), logging.getLogger("perception"))
        detector = _default
    return detector.find_cones(points)


class Perception(Node):
    def __init__(self):
        super().__init__("perception")
        config = declare_config(self, ConeDetectionConfig)
        self.get_logger().info("Numba warm-up (10-20 s the first time after a change, ~2 s after)")
        self.detector = Ifs09Perception(STRATEGY, config, self.get_logger())
        self.pub = self.create_publisher(ConeArray, contract.CONES, 10)
        self.create_subscription(PointCloud2, contract.LIDAR, self.on_cloud, contract.SENSOR_QOS)
        self.get_logger().info(f"perception ready, strategy {STRATEGY.__name__}")

    def on_cloud(self, msg: PointCloud2):
        xy, colors = find_cones(read_fields(msg, ("x", "y", "z", "intensity")), self.detector)
        header = msg.header
        header.frame_id = contract.BASE_FRAME
        self.pub.publish(numpy_to_cones(header, xy, colors))


def main():
    run(Perception)


if __name__ == "__main__":
    main()
