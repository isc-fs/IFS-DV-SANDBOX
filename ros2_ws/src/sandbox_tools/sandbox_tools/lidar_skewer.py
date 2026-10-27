"""Adds realistic motion distortion to the (distortion-free) sim LiDAR.

  in : /lidar_points         instant snapshot from the sim
       /testing_only/odom    ground-truth twist of the car
  out: /lidar_points_skewed  same points as a sweeping sensor would see them,
                             plus a float32 field "t" = time offset (s) of each
                             point relative to header.stamp (end of sweep)

Start it with  sb-launch skew:=true . The model is in skew_model.py.
"""
from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.pointcloud import make_cloud, read_fields
from sandbox_tools.skew_model import add_skew, point_times


class LidarSkewer(Node):
    def __init__(self):
        super().__init__("lidar_skewer")
        self.declare_parameter("scan_period", 0.1)
        self.declare_parameter("fov_min_deg", -60.0)
        self.declare_parameter("fov_max_deg", 60.0)
        self.twist = None
        self.create_subscription(Odometry, contract.GT_ODOM, self.on_odom, contract.SENSOR_QOS)
        self.create_subscription(PointCloud2, contract.LIDAR, self.on_cloud, contract.SENSOR_QOS)
        self.pub = self.create_publisher(PointCloud2, contract.LIDAR_SKEWED, contract.SENSOR_QOS)

    def on_odom(self, msg: Odometry):
        self.twist = msg.twist.twist

    def on_cloud(self, msg: PointCloud2):
        if self.twist is None:
            return
        pts = read_fields(msg, ("x", "y", "z", "intensity"))
        t = point_times(pts[:, :3], self.get_parameter("scan_period").value,
                        (self.get_parameter("fov_min_deg").value,
                         self.get_parameter("fov_max_deg").value))
        tw = self.twist
        skewed = add_skew(pts[:, :3], t, tw.linear.x, tw.linear.y, tw.angular.z)
        self.pub.publish(make_cloud(msg.header, {
            "x": skewed[:, 0], "y": skewed[:, 1], "z": skewed[:, 2],
            "intensity": pts[:, 3], "t": t,
        }))


def main():
    run(LidarSkewer)


if __name__ == "__main__":
    main()
