"""Ground-truth "perception": publishes the cones a perfect sensor would see.

Use it to work on SLAM / planning / control without depending on
perception. Its output follows the same contract as a real perception
node (ConeArray in base_link on /sandbox/cones), so nothing downstream
can tell the difference... unless you add noise:

  position_noise   std of Gaussian noise on x and y (m)
  miss_rate        probability of not seeing a visible cone
  unknown_rate     probability of a cone coming out with colour UNKNOWN
  swap_rate        probability of blue <-> yellow being swapped

e.g.  sb-launch perception:=gt  then
      ros2 param set /gt_cones position_noise 0.15
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from fs_msgs.msg import Track
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, numpy_to_cones, world_to_body


class GtCones(Node):
    def __init__(self):
        super().__init__("gt_cones")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("max_range", 20.0)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("position_noise", 0.0)
        self.declare_parameter("miss_rate", 0.0)
        self.declare_parameter("unknown_rate", 0.0)
        self.declare_parameter("swap_rate", 0.0)
        self.declare_parameter("seed", 0)

        self.rng = np.random.default_rng(self.get_parameter("seed").value)
        self.track_xy = np.empty((0, 2))
        self.track_colors = np.empty(0, dtype=np.uint8)
        self.odom: Odometry | None = None

        self.create_subscription(Track, contract.GT_TRACK, self.on_track, contract.LATCHED_QOS)
        self.create_subscription(Odometry, contract.GT_ODOM, self.on_odom, contract.SENSOR_QOS)
        self.pub = self.create_publisher(ConeArray, contract.CONES, 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)
        self.get_logger().info("Publishing ground-truth cones on " + contract.CONES)

    def on_track(self, msg: Track):
        self.track_xy = np.array([(c.location.x, c.location.y) for c in msg.track]).reshape(-1, 2)
        self.track_colors = np.array([c.color for c in msg.track], dtype=np.uint8)

    def on_odom(self, msg: Odometry):
        self.odom = msg

    def tick(self):
        if self.odom is None or len(self.track_xy) == 0:
            return
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        pose = Pose2D.from_odometry(self.odom)
        body = world_to_body(self.track_xy, pose)
        rng_m = np.hypot(body[:, 0], body[:, 1])
        bearing = np.degrees(np.abs(np.arctan2(body[:, 1], body[:, 0])))
        keep = (rng_m < p("max_range")) & (bearing < p("fov_deg") / 2.0)
        keep &= self.rng.random(len(keep)) >= p("miss_rate")

        xy = body[keep]
        colors = self.track_colors[keep].copy()
        if p("position_noise") > 0:
            xy = xy + self.rng.normal(0.0, p("position_noise"), xy.shape)
        swap = self.rng.random(len(colors)) < p("swap_rate")
        blue, yellow = colors == Cone.BLUE, colors == Cone.YELLOW
        colors[swap & blue] = Cone.YELLOW
        colors[swap & yellow] = Cone.BLUE
        colors[self.rng.random(len(colors)) < p("unknown_rate")] = Cone.UNKNOWN

        header = self.odom.header
        header.frame_id = contract.BASE_FRAME
        self.pub.publish(numpy_to_cones(header, xy, colors))


def main():
    run(GtCones)


if __name__ == "__main__":
    main()
