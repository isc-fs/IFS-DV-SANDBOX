"""Ground-truth "odometry": perfect, drift-free odometry from the simulator.

    sb-launch odometry:=gt

Use it to work on SLAM (or anything downstream) without odometry drift.
Same contract as a real odometry node: /sandbox/odom + TF odom -> base_link.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from sandbox_tools import contract
from sandbox_tools.node_utils import run


class GtOdom(Node):
    def __init__(self):
        super().__init__("gt_odom")
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, contract.ODOM, 10)
        self.create_subscription(Odometry, contract.GT_ODOM, self.on_gt, contract.SENSOR_QOS)
        self.last = -1e9

    def on_gt(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.last < 0.01:            # ~100 Hz is plenty
            return
        self.last = t
        msg.header.frame_id = contract.ODOM_FRAME
        msg.child_frame_id = contract.BASE_FRAME
        self.pub.publish(msg)
        tf = TransformStamped(header=msg.header, child_frame_id=contract.BASE_FRAME)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        tf.transform.translation.x, tf.transform.translation.y = p.x, p.y
        tf.transform.rotation = q
        self.tf.sendTransform(tf)


def main():
    run(GtOdom)


if __name__ == "__main__":
    main()
