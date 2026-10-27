"""SLAM baseline: correct the odometry (localisation) + build the cone map (mapping).

  in : /sandbox/odom       Odometry, odom -> base_link   from the odometry stage
       /sandbox/cones      ConeArray (base_link)         from perception
       /testing_only/odom  ground-truth pose             (only with pose_source:=gt, the CHEAT)
  out: /sandbox/pose       Odometry, map -> base_link    the best estimate of the car pose
       TF map -> odom      the correction SLAM applies on top of odometry
       /sandbox/map        ConeArray (map), cones seen at least `min_hits` times

The TF tree is the standard ROS one (same as the real car):

    map --(SLAM: correction)--> odom --(odometry: smooth, drifts)--> base_link

Localisation, two options (parameter pose_source):
  odom  (default) trust the odometry: map -> odom stays at identity, so the
        pose IS the odometry, drift included. Watch the map smear when the
        odometry drifts: correcting that with the cones is what real SLAM does.
  gt    copy the simulator's ground-truth pose and set map -> odom to
        whatever makes odometry match it. Perfect, and impossible on the car.

Mapping: every detected cone is moved to the map frame using the pose AT
THE TIME OF THE SCAN (interpolated from a pose history), then matched to
the nearest known cone. Close enough -> same cone, average the position;
otherwise -> new cone. No outlier handling, no loop closure, and the
cones never correct the pose.

Real pipeline equivalent: IFS09-DV-PIPELINE/cone_slam (GTSAM factor graph:
cone observations correct the odometry, published as map -> odom).
"""
from __future__ import annotations

import bisect
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sandbox_msgs.msg import ConeArray
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import (Pose2D, body_to_world, cones_to_numpy, numpy_to_cones,
                                    quat_from_yaw, stamp_to_sec, wrap_angle)


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """a * b: pose b (expressed in frame a) moved into a's parent frame."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, float(wrap_angle(a.yaw + b.yaw)))


def inverse(a: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(-(c * a.x + s * a.y), -(-s * a.x + c * a.y), -a.yaw)


class SlamBaseline(Node):
    def __init__(self):
        super().__init__("slam")
        self.declare_parameter("pose_source", "odom")   # odom | gt
        self.declare_parameter("match_distance", 1.0)   # m
        self.declare_parameter("min_hits", 2)

        self.map_to_odom = Pose2D()                     # SLAM's correction (identity = trust odometry)
        self.gt_latest: Pose2D | None = None
        self.history: list[Pose2D] = []                 # recent map-frame poses, oldest first
        self.landmarks = np.empty((0, 2))
        self.hits = np.empty(0, dtype=int)
        self.color_votes = np.empty((0, 5), dtype=int)  # one counter per colour

        self.tf = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(Odometry, contract.POSE, 10)
        self.pub_map = self.create_publisher(ConeArray, contract.MAP, 10)
        self.create_subscription(Odometry, contract.ODOM, self.on_odom, 10)
        self.create_subscription(ConeArray, contract.CONES, self.on_cones, 10)
        self.source = self.get_parameter("pose_source").value
        if self.source == "gt":
            self.create_subscription(Odometry, contract.GT_ODOM, self.on_gt, contract.SENSOR_QOS)
        self.get_logger().info(f"slam baseline, pose_source={self.source}")

    # ---------------- localisation ----------------
    def on_gt(self, msg: Odometry):
        self.gt_latest = Pose2D.from_odometry(msg)

    def on_odom(self, msg: Odometry):
        odom = Pose2D.from_odometry(msg)
        if self.source == "gt" and self.gt_latest is not None:
            # choose map -> odom so that (map -> odom) * odom = ground truth
            self.map_to_odom = compose(self.gt_latest, inverse(odom))
        pose = compose(self.map_to_odom, odom)
        pose.vx, pose.vy, pose.yaw_rate, pose.stamp = odom.vx, odom.vy, odom.yaw_rate, odom.stamp
        self.publish(pose, msg.header.stamp)

    def publish(self, pose: Pose2D, stamp):
        self.history.append(pose)
        if len(self.history) > 400:
            self.history = self.history[-300:]
        header = Header(frame_id=contract.MAP_FRAME, stamp=stamp)

        out = Odometry(header=header, child_frame_id=contract.BASE_FRAME)
        out.pose.pose.position.x, out.pose.pose.position.y = pose.x, pose.y
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(pose.yaw)
        out.twist.twist.linear.x, out.twist.twist.linear.y = pose.vx, pose.vy
        out.twist.twist.angular.z = pose.yaw_rate
        self.pub_pose.publish(out)

        tf = TransformStamped(header=header, child_frame_id=contract.ODOM_FRAME)
        m = self.map_to_odom
        tf.transform.translation.x, tf.transform.translation.y = m.x, m.y
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = quat_from_yaw(m.yaw)
        self.tf.sendTransform(tf)

    def pose_at(self, t: float) -> Pose2D | None:
        """Pose at time t, linearly interpolated from the history."""
        if not self.history:
            return None
        times = [p.stamp for p in self.history]
        i = bisect.bisect_left(times, t)
        if i == 0:
            return self.history[0]
        if i >= len(times):
            return self.history[-1]
        a, b = self.history[i - 1], self.history[i]
        f = (t - a.stamp) / max(b.stamp - a.stamp, 1e-9)
        yaw = a.yaw + f * float(wrap_angle(b.yaw - a.yaw))
        return Pose2D(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), yaw, stamp=t)

    # ---------------- mapping ----------------
    def on_cones(self, msg: ConeArray):
        pose = self.pose_at(stamp_to_sec(msg.header.stamp))
        if pose is None:
            return
        xy, colors, _ = cones_to_numpy(msg)
        world = body_to_world(xy, pose)
        match = self.get_parameter("match_distance").value
        for (x, y), c in zip(world, colors):
            if len(self.landmarks):
                d = np.hypot(self.landmarks[:, 0] - x, self.landmarks[:, 1] - y)
                j = int(np.argmin(d))
                if d[j] < match:
                    n = self.hits[j]
                    self.landmarks[j] = (self.landmarks[j] * n + (x, y)) / (n + 1)
                    self.hits[j] += 1
                    self.color_votes[j, c] += 1
                    continue
            self.landmarks = np.vstack((self.landmarks, (x, y)))
            self.hits = np.append(self.hits, 1)
            votes = np.zeros((1, 5), dtype=int)
            votes[0, c] = 1
            self.color_votes = np.vstack((self.color_votes, votes))

        ok = self.hits >= self.get_parameter("min_hits").value
        colors = np.argmax(self.color_votes[ok], axis=1).astype(np.uint8) if ok.any() else []
        header = Header(frame_id=contract.MAP_FRAME, stamp=msg.header.stamp)
        self.pub_map.publish(numpy_to_cones(header, self.landmarks[ok], colors))


def main():
    run(SlamBaseline)


if __name__ == "__main__":
    main()
