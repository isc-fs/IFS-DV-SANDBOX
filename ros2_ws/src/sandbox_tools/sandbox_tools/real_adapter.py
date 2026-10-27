"""Republishes the REAL pipeline (IFS09-DV-PIPELINE) on the sandbox contract,
so the same scorer can score it. Used for the leaderboard reference runs
(`sb-launch real:=true`, see tools/benchmark_real.sh).

  /Conos_raw   MarkerArray, base_link   -> /sandbox/cones (colour UNKNOWN: the
                                            real perception is colour-blind)
  /odom        Odometry, odom frame     -> /sandbox/odom (the real EKF, odometry_filter)
  /slam/pose   Odometry, slam map frame -> /sandbox/pose
  /Conos       MarkerArray, slam map    -> /sandbox/map
  /Path        Path, slam map           -> /sandbox/path
  (control: the real stack drives /control_command itself)

The real SLAM starts its map frame wherever the car is when it starts, so
everything in that frame is moved into the sandbox world frame with the
ground-truth pose at the first /slam/pose message.
"""
from __future__ import annotations

import bisect
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, numpy_to_cones, quat_from_yaw, stamp_to_sec, yaw_from_quat


class RealAdapter(Node):
    def __init__(self):
        super().__init__("real_adapter")
        self.gt_t, self.gt = [], []
        self.align = None                 # (tx, ty, theta): world <- slam map
        self.map_cones = {}               # marker id -> (x, y) in slam map frame
        self.pub_cones = self.create_publisher(ConeArray, contract.CONES, 10)
        self.pub_pose = self.create_publisher(Odometry, contract.POSE, 10)
        self.pub_odom = self.create_publisher(Odometry, contract.ODOM, 10)
        self.pub_map = self.create_publisher(ConeArray, contract.MAP, 10)
        self.pub_path = self.create_publisher(Path, contract.PATH, 10)
        self.create_subscription(Odometry, contract.GT_ODOM, self.on_gt, contract.SENSOR_QOS)
        self.create_subscription(MarkerArray, "/Conos_raw", self.on_raw, 10)
        self.create_subscription(Odometry, "/slam/pose", self.on_pose, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, contract.SENSOR_QOS)
        self.create_subscription(MarkerArray, "/Conos", self.on_map, 10)
        self.create_subscription(Path, "/Path", self.on_path, 10)

    def on_gt(self, msg: Odometry):
        p = Pose2D.from_odometry(msg)
        self.gt_t.append(p.stamp)
        self.gt.append(p)
        if len(self.gt_t) > 5000:
            self.gt_t, self.gt = self.gt_t[-4000:], self.gt[-4000:]

    def to_world(self, xy: np.ndarray) -> np.ndarray:
        tx, ty, th = self.align
        c, s = math.cos(th), math.sin(th)
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        return np.column_stack((c * xy[:, 0] - s * xy[:, 1] + tx, s * xy[:, 0] + c * xy[:, 1] + ty))

    def on_raw(self, msg: MarkerArray):
        pts = [(m.pose.position.x, m.pose.position.y) for m in msg.markers if m.action == Marker.ADD]
        stamp = next((m.header.stamp for m in msg.markers), None)
        if stamp is None:
            return
        header = Header(frame_id=contract.BASE_FRAME, stamp=stamp)
        self.pub_cones.publish(numpy_to_cones(header, np.array(pts).reshape(-1, 2),
                                              [Cone.UNKNOWN] * len(pts)))

    def on_pose(self, msg: Odometry):
        p = Pose2D.from_odometry(msg)
        if self.align is None:
            if not self.gt_t:
                # Real car bag: no ground truth, keep the SLAM's own frame.
                self.align = (0.0, 0.0, 0.0)
                self.get_logger().info("no ground truth: keeping the real SLAM frame as is")
        if self.align is None:
            i = min(bisect.bisect_left(self.gt_t, p.stamp), len(self.gt_t) - 1)
            g = self.gt[i]
            th = g.yaw - p.yaw
            c, s = math.cos(th), math.sin(th)
            self.align = (g.x - (c * p.x - s * p.y), g.y - (s * p.x + c * p.y), th)
            self.get_logger().info(f"aligned real SLAM frame to the world: {self.align}")
        (x, y), = self.to_world([(p.x, p.y)])
        out = Odometry(header=Header(frame_id=contract.MAP_FRAME, stamp=msg.header.stamp),
                       child_frame_id=contract.BASE_FRAME)
        out.pose.pose.position.x, out.pose.pose.position.y = float(x), float(y)
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(p.yaw + self.align[2])
        out.twist = msg.twist                      # body frame, unchanged
        self.pub_pose.publish(out)

    def on_odom(self, msg: Odometry):
        # The real odom and SLAM frames both start at the car's start pose, so
        # the same alignment moves the odometry into the world frame.
        if self.align is None:
            return
        p = Pose2D.from_odometry(msg)
        (x, y), = self.to_world([(p.x, p.y)])
        out = Odometry(header=Header(frame_id=contract.ODOM_FRAME, stamp=msg.header.stamp),
                       child_frame_id=contract.BASE_FRAME)
        out.pose.pose.position.x, out.pose.pose.position.y = float(x), float(y)
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = quat_from_yaw(p.yaw + self.align[2])
        out.twist = msg.twist
        self.pub_odom.publish(out)

    def on_map(self, msg: MarkerArray):
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                self.map_cones.clear()
            elif m.action == Marker.DELETE:
                self.map_cones.pop((m.ns, m.id), None)
            else:
                self.map_cones[(m.ns, m.id)] = (m.pose.position.x, m.pose.position.y)
        if self.align is None or not msg.markers:
            return
        xy = self.to_world(np.array(list(self.map_cones.values())).reshape(-1, 2))
        header = Header(frame_id=contract.MAP_FRAME, stamp=msg.markers[-1].header.stamp)
        self.pub_map.publish(numpy_to_cones(header, xy, [Cone.UNKNOWN] * len(xy)))

    def on_path(self, msg: Path):
        if self.align is None:
            return
        xy = self.to_world(np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses]).reshape(-1, 2))
        out = Path(header=Header(frame_id=contract.MAP_FRAME, stamp=msg.header.stamp))
        for (x, y), src in zip(xy, msg.poses):
            ps = PoseStamped(header=out.header)
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            q = src.pose.orientation
            o = ps.pose.orientation
            o.x, o.y, o.z, o.w = quat_from_yaw(yaw_from_quat(q.x, q.y, q.z, q.w) + self.align[2])
            out.poses.append(ps)
        self.pub_path.publish(out)


def main():
    run(RealAdapter)


if __name__ == "__main__":
    main()
