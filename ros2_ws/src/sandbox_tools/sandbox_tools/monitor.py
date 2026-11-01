"""Referee: lap times, cone hits and speed, measured from ground truth.

Prints one line per lap, e.g.
    LAP 2   41.37 s   avg 6.1 m/s   max 8.9 m/s   cones hit: 1

Publishes the same as text on /sandbox/monitor (std_msgs/String) so it
shows up in Foxglove. Use it to compare your pipeline against the
baseline (or against your teammates).

It also compares /sandbox/pose (whatever SLAM publishes) with the ground
truth and publishes the position error on /sandbox/monitor/pose_error
(std_msgs/Float32, metres). With the GT pose it's ~0; with yours it
shows how good your localisation is.
"""
from __future__ import annotations

import bisect
import math

import numpy as np
import rclpy
from fs_msgs.msg import Track
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sandbox_msgs.msg import Cone
from std_msgs.msg import Float32, String

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, world_to_body

CAR_HALF_LENGTH = 1.5   # m, IFS-08 is ~2.9 m long
CAR_HALF_WIDTH = 0.75   # m, ~1.4 m wide + a little margin


class Monitor(Node):
    def __init__(self):
        super().__init__("monitor")
        self.declare_parameter("gate_radius", 3.0)
        self.declare_parameter("min_lap_distance", 30.0)
        self.cones = np.empty((0, 2))
        self.gate = None
        self.last = None
        self.lap = 0
        self.lap_start = None
        self.dist_since_gate = 0.0
        self.speeds = []
        self.hit = set()
        self.hits_total = 0
        self.pose_err = []
        self.gt = None
        self.gt_hist: list[Pose2D] = []           # recent GT poses, to compare at the right time
        self.pub = self.create_publisher(String, "/sandbox/monitor", 10)
        self.pub_err = self.create_publisher(Float32, "/sandbox/monitor/pose_error", 10)
        self.create_subscription(Odometry, contract.POSE, self.on_pose, 10)
        self.create_subscription(Track, contract.GT_TRACK, self.on_track, contract.LATCHED_QOS)
        self.create_subscription(Odometry, contract.GT_ODOM, self.on_odom, contract.SENSOR_QOS)

    def on_track(self, msg: Track):
        self.cones = np.array([(c.location.x, c.location.y) for c in msg.track]).reshape(-1, 2)
        gate = [(c.location.x, c.location.y) for c in msg.track if c.color == Cone.ORANGE_BIG]
        self.gate = np.mean(gate, axis=0) if gate else None

    def say(self, text: str):
        self.get_logger().info(text)
        self.pub.publish(String(data=text))

    def on_pose(self, msg: Odometry):
        if not self.gt_hist:
            return
        p = Pose2D.from_odometry(msg)
        # ground truth at the pose's own timestamp (not the latest one: the
        # pipeline is a bit behind, and at speed that alone looks like error)
        times = [g.stamp for g in self.gt_hist]
        i = min(bisect.bisect_left(times, p.stamp), len(times) - 1)
        g = self.gt_hist[i]
        err = math.hypot(p.x - g.x, p.y - g.y)
        self.pose_err.append(err)
        self.pub_err.publish(Float32(data=float(err)))

    def on_odom(self, msg: Odometry):
        pose = Pose2D.from_odometry(msg)
        self.gt = pose
        self.gt_hist.append(pose)
        if len(self.gt_hist) > 600:
            self.gt_hist = self.gt_hist[-400:]
        if self.last is not None:
            self.dist_since_gate += math.hypot(pose.x - self.last.x, pose.y - self.last.y)
        self.last = pose
        self.speeds.append(pose.speed)

        # Cone hits: a cone inside the car's footprint.
        if len(self.cones):
            body = world_to_body(self.cones, pose)
            inside = np.flatnonzero((np.abs(body[:, 0]) < CAR_HALF_LENGTH)
                                    & (np.abs(body[:, 1]) < CAR_HALF_WIDTH))
            for i in inside:
                if i not in self.hit:
                    self.hit.add(i)
                    self.hits_total += 1
                    self.say(f"CONE HIT at ({self.cones[i, 0]:.1f}, {self.cones[i, 1]:.1f})")

        # Lap timing: passing near the start/finish gate.
        if self.gate is None:
            return
        near = math.hypot(pose.x - self.gate[0], pose.y - self.gate[1]) < self.get_parameter("gate_radius").value
        if not near:
            return
        if self.lap_start is None:
            if pose.speed > 0.5:   # the car starts on the line: start timing when it moves
                self.say("Started lap timing")
                self.new_lap(pose)
        elif self.dist_since_gate > self.get_parameter("min_lap_distance").value:
            t = pose.stamp - self.lap_start
            v = np.array(self.speeds)
            self.say(f"LAP {self.lap}   {t:.2f} s   avg {v.mean():.1f} m/s   "
                     f"max {v.max():.1f} m/s   cones hit: {len(self.hit)}"
                     + (f"   pose error max {max(self.pose_err):.2f} m" if self.pose_err else ""))
            self.new_lap(pose)

    def new_lap(self, pose: Pose2D):
        self.lap += 1
        self.lap_start = pose.stamp
        self.dist_since_gate = 0.0
        self.speeds = []
        self.hit = set()
        self.pose_err = []


def main():
    run(Monitor)


if __name__ == "__main__":
    main()
