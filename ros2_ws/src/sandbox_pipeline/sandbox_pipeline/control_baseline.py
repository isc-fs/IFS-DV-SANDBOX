"""CONTROL baseline: path + pose -> throttle, steering, brake.

  in : /sandbox/path      nav_msgs/Path (map)
       /sandbox/pose      Odometry (map -> base_link), uses twist.linear.x as speed
  out: /control_command   fs_msgs/ControlCommand at 40 Hz
                          throttle [0,1], brake [0,1], steering [-1,1] (+ = RIGHT)

Two independent controllers (this split is exactly what the real team
wants to unify this season):
  lateral       Pure Pursuit: aim at the path point `lookahead` metres away,
                steer along the circle arc that reaches it
                  kappa = 2 sin(alpha) / Ld,   delta = atan(L kappa)
  longitudinal  P controller towards ONE constant target speed. It does not
                slow down for corners at all.

Safety: if the path or the pose is older than `timeout` seconds, brake.

Real pipeline equivalent: IFS09-DV-PIPELINE/control (pure pursuit / stanley
+ PI speed controller with a curvature-based speed profile).
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, stamp_to_sec, world_to_body


class ControlBaseline(Node):
    def __init__(self):
        super().__init__("control")
        self.declare_parameter("rate_hz", 40.0)
        self.declare_parameter("target_speed", 5.0)     # m/s
        self.declare_parameter("kp_speed", 0.3)
        self.declare_parameter("lookahead_min", 3.0)    # m
        self.declare_parameter("lookahead_gain", 0.3)   # s  (Ld = min + gain * v)
        self.declare_parameter("wheelbase", 1.627)      # m
        self.declare_parameter("max_steer_deg", 22.4)   # must match the sim / car
        self.declare_parameter("timeout", 0.5)          # s

        self.path: Path | None = None
        self.pose: Odometry | None = None
        self.create_subscription(Path, contract.PATH, lambda m: setattr(self, "path", m), 10)
        self.create_subscription(Odometry, contract.POSE, lambda m: setattr(self, "pose", m), 10)
        self.pub = self.create_publisher(ControlCommand, contract.CONTROL, 10)
        self.pub_target = self.create_publisher(PointStamped, "/sandbox/control/lookahead", 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def tick(self):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        now = self.get_clock().now().nanoseconds * 1e-9
        stale = (self.path is None or self.pose is None or len(self.path.poses) < 2
                 or now - stamp_to_sec(self.pose.header.stamp) > p("timeout")
                 or now - stamp_to_sec(self.path.header.stamp) > p("timeout"))
        if stale:
            self.pub.publish(ControlCommand(throttle=0.0, steering=0.0, brake=1.0))
            return

        pose = Pose2D.from_odometry(self.pose)
        pts = np.array([(ps.pose.position.x, ps.pose.position.y) for ps in self.path.poses])
        body = world_to_body(pts, pose)

        # ---- lateral: pure pursuit ----
        ld = p("lookahead_min") + p("lookahead_gain") * pose.vx
        target = self.point_at_distance(body, ld)
        alpha = math.atan2(target[1], target[0])
        dist = max(float(np.hypot(*target)), 0.1)
        kappa = 2.0 * math.sin(alpha) / dist
        delta = math.atan(p("wheelbase") * kappa)
        steering = float(np.clip(delta / math.radians(p("max_steer_deg")), -1.0, 1.0))

        # ---- longitudinal: P on speed ----
        err = p("target_speed") - pose.vx
        throttle = float(np.clip(p("kp_speed") * err, 0.0, 1.0))
        brake = float(np.clip(-p("kp_speed") * err, 0.0, 1.0)) if err < -0.5 else 0.0

        # /control_command uses the car's convention, + = RIGHT (like IFSSIM and
        # IFS09-DV-PIPELINE). Pure pursuit gives + = left (y is left): flip it.
        self.pub.publish(ControlCommand(throttle=throttle, steering=-steering, brake=brake))
        pt = PointStamped()
        pt.header.frame_id = contract.BASE_FRAME
        pt.header.stamp = self.pose.header.stamp
        pt.point.x, pt.point.y = float(target[0]), float(target[1])
        self.pub_target.publish(pt)

    @staticmethod
    def point_at_distance(path_body: np.ndarray, ld: float) -> np.ndarray:
        """First point on the polyline that is `ld` metres from the car (or the last point)."""
        d = np.hypot(path_body[:, 0], path_body[:, 1])
        for i in range(1, len(path_body)):
            if d[i] >= ld and path_body[i, 0] > 0.0:
                # interpolate between point i-1 and i to land exactly at ld
                a, b = path_body[i - 1], path_body[i]
                lo, hi = 0.0, 1.0
                for _ in range(20):
                    mid = (lo + hi) / 2
                    if np.hypot(*(a + mid * (b - a))) < ld:
                        lo = mid
                    else:
                        hi = mid
                return a + hi * (b - a)
        return path_body[-1]


def main():
    run(ControlBaseline)


if __name__ == "__main__":
    main()
