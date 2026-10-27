"""PLANNING baseline: cone map + car pose -> path to follow.

  in : /sandbox/map    ConeArray (map)
       /sandbox/pose   Odometry  (map -> base_link)
  out: /sandbox/path   nav_msgs/Path (map), starts at the car, ~10 Hz

Steps:
  1. take the mapped cones near the car and express them in the car frame
  2. pair every blue cone with its nearest yellow cone -> midpoint
     (if only one side is visible, offset those cones half a track width inwards)
  3. chain midpoints: start at the car, repeatedly jump to the closest
     midpoint ahead that doesn't turn too sharply
  4. back to the map frame and publish

Weak spots: depends 100 % on colours being right, a single wrong cone
can bend the path, no smoothing, no look at what comes after the horizon.

Real pipeline equivalent: IFS09-DV-PIPELINE/path_planning (FaSTTUBe planner).
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, body_to_world, cones_to_numpy, quat_from_yaw, world_to_body


class PlanningBaseline(Node):
    def __init__(self):
        super().__init__("planning")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("horizon", 20.0)          # only use cones closer than this (m)
        self.declare_parameter("half_width", 1.5)        # half the track width (m)
        self.declare_parameter("max_step", 7.0)          # max distance between path points (m)
        self.declare_parameter("max_turn_deg", 60.0)     # max heading change between steps

        self.map: ConeArray | None = None
        self.pose: Odometry | None = None
        self.create_subscription(ConeArray, contract.MAP, lambda m: setattr(self, "map", m), 10)
        self.create_subscription(Odometry, contract.POSE, lambda m: setattr(self, "pose", m), 10)
        self.pub = self.create_publisher(Path, contract.PATH, 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self.tick)

    def tick(self):
        if self.map is None or self.pose is None:
            return
        pose = Pose2D.from_odometry(self.pose)
        xy, colors, _ = cones_to_numpy(self.map)
        if len(xy) == 0:
            return
        body = world_to_body(xy, pose)
        near = (np.hypot(body[:, 0], body[:, 1]) < self.get_parameter("horizon").value) & (body[:, 0] > -2.0)
        body, colors = body[near], colors[near]

        mids = self.midpoints(body[colors == Cone.BLUE], body[colors == Cone.YELLOW])
        path_body = self.chain(mids)
        self.publish(path_body, pose)

    def midpoints(self, blue: np.ndarray, yellow: np.ndarray) -> np.ndarray:
        hw = self.get_parameter("half_width").value
        if len(blue) and len(yellow):
            out = []
            for b in blue:
                d = np.linalg.norm(yellow - b, axis=1)
                j = int(np.argmin(d))
                if 2.0 < d[j] < 4.0 * hw:                    # plausible track width
                    out.append((b + yellow[j]) / 2.0)
            if out:
                return np.array(out)
        # One side only: shift those cones towards the inside of the track.
        if len(blue):
            return blue + (0.0, -hw)
        if len(yellow):
            return yellow + (0.0, hw)
        return np.empty((0, 2))

    def chain(self, mids: np.ndarray) -> np.ndarray:
        max_step = self.get_parameter("max_step").value
        max_turn = math.radians(self.get_parameter("max_turn_deg").value)
        path = [np.zeros(2)]
        heading = 0.0
        remaining = list(mids)
        while remaining:
            cur = path[-1]
            best, best_d = None, max_step
            for k, m in enumerate(remaining):
                v = m - cur
                d = float(np.hypot(*v))
                turn = abs((math.atan2(v[1], v[0]) - heading + math.pi) % (2 * math.pi) - math.pi)
                if 0.5 < d < best_d and turn < max_turn:
                    best, best_d = k, d
            if best is None:
                break
            nxt = remaining.pop(best)
            heading = math.atan2(*(nxt - cur)[::-1])
            path.append(nxt)
        return np.array(path)

    def publish(self, path_body: np.ndarray, pose: Pose2D):
        world = body_to_world(path_body, pose)
        msg = Path()
        msg.header.frame_id = contract.MAP_FRAME
        msg.header.stamp = self.pose.header.stamp
        for i, (x, y) in enumerate(world):
            j = min(i + 1, len(world) - 1)
            yaw = math.atan2(world[j][1] - y, world[j][0] - x) if j != i else pose.yaw
            ps = PoseStamped(header=msg.header)
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            o = ps.pose.orientation
            o.x, o.y, o.z, o.w = quat_from_yaw(yaw)
            msg.poses.append(ps)
        self.pub.publish(msg)


def main():
    run(PlanningBaseline)


if __name__ == "__main__":
    main()
