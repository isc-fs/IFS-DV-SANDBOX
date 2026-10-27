"""Smoke test: does the whole loop work? Used by CI and by `sb-check`.

Watches a running sandbox for `duration` seconds of SIM time, then checks:
  * every contract topic was published (odom, cones, pose, map, path, control)
  * the car really drove (distance travelled, from ground truth)
  * no cones were hit (counted by the monitor)
Prints a report and exits with 0 (pass) or 1 (fail).

Run it through tools/smoke_test.sh, which also starts the pipeline.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from fs_msgs.msg import ControlCommand
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sandbox_msgs.msg import ConeArray
from std_msgs.msg import String

from sandbox_tools import contract

WATCHED = {
    contract.ODOM: Odometry,
    contract.CONES: ConeArray,
    contract.POSE: Odometry,
    contract.MAP: ConeArray,
    contract.PATH: Path,
    contract.CONTROL: ControlCommand,
}


class SmokeCheck(Node):
    def __init__(self):
        super().__init__("smoke_check")
        num = ParameterDescriptor(dynamic_typing=True)  # accept 40 as well as 40.0
        self.declare_parameter("duration", 40.0, num)        # sim seconds to watch
        self.declare_parameter("wall_timeout", 240.0, num)   # give up after this (real seconds)
        self.declare_parameter("min_distance", 80.0, num)    # metres the car must drive
        self.declare_parameter("max_cone_hits", 0)

        self.counts = {t: 0 for t in WATCHED}
        for topic, typ in WATCHED.items():
            self.create_subscription(typ, topic, lambda _m, t=topic: self.count(t), 10)
        self.create_subscription(Odometry, contract.GT_ODOM, self.on_odom, contract.SENSOR_QOS)
        self.create_subscription(String, "/sandbox/monitor", self.on_monitor, 10)
        self.t0 = None
        self.t = None
        self.last_xy = None
        self.distance = 0.0
        self.hits = 0

    def count(self, topic):
        self.counts[topic] += 1

    def on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        self.t = t
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.distance += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def on_monitor(self, msg: String):
        if msg.data.startswith("CONE HIT"):
            self.hits += 1

    @property
    def elapsed(self) -> float:
        return 0.0 if self.t0 is None else self.t - self.t0

    def report(self) -> bool:
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        dur = max(self.elapsed, 1e-6)
        ok = True
        lines = [f"Smoke test over {self.elapsed:.1f} s of sim time", ""]
        for topic, n in self.counts.items():
            good = n > 0
            ok &= good
            lines.append(f"  [{'ok' if good else 'FAIL'}] {topic:<20} {n:5d} msgs  ({n / dur:5.1f} Hz)")
        good = self.distance >= float(p("min_distance"))
        ok &= good
        lines.append(f"  [{'ok' if good else 'FAIL'}] distance driven      {self.distance:6.1f} m  (need >= {float(p('min_distance')):.0f})")
        good = self.hits <= p("max_cone_hits")
        ok &= good
        lines.append(f"  [{'ok' if good else 'FAIL'}] cones hit            {self.hits:6d}    (max {p('max_cone_hits')})")
        good = self.elapsed >= float(p("duration")) * 0.99
        ok &= good
        if not good:
            lines.append(f"  [FAIL] only {self.elapsed:.1f} s of sim time before the wall-clock timeout")
        lines += ["", "PASS" if ok else "FAIL"]
        print("\n".join(lines), flush=True)
        return ok


def main():
    rclpy.init()
    node = SmokeCheck()
    deadline = time.monotonic() + float(node.get_parameter("wall_timeout").value)
    duration = float(node.get_parameter("duration").value)
    try:
        while time.monotonic() < deadline and node.elapsed < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = node.report()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
