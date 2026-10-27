"""Scores perception (and SLAM) on REAL CAR data replayed from a bag.

A real bag has no ground truth: nobody knows where the cones really were.
So instead of "correct" we measure:
  * timing: does your node keep up with real 174 k-point scans?
    (latency from the scan ARRIVING to your cones, and % of scans you
    answered; the Hesai stamps a scan at its start, ~1 sweep before it
    arrives, so "now - stamp" would add ~115 ms nobody can remove)
  * agreement with the real pipeline on the same data: of the cones the
    real pipeline detected, how many did you detect too (and vice versa),
    and how close is your trajectory to the real SLAM's.
Agreement isn't truth: the real pipeline makes mistakes too. Look at
your detections in Lichtblick as well.

The reference (the real pipeline's detections + trajectory on this bag) is
in benchmark/reference/<scenario>-outputs.json. With `record:=true` this
node writes that file instead (heads, via tools/benchmark_real.sh).

Started by tools/benchmark.py, which also plays the bag.
"""
from __future__ import annotations

import bisect
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sandbox_msgs.msg import ConeArray
from sensor_msgs.msg import PointCloud2

from sandbox_tools import contract, scoring
from sandbox_tools.geometry import Pose2D, cones_to_numpy, stamp_to_sec

AGREE_RANGE = 15.0   # compare detections closer than this
AGREE_DIST = 0.5     # same cone if closer than this


class BagScorer(Node):
    def __init__(self):
        super().__init__("bag_scorer")
        num = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("output", "/tmp/score.json")
        self.declare_parameter("reference", "")
        self.declare_parameter("record", False)
        self.declare_parameter("label", "me")
        self.declare_parameter("scenario", "real-bag")
        self.declare_parameter("config", "{}")
        self.declare_parameter("idle_timeout", 8.0, num)    # real s without scans = bag finished
        self.declare_parameter("wall_timeout", 900.0, num)

        self.scans: list[float] = []
        self.arrival: dict[float, float] = {}   # scan stamp -> sim time it arrived
        self.dets: dict[float, np.ndarray] = {}   # scan stamp -> (N,2) detections
        self.latency: list[float] = []
        self.poses: list[tuple[float, float, float]] = []
        self.odom_poses: list[tuple[float, float, float]] = []
        self.last_scan_wall = None
        self.create_subscription(PointCloud2, contract.LIDAR, self.on_scan, contract.SENSOR_QOS)
        self.create_subscription(ConeArray, contract.CONES, self.on_cones, 10)
        self.create_subscription(Odometry, contract.POSE, self.on_pose, 10)
        self.create_subscription(Odometry, contract.ODOM, self.on_odom, 10)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, msg: PointCloud2):
        t = stamp_to_sec(msg.header.stamp)
        self.scans.append(t)
        self.arrival[round(t, 3)] = self.now()
        self.last_scan_wall = time.monotonic()

    def on_cones(self, msg: ConeArray):
        t = stamp_to_sec(msg.header.stamp)
        arrived = self.arrival.get(round(t, 3))
        if arrived is not None:
            self.latency.append(max(0.0, self.now() - arrived))
        xy, _, _ = cones_to_numpy(msg)
        xy = xy[np.hypot(xy[:, 0], xy[:, 1]) < AGREE_RANGE] if len(xy) else xy
        self.dets[round(t, 3)] = xy

    def on_pose(self, msg: Odometry):
        p = Pose2D.from_odometry(msg)
        if not self.poses or p.stamp - self.poses[-1][0] >= 0.1:
            self.poses.append((p.stamp, p.x, p.y))

    def on_odom(self, msg: Odometry):
        p = Pose2D.from_odometry(msg)
        if not self.odom_poses or p.stamp - self.odom_poses[-1][0] >= 0.1:
            self.odom_poses.append((p.stamp, p.x, p.y))

    @staticmethod
    def traj_diff(mine, ref_poses):
        """Mean distance between two trajectories at matching times."""
        if not ref_poses or not mine:
            return None
        rt = [p[0] for p in ref_poses]
        d = []
        for t, x, y in mine:
            i = min(bisect.bisect_left(rt, t), len(rt) - 1)
            if i > 0 and abs(rt[i - 1] - t) < abs(rt[i] - t):   # nearest sample, not the next one
                i -= 1
            if abs(rt[i] - t) < 0.06:
                d.append(math.hypot(x - ref_poses[i][1], y - ref_poses[i][2]))
        return float(np.mean(d)) if d else None

    # ----------------------------------------------------------------
    def agreement(self, ref: dict):
        """Detections vs the reference detections, scan by scan."""
        ref_dets = {round(float(k), 3): np.array(v).reshape(-1, 2) for k, v in ref.get("detections", {}).items()}
        tp = n_mine = n_ref = 0
        for t, ref_xy in ref_dets.items():
            mine = self.dets.get(t)
            if mine is None:
                n_ref += len(ref_xy)          # scan we didn't answer: everything missed
                continue
            tp += len(scoring.match(mine, ref_xy, AGREE_DIST))
            n_mine += len(mine)
            n_ref += len(ref_xy)
        recall = tp / n_ref if n_ref else None
        precision = tp / n_mine if n_mine else None
        traj = self.traj_diff(self.poses, ref.get("poses", []))
        odom = self.traj_diff(self.odom_poses, ref.get("odom_poses", []))
        return recall, precision, traj, odom

    def results(self) -> dict:
        n_scans = len(set(round(s, 3) for s in self.scans))
        answered = len(self.dets)
        lat95 = float(np.percentile(self.latency, 95)) if self.latency else None
        ref = {}
        path = self.get_parameter("reference").value
        if path and os.path.exists(path):
            with open(path) as f:
                ref = json.load(f)
        recall, precision, traj, odom = self.agreement(ref) if ref else (None, None, None, None)
        processed = answered / n_scans if n_scans else 0.0

        p_score = scoring.bag_perception_score(recall, precision, lat95, processed)
        s_score = None if traj is None or not self.poses else round(100 * scoring.clamp01(1 - traj / 2.0), 1)

        return {
            "label": self.get_parameter("label").value,
            "scenario": self.get_parameter("scenario").value,
            "config": json.loads(self.get_parameter("config").value),
            "date": datetime.now().isoformat(timespec="seconds"),
            "status": "finished" if answered else "dnf: no cones published",
            "laps_required": 0, "lap_times_s": [], "cones_hit": None, "time_s": None,
            "perception": {
                "scans": n_scans, "processed_fraction": round(processed, 3),
                "detections_per_scan": round(float(np.mean([len(v) for v in self.dets.values()])), 1) if self.dets else None,
                "latency_p50_ms": round(1000 * float(np.median(self.latency)), 1) if self.latency else None,
                "latency_p95_ms": None if lat95 is None else round(1000 * lat95, 1),
                "latency_max_ms": round(1000 * max(self.latency), 1) if self.latency else None,
                "agreement_recall": None if recall is None else round(recall, 3),
                "agreement_precision": None if precision is None else round(precision, 3),
                "score": p_score,
            },
            "odometry": {"trajectory_agreement_m": None if odom is None else round(odom, 3),
                         "poses": len(self.odom_poses),
                         "score": None if odom is None else round(100 * scoring.clamp01(1 - odom / 2.0), 1)},
            "slam": {"trajectory_agreement_m": None if traj is None else round(traj, 3),
                     "poses": len(self.poses), "score": s_score},
            "planning": {"score": None}, "control": {"score": None},
        }

    def reference_dump(self) -> dict:
        return {"detections": {f"{t:.3f}": v.round(3).tolist() for t, v in self.dets.items()},
                "poses": [[round(t, 3), round(x, 3), round(y, 3)] for t, x, y in self.poses],
                "odom_poses": [[round(t, 3), round(x, 3), round(y, 3)] for t, x, y in self.odom_poses]}


def main():
    rclpy.init()
    node = BagScorer()
    start = time.monotonic()
    deadline = start + float(node.get_parameter("wall_timeout").value)
    idle = float(node.get_parameter("idle_timeout").value)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            waited = time.monotonic() - (node.last_scan_wall or start)
            if waited > (idle if node.last_scan_wall else 60.0):
                break
    except KeyboardInterrupt:
        pass
    out = node.get_parameter("output").value
    if node.get_parameter("record").value:
        os.makedirs(os.path.dirname(node.get_parameter("reference").value), exist_ok=True)
        with open(node.get_parameter("reference").value, "w") as f:
            json.dump(node.reference_dump(), f, separators=(",", ":"))
        print("reference written:", node.get_parameter("reference").value)
    res = node.results()
    with open(out, "w") as f:
        json.dump(res, f, indent=2)
    print(json.dumps({"scenario": res["scenario"], "status": res["status"], "perception": res["perception"]}))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
