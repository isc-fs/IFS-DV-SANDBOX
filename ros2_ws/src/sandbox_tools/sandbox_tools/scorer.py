"""Benchmark scorer: watches one run and scores every stage against ground truth.

Started by tools/benchmark.py (`sb-score`), not by hand. It listens to the
contract topics, so it scores whatever runs as each stage: the baseline,
your node, ground truth, or the real pipeline (through real_adapter).

The run ends when the car completes `laps` laps, or it's a DNF (car
stopped, left the track, or `timeout` sim seconds passed). Then it writes
one JSON file with all metrics and scores and exits.
"""
from __future__ import annotations

import bisect
import json
import math
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from fs_msgs.msg import ControlCommand, Track
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sandbox_msgs.msg import Cone, ConeArray

from sandbox_tools import contract, scoring
from sandbox_tools.geometry import Pose2D, cones_to_numpy, stamp_to_sec, world_to_body, wrap_angle

CAR_HALF_LENGTH, CAR_HALF_WIDTH = 1.5, 0.75
FOV_DEG = 60.0
SEEN_RANGE = 15.0        # a GT cone counts as "seen" (map recall) once this close and in FOV
RECALL_RANGE = 15.0      # perception recall is measured on cones closer than this
DETECT_RANGE = 22.0      # detections are matched against GT cones up to this range


class Scorer(Node):
    def __init__(self):
        super().__init__("scorer")
        num = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("laps", 1)
        self.declare_parameter("timeout", 400.0, num)       # sim seconds
        self.declare_parameter("wall_timeout", 900.0, num)  # real seconds
        self.declare_parameter("output", "/tmp/score.json")
        self.declare_parameter("label", "me")
        self.declare_parameter("scenario", "")
        self.declare_parameter("config", "{}")              # JSON: stage -> what ran
        self.declare_parameter("start_timeout", 120.0, num)  # real seconds to wait for the car to move

        # ground truth
        self.gt_t: list[float] = []
        self.gt_pose: list[Pose2D] = []
        self.track_xy = np.empty((0, 2))
        self.track_col = np.empty(0, dtype=np.uint8)
        self.blue = self.yellow = np.empty((0, 2))
        self.gate = None
        self.seen = set()

        # race state
        self.t_start = None
        self.lap_start = None
        self.laps: list[float] = []
        self.dist_since_gate = 0.0
        self.last_xy = None
        self.hit = set()
        self.hits = 0
        self.stopped_since = None
        self.off_since = None
        self.status = None
        self.speeds = []

        # stage metrics accumulators
        self.det_tp = self.det_n = self.gt_near_n = self.gt_near_tp = 0
        self.det_err, self.det_color_ok, self.det_color_n, self.latency = [], 0, 0, []
        self.pose_err, self.yaw_err = [], []
        self.odom_err, self.odom_yaw_err = [], []
        self.total_distance = 0.0
        self._last_odom_eval = -1e9
        self.last_map = None
        self.path_off, self.path_out, self.path_len = [], [], []
        self.cte = []
        self.steer_prev = None
        self.steer_rates = []
        self._last_pose_eval = self._last_path_eval = self._last_cte_eval = -1e9

        self.create_subscription(Track, contract.GT_TRACK, self.on_track, contract.LATCHED_QOS)
        self.create_subscription(Odometry, contract.GT_ODOM, self.on_gt, contract.SENSOR_QOS)
        self.create_subscription(ConeArray, contract.CONES, self.on_cones, 10)
        self.create_subscription(Odometry, contract.POSE, self.on_pose, 10)
        self.create_subscription(Odometry, contract.ODOM, self.on_odom, 10)
        self.create_subscription(ConeArray, contract.MAP, lambda m: setattr(self, "last_map", m), 10)
        self.create_subscription(Path, contract.PATH, self.on_path, 10)
        self.create_subscription(ControlCommand, contract.CONTROL, self.on_cmd, 10)

    # ------------------------------------------------------------ helpers
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def gt_at(self, t: float) -> Pose2D | None:
        if not self.gt_t:
            return None
        i = bisect.bisect_left(self.gt_t, t)
        if i == 0:
            return self.gt_pose[0]
        if i >= len(self.gt_t):
            return self.gt_pose[-1]
        a, b = self.gt_pose[i - 1], self.gt_pose[i]
        f = (t - a.stamp) / max(b.stamp - a.stamp, 1e-9)
        return Pose2D(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
                      a.yaw + f * float(wrap_angle(b.yaw - a.yaw)), stamp=t)

    def visible(self, pose: Pose2D, max_range: float):
        body = world_to_body(self.track_xy, pose)
        r = np.hypot(body[:, 0], body[:, 1])
        ok = (r < max_range) & (np.abs(np.degrees(np.arctan2(body[:, 1], body[:, 0]))) < FOV_DEG)
        return body, r, ok

    # ------------------------------------------------------------ ground truth
    def on_track(self, msg: Track):
        self.track_xy = np.array([(c.location.x, c.location.y) for c in msg.track]).reshape(-1, 2)
        self.track_col = np.array([c.color for c in msg.track], dtype=np.uint8)
        self.blue = self.track_xy[self.track_col == Cone.BLUE]
        self.yellow = self.track_xy[self.track_col == Cone.YELLOW]
        gate = self.track_xy[self.track_col == Cone.ORANGE_BIG]
        self.gate = gate.mean(axis=0) if len(gate) else None

    def on_gt(self, msg: Odometry):
        pose = Pose2D.from_odometry(msg)
        if self.gt_t and pose.stamp <= self.gt_t[-1]:
            return
        self.gt_t.append(pose.stamp)
        self.gt_pose.append(pose)
        if len(self.track_xy) == 0 or self.status:
            return
        t = pose.stamp
        if self.last_xy is not None:
            step = math.hypot(pose.x - self.last_xy[0], pose.y - self.last_xy[1])
            self.dist_since_gate += step
            if self.t_start is not None:
                self.total_distance += step
        self.last_xy = (pose.x, pose.y)

        _, _, ok = self.visible(pose, SEEN_RANGE)
        self.seen.update(np.flatnonzero(ok).tolist())

        # cone hits
        body = world_to_body(self.track_xy, pose)
        for i in np.flatnonzero((np.abs(body[:, 0]) < CAR_HALF_LENGTH) & (np.abs(body[:, 1]) < CAR_HALF_WIDTH)):
            if i not in self.hit:
                self.hit.add(int(i))
                self.hits += 1

        # laps
        near = self.gate is not None and math.hypot(pose.x - self.gate[0], pose.y - self.gate[1]) < 3.0
        if self.t_start is None:
            if near and pose.speed > 0.5:
                self.t_start = self.lap_start = t
                self.dist_since_gate = 0.0
            return
        if near and self.dist_since_gate > 30.0:
            self.laps.append(round(t - self.lap_start, 2))
            self.get_logger().info(f"lap {len(self.laps)}: {self.laps[-1]:.2f} s")
            self.lap_start = t
            self.dist_since_gate = 0.0
            self.hit = set()
            if len(self.laps) >= self.get_parameter("laps").value:
                self.status = "finished"
                return

        # tracking + DNF checks (10 Hz is plenty)
        self.speeds.append(pose.speed)
        if t - self._last_cte_eval >= 0.1:
            self._last_cte_eval = t
            off = float(scoring.centre_offset([(pose.x, pose.y)], self.blue, self.yellow)[0])
            self.cte.append(off)
            self.off_since = (self.off_since or t) if off > 3.0 else None
            if self.off_since and t - self.off_since > 2.0:
                self.status = "dnf: left the track"
        self.stopped_since = (self.stopped_since or t) if pose.speed < 0.3 else None
        if self.stopped_since and t - self.stopped_since > 10.0:
            self.status = "dnf: car stopped"
        if t - self.t_start > float(self.get_parameter("timeout").value):
            self.status = "dnf: timeout"

    # ------------------------------------------------------------ stages
    def on_cones(self, msg: ConeArray):
        if len(self.track_xy) == 0 or self.t_start is None or self.status:
            return
        t = stamp_to_sec(msg.header.stamp)
        pose = self.gt_at(t)
        if pose is None:
            return
        self.latency.append(max(0.0, self.now() - t))
        xy, colors, _ = cones_to_numpy(msg)
        body, r, ok = self.visible(pose, DETECT_RANGE)
        idx = np.flatnonzero(ok)
        pairs = scoring.match(xy, body[idx], 0.5)
        self.det_n += len(xy)
        self.det_tp += len(pairs)
        near = set(idx[r[idx] < RECALL_RANGE].tolist())
        self.gt_near_n += len(near)
        for i, j, d in pairs:
            gi = int(idx[j])
            self.det_err.append(d)
            if gi in near:
                self.gt_near_tp += 1
            if colors[i] != Cone.UNKNOWN:
                self.det_color_n += 1
                self.det_color_ok += int(colors[i] == self.track_col[gi])

    def on_pose(self, msg: Odometry):
        if self.t_start is None or self.status:
            return
        p = Pose2D.from_odometry(msg)
        if p.stamp - self._last_pose_eval < 0.05:
            return
        self._last_pose_eval = p.stamp
        gt = self.gt_at(p.stamp)
        if gt is None:
            return
        self.pose_err.append(math.hypot(p.x - gt.x, p.y - gt.y))
        self.yaw_err.append(abs(float(wrap_angle(p.yaw - gt.yaw))))

    def on_odom(self, msg: Odometry):
        """Odometry vs ground truth. The odom frame starts at the start pose in
        world coordinates, so before any SLAM correction it IS the world frame."""
        if self.t_start is None or self.status:
            return
        p = Pose2D.from_odometry(msg)
        if p.stamp - self._last_odom_eval < 0.05:
            return
        self._last_odom_eval = p.stamp
        gt = self.gt_at(p.stamp)
        if gt is None:
            return
        self.odom_err.append(math.hypot(p.x - gt.x, p.y - gt.y))
        self.odom_yaw_err.append(abs(float(wrap_angle(p.yaw - gt.yaw))))

    def on_path(self, msg: Path):
        if self.t_start is None or self.status or len(self.blue) == 0:
            return
        t = stamp_to_sec(msg.header.stamp)
        if t - self._last_path_eval < 0.2:
            return
        self._last_path_eval = t
        xy = np.array([(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]).reshape(-1, 2)
        if len(xy) < 2:
            self.path_len.append(0.0)
            return
        self.path_len.append(float(min(15.0, np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))))
        pts = scoring.resample_path(xy, 1.0, 15.0)
        off = scoring.centre_offset(pts, self.blue, self.yellow)
        self.path_off.extend(off.tolist())
        self.path_out.extend((off > 1.2).tolist())

    def on_cmd(self, msg: ControlCommand):
        if self.t_start is None or self.status:
            return
        t = self.now()
        if self.steer_prev is not None and t > self.steer_prev[0]:
            self.steer_rates.append((msg.steering - self.steer_prev[1]) / (t - self.steer_prev[0]))
        self.steer_prev = (t, msg.steering)

    # ------------------------------------------------------------ report
    def results(self) -> dict:
        mean = lambda v: float(np.mean(v)) if len(v) else None  # noqa: E731
        r3 = lambda v: None if v is None else round(v, 3)  # noqa: E731

        # perception
        recall = self.gt_near_tp / self.gt_near_n if self.gt_near_n else None
        precision = self.det_tp / self.det_n if self.det_n else None
        pos_err = mean(self.det_err)
        lat95 = float(np.percentile(self.latency, 95)) if self.latency else None
        perception = {
            "recall": r3(recall), "precision": r3(precision), "position_error_m": r3(pos_err),
            "color_accuracy": r3(self.det_color_ok / self.det_color_n) if self.det_color_n else None,
            "latency_p95_ms": None if lat95 is None else round(1000 * lat95, 1),
            "score": scoring.perception_score(recall, precision or 0.0, pos_err if pos_err is not None else 0.3,
                                              lat95 if lat95 is not None else 1.0),
        }

        # odometry
        drift = (100 * max(self.odom_err) / self.total_distance
                 if self.odom_err and self.total_distance > 10 else None)
        odometry = {
            "error_mean_m": r3(mean(self.odom_err)),
            "error_max_m": r3(max(self.odom_err) if self.odom_err else None),
            "yaw_error_mean_deg": None if not self.odom_yaw_err else round(math.degrees(mean(self.odom_yaw_err)), 2),
            "distance_m": round(self.total_distance, 1),
            "drift_percent": r3(drift),
            "score": scoring.odometry_score(drift),
        }

        # slam
        map_recall = map_precision = map_err = None
        if self.last_map is not None and len(self.track_xy):
            mxy, _, _ = cones_to_numpy(self.last_map)
            seen = np.array(sorted(self.seen), dtype=int)
            pairs = scoring.match(mxy, self.track_xy, 1.0)
            matched_gt = {j for _, j, _ in pairs}
            map_precision = len(pairs) / len(mxy) if len(mxy) else 0.0
            map_recall = len(matched_gt & set(seen.tolist())) / len(seen) if len(seen) else None
            map_err = mean([d for _, _, d in pairs])
        pe = mean(self.pose_err)
        slam = {
            "pose_error_mean_m": r3(pe), "pose_error_max_m": r3(max(self.pose_err) if self.pose_err else None),
            "yaw_error_mean_deg": None if not self.yaw_err else round(math.degrees(mean(self.yaw_err)), 2),
            "map_recall": r3(map_recall), "map_precision": r3(map_precision), "map_error_m": r3(map_err),
            "score": scoring.slam_score(pe, map_recall, map_precision, map_err),
        }

        # planning
        po = mean(self.path_off)
        planning = {
            "centre_offset_mean_m": r3(po),
            "outside_fraction": r3(mean([float(x) for x in self.path_out])),
            "length_mean_m": r3(mean(self.path_len)),
            "score": scoring.planning_score(po, mean([float(x) for x in self.path_out]) or 0.0,
                                            mean(self.path_len) or 0.0),
        }

        # control
        cte = mean(self.cte)
        srms = float(np.sqrt(np.mean(np.square(self.steer_rates)))) if self.steer_rates else 0.0
        control = {
            "centre_offset_mean_m": r3(cte), "centre_offset_max_m": r3(max(self.cte) if self.cte else None),
            "steer_rate_rms": round(srms, 3), "cones_hit": self.hits,
            "speed_mean_ms": r3(mean(self.speeds)),
            "score": scoring.control_score(cte, self.hits, srms),
        }

        laps_req = self.get_parameter("laps").value
        return {
            "label": self.get_parameter("label").value,
            "scenario": self.get_parameter("scenario").value,
            "config": json.loads(self.get_parameter("config").value),
            "date": datetime.now().isoformat(timespec="seconds"),
            "status": self.status or "dnf: never started",
            "laps_required": laps_req,
            "lap_times_s": self.laps,
            "cones_hit": self.hits,
            "time_s": scoring.corrected_time(self.laps, self.hits, laps_req),
            "odometry": odometry, "perception": perception, "slam": slam,
            "planning": planning, "control": control,
        }


def main():
    rclpy.init()
    node = Scorer()
    deadline = time.monotonic() + float(node.get_parameter("wall_timeout").value)
    started_wall = time.monotonic()
    try:
        while node.status is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.t_start is None and time.monotonic() - started_wall > float(node.get_parameter("start_timeout").value):
                node.status = "dnf: car never crossed the start line"
    except KeyboardInterrupt:
        node.status = node.status or "dnf: interrupted"
    if node.status is None:
        node.status = "dnf: wall-clock timeout"
    res = node.results()
    out = node.get_parameter("output").value
    with open(out, "w") as f:
        json.dump(res, f, indent=2)
    print(json.dumps({k: res[k] for k in ("scenario", "status", "lap_times_s", "cones_hit", "time_s")}))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
