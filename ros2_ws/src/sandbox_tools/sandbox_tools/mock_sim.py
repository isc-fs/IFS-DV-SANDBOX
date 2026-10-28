"""A tiny 2D simulator that speaks the same topics as the IFSSIM bridge.

Use it when you can't (or don't want to) run the real simulator: on a
laptop without a GPU, on a train, or to test something in 10 seconds.

    sb-launch sim:=mock                         # default track
    sb-launch sim:=mock track:=skidpad.csv      # any CSV in tracks/

What it fakes:
  * the car: kinematic bicycle, throttle/brake -> acceleration, steering lag
  * /lidar_points: ray-cast against flat ground + cones (116 channels, 120 deg FOV)
  * /imu, /motor_rpm, /steering_angle, /brake_pressure, /testing_only/odom,
    /testing_only/track, /clock, sensor TFs

It is NOT physics. Tyres never slip, the LiDAR is perfect geometry plus a
bit of noise. Anything that works here must still be checked in IFSSIM.
"""
from __future__ import annotations

import math
import os

import numpy as np
import rclpy
from fs_msgs.msg import Cone as FsCone
from fs_msgs.msg import ControlCommand, Track
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32, Header
from tf2_ros import StaticTransformBroadcaster

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import quat_from_yaw
from sandbox_tools.pointcloud import make_cloud

COLOR_NAMES = {"yellow": FsCone.YELLOW, "blue": FsCone.BLUE,
               "big_orange": FsCone.ORANGE_BIG, "orange_big": FsCone.ORANGE_BIG,
               "small_orange": FsCone.ORANGE_SMALL, "orange": FsCone.ORANGE_SMALL}

WHEELBASE = 1.627
MAX_STEER = math.radians(22.4)   # same as IFSSIM settings.json MaxSteerAngle
LIDAR_HEIGHT = 1.10
RPM_TO_MS = 0.00727              # same constant the real pipeline's odometry uses


def load_track_csv(path: str):
    """CSV rows: colour,x,y,... -> (xy (N,2), colors (N,))."""
    xy, colors = [], []
    with open(path) as f:
        for line in f:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 3 or parts[0] not in COLOR_NAMES:
                continue
            xy.append((float(parts[1]), float(parts[2])))
            colors.append(COLOR_NAMES[parts[0]])
    return np.array(xy), np.array(colors, dtype=np.uint8)


def start_pose(xy: np.ndarray, colors: np.ndarray):
    """Put the car 2 m behind the orange gate, facing along the track."""
    gate = xy[colors == FsCone.ORANGE_BIG]
    center = gate.mean(axis=0) if len(gate) else np.zeros(2)
    blue, yellow = xy[colors == FsCone.BLUE], xy[colors == FsCone.YELLOW]
    b = blue[np.argmin(np.linalg.norm(blue - center, axis=1))]
    y = yellow[np.argmin(np.linalg.norm(yellow - center, axis=1))]
    left = (b - y) / np.linalg.norm(b - y)          # points from right side to left side
    forward = np.array([left[1], -left[0]])         # left rotated -90 deg
    pos = center - 2.0 * forward
    return pos[0], pos[1], math.atan2(forward[1], forward[0])


class MockSim(Node):
    def __init__(self):
        super().__init__("mock_sim")
        self.declare_parameter("track", "track_20260404_013721.csv")
        self.declare_parameter("tracks_dir", os.path.join(os.environ.get("SANDBOX_ROOT", "/sandbox"), "tracks"))
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("lidar_hz", 10.0)
        self.declare_parameter("lidar_range", 30.0)
        self.declare_parameter("lidar_azimuth_step_deg", 0.1)
        self.declare_parameter("lidar_noise", 0.03)
        self.declare_parameter("max_accel", 8.0)
        self.declare_parameter("max_decel", 12.0)

        track = self.get_parameter("track").value
        path = track if os.path.isabs(track) else os.path.join(self.get_parameter("tracks_dir").value, track)
        self.cones_xy, self.cones_color = load_track_csv(path)
        self.x, self.y, self.yaw = start_pose(self.cones_xy, self.cones_color)
        self.v = 0.0
        self.steer = 0.0
        self.accel = 0.0
        self.yaw_rate = 0.0
        self.cmd = ControlCommand()
        self.cmd_time = -1e9
        self.t = 0.0
        self.rng = np.random.default_rng(0)
        self.get_logger().info(f"Mock sim on {path} ({len(self.cones_xy)} cones)")

        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_odom = self.create_publisher(Odometry, contract.GT_ODOM, contract.SENSOR_QOS)
        self.pub_imu = self.create_publisher(Imu, contract.IMU, contract.SENSOR_QOS)
        self.pub_lidar = self.create_publisher(PointCloud2, contract.LIDAR, contract.SENSOR_QOS)
        # Same as the IFSSIM bridge: a copy for the visualiser (here the cloud is small anyway).
        self.pub_lidar_viz = self.create_publisher(PointCloud2, contract.LIDAR + "/viz", contract.SENSOR_QOS)
        self.pub_track = self.create_publisher(Track, contract.GT_TRACK, contract.LATCHED_QOS)
        self.pub_rpm = self.create_publisher(Float32, "/motor_rpm", contract.SENSOR_QOS)
        self.pub_steer = self.create_publisher(Float32, "/steering_angle", contract.SENSOR_QOS)
        self.pub_brake = self.create_publisher(Float32, "/brake_pressure", contract.SENSOR_QOS)
        self.create_subscription(ControlCommand, contract.CONTROL, self.on_cmd, 10)

        self.static_tf = StaticTransformBroadcaster(self)
        tfs = []
        for child in (contract.LIDAR_FRAME, "imu_link"):
            tf = TransformStamped()
            tf.header.frame_id = contract.BASE_FRAME
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            tfs.append(tf)
        self.static_tf.sendTransform(tfs)

        self.dt = 1.0 / self.get_parameter("rate_hz").value
        self.lidar_every = max(1, round(self.get_parameter("rate_hz").value / self.get_parameter("lidar_hz").value))
        self.tick_count = 0
        self._build_rays()
        self.publish_track()
        self.create_timer(5.0, self.publish_track)
        self.create_timer(self.dt, self.step)

    # ------------------------------------------------------------------
    def on_cmd(self, msg: ControlCommand):
        self.cmd = msg
        self.cmd_time = self.t

    def header(self, frame: str) -> Header:
        return Header(frame_id=frame, stamp=Time(nanoseconds=int(self.t * 1e9)).to_msg())

    def publish_track(self):
        msg = Track()
        for (x, y), c in zip(self.cones_xy, self.cones_color):
            cone = FsCone(color=int(c))
            cone.location.x, cone.location.y = float(x), float(y)
            msg.track.append(cone)
        self.pub_track.publish(msg)

    # ------------------------------------------------------------------
    def step(self):
        dt = self.dt
        self.t += dt
        stale = self.t - self.cmd_time > 0.5
        throttle = 0.0 if stale else float(np.clip(self.cmd.throttle, 0.0, 1.0))
        brake = 0.3 if stale else float(np.clip(self.cmd.brake, 0.0, 1.0))
        # The command is + = right, like IFSSIM. self.steer is + = left (y is left).
        steer_cmd = 0.0 if stale else -float(np.clip(self.cmd.steering, -1.0, 1.0))

        # Steering: first-order lag towards the command.
        self.steer += (steer_cmd * MAX_STEER - self.steer) * min(1.0, dt / 0.08)
        # Longitudinal: motor, regen brake, drag, rolling resistance.
        a = (throttle * self.get_parameter("max_accel").value
             - brake * self.get_parameter("max_decel").value
             - 0.02 * self.v * self.v - (0.3 if self.v > 0.05 else 0.0))
        self.v = max(0.0, self.v + a * dt)
        self.accel = a if self.v > 0.0 else 0.0
        self.yaw_rate = self.v * math.tan(self.steer) / WHEELBASE
        self.yaw += self.yaw_rate * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        self.pub_clock.publish(Clock(clock=Time(nanoseconds=int(self.t * 1e9)).to_msg()))
        self.publish_odom()
        self.publish_imu()
        self.pub_rpm.publish(Float32(data=float(self.v / RPM_TO_MS)))
        self.pub_steer.publish(Float32(data=-float(self.steer)))   # + = right, like IFSSIM
        self.pub_brake.publish(Float32(data=float(brake)))
        self.tick_count += 1
        if self.tick_count % self.lidar_every == 0:
            self.publish_lidar()

    def publish_odom(self):
        msg = Odometry(header=self.header("odom"), child_frame_id="fsds/FSCar")
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        q = quat_from_yaw(self.yaw)
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.yaw_rate
        self.pub_odom.publish(msg)

    def publish_imu(self):
        msg = Imu(header=self.header("imu_link"))
        q = quat_from_yaw(self.yaw)
        o = msg.orientation
        o.x, o.y, o.z, o.w = q
        n = self.rng.normal
        msg.angular_velocity.z = self.yaw_rate + n(0, 0.0035)
        msg.linear_acceleration.x = self.accel + n(0, 0.024)
        msg.linear_acceleration.y = self.v * self.yaw_rate + n(0, 0.024)
        msg.linear_acceleration.z = 9.81 + n(0, 0.024)
        self.pub_imu.publish(msg)

    # ------------------------------------------------------------------
    def _build_rays(self):
        step = self.get_parameter("lidar_azimuth_step_deg").value
        self.az = np.radians(np.arange(-60.0, 60.0 + 1e-9, step))      # (A,)
        self.el = np.radians(np.linspace(-12.4, 5.9, 116))              # (C,)
        tan_el = np.tan(self.el)
        with np.errstate(divide="ignore"):
            ground = np.where(tan_el < 0, LIDAR_HEIGHT / -tan_el, np.inf)  # horizontal range
        self.ground_rho = np.repeat(ground[:, None], len(self.az), axis=1)  # (C, A)
        self.tan_el = tan_el

    def publish_lidar(self):
        max_r = self.get_parameter("lidar_range").value
        rho = self.ground_rho.copy()
        inten = np.full(rho.shape, 0.08, dtype=np.float32)

        c, s = math.cos(self.yaw), math.sin(self.yaw)
        d = self.cones_xy - (self.x, self.y)
        bx, by = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
        dist = np.hypot(bx, by)
        az_c = np.arctan2(by, bx)
        step = self.az[1] - self.az[0]
        for i in np.flatnonzero((dist < max_r) & (dist > 0.3) & (np.abs(az_c) < math.radians(62))):
            big = self.cones_color[i] == FsCone.ORANGE_BIG
            r0, height = (0.143, 0.505) if big else (0.114, 0.325)
            half = math.asin(min(1.0, r0 / dist[i]))
            j0 = max(0, int(np.floor((az_c[i] - half - self.az[0]) / step)))
            j1 = min(len(self.az), int(np.ceil((az_c[i] + half - self.az[0]) / step)) + 1)
            if j1 <= j0:
                continue
            daz = self.az[j0:j1] - az_c[i]
            b = np.broadcast_to(dist[i] * np.sin(daz), (len(self.el), j1 - j0))  # lateral miss distance
            along = dist[i] * np.cos(daz)
            # Cone surface: radius shrinks linearly with height. Two fixed-point
            # steps: guess the hit distance, get the height there, use that radius.
            hit_rho = np.broadcast_to(along - np.sqrt(np.maximum(r0 * r0 - b * b, 0.0)), b.shape)
            for _ in range(2):
                z_hit = LIDAR_HEIGHT + hit_rho * self.tan_el[:, None]   # height above ground
                r = r0 * np.clip(1.0 - z_hit / height, 0.0, 1.0)
                hit_rho = along - np.sqrt(np.maximum(r * r - b * b, 0.0))
            z_hit = LIDAR_HEIGHT + hit_rho * self.tan_el[:, None]
            r = r0 * np.clip(1.0 - z_hit / height, 0.0, 1.0)
            ok = (np.abs(b) < r) & (z_hit >= 0.0) & (z_hit <= height)
            sub = rho[:, j0:j1]
            closer = ok & (hit_rho < sub)
            sub[closer] = hit_rho[closer]
            col = self.cones_color[i]
            val = 0.2 if col == FsCone.BLUE else 0.55 if col == FsCone.YELLOW else 0.65
            inten[:, j0:j1][closer] = val

        valid = rho < max_r
        keep = valid & (self.rng.random(rho.shape) > 0.01)            # 1 % dropout
        # Intensity model of IFSSIM: reflectance * (R_ref / range)^2 with R_ref = 5 m.
        inten = inten * np.minimum(1.0, (5.0 / np.maximum(rho, 1e-3)) ** 2)
        r = rho[keep] + self.rng.normal(0.0, self.get_parameter("lidar_noise").value, keep.sum())
        az = np.broadcast_to(self.az[None, :], rho.shape)[keep]
        tz = np.broadcast_to(self.tan_el[:, None], rho.shape)[keep]
        cloud = make_cloud(self.header(contract.LIDAR_FRAME), {
            "x": r * np.cos(az), "y": r * np.sin(az), "z": r * tz, "intensity": inten[keep],
        })
        self.pub_lidar.publish(cloud)
        self.pub_lidar_viz.publish(cloud)


def main():
    run(MockSim)


if __name__ == "__main__":
    main()
