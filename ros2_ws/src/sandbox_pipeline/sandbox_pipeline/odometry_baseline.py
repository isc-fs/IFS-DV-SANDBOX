"""ODOMETRY baseline: wheel speed + gyro -> where the car moved (dead reckoning).

  in : /motor_rpm      Float32, motor RPM        -> forward speed = rpm * 0.00727
       /imu            sensor_msgs/Imu           -> yaw rate (angular_velocity.z)
       /testing_only/odom  ONLY its first message: the start pose (see below)
  out: /sandbox/odom   nav_msgs/Odometry, frame odom -> base_link, ~100 Hz
       TF odom -> base_link

Each IMU message:
    yaw += yaw_rate * dt
    x   += speed * cos(yaw) * dt
    y   += speed * sin(yaw) * dt
While the car stands still at the start, the gyro bias (what it reads
when not turning) is averaged and then subtracted, but only if there were
at least `min_bias_samples` samples: a bias averaged from a few noisy
samples is worse than no correction at all (it adds a constant yaw drift).

Odometry is smooth and fast, but it DRIFTS: every small error in speed or
yaw rate is integrated forever. Correcting that drift with the cones is
SLAM's job (it publishes map -> odom).

The start pose: the odom frame starts where the car starts, expressed in
the map (= world) frame. That's the only ground truth this node reads,
and it's equivalent to "the car knows it's on the start line". On a
real-car bag there's no ground truth, so it starts at (0, 0, 0).

Weak spots: no wheel slip model, the RPM -> speed factor is fixed, no
lateral velocity, no filtering, the bias is only estimated once.

Real pipeline equivalent: IFS09-DV-PIPELINE/odometry_filter (C++ EKF:
IMU prediction, motor RPM + steering corrections).
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Header
from tf2_ros import TransformBroadcaster

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.geometry import Pose2D, quat_from_yaw, stamp_to_sec


class OdometryBaseline(Node):
    def __init__(self):
        super().__init__("odometry")
        self.declare_parameter("rpm_to_ms", contract.RPM_TO_MS)
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("start_pose_timeout", 2.0)   # s to wait for the start pose
        self.declare_parameter("min_bias_samples", 200)     # ~2 s of IMU while standing still

        self.speed = 0.0
        self.pose: Pose2D | None = None        # None until we know where we start
        self.first_imu_t = None
        self.last_t = None
        self.last_pub = -1e9
        self.bias_sum, self.bias_n, self.bias = 0.0, 0, 0.0
        self.moving = False

        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, contract.ODOM, 10)
        self.create_subscription(Float32, contract.MOTOR_RPM, self.on_rpm, contract.SENSOR_QOS)
        self.create_subscription(Imu, contract.IMU, self.on_imu, contract.SENSOR_QOS)
        self.gt_sub = self.create_subscription(Odometry, contract.GT_ODOM, self.on_start_pose, contract.SENSOR_QOS)

    def on_start_pose(self, msg: Odometry):
        if self.pose is None:
            p = Pose2D.from_odometry(msg)
            self.pose = Pose2D(p.x, p.y, p.yaw, stamp=p.stamp)
            self.get_logger().info(f"start pose ({p.x:.2f}, {p.y:.2f}, {math.degrees(p.yaw):.1f} deg)")
        self.destroy_subscription(self.gt_sub)          # never look at ground truth again

    def on_rpm(self, msg: Float32):
        self.speed = msg.data * self.get_parameter("rpm_to_ms").value
        self.moving = self.moving or abs(self.speed) > 0.1

    def on_imu(self, msg: Imu):
        t = stamp_to_sec(msg.header.stamp)
        if self.first_imu_t is None:
            self.first_imu_t = t
        if self.pose is None:
            if t - self.first_imu_t < self.get_parameter("start_pose_timeout").value:
                return
            self.pose = Pose2D(stamp=t)                 # no ground truth (real bag): start at 0
            self.get_logger().info("no start pose available: starting at (0, 0, 0)")

        wz = msg.angular_velocity.z
        if not self.moving:                             # standing still: learn the gyro bias
            self.bias_sum += wz
            self.bias_n += 1
            if self.bias_n >= self.get_parameter("min_bias_samples").value:
                self.bias = self.bias_sum / self.bias_n

        if self.last_t is not None:
            dt = t - self.last_t
            if 0.0 < dt < 0.5:                          # skip only absurd gaps
                p = self.pose
                p.yaw_rate = wz - self.bias
                p.yaw = math.atan2(math.sin(p.yaw + p.yaw_rate * dt), math.cos(p.yaw + p.yaw_rate * dt))
                p.x += self.speed * math.cos(p.yaw) * dt
                p.y += self.speed * math.sin(p.yaw) * dt
                p.vx = self.speed
        self.last_t = t
        self.pose.stamp = t
        if t - self.last_pub >= 1.0 / self.get_parameter("publish_hz").value:
            self.last_pub = t
            self.publish(msg.header.stamp)

    def publish(self, stamp):
        p = self.pose
        header = Header(frame_id=contract.ODOM_FRAME, stamp=stamp)
        q = quat_from_yaw(p.yaw)
        odom = Odometry(header=header, child_frame_id=contract.BASE_FRAME)
        odom.pose.pose.position.x, odom.pose.pose.position.y = p.x, p.y
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        odom.twist.twist.linear.x = p.vx
        odom.twist.twist.angular.z = p.yaw_rate
        self.pub.publish(odom)
        tf = TransformStamped(header=header, child_frame_id=contract.BASE_FRAME)
        tf.transform.translation.x, tf.transform.translation.y = p.x, p.y
        r = tf.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.tf.sendTransform(tf)


def main():
    run(OdometryBaseline)


if __name__ == "__main__":
    main()
