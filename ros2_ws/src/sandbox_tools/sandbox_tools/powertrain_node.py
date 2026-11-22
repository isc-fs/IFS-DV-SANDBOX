"""POWERTRAIN: /control_command -> /motor_command (torque of the 2 rear motors).

  in : /control_command   fs_msgs/ControlCommand   (what your control node already publishes)
       /motor_rpm         std_msgs/Float32         speed, to fade the regen out when slow
  out: /motor_command     sandbox_msgs/MotorCommand   left / right torque, [-0.2, 1]

Off by default: the car drives with /control_command alone, as always. Opt in:
  sb-launch powertrain:=true               # brake -> regen (max 20 % of the forward torque)
  sb-launch powertrain:=true ediff:=true   # + electronic differential between the wheels
  ros2 param set /powertrain ediff true    # or switch it while running

To command each motor yourself, leave this node off and publish /motor_command
(plus /control_command for steering and the friction brake): your own torque
vectoring, your own regen.
"""
from __future__ import annotations

import math

import rclpy
from fs_msgs.msg import ControlCommand
from rclpy.node import Node
from sandbox_msgs.msg import MotorCommand
from std_msgs.msg import Float32

from sandbox_tools import contract
from sandbox_tools.node_utils import run
from sandbox_tools.powertrain import split_torque


class Powertrain(Node):
    def __init__(self):
        super().__init__("powertrain")
        self.declare_parameter("ediff", False)
        self.declare_parameter("wheelbase", 1.627)       # m
        self.declare_parameter("track", 1.2)             # m, rear track width (check with the car!)
        self.declare_parameter("max_steer_deg", 22.4)    # must match the sim / car
        self.declare_parameter("regen_fade_speed", 2.0)  # m/s
        self.speed = 0.0
        self.create_subscription(Float32, "/motor_rpm", self.on_rpm, contract.SENSOR_QOS)
        self.create_subscription(ControlCommand, contract.CONTROL, self.on_cmd, 10)
        self.pub = self.create_publisher(MotorCommand, contract.MOTOR_COMMAND, 10)

    def on_rpm(self, msg: Float32):
        self.speed = max(0.0, msg.data * contract.RPM_TO_MS)

    def on_cmd(self, msg: ControlCommand):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        left, right = split_torque(
            msg.throttle, msg.brake, msg.steering, self.speed,
            ediff=p("ediff"), wheelbase=p("wheelbase"), track=p("track"),
            max_steer=math.radians(p("max_steer_deg")), regen_fade_speed=p("regen_fade_speed"))
        self.pub.publish(MotorCommand(header=msg.header, left=left, right=right))


def main():
    run(Powertrain)


if __name__ == "__main__":
    main()
