"""Drive the car with the keyboard (publishes /control_command).

    ros2 run sandbox_tools teleop                 # in its own terminal
    ros2 run sandbox_tools teleop --max-throttle 0.3

  W / S   throttle / brake      A / D   steer left / right
  SPACE   stop                  Q       quit

Launch the sandbox with control:=off first, otherwise the control node
and you fight over the car.
"""
import argparse
import sys
import termios
import tty

import rclpy
from fs_msgs.msg import ControlCommand

STEP = 0.1
DECAY = 0.85   # steering returns to centre when no key is pressed


def get_key(fd, old):
    tty.setraw(fd)
    ch = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-throttle", type=float, default=0.5)
    parser.add_argument("--max-steer", type=float, default=1.0)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = rclpy.create_node("teleop")
    pub = node.create_publisher(ControlCommand, "/control_command", 10)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    throttle = steer = brake = 0.0
    print(__doc__)
    try:
        while rclpy.ok():
            key = get_key(fd, old).lower()
            if key == "q":
                break
            if key == "w":
                throttle, brake = min(1.0, throttle + STEP), 0.0
            elif key == "s":
                throttle, brake = 0.0, min(1.0, brake + STEP)
            elif key == "a":
                steer = min(1.0, steer + STEP)    # positive steering = left
            elif key == "d":
                steer = max(-1.0, steer - STEP)
            elif key == " ":
                throttle, steer, brake = 0.0, 0.0, 1.0
            else:
                steer *= DECAY
                throttle = max(0.0, throttle - 0.02)
                brake = 0.0
            msg = ControlCommand()
            msg.throttle = float(min(throttle, args.max_throttle))
            msg.steering = float(max(-args.max_steer, min(args.max_steer, steer)))
            msg.brake = float(brake)
            pub.publish(msg)
            sys.stdout.write(f"\rthrottle {msg.throttle:.2f}  steer {msg.steering:+.2f}  brake {msg.brake:.2f}   ")
            sys.stdout.flush()
    finally:
        pub.publish(ControlCommand(throttle=0.0, steering=0.0, brake=1.0))
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\nStopped.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
