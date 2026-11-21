"""Two rear motors, one per wheel: torque split, regen and electronic differential.

Pure maths, no ROS, so it is easy to test and to reuse inside your own node.

Torque is normalised to the motor's maximum forward torque:
    +1.0 full drive, 0.0 coast, -REGEN_LIMIT full regen.

Regen is limited to 20 % of the forward torque, so the lowest value a motor
can be asked for is -0.2.

Electronic differential: in a turn the inner rear wheel travels a shorter
path than the outer one, so it spins slower. The two motors are independent
(no shared axle), so equal torques behave like an open differential: no
yaw moment, the car turns only with the front wheels. The e-diff goes one
step further and splits the torque in proportion to the speed each wheel
has to run at, so the outer wheel pushes a bit more and helps the car
rotate (a mild torque vectoring):

    R = wheelbase / tan(delta)            (turn radius of the rear axle centre)
    k = tan(delta) * track / (2 * wheelbase)
    inner = T * (1 - k)      outer = T * (1 + k)       (the sum stays 2 T)
"""
from __future__ import annotations

import math

REGEN_LIMIT = 0.2          # max regen torque, as a fraction of max forward torque
TORQUE_MIN, TORQUE_MAX = -REGEN_LIMIT, 1.0


def clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def ediff_factors(steer_left: float, wheelbase: float, track: float) -> tuple[float, float]:
    """(left, right) torque factors for a steer angle in rad, + = left (like y)."""
    k = math.tan(steer_left) * track / (2.0 * wheelbase)
    k = clip(k, -0.9, 0.9)      # never take a wheel's torque all the way to zero or flip it
    return 1.0 - k, 1.0 + k     # left turn: left is the inner wheel -> less torque


def split_torque(throttle: float, brake: float, steering: float, speed: float, *,
                 ediff: bool = True, wheelbase: float = 1.627, track: float = 1.2,
                 max_steer: float = math.radians(22.4), regen_fade_speed: float = 2.0
                 ) -> tuple[float, float]:
    """ControlCommand-style inputs -> (left, right) motor torque.

    throttle [0,1], brake [0,1], steering [-1,1] (+ = RIGHT, like /control_command),
    speed in m/s (>= 0). Braking cancels the throttle; the regen comes first and
    the friction brake of the car (ControlCommand.brake) stays on top of it.
    Regen fades out below `regen_fade_speed` (a slow motor has no back-EMF to harvest).
    """
    throttle, brake = clip(throttle, 0.0, 1.0), clip(brake, 0.0, 1.0)
    if brake > 0.0:
        fade = clip(speed / regen_fade_speed, 0.0, 1.0)
        base = -REGEN_LIMIT * brake * fade
    else:
        base = throttle
    left_f, right_f = 1.0, 1.0
    if ediff:
        left_f, right_f = ediff_factors(-clip(steering, -1.0, 1.0) * max_steer, wheelbase, track)
    return (clip(base * left_f, TORQUE_MIN, TORQUE_MAX),
            clip(base * right_f, TORQUE_MIN, TORQUE_MAX))
