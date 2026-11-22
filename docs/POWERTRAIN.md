# Two rear motors (optional)

Next year's car has **two motors, one per rear wheel**, with regenerative
braking limited to **20 % of the forward torque**. The sandbox lets you try
that control interface. **It is optional and off by default:** if you do
nothing, the car drives exactly as before (`throttle`, `brake`, `steering`
on `/control_command`).

It only works on the mock sim (`sim:=mock`). IFSSIM has one drivetrain.

## Three ways to drive

| | What you run | What you publish | What you get |
|---|---|---|---|
| **1. As before** (default) | `sb-launch sim:=mock` | `/control_command` | Throttle on both motors, friction brake, no regen |
| **2. Simple two-motor mode** | `sb-launch sim:=mock powertrain:=true` | `/control_command` | The `powertrain` node splits it into the two motors. `brake` gives regen first (max 20 % of forward torque, fades out below 2 m/s) |
| **2b. + electronic differential** | `sb-launch sim:=mock powertrain:=true ediff:=true` | `/control_command` | Also less torque on the inner wheel in a turn and more on the outer one (same total) |
| **3. Granular** | `sb-launch sim:=mock` | `/control_command` **and** `/motor_command` | You command each motor yourself |

`ediff:=true` without `powertrain:=true` is an error. Change the e-diff while running:
`ros2 param set /powertrain ediff true`.

The same flags work in the benchmark: `sb-score powertrain:=true`,
`sb-score powertrain:=true ediff:=true` (mock scenarios only). Runs get
`+regen` / `+ediff` in the Control cell of the leaderboard, so they are
separate rows from the default.

## Granular control: `/motor_command`

`sandbox_msgs/MotorCommand`: `left` and `right`, normalised to the motor's
maximum forward torque.

```
 1.0  full forward torque
 0.0  coast
-0.2  full regen   (regen is limited to 20 % of the forward torque)
```

You still publish `/control_command`: **steering** and the friction
**brake** come from it. While your `/motor_command` arrives, the `throttle`
in `/control_command` is ignored.

In your control node (copy of `templates/my_pipeline/.../control.py`):

```python
from sandbox_msgs.msg import MotorCommand           # new import

# in __init__:
self.pub_motor = self.create_publisher(MotorCommand, contract.MOTOR_COMMAND, 10)

# in tick(), next to the ControlCommand (compute() gives + = left steering):
base = float(throttle)
bias = 0.15 * float(steering)          # a bit more torque on the outer wheel
self.pub_motor.publish(MotorCommand(left=base - bias, right=base + bias))
```

The mock prints a one-time warning that the `throttle` in `/control_command`
is ignored. That is expected here, and it goes away if you publish
`throttle=0` there.

For a left turn (`steering > 0`) the right wheel is the outer one, so it gets
more torque; that bias is a very simple torque vectoring. Replace it with
your own: a yaw-rate controller, regen on corner entry, per-wheel traction
limits...

The maths of the built-in e-diff is also available as a function:
`sandbox_tools.powertrain.split_torque(throttle, brake, steering, speed, ediff=True)`.

## Things that go wrong (and what tells you)

| Mistake | What happens |
|---|---|
| `powertrain:=true` **and** your own `/motor_command` publisher | They fight over the topic. The `powertrain` node warns |
| Torque outside `[-0.2, 1]` | Clipped. The mock sim warns once |
| `/motor_command` stops for more than 0.5 s | The mock goes back to the `/control_command` throttle on both motors. It warns once |
| Your `/motor_command` is active and `/control_command` has a throttle | The throttle is ignored. The mock sim warns once |
| `powertrain:=true` with `sim:=ifssim` (or `sb-score` on an IFSSIM scenario) | Error at launch |
| Your own `/motor_command` on IFSSIM | Ignored. The monitor warns |
| `ediff:=true` without `powertrain:=true` | Error at launch |

## The mock sim's model, and numbers to confirm

Each motor pushes its own wheel, so a torque difference turns the car (a yaw
moment, damped by the tyres). `/motor_rpm_left` and `/motor_rpm_right` show
the wheel speeds: the inner wheel runs slower in a turn.

It is not real physics. These values are **placeholders**, not measured on
the car. They are parameters, so check them against the real car:

| Value | Where | Placeholder |
|---|---|---|
| Rear track width | `track_width` on `/powertrain` and `/mock_sim` (set both) | 1.2 m |
| Car mass | `mass` on `/mock_sim` | 230 kg |
| Yaw inertia | `yaw_inertia` on `/mock_sim` | 110 kg m² |
| Yaw damping | `yaw_damping_tau` on `/mock_sim` | 0.15 s |
| Regen fade-out speed | `regen_fade_speed` on `/powertrain` | 2 m/s |
| Regen limit | `REGEN_LIMIT` in `sandbox_tools/powertrain.py` | 20 % (team spec) |

On the baseline control, regen and the e-diff change the lap time by only
about 0.02 s on the mock. They matter when you use them on purpose.
