# Topic contract

The only thing every node must respect. Constants for all of these are
in `sandbox_tools/contract.py` (`from sandbox_tools import contract`).

## Pipeline topics

| Topic | Type | Frame | Rate | Published by | Notes |
|---|---|---|---|---|---|
| `/sandbox/odom` | `nav_msgs/Odometry` | `odom` → `base_link` | ~100 Hz | odometry | `twist.twist.linear.x` = forward speed. Also broadcast as TF `odom → base_link`. The `odom` frame starts on the start pose |
| `/sandbox/cones` | `sandbox_msgs/ConeArray` | `base_link` | per scan (10 Hz) | perception | `header.stamp` = stamp of the LiDAR scan |
| `/sandbox/pose` | `nav_msgs/Odometry` | `map` → `base_link` | ≥ 10 Hz | slam | `twist.twist.linear.x` = forward speed (control uses it). SLAM also broadcasts TF `map → odom` (its correction) |
| `/sandbox/map` | `sandbox_msgs/ConeArray` | `map` | ~10 Hz | slam | the whole map every time, not just new cones |
| `/sandbox/path` | `nav_msgs/Path` | `map` | ~10 Hz | planning | first point = the car, ordered in driving direction |
| `/control_command` | `fs_msgs/ControlCommand` | n/a | 40 Hz | control | `throttle` [0,1], `brake` [0,1], `steering` [-1,1], **+ = right** (car convention, as in IFSSIM and IFS09). 1 = 22.4° at the wheels |

## From the simulator

| Topic | Type | Frame | Rate | Notes |
|---|---|---|---|---|
| `/lidar_points` | `sensor_msgs/PointCloud2` | `hesai_lidar` | 10 Hz | fields `x y z intensity` (float32). Sensor 1.10 m above ground, 120° FOV, ~30 m range. IFSSIM: ~1 M points/scan, mock: ~75 k |
| `/lidar_points/viz` | `sensor_msgs/PointCloud2` | `hesai_lidar` | 10 Hz | lighter copy, for Lichtblick only |
| `/imu` | `sensor_msgs/Imu` | `imu_link` | 400 Hz (mock 100) | angular velocity + linear acceleration, with noise and bias |
| `/clock` | `rosgraph_msgs/Clock` | n/a | | simulation time, all nodes use it |
| `/testing_only/odom` | `nav_msgs/Odometry` | `odom` (= `map`) | ~100 Hz | **ground truth** pose + twist. Not available on the real car |
| `/testing_only/track` | `fs_msgs/Track` | `odom` (= `map`) | latched | **ground truth** cones |

Also published: `/motor_rpm` (Float32, motor RPM; speed ≈ rpm × 0.00727 m/s),
`/steering_angle` (Float32, road-wheel angle in rad), `/brake_pressure`.
IFSSIM also has `/gss` and `/gps`. See the IFSSIM docs (REFERENCE.md).

**Real car bag** (`real-bag` scenario): the same topic names, but the LiDAR
is the real Hesai: 174 k points per scan at ~8.7 Hz, extra fields `ring`
(uint16) and `timestamp` (float64, per point). `/tf_static` says the LiDAR is
0.913 m above `base_link`, but the sensor is pitched: in the data the ground
is ~0.5 m below it near the car and rises with distance. No `/testing_only/*`
ground truth.

## Tools

| Topic | Type | From |
|---|---|---|
| `/lidar_points_skewed` | `PointCloud2` + field `t` (s, relative to stamp) | `lidar_skewer` (`skew:=true`) |
| `/sandbox/viz/{cones,map,track}` | `visualization_msgs/MarkerArray` | `visualizer` |
| `/sandbox/control/lookahead` | `geometry_msgs/PointStamped` | control baseline (optional for yours) |
| `/sandbox/monitor` | `std_msgs/String` | `monitor`: laps, cone hits |
| `/sandbox/monitor/pose_error` | `std_msgs/Float32` | `monitor`: distance between `/sandbox/pose` and ground truth (at the pose's timestamp) |

## Messages

```
# sandbox_msgs/Cone
uint8 YELLOW=0        # right side of the track
uint8 BLUE=1          # left side
uint8 ORANGE_BIG=2    # start/finish
uint8 ORANGE_SMALL=3
uint8 UNKNOWN=4
float64 x
float64 y
uint8 color
float32 confidence    # 0..1

# sandbox_msgs/ConeArray
std_msgs/Header header
Cone[] cones
```

Same colour numbers as `fs_msgs/Cone` used by the sim and the real
pipeline.

## Node names

Whatever you run as a stage gets the stage's name: `/odometry`,
`/perception`, `/slam`, `/planning`, `/control`. Ground-truth perception is
`/gt_cones`, ground-truth odometry `/gt_odom`.

TF tree: `map → odom` (SLAM), `odom → base_link` (odometry),
`base_link → hesai_lidar / imu_link` (static, from the simulator). Parameter files use these names.

## Want to add something?

You can publish any extra topic you like (debug info, intermediate
results), preferably under `/<yourname>/...`. Just don't change the
topics above: other people's nodes rely on them.
