"""The topic contract: the only thing every node has to agree on.

Inside a node you can do whatever you want. Between nodes, use these
topics, types and frames, and any stage can be swapped for any other
implementation (yours, a teammate's, ground truth, C++...).

See docs/TOPICS.md for the explanation of each one.
"""
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# --- Frames -----------------------------------------------------------
MAP_FRAME = "map"            # fixed world frame (= the sim's "odom" frame)
ODOM_FRAME = "odom"          # odometry frame: smooth but drifts (map -> odom is SLAM's correction)
BASE_FRAME = "base_link"     # the car
LIDAR_FRAME = "hesai_lidar"  # LiDAR, 1.10 m above the ground. Ground at z = -1.10
                             # on the mock, z = 0 on IFSSIM: find it in the scan

# --- Coming from the simulator (or the mock sim) ----------------------
LIDAR = "/lidar_points"              # sensor_msgs/PointCloud2  ~10 Hz, frame hesai_lidar
IMU = "/imu"                         # sensor_msgs/Imu          ~400 Hz, frame imu_link
GT_ODOM = "/testing_only/odom"       # nav_msgs/Odometry        ground-truth pose + twist
GT_TRACK = "/testing_only/track"     # fs_msgs/Track            ground-truth cones (world frame)

# --- The sandbox pipeline ---------------------------------------------
LIDAR_SKEWED = "/lidar_points_skewed"  # PointCloud2 + per-point "t" field (deskew challenge)
MOTOR_RPM = "/motor_rpm"             # std_msgs/Float32          motor RPM (speed = rpm * RPM_TO_MS)
RPM_TO_MS = 0.00727                  # same constant as the real pipeline
ODOM = "/sandbox/odom"               # nav_msgs/Odometry        dead reckoning, frame odom -> base_link
CONES = "/sandbox/cones"             # sandbox_msgs/ConeArray   detections, frame base_link
POSE = "/sandbox/pose"               # nav_msgs/Odometry        car pose, frame map -> base_link (+ TF map -> odom)
MAP = "/sandbox/map"                 # sandbox_msgs/ConeArray   cone map, frame map
PATH = "/sandbox/path"               # nav_msgs/Path            path to follow, frame map
CONTROL = "/control_command"         # fs_msgs/ControlCommand   to the car

# --- QoS ----------------------------------------------------------------
# Sensor streams: drop old data rather than queue it.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
# The GT track is published rarely and "latched".
LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)
# Everything else (our own topics) uses the default: reliable, depth 10.
