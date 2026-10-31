"""Start the sandbox: simulator link + one node per pipeline stage + tools.

    sb-launch                                   # IFSSIM + baseline everything
    sb-launch sim:=mock                         # no Unreal needed
    sb-launch perception:=gt                    # skip perception, use ground-truth cones
    sb-launch odometry:=gt                      # perfect odometry (no drift)
    sb-launch perception:=ifs09                 # the real pipeline's cone detection
    sb-launch control:=off                      # drive yourself: ros2 run sandbox_tools teleop
    sb-launch planning:=alice_pipeline/planning # use your own node
    sb-launch params:=/sandbox/my_params.yaml   # override parameters

Stage values:
    baseline   the simple node in sandbox_pipeline (default)
    ifs09      the real IFS09-DV-PIPELINE code for that stage (perception only, for now)
    gt         ground truth (perception and odometry only)
    off        don't start this stage
    pkg/exec   any executable from any package (yours!)
    exec       an executable from sandbox_pipeline

Whatever runs as a stage gets the node name of the stage (odometry,
perception, slam, planning, control), so `ros2 param set /control target_speed 6.0`
works no matter whose code it is.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STAGES = ("odometry", "perception", "slam", "planning", "control")
IFS09_STAGES = ("perception",)   # stages with a <stage>_ifs09 node in sandbox_pipeline


def resolve(stage: str, value: str):
    """Return (package, executable, node_name) or None."""
    if value == "off":
        return None
    if value == "baseline":
        return "sandbox_pipeline", f"{stage}_baseline", stage
    if value == "ifs09":
        if stage not in IFS09_STAGES:
            raise RuntimeError(f"{stage}:=ifs09 does not exist yet (only "
                               + ", ".join(f"{s}:=ifs09" for s in IFS09_STAGES) + ").")
        return "sandbox_pipeline", f"{stage}_ifs09", stage
    if value == "gt":
        if stage == "perception":
            return "sandbox_tools", "gt_cones", "gt_cones"
        if stage == "odometry":
            return "sandbox_tools", "gt_odom", "gt_odom"
        raise RuntimeError(f"{stage}:=gt does not exist (only perception:=gt and odometry:=gt). "
                           "For a perfect SLAM pose use the slam baseline with pose_source: gt.")
    if "/" in value:
        pkg, exe = value.split("/", 1)
        return pkg, exe, stage
    return "sandbox_pipeline", value, stage


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    params_file = arg("params")
    common = [{"use_sim_time": True}]
    if params_file:
        common.append(params_file)

    nodes = []
    sim = arg("sim")
    if sim == "ifssim":
        nodes.append(Node(
            package="ifssim_bridge", executable="ifssim_bridge", name="ifssim_bridge",
            output="screen",
            parameters=[{
                "host": os.environ.get("IFSSIM_HOST", "127.0.0.1"),
                "port": int(os.environ.get("IFSSIM_PORT", "41451")),
                "competition_mode": False,          # keeps the /testing_only/* ground truth
                "lidar_viz_decimation": 4,          # light /lidar_points/viz for Lichtblick
            }],
        ))
    elif sim == "mock":
        nodes.append(Node(
            package="sandbox_tools", executable="mock_sim", name="mock_sim", output="screen",
            parameters=[{"track": arg("track")}] + ([params_file] if params_file else []),
        ))
    elif sim != "none":
        raise RuntimeError(f"sim:={sim} unknown (ifssim | mock | none)")

    for stage in STAGES:
        r = resolve(stage, arg(stage))
        if r is None:
            continue
        pkg, exe, name = r
        nodes.append(Node(package=pkg, executable=exe, name=name, output="screen",
                          parameters=common, emulate_tty=True))

    if arg("real") == "true":
        # The real IFS09 pipeline runs in its own container (tools/benchmark_real.sh);
        # this only republishes its outputs on the sandbox contract for scoring.
        nodes.append(Node(package="sandbox_tools", executable="real_adapter", name="real_adapter",
                          output="screen", parameters=common))

    if arg("skew") == "true":
        nodes.append(Node(package="sandbox_tools", executable="lidar_skewer", name="lidar_skewer",
                          output="screen", parameters=common))

    nodes.append(Node(package="sandbox_tools", executable="visualizer", name="visualizer",
                      parameters=common))
    nodes.append(Node(package="sandbox_tools", executable="monitor", name="monitor",
                      output="screen", parameters=common))

    if arg("foxglove") == "true":
        nodes.append(Node(
            package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
            parameters=[{"port": 8765, "address": "0.0.0.0", "use_sim_time": True,
                         "send_buffer_limit": 64 * 1024 * 1024}],
        ))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument("sim", default_value="ifssim", description="ifssim | mock | none (bag replay)"),
        DeclareLaunchArgument("track", default_value="track_20260404_013721.csv",
                              description="track CSV for sim:=mock (file in tracks/)"),
        DeclareLaunchArgument("skew", default_value="false", description="publish /lidar_points_skewed"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("real", default_value="false",
                              description="republish the real IFS09 pipeline's outputs (benchmark reference)"),
        DeclareLaunchArgument("params", default_value="", description="optional params YAML"),
    ] + [DeclareLaunchArgument(s, default_value="baseline") for s in STAGES]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
