import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = "vla_kinova_jtc_streamer"
EXECUTABLE = "jtc_stream_node"
CONFIG = "jtc_stream_node.yaml"

# The yaml is the source of truth; each arg defaults to "" and only overrides the yaml
# when the user passes it on the CLI.
_BOOL = lambda v: v.lower() in ("true", "1", "yes")
_OVERRIDABLE = [
    ("tick_hz", float),
    ("jtc_horizon", float),
    ("control_hz", float),
    ("plan_topic", str),
    ("arm_trajectory_topic", str),
]


def _robot_yaml(context):
    # launch_ros only warns about a missing params file, so fail loudly on a bad robot:=
    robot = LaunchConfiguration("robot").perform(context)
    path = os.path.join(get_package_share_directory("vela_robots"), "config", robot, "robot.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"robot:={robot} has no profile ({path})")
    return path


def launch_setup(context, *args, **kwargs):
    config = os.path.join(get_package_share_directory(PKG), "config", CONFIG)
    overrides = {}
    for name, cast in _OVERRIDABLE:
        val = LaunchConfiguration(name).perform(context)
        if val != "":
            overrides[name] = cast(val)
    overrides["use_sim_time"] = _BOOL(LaunchConfiguration("use_sim").perform(context))
    return [Node(package=PKG, executable=EXECUTABLE, name=EXECUTABLE,
                 output="screen", parameters=[config, _robot_yaml(context), overrides])]


def generate_launch_description():
    def arg(name, desc):
        return DeclareLaunchArgument(name, default_value="",
                                     description=f"{desc} (empty => value from {CONFIG})")
    return LaunchDescription([
        arg("tick_hz", "Oversample/stream rate (Hz)"),
        arg("jtc_horizon", "JTC point time_from_start (s); latency<->smoothness knob"),
        arg("control_hz", "Fallback plan knot spacing (Hz) when the plan has <2 points"),
        arg("plan_topic", "30 Hz plan topic from the Python VLA client"),
        DeclareLaunchArgument("arm_trajectory_topic", default_value="",
                              description="JTC trajectory topic to stream to (empty => value from robot.yaml)"),
        DeclareLaunchArgument("robot", default_value="kinova_gen3_6dof",
                              description="Robot profile: vela_robots/config/<robot>/robot.yaml"),
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="true => sim clock; false => system clock (real robot)"),
        OpaqueFunction(function=launch_setup),
    ])
