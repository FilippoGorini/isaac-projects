"""Launch the lerobot_ros dataset_recorder with the robot profile's TOML as default.

Convenience wrapper so you don't have to type the full
`ros2 run lerobot_ros dataset_recorder --ros-args -p config:=$(ros2 pkg prefix
vla_robots)/share/vla_robots/config/kinova_gen3_6dof/recorder.toml` every time. Also sets
HF_HUB_OFFLINE=1 so reopening an existing dataset doesn't try to fetch it
from the Hugging Face Hub.

The recorder needs the lerobot_ros venv (bootstrap_lerobot_ros.sh puts it in
src/lerobot_ros/.venv and rewrites the entry-point shebang to its python). A full
`colcon build` regenerates the entry point with #!/usr/bin/python3 ("No module named
lerobot"), so the recorder is started with the venv python explicitly when it exists.

Usage:
    ros2 launch vla_data_collection lerobot_recorder.launch.py
    ros2 launch vla_data_collection lerobot_recorder.launch.py config:=<other.toml>
"""

import os

import toml
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _venv_python():
    # <ws>/install/lerobot_ros -> <ws>/src/lerobot_ros/.venv/bin/python
    ws = os.path.dirname(os.path.dirname(get_package_prefix("lerobot_ros")))
    path = os.path.join(ws, "src", "lerobot_ros", ".venv", "bin", "python")
    return path if os.path.isfile(path) else None


def _check_joints(context):
    # The clients read state/actions by the profile's joint order, so a recorder config that
    # differs (missing joint, other order) would silently record an unusable dataset.
    robot = LaunchConfiguration("robot").perform(context)
    config = LaunchConfiguration("config").perform(context)
    with open(os.path.join(get_package_share_directory("vla_robots"), "config", robot, "robot.yaml")) as f:
        profile = yaml.safe_load(f)["/**"]["ros__parameters"]
    expected = profile["arm_joints"] + [profile["gripper_joint"]]
    # lerobot_ros concatenates the JointState topics of each tag in TOML order
    columns = {"observation": [], "action": []}
    for topic in toml.load(config)["topics"].values():
        tag = topic.get("tag", "observation")
        if topic.get("msg_type") == "JointState" and tag in columns:
            columns[tag] += topic["joints"]
    for tag, joints in columns.items():
        if joints != expected:
            raise RuntimeError(f"{config}: {tag} joints {joints} != arm_joints + gripper_joint "
                               f"{expected} of robot:={robot}")
    return []


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("vla_robots"),
        "config",
        LaunchConfiguration("robot"),
        "recorder.toml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="kinova_gen3_6dof",
            description="Robot profile: default config is vla_robots/config/<robot>/recorder.toml",
        ),
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the lerobot_ros recorder TOML config.",
        ),
        OpaqueFunction(function=_check_joints),
        SetEnvironmentVariable("HF_HUB_OFFLINE", "1"),
        Node(
            package="lerobot_ros",
            executable="dataset_recorder",
            output="screen",
            prefix=_venv_python(),   # None: fall back to the entry point's own shebang
            parameters=[{"config": LaunchConfiguration("config")}],
        ),
    ])
