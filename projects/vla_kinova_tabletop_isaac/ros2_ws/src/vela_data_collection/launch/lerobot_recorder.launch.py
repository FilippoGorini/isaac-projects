"""Launch the lerobot_ros dataset_recorder with the robot profile's TOML as default.

Convenience wrapper so you don't have to type the full
`ros2 run lerobot_ros dataset_recorder --ros-args -p config:=$(ros2 pkg prefix
vela_robots)/share/vela_robots/config/kinova_gen3_6dof/recorder.toml` every time. Also sets
HF_HUB_OFFLINE=1 so reopening an existing dataset doesn't try to fetch it
from the Hugging Face Hub.

The recorder needs the lerobot_ros venv (bootstrap_lerobot_ros.sh puts it in
src/lerobot_ros/.venv and rewrites the entry-point shebang to its python). A full
`colcon build` regenerates the entry point with #!/usr/bin/python3 ("No module named
lerobot"), so the recorder is started with the venv python explicitly when it exists.

Usage:
    ros2 launch vela_data_collection lerobot_recorder.launch.py
    ros2 launch vela_data_collection lerobot_recorder.launch.py config:=<other.toml>
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _venv_python():
    # <ws>/install/lerobot_ros -> <ws>/src/lerobot_ros/.venv/bin/python
    ws = os.path.dirname(os.path.dirname(get_package_prefix("lerobot_ros")))
    path = os.path.join(ws, "src", "lerobot_ros", ".venv", "bin", "python")
    return path if os.path.isfile(path) else None


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("vela_robots"),
        "config",
        LaunchConfiguration("robot"),
        "recorder.toml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="kinova_gen3_6dof",
            description="Robot profile: default config is vela_robots/config/<robot>/recorder.toml",
        ),
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to the lerobot_ros recorder TOML config.",
        ),
        SetEnvironmentVariable("HF_HUB_OFFLINE", "1"),
        Node(
            package="lerobot_ros",
            executable="dataset_recorder",
            output="screen",
            prefix=_venv_python(),   # None: fall back to the entry point's own shebang
            parameters=[{"config": LaunchConfiguration("config")}],
        ),
    ])
