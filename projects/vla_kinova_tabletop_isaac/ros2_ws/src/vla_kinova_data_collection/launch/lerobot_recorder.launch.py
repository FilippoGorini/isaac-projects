"""Launch the lerobot_ros dataset_recorder with the Kinova TOML as default.

Convenience wrapper so you don't have to type the full
`ros2 run lerobot_ros dataset_recorder --ros-args -p config:=$(ros2 pkg prefix
vla_kinova_data_collection)/share/.../kinova_pi05.toml` every time. Also sets
HF_HUB_OFFLINE=1 so reopening an existing dataset doesn't try to fetch it
from the Hugging Face Hub.

Usage:
    ros2 launch vla_kinova_data_collection lerobot_recorder.launch.py
    ros2 launch vla_kinova_data_collection lerobot_recorder.launch.py config:=<other.toml>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("vla_kinova_data_collection"),
        "config",
        "kinova_pi05.toml",
    ])

    return LaunchDescription([
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
            parameters=[{"config": LaunchConfiguration("config")}],
        ),
    ])
