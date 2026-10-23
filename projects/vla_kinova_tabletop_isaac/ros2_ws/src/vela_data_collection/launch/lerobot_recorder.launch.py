"""Launch the lerobot_ros dataset_recorder with the robot profile's TOML as default.

Convenience wrapper so you don't have to type the full
`ros2 run lerobot_ros dataset_recorder --ros-args -p config:=$(ros2 pkg prefix
vela_robots)/share/vela_robots/config/kinova_gen3_6dof/recorder.toml` every time. Also sets
HF_HUB_OFFLINE=1 so reopening an existing dataset doesn't try to fetch it
from the Hugging Face Hub.

Usage:
    ros2 launch vela_data_collection lerobot_recorder.launch.py
    ros2 launch vela_data_collection lerobot_recorder.launch.py config:=<other.toml>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
            parameters=[{"config": LaunchConfiguration("config")}],
        ),
    ])
