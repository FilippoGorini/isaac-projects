import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = "vla_policy_client"
    config = os.path.join(get_package_share_directory(pkg), "config", "client.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("policy_host", default_value="localhost",
                              description="Hostname or IP of the policy server"),
        DeclareLaunchArgument("policy_port", default_value="8000",
                              description="WebSocket port of the policy server"),
        DeclareLaunchArgument("prompt", default_value="pick up the object",
                              description="Language instruction for the policy"),

        Node(
            package=pkg,
            executable="policy_client",
            name="policy_client",
            output="screen",
            parameters=[
                config,
                {
                    "policy_host": LaunchConfiguration("policy_host"),
                    "policy_port": LaunchConfiguration("policy_port"),
                    "prompt": LaunchConfiguration("prompt"),
                },
            ],
        ),
    ])
