import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = "vla_policy_client"
    config = os.path.join(
        get_package_share_directory(pkg), "config", "client_synchronous.yaml"
    )

    use_sim = LaunchConfiguration("use_sim")

    return LaunchDescription([
        DeclareLaunchArgument("policy_host", default_value="localhost",
                              description="Hostname or IP of the policy server"),
        DeclareLaunchArgument("policy_port", default_value="8000",
                              description="WebSocket port of the policy server"),
        DeclareLaunchArgument("prompt", default_value="lift the blue cube",
                              description="Language instruction for the policy"),
        DeclareLaunchArgument(
            "use_sim", default_value="true",
            description="true => Isaac Sim camera topics + use_sim_time; "
                        "false => real-robot camera topics. Match the controllers bringup.",
        ),
        DeclareLaunchArgument(
            "resize_images", default_value="true",
            description="Resize-with-pad frames client-side (PIL) before sending, "
                        "to shrink the websocket payload",
        ),
        DeclareLaunchArgument(
            "image_resolution", default_value="224",
            description="Target square size for client-side resize, should match the server-side resize of the policy.",
        ),

        Node(
            package=pkg,
            executable="policy_client_synchronous",
            name="policy_client_synchronous",
            output="screen",
            parameters=[
                config,
                {
                    "policy_host": LaunchConfiguration("policy_host"),
                    "policy_port": LaunchConfiguration("policy_port"),
                    "prompt": LaunchConfiguration("prompt"),
                    "use_sim": use_sim,
                    "use_sim_time": use_sim,
                    "resize_images": LaunchConfiguration("resize_images"),
                    "image_resolution": LaunchConfiguration("image_resolution"),
                },
            ],
        ),
    ])
