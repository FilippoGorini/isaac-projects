import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

ROBOT_YAML = PathJoinSubstitution(
    [FindPackageShare("vela_robots"), "config", LaunchConfiguration("robot"), "robot.yaml"])


def _check_robot(context):
    # launch_ros only warns about a missing params file, so fail loudly on a bad robot:=
    path = ROBOT_YAML.perform(context)
    if not os.path.isfile(path):
        raise RuntimeError(f"robot:={LaunchConfiguration('robot').perform(context)} has no profile ({path})")
    return []


def generate_launch_description():
    pkg = "vela_policy_client"
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
        DeclareLaunchArgument("robot", default_value="kinova_gen3_6dof",
                              description="Robot profile: vela_robots/config/<robot>/robot.yaml"),
        OpaqueFunction(function=_check_robot),

        Node(
            package=pkg,
            executable="policy_client_synchronous",
            name="policy_client_synchronous",
            output="screen",
            parameters=[
                config,
                ROBOT_YAML,
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
