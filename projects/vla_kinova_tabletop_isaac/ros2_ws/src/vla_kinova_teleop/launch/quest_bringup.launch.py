"""
Quest 3 bringup: starts the ROS-TCP endpoint that the Quest VR app connects to,
plus the q2r2 right-arm controller that turns Quest controller poses into the
/target_frame topic consumed by the twist pose-tracking node.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    ros_ip = LaunchConfiguration("ros_ip").perform(context)
    ros_tcp_port = int(LaunchConfiguration("ros_tcp_port").perform(context))

    tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        output="screen",
        parameters=[{"ROS_IP": ros_ip}, {"ROS_TCP_PORT": ros_tcp_port}],
    )

    right_arm = Node(
        package="q2r2_bringup",
        executable="right_arm_controller",
        emulate_tty=True,
        output="screen",
    )

    return [tcp_endpoint, right_arm]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "ros_ip",
            default_value="0.0.0.0",
            description="Address the ROS-TCP endpoint binds to; '0.0.0.0' accepts any client",
        ),
        DeclareLaunchArgument(
            "ros_tcp_port",
            default_value="10000",
            description="TCP port the Quest VR app connects to",
        ),
        OpaqueFunction(function=launch_setup),
    ])
