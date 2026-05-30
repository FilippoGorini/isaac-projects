"""
Thia launch file brings up controllers, swaps from joint_trajectory_controller
to picknik_twist_controller, then starts the twist-based pose tracking node.
This launch file can be used with real robot only, no IsaacSim (for now at least).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    my_package = "vla_kinova_tabletop"
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)

    # Suppress Cyclone DDS multicast-write warnings to avoid flooding terminal
    # The ros2 control node sometimes crashes but real errors were lost in the dds warnings 
    quiet_dds = SetEnvironmentVariable(
        name="CYCLONEDDS_URI",
        value="<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity></Tracing></Domain></CycloneDDS>",
    )

    config_dir = os.path.join(get_package_share_directory(my_package), "config")
    twist_pid_yaml = os.path.join(config_dir, "twist_pose_tracking.yaml")

    # We reuse the existing launch file to bringup the kinova controllers (twist controller starts inactive)
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(my_package),
            "launch",
            "kinova_controllers.launch.py",
        )),
        launch_arguments={
            "use_sim": "false",
            "robot_ip": robot_ip,
            "auto_home": "false",
        }.items(),
    )

    # Deactivate JTC and activate twist controller (we wait a bit to ensure the kinova_controllers launch is done)
    switch_to_twist = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "control", "switch_controllers",
                "--activate", "twist_controller",
                "--deactivate", "joint_trajectory_controller",
            ],
            output="screen",
        )],
    )

    # Start pose tracking node after the controllers have been switched
    tracking_node = TimerAction(
        period=5.0,            
        actions=[Node(
            package=my_package,
            executable="twist_pose_tracking_node.py",
            name="twist_pose_tracking_node",
            output="screen",
            parameters=[twist_pid_yaml],
        )],
    )

    # We keep the same rviz config we used with MoveIt Servo
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(config_dir, "servo.rviz")],
        condition=IfCondition(launch_rviz),
    )

    return [quiet_dds, controllers_launch, switch_to_twist, tracking_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.2.12",
            description="IP address of the real Kinova arm",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Bring up RViz alongside the twist tracking stack",
        ),
        OpaqueFunction(function=launch_setup),
    ])
