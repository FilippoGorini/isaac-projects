"""
This launch file brings up controllers, swaps from joint_trajectory_controller
to picknik_twist_controller, then starts the twist-based pose tracking node and the
kortex_twist_adapter that turns its TwistStamped into the firmware's tool-frame Twist.
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
    bringup_pkg = "kinova_bringup"
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)
    tf_publish_rate = LaunchConfiguration("tf_publish_rate").perform(context)
    gripper_max_force = LaunchConfiguration("gripper_max_force").perform(context)

    # Suppress Cyclone DDS multicast-write warnings to avoid flooding terminal
    # The ros2 control node sometimes crashes but real errors were lost in the dds warnings
    quiet_dds = SetEnvironmentVariable(
        name="CYCLONEDDS_URI",
        value="<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity></Tracing></Domain></CycloneDDS>",
    )

    # frames / twist topics for the adapter come from the Kinova robot profile
    robot_yaml = os.path.join(
        get_package_share_directory("vela_robots"), "config", "kinova_gen3_6dof", "robot.yaml"
    )

    # Reuse the bringup launch file to bring up the kinova controllers (twist controller starts inactive)
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(bringup_pkg),
            "launch",
            "kinova_controllers.launch.py",
        )),
        launch_arguments={
            "use_sim": "false",
            "robot_ip": robot_ip,
            "auto_home": "false",
            "tf_publish_rate": tf_publish_rate,
            "gripper_max_force": gripper_max_force,
            "unwrap_joint_states": "true",      # Unwrap joint states to collect continuous data without -pi/+pi wraparounds
        }.items(),
    )

    # Deactivate JTC and activate twist controller (we wait a bit to ensure the kinova_controllers launch is done)
    # Also swap the gripper controller by uncommenting the arguments if you want to use velocity gripper mode
    switch_to_twist = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "control", "switch_controllers",
                "--activate", "twist_controller", #"gripper_velocity_controller",
                "--deactivate", "joint_trajectory_controller", #"robotiq_gripper_controller",
            ],
            output="screen",
        )],
    )

    # Start pose tracking node and the adapter after the controllers have been switched
    tracking_node = TimerAction(
        period=6.0,            
        actions=[
            Node(
                package=bringup_pkg,
                executable="kortex_twist_adapter.py",
                name="kortex_twist_adapter",
                output="screen",
                parameters=[robot_yaml],
            ),
            # generic tracker launch plus the kinova profile
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("vela_teleop"), "launch", "pose_tracking.launch.py",
                )),
                launch_arguments={"robot": "kinova_gen3_6dof"}.items(),
            ),
        ],
    )

    # Reuse the rviz config from bringup (originally written for MoveIt Servo)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory(bringup_pkg), "config", "servo.rviz"),
        ],
        condition=IfCondition(launch_rviz),
    )

    return [quiet_dds, controllers_launch, switch_to_twist, tracking_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.50.12",
            description="IP address of the real Kinova arm",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Bring up RViz alongside the twist tracking stack",
        ),
        DeclareLaunchArgument(
            "tf_publish_rate",
            default_value="200.0",
            description="robot_state_publisher /tf publish frequency [Hz], forwarded to kinova_controllers.launch.py",
        ),
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Gripper grasp force limit [0-100%], forwarded to kinova_controllers.launch.py. "
                        "Note: only applied in low-level cyclic mode; high-level twist teleop ignores it.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
