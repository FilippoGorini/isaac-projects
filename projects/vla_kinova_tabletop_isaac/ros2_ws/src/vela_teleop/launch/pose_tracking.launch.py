"""
Starts the twist pose tracking node for one robot profile (robot:=<name>).

Parameters: vela_teleop's twist_pose_tracking.yaml (default gains), then
vela_robots/config/<robot>/robot.yaml on top (frames, twist topic, optional retuned gains).
Only the tracker: the robot bringup, the controller switch and the twist consumer
(e.g. kinova_bringup's kortex_twist_adapter, or MoveIt Servo) belong to the robot's own
bringup launch, which includes this file. The Quest side is quest_bringup.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_yaml(context):
    # launch_ros only warns about a missing params file, so fail loudly on a bad robot:=
    robot = LaunchConfiguration("robot").perform(context)
    path = os.path.join(get_package_share_directory("vela_robots"), "config", robot, "robot.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"robot:={robot} has no profile ({path})")
    return path


def launch_setup(context, *args, **kwargs):
    gains_yaml = os.path.join(
        get_package_share_directory("vela_teleop"), "config", "twist_pose_tracking.yaml"
    )
    return [Node(
        package="vela_teleop",
        executable="twist_pose_tracking_node.py",
        name="twist_pose_tracking_node",
        output="screen",
        parameters=[gains_yaml, _robot_yaml(context)],   # profile last: it can override the gains too
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="kinova_gen3_6dof",
            description="Robot profile: vela_robots/config/<robot>/robot.yaml",
        ),
        OpaqueFunction(function=launch_setup),
    ])
