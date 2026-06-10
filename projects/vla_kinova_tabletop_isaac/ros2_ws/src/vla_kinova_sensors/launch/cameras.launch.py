import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    launch_wrist = LaunchConfiguration("launch_wrist").perform(context).lower() == "true"
    launch_zed = LaunchConfiguration("launch_zed").perform(context).lower() == "true"
    zed_camera_model = LaunchConfiguration("zed_camera_model").perform(context)
    kinova_ip = LaunchConfiguration("kinova_ip").perform(context)

    actions = []

    if launch_wrist:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("kinova_vision"),
                "launch", "kinova_vision.launch.py",
            )),
            launch_arguments={
                "launch_depth": "false",
                "device": kinova_ip,
            }.items(),
        ))

    if launch_zed:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch", "zed_camera.launch.py",
            )),
            launch_arguments={"camera_model": zed_camera_model}.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_wrist",
            default_value="true",
            description="Bring up the Kinova wrist camera (RGB only, depth disabled)",
        ),
        DeclareLaunchArgument(
            "launch_zed",
            default_value="true",
            description="Bring up the ZED 2 camera",
        ),
        DeclareLaunchArgument(
            "zed_camera_model",
            default_value="zed2",
            description="ZED model passed to zed_wrapper",
        ),
        DeclareLaunchArgument(
            "kinova_ip",
            default_value="192.168.50.12",
            description="IPv4 address of the Kinova arm; forwarded to kinova_vision as 'device'",
        ),
        OpaqueFunction(function=launch_setup),
    ])
