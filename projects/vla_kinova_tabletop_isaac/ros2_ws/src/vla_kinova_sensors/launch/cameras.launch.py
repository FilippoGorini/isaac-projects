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

    zed_disable_depth = LaunchConfiguration("zed_disable_depth").perform(context).lower() == "true"
    zed_resolution = LaunchConfiguration("zed_resolution").perform(context)
    zed_grab_fps = LaunchConfiguration("zed_grab_fps").perform(context)
    zed_pub_fps = LaunchConfiguration("zed_pub_fps").perform(context)

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
        overrides = [
            f"general.grab_resolution:={zed_resolution}",
            f"general.grab_frame_rate:={zed_grab_fps}",
            f"general.pub_frame_rate:={zed_pub_fps}",
        ]
        if zed_disable_depth:
            # 'NONE' disables every module that needs depth (point cloud, pos tracking, ...).
            overrides.append("depth.depth_mode:=NONE")

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch", "zed_camera.launch.py",
            )),
            launch_arguments={
                "camera_model": zed_camera_model,
                "param_overrides": ";".join(overrides),
            }.items(),
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
        DeclareLaunchArgument(
            "zed_disable_depth",
            default_value="true",
            description="Disable depth on the ZED (sets depth.depth_mode:=NONE, which also disables point cloud / pos tracking)",
        ),
        DeclareLaunchArgument(
            "zed_resolution",
            default_value="HD720",
            description="ZED grab resolution: 'HD2K', 'HD1080', 'HD720', 'VGA', 'AUTO'",
        ),
        DeclareLaunchArgument(
            "zed_grab_fps",
            default_value="30",
            description="ZED internal grab frame rate (Hz). Allowed values depend on resolution",
        ),
        DeclareLaunchArgument(
            "zed_pub_fps",
            default_value="0.0",
            description="ZED image publish rate (Hz). 0 = no limit (matches grab rate)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
