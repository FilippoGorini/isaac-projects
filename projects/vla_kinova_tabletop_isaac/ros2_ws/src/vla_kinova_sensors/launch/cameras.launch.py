import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def realsense_node(name, serial, color_profile):
    # Run the node directly instead of including rs_launch.py: that wrapper
    # forwards every launch-arg in the shared context to the node, which
    # spams 'parameter not supported' warnings for kinova's / our args
    # As for the wrist, we don't enable the compressed/theora image transports
    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="realsense",
        name=name,
        output="screen",
        parameters=[{
            "camera_name": name,
            # "" = first device found; pin it when more than one D435 is plugged in.
            # value_type=str or launch yaml-parses numeric serials into ints
            "serial_no": ParameterValue(serial, value_type=str),
            "enable_depth": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "pointcloud.enable": False,
            "rgb_camera.color_profile": color_profile,
            f"{name}.color.image_raw.enable_pub_plugins": ["image_transport/raw"],
        }],
    )


def launch_setup(context, *args, **kwargs):
    launch_wrist = LaunchConfiguration("launch_wrist").perform(context).lower() == "true"
    launch_zed = LaunchConfiguration("launch_zed").perform(context).lower() == "true"
    launch_realsense = (
        LaunchConfiguration("launch_realsense").perform(context).lower() == "true"
    )
    launch_realsense_2 = (
        LaunchConfiguration("launch_realsense_2").perform(context).lower() == "true"
    )
    realsense_color_profile = LaunchConfiguration("realsense_color_profile").perform(context)
    realsense_serial = LaunchConfiguration("realsense_serial").perform(context)
    realsense_2_serial = LaunchConfiguration("realsense_2_serial").perform(context)
    zed_camera_model = LaunchConfiguration("zed_camera_model").perform(context)
    kinova_ip = LaunchConfiguration("kinova_ip").perform(context)
    disable_autofocus = LaunchConfiguration("disable_autofocus").perform(context)

    zed_disable_depth = LaunchConfiguration("zed_disable_depth").perform(context).lower() == "true"
    zed_resolution = LaunchConfiguration("zed_resolution").perform(context)
    zed_grab_fps = LaunchConfiguration("zed_grab_fps").perform(context)
    zed_pub_fps = LaunchConfiguration("zed_pub_fps").perform(context)

    actions = []

    if launch_wrist:
        # Include our own kinova camera bringup without depth and without comrpessed image transports
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("vla_kinova_sensors"),
                "launch", "wrist_camera.launch.py",
            )),
            launch_arguments={
                "device": kinova_ip,
                "disable_autofocus": disable_autofocus,
            }.items(),
        ))

    if launch_zed:
        # For now the zed still publsihes compressed images too, will fix it if we ever switch to zed again, but unnecessary as of now
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

    if launch_realsense_2 and not (realsense_serial and realsense_2_serial):
        # Without both serials the two nodes race for the same first device
        raise RuntimeError(
            "launch_realsense_2:=true needs both realsense_serial and realsense_2_serial "
            "(list them with `rs-enumerate-devices -s`)"
        )

    if launch_realsense:
        actions.append(realsense_node("d435", realsense_serial, realsense_color_profile))

    if launch_realsense_2:
        # Second external camera, same profile as the first
        actions.append(realsense_node("d435_2", realsense_2_serial, realsense_color_profile))

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
            default_value="false",
            description="Bring up the ZED 2 camera",
        ),
        DeclareLaunchArgument(
            "launch_realsense",
            default_value="true",
            description="Bring up the Intel RealSense D435 (RGB only; recorded as observation.images.external)",
        ),
        DeclareLaunchArgument(
            "realsense_serial",
            default_value="243522072381",
            description="Serial of the first D435 (empty = first device found, only valid without launch_realsense_2)",
        ),
        DeclareLaunchArgument(
            "launch_realsense_2",
            default_value="false",
            description="Bring up a second D435 (RGB only; recorded as observation.images.external_2 with kinova_pi05_2ext.toml)",
        ),
        DeclareLaunchArgument(
            "realsense_2_serial",
            default_value="244222074111",
            description="Serial of the second D435",
        ),
        DeclareLaunchArgument(
            "realsense_color_profile",
            default_value="640x480x30",
            description="RealSense color profile WIDTHxHEIGHTxFPS, shared by both D435s (valid combos depend on USB link; `rs-enumerate-devices -c`)",
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
            "disable_autofocus",
            default_value="true",
            description="Replace the wrist colour autofocus with a fixed manual focus once the stream is up",
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
