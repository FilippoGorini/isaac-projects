# This launch files mirrors the kinova_vision's original launchfile, while also disabling the compressed/theora ...
# ... image transports: as we're recording on the same machine which publishes the images we don't need to waste CPU on compressing images
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    camera = LaunchConfiguration("camera").perform(context)
    device = LaunchConfiguration("device").perform(context)
    color_frame_id = LaunchConfiguration("color_frame_id").perform(context)
    camera_link_frame_id = LaunchConfiguration("camera_link_frame_id").perform(context)
    color_camera_info_url = LaunchConfiguration("color_camera_info_url").perform(context)
    max_color_pub_rate = LaunchConfiguration("max_color_pub_rate").perform(context)

    color_node = Node(
        package="kinova_vision",
        namespace=camera,
        executable="kinova_vision_node",
        name="kinova_vision_color",
        output="both",
        parameters=[{
            "camera_type": "color",
            "camera_name": "color",
            "camera_info_url_default":
                "package://kinova_vision/launch/calibration/default_color_calib_%ux%u.ini",
            "camera_info_url_user": color_camera_info_url,
            "stream_config":
                f"rtspsrc location=rtsp://{device}/color latency=30 "
                "! rtph264depay ! avdec_h264 ! videoconvert",
            "frame_id": color_frame_id,
            "max_pub_rate": float(max_color_pub_rate),
            # Publish raw only: no compressed / compressedDepth / theora encoders
            "image_raw.enable_pub_plugins": ["image_transport/raw"],
        }],
        remappings=[
            ("camera_info", "color/camera_info"),
            ("image_raw", "color/image_raw"),
        ],
    )

    color_tf_publisher = Node(
        package="tf2_ros",
        namespace=camera,
        executable="static_transform_publisher",
        name="camera_color_tf_publisher",
        output="both",
        arguments=["0", "0", "0", "0", "0", "0", camera_link_frame_id, color_frame_id],
    )

    return [color_node, color_tf_publisher]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "device", default_value="192.168.50.12",
            description="Kinova arm IPv4 address (RTSP color stream source)",
        ),
        DeclareLaunchArgument(
            "camera", default_value="camera",
            description="Namespace all wrist-camera topics are pushed into",
        ),
        DeclareLaunchArgument(
            "color_frame_id", default_value="camera_color_frame",
            description="Color image frame_id",
        ),
        DeclareLaunchArgument(
            "camera_link_frame_id", default_value="camera_link",
            description="Parent frame for the color tf publisher",
        ),
        DeclareLaunchArgument(
            "color_camera_info_url", default_value="",
            description="URL of a custom color calibration file (empty = sensor default)",
        ),
        DeclareLaunchArgument(
            "max_color_pub_rate", default_value="30.0",
            description="Maximum color image publication rate (Hz)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
