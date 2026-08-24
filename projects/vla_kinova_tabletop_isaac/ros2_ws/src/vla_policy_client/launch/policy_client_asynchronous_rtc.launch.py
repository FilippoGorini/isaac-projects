import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = "vla_policy_client"
EXECUTABLE = "policy_client_asynchronous_rtc"
CONFIG = "client_asynchronous_rtc.yaml"

# The yaml is the source of truth; each arg defaults to "" and only overrides the yaml
# when the user passes it on the CLI.
_BOOL = lambda v: v.lower() in ("true", "1", "yes")
_OVERRIDABLE = [
    ("policy_host", str),
    ("policy_port", int),
    ("prompt", str),
    ("control_hz", float),
    ("inference_hz", float),
    ("use_sim", _BOOL),
    ("plan_topic", str),
    ("gripper_hz", float),
    ("rtc_delay_steps", int),
    ("rtc_blend_steps", int),
    ("resize_images", _BOOL),
    ("image_resolution", int),
    ("debug_log_dir", str),
    ("js_log_hz", float),
    ("rate_report_sec", float),
]


def launch_setup(context, *args, **kwargs):
    config = os.path.join(get_package_share_directory(PKG), "config", CONFIG)
    overrides = {}
    for name, cast in _OVERRIDABLE:
        val = LaunchConfiguration(name).perform(context)
        if val != "":
            overrides[name] = cast(val)
    overrides["use_sim_time"] = _BOOL(LaunchConfiguration("use_sim").perform(context))
    return [Node(package=PKG, executable=EXECUTABLE, name=EXECUTABLE,
                 output="screen", parameters=[config, overrides])]


def generate_launch_description():
    def arg(name, desc):
        return DeclareLaunchArgument(name, default_value="",
                                     description=f"{desc} (empty => value from {CONFIG})")
    return LaunchDescription([
        arg("policy_host", "Policy server host"),
        arg("policy_port", "Policy server port"),
        arg("prompt", "Task prompt"),
        arg("control_hz", "Incoming VLA action rate / 30 Hz plan spacing"),
        arg("inference_hz", "Target chunk-generation rate (0 => free-running)"),
        arg("plan_topic", "30 Hz plan topic the C++ jtc_stream_node subscribes to"),
        arg("gripper_hz", "Gripper loop rate"),
        arg("rtc_delay_steps", "RTC: fixed committed-prefix length d (<= 8)"),
        arg("rtc_blend_steps", "RTC: short safety cross-fade (latency overrun); 0 => pure RTC"),
        arg("resize_images", "Client-side resize-with-pad before sending"),
        arg("image_resolution", "Resize target (px)"),
        arg("debug_log_dir", "Directory for the append-only JSONL trace ('' => off)"),
        arg("js_log_hz", "Measured-state trace rate (0 => every sample)"),
        arg("rate_report_sec", "Period of the live [rate] inference readouts"),
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="true => sim camera topics + sim clock; false => real robot"),
        OpaqueFunction(function=launch_setup),
    ])
