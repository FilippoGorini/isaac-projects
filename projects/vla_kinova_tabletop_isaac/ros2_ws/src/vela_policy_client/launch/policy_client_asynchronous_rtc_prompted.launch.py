"""Voice-prompted bring-up: the voice asynchronous RTC policy client + the C++ JTC streamer.

Starts policy_client_asynchronous_rtc_prompted (which HOLDS observations until the first prompt
arrives on `prompt_topic`) together with the vela_jtc_streamer/jtc_stream_node, so one
command brings up the whole arm pipeline; it then idles until a prompt is spoken. Run the
speech_to_prompt node in a SECOND terminal (it owns the keyboard + microphone and loads whisper;
ros2 run, NOT a launch file -- launch gives children no stdin so the keys wouldn't work):

    ros2 launch vela_policy_client policy_client_asynchronous_rtc_prompted.launch.py   # this file
    ros2 run vela_policy_client speech_to_prompt                                        # 2nd terminal

`plan_topic`, `control_hz` and `use_sim` are declared ONCE here and forwarded to both nodes
so they can't drift apart. Every other arg defaults to "" => yaml value.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = "vela_policy_client"
EXECUTABLE = "policy_client_asynchronous_rtc_prompted"
CONFIG = "client_asynchronous_rtc_prompted.yaml"

# The yaml is the source of truth; each arg defaults to "" and only overrides the yaml
# when the user passes it on the CLI.
_BOOL = lambda v: v.lower() in ("true", "1", "yes")
_OVERRIDABLE = [
    ("policy_host", str),
    ("policy_port", int),
    ("prompt", str),            # optional INITIAL prompt ("" => wait for the first voice prompt)
    ("prompt_topic", str),
    ("control_hz", float),
    ("inference_hz", float),
    ("use_sim", _BOOL),
    ("plan_topic", str),
    ("gripper_hz", float),
    ("rtc_delay_steps", int),
    ("model_max_delay", int),
    ("rtc_blend_steps", int),
    ("resize_images", _BOOL),
    ("image_resolution", int),
    ("debug_log_dir", str),
    ("js_log_hz", float),
    ("rate_report_sec", float),
]


def _robot_yaml(context):
    # launch_ros only warns about a missing params file, so fail loudly on a bad robot:=
    robot = LaunchConfiguration("robot").perform(context)
    path = os.path.join(get_package_share_directory("vela_robots"), "config", robot, "robot.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"robot:={robot} has no profile ({path})")
    return path


def _jtc_topic(context):
    with open(_robot_yaml(context)) as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]["arm_trajectory_topic"]


def launch_setup(context, *args, **kwargs):
    config = os.path.join(get_package_share_directory(PKG), "config", CONFIG)
    overrides = {}
    for name, cast in _OVERRIDABLE:
        val = LaunchConfiguration(name).perform(context)
        if val != "":
            overrides[name] = cast(val)
    overrides["use_sim_time"] = _BOOL(LaunchConfiguration("use_sim").perform(context))
    use_streamer = LaunchConfiguration("use_streamer").perform(context)
    streamer_on = _BOOL(use_streamer) if use_streamer else not overrides["use_sim_time"]
    if not streamer_on:
        overrides["plan_topic"] = _jtc_topic(context)
    client = Node(package=PKG, executable=EXECUTABLE, name=EXECUTABLE,
                  output="screen", parameters=[config, _robot_yaml(context), overrides])
    if not streamer_on:
        return [LogInfo(msg=f"streamer off, plan goes straight to {overrides['plan_topic']}"), client]

    streamer_launch = os.path.join(
        get_package_share_directory("vela_jtc_streamer"), "launch",
        "jtc_stream_node.launch.py")
    streamer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(streamer_launch),
        launch_arguments={
            "tick_hz": LaunchConfiguration("tick_hz"),
            "jtc_horizon": LaunchConfiguration("jtc_horizon"),
            "control_hz": LaunchConfiguration("control_hz", default="30.0"),
            "plan_topic": LaunchConfiguration("plan_topic", default="/vla_arm_plan"),
            "use_sim": LaunchConfiguration("use_sim"),
            "robot": LaunchConfiguration("robot"),
        }.items(),
    )
    return [client, streamer]


def generate_launch_description():
    def arg(name, desc):
        return DeclareLaunchArgument(name, default_value="",
                                     description=f"{desc} (empty => value from {CONFIG})")

    return LaunchDescription([
        arg("policy_host", "Policy server host"),
        arg("policy_port", "Policy server port"),
        arg("prompt", "Optional INITIAL prompt ('' => hold until the first voice prompt)"),
        arg("prompt_topic", "Topic the speech_to_prompt node publishes prompts on"),
        arg("inference_hz", "Target chunk-generation rate (0 => free-running)"),
        arg("gripper_hz", "Gripper loop rate"),
        arg("rtc_delay_steps", "RTC: fixed committed-prefix length d (<= model_max_delay-1)"),
        arg("model_max_delay", "Served checkpoint's trained max_delay (clutter_rtc=10, finetune_rtc=8)"),
        arg("rtc_blend_steps", "RTC: short safety cross-fade (latency overrun); 0 => pure RTC"),
        arg("resize_images", "Client-side resize-with-pad before sending"),
        arg("image_resolution", "Resize target (px)"),
        arg("debug_log_dir", "Directory for the append-only JSONL trace ('' => off)"),
        arg("js_log_hz", "Measured-state trace rate (0 => every sample)"),
        arg("rate_report_sec", "Period of the live [rate] inference readouts"),
        arg("tick_hz", "streamer: oversample/stream rate (Hz)"),
        arg("jtc_horizon", "streamer: JTC point time_from_start (s)"),
        # Shared args both nodes must agree on (declared once, non-empty defaults).
        DeclareLaunchArgument("plan_topic", default_value="/vla_arm_plan",
                              description="30 Hz plan topic (client publishes, streamer subscribes)"),
        DeclareLaunchArgument("control_hz", default_value="30.0",
                              description="30 Hz plan spacing (client) + streamer fallback knot spacing"),
        DeclareLaunchArgument("robot", default_value="kinova_gen3_6dof",
                              description="Robot profile: vela_robots/config/<robot>/robot.yaml"),
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="true => sim clock + sim cameras; false => real robot"),
        DeclareLaunchArgument("use_streamer", default_value="",
                              description="true / false; empty => off in sim (Isaac's coarse clock starves "
                                          "the 300 Hz stream), on for the real robot. Off: plan goes "
                                          "straight to the profile's arm_trajectory_topic"),
        OpaqueFunction(function=launch_setup),
    ])
