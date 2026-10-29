"""Combined bring-up: the asynchronous RTC VLA policy client + the C++ JTC streamer.

The client (vela_policy_client/policy_client_asynchronous_rtc) publishes the 30 Hz plan on
`plan_topic`; the streamer (vela_jtc_streamer/jtc_stream_node) subscribes there,
oversamples it and streams short-horizon points to the JTC. This wraps both sub-launches
so one command brings up the whole arm pipeline. `plan_topic`, `control_hz` and `use_sim`
are declared ONCE here and forwarded to both so they can't drift apart. Every other arg
defaults to "" => each sub-launch falls back to its own yaml.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

LC = LaunchConfiguration
_BOOL = lambda v: v.lower() in ("true", "1", "yes")


def _jtc_topic(context):
    path = os.path.join(get_package_share_directory("vela_robots"), "config",
                        LC("robot").perform(context), "robot.yaml")
    with open(path) as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]["arm_trajectory_topic"]


def launch_setup(context, *args, **kwargs):
    client_launch = os.path.join(
        get_package_share_directory("vela_policy_client"), "launch",
        "policy_client_asynchronous_rtc.launch.py")
    streamer_launch = os.path.join(
        get_package_share_directory("vela_jtc_streamer"), "launch",
        "jtc_stream_node.launch.py")
    use_streamer = LC("use_streamer").perform(context)
    streamer_on = _BOOL(use_streamer) if use_streamer else not _BOOL(LC("use_sim").perform(context))
    plan_topic = LC("plan_topic") if streamer_on else _jtc_topic(context)

    client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(client_launch),
        launch_arguments={
            "rtc_delay_steps": LC("rtc_delay_steps"),
            "rtc_blend_steps": LC("rtc_blend_steps"),
            "inference_hz": LC("inference_hz"),
            "prompt": LC("prompt"),
            "policy_host": LC("policy_host"),
            "policy_port": LC("policy_port"),
            "debug_log_dir": LC("debug_log_dir"),
            "plan_topic": plan_topic,
            "control_hz": LC("control_hz"),
            "use_sim": LC("use_sim"),
            "robot": LC("robot"),
        }.items(),
    )
    if not streamer_on:
        return [LogInfo(msg=f"streamer off, plan goes straight to {plan_topic}"), client]
    streamer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(streamer_launch),
        launch_arguments={
            "tick_hz": LC("tick_hz"),
            "jtc_horizon": LC("jtc_horizon"),
            "control_hz": LC("control_hz"),
            "plan_topic": LC("plan_topic"),
            "use_sim": LC("use_sim"),
            "robot": LC("robot"),
        }.items(),
    )
    return [client, streamer]


def generate_launch_description():

    # Forwarded per-node args (empty => that sub-launch uses its own yaml default).
    fwd = [
        ("rtc_delay_steps", "client RTC: fixed committed-prefix length d (<= 8)"),
        ("rtc_blend_steps", "client RTC: short safety cross-fade (latency overrun); 0 => pure RTC"),
        ("inference_hz", "client: target chunk-generation rate (0 => free-running)"),
        ("prompt", "client: task prompt"),
        ("policy_host", "client: policy server host"),
        ("policy_port", "client: policy server port"),
        ("debug_log_dir", "client: directory for the JSONL trace ('' => off)"),
        ("tick_hz", "streamer: oversample/stream rate (Hz)"),
        ("jtc_horizon", "streamer: JTC point time_from_start (s)"),
    ]
    declared = [DeclareLaunchArgument(n, default_value="", description=d) for n, d in fwd]

    # Shared args both nodes must agree on (declared once, non-empty defaults).
    declared += [
        DeclareLaunchArgument("plan_topic", default_value="/vla_arm_plan",
                              description="30 Hz plan topic (client publishes, streamer subscribes)"),
        DeclareLaunchArgument("control_hz", default_value="30.0",
                              description="30 Hz plan spacing (client) + streamer fallback knot spacing"),
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="true => sim clock + sim cameras; false => real robot"),
        DeclareLaunchArgument("robot", default_value="kinova_gen3_6dof",
                              description="Robot profile: vela_robots/config/<robot>/robot.yaml"),
        DeclareLaunchArgument("use_streamer", default_value="",
                              description="true / false; empty => off in sim (Isaac's coarse clock starves "
                                          "the 300 Hz stream), on for the real robot. Off: plan goes "
                                          "straight to the profile's arm_trajectory_topic"),
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=launch_setup)])
