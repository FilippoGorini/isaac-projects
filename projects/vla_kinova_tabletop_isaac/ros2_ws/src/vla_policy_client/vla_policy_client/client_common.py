"""Robot-facing pieces shared by the policy clients: the robot profile, the observation and the
binary gripper gate. The clients only differ in how they turn action chunks into arm motion."""

from dataclasses import dataclass

import numpy as np
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from sensor_msgs.msg import Image

from vla_policy_client.robot_params import T, declare_robot_params, robot_param

try:
    from openpi_client import image_tools
except ImportError as e:
    raise SystemExit(
        f"Failed to import openpi_client ({e}). Run: pip install openpi-client"
    ) from e

# Robot facts, no defaults (robot profile, see robot_params.py):
#   gripper hysteresis on the model's near-binary gripper output + the binary goal values;
#   arm joints in state/action order + the gripper joint (obs = joints + gripper);
#   cameras as parallel arrays: the image from topic i goes to the server as
#   obs[camera_keys[i]], use_sim picks the topic list.
ROBOT_PARAMS = {
    "gripper_close_threshold": T.DOUBLE, "gripper_open_threshold": T.DOUBLE,
    "gripper_closed_position": T.DOUBLE, "gripper_open_position": T.DOUBLE,
    "gripper_max_effort": T.DOUBLE,
    "arm_joints": T.STRING_ARRAY, "gripper_joint": T.STRING,
    "camera_keys": T.STRING_ARRAY,
    "sim_camera_topics": T.STRING_ARRAY, "real_camera_topics": T.STRING_ARRAY,
    "gripper_action": T.STRING,
}


@dataclass(frozen=True)
class RobotProfile:
    arm_joints: list
    gripper_joint: str
    cameras: dict               # camera_key -> image topic
    close_threshold: float
    open_threshold: float
    closed_position: float
    open_position: float
    max_effort: float
    gripper_action: str


def load_robot_profile(node, use_sim: bool) -> RobotProfile:
    """Declare and read the robot-profile parameters; exits naming any missing one."""
    declare_robot_params(node, ROBOT_PARAMS)
    rp = lambda name: robot_param(node, name)  # noqa: E731
    camera_keys = list(rp("camera_keys"))
    topics_param = "sim_camera_topics" if use_sim else "real_camera_topics"
    camera_topics = list(rp(topics_param))
    if len(camera_topics) != len(camera_keys):
        raise SystemExit(f"{topics_param} has {len(camera_topics)} topics but camera_keys "
                         f"has {len(camera_keys)}; they must be parallel arrays.")
    return RobotProfile(
        arm_joints=list(rp("arm_joints")), gripper_joint=rp("gripper_joint"),
        cameras=dict(zip(camera_keys, camera_topics)),
        close_threshold=rp("gripper_close_threshold"), open_threshold=rp("gripper_open_threshold"),
        closed_position=rp("gripper_closed_position"), open_position=rp("gripper_open_position"),
        max_effort=rp("gripper_max_effort"), gripper_action=rp("gripper_action"),
    )


def ros_image_to_uint8_hwc(msg: Image) -> np.ndarray:
    """ROS Image -> uint8 (H, W, 3) RGB, no resize."""
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "bgr8":
        return data[:, :, ::-1].copy()
    return data[:, :, :3].copy() if msg.encoding not in ("rgb8", "8UC3") else data.copy()


def make_obs(joint_positions: dict, images: dict, profile: RobotProfile, prompt: str,
             image_res: int):
    """openpi observation from snapshots of the latest joint positions and camera frames.

    Returns (obs, None), or (None, status) naming what is still missing. image_res > 0 resizes
    with padding client-side (lighter websocket payload; the server resizes anyway); 0 sends
    native frames.
    """
    missing = [j for j in profile.arm_joints + [profile.gripper_joint] if j not in joint_positions]
    if missing or any(img is None for img in images.values()):
        return None, f"joints missing {len(missing)} | " + " | ".join(
            f"{k} {'OK' if img is not None else 'WAIT'}" for k, img in images.items())
    joints = np.array([joint_positions[j] for j in profile.arm_joints], dtype=np.float32)
    gripper = np.array([joint_positions[profile.gripper_joint]], dtype=np.float32)
    if image_res > 0:
        images = {k: image_tools.resize_with_pad(v, image_res, image_res) for k, v in images.items()}
    return {"joints": joints, "gripper": gripper,
            **{k: np.ascontiguousarray(v) for k, v in images.items()}, "prompt": prompt}, None


class GripperGate:
    """Binary gripper driven by the model's near-binary gripper column: hysteresis (close when it
    rises above close_threshold, open when it falls below open_threshold, hold in between),
    level-triggered, so one GripperCommand goal per state change. Starts open (the robot is homed
    before VLA control begins).

    on_decision(cmd, position, value, dispatched) is called for every gate decision, so a client
    can trace it.
    """

    def __init__(self, node, profile: RobotProfile, on_decision=None):
        self._node = node
        self._p = profile
        self._client = ActionClient(node, GripperCommand, profile.gripper_action)
        self._on_decision = on_decision
        # +1 if the gripper closes towards larger values (e.g. a Robotiq knuckle), -1 otherwise
        self._sign = 1.0 if profile.closed_position >= profile.open_position else -1.0
        self.closed = False

    def update(self, value: float) -> None:
        # Flip the state ONLY if the goal was actually dispatched. If we flipped first and the
        # dispatch was skipped (server not up), the level-triggered gate would think it already
        # commanded the new state and never retry -- leaving the gripper stuck (e.g. closed on
        # the cube, never reopening).
        s, p = self._sign, self._p
        if not self.closed and s * value > s * p.close_threshold:
            if self._send(p.closed_position, value):
                self.closed = True
        elif self.closed and s * value < s * p.open_threshold:
            if self._send(p.open_position, value):
                self.closed = False

    def _send(self, position: float, value: float) -> bool:
        """Dispatch a go-to goal. Returns True iff it was sent (server available). NOTE: this
        confirms DISPATCH, not server acceptance -- send_goal_async is fire-and-forget."""
        cmd = "CLOSE" if position == self._p.closed_position else "OPEN"
        dispatched = self._client.wait_for_server(timeout_sec=0.0)
        if dispatched:
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = self._p.max_effort
            self._client.send_goal_async(goal)
            self._node.get_logger().info(f"Gripper -> {cmd} ({position:.2f})")
        else:
            self._node.get_logger().warn("Gripper action server unavailable; skipping goal.")
        if self._on_decision is not None:
            self._on_decision(cmd, position, value, dispatched)
        return dispatched
