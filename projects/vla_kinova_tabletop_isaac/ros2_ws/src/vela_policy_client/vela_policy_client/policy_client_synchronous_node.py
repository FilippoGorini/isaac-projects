#!/usr/bin/env python3
"""Synchronous ROS 2 policy client for a finetuned chunked VLA (e.g. pi0.5).

One blocking cycle per chunk (movement happens in visible steps, by design):

  1. snapshot the current joint state + one frame per camera,
  2. send the observation over websocket and wait for the action chunk,
  3. send the whole chunk as ONE FollowJointTrajectory goal (point i stamped at
     (i+1)/control_hz s) and, concurrently, step the chunk's per-timestep gripper
     output at control_hz through a hysteresis gate,
  4. await the arm trajectory result, then restart the cycle.

The 30-step gripper loop provides the wall-clock pacing (~1 s for a 30-point
chunk at 30 Hz); the arm runs the same span concurrently via the JTC and its
awaited result gates the next inference. A MultiThreadedExecutor is required so
the action-client callbacks fire while the control loop thread runs.

Works against both the real robot and a simulator: only the camera topics and
use_sim_time differ, selected by the `use_sim` parameter. Robot facts (joints,
gripper, cameras, gripper action) have no defaults: they come from the robot
profile, see robot_params.py.
"""

import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from vela_policy_client.robot_params import T, declare_robot_params, robot_param

try:
    from openpi_client import image_tools
    from openpi_client.websocket_client_policy import WebsocketClientPolicy
except ImportError as e:
    raise SystemExit(
        f"Failed to import openpi_client ({e}). "
        "Run: pip install openpi-client typing-extensions"
    ) from e

# To save websocket bandwidth, images are padded and resized client-side to `image_resolution` (default 224) using PIL
# Notice that PIL bilinear interpolation is not byte for byte identical to JAX linear interpolation on the server side


def _ros_image_to_uint8_hwc(msg: Image) -> np.ndarray:
    """ROS Image -> uint8 (H, W, 3) RGB, no resize."""
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "bgr8":
        return data[:, :, ::-1].copy()
    return data[:, :, :3].copy() if msg.encoding not in ("rgb8", "8UC3") else data.copy()


class PolicyClientSynchronousNode(Node):
    def __init__(self):
        super().__init__("policy_client_synchronous")

        # --- parameters ---
        self.declare_parameter("policy_host", "localhost")
        self.declare_parameter("policy_port", 8000)
        self.declare_parameter("prompt", "lift the blue cube")
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("use_sim", True)

        # Robot facts, no defaults (robot profile, see robot_params.py):
        #   gripper hysteresis (two-threshold, level-triggered) on the model's near-binary
        #   gripper output: close when it rises above `close_threshold`, open when it falls
        #   below `open_threshold`, hold in between. A goal is only sent on a state change,
        #   so the controller is not spammed every frame;
        #   arm joints in state/action order + the gripper joint (obs = joints + gripper);
        #   cameras as parallel arrays: the image from topic i goes to the server as
        #   obs[camera_keys[i]]. Both the sim and real lists are parameters; `use_sim` picks one.
        declare_robot_params(self, {
            "gripper_close_threshold": T.DOUBLE, "gripper_open_threshold": T.DOUBLE,
            "gripper_closed_position": T.DOUBLE, "gripper_open_position": T.DOUBLE,
            "gripper_max_effort": T.DOUBLE,
            "arm_joints": T.STRING_ARRAY, "gripper_joint": T.STRING,
            "camera_keys": T.STRING_ARRAY,
            "sim_camera_topics": T.STRING_ARRAY, "real_camera_topics": T.STRING_ARRAY,
            "gripper_action": T.STRING,
        })
        # The joint-state and controller topics are NOT split: /joint_states (the
        # joint_state_broadcaster output, what the dataset was recorded from) and the
        # controller actions are the same in sim and real. The node indexes joint values
        # by name, so the /joint_states ordering is irrelevant.
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "arm_action", "/joint_trajectory_controller/follow_joint_trajectory"
        )

        self.declare_parameter("resize_images", True)
        self.declare_parameter("image_resolution", 224)

        gp = self.get_parameter
        host = gp("policy_host").get_parameter_value().string_value
        port = gp("policy_port").get_parameter_value().integer_value
        self._prompt = gp("prompt").get_parameter_value().string_value
        hz = gp("control_hz").get_parameter_value().double_value
        use_sim = gp("use_sim").get_parameter_value().bool_value

        rp = lambda name: robot_param(self, name)  # noqa: E731
        self._close_thr = rp("gripper_close_threshold")
        self._open_thr = rp("gripper_open_threshold")
        self._grip_closed_pos = rp("gripper_closed_position")
        self._grip_open_pos = rp("gripper_open_position")
        self._grip_effort = rp("gripper_max_effort")
        # +1 if the gripper closes towards larger values (e.g. a Robotiq knuckle), -1 otherwise
        self._grip_sign = 1.0 if self._grip_closed_pos >= self._grip_open_pos else -1.0

        self._arm_joints = list(rp("arm_joints"))
        self._gripper_joint = rp("gripper_joint")
        self._n_arm = len(self._arm_joints)

        js_topic = gp("joint_states_topic").get_parameter_value().string_value
        arm_action = gp("arm_action").get_parameter_value().string_value
        gripper_action = rp("gripper_action")

        self._resize_images = gp("resize_images").get_parameter_value().bool_value
        self._image_res = gp("image_resolution").get_parameter_value().integer_value

        camera_keys = list(rp("camera_keys"))
        topics_param = "sim_camera_topics" if use_sim else "real_camera_topics"
        camera_topics = list(rp(topics_param))
        if len(camera_topics) != len(camera_keys):
            raise SystemExit(
                f"{topics_param} has {len(camera_topics)} topics but camera_keys "
                f"has {len(camera_keys)}; they must be parallel arrays."
            )
        cameras = dict(zip(camera_keys, camera_topics))

        self._dt = 1.0 / hz
        # Gripper starts open (the robot is homed before VLA control begins).
        self._gripper_closed = False

        self.get_logger().info(
            f"Mode: {'SIM' if use_sim else 'REAL'} | "
            f"cameras={cameras} js='{js_topic}' | "
            f"resize={'%dx%d (PIL)' % (self._image_res, self._image_res) if self._resize_images else 'off (native)'}"
        )

        # --- policy server connection ---
        self.get_logger().info(f"Connecting to policy server ws://{host}:{port} ...")
        self._policy = WebsocketClientPolicy(host=host, port=port)
        self.get_logger().info("Connected.")

        # --- observation state (guarded by _obs_lock) ---
        self._obs_lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._images: dict[str, np.ndarray | None] = {k: None for k in camera_keys}

        # --- action clients ---
        self._arm_client = ActionClient(self, FollowJointTrajectory, arm_action)
        self._gripper_client = ActionClient(self, GripperCommand, gripper_action)

        # --- arm-goal completion plumbing (set by action-client callbacks) ---
        self._arm_done = threading.Event()
        self._arm_done.set()

        # --- subscriptions ---
        self.create_subscription(JointState, js_topic, self._cb_joint_states, 10)
        for key, topic in cameras.items():
            self.create_subscription(
                Image, topic, lambda msg, key=key: self._cb_image(key, msg), 2
            )

        # --- control loop thread ---
        self._running = True
        self._loop_thread = threading.Thread(target=self._run_control_loop, daemon=True)
        self._loop_thread.start()
        self.get_logger().info("Synchronous policy client started.")

    # ------------------------------------------------------------------
    # Observation callbacks
    # ------------------------------------------------------------------

    def _cb_joint_states(self, msg: JointState) -> None:
        with self._obs_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def _cb_image(self, key: str, msg: Image) -> None:
        arr = _ros_image_to_uint8_hwc(msg)
        with self._obs_lock:
            self._images[key] = arr

    # ------------------------------------------------------------------
    # Observation packaging
    # ------------------------------------------------------------------

    def _build_obs(self) -> dict | None:
        with self._obs_lock:
            missing = [
                j for j in self._arm_joints + [self._gripper_joint]
                if j not in self._joint_positions
            ]
            if missing or any(img is None for img in self._images.values()):
                return None
            joints = np.array(
                [self._joint_positions[j] for j in self._arm_joints], dtype=np.float32
            )
            gripper = np.array(
                [self._joint_positions[self._gripper_joint]], dtype=np.float32
            )
            images = dict(self._images)

        # Resize outside of the lock to keep the callbacks responsive
        if self._resize_images:
            images = {
                k: image_tools.resize_with_pad(v, self._image_res, self._image_res)
                for k, v in images.items()
            }

        return {
            "joints": joints,
            "gripper": gripper,
            **{k: np.ascontiguousarray(v) for k, v in images.items()},
            "prompt": self._prompt,
        }

    # ------------------------------------------------------------------
    # Arm: whole chunk as one FollowJointTrajectory goal, result awaited
    # ------------------------------------------------------------------

    def _send_arm_chunk(self, actions: np.ndarray) -> bool:
        """Send the chunk as a single multi-point goal. Returns False if the arm
        action server is unavailable."""
        if not self._arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Arm action server unavailable.")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._arm_joints
        for i, action in enumerate(actions):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in action[:self._n_arm]]
            elapsed_ns = int((i + 1) * self._dt * 1e9)
            pt.time_from_start = Duration(
                sec=elapsed_ns // 1_000_000_000,
                nanosec=elapsed_ns % 1_000_000_000,
            )
            goal.trajectory.points.append(pt)

        self._arm_done.clear()
        send_future = self._arm_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_arm_goal_response)
        return True

    def _on_arm_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Arm trajectory goal rejected.")
            self._arm_done.set()
            return
        goal_handle.get_result_async().add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future) -> None:
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(
                f"Arm trajectory finished with error_code={result.error_code} "
                f"({result.error_string})"
            )
        self._arm_done.set()

    # ------------------------------------------------------------------
    # Gripper: hysteresis gate, level-triggered (one goal per state change)
    # ------------------------------------------------------------------

    def _update_gripper(self, value: float) -> None:
        s = self._grip_sign
        if not self._gripper_closed and s * value > s * self._close_thr:
            self._gripper_closed = True
            self._send_gripper_goal(self._grip_closed_pos)
        elif self._gripper_closed and s * value < s * self._open_thr:
            self._gripper_closed = False
            self._send_gripper_goal(self._grip_open_pos)

    def _send_gripper_goal(self, position: float) -> None:
        if not self._gripper_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn("Gripper action server unavailable; skipping goal.")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self._grip_effort
        self._gripper_client.send_goal_async(goal)
        self.get_logger().info(
            f"Gripper -> {'CLOSE' if position == self._grip_closed_pos else 'OPEN'} "
            f"({position:.2f})"
        )

    # ------------------------------------------------------------------
    # Control loop (one blocking cycle per chunk)
    # ------------------------------------------------------------------

    def _run_control_loop(self) -> None:
        try:
            self._control_loop()
        except Exception:
            # Ctrl-C shuts the context down while the loop may be mid-chunk (gripper / action
            # calls then raise RCLError); only real errors get the thread traceback.
            if self._running and rclpy.ok():
                raise

    def _control_loop(self) -> None:
        while self._running and rclpy.ok():
            if self.get_clock().now().nanoseconds == 0:     # sim time but no /clock yet
                self.get_logger().warn("Waiting for /clock ...", throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue
            obs = self._build_obs()
            if obs is None:
                self.get_logger().warn(
                    "Waiting for joint states and all camera frames ...",
                    throttle_duration_sec=2.0,
                )
                time.sleep(0.1)
                continue

            t0 = time.monotonic()
            try:
                result = self._policy.infer(obs)
            except Exception as exc:
                self.get_logger().error(f"Inference error: {exc}")
                time.sleep(0.2)
                continue
            t_infer = time.monotonic() - t0

            # Get server inference time
            server_ms = result.get("server_timing", {}).get("infer_ms")

            actions = np.asarray(result["actions"])
            if actions.ndim != 2 or actions.shape[1] < self._n_arm + 1:
                self.get_logger().error(f"Unexpected action shape: {actions.shape}")
                continue
            horizon = actions.shape[0]

            if not self._send_arm_chunk(actions):
                time.sleep(0.2)
                continue

            # Step the chunk's per-timestep gripper output at control_hz, paced on the
            # SIM clock (get_clock honors use_sim_time) so it advances in lock-step with
            # the JTC, which plays the chunk's time_from_start on the sim clock. Wall pacing
            # here outruns the arm whenever RTF < 1: the loop finishes a wall-second early and
            # the next goal preempts the still-running chunk, so the arm barely moves.
            clock = self.get_clock()
            t0_ns = clock.now().nanoseconds
            step_ns = int(self._dt * 1e9)
            for i in range(horizon):
                self._update_gripper(float(actions[i, self._n_arm]))
                target_ns = t0_ns + (i + 1) * step_ns
                while self._running and rclpy.ok() and clock.now().nanoseconds < target_ns:
                    time.sleep(0.002)

            # Gate the next inference on the arm trajectory actually finishing.
            if not self._arm_done.wait(timeout=max(2.0, horizon * self._dt + 1.0)):
                self.get_logger().warn("Arm trajectory did not complete in time.")

            if server_ms is not None:
                timing = (
                    f"round-trip latency {t_infer * 1e3:.1f} ms "
                    f"(server latency {server_ms:.1f} ms + transport latency{t_infer * 1e3 - server_ms:.1f} ms)"
                )
            else:
                timing = f"round-trip latency{t_infer * 1e3:.1f} ms"
            self.get_logger().info(
                f"Cycle done | {timing} | horizon {horizon} "
                f"({horizon * self._dt:.2f} s)"
            )

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        self._arm_done.set()  # unblock a pending trajectory wait
        # Let the loop thread exit before the handles go away (a daemon thread still using
        # them during interpreter teardown can abort it)
        self._loop_thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyClientSynchronousNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        # Ctrl-C can surface as RCLError ("context is not valid") from the executor
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
