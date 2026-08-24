#!/usr/bin/env python3
"""Asynchronous RTC VLA policy client for the Kinova Gen3 -- GRID-anchored, RTC-only.

Everything lives on a single shared 30 Hz integer timeline. There is one origin `t0` and grid
step `n` is the instant `t0 + n*dt`; the active plan is stored as knots + the grid index of
knot 0, so "where are we in the plan" is an INTEGER index, never a fractional wall-clock
sample. This removes the round()/interpolate ambiguity the wall-clock variant needs:

  * Observation capture is GRID-LOCKED: the inference loop blocks until the next scheduled grid
    tick, then snapshots the sensors and queries the server, so `t_obs` is grid-aligned and
    `n_obs` is exact (no ceil rounding). The state and the committed prefix then share the grid
    phase, killing the sub-step state/prefix mismatch. (Images stay whatever-latest -- cameras
    publish asynchronously -- same as how the dataset was recorded.) The replan cadence is thus
    quantized to the grid: the next launch is the first tick >= inference_period later.
  * RTC committed prefix = an integer SLICE of the active plan at the grid steps the arm will
    execute during inference (steps [n_obs, n_obs+d), starting at the launch step). The RTC model
    hard-pins the returned chunk's first d steps to it, so returned raw[0:d] == the committed
    prefix exactly, by construction -- no sampling error to correct.
  * The chunk is anchored at "now" (n_arrive) after dropping only the steps that actually
    elapsed during inference. On time (chunk back within d steps) it is [not-yet-elapsed pinned
    prefix][RTC-coherent fresh continuation] -- both seamless -- so it is swapped in CLEANLY,
    no fade (fading toward the old chunk's stale continuation would only drag the RTC-smooth
    plan backward). Only on an OVERRUN (chunk later than d steps, so the arm ran off the
    committed prefix onto the old stale continuation) is the fresh chunk smoothstep-crossfaded
    into the old plan over `rtc_blend_steps`, integer-aligned on the grid, starting as soon as
    the chunk arrives.
  * The gripper consumer reads the plan's gripper column by the same integer grid index.

This client is RTC-ONLY: it always sends the committed prefix + fixed delay and needs the
RTC-finetuned checkpoint (pi05_kinova_finetune_rtc). For standard (non-RTC) inference, or the
ACT-style ensemble / wall-clock behaviour, use policy_client_asynchronous_walltime.

The arm is still shaped + streamed by the C++ vla_kinova_jtc_streamer/jtc_stream_node (this
node only publishes the 30 Hz knots on `plan_topic`); the grid just defines the knots the
streamer interpolates.

Debug trace: set `debug_log_dir` for an append-only JSONL (Ctrl-C safe); schema/loader in
vla_policy_client/trace_io.py, plot with `plot_vla_trace`. Times are logged grid-aligned
(t_obs = t0 + n_obs*dt, t_arrive = plan anchor), so raw[d:] and the blended plan overlap
exactly in the plots.
"""

import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from openpi_client import image_tools
    from openpi_client.websocket_client_policy import WebsocketClientPolicy
except ImportError as e:
    raise SystemExit(
        f"Failed to import openpi_client ({e}). Run: pip install openpi-client"
    ) from e

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"


def _ros_image_to_uint8_hwc(msg: Image) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    if msg.encoding == "bgr8":
        return data[:, :, ::-1].copy()
    return data[:, :, :3].copy() if msg.encoding not in ("rgb8", "8UC3") else data.copy()


class PolicyClientAsynchronousRtcNode(Node):
    def __init__(self):
        super().__init__("policy_client_asynchronous_rtc")

        # --- parameters ---
        self.declare_parameter("policy_host", "localhost")
        self.declare_parameter("policy_port", 8000)
        self.declare_parameter("prompt", "lift the blue cube")
        self.declare_parameter("control_hz", 30.0)     # dataset fps => 30 Hz grid spacing
        self.declare_parameter("inference_hz", 1.5)    # 0.0 => free-running
        # RTC (real-time chunking) is ALWAYS on in this node: each observation carries the
        # committed action prefix + a fixed inference delay d, so the RTC-finetuned model
        # (pi05_kinova_finetune_rtc) hard-pins the new chunk's first d steps to the plan knots the
        # arm is executing -- a seamless continuation. Needs the RTC checkpoint.
        # d: committed-prefix length in grid steps. ~140 ms at 30 Hz => 5. Max 7:
        # pi05_kinova_finetune_rtc trained on randint(0, max_delay=8) = delays 0..7. Choose d in
        # slight excess of the real latency so the next chunk is back before the prefix runs out.
        self.declare_parameter("rtc_delay_steps", 5)
        # RTC safety blend: short smoothstep seam absorbing a latency overrun (chunk later than d
        # steps) or imperfect pinning. Small -- RTC handles the bulk; 0 => pure RTC.
        self.declare_parameter("rtc_blend_steps", 3)
        self.declare_parameter("use_sim", True)

        # Gripper loop rate; default = the 30 Hz grid rate (one VLA gripper knot per tick).
        self.declare_parameter("gripper_hz", 30.0)

        # Gripper hysteresis (near-binary model output: 0 open / ~0.8 closed).
        self.declare_parameter("gripper_close_threshold", 0.55)
        self.declare_parameter("gripper_open_threshold", 0.30)
        self.declare_parameter("gripper_closed_position", 0.8)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_max_effort", 50.0)

        # Camera topics (use_sim picks the pair). /joint_states is read by joint name, so its
        # scrambled order is irrelevant.
        self.declare_parameter("sim_base_image_topic", "/isaac_external_camera/color/image_raw")
        self.declare_parameter("sim_wrist_image_topic", "/isaac_wrist_camera/color/image_raw")
        self.declare_parameter("real_base_image_topic", "/realsense/d435/color/image_raw")
        self.declare_parameter("real_wrist_image_topic", "/camera/color/image_raw")
        self.declare_parameter("joint_states_topic", "/joint_states")
        # 30 Hz plan output: the C++ jtc_stream_node subscribes here, shapes it, and streams to
        # the JTC. This node does NOT publish to the JTC directly.
        self.declare_parameter("plan_topic", "/vla_arm_plan")
        self.declare_parameter("gripper_action", "/robotiq_gripper_controller/gripper_cmd")

        self.declare_parameter("resize_images", True)
        self.declare_parameter("image_resolution", 224)

        # Debug trace (append-only JSONL; "" => off). js_log_hz throttles the measured-state
        # records; rate_report_sec = period of the live [rate] readouts.
        self.declare_parameter("debug_log_dir", "")
        self.declare_parameter("js_log_hz", 100.0)
        self.declare_parameter("rate_report_sec", 2.0)

        gp = self.get_parameter
        host = gp("policy_host").get_parameter_value().string_value
        port = gp("policy_port").get_parameter_value().integer_value
        self._prompt = gp("prompt").get_parameter_value().string_value
        self._dataset_hz = gp("control_hz").get_parameter_value().double_value
        self._infer_hz = gp("inference_hz").get_parameter_value().double_value
        self._rtc_delay = gp("rtc_delay_steps").get_parameter_value().integer_value
        self._rtc_blend_steps = gp("rtc_blend_steps").get_parameter_value().integer_value
        use_sim = gp("use_sim").get_parameter_value().bool_value

        self._gripper_hz = max(gp("gripper_hz").get_parameter_value().double_value, 1.0)

        self._close_thr = gp("gripper_close_threshold").get_parameter_value().double_value
        self._open_thr = gp("gripper_open_threshold").get_parameter_value().double_value
        self._grip_closed_pos = gp("gripper_closed_position").get_parameter_value().double_value
        self._grip_open_pos = gp("gripper_open_position").get_parameter_value().double_value
        self._grip_effort = gp("gripper_max_effort").get_parameter_value().double_value

        js_topic = gp("joint_states_topic").get_parameter_value().string_value
        plan_topic = gp("plan_topic").get_parameter_value().string_value
        gripper_action = gp("gripper_action").get_parameter_value().string_value

        self._resize_images = gp("resize_images").get_parameter_value().bool_value
        self._image_res = gp("image_resolution").get_parameter_value().integer_value

        js_log_hz = gp("js_log_hz").get_parameter_value().double_value
        self._js_log_period = 1.0 / js_log_hz if js_log_hz > 0 else 0.0   # 0 => every sample
        self._rate_report_sec = gp("rate_report_sec").get_parameter_value().double_value

        if use_sim:
            base_topic = gp("sim_base_image_topic").get_parameter_value().string_value
            wrist_topic = gp("sim_wrist_image_topic").get_parameter_value().string_value
        else:
            base_topic = gp("real_base_image_topic").get_parameter_value().string_value
            wrist_topic = gp("real_wrist_image_topic").get_parameter_value().string_value

        if self._rtc_delay < 1:
            raise SystemExit("rtc_delay_steps must be >= 1.")
        if self._rtc_delay > 7:
            self.get_logger().warn(
                f"rtc_delay_steps={self._rtc_delay} is out of the model's trained range: "
                "pi05_kinova_finetune_rtc sampled delays randint(0, max_delay=8) = 0..7, "
                "so d>7 is out of distribution.")

        self._dt_dataset = 1.0 / self._dataset_hz
        self._infer_period = (1.0 / self._infer_hz) if self._infer_hz > 0.0 else 0.0
        self._gripper_period = 1.0 / self._gripper_hz
        self._gripper_closed = False

        infer_str = "free-running" if self._infer_period == 0.0 else f"{self._infer_hz:.2f} Hz"
        self.get_logger().info(
            f"Mode: {'SIM' if use_sim else 'REAL'} | grid {self._dataset_hz:.0f} Hz | "
            f"inference {infer_str}\n"
            f"  plan -> '{plan_topic}' (shaped + streamed by the C++ jtc_stream_node) | "
            f"gripper loop {self._gripper_hz:.0f} Hz\n"
            f"  base='{base_topic}' wrist='{wrist_topic}' js='{js_topic}'\n"
            f"  RTC: committed prefix + fixed delay d={self._rtc_delay} | "
            f"safety blend {self._rtc_blend_steps} steps"
        )

        # --- policy server ---
        self.get_logger().info(f"Connecting to server ws://{host}:{port} ...")
        self._policy = WebsocketClientPolicy(host=host, port=port)
        self.get_logger().info("Connected.")

        # --- observation state (guarded by _obs_lock) ---
        self._obs_lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._measured_arm = None
        self._base_image = None
        self._wrist_image = None
        self._missing_status = "initializing"

        # --- shared 30 Hz grid ---
        # Origin of the integer timeline; grid step n is the instant t0 + n*dt. Set lazily at
        # the first observation so the grid starts with the (sim) clock actually running.
        self._t0: float | None = None
        # Grid step to launch the next inference on (obs capture is locked to this tick).
        self._n_launch: int = 0
        # Active plan (guarded by _plan_lock): (knots (M,7), n0) -- knot k is at grid step n0+k.
        self._plan_lock = threading.Lock()
        self._plan: tuple[np.ndarray, int] | None = None

        # --- trace throttle + rate-meter state ---
        self._last_js_log = 0.0
        self._infer_n = 0
        self._infer_report_wall = time.monotonic()

        # --- pubs / subs / actions ---
        self._plan_pub = self.create_publisher(JointTrajectory, plan_topic, 10)
        self._gripper_client = ActionClient(self, GripperCommand, gripper_action)
        self.create_subscription(JointState, js_topic, self._cb_joint_states, 10)
        self.create_subscription(Image, base_topic, self._cb_base_image, 2)
        self.create_subscription(Image, wrist_topic, self._cb_wrist_image, 2)

        # --- debug JSONL logging (opt-in, append-only => Ctrl-C safe) ---
        self._log_lock = threading.Lock()
        self._log_fh = None
        dbg_dir = gp("debug_log_dir").get_parameter_value().string_value
        if dbg_dir:
            # expanduser: a launch arg like debug_log_dir:=~/x isn't tilde-expanded by the shell
            # (not at word start), so os.makedirs would make a literal "~" directory.
            dbg_dir = os.path.expanduser(dbg_dir)
            os.makedirs(dbg_dir, exist_ok=True)
            path = os.path.join(dbg_dir, f"vla_async_{int(time.time())}.jsonl")
            self._log_fh = open(path, "w", buffering=1)  # line-buffered
            self._log_event({"type": "meta", "node": "asynchronous_rtc", "clock": "grid",
                             "control_hz": self._dataset_hz, "inference_hz": self._infer_hz,
                             "rtc_prefix": True, "rtc_delay": self._rtc_delay,
                             "rtc_blend_steps": self._rtc_blend_steps,
                             # gripper hysteresis band, so the plotter can draw the thresholds
                             "grip_close_thr": self._close_thr, "grip_open_thr": self._open_thr,
                             "grip_closed_pos": self._grip_closed_pos,
                             "grip_open_pos": self._grip_open_pos,
                             "t_start": self._now()})
            self.get_logger().info(f"Debug trace -> {path}")

        # --- threads: producer (inference) + consumer (gripper) ---
        self._running = True
        self._infer_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._infer_thread.start()
        self._control_thread.start()
        self.get_logger().info("Asynchronous RTC policy client (grid) started.")

    def _now(self) -> float:
        """Current ROS time in seconds (sim time when use_sim_time is set)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # A time exactly on a grid tick can land at 109.9999999 in float64; the epsilon snaps such
    # near-integer values to the tick so floor/ceil don't pick the wrong neighbour (matters when
    # a time IS grid-aligned, e.g. a future grid-locked capture).
    _STEP_EPS = 1e-9

    def _get_last_step(self, t: float) -> int:
        """Grid step at or before ROS time `t` -- the step currently in progress (floor). Use
        for 'now'/arrival: how many controller steps have actually completed, and the anchor
        (<= now => the streamer picks it up seamlessly, never a future-step hold)."""
        return math.floor((t - self._t0) / self._dt_dataset + self._STEP_EPS)

    def _get_next_step(self, t: float) -> int:
        """Grid step at or after ROS time `t` -- the next step to be executed (ceil). Use for
        the observation: the committed prefix must be strictly-future setpoints relative to when
        the obs was captured and the model queried."""
        return math.ceil((t - self._t0) / self._dt_dataset - self._STEP_EPS)

    # ------------------------------------------------------------------
    # Observation callbacks
    # ------------------------------------------------------------------

    def _cb_joint_states(self, msg: JointState) -> None:
        with self._obs_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos
            arm = None
            grip = self._joint_positions.get(GRIPPER_JOINT)
            if all(j in self._joint_positions for j in ARM_JOINTS):
                self._measured_arm = np.array([self._joint_positions[j] for j in ARM_JOINTS])
                arm = self._measured_arm

        # Trace the measured state (outside the lock), throttled to js_log_hz. Capture
        # velocity/effort too when the broadcaster provides them, and the measured gripper
        # position so the gripper can be plotted against its commands just like the arm.
        if arm is None or self._log_fh is None:
            return
        now = self._now()
        if self._js_log_period > 0.0 and now - self._last_js_log < self._js_log_period:
            return
        self._last_js_log = now
        rec = {"type": "js", "t": now, "arm": arm.tolist()}
        if grip is not None:
            rec["grip"] = float(grip)
        if len(msg.velocity) == len(msg.name):
            vmap = {n: v for n, v in zip(msg.name, msg.velocity)}
            rec["vel"] = [vmap[j] for j in ARM_JOINTS]
        if len(msg.effort) == len(msg.name):
            emap = {n: e for n, e in zip(msg.name, msg.effort)}
            rec["eff"] = [emap[j] for j in ARM_JOINTS]
        self._log_event(rec)

    def _cb_base_image(self, msg: Image) -> None:
        arr = _ros_image_to_uint8_hwc(msg)
        with self._obs_lock:
            self._base_image = arr

    def _cb_wrist_image(self, msg: Image) -> None:
        arr = _ros_image_to_uint8_hwc(msg)
        with self._obs_lock:
            self._wrist_image = arr

    def _log_event(self, rec: dict) -> None:
        if self._log_fh is None:
            return
        with self._log_lock:
            try:
                self._log_fh.write(json.dumps(rec) + "\n")
            except Exception:
                pass

    def _maybe_report_infer_rate(self, now_wall: float) -> None:
        """Emit the achieved inference rate (wall clock) every rate_report_sec."""
        elapsed = now_wall - self._infer_report_wall
        if elapsed < self._rate_report_sec or self._infer_n == 0:
            return
        hz = self._infer_n / elapsed
        self.get_logger().info(
            f"[rate] inference {hz:.2f} Hz (target "
            f"{'free-run' if self._infer_period == 0.0 else '%.2f Hz' % self._infer_hz})")
        self._log_event({"type": "rate", "loop": "inference", "t": self._now(),
                         "hz": hz, "target_hz": self._infer_hz})
        self._infer_n = 0
        self._infer_report_wall = now_wall

    def _build_obs(self) -> dict | None:
        with self._obs_lock:
            missing = [j for j in ARM_JOINTS + [GRIPPER_JOINT] if j not in self._joint_positions]
            if missing or self._base_image is None or self._wrist_image is None:
                self._missing_status = (
                    f"joints missing {len(missing)} | "
                    f"base {'OK' if self._base_image is not None else 'WAIT'} | "
                    f"wrist {'OK' if self._wrist_image is not None else 'WAIT'}"
                )
                return None
            joints = np.array([self._joint_positions[j] for j in ARM_JOINTS], dtype=np.float32)
            gripper = np.array([self._joint_positions[GRIPPER_JOINT]], dtype=np.float32)
            base_rgb, wrist_rgb = self._base_image, self._wrist_image

        # Resize outside the lock to keep the callbacks responsive.
        if self._resize_images:
            base_rgb = image_tools.resize_with_pad(base_rgb, self._image_res, self._image_res)
            wrist_rgb = image_tools.resize_with_pad(wrist_rgb, self._image_res, self._image_res)

        return {"joints": joints, "gripper": gripper,
                "base_rgb": np.ascontiguousarray(base_rgb),
                "wrist_rgb": np.ascontiguousarray(wrist_rgb), "prompt": self._prompt}

    # ------------------------------------------------------------------
    # Inference loop (producer): observe -> infer -> update the grid plan
    # ------------------------------------------------------------------

    def _inference_loop(self) -> None:
        while self._running and rclpy.ok():
            try:
                self._run_inference_cycle()
            except Exception as exc:
                # Anomalous data (malformed response, unexpected shape, blend math) must NOT kill
                # this daemon thread -- if it died the executor would keep spinning but publish no
                # new plans and the arm would freeze at the last trajectory (only a stderr
                # traceback, no recovery). Log with traceback, back off, and re-arm the schedule.
                self.get_logger().error(f"Inference cycle error: {exc}", exc_info=True)
                time.sleep(0.5)
                if self._t0 is not None:
                    self._n_launch = self._get_next_step(self._now())

    def _run_inference_cycle(self) -> None:
        # Grid-lock: once the grid is anchored, block until the scheduled launch tick so the obs
        # snapshot + query land ON a grid step -- the state and the committed prefix then share the
        # grid's phase exactly (no ceil rounding on the obs). obs building is only a few ms, well
        # inside a 33 ms step, so this holds tightly.
        if self._t0 is not None:
            self._wait_until_ros(self._t0 + self._n_launch * self._dt_dataset)

        obs = self._build_obs()
        if obs is None:
            self.get_logger().warn(
                f"Waiting for observations... [{self._missing_status}]",
                throttle_duration_sec=2.0)
            time.sleep(0.05)
            if self._t0 is not None:
                self._n_launch = self._get_next_step(self._now())   # retry on the next tick
            return

        if self._t0 is None:                        # anchor the grid to the first observation
            self._t0 = self._now()
            self._n_launch = self._get_next_step(self._now())
        # We launched ON grid step n_launch, so the observation IS that step -- exact, no
        # rounding. Keep the raw ROS time only for the true round-trip latency readout.
        n_obs = self._n_launch
        t_obs_actual = self._now()

        with self._plan_lock:
            plan = self._plan
        first_chunk = plan is None

        # RTC: attach the committed action prefix (an integer slice of the active plan at the grid
        # steps the arm executes during inference) + fixed delay, so the model pins the new chunk's
        # first d steps to it. Skipped on the first cycle (no plan is executing yet).
        sent_prefix = None
        if not first_chunk:
            sent_prefix = self._prefix_from_plan(plan, n_obs, self._rtc_delay)
            obs["action_prefix"] = sent_prefix
            obs["delay"] = int(self._rtc_delay)

        try:
            result = self._policy.infer(obs)
        except Exception as exc:
            self.get_logger().error(f"Inference error: {exc}")
            time.sleep(0.1)
            self._schedule_next(n_obs)
            return
        # Arrival: the number of controller steps that actually completed during inference, so it
        # snaps to the LAST fully-elapsed grid step (floor). This is also the plan anchor
        # (<= now => seamless).
        t_arrive_actual = self._now()
        n_arrive = self._get_last_step(t_arrive_actual)
        self._infer_n += 1
        self._maybe_report_infer_rate(time.monotonic())

        server_ms = result.get("server_timing", {}).get("infer_ms")
        actions = np.asarray(result["actions"])
        if actions.ndim != 2 or actions.shape[1] < 7:
            self.get_logger().error(f"Unexpected action shape: {actions.shape}")
            self._schedule_next(n_obs)
            return
        horizon = actions.shape[0]

        # action i is the action for grid step n_obs + i. Drop only the steps that ACTUALLY
        # elapsed during inference and anchor the plan at "now" (n_arrive) -- never at a future
        # grid step, which would make the streamer hold knot 0 and jump. RTC promised the model a
        # FIXED delay d (it pinned actions[0:d]), but we keep the not-yet-elapsed pinned steps
        # actions[d_elapsed:d] as a seamless lead-in; the fresh content actions[d:] then begins
        # `off` steps into the retained plan -- that is where the old->new cross-fade sits. A
        # latency overrun (d_elapsed > d) collapses off to 0, so the fade eases from the current
        # pose. First chunk: no plan was executing during this inference, so the arm held its
        # startup pose (~ action[0]). Keep the WHOLE chunk and anchor it at arrival so the streamer
        # eases out from action[0] instead of jumping to action[d_elapsed].
        d_elapsed = 0 if first_chunk else max(0, n_arrive - n_obs)
        if d_elapsed >= horizon - 1:
            self.get_logger().warn(
                f"Chunk fully stale (elapsed {d_elapsed} >= horizon {horizon}); skipping.")
            self._schedule_next(n_obs)
            return

        retained = actions[d_elapsed:]                      # knot 0 at grid step n_arrive
        n_start = n_arrive
        if first_chunk:
            # No active plan to splice into; the blend is a no-op anyway.
            off = 0
            K = 0
        elif d_elapsed <= self._rtc_delay:
            # On time: the arm is still inside the committed prefix, so the retained plan is
            # [not-yet-elapsed pinned prefix][RTC-coherent fresh continuation] -- both seamless
            # with what the arm is executing. Swap it in CLEANLY (no fade): blending toward the
            # old chunk's stale continuation would only drag the RTC-smooth plan backward.
            # `off` marks the fresh seam for the trace only.
            off = self._rtc_delay - d_elapsed
            K = 0
        else:
            # Overrun: inference outran the promised d, so the arm ran off the committed prefix
            # onto the OLD chunk's stale continuation. The fresh chunk (from actions[d_elapsed])
            # disagrees with where the arm now is -> cross-fade old -> new from the anchor, as
            # soon as the chunk is available.
            off = 0
            K = self._rtc_blend_steps
        seam_gap = self._seam_gap(retained, n_start, off, plan)
        blended = self._blend_on_grid(retained, n_start, off, plan, K)

        with self._plan_lock:
            self._plan = (blended, n_start)
        self._publish_plan(blended, n_start)

        t_obs = self._t0 + n_obs * self._dt_dataset         # grid-aligned, for the trace
        t_anchor = self._t0 + n_start * self._dt_dataset
        gap = "n/a (first)" if seam_gap is None else f"{seam_gap:.4f} rad"
        timing = (f"round-trip {(t_arrive_actual - t_obs_actual) * 1e3:.1f} ms"
                  f" ({d_elapsed} steps)"
                  + (f" (server {server_ms:.1f} ms)" if server_ms is not None else ""))
        self.get_logger().info(
            f"Chunk | {timing} | dropped {d_elapsed}/{horizon}, kept {len(retained)}, "
            f"seam@{off} | gap {gap}")
        self._log_event({
            "type": "chunk", "t_obs": t_obs, "t_arrive": t_anchor,
            "d": int(d_elapsed), "horizon": int(horizon),
            "raw": actions[:, :6].tolist(), "blended": blended[:, :6].tolist(),
            # Gripper command trajectory (col 6): lets a stuck / never-reopened gripper be
            # diagnosed straight from the trace -- did the model even command open (value below
            # gripper_open_threshold), or did it sit in the hysteresis dead zone?
            "grip_raw": actions[:, 6].tolist(), "grip_blended": blended[:, 6].tolist(),
            # RTC: the committed prefix we SENT (arm cols); on the grid the pin is exact so
            # the plotter's overlay should sit exactly on the returned raw[0:d].
            "sent_prefix": (sent_prefix[:, :6].tolist() if sent_prefix is not None else None)})
        self._schedule_next(n_obs)

    def _wait_until_ros(self, target: float) -> None:
        """Block until ROS time reaches `target`. Polls in small increments so it stays correct
        under sim time -- a plain wall-clock sleep would desync if sim runs off real-time or
        stalls. A grid step (~33 ms) is a handful of polls; between inferences the thread is idle
        anyway."""
        while self._running and rclpy.ok():
            remaining = target - self._now()
            if remaining <= 0.0:
                return
            time.sleep(min(remaining, 0.002))

    def _schedule_next(self, n_obs: int) -> None:
        """Set the grid step to launch the next inference on, computed in STEP space off the exact
        launch tick `n_obs`: `n_obs + period_steps`. Working in integer steps (not a wall-clock
        time) avoids the sub-step slop `t_obs_actual` carries -- the wait wakes a couple ms late,
        plus obs building -- which a ceil-to-tick would otherwise round up into a permanent
        +1-step cadence drift. Clamp to `floor(now)+1`: if the last cycle overran the whole period,
        `n_obs + period_steps` is already in the past, and firing on it would stamp a STALE n_obs;
        firing on the next tick from now instead keeps n_obs matched to the real capture and lets
        the replan just catch up. Free-running (period 0) => the very next tick."""
        if self._infer_period > 0.0:
            period_steps = max(1, round(self._infer_period / self._dt_dataset))
            self._n_launch = max(n_obs + period_steps, self._get_last_step(self._now()) + 1)
        else:
            self._n_launch = self._get_last_step(self._now()) + 1

    # ------------------------------------------------------------------
    # Grid plan bookkeeping: integer-indexed prefix / seam / blend
    # ------------------------------------------------------------------

    def _prefix_from_plan(self, plan, n_obs: int, d: int) -> np.ndarray:
        """RTC committed prefix: the d plan knots (absolute, 6 arm + 1 gripper) the arm will
        execute during the inference delay -- grid steps [n_obs, n_obs+d). A pure integer slice
        of the active plan (clamped at its ends), so it is exactly what the arm executes; the
        model pins the returned chunk's first d steps to it. The server re-anchors/normalizes it
        like the actions, so it goes out RAW."""
        knots, n0 = plan
        m = len(knots)
        idxs = [min(max(n_obs - n0 + i, 0), m - 1) for i in range(d)]
        return np.asarray(knots[idxs, :7], dtype=np.float32)

    def _seam_gap(self, retained_new: np.ndarray, n_start: int, off: int, plan) -> float | None:
        """Largest per-joint disagreement (rad) at the seam -- the new chunk's first FRESH knot
        (retained index `off`, grid step n_start+off) vs the active plan's knot at that same grid
        step, i.e. the old chunk's continuation the blend has to absorb. With RTC landing cleanly
        this is ~0. None if there is no active plan yet."""
        if plan is None or off >= len(retained_new):
            return None
        knots_old, n0_old = plan
        j = min(max(n_start + off - n0_old, 0), len(knots_old) - 1)
        return float(np.max(np.abs(retained_new[off, :6] - knots_old[j, :6])))

    def _blend_on_grid(self, retained_new: np.ndarray, n_start: int, off: int, plan,
                       blend_steps: int) -> np.ndarray:
        """Smoothstep cross-fade the arm columns of `retained_new` (knot 0 at grid step n_start)
        into the OLD plan's continuation, starting at the seam (retained index `off`, the first
        FRESH knot). Weight ramps 0 -> 1 over the fade: at k=0 the output is the old plan (the
        old chunk's step after the committed prefix), at the end it is the new chunk. Retained
        knots before `off` are the not-yet-elapsed pinned prefix (== old plan, seamless) and are
        left untouched. Cubic weight (zero slope at both ends) => velocity-continuous seam. Arm
        columns only; the gripper column stays the new chunk's. No-op with no active plan or
        blend_steps<=0 (e.g. RTC landing exactly => the seam gap is already ~0)."""
        if plan is None or blend_steps <= 0:
            return retained_new
        knots_old, n0_old = plan
        m_old = len(knots_old)
        K = min(blend_steps, len(retained_new) - 1 - off)
        if K <= 0:
            return retained_new
        blended = retained_new.copy()
        for k in range(K + 1):
            ri = off + k                                       # retained knot being faded
            j = min(max(n_start + ri - n0_old, 0), m_old - 1)  # old plan knot at this grid step
            s = k / K
            w = s * s * (3.0 - 2.0 * s)                        # 0 (pure old) -> 1 (pure new)
            blended[ri, :6] = (1.0 - w) * knots_old[j, :6] + w * retained_new[ri, :6]
        return blended

    # ------------------------------------------------------------------
    # 30 Hz plan output (consumed by the C++ jtc_stream_node)
    # ------------------------------------------------------------------

    def _publish_plan(self, knots: np.ndarray, n0: int) -> None:
        """Publish the 30 Hz arm plan as a JointTrajectory on `plan_topic`. header.stamp =
        the grid time of knot 0 (t0 + n0*dt); knot k stamped at k/control_hz. Arm only; the
        gripper column stays in this node."""
        t_ref = self._t0 + n0 * self._dt_dataset
        msg = JointTrajectory()
        msg.header.stamp = Time(nanoseconds=int(t_ref * 1e9)).to_msg()
        msg.joint_names = ARM_JOINTS
        for k in range(len(knots)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in knots[k, :6]]
            elapsed = k * self._dt_dataset
            pt.time_from_start = Duration(sec=int(elapsed),
                                          nanosec=int((elapsed - int(elapsed)) * 1e9))
            msg.points.append(pt)
        self._plan_pub.publish(msg)

    # ------------------------------------------------------------------
    # Gripper loop (consumer): step the plan's gripper column through the gate
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """Light wall-clock daemon: steps the gripper off the grid at gripper_hz. The arm is
        shaped + streamed by the C++ jtc_stream_node, not here."""
        while self._running and rclpy.ok():
            t0 = time.monotonic()
            try:
                self._control_step()
            except Exception as exc:
                self.get_logger().error(f"Gripper step error: {exc}",
                                        throttle_duration_sec=2.0)
            sleep_for = self._gripper_period - (time.monotonic() - t0)
            if sleep_for > 0.0:
                time.sleep(sleep_for)

    def _control_step(self) -> None:
        with self._plan_lock:
            plan = self._plan
        if plan is None or self._t0 is None:
            return
        knots, n0 = plan
        # The step currently in progress (floor) -> the gripper command being executed now.
        idx = min(max(self._get_last_step(self._now()) - n0, 0), len(knots) - 1)
        self._update_gripper(float(knots[idx, 6]))

    # ------------------------------------------------------------------
    # Gripper: hysteresis gate, level-triggered (one goal per state change)
    # ------------------------------------------------------------------

    def _update_gripper(self, value: float) -> None:
        # Flip the internal state ONLY if the goal was actually dispatched. If we flipped first and
        # the dispatch was skipped (server not up), the level-triggered gate would think it already
        # commanded the new state and never retry -- leaving the gripper stuck (e.g. closed on the
        # cube, never reopening).
        if not self._gripper_closed and value > self._close_thr:
            if self._send_gripper_goal(self._grip_closed_pos, value):
                self._gripper_closed = True
        elif self._gripper_closed and value < self._open_thr:
            if self._send_gripper_goal(self._grip_open_pos, value):
                self._gripper_closed = False

    def _send_gripper_goal(self, position: float, value: float) -> bool:
        """Dispatch a gripper go-to goal. Returns True iff it was sent (server available). NOTE:
        this confirms DISPATCH, not server acceptance -- send_goal_async is fire-and-forget."""
        cmd = "CLOSE" if position == self._grip_closed_pos else "OPEN"
        dispatched = self._gripper_client.wait_for_server(timeout_sec=0.0)
        if dispatched:
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = self._grip_effort
            self._gripper_client.send_goal_async(goal)
            self.get_logger().info(f"Gripper -> {cmd} ({position:.2f})")
        else:
            self.get_logger().warn("Gripper action server unavailable; skipping goal.")
        # Trace every gate decision so a never-reopened gripper is diagnosable: which state change
        # fired, at what commanded value, and whether it actually dispatched.
        self._log_event({"type": "gripper", "t": self._now(), "cmd": cmd,
                         "position": float(position), "value": float(value),
                         "dispatched": bool(dispatched)})
        return dispatched

    def destroy_node(self):
        self._running = False
        if self._log_fh is not None:
            try:
                self._log_fh.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PolicyClientAsynchronousRtcNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
