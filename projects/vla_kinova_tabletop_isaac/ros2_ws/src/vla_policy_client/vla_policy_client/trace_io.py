"""Shared reader for the VLA asynchronous policy-client debug trace.

The client writes an append-only, line-buffered JSONL file (one JSON object per line, so
a hard Ctrl-C never corrupts more than the last line) when its `debug_log_dir` param is
set. This module is the SINGLE place that knows the trace schema, so the plotter, the
NPZ converter, and future thesis analysis all load a run the same way:

    from vla_policy_client.trace_io import load_run
    run = load_run("path/to/vla_async_<ts>.jsonl")
    run.js_pos          # (N, 6) measured arm positions [rad]
    run.chunk_raw[0]    # (H, 6) first raw model action chunk (arm)
    t, pos = run.executed_plan(joint=2)   # the 30 Hz interpolated plan actually streamed

Record types (JSON field `type`), one per line:

  meta   once at start -- control_hz, arm_mode, blend_steps, ensemble_decay,
         ensemble_blend_steps, inference_hz, t_start.
  chunk  one per inference cycle:
           t_obs      ROS time [s] the observation was captured. Raw-chunk anchor:
                      raw point i is scheduled for t_obs + i/control_hz.
           t_arrive   ROS time [s] the chunk arrived. Blended-plan anchor.
           d          stale-prefix length dropped (points eaten by inference latency).
           horizon    model chunk length H.
           raw        (H, 6) raw model arm actions.
           blended    (M, 6) the 30 Hz arm plan actually adopted (interp mode: stale
                      prefix dropped + smoothstep cross-fade into the previous plan;
                      ensemble mode: temporal-ensembled across buffered chunks).
           grip_raw   (H,) raw model gripper action (col 6), same anchoring as `raw`.
           grip_blended (M,) gripper column of the adopted plan (same anchoring as `blended`).
  js     measured /joint_states, throttled to js_log_hz: t, arm (6), [grip], [vel (6)], [eff (6)].
  gripper one per gate decision (the discrete OPEN/CLOSE goals the node actually dispatches):
           t, cmd ("OPEN"/"CLOSE"), position (commanded), value (model gripper value that
           tripped the gate), dispatched (bool -- was the action server reachable).
  rate   periodic achieved inference rate: t, hz, target_hz.
"""

from __future__ import annotations

import json
from dataclasses import dataclass

import numpy as np

# fallback for traces written before the client logged meta["arm_joints"]
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


@dataclass
class Run:
    """One loaded trace. Times are absolute ROS seconds; subtract `t0` to normalize."""

    meta: dict
    # per-inference chunks (ragged: horizon varies with the dropped stale prefix)
    chunk_t_obs: np.ndarray       # (C,)
    chunk_t_arrive: np.ndarray    # (C,)
    chunk_d: np.ndarray           # (C,) int
    chunk_horizon: np.ndarray     # (C,) int
    chunk_raw: list               # C arrays, each (H_c, 6)
    chunk_blended: list           # C arrays, each (M_c, 6)
    chunk_sent_prefix: list       # C entries: (d, 6) committed prefix SENT to the server, or
    #                               None (first cycle / non-RTC / pre-logging traces)
    chunk_grip_raw: list          # C arrays, each (H_c,) -- raw model gripper action, or None
    chunk_grip_blended: list      # C arrays, each (M_c,) -- adopted gripper column, or None
    # measured joint states
    js_t: np.ndarray              # (N,)
    js_pos: np.ndarray            # (N, 6)
    js_grip: np.ndarray | None    # (N,) measured gripper position, or None (pre-logging traces)
    js_vel: np.ndarray | None     # (N, 6) or None
    js_eff: np.ndarray | None     # (N, 6) or None
    # gripper gate decisions (discrete OPEN/CLOSE goals actually dispatched)
    grip_evt_t: np.ndarray        # (G,)
    grip_evt_cmd: list            # (G,) "OPEN"/"CLOSE" strings
    grip_evt_position: np.ndarray  # (G,) commanded position
    grip_evt_value: np.ndarray    # (G,) model gripper value that tripped the gate
    grip_evt_dispatched: np.ndarray  # (G,) bool
    # inference-rate reports
    rate_t: np.ndarray            # (R,)
    rate_hz: np.ndarray           # (R,)

    @property
    def arm_joints(self) -> list:
        return list(self.meta.get("arm_joints", ARM_JOINTS))

    @property
    def control_hz(self) -> float:
        return float(self.meta.get("control_hz", 30.0))

    @property
    def dt(self) -> float:
        return 1.0 / self.control_hz

    @property
    def n_chunks(self) -> int:
        return len(self.chunk_raw)

    @property
    def t0(self) -> float:
        """Earliest timestamp in the run (normalize the time axis by subtracting this)."""
        cands = []
        if len(self.js_t):
            cands.append(self.js_t.min())
        if len(self.chunk_t_obs):
            cands.append(self.chunk_t_obs.min())
        return float(min(cands)) if cands else 0.0

    def _used_len(self, ci: int) -> int:
        """Knots of chunk ci's blended plan executed before the next chunk replaces it.
        The last chunk runs to the end of its lookahead."""
        bl = self.chunk_blended[ci]
        if ci + 1 >= self.n_chunks:
            return len(bl)
        span = self.chunk_t_arrive[ci + 1] - self.chunk_t_arrive[ci]
        # -1e-6: anchor spacing is an integer number of grid steps, but float64 makes span/dt land
        # at e.g. 8.0000001, so a bare ceil jitters between 8 and 9. Snap it to the true integer.
        return max(1, min(len(bl), int(np.ceil(span / self.dt - 1e-6))))

    def executed_plan(self, joint: int):
        """The 30 Hz interpolated plan actually streamed to the JTC: each chunk's used
        segment (before the next chunk arrives) stitched together. Returns (t, pos) in
        absolute ROS time for arm `joint` (0-5)."""
        t, p = [], []
        for ci in range(self.n_chunks):
            bl = self.chunk_blended[ci]
            u = self._used_len(ci)
            t.extend((self.chunk_t_arrive[ci] + np.arange(u) * self.dt).tolist())
            p.extend(bl[:u, joint].tolist())
        return np.asarray(t), np.asarray(p)

    def executed_gripper(self):
        """The adopted 30 Hz gripper command trajectory (grip_blended), each chunk's used
        segment stitched together the same way executed_plan does for the arm. Returns
        (t, value) in absolute ROS time, or (empty, empty) if no gripper columns were logged.
        This is the value the gate reads -- compare against the discrete OPEN/CLOSE goals in
        grip_evt_* to see whether a reopen was ever commanded but never crossed the threshold."""
        t, g = [], []
        for ci in range(self.n_chunks):
            gb = self.chunk_grip_blended[ci] if ci < len(self.chunk_grip_blended) else None
            if gb is None:
                continue
            u = self._used_len(ci)
            t.extend((self.chunk_t_arrive[ci] + np.arange(u) * self.dt).tolist())
            g.extend(np.asarray(gb)[:u].tolist())
        return np.asarray(t), np.asarray(g)


def load_run(path: str) -> Run:
    """Parse a JSONL trace into a Run. Tolerates a truncated final line."""
    meta: dict = {"control_hz": 30.0}
    t_obs, t_arr, d, hor, raw, blended, sent_prefix = [], [], [], [], [], [], []
    grip_raw, grip_blended = [], []
    js_t, js_pos, js_grip, js_vel, js_eff = [], [], [], [], []
    ge_t, ge_cmd, ge_pos, ge_val, ge_disp = [], [], [], [], []
    rate_t, rate_hz = [], []
    have_vel = have_eff = have_grip = False
    n_arm = len(ARM_JOINTS)

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue  # truncated last line after a hard kill -- ignore it
            tp = r.get("type")
            if tp == "meta":
                meta.update(r)
                n_arm = len(meta.get("arm_joints", ARM_JOINTS))
            elif tp == "chunk":
                t_obs.append(r["t_obs"])
                t_arr.append(r["t_arrive"])
                d.append(int(r.get("d", 0)))
                hor.append(int(r.get("horizon", 0)))
                raw.append(np.asarray(r["raw"], dtype=float))
                blended.append(np.asarray(r["blended"], dtype=float))
                sp = r.get("sent_prefix")
                sent_prefix.append(np.asarray(sp, dtype=float) if sp is not None else None)
                gr = r.get("grip_raw")
                grip_raw.append(np.asarray(gr, dtype=float) if gr is not None else None)
                gb = r.get("grip_blended")
                grip_blended.append(np.asarray(gb, dtype=float) if gb is not None else None)
            elif tp == "js":
                js_t.append(r["t"])
                js_pos.append(r["arm"])
                if "grip" in r:
                    have_grip = True
                js_grip.append(r.get("grip", np.nan))
                if "vel" in r:
                    have_vel = True
                js_vel.append(r.get("vel", [np.nan] * n_arm))
                if "eff" in r:
                    have_eff = True
                js_eff.append(r.get("eff", [np.nan] * n_arm))
            elif tp == "gripper":
                ge_t.append(r["t"])
                ge_cmd.append(r.get("cmd", ""))
                ge_pos.append(r.get("position", np.nan))
                ge_val.append(r.get("value", np.nan))
                ge_disp.append(bool(r.get("dispatched", False)))
            elif tp == "rate":
                rate_t.append(r["t"])
                rate_hz.append(r.get("hz", np.nan))

    def arr(x, w=n_arm):
        return np.asarray(x, dtype=float) if len(x) else np.zeros((0, w))

    return Run(
        meta=meta,
        chunk_t_obs=np.asarray(t_obs, dtype=float),
        chunk_t_arrive=np.asarray(t_arr, dtype=float),
        chunk_d=np.asarray(d, dtype=int),
        chunk_horizon=np.asarray(hor, dtype=int),
        chunk_raw=raw,
        chunk_blended=blended,
        chunk_sent_prefix=sent_prefix,
        chunk_grip_raw=grip_raw,
        chunk_grip_blended=grip_blended,
        js_t=np.asarray(js_t, dtype=float),
        js_pos=arr(js_pos),
        js_grip=np.asarray(js_grip, dtype=float) if have_grip else None,
        js_vel=arr(js_vel) if have_vel else None,
        js_eff=arr(js_eff) if have_eff else None,
        grip_evt_t=np.asarray(ge_t, dtype=float),
        grip_evt_cmd=ge_cmd,
        grip_evt_position=np.asarray(ge_pos, dtype=float),
        grip_evt_value=np.asarray(ge_val, dtype=float),
        grip_evt_dispatched=np.asarray(ge_disp, dtype=bool),
        rate_t=np.asarray(rate_t, dtype=float),
        rate_hz=np.asarray(rate_hz, dtype=float),
    )


def save_npz(run: Run, path: str) -> None:
    """Write a Run to a compressed .npz of named arrays for thesis analysis. The ragged
    per-chunk lists are stored as object arrays, so load with allow_pickle=True:

        d = np.load(path, allow_pickle=True)
        meta = json.loads(str(d["meta"]))
        js_pos = d["js_pos"]                 # (N, 6)
        raw0 = d["chunk_raw"][0]             # (H, 6)
    """
    np.savez_compressed(
        path,
        meta=json.dumps(run.meta),
        chunk_t_obs=run.chunk_t_obs,
        chunk_t_arrive=run.chunk_t_arrive,
        chunk_d=run.chunk_d,
        chunk_horizon=run.chunk_horizon,
        chunk_raw=np.array(run.chunk_raw, dtype=object),
        chunk_blended=np.array(run.chunk_blended, dtype=object),
        chunk_sent_prefix=np.array(run.chunk_sent_prefix, dtype=object),
        chunk_grip_raw=np.array(run.chunk_grip_raw, dtype=object),
        chunk_grip_blended=np.array(run.chunk_grip_blended, dtype=object),
        js_t=run.js_t,
        js_pos=run.js_pos,
        js_grip=np.zeros((0,)) if run.js_grip is None else run.js_grip,
        js_vel=np.zeros((0, len(run.arm_joints))) if run.js_vel is None else run.js_vel,
        js_eff=np.zeros((0, len(run.arm_joints))) if run.js_eff is None else run.js_eff,
        grip_evt_t=run.grip_evt_t,
        grip_evt_cmd=np.array(run.grip_evt_cmd, dtype=object),
        grip_evt_position=run.grip_evt_position,
        grip_evt_value=run.grip_evt_value,
        grip_evt_dispatched=run.grip_evt_dispatched,
        rate_t=run.rate_t,
        rate_hz=run.rate_hz,
    )
