#!/usr/bin/env python3
"""Mock policy server: same websocket protocol as openpi's serve_policy, no GPU / model.

Lets the clients (and everything downstream of them) be tested without the GPU box. The reply
is built from the observation only (D = len(joints) + len(gripper)), so it needs no robot config:

  hold  repeat the current state every chunk -> the arm must not move (safest real-robot test)
  sine  state latched at the connection's first obs (again when an RTC client restarts its
        plan) + amp*sin on one action column. The phase is wall time since that obs, so
        consecutive chunks agree. `--joint -1` puts the sine on the gripper column instead
        (amp ~0.8 crosses the hysteresis band -> gripper gate events)

RTC: if the obs carries `action_prefix` (+ `delay`), actions[:delay] is overwritten with it, like
the RTC model's pin. `--latency-ms` sleeps before replying to emulate inference time.

  ros2 run vela_policy_client mock_policy_server --mode sine --latency-ms 120
"""

import argparse
import logging
import time
import traceback

import numpy as np
from openpi_client import msgpack_numpy
from websockets.exceptions import ConnectionClosed
from websockets.frames import CloseCode
from websockets.sync.server import serve

logger = logging.getLogger("mock_policy_server")


class MockPolicy:
    def __init__(self, args):
        self._args = args
        self._center = None     # sine: state at the first obs
        self._t_start = None
        self._had_prefix = False

    def infer(self, obs: dict) -> dict:
        a = self._args
        t_obs = time.monotonic()
        state = np.concatenate([obs["joints"], obs["gripper"]]).astype(np.float32)
        has_prefix = "action_prefix" in obs
        if a.mode == "hold":
            actions = np.tile(state, (a.horizon, 1))
        else:
            # re-latch when an RTC client starts a new plan (no prefix after prefixed obs, e.g. the
            # prompted client resuming), else the sine restarts away from where the arm stopped
            if self._center is None or (self._had_prefix and not has_prefix):
                self._center, self._t_start = state, t_obs
                logger.info(f"sine center (D={len(state)}): {np.array2string(state, precision=3)}")
            # action i is for t_obs + i/hz
            # ponytail: phase runs on server wall time, so under sim time with RTF < 1 consecutive
            # chunks are slightly offset (small seams); send the client's clock in the obs if it matters
            t = t_obs - self._t_start + np.arange(a.horizon) / a.hz
            actions = np.tile(self._center, (a.horizon, 1))
            actions[:, a.joint] += a.amp * np.sin(2.0 * np.pi * t / a.period)
        self._had_prefix = has_prefix
        if has_prefix:
            prefix = np.asarray(obs["action_prefix"], dtype=np.float32)
            d = int(obs.get("delay", len(prefix)))
            actions[:d] = prefix[:d]
        if a.latency_ms > 0:
            time.sleep(max(0.0, a.latency_ms / 1000.0 - (time.monotonic() - t_obs)))
        return {"actions": actions}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=["hold", "sine"], default="hold")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--horizon", type=int, default=30, help="chunk length (Kinova checkpoints: 30)")
    p.add_argument("--hz", type=float, default=30.0, help="action rate, for the sine phase")
    p.add_argument("--amp", type=float, default=0.05, help="sine amplitude (rad)")
    p.add_argument("--period", type=float, default=4.0, help="sine period (s)")
    p.add_argument("--joint", type=int, default=0,
                   help="action column the sine is applied to (-1 = gripper)")
    p.add_argument("--latency-ms", type=float, default=0.0, help="emulated inference time")
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")

    conns = set()   # open connections, closed on Ctrl-C (handler threads aren't daemons)

    def handler(ws):
        conns.add(ws)
        try:
            serve_connection(ws)
        finally:
            conns.discard(ws)

    def serve_connection(ws):
        peer = ws.remote_address
        logger.info(f"Connection from {peer} opened")
        packer = msgpack_numpy.Packer()
        ws.send(packer.pack({"policy": "mock", "mode": args.mode, "action_horizon": args.horizon}))
        policy = MockPolicy(args)
        n = 0
        while True:
            try:
                obs = msgpack_numpy.unpackb(ws.recv())
                t0 = time.monotonic()
                out = policy.infer(obs)
                out["server_timing"] = {"infer_ms": (time.monotonic() - t0) * 1000}
                ws.send(packer.pack(out))
                n += 1
            except ConnectionClosed:
                logger.info(f"Connection from {peer} closed after {n} chunks")
                break
            except Exception:
                # same as openpi: traceback as a text frame, then close
                ws.send(traceback.format_exc())
                ws.close(code=CloseCode.INTERNAL_ERROR,
                         reason="Internal server error. Traceback included in previous frame.")
                raise

    logger.info(f"mode={args.mode} horizon={args.horizon} latency={args.latency_ms:.0f} ms "
                f"on ws://{args.host}:{args.port}")
    with serve(handler, args.host, args.port, compression=None, max_size=None) as server:
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            for ws in list(conns):
                ws.close()


if __name__ == "__main__":
    main()
