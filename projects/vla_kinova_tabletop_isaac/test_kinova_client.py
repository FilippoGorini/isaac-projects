#!/usr/bin/env python3
"""Kinova-shaped latency benchmark for the pi0.5 policy server.

Mirrors openpi's examples/simple_client/main.py, but sends the EXACT observation
the ROS 2 client sends (KinovaInputs shape) at the exact on-wire sizes, then
reports timing statistics over N requests.

Observation (matches policy_client_synchronous_node._build_obs after resize):
  joints     float32 (6,)
  gripper    float32 (1,)
  base_rgb   uint8   (224, 224, 3)   ~147 KB
  wrist_rgb  uint8   (224, 224, 3)   ~147 KB   -> ~294 KB/request
  prompt     str

Pass --rtc-delay d (>0) to ALSO attach the RTC committed prefix (action_prefix
(d,7) + delay=d), exactly like policy_client_asynchronous_rtc after its first
cycle. This exercises the RTC pin path (Pi0Faster) instead of plain inference, so
the timing matches the real node, and it doubles as a pre-flight check that the
served checkpoint is actually RTC-capable. Prefix bytes are negligible (~d*28 B)
next to the ~294 KB of images, so it does not skew transport time -- it only
confirms the RTC code path costs the same server compute (it should: same denoise
steps, the pin is a cheap masked write).

Two warmup requests (untimed) trigger server-side JIT and warm the TCP window,
matching how the real client reaches steady state. With --rtc-delay the warmup
carries the prefix too, so the RTC-specific jit compilation is triggered untimed.
Use --idle-sec to insert a gap between requests and observe TCP slow-start-after-idle
(should be a no-op if net.ipv4.tcp_slow_start_after_idle=0 is set on this machine).

Usage:
    python3 test_kinova_client.py --host 194.93.48.73 --port 8000 -n 30
    python3 test_kinova_client.py --host 194.93.48.73 --idle-sec 0.5   # mimic node cadence
    python3 test_kinova_client.py --host 194.93.48.73 --rtc-delay 5    # measure the RTC path
"""
import argparse
import statistics
import time

import numpy as np
from openpi_client.websocket_client_policy import WebsocketClientPolicy

IMG_SHAPE = (224, 224, 3)  # matches image_resolution=224 after resize_with_pad


def make_obs(prompt: str, rtc_delay: int = 0) -> dict:
    obs = {
        "joints": np.zeros(6, dtype=np.float32),
        "gripper": np.zeros(1, dtype=np.float32),
        "base_rgb": np.random.randint(0, 256, IMG_SHAPE, dtype=np.uint8),
        "wrist_rgb": np.random.randint(0, 256, IMG_SHAPE, dtype=np.uint8),
        "prompt": prompt,
    }
    if rtc_delay > 0:
        # RTC committed prefix: d raw-absolute setpoints (6 arm rad + 1 gripper), the same shape
        # policy_client_asynchronous_rtc sends via _prefix_from_plan. The values are arbitrary for
        # a latency probe (server compute is prefix-content-independent) -- a gentle ramp off the
        # zero state stands in for a real slice of the committed plan.
        prefix = np.zeros((rtc_delay, 7), dtype=np.float32)
        prefix[:, :6] = np.linspace(0.0, 0.02, rtc_delay, dtype=np.float32)[:, None]
        obs["action_prefix"] = prefix
        obs["delay"] = int(rtc_delay)
    return obs


def _pct(xs: list[float], q: float) -> float:
    return float(np.percentile(xs, q))


def print_stats(name: str, xs: list[float]) -> None:
    print(
        f"{name:<16}"
        f"{statistics.mean(xs):>9.1f}"
        f"{statistics.pstdev(xs):>9.1f}"
        f"{min(xs):>9.1f}"
        f"{_pct(xs, 50):>9.1f}"
        f"{_pct(xs, 90):>9.1f}"
        f"{_pct(xs, 95):>9.1f}"
        f"{_pct(xs, 99):>9.1f}"
        f"{max(xs):>9.1f}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--prompt", default="lift the blue cube")
    ap.add_argument("-n", "--num-steps", type=int, default=20)
    ap.add_argument("--warmup", type=int, default=2, help="Untimed warmup requests")
    ap.add_argument(
        "--idle-sec", type=float, default=0.0,
        help="Sleep between requests; set ~0.5 to mimic the node's per-cycle idle.",
    )
    ap.add_argument(
        "--rtc-delay", type=int, default=0,
        help="If >0, attach the RTC committed prefix (action_prefix (d,7) + delay=d) so the probe "
             "hits the RTC pin path, matching policy_client_asynchronous_rtc. 0 => plain inference.",
    )
    args = ap.parse_args()

    obs = make_obs(args.prompt, args.rtc_delay)
    payload_kb = (obs["base_rgb"].nbytes + obs["wrist_rgb"].nbytes) / 1024
    rtc_str = f"RTC prefix d={args.rtc_delay}" if args.rtc_delay > 0 else "plain (no RTC prefix)"
    print(f"Connecting to ws://{args.host}:{args.port} ...")
    policy = WebsocketClientPolicy(host=args.host, port=args.port)
    print(
        f"Connected. payload {payload_kb:.0f} KB/request "
        f"(2x {IMG_SHAPE} uint8) | {rtc_str} | warmup {args.warmup} | "
        f"steps {args.num_steps} | idle {args.idle_sec:.2f} s"
    )

    for _ in range(args.warmup):
        policy.infer(make_obs(args.prompt, args.rtc_delay))

    round_trip, server, transport = [], [], []
    for i in range(args.num_steps):
        # Fresh pixels each step so nothing is trivially cached.
        obs["base_rgb"] = np.random.randint(0, 256, IMG_SHAPE, dtype=np.uint8)
        obs["wrist_rgb"] = np.random.randint(0, 256, IMG_SHAPE, dtype=np.uint8)

        t0 = time.monotonic()
        result = policy.infer(obs)
        rt = (time.monotonic() - t0) * 1e3

        srv = result.get("server_timing", {}).get("infer_ms")
        round_trip.append(rt)
        if srv is not None:
            server.append(srv)
            transport.append(rt - srv)
        print(
            f"[{i:02d}] round-trip {rt:6.1f} ms"
            + (f" | server {srv:5.1f} ms | transport {rt - srv:6.1f} ms" if srv is not None else "")
        )

        if args.idle_sec > 0.0:
            time.sleep(args.idle_sec)

    print("\n" + "-" * 88)
    header = f"{'metric (ms)':<16}" + "".join(
        f"{h:>9}" for h in ("mean", "std", "min", "p50", "p90", "p95", "p99", "max")
    )
    print(header)
    print("-" * 88)
    print_stats("round_trip", round_trip)
    if server:
        print_stats("server", server)
        print_stats("transport", transport)


if __name__ == "__main__":
    main()
