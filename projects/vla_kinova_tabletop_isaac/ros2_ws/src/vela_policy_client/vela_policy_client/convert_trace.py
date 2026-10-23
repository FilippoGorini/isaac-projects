#!/usr/bin/env python3
"""Convert an asynchronous VLA policy-client JSONL trace to a compressed .npz of named
arrays for thesis analysis. Run this OFFLINE after a run (never at node shutdown), so a
Ctrl-C under `ros2 launch` can never truncate it -- the live JSONL is the crash-safe
record, the .npz is the tidy derived artifact.

Usage:
  convert_vla_trace TRACE.jsonl [--out TRACE.npz]

Load the result with:
  import json, numpy as np
  d = np.load("TRACE.npz", allow_pickle=True)
  meta = json.loads(str(d["meta"]))
  js_pos = d["js_pos"]            # (N, 6) measured arm positions
  raw0 = d["chunk_raw"][0]        # (H, 6) first raw model action chunk
"""

import argparse
import os

from vela_policy_client.trace_io import load_run, save_npz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace", help="JSONL trace file")
    ap.add_argument("--out", default=None, help="Output .npz (default: alongside the trace)")
    args = ap.parse_args()

    run = load_run(args.trace)
    out = args.out or (os.path.splitext(args.trace)[0] + ".npz")
    save_npz(run, out)
    print(f"wrote {out}  ({run.n_chunks} chunks, {len(run.js_t)} joint-state samples)")


if __name__ == "__main__":
    main()
