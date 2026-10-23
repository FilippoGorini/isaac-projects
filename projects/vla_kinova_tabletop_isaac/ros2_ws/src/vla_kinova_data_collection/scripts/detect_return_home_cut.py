#!/usr/bin/env python3
"""H1 phase-1: detect the return-home cut point per episode (report only, no writes).

The return-home is the stereotyped tail every episode ends with. We cut each episode at the
FINAL gripper release (last closed->open command = object placed) + a settle margin; everything
after that is the arm travelling back to home and is dropped in phase 2.

Use the gripper COMMAND (action[6]), not the measured state: the command is the operator's
discrete open/close target -- perfectly binary (0.0 open / 0.8 close), independent of object
thickness -- so a single 0.4 midpoint is unambiguous. The measured state instead ramps and
plateaus at a thickness-dependent value (pen ~0.8, cube ~0.3), which is why it needed awkward
low thresholds. Detect by gripper, NOT pose distance: episode STARTS are also near home, so a
distance-to-home trigger would clip the start.

Margin: the command-open edge LEADS the physical release by ~17 frames (measured median; p90
20), so the margin must cover that lag plus a small settle before the tail is dropped -- default
30 frames (~1.0s). Raise it if a spot-check shows a cut landing mid-release.

Run with the lerobot-edit venv:
  datasets/lerobot-edit-venv/bin/python detect_return_home_cut.py <dataset_root> [--out cuts.csv]
  datasets/lerobot-edit-venv/bin/python detect_return_home_cut.py --selftest
"""
import argparse
import glob
import sys

import numpy as np
import pandas as pd

GRIPPER_DIM = 6      # robotiq_85_left_knuckle_joint
CHANNEL = "action"   # use the command, not observation.state (binary 0/0.8, thickness-independent)


def last_release(g, open_thr, close_thr):
    """Frame index of the last closed->open transition, or None if never closed then opened.

    Hysteresis: grippers start open; flip to 'closed' above close_thr, back to 'open' below
    open_thr. The returned index is the frame at which the final open completes (release done).
    """
    state = "open"
    last = None
    for i, v in enumerate(g):
        if state == "open" and v > close_thr:
            state = "closed"
        elif state == "closed" and v < open_thr:
            state = "open"
            last = i
    return last


def analyse(root, open_thr, close_thr, margin, fps, channel=CHANNEL):
    data_files = glob.glob(f"{root}/data/**/*.parquet", recursive=True)
    df = pd.concat(
        [pd.read_parquet(f, columns=["episode_index", "frame_index", channel]) for f in data_files],
        ignore_index=True,
    )
    df["g"] = np.stack(df[channel].values)[:, GRIPPER_DIM]
    meta = pd.concat(
        [pd.read_parquet(f, columns=["episode_index", "tasks", "length"])
         for f in glob.glob(f"{root}/meta/episodes/**/*.parquet", recursive=True)],
        ignore_index=True,
    )
    tasks = dict(zip(meta.episode_index, (t[0] for t in meta.tasks)))

    rows = []
    for ep, grp in df.sort_values("frame_index").groupby("episode_index"):
        g = grp["g"].values
        n = len(g)
        rel = last_release(g, open_thr, close_thr)
        if rel is None:
            cut, new_len, flag = n - 1, n, "no_release"
        else:
            cut = min(rel + margin, n - 1)
            new_len = cut + 1
            flag = "" if (n - new_len) >= margin else "little_trim"  # release already near the end
        removed = n - new_len
        rows.append(dict(
            episode=int(ep), length=n,
            release_frame=(-1 if rel is None else rel),
            release_sec=(-1.0 if rel is None else round(rel / fps, 2)),
            cut_frame=cut, cut_sec=round(cut / fps, 2), new_length=new_len,
            frames_removed=removed, sec_removed=round(removed / fps, 2),
            pct_removed=round(100 * removed / n, 1),
            flag=flag, task=tasks.get(ep, ""),
        ))
    return pd.DataFrame(rows).sort_values("episode").reset_index(drop=True)


def _selftest():
    ot, ct = 0.4, 0.4  # the real defaults; command signal is binary 0/0.8
    o, c = 0.0, 0.8
    # open -> grasp -> release -> travel -> grasp again -> release near end
    trace = [o] * 5 + [c] * 10 + [o] * 20 + [c] * 8 + [o] * 4
    assert last_release(trace, ot, ct) == 43, last_release(trace, ot, ct)
    # single clean place
    assert last_release([o] * 3 + [c] * 6 + [o] * 6, ot, ct) == 9
    # never opens after closing -> None (the 262/460 trimmed case: reopen was already cut off)
    assert last_release([o] * 5 + [c] * 10, ot, ct) is None
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", nargs="?", help="dataset root (contains data/ and meta/)")
    ap.add_argument("--open-thr", type=float, default=0.4, help="command below this = open")
    ap.add_argument("--close-thr", type=float, default=0.4, help="command above this = close (signal is binary 0/0.8)")
    ap.add_argument("--margin", type=int, default=30, help="frames kept after the open command; covers the ~17f command->release lag + settle (30 = 1.0s @30fps)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--channel", default=CHANNEL, help="gripper source: 'action' (command, default) or 'observation.state'")
    ap.add_argument("--out", default=None, help="write full per-episode CSV here")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        _selftest()
        return
    if not a.root:
        ap.error("dataset root required (or --selftest)")

    r = analyse(a.root, a.open_thr, a.close_thr, a.margin, a.fps, a.channel)
    tot = r.length.sum()
    print(f"episodes {len(r)}  frames {tot:,} -> {r.new_length.sum():,} "
          f"(removing {r.frames_removed.sum():,} = {100*r.frames_removed.sum()/tot:.1f}%)")
    print(f"return-home removed per ep (s): median {r.sec_removed.median():.2f}  "
          f"mean {r.sec_removed.mean():.2f}  p90 {r.sec_removed.quantile(.9):.2f}  max {r.sec_removed.max():.2f}")
    flagged = r[r.flag != ""]
    print(f"flagged: {len(flagged)}  ({', '.join(f'ep{e}:{f}' for e, f in zip(flagged.episode, flagged.flag)) or 'none'})")
    pd.set_option("display.max_colwidth", 45, "display.width", 200)
    cols = ["episode", "length", "release_sec", "cut_sec", "new_length", "sec_removed", "flag", "task"]
    print("\n-- smallest trims (inspect these: release near episode end) --")
    print(r.nsmallest(12, "frames_removed")[cols].to_string(index=False))
    print("\n-- largest trims --")
    print(r.nlargest(8, "frames_removed")[cols].to_string(index=False))
    if a.out:
        r.to_csv(a.out, index=False)
        print(f"\nwrote {a.out}")


if __name__ == "__main__":
    sys.exit(main())
