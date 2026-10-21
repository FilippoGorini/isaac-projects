#!/usr/bin/env python3
"""Per-joint overlay plot for an asynchronous VLA policy-client trace.

For each arm joint (one figure each) overlays, on a shared normalized time axis, the whole
command pipeline this node can see, so you can tell where a wobble enters:

  * raw model chunks   -- orange, faint. Each chunk's full H-step lookahead, anchored at
                          the time it was MEANT to start (t_obs + i*dt). The dropped stale
                          prefix (inference latency) is faint red x's.
  * 30 Hz plan         -- cyan, bold. The interpolated (blended / ensembled) plan actually
                          streamed to the JTC: each chunk's used segment stitched together.
  * measured           -- black. What the arm actually did (/joint_states).

Read it: raw chunks disagree a lot at a seam but the cyan plan stays smooth => it is chunk
DISAGREEMENT the blend/ensemble is absorbing; plan smooth but measured buzzes around it =>
JTC / hardware, not the client.

Usage:
  plot_vla_trace TRACE.jsonl [--joints 0 1 2] [--stride N] [--tmin S] [--tmax S]
                 [--out fig.png] [--no-show] [--no-chunks]
"""

import argparse
import os

import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import numpy as np

from vla_policy_client.trace_io import ARM_JOINTS, load_run


def _pin_residual(run, j):
    """Median |returned raw[i] - sent_prefix[i]| over pinned steps of all RTC chunks, for
    joint j. This is the TRUE pin error the RTC re-freeze should drive to ~0. None if no
    trace has a logged sent prefix (pre-logging runs)."""
    errs = []
    for ci in range(run.n_chunks):
        sp = run.chunk_sent_prefix[ci] if ci < len(run.chunk_sent_prefix) else None
        if sp is None or not len(sp):
            continue
        k = min(len(sp), len(run.chunk_raw[ci]))
        errs.append(np.abs(run.chunk_raw[ci][:k, j] - sp[:k, j]))
    return float(np.median(np.concatenate(errs))) if errs else None


def _plot_chunks_flat(ax, run, j, t0, args):
    """Default style: every chunk in the same faint orange (kept) + red dropped prefix + green
    sent-prefix rings. Compact, but chunks are indistinguishable where they overlap."""
    dt = run.dt
    first = True
    first_sp = True
    for ci in range(run.n_chunks):
        if ci % args.stride:
            continue
        raw = run.chunk_raw[ci]
        d = int(run.chunk_d[ci])
        traw = (run.chunk_t_obs[ci] - t0) + np.arange(len(raw)) * dt
        ax.plot(traw[d:], raw[d:, j], color="tab:orange", lw=0.8, alpha=0.30,
                label="raw chunk (kept)" if first else None, zorder=1)
        if d > 0:
            ax.plot(traw[:d + 1], raw[:d + 1, j], color="tab:red", lw=0.6, alpha=0.30,
                    ls=":", marker="x", ms=3,
                    label="raw chunk (dropped)" if first else None, zorder=1)
        # committed prefix we SENT (RTC): overlay at the same steps as raw[:d] so any
        # gap to the red dropped-prefix is the true pin residual. Anchored like raw.
        sp = run.chunk_sent_prefix[ci] if ci < len(run.chunk_sent_prefix) else None
        if sp is not None and len(sp):
            ax.plot(traw[:len(sp)], sp[:, j], color="tab:green", lw=0, marker="o",
                    ms=4, mfc="none", alpha=0.8,
                    label="sent prefix" if first_sp else None, zorder=5)
            first_sp = False
        first = False


# Repeating palette for --chunk-style cycle: a Tableau-10-based set ordered so consecutive chunks
# always jump hue family (blue->orange->red->green->...), and it repeats every len(_CYCLE) -- far
# more than the handful of chunks that overlap in any zoom window. Gray is omitted so it can't be
# confused with the faint stitched-plan line.
_CYCLE = ["#4E79A7", "#F28E2B", "#E15759", "#59A14F", "#B07AA1",
          "#76B7B2", "#FF9DA7", "#EDC948", "#9C755F", "#D637A8"]


def _plot_chunks_coloured(ax, run, j, t0, args):
    """Readable styles ('time' | 'cycle'): split each chunk's stored plan into the part that
    ACTUALLY RAN (solid, bold) vs the PROVISIONAL TAIL that was overwritten by the next replan
    (dashed, faint). The seam anchor (blend[0] at t_arrive) is a dot -- consecutive dots trace the
    plan's real continuity, while the dashed tails peeling away are the superseded lookahead.
    'time' colours by arrival time (continuous cmap + colorbar); 'cycle' cycles a high-contrast
    palette by chunk order so adjacent chunks are always easy to separate (no colorbar)."""
    dt = run.dt
    tarr = run.chunk_t_arrive - t0
    finite = tarr[np.isfinite(tarr)]
    cyclic = args.chunk_style == "cycle"
    period = args.chunk_cmap_period
    if not cyclic:
        cmap = plt.get_cmap(args.chunk_cmap)
        if period and period > 0:
            # Wrap the gradient every `period` seconds so adjacent chunks stay distinct even on a
            # full-episode view (tune period to taste).
            norm = Normalize(0.0, period)
            colf = lambda ta: cmap(norm(ta % period))
            cb_label = f"chunk time mod {period:g} s"
        else:
            # Spread the gradient once over the zoom window (if given) else the whole run.
            lo = args.tmin if args.tmin is not None else (finite.min() if len(finite) else 0.0)
            hi = args.tmax if args.tmax is not None else (finite.max() if len(finite) else 1.0)
            norm = Normalize(lo, hi)
            colf = lambda ta: cmap(norm(ta))
            cb_label = "chunk arrival time [s]"
    le = lt = la = lsp = lobs = True
    k = 0                                                # running index over PLOTTED chunks
    for ci in range(run.n_chunks):
        if ci % args.stride:
            continue
        bl = run.chunk_blended[ci]                       # the plan this chunk put on the grid
        if bl is None or not len(bl):
            continue
        used = min(run._used_len(ci), len(bl))           # steps executed before the next replan
        e = min(used + 1, len(bl))                        # +1 draws through to the next anchor so
        col = _CYCLE[k % len(_CYCLE)] if cyclic else colf(tarr[ci])  # segments connect flush
        k += 1
        t = tarr[ci] + np.arange(len(bl)) * dt           # anchored at t_arrive
        ax.plot(t[:e], bl[:e, j], color=col, lw=1.6, alpha=0.95, solid_capstyle="round",
                label="chunk: executed" if le else None, zorder=4)
        le = False
        if e < len(bl):                                  # provisional tail (never executed)
            ax.plot(t[e - 1:], bl[e - 1:, j], color=col, lw=1.1, alpha=0.55, ls="--",
                    label="chunk: tail (provisional)" if lt else None, zorder=3)
            lt = False
        ax.plot(t[0], bl[0, j], marker="o", ms=4.5, color=col, mec="k", mew=0.4, zorder=7,
                label="chunk anchor (seam)" if la else None)
        la = False
        sp = run.chunk_sent_prefix[ci] if ci < len(run.chunk_sent_prefix) else None
        if sp is not None and len(sp):                   # committed prefix (the inference lead-in)
            tsp = (run.chunk_t_obs[ci] - t0) + np.arange(len(sp)) * dt
            ax.plot(tsp, sp[:, j], color="dimgray", lw=0, marker="x", ms=4, alpha=0.45,
                    label="sent prefix" if lsp else None, zorder=6)
            lsp = False
            # observation instant this chunk was launched from (t_obs), in the chunk's colour.
            # It lands on the PREVIOUS plan -- the arm executed that during the inference delay,
            # so t_obs -> the same-colour anchor is exactly this chunk's round-trip latency.
            ax.plot(tsp[0], sp[0, j], marker="*", ms=12, color=col, mec="k", mew=0.5, zorder=8,
                    label="observation (chunk start)" if lobs else None)
            lobs = False
    if not cyclic:
        sm = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array([])
        cb = ax.figure.colorbar(sm, ax=ax, pad=0.01, fraction=0.035)
        cb.set_label(cb_label, fontsize=8)


def _plot_joint(ax, run, j, t0, args):
    # raw chunks: flat (default) or coloured (time / cycle) executed-vs-tail split
    if not args.no_chunks:
        (_plot_chunks_flat if args.chunk_style == "flat" else _plot_chunks_coloured)(ax, run, j, t0, args)

    # the 30 Hz plan actually streamed. In a coloured style the executed segments already tile
    # into this line, so draw it faint just to show the stitched whole; in 'flat' mode it is the
    # primary plan line.
    et, ep = run.executed_plan(j)
    if len(et):
        if args.chunk_style == "flat":
            ax.plot(et - t0, ep, color="tab:cyan", lw=1.8, label="30 Hz plan", zorder=4)
        else:
            ax.plot(et - t0, ep, color="black", lw=0.8, alpha=0.25, zorder=2,
                    label="30 Hz plan (stitched)")

    # measured
    if len(run.js_t):
        ax.plot(run.js_t - t0, run.js_pos[:, j], color="black", lw=1.3,
                label="measured", zorder=6)

    ax.set_ylabel("position [rad]", fontsize=9)
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    if args.tmin is not None or args.tmax is not None:
        ax.set_xlim(args.tmin, args.tmax)
        _fit_y_to_window(ax, run, j, t0, args)


def _fit_y_to_window(ax, run, j, t0, args):
    """When zoomed on x, matplotlib keeps the y-range of the WHOLE run, squishing the window's
    data into a sliver. Refit y to just the plan + measured samples visible in [tmin,tmax]."""
    lo = args.tmin if args.tmin is not None else -np.inf
    hi = args.tmax if args.tmax is not None else np.inf
    ys = []
    et, ep = run.executed_plan(j)
    if len(et):
        m = ((et - t0) >= lo) & ((et - t0) <= hi)
        ys.append(ep[m])
    if len(run.js_t):
        m = ((run.js_t - t0) >= lo) & ((run.js_t - t0) <= hi)
        ys.append(run.js_pos[m, j])
    ys = np.concatenate([y for y in ys if len(y)]) if any(len(y) for y in ys) else None
    if ys is not None and len(ys):
        pad = 0.05 * (ys.max() - ys.min() + 1e-6)
        ax.set_ylim(ys.min() - pad, ys.max() + pad)


def _out_path(base, jname):
    root, ext = os.path.splitext(base)
    return f"{root}_{jname}{ext or '.png'}"


def _plot_gripper(run, t0, args):
    """One figure overlaying the whole gripper pipeline, mirroring the per-joint arm plots:

      * raw model gripper chunks -- orange, faint (dropped stale prefix as faint red x's);
      * adopted 30 Hz gripper command (grip_blended stitched) -- cyan;
      * gate decisions (the discrete OPEN/CLOSE goals dispatched) -- a step of the commanded
        position + markers at the model value that tripped the gate (hollow = not dispatched);
      * measured gripper position (/joint_states) -- black;
      * the hysteresis thresholds -- dashed lines.

    Read it: if grip_blended dips below the open threshold but no OPEN event fires (or fires but
    was never re-crossed), the never-reopen is the gate; if grip_blended never dips, the model
    stayed in the closed basin. Returns the figure, or None if the trace has no gripper data."""
    has_chunk_grip = any(g is not None for g in run.chunk_grip_blended)
    has_js_grip = run.js_grip is not None and len(run.js_grip)
    has_evt = len(run.grip_evt_t) > 0
    if not (has_chunk_grip or has_js_grip or has_evt):
        return None

    dt = run.dt
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))

    # raw model gripper chunks (+ dropped stale prefix), anchored like the arm raw chunks
    if not args.no_chunks:
        first = True
        for ci in range(run.n_chunks):
            if ci % args.stride:
                continue
            gr = run.chunk_grip_raw[ci] if ci < len(run.chunk_grip_raw) else None
            if gr is None:
                continue
            d = int(run.chunk_d[ci])
            traw = (run.chunk_t_obs[ci] - t0) + np.arange(len(gr)) * dt
            ax.plot(traw[d:], gr[d:], color="tab:orange", lw=0.8, alpha=0.30,
                    label="raw model gripper (kept)" if first else None, zorder=1)
            if d > 0:
                ax.plot(traw[:d + 1], gr[:d + 1], color="tab:red", lw=0.6, alpha=0.30,
                        ls=":", marker="x", ms=3,
                        label="raw model gripper (dropped)" if first else None, zorder=1)
            first = False

    # adopted 30 Hz gripper command actually stepped through the gate
    gt, gg = run.executed_gripper()
    if len(gt):
        ax.plot(gt - t0, gg, color="tab:cyan", lw=1.8, label="adopted gripper cmd", zorder=4)

    # hysteresis thresholds
    for key, lbl in (("grip_close_thr", "close thr"), ("grip_open_thr", "open thr")):
        v = run.meta.get(key)
        if v is not None:
            ax.axhline(v, color="gray", ls="--", lw=0.8, alpha=0.7)
            ax.text(0.002, v, f" {lbl} {v:.2f}", transform=ax.get_yaxis_transform(),
                    va="bottom", fontsize=7, color="gray")

    # gate decisions: step of the commanded position + a marker at the model value that tripped it
    if has_evt:
        et = run.grip_evt_t - t0
        ax.step(et, run.grip_evt_position, where="post", color="tab:purple", lw=1.2,
                alpha=0.8, label="gate command (position)", zorder=3)
        for k in range(len(et)):
            disp = bool(run.grip_evt_dispatched[k])
            col = "tab:green" if run.grip_evt_cmd[k] == "OPEN" else "tab:red"
            ax.plot(et[k], run.grip_evt_value[k], marker="o", ms=7, color=col,
                    mfc=col if disp else "none", zorder=7)

    # measured gripper
    if has_js_grip:
        ax.plot(run.js_t - t0, run.js_grip, color="black", lw=1.3, label="measured", zorder=6)

    ax.set_ylabel("gripper [0 open .. ~0.8 closed]", fontsize=9)
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    if args.tmin is not None or args.tmax is not None:
        ax.set_xlim(args.tmin, args.tmax)
    n_open = int(np.sum([c == "OPEN" for c in run.grip_evt_cmd]))
    n_close = int(np.sum([c == "CLOSE" for c in run.grip_evt_cmd]))
    fig.suptitle(f"VLA async trace | GRIPPER | gate: {n_close} CLOSE / {n_open} OPEN | "
                 f"{run.n_chunks} chunks", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace", help="JSONL trace file")
    ap.add_argument("--joints", type=int, nargs="+", default=list(range(6)),
                    help="Joint indices to plot (0-5), default all")
    ap.add_argument("--stride", type=int, default=1, help="Plot every Nth chunk (default 1)")
    ap.add_argument("--tmin", type=float, default=None, help="Start time (s, normalized)")
    ap.add_argument("--tmax", type=float, default=None, help="End time (s, normalized)")
    ap.add_argument("--out", default=None,
                    help="Save one figure per joint (run.png -> run_joint_1.png)")
    ap.add_argument("--no-show", action="store_true", help="Do not open a window")
    ap.add_argument("--chunk-style", choices=["flat", "time", "cycle"], default="flat",
                    help="flat (default): all chunks faint orange. time: colour by arrival time "
                         "(gradient + colorbar). cycle: high-contrast repeating palette by chunk "
                         "order, best for telling adjacent overlapping chunks apart. time/cycle "
                         "also split executed (solid) vs provisional tail (dashed) + seam anchors.")
    ap.add_argument("--chunk-cmap", default="viridis",
                    help="Colormap for --chunk-style time (e.g. viridis, turbo, plasma). Ignored by cycle.")
    ap.add_argument("--chunk-cmap-period", type=float, default=0.0,
                    help="For --chunk-style time: repeat the gradient every N seconds (adjacent "
                         "chunks stay distinct on a full-episode view). 0 = spread once over the run/window.")
    ap.add_argument("--no-chunks", action="store_true", help="Hide the raw model chunks")
    ap.add_argument("--no-gripper", action="store_true", help="Skip the gripper figure")
    args = ap.parse_args()

    run = load_run(args.trace)
    if not run.n_chunks and not len(run.js_t):
        raise SystemExit("Trace has no data.")
    t0 = run.t0
    mode = "RTC" if run.meta.get("rtc_prefix") else run.meta.get("arm_mode", "?")

    for j in args.joints:
        jname = ARM_JOINTS[j] if j < len(ARM_JOINTS) else f"j{j}"
        fig, ax = plt.subplots(1, 1, figsize=(14, 5))
        _plot_joint(ax, run, j, t0, args)
        if run.meta.get("rtc_prefix"):
            detail = f"RTC d={run.meta.get('rtc_delay')} blend={run.meta.get('rtc_blend_steps')}"
            res = _pin_residual(run, j)
            if res is not None:
                detail += f" | pin resid {res * 1e3:.2f} mrad"
        elif mode == "interp":
            detail = f"blend {run.meta.get('blend_steps')}"
        else:
            detail = f"decay {run.meta.get('ensemble_decay')}"
        fig.suptitle(f"VLA async trace | {jname} | mode {mode} ({detail}) | "
                     f"{run.n_chunks} chunks | control {run.control_hz:.0f} Hz",
                     fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        if args.out:
            path = _out_path(args.out, jname)
            fig.savefig(path, dpi=130)
            print(f"saved {path}")

    if not args.no_gripper:
        gfig = _plot_gripper(run, t0, args)
        if gfig is None:
            print("no gripper data in trace; skipping gripper figure")
        elif args.out:
            path = _out_path(args.out, "gripper")
            gfig.savefig(path, dpi=130)
            print(f"saved {path}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
