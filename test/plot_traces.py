#!/usr/bin/env python3
"""Plot BAC scenario harness traces.

Usage:
    python3 plot_traces.py [--dir TRACES_DIR] [scenario_name ...]

With no scenario names, plots every *.csv (excluding *_world.csv) in the dir.
For each scenario, draws the world + trajectory (left) and v/w/status/clearance
time series (right), and saves <scenario>.png next to the CSV.
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Same values as the default parameters
ROBOT_FRONT = 0.5
ROBOT_REAR = -0.5
ROBOT_WIDTH = 0.95

STATUS_COLORS = {0: "#2a9d2a", 1: "#e69500", 2: "#d62728"}  # CLEAR / AVOIDING / STOP
STATUS_NAMES = {0: "CLEAR", 1: "AVOIDING", 2: "STOP"}


def read_csv(path):
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = [{k: float(v) for k, v in row.items()} for row in reader]
    return rows


def robot_outline(x, y, th):
    c, s = math.cos(th), math.sin(th)
    hw = ROBOT_WIDTH / 2
    corners = [(ROBOT_FRONT, -hw), (ROBOT_FRONT, hw), (ROBOT_REAR, hw), (ROBOT_REAR, -hw), (ROBOT_FRONT, -hw)]
    return (
        [x + c * lx - s * ly for lx, ly in corners],
        [y + s * lx + c * ly for lx, ly in corners],
    )


def plot_scenario(trace_path: Path):
    name = trace_path.stem
    trace = read_csv(trace_path)
    world_path = trace_path.with_name(name + "_world.csv")
    walls = read_csv(world_path) if world_path.exists() else []

    fig = plt.figure(figsize=(14, 7))
    fig.suptitle(name)
    ax_map = fig.add_subplot(1, 2, 1)
    ax_v = fig.add_subplot(3, 2, 2)
    ax_w = fig.add_subplot(3, 2, 4, sharex=ax_v)
    ax_c = fig.add_subplot(3, 2, 6, sharex=ax_v)

    for wall in walls:
        ax_map.plot([wall["x1"], wall["x2"]], [wall["y1"], wall["y2"]], "k-", lw=2)

    xs = [r["x"] for r in trace]
    ys = [r["y"] for r in trace]
    colors = [STATUS_COLORS[int(r["status"])] for r in trace]
    ax_map.scatter(xs, ys, c=colors, s=4, zorder=3)

    # robot footprint every ~2.5s
    step = max(1, int(2.5 / (trace[1]["t"] - trace[0]["t"]))) if len(trace) > 1 else 1
    for r in trace[::step]:
        ox, oy = robot_outline(r["x"], r["y"], r["th"])
        ax_map.plot(ox, oy, color="#1f77b4", lw=0.6, alpha=0.5)

    handles = [plt.Line2D([], [], marker="o", ls="", color=c, label=STATUS_NAMES[k]) for k, c in STATUS_COLORS.items()]
    ax_map.legend(handles=handles, loc="best", fontsize=8)
    ax_map.set_aspect("equal")
    ax_map.grid(alpha=0.3)
    ax_map.set_xlabel("x [m]")
    ax_map.set_ylabel("y [m]")

    ts = [r["t"] for r in trace]
    ax_v.plot(ts, [r["cmd_v"] for r in trace], label="cmd v", color="#888")
    ax_v.plot(ts, [r["out_v"] for r in trace], label="out v", color="#e69500")
    ax_v.plot(ts, [r["act_v"] for r in trace], label="act v", color="#1f77b4")
    ax_v.set_ylabel("v [m/s]")
    ax_v.legend(fontsize=7)
    ax_v.grid(alpha=0.3)

    ax_w.plot(ts, [r["cmd_w"] for r in trace], label="cmd w", color="#888")
    ax_w.plot(ts, [r["out_w"] for r in trace], label="out w", color="#e69500")
    ax_w.plot(ts, [r["act_w"] for r in trace], label="act w", color="#1f77b4")
    ax_w.set_ylabel("w [rad/s]")
    ax_w.legend(fontsize=7)
    ax_w.grid(alpha=0.3)

    ax_c.plot(ts, [min(r["clearance"], 2.0) for r in trace], color="#d62728")
    ax_c.set_ylabel("clearance [m] (cap 2)")
    ax_c.set_xlabel("t [s]")
    ax_c.grid(alpha=0.3)

    out_path = trace_path.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f"wrote {out_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="traces")
    parser.add_argument("scenarios", nargs="*")
    args = parser.parse_args()

    trace_dir = Path(args.dir)
    if args.scenarios:
        paths = [trace_dir / f"{n}.csv" for n in args.scenarios]
    else:
        paths = sorted(p for p in trace_dir.glob("*.csv") if not p.stem.endswith("_world"))

    for path in paths:
        if not path.exists():
            print(f"skip (not found): {path}")
            continue
        plot_scenario(path)


if __name__ == "__main__":
    main()
