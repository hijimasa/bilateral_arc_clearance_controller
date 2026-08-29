#!/usr/bin/env python3
"""Evaluate the captured Gazebo evidence and write a provenance sidecar."""

import csv
import glob
import hashlib
import json
import math
import os
import subprocess
import sys


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_value(root, *args):
    try:
        return subprocess.check_output(
            ["git", "-c", f"safe.directory={root}", "-C", root, *args], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unavailable"


def main():
    package_root, output_dir = sys.argv[1:3]
    telemetry_path = os.path.join(output_dir, "telemetry.csv")
    with open(telemetry_path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    frames = sorted(glob.glob(os.path.join(output_dir, "frames", "frame_*.jpg")))
    if not rows:
        raise SystemExit("no telemetry rows were captured")

    xs = [float(row["x_m"]) for row in rows]
    ys = [float(row["y_m"]) for row in rows]
    statuses = [int(row["status"]) for row in rows]
    collisions = [bool(int(row["collision"])) for row in rows]
    metrics = {
        "frame_count": len(frames),
        "telemetry_rows": len(rows),
        "final_x_m": xs[-1],
        "max_abs_lateral_deviation_m": max(abs(y) for y in ys),
        "avoiding_frames": sum(status == 1 for status in statuses),
        "stop_frames": sum(status == 2 for status in statuses),
        "collision_detected": any(collisions),
    }
    checks = {
        "camera_stream_recorded": len(frames) >= 180,
        "forward_progress": xs[-1] >= 4.5,
        "avoidance_activated": metrics["avoiding_frames"] >= 5,
        "visible_detour": metrics["max_abs_lateral_deviation_m"] >= 0.25,
        "stayed_in_camera_lane": metrics["max_abs_lateral_deviation_m"] <= 2.4,
        "returned_toward_center": abs(ys[-1]) <= 1.0,
        "no_contact": not metrics["collision_detected"],
    }
    inputs = [
        "src/bac_core.cpp", "src/bac_filter_node.cpp", "examples/gazebo/bac_demo.yaml",
        "examples/gazebo/worlds/appearing_obstacle.world", "examples/gazebo/models/robot.urdf",
        "examples/gazebo/models/appearing_obstacle.sdf",
    ]
    result = {
        "schema_version": 1,
        "evidence_scope": "Gazebo Classic simulation; not physical-robot validation",
        "scenario": "an obstacle is spawned on the upstream path after motion begins",
        "ros_distribution": os.environ.get("ROS_DISTRO", "unknown"),
        "gazebo_version": os.environ.get("BAC_GAZEBO_VERSION", "unknown"),
        "package_commit_at_capture": git_value(package_root, "rev-parse", "HEAD"),
        "package_status_at_capture": git_value(package_root, "status", "--short"),
        "input_sha256": {path: sha256(os.path.join(package_root, path)) for path in inputs},
        "metrics": metrics,
        "checks": checks,
        "passed": all(checks.values()),
    }
    with open(os.path.join(output_dir, "evidence.json"), "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
