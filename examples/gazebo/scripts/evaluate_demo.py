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


def rectangle_vertices(x, y, yaw, front, rear, width):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        (x + cosine * px - sine * py, y + sine * px + cosine * py)
        for px, py in ((front, width / 2), (front, -width / 2),
                       (rear, -width / 2), (rear, width / 2))
    ]


def point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared
    fraction = max(0.0, min(1.0, fraction))
    closest = (start[0] + fraction * dx, start[1] + fraction * dy)
    return math.hypot(point[0] - closest[0], point[1] - closest[1])


def projections_overlap(left, right, axis):
    left_projection = [x * axis[0] + y * axis[1] for x, y in left]
    right_projection = [x * axis[0] + y * axis[1] for x, y in right]
    return max(min(left_projection), min(right_projection)) <= min(
        max(left_projection), max(right_projection))


def polygons_overlap(left, right):
    for polygon in (left, right):
        for index, point in enumerate(polygon):
            next_point = polygon[(index + 1) % len(polygon)]
            edge = (next_point[0] - point[0], next_point[1] - point[1])
            if not projections_overlap(left, right, (-edge[1], edge[0])):
                return False
    return True


def polygon_distance(left, right):
    if polygons_overlap(left, right):
        return 0.0
    distance = math.inf
    for source, target in ((left, right), (right, left)):
        for point in source:
            for index, start in enumerate(target):
                end = target[(index + 1) % len(target)]
                distance = min(distance, point_segment_distance(point, start, end))
    return distance


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
    yaws = [float(row["yaw_rad"]) for row in rows]
    statuses = [int(row["status"]) for row in rows]
    collisions = [bool(int(row["collision"])) for row in rows]
    pre_gate_ys = [abs(y) for x, y in zip(xs, ys) if 9.2 <= x <= 9.6]
    gate_ys = [abs(y) for x, y in zip(xs, ys) if 9.75 <= x <= 10.25]
    obstacle = rectangle_vertices(3.4, 0.20, 0.0, 0.325, -0.325, 0.55)
    open_clearances = [
        polygon_distance(
            rectangle_vertices(x, y, yaw, 0.35, -0.35, 0.50), obstacle)
        for x, y, yaw in zip(xs, ys, yaws) if 1.0 <= x <= 5.5
    ]
    metrics = {
        "frame_count": len(frames),
        "telemetry_rows": len(rows),
        "final_x_m": xs[-1],
        "max_abs_lateral_deviation_m": max(abs(y) for y in ys),
        "max_abs_y_before_gate_m": max(pre_gate_ys, default=math.inf),
        "max_abs_y_in_gate_m": max(gate_ys, default=math.inf),
        "min_open_obstacle_body_clearance_m": min(open_clearances, default=0.0),
        "avoiding_frames": sum(status == 1 for status in statuses),
        "stop_frames": sum(status == 2 for status in statuses),
        "collision_detected": any(collisions),
    }
    checks = {
        "camera_stream_recorded": len(frames) >= 180,
        "forward_progress_beyond_gate": xs[-1] >= 10.6,
        "avoidance_activated": metrics["avoiding_frames"] >= 5,
        "visible_detour": metrics["max_abs_lateral_deviation_m"] >= 0.25,
        "generous_open_obstacle_clearance":
            metrics["min_open_obstacle_body_clearance_m"] >= 0.24,
        "stayed_in_camera_lane": metrics["max_abs_lateral_deviation_m"] <= 2.4,
        "recentered_before_gate": bool(pre_gate_ys) and max(pre_gate_ys) <= 0.30,
        "passed_one_meter_gate": bool(gate_ys) and max(gate_ys) <= 0.24,
        "no_contact": not metrics["collision_detected"],
    }
    inputs = [
        "src/bac_core.cpp", "src/bac_filter_node.cpp", "examples/gazebo/bac_demo.yaml",
        "examples/gazebo/worlds/adaptive_clearance.world", "examples/gazebo/models/robot.urdf",
        "examples/gazebo/models/open_space_obstacle.sdf",
        "examples/gazebo/Dockerfile", "examples/gazebo/run_demo.sh",
        "examples/gazebo/scripts/demo_driver.py",
        "examples/gazebo/scripts/record_demo.py", "examples/gazebo/scripts/evaluate_demo.py",
    ]
    result = {
        "schema_version": 1,
        "evidence_scope": "Gazebo Classic simulation; not physical-robot validation",
        "scenario": "static open-space obstacle avoidance, centerline recovery, then a 1.0 m gate",
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
