#!/usr/bin/env python3
"""Record an annotated overhead stream and synchronized BAC telemetry."""

import csv
import math
import os
import time

import cv2
import numpy as np
import rclpy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int8


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class DemoRecorder(Node):
    FIELDS = [
        "wall_time_s", "sim_time_s", "x_m", "y_m", "yaw_rad",
        "cmd_in_v_mps", "cmd_in_w_radps", "cmd_out_v_mps", "cmd_out_w_radps",
        "status", "min_scan_m", "collision",
    ]

    def __init__(self):
        super().__init__("bac_demo_recorder")
        self.output_dir = os.environ["BAC_DEMO_OUTPUT"]
        self.duration = float(os.environ.get("BAC_DEMO_DURATION", "28"))
        self.frames_dir = os.path.join(self.output_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)
        self.csv_file = open(os.path.join(self.output_dir, "telemetry.csv"), "w", newline="")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.FIELDS)
        self.csv_writer.writeheader()
        self.started = time.monotonic()
        self.last_frame = 0.0
        self.frame_index = 0
        self.pose = [0.0, 0.0, 0.0]
        self.cmd_in = [0.0, 0.0]
        self.cmd_out = [0.0, 0.0]
        self.status = 2
        self.min_scan = math.inf
        self.collision = False

        # Camera topic naming differs slightly between gazebo_ros releases.
        self.create_subscription(Image, "/demo_camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Image, "/demo_camera/overhead/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Twist, "/nav_cmd_vel", self.on_cmd_in, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_out, 10)
        self.create_subscription(Int8, "/avoid_status", self.on_status, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(ContactsState, "/bumper_states", self.on_contacts, 10)

    def on_odom(self, msg):
        self.pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
        ]

    def on_cmd_in(self, msg):
        self.cmd_in = [msg.linear.x, msg.angular.z]

    def on_cmd_out(self, msg):
        self.cmd_out = [msg.linear.x, msg.angular.z]

    def on_status(self, msg):
        self.status = int(msg.data)

    def on_scan(self, msg):
        finite = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        self.min_scan = min(finite) if finite else math.inf

    def on_contacts(self, msg):
        if msg.states:
            self.collision = True

    def on_image(self, msg):
        elapsed = time.monotonic() - self.started
        # Enforce 12 fps if both compatible camera topic names happen to exist.
        if elapsed - self.last_frame < 1.0 / 12.5:
            return
        self.last_frame = elapsed
        channels = 3
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.width * channels
        if raw.size < expected:
            self.get_logger().warning("camera frame is shorter than expected")
            return
        image = raw[:expected].reshape((msg.height, msg.width, channels))
        if msg.encoding.lower() == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        else:
            image = image.copy()

        status_name = {0: "CLEAR", 1: "AVOIDING", 2: "STOP"}.get(self.status, "UNKNOWN")
        cv2.rectangle(image, (0, 0), (msg.width, 78), (18, 22, 30), -1)
        cv2.putText(image, "BAC / Gazebo appearing-obstacle evidence", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
        line = (f"t={elapsed:4.1f}s  status={status_name:<8}  "
                f"pose=({self.pose[0]:.2f}, {self.pose[1]:+.2f}) m  "
                f"cmd=({self.cmd_out[0]:.2f}, {self.cmd_out[1]:+.2f})")
        color = (65, 220, 255) if self.status == 1 else (170, 240, 170)
        cv2.putText(image, line, (20, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    color, 1, cv2.LINE_AA)
        cv2.putText(image, "Simulation evidence - not physical-robot validation",
                    (20, msg.height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (230, 230, 230), 1, cv2.LINE_AA)
        cv2.imwrite(os.path.join(self.frames_dir, f"frame_{self.frame_index:05d}.jpg"), image,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 92])

        now = self.get_clock().now().nanoseconds * 1.0e-9
        self.csv_writer.writerow({
            "wall_time_s": f"{elapsed:.6f}", "sim_time_s": f"{now:.6f}",
            "x_m": f"{self.pose[0]:.6f}", "y_m": f"{self.pose[1]:.6f}",
            "yaw_rad": f"{self.pose[2]:.6f}", "cmd_in_v_mps": f"{self.cmd_in[0]:.6f}",
            "cmd_in_w_radps": f"{self.cmd_in[1]:.6f}", "cmd_out_v_mps": f"{self.cmd_out[0]:.6f}",
            "cmd_out_w_radps": f"{self.cmd_out[1]:.6f}", "status": self.status,
            "min_scan_m": "inf" if not math.isfinite(self.min_scan) else f"{self.min_scan:.6f}",
            "collision": int(self.collision),
        })
        self.frame_index += 1

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()


def main():
    rclpy.init()
    recorder = DemoRecorder()
    try:
        # A wall-clock loop also terminates cleanly if Gazebo stops publishing
        # /clock or camera data during a failed evidence run.
        while rclpy.ok() and time.monotonic() - recorder.started < recorder.duration:
            rclpy.spin_once(recorder, timeout_sec=0.1)
    finally:
        recorder.close()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
