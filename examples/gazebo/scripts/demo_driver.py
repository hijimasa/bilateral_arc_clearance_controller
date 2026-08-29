#!/usr/bin/env python3
"""Publish the straight upstream command used by the BAC Gazebo evidence run."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int8


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class DemoDriver(Node):
    def __init__(self):
        super().__init__("bac_demo_driver")
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.status = 2
        self.publisher = self.create_publisher(Twist, "/nav_cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Int8, "/avoid_status", self.on_status, 10)
        self.create_timer(0.05, self.publish_command)

    def on_odom(self, msg):
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def on_status(self, msg):
        self.status = int(msg.data)

    def publish_command(self):
        command = Twist()
        if self.have_odom:
            command.linear.x = 0.35
            # Use only a small correction while BAC owns the avoidance turn.
            # Once CLEAR, the upstream follower gently returns to y=0/yaw=0.
            correction = max(-0.65, min(0.65, -0.35 * self.y - 1.0 * self.yaw))
            if self.status == 0:
                command.angular.z = correction
            elif self.status == 1:
                command.angular.z = 0.15 * correction
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = DemoDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
