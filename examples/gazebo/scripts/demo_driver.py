#!/usr/bin/env python3
"""Publish the centerline-following command used by the BAC Gazebo evidence run."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


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
        self.publisher = self.create_publisher(Twist, "/nav_cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(0.05, self.publish_command)

    def on_odom(self, msg):
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_odom = True

    def publish_command(self):
        command = Twist()
        if self.have_odom:
            command.linear.x = 0.35
            # The upstream path follower continuously requests y=0/yaw=0,
            # including while BAC reports AVOIDING. This mirrors the Nav2
            # filter integration: upstream supplies intent and BAC modifies
            # it as needed for local collision avoidance.
            command.angular.z = max(-0.6, min(0.6, -0.3 * self.y - 0.8 * self.yaw))
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
