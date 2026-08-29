#!/usr/bin/env python3
"""Spawn the demo obstacle when odometry reaches a repeatable x position."""

import pathlib
import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PositionTriggeredSpawner(Node):
    def __init__(self, model_path):
        super().__init__("bac_demo_obstacle_spawner")
        self.model_xml = pathlib.Path(model_path).read_text()
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        self.request_sent = False
        self.future = None
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

    def on_odom(self, msg):
        if self.request_sent or msg.pose.pose.position.x < 1.0:
            return
        request = SpawnEntity.Request()
        request.name = "appearing_obstacle"
        request.xml = self.model_xml
        request.initial_pose.position.x = 3.4
        request.initial_pose.position.y = 0.35
        request.initial_pose.position.z = 0.4
        self.future = self.client.call_async(request)
        self.request_sent = True


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: spawn_obstacle.py MODEL.sdf [--ros-args ...]")
    model_path = sys.argv[1]
    rclpy.init(args=[sys.argv[0], *sys.argv[2:]])
    node = PositionTriggeredSpawner(model_path)
    if not node.client.wait_for_service(timeout_sec=10.0):
        raise SystemExit("/spawn_entity did not become ready")
    try:
        while rclpy.ok() and (node.future is None or not node.future.done()):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.future is None:
            raise SystemExit("odometry ended before the x trigger")
        response = node.future.result()
        print(response.status_message)
        if not response.success:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
