#!/usr/bin/env python3
"""A pretend robot for testing ros/pid_node.py with no hardware.

Runs the same simulator as the notebook (common/robot.py: limits, latency, noisy pose) behind ROS topics:
    listens   /cmd_vel    (Twist)
    publishes /robot_pose (PoseStamped, noisy like a real pose source)

Like the real Pi, it zeroes the speed if no command arrives for 0.5 s.

Parameters:  start_pose [x, y, theta] (default [0, 0, 0]);  drop_pose_after seconds (default -1 = never):
after that time it stops publishing the pose, to test the node's stale-pose watchdog.
"""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node

from common.config import load_config
from common.robot import DiffDriveSim


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        start = list(self.declare_parameter("start_pose", [0.0, 0.0, 0.0]).value)
        self.drop_after = self.declare_parameter("drop_pose_after", -1.0).value
        self.sim = DiffDriveSim(load_config(), start)
        self.v_cmd = self.w_cmd = 0.0
        self.last_cmd_time = None
        self.t0 = self.get_clock().now()
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self.create_timer(self.sim.dt, self._tick)
        self.get_logger().info(f"fake robot at {start}")

    def _on_cmd(self, msg):
        self.v_cmd, self.w_cmd = msg.linear.x, msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        if self.last_cmd_time is None or (now - self.last_cmd_time).nanoseconds * 1e-9 > 0.5:
            self.v_cmd = self.w_cmd = 0.0                                  # the Pi's 0.5 s command timeout
        self.sim.step(self.v_cmd, self.w_cmd)
        if self.drop_after >= 0 and (now - self.t0).nanoseconds * 1e-9 > self.drop_after:
            return                                                         # simulate a dead pose source
        x, y, th = self.sim.measure()
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = now.to_msg(), "map"
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
