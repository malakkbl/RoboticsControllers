#!/usr/bin/env python3
"""ROS 2 node: runs controllers/pid.py on a real robot.

    /robot_pose (pose message)  ->  PIDPathTracker.step()  ->  /cmd_vel (Twist)

Run from anywhere (it finds the project root itself):
    python3 ros/pid_node.py --ros-args -p path_file:=paths/line_1m.csv -p v_ref:=0.08 -p dry_run:=true

Parameters (all optional; set with `-p name:=value`):
    pose_topic    /robot_pose      topic that publishes the robot pose            TO CONFIRM on the robot
    pose_type     PoseStamped      PoseStamped | Pose | PoseWithCovarianceStamped | Odometry | Pose2D
    cmd_topic     /cmd_vel         topic the robot listens to                     TO CONFIRM on the robot
    cmd_stamped   false            true -> publish TwistStamped instead of Twist
    path_file     ""               waypoint file (.csv/.yaml). Empty -> use the preset `path_name` from config.yaml
    path_name     line             preset used when path_file is empty
    path_frame    absolute         absolute: path coordinates are in the pose frame, used as they are
                                   robot_start: the path is shifted so its first point is where the robot is now
                                   (translation only, the path is not rotated)
    v_ref         0.0              cruise speed override in m/s (0 = use config.yaml). Use a low value for first tests
    dry_run       false            compute and log commands but publish ZERO speed (safe check of the whole chain)
    pose_timeout  0.3              s without a new pose before the robot is stopped
    config_file   ""               other config yaml (default: the project's config.yaml)

Safety behaviour: zero speed is published when the pose is stale, when the goal is reached, when
max_time (config sim.max_time) is exceeded, and on shutdown (Ctrl+C).
"""
import importlib
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))     # project root -> `common`, `controllers`

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from common.config import load_config
from common.paths import Path, make_path, path_from_file
from common.pose_utils import POSE_MSG_TYPES, extract_pose
from controllers.pid import PIDPathTracker


class PIDNode(Node):
    def __init__(self):
        super().__init__("pid_path_tracker")
        p = lambda name, default: self.declare_parameter(name, default).value
        pose_topic, self.pose_type = p("pose_topic", "/robot_pose"), p("pose_type", "PoseStamped")
        cmd_topic, self.cmd_stamped = p("cmd_topic", "/cmd_vel"), p("cmd_stamped", False)
        self.path_file, path_name = p("path_file", ""), p("path_name", "line")
        self.path_frame = p("path_frame", "absolute")
        self.pose_timeout, self.dry_run = p("pose_timeout", 0.3), p("dry_run", False)
        v_ref, config_file = p("v_ref", 0.0), p("config_file", "")

        if self.pose_type not in POSE_MSG_TYPES:
            raise ValueError(f"pose_type must be one of {list(POSE_MSG_TYPES)}, got {self.pose_type!r}")
        if self.path_frame not in ("absolute", "robot_start"):
            raise ValueError("path_frame must be 'absolute' or 'robot_start'")

        self.cfg = load_config(config_file or None)
        if v_ref > 0:
            self.cfg["controllers"]["pid"]["v_ref"] = float(v_ref)
        self.dt = 1.0 / self.cfg["robot"]["control_rate_hz"]
        self.max_time = float(self.cfg["sim"]["max_time"])
        self.ctrl = PIDPathTracker(self.cfg)

        # build the path now so a bad file fails at startup, not in the middle of a run
        self.base_path = (path_from_file(self.path_file) if self.path_file
                          else make_path(self.cfg["paths"][path_name]))

        module, cls = POSE_MSG_TYPES[self.pose_type]
        msg_type = getattr(importlib.import_module(module), cls)
        # best-effort + small queue: compatible with both reliable and best-effort publishers, and never replays old poses
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(msg_type, pose_topic, self._on_pose, qos)
        self.cmd_pub = self.create_publisher(TwistStamped if self.cmd_stamped else Twist, cmd_topic, 10)

        self.pose = None            # latest [x, y, theta]
        self.last_pose_time = None
        self.t0 = None              # set when the run starts (first fresh pose)
        self.stale = False
        self.finished = False
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"pid node ready: pose={pose_topic} ({self.pose_type}) cmd={cmd_topic} "
            f"rate={1 / self.dt:.0f} Hz v_ref={self.cfg['controllers']['pid']['v_ref']} "
            f"path={'file ' + self.path_file if self.path_file else 'preset ' + path_name} "
            f"frame={self.path_frame}{'  ** DRY RUN: publishing zero **' if self.dry_run else ''}")

    # ---- input -------------------------------------------------------------------------
    def _on_pose(self, msg):
        try:
            pose = extract_pose(msg, self.pose_type)
        except Exception as e:                                        # wrong pose_type for this topic
            self.get_logger().error(f"cannot read pose message: {e}", throttle_duration_sec=2.0)
            return
        if not all(math.isfinite(v) for v in pose):
            self.get_logger().warn("ignoring non-finite pose", throttle_duration_sec=2.0)
            return
        self.pose = np.array(pose)
        self.last_pose_time = self.get_clock().now()

    # ---- output ------------------------------------------------------------------------
    def _publish(self, v, w):
        if self.cmd_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        else:
            msg = Twist()
            msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def stop(self):
        """Zero speed, a few times in case a message is lost."""
        for _ in range(3):
            self._publish(0.0, 0.0)

    # ---- control loop ------------------------------------------------------------------
    def _start(self, pose):
        path = self.base_path
        if self.path_frame == "robot_start":
            path = Path(path.xy + (pose[:2] - path.xy[0]))
        else:
            d0 = float(np.hypot(*(path.xy[0] - pose[:2])))
            if d0 > 0.5:
                self.get_logger().warn(
                    f"robot is {d0:.2f} m from the path start: is the path in the same frame as the pose? "
                    "(use path_frame:=robot_start to start the path where the robot is)")
        self.ctrl.reset(path, self.dt)
        self.t0 = self.get_clock().now()
        self.get_logger().info(
            f"run started: pose=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f} rad), path {path.length:.2f} m, "
            f"from ({path.xy[0][0]:.2f}, {path.xy[0][1]:.2f}) to ({path.end[0]:.2f}, {path.end[1]:.2f})")

    def _tick(self):
        if self.finished:
            return
        now = self.get_clock().now()
        if self.pose is None:
            self.get_logger().warn(f"waiting for the first pose on the pose topic...", throttle_duration_sec=2.0)
            return
        age = (now - self.last_pose_time).nanoseconds * 1e-9
        if age > self.pose_timeout:
            self.stale = True
            self._publish(0.0, 0.0)
            self.get_logger().warn(f"pose is stale ({age:.2f} s old): publishing zero speed", throttle_duration_sec=1.0)
            return
        if self.t0 is None:
            self._start(self.pose)
        if self.stale:                                                # pose is back: drop the filter/integral memory
            self.stale = False
            self.ctrl.pid_h.reset()
            self.ctrl.pid_x.reset()
            self.get_logger().info("pose is back, resuming")

        t = (now - self.t0).nanoseconds * 1e-9
        if t > self.max_time:
            self.get_logger().warn(f"max_time {self.max_time:.0f} s exceeded: stopping")
            self.finished = True
            return
        v, w = self.ctrl.step(self.pose, t)
        if self.ctrl.done:
            self.get_logger().info(f"goal reached at t={t:.1f} s, pose=({self.pose[0]:.2f}, {self.pose[1]:.2f})")
            self.finished = True
            return
        d = self.ctrl.debug or {}
        self.get_logger().info(
            f"t={t:5.1f}  pose=({self.pose[0]:.2f}, {self.pose[1]:.2f}, {self.pose[2]:+.2f})  "
            f"heading_err={d.get('heading_err', float('nan')):+.2f}  cmd v={v:.3f} w={w:+.3f}",
            throttle_duration_sec=1.0)
        if self.dry_run:
            self._publish(0.0, 0.0)
        else:
            self._publish(v, w)


def main():
    rclpy.init()
    node = PIDNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
