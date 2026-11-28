"""End-to-end check of ros/pid_node.py against a fake robot, inside a ROS 2 environment (see Hardware.md, Docker).
Run: python3 tests/ros_integration.py

Scenarios
  A  normal run on paths/line_1m.csv: the node must reach the goal, exit by itself, respect the speed limits
  B  the pose stops arriving: the node must command zero speed (watchdog)
  C  dry_run: the node must publish only zero speed, so the robot does not move
"""
import math
import pathlib
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common.config import load_config

CFG = load_config()
V_MAX, W_MAX = CFG["robot"]["v_max"], CFG["robot"]["w_max"]


class Recorder(Node):
    def __init__(self):
        super().__init__("recorder")
        self.cmds, self.poses = [], []                        # (time, v, w) / (time, x, y)
        self.create_subscription(Twist, "/cmd_vel", lambda m: self.cmds.append((time.monotonic(), m.linear.x, m.angular.z)), 10)
        self.create_subscription(PoseStamped, "/robot_pose",
                                 lambda m: self.poses.append((time.monotonic(), m.pose.position.x, m.pose.position.y)), 10)


def start(script, *params, log):
    args = [sys.executable, str(ROOT / "ros" / script), "--ros-args"]
    for p in params:
        args += ["-p", p]
    return subprocess.Popen(args, stdout=open(log, "w"), stderr=subprocess.STDOUT)


def stop(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()


def spin_for(rec, seconds, until=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end and (until is None or not until()):
        rclpy.spin_once(rec, timeout_sec=0.05)


def tail(path, n=12):
    return "\n".join(pathlib.Path(path).read_text().splitlines()[-n:])


def scenario(rec, name, fake_params, node_params, seconds, until_node_exit=False):
    rec.cmds.clear(); rec.poses.clear()
    fake = start("fake_robot.py", *fake_params, log=f"/tmp/{name}_fake.log")
    time.sleep(1.5)                                                         # let the fake robot start publishing
    node = start("pid_node.py", *node_params, log=f"/tmp/{name}_node.log")
    t_start = time.monotonic()
    spin_for(rec, seconds, until=(lambda: node.poll() is not None) if until_node_exit else None)
    exited_by_itself = node.poll() is not None
    exit_code = node.returncode
    stop(node); stop(fake)
    spin_for(rec, 0.3)
    return dict(t_start=t_start, exited=exited_by_itself, code=exit_code, node_log=f"/tmp/{name}_node.log")


def check(cond, msg, failures, log=None):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        failures.append(msg)
        if log:
            print("       --- node log (tail) ---\n" + "\n".join("       " + l for l in tail(log).splitlines()))


def main():
    rclpy.init()
    rec = Recorder()
    failures = []

    print("A: normal run to the goal")
    r = scenario(rec, "A", ["start_pose:=[0.0, 0.1, 0.0]"], ["path_file:=" + str(ROOT / "paths/line_1m.csv")], 60, until_node_exit=True)
    nz = [(v, w) for _, v, w in rec.cmds]
    end_err = math.hypot(rec.poses[-1][1] - 1.0, rec.poses[-1][2] - 0.0) if rec.poses else 9
    check(r["exited"] and r["code"] == 0, "node reached the goal and exited cleanly", failures, r["node_log"])
    check(end_err < 0.1, f"robot stopped near the goal (end error {100 * end_err:.1f} cm)", failures, r["node_log"])
    check(bool(nz) and max(abs(v) for v, _ in nz) <= V_MAX + 1e-9 and max(abs(w) for _, w in nz) <= W_MAX + 1e-9,
          f"commands within limits (max v {max((abs(v) for v, _ in nz), default=0):.3f}, max w {max((abs(w) for _, w in nz), default=0):.3f})", failures)
    check(any(v > 0.05 for v, _ in nz), "it actually drove (commanded v > 0.05 m/s at some point)", failures)
    check(nz[-1] == (0.0, 0.0), "last command is zero speed", failures)

    print("B: pose stops arriving after 4 s")
    r = scenario(rec, "B", ["start_pose:=[0.0, 0.0, 0.0]", "drop_pose_after:=4.0"],
                 ["path_file:=" + str(ROOT / "paths/line_1m.csv"), "v_ref:=0.05"], 9)
    t_last_pose = rec.poses[-1][0]
    after = [(v, w) for t, v, w in rec.cmds if t > t_last_pose + 0.6]
    check(any(v > 0 for t, v, w in rec.cmds if t < t_last_pose), "it was driving before the pose stopped", failures, r["node_log"])
    check(len(after) > 10 and all(v == 0.0 and w == 0.0 for v, w in after),
          f"zero speed commanded while the pose is stale ({len(after)} commands checked)", failures, r["node_log"])
    check("pose is stale" in pathlib.Path(r["node_log"]).read_text(), "node logged the stale-pose warning", failures, r["node_log"])

    print("C: dry run publishes only zero speed")
    r = scenario(rec, "C", ["start_pose:=[0.0, 0.0, 0.5]"], ["path_file:=" + str(ROOT / "paths/line_1m.csv"), "dry_run:=true"], 5)
    moved = math.hypot(rec.poses[-1][1] - rec.poses[0][1], rec.poses[-1][2] - rec.poses[0][2]) if rec.poses else 9
    check(len(rec.cmds) > 20 and all(v == 0.0 and w == 0.0 for _, v, w in rec.cmds),
          f"all {len(rec.cmds)} commands are zero", failures, r["node_log"])
    check(moved < 0.03, f"robot did not move ({100 * moved:.1f} cm of pose noise at most)", failures)

    rec.destroy_node(); rclpy.shutdown()
    print("\nALL ROS CHECKS PASSED" if not failures else f"\n{len(failures)} CHECK(S) FAILED")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
