"""Differential-drive simulator (unicycle kinematics) with the non-idealities that matter on the real robot."""
from collections import deque
import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class DiffDriveSim:
    """State (x, y, theta, v, w). Input: velocity command (v, w) = geometry_msgs/Twist.
    Includes: speed limits, acceleration limits, command latency, noisy pose measurement."""

    def __init__(self, cfg, start_pose):
        r, n = cfg["robot"], cfg["noise"]
        self.dt = 1.0 / r["control_rate_hz"]
        self.v_max, self.w_max = r["v_max"], r["w_max"]
        self.a_max, self.alpha_max = r["a_max"], r["alpha_max"]
        self.delay = int(r["cmd_delay_steps"])
        self.w_bias = float(r.get("w_bias", 0.0))   # constant yaw-rate disturbance (e.g. unequal wheels)
        self.pos_std, self.yaw_std = n["pos_std"], n["yaw_std"]
        self.rng = np.random.default_rng(n["seed"])
        self.wheel_radius, self.wheel_sep = r["wheel_radius"], r["wheel_separation"]
        self.reset(start_pose)

    def reset(self, pose):
        self.x, self.y, self.th = map(float, pose)
        self.v = self.w = 0.0
        self._queue = deque([(0.0, 0.0)] * self.delay)

    @property
    def true_pose(self):
        return np.array([self.x, self.y, self.th])

    def measure(self):
        """What the controller gets (like /robot_pose): true pose + noise."""
        return np.array([self.x + self.rng.normal(0, self.pos_std),
                         self.y + self.rng.normal(0, self.pos_std),
                         wrap(self.th + self.rng.normal(0, self.yaw_std))])

    def step(self, v_cmd, w_cmd):
        # 1. saturate command, then pass through the latency queue
        v_cmd = float(np.clip(v_cmd, -self.v_max, self.v_max))
        w_cmd = float(np.clip(w_cmd, -self.w_max, self.w_max))
        self._queue.append((v_cmd, w_cmd))
        v_t, w_t = self._queue.popleft()
        # 2. acceleration limits (motors can't change speed instantly)
        self.v += np.clip(v_t - self.v, -self.a_max * self.dt, self.a_max * self.dt)
        self.w += np.clip(w_t - self.w, -self.alpha_max * self.dt, self.alpha_max * self.dt)
        # 3. integrate unicycle (midpoint heading)
        mid = self.th + 0.5 * self.w * self.dt
        self.x += self.v * np.cos(mid) * self.dt
        self.y += self.v * np.sin(mid) * self.dt
        self.th = wrap(self.th + (self.w + self.w_bias) * self.dt)

    def wheel_speeds(self):
        """(left, right) wheel angular speeds in rad/s, what diff_drive_controller would command."""
        vl = self.v - self.w * self.wheel_sep / 2
        vr = self.v + self.w * self.wheel_sep / 2
        return vl / self.wheel_radius, vr / self.wheel_radius
