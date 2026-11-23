"""PID path tracking for a differential-drive robot.

Idea (2 loops, both simple):
  * STEERING:  aim at a "carrot" point `lookahead` metres ahead on the path.
               heading_error = angle(robot -> carrot) - robot heading
               w = PID(heading_error)                      <- the PID part
               (optional 2nd PID on lateral distance to the path, off by default)
  * SPEED:     v = v_ref, reduced when the heading error is large (turn first, then drive)
               and ramped down near the end of the path (so it stops instead of overshooting).

Works for ANY path (a list of waypoints); "go to goal" is just a 2-point path.
"""
import numpy as np
from common.controller_base import Controller
from common.robot import wrap


class PID:
    """Textbook PID with anti-windup, output clamp and a filtered derivative."""

    def __init__(self, kp, ki, kd, out_limit=np.inf, d_filter=0.3):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit, self.d_filter = out_limit, d_filter
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev_e = None
        self.d = 0.0

    def __call__(self, e, dt):
        d_raw = 0.0 if self.prev_e is None else (e - self.prev_e) / dt
        self.d = self.d_filter * self.d + (1 - self.d_filter) * d_raw   # low-pass: measurements are noisy
        self.prev_e = e
        u_unsat = self.kp * e + self.ki * self.i + self.kd * self.d
        u = float(np.clip(u_unsat, -self.out_limit, self.out_limit))
        if u == u_unsat or np.sign(e) != np.sign(u_unsat):               # anti-windup: only integrate if not saturated
            self.i += e * dt
        return u


class PIDPathTracker(Controller):
    def __init__(self, cfg):
        c = cfg["controllers"]["pid"]
        self.c = c
        self.v_max, self.w_max = cfg["robot"]["v_max"], cfg["robot"]["w_max"]
        self.tol = cfg["sim"]["goal_tolerance"]
        h, x = c["heading"], c["cross_track"]
        self.pid_h = PID(h["kp"], h["ki"], h["kd"], self.w_max, c["deriv_filter"])
        self.pid_x = PID(x["kp"], x["ki"], x["kd"], self.w_max, c["deriv_filter"])
        self._done = False

    def reset(self, path, dt):
        self.path, self.dt = path, dt
        self.idx = 0
        self._done = False
        self.pid_h.reset(); self.pid_x.reset()
        self.debug = {}

    @property
    def done(self):
        return self._done

    def step(self, pose, t):
        x, y, th = pose
        pos = np.array([x, y])
        c, path = self.c, self.path

        # progress along path (monotonic, windowed) and carrot point
        self.idx = path.closest_index(pos, self.idx)
        carrot, _ = path.point_at(self.idx, c["lookahead"])
        dist_end = float(np.hypot(*(path.end - pos)))
        # finished: near the end AND we've reached the last stretch of the path
        if dist_end < self.tol and self.idx >= len(path) - 1 - int(0.3 / path.ds):
            self._done = True
            return 0.0, 0.0

        # steering loop
        heading_err = wrap(np.arctan2(carrot[1] - y, carrot[0] - x) - th)
        w = self.pid_h(heading_err, self.dt)
        if c["cross_track"]["kp"] or c["cross_track"]["ki"] or c["cross_track"]["kd"]:
            p0 = path.xy[self.idx]
            tang = path.xy[min(self.idx + 5, len(path) - 1)] - p0
            n = np.hypot(*tang)
            if n > 1e-9:  # signed lateral error: >0 when robot is to the right of the path direction
                lat = (tang[0] * (p0[1] - y) - tang[1] * (p0[0] - x)) / n
                w += self.pid_x(lat, self.dt)

        # speed loop: slow down when misaligned and when approaching the end
        align = max(c["min_speed_factor"], np.cos(heading_err)) if abs(heading_err) < np.pi / 2 else c["min_speed_factor"]
        near_end = min(1.0, dist_end / c["slowdown_dist"])
        v = min(c["v_ref"], self.v_max) * align * max(near_end, 0.2)

        self.debug = {"target": carrot, "heading_err": heading_err}
        return v, float(np.clip(w, -self.w_max, self.w_max))
