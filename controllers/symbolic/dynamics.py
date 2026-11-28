"""Robot model used by the abstraction: one "move" = one symbolic command held for tau seconds.

Two kinds of move (config_symbolic.yaml -> motion):
  * "constant":      (v, w) applied instantly for tau seconds  (ideal unicycle, lecture slide
                     "Example: navigation problem"). Simple, but a real robot cannot change speed
                     instantly, so it drifts away from this model (we measured up to 8 cm in the simulator).
  * "rest_to_rest":  the robot starts stopped, speeds up smoothly to (v, w), cruises, slows down and is
                     stopped again before tau. The ramps respect the acceleration limits of config.yaml
                     (a_max, alpha_max) and the command delay, so the robot can follow it exactly.
                     This is what we use from step 4 on.

Either way a move is described by its path in the ROBOT's own frame (start at 0, 0, facing +x):
a list of points d(t). To place it anywhere, rotate by the start heading and shift to the start position.
"""
import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def unicycle_step(x, y, th, v, w, tau):
    """Ideal unicycle: where the robot is after holding (v, w) for tau seconds (arc of circle)."""
    x, y, th = (np.asarray(a, dtype=float) for a in (x, y, th))
    th_new = th + w * tau
    if abs(w) < 1e-9:
        return x + v * tau * np.cos(th), y + v * tau * np.sin(th), wrap(th_new)
    return (x + v / w * (np.sin(th_new) - np.sin(th)),
            y - v / w * (np.cos(th_new) - np.cos(th)),
            wrap(th_new))


def _ramp(peak, acc, n_active, dt):
    """Speeds for n_active control steps: ramp up at `acc`, cruise at `peak`, ramp down to 0 (a trapezoid)."""
    if n_active <= 0 or peak == 0:
        return np.zeros(max(n_active, 0))
    t = (np.arange(n_active) + 0.5) * dt
    T = n_active * dt
    t_ramp = min(abs(peak) / acc, T / 2)
    shape = np.minimum.reduce([np.ones(n_active), t / t_ramp, (T - t) / t_ramp])
    return peak * np.clip(shape, 0.0, 1.0)


class Move:
    """One symbolic command (v, w) turned into a concrete motion.

    vs, ws : the velocity commands to send, one per control step (what the controller publishes)
    pts    : (n+1, 3) path in the robot frame [x, y, theta] at each control step, starting at (0, 0, 0)
    """

    def __init__(self, v, w, tau, dt, kind="rest_to_rest", a_max=0.5, alpha_max=3.0, delay_steps=0, settle_steps=0):
        self.v, self.w, self.tau, self.dt = v, w, tau, dt
        n = int(round(tau / dt))
        if kind == "constant":
            vs, ws = np.full(n, v), np.full(n, w)
        elif kind == "rest_to_rest":
            # finish early: the delayed robot must be stopped, and then stay still for settle_steps
            # (the controller averages a few pose measurements while the robot is still)
            n_active = n - delay_steps - 1 - settle_steps
            vs = np.r_[_ramp(v, a_max, n_active, dt), np.zeros(n - n_active)]
            ws = np.r_[_ramp(w, alpha_max, n_active, dt), np.zeros(n - n_active)]
        else:
            raise ValueError(f"unknown motion kind {kind}")
        self.vs, self.ws = vs, ws
        # integrate in the robot frame, exactly like common/robot.py (midpoint heading)
        pts = np.zeros((n + 1, 3))
        for k in range(n):
            x, y, th = pts[k]
            mid = th + 0.5 * ws[k] * dt
            pts[k + 1] = (x + vs[k] * np.cos(mid) * dt, y + vs[k] * np.sin(mid) * dt, th + ws[k] * dt)
        self.pts = pts
        self.dist = np.hypot(pts[:, 0], pts[:, 1])     # straight-line distance from the start, at each step

    def place(self, pose, upto=None):
        """The move's path in the world, starting from `pose` = [x, y, theta]. Returns (n+1, 3)."""
        x, y, th = pose
        c, s = np.cos(th), np.sin(th)
        P = self.pts if upto is None else self.pts[: upto + 1]
        return np.column_stack([x + c * P[:, 0] - s * P[:, 1], y + s * P[:, 0] + c * P[:, 1], wrap(th + P[:, 2])])

    def end(self, pose):
        return self.place(pose)[-1]


def growth_bound(half_width, distance, disturbance):
    """How much a box of states can GROW in one move (lecture slide 41, "bounded derivatives").

    If the start is uncertain by +-half_width = (dx, dy, dth), after moving `distance` metres:
        dx+  = dx  + distance*dth + wx      (heading off by dth -> end point off by about distance*dth)
        dy+  = dy  + distance*dth + wy
        dth+ = dth                + wth
    """
    dx, dy, dth = half_width
    wx, wy, wth = disturbance
    bend = distance * dth
    return np.array([dx + bend + wx, dy + bend + wy, dth + wth])
