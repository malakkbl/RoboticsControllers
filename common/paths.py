"""Paths: any list of (x, y) waypoints -> a dense, arc-length parameterised Path."""
import numpy as np


class Path:
    """Dense polyline with cumulative arc length. Works for ANY waypoint list."""

    def __init__(self, waypoints, ds=0.01):
        wp = np.asarray(waypoints, dtype=float)
        assert wp.ndim == 2 and wp.shape[1] == 2 and len(wp) >= 2, "need >=2 (x,y) waypoints"
        seg = np.hypot(*np.diff(wp, axis=0).T)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        keep = np.concatenate([[True], seg > 1e-9])           # drop duplicate points
        wp, s = wp[keep], s[keep]
        self.length = float(s[-1])
        grid = np.arange(0.0, self.length, ds)
        grid = np.append(grid, self.length)
        self.s = grid
        self.xy = np.column_stack([np.interp(grid, s, wp[:, 0]), np.interp(grid, s, wp[:, 1])])
        self.ds = ds

    def __len__(self):
        return len(self.s)

    @property
    def end(self):
        return self.xy[-1]

    def closest_index(self, pos, hint=0, window_m=0.8):
        """Closest path point, searched only in a window AHEAD of `hint`.
        The window stops the robot from jumping to another branch of a self-crossing path (figure-8)."""
        hi = min(len(self) - 1, hint + int(window_m / self.ds))
        d = np.hypot(*(self.xy[hint:hi + 1] - pos).T)
        return hint + int(np.argmin(d))

    def point_at(self, idx, lookahead):
        """Point `lookahead` metres further along the path than index idx (clamped to the end)."""
        j = min(len(self) - 1, idx + int(round(lookahead / self.ds)))
        return self.xy[j], j

    def cross_track(self, pos):
        """Unsigned distance from pos to the whole path (for metrics)."""
        return float(np.min(np.hypot(*(self.xy - pos).T)))


def make_path(spec, origin=(0.0, 0.0)):
    """Build a Path from a config entry like {type: circle, radius: 0.6}."""
    t = spec["type"]
    if t == "line":
        pts = [[0, 0], [spec["length"], 0]]
    elif t == "circle":                     # starts at (0,0), centre at (0, r), counter-clockwise, 1 lap
        r = spec["radius"]
        a = np.linspace(-np.pi / 2, 3 * np.pi / 2, 400)
        pts = np.column_stack([r * np.cos(a), r + r * np.sin(a)])
    elif t == "square":
        s = spec["side"]
        pts = [[0, 0], [s, 0], [s, s], [0, s], [0, 0]]
    elif t == "figure8":                    # Gerono lemniscate, starts at (0,0)
        a = spec["a"]
        u = np.linspace(0, 2 * np.pi, 600)
        pts = np.column_stack([a * np.sin(u), a * np.sin(u) * np.cos(u)])
    elif t == "waypoints":
        pts = spec["points"]
    else:
        raise ValueError(f"unknown path type {t}")
    pts = np.asarray(pts, dtype=float) + np.asarray(origin)
    return Path(pts)


def goal_path(start_xy, goal_xy):
    """'Go to goal' objective = a straight path from where you are to the goal."""
    return Path([start_xy, goal_xy])
