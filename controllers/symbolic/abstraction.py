""" A MORE INTUITIVE EXPLANATION OF THE TRANSITION SYSTEM
(imane's notes. [v2: ...] marks a number that changed in v2, after the simulator tests of step 4.)

part 1 of 4: the states (Q)

+ One state = one small cell + one heading slice. That gives 80 × 60 cells × 32 slices = 153,600 states. We add one extra state, Q_OUT, which means "left the grid".
  [v2: small cells are now 2.5 cm (subdiv 12), so 96 × 72 cells × 32 slices = 221,184 states.]
+ Each state gets one number q. It works like seat numbers in a cinema, where (row, seat) becomes a single number. index(i, j, k) turns (column, row, slice) into q, and ijk(q) does the reverse.
+ quantize(pose) turns a real pose into its state. Take the start pose (0.15 m, 0.15 m, 0 rad):
    0.15 ÷ 0.03 = 5, so column 5
    0.15 ÷ 0.03 = 5, so row 5
    heading 0 lands in slice 16, because the slices are counted from −180°
    result: q = 9776
  [v2: start pose is now (0.16 m, 0.16 m, 0.1 rad): 0.16 ÷ 0.025 = 6.4 -> column 6, row 6, slice 16, q = 14032.
   It was moved off the cell edges so that sensor noise cannot put it in a neighbouring cell without a sign.]
+ cell_bounds(q) goes the other way. From q, it gives that state's little box: x from 0.15 to 0.18 m, y from 0.15 to 0.18 m, and a heading range.
  [v2: with 2.5 cm cells: x from 0.15 to 0.175 m, y from 0.15 to 0.175 m.]

-----
part 2 of 4: commands (P) and transitions (F)

P, the commands. This is just the menu from the config: 14 pairs of (speed, turn rate). Each one is held for 1 second.
  [v2: each command is now a 1.5 s "move": ramp up, cruise at (v, w), ramp down, stop (see dynamics.py, Move).]

F, the transitions. For every state q and every command p, F(q, p) is the list of states the robot might be in 1 second later. It's built in 3 moves, which are just dynamics.py put to work:

1. Move the centre. unicycle_step moves the centre of cell q.
   [v2: the centre is moved along the Move's path instead of unicycle_step.]
2. Add the margin. growth_bound adds ±4.5 cm, which gives a box Y.
   [v2: the margin now also counts the sensor error (±0.5 cm at the start and at the end).]
3. List the cells. Every small cell the box Y touches goes into F(q, p).

An example. Start in the cell at x 0.60 to 0.63 m, y 0.30 to 0.33 m, facing east. Command: 0.2 m/s, no turning.

+ The centre (0.615, 0.315) moves 20 cm right, to (0.815, 0.315).
+ With ±4.5 cm, the box is x from 0.77 to 0.86 and y from 0.27 to 0.36.
+ That box touches 4 columns × 4 rows of small cells, in 3 heading slices. So F has 4 × 4 × 3 = 48 states.
  [v2: same idea; the exact numbers differ a little (2.5 cm cells, 1.5 s moves of about 19 cm).]

In the code.
+ post(q, p) gives you that list of states.
+ transition_table(q) gives all 14 commands with their lists. That's one row of the "transition table" from your lab.
+ The computer does this for all 153,600 × 14 pairs, which takes about 1 second.
  [v2: 221,184 × 14 pairs, about 3 seconds.]

-------
part 3 of 4: the swept area

The problem. F only says where the robot is at the end of the second. It says nothing about the path in between. The robot could start clear of an obstacle, end clear of it, and still clip its corner on the way.
The fix. For each state and command, we also store the swept area. It's a rectangle that covers:

+ the start cell
+ the end box Y
+ a little extra for the bulge when the path curves (turning commands)
  [v2: computed more simply: we take every point of the move's path, widen each one by the same margin as Y,
   and the swept rectangle is the box around all of them.]

The robot's whole 1-second path is guaranteed to stay inside that rectangle. In the doc pictures, it's the green dotted rectangle.
How step 2 will use it. A command is only allowed if its swept area touches no forbidden cell. "Forbidden" means obstacles, walls and C. That's how "always avoid" holds during the move as well as at the end.
In the code it's sweep_lo and sweep_hi, the two corners of the rectangle. The test file checks this too. It samples points along 20,000 random paths and confirms every one stayed inside its swept area. Result: 0 failures.

 ------
 part 4 of 4: the labels

Labels are tags on each small cell: "this cell is A", "this cell is an obstacle", and so on. Step 2 reads these tags to know what to reach and what to avoid.

Step 1: grow the dangerous things. The robot's centre must stay 10 cm (the robot radius) away from obstacles, walls and C. So we make each of them 10 cm bigger on every side. In the example, the obstacle block covers x 0.9 to 1.5 m. Grown, it covers x 0.8 to 1.6 m. That's the grey band in the first picture of the doc. Walls work the same way: any cell less than 10 cm from the edge of the arena is tagged "wall".
  [v2: grown by 10 cm + 0.5 cm of sensor error.]

Step 2: two kinds of tag, both cautious.
    + "touches": the cell overlaps the grown region, even by a millimetre. We use it for things to avoid. If in doubt, it counts as danger.
    + "inside": the cell is completely inside the region. We use it for things to reach. We only count "I'm in A" when we're sure.

Example. Region A covers x 1.8 to 2.1 m and y 0.3 to 0.6 m. That's 10 × 10 = 100 small cells, all tagged "inside A".
  [v2: A is 12 × 12 small cells of 2.5 cm; the outer ring is not counted because of sensor error, so 10 × 10 = 100 are "inside A".]

The tags don't depend on heading. A cell is A whichever way the robot faces.
"""

"""STEP 1: turn the continuous robot into a finite transition system S = (Q, P, F).

  Q : symbolic states  = small cells of (x, y, theta)  +  one extra state Q_OUT ("left the grid")
  P : symbolic inputs  = a finite list of (v, w) commands, each held for tau seconds
  F : F(q, p) = every cell that the robot MIGHT be in after applying p from ANY point of cell q
              (for any disturbance). It over-approximates the real motion, so it is "sound":
              whatever the real robot does, it lands in one of the cells of F(q, p).

Because our cells form a regular grid, F(q, p) is always a BOX of cell indices
[i_lo..i_hi] x [j_lo..j_hi] x [k_lo..k_hi]. We store only the two corners: compact and fast.

We also store, for each (q, p), the "swept" box: all (x, y) cells the robot can cross DURING the
tau seconds (not only at the end). Step 2 uses it so the robot never cuts the corner of an obstacle.

Labels say which cells belong to which region of the prof's grid (A, B, C, target, obstacle, wall):
  inside[name][q]  -> the whole cell is inside the region   (use it for "reach A": we are SURE)
  touches[name][q] -> the cell touches the region inflated by the robot radius
                      (use it for "avoid C": any doubt counts as touching)

Imperfect measurements (lecture slide "Imperfect measurements"): the robot only knows a MEASURED pose,
off by up to `measurement_error`. So every cell is treated as slightly bigger than it is (start and end
of each move), avoid regions are grown by that error too, and "inside A" needs that much room.
"""
import numpy as np
import yaml

from common.config import load_config
from .dynamics import Move, growth_bound, wrap
from .world import GridWorld, HERE


def load_symbolic_params(cfg=None):
    """controllers.symbolic from config.yaml if it is there, else config_symbolic.yaml next to this file."""
    cfg = cfg or load_config()
    if "symbolic" in cfg.get("controllers", {}):
        return cfg["controllers"]["symbolic"]
    with open(HERE / "config_symbolic.yaml") as f:
        return yaml.safe_load(f)["symbolic"]


class Abstraction:
    def __init__(self, world: GridWorld, params: dict, cfg: dict | None = None):
        """cfg = the whole config.yaml (robot limits, control rate, delay, noise)."""
        self.world, self.params = world, params
        p = params
        cfg = cfg or load_config()
        robot = cfg["robot"]
        # ---------- Q: the partition of the state space ----------
        self.dx = world.cs / int(p["subdiv"])                 # size of a small cell (m)
        self.nx, self.ny = world.cols * int(p["subdiv"]), world.rows * int(p["subdiv"])
        self.nth = int(p["n_theta"])
        self.dth = 2 * np.pi / self.nth                       # size of a heading slice (rad)
        self.N = self.nx * self.ny * self.nth                 # number of real cells
        self.Q_OUT = self.N                                   # the extra "left the grid" state
        self.tau = float(p["tau"])
        self.W = np.asarray(p["disturbance"], dtype=float)
        self.r_robot = float(p["robot_radius"])
        # how wrong the measured pose can be: from config, or 3 x the noise std of config.yaml,
        # divided by sqrt(n_average) because the robot averages n_average readings while stopped
        n = cfg.get("noise", {})
        self.n_avg = int(p.get("n_average", 1))
        k = 3 / np.sqrt(self.n_avg)
        self.meas = np.asarray(p.get("measurement_error") or
                               [k * n.get("pos_std", 0), k * n.get("pos_std", 0), k * n.get("yaw_std", 0)], float)

        # ---------- P: the sampled inputs ----------
        vmax, wmax = robot.get("v_max", np.inf), robot.get("w_max", np.inf)
        self.inputs = [(float(v), float(w)) for v in p["v_levels"] for w in p["w_levels"]
                       if abs(v) <= vmax + 1e-9 and abs(w) <= wmax + 1e-9 and not (v == 0 and w == 0)]
        self.P = len(self.inputs)
        # each command becomes a concrete Move (see dynamics.py): its path and its velocity commands
        self.dt = 1.0 / robot["control_rate_hz"]
        kind = p.get("motion", "rest_to_rest")
        self.moves = [Move(v, w, self.tau, self.dt, kind, robot.get("a_max", 0.5), robot.get("alpha_max", 3.0),
                           int(robot.get("cmd_delay_steps", 0)), self.n_avg) for (v, w) in self.inputs]

        # ---------- F: transitions, computed for all cells at once ----------
        self._build_transitions()
        # ---------- labels: which cells are A, B, obstacle, ... ----------
        self._build_labels()

    # ------------------------------------------------------------------ indexing
    def index(self, i, j, k):
        """(i, j, k) cell indices -> one integer q. k is the heading slice."""
        return (np.asarray(i) * self.ny + np.asarray(j)) * self.nth + np.asarray(k) % self.nth

    def ijk(self, q):
        q = np.asarray(q)
        return q // (self.ny * self.nth), (q // self.nth) % self.ny, q % self.nth

    def cell_bounds(self, q):
        """Lower and upper corners [x, y, theta] of cell q."""
        i, j, k = self.ijk(q)
        x0, y0 = self.world.x0, self.world.y0
        lo = np.stack([x0 + i * self.dx, y0 + j * self.dx, -np.pi + k * self.dth], -1)
        return lo, lo + np.array([self.dx, self.dx, self.dth])

    def quantize(self, pose):
        """The quantizer theta(x) of the lecture: real pose [x, y, theta] -> symbolic state q."""
        x, y, th = pose
        i = int(np.floor((x - self.world.x0) / self.dx))
        j = int(np.floor((y - self.world.y0) / self.dx))
        if not (0 <= i < self.nx and 0 <= j < self.ny):
            return self.Q_OUT
        k = int(np.floor((wrap(th) + np.pi) / self.dth)) % self.nth
        return int(self.index(i, j, k))

    def center(self, q):
        lo, hi = self.cell_bounds(q)
        return (lo + hi) / 2

    # ------------------------------------------------------------------ F(q, p)
    def _to_index_box(self, lo, hi):
        """Continuous box -> box of cell indices (lo/hi are (..., 3) arrays). Heading is NOT wrapped
        here: k_lo can be < 0 or k_hi >= nth, we take it modulo nth when we list the cells."""
        x0, y0 = self.world.x0, self.world.y0
        i_lo = np.floor((lo[..., 0] - x0) / self.dx).astype(int)
        i_hi = np.floor((hi[..., 0] - x0) / self.dx).astype(int)
        j_lo = np.floor((lo[..., 1] - y0) / self.dx).astype(int)
        j_hi = np.floor((hi[..., 1] - y0) / self.dx).astype(int)
        k_lo = np.floor((lo[..., 2] + np.pi) / self.dth).astype(int)
        k_hi = np.floor((hi[..., 2] + np.pi) / self.dth).astype(int)
        k_hi = np.minimum(k_hi, k_lo + self.nth - 1)          # never more than one full turn
        return np.stack([i_lo, j_lo, k_lo], -1), np.stack([i_hi, j_hi, k_hi], -1)

    def _build_transitions(self):
        q = np.arange(self.N)
        c = self.center(q)                                     # (N, 3) cell centres
        # half size of a cell, plus the measurement error (the true start can be a bit outside the cell)
        half = np.array([self.dx / 2, self.dx / 2, self.dth / 2]) + self.meas
        cos_c, sin_c = np.cos(c[:, 2]), np.sin(c[:, 2])

        # int16 / float32 keep memory small (about 75 MB for 150 000 states x 14 inputs)
        self.succ_lo = np.zeros((self.N, self.P, 3), np.int16)   # F(q, p) = box [succ_lo, succ_hi]
        self.succ_hi = np.zeros((self.N, self.P, 3), np.int16)
        self.sweep_lo = np.zeros((self.N, self.P, 2), np.int16)  # (x, y) cells crossed during the step
        self.sweep_hi = np.zeros((self.N, self.P, 2), np.int16)
        self.out = np.zeros((self.N, self.P), bool)              # True -> Q_OUT is a possible successor
        self.Y_lo = np.zeros((self.N, self.P, 3), np.float32)    # continuous reachable box (for plots)
        self.Y_hi = np.zeros((self.N, self.P, 3), np.float32)

        for p, mv in enumerate(self.moves):
            # 1) where does the CENTRE of each cell go? (rotate the move's path by the heading, shift it)
            dx_, dy_, dth_ = mv.pts[-1]
            fc = np.stack([c[:, 0] + cos_c * dx_ - sin_c * dy_,
                           c[:, 1] + sin_c * dx_ + cos_c * dy_,
                           c[:, 2] + dth_], -1)                          # unwrapped heading
            # 2) how big can the uncertainty be around it? (lecture: Y = [f(xc) - dy, f(xc) + dy])
            #    + the measurement error again: the NEXT cell is chosen from a measured pose
            r = growth_bound(half, mv.dist[-1], self.W) + self.meas
            Ylo, Yhi = fc - r, fc + r
            self.Y_lo[:, p], self.Y_hi[:, p] = Ylo, Yhi
            # 3) which cells does that box Y touch?  -> F(q, p)
            lo, hi = self._to_index_box(Ylo, Yhi)
            self.succ_lo[:, p], self.succ_hi[:, p] = lo, hi
            self.out[:, p] = ((lo[:, 0] < 0) | (hi[:, 0] >= self.nx) |
                              (lo[:, 1] < 0) | (hi[:, 1] >= self.ny))
            # 4) swept area: every point of the path, each widened like in 2), then the box around all
            s_lo = np.full((self.N, 2), np.inf); s_hi = np.full((self.N, 2), -np.inf)
            gap = np.max(np.hypot(*np.diff(mv.pts[:, :2], axis=0).T)) if len(mv.pts) > 1 else 0.0
            for (px, py, _), dist in zip(mv.pts, mv.dist):
                cx = c[:, 0] + cos_c * px - sin_c * py
                cy = c[:, 1] + sin_c * px + cos_c * py
                rad = half[:2] + dist * half[2] + self.W[:2] + gap
                s_lo = np.minimum(s_lo, np.stack([cx, cy], -1) - rad)
                s_hi = np.maximum(s_hi, np.stack([cx, cy], -1) + rad)
            slo3, shi3 = self._to_index_box(np.c_[s_lo, np.zeros(self.N)], np.c_[s_hi, np.zeros(self.N)])
            self.sweep_lo[:, p], self.sweep_hi[:, p] = slo3[:, :2], shi3[:, :2]
            self.out[:, p] |= ((slo3[:, 0] < 0) | (shi3[:, 0] >= self.nx) |
                               (slo3[:, 1] < 0) | (shi3[:, 1] >= self.ny))

    def move(self, pose, p):
        """The real path of command p started from `pose`: (n+1, 3) array of [x, y, theta]."""
        return self.moves[p].place(pose)

    def random_move(self, true_pose, p, rng):
        """For tests: the robot's TRUE path for command p from true_pose, plus a random disturbance
        (within `disturbance`) at the end. Returns (path (n+1, 3), true end pose)."""
        path = self.moves[p].place(true_pose)
        return path, path[-1] + rng.uniform(-self.W, self.W)

    def measurement_corners(self, pose):
        """The 27 poses at the corners/edges of pose +- measurement_error. If the controller has a sign
        for ALL of them, it can start here whatever the sensor says."""
        import itertools
        return [np.asarray(pose, float) + np.array(s) * self.meas for s in itertools.product((-1, 0, 1), repeat=3)]

    def random_measure(self, true_pose, rng):
        """For tests: a measured pose = true pose + noise within `measurement_error`."""
        return np.asarray(true_pose) + rng.uniform(-self.meas, self.meas)

    def post(self, q, p):
        """F(q, p) as a list of symbolic states (includes Q_OUT if the robot may leave the grid)."""
        (il, jl, kl), (ih, jh, kh) = (int(a) for a in self.succ_lo[q, p]), (int(a) for a in self.succ_hi[q, p])
        I, J, K = np.meshgrid(np.arange(max(il, 0), min(ih, self.nx - 1) + 1),
                              np.arange(max(jl, 0), min(jh, self.ny - 1) + 1),
                              np.arange(kl, kh + 1), indexing="ij")
        succ = np.unique(self.index(I, J, K).ravel()).tolist()
        return succ + [self.Q_OUT] if self.out[q, p] else succ

    def transition_table(self, q):
        """{(v, w): [successors]} for one state: the 'transition table' of the lab, for one row."""
        return {u: self.post(q, p) for p, u in enumerate(self.inputs)}

    # ------------------------------------------------------------------ labels
    def _xy_overlap(self, rects, inflate):
        """(nx, ny) bool: True where an (x, y) cell overlaps one of the rectangles grown by `inflate`."""
        xs = self.world.x0 + np.arange(self.nx) * self.dx
        ys = self.world.y0 + np.arange(self.ny) * self.dx
        m = np.zeros((self.nx, self.ny), bool)
        eps = 1e-9   # sharing only an edge does not count as overlapping
        for (a, b, c, d) in rects:
            a, b, c, d = a - inflate, b - inflate, c + inflate, d + inflate
            m |= ((xs[:, None] + self.dx > a + eps) & (xs[:, None] < c - eps) &
                  (ys[None, :] + self.dx > b + eps) & (ys[None, :] < d - eps))
        return m

    def _xy_inside(self, rects, margin=0.0):
        """(nx, ny) bool: True where an (x, y) cell, grown by `margin`, is completely inside one rectangle."""
        m = np.zeros((self.nx, self.ny), bool)
        xs = self.world.x0 + np.arange(self.nx) * self.dx
        ys = self.world.y0 + np.arange(self.ny) * self.dx
        eps = 1e-9
        for (a, b, c, d) in rects:      # prof cells are aligned with our small cells, so per-rect is exact
            m |= ((xs[:, None] - margin >= a - eps) & (xs[:, None] + self.dx + margin <= c + eps) &
                  (ys[None, :] - margin >= b - eps) & (ys[None, :] + self.dx + margin <= d + eps))
        return m

    def _build_labels(self):
        """inside_xy / touches_xy are (nx, ny) maps; inside / touches are the same for every q (N,)."""
        W = self.world
        self.inside_xy, self.touches_xy = {}, {}
        for name in W.names:
            rects = W.rects(name)
            self.inside_xy[name] = self._xy_inside(rects, self.meas[0])
            self.touches_xy[name] = self._xy_overlap(rects, self.r_robot + self.meas[0])
        # walls: cells closer than robot_radius (+ measurement error) to the border of the grid
        r = self.r_robot + self.meas[0]
        xmin, ymin, xmax, ymax = W.bounds
        xs = xmin + (np.arange(self.nx) + 0.5) * self.dx
        ys = ymin + (np.arange(self.ny) + 0.5) * self.dx
        h = self.dx / 2
        self.touches_xy["wall"] = ((xs[:, None] - h < xmin + r) | (xs[:, None] + h > xmax - r) |
                                   (ys[None, :] - h < ymin + r) | (ys[None, :] + h > ymax - r))
        self.inside_xy["wall"] = np.zeros_like(self.touches_xy["wall"])
        expand = lambda m: np.repeat(m.ravel(), self.nth)   # same label for every heading
        self.inside = {k: expand(m) for k, m in self.inside_xy.items()}
        self.touches = {k: expand(m) for k, m in self.touches_xy.items()}

    def labels_of(self, q):
        """Names of the regions cell q is fully inside, and names it touches (inflated)."""
        if q == self.Q_OUT:
            return {"inside": [], "touches": ["out"]}
        return {"inside": [n for n, m in self.inside.items() if m[q]],
                "touches": [n for n, m in self.touches.items() if m[q]]}

    # ------------------------------------------------------------------ summary
    def summary(self):
        sizes = np.prod(np.clip(self.succ_hi.astype(int) - self.succ_lo + 1, 1, None), axis=-1)
        return {
            "small cell [m]": self.dx, "heading slice [deg]": round(np.degrees(self.dth), 2),
            "grid of small cells": (self.nx, self.ny, self.nth),
            "|Q| (states, + Q_OUT)": self.N + 1, "|P| (inputs)": self.P,
            "tau [s]": self.tau, "measurement error": self.meas.tolist(),
            "mean successors per (q,p)": round(float(sizes.mean()), 1),
            "share of (q,p) that may leave the grid": round(float(self.out.mean()), 3),
        }


def build_default(cfg=None, problem_path=None):
    """Convenience: load problem.yaml + config and build the abstraction."""
    cfg = cfg or load_config()
    world = GridWorld.from_yaml(problem_path)
    return Abstraction(world, load_symbolic_params(cfg), cfg)
