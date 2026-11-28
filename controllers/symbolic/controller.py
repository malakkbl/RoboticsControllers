"""STEP 4: the symbolic controller as a normal controller of this repo (same interface as pid.py).

It plugs into common/runner.py, common/metrics.py and common/plotting.py like the PID, and later into
the ROS node: every control step it receives the measured pose and returns (v, w).

What it does every move (tau seconds = one symbolic step, lecture slides 43-44 "controller concretization"):
  1. at the start of the move: average the pose readings taken while the robot was standing still,
     find the cell (quantizer), update the memory (mode), read the sign -> command p
  2. during the move: send the speed profile of that command (ramp up, cruise, ramp down, stop)
  3. at the end of the move, standing still: collect a few readings for step 1 of the next move

Writing the signs (step 3) takes a few minutes, so they are saved in outputs/symbolic_cache/ and reused
as long as problem.yaml and the settings do not change.
"""
import hashlib
import json

import numpy as np

from common.config import load_config, ROOT
from common.controller_base import Controller
from common.paths import Path
from .abstraction import Abstraction, load_symbolic_params
from .automaton import Automaton, product_reach_avoid, SequenceRunner
from .synthesis import default_avoid
from .world import GridWorld

CACHE = ROOT / "outputs" / "symbolic_cache"


def _cache_key(problem, params, cfg):
    blob = json.dumps([problem, params, cfg["robot"], cfg.get("noise", {})], sort_keys=True, default=str)
    return hashlib.sha1(blob.encode()).hexdigest()[:12]


def task_path(world):
    """A reference polyline start -> centre of each region to visit (only used for plots and the runner's log)."""
    pts = [world.start_pose[:2]]
    for name in world.spec.get("visit_in_order", []):
        rects = np.array(world.rects(name))
        pts.append([(rects[:, 0].min() + rects[:, 2].max()) / 2, (rects[:, 1].min() + rects[:, 3].max()) / 2])
    return Path(pts)


class SymbolicController(Controller):
    def __init__(self, cfg=None, problem_path=None, use_cache=True, verbose=True):
        cfg = cfg or load_config()
        self.world = GridWorld.from_yaml(problem_path)
        params = load_symbolic_params(cfg)
        self.abst = Abstraction(self.world, params, cfg)
        self.aut = Automaton.from_spec(self.world.spec)
        avoid = default_avoid(self.abst)

        key = _cache_key(self.world.__dict__ | {"spec": self.world.spec}, params, cfg)
        file = CACHE / f"signs_{key}.npz"
        if use_cache and file.exists():
            d = np.load(file)
            self.res = {"win": d["win"], "command": d["command"].astype(int), "ring": d["ring"].astype(int)}
            if verbose:
                print(f"Signs loaded from {file.name}")
        else:
            if verbose:
                print("Writing the signs (step 3 synthesis), a few minutes the first time...")
            self.res = product_reach_avoid(self.abst, self.aut, avoid)
            if use_cache:
                CACHE.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(file, win=self.res["win"], command=self.res["command"].astype(np.int16),
                                    ring=self.res["ring"].astype(np.int16))
        self.runner = SequenceRunner(self.abst, self.aut, self.res)
        self.n = len(self.abst.moves[0].vs)          # control steps per move
        self.debug = {}

    # ---------------------------------------------------------------- Controller interface
    def reset(self, path, dt):
        assert abs(dt - self.abst.dt) < 1e-9, "control rate differs from the one the moves were built for"
        self.runner.reset()
        self._done = False
        self.status = None
        self.move = None
        self.readings = []
        self.k = self.n - self.abst.n_avg           # begin by standing still and taking readings
        self.mode_log, self.move_log = [], []

    @property
    def done(self):
        return self._done

    def step(self, pose, t):
        if self.k == 0:                              # ---- start of a move: decide
            est = self._estimate()
            self.status, u = self.runner.step(est)
            if self.status == "done":
                self._done = True
                self.mode_log.append(self.runner.mode)
                return 0.0, 0.0
            if self.status == "lost":                # should never happen; stop and stay stopped
                self.move = None
            else:
                p = self.abst.inputs.index(u)
                self.move = self.abst.moves[p]
                self.move_log.append((t, u, self.runner.mode))
                self.debug = {"target": self.move.end(est)[:2], "mode": self.runner.mode}
            self.readings = []
        v, w = (0.0, 0.0) if self.move is None else (self.move.vs[self.k], self.move.ws[self.k])
        if self.k >= self.n - self.abst.n_avg:       # ---- end of a move: robot is still, take readings
            self.readings.append(np.asarray(pose, float))
        self.mode_log.append(self.runner.mode)
        self.k = (self.k + 1) % self.n
        return float(v), float(w)

    def _estimate(self):
        """Average of the readings taken while still (heading averaged on the circle)."""
        R = np.array(self.readings)
        th = np.arctan2(np.sin(R[:, 2]).mean(), np.cos(R[:, 2]).mean())
        return np.array([R[:, 0].mean(), R[:, 1].mean(), th])
