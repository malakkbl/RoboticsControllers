"""STEP 3: tasks with an order, like "visit A, then B, then target, always avoid C".

Lecture: slide "Automata-based specifications" (go to X1, then go to X2) and the LTL lecture.
In LTL the task is   F(A & F(B & F target))  &  G !C   ("eventually A, then eventually B, then
eventually target" and "globally never C").

Two parts:
  * "always avoid C"      -> handled exactly like step 2 (forbidden cells + swept area).
  * "A, then B, then T"   -> needs MEMORY: the same cell may need a different command depending on
                             what was already visited. The memory is a small automaton:

        mode 0 "go to A" --(robot inside A)--> mode 1 "go to B" --(inside B)--> mode 2 "go to target"
                                                                   --(inside target)--> mode 3 "done"

We then combine graph and automaton (the "product"): a product state = (mode, q).
Signs are written backwards exactly as in step 2, but now one sign per (mode, cell).
---------------
It's step 2 three times, with two twists.

Twist 1: the maps are written backwards. The target map is written first, then the B map, then the A map. The robot still drives forwards, A → B → target. It's the same "signs backwards, driving forwards" idea as step 2, one level up.

Twist 2: arriving isn't enough. You must arrive somewhere you can continue from. Say the robot reaches B, but in a corner of B, facing a wall, where the target map has no sign. It touched B, but now it's stuck. So when we write the B map, "arrived at B" only counts for spots in B where the next map (to the target) already has a sign. Same for A: it only counts if the B map has a sign there.

That's why we can't run three completely separate step 2's and glue them together. Each map has to know about the next one. Writing them backwards makes that possible, because when we write the B map, the target map is already there to check against.

In the code, the 3 maps are written together in one loop. That's the "product" from the lecture slide. But the idea is exactly what you said: step 2, chained, from the end back to the start.

"""
import numpy as np

from .synthesis import _box_all_3d, avoid_map, allowed_moves


class Automaton:
    """A small automaton over region names.
    transitions[mode] = list of (region_name, next_mode): if the robot is inside that region, jump.
    Checked in order; if none matches, stay in the same mode. `accepting` = modes where the task is done."""

    def __init__(self, n_modes, transitions, accepting, names=None):
        self.n_modes, self.transitions, self.accepting = n_modes, transitions, set(accepting)
        self.names = names or [f"mode {i}" for i in range(n_modes)]

    @classmethod
    def sequence(cls, regions):
        """'visit regions[0], then regions[1], ...'  ->  len(regions) + 1 modes in a line."""
        n = len(regions)
        trans = {i: [(regions[i], i + 1)] for i in range(n)}
        trans[n] = []
        names = [f"go to {r}" for r in regions] + ["done"]
        return cls(n + 1, trans, [n], names)

    @classmethod
    def from_spec(cls, spec):
        return cls.sequence(spec["visit_in_order"])

    def step(self, mode, inside_names):
        """Next mode after the robot is seen in a cell that is fully inside `inside_names`."""
        for region, nxt in self.transitions.get(mode, []):
            if region in inside_names:
                return nxt
        return mode

    def next_mode_all(self, abst, mode):
        """For ONE mode, the next mode for EVERY state q at once (N,) - used by the synthesis."""
        nxt = np.full(abst.N, mode)
        for region, t in reversed(self.transitions.get(mode, [])):
            nxt = np.where(abst.inside[region], t, nxt)
        return nxt

    def describe(self):
        lines = []
        for m in range(self.n_modes):
            arrows = ", ".join(f"inside {r} -> {self.names[t]}" for r, t in self.transitions.get(m, []))
            lines.append(f"  {m}: {self.names[m]:15s} {arrows or '(task finished)'}")
        return "\n".join(lines)


def product_reach_avoid(abst, aut, avoid_names, max_rings=10_000, verbose=False):
    """Signs for every (mode, cell): reach an accepting mode, never touching avoid_names.

    Same backwards ripples as step 2. The only new thing: when the robot lands in q', the memory
    jumps to next_mode(q'), so "is the landing state already signed?" is checked in THAT mode."""
    S, N = aut.n_modes, abst.N
    avoid_xy = avoid_map(abst, avoid_names)
    avoid = np.repeat(avoid_xy.ravel(), abst.nth)
    allowed = allowed_moves(abst, avoid_xy)
    order = sorted(range(abst.P), key=lambda p: (-abs(abst.inputs[p][0]), abs(abst.inputs[p][1])))
    nxt = np.stack([aut.next_mode_all(abst, s) for s in range(S)])        # (S, N)

    win = np.zeros((S, N), bool)
    for s in aut.accepting:
        win[s] = ~avoid                                                   # task done: every safe cell is fine
    ring = np.where(win, 0, -1)
    command = np.full((S, N), -1)
    can_move = allowed.any(axis=1)
    # which modes a mode depends on (itself + the modes it can jump to): only recompute a mode if one
    # of those changed in the last ring (saves a lot of time)
    deps = [set(np.unique(nxt[s]).tolist()) | {s} for s in range(S)]
    changed = set(range(S))
    for k in range(1, max_rings + 1):
        added = 0
        new_all = np.zeros((S, N), bool)
        for s in range(S):
            if s in aut.accepting or not (deps[s] & changed):
                continue
            # landing in q' counts as landing in product state (next_mode(q'), q')
            landing_ok = win[nxt[s], np.arange(N)]                         # (N,)
            rows = np.flatnonzero(~win[s] & ~avoid & can_move)             # only states without a sign yet
            ok = _box_all_3d(landing_ok.reshape(abst.nx, abst.ny, abst.nth),
                             abst.succ_lo[rows], abst.succ_hi[rows]) & allowed[rows]
            joined = ok.any(axis=1)
            for p in reversed(order):
                command[s, rows[joined & ok[:, p]]] = p
            new_all[s, rows[joined]] = True
            added += int(joined.sum())
        if added == 0:
            break
        changed = {s for s in range(S) if new_all[s].any()}
        win |= new_all                                                    # update all modes together (one ring)
        ring[new_all] = k
        if verbose and (k % 10 == 0):
            print(f"  ring {k:3d}: signed states per mode = {win.sum(axis=1).tolist()}")
    return {"win": win, "command": command, "ring": ring, "avoid": avoid, "next_mode": nxt,
            "n_rings": int(ring.max()), "avoid_names": list(avoid_names)}


class SequenceRunner:
    """Moment 2 with memory: keeps the current mode, reads the sign of (mode, cell)."""

    def __init__(self, abst, aut, result):
        self.abst, self.aut, self.res = abst, aut, result
        self.mode = 0

    def reset(self):
        self.mode = 0

    def step(self, pose):
        """Returns (status, (v, w) or None). status: 'move', 'done' or 'lost'."""
        a = self.abst
        q = a.quantize(pose)
        if q == a.Q_OUT:
            return "lost", None
        self.mode = self.aut.step(self.mode, a.labels_of(q)["inside"])     # update the memory first
        if self.mode in self.aut.accepting:
            return "done", None
        if not self.res["win"][self.mode, q]:
            return "lost", None
        return "move", a.inputs[self.res["command"][self.mode, q]]
