"""STEP 2: reach-avoid synthesis on the graph S = (Q, P, F) built in step 1.

Task: "reach the GOAL cells, and never touch the AVOID cells on the way".

Idea (frome lecture slides, "Reachability synthesis": P_{k+1} = Xs U Pre(P_k)), worked BACKWARDS from the goal:
    ring 0 = goal cells (that are safe)
    ring 1 = cells with a command that is SURE to land in ring 0
    ring 2 = cells with a command that is SURE to land in ring 0 or 1
    ...      stop when a ring adds nothing new.
"Sure" = ALL the possible landing cells F(q, p) are already marked (that is Pre in the lecture),
and the swept area of that command touches no AVOID cell.

Result:
    win[q]     True if, from state q, the goal can be reached for sure
    command[q] index of the command to use in q (-1 if none); look it up with abst.inputs[command[q]]
    ring[q]    the ring number = at most how many 1-second moves are left to the goal
               [v2: moves are now 1.5 s]
"""

""" More intuitively, think of the algorithm as two moments:
Moment 1: before the robot moves (the computer, once, backwards).
The computer starts at the target and works outwards. In every cell, it writes one instruction, like a road sign: "from here, do this command". Cells next to the target get signs first, then cells a bit further away, and so on. Those "rings" are just the order in which the signs get written. When this is finished, every cell from which the target can be reached safely has a sign.

Moment 2: while the robot drives (forwards).
The robot doesn't plan a path. Every second, it:

looks at which cell it's in
reads the sign in that cell
does that command for 1 second
[v2: one move now lasts 1.5 s, and the robot averages 9 position readings while stopped before reading the sign.]

Then it repeats. The path from start → next cell → next cell → target simply appears as it follows the signs.

Think of road signs to a hospital. Someone first put up signs all over the city by starting at the hospital and walking outwards. That's moment 1. Then a driver who knows nothing just follows the sign at each crossing. That's moment 2.

Why write the signs backwards? At the target, we know for sure we've arrived. One step back, we can check for sure that a command lands there. Going forwards from the start, we can't know if a turn leads to a dead end.


"""
import numpy as np


# ----------------------------------------------------------------------------- helpers
def _box_sum_2d(mask, lo, hi):
    """For many index rectangles [lo, hi] (inclusive) at once: how many True cells of `mask` are inside.
    Uses a 'summed-area table' so each rectangle costs 4 look-ups instead of a loop."""
    nx, ny = mask.shape
    S = np.zeros((nx + 1, ny + 1), np.int32)
    S[1:, 1:] = mask.astype(np.int32).cumsum(0).cumsum(1)
    i0 = np.clip(lo[..., 0], 0, nx - 1); j0 = np.clip(lo[..., 1], 0, ny - 1)
    i1 = np.clip(hi[..., 0], i0, nx - 1) + 1; j1 = np.clip(hi[..., 1], j0, ny - 1) + 1
    return S[i1, j1] - S[i0, j1] - S[i1, j0] + S[i0, j0]


def _box_all_3d(Z, lo, hi):
    """For every (q, p): are ALL states in the box F(q, p) marked in Z?   Z has shape (nx, ny, nth).
    Same summed-area trick in 3D. The heading wraps around (slice 31 is next to slice 0), so we glue
    two copies of Z along the heading axis."""
    nx, ny, nth = Z.shape
    Zt = np.concatenate([Z, Z], axis=2).astype(np.int32)
    S = np.zeros((nx + 1, ny + 1, 2 * nth + 1), np.int32)
    S[1:, 1:, 1:] = Zt.cumsum(0).cumsum(1).cumsum(2)
    lo = lo.astype(np.int64); hi = hi.astype(np.int64)
    shift = (lo[..., 2] // nth) * nth                       # bring the heading range into [0, 2*nth)
    k0 = lo[..., 2] - shift; k1 = hi[..., 2] - shift + 1
    i0 = np.clip(lo[..., 0], 0, nx - 1); j0 = np.clip(lo[..., 1], 0, ny - 1)
    i1 = np.clip(hi[..., 0], i0, nx - 1) + 1; j1 = np.clip(hi[..., 1], j0, ny - 1) + 1
    count = (S[i1, j1, k1] - S[i0, j1, k1] - S[i1, j0, k1] - S[i1, j1, k0]
             + S[i0, j0, k1] + S[i0, j1, k0] + S[i1, j0, k0] - S[i0, j0, k0])
    volume = (i1 - i0) * (j1 - j0) * (k1 - k0)
    return count == volume


# ----------------------------------------------------------------------------- the 3 building blocks
def avoid_map(abst, avoid_names):
    """(nx, ny) map of forbidden cells: the union of the 'touches' tags (already grown by the robot radius)."""
    m = np.zeros((abst.nx, abst.ny), bool)
    for name in avoid_names:
        m |= abst.touches_xy[name]
    return m


def allowed_moves(abst, avoid_xy):
    """(N, P) True where command p is allowed in state q:
    it cannot leave the grid, and its swept area touches no forbidden cell."""
    hits = _box_sum_2d(avoid_xy, abst.sweep_lo, abst.sweep_hi)
    return (hits == 0) & ~abst.out


def pre(abst, Z, allowed, rows=None):
    """The lecture's Pre, for all states at once.
    Returns (N, P): True where command p is allowed in q AND every landing state F(q, p) is in Z.
    With `rows` (array of state numbers), only those states are checked (faster), shape (len(rows), P)."""
    Z3 = Z.reshape(abst.nx, abst.ny, abst.nth)
    if rows is None:
        return _box_all_3d(Z3, abst.succ_lo, abst.succ_hi) & allowed
    return _box_all_3d(Z3, abst.succ_lo[rows], abst.succ_hi[rows]) & allowed[rows]


# ----------------------------------------------------------------------------- the algorithm
def reach_avoid(abst, goal_names, avoid_names, max_rings=10_000, verbose=False):
    """Reach any cell tagged 'inside' one of goal_names, never touching avoid_names. See module docstring."""
    avoid_xy = avoid_map(abst, avoid_names)
    avoid = np.repeat(avoid_xy.ravel(), abst.nth)                 # same for every heading
    goal = np.zeros(abst.N, bool)
    for name in goal_names:
        goal |= abst.inside[name]
    goal &= ~avoid                                                # a goal cell must be safe itself

    allowed = allowed_moves(abst, avoid_xy)
    # preference when several commands work: fastest first, then least turning (smoother driving)
    order = sorted(range(abst.P), key=lambda p: (-abs(abst.inputs[p][0]), abs(abst.inputs[p][1])))

    win = goal.copy()
    ring = np.where(goal, 0, -1)
    command = np.full(abst.N, -1)
    for k in range(1, max_rings + 1):
        rows = np.flatnonzero(~win & ~avoid & allowed.any(axis=1))  # only states without a sign yet
        ok = pre(abst, win, allowed, rows)                        # (len(rows), P)
        joined = ok.any(axis=1)
        new = np.zeros(abst.N, bool)
        new[rows[joined]] = True                                  # states that join in this ring
        if not new.any():
            break                                                 # fixed point: nothing new -> stop
        for p in reversed(order):                                 # last write wins -> preferred command
            command[rows[joined & ok[:, p]]] = p
        ring[new] = k
        win |= new
        if verbose:
            print(f"  ring {k:3d}: +{int(new.sum()):6d} states   (total {int(win.sum())})")
    return {"win": win, "command": command, "ring": ring, "goal": goal, "avoid": avoid,
            "n_rings": int(ring.max()), "goal_names": list(goal_names), "avoid_names": list(avoid_names)}


def default_avoid(abst):
    """Obstacles + walls + whatever the spec says to always avoid (e.g. C)."""
    return ["obstacle", "wall"] + list(abst.world.spec.get("always_avoid", []))


def lookup(abst, result, pose):
    """Use the controller: real pose -> (v, w) to apply for tau seconds, or None.
    Returns ('goal', None) when already in the goal, ('lost', None) if the state is not winning."""
    q = abst.quantize(pose)
    if q == abst.Q_OUT or not result["win"][q]:
        return "lost", None
    if result["goal"][q]:
        return "goal", None
    return "move", abst.inputs[result["command"][q]]
