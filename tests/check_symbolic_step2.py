""" More intuitive explanation
What this file does:

1. Writes the signs. It runs moment 1 for the task "reach the target, avoid obstacles, walls and C". It prints each ring as it goes, so you can watch the signs spread out from the target. Then it says whether the start cell got a sign. If the start has no sign, there's no guaranteed way, and the script tells you to use smaller cells or more commands.
2. Drives the robot 200 times. Each run starts at the start and follows the signs (moment 2), with a random push each second to imitate real-world imperfection. For every run, it checks three things:
   - the robot reaches the target
   - it never touches a forbidden cell, even in the middle of a move
   - it never lands in a cell without a sign
If any run fails, the script stops with an error.
3. Draws the picture you saw: green where signs exist, purple for the runs.
This is the lecture's main promise in action (slide 44): if the plan works on the graph, it works on the robot. The test is our evidence for it.
[v2: the start check now tests every position the sensor might report around the start, and each run also adds random sensor error.]

"""

"""Step 2 check: compute the reach-avoid controller and TEST it.

Task tested: "reach the target, never touch obstacles, walls or C" (from problem.yaml).
Test: drive the robot 200 times from the start with the controller, with random disturbance and
random measurement error (within the limits of config_symbolic.yaml), and check every run
  - reaches the target,
  - within the number of moves promised by the ring number of the start,
  - never touches a forbidden cell, even in the middle of a move.

Run from the repo root:  python tests/check_symbolic_step2.py   (about 30 s)
Saves outputs/symbolic_step2_reach_avoid.png
"""
import sys, pathlib, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from controllers.symbolic.abstraction import build_default
from controllers.symbolic.synthesis import reach_avoid, default_avoid, lookup, avoid_map
from controllers.symbolic.viz import plot_world

abst = build_default()
avoid_names = default_avoid(abst)
print(f"Task: reach 'target', always avoid {avoid_names}")
t0 = time.time()
res = reach_avoid(abst, ["target"], avoid_names, verbose=True)
print(f"Done in {time.time() - t0:.1f} s: {res['n_rings']} rings, "
      f"{int(res['win'].sum())} of {abst.N} states can surely reach the target")

start = abst.world.start_pose
q0 = abst.quantize(start)
if not all(res["win"][abst.quantize(c)] for c in abst.measurement_corners(start)):
    sys.exit("The start is NOT safely inside the winning states (counting sensor error): no guaranteed way.\n"
             "Try: start a bit further from walls/cell edges, smaller cells, or more commands.")
print(f"Start state q0 = {q0} is winning. Ring {res['ring'][q0]} -> at most {res['ring'][q0]} moves "
      f"of {abst.tau} s. First command: (v, w) = {abst.inputs[res['command'][q0]]}")

# ---- closed-loop test with random disturbance
avoid_xy = avoid_map(abst, avoid_names)
def cell_xy(x, y):
    return int(np.floor((x - abst.world.x0) / abst.dx)), int(np.floor((y - abst.world.y0) / abst.dx))

rng = np.random.default_rng(0)
runs, trajs, n_moves = 200, [], []
for r in range(runs):
    x = np.array(start, float)                          # TRUE pose
    traj, promised = [x[:2].copy()], res["ring"][q0]
    for step in range(promised + 1):
        status, u = lookup(abst, res, abst.random_measure(x, rng))   # the controller only sees a measurement
        assert status != "lost", f"run {r}: robot got outside the winning states"
        if status == "goal":
            break
        path, x = abst.random_move(x, abst.inputs.index(u), rng)
        for xs, ys, _ in path[1:]:                       # check points during the move too
            i, j = cell_xy(xs, ys)
            assert 0 <= i < abst.nx and 0 <= j < abst.ny and not avoid_xy[i, j], f"run {r}: touched a forbidden cell"
            traj.append([xs, ys])
    else:
        raise AssertionError(f"run {r}: not in the goal after the promised {promised} moves")
    n_moves.append(step); trajs.append(np.array(traj))
print(f"\nClosed-loop test: {runs}/{runs} runs reached the target safely, "
      f"in {min(n_moves)} to {max(n_moves)} moves (promised at most {res['ring'][q0]}).")

# ---- picture: winning region + a few runs
out = ROOT / "outputs"; out.mkdir(exist_ok=True)
fig, ax = plt.subplots(figsize=(8, 6))
plot_world(abst.world, ax, abst=abst)
share = res["win"].reshape(abst.nx, abst.ny, abst.nth).mean(axis=2)   # share of headings that win, per cell
xmin, ymin, xmax, ymax = abst.world.bounds
im = ax.imshow(np.where(share > 0, share, np.nan).T, origin="lower", extent=(xmin, xmax, ymin, ymax),
               cmap="Greens", vmin=0, vmax=1, alpha=0.55, interpolation="nearest")
for tr in trajs[:15]:
    ax.plot(tr[:, 0], tr[:, 1], "-", color="tab:purple", lw=0.8, alpha=0.7)
ax.set_title("Step 2: green = cells that can surely reach the target\n(darker = more headings work), purple = 15 test runs")
fig.colorbar(im, ax=ax, location="bottom", shrink=0.5, pad=0.12, label="share of headings that win")
fig.savefig(out / "symbolic_step2_reach_avoid.png", dpi=130, bbox_inches="tight")
print(f"Picture saved in {out}/symbolic_step2_reach_avoid.png")
