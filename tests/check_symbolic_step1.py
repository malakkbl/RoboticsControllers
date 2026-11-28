"""
Run it from the RoboticsControllers folder with python tests/check_symbolic_step1.py. It does 4 things in about 5 seconds:

Builds the graph and prints its size: 153,601 states, 14 commands.
Prints the start state (q = 9776), its tags (none) and its row of the transition table.
Runs the honesty test. It drops the robot at 20,000 random spots with random commands and random disturbance, then moves it with the real formulas. Each time, it checks that the robot landed in a cell that F predicted, and that its path stayed inside the swept area. It must print 0 failures. If even one failed, F would be lying, and nothing built on it could be trusted.
Saves the two pictures in outputs/
[v2: about 10 seconds; 221,185 states; start state q = 14032; the honesty test now also adds random sensor error.]
"""

"""Step 1 check: build the abstraction, print its size, and TEST that it is sound.

Soundness test ("is F really an over-approximation?"):
  take random cells q and commands p. The robot MEASURES a pose inside q, but its true pose is a bit
  off (measurement error). Move the true robot with the command (+ random disturbance), measure again,
  and check that the measured landing cell is in F(q, p). Also check every point of the true path stays
  inside the swept area. If even one sample fails, the abstraction is wrong.

Run from the repo root:  python tests/check_symbolic_step1.py
It also saves two pictures in outputs/symbolic_step1_*.png
"""
import sys, pathlib
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from controllers.symbolic.abstraction import build_default
from controllers.symbolic.viz import plot_world, plot_transition

abst = build_default()
print("Abstraction built:")
for k, v in abst.summary().items():
    print(f"  {k:40s} {v}")

q0 = abst.quantize(abst.world.start_pose)
print(f"\nStart pose {abst.world.start_pose} -> symbolic state q0 = {q0}  (i, j, k) = {tuple(int(a) for a in abst.ijk(q0))}")
print("labels of q0:", abst.labels_of(q0))
print("Transition table row of q0 (command -> number of possible next states):")
for u, succ in abst.transition_table(q0).items():
    print(f"  v={u[0]:4.1f} w={u[1]:5.1f} -> {len(succ):3d} states{'  (may leave grid)' if abst.Q_OUT in succ else ''}")

rng = np.random.default_rng(1)
n, bad_end, bad_sweep = 20000, 0, 0
for _ in range(n):
    q = int(rng.integers(abst.N)); p = int(rng.integers(abst.P))
    lo, hi = abst.cell_bounds(q)
    measured = rng.uniform(lo, hi)                       # what the robot believes
    true = abst.random_measure(measured, rng)            # where it really is (a bit off)
    path, end = abst.random_move(true, p, rng)
    if abst.quantize(abst.random_measure(end, rng)) not in abst.post(q, p):
        bad_end += 1
    if not abst.out[q, p]:
        i = np.floor((path[:, 0] - abst.world.x0) / abst.dx); j = np.floor((path[:, 1] - abst.world.y0) / abst.dx)
        if not ((abst.sweep_lo[q, p, 0] <= i) & (i <= abst.sweep_hi[q, p, 0]) &
                (abst.sweep_lo[q, p, 1] <= j) & (j <= abst.sweep_hi[q, p, 1])).all():
            bad_sweep += 1
print(f"\nSoundness test on {n} random samples: {bad_end} end-point failures, {bad_sweep} swept-area failures")
assert bad_end == 0 and bad_sweep == 0, "abstraction is NOT sound"
print("OK: the robot always lands in a cell predicted by F(q, p).")

out = ROOT / "outputs"; out.mkdir(exist_ok=True)
ax = plot_world(abst.world, abst=abst); ax.set_title("Prof grid, regions, inflated no-go zone (grey)")
plt.savefig(out / "symbolic_step1_world.png", dpi=130, bbox_inches="tight")
fig, axs = plt.subplots(1, 3, figsize=(14, 5))
q_demo = abst.quantize([0.6, 0.3, 0.0])
for ax, u in zip(axs, [(0.2, 0.0), (0.2, 0.8), (0.0, 0.8)]):
    plot_transition(abst, q_demo, abst.inputs.index(u), ax)
fig.tight_layout(); fig.savefig(out / "symbolic_step1_transitions.png", dpi=130)
print(f"Pictures saved in {out}/")
