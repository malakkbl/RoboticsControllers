"""Step 3 check: "visit A, then B, then target, always avoid C" (spec in problem.yaml).

This test file works like step 2's:

It prints the memory machine, so you can see the 4 modes and when each switches.
It writes the 3 maps of signs. This takes about 1 minute.
It drives the robot 200 times with random pushes. For each run, it checks that the robot visited A, then B, then the target in that order, that it never touched C, obstacles or walls, and that it never got lost.
It saves the picture coloured by mode.
[v2: each run also adds random sensor error, and the start is checked for every reading the sensor might give.]

Run from the repo root:  python tests/check_symbolic_step3.py   (about 1 minute)
"""
import sys, pathlib, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from controllers.symbolic.abstraction import build_default
from controllers.symbolic.synthesis import default_avoid, avoid_map
from controllers.symbolic.automaton import Automaton, product_reach_avoid, SequenceRunner
from controllers.symbolic.viz import plot_world

abst = build_default()
spec = abst.world.spec
aut = Automaton.from_spec(spec)
avoid_names = default_avoid(abst)
print(f"Task: visit {' -> '.join(spec['visit_in_order'])}, always avoid {avoid_names}")
print("Memory (automaton):\n" + aut.describe())

t0 = time.time()
res = product_reach_avoid(abst, aut, avoid_names, verbose=True)
print(f"Signs written in {time.time() - t0:.1f} s ({res['n_rings']} rings)")

runner = SequenceRunner(abst, aut, res)
start = abst.world.start_pose
for c in abst.measurement_corners(start):       # whatever the sensor says at the start, there must be a sign
    runner.reset()
    if runner.step(c)[0] == "lost":
        sys.exit("The start has NO sign for some possible sensor reading: no guaranteed way.\n"
                 "Try: start a bit further from walls/cell edges, smaller cells, or more commands.")
runner.reset()
status, u = runner.step(start)
if status == "lost":
    sys.exit("The start has NO sign: no guaranteed way for this task. Try smaller cells / more commands.")
q0 = abst.quantize(start)
print(f"Start is signed: at most {res['ring'][runner.mode, q0]} moves of {abst.tau} s. First command {u}")

avoid_xy = avoid_map(abst, avoid_names)
rng = np.random.default_rng(0)
runs, trajs, n_moves = 200, [], []
for r in range(runs):
    runner.reset()
    x = np.array(start, float)
    traj, modes_seen = [(x[0], x[1], 0)], []
    for step in range(2000):
        status, u = runner.step(abst.random_measure(x, rng))     # the controller only sees a measurement
        modes_seen.append(runner.mode)
        assert status != "lost", f"run {r}: lost at step {step}"
        if status == "done":
            break
        path, x = abst.random_move(x, abst.inputs.index(u), rng)
        for xs, ys, _ in path[1:]:
            i = int(np.floor((xs - abst.world.x0) / abst.dx)); j = int(np.floor((ys - abst.world.y0) / abst.dx))
            assert 0 <= i < abst.nx and 0 <= j < abst.ny and not avoid_xy[i, j], f"run {r}: touched a forbidden cell"
            traj.append((xs, ys, runner.mode))
    else:
        raise AssertionError(f"run {r}: did not finish")
    assert modes_seen == sorted(modes_seen), "memory went backwards?!"
    n_moves.append(step); trajs.append(np.array(traj))
print(f"\nClosed-loop test: {runs}/{runs} runs visited {' -> '.join(spec['visit_in_order'])} in order, "
      f"never touched {avoid_names}, in {min(n_moves)} to {max(n_moves)} moves.")

out = ROOT / "outputs"; out.mkdir(exist_ok=True)
fig, ax = plt.subplots(figsize=(8, 6))
plot_world(abst.world, ax, abst=abst)
colors = ["tab:green", "tab:blue", "tab:red"]
for tr in trajs[:10]:
    for m in range(aut.n_modes - 1):
        seg = tr[tr[:, 2] == m]
        ax.plot(seg[:, 0], seg[:, 1], ".", ms=1.5, color=colors[m % 3],
                label=aut.names[m] if tr is trajs[0] else None)
ax.set_title("Step 3: 10 test runs, coloured by what the robot is looking for")
ax.legend(loc="lower left", bbox_to_anchor=(1.01, 0), fontsize=8, markerscale=6)
fig.savefig(out / "symbolic_step3_sequence.png", dpi=130, bbox_inches="tight")
print(f"Picture saved in {out}/symbolic_step3_sequence.png")
