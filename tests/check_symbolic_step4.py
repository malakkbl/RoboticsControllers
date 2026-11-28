"""Step 4 check: the symbolic controller in the SAME simulator as the PID (common/robot.py:
acceleration limits, 2-step command delay, noisy pose), through common/runner.py.

10 runs with different sensor-noise seeds. For each run we check, on the TRUE robot positions:
  - the task finished (A, then B, then target),
  - the robot body (radius robot_radius) never overlapped an obstacle or C, and never left the arena.
Prints time, number of moves and the smallest clearance. Saves outputs/symbolic_step4_*.png

Run from the repo root:  python tests/check_symbolic_step4.py
(first run: a few minutes to write the signs, then they are cached in outputs/symbolic_cache/)
"""
import sys, pathlib, copy
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from common.config import load_config
from common.runner import run
from common.plotting import plot_signals
from controllers.symbolic.controller import SymbolicController, task_path
from controllers.symbolic.viz import plot_world

cfg = load_config()
ctrl = SymbolicController(cfg)
world, abst = ctrl.world, ctrl.abst
path = task_path(world)
order = world.spec["visit_in_order"]
forbidden = world.rects("obstacle") + [r for n in world.spec.get("always_avoid", []) for r in world.rects(n)]


def clearance(x, y):
    """Distance from the robot CENTRE to the nearest forbidden rectangle or wall, minus the robot radius."""
    d = [x - world.bounds[0], y - world.bounds[1], world.bounds[2] - x, world.bounds[3] - y]
    for (a, b, c, e) in forbidden:
        dx = max(a - x, 0, x - c); dy = max(b - y, 0, y - e)
        d.append(np.hypot(dx, dy))
    return min(d) - abst.r_robot


print(f"Task: visit {' -> '.join(order)}, avoid {['obstacle', 'wall'] + world.spec.get('always_avoid', [])}")
print(f"{'seed':>4s} {'done':>5s} {'time[s]':>8s} {'moves':>6s} {'min clearance[cm]':>18s}")
logs, ok = [], True
for seed in range(10):
    c = copy.deepcopy(cfg); c["noise"]["seed"] = seed
    c["sim"]["max_time"] = 300.0                    # the task is long: ~45 moves of 1.5 s
    log = run(ctrl, path, c, start_pose=world.start_pose)
    modes = np.array(ctrl.mode_log)
    in_order = bool(np.all(np.diff(modes) >= 0)) and ctrl.done
    clr = min(clearance(x, y) for x, y in zip(log["x"], log["y"]))
    ok &= in_order and clr >= 0
    logs.append((log, modes))
    print(f"{seed:4d} {str(in_order):>5s} {log['t'][-1]:8.1f} {len(ctrl.move_log):6d} {100 * clr:18.1f}")
assert ok, "a run failed (task not finished in order, or the robot touched something)"
print("\nOK: every run did A -> B -> target in order and never touched an obstacle, C or a wall.")

out = ROOT / "outputs"; out.mkdir(exist_ok=True)
fig, ax = plt.subplots(figsize=(8, 6))
plot_world(world, ax, abst=abst)
colors = ["tab:green", "tab:blue", "tab:red"]
log, modes = logs[0]
for m in range(ctrl.aut.n_modes - 1):
    sel = modes == m
    ax.plot(log["x"][sel], log["y"][sel], ".", ms=2, color=colors[m % 3], label=ctrl.aut.names[m])
ax.set_title("Step 4: symbolic controller in common/robot.py (seed 0)")
ax.legend(loc="lower left", bbox_to_anchor=(1.01, 0), fontsize=8, markerscale=5)
fig.savefig(out / "symbolic_step4_run.png", dpi=130, bbox_inches="tight")
plot_signals(log, "Step 4: speeds (each move: ramp up, cruise, ramp down, stop)").savefig(out / "symbolic_step4_signals.png", dpi=110)
print(f"Pictures saved in {out}/symbolic_step4_*.png")
