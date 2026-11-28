# Symbolic controller

Goal: the prof gives a grid (size, obstacles, regions) and a task such as
"visit A, then B, then the target, and always avoid C". We build a controller that is
correct by construction, following the lecture *Symbolic control* (Saoud & Belamfedel Alaoui):

    real robot  --(1) abstraction-->  finite graph  --(2,3) synthesis-->  symbolic controller  --(4) refinement-->  real controller

| Step | Files | Status |
|---|---|---|
| 1. World + abstraction S = (Q, P, F) | `problem.yaml`, `config_symbolic.yaml`, `world.py`, `dynamics.py`, `abstraction.py`, `viz.py`, `tests/check_symbolic_step1.py` | done |
| 2. Reach-avoid synthesis (fixed point with Pre) | `synthesis.py`, `tests/check_symbolic_step2.py` | done |
| 3. "A then B, avoid C": product with an automaton | `automaton.py`, `tests/check_symbolic_step3.py` | done |
| 4. Plug into `common/` (Controller interface, simulator, metrics, plots) | `controller.py`, `tests/check_symbolic_step4.py` | done |

When the prof gives the real problem, edit only `problem.yaml`.
Check step 1: `python tests/check_symbolic_step1.py` (from the repo root, about 5 s).
Check step 2: `python tests/check_symbolic_step2.py` (about 15 s).
Check step 3: `python tests/check_symbolic_step3.py` (about 1 min).
Check step 4: `python tests/check_symbolic_step4.py` (about 1 min the first time, then seconds: the signs are cached).

Use it like the PID:

    from controllers.symbolic.controller import SymbolicController, task_path
    ctrl = SymbolicController(cfg)
    log = run(ctrl, task_path(ctrl.world), cfg, start_pose=ctrl.world.start_pose)

Each move lasts `tau` = 1.5 s: speed ramps up, cruises, ramps down, then the robot stands still for a moment
and averages 9 pose readings before choosing the next move. Start the robot a bit away from walls and cell
edges: the tests stop with a clear message if the start is not safely covered.
