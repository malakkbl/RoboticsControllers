# Robot controllers: PID / symbolic / RL

Same simulator, paths, metrics and config for every controller.

    config.yaml            shared: robot limits, noise, test paths, per-controller gains
    common/                simulator (robot.py), paths, metrics, plotting, runner, Controller interface
    controllers/pid.py     1) classical PID path tracker      (done)
    controllers/symbolic.py  2) symbolic controller           (todo)
    controllers/rl.py        3) reinforcement learning        (todo)
    notebooks/01_pid_controller.ipynb   experiments for the PID
    tests/run_all.py       quick regression on all preset paths

Setup: `pip install -r requirements.txt`, then `jupyter lab notebooks/` or `python tests/run_all.py`.

Controller interface (common/controller_base.py): `reset(path, dt)`, `step(pose, t) -> (v, w)`, `done`.
Values marked "(assumed)" in config.yaml must be measured on the real robot.
