"""Quick regression: PID on every preset path. Run: python tests/run_all.py"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from common.config import load_config
from common.paths import make_path
from common.runner import run
from common.metrics import compute_metrics
from controllers.pid import PIDPathTracker

cfg = load_config()
print(f"{'path':10s} {'reached':8s} {'time':>6s} {'rmse[cm]':>9s} {'max[cm]':>8s} {'end[cm]':>8s}")
for name, spec in cfg["paths"].items():
    path = make_path(spec)
    log = run(PIDPathTracker(cfg), path, cfg)
    m = compute_metrics(log, path)
    print(f"{name:10s} {str(m['reached']):8s} {m['time_s']:6.1f} {100*m['rmse_cte_m']:9.2f} {100*m['max_cte_m']:8.2f} {100*m['end_error_m']:8.2f}")
