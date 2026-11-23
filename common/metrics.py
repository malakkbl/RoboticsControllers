import numpy as np


def compute_metrics(log, path):
    """Same metrics for every controller so results are comparable."""
    cte = log["cte"]
    dt = np.diff(log["t"]).mean() if len(log["t"]) > 1 else 1.0
    end_err = float(np.hypot(log["x"][-1] - path.end[0], log["y"][-1] - path.end[1]))
    jerk_w = np.abs(np.diff(log["w"])).sum() if len(log["w"]) > 1 else 0.0   # total steering effort change
    return {
        "reached": bool(log["reached"]),
        "time_s": float(log["t"][-1]),
        "rmse_cte_m": float(np.sqrt(np.mean(cte ** 2))),
        "max_cte_m": float(cte.max()),
        "end_error_m": end_err,
        "mean_speed": float(np.mean(np.abs(log["v"]))),
        "steering_effort": float(jerk_w),
    }
