"""Run any Controller on any Path in the simulator and log everything."""
import numpy as np
from .robot import DiffDriveSim


def run(controller, path, cfg, start_pose=None, sim=None):
    start_pose = cfg["start_pose"] if start_pose is None else start_pose
    sim = sim or DiffDriveSim(cfg, start_pose)
    dt = sim.dt
    controller.reset(path, dt)
    keys = ["t", "x", "y", "th", "v", "w", "v_cmd", "w_cmd", "cte", "wl", "wr", "tx", "ty"]
    log = {k: [] for k in keys}
    n_max = int(cfg["sim"]["max_time"] / dt)
    reached = False
    for k in range(n_max):
        t = k * dt
        v_cmd, w_cmd = controller.step(sim.measure(), t)
        sim.step(v_cmd, w_cmd)
        wl, wr = sim.wheel_speeds()
        tgt = (controller.debug or {}).get("target", (np.nan, np.nan))
        for key, val in zip(keys, [t, sim.x, sim.y, sim.th, sim.v, sim.w, v_cmd, w_cmd,
                                   path.cross_track(sim.true_pose[:2]), wl, wr, tgt[0], tgt[1]]):
            log[key].append(val)
        if controller.done:
            reached = True
            break
    out = {k: np.asarray(v) for k, v in log.items()}
    out["reached"] = reached
    return out
