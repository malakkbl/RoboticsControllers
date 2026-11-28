"""Checks for the deployment helpers (no ROS needed). Run: python tests/test_helpers.py"""
import math
import pathlib
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from common.config import load_config
from common.paths import Path, load_waypoints, path_from_file
from common.pose_utils import extract_pose, yaw_from_quat
from common.runner import run
from controllers.pid import PIDPathTracker


def write(tmp, name, text):
    p = pathlib.Path(tmp) / name
    p.write_text(text)
    return p


def raises(fn, *a):
    try:
        fn(*a)
    except ValueError:
        return True
    return False


def main():
    with tempfile.TemporaryDirectory() as tmp:
        # --- path files ---
        pts = load_waypoints(write(tmp, "a.csv", "# comment\nx,y\n0,0\n1.0, 0.5  # inline\n\n2;1\n"))
        assert pts.tolist() == [[0, 0], [1, 0.5], [2, 1]], pts
        assert load_waypoints(write(tmp, "b.yaml", "points:\n  - [0, 0]\n  - [1, 1]\n")).shape == (2, 2)
        assert load_waypoints(write(tmp, "c.yml", "- [0, 0]\n- [0, 2]\n")).shape == (2, 2)
        assert raises(load_waypoints, write(tmp, "d.csv", "0,0\n"))                  # one point only
        assert raises(load_waypoints, write(tmp, "e.csv", "0,0\n1,1\noops\n"))       # junk after the data
        assert raises(load_waypoints, write(tmp, "f.csv", "0,0\nnan,1\n"))           # not finite
        p = path_from_file(ROOT / "paths" / "line_1m.csv", origin=(1, 2))
        assert isinstance(p, Path) and np.allclose(p.xy[0], [1, 2]) and abs(p.length - 1.0) < 1e-9

    # --- pose conversion ---
    assert abs(yaw_from_quat(0, 0, 0, 1)) < 1e-12
    for th in (-3.0, -1.0, 0.0, 0.7, 3.0):
        assert abs(yaw_from_quat(0, 0, math.sin(th / 2), math.cos(th / 2)) - th) < 1e-9
    q = NS(x=0.0, y=0.0, z=math.sin(0.4), w=math.cos(0.4))                           # yaw 0.8
    pose = NS(position=NS(x=1.0, y=2.0, z=0.0), orientation=q)
    for kind, msg in {"PoseStamped": NS(pose=pose), "Pose": pose,
                      "PoseWithCovarianceStamped": NS(pose=NS(pose=pose)), "Odometry": NS(pose=NS(pose=pose)),
                      "Pose2D": NS(x=1.0, y=2.0, theta=0.8)}.items():
        x, y, th = extract_pose(msg, kind)
        assert (x, y) == (1.0, 2.0) and abs(th - 0.8) < 1e-9, kind
    assert raises(extract_pose, pose, "Nonsense")

    # --- the path file really works with the controller in the simulator ---
    cfg = load_config()
    path = path_from_file(ROOT / "paths" / "line_1m.csv")
    log = run(PIDPathTracker(cfg), path, cfg, start_pose=[0.0, 0.1, 0.0])
    end_err = math.hypot(log["x"][-1] - path.end[0], log["y"][-1] - path.end[1])
    assert log["reached"] and end_err < 0.1, (log["reached"], end_err)
    print(f"all helper checks passed (sim on paths/line_1m.csv: reached in {log['t'][-1]:.1f} s, end error {100 * end_err:.1f} cm)")


if __name__ == "__main__":
    main()
