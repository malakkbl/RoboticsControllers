"""Pictures to check the world and the abstraction with your own eyes."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

COLORS = {"obstacle": "0.25", "target": "tab:red", "A": "tab:green", "B": "tab:blue", "C": "tab:orange"}


def plot_world(world, ax=None, abst=None):
    """Prof grid, obstacles and regions. With `abst`, also shades the inflated 'avoid' cells."""
    ax = ax or plt.subplots(figsize=(7, 5.5))[1]
    xmin, ymin, xmax, ymax = world.bounds
    if abst is not None:   # light grey: cells the robot centre must not enter (inflated obstacles + walls)
        bad = abst.touches_xy["obstacle"] | abst.touches_xy["wall"]
        ax.imshow(bad.T, origin="lower", extent=(xmin, xmax, ymin, ymax), cmap="Greys", alpha=0.25,
                  vmin=0, vmax=1.5, interpolation="nearest")
    for name in world.names:
        col = COLORS.get(name, "tab:purple")
        for k, (a, b, c, d) in enumerate(world.rects(name)):
            ax.add_patch(Rectangle((a, b), c - a, d - b, color=col, alpha=0.85 if name == "obstacle" else 0.45,
                                   label=name if k == 0 else None))
            if name != "obstacle":
                ax.text((a + c) / 2, (b + d) / 2, name, ha="center", va="center", fontsize=9, weight="bold")
    for c in range(world.cols + 1):
        ax.plot([world.x0 + c * world.cs] * 2, [ymin, ymax], "k-", lw=0.6)
    for r in range(world.rows + 1):
        ax.plot([xmin, xmax], [world.y0 + r * world.cs] * 2, "k-", lw=0.6)
    x, y, th = world.start_pose
    ax.plot(x, y, "ko"); ax.arrow(x, y, 0.08 * np.cos(th), 0.08 * np.sin(th), head_width=0.03, color="k")
    ax.set_xlim(xmin - 0.05, xmax + 0.05); ax.set_ylim(ymin - 0.05, ymax + 0.05)
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    return ax


def plot_transition(abst, q, p, ax=None):
    """One transition: the start cell q (black), the reachable box Y (red dashed),
    the successor cells F(q, p) in (x, y) (blue) and the swept area (green dotted)."""
    ax = ax or plt.subplots(figsize=(5, 5))[1]
    lo, hi = abst.cell_bounds(q)
    ax.add_patch(Rectangle(lo[:2], *(hi - lo)[:2], fill=False, ec="k", lw=2, label="cell q"))
    succ = [s for s in abst.post(q, p) if s != abst.Q_OUT]
    xy = {tuple(abst.ijk(s)[:2]) for s in succ}
    for (i, j) in xy:
        a = abst.world.x0 + i * abst.dx; b = abst.world.y0 + j * abst.dx
        ax.add_patch(Rectangle((a, b), abst.dx, abst.dx, color="tab:blue", alpha=0.25))
    Ylo, Yhi = abst.Y_lo[q, p], abst.Y_hi[q, p]
    ax.add_patch(Rectangle(Ylo[:2], *(Yhi - Ylo)[:2], fill=False, ec="tab:red", ls="--", lw=2, label="Y (reachable box)"))
    slo, shi = abst.sweep_lo[q, p], abst.sweep_hi[q, p]
    a, b = abst.world.x0 + slo * abst.dx
    c, d = abst.world.x0 + (shi + 1) * abst.dx
    ax.add_patch(Rectangle((a, b), c - a, d - b, fill=False, ec="tab:green", ls=":", lw=2, label="swept cells"))
    ks = sorted({int(abst.ijk(s)[2]) for s in succ})
    v, w = abst.inputs[p]
    ax.set_title(f"q={q}, input (v={v}, w={w}) for {abst.tau}s\n"
                 f"{len(succ)} successors, heading slices {ks}{'  + Q_OUT' if abst.out[q, p] else ''}", fontsize=9)
    ax.set_xlim(a - abst.dx, c + abst.dx); ax.set_ylim(b - abst.dx, d + abst.dx)
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="lower right")
    return ax
