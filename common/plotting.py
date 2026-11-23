import matplotlib.pyplot as plt


def plot_run(log, path, title="", ax=None):
    ax = ax or plt.subplots(figsize=(5, 5))[1]
    ax.plot(path.xy[:, 0], path.xy[:, 1], "k--", lw=1.5, label="reference path")
    ax.plot(log["x"], log["y"], "C0", lw=2, label="robot")
    ax.plot(log["x"][0], log["y"][0], "go", label="start")
    ax.plot(*path.end, "r*", ms=12, label="end")
    ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_title(title)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    return ax


def plot_signals(log, title=""):
    fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    axs[0].plot(log["t"], log["cte"] * 100); axs[0].set_ylabel("cross-track error [cm]")
    axs[1].plot(log["t"], log["v_cmd"], label="cmd"); axs[1].plot(log["t"], log["v"], label="actual")
    axs[1].set_ylabel("v [m/s]"); axs[1].legend()
    axs[2].plot(log["t"], log["w_cmd"], label="cmd"); axs[2].plot(log["t"], log["w"], label="actual")
    axs[2].set_ylabel("w [rad/s]"); axs[2].set_xlabel("t [s]"); axs[2].legend()
    for a in axs: a.grid(alpha=.3)
    fig.suptitle(title); fig.tight_layout()
    return fig
