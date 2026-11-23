"""The ONE interface every controller (PID, symbolic, RL) must follow.
Because of this, the same simulator, metrics and ROS wrapper work for all of them."""
from abc import ABC, abstractmethod


class Controller(ABC):
    @abstractmethod
    def reset(self, path, dt):
        """Called once before a run. `path` is a common.paths.Path (any objective is a path)."""

    @abstractmethod
    def step(self, pose, t):
        """pose = measured [x, y, theta]. Returns (v, w) velocity command."""

    @property
    @abstractmethod
    def done(self):
        """True when the objective is reached."""

    # optional hook so controllers can expose internals for plotting
    debug = None
