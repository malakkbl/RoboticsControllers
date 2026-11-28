"""
This file reads problem.yaml and turns cells into rectangles defined in metres. It has one class, GridWorld. Here is what it does:

+ from_yaml opens problem.yaml and reads it into Python.
+ __init__ stores the numbers: cols, rows, cell size, the obstacle list, the regions and the start pose. It also checks that every cell we wrote is inside the grid. That catches typos like [9, 2] on an 8-column grid.
+ cell_rect([col, row]) gives the cell's rectangle in metres, as (xmin, ymin, xmax, ymax). With 0.30 m cells, cell [3, 2] becomes (0.9, 0.6, 1.2, 0.9). It's just column × size and row × size.
+ rects("A") gives all the rectangles of region A. rects("obstacle") gives all the obstacles.
+ bounds gives the whole arena: (0, 0, 2.4, 1.8).

"""

"""The prof's grid world: size, obstacles, named regions (A, B, C, target...), start pose and spec.

Everything here is in metres. A region is a list of rectangles (xmin, ymin, xmax, ymax),
one rectangle per prof cell.
"""
from pathlib import Path
import yaml

HERE = Path(__file__).resolve().parent


class GridWorld:
    def __init__(self, problem):
        g = problem["grid"]
        self.cols, self.rows = int(g["cols"]), int(g["rows"])
        self.cs = float(g["cell_size"])
        self.x0, self.y0 = map(float, g.get("origin", [0.0, 0.0]))
        self.width, self.height = self.cols * self.cs, self.rows * self.cs
        self.obstacle_cells = [tuple(c) for c in problem.get("obstacles", [])]
        self.region_cells = {name: [tuple(c) for c in cells]
                             for name, cells in problem.get("regions", {}).items()}
        self.start_pose = problem.get("start_pose", [self.x0 + self.cs / 2, self.y0 + self.cs / 2, 0.0])
        self.spec = problem.get("spec", {})
        for c in self.obstacle_cells + [c for cs in self.region_cells.values() for c in cs]:
            assert 0 <= c[0] < self.cols and 0 <= c[1] < self.rows, f"cell {c} is outside the grid"

    @classmethod
    def from_yaml(cls, path=None):
        with open(path or HERE / "problem.yaml") as f:
            return cls(yaml.safe_load(f))

    def cell_rect(self, cell):
        """Rectangle (xmin, ymin, xmax, ymax) in metres of a prof cell [col, row]."""
        c, r = cell
        return (self.x0 + c * self.cs, self.y0 + r * self.cs,
                self.x0 + (c + 1) * self.cs, self.y0 + (r + 1) * self.cs)

    @property
    def bounds(self):
        return (self.x0, self.y0, self.x0 + self.width, self.y0 + self.height)

    def rects(self, name):
        """Rectangles of a region by name. 'obstacle' is the obstacle list."""
        cells = self.obstacle_cells if name == "obstacle" else self.region_cells[name]
        return [self.cell_rect(c) for c in cells]

    @property
    def names(self):
        return ["obstacle"] + list(self.region_cells)
