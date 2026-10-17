"""Map-derived camera viewpoints; no assessment coordinates are encoded."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, label
from .core import angle_difference


@dataclass
class Viewpoint:
    x: float
    y: float
    yaw: float
    visible: frozenset


class CoveragePlanner:
    def __init__(self, data, resolution, origin, clearance=0.20, spacing=0.65,
                 view_range=2.0, fov=1.35, headings=4, max_candidates=250,
                 max_goals=120, coverage_fraction=0.98):
        self.grid = np.asarray(data, dtype=int)
        self.resolution = float(resolution)
        self.origin = origin
        if (self.grid.ndim != 2 or resolution <= 0 or clearance <= 0
                or spacing <= 0 or view_range <= 0 or not 0 < fov < 2*math.pi
                or headings < 2 or max_candidates < 1 or max_goals < 1):
            raise ValueError('Invalid coverage parameters')
        self.free = (self.grid >= 0) & (self.grid < 25)
        # Treat beyond-map space as an obstacle, including fully free test maps.
        padded = np.pad(self.free, 1, constant_values=False)
        self.safe = (distance_transform_edt(padded)[1:-1, 1:-1]
                     * resolution >= clearance) & self.free
        self.spacing = spacing
        self.range = view_range
        self.fov = fov
        self.headings = headings
        self.max_candidates = max_candidates
        self.max_goals = max_goals
        self.fraction = coverage_fraction
        self.target_cells = set()
        self.coverable = set()
        self.truncated = False

    def world_to_cell(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x-ox, y-oy
        return (math.floor((-math.sin(yaw)*dx + math.cos(yaw)*dy)/self.resolution),
                math.floor((math.cos(yaw)*dx + math.sin(yaw)*dy)/self.resolution))

    def cell_to_world(self, row, col):
        ox, oy, yaw = self.origin
        x, y = (col+0.5)*self.resolution, (row+0.5)*self.resolution
        return ox+math.cos(yaw)*x-math.sin(yaw)*y, oy+math.sin(yaw)*x+math.cos(yaw)*y

    def in_bounds(self, row, col):
        return 0 <= row < self.grid.shape[0] and 0 <= col < self.grid.shape[1]

    def visible(self, row, col, yaw):
        """Ray-cast camera field of view, stopping at occupied/unknown cells."""
        cells = set()
        local_yaw = yaw - self.origin[2]
        # Angular sampling resolves a target sampling cell at maximum range.
        angular_step = min(0.04, 0.15 / self.range)
        for a in np.arange(local_yaw-self.fov/2, local_yaw+self.fov/2+angular_step,
                           angular_step):
            for d in np.arange(0.0, self.range, self.resolution*0.7):
                r = int(math.floor(row+0.5+math.sin(a)*d/self.resolution))
                c = int(math.floor(col+0.5+math.cos(a)*d/self.resolution))
                if not self.in_bounds(r, c) or not self.free[r, c]:
                    break
                if (r, c) in self.target_cells:
                    cells.add((r, c))
        return frozenset(cells)

    def plan(self, start_x, start_y):
        row, col = self.world_to_cell(start_x, start_y)
        if not self.in_bounds(row, col) or not self.safe[row, col]:
            raise ValueError('Starting pose is not in free space with robot clearance')
        components, _ = label(self.safe, structure=[[0,1,0],[1,1,1],[0,1,0]])
        reachable = components == components[row, col]
        # Include near-wall free space connected to the start as observation targets.
        raw, _ = label(self.free, structure=[[0,1,0],[1,1,1],[0,1,0]])
        raw_reachable = raw == raw[row, col]
        target_stride = max(1, round(0.15/self.resolution))
        self.target_cells = {(r, c) for r in range(0, self.grid.shape[0], target_stride)
                             for c in range(0, self.grid.shape[1], target_stride)
                             if raw_reachable[r, c]}
        stride = max(1, round(self.spacing/self.resolution))
        candidates = [(row, col)] + [(r, c)
                      for r in range(0, self.grid.shape[0], stride)
                      for c in range(0, self.grid.shape[1], stride)
                      if reachable[r, c] and (r, c) != (row, col)]
        if len(candidates) > self.max_candidates:
            # Uniformly subsample, retaining the starting point.
            indices = np.linspace(0, len(candidates)-1, self.max_candidates, dtype=int)
            candidates = [candidates[i] for i in indices]
            self.truncated = True
        options = []
        self.coverable = set()
        for r, c in candidates:
            x, y = self.cell_to_world(r, c)
            for k in range(self.headings):
                yaw = 2*math.pi*k/self.headings + self.origin[2]
                visible = self.visible(r, c, yaw)
                if visible:
                    options.append(Viewpoint(x, y, yaw, visible))
                    self.coverable.update(visible)
        if not self.coverable:
            raise ValueError('No camera viewpoints can be generated')
        seen, route = set(), []
        x, y = start_x, start_y
        while options and len(seen) < self.fraction * len(self.coverable):
            best = max(options, key=lambda v: len(v.visible-seen)
                       / (1 + math.hypot(v.x-x, v.y-y)))
            if not best.visible-seen:
                break
            route.append(best)
            seen.update(best.visible)
            x, y = best.x, best.y
            options.remove(best)
            if len(route) >= self.max_goals:
                self.truncated = len(seen) < self.fraction * len(self.coverable)
                break
        return route


def returned_home(current, home, tolerance):
    return math.hypot(current[0]-home[0], current[1]-home[1]) <= tolerance
