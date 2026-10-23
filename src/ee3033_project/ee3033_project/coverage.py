"""Map-derived camera viewpoints; no assessment coordinates are encoded."""
from dataclasses import dataclass
import math
import time
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
                 max_goals=120, coverage_fraction=0.98, candidate_limit=1000):
        self.grid = np.asarray(data, dtype=int)
        self.resolution = float(resolution)
        self.origin = origin
        if (self.grid.ndim != 2 or resolution <= 0 or clearance <= 0
                or spacing <= 0 or view_range <= 0 or not 0 < fov < 2*math.pi
                or headings < 2 or max_candidates < 1 or max_goals < 1
                or candidate_limit < 1 or not 0 < coverage_fraction <= 1):
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
        self.candidate_limit = candidate_limit
        self.max_goals = max_goals
        self.fraction = coverage_fraction
        self.target_cells = set()
        self.coverable = set()
        self.truncated = False
        self.options = ()
        self.unobservable = set()
        self.candidate_threshold_exceeded = False

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

    def prepare(self, start_x, start_y, deadline=None, cancel=None):
        """Build stable candidates; stop cooperatively on timeout or operator Stop."""
        def check_budget():
            if cancel is not None and cancel.is_set():
                raise RuntimeError('Search planning canceled')
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError('Search planning time limit exceeded')

        check_budget()
        self.truncated = False
        self.options = ()
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
        # Preserve all candidates: uniform truncation silently removes critical
        # corner viewpoints. max_candidates is now a warning threshold only.
        self.candidate_threshold_exceeded = len(candidates) > self.max_candidates
        if len(candidates) > self.candidate_limit:
            raise ValueError('Too many search candidates; increase viewpoint_spacing or the explicit candidate_limit')
        options = []
        self.coverable = set()
        for r, c in candidates:
            check_budget()
            x, y = self.cell_to_world(r, c)
            for k in range(self.headings):
                check_budget()
                yaw = 2*math.pi*k/self.headings + self.origin[2]
                visible = self.visible(r, c, yaw)
                if visible:
                    options.append(Viewpoint(x, y, yaw, visible))
                    self.coverable.update(visible)
        # plan() consumes its own working copy, never the runtime candidate pool.
        self.options = tuple(options)
        self.unobservable = self.target_cells - self.coverable
        if not self.coverable:
            raise ValueError('No camera viewpoints can be generated')
        check_budget()
        return self.options

    def plan(self, start_x, start_y):
        options = list(self.prepare(start_x, start_y))
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

    def next_viewpoint(self, x, y, observed, excluded=(), yaw=None,
                       linear_speed=0.12, angular_speed=0.4,
                       observation_seconds=3.0):
        """Select new visible cells per estimated second of movement and observation.

        Estimate only: Nav2 may take longer around obstacles. The current yaw
        is optional for compatibility with existing callers/tests.
        """
        observed = set(observed)
        excluded = set(excluded)
        best, best_score = None, -1.0
        for index, v in enumerate(self.options):
            if index in excluded:
                continue
            gain = len(v.visible - observed)
            if gain == 0:
                continue
            dx, dy = v.x-x, v.y-y
            distance = math.hypot(dx, dy)
            # Travel and final orientation matter; avoid rewarding extra stops.
            travel = distance / max(0.01, linear_speed)
            heading = math.atan2(dy, dx) if distance > 0.05 else (yaw if yaw is not None else v.yaw)
            rotation = abs(angle_difference(v.yaw, heading))
            if yaw is not None:
                rotation += abs(angle_difference(heading, yaw))
            turn = rotation / max(0.05, angular_speed)
            cost = max(0.0, observation_seconds) + travel + turn + 1.0
            score = gain / cost
            if score > best_score:
                best, best_score = index, score
        return best


def returned_home(current, home, tolerance):
    return math.hypot(current[0]-home[0], current[1]-home[1]) <= tolerance
