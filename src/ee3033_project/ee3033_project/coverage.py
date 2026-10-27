"""Map-derived camera viewpoints; no assessment coordinates are encoded."""
from dataclasses import dataclass
import math
import time
import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt, label, binary_dilation
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
                 max_goals=120, coverage_fraction=0.98, candidate_limit=1000,
                 camera_model=None):
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
        self.target_stride = max(1, round(.15/self.resolution))
        self.target_cells = set()
        self.coverable = set()
        self.truncated = False
        self.options = ()
        self.unobservable = set()
        self.candidate_threshold_exceeded = False
        self.camera_model = camera_model
        self.path_cache = None
        self.gap_pass = False
        self.blocked_cells = set()
        self.map_partial = False
        self.declared_partial = False
        self.native_links = {}
        self.native_probes = []
        self.clearance = clearance

    def travel_field(self, x, y, deadline=None, cancel=None):
        """Shortest collision-clear grid paths; no Euclidean shortcuts through walls."""
        if cancel is not None and cancel.is_set():
            raise RuntimeError('Search planning canceled')
        if deadline is not None and time.monotonic() >= deadline:
            raise TimeoutError('Search path planning time limit exceeded')
        start = self.world_to_cell(x, y)
        if self.path_cache is not None and self.path_cache[0] == start:
            return self.path_cache[1:]
        distances = np.full(self.grid.shape, np.inf)
        parents = {}
        if not self.in_bounds(*start) or not self.safe[start]:
            return distances, parents
        distances[start] = 0.
        queue = [(0., start)]
        count = 0
        while queue:
            distance, cell = heapq.heappop(queue)
            if distance != distances[cell]:
                continue
            count += 1
            if count % 256 == 0:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError('Search planning canceled')
                if deadline is not None and time.monotonic() >= deadline:
                    raise TimeoutError('Search path planning time limit exceeded')
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                other = (cell[0]+dr, cell[1]+dc)
                candidate = distance+self.resolution
                if self.in_bounds(*other) and self.safe[other] and candidate < distances[other]:
                    distances[other] = candidate
                    parents[other] = cell
                    heapq.heappush(queue, (candidate, other))
            for other, length in self.native_links.get(cell, ()):
                candidate = distance+length
                if candidate < distances[other]:
                    distances[other] = candidate
                    parents[other] = cell
                    heapq.heappush(queue, (candidate, other))
        self.path_cache = (start, distances, parents)
        return distances, parents

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

    def visible(self, row, col, yaw, camera_model=None):
        """Ray-cast camera field of view, stopping at occupied/unknown cells."""
        cells = set()
        x, y = self.cell_to_world(row, col)
        local_yaw = yaw - self.origin[2]
        origin_row, origin_col = row+.5, col+.5
        camera = camera_model if camera_model is not None else self.camera_model
        if camera is not None:
            dx, dy = camera.translation[:2]
            cx = x+math.cos(yaw)*dx-math.sin(yaw)*dy
            cy = y+math.sin(yaw)*dx+math.cos(yaw)*dy
            ox, oy, origin_yaw = self.origin
            origin_row = (-math.sin(origin_yaw)*(cx-ox)+math.cos(origin_yaw)*(cy-oy))/self.resolution
            origin_col = (math.cos(origin_yaw)*(cx-ox)+math.sin(origin_yaw)*(cy-oy))/self.resolution
            local_yaw += camera.yaw
        # Angular sampling resolves a target sampling cell at maximum range.
        angular_step = min(0.04, 0.15 / self.range)
        for a in np.arange(local_yaw-self.fov/2, local_yaw+self.fov/2+angular_step,
                           angular_step):
            for d in np.arange(0.0, self.range, self.resolution*0.7):
                r = int(math.floor(origin_row+math.sin(a)*d/self.resolution))
                c = int(math.floor(origin_col+math.cos(a)*d/self.resolution))
                if not self.in_bounds(r, c) or not self.free[r, c]:
                    break
                if (r, c) in self.target_cells:
                    cells.add((r, c))
        if camera is not None and cells:
            ordered = sorted(cells)
            world = np.array([self.cell_to_world(*cell) for cell in ordered])
            dx, dy = world[:, 0]-x, world[:, 1]-y
            relative = np.column_stack([math.cos(yaw)*dx+math.sin(yaw)*dy,
                                        -math.sin(yaw)*dx+math.cos(yaw)*dy])
            valid = camera.contains_floor(relative)
            cells = {cell for cell, good in zip(ordered, valid) if good}
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
        self.components = components
        reachable = components == components[row, col]
        self.reachable = reachable
        # Include near-wall free space connected to the start as observation targets.
        raw, _ = label(self.free, structure=[[0,1,0],[1,1,1],[0,1,0]])
        raw_reachable = raw == raw[row, col]
        target_stride = max(1, round(0.15/self.resolution))
        self.target_cells = {(r, c) for r in range(0, self.grid.shape[0], target_stride)
                             for c in range(0, self.grid.shape[1], target_stride)
                             if raw_reachable[r, c]}
        self.target_stride = target_stride
        self.blocked_cells = {p for p in self.target_cells if not reachable[p] and self.safe[p]}
        self.map_partial = bool(self.declared_partial or np.any(raw_reachable & binary_dilation(self.grid < 0)))
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
        # One targeted gap pass: fill holes left by the coarse position/heading
        # lattice without silently dropping original viewpoints.
        remaining = self.target_cells-self.coverable
        added = 0
        for r, c in candidates:
            if not remaining or added >= self.max_candidates:
                break
            check_budget()
            x, y = self.cell_to_world(r, c)
            nearest = min(remaining, key=lambda cell: (cell[0]-r)**2+(cell[1]-c)**2)
            tx, ty = self.cell_to_world(*nearest)
            yaw = math.atan2(ty-y, tx-x)-(self.camera_model.yaw if self.camera_model else 0.)
            visible = self.visible(r, c, yaw)
            if visible & remaining:
                options.append(Viewpoint(x, y, yaw, visible))
                self.coverable.update(visible)
                remaining -= visible
                added += 1
        self.options = tuple(options)
        self.gap_pass = added > 0
        # Repair coarse-lattice holes, including views behind a wall reached
        # through a passage. Never shrink the target set to the candidate union.
        existing = {(round(v.x, 5), round(v.y, 5), round(v.yaw, 3)) for v in options}
        for spacing in (.325, .15):
            remaining = self.target_cells-self.coverable
            if not remaining:
                break
            stride = max(1, round(spacing/self.resolution))
            for r in range(0, self.grid.shape[0], stride):
                for c in range(0, self.grid.shape[1], stride):
                    check_budget()
                    if not reachable[r, c] or not remaining:
                        continue
                    x, y = self.cell_to_world(r, c)
                    # Different missing patches get their own facing direction.
                    nearby = sorted(remaining, key=lambda p: (p[0]-r)**2+(p[1]-c)**2)
                    for target in nearby[:3]:
                        tx, ty = self.cell_to_world(*target)
                        if math.hypot(tx-x, ty-y) > self.range:
                            break
                        yaw = math.atan2(ty-y, tx-x)-(self.camera_model.yaw if self.camera_model else 0.)
                        key = (round(x, 5), round(y, 5), round(yaw, 3))
                        if key in existing:
                            continue
                        visible = self.visible(r, c, yaw)
                        if not visible & remaining:
                            continue
                        if len(options) >= self.candidate_limit*self.headings:
                            raise ValueError('Adaptive search candidate limit reached; search is incomplete')
                        options.append(Viewpoint(x, y, yaw, visible))
                        existing.add(key)
                        self.coverable.update(visible)
                        remaining -= visible
                        self.gap_pass = True
        self.options = tuple(options)
        self.unobservable = self.target_cells - self.coverable
        self.native_probes = []
        for component in sorted(set(int(components[p]) for p in self.blocked_cells)-{0}):
            cells = np.argwhere(components == component)
            # One nearest entry per disconnected component, explicitly checked
            # by Nav2 rather than silently dropping the far side of a wall.
            r, c = min(cells, key=lambda p: (p[0]-row)**2+(p[1]-col)**2)
            x, y = self.cell_to_world(r, c)
            target = min(self.target_cells, key=lambda p: (p[0]-r)**2+(p[1]-c)**2)
            tx, ty = self.cell_to_world(*target)
            self.native_probes.append((component, Viewpoint(x, y, math.atan2(ty-y, tx-x), frozenset())))
        if not self.coverable:
            raise ValueError('No camera viewpoints can be generated')
        check_budget()
        return self.options

    def accept_native_connection(self, component, points, deadline=None, cancel=None):
        """Admit only a native path whose entire sampled centre path is safe."""
        if len(points) < 2:
            return False
        length = 0.
        for a, b in zip(points, points[1:]):
            d = math.dist(a, b)
            length += d
            for t in np.linspace(0., 1., max(2, math.ceil(d/(self.resolution/2))+1)):
                cell = self.world_to_cell(a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
                if not self.in_bounds(*cell) or not self.safe[cell]:
                    return False
        a, b = self.world_to_cell(*points[0]), self.world_to_cell(*points[-1])
        self.native_links.setdefault(a, []).append((b, length))
        self.native_links.setdefault(b, []).append((a, length))
        self.path_cache = None
        self.reachable |= self.components == component
        self.blocked_cells = {p for p in self.blocked_cells if self.components[p] != component}
        options = list(self.options)
        stride = max(1, round(.325/self.resolution))
        for r in range(0, self.grid.shape[0], stride):
            for c in range(0, self.grid.shape[1], stride):
                if cancel is not None and cancel.is_set():
                    raise RuntimeError('Search planning canceled')
                if deadline is not None and time.monotonic() >= deadline:
                    self.options = tuple(options)
                    self.unobservable = self.target_cells-self.coverable
                    return True
                if self.components[r, c] != component:
                    continue
                x, y = self.cell_to_world(r, c)
                for k in range(self.headings):
                    yaw = k*math.tau/self.headings+self.origin[2]
                    visible = self.visible(r, c, yaw)
                    if visible and len(options) < self.candidate_limit*self.headings:
                        options.append(Viewpoint(x, y, yaw, visible))
                        self.coverable.update(visible)
        self.options = tuple(options)
        self.unobservable = self.target_cells-self.coverable
        return True

    def completeness(self, observed):
        missing = self.target_cells-set(observed)
        mask = np.zeros(self.grid.shape, dtype=bool)
        for r, c in missing:
            mask[r, c] = True
        # Label on the target lattice, whose samples are usually 0.15 m apart.
        lattice = mask[::self.target_stride, ::self.target_stride]
        groups, count = label(lattice, structure=np.ones((3, 3), dtype=int))
        largest = max(np.bincount(groups.ravel())[1:], default=0)*(
            self.target_stride*self.resolution)**2
        fraction = len(set(observed) & self.target_cells)/max(1, len(self.target_cells))
        return dict(complete=bool(not self.map_partial and not (self.blocked_cells-set(observed))
                    and fraction >= self.fraction and largest < .09),
                    observed_fraction=fraction, largest_missing_patch=float(largest),
                    blocked_cells=len(self.blocked_cells-set(observed)), unknown_cells=int(np.sum(self.grid < 0)),
                    candidate_uncovered_cells=len(self.target_cells-self.coverable),
                    missing_cells=len(missing), map_partial=self.map_partial)

    def lookahead(self, x, y, observed, excluded=(), yaw=0., linear_speed=.12,
                  angular_speed=.4, observation_seconds=.8, path_lengths=None,
                  deadline=None, cancel=None):
        """Three-step beam search maximizing area under a 60-second coverage curve."""
        try:
            ranked = self.rank_viewpoints(x, y, observed, excluded, yaw, linear_speed,
                angular_speed, observation_seconds, path_lengths, deadline, cancel)
        except TimeoutError:
            return None
        if not ranked:
            return None
        fallback = ranked[0][1]
        shortlist = [i for _, i in ranked[:8]]
        valid = {i for _, i in ranked}
        for sector in range(8):
            candidates = [i for i in valid if int((math.atan2(self.options[i].y-y,
                self.options[i].x-x) % math.tau)/(math.tau/8)) == sector]
            if candidates:
                i = max(candidates, key=lambda i: len(self.options[i].visible-set(observed)))
                if i not in shortlist:
                    shortlist.append(i)
        initial = Viewpoint(x, y, yaw, frozenset())
        beam = [(0., 0., (), frozenset(observed), initial)]
        best = None
        for _ in range(3):
            expanded = []
            for utility, elapsed, route, seen, previous in beam:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError('Search planning canceled')
                if deadline is not None and time.monotonic() >= deadline:
                    return best[2][0] if best else fallback
                try:
                    distances, _ = self.travel_field(previous.x, previous.y, deadline, cancel)
                except TimeoutError:
                    return best[2][0] if best else fallback
                for i in shortlist:
                    if i in route:
                        continue
                    v = self.options[i]
                    gain = len(v.visible-seen)
                    if not gain:
                        continue
                    distance = float(distances[self.world_to_cell(v.x, v.y)])
                    if not route and path_lengths is not None:
                        distance = path_lengths.get(i, distance)
                    if not math.isfinite(distance):
                        continue
                    heading = math.atan2(v.y-previous.y, v.x-previous.x) if distance > .05 else previous.yaw
                    turn = abs(angle_difference(heading, previous.yaw))+abs(angle_difference(v.yaw, heading))
                    arrival = elapsed+distance/max(.01, linear_speed)+turn/max(.05, angular_speed)+observation_seconds+1.
                    value = utility+gain*max(0., 60.-arrival)
                    entry = (value, arrival, route+(i,), seen | v.visible, v)
                    expanded.append(entry)
                    if best is None or (value, -arrival) > (best[0], -best[1]):
                        best = entry
            if not expanded:
                break
            beam = sorted(expanded, key=lambda e: (-e[0], e[1], e[2]))[:12]
        return best[2][0] if best and best[0] > 0 else fallback

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
        ranked = self.rank_viewpoints(x, y, observed, excluded, yaw, linear_speed,
                                     angular_speed, observation_seconds)
        return ranked[0][1] if ranked else None

    def rank_viewpoints(self, x, y, observed, excluded=(), yaw=None,
                        linear_speed=.12, angular_speed=.4, observation_seconds=3.,
                        path_lengths=None, deadline=None, cancel=None):
        observed = set(observed)
        excluded = set(excluded)
        distances, _ = self.travel_field(x, y, deadline, cancel)
        ranked = []
        for index, v in enumerate(self.options):
            if index % 32 == 0:
                if cancel is not None and cancel.is_set():
                    raise RuntimeError('Search planning canceled')
                if deadline is not None and time.monotonic() >= deadline:
                    break
            if index in excluded:
                continue
            gain = len(v.visible - observed)
            if gain == 0:
                continue
            dx, dy = v.x-x, v.y-y
            cell = self.world_to_cell(v.x, v.y)
            distance = float(distances[cell]) if self.in_bounds(*cell) else math.inf
            if path_lengths is not None and index in path_lengths:
                distance = path_lengths[index]
            if not math.isfinite(distance):
                continue
            # Travel and final orientation matter; avoid rewarding extra stops.
            travel = distance / max(0.01, linear_speed)
            heading = math.atan2(dy, dx) if distance > 0.05 else (yaw if yaw is not None else v.yaw)
            rotation = abs(angle_difference(v.yaw, heading))
            if yaw is not None:
                rotation += abs(angle_difference(heading, yaw))
            turn = rotation / max(0.05, angular_speed)
            cost = max(0.0, observation_seconds) + travel + turn + 1.0
            score = gain / cost
            ranked.append((score, index))
        return sorted(ranked, key=lambda entry: (-entry[0], entry[1]))


def returned_home(current, home, tolerance):
    return math.hypot(current[0]-home[0], current[1]-home[1]) <= tolerance
