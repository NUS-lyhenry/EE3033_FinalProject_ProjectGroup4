"""Choose a short, known-free escape from a stalled mapping position."""
import math
import time
import numpy as np
from scipy.ndimage import distance_transform_edt
from .coverage import CoveragePlanner


def mapping_escape_goal(data, resolution, origin, pose, safe_mask=None, requested_clearance=.15,
                        stalled_goal=None):
    """Return a Nav2 goal in more open mapped space, or None.

    A conservative cell-diagonal allowance protects the entire straight
    connector. Unknown cells and points outside the map are never traversed.
    This selects a goal; Nav2 and the final velocity gate still control motion.
    """
    if pose is None or not np.isfinite([resolution, requested_clearance, *origin, *pose]).all():
        return None
    grid = np.asarray(data)
    if grid.ndim != 2 or resolution <= 0 or requested_clearance <= 0:
        return None
    free = (grid >= 0) & (grid < 25)
    clearance = distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]*resolution
    minimum = max(.15, requested_clearance) + resolution/math.sqrt(2.)
    safe = free & (clearance >= minimum)
    if safe_mask is not None:
        mask = np.asarray(safe_mask, dtype=bool)
        if mask.shape != grid.shape:
            return None
        safe &= mask
    planner = CoveragePlanner(grid, resolution, origin, clearance=minimum)
    start = planner.world_to_cell(*pose[:2])
    if not planner.in_bounds(*start) or not safe[start]:
        return None
    current = clearance[start]
    radius = math.ceil(.70/resolution)+1
    r0, r1 = max(0, start[0]-radius), min(grid.shape[0], start[0]+radius+1)
    c0, c1 = max(0, start[1]-radius), min(grid.shape[1], start[1]+radius+1)
    candidates = np.argwhere(safe[r0:r1, c0:c1] &
                             (clearance[r0:r1, c0:c1] >= max(.25, current+.05)))
    candidates += (r0, c0)
    choices = []
    for row, col in candidates:
        x, y = planner.cell_to_world(int(row), int(col))
        distance = math.hypot(x-pose[0], y-pose[1])
        if not .40 <= distance <= .70:
            continue
        cells = np.rint(np.linspace(start, (row, col),
                                   max(2, math.ceil(distance/resolution*4)+1))).astype(int)
        if not np.all(safe[cells[:, 0], cells[:, 1]]):
            continue
        score = clearance[row, col] - .2*abs(distance-.55)
        choices.append((score, float(x), float(y)))
    # Near an obstacle, first move toward open space. In an already open
    # position, a short leg toward the blocked goal is more useful than an
    # arbitrary move still farther away from that goal.
    if choices and (current < .35 or stalled_goal is None):
        _, x, y = max(choices)
        return (x, y, math.atan2(y-pose[1], x-pose[0]))
    if stalled_goal is None or not np.isfinite(stalled_goal[:2]).all():
        return None
    # In an already-open area, split a long, repeatedly stalled route into a
    # short intermediate leg. Only use cells on a complete safe-grid path and
    # check the whole direct connector; never use partial Dijkstra results.
    planner.safe = safe
    try:
        distances, parents = planner.travel_field(*pose[:2], deadline=time.monotonic()+.25)
    except (TimeoutError, RuntimeError):
        return None
    reachable = np.argwhere(np.isfinite(distances))
    if not len(reachable):
        return None
    target = np.asarray(planner.world_to_cell(*stalled_goal[:2]))
    i = int(np.argmin(np.sum((reachable-target)**2, axis=1)))
    end = tuple(map(int, reachable[i]))
    if np.linalg.norm(reachable[i]-target)*resolution > .25:
        return None
    path = []
    while end != start and end in parents:
        path.append(end)
        end = parents[end]
    if end != start:
        return None
    best = None
    for row, col in reversed(path):
        x, y = planner.cell_to_world(row, col)
        distance = math.hypot(x-pose[0], y-pose[1])
        if distance > .70:
            break
        if distance < .40:
            continue
        cells = np.rint(np.linspace(start, (row, col),
                                   max(2, math.ceil(distance/resolution*4)+1))).astype(int)
        if np.all(safe[cells[:, 0], cells[:, 1]]):
            best = (float(x), float(y), math.atan2(y-pose[1], x-pose[0]))
    return best
