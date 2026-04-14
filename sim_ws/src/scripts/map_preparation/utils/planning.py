import heapq
import math

import cv2
import numpy as np

from .constants import OCCUPIED


def brushfire_algo(occupancy_grid, use8CellWindow=True):
    """
    Fast brushfire distance map using OpenCV's distance transform.

    Returns a float grid where larger values are farther from obstacles.
    Values are shifted by +1 so obstacle-adjacent cells stay >= 2, matching
    the scale expected by the original safety-cost logic.
    """
    _ = use8CellWindow  # retained for API compatibility

    if occupancy_grid.ndim != 2:
        raise ValueError("brushfire_algo expects a 2D grid")

    traversable = (occupancy_grid != OCCUPIED).astype(np.uint8) * 255
    dist = cv2.distanceTransform(traversable, cv2.DIST_L2, 3)
    return dist + 1.0


def a_star_algo(
    occupancy_grid,
    brushfire_weights,
    safety_weight,
    start_pos,
    end_pos,
    safety_penalty=None,
    turn_weight=0,
):
    """
    Finds the shortest, safest, and smoother path from start_pos to end_pos
    using A* with brushfire-based safety and turn penalties.

    Returns
    -------
    list of (row, col) or None if no path exists
    """

    row_count, col_count = occupancy_grid.shape
    occ_mask = occupancy_grid == OCCUPIED

    sqrt2 = math.sqrt(2.0)

    for pos, name in [(start_pos, "Start"), (end_pos, "End")]:
        r, c = pos
        if not (0 <= r < row_count and 0 <= c < col_count):
            print(f"{name} position {pos} is outside the map.")
            return None
        if occ_mask[r, c]:
            print(f"{name} position {pos} is inside an obstacle.")
            return None

    sr, sc = start_pos
    er, ec = end_pos

    if (sr, sc) == (er, ec):
        return [(sr, sc)]

    if safety_penalty is None:
        d_safe = np.clip(brushfire_weights.astype(np.float64), 1e-6, None)
        safety_penalty = safety_weight / (d_safe * d_safe)

    closed = np.zeros((row_count, col_count), dtype=np.bool_)
    g = np.full((row_count, col_count), np.inf, dtype=np.float64)
    f = np.full((row_count, col_count), np.inf, dtype=np.float64)

    parent_r = np.full((row_count, col_count), -1, dtype=np.int32)
    parent_c = np.full((row_count, col_count), -1, dtype=np.int32)

    def h(r, c):
        dr = abs(r - er)
        dc = abs(c - ec)
        return (dr + dc) + (sqrt2 - 2.0) * min(dr, dc)

    g[sr, sc] = 0.0
    f[sr, sc] = h(sr, sc)

    parent_r[sr, sc] = sr
    parent_c[sr, sc] = sc

    open_list = []
    heapq.heappush(open_list, (f[sr, sc], sr, sc))

    directions = (
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (1, 1, sqrt2),
        (1, -1, sqrt2),
        (-1, 1, sqrt2),
        (-1, -1, sqrt2),
    )

    while open_list:
        _, r, c = heapq.heappop(open_list)

        if closed[r, c]:
            continue
        closed[r, c] = True

        if (r, c) == (er, ec):
            path = []
            cr, cc = r, c
            while True:
                path.append((int(cr), int(cc)))
                pr, pc = parent_r[cr, cc], parent_c[cr, cc]

                if pr == cr and pc == cc:
                    break
                if pr < 0 or pc < 0:
                    return None

                cr, cc = pr, pc

            return path[::-1]

        grc = g[r, c]

        for dr, dc, move_cost in directions:
            nr, nc = r + dr, c + dc

            if not (0 <= nr < row_count and 0 <= nc < col_count):
                continue
            if occ_mask[nr, nc] or closed[nr, nc]:
                continue

            turn_penalty = 0.0
            pr, pc = parent_r[r, c], parent_c[r, c]

            if not (pr == r and pc == c):
                prev_dir = (r - pr, c - pc)
                new_dir = (dr, dc)

                prev_norm = math.hypot(prev_dir[0], prev_dir[1])
                new_norm = math.hypot(new_dir[0], new_dir[1])

                if prev_norm > 0 and new_norm > 0:
                    dot = (
                        prev_dir[0] * new_dir[0] + prev_dir[1] * new_dir[1]
                    ) / (prev_norm * new_norm)

                    dot = max(-1.0, min(1.0, dot))

                    angle = math.acos(dot)

                    turn_penalty = turn_weight * (angle / math.pi)

            g_new = grc + move_cost + safety_penalty[nr, nc] + turn_penalty

            if g_new < g[nr, nc]:
                g[nr, nc] = g_new
                parent_r[nr, nc] = r
                parent_c[nr, nc] = c

                f_new = g_new + h(nr, nc)
                f[nr, nc] = f_new

                heapq.heappush(open_list, (f_new, nr, nc))

    return None


def plan_full_path(
    occupancy_grid,
    brushfire_grid,
    safety_weight,
    turn_weight,
    start_pos,
    waypoints,
):
    """
    Runs A* on each consecutive pair of positions and concatenates the
    resulting segments into one continuous closed-loop path.

    Segment chain:  start_pos -> cp[0] -> cp[1] -> ... -> cp[-1] -> start_pos

    Returns
    -------
    list of (row, col) or None if any segment has no solution
    """
    d_safe = np.clip(brushfire_grid.astype(np.float64), 1e-6, None)
    safety_penalty = safety_weight / (d_safe * d_safe)

    all_positions = [start_pos] + list(waypoints) + [start_pos]
    full_path = []

    n_segs = len(all_positions) - 1
    for i in range(n_segs):
        seg_start = all_positions[i]
        seg_end = all_positions[i + 1]

        label = (
            "closing segment (back to start)"
            if i == n_segs - 1
            else f"segment {i + 1}/{n_segs}"
        )
        print(f"  {label}:  {seg_start} -> {seg_end}")

        segment = a_star_algo(
            occupancy_grid,
            brushfire_grid,
            safety_weight,
            seg_start,
            seg_end,
            safety_penalty=safety_penalty,
            turn_weight=turn_weight,
        )

        if segment is None:
            print(f"  No path found for {label}. Try repositioning checkpoint {i + 1}.")
            return None

        print(f"  OK  ({len(segment)} cells)")

        if full_path:
            segment = segment[1:]

        full_path.extend(segment)

    return full_path
