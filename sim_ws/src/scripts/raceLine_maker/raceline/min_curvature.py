import numpy as np
import matplotlib.pyplot as plt
from raceline.grid_utils import grid_to_world, world_to_grid
from raceline.constants import FREE

def order_centerline(points):
    pts = np.array(points)
    visited = set()

    # Build KDTree for nearest neighbors
    from scipy.spatial import KDTree
    tree = KDTree(pts)

    ordered = []
    current_idx = 0
    ordered.append(tuple(pts[current_idx]))
    visited.add(current_idx)

    for _ in range(len(pts) - 1):
        dists, idxs = tree.query(pts[current_idx], k=8)

        for i in idxs:
            if i not in visited:
                visited.add(i)
                ordered.append(tuple(pts[i]))
                current_idx = i
                break

    return ordered

def resample_path(path, num_points=500):
    pts = np.array(path, dtype=float)

    # close loop
    pts = np.vstack([pts, pts[0]])

    # arc length
    d = np.sqrt(np.sum(np.diff(pts, axis=0)**2, axis=1))
    s = np.concatenate([[0], np.cumsum(d)])

    s_uniform = np.linspace(0, s[-1], num_points)

    new_pts = np.zeros((num_points, 2))
    for i in range(2):
        new_pts[:, i] = np.interp(s_uniform, s, pts[:, i])

    return [tuple(p) for p in new_pts]

def compute_normals(path):
    pts = np.array(path, dtype=float)
    n = len(pts)

    normals = np.zeros((n, 2))

    for i in range(n):
        p_prev = pts[i - 1]
        p_next = pts[(i + 1) % n]

        tangent = p_next - p_prev
        norm = np.linalg.norm(tangent)

        if norm < 1e-9:
            continue

        tangent /= norm

        normals[i] = np.array([-tangent[1], tangent[0]])

    return normals

def compute_track_widths(
    smooth_raceline,
    normals,
    occupancy_grid,
    resolution,
    max_width=10.0,
):
    """
    Computes left/right drivable widths by raycasting along grid-space normals.

    All inputs MUST be in grid coordinates:
        - smooth_raceline: (row, col)
        - normals: (dr, dc) in grid frame

    Returns:
        w_left, w_right in metres
    """

    n = len(smooth_raceline)

    w_left = np.zeros(n)
    w_right = np.zeros(n)

    h, w = occupancy_grid.shape
    max_steps = int(max_width / resolution)

    for i, (r, c) in enumerate(smooth_raceline):
        dr, dc = normals[i]

        # normalize safety (avoid drift issues)
        norm = np.hypot(dr, dc)
        if norm < 1e-9:
            continue
        dr /= norm
        dc /= norm

        # -------------------------
        # RIGHT side (+ normal)
        # -------------------------
        dist = 0.0
        for s in range(1, max_steps):
            rr = int(round(r + dr * s))
            cc = int(round(c + dc * s))

            if not (0 <= rr < h and 0 <= cc < w):
                break
            if occupancy_grid[rr, cc] != FREE:
                break

            dist = s * resolution

        w_right[i] = dist

        # -------------------------
        # LEFT side (- normal)
        # -------------------------
        dist = 0.0
        for s in range(1, max_steps):
            rr = int(round(r - dr * s))
            cc = int(round(c - dc * s))

            if not (0 <= rr < h and 0 <= cc < w):
                break
            if occupancy_grid[rr, cc] != FREE:
                break

            dist = s * resolution

        w_left[i] = dist

    return w_left, w_right

def debug_show_centerline_pipeline(
    occupancy_grid,
    center_pts,
    ordered,
    resampled,
    smooth
):
    plt.figure(figsize=(10, 10))

    plt.imshow(occupancy_grid, cmap="gray", vmin=0, vmax=100, origin="upper")

    def plot_path(path, color, label, lw=1.5, s=4):
        pts = np.array(path)
        if len(pts) == 0:
            return
        plt.plot(pts[:, 1], pts[:, 0], color=color, lw=lw, label=label)
        plt.scatter(pts[:, 1], pts[:, 0], c=color, s=s)

    # Raw unordered centerline
    plot_path(center_pts, "red", "raw centerline", lw=0.5, s=2)

    # Ordered
    plot_path(ordered, "orange", "ordered", lw=1)

    # Resampled
    plot_path(resampled, "yellow", "resampled", lw=1)

    # Final smooth
    plot_path(smooth, "cyan", "smooth", lw=2.5, s=0)

    plt.legend()
    plt.title("Centerline → Smooth Pipeline")
    plt.gca().invert_yaxis()
    plt.show()

def plot_widths_simple(
    smooth_raceline,
    normals,
    w_left,
    w_right,
    occupancy_grid,
    resolution
):
    pts = np.array(smooth_raceline)
    normals = np.array(normals)

    plt.figure(figsize=(8, 8))
    plt.imshow(occupancy_grid, cmap="gray", origin="upper")

    plt.plot(pts[:,1], pts[:,0], "b-", linewidth=1)

    for i in range(len(pts)):
        r, c = pts[i]
        dr, dc = normals[i]

        # convert METRES → PIXELS
        L = w_left[i] / resolution
        R = w_right[i] / resolution

        # LEFT
        rL = r - dr * L
        cL = c - dc * L

        # RIGHT
        rR = r + dr * R
        cR = c + dc * R

        plt.plot([c, cL], [r, rL], "g-")
        plt.plot([c, cR], [r, rR], "r-")

    plt.axis("equal")
    plt.title("Track widths (fixed frame consistency)")
    plt.show()

def debug_plot_normals(occupancy_grid, path, normals, stride=1, scale=10):
    """
    Visualize normals along a path.

    Parameters
    ----------
    occupancy_grid : np.ndarray
    path           : list[(row, col)]
    normals        : np.ndarray (Nx2)
    stride         : int   (plot every Nth normal)
    scale          : float (length of arrows)
    """

    pts = np.array(path, dtype=float)
    norms = np.array(normals, dtype=float)

    plt.figure(figsize=(10, 10))
    plt.imshow(occupancy_grid, cmap="gray", vmin=0, vmax=100, origin="upper")

    # plot centerline
    plt.plot(pts[:, 1], pts[:, 0], 'cyan', linewidth=2, label="centerline")

    # subsample for readability
    idx = np.arange(0, len(pts), stride)

    # arrows (note: col = x, row = y)
    plt.quiver(
        pts[idx, 1],            # x (col)
        pts[idx, 0],            # y (row)
        norms[idx, 1] * scale,  # dx
        norms[idx, 0] * scale,  # dy
        color='red',
        angles='xy',
        scale_units='xy',
        scale=1,
        width=0.003
    )

    plt.title("Normals Debug")
    plt.legend()
    plt.gca().invert_yaxis()
    plt.show()