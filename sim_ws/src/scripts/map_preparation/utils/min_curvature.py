import numpy as np
import matplotlib.pyplot as plt
from utils.constants import FREE
import cvxpy as cp
from scipy.spatial import KDTree

def order_centerline(points):
    pts = np.array(points)
    visited = set()

    # Build KDTree for nearest neighbors
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
    vehicle_width=0.0
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

    margin = vehicle_width

    w_left_safe  = np.maximum(w_left  - margin, 0.0)
    w_right_safe = np.maximum(w_right - margin, 0.0)

    return w_left_safe, w_right_safe

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

def solve_min_curvature_raceline(
    centerline,
    normals,
    w_left,
    w_right,
    resolution=1.0,
    smoothness_weight=1e-4,
):
    p = np.array(centerline, dtype=float)   # (N, 2) in pixels
    n = np.array(normals, dtype=float)      # (N, 2) unit vectors in pixel frame

    # Re-normalise defensively
    norms = np.linalg.norm(n, axis=1, keepdims=True)
    norms = np.where(norms < 1e-9, 1.0, norms)
    n = n / norms

    N = len(p)

    # Convert width constraints from metres → pixels so units match
    w_left_px  = w_left  / resolution
    w_right_px = w_right / resolution

    alpha = cp.Variable(N)
    alpha_expanded = cp.reshape(alpha, (N, 1))

    # raceline in pixel space: (N, 2)
    r = p + cp.multiply(alpha_expanded, n)

    # Wrap-around second differences (closed loop)
    r_prev = cp.vstack([r[-1:, :], r[:-1, :]])
    r_next = cp.vstack([r[1:,  :], r[:1,  :]])
    curvature = r_next - 2 * r + r_prev

    cost = cp.sum_squares(curvature)
    cost += smoothness_weight * cp.sum_squares(alpha[1:] - alpha[:-1])

    constraints = [
        alpha >= -w_left_px,
        alpha <=  w_right_px,
    ]

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, verbose=False, max_iter=10000, eps_abs=1e-6, eps_rel=1e-6)

    if alpha.value is None:
        print("[WARNING] Solver did not converge — returning centreline as fallback.")
        return p, np.zeros(N)

    raceline_px = p + alpha.value[:, None] * n

    print(f"alpha range: {alpha.value.min():.2f}…{alpha.value.max():.2f} px  "
          f"({alpha.value.min()*resolution:.3f}…{alpha.value.max()*resolution:.3f} m)")

    return raceline_px, alpha.value

def plot_raceline_result(
    occupancy_grid,
    centerline,
    raceline,
    normals=None,
    w_left=None,
    w_right=None,
    resolution=None,
    title="Minimum Curvature Raceline",
):
    """
    Visual comparison plot:
    - centerline (blue)
    - optimized raceline (red)
    - optional width rays
    """

    plt.figure(figsize=(8, 8))
    plt.imshow(occupancy_grid, cmap="gray", origin="upper")

    cl = np.array(centerline)
    rl = np.array(raceline)

    # centerline
    plt.plot(cl[:, 1], cl[:, 0], "b-", linewidth=1, label="Centerline")

    # raceline
    plt.plot(rl[:, 1], rl[:, 0], "r-", linewidth=2, label="Raceline")

    # optional: width visualization (light debugging)
    if normals is not None and w_left is not None and w_right is not None:
        n = np.array(normals)

        for i in range(0, len(cl), max(1, len(cl)//80)):
            r, c = cl[i]
            dr, dc = n[i]

            L = w_left[i] / resolution
            R = w_right[i] / resolution

            plt.plot([c, c - dc * L], [r, r - dr * L], "g-", alpha=0.2)
            plt.plot([c, c + dc * R], [r, r + dr * R], "r-", alpha=0.2)

    plt.legend()
    plt.title(title)
    plt.axis("equal")
    plt.show()

def debug_plot_solver_inputs(
    occupancy_grid,
    centerline,
    normals,
    w_left,
    w_right,
    resolution,
    stride=5,
):
    """
    Debug plot to verify:
    - normals are correct
    - widths are correct
    - units are consistent

    Must be called RIGHT before solver.
    """

    pts = np.array(centerline)
    normals = np.array(normals)

    # Convert widths to pixels if needed
    wL = w_left / resolution
    wR = w_right / resolution

    plt.figure(figsize=(10, 10))
    plt.imshow(occupancy_grid, cmap="gray", origin="upper")

    # centerline
    plt.plot(pts[:, 1], pts[:, 0], "b-", linewidth=1, label="centerline")

    for i in range(0, len(pts), stride):
        r, c = pts[i]
        dr, dc = normals[i]

        # endpoints
        left_pt  = (r - dr * wL[i], c - dc * wL[i])
        right_pt = (r + dr * wR[i], c + dc * wR[i])

        # draw full width segment
        plt.plot(
            [left_pt[1], right_pt[1]],
            [left_pt[0], right_pt[0]],
            "yellow",
            linewidth=1.5,
            alpha=0.7
        )

        # optional: mark endpoints
        plt.scatter(left_pt[1], left_pt[0], c="green", s=10)
        plt.scatter(right_pt[1], right_pt[0], c="red", s=10)

    plt.title("Solver Input Debug (Widths + Normals)")
    plt.legend()
    plt.axis("equal")
    plt.show()