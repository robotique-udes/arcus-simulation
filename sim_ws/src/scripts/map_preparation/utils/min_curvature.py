import numpy as np
import matplotlib.pyplot as plt
from utils.constants import FREE
import cvxpy as cp
from scipy.spatial import KDTree
import networkx as nx


# ---------------------------------------------------------
# GRAPH-BASED CENTERLINE ORDERING
# ---------------------------------------------------------

def _build_skeleton_graph(points: np.ndarray) -> nx.Graph:
    """
    Connect skeleton pixels into a graph by linking each point to its
    8-connected neighbors that also exist in the point set.
    """
    G = nx.Graph()
    G.add_nodes_from(range(len(points)))

    index_map = {(int(r), int(c)): i for i, (r, c) in enumerate(points)}

    for i, (r, c) in enumerate(points):
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nb = (r + dr, c + dc)
                if nb in index_map:
                    j = index_map[nb]
                    if i < j:
                        dist = np.hypot(dr, dc)
                        G.add_edge(i, j, weight=dist)

    return G


def _find_junction_nodes(G: nx.Graph) -> list:
    """Nodes with degree >= 3 are branch points / junctions."""
    return [n for n, d in G.degree() if d >= 3]


def _snap_anchor_indices(points: np.ndarray, anchor_points) -> list:
    """Map user-provided anchor points to the nearest skeleton nodes."""
    if anchor_points is None:
        return []

    pts = np.array(points, dtype=float)
    if len(pts) == 0:
        return []

    tree = KDTree(pts)
    indices = []

    for anchor in anchor_points:
        if anchor is None:
            continue

        if np.isscalar(anchor):
            idx = int(anchor)
        else:
            _, idx = tree.query(np.array(anchor, dtype=float))
            idx = int(idx)

        if not indices or idx != indices[-1]:
            indices.append(idx)

    return indices


def _path_from_anchor_indices(G: nx.Graph, anchor_indices: list) -> list:
    """Build an ordered closed path by stitching shortest paths between anchors."""
    if len(anchor_indices) < 2:
        return []

    ordered = [anchor_indices[0]]

    for start, end in zip(anchor_indices, anchor_indices[1:]):
        segment = nx.shortest_path(G, start, end, weight="weight")
        if ordered[-1] == segment[0]:
            ordered.extend(segment[1:])
        else:
            ordered.extend(segment)

    if ordered[-1] != ordered[0]:
        closing_segment = nx.shortest_path(G, ordered[-1], ordered[0], weight="weight")
        ordered.extend(closing_segment[1:])

    return ordered


def _remove_spurs(G: nx.Graph, min_spur_length: int = 20) -> nx.Graph:
    """
    Iteratively prune dead-end branches shorter than min_spur_length edges.
    This is the graph equivalent of skeleton pruning.
    """
    G = G.copy()
    changed = True
    while changed:
        changed = False
        endpoints = [n for n, d in G.degree() if d == 1]
        for ep in endpoints:
            branch = [ep]
            prev = None
            current = ep
            for _ in range(min_spur_length):
                nbs = [n for n in G.neighbors(current) if n != prev]
                if len(nbs) != 1:
                    break
                prev = current
                current = nbs[0]
                branch.append(current)

            if len(branch) < min_spur_length:
                for node in branch[:-1]:
                    if G.has_node(node):
                        G.remove_node(node)
                changed = True

    return G


def order_centerline(points, min_spur_length: int = 20, anchor_points=None) -> list:
    """
    Order centerline points by tracing the skeleton as a graph.

    Handles tracks with multiple loops (e.g. a main loop + hairpin) by:
      1. Building a pixel-connectivity graph from skeleton points
      2. Pruning short spur branches
    3. Tracing via DFS from a junction node to visit all branches
    4. Optionally following a user-selected route through the skeleton

    Parameters
    ----------
    points         : array-like (N, 2)  row-col skeleton pixels
    min_spur_length: int  prune branches shorter than this many pixels
    anchor_points   : optional ordered list of (row, col) points that define
                      the general route to follow through the skeleton

    Returns
    -------
    list of (row, col) tuples in traversal order
    """
    pts = np.array(points, dtype=int)
    if len(pts) == 0:
        return []

    G = _build_skeleton_graph(pts)

    # Keep only the largest connected component
    largest_cc = max(nx.connected_components(G), key=len)
    G = G.subgraph(largest_cc).copy()

    # Prune spurs
    G = _remove_spurs(G, min_spur_length=min_spur_length)

    if len(G.nodes) == 0:
        return list(map(tuple, pts))

    # Keep largest component again after pruning
    largest_cc = max(nx.connected_components(G), key=len)
    G = G.subgraph(largest_cc).copy()

    junctions = _find_junction_nodes(G)

    anchor_indices = _snap_anchor_indices(pts, anchor_points)
    if len(anchor_indices) >= 2:
        try:
            path = _path_from_anchor_indices(G, anchor_indices)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            path = []

        if path:
            return [tuple(pts[i]) for i in path]

    if not junctions:
        # Simple loop or chain — DFS from a degree-1 endpoint or any node
        endpoints = [n for n, d in G.degree() if d == 1]
        start = endpoints[0] if endpoints else list(G.nodes)[0]
        path = list(nx.dfs_preorder_nodes(G, source=start))
    else:
        # Track has branches (hairpin, chicane, etc.)
        # Try a few junction starts and keep the one that visits the most nodes
        best_path = []
        for start in junctions[:3]:
            path = list(nx.dfs_preorder_nodes(G, source=start))
            if len(path) > len(best_path):
                best_path = path
        path = best_path

    return [tuple(pts[i]) for i in path]


# ---------------------------------------------------------
# PATH PROCESSING
# ---------------------------------------------------------

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


# ---------------------------------------------------------
# TRACK WIDTH RAYCASTING
# ---------------------------------------------------------

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


# ---------------------------------------------------------
# DEBUG / VISUALIZATION
# ---------------------------------------------------------

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

    plot_path(center_pts, "red",    "raw centerline", lw=0.5, s=2)
    plot_path(ordered,    "orange", "ordered",        lw=1)
    plot_path(resampled,  "yellow", "resampled",      lw=1)
    plot_path(smooth,     "cyan",   "smooth",         lw=2.5, s=0)

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

        L = w_left[i] / resolution
        R = w_right[i] / resolution

        rL = r - dr * L
        cL = c - dc * L

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

    plt.plot(pts[:, 1], pts[:, 0], 'cyan', linewidth=2, label="centerline")

    idx = np.arange(0, len(pts), stride)

    plt.quiver(
        pts[idx, 1],
        pts[idx, 0],
        norms[idx, 1] * scale,
        norms[idx, 0] * scale,
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

    wL = w_left / resolution
    wR = w_right / resolution

    plt.figure(figsize=(10, 10))
    plt.imshow(occupancy_grid, cmap="gray", origin="upper")

    plt.plot(pts[:, 1], pts[:, 0], "b-", linewidth=1, label="centerline")

    for i in range(0, len(pts), stride):
        r, c = pts[i]
        dr, dc = normals[i]

        left_pt  = (r - dr * wL[i], c - dc * wL[i])
        right_pt = (r + dr * wR[i], c + dc * wR[i])

        plt.plot(
            [left_pt[1], right_pt[1]],
            [left_pt[0], right_pt[0]],
            "yellow",
            linewidth=1.5,
            alpha=0.7
        )

        plt.scatter(left_pt[1],  left_pt[0],  c="green", s=10)
        plt.scatter(right_pt[1], right_pt[0], c="red",   s=10)

    plt.title("Solver Input Debug (Widths + Normals)")
    plt.legend()
    plt.axis("equal")
    plt.show()


# ---------------------------------------------------------
# MIN-CURVATURE SOLVER
# ---------------------------------------------------------

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
    solver_attempts = [
        (cp.ECOS, dict(verbose=False, max_iters=2000, abstol=1e-7, reltol=1e-7)),
        (cp.OSQP, dict(verbose=False, max_iter=10000, eps_abs=1e-6, eps_rel=1e-6)),
        (cp.SCS,  dict(verbose=False, max_iters=2500,  eps=1e-5)),
    ]

    solved = False
    for solver, solver_kwargs in solver_attempts:
        try:
            prob.solve(solver=solver, **solver_kwargs)
        except cp.error.SolverError:
            continue

        if alpha.value is not None:
            solved = True
            break

    if not solved:
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

    plt.plot(cl[:, 1], cl[:, 0], "b-", linewidth=1, label="Centerline")
    plt.plot(rl[:, 1], rl[:, 0], "r-", linewidth=2, label="Raceline")

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


def solve_min_curvature_from_raceline(
    raceline,
    occupancy_grid,
    resolution,
    vehicle_width=0.0,
    smoothness_weight=1e-4,
    debug=False,
):
    """
    Solve min-curvature using a provided raceline as the reference path.

    Returns:
        raceline_px, alpha, normals, w_left, w_right
    """
    if raceline is None or len(raceline) < 3:
        return np.array(raceline, dtype=float), np.zeros(0), None, None, None

    normals = compute_normals(raceline)
    w_left, w_right = compute_track_widths(
        raceline,
        normals,
        occupancy_grid,
        resolution,
        vehicle_width=vehicle_width,
    )

    if debug:
        debug_plot_normals(occupancy_grid, raceline, normals)
        plot_widths_simple(raceline, normals, w_left, w_right, occupancy_grid, resolution)
        debug_plot_solver_inputs(
            occupancy_grid,
            raceline,
            normals,
            w_left,
            w_right,
            resolution,
        )

    min_curv_raceline, alpha = solve_min_curvature_raceline(
        raceline,
        normals,
        w_left,
        w_right,
        resolution,
        smoothness_weight=smoothness_weight,
    )

    return min_curv_raceline, alpha, normals, w_left, w_right