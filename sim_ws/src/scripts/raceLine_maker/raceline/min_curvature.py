import numpy as np
import matplotlib.pyplot as plt
import quadprog

from .constants import FREE

def grid_path_to_world(smooth_raceline, origin, resolution: float, height: int) -> np.ndarray:
    """
    Convert a list of grid (row, col) positions to world (x, y) coordinates
    in metres.

    This is the first step in the pipeline and must be called before any
    geometry (normals, widths, reftrack) is computed.

    :param smooth_raceline: List of (row, col) integer grid positions.
    :param origin:          Map origin (ox, oy, ...) in metres, from YAML.
    :param resolution:      Metres per pixel.
    :param height:          Grid height in pixels (needed for row-flip).
    :return: (n, 2) float array of world [x, y] coordinates, with any
             duplicate closing point removed.
    """
    ox, oy = origin[:2]
    world_pts = np.array(
        [
            (col * resolution + ox, (height - 1 - row) * resolution + oy)
            for row, col in smooth_raceline
        ],
        dtype=float,
    )

    # Drop duplicate closing point if the smoothed path wraps back to start.
    if len(world_pts) > 1 and np.linalg.norm(world_pts[0] - world_pts[-1]) < 1e-9:
        world_pts = world_pts[:-1]

    return world_pts

def compute_normal_vectors(world_xy: np.ndarray) -> np.ndarray:
    """
    Compute unit normal vectors pointing left of the travel direction for each
    waypoint, using wrap-around central differences for the closed loop.

    :param world_xy: (n, 2) array of world [x, y] positions from
                     grid_path_to_world.
    :return:         (n, 2) array of unit normal vectors [nx, ny].
    """
    n = world_xy.shape[0]
    normals = np.zeros((n, 2))

    for i in range(n):
        p_prev = world_xy[(i - 1) % n]
        p_next = world_xy[(i + 1) % n]

        tangent = p_next - p_prev
        tang_norm = np.linalg.norm(tangent)

        if tang_norm < 1e-12:
            # Fall back to forward difference.
            tangent = world_xy[(i + 1) % n] - world_xy[i]
            tang_norm = np.linalg.norm(tangent)

        if tang_norm < 1e-12:
            normals[i] = normals[i - 1]
            continue

        tangent /= tang_norm
        # Rotate 90° counter-clockwise → left-pointing normal.
        normals[i] = np.array([-tangent[1], tangent[0]])

    return normals

def compute_track_widths(
    smooth_raceline,
    normvectors: np.ndarray,
    occupancy_grid: np.ndarray,
    resolution: float,
    max_width: float = 10.0,
) -> tuple:
    """
    Raycast left and right along each normal vector to find the drivable
    half-widths at every waypoint directly from the occupancy grid.

    smooth_raceline must already be deduplicated (same length as normvectors),
    which is guaranteed if grid_path_to_world was called first and its output
    length matches — trim the grid list to the same length if needed.

    :param smooth_raceline: List of (row, col) grid positions, length n.
    :param normvectors:     (n, 2) unit normal vectors from compute_normal_vectors.
    :param occupancy_grid:  2-D int8 grid (FREE=0, OCCUPIED=100, UNKNOWN=-1).
    :param resolution:      Metres per pixel.
    :param max_width:       Maximum raycast distance [m].
    :return: (w_left, w_right) – two (n,) float arrays of half-widths in metres.
    """
    n = normvectors.shape[0]
    assert len(smooth_raceline) == n, (
        f"smooth_raceline has {len(smooth_raceline)} points but normvectors has {n}. "
        "Trim smooth_raceline to the deduplicated length returned by grid_path_to_world."
    )

    w_left  = np.zeros(n)
    w_right = np.zeros(n)

    max_steps = int(max_width / resolution)
    grid_h, grid_w = occupancy_grid.shape

    for i, (row, col) in enumerate(smooth_raceline):
        nx, ny = normvectors[i]
        # World normal → grid offset:  +x = +col,  +y = -row (rows grow down).
        dn_col =  nx
        dn_row = -ny

        for sign, store in ((1, "left"), (-1, "right")):
            dist = 0.0
            for step in range(1, max_steps + 1):
                r = int(round(row + sign * dn_row * step))
                c = int(round(col + sign * dn_col * step))

                if not (0 <= r < grid_h and 0 <= c < grid_w):
                    break
                if occupancy_grid[r, c] != FREE:
                    break

                dist = step * resolution

            if store == "left":
                w_left[i] = dist
            else:
                w_right[i] = dist

    return w_left, w_right

def build_reftrack(
    world_xy: np.ndarray,
    w_left: np.ndarray,
    w_right: np.ndarray,
) -> np.ndarray:
    """
    Assemble the final reftrack array expected by opt_min_curv.

    All conversion and geometry is done by the earlier pipeline steps; this
    function is a simple column-stack.

    :param world_xy: (n, 2) from grid_path_to_world.
    :param w_left:   (n,)   from compute_track_widths.
    :param w_right:  (n,)   from compute_track_widths.
    :return: (n, 4) array with columns [x, y, w_left, w_right].
    """
    assert world_xy.shape[0] == len(w_left) == len(w_right), \
        "world_xy, w_left, and w_right must all have the same length."
    return np.column_stack([world_xy, w_left, w_right])

def generate_spline_matrix(no_points: int, closed: bool = True) -> np.ndarray:
    """
    Build the (4·n_splines) × (4·n_splines) linear system enforcing
    C2-continuity of the piecewise-cubic spline.

    Each segment i: s_i(t) = a_i + b_i·t + c_i·t² + d_i·t³,  t ∈ [0, 1].
    Unknowns packed as [a0, b0, c0, d0, a1, b1, c1, d1, …].

    Four equations per segment:
      (a) s_i(0) = p_i                     – value at segment start
      (b) s_i(1) = s_{i+1}(0)              – value continuity
      (c) s_i'(1) = s_{i+1}'(0)            – first-derivative continuity
      (d) s_i''(1) = s_{i+1}''(0)          – second-derivative continuity

    :param no_points: Number of waypoints (= number of segments for closed track).
    :param closed:    True for a closed (loop) track.
    :return: Square matrix A of shape (4·n_splines, 4·n_splines).
    """
    no_splines = no_points if closed else no_points - 1
    A = np.zeros((no_splines * 4, no_splines * 4))

    row = 0
    for i in range(no_splines):
        base      = i * 4
        next_base = ((i + 1) % no_splines) * 4

        # (a) Value at start: a_i = p_i
        A[row, base] = 1.0
        row += 1

        if i < no_splines - 1 or closed:
            # (b) Value continuity: a_i + b_i + c_i + d_i - a_{i+1} = 0
            A[row, base]      =  1.0
            A[row, base + 1]  =  1.0
            A[row, base + 2]  =  1.0
            A[row, base + 3]  =  1.0
            A[row, next_base] = -1.0
            row += 1

            # (c) First-derivative: b_i + 2c_i + 3d_i - b_{i+1} = 0
            A[row, base + 1]      =  1.0
            A[row, base + 2]      =  2.0
            A[row, base + 3]      =  3.0
            A[row, next_base + 1] = -1.0
            row += 1

            # (d) Second-derivative: 2c_i + 6d_i - 2c_{i+1} = 0
            A[row, base + 2]      =  2.0
            A[row, base + 3]      =  6.0
            A[row, next_base + 2] = -2.0
            row += 1

    # Open track: pin end-value of last segment.
    if not closed:
        base = (no_splines - 1) * 4
        A[row, base]     = 1.0
        A[row, base + 1] = 1.0
        A[row, base + 2] = 1.0
        A[row, base + 3] = 1.0

    return A


def opt_min_curv(
    reftrack: np.ndarray,
    normvectors: np.ndarray,
    A: np.ndarray,
    kappa_bound: float,
    w_veh: float,
    plot_debug: bool = False,
    closed: bool = True,
    fix_s: bool = False,
    fix_e: bool = False,
) -> tuple:
    """
    Solve a QP that finds per-waypoint lateral offsets α along the normal
    vectors that minimise integrated squared curvature subject to track-width
    constraints.

    :param reftrack:    (n, 4) – [x, y, w_left, w_right] from build_reftrack.
    :param normvectors: (n, 2) – unit normals from compute_normal_vectors.
    :param A:           Spline system matrix from generate_spline_matrix.
    :param kappa_bound: Maximum curvature [1/m] (reserved for future hard constraint).
    :param w_veh:       Vehicle width [m] – shrinks the drivable corridor.
    :param plot_debug:  Show path and curvature plots if True.
    :param closed:      True for a closed (loop) track.
    :param fix_s:       Pin start point to the reference line (open tracks).
    :param fix_e:       Pin end point to the reference line (open tracks).
    :return: (alpha_mincurv, curv_max)
               alpha_mincurv – (n,) lateral offsets in metres.
               curv_max      – maximum absolute curvature of the optimised path.
    """
    no_points  = reftrack.shape[0]
    no_splines = no_points if closed else no_points - 1

    if normvectors.shape[0] != no_points:
        raise RuntimeError("reftrack and normvectors must have the same number of rows.")
    expected = no_splines * 4
    if A.shape != (expected, expected):
        raise RuntimeError(f"Matrix A must be ({expected}, {expected}), got {A.shape}.")

    # ------------------------------------------------------------------
    # Extraction matrices
    # ------------------------------------------------------------------
    A_ex_b = np.zeros((no_points, no_splines * 4))
    A_ex_c = np.zeros((no_points, no_splines * 4))

    for i in range(no_splines):
        A_ex_b[i, i * 4 + 1] = 1.0
        A_ex_c[i, i * 4 + 2] = 2.0

    if not closed:
        last = (no_splines - 1) * 4
        A_ex_b[-1, last:last + 4] = [0.0, 1.0, 2.0, 3.0]
        A_ex_c[-1, last:last + 4] = [0.0, 0.0, 2.0, 6.0]

    # ------------------------------------------------------------------
    # Invert spline system; cache products used multiple times below
    # ------------------------------------------------------------------
    try:
        A_inv = np.linalg.inv(A)
    except np.linalg.LinAlgError:
        A_inv = np.linalg.inv(A + np.eye(A.shape[0]) * 1e-8)

    A_ex_b_inv = A_ex_b @ A_inv   # (n, 4·n_splines) – reused for ref + opt
    A_ex_c_inv = A_ex_c @ A_inv

    # ------------------------------------------------------------------
    # RHS coordinate vectors
    # ------------------------------------------------------------------
    q_x = np.zeros(no_splines * 4)
    q_y = np.zeros(no_splines * 4)
    for i in range(no_splines):
        q_x[i * 4] = reftrack[i, 0]
        q_y[i * 4] = reftrack[i, 1]

    # ------------------------------------------------------------------
    # Normal-vector perturbation matrices
    # ------------------------------------------------------------------
    M_x = np.zeros((no_splines * 4, no_points))
    M_y = np.zeros((no_splines * 4, no_points))
    for i in range(no_splines):
        M_x[i * 4, i] = normvectors[i, 0]
        M_y[i * 4, i] = normvectors[i, 1]

    # ------------------------------------------------------------------
    # Reference path derivatives
    # ------------------------------------------------------------------
    xd_ref  = A_ex_b_inv @ q_x
    yd_ref  = A_ex_b_inv @ q_y
    xdd_ref = A_ex_c_inv @ q_x
    ydd_ref = A_ex_c_inv @ q_y

    # ------------------------------------------------------------------
    # Sensitivity of second derivatives to alpha
    # ------------------------------------------------------------------
    T_cx = A_ex_c_inv @ M_x   # (n, n)
    T_cy = A_ex_c_inv @ M_y

    # ------------------------------------------------------------------
    # Curvature denominator weights
    # ------------------------------------------------------------------
    denom      = (xd_ref ** 2 + yd_ref ** 2) ** 1.5
    safe_denom = np.where(denom > 1e-12, denom, 1e-12)
    w          = 1.0 / safe_denom ** 2

    # ------------------------------------------------------------------
    # QP cost matrices
    # ------------------------------------------------------------------
    W_yy = np.diag(w * yd_ref ** 2)
    W_xx = np.diag(w * xd_ref ** 2)
    W_xy = np.diag(w * xd_ref * yd_ref)

    H = (T_cx.T @ W_yy @ T_cx
         + T_cy.T @ W_xx @ T_cy
         - T_cx.T @ W_xy @ T_cy
         - T_cy.T @ W_xy @ T_cx)
    H = 0.5 * (H + H.T) + np.eye(no_points) * 1e-8

    curv_num_ref = xd_ref * ydd_ref - yd_ref * xdd_ref
    f = 2.0 * (
          T_cx.T @ np.diag(w * yd_ref)  @ curv_num_ref
        - T_cy.T @ np.diag(w * xd_ref) @ curv_num_ref
    )

    # ------------------------------------------------------------------
    # Inequality constraints:  lb ≤ alpha ≤ ub
    # ------------------------------------------------------------------
    half_veh = w_veh / 2.0
    lb = -(reftrack[:, 2] - half_veh)
    ub =   reftrack[:, 3] - half_veh

    if fix_s:
        lb[0] = ub[0] = 0.0
    if fix_e:
        lb[-1] = ub[-1] = 0.0

    C = np.vstack([np.eye(no_points), -np.eye(no_points)]).T
    b = np.concatenate([lb, -ub])

    # ------------------------------------------------------------------
    # Solve QP
    # ------------------------------------------------------------------
    try:
        alpha_mincurv = quadprog.solve_qp(H, -f, C, b, 0)[0]
    except ValueError as exc:
        raise RuntimeError(f"QP solver failed: {exc}") from exc

    # ------------------------------------------------------------------
    # Curvature of the optimised path (reuses A_ex_b_inv / A_ex_c_inv)
    # ------------------------------------------------------------------
    q_x_opt = q_x + M_x @ alpha_mincurv
    q_y_opt = q_y + M_y @ alpha_mincurv

    xd_opt  = A_ex_b_inv @ q_x_opt
    yd_opt  = A_ex_b_inv @ q_y_opt
    xdd_opt = A_ex_c_inv @ q_x_opt
    ydd_opt = A_ex_c_inv @ q_y_opt

    denom_opt = (xd_opt ** 2 + yd_opt ** 2) ** 1.5
    kappa_opt = np.where(
        denom_opt > 1e-12,
        (xd_opt * ydd_opt - yd_opt * xdd_opt) / denom_opt,
        0.0,
    )
    curv_max = float(np.amax(np.abs(kappa_opt)))

    # ------------------------------------------------------------------
    # Debug plot
    # ------------------------------------------------------------------
    if plot_debug:
        opt_x = reftrack[:, 0] + alpha_mincurv * normvectors[:, 0]
        opt_y = reftrack[:, 1] + alpha_mincurv * normvectors[:, 1]
        kappa_ref = np.where(denom > 1e-12, curv_num_ref / denom, 0.0)

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        axes[0].plot(reftrack[:, 0], reftrack[:, 1], ":", label="Reference track")
        axes[0].plot(opt_x, opt_y, label="Optimised path")
        axes[0].set_aspect("equal")
        axes[0].legend()
        axes[0].set_title("Path comparison")

        axes[1].plot(kappa_ref, label="Reference curvature")
        axes[1].plot(kappa_opt, label="Optimised curvature")
        axes[1].legend()
        axes[1].set_title("Curvature comparison")

        plt.tight_layout()
        plt.show()

    return alpha_mincurv, curv_max