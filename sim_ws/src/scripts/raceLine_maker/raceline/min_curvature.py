import numpy as np
import math
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
    :return: (w_left, w_right) - two (n,) float arrays of half-widths in metres.
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

def generate_spline_matrix(world_xy: np.ndarray) -> np.ndarray:
    """
    Build the (4·n_splines) x (4·n_splines) linear system enforcing
    C2-continuity of the piecewise-cubic spline, with per-segment scaling
    so that derivative continuity is enforced in metres (not parameter units).
 
    Each segment i: s_i(t) = a_i + b_i·t + c_i·t² + d_i·t³,  t ∈ [0, 1].
    Unknowns packed as [a0, b0, c0, d0, a1, b1, c1, d1, …].
 
    The scaling factor for segment i is 1/el_i where el_i is the Euclidean
    distance between waypoint i and i+1. This converts the parameter-space
    derivatives into physical (metre-space) derivatives, matching the
    convention used in the reference calc_splines implementation.
 
    Four equations per interior junction:
      (a) s_i(0)   = p_i                   - value at segment start
      (b) s_i(1)   = p_{i+1}               - value at segment end
      (c) s_i'(1)  = s_{i+1}'(0)           - first-derivative continuity
      (d) s_i''(1) = s_{i+1}''(0)          - second-derivative continuity
 
    The last two rows close the loop (or pin the open-track end heading):
      (-2) first-derivative continuity at the wrap-around junction
      (-1) second-derivative continuity at the wrap-around junction
 
    :param world_xy: (n, 2) array of waypoint world coordinates [x, y].
    :param closed:   True for a closed (loop) track.
    :return: Square matrix A of shape (4·n_splines, 4·n_splines).
    """
    no_points  = world_xy.shape[0]
    no_splines = no_points
 
    # Compute per-segment Euclidean lengths and scaling factors.
    # For closed tracks the last segment connects point[-1] back to point[0].
    el_lengths = np.array([
        np.linalg.norm(world_xy[(i + 1) % no_points] - world_xy[i])
        for i in range(no_splines)
    ])
    # Guard against zero-length segments (duplicate waypoints).
    el_lengths = np.where(el_lengths > 1e-9, el_lengths, 1e-9)
    scaling = 1.0 / el_lengths   # shape (no_splines,)
 
    # Template for a standard interior junction (8 columns: cur + next segment)
    template_A = np.array([
        [1, 0, 0, 0,  0,  0,  0, 0],   # (a) s_i(0)   = p_i
        [1, 1, 1, 1,  0,  0,  0, 0],   # (b) s_i(1)   = p_{i+1}
        [0, 1, 2, 3,  0, -1,  0, 0],   # (c) s_i'(1)  = s_{i+1}'(0)  (unscaled)
        [0, 0, 2, 6,  0,  0, -2, 0],   # (d) s_i''(1) = s_{i+1}''(0) (unscaled)
    ], dtype=float)
 
    A = np.zeros((no_splines * 4, no_splines * 4))
 
    for i in range(no_splines):
        j = i * 4
 
        if i < no_splines - 1:
            # Interior segment: fill value + continuity rows.
            A[j:j + 4, j:j + 8] = template_A
 
            # Scale derivative rows so continuity is in metre-space.
            # Row (c): multiply the outgoing  b_{i+1} column by scaling[i]
            #          so that  b_i/el_i = b_{i+1}/el_{i+1} at the junction.
            A[j + 2, j + 5] *= scaling[i]
 
            # Row (d): second derivative scales by scaling[i]^2
            A[j + 3, j + 6] *= scaling[i] ** 2
 
        else:
            # Last segment: only pin the two endpoint values here.
            # The derivative-continuity wrap-around rows are filled below.
            A[j:j + 2, j:j + 4] = [
                [1, 0, 0, 0],   # (a) s_{n-1}(0) = p_{n-1}
                [1, 1, 1, 1],   # (b) s_{n-1}(1) = p_0  (closing the loop)
            ]
 
    # Close the loop: derivative continuity between the last and first segment.
    # Row -2: first-derivative wrap  →  b_{n-1} * scaling[n-1] - b_0 - 2c_0 - 3d_0 = 0
    A[-2, 1]   =  scaling[-1]   # b_{n-1} scaled
    A[-2, -3:] = [-1, -2, -3]   # -b_0, -2c_0, -3d_0  (first three derivative terms of segment 0 at t=0 → b_0)
 
    # Row -1: second-derivative wrap  →  2c_{n-1} * scaling[n-1]^2 - 2c_0 - 6d_0 = 0
    A[-1, 2]   =  2.0 * scaling[-1] ** 2
    A[-1, -2:] = [-2, -6]       # -2c_0, -6d_0
 
    return A


def opt_min_curv(
    reftrack: np.ndarray,
    normvectors: np.ndarray,
    A: np.ndarray,
    kappa_bound: float,
    w_veh: float,
    plot_debug: bool = False,
) -> tuple:
    """
    Minimise the summed curvature of a path by moving each point along its
    normal vector within the track width.
 
    Follows the formulation from:
      Heilmeier et al., "Minimum Curvature Trajectory Planning and Control
      for an Autonomous Racecar", Vehicle System Dynamics, 2019.
 
    :param reftrack:    (n, 4) - [x, y, w_tr_right, w_tr_left] from build_reftrack.
                        NOTE: column 2 = right width, column 3 = left width.
    :param normvectors: (n, 2) - unit normal vectors from compute_normal_vectors.
    :param A:           Spline system matrix from generate_spline_matrix.
    :param kappa_bound: Maximum curvature constraint [1/m].
    :param w_veh:       Vehicle width [m] - shrinks the drivable corridor.
    :param plot_debug:  Plot curvature comparison if True.
    :param psi_s:       Start heading for open tracks.
    :param psi_e:       End heading for open tracks.
    :param fix_s:       Pin start point to reference line (open tracks).
    :param fix_e:       Pin end point to reference line (open tracks).
    :return: (alpha_mincurv, curv_error_max)
               alpha_mincurv  - (n,) lateral offsets in metres.
               curv_error_max - max curvature error between original and
                                solution linearisations.
    """
    no_points  = reftrack.shape[0]
    no_splines = no_points
 
    if normvectors.shape[0] != no_points:
        raise RuntimeError("reftrack and normvectors must have the same number of rows.")
    if (no_points * 4 != A.shape[0]) or \
       A.shape[0] != A.shape[1]:
        raise RuntimeError("Spline matrix A has wrong dimensions.")
 
    # ------------------------------------------------------------------
    # Extraction matrices
    # A_ex_b: extracts b_i coefficients  → first derivative at t=0
    # A_ex_c: extracts c_i coefficients  → used to get second derivative
    #         (second deriv at t=0 = 2*c_i, handled by the 2* in the matrix)
    # ------------------------------------------------------------------
    A_ex_b = np.zeros((no_points, no_splines * 4), dtype=int)
    A_ex_c = np.zeros((no_points, no_splines * 4), dtype=int)
 
    for i in range(no_splines):
        A_ex_b[i, i * 4 + 1] = 1   # b_i
        A_ex_c[i, i * 4 + 2] = 2   # 2*c_i → second derivative at t=0
 
    A_inv = np.linalg.inv(A)
    T_c   = np.matmul(A_ex_c, A_inv)   # maps coordinate RHS to second derivatives
 
    # ------------------------------------------------------------------
    # M_x, M_y: encode normal-vector perturbation into the spline RHS.
    # Each spline segment i connects point i to point i+1, so both
    # endpoints receive their respective normal vector components.
    # ------------------------------------------------------------------
    M_x = np.zeros((no_splines * 4, no_points))
    M_y = np.zeros((no_splines * 4, no_points))
 
    for i in range(no_splines):
        j = i * 4
        if i < no_points - 1:
            M_x[j,     i    ] = normvectors[i,     0]
            M_x[j + 1, i + 1] = normvectors[i + 1, 0]
            M_y[j,     i    ] = normvectors[i,     1]
            M_y[j + 1, i + 1] = normvectors[i + 1, 1]
        else:
            # Close the loop: last segment connects back to point 0
            M_x[j,     i] = normvectors[i, 0]
            M_x[j + 1, 0] = normvectors[0, 0]
            M_y[j,     i] = normvectors[i, 1]
            M_y[j + 1, 0] = normvectors[0, 1]
 
    # ------------------------------------------------------------------
    # q_x, q_y: pack waypoint coordinates into the spline RHS.
    # Same two-endpoint-per-segment structure as M_x/M_y.
    # ------------------------------------------------------------------
    q_x = np.zeros((no_splines * 4, 1))
    q_y = np.zeros((no_splines * 4, 1))
 
    for i in range(no_splines):
        j = i * 4
        if i < no_points - 1:
            q_x[j,     0] = reftrack[i,     0]
            q_x[j + 1, 0] = reftrack[i + 1, 0]
            q_y[j,     0] = reftrack[i,     1]
            q_y[j + 1, 0] = reftrack[i + 1, 1]
        else:
            q_x[j,     0] = reftrack[i, 0]
            q_x[j + 1, 0] = reftrack[0, 0]
            q_y[j,     0] = reftrack[i, 1]
            q_y[j + 1, 0] = reftrack[0, 1]
 
    # ------------------------------------------------------------------
    # First derivatives at each waypoint (diagonal matrix form so that
    # element-wise operations below stay geometrically correct per point)
    # ------------------------------------------------------------------
    x_prime = np.eye(no_points, no_points) * np.matmul(np.matmul(A_ex_b, A_inv), q_x)
    y_prime = np.eye(no_points, no_points) * np.matmul(np.matmul(A_ex_b, A_inv), q_y)
 
    x_prime_sq        = np.power(x_prime, 2)
    y_prime_sq        = np.power(y_prime, 2)
    x_prime_y_prime = -2 * np.matmul(x_prime, y_prime)
 
    # Curvature denominator (x'^2 + y'^2)^1.5, element-wise on diagonal
    curv_den    = np.power(x_prime_sq + y_prime_sq, 1.5)
    curv_part   = np.divide(1.0, curv_den,
                            out=np.zeros_like(curv_den), where=curv_den != 0)
    curv_part_sq = np.power(curv_part, 2)
 
    # P matrices: quadratic weights for the curvature cost
    P_xx = np.matmul(curv_part_sq, y_prime_sq)
    P_yy = np.matmul(curv_part_sq, x_prime_sq)
    P_xy = np.matmul(curv_part_sq, x_prime_y_prime)
 
    # ------------------------------------------------------------------
    # Final QP matrices
    # T_nx / T_ny map alpha to the second-derivative perturbation
    # ------------------------------------------------------------------
    T_nx = np.matmul(T_c, M_x)
    T_ny = np.matmul(T_c, M_y)
 
    H_x = np.matmul(T_nx.T, np.matmul(P_xx, T_nx))
    H_xy = np.matmul(T_ny.T, np.matmul(P_xy, T_nx))
    H_y = np.matmul(T_ny.T, np.matmul(P_yy, T_ny))
    H    = H_x + H_xy + H_y
    H    = (H + H.T) / 2.0   # enforce symmetry
 
    f_x = 2 * np.matmul(np.matmul(q_x.T, T_c.T), np.matmul(P_xx, T_nx))
    f_xy = np.matmul(np.matmul(q_x.T, T_c.T), np.matmul(P_xy, T_ny)) \
           + np.matmul(np.matmul(q_y.T, T_c.T), np.matmul(P_xy, T_nx))
    f_y = 2 * np.matmul(np.matmul(q_y.T, T_c.T), np.matmul(P_yy, T_ny))
    f = f_x + f_xy + f_y
    f = np.squeeze(f)
 
    # ------------------------------------------------------------------
    # Kappa (curvature) constraints
    # ------------------------------------------------------------------
    Q_x = np.matmul(curv_part, y_prime)
    Q_y = np.matmul(curv_part, x_prime)
 
    E_kappa = np.matmul(Q_y, T_ny) - np.matmul(Q_x, T_nx)
    k_kappa_ref = np.matmul(Q_y, np.matmul(T_c, q_y)) - np.matmul(Q_x, np.matmul(T_c, q_x))
 
    con_ge = np.ones((no_points, 1)) * kappa_bound - k_kappa_ref
    con_le = -(np.ones((no_points, 1)) * -kappa_bound - k_kappa_ref)
    con_stack = np.append(con_ge, con_le)
 
    # ------------------------------------------------------------------
    # Track-width inequality constraints
    # alpha ≤  dev_max_right  (shift right)
    # alpha ≥ -dev_max_left   (shift left)
    # ------------------------------------------------------------------
    dev_max_right = reftrack[:, 2] - w_veh / 2.0
    dev_max_left  = reftrack[:, 3] - w_veh / 2.0
 
    if np.any(-dev_max_right > dev_max_left) or np.any(-dev_max_left > dev_max_right):
        raise RuntimeError(
            "Track too narrow for the given vehicle width at one or more points."
        )
 
    # G alpha ≤ h  (quadprog sign convention: passed as -G, -h)
    G = np.vstack((
         np.eye(no_points),    # alpha ≤  dev_max_right
        -np.eye(no_points),    # alpha ≥ -dev_max_left
         E_kappa,              # curvature ≤  kappa_bound
        -E_kappa,              # curvature ≥ -kappa_bound
    ))
    h = np.append(dev_max_right, dev_max_left)
    h = np.append(h, con_stack)
 
    # ------------------------------------------------------------------
    # Solve QP
    # quadprog: min ½ α'Hα + f'α  s.t.  G α ≤ h
    # passed as: solve_qp(H, -f, -G.T, -h)
    #
    # quadprog requires H to be strictly positive definite. The curvature
    # Hessian is only positive *semi*-definite (rank deficiency arises from
    # straight sections where the denominator → 0), so add a small
    # regularisation term. 1e-6 * I fixes numerical rank without meaningfully
    # changing the solution on any real track.
    # ------------------------------------------------------------------
 
    try:
        alpha_mincurv = quadprog.solve_qp(H, -f, -G.T, -h, 0)[0]
    except ValueError as exc:
        raise RuntimeError(f"QP solver failed: {exc}") from exc
 
    # ------------------------------------------------------------------
    # Curvature error: compare linearisation around refline vs solution
    # ------------------------------------------------------------------
    q_x_tmp = q_x + np.matmul(M_x, np.expand_dims(alpha_mincurv, 1))
    q_y_tmp = q_y + np.matmul(M_y, np.expand_dims(alpha_mincurv, 1))
 
    x_prime_tmp = np.eye(no_points, no_points) * np.matmul(np.matmul(A_ex_b, A_inv), q_x_tmp)
    y_prime_tmp = np.eye(no_points, no_points) * np.matmul(np.matmul(A_ex_b, A_inv), q_y_tmp)
 
    x_prime_prime = np.squeeze(np.matmul(T_c, q_x) + np.matmul(T_nx, np.expand_dims(alpha_mincurv, 1)))
    y_prime_prime = np.squeeze(np.matmul(T_c, q_y) + np.matmul(T_ny, np.expand_dims(alpha_mincurv, 1)))
 
    curv_orig_lin = np.zeros(no_points)
    curv_sol_lin  = np.zeros(no_points)
 
    for i in range(no_points):
        curv_orig_lin[i] = (x_prime[i, i] * y_prime_prime[i] - y_prime[i, i] * x_prime_prime[i]) \
                          / math.pow(math.pow(x_prime[i, i], 2) + math.pow(y_prime[i, i], 2), 1.5)
        curv_sol_lin[i] = (x_prime_tmp[i, i] * y_prime_prime[i] - y_prime_tmp[i, i] * x_prime_prime[i]) \
                           / math.pow(math.pow(x_prime_tmp[i, i], 2) + math.pow(y_prime_tmp[i, i], 2), 1.5)
 
    curv_error_max = float(np.amax(np.abs(curv_sol_lin - curv_orig_lin)))
 
    # ------------------------------------------------------------------
    # Debug plot
    # ------------------------------------------------------------------
    if plot_debug:
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
 
        opt_x = reftrack[:, 0] + alpha_mincurv * normvectors[:, 0]
        opt_y = reftrack[:, 1] + alpha_mincurv * normvectors[:, 1]
        axes[0].plot(reftrack[:, 0], reftrack[:, 1], ":", label="Reference track")
        axes[0].plot(opt_x, opt_y, label="Optimised path")
        axes[0].set_aspect("equal")
        axes[0].legend()
        axes[0].set_title("Path comparison")
 
        axes[1].plot(curv_orig_lin, label="Original linearisation")
        axes[1].plot(curv_sol_lin,  label="Solution linearisation")
        axes[1].legend()
        axes[1].set_title("Curvature comparison")
 
        plt.tight_layout()
        plt.show()
 
    return alpha_mincurv, curv_error_max