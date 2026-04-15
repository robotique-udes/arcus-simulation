import numpy as np
from scipy.interpolate import splprep, splev


def smooth_path(path, smoothing_factor=None, num_points=None, closed=False):
    """
    Smooths a raw (row, col) path using a parametric cubic B-spline
    (scipy splprep / splev).

    Returns
    -------
    list of (float, float)  as (row, col) floats
    """
    if len(path) < 4:
        print("[smooth_path] Path too short to smooth (need >= 4 points). Returning raw path.")
        return path

    rows = np.array([p[0] for p in path], dtype=float)
    cols = np.array([p[1] for p in path], dtype=float)

    if num_points is None:
        num_points = len(path)

    if smoothing_factor is None:
        smoothing_factor = float(len(path))

    try:
        tck, _ = splprep([rows, cols], s=smoothing_factor, per=closed, k=3)

        u_new = np.linspace(0, 1, num_points)
        rows_s, cols_s = splev(u_new, tck)

        return list(zip(rows_s.tolist(), cols_s.tolist()))

    except Exception as e:
        print(f"[smooth_path] Spline fitting failed ({e}). Returning raw path.")
        return path

