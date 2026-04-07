import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button
import time

from .constants import OCCUPIED, UNKNOWN
from .smoothing import smooth_path
from .viz import build_vis


def edit_raceline_with_drag(
    occupancy_grid,
    initial_path,
    handle_stride=25,
    smoothing_factor=1500.0,
    lock_start=True,
    adaptive_handles=True,
    curvature_handle_ratio=0.12,
    min_handle_spacing=8,
    drag_preview_points=260,
    final_preview_points=None,
    max_redraw_hz=24,
):
    """
    Opens an interactive editor to drag sparse control handles and reshape an
    already generated loop raceline.

    Parameters
    ----------
    occupancy_grid : np.ndarray
    initial_path   : list[(row, col)]
        Input loop path (typically A* output or its smoothed version).
    handle_stride  : int
        Every Nth point from the input path becomes a draggable handle.
    smoothing_factor : float
        Spline smoothing used to rebuild the edited raceline preview/output.
    lock_start : bool
        If True, start/finish handle (index 0) cannot be dragged.
    adaptive_handles : bool
        If True, adds extra handles in high-curvature areas.
    curvature_handle_ratio : float
        Fraction of points considered as additional curvature-based handles.
    min_handle_spacing : int
        Minimum spacing (in source indices) between selected handles.
    drag_preview_points : int
        Number of preview points while dragging (low for responsiveness).
    final_preview_points : int or None
        Number of preview points on release/confirm (high for quality).
        If None, uses max(400, len(initial_path)).
    max_redraw_hz : float
        Redraw rate cap while dragging.

    Returns
    -------
    list[(row, col)] | None
        Edited path if confirmed, or None if cancelled.
    """
    if not initial_path or len(initial_path) < 6:
        print("[drag editor] Path too short for interactive editing.")
        return initial_path

    vis = build_vis(occupancy_grid)

    fig, ax = plt.subplots(figsize=(14, 11))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    plt.subplots_adjust(bottom=0.13)

    ax.imshow(vis, origin="lower")
    ax.tick_params(colors="#888888")
    for spine in ax.spines.values():
        spine.set_edgecolor("#333355")

    ax.set_title(
        "Drag Edit Mode: LEFT-CLICK+DRAG handles  |  Confirm to accept  |  Cancel to discard\n"
        "Green = start/finish handle (locked by default)  |  Orange = editable handles",
        color="white",
        fontsize=11,
        pad=12,
    )

    status_text = ax.text(
        0.5,
        -0.055,
        "Drag orange handles to reshape the raceline, then Confirm",
        transform=ax.transAxes,
        ha="center",
        va="top",
        color="#aaaacc",
        fontsize=10,
    )

    ax_conf = plt.axes([0.58, 0.02, 0.16, 0.055])
    btn_conf = Button(ax_conf, "Confirm edit", color="#22224a", hovercolor="#3333aa")
    btn_conf.label.set_color("white")

    ax_cancel = plt.axes([0.76, 0.02, 0.14, 0.055])
    btn_cancel = Button(ax_cancel, "Cancel", color="#3a1f1f", hovercolor="#5a2a2a")
    btn_cancel.label.set_color("white")

    n_in = len(initial_path)
    stride = max(3, int(handle_stride))

    if np.allclose(initial_path[0], initial_path[-1]):
        source_path = initial_path[:-1]
    else:
        source_path = initial_path

    n_src = len(source_path)

    def _angle_score(i):
        ip = (i - 1) % n_src
        inx = (i + 1) % n_src

        p0 = np.array(source_path[ip], dtype=float)
        p1 = np.array(source_path[i], dtype=float)
        p2 = np.array(source_path[inx], dtype=float)

        v1 = p1 - p0
        v2 = p2 - p1

        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-9 or n2 < 1e-9:
            return 0.0

        dot = float(np.dot(v1, v2) / (n1 * n2))
        dot = max(-1.0, min(1.0, dot))
        return float(np.arccos(dot))

    base_idx = set(range(0, n_src, stride))
    base_idx.add(0)

    min_sep = max(2, int(min_handle_spacing))

    def _far_enough(candidate, chosen):
        for j in chosen:
            d = abs(candidate - j)
            d = min(d, n_src - d)
            if d < min_sep:
                return False
        return True

    if adaptive_handles and n_src >= 12:
        scores = np.array([_angle_score(i) for i in range(n_src)], dtype=float)
        target_extra = int(max(1, round(float(curvature_handle_ratio) * n_src)))
        ranked = np.argsort(-scores).tolist()
        chosen = set(base_idx)

        for i in ranked:
            if len(chosen) >= len(base_idx) + target_extra:
                break
            if i in chosen:
                continue
            if not _far_enough(i, chosen):
                continue
            chosen.add(i)

        handle_idx = sorted(chosen)
    else:
        handle_idx = sorted(base_idx)

    controls = [(float(source_path[i][0]), float(source_path[i][1])) for i in handle_idx]

    if final_preview_points is None:
        final_preview_points = max(400, n_in)
    drag_preview_points = max(120, int(drag_preview_points))
    final_preview_points = max(drag_preview_points, int(final_preview_points))
    redraw_period = 1.0 / max(1.0, float(max_redraw_hz))

    state = {
        "controls": controls,
        "drag_idx": None,
        "confirmed": False,
        "cancelled": False,
        "preview": None,
        "poly_artist": None,
        "sf_artist": None,
        "handles_artist": None,
        "dragging": False,
        "last_redraw_t": 0.0,
    }

    h_g, w_g = occupancy_grid.shape

    def _is_valid(row_f, col_f):
        row = int(round(row_f))
        col = int(round(col_f))
        if not (0 <= row < h_g and 0 <= col < w_g):
            status_text.set_text("Point is out of map bounds")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return False
        if occupancy_grid[row, col] == OCCUPIED:
            status_text.set_text("Point is inside an obstacle - move to free space")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return False
        if occupancy_grid[row, col] == UNKNOWN:
            status_text.set_text("Point is in unknown space - move to free space")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return False
        return True

    def _rebuild_preview(preview_n):
        state["preview"] = smooth_path(
            state["controls"],
            smoothing_factor=float(smoothing_factor),
            num_points=preview_n,
            closed=True,
        )

    def _redraw(force=False):
        now = time.perf_counter()
        if state["dragging"] and not force and (now - state["last_redraw_t"] < redraw_period):
            return

        if state["poly_artist"] is not None:
            state["poly_artist"].remove()
        if state["sf_artist"] is not None:
            state["sf_artist"].remove()
        if state["handles_artist"] is not None:
            state["handles_artist"].remove()

        preview_n = drag_preview_points if state["dragging"] else final_preview_points
        _rebuild_preview(preview_n)

        spline_pts = np.array([(c, r) for r, c in state["preview"]], dtype=float)
        line, = ax.plot(spline_pts[:, 0], spline_pts[:, 1], "-", color="#00d2dc", lw=2.8, zorder=4)
        state["poly_artist"] = line

        controls_arr = np.array([(c, r) for r, c in state["controls"]], dtype=float)

        sf = controls_arr[0]
        sf_artist, = ax.plot(
            sf[0],
            sf[1],
            "o",
            color="#00e676",
            markersize=13,
            markeredgewidth=2,
            markeredgecolor="white",
            zorder=7,
        )
        state["sf_artist"] = sf_artist

        if len(controls_arr) > 1:
            handles = controls_arr[1:]
            handles_artist, = ax.plot(
                handles[:, 0],
                handles[:, 1],
                "s",
                color="#ffab00",
                markersize=9,
                markeredgewidth=1.4,
                markeredgecolor="white",
                zorder=6,
                linestyle="None",
            )
            state["handles_artist"] = handles_artist

        status_text.set_text("Drag orange handles to reshape the raceline, then Confirm")
        status_text.set_color("#69f0ae")
        state["last_redraw_t"] = now
        fig.canvas.draw_idle()

    def _pick_handle(col_f, row_f):
        pts = np.array([(c, r) for r, c in state["controls"]], dtype=float)
        d2 = (pts[:, 0] - col_f) ** 2 + (pts[:, 1] - row_f) ** 2
        idx = int(np.argmin(d2))
        if d2[idx] > 10.0**2:
            return None
        if lock_start and idx == 0:
            return None
        return idx

    def on_press(event):
        if event.inaxes is not ax:
            return
        if event.button != 1:
            return
        if event.xdata is None or event.ydata is None:
            return

        idx = _pick_handle(event.xdata, event.ydata)
        if idx is None:
            return
        state["drag_idx"] = idx
        state["dragging"] = True
        status_text.set_text(f"Dragging handle {idx}")
        status_text.set_color("#69f0ae")
        fig.canvas.draw_idle()

    def on_motion(event):
        idx = state["drag_idx"]
        if idx is None:
            return
        if event.inaxes is not ax:
            return
        if event.xdata is None or event.ydata is None:
            return

        row_f = float(event.ydata)
        col_f = float(event.xdata)

        if not _is_valid(row_f, col_f):
            return

        state["controls"][idx] = (row_f, col_f)
        _redraw()

    def on_release(_event):
        state["drag_idx"] = None
        state["dragging"] = False
        _redraw(force=True)

    def on_confirm(_event):
        state["confirmed"] = True
        plt.close(fig)

    def on_cancel(_event):
        state["cancelled"] = True
        plt.close(fig)

    _redraw()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("button_release_event", on_release)
    btn_conf.on_clicked(on_confirm)
    btn_cancel.on_clicked(on_cancel)

    plt.show()

    if state["cancelled"]:
        return None
    if state["confirmed"]:
        return state["preview"]
    return None
