import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button

from .constants import OCCUPIED, UNKNOWN
from .smoothing import smooth_path
from .viz import build_vis


def pick_spline_control_points(occupancy_grid, start_pos):
    """
    Opens an interactive map window to manually define raceline control points.

    Returns
    -------
    list of (row, col) | None
    """
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
        "Manual spline mode: LEFT-CLICK add control points  |  RIGHT-CLICK undo  |  Confirm when done\n"
        "Green = start/finish control point  |  Yellow = control points  |  Cyan = spline preview",
        color="white",
        fontsize=11,
        pad=12,
    )

    status_text = ax.text(
        0.5,
        -0.055,
        "Place at least 3 additional control points, then Confirm",
        transform=ax.transAxes,
        ha="center",
        va="top",
        color="#aaaacc",
        fontsize=10,
    )

    ax_conf = plt.axes([0.60, 0.02, 0.20, 0.055])
    btn_conf = Button(ax_conf, "Confirm spline", color="#22224a", hovercolor="#3333aa")
    btn_conf.label.set_color("white")
    btn_conf.label.set_fontsize(10)

    state = {
        "points": [start_pos],
        "markers": [],
        "labels": [],
        "control_poly": None,
        "spline_poly": None,
        "confirmed": False,
    }

    h_g, w_g = occupancy_grid.shape

    def _is_valid(row, col):
        if not (0 <= row < h_g and 0 <= col < w_g):
            return False
        if occupancy_grid[row, col] == OCCUPIED:
            status_text.set_text("Point is inside an obstacle - choose a free (light) area")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return False
        if occupancy_grid[row, col] == UNKNOWN:
            status_text.set_text("Point is in unknown space - choose a free (light) area")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return False
        return True

    def _rebuild_artists():
        for mk in state["markers"]:
            mk.remove()
        for lb in state["labels"]:
            lb.remove()
        state["markers"].clear()
        state["labels"].clear()

        if state["control_poly"] is not None:
            state["control_poly"].remove()
            state["control_poly"] = None

        if state["spline_poly"] is not None:
            state["spline_poly"].remove()
            state["spline_poly"] = None

        pts = state["points"]
        for i, (r, c) in enumerate(pts):
            if i == 0:
                mk, = ax.plot(
                    c,
                    r,
                    "o",
                    color="#00e676",
                    markersize=14,
                    markeredgewidth=2,
                    markeredgecolor="white",
                    zorder=7,
                )
                lb = ax.annotate(
                    "S/F",
                    xy=(c, r),
                    xytext=(c + 10, r + 10),
                    color="#00e676",
                    fontsize=9,
                    fontweight="bold",
                    zorder=8,
                    arrowprops=dict(arrowstyle="->", color="#00e676", lw=1.3),
                )
            else:
                mk, = ax.plot(
                    c,
                    r,
                    "s",
                    color="#ffab00",
                    markersize=10,
                    markeredgewidth=1.6,
                    markeredgecolor="white",
                    zorder=6,
                )
                lb = ax.annotate(
                    str(i),
                    xy=(c, r),
                    xytext=(c + 6, r + 6),
                    color="#ffab00",
                    fontsize=8,
                    fontweight="bold",
                    zorder=7,
                    arrowprops=dict(arrowstyle="->", color="#ffab00", lw=1.1),
                )

            state["markers"].append(mk)
            state["labels"].append(lb)

        if len(pts) >= 2:
            control_pts = np.array([(c, r) for r, c in pts] + [(pts[0][1], pts[0][0])], dtype=float)
            ln, = ax.plot(control_pts[:, 0], control_pts[:, 1], "--", color="#ffffff55", lw=1.2, zorder=3)
            state["control_poly"] = ln

        if len(pts) >= 4:
            preview_n = max(200, len(pts) * 25)
            preview = smooth_path(
                pts,
                smoothing_factor=float(len(pts)) * 0.2,
                num_points=preview_n,
                closed=True,
            )
            spline_pts = np.array([(c, r) for r, c in preview], dtype=float)
            ln, = ax.plot(spline_pts[:, 0], spline_pts[:, 1], "-", color="#00d2dc", lw=2.5, zorder=4)
            state["spline_poly"] = ln

        n = len(pts)
        if n < 4:
            status_text.set_text(f"{n} point(s) placed - add at least {4 - n} more to confirm")
            status_text.set_color("#aaaacc")
        else:
            status_text.set_text(f"{n} control points placed - right-click to undo or Confirm spline")
            status_text.set_color("#69f0ae")

        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes is not ax:
            return
        if event.xdata is None or event.ydata is None:
            return

        col = int(round(event.xdata))
        row = int(round(event.ydata))

        if event.button == 3:
            if len(state["points"]) > 1:
                state["points"].pop()
                _rebuild_artists()
            return

        if event.button != 1:
            return

        if not _is_valid(row, col):
            return

        state["points"].append((row, col))
        _rebuild_artists()

    def on_confirm(_event):
        if len(state["points"]) < 4:
            status_text.set_text("Need at least 4 control points total to fit a closed spline")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return
        state["confirmed"] = True
        plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", on_click)
    btn_conf.on_clicked(on_confirm)

    _rebuild_artists()
    plt.show()

    if state["confirmed"]:
        return state["points"]
    return None
