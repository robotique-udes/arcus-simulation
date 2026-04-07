import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from .constants import OCCUPIED, UNKNOWN
from .viz import build_vis


def pick_waypoints(occupancy_grid, start_pos):
    """
    Opens an interactive map window where the user places intermediate
    checkpoints around the track and optionally moves the start/finish point.

    Returns
    -------
    tuple( list of (row, col), (row, col) ) | None
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

    state = {
        "start_pos": start_pos,
        "waypoints": [],
        "markers": [],
        "labels": [],
        "lines": [],
        "sf_marker": None,
        "sf_label": None,
        "move_mode": False,
        "confirmed": False,
    }

    ax.set_title(
        "LEFT-CLICK to add checkpoints  |  RIGHT-CLICK to undo last  |  "
        '"Move S/F" to relocate start/finish  |  "Confirm" when done\n'
        "Green = start/finish  |  Yellow squares = intermediate checkpoints",
        color="white",
        fontsize=11,
        pad=12,
    )

    status_text = ax.text(
        0.5,
        -0.055,
        "Left-click to place your first checkpoint",
        transform=ax.transAxes,
        ha="center",
        va="top",
        color="#aaaacc",
        fontsize=10,
    )

    ax_move = plt.axes([0.18, 0.02, 0.22, 0.055])
    btn_move = Button(
        ax_move,
        "Move Start / Finish",
        color="#22224a",
        hovercolor="#1a3a1a",
    )
    btn_move.label.set_color("white")
    btn_move.label.set_fontsize(10)

    ax_conf = plt.axes([0.60, 0.02, 0.20, 0.055])
    btn_conf = Button(ax_conf, "Confirm waypoints", color="#22224a", hovercolor="#3333aa")
    btn_conf.label.set_color("white")
    btn_conf.label.set_fontsize(10)

    def _draw_sf(row, col):
        if state["sf_marker"] is not None:
            state["sf_marker"].remove()
        if state["sf_label"] is not None:
            state["sf_label"].remove()

        mk, = ax.plot(
            col,
            row,
            "o",
            color="#00e676",
            markersize=14,
            markeredgewidth=2,
            markeredgecolor="white",
            zorder=7,
        )
        lbl = ax.annotate(
            "S/F",
            xy=(col, row),
            xytext=(col + 10, row + 10),
            color="#00e676",
            fontsize=9,
            fontweight="bold",
            zorder=8,
            arrowprops=dict(arrowstyle="->", color="#00e676", lw=1.5),
        )

        state["sf_marker"] = mk
        state["sf_label"] = lbl

    _draw_sf(start_pos[0], start_pos[1])

    def _redraw_connectors():
        for ln in state["lines"]:
            ln.remove()
        state["lines"].clear()

        sp = state["start_pos"]
        all_pts = [sp] + state["waypoints"] + [sp]
        for i in range(len(all_pts) - 1):
            r0, c0 = all_pts[i]
            r1, c1 = all_pts[i + 1]
            is_closing = i == len(all_pts) - 2
            style = "-." if is_closing else "--"
            color = "#00e67644" if is_closing else "#ffffff44"
            ln, = ax.plot([c0, c1], [r0, r1], style, color=color, lw=1.2, zorder=3)
            state["lines"].append(ln)

    def _update_status():
        if state["move_mode"]:
            status_text.set_text(
                "MOVE MODE - left-click to set the new Start/Finish position  |  "
                "right-click to cancel"
            )
            status_text.set_color("#00e676")
            fig.canvas.draw_idle()
            return

        n = len(state["waypoints"])
        if n == 0:
            msg, color = "Left-click to place your first checkpoint", "#aaaacc"
        elif n == 1:
            msg = "1 checkpoint placed  |  right-click to undo  |  confirm or add more"
            color = "#69f0ae"
        else:
            msg = f"{n} checkpoints placed  |  right-click to undo  |  confirm when ready"
            color = "#69f0ae"
        status_text.set_text(msg)
        status_text.set_color(color)
        fig.canvas.draw_idle()

    def _validate_cell(row, col):
        h_g, w_g = occupancy_grid.shape
        if not (0 <= row < h_g and 0 <= col < w_g):
            return False
        if occupancy_grid[row, col] == OCCUPIED:
            status_text.set_text("That point is inside an obstacle - choose a free (light) area")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return False
        if occupancy_grid[row, col] == UNKNOWN:
            status_text.set_text("That point is in unknown space - choose a free (light) area")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return False
        return True

    def on_click(event):
        if event.inaxes is not ax:
            return
        if event.xdata is None or event.ydata is None:
            return

        col = int(round(event.xdata))
        row = int(round(event.ydata))

        if state["move_mode"]:
            if event.button == 3:
                state["move_mode"] = False
                btn_move.color = "#22224a"
                btn_move.hovercolor = "#1a3a1a"
                btn_move.ax.set_facecolor("#22224a")
                _update_status()
                fig.canvas.draw_idle()
                return

            if event.button == 1:
                if not _validate_cell(row, col):
                    return
                state["start_pos"] = (row, col)
                state["move_mode"] = False
                btn_move.color = "#22224a"
                btn_move.hovercolor = "#1a3a1a"
                btn_move.ax.set_facecolor("#22224a")
                _draw_sf(row, col)
                _redraw_connectors()
                _update_status()
                fig.canvas.draw_idle()
            return

        if event.button == 3:
            if not state["waypoints"]:
                return
            state["waypoints"].pop()
            state["markers"].pop().remove()
            state["labels"].pop().remove()
            _redraw_connectors()
            _update_status()
            fig.canvas.draw_idle()
            return

        if event.button != 1:
            return

        if not _validate_cell(row, col):
            return

        state["waypoints"].append((row, col))
        n = len(state["waypoints"])

        mk, = ax.plot(
            col,
            row,
            "s",
            color="#ffab00",
            markersize=12,
            markeredgewidth=2,
            markeredgecolor="white",
            zorder=6,
        )
        state["markers"].append(mk)

        lbl = ax.annotate(
            str(n),
            xy=(col, row),
            xytext=(col + 7, row + 7),
            color="#ffab00",
            fontsize=8,
            fontweight="bold",
            zorder=7,
            arrowprops=dict(arrowstyle="->", color="#ffab00", lw=1.2),
        )
        state["labels"].append(lbl)

        _redraw_connectors()
        _update_status()
        fig.canvas.draw_idle()

    def on_move_sf(_event):
        state["move_mode"] = not state["move_mode"]
        if state["move_mode"]:
            btn_move.ax.set_facecolor("#1a3a1a")
        else:
            btn_move.ax.set_facecolor("#22224a")
        _update_status()
        fig.canvas.draw_idle()

    def on_confirm(_event):
        if not state["waypoints"]:
            status_text.set_text("Place at least one checkpoint before confirming")
            status_text.set_color("#ff6d00")
            fig.canvas.draw_idle()
            return
        state["confirmed"] = True
        plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", on_click)
    btn_move.on_clicked(on_move_sf)
    btn_conf.on_clicked(on_confirm)

    plt.show()

    if state["confirmed"] and state["waypoints"]:
        return state["waypoints"], state["start_pos"]
    return None
