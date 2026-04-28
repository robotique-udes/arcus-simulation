import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox

from .constants import OCCUPIED, UNKNOWN
from .viz import build_vis


def pick_speed_zones(occupancy_grid):
    """
    Open an interactive map window to draw one or more polygons and assign a max speed.

    Controls
    --------
    - Left-click on map: add a vertex to the current polygon.
    - Right-click on map: undo last vertex in the current polygon.
    - Enter max speed in textbox before committing a polygon.
    - New Polygon button: commit current polygon (min 3 vertices) and start a new one.
    - Save button: commit current polygon if valid and exit with all polygons.
    - Cancel button: discard and exit.

    Returns
    -------
    list[dict] | None
        Polygons represented as dictionaries:
        {"points": [(row, col), ...], "max_speed": float}
        Returns None when cancelled or when nothing valid is drawn.
    """
    return _pick_polygons_internal(occupancy_grid, with_speed=True)


def _pick_polygons_internal(occupancy_grid, with_speed):
    vis = build_vis(occupancy_grid)

    fig, ax = plt.subplots(figsize=(14, 11))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    plt.subplots_adjust(bottom=0.17 if with_speed else 0.13)

    ax.imshow(vis, origin="lower")
    ax.tick_params(colors="#888888")
    for spine in ax.spines.values():
        spine.set_edgecolor("#333355")

    ax.set_title(
        "Speed Zone Drawer: LEFT-CLICK add vertex  |  RIGHT-CLICK undo point\n"
        "Use 'New Polygon' for additional shapes, then 'Save & Exit'",
        color="white",
        fontsize=11,
        pad=12,
    )

    status_text = ax.text(
        0.5,
        -0.055,
        "Left-click to place vertices for polygon #1",
        transform=ax.transAxes,
        ha="center",
        va="top",
        color="#aaaacc",
        fontsize=10,
    )

    ax_new = plt.axes([0.28, 0.02, 0.16, 0.055]) # x, y, width, height
    btn_new = Button(ax_new, "New Polygon", color="#224222", hovercolor="#2b5f2b")
    btn_new.label.set_color("white")

    ax_save = plt.axes([0.46, 0.02, 0.16, 0.055])
    btn_save = Button(ax_save, "Save & Exit", color="#22224a", hovercolor="#3333aa")
    btn_save.label.set_color("white")

    ax_cancel = plt.axes([0.64, 0.02, 0.16, 0.055])
    btn_cancel = Button(ax_cancel, "Cancel", color="#3a1f1f", hovercolor="#5a2a2a")
    btn_cancel.label.set_color("white")

    txt_speed = None
    if with_speed:
        ax_speed = plt.axes([0.12, 0.02, 0.15, 0.055])
        txt_speed = TextBox(ax_speed, "Max Speed (m/s): ", initial="")
        txt_speed.label.set_color("white")
        txt_speed.text_disp.set_color("#000000")
        txt_speed.text_disp.set_size(15)
        ax_speed.set_facecolor("#10172a")

    state = {
        "current": [],
        "polygons": [],
        "current_markers": [],
        "current_labels": [],
        "current_lines": [],
        "saved_lines": [],
        "saved_labels": [],
        "confirmed": False,
        "cancelled": False,
    }

    h_g, w_g = occupancy_grid.shape

    def _set_status(msg, color="#aaaacc"):
        status_text.set_text(msg)
        status_text.set_color(color)
        fig.canvas.draw_idle()

    def _validate_cell(row, col):
        if not (0 <= row < h_g and 0 <= col < w_g):
            _set_status("Point is out of map bounds", "#ff6d00")
            return False

        if occupancy_grid[row, col] == OCCUPIED:
            _set_status("Point is inside an obstacle; choose free space", "#ff6d00")
            return False

        if occupancy_grid[row, col] == UNKNOWN:
            _set_status("Point is in unknown space; choose free space", "#ff6d00")
            return False

        return True

    def _read_max_speed():
        if not with_speed:
            return None

        raw = txt_speed.text.strip()
        if not raw:
            _set_status("Enter max speed before saving polygon", "#ff6d00")
            return None

        try:
            value = float(raw)
        except ValueError:
            _set_status("Max speed must be numeric", "#ff6d00")
            return None

        if value <= 0.0:
            _set_status("Max speed must be > 0", "#ff6d00")
            return None

        return value

    def _clear_current_visuals():
        for artist in state["current_markers"]:
            artist.remove()
        for artist in state["current_labels"]:
            artist.remove()
        for artist in state["current_lines"]:
            artist.remove()

        state["current_markers"].clear()
        state["current_labels"].clear()
        state["current_lines"].clear()

    def _redraw_current_polygon():
        for artist in state["current_lines"]:
            artist.remove()
        state["current_lines"].clear()

        pts = state["current"]
        if len(pts) < 2:
            fig.canvas.draw_idle()
            return

        for i in range(len(pts) - 1):
            r0, c0 = pts[i]
            r1, c1 = pts[i + 1]
            ln, = ax.plot([c0, c1], [r0, r1], "--", color="#00d2dc99", lw=1.5, zorder=3)
            state["current_lines"].append(ln)

        if len(pts) >= 3:
            r0, c0 = pts[-1]
            r1, c1 = pts[0]
            ln, = ax.plot([c0, c1], [r0, r1], "-.", color="#00d2dc66", lw=1.2, zorder=3)
            state["current_lines"].append(ln)

        fig.canvas.draw_idle()

    def _draw_saved_polygon(poly_idx, polygon):
        color = "#ffab00"

        for i in range(len(polygon)):
            r0, c0 = polygon[i]
            r1, c1 = polygon[(i + 1) % len(polygon)]
            ln, = ax.plot([c0, c1], [r0, r1], "-", color=color, lw=2.0, zorder=4)
            state["saved_lines"].append(ln)

        center_row = sum(p[0] for p in polygon) / len(polygon)
        center_col = sum(p[1] for p in polygon) / len(polygon)
        label_text = f"P{poly_idx + 1}"
        if with_speed:
            speed = state["polygons"][poly_idx]["max_speed"]
            label_text = f"P{poly_idx + 1}\n{speed:g} m/s"

        lbl = ax.text(
            center_col,
            center_row,
            label_text,
            color="#ffd54f",
            fontsize=9,
            fontweight="bold",
            ha="center",
            va="center",
            bbox={"boxstyle": "round,pad=0.25", "fc": "#00000066", "ec": "#ffd54f", "lw": 1},
            zorder=6,
        )
        state["saved_labels"].append(lbl)

    def _commit_current_polygon():
        if len(state["current"]) < 3:
            _set_status("A polygon needs at least 3 vertices", "#ff6d00")
            return False

        if with_speed:
            max_speed = _read_max_speed()
            if max_speed is None:
                return False
            polygon = {"points": list(state["current"]), "max_speed": max_speed}
            polygon_points = polygon["points"]
        else:
            polygon = list(state["current"])
            polygon_points = polygon

        state["polygons"].append(polygon)

        _draw_saved_polygon(len(state["polygons"]) - 1, polygon_points)

        _clear_current_visuals()
        state["current"].clear()

        _set_status(
            f"Saved polygon #{len(state['polygons'])}. Draw next polygon or click Save & Exit",
            "#69f0ae",
        )
        if with_speed:
            txt_speed.set_val("")
        return True

    def on_click(event):
        if event.inaxes is not ax:
            return
        if event.xdata is None or event.ydata is None:
            return

        col = int(round(event.xdata))
        row = int(round(event.ydata))

        if event.button == 3:
            if not state["current"]:
                _set_status("No vertex to undo in current polygon", "#ffab00")
                return

            state["current"].pop()
            state["current_markers"].pop().remove()
            state["current_labels"].pop().remove()
            _redraw_current_polygon()

            if state["current"]:
                _set_status(
                    f"Current polygon vertices: {len(state['current'])}",
                    "#aaaacc",
                )
            else:
                _set_status(
                    f"Current polygon cleared. Click to start polygon #{len(state['polygons']) + 1}",
                    "#aaaacc",
                )
            return

        if event.button != 1:
            return

        if not _validate_cell(row, col):
            return

        state["current"].append((row, col))
        vertex_idx = len(state["current"])

        mk, = ax.plot(
            col,
            row,
            "o",
            color="#00d2dc",
            markersize=9,
            markeredgewidth=1.5,
            markeredgecolor="white",
            zorder=6,
        )
        state["current_markers"].append(mk)

        lbl = ax.annotate(
            str(vertex_idx),
            xy=(col, row),
            xytext=(col + 5, row + 5),
            color="#00d2dc",
            fontsize=8,
            fontweight="bold",
            zorder=7,
            arrowprops={"arrowstyle": "->", "color": "#00d2dc", "lw": 1.0},
        )
        state["current_labels"].append(lbl)

        _redraw_current_polygon()
        _set_status(
            f"Polygon #{len(state['polygons']) + 1}: {vertex_idx} vertex/vertices",
            "#69f0ae",
        )

    def on_new_polygon(_event):
        if _commit_current_polygon():
            _set_status(
                f"Ready for polygon #{len(state['polygons']) + 1}. Left-click to add vertices",
                "#aaaacc",
            )

    def on_save(_event):
        if state["current"]:
            if len(state["current"]) < 3:
                _set_status(
                    "Current polygon has fewer than 3 vertices. Add points or undo before saving",
                    "#ff6d00",
                )
                return
            _commit_current_polygon()

        if not state["polygons"]:
            _set_status("No polygons to save", "#ff6d00")
            return

        state["confirmed"] = True
        plt.close(fig)

    def on_cancel(_event):
        state["cancelled"] = True
        plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", on_click)
    btn_new.on_clicked(on_new_polygon)
    btn_save.on_clicked(on_save)
    btn_cancel.on_clicked(on_cancel)

    plt.show()

    if state["cancelled"] or not state["confirmed"]:
        return None

    return state["polygons"]
