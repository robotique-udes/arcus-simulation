import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from scipy.ndimage import maximum_filter, binary_closing
from scipy.spatial import KDTree
from skimage.morphology import skeletonize
import cv2

from utils.planning import brushfire_algo
from .constants import FREE
from .viz import build_vis

def tune_centerline(occupancy_grid):
    mask = (occupancy_grid == FREE)
    dist = brushfire_algo(occupancy_grid)

    fig, ax = plt.subplots(figsize=(12, 10))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    plt.subplots_adjust(bottom=0.25)

    ax.imshow(occupancy_grid, cmap="gray", vmin=0, vmax=100, origin="upper")
    ax.set_title("Centerline Tuning", color="white")

    scatter = ax.scatter([], [], c="red", s=2)

    # ---------------- STATE ----------------
    state = {
        "points": [],
        "confirmed": False,
        "cancelled": False,
    }

    # ---------------- SLIDERS ----------------
    ax_tol = plt.axes([0.15, 0.15, 0.65, 0.03], facecolor="#1f1f38")
    ax_clear = plt.axes([0.15, 0.10, 0.65, 0.03], facecolor="#1f1f38")
    ax_close = plt.axes([0.15, 0.05, 0.65, 0.03], facecolor="#1f1f38")

    s_tol = Slider(ax_tol, "RIDGE_TOL", 0.0, 3.0, valinit=0.5)
    s_clear = Slider(ax_clear, "MIN_CLEAR", 0, 50, valinit=20, valstep=1)
    s_close = Slider(ax_close, "CLOSING", 1, 7, valinit=3, valstep=0.1)

    for s in [s_tol, s_clear, s_close]:
        s.label.set_color("white")
        s.valtext.set_color("white")

    # ---------------- BUTTONS ----------------
    ax_conf = plt.axes([0.60, 0.01, 0.18, 0.045])
    btn_conf = Button(ax_conf, "Confirm centerline", color="#22224a", hovercolor="#3333aa")
    btn_conf.label.set_color("white")

    ax_cancel = plt.axes([0.80, 0.01, 0.15, 0.045])
    btn_cancel = Button(ax_cancel, "Cancel", color="#3a1f1f", hovercolor="#5a2a2a")
    btn_cancel.label.set_color("white")

    # ---------------- CORE PIPELINE ----------------
    def compute():
        tol = s_tol.val
        clear = s_clear.val
        close_size = int(s_close.val)

        neighborhood = maximum_filter(dist, size=3)
        ridge = (dist >= (neighborhood - tol)) & mask
        ridge &= (dist > clear)

        ridge = binary_closing(ridge, structure=np.ones((close_size, close_size)))

        skeleton = skeletonize(ridge)
        skeleton = keep_largest_component(skeleton)
        skeleton = prune_skeleton(skeleton, iterations=20)

        pts = np.column_stack(np.nonzero(skeleton))
        return pts

    # ---------------- UPDATE ----------------
    def update(_=None):
        pts = compute()
        state["points"] = pts

        if len(pts) > 0:
            scatter.set_offsets(np.c_[pts[:, 1], pts[:, 0]])
        else:
            scatter.set_offsets([])

        fig.canvas.draw_idle()

    # ---------------- BUTTON CALLBACKS ----------------
    def on_confirm(_):
        if len(state["points"]) == 0:
            print("No centerline to confirm.")
            return
        state["confirmed"] = True
        plt.close(fig)

    def on_cancel(_):
        state["cancelled"] = True
        plt.close(fig)

    # ---------------- HOOKS ----------------
    s_tol.on_changed(update)
    s_clear.on_changed(update)
    s_close.on_changed(update)

    btn_conf.on_clicked(on_confirm)
    btn_cancel.on_clicked(on_cancel)

    # initial draw
    update()

    plt.show()

    # ---------------- RETURN ----------------
    if state["cancelled"]:
        return None

    if state["confirmed"]:
        pts = state["points"]
        return [tuple(p) for p in pts]

    return None


def pick_centerline_route(occupancy_grid, centerline):
    """
    Lets the user pick ordered route anchors on top of the tuned centerline.

    The returned anchors are later snapped to the nearest skeleton pixels and
    stitched into a closed route through the centerline graph.
    """
    if centerline is None or len(centerline) < 2:
        return None

    vis = build_vis(occupancy_grid)
    center_pts = np.array(centerline, dtype=float)
    tree = KDTree(center_pts)

    fig, ax = plt.subplots(figsize=(14, 11))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    plt.subplots_adjust(bottom=0.13)

    ax.imshow(vis, origin="upper")
    ax.scatter(
        center_pts[:, 1],
        center_pts[:, 0],
        c="#4cc3ff",
        s=5,
        alpha=0.55,
        zorder=2,
    )
    ax.tick_params(colors="#888888")
    for spine in ax.spines.values():
        spine.set_edgecolor("#333355")

    ax.set_title(
        "Route selection: LEFT-CLICK anchors on the centerline  |  RIGHT-CLICK undo  |  Confirm to use anchors\n"
        "Use Auto Ordering skips manual route selection",
        color="white",
        fontsize=11,
        pad=12,
    )

    status_text = ax.text(
        0.5,
        -0.055,
        "Click near the desired skeleton branch to place the first route anchor",
        transform=ax.transAxes,
        ha="center",
        va="top",
        color="#aaaacc",
        fontsize=10,
    )

    ax_confirm = plt.axes([0.58, 0.02, 0.16, 0.055])
    btn_confirm = Button(ax_confirm, "Confirm route", color="#22224a", hovercolor="#3333aa")
    btn_confirm.label.set_color("white")
    btn_confirm.label.set_fontsize(10)

    ax_auto = plt.axes([0.76, 0.02, 0.15, 0.055])
    btn_auto = Button(ax_auto, "Use Auto", color="#224222", hovercolor="#2b5f2b")
    btn_auto.label.set_color("white")
    btn_auto.label.set_fontsize(10)

    state = {
        "anchors": [],
        "markers": [],
        "labels": [],
        "confirmed": False,
        "use_auto": False,
    }

    def _update_status(message, color="#aaaacc"):
        status_text.set_text(message)
        status_text.set_color(color)
        fig.canvas.draw_idle()

    def _remove_last_anchor():
        if not state["anchors"]:
            return
        state["anchors"].pop()
        state["markers"].pop().remove()
        state["labels"].pop().remove()

    def on_click(event):
        if event.inaxes is not ax or event.xdata is None or event.ydata is None:
            return

        if event.button == 3:
            _remove_last_anchor()
            if state["anchors"]:
                _update_status(f"{len(state['anchors'])} route anchor(s) placed", "#69f0ae")
            else:
                _update_status("Click near the desired skeleton branch to place the first route anchor")
            return

        if event.button != 1:
            return

        col = float(event.xdata)
        row = float(event.ydata)

        dist, idx = tree.query(np.array([row, col], dtype=float))
        if dist > 10.0:
            _update_status("Click closer to the centerline skeleton", "#ff6d00")
            return

        snapped = tuple(center_pts[int(idx)])
        if state["anchors"] and snapped == state["anchors"][-1]:
            return

        state["anchors"].append(snapped)
        n = len(state["anchors"])

        mk, = ax.plot(
            snapped[1],
            snapped[0],
            "o",
            color="#ffab00",
            markersize=10,
            markeredgewidth=2,
            markeredgecolor="white",
            zorder=6,
        )
        state["markers"].append(mk)

        lb = ax.annotate(
            str(n),
            xy=(snapped[1], snapped[0]),
            xytext=(snapped[1] + 8, snapped[0] + 8),
            color="#ffab00",
            fontsize=8,
            fontweight="bold",
            zorder=7,
            arrowprops=dict(arrowstyle="->", color="#ffab00", lw=1.2),
        )
        state["labels"].append(lb)
        _update_status(f"{n} route anchor(s) placed", "#69f0ae")

    def on_confirm(_event):
        if len(state["anchors"]) < 2:
            _update_status("Place at least two anchors or choose Use Auto", "#ff6d00")
            return
        state["confirmed"] = True
        plt.close(fig)

    def on_auto(_event):
        state["use_auto"] = True
        plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", on_click)
    btn_confirm.on_clicked(on_confirm)
    btn_auto.on_clicked(on_auto)

    plt.show()

    if state["confirmed"]:
        return state["anchors"]

    if state["use_auto"]:
        return None

    return None

def keep_largest_component(binary):
    binary = binary.astype(np.uint8)
    num_labels, labels = cv2.connectedComponents(binary)

    if num_labels <= 1:
        return binary.astype(bool)

    sizes = np.bincount(labels.flatten())
    sizes[0] = 0

    largest = sizes.argmax()
    return (labels == largest)

def prune_skeleton(skeleton, iterations=10):
    """Remove spur branches by iteratively deleting endpoint pixels."""
    skel = skeleton.copy()
    for _ in range(iterations):
        # Count 8-connected neighbors for each skeleton pixel
        neighbors = sum(
            np.roll(np.roll(skel, i, 0), j, 1)
            for i in (-1, 0, 1) for j in (-1, 0, 1)
            if (i != 0 or j != 0)
        )
        # An endpoint has exactly 1 neighbor
        endpoints = skel & (neighbors == 1)
        skel[endpoints] = False
    return skel