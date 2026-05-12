import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from scipy.ndimage import maximum_filter, binary_closing
from skimage.morphology import skeletonize
import cv2

from utils.planning import brushfire_algo
from .constants import FREE

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