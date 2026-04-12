import cv2
import matplotlib.pyplot as plt
import numpy as np

from .constants import FREE, OCCUPIED, UNKNOWN


def build_vis(occupancy_grid):
    """
    Converts the occupancy grid to an RGB uint8 image for display.
    Paths and markers are drawn separately by the callers via matplotlib.
    """
    vis = np.zeros((*occupancy_grid.shape, 3), dtype=np.uint8)
    vis[occupancy_grid == UNKNOWN] = [60, 60, 80]
    vis[occupancy_grid == FREE] = [230, 230, 245]
    vis[occupancy_grid == OCCUPIED] = [20, 20, 30]
    return vis


def show_grid(occupancy_grid, raw_raceline=None, smooth_raceline=None, waypoints=None):
    """
    Renders the occupancy grid with optional overlays.
    """
    vis = build_vis(occupancy_grid)

    if raw_raceline and len(raw_raceline) >= 2:
        pts = np.array([(c, r) for r, c in raw_raceline], dtype=np.int32)
        cv2.polylines(
            vis,
            [pts.reshape(-1, 1, 2)],
            isClosed=False,
            color=(60, 140, 80),
            thickness=2,
            lineType=cv2.LINE_AA,
        )

    if smooth_raceline and len(smooth_raceline) >= 2:
        pts = np.array(
            [(int(round(c)), int(round(r))) for r, c in smooth_raceline], dtype=np.int32
        )
        cv2.polylines(
            vis,
            [pts.reshape(-1, 1, 2)],
            isClosed=False,
            color=(0, 210, 220),
            thickness=3,
            lineType=cv2.LINE_AA,
        )

    fig, ax = plt.subplots(figsize=(14, 11))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    ax.imshow(vis, origin="lower")
    ax.set_title(
        "Computed Path  (muted green = raw A*  |  cyan = smoothed)",
        color="white",
        fontsize=12,
        pad=14,
    )
    ax.tick_params(colors="#888888")
    for spine in ax.spines.values():
        spine.set_edgecolor("#333355")

    if raw_raceline:
        sr, sc = raw_raceline[0]
        ax.plot(
            sc,
            sr,
            "o",
            color="#00e676",
            markersize=13,
            markeredgewidth=2,
            markeredgecolor="white",
            zorder=6,
            label="Start / Finish",
        )
        ax.annotate(
            "S/F",
            xy=(sc, sr),
            xytext=(sc + 8, sr + 8),
            color="#00e676",
            fontsize=8,
            fontweight="bold",
            zorder=7,
            arrowprops=dict(arrowstyle="->", color="#00e676", lw=1.2),
        )

    if waypoints:
        for i, (wr, wc) in enumerate(waypoints):
            ax.plot(
                wc,
                wr,
                "s",
                color="#ffab00",
                markersize=11,
                markeredgewidth=1.5,
                markeredgecolor="white",
                zorder=6,
            )
            ax.text(
                wc + 5,
                wr + 5,
                str(i + 1),
                color="#ffab00",
                fontsize=8,
                fontweight="bold",
                zorder=7,
            )

    handles = []
    if raw_raceline:
        handles.append(plt.Line2D([0], [0], color="#3c8c50", lw=2, label="Raw A* path"))
    if smooth_raceline:
        handles.append(plt.Line2D([0], [0], color="#00d2dc", lw=2, label="Smoothed path"))
    if handles:
        ax.legend(
            handles=handles,
            loc="upper right",
            framealpha=0.5,
            facecolor="#1a1a2e",
            edgecolor="#444466",
            labelcolor="white",
            fontsize=9,
        )

    ax.grid(False)
    plt.tight_layout()
    plt.show()
