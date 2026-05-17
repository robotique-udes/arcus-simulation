import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider
import csv
from pathlib import Path
import warnings

warnings.filterwarnings('ignore')

from .viz import build_vis
from .grid_utils import world_to_grid, grid_to_world


def _coeff_to_color(c):
    """0.0 → red | 1.0 → orange | 2.0 → green"""
    c = np.clip(c, 0.0, 2.0)
    if c <= 1.0:
        t = c
        r = 1.0
        g = 0.133 + t * (0.533 - 0.133)
        b = 0.133 * (1.0 - t)
    else:
        t = c - 1.0
        r = 1.0 - t
        g = 0.533 + t * (0.867 - 0.533)
        b = 0.0
    return (r, g, b)


def _arc_indices(idx1, idx2, n, flipped):
    """
    Return the list of waypoint indices for one of the two arcs between idx1 and idx2
    on a closed loop of length n.

    flipped=False → the arc that goes directly idx1 … idx2 (shorter if idx2 > idx1)
    flipped=True  → the wrap-around arc idx2 … n-1, 0 … idx1
    """
    a, b = sorted([idx1, idx2])
    if not flipped:
        return list(range(a, b + 1))
    else:
        # wrap: b → n-1 → 0 → a
        return list(range(b, n)) + list(range(0, a + 1))


# ── Layout constants ──────────────────────────────────────────────────────────
#  0.010  buttons    h=0.045  top=0.055
#  0.066  status     (fig.text, single line)
#  0.093  slider     h=0.030  top=0.123
#  0.135  colourbar  h=0.014  top=0.149
#  0.172  ── map axes bottom ──────────────────────────────────────────────────

_BTN_Y  = 0.010;  _BTN_H  = 0.045
_STA_Y  = 0.066
_SLD_Y  = 0.093;  _SLD_H  = 0.030
_CB_Y   = 0.135;  _CB_H   = 0.014
_AX_BOT = 0.172

_LEFT  = 0.08
_RIGHT = 0.95
_W     = _RIGHT - _LEFT


class SpeedCoefficientEditor:
    """
    Interactive editor to assign speed coefficients (0–2) to waypoint segments.

    Two waypoints always define two arcs on a closed loop.  After selecting
    both endpoints the active arc is highlighted in bright blue; the other arc
    is shown dimmed.  Click "Flip arc" to switch which arc is active before
    applying the coefficient.

    Workflow:
    1. Click waypoint A.
    2. Click waypoint B  →  one arc lights up, the other dims.
    3. Click "Flip arc" if you want the other arc.
    4. Adjust slider  →  click Apply.
    5. Repeat or Save & Close.
    """

    def __init__(self, occupancy_grid, waypoints_path, map_metadata=None):
        self.occupancy_grid = occupancy_grid
        self.waypoints_path = Path(waypoints_path)
        self.map_metadata   = map_metadata or {}

        self.origin     = self.map_metadata.get('origin', [0.0, 0.0, 0.0])[:2]
        self.resolution = self.map_metadata.get('resolution', 0.05)
        self.height     = self.occupancy_grid.shape[0]

        self.waypoints_world = []
        self.waypoints_grid  = []
        self.speed_coeffs    = []
        self._load_waypoints()

        self.selected_indices  = []
        self.current_segment   = None   # list of indices in active arc
        self.arc_flipped       = False

        self.fig               = None
        self.ax                = None
        self.scatter_selected  = None
        self.scatter_segment   = None   # active arc — bright blue squares
        self.scatter_other_arc = None   # inactive arc — dim grey squares
        self.slider_coeff      = None
        self.text_coeff        = None
        self.text_status       = None
        self.btn_flip          = None
        self.result            = None

    # ── I/O ──────────────────────────────────────────────────────────────────

    def _load_waypoints(self):
        try:
            with open(self.waypoints_path, 'r') as f:
                for row in csv.reader(f):
                    if len(row) >= 2:
                        x, y = float(row[0]), float(row[1])
                        self.waypoints_world.append((x, y))
                        self.waypoints_grid.append(
                            world_to_grid((x, y), self.origin, self.resolution, self.height)
                        )
                        self.speed_coeffs.append(float(row[2]) if len(row) > 2 else 1.0)
            print(f"[SpeedCoefficientEditor] Loaded {len(self.waypoints_world)} waypoints")
        except Exception as e:
            print(f"[SpeedCoefficientEditor] Error loading waypoints: {e}")

    def _save_waypoints(self):
        try:
            with open(self.waypoints_path, 'w', newline='') as f:
                w = csv.writer(f)
                for (x, y), c in zip(self.waypoints_world, self.speed_coeffs):
                    w.writerow([x, y, c])
            print(f"[SpeedCoefficientEditor] Saved to {self.waypoints_path}")
            return True
        except Exception as e:
            print(f"[SpeedCoefficientEditor] Save error: {e}")
            return False

    # ── Main entry point ──────────────────────────────────────────────────────

    def edit(self):
        """Open the editor. Returns True if saved, False if cancelled."""
        if len(self.waypoints_world) < 2:
            print("[SpeedCoefficientEditor] Not enough waypoints.")
            return False

        plt.rcParams['figure.dpi'] = 100
        vis = build_vis(self.occupancy_grid)

        self.fig, self.ax = plt.subplots(figsize=(14, 11))
        self.fig.patch.set_facecolor("#1a1a2e")
        self.ax.set_facecolor("#1a1a2e")
        plt.subplots_adjust(bottom=_AX_BOT, left=_LEFT, right=_RIGHT, top=0.92)

        self.ax.imshow(vis, origin="lower", interpolation='nearest')
        self.ax.tick_params(colors="#888888", labelsize=8)
        for sp in self.ax.spines.values():
            sp.set_edgecolor("#333355")
            sp.set_linewidth(0.5)

        self._draw_raceline()

        # Selected endpoint markers
        self.scatter_selected = self.ax.scatter(
            [], [], c='#00ffff', s=280, marker='o',
            edgecolors='white', linewidth=2.0, zorder=6
        )
        # Active arc — bright blue squares
        self.scatter_segment = self.ax.scatter(
            [], [], c='#1f88ff', s=28, marker='s',
            edgecolors='#00ccff', linewidth=0.6,
            zorder=4, alpha=0.85
        )
        # Inactive arc — dim, so user can see which arc they're NOT editing
        self.scatter_other_arc = self.ax.scatter(
            [], [], c='#3a3a5a', s=14, marker='s',
            edgecolors='none', linewidth=0,
            zorder=3, alpha=0.6
        )

        self.ax.set_title(
            "Speed Coefficient Editor\n"
            "① Click waypoint 1  →  ② Click waypoint 2  →  ③ Flip arc if needed  →  ④ Adjust & Apply",
            color="white", fontsize=11, weight='bold', pad=10
        )

        # ── Colour bar ────────────────────────────────────────────────────────
        ax_cb     = self.fig.add_axes([_LEFT, _CB_Y, _W, _CB_H])
        cb_vals   = np.linspace(0, 2, 256)
        cb_colors = np.array([_coeff_to_color(v) for v in cb_vals])
        ax_cb.imshow(cb_colors.reshape(1, 256, 3), aspect='auto', origin='lower')
        ax_cb.set_xticks([0, 64, 128, 192, 255])
        ax_cb.set_xticklabels(
            ['0.0  slow', '0.5', '1.0  normal', '1.5', '2.0  fast'],
            color='#aaaacc', fontsize=8
        )
        ax_cb.set_yticks([])
        for sp in ax_cb.spines.values():
            sp.set_edgecolor("#333355")

        # ── Slider + readout ──────────────────────────────────────────────────
        ax_sld = self.fig.add_axes([_LEFT, _SLD_Y, _W * 0.74, _SLD_H],
                                    facecolor='#1f1f38')
        self.slider_coeff = Slider(
            ax_sld, 'Speed ×', 0.0, 2.0, valinit=1.0,
            color='#00d2dc', track_color='#333355'
        )
        self.slider_coeff.label.set_color('white')
        self.slider_coeff.label.set_fontsize(10)
        self.slider_coeff.valtext.set_visible(False)
        self.slider_coeff.on_changed(self._on_slider_changed)

        self.text_coeff = self.fig.text(
            _LEFT + _W * 0.77,
            _SLD_Y + _SLD_H * 0.5,
            '1.00',
            fontsize=13, color='#00d2dc', weight='bold',
            family='monospace', va='center', ha='left'
        )

        # ── Status text ───────────────────────────────────────────────────────
        self.text_status = self.fig.text(
            0.5, _STA_Y,
            'Ready — click two waypoints to select a segment',
            fontsize=9, color='#aaaacc', family='monospace',
            ha='center', va='center'
        )

        # ── Buttons ───────────────────────────────────────────────────────────
        # Apply | Clear | [Flip arc]          Save & Close | Cancel
        bw  = 0.11
        gap = 0.010

        def _btn(x, w, label, bg, hover, fg='white'):
            a = self.fig.add_axes([x, _BTN_Y, w, _BTN_H])
            b = Button(a, label, color=bg, hovercolor=hover)
            b.label.set_color(fg)
            b.label.set_fontsize(10)
            return b

        btn_apply  = _btn(_LEFT,                   bw,     'Apply',        '#22224a', '#3333aa')
        btn_clear  = _btn(_LEFT + bw + gap,        bw,     'Clear',        '#24354a', '#34506e')
        self.btn_flip = _btn(_LEFT + 2*(bw+gap),   bw,     'Flip arc',     '#2a1a4a', '#4a2a7a')
        btn_save   = _btn(_RIGHT - 2*bw - gap,     bw,     'Save & Close', '#1a3a1a', '#2a5a2a', fg='#00e676')
        btn_cancel = _btn(_RIGHT - bw,             bw,     'Cancel',       '#3a1f1f', '#5a2a2a')

        btn_apply    .on_clicked(lambda e: self._apply_coefficient())
        btn_clear    .on_clicked(lambda e: self._clear_selection())
        self.btn_flip.on_clicked(lambda e: self._flip_arc())
        btn_save     .on_clicked(lambda e: self._save_and_close())
        btn_cancel   .on_clicked(lambda e: self._cancel())

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        plt.show()
        return self.result

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _draw_raceline(self):
        wg = np.array(self.waypoints_grid)
        for i in range(len(wg) - 1):
            col1, row1 = wg[i,   1], wg[i,   0]
            col2, row2 = wg[i+1, 1], wg[i+1, 0]
            coeff = (self.speed_coeffs[i] + self.speed_coeffs[i+1]) / 2.0
            self.ax.plot([col1, col2], [row1, row2],
                         color=_coeff_to_color(coeff),
                         linewidth=4, zorder=2, solid_capstyle='round')

    def _redraw_raceline(self):
        self.ax.clear()
        self.ax.imshow(build_vis(self.occupancy_grid), origin="lower", interpolation='nearest')
        self.ax.tick_params(colors="#888888", labelsize=8)
        for sp in self.ax.spines.values():
            sp.set_edgecolor("#333355")
            sp.set_linewidth(0.5)
        self._draw_raceline()

        self.scatter_selected = self.ax.scatter(
            [], [], c='#00ffff', s=280, marker='o',
            edgecolors='white', linewidth=2.0, zorder=6
        )
        self.scatter_segment = self.ax.scatter(
            [], [], c='#1f88ff', s=28, marker='s',
            edgecolors='#00ccff', linewidth=0.6,
            zorder=4, alpha=0.85
        )
        self.scatter_other_arc = self.ax.scatter(
            [], [], c='#3a3a5a', s=14, marker='s',
            edgecolors='none', linewidth=0,
            zorder=3, alpha=0.6
        )
        self.ax.set_title(
            "Speed Coefficient Editor\n"
            "① Click waypoint 1  →  ② Click waypoint 2  →  ③ Flip arc if needed  →  ④ Adjust & Apply",
            color="white", fontsize=11, weight='bold', pad=10
        )
        self.fig.canvas.draw_idle()

    # ── Arc helpers ───────────────────────────────────────────────────────────

    def _both_arcs(self):
        """Return (active_indices, inactive_indices) given current selection and flip state."""
        n    = len(self.waypoints_world)
        a, b = sorted(self.selected_indices)
        arc_a = list(range(a, b + 1))                          # direct arc
        arc_b = list(range(b, n)) + list(range(0, a + 1))     # wrap-around arc
        if not self.arc_flipped:
            return arc_a, arc_b
        else:
            return arc_b, arc_a

    def _update_arc_display(self):
        active_idx, other_idx = self._both_arcs()
        wg = np.array(self.waypoints_grid)

        # Active arc
        active_pts = wg[active_idx]
        self.scatter_segment.set_offsets(active_pts[:, [1, 0]])

        # Inactive arc (dim)
        other_pts = wg[other_idx]
        self.scatter_other_arc.set_offsets(other_pts[:, [1, 0]])

        # Slider: average of active arc
        avg = float(np.mean([self.speed_coeffs[i] for i in active_idx]))
        self.slider_coeff.set_val(avg)
        self._update_coeff_text()

        n_active = len(active_idx)
        n_total  = len(self.waypoints_world)
        self._set_status(
            f'✓ Arc: {active_idx[0]} → {active_idx[-1]}  ({n_active}/{n_total} pts)'
            f'  |  Use "Flip arc" to switch sides  |  Adjust slider & Apply.',
            color='#aaaaff'
        )
        self.fig.canvas.draw_idle()

    # ── Event handlers ────────────────────────────────────────────────────────

    def _on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return

        if len(self.selected_indices) >= 2:
            self._set_status('⚠  Segment selected — click Clear to start over, or Flip arc to switch.')
            self.fig.canvas.draw_idle()
            return

        col, row    = int(event.xdata), int(event.ydata)
        wg          = np.array(self.waypoints_grid)
        nearest_idx = int(np.argmin(np.linalg.norm(wg - np.array([row, col]), axis=1)))

        if nearest_idx in self.selected_indices:
            self._set_status('✗ Already selected — choose a different waypoint.')
            self.fig.canvas.draw_idle()
            return

        self.selected_indices.append(nearest_idx)
        self._update_selected_display()

        if len(self.selected_indices) == 1:
            self._set_status(f'✓ Waypoint #{nearest_idx} selected — now click waypoint 2.')
        else:
            self.arc_flipped = False
            self._update_arc_display()

        self.fig.canvas.draw_idle()

    def _on_slider_changed(self, _val):
        self._update_coeff_text()

    # ── UI helpers ────────────────────────────────────────────────────────────

    def _set_status(self, msg, color='#aaaacc'):
        self.text_status.set_text(msg)
        self.text_status.set_color(color)

    def _update_coeff_text(self):
        self.text_coeff.set_text(f'{self.slider_coeff.val:.2f}')

    def _update_selected_display(self):
        if self.selected_indices:
            wg  = np.array(self.waypoints_grid)
            sel = wg[self.selected_indices]
            self.scatter_selected.set_offsets(sel[:, [1, 0]])
        else:
            self.scatter_selected.set_offsets(np.empty((0, 2)))
        self.fig.canvas.draw_idle()

    # ── Actions ───────────────────────────────────────────────────────────────

    def _flip_arc(self):
        """Toggle between the two arcs defined by the two selected endpoints."""
        if len(self.selected_indices) < 2:
            self._set_status('⚠  Select two waypoints first, then flip.')
            self.fig.canvas.draw_idle()
            return
        self.arc_flipped = not self.arc_flipped
        self._update_arc_display()

    def _apply_coefficient(self):
        if len(self.selected_indices) < 2:
            self._set_status('⚠  No segment selected — click two waypoints first.')
            self.fig.canvas.draw_idle()
            return

        active_idx, _ = self._both_arcs()
        coeff = self.slider_coeff.val
        for i in active_idx:
            self.speed_coeffs[i] = coeff
        print(f"[SpeedCoefficientEditor] Applied ×{coeff:.2f} to {len(active_idx)} waypoints")

        self._redraw_raceline()
        self._set_status(
            f'✓ Applied ×{coeff:.2f} to {len(active_idx)} pts.  Select next segment or Save.',
            color='#69f0ae'
        )
        self._clear_selection()

    def _clear_selection(self):
        self.selected_indices = []
        self.current_segment  = None
        self.arc_flipped      = False
        self.scatter_selected .set_offsets(np.empty((0, 2)))
        self.scatter_segment  .set_offsets(np.empty((0, 2)))
        self.scatter_other_arc.set_offsets(np.empty((0, 2)))
        self.slider_coeff.set_val(1.0)
        self._update_coeff_text()
        self._set_status('Ready — click two waypoints to select a segment')
        self.fig.canvas.draw_idle()

    def _save_and_close(self):
        if self._save_waypoints():
            self.result = True
            plt.close(self.fig)

    def _cancel(self):
        self.result = False
        plt.close(self.fig)


# ── Public API ────────────────────────────────────────────────────────────────

def edit_speed_coefficients(occupancy_grid, waypoints_path, map_data=None):
    """Launch the speed coefficient editor. Returns True if saved, False if cancelled."""
    return SpeedCoefficientEditor(occupancy_grid, waypoints_path, map_data).edit()