import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider
import csv
from pathlib import Path
import warnings

# Suppress all warnings
warnings.filterwarnings('ignore')

from .viz import build_vis
from .grid_utils import world_to_grid, grid_to_world


class SpeedCoefficientEditor:
    """
    Interactive editor to assign speed coefficients (0-5) to waypoint segments.
    
    Workflow:
    1. Display map + raceline
    2. Click waypoint 1 → Click waypoint 2 → Selects segment
    3. Use slider to set speed coefficient for that segment
    4. Repeat or save
    """
    
    def __init__(self, occupancy_grid, waypoints_path, map_metadata=None):
        """
        Parameters
        ----------
        occupancy_grid : np.ndarray
            The occupancy grid for visualization
        waypoints_path : str
            Path to waypoints.csv (world coordinates)
        map_metadata : dict, optional
            Map YAML metadata with origin, resolution, etc.
        """
        self.occupancy_grid = occupancy_grid
        self.waypoints_path = Path(waypoints_path)
        self.map_metadata = map_metadata or {}
        
        # Extract transformation parameters from metadata
        self.origin = self.map_metadata.get('origin', [0.0, 0.0, 0.0])[:2]
        self.resolution = self.map_metadata.get('resolution', 0.05)
        self.height = self.occupancy_grid.shape[0]
        
        # Load waypoints (world coordinates)
        self.waypoints_world = []  # [(x, y), ...]
        self.waypoints_grid = []   # [(row, col), ...]
        self.speed_coeffs = []
        self._load_waypoints()
        
        # Editor state
        self.selected_indices = []  # [start_idx, end_idx]
        self.current_segment = None
        self.segment_coeffs = {}  # {(start, end): coeff}
        
        # UI state
        self.fig = None
        self.ax = None
        self.line_raceline = None
        self.line_segment = None
        self.scatter_points = None
        self.scatter_selected = None
        self.slider_coeff = None
        self.result = None
        
    def _load_waypoints(self):
        """Load waypoints and existing speed coefficients from CSV."""
        try:
            with open(self.waypoints_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 2:
                        x, y = float(row[0]), float(row[1])
                        self.waypoints_world.append((x, y))
                        
                        # Convert world to grid for visualization
                        grid_pos = world_to_grid((x, y), self.origin, self.resolution, self.height)
                        self.waypoints_grid.append(grid_pos)
                        
                        # Load existing speed coeff if present (column 3)
                        coeff = float(row[2]) if len(row) > 2 else 1.0
                        self.speed_coeffs.append(coeff)
            
            print(f"[SpeedCoefficientEditor] Loaded {len(self.waypoints_world)} waypoints")
            print(f"  Origin: {self.origin}, Resolution: {self.resolution}, Grid height: {self.height}")
        except Exception as e:
            print(f"[SpeedCoefficientEditor] Error loading waypoints: {e}")
            
    def _save_waypoints(self):
        """Save waypoints with speed coefficients back to CSV (world coords)."""
        try:
            with open(self.waypoints_path, 'w', newline='') as f:
                writer = csv.writer(f)
                for pt_world, coeff in zip(self.waypoints_world, self.speed_coeffs):
                    x, y = pt_world
                    writer.writerow([x, y, coeff])
            print(f"[SpeedCoefficientEditor] Saved {len(self.waypoints_world)} waypoints to {self.waypoints_path}")
            return True
        except Exception as e:
            print(f"[SpeedCoefficientEditor] Error saving waypoints: {e}")
            return False
    
    def edit(self):
        """Open interactive editor. Returns True if saved, False if cancelled."""
        if not self.waypoints_world or len(self.waypoints_world) < 2:
            print("[SpeedCoefficientEditor] Not enough waypoints to edit.")
            return False
        
        # Suppress matplotlib warnings
        plt.rcParams['figure.dpi'] = 100
        
        # Build visualization
        vis = build_vis(self.occupancy_grid)
        
        # Create figure with dark theme
        self.fig, self.ax = plt.subplots(figsize=(16, 10))
        self.fig.patch.set_facecolor("#0d1117")
        self.ax.set_facecolor("#0d1117")
        plt.subplots_adjust(bottom=0.28, left=0.08, right=0.95, top=0.92)
        
        # Display map
        self.ax.imshow(vis, origin="lower", cmap='gray', interpolation='nearest')
        self.ax.tick_params(colors="#666666", labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_edgecolor("#333333")
            spine.set_linewidth(0.5)
        
        # Draw raceline colored by speed coefficient
        waypoints_grid_array = np.array(self.waypoints_grid)
        
        # Color palette: smooth gradient from red (slow) to green (fast)
        colors_palette = [
            '#ff3333',  # 0.0 - red
            '#ff6600',  # 0.5 - orange-red
            '#ffaa00',  # 1.0 - orange
            '#ffdd00',  # 1.5 - yellow
            '#aaee00',  # 2.0 - yellow-green
            '#44dd00',  # 2.5 - lime
            '#00cc00',  # 3.0 - green
        ]
        
        for i in range(len(waypoints_grid_array) - 1):
            col1, row1 = waypoints_grid_array[i, 1], waypoints_grid_array[i, 0]
            col2, row2 = waypoints_grid_array[i+1, 1], waypoints_grid_array[i+1, 0]
            
            # Interpolate color based on speed coefficient
            coeff = (self.speed_coeffs[i] + self.speed_coeffs[i+1]) / 2.0
            coeff = np.clip(coeff, 0, 3.0)
            color_idx = int((coeff / 3.0) * (len(colors_palette) - 1))
            color = colors_palette[color_idx]
            
            self.ax.plot([col1, col2], [row1, row2], color=color, linewidth=4, zorder=2, solid_capstyle='round')
        
        # Scatter for selected waypoints (cyan circles)
        self.scatter_selected = self.ax.scatter(
            [], [], c='#00ffff', s=400, marker='o', edgecolors='white', linewidth=2.5,
            zorder=5
        )
        
        # Scatter for segment highlight (blue squares)
        self.scatter_segment = self.ax.scatter(
            [], [], c='#1f88ff', s=120, marker='s', edgecolors='#00ffff', linewidth=1.5,
            zorder=4, alpha=0.8
        )
        
        # Title
        title_text = (
            "Speed Coefficient Editor\n"
            "① Click waypoint 1  →  ② Click waypoint 2  →  ③ Adjust slider  →  ④ Click 'Apply'\n"
            "Color: Red (slow) → Orange → Yellow → Green (fast)  |  Range: 0.0 – 3.0"
        )
        self.ax.set_title(title_text, color="#e0e0e0", fontsize=11, weight='bold', pad=15)
        
        # Custom legend with manual entries (no label warnings)
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='#ff3333', lw=4, label='Speed 0.0 (slow)', solid_capstyle='round'),
            Line2D([0], [0], color='#ffdd00', lw=4, label='Speed 1.5 (medium)', solid_capstyle='round'),
            Line2D([0], [0], color='#00cc00', lw=4, label='Speed 3.0 (fast)', solid_capstyle='round'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='#00ffff', markersize=8,
                   markeredgecolor='white', markeredgewidth=1.5, label='Selected waypoint', linestyle=''),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='#1f88ff', markersize=5,
                   markeredgecolor='#00ffff', markeredgewidth=1, label='Segment range', linestyle=''),
        ]
        self.ax.legend(handles=legend_elements, loc='upper left', fontsize=8.5,
                      facecolor='#1a1a2e', edgecolor='#444444', framealpha=0.95)
        
        # Slider for speed coefficient (0-3)
        ax_slider = plt.axes([0.15, 0.165, 0.7, 0.035])
        ax_slider.set_facecolor('#1a1a2e')
        self.slider_coeff = Slider(
            ax_slider, 'Speed Multiplier', 0.0, 3.0, valinit=1.0, 
            color='#00ffff', track_color='#333333'
        )
        ax_slider.tick_params(colors='#666666', labelsize=9)
        self.slider_coeff.on_changed(self._on_slider_changed)
        
        # Text display for slider value
        self.text_coeff = self.fig.text(0.88, 0.175, '1.00', fontsize=14, color='#00ffff', 
                                        weight='bold', family='monospace')
        
        # Status text
        self.text_status = self.fig.text(0.15, 0.08, 'Ready: Select two waypoints', 
                                        fontsize=10, color='#aaaaaa', family='monospace')
        
        # Buttons with better styling
        btn_height = 0.045
        btn_spacing = 0.18
        
        ax_apply = plt.axes([0.15, 0.01, btn_spacing - 0.01, btn_height])
        btn_apply = Button(ax_apply, 'Apply', color='#1f88ff', hovercolor='#00ccff')
        btn_apply.label.set_color('#ffffff')
        btn_apply.label.set_fontsize(10)
        btn_apply.on_clicked(lambda e: self._apply_coefficient())
        
        ax_clear = plt.axes([0.15 + btn_spacing, 0.01, btn_spacing - 0.01, btn_height])
        btn_clear = Button(ax_clear, 'Clear', color='#ff8844', hovercolor='#ffaa66')
        btn_clear.label.set_color('#ffffff')
        btn_clear.label.set_fontsize(10)
        btn_clear.on_clicked(lambda e: self._clear_selection())
        
        ax_save = plt.axes([0.15 + btn_spacing * 2, 0.01, btn_spacing - 0.01, btn_height])
        btn_save = Button(ax_save, 'Save & Close', color='#00cc00', hovercolor='#00ff00')
        btn_save.label.set_color('#000000')
        btn_save.label.set_fontsize(10)
        btn_save.on_clicked(lambda e: self._save_and_close())
        
        ax_cancel = plt.axes([0.15 + btn_spacing * 3, 0.01, btn_spacing - 0.01, btn_height])
        btn_cancel = Button(ax_cancel, 'Cancel', color='#cc0000', hovercolor='#ff0000')
        btn_cancel.label.set_color('#ffffff')
        btn_cancel.label.set_fontsize(10)
        btn_cancel.on_clicked(lambda e: self._cancel())
        
        # Connect click event
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        
        plt.show()
        
        return self.result
    
    def _on_click(self, event):
        """Handle waypoint selection - only allow 2 at a time."""
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        
        # Ignore clicks if we already have 2 selected (force user to clear first)
        if len(self.selected_indices) >= 2:
            self.text_status.set_text('⚠ Segment already selected. Click "Clear" first.')
            self.fig.canvas.draw_idle()
            return
        
        # Click is in (col, row) from imshow, convert to grid (row, col)
        col, row = int(event.xdata), int(event.ydata)
        click_grid = np.array([row, col])
        
        # Find nearest waypoint in grid space
        waypoints_grid_array = np.array(self.waypoints_grid)
        distances = np.linalg.norm(waypoints_grid_array - click_grid, axis=1)
        nearest_idx = np.argmin(distances)
        
        # Prevent selecting the same waypoint twice
        if nearest_idx in self.selected_indices:
            self.text_status.set_text('✗ Already selected. Choose a different waypoint.')
            self.fig.canvas.draw_idle()
            return
        
        # Add to selection
        self.selected_indices.append(nearest_idx)
        world_pt = self.waypoints_world[nearest_idx]
        
        # Update display
        self._update_selected_display()
        
        # Update status
        if len(self.selected_indices) == 1:
            self.text_status.set_text(f'✓ Waypoint 1 selected (#{nearest_idx}). Now click waypoint 2.')
        elif len(self.selected_indices) == 2:
            idx1, idx2 = sorted(self.selected_indices)
            self._show_segment(idx1, idx2)
            self.text_status.set_text(f'✓ Segment {idx1}→{idx2} selected ({idx2-idx1+1} waypoints). Adjust slider & click Apply.')
        
        self.fig.canvas.draw_idle()
    
    def _update_selected_display(self):
        """Update visual display of selected waypoints (circles)."""
        if self.selected_indices:
            waypoints_grid_array = np.array(self.waypoints_grid)
            selected_grid = waypoints_grid_array[self.selected_indices]
            self.scatter_selected.set_offsets(selected_grid[:, [1, 0]])  # (col, row)
        else:
            self.scatter_selected.set_offsets(np.empty((0, 2)))
        self.fig.canvas.draw_idle()
    
    def _show_segment(self, idx1, idx2):
        """Highlight selected segment with blue squares."""
        self.current_segment = (idx1, idx2)
        
        # Show segment as blue squares
        segment_grid = np.array(self.waypoints_grid[idx1:idx2+1])
        self.scatter_segment.set_offsets(segment_grid[:, [1, 0]])  # (col, row)
        
        # Set slider to average coeff for segment
        segment_coeffs = self.speed_coeffs[idx1:idx2+1]
        avg_coeff = np.mean(segment_coeffs)
        self.slider_coeff.set_val(avg_coeff)
        self._update_slider_text()
    
    def _on_slider_changed(self, val):
        """Update slider label (real-time)."""
        self._update_slider_text()
    
    def _update_slider_text(self):
        """Update the text display of slider value."""
        val = self.slider_coeff.val
        self.text_coeff.set_text(f'{val:.2f}')
    
    def _apply_coefficient(self):
        """Apply current slider value to selected segment."""
        if self.current_segment is None:
            self.text_status.set_text('⚠ No segment selected. Select two waypoints first.')
            self.fig.canvas.draw_idle()
            return
        
        idx1, idx2 = self.current_segment
        coeff = self.slider_coeff.val
        
        for i in range(idx1, idx2 + 1):
            self.speed_coeffs[i] = coeff
        
        print(f"[SpeedCoefficientEditor] Applied coeff={coeff:.2f} to segment {idx1}→{idx2}")
        
        # Redraw raceline with new colors
        self._redraw_raceline()
        
        # Update status
        self.text_status.set_text(f'✓ Applied {coeff:.2f} to segment {idx1}→{idx2}. Select next segment or Save.')
        
        # Clear selection
        self._clear_selection()
    
    def _redraw_raceline(self):
        """Redraw entire raceline with current speed coefficients."""
        self.ax.clear()
        
        # Re-display map
        vis = build_vis(self.occupancy_grid)
        self.ax.imshow(vis, origin="lower", cmap='gray', interpolation='nearest')
        self.ax.tick_params(colors="#666666", labelsize=8)
        for spine in self.ax.spines.values():
            spine.set_edgecolor("#333333")
            spine.set_linewidth(0.5)
        
        # Redraw raceline
        waypoints_grid_array = np.array(self.waypoints_grid)
        colors_palette = [
            '#ff3333', '#ff6600', '#ffaa00', '#ffdd00', '#aaee00', '#44dd00', '#00cc00'
        ]
        
        for i in range(len(waypoints_grid_array) - 1):
            col1, row1 = waypoints_grid_array[i, 1], waypoints_grid_array[i, 0]
            col2, row2 = waypoints_grid_array[i+1, 1], waypoints_grid_array[i+1, 0]
            
            coeff = (self.speed_coeffs[i] + self.speed_coeffs[i+1]) / 2.0
            coeff = np.clip(coeff, 0, 3.0)
            color_idx = int((coeff / 3.0) * (len(colors_palette) - 1))
            color = colors_palette[color_idx]
            
            self.ax.plot([col1, col2], [row1, row2], color=color, linewidth=4, zorder=2, solid_capstyle='round')
        
        # Recreate scatters
        self.scatter_selected = self.ax.scatter(
            [], [], c='#00ffff', s=400, marker='o', edgecolors='white', linewidth=2.5, zorder=5
        )
        self.scatter_segment = self.ax.scatter(
            [], [], c='#1f88ff', s=350, marker='s', edgecolors='#00ffff', linewidth=2, zorder=4, alpha=0.7
        )
        
        # Title
        title_text = (
            "Speed Coefficient Editor\n"
            "① Click waypoint 1  →  ② Click waypoint 2  →  ③ Adjust slider  →  ④ Click 'Apply'\n"
            "Color: Red (slow) → Orange → Yellow → Green (fast)  |  Range: 0.0 – 3.0"
        )
        self.ax.set_title(title_text, color="#e0e0e0", fontsize=11, weight='bold', pad=15)
        self.ax.legend(loc='upper left', fontsize=9, facecolor='#1a1a2e', edgecolor='#444444')
        
        self.fig.canvas.draw_idle()
    
    def _clear_selection(self):
        """Clear current selection."""
        self.selected_indices = []
        self.current_segment = None
        self.scatter_selected.set_offsets(np.empty((0, 2)))
        self.scatter_segment.set_offsets(np.empty((0, 2)))
        self.slider_coeff.set_val(1.0)
        self._update_slider_text()
        self.text_status.set_text('Ready: Select two waypoints')
        self.fig.canvas.draw_idle()
    
    def _save_and_close(self):
        """Save coefficients and close."""
        if self._save_waypoints():
            print("[SpeedCoefficientEditor] Changes saved successfully.")
            self.result = True
            plt.close(self.fig)
        else:
            print("[SpeedCoefficientEditor] Failed to save.")
    
    def _cancel(self):
        """Close without saving."""
        self.result = False
        plt.close(self.fig)


def edit_speed_coefficients(occupancy_grid, waypoints_path, map_data=None):
    """
    Convenience function to launch speed coefficient editor.
    
    Returns
    -------
    bool
        True if saved, False if cancelled.
    """
    editor = SpeedCoefficientEditor(occupancy_grid, waypoints_path, map_data)
    return editor.edit()
