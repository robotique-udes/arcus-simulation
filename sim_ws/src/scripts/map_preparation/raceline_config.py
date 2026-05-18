from dataclasses import dataclass, field


@dataclass(frozen=True)
class RacelineConfig:
    # Map files
    map_folder: str = "slam_maps"
    forced_map_basename: str = ""

    # Output
    csv_folder: str = "saved/"
    csv_name: str = "waypoints"

    # Default start / finish position (world coordinates in metres)
    start_world: list[float] = field(default_factory=lambda: [0.0, 0.0])

    # A* tuning
    safety_weight: float = 100
    turn_weight: float = 0.0

    # Raceline mode: "astar", "astar_no_min_curvature", "min_curvature" or "manual_spline"
    raceline_mode: str = "astar_no_min_curvature"

    # Manual spline settings
    manual_spline_smoothing: float = 8.0
    manual_spline_points: int = 1200

    # A* smoothing factor
    smoothing_factor: float = 8000.0

    # Optional post-A* drag editor
    enable_drag_edit: bool = True
    drag_handle_stride: int = 5
    drag_smoothing_factor: float = 4000.0
    drag_adaptive_handles: bool = True
    drag_curvature_handle_ratio: float = 0.12
    drag_min_handle_spacing: int = 8
    drag_influence_radius: int = 15
    drag_preview_points: int = 260
    drag_final_preview_points: int = 1200
    drag_max_redraw_hz: float = 24.0

    debug_min_curvature: bool = False
    vehicle_width: float = 0.3  # metres
    min_curv_smoothing_factor: float = 1000.0


DEFAULT_CONFIG = RacelineConfig()