"""
Authors: Ramtin B. Meidani, Charles McCabe
Refactored into modular files.

Workflow
--------
1. Open an interactive window to place waypoints, or use manual spline mode.
2. Plan a full loop raceline.
3. Smooth and export (x, y) CSV in metres.
"""

import os

from raceline.grid_utils import grid_generator, grid_to_world, world_to_grid
from raceline.map_io import (
    find_latest_csv,
    find_latest_map_pair,
    load_csv_xy,
    pgm_opener,
    save_csv,
    yaml_opener,
)
from raceline.min_curvature import (
    build_reftrack,
    compute_normal_vectors,
    compute_track_widths,
    grid_path_to_world,
    generate_spline_matrix,
    opt_min_curv,
)
from raceline.planning import brushfire_algo, plan_full_path
from raceline.ui_raceline_editor import edit_raceline_with_drag
from raceline_config import DEFAULT_CONFIG
from raceline.smoothing import smooth_path
from raceline.ui_spline import pick_spline_control_points
from raceline.ui_waypoints import pick_waypoints
from raceline.viz import show_grid


def run():
    cfg = DEFAULT_CONFIG

    if not os.path.isdir(cfg.map_folder):
        raise FileNotFoundError(f"Map folder not found: {cfg.map_folder}")

    if cfg.forced_map_basename.strip():
        map_base = cfg.forced_map_basename.strip()
        yaml_file = map_base + ".yaml"
        pgm_file = map_base + ".pgm"

        full_yaml_path = os.path.join(cfg.map_folder, yaml_file)
        full_pgm_path = os.path.join(cfg.map_folder, pgm_file)

        if not os.path.isfile(full_yaml_path):
            raise FileNotFoundError(f"Forced YAML file not found: {full_yaml_path}")
        if not os.path.isfile(full_pgm_path):
            raise FileNotFoundError(f"Forced PGM file not found: {full_pgm_path}")

        print(f"Using forced map pair from '{cfg.map_folder}':")
        print(f"  YAML: {yaml_file}")
        print(f"  PGM : {pgm_file}")
    else:
        yaml_file, pgm_file = find_latest_map_pair(cfg.map_folder)
        print(f"Using latest map pair from '{cfg.map_folder}':")
        print(f"  YAML: {yaml_file}")
        print(f"  PGM : {pgm_file}")

    # Load map
    full_yaml_path = os.path.join(cfg.map_folder, yaml_file)
    full_pgm_path = os.path.join(cfg.map_folder, pgm_file)

    loaded_yaml = yaml_opener(full_yaml_path)
    loaded_img = pgm_opener(full_pgm_path)
    occupancy_grid = grid_generator(loaded_yaml, loaded_img)

    height, _ = occupancy_grid.shape
    origin = loaded_yaml["origin"]
    res = loaded_yaml["resolution"]

    start_pos = world_to_grid(cfg.start_world, origin, res, height)
    print(f"Default start position (grid): row={start_pos[0]}, col={start_pos[1]}")

    if cfg.raceline_mode == "astar":
        print("\nOpening map window.")
        print("  Left-click       : place intermediate checkpoints around the track")
        print("  Right-click      : undo the last checkpoint")
        print("  Move Start/Finish: relocate the S/F marker")
        print("  Confirm          : accept and proceed\n")

        result = pick_waypoints(occupancy_grid, start_pos)

        if not result:
            print("No checkpoints selected. Exiting.")
            return

        if isinstance(result, dict) and result.get("action") == "import_last":
            latest_csv = find_latest_csv(cfg.csv_folder)
            print(f"\nImporting latest saved raceline: {latest_csv}")
            imported_world = load_csv_xy(latest_csv)

            imported_grid = [
                world_to_grid((x, y), origin, res, height) for x, y in imported_world
            ]

            smooth_raceline = imported_grid
            raw_raceline = imported_grid
            waypoints = []

            if cfg.enable_drag_edit:
                print("Opening drag editor for imported raceline...")
                edited = edit_raceline_with_drag(
                    occupancy_grid,
                    smooth_raceline,
                    handle_stride=cfg.drag_handle_stride,
                    smoothing_factor=cfg.drag_smoothing_factor,
                    lock_start=True,
                    adaptive_handles=cfg.drag_adaptive_handles,
                    curvature_handle_ratio=cfg.drag_curvature_handle_ratio,
                    min_handle_spacing=cfg.drag_min_handle_spacing,
                    drag_influence_radius=cfg.drag_influence_radius,
                    drag_preview_points=cfg.drag_preview_points,
                    final_preview_points=cfg.drag_final_preview_points,
                    max_redraw_hz=cfg.drag_max_redraw_hz,
                )
                if edited is not None:
                    smooth_raceline = edited
                    print(f"Edited imported raceline confirmed: {len(smooth_raceline)} waypoints")
                else:
                    print("Drag edit cancelled. Keeping imported raceline.")

            start_pos = smooth_raceline[0]
        else:
            waypoints, start_pos = result

            sx, sy = grid_to_world(start_pos, origin, res, height)
            print(
                f"\nStart/Finish (grid): row={start_pos[0]}, col={start_pos[1]}  world=({sx:.2f} m, {sy:.2f} m)"
            )
            print(f"{len(waypoints)} checkpoint(s) confirmed (loop closes back to start):")
            for i, wp in enumerate(waypoints):
                wx, wy = grid_to_world(wp, origin, res, height)
                print(
                    f"  [checkpoint {i + 1}]  grid=({wp[0]}, {wp[1]})  world=({wx:.2f} m, {wy:.2f} m)"
                )

            print("\nRunning brushfire... (may take a moment on large maps)")
            brushfire_grid = brushfire_algo(occupancy_grid)

            print("\nRunning A* across all segments...")
            raw_raceline = plan_full_path(
                occupancy_grid,
                brushfire_grid,
                cfg.safety_weight,
                cfg.turn_weight,
                start_pos,
                waypoints,
            )

            if raw_raceline is None:
                print("\nNo complete path found. Try repositioning a checkpoint near a narrow gap.")
                return

            print(f"\nRaw raceline: {len(raw_raceline)} waypoints")

            smooth_raceline = smooth_path(
                raw_raceline,
                smoothing_factor=cfg.smoothing_factor,
                num_points=len(raw_raceline),
                closed=True,
            )
            print(f"Smoothed raceline: {len(smooth_raceline)} waypoints")

            if cfg.enable_drag_edit:
                print("\nOpening drag editor for post-A* raceline tuning...")
                edited = edit_raceline_with_drag(
                    occupancy_grid,
                    smooth_raceline,
                    handle_stride=cfg.drag_handle_stride,
                    smoothing_factor=cfg.drag_smoothing_factor,
                    lock_start=True,
                    adaptive_handles=cfg.drag_adaptive_handles,
                    curvature_handle_ratio=cfg.drag_curvature_handle_ratio,
                    min_handle_spacing=cfg.drag_min_handle_spacing,
                    drag_influence_radius=cfg.drag_influence_radius,
                    drag_preview_points=cfg.drag_preview_points,
                    final_preview_points=cfg.drag_final_preview_points,
                    max_redraw_hz=cfg.drag_max_redraw_hz,
                )
                if edited is not None:
                    smooth_raceline = edited
                    print(f"Edited raceline confirmed: {len(smooth_raceline)} waypoints")
                else:
                    print("Drag edit cancelled. Keeping pre-edit smoothed raceline.")

        # --------------------------------------------------------------
        # Minimum-curvature optimisation
        # --------------------------------------------------------------

        world_xy = grid_path_to_world(smooth_raceline, origin, res, height)
        n        = world_xy.shape[0]
        smooth_raceline_trimmed = smooth_raceline[:n]

        normvectors = compute_normal_vectors(world_xy)

        w_left, w_right = compute_track_widths(
            smooth_raceline_trimmed,
            normvectors,
            occupancy_grid,
            res,
            max_width=cfg.track_max_width,
        )

        reftrack = build_reftrack(world_xy, w_left, w_right)

        A = generate_spline_matrix(world_xy)

        alpha_mincurv, curv_max = opt_min_curv(
            reftrack=reftrack,
            normvectors=normvectors,
            A=A,
            kappa_bound=cfg.kappa_bound,
            w_veh=cfg.vehicle_width,
            plot_debug=True,
        )

        print(f"\nOptimisation complete. Max curvature: {curv_max:.4f} m^-1")

        optimized_xy = world_xy + alpha_mincurv[:, None] * normvectors

        smooth_world = optimized_xy.tolist()

    # ------------------------------------------------------------------
    # Mode: manual spline
    # ------------------------------------------------------------------
    elif cfg.raceline_mode == "manual_spline":
        print("\nOpening manual spline window.")
        print("  Left-click  : add control points around the track")
        print("  Right-click : undo last control point")
        print("  Confirm     : fit and accept closed spline\n")

        control_points = pick_spline_control_points(occupancy_grid, start_pos)
        if not control_points:
            print("No control points confirmed. Exiting.")
            return

        start_pos = control_points[0]
        raw_raceline = control_points
        smooth_raceline = smooth_path(
            control_points,
            smoothing_factor=cfg.manual_spline_smoothing,
            num_points=cfg.manual_spline_points,
            closed=True,
        )

        print(f"Manual control points: {len(control_points)}")
        print(f"Smoothed raceline: {len(smooth_raceline)} waypoints")
        waypoints = control_points[1:]
    else:
        raise ValueError(
            f"Unknown RACELINE_MODE '{cfg.raceline_mode}'. Use 'astar' or 'manual_spline'."
        )

    path_csv = os.path.join(cfg.csv_folder, cfg.csv_name)
    os.makedirs(cfg.csv_folder, exist_ok=True)

    smooth_world = [grid_to_world(pt, origin, res, height) for pt in smooth_raceline]
    save_csv(smooth_world, path_csv)

    print(f"\nSmoothed path saved -> {path_csv}.csv  ({len(smooth_world)} waypoints, metres)")
    print(f"  Start : x={smooth_world[0][0]:.3f} m,  y={smooth_world[0][1]:.3f} m")
    print(f"  End   : x={smooth_world[-1][0]:.3f} m,  y={smooth_world[-1][1]:.3f} m")

    show_grid(
        occupancy_grid,
        raw_raceline=raw_raceline,
        smooth_raceline=smooth_raceline,
        waypoints=waypoints,
    )


if __name__ == "__main__":
    run()
