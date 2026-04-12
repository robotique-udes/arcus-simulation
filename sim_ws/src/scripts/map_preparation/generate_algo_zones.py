"""
Interactive algorithm-zone drawer for occupancy maps.

Workflow
--------
1. Load latest (or forced) map YAML/PGM pair.
2. Draw one or more polygons directly on the map.
3. Assign either 'disparity' or 'pure_pursuit' to each polygon.
4. Export polygon vertices to CSV in world coordinates (metres).
"""

import csv
import os

from utils.grid_utils import grid_generator, grid_to_world
from utils.map_io import find_latest_map_pair, pgm_opener, yaml_opener
from utils.ui_algo_specifier import pick_algo_zones


def save_algo_zones_csv(polygons_grid, origin, resolution, height, output_no_ext):
    """
    Save polygons to CSV using world-space coordinates in metres.

    CSV schema:
        polygon_id,vertex_index,x,y,algorithm
    """
    os.makedirs(os.path.dirname(output_no_ext), exist_ok=True)
    csv_path = output_no_ext + ".csv"

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f, delimiter=",")
        writer.writerow(["polygon_id", "vertex_index", "x", "y", "algorithm"])

        for p_idx, polygon in enumerate(polygons_grid, start=1):
            points = polygon.get("points", []) if isinstance(polygon, dict) else polygon
            algorithm = polygon.get("algorithm", "") if isinstance(polygon, dict) else ""

            for v_idx, (row, col) in enumerate(points, start=1):
                x, y = grid_to_world((row, col), origin, resolution, height)
                writer.writerow([p_idx, v_idx, x, y, algorithm])


def run():
    # Map files
    map_folder: str = "slam_maps"
    forced_map_basename: str = ""

    # Output
    csv_folder: str = "saved/"
    csv_name: str = "algos"

    if not os.path.isdir(map_folder):
        raise FileNotFoundError(f"Map folder not found: {map_folder}")

    if forced_map_basename.strip():
        map_base = forced_map_basename.strip()
        yaml_file = map_base + ".yaml"
        pgm_file = map_base + ".pgm"

        full_yaml_path = os.path.join(map_folder, yaml_file)
        full_pgm_path = os.path.join(map_folder, pgm_file)

        if not os.path.isfile(full_yaml_path):
            raise FileNotFoundError(f"Forced YAML file not found: {full_yaml_path}")
        if not os.path.isfile(full_pgm_path):
            raise FileNotFoundError(f"Forced PGM file not found: {full_pgm_path}")

        print(f"Using forced map pair from '{map_folder}':")
        print(f"  YAML: {yaml_file}")
        print(f"  PGM : {pgm_file}")
    else:
        yaml_file, pgm_file = find_latest_map_pair(map_folder)
        print(f"Using latest map pair from '{map_folder}':")
        print(f"  YAML: {yaml_file}")
        print(f"  PGM : {pgm_file}")

    full_yaml_path = os.path.join(map_folder, yaml_file)
    full_pgm_path = os.path.join(map_folder, pgm_file)

    loaded_yaml = yaml_opener(full_yaml_path)
    loaded_img = pgm_opener(full_pgm_path)
    occupancy_grid = grid_generator(loaded_yaml, loaded_img)

    print("\nOpening algorithm-zone editor...")
    polygons = pick_algo_zones(occupancy_grid)

    if not polygons:
        print("No polygons were confirmed. Exiting without saving.")
        return

    origin = loaded_yaml["origin"]
    resolution = loaded_yaml["resolution"]
    height = occupancy_grid.shape[0]

    out_no_ext = os.path.join(csv_folder, csv_name)
    save_algo_zones_csv(polygons, origin, resolution, height, out_no_ext)

    total_vertices = sum(len(poly.get("points", [])) if isinstance(poly, dict) else len(poly) for poly in polygons)
    print(f"\nSaved {len(polygons)} polygon(s), {total_vertices} vertex/vertices to {out_no_ext}.csv")


if __name__ == "__main__":
    run()
