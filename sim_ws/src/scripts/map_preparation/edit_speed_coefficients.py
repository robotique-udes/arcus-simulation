# edit_speed_coefficients.py
from utils.ui_speed_coefficient import edit_speed_coefficients
from utils.map_io import find_latest_map_pair, pgm_opener, yaml_opener
from utils.grid_utils import grid_generator
from raceline_config import DEFAULT_CONFIG
import os, sys

def main():
    cfg = DEFAULT_CONFIG

    if not os.path.isdir(cfg.map_folder):
        print(f"Error: Map folder not found: {cfg.map_folder}", file=sys.stderr)
        sys.exit(1)

    try:
        yaml_file, pgm_file = find_latest_map_pair(cfg.map_folder)
    except FileNotFoundError as e:
        print(f"Error: No map found in '{cfg.map_folder}'.\n{e}", file=sys.stderr)
        sys.exit(1)

    full_yaml_path = os.path.join(cfg.map_folder, yaml_file)
    full_pgm_path  = os.path.join(cfg.map_folder, pgm_file)

    print(f"[Speed Coeff Editor] Loading map:")
    print(f"  YAML: {yaml_file}")
    print(f"  PGM : {pgm_file}")

    loaded_yaml    = yaml_opener(full_yaml_path)
    loaded_img     = pgm_opener(full_pgm_path)
    occupancy_grid = grid_generator(loaded_yaml, loaded_img)

    csv_path = os.path.join(cfg.csv_folder, f"{cfg.csv_name}.csv")
    if not os.path.exists(csv_path):
        print(f"Error: No waypoints at '{csv_path}'. Generate raceline first.", file=sys.stderr)
        sys.exit(1)

    success = edit_speed_coefficients(occupancy_grid, csv_path, loaded_yaml)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()