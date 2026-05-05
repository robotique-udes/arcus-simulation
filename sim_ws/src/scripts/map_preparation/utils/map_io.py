import csv
import os
import re

import numpy as np
import yaml
from PIL import Image


def pgm_opener(path):
    """Opens an image file and normalizes pixel values to [0, 1]."""
    img = Image.open(path).convert("L")
    return np.array(img) / 255.0


def yaml_opener(path):
    """Loads a YAML file and returns its contents as a dict."""
    with open(path, "r") as f:
        return yaml.safe_load(f)


def save_csv(data, filename, speeds=None):
    """
    Saves waypoint rows to a CSV file.

    Parameters
    ----------
    data     : list of tuples  e.g. [(x0,y0), (x1,y1), ...]
    filename : str             path WITHOUT the .csv extension
    speeds   : list/np.ndarray or None
        If provided, each output row is written as (x, y, speed).
        Length must match len(data).
    """
    filepath = filename + ".csv"
    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f, delimiter=",")

        if speeds is None:
            writer.writerows(data)
            return

        if len(speeds) != len(data):
            raise ValueError("save_csv: 'speeds' length must match 'data' length")

        rows = []
        for point, v in zip(data, speeds):
            if len(point) < 2:
                raise ValueError("save_csv: each data point must contain at least x and y")
            rows.append((point[0], point[1], v))

        writer.writerows(rows)


def find_latest_map_pair(map_folder):
    """
    Finds the newest timestamped YAML/PGM pair in map_folder.

    Expected filename style includes a timestamp token like YYYYMMDD_HHMMSS,
    e.g. slam_map_20260314_130422.yaml and matching .pgm.

    Returns
    -------
    tuple(str, str)
        (yaml_filename, pgm_filename)
    """
    ts_pattern = re.compile(r"(\d{8}_\d{6})")
    candidates = []

    for entry in os.listdir(map_folder):
        if not entry.lower().endswith(".yaml"):
            continue

        stem = os.path.splitext(entry)[0]
        m = ts_pattern.search(stem)
        if not m:
            continue

        pgm_name = stem + ".pgm"
        pgm_path = os.path.join(map_folder, pgm_name)
        if not os.path.isfile(pgm_path):
            continue

        is_clean = stem.endswith("_clean")
        candidates.append((m.group(1), is_clean, entry, pgm_name))

    if not candidates:
        raise FileNotFoundError(
            f"No timestamped YAML/PGM map pairs found in '{map_folder}'."
        )

    _, _, yaml_name, pgm_name = max(candidates, key=lambda x: (x[0], x[1]))
    return yaml_name, pgm_name


def find_latest_csv(csv_folder):
    """Finds the most recently modified .csv file in csv_folder."""
    if not os.path.isdir(csv_folder):
        raise FileNotFoundError(f"CSV folder not found: {csv_folder}")

    candidates = []
    for entry in os.listdir(csv_folder):
        if not entry.lower().endswith(".csv"):
            continue
        full_path = os.path.join(csv_folder, entry)
        if os.path.isfile(full_path):
            candidates.append((os.path.getmtime(full_path), full_path))

    if not candidates:
        raise FileNotFoundError(f"No CSV files found in '{csv_folder}'.")

    _, latest_path = max(candidates, key=lambda x: x[0])
    return latest_path


def load_csv_xy(filepath):
    """
    Loads (x, y) points from a raceline CSV.

    Accepts rows with at least two numeric columns, e.g. (x, y) or (x, y, speed).
    """
    points = []
    with open(filepath, "r", newline="") as f:
        reader = csv.reader(f, delimiter=",")
        for line_idx, row in enumerate(reader, start=1):
            if len(row) < 2:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
            except ValueError as exc:
                raise ValueError(
                    f"Invalid numeric values in {filepath} at line {line_idx}: {row}"
                ) from exc
            points.append((x, y))

    if len(points) < 4:
        raise ValueError(f"Imported raceline must contain at least 4 points, got {len(points)}")

    return points
