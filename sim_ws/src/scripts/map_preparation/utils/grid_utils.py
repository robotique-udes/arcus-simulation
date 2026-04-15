import numpy as np

from .constants import FREE, OCCUPIED, UNKNOWN


def grid_generator(loaded_yaml, img):
    """
    Builds a discrete occupancy grid from a grayscale map image and its
    associated YAML metadata.

    Returns
    -------
    np.ndarray of int8 with values UNKNOWN(-1), FREE(0), OCCUPIED(100)
    """
    mode = loaded_yaml["mode"]
    negate = loaded_yaml["negate"]
    occ_thr = loaded_yaml["occupied_thresh"]
    free_thr = loaded_yaml["free_thresh"]

    p = img if negate else 1.0 - img

    height, width = img.shape
    occupancy_grid = np.full((height, width), UNKNOWN, dtype=np.int8)

    if mode == "trinary":
        occupancy_grid[p >= occ_thr] = OCCUPIED
        occupancy_grid[p <= free_thr] = FREE
    elif mode == "scale":
        occupancy_grid = np.clip((1.0 - img) * 100, 0, 100).astype(np.int8)
    elif mode == "raw":
        occupancy_grid = (img * 255).astype(np.int16)
    else:
        raise ValueError(f"Unknown map mode: {mode}")

    return occupancy_grid


def world_to_grid(world_pos, origin, resolution, height):
    """
    Converts real-world (x, y) metres into (row, col) grid indices.

    The Y axis is flipped because image rows grow downward while the ROS
    world frame grows upward.
    """
    x, y = world_pos[:2]
    ox, oy = origin[:2]

    col = int((x - ox) / resolution)
    row = int((y - oy) / resolution)
    row = height - 1 - row

    return (row, col)


def grid_to_world(grid_pos, origin, resolution, height):
    """
    Converts (row, col) grid indices back to real-world (x, y) in metres.
    Exact inverse of world_to_grid.
    """
    row, col = grid_pos
    ox, oy = origin[:2]

    x = col * resolution + ox
    y = (height - 1 - row) * resolution + oy

    return (x, y)
