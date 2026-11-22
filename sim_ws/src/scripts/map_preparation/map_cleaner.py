import copy
import os

import cv2
import numpy as np
import yaml

from utils.map_io import find_latest_map_pair, pgm_opener, yaml_opener


# ---------------------------------------------------------
# DETECTION PARAMETERS (tune these)
# ---------------------------------------------------------

MIN_WALL_AREA     = 500   # px: large walls
MIN_OBSTACLE_AREA = 200   # px: small obstacle (square/cone) after dilation
MAX_OBSTACLE_SIZE = 30    # px: max bounding-box side of a small obstacle
MIN_OBSTACLE_FILL = 0.6   # area / (w*h): rejects lidar ray streaks


# ---------------------------------------------------------
# HELPER FUNCTIONS
# ---------------------------------------------------------

def get_normal(p_prev, p_next):
    tangent = p_next - p_prev
    tangent = tangent / (np.linalg.norm(tangent) + 1e-6)
    normal = np.array([-tangent[1], tangent[0]])
    return normal


def reorder_points_nearest(points):
    if len(points) == 0:
        return np.array([])

    points = np.asarray(points, dtype=np.int32)
    used = np.zeros(len(points), dtype=bool)
    ordered = []

    # Start from leftmost point
    start_idx = np.argmin(points[:, 0])
    current_idx = start_idx

    ordered.append(points[current_idx])
    used[current_idx] = True

    for _ in range(len(points) - 1):
        current_point = points[current_idx]
        remaining = np.where(~used)[0]
        if len(remaining) == 0:
            break
        dists = np.linalg.norm(points[remaining] - current_point, axis=1)
        nearest_idx = remaining[np.argmin(dists)]
        ordered.append(points[nearest_idx])
        used[nearest_idx] = True
        current_idx = nearest_idx

    return np.array(ordered, dtype=np.int32)


def point_in_any_contour(contours, point, min_dist=0):
    """Return True if point is inside any of the given contours (with optional margin)."""
    for c in contours:
        d = cv2.pointPolygonTest(c, (float(point[0]), float(point[1])), True)
        if d >= -min_dist:
            return True
    return False


def generate_other_contour(reference_contours, mode, binary, min_distance=10, max_search=150):
    """
    Generate an outer contour from one or more inner contours, or vice versa.

    Parameters
    ----------
    reference_contours : list of np.ndarray
        One or more contours used as the reference (inner or outer).
    mode : str
        "outer_from_inner" — probe outward from inner contour(s) to find the outer wall.
        "inner_from_outer" — probe inward from the outer contour to find inner wall(s).
    binary : np.ndarray
        Preprocessed binary image (walls = 255).
    min_distance : int
        Minimum probe distance in pixels.
    max_search : int
        Maximum probe distance in pixels.

    Returns
    -------
    np.ndarray  shape (-1, 1, 2)
    """
    generated_points = []

    for ref_contour in reference_contours:
        pts = ref_contour.reshape(-1, 2)

        for i in range(len(pts)):
            p_prev = pts[i - 1]
            p_curr = pts[i]
            p_next = pts[(i + 1) % len(pts)]

            normal = get_normal(p_prev, p_next)

            # Ensure normal points in the correct direction relative to THIS contour
            test_point = p_curr + normal * min_distance
            dist_test = cv2.pointPolygonTest(ref_contour, tuple(test_point.astype(float)), True)

            if mode == "outer_from_inner":
                # Normal must point OUTSIDE this inner contour
                if dist_test >= 0:
                    normal = -normal
            else:  # inner_from_outer
                # Normal must point INSIDE the outer contour
                if dist_test < 0:
                    normal = -normal

            # Probe along normal to find the target boundary
            for d in range(min_distance, max_search):
                probe = p_curr + normal * d
                x, y = int(probe[0]), int(probe[1])

                if not (0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]):
                    continue

                if mode == "outer_from_inner":
                    if binary[y, x] == 0:
                        continue
                    # Accept point only if it is outside ALL inner contours
                    if not point_in_any_contour(reference_contours, np.array([x, y]), min_distance):
                        generated_points.append(np.array([x, y], dtype=np.int32))
                        break

                else:  # inner_from_outer
                    if binary[y, x] != 0:
                        dist_outer = cv2.pointPolygonTest(ref_contour, (float(x), float(y)), True)
                        if dist_outer > min_distance:
                            generated_points.append(np.array([x, y], dtype=np.int32))
                            break

    if len(generated_points) == 0:
        return np.array([], dtype=np.int32).reshape(-1, 1, 2)

    generated_points = reorder_points_nearest(generated_points)
    return np.array(generated_points, dtype=np.int32).reshape(-1, 1, 2)


def select_contour(img, contours, title, highlights=None):
    """
    Interactive contour picker. LEFT/RIGHT to cycle, ENTER to confirm.

    Parameters
    ----------
    highlights : list of np.ndarray or None
        Already-chosen contours drawn in green for reference.
    """
    idx = 0

    while True:
        display = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        if highlights:
            cv2.drawContours(display, highlights, -1, (0, 255, 0), 2)

        cv2.drawContours(display, contours, idx, (0, 0, 255), 3)
        cv2.putText(
            display,
            f"{title} ({idx + 1}/{len(contours)}) LEFT/RIGHT/ENTER",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 0),
            2,
        )
        cv2.imshow(title, display)

        key = cv2.waitKey(0)
        if key == 81:   # LEFT
            idx = (idx - 1) % len(contours)
        elif key == 83:  # RIGHT
            idx = (idx + 1) % len(contours)
        elif key == 13:  # ENTER
            cv2.destroyWindow(title)
            return contours[idx]


def _wrap_text(text, font, scale, thickness, max_width):
    """Split text into lines that fit within max_width pixels."""
    lines, current = [], ""
    for word in text.split():
        trial = (current + " " + word).strip()
        if not current or cv2.getTextSize(trial, font, scale, thickness)[0][0] <= max_width:
            current = trial
        else:
            lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines


def ask_yes_no(image, question, title="Question", yes_text="YES (y)", no_text="NO (n)"):
    """
    Show `image` with a question and two buttons. Returns True (yes) or False (no).
    Click a button, or press y/ENTER or n/ESC.
    """
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    font = cv2.FONT_HERSHEY_SIMPLEX
    margin, line_h, btn_h, gap = 20, 28, 36, 30
    h, w = image.shape[:2]

    # Canvas must be wide enough for both buttons; the question wraps to fit
    btn_w = max(cv2.getTextSize(t, font, 0.6, 2)[0][0] for t in (yes_text, no_text)) + 30
    canvas_w = max(w, 2 * btn_w + gap + 2 * margin)
    lines = _wrap_text(question, font, 0.7, 2, canvas_w - 2 * margin)

    y0 = h + 30 + (len(lines) - 1) * line_h + 16
    bar_h = (y0 - h) + btn_h + 16

    canvas = np.full((h + bar_h, canvas_w, 3), (40, 40, 40), dtype=np.uint8)
    x_off = (canvas_w - w) // 2
    canvas[:h, x_off:x_off + w] = image
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (margin, h + 30 + i * line_h), font, 0.7, (255, 255, 255), 2)

    yes_rect = (margin, y0, margin + btn_w, y0 + btn_h)
    no_rect = (margin + btn_w + gap, y0, margin + 2 * btn_w + gap, y0 + btn_h)
    for (bx0, by0, bx1, by1), text, color in (
        (yes_rect, yes_text, (0, 150, 0)),
        (no_rect, no_text, (0, 0, 170)),
    ):
        cv2.rectangle(canvas, (bx0, by0), (bx1, by1), color, -1)
        cv2.putText(canvas, text, (bx0 + 15, by0 + 25), font, 0.6, (255, 255, 255), 2)

    result = {"value": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            for rect, value in ((yes_rect, True), (no_rect, False)):
                if rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]:
                    result["value"] = value

    cv2.namedWindow(title)
    cv2.setMouseCallback(title, on_mouse)
    cv2.imshow(title, canvas)
    cv2.waitKey(1)

    # WND_PROP_VISIBLE always returns -1 on some backends (GTK): only trust it if it works
    try:
        can_detect_close = cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        can_detect_close = False

    while result["value"] is None:
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("y"), ord("Y"), 13):
            result["value"] = True
        elif key in (ord("n"), ord("N"), 27):
            result["value"] = False
        elif can_detect_close and cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
            result["value"] = False

    cv2.destroyWindow(title)
    cv2.waitKey(1)
    return result["value"]


def autodetect_contours(contours, hierarchy, min_area=5000, min_inner_area=100):
    """
    Use RETR_TREE hierarchy to automatically identify the outer contour
    and all inner contours (islands and small obstacles inside the track).

    Returns
    -------
    outer : np.ndarray
    inners : list of np.ndarray
    """
    h = hierarchy[0]  # shape (N, 4): [next, prev, first_child, parent]

    # Wall = top-level contour (no parent) with the largest area
    top_level = [
        i for i in range(len(contours))
        if h[i][3] == -1 and cv2.contourArea(contours[i]) > min_area
    ]
    if not top_level:
        raise RuntimeError("No top-level contour with sufficient area found.")

    wall_idx = max(top_level, key=lambda i: cv2.contourArea(contours[i]))

    # A thick wall has two edges: the track boundary is its inner edge (the child "hole")
    outer_idx = wall_idx
    holes = [i for i in range(len(contours)) if h[i][3] == wall_idx]
    if holes:
        hole_idx = max(holes, key=lambda i: cv2.contourArea(contours[i]))
        if cv2.contourArea(contours[hole_idx]) > 0.5 * cv2.contourArea(contours[wall_idx]):
            outer_idx = hole_idx

    outer = contours[outer_idx]

    # Inners = direct children of the track boundary
    inners = [
        contours[i] for i in range(len(contours))
        if h[i][3] == outer_idx and cv2.contourArea(contours[i]) >= min_inner_area
    ]
    return outer, inners


# ---------------------------------------------------------
# PATH HELPERS
# ---------------------------------------------------------

def _resolve_paths():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    input_folder = os.path.join(script_dir, "slam_maps")

    yaml_file, pgm_file = find_latest_map_pair(input_folder)
    input_yaml_path = os.path.join(input_folder, yaml_file)
    input_pgm_path = os.path.join(input_folder, pgm_file)

    base_name = os.path.splitext(yaml_file)[0]
    output_base_name = base_name + "_clean"
    output_yaml_path = os.path.join(input_folder, output_base_name + ".yaml")
    output_pgm_path = os.path.join(input_folder, output_base_name + ".pgm")

    return input_yaml_path, input_pgm_path, output_yaml_path, output_pgm_path


def _write_clean_map_yaml(source_yaml, output_yaml_path, output_pgm_name):
    cleaned_yaml = copy.deepcopy(source_yaml)
    cleaned_yaml["image"] = output_pgm_name
    with open(output_yaml_path, "w") as f:
        yaml.safe_dump(cleaned_yaml, f, sort_keys=False)


# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------

def main():
    input_yaml_path, input_pgm_path, output_yaml_path, output_pgm_path = _resolve_paths()
    print(
        f"Using latest saved map pair:\n"
        f"  YAML: {os.path.basename(input_yaml_path)}\n"
        f"  PGM : {os.path.basename(input_pgm_path)}"
    )

    # ---------------------------------------------------------
    # LOAD IMAGE
    # ---------------------------------------------------------
    img = pgm_opener(input_pgm_path)
    img = (img * 255).astype(np.uint8)

    # ---------------------------------------------------------
    # PREPROCESSING
    # ---------------------------------------------------------
    binary = cv2.inRange(img, 0, 60)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    binary = cv2.dilate(binary, kernel, iterations=1)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    filtered = np.zeros_like(binary)
    for i in range(1, num_labels):
        x, y, w, h, area = stats[i]
        is_wall = area >= MIN_WALL_AREA
        is_small_obstacle = (
            area >= MIN_OBSTACLE_AREA
            and max(w, h) <= MAX_OBSTACLE_SIZE
            and area / (w * h) >= MIN_OBSTACLE_FILL
        )
        if is_wall or is_small_obstacle:
            filtered[labels == i] = 255
    binary = filtered

    # ---------------------------------------------------------
    # FIND CONTOURS (RETR_TREE gives parent/child relationships)
    # ---------------------------------------------------------
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # ---------------------------------------------------------
    # AUTO-DETECT OUTER + INNERS
    # ---------------------------------------------------------
    try:
        outer, inners = autodetect_contours(contours, hierarchy, min_area=5000)
        print(
            f"\nAuto-detected: 1 outer contour ({cv2.contourArea(outer):.0f} px²), "
            f"{len(inners)} inner contour(s): "
            + ", ".join(f"{cv2.contourArea(c):.0f} px²" for c in inners)
        )
    except RuntimeError as e:
        print(f"Auto-detection failed: {e}")
        outer, inners = None, []

    # ---------------------------------------------------------
    # PREVIEW AUTO-DETECTED RESULT
    # ---------------------------------------------------------
    use_auto = False
    if outer is not None:
        preview = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(preview, [outer], -1, (0, 0, 255), 3)   # outer = red
        if inners:
            cv2.drawContours(preview, inners, -1, (0, 255, 0), 3)  # inners = green
        use_auto = ask_yes_no(
            preview,
            "Use auto-detected contours?  (red = outer, green = inner)",
            title="Auto-detected contours",
        )

    if not use_auto:
        # --- Manual selection ---
        contour_list = list(contours)

        outer = select_contour(img, contour_list, "Select OUTER contour")

        inners = []
        while True:
            current = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(current, [outer], -1, (0, 0, 255), 3)
            if inners:
                cv2.drawContours(current, inners, -1, (0, 255, 0), 3)
            if not ask_yes_no(
                current,
                f"Add an INNER contour? ({len(inners)} selected so far)",
                title="Add inner contour",
            ):
                break
            inner = select_contour(
                img, contour_list,
                f"Select INNER contour #{len(inners) + 1}",
                highlights=[outer] + inners,
            )
            inners.append(inner)

    # ---------------------------------------------------------
    # HANDLE MISSING CONTOUR (generate automatically)
    # ---------------------------------------------------------
    if not inners:
        base = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(base, [outer], -1, (0, 0, 255), 3)
        if ask_yes_no(
            base,
            "No inner contour. Generate it automatically from the outer?",
            title="Generate inner contour",
        ):
            print("Generating inner contour(s) from outer…")
            generated = generate_other_contour([outer], "inner_from_outer", binary)
            if len(generated) > 0:
                inners = [generated]
                print(f"  Generated {len(generated)} inner points.")
            else:
                print("  Could not generate inner contour — check min_distance / max_search.")

    # ---------------------------------------------------------
    # BUILD CLEAN MAP
    # ---------------------------------------------------------
    h_img, w_img = img.shape

    track_map = np.zeros((h_img, w_img), dtype=np.uint8)   # black background (everything is an obstacle)
    cv2.fillPoly(track_map, [outer], 255)                  # outer interior in white (free)

    for inner in inners:
        cv2.fillPoly(track_map, [inner], 0)                # inner interiors in black (obstacle)

    cv2.polylines(track_map, [outer], True, 0, 3)
    for inner in inners:
        cv2.polylines(track_map, [inner], True, 0, 3)

    # ---------------------------------------------------------
    # PREVIEW CLEAN MAP + SAVE
    # ---------------------------------------------------------
    if not ask_yes_no(
        track_map,
        "Save this map?",
        title="Clean map",
        yes_text="SAVE (y)",
        no_text="CANCEL (n)",
    ):
        print("Save cancelled.")
        return

    cv2.imwrite(output_pgm_path, track_map)
    _write_clean_map_yaml(
        yaml_opener(input_yaml_path),
        output_yaml_path,
        os.path.basename(output_pgm_path),
    )

    print(
        f"\nSaved cleaned map to:\n"
        f"  YAML: {output_yaml_path}\n"
        f"  PGM : {output_pgm_path}"
    )


if __name__ == "__main__":
    main()