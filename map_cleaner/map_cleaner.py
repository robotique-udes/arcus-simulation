import cv2
import numpy as np


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

    # Force numpy array
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

def generate_other_contour(reference_contour, mode, binary, min_distance=10, max_search=150):
    pts = reference_contour.reshape(-1, 2)
    generated_points = []

    for i in range(len(pts)):
        p_prev = pts[i - 1]
        p_curr = pts[i]
        p_next = pts[(i + 1) % len(pts)]

        normal = get_normal(p_prev, p_next)

        # --- Ensure normal points in the correct direction ---
        test_point = p_curr + normal * min_distance
        dist_test = cv2.pointPolygonTest(reference_contour, tuple(test_point.astype(float)), True)

        if mode == "outer_from_inner":
            # Normal must point OUTSIDE inner contour
            if dist_test >= 0:
                normal = -normal
        else:  # "inner_from_outer"
            # Normal must point INSIDE outer contour
            if dist_test < 0:
                normal = -normal

        # --- Project along normal to find the target contour ---
        found = False
        for d in range(min_distance, max_search):
            probe = p_curr + normal * d
            x, y = int(probe[0]), int(probe[1])

            if not (0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]):
                continue

            if mode == "outer_from_inner":
                # Must be valid outer region
                if binary[y, x] == 0:
                    continue

                # Must be strictly outside inner contour
                dist_inner = cv2.pointPolygonTest(reference_contour, (float(x), float(y)), True)
                if dist_inner < -min_distance:
                    generated_points.append(np.array([x, y], dtype=np.int32))
                    found = True
                    break

            else:  # inner_from_outer
                # Must be inside the outer contour
                if binary[y, x] != 0:
                    dist_outer = cv2.pointPolygonTest(reference_contour, (float(x), float(y)), True)
                    if dist_outer > min_distance:
                        generated_points.append(np.array([x, y], dtype=np.int32))
                        found = True
                        break

        if not found:
            # Optional: fallback if no valid point is found
            continue

    generated_points = reorder_points_nearest(generated_points)

    return np.array(generated_points, dtype=np.int32).reshape(-1, 1, 2)

def select_contour(img, contours, title, highlight=None):
    idx = 0

    while True:
        display = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        if highlight is not None:
            cv2.drawContours(display, [highlight], -1, (0, 255, 0), 2)

        cv2.drawContours(display, contours, idx, (0, 0, 255), 3)

        cv2.putText(display,
                    f"{title} ({idx+1}/{len(contours)}) LEFT/RIGHT/ENTER",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0,255,0),
                    2)

        cv2.imshow(title, display)

        key = cv2.waitKey(0)

        if key == 81:  # LEFT
            idx = (idx - 1) % len(contours)

        elif key == 83:  # RIGHT
            idx = (idx + 1) % len(contours)

        elif key == 13:  # ENTER
            cv2.destroyWindow(title)
            return contours[idx]


# ---------------------------------------------------------
# LOAD IMAGE
# ---------------------------------------------------------

img = cv2.imread("slam_map.pgm", cv2.IMREAD_GRAYSCALE)

# ---------------------------------------------------------
# PREPROCESSING
# ---------------------------------------------------------

binary = cv2.inRange(img, 0, 60)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
binary = cv2.dilate(binary, kernel, iterations=1)

num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)

min_area = 500
filtered = np.zeros_like(binary)

for i in range(1, num_labels):
    if stats[i, cv2.CC_STAT_AREA] >= min_area:
        filtered[labels == i] = 255

binary = filtered

# ---------------------------------------------------------
# FIND CONTOURS
# ---------------------------------------------------------

contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

# ---------------------------------------------------------
# USER SELECT FIRST CONTOUR
# ---------------------------------------------------------

chosen_contour = select_contour(img, contours, "Choose Contour")

cv2.destroyAllWindows()

print("\nSelected contour.")
ctype = input("Is this contour INNER (i) or OUTER (o)? ")

inner = None
outer = None

if ctype.lower() == "i":
    inner = chosen_contour
else:
    outer = chosen_contour


# ---------------------------------------------------------
# HANDLE SECOND CONTOUR
# ---------------------------------------------------------

mode = input("Find outer automatically (a) or manually select (m)? ")

if inner is not None:
    if mode == "m":
        outer = select_contour(img, contours, "Select OUTER contour", inner)
    else:
        outer = generate_other_contour(inner, "outer_from_inner", binary)


elif outer is not None:
    if mode == "m":
        inner = select_contour(img, contours, "Select INNER contour", outer)
    else:
        inner = generate_other_contour(outer, "inner_from_outer", binary)


# ---------------------------------------------------------
# DRAW FINAL RESULT
# ---------------------------------------------------------

contour_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

if inner is not None:
    cv2.drawContours(contour_img, [inner], -1, (0,255,0), 3)

if outer is not None:
    cv2.polylines(contour_img, [outer], True, (0,0,255), 3)

cv2.imshow("Contours", contour_img)


# ---------------------------------------------------------
# SAVE RESULT
# ---------------------------------------------------------

h, w = img.shape
contour_only = np.ones((h,w), dtype=np.uint8) * 255

if inner is not None:
    cv2.polylines(contour_only, [inner], True, 0, 3)

if outer is not None:
    cv2.polylines(contour_only, [outer], True, 0, 3)

cv2.imwrite("final_contours.png", contour_only)

cv2.waitKey(0)
cv2.destroyAllWindows()