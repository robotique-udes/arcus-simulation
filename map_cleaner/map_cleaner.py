import cv2
import numpy as np
from scipy.spatial import cKDTree
from collections import deque

# --- Load PGM image ---
img = cv2.imread("slam_map.pgm", cv2.IMREAD_GRAYSCALE)
cv2.imshow("Original", img)

# --- Threshold and cleanup ---
binary = cv2.inRange(img, 0, 60)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
binary = cv2.dilate(binary, kernel, iterations=1)

# --- Remove small connected components ---
num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
min_area = 500
filtered = np.zeros_like(binary)
for i in range(1, num_labels):
    if stats[i, cv2.CC_STAT_AREA] >= min_area:
        filtered[labels == i] = 255
binary = filtered
cv2.imshow("Binary", binary)

# --- Find contours ---
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contour_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# --- Inner contour (largest) ---
inner = max(contours, key=cv2.contourArea)
cv2.drawContours(contour_img, [inner], -1, (0, 255, 0), 3)

# --- Outer contour points ---
min_distance = 10
sample_step = 1

inner_pts = inner.reshape(-1, 2)
outer_pts = np.column_stack(np.where(binary > 0))[:, ::-1]  # y,x -> x,y

# ----- Filter outer points: must be outside inner contour and at least min_distance away -----
filtered_outer_pts = []
for pt in outer_pts:
    x, y = float(pt[0]), float(pt[1])
    dist = cv2.pointPolygonTest(inner, (x, y), measureDist=True)
    if dist < -min_distance:
        filtered_outer_pts.append([int(x), int(y)])
filtered_outer_pts = np.array(filtered_outer_pts)

# --- Helper functions ---
def get_normal(p_prev, p_next):
    tangent = p_next - p_prev
    tangent = tangent / (np.linalg.norm(tangent) + 1e-6)
    normal = np.array([-tangent[1], tangent[0]])  # 90° rotation
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

unique_closest_points = []
closest_display = contour_img.copy()

max_search = 150

for i in range(len(inner_pts)):
    p_prev = inner_pts[i - 1]
    p_curr = inner_pts[i]
    p_next = inner_pts[(i + 1) % len(inner_pts)]

    normal = get_normal(p_prev, p_next)

    # --- Ensure normal points OUTWARD ---
    test_point = p_curr + normal * min_distance
    dist_test = cv2.pointPolygonTest(inner, tuple(test_point.astype(float)), True)

    # If still inside inner contour, flip normal
    if dist_test >= 0:
        normal = -normal

    found = False

    for d in range(min_distance, max_search):
        probe = p_curr + normal * d
        x, y = int(probe[0]), int(probe[1])

        if not (0 <= x < binary.shape[1] and 0 <= y < binary.shape[0]):
            continue

        # --- MUST be valid outer region ---
        if binary[y, x] == 0:
            continue

        # --- MUST be strictly outside inner contour ---
        dist_inner = cv2.pointPolygonTest(inner, (float(x), float(y)), True)

        if dist_inner < -min_distance:
            unique_closest_points.append(np.array([x, y], dtype=np.int32))

            cv2.circle(closest_display, (x, y), 3, (0, 0, 255), -1)
            cv2.line(closest_display, tuple(p_curr), (x, y), (255, 0, 0), 1)

            found = True
            break

    if not found:
        continue

unique_closest_points = reorder_points_nearest(unique_closest_points)

cv2.imshow("Inner to Closest Outer Points", closest_display)

full_outer_path = np.array(unique_closest_points, dtype=np.int32).reshape(-1,1,2)
cv2.polylines(contour_img, [full_outer_path], isClosed=True, color=(0,0,255), thickness=3)

# --- Save inner + outer contours only (black on white background) ---

# Create white canvas
h, w = img.shape
contour_only = np.ones((h, w), dtype=np.uint8) * 255  # white background

# Ensure correct shapes
outer_contour = full_outer_path
inner_contour = inner.reshape(-1, 1, 2).astype(np.int32)

# Draw outer contour in black
cv2.polylines(
    contour_only,
    [outer_contour, inner_contour],
    isClosed=True,
    color=0,   # black
    thickness=3
)

# Save as PNG
cv2.imwrite("final_contours.png", contour_only)

# --- Show final results ---
cv2.imshow("Contours", contour_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
