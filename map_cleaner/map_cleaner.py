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

tree = cKDTree(filtered_outer_pts)

# --- Helper functions ---
def interpolate_points(p1, p2, step=1):
    dist = np.linalg.norm(p2 - p1)
    num_points = max(int(dist // step), 1)
    return np.linspace(p1, p2, num=num_points, endpoint=True)

def bfs_path(binary_img, start, goal):
    h, w = binary_img.shape
    visited = np.zeros((h, w), dtype=bool)
    prev = np.full((h, w, 2), -1, dtype=int)
    queue = deque([start])
    visited[start[1], start[0]] = True
    neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            path = []
            while (x, y) != start:
                path.append((x, y))
                x, y = prev[y, x]
            path.append(start)
            return path[::-1]
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and not visited[ny, nx] and binary_img[ny, nx] > 0:
                visited[ny, nx] = True
                prev[ny, nx] = (x, y)
                queue.append((nx, ny))
    return [start, goal]

# --- Map inner contour to unique closest outer points with visualization ---
used_indices = set()
unique_closest_points = []
closest_display = contour_img.copy()

for i in range(len(inner_pts)):
    p_start = inner_pts[i]
    p_end = inner_pts[(i + 1) % len(inner_pts)]
    interp_points = interpolate_points(p_start, p_end, sample_step)
    for pt in interp_points:
        # Query nearest 20 outer points
        distances, indices = tree.query(tuple(pt), k=20)
        # Find the first outer point not already used
        chosen_idx = None
        for idx in indices:
            if idx not in used_indices:
                chosen_idx = idx
                break
        if chosen_idx is None:
            # All nearby points are used, skip this point
            continue
        used_indices.add(chosen_idx)
        outer_pt = filtered_outer_pts[chosen_idx]
        unique_closest_points.append(outer_pt)
        # Visualization
        cv2.circle(closest_display, tuple(outer_pt), 3, (0,0,255), -1)
        cv2.line(closest_display, tuple(pt.astype(int)), tuple(outer_pt), (255,0,0), 1)

cv2.imshow("Inner to Closest Outer Points", closest_display)

# --- Build final polygon and display BFS gaps ---
gap_threshold = 75
full_outer_path = []
gap_display = contour_img.copy()

for i in range(len(unique_closest_points)):
    start = tuple(unique_closest_points[i].astype(int))
    goal = tuple(unique_closest_points[(i+1) % len(unique_closest_points)].astype(int))
    if np.linalg.norm(np.array(start) - np.array(goal)) > gap_threshold:
        # Use BFS for large gap
        path_segment = bfs_path(binary, start, goal)
        # Draw BFS path in green
        for j in range(1, len(path_segment)):
            cv2.line(gap_display, path_segment[j-1], path_segment[j], (0,255,0), 1)
    else:
        path_segment = [start, goal]
    if i > 0:
        path_segment = path_segment[1:]
    full_outer_path.extend(path_segment)

cv2.imshow("Contours with BFS Gaps", gap_display)

full_outer_path = np.array(full_outer_path, dtype=np.int32).reshape(-1,1,2)
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
    [outer_contour],
    isClosed=True,
    color=0,   # black
    thickness=3
)

# Draw inner contour in black
cv2.polylines(
    contour_only,
    [inner_contour],
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
