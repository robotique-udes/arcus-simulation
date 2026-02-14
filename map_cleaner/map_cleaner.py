import cv2
import numpy as np

# Load PGM image
img = cv2.imread("slam_map.pgm", cv2.IMREAD_GRAYSCALE)

# --- Threshold ---
binary = cv2.inRange(img, 0, 60)

# --- Morphological cleanup ---
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
binary = cv2.dilate(binary, kernel, iterations=1)

# --- Remove small connected components (noise filtering) ---
num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)

min_area = 500

filtered = np.zeros_like(binary)

for i in range(1, num_labels):  # skip background
    if stats[i, cv2.CC_STAT_AREA] >= min_area:
        filtered[labels == i] = 255

binary = filtered

# ----- CONTOURS -----
contours, _ = cv2.findContours(
    binary,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE
)

contour_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

# ----- INNER CONTOUR -----
inner = max(contours, key=cv2.contourArea)
epsilon_inner = 0.01 * cv2.arcLength(inner, True)
approx_inner = cv2.approxPolyDP(inner, epsilon_inner, True)

cv2.drawContours(contour_img, [approx_inner], -1, (0, 255, 0), 3)

# ----- OUTER CONTOUR -----
import numpy as np
from scipy.spatial import cKDTree
import cv2

min_distance = 10  # minimum pixels away from inner contour
sample_step = 1   # sample every pixel along edges

inner_pts = approx_inner.reshape(-1, 2)  # Nx2 array
outer_pts = np.column_stack(np.where(binary > 0))  # y,x coords
outer_pts = outer_pts[:, ::-1]  # to x,y

# Filter outer points: outside inner contour & min_distance away
filtered_outer_pts = []
for pt in outer_pts:
    x, y = float(pt[0]), float(pt[1])
    dist = cv2.pointPolygonTest(approx_inner, (x, y), measureDist=True)
    if dist < -min_distance:
        filtered_outer_pts.append([x, y])

filtered_outer_pts = np.array(filtered_outer_pts)
if filtered_outer_pts.shape[0] == 0:
    print("Warning: no outer points found with given min_distance!")
    filtered_outer_pts = outer_pts  # fallback

# Build KD-tree for nearest neighbor search
tree = cKDTree(filtered_outer_pts)

# Function to linearly interpolate points between two vertices
def interpolate_points(p1, p2, step=1):
    dist = np.linalg.norm(p2 - p1)
    num_points = max(int(dist // step), 1)
    return np.linspace(p1, p2, num=num_points, endpoint=True)

# Remove consecutive duplicate points to avoid line artifacts
def remove_consecutive_duplicates(points):
    filtered = [points[0]]
    for p in points[1:]:
        if not np.array_equal(p, filtered[-1]):
            filtered.append(p)
    return np.array(filtered)

# --- Initialize a set to keep track of used outer points ---
used_indices = set()
unique_closest_points = []

for i in range(len(inner_pts)):
    p_start = inner_pts[i]
    p_end = inner_pts[(i + 1) % len(inner_pts)]  # wrap around
    
    interp_points = interpolate_points(p_start, p_end, step=sample_step)
    
    for pt in interp_points:
        # Query multiple nearest neighbors to find one not used yet
        distances, indices = tree.query((float(pt[0]), float(pt[1])), k=10)  # check up to 10 nearest
        
        chosen_idx = None
        for idx in indices:
            if idx not in used_indices:
                chosen_idx = idx
                break
        
        # If all neighbors used, fallback to closest anyway
        if chosen_idx is None:
            chosen_idx = indices[0]
        
        used_indices.add(chosen_idx)
        unique_closest_points.append(filtered_outer_pts[chosen_idx])

# Remove consecutive duplicates as before
polygon_pts = remove_consecutive_duplicates(unique_closest_points).astype(np.int32).reshape(-1, 1, 2)

# Draw the polygon linking all unique closest outer points
cv2.polylines(contour_img, [polygon_pts], isClosed=True, color=(0, 0, 255), thickness=3)



# --- Show results ---
cv2.imshow("Original", img)
cv2.imshow("Binary", binary)
cv2.imshow("Inner Contour", contour_img)

cv2.waitKey(0)
cv2.destroyAllWindows()
