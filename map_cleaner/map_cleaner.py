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

min_distance = 10  # minimum pixels away from inner contour

# Get all inner contour points
inner_pts = approx_inner.reshape(-1, 2)  # Nx2 array

# Get all wall points from the binary map
outer_pts = np.column_stack(np.where(binary > 0))  # y, x coordinates
outer_pts = outer_pts[:, ::-1]  # convert to x, y order

# Filter outer points: must be outside inner contour and at least min_distance away
filtered_outer_pts = []
for pt in outer_pts:
    x, y = float(pt[0]), float(pt[1])  # convert to float for pointPolygonTest
    dist = cv2.pointPolygonTest(approx_inner, (x, y), measureDist=True)
    if dist < -min_distance:  # negative = outside
        filtered_outer_pts.append([x, y])

filtered_outer_pts = np.array(filtered_outer_pts)
if filtered_outer_pts.shape[0] == 0:
    print("Warning: no outer points found with given min_distance!")
    filtered_outer_pts = outer_pts  # fallback

# Build KD-tree for fast nearest neighbor search
tree = cKDTree(filtered_outer_pts)

# Map each inner vertex to the nearest valid outer point
closest_outer_pts = []
for pt in inner_pts:
    dist, idx = tree.query((float(pt[0]), float(pt[1])))  # query with float
    closest_outer_pts.append(filtered_outer_pts[idx])

closest_outer_pts = np.array(closest_outer_pts, dtype=np.int32)

# Optional: approximate polygon to smooth
epsilon_outer = 0.01 * cv2.arcLength(closest_outer_pts, True)
approx_outer = cv2.approxPolyDP(closest_outer_pts, epsilon_outer, True)

# Draw outer contour
cv2.drawContours(contour_img, [approx_outer], -1, (0, 0, 255), 3)  # RED

# --- Show results ---
cv2.imshow("Original", img)
cv2.imshow("Binary", binary)
cv2.imshow("Inner Contour", contour_img)

cv2.waitKey(0)
cv2.destroyAllWindows()
