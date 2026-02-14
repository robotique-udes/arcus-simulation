import cv2
import numpy as np

# Load PGM image
img = cv2.imread("slam_map.pgm", cv2.IMREAD_GRAYSCALE)

# --- Threshold ---
binary = cv2.inRange(img, 0, 60)

# --- Morphological cleanup ---
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
binary = cv2.dilate(binary, kernel, iterations=1)

# --- Edge detection ---
edges = cv2.Canny(binary, 50, 150)

# --- Hough lines ---
lines = cv2.HoughLinesP(
    edges,
    rho=1,
    theta=np.pi/180,
    threshold=60,
    minLineLength=40,
    maxLineGap=30
)

line_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(line_img, (x1, y1), (x2, y2), (0, 0, 255), 2)

# --- Contour extraction ---
contours, _ = cv2.findContours(
    binary,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE
)

largest = max(contours, key=cv2.contourArea)

epsilon = 0.01 * cv2.arcLength(largest, True)
approx = cv2.approxPolyDP(largest, epsilon, True)

contour_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
cv2.drawContours(contour_img, [approx], -1, (0, 255, 0), 3)

# --- Show results ---
cv2.imshow("Original", img)
cv2.imshow("Binary", binary)
cv2.imshow("Detected Lines", line_img)
cv2.imshow("Inner Contour", contour_img)

cv2.waitKey(0)
cv2.destroyAllWindows()
