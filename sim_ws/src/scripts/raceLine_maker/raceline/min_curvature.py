import numpy as np

def order_centerline(points):
    pts = np.array(points)
    visited = set()

    # Build KDTree for nearest neighbors
    from scipy.spatial import KDTree
    tree = KDTree(pts)

    ordered = []
    current_idx = 0
    ordered.append(tuple(pts[current_idx]))
    visited.add(current_idx)

    for _ in range(len(pts) - 1):
        dists, idxs = tree.query(pts[current_idx], k=8)

        for i in idxs:
            if i not in visited:
                visited.add(i)
                ordered.append(tuple(pts[i]))
                current_idx = i
                break

    return ordered

def resample_path(path, num_points=500):
    pts = np.array(path, dtype=float)

    # close loop
    pts = np.vstack([pts, pts[0]])

    # arc length
    d = np.sqrt(np.sum(np.diff(pts, axis=0)**2, axis=1))
    s = np.concatenate([[0], np.cumsum(d)])

    s_uniform = np.linspace(0, s[-1], num_points)

    new_pts = np.zeros((num_points, 2))
    for i in range(2):
        new_pts[:, i] = np.interp(s_uniform, s, pts[:, i])

    return [tuple(p) for p in new_pts]
