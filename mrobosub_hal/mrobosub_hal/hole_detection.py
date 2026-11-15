"""
Hole detection in a planar target (e.g. a board with cut-outs) from a ZED depth map.

Pure numpy/OpenCV, no ROS, so it can be called from the zed node now and moved
into a perception node later without changes.

Coordinates follow the ZED default (COORDINATE_SYSTEM.IMAGE): X right, Y down,
Z forward along the optical axis, in the units the camera was opened with
(meters if InitParameters.coordinate_units = sl.UNIT.METER).

Requires RuntimeParameters.enable_fill_mode = False (the default); fill mode
interpolates across exactly the gaps this looks for.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Intrinsics:
    fx: float  # focal length in pixels, x
    fy: float  # focal length in pixels, y
    cx: float  # optical center, x (pixels)
    cy: float  # optical center, y (pixels)


@dataclass
class Hole:
    center_px: tuple       # (u, v) pixel centroid
    center_xyz: np.ndarray  # 3D centroid on the board plane, camera frame
    diameter_m: float      # equivalent-circle diameter
    radius_px: float       # for drawing
    circularity: float     # 4*pi*A/P^2, ~1.0 for a circle


class HoleDetector:
    def __init__(
        self,
        K: Intrinsics,
        shape: tuple,               # (height, width) of the depth map
        behind_m: float = 0.10,     # how far behind the plane counts as "seeing through"
        plane_thresh_m: float = 0.02,  # RANSAC inlier distance
        min_diameter_m: float = 0.02,
        max_diameter_m: float = 2.00,
        min_circularity: float = 0.6,  # raise (e.g. 0.6) to accept only round holes
        sample_step: int = 4,       # depth subsampling for plane fitting
        edge_erode_px: int = 15,    # shrink board outline to ignore its own edges
        min_plane_points: int = 500,
    ):
        self.K = K
        self.shape = shape
        self.behind_m = behind_m
        self.plane_thresh_m = plane_thresh_m
        self.min_diameter_m = min_diameter_m
        self.max_diameter_m = max_diameter_m
        self.min_circularity = min_circularity
        self.step = sample_step
        self.erode_kernel = np.ones((edge_erode_px, edge_erode_px), np.uint8)
        self.open_kernel = np.ones((5, 5), np.uint8)
        self.min_plane_points = min_plane_points
        self.rng = np.random.default_rng()

        # Per-pixel ray directions (x/z, y/z), computed once
        v, u = np.mgrid[0 : shape[0], 0 : shape[1]].astype(np.float32)
        self.ray_x = (u - K.cx) / K.fx
        self.ray_y = (v - K.cy) / K.fy

    # ---------- public ----------

    def detect(self, depth: np.ndarray, roi: np.ndarray = None) -> list:
        """Return holes found in the dominant plane of `depth`.

        roi: optional boolean mask (same shape as depth) restricting where the
        board is searched for, e.g. from an RGB detector. Without it, the
        dominant plane in view is used, which may be a wall or floor instead.
        """
        search = depth if roi is None else np.where(roi, depth, np.nan)

        pts, uv = self._depth_to_points(search)
        if len(pts) < self.min_plane_points:
            return []

        fit = self._fit_nearest_plane(pts)
        if fit is None:
            return []
        n, d, inliers = fit

        plane_z = self._plane_depth_map(n, d)
        board = self._board_mask(uv[inliers])

        with np.errstate(invalid="ignore"):
            behind = (depth - plane_z) > self.behind_m
        invalid = ~np.isfinite(depth)
        mask = ((behind | invalid) & board).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel)

        return self._extract_holes(mask, plane_z)

    # ---------- steps ----------

    def _depth_to_points(self, depth):
        """Subsampled valid depth pixels -> 3D points (N,3) and their pixel coords (N,2)."""
        s = self.step
        d = depth[::s, ::s]
        v, u = np.mgrid[0 : depth.shape[0] : s, 0 : depth.shape[1] : s]
        valid = np.isfinite(d) & (d > 0)
        z = d[valid]
        u = u[valid].astype(np.float32)
        v = v[valid].astype(np.float32)
        x = (u - self.K.cx) * z / self.K.fx
        y = (v - self.K.cy) * z / self.K.fy
        return np.column_stack([x, y, z]), np.column_stack([u, v])

    def _fit_nearest_plane(self, pts, max_planes=3):
        """Fit up to `max_planes` planes in sequence and keep the nearest one.

        The board is usually in front of the background (pool wall, floor), but the
        background can have more pixels, so the biggest plane isn't necessarily the
        board. Each round removes the previous plane's inliers and fits again.
        """
        remaining = np.ones(len(pts), bool)
        best = None
        for _ in range(max_planes):
            if np.count_nonzero(remaining) < self.min_plane_points:
                break
            fit = self._fit_plane_ransac(pts[remaining])
            if fit is None:
                break
            n, d, _ = fit
            inliers = np.abs(pts @ n + d) < self.plane_thresh_m
            count = np.count_nonzero(inliers & remaining)
            if count < self.min_plane_points:
                break
            dist = np.median(pts[inliers & remaining, 2])  # median Z of this plane
            if best is None or dist < best[0]:
                best = (dist, n, d, inliers)
            remaining &= ~inliers
        if best is None:
            return None
        _, n, d, inliers = best
        return n, d, inliers

    def _fit_plane_ransac(self, pts, iters=150, sample=2000):
        """Plane n·p + d = 0 with |n| = 1. Vectorized RANSAC on a subsample, then SVD refine."""
        rng = self.rng
        sub = pts if len(pts) <= sample else pts[rng.choice(len(pts), sample, replace=False)]
        if len(sub) < 3:
            return None

        # Score all candidate planes at once instead of looping in Python
        idx = rng.integers(0, len(sub), size=(iters, 3))          # candidate triplets
        p0, p1, p2 = sub[idx[:, 0]], sub[idx[:, 1]], sub[idx[:, 2]]
        n = np.cross(p1 - p0, p2 - p0)                             # (iters, 3) normals
        norm = np.linalg.norm(n, axis=1)
        ok = norm > 1e-9                                           # drop degenerate triplets
        if not ok.any():
            return None
        n = n[ok] / norm[ok, None]
        d = -np.einsum("ij,ij->i", n, p0[ok])
        counts = (np.abs(sub @ n.T + d) < self.plane_thresh_m).sum(axis=0)
        best = counts.argmax()
        best_n, best_d = n[best], d[best]

        # Least-squares refinement on all inliers
        inl = pts[np.abs(pts @ best_n + best_d) < self.plane_thresh_m]
        if len(inl) < 3:
            return None
        centroid = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - centroid, full_matrices=False)
        n = vt[-1]
        d = -n @ centroid
        inliers = np.abs(pts @ n + d) < self.plane_thresh_m
        return n, d, inliers

    def _plane_depth_map(self, n, d):
        """Depth (Z) at which each pixel's ray hits the plane; NaN if it doesn't."""
        denom = n[0] * self.ray_x + n[1] * self.ray_y + n[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            z = -d / denom
        z[~np.isfinite(z) | (z <= 0)] = np.nan
        return z

    def _board_mask(self, inlier_uv):
        """Filled convex hull of the plane's inlier pixels, eroded to skip its edges."""
        mask = np.zeros(self.shape, np.uint8)
        if len(inlier_uv) >= 3:
            hull = cv2.convexHull(inlier_uv.astype(np.int32))
            cv2.fillConvexPoly(mask, hull, 1)
            mask = cv2.erode(mask, self.erode_kernel)
        return mask.astype(bool)

    def _extract_holes(self, mask, plane_z):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        holes = []
        for c in contours:
            area = cv2.contourArea(c)
            perim = cv2.arcLength(c, True)
            if area < 10 or perim == 0:
                continue
            m = cv2.moments(c)
            u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
            z = plane_z[int(round(v)), int(round(u))]
            if not np.isfinite(z):
                continue

            diam_px = 2.0 * np.sqrt(area / np.pi)
            diam_m = diam_px * z / self.K.fx
            circ = 4.0 * np.pi * area / (perim * perim)
            if not (self.min_diameter_m <= diam_m <= self.max_diameter_m):
                continue
            if circ < self.min_circularity:
                continue

            xyz = np.array([(u - self.K.cx) * z / self.K.fx, (v - self.K.cy) * z / self.K.fy, z])
            holes.append(Hole((u, v), xyz, diam_m, diam_px / 2.0, circ))

        holes.sort(key=lambda h: h.diameter_m, reverse=True)
        return holes


@dataclass
class _Track:
    hole: Hole
    hits: int = 1
    misses: int = 0


class HoleTracker:
    """Keeps only holes that persist across frames, smoothing their positions."""

    def __init__(self, match_dist_m=0.10, min_hits=5, max_misses=3, alpha=0.3):
        self.match_dist_m = match_dist_m  # max 3D jump to count as the same hole
        self.min_hits = min_hits          # frames seen before it's reported
        self.max_misses = max_misses      # frames missed before it's dropped
        self.alpha = alpha                # smoothing weight for new observations
        self.tracks: list = []

    def update(self, holes: list) -> list:
        unmatched = list(holes)
        for t in self.tracks:
            best = None
            best_dist = self.match_dist_m
            for h in unmatched:
                dist = np.linalg.norm(h.center_xyz - t.hole.center_xyz)
                if dist < best_dist:
                    best, best_dist = h, dist
            if best is None:
                t.misses += 1
                continue
            unmatched.remove(best)
            a = self.alpha
            t.hole = Hole(
                center_px=tuple((1 - a) * np.array(t.hole.center_px) + a * np.array(best.center_px)),
                center_xyz=(1 - a) * t.hole.center_xyz + a * best.center_xyz,
                diameter_m=(1 - a) * t.hole.diameter_m + a * best.diameter_m,
                radius_px=(1 - a) * t.hole.radius_px + a * best.radius_px,
                circularity=best.circularity,
            )
            t.hits += 1
            t.misses = 0

        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]
        self.tracks += [_Track(h) for h in unmatched]
        return [t.hole for t in self.tracks if t.hits >= self.min_hits]
