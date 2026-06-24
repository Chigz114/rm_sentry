"""
EsdfMap2D — Signed 2D ESDF with bilinear interpolation and gradient query.

Input:  nav_msgs/OccupancyGrid (raw costmap, cell ∈ {100=occ, 0=free, -1=unknown})
Output: signed distance field d_signed(x,y)
    - free region:  d_signed > 0  (distance to nearest obstacle)
    - inside obstacle: d_signed < 0  (distance to nearest free space, negated)
    - obstacle boundary: d_signed = 0

Gradient: central difference on the signed ESDF grid, then bilinear interpolated.

This is the foundation for MINCO collision cost — the signed ESDF ensures
that trajectory samples landing inside obstacles still receive a valid
gradient pointing toward free space.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt


class EsdfMap2D:
    def __init__(self, max_distance_m: float = 5.0, smooth_sigma: float = 1.0):
        self.max_dist = max_distance_m
        self.smooth_sigma = smooth_sigma
        self.width = 0
        self.height = 0
        self.resolution = 0.1
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.frame_id = ""
        self._signed_esdf = None      # (H, W) float32, in meters
        self._grad_x = None           # (H, W) float32, ∂d/∂x in world frame
        self._grad_y = None           # (H, W) float32, ∂d/∂y in world frame
        self._valid = False

    @property
    def valid(self) -> bool:
        return self._valid

    def update(self, grid_data: np.ndarray, width: int, height: int,
               resolution: float, origin_x: float, origin_y: float,
               frame_id: str = ""):
        """Rebuild signed ESDF from raw costmap data.

        Args:
            grid_data: flat int8 array (row-major, H rows × W cols), cell ∈ {100, 0, -1}
            width, height: grid dimensions
            resolution: meters per cell
            origin_x, origin_y: world position of grid (0,0) cell's corner (not center)
            frame_id: ROS frame for logging
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.frame_id = frame_id

        grid = np.asarray(grid_data, dtype=np.int8).reshape(height, width)
        obstacle_mask = (grid == 100)

        if not obstacle_mask.any():
            # No obstacles: everything is free at max distance
            self._signed_esdf = np.full((height, width), self.max_dist, dtype=np.float32)
        elif obstacle_mask.all():
            # All obstacles: everything is inside at -max distance
            self._signed_esdf = np.full((height, width), -self.max_dist, dtype=np.float32)
        else:
            # d_free_to_occ: distance from each free cell to nearest obstacle
            d_free = distance_transform_edt(~obstacle_mask).astype(np.float32) * resolution
            # d_occ_to_free: distance from each obstacle cell to nearest free
            d_occ = distance_transform_edt(obstacle_mask).astype(np.float32) * resolution
            # Signed: positive in free, negative in obstacle
            self._signed_esdf = d_free - d_occ

        # Truncate to ±max_dist
        np.clip(self._signed_esdf, -self.max_dist, self.max_dist, out=self._signed_esdf)

        # Optional light Gaussian smoothing to reduce 0.1m staircase in gradients
        if self.smooth_sigma > 0:
            from scipy.ndimage import gaussian_filter
            self._signed_esdf = gaussian_filter(
                self._signed_esdf, sigma=self.smooth_sigma, mode='nearest'
            ).astype(np.float32)

        # Compute gradient via central difference (2nd order accurate)
        # grad_x: ∂d/∂col * (1/res) → ∂d/∂x_world
        # grad_y: ∂d/∂row * (1/res) → ∂d/∂y_world
        # numpy gradient: axis=1 is columns (x), axis=0 is rows (y)
        gy, gx = np.gradient(self._signed_esdf, resolution)
        self._grad_x = gx.astype(np.float32)
        self._grad_y = gy.astype(np.float32)

        self._valid = True

    def _world_to_grid(self, x: float, y: float):
        """Convert world coords to continuous grid indices (col, row)."""
        col = (x - self.origin_x) / self.resolution - 0.5
        row = (y - self.origin_y) / self.resolution - 0.5
        return col, row

    def _bilinear_sample(self, field: np.ndarray, x: float, y: float) -> float:
        """Bilinear interpolation of a 2D field at world (x, y).

        Field is (H, W) indexed as [row, col].
        Returns 0.0 if out of bounds.
        """
        col, row = self._world_to_grid(x, y)
        r0 = int(np.floor(row))
        c0 = int(np.floor(col))
        r1 = r0 + 1
        c1 = c0 + 1

        if r0 < 0 or c0 < 0 or r1 >= self.height or c1 >= self.width:
            return self.max_dist

        dr = row - r0
        dc = col - c0

        v00 = field[r0, c0]
        v01 = field[r0, c1]
        v10 = field[r1, c0]
        v11 = field[r1, c1]

        v0 = v00 * (1 - dc) + v01 * dc
        v1 = v10 * (1 - dc) + v11 * dc
        return float(v0 * (1 - dr) + v1 * dr)

    def get_distance(self, x: float, y: float) -> float:
        """Signed distance at world (x, y). Positive=free, negative=inside obstacle."""
        if not self._valid:
            return self.max_dist
        return self._bilinear_sample(self._signed_esdf, x, y)

    def get_distance_and_gradient(self, x: float, y: float):
        """Returns (d_signed, grad_x, grad_y) at world (x, y).

        grad_x, grad_y point in the direction of increasing distance (toward free space).
        """
        if not self._valid:
            return self.max_dist, 0.0, 0.0
        d = self._bilinear_sample(self._signed_esdf, x, y)
        gx = self._bilinear_sample(self._grad_x, x, y)
        gy = self._bilinear_sample(self._grad_y, x, y)
        return d, gx, gy

    def get_distances_batch(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Vectorized signed distance query at multiple world points.

        Args:
            xs, ys: 1D arrays of world x/y coordinates
        Returns:
            1D array of signed distances (same length as xs/ys)
        """
        if not self._valid:
            return np.full(len(xs), self.max_dist, dtype=np.float32)

        cols = (xs - self.origin_x) / self.resolution - 0.5
        rows = (ys - self.origin_y) / self.resolution - 0.5

        r0 = np.floor(rows).astype(int)
        c0 = np.floor(cols).astype(int)
        r1 = r0 + 1
        c1 = c0 + 1

        # Out-of-bounds mask
        oob = (r0 < 0) | (c0 < 0) | (r1 >= self.height) | (c1 >= self.width)

        # Clamp for safe indexing
        r0c = np.clip(r0, 0, self.height - 1)
        c0c = np.clip(c0, 0, self.width - 1)
        r1c = np.clip(r1, 0, self.height - 1)
        c1c = np.clip(c1, 0, self.width - 1)

        v00 = self._signed_esdf[r0c, c0c]
        v01 = self._signed_esdf[r0c, c1c]
        v10 = self._signed_esdf[r1c, c0c]
        v11 = self._signed_esdf[r1c, c1c]

        dr = rows - r0
        dc = cols - c0

        v0 = v00 * (1 - dc) + v01 * dc
        v1 = v10 * (1 - dc) + v11 * dc
        result = v0 * (1 - dr) + v1 * dr

        result[oob] = self.max_dist
        return result

    def get_signed_esdf_array(self) -> np.ndarray:
        """Return the full signed ESDF array (H, W), for visualization."""
        return self._signed_esdf

    def get_gradient_arrays(self):
        """Return (grad_x, grad_y) arrays (H, W), for visualization."""
        return self._grad_x, self._grad_y

    def world_to_grid_index(self, x: float, y: float):
        """Convert world coords to nearest grid index (col, row). Returns None if out of bounds."""
        col = int(round((x - self.origin_x) / self.resolution - 0.5))
        row = int(round((y - self.origin_y) / self.resolution - 0.5))
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None
