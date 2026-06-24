"""
JPS path post-processing pipeline for MINCO initialization.

Pipeline:
  1. Line-of-sight pruning — remove collinear/redundant waypoints
  2. Waypoint spacing control — enforce 0.3–0.8m between consecutive waypoints
  3. Time allocation — T_i = dist_i / v_max, with minimum clamp

Input:  list of (x, y) world coordinates (from JPS path)
Output: (waypoints, durations) — ready for MINCO initialization

Key rules (from design doc §5.5):
  - Keep corner points (direction changes)
  - Keep narrow passage entry/exit points
  - Thin out straight segments
  - Min segment length 0.3m, max 0.8m
  - Time allocation with T_min clamp to avoid T→0
"""
import math
import numpy as np
from typing import List, Tuple


def line_of_sight_prune(points: List[Tuple[float, float]],
                        eps: float = 0.01) -> List[Tuple[float, float]]:
    """Remove collinear intermediate points (Douglas-Peucker style simplification).

    Keeps points where direction changes by more than eps radians.
    Always keeps first and last points.
    """
    if len(points) <= 2:
        return list(points)

    result = [points[0]]
    for i in range(1, len(points) - 1):
        # Direction from prev kept point to current
        ax, ay = result[-1]
        bx, by = points[i]
        cx, cy = points[i + 1]

        # Vectors
        v1x, v1y = bx - ax, by - ay
        v2x, v2y = cx - bx, cy - by

        l1 = math.hypot(v1x, v1y)
        l2 = math.hypot(v2x, v2y)
        if l1 < 1e-9 or l2 < 1e-9:
            continue

        cos_angle = (v1x * v2x + v1y * v2y) / (l1 * l2)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angle = math.acos(cos_angle)

        if angle > eps:
            result.append(points[i])

    result.append(points[-1])
    return result


def spacing_control(points: List[Tuple[float, float]],
                    min_spacing: float = 0.3,
                    max_spacing: float = 0.8) -> List[Tuple[float, float]]:
    """Enforce waypoint spacing constraints.

    - If consecutive points are closer than min_spacing, merge/drop the later one
      (but never drop corner points — detected by direction change).
    - If consecutive points are farther than max_spacing, insert intermediate points.
    - Always keeps first and last.
    """
    if len(points) <= 1:
        return list(points)

    result = [points[0]]

    for i in range(1, len(points)):
        prev = result[-1]
        curr = points[i]
        dist = math.hypot(curr[0] - prev[0], curr[1] - prev[1])

        is_last = (i == len(points) - 1)

        if dist < min_spacing and not is_last:
            # Too close: skip this point (it's a redundant intermediate)
            # But if next point changes direction significantly, keep it
            if i + 1 < len(points):
                nxt = points[i + 1]
                v1x, v1y = curr[0] - prev[0], curr[1] - prev[1]
                v2x, v2y = nxt[0] - curr[0], nxt[1] - curr[1]
                l1 = math.hypot(v1x, v1y)
                l2 = math.hypot(v2x, v2y)
                if l1 > 1e-9 and l2 > 1e-9:
                    cos_a = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (l1 * l2)))
                    if math.acos(cos_a) > 0.15:  # ~8.6 degrees
                        result.append(curr)
                        continue
            # Skip
            continue

        if dist > max_spacing:
            # Too far: insert evenly spaced intermediate points
            n_insert = int(math.ceil(dist / max_spacing)) - 1
            for j in range(1, n_insert + 1):
                t = j / (n_insert + 1)
                ix = prev[0] + t * (curr[0] - prev[0])
                iy = prev[1] + t * (curr[1] - prev[1])
                result.append((ix, iy))

        result.append(curr)

    return result


def allocate_times(waypoints: List[Tuple[float, float]],
                   v_max: float = 4.0,
                   t_min: float = 0.1) -> List[float]:
    """Allocate duration for each segment: T_i = dist_i / v_max, clamped to t_min.

    Args:
        waypoints: ordered list of (x, y) world coords
        v_max: nominal max velocity for time estimation
        t_min: minimum segment duration to avoid T→0 singularity

    Returns:
        durations: list of T_i for each segment (len = len(waypoints) - 1)
    """
    if len(waypoints) < 2:
        return []

    durations = []
    for i in range(len(waypoints) - 1):
        dist = math.hypot(
            waypoints[i + 1][0] - waypoints[i][0],
            waypoints[i + 1][1] - waypoints[i][1],
        )
        t = max(dist / v_max, t_min)
        durations.append(t)

    return durations


def _merge_short_segments(points: List[Tuple[float, float]],
                          min_spacing: float = 1.5
                          ) -> List[Tuple[float, float]]:
    """Merge consecutive waypoints closer than min_spacing.

    Iteratively replaces two close consecutive points with their midpoint.
    Always keeps first and last points. Runs until no segment is too short.
    """
    if len(points) <= 2:
        return list(points)

    result = list(points)
    changed = True
    while changed and len(result) > 2:
        changed = False
        for i in range(len(result) - 1):
            d = math.hypot(
                result[i+1][0] - result[i][0],
                result[i+1][1] - result[i][1],
            )
            if d < min_spacing:
                # Don't merge if one of them is first or last
                if i == 0:
                    # Drop the second point (keep start)
                    result.pop(i + 1)
                elif i + 1 == len(result) - 1:
                    # Drop the first point (keep end)
                    result.pop(i)
                else:
                    # Merge into midpoint
                    mx = (result[i][0] + result[i+1][0]) / 2.0
                    my = (result[i][1] + result[i+1][1]) / 2.0
                    result[i] = (mx, my)
                    result.pop(i + 1)
                changed = True
                break
    return result


def postprocess_jps_path(points: List[Tuple[float, float]],
                         v_max: float = 4.0,
                         min_spacing: float = 0.3,
                         max_spacing: float = 0.8,
                         prune_eps: float = 0.25,
                         t_min: float = 0.1
                         ) -> Tuple[List[Tuple[float, float]], List[float]]:
    """Full post-processing pipeline: prune → spacing → time allocation.

    Args:
        points: raw JPS waypoints as list of (x, y) world coordinates
        v_max: nominal velocity for time allocation
        min_spacing, max_spacing: waypoint spacing bounds
        prune_eps: angle threshold (rad) for collinear pruning
        t_min: minimum segment duration

    Returns:
        (waypoints, durations): processed waypoints and per-segment durations
    """
    if len(points) < 2:
        return list(points), []

    # Step 1: collinear pruning
    pruned = line_of_sight_prune(points, eps=prune_eps)

    # Step 2: spacing control
    spaced = spacing_control(pruned, min_spacing=min_spacing, max_spacing=max_spacing)

    # Step 2b: enforce minimum segment length — merge any segments still too short
    spaced = _merge_short_segments(spaced, min_spacing=min_spacing)

    # Step 3: time allocation
    durations = allocate_times(spaced, v_max=v_max, t_min=t_min)

    return spaced, durations
