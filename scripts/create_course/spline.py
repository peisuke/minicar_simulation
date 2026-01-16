"""Catmull-Rom spline interpolation."""

import numpy as np
from typing import List
from .models import WayPoint


def catmull_rom_segment(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray,
                        n_points: int, alpha: float = 0.5) -> np.ndarray:
    """
    Compute points on a single Catmull-Rom spline segment.

    Uses centripetal parameterization (alpha=0.5) by default for
    better handling of sharp corners.

    Args:
        p0, p1, p2, p3: Four control points (segment goes from p1 to p2)
        n_points: Number of points to generate
        alpha: Parameterization (0=uniform, 0.5=centripetal, 1=chordal)

    Returns:
        Array of shape (n_points, 2) with interpolated points
    """
    def tj(ti: float, pi: np.ndarray, pj: np.ndarray) -> float:
        d = np.sqrt(np.sum((pj - pi) ** 2))
        return ti + d ** alpha

    t0 = 0.0
    t1 = tj(t0, p0, p1)
    t2 = tj(t1, p1, p2)
    t3 = tj(t2, p2, p3)

    # Generate points for t in [t1, t2]
    t = np.linspace(t1, t2, n_points)

    # Handle degenerate case
    if t2 - t1 < 1e-10:
        return np.tile(p1, (n_points, 1))

    # Catmull-Rom calculation
    A1 = (t1 - t[:, np.newaxis]) / (t1 - t0) * p0 + (t[:, np.newaxis] - t0) / (t1 - t0) * p1
    A2 = (t2 - t[:, np.newaxis]) / (t2 - t1) * p1 + (t[:, np.newaxis] - t1) / (t2 - t1) * p2
    A3 = (t3 - t[:, np.newaxis]) / (t3 - t2) * p2 + (t[:, np.newaxis] - t2) / (t3 - t2) * p3

    B1 = (t2 - t[:, np.newaxis]) / (t2 - t0) * A1 + (t[:, np.newaxis] - t0) / (t2 - t0) * A2
    B2 = (t3 - t[:, np.newaxis]) / (t3 - t1) * A2 + (t[:, np.newaxis] - t1) / (t3 - t1) * A3

    C = (t2 - t[:, np.newaxis]) / (t2 - t1) * B1 + (t[:, np.newaxis] - t1) / (t2 - t1) * B2

    return C


def catmull_rom_spline(waypoints: List[WayPoint], n_points_per_segment: int = 50,
                       closed: bool = True) -> np.ndarray:
    """
    Generate a smooth curve through waypoints using Catmull-Rom spline.

    Args:
        waypoints: List of WayPoint objects
        n_points_per_segment: Points to generate per segment
        closed: If True, create a closed loop

    Returns:
        Array of shape (n, 2) with curve points
    """
    if len(waypoints) < 2:
        if len(waypoints) == 1:
            return np.array([[waypoints[0].x, waypoints[0].y]])
        return np.array([])

    points = np.array([[wp.x, wp.y] for wp in waypoints])
    n = len(points)

    if closed:
        # Wrap around for closed curve
        points = np.vstack([points[-1], points, points[0], points[1]])
    else:
        # Extend endpoints for open curve
        p_start = 2 * points[0] - points[1]
        p_end = 2 * points[-1] - points[-2]
        points = np.vstack([p_start, points, p_end])

    curve_points = []
    n_segments = n if closed else n - 1

    for i in range(n_segments):
        p0 = points[i]
        p1 = points[i + 1]
        p2 = points[i + 2]
        p3 = points[i + 3]

        segment = catmull_rom_segment(p0, p1, p2, p3, n_points_per_segment)
        # Avoid duplicate points at segment boundaries
        if i < n_segments - 1:
            curve_points.append(segment[:-1])
        else:
            curve_points.append(segment)

    result = np.vstack(curve_points)

    # For closed curves, ensure exact closure
    if closed:
        result[-1] = result[0]

    return result


def catmull_rom_open(waypoints: List[WayPoint], n_points_per_segment: int = 50) -> np.ndarray:
    """
    Generate an open (non-looping) Catmull-Rom spline.

    Args:
        waypoints: List of WayPoint objects
        n_points_per_segment: Points to generate per segment

    Returns:
        Array of shape (n, 2) with curve points
    """
    return catmull_rom_spline(waypoints, n_points_per_segment, closed=False)
