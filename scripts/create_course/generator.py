"""Circuit course generator."""

import numpy as np
from typing import List, Tuple, Optional
from .models import WayPoint, Course, GeneratorConfig
from .ellipse import Ellipse
from .spline import catmull_rom_spline, catmull_rom_open


def _angular_distance(theta1: float, theta2: float) -> float:
    """Calculate the minimum angular distance on a circle."""
    diff = abs(theta1 - theta2)
    return min(diff, 2 * np.pi - diff)


def _segments_intersect(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> bool:
    """Check if line segment (p1-p2) intersects with line segment (p3-p4)."""
    def cross(o: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)

    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _curves_intersect(curve1: np.ndarray, curve2: np.ndarray, skip_endpoints: int = 5) -> bool:
    """Check if two curves intersect, skipping endpoints to avoid false positives."""
    if len(curve1) < 2 or len(curve2) < 2:
        return False

    start_idx = skip_endpoints
    end_idx = len(curve2) - 1 - skip_endpoints
    if start_idx >= end_idx:
        return False

    for i in range(len(curve1) - 1):
        for j in range(start_idx, end_idx):
            if _segments_intersect(curve1[i], curve1[i + 1], curve2[j], curve2[j + 1]):
                return True
    return False


def _curve_length(curve: np.ndarray) -> float:
    """Calculate total length of a curve."""
    if len(curve) < 2:
        return 0.0
    diffs = np.diff(curve, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


class CircuitGenerator:
    """Generates circuit courses with outer loop and shortcuts."""

    # Shortcut waypoint count scaling constants
    MIN_ANGLE = np.pi / 4   # 45°
    MAX_ANGLE = np.pi       # 180°
    MIN_WAYPOINT_COUNT = 3
    MAX_WAYPOINT_COUNT = 7

    def __init__(self, config: GeneratorConfig | None = None):
        self.config = config or GeneratorConfig()
        self._rng = np.random.default_rng(self.config.seed)
        self._ellipse = Ellipse(self.config.ellipse)
        self._outer_waypoints: List[WayPoint] = []
        self._outer_thetas: np.ndarray = np.array([])
        self._outer_curve_points: np.ndarray = np.array([])

    def generate(self) -> Tuple[Course, List[Course]]:
        """Generate the complete circuit."""
        # Retry entire generation if shortcuts fail
        for _ in range(50):
            outer_course = self._generate_outer_course()
            shortcuts = self._generate_shortcuts()
            if len(shortcuts) == 2:
                return outer_course, shortcuts
        return outer_course, shortcuts

    def _generate_outer_course(self) -> Course:
        """Generate the outer (main) loop course."""
        thetas = self._generate_waypoint_angles()
        self._outer_thetas = thetas

        waypoints = []
        for theta in thetas:
            base_point = self._ellipse.point_at(theta)
            normal = self._ellipse.normal_at(theta)
            offset = self._rng.uniform(self.config.min_offset, self.config.max_offset)
            waypoints.append(WayPoint.from_array(base_point + normal * offset))

        self._outer_waypoints = waypoints

        curve_points = catmull_rom_spline(
            waypoints,
            n_points_per_segment=self.config.spline_resolution,
            closed=True
        )
        self._outer_curve_points = curve_points

        return Course(name="outer", waypoints=waypoints, curve_points=curve_points, is_closed=True)

    def _generate_waypoint_angles(self) -> np.ndarray:
        """Generate random angles for waypoint placement."""
        cfg = self.config
        angles = [0.0]
        current = 0.0

        while current < 2 * np.pi:
            interval = self._rng.uniform(cfg.min_waypoint_interval, cfg.max_waypoint_interval)
            current += interval
            if current < 2 * np.pi - cfg.min_waypoint_interval:
                angles.append(current)

        if len(angles) < cfg.min_waypoint_count:
            n_add = cfg.min_waypoint_count - len(angles)
            additional = np.linspace(0, 2 * np.pi, n_add + 2)[1:-1]
            angles = sorted(set(angles) | set(additional.tolist()))

        if len(angles) > cfg.max_waypoint_count:
            indices = self._rng.choice(len(angles), size=cfg.max_waypoint_count, replace=False)
            angles = [angles[i] for i in sorted(indices)]

        return np.array(angles)

    def _generate_shortcuts(self) -> List[Course]:
        """Generate two shortcut courses with sufficient distance savings."""
        if len(self._outer_waypoints) < 4:
            return []

        indices = self._select_well_spaced_indices(count=4, min_angle=self.MIN_ANGLE)
        if indices is None:
            return []

        pair1, pair2 = self._find_non_crossing_pairs(indices)

        shortcut1 = self._generate_single_shortcut(pair1[0], pair1[1], "shortcut_1")
        if shortcut1 is None:
            return []

        shortcut2 = self._generate_single_shortcut(pair2[0], pair2[1], "shortcut_2")
        if shortcut2 is None:
            return []

        return [shortcut1, shortcut2]

    def _select_well_spaced_indices(self, count: int, min_angle: float, max_attempts: int = 100) -> Optional[np.ndarray]:
        """Select waypoint indices with sufficient angular separation."""
        n = len(self._outer_thetas)
        if n < count:
            return None

        for _ in range(max_attempts):
            indices = self._rng.choice(n, size=count, replace=False)
            if self._indices_well_spaced(indices, min_angle):
                return indices
        return None

    def _indices_well_spaced(self, indices: np.ndarray, min_angle: float) -> bool:
        """Check if all pairs of indices have sufficient angular separation."""
        for i in range(len(indices)):
            for j in range(i + 1, len(indices)):
                if _angular_distance(self._outer_thetas[indices[i]], self._outer_thetas[indices[j]]) < min_angle:
                    return False
        return True

    def _find_non_crossing_pairs(self, indices: np.ndarray) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        """Find pairing that doesn't cross and maximizes shortcut effectiveness."""
        # Sort indices by their position on outer course
        sorted_indices = np.sort(indices)

        # For 4 points in order [0,1,2,3] on a circle:
        # - (0,1)+(2,3): adjacent pairs, short shortcuts, don't cross
        # - (0,2)+(1,3): cross each other
        # - (0,3)+(1,2): opposite pairs, long shortcuts, don't cross
        # Choose (0,3) and (1,2) for effective shortcuts without crossing
        return (int(sorted_indices[0]), int(sorted_indices[3])), (int(sorted_indices[1]), int(sorted_indices[2]))

    def _calculate_shortcut_waypoint_count(self, idx1: int, idx2: int) -> int:
        """Calculate waypoint count based on angular distance between endpoints."""
        angle_diff = _angular_distance(self._outer_thetas[idx1], self._outer_thetas[idx2])
        angle_clamped = np.clip(angle_diff, self.MIN_ANGLE, self.MAX_ANGLE)
        ratio = (angle_clamped - self.MIN_ANGLE) / (self.MAX_ANGLE - self.MIN_ANGLE)
        return int(self.MIN_WAYPOINT_COUNT + ratio * (self.MAX_WAYPOINT_COUNT - self.MIN_WAYPOINT_COUNT))

    def _get_outer_segment_length(self, idx1: int, idx2: int) -> float:
        """Calculate the length of outer course between two waypoint indices (shorter path)."""
        n_waypoints = len(self._outer_waypoints)
        resolution = self.config.spline_resolution

        # Calculate curve point indices
        start_curve_idx = idx1 * resolution
        end_curve_idx = idx2 * resolution

        # Get the shorter path
        if start_curve_idx <= end_curve_idx:
            forward_dist = _curve_length(self._outer_curve_points[start_curve_idx:end_curve_idx + 1])
            backward_dist = (_curve_length(self._outer_curve_points[:start_curve_idx + 1]) +
                           _curve_length(self._outer_curve_points[end_curve_idx:]))
        else:
            forward_dist = _curve_length(self._outer_curve_points[end_curve_idx:start_curve_idx + 1])
            backward_dist = (_curve_length(self._outer_curve_points[:end_curve_idx + 1]) +
                           _curve_length(self._outer_curve_points[start_curve_idx:]))

        return min(forward_dist, backward_dist)

    def _generate_single_shortcut(self, idx1: int, idx2: int, name: str) -> Optional[Course]:
        """Generate a single shortcut between two waypoints, ensuring sufficient distance savings."""
        cfg = self.config
        start = self._outer_waypoints[idx1].to_array()
        end = self._outer_waypoints[idx2].to_array()

        direction = end - start
        length = np.linalg.norm(direction)
        if length < 1e-10:
            return Course(name=name, waypoints=[], curve_points=np.array([]), is_closed=False)

        direction_norm = direction / length
        normal = np.array([-direction_norm[1], direction_norm[0]])

        # Point normal toward ellipse center (inside the track)
        ellipse_center = np.array([cfg.ellipse.center_x, cfg.ellipse.center_y])
        if np.dot(normal, ellipse_center - (start + end) / 2) < 0:
            normal = -normal

        waypoint_count = self._calculate_shortcut_waypoint_count(idx1, idx2)
        outer_segment_length = self._get_outer_segment_length(idx1, idx2)

        # Minimum required shortcut ratio (shortcut should be at most 90% of outer path)
        max_shortcut_ratio = 0.9

        # Generate shortcut with random offset from config (positive = toward center)
        max_offset = max(abs(cfg.shortcut_min_offset), abs(cfg.shortcut_max_offset))
        offset = self._rng.uniform(0.1 * max_offset, max_offset)

        waypoints = [WayPoint.from_array(start)]
        n_inner = waypoint_count - 2
        if n_inner > 0:
            for i in range(n_inner):
                t = (i + 1) / (n_inner + 1)
                pt = start + direction * t + normal * offset
                waypoints.append(WayPoint.from_array(pt))
        waypoints.append(WayPoint.from_array(end))

        curve_points = catmull_rom_open(waypoints, n_points_per_segment=self.config.spline_resolution)

        # Check for collision with outer course
        if len(self._outer_curve_points) > 0 and _curves_intersect(self._outer_curve_points, curve_points):
            return None

        # Check if shortcut provides sufficient distance savings
        shortcut_length = _curve_length(curve_points)
        ratio = shortcut_length / outer_segment_length if outer_segment_length > 0 else 1.0

        if ratio <= max_shortcut_ratio:
            return Course(name=name, waypoints=waypoints, curve_points=curve_points, is_closed=False)

        return None
