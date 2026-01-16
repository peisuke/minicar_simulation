"""Ellipse calculations for course generation."""

import numpy as np
from .models import EllipseParams


class Ellipse:
    """Represents a 2D ellipse with utility methods."""

    def __init__(self, params: EllipseParams):
        self.params = params

    def point_at(self, theta: float) -> np.ndarray:
        """
        Get point on ellipse at parameter angle theta.

        Args:
            theta: Parameter angle in radians (0 to 2*pi)

        Returns:
            2D point [x, y] on the ellipse
        """
        p = self.params
        # Point on unrotated ellipse
        x = p.semi_major * np.cos(theta)
        y = p.semi_minor * np.sin(theta)

        # Apply rotation
        cos_r = np.cos(p.rotation)
        sin_r = np.sin(p.rotation)
        x_rot = x * cos_r - y * sin_r
        y_rot = x * sin_r + y * cos_r

        # Translate to center
        return np.array([x_rot + p.center_x, y_rot + p.center_y])

    def normal_at(self, theta: float, outward: bool = True) -> np.ndarray:
        """
        Get unit normal vector at parameter angle theta.

        Args:
            theta: Parameter angle in radians
            outward: If True, points outward; otherwise inward

        Returns:
            Unit normal vector [nx, ny]
        """
        p = self.params
        # Gradient of ellipse at theta (tangent direction)
        # dx/dtheta = -a*sin(theta), dy/dtheta = b*cos(theta)
        tx = -p.semi_major * np.sin(theta)
        ty = p.semi_minor * np.cos(theta)

        # Normal is perpendicular to tangent (rotate 90 degrees)
        # For outward normal: rotate tangent clockwise
        nx = ty
        ny = -tx

        # Apply ellipse rotation
        cos_r = np.cos(p.rotation)
        sin_r = np.sin(p.rotation)
        nx_rot = nx * cos_r - ny * sin_r
        ny_rot = nx * sin_r + ny * cos_r

        # Normalize
        length = np.sqrt(nx_rot**2 + ny_rot**2)
        normal = np.array([nx_rot / length, ny_rot / length])

        if not outward:
            normal = -normal

        return normal

    def points_at_angles(self, thetas: np.ndarray) -> np.ndarray:
        """
        Get multiple points on ellipse.

        Args:
            thetas: Array of parameter angles

        Returns:
            Array of shape (n, 2) with points
        """
        return np.array([self.point_at(t) for t in thetas])

    def arc_length_approx(self, theta1: float, theta2: float, n_samples: int = 100) -> float:
        """
        Approximate arc length between two angles.

        Args:
            theta1: Start angle
            theta2: End angle
            n_samples: Number of samples for approximation

        Returns:
            Approximate arc length
        """
        thetas = np.linspace(theta1, theta2, n_samples)
        points = self.points_at_angles(thetas)
        diffs = np.diff(points, axis=0)
        return np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
