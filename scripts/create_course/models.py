"""Data models for circuit course generation."""

from dataclasses import dataclass, field
from typing import List
import numpy as np


@dataclass
class WayPoint:
    """A control point on the course."""
    x: float
    y: float

    def to_array(self) -> np.ndarray:
        """Convert to numpy array."""
        return np.array([self.x, self.y])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "WayPoint":
        """Create from numpy array."""
        return cls(x=float(arr[0]), y=float(arr[1]))

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {"x": self.x, "y": self.y}


@dataclass
class Course:
    """A course represented by waypoints and interpolated curve."""
    name: str
    waypoints: List[WayPoint] = field(default_factory=list)
    curve_points: np.ndarray = field(default_factory=lambda: np.array([]))
    is_closed: bool = True

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "name": self.name,
            "waypoints": [wp.to_dict() for wp in self.waypoints],
            "curve_points": self.curve_points.tolist() if self.curve_points.size > 0 else [],
            "is_closed": self.is_closed,
        }


@dataclass
class EllipseParams:
    """Parameters for the base ellipse."""
    center_x: float = 0.0
    center_y: float = 0.0
    semi_major: float = 3.0    # Long axis (meters) - RC car scale
    semi_minor: float = 1.8    # Short axis (meters)
    rotation: float = 0.0      # Rotation angle in radians


@dataclass
class GeneratorConfig:
    """Configuration for course generation."""
    # Ellipse parameters
    ellipse: EllipseParams = field(default_factory=EllipseParams)

    # Outer course waypoint settings
    min_waypoint_count: int = 12
    max_waypoint_count: int = 20
    min_waypoint_interval: float = 0.2  # Minimum angular interval (radians)
    max_waypoint_interval: float = 0.6  # Maximum angular interval (radians)

    # Push-out (offset) settings
    min_offset: float = -0.5   # Negative = inward (meters)
    max_offset: float = 0.5    # Positive = outward (meters)

    # Shortcut settings
    shortcut_min_offset: float = -0.3
    shortcut_max_offset: float = 0.3

    # Spline settings
    spline_resolution: int = 50  # Points per segment

    # Random seed (None for random)
    seed: int | None = None
