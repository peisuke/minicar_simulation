"""JSON export functionality."""

import json
from pathlib import Path
from typing import List
from dataclasses import asdict
from .models import Course, GeneratorConfig


def export_circuit(
    outer_course: Course,
    shortcuts: List[Course],
    config: GeneratorConfig,
    output_path: str | Path
) -> None:
    """
    Export circuit data to JSON file.

    Args:
        outer_course: The main outer loop course
        shortcuts: List of shortcut courses
        config: Generator configuration used
        output_path: Path to output JSON file
    """
    data = {
        "metadata": _config_to_dict(config),
        "outer_course": outer_course.to_dict(),
        "shortcuts": [s.to_dict() for s in shortcuts],
    }

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _config_to_dict(config: GeneratorConfig) -> dict:
    """Convert config to JSON-serializable dict."""
    return {
        "ellipse": {
            "center_x": config.ellipse.center_x,
            "center_y": config.ellipse.center_y,
            "semi_major": config.ellipse.semi_major,
            "semi_minor": config.ellipse.semi_minor,
            "rotation": config.ellipse.rotation,
        },
        "waypoint_settings": {
            "min_count": config.min_waypoint_count,
            "max_count": config.max_waypoint_count,
            "min_interval": config.min_waypoint_interval,
            "max_interval": config.max_waypoint_interval,
        },
        "offset_settings": {
            "min_offset": config.min_offset,
            "max_offset": config.max_offset,
            "shortcut_min_offset": config.shortcut_min_offset,
            "shortcut_max_offset": config.shortcut_max_offset,
        },
        "spline_resolution": config.spline_resolution,
        "seed": config.seed,
    }
