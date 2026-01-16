"""Main entry point for circuit course generation and Gazebo world export."""

import argparse
import json
import math
import sys
from pathlib import Path

# Add script directory to path for create_course module import
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import cv2
import numpy as np

from create_course.models import GeneratorConfig, EllipseParams, Course
from create_course.generator import CircuitGenerator
from create_course.exporter import export_circuit

# Directory setup
PKG_ROOT = SCRIPT_DIR.parent
OUTPUT_DIR = PKG_ROOT / "output"
WORLDS_DIR = PKG_ROOT / "worlds"
MODELS_DIR = PKG_ROOT / "models"


# =============================================================================
# Gazebo World Generation
# =============================================================================

def courses_to_routes(outer_course: Course, shortcuts: list) -> list:
    """Convert Course objects to routes format [(x, y), ...]."""
    routes = []
    routes.append([(p[0], p[1]) for p in outer_course.curve_points])
    for shortcut in shortcuts:
        routes.append([(p[0], p[1]) for p in shortcut.curve_points])
    return routes


def rasterize_centerlines(routes_xy_m, img_wh, meters_per_pixel, line_thickness_px=1):
    """Rasterize route centerlines to a mask image."""
    W, H = img_wh
    mask = np.zeros((H, W), dtype=np.uint8)

    def to_px(p_m):
        cx = W / 2.0
        cy = H / 2.0
        x = int(round(p_m[0] / meters_per_pixel + cx))
        y = int(round(p_m[1] / meters_per_pixel + cy))
        return (x, y)

    for r in routes_xy_m:
        if len(r) < 2:
            continue
        pts = np.array([to_px(p) for p in r], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(mask, [pts], isClosed=False, color=255,
                      thickness=line_thickness_px, lineType=cv2.LINE_AA)

    _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    return mask


def thicken_by_morphology_px(center_mask, road_width_px):
    """Thicken centerline mask by morphological dilation."""
    r_px = max(1, int(round(road_width_px / 2)))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_px + 1, 2 * r_px + 1))
    thick = cv2.dilate(center_mask, k, iterations=1)
    _, thick = cv2.threshold(thick, 127, 255, cv2.THRESH_BINARY)
    return thick


def mask_to_polygons(mask, approx_eps_px=2.0):
    """Extract polygons from mask image."""
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None or len(contours) == 0:
        return []

    hierarchy = hierarchy[0]
    polys = []
    used = set()

    def approx(cnt):
        return cv2.approxPolyDP(cnt, epsilon=float(approx_eps_px), closed=True)

    for i, h in enumerate(hierarchy):
        if h[3] != -1 or i in used:
            continue

        outer = [(int(p[0][0]), int(p[0][1])) for p in approx(contours[i])]
        holes = []
        child = h[2]
        while child != -1:
            holes.append([(int(p[0][0]), int(p[0][1])) for p in approx(contours[child])])
            used.add(child)
            child = hierarchy[child][0]

        used.add(i)
        polys.append({"outer": outer, "holes": holes})

    return polys


def offset_polygon_px(poly, offset_px):
    """Offset polygon vertices by given amount."""
    pts = np.array(poly, dtype=np.float32)
    n = len(pts)
    out = []

    for i in range(n):
        p_prev, p_curr, p_next = pts[i - 1], pts[i], pts[(i + 1) % n]
        v1, v2 = p_curr - p_prev, p_next - p_curr

        def norm(v):
            L = math.hypot(v[0], v[1]) + 1e-9
            return np.array([-v[1] / L, v[0] / L], dtype=np.float32)

        n_avg = norm(v1) + norm(v2)
        L = math.hypot(n_avg[0], n_avg[1])
        n_avg = norm(v1) if L < 1e-6 else n_avg / L
        out.append(p_curr + n_avg * offset_px)

    return [(float(p[0]), float(p[1])) for p in out]


def polygons_to_wall_triangles(polys, wall_height_m, wall_thickness_px,
                                meters_per_pixel, img_wh, z0=0.0):
    """Convert polygons to 3D wall mesh triangles."""
    half = wall_thickness_px / 2.0

    def px_to_m(p):
        cx, cy = img_wh[0] / 2.0, img_wh[1] / 2.0
        return ((p[0] - cx) * meters_per_pixel, (p[1] - cy) * meters_per_pixel)

    def ring_to_tris(ring_outer, ring_inner):
        tris = []
        n = len(ring_outer)
        for i in range(n):
            a0, b0 = ring_outer[i], ring_outer[(i + 1) % n]
            a1, b1 = ring_inner[i], ring_inner[(i + 1) % n]

            A0, B0 = (*px_to_m(a0), z0), (*px_to_m(b0), z0)
            A1, B1 = (*px_to_m(a1), z0), (*px_to_m(b1), z0)
            A0u, B0u = (A0[0], A0[1], z0 + wall_height_m), (B0[0], B0[1], z0 + wall_height_m)
            A1u, B1u = (A1[0], A1[1], z0 + wall_height_m), (B1[0], B1[1], z0 + wall_height_m)

            tris.extend([(A0, B0, B0u), (A0, B0u, A0u), (B1, A1, A1u), (B1, A1u, B1u),
                         (A0u, B0u, B1u), (A0u, B1u, A1u)])
        return tris

    triangles = []
    for p in polys:
        triangles.extend(ring_to_tris(offset_polygon_px(p["outer"], +half),
                                       offset_polygon_px(p["outer"], -half)))
        for h in p["holes"]:
            triangles.extend(ring_to_tris(offset_polygon_px(h, -half),
                                           offset_polygon_px(h, +half)))
    return triangles


def make_double_sided(triangles):
    """Make triangles double-sided."""
    return [(a, b, c) for a, b, c in triangles] + [(a, c, b) for a, b, c in triangles]


def routes_to_wall_triangles(routes_xy_m, road_width_m, wall_height_m,
                              wall_thickness_m, meters_per_pixel, img_wh):
    """Convert routes to wall mesh triangles."""
    road_width_px = road_width_m / meters_per_pixel
    wall_thickness_px = wall_thickness_m / meters_per_pixel

    center = rasterize_centerlines(routes_xy_m, img_wh, meters_per_pixel)
    thick = thicken_by_morphology_px(center, road_width_px)
    polys = mask_to_polygons(thick)

    triangles = polygons_to_wall_triangles(polys, wall_height_m, wall_thickness_px,
                                            meters_per_pixel, img_wh)
    return make_double_sided(triangles), polys


def write_stl(path, triangles, solid_name="mesh"):
    """Write triangles to ASCII STL file."""
    def normal(tri):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        L = math.sqrt(nx * nx + ny * ny + nz * nz) + 1e-12
        return nx / L, ny / L, nz / L

    with open(path, "w") as f:
        f.write(f"solid {solid_name}\n")
        for tri in triangles:
            nx, ny, nz = normal(tri)
            f.write(f"  facet normal {nx:.6e} {ny:.6e} {nz:.6e}\n")
            f.write("    outer loop\n")
            for x, y, z in tri:
                f.write(f"      vertex {x:.6e} {y:.6e} {z:.6e}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {solid_name}\n")


def save_course_image(routes_xy_m, output_path, img_wh=(3000, 2000),
                      meters_per_pixel=0.1, road_width_px=60):
    """Save course visualization as PNG with proper road width."""
    W, H = img_wh
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = (40, 40, 40)  # Dark gray background

    def to_px(p_m):
        cx, cy = W / 2.0, H / 2.0
        x = int(round(p_m[0] / meters_per_pixel + cx))
        y = int(round(-p_m[1] / meters_per_pixel + cy))  # Flip Y for image coords
        return (x, y)

    colors = [
        (255, 255, 255),  # Outer: white
        (0, 200, 255),    # Shortcut 1: yellow
        (255, 100, 100),  # Shortcut 2: light blue
    ]

    # Draw roads with proper width
    for i, route in enumerate(routes_xy_m):
        if len(route) < 2:
            continue
        pts = np.array([to_px(p) for p in route], dtype=np.int32)
        color = colors[i % len(colors)]
        cv2.polylines(img, [pts], isClosed=(i == 0), color=color,
                      thickness=road_width_px, lineType=cv2.LINE_AA)

    cv2.imwrite(str(output_path), img)
    return output_path


def export_gazebo_model(triangles, model_name, mesh_filename="walls.stl"):
    """Export triangles as Gazebo model to default MODELS_DIR."""
    return export_gazebo_model_to(triangles, model_name, MODELS_DIR, mesh_filename)


def export_gazebo_model_to(triangles, model_name, models_dir, mesh_filename="walls.stl"):
    """Export triangles as Gazebo model to specified directory."""
    model_dir = models_dir / model_name
    mesh_dir = model_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    write_stl(mesh_dir / mesh_filename, triangles)

    (model_dir / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
</model>
""")

    (model_dir / "model.sdf").write_text(f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{model_name}">
    <static>true</static>
    <link name="walls">
      <collision name="collision">
        <geometry><mesh><uri>model://{model_name}/meshes/{mesh_filename}</uri></mesh></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://{model_name}/meshes/{mesh_filename}</uri></mesh></geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")
    return model_dir


# =============================================================================
# Spawn Pose Calculation
# =============================================================================

def calculate_spawn_pose(curve_points: np.ndarray, position_ratio: float, z: float = 0.05):
    """
    Calculate spawn pose (x, y, z, yaw) from course centerline.

    Args:
        curve_points: Array of (x, y) points along the course centerline
        position_ratio: Position on course as ratio (0.0 to 1.0)
        z: Height above ground

    Returns:
        dict with x, y, z, yaw (in radians)
    """
    n_points = len(curve_points)
    if n_points < 2:
        return {"x": 0.0, "y": 0.0, "z": z, "yaw": 0.0}

    # Clamp ratio to valid range
    position_ratio = max(0.0, min(1.0, position_ratio))

    # Calculate index from ratio
    idx = int(position_ratio * (n_points - 1))
    idx = min(idx, n_points - 2)  # Ensure we have a next point

    # Get current and next point for direction calculation
    current = curve_points[idx]
    next_idx = (idx + 1) % n_points
    next_pt = curve_points[next_idx]

    # Calculate yaw from direction vector
    dx = next_pt[0] - current[0]
    dy = next_pt[1] - current[1]
    yaw = math.atan2(dy, dx)

    return {
        "x": float(current[0]),
        "y": float(current[1]),
        "z": z,
        "yaw": yaw
    }


def export_spawn_pose(spawn_pose: dict, output_path: Path):
    """Export spawn pose to JSON file."""
    with open(output_path, "w") as f:
        json.dump(spawn_pose, f, indent=2)


# =============================================================================
# Main
# =============================================================================

def parse_seed(value):
    """Parse seed value, returning None for empty string."""
    if value is None or value == "":
        return None
    return int(value)


def parse_start_position(value):
    """Parse start position value, returning 0.0 for empty string."""
    if value is None or value == "":
        return 0.0
    return float(value)


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Generate circuit course and export to JSON and Gazebo model."
    )
    parser.add_argument(
        "--seed",
        type=parse_seed,
        default=None,
        help="Random seed for reproducible course generation. If not specified, a random course is generated."
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Output directory for generated files. Defaults to package's output directory."
    )
    parser.add_argument(
        "--models-dir",
        type=str,
        default=None,
        help="Models directory for Gazebo model output. Defaults to package's models directory."
    )
    parser.add_argument(
        "--start-position",
        type=parse_start_position,
        default=0.0,
        help="Start position on course as ratio (0.0 to 1.0). Default: 0.0"
    )
    return parser.parse_args()


def main():
    """Generate circuit course and export to JSON and Gazebo model."""
    args = parse_args()

    # Use provided directories or defaults
    output_dir = Path(args.output_dir) if args.output_dir else OUTPUT_DIR
    models_dir = Path(args.models_dir) if args.models_dir else MODELS_DIR

    # Ensure directories exist
    output_dir.mkdir(parents=True, exist_ok=True)

    # Configure the generator
    # Scale adjusted to match original course size (~6m x 3m)
    config = GeneratorConfig(
        ellipse=EllipseParams(
            center_x=0.0,
            center_y=0.0,
            semi_major=3.0,       # 100.0 -> 3.0 (approx 1/33 scale)
            semi_minor=1.8,       # 60.0 -> 1.8
            rotation=0.0,
        ),
        min_waypoint_count=12,
        max_waypoint_count=20,
        min_waypoint_interval=0.2,
        max_waypoint_interval=0.6,
        min_offset=-0.5,          # -15.0 -> -0.5
        max_offset=0.5,           # 15.0 -> 0.5
        shortcut_min_offset=-0.3, # -8.0 -> -0.3
        shortcut_max_offset=0.3,  # 8.0 -> 0.3
        spline_resolution=50,
        seed=args.seed,
    )

    # Generate the circuit
    generator = CircuitGenerator(config)
    outer_course, shortcuts = generator.generate()

    # Export to JSON
    output_path = output_dir / "circuit.json"
    export_circuit(outer_course, shortcuts, config, output_path)

    print("Circuit generated successfully!")
    print(f"  Seed: {args.seed if args.seed is not None else 'random'}")
    print(f"  JSON: {output_path}")
    print(f"  Outer: {len(outer_course.waypoints)} waypoints, {len(outer_course.curve_points)} curve points")
    for sc in shortcuts:
        print(f"  {sc.name}: {len(sc.waypoints)} waypoints, {len(sc.curve_points)} curve points")

    # Generate Gazebo model
    # Parameters adjusted to match original code scale (1px = 1cm)
    routes = courses_to_routes(outer_course, shortcuts)
    triangles, polys = routes_to_wall_triangles(
        routes_xy_m=routes,
        road_width_m=0.6,         # 60px * 0.01 = 0.6m
        wall_height_m=0.3,
        wall_thickness_m=0.05,    # 5px * 0.01 = 0.05m
        meters_per_pixel=0.01,    # 0.1 -> 0.01 (1px = 1cm)
        img_wh=(800, 500),        # (3000, 2000) -> (800, 500)
    )

    model_path = export_gazebo_model_to(triangles, model_name="road_env", models_dir=models_dir)
    print(f"  Gazebo model: {model_path}")
    print(f"  Triangles: {len(triangles)}, Polygons: {len(polys)}")

    # Save course preview image
    image_path = output_dir / "circuit.png"
    save_course_image(routes, image_path, img_wh=(800, 500), meters_per_pixel=0.01)
    print(f"  Preview: {image_path}")

    # Calculate and export spawn pose
    spawn_pose = calculate_spawn_pose(
        outer_course.curve_points,
        args.start_position
    )
    spawn_pose_path = output_dir / "spawn_pose.json"
    export_spawn_pose(spawn_pose, spawn_pose_path)
    print(f"  Spawn pose: {spawn_pose_path}")
    print(f"    x={spawn_pose['x']:.3f}, y={spawn_pose['y']:.3f}, "
          f"z={spawn_pose['z']:.3f}, yaw={spawn_pose['yaw']:.3f} rad ({math.degrees(spawn_pose['yaw']):.1f}°)")


if __name__ == "__main__":
    main()
