#!/usr/bin/env python3
"""Generate Gazebo course from PNG image.

Usage:
    python3 generate_course_from_image.py --input input/map.png
    python3 generate_course_from_image.py --input input/map.png --scale 0.02 --wall-height 0.3
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import Delaunay


SCRIPT_DIR = Path(__file__).resolve().parent
PKG_ROOT = SCRIPT_DIR.parent
DEFAULT_INPUT = PKG_ROOT / "input" / "map.png"
DEFAULT_MODELS_DIR = Path("/tmp/minicar_simulation/models")
DEFAULT_OUTPUT_DIR = Path("/tmp/minicar_simulation/output")


def load_and_preprocess_image(image_path: Path, threshold: int = 128):
    """Load image and extract black lines as binary mask."""
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Cannot load image: {image_path}")

    # Black lines -> white mask (invert)
    _, binary = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY_INV)
    return img, binary


def interpolate_contour(points, max_dist=10):
    """Add points along contour edges."""
    result = []
    n = len(points)
    for i in range(n):
        p0 = points[i]
        p1 = points[(i + 1) % n]
        result.append(p0)

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > max_dist:
            num_segments = int(math.ceil(dist / max_dist))
            for j in range(1, num_segments):
                t = j / num_segments
                px = int(p0[0] + t * dx)
                py = int(p0[1] + t * dy)
                result.append((px, py))
    return result


def extract_contour_pairs(binary, approx_eps=3.0, interpolate_dist=10):
    """Extract outer-inner contour pairs from binary mask."""
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if hierarchy is None:
        return [], []

    hierarchy = hierarchy[0]
    pairs = []
    all_contours = []

    for i, h in enumerate(hierarchy):
        # Find outer contours (no parent) that have children (holes)
        if h[3] == -1 and h[2] != -1:
            outer = cv2.approxPolyDP(contours[i], epsilon=approx_eps, closed=True)
            outer_pts = [(int(p[0][0]), int(p[0][1])) for p in outer]
            outer_pts_interp = interpolate_contour(outer_pts, max_dist=interpolate_dist)
            all_contours.append(outer_pts)

            # Get all children (holes)
            child_idx = h[2]
            inner_pts = []
            inner_pts_interp = []
            while child_idx != -1:
                inner = cv2.approxPolyDP(contours[child_idx], epsilon=approx_eps, closed=True)
                pts = [(int(p[0][0]), int(p[0][1])) for p in inner]
                inner_pts.extend(pts)
                inner_pts_interp.extend(interpolate_contour(pts, max_dist=interpolate_dist))
                all_contours.append(pts)
                child_idx = hierarchy[child_idx][0]

            if len(outer_pts_interp) >= 3 and len(inner_pts_interp) >= 3:
                pairs.append((outer_pts_interp, inner_pts_interp))

    return pairs, all_contours


def triangle_max_edge(p0, p1, p2):
    """Calculate max edge length of triangle."""
    d01 = math.sqrt((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2)
    d12 = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
    d20 = math.sqrt((p0[0] - p2[0]) ** 2 + (p0[1] - p2[1]) ** 2)
    return max(d01, d12, d20)


def create_top_faces(pairs, binary, wall_height_m, meters_per_pixel, img_shape, max_edge_px=30, z0=0.0):
    """Create top face triangles using Delaunay triangulation."""
    H, W = img_shape[:2]

    def px_to_m(p):
        cx, cy = W / 2.0, H / 2.0
        return ((p[0] - cx) * meters_per_pixel, -(p[1] - cy) * meters_per_pixel)

    triangles = []
    z_top = z0 + wall_height_m

    for outer_pts, inner_pts in pairs:
        all_pts = list(set(outer_pts + inner_pts))
        if len(all_pts) < 3:
            continue

        pts_array = np.array(all_pts, dtype=np.float64)

        try:
            tri = Delaunay(pts_array)
        except Exception:
            continue

        for simplex in tri.simplices:
            p0 = pts_array[simplex[0]]
            p1 = pts_array[simplex[1]]
            p2 = pts_array[simplex[2]]

            # Check centroid is on wall
            cx = int((p0[0] + p1[0] + p2[0]) / 3)
            cy = int((p0[1] + p1[1] + p2[1]) / 3)

            if not (0 <= cx < W and 0 <= cy < H and binary[cy, cx] > 0):
                continue

            # Check max edge length
            max_edge = triangle_max_edge(p0, p1, p2)
            if max_edge > max_edge_px:
                continue

            # Convert to meters
            v0 = px_to_m(p0)
            v1 = px_to_m(p1)
            v2 = px_to_m(p2)

            triangles.append((
                (v0[0], v0[1], z_top),
                (v1[0], v1[1], z_top),
                (v2[0], v2[1], z_top)
            ))

    return triangles


def create_side_faces(contours, wall_height_m, meters_per_pixel, img_shape, z0=0.0):
    """Create side faces by extruding contours."""
    H, W = img_shape[:2]

    def px_to_m(p):
        cx, cy = W / 2.0, H / 2.0
        return ((p[0] - cx) * meters_per_pixel, -(p[1] - cy) * meters_per_pixel)

    triangles = []

    for contour in contours:
        n = len(contour)
        for i in range(n):
            p0 = contour[i]
            p1 = contour[(i + 1) % n]

            a = px_to_m(p0)
            b = px_to_m(p1)

            a_bot = (a[0], a[1], z0)
            a_top = (a[0], a[1], z0 + wall_height_m)
            b_bot = (b[0], b[1], z0)
            b_top = (b[0], b[1], z0 + wall_height_m)

            triangles.append((a_bot, b_bot, b_top))
            triangles.append((a_bot, b_top, a_top))

    return triangles


def make_double_sided(triangles):
    """Make triangles double-sided."""
    return [(a, b, c) for a, b, c in triangles] + [(a, c, b) for a, b, c in triangles]


def write_stl(path: Path, triangles, solid_name="walls"):
    """Write triangles to ASCII STL file."""
    def normal(tri):
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = tri
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        L = math.sqrt(nx * nx + ny * ny + nz * nz) + 1e-12
        return nx / L, ny / L, nz / L

    path.parent.mkdir(parents=True, exist_ok=True)
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


def export_gazebo_model(triangles, model_name, models_dir: Path):
    """Export triangles as Gazebo model."""
    model_dir = models_dir / model_name
    mesh_dir = model_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    mesh_filename = "walls.stl"
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
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.15 0.15 0.15 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")
    return model_dir


def find_spawn_position(binary_mask, meters_per_pixel):
    """Find a good spawn position (center of largest open area)."""
    H, W = binary_mask.shape
    cx, cy = W / 2.0, H / 2.0

    road_mask = cv2.bitwise_not(binary_mask)
    dist = cv2.distanceTransform(road_mask, cv2.DIST_L2, 5)
    _, max_val, _, max_loc = cv2.minMaxLoc(dist)

    if max_val < 5:
        return {"x": 0.0, "y": 0.0, "z": 0.05, "yaw": 0.0}

    px, py = max_loc
    x = (px - cx) * meters_per_pixel
    y = -(py - cy) * meters_per_pixel

    return {"x": float(x), "y": float(y), "z": 0.05, "yaw": 0.0}


def save_debug_image(original, binary, contours, output_path: Path):
    """Save debug visualization."""
    H, W = original.shape[:2]
    debug = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    for contour in contours:
        pts = np.array(contour, np.int32).reshape(-1, 1, 2)
        cv2.polylines(debug, [pts], True, (0, 255, 0), 1)
        for p in contour:
            cv2.circle(debug, p, 2, (0, 0, 255), -1)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), debug)
    return output_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate Gazebo course from PNG image"
    )
    parser.add_argument(
        "--input", "-i",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"Input PNG image path (default: {DEFAULT_INPUT})"
    )
    parser.add_argument(
        "--models-dir",
        type=Path,
        default=DEFAULT_MODELS_DIR,
        help=f"Output models directory (default: {DEFAULT_MODELS_DIR})"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Output directory for spawn pose etc (default: {DEFAULT_OUTPUT_DIR})"
    )
    parser.add_argument(
        "--model-name",
        type=str,
        default="road_env",
        help="Gazebo model name (default: road_env)"
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=0.02,
        help="Meters per pixel (default: 0.02, i.e., 700px = 14m)"
    )
    parser.add_argument(
        "--wall-height",
        type=float,
        default=0.3,
        help="Wall height in meters (default: 0.3)"
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=128,
        help="Binary threshold for line detection (default: 128)"
    )
    parser.add_argument(
        "--max-edge",
        type=int,
        default=30,
        help="Max triangle edge length in pixels for top faces (default: 30)"
    )
    parser.add_argument(
        "--rotate",
        type=float,
        default=0.0,
        help="Rotate image by degrees (counter-clockwise, default: 0)"
    )
    return parser.parse_args()


def rotate_image(img, angle_deg):
    """Rotate image by angle (counter-clockwise). Expands canvas to fit."""
    if angle_deg == 0:
        return img
    h, w = img.shape[:2]
    center = (w / 2, h / 2)
    M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    cos = abs(M[0, 0])
    sin = abs(M[0, 1])
    new_w = int(h * sin + w * cos)
    new_h = int(h * cos + w * sin)
    M[0, 2] += (new_w - w) / 2
    M[1, 2] += (new_h - h) / 2
    return cv2.warpAffine(img, M, (new_w, new_h), borderValue=255)


def main():
    args = parse_args()

    print(f"Loading image: {args.input}")
    original, binary = load_and_preprocess_image(args.input, args.threshold)

    # Rotate if specified
    if args.rotate != 0:
        print(f"  Rotating by {args.rotate} degrees")
        original = rotate_image(original, args.rotate)
        binary = rotate_image(binary, args.rotate)
        # Re-threshold after rotation to clean up interpolation artifacts
        _, binary = cv2.threshold(binary, 128, 255, cv2.THRESH_BINARY)

    H, W = original.shape[:2]
    print(f"  Image size: {W}x{H} pixels")
    print(f"  Scale: {args.scale} m/px -> {W * args.scale:.1f}m x {H * args.scale:.1f}m")

    # Extract contour pairs (outer + inner)
    pairs, all_contours = extract_contour_pairs(binary, approx_eps=3.0, interpolate_dist=10)
    print(f"  Found {len(pairs)} contour pairs, {len(all_contours)} total contours")

    # Create side faces
    side_triangles = create_side_faces(
        all_contours,
        wall_height_m=args.wall_height,
        meters_per_pixel=args.scale,
        img_shape=original.shape,
    )
    print(f"  Side faces: {len(side_triangles)} triangles")

    # Create top faces
    top_triangles = create_top_faces(
        pairs,
        binary,
        wall_height_m=args.wall_height,
        meters_per_pixel=args.scale,
        img_shape=original.shape,
        max_edge_px=args.max_edge,
    )
    print(f"  Top faces: {len(top_triangles)} triangles")

    # Combine and make double-sided
    triangles = side_triangles + top_triangles
    triangles = make_double_sided(triangles)
    print(f"  Total triangles (double-sided): {len(triangles)}")

    # Export Gazebo model
    model_dir = export_gazebo_model(triangles, args.model_name, args.models_dir)
    print(f"  Gazebo model: {model_dir}")

    # Find spawn position
    spawn_pose = find_spawn_position(binary, args.scale)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    spawn_pose_path = args.output_dir / "spawn_pose.json"
    with open(spawn_pose_path, "w") as f:
        json.dump(spawn_pose, f, indent=2)
    print(f"  Spawn pose: {spawn_pose_path}")
    print(f"    x={spawn_pose['x']:.3f}, y={spawn_pose['y']:.3f}, yaw={spawn_pose['yaw']:.3f}")

    # Save debug image
    debug_path = args.output_dir / "debug_walls.png"
    save_debug_image(original, binary, all_contours, debug_path)
    print(f"  Debug image: {debug_path}")

    print("\nDone! To run simulation:")
    print("  ros2 launch minicar_simulation road_env_ackermann.launch.py generate_course:=false")


if __name__ == "__main__":
    main()
