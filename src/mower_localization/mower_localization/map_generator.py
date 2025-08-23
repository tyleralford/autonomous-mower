"""
Map generation utilities for Autonomous Mower Phase 3.

Reads recorded polygons (boundary, keepouts, optional travel areas) from CSV files
in world coordinates (meters, UTM frame) and generates a Nav2 static map as a
PGM grayscale image with corresponding YAML metadata.

Cost encoding (per PRD):
- Lethal (outside boundary and inside keepouts): black (0)
- Medium (travel areas): grey (128)
- Free (inside boundary excluding lethal and medium): white (255)

The YAML uses the 'map_server' format and sets origin at the lower-left corner
of the generated image in the UTM world frame (same frame as /odometry/filtered).
Resolution is configurable.
"""
from __future__ import annotations

import csv
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict

import numpy as np

try:
    import cv2  # type: ignore
except Exception as e:  # pragma: no cover - environment-specific
    raise RuntimeError(
        "OpenCV (cv2) is required for map generation. Install opencv-python."
    ) from e


FREE = 255
LETHAL = 0
MEDIUM = 128


@dataclass
class MapSpec:
    resolution: float = 0.05  # meters per pixel
    padding_m: float = 1.0    # border padding around polygons in meters


def _read_csv_points(path: str) -> np.ndarray:
    pts: List[Tuple[float, float]] = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if not {"x", "y"}.issubset(set(reader.fieldnames or [])):
            raise ValueError(f"CSV {path} missing x,y headers")
        for row in reader:
            try:
                x = float(row["x"])  # type: ignore[index]
                y = float(row["y"])  # type: ignore[index]
                pts.append((x, y))
            except Exception:
                continue
    if len(pts) < 3:
        raise ValueError(f"CSV {path} has insufficient points for polygon: {len(pts)}")
    # Optionally decimate duplicates
    arr = np.array(pts, dtype=np.float32)
    # Close polygon by ensuring last != first; cv2.fillPoly handles closure
    return arr


def _compute_bounds(polys: List[np.ndarray]) -> Tuple[float, float, float, float]:
    xs: List[float] = []
    ys: List[float] = []
    for p in polys:
        xs.extend(p[:, 0].tolist())
        ys.extend(p[:, 1].tolist())
    return min(xs), min(ys), max(xs), max(ys)


def _world_to_pixel_transform(
    min_x: float, min_y: float, max_y: float, resolution: float
):
    def w2p(points_xy: np.ndarray) -> np.ndarray:
        px = ((points_xy[:, 0] - min_x) / resolution).astype(np.int32)
        py = ((max_y - points_xy[:, 1]) / resolution).astype(np.int32)
        return np.stack([px, py], axis=1)

    return w2p


def _write_yaml(yaml_path: str, map_path: str, resolution: float, origin: Tuple[float, float, float]):
    # Minimal map_server YAML
    content = (
        f"image: {os.path.basename(map_path)}\n"
        f"mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    with open(yaml_path, "w") as f:
        f.write(content)


def _collect_zone_files(directory: str) -> Dict[str, List[str]]:
    files = os.listdir(directory)
    result: Dict[str, List[str]] = {"boundary": [], "keepout": [], "travel": []}
    for name in files:
        if not name.endswith(".csv"):
            continue
        lower = name.lower()
        path = os.path.join(directory, name)
        if lower.startswith("boundary"):
            result["boundary"].append(path)
        elif lower.startswith("keepout") or lower.startswith("keep_out"):
            result["keepout"].append(path)
        elif lower.startswith("travel"):
            result["travel"].append(path)
    return result


def generate_map(
    output_dir: str,
    map_spec: Optional[MapSpec] = None,
    boundary_files: Optional[List[str]] = None,
    keepout_files: Optional[List[str]] = None,
    travel_files: Optional[List[str]] = None,
    frame: str = "utm",
) -> Tuple[str, str]:
    """
    Generate map.pgm and map.yaml in output_dir.

    If file lists aren't provided, they are discovered in output_dir by prefix.

    Returns a tuple (map_yaml_path, map_pgm_path).
    frame: 'utm' (default) writes map.yaml origin in UTM. 'map' writes origin as 0,0,0 and saves
           the original UTM lower-left in map_origin_utm.yaml for transform publishing.
    """
    spec = map_spec or MapSpec()
    os.makedirs(output_dir, exist_ok=True)

    if boundary_files is None or keepout_files is None or travel_files is None:
        buckets = _collect_zone_files(output_dir)
        boundary_files = boundary_files or buckets["boundary"]
        keepout_files = keepout_files or buckets["keepout"]
        travel_files = travel_files or buckets["travel"]

    if not boundary_files:
        raise FileNotFoundError("No boundary CSV found. Expect files starting with 'boundary' in output_dir.")

    # Load polygons
    boundary_polys = [_read_csv_points(p) for p in boundary_files]
    # Use the outermost boundary: take the polygon with max area
    def poly_area(points: np.ndarray) -> float:
        x = points[:, 0]
        y = points[:, 1]
        return 0.5 * float(np.abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))

    outer_boundary = max(boundary_polys, key=poly_area)
    keepout_polys = []
    for p in (keepout_files or []):
        try:
            keepout_polys.append(_read_csv_points(p))
        except Exception:
            continue
    travel_polys = []
    for p in (travel_files or []):
        try:
            travel_polys.append(_read_csv_points(p))
        except Exception:
            continue

    # Compute bounds with padding
    min_x, min_y, max_x, max_y = _compute_bounds([outer_boundary])
    min_x -= spec.padding_m
    min_y -= spec.padding_m
    max_x += spec.padding_m
    max_y += spec.padding_m

    width_px = int(np.ceil((max_x - min_x) / spec.resolution))
    height_px = int(np.ceil((max_y - min_y) / spec.resolution))

    # Prepare transforms and blank image initialized to LETHAL (outside)
    to_px = _world_to_pixel_transform(min_x, min_y, max_y, spec.resolution)
    img = np.full((height_px, width_px), LETHAL, dtype=np.uint8)

    # Draw free space inside boundary
    boundary_px = to_px(outer_boundary)
    cv2.fillPoly(img, [boundary_px], color=FREE)

    # Draw keepouts as lethal inside boundary
    for kp in keepout_polys:
        cv2.fillPoly(img, [to_px(kp)], color=LETHAL)

    # Draw travel areas as medium cost
    for tp in travel_polys:
        cv2.fillPoly(img, [to_px(tp)], color=MEDIUM)

    # Write files
    map_pgm = os.path.join(output_dir, "map.pgm")
    map_yaml = os.path.join(output_dir, "map.yaml")
    # Use PGM writer via OpenCV
    if not cv2.imwrite(map_pgm, img):  # pragma: no cover
        raise RuntimeError("Failed to write map.pgm")

    utm_origin = (float(min_x), float(min_y), 0.0)
    if frame == "map":
        # Write map frame origin at (0,0,0)
        _write_yaml(map_yaml, map_pgm, spec.resolution, (0.0, 0.0, 0.0))
        # Persist utm origin companion file
        companion = os.path.join(output_dir, "map_origin_utm.yaml")
        with open(companion, "w") as f:
            f.write(f"utm_origin: [{utm_origin[0]:.6f}, {utm_origin[1]:.6f}, {utm_origin[2]:.6f}]\n")
    else:
        _write_yaml(map_yaml, map_pgm, spec.resolution, utm_origin)

    return map_yaml, map_pgm
