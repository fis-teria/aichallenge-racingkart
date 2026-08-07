#!/usr/bin/env python3
"""Generate a closed centerline from final_ver3's production occupancy map.

The input trajectory supplies the forward station order and its speed semantics.
At every station, the generator ray-casts the local normal to the first occupied
or out-of-map cell on both sides, uses their midpoint, and smooths it only when
the resulting nominal vehicle footprint remains free.
``wide_lanelet2_map.osm`` is checked only as lanelet-topology provenance; its
centerline geometry is deliberately not used as a runtime reference.

The companion ``course_centerline_overtake_permission.csv`` has the same
all-true index partitions as the current profile.  That semantic equivalence
holds only while this generator preserves the 350-station order; runtime CSV
parsing keeps the existing fail-closed index-bound checks active.
"""

from __future__ import annotations

import argparse
import csv
import math
import xml.etree.ElementTree as element_tree
from dataclasses import dataclass
from pathlib import Path


LANELET_ORDER = ("14", "9169", "1483", "9118", "9123", "9128", "9134", "9145", "2256", "2263", "9151", "9157", "9163")
FIELDNAMES = ("s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2")
EPSILON_M = 1.0e-6
# Production nominal footprint: wheel_base + front_overhang, rear_overhang,
# left_extent, right_extent. Keep these synchronized with the active YAML.
FRONT_M, REAR_M, LEFT_M, RIGHT_M = 1.087 + 0.467, 0.510, 0.650, 0.650


@dataclass(frozen=True)
class OccupancyMap:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    occupied: bytes

    def is_occupied(self, x_m: float, y_m: float) -> bool:
        x = math.floor((x_m - self.origin_x_m) / self.resolution_m)
        y = math.floor((y_m - self.origin_y_m) / self.resolution_m)
        return x < 0 or y < 0 or x >= self.width or y >= self.height or bool(self.occupied[y * self.width + x])


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    """Read the P5/P2 formats accepted by the production GridMap loader."""
    with path.open("rb") as stream:
        def token() -> bytes:
            value = bytearray()
            while True:
                byte = stream.read(1)
                if not byte:
                    raise ValueError(f"truncated PGM header: {path}")
                if byte == b"#":
                    stream.readline()
                elif not byte.isspace():
                    value.extend(byte)
                    break
            while True:
                byte = stream.read(1)
                if not byte or byte.isspace():
                    return bytes(value)
                value.extend(byte)

        magic, width, height, max_value = token(), int(token()), int(token()), int(token())
        if magic not in (b"P5", b"P2") or not 0 < max_value <= 255:
            raise ValueError(f"unsupported PGM format: {path}")
        if magic == b"P5":
            pixels = stream.read(width * height)
        else:
            pixels = bytes(int(token()) for _ in range(width * height))
    if len(pixels) != width * height:
        raise ValueError(f"truncated PGM pixels: {path}")
    return width, height, pixels


def remove_small_occupied_components(occupied: bytearray, width: int, height: int) -> None:
    """Match GridMap::readPgm's eight-connected five-cell noise filtering."""
    visited = bytearray(width * height)
    for start_y in range(height):
        for start_x in range(width):
            start = start_y * width + start_x
            if not occupied[start] or visited[start]:
                continue
            component, queue = [], [(start_x, start_y)]
            visited[start] = 1
            while queue:
                x, y = queue.pop()
                component.append(y * width + x)
                for delta_y in (-1, 0, 1):
                    for delta_x in (-1, 0, 1):
                        next_x, next_y = x + delta_x, y + delta_y
                        if (delta_x == 0 and delta_y == 0 or next_x < 0 or next_y < 0 or
                                next_x >= width or next_y >= height):
                            continue
                        next_index = next_y * width + next_x
                        if occupied[next_index] and not visited[next_index]:
                            visited[next_index] = 1
                            queue.append((next_x, next_y))
            if len(component) < 5:
                for index in component:
                    occupied[index] = 0


def load_occupancy(yaml_path: Path) -> OccupancyMap:
    values: dict[str, str] = {}
    origin: list[float] = []
    for raw_line in yaml_path.read_text().splitlines():
        line = raw_line.strip()
        if line.startswith("-") and len(origin) < 3:
            origin.append(float(line[1:].strip()))
        elif ":" in line:
            key, value = (part.strip() for part in line.split(":", 1))
            values[key] = value.strip('"')
    if len(origin) < 2:
        raise ValueError(f"map origin is incomplete: {yaml_path}")
    width, height, pixels = read_pgm(yaml_path.parent / values["image"])
    occupied_threshold = float(values["occupied_thresh"])
    negate = values.get("negate", "0") not in ("0", "false", "False")
    # Match GridMap::readPgm: image rows are vertically flipped into map cells.
    occupied = bytearray(width * height)
    for image_y in range(height):
        for x in range(width):
            normalized = pixels[image_y * width + x] / 255.0
            probability = normalized if negate else 1.0 - normalized
            occupied[(height - 1 - image_y) * width + x] = probability > occupied_threshold
    remove_small_occupied_components(occupied, width, height)
    return OccupancyMap(width, height, float(values["resolution"]), origin[0], origin[1], bytes(occupied))


def load_reference(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as stream:
        rows = [{name: float(row[name]) for name in FIELDNAMES} for row in csv.DictReader(stream)]
    if len(rows) < 3 or any(not math.isfinite(value) for row in rows for value in row.values()):
        raise ValueError(f"reference CSV {path} must contain at least three finite rows")
    return rows


def footprint_is_free(occupancy: OccupancyMap, x_m: float, y_m: float, psi_rad: float) -> bool:
    """Conservatively raster-sample the complete nominal rectangular footprint."""
    spacing_m = occupancy.resolution_m / 2.0
    cos_yaw, sin_yaw = math.cos(psi_rad), math.sin(psi_rad)
    longitudinal_samples = math.ceil((FRONT_M + REAR_M) / spacing_m)
    lateral_samples = math.ceil((LEFT_M + RIGHT_M) / spacing_m)
    for i in range(longitudinal_samples + 1):
        longitudinal_m = -REAR_M + (FRONT_M + REAR_M) * i / longitudinal_samples
        for j in range(lateral_samples + 1):
            lateral_m = -RIGHT_M + (LEFT_M + RIGHT_M) * j / lateral_samples
            if occupancy.is_occupied(x_m + cos_yaw * longitudinal_m - sin_yaw * lateral_m,
                                     y_m + sin_yaw * longitudinal_m + cos_yaw * lateral_m):
                return False
    return True


def normal_clearance(occupancy: OccupancyMap, row: dict[str, float], sign: float) -> float:
    normal_x, normal_y = -math.sin(row["psi_rad"]) * sign, math.cos(row["psi_rad"]) * sign
    step_m, distance_m = occupancy.resolution_m / 2.0, 0.0
    while not occupancy.is_occupied(row["x_m"] + normal_x * (distance_m + step_m), row["y_m"] + normal_y * (distance_m + step_m)):
        distance_m += step_m
        if distance_m > 30.0:
            raise ValueError("normal ray did not reach a wall or map boundary")
    return distance_m


def centered_points(reference: list[dict[str, float]], occupancy: OccupancyMap) -> list[tuple[float, float]]:
    raw_shifts = []
    for row in reference:
        if not footprint_is_free(occupancy, row["x_m"], row["y_m"], row["psi_rad"]):
            raise ValueError(f"input reference footprint is occupied at s={row['s_m']:.3f} m")
        left_m, right_m = normal_clearance(occupancy, row, 1.0), normal_clearance(occupancy, row, -1.0)
        # The detected free interval, rather than an arbitrary fixed shift
        # limit, bounds the midpoint. Footprint validation below is the final
        # no-wall-crossing gate for the smoothed and raw candidates.
        raw_shifts.append((left_m - right_m) / 2.0)

    points = []
    for index, row in enumerate(reference):
        # One cyclic [1, 2, 1]/4 pass avoids discontinuous station-to-station shifts.
        shift_m = (raw_shifts[index - 1] + 2.0 * raw_shifts[index] + raw_shifts[(index + 1) % len(reference)]) / 4.0
        candidates = (shift_m, raw_shifts[index], 0.0)
        normal_x, normal_y = -math.sin(row["psi_rad"]), math.cos(row["psi_rad"])
        for candidate in candidates:
            x_m, y_m = row["x_m"] + candidate * normal_x, row["y_m"] + candidate * normal_y
            if footprint_is_free(occupancy, x_m, y_m, row["psi_rad"]):
                points.append((x_m, y_m))
                break
        else:
            raise ValueError(f"no footprint-safe bounded center at s={row['s_m']:.3f} m")
    return points


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def build_rows(points: list[tuple[float, float]], reference: list[dict[str, float]]) -> list[dict[str, float]]:
    segment_lengths = []
    segment_headings = []
    for index, point in enumerate(points):
        following = points[(index + 1) % len(points)]
        dx, dy = following[0] - point[0], following[1] - point[1]
        length_m = math.hypot(dx, dy)
        if length_m <= EPSILON_M:
            raise ValueError("duplicate or zero-length centerline segment")
        segment_lengths.append(length_m)
        segment_headings.append(math.atan2(dy, dx))

    rows, s_m = [], 0.0
    for index, point in enumerate(points):
        next_index = (index + 1) % len(points)
        rows.append({"s_m": s_m, "x_m": point[0], "y_m": point[1],
                     "psi_rad": segment_headings[index],
                     "kappa_radpm": wrap_angle(segment_headings[next_index] - segment_headings[index]) /
                                     segment_lengths[index],
                     "vx_mps": reference[index]["vx_mps"], "ax_mps2": reference[index]["ax_mps2"]})
        s_m += segment_lengths[index]
    return rows


def validate_topology(path: Path) -> None:
    root = element_tree.parse(path).getroot()
    ways = {way.attrib["id"]: way for way in root.findall("way")}
    last_node = None
    for relation_id in LANELET_ORDER:
        relation = root.find(f"relation[@id='{relation_id}']")
        centerline = None if relation is None else next((member for member in relation.findall("member") if member.attrib.get("role") == "centerline"), None)
        if centerline is None or centerline.attrib["ref"] not in ways:
            raise ValueError(f"missing provenance centerline for lanelet {relation_id}")
        nodes = [node.attrib["ref"] for node in ways[centerline.attrib["ref"]].findall("nd")]
        if len(nodes) < 2 or (last_node is not None and nodes[0] != last_node):
            raise ValueError(f"broken provenance topology at lanelet {relation_id}")
        last_node = nodes[-1]


def validate(rows: list[dict[str, float]], occupancy: OccupancyMap) -> None:
    if len(rows) < 3 or any(not math.isfinite(value) for row in rows for value in row.values()):
        raise ValueError("centerline must contain at least three finite rows")
    if any(later["s_m"] <= earlier["s_m"] for earlier, later in zip(rows, rows[1:])):
        raise ValueError("s_m is not strictly increasing")
    for first, second in zip(rows, rows[1:] + rows[:1]):
        if math.hypot(second["x_m"] - first["x_m"], second["y_m"] - first["y_m"]) <= EPSILON_M:
            raise ValueError("duplicate or zero-length centerline segment")
    for row in rows:
        if not footprint_is_free(occupancy, row["x_m"], row["y_m"], row["psi_rad"]):
            raise ValueError(f"generated footprint is occupied at s={row['s_m']:.3f} m")


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, default=package.parent / "multi_purpose_mpc_ros/env/final_ver3/occupancy_grid_map.yaml")
    parser.add_argument("--reference", type=Path, default=package.parent / "multi_purpose_mpc_ros/env/final_ver3/traj_mincurv_manual.csv")
    parser.add_argument("--osm-provenance", type=Path, default=package.parent / "aichallenge_submit_launch/map/wide_lanelet2_map.osm")
    parser.add_argument("--output", type=Path, default=package / "data/course_centerline.csv")
    args = parser.parse_args()
    validate_topology(args.osm_provenance)
    reference, occupancy = load_reference(args.reference), load_occupancy(args.map_yaml)
    rows = build_rows(centered_points(reference, occupancy), reference)
    validate(rows, occupancy)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows({name: f"{row[name]:.7f}" for name in FIELDNAMES} for row in rows)


if __name__ == "__main__":
    main()
