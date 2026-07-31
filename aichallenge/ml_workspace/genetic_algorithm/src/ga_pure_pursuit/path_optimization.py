from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any


def split_genome(
    genome: dict[str, float], prefix: str = "path_offset_"
) -> tuple[dict[str, float], dict[str, float]]:
    controller = {name: value for name, value in genome.items() if not name.startswith(prefix)}
    path = {name: value for name, value in genome.items() if name.startswith(prefix)}
    return controller, path


def _periodic_interpolate(values: list[float], progress: float) -> float:
    position = (progress % 1.0) * len(values)
    lower = int(math.floor(position)) % len(values)
    ratio = position - math.floor(position)
    upper = (lower + 1) % len(values)
    return values[lower] + ratio * (values[upper] - values[lower])


def generate_candidate_path(
    source_path: Path,
    destination_path: Path,
    path_genes: dict[str, float],
    *,
    anchor_count: int,
    loop_start_index: int,
    prefix: str = "path_offset_",
) -> dict[str, float]:
    offsets = [float(path_genes[f"{prefix}{index:02d}"]) for index in range(anchor_count)]
    with source_path.open(newline="", encoding="utf-8") as source:
        fieldnames = tuple(csv.DictReader(source).fieldnames or ())
    with source_path.open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    if not rows or loop_start_index < 1 or loop_start_index >= len(rows) - 1:
        raise ValueError("invalid trajectory or loop_start_index")

    points = [(float(row["x"]), float(row["y"])) for row in rows]
    loop = points[loop_start_index:]
    cumulative = [0.0]
    for previous, current in zip(loop, loop[1:]):
        cumulative.append(cumulative[-1] + math.hypot(
            current[0] - previous[0], current[1] - previous[1]
        ))
    loop_length = cumulative[-1]
    if loop_length <= 0.0:
        raise ValueError("loop length must be positive")

    shifted = points.copy()
    last_loop_index = len(loop) - 1
    for local_index, (x, y) in enumerate(loop):
        progress = cumulative[local_index] / loop_length
        offset = _periodic_interpolate(offsets, progress)
        previous = loop[local_index - 1 if local_index > 0 else last_loop_index - 1]
        following = loop[local_index + 1 if local_index < last_loop_index else 1]
        tangent_x = following[0] - previous[0]
        tangent_y = following[1] - previous[1]
        tangent_length = math.hypot(tangent_x, tangent_y)
        if tangent_length <= 1.0e-9:
            normal_x, normal_y = 0.0, 0.0
        else:
            normal_x = -tangent_y / tangent_length
            normal_y = tangent_x / tangent_length
        shifted[loop_start_index + local_index] = (
            x + offset * normal_x,
            y + offset * normal_y,
        )

    # The source loop repeats its start as the final point. Keep that closure exact.
    shifted[-1] = shifted[loop_start_index]
    for index in range(loop_start_index, len(rows)):
        previous_index = index - 1 if index > loop_start_index else len(rows) - 2
        next_index = index + 1 if index < len(rows) - 1 else loop_start_index + 1
        yaw = math.atan2(
            shifted[next_index][1] - shifted[previous_index][1],
            shifted[next_index][0] - shifted[previous_index][0],
        )
        rows[index]["x"] = repr(shifted[index][0])
        rows[index]["y"] = repr(shifted[index][1])
        rows[index]["x_quat"] = "0.0"
        rows[index]["y_quat"] = "0.0"
        rows[index]["z_quat"] = repr(math.sin(0.5 * yaw))
        rows[index]["w_quat"] = repr(math.cos(0.5 * yaw))

    destination_path.parent.mkdir(parents=True, exist_ok=True)
    with destination_path.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(destination, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    differences = [
        offsets[(index + 1) % anchor_count] - offsets[index]
        for index in range(anchor_count)
    ]
    second_differences = [
        differences[(index + 1) % anchor_count] - differences[index]
        for index in range(anchor_count)
    ]
    return {
        "path_offset_rms_m": math.sqrt(sum(value * value for value in offsets) / anchor_count),
        "path_offset_max_m": max(abs(value) for value in offsets),
        "path_offset_smoothness_m": math.sqrt(
            sum(value * value for value in second_differences) / anchor_count
        ),
    }
