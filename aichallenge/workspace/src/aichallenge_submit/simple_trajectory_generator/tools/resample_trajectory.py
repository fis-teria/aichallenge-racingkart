#!/usr/bin/env python3
"""Resample an AI Challenge trajectory CSV at uniform arc-length intervals."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


FIELDNAMES = ("x", "y", "z", "x_quat", "y_quat", "z_quat", "w_quat", "speed")


def interpolate(a: float, b: float, ratio: float) -> float:
    return a + ratio * (b - a)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_csv", type=Path)
    parser.add_argument("--spacing", type=float, default=0.5)
    args = parser.parse_args()
    if not math.isfinite(args.spacing) or args.spacing <= 0.0:
        parser.error("--spacing must be a positive finite number")

    with args.input_csv.open(newline="", encoding="utf-8") as source:
        rows = [
            {name: float(value) for name, value in row.items()}
            for row in csv.DictReader(source)
        ]
    if len(rows) < 2:
        raise ValueError("trajectory must contain at least two points")

    cumulative = [0.0]
    for previous, current in zip(rows, rows[1:]):
        cumulative.append(
            cumulative[-1]
            + math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
        )
    total_length = cumulative[-1]
    sample_distances = [
        min(index * args.spacing, total_length)
        for index in range(math.ceil(total_length / args.spacing) + 1)
    ]
    sample_distances[-1] = total_length

    output: list[dict[str, float]] = []
    segment = 0
    for distance in sample_distances:
        while segment + 1 < len(cumulative) - 1 and cumulative[segment + 1] < distance:
            segment += 1
        segment_length = cumulative[segment + 1] - cumulative[segment]
        ratio = 0.0 if segment_length <= 1.0e-9 else (
            distance - cumulative[segment]
        ) / segment_length
        start = rows[segment]
        end = rows[segment + 1]
        yaw = math.atan2(end["y"] - start["y"], end["x"] - start["x"])
        output.append(
            {
                "x": interpolate(start["x"], end["x"], ratio),
                "y": interpolate(start["y"], end["y"], ratio),
                "z": interpolate(start["z"], end["z"], ratio),
                "x_quat": 0.0,
                "y_quat": 0.0,
                "z_quat": math.sin(yaw * 0.5),
                "w_quat": math.cos(yaw * 0.5),
                "speed": interpolate(start["speed"], end["speed"], ratio),
            }
        )

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(destination, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(output)
    print(
        f"resampled {len(rows)} -> {len(output)} points; "
        f"length={total_length:.3f}m spacing={args.spacing:.3f}m"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
