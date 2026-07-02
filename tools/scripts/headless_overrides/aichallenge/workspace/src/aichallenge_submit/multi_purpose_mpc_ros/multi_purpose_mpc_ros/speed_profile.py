from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence


@dataclass(frozen=True)
class SpeedProfilePoint:
    wp_id: int
    curvature_speed_mps: float
    section_cap_mps: float | None
    global_cap_mps: float
    target_speed_mps: float
    source: str


def combine_speed_profile(
    curvature_speed_mps: Sequence[float | None],
    global_cap_mps: float,
    section_cap_provider: Callable[[int], float | None] | None = None,
) -> list[SpeedProfilePoint]:
    output: list[SpeedProfilePoint] = []
    global_cap = max(0.0, float(global_cap_mps))
    for wp_id, curvature_speed in enumerate(curvature_speed_mps):
        curvature_cap = global_cap if curvature_speed is None else max(0.0, float(curvature_speed))
        section_cap = section_cap_provider(wp_id) if section_cap_provider is not None else None
        section_cap = None if section_cap is None else max(0.0, float(section_cap))
        target_speed = min(
            curvature_cap,
            global_cap,
            section_cap if section_cap is not None else global_cap,
        )
        output.append(
            SpeedProfilePoint(
                wp_id=wp_id,
                curvature_speed_mps=curvature_cap,
                section_cap_mps=section_cap,
                global_cap_mps=global_cap,
                target_speed_mps=target_speed,
                source=_speed_source(target_speed, curvature_cap, section_cap, global_cap),
            )
        )
    return output


def apply_combined_speed_profile(reference_path, profile: Sequence[SpeedProfilePoint]) -> None:
    reference_path.set_v_ref([point.target_speed_mps for point in profile])


def _speed_source(
    target_speed_mps: float,
    curvature_speed_mps: float,
    section_cap_mps: float | None,
    global_cap_mps: float,
) -> str:
    eps = 1.0e-6
    if section_cap_mps is not None and abs(target_speed_mps - section_cap_mps) <= eps:
        return "section_cap"
    if abs(target_speed_mps - curvature_speed_mps) <= eps:
        return "curvature"
    if abs(target_speed_mps - global_cap_mps) <= eps:
        return "global_v_max"
    return "combined"
