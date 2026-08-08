from __future__ import annotations

import statistics
from dataclasses import asdict, dataclass
from typing import Any

from .fitness import high_steering_exposure_rad_s, steering_unwind_delay_seconds


@dataclass(frozen=True)
class ConstrainedResult:
    feasible: bool
    violation: float
    robust_lap: float | None
    objective_value: float
    complete_repeat_ratio: float
    lateral_error_p95_m: float
    lateral_error_max_m: float
    failure_reasons: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["failure_reasons"] = list(self.failure_reasons)
        return result


def _percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def timer_is_valid(metrics: dict[str, Any]) -> bool:
    if metrics.get("invalid") or not metrics.get("completed"):
        return False
    if metrics.get("flying_lap_time_seconds") is None:
        return False
    first_section = next((item for item in metrics.get("section_splits", [])
                          if int(item.get("section", -1)) == 1), None)
    return first_section is not None and float(
        first_section.get("lap_time_seconds") or 0.0
    ) >= 1.0


def evaluate_constrained(episodes: list[dict[str, Any]], settings: dict[str, Any]) -> ConstrainedResult:
    if not episodes:
        raise ValueError("constrained objective requires at least one episode")
    required_ratio = float(settings.get("required_complete_repeat_ratio", 1.0))
    collision_limit = int(settings.get("maximum_collision_count", 0))
    wall_limit = int(settings.get("maximum_wall_count", 0))
    p95_limit = float(settings.get("lateral_error_p95_limit_m", 1.35))
    maximum_limit = float(settings.get("lateral_error_max_limit_m", 2.0))
    weights = settings.get("violation_weights", {})
    unfinished_weight = float(weights.get("unfinished", 1000.0))
    collision_weight = float(weights.get("collision", 100.0))
    wall_weight = float(weights.get("wall", 100.0))
    timer_weight = float(weights.get("invalid_timer", 1000.0))
    p95_weight = float(weights.get("lateral_p95", 10.0))
    maximum_weight = float(weights.get("lateral_max", 10.0))

    valid = [item for item in episodes if timer_is_valid(item)]
    complete_ratio = len(valid) / len(episodes)
    collisions = sum(int(item.get("collision_count", 0)) for item in episodes)
    walls = sum(int(item.get("wall_count", 0)) for item in episodes)
    lateral_p95 = max(float(item.get("lateral_error_p95_m", float("inf")))
                      for item in episodes)
    lateral_max = max(float(item.get("lateral_error_max_m", float("inf")))
                      for item in episodes)
    reasons: list[str] = []
    if complete_ratio < required_ratio:
        reasons.append("incomplete_or_invalid_timer")
    if collisions > collision_limit:
        reasons.append("collision")
    if walls > wall_limit:
        reasons.append("wall")
    if lateral_p95 > p95_limit:
        reasons.append("lateral_p95")
    if lateral_max > maximum_limit:
        reasons.append("lateral_max")
    violation = max(0.0, required_ratio - complete_ratio) * unfinished_weight
    violation += (len(episodes) - len(valid)) * timer_weight
    violation += max(0, collisions - collision_limit) * collision_weight
    violation += max(0, walls - wall_limit) * wall_weight
    violation += max(0.0, lateral_p95 - p95_limit) * p95_weight
    violation += max(0.0, lateral_max - maximum_limit) * maximum_weight
    feasible = not reasons
    robust_lap = None
    if valid:
        laps = [float(item["flying_lap_time_seconds"]) for item in valid]
        spread = _percentile(laps, 0.9) - _percentile(laps, 0.1)
        exposure = statistics.median(high_steering_exposure_rad_s(item) for item in valid)
        unwind = statistics.median(steering_unwind_delay_seconds(item) for item in valid)
        robust_lap = statistics.median(laps) + 0.25 * spread + 0.02 * exposure + 0.02 * unwind
    infeasible_base = float(settings.get("infeasible_objective_base", 1_000_000.0))
    objective = robust_lap if feasible and robust_lap is not None else infeasible_base + violation
    return ConstrainedResult(feasible, violation, robust_lap, objective, complete_ratio,
                             lateral_p95, lateral_max, tuple(reasons))
