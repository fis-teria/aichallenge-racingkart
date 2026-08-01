from __future__ import annotations

import statistics
from typing import Any


def unintended_speed_loss_mps(
    metrics: dict[str, Any],
    acceleration_threshold_mps2: float = 0.3,
    commanded_acceleration_min_mps2: float = 0.1,
    minimum_lap: int = 2,
) -> float:
    """Integrate avoidable deceleration while the controller requests acceleration.

    Acceleration is reconstructed from speed and lap-local timestamps so older
    traces remain usable even when their instantaneous acceleration diagnostic
    was sampled incorrectly.
    """
    trace = metrics.get("diagnostic_trace", [])
    total = 0.0
    previous: dict[str, Any] | None = None
    for item in trace:
        lap = item.get("lap")
        lap_time = item.get("lap_time_seconds")
        speed = item.get("actual_speed_mps")
        commanded = item.get("commanded_acceleration_mps2")
        if (
            lap is None
            or lap_time is None
            or speed is None
            or commanded is None
        ):
            previous = None
            continue
        if previous is not None and int(lap) >= minimum_lap:
            same_lap = int(previous["lap"]) == int(lap)
            delta_time = float(lap_time) - float(previous["lap_time_seconds"])
            if same_lap and 1.0e-4 < delta_time <= 1.0:
                acceleration = (
                    float(speed) - float(previous["actual_speed_mps"])
                ) / delta_time
                if (
                    float(commanded) >= commanded_acceleration_min_mps2
                    and acceleration < -acceleration_threshold_mps2
                ):
                    total += (
                        -acceleration - acceleration_threshold_mps2
                    ) * delta_time
        previous = item
    return float(total)


def speed_recovery_deficit_m(
    metrics: dict[str, Any],
    deadband_mps: float = 0.5,
    commanded_acceleration_min_mps2: float = 0.1,
    minimum_lap: int = 2,
) -> float:
    """Integrate speed missing from the lap-local peak while acceleration is requested."""
    total = 0.0
    peak_speed = 0.0
    previous: dict[str, Any] | None = None
    previous_lap: int | None = None
    for item in metrics.get("diagnostic_trace", []):
        lap = item.get("lap")
        lap_time = item.get("lap_time_seconds")
        speed = item.get("actual_speed_mps")
        commanded = item.get("commanded_acceleration_mps2")
        if lap is None or lap_time is None or speed is None or commanded is None:
            previous = None
            continue
        lap = int(lap)
        speed = float(speed)
        if previous_lap != lap:
            peak_speed = speed
            previous = None
            previous_lap = lap
        peak_speed = max(peak_speed, speed)
        if previous is not None and lap >= minimum_lap:
            delta_time = float(lap_time) - float(previous["lap_time_seconds"])
            if 1.0e-4 < delta_time <= 1.0 and float(commanded) >= commanded_acceleration_min_mps2:
                deficit = max(0.0, peak_speed - speed - deadband_mps)
                total += deficit * delta_time
        previous = item
    return float(total)


def high_steering_speed_loss_mps(
    metrics: dict[str, Any],
    steering_threshold_rad: float = 0.18,
    acceleration_threshold_mps2: float = 0.15,
    commanded_acceleration_min_mps2: float = 0.1,
    minimum_lap: int = 2,
) -> float:
    """Integrate deceleration that occurs while a large steering angle is held."""
    total = 0.0
    previous: dict[str, Any] | None = None
    for item in metrics.get("diagnostic_trace", []):
        required = (
            item.get("lap"), item.get("lap_time_seconds"),
            item.get("actual_speed_mps"), item.get("commanded_acceleration_mps2"),
            item.get("steering_angle_rad"),
        )
        if any(value is None for value in required):
            previous = None
            continue
        if previous is not None and int(item["lap"]) >= minimum_lap:
            same_lap = int(previous["lap"]) == int(item["lap"])
            delta_time = float(item["lap_time_seconds"]) - float(
                previous["lap_time_seconds"]
            )
            acceleration = (
                float(item["actual_speed_mps"]) - float(previous["actual_speed_mps"])
            ) / delta_time if delta_time > 0.0 else 0.0
            large_steering = max(
                abs(float(item["steering_angle_rad"])),
                abs(float(previous["steering_angle_rad"])),
            ) >= steering_threshold_rad
            if (
                same_lap
                and 1.0e-4 < delta_time <= 1.0
                and large_steering
                and float(item["commanded_acceleration_mps2"])
                >= commanded_acceleration_min_mps2
                and acceleration < -acceleration_threshold_mps2
            ):
                total += (-acceleration - acceleration_threshold_mps2) * delta_time
        previous = item
    return float(total)


def score(metrics: dict[str, Any], weights: dict[str, float]) -> float:
    invalid = bool(metrics.get("invalid", False))
    completed = bool(metrics.get("completed", False))
    value = weights["invalid"] * invalid
    value += weights["unfinished"] * (not completed)
    value += weights["collision"] * float(metrics.get("collision_count", 0))
    value += weights["wall"] * float(metrics.get("wall_count", 0))
    value += weights["over"] * float(metrics.get("over_count", 0))
    value += weights["penalty_second"] * float(metrics.get("penalty_seconds", 0.0))
    value += float(metrics.get("lap_time_seconds", metrics.get("elapsed_seconds", 0.0)))
    lateral_p95 = float(metrics.get("lateral_error_p95_m", 0.0))
    lateral_max = float(metrics.get("lateral_error_max_m", 0.0))
    value += weights["lateral_error_p95"] * lateral_p95
    value += float(weights.get("lateral_error_max", 0.0)) * lateral_max
    soft_limit = weights.get("lateral_error_soft_limit_m")
    if soft_limit is not None and lateral_p95 > float(soft_limit):
        excess = lateral_p95 - float(soft_limit)
        value += float(weights.get("lateral_error_excess_quadratic", 0.0)) * excess * excess
    hard_limit = weights.get("lateral_error_hard_limit_m")
    if hard_limit is not None and lateral_p95 > float(hard_limit):
        value += float(weights.get("lateral_error_hard_penalty", 0.0))
    value += weights["steering_delta_rms"] * float(metrics.get("steering_delta_rms", 0.0))
    value += weights["steering_rate_limit_ratio"] * float(
        metrics.get("steering_rate_limit_ratio", 0.0)
    )
    value += float(weights.get("path_offset_rms", 0.0)) * float(
        metrics.get("path_offset_rms_m", 0.0)
    )
    value += float(weights.get("path_offset_smoothness", 0.0)) * float(
        metrics.get("path_offset_smoothness_m", 0.0)
    )
    value += float(weights.get("path_length_excess", 0.0)) * float(
        metrics.get("path_length_excess_m", 0.0)
    )
    value += float(weights.get("steering_angle_rms", 0.0)) * float(
        metrics.get("steering_angle_rms", 0.0)
    )
    speed_loss = unintended_speed_loss_mps(
        metrics,
        acceleration_threshold_mps2=float(
            weights.get("unintended_speed_loss_threshold_mps2", 0.3)
        ),
        commanded_acceleration_min_mps2=float(
            weights.get("unintended_speed_loss_command_min_mps2", 0.1)
        ),
        minimum_lap=int(weights.get("unintended_speed_loss_minimum_lap", 2)),
    )
    value += float(weights.get("unintended_speed_loss", 0.0)) * speed_loss
    recovery_deficit = speed_recovery_deficit_m(
        metrics,
        deadband_mps=float(weights.get("speed_recovery_deadband_mps", 0.5)),
        commanded_acceleration_min_mps2=float(
            weights.get("speed_recovery_command_min_mps2", 0.1)
        ),
        minimum_lap=int(weights.get("speed_recovery_minimum_lap", 2)),
    )
    value += float(weights.get("speed_recovery_deficit", 0.0)) * recovery_deficit
    high_steering_loss = high_steering_speed_loss_mps(
        metrics,
        steering_threshold_rad=float(
            weights.get("high_steering_speed_loss_threshold_rad", 0.18)
        ),
        acceleration_threshold_mps2=float(
            weights.get("high_steering_speed_loss_acceleration_threshold_mps2", 0.15)
        ),
        commanded_acceleration_min_mps2=float(
            weights.get("high_steering_speed_loss_command_min_mps2", 0.1)
        ),
        minimum_lap=int(weights.get("high_steering_speed_loss_minimum_lap", 2)),
    )
    value += float(weights.get("high_steering_speed_loss", 0.0)) * high_steering_loss
    if not completed:
        value -= weights["progress_credit"] * float(metrics.get("progress", 0.0))
    return float(value)


def robust_score(scores: list[float], mad_weight: float = 0.5, worst_weight: float = 0.1) -> float:
    median = statistics.median(scores)
    mad = statistics.median(abs(item - median) for item in scores)
    return median + mad_weight * mad + worst_weight * max(scores)
