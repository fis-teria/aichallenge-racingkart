from __future__ import annotations


def summarize_control_timeseries(rows: list[dict[str, object]]) -> dict[str, float | None]:
    accels = [_as_float(row.get("accel_mps2")) for row in rows]
    steers = [_as_float(row.get("steer_rad")) for row in rows]
    throttles = [_as_float(row.get("throttle")) for row in rows]
    brakes = [_as_float(row.get("brake")) for row in rows]
    accels = [value for value in accels if value is not None]
    steers = [value for value in steers if value is not None]
    throttles = [value for value in throttles if value is not None]
    brakes = [value for value in brakes if value is not None]
    return {
        "max_command_accel_mps2": max(accels) if accels else None,
        "max_command_decel_mps2": min(accels) if accels else None,
        "max_command_abs_steer_rad": max((abs(value) for value in steers), default=None),
        "avg_throttle": _mean(throttles),
        "avg_brake": _mean(brakes),
    }


def _as_float(value: object) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None
