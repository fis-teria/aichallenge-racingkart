from __future__ import annotations

from typing import Any

from .event_extractor import normalize_state


def detect_blocked_intervals(rows: list[dict[str, object]], config: dict[str, Any]) -> list[dict[str, object]]:
    return _segments(rows, lambda row: _is_blocked(row, config), "blocked")


def detect_missed_overtake_chances(rows: list[dict[str, object]], config: dict[str, Any]) -> list[dict[str, object]]:
    min_duration = _cfg(config, "opportunity.min_opportunity_duration_sec", 1.0)
    candidates = _segments(rows, lambda row: _is_missed_chance_sample(row, config), "missed_chance")
    return [segment for segment in candidates if (_float(segment.get("duration_sec")) or 0.0) >= min_duration]


def _segments(rows: list[dict[str, object]], predicate, kind: str) -> list[dict[str, object]]:
    ordered = sorted(rows, key=lambda row: _time(row) or 0.0)
    output: list[dict[str, object]] = []
    active: list[dict[str, object]] = []
    for row in ordered:
        if predicate(row):
            active.append(row)
        elif active:
            output.append(_segment(active, kind))
            active = []
    if active:
        output.append(_segment(active, kind))
    return output


def _segment(rows: list[dict[str, object]], kind: str) -> dict[str, object]:
    start = rows[0]
    end = rows[-1]
    start_time = _time(start)
    end_time = _time(end)
    return {
        "kind": kind,
        "start_time_sec": start_time,
        "end_time_sec": end_time,
        "duration_sec": None if start_time is None or end_time is None else max(0.0, end_time - start_time),
        "section": start.get("section", ""),
        "start_s": _first_float(start, "s", "ego_s", "track_s_m"),
        "end_s": _first_float(end, "s", "ego_s", "track_s_m"),
        "start_x": _first_float(start, "ego_x", "x_m"),
        "start_y": _first_float(start, "ego_y", "y_m"),
        "min_front_distance_m": _min(_first_float(row, "front_distance_m") for row in rows),
        "min_cbf_h": _min(_first_float(row, "min_cbf_h") for row in rows),
    }


def _is_blocked(row: dict[str, object], config: dict[str, Any]) -> bool:
    state = normalize_state(row.get("overtake_state"))
    if state == "FOLLOWING":
        return True
    blocked = row.get("blocked")
    if isinstance(blocked, bool):
        return blocked
    if str(blocked).lower() in {"true", "1", "yes"}:
        return True
    distance = _first_float(row, "front_distance_m")
    if distance is None:
        return False
    return distance < _cfg(config, "blocked.blocked_distance_threshold_m", 8.0)


def _is_missed_chance_sample(row: dict[str, object], config: dict[str, Any]) -> bool:
    state = normalize_state(row.get("overtake_state"))
    if state in {"OVERTAKE_PREP", "OVERTAKING", "RETURNING", "ABORTED"}:
        return False
    if state not in {"FOLLOWING", "UNKNOWN"} and not _is_blocked(row, config):
        return False

    front_distance = _first_float(row, "front_distance_m")
    if front_distance is None or front_distance >= _cfg(config, "blocked.blocked_distance_threshold_m", 8.0):
        return False

    speed = _first_float(row, "ego_speed_mps", "speed_mps")
    target_speed = _first_float(row, "speed_target_mps", "target_speed_mps", "global_cap_mps")
    if speed is None or target_speed is None:
        return False
    if speed >= target_speed - _cfg(config, "blocked.speed_loss_threshold_mps", 0.8):
        return False

    relative_speed = _first_float(row, "relative_speed_mps")
    if relative_speed is not None and relative_speed > _cfg(config, "blocked.relative_speed_threshold_mps", 0.5):
        return False

    curvature = abs(_first_float(row, "reference_curvature", "trajectory_curvature_1pm") or 0.0)
    if curvature >= _cfg(config, "opportunity.curvature_overtake_threshold", 0.08):
        return False

    min_cbf_h = _first_float(row, "min_cbf_h")
    if min_cbf_h is not None and min_cbf_h <= _cfg(config, "opportunity.cbf_h_safe_threshold", 0.5):
        return False

    target_offset = _first_float(row, "target_lateral_offset_m")
    if target_offset is not None and abs(target_offset) <= 1e-6:
        return False
    return True


def _time(row: dict[str, object]) -> float | None:
    return _first_float(row, "timestamp_sec", "time_sec")


def _first_float(row: dict[str, object], *keys: str) -> float | None:
    for key in keys:
        value = _float(row.get(key))
        if value is not None:
            return value
    return None


def _float(value: object) -> float | None:
    if isinstance(value, bool) or value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _min(values) -> float | None:
    parsed = [value for value in values if value is not None]
    return min(parsed) if parsed else None


def _cfg(config: dict[str, Any], dotted: str, default: float) -> float:
    current: Any = config
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return float(current) if isinstance(current, (int, float)) else default
