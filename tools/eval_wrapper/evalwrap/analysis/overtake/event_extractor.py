from __future__ import annotations

import math
from typing import Any


ATTEMPT_STATES = {"OVERTAKE_PREP", "OVERTAKING"}
PASS_STATES = {"OVERTAKE_PREP", "OVERTAKING", "RETURNING"}


def normalize_state(value: object) -> str:
    text = str(value or "").strip().upper()
    if not text:
        return "UNKNOWN"
    aliases = {
        "FREE_RUN": "NORMAL",
        "FOLLOW_BLOCKED": "FOLLOWING",
        "PREPARE_OVERTAKE_LEFT": "OVERTAKE_PREP",
        "PREPARE_OVERTAKE_RIGHT": "OVERTAKE_PREP",
        "OVERTAKE_LEFT": "OVERTAKING",
        "OVERTAKE_RIGHT": "OVERTAKING",
        "MERGE_BACK": "RETURNING",
        "ABORT_RECOVERY": "ABORTED",
        "SIDE_BY_SIDE_KEEP": "SIDE_BY_SIDE",
        "YIELD_BEHIND": "ABORTED",
    }
    return aliases.get(text, text)


def extract_overtake_attempts(rows: list[dict[str, object]], config: dict[str, Any]) -> list[dict[str, object]]:
    ordered = sorted(rows, key=lambda row: _time(row) if _time(row) is not None else 0.0)
    attempts: list[dict[str, object]] = []
    active: list[dict[str, object]] = []
    attempt_id = 0
    saw_returning = False

    for row in ordered:
        state = normalize_state(row.get("overtake_state"))
        if not active:
            if state in ATTEMPT_STATES:
                attempt_id += 1
                active = [row]
                saw_returning = state == "RETURNING"
            continue

        active.append(row)
        if state == "RETURNING":
            saw_returning = True

        elapsed = _elapsed(active)
        timeout = elapsed is not None and elapsed > _cfg(config, "success.t_pass_max_sec", 8.0)
        finished = state in {"ABORTED", "COMPLETED"} or (saw_returning and state in {"NORMAL", "FOLLOWING"})
        if finished or timeout:
            attempts.append(classify_attempt(active, config, attempt_id, timeout=timeout, saw_returning=saw_returning))
            active = []
            saw_returning = False

    if active:
        attempts.append(classify_attempt(active, config, attempt_id, timeout=True, saw_returning=saw_returning))
    return attempts


def classify_attempt(
    attempt_rows: list[dict[str, object]],
    config: dict[str, Any],
    attempt_id: int,
    *,
    timeout: bool = False,
    saw_returning: bool = False,
) -> dict[str, object]:
    start = attempt_rows[0]
    end = attempt_rows[-1]
    states = [normalize_state(row.get("overtake_state")) for row in attempt_rows]
    collision = any(_truthy(row.get("collision_flag")) for row in attempt_rows)
    penalty = any(_truthy(row.get("penalty_flag")) for row in attempt_rows)
    max_slack = _max_float(row.get("cbf_slack") for row in attempt_rows)
    min_cbf_h = _min_float(row.get("min_cbf_h") for row in attempt_rows)
    min_distance = _min_float(row.get("closest_vehicle_distance_m") or row.get("front_distance_m") for row in attempt_rows)
    infeasible_count = sum(1 for row in attempt_rows if _is_infeasible(row.get("mpc_status")))
    unsafe = (
        collision
        or penalty
        or infeasible_count > 0
        or (max_slack is not None and max_slack > _cfg(config, "success.major_slack_threshold", 0.20))
    )

    if "ABORTED" in states:
        result = "aborted"
    elif saw_returning or "COMPLETED" in states or _passed_target(attempt_rows, config):
        result = "unsafe_success" if unsafe else "success"
    elif timeout:
        result = "failed"
    else:
        result = "unknown"

    abort_reason = str(end.get("overtake_abort_reason") or end.get("abort_reason") or "").strip()
    if result in {"aborted", "failed"} and not abort_reason:
        abort_reason = infer_abort_reason(attempt_rows, config, timeout=timeout)

    time_to_pass = _time_to_first_state(attempt_rows, "RETURNING")
    return_to_line = None
    if time_to_pass is not None:
        return_to_line = (_time(end) or 0.0) - ((_time(start) or 0.0) + time_to_pass)

    return {
        "attempt_id": attempt_id,
        "target_vehicle_id": _first_non_empty(attempt_rows, "target_vehicle_id", "front_vehicle_id"),
        "start_time_sec": _time(start),
        "end_time_sec": _time(end),
        "start_lap": start.get("lap", ""),
        "start_section": start.get("section", ""),
        "start_s": _first_float_value(start, "s", "ego_s", "track_s_m"),
        "end_s": _first_float_value(end, "s", "ego_s", "track_s_m"),
        "start_x": _first_float_value(start, "ego_x", "x_m"),
        "start_y": _first_float_value(start, "ego_y", "y_m"),
        "end_x": _first_float_value(end, "ego_x", "x_m"),
        "end_y": _first_float_value(end, "ego_y", "y_m"),
        "result": result,
        "abort_reason": abort_reason,
        "time_to_pass_sec": time_to_pass,
        "return_to_line_time_sec": return_to_line if return_to_line is None else max(0.0, return_to_line),
        "min_vehicle_distance_m": min_distance,
        "min_cbf_h": min_cbf_h,
        "max_cbf_slack": max_slack,
        "mpc_infeasible_count": infeasible_count,
        "collision": collision,
        "penalty": penalty,
        "overtake_gain_sec": None,
    }


def infer_abort_reason(rows: list[dict[str, object]], config: dict[str, Any], *, timeout: bool = False) -> str:
    if any((_float(row.get("min_cbf_h")) is not None and _float(row.get("min_cbf_h")) < _cfg(config, "safety.cbf_h_warn", 0.2)) for row in rows):
        return "cbf_too_close"
    if any((_float(row.get("cbf_slack")) or 0.0) > 0.0 for row in rows):
        return "cbf_too_close"
    if any(_is_infeasible(row.get("mpc_status")) for row in rows):
        return "mpc_infeasible"
    if any(abs(_float(row.get("reference_curvature")) or 0.0) >= _cfg(config, "opportunity.curvature_overtake_threshold", 0.08) for row in rows):
        return "corner_entry"
    speeds = [_float(row.get("ego_speed_mps")) for row in rows]
    if any(speed is not None and speed < 0.5 for speed in speeds):
        return "low_speed"
    if timeout:
        return "timeout"
    return "unknown"


def _passed_target(rows: list[dict[str, object]], config: dict[str, Any]) -> bool:
    margin = _cfg(config, "success.pass_margin_m", 1.0)
    for row in rows:
        ego_s = _first_float_value(row, "ego_s", "s", "track_s_m")
        target_s = _first_float_value(row, "target_s", "front_s")
        if ego_s is not None and target_s is not None and ego_s > target_s + margin:
            return True
    return False


def _time_to_first_state(rows: list[dict[str, object]], state: str) -> float | None:
    start = _time(rows[0])
    if start is None:
        return None
    for row in rows:
        if normalize_state(row.get("overtake_state")) == state:
            current = _time(row)
            return None if current is None else max(0.0, current - start)
    return None


def _elapsed(rows: list[dict[str, object]]) -> float | None:
    start = _time(rows[0])
    end = _time(rows[-1])
    if start is None or end is None:
        return None
    return end - start


def _time(row: dict[str, object]) -> float | None:
    timestamp = _float(row.get("timestamp_sec"))
    if timestamp is not None:
        return timestamp
    return _float(row.get("time_sec"))


def _first_non_empty(rows: list[dict[str, object]], *keys: str) -> object:
    for row in rows:
        for key in keys:
            value = row.get(key)
            if value not in (None, ""):
                return value
    return ""


def _first_float_value(row: dict[str, object], *keys: str) -> float | None:
    for key in keys:
        value = _float(row.get(key))
        if value is not None:
            return value
    return None


def _is_infeasible(value: object) -> bool:
    text = str(value or "").lower()
    return "infeasible" in text or "fatal" in text


def _truthy(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value or "").strip().lower() in {"1", "true", "yes", "collision", "penalty"}


def _float(value: object) -> float | None:
    if isinstance(value, bool) or value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _min_float(values) -> float | None:
    parsed = [value for value in (_float(item) for item in values) if value is not None]
    return min(parsed) if parsed else None


def _max_float(values) -> float | None:
    parsed = [value for value in (_float(item) for item in values) if value is not None]
    return max(parsed) if parsed else None


def _cfg(config: dict[str, Any], dotted: str, default: float) -> float:
    current: Any = config
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return float(current) if isinstance(current, (int, float)) else default
