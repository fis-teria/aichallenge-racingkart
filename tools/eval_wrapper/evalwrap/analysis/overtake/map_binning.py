from __future__ import annotations

from typing import Any


def build_overtake_map_bins(
    attempts: list[dict[str, object]],
    rows: list[dict[str, object]],
    blocked: list[dict[str, object]],
    missed: list[dict[str, object]],
    config: dict[str, Any],
) -> list[dict[str, object]]:
    use_s = any(_float(row.get("s")) is not None or _float(row.get("track_s_m")) is not None for row in rows)
    if use_s:
        return _s_bins(attempts, rows, blocked, missed, config)
    return _section_bins(attempts, rows, blocked, missed)


def _s_bins(
    attempts: list[dict[str, object]],
    rows: list[dict[str, object]],
    blocked: list[dict[str, object]],
    missed: list[dict[str, object]],
    config: dict[str, Any],
) -> list[dict[str, object]]:
    width = max(0.1, _cfg(config, "map.bin_width_m", 5.0))
    values = [_first_float(row, "s", "ego_s", "track_s_m") for row in rows]
    s_values = [value for value in values if value is not None]
    if not s_values:
        return []
    max_s = max(s_values)
    count = max(1, int(max_s // width) + 1)
    bins = [_empty_bin(f"s{i:03d}", i * width, (i + 1) * width, "") for i in range(count)]
    _fill_bins(bins, attempts, rows, blocked, missed, width=width)
    return bins


def _section_bins(
    attempts: list[dict[str, object]],
    rows: list[dict[str, object]],
    blocked: list[dict[str, object]],
    missed: list[dict[str, object]],
) -> list[dict[str, object]]:
    sections = sorted({str(row.get("section")) for row in rows if row.get("section") not in (None, "")})
    bins = [_empty_bin(f"section_{section}", "", "", section) for section in sections]
    by_section = {row["section"]: row for row in bins}
    for attempt in attempts:
        section = str(attempt.get("start_section") or "")
        if section in by_section:
            _count_attempt(by_section[section], attempt)
    for segment in blocked:
        section = str(segment.get("section") or "")
        if section in by_section:
            by_section[section]["blocked_time_sec"] += _float(segment.get("duration_sec")) or 0.0
    for segment in missed:
        section = str(segment.get("section") or "")
        if section in by_section:
            by_section[section]["missed_chance_count"] += 1
    for row in rows:
        section = str(row.get("section") or "")
        if section in by_section:
            _accumulate_safety(by_section[section], row)
    _finalize(bins)
    return bins


def _fill_bins(
    bins: list[dict[str, object]],
    attempts: list[dict[str, object]],
    rows: list[dict[str, object]],
    blocked: list[dict[str, object]],
    missed: list[dict[str, object]],
    *,
    width: float,
) -> None:
    for attempt in attempts:
        index = _bin_index(_float(attempt.get("start_s")), width, len(bins))
        if index is not None:
            _count_attempt(bins[index], attempt)
    for segment in blocked:
        index = _bin_index(_float(segment.get("start_s")), width, len(bins))
        if index is not None:
            bins[index]["blocked_time_sec"] += _float(segment.get("duration_sec")) or 0.0
    for segment in missed:
        index = _bin_index(_float(segment.get("start_s")), width, len(bins))
        if index is not None:
            bins[index]["missed_chance_count"] += 1
    for row in rows:
        index = _bin_index(_first_float(row, "s", "ego_s", "track_s_m"), width, len(bins))
        if index is not None:
            _accumulate_safety(bins[index], row)
    _finalize(bins)


def _empty_bin(bin_id: str, s_start: object, s_end: object, section: object) -> dict[str, object]:
    return {
        "bin_id": bin_id,
        "s_start": s_start,
        "s_end": s_end,
        "section": section,
        "attempt_count": 0,
        "success_count": 0,
        "abort_count": 0,
        "missed_chance_count": 0,
        "blocked_time_sec": 0.0,
        "avg_min_cbf_h": None,
        "max_cbf_slack": None,
        "collision_count": 0,
        "success_rate": None,
        "_cbf_sum": 0.0,
        "_cbf_count": 0,
    }


def _count_attempt(row: dict[str, object], attempt: dict[str, object]) -> None:
    result = str(attempt.get("result") or "")
    row["attempt_count"] += 1
    if result in {"success", "unsafe_success"}:
        row["success_count"] += 1
    if result == "aborted":
        row["abort_count"] += 1
    if bool(attempt.get("collision")):
        row["collision_count"] += 1


def _accumulate_safety(bin_row: dict[str, object], row: dict[str, object]) -> None:
    cbf_h = _float(row.get("min_cbf_h"))
    if cbf_h is not None:
        bin_row["_cbf_sum"] += cbf_h
        bin_row["_cbf_count"] += 1
    slack = _float(row.get("cbf_slack"))
    if slack is not None:
        current = _float(bin_row.get("max_cbf_slack"))
        bin_row["max_cbf_slack"] = slack if current is None else max(current, slack)
    if _truthy(row.get("collision_flag")):
        bin_row["collision_count"] += 1


def _finalize(bins: list[dict[str, object]]) -> None:
    for row in bins:
        attempts = int(row["attempt_count"])
        row["success_rate"] = (int(row["success_count"]) / attempts) if attempts else None
        count = int(row.pop("_cbf_count"))
        total = float(row.pop("_cbf_sum"))
        row["avg_min_cbf_h"] = (total / count) if count else None


def _bin_index(value: float | None, width: float, count: int) -> int | None:
    if value is None:
        return None
    return min(max(0, int(value // width)), count - 1)


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


def _truthy(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value or "").strip().lower() in {"1", "true", "yes"}


def _cfg(config: dict[str, Any], dotted: str, default: float) -> float:
    current: Any = config
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return float(current) if isinstance(current, (int, float)) else default
