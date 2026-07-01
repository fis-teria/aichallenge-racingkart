from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

from .event_extractor import extract_overtake_attempts, normalize_state
from .judgement import judge_overtake_run
from .map_binning import build_overtake_map_bins
from .opportunity_detector import detect_blocked_intervals, detect_missed_overtake_chances


TIMESERIES_FIELDS = [
    "run_id",
    "domain_id",
    "timestamp_sec",
    "lap",
    "section",
    "s",
    "ego_x",
    "ego_y",
    "ego_speed_mps",
    "front_vehicle_id",
    "front_distance_m",
    "front_delta_d",
    "relative_speed_mps",
    "overtake_state",
    "selected",
    "attempt_id",
    "target_vehicle_id",
    "target_lateral_offset_m",
    "ego_lateral_offset",
    "reference_curvature",
    "target_speed_mps",
    "min_cbf_h",
    "cbf_slack",
    "active_cbf_constraint_count",
    "closest_vehicle_id",
    "closest_vehicle_distance_m",
    "mpc_status",
    "mpc_solve_time_ms",
    "mpc_infeasible_count",
    "collision_flag",
    "penalty_flag",
    "blocked",
    "side_by_side",
    "corner_side_by_side",
    "corner_abs_curvature",
    "side_vehicle_id",
    "side_delta_s",
    "side_delta_d",
    "side_lateral_gap_m",
    "side_relative_speed_mps",
    "left_pass_gap_m",
    "right_pass_gap_m",
    "can_pass_left",
    "can_pass_right",
    "pass_gap_required_m",
    "pass_gap_reason",
    "active_override",
    "reason",
    "abort_reason",
]

ATTEMPT_FIELDS = [
    "run_id",
    "domain_id",
    "attempt_id",
    "target_vehicle_id",
    "start_time_sec",
    "end_time_sec",
    "start_lap",
    "start_section",
    "start_s",
    "end_s",
    "result",
    "abort_reason",
    "time_to_pass_sec",
    "return_to_line_time_sec",
    "min_vehicle_distance_m",
    "min_cbf_h",
    "max_cbf_slack",
    "mpc_infeasible_count",
    "collision",
    "penalty",
    "overtake_gain_sec",
    "start_x",
    "start_y",
    "end_x",
    "end_y",
]

BIN_FIELDS = [
    "run_id",
    "domain_id",
    "bin_id",
    "s_start",
    "s_end",
    "section",
    "attempt_count",
    "success_count",
    "abort_count",
    "missed_chance_count",
    "blocked_time_sec",
    "avg_min_cbf_h",
    "max_cbf_slack",
    "collision_count",
    "success_rate",
]


def build_overtake_outputs(
    run_id: str,
    domains: list[object],
    processed_dir: Path,
    config: dict[str, Any] | None = None,
) -> dict[str, object]:
    cfg = config or {}
    all_timeseries: list[dict[str, object]] = []
    all_attempts: list[dict[str, object]] = []
    all_bins: list[dict[str, object]] = []
    domain_metrics: dict[str, dict[str, object]] = {}
    warnings: list[str] = []

    for domain in domains:
        domain_id = str(getattr(domain, "domain_id"))
        timeseries = build_overtake_timeseries(run_id, domain)
        attempts = extract_overtake_attempts(timeseries, cfg)
        blocked = detect_blocked_intervals(timeseries, cfg)
        missed = detect_missed_overtake_chances(timeseries, cfg)
        bins = build_overtake_map_bins(attempts, timeseries, blocked, missed, cfg)
        metrics = compute_overtake_metrics(
            attempts,
            timeseries,
            blocked,
            missed,
            domain_metrics=getattr(domain, "metrics", None),
            config=cfg,
        )
        metrics["judgement"] = judge_overtake_run(metrics, cfg)
        metrics["analysis_available"] = bool(getattr(domain, "overtake_debug_timeseries", []))
        if not metrics["analysis_available"]:
            warnings.append(f"{domain_id}: overtake debug topics were not recorded")

        for row in timeseries:
            row["run_id"] = run_id
            row["domain_id"] = domain_id
        for row in attempts:
            row["run_id"] = run_id
            row["domain_id"] = domain_id
        for row in bins:
            row["run_id"] = run_id
            row["domain_id"] = domain_id

        all_timeseries.extend(timeseries)
        all_attempts.extend(attempts)
        all_bins.extend(bins)
        domain_metrics[domain_id] = metrics

    processed_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(processed_dir / "overtake_timeseries.csv", TIMESERIES_FIELDS, all_timeseries)
    _write_csv(processed_dir / "overtake_attempts.csv", ATTEMPT_FIELDS, all_attempts)
    _write_csv(processed_dir / "overtake_map_bins.csv", BIN_FIELDS, all_bins)
    output = {
        "run_id": run_id,
        "domains": domain_metrics,
        "warnings": warnings,
    }
    (processed_dir / "overtake_metrics.json").write_text(
        json.dumps(output, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return output


def build_overtake_timeseries(run_id: str, domain: object) -> list[dict[str, object]]:
    vehicle_rows = list(getattr(domain, "vehicle_timeseries", []))
    overtake_rows = list(getattr(domain, "overtake_debug_timeseries", []))
    speed_rows = list(getattr(domain, "speed_profile_debug_timeseries", []))

    if vehicle_rows:
        base_rows = vehicle_rows
    else:
        base_rows = overtake_rows

    rows: list[dict[str, object]] = []
    for base in base_rows:
        time_sec = _first_float(base, "time_sec", "timestamp_sec")
        overtake = _nearest(overtake_rows, time_sec, tolerance_sec=0.4)
        speed = _nearest(speed_rows, time_sec, tolerance_sec=0.4)
        if not overtake and not speed and base in overtake_rows:
            overtake = base
        row = _merge_row(base, overtake or {}, speed or {})
        row["run_id"] = run_id
        row["domain_id"] = getattr(domain, "domain_id")
        rows.append(row)

    if not rows and overtake_rows:
        for overtake in overtake_rows:
            row = _merge_row({}, overtake, {})
            row["run_id"] = run_id
            row["domain_id"] = getattr(domain, "domain_id")
            rows.append(row)
    return sorted(rows, key=lambda row: _float(row.get("timestamp_sec")) or 0.0)


def compute_overtake_metrics(
    attempts: list[dict[str, object]],
    timeseries: list[dict[str, object]],
    blocked: list[dict[str, object]],
    missed: list[dict[str, object]],
    *,
    domain_metrics: object | None,
    config: dict[str, Any],
) -> dict[str, object]:
    attempt_count = len(attempts)
    success_count = sum(1 for row in attempts if row.get("result") == "success")
    unsafe_success_count = sum(1 for row in attempts if row.get("result") == "unsafe_success")
    aborted_count = sum(1 for row in attempts if row.get("result") == "aborted")
    failure_count = sum(1 for row in attempts if row.get("result") in {"failed", "aborted", "unsafe_success"})
    collision_count = _domain_int(domain_metrics, "collision_count") + sum(1 for row in attempts if row.get("collision"))
    penalty_count = _domain_int(domain_metrics, "penalty_count") + sum(1 for row in attempts if row.get("penalty"))
    blocked_time = sum(_float(row.get("duration_sec")) or 0.0 for row in blocked)
    opportunity_time = sum(_float(row.get("duration_sec")) or 0.0 for row in missed)
    mpc_infeasible_count = sum(_int(row.get("mpc_infeasible_count")) for row in attempts)
    mpc_infeasible_count += sum(1 for row in timeseries if "infeasible" in str(row.get("mpc_status") or "").lower())
    min_vehicle_distance = _min(
        [_float(row.get("min_vehicle_distance_m")) for row in attempts]
        + [_float(row.get("closest_vehicle_distance_m")) for row in timeseries]
    )
    min_cbf_h = _min([_float(row.get("min_cbf_h")) for row in attempts] + [_float(row.get("min_cbf_h")) for row in timeseries])
    max_cbf_slack = _max(
        [_float(row.get("max_cbf_slack")) for row in attempts] + [_float(row.get("cbf_slack")) for row in timeseries]
    )

    attempt_per_opportunity = None
    opportunities = len(missed) + attempt_count
    if opportunities:
        attempt_per_opportunity = attempt_count / opportunities

    return {
        "attempt_count": attempt_count,
        "success_count": success_count,
        "failure_count": failure_count,
        "aborted_count": aborted_count,
        "success_rate": (success_count / attempt_count) if attempt_count else 0.0,
        "failure_rate": (failure_count / attempt_count) if attempt_count else 0.0,
        "unsafe_success_count": unsafe_success_count,
        "abort_reason_counts": _counts(row.get("abort_reason") for row in attempts if row.get("abort_reason")),
        "blocked_time_sec": blocked_time,
        "blocked_time_delta_sec": None,
        "total_time_delta_sec": None,
        "overtake_opportunity_time_sec": opportunity_time,
        "missed_overtake_chance_count": len(missed),
        "attempt_per_opportunity": attempt_per_opportunity,
        "collision_count": collision_count,
        "penalty_count": penalty_count,
        "min_vehicle_distance_m": min_vehicle_distance,
        "min_cbf_h": min_cbf_h,
        "max_cbf_slack": max_cbf_slack,
        "mpc_infeasible_count": mpc_infeasible_count,
        "avg_time_to_pass_sec": _avg(_float(row.get("time_to_pass_sec")) for row in attempts),
        "avg_return_to_line_time_sec": _avg(_float(row.get("return_to_line_time_sec")) for row in attempts),
        "avg_speed_loss_after_pass_mps": None,
    }


def _merge_row(base: dict[str, object], overtake: dict[str, object], speed: dict[str, object]) -> dict[str, object]:
    state = _first_value(overtake, "overtake_state", "mode")
    return {
        "timestamp_sec": _coalesce(_first_value(base, "time_sec", "timestamp_sec"), _first_value(overtake, "time_sec", "timestamp_sec")),
        "lap": _first_value(base, "lap", "awsim_lap"),
        "section": _coalesce(_first_value(base, "section"), _first_value(overtake, "section")),
        "s": _coalesce(_first_value(base, "track_s_m", "s", "ego_s"), _first_value(overtake, "ego_s", "s")),
        "ego_x": _coalesce(_first_value(base, "x_m", "ego_x"), _first_value(overtake, "ego_x")),
        "ego_y": _coalesce(_first_value(base, "y_m", "ego_y"), _first_value(overtake, "ego_y")),
        "ego_speed_mps": _coalesce(_first_value(base, "speed_mps", "ego_speed_mps"), _first_value(overtake, "ego_speed_mps")),
        "front_vehicle_id": _first_value(overtake, "front_vehicle_id"),
        "front_distance_m": _first_value(overtake, "front_distance_m", "front_delta_s"),
        "front_delta_d": _first_value(overtake, "front_delta_d"),
        "relative_speed_mps": _first_value(overtake, "relative_speed_mps", "front_rel_v"),
        "overtake_state": normalize_state(state),
        "selected": _first_value(overtake, "selected"),
        "attempt_id": _first_value(overtake, "overtake_attempt_id", "attempt_id"),
        "target_vehicle_id": _first_value(overtake, "target_vehicle_id", "front_vehicle_id"),
        "target_lateral_offset_m": _first_value(overtake, "target_lateral_offset_m"),
        "ego_lateral_offset": _first_value(overtake, "ego_lateral_offset"),
        "reference_curvature": _first_value(base, "trajectory_curvature_1pm", "reference_curvature"),
        "target_speed_mps": _first_value(speed, "target_speed_mps", "global_cap_mps", "command_speed_mps"),
        "min_cbf_h": _first_value(overtake, "min_cbf_h", "min_ellipse_h"),
        "cbf_slack": _first_value(overtake, "cbf_slack"),
        "active_cbf_constraint_count": _first_value(overtake, "active_cbf_constraint_count"),
        "closest_vehicle_id": _first_value(overtake, "closest_vehicle_id", "front_vehicle_id"),
        "closest_vehicle_distance_m": _first_value(overtake, "closest_vehicle_distance_m", "front_distance_m", "front_delta_s"),
        "mpc_status": _coalesce(_first_value(speed, "mpc_status"), "unknown"),
        "mpc_solve_time_ms": _first_value(speed, "mpc_solve_time_ms"),
        "mpc_infeasible_count": _first_value(speed, "mpc_infeasible_count"),
        "collision_flag": _first_value(base, "collision_flag"),
        "penalty_flag": _first_value(base, "penalty_flag"),
        "blocked": _first_value(overtake, "blocked"),
        "side_by_side": _first_value(overtake, "side_by_side"),
        "corner_side_by_side": _first_value(overtake, "corner_side_by_side"),
        "corner_abs_curvature": _first_value(overtake, "corner_abs_curvature"),
        "side_vehicle_id": _first_value(overtake, "side_vehicle_id"),
        "side_delta_s": _first_value(overtake, "side_delta_s"),
        "side_delta_d": _first_value(overtake, "side_delta_d"),
        "side_lateral_gap_m": _first_value(overtake, "side_lateral_gap_m"),
        "side_relative_speed_mps": _first_value(overtake, "side_relative_speed_mps"),
        "left_pass_gap_m": _first_value(overtake, "left_pass_gap_m"),
        "right_pass_gap_m": _first_value(overtake, "right_pass_gap_m"),
        "can_pass_left": _first_value(overtake, "can_pass_left"),
        "can_pass_right": _first_value(overtake, "can_pass_right"),
        "pass_gap_required_m": _first_value(overtake, "pass_gap_required_m"),
        "pass_gap_reason": _first_value(overtake, "pass_gap_reason"),
        "active_override": _first_value(overtake, "active_override"),
        "reason": _first_value(overtake, "reason"),
        "abort_reason": _first_value(overtake, "overtake_abort_reason", "abort_reason"),
    }


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def _nearest(rows: list[dict[str, object]], time_sec: float | None, tolerance_sec: float) -> dict[str, object] | None:
    if time_sec is None:
        return None
    best = None
    best_delta = tolerance_sec
    for row in rows:
        row_time = _first_float(row, "time_sec", "timestamp_sec")
        if row_time is None:
            continue
        delta = abs(row_time - time_sec)
        if delta <= best_delta:
            best = row
            best_delta = delta
    return best


def _first_value(row: dict[str, object], *keys: str) -> object:
    for key in keys:
        value = row.get(key)
        if value not in (None, ""):
            return value
    return ""


def _coalesce(*values: object) -> object:
    for value in values:
        if value not in (None, ""):
            return value
    return ""


def _first_float(row: dict[str, object], *keys: str) -> float | None:
    for key in keys:
        value = _float(row.get(key))
        if value is not None:
            return value
    return None


def _domain_int(metrics: object | None, key: str) -> int:
    if metrics is None:
        return 0
    return _int(getattr(metrics, key, None))


def _counts(values) -> dict[str, int]:
    output: dict[str, int] = {}
    for value in values:
        key = str(value)
        output[key] = output.get(key, 0) + 1
    return output


def _avg(values) -> float | None:
    parsed = [value for value in values if value is not None]
    return sum(parsed) / len(parsed) if parsed else None


def _min(values) -> float | None:
    parsed = [value for value in values if value is not None]
    return min(parsed) if parsed else None


def _max(values) -> float | None:
    parsed = [value for value in values if value is not None]
    return max(parsed) if parsed else None


def _float(value: object) -> float | None:
    if isinstance(value, bool) or value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _int(value: object) -> int:
    if isinstance(value, bool) or value in (None, ""):
        return 0
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0
