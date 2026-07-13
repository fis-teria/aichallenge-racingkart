from __future__ import annotations

import math
from collections.abc import Iterable
from typing import Any


DEFAULT_LATERAL_ERROR_WARN_M = 0.25


def compare_planner_and_mpc_horizons(
    *,
    overrides: list[dict[str, object]],
    horizons: list[dict[str, object]],
    contracts: list[dict[str, object]],
    reference_history: list[dict[str, object]],
    pure_pursuit_debug: list[dict[str, object]],
    mux_debug: list[dict[str, object]] | None = None,
    final_control_commands: list[dict[str, object]] | None = None,
    lateral_error_warn_m: float = DEFAULT_LATERAL_ERROR_WARN_M,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    """Compare the planner d[] requested for a solver horizon with its points.

    The reference is the latest recorded planning trajectory.  If that topic is
    absent, the result is deliberately marked unverifiable instead of treated
    as a match.  Contract stamp/mode/generation make pairing deterministic.
    """
    contract_by_stamp = {
        (_int(row, "horizon_stamp_sec"), _int(row, "horizon_stamp_nanosec")): row
        for row in contracts
        if _int(row, "horizon_stamp_sec") is not None
        and _int(row, "horizon_stamp_nanosec") is not None
    }
    sorted_overrides = sorted(overrides, key=_time)
    sorted_references = sorted(reference_history, key=_time)
    sorted_pp_debug = sorted(pure_pursuit_debug, key=_time)
    sorted_mux_debug = sorted(mux_debug or [], key=_time)
    sorted_final_controls = sorted(final_control_commands or [], key=_time)
    summaries: list[dict[str, object]] = []
    points: list[dict[str, object]] = []

    for horizon in sorted(horizons, key=_time):
        time_sec = _time(horizon)
        contract = contract_by_stamp.get(
            (_int(horizon, "stamp_sec"), _int(horizon, "stamp_nanosec"))
        )
        active_override = _latest_before(sorted_overrides, time_sec)
        if contract is None:
            if active_override is None:
                continue
            summaries.append(_base_summary(
                time_sec, "unverifiable_contract_missing", active_override, None,
                _nearest_pp_debug(sorted_pp_debug, time_sec),
                _nearest_mux_debug(sorted_mux_debug, time_sec)))
            continue

        source = str(contract.get("source") or "unknown")
        mode_id = _int(contract, "mode_id") or 0
        generation = _int(contract, "override_generation") or 0
        if source != "solver_prediction" or mode_id <= 0 or generation <= 0:
            continue

        override = _latest_matching_override(
            sorted_overrides, time_sec, mode_id, generation)
        pp_debug = _nearest_pp_debug(sorted_pp_debug, time_sec)
        if override is None:
            summaries.append(_base_summary(
                time_sec, "unverifiable_override_missing", None, contract, pp_debug,
                _nearest_mux_debug(sorted_mux_debug, time_sec)))
            continue
        reference = _latest_before(sorted_references, time_sec)
        reference_points = list(reference.get("points", [])) if reference else []
        if len(reference_points) < 2:
            summaries.append(_base_summary(
                time_sec, "unverifiable_reference_missing", override, contract, pp_debug,
                _nearest_mux_debug(sorted_mux_debug, time_sec)))
            continue

        requested_offsets = list(override.get("lateral_offsets_m", []))
        horizon_points = list(horizon.get("points", []))
        compared: list[float] = []
        point_rows: list[dict[str, object]] = []
        first_mismatch_index: int | None = None
        for index, (requested_d, horizon_point) in enumerate(
                zip(requested_offsets, horizon_points)):
            requested = _finite(requested_d)
            actual = _signed_lateral_offset(horizon_point, reference_points)
            if requested is None or actual is None:
                continue
            error = actual - requested
            abs_error = abs(error)
            compared.append(abs_error)
            if first_mismatch_index is None and abs_error > lateral_error_warn_m:
                first_mismatch_index = index
            point_rows.append(
                {
                    "time_sec": time_sec,
                    "mode_id": mode_id,
                    "override_generation": generation,
                    "point_index": index,
                    "planner_lateral_offset_m": requested,
                    "mpc_lateral_offset_m": actual,
                    "lateral_error_m": error,
                    "abs_lateral_error_m": abs_error,
                    "within_tolerance": abs_error <= lateral_error_warn_m,
                    "reference_topic": reference.get("topic") if reference else None,
                }
            )

        summary = _base_summary(
            time_sec, "matched", override, contract, pp_debug,
            _nearest_mux_debug(sorted_mux_debug, time_sec))
        summary["reference_topic"] = reference.get("topic") if reference else None
        summary["horizon_points"] = len(horizon_points)
        summary["compared_points"] = len(compared)
        summary["max_abs_lateral_error_m"] = max(compared) if compared else None
        summary["mean_abs_lateral_error_m"] = (
            sum(compared) / len(compared) if compared else None
        )
        summary["first_mismatch_point_index"] = first_mismatch_index
        if not compared:
            summary["status"] = "unverifiable_no_finite_points"
        elif first_mismatch_index is not None:
            summary["status"] = "mismatch"
        else:
            (execution_status, execution_reason, execution_pp, execution_mux,
             execution_control) = _execution_status(
                sorted_pp_debug, sorted_mux_debug, sorted_final_controls,
                horizon, contract)
            summary["execution_status"] = execution_status
            summary["execution_reason"] = execution_reason
            if execution_pp is not None:
                summary["pp_command_stamp_sec"] = _int(
                    execution_pp, "command_stamp_sec")
                summary["pp_command_stamp_nanosec"] = _int(
                    execution_pp, "command_stamp_nanosec")
            if execution_mux is not None:
                summary["mux_output_stamp_sec"] = _int(
                    execution_mux, "output_stamp_sec")
                summary["mux_output_stamp_nanosec"] = _int(
                    execution_mux, "output_stamp_nanosec")
            if execution_control is not None:
                summary["final_command_speed_mps"] = _finite(
                    execution_control.get("target_speed_mps"))
                summary["final_command_accel_mps2"] = _finite(
                    execution_control.get("accel_mps2"))
                summary["final_command_steer_rad"] = _finite(
                    execution_control.get("steer_rad"))
            if execution_status == "verified":
                summary["status"] = "matched"
            elif execution_status == "not_executed":
                summary["status"] = "published_match_not_executed"
            else:
                summary["status"] = "unverifiable_execution"
        for point_row in point_rows:
            point_row["summary_status"] = summary["status"]
            point_row["execution_status"] = summary["execution_status"]
            points.append(point_row)
        summaries.append(summary)

    return summaries, points


def contract_metrics(rows: Iterable[dict[str, object]]) -> dict[str, float | int | None]:
    rows = list(rows)
    compared = [row for row in rows if _finite(row.get("max_abs_lateral_error_m")) is not None]
    errors = [
        _finite(row.get("max_abs_lateral_error_m"))
        for row in compared
    ]
    finite_errors = [error for error in errors if error is not None]
    return {
        "planner_mpc_contract_available": int(bool(rows)),
        "planner_mpc_contract_comparison_count": len(compared),
        "planner_mpc_contract_mismatch_count": sum(
            row.get("status") == "mismatch" for row in compared
        ),
        "planner_mpc_contract_execution_match_count": sum(
            row.get("status") == "matched" for row in rows
        ),
        "planner_mpc_contract_not_executed_count": sum(
            row.get("status") == "published_match_not_executed" for row in rows
        ),
        "planner_mpc_contract_execution_unverifiable_count": sum(
            row.get("status") == "unverifiable_execution" for row in rows
        ),
        "planner_mpc_contract_max_abs_lateral_error_m": (
            max(finite_errors) if finite_errors else None
        ),
    }


def _base_summary(
    time_sec: float,
    status: str,
    override: dict[str, object] | None,
    contract: dict[str, object] | None,
    pp_debug: dict[str, object] | None,
    mux_debug: dict[str, object] | None,
) -> dict[str, object]:
    return {
        "time_sec": time_sec,
        "status": status,
        "mode_id": _int(contract, "mode_id") if contract else _int(override, "mode_id"),
        "override_generation": (
            _int(contract, "override_generation")
            if contract else _int(override, "override_generation")
        ),
        "horizon_source": contract.get("source") if contract else None,
        "planner_override_points": len(override.get("lateral_offsets_m", [])) if override else 0,
        "horizon_points": 0,
        "compared_points": 0,
        "max_abs_lateral_error_m": None,
        "mean_abs_lateral_error_m": None,
        "first_mismatch_point_index": None,
        "execution_status": "not_checked",
        "execution_reason": None,
        "pp_command_stamp_sec": None,
        "pp_command_stamp_nanosec": None,
        "mux_output_stamp_sec": None,
        "mux_output_stamp_nanosec": None,
        "final_command_speed_mps": None,
        "final_command_accel_mps2": None,
        "final_command_steer_rad": None,
        "pp_mpc_horizon_applied": pp_debug.get("mpc_horizon_applied") if pp_debug else None,
        "pp_trajectory_source": pp_debug.get("trajectory_source") if pp_debug else None,
        "pp_mpc_horizon_reject_reason": (
            pp_debug.get("mpc_horizon_reject_reason") if pp_debug else None
        ),
        "mux_source": mux_debug.get("source") if mux_debug else None,
        "mux_reason": mux_debug.get("reason") if mux_debug else None,
    }


def _execution_status(
    pp_debug_rows: list[dict[str, object]],
    mux_debug_rows: list[dict[str, object]],
    final_control_rows: list[dict[str, object]],
    horizon: dict[str, object],
    contract: dict[str, object],
) -> tuple[
        str, str, dict[str, object] | None, dict[str, object] | None,
        dict[str, object] | None]:
    """Prove an exact horizon -> PP -> mux -> recorded-final-command chain."""
    horizon_stamp = (_int(horizon, "stamp_sec"), _int(horizon, "stamp_nanosec"))
    expected_contract = (
        str(contract.get("source") or "unknown"),
        _int(contract, "mode_id"),
        _int(contract, "override_generation"),
    )
    evaluated_rows = [
        row for row in pp_debug_rows
        if (_int(row, "evaluated_horizon_stamp_sec"),
            _int(row, "evaluated_horizon_stamp_nanosec")) == horizon_stamp
    ]
    if not evaluated_rows:
        return "unverifiable", "pure_pursuit_evaluation_missing", None, None, None

    for pp_debug in evaluated_rows:
        if pp_debug.get("mpc_horizon_applied") is not True:
            return "not_executed", "pure_pursuit_rejected_horizon", pp_debug, None, None
        if pp_debug.get("trajectory_source") != "mpc_horizon":
            return "not_executed", "pure_pursuit_trajectory_source", pp_debug, None, None
        applied_stamp = (
            _int(pp_debug, "applied_horizon_stamp_sec"),
            _int(pp_debug, "applied_horizon_stamp_nanosec"),
        )
        if applied_stamp != horizon_stamp:
            return "not_executed", "pure_pursuit_applied_other_horizon", pp_debug, None, None
        applied_contract = (
            str(pp_debug.get("applied_horizon_source") or "unknown"),
            _int(pp_debug, "applied_horizon_mode_id"),
            _int(pp_debug, "applied_horizon_generation"),
        )
        if applied_contract != expected_contract:
            return (
                "not_executed", "pure_pursuit_applied_contract_mismatch",
                pp_debug, None, None)
        pp_command_stamp = (
            _int(pp_debug, "command_stamp_sec"),
            _int(pp_debug, "command_stamp_nanosec"),
        )
        if None in pp_command_stamp:
            return "unverifiable", "pure_pursuit_command_stamp_missing", pp_debug, None, None
        selected_mux_rows = [
            row for row in mux_debug_rows
            if (_int(row, "selected_input_stamp_sec"),
                _int(row, "selected_input_stamp_nanosec")) == pp_command_stamp
        ]
        if not selected_mux_rows:
            continue
        for mux_debug in selected_mux_rows:
            if (mux_debug.get("selected_source") != "pure_pursuit" or
                    mux_debug.get("source") != "pure_pursuit"):
                return (
                    "not_executed", "hybrid_mux_selected_other_source",
                    pp_debug, mux_debug, None)
            output_stamp = (
                _int(mux_debug, "output_stamp_sec"),
                _int(mux_debug, "output_stamp_nanosec"),
            )
            if None in output_stamp:
                return (
                    "unverifiable", "hybrid_mux_output_stamp_missing", pp_debug,
                    mux_debug, None)
            final_rows = [
                row for row in final_control_rows
                if (_int(row, "command_stamp_sec"),
                    _int(row, "command_stamp_nanosec")) == output_stamp
            ]
            if not final_rows:
                return (
                    "unverifiable", "final_control_exact_join_missing", pp_debug,
                    mux_debug, None)
            final_control = final_rows[-1]
            if not _same_command_payload(mux_debug, final_control):
                return (
                    "unverifiable", "final_control_payload_mismatch", pp_debug,
                    mux_debug, final_control)
            return (
                "verified", "pure_pursuit_horizon_selected_by_mux", pp_debug,
                mux_debug, final_control)
    return "unverifiable", "hybrid_mux_exact_join_missing", None, None, None


def _same_command_payload(
    mux_debug: dict[str, object], final_control: dict[str, object],
) -> bool:
    pairs = (
        ("output_speed_mps", "target_speed_mps"),
        ("output_accel_mps2", "accel_mps2"),
        ("output_steer_rad", "steer_rad"),
    )
    for mux_key, final_key in pairs:
        mux_value = _finite(mux_debug.get(mux_key))
        final_value = _finite(final_control.get(final_key))
        if (mux_value is None or final_value is None or
                not math.isclose(mux_value, final_value, rel_tol=1.0e-6,
                                 abs_tol=1.0e-5)):
            return False
    return True


def _latest_matching_override(
    overrides: list[dict[str, object]], time_sec: float, mode_id: int,
    generation: int,
) -> dict[str, object] | None:
    candidates = [
        row for row in overrides
        if _time(row) <= time_sec and bool(row.get("active"))
        and _int(row, "mode_id") == mode_id
        and _int(row, "override_generation") == generation
    ]
    return candidates[-1] if candidates else None


def _latest_before(rows: list[dict[str, object]], time_sec: float) -> dict[str, object] | None:
    candidates = [row for row in rows if _time(row) <= time_sec]
    return candidates[-1] if candidates else None


def _nearest_pp_debug(rows: list[dict[str, object]], time_sec: float) -> dict[str, object] | None:
    if not rows:
        return None
    nearest = min(rows, key=lambda row: abs(_time(row) - time_sec))
    return nearest if abs(_time(nearest) - time_sec) <= 0.20 else None


def _nearest_mux_debug(rows: list[dict[str, object]], time_sec: float) -> dict[str, object] | None:
    if not rows:
        return None
    nearest = min(rows, key=lambda row: abs(_time(row) - time_sec))
    return nearest if abs(_time(nearest) - time_sec) <= 0.30 else None


def _signed_lateral_offset(
    point: object, reference_points: list[object]) -> float | None:
    if not isinstance(point, tuple) or len(point) < 2:
        return None
    x, y = _finite(point[0]), _finite(point[1])
    if x is None or y is None:
        return None
    best: tuple[float, float] | None = None
    for previous, current in zip(reference_points, reference_points[1:]):
        if not isinstance(previous, tuple) or not isinstance(current, tuple):
            continue
        x0, y0 = _finite(previous[0]), _finite(previous[1])
        x1, y1 = _finite(current[0]), _finite(current[1])
        if None in {x0, y0, x1, y1}:
            continue
        dx, dy = x1 - x0, y1 - y0
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        ratio = min(1.0, max(0.0, ((x - x0) * dx + (y - y0) * dy) / length_sq))
        projected_x, projected_y = x0 + ratio * dx, y0 + ratio * dy
        lateral = (-dy * (x - projected_x) + dx * (y - projected_y)) / math.sqrt(length_sq)
        distance_sq = (x - projected_x) ** 2 + (y - projected_y) ** 2
        if best is None or distance_sq < best[0]:
            best = (distance_sq, lateral)
    return best[1] if best else None


def _time(row: dict[str, object]) -> float:
    return _finite(row.get("time_sec")) or -math.inf


def _int(row: dict[str, object] | None, key: str) -> int | None:
    if row is None:
        return None
    value = _finite(row.get(key))
    if value is None or not float(value).is_integer():
        return None
    return int(value)


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None
