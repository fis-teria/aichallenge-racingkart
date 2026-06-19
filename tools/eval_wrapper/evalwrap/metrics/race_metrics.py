from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path

from evalwrap.metrics.event_detector import Event, events_from_counts, events_from_log_excerpts, events_from_rosbag
from evalwrap.metrics.judgement import judge_domain
from evalwrap.parsers.details_parser import ParsedDetails
from evalwrap.parsers.log_parser import ParsedLogs
from evalwrap.parsers.rosbag_parser import parse_rosbag
from evalwrap.parsers.summary_parser import ParsedSummary
from evalwrap.reference_trajectory import ReferenceTrajectory


@dataclass
class DomainMetrics:
    finish: bool | None = None
    total_time_sec: float | None = None
    lap_count: int | None = None
    best_lap_sec: float | None = None
    avg_lap_sec: float | None = None
    penalty_count: int = 0
    collision_count: int = 0
    stuck_count: int = 0
    low_speed_time_sec: float | None = None
    max_speed_mps: float | None = None
    avg_speed_mps: float | None = None
    max_abs_steer_rad: float | None = None
    steer_oscillation_score: float | None = None
    max_accel_mps2: float | None = None
    max_decel_mps2: float | None = None
    avg_path_error_m: float | None = None
    max_path_error_m: float | None = None
    trajectory_source: str | None = None
    rosbag_available: bool = False
    rosbag_reason: str | None = None
    judgement: str = "needs_more_eval"
    warnings: list[str] | None = None


@dataclass
class DomainResult:
    domain_id: str
    metrics: DomainMetrics
    lap_times: list[float]
    events: list[Event]
    log_excerpts: list[dict[str, object]]
    vehicle_timeseries: list[dict[str, object]]
    control_timeseries: list[dict[str, object]]
    section_summary: list[dict[str, object]]
    corner_summary: list[dict[str, object]]
    trajectory_reference: list[dict[str, object]]


def build_domain_result(
    run_id: str,
    domain_id: str,
    summary: ParsedSummary,
    details: ParsedDetails,
    logs: ParsedLogs,
    raw_domain,
    thresholds: dict[str, float] | None = None,
    reference_trajectory: ReferenceTrajectory | None = None,
) -> DomainResult:
    lap_times = details.lap_times or []
    if not lap_times and summary.best_lap_sec is not None and summary.lap_count == 1:
        lap_times = [summary.best_lap_sec]

    penalty_count = _first_int(summary.penalty_count, details.penalty_count, default=0)
    collision_count = _first_int(summary.collision_count, details.collision_count, default=0)
    best_lap = summary.best_lap_sec
    avg_lap = summary.avg_lap_sec
    if lap_times:
        best_lap = min(lap_times)
        avg_lap = sum(lap_times) / len(lap_times)

    fallback_points = reference_trajectory.points if reference_trajectory and reference_trajectory.points else None
    fallback_source = None
    if reference_trajectory and fallback_points:
        fallback_source = _reference_trajectory_source(reference_trajectory)
    rosbag = parse_rosbag(
        raw_domain,
        thresholds=thresholds,
        fallback_trajectory_points=fallback_points,
        fallback_trajectory_source=fallback_source,
    )
    reference_warnings = reference_trajectory.warnings if reference_trajectory else []
    warnings = [*summary.warnings, *details.warnings, *logs.warnings, *reference_warnings, *rosbag.warnings]
    if not rosbag.available and rosbag.reason:
        warnings.append(rosbag.reason)
    rosbag_metrics = rosbag.metrics
    metrics = DomainMetrics(
        finish=summary.finish,
        total_time_sec=summary.total_time_sec,
        lap_count=summary.lap_count or (len(lap_times) if lap_times else None),
        best_lap_sec=best_lap,
        avg_lap_sec=avg_lap,
        penalty_count=penalty_count,
        collision_count=collision_count,
        stuck_count=int(rosbag_metrics.get("stuck_count") or 0),
        low_speed_time_sec=_optional_float(rosbag_metrics.get("low_speed_time_sec")),
        max_speed_mps=_optional_float(rosbag_metrics.get("max_speed_mps")),
        avg_speed_mps=_optional_float(rosbag_metrics.get("avg_speed_mps")),
        max_abs_steer_rad=_optional_float(rosbag_metrics.get("max_abs_steer_rad")),
        steer_oscillation_score=_optional_float(rosbag_metrics.get("steer_oscillation_score")),
        max_accel_mps2=_optional_float(rosbag_metrics.get("max_accel_mps2")),
        max_decel_mps2=_optional_float(rosbag_metrics.get("max_decel_mps2")),
        avg_path_error_m=_optional_float(rosbag_metrics.get("avg_path_error_m")),
        max_path_error_m=_optional_float(rosbag_metrics.get("max_path_error_m")),
        trajectory_source=rosbag.trajectory_source,
        rosbag_available=rosbag.available,
        rosbag_reason=rosbag.reason,
        judgement=judge_domain(summary.finish, summary.total_time_sec, penalty_count, collision_count),
        warnings=warnings,
    )
    events = events_from_counts(run_id, domain_id, penalty_count, collision_count)
    events.extend(events_from_log_excerpts(run_id, domain_id, logs.excerpts))
    events.extend(events_from_rosbag(run_id, domain_id, rosbag.events))
    return DomainResult(
        domain_id=domain_id,
        metrics=metrics,
        lap_times=lap_times,
        events=events,
        log_excerpts=[asdict(item) for item in logs.excerpts],
        vehicle_timeseries=[dict(item) for item in rosbag.vehicle_timeseries],
        control_timeseries=[dict(item) for item in rosbag.control_timeseries],
        section_summary=[dict(item) for item in rosbag.section_summary],
        corner_summary=[dict(item) for item in rosbag.corner_summary],
        trajectory_reference=[dict(item) for item in rosbag.trajectory_reference],
    )


def write_processed_outputs(run_id: str, domains: list[DomainResult], processed_dir: Path) -> dict[str, object]:
    processed_dir.mkdir(parents=True, exist_ok=True)
    metrics = {
        "run_id": run_id,
        "domains": {domain.domain_id: asdict(domain.metrics) for domain in domains},
        "log_excerpts": {domain.domain_id: domain.log_excerpts for domain in domains},
        "events": {domain.domain_id: [event.to_dict() for event in domain.events] for domain in domains},
    }
    (processed_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")
    _write_lap_summary(run_id, domains, processed_dir / "lap_summary.csv")
    _write_section_summary(run_id, domains, processed_dir / "section_summary.csv")
    _write_corner_summary(run_id, domains, processed_dir / "corner_summary.csv")
    _write_trajectory_reference(run_id, domains, processed_dir / "trajectory_reference.csv")
    _write_vehicle_timeseries(run_id, domains, processed_dir / "vehicle_timeseries.csv")
    _write_control_timeseries(run_id, domains, processed_dir / "control_timeseries.csv")
    _write_motion_log(run_id, domains, processed_dir / "motion_log.csv")
    _write_events(domains, processed_dir / "events.csv")
    return metrics


def _first_int(*values: int | None, default: int) -> int:
    for value in values:
        if value is not None:
            return int(value)
    return default


def _write_lap_summary(run_id: str, domains: list[DomainResult], path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "run_id",
                "domain_id",
                "lap",
                "lap_time_sec",
                "avg_speed_mps",
                "max_speed_mps",
                "penalty_count",
                "collision_count",
            ]
        )
        for domain in domains:
            for idx, lap_time in enumerate(domain.lap_times, start=1):
                writer.writerow(
                    [
                        run_id,
                        domain.domain_id,
                        idx,
                        lap_time,
                        "",
                        "",
                        domain.metrics.penalty_count,
                        domain.metrics.collision_count,
                    ]
                )


def _write_section_summary(run_id: str, domains: list[DomainResult], path: Path) -> None:
    fieldnames = [
        "run_id",
        "domain_id",
        "lap",
        "section",
        "entry_time_sec",
        "exit_time_sec",
        "duration_sec",
        "avg_speed_mps",
        "max_speed_mps",
        "min_speed_mps",
        "event_count",
        "avg_path_error_m",
        "max_path_error_m",
        "distance_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for domain in domains:
            for row in domain.section_summary:
                writer.writerow(_row_with_run_domain(row, run_id, domain.domain_id, fieldnames))


def _write_vehicle_timeseries(run_id: str, domains: list[DomainResult], path: Path) -> None:
    fieldnames = [
        "run_id",
        "domain_id",
        "time_sec",
        "x_m",
        "y_m",
        "distance_m",
        "section",
        "corner_id",
        "track_s_m",
        "trajectory_curvature_1pm",
        "speed_mps",
        "acceleration_mps2",
        "steering_rad",
        "yaw_rate_rps",
        "path_error_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for domain in domains:
            for row in domain.vehicle_timeseries:
                writer.writerow(_row_with_run_domain(row, run_id, domain.domain_id, fieldnames))


def _write_corner_summary(run_id: str, domains: list[DomainResult], path: Path) -> None:
    fieldnames = [
        "run_id",
        "domain_id",
        "corner_id",
        "pass",
        "entry_time_sec",
        "exit_time_sec",
        "duration_sec",
        "entry_speed_mps",
        "min_speed_mps",
        "avg_speed_mps",
        "max_speed_mps",
        "exit_speed_mps",
        "avg_path_error_m",
        "max_path_error_m",
        "entry_distance_m",
        "exit_distance_m",
        "start_track_s_m",
        "end_track_s_m",
        "corner_length_m",
        "peak_curvature_1pm",
        "event_count",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for domain in domains:
            for row in domain.corner_summary:
                writer.writerow(_row_with_run_domain(row, run_id, domain.domain_id, fieldnames))


def _write_trajectory_reference(run_id: str, domains: list[DomainResult], path: Path) -> None:
    fieldnames = [
        "run_id",
        "domain_id",
        "point_index",
        "x_m",
        "y_m",
        "track_s_m",
        "trajectory_curvature_1pm",
        "corner_id",
        "trajectory_source",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for domain in domains:
            for row in domain.trajectory_reference:
                writer.writerow(_row_with_run_domain(row, run_id, domain.domain_id, fieldnames))


def _write_control_timeseries(run_id: str, domains: list[DomainResult], path: Path) -> None:
    fieldnames = [
        "run_id",
        "domain_id",
        "time_sec",
        "target_speed_mps",
        "accel_mps2",
        "steer_rad",
        "throttle",
        "brake",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for domain in domains:
            for row in domain.control_timeseries:
                writer.writerow(_row_with_run_domain(row, run_id, domain.domain_id, fieldnames))


def _write_motion_log(run_id: str, domains: list[DomainResult], path: Path) -> None:
    fieldnames = [
        "run_id",
        "domain_id",
        "time_sec",
        "speed_mps",
        "acceleration_mps2",
        "steering_rad",
        "target_speed_mps",
        "command_accel_mps2",
        "command_steer_rad",
        "throttle",
        "brake",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for domain in domains:
            if domain.vehicle_timeseries:
                for vehicle in domain.vehicle_timeseries:
                    time_sec = _optional_float(vehicle.get("time_sec"))
                    control = _nearest_control_row(domain.control_timeseries, time_sec)
                    writer.writerow(
                        {
                            "run_id": run_id,
                            "domain_id": domain.domain_id,
                            "time_sec": vehicle.get("time_sec", ""),
                            "speed_mps": vehicle.get("speed_mps", ""),
                            "acceleration_mps2": vehicle.get("acceleration_mps2", ""),
                            "steering_rad": vehicle.get("steering_rad", ""),
                            "target_speed_mps": control.get("target_speed_mps", "") if control else "",
                            "command_accel_mps2": control.get("accel_mps2", "") if control else "",
                            "command_steer_rad": control.get("steer_rad", "") if control else "",
                            "throttle": control.get("throttle", "") if control else "",
                            "brake": control.get("brake", "") if control else "",
                        }
                    )
                continue

            for control in domain.control_timeseries:
                writer.writerow(
                    {
                        "run_id": run_id,
                        "domain_id": domain.domain_id,
                        "time_sec": control.get("time_sec", ""),
                        "speed_mps": "",
                        "acceleration_mps2": "",
                        "steering_rad": "",
                        "target_speed_mps": control.get("target_speed_mps", ""),
                        "command_accel_mps2": control.get("accel_mps2", ""),
                        "command_steer_rad": control.get("steer_rad", ""),
                        "throttle": control.get("throttle", ""),
                        "brake": control.get("brake", ""),
                    }
                )


def _nearest_control_row(rows: list[dict[str, object]], time_sec: float | None, tolerance_sec: float = 0.25) -> dict[str, object] | None:
    if time_sec is None or not rows:
        return None
    best: dict[str, object] | None = None
    best_delta = tolerance_sec
    for row in rows:
        row_time = _optional_float(row.get("time_sec"))
        if row_time is None:
            continue
        delta = abs(row_time - time_sec)
        if delta <= best_delta:
            best = row
            best_delta = delta
    return best


def _write_events(domains: list[DomainResult], path: Path) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=["run_id", "domain_id", "time_sec", "lap", "section", "event_type", "severity", "description"],
        )
        writer.writeheader()
        for domain in domains:
            for event in domain.events:
                writer.writerow(event.to_dict())


def _row_with_run_domain(row: dict[str, object], run_id: str, domain_id: str, fieldnames: list[str]) -> dict[str, object]:
    output = {field: row.get(field, "") for field in fieldnames}
    output["run_id"] = run_id
    output["domain_id"] = domain_id
    return output


def _optional_float(value: object) -> float | None:
    return float(value) if isinstance(value, (int, float)) else None


def _reference_trajectory_source(reference: ReferenceTrajectory) -> str:
    if reference.csv_path is None:
        return reference.source
    return f"{reference.source}:{reference.csv_path}"
