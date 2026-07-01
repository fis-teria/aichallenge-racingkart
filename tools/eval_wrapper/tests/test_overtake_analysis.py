from __future__ import annotations

import csv
import json
from pathlib import Path
from types import SimpleNamespace

from evalwrap.analysis.overtake.event_extractor import extract_overtake_attempts
from evalwrap.analysis.overtake.metrics import build_overtake_outputs
from evalwrap.analysis.overtake.opportunity_detector import detect_missed_overtake_chances
from evalwrap.metrics.race_metrics import DomainMetrics
from evalwrap.reports.overtake_report import generate_overtake_report


CONFIG = {
    "success": {"t_pass_max_sec": 8.0, "major_slack_threshold": 0.20, "pass_margin_m": 1.0},
    "blocked": {
        "blocked_distance_threshold_m": 8.0,
        "speed_loss_threshold_mps": 0.8,
        "relative_speed_threshold_mps": 0.5,
    },
    "opportunity": {
        "curvature_overtake_threshold": 0.08,
        "cbf_h_safe_threshold": 0.5,
        "min_opportunity_duration_sec": 1.0,
    },
    "safety": {"cbf_h_warn": 0.2},
    "map": {"bin_width_m": 5.0},
    "judgement": {
        "candidate_success_rate": 0.60,
        "attack_success_rate": 0.50,
        "small_regression_allowance_sec": 0.5,
        "regression_limit_sec": 1.5,
        "infeasible_limit": 2,
    },
}


def test_extract_overtake_attempts_classifies_success_from_returning_transition() -> None:
    rows = [
        {"timestamp_sec": 0.0, "overtake_state": "FOLLOWING", "ego_speed_mps": 4.0},
        {"timestamp_sec": 1.0, "overtake_state": "PREPARE_OVERTAKE_LEFT", "front_vehicle_id": "d2"},
        {"timestamp_sec": 2.0, "overtake_state": "OVERTAKE_LEFT", "front_vehicle_id": "d2", "min_cbf_h": 0.8},
        {"timestamp_sec": 3.0, "overtake_state": "MERGE_BACK", "front_vehicle_id": "d2", "min_cbf_h": 0.7},
        {"timestamp_sec": 4.0, "overtake_state": "FREE_RUN", "front_vehicle_id": "d2"},
    ]

    attempts = extract_overtake_attempts(rows, CONFIG)

    assert len(attempts) == 1
    assert attempts[0]["result"] == "success"
    assert attempts[0]["target_vehicle_id"] == "d2"
    assert attempts[0]["time_to_pass_sec"] == 2.0
    assert attempts[0]["return_to_line_time_sec"] == 1.0


def test_extract_overtake_attempts_classifies_cbf_abort() -> None:
    rows = [
        {"timestamp_sec": 1.0, "overtake_state": "OVERTAKE_PREP", "min_cbf_h": 0.6},
        {"timestamp_sec": 2.0, "overtake_state": "OVERTAKING", "cbf_slack": 0.04},
        {"timestamp_sec": 3.0, "overtake_state": "ABORTED", "cbf_slack": 0.04},
    ]

    attempts = extract_overtake_attempts(rows, CONFIG)

    assert attempts[0]["result"] == "aborted"
    assert attempts[0]["abort_reason"] == "cbf_too_close"


def test_detect_missed_overtake_chance_requires_blocked_speed_loss_and_safe_context() -> None:
    rows = [
        {
            "timestamp_sec": 0.0,
            "overtake_state": "FOLLOWING",
            "front_distance_m": 5.0,
            "ego_speed_mps": 4.0,
            "target_speed_mps": 6.0,
            "relative_speed_mps": 0.1,
            "reference_curvature": 0.02,
            "min_cbf_h": 0.9,
            "target_lateral_offset_m": 0.8,
        },
        {
            "timestamp_sec": 1.2,
            "overtake_state": "FOLLOWING",
            "front_distance_m": 5.2,
            "ego_speed_mps": 4.1,
            "target_speed_mps": 6.0,
            "relative_speed_mps": 0.0,
            "reference_curvature": 0.02,
            "min_cbf_h": 0.9,
            "target_lateral_offset_m": 0.8,
        },
    ]

    missed = detect_missed_overtake_chances(rows, CONFIG)

    assert len(missed) == 1
    assert missed[0]["duration_sec"] == 1.2


def test_build_overtake_outputs_writes_processed_files(tmp_path: Path) -> None:
    domain = SimpleNamespace(
        domain_id="d1",
        metrics=DomainMetrics(finish=True),
        vehicle_timeseries=[
            {"time_sec": 0.0, "x_m": 0.0, "y_m": 0.0, "track_s_m": 0.0, "section": 1, "speed_mps": 4.0},
            {"time_sec": 1.0, "x_m": 1.0, "y_m": 0.0, "track_s_m": 1.0, "section": 1, "speed_mps": 4.5},
            {"time_sec": 2.0, "x_m": 2.0, "y_m": 0.0, "track_s_m": 2.0, "section": 1, "speed_mps": 5.0},
            {"time_sec": 3.0, "x_m": 3.0, "y_m": 0.0, "track_s_m": 3.0, "section": 1, "speed_mps": 5.0},
        ],
        speed_profile_debug_timeseries=[
            {"time_sec": 0.0, "target_speed_mps": 6.0, "mpc_status": "solved", "mpc_solve_time_ms": 4.0},
            {"time_sec": 1.0, "target_speed_mps": 6.0, "mpc_status": "solved", "mpc_solve_time_ms": 4.0},
        ],
        overtake_debug_timeseries=[
            {"time_sec": 0.0, "mode": "FOLLOW_BLOCKED", "front_delta_s": 5.0, "front_vehicle_id": "d2"},
            {
                "time_sec": 1.0,
                "mode": "PREPARE_OVERTAKE_LEFT",
                "selected": "PASS_LEFT",
                "front_delta_s": 4.0,
                "front_vehicle_id": "d2",
                "attempt_id": 1,
                "active_override": True,
                "pass_gap_reason": "ok",
                "min_cbf_h": 0.8,
            },
            {
                "time_sec": 2.0,
                "mode": "OVERTAKE_LEFT",
                "selected": "PASS_LEFT",
                "front_delta_s": 2.0,
                "front_vehicle_id": "d2",
                "attempt_id": 1,
                "active_override": True,
                "pass_gap_reason": "ok",
                "min_cbf_h": 0.7,
            },
            {"time_sec": 3.0, "mode": "MERGE_BACK", "front_delta_s": 8.0, "front_vehicle_id": "d2", "attempt_id": 1, "min_cbf_h": 0.9},
            {"time_sec": 3.2, "mode": "FREE_RUN", "front_vehicle_id": "d2", "attempt_id": 1},
        ],
    )

    result = build_overtake_outputs("run", [domain], tmp_path, CONFIG)

    assert result["domains"]["d1"]["attempt_count"] == 1
    assert result["domains"]["d1"]["success_count"] == 1
    assert (tmp_path / "overtake_metrics.json").exists()
    with (tmp_path / "overtake_attempts.csv").open("r", encoding="utf-8", newline="") as handle:
        attempts = list(csv.DictReader(handle))
    assert attempts[0]["result"] == "success"
    metrics = json.loads((tmp_path / "overtake_metrics.json").read_text(encoding="utf-8"))
    assert metrics["domains"]["d1"]["analysis_available"] is True
    with (tmp_path / "overtake_timeseries.csv").open("r", encoding="utf-8", newline="") as handle:
        timeseries = list(csv.DictReader(handle))
    assert timeseries[1]["selected"] == "PASS_LEFT"
    assert timeseries[1]["active_override"] == "True"
    assert timeseries[1]["pass_gap_reason"] == "ok"


def test_generate_overtake_report_surfaces_decision_metrics(tmp_path: Path) -> None:
    processed = tmp_path / "processed"
    processed.mkdir()
    (processed / "overtake_metrics.json").write_text(
        json.dumps(
            {
                "run_id": "run",
                "domains": {
                    "d1": {
                        "analysis_available": True,
                        "attempt_count": 1,
                        "success_count": 0,
                        "success_rate": 0.0,
                        "failure_rate": 0.0,
                        "blocked_time_sec": 1.2,
                        "missed_overtake_chance_count": 0,
                        "collision_count": 0,
                        "penalty_count": 0,
                        "min_vehicle_distance_m": 4.0,
                        "min_cbf_h": 0.8,
                        "max_cbf_slack": 0.0,
                        "mpc_infeasible_count": 0,
                        "abort_reason_counts": {},
                        "judgement": "candidate",
                    }
                },
                "warnings": [],
            }
        ),
        encoding="utf-8",
    )
    _write_csv(processed / "overtake_attempts.csv", ["domain_id", "attempt_id", "result"], [])
    _write_csv(
        processed / "overtake_timeseries.csv",
        [
            "domain_id",
            "timestamp_sec",
            "s",
            "ego_x",
            "ego_y",
            "overtake_state",
            "selected",
            "blocked",
            "side_by_side",
            "front_vehicle_id",
            "front_distance_m",
            "front_delta_d",
            "can_pass_left",
            "can_pass_right",
            "pass_gap_reason",
            "active_override",
            "reason",
            "min_cbf_h",
        ],
        [
            {
                "domain_id": "d1",
                "timestamp_sec": 1.0,
                "s": 12.0,
                "ego_x": 1.0,
                "ego_y": 2.0,
                "overtake_state": "FOLLOWING",
                "selected": "FOLLOW",
                "blocked": "true",
                "side_by_side": "false",
                "front_vehicle_id": "d2",
                "front_distance_m": 5.0,
                "front_delta_d": 0.1,
                "can_pass_left": "true",
                "can_pass_right": "false",
                "pass_gap_reason": "right_gap_narrow",
                "active_override": "true",
                "reason": "",
                "min_cbf_h": 0.9,
            },
            {
                "domain_id": "d1",
                "timestamp_sec": 2.0,
                "s": 16.0,
                "ego_x": 2.0,
                "ego_y": 2.0,
                "overtake_state": "OVERTAKING",
                "selected": "PASS_LEFT",
                "blocked": "true",
                "side_by_side": "false",
                "front_vehicle_id": "d2",
                "front_distance_m": 4.0,
                "front_delta_d": 0.2,
                "can_pass_left": "true",
                "can_pass_right": "false",
                "pass_gap_reason": "right_gap_narrow",
                "active_override": "true",
                "reason": "",
                "min_cbf_h": 0.8,
            },
        ],
    )
    _write_csv(processed / "overtake_map_bins.csv", ["domain_id", "bin_id"], [])

    report = generate_overtake_report(tmp_path)

    assert report is not None
    html = report.read_text(encoding="utf-8")
    assert "Decision Summary" in html
    assert "Decision Samples" in html
    assert "PASS_LEFT" in html
    assert "right_gap_narrow" in html
    assert "active_override_samples" in html


def test_generate_overtake_report_keeps_single_vehicle_debug_quiet(tmp_path: Path) -> None:
    processed = tmp_path / "processed"
    processed.mkdir()
    (processed / "overtake_metrics.json").write_text(
        json.dumps(
            {
                "run_id": "run",
                "domains": {
                    "d1": {
                        "analysis_available": True,
                        "attempt_count": 0,
                        "success_count": 0,
                        "success_rate": 0.0,
                        "failure_rate": 0.0,
                        "blocked_time_sec": 0.0,
                        "missed_overtake_chance_count": 0,
                        "collision_count": 0,
                        "penalty_count": 0,
                        "min_vehicle_distance_m": None,
                        "min_cbf_h": None,
                        "max_cbf_slack": None,
                        "mpc_infeasible_count": 0,
                        "abort_reason_counts": {},
                        "judgement": "no_attempt",
                    }
                },
                "warnings": [],
            }
        ),
        encoding="utf-8",
    )
    _write_csv(processed / "overtake_attempts.csv", ["domain_id", "attempt_id", "result"], [])
    _write_csv(
        processed / "overtake_timeseries.csv",
        ["domain_id", "timestamp_sec", "selected", "blocked", "side_by_side", "pass_gap_reason", "active_override"],
        [
            {
                "domain_id": "d1",
                "timestamp_sec": 1.0,
                "selected": "FASTEST",
                "blocked": "false",
                "side_by_side": "false",
                "pass_gap_reason": "no_target",
                "active_override": "false",
            }
        ],
    )
    _write_csv(processed / "overtake_map_bins.csv", ["domain_id", "bin_id"], [])

    report = generate_overtake_report(tmp_path)

    assert report is not None
    html = report.read_text(encoding="utf-8")
    assert "No overtake scene observed" in html
    assert "Decision Samples" not in html


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
