from __future__ import annotations

import math
import struct
from pathlib import Path

from evalwrap.parsers.rosbag_parser import (
    _apply_corner_id_rotation,
    _decode_ackermann_control_command,
    build_analysis_from_series,
    parse_rosbag,
)


def test_build_analysis_from_series_generates_timeseries_metrics_sections_and_events() -> None:
    odometry = [
        {"time_sec": 0.0, "x_m": 0.0, "y_m": 0.0, "speed_mps": 0.2, "yaw_rate_rps": 0.0},
        {"time_sec": 1.0, "x_m": 1.0, "y_m": 0.2, "speed_mps": 0.1, "yaw_rate_rps": 0.0},
        {"time_sec": 2.0, "x_m": 2.0, "y_m": 0.4, "speed_mps": 0.1, "yaw_rate_rps": 0.0},
        {"time_sec": 3.0, "x_m": 3.0, "y_m": 2.0, "speed_mps": 1.5, "yaw_rate_rps": 0.0},
        {"time_sec": 4.0, "x_m": 4.0, "y_m": 0.5, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
    ]
    acceleration = [
        {"time_sec": 0.0, "acceleration_mps2": 0.0},
        {"time_sec": 1.0, "acceleration_mps2": 1.2},
        {"time_sec": 2.0, "acceleration_mps2": -1.8},
        {"time_sec": 3.0, "acceleration_mps2": 0.4},
        {"time_sec": 4.0, "acceleration_mps2": 0.1},
    ]
    control = [
        {"time_sec": 0.0, "target_speed_mps": 3.0, "accel_mps2": 0.0, "steer_rad": 0.0},
        {"time_sec": 1.0, "target_speed_mps": 3.0, "accel_mps2": 1.2, "steer_rad": 0.7},
        {"time_sec": 2.0, "target_speed_mps": 3.0, "accel_mps2": -1.8, "steer_rad": -0.7},
        {"time_sec": 3.0, "target_speed_mps": 3.0, "accel_mps2": 0.4, "steer_rad": 0.2},
    ]
    trajectory = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]

    parsed = build_analysis_from_series(
        odometry=odometry,
        acceleration=acceleration,
        control=control,
        trajectory_points=trajectory,
        thresholds={
            "low_speed_mps": 0.5,
            "stuck_duration_sec": 2.0,
            "hard_brake_mps2": -1.5,
            "high_accel_mps2": 1.0,
            "steer_oscillation_warn": 0.8,
            "section_count": 2,
            "sync_tolerance_sec": 0.1,
            "path_error_warn_m": 1.0,
        },
    )

    assert parsed.available
    assert parsed.metrics["stuck_count"] == 1
    assert parsed.metrics["max_speed_mps"] == 2.0
    assert parsed.metrics["max_decel_mps2"] == -1.8
    assert parsed.metrics["max_path_error_m"] == 2.0
    assert len(parsed.vehicle_timeseries) == 5
    assert len(parsed.control_timeseries) == 4
    assert {row["section"] for row in parsed.section_summary} == {1, 2}
    event_types = {event["event_type"] for event in parsed.events}
    assert {"stuck", "hard_brake", "high_accel", "steer_oscillation", "path_deviation"}.issubset(event_types)


def test_build_analysis_from_series_preserves_overtake_debug_rows() -> None:
    parsed = build_analysis_from_series(
        odometry=[{"time_sec": 1.0, "x_m": 0.0, "y_m": 0.0, "speed_mps": 1.0, "yaw_rate_rps": 0.0}],
        overtake_debug=[
            {
                "time_sec": 1.0,
                "mode": "OVERTAKE_LEFT",
                "front_delta_s": 4.0,
                "front_vehicle_id": "d2",
            }
        ],
    )

    assert parsed.available
    assert parsed.overtake_debug_timeseries[0]["mode"] == "OVERTAKE_LEFT"
    assert parsed.overtake_debug_timeseries[0]["front_vehicle_id"] == "d2"


def test_parse_rosbag_gracefully_handles_missing_storage(tmp_path: Path) -> None:
    parsed = parse_rosbag(tmp_path)

    assert not parsed.available
    assert parsed.reason == "rosbag2_autoware storage not found"


def test_build_analysis_from_series_generates_awsim_section_summary() -> None:
    odometry = [
        {"time_sec": 10.0, "x_m": 0.0, "y_m": 0.0, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
        {"time_sec": 11.0, "x_m": 2.0, "y_m": 0.0, "speed_mps": 3.0, "yaw_rate_rps": 0.0},
        {"time_sec": 12.0, "x_m": 5.0, "y_m": 0.0, "speed_mps": 4.0, "yaw_rate_rps": 0.0},
        {"time_sec": 13.0, "x_m": 9.0, "y_m": 0.0, "speed_mps": 5.0, "yaw_rate_rps": 0.0},
    ]
    awsim_status = [
        {"time_sec": 10.0, "vehicle_index": 1, "lap": 1, "lap_time_sec": 0.0, "section": 1},
        {"time_sec": 11.0, "vehicle_index": 1, "lap": 1, "lap_time_sec": 1.0, "section": 1},
        {"time_sec": 12.0, "vehicle_index": 1, "lap": 1, "lap_time_sec": 2.0, "section": 2},
        {"time_sec": 13.0, "vehicle_index": 1, "lap": 1, "lap_time_sec": 3.0, "section": 2},
    ]

    parsed = build_analysis_from_series(odometry=odometry, awsim_status=awsim_status)

    assert parsed.available
    assert len(parsed.awsim_section_summary) == 2
    first = parsed.awsim_section_summary[0]
    assert first["lap"] == 1
    assert first["section"] == 1
    assert first["entry_time_sec"] == 0.0
    assert first["exit_time_sec"] == 2.0
    assert first["entry_lap_time_sec"] == 0.0
    assert first["exit_lap_time_sec"] == 2.0
    assert first["duration_sec"] == 2.0
    assert first["avg_speed_mps"] == 3.0


def test_awsim_section_summary_keeps_previous_lap_exit_time_at_lap_rollover() -> None:
    odometry = [
        {"time_sec": 20.0, "x_m": 0.0, "y_m": 0.0, "speed_mps": 4.0, "yaw_rate_rps": 0.0},
        {"time_sec": 21.0, "x_m": 4.0, "y_m": 0.0, "speed_mps": 4.0, "yaw_rate_rps": 0.0},
        {"time_sec": 22.0, "x_m": 8.0, "y_m": 0.0, "speed_mps": 4.0, "yaw_rate_rps": 0.0},
    ]
    awsim_status = [
        {"time_sec": 20.0, "vehicle_index": 1, "lap": 1, "lap_time_sec": 8.0, "section": 9},
        {"time_sec": 21.0, "vehicle_index": 1, "lap": 1, "lap_time_sec": 9.0, "section": 9},
        {"time_sec": 22.0, "vehicle_index": 1, "lap": 2, "lap_time_sec": 0.2, "section": 1},
    ]

    parsed = build_analysis_from_series(odometry=odometry, awsim_status=awsim_status)

    assert parsed.available
    final_previous_lap_section = parsed.awsim_section_summary[0]
    assert final_previous_lap_section["lap"] == 1
    assert final_previous_lap_section["section"] == 9
    assert final_previous_lap_section["exit_time_sec"] == 2.0
    assert final_previous_lap_section["exit_lap_time_sec"] == 10.0


def test_decode_ackermann_control_command_from_cdr_bytes() -> None:
    raw = bytearray(48)
    raw[0:4] = b"\x00\x01\x00\x00"
    struct.pack_into("<f", raw, 20, -0.5)
    struct.pack_into("<f", raw, 24, 1.25)
    struct.pack_into("<f", raw, 36, 7.25)
    struct.pack_into("<f", raw, 40, -0.125)
    struct.pack_into("<f", raw, 44, 0.0)

    decoded = _decode_ackermann_control_command(bytes(raw), 12.5)

    assert decoded == {
        "time_sec": 12.5,
        "target_speed_mps": 7.25,
        "accel_mps2": -0.125,
        "steer_rad": -0.5,
        "throttle": None,
        "brake": None,
    }


def test_build_analysis_from_series_generates_corner_summary_from_trajectory_curvature() -> None:
    trajectory = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 1.0), (3.0, 2.0), (3.0, 3.0)]
    odometry = [
        {"time_sec": 0.0, "x_m": 0.0, "y_m": 0.0, "speed_mps": 4.0, "yaw_rate_rps": 0.0},
        {"time_sec": 1.0, "x_m": 1.0, "y_m": 0.0, "speed_mps": 3.5, "yaw_rate_rps": 0.0},
        {"time_sec": 2.0, "x_m": 2.0, "y_m": 0.0, "speed_mps": 2.5, "yaw_rate_rps": 0.0},
        {"time_sec": 3.0, "x_m": 3.0, "y_m": 1.0, "speed_mps": 1.5, "yaw_rate_rps": 0.0},
        {"time_sec": 4.0, "x_m": 3.0, "y_m": 2.0, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
        {"time_sec": 5.0, "x_m": 3.0, "y_m": 3.0, "speed_mps": 3.0, "yaw_rate_rps": 0.0},
    ]

    parsed = build_analysis_from_series(
        odometry=odometry,
        trajectory_points=trajectory,
        thresholds={
            "corner_curvature_min_1pm": 0.1,
            "corner_min_length_m": 0.5,
            "corner_merge_gap_m": 0.0,
            "corner_padding_m": 1.0,
        },
    )

    assert parsed.corner_summary
    first_corner = parsed.corner_summary[0]
    assert first_corner["corner_id"] == "corner_01"
    assert first_corner["pass"] == 1
    assert first_corner["entry_speed_mps"] == 3.5
    assert first_corner["min_speed_mps"] == 1.5
    assert first_corner["exit_speed_mps"] == 2.0
    assert first_corner["duration_sec"] == 3.0
    assert any(row.get("corner_id") == "corner_01" for row in parsed.vehicle_timeseries)
    assert parsed.trajectory_reference
    assert parsed.trajectory_reference[0]["point_index"] == 0


def test_build_analysis_from_series_preserves_trajectory_source_for_reference_rows() -> None:
    parsed = build_analysis_from_series(
        odometry=[
            {"time_sec": 0.0, "x_m": 0.0, "y_m": 0.0, "speed_mps": 1.0, "yaw_rate_rps": 0.0},
            {"time_sec": 1.0, "x_m": 1.0, "y_m": 0.0, "speed_mps": 1.0, "yaw_rate_rps": 0.0},
            {"time_sec": 2.0, "x_m": 1.0, "y_m": 1.0, "speed_mps": 1.0, "yaw_rate_rps": 0.0},
        ],
        trajectory_points=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        trajectory_source="mpc_csv:/tmp/ref.csv",
    )

    assert parsed.trajectory_source == "mpc_csv:/tmp/ref.csv"
    assert {row["trajectory_source"] for row in parsed.trajectory_reference} == {"mpc_csv:/tmp/ref.csv"}


def test_build_analysis_from_series_computes_grade_from_odometry_z() -> None:
    parsed = build_analysis_from_series(
        odometry=[
            {"time_sec": 0.0, "x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
            {"time_sec": 1.0, "x_m": 1.0, "y_m": 0.0, "z_m": 0.1, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
            {"time_sec": 2.0, "x_m": 2.0, "y_m": 0.0, "z_m": 0.2, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
        ],
        acceleration=[
            {"time_sec": 0.0, "acceleration_mps2": 0.0},
            {"time_sec": 1.0, "acceleration_mps2": -0.2},
            {"time_sec": 2.0, "acceleration_mps2": -0.4},
        ],
    )

    assert parsed.available
    middle = parsed.vehicle_timeseries[1]
    assert middle["grade_source"] == "odometry"
    assert math.isclose(float(middle["grade_percent"]), 10.0, rel_tol=1e-6)


def test_build_analysis_from_series_skips_grade_when_odometry_is_too_slow() -> None:
    parsed = build_analysis_from_series(
        odometry=[
            {"time_sec": 0.0, "x_m": 0.0, "y_m": 0.0, "z_m": 0.0, "speed_mps": 0.0, "yaw_rate_rps": 0.0},
            {"time_sec": 1.0, "x_m": 0.0001, "y_m": 0.0, "z_m": 0.1, "speed_mps": 0.0, "yaw_rate_rps": 0.0},
            {"time_sec": 2.0, "x_m": 0.0002, "y_m": 0.0, "z_m": 0.2, "speed_mps": 0.0, "yaw_rate_rps": 0.0},
        ],
    )

    assert parsed.available
    assert all(row["grade_percent"] is None for row in parsed.vehicle_timeseries)


def test_build_analysis_from_series_computes_grade_from_trajectory_z() -> None:
    parsed = build_analysis_from_series(
        odometry=[
            {"time_sec": 0.0, "x_m": 0.0, "y_m": 0.0, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
            {"time_sec": 1.0, "x_m": 1.0, "y_m": 0.0, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
            {"time_sec": 2.0, "x_m": 2.0, "y_m": 0.0, "speed_mps": 2.0, "yaw_rate_rps": 0.0},
        ],
        trajectory_points=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.2), (2.0, 0.0, 0.4)],
        trajectory_source="test_trajectory",
    )

    assert parsed.available
    assert parsed.vehicle_timeseries[1]["grade_source"] == "trajectory"
    assert math.isclose(float(parsed.vehicle_timeseries[1]["grade_percent"]), 20.0, rel_tol=1e-6)
    assert math.isclose(float(parsed.trajectory_reference[1]["grade_percent"]), 20.0, rel_tol=1e-6)


def test_apply_corner_id_rotation_starts_numbering_from_later_corner() -> None:
    segments = [
        {"corner_id": "corner_01", "start_idx": 0, "end_idx": 1},
        {"corner_id": "corner_02", "start_idx": 2, "end_idx": 3},
        {"corner_id": "corner_03", "start_idx": 4, "end_idx": 5},
    ]

    rotated = _apply_corner_id_rotation(segments, 1)

    assert [segment["corner_id"] for segment in rotated] == ["corner_03", "corner_01", "corner_02"]
