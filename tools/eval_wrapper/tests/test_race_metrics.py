from __future__ import annotations

import csv
from pathlib import Path

from evalwrap.metrics.race_metrics import DomainMetrics, DomainResult, write_processed_outputs


def test_write_processed_outputs_writes_grade_profile_and_motion_log(tmp_path: Path) -> None:
    domain = DomainResult(
        domain_id="d1",
        metrics=DomainMetrics(finish=True),
        lap_times=[],
        events=[],
        log_excerpts=[],
        vehicle_timeseries=[
            {
                "time_sec": 0.0,
                "x_m": 9.0,
                "y_m": 19.0,
                "z_m": 0.4,
                "distance_m": 0.0,
                "track_s_m": 0.0,
                "speed_mps": 0.0,
                "acceleration_mps2": 0.0,
            },
            {
                "time_sec": 1.0,
                "x_m": 10.0,
                "y_m": 20.0,
                "z_m": 0.5,
                "distance_m": 2.0,
                "track_s_m": 2.2,
                "grade_percent": 4.5,
                "grade_rad": 0.0449,
                "grade_source": "odometry",
                "speed_mps": 6.0,
                "acceleration_mps2": -0.3,
            }
        ],
        control_timeseries=[
            {
                "time_sec": 1.0,
                "target_speed_mps": 7.0,
                "accel_mps2": -0.2,
                "steer_rad": 0.1,
                "throttle": None,
                "brake": None,
            }
        ],
        delay_debug_timeseries=[],
        speed_profile_debug_timeseries=[],
        overtake_debug_timeseries=[],
        section_summary=[],
        awsim_section_summary=[],
        corner_summary=[],
        trajectory_reference=[],
    )

    write_processed_outputs("run", [domain], tmp_path)

    with (tmp_path / "grade_profile.csv").open("r", encoding="utf-8", newline="") as handle:
        grade_rows = list(csv.DictReader(handle))
    assert len(grade_rows) == 1
    assert grade_rows[0]["grade_percent"] == "4.5"
    assert grade_rows[0]["grade_source"] == "odometry"
    assert grade_rows[0]["command_accel_mps2"] == "-0.2"

    with (tmp_path / "motion_log.csv").open("r", encoding="utf-8", newline="") as handle:
        motion_rows = list(csv.DictReader(handle))
    assert motion_rows[0]["grade_percent"] == ""
    assert motion_rows[1]["grade_percent"] == "4.5"
    assert motion_rows[1]["grade_source"] == "odometry"
