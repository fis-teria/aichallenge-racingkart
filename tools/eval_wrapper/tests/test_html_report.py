from __future__ import annotations

from pathlib import Path

from evalwrap.reports.html_report import generate_run_report


def test_generate_run_report_renders_corner_split_table(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    processed_dir = run_dir / "processed"
    processed_dir.mkdir(parents=True)
    (processed_dir / "corner_summary.csv").write_text(
        "\n".join(
            [
                "run_id,domain_id,corner_id,pass,entry_time_sec,exit_time_sec,duration_sec,min_speed_mps,avg_path_error_m,max_path_error_m,event_count",
                "run,d1,corner_01,1,4.0,5.25,1.25,5.2,0.14,0.22,1",
                "run,d1,corner_02,1,8.0,9.5,1.5,4.8,0.18,0.31,0",
                "run,d1,corner_01,2,68.0,69.125,1.125,5.4,0.12,0.20,0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (processed_dir / "trajectory_reference.csv").write_text(
        "\n".join(
            [
                "run_id,domain_id,point_index,x_m,y_m,track_s_m,trajectory_curvature_1pm,corner_id,trajectory_source",
                "run,d1,0,0.0,0.0,0.0,0.0,,mpc_csv:ref.csv",
                "run,d1,1,1.0,0.0,1.0,0.2,corner_01,mpc_csv:ref.csv",
                "run,d1,2,1.0,1.0,2.0,0.2,corner_01,mpc_csv:ref.csv",
                "run,d1,3,2.0,1.0,3.0,0.2,corner_02,mpc_csv:ref.csv",
                "run,d1,4,2.0,2.0,4.0,0.2,corner_02,mpc_csv:ref.csv",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (processed_dir / "vehicle_timeseries.csv").write_text(
        "\n".join(
            [
                "run_id,domain_id,time_sec,x_m,y_m,distance_m,section,corner_id,track_s_m,trajectory_curvature_1pm,speed_mps,acceleration_mps2,steering_rad,yaw_rate_rps,path_error_m",
                "run,d1,1.0,0.0,0.0,0.0,1,corner_01,0.0,0.0,7.0,0.1,0.0,0.0,0.1",
                "run,d1,2.0,1.0,0.0,1.0,1,corner_01,1.0,0.1,7.5,-0.2,0.1,0.0,0.2",
                "run,d1,3.0,2.0,0.0,2.0,1,corner_02,2.0,0.1,6.5,-0.8,0.2,0.0,0.3",
                "run,d1,4.0,3.0,0.0,3.0,1,corner_02,3.0,0.0,5.2,-0.3,0.1,0.0,0.1",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (processed_dir / "control_timeseries.csv").write_text(
        "\n".join(
            [
                "run_id,domain_id,time_sec,target_speed_mps,accel_mps2,steer_rad,throttle,brake",
                "run,d1,1.0,8.0,1.0,0.0,,",
                "run,d1,2.0,8.0,0.0,0.1,,",
                "run,d1,3.0,5.0,0.0,0.2,,",
                "run,d1,4.0,5.0,0.0,0.1,,",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (processed_dir / "speed_profile_debug.csv").write_text(
        "\n".join(
            [
                "run_id,domain_id,time_sec,wp_id,source,target_speed_mps,curvature_speed_mps,section_cap_mps,global_cap_mps,actual_speed_mps,command_speed_mps,mpc_status,mpc_solve_time_ms,mpc_infeasible_count",
                "run,d1,1.0,10,global,8.0,9.0,,8.0,7.0,1.0,solved,4.0,0",
                "run,d1,2.0,20,curvature,8.0,8.0,,9.0,7.5,0.0,solved,4.5,0",
                "run,d1,3.0,30,section,5.0,8.0,5.0,9.0,6.5,0.0,infeasible,96.0,1",
                "run,d1,4.0,40,section,5.0,8.0,5.0,9.0,5.2,0.0,solved,5.0,1",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (processed_dir / "events.csv").write_text(
        "\n".join(
            [
                "run_id,domain_id,time_sec,lap,section,event_type,severity,description",
                "run,d1,3.0,1,1,path_error,warning,path error exceeded threshold",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    (processed_dir / "grade_profile.csv").write_text(
        "\n".join(
            [
                "run_id,domain_id,time_sec,distance_m,track_s_m,x_m,y_m,z_m,trajectory_z_m,grade_percent,grade_rad,grade_source,speed_mps,target_speed_mps,acceleration_mps2,command_accel_mps2,command_steer_rad",
                "run,d1,1.0,0.0,0.0,0.0,0.0,0.0,,2.0,0.02,odometry,7.0,8.0,0.1,1.0,0.0",
                "run,d1,2.0,1.0,1.0,1.0,0.0,0.1,,4.0,0.04,odometry,7.5,8.0,-0.2,0.0,0.1",
                "run,d1,3.0,2.0,2.0,2.0,0.0,0.0,,-3.0,-0.03,odometry,6.5,5.0,-0.8,0.0,0.2",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    report = generate_run_report(
        run_dir,
        {"run_id": "run", "status": "success"},
        {"domains": {"d1": {"finish": True, "max_command_decel_mps2": -0.125}}, "log_excerpts": {}},
    )

    html = report.read_text(encoding="utf-8")
    assert "Section Splits" in html
    assert "AWSIM section data was not recorded for this run" in html
    assert "corner_01" in html
    assert "corner_02" in html
    assert "0:05.250" in html
    assert "+1.250s" in html
    assert "1:09.125" in html
    assert "Corner Map" in html
    assert "data-corner-map='corner_01'" in html
    assert "corner-map-highlight" in html
    assert "Speed Profile" in html
    assert "speed-map-segment" in html
    assert "linearGradient" in html
    assert "actual speed map" in html
    assert "actual min 18.7 km/h" in html
    assert "28.8 km/h -&gt; 18.0 km/h" in html
    assert "Track Diagnostics" in html
    assert "Speed Error Map" in html
    assert "speed error map" in html
    assert "Path Error Map" in html
    assert "path error map" in html
    assert "Speed Limit Source Map" in html
    assert "speed limit source map" in html
    assert "Event Marker Map" in html
    assert "event marker map" in html
    assert "path_error warning" in html
    assert "MPC Health Map" in html
    assert "mpc health map" in html
    assert "Worst MPC Health Samples" in html
    assert "infeasible" in html
    assert "thresholds" in html
    assert "Control Response" in html
    assert "Acceleration Response" in html
    assert "acceleration response chart" in html
    assert "Steering Response" in html
    assert "steering response chart" in html
    assert "Corner Performance" in html
    assert "Corner Duration" in html
    assert "Corner Minimum Speed" in html
    assert "Corner Path Error" in html
    assert "Grade & Acceleration Profile" in html
    assert "grade map" in html
    assert "grade-chart-grade" in html
    assert "4.00%" in html
    assert "grade_profile.csv" in html
    assert "max_command_decel_mps2" in html
    assert "-0.125" in html


def test_generate_run_report_prefers_awsim_section_split_table(tmp_path: Path) -> None:
    run_dir = tmp_path / "run"
    processed_dir = run_dir / "processed"
    processed_dir.mkdir(parents=True)
    (processed_dir / "awsim_section_summary.csv").write_text(
        "\n".join(
            [
                "run_id,domain_id,lap,section,entry_time_sec,exit_time_sec,entry_lap_time_sec,exit_lap_time_sec,duration_sec,avg_speed_mps,max_speed_mps,min_speed_mps,avg_path_error_m,max_path_error_m,sample_count",
                "run,d1,1,1,0.0,4.0,0.0,4.0,4.0,6.0,7.0,5.0,0.2,0.4,20",
                "run,d1,1,2,4.0,9.5,4.0,9.5,5.5,7.0,8.0,6.0,0.3,0.5,25",
                "run,d1,2,1,0.0,3.5,0.0,3.5,3.5,6.5,7.5,5.5,0.2,0.4,18",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    report = generate_run_report(
        run_dir,
        {"run_id": "run", "status": "success"},
        {"domains": {"d1": {"finish": True}}, "log_excerpts": {}},
    )

    html = report.read_text(encoding="utf-8")
    assert "Section Splits" in html
    assert "section_1" in html
    assert "section_2" in html
    assert "4.000s" in html
    assert "5.500s" in html
    assert "lap 0:09.500" in html
    assert "18.0 km/h..25.2 km/h" in html
    assert "AWSIM section data was not recorded" not in html
