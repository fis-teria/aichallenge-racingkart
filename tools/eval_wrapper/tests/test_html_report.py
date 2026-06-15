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
                "run_id,domain_id,corner_id,pass,entry_time_sec,exit_time_sec,duration_sec",
                "run,d1,corner_01,1,4.0,5.25,1.25",
                "run,d1,corner_02,1,8.0,9.5,1.5",
                "run,d1,corner_01,2,68.0,69.125,1.125",
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

    report = generate_run_report(
        run_dir,
        {"run_id": "run", "status": "success"},
        {"domains": {"d1": {"finish": True}}, "log_excerpts": {}},
    )

    html = report.read_text(encoding="utf-8")
    assert "Corner Splits" in html
    assert "corner_01" in html
    assert "corner_02" in html
    assert "0:05.250" in html
    assert "+1.250s" in html
    assert "1:09.125" in html
    assert "Corner Map" in html
    assert "data-corner-map='corner_01'" in html
    assert "corner-map-highlight" in html
