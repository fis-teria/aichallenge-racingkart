from __future__ import annotations

from pathlib import Path

from evalwrap.store import leaderboard, save_run


def test_store_persists_command_metrics(tmp_path: Path) -> None:
    db_path = tmp_path / "analysis.sqlite"
    manifest = {
        "run_id": "run",
        "label": "delay aware",
        "created_at": "2026-06-20T00:00:00Z",
        "status": "success",
    }
    metrics = {
        "domains": {
            "d1": {
                "finish": True,
                "max_command_accel_mps2": 3.0,
                "max_command_decel_mps2": -0.125,
                "max_command_abs_steer_rad": 0.5,
                "judgement": "candidate",
            }
        },
        "events": {},
    }

    save_run(db_path, manifest, metrics, tmp_path / "report" / "index.html")

    rows = leaderboard(db_path, "max_command_decel_mps2")
    assert len(rows) == 1
    assert rows[0]["run_id"] == "run"
    assert rows[0]["metric_value"] == -0.125
