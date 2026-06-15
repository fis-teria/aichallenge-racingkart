from __future__ import annotations

import sqlite3
from pathlib import Path
from typing import Any


SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
  run_id TEXT PRIMARY KEY,
  label TEXT NOT NULL,
  created_at TEXT NOT NULL,
  status TEXT NOT NULL,
  git_commit TEXT,
  git_branch TEXT,
  diff_hash TEXT,
  submit_sha256 TEXT,
  eval_mode TEXT,
  report_path TEXT,
  judgement TEXT
);

CREATE TABLE IF NOT EXISTS domain_metrics (
  run_id TEXT NOT NULL,
  domain_id TEXT NOT NULL,
  finish INTEGER,
  total_time_sec REAL,
  lap_count INTEGER,
  best_lap_sec REAL,
  avg_lap_sec REAL,
  penalty_count INTEGER,
  collision_count INTEGER,
  stuck_count INTEGER,
  low_speed_time_sec REAL,
  max_speed_mps REAL,
  avg_speed_mps REAL,
  max_abs_steer_rad REAL,
  steer_oscillation_score REAL,
  max_accel_mps2 REAL,
  max_decel_mps2 REAL,
  avg_path_error_m REAL,
  max_path_error_m REAL,
  judgement TEXT,
  PRIMARY KEY (run_id, domain_id)
);

CREATE TABLE IF NOT EXISTS events (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  run_id TEXT NOT NULL,
  domain_id TEXT NOT NULL,
  time_sec REAL,
  lap INTEGER,
  section INTEGER,
  event_type TEXT NOT NULL,
  severity TEXT,
  description TEXT
);
"""


def init_db(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(path) as conn:
        conn.executescript(SCHEMA)
        _ensure_domain_metric_columns(conn)


def save_run(db_path: Path, manifest: dict[str, Any], metrics: dict[str, Any], report_path: Path) -> None:
    init_db(db_path)
    domains = metrics.get("domains", {})
    judgement = _aggregate_judgement(domains)
    with sqlite3.connect(db_path) as conn:
        conn.execute(
            """
            INSERT OR REPLACE INTO runs
            (run_id, label, created_at, status, git_commit, git_branch, diff_hash, submit_sha256, eval_mode, report_path, judgement)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                manifest["run_id"],
                manifest["label"],
                manifest["created_at"],
                manifest["status"],
                manifest.get("repo", {}).get("commit"),
                manifest.get("repo", {}).get("branch"),
                manifest.get("repo", {}).get("diff_hash"),
                manifest.get("submission", {}).get("tar_sha256"),
                manifest.get("eval", {}).get("mode"),
                str(report_path),
                judgement,
            ),
        )
        for domain_id, domain in domains.items():
            conn.execute(
                """
                INSERT OR REPLACE INTO domain_metrics
                (run_id, domain_id, finish, total_time_sec, lap_count, best_lap_sec, avg_lap_sec,
                 penalty_count, collision_count, stuck_count, low_speed_time_sec, max_speed_mps, avg_speed_mps,
                 max_abs_steer_rad, steer_oscillation_score, max_accel_mps2, max_decel_mps2,
                 avg_path_error_m, max_path_error_m, judgement)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    manifest["run_id"],
                    domain_id,
                    _bool_to_int(domain.get("finish")),
                    domain.get("total_time_sec"),
                    domain.get("lap_count"),
                    domain.get("best_lap_sec"),
                    domain.get("avg_lap_sec"),
                    domain.get("penalty_count"),
                    domain.get("collision_count"),
                    domain.get("stuck_count"),
                    domain.get("low_speed_time_sec"),
                    domain.get("max_speed_mps"),
                    domain.get("avg_speed_mps"),
                    domain.get("max_abs_steer_rad"),
                    domain.get("steer_oscillation_score"),
                    domain.get("max_accel_mps2"),
                    domain.get("max_decel_mps2"),
                    domain.get("avg_path_error_m"),
                    domain.get("max_path_error_m"),
                    domain.get("judgement"),
                ),
            )
        conn.execute("DELETE FROM events WHERE run_id = ?", (manifest["run_id"],))
        for domain_id, events in metrics.get("events", {}).items():
            for event in events:
                conn.execute(
                    """
                    INSERT INTO events
                    (run_id, domain_id, time_sec, lap, section, event_type, severity, description)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        manifest["run_id"],
                        domain_id,
                        event.get("time_sec"),
                        event.get("lap"),
                        event.get("section"),
                        event.get("event_type"),
                        event.get("severity"),
                        event.get("description"),
                    ),
                )


def list_runs(db_path: Path) -> list[sqlite3.Row]:
    init_db(db_path)
    with sqlite3.connect(db_path) as conn:
        conn.row_factory = sqlite3.Row
        return list(conn.execute("SELECT * FROM runs ORDER BY created_at DESC"))


def leaderboard(db_path: Path, metric: str) -> list[sqlite3.Row]:
    allowed = {
        "total_time_sec",
        "best_lap_sec",
        "avg_lap_sec",
        "penalty_count",
        "collision_count",
        "stuck_count",
        "low_speed_time_sec",
        "max_speed_mps",
        "avg_speed_mps",
        "max_abs_steer_rad",
        "steer_oscillation_score",
        "max_accel_mps2",
        "max_decel_mps2",
        "avg_path_error_m",
        "max_path_error_m",
    }
    if metric not in allowed:
        raise ValueError(f"unsupported metric: {metric}")
    init_db(db_path)
    order = "DESC" if metric in {"max_speed_mps", "avg_speed_mps", "max_accel_mps2"} else "ASC"
    with sqlite3.connect(db_path) as conn:
        conn.row_factory = sqlite3.Row
        return list(
            conn.execute(
                f"""
                SELECT r.run_id, r.label, d.domain_id, d.{metric} AS metric_value, d.judgement, r.report_path
                FROM domain_metrics d
                JOIN runs r ON r.run_id = d.run_id
                WHERE d.{metric} IS NOT NULL
                ORDER BY d.{metric} {order}
                """
            )
        )


def _bool_to_int(value: Any) -> int | None:
    if value is None:
        return None
    return 1 if bool(value) else 0


def _ensure_domain_metric_columns(conn: sqlite3.Connection) -> None:
    existing = {row[1] for row in conn.execute("PRAGMA table_info(domain_metrics)")}
    desired = {
        "low_speed_time_sec": "REAL",
        "max_accel_mps2": "REAL",
        "max_decel_mps2": "REAL",
        "avg_path_error_m": "REAL",
        "max_path_error_m": "REAL",
    }
    for name, column_type in desired.items():
        if name not in existing:
            conn.execute(f"ALTER TABLE domain_metrics ADD COLUMN {name} {column_type}")


def _aggregate_judgement(domains: dict[str, Any]) -> str:
    judgements = [domain.get("judgement") for domain in domains.values()]
    for preferred in ("candidate", "stable_candidate", "attack_candidate", "needs_more_eval", "reject"):
        if preferred in judgements:
            return preferred
    return "needs_more_eval"
