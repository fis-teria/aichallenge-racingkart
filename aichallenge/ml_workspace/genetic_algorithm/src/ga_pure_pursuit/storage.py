from __future__ import annotations

import json
import sqlite3
import threading
import time
from pathlib import Path
from typing import Any


SCHEMA = """
CREATE TABLE IF NOT EXISTS candidates (
  candidate_id TEXT PRIMARY KEY,
  generation INTEGER NOT NULL,
  genome_json TEXT NOT NULL,
  parameter_hash TEXT NOT NULL,
  fitness REAL,
  status TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS episodes (
  episode_id TEXT PRIMARY KEY,
  candidate_id TEXT NOT NULL,
  repeat_index INTEGER NOT NULL,
  metrics_json TEXT NOT NULL,
  score REAL NOT NULL,
  exit_reason TEXT NOT NULL,
  FOREIGN KEY(candidate_id) REFERENCES candidates(candidate_id)
);
CREATE INDEX IF NOT EXISTS candidates_fitness ON candidates(fitness);
CREATE TABLE IF NOT EXISTS candidate_metadata (
  candidate_id TEXT PRIMARY KEY,
  optimizer_type TEXT NOT NULL,
  phase TEXT NOT NULL,
  feasible INTEGER,
  violation REAL,
  robust_lap REAL,
  objective_value REAL,
  details_json TEXT NOT NULL,
  FOREIGN KEY(candidate_id) REFERENCES candidates(candidate_id)
);
CREATE INDEX IF NOT EXISTS candidate_metadata_objective
  ON candidate_metadata(objective_value);
"""


class Storage:
    def __init__(self, path: Path):
        self.path = path
        self._lock = threading.RLock()
        self.connection = sqlite3.connect(path, timeout=30.0, check_same_thread=False)
        self.connection.execute("PRAGMA busy_timeout=30000")
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute("PRAGMA synchronous=NORMAL")
        self.connection.executescript(SCHEMA)

    def _write_transaction(self, operation, attempts: int = 5) -> None:
        """Commit a write while tolerating short-lived dashboard/worker locks."""
        for attempt in range(attempts):
            try:
                with self._lock:
                    with self.connection:
                        operation()
                return
            except sqlite3.OperationalError as error:
                if "locked" not in str(error).lower() or attempt + 1 >= attempts:
                    raise
                with self._lock:
                    self.connection.rollback()
                time.sleep(0.05 * (2 ** attempt))

    def close(self) -> None:
        with self._lock:
            self.connection.close()

    def get_fitness(self, parameter_hash: str) -> float | None:
        with self._lock:
            row = self.connection.execute(
                "SELECT fitness FROM candidates WHERE parameter_hash=? AND status='complete' "
                "ORDER BY fitness LIMIT 1",
                (parameter_hash,),
            ).fetchone()
        return None if row is None else float(row[0])

    def get_objective(self, parameter_hash: str) -> float | None:
        with self._lock:
            row = self.connection.execute(
                "SELECT m.objective_value FROM candidate_metadata m "
                "JOIN candidates c ON c.candidate_id=m.candidate_id "
                "WHERE c.parameter_hash=? AND c.status='complete' "
                "AND m.objective_value IS NOT NULL ORDER BY m.objective_value LIMIT 1",
                (parameter_hash,),
            ).fetchone()
        return None if row is None else float(row[0])

    def begin_candidate(
        self, candidate_id: str, generation: int, genome: dict[str, float], parameter_hash: str
    ) -> None:
        def write() -> None:
            self.connection.execute(
                "INSERT OR REPLACE INTO candidates VALUES (?, ?, ?, ?, NULL, 'running')",
                (candidate_id, generation, json.dumps(genome, sort_keys=True), parameter_hash),
            )
        self._write_transaction(write)

    def complete_candidate(
        self,
        candidate_id: str,
        episodes: list[tuple[int, dict[str, Any], float]],
        fitness: float,
    ) -> None:
        def write() -> None:
            for repeat, metrics, episode_score in episodes:
                self.connection.execute(
                    "INSERT OR REPLACE INTO episodes VALUES (?, ?, ?, ?, ?, ?)",
                    (
                        f"{candidate_id}-r{repeat:02d}",
                        candidate_id,
                        repeat,
                        json.dumps(metrics, sort_keys=True),
                        episode_score,
                        str(metrics.get("exit_reason", "unknown")),
                    ),
                )
            self.connection.execute(
                "UPDATE candidates SET fitness=?, status='complete' WHERE candidate_id=?",
                (fitness, candidate_id),
            )
        self._write_transaction(write)

    def save_candidate_metadata(self, candidate_id: str, optimizer_type: str,
                                phase: str, result: dict[str, Any]) -> None:
        def write() -> None:
            self.connection.execute(
                "INSERT OR REPLACE INTO candidate_metadata VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                (candidate_id, optimizer_type, phase, int(bool(result["feasible"])),
                 float(result["violation"]), result.get("robust_lap"),
                 float(result["objective_value"]), json.dumps(result, sort_keys=True)),
            )
        self._write_transaction(write)

    def best(self, limit: int = 1) -> list[dict[str, Any]]:
        with self._lock:
            rows = self.connection.execute(
                "SELECT candidate_id, generation, genome_json, parameter_hash, fitness "
                "FROM candidates WHERE status='complete' ORDER BY fitness LIMIT ?",
                (limit,),
            ).fetchall()
        return [
            {
                "candidate_id": row[0],
                "generation": row[1],
                "parameters": json.loads(row[2]),
                "parameter_hash": row[3],
                "fitness": row[4],
            }
            for row in rows
        ]

    def best_objective(self, limit: int = 1) -> list[dict[str, Any]]:
        with self._lock:
            rows = self.connection.execute(
                "SELECT c.candidate_id, c.generation, c.genome_json, c.parameter_hash, "
                "c.fitness, m.phase, m.details_json FROM candidates c "
                "JOIN candidate_metadata m ON m.candidate_id=c.candidate_id "
                "WHERE c.status='complete' AND m.objective_value IS NOT NULL "
                "ORDER BY m.objective_value, c.candidate_id LIMIT ?", (limit,)
            ).fetchall()
        result = []
        for row in rows:
            objective = json.loads(row[6])
            result.append({
                "candidate_id": row[0], "generation": row[1],
                "parameters": json.loads(row[2]), "parameter_hash": row[3],
                "fitness": row[4], "phase": row[5], **objective,
            })
        return result

    def generation_best_fitness(self) -> list[tuple[int, float]]:
        with self._lock:
            rows = self.connection.execute(
                "SELECT generation, MIN(fitness) FROM candidates "
                "WHERE status='complete' GROUP BY generation ORDER BY generation"
            ).fetchall()
        return [(int(row[0]), float(row[1])) for row in rows]

    def training_samples(self) -> list[tuple[dict[str, float], float]]:
        with self._lock:
            rows = self.connection.execute(
                "SELECT genome_json, fitness FROM candidates "
                "WHERE status='complete' AND fitness IS NOT NULL"
            ).fetchall()
        return [
            (json.loads(row[0]), float(row[1]))
            for row in rows
        ]

    def observations(self):
        from .phase2 import Observation
        with self._lock:
            rows = self.connection.execute(
                "SELECT c.candidate_id, c.genome_json, c.fitness, e.metrics_json "
                "FROM candidates c JOIN episodes e ON e.candidate_id=c.candidate_id "
                "WHERE c.status='complete' AND c.fitness IS NOT NULL "
                "ORDER BY c.candidate_id, e.repeat_index"
            ).fetchall()
        grouped: dict[str, tuple[dict[str, float], float, list[dict[str, Any]]]] = {}
        for candidate_id, genome_json, fitness, metrics_json in rows:
            item = grouped.setdefault(
                candidate_id, (json.loads(genome_json), float(fitness), [])
            )
            item[2].append(json.loads(metrics_json))
        observations = []
        for genome, fitness, episodes in grouped.values():
            numeric: dict[str, float] = {}
            keys = {key for episode in episodes for key, value in episode.items()
                    if isinstance(value, (int, float)) and not isinstance(value, bool)}
            for key in keys:
                values = [float(episode[key]) for episode in episodes
                          if isinstance(episode.get(key), (int, float))
                          and not isinstance(episode.get(key), bool)]
                if values:
                    numeric[key] = sum(values) / len(values)
            completion = sum(bool(episode.get("completed", False)) for episode in episodes) / len(episodes)
            observations.append(Observation(genome, fitness, completion, numeric))
        return observations
