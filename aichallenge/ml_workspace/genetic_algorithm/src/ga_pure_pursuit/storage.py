from __future__ import annotations

import json
import sqlite3
import threading
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
"""


class Storage:
    def __init__(self, path: Path):
        self.path = path
        self._lock = threading.RLock()
        self.connection = sqlite3.connect(path, check_same_thread=False)
        self.connection.executescript(SCHEMA)

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

    def begin_candidate(
        self, candidate_id: str, generation: int, genome: dict[str, float], parameter_hash: str
    ) -> None:
        with self._lock:
            with self.connection:
                self.connection.execute(
                    "INSERT OR REPLACE INTO candidates VALUES (?, ?, ?, ?, NULL, 'running')",
                    (candidate_id, generation, json.dumps(genome, sort_keys=True), parameter_hash),
                )

    def complete_candidate(
        self,
        candidate_id: str,
        episodes: list[tuple[int, dict[str, Any], float]],
        fitness: float,
    ) -> None:
        with self._lock:
            with self.connection:
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
