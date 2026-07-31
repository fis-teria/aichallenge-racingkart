from __future__ import annotations

import json
import math
import random
import sqlite3
from pathlib import Path
from typing import Any

from .fitness import robust_score, score
from .genome import Bounds, parameter_hash


class KnnSurrogate:
    """Small-data CPU surrogate using normalized, distance-weighted neighbors."""

    def __init__(
        self,
        bounds: dict[str, Bounds],
        baseline: dict[str, float],
        neighbors: int = 12,
    ):
        self.names = sorted(bounds)
        self.bounds = bounds
        self.baseline = baseline
        self.neighbors = max(2, int(neighbors))
        self.samples: list[tuple[list[float], float]] = []

    def vector(self, genome: dict[str, float]) -> list[float]:
        values = []
        for name in self.names:
            bound = self.bounds[name]
            value = float(genome.get(name, self.baseline[name]))
            values.append((value - bound.low) / (bound.high - bound.low))
        return values

    def fit(self, samples: list[tuple[dict[str, float], float]]) -> None:
        deduplicated: dict[str, tuple[dict[str, float], float]] = {}
        for genome, target in samples:
            if math.isfinite(float(target)):
                deduplicated[parameter_hash(genome)] = (genome, float(target))
        self.samples = [
            (self.vector(genome), target)
            for genome, target in deduplicated.values()
        ]

    def predict(self, genome: dict[str, float]) -> tuple[float, float]:
        if not self.samples:
            return float("inf"), float("inf")
        vector = self.vector(genome)
        distances = sorted(
            (
                math.sqrt(
                    sum((left - right) ** 2 for left, right in zip(vector, sample))
                    / max(1, len(vector))
                ),
                target,
            )
            for sample, target in self.samples
        )[: min(self.neighbors, len(self.samples))]
        weights = [1.0 / (distance + 1.0e-3) for distance, _ in distances]
        weight_sum = sum(weights)
        prediction = sum(
            weight * target for weight, (_, target) in zip(weights, distances)
        ) / weight_sum
        variance = sum(
            weight * (target - prediction) ** 2
            for weight, (_, target) in zip(weights, distances)
        ) / weight_sum
        uncertainty = math.sqrt(max(0.0, variance)) + distances[0][0]
        return float(prediction), float(uncertainty)


def select_candidates(
    candidates: list[dict[str, float]],
    model: KnnSurrogate,
    count: int,
    rng: random.Random,
    exploit_fraction: float = 0.6,
    uncertainty_fraction: float = 0.2,
) -> list[dict[str, float]]:
    if len(candidates) <= count:
        return list(candidates)
    predicted = [
        (candidate, *model.predict(candidate))
        for candidate in candidates
    ]
    exploit_count = min(count, round(count * exploit_fraction))
    uncertainty_count = min(
        count - exploit_count, round(count * uncertainty_fraction)
    )
    selected = sorted(predicted, key=lambda item: item[1])[:exploit_count]
    selected_ids = {id(item[0]) for item in selected}
    remaining = [item for item in predicted if id(item[0]) not in selected_ids]
    uncertain = sorted(remaining, key=lambda item: item[2], reverse=True)[
        :uncertainty_count
    ]
    selected.extend(uncertain)
    selected_ids.update(id(item[0]) for item in uncertain)
    remaining = [item for item in predicted if id(item[0]) not in selected_ids]
    rng.shuffle(remaining)
    selected.extend(remaining[: count - len(selected)])
    rng.shuffle(selected)
    return [item[0] for item in selected]


def load_training_samples(
    run_dirs: list[str | Path],
    fitness_weights: dict[str, float],
    required_gene_names: set[str],
) -> list[tuple[dict[str, float], float]]:
    samples: list[tuple[dict[str, float], float]] = []
    for value in run_dirs:
        database = Path(value) / "run.sqlite3"
        if not database.exists():
            continue
        connection = sqlite3.connect(database)
        try:
            rows = connection.execute(
                "SELECT c.candidate_id, c.genome_json, e.metrics_json "
                "FROM candidates c JOIN episodes e ON e.candidate_id=c.candidate_id "
                "WHERE c.status='complete' ORDER BY c.candidate_id, e.repeat_index"
            ).fetchall()
        finally:
            connection.close()
        grouped: dict[str, tuple[dict[str, float], list[float]]] = {}
        for candidate_id, genome_json, metrics_json in rows:
            genome = json.loads(genome_json)
            if not required_gene_names.issubset(genome):
                continue
            metrics = json.loads(metrics_json)
            item = grouped.setdefault(candidate_id, (genome, []))
            item[1].append(score(metrics, fitness_weights))
        samples.extend(
            (genome, robust_score(scores))
            for genome, scores in grouped.values()
            if scores
        )
    return samples
