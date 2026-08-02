from __future__ import annotations

import math
import random
import json
import sqlite3
from pathlib import Path
from dataclasses import dataclass
from typing import Any

from .genome import Bounds, parameter_hash, repair


@dataclass(frozen=True)
class Observation:
    genome: dict[str, float]
    fitness: float
    completion_probability: float
    metrics: dict[str, float]


class KnnFeasibilityModel:
    """Distance-weighted completion probability model for small GA datasets."""

    def __init__(self, bounds: dict[str, Bounds], baseline: dict[str, float], neighbors: int = 16):
        self.names = sorted(bounds)
        self.bounds = bounds
        self.baseline = baseline
        self.neighbors = max(2, int(neighbors))
        self.samples: list[tuple[list[float], float]] = []

    def vector(self, genome: dict[str, float]) -> list[float]:
        return [
            (float(genome.get(name, self.baseline[name])) - self.bounds[name].low)
            / (self.bounds[name].high - self.bounds[name].low)
            for name in self.names
        ]

    def fit(self, observations: list[Observation]) -> None:
        unique: dict[str, Observation] = {}
        for item in observations:
            unique[parameter_hash(item.genome)] = item
        self.samples = [
            (self.vector(item.genome), min(1.0, max(0.0, item.completion_probability)))
            for item in unique.values()
        ]

    def predict(self, genome: dict[str, float]) -> tuple[float, float]:
        if not self.samples:
            return 0.5, 1.0
        vector = self.vector(genome)
        nearest = sorted(
            (
                math.sqrt(sum((a - b) ** 2 for a, b in zip(vector, sample)) / len(vector)),
                target,
            )
            for sample, target in self.samples
        )[: min(self.neighbors, len(self.samples))]
        weights = [1.0 / (distance + 1.0e-3) for distance, _ in nearest]
        probability = sum(w * target for w, (_, target) in zip(weights, nearest)) / sum(weights)
        uncertainty = math.sqrt(max(0.0, probability * (1.0 - probability))) + nearest[0][0]
        return probability, uncertainty


@dataclass
class ArchiveEntry:
    genome: dict[str, float]
    fitness: float
    descriptors: tuple[float, float]


class MapElitesArchive:
    """Two-dimensional archive: path aggressiveness x steering demand."""

    def __init__(self, bins: tuple[int, int] = (8, 8)):
        self.bins = bins
        self.cells: dict[tuple[int, int], ArchiveEntry] = {}

    @staticmethod
    def descriptors(observation: Observation) -> tuple[float, float]:
        offsets = [abs(value) for name, value in observation.genome.items() if name.startswith("path_offset_")]
        path_aggressiveness = sum(offsets) / max(1, len(offsets))
        steering = float(observation.metrics.get(
            "steering_angle_rms", observation.metrics.get("steering_angle_rms_rad", 0.0)
        ))
        return path_aggressiveness, steering

    def _cell(self, descriptors: tuple[float, float]) -> tuple[int, int]:
        # Offsets are searched up to about 2 m; steering RMS above 0.4 rad is saturated.
        return (
            min(self.bins[0] - 1, max(0, int(descriptors[0] / 2.0 * self.bins[0]))),
            min(self.bins[1] - 1, max(0, int(descriptors[1] / 0.4 * self.bins[1]))),
        )

    def add(self, observation: Observation) -> bool:
        if observation.completion_probability < 0.999:
            return False
        descriptors = self.descriptors(observation)
        cell = self._cell(descriptors)
        current = self.cells.get(cell)
        if current is not None and current.fitness <= observation.fitness:
            return False
        self.cells[cell] = ArchiveEntry(observation.genome.copy(), observation.fitness, descriptors)
        return True

    def build(self, observations: list[Observation]) -> None:
        for item in observations:
            self.add(item)

    def genomes(self) -> list[dict[str, float]]:
        return [entry.genome for entry in self.cells.values()]


class EmitterBandit:
    """UCB scheduler; successful archive/fitness improvements earn more offspring."""

    def __init__(self, names: list[str]):
        self.names = names
        self.pulls = {name: 0 for name in names}
        self.rewards = {name: 0.0 for name in names}

    def choose(self) -> str:
        for name in self.names:
            if self.pulls[name] == 0:
                self.pulls[name] += 1
                return name
        total = sum(self.pulls.values())
        chosen = max(
            self.names,
            key=lambda name: self.rewards[name] / self.pulls[name]
            + math.sqrt(2.0 * math.log(total) / self.pulls[name]),
        )
        self.pulls[chosen] += 1
        return chosen

    def update(self, name: str, reward: float) -> None:
        if name not in self.pulls:
            return
        self.rewards[name] += max(0.0, float(reward))

    def state(self) -> dict[str, Any]:
        return {"pulls": self.pulls, "rewards": self.rewards}

    def restore(self, state: dict[str, Any]) -> None:
        for name in self.names:
            self.pulls[name] = int(state.get("pulls", {}).get(name, 0))
            self.rewards[name] = float(state.get("rewards", {}).get(name, 0.0))


def trust_region_candidate(
    parent: dict[str, float], bounds: dict[str, Bounds], rng: random.Random,
    radius: float, active_names: set[str],
) -> dict[str, float]:
    child = parent.copy()
    for name in active_names:
        bound = bounds[name]
        child[name] += rng.gauss(0.0, radius * (bound.high - bound.low))
    return repair(child, bounds)


def path_novelty_candidate(
    parent: dict[str, float], bounds: dict[str, Bounds], rng: random.Random, radius: float,
) -> dict[str, float]:
    names = {name for name in bounds if name.startswith("path_offset_")}
    return trust_region_candidate(parent, bounds, rng, radius, names)


def select_feasible_candidates(
    candidates: list[tuple[dict[str, float], str]], fitness_model: Any,
    feasibility_model: KnnFeasibilityModel, count: int, rng: random.Random,
    failure_penalty: float = 200000.0, novelty_fraction: float = 0.2,
) -> list[tuple[dict[str, float], str]]:
    scored = []
    for genome, emitter in candidates:
        prediction, uncertainty = fitness_model.predict(genome)
        probability, feasibility_uncertainty = feasibility_model.predict(genome)
        acquisition = prediction + failure_penalty * (1.0 - probability)
        scored.append((genome, emitter, acquisition, uncertainty + feasibility_uncertainty))
    exploit_count = max(1, count - round(count * novelty_fraction))
    selected = sorted(scored, key=lambda item: item[2])[:exploit_count]
    selected_ids = {id(item[0]) for item in selected}
    remaining = [item for item in scored if id(item[0]) not in selected_ids]
    uncertain = sorted(remaining, key=lambda item: item[3], reverse=True)[: count - len(selected)]
    selected.extend(uncertain)
    rng.shuffle(selected)
    return [(item[0], item[1]) for item in selected]


def load_observations(run_dirs: list[str | Path], required_names: set[str]) -> list[Observation]:
    observations: list[Observation] = []
    for run_dir in run_dirs:
        database = Path(run_dir) / "run.sqlite3"
        if not database.exists():
            continue
        connection = sqlite3.connect(database)
        try:
            rows = connection.execute(
                "SELECT c.candidate_id,c.genome_json,c.fitness,e.metrics_json "
                "FROM candidates c JOIN episodes e ON e.candidate_id=c.candidate_id "
                "WHERE c.status='complete' AND c.fitness IS NOT NULL "
                "ORDER BY c.candidate_id,e.repeat_index"
            ).fetchall()
        finally:
            connection.close()
        grouped: dict[str, tuple[dict[str, float], float, list[dict[str, Any]]]] = {}
        for candidate_id, genome_json, fitness, metrics_json in rows:
            genome = json.loads(genome_json)
            if not required_names.issubset(genome):
                continue
            grouped.setdefault(candidate_id, (genome, float(fitness), []))[2].append(
                json.loads(metrics_json)
            )
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
