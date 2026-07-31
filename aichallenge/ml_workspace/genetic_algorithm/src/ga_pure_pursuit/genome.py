from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Any

from .config import digest


@dataclass(frozen=True)
class Bounds:
    low: float
    high: float

    def clip(self, value: float) -> float:
        return min(self.high, max(self.low, float(value)))


def bounds_from_config(config: dict[str, Any]) -> dict[str, Bounds]:
    return {
        name: Bounds(float(spec["min"]), float(spec["max"]))
        for name, spec in config["search_space"].items()
    }


def repair(genome: dict[str, float], bounds: dict[str, Bounds]) -> dict[str, float]:
    repaired = {name: limit.clip(genome[name]) for name, limit in bounds.items()}
    if {
        "curvature_lookahead_min_distance",
        "lookahead_min_distance",
    }.issubset(repaired):
        repaired["curvature_lookahead_min_distance"] = min(
            repaired["curvature_lookahead_min_distance"],
            repaired["lookahead_min_distance"],
        )
    return repaired


def parameter_hash(genome: dict[str, float]) -> str:
    normalized = {name: round(float(value), 12) for name, value in genome.items()}
    return digest(normalized)


def random_genome(bounds: dict[str, Bounds], rng: random.Random) -> dict[str, float]:
    return repair(
        {name: rng.uniform(limit.low, limit.high) for name, limit in bounds.items()},
        bounds,
    )


def initialize_population(
    baseline: dict[str, float],
    bounds: dict[str, Bounds],
    size: int,
    rng: random.Random,
    seeds: list[dict[str, float]] | None = None,
) -> list[dict[str, float]]:
    population = []
    for seed in [baseline, *(seeds or [])]:
        complete = {name: float(seed.get(name, baseline[name])) for name in bounds}
        repaired = repair(complete, bounds)
        if parameter_hash(repaired) not in {parameter_hash(item) for item in population}:
            population.append(repaired)
    local_count = max(1, size // 3)
    while len(population) < size:
        if len(population) <= local_count:
            genome = {
                name: float(baseline[name])
                + rng.gauss(0.0, 0.08 * (limit.high - limit.low))
                for name, limit in bounds.items()
            }
            genome = repair(genome, bounds)
        else:
            genome = random_genome(bounds, rng)
        if parameter_hash(genome) not in {parameter_hash(item) for item in population}:
            population.append(genome)
    return population
