from __future__ import annotations

import random

from .genome import Bounds, repair


def tournament(
    ranked: list[tuple[dict[str, float], float]],
    size: int,
    rng: random.Random,
) -> dict[str, float]:
    contenders = rng.sample(ranked, min(size, len(ranked)))
    return min(contenders, key=lambda item: item[1])[0].copy()


def blend_crossover(
    first: dict[str, float],
    second: dict[str, float],
    bounds: dict[str, Bounds],
    rng: random.Random,
    alpha: float = 0.25,
) -> tuple[dict[str, float], dict[str, float]]:
    child_a, child_b = {}, {}
    for name in bounds:
        low = min(first[name], second[name])
        high = max(first[name], second[name])
        span = high - low
        child_a[name] = rng.uniform(low - alpha * span, high + alpha * span)
        child_b[name] = rng.uniform(low - alpha * span, high + alpha * span)
    return repair(child_a, bounds), repair(child_b, bounds)


def mutate(
    genome: dict[str, float],
    bounds: dict[str, Bounds],
    probability: float,
    sigma_fraction: float,
    rng: random.Random,
    active_names: set[str] | None = None,
) -> dict[str, float]:
    mutated = genome.copy()
    for name, limit in bounds.items():
        if active_names is not None and name not in active_names:
            continue
        if rng.random() < probability:
            mutated[name] += rng.gauss(0.0, sigma_fraction * (limit.high - limit.low))
    return repair(mutated, bounds)
