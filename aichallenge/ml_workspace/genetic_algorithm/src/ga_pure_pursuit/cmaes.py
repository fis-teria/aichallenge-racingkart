from __future__ import annotations

import base64
import pickle
from dataclasses import dataclass
from typing import Any

from .genome import Bounds, repair

CONTROLLER_GENES = (
    "lookahead_gain", "lookahead_min_distance", "steering_tire_angle_gain",
    "curvature_lookahead_min_distance", "curvature_lookahead_sensitivity",
    "curvature_lookahead_smoothing_alpha", "curvature_speed_preview_distance",
    "dual_preview_near_ratio", "dual_preview_blend",
)


def phase_gene_names(phase: str, bounds: dict[str, Bounds]) -> tuple[str, ...]:
    path = tuple(sorted(name for name in bounds if name.startswith("path_offset_")))
    controller = tuple(name for name in CONTROLLER_GENES if name in bounds)
    if phase == "controller":
        return controller or tuple(name for name in bounds if name not in path)
    if phase == "path":
        return path
    if phase == "joint":
        return tuple(bounds)
    raise ValueError(f"unknown CMA-ES phase: {phase}")


def encode_genome(genome, names, bounds) -> list[float]:
    return [(float(genome[name]) - bounds[name].low) /
            (bounds[name].high - bounds[name].low) for name in names]


def decode_genome(values, names, fixed, bounds) -> dict[str, float]:
    genome = dict(fixed)
    for name, value in zip(names, values):
        limit = bounds[name]
        normalized = min(1.0, max(0.0, float(value)))
        genome[name] = limit.low + normalized * (limit.high - limit.low)
    return repair(genome, bounds)


@dataclass
class BlockCmaState:
    phase: str
    fixed: dict[str, float]
    best_genome: dict[str, float]
    best_fitness: float
    stagnant_generations: int = 0
    phase_generations: int = 0


class BlockCmaEs:
    """Checkpointable pycma adapter operating in normalized gene space."""

    def __init__(self, bounds, center, settings: dict[str, Any], seed: int,
                 phase: str = "controller"):
        try:
            import cma
        except ImportError as error:
            raise RuntimeError("block_cmaes requires the 'cma' and 'numpy' packages") from error
        self.bounds, self.settings, self.seed = bounds, settings, int(seed)
        self.state = BlockCmaState(phase, dict(center), dict(center), float("inf"))
        self.names = phase_gene_names(phase, bounds)
        if not self.names:
            raise ValueError(f"CMA-ES phase {phase!r} has no active genes")
        sigma = float(settings.get(f"{phase}_sigma", {
            "controller": 0.04, "path": 0.03, "joint": 0.015}[phase]))
        self.es = cma.CMAEvolutionStrategy(
            encode_genome(center, self.names, bounds), sigma,
            {"bounds": [0.0, 1.0], "popsize": int(settings.get("population_size", 32)),
             "seed": self.seed, "verbose": -9},
        )

    @property
    def phase(self):
        return self.state.phase

    @property
    def sigma(self):
        return float(self.es.sigma)

    def ask(self):
        self._asked = self.es.ask()
        # pycma samples around its mean but does not guarantee that the known
        # best point itself is evaluated. Preserve one exact elite so a phase
        # can never discard the validated center solely through sampling noise.
        self._asked[0] = encode_genome(
            self.state.best_genome, self.names, self.bounds
        )
        return [decode_genome(x, self.names, self.state.fixed, self.bounds)
                for x in self._asked]

    def tell(self, genomes, fitness) -> bool:
        if len(genomes) != len(fitness) or len(genomes) != len(self._asked):
            raise ValueError("CMA-ES ask/tell population sizes differ")
        self.es.tell(self._asked, [float(value) for value in fitness])
        best_index = min(range(len(fitness)), key=lambda index: fitness[index])
        best = float(fitness[best_index])
        threshold = float(self.settings.get("minimum_improvement_seconds", 0.01))
        if self.state.best_fitness - best >= threshold:
            self.state.best_genome = dict(genomes[best_index])
            self.state.best_fitness = best
            self.state.stagnant_generations = 0
        else:
            self.state.stagnant_generations += 1
            if best < self.state.best_fitness:
                self.state.best_genome, self.state.best_fitness = dict(genomes[best_index]), best
        self.state.phase_generations += 1
        return (self.phase == "joint" and self.state.phase_generations >=
                int(self.settings.get("joint_max_generations", 5))) or (
                self.state.stagnant_generations >=
                int(self.settings.get("phase_stagnation_generations", 6)))

    def transition(self):
        next_phase = {"controller": "path", "path": "joint", "joint": "controller"}[self.phase]
        best, best_fitness = dict(self.state.best_genome), self.state.best_fitness
        replacement = BlockCmaEs(self.bounds, best, self.settings, self.seed + 1, next_phase)
        replacement.state.best_genome, replacement.state.best_fitness = best, best_fitness
        self.__dict__.update(replacement.__dict__)

    def checkpoint(self):
        return {
            "phase": self.state.phase, "fixed": self.state.fixed,
            "best_genome": self.state.best_genome, "best_fitness": self.state.best_fitness,
            "stagnant_generations": self.state.stagnant_generations,
            "phase_generations": self.state.phase_generations, "seed": self.seed,
            "active_genes": list(self.names),
            "pycma_state": base64.b64encode(pickle.dumps(self.es)).decode("ascii"),
        }

    @classmethod
    def restore(cls, bounds, settings, data):
        instance = cls(bounds, data["fixed"], settings, int(data["seed"]), str(data["phase"]))
        instance.es = pickle.loads(base64.b64decode(data["pycma_state"]))
        instance.names = tuple(data["active_genes"])
        instance.state = BlockCmaState(
            str(data["phase"]), {k: float(v) for k, v in data["fixed"].items()},
            {k: float(v) for k, v in data["best_genome"].items()},
            float(data["best_fitness"]), int(data["stagnant_generations"]),
            int(data["phase_generations"]),
        )
        return instance
