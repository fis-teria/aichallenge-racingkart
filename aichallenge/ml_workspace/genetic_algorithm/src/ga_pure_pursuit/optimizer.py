from __future__ import annotations

import json
import random
import ast
import itertools
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import digest
from .evaluator import InfrastructureError, make_evaluator
from .fitness import robust_score, score
from .genome import bounds_from_config, initialize_population, parameter_hash
from .operators import blend_crossover, mutate, tournament
from .storage import Storage
from .surrogate import KnnSurrogate, load_training_samples, select_candidates


def required_repeat_count(settings: dict[str, Any], metrics: dict[str, Any]) -> int:
    repeat_count = int(settings.get("repeats", 1))
    threshold = settings.get("fast_candidate_lap_time_threshold_sec")
    if (
        threshold is not None
        and bool(metrics.get("completed", False))
        and float(metrics.get("lap_time_seconds", float("inf"))) <= float(threshold)
    ):
        repeat_count = max(
            repeat_count, int(settings.get("fast_candidate_repeats", repeat_count))
        )
    return repeat_count


def stagnation_generations(history: list[tuple[int, float]], epsilon: float = 1.0e-6) -> int:
    best = float("inf")
    last_improvement = -1
    for generation, fitness in history:
        if fitness < best - epsilon:
            best = fitness
            last_improvement = generation
    return 0 if not history else history[-1][0] - last_improvement


def active_gene_names(
    generation: int,
    bounds: dict[str, Any],
    block_generations: int,
) -> tuple[str, set[str]]:
    path_names = {name for name in bounds if name.startswith("path_offset_")}
    controller_names = set(bounds) - path_names
    if not path_names or block_generations <= 0:
        return "all", set(bounds)
    if (generation // block_generations) % 2 == 0:
        return "controller", controller_names
    return "path", path_names


class Optimizer:
    def __init__(
        self,
        config: dict[str, Any],
        root: Path,
        run_id: str | None = None,
        resume: bool = False,
    ):
        self.config = config
        self.run_id = run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.run_dir = root / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)
        (self.run_dir / "checkpoint").mkdir(exist_ok=True)
        self.storage = Storage(self.run_dir / "run.sqlite3")
        self.rng = random.Random(int(config["run"]["seed"]))
        self.bounds = bounds_from_config(config)
        self.evaluator = make_evaluator(config, self.run_dir)
        self.resume = resume
        self._history_lock = threading.Lock()
        surrogate_settings = config.get("surrogate", {})
        self.surrogate = KnnSurrogate(
            self.bounds,
            config["baseline"],
            int(surrogate_settings.get("neighbors", 12)),
        )
        self.external_training_samples = load_training_samples(
            surrogate_settings.get("training_run_dirs", []),
            config["fitness"],
            set(self.bounds),
        )

    def _initial_seeds(self) -> list[dict[str, float]]:
        seeds: list[dict[str, float]] = []
        for filename in self.config["run"].get("initial_seed_files", []):
            data = json.loads(Path(filename).read_text(encoding="utf-8"))
            parameters = data.get("parameters", data)
            if not isinstance(parameters, dict):
                raise ValueError(f"seed does not contain a parameter mapping: {filename}")
            seeds.append({name: float(value) for name, value in parameters.items()})
        seed_count = int(
            self.config.get("surrogate", {}).get("initial_seed_count", 0)
        )
        seeds.extend(
            genome.copy()
            for genome, _ in sorted(
                self.external_training_samples, key=lambda item: item[1]
            )[:seed_count]
        )
        return seeds

    def _save_manifest(self) -> None:
        manifest = {
            "run_id": self.run_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "config_hash": digest(self.config),
            "seed": self.config["run"]["seed"],
            "evaluator_mode": self.config["evaluator"]["mode"],
        }
        (self.run_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8"
        )
        (self.run_dir / "experiment_resolved.json").write_text(
            json.dumps(self.config, indent=2, sort_keys=True), encoding="utf-8"
        )

    def _checkpoint(self, generation: int, population: list[dict[str, float]]) -> None:
        data = {
            "generation": generation,
            "population": population,
            "random_state": repr(self.rng.getstate()),
        }
        target = self.run_dir / "checkpoint" / f"generation_{generation:05d}.json"
        target.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")

    def _resume_state(self) -> tuple[int, list[dict[str, float]]] | None:
        checkpoints = sorted((self.run_dir / "checkpoint").glob("generation_*.json"))
        if not checkpoints:
            return None
        data = json.loads(checkpoints[-1].read_text(encoding="utf-8"))
        self.rng.setstate(ast.literal_eval(data["random_state"]))
        return int(data["generation"]) + 1, data["population"]

    def _evaluate(self, generation: int, index: int, genome: dict[str, float]) -> float:
        hash_value = parameter_hash(genome)
        cached = self.storage.get_fitness(hash_value)
        if cached is not None:
            return cached
        candidate_id = f"g{generation:04d}-i{index:04d}"
        self.storage.begin_candidate(candidate_id, generation, genome, hash_value)
        episodes = []
        retries = int(self.config["evaluator"].get("max_infrastructure_retries", 2))
        settings = self.config["run"]
        repeat = 0
        repeat_count = int(settings.get("repeats", 1))
        while repeat < repeat_count:
            for attempt in range(retries + 1):
                try:
                    metrics = self.evaluator.evaluate(candidate_id, hash_value, genome, repeat)
                    episode_score = score(metrics, self.config["fitness"])
                    episodes.append((repeat, metrics, episode_score))
                    repeat_count = max(
                        repeat_count, required_repeat_count(settings, metrics)
                    )
                    break
                except InfrastructureError:
                    if attempt == retries:
                        raise
            repeat += 1
        fitness = robust_score([item[2] for item in episodes])
        self.storage.complete_candidate(candidate_id, episodes, fitness)
        with self._history_lock:
            with (self.run_dir / "candidates.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(
                        {
                            "candidate_id": candidate_id,
                            "generation": generation,
                            "parameter_hash": hash_value,
                            "parameters": genome,
                            "fitness": fitness,
                        },
                        sort_keys=True,
                    )
                    + "\n"
                )
        return fitness

    def run(self) -> dict[str, Any]:
        if not self.resume:
            self._save_manifest()
        settings = self.config["run"]
        resumed = self._resume_state() if self.resume else None
        if resumed:
            start_generation, population = resumed
        else:
            start_generation = 0
            population = initialize_population(
                self.config["baseline"],
                self.bounds,
                int(settings["population_size"]),
                self.rng,
                self._initial_seeds(),
            )
        generation_limit = int(settings["generations"])
        generation_numbers = (
            itertools.count(start_generation)
            if generation_limit == 0
            else range(start_generation, generation_limit)
        )
        for generation in generation_numbers:
            parallel_workers = int(settings.get("parallel_workers", 1))
            indexed_population = list(enumerate(population))
            if parallel_workers > 1:
                with ThreadPoolExecutor(max_workers=parallel_workers) as executor:
                    fitness_values = list(
                        executor.map(
                            lambda item: self._evaluate(generation, item[0], item[1]),
                            indexed_population,
                        )
                    )
                ranked = [
                    (genome, fitness)
                    for (_, genome), fitness in zip(indexed_population, fitness_values)
                ]
            else:
                ranked = [
                    (genome, self._evaluate(generation, index, genome))
                    for index, genome in indexed_population
                ]
            ranked.sort(key=lambda item: item[1])
            print(
                f"generation={generation} best={ranked[0][1]:.6f} "
                f"hash={parameter_hash(ranked[0][0])}",
                flush=True,
            )
            history = self.storage.generation_best_fitness()
            stagnant = stagnation_generations(
                history, float(settings.get("stagnation_epsilon", 1.0e-4))
            )
            threshold = int(settings.get("adaptive_mutation_after_generations", 5))
            mutation_multiplier = (
                float(settings.get("adaptive_mutation_multiplier", 2.0))
                if stagnant >= threshold else 1.0
            )
            mode, active_names = active_gene_names(
                generation,
                self.bounds,
                int(settings.get("alternating_block_generations", 0)),
            )
            mutation_probability = min(
                1.0,
                float(settings["mutation_probability"]) *
                (1.5 if mutation_multiplier > 1.0 else 1.0),
            )
            mutation_sigma = min(
                0.5,
                float(settings["mutation_sigma_fraction"]) * mutation_multiplier,
            )
            print(
                f"exploration mode={mode} stagnant={stagnant} "
                f"mutation_p={mutation_probability:.3f} sigma={mutation_sigma:.3f}",
                flush=True,
            )
            elites = [item[0].copy() for item in ranked[: int(settings["elite_count"])]]
            population_size = int(settings["population_size"])
            surrogate_settings = self.config.get("surrogate", {})
            surrogate_enabled = bool(surrogate_settings.get("enabled", False))
            pool_multiplier = (
                max(1, int(surrogate_settings.get("candidate_pool_multiplier", 1)))
                if surrogate_enabled else 1
            )
            selection_count = population_size - len(elites)
            candidate_pool_size = max(
                selection_count,
                population_size * pool_multiplier - len(elites),
            )
            immigrant_count = min(
                candidate_pool_size,
                round(
                    candidate_pool_size
                    * float(settings.get("immigrant_fraction", 0.0))
                ),
            )
            breeding_limit = candidate_pool_size - immigrant_count
            candidate_pool: list[dict[str, float]] = []
            while len(candidate_pool) < breeding_limit:
                first = tournament(ranked, int(settings["tournament_size"]), self.rng)
                second = tournament(ranked, int(settings["tournament_size"]), self.rng)
                if self.rng.random() < float(settings["crossover_probability"]):
                    first, second = blend_crossover(first, second, self.bounds, self.rng)
                for child in (first, second):
                    candidate_pool.append(
                        mutate(
                            child,
                            self.bounds,
                            mutation_probability,
                            mutation_sigma,
                            self.rng,
                            active_names,
                        )
                    )
                    if len(candidate_pool) >= breeding_limit:
                        break
            for _ in range(immigrant_count):
                candidate_pool.append(
                    mutate(
                        elites[0],
                        self.bounds,
                        float(settings.get("immigrant_mutation_probability", 0.65)),
                        float(settings.get("immigrant_sigma_fraction", 0.18)),
                        self.rng,
                        active_names,
                    )
                )
            training_samples = (
                self.external_training_samples + self.storage.training_samples()
            )
            minimum_samples = int(
                surrogate_settings.get("minimum_training_samples", 40)
            )
            if surrogate_enabled and len(training_samples) >= minimum_samples:
                self.surrogate.fit(training_samples)
                selected = select_candidates(
                    candidate_pool,
                    self.surrogate,
                    selection_count,
                    self.rng,
                    float(surrogate_settings.get("exploit_fraction", 0.6)),
                    float(surrogate_settings.get("uncertainty_fraction", 0.2)),
                )
                predictions = [
                    self.surrogate.predict(candidate)[0] for candidate in selected
                ]
                print(
                    f"surrogate samples={len(self.surrogate.samples)} "
                    f"pool={len(candidate_pool)} selected={len(selected)} "
                    f"predicted_best={min(predictions):.6f}",
                    flush=True,
                )
            else:
                selected = candidate_pool[:selection_count]
                if surrogate_enabled:
                    print(
                        f"surrogate warmup samples={len(training_samples)}/"
                        f"{minimum_samples}",
                        flush=True,
                    )
            population = elites + selected
            if (generation + 1) % int(settings.get("checkpoint_every", 1)) == 0:
                self._checkpoint(generation, population)
        best = self.storage.best(1)[0]
        (self.run_dir / "best.json").write_text(
            json.dumps(best, indent=2, sort_keys=True), encoding="utf-8"
        )
        self.storage.close()
        return best
