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
from .phase2 import (
    EmitterBandit, KnnFeasibilityModel, MapElitesArchive, load_observations,
    path_novelty_candidate, select_feasible_candidates, trust_region_candidate,
)
from .cmaes import BlockCmaEs
from .constrained_objective import evaluate_constrained


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
        phase2 = config.get("phase2", {})
        self.phase2_enabled = bool(phase2.get("enabled", False))
        emitter_names = ["ga", "trust_region", "path_novelty", "random_restart"]
        self.emitter_bandit = EmitterBandit(emitter_names)
        self.feasibility = KnnFeasibilityModel(
            self.bounds, config["baseline"], int(phase2.get("feasibility_neighbors", 16))
        )
        training_dirs = list(surrogate_settings.get("training_run_dirs", []))
        training_dirs.extend(phase2.get("training_run_dirs", []))
        self.external_observations = load_observations(training_dirs, set(self.bounds))
        self.archive = MapElitesArchive(tuple(phase2.get("archive_bins", [8, 8])))
        self.archive.build(self.external_observations)

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

    def _checkpoint(self, generation: int, population: list[dict[str, float]], emitters=None, optimizer_state=None) -> None:
        data = {
            "generation": generation,
            "population": population,
            "random_state": repr(self.rng.getstate()),
            "population_emitters": emitters or ["seed"] * len(population),
            "emitter_bandit": self.emitter_bandit.state(),
            "optimizer_type": self.config.get("optimizer", {}).get("type", "genetic_algorithm"),
            "optimizer_state": optimizer_state,
        }
        target = self.run_dir / "checkpoint" / f"generation_{generation:05d}.json"
        target.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")

    def _resume_state(self) -> tuple[int, list[dict[str, float]]] | None:
        checkpoints = sorted((self.run_dir / "checkpoint").glob("generation_*.json"))
        if not checkpoints:
            return None
        data = json.loads(checkpoints[-1].read_text(encoding="utf-8"))
        self.rng.setstate(ast.literal_eval(data["random_state"]))
        self.emitter_bandit.restore(data.get("emitter_bandit", {}))
        self._resumed_emitters = data.get("population_emitters", ["seed"] * len(data["population"]))
        return int(data["generation"]) + 1, data["population"]

    def _evaluate(self, generation: int, index: int, genome: dict[str, float],
                  constrained: bool = False, phase: str = "ga") -> float:
        hash_value = parameter_hash(genome)
        cached = (self.storage.get_objective(hash_value) if constrained
                  else self.storage.get_fitness(hash_value))
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
        objective = None
        if constrained:
            objective = evaluate_constrained(
                [item[1] for item in episodes], self.config["constrained_objective"]
            )
            self.storage.save_candidate_metadata(
                candidate_id, "block_cmaes", phase, objective.to_dict()
            )
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
                            "objective_value": (
                                objective.objective_value if objective else None
                            ),
                            "feasible": objective.feasible if objective else None,
                            "robust_lap": objective.robust_lap if objective else None,
                        },
                        sort_keys=True,
                    )
                    + "\n"
                )
        return objective.objective_value if objective else fitness

    def run(self) -> dict[str, Any]:
        if self.config.get("optimizer", {}).get("type") == "block_cmaes":
            return self._run_block_cmaes()
        if not self.resume:
            self._save_manifest()
        settings = self.config["run"]
        resumed = self._resume_state() if self.resume else None
        if resumed:
            start_generation, population = resumed
            population_emitters = self._resumed_emitters
        else:
            start_generation = 0
            population = initialize_population(
                self.config["baseline"],
                self.bounds,
                int(settings["population_size"]),
                self.rng,
                self._initial_seeds(),
            )
            population_emitters = ["seed"] * len(population)
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
            if self.phase2_enabled:
                cutoff = sorted(fitness_values)[max(0, len(fitness_values) // 4 - 1)]
                for emitter, fitness in zip(population_emitters, fitness_values):
                    self.emitter_bandit.update(emitter, 1.0 if fitness <= cutoff else 0.0)
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
            candidate_emitters: list[str] = []
            phase2_settings = self.config.get("phase2", {})
            archive_parents = self.archive.genomes()
            emitter_allocation = (
                self.emitter_bandit.allocation(
                    breeding_limit, self.rng,
                    float(phase2_settings.get("minimum_emitter_pool_fraction", 0.1)),
                )
                if self.phase2_enabled else ["ga"] * breeding_limit
            )
            while len(candidate_pool) < breeding_limit:
                emitter = emitter_allocation[len(candidate_pool)]
                first = tournament(ranked, int(settings["tournament_size"]), self.rng)
                second = tournament(ranked, int(settings["tournament_size"]), self.rng)
                if emitter == "trust_region":
                    parent = self.rng.choice(archive_parents or elites)
                    candidate_pool.append(trust_region_candidate(
                        parent, self.bounds, self.rng,
                        float(phase2_settings.get("trust_region_radius", mutation_sigma)),
                        active_names,
                    ))
                    candidate_emitters.append(emitter)
                    continue
                if emitter == "path_novelty":
                    parent = self.rng.choice(archive_parents or elites)
                    candidate_pool.append(path_novelty_candidate(
                        parent, self.bounds, self.rng,
                        float(phase2_settings.get("path_novelty_radius", 0.16)),
                    ))
                    candidate_emitters.append(emitter)
                    continue
                if emitter == "random_restart":
                    candidate_pool.append(initialize_population(
                        self.config["baseline"], self.bounds, 1, self.rng
                    )[0])
                    candidate_emitters.append(emitter)
                    continue
                if self.rng.random() < float(settings["crossover_probability"]):
                    first, second = blend_crossover(first, second, self.bounds, self.rng)
                candidate_pool.append(
                    mutate(
                        first,
                        self.bounds,
                        mutation_probability,
                        mutation_sigma,
                        self.rng,
                        active_names,
                    )
                )
                candidate_emitters.append("ga")
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
                candidate_emitters.append("random_restart")
            training_samples = (
                self.external_training_samples + self.storage.training_samples()
            )
            minimum_samples = int(
                surrogate_settings.get("minimum_training_samples", 40)
            )
            if surrogate_enabled and len(training_samples) >= minimum_samples:
                self.surrogate.fit(training_samples)
                if self.phase2_enabled:
                    observations = self.external_observations + self.storage.observations()
                    self.feasibility.fit(observations)
                    self.archive.build(observations)
                    selected_pairs = select_feasible_candidates(
                        list(zip(candidate_pool, candidate_emitters)), self.surrogate,
                        self.feasibility, selection_count, self.rng,
                        float(phase2_settings.get("failure_penalty", 200000.0)),
                        float(phase2_settings.get("novelty_fraction", 0.2)),
                        int(phase2_settings.get("minimum_selected_per_emitter", 2)),
                    )
                    selected = [item[0] for item in selected_pairs]
                    selected_emitters = [item[1] for item in selected_pairs]
                else:
                    selected = select_candidates(
                        candidate_pool, self.surrogate, selection_count, self.rng,
                        float(surrogate_settings.get("exploit_fraction", 0.6)),
                        float(surrogate_settings.get("uncertainty_fraction", 0.2)),
                    )
                    selected_emitters = [candidate_emitters[candidate_pool.index(item)] for item in selected]
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
                selected_emitters = candidate_emitters[:selection_count]
                if surrogate_enabled:
                    print(
                        f"surrogate warmup samples={len(training_samples)}/"
                        f"{minimum_samples}",
                        flush=True,
                    )
            population = elites + selected
            population_emitters = ["elite"] * len(elites) + selected_emitters
            if (generation + 1) % int(settings.get("checkpoint_every", 1)) == 0:
                self._checkpoint(generation, population, population_emitters)
        best = self.storage.best(1)[0]
        (self.run_dir / "best.json").write_text(
            json.dumps(best, indent=2, sort_keys=True), encoding="utf-8"
        )
        self.storage.close()
        return best

    def _run_block_cmaes(self) -> dict[str, Any]:
        if not self.resume:
            self._save_manifest()
        settings = self.config["run"]
        optimizer_settings = dict(self.config.get("optimizer", {}))
        optimizer_settings.setdefault("population_size", settings["population_size"])
        checkpoints = sorted((self.run_dir / "checkpoint").glob("generation_*.json"))
        if self.resume and checkpoints:
            data = json.loads(checkpoints[-1].read_text(encoding="utf-8"))
            if data.get("optimizer_type") != "block_cmaes" or not data.get("optimizer_state"):
                raise ValueError("latest checkpoint is not a block_cmaes checkpoint")
            cma_optimizer = BlockCmaEs.restore(self.bounds, optimizer_settings, data["optimizer_state"])
            start_generation = int(data["generation"]) + 1
        else:
            seeds = self._initial_seeds()
            source = seeds[0] if seeds else self.config["baseline"]
            center = {name: float(source.get(name, self.config["baseline"][name])) for name in self.bounds}
            cma_optimizer = BlockCmaEs(self.bounds, center, optimizer_settings, int(settings["seed"]))
            start_generation = 0
        generation_limit = int(settings["generations"])
        generation_numbers = itertools.count(start_generation) if generation_limit == 0 else range(start_generation, generation_limit)
        for generation in generation_numbers:
            population = cma_optimizer.ask()
            indexed = list(enumerate(population))
            workers = int(settings.get("parallel_workers", 1))
            if workers > 1:
                with ThreadPoolExecutor(max_workers=workers) as executor:
                    fitness_values = list(executor.map(
                        lambda item: self._evaluate(
                            generation, item[0], item[1], True, cma_optimizer.phase
                        ), indexed))
            else:
                fitness_values = [self._evaluate(
                    generation, index, genome, True, cma_optimizer.phase
                ) for index, genome in indexed]
            transition = cma_optimizer.tell(population, fitness_values)
            best_index = min(range(len(population)), key=lambda index: fitness_values[index])
            print(f"generation={generation} optimizer=block_cmaes phase={cma_optimizer.phase} "
                  f"sigma={cma_optimizer.sigma:.6f} best={fitness_values[best_index]:.6f} "
                  f"hash={parameter_hash(population[best_index])}", flush=True)
            if transition:
                previous = cma_optimizer.phase
                cma_optimizer.transition()
                print(f"cmaes phase transition {previous}->{cma_optimizer.phase}", flush=True)
            if (generation + 1) % int(settings.get("checkpoint_every", 1)) == 0:
                self._checkpoint(generation, population,
                                 [f"cmaes:{cma_optimizer.phase}"] * len(population),
                                 cma_optimizer.checkpoint())
        best = self.storage.best_objective(1)[0]
        (self.run_dir / "best.json").write_text(json.dumps(best, indent=2, sort_keys=True), encoding="utf-8")
        self.storage.close()
        return best
