from __future__ import annotations

import random
import json
import threading
import time
import csv
from concurrent.futures import ThreadPoolExecutor

from ga_pure_pursuit.evaluator import SharedAwsimBatchEvaluator, WorkerPoolEvaluator
from ga_pure_pursuit.diagnostics import render_html
from ga_pure_pursuit.episode_monitor import finite_mean, percentile
from ga_pure_pursuit.fitness import (
    high_steering_speed_loss_mps,
    score,
    speed_recovery_deficit_m,
    unintended_speed_loss_mps,
)
from ga_pure_pursuit.genome import Bounds, initialize_population, parameter_hash, repair
from ga_pure_pursuit.operators import blend_crossover, mutate
from ga_pure_pursuit.optimizer import (
    active_gene_names,
    required_repeat_count,
    stagnation_generations,
)
from ga_pure_pursuit.path_optimization import generate_candidate_path, split_genome
from ga_pure_pursuit.progress_dashboard import (
    is_valid_timed_lap,
    render_html as render_progress_html,
    validated_metric,
    validated_lap_time,
)
from ga_pure_pursuit.config import validate_config
from ga_pure_pursuit.surrogate import KnnSurrogate, select_candidates


def test_repair_is_bounded_and_idempotent():
    bounds = {"a": Bounds(0.0, 1.0), "lookahead_min_distance": Bounds(1.0, 6.0),
              "curvature_lookahead_min_distance": Bounds(1.0, 4.0)}
    value = {"a": 2.0, "lookahead_min_distance": 2.0,
             "curvature_lookahead_min_distance": 3.0}
    repaired = repair(value, bounds)
    assert repaired == repair(repaired, bounds)
    assert repaired["a"] == 1.0
    assert repaired["curvature_lookahead_min_distance"] == 2.0


def test_hash_is_order_independent():
    assert parameter_hash({"a": 1.0, "b": 2.0}) == parameter_hash({"b": 2.0, "a": 1.0})


def test_operators_remain_bounded():
    bounds = {"a": Bounds(0.0, 1.0)}
    rng = random.Random(7)
    first, second = blend_crossover({"a": 0.0}, {"a": 1.0}, bounds, rng)
    assert 0.0 <= first["a"] <= 1.0
    assert 0.0 <= second["a"] <= 1.0
    assert 0.0 <= mutate(first, bounds, 1.0, 10.0, rng)["a"] <= 1.0


def test_mutation_can_be_limited_to_active_gene_block():
    bounds = {"controller": Bounds(0.0, 1.0), "path_offset_00": Bounds(-1.0, 1.0)}
    original = {"controller": 0.5, "path_offset_00": 0.0}
    mutated = mutate(
        original, bounds, 1.0, 0.2, random.Random(5), {"path_offset_00"}
    )
    assert mutated["controller"] == original["controller"]
    assert mutated["path_offset_00"] != original["path_offset_00"]


def test_stagnation_and_alternating_gene_blocks():
    assert stagnation_generations([(0, 10.0), (1, 9.0), (2, 9.0), (3, 9.0)]) == 2
    bounds = {"controller": object(), "path_offset_00": object()}
    assert active_gene_names(0, bounds, 5) == ("controller", {"controller"})
    assert active_gene_names(5, bounds, 5) == ("path", {"path_offset_00"})


def test_unfinished_is_ranked_after_completed():
    weights = {
        "invalid": 1_000_000, "unfinished": 200_000, "collision": 20_000,
        "wall": 10_000, "over": 5_000, "penalty_second": 1_000,
        "lateral_error_p95": 5, "steering_delta_rms": 0.5,
        "steering_rate_limit_ratio": 10, "progress_credit": 100,
    }
    assert score({"completed": True, "lap_time_seconds": 100}, weights) < score(
        {"completed": False, "elapsed_seconds": 1, "progress": 0.99}, weights
    )


def test_unintended_speed_loss_uses_lap_clock_and_acceleration_intent():
    metrics = {
        "diagnostic_trace": [
            {"lap": 2, "lap_time_seconds": 1.0, "actual_speed_mps": 8.0,
             "commanded_acceleration_mps2": 1.0},
            {"lap": 2, "lap_time_seconds": 1.2, "actual_speed_mps": 7.8,
             "commanded_acceleration_mps2": 1.0},
            {"lap": 2, "lap_time_seconds": 1.4, "actual_speed_mps": 7.6,
             "commanded_acceleration_mps2": -1.0},
        ]
    }
    # First interval is -1 m/s^2 while acceleration is requested:
    # (-a - threshold) * dt = (1 - 0.3) * 0.2.
    assert abs(unintended_speed_loss_mps(metrics) - 0.14) < 1.0e-9


def test_speed_recovery_deficit_integrates_lap_local_peak_gap():
    metrics = {
        "diagnostic_trace": [
            {"lap": 2, "lap_time_seconds": 1.0, "actual_speed_mps": 9.0,
             "commanded_acceleration_mps2": 0.8},
            {"lap": 2, "lap_time_seconds": 1.5, "actual_speed_mps": 7.0,
             "commanded_acceleration_mps2": 0.8},
            {"lap": 2, "lap_time_seconds": 2.0, "actual_speed_mps": 8.5,
             "commanded_acceleration_mps2": 0.8},
            {"lap": 3, "lap_time_seconds": 0.2, "actual_speed_mps": 6.0,
             "commanded_acceleration_mps2": 0.8},
        ]
    }
    # Deficits beyond the 0.5 m/s deadband: 1.5*0.5 + 0.0*0.5.
    assert abs(speed_recovery_deficit_m(metrics) - 0.75) < 1.0e-9


def test_high_steering_speed_loss_only_counts_large_steering_deceleration():
    metrics = {
        "diagnostic_trace": [
            {"lap": 2, "lap_time_seconds": 1.0, "actual_speed_mps": 9.0,
             "commanded_acceleration_mps2": 0.8, "steering_angle_rad": 0.20},
            {"lap": 2, "lap_time_seconds": 1.2, "actual_speed_mps": 8.6,
             "commanded_acceleration_mps2": 0.8, "steering_angle_rad": 0.22},
            {"lap": 2, "lap_time_seconds": 1.4, "actual_speed_mps": 8.2,
             "commanded_acceleration_mps2": 0.8, "steering_angle_rad": 0.05},
            {"lap": 2, "lap_time_seconds": 1.6, "actual_speed_mps": 7.8,
             "commanded_acceleration_mps2": 0.8, "steering_angle_rad": 0.05},
        ]
    }
    # The first two deceleration intervals touch a large steering sample.
    assert abs(high_steering_speed_loss_mps(metrics) - 0.74) < 1.0e-9


def test_cpu_surrogate_prefers_predicted_fast_candidates_and_keeps_exploration():
    bounds = {"a": Bounds(0.0, 1.0)}
    model = KnnSurrogate(bounds, {"a": 0.5}, neighbors=2)
    model.fit([
        ({"a": 0.0}, 10.0),
        ({"a": 0.2}, 12.0),
        ({"a": 0.8}, 80.0),
        ({"a": 1.0}, 100.0),
    ])
    candidates = [{"a": value / 10.0} for value in range(11)]
    selected = select_candidates(
        candidates, model, 5, random.Random(4),
        exploit_fraction=0.6, uncertainty_fraction=0.2,
    )
    assert len(selected) == 5
    assert any(item["a"] <= 0.2 for item in selected)
    assert len({item["a"] for item in selected}) == 5


def test_lateral_tracking_penalty_has_soft_and_hard_regions():
    weights = {
        "invalid": 1_000_000, "unfinished": 200_000, "collision": 20_000,
        "wall": 10_000, "over": 5_000, "penalty_second": 1_000,
        "lateral_error_p95": 2.0, "lateral_error_max": 0.5,
        "lateral_error_soft_limit_m": 0.6,
        "lateral_error_excess_quadratic": 20.0,
        "lateral_error_hard_limit_m": 1.0,
        "lateral_error_hard_penalty": 100_000.0,
        "steering_delta_rms": 0.5, "steering_rate_limit_ratio": 10,
        "progress_credit": 100,
    }
    base = {"completed": True, "lap_time_seconds": 50, "lateral_error_max_m": 1.0}
    low = score({**base, "lateral_error_p95_m": 0.5}, weights)
    soft = score({**base, "lateral_error_p95_m": 0.8}, weights)
    hard = score({**base, "lateral_error_p95_m": 1.1}, weights)
    assert low < soft < hard
    assert hard > 100_000


def test_worker_pool_round_trip(tmp_path):
    evaluator = WorkerPoolEvaluator(
        {
            "worker_ids": ["worker-1"],
            "ipc_root": str(tmp_path),
            "poll_interval_sec": 0.01,
        },
        timeout_sec=2.0,
        run_dir=tmp_path / "run",
    )

    def fake_worker():
        requests = tmp_path / "worker-1" / "requests"
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            paths = list(requests.glob("*.json")) if requests.exists() else []
            if paths:
                request = json.loads(paths[0].read_text(encoding="utf-8"))
                result = tmp_path / "worker-1" / "results" / f"{request['job_id']}.json"
                result.parent.mkdir(parents=True, exist_ok=True)
                result.write_text(
                    json.dumps({"completed": True, "exit_reason": "completed"}),
                    encoding="utf-8",
                )
                return
            time.sleep(0.01)

    worker = threading.Thread(target=fake_worker)
    worker.start()
    metrics = evaluator.evaluate("candidate", "sha256:test", {"a": 1.0}, 0)
    worker.join()
    assert metrics["completed"] is True


def test_shared_awsim_evaluator_collects_four_calls(tmp_path):
    evaluator = SharedAwsimBatchEvaluator(
        {
            "vehicle_domain_ids": [1, 2, 3, 4],
            "batch_collect_timeout_sec": 0.05,
            "batch_barrier_timeout_sec": 1.0,
        },
        timeout_sec=2.0,
        run_dir=tmp_path,
    )
    observed = []

    def fake_batch(requests):
        observed.append([request[0] for request in requests])
        return [
            {"completed": True, "candidate_id": request[0]}
            for request in requests
        ]

    evaluator._execute_batch = fake_batch
    with ThreadPoolExecutor(max_workers=4) as executor:
        results = list(
            executor.map(
                lambda index: evaluator.evaluate(
                    f"candidate-{index}", f"hash-{index}", {"a": index}, 0
                ),
                range(4),
            )
        )
    assert len(observed) == 1
    assert set(observed[0]) == {f"candidate-{index}" for index in range(4)}
    assert {result["candidate_id"] for result in results} == set(observed[0])


def test_shared_awsim_pool_splits_eight_calls_between_environments(tmp_path):
    evaluator = SharedAwsimBatchEvaluator(
        {
            "environments": [
                {
                    "name": "env1",
                    "admin_domain_id": 0,
                    "vehicle_domain_ids": [1, 2, 3, 4],
                },
                {
                    "name": "env2",
                    "admin_domain_id": 10,
                    "vehicle_domain_ids": [11, 12, 13, 14],
                },
            ],
            "batch_collect_timeout_sec": 0.05,
            "batch_barrier_timeout_sec": 1.0,
        },
        timeout_sec=2.0,
        run_dir=tmp_path,
    )
    observed = []

    def fake_environment(requests, environment):
        observed.append(
            (
                environment["name"],
                environment["admin_domain_id"],
                [request[0] for request in requests],
            )
        )
        return [
            {"completed": True, "candidate_id": request[0]}
            for request in requests
        ]

    evaluator._execute_environment = fake_environment
    with ThreadPoolExecutor(max_workers=8) as executor:
        results = list(
            executor.map(
                lambda index: evaluator.evaluate(
                    f"candidate-{index}", f"hash-{index}", {"a": index}, 0
                ),
                range(8),
            )
        )
    assert {(name, admin) for name, admin, _ in observed} == {
        ("env1", 0), ("env2", 10)
    }
    assert all(len(candidates) == 4 for _, _, candidates in observed)
    assert {result["candidate_id"] for result in results} == {
        f"candidate-{index}" for index in range(8)
    }


def test_shared_awsim_partial_batch_marks_unassigned_domains_idle(tmp_path):
    evaluator = SharedAwsimBatchEvaluator(
        {
            "environments": [
                {
                    "name": "env1",
                    "admin_domain_id": 0,
                    "vehicle_domain_ids": [1, 2, 3, 4],
                },
                {
                    "name": "env2",
                    "admin_domain_id": 10,
                    "vehicle_domain_ids": [11, 12, 13, 14],
                },
            ],
            "batch_collect_timeout_sec": 0.05,
            "batch_barrier_timeout_sec": 1.0,
        },
        timeout_sec=2.0,
        run_dir=tmp_path,
    )
    idle_domains = []
    observed = []
    evaluator._set_domain_idle = idle_domains.append

    def fake_environment(requests, environment):
        observed.append((environment["name"], [item[0] for item in requests]))
        return [{"candidate_id": item[0]} for item in requests]

    evaluator._execute_environment = fake_environment
    results = evaluator._execute_batch(
        [
            ("candidate-a", "hash-a", {"a": 1.0}, 0),
            ("candidate-b", "hash-b", {"a": 2.0}, 0),
        ]
    )

    assert observed == [("env1", ["candidate-a", "candidate-b"])]
    assert set(idle_domains) == {3, 4, 11, 12, 13, 14}
    assert [result["candidate_id"] for result in results] == [
        "candidate-a", "candidate-b"
    ]


def test_speed_diagnostic_helpers_and_html():
    assert finite_mean([]) == 0.0
    assert finite_mean([1.0, 3.0]) == 2.0
    assert percentile([3.0, 1.0, 2.0], 0.95) == 3.0
    metrics = {
        "elapsed_seconds": 2.0,
        "actual_speed_mean_mps": 5.0,
        "speed_limited_ratio": 0.5,
        "diagnostic_trace": [
            {
                "elapsed_seconds": 0.0,
                "actual_speed_mps": 0.0,
                "target_speed_mps": 9.0,
                "curvature_speed_limit_mps": 8.0,
                "speed_limited": True,
            },
            {
                "elapsed_seconds": 2.0,
                "actual_speed_mps": 8.0,
                "target_speed_mps": 9.0,
                "curvature_speed_limit_mps": 10.0,
                "speed_limited": False,
            },
        ],
        "section_splits": [],
    }
    document = render_html("candidate-test", metrics)
    assert "candidate-test" in document
    assert "Actual speed" in document
    assert "50.0%" in document


def test_progress_dashboard_html_contains_explanatory_panels():
    document = render_progress_html({
        "run_id": "test-run",
        "updated": "now",
        "generations": [],
        "gene_names": [],
        "gene_heatmap": [],
        "baseline_path": [],
        "best_path": [],
        "trace": [],
        "best": {},
    })
    assert "GA Progress Dashboard" in document
    assert "世代ごとの改善" in document
    assert "速度追従" in document
    assert "遺伝子の収束" in document


def test_zero_generations_is_accepted_as_unlimited():
    config = {
        "run": {"population_size": 4, "generations": 0, "elite_count": 1},
        "evaluator": {},
        "baseline": {"a": 0.5},
        "search_space": {"a": {"min": 0.0, "max": 1.0}},
        "fitness": {},
    }
    validate_config(config)


def test_shared_awsim_config_requires_one_worker_per_domain():
    config = {
        "run": {
            "population_size": 4,
            "generations": 1,
            "elite_count": 1,
            "parallel_workers": 4,
        },
        "evaluator": {
            "mode": "shared_awsim_batch",
            "vehicle_domain_ids": [1, 2, 3, 4],
        },
        "baseline": {"a": 0.5},
        "search_space": {"a": {"min": 0.0, "max": 1.0}},
        "fitness": {},
    }
    validate_config(config)
    config["run"]["parallel_workers"] = 3
    try:
        validate_config(config)
    except ValueError as error:
        assert "domain count" in str(error)
    else:
        raise AssertionError("mismatched shared-AWSIM concurrency was accepted")


def test_shared_awsim_pool_config_requires_unique_domains():
    config = {
        "run": {
            "population_size": 16,
            "generations": 1,
            "elite_count": 1,
            "parallel_workers": 8,
        },
        "evaluator": {
            "mode": "shared_awsim_batch",
            "environments": [
                {"admin_domain_id": 0, "vehicle_domain_ids": [1, 2, 3, 4]},
                {"admin_domain_id": 10, "vehicle_domain_ids": [11, 12, 13, 14]},
            ],
        },
        "baseline": {"a": 0.5},
        "search_space": {"a": {"min": 0.0, "max": 1.0}},
        "fitness": {},
    }
    validate_config(config)
    config["evaluator"]["environments"][1]["vehicle_domain_ids"][0] = 1
    try:
        validate_config(config)
    except ValueError as error:
        assert "unique vehicle domains" in str(error)
    else:
        raise AssertionError("duplicate pool vehicle domains were accepted")


def test_dashboard_rejects_reset_lap_timer_and_uses_repeat_median():
    broken = {
        "completed": True,
        "lap_time_seconds": 48.4,
        "section_splits": [{"section": 1, "lap_time_seconds": 0.02}],
    }
    normal_first = {
        "completed": True,
        "lap_time_seconds": 56.3,
        "section_splits": [{"section": 1, "lap_time_seconds": 7.8}],
    }
    normal_second = {
        "completed": True,
        "lap_time_seconds": 56.7,
        "section_splits": [{"section": 1, "lap_time_seconds": 7.9}],
    }
    assert not is_valid_timed_lap(broken)
    assert is_valid_timed_lap(normal_first)
    assert validated_lap_time(
        {"episodes": [broken, normal_first, normal_second]}
    ) == 56.5
    normal_first["flying_lap_time_seconds"] = 44.0
    normal_second["flying_lap_time_seconds"] = 45.0
    assert validated_metric(
        {"episodes": [broken, normal_first, normal_second]},
        "flying_lap_time_seconds",
    ) == 44.5


def test_fast_candidate_gets_repeated_validation():
    settings = {
        "repeats": 1,
        "fast_candidate_lap_time_threshold_sec": 55.0,
        "fast_candidate_repeats": 3,
    }
    assert required_repeat_count(
        settings, {"completed": True, "lap_time_seconds": 54.9}
    ) == 3
    assert required_repeat_count(
        settings, {"completed": True, "lap_time_seconds": 55.1}
    ) == 1
    assert required_repeat_count(
        settings, {"completed": False, "lap_time_seconds": 40.0}
    ) == 1


def test_seed_is_injected_and_missing_new_genes_use_baseline():
    bounds = {"old": Bounds(0.0, 10.0), "new": Bounds(0.0, 1.0)}
    population = initialize_population(
        {"old": 5.0, "new": 0.25},
        bounds,
        4,
        random.Random(3),
        seeds=[{"old": 7.0}],
    )
    assert population[0] == {"old": 5.0, "new": 0.25}
    assert population[1] == {"old": 7.0, "new": 0.25}


def test_path_genes_are_split_and_generate_closed_offset_path(tmp_path):
    source = tmp_path / "source.csv"
    fieldnames = ("x", "y", "z", "x_quat", "y_quat", "z_quat", "w_quat", "speed")
    rows = [
        (0, -1), (0, 0), (1, 0), (1, 1), (0, 1), (0, 0)
    ]
    with source.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fieldnames)
        for x, y in rows:
            writer.writerow((x, y, 0, 0, 0, 0, 1, 9))
    controller, path = split_genome(
        {"lookahead_gain": 1.0, "path_offset_00": 0.2, "path_offset_01": 0.2}
    )
    assert controller == {"lookahead_gain": 1.0}
    destination = tmp_path / "candidate.csv"
    metrics = generate_candidate_path(
        source,
        destination,
        path,
        anchor_count=2,
        loop_start_index=1,
    )
    with destination.open(newline="", encoding="utf-8") as stream:
        generated = list(csv.DictReader(stream))
    assert generated[1]["x"] == generated[-1]["x"]
    assert generated[1]["y"] == generated[-1]["y"]
    assert metrics["path_offset_max_m"] == 0.2
    assert metrics["path_length_m"] > 0.0
    assert metrics["path_length_excess_m"] >= 0.0
