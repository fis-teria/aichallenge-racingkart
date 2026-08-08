#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sqlite3
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ga_pure_pursuit.constrained_objective import evaluate_constrained


DEFAULT_SETTINGS = {
    "required_complete_repeat_ratio": 1.0,
    "maximum_collision_count": 0,
    "maximum_wall_count": 0,
    "lateral_error_p95_limit_m": 1.35,
    "lateral_error_max_limit_m": 2.0,
    "infeasible_objective_base": 1_000_000.0,
}


def _read_candidates(database: Path) -> list[dict[str, Any]]:
    connection = sqlite3.connect(f"file:{database.resolve()}?mode=ro", uri=True)
    try:
        rows = connection.execute(
            "SELECT c.candidate_id, c.generation, c.genome_json, c.parameter_hash, "
            "c.fitness, e.repeat_index, e.metrics_json FROM candidates c "
            "JOIN episodes e ON e.candidate_id=c.candidate_id "
            "WHERE c.status='complete' AND c.fitness IS NOT NULL "
            "ORDER BY c.candidate_id, e.repeat_index"
        ).fetchall()
    finally:
        connection.close()
    grouped: dict[str, dict[str, Any]] = {}
    for candidate_id, generation, genome, hash_value, fitness, repeat, metrics in rows:
        item = grouped.setdefault(candidate_id, {
            "candidate_id": candidate_id, "generation": int(generation),
            "parameters": json.loads(genome), "parameter_hash": hash_value,
            "fitness": float(fitness), "episodes": [],
        })
        item["episodes"].append(json.loads(metrics))
    return list(grouped.values())


def _rank(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=values.__getitem__)
    ranks = [0.0] * len(values)
    position = 0
    while position < len(order):
        end = position + 1
        while end < len(order) and values[order[end]] == values[order[position]]:
            end += 1
        rank = (position + end - 1) / 2.0
        for index in order[position:end]:
            ranks[index] = rank
        position = end
    return ranks


def _spearman(first: list[float], second: list[float]) -> float | None:
    if len(first) < 2:
        return None
    x, y = _rank(first), _rank(second)
    mean_x, mean_y = statistics.mean(x), statistics.mean(y)
    numerator = sum((a - mean_x) * (b - mean_y) for a, b in zip(x, y))
    denominator = math.sqrt(sum((a - mean_x) ** 2 for a in x) *
                            sum((b - mean_y) ** 2 for b in y))
    return None if denominator == 0.0 else numerator / denominator


def analyze(databases: list[Path], settings: dict[str, Any]) -> dict[str, Any]:
    candidates = []
    for database in databases:
        for candidate in _read_candidates(database):
            candidate["run_id"] = database.parent.name
            objective = evaluate_constrained(candidate["episodes"], settings)
            candidate["objective"] = objective.to_dict()
            candidates.append(candidate)
    failures = Counter(reason for item in candidates
                       for reason in item["objective"]["failure_reasons"])
    feasible = [item for item in candidates if item["objective"]["feasible"]]
    feasible.sort(key=lambda item: (
        item["objective"]["robust_lap"], item["objective"]["lateral_error_p95_m"],
        item["parameter_hash"],
    ))
    fitness_order = sorted(candidates, key=lambda item: item["fitness"])
    fitness_position = {(item["run_id"], item["candidate_id"]): index + 1
                        for index, item in enumerate(fitness_order)}
    top = []
    for rank, item in enumerate(feasible[:30], 1):
        objective = item["objective"]
        top.append({
            "rank": rank, "run_id": item["run_id"],
            "candidate_id": item["candidate_id"], "generation": item["generation"],
            "parameter_hash": item["parameter_hash"], "fitness": item["fitness"],
            "fitness_rank": fitness_position[(item["run_id"], item["candidate_id"])],
            "robust_lap": objective["robust_lap"],
            "lateral_error_p95_m": objective["lateral_error_p95_m"],
            "lateral_error_max_m": objective["lateral_error_max_m"],
            "repeat_count": len(item["episodes"]),
        })
    correlation = _spearman(
        [item["fitness"] for item in feasible],
        [item["objective"]["robust_lap"] for item in feasible],
    )
    sensitivity = []
    for p95 in (1.10, 1.20, 1.35, 1.50, 1.75, 2.00):
        for maximum in (1.80, 2.00, 2.50, 3.00):
            varied = dict(settings, lateral_error_p95_limit_m=p95,
                          lateral_error_max_limit_m=maximum)
            feasible_count = sum(evaluate_constrained(item["episodes"], varied).feasible
                                 for item in candidates)
            sensitivity.append({
                "lateral_error_p95_limit_m": p95,
                "lateral_error_max_limit_m": maximum,
                "feasible_count": feasible_count,
                "feasible_ratio": feasible_count / len(candidates) if candidates else 0.0,
            })
    requested = {}
    for identifier in ("g0086-i0028", "g0099-i0022"):
        matches = [item for item in candidates if item["candidate_id"] == identifier]
        requested[identifier] = [{
            "run_id": item["run_id"], "fitness": item["fitness"],
            **item["objective"],
        } for item in matches]
    return {
        "settings": settings, "database_count": len(databases),
        "candidate_count": len(candidates), "feasible_count": len(feasible),
        "feasible_ratio": len(feasible) / len(candidates) if candidates else 0.0,
        "failure_counts": dict(sorted(failures.items())),
        "fitness_robust_lap_spearman": correlation,
        "top_robust_lap": top, "requested_candidates": requested,
        "sensitivity": sensitivity,
    }


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# 制約付きrobust Lap オフライン再評価", "",
        f"- DB数: {report['database_count']}",
        f"- 完了個体数: {report['candidate_count']}",
        f"- 実行可能個体数: {report['feasible_count']}",
        f"- 実行可能率: {report['feasible_ratio']:.2%}",
        f"- 旧Fitnessとrobust LapのSpearman相関: {report['fitness_robust_lap_spearman']}",
        "", "## 制約違反内訳", "",
    ]
    lines.extend(f"- {name}: {count}" for name, count in report["failure_counts"].items())
    lines += ["", "## robust Lap 上位30", "",
              "|順位|Run|個体|robust Lap|旧Fitness|旧順位|p95|最大|repeat|",
              "|---:|---|---|---:|---:|---:|---:|---:|---:|"]
    for item in report["top_robust_lap"]:
        lines.append(f"|{item['rank']}|{item['run_id']}|{item['candidate_id']}|"
                     f"{item['robust_lap']:.3f}|{item['fitness']:.3f}|{item['fitness_rank']}|"
                     f"{item['lateral_error_p95_m']:.3f}|{item['lateral_error_max_m']:.3f}|"
                     f"{item['repeat_count']}|")
    lines += ["", "## 閾値感度", "",
              "|p95上限|最大上限|実行可能数|実行可能率|",
              "|---:|---:|---:|---:|"]
    for item in report["sensitivity"]:
        lines.append(f"|{item['lateral_error_p95_limit_m']:.2f}|"
                     f"{item['lateral_error_max_limit_m']:.2f}|{item['feasible_count']}|"
                     f"{item['feasible_ratio']:.2%}|")
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs-root", type=Path, default=ROOT / "runs")
    parser.add_argument("--run-dir", action="append", type=Path, default=[])
    parser.add_argument("--config", type=Path,
                        default=ROOT / "config" / "experiment_shared_ghost4_ros2.yaml")
    parser.add_argument("--output-prefix", type=Path,
                        default=ROOT / "analysis" / "constrained_objective")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    settings = dict(DEFAULT_SETTINGS, **config.get("constrained_objective", {}))
    databases = ([directory / "run.sqlite3" for directory in args.run_dir]
                 if args.run_dir else sorted(args.runs_root.glob("*/run.sqlite3")))
    databases = [path for path in databases if path.exists()]
    report = analyze(databases, settings)
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    args.output_prefix.with_suffix(".json").write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    args.output_prefix.with_suffix(".md").write_text(render_markdown(report), encoding="utf-8")
    print(json.dumps({key: report[key] for key in (
        "database_count", "candidate_count", "feasible_count", "feasible_ratio",
        "failure_counts", "fitness_robust_lap_spearman")}, indent=2))


if __name__ == "__main__":
    main()
