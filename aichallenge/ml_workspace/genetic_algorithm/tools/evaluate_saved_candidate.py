#!/usr/bin/env python3
"""Evaluate one saved GA genome in an already running ROS/AWSIM environment."""

from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path

from ga_pure_pursuit.config import load_config
from ga_pure_pursuit.evaluator import make_evaluator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-run", required=True, type=Path)
    parser.add_argument("--output-run", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--candidate-id")
    parser.add_argument("--label", required=True)
    parser.add_argument("--domain", type=int, default=1)
    return parser.parse_args()


def load_candidate(source_run: Path, candidate_id: str | None):
    connection = sqlite3.connect(source_run / "run.sqlite3")
    try:
        if candidate_id:
            row = connection.execute(
                "SELECT candidate_id, genome_json, parameter_hash, fitness "
                "FROM candidates WHERE candidate_id=? AND status='complete'",
                (candidate_id,),
            ).fetchone()
        else:
            row = connection.execute(
                "SELECT candidate_id, genome_json, parameter_hash, fitness "
                "FROM candidates WHERE status='complete' AND fitness IS NOT NULL "
                "ORDER BY fitness ASC LIMIT 1"
            ).fetchone()
    finally:
        connection.close()
    if row is None:
        raise SystemExit("completed candidate not found")
    return row[0], json.loads(row[1]), row[2], float(row[3])


def main() -> None:
    args = parse_args()
    source_id, genome, parameter_hash, source_fitness = load_candidate(
        args.source_run, args.candidate_id
    )
    args.output_run.mkdir(parents=True, exist_ok=True)
    config = load_config(args.config)
    config["evaluator"]["mode"] = "ros2"
    config["evaluator"]["ros"]["vehicle_domain_id"] = args.domain
    evaluator = make_evaluator(config, args.output_run)
    evaluation_id = f"{args.label}-{source_id}"
    metrics = evaluator.evaluate(evaluation_id, parameter_hash, genome, 0)
    result = {
        "condition": args.label,
        "source_candidate_id": source_id,
        "source_fitness": source_fitness,
        "parameter_hash": parameter_hash,
        "genome": genome,
        "metrics": metrics,
    }
    output = args.output_run / "summary.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
