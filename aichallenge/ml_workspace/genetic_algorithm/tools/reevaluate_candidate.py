#!/usr/bin/env python3
"""Re-evaluate one stored GA candidate against the active single vehicle."""

import argparse
import json
import os
import signal
import sqlite3
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from ga_pure_pursuit.config import load_config
from ga_pure_pursuit.evaluator import make_evaluator


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-run", required=True, type=Path)
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--output-run", required=True, type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--vehicle-count", type=int, default=1)
    parser.add_argument("--acceleration-limit", type=float)
    parser.add_argument("--timing-probe")
    args = parser.parse_args()

    args.output_run.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(args.source_run / "run.sqlite3") as connection:
        row = connection.execute(
            "select genome_json, parameter_hash, fitness from candidates where candidate_id = ?",
            (args.candidate_id,),
        ).fetchone()
    if row is None:
        raise SystemExit(f"candidate not found: {args.candidate_id}")
    genome_json, parameter_hash, source_fitness = row
    genome = json.loads(genome_json)

    config = load_config(args.source_run / "experiment_resolved.json")
    evaluator_config = config["evaluator"]
    if args.vehicle_count == 1:
        evaluator_config["mode"] = "ros2"
        evaluator_config["timeout_sec"] = evaluator_config.get("episode_timeout_sec", 210.0)
    else:
        evaluator_config["mode"] = "shared_awsim_batch"
        evaluator_config["vehicle_domain_ids"] = list(range(1, args.vehicle_count + 1))
    if args.acceleration_limit is not None:
        evaluator_config.setdefault("fixed_controller_parameters", {})[
            "longitudinal_acceleration_limit"
        ] = args.acceleration_limit
    evaluator = make_evaluator(config, args.output_run)

    probe = None
    if args.timing_probe:
        probe = subprocess.Popen(
            [
                sys.executable,
                str(args.source_run.parent / "control_timing_probe.py"),
                "--output",
                args.timing_probe,
            ],
            env={**os.environ, "ROS_DOMAIN_ID": "1"},
        )

    try:
        results = []
        for repeat in range(args.repeats):
            if args.vehicle_count == 1:
                batch = [evaluator.evaluate(args.candidate_id, parameter_hash, genome, repeat)]
            else:
                def evaluate_vehicle(index: int):
                    return evaluator.evaluate(
                        f"{args.candidate_id}-d{index + 1}", parameter_hash, genome, repeat
                    )

                with ThreadPoolExecutor(max_workers=args.vehicle_count) as executor:
                    batch = list(executor.map(evaluate_vehicle, range(args.vehicle_count)))
            results.extend(batch)
    finally:
        if probe is not None:
            probe.send_signal(signal.SIGINT)
            probe.wait(timeout=10)

    output = {
        "source_run": str(args.source_run),
        "candidate_id": args.candidate_id,
        "parameter_hash": parameter_hash,
        "source_fitness": source_fitness,
        "vehicle_count": args.vehicle_count,
        "acceleration_limit": args.acceleration_limit,
        "repeats": results,
    }
    (args.output_run / "summary.json").write_text(
        json.dumps(output, indent=2, sort_keys=True), encoding="utf-8"
    )
    print(json.dumps(output, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
