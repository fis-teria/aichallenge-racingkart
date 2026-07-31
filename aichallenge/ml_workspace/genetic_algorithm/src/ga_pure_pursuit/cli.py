from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import load_config
from .export_best import export_best
from .optimizer import Optimizer


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(prog="ga-pure-pursuit")
    commands = result.add_subparsers(dest="command", required=True)
    search = commands.add_parser("search", help="run a genetic search")
    search.add_argument("--config", required=True, type=Path)
    search.add_argument("--runs-dir", type=Path, default=Path("runs"))
    search.add_argument("--run-id")
    resume = commands.add_parser("resume", help="resume from the latest checkpoint")
    resume.add_argument("--run-dir", required=True, type=Path)
    resume.add_argument(
        "--parallel-workers",
        type=int,
        help="override only the execution concurrency recorded in the run",
    )
    resume.add_argument(
        "--worker-ids",
        help="comma-separated worker pool override used with --parallel-workers",
    )
    export = commands.add_parser("export", help="export the best candidate as ROS YAML")
    export.add_argument("--run-dir", required=True, type=Path)
    export.add_argument("--output", required=True, type=Path)
    return result


def main() -> None:
    args = parser().parse_args()
    if args.command == "search":
        best = Optimizer(
            load_config(args.config), args.runs_dir, run_id=args.run_id
        ).run()
        print(json.dumps(best, indent=2, sort_keys=True))
    elif args.command == "resume":
        config = load_config(args.run_dir / "experiment_resolved.json")
        if args.parallel_workers is not None:
            if args.parallel_workers < 1:
                raise SystemExit("--parallel-workers must be at least 1")
            config["run"]["parallel_workers"] = args.parallel_workers
        if args.worker_ids:
            worker_ids = [value.strip() for value in args.worker_ids.split(",") if value.strip()]
            if not worker_ids:
                raise SystemExit("--worker-ids must contain at least one worker")
            config["evaluator"]["worker_ids"] = worker_ids
        if (
            config["evaluator"].get("mode") == "worker_pool"
            and int(config["run"].get("parallel_workers", 1))
            > len(config["evaluator"].get("worker_ids", []))
        ):
            raise SystemExit("parallel worker count exceeds the configured worker IDs")
        best = Optimizer(
            config,
            args.run_dir.parent,
            run_id=args.run_dir.name,
            resume=True,
        ).run()
        print(json.dumps(best, indent=2, sort_keys=True))
    else:
        print(json.dumps(export_best(args.run_dir, args.output), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
