from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .config import load_config
from .reports.compare_report import generate_compare_report
from .run_manager import run_pipeline
from .store import leaderboard as load_leaderboard
from .store import list_runs
from .utils.fs_utils import find_repo_root


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not hasattr(args, "func"):
        parser.print_help()
        return 0
    return int(args.func(args) or 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="evalwrap")
    parser.add_argument("--repo-root", type=Path, default=None, help="aichallenge-racingkart repository root")
    parser.add_argument("--config", type=Path, default=None, help="config yaml path")
    sub = parser.add_subparsers(dest="command")

    run = sub.add_parser("run", help="execute local evaluation and collect artifacts")
    run.add_argument("--label", required=True)
    run.add_argument("--note", default="")
    run.add_argument("--skip-build", action="store_true")
    run.add_argument("--no-eval", action="store_true")
    run.set_defaults(func=cmd_run)

    parallel = sub.add_parser("run-parallel", help="execute multi-submission evaluation and collect artifacts")
    parallel.add_argument("--label", required=True)
    parallel.add_argument("--note", default="")
    parallel.add_argument("--submit", type=Path, action="append", required=True)
    parallel.add_argument("--path", type=Path, default=None, help="optional output run directory to collect")
    parallel.add_argument("--no-eval", action="store_true")
    parallel.set_defaults(func=cmd_run_parallel)

    ingest = sub.add_parser("ingest", help="collect an existing output/latest directory")
    ingest.add_argument("--label", required=True)
    ingest.add_argument("--note", default="")
    ingest.add_argument("--path", type=Path, required=True)
    ingest.set_defaults(func=cmd_ingest)

    compare = sub.add_parser("compare", help="compare two runs")
    compare.add_argument("--base", required=True)
    compare.add_argument("--target", required=True)
    compare.set_defaults(func=cmd_compare)

    sub_list = sub.add_parser("list", help="list collected runs")
    sub_list.set_defaults(func=cmd_list)

    board = sub.add_parser("leaderboard", help="rank runs by a metric")
    board.add_argument("--metric", default="total_time_sec")
    board.set_defaults(func=cmd_leaderboard)

    doctor = sub.add_parser("doctor", help="check repository and wrapper prerequisites")
    doctor.set_defaults(func=cmd_doctor)

    return parser


def _config(args) -> object:
    return load_config(args.repo_root, args.config)


def cmd_run(args) -> int:
    config = _config(args)
    result = run_pipeline(
        config,
        label=args.label,
        note=args.note,
        mode="single",
        skip_build=args.skip_build,
        no_eval=args.no_eval,
    )
    _print_result(result)
    return 0 if result.status in {"success", "partial"} else 1


def cmd_ingest(args) -> int:
    config = _config(args)
    result = run_pipeline(config, label=args.label, note=args.note, mode="ingest", output_path=args.path)
    _print_result(result)
    return 0 if result.status in {"success", "partial"} else 1


def cmd_run_parallel(args) -> int:
    config = _config(args)
    submits = [path if path.is_absolute() else config.repo_root / path for path in args.submit]
    if not 1 <= len(submits) <= 4:
        print("run-parallel requires 1 to 4 --submit values", file=sys.stderr)
        return 2
    missing = [path for path in submits if not path.exists()]
    if missing:
        print(f"missing submit file: {missing[0]}", file=sys.stderr)
        return 2
    output_path = None
    if args.path is not None:
        output_path = args.path if args.path.is_absolute() else config.repo_root / args.path
    result = run_pipeline(
        config,
        label=args.label,
        note=args.note,
        mode="parallel",
        output_path=output_path,
        no_eval=args.no_eval,
        parallel_submits=submits,
    )
    _print_result(result)
    return 0 if result.status in {"success", "partial"} else 1


def cmd_compare(args) -> int:
    config = _config(args)
    report = generate_compare_report(config.analysis_dir, args.base, args.target)
    print(f"compare report: {report}")
    return 0


def cmd_list(args) -> int:
    config = _config(args)
    rows = list_runs(config.analysis_dir / "experiments.sqlite")
    if not rows:
        print("no runs")
        return 0
    for row in rows:
        print(f"{row['run_id']}\t{row['status']}\t{row['label']}\t{row['judgement']}\t{row['report_path']}")
    return 0


def cmd_leaderboard(args) -> int:
    config = _config(args)
    try:
        rows = load_leaderboard(config.analysis_dir / "experiments.sqlite", args.metric)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if not rows:
        print("no leaderboard rows")
        return 0
    print(f"rank\tmetric\tvalue\trun_id\tdomain\tjudgement")
    for rank, row in enumerate(rows, start=1):
        print(
            f"{rank}\t{args.metric}\t{row['metric_value']}\t{row['run_id']}\t{row['domain_id']}\t{row['judgement']}"
        )
    return 0


def cmd_doctor(args) -> int:
    try:
        config = _config(args)
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL repo root: {exc}")
        return 1
    checks = {
        "repo_root": config.repo_root.exists(),
        "create_submit_file.bash": (config.repo_root / "create_submit_file.bash").exists(),
        "docker_build.sh": (config.repo_root / "docker_build.sh").exists(),
        "Makefile": (config.repo_root / "Makefile").exists(),
        "output_latest": config.output_latest.exists(),
        "analysis_dir_parent": config.analysis_dir.parent.exists(),
    }
    try:
        import yaml as _yaml  # noqa: F401

        checks["python_yaml"] = True
    except Exception:
        checks["python_yaml"] = False
    for name, ok in checks.items():
        print(f"{'OK' if ok else 'WARN'} {name}: {ok}")
    print(f"repo: {config.repo_root}")
    return 0 if all(ok for name, ok in checks.items() if name not in {"output_latest"}) else 1


def _print_result(result) -> None:
    print(f"run_id: {result.run_id}")
    print(f"status: {result.status}")
    print(f"manifest: {result.manifest_path}")
    print(f"metrics: {result.metrics_path}")
    print(f"report: {result.report_path}")


if __name__ == "__main__":
    raise SystemExit(main())
