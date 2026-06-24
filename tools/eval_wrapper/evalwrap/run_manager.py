from __future__ import annotations

import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from .collector import CollectionResult, collect_output
from .config import EvalConfig
from .executor import CommandResult, run_eval_commands, run_parallel_commands
from .metrics.race_metrics import DomainResult, build_domain_result, write_processed_outputs
from .parsers.details_parser import parse_details
from .parsers.log_parser import parse_logs
from .parsers.summary_parser import parse_summary
from .reference_trajectory import ReferenceTrajectory, load_reference_trajectory
from .reports.html_report import generate_run_report
from .store import save_run
from .utils.fs_utils import ensure_dir, slugify
from .utils.git_utils import collect_git_info
from .utils.hash_utils import sha256_file


@dataclass
class PipelineResult:
    run_id: str
    run_dir: Path
    manifest_path: Path
    report_path: Path
    metrics_path: Path
    status: str


def make_run_id(label: str, now: datetime | None = None) -> str:
    timestamp = (now or datetime.now().astimezone()).strftime("%Y%m%d_%H%M%S")
    return f"{timestamp}_{slugify(label)}"


def run_pipeline(
    config: EvalConfig,
    label: str,
    note: str = "",
    mode: str = "ingest",
    output_path: Path | None = None,
    skip_build: bool = False,
    no_eval: bool = False,
    parallel_submits: list[Path] | None = None,
) -> PipelineResult:
    started = datetime.now().astimezone()
    run_id = make_run_id(label, started)
    run_dir = config.analysis_dir / "runs" / run_id
    processed_dir = ensure_dir(run_dir / "processed")
    command_log_dir = ensure_dir(run_dir / config.command_log_dir)

    git_info = collect_git_info(config.repo_root)
    diff_patch_path = run_dir / "diff.patch"
    diff_patch_path.parent.mkdir(parents=True, exist_ok=True)
    diff_patch_path.write_text(git_info.diff_patch, encoding="utf-8")

    command_results: list[CommandResult] = []
    command_warnings: list[str] = []
    eval_output_source: Path | None = None
    if mode == "single" and not no_eval:
        command_results = run_eval_commands(config.repo_root, command_log_dir, skip_build=skip_build)
        command_warnings.extend(
            f"command failed ({result.returncode}): {' '.join(result.command)}"
            for result in command_results
            if result.returncode != 0
        )
        if command_results and all(result.returncode == 0 for result in command_results):
            eval_output_source, wait_warnings = _wait_for_single_eval_output(config.repo_root, config.domains, started)
            command_warnings.extend(wait_warnings)
    elif mode == "parallel" and not no_eval:
        command_results = run_parallel_commands(config.repo_root, command_log_dir, parallel_submits or [])
        command_warnings.extend(
            f"command failed ({result.returncode}): {' '.join(result.command)}"
            for result in command_results
            if result.returncode != 0
        )

    reference_trajectory = load_reference_trajectory(config.repo_root, config.reference_trajectory)
    use_reference_fallback = bool(config.reference_trajectory.get("use_when_rosbag_trajectory_missing", True))

    source = output_path or eval_output_source or (_latest_parallel_output(config.repo_root) if mode == "parallel" else config.output_latest)
    collection = CollectionResult() if no_eval else collect_output(source, run_dir, config.domains)
    domain_results = _parse_domains(
        run_id,
        run_dir,
        collection.domains,
        config.thresholds,
        reference_trajectory if use_reference_fallback else None,
    )
    metrics = write_processed_outputs(run_id, domain_results, processed_dir)

    finished = datetime.now().astimezone()
    warnings = [*collection.warnings, *command_warnings]
    if reference_trajectory is not None:
        warnings.extend(reference_trajectory.warnings)
    for domain in domain_results:
        warnings.extend(domain.metrics.warnings or [])
    status = _status(no_eval=no_eval, domains=collection.domains, warnings=warnings, command_results=command_results)

    manifest = _manifest(
        run_id=run_id,
        label=label,
        note=note,
        status=status,
        config=config,
        git_info=git_info,
        mode=mode,
        command_results=command_results,
        domains=collection.domains,
        started=started,
        finished=finished,
        warnings=warnings,
        reference_trajectory=reference_trajectory,
    )
    manifest_path = run_dir / "manifest.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8")
    report_path = generate_run_report(run_dir, manifest, metrics)
    save_run(config.analysis_dir / "experiments.sqlite", manifest, metrics, report_path)

    return PipelineResult(
        run_id=run_id,
        run_dir=run_dir,
        manifest_path=manifest_path,
        report_path=report_path,
        metrics_path=processed_dir / "metrics.json",
        status=status,
    )


def _parse_domains(
    run_id: str,
    run_dir: Path,
    domains: list[str],
    thresholds: dict[str, float] | None = None,
    reference_trajectory: ReferenceTrajectory | None = None,
) -> list[DomainResult]:
    results: list[DomainResult] = []
    for domain_id in domains:
        raw_domain = run_dir / "raw" / domain_id
        summary = parse_summary(raw_domain / "result-summary.json")
        details = parse_details(raw_domain / f"{domain_id}-result-details.json")
        logs = parse_logs(raw_domain)
        results.append(
            build_domain_result(
                run_id,
                domain_id,
                summary,
                details,
                logs,
                raw_domain,
                thresholds,
                reference_trajectory,
            )
        )
    return results


def _status(no_eval: bool, domains: list[str], warnings: list[str], command_results: list[CommandResult]) -> str:
    if command_results and command_results[-1].returncode != 0 and not domains:
        return "failed"
    if no_eval or warnings or not domains or any(result.returncode != 0 for result in command_results):
        return "partial"
    return "success"


def _manifest(
    run_id: str,
    label: str,
    note: str,
    status: str,
    config: EvalConfig,
    git_info,
    mode: str,
    command_results: list[CommandResult],
    domains: list[str],
    started: datetime,
    finished: datetime,
    warnings: list[str],
    reference_trajectory: ReferenceTrajectory | None,
) -> dict[str, Any]:
    tar_sha = sha256_file(config.submission_tar)
    return {
        "run_id": run_id,
        "label": label,
        "created_at": started.isoformat(),
        "status": status,
        "repo": {
            "root": str(config.repo_root),
            "branch": git_info.branch,
            "commit": git_info.commit,
            "dirty": git_info.dirty,
            "diff_hash": git_info.diff_hash,
            "diff_patch": "diff.patch",
        },
        "submission": {
            "tar_path": str(config.submission_tar.relative_to(config.repo_root))
            if config.submission_tar.exists()
            else str(config.submission_tar),
            "tar_sha256": tar_sha,
        },
        "eval": {
            "mode": mode,
            "command": [" ".join(result.command) for result in command_results],
            "command_results": [
                {
                    "command": result.command,
                    "returncode": result.returncode,
                    "log_path": str(result.log_path),
                }
                for result in command_results
            ],
            "domains": domains,
            "started_at": started.isoformat(),
            "finished_at": finished.isoformat(),
            "duration_sec": (finished - started).total_seconds(),
        },
        "reference_trajectory": _reference_trajectory_manifest(reference_trajectory),
        "notes": {
            "hypothesis": label,
            "change_summary": note,
        },
        "warnings": warnings,
    }


def _reference_trajectory_manifest(reference_trajectory: ReferenceTrajectory | None) -> dict[str, Any]:
    if reference_trajectory is None:
        return {"enabled": False}
    return {
        "enabled": True,
        "source": reference_trajectory.source,
        "csv_path": str(reference_trajectory.csv_path) if reference_trajectory.csv_path else None,
        "config_path": str(reference_trajectory.config_path) if reference_trajectory.config_path else None,
        "circular": reference_trajectory.circular,
        "point_count": len(reference_trajectory.points),
        "warnings": reference_trajectory.warnings,
    }


def _latest_parallel_output(repo_root: Path) -> Path:
    output_dir = repo_root / "output"
    if not output_dir.exists():
        return repo_root / "output" / "latest"
    candidates = [
        path
        for path in output_dir.iterdir()
        if path.is_dir() and any((path / f"d{domain}").is_dir() for domain in range(1, 5))
    ]
    if not candidates:
        return output_dir / "latest"
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _wait_for_single_eval_output(
    repo_root: Path,
    domains: list[int],
    started: datetime,
    timeout_sec: float = 900.0,
    poll_sec: float = 2.0,
) -> tuple[Path | None, list[str]]:
    output_dir = repo_root / "output"
    deadline = time.monotonic() + timeout_sec
    started_ts = started.timestamp() - 5.0
    latest_candidate: Path | None = None
    latest_reason = "no output run directory found"

    while True:
        latest_candidate, latest_reason = _latest_single_eval_candidate(output_dir, domains, started_ts)
        if latest_candidate is not None and _output_run_ready(latest_candidate, domains)[0]:
            return latest_candidate, []
        if time.monotonic() >= deadline:
            detail = f": {latest_reason}" if latest_reason else ""
            return latest_candidate, [f"timed out waiting for eval output to finish{detail}"]
        time.sleep(poll_sec)


def _latest_single_eval_candidate(output_dir: Path, domains: list[int], min_mtime: float) -> tuple[Path | None, str]:
    if not output_dir.exists():
        return None, f"output directory not found: {output_dir}"

    candidates: list[Path] = []
    for child in output_dir.iterdir():
        if not child.is_dir() or child.name in {"latest", "docker"}:
            continue
        if child.stat().st_mtime < min_mtime:
            continue
        if any((child / f"d{domain}").is_dir() for domain in domains):
            candidates.append(child)

    if not candidates:
        return None, "no new d1-d4 output directory found"

    latest = max(candidates, key=lambda path: path.stat().st_mtime)
    ready, reason = _output_run_ready(latest, domains)
    return latest, reason if not ready else ""


def _output_run_ready(output_run: Path, domains: list[int]) -> tuple[bool, str]:
    reasons = []
    saw_domain = False
    for domain in domains:
        name = f"d{domain}"
        domain_dir = output_run / name
        if not domain_dir.is_dir():
            continue
        saw_domain = True
        summary = domain_dir / "result-summary.json"
        details = domain_dir / f"{name}-result-details.json"
        if not summary.exists() or not details.exists():
            reasons.append(f"{name}: result json not ready")
            continue
        rosbag_ready, rosbag_reason = _rosbag_storage_ready(domain_dir)
        if not rosbag_ready:
            reasons.append(f"{name}: {rosbag_reason}")
            continue
        return True, ""

    if not saw_domain:
        return False, "no d1-d4 domain directory found"
    return False, "; ".join(reasons) if reasons else "no complete domain output found"


def _rosbag_storage_ready(domain_dir: Path) -> tuple[bool, str]:
    for bag_dir_name in ("rosbag2_autoware",):
        bag_dir = domain_dir / bag_dir_name
        if bag_dir.is_dir():
            has_storage = any(path.suffix in {".mcap", ".db3"} for path in bag_dir.iterdir())
            if has_storage and not (bag_dir / "metadata.yaml").exists():
                return False, f"{bag_dir_name} metadata.yaml not ready"

    return True, ""
