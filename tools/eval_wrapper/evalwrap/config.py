from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from .utils.fs_utils import find_repo_root


@dataclass(frozen=True)
class EvalConfig:
    repo_root: Path
    analysis_dir: Path
    output_latest: Path
    submission_tar: Path
    domains: list[int] = field(default_factory=lambda: [1, 2, 3, 4])
    command_log_dir: str = "command_logs"
    thresholds: dict[str, float] = field(default_factory=dict)
    reference_trajectory: dict[str, Any] = field(default_factory=dict)
    overtake_analysis: dict[str, Any] = field(default_factory=dict)


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Config file must contain a mapping: {path}")
    return loaded


def load_config(repo_root: Path | None = None, config_path: Path | None = None) -> EvalConfig:
    root = find_repo_root(repo_root or Path.cwd())
    default_path = package_root() / "configs" / "default.yaml"
    thresholds_path = package_root() / "configs" / "thresholds.yaml"
    overtake_path = package_root() / "configs" / "overtake_analysis.yaml"
    data = _load_yaml(default_path)
    if config_path:
        data.update(_load_yaml(config_path))
    thresholds = _load_yaml(thresholds_path)
    overtake_analysis = _load_yaml(overtake_path)

    analysis_dir = root / str(data.get("analysis_dir", "analysis"))
    output_latest = root / str(data.get("output_latest", "output/latest"))
    submission_tar = root / str(data.get("submission_tar", "submit/aichallenge_submit.tar.gz"))
    domains = [int(d) for d in data.get("domains", [1, 2, 3, 4])]

    return EvalConfig(
        repo_root=root,
        analysis_dir=analysis_dir,
        output_latest=output_latest,
        submission_tar=submission_tar,
        domains=domains,
        command_log_dir=str(data.get("command_log_dir", "command_logs")),
        thresholds={str(k): float(v) for k, v in thresholds.items()},
        reference_trajectory=dict(data.get("reference_trajectory", {}) or {}),
        overtake_analysis=dict(data.get("overtake_analysis", overtake_analysis) or {}),
    )
