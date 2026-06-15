from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


DEFAULT_MPC_PACKAGE_PATH = "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros"
DEFAULT_MPC_CONFIG_PATH = f"{DEFAULT_MPC_PACKAGE_PATH}/config/config.yaml"


@dataclass(frozen=True)
class ReferenceTrajectory:
    points: list[tuple[float, float]]
    source: str
    csv_path: Path | None = None
    config_path: Path | None = None
    circular: bool = False
    warnings: list[str] = field(default_factory=list)


def load_reference_trajectory(repo_root: Path, settings: dict[str, Any] | None) -> ReferenceTrajectory | None:
    config = settings or {}
    if not bool(config.get("enabled", True)):
        return None

    source = str(config.get("source", "mpc_config"))
    if source != "mpc_config":
        return ReferenceTrajectory(points=[], source=source, warnings=[f"unsupported reference trajectory source: {source}"])

    warnings: list[str] = []
    package_root = _resolve_path(repo_root, str(config.get("mpc_package_path", DEFAULT_MPC_PACKAGE_PATH)), repo_root)
    config_path = _resolve_path(repo_root, str(config.get("mpc_config_path", DEFAULT_MPC_CONFIG_PATH)), repo_root)
    mpc_config = _load_yaml(config_path, warnings)

    reference_path = mpc_config.get("reference_path", {}) if isinstance(mpc_config, dict) else {}
    if not isinstance(reference_path, dict):
        reference_path = {}

    csv_setting = str(config.get("csv_path") or reference_path.get("csv_path") or "")
    if not csv_setting:
        return ReferenceTrajectory(
            points=[],
            source="mpc_csv",
            config_path=config_path,
            warnings=[*warnings, "MPC reference_path.csv_path is empty"],
        )

    circular = bool(reference_path.get("circular", config.get("circular", False)))
    if bool(reference_path.get("update_by_topic", False)):
        warnings.append("MPC reference_path.update_by_topic is true; CSV fallback may differ from runtime trajectory")

    csv_path = _resolve_path(repo_root, csv_setting, package_root)
    points = _read_csv_points(csv_path, warnings)
    points = _normalize_points(points, circular=circular)
    if len(points) < 3:
        warnings.append(f"reference trajectory has too few usable points: {csv_path}")

    return ReferenceTrajectory(
        points=points,
        source="mpc_csv",
        csv_path=csv_path,
        config_path=config_path,
        circular=circular,
        warnings=warnings,
    )


def _load_yaml(path: Path, warnings: list[str]) -> dict[str, Any]:
    if not path.exists():
        warnings.append(f"MPC config not found: {path}")
        return {}
    try:
        with path.open("r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}
    except Exception as exc:  # noqa: BLE001 - config loading should not stop report generation
        warnings.append(f"failed to load MPC config {path}: {exc}")
        return {}
    if not isinstance(loaded, dict):
        warnings.append(f"MPC config must contain a mapping: {path}")
        return {}
    return loaded


def _read_csv_points(path: Path, warnings: list[str]) -> list[tuple[float, float]]:
    if not path.exists():
        warnings.append(f"reference trajectory CSV not found: {path}")
        return []

    points: list[tuple[float, float]] = []
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            for row in csv.DictReader(handle):
                x = _first_float(row, ("x_m", "x"))
                y = _first_float(row, ("y_m", "y"))
                if x is None or y is None:
                    continue
                points.append((x, y))
    except Exception as exc:  # noqa: BLE001
        warnings.append(f"failed to read reference trajectory CSV {path}: {exc}")
        return []

    if not points:
        warnings.append(f"reference trajectory CSV has no x/y points: {path}")
    return points


def _normalize_points(points: list[tuple[float, float]], *, circular: bool) -> list[tuple[float, float]]:
    normalized: list[tuple[float, float]] = []
    for point in points:
        if not normalized or _distance(normalized[-1], point) > 1e-6:
            normalized.append(point)
    if circular and len(normalized) >= 2 and _distance(normalized[0], normalized[-1]) <= 1e-3:
        normalized.pop()
    return normalized


def _resolve_path(repo_root: Path, value: str, relative_root: Path) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    candidate = relative_root / path
    if candidate.exists():
        return candidate
    return repo_root / path


def _first_float(row: dict[str, str | None], keys: tuple[str, ...]) -> float | None:
    for key in keys:
        value = row.get(key)
        if value is None or value == "":
            continue
        try:
            parsed = float(value)
        except ValueError:
            continue
        if math.isfinite(parsed):
            return parsed
    return None


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
