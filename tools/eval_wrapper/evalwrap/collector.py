from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .utils.fs_utils import copytree_replace, ensure_dir


@dataclass
class CollectionResult:
    domains: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def _domain_name(domain: int | str) -> str:
    text = str(domain)
    return text if text.startswith("d") else f"d{text}"


def _latest_domain_output(output_root: Path, domains: list[int]) -> Path | None:
    if not output_root.exists() or not output_root.is_dir():
        return None
    candidates = []
    for child in output_root.iterdir():
        if not child.is_dir() or child.name in {"latest", "docker"}:
            continue
        if any((child / _domain_name(domain)).is_dir() for domain in domains):
            candidates.append(child)
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _resolve_collection_source(source: Path, domains: list[int]) -> Path:
    resolved = source.resolve()
    if source.name == "latest":
        latest_run = _latest_domain_output(source.parent.resolve(), domains)
        if latest_run is not None:
            return latest_run
    if resolved.exists() and resolved.name == "output":
        latest_run = _latest_domain_output(resolved, domains)
        if latest_run is not None:
            return latest_run
    return resolved


def collect_output(source: Path, run_dir: Path, domains: list[int]) -> CollectionResult:
    result = CollectionResult()
    raw_dir = ensure_dir(run_dir / "raw")
    resolved = _resolve_collection_source(source, domains)

    if not resolved.exists():
        result.warnings.append(f"output path not found: {source}")
        return result

    if resolved.is_dir() and resolved.name.startswith("d") and resolved.name[1:].isdigit():
        dst = raw_dir / resolved.name
        copytree_replace(resolved, dst)
        result.domains.append(resolved.name)
        return result

    for domain in domains:
        name = _domain_name(domain)
        src = resolved / name
        if not src.exists():
            continue
        if not src.is_dir():
            result.warnings.append(f"domain output is not a directory: {src}")
            continue
        copytree_replace(src, raw_dir / name)
        result.domains.append(name)

    if not result.domains:
        result.warnings.append(f"no d1-d4 directories found under {resolved}")
    return result
