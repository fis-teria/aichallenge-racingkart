from __future__ import annotations

import re
import shutil
from pathlib import Path


REPO_MARKERS = ("create_submit_file.bash", "docker_build.sh", "Makefile")


def find_repo_root(start: Path | None = None) -> Path:
    current = (start or Path.cwd()).resolve()
    candidates = [current, *current.parents]
    for path in candidates:
        if all((path / marker).exists() for marker in REPO_MARKERS):
            return path
        child = path / "aichallenge-racingkart"
        if child.is_dir() and all((child / marker).exists() for marker in REPO_MARKERS):
            return child.resolve()
    raise FileNotFoundError(
        f"Could not find aichallenge-racingkart repo root from {current}. "
        f"Expected markers: {', '.join(REPO_MARKERS)}"
    )


def slugify(value: str, default: str = "run") -> str:
    value = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    value = re.sub(r"_+", "_", value).strip("._-")
    return value or default


def copytree_replace(src: Path, dst: Path) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst, symlinks=False)


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def relative_to_or_name(path: Path, start: Path) -> str:
    try:
        return str(path.relative_to(start))
    except ValueError:
        return str(path)
