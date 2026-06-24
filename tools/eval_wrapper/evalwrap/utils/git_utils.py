from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path

from .hash_utils import sha256_text


@dataclass(frozen=True)
class GitInfo:
    branch: str | None
    commit: str | None
    dirty: bool
    diff_hash: str
    diff_patch: str


def _git(repo_root: Path, args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=repo_root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def collect_git_info(repo_root: Path) -> GitInfo:
    branch_result = _git(repo_root, ["branch", "--show-current"])
    commit_result = _git(repo_root, ["rev-parse", "HEAD"])
    status_result = _git(repo_root, ["status", "--porcelain"])
    diff_result = _git(repo_root, ["diff", "--binary", "HEAD", "--"])

    diff_patch = diff_result.stdout if diff_result.returncode == 0 else ""
    branch = branch_result.stdout.strip() or None
    commit = commit_result.stdout.strip() if commit_result.returncode == 0 else None
    dirty = bool(status_result.stdout.strip()) if status_result.returncode == 0 else True
    return GitInfo(
        branch=branch,
        commit=commit,
        dirty=dirty,
        diff_hash=sha256_text(diff_patch),
        diff_patch=diff_patch,
    )
