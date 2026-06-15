from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .utils.subprocess_utils import run_command


@dataclass
class CommandResult:
    command: list[str]
    log_path: Path
    returncode: int


def run_eval_commands(repo_root: Path, command_log_dir: Path, skip_build: bool = False) -> list[CommandResult]:
    command_log_dir.mkdir(parents=True, exist_ok=True)
    commands: list[list[str]] = []
    if not skip_build:
        commands.extend(
            [
                ["./create_submit_file.bash"],
                ["./docker_build.sh", "eval"],
            ]
        )
    commands.append(["make", "eval"])
    results: list[CommandResult] = []
    for index, cmd in enumerate(commands, start=1):
        name = cmd[0].replace("./", "").replace("/", "_")
        log_path = command_log_dir / f"{index:02d}_{name}.log"
        returncode = run_command(cmd, cwd=repo_root, log_path=log_path)
        results.append(CommandResult(command=cmd, log_path=log_path, returncode=returncode))
        if returncode != 0:
            break
    return results


def run_parallel_commands(repo_root: Path, command_log_dir: Path, submits: list[Path]) -> list[CommandResult]:
    command_log_dir.mkdir(parents=True, exist_ok=True)
    cmd = ["./run_parallel_submissions.bash", "--submit", *[str(path) for path in submits]]
    log_path = command_log_dir / "01_run_parallel_submissions.log"
    returncode = run_command(cmd, cwd=repo_root, log_path=log_path)
    return [CommandResult(command=cmd, log_path=log_path, returncode=returncode)]
