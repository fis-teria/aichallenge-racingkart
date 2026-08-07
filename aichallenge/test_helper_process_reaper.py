from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path

import pytest


REAPER = Path(__file__).with_name("helper_process_reaper.py")


def test_reaper_kills_and_waits_adopted_term_ignoring_process_group(
    tmp_path: Path,
) -> None:
    pid_path = tmp_path / "orphan.pid"
    command = (
        "setsid bash -c 'trap \"\" TERM; echo $$ >\"$1\"; "
        "while true; do sleep 1; done' _ \"$1\" & "
        "while [ ! -s \"$1\" ]; do sleep 0.01; done"
    )
    result = subprocess.run(
        [
            "python3",
            str(REAPER),
            "--grace-sec",
            "0.2",
            "--",
            "bash",
            "-c",
            command,
            "_",
            str(pid_path),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr
    orphan_pid = int(pid_path.read_text(encoding="utf-8"))
    with pytest.raises(ProcessLookupError):
        os.kill(orphan_pid, 0)


def test_reaper_starts_three_second_shutdown_before_absolute_deadline_and_reaps_same_and_escaped_groups(
    tmp_path: Path,
) -> None:
    same_group_pid_path = tmp_path / "same-group.pid"
    escaped_group_pid_path = tmp_path / "escaped-group.pid"
    deadline_monotonic_ns = time.monotonic_ns() + 4_000_000_000
    command = (
        'trap "" TERM; '
        '(trap "" TERM; echo "$BASHPID" >"$1"; while true; do sleep 1; done) & '
        'setsid bash -c \'trap "" TERM; echo "$BASHPID" >"$1"; '
        "while true; do sleep 1; done' _ \"$2\" & "
        'while [ ! -s "$1" ] || [ ! -s "$2" ]; do sleep 0.01; done; '
        "while true; do sleep 1; done"
    )
    started = time.monotonic()
    result = subprocess.run(
        [
            "python3",
            str(REAPER),
            "--grace-sec",
            "2",
            "--deadline-monotonic-ns",
            str(deadline_monotonic_ns),
            "--",
            "bash",
            "-c",
            command,
            "_",
            str(same_group_pid_path),
            str(escaped_group_pid_path),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    elapsed = time.monotonic() - started

    assert result.returncode == 124, result.stderr
    assert 2.5 < elapsed < 4.1
    assert time.monotonic_ns() < deadline_monotonic_ns
    for pid_path in (same_group_pid_path, escaped_group_pid_path):
        pid = int(pid_path.read_text(encoding="utf-8"))
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
