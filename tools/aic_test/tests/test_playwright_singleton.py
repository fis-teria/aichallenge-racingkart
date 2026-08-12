from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import pytest


MODULE = "aic_test.playwright_singleton"


def _environment() -> dict[str, str]:
    environment = dict(os.environ)
    tools_root = Path(__file__).resolve().parents[1]
    environment["PYTHONPATH"] = str(tools_root)
    return environment


def _owner(tmp_path: Path, lifetime_s: float = 30.0) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [
            sys.executable,
            "-m",
            MODULE,
            "--lock",
            str(tmp_path / "transport.lock"),
            "--",
            sys.executable,
            "-c",
            f"import time; time.sleep({lifetime_s})",
        ],
        env=_environment(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def test_singleton_lock_is_exclusive_and_fail_fast(tmp_path: Path) -> None:
    owner = _owner(tmp_path)
    try:
        time.sleep(0.2)
        contender = subprocess.run(
            [
                sys.executable,
                "-m",
                MODULE,
                "--lock",
                str(tmp_path / "transport.lock"),
                "--",
                sys.executable,
                "-c",
                "raise SystemExit(99)",
            ],
            env=_environment(),
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
        assert contender.returncode == 75
        assert '"status": "LOCK_BUSY"' in contender.stderr
    finally:
        owner.terminate()
        owner.communicate(timeout=5)

    successor = subprocess.run(
        [
            sys.executable,
            "-m",
            MODULE,
            "--lock",
            str(tmp_path / "transport.lock"),
            "--",
            sys.executable,
            "-c",
            "raise SystemExit(0)",
        ],
        env=_environment(),
        text=True,
        capture_output=True,
        timeout=5,
        check=False,
    )
    assert successor.returncode == 0


def test_stale_lock_file_without_owner_does_not_block(tmp_path: Path) -> None:
    lock_path = tmp_path / "transport.lock"
    lock_path.write_text("stale metadata is not authority", encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            MODULE,
            "--lock",
            str(lock_path),
            "--",
            sys.executable,
            "-c",
            "raise SystemExit(0)",
        ],
        env=_environment(),
        text=True,
        capture_output=True,
        timeout=5,
        check=False,
    )
    assert result.returncode == 0


def test_source_never_uses_broad_process_cleanup() -> None:
    source = (Path(__file__).resolve().parents[1] / "aic_test/playwright_singleton.py").read_text(
        encoding="utf-8"
    )
    assert "pkill" not in source
    assert "killall" not in source
    assert "os.killpg(child.pid" in source


def test_default_server_command_is_loopback_headed_and_persistent() -> None:
    from aic_test.playwright_singleton import (
        DEFAULT_OUTPUT,
        DEFAULT_PROFILE,
        _server_command,
    )

    command = _server_command(8931, DEFAULT_PROFILE, DEFAULT_OUTPUT)
    assert command[command.index("--host") + 1] == "127.0.0.1"
    assert command[command.index("--allowed-hosts") + 1] == "localhost:8931"
    assert command[command.index("--port") + 1] == "8931"
    assert command[command.index("--user-data-dir") + 1] == str(DEFAULT_PROFILE)
    assert "--shared-browser-context" in command
    assert "--headless" not in command


def test_sigterm_stops_only_owned_child_and_releases_lock(tmp_path: Path) -> None:
    child_pid_path = tmp_path / "child.pid"
    owner = subprocess.Popen(
        [
            sys.executable,
            "-m",
            MODULE,
            "--lock",
            str(tmp_path / "transport.lock"),
            "--",
            sys.executable,
            "-c",
            (
                "import os,signal,time; "
                f"open({str(child_pid_path)!r},'w').write(str(os.getpid())); "
                "signal.signal(signal.SIGTERM, lambda *_: raise_exit()); "
                "time.sleep(30)"
            ).replace("raise_exit()", "exit(0)"),
        ],
        env=_environment(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    deadline = time.monotonic() + 5
    while not child_pid_path.exists() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert child_pid_path.exists()
    child_pid = int(child_pid_path.read_text(encoding="utf-8"))
    owner.terminate()
    owner.communicate(timeout=5)
    with pytest.raises(ProcessLookupError):
        os.kill(child_pid, 0)
