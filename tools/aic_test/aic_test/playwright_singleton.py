from __future__ import annotations

import argparse
import fcntl
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Sequence


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8931
DEFAULT_PROFILE = Path("/home/graneple/.cache/ms-playwright/chatgpt-external-review")
DEFAULT_OUTPUT = Path(
    "/home/graneple/.cache/ms-playwright/chatgpt-external-review-output"
)
DEFAULT_LOCK = Path("/tmp/codex-playwright-external-review.lock")


def _server_command(port: int, profile: Path, output: Path) -> list[str]:
    return [
        "npx",
        "-y",
        "@playwright/mcp@0.0.75",
        "--host",
        DEFAULT_HOST,
        "--allowed-hosts",
        f"localhost:{port}",
        "--port",
        str(port),
        "--browser",
        "chrome",
        "--user-data-dir",
        str(profile),
        "--shared-browser-context",
        "--timeout-action",
        "10000",
        "--timeout-navigation",
        "90000",
        "--output-dir",
        str(output),
        "--image-responses",
        "omit",
    ]


def run_singleton(
    command: Sequence[str],
    *,
    lock_path: Path,
    environment: dict[str, str] | None = None,
    terminate_timeout_s: float = 10.0,
) -> int:
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(
                json.dumps(
                    {
                        "status": "LOCK_BUSY",
                        "lock_path": str(lock_path),
                        "exit_code": 75,
                    },
                    sort_keys=True,
                ),
                file=sys.stderr,
            )
            return 75

        child = subprocess.Popen(
            list(command),
            env=environment,
            start_new_session=True,
            close_fds=True,
            pass_fds=(lock_fd,),
        )
        stopping = False

        def request_stop(_signum: int, _frame: object) -> None:
            nonlocal stopping
            if stopping or child.poll() is not None:
                return
            stopping = True
            os.killpg(child.pid, signal.SIGTERM)

        previous_handlers = {
            sig: signal.signal(sig, request_stop)
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
        }
        try:
            while child.poll() is None:
                time.sleep(0.05)
            return int(child.returncode or 0)
        finally:
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
                deadline = time.monotonic() + terminate_timeout_s
                while child.poll() is None and time.monotonic() < deadline:
                    time.sleep(0.05)
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGKILL)
            child.wait()
    finally:
        os.close(lock_fd)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the dedicated external-review Playwright MCP singleton."
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--lock", type=Path, default=DEFAULT_LOCK)
    parser.add_argument(
        "command",
        nargs=argparse.REMAINDER,
        help="Test-only command override after --; omit for Playwright MCP.",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")
    command = list(args.command)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        command = _server_command(args.port, args.profile, args.output_dir)
    environment = dict(os.environ)
    environment.setdefault("DISPLAY", ":0")
    environment.setdefault("WAYLAND_DISPLAY", "wayland-0")
    environment.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
    environment.setdefault("XDG_SESSION_TYPE", "wayland")
    return run_singleton(command, lock_path=args.lock, environment=environment)


if __name__ == "__main__":
    raise SystemExit(main())
