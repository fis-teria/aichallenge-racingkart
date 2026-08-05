#!/usr/bin/env python3
"""Bounded private Stage-2 fixture supervisor; never starts AWSIM."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from typing import Any


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        os.fchmod(stream.fileno(), 0o644)
        json.dump(payload, stream, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _setsid_command(command: list[str]) -> list[str]:
    return ["setsid", *command]


def _parse_argv_json(value: str) -> list[str]:
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as error:
        raise argparse.ArgumentTypeError("argv must be JSON") from error
    if not isinstance(parsed, list) or not parsed or not all(
        isinstance(item, str) and item for item in parsed
    ):
        raise argparse.ArgumentTypeError("argv must be a nonempty JSON string list")
    return list(parsed)


def _validate_fixture_commands(
    observer: list[str], launch: list[str], driver: list[str], token: str
) -> bool:
    allowed_driver_options = {
        "--duration", "--baseline-duration", "--valid-duration",
        "--post-invalid-dwell", "--generation",
    }
    driver_options = driver[2:]
    option_names = driver_options[::2]
    return bool(
        len(observer) == 2
        and observer[0] == "python3"
        and observer[1].endswith("/v4_pp_mux_observer.py")
        and len(driver) >= 2
        and driver[0] == "python3"
        and driver[1].endswith("/v4_pp_mux_driver.py")
        and len(driver_options) % 2 == 0
        and len(option_names) == len(set(option_names))
        and set(option_names) <= allowed_driver_options
        and launch
        == [
            "ros2",
            "launch",
            "aichallenge_submit_launch",
            "v4_pp_mux_stage2.launch.xml",
            f"topic_token:={token}",
        ]
    )


def _start_owned(command: list[str], log_stream: Any) -> subprocess.Popen[bytes]:
    return subprocess.Popen(
        _setsid_command(command),
        stdin=subprocess.DEVNULL,
        stdout=log_stream,
        stderr=subprocess.STDOUT,
    )


def _group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_group_gone(
    pgid: int, timeout_s: float, process: subprocess.Popen[bytes]
) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        process.poll()  # Reap an exited leader so it cannot look like residue.
        if not _group_exists(pgid):
            return True
        time.sleep(0.05)
    return not _group_exists(pgid)


def _terminate_owned_group(
    process: subprocess.Popen[bytes], pgid: int, timeout_s: float = 2.0
) -> str:
    """Escalate only the process group created by our `setsid` command."""
    if pgid != process.pid:
        return "not_owned_process_group"
    if not _group_exists(pgid):
        process.poll()
        return "already_exited"
    for sig, label in ((signal.SIGINT, "sigint"), (signal.SIGTERM, "sigterm"), (signal.SIGKILL, "sigkill")):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return label
        if _wait_group_gone(pgid, timeout_s, process):
            process.poll()
            return label
    return "kill_timeout"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--private-root", required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--observer-result", type=Path, required=True)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--observer-argv", type=_parse_argv_json, required=True)
    parser.add_argument("--launch-argv", type=_parse_argv_json, required=True)
    parser.add_argument("--driver-argv", type=_parse_argv_json, required=True)
    args = parser.parse_args()
    token = args.private_root.removeprefix("/aic_test/v4_pp_mux_stage2/")
    if not (
        args.private_root.startswith("/aic_test/v4_pp_mux_stage2/")
        and token
        and "/" not in token
        and all(character.isalnum() or character in "_-" for character in token)
        and 3.0 <= args.timeout <= 30.0
    ):
        parser.error("bounded private Stage-2 runner configuration required")

    observer_command = [
        *args.observer_argv,
        "--private-root", args.private_root,
        "--timeout", str(args.timeout),
        "--result", str(args.observer_result),
        "--ready", str(args.artifact_dir / "observer-ready.json"),
    ]
    # Launch argv is supplied explicitly by the existing installed environment;
    # it is never evaluated by a shell or inferred from production defaults.
    launch_command = list(args.launch_argv)
    driver_command = [*args.driver_argv, "--private-root", args.private_root]
    if not _validate_fixture_commands(
        args.observer_argv, launch_command, args.driver_argv, token
    ):
        parser.error("commands must bind to the private Stage-2 fixture")
    components: dict[str, subprocess.Popen[bytes]] = {}
    process_groups: dict[str, int] = {}
    log_streams: dict[str, Any] = {}
    statuses: dict[str, Any] = {}
    success = False
    try:
        ready_path = args.artifact_dir / "observer-ready.json"
        if args.result.exists() or args.observer_result.exists() or ready_path.exists():
            raise RuntimeError("refusing stale result path")
        for name, command in (
            ("observer", observer_command),
            ("launch", launch_command),
            ("driver", driver_command),
        ):
            args.artifact_dir.mkdir(parents=True, exist_ok=True)
            log_path = args.artifact_dir / f"stage2-{name}.log"
            log_stream = log_path.open("wb")
            log_streams[name] = log_stream
            process = _start_owned(command, log_stream)
            components[name] = process
            time.sleep(0.10)
            pgid = process.pid
            try:
                actual_pgid = os.getpgid(process.pid)
            except ProcessLookupError:
                actual_pgid = -1
            process_groups[name] = pgid
            statuses[name] = {
                "pid": process.pid,
                "pgid": pgid,
                "actual_pgid": actual_pgid,
                "argv": command,
                "log": str(log_path),
                "started": True,
            }
            # Fail fast when an executable/launch exits immediately.
            if process.poll() is not None:
                statuses[name]["exit_code"] = process.returncode
                raise RuntimeError(f"{name} exited during startup")
            if actual_pgid != pgid:
                raise RuntimeError(f"{name} did not enter owned process group")
            if name == "observer":
                ready_deadline = time.monotonic() + min(3.0, args.timeout)
                while time.monotonic() < ready_deadline and not ready_path.is_file():
                    if process.poll() is not None:
                        break
                    time.sleep(0.05)
                if not ready_path.is_file():
                    raise RuntimeError("observer readiness timeout")
                try:
                    ready_payload = json.loads(ready_path.read_text())
                except (OSError, ValueError, TypeError) as error:
                    raise RuntimeError("invalid observer readiness marker") from error
                if not (
                    ready_payload.get("schema_version") == 1
                    and ready_payload.get("private_root") == args.private_root
                    and ready_payload.get("ready") is True
                ):
                    raise RuntimeError("observer readiness identity mismatch")
        try:
            components["driver"].wait(timeout=args.timeout)
        except subprocess.TimeoutExpired as error:
            raise RuntimeError("driver timeout") from error
        statuses["driver"]["exit_code"] = components["driver"].returncode
        if components["driver"].returncode != 0:
            raise RuntimeError("driver failed")
        try:
            components["observer"].wait(timeout=min(5.0, args.timeout))
        except subprocess.TimeoutExpired:
            statuses["observer"]["wait_timeout"] = True
        else:
            statuses["observer"]["exit_code"] = components["observer"].returncode
        if args.observer_result.is_file():
            try:
                observer_payload = json.loads(args.observer_result.read_text())
                success = bool(
                    observer_payload.get("schema_version") == 1
                    and observer_payload.get("private_root") == args.private_root
                    and observer_payload.get("verdict") == "PASS"
                    and observer_payload.get("invalid_fixture_classification")
                    == "PP_INVALID_CONFIRMED"
                )
            except (OSError, ValueError, TypeError):
                success = False
        else:
            success = False
    except Exception as error:
        statuses["error"] = str(error)
    finally:
        for name in ("driver", "launch", "observer"):
            process = components.get(name)
            if process is None:
                continue
            pgid = process_groups.get(name, -1)
            statuses.setdefault(name, {})["teardown"] = _terminate_owned_group(
                process, pgid
            )
            statuses[name]["residue"] = pgid > 0 and _group_exists(pgid)
            if statuses[name]["residue"]:
                success = False
        for stream in log_streams.values():
            stream.close()
        payload = {
            "schema_version": 1,
            "private_root": args.private_root,
            "verdict": "PASS" if success else "HOLD",
            "observer_result": str(args.observer_result),
            "components": statuses,
        }
        try:
            _write_json_atomic(args.result, payload)
        except Exception:
            success = False
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
