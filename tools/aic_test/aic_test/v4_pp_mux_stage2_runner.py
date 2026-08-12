#!/usr/bin/env python3
"""Bounded private Stage-2 fixture supervisor; never starts AWSIM."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import tempfile
import time
from typing import Any

from aic_test.v4_pp_mux_observer import (
    classify_latest_sample_artifacts,
    write_latest_sample_selector_manifest,
)


ACCEPTABLE_LATEST_SAMPLE_TERMINALS = {
    "RECEIVED_AND_EVALUATED",
}


def _claim_fields(terminal: object) -> dict[str, bool]:
    """Keep receipt/supersession distinct from evaluated control evidence."""
    evaluated = terminal == "RECEIVED_AND_EVALUATED"
    return {
        "runner_success": evaluated,
        "pp_command_evaluated": evaluated,
        # This source/test runner never proves the externally final Mux output
        # or delivery to vehicle control, even for an evaluated input.
        "final_mux_use_proven": False,
        "vehicle_control_proven": False,
        "stage2_style_pass": False,
    }


def _build_observer_command(
    observer_argv: list[str], private_root: str, timeout: float,
    observer_result: Path, ready_path: Path, capture_epoch_nonce: str,
    capture_close_marker: Path,
) -> list[str]:
    return [
        *observer_argv,
        "--private-root", private_root,
        "--timeout", str(timeout),
        "--result", str(observer_result),
        "--ready", str(ready_path),
        "--capture-epoch-nonce", capture_epoch_nonce,
        "--capture-close-marker", str(capture_close_marker),
    ]


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
    observer: list[str], launch: list[str], driver: list[str], token: str,
    mux_capture_path: Path | None = None,
    selector_manifest_sha256: str | None = None,
    capture_epoch_nonce: str | None = None,
    capture_epoch: int | None = None,
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
        and launch[:5]
        == [
            "ros2",
            "launch",
            "aichallenge_submit_launch",
            "v4_pp_mux_stage2.launch.xml",
            f"topic_token:={token}",
        ]
        and (
            (mux_capture_path is None and len(launch) == 5)
            or (
                mux_capture_path is not None
                and isinstance(selector_manifest_sha256, str)
                and len(selector_manifest_sha256) == 64
                and isinstance(capture_epoch_nonce, str)
                and len(capture_epoch_nonce) >= 16
                and capture_epoch == 1
                and launch[5:]
                == [
                    f"mux_capture_output_path:={mux_capture_path}",
                    "mux_selector_manifest_sha256:=" + selector_manifest_sha256,
                    "mux_capture_epoch_nonce:=" + capture_epoch_nonce,
                    "mux_capture_epoch:=1",
                ]
            )
        )
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

    selector_manifest_path = args.artifact_dir / "latest-sample-selector-manifest.json"
    mux_capture_path = args.artifact_dir / "mux-latest-sample-capture.json"
    classification_path = args.artifact_dir / "mux-latest-sample-classification.json"
    driver_complete_path = args.artifact_dir / "driver-complete.json"
    if (
        args.result.exists()
        or args.observer_result.exists()
        or (args.artifact_dir / "observer-ready.json").exists()
        or selector_manifest_path.exists()
        or mux_capture_path.exists()
        or classification_path.exists()
        or driver_complete_path.exists()
    ):
        raise RuntimeError("refusing stale result path")
    capture_epoch_nonce = secrets.token_hex(16)
    observer_command = _build_observer_command(
        args.observer_argv,
        args.private_root,
        args.timeout,
        args.observer_result,
        args.artifact_dir / "observer-ready.json",
        capture_epoch_nonce,
        driver_complete_path,
    )
    selector_manifest_sha256 = write_latest_sample_selector_manifest(
        selector_manifest_path, args.private_root, capture_epoch_nonce, 1
    )
    # Launch argv is supplied explicitly by the existing installed environment;
    # it is never evaluated by a shell or inferred from production defaults.
    launch_command = [
        *args.launch_argv,
        f"mux_capture_output_path:={mux_capture_path}",
        "mux_selector_manifest_sha256:=" + selector_manifest_sha256,
        "mux_capture_epoch_nonce:=" + capture_epoch_nonce,
        "mux_capture_epoch:=1",
    ]
    driver_command = [*args.driver_argv, "--private-root", args.private_root]
    if not _validate_fixture_commands(
        args.observer_argv, launch_command, args.driver_argv, token,
        mux_capture_path, selector_manifest_sha256, capture_epoch_nonce, 1,
    ):
        parser.error("commands must bind to the private Stage-2 fixture")
    components: dict[str, subprocess.Popen[bytes]] = {}
    process_groups: dict[str, int] = {}
    log_streams: dict[str, Any] = {}
    statuses: dict[str, Any] = {}
    success = False
    try:
        ready_path = args.artifact_dir / "observer-ready.json"
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
        _write_json_atomic(
            driver_complete_path,
            {
                "schema_version": 1,
                "private_root": args.private_root,
                "capture_epoch_nonce": capture_epoch_nonce,
                "driver_complete": True,
            },
        )
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
        classification = classify_latest_sample_artifacts(
            selector_manifest_path=selector_manifest_path,
            mux_capture_path=mux_capture_path,
            pp_result_path=args.observer_result,
        )
        _write_json_atomic(classification_path, classification)
        claims = _claim_fields(classification.get("terminal"))
        if not claims["runner_success"]:
            success = False
        for stream in log_streams.values():
            stream.close()
        payload = {
            "schema_version": 1,
            "private_root": args.private_root,
            "verdict": "PASS" if success else "HOLD",
            "observer_result": str(args.observer_result),
            "latest_sample_selector_manifest": str(selector_manifest_path),
            "latest_sample_classification": str(classification_path),
            "latest_sample_terminal": classification.get("terminal"),
            **claims,
            "components": statuses,
        }
        try:
            _write_json_atomic(args.result, payload)
        except Exception:
            success = False
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
