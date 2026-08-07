#!/usr/bin/env python3
"""Spawn-isolated diagnostic input publisher for C002AY0 fixtures."""

from __future__ import annotations

import copy
import argparse
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import time

from autoware_auto_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.context import Context
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from v2x_msgs.msg import V2XVehiclePositionArray
from multi_purpose_mpc_ros_msgs.msg import OvertakePlan

from c002ay0_pp_runtime_measurement import (
    E2E_INPUT_PUBLISH_TICK_LEDGER_LIMIT,
    FIXED_SEC,
    fixed_clock,
    odometry,
    overtake_plan,
    trajectory,
    v2x_fixture,
)


TICK_PERIOD_NS = 10_000_000
DEFAULT_PARENT_LEASE_TIMEOUT_SEC = 30.0
IPC_BUFFER_BYTES = 65_536
COMMAND_PHASE = 1
COMMAND_FREEZE = 2
COMMAND_SHUTDOWN = 3
ACK_READY = 1
ACK_PHASE = 2
ACK_FREEZE = 3
ACK_SHUTDOWN = 4
COMMAND = struct.Struct("!IIQB7x")
ACK = struct.Struct("!IIQQQQ6I")
TOPIC_COUNT = 6
MARKERS = {
    "scope": "isolated_input_publisher_diagnostic",
    "diagnostic_only": True,
    "not_acceptance": True,
    "m4_wcet_eligible": False,
    "m4_acceptance_credit": False,
    "parent_lease_timeout_sec": DEFAULT_PARENT_LEASE_TIMEOUT_SEC,
    "launcher_diagnostic_baseline": "direct_popen_exec_v1",
    "launcher_timing_parity": False,
}
EXPECTED_ACK_KIND = {
    COMMAND_PHASE: ACK_PHASE,
    COMMAND_FREEZE: ACK_FREEZE,
    COMMAND_SHUTDOWN: ACK_SHUTDOWN,
}
ASSOCIATED_CHILD_ROLE = "isolated_spawn_associated_child_unclassified"
ASSOCIATED_CHILD_PROVENANCE = (
    "direct_child_delta_around_subprocess_spawn"
)
CHILD_MODE_ARGUMENT = "--c002ay0-isolated-child"
CHILD_MODULE_PATH = str(Path(__file__).resolve())
PYTHON_EXECUTABLE = sys.executable
CYCLONEDDS_URI_KEY = "CYCLONEDDS_URI"


def require_explicit_empty_cyclonedds_uri(
    environment: dict[str, str] | os._Environ[str],
) -> None:
    if (
        CYCLONEDDS_URI_KEY not in environment
        or environment[CYCLONEDDS_URI_KEY] != ""
    ):
        raise AssertionError(
            "isolated_publisher_cyclonedds_uri_not_explicit_empty"
        )


class DirectChildSample(dict):
    def __init__(
        self,
        children: dict[int, dict[str, object]] | None = None,
        *,
        errors: list[dict[str, object]] | None = None,
    ) -> None:
        super().__init__(children or {})
        self.errors = list(errors or [])


def process_identity_snapshot(pid: int) -> dict[str, object]:
    process_root = f"/proc/{pid}"
    try:
        with open(f"{process_root}/stat") as stream:
            content = stream.read()
        right_parenthesis = content.rfind(")")
        if right_parenthesis < 0:
            raise ValueError("stat comm terminator missing")
        comm = content[content.find("(") + 1 : right_parenthesis]
        fields = content[right_parenthesis + 2 :].split()
        if len(fields) <= 19:
            raise ValueError("stat fields incomplete")
        state = fields[0]
        ppid = int(fields[1])
        session_id = int(fields[3])
        starttime_ticks = int(fields[19])
    except (FileNotFoundError, ProcessLookupError) as error:
        return {
            "pid": pid,
            "readable": False,
            "classification": "absent",
            "error": f"{type(error).__name__}: {error}",
        }
    except (PermissionError, ValueError) as error:
        return {
            "pid": pid,
            "readable": False,
            "classification": "unreadable",
            "error": f"{type(error).__name__}: {error}",
        }
    try:
        with open(f"{process_root}/cmdline", "rb") as stream:
            cmdline = (
                stream.read()
                .replace(b"\0", b" ")
                .decode(errors="replace")
                .strip()
            )
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        cmdline = ""
    try:
        with open(f"{process_root}/cgroup") as stream:
            cgroup = stream.read().splitlines()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        cgroup = []
    return {
        "pid": pid,
        "readable": True,
        "classification": "zombie" if state == "Z" else "live",
        "state": state,
        "ppid": ppid,
        "sid": session_id,
        "session_id": session_id,
        "starttime_ticks": starttime_ticks,
        "cmdline": cmdline,
        "comm": comm,
        "cgroup": cgroup,
    }


def sample_direct_child_identities(
    owner_pid: int,
) -> dict[int, dict[str, object]]:
    errors: list[dict[str, object]] = []
    try:
        task_entries = list(os.scandir(f"/proc/{owner_pid}/task"))
    except (FileNotFoundError, PermissionError, OSError) as error:
        return DirectChildSample(
            errors=[
                {
                    "pid": owner_pid,
                    "kind": "owner_task_enumeration_failed",
                    "detail": f"{type(error).__name__}: {error}",
                }
            ]
        )
    child_pids: set[int] = set()
    for entry in task_entries:
        if not entry.name.isdigit():
            errors.append(
                {
                    "pid": owner_pid,
                    "kind": "owner_task_pid_parse_invalid",
                    "detail": f"task_entry={entry.name!r}",
                }
            )
            continue
        task_id = int(entry.name)
        try:
            with open(
                f"/proc/{owner_pid}/task/{task_id}/children"
            ) as stream:
                child_tokens = stream.read().split()
        except (FileNotFoundError, PermissionError, OSError) as error:
            errors.append(
                {
                    "pid": task_id,
                    "kind": "owner_task_children_read_failed",
                    "detail": f"{type(error).__name__}: {error}",
                }
            )
            continue
        for token in child_tokens:
            try:
                child_pid = int(token)
            except ValueError:
                errors.append(
                    {
                        "pid": task_id,
                        "kind": "direct_child_pid_parse_invalid",
                        "detail": f"child_token={token!r}",
                    }
                )
                continue
            if child_pid <= 0:
                errors.append(
                    {
                        "pid": task_id,
                        "kind": "direct_child_pid_parse_invalid",
                        "detail": f"child_pid={child_pid}",
                    }
                )
                continue
            child_pids.add(child_pid)
    children = {}
    for pid in sorted(child_pids):
        identity = process_identity_snapshot(pid)
        children[pid] = identity
        if identity.get("readable") is not True:
            errors.append(
                {
                    "pid": pid,
                    "kind": "direct_child_identity_unreadable",
                    "detail": str(identity.get("error", "")),
                }
            )
        elif identity.get("ppid") != owner_pid:
            errors.append(
                {
                    "pid": pid,
                    "kind": "direct_child_ppid_mismatch",
                    "detail": (
                        f"expected={owner_pid} "
                        f"actual={identity.get('ppid')}"
                    ),
                }
            )
    return DirectChildSample(children, errors=errors)


def invoke_direct_child_sampler(
    sampler: object,
    owner_pid: int,
    stage: str,
) -> DirectChildSample:
    try:
        sampled = sampler(owner_pid)
    except BaseException as error:
        return DirectChildSample(
            errors=[
                {
                    "pid": owner_pid,
                    "kind": f"{stage}_sampler_failed",
                    "detail": f"{type(error).__name__}: {error}",
                }
            ]
        )
    if isinstance(sampled, DirectChildSample):
        return sampled
    if isinstance(sampled, dict):
        return DirectChildSample(sampled)
    return DirectChildSample(
        errors=[
            {
                "pid": owner_pid,
                "kind": f"{stage}_sampler_result_invalid",
                "detail": f"type={type(sampled).__name__}",
            }
        ]
    )


def direct_child_sample_failure_text(
    error: dict[str, object],
) -> str:
    return f"{error.get('kind')}: {error.get('detail')}"


def direct_child_sample_error_is_global(
    error: dict[str, object],
) -> bool:
    return str(error.get("kind", "")).startswith(
        (
            "owner_task_",
            "direct_child_pid_parse_",
            "before_spawn_sampler_",
            "after_spawn_sampler_",
            "cleanup_sampler_",
        )
    )


def global_direct_child_sample_failures(
    sample: DirectChildSample,
) -> list[str]:
    return [
        direct_child_sample_failure_text(error)
        for error in sample.errors
        if direct_child_sample_error_is_global(error)
    ]


def close_spawn_sockets(
    parent_socket: object,
    child_socket: object,
) -> list[str]:
    failures = []
    for label, endpoint in (
        ("child_socket", child_socket),
        ("parent_socket", parent_socket),
    ):
        try:
            endpoint.close()
        except BaseException as error:
            failures.append(
                f"{label}={type(error).__name__}: {error}"
            )
    return failures


def spawn_sample_failure_error(
    *,
    stage: str,
    sample_failures: list[str],
    cleanup_failures: list[str],
) -> AssertionError:
    sample_text = "; ".join(sample_failures)
    cleanup_text = "; ".join(cleanup_failures)
    error = AssertionError(
        f"isolated_publisher_{stage}_sample_invalid "
        f"sample_failure={sample_text} "
        f"cleanup_failure={cleanup_text}"
    )
    setattr(error, "_associated_child_sample_failures", sample_failures)
    setattr(error, "_target_cleanup_failure", cleanup_text)
    return error


def validate_exact_spawn_target(
    *,
    before: DirectChildSample,
    after: DirectChildSample,
    target_pid: int,
    owner_pid: int,
    owner_sid: int,
) -> tuple[dict[str, object] | None, list[str]]:
    failures = [
        *(
            f"before_{direct_child_sample_failure_text(error)}"
            for error in before.errors
        ),
        *(
            f"after_{direct_child_sample_failure_text(error)}"
            for error in after.errors
        ),
    ]
    delta: dict[int, dict[str, object]] = {}
    for pid, identity in after.items():
        previous = before.get(pid)
        if (
            isinstance(previous, dict)
            and previous.get("starttime_ticks")
            == identity.get("starttime_ticks")
        ):
            continue
        delta[pid] = identity
    if set(delta) != {target_pid}:
        failures.append(
            "direct_child_delta_not_exact "
            f"expected={[target_pid]} actual={sorted(delta)}"
        )
    target = delta.get(target_pid)
    if target is None:
        failures.append("target_missing_from_direct_child_delta")
        return None, list(dict.fromkeys(failures))
    if target.get("readable") is not True:
        failures.append(
            "target_identity_unreadable "
            f"detail={target.get('error', '')}"
        )
    if target.get("pid") != target_pid:
        failures.append(
            "target_pid_mismatch "
            f"expected={target_pid} actual={target.get('pid')}"
        )
    starttime_ticks = target.get("starttime_ticks")
    if type(starttime_ticks) is not int or starttime_ticks <= 0:
        failures.append(
            "target_starttime_invalid "
            f"actual={starttime_ticks}"
        )
    if target.get("ppid") != owner_pid:
        failures.append(
            "target_ppid_mismatch "
            f"expected={owner_pid} actual={target.get('ppid')}"
        )
    if target.get("session_id") != owner_sid:
        failures.append(
            "target_sid_mismatch "
            f"expected={owner_sid} actual={target.get('session_id')}"
        )
    return target, list(dict.fromkeys(failures))


def target_lifecycle_record(
    identity: dict[str, object],
    *,
    owner_pid: int,
    spawn_before_monotonic_ns: int,
    spawn_after_monotonic_ns: int,
) -> dict[str, object]:
    return {
        "pid": identity["pid"],
        "role": "isolated_input_publisher",
        "provenance": "exact_direct_popen_child_delta",
        "owner_pid": owner_pid,
        "starttime_ticks": identity["starttime_ticks"],
        "sid": identity["session_id"],
        "session_id": identity["session_id"],
        "ppid": identity["ppid"],
        "identity_at_spawn": copy.deepcopy(identity),
        "identity_before_cleanup": None,
        "identity_poststop": None,
        "descendants_before_stop": [],
        "descendants_poststop": [],
        "spawn_before_monotonic_ns": spawn_before_monotonic_ns,
        "spawn_after_monotonic_ns": spawn_after_monotonic_ns,
        "cleanup_stage": "spawn_observed",
        "alive": True,
        "exitcode": None,
        "unresolved": True,
        "cleanup_failure": "",
    }


def build_spawn_associated_child_ledger(
    *,
    before: dict[int, dict[str, object]],
    after: dict[int, dict[str, object]],
    target_pid: int,
    owner_pid: int,
    spawn_before_monotonic_ns: int,
    spawn_after_monotonic_ns: int,
) -> list[dict[str, object]]:
    records = []
    records_by_pid: dict[int, dict[str, object]] = {}
    for pid, identity in sorted(after.items()):
        previous = before.get(pid)
        if pid == target_pid or (
            isinstance(previous, dict)
            and previous.get("starttime_ticks")
            == identity.get("starttime_ticks")
        ):
            continue
        record = {
                "pid": pid,
                "role": ASSOCIATED_CHILD_ROLE,
                "provenance": ASSOCIATED_CHILD_PROVENANCE,
                "owner_pid": owner_pid,
                "starttime_ticks": identity.get("starttime_ticks"),
                "sid": identity.get("session_id"),
                "session_id": identity.get("session_id"),
                "ppid": identity.get("ppid"),
                "state": identity.get("state"),
                "cmdline": identity.get("cmdline"),
                "comm": identity.get("comm"),
                "cgroup": copy.deepcopy(identity.get("cgroup")),
                "spawn_before_monotonic_ns": (
                    spawn_before_monotonic_ns
                ),
                "spawn_after_monotonic_ns": spawn_after_monotonic_ns,
                "identity_at_spawn": copy.deepcopy(identity),
                "identity_before_cleanup": copy.deepcopy(identity),
                "cleanup_stage": "spawn_observed",
                "alive": True,
                "exitcode": None,
                "unresolved": True,
                "cleanup_failure": "",
            }
        records.append(record)
        records_by_pid[pid] = record
    sample_errors = [
        *getattr(before, "errors", []),
        *getattr(after, "errors", []),
    ]
    for sample_error in sample_errors:
        if direct_child_sample_error_is_global(sample_error):
            continue
        error_pid = sample_error.get("pid")
        if (
            type(error_pid) is not int
            or error_pid <= 0
            or error_pid == target_pid
        ):
            continue
        detail = direct_child_sample_failure_text(sample_error)
        record = records_by_pid.get(error_pid)
        if record is None:
            identity = after.get(error_pid) or before.get(error_pid) or {
                "pid": error_pid,
                "readable": False,
                "classification": "unreadable",
                "error": detail,
            }
            record = {
                "pid": error_pid,
                "role": ASSOCIATED_CHILD_ROLE,
                "provenance": ASSOCIATED_CHILD_PROVENANCE,
                "owner_pid": owner_pid,
                "starttime_ticks": identity.get("starttime_ticks"),
                "sid": identity.get("session_id"),
                "session_id": identity.get("session_id"),
                "ppid": identity.get("ppid"),
                "state": identity.get("state"),
                "cmdline": identity.get("cmdline"),
                "comm": identity.get("comm"),
                "cgroup": copy.deepcopy(identity.get("cgroup", [])),
                "spawn_before_monotonic_ns": (
                    spawn_before_monotonic_ns
                ),
                "spawn_after_monotonic_ns": spawn_after_monotonic_ns,
                "identity_at_spawn": copy.deepcopy(identity),
                "identity_before_cleanup": copy.deepcopy(identity),
                "cleanup_stage": "spawn_sample_invalid",
                "alive": True,
                "exitcode": None,
                "unresolved": True,
                "cleanup_failure": detail,
                "sample_invalid": True,
            }
            records.append(record)
            records_by_pid[error_pid] = record
        else:
            previous_failure = str(record.get("cleanup_failure", ""))
            record.update(
                {
                    "cleanup_stage": "spawn_sample_invalid",
                    "unresolved": True,
                    "cleanup_failure": "; ".join(
                        value
                        for value in (previous_failure, detail)
                        if value
                    ),
                    "sample_invalid": True,
                }
            )
    records.sort(key=lambda record: int(record["pid"]))
    return records


def reconcile_spawn_associated_child_ledger(
    records: list[dict[str, object]],
    *,
    owner_pid: int,
    direct_child_sampler: object = sample_direct_child_identities,
    waitpid: object = os.waitpid,
    sample_failures: list[str] | None = None,
) -> list[dict[str, object]]:
    current = invoke_direct_child_sampler(
        direct_child_sampler,
        owner_pid,
        "cleanup",
    )
    current_errors = list(getattr(current, "errors", []))
    global_sample_invalid = any(
        direct_child_sample_error_is_global(error)
        for error in current_errors
    )
    if sample_failures is not None:
        for error in current_errors:
            if direct_child_sample_error_is_global(error):
                failure = direct_child_sample_failure_text(error)
                if failure not in sample_failures:
                    sample_failures.append(failure)
    errors_by_pid: dict[int, list[str]] = {}
    for error in current_errors:
        error_pid = error.get("pid")
        if type(error_pid) is int and error_pid > 0:
            errors_by_pid.setdefault(error_pid, []).append(
                f"{error.get('kind')}: {error.get('detail')}"
            )
    for record in records:
        pid = int(record["pid"])
        expected = record["identity_at_spawn"]
        cleanup_sample_errors = errors_by_pid.get(pid, [])
        if (
            record.get("sample_invalid") is True
            or global_sample_invalid
            or cleanup_sample_errors
        ):
            previous_failure = str(record.get("cleanup_failure", ""))
            record.update(
                {
                    "cleanup_stage": "cleanup_sample_invalid",
                    "alive": True,
                    "exitcode": None,
                    "unresolved": True,
                    "cleanup_failure": "; ".join(
                        value
                        for value in (
                            previous_failure,
                            *cleanup_sample_errors,
                            (
                                "direct child cleanup sample incomplete"
                                if global_sample_invalid
                                else ""
                            ),
                        )
                        if value
                    ),
                }
            )
            continue
        observed = current.get(pid)
        if observed is None:
            record.update(
                {
                    "cleanup_stage": "absent_no_residue",
                    "alive": False,
                    "exitcode": None,
                    "unresolved": False,
                    "cleanup_failure": "",
                }
            )
            continue
        identity_matches = (
            observed.get("readable") is True
            and observed.get("pid") == pid
            and observed.get("starttime_ticks")
            == expected.get("starttime_ticks")
            and observed.get("session_id")
            == expected.get("session_id")
            and observed.get("ppid") == owner_pid
            and expected.get("ppid") == owner_pid
        )
        if not identity_matches:
            record.update(
                {
                    "cleanup_stage": "identity_mismatch_not_reaped",
                    "alive": True,
                    "exitcode": None,
                    "unresolved": True,
                    "cleanup_failure": (
                        "associated child identity changed; not reaped"
                    ),
                }
            )
            continue
        record["identity_before_cleanup"] = copy.deepcopy(observed)
        if observed.get("state") != "Z":
            record.update(
                {
                    "cleanup_stage": "unknown_live_not_signaled",
                    "alive": True,
                    "exitcode": None,
                    "unresolved": True,
                    "cleanup_failure": (
                        "associated child remains live; not signaled"
                    ),
                }
            )
            continue
        try:
            waited_pid, wait_status = waitpid(pid, os.WNOHANG)
        except (ChildProcessError, OSError) as error:
            record.update(
                {
                    "cleanup_stage": "waitpid_failed",
                    "alive": True,
                    "exitcode": None,
                    "unresolved": True,
                    "cleanup_failure": (
                        f"associated child waitpid failed: {error}"
                    ),
                }
            )
            continue
        if waited_pid != pid:
            record.update(
                {
                    "cleanup_stage": "zombie_not_reaped",
                    "alive": True,
                    "exitcode": None,
                    "unresolved": True,
                    "cleanup_failure": (
                        "associated child zombie remained after WNOHANG"
                    ),
                }
            )
            continue
        record.update(
            {
                "cleanup_stage": "reaped_no_residue",
                "alive": False,
                "exitcode": os.waitstatus_to_exitcode(wait_status),
                "unresolved": False,
                "cleanup_failure": "",
            }
        )
    return records


def configure_ipc(sock: socket.socket) -> None:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, IPC_BUFFER_BYTES)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, IPC_BUFFER_BYTES)
    sock.setblocking(False)


class PopenProcessAdapter:
    """Small lifecycle-compatible view of the one known Popen target."""

    def __init__(self, popen: subprocess.Popen[bytes]) -> None:
        self._popen = popen

    @property
    def pid(self) -> int:
        return self._popen.pid

    @property
    def exitcode(self) -> int | None:
        return self._popen.poll()

    def is_alive(self) -> bool:
        return self._popen.poll() is None

    def join(self, timeout: float | None = None) -> None:
        try:
            self._popen.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass

    def terminate(self) -> None:
        self._popen.terminate()

    def kill(self) -> None:
        self._popen.kill()


def child_argv(
    *,
    command_fd: int,
    parent_pid: int,
    curved: bool,
    real_state_lattice: bool,
    parent_lease_timeout_sec: float,
) -> list[str]:
    return [
        PYTHON_EXECUTABLE,
        CHILD_MODULE_PATH,
        CHILD_MODE_ARGUMENT,
        "--command-fd",
        str(command_fd),
        "--parent-pid",
        str(parent_pid),
        "--curved",
        "1" if curved else "0",
        "--real-state-lattice",
        "1" if real_state_lattice else "0",
        "--parent-lease-timeout-sec",
        repr(parent_lease_timeout_sec),
    ]


def parse_child_argv(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(CHILD_MODE_ARGUMENT, action="store_true")
    parser.add_argument("--command-fd", type=int, required=True)
    parser.add_argument("--parent-pid", type=int, required=True)
    parser.add_argument("--curved", type=int, choices=(0, 1), required=True)
    parser.add_argument(
        "--real-state-lattice",
        type=int,
        choices=(0, 1),
        required=True,
    )
    parser.add_argument(
        "--parent-lease-timeout-sec",
        type=float,
        required=True,
    )
    parsed = parser.parse_args(argv)
    if not getattr(parsed, "c002ay0_isolated_child"):
        parser.error(f"{CHILD_MODE_ARGUMENT} is required")
    if parsed.command_fd < 0:
        parser.error("--command-fd must be non-negative")
    if parsed.parent_pid <= 0:
        parser.error("--parent-pid must be positive")
    if parsed.parent_pid == os.getpid():
        parser.error("--parent-pid must differ from child pid")
    if (
        not math.isfinite(parsed.parent_lease_timeout_sec)
        or parsed.parent_lease_timeout_sec <= 0.0
    ):
        parser.error("--parent-lease-timeout-sec must be positive")
    return parsed


def child_socket_from_fd(command_fd: int) -> socket.socket:
    command_socket = socket.socket(fileno=command_fd)
    socket_type = command_socket.getsockopt(socket.SOL_SOCKET, socket.SO_TYPE)
    if socket_type != socket.SOCK_SEQPACKET:
        command_socket.close()
        raise AssertionError(
            "isolated_publisher_command_socket_type_invalid "
            f"expected={socket.SOCK_SEQPACKET} actual={socket_type}"
        )
    return command_socket


def validate_command(
    packet: bytes,
    *,
    expected_sequence: int,
    expected_generation: int,
) -> tuple[int, int, int, bool]:
    if len(packet) != COMMAND.size:
        raise AssertionError("isolated_publisher_command_size_invalid")
    command, sequence, generation, armed = COMMAND.unpack(packet)
    if command not in (COMMAND_PHASE, COMMAND_FREEZE, COMMAND_SHUTDOWN):
        raise AssertionError("isolated_publisher_command_unknown")
    if sequence != expected_sequence:
        raise AssertionError(
            "isolated_publisher_command_sequence_invalid "
            f"expected={expected_sequence} actual={sequence}"
        )
    if generation != expected_generation:
        raise AssertionError(
            "isolated_publisher_command_generation_invalid "
            f"expected={expected_generation} actual={generation}"
        )
    if armed not in (0, 1):
        raise AssertionError("isolated_publisher_command_armed_invalid")
    return command, sequence, generation, bool(armed)


def send_fixed_packet(sock: object, packet: bytes, label: str) -> None:
    try:
        sent = sock.send(packet)
    except BlockingIOError as error:
        raise AssertionError(
            f"isolated_publisher_{label}_ipc_full"
        ) from error
    if sent != len(packet):
        raise AssertionError(
            f"isolated_publisher_{label}_ipc_partial "
            f"sent={sent} expected={len(packet)}"
        )


def stop_process_bounded(process: object) -> None:
    process.join(1.0)
    if process.is_alive():
        process.terminate()
        process.join(1.0)
    if process.is_alive():
        process.kill()
        process.join(1.0)
    if process.is_alive():
        raise AssertionError("isolated_publisher_process_residue")


def validate_release(now_ns: int, release_ns: int) -> None:
    if now_ns - release_ns >= TICK_PERIOD_NS:
        raise AssertionError(
            "isolated_publisher_missed_release "
            f"lateness_ns={now_ns - release_ns}"
        )


def validate_ack(
    ack: dict[str, object],
    *,
    previous_ack: dict[str, object] | None,
    expected_kind: int,
    expected_sequence: int,
    expected_generation: int,
) -> None:
    if ack["kind"] != expected_kind:
        raise AssertionError(
            "isolated_publisher_ack_kind_invalid "
            f"expected={expected_kind} actual={ack['kind']}"
        )
    if (
        ack["sequence"] != expected_sequence
        or ack["generation"] != expected_generation
    ):
        raise AssertionError("isolated_publisher_ack_identity_invalid")
    if ack["publish_complete_watermark"] != ack["tick_index"]:
        raise AssertionError("isolated_publisher_ack_watermark_invalid")
    if previous_ack is not None and (
        ack["tick_index"] <= previous_ack["tick_index"]
        or ack["publish_complete_watermark"]
        <= previous_ack["publish_complete_watermark"]
        or ack["stamp_ns"] <= previous_ack["stamp_ns"]
    ):
        raise AssertionError("isolated_publisher_ack_nonmonotonic")
    if min(ack["topic_counts"]) <= 0:
        raise AssertionError("isolated_publisher_ack_topic_ledger_invalid")


def _child_main(
    command_socket: socket.socket,
    parent_pid: int,
    curved: bool,
    real_state_lattice: bool,
    parent_lease_timeout_sec: float,
) -> None:
    if os.getpid() == parent_pid:
        raise AssertionError("isolated_publisher_pid_not_isolated")
    require_explicit_empty_cyclonedds_uri(os.environ)
    configure_ipc(command_socket)
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        "c002ay0_isolated_input_publisher",
        context=context,
    )
    publishers = (
        node.create_publisher(Clock, "/clock", 10),
        node.create_publisher(Odometry, "/ay0/input/kinematics", 10),
        node.create_publisher(
            V2XVehiclePositionArray,
            "/ay0/input/v2x",
            10,
        ),
        node.create_publisher(
            Trajectory,
            "/ay0/input/trajectory",
            10,
        ),
        node.create_publisher(
            OvertakePlan,
            "/ay0/input/overtake_plan",
            10,
        ),
        node.create_publisher(
            Bool,
            "/ay0/input/race_armed",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        ),
    )
    input_publish_tick_pub = node.create_publisher(
        String,
        "/test/c002ay0/input_publish_tick",
        QoSProfile(
            depth=E2E_INPUT_PUBLISH_TICK_LEDGER_LIMIT,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    clock_message = fixed_clock()
    odom_message = odometry(real_state_lattice)
    v2x_message = v2x_fixture()
    trajectory_message = trajectory(curved, real_state_lattice)
    plan_message = overtake_plan()
    contract_divisor = 10 if real_state_lattice else 5
    counts = [0] * TOPIC_COUNT
    command_sequence = 0
    generation = 0
    armed = False
    pending: tuple[int, int, int, bool] | None = None
    frozen = False
    tick_index = 0
    publish_complete_watermark = 0
    started_ns = time.monotonic_ns()
    next_release_ns = started_ns
    lease_timeout_ns = int(parent_lease_timeout_sec * 1e9)
    last_parent_contact_ns = started_ns
    audit_discovery_deadline_ns = started_ns + 4_000_000_000
    while (
        input_publish_tick_pub.get_subscription_count() < 1
        and time.monotonic_ns() < audit_discovery_deadline_ns
    ):
        time.sleep(0.01)
    if input_publish_tick_pub.get_subscription_count() < 1:
        raise AssertionError(
            "isolated_publisher_input_tick_audit_subscriber_missing"
        )

    def send_ack(
        ack_kind: int,
        sequence: int,
        ack_generation: int,
        stamp_ns: int,
    ) -> None:
        send_fixed_packet(
            command_socket,
            ACK.pack(
                ack_kind,
                sequence,
                ack_generation,
                tick_index,
                publish_complete_watermark,
                stamp_ns,
                *counts,
            ),
            "ack",
        )

    send_ack(ACK_READY, 0, 0, FIXED_SEC * 1_000_000_000)
    try:
        while True:
            now_ns = time.monotonic_ns()
            timeout_sec = max(0.0, (next_release_ns - now_ns) / 1e9)
            timeout_sec = min(
                timeout_sec,
                max(
                    0.0,
                    (
                        last_parent_contact_ns
                        + lease_timeout_ns
                        - now_ns
                    )
                    / 1e9,
                ),
            )
            readable, _, _ = select.select(
                (command_socket,),
                (),
                (),
                timeout_sec,
            )
            if readable:
                packet = command_socket.recv(COMMAND.size + 1)
                if not packet:
                    raise AssertionError(
                        "isolated_publisher_parent_eof"
                    )
                if pending is not None:
                    raise AssertionError(
                        "isolated_publisher_command_overlap"
                    )
                pending = validate_command(
                    packet,
                    expected_sequence=command_sequence + 1,
                    expected_generation=generation + 1,
                )
                command_sequence = pending[1]
                generation = pending[2]
                last_parent_contact_ns = time.monotonic_ns()
                if pending[0] == COMMAND_PHASE:
                    armed = pending[3]
                if pending[0] in (COMMAND_FREEZE, COMMAND_SHUTDOWN):
                    frozen = False
                continue

            now_ns = time.monotonic_ns()
            if now_ns - last_parent_contact_ns >= lease_timeout_ns:
                publishers[5].publish(Bool(data=False))
                counts[5] += 1
                raise AssertionError(
                    "isolated_publisher_parent_lease_expired"
                )
            validate_release(now_ns, next_release_ns)
            if frozen:
                next_release_ns += TICK_PERIOD_NS
                continue
            tick_index += 1
            contract_tick = tick_index % contract_divisor == 0
            if (
                contract_tick
                and pending is not None
                and pending[0] in (COMMAND_FREEZE, COMMAND_SHUTDOWN)
            ):
                armed = False
            stamp_ns = (
                FIXED_SEC * 1_000_000_000
                + tick_index * TICK_PERIOD_NS
            )
            clock_message.clock.sec = stamp_ns // 1_000_000_000
            clock_message.clock.nanosec = stamp_ns % 1_000_000_000
            odom_message.header.stamp.sec = clock_message.clock.sec
            odom_message.header.stamp.nanosec = (
                clock_message.clock.nanosec
            )
            v2x_message.header.stamp.sec = clock_message.clock.sec
            v2x_message.header.stamp.nanosec = (
                clock_message.clock.nanosec
            )
            for vehicle in v2x_message.vehicles:
                vehicle.header.stamp.sec = clock_message.clock.sec
                vehicle.header.stamp.nanosec = (
                    clock_message.clock.nanosec
                )
            publishers[0].publish(clock_message)
            clock_publish_steady_ns = time.monotonic_ns()
            counts[0] += 1
            publishers[1].publish(odom_message)
            counts[1] += 1
            publishers[2].publish(v2x_message)
            counts[2] += 1
            if contract_tick:
                trajectory_message.header.stamp.sec = (
                    clock_message.clock.sec
                )
                trajectory_message.header.stamp.nanosec = (
                    clock_message.clock.nanosec
                )
                plan_message.header.stamp.sec = clock_message.clock.sec
                plan_message.header.stamp.nanosec = (
                    clock_message.clock.nanosec
                )
                publishers[3].publish(trajectory_message)
                counts[3] += 1
                publishers[4].publish(plan_message)
                counts[4] += 1
                plan_publish_steady_ns = time.monotonic_ns()
                input_publish_tick_pub.publish(
                    String(
                        data=json.dumps(
                            {
                                "schema_version": 1,
                                "tick_index": tick_index,
                                "clock_stamp_ns": stamp_ns,
                                "trajectory_stamp_ns": stamp_ns,
                                "plan_stamp_ns": stamp_ns,
                                "clock_publish_steady_ns": (
                                    clock_publish_steady_ns
                                ),
                                "plan_publish_steady_ns": (
                                    plan_publish_steady_ns
                                ),
                                "covered_clock_first_stamp_ns": (
                                    stamp_ns
                                    - (contract_divisor - 1)
                                    * TICK_PERIOD_NS
                                ),
                                "covered_clock_last_stamp_ns": stamp_ns,
                                "covered_clock_tick_period_ns": (
                                    TICK_PERIOD_NS
                                ),
                                "covered_clock_tick_count": (
                                    contract_divisor
                                ),
                            },
                            sort_keys=True,
                            separators=(",", ":"),
                        )
                    )
                )
            publishers[5].publish(Bool(data=armed))
            counts[5] += 1

            if pending is not None and contract_tick:
                publish_complete_watermark = tick_index
                command, sequence, ack_generation, _ = pending
                pending = None
                ack_kind = {
                    COMMAND_PHASE: ACK_PHASE,
                    COMMAND_FREEZE: ACK_FREEZE,
                    COMMAND_SHUTDOWN: ACK_SHUTDOWN,
                }[command]
                send_ack(
                    ack_kind,
                    sequence,
                    ack_generation,
                    stamp_ns,
                )
                if command == COMMAND_FREEZE:
                    frozen = True
                if command == COMMAND_SHUTDOWN:
                    break
            next_release_ns += TICK_PERIOD_NS
    finally:
        node.destroy_publisher(input_publish_tick_pub)
        for publisher in publishers:
            node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown(context=context)
        command_socket.close()


def child_entry(argv: list[str]) -> None:
    parsed = parse_child_argv(argv)
    command_socket = child_socket_from_fd(parsed.command_fd)
    _child_main(
        command_socket,
        parsed.parent_pid,
        bool(parsed.curved),
        bool(parsed.real_state_lattice),
        parsed.parent_lease_timeout_sec,
    )


class IsolatedInputPublisher:
    def __init__(
        self,
        *,
        curved: bool,
        real_state_lattice: bool,
        parent_lease_timeout_sec: float = (
            DEFAULT_PARENT_LEASE_TIMEOUT_SEC
        ),
        direct_child_sampler: object = sample_direct_child_identities,
        monotonic_ns: object = time.monotonic_ns,
        waitpid: object = os.waitpid,
        popen_factory: object = subprocess.Popen,
        process_identity_sampler: object = process_identity_snapshot,
    ) -> None:
        if (
            not math.isfinite(parent_lease_timeout_sec)
            or parent_lease_timeout_sec <= 0.0
        ):
            raise ValueError("parent lease timeout must be positive")
        self._owner_pid = os.getpid()
        self._owner_sid = os.getsid(self._owner_pid)
        self._direct_child_sampler = direct_child_sampler
        self._process_identity_sampler = process_identity_sampler
        self._waitpid = waitpid
        self._target_process_lifecycle: dict[str, object] | None = None
        self._associated_child_lifecycle: list[
            dict[str, object]
        ] = []
        self._associated_child_cleanup_complete = False
        self._associated_child_cleanup_failure = ""
        self._associated_child_sample_failures: list[str] = []
        self._target_cleanup_complete = False
        self._target_cleanup_failure = ""
        self._closed = False
        self._shutdown_complete = False
        self._parent_socket, child_socket = socket.socketpair(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )
        configure_ipc(self._parent_socket)
        spawn_before_monotonic_ns = monotonic_ns()
        direct_children_before = invoke_direct_child_sampler(
            direct_child_sampler,
            self._owner_pid,
            "before_spawn",
        )
        before_sample_failures = [
            direct_child_sample_failure_text(error)
            for error in direct_children_before.errors
        ]
        if before_sample_failures:
            cleanup_failures = close_spawn_sockets(
                self._parent_socket,
                child_socket,
            )
            self._closed = True
            self._associated_child_sample_failures = (
                before_sample_failures
            )
            self._associated_child_cleanup_failure = "; ".join(
                cleanup_failures
            )
            raise spawn_sample_failure_error(
                stage="before_spawn",
                sample_failures=before_sample_failures,
                cleanup_failures=cleanup_failures,
            )
        child_fd = child_socket.fileno()
        argv = child_argv(
            command_fd=child_fd,
            parent_pid=self._owner_pid,
            curved=curved,
            real_state_lattice=real_state_lattice,
            parent_lease_timeout_sec=parent_lease_timeout_sec,
        )
        child_environment = os.environ.copy()
        require_explicit_empty_cyclonedds_uri(child_environment)
        try:
            popen = popen_factory(
                argv,
                env=child_environment,
                stdin=subprocess.DEVNULL,
                pass_fds=(child_fd,),
                close_fds=True,
                shell=False,
                preexec_fn=None,
            )
        except BaseException as error:
            cleanup_failures = close_spawn_sockets(
                self._parent_socket,
                child_socket,
            )
            self._closed = True
            try:
                setattr(error, "_spawn_cleanup_failures", cleanup_failures)
            except BaseException:
                pass
            raise
        self._process = PopenProcessAdapter(popen)
        direct_children_after = invoke_direct_child_sampler(
            direct_child_sampler,
            self._owner_pid,
            "after_spawn",
        )
        spawn_after_monotonic_ns = monotonic_ns()
        target_identity, after_sample_failures = (
            validate_exact_spawn_target(
                before=direct_children_before,
                after=direct_children_after,
                target_pid=self._process.pid,
                owner_pid=self._owner_pid,
                owner_sid=self._owner_sid,
            )
        )
        if after_sample_failures:
            cleanup_failures = []
            try:
                stop_process_bounded(self._process)
            except BaseException as error:
                cleanup_failures.append(
                    "target_process="
                    f"{type(error).__name__}: {error}"
                )
            finally:
                cleanup_failures.extend(
                    close_spawn_sockets(
                        self._parent_socket,
                        child_socket,
                    )
                )
                self._closed = True
            self._associated_child_sample_failures = after_sample_failures
            self._associated_child_cleanup_failure = "; ".join(
                cleanup_failures
            )
            raise spawn_sample_failure_error(
                stage="after_spawn",
                sample_failures=after_sample_failures,
                cleanup_failures=cleanup_failures,
            )
        assert target_identity is not None
        self._target_process_lifecycle = target_lifecycle_record(
            target_identity,
            owner_pid=self._owner_pid,
            spawn_before_monotonic_ns=spawn_before_monotonic_ns,
            spawn_after_monotonic_ns=spawn_after_monotonic_ns,
        )
        child_socket.close()
        self._sequence = 0
        self._generation = 0
        ready = self._receive_ack(2.0)
        if (
            ready["kind"] != ACK_READY
            or ready["sequence"] != 0
            or ready["generation"] != 0
            or ready["publish_complete_watermark"]
            != ready["tick_index"]
        ):
            self._raise_after_close(
                AssertionError("isolated_publisher_ready_invalid")
            )
        self.ready = ready
        self._last_ack = ready

    def _raise_after_close(self, primary_error: BaseException) -> None:
        cleanup_failures = []
        try:
            self.close()
        except BaseException as error:
            cleanup_failures.append(
                f"{type(error).__name__}: {error}"
            )
        try:
            setattr(
                primary_error,
                "_isolated_publisher_cleanup_failures",
                cleanup_failures,
            )
            if cleanup_failures:
                primary_error.args = (
                    f"{primary_error}; cleanup_failure="
                    f"{'; '.join(cleanup_failures)}",
                )
        except BaseException:
            pass
        raise primary_error

    def _receive_ack(self, timeout_sec: float) -> dict[str, object]:
        readable, _, _ = select.select(
            (self._parent_socket,),
            (),
            (),
            timeout_sec,
        )
        if not readable:
            self._raise_after_close(
                AssertionError("isolated_publisher_ack_stall")
            )
        packet = self._parent_socket.recv(ACK.size + 1)
        if len(packet) != ACK.size:
            self._raise_after_close(
                AssertionError("isolated_publisher_ack_size_invalid")
            )
        values = ACK.unpack(packet)
        return {
            "kind": values[0],
            "sequence": values[1],
            "generation": values[2],
            "tick_index": values[3],
            "publish_complete_watermark": values[4],
            "stamp_ns": values[5],
            "topic_counts": tuple(values[6:]),
        }

    def command(
        self,
        command: int,
        *,
        armed: bool = False,
        timeout_sec: float = 2.0,
    ) -> dict[str, object]:
        self._sequence += 1
        self._generation += 1
        send_fixed_packet(
            self._parent_socket,
            COMMAND.pack(
                command,
                self._sequence,
                self._generation,
                int(armed),
            ),
            "command",
        )
        ack = self._receive_ack(timeout_sec)
        try:
            validate_ack(
                ack,
                previous_ack=self._last_ack,
                expected_kind=EXPECTED_ACK_KIND[command],
                expected_sequence=self._sequence,
                expected_generation=self._generation,
            )
        except AssertionError as error:
            self._raise_after_close(error)
        self._last_ack = ack
        return ack

    def phase(self, armed: bool) -> dict[str, object]:
        return self.command(COMMAND_PHASE, armed=armed)

    def freeze(self) -> dict[str, object]:
        return self.command(COMMAND_FREEZE)

    def shutdown(self) -> dict[str, object]:
        if self._shutdown_complete:
            return self._last_ack
        if self._closed:
            raise AssertionError("isolated_publisher_already_closed")
        ack = self.command(COMMAND_SHUTDOWN)
        self._cleanup_known_target()
        self._shutdown_complete = True
        return ack

    def _cleanup_known_target(self) -> None:
        if getattr(self, "_target_cleanup_complete", False):
            failure = getattr(self, "_target_cleanup_failure", "")
            if failure:
                raise AssertionError(failure)
            return
        failures = []
        lifecycle = getattr(self, "_target_process_lifecycle", None)
        target_pid = self._process.pid
        identity_sampler = getattr(
            self,
            "_process_identity_sampler",
            process_identity_snapshot,
        )
        descendant_sampler = getattr(
            self,
            "_direct_child_sampler",
            sample_direct_child_identities,
        )
        identity_before_cleanup = identity_sampler(target_pid)
        target_was_present = (
            identity_before_cleanup.get("classification") != "absent"
        )
        descendants = (
            invoke_direct_child_sampler(
                descendant_sampler,
                target_pid,
                "target_descendants_before_stop",
            )
            if target_was_present
            else DirectChildSample()
        )
        descendant_errors = [
            direct_child_sample_failure_text(error)
            for error in descendants.errors
        ]
        if descendant_errors:
            failures.extend(
                f"target_descendant_sample={failure}"
                for failure in descendant_errors
            )
        if descendants:
            failures.append(
                "target_descendants_created "
                f"pids={sorted(descendants)}"
            )
        if lifecycle is not None:
            lifecycle["descendants_before_stop"] = [
                copy.deepcopy(identity)
                for _, identity in sorted(descendants.items())
            ]
            lifecycle["identity_before_cleanup"] = copy.deepcopy(
                identity_before_cleanup
            )
            expected = lifecycle["identity_at_spawn"]
            if target_was_present and not (
                identity_before_cleanup.get("readable") is True
                and identity_before_cleanup.get("pid") == target_pid
                and identity_before_cleanup.get("starttime_ticks")
                == expected.get("starttime_ticks")
                and identity_before_cleanup.get("session_id")
                == expected.get("session_id")
                and identity_before_cleanup.get("ppid")
                == self._owner_pid
            ):
                failures.append("target_identity_before_stop_mismatch")
        try:
            stop_process_bounded(self._process)
        except BaseException as error:
            failures.append(
                f"target_process={type(error).__name__}: {error}"
            )
        descendant_poststop = []
        for pid, identity in sorted(descendants.items()):
            observed = identity_sampler(pid)
            descendant_poststop.append(copy.deepcopy(observed))
            if observed.get("classification") != "absent":
                failures.append(
                    "target_descendant_poststop_present "
                    f"pid={pid}"
                )
        target_poststop = identity_sampler(target_pid)
        if target_poststop.get("classification") != "absent":
            failures.append(
                "target_poststop_not_absent "
                f"identity={target_poststop}"
            )
        if lifecycle is not None:
            lifecycle["descendants_poststop"] = descendant_poststop
            lifecycle["identity_poststop"] = copy.deepcopy(target_poststop)
            lifecycle["alive"] = False
            lifecycle["exitcode"] = self._process.exitcode
            lifecycle["unresolved"] = bool(failures)
            lifecycle["cleanup_stage"] = (
                "absent_no_residue"
                if not failures
                else "cleanup_invalid"
            )
            lifecycle["cleanup_failure"] = "; ".join(failures)
        self._target_cleanup_failure = "; ".join(failures)
        self._target_cleanup_complete = True
        if self._target_cleanup_failure:
            raise AssertionError(self._target_cleanup_failure)

    def _reconcile_associated_children(self) -> None:
        if getattr(self, "_associated_child_cleanup_complete", False):
            if getattr(self, "_associated_child_cleanup_failure", ""):
                raise AssertionError(
                    self._associated_child_cleanup_failure
                )
            return
        records = getattr(self, "_associated_child_lifecycle", [])
        sample_failures = getattr(
            self,
            "_associated_child_sample_failures",
            [],
        )
        reconcile_spawn_associated_child_ledger(
            records,
            owner_pid=getattr(self, "_owner_pid", os.getpid()),
            direct_child_sampler=getattr(
                self,
                "_direct_child_sampler",
                sample_direct_child_identities,
            ),
            waitpid=getattr(self, "_waitpid", os.waitpid),
            sample_failures=sample_failures,
        )
        failures = [
            *sample_failures,
            *(
                (
                    f"pid={record['pid']} "
                    f"{record.get('cleanup_failure')}"
                )
                for record in records
                if record.get("cleanup_failure")
            ),
        ]
        self._associated_child_cleanup_failure = "; ".join(failures)
        self._associated_child_cleanup_complete = True
        if self._associated_child_cleanup_failure:
            raise AssertionError(
                self._associated_child_cleanup_failure
            )

    @property
    def pid(self) -> int:
        return self._process.pid

    @property
    def current_ack(self) -> dict[str, object]:
        return dict(self._last_ack)

    @property
    def process_status(self) -> dict[str, object]:
        return {
            "pid": self._process.pid,
            "role": "isolated_input_publisher",
            "alive": self._process.is_alive(),
            "exitcode": self._process.exitcode,
            "target_process_lifecycle": copy.deepcopy(
                getattr(self, "_target_process_lifecycle", None)
            ),
            "target_cleanup_failure": getattr(
                self,
                "_target_cleanup_failure",
                "",
            ),
            "associated_child_lifecycle": copy.deepcopy(
                getattr(
                    self,
                    "_associated_child_lifecycle",
                    [],
                )
            ),
            "associated_child_cleanup_failure": getattr(
                self,
                "_associated_child_cleanup_failure",
                "",
            ),
            "associated_child_sample_failures": list(
                getattr(
                    self,
                    "_associated_child_sample_failures",
                    [],
                )
            ),
        }

    def assert_alive(self) -> None:
        if self._closed or not self._process.is_alive():
            self.close()
            raise AssertionError("isolated_publisher_process_not_alive")

    def close(self) -> None:
        if self._closed:
            return
        try:
            self._cleanup_known_target()
        finally:
            self._parent_socket.close()
            self._closed = True


if __name__ == "__main__":
    child_entry(sys.argv[1:])
