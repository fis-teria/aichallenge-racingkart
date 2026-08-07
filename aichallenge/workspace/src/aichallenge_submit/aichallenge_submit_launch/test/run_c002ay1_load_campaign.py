#!/usr/bin/env python3
"""Bounded N1-L-N2 load diagnostic for the C002AY1 parity fixture.

This is diagnostic evidence only.  It is deliberately not a race acceptance
runner: the fixture remains the authority for the runtime safety/contract
gates, and this wrapper only records a reproducible scheduler-load condition.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


LABEL = "DIAGNOSTIC_NOT_RACE_ACCEPTANCE"
MAX_FIXTURE_TIMEOUT_SEC = 60.0
DEFAULT_LOAD_DUTY = 0.75
DEFAULT_WARMUP_SEC = 5.0
WORKER_CODE = r'''
import json
import os
from pathlib import Path
import signal
import sys
import time

duty = float(sys.argv[1])
metric_path = Path(sys.argv[2])
running = True

def stop(_signum, _frame):
    global running
    running = False

signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
started = time.monotonic()
try:
    while running:
        cycle_started = time.monotonic()
        busy_until = cycle_started + 0.1 * duty
        while running and time.monotonic() < busy_until:
            pass
        remaining = cycle_started + 0.1 - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)
finally:
    metric_path.write_text(json.dumps({
        "pid": os.getpid(),
        "cpu_sec": time.process_time(),
        "elapsed_sec": time.monotonic() - started,
        "affinity": sorted(os.sched_getaffinity(0)),
    }) + "\n")
'''


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def parse_cpu_list(value: str) -> set[int]:
    cpus: set[int] = set()
    for item in value.split(","):
        bounds = item.strip().split("-", 1)
        try:
            first = int(bounds[0])
            last = int(bounds[-1])
        except ValueError as error:
            raise ValueError(f"invalid CPU list: {value}") from error
        if first < 0 or last < first:
            raise ValueError(f"invalid CPU range: {item}")
        cpus.update(range(first, last + 1))
    if not cpus:
        raise ValueError("CPU list must not be empty")
    return cpus


def cgroup_cpu_stat() -> dict[str, object]:
    """Capture this process's cgroup cpu.stat without assuming cgroup version."""
    try:
        cgroup_lines = Path("/proc/self/cgroup").read_text().splitlines()
    except OSError as error:
        return {"available": False, "error": str(error)}

    candidates: list[Path] = []
    for line in cgroup_lines:
        fields = line.split(":", 2)
        if len(fields) != 3:
            continue
        controllers, relative_path = fields[1], fields[2].lstrip("/")
        if not controllers:
            candidates.append(Path("/sys/fs/cgroup") / relative_path / "cpu.stat")
        elif "cpu" in controllers.split(","):
            candidates.append(
                Path("/sys/fs/cgroup/cpu") / relative_path / "cpu.stat"
            )
    for path in candidates:
        try:
            return {
                "available": True,
                "path": str(path),
                "content": path.read_text(),
            }
        except OSError:
            continue
    return {"available": False, "cgroup": cgroup_lines}


def affinity(pid: int) -> list[int] | None:
    try:
        return sorted(os.sched_getaffinity(pid))
    except (AttributeError, OSError):
        return None


def terminate(process: subprocess.Popen[str], timeout_sec: float = 2.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        process.wait(timeout=1.0)


def run_fixture(
    command: list[str], environment: dict[str, str], stdout_log: Path, timeout_sec: float
) -> tuple[int, bool, list[int] | None]:
    with stdout_log.open("w") as log:
        process = subprocess.Popen(
            command,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        # taskset applies affinity and then execs the fixture in the same PID.
        # Avoid sampling the short pre-exec window.
        time.sleep(0.05)
        process_affinity = affinity(process.pid)
        try:
            return process.wait(timeout=timeout_sec), False, process_affinity
        except subprocess.TimeoutExpired:
            terminate(process)
            return (
                process.returncode if process.returncode is not None else 124,
                True,
                process_affinity,
            )


def start_workers(
    *, count: int, duty: float, cpu_list: str, python: str, run_root: Path
) -> list[tuple[subprocess.Popen[str], Path]]:
    workers = []
    try:
        for index in range(count):
            metric_path = run_root / f"worker-{index:02d}.json"
            stdout_path = run_root / f"worker-{index:02d}.stdout.log"
            log = stdout_path.open("w")
            try:
                process = subprocess.Popen(
                    [
                        "taskset", "-c", cpu_list, python, "-c", WORKER_CODE,
                        str(duty), str(metric_path),
                    ],
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                )
            finally:
                log.close()
            workers.append((process, metric_path))
    except BaseException:
        for process, _ in workers:
            terminate(process)
        raise
    return workers


def collect_workers(
    workers: list[tuple[subprocess.Popen[str], Path]], elapsed_sec: float
) -> dict[str, object]:
    alive_before_stop = [process.poll() is None for process, _ in workers]
    for process, _ in workers:
        terminate(process)
    metrics: list[dict[str, object]] = []
    for index, (process, metric_path) in enumerate(workers):
        metric: dict[str, object] = {
            "pid": process.pid,
            "exit_code": process.returncode,
            "alive_before_stop": alive_before_stop[index],
            "metric_path": str(metric_path),
        }
        try:
            metric.update(json.loads(metric_path.read_text()))
        except (OSError, json.JSONDecodeError) as error:
            metric["metric_error"] = str(error)
        metrics.append(metric)
    cpu_sec = sum(
        float(metric.get("cpu_sec", 0.0))
        for metric in metrics
        if isinstance(metric.get("cpu_sec"), (int, float))
    )
    worker_count = len(workers)
    achieved_duty = (
        cpu_sec / (elapsed_sec * worker_count)
        if elapsed_sec > 0.0 and worker_count > 0
        else None
    )
    return {
        "workers": metrics,
        "worker_cpu_sec": cpu_sec,
        "achieved_aggregate_duty": achieved_duty,
    }


def production_ledger_summary(
    path: Path, *, minimum_pass_cycles: int
) -> dict[str, object]:
    if not path.is_file():
        return {
            "available": False,
            "valid": False,
            "error": f"missing production ledger: {path}",
        }
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        return {"available": False, "valid": False, "error": str(error)}
    exact = payload.get("exact_availability", {})
    grants = [
        event
        for event in payload.get("events", [])
        if event.get("kind") == "grant"
    ]
    valid_grants = [event for event in grants if bool(event.get("valid"))]
    revokes = [event for event in grants if not bool(event.get("valid"))]
    last_revoke_sequence = max(
        (int(event.get("sequence", -1)) for event in revokes),
        default=-1,
    )
    regrants_after_revoke = [
        event
        for event in valid_grants
        if int(event.get("sequence", -1)) > last_revoke_sequence
    ]
    post_revoke_commands = [
        event
        for event in payload.get("events", [])
        if event.get("kind") == "command"
        and event.get("fixture_phase") == "outage_stop"
    ]
    forbidden_post_revoke_commands = [
        event
        for event in post_revoke_commands
        if (
            float(event.get("speed_mps", float("nan"))) != 0.0
            or float(event.get("acceleration_mps2", float("nan"))) >= 0.0
            or float(event.get("steering_rad", float("nan"))) != 0.0
            or float(event.get("steering_rate_radps", float("nan"))) != 0.0
        )
    ]
    pass_cycles = int(exact.get("pass", {}).get("cycles", 0))
    pass_available = int(
        exact.get("pass", {}).get("available_cycles", 0)
    )
    outage_cycles = int(exact.get("outage_stop", {}).get("cycles", 0))
    outage_available = int(
        exact.get("outage_stop", {}).get("available_cycles", 0)
    )
    valid = (
        pass_cycles >= 1
        and pass_available == pass_cycles
        and outage_cycles >= 2
        and outage_available == 0
        and bool(valid_grants)
        and bool(revokes)
        and not regrants_after_revoke
        and bool(post_revoke_commands)
        and not forbidden_post_revoke_commands
    )
    return {
        "available": True,
        "valid": valid,
        "path": str(path),
        "exact_availability": exact,
        "valid_grant_count": len(valid_grants),
        "revoke_count": len(revokes),
        "regrant_after_revoke_count": len(regrants_after_revoke),
        "post_revoke_command_count": len(post_revoke_commands),
        "forbidden_post_revoke_command_count": len(
            forbidden_post_revoke_commands
        ),
        "transition_summary": payload.get("transition_summary", {}),
    }


def mux_runtime_summary(
    path: Path, minimum_pass_cycles: int
) -> dict[str, object]:
    if not path.is_file():
        return {
            "available": False,
            "valid": False,
            "error": f"missing raw Mux runtime measurement: {path}",
        }
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        return {"available": False, "valid": False, "error": str(error)}
    schema_version = payload.get("schema_version")
    records = payload.get("records", [])
    drops = int(payload.get("drops", -1))
    measurement_faults = int(payload.get("measurement_faults", -1))
    cycle_records_schema_version = payload.get(
        "cycle_records_schema_version"
    )
    cycle_records = payload.get("cycle_records", [])
    shape_valid = all(
        isinstance(record, list)
        and len(record) == 3
        and all(type(value) is int for value in record)
        and int(record[0]) > 0
        and int(record[1]) > 0
        and int(record[2]) >= 0
        for record in records
    )
    sequences = [int(record[0]) for record in records] if shape_valid else []
    starts = [int(record[1]) for record in records] if shape_valid else []
    durations = [int(record[2]) for record in records] if shape_valid else []
    sequence_valid = all(
        current == previous + 1
        for previous, current in zip(sequences, sequences[1:])
    )
    starts_monotonic = all(
        current > previous
        for previous, current in zip(starts, starts[1:])
    )
    gaps = [
        current - previous
        for previous, current in zip(starts, starts[1:])
    ]
    p99_ns = (
        sorted(durations)[
            min(len(durations) - 1, len(durations) * 99 // 100)
        ]
        if durations
        else -1
    )
    cycle_sequences = [
        int(record.get("cycle_sequence", -1))
        for record in cycle_records
        if isinstance(record, dict)
    ]
    cycle_records_one_to_one = bool(
        len(cycle_records) == len(records)
        and cycle_sequences == sequences
    )
    incomplete_cycle_indices = [
        index
        for index, record in enumerate(cycle_records)
        if isinstance(record, dict)
        and not bool(record.get("callback_completed"))
    ]
    trailing_shutdown_incomplete_valid = (
        not incomplete_cycle_indices
        or incomplete_cycle_indices == [len(cycle_records) - 1]
    )
    def is_sha256_hex(value: object) -> bool:
        return (
            isinstance(value, str)
            and len(value) == 64
            and value != "0" * 64
            and all(character in "0123456789abcdef" for character in value)
        )

    def v2_record_shape_valid(record: object) -> bool:
        if not isinstance(record, dict) or not bool(record.get("callback_completed")):
            return True
        pp_identity = record.get("selected_pp_identity")
        plan_identity = record.get("selected_plan_sample_identity")
        limiter = record.get("limiter")
        grant = record.get("grant")
        if not all(
            isinstance(value, dict)
            for value in (pp_identity, plan_identity, limiter, grant)
        ):
            return False
        if (
            not is_sha256_hex(record.get("final_command_canonical_sha256"))
            or not is_sha256_hex(record.get("final_command_cdr_sha256"))
            or record.get("selected_input_command_canonical_sha256") is not None
            and not is_sha256_hex(record.get("selected_input_command_canonical_sha256"))
        ):
            return False
        pp_fields = (
            "producer_instance_id",
            "command_sequence",
            "command_stamp_ns",
            "plan_generation",
        )
        if not isinstance(pp_identity.get("present"), bool) or any(
            field not in pp_identity for field in pp_fields
        ):
            return False
        if bool(pp_identity["present"]):
            if not all(type(pp_identity[field]) is int for field in pp_fields):
                return False
        elif any(pp_identity[field] is not None for field in pp_fields):
            return False
        plan_fields = (
            "race_arm_epoch",
            "planner_instance_id",
            "attempt_id",
            "target_vehicle_id",
            "pass_direction",
            "connector_transaction_id",
            "plan_stamp_ns",
            "plan_generation",
            "phase",
            "candidate_revision",
            "candidate_content_sha256",
            "identity_valid",
        )
        if not isinstance(plan_identity.get("present"), bool) or any(
            field not in plan_identity for field in plan_fields
        ):
            return False
        if bool(plan_identity["present"]):
            if (
                not all(
                    type(plan_identity[field]) is int
                    for field in (
                        "race_arm_epoch",
                        "planner_instance_id",
                        "attempt_id",
                        "pass_direction",
                        "connector_transaction_id",
                        "plan_stamp_ns",
                        "plan_generation",
                        "phase",
                        "candidate_revision",
                    )
                )
                or not isinstance(plan_identity["target_vehicle_id"], str)
                or not is_sha256_hex(
                    plan_identity["candidate_content_sha256"]
                )
                or not isinstance(plan_identity["identity_valid"], bool)
            ):
                return False
        elif any(plan_identity[field] is not None for field in plan_fields):
            return False
        limiter_fields = (
            "raw_steering_rad",
            "limited_steering_rad",
            "steering_delta_rad",
            "angle_limited",
            "rate_limited",
            "reset",
            "enabled",
            "max_steering_angle_rad",
            "max_steering_rate_radps",
            "max_steering_delta_per_cycle_rad",
            "reset_dt_threshold_sec",
            "reset_on_mode_change",
        )
        if any(field not in limiter for field in limiter_fields):
            return False
        if not all(
            isinstance(limiter[field], bool)
            for field in (
                "angle_limited",
                "rate_limited",
                "reset",
                "enabled",
                "reset_on_mode_change",
            )
        ):
            return False
        grant_fields = (
            "present",
            "issuer_instance_id",
            "sequence",
            "valid",
            "commit_steady_ns",
            "final_command_cdr_sha256",
        )
        if any(field not in grant for field in grant_fields):
            return False
        if not isinstance(grant["present"], bool):
            return False
        if not bool(grant["present"]):
            return all(grant[field] is None for field in grant_fields[1:])
        if (
            type(grant["issuer_instance_id"]) is not int
            or type(grant["sequence"]) is not int
            or not isinstance(grant["valid"], bool)
        ):
            return False
        if bool(grant["valid"]):
            return bool(
                type(grant["commit_steady_ns"]) is int
                and int(grant["commit_steady_ns"]) >= 0
                and is_sha256_hex(grant["final_command_cdr_sha256"])
            )
        return bool(
            grant["commit_steady_ns"] is None
            and grant["final_command_cdr_sha256"] is None
        )

    v2_cycle_records_valid = all(
        v2_record_shape_valid(record) for record in cycle_records
    )
    pass_records = [
        record
        for record in cycle_records
        if isinstance(record, dict)
        and bool(record.get("callback_completed"))
        and bool(record.get("exact_available"))
        and record.get("source") == "pure_pursuit"
        and bool(record.get("grant_published"))
        and bool(record.get("grant_valid"))
        and int(record.get("authority_generation", -1)) == 2
        and int(record.get("tracking_generation", -1)) == 2
        and float(record.get("command", [0.0])[0]) > 0.0
        and record.get("grant_command") == record.get("command")
        and isinstance(record.get("selected_pp_identity"), dict)
        and isinstance(record.get("selected_plan_sample_identity"), dict)
        and isinstance(record.get("grant"), dict)
        and bool(record["selected_pp_identity"].get("present"))
        and bool(record["selected_plan_sample_identity"].get("present"))
        and record["selected_plan_sample_identity"].get("identity_valid") is True
        and record["selected_pp_identity"].get("plan_generation")
        == record["selected_plan_sample_identity"].get("plan_generation")
        == record.get("authority_generation")
        == record.get("tracking_generation")
        and is_sha256_hex(
            record.get("selected_input_command_canonical_sha256")
        )
        and bool(record["grant"].get("present"))
        and record["grant"].get("valid") is True
        and record["grant"].get("sequence") == record.get("grant_sequence")
        and record["grant"].get("final_command_cdr_sha256")
        == record.get("final_command_cdr_sha256")
    ]
    pass_sequences = [
        int(record["cycle_sequence"]) for record in pass_records
    ]
    pass_contiguous = all(
        current == previous + 1
        for previous, current in zip(
            pass_sequences, pass_sequences[1:]
        )
    )
    pass_grant_sequences = [
        int(record["grant"]["sequence"])
        for record in pass_records
    ]
    pass_grants_contiguous = all(
        current == previous + 1
        for previous, current in zip(
            pass_grant_sequences, pass_grant_sequences[1:]
        )
    )
    revoke_records = [
        record
        for record in cycle_records
        if isinstance(record, dict)
        and bool(record.get("grant_published"))
        and not bool(record.get("grant_valid"))
        and isinstance(record.get("grant"), dict)
        and isinstance(record.get("selected_plan_sample_identity"), dict)
        and bool(record["grant"].get("present"))
        and record["grant"].get("valid") is False
        and record["grant"].get("sequence") == record.get("grant_sequence")
        and not bool(record["selected_plan_sample_identity"].get("present"))
        and record.get("grant_reason")
        == "positive_pass_motion_not_requested"
    ]
    revoke_sequence = (
        int(revoke_records[0]["cycle_sequence"])
        if revoke_records
        else -1
    )
    revoke_grant_sequence_valid = bool(
        revoke_records
        and pass_grant_sequences
        and int(revoke_records[0].get("grant_sequence", -1))
        == pass_grant_sequences[-1] + 1
    )
    post_revoke_records = [
        record
        for record in cycle_records
        if isinstance(record, dict)
        and bool(record.get("callback_completed"))
        and int(record.get("cycle_sequence", -1)) >= revoke_sequence
    ] if revoke_sequence >= 0 else []
    post_revoke_valid = bool(post_revoke_records) and all(
        not bool(record.get("exact_available"))
        and not bool(record.get("grant_valid"))
        and (
            [float(value) for value in record.get("command", [])]
            == [0.0, -1.5, 0.0, 0.0]
        )
        for record in post_revoke_records
    )
    valid = (
        shape_valid
        and schema_version == 1
        and len(records) >= 100
        and bool(sequences)
        and sequences[0] == 1
        and sequences[-1] - sequences[0] + 1 == len(sequences)
        and drops == 0
        and measurement_faults == 0
        and cycle_records_schema_version == 2
        and sequence_valid
        and starts_monotonic
        and p99_ns <= 10_000_000
        and max(durations, default=100_000_001) <= 100_000_000
        and max(gaps, default=100_000_001) <= 100_000_000
        and cycle_records_one_to_one
        and trailing_shutdown_incomplete_valid
        and v2_cycle_records_valid
        and len(pass_records) >= minimum_pass_cycles
        and pass_contiguous
        and pass_grants_contiguous
        and len(revoke_records) == 1
        and revoke_grant_sequence_valid
        and post_revoke_valid
    )
    return {
        "available": True,
        "valid": valid,
        "path": str(path),
        "sha256": sha256(path),
        "schema_version": schema_version,
        "record_fields": [
            "sequence",
            "started_steady_ns",
            "duration_ns",
        ],
        "record_count": len(records),
        "first_sequence": sequences[0] if sequences else None,
        "last_sequence": sequences[-1] if sequences else None,
        "drops": drops,
        "measurement_faults": measurement_faults,
        "cycle_records_schema_version": cycle_records_schema_version,
        "cycle_records_v2_valid": v2_cycle_records_valid,
        "sequence_strict_plus_one": sequence_valid,
        "steady_start_strictly_monotonic": starts_monotonic,
        "p99_ns": p99_ns,
        "max_duration_ns": max(durations, default=-1),
        "max_gap_ns": max(gaps, default=-1),
        "cycle_records_one_to_one": cycle_records_one_to_one,
        "incomplete_cycle_count": len(incomplete_cycle_indices),
        "trailing_shutdown_incomplete_valid": (
            trailing_shutdown_incomplete_valid
        ),
        "same_callback_pass_count": len(pass_records),
        "same_callback_pass_contiguous": pass_contiguous,
        "same_callback_grants_contiguous": pass_grants_contiguous,
        "same_callback_revoke_count": len(revoke_records),
        "same_callback_revoke_grant_sequence_valid": (
            revoke_grant_sequence_valid
        ),
        "same_callback_post_revoke_count": len(post_revoke_records),
        "same_callback_post_revoke_valid": post_revoke_valid,
    }


def first_failure_line(output: str) -> str:
    lines = [line.strip() for line in output.splitlines() if line.strip()]
    for line in reversed(lines):
        if "AssertionError:" in line or "Traceback " in line:
            return line
    return lines[-1] if lines else ""


def successor_budget_summary(
    fixture_path: Path, mux_runtime_path: Path
) -> dict[str, object]:
    if not fixture_path.is_file():
        return {
            "available": False,
            "complete": False,
            "error": f"missing successor fixture timing: {fixture_path}",
        }
    try:
        fixture = json.loads(fixture_path.read_text())
        mux = (
            json.loads(mux_runtime_path.read_text())
            if mux_runtime_path.is_file()
            else {}
        )
    except (OSError, json.JSONDecodeError) as error:
        return {"available": False, "complete": False, "error": str(error)}
    publish_started_ns = int(fixture.get("publish_started_ns", -1))
    publish_completed_ns = int(fixture.get("publish_completed_ns", -1))
    cycles = [
        record
        for record in mux.get("cycle_records", [])
        if isinstance(record, dict)
        and bool(record.get("callback_completed"))
        and int(record.get("started_steady_ns", -1)) >= publish_started_ns
    ]
    successor_cycles = [
        record
        for record in cycles
        if int(record.get("authority_generation", -1)) == 2
        and int(record.get("tracking_generation", -1)) == 2
    ]
    first_successor = successor_cycles[0] if successor_cycles else None
    first_exact = next(
        (
            record
            for record in successor_cycles
            if bool(record.get("exact_available"))
            and bool(record.get("grant_valid"))
        ),
        None,
    )
    first_receipts = (
        {
            key: int(value)
            for key, value in first_successor.get(
                "input_receipt_steady_ns", {}
            ).items()
        }
        if first_successor is not None
        else {}
    )
    ready_receipts = (
        {
            key: int(value)
            for key, value in first_exact.get(
                "input_receipt_steady_ns", {}
            ).items()
        }
        if first_exact is not None
        else {}
    )
    required_ready_values = [
        value
        for key, value in ready_receipts.items()
        if key
        in {
            "overtake_plan",
            "safety_constraint",
            "pure_pursuit_command",
            "pure_pursuit_tracking",
            "pure_pursuit_envelope",
        }
        and value >= publish_started_ns
    ]
    exact_inputs_ready_ns = (
        max(required_ready_values) if len(required_ready_values) == 5 else -1
    )
    exact_cycle_started_ns = (
        int(first_exact.get("started_steady_ns", -1))
        if first_exact is not None
        else -1
    )
    exact_cycle_completed_ns = (
        exact_cycle_started_ns + int(first_exact.get("duration_ns", 0))
        if first_exact is not None
        else -1
    )
    grant_commit_steady_ns = (
        int(first_exact.get("grant_commit_steady_ns", -1))
        if first_exact is not None
        else -1
    )
    total_to_commit_ns = (
        grant_commit_steady_ns - publish_started_ns
        if grant_commit_steady_ns >= publish_started_ns >= 0
        else -1
    )
    order_valid = bool(
        publish_started_ns >= 0
        and publish_completed_ns >= publish_started_ns
        and exact_inputs_ready_ns >= publish_started_ns
        and exact_cycle_started_ns >= exact_inputs_ready_ns
        and grant_commit_steady_ns >= exact_cycle_started_ns
        and exact_cycle_completed_ns >= grant_commit_steady_ns
    )
    observer_events = fixture.get("observer_events", {})
    free_run_ack_event_count = len(observer_events.get("pp_ack", []))
    return {
        "available": True,
        "complete": first_exact is not None,
        "valid": bool(
            first_exact is not None
            and order_valid
            and free_run_ack_event_count == 0
            and fixture.get("status") == "authority_committed"
            and not fixture.get("failure")
            and 0 <= total_to_commit_ns <= 200_000_000
        ),
        "order_valid": order_valid,
        "pass_free_run_ack_expected": False,
        "pass_free_run_ack_event_count": free_run_ack_event_count,
        "fixture_path": str(fixture_path),
        "mux_runtime_path": str(mux_runtime_path),
        "status": fixture.get("status", ""),
        "failure": fixture.get("failure", ""),
        "publish_started_ns": publish_started_ns,
        "publish_completed_ns": publish_completed_ns,
        "fixture_publish_duration_ns": (
            publish_completed_ns - publish_started_ns
        ),
        "first_successor_cycle_sequence": (
            int(first_successor.get("cycle_sequence", -1))
            if first_successor is not None
            else -1
        ),
        "first_successor_input_receipts_ns": first_receipts,
        "exact_cycle_sequence": (
            int(first_exact.get("cycle_sequence", -1))
            if first_exact is not None
            else -1
        ),
        "exact_input_receipts_ns": ready_receipts,
        "exact_inputs_ready_ns": exact_inputs_ready_ns,
        "exact_cycle_started_ns": exact_cycle_started_ns,
        "exact_cycle_completed_ns": exact_cycle_completed_ns,
        "grant_commit_steady_ns": grant_commit_steady_ns,
        "mux_schedule_wait_ns": (
            exact_cycle_started_ns - exact_inputs_ready_ns
            if exact_cycle_started_ns >= exact_inputs_ready_ns >= 0
            else -1
        ),
        "total_to_commit_upper_bound_ns": total_to_commit_ns,
        "production_budget_ns": 200_000_000,
        "production_budget_slack_ns": (
            200_000_000 - total_to_commit_ns
            if total_to_commit_ns >= 0
            else -1
        ),
        "last_cycle_sequence": (
            int(cycles[-1].get("cycle_sequence", -1)) if cycles else -1
        ),
        "last_cycle_reason": (
            str(cycles[-1].get("reason", "")) if cycles else ""
        ),
        "observer_events": observer_events,
    }


def run_one(
    *, name: str, domain_id: int, artifact_root: Path, fixture: Path,
    python: str, cpu_list: str, fixture_timeout_sec: float, worker_count: int,
    load_duty: float, load_min_duty: float, load_max_duty: float,
    production_pass_cycles: int,
) -> dict[str, object]:
    run_root = artifact_root / name
    if run_root.exists():
        raise FileExistsError(f"refusing to overwrite {run_root}")
    run_root.mkdir()
    ros_home = run_root / "ros-home"
    ros_log_dir = run_root / "ros-logs"
    ros_home.mkdir()
    ros_log_dir.mkdir()
    environment = os.environ.copy()
    environment.update({
        "ROS_DOMAIN_ID": str(domain_id),
        "ROS_LOCALHOST_ONLY": "1",
        "ROS_HOME": str(ros_home),
        "ROS_LOG_DIR": str(ros_log_dir),
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "CYCLONEDDS_URI": "",
        "C002AY1_PRODUCTION_ARTIFACT_DIR": str(
            run_root / "production-profile"
        ),
        "C002AY1_PRODUCTION_ONLY": "1",
        "C002AY1_PRODUCTION_PASS_CYCLES": str(production_pass_cycles),
    })
    environment.pop("ROS_AUTOMATIC_DISCOVERY_RANGE", None)
    workers: list[tuple[subprocess.Popen[str], Path]] = []
    started_monotonic = time.monotonic()
    fixture_exit = 125
    fixture_timed_out = False
    fixture_affinity: list[int] | None = None
    execution_error = ""
    cgroup_before = cgroup_cpu_stat()
    try:
        if name == "L":
            workers = start_workers(
                count=worker_count, duty=load_duty, cpu_list=cpu_list,
                python=python, run_root=run_root,
            )
            time.sleep(DEFAULT_WARMUP_SEC)
        fixture_command = ["taskset", "-c", cpu_list, python, str(fixture)]
        fixture_exit, fixture_timed_out, fixture_affinity = run_fixture(
            fixture_command, environment, run_root / "stdout.log", fixture_timeout_sec
        )
    except Exception as error:
        execution_error = f"{type(error).__name__}: {error}"
    finally:
        elapsed_sec = time.monotonic() - started_monotonic
        worker_summary = collect_workers(workers, elapsed_sec)
        cgroup_after = cgroup_cpu_stat()
    stdout_log = run_root / "stdout.log"
    fixture_output = (
        stdout_log.read_text(errors="replace") if stdout_log.exists() else ""
    )
    ledger_path = (
        run_root
        / "production-profile"
        / "c002ay1-production-profile-ledger.json"
    )
    production_ledger = production_ledger_summary(
        ledger_path, minimum_pass_cycles=production_pass_cycles
    )
    mux_runtime = mux_runtime_summary(
        run_root / "production-profile" / "mux-runtime.json",
        minimum_pass_cycles=production_pass_cycles,
    )
    successor_budget = successor_budget_summary(
        run_root
        / "production-profile"
        / "successor-budget-fixture.json",
        run_root / "production-profile" / "mux-runtime.json",
    )
    achieved_duty = worker_summary["achieved_aggregate_duty"]
    load_valid = name != "L" or (
        isinstance(achieved_duty, float)
        and load_min_duty <= achieved_duty <= load_max_duty
        and all("cpu_sec" in item for item in worker_summary["workers"])
        and all(
            bool(item.get("alive_before_stop"))
            for item in worker_summary["workers"]
        )
    )
    passed = (
        fixture_exit == 0
        and not fixture_timed_out
        and not execution_error
        and bool(production_ledger["valid"])
        and bool(mux_runtime["valid"])
        and bool(successor_budget.get("valid", False))
    )
    first_false = ""
    if not passed:
        first_false = first_failure_line(fixture_output)
        if not first_false and not bool(production_ledger["valid"]):
            first_false = str(
                production_ledger.get("error", "production ledger hard gate")
            )
        if not first_false and not bool(mux_runtime["valid"]):
            first_false = str(
                mux_runtime.get("error", "Mux runtime hard gate")
            )
        if not first_false and not bool(
            successor_budget.get("valid", False)
        ):
            first_false = str(
                successor_budget.get(
                    "error", "successor budget hard gate"
                )
            )
    elif not load_valid:
        first_false = "load validity hard gate"
    result: dict[str, object] = {
        "label": LABEL,
        "name": name,
        "domain_id": domain_id,
        "cpu_list": cpu_list,
        "runner_affinity": affinity(os.getpid()),
        "fixture_affinity_supporting_evidence": fixture_affinity,
        "ros_home": str(ros_home),
        "ros_log_dir": str(ros_log_dir),
        "fixture_command": ["taskset", "-c", cpu_list, python, str(fixture)],
        "fixture_exit_code": fixture_exit,
        "fixture_timed_out": fixture_timed_out,
        "fixture_stdout_log": str(stdout_log),
        "fixture_output": fixture_output,
        "first_false": first_false,
        "production_ledger": production_ledger,
        "mux_runtime": mux_runtime,
        "successor_budget": successor_budget,
        "execution_error": execution_error,
        "elapsed_sec": elapsed_sec,
        "cgroup_cpu_stat_before": cgroup_before,
        "cgroup_cpu_stat_after": cgroup_after,
        "load": {
            "enabled": name == "L",
            "requested_duty": load_duty if name == "L" else 0.0,
            "warmup_sec": DEFAULT_WARMUP_SEC if name == "L" else 0.0,
            "valid_duty_band": [load_min_duty, load_max_duty],
            **worker_summary,
            "valid": load_valid,
        },
        "passed": passed,
        "valid": passed and load_valid,
    }
    write_json(run_root / "result.json", result)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument(
        "--fixture", type=Path,
        default=Path(__file__).with_name("test_c002ay1_runtime_graph_output_parity.py"),
    )
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--cpu-list", default="0-11")
    parser.add_argument("--n1-domain-id", type=int, default=211)
    parser.add_argument("--load-domain-id", type=int, default=212)
    parser.add_argument("--n2-domain-id", type=int, default=213)
    parser.add_argument("--fixture-timeout-sec", type=float, default=MAX_FIXTURE_TIMEOUT_SEC)
    parser.add_argument("--worker-count", type=int, default=12)
    parser.add_argument("--load-duty", type=float, default=DEFAULT_LOAD_DUTY)
    parser.add_argument("--load-min-duty", type=float, default=0.65)
    parser.add_argument("--load-max-duty", type=float, default=0.90)
    parser.add_argument("--production-pass-cycles", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    domains = (args.n1_domain_id, args.load_domain_id, args.n2_domain_id)
    if len(set(domains)) != 3 or any(domain < 0 or domain > 232 for domain in domains):
        raise ValueError("N1, L, and N2 need distinct ROS domain IDs in [0, 232]")
    if args.artifact_root.exists():
        raise FileExistsError(f"refusing to overwrite artifact root {args.artifact_root}")
    if not args.fixture.is_file():
        raise FileNotFoundError(args.fixture)
    if not 0.0 < args.fixture_timeout_sec <= MAX_FIXTURE_TIMEOUT_SEC:
        raise ValueError(f"--fixture-timeout-sec must be in (0, {MAX_FIXTURE_TIMEOUT_SEC}]")
    if args.worker_count < 1:
        raise ValueError("--worker-count must be positive")
    requested_cpus = parse_cpu_list(args.cpu_list)
    allowed_cpus = set(os.sched_getaffinity(0))
    if not requested_cpus <= allowed_cpus:
        raise ValueError(
            f"CPU list {sorted(requested_cpus)} exceeds allowed affinity "
            f"{sorted(allowed_cpus)}"
        )
    if args.worker_count > len(requested_cpus):
        raise ValueError("worker count must not exceed selected CPU count")
    if not 0.0 < args.load_duty <= 0.90:
        raise ValueError("--load-duty must be in (0, 0.90]")
    if not 0.0 <= args.load_min_duty <= args.load_max_duty <= 0.90:
        raise ValueError("invalid load duty band")
    if args.production_pass_cycles != 10:
        raise ValueError(
            "--production-pass-cycles currently supports only the "
            "production-bound 10-cycle cohort"
        )

    args.artifact_root.mkdir(parents=True)
    manifest = {
        "schema": 1,
        "label": LABEL,
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": sha256(Path(__file__)),
        "fixture": str(args.fixture.resolve()),
        "fixture_sha256": sha256(args.fixture),
        "python": args.python,
        "cpu_list": args.cpu_list,
        "domains": {"N1": domains[0], "L": domains[1], "N2": domains[2]},
        "fixture_timeout_sec": args.fixture_timeout_sec,
        "worker_count": args.worker_count,
        "load_duty": args.load_duty,
        "load_valid_duty_band": [args.load_min_duty, args.load_max_duty],
        "production_pass_cycles": args.production_pass_cycles,
    }
    write_json(args.artifact_root / "manifest.json", manifest)

    results: list[dict[str, object]] = []
    stop_reason = ""
    for name, domain_id in (("N1", domains[0]), ("L", domains[1]), ("N2", domains[2])):
        result = run_one(
            name=name, domain_id=domain_id, artifact_root=args.artifact_root,
            fixture=args.fixture, python=args.python, cpu_list=args.cpu_list,
            fixture_timeout_sec=args.fixture_timeout_sec, worker_count=args.worker_count,
            load_duty=args.load_duty, load_min_duty=args.load_min_duty,
            load_max_duty=args.load_max_duty,
            production_pass_cycles=args.production_pass_cycles,
        )
        results.append(result)
        write_json(args.artifact_root / "aggregate.json", results)
        if not bool(result["valid"]):
            stop_reason = f"{name} invalid; no retry and remaining runs not started"
            break

    summary = {
        "label": LABEL,
        "requested_runs": 3,
        "completed_runs": len(results),
        "valid_runs": sum(bool(result["valid"]) for result in results),
        "all_valid": len(results) == 3 and all(bool(result["valid"]) for result in results),
        "stopped": bool(stop_reason),
        "stop_reason": stop_reason,
    }
    write_json(args.artifact_root / "summary.json", summary)
    print(json.dumps(summary, sort_keys=True))
    return 0 if summary["all_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
