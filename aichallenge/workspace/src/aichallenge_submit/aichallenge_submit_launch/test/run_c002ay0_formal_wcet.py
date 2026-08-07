#!/usr/bin/env python3
"""Run the C002AY0 callback-cohort fixture with recorder OFF/ON pairs."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import signal
import subprocess
import sys
import time


RECORDER_TOPICS = (
    "/clock",
    "/ay0/input/kinematics",
    "/ay0/input/v2x",
    "/ay0/input/trajectory",
    "/ay0/input/overtake_plan",
    "/ay0/input/race_armed",
    "/debug/overtake/state_lattice/authorized_cartesian_trajectory",
    "/debug/overtake/state_lattice/source_binding",
)
MARKERS = (
    "C002AY0_PP_BINDING_CALLBACK_COHORT_WCET",
    "C002AY0_PP_BINDING_E2E_STRETCH",
    "C002AY0_PP_BINDING_E2E_LEGACY_TAIL",
    "C002AY0_PP_BINDING_PRODUCTION_OBSERVER_COVERAGE",
    "C002AY0_PP_BINDING_FULL_DELIVERY_E2E",
    "C002AY0_PP_BINDING_OBSERVED_TERMINAL_AUDIT_INTEGRITY",
)
SCHEDULING_SCENARIOS = (
    ("SHARED_CONTROL_0_3", "OFF"),
    ("PP_BOUND_CPU4", "ON"),
    ("PP_BOUND_CPU4", "OFF"),
    ("SHARED_CONTROL_0_3", "ON"),
)
RESERVED_CTEST_DOMAINS = frozenset({221, 222, 223, 224})
ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC = 30.0
ISOLATED_INPUT_PUBLISHER_LAUNCHER_BASELINE = "direct_popen_exec_v1"
DDS_ENVIRONMENT_CONTRACT = {
    "fixture_cyclonedds_uri_present": True,
    "fixture_cyclonedds_uri": "",
    "fixture_verified_before_arm": True,
    "isolated_child_cyclonedds_uri": "",
    "isolated_child_environment_source": (
        "explicit_copy_of_verified_fixture_environment"
    ),
    "isolated_child_preinit_check": "required_before_ready_ack",
    "isolated_child_preinit_verified_by_ready_ack": True,
}


def configure_formal_dds_environment(
    environment: dict[str, str] | os._Environ[str],
) -> dict[str, str]:
    configured = dict(environment)
    configured["CYCLONEDDS_URI"] = ""
    return configured


def require_formal_dds_environment(
    environment: dict[str, str],
) -> None:
    if (
        "CYCLONEDDS_URI" not in environment
        or environment["CYCLONEDDS_URI"] != ""
    ):
        raise RuntimeError(
            "formal fixture CYCLONEDDS_URI must be explicit empty"
        )


def dds_environment_contract_is_exact(value: object) -> bool:
    return value == DDS_ENVIRONMENT_CONTRACT


def isolated_publisher_lifecycle_is_closed(
    record: object,
) -> bool:
    if not isinstance(record, dict):
        return False
    target = record.get("target_process_lifecycle")
    poststop = (
        target.get("identity_poststop")
        if isinstance(target, dict)
        else None
    )
    return (
        isinstance(target, dict)
        and target.get("pid") == record.get("pid")
        and target.get("provenance") == "exact_direct_popen_child_delta"
        and target.get("cleanup_stage") == "absent_no_residue"
        and target.get("alive") is False
        and target.get("exitcode") == 0
        and target.get("unresolved") is False
        and target.get("cleanup_failure") == ""
        and target.get("descendants_before_stop") == []
        and target.get("descendants_poststop") == []
        and isinstance(poststop, dict)
        and poststop.get("pid") == record.get("pid")
        and poststop.get("classification") == "absent"
        and record.get("target_cleanup_failure") == ""
        and record.get("associated_child_lifecycle") == []
        and record.get("associated_child_cleanup_failure") == ""
        and record.get("associated_child_sample_failures") == []
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def package_prefix(package: str) -> Path:
    result = subprocess.run(
        ["ros2", "pkg", "prefix", package],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    return Path(result.stdout.strip())


def runtime_artifact_fingerprints() -> dict[str, dict[str, object]]:
    pp_prefix = package_prefix("simple_pure_pursuit")
    planner_prefix = package_prefix("state_lattice_overtake_planner")
    transport_prefix = package_prefix("overtake_transport_contract")
    paths = {
        "simple_pure_pursuit": (
            pp_prefix / "lib/simple_pure_pursuit/simple_pure_pursuit"
        ),
        "state_lattice_node": (
            planner_prefix
            / "lib/state_lattice_overtake_planner/"
            "state_lattice_overtake_planner_node"
        ),
        "state_lattice_worker": (
            planner_prefix
            / "lib/state_lattice_overtake_planner/"
            "c002ay0_state_lattice_shadow_worker"
        ),
        "snapshot_worker": (
            transport_prefix
            / "lib/overtake_transport_contract/c002ay0_shadow_worker"
        ),
        "state_lattice_params": (
            planner_prefix
            / "share/state_lattice_overtake_planner/config/"
            "state_lattice_overtake_planner.param.yaml"
        ),
        "overtake_permission": (
            planner_prefix
            / "share/state_lattice_overtake_planner/config/"
            "overtake_permission.csv"
        ),
    }
    fingerprints = {}
    for name, path in paths.items():
        resolved = path.resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"{name}: {path} -> {resolved}")
        fingerprints[name] = {
            "path": str(path),
            "resolved_path": str(resolved),
            "bytes": resolved.stat().st_size,
            "sha256": sha256(resolved),
        }
    return fingerprints


def parse_marker_occurrences(output: str) -> dict[str, list[str]]:
    result = {}
    for marker in MARKERS:
        result[marker] = re.findall(
            rf"(?m)^{re.escape(marker)}=(PASS|FAIL)\b",
            output,
        )
    return result


def parse_markers(occurrences: dict[str, list[str]]) -> dict[str, str]:
    result = {}
    for marker, values in occurrences.items():
        result[marker] = (
            values[0]
            if len(values) == 1
            else "MISSING"
            if not values
            else "DUPLICATE"
        )
    return result


def parse_terminal_ledgers(
    output: str,
) -> tuple[dict[str, object], dict[str, int]]:
    ledgers: dict[str, object] = {}
    counts: dict[str, int] = {}
    for line in output.splitlines():
        match = re.match(
            r"^(binding_on_[ab])\b.*\be2e_terminal=(\{.*\})$",
            line,
        )
        if match:
            counts[match.group(1)] = counts.get(match.group(1), 0) + 1
            ledgers[match.group(1)] = json.loads(match.group(2))
    return ledgers, counts


def parse_cpu_list(value: str) -> set[int]:
    cpus: set[int] = set()
    for item in value.split(","):
        bounds = item.strip().split("-", 1)
        try:
            first = int(bounds[0])
            last = int(bounds[-1])
        except (IndexError, ValueError) as error:
            raise ValueError(f"invalid CPU list: {value}") from error
        if first < 0 or last < first:
            raise ValueError(f"invalid CPU range: {item}")
        cpus.update(range(first, last + 1))
    if not cpus:
        raise ValueError("CPU list must not be empty")
    return cpus


def validate_domains(
    domains: tuple[int, ...],
    *,
    expected_count: int,
) -> tuple[int, ...]:
    if len(domains) != expected_count or len(set(domains)) != expected_count:
        raise ValueError(
            f"diagnostic needs {expected_count} distinct ROS domain(s)"
        )
    if any(domain < 0 or domain > 232 for domain in domains):
        raise ValueError("ROS domains must be in [0, 232]")
    if RESERVED_CTEST_DOMAINS.intersection(domains):
        raise ValueError(
            f"ROS domains overlap reserved CTest domains "
            f"{sorted(RESERVED_CTEST_DOMAINS)}"
        )
    return domains


def parse_domains(value: str) -> tuple[int, ...]:
    try:
        domains = tuple(int(item.strip()) for item in value.split(","))
    except ValueError as error:
        raise ValueError(f"invalid ROS domain list: {value}") from error
    return validate_domains(domains, expected_count=4)


def cpu_topology(cpus: set[int]) -> dict[str, object]:
    topology: dict[str, object] = {}
    for cpu in sorted(cpus):
        root = Path(f"/sys/devices/system/cpu/cpu{cpu}")
        online_path = root / "online"
        online = (
            online_path.read_text().strip() != "0"
            if online_path.exists()
            else root.exists()
        )
        siblings_path = root / "topology/thread_siblings_list"
        topology[str(cpu)] = {
            "online": online,
            "thread_siblings_list": (
                siblings_path.read_text().strip()
                if siblings_path.is_file()
                else None
            ),
        }
    return topology


def require_empty_domain(
    domain_id: int,
    ros_log_root: Path,
) -> dict[str, object]:
    domain_log_dir = ros_log_root / f"domain-{domain_id}"
    domain_log_dir.mkdir(parents=True)
    environment = configure_formal_dds_environment(os.environ)
    environment.update(
        {
            "ROS_DOMAIN_ID": str(domain_id),
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_LOG_DIR": str(domain_log_dir),
        }
    )
    require_formal_dds_environment(environment)
    result = subprocess.run(
        ["ros2", "node", "list", "--no-daemon", "--spin-time", "1"],
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    nodes = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if result.returncode != 0 or nodes:
        raise RuntimeError(
            f"ROS domain {domain_id} is not a clean diagnostic domain: "
            f"exit={result.returncode} nodes={nodes} stderr={result.stderr.strip()}"
        )
    return {
        "domain_id": domain_id,
        "exit_code": result.returncode,
        "nodes": nodes,
        "ros_log_dir": str(domain_log_dir),
        "dds_environment_contract": dict(DDS_ENVIRONMENT_CONTRACT),
    }


def process_scheduler_snapshot(pid: int, role: str) -> dict[str, object]:
    tasks = []
    for task_path in sorted(
        Path(f"/proc/{pid}/task").iterdir(),
        key=lambda path: int(path.name),
    ):
        tid = int(task_path.name)
        tasks.append(
            {
                "tid": tid,
                "affinity": sorted(os.sched_getaffinity(tid)),
                "scheduler_policy": os.sched_getscheduler(tid),
                "scheduler_priority": os.sched_getparam(tid).sched_priority,
                "nice": os.getpriority(os.PRIO_PROCESS, tid),
            }
        )
    if not tasks:
        raise RuntimeError(f"{role}: no task IDs for PID {pid}")
    return {
        "role": role,
        "pid": pid,
        "cgroup": cgroup_cpu_stat(pid),
        "tids": tasks,
    }


def cgroup_cpu_stat(pid: int) -> dict[str, object]:
    try:
        cgroup_lines = Path(f"/proc/{pid}/cgroup").read_text().splitlines()
    except OSError as error:
        return {"available": False, "error": str(error)}
    candidates: list[Path] = []
    for line in cgroup_lines:
        fields = line.split(":", 2)
        if len(fields) != 3:
            continue
        controllers, relative_path = fields[1], fields[2].lstrip("/")
        if not controllers:
            candidates.append(
                Path("/sys/fs/cgroup") / relative_path / "cpu.stat"
            )
        elif "cpu" in controllers.split(","):
            candidates.append(
                Path("/sys/fs/cgroup/cpu") / relative_path / "cpu.stat"
            )
    for path in candidates:
        try:
            content = path.read_text()
        except OSError:
            continue
        values = {}
        for line in content.splitlines():
            fields = line.split()
            if len(fields) == 2:
                try:
                    values[fields[0]] = int(fields[1])
                except ValueError:
                    continue
        return {
            "available": True,
            "path": str(path),
            "cgroup": cgroup_lines,
            "values": values,
        }
    return {"available": False, "cgroup": cgroup_lines}


def verify_scheduler_snapshot(
    snapshot: dict[str, object],
    expected_cpus: set[int],
) -> None:
    mismatches = [
        {
            "tid": task["tid"],
            "affinity": task["affinity"],
        }
        for task in snapshot["tids"]
        if set(task["affinity"]) != expected_cpus
    ]
    if mismatches:
        raise RuntimeError(
            f"{snapshot['role']}: affinity mismatch "
            f"expected={sorted(expected_cpus)} actual={mismatches}"
        )
    if not bool(snapshot["cgroup"]["available"]):
        raise RuntimeError(
            f"{snapshot['role']}: cgroup cpu.stat unavailable "
            f"{snapshot['cgroup']}"
        )


def inspect_scheduling_evidence(
    evidence_root: Path,
    profile: str,
    expected_fixture_cpus: set[int],
    expected_pp_cpus: set[int],
    idle_break_diagnostic: bool,
    isolated_input_publisher_diagnostic: bool,
) -> dict[str, object]:
    expected_scenarios = (
        ("off",)
        if isolated_input_publisher_diagnostic
        else ("off", "binding_on_a", "binding_on_b")
    )
    evidence = {}
    errors = []
    signatures: dict[str, set[tuple[int, int, int]]] = {}
    cgroup_paths: set[str] = set()
    for scenario in expected_scenarios:
        path = evidence_root / f"{scenario}.json"
        if not path.is_file():
            errors.append(f"{scenario}: scheduling evidence missing")
            continue
        try:
            item = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            errors.append(f"{scenario}: scheduling evidence unreadable: {error}")
            continue
        evidence[scenario] = item
        collection_status = item.get("collection_status")
        process_collection_completed = (
            isinstance(collection_status, dict)
            and isinstance(
                collection_status.get("process_snapshot"),
                dict,
            )
            and collection_status["process_snapshot"].get("status")
            == "completed"
        )
        reanchor_collection_completed = (
            isinstance(collection_status, dict)
            and isinstance(collection_status.get("reanchor"), dict)
            and collection_status["reanchor"].get("status")
            == "completed"
        )
        if not process_collection_completed:
            errors.append(
                f"{scenario}: process evidence collection not reached "
                f"status={collection_status}"
            )
        if not reanchor_collection_completed:
            errors.append(
                f"{scenario}: reanchor collection not reached "
                f"status={collection_status}"
            )
        if item.get("idle_break_diagnostic") is not idle_break_diagnostic:
            errors.append(
                f"{scenario}: idle-break diagnostic marker mismatch"
            )
        if (
            item.get("isolated_input_publisher_diagnostic")
            is not isolated_input_publisher_diagnostic
        ):
            errors.append(
                f"{scenario}: isolated-input marker mismatch"
            )
        if isolated_input_publisher_diagnostic:
            isolated_markers = item.get(
                "isolated_input_publisher_markers"
            )
            if (
                not isinstance(isolated_markers, dict)
                or isolated_markers.get("diagnostic_only") is not True
                or isolated_markers.get("not_acceptance") is not True
                or isolated_markers.get("m4_wcet_eligible") is not False
                or isolated_markers.get("m4_acceptance_credit") is not False
                or isolated_markers.get("parent_lease_timeout_sec")
                != ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC
                or isolated_markers.get("launcher_diagnostic_baseline")
                != ISOLATED_INPUT_PUBLISHER_LAUNCHER_BASELINE
                or isolated_markers.get("launcher_timing_parity") is not False
            ):
                errors.append(
                    f"{scenario}: isolated-input exclusion marker mismatch"
                )
            if not dds_environment_contract_is_exact(
                item.get("dds_environment_contract")
            ):
                errors.append(
                    f"{scenario}: DDS environment contract mismatch"
                )
        if (
            item.get("diagnostic_only") is not True
            or item.get("not_acceptance") is not True
            or item.get("m4_wcet_eligible") is not False
            or item.get("no_retry") is not True
        ):
            errors.append(f"{scenario}: diagnostic exclusion marker mismatch")
        if item.get("profile") != profile:
            errors.append(
                f"{scenario}: profile={item.get('profile')} expected={profile}"
            )
        if process_collection_completed:
            requested = item.get("requested", {})
            if set(requested.get("fixture_cpus", [])) != expected_fixture_cpus:
                errors.append(f"{scenario}: fixture CPU request mismatch")
            if set(requested.get("pure_pursuit_cpus", [])) != expected_pp_cpus:
                errors.append(f"{scenario}: PP CPU request mismatch")
        if item.get("residue_pids") != []:
            errors.append(
                f"{scenario}: process residue={item.get('residue_pids')}"
            )
        if item.get("scenario_integrity_valid") is not True:
            errors.append(f"{scenario}: scenario integrity invalid")
        lifecycle_records = item.get("process_lifecycle")
        if not isinstance(lifecycle_records, list) or not lifecycle_records:
            errors.append(f"{scenario}: process lifecycle missing")
        else:
            lifecycle_pids: set[int] = set()
            isolated_lifecycle_records = []
            for record in lifecycle_records:
                if not isinstance(record, dict):
                    errors.append(
                        f"{scenario}: process lifecycle record invalid"
                    )
                    continue
                pid = record.get("pid")
                if type(pid) is not int or pid <= 0 or pid in lifecycle_pids:
                    errors.append(
                        f"{scenario}: process lifecycle PID invalid={pid}"
                    )
                    continue
                lifecycle_pids.add(pid)
                if record.get("role") == "isolated_input_publisher":
                    isolated_lifecycle_records.append(record)
                if (
                    not isinstance(record.get("role"), str)
                    or not record["role"]
                    or not isinstance(record.get("provenance"), str)
                    or not record["provenance"]
                    or not isinstance(record.get("cleanup_stage"), str)
                    or not record["cleanup_stage"]
                    or type(record.get("alive")) is not bool
                    or (
                        record.get("exitcode") is not None
                        and type(record.get("exitcode")) is not int
                    )
                    or type(record.get("unresolved")) is not bool
                    or record.get("unresolved") is not record.get("alive")
                    or not isinstance(
                        record.get("identity_before_cleanup"),
                        dict,
                    )
                    or record["identity_before_cleanup"].get("pid") != pid
                    or record["identity_before_cleanup"].get("readable")
                    is not True
                    or type(
                        record["identity_before_cleanup"].get(
                            "starttime_ticks"
                        )
                    )
                    is not int
                    or type(
                        record["identity_before_cleanup"].get("session_id")
                    )
                    is not int
                    or type(
                        record["identity_before_cleanup"].get("ppid")
                    )
                    is not int
                    or not isinstance(
                        record["identity_before_cleanup"].get("state"),
                        str,
                    )
                    or not (
                        record["identity_before_cleanup"].get("cmdline")
                        or record["identity_before_cleanup"].get("comm")
                    )
                    or not isinstance(
                        record["identity_before_cleanup"].get("cgroup"),
                        list,
                    )
                ):
                    errors.append(
                        f"{scenario}: process lifecycle status invalid "
                        f"pid={pid}"
                    )
                if record.get("unresolved") is True:
                    errors.append(
                        f"{scenario}: unresolved process lifecycle pid={pid}"
                    )
            if isolated_input_publisher_diagnostic and (
                len(isolated_lifecycle_records) != 1
                or isolated_lifecycle_records[0].get("cleanup_stage")
                != "closed_no_residue"
                or isolated_lifecycle_records[0].get("alive") is not False
                or isolated_lifecycle_records[0].get("exitcode") != 0
                or isolated_lifecycle_records[0].get("unresolved") is not False
                or not isolated_publisher_lifecycle_is_closed(
                    isolated_lifecycle_records[0]
                )
            ):
                errors.append(
                    f"{scenario}: isolated publisher lifecycle invalid"
                )
        if (
            isolated_input_publisher_diagnostic
            and process_collection_completed
        ):
            phase_acks = item.get("isolated_phase_acks")
            if not isinstance(phase_acks, list) or not phase_acks:
                errors.append(f"{scenario}: isolated phase ACKs missing")
            else:
                previous_ack = None
                for ack in phase_acks:
                    if (
                        not isinstance(ack, dict)
                        or ack.get("kind") != 2
                        or type(ack.get("sequence")) is not int
                        or type(ack.get("generation")) is not int
                        or type(ack.get("tick_index")) is not int
                        or type(ack.get("publish_complete_watermark"))
                        is not int
                        or type(ack.get("stamp_ns")) is not int
                        or ack.get("publish_complete_watermark")
                        != ack.get("tick_index")
                        or not isinstance(
                            ack.get("request_stage"),
                            str,
                        )
                    ):
                        errors.append(
                            f"{scenario}: isolated phase ACK invalid"
                        )
                        break
                    if previous_ack is not None and (
                        ack["sequence"] != previous_ack["sequence"] + 1
                        or ack["generation"]
                        != previous_ack["generation"] + 1
                        or ack["tick_index"] <= previous_ack["tick_index"]
                        or ack["stamp_ns"] <= previous_ack["stamp_ns"]
                    ):
                        errors.append(
                            f"{scenario}: isolated phase ACK nonmonotonic"
                        )
                        break
                    if previous_ack is None and (
                        ack["sequence"] != 1
                        or ack["generation"] != 1
                    ):
                        errors.append(
                            f"{scenario}: isolated initial phase ACK invalid"
                        )
                        break
                    previous_ack = ack
            reanchor = item.get("reanchor_fence", {})
            if reanchor_collection_completed and (
                not isinstance(reanchor, dict)
                or reanchor.get("completed") is not True
                or reanchor.get(
                    "isolated_post_ack_exact_cohort_valid"
                )
                is not True
                or not isinstance(
                    reanchor.get("isolated_phase_ack"),
                    dict,
                )
            ):
                errors.append(
                    f"{scenario}: isolated post-ACK cohort fence invalid"
                )
            elif reanchor_collection_completed:
                fence_ack = reanchor["isolated_phase_ack"]
                fence_identity = tuple(
                    fence_ack.get(key)
                    for key in (
                        "kind",
                        "sequence",
                        "generation",
                        "tick_index",
                        "publish_complete_watermark",
                        "stamp_ns",
                    )
                )
                known_identities = (
                    {
                        tuple(
                            ack.get(key)
                            for key in (
                                "kind",
                                "sequence",
                                "generation",
                                "tick_index",
                                "publish_complete_watermark",
                                "stamp_ns",
                            )
                        )
                        for ack in phase_acks
                        if isinstance(ack, dict)
                    }
                    if isinstance(phase_acks, list)
                    else set()
                )
                if (
                    fence_identity not in known_identities
                    or type(fence_ack.get("stamp_ns")) is not int
                    or type(reanchor.get("selected_anchor_stamp_ns"))
                    is not int
                    or reanchor["selected_anchor_stamp_ns"]
                    <= fence_ack["stamp_ns"]
                ):
                    errors.append(
                        f"{scenario}: isolated ACK/cohort provenance mismatch"
                    )
        if process_collection_completed:
            throttle_delta = item.get("cgroup_throttle_delta", {})
            if any(
                int(throttle_delta.get(key, 0)) > 0
                for key in ("nr_throttled", "throttled_usec")
            ):
                errors.append(
                    f"{scenario}: CPU throttling increased={throttle_delta}"
                )
        processes = (
            list(item.get("processes", []))
            + list(item.get("final_processes", []))
            if process_collection_completed
            else []
        )
        for process in processes:
            role = str(process.get("role", ""))
            cgroup = process.get("cgroup", {})
            if not bool(cgroup.get("available")):
                errors.append(f"{scenario}: {role} cgroup unavailable")
            else:
                cgroup_paths.add(str(cgroup.get("path")))
                if cgroup.get("path") != item.get(
                    "cgroup_before", {}
                ).get("path"):
                    errors.append(
                        f"{scenario}: {role} cgroup path does not match "
                        "monitored cpu.stat"
                    )
            expected = (
                expected_pp_cpus
                if role.startswith("pure_pursuit")
                else expected_fixture_cpus
            )
            for task in process.get("tids", []):
                if set(task.get("affinity", [])) != expected:
                    errors.append(
                        f"{scenario}: {role} TID {task.get('tid')} "
                        f"affinity={task.get('affinity')} expected={sorted(expected)}"
                    )
                signatures.setdefault(role, set()).add(
                    (
                        int(task.get("scheduler_policy", -1)),
                        int(task.get("scheduler_priority", -1)),
                        int(task.get("nice", 999)),
                    )
                )
    return {
        "valid": not errors and set(evidence) == set(expected_scenarios),
        "idle_break_diagnostic": idle_break_diagnostic,
        "isolated_input_publisher_diagnostic": (
            isolated_input_publisher_diagnostic
        ),
        "diagnostic_only": (
            True if isolated_input_publisher_diagnostic else None
        ),
        "not_acceptance": (
            True if isolated_input_publisher_diagnostic else None
        ),
        "m4_wcet_eligible": (
            False if isolated_input_publisher_diagnostic else None
        ),
        "m4_acceptance_credit": (
            False if isolated_input_publisher_diagnostic else None
        ),
        "no_retry": (
            True if isolated_input_publisher_diagnostic else None
        ),
        "parent_contact_lease_timeout_sec": (
            ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC
            if isolated_input_publisher_diagnostic
            else None
        ),
        "errors": errors,
        "evidence": evidence,
        "scheduler_signatures": {
            role: sorted(values) for role, values in sorted(signatures.items())
        },
        "cgroup_paths": sorted(cgroup_paths),
    }


def process_identity_snapshot(
    pid: int,
    *,
    proc_root: Path = Path("/proc"),
) -> dict[str, object]:
    process_root = proc_root / str(pid)
    try:
        content = (process_root / "stat").read_text()
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
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError) as error:
        return {
            "pid": pid,
            "readable": False,
            "classification": "unreadable",
            "error": f"{type(error).__name__}: {error}",
        }
    try:
        cmdline = (
            (process_root / "cmdline")
            .read_bytes()
            .replace(b"\0", b" ")
            .decode(errors="replace")
            .strip()
        )
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        cmdline = ""
    try:
        cgroup = (process_root / "cgroup").read_text().splitlines()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        cgroup = None
    classification = (
        "zombie"
        if state == "Z"
        else "live"
        if state in {"R", "S", "D", "T", "t", "I", "W", "P"}
        else "unknown"
    )
    return {
        "pid": pid,
        "readable": True,
        "classification": classification,
        "state": state,
        "ppid": ppid,
        "sid": session_id,
        "session_id": session_id,
        "starttime": starttime_ticks,
        "starttime_ticks": starttime_ticks,
        "cmdline": cmdline,
        "comm": comm,
        "cgroup": cgroup,
    }


def session_process_snapshots(
    session_id: int,
    *,
    known_identities: dict[int, dict[str, object]] | None = None,
    proc_root: Path = Path("/proc"),
) -> list[dict[str, object]]:
    known = known_identities if known_identities is not None else {}
    matches: list[dict[str, object]] = []
    observed_pids: set[int] = set()
    for stat_path in proc_root.glob("[0-9]*/stat"):
        try:
            pid = int(stat_path.parent.name)
        except ValueError:
            continue
        snapshot = process_identity_snapshot(pid, proc_root=proc_root)
        if (
            snapshot.get("readable") is True
            and snapshot.get("session_id") == session_id
        ):
            observed_pids.add(pid)
            previous = known.get(pid)
            if previous is not None and (
                previous.get("starttime_ticks")
                != snapshot.get("starttime_ticks")
                or previous.get("session_id") != snapshot.get("session_id")
            ):
                snapshot["classification"] = "identity_changed"
                snapshot["previous_identity"] = previous
            else:
                known.setdefault(pid, dict(snapshot))
            matches.append(snapshot)
    for pid, previous in list(known.items()):
        if pid in observed_pids or not (proc_root / str(pid)).exists():
            continue
        snapshot = process_identity_snapshot(pid, proc_root=proc_root)
        if snapshot.get("readable") is not True:
            snapshot["classification"] = "unreadable"
            snapshot["previous_identity"] = previous
            matches.append(snapshot)
        elif (
            snapshot.get("starttime_ticks")
            != previous.get("starttime_ticks")
            or snapshot.get("session_id") != previous.get("session_id")
        ):
            snapshot["classification"] = "identity_changed"
            snapshot["previous_identity"] = previous
            matches.append(snapshot)
    return sorted(matches, key=lambda item: int(item["pid"]))


def residue_snapshot_pids(
    snapshots: list[dict[str, object]],
) -> list[int]:
    return sorted(
        {
            int(snapshot["pid"])
            for snapshot in snapshots
            if type(snapshot.get("pid")) is int
        }
    )


def wait_session_residue_snapshots(
    session_id: int,
    timeout_sec: float,
    *,
    initial_identity: dict[str, object] | None = None,
) -> list[dict[str, object]]:
    known_identities = (
        {session_id: dict(initial_identity)}
        if initial_identity is not None
        else {}
    )
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        residue = session_process_snapshots(
            session_id,
            known_identities=known_identities,
        )
        if not residue:
            return []
        time.sleep(0.01)
    return session_process_snapshots(
        session_id,
        known_identities=known_identities,
    )


def wait_session_empty(session_id: int, timeout_sec: float) -> list[int]:
    return residue_snapshot_pids(
        wait_session_residue_snapshots(session_id, timeout_sec)
    )


def stop_process_group(
    process: subprocess.Popen[str],
    first_signal: signal.Signals,
    timeout_sec: float,
) -> bool:
    if process.poll() is not None:
        return True
    os.killpg(process.pid, first_signal)
    try:
        process.wait(timeout=timeout_sec)
        return True
    except subprocess.TimeoutExpired:
        return False


def run_fixture(
    command: list[str],
    environment: dict[str, str],
    log_path: Path,
    timeout_sec: float,
) -> tuple[int, bool, list[int], list[dict[str, object]], int]:
    require_formal_dds_environment(environment)
    with log_path.open("w") as log:
        process = subprocess.Popen(
            command,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        initial_identity = process_identity_snapshot(process.pid)
        try:
            exit_code = process.wait(timeout=timeout_sec)
            residue_snapshots = wait_session_residue_snapshots(
                process.pid,
                1.0,
                initial_identity=initial_identity,
            )
            return (
                exit_code,
                False,
                residue_snapshot_pids(residue_snapshots),
                residue_snapshots,
                process.pid,
            )
        except subprocess.TimeoutExpired:
            stop_process_group(process, signal.SIGINT, 5.0)
            if process.poll() is None:
                stop_process_group(process, signal.SIGTERM, 2.0)
            residue_snapshots = wait_session_residue_snapshots(
                process.pid,
                1.0,
                initial_identity=initial_identity,
            )
            return (
                process.returncode if process.returncode is not None else 124,
                True,
                residue_snapshot_pids(residue_snapshots),
                residue_snapshots,
                process.pid,
            )


def reconcile_fixture_session_processes(
    *,
    fixture_pid: int,
    fixture_exit_code: int,
    fixture_timed_out: bool,
    fixture_session_residue: list[int],
    fixture_session_residue_snapshots: list[dict[str, object]],
    scheduling_evidence: dict[str, object] | None,
) -> dict[str, object]:
    records_by_pid: dict[int, dict[str, object]] = {
        fixture_pid: {
            "pid": fixture_pid,
            "role": "fixture_session_leader",
            "provenance": "formal_runner_subprocess_handle",
            "cleanup_stage": (
                "timeout_signal_sequence_complete"
                if fixture_timed_out
                else "wait_complete"
            ),
            "alive": fixture_pid in fixture_session_residue,
            "exitcode": fixture_exit_code,
            "unresolved": fixture_pid in fixture_session_residue,
        }
    }
    errors: list[str] = []
    snapshot_pids = residue_snapshot_pids(
        fixture_session_residue_snapshots
    )
    if snapshot_pids != sorted(set(fixture_session_residue)):
        errors.append(
            "fixture session residue PID/snapshot mismatch "
            f"pids={fixture_session_residue} snapshots={snapshot_pids}"
        )
    evidence = (
        scheduling_evidence.get("evidence", {})
        if isinstance(scheduling_evidence, dict)
        else {}
    )
    for scenario, item in evidence.items():
        for record in item.get("process_lifecycle", []):
            if not isinstance(record, dict) or type(record.get("pid")) is not int:
                errors.append(f"{scenario}: lifecycle record unavailable")
                continue
            pid = int(record["pid"])
            previous = records_by_pid.get(pid)
            if previous is not None and (
                previous.get("role") != record.get("role")
                or previous.get("provenance") != record.get("provenance")
                or previous.get("alive") != record.get("alive")
                or previous.get("unresolved") != record.get("unresolved")
            ):
                errors.append(
                    f"pid={pid}: conflicting lifecycle record"
                )
                continue
            records_by_pid[pid] = dict(record)
            if bool(record.get("unresolved")):
                errors.append(f"pid={pid}: unresolved lifecycle")
    residue_records = []
    snapshots_by_pid = {
        int(snapshot["pid"]): snapshot
        for snapshot in fixture_session_residue_snapshots
        if type(snapshot.get("pid")) is int
    }
    for pid in fixture_session_residue:
        snapshot = snapshots_by_pid.get(pid)
        record = records_by_pid.get(pid)
        identity = (
            record.get("identity_before_cleanup")
            if isinstance(record, dict)
            else None
        )
        identity_matches = (
            pid == fixture_pid
            or (
                isinstance(identity, dict)
                and isinstance(snapshot, dict)
                and identity.get("starttime_ticks")
                == snapshot.get("starttime_ticks")
                and identity.get("session_id")
                == snapshot.get("session_id")
            )
        )
        if (
            snapshot is None
            or snapshot.get("classification")
            not in ("live", "zombie")
            or record is None
            or record.get("unresolved") is not True
            or not identity_matches
        ):
            errors.append(f"pid={pid}: session residue role unresolved")
            residue_records.append(
                {
                    "pid": pid,
                    "role": None,
                    "provenance": None,
                    "cleanup_stage": None,
                    "historical_record": record,
                    "identity_snapshot": snapshot,
                }
            )
        else:
            residue_records.append(
                {
                    **record,
                    "identity_snapshot": snapshot,
                }
            )
            errors.append(
                f"pid={pid}: unresolved fixture session residue "
                f"classification={snapshot.get('classification')} "
                f"role={record.get('role')}"
            )
    return {
        "valid": not errors and not fixture_session_residue,
        "errors": errors,
        "fixture_session_residue": list(fixture_session_residue),
        "fixture_session_residue_snapshots": (
            fixture_session_residue_snapshots
        ),
        "residue_records": residue_records,
        "known_processes": [
            records_by_pid[pid] for pid in sorted(records_by_pid)
        ],
    }


def finalize_recorder(
    recorder: subprocess.Popen[str],
    timeout_sec: float,
) -> tuple[int | None, bool, list[int]]:
    if recorder.poll() is not None:
        return (
            recorder.returncode,
            False,
            wait_session_empty(recorder.pid, 1.0),
        )
    os.killpg(recorder.pid, signal.SIGINT)
    try:
        return (
            recorder.wait(timeout=timeout_sec),
            False,
            wait_session_empty(recorder.pid, 1.0),
        )
    except subprocess.TimeoutExpired:
        # The formal artifact is invalid once bounded SIGINT finalization fails.
        stop_process_group(recorder, signal.SIGTERM, 2.0)
        return (
            recorder.returncode,
            True,
            wait_session_empty(recorder.pid, 1.0),
        )


def inspect_bag(bag_path: Path, run_root: Path) -> dict[str, object]:
    metadata = bag_path / "metadata.yaml"
    mcap_files = list(bag_path.glob("*.mcap"))
    result: dict[str, object] = {
        "metadata_exists": metadata.is_file(),
        "mcap_files": [
            {"path": path.name, "bytes": path.stat().st_size}
            for path in mcap_files
        ],
        "topic_counts": {},
        "valid": False,
    }
    info = subprocess.run(
        ["ros2", "bag", "info", str(bag_path)],
        check=False,
        capture_output=True,
        text=True,
        timeout=20,
    )
    info_text = info.stdout + info.stderr
    (run_root / "bag_info.txt").write_text(info_text)
    topic_counts = {}
    for topic in RECORDER_TOPICS:
        match = re.search(
            rf"Topic:\s*{re.escape(topic)}\s*\|.*?Count:\s*(\d+)\b",
            info_text,
        )
        topic_counts[topic] = int(match.group(1)) if match else 0
    result["topic_counts"] = topic_counts
    result["bag_info_exit_code"] = info.returncode
    result["valid"] = (
        metadata.is_file()
        and bool(mcap_files)
        and all(path.stat().st_size > 0 for path in mcap_files)
        and info.returncode == 0
        and all(count > 0 for count in topic_counts.values())
    )
    return result


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def configure_idle_break_diagnostic_environment(
    environment: dict[str, str],
    enabled: bool,
) -> dict[str, str]:
    configured = dict(environment)
    configured.pop("C002AY0_IDLE_BREAK_DIAGNOSTIC", None)
    if enabled:
        configured["C002AY0_IDLE_BREAK_DIAGNOSTIC"] = "1"
    return configured


def configure_isolated_input_publisher_environment(
    environment: dict[str, str],
    enabled: bool,
) -> dict[str, str]:
    configured = dict(environment)
    configured.pop(
        "C002AY0_ISOLATED_INPUT_PUBLISHER_DIAGNOSTIC",
        None,
    )
    if enabled:
        configured[
            "C002AY0_ISOLATED_INPUT_PUBLISHER_DIAGNOSTIC"
        ] = "1"
    return configured


def validate_isolated_input_publisher_cli(
    *,
    enabled: bool,
    scheduling_diagnostic: bool,
    single_shared_control_off_domain: int | None,
) -> None:
    if enabled and (
        not scheduling_diagnostic
        or single_shared_control_off_domain is None
    ):
        raise ValueError(
            "--isolated-input-publisher-diagnostic requires "
            "--scheduling-diagnostic and "
            "--single-shared-control-off-domain"
        )


def diagnostic_mode_markers(
    diagnostic_only: bool,
    idle_break_diagnostic: bool,
) -> dict[str, bool]:
    if idle_break_diagnostic and not diagnostic_only:
        raise ValueError("idle-break diagnostic must remain diagnostic-only")
    return {
        "diagnostic_only": diagnostic_only,
        "not_acceptance": diagnostic_only,
        "m4_wcet_eligible": not diagnostic_only,
        "no_retry": diagnostic_only,
        "idle_break_diagnostic": idle_break_diagnostic,
    }


def classify_single_scheduling_result(result: dict[str, object]) -> str:
    evidence = result.get("scheduling_evidence") or {}
    scenario_evidence = evidence.get("evidence") or {}
    isolated_marker = result.get(
        "isolated_input_publisher_diagnostic"
    )
    expected_scenarios = (
        ("off",)
        if isolated_marker is True
        else ("off", "binding_on_a", "binding_on_b")
    )
    present_scenarios = tuple(
        scenario
        for scenario in expected_scenarios
        if scenario in scenario_evidence
    )
    evidence_is_ordered_prefix = (
        bool(present_scenarios)
        and set(scenario_evidence) == set(present_scenarios)
        and present_scenarios
        == expected_scenarios[: len(present_scenarios)]
    )

    def selected_drain(item: dict[str, object]) -> dict[str, object]:
        key = (
            "isolated_observer_drain"
            if result.get("isolated_input_publisher_diagnostic") is True
            else "paced_callback_drain"
        )
        drain = item.get(key)
        return drain if isinstance(drain, dict) else {}

    deadline_misses = {
        scenario: int(
            selected_drain(scenario_evidence[scenario]).get(
                "deadline_miss_count",
                0,
            )
        )
        for scenario in present_scenarios
    }
    deadline_miss_count = sum(deadline_misses.values())
    deadline_is_prefix_terminal = (
        bool(present_scenarios)
        and deadline_miss_count == 1
        and deadline_misses.get(present_scenarios[-1], 0) == 1
    )
    terminal_tick = (
        selected_drain(
            scenario_evidence[present_scenarios[-1]]
        ).get("tick_observation", {}).get("last", {})
        if present_scenarios
        else {}
    )
    zero_wait_terminal_valid = (
        deadline_is_prefix_terminal
        and terminal_tick.get("fatal_phase") == "zero_wait"
        and type(terminal_tick.get("spin_attempts")) is int
        and int(terminal_tick["spin_attempts"]) > 0
    )
    isolated_marker_valid = type(isolated_marker) is bool
    isolated_result_markers_valid = (
        isolated_marker is False
        or (
            result.get("diagnostic_only") is True
            and result.get("not_acceptance") is True
            and result.get("m4_wcet_eligible") is False
            and result.get("m4_acceptance_credit") is False
            and result.get("no_retry") is True
            and result.get("parent_contact_lease_timeout_sec")
            == ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC
            and dds_environment_contract_is_exact(
                result.get("dds_environment_contract")
            )
            and evidence.get("isolated_input_publisher_diagnostic") is True
            and evidence.get("diagnostic_only") is True
            and evidence.get("not_acceptance") is True
            and evidence.get("m4_wcet_eligible") is False
            and evidence.get("m4_acceptance_credit") is False
            and evidence.get("no_retry") is True
            and evidence.get("parent_contact_lease_timeout_sec")
            == ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC
        )
    )

    def lifecycle_is_resolved(item: dict[str, object]) -> bool:
        records = item.get("process_lifecycle")
        return (
            isinstance(records, list)
            and bool(records)
            and all(
                isinstance(record, dict)
                and type(record.get("pid")) is int
                and isinstance(record.get("role"), str)
                and bool(record.get("role"))
                and isinstance(record.get("provenance"), str)
                and bool(record.get("provenance"))
                and isinstance(record.get("cleanup_stage"), str)
                and bool(record.get("cleanup_stage"))
                and record.get("alive") is False
                and record.get("unresolved") is False
                and isinstance(
                    record.get("identity_before_cleanup"),
                    dict,
                )
                and record["identity_before_cleanup"].get("pid")
                == record.get("pid")
                and record["identity_before_cleanup"].get("readable")
                is True
                and type(
                    record["identity_before_cleanup"].get(
                        "starttime_ticks"
                    )
                )
                is int
                and type(
                    record["identity_before_cleanup"].get("session_id")
                )
                is int
                and type(
                    record["identity_before_cleanup"].get("ppid")
                )
                is int
                and isinstance(
                    record["identity_before_cleanup"].get("state"),
                    str,
                )
                and bool(
                    record["identity_before_cleanup"].get("cmdline")
                    or record["identity_before_cleanup"].get("comm")
                )
                and isinstance(
                    record["identity_before_cleanup"].get("cgroup"),
                    list,
                )
                and (
                    record.get("exitcode") is None
                    or type(record.get("exitcode")) is int
                )
                for record in records
            )
        )

    def isolated_fence_is_valid(item: dict[str, object]) -> bool:
        if isolated_marker is False:
            return True
        phase_acks = item.get("isolated_phase_acks")
        reanchor = item.get("reanchor_fence")
        if (
            not isinstance(phase_acks, list)
            or not phase_acks
            or not isinstance(reanchor, dict)
            or reanchor.get("completed") is not True
            or reanchor.get("isolated_post_ack_exact_cohort_valid")
            is not True
            or not isinstance(reanchor.get("isolated_phase_ack"), dict)
        ):
            return False
        fence_ack = reanchor["isolated_phase_ack"]
        ack_identity = (
            fence_ack.get("kind"),
            fence_ack.get("sequence"),
            fence_ack.get("generation"),
            fence_ack.get("tick_index"),
            fence_ack.get("publish_complete_watermark"),
            fence_ack.get("stamp_ns"),
        )
        return any(
            isinstance(ack, dict)
            and (
                ack.get("kind"),
                ack.get("sequence"),
                ack.get("generation"),
                ack.get("tick_index"),
                ack.get("publish_complete_watermark"),
                ack.get("stamp_ns"),
            )
            == ack_identity
            for ack in phase_acks
        )

    def isolated_lifecycle_is_valid(item: dict[str, object]) -> bool:
        if isolated_marker is False:
            return True
        records = item.get("process_lifecycle")
        isolated_records = [
            record
            for record in records
            if isinstance(record, dict)
            and record.get("role") == "isolated_input_publisher"
        ] if isinstance(records, list) else []
        return (
            len(isolated_records) == 1
            and isolated_publisher_lifecycle_is_closed(
                isolated_records[0]
            )
        )

    def collection_is_complete(item: dict[str, object]) -> bool:
        status = item.get("collection_status")
        return (
            isinstance(status, dict)
            and isinstance(status.get("process_snapshot"), dict)
            and status["process_snapshot"].get("status") == "completed"
            and isinstance(status.get("reanchor"), dict)
            and status["reanchor"].get("status") == "completed"
        )

    diagnostic_observation_integrity_valid = (
        isolated_marker_valid
        and isolated_result_markers_valid
        and evidence_is_ordered_prefix
        and all(
            item.get("idle_break_diagnostic") is True
            and item.get("diagnostic_only") is True
            and item.get("not_acceptance") is True
            and item.get("m4_wcet_eligible") is False
            and item.get("no_retry") is True
            and item.get("scenario_integrity_valid") is True
            and item.get("residue_pids") == []
            and lifecycle_is_resolved(item)
            and isolated_fence_is_valid(item)
            and isolated_lifecycle_is_valid(item)
            and (
                isolated_marker is False
                or dds_environment_contract_is_exact(
                    item.get("dds_environment_contract")
                )
            )
            and collection_is_complete(item)
            and item.get("isolated_input_publisher_diagnostic")
            is isolated_marker
            and (
                isolated_marker is False
                or (
                    isinstance(
                        item.get("isolated_input_publisher_markers"),
                        dict,
                    )
                    and item["isolated_input_publisher_markers"].get(
                        "diagnostic_only"
                    )
                    is True
                    and item["isolated_input_publisher_markers"].get(
                        "not_acceptance"
                    )
                    is True
                    and item["isolated_input_publisher_markers"].get(
                        "m4_wcet_eligible"
                    )
                    is False
                    and item["isolated_input_publisher_markers"].get(
                        "m4_acceptance_credit"
                    )
                    is False
                    and item["isolated_input_publisher_markers"].get(
                        "parent_lease_timeout_sec"
                    )
                    == ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC
                    and item["isolated_input_publisher_markers"].get(
                        "launcher_diagnostic_baseline"
                    )
                    == ISOLATED_INPUT_PUBLISHER_LAUNCHER_BASELINE
                    and item["isolated_input_publisher_markers"].get(
                        "launcher_timing_parity"
                    )
                    is False
                )
            )
            and isinstance(selected_drain(item).get("spin_observation"), dict)
            and "integrity_failure_count"
            in selected_drain(item)["spin_observation"]
            and int(
                selected_drain(item)["spin_observation"][
                    "integrity_failure_count"
                ]
            )
            == 0
            and (
                isolated_marker is False
                or (
                    selected_drain(item).get("applicable") is True
                    and selected_drain(item).get("diagnostic_only") is True
                    and selected_drain(item).get("not_acceptance") is True
                    and selected_drain(item).get("m4_wcet_eligible") is False
                    and selected_drain(item).get(
                        "input_release_authority"
                    )
                    == "isolated_input_publisher"
                    and selected_drain(item).get(
                        "input_release_period_sec"
                    )
                    == 0.01
                    and selected_drain(item).get(
                        "input_release_gap_gate_sec"
                    )
                    == 0.012
                    and selected_drain(item).get(
                        "input_release_deadline_applies"
                    )
                    is False
                    and selected_drain(item).get(
                        "observer_processing_in_input_release_deadline"
                    )
                    is False
                    and selected_drain(item).get(
                        "canonical_hash_ledger_deferred"
                    )
                    is False
                    and int(
                        selected_drain(item).get(
                            "saturation_count",
                            -1,
                        )
                    )
                    == 0
                    and int(
                        selected_drain(item).get("step_count", 0)
                    )
                    > 0
                    and int(
                        selected_drain(item).get(
                            "max_callbacks_per_step",
                            -1,
                        )
                    )
                    == 64
                    and 0
                    < int(
                        selected_drain(item).get(
                            "max_spin_attempts",
                            0,
                        )
                    )
                    <= 64
                )
            )
            for item in scenario_evidence.values()
        )
        and (
            zero_wait_terminal_valid
            or (
                deadline_miss_count == 0
                and present_scenarios == expected_scenarios
                and evidence.get("valid") is True
            )
        )
        and result.get("fixture_session_residue") == []
        and isinstance(
            result.get("fixture_session_reconciliation"),
            dict,
        )
        and result["fixture_session_reconciliation"].get("valid") is True
    )
    if not diagnostic_observation_integrity_valid:
        return "INVALID_NOT_INTERPRETABLE"
    if deadline_miss_count > 0:
        return "ZERO_WAIT_OVERRUN_REPRODUCED_DIAGNOSTIC_STOP"
    if not result["integrity_valid"]:
        return "INVALID_NOT_INTERPRETABLE"
    return "NO_ZERO_WAIT_OVERRUN_INCONCLUSIVE_NO_RETRY"


def run_one(
    *,
    condition: str,
    iteration: int,
    domain_id: int,
    artifact_root: Path,
    fixture: Path,
    python: str,
    fixture_cpu_list: str,
    recorder_cpu_list: str,
    pp_cpu_list: str | None,
    scheduling_profile: str | None,
    fixture_timeout_sec: float,
    idle_break_diagnostic: bool = False,
    isolated_input_publisher_diagnostic: bool = False,
) -> dict[str, object]:
    profile_suffix = (
        f"-{scheduling_profile.lower()}" if scheduling_profile is not None else ""
    )
    run_root = (
        artifact_root
        / f"{iteration:02d}-{condition.lower()}{profile_suffix}"
    )
    if run_root.exists():
        raise FileExistsError(f"refusing to overwrite {run_root}")
    run_root.mkdir(parents=True)
    runner_scheduler_before: dict[str, object] | None = None
    if scheduling_profile is not None:
        runner_scheduler_before = process_scheduler_snapshot(
            os.getpid(),
            "formal_runner",
        )
        verify_scheduler_snapshot(
            runner_scheduler_before,
            parse_cpu_list(fixture_cpu_list),
        )
    ros_log_dir = run_root / "ros-logs"
    ros_log_dir.mkdir()
    environment = configure_idle_break_diagnostic_environment(
        os.environ,
        idle_break_diagnostic,
    )
    environment = configure_isolated_input_publisher_environment(
        environment,
        isolated_input_publisher_diagnostic,
    )
    environment = configure_formal_dds_environment(environment)
    environment.update(
        {
            "ROS_DOMAIN_ID": str(domain_id),
            "ROS_LOCALHOST_ONLY": "1",
            "ROS_LOG_DIR": str(ros_log_dir),
            "C002AY0_REAL_STATE_LATTICE_E2E_WCET_ONLY": "1",
        }
    )
    require_formal_dds_environment(environment)
    if condition == "ON":
        environment["C002AY0_REQUIRE_RECORDER_DISCOVERY"] = "1"
        environment["C002AY0_RECORDER_NODE_NAME"] = "rosbag2_recorder"
    if scheduling_profile is not None:
        if pp_cpu_list is None:
            raise ValueError("scheduling profile requires PP CPU list")
        evidence_root = run_root / "scheduling-evidence"
        evidence_root.mkdir()
        environment.update(
            {
                "C002AY0_SCHEDULING_EVIDENCE_REQUIRED": "1",
                "C002AY0_SCHEDULING_EVIDENCE_ROOT": str(evidence_root),
                "C002AY0_EXPECT_FIXTURE_CPU_LIST": fixture_cpu_list,
                "C002AY0_PP_CPU_LIST": pp_cpu_list,
                "C002AY0_SCHEDULING_PROFILE": scheduling_profile,
            }
        )
    if idle_break_diagnostic and (
        scheduling_profile != "SHARED_CONTROL_0_3"
        or condition != "OFF"
    ):
        raise ValueError(
            "idle-break diagnostic requires SHARED_CONTROL_0_3/OFF"
        )
    if isolated_input_publisher_diagnostic and (
        scheduling_profile != "SHARED_CONTROL_0_3"
        or condition != "OFF"
    ):
        raise ValueError(
            "isolated-input diagnostic requires SHARED_CONTROL_0_3/OFF"
        )

    fixture_command = [
        "taskset",
        "-c",
        fixture_cpu_list,
        python,
        str(fixture),
    ]
    recorder: subprocess.Popen[str] | None = None
    recorder_log = None
    bag_path = run_root / "bag"
    child_usage_before = resource.getrusage(resource.RUSAGE_CHILDREN)
    started_at = time.time()
    if condition == "ON":
        recorder_log = (run_root / "recorder.log").open("w")
        recorder = subprocess.Popen(
            [
                "taskset",
                "-c",
                recorder_cpu_list,
                "ros2",
                "bag",
                "record",
                "--storage",
                "mcap",
                "--output",
                str(bag_path),
                *RECORDER_TOPICS,
            ],
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=recorder_log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    fixture_exit = 125
    fixture_timed_out = False
    fixture_pid = -1
    recorder_exit: int | None = None
    recorder_finalize_timeout = False
    fixture_session_residue: list[int] = []
    fixture_session_residue_snapshots: list[dict[str, object]] = []
    recorder_session_residue: list[int] = []
    recorder_scheduler_evidence: dict[str, object] | None = None
    recorder_scheduler_error = ""
    try:
        (
            fixture_exit,
            fixture_timed_out,
            fixture_session_residue,
            fixture_session_residue_snapshots,
            fixture_pid,
        ) = run_fixture(
            fixture_command,
            environment,
            run_root / "fixture.log",
            fixture_timeout_sec,
        )
    finally:
        if recorder is not None:
            if scheduling_profile is not None and recorder.poll() is None:
                try:
                    recorder_scheduler_evidence = process_scheduler_snapshot(
                        recorder.pid,
                        "recorder",
                    )
                    expected_recorder_cpus = parse_cpu_list(recorder_cpu_list)
                    verify_scheduler_snapshot(
                        recorder_scheduler_evidence,
                        expected_recorder_cpus,
                    )
                except (OSError, RuntimeError) as error:
                    recorder_scheduler_error = str(error)
            (
                recorder_exit,
                recorder_finalize_timeout,
                recorder_session_residue,
            ) = finalize_recorder(recorder, 10.0)
        if recorder_log is not None:
            recorder_log.close()

    elapsed_sec = time.time() - started_at
    child_usage_after = resource.getrusage(resource.RUSAGE_CHILDREN)
    child_cpu_sec = (
        child_usage_after.ru_utime
        + child_usage_after.ru_stime
        - child_usage_before.ru_utime
        - child_usage_before.ru_stime
    )
    fixture_output = (run_root / "fixture.log").read_text(errors="replace")
    bag_summary = (
        inspect_bag(bag_path, run_root)
        if condition == "ON" and not recorder_finalize_timeout
        else None
    )
    marker_occurrences = parse_marker_occurrences(fixture_output)
    markers = parse_markers(marker_occurrences)
    terminal_ledgers, terminal_ledger_counts = parse_terminal_ledgers(
        fixture_output
    )
    scheduling_evidence_summary = (
        inspect_scheduling_evidence(
            run_root / "scheduling-evidence",
            scheduling_profile,
            parse_cpu_list(fixture_cpu_list),
            parse_cpu_list(pp_cpu_list),
            idle_break_diagnostic,
            isolated_input_publisher_diagnostic,
        )
        if scheduling_profile is not None and pp_cpu_list is not None
        else None
    )
    fixture_session_reconciliation = (
        reconcile_fixture_session_processes(
            fixture_pid=fixture_pid,
            fixture_exit_code=fixture_exit,
            fixture_timed_out=fixture_timed_out,
            fixture_session_residue=fixture_session_residue,
            fixture_session_residue_snapshots=(
                fixture_session_residue_snapshots
            ),
            scheduling_evidence=scheduling_evidence_summary,
        )
        if scheduling_profile is not None
        else None
    )
    runner_scheduler_after: dict[str, object] | None = None
    scheduling_cgroup_paths: set[str] = set()
    if scheduling_profile is not None:
        runner_scheduler_after = process_scheduler_snapshot(
            os.getpid(),
            "formal_runner",
        )
        verify_scheduler_snapshot(
            runner_scheduler_after,
            parse_cpu_list(fixture_cpu_list),
        )
        scheduling_cgroup_paths.update(
            str(path)
            for path in scheduling_evidence_summary["cgroup_paths"]
        )
        scheduling_cgroup_paths.add(
            str(runner_scheduler_before["cgroup"]["path"])
        )
        scheduling_cgroup_paths.add(
            str(runner_scheduler_after["cgroup"]["path"])
        )
        if recorder_scheduler_evidence is not None:
            scheduling_cgroup_paths.add(
                str(recorder_scheduler_evidence["cgroup"]["path"])
            )
    write_json(run_root / "terminal_ledgers.json", terminal_ledgers)
    result: dict[str, object] = {
        "condition": condition,
        "iteration": iteration,
        "domain_id": domain_id,
        "scheduling_profile": scheduling_profile,
        **diagnostic_mode_markers(
            scheduling_profile is not None,
            idle_break_diagnostic,
        ),
        "isolated_input_publisher_diagnostic": (
            isolated_input_publisher_diagnostic
        ),
        "m4_acceptance_credit": (
            False if isolated_input_publisher_diagnostic else None
        ),
        "parent_contact_lease_timeout_sec": (
            ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC
            if isolated_input_publisher_diagnostic
            else None
        ),
        "dds_environment_contract": (
            dict(DDS_ENVIRONMENT_CONTRACT)
            if isolated_input_publisher_diagnostic
            else None
        ),
        "fixture_cpu_list": fixture_cpu_list,
        "recorder_cpu_list": recorder_cpu_list,
        "pp_cpu_list": pp_cpu_list,
        "ros_log_dir": str(ros_log_dir),
        "started_at_unix_sec": started_at,
        "elapsed_sec": elapsed_sec,
        "fixture_exit_code": fixture_exit,
        "fixture_timed_out": fixture_timed_out,
        "fixture_pid": fixture_pid,
        "fixture_session_residue": fixture_session_residue,
        "fixture_session_residue_snapshots": (
            fixture_session_residue_snapshots
        ),
        "fixture_session_reconciliation": fixture_session_reconciliation,
        "fixture_sha256": sha256(fixture),
        "child_cpu_sec": child_cpu_sec,
        "child_cpu_percent": (
            child_cpu_sec / elapsed_sec * 100.0 if elapsed_sec > 0.0 else None
        ),
        "markers": markers,
        "marker_occurrences": marker_occurrences,
        "terminal_ledgers": terminal_ledgers,
        "terminal_ledger_counts": terminal_ledger_counts,
        "recorder_exit_code": recorder_exit,
        "recorder_finalize_timeout": recorder_finalize_timeout,
        "recorder_session_residue": recorder_session_residue,
        "recorder_scheduler_evidence": recorder_scheduler_evidence,
        "recorder_scheduler_error": recorder_scheduler_error,
        "scheduling_evidence": scheduling_evidence_summary,
        "runner_scheduler_before": runner_scheduler_before,
        "runner_scheduler_after": runner_scheduler_after,
        "scheduling_cgroup_paths": sorted(scheduling_cgroup_paths),
        "bag": bag_summary,
    }
    result["integrity_valid"] = (
        fixture_exit == 0
        and not fixture_timed_out
        and not fixture_session_residue
        and (
            scheduling_profile is None
            or bool(
                fixture_session_reconciliation
                and fixture_session_reconciliation["valid"]
            )
        )
        and marker_occurrences[
            "C002AY0_PP_BINDING_CALLBACK_COHORT_WCET"
        ] == ["PASS"]
        and marker_occurrences[
            "C002AY0_PP_BINDING_OBSERVED_TERMINAL_AUDIT_INTEGRITY"
        ] == ["PASS"]
        and len(
            marker_occurrences["C002AY0_PP_BINDING_E2E_STRETCH"]
        ) == 1
        and set(terminal_ledgers) == {"binding_on_a", "binding_on_b"}
        and terminal_ledger_counts == {"binding_on_a": 1, "binding_on_b": 1}
        and (
            scheduling_profile is None
            or bool(
                scheduling_evidence_summary
                and scheduling_evidence_summary["valid"]
            )
        )
        and (
            condition == "OFF"
            or (
                recorder_exit == 0
                and not recorder_finalize_timeout
                and not recorder_session_residue
                and not recorder_scheduler_error
                and (
                    scheduling_profile is None
                    or recorder_scheduler_evidence is not None
                )
                and bool(bag_summary and bag_summary["valid"])
            )
        )
        and (
            scheduling_profile is None
            or len(scheduling_cgroup_paths) == 1
        )
    )
    result["timing_pass"] = (
        marker_occurrences["C002AY0_PP_BINDING_E2E_STRETCH"] == ["PASS"]
    )
    result["valid"] = bool(
        result["integrity_valid"] and result["timing_pass"]
    )
    write_json(run_root / "result.json", result)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, default=Path(__file__).with_name(
        "c002ay0_pp_runtime_measurement.py"
    ))
    parser.add_argument("--python", default="/usr/bin/python3")
    parser.add_argument("--cpu-list", default="0-11")
    parser.add_argument("--scheduling-diagnostic", action="store_true")
    parser.add_argument(
        "--single-shared-control-off-domain",
        type=int,
        default=None,
        help=(
            "run exactly one diagnostic-only SHARED_CONTROL_0_3/OFF arm "
            "on this ROS domain with the idle-break diagnostic enabled; "
            "requires --scheduling-diagnostic"
        ),
    )
    parser.add_argument(
        "--isolated-input-publisher-diagnostic",
        action="store_true",
        help=(
            "use the spawn-isolated input publisher only for the explicit "
            "single SHARED_CONTROL_0_3/OFF diagnostic arm"
        ),
    )
    parser.add_argument("--fixture-cpu-list", default="0-3")
    parser.add_argument("--pp-bound-cpu-list", default="4")
    parser.add_argument("--recorder-cpu-list", default="6-11")
    parser.add_argument("--scheduling-domains", default="225,226,227,228")
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--off-domain-base", type=int, default=81)
    parser.add_argument("--on-domain-base", type=int, default=101)
    parser.add_argument("--fixture-timeout-sec", type=float, default=75.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.repetitions < 1:
        raise ValueError("--repetitions must be positive")
    if (
        args.single_shared_control_off_domain is not None
        and not args.scheduling_diagnostic
    ):
        raise ValueError(
            "--single-shared-control-off-domain requires "
            "--scheduling-diagnostic"
        )
    validate_isolated_input_publisher_cli(
        enabled=args.isolated_input_publisher_diagnostic,
        scheduling_diagnostic=args.scheduling_diagnostic,
        single_shared_control_off_domain=(
            args.single_shared_control_off_domain
        ),
    )
    if args.artifact_root.exists():
        raise FileExistsError(
            f"refusing to overwrite artifact root {args.artifact_root}"
        )
    if not args.fixture.is_file():
        raise FileNotFoundError(args.fixture)
    args.artifact_root.mkdir(parents=True)
    scheduling_domains: tuple[int, ...] = ()
    topology: dict[str, object] | None = None
    domain_preflight: list[dict[str, object]] = []
    try:
        if args.scheduling_diagnostic:
            fixture_cpus = parse_cpu_list(args.fixture_cpu_list)
            pp_bound_cpus = parse_cpu_list(args.pp_bound_cpu_list)
            recorder_cpus = parse_cpu_list(args.recorder_cpu_list)
            if fixture_cpus & pp_bound_cpus:
                raise ValueError(
                    "fixture and PP-bound CPU lists must not overlap"
                )
            if fixture_cpus & recorder_cpus or pp_bound_cpus & recorder_cpus:
                raise ValueError("recorder CPU list must not overlap test CPUs")
            allowed_cpus = set(os.sched_getaffinity(0))
            requested_cpus = (
                fixture_cpus | pp_bound_cpus | recorder_cpus | {5}
            )
            if not requested_cpus <= allowed_cpus:
                raise ValueError(
                    f"requested CPUs {sorted(requested_cpus)} exceed allowed "
                    f"affinity {sorted(allowed_cpus)}"
                )
            os.sched_setaffinity(0, fixture_cpus)
            runner_preflight_snapshot = process_scheduler_snapshot(
                os.getpid(),
                "formal_runner",
            )
            verify_scheduler_snapshot(
                runner_preflight_snapshot,
                fixture_cpus,
            )
            topology = cpu_topology(requested_cpus)
            if any(
                not bool(entry["online"])
                for entry in topology.values()
            ):
                raise ValueError(
                    f"offline scheduling diagnostic CPU: {topology}"
                )
            if parse_cpu_list(
                str(topology["4"]["thread_siblings_list"])
            ) != {4, 5}:
                raise ValueError(
                    "CPU 4/5 are not the predeclared SMT sibling pair"
                )
            if args.single_shared_control_off_domain is None:
                scheduling_domains = parse_domains(args.scheduling_domains)
            else:
                scheduling_domains = validate_domains(
                    (args.single_shared_control_off_domain,),
                    expected_count=1,
                )
            preflight_log_root = args.artifact_root / "domain-preflight-ros-logs"
            preflight_log_root.mkdir()
            domain_preflight = [
                require_empty_domain(domain, preflight_log_root)
                for domain in scheduling_domains
            ]
    except Exception as error:
        write_json(
            args.artifact_root / "preflight_failure.json",
            {
                "type": type(error).__name__,
                "error": str(error),
                "scheduling_diagnostic": args.scheduling_diagnostic,
            },
        )
        raise

    manifest = {
        "schema": 1,
        "fixture": str(args.fixture.resolve()),
        "fixture_sha256": sha256(args.fixture),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "python": args.python,
        "cpu_list": args.cpu_list,
        "scheduling_diagnostic": args.scheduling_diagnostic,
        **diagnostic_mode_markers(
            args.scheduling_diagnostic,
            args.single_shared_control_off_domain is not None,
        ),
        "single_shared_control_off_domain": (
            args.single_shared_control_off_domain
        ),
        "isolated_input_publisher_diagnostic": (
            args.isolated_input_publisher_diagnostic
        ),
        "m4_acceptance_credit": (
            False
            if args.isolated_input_publisher_diagnostic
            else None
        ),
        "parent_contact_lease_timeout_sec": (
            ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC
            if args.isolated_input_publisher_diagnostic
            else None
        ),
        "fixture_cpu_list": args.fixture_cpu_list,
        "pp_bound_cpu_list": args.pp_bound_cpu_list,
        "recorder_cpu_list": args.recorder_cpu_list,
        "scheduling_domains": scheduling_domains,
        "cpu_topology": topology,
        "domain_preflight": domain_preflight,
        "dds_environment_contract": (
            dict(DDS_ENVIRONMENT_CONTRACT)
            if args.isolated_input_publisher_diagnostic
            else None
        ),
        "runner_preflight_scheduler": (
            runner_preflight_snapshot
            if args.scheduling_diagnostic
            else None
        ),
        "repetitions": args.repetitions,
        "off_domain_base": args.off_domain_base,
        "on_domain_base": args.on_domain_base,
        "recorder_topics": list(RECORDER_TOPICS),
        "runtime_artifacts": runtime_artifact_fingerprints(),
        "environment": {
            key: os.environ.get(key, "")
            for key in (
                "C002AY0_EXACT_PROPOSAL_RELAY",
                "C002AY0_DOCKER_IMAGE_ID",
                "C002AY0_SOURCE_HEAD",
                "RMW_IMPLEMENTATION",
                "ROS_DISTRO",
            )
        },
    }
    write_json(args.artifact_root / "manifest.json", manifest)
    results = []
    stop_reason = ""
    if args.scheduling_diagnostic:
        scheduling_scenarios = (
            (("SHARED_CONTROL_0_3", "OFF"),)
            if args.single_shared_control_off_domain is not None
            else SCHEDULING_SCENARIOS
        )
        for iteration, ((profile, condition), domain_id) in enumerate(
            zip(scheduling_scenarios, scheduling_domains),
            start=1,
        ):
            result = run_one(
                condition=condition,
                iteration=iteration,
                domain_id=domain_id,
                artifact_root=args.artifact_root,
                fixture=args.fixture,
                python=args.python,
                fixture_cpu_list=args.fixture_cpu_list,
                recorder_cpu_list=args.recorder_cpu_list,
                pp_cpu_list=(
                    args.pp_bound_cpu_list
                    if profile == "PP_BOUND_CPU4"
                    else args.fixture_cpu_list
                ),
                scheduling_profile=profile,
                fixture_timeout_sec=args.fixture_timeout_sec,
                idle_break_diagnostic=(
                    args.single_shared_control_off_domain is not None
                ),
                isolated_input_publisher_diagnostic=(
                    args.isolated_input_publisher_diagnostic
                ),
            )
            results.append(result)
            write_json(args.artifact_root / "aggregate.json", results)
            if not result["integrity_valid"]:
                stop_reason = (
                    f"{iteration:02d}-{condition.lower()}-{profile.lower()} "
                    "integrity invalid"
                )
                break

        if args.single_shared_control_off_domain is not None:
            result = results[0]
            classification = classify_single_scheduling_result(result)
            summary = {
                "requested_runs": 1,
                "completed_runs": 1,
                "integrity_valid_runs": int(
                    bool(result["integrity_valid"])
                ),
                "timing_pass_runs": int(bool(result["timing_pass"])),
                "stopped": True,
                "stop_reason": (
                    "single diagnostic arm completed; no retry permitted"
                ),
                "classification": classification,
                **diagnostic_mode_markers(True, True),
                "isolated_input_publisher_diagnostic": (
                    args.isolated_input_publisher_diagnostic
                ),
                "m4_acceptance_credit": (
                    False
                    if args.isolated_input_publisher_diagnostic
                    else None
                ),
                "parent_contact_lease_timeout_sec": (
                    ISOLATED_INPUT_PUBLISHER_PARENT_LEASE_TIMEOUT_SEC
                    if args.isolated_input_publisher_diagnostic
                    else None
                ),
                "acceptance_pass": False,
            }
            write_json(args.artifact_root / "summary.json", summary)
            print(json.dumps(summary, sort_keys=True))
            return 1 if classification == "INVALID_NOT_INTERPRETABLE" else 2

        if stop_reason or len(results) != 4:
            classification = "INVALID_NOT_INTERPRETABLE"
        else:
            scheduler_signatures = [
                result["scheduling_evidence"]["scheduler_signatures"]
                for result in results
            ]
            runner_signatures = [
                sorted(
                    {
                        (
                            int(task["scheduler_policy"]),
                            int(task["scheduler_priority"]),
                            int(task["nice"]),
                        )
                        for task in result["runner_scheduler_after"]["tids"]
                    }
                )
                for result in results
            ]
            if any(
                signature != scheduler_signatures[0]
                for signature in scheduler_signatures[1:]
            ) or any(
                signature != runner_signatures[0]
                for signature in runner_signatures[1:]
            ):
                stop_reason = "scheduler policy/priority/nice changed across arms"
                classification = "INVALID_NOT_INTERPRETABLE"
            else:
                shared_pass = [
                    bool(results[index]["timing_pass"]) for index in (0, 3)
                ]
                bound_pass = [
                    bool(results[index]["timing_pass"]) for index in (1, 2)
                ]
                if not all(bound_pass):
                    classification = "CANDIDATE_REJECTED"
                elif not all(shared_pass):
                    classification = (
                        "PP_PROCESS_BINDING_CANDIDATE_SUPPORTED_NOT_ACCEPTANCE"
                    )
                else:
                    classification = "INCONCLUSIVE_NO_RETRY"
        summary = {
            "requested_runs": 4,
            "completed_runs": len(results),
            "integrity_valid_runs": sum(
                bool(result["integrity_valid"]) for result in results
            ),
            "timing_pass_runs": sum(
                bool(result["timing_pass"]) for result in results
            ),
            "stopped": bool(stop_reason),
            "stop_reason": stop_reason,
            "classification": classification,
            **diagnostic_mode_markers(True, False),
            "isolated_input_publisher_diagnostic": False,
            "m4_acceptance_credit": None,
            "parent_contact_lease_timeout_sec": None,
            "acceptance_pass": False,
        }
        write_json(args.artifact_root / "summary.json", summary)
        print(json.dumps(summary, sort_keys=True))
        return 1 if classification == "INVALID_NOT_INTERPRETABLE" else 2

    for iteration in range(1, args.repetitions + 1):
        for condition, domain_id in (
            ("OFF", args.off_domain_base + iteration - 1),
            ("ON", args.on_domain_base + iteration - 1),
        ):
            result = run_one(
                condition=condition,
                iteration=iteration,
                domain_id=domain_id,
                artifact_root=args.artifact_root,
                fixture=args.fixture,
                python=args.python,
                fixture_cpu_list=args.cpu_list,
                recorder_cpu_list=args.cpu_list,
                pp_cpu_list=None,
                scheduling_profile=None,
                fixture_timeout_sec=args.fixture_timeout_sec,
                idle_break_diagnostic=False,
                isolated_input_publisher_diagnostic=False,
            )
            results.append(result)
            write_json(args.artifact_root / "aggregate.json", results)
            if not result["valid"]:
                stop_reason = (
                    f"{iteration:02d}-{condition.lower()} formal run invalid"
                )
                break
        if stop_reason:
            break

    summary = {
        "requested_runs": args.repetitions * 2,
        "completed_runs": len(results),
        "valid_runs": sum(bool(result["valid"]) for result in results),
        "stopped": bool(stop_reason),
        "stop_reason": stop_reason,
        "all_valid": len(results) == args.repetitions * 2
        and all(
            bool(result["valid"]) and bool(result["m4_wcet_eligible"])
            for result in results
        ),
        "idle_break_diagnostic": False,
        "isolated_input_publisher_diagnostic": False,
        "m4_acceptance_credit": None,
        "parent_contact_lease_timeout_sec": None,
    }
    write_json(args.artifact_root / "summary.json", summary)
    print(json.dumps(summary, sort_keys=True))
    return 0 if summary["all_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
