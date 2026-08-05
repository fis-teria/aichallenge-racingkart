from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import posixpath
import re
import shutil
import secrets
import shlex
import stat
import subprocess
import sys
import tempfile
import time
from urllib.parse import unquote, urlparse
from datetime import datetime
from pathlib import Path, PurePosixPath
from typing import Any, Sequence
import xml.etree.ElementTree as ElementTree

from . import SCHEMA_VERSION
from .model import read_result, write_json_atomic


SERVICE = "autoware-simulator-evaluation"
FATAL_LOG_MARKERS = (
    "InvalidParameterTypeException",
    "malformed launch argument",
    "process has died",
    "terminate called after throwing",
)
CYCLONEDDS_DEFAULT_PORTS = {
    "base": 7400,
    "domain_gain": 250,
    "multicast_meta_offset": 0,
    "multicast_data_offset": 1,
    "participant_gain": 2,
    "unicast_meta_offset": 10,
    "unicast_data_offset": 11,
}
MAX_UDP_PORT = 65535
GATE2_RUNTIME_TIMEOUT_S = 240.0
GATE2_FRESHNESS_TOLERANCE_NS = 2_000_000_000
GATE2_ENV_EXACT_KEYS = frozenset(
    {
        "AWSIM_EXTRA_ARGS", "GATE_EXTRA_ARGS", "CONTROL_METHOD", "RUN_KIND",
        "PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL", "OUTPUT_ROOT", "OUTPUT_HOST_ROOT",
        "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED",
        "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED",
        "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID",
        "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID",
        "STATE_LATTICE_V2_SESSION_ID",
        "OVERTAKE_TRAJECTORY_BACKEND", "PP_CORE_EXACT_SNAPSHOT_ENABLED", "ROSBAG",
        "AWSIM_START_MODE", "RACE_ARM_ON_VEHICLE_STATE",
        "MAKEFLAGS", "MFLAGS", "MAKEOVERRIDES", "MAKELEVEL",
        "COMPOSE_PROJECT_NAME", "COMPOSE_FILE",
    }
)
GATE2_ANALYSIS_TIMEOUT_S = 60.0
GATE2_METADATA_MAX_BYTES = 1024 * 1024
MAX_OBSERVER_DEADLINE_CHAIN_CYCLES = 256
CYCLONEDDS_NAMESPACE = "https://cdds.io/config"
CYCLONEDDS_ROOT_TAG = f"{{{CYCLONEDDS_NAMESPACE}}}CycloneDDS"
CYCLONEDDS_SEMANTIC_ELEMENTS = frozenset(
    {
        "CycloneDDS", "Domain", "General", "Discovery", "Ports", "Interfaces",
        "NetworkInterface", "AllowMulticast", "ExternalDomainId",
        "ParticipantIndex", "MaxAutoParticipantIndex", "Base", "DomainGain",
        "MulticastMetaOffset", "MulticastDataOffset", "ParticipantGain",
        "UnicastMetaOffset", "UnicastDataOffset",
    }
)
V2_UPTAKE_EXPECTED_HASH_ENV = {
    "archive_sha256": "AIC_TEST_V2_UPTAKE_EXPECTED_ARCHIVE_SHA256",
    "planner_source_sha256": "AIC_TEST_V2_UPTAKE_EXPECTED_PLANNER_SOURCE_SHA256",
    "pp_source_sha256": "AIC_TEST_V2_UPTAKE_EXPECTED_PP_SOURCE_SHA256",
    "planner_binary_sha256": "AIC_TEST_V2_UPTAKE_EXPECTED_PLANNER_BINARY_SHA256",
    "pp_binary_sha256": "AIC_TEST_V2_UPTAKE_EXPECTED_PP_BINARY_SHA256",
    "observer_sha256": "AIC_TEST_V2_UPTAKE_EXPECTED_OBSERVER_SHA256",
    "driver_sha256": "AIC_TEST_V2_UPTAKE_EXPECTED_DRIVER_SHA256",
}
V2_UPTAKE_IMAGE_REFERENCE_ENV = "AIC_TEST_V2_UPTAKE_EXPECTED_IMAGE_REFERENCE"
V2_UPTAKE_IMAGE_ID_ENV = "AIC_TEST_V2_UPTAKE_EXPECTED_IMAGE_ID"
V2_UPTAKE_EXECUTION_ID_ENV = "AIC_TEST_V2_UPTAKE_EXECUTION_ID"
V2_UPTAKE_REVIEW_STATE_PATH_ENV = "AIC_TEST_V2_UPTAKE_REVIEW_STATE_PATH"
V2_UPTAKE_REVIEW_STATE_SHA_ENV = "AIC_TEST_V2_UPTAKE_REVIEW_STATE_SHA256"
V2_UPTAKE_REVIEWED_EXECUTION_ID_ENV = "AIC_TEST_V2_UPTAKE_REVIEWED_EXECUTION_ID"
V2_UPTAKE_REVIEWED_HANDOFF_SHA_ENV = "AIC_TEST_V2_UPTAKE_REVIEWED_HANDOFF_SHA256"
V2_UPTAKE_CURRENT_EDGE_ID_ENV = "AIC_TEST_V2_UPTAKE_CURRENT_EDGE_ID"
V2_UPTAKE_REVIEWED_HANDOFF_PATH_ENV = "AIC_TEST_V2_UPTAKE_REVIEWED_HANDOFF_PATH"
PLANNER_INSTALL_ROOT = PurePosixPath(
    "/aichallenge/workspace/install/state_lattice_overtake_planner"
)
PLANNER_BUILD_TARGET = PurePosixPath(
    "/aichallenge/workspace/build/state_lattice_overtake_planner/"
    "state_lattice_overtake_planner_node"
)
PP_INSTALL_ROOT = PurePosixPath("/aichallenge/workspace/install/simple_pure_pursuit")
PP_BUILD_TARGET = PurePosixPath(
    "/aichallenge/workspace/build/simple_pure_pursuit/simple_pure_pursuit"
)
PLANNER_BINARY_ATTESTATION_MAX_HOPS = 16
V2_UPTAKE_UDP_SOCKET_INSPECTION_TIMEOUT_S = 5.0


def _regular_file_sha256(path: Path) -> str | None:
    try:
        if not path.is_file() or stat.S_ISLNK(path.lstat().st_mode):
            return None
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def _expected_prior_consumed_execution_ids(execution_id: str) -> tuple[str, ...]:
    if execution_id == "m4-v2-planner-generation-fresh-runtime-07":
        return (
            "m4-v2-planner-generation-fresh-runtime-04",
            "m4-v2-planner-generation-fresh-runtime-06",
        )
    return ("m4-v2-planner-generation-fresh-runtime-04",)


def _expected_prior_consumed_first_reason(execution_id: str) -> str:
    return {
        "m4-v2-planner-generation-fresh-runtime-04": "image_planner_binary_sha256_mismatch",
        "m4-v2-planner-generation-fresh-runtime-06": "image_pp_binary_sha256_mismatch",
    }[execution_id]


def _prior_consumed_attempt_is_valid(attempt: object, expected_execution_id: str) -> bool:
    required = {
        "handoff_path", "handoff_sha256", "execution_id", "outcome", "first_reason",
        "capture_started", "runtime_budget", "result_path", "result_sha256",
    }
    if not isinstance(attempt, dict) or set(attempt) != required:
        return False
    handoff_path = Path(attempt["handoff_path"]) if isinstance(attempt["handoff_path"], str) and Path(attempt["handoff_path"]).is_absolute() else None
    result_path = Path(attempt["result_path"]) if isinstance(attempt["result_path"], str) and Path(attempt["result_path"]).is_absolute() else None
    handoff_sha = _normalized_sha256(attempt["handoff_sha256"])
    result_sha = _normalized_sha256(attempt["result_sha256"])
    budget = {"fresh_runtime_max": 1, "fresh_runtime_used": 1, "retry_allowed": False}
    if (
        handoff_path is None or result_path is None or handoff_sha is None or result_sha is None
        or attempt["execution_id"] != expected_execution_id
        or attempt["outcome"] != "INFRASTRUCTURE_FAILED"
        or attempt["first_reason"] != _expected_prior_consumed_first_reason(expected_execution_id)
        or attempt["capture_started"] is not False
        or attempt["runtime_budget"] != budget
    ):
        return False
    try:
        if (
            stat.S_ISLNK(handoff_path.lstat().st_mode) or not handoff_path.is_file()
            or stat.S_IMODE(handoff_path.stat().st_mode) != 0o644
            or _regular_file_sha256(handoff_path) != handoff_sha
            or stat.S_ISLNK(result_path.lstat().st_mode) or not result_path.is_file()
            or stat.S_IMODE(result_path.stat().st_mode) != 0o644
            or _regular_file_sha256(result_path) != result_sha
        ):
            return False
        handoff = json.loads(handoff_path.read_bytes())
        result = json.loads(result_path.read_bytes())
        return (
            handoff.get("execution_id") == expected_execution_id
            and handoff.get("runtime", {}).get("outcome") == attempt["outcome"]
            and handoff.get("runtime", {}).get("first_reason") == attempt["first_reason"]
            and handoff.get("runtime", {}).get("capture_started") is False
            and handoff.get("runtime_budget") == budget
            and handoff.get("binding", {}).get("result_path") == str(result_path)
            and handoff.get("binding", {}).get("result_sha256") == result_sha
            and result.get("outcome") == attempt["outcome"]
            and result.get("prearm", {}).get("execution_id") == expected_execution_id
            and result.get("review_state_budget_transition", {}).get("before_sha256") is not None
            and result.get("review_state_budget_transition", {}).get("after_sha256") is not None
            and isinstance(result.get("one_shot_budget_guard"), str)
            and result.get("docker_create_returncode") == 0
            and result.get("observer") is None
            and result.get("container_cleanup", {}).get("absent_after_cleanup") is True
        )
    except (OSError, TypeError, json.JSONDecodeError):
        return False


def _deferred_connection_unavailable_gate_is_satisfied(
    gate: dict[str, Any], expected_handoff: str, handoff_path: Path,
    execution_id: str, image_reference: str, image_id: str,
    expected_hashes: dict[str, str], ros_domain_id: int | None,
    requested_runtime_timeout_s: float | None,
) -> bool:
    """Validate the narrowly scoped external-review connection exception."""
    required = {
        "completed", "terminal_status", "workflow_advance_allowed",
        "reviewed_execution_id", "reviewed_handoff_sha256", "reviewed_handoff_path",
        "external_review_status", "review_packet_path", "review_packet_sha256",
        "connection_attempts", "terminal_connection_error", "local_safety_review",
        "retrospective_review", "authorization",
    }
    if not required <= set(gate):
        return False
    packet_raw = gate["review_packet_path"]
    packet_path = Path(packet_raw) if isinstance(packet_raw, str) and Path(packet_raw).is_absolute() else None
    packet_sha = _normalized_sha256(gate["review_packet_sha256"])
    terminal_error = gate["terminal_connection_error"]
    attempts = gate["connection_attempts"]
    local_safety = gate["local_safety_review"]
    retrospective = gate["retrospective_review"]
    authorization = gate["authorization"]
    if (
        gate["completed"] is not False
        or gate["terminal_status"] != "DEFERRED_CONNECTION_UNAVAILABLE"
        or gate["external_review_status"] != "DEFERRED_CONNECTION_UNAVAILABLE"
        or gate["workflow_advance_allowed"] is not True
        or packet_path is None
        or packet_sha is None
        or (terminal_error is not None and (not isinstance(terminal_error, str) or not terminal_error.strip()))
        or not isinstance(attempts, list)
        or not isinstance(local_safety, dict)
        or not isinstance(retrospective, dict)
        or not isinstance(authorization, dict)
    ):
        return False
    bounded_failures = 0
    for attempt in attempts:
        if (
            not isinstance(attempt, dict)
            or set(attempt) != {"timestamp", "outcome", "timeout_s"}
            or not isinstance(attempt["timestamp"], str)
            or not attempt["timestamp"].strip()
            or attempt["outcome"] not in {
                "BROWSER_UNAVAILABLE", "CONNECTION_UNAVAILABLE", "DISCONNECTED",
                "PROJECT_UNAVAILABLE", "TERMINAL_ERROR",
            }
            or type(attempt["timeout_s"]) not in {int, float}
            or not math.isfinite(attempt["timeout_s"])
            or not 0 < attempt["timeout_s"] <= 1800
        ):
            return False
        bounded_failures += 1
    if bounded_failures < 3:
        return False
    authorization_keys = {
        "user_authorization", "authorization_record_path", "authorization_record_sha256",
        "authorized_runtime_execution_id", "ros_domain_id", "expected_image_reference",
        "expected_image_id", "expected_hashes", "outer_timeout_s", "runtime_timeout_s",
        "fresh_runtime_max", "fresh_runtime_used", "retry_allowed", "prior_consumed_attempts",
    }
    record_raw = authorization.get("authorization_record_path")
    record_path = Path(record_raw) if isinstance(record_raw, str) and Path(record_raw).is_absolute() else None
    record_sha = _normalized_sha256(authorization.get("authorization_record_sha256"))
    if (
        set(authorization) != authorization_keys
        or authorization.get("user_authorization") != "EXPLICIT"
        or record_path is None or record_sha is None
        or authorization.get("authorized_runtime_execution_id") != execution_id
        or type(authorization.get("ros_domain_id")) is not int
        or authorization.get("ros_domain_id") != ros_domain_id
        or authorization.get("expected_image_reference") != image_reference
        or authorization.get("expected_image_id") != image_id
        or authorization.get("expected_hashes") != expected_hashes
        or authorization.get("outer_timeout_s") != 90
        or authorization.get("runtime_timeout_s") != 90
        or type(requested_runtime_timeout_s) not in {int, float}
        or not math.isfinite(requested_runtime_timeout_s)
        or requested_runtime_timeout_s != 90.0
        or authorization.get("fresh_runtime_max") != 1
        or authorization.get("fresh_runtime_used") != 0
        or authorization.get("retry_allowed") is not False
        or not isinstance(authorization.get("prior_consumed_attempts"), list)
    ):
        return False
    expected_prior_ids = _expected_prior_consumed_execution_ids(execution_id)
    prior_attempts = authorization["prior_consumed_attempts"]
    if (
        len(prior_attempts) != len(expected_prior_ids)
        or any(not _prior_consumed_attempt_is_valid(attempt, expected_id)
               for attempt, expected_id in zip(prior_attempts, expected_prior_ids))
    ):
        return False
    if (
        set(local_safety) != {"disposition", "blocker_count", "major_count"}
        or local_safety.get("disposition") != "GO"
        or local_safety.get("blocker_count") != 0
        or local_safety.get("major_count") != 0
        or type(local_safety.get("blocker_count")) is not int
        or type(local_safety.get("major_count")) is not int
        or set(retrospective) != {"status", "review_packet_path", "review_packet_sha256", "handoff_sha256"}
        or retrospective.get("status") != "QUEUED"
        or retrospective.get("review_packet_path") != str(packet_path)
        or retrospective.get("review_packet_sha256") != packet_sha
        or retrospective.get("handoff_sha256") != expected_handoff
    ):
        return False
    try:
        return (
            not stat.S_ISLNK(packet_path.lstat().st_mode)
            and packet_path.is_file()
            and stat.S_IMODE(packet_path.stat().st_mode) == 0o644
            and _regular_file_sha256(packet_path) == packet_sha
            and Path(gate["reviewed_handoff_path"]).resolve() == handoff_path.resolve()
            and not stat.S_ISLNK(record_path.lstat().st_mode)
            and record_path.is_file()
            and stat.S_IMODE(record_path.stat().st_mode) == 0o644
            and _regular_file_sha256(record_path) == record_sha
            and json.loads(record_path.read_bytes()) == {
                **authorization, "authorization_record_sha256": ""
            }
        )
    except (OSError, TypeError, json.JSONDecodeError):
        return False


def _v2_uptake_candidate_udp_ports(cyclonedds: dict[str, Any]) -> set[int] | None:
    """Return every host UDP port CycloneDDS may use for this direct run."""
    effective = cyclonedds.get("effective_ports")
    domain_base = cyclonedds.get("computed_domain_base")
    participant_index = cyclonedds.get("participant_index")
    max_index = cyclonedds.get("max_auto_participant_index")
    required = {
        "multicast_meta_offset", "multicast_data_offset", "participant_gain",
        "unicast_meta_offset", "unicast_data_offset",
    }
    if (
        not isinstance(effective, dict) or not required <= set(effective)
        or type(domain_base) is not int or type(max_index) is not int
        or not isinstance(participant_index, str)
        or any(type(effective[name]) is not int for name in required)
        or not 0 <= max_index <= 1024
    ):
        return None
    ports = {
        domain_base + effective["multicast_meta_offset"],
        domain_base + effective["multicast_data_offset"],
    }
    if participant_index == "none":
        indexes: range | tuple[int, ...] = ()
    elif participant_index in {"auto", "default"}:
        indexes = range(max_index + 1)
    elif participant_index.isdigit():
        indexes = (int(participant_index),)
    else:
        return None
    for index in indexes:
        ports.add(domain_base + effective["unicast_meta_offset"] + index * effective["participant_gain"])
        ports.add(domain_base + effective["unicast_data_offset"] + index * effective["participant_gain"])
    if not ports or any(port < 1 or port > MAX_UDP_PORT for port in ports):
        return None
    return ports


def _v2_uptake_udp_socket_preflight(
    cyclonedds: dict[str, Any]
) -> tuple[dict[str, Any], str | None]:
    """Fail closed unless socket-owner inspection proves candidate ports are free."""
    candidates = _v2_uptake_candidate_udp_ports(cyclonedds)
    evidence: dict[str, Any] = {
        "command": ["ss", "-H", "-lunp"],
        "timeout_s": V2_UPTAKE_UDP_SOCKET_INSPECTION_TIMEOUT_S,
        "candidate_ports": sorted(candidates) if candidates is not None else [],
        "inspection_available": False,
        "bound_socket_lines": [],
    }
    if candidates is None:
        return evidence, "udp_socket_candidate_ports_unavailable"
    try:
        completed = subprocess.run(
            evidence["command"], check=False, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=V2_UPTAKE_UDP_SOCKET_INSPECTION_TIMEOUT_S,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        evidence["inspection_error"] = type(error).__name__
        return evidence, "udp_socket_inspection_unavailable"
    evidence["returncode"] = completed.returncode
    evidence["stdout_sha256"] = hashlib.sha256(completed.stdout.encode()).hexdigest()
    evidence["stderr_sha256"] = hashlib.sha256(completed.stderr.encode()).hexdigest()
    if completed.returncode != 0:
        return evidence, "udp_socket_inspection_unavailable"
    evidence["inspection_available"] = True
    for line in completed.stdout.splitlines():
        ports = {int(value) for value in re.findall(r":(\d{1,5})(?!\d)", line)}
        if ports & candidates:
            evidence["bound_socket_lines"].append(line)
    if evidence["bound_socket_lines"]:
        return evidence, "udp_socket_candidate_port_bound"
    return evidence, None


def _v2_uptake_static_prearm(
    repo: Path, ros_domain_id: int | None = None,
    requested_runtime_timeout_s: float | None = None,
) -> tuple[dict[str, Any] | None, str | None]:
    """Validate immutable runtime inputs without invoking Docker."""
    if sys.flags.optimize != 0 or os.environ.get("PYTHONOPTIMIZE") not in {None, ""}:
        return None, "python_optimize_enabled"
    image_reference = os.environ.get(V2_UPTAKE_IMAGE_REFERENCE_ENV)
    image_id = os.environ.get(V2_UPTAKE_IMAGE_ID_ENV)
    if not isinstance(image_reference, str) or not re.fullmatch(
        r"aichallenge-2025-eval@sha256:[0-9a-f]{64}", image_reference
    ):
        return None, "expected_image_reference_invalid"
    if not isinstance(image_id, str) or not re.fullmatch(r"sha256:[0-9a-f]{64}", image_id):
        return None, "expected_image_id_invalid"
    paths = {
        "archive_sha256": repo / "submit" / "aichallenge_submit.tar.gz",
        "planner_source_sha256": repo / "aichallenge" / "workspace" / "src" / "aichallenge_submit" / "state_lattice_overtake_planner" / "src" / "state_lattice_overtake_planner_node.cpp",
        "pp_source_sha256": repo / "aichallenge" / "workspace" / "src" / "aichallenge_submit" / "simple_pure_pursuit" / "src" / "simple_pure_pursuit.cpp",
        "observer_sha256": repo / "tools" / "aic_test" / "aic_test" / "v2_uptake_observer.py",
        "driver_sha256": repo / "tools" / "aic_test" / "aic_test" / "v2_direct_input_driver.py",
    }
    actual = {name: _regular_file_sha256(path) for name, path in paths.items()}
    for name, environment_name in V2_UPTAKE_EXPECTED_HASH_ENV.items():
        expected = _normalized_sha256(os.environ.get(environment_name))
        if expected is None:
            return None, f"expected_{name}_invalid"
        if name in actual and actual[name] != expected:
            return None, f"{name}_mismatch"
    execution_id = os.environ.get(V2_UPTAKE_EXECUTION_ID_ENV)
    if not isinstance(execution_id, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", execution_id):
        return None, "v2_uptake_execution_id_invalid"
    state_raw = os.environ.get(V2_UPTAKE_REVIEW_STATE_PATH_ENV)
    state_path = Path(state_raw) if isinstance(state_raw, str) and Path(state_raw).is_absolute() else None
    expected_state_sha = _normalized_sha256(os.environ.get(V2_UPTAKE_REVIEW_STATE_SHA_ENV))
    expected_execution = os.environ.get(V2_UPTAKE_REVIEWED_EXECUTION_ID_ENV)
    expected_handoff = _normalized_sha256(os.environ.get(V2_UPTAKE_REVIEWED_HANDOFF_SHA_ENV))
    expected_edge_id = os.environ.get(V2_UPTAKE_CURRENT_EDGE_ID_ENV)
    handoff_raw = os.environ.get(V2_UPTAKE_REVIEWED_HANDOFF_PATH_ENV)
    handoff_path = Path(handoff_raw) if isinstance(handoff_raw, str) and Path(handoff_raw).is_absolute() else None
    if state_path is None or expected_state_sha is None or not isinstance(expected_execution, str) or not expected_execution or expected_handoff is None or expected_handoff == "0" * 64 or not isinstance(expected_edge_id, str) or not expected_edge_id or handoff_path is None:
        return None, "review_state_contract_invalid"
    try:
        if stat.S_ISLNK(state_path.lstat().st_mode) or not state_path.is_file():
            return None, "review_state_path_invalid"
        state_bytes = state_path.read_bytes()
        if hashlib.sha256(state_bytes).hexdigest() != expected_state_sha:
            return None, "review_state_hash_mismatch"
        review = json.loads(state_bytes)
    except (OSError, json.JSONDecodeError):
        return None, "review_state_unreadable"
    gate = review.get("external_review_gate") if isinstance(review, dict) else None
    edge = review.get("current_owning_edge") if isinstance(review, dict) else None
    budget = edge.get("runtime_budget") if isinstance(edge, dict) else None
    required_gate = {"completed", "terminal_status", "workflow_advance_allowed", "reviewed_execution_id", "reviewed_handoff_sha256", "reviewed_handoff_path"}
    required_edge = {"id", "permits_runtime_arm", "authorized_runtime_execution_id", "runtime_budget"}
    required_budget = {"fresh_runtime_max", "fresh_runtime_used", "retry_allowed"}
    if (not isinstance(review, dict) or type(review.get("schema_version")) is not int or review["schema_version"] != 1 or not isinstance(gate, dict) or not required_gate <= set(gate) or not isinstance(edge, dict) or not required_edge <= set(edge) or not isinstance(budget, dict) or set(budget) != required_budget or type(gate["completed"]) is not bool or not isinstance(gate["terminal_status"], str) or type(gate["workflow_advance_allowed"]) is not bool or not isinstance(gate["reviewed_execution_id"], str) or not isinstance(gate["reviewed_handoff_sha256"], str) or not isinstance(gate["reviewed_handoff_path"], str) or not isinstance(edge["id"], str) or type(edge["permits_runtime_arm"]) is not bool or not isinstance(edge["authorized_runtime_execution_id"], str) or type(budget["fresh_runtime_max"]) is not int or type(budget["fresh_runtime_used"]) is not int or type(budget["retry_allowed"]) is not bool):
        return None, "review_state_schema_invalid"
    try:
        if stat.S_ISLNK(handoff_path.lstat().st_mode) or not handoff_path.is_file() or stat.S_IMODE(handoff_path.stat().st_mode) != 0o644 or _regular_file_sha256(handoff_path) != expected_handoff:
            return None, "reviewed_handoff_path_invalid"
    except OSError:
        return None, "reviewed_handoff_path_invalid"
    completed_gate = (
        gate.get("completed") is True
        and gate.get("selected_model") in {"GPT-5.6 Sol Pro", "GPT-5.6 Pro"}
        and gate.get("terminal_status") == "COMPLETED"
        and gate.get("workflow_advance_allowed") is True
    )
    deferred_gate = _deferred_connection_unavailable_gate_is_satisfied(
        gate, expected_handoff, handoff_path, execution_id, image_reference, image_id,
        {name: os.environ[environment_name].lower()
         for name, environment_name in V2_UPTAKE_EXPECTED_HASH_ENV.items()},
        ros_domain_id, requested_runtime_timeout_s,
    )
    if not isinstance(gate, dict) or not isinstance(edge, dict) or not isinstance(budget, dict) or not (completed_gate or deferred_gate) or gate.get("reviewed_execution_id") != expected_execution or gate.get("reviewed_handoff_sha256") != expected_handoff or Path(gate.get("reviewed_handoff_path", "")).resolve() != handoff_path.resolve() or edge.get("id") != expected_edge_id or edge.get("authorized_runtime_execution_id") != execution_id or budget.get("fresh_runtime_max") != 1 or budget.get("fresh_runtime_used") != 0 or budget.get("retry_allowed") is not False or edge.get("permits_runtime_arm") is not True:
        return None, "review_state_gate_not_satisfied"
    return {
        "expected_image_reference": image_reference,
        "expected_image_id": image_id,
        "expected_hashes": {name: os.environ[environment_name].lower()
                            for name, environment_name in V2_UPTAKE_EXPECTED_HASH_ENV.items()},
        "actual_hashes": actual,
        "paths": {name: str(path) for name, path in paths.items()},
        "execution_id": execution_id,
        "review_state_path": str(state_path), "review_state_sha256": expected_state_sha,
        "reviewed_execution_id": expected_execution, "reviewed_handoff_sha256": expected_handoff,
        "current_edge_id": expected_edge_id,
        "reviewed_handoff_path": str(handoff_path),
        "external_review_status": gate.get("external_review_status", "COMPLETED"),
        "runtime_timeout_s": (
            gate["authorization"]["runtime_timeout_s"]
            if deferred_gate else requested_runtime_timeout_s
        ),
    }, None


def _consume_v2_uptake_one_shot_budget(
    repo: Path, run_id: str, prearm: dict[str, Any]
) -> tuple[Path | None, str | None]:
    """Durably consume the V2 host-run budget; callers must never restore it."""
    guard = repo / "analysis" / "aic_test" / (
        f"v2_uptake_one_shot_{prearm['current_edge_id']}_{prearm['reviewed_handoff_sha256']}.json"
    )
    guard.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(guard, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    except FileExistsError:
        return None, "v2_uptake_one_shot_budget_already_consumed"
    except OSError as error:
        return None, f"v2_uptake_one_shot_budget_create_failed:{type(error).__name__}"
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump({"schema_version": 1, "run_id": run_id, "execution_id": prearm["execution_id"], "prearm": prearm}, stream,
                      sort_keys=True, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        directory_fd = os.open(guard.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except OSError as error:
        return None, f"v2_uptake_one_shot_budget_sync_failed:{type(error).__name__}"
    return guard, None


def _consume_v2_uptake_state_budget(prearm: dict[str, Any]) -> tuple[str | None, str | None]:
    """Transition the reviewed edge budget once; a guard already holds dispatch."""
    path = Path(prearm["review_state_path"])
    try:
        raw = path.read_bytes()
        before = hashlib.sha256(raw).hexdigest()
        if before != prearm["review_state_sha256"]:
            return None, "review_state_drift_before_budget_transition"
        state = json.loads(raw)
        budget = state["current_owning_edge"]["runtime_budget"]
        if budget.get("fresh_runtime_used") != 0:
            return None, "review_state_budget_not_fresh"
        budget["fresh_runtime_used"] = 1
        temporary = path.with_suffix(path.suffix + ".arm.tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(state, stream, sort_keys=True, ensure_ascii=False)
            stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return before, hashlib.sha256(path.read_bytes()).hexdigest()
    except (OSError, KeyError, TypeError, json.JSONDecodeError):
        return None, "review_state_budget_transition_failed"


def _container_binary_attestation(
    repo: Path, container_name: str, destination: Path, expected_sha256: str,
    *, install_root: PurePosixPath, build_target: PurePosixPath,
    executable_relative_path: PurePosixPath, binary_name: str,
) -> tuple[str | None, dict[str, Any], str | None]:
    """Copy and resolve an allowlisted executable in the stopped container only."""
    requested = install_root / executable_relative_path
    observation: dict[str, Any] = {
        "requested_path": str(requested),
        "initial_type": None,
        "normalized_chain": [],
        "copy_outcomes": [],
        "resolved_path": None,
        "terminal_type": None,
        "size_bytes": None,
        "actual_sha256": None,
    }
    current = requested
    seen: set[str] = set()
    for hop in range(PLANNER_BINARY_ATTESTATION_MAX_HOPS + 1):
        normalized = PurePosixPath(posixpath.normpath(str(current)))
        if not normalized.is_absolute() or (
            not normalized.is_relative_to(install_root)
            and normalized != build_target
        ):
            return None, observation, f"image_{binary_name}_binary_resolution_failed:path_escape"
        rendered = str(normalized)
        if rendered in seen:
            return None, observation, f"image_{binary_name}_binary_resolution_failed:symlink_cycle"
        seen.add(rendered)
        observation["normalized_chain"].append(rendered)
        copied_path = destination / f"{binary_name}_binary_hop_{hop:02d}"
        copied = _run(
            ["docker", "cp", f"{container_name}:{rendered}", str(copied_path)],
            cwd=repo, timeout=10,
        )
        copy_outcome: dict[str, Any] = {
            "normalized_path": rendered,
            "returncode": copied.returncode,
        }
        observation["copy_outcomes"].append(copy_outcome)
        if copied.returncode != 0:
            if hop == 0:
                observation["initial_type"] = "unobserved"
            return None, observation, f"image_{binary_name}_binary_resolution_failed:copy_failed"
        try:
            mode = copied_path.lstat().st_mode
        except FileNotFoundError:
            if hop == 0:
                observation["initial_type"] = "missing"
            return None, observation, f"image_{binary_name}_binary_resolution_failed:copy_missing"
        except OSError:
            if hop == 0:
                observation["initial_type"] = "unobserved"
            return None, observation, f"image_{binary_name}_binary_resolution_failed:copy_missing"
        file_type = (
            "symlink" if stat.S_ISLNK(mode) else "regular" if stat.S_ISREG(mode)
            else "directory" if stat.S_ISDIR(mode) else "other"
        )
        copy_outcome["local_type"] = file_type
        if hop == 0:
            observation["initial_type"] = file_type
        if file_type == "symlink":
            try:
                target = os.readlink(copied_path)
            except OSError:
                return None, observation, f"image_{binary_name}_binary_resolution_failed:symlink_unreadable"
            current = (
                PurePosixPath(target)
                if PurePosixPath(target).is_absolute()
                else normalized.parent / target
            )
            continue
        observation["resolved_path"] = rendered
        observation["terminal_type"] = file_type
        if file_type != "regular":
            return None, observation, f"image_{binary_name}_binary_resolution_failed:terminal_not_regular"
        try:
            size_bytes = copied_path.stat().st_size
            actual_sha256 = hashlib.sha256(copied_path.read_bytes()).hexdigest()
        except OSError:
            return None, observation, f"image_{binary_name}_binary_resolution_failed:terminal_unreadable"
        observation["size_bytes"] = size_bytes
        observation["actual_sha256"] = actual_sha256
        if actual_sha256 != expected_sha256:
            return None, observation, f"image_{binary_name}_binary_sha256_mismatch"
        return actual_sha256, observation, None
    return None, observation, f"image_{binary_name}_binary_resolution_failed:too_many_symlink_hops"


def _container_planner_binary_attestation(
    repo: Path, container_name: str, destination: Path, expected_sha256: str
) -> tuple[str | None, dict[str, Any], str | None]:
    return _container_binary_attestation(
        repo, container_name, destination, expected_sha256,
        install_root=PLANNER_INSTALL_ROOT, build_target=PLANNER_BUILD_TARGET,
        executable_relative_path=PurePosixPath(
            "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"
        ), binary_name="planner",
    )


def _container_pp_binary_attestation(
    repo: Path, container_name: str, destination: Path, expected_sha256: str
) -> tuple[str | None, dict[str, Any], str | None]:
    return _container_binary_attestation(
        repo, container_name, destination, expected_sha256,
        install_root=PP_INSTALL_ROOT, build_target=PP_BUILD_TARGET,
        executable_relative_path=PurePosixPath("lib/simple_pure_pursuit/simple_pure_pursuit"),
        binary_name="pp",
    )


def _v2_uptake_image_hashes(
    repo: Path, container_name: str, run_dir: Path, prearm: dict[str, Any]
) -> tuple[dict[str, str] | None, dict[str, Any], dict[str, Any], str | None]:
    """Read stopped-container binaries/sources before start via docker cp."""
    container_paths = {
        "planner_source_sha256": "/aichallenge/workspace/src/aichallenge_submit/state_lattice_overtake_planner/src/state_lattice_overtake_planner_node.cpp",
        "pp_source_sha256": "/aichallenge/workspace/src/aichallenge_submit/simple_pure_pursuit/src/simple_pure_pursuit.cpp",
        "planner_binary_sha256": "/aichallenge/workspace/install/state_lattice_overtake_planner/lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node",
        "pp_binary_sha256": "/aichallenge/workspace/install/simple_pure_pursuit/lib/simple_pure_pursuit/simple_pure_pursuit",
        "observer_sha256": "/opt/aic_test/v2_uptake_observer.py",
        "driver_sha256": "/opt/aic_test/v2_direct_input_driver.py",
    }
    destination = run_dir / "image-attestation"
    destination.mkdir(exist_ok=True)
    actual: dict[str, str] = {}
    planner_observation: dict[str, Any] = {}
    pp_observation: dict[str, Any] = {}
    for name, source in container_paths.items():
        if name == "planner_binary_sha256":
            actual_hash, planner_observation, error = _container_planner_binary_attestation(
                repo, container_name, destination, prearm["expected_hashes"][name]
            )
            if error is not None:
                return None, planner_observation, pp_observation, error
            assert actual_hash is not None
            actual[name] = actual_hash
            continue
        if name == "pp_binary_sha256":
            actual_hash, pp_observation, error = _container_pp_binary_attestation(
                repo, container_name, destination, prearm["expected_hashes"][name]
            )
            if error is not None:
                return None, planner_observation, pp_observation, error
            assert actual_hash is not None
            actual[name] = actual_hash
            continue
        target = destination / name
        copied = _run(["docker", "cp", f"{container_name}:{source}", str(target)], cwd=repo, timeout=10)
        actual_hash = _regular_file_sha256(target) if copied.returncode == 0 else None
        if actual_hash != prearm["expected_hashes"][name]:
            return None, planner_observation, pp_observation, f"image_{name}_mismatch"
        actual[name] = actual_hash
    return actual, planner_observation, pp_observation, None


def _observer_bounded_deadline_chain_valid(observer_payload: object) -> bool:
    """Accept only a structurally complete, bounded observer deadline chain."""
    if not isinstance(observer_payload, dict):
        return False
    stream_state = observer_payload.get("stream_state")
    evidence = observer_payload.get("evidence")
    if not isinstance(stream_state, dict) or not isinstance(evidence, dict):
        return False
    retained_count = stream_state.get("retained_s0_window_count")
    retained_identity_count = stream_state.get("retained_identity_count")
    capacity = stream_state.get("capacity")
    availability_cycles = evidence.get("availability_cycles")
    if (
        type(retained_count) is not int
        or type(retained_identity_count) is not int
        or type(capacity) is not int
        or not isinstance(availability_cycles, list)
    ):
        return False
    return (
        1 <= retained_count <= MAX_OBSERVER_DEADLINE_CHAIN_CYCLES
        and 0 <= retained_identity_count <= capacity
        and capacity > 0
        and 1 <= len(availability_cycles) <= retained_count
        and all(isinstance(cycle, dict) for cycle in availability_cycles)
        and stream_state.get("active_cycle_present") is False
        and stream_state.get("pending_cycle_sequence") is None
        and stream_state.get("pending_cycle_event_count") == 0
        and stream_state.get("capacity_overflowed") is False
    )


def _xml_child(
    element: ElementTree.Element | None, name: str
) -> ElementTree.Element | None:
    if element is None:
        return None
    return next((child for child in element if child.tag == f"{{{CYCLONEDDS_NAMESPACE}}}{name}"), None)


def _xml_children(
    element: ElementTree.Element | None, name: str
) -> list[ElementTree.Element]:
    if element is None:
        return []
    return [
        child for child in element if child.tag == f"{{{CYCLONEDDS_NAMESPACE}}}{name}"
    ]


def _cyclonedds_config_path(repo: Path) -> tuple[Path | None, dict[str, Any]]:
    uri = os.environ.get("CYCLONEDDS_URI")
    if uri is None:
        candidate = repo / "vehicle" / "cyclonedds.xml"
        source = {
            "source": "repository_default",
            "host_cyclonedds_uri": None,
            "container_cyclonedds_uri": "file:///opt/autoware/cyclonedds.xml",
        }
    else:
        parsed = urlparse(uri)
        if (
            parsed.scheme != "file"
            or not parsed.path
            or not Path(unquote(parsed.path)).is_absolute()
            or parsed.netloc not in {"", "localhost"}
            or parsed.params
            or parsed.query
            or parsed.fragment
        ):
            return None, {
                "source": "host_environment",
                "host_cyclonedds_uri": uri,
                "error": "unsupported_or_relative_cyclonedds_uri",
            }
        candidate = Path(unquote(parsed.path))
        source = {
            "source": "host_environment",
            "host_cyclonedds_uri": uri,
            "container_cyclonedds_uri": "file:///opt/autoware/cyclonedds.xml",
        }
    try:
        if stat.S_ISLNK(candidate.lstat().st_mode):
            source["error"] = "cyclonedds_config_final_component_symlink"
            return None, source
        path = candidate.resolve(strict=True)
    except OSError:
        source["error"] = "cyclonedds_config_missing"
        return None, source
    if not path.is_file():
        source["error"] = "cyclonedds_config_not_regular_file"
        return None, source
    return path, source


def _single_xml_child(
    element: ElementTree.Element | None, name: str, error: str
) -> tuple[ElementTree.Element | None, str | None]:
    children = _xml_children(element, name)
    if len(children) > 1:
        return None, error
    return (children[0] if children else None), None


def _single_xml_text(
    element: ElementTree.Element | None, name: str, error: str
) -> tuple[str | None, str | None]:
    child, duplicate_error = _single_xml_child(element, name, error)
    if duplicate_error is not None:
        return None, duplicate_error
    if child is None:
        return None, None
    return (child.text.strip() if child.text is not None else ""), None


def _nonnegative_integer(
    value: str | None, default: int | None, error: str
) -> tuple[int | None, str | None]:
    if value is None:
        return default, None
    if not re.fullmatch(r"[0-9]+", value):
        return None, error
    return int(value), None


def _normalized_sha256(value: object) -> str | None:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9A-Fa-f]{64}", value):
        return None
    return value.lower()


def _cyclonedds_config_hash_matches(evidence: dict[str, Any]) -> bool:
    try:
        expected = _normalized_sha256(evidence["config_sha256"])
        return expected is not None and hashlib.sha256(
            Path(evidence["config_path"]).read_bytes()
        ).hexdigest() == expected
    except (KeyError, OSError):
        return False


def _cyclonedds_domain_admissibility(
    repo: Path, ros_domain_id: int
) -> dict[str, Any]:
    """Resolve one bounded CycloneDDS config and fail closed on ambiguity."""
    path, evidence = _cyclonedds_config_path(repo)
    evidence.update(
        {
            "ros_domain_id": ros_domain_id,
            "domain_config_binding": "DOMAIN_UNUSED",
            "domain_port_admissibility": "DOMAIN_PORT_INADMISSIBLE",
            "admissible": False,
        }
    )
    if ros_domain_id < 0:
        evidence["error"] = "ros_domain_id_negative"
        return evidence
    if path is None:
        return evidence
    evidence["config_path"] = str(path)
    try:
        raw = path.read_bytes()
        evidence["config_sha256"] = hashlib.sha256(raw).hexdigest()
        root = ElementTree.fromstring(raw)
    except (OSError, ElementTree.ParseError) as error:
        evidence["error"] = f"cyclonedds_config_unreadable_or_malformed:{type(error).__name__}"
        return evidence
    if root.tag != CYCLONEDDS_ROOT_TAG:
        evidence["error"] = "cyclonedds_root_namespace_invalid"
        return evidence
    for element in root.iter():
        local_name = element.tag.rsplit("}", 1)[-1]
        if (
            local_name in CYCLONEDDS_SEMANTIC_ELEMENTS
            and element.tag != f"{{{CYCLONEDDS_NAMESPACE}}}{local_name}"
        ):
            evidence["error"] = "cyclonedds_semantic_namespace_invalid"
            return evidence
    domains = _xml_children(root, "Domain")
    for domain in domains:
        if set(domain.attrib) - {"Id"}:
            evidence["error"] = "cyclonedds_domain_attribute_invalid"
            return evidence
    matching = [
        domain
        for domain in domains
        if domain.get("Id", "any") in {"any", str(ros_domain_id)}
    ]
    if len(matching) != 1:
        evidence["error"] = "cyclonedds_domain_binding_ambiguous"
        return evidence
    domain = matching[0]
    evidence["domain_config_binding"] = "DOMAIN_CONFIG_BOUND"
    general, error = _single_xml_child(
        domain, "General", "cyclonedds_duplicate_general"
    )
    if error is not None:
        evidence["error"] = error
        return evidence
    discovery, error = _single_xml_child(
        domain, "Discovery", "cyclonedds_duplicate_discovery"
    )
    if error is not None:
        evidence["error"] = error
        return evidence
    ports, error = _single_xml_child(
        discovery, "Ports", "cyclonedds_duplicate_ports"
    )
    if error is not None:
        evidence["error"] = error
        return evidence
    port_names = {
        "base": "Base",
        "domain_gain": "DomainGain",
        "multicast_meta_offset": "MulticastMetaOffset",
        "multicast_data_offset": "MulticastDataOffset",
        "participant_gain": "ParticipantGain",
        "unicast_meta_offset": "UnicastMetaOffset",
        "unicast_data_offset": "UnicastDataOffset",
    }
    effective_ports = {}
    for key, default in CYCLONEDDS_DEFAULT_PORTS.items():
        value, error = _single_xml_text(
            ports, port_names[key], "cyclonedds_duplicate_port_field"
        )
        if error is not None:
            evidence["error"] = error
            return evidence
        parsed, parse_error = _nonnegative_integer(
            value, default, "cyclonedds_port_value_malformed"
        )
        if parse_error is not None:
            evidence["error"] = parse_error
            return evidence
        effective_ports[key] = parsed
    evidence["effective_ports"] = effective_ports
    external_domain, error = _single_xml_text(
        discovery, "ExternalDomainId", "cyclonedds_duplicate_external_domain_id"
    )
    if error is not None:
        evidence["error"] = error
        return evidence
    if external_domain is None or external_domain.lower() == "default":
        effective_domain_id = ros_domain_id
    else:
        effective_domain_id, parse_error = _nonnegative_integer(
            external_domain, None, "cyclonedds_external_domain_id_unsupported"
        )
        if parse_error is not None:
            evidence["error"] = parse_error
            return evidence
    evidence["effective_domain_id"] = effective_domain_id
    domain_base = effective_ports["base"] + effective_domain_id * effective_ports["domain_gain"]
    evidence["computed_domain_base"] = domain_base
    participant_index, error = _single_xml_text(
        discovery, "ParticipantIndex", "cyclonedds_duplicate_participant_index"
    )
    if error is not None:
        evidence["error"] = error
        return evidence
    max_auto, error = _single_xml_text(
        discovery,
        "MaxAutoParticipantIndex",
        "cyclonedds_duplicate_max_auto_participant_index",
    )
    if error is not None:
        evidence["error"] = error
        return evidence
    max_index, parse_error = _nonnegative_integer(
        max_auto, 99, "cyclonedds_max_auto_participant_index_invalid"
    )
    if parse_error is not None or max_index is None or max_index > 1024:
        evidence["error"] = "cyclonedds_max_auto_participant_index_invalid"
        return evidence
    evidence["max_auto_participant_index"] = max_index
    requested_index = "default" if participant_index is None else participant_index.lower()
    if requested_index == "default":
        modes: list[str | int] = ["none", "auto"]
        evidence["participant_index_source"] = "default_possible_modes"
    elif requested_index in {"none", "auto"}:
        modes = [requested_index]
        evidence["participant_index_source"] = "explicit"
    else:
        index, parse_error = _nonnegative_integer(
            requested_index, None, "cyclonedds_participant_index_unsupported"
        )
        if parse_error is not None or index is None:
            evidence["error"] = "cyclonedds_participant_index_unsupported"
            return evidence
        modes = [index]
        evidence["participant_index_source"] = "explicit"

    multicast_ports = {
        "multicast_meta": domain_base + effective_ports["multicast_meta_offset"],
        "multicast_data": domain_base + effective_ports["multicast_data_offset"],
    }

    def ports_for_mode(mode: str | int) -> list[dict[str, int]]:
        if mode == "none":
            return [multicast_ports]
        indexes = range(max_index + 1) if mode == "auto" else [int(mode)]
        return [
            {
                **multicast_ports,
                "unicast_meta": domain_base
                + effective_ports["unicast_meta_offset"]
                + index * effective_ports["participant_gain"],
                "unicast_data": domain_base
                + effective_ports["unicast_data_offset"]
                + index * effective_ports["participant_gain"],
            }
            for index in indexes
        ]

    branch_evidence = {}
    for mode in modes:
        candidates = ports_for_mode(mode)
        valid = all(
            all(1 <= port <= MAX_UDP_PORT for port in candidate.values())
            and len(set(candidate.values())) == len(candidate)
            for candidate in candidates
        )
        branch_evidence[str(mode)] = {
            "admissible": valid,
            "candidate_count": len(candidates),
            "first_candidate_ports": candidates[0],
        }
    evidence["participant_index"] = requested_index
    evidence["participant_index_branches"] = branch_evidence
    evidence["computed_ports"] = branch_evidence
    if all(branch["admissible"] for branch in branch_evidence.values()):
        evidence["domain_port_admissibility"] = "DOMAIN_PORT_ADMISSIBLE"
        evidence["admissible"] = True
    else:
        evidence["error"] = "cyclonedds_domain_port_out_of_range_or_collision"
    return evidence


def _write_host_readable_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    """Write a V2 handoff JSON atomically with host-readable permissions."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False,
            prefix=f".{path.name}.", suffix=".tmp",
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o644)
            json.dump(
                {"schema_version": SCHEMA_VERSION, **payload},
                stream,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        if stat.S_IMODE(path.stat().st_mode) != 0o644:
            raise OSError(f"host-readable mode mismatch: {path}")
        with path.open(encoding="utf-8") as stream:
            json.load(stream)
    except Exception:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        raise


def private_prelaunch_start_decision(request_text: str | None, permit: dict[str, Any] | None, *, expected_nonce: str, observer_alive: bool, pp_alive: bool) -> bool:
    """Fixture-only final Planner-start gate; false means no start sentinel."""
    if not observer_alive or not pp_alive or request_text is None:
        return False
    try:
        request = json.loads(request_text)
    except (TypeError, json.JSONDecodeError):
        return False
    if request != {"schema_version": 1, "run_nonce": expected_nonce} or not isinstance(permit, dict):
        return False
    required_keys = {
        "schema_version",
        "run_nonce",
        "epoch_boundary",
        "proposal_count",
        "status_fault",
        "blocking_reasons",
        "experimental_epoch_status_ordinal",
        "permit_status_ordinal",
    }
    return (
        set(permit) == required_keys
        and permit["schema_version"] == 1
        and permit["run_nonce"] == expected_nonce
        and permit["epoch_boundary"] == "observer_prelaunch_permit"
        and permit["proposal_count"] == 0
        and permit["status_fault"] is False
        and permit["blocking_reasons"] == []
        and isinstance(permit["experimental_epoch_status_ordinal"], int)
        and not isinstance(permit["experimental_epoch_status_ordinal"], bool)
        and permit["experimental_epoch_status_ordinal"] > 0
        and isinstance(permit["permit_status_ordinal"], int)
        and not isinstance(permit["permit_status_ordinal"], bool)
        and permit["permit_status_ordinal"] >= permit["experimental_epoch_status_ordinal"]
    )


def private_prelaunch_start_files_valid(
    request_path: Path,
    permit_path: Path,
    *,
    expected_nonce: str,
    observer_pid: int,
    pp_pid: int,
) -> bool:
    """Runtime-owned final gate shared by the generated runner and tests."""
    try:
        request_text = request_path.read_text(encoding="utf-8")
    except OSError:
        request_text = None
    permit = _read_host_readable_json(permit_path)

    def alive(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except (OSError, ValueError):
            return False
        return True

    return private_prelaunch_start_decision(
        request_text,
        permit,
        expected_nonce=expected_nonce,
        observer_alive=alive(observer_pid),
        pp_alive=alive(pp_pid),
    )


def _read_host_readable_json(path: Path) -> dict[str, Any] | None:
    try:
        if not path.is_file() or stat.S_IMODE(path.stat().st_mode) != 0o644:
            return None
        with path.open(encoding="utf-8") as stream:
            payload = json.load(stream)
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _run(
    command: Sequence[str],
    *,
    cwd: Path,
    env: dict[str, str] | None = None,
    timeout: float | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(command),
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )


def _repo_root(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit).resolve()
    return Path(__file__).resolve().parents[3]


def _new_output_dir(repo: Path, started_wall_ns: int) -> Path | None:
    candidates = [
        path
        for path in (repo / "output").glob("20*")
        if path.is_dir() and path.stat().st_mtime_ns >= started_wall_ns
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime_ns, default=None)


def _container_id(repo: Path) -> str:
    result = _run(
        ["docker", "compose", "ps", "-q", SERVICE], cwd=repo, timeout=10
    )
    return result.stdout.strip() if result.returncode == 0 else ""


def _direct_container_name(run_id: str) -> str:
    return "aic-test-v2-" + re.sub(r"[^a-zA-Z0-9_.-]", "-", run_id).lower()


def _container_exists(repo: Path, name: str) -> bool:
    return _run(
        ["docker", "container", "inspect", name], cwd=repo, timeout=10
    ).returncode == 0


def _v2_uptake_doctor_residue(repo: Path, container_name: str) -> tuple[dict[str, Any], str | None]:
    """Read only the direct-run residue that can corrupt this V2 arm edge."""
    evidence: dict[str, Any] = {
        "evaluation_container_running": bool(_container_id(repo)),
        "selected_container_exists": _container_exists(repo, container_name),
        "active_v2_containers": [],
        "relevant_host_processes": [],
    }
    containers = _run(
        ["docker", "container", "ls", "-a", "--filter", "name=aic-test-v2-", "--format", "{{.Names}}"],
        cwd=repo, timeout=10,
    )
    if containers.returncode != 0:
        return evidence, "v2_uptake_doctor_container_scan_failed"
    evidence["active_v2_containers"] = [
        name for name in containers.stdout.splitlines()
        if re.fullmatch(r"aic-test-v2-[A-Za-z0-9_.-]+", name)
    ]
    processes = _run(["ps", "-eo", "pid=,args="], cwd=repo, timeout=10)
    if processes.returncode != 0:
        return evidence, "v2_uptake_doctor_process_scan_failed"
    watched = {
        "state_lattice_overtake_planner_node",
        "simple_pure_pursuit",
        "v2_uptake_observer.py",
        "v2_direct_input_driver.py",
    }
    for line in processes.stdout.splitlines():
        pid, _, command = line.strip().partition(" ")
        try:
            tokens = shlex.split(command)
        except ValueError:
            continue
        if any(Path(token).name in watched for token in tokens):
            evidence["relevant_host_processes"].append({"pid": pid, "argv": tokens})
    if evidence["evaluation_container_running"] or evidence["selected_container_exists"] or evidence["active_v2_containers"] or evidence["relevant_host_processes"]:
        return evidence, "v2_uptake_doctor_residue_present"
    return evidence, None


def _cleanup_direct_container(repo: Path, name: str) -> dict[str, Any]:
    """Boundedly remove only this task-created container after an exception."""
    stopped = _run(["docker", "stop", "--time", "5", name], cwd=repo, timeout=15)
    removed = _run(["docker", "rm", "--force", name], cwd=repo, timeout=15)
    return {
        "container_name": name,
        "stop_returncode": stopped.returncode,
        "rm_returncode": removed.returncode,
        "absent_after_cleanup": not _container_exists(repo, name),
    }


def _git_fingerprint(repo: Path) -> dict[str, Any]:
    head = _run(["git", "rev-parse", "HEAD"], cwd=repo, timeout=10)
    status = _run(["git", "status", "--porcelain"], cwd=repo, timeout=10)
    archive = repo / "submit" / "aichallenge_submit.tar.gz"
    archive_hash = None
    if archive.is_file():
        digest = _run(["sha256sum", str(archive)], cwd=repo, timeout=10)
        if digest.returncode == 0:
            archive_hash = digest.stdout.split()[0]
    image = _run(
        [
            "docker",
            "image",
            "inspect",
            "aichallenge-2025-eval",
            "--format",
            "{{.Id}} {{.Created}}",
        ],
        cwd=repo,
        timeout=10,
    )
    return {
        "git_head": head.stdout.strip() if head.returncode == 0 else None,
        "git_dirty": bool(status.stdout.strip()),
        "submit_archive_sha256": archive_hash,
        "eval_image": image.stdout.strip() if image.returncode == 0 else None,
    }


def _base_result(run_id: str, run_dir: Path, repo: Path) -> dict[str, Any]:
    return {
        "run_id": run_id,
        "scenario": "control-smoke",
        "outcome": "INFRASTRUCTURE_FAILED",
        "reasons": [],
        "runtime": {
            "control_method": "state_lattice_pure_pursuit",
            "run_rviz": False,
            "camera": False,
            "lidar": False,
            "v2_live_proposal_publish_enabled": False,
            "v2_live_proposal_accept_enabled": False,
            "ros_domain_id": 1,
        },
        "fingerprint": _git_fingerprint(repo),
        "observer": None,
        "predicates": {},
        "artifacts": {
            "result": str(run_dir / "result.json"),
            "launcher_log": str(run_dir / "launcher.log"),
            "observer_log": str(run_dir / "observer.log"),
            "observer_evidence": str(run_dir / "observer.json"),
            "observer_first_fault": str(run_dir / "observer_first_fault.json"),
            "runtime_output": None,
            "autoware_log": None,
            "rosbag_metadata": None,
        },
    }


def run_control_smoke(args: argparse.Namespace) -> int:
    repo = _repo_root(args.repo_root)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_id = args.run_id or f"{timestamp}-control-smoke"
    run_dir = repo / "analysis" / "aic_test" / "runs" / run_id
    result_path = run_dir / "result.json"
    run_dir.mkdir(parents=True, exist_ok=False)
    payload = _base_result(run_id, run_dir, repo)
    reasons: list[str] = payload["reasons"]
    launcher_lines: list[str] = []
    started_wall_ns = time.time_ns()

    free_bytes = shutil.disk_usage(repo).free
    payload["predicates"]["disk_free_ge_5_gib"] = free_bytes >= 5 * 1024**3
    if not payload["predicates"]["disk_free_ge_5_gib"]:
        reasons.append("insufficient_disk_space")
        write_json_atomic(result_path, payload)
        print(result_path)
        return 3

    if _container_id(repo):
        reasons.append("evaluation_container_already_running")
        write_json_atomic(result_path, payload)
        print(result_path)
        return 3

    env = os.environ.copy()
    env.update(
        {
            "CONTROL_METHOD": "state_lattice_pure_pursuit",
            "RUN_RVIZ": "false",
            "AWSIM_EXTRA_ARGS": (
                "-batchmode -nographics --camera false --lidar false"
            ),
            "AWSIM_VEHICLES": "1",
            "AWSIM_LAPS": "1",
            "AWSIM_TIMEOUT": str(args.awsim_timeout),
            "AWSIM_START_COUNT_SECONDS": "3",
            "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED": "false",
            "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED": "false",
            "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID": "0",
            "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID": "",
            "STATE_LATTICE_V2_SESSION_ID": "",
        }
    )

    try:
        launch = _run(["make", "eval"], cwd=repo, env=env, timeout=30)
        launcher_lines.append(launch.stdout)
        if launch.returncode != 0:
            reasons.append("make_eval_failed")
            return_code = 3
        else:
            return_code = 3
            deadline = time.monotonic() + args.startup_timeout
            container = ""
            output_dir = None
            while time.monotonic() < deadline:
                container = _container_id(repo)
                output_dir = _new_output_dir(repo, started_wall_ns)
                if container and output_dir and (output_dir / "d1/autoware.log").is_file():
                    break
                time.sleep(1.0)
            if not container or output_dir is None:
                reasons.append("runtime_did_not_start")
            else:
                payload["artifacts"]["runtime_output"] = str(output_dir)
                autoware_log = output_dir / "d1" / "autoware.log"
                payload["artifacts"]["autoware_log"] = str(autoware_log)
                startup_deadline = time.monotonic() + args.startup_timeout
                while time.monotonic() < startup_deadline:
                    log_text = autoware_log.read_text(
                        encoding="utf-8", errors="replace"
                    )
                    if any(marker in log_text for marker in FATAL_LOG_MARKERS):
                        break
                    if "state_lattice_overtake_planner_node" in log_text and (
                        "simple_pure_pursuit_node" in log_text
                    ):
                        break
                    time.sleep(1.0)

                observer = _run(
                    [
                        "docker",
                        "exec",
                        "-e",
                        "ROS_DOMAIN_ID=1",
                        container,
                        "bash",
                        "-lc",
                        (
                            "source /autoware/install/setup.bash && "
                            "source /aichallenge/workspace/install/setup.bash && "
                            "exec python3 /opt/aic_test/observer.py "
                            f"--duration {args.observe_duration} "
                            "--expected-domain-id 1"
                        ),
                    ],
                    cwd=repo,
                    timeout=args.observe_duration + 15,
                )
                (run_dir / "observer.log").write_text(
                    observer.stdout, encoding="utf-8"
                )
                observer_payload = None
                for line in reversed(observer.stdout.splitlines()):
                    try:
                        candidate = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if isinstance(candidate, dict) and "outcome" in candidate:
                        observer_payload = candidate
                        break
                payload["observer"] = observer_payload
                if observer_payload is None:
                    reasons.append("observer_result_missing")
                else:
                    write_json_atomic(run_dir / "observer.json", observer_payload)
                    if observer_payload.get("outcome") != "PASS":
                        reasons.append("observer_invalid_evidence")
                    elif observer.returncode != 0:
                        reasons.append("observer_failed")
    except (OSError, subprocess.TimeoutExpired) as error:
        reasons.append(f"runtime_exception:{type(error).__name__}")
        return_code = 3
    finally:
        stopped = _run(["make", "down"], cwd=repo, env=env, timeout=210)
        launcher_lines.append(stopped.stdout)
        (run_dir / "launcher.log").write_text(
            "\n".join(launcher_lines), encoding="utf-8"
        )

    output_value = payload["artifacts"]["runtime_output"]
    if output_value:
        output_dir = Path(output_value)
        autoware_log = output_dir / "d1" / "autoware.log"
        log_text = (
            autoware_log.read_text(encoding="utf-8", errors="replace")
            if autoware_log.is_file()
            else ""
        )
        metadata = output_dir / "d1" / "rosbag2_autoware" / "metadata.yaml"
        predicates = payload["predicates"]
        predicates.update(
            {
                "control_method_selected": (
                    "control_method: state_lattice_pure_pursuit" in log_text
                ),
                "rviz_disabled": (
                    "run_rviz: false" in log_text
                    and "[rviz2-" not in log_text
                ),
                "headless_sensor_flags_selected": (
                    "awsim_extra_args: -batchmode -nographics --camera false --lidar false"
                    in log_text
                ),
                "v2_defaults_off": (
                    payload["runtime"]["v2_live_proposal_publish_enabled"] is False
                    and payload["runtime"]["v2_live_proposal_accept_enabled"] is False
                ),
                "required_processes_started": all(
                    marker in log_text
                    for marker in (
                        "[state_lattice_overtake_planner_node-",
                        "[simple_pure_pursuit-",
                        "[hybrid_control_mux_node.py-",
                    )
                ),
                "no_fatal_runtime_marker": not any(
                    marker in log_text for marker in FATAL_LOG_MARKERS
                ),
                "observer_passed": (
                    isinstance(payload["observer"], dict)
                    and payload["observer"].get("outcome") == "PASS"
                ),
                "observer_evidence_present": (
                    run_dir.joinpath("observer.json").is_file()
                ),
            }
        )
        payload["official_rosbag"] = {
            "available": metadata.is_file(),
            "required_for_phase1_smoke": False,
            "reason_if_unavailable": (
                None
                if metadata.is_file()
                else "official recorder starts later in the race lifecycle"
            ),
        }
        if metadata.is_file():
            payload["artifacts"]["rosbag_metadata"] = str(metadata)
        failed = sorted(name for name, value in predicates.items() if not value)
        reasons.extend(reason for reason in failed if reason not in reasons)

    if not reasons and payload["predicates"]:
        payload["outcome"] = "PASS"
        return_code = 0
    elif payload["observer"] and payload["observer"].get("outcome") == "INVALID_EVIDENCE":
        payload["outcome"] = "INVALID_EVIDENCE"
        return_code = 4
    else:
        payload["outcome"] = "INFRASTRUCTURE_FAILED"
        return_code = 3
    payload["reasons"] = sorted(set(reasons))
    write_json_atomic(result_path, payload)
    if args.json:
        print(json.dumps(read_result(result_path), ensure_ascii=False, allow_nan=False))
    else:
        print(f"{payload['outcome']}: {result_path}")
    return return_code


def run_v2_uptake_smoke(args: argparse.Namespace) -> int:
    # This must precede repository/state/doctor reads: -O/-OO disables Python
    # assertions used by generated runtime evidence helpers.
    if sys.flags.optimize != 0:
        print(json.dumps({
            "scenario": "v2-uptake-smoke", "outcome": "PRECONDITION_NOT_MET",
            "reasons": ["python_optimize_enabled"],
        }, ensure_ascii=False, allow_nan=False))
        return 3
    repo = _repo_root(args.repo_root)
    final_fence_profile = (
        args.scenario == "v2-uptake-final-fence-smoke"
    )
    cyclonedds_evidence = _cyclonedds_domain_admissibility(
        repo, args.ros_domain_id
    )
    if cyclonedds_evidence["admissible"] and not _cyclonedds_config_hash_matches(
        cyclonedds_evidence
    ):
        cyclonedds_evidence["admissible"] = False
        cyclonedds_evidence["domain_port_admissibility"] = (
            "DOMAIN_PORT_INADMISSIBLE"
        )
        cyclonedds_evidence["error"] = "cyclonedds_config_hash_mismatch"
    if not cyclonedds_evidence["admissible"]:
        preflight = {
            "scenario": args.scenario,
            "outcome": "PRECONDITION_NOT_MET",
            "reasons": [
                cyclonedds_evidence.get(
                    "error", "cyclonedds_domain_port_inadmissible"
                )
            ],
            "predicates": {"cyclonedds_domain_admissible": False},
            "cyclonedds": cyclonedds_evidence,
        }
        print(json.dumps(preflight, ensure_ascii=False, allow_nan=False))
        return 3
    prearm, prearm_error = _v2_uptake_static_prearm(
        repo, args.ros_domain_id, args.runtime_timeout
    )
    if prearm_error is not None:
        print(json.dumps({
            "scenario": "v2-uptake-smoke", "outcome": "PRECONDITION_NOT_MET",
            "reasons": [prearm_error], "predicates": {"static_prearm": False},
        }, ensure_ascii=False, allow_nan=False))
        return 3
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_id = args.run_id or f"{timestamp}-v2-uptake-smoke"
    container_name = _direct_container_name(run_id)
    run_dir = repo / "analysis" / "aic_test" / "runs" / run_id
    result_path = run_dir / "result.json"
    run_dir.mkdir(parents=True, exist_ok=False)
    udp_socket_preflight, udp_socket_error = _v2_uptake_udp_socket_preflight(
        cyclonedds_evidence
    )
    if udp_socket_error is not None:
        preflight = {
            "run_id": run_id,
            "scenario": "v2-uptake-smoke",
            "outcome": "PRECONDITION_NOT_MET",
            "reasons": [udp_socket_error],
            "predicates": {"udp_socket_candidates_free": False},
            "udp_socket_preflight": udp_socket_preflight,
        }
        write_json_atomic(result_path, preflight)
        if args.json:
            print(json.dumps(preflight, ensure_ascii=False, allow_nan=False))
        else:
            print(f"{preflight['outcome']}: {result_path}")
        return 3
    payload = {
        "run_id": run_id,
        "scenario": args.scenario,
        "outcome": "INFRASTRUCTURE_FAILED",
        "reasons": [],
        "maximum_label": "PP_UPTAKE_AVAILABILITY_VERIFIED_NON_AUTHORITATIVE",
        "qualifier": (
            "deadline_bounded_continuous_pp_cycle_availability_with_exact_replacement; "
            "pp_application_mux_m4_not_verified"
        ),
        "runtime": {
            "kind": "direct_ros_production_binaries",
            "ros_domain_id": args.ros_domain_id,
            "planner_live_control_output": False,
            "planner_instant_control": False,
            "planner_controller_trackability_profile": "shadow_only",
            "v2_explicitly_enabled_for_this_scenario": True,
            "v2_final_fence_diagnostic_enabled": final_fence_profile,
            "wall_lifetime_s": 30,
            "container_name": container_name,
            "cyclonedds": cyclonedds_evidence,
        },
        "udp_socket_preflight": udp_socket_preflight,
        "fingerprint": {
            "git_head": _run(["git", "rev-parse", "HEAD"], cwd=repo, timeout=10).stdout.strip(),
            "git_dirty": bool(_run(["git", "status", "--porcelain"], cwd=repo, timeout=10).stdout.strip()),
        },
        "prearm": prearm,
        "observer": None,
        "container_cleanup": None,
        "predicates": {},
        "artifacts": {
            "result": str(result_path),
            "observer_log": str(run_dir / "observer.log"),
            "observer_evidence": str(run_dir / "observer.json"),
            "input_driver_log": str(run_dir / "input_driver.log"),
            "input_primed": str(run_dir / "input_primed.json"),
            "capture_complete": str(run_dir / "capture_complete.json"),
            "pp_binding_cycle": str(run_dir / "pp_binding_cycle.json"),
            "effective_time_parameters": str(
                run_dir / "effective_time_parameters.json"
            ),
            "planner_log": str(run_dir / "planner.log"),
            "pure_pursuit_log": str(run_dir / "pure_pursuit.log"),
            "component_exit_statuses": str(run_dir / "component_exit_statuses.json"),
            "prelaunch_binding": str(run_dir / "prelaunch-binding.json"),
        },
    }
    reasons: list[str] = payload["reasons"]
    # This scenario reuses an already-built image and writes only bounded logs.
    minimum_free_bytes = 512 * 1024**2
    payload["runtime"]["minimum_free_bytes"] = minimum_free_bytes
    payload["runtime"]["free_bytes_at_preflight"] = shutil.disk_usage(repo).free
    if payload["runtime"]["free_bytes_at_preflight"] < minimum_free_bytes:
        reasons.append("insufficient_disk_space")
    cyclonedds_config = Path(cyclonedds_evidence["config_path"])
    if reasons:
        _write_host_readable_json_atomic(result_path, payload)
        print(f"{payload['outcome']}: {result_path}")
        return 3

    # Read-only Docker residue is a static predicate. It must fail before the
    # reviewed one-shot budget is consumed.
    doctor_residue, doctor_error = _v2_uptake_doctor_residue(repo, container_name)
    payload["doctor_residue"] = doctor_residue
    if doctor_error is not None:
        reasons.append(doctor_error)
    if reasons:
        _write_host_readable_json_atomic(result_path, payload)
        print(f"{payload['outcome']}: {result_path}")
        return 3
    budget_guard, budget_error = _consume_v2_uptake_one_shot_budget(
        repo, run_id, prearm
    )
    if budget_error is not None:
        reasons.append(budget_error)
        _write_host_readable_json_atomic(result_path, payload)
        print(f"{payload['outcome']}: {result_path}")
        return 3
    payload["one_shot_budget_guard"] = str(budget_guard)
    state_hash_before, state_hash_after = _consume_v2_uptake_state_budget(prearm)
    if state_hash_before is None:
        reasons.append(state_hash_after or "review_state_budget_transition_failed")
        _write_host_readable_json_atomic(result_path, payload)
        print(f"{payload['outcome']}: {result_path}")
        return 3
    payload["review_state_budget_transition"] = {
        "before_sha256": state_hash_before, "after_sha256": state_hash_after,
    }

    container_script = """
set -euo pipefail
readonly EXPECTED_CYCLONEDDS_URI="__EXPECTED_CYCLONEDDS_URI__"
readonly EXPECTED_CYCLONEDDS_SHA256="__EXPECTED_CYCLONEDDS_SHA256__"
test "${CYCLONEDDS_URI:-}" = "${EXPECTED_CYCLONEDDS_URI}"
test -f /opt/autoware/cyclonedds.xml
test ! -L /opt/autoware/cyclonedds.xml
actual_cyclonedds_sha256="$(sha256sum /opt/autoware/cyclonedds.xml | awk '{print $1}')"
test -n "${actual_cyclonedds_sha256}"
[[ "${EXPECTED_CYCLONEDDS_SHA256}" =~ ^[0-9a-f]{64}$ ]]
[[ "${actual_cyclonedds_sha256}" =~ ^[0-9a-f]{64}$ ]]
test "${EXPECTED_CYCLONEDDS_SHA256}" != "0000000000000000000000000000000000000000000000000000000000000000"
test "${actual_cyclonedds_sha256}" = "${EXPECTED_CYCLONEDDS_SHA256}"
verify_strict_mode() {
  case "$-" in *e*) ;; *) return 1;; esac
  [[ -o nounset ]] && [[ -o pipefail ]]
}
attest_cyclonedds_after_sources() {
  test "${CYCLONEDDS_URI:-}" = "${EXPECTED_CYCLONEDDS_URI}"
  test -f /opt/autoware/cyclonedds.xml
  test ! -L /opt/autoware/cyclonedds.xml
  actual_cyclonedds_sha256="$(sha256sum /opt/autoware/cyclonedds.xml | awk '{print $1}')"
  test -n "${actual_cyclonedds_sha256}"
  [[ "${EXPECTED_CYCLONEDDS_SHA256}" =~ ^[0-9a-f]{64}$ ]]
  [[ "${actual_cyclonedds_sha256}" =~ ^[0-9a-f]{64}$ ]]
  test "${EXPECTED_CYCLONEDDS_SHA256}" != "0000000000000000000000000000000000000000000000000000000000000000"
  test "${actual_cyclonedds_sha256}" = "${EXPECTED_CYCLONEDDS_SHA256}"
}
attest_cyclonedds_after_sources
mkdir -p /evidence
readonly OBSERVER_LIFETIME_S=36
readonly CAPTURE_LIFETIME_S=30
readonly STARTUP_TIMEOUT_TICKS=50
readonly PRIVATE_INPUT=/aic_test/v2/input
readonly PRIVATE_OUTPUT=/aic_test/v2/output
declare -A component_pid=()
declare -A component_rc=()
declare -A component_alive_before_shutdown=()
declare -A component_residue=()

start_component() {
  local name="$1"
  shift
  setsid "$@" > "/evidence/${name}.log" 2>&1 &
  component_pid["${name}"]=$!
}

shutdown_component() {
  local name="$1" pid rc attempt
  pid="${component_pid[${name}]}"
  if kill -0 "${pid}" 2>/dev/null; then
    component_alive_before_shutdown["${name}"]=true
  else
    component_alive_before_shutdown["${name}"]=false
  fi
  # A setsid group may outlive a failed group leader. Clean the group
  # independently from leader health so container teardown is not the guard.
  if kill -0 -- "-${pid}" 2>/dev/null; then
    kill -TERM -- "-${pid}" 2>/dev/null || true
    for attempt in $(seq 1 50); do
      if ! kill -0 -- "-${pid}" 2>/dev/null; then break; fi
      sleep 0.1
    done
    if kill -0 -- "-${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
  fi
  if wait "${pid}"; then rc=0; else rc=$?; fi
  component_rc["${name}"]="${rc}"
  if kill -0 -- "-${pid}" 2>/dev/null; then
    component_residue["${name}"]=true
  else
    component_residue["${name}"]=false
  fi
}

write_statuses() {
  python3 - <<'PY'
import json
import os
names = ("planner", "pure_pursuit", "input_driver", "observer")
print(json.dumps({name: {"exit_status": int(os.environ.get(name + "_RC", "255")),
                         "alive_before_shutdown": os.environ.get(name + "_ALIVE", "false") == "true",
                         "residue": os.environ.get(name + "_RESIDUE", "true") == "true"}
                  for name in names}, sort_keys=True))
PY
}

finish() {
  for name in input_driver planner pure_pursuit observer; do
    if [[ -n "${component_pid[${name}]:-}" && -z "${component_rc[${name}]:-}" ]]; then
      shutdown_component "${name}"
    fi
  done
  planner_RC="${component_rc[planner]:-255}" planner_ALIVE="${component_alive_before_shutdown[planner]:-false}" \
  planner_RESIDUE="${component_residue[planner]:-true}" \
  pure_pursuit_RC="${component_rc[pure_pursuit]:-255}" pure_pursuit_ALIVE="${component_alive_before_shutdown[pure_pursuit]:-false}" \
  pure_pursuit_RESIDUE="${component_residue[pure_pursuit]:-true}" \
  input_driver_RC="${component_rc[input_driver]:-255}" input_driver_ALIVE="${component_alive_before_shutdown[input_driver]:-false}" \
  input_driver_RESIDUE="${component_residue[input_driver]:-true}" \
  observer_RC="${component_rc[observer]:-255}" observer_ALIVE="${component_alive_before_shutdown[observer]:-false}" \
  observer_RESIDUE="${component_residue[observer]:-true}" \
    write_statuses > /evidence/component_exit_statuses.json
}
cleanup_traps_armed=false
arm_cleanup_traps() {
  trap finish EXIT INT TERM
  cleanup_traps_armed=true
}
verify_cleanup_traps() {
  test "${cleanup_traps_armed}" = true
}
arm_cleanup_traps
verify_cleanup_traps

set +u
source /autoware/install/setup.bash
builtin set -euo pipefail
arm_cleanup_traps
verify_strict_mode
verify_cleanup_traps

set +u
source /aichallenge/workspace/install/setup.bash
builtin set -euo pipefail
arm_cleanup_traps
verify_strict_mode
verify_cleanup_traps

verify_strict_mode
verify_cleanup_traps
attest_cyclonedds_after_sources
start_component observer python3 /opt/aic_test/v2_uptake_observer.py \
  --duration "${OBSERVER_LIFETIME_S}" \
  --ready-file /evidence/observer_ready.json \
  --sealed-file /evidence/observer_sealed.json \
  --capture-complete-file /evidence/capture_complete.json \
  --terminal-graph-file /evidence/observer_terminal_graph.json \
  --first-fault-file /evidence/observer_first_fault.json \
  --pp-binding-cycle-file /evidence/pp_binding_cycle.json \
  --prelaunch-request-file /evidence/pp_prelaunch_request.json \
  --prelaunch-permit-file /evidence/pp_prelaunch_permit.json \
  --expected-run-nonce "${AIC_TEST_RUN_NONCE}" \
  --final-fence-fixed-file /evidence/final_fence_fixed.json \
  --planner-stopped-file /evidence/planner_stopped.json \
  __FINAL_FENCE_OBSERVER_ARGS__ \
  --result-file /evidence/observer.json
for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
  if [[ -f /evidence/observer_ready.json ]]; then break; fi
  sleep 0.1
done
test -f /evidence/observer_ready.json

start_component input_driver python3 /opt/aic_test/v2_direct_input_driver.py \
  --duration "${CAPTURE_LIFETIME_S}" --input-root "${PRIVATE_INPUT}" \
  --run-nonce "${AIC_TEST_RUN_NONCE}" --primed-file /evidence/input_primed.json \
  --supervisor-pid "$$" --terminal-graph-timeout 5 \
  --start-gate-file /evidence/capture_start.json \
  --capture-complete-file /evidence/capture_complete.json \
  --terminal-graph-file /evidence/observer_terminal_graph.json
for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
  if [[ -s /evidence/input_primed.json ]]; then break; fi
  if ! kill -0 "${component_pid[input_driver]}" 2>/dev/null; then break; fi
  sleep 0.1
done
test -s /evidence/input_primed.json
kill -0 "${component_pid[input_driver]}" 2>/dev/null
python3 - "${AIC_TEST_RUN_NONCE}" "${PRIVATE_INPUT}" <<'PY'
import json
import sys

with open("/evidence/input_primed.json", encoding="utf-8") as stream:
    evidence = json.load(stream)
expected_nonce, expected_root = sys.argv[1:]
assert evidence.get("schema_version") == 1
assert evidence.get("run_nonce") == expected_nonce
assert evidence.get("input_root") == expected_root
assert evidence.get("sample_count", 0) >= 3
assert evidence.get("first_clock_ns", 0) > 0
assert evidence.get("last_clock_ns", 0) > evidence["first_clock_ns"]
assert evidence.get("strictly_increasing_clock") is True
assert evidence.get("race_armed") is False
assert evidence.get("baseline_published") == {
    "clock": True,
    "odometry": True,
    "trajectory": True,
    "v2x": True,
}
assert evidence.get("frames") == {
    "odometry": "map",
    "trajectory": "map",
    "v2x": "map",
}
PY

start_component pure_pursuit ros2 run simple_pure_pursuit simple_pure_pursuit --ros-args \
  -r __node:=simple_pure_pursuit_node \
  -p use_sim_time:=true -p state_lattice_v2_live_proposal_accept_enabled:=true \
  -p "state_lattice_v2_expected_producer_instance_id:='7101'" -p state_lattice_v2_base_attestation_publish_enabled:=true \
  -p "state_lattice_v2_base_attestation_producer_instance_id:='7201'" -p "state_lattice_v2_base_attestation_session_id:='1'" \
  -r input/kinematics:=${PRIVATE_INPUT}/kinematics -r input/trajectory:=${PRIVATE_INPUT}/trajectory \
  -r input/overtake_plan:=${PRIVATE_INPUT}/overtake_plan -r input/race_armed:=${PRIVATE_INPUT}/race_armed -r output/control_cmd:=${PRIVATE_OUTPUT}/control_cmd \
  -r output/raw_control_cmd:=${PRIVATE_OUTPUT}/raw_control_cmd -r output/recovery_control_cmd:=${PRIVATE_OUTPUT}/recovery_control_cmd \
  -r output/controller_tracking_status:=${PRIVATE_OUTPUT}/controller_tracking_status \
  -r output/controller_command_envelope:=${PRIVATE_OUTPUT}/controller_command_envelope \
  -r output/controller_execution_envelope:=${PRIVATE_OUTPUT}/controller_execution_envelope \
  -r output/free_run_execution_ack:=${PRIVATE_OUTPUT}/free_run_execution_ack \
  -r output/free_run_source_key:=${PRIVATE_OUTPUT}/free_run_source_key \
  -r /control/debug/lookahead_point:=${PRIVATE_OUTPUT}/lookahead_point -r /pure_pursuit/debug:=${PRIVATE_OUTPUT}/debug
for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
  if [[ -s /evidence/pp_binding_cycle.json ]]; then break; fi
  if ! kill -0 "${component_pid[pure_pursuit]}" 2>/dev/null; then break; fi
  sleep 0.1
done
test -s /evidence/pp_binding_cycle.json
kill -0 "${component_pid[pure_pursuit]}" 2>/dev/null
python3 - "${AIC_TEST_RUN_NONCE}" <<'PY'
import json
import sys

with open("/evidence/pp_binding_cycle.json", encoding="utf-8") as stream:
    evidence = json.load(stream)
assert evidence.get("schema_version") == 1
assert evidence.get("run_nonce") == sys.argv[1]
assert evidence.get("node_fqn") == "/simple_pure_pursuit_node"
assert evidence.get("status_topic") == "/debug/overtake/state_lattice/v2_binding_status"
assert evidence.get("pp_cycle_sequence", 0) > 0
assert evidence.get("timer_entry_ros_ns", 0) > 0
assert evidence.get("header_frame_id") == "map"
assert evidence.get("timer_entry_monotonic_ns", 0) > 0
assert evidence.get("observer_receive_monotonic_ns", 0) > 0
assert evidence.get("proposal_count_at_barrier") == 0
assert evidence.get("normal_status_integrity") is True
PY
python3 - "${AIC_TEST_RUN_NONCE}" <<'PY'
import json
from pathlib import Path
import sys

target = Path("/evidence/pp_prelaunch_request.json")
temporary = target.with_suffix(".tmp")
temporary.write_text(json.dumps({"schema_version": 1, "run_nonce": sys.argv[1]}) + "\\n", encoding="utf-8")
temporary.replace(target)
PY
for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
  if [[ -s /evidence/pp_prelaunch_permit.json ]]; then break; fi
  if ! kill -0 "${component_pid[observer]}" 2>/dev/null; then break; fi
  sleep 0.1
done
test -s /evidence/pp_prelaunch_permit.json
python3 - "${AIC_TEST_RUN_NONCE}" "${component_pid[observer]}" "${component_pid[pure_pursuit]}" <<'PY'
import json
import os
import stat
import sys

expected_nonce, observer_pid, pp_pid = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
def require(condition, reason):
    if not condition:
        raise SystemExit(reason)

try:
    with open("/evidence/pp_prelaunch_request.json", encoding="utf-8") as stream:
        request = json.load(stream)
    permit_path = "/evidence/pp_prelaunch_permit.json"
    permit_stat = os.stat(permit_path)
    require(stat.S_ISREG(permit_stat.st_mode), "permit_not_regular")
    require(stat.S_IMODE(permit_stat.st_mode) == 0o644, "permit_mode_invalid")
    with open(permit_path, encoding="utf-8") as stream:
        permit = json.load(stream)
except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
    raise SystemExit("prelaunch_evidence_invalid") from error

required_keys = {
    "schema_version", "run_nonce", "epoch_boundary", "proposal_count",
    "status_fault", "blocking_reasons", "experimental_epoch_status_ordinal",
    "permit_status_ordinal",
}
require(request == {"schema_version": 1, "run_nonce": expected_nonce}, "request_invalid")
require(isinstance(permit, dict) and set(permit) == required_keys, "permit_shape_invalid")
require(permit["schema_version"] == 1 and permit["run_nonce"] == expected_nonce, "permit_identity_invalid")
require(permit["epoch_boundary"] == "observer_prelaunch_permit", "permit_epoch_invalid")
require(permit["proposal_count"] == 0 and permit["status_fault"] is False, "permit_status_invalid")
require(permit["blocking_reasons"] == [], "permit_blocked")
experimental = permit["experimental_epoch_status_ordinal"]
permit_ordinal = permit["permit_status_ordinal"]
require(type(experimental) is int and experimental > 0, "experimental_ordinal_invalid")
require(type(permit_ordinal) is int and permit_ordinal >= experimental, "permit_ordinal_invalid")
for pid in (observer_pid, pp_pid):
    os.kill(pid, 0)
PY
start_component planner ros2 run state_lattice_overtake_planner state_lattice_overtake_planner_node --ros-args \
  --params-file /aichallenge/workspace/install/state_lattice_overtake_planner/share/state_lattice_overtake_planner/config/state_lattice_overtake_planner.param.yaml \
  -r __node:=state_lattice_overtake_planner_node \
  -p use_sim_time:=true -p live_control_output_enabled:=false -p instant_control_enabled:=false \
  -p controller_trackability_profile:=shadow_only -p safety_evaluation_enabled:=true \
  -p c002ay0_state_lattice_shadow_enabled:=false -p state_lattice_v2_live_proposal_publish_enabled:=true \
  __FINAL_FENCE_PLANNER_ARGS__ \
  -p own_vehicle_id:=d2 -p state_lattice_v2_producer_instance_id:=7101 -p state_lattice_v2_base_attestation_accept_enabled:=true \
  -p "state_lattice_v2_expected_pp_producer_instance_id:='7201'" -p "state_lattice_v2_expected_pp_session_id:='1'" \
  -p ego_state_topic:=${PRIVATE_INPUT}/kinematics -p opponent_topic:=${PRIVATE_INPUT}/v2x \
  -p instant_control_topic:=${PRIVATE_OUTPUT}/instant_control_cmd \
  -p reference_override_topic:=${PRIVATE_OUTPUT}/reference_override
planner_use_sim_time=""
pp_use_sim_time=""
for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
  if planner_use_sim_time="$(ros2 param get /state_lattice_overtake_planner_node use_sim_time 2>/dev/null)" && \
     [[ "${planner_use_sim_time}" == "Boolean value is: True" ]]; then
    break
  fi
  planner_use_sim_time=""
  sleep 0.1
done
for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
  if pp_use_sim_time="$(ros2 param get /simple_pure_pursuit_node use_sim_time 2>/dev/null)" && \
     [[ "${pp_use_sim_time}" == "Boolean value is: True" ]]; then
    break
  fi
  pp_use_sim_time=""
  sleep 0.1
done
test "${planner_use_sim_time}" = "Boolean value is: True"
test "${pp_use_sim_time}" = "Boolean value is: True"
python3 - <<'PY'
import json
from pathlib import Path

target = Path("/evidence/effective_time_parameters.json")
temporary = target.with_suffix(".tmp")
temporary.write_text(
    json.dumps(
        {
            "schema_version": 1,
            "planner_fqn": "/state_lattice_overtake_planner_node",
            "planner_use_sim_time": True,
            "pp_fqn": "/simple_pure_pursuit_node",
            "pp_use_sim_time": True,
        },
        sort_keys=True,
    )
    + "\\n",
    encoding="utf-8",
)
temporary.replace(target)
PY
for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
  if [[ -f /evidence/observer_sealed.json ]]; then break; fi
  sleep 0.1
done
test -f /evidence/observer_sealed.json
touch /evidence/capture_start.json

if wait "${component_pid[input_driver]}"; then component_rc[input_driver]=0; else component_rc[input_driver]=$?; fi
if kill -0 -- "-${component_pid[input_driver]}" 2>/dev/null; then component_residue[input_driver]=true; else component_residue[input_driver]=false; fi
if [[ "__FINAL_FENCE_PROFILE_BOOL__" == true ]]; then
  for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
    if [[ -s /evidence/final_fence_fixed.validated.json ]]; then break; fi
    sleep 0.1
  done
  test -s /evidence/final_fence_fixed.json
  test -s /evidence/final_fence_fixed.validated.json
  python3 - "${AIC_TEST_RUN_NONCE}" <<'PY'
import hashlib
import json
import re
import sys

def require(condition, reason):
    if not condition:
        raise SystemExit(reason)

def digest(payload):
    return hashlib.sha256(json.dumps(
        payload, ensure_ascii=False, allow_nan=False, sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")).hexdigest()

with open("/evidence/final_fence_fixed.json", encoding="utf-8") as stream:
    marker = json.load(stream)
with open("/evidence/final_fence_fixed.validated.json", encoding="utf-8") as stream:
    sidecar = json.load(stream)
marker_hash = marker.pop("terminal_canonical_sha256", None)
sidecar_hash = sidecar.pop("terminal_canonical_sha256", None)
require(marker_hash == digest(marker), "final_fence_marker_hash_invalid")
require(sidecar_hash == digest(sidecar), "final_fence_sidecar_hash_invalid")
require(set(marker) == {
    "schema_version", "execution_nonce", "sealed_epoch_id",
    "fence_canonical_sha256", "final_committed_ordinal",
    "frozen_pp_cycle_sequence", "fence_fixed",
}, "final_fence_marker_shape_invalid")
require(marker["schema_version"] == 1, "final_fence_marker_schema_invalid")
require(marker["execution_nonce"] == sys.argv[1], "final_fence_marker_nonce_invalid")
require(marker["sealed_epoch_id"] == 1, "final_fence_marker_epoch_invalid")
require(
    isinstance(marker["fence_canonical_sha256"], str)
    and re.fullmatch(r"[0-9a-f]{64}", marker["fence_canonical_sha256"])
    and marker["fence_canonical_sha256"] != "0" * 64,
    "final_fence_marker_fence_digest_invalid",
)
require(
    type(marker["final_committed_ordinal"]) is int
    and marker["final_committed_ordinal"] >= 0,
    "final_fence_marker_ordinal_invalid",
)
require(
    type(marker["frozen_pp_cycle_sequence"]) is int
    and marker["frozen_pp_cycle_sequence"] > 0,
    "final_fence_marker_t_invalid",
)
require(marker["fence_fixed"] is True, "final_fence_marker_not_fixed")
require(sidecar == {
    "schema_version": 1,
    "execution_nonce": sys.argv[1],
    "marker_terminal_canonical_sha256": marker_hash,
    "marker_validated": True,
}, "final_fence_sidecar_binding_invalid")
PY
  shutdown_component planner
  printf '{"schema_version":1,"planner_stopped":true}\n' > /evidence/planner_stopped.json
else
  for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
    if [[ -f /evidence/observer_terminal_graph.json ]]; then break; fi
    sleep 0.1
  done
  test -f /evidence/observer_terminal_graph.json
  shutdown_component planner
fi
for attempt in $(seq 1 "${STARTUP_TIMEOUT_TICKS}"); do
  if [[ -f /evidence/observer_terminal_graph.json ]]; then break; fi
  sleep 0.1
done
test -f /evidence/observer_terminal_graph.json
if wait "${component_pid[observer]}"; then component_rc[observer]=0; else component_rc[observer]=$?; fi
if kill -0 -- "-${component_pid[observer]}" 2>/dev/null; then component_residue[observer]=true; else component_residue[observer]=false; fi
test -s /evidence/observer.json
shutdown_component pure_pursuit
exit 0
""".replace(
        "__EXPECTED_CYCLONEDDS_URI__",
        cyclonedds_evidence["container_cyclonedds_uri"],
    ).replace(
        "__EXPECTED_CYCLONEDDS_SHA256__", cyclonedds_evidence["config_sha256"]
    ).replace(
        "__FINAL_FENCE_OBSERVER_ARGS__",
        (
            "--final-fence-enabled "
            "--final-fence-producer-instance-id 7101 "
            "--final-fence-session-id 1 "
            "--final-fence-sealed-epoch-id 1"
            if final_fence_profile
            else ""
        ),
    ).replace(
        "__FINAL_FENCE_PROFILE_BOOL__",
        "true" if final_fence_profile else "false",
    ).replace(
        "__FINAL_FENCE_PLANNER_ARGS__",
        (
            "-p state_lattice_v2_final_fence_enabled:=true "
            "-p state_lattice_v2_final_fence_execution_nonce:=${AIC_TEST_RUN_NONCE} "
            "-p \"state_lattice_v2_final_fence_expected_session_id:='1'\" "
            "-p state_lattice_v2_final_fence_sealed_epoch_id:=1"
            if final_fence_profile
            else ""
        ),
    )
    command = [
        "docker",
        "create",
        "--pull=never",
        "--rm",
        "--name",
        container_name,
        "--network",
        "host",
        "--ipc",
        "host",
        "--entrypoint",
        "bash",
        "-e",
        f"ROS_DOMAIN_ID={args.ros_domain_id}",
        "-e",
        f"AIC_TEST_RUN_NONCE={run_id}",
        "-e",
        f"CYCLONEDDS_URI={cyclonedds_evidence['container_cyclonedds_uri']}",
        "-v",
        f"{run_dir}:/evidence",
        "-v",
        f"{cyclonedds_config}:/opt/autoware/cyclonedds.xml:ro",
        prearm["expected_image_reference"],
        "-lc",
        container_script,
    ]
    try:
        created = _run(command, cwd=repo, timeout=30)
        payload["docker_create_returncode"] = created.returncode
        if created.returncode != 0:
            reasons.append("docker_create_failed")
            execution = None
        else:
            inspected = _run(
                ["docker", "container", "inspect", "--format", "{{.Image}}", container_name],
                cwd=repo, timeout=10,
            )
            actual_image_id = inspected.stdout.strip() if inspected.returncode == 0 else None
            if actual_image_id != prearm["expected_image_id"]:
                payload["created_image_id"] = actual_image_id
                reasons.append("created_image_id_mismatch")
                execution = None
            else:
                image_hashes, planner_binary_observation, pp_binary_observation, image_hash_error = _v2_uptake_image_hashes(
                    repo, container_name, run_dir, prearm
                )
                payload["planner_binary_attestation"] = planner_binary_observation
                payload["pp_binary_attestation"] = pp_binary_observation
                if image_hash_error is not None:
                    reasons.append(image_hash_error)
                    execution = None
                else:
                    binding = {
                    "run_id": run_id, "ros_domain_id": args.ros_domain_id,
                    "create_argv": command, "start_argv": ["docker", "start", "-a", container_name],
                    "expected_image_reference": prearm["expected_image_reference"],
                    "expected_image_id": prearm["expected_image_id"],
                    "actual_image_id": actual_image_id,
                    "host_hashes": prearm["actual_hashes"],
                    "image_hashes": image_hashes,
                    "planner_binary_attestation": planner_binary_observation,
                    "pp_binary_attestation": pp_binary_observation,
                    "expected_hashes": prearm["expected_hashes"],
                    "review_state": {
                        "path": prearm["review_state_path"],
                        "sha256": prearm["review_state_sha256"],
                        "reviewed_execution_id": prearm["reviewed_execution_id"],
                        "reviewed_handoff_sha256": prearm["reviewed_handoff_sha256"],
                        "current_edge_id": prearm["current_edge_id"],
                        "handoff_path": prearm["reviewed_handoff_path"],
                        "budget_transition": payload["review_state_budget_transition"],
                    },
                    "one_shot_budget_guard": str(budget_guard),
                    "doctor_residue": doctor_residue,
                    }
                    _write_host_readable_json_atomic(
                        run_dir / "prelaunch-binding.json", binding
                    )
                    execution = _run(
                        ["docker", "start", "-a", container_name],
                        cwd=repo, timeout=prearm["runtime_timeout_s"],
                    )
        if execution is not None:
            (run_dir / "launcher.log").write_text(execution.stdout, encoding="utf-8")
    except (OSError, subprocess.TimeoutExpired) as error:
        reasons.append(f"runtime_exception:{type(error).__name__}")
        execution = None
        try:
            payload["container_cleanup"] = _cleanup_direct_container(
                repo, container_name
            )
        except (OSError, subprocess.TimeoutExpired) as cleanup_error:
            payload["container_cleanup"] = {
                "container_name": container_name,
                "cleanup_exception": type(cleanup_error).__name__,
                "absent_after_cleanup": False,
            }
        if not payload["container_cleanup"]["absent_after_cleanup"]:
            reasons.append("direct_container_cleanup_incomplete")

    if execution is None and payload["container_cleanup"] is None:
        try:
            payload["container_cleanup"] = _cleanup_direct_container(
                repo, container_name
            )
        except (OSError, subprocess.TimeoutExpired) as cleanup_error:
            payload["container_cleanup"] = {
                "container_name": container_name,
                "cleanup_exception": type(cleanup_error).__name__,
                "absent_after_cleanup": False,
            }
        if not payload["container_cleanup"]["absent_after_cleanup"]:
            reasons.append("direct_container_cleanup_incomplete")

    if payload["container_cleanup"] is None:
        try:
            payload["container_cleanup"] = {
                "container_name": container_name,
                "cleanup_required": False,
                "absent_after_cleanup": not _container_exists(repo, container_name),
            }
        except (OSError, subprocess.TimeoutExpired) as inspect_error:
            payload["container_cleanup"] = {
                "container_name": container_name,
                "cleanup_required": False,
                "inspect_exception": type(inspect_error).__name__,
                "absent_after_cleanup": False,
            }

    observer_payload = None
    try:
        candidate = json.loads((run_dir / "observer.json").read_text())
        if isinstance(candidate, dict) and "outcome" in candidate:
            observer_payload = candidate
    except (OSError, json.JSONDecodeError):
        pass
    payload["observer"] = observer_payload
    statuses_path = run_dir / "component_exit_statuses.json"
    try:
        component_statuses = json.loads(statuses_path.read_text())
    except (OSError, json.JSONDecodeError):
        component_statuses = None
    payload["component_exit_statuses"] = component_statuses
    time_parameters_path = run_dir / "effective_time_parameters.json"
    try:
        effective_time_parameters = json.loads(time_parameters_path.read_text())
    except (OSError, json.JSONDecodeError):
        effective_time_parameters = None
    payload["effective_time_parameters"] = effective_time_parameters
    input_primed = _read_host_readable_json(run_dir / "input_primed.json")
    capture_complete = _read_host_readable_json(run_dir / "capture_complete.json")
    predicates = payload["predicates"]
    predicates.update(
        {
            "container_exit_zero": execution is not None and execution.returncode == 0,
            "observer_availability_verified": (
                isinstance(observer_payload, dict)
                and observer_payload.get("outcome") == "PASS"
                and observer_payload.get("label")
                == "PP_UPTAKE_AVAILABILITY_VERIFIED_NON_AUTHORITATIVE"
                and observer_payload.get("run_nonce") == run_id
            ),
            "observer_non_authoritative": (
                isinstance(observer_payload, dict)
                and observer_payload.get("authority", {}).get(
                    "no_authority_topic_intersection"
                )
                is True
            ),
            "observer_graph_sealed": (
                isinstance(observer_payload, dict)
                and observer_payload.get("graph", {}).get("sealed", {}).get("ready")
                is True
            ),
            "observer_terminal_graph_valid": (
                isinstance(observer_payload, dict)
                and observer_payload.get("graph", {}).get("terminal", {}).get("ready")
                is True
            ),
            "observer_exact_publisher_provenance": (
                isinstance(observer_payload, dict)
                and all(
                    not observer_payload.get("graph", {}).get(stage, {}).get(
                        "publisher_ownership_failures", ["missing"]
                    )
                    for stage in ("sealed", "terminal")
                )
            ),
            "observer_exact_subscriber_provenance": (
                isinstance(observer_payload, dict)
                and all(
                    not observer_payload.get("graph", {}).get(stage, {}).get(
                        "subscriber_ownership_failures", ["missing"]
                    )
                    for stage in ("sealed", "terminal")
                )
            ),
            "observer_bounded_stream_state": (
                _observer_bounded_deadline_chain_valid(observer_payload)
            ),
            "observer_epoch_order_valid": (
                isinstance(observer_payload, dict)
                and observer_payload.get("epoch", {}).get(
                    "proposal_before_pp_binding_cycle_count"
                )
                == 0
                and observer_payload.get("epoch", {}).get(
                    "binding_cycle_before_all_proposals"
                )
                is True
            ),
            "effective_time_parameters_verified": (
                isinstance(effective_time_parameters, dict)
                and effective_time_parameters.get("schema_version") == 1
                and effective_time_parameters.get("planner_fqn")
                == "/state_lattice_overtake_planner_node"
                and effective_time_parameters.get("planner_use_sim_time") is True
                and effective_time_parameters.get("pp_fqn")
                == "/simple_pure_pursuit_node"
                and effective_time_parameters.get("pp_use_sim_time") is True
            ),
            "direct_container_absent_after_run": payload["container_cleanup"].get(
                "absent_after_cleanup"
            )
            is True,
            "component_exit_statuses_present": isinstance(component_statuses, dict),
            "input_primed_host_readable": (
                isinstance(input_primed, dict)
                and input_primed.get("schema_version") == 1
                and input_primed.get("run_nonce") == run_id
            ),
            "capture_complete_host_readable": (
                isinstance(capture_complete, dict)
                and capture_complete.get("schema_version") == 1
                and capture_complete.get("run_nonce") == run_id
                and capture_complete.get("capture_complete") is True
            ),
            "input_driver_exit_zero": isinstance(component_statuses, dict)
            and component_statuses.get("input_driver", {}).get("exit_status") == 0,
            "observer_exit_zero": isinstance(component_statuses, dict)
            and component_statuses.get("observer", {}).get("exit_status") == 0,
            "production_processes_alive_until_shutdown": isinstance(component_statuses, dict)
            and all(component_statuses.get(name, {}).get("alive_before_shutdown") is True
                    for name in ("planner", "pure_pursuit")),
            "no_process_residue": isinstance(component_statuses, dict)
            and all(component_statuses.get(name, {}).get("residue") is False
                    for name in ("planner", "pure_pursuit", "input_driver", "observer")),
        }
    )
    reasons.extend(name for name, value in predicates.items() if not value)
    if not reasons:
        payload["outcome"] = "PASS"
        return_code = 0
    elif isinstance(observer_payload, dict):
        payload["outcome"] = "INVALID_EVIDENCE"
        return_code = 4
    else:
        return_code = 3
    payload["reasons"] = sorted(set(reasons))
    _write_host_readable_json_atomic(result_path, payload)
    if args.json:
        print(json.dumps(read_result(result_path), ensure_ascii=False, allow_nan=False))
    else:
        print(f"{payload['outcome']}: {result_path}")
    return return_code


def _artifact_record(path: Path) -> dict[str, Any]:
    """Describe an existing artifact without copying it into aic-test output."""
    record: dict[str, Any] = {"path": str(path), "exists": path.is_file()}
    if path.is_file():
        try:
            record["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        except OSError as error:
            record["hash_error"] = type(error).__name__
    return record


def _gate2_artifacts(output_dir: Path) -> dict[str, Any]:
    d1_dir = output_dir / "d1"
    rosbag_dir = d1_dir / "rosbag2_autoware"
    provenance_candidates = (
        output_dir / "provenance" / "prelaunch-manifest.json",
        output_dir / "provenance" / "postrun-manifest.json",
        output_dir / "provenance" / "verification.json",
        output_dir / "provenance" / "source-tree.sha256",
        output_dir / "provenance" / "artifact-fingerprint.sha256",
    )
    return {
        "runtime_output": {"path": str(output_dir), "exists": output_dir.is_dir()},
        "safety_gate_result": _artifact_record(output_dir / "safety-gate-result.json"),
        "d1_autoware_log": _artifact_record(d1_dir / "autoware.log"),
        "d1_safety_gate_result": _artifact_record(d1_dir / "safety-gate-result.json"),
        "d1_rosbag": {
            "path": str(rosbag_dir),
            "exists": rosbag_dir.is_dir(),
            "metadata": _artifact_record(rosbag_dir / "metadata.yaml"),
        },
        "provenance_manifests": [
            _artifact_record(candidate) for candidate in provenance_candidates
            if candidate.is_file()
        ],
    }


def _validate_gate2_result(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    """Validate the sole authoritative physical-outcome artifact for Gate 2."""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return None, f"safety_gate_result_unreadable:{type(error).__name__}"
    if not isinstance(payload, dict):
        return None, "safety_gate_result_not_object"
    if payload.get("schema_version") != "v1":
        return None, "safety_gate_result_schema_version_invalid"
    if payload.get("gate_arg") != "test2":
        return None, "safety_gate_result_not_test2"
    if payload.get("scenario_folder") != "SafetyGate":
        return None, "safety_gate_result_scenario_folder_invalid"
    if not isinstance(payload.get("all_passed"), bool):
        return None, "safety_gate_result_all_passed_invalid"
    tests = payload.get("tests")
    if not isinstance(tests, list) or len(tests) != 1:
        return None, "safety_gate_result_tests_invalid"
    test2 = tests[0]
    if (
        not isinstance(test2, dict)
        or test2.get("test_name") != "test2"
        or not isinstance(test2.get("passed"), bool)
        or test2["passed"] is not payload["all_passed"]
    ):
        return None, "safety_gate_result_test2_invalid"
    return payload, None


def _gate2_sanitized_environment() -> dict[str, str]:
    return {
        key: value for key, value in os.environ.items()
        if key not in GATE2_ENV_EXACT_KEYS
        and not key.startswith("AWSIM_")
        and not key.startswith("D1_")
    }


def _gate2_compose_preflight(
    repo: Path, environment: dict[str, str]
) -> tuple[str | None, str | None]:
    for project in (None, "1", "2", "3", "4"):
        command = ["docker", "compose"]
        if project is not None:
            command.extend(["-p", project])
        command.extend(["ps", "-q"])
        try:
            result = _run(command, cwd=repo, env=environment, timeout=10)
        except (OSError, subprocess.TimeoutExpired) as error:
            return None, f"compose_preflight_exception:{type(error).__name__}"
        if result.returncode != 0:
            return None, "compose_preflight_command_failed"
        if result.stdout.strip():
            label = "default" if project is None else project
            return f"managed_compose_project_active:{label}", None
    return None, None


def _gate2_manifest_error(
    path: Path, repo: Path, run_id: str, started_wall_ns: int
) -> str | None:
    try:
        if path.stat().st_mtime_ns + GATE2_FRESHNESS_TOLERANCE_NS < started_wall_ns:
            return "prelaunch_manifest_stale"
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return f"prelaunch_manifest_unreadable:{type(error).__name__}"
    if not isinstance(manifest, dict):
        return "prelaunch_manifest_schema_invalid"
    if manifest.get("schema_version") != 2:
        return "prelaunch_manifest_schema_version_invalid"
    run = manifest.get("run")
    capture = manifest.get("capture_stability")
    launch_context = manifest.get("launch_context")
    if not isinstance(launch_context, dict):
        return "prelaunch_manifest_launch_context_invalid"
    plain = launch_context.get("plain")
    hashed = launch_context.get("hashed")
    if not isinstance(run, dict) or not isinstance(capture, dict) or not isinstance(plain, dict) or not isinstance(hashed, dict):
        return "prelaunch_manifest_schema_invalid"
    if run.get("run_id") != run_id:
        return "prelaunch_manifest_run_id_mismatch"
    if run.get("container_output_root") != "/output":
        return "prelaunch_manifest_container_output_root_mismatch"
    if run.get("host_output_root") != str((repo / "output").resolve()):
        return "prelaunch_manifest_host_output_root_mismatch"
    if capture.get("match") is not True:
        return "prelaunch_manifest_capture_stability_invalid"
    expected_plain = {
        "RUN_GATE_SCENARIO": "SafetyGate/scenario2.yaml",
        "CONTROL_METHOD": "state_lattice_pure_pursuit",
        "RUN_KIND": "planner-pp-control-smoke",
        "PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL": "true",
        "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED": "true",
        "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED": "true",
        "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID": "4101",
        "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID": "4201",
        "STATE_LATTICE_V2_SESSION_ID": "1",
        "ROSBAG": "true",
        "AWSIM_VEHICLES": "4",
        "AUTOWARE_RUN_MODE": "awsim-no-viz",
        "AUTOSTART_DEBUG_VISUALIZATION": "false",
    }
    if any(plain.get(key) != value for key, value in expected_plain.items()):
        return "prelaunch_manifest_launch_context_mismatch"
    gate_args = hashed.get("GATE_EXTRA_ARGS")
    expected_gate_args_hash = hashlib.sha256(
        b"-batchmode -nographics --camera false --lidar false"
    ).hexdigest()
    if not isinstance(gate_args, dict) or gate_args.get("present") is not True or gate_args.get("sha256") != expected_gate_args_hash:
        return "prelaunch_manifest_gate_extra_args_mismatch"
    return None


def _gate2_postrun_error(path: Path, run_id: str, started_wall_ns: int) -> str | None:
    try:
        if path.stat().st_mtime_ns + GATE2_FRESHNESS_TOLERANCE_NS < started_wall_ns:
            return f"{path.stem}_stale"
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return f"{path.stem}_unreadable:{type(error).__name__}"
    if not isinstance(payload, dict) or payload.get("schema_version") != 2:
        return f"{path.stem}_schema_invalid"
    if path.name == "postrun-manifest.json":
        run = payload.get("run")
        if not isinstance(run, dict):
            return "postrun-manifest_run_invalid"
        if run.get("run_id") != run_id:
            return "postrun-manifest_run_id_mismatch"
    elif (
        payload.get("verified") is not True
        or not isinstance(payload.get("comparisons"), dict)
        or not payload["comparisons"]
        or any(
            not isinstance(comparison, dict) or comparison.get("match") is not True
            for comparison in payload["comparisons"].values()
        )
        or not isinstance(payload.get("rosbag_metadata"), list)
        or not payload["rosbag_metadata"]
    ):
        return "verification_invalid"
    return None


def run_safegate2_stopped_overtake(args: argparse.Namespace) -> int:
    """Run the fixed State Lattice-to-PP Gate 2 target without overrides."""
    repo = _repo_root(args.repo_root)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    run_id = (
        args.run_id
        if args.run_id is not None
        else f"{timestamp}-safegate2-stopped-overtake-{secrets.token_hex(4)}"
    )
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", run_id):
        print("PRECONDITION_NOT_MET: invalid run id")
        return 3
    run_dir = repo / "analysis" / "aic_test" / "runs" / run_id
    output_dir = repo / "output" / run_id
    environment = _gate2_sanitized_environment()
    if output_dir.exists():
        print(f"PRECONDITION_NOT_MET: output directory already exists: {output_dir}")
        return 3
    active_project, preflight_error = _gate2_compose_preflight(repo, environment)
    if active_project is not None:
        print(f"PRECONDITION_NOT_MET: {active_project}")
        return 3
    if preflight_error is not None:
        print(f"INFRASTRUCTURE_FAILED: {preflight_error}")
        return 3
    try:
        run_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f"PRECONDITION_NOT_MET: run directory already exists: {run_dir}")
        return 3
    except OSError as error:
        print(f"INFRASTRUCTURE_FAILED: cannot create run directory: {type(error).__name__}")
        return 3

    result_path = run_dir / "result.json"
    payload: dict[str, Any] = {
        "run_id": run_id,
        "scenario": "safegate2-stopped-overtake",
        "outcome": "INFRASTRUCTURE_FAILED",
        "reasons": [],
        "command": [
            "make",
            "planner-pp-control-smoke",
            f"RUN_ID={run_id}",
            "GATE_EXTRA_ARGS=-batchmode -nographics --camera false --lidar false",
            "AWSIM_EXTRA_ARGS=",
            "AUTOWARE_RUN_MODE=awsim-no-viz",
            "AUTOSTART_DEBUG_VISUALIZATION=false",
            "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED=true",
            "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED=true",
            "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID=4101",
            "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID=4201",
            "STATE_LATTICE_V2_SESSION_ID=1",
            "OUTPUT_HOST_ROOT=./output",
            "OUTPUT_ROOT=/output",
        ],
        "maximum_claim": "OFFICIAL_GATE2_PHYSICAL_OUTCOME_ONLY",
        "artifacts": {"result": str(result_path)},
    }
    reasons: list[str] = payload["reasons"]
    started_wall_ns = time.time_ns()
    payload["started_wall_ns"] = started_wall_ns
    started = False
    try:
        started = True
        completed = _run(
            payload["command"], cwd=repo, env=environment,
            timeout=GATE2_RUNTIME_TIMEOUT_S
        )
        payload["make_returncode"] = completed.returncode
    except subprocess.TimeoutExpired:
        reasons.append("fixed_gate2_target_timed_out")
    except OSError as error:
        reasons.append(f"fixed_gate2_target_exception:{type(error).__name__}")
    finally:
        if started:
            try:
                down = _run(["make", "down"], cwd=repo, env=environment, timeout=210)
                payload["cleanup"] = {"command": ["make", "down"], "returncode": down.returncode}
                if down.returncode != 0:
                    reasons.append("make_down_failed")
            except (OSError, subprocess.TimeoutExpired) as error:
                payload["cleanup"] = {"command": ["make", "down"], "error": type(error).__name__}
                reasons.append("make_down_failed")

    cleanup_active, cleanup_error = _gate2_compose_preflight(repo, environment)
    if cleanup_active is not None:
        reasons.append(f"cleanup_residue:{cleanup_active}")
    if cleanup_error is not None:
        reasons.append(f"cleanup_scan_failed:{cleanup_error}")
    verification_error = None
    if not reasons:
        verify_command = [
            "make", "verify-run-fingerprint", f"VERIFY_RUN_ID={run_id}",
            "OUTPUT_HOST_ROOT=./output", "OUTPUT_ROOT=/output",
        ]
        try:
            verify = _run(
                verify_command, cwd=repo, env=environment, timeout=30
            )
            payload["verification"] = {
                "command": verify_command, "returncode": verify.returncode,
            }
            if verify.returncode != 0:
                verification_error = "verify_run_fingerprint_failed"
        except (OSError, subprocess.TimeoutExpired) as error:
            verification_error = f"verify_run_fingerprint_exception:{type(error).__name__}"
        if verification_error is None:
            for path in (
                output_dir / "provenance" / "postrun-manifest.json",
                output_dir / "provenance" / "verification.json",
            ):
                validation_error = _gate2_postrun_error(path, run_id, started_wall_ns)
                if validation_error is not None:
                    verification_error = validation_error
                    break

    payload["artifacts"].update(_gate2_artifacts(output_dir))
    official, evidence_error = _validate_gate2_result(
        output_dir / "safety-gate-result.json"
    )
    official_result_exists = (output_dir / "safety-gate-result.json").is_file()
    completed_normally = not reasons and payload.get("make_returncode") == 0
    required_paths = (
        output_dir / "d1" / "autoware.log",
        output_dir / "d1" / "rosbag2_autoware" / "metadata.yaml",
        output_dir / "provenance" / "prelaunch-manifest.json",
        output_dir / "provenance" / "source-tree.sha256",
        output_dir / "provenance" / "artifact-fingerprint.sha256",
        output_dir / "provenance" / "postrun-manifest.json",
        output_dir / "provenance" / "verification.json",
    )
    manifest_error = _gate2_manifest_error(
        output_dir / "provenance" / "prelaunch-manifest.json", repo, run_id,
        started_wall_ns,
    )
    evidence_errors: list[str] = []
    if official_result_exists:
        try:
            if (output_dir / "safety-gate-result.json").stat().st_mtime_ns + GATE2_FRESHNESS_TOLERANCE_NS < started_wall_ns:
                evidence_errors.append("safety_gate_result_stale")
        except OSError:
            evidence_errors.append("safety_gate_result_stat_failed")
    if manifest_error is not None:
        evidence_errors.append(manifest_error)
    if verification_error is not None:
        evidence_errors.append(verification_error)
    if official is not None:
        evidence_errors.extend(
            f"required_artifact_missing:{path.relative_to(output_dir)}"
            for path in required_paths if not path.is_file()
        )
    if official is not None:
        payload["official_gate2"] = {
            "all_passed": official["all_passed"],
            "test2_fail_reason": official["tests"][0].get("fail_reason"),
        }
    if not completed_normally and (
        not official_result_exists or evidence_error is not None or evidence_errors
    ):
        if payload.get("make_returncode") != 0:
            reasons.append("fixed_gate2_target_failed")
        payload["outcome"] = "INFRASTRUCTURE_FAILED"
        return_code = 3
    elif evidence_error is not None:
        reasons.append(evidence_error)
        payload["outcome"] = "INVALID_EVIDENCE"
        return_code = 4
    elif evidence_errors:
        reasons.extend(evidence_errors)
        payload["outcome"] = "INVALID_EVIDENCE"
        return_code = 4
    elif reasons or payload.get("make_returncode") != 0:
        if payload.get("make_returncode") != 0:
            reasons.append("fixed_gate2_target_failed")
        payload["outcome"] = "INFRASTRUCTURE_FAILED"
        return_code = 3
    elif official is not None and official["all_passed"]:
        payload["outcome"] = "PASS"
        return_code = 0
    else:
        payload["outcome"] = "ASSERTION_FAILED"
        return_code = 1
    payload["reasons"] = sorted(set(reasons))
    write_json_atomic(result_path, payload)
    if args.json:
        print(json.dumps(read_result(result_path), ensure_ascii=False, allow_nan=False))
    else:
        print(f"{payload['outcome']}: {result_path}")
    return return_code


def run_scenario(args: argparse.Namespace) -> int:
    if args.scenario == "control-smoke":
        return run_control_smoke(args)
    if args.scenario == "safegate2-stopped-overtake":
        return run_safegate2_stopped_overtake(args)
    return run_v2_uptake_smoke(args)


def show_result(args: argparse.Namespace) -> int:
    repo = _repo_root(args.repo_root)
    path = repo / "analysis" / "aic_test" / "runs" / args.run_id / "result.json"
    payload = read_result(path)
    if args.format == "json":
        print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    else:
        print(f"{payload['outcome']}: {path}")
    return 0


def _gate2_metrics_timeline(raw_records: list[object]) -> tuple[dict[str, Any] | None, str | None]:
    """Strictly reduce recorded JSON-string metrics into an ordered timeline."""
    if not isinstance(raw_records, list) or not raw_records:
        return None, "metrics_records_empty_or_invalid"
    topic_counts: dict[str, int] = {}
    transitions: list[dict[str, Any]] = []
    repaired_legacy_count = 0
    pp_debug_count = 0
    base_attestation_status_count = 0
    base_attestation_transitions: list[dict[str, Any]] = []
    previous_base_attestation: tuple[object, ...] | None = None
    v4_applied_count = 0
    first_v4_application: dict[str, Any] | None = None
    first_v4_nonapplication: dict[str, Any] | None = None
    previous_timestamp_ns: int | None = None
    previous: tuple[object, object] | None = None
    for record in raw_records:
        if not isinstance(record, dict):
            return None, "metrics_record_invalid"
        topic = record.get("topic")
        timestamp_ns = record.get("timestamp_ns")
        data = record.get("data")
        if not isinstance(topic, str) or not isinstance(timestamp_ns, int) or isinstance(timestamp_ns, bool) or not isinstance(data, str):
            return None, "metrics_record_schema_invalid"
        if previous_timestamp_ns is not None and timestamp_ns < previous_timestamp_ns:
            return None, "metrics_timestamp_out_of_order"
        previous_timestamp_ns = timestamp_ns
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        if topic == "/debug/overtake/state_lattice/v2_binding_status":
            try:
                status = json.loads(data)
            except json.JSONDecodeError:
                return None, "v2_binding_status_json_invalid"
            if not isinstance(status, dict):
                return None, "v2_binding_status_json_not_object"
            if status.get("availability_summary") is not True:
                continue
            required = (
                "base_attestation_stage", "base_attestation_build_result",
                "base_attestation_build_diagnostic",
                "base_attestation_validation_reason",
                "base_attestation_point_invalid_field",
                "base_attestation_failure_window_index",
                "base_attestation_failure_source_index",
                "base_attestation_snapshot_result",
                "base_attestation_local_reject_reason",
                "base_attestation_attempt_count",
                "base_attestation_publish_count",
                "base_attestation_race_arm_epoch",
            )
            if any(
                not isinstance(status.get(key), int)
                or isinstance(status.get(key), bool)
                or status[key] < 0
                for key in required
            ):
                return None, "v2_binding_status_base_attestation_schema_invalid"
            base_attestation_status_count += 1
            state = tuple(status[key] for key in required)
            if state != previous_base_attestation:
                previous_base_attestation = state
                base_attestation_transitions.append(
                    {"timestamp_ns": timestamp_ns, **{key: status[key] for key in required}}
                )
            continue
        if topic == "/pure_pursuit/debug":
            try:
                debug = json.loads(data)
            except json.JSONDecodeError:
                return None, "pp_debug_json_invalid"
            if not isinstance(debug, dict):
                return None, "pp_debug_json_not_object"
            # The bag topic can also carry the disabled wall-recovery planner's
            # bounded debug record. It is not a Pure Pursuit application event.
            if debug.get("controller") != "simple_pure_pursuit":
                continue
            if debug.get("stale_input") is True:
                continue
            applied = debug.get("overtake_override_applied")
            generation = debug.get("overtake_override_generation")
            reason = debug.get("overtake_override_apply_reason")
            source = debug.get("trajectory_source")
            if (
                not isinstance(applied, bool)
                or not isinstance(generation, int)
                or isinstance(generation, bool)
                or generation < 0
                or not isinstance(reason, str)
                or not isinstance(source, str)
            ):
                return None, "pp_debug_v4_application_schema_invalid"
            pp_debug_count += 1
            observation = {
                "timestamp_ns": timestamp_ns,
                "applied": applied,
                "generation": generation,
                "trajectory_source": source,
                "apply_reason": reason,
                "steering_tire_angle_rad": debug.get("steering_tire_angle_rad"),
                "lateral_offset_m": debug.get("overtake_lateral_offset_m"),
                "spatial_horizon_arc_m": debug.get("overtake_spatial_horizon_arc_m"),
            }
            v4_values = {
                key: debug.get(key)
                for key in (
                    "v4_poc_contract", "v4_poc_identity_required",
                    "v4_poc_identity_matched", "v4_poc_geometry_applied",
                    "v4_poc_generation",
                )
            }
            v4_present = [value is not None for value in v4_values.values()]
            if any(v4_present) and not all(v4_present):
                return None, "pp_debug_v4_poc_fields_partial"
            if all(v4_present):
                if (
                    any(
                        not isinstance(v4_values[key], bool)
                        for key in (
                            "v4_poc_contract", "v4_poc_identity_required",
                            "v4_poc_identity_matched",
                            "v4_poc_geometry_applied",
                        )
                    )
                    or not isinstance(v4_values["v4_poc_generation"], int)
                    or isinstance(v4_values["v4_poc_generation"], bool)
                ):
                    return None, "pp_debug_v4_poc_fields_invalid"
                v4_applied = (
                    applied
                    and source == "trajectory_overtake_override"
                    and v4_values["v4_poc_contract"] is True
                    and v4_values["v4_poc_identity_required"] is True
                    and v4_values["v4_poc_identity_matched"] is True
                    and v4_values["v4_poc_geometry_applied"] is True
                    and v4_values["v4_poc_generation"] == generation
                    and generation > 0
                )
                observation.update(v4_values)
            else:
                v4_applied = False
            if v4_applied:
                v4_applied_count += 1
                if first_v4_application is None:
                    first_v4_application = observation
            elif first_v4_nonapplication is None:
                first_v4_nonapplication = observation
            continue
        if topic != "/debug/overtake/metrics":
            continue
        repaired, repair_count = re.subn(
            r'("preventive_side_role_generation"\s*:\s*-?\d+)"(?=\s*[,}])',
            r"\1",
            data,
        )
        try:
            metric = json.loads(repaired)
        except json.JSONDecodeError:
            return None, "metrics_json_invalid"
        repaired_legacy_count += repair_count
        if not isinstance(metric, dict):
            return None, "metrics_json_not_object"
        mode = metric.get("mode", metric.get("overtake_state"))
        reason = metric.get("reason", metric.get("blocked_reason"))
        state = (mode, reason)
        if state == previous:
            continue
        previous = state
        key_fields = {
            key: value for key, value in metric.items()
            if (
                "candidate" in key
                or "cost" in key
                or key.startswith("state_lattice_v2_")
                or key in {"selected", "reason", "blocked_reason"}
            )
            and isinstance(value, (str, int, float, bool, type(None)))
        }
        transitions.append({"transition_index": len(transitions), "timestamp_ns": timestamp_ns, "source_topic": "/debug/overtake/metrics", "mode": mode, "reason": reason, "fields": key_fields})
    if not topic_counts.get("/debug/overtake/metrics") or not topic_counts.get("/debug/overtake/mode") or not transitions:
        return None, "metrics_required_topics_or_transitions_missing"
    first_transition = transitions[0]
    overtake_modes = {"OVERTAKE_LEFT", "OVERTAKE_RIGHT", "SIDE_BY_SIDE_KEEP"}
    failure_modes = {"SAFE_STOP", "FOLLOW_BLOCKED"}
    saw_overtake = False
    first_failure = None
    overtake_anchor = None
    for transition in transitions:
        if transition["mode"] in overtake_modes:
            saw_overtake = True
            if overtake_anchor is None:
                overtake_anchor = transition
        elif saw_overtake and transition["mode"] in failure_modes:
            first_failure = {
                **transition,
                "anchor_transition_index": overtake_anchor["transition_index"],
                "anchor_timestamp_ns": overtake_anchor["timestamp_ns"],
            }
            break
    return {
        "topic_counts": topic_counts,
        "mode_reason_transitions": transitions,
        "first_transition": first_transition,
        "overtake_anchor_transition": overtake_anchor,
        "first_stop_or_blocked_transition_after_overtake_mode": first_failure,
        "v4_pp_application": {
            "observation": (
                "PP_DEBUG_REPORTED_V4_POC_GEOMETRY_APPLIED"
                if v4_applied_count > 0
                else "V4_POC_GEOMETRY_APPLICATION_NOT_OBSERVED"
            ),
            "pp_debug_record_count": pp_debug_count,
            "applied_record_count": v4_applied_count,
            "first_applied_record": first_v4_application,
            "first_nonapplied_record": first_v4_nonapplication,
            "byte_exact_payload_identity_proven": False,
            "m4_credit": False,
        },
        "v2_base_attestation": {
            "summary_record_count": base_attestation_status_count,
            "transitions": base_attestation_transitions,
            "last": base_attestation_transitions[-1]
            if base_attestation_transitions else None,
        },
        "warnings": ([{"code": "legacy_preventive_side_role_generation_quote_repaired", "count": repaired_legacy_count}] if repaired_legacy_count else []),
    }, None


def _gate2_analysis_bag_path(repo: Path, run_id: str) -> tuple[Path | None, str | None]:
    output_root = repo / "output"
    try:
        if stat.S_ISLNK(output_root.lstat().st_mode):
            return None, "rosbag_output_ancestor_symlink_rejected"
    except OSError:
        return None, "rosbag_metadata_missing"
    run_root = output_root / run_id
    bag = run_root / "d1" / "rosbag2_autoware"
    metadata = bag / "metadata.yaml"
    for path in (run_root, run_root / "d1", bag, metadata):
        try:
            if stat.S_ISLNK(path.lstat().st_mode):
                return None, "rosbag_symlink_component_rejected"
        except OSError:
            return None, "rosbag_metadata_missing"
    try:
        resolved_output_root = output_root.resolve(strict=True)
        resolved_run_root = run_root.resolve(strict=True)
        if not resolved_run_root.is_relative_to(resolved_output_root):
            return None, "rosbag_run_root_escape_rejected"
        expected_root = (run_root / "d1").resolve(strict=True)
        resolved_bag = bag.resolve(strict=True)
        if not resolved_bag.is_relative_to(expected_root) or not metadata.is_file():
            return None, "rosbag_path_escape_or_metadata_missing"
    except OSError:
        return None, "rosbag_path_escape_or_metadata_missing"
    return resolved_bag, None


def _gate2_analysis_container_name(run_id: str, nonce: str) -> str:
    safe_run_id = re.sub(r"[^a-z0-9_.-]", "-", run_id.lower())
    return f"aic-test-analysis-{safe_run_id[:28]}-{nonce}"


def _gate2_analysis_container_ownership(
    repo: Path, name: str, nonce: str
) -> bool | None:
    inspected = _run(
        ["docker", "container", "inspect", "--format", "{{ index .Config.Labels \"aic-test.analysis_nonce\" }}", name],
        cwd=repo,
        timeout=10,
    )
    if inspected.returncode != 0:
        listed = _run(
            ["docker", "container", "ls", "-a", "--filter", f"name=^/{name}$", "--format", "{{.Names}}"],
            cwd=repo,
            timeout=10,
        )
        if listed.returncode != 0:
            raise OSError("analysis_container_absence_check_failed")
        names = listed.stdout.splitlines()
        if not names:
            return None
        if names != [name]:
            raise OSError("analysis_container_absence_check_ambiguous")
        raise OSError("analysis_container_inspect_failed_while_present")
    return inspected.stdout.strip() == nonce


def _gate2_cleanup_analysis_container(repo: Path, name: str, nonce: str) -> bool:
    ownership = _gate2_analysis_container_ownership(repo, name, nonce)
    if ownership is None:
        return True
    if not ownership:
        return False
    removed = _run(["docker", "rm", "-f", name], cwd=repo, timeout=15)
    return removed.returncode == 0 and _gate2_analysis_container_ownership(repo, name, nonce) is None


def analyze_safegate2_stopped_overtake(args: argparse.Namespace) -> int:
    repo = _repo_root(args.repo_root)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", args.run_id):
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": "invalid_run_id"}))
        return 4
    domain_dir, path_error = _gate2_analysis_bag_path(repo, args.run_id)
    if path_error is not None or domain_dir is None:
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": path_error}))
        return 4
    metadata_path = domain_dir / "metadata.yaml"
    try:
        if metadata_path.stat().st_size > GATE2_METADATA_MAX_BYTES:
            print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": "rosbag_metadata_too_large"}))
            return 4
        metadata = metadata_path.read_text(encoding="utf-8", errors="strict")
    except UnicodeDecodeError:
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": "rosbag_metadata_unreadable"}))
        return 4
    storage_match = re.search(r"^\s*storage_identifier:\s*([A-Za-z0-9_]+)\s*$", metadata, re.MULTILINE)
    storage_id = storage_match.group(1) if storage_match else None
    if storage_id != "mcap":
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": "rosbag_storage_unsupported", "storage_identifier": storage_id}))
        return 4
    script = """import json
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
r=SequentialReader();r.open(StorageOptions(uri='/bag',storage_id='STORAGE_ID'),ConverterOptions('', ''))
t={x.name:x.type for x in r.get_all_topics_and_types()}; out=[]
total=0; limit_records=10000; limit_bytes=67108864
while r.has_next():
 n,b,s=r.read_next()
 if n not in ('/debug/overtake/metrics','/debug/overtake/mode','/pure_pursuit/debug','/debug/overtake/state_lattice/v2_binding_status'): continue
 m=deserialize_message(b,get_message(t[n]))
 if n == '/debug/overtake/state_lattice/v2_binding_status':
  data=json.dumps({'availability_summary':m.availability_summary,'base_attestation_stage':m.base_attestation_stage,'base_attestation_build_result':m.base_attestation_build_result,'base_attestation_build_diagnostic':m.base_attestation_build_diagnostic,'base_attestation_validation_reason':m.base_attestation_validation_reason,'base_attestation_point_invalid_field':m.base_attestation_point_invalid_field,'base_attestation_failure_window_index':m.base_attestation_failure_window_index,'base_attestation_failure_source_index':m.base_attestation_failure_source_index,'base_attestation_snapshot_result':m.base_attestation_snapshot_result,'base_attestation_local_reject_reason':m.base_attestation_local_reject_reason,'base_attestation_attempt_count':m.base_attestation_attempt_count,'base_attestation_publish_count':m.base_attestation_publish_count,'base_attestation_race_arm_epoch':m.base_attestation_race_arm_epoch},separators=(',',':'))
 else: data=m.data
 total+=len(data.encode('utf-8'))
 if len(out)>=limit_records or total>limit_bytes: raise RuntimeError('analysis_record_or_byte_limit_exceeded')
 out.append({'topic':n,'timestamp_ns':s,'data':data})
print('AIC_TEST_ANALYSIS_JSON='+json.dumps(out,allow_nan=False))""".replace("STORAGE_ID", storage_id)
    container_nonce = secrets.token_hex(8)
    container_name = _gate2_analysis_container_name(args.run_id, container_nonce)
    command = ["docker", "run", "--rm", "--name", container_name, "--label", f"aic-test.analysis_nonce={container_nonce}", "--pull=never", "--network", "none", "--read-only", "--cap-drop=ALL", "--security-opt=no-new-privileges", "--memory=512m", "--cpus=1", "--pids-limit=128", "--user", "65534:65534", "--mount", f"type=bind,src={domain_dir},dst=/bag,readonly", "aichallenge-2025-eval:latest", "bash", "-lc", "source /autoware/install/setup.bash; exec python3 -c \"$1\"", "aic-test-analyzer", script]
    try:
        result = _run(command, cwd=repo, timeout=GATE2_ANALYSIS_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired) as error:
        try:
            cleaned = _gate2_cleanup_analysis_container(repo, container_name, container_nonce)
        except (OSError, subprocess.TimeoutExpired):
            cleaned = False
        if not cleaned:
            print(json.dumps({"outcome": "INFRASTRUCTURE_FAILED", "reason": "rosbag_analysis_timeout_cleanup_failed"}))
            return 3
        print(json.dumps({"outcome": "INFRASTRUCTURE_FAILED", "reason": f"rosbag_analysis_exception:{type(error).__name__}"}))
        return 3
    try:
        residual = _gate2_analysis_container_ownership(repo, container_name, container_nonce)
        if residual is False:
            print(json.dumps({"outcome": "INFRASTRUCTURE_FAILED", "reason": "rosbag_analysis_container_owner_mismatch"}))
            return 3
        if residual is True:
            if not _gate2_cleanup_analysis_container(repo, container_name, container_nonce):
                print(json.dumps({"outcome": "INFRASTRUCTURE_FAILED", "reason": "rosbag_analysis_residual_cleanup_failed"}))
                return 3
            print(json.dumps({"outcome": "INFRASTRUCTURE_FAILED", "reason": "rosbag_analysis_container_residual"}))
            return 3
    except (OSError, subprocess.TimeoutExpired):
        print(json.dumps({"outcome": "INFRASTRUCTURE_FAILED", "reason": "rosbag_analysis_residual_check_failed"}))
        return 3
    if result.returncode != 0:
        print(json.dumps({"outcome": "INFRASTRUCTURE_FAILED", "reason": "rosbag_analysis_failed"}))
        return 3
    if len(result.stdout.encode("utf-8")) > 64 * 1024 * 1024:
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": "rosbag_analysis_output_limit_exceeded"}))
        return 4
    marker_lines = [line[len("AIC_TEST_ANALYSIS_JSON="):] for line in result.stdout.splitlines() if line.startswith("AIC_TEST_ANALYSIS_JSON=")]
    if len(marker_lines) != 1:
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": "rosbag_analysis_marker_invalid"}))
        return 4
    try:
        records = json.loads(marker_lines[0])
    except json.JSONDecodeError:
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": "rosbag_analysis_output_invalid"}))
        return 4
    timeline, error = _gate2_metrics_timeline(records if isinstance(records, list) else [])
    if error is not None:
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": error}))
        return 4
    classification = "POST_OVERTAKE_STOP_OR_BLOCKED_OBSERVED" if timeline["first_stop_or_blocked_transition_after_overtake_mode"] else ("NO_POST_OVERTAKE_STOP_OR_BLOCKED_OBSERVED" if timeline["overtake_anchor_transition"] else "NO_OVERTAKE_OBSERVED")
    planner_pp_outcome = (
        "PP_DEBUG_REPORTED_V4_POC_GEOMETRY_APPLIED_OBSERVED"
        if timeline["v4_pp_application"]["applied_record_count"] > 0
        else "V4_POC_GEOMETRY_APPLICATION_NOT_OBSERVED"
    )
    print(json.dumps({"analysis_status": "COMPLETE", "claim_cap": "READ_ONLY_PLANNER_PP_APPLICATION_DIAGNOSTIC_ONLY", "physical_outcome": "NOT_EVALUATED", "planner_pp_authority_outcome": planner_pp_outcome, "m4_credit": False, "m5_credit": False, "causal_owner": "NOT_DETERMINED", "behavior_classification": classification, "run_id": args.run_id, "rosbag": str(domain_dir), "mode_topic_usage": "COUNT_ONLY_NOT_JOINED", **timeline}, allow_nan=False))
    return 0


def doctor(args: argparse.Namespace) -> int:
    repo = _repo_root(args.repo_root)
    checks = {
        "repo": repo.is_dir(),
        "compose": (repo / "docker-compose.yml").is_file(),
        "submit_archive": (repo / "submit/aichallenge_submit.tar.gz").is_file(),
        "docker": shutil.which("docker") is not None,
        "eval_image": bool(_git_fingerprint(repo).get("eval_image")),
        "disk_free_ge_5_gib": shutil.disk_usage(repo).free >= 5 * 1024**3,
    }
    print(json.dumps({"checks": checks}, ensure_ascii=False, indent=2))
    return 0 if all(checks.values()) else 3


def external_review(args: argparse.Namespace) -> int:
    from .review_queue import (
        CANONICAL_REVIEW_QUEUE_ROOT,
        ReviewQueue,
        ReviewQueueError,
    )

    queue = ReviewQueue(CANONICAL_REVIEW_QUEUE_ROOT)
    try:
        if args.review_command == "enqueue":
            result = queue.enqueue(
                execution_id=args.execution_id, review_attempt_id=args.review_attempt_id,
                handoff=Path(args.handoff),
                packet=Path(args.packet), objective=args.objective,
                requested_by=args.requested_by, retry_of=args.retry_of,
            )
        elif args.review_command == "claim":
            result = queue.claim(execution_id=args.execution_id,
                                 review_attempt_id=args.review_attempt_id,
                                 worker_id=args.worker_id)
        elif args.review_command == "defer":
            result = queue.defer(
                execution_id=args.execution_id, review_attempt_id=args.review_attempt_id,
                worker_id=args.worker_id, evidence=Path(args.connection_evidence),
                evidence_sha256=args.connection_evidence_sha256,
                terminal_connection_error=args.terminal_connection_error,
            )
        elif args.review_command == "complete":
            result = queue.complete(
                execution_id=args.execution_id, review_attempt_id=args.review_attempt_id,
                worker_id=args.worker_id,
                model=args.model, severity=args.severity, disposition=args.disposition,
                transition_permission=args.transition_permission,
                response=Path(args.response), minimum_changes=args.minimum_change,
                submission_id=args.submission_id, project_url=args.project_url,
                fallback_reason=args.fallback_reason,
            )
        else:
            result = queue.status(args.execution_id, args.review_attempt_id)
    except (OSError, ReviewQueueError) as exc:
        print(json.dumps({"outcome": "INVALID_EVIDENCE", "reason": str(exc)}, ensure_ascii=False))
        return 4
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result["integrity_valid"] else 4


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="aic-test")
    parser.add_argument("--repo-root")
    subparsers = parser.add_subparsers(dest="command", required=True)

    doctor_parser = subparsers.add_parser("doctor")
    doctor_parser.set_defaults(func=doctor)

    run_parser = subparsers.add_parser("run")
    run_parser.add_argument(
        "scenario",
        choices=(
            "control-smoke",
            "safegate2-stopped-overtake",
            "v2-uptake-smoke",
            "v2-uptake-final-fence-smoke",
        ),
    )
    run_parser.add_argument("--run-id")
    run_parser.add_argument("--startup-timeout", type=float, default=60.0)
    run_parser.add_argument("--observe-duration", type=float, default=20.0)
    run_parser.add_argument("--awsim-timeout", type=int, default=90)
    run_parser.add_argument("--runtime-timeout", type=float, default=90.0)
    run_parser.add_argument("--ros-domain-id", type=int, default=231)
    run_parser.add_argument("--json", action="store_true")
    run_parser.set_defaults(func=run_scenario)

    result_parser = subparsers.add_parser("result")
    result_parser.add_argument("run_id")
    result_parser.add_argument("--format", choices=("text", "json"), default="text")
    result_parser.set_defaults(func=show_result)

    analyze_parser = subparsers.add_parser("analyze")
    analyze_parser.add_argument("scenario", choices=("safegate2-stopped-overtake",))
    analyze_parser.add_argument("run_id")
    analyze_parser.set_defaults(func=analyze_safegate2_stopped_overtake)

    review_parser = subparsers.add_parser("external-review")
    review_subparsers = review_parser.add_subparsers(dest="review_command", required=True)

    enqueue_parser = review_subparsers.add_parser("enqueue")
    enqueue_parser.add_argument("--execution-id", required=True)
    enqueue_parser.add_argument("--review-attempt-id", required=True)
    enqueue_parser.add_argument("--handoff", required=True)
    enqueue_parser.add_argument("--packet", required=True)
    enqueue_parser.add_argument("--objective", required=True)
    enqueue_parser.add_argument("--requested-by", choices=("vscode", "bridge"), required=True)
    enqueue_parser.add_argument("--retry-of")
    enqueue_parser.set_defaults(func=external_review)

    claim_parser = review_subparsers.add_parser("claim")
    claim_parser.add_argument("--execution-id", required=True)
    claim_parser.add_argument("--review-attempt-id", required=True)
    claim_parser.add_argument("--worker-id", required=True)
    claim_parser.set_defaults(func=external_review)

    defer_parser = review_subparsers.add_parser("defer")
    defer_parser.add_argument("--execution-id", required=True)
    defer_parser.add_argument("--review-attempt-id", required=True)
    defer_parser.add_argument("--worker-id", required=True)
    defer_parser.add_argument("--connection-evidence", required=True)
    defer_parser.add_argument("--connection-evidence-sha256", required=True)
    defer_parser.add_argument("--terminal-connection-error")
    defer_parser.set_defaults(func=external_review)

    complete_parser = review_subparsers.add_parser("complete")
    complete_parser.add_argument("--execution-id", required=True)
    complete_parser.add_argument("--review-attempt-id", required=True)
    complete_parser.add_argument("--worker-id", required=True)
    complete_parser.add_argument("--model", choices=("GPT-5.6 Sol Pro", "GPT-5.6 Pro"), required=True)
    complete_parser.add_argument("--severity", choices=("Blocker", "Major", "Minor", "None"), required=True)
    complete_parser.add_argument("--disposition", required=True)
    complete_parser.add_argument("--transition-permission", choices=("GO", "HOLD"), required=True)
    complete_parser.add_argument("--response", required=True)
    complete_parser.add_argument("--minimum-change", action="append", default=[])
    complete_parser.add_argument("--submission-id", required=True)
    complete_parser.add_argument("--project-url", required=True)
    complete_parser.add_argument("--fallback-reason")
    complete_parser.set_defaults(func=external_review)

    status_parser = review_subparsers.add_parser("status")
    status_parser.add_argument("--execution-id", required=True)
    status_parser.add_argument("--review-attempt-id")
    status_parser.set_defaults(func=external_review)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))
