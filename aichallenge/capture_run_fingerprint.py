#!/usr/bin/env python3
"""Capture a self-contained, run-local fingerprint before AWSIM starts.

The recorder is intentionally host-side and ROS-independent.  It reads the
bind-mounted source/build inputs, writes only below the requested raw run
directory, and never mutates source, build, install, or output/latest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from datetime import datetime
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, Mapping


SCHEMA_VERSION = 2
PROVENANCE_DIRNAME = "provenance"
RUN_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")
DOMAIN_ID_PATTERN = re.compile(r"[1-9][0-9]*")
MAX_ARTIFACT_DOMAIN_ID = 4
EXCLUDED_SOURCE_PARTS = {
    ".git",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
}

RUNTIME_ARTIFACTS = {
    "overtake_planner_node": "aichallenge/workspace/build/overtake_planner/overtake_planner_node",
    "overtake_planner_core": "aichallenge/workspace/build/overtake_planner/libovertake_planner_core.so",
    "simple_pure_pursuit": "aichallenge/workspace/build/simple_pure_pursuit/simple_pure_pursuit",
    "simple_trajectory_generator": (
        "aichallenge/workspace/build/simple_trajectory_generator/simple_trajectory_generator_node"
    ),
    "wall_recovery_planner": (
        "aichallenge/workspace/build/wall_recovery_planner/wall_recovery_planner_node"
    ),
    "racing_kart_gnss_poser": "aichallenge/workspace/build/racing_kart_gnss_poser/gnss_poser",
    "racing_kart_gnss_poser_core": (
        "aichallenge/workspace/build/racing_kart_gnss_poser/libgnss_poser_node.so"
    ),
    "delay_compensated_odometry": (
        "aichallenge/workspace/build/delay_aware_mpc_ros/delay_compensated_odometry_node"
    ),
}

RUNTIME_INSTALL_LINKS = {
    "overtake_planner_node_install": (
        "aichallenge/workspace/install/overtake_planner/lib/overtake_planner/overtake_planner_node"
    ),
    "overtake_planner_core_install": (
        "aichallenge/workspace/install/overtake_planner/lib/libovertake_planner_core.so"
    ),
    "overtake_planner_config_install": (
        "aichallenge/workspace/install/overtake_planner/share/overtake_planner/config/"
        "overtake_planner.param.yaml"
    ),
}

RUNTIME_TREE_INPUTS = {
    "workspace_build": "aichallenge/workspace/build",
    "workspace_install": "aichallenge/workspace/install",
    "mpc_python_environment": (
        "aichallenge/workspace/install/multi_purpose_mpc_ros/.venv"
    ),
    "mpc_ros_install": "aichallenge/workspace/install/multi_purpose_mpc_ros",
    "mpc_message_runtime": (
        "aichallenge/workspace/install/multi_purpose_mpc_ros_msgs"
    ),
}

LAUNCH_CONFIG_INPUTS = {
    "overtake_planner_config": (
        "aichallenge/workspace/src/aichallenge_submit/overtake_planner/config/"
        "overtake_planner.param.yaml"
    ),
    "overtake_planner_launch": (
        "aichallenge/workspace/src/aichallenge_submit/overtake_planner/launch/"
        "overtake_planner.launch.xml"
    ),
    "hybrid_control_mux_config": (
        "aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/config/"
        "hybrid_control_mux.param.yaml"
    ),
    "pure_pursuit_horizon_config": (
        "aichallenge/workspace/src/aichallenge_submit/hybrid_control_mux/config/"
        "pure_pursuit_mpc_horizon.param.yaml"
    ),
    "mpc_config": (
        "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml"
    ),
    "hybrid_control_launch": (
        "aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/"
        "hybrid_delay_aware_mpc.launch.xml"
    ),
    "pure_pursuit_launch": (
        "aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/"
        "pure_pursuit.launch.xml"
    ),
    "pure_pursuit_horizon_launch": (
        "aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/"
        "pure_pursuit_mpc_horizon.launch.xml"
    ),
    "state_lattice_pure_pursuit_launch": (
        "aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/"
        "state_lattice_pure_pursuit.launch.xml"
    ),
    "state_lattice_config": (
        "aichallenge/workspace/src/aichallenge_submit/state_lattice_overtake_planner/config/"
        "state_lattice_overtake_planner.param.yaml"
    ),
    "state_lattice_launch": (
        "aichallenge/workspace/src/aichallenge_submit/state_lattice_overtake_planner/launch/"
        "state_lattice_overtake_planner.launch.xml"
    ),
}

HARNESS_INPUTS = {
    "makefile": "Makefile",
    "fingerprint_recorder": "aichallenge/capture_run_fingerprint.py",
    "docker_compose": "docker-compose.yml",
    "docker_compose_gpu": "docker-compose.gpu.yml",
    "dockerfile": "Dockerfile",
    "run_autoware": "aichallenge/run_autoware.bash",
    "run_simulator": "aichallenge/run_simulator.bash",
    "request_awsim_start": "aichallenge/request_awsim_start.bash",
    "helper_process_reaper": "aichallenge/helper_process_reaper.py",
    "admin_state_observer": "aichallenge/admin_state_observer.py",
    "race_arm_observer": "aichallenge/race_arm_observer.py",
    "vehicle_readiness_observer": "aichallenge/vehicle_readiness_observer.py",
    "wait_for_typed_service": "aichallenge/wait_for_typed_service.py",
    "run_awsim_with_d1_watchdog": "aichallenge/run_awsim_with_d1_watchdog.bash",
    "d1_start_progress_watchdog": "aichallenge/d1_start_progress_watchdog.py",
    "autostart_orchestrator": (
        "aichallenge/workspace/src/aichallenge_system/autostart_orchestrator_py/"
        "autostart_orchestrator_py/autostart_orchestrator_node.py"
    ),
}

SIMULATOR_INPUTS = {
    "awsim_executable": "aichallenge/simulator/AWSIM/AWSIM.x86_64",
    "unity_player": "aichallenge/simulator/AWSIM/UnityPlayer.so",
    "safety_gate_plugin": (
        "aichallenge/simulator/AWSIM/AWSIM_Data/Managed/AIChallenge2026.SafetyGate.dll"
    ),
    "awsim_game_logic": (
        "aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll"
    ),
    "awsim_global_managers": "aichallenge/simulator/AWSIM/AWSIM_Data/globalgamemanagers",
    "awsim_level0": "aichallenge/simulator/AWSIM/AWSIM_Data/level0",
    "awsim_level1": "aichallenge/simulator/AWSIM/AWSIM_Data/level1",
    "awsim_level2": "aichallenge/simulator/AWSIM/AWSIM_Data/level2",
    "awsim_level3": "aichallenge/simulator/AWSIM/AWSIM_Data/level3",
}

SIMULATOR_TREE_INPUTS = {
    "awsim_runtime": "aichallenge/simulator/AWSIM",
    "streaming_assets": "aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets",
}

SUBMISSION_INPUTS = {
    "submission_tar": "submit/aichallenge_submit.tar.gz",
}

PLAINTEXT_ENV_KEYS = (
    "RUN_KIND",
    "SIM_MODE",
    "CONTROL_METHOD",
    "PP_CORE_EXACT_SNAPSHOT_ENABLED",
    "OVERTAKE_TRAJECTORY_BACKEND",
    "PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL",
    "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED",
    "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED",
    "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID",
    "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID",
    "STATE_LATTICE_V2_SESSION_ID",
    "STATE_LATTICE_V4_POC_COMMAND_ACTIVATION_ENABLED",
    "STATE_LATTICE_EXACT_SPATIAL_FOLLOW_SHADOW_ENABLED",
    "ROSBAG",
    "AWSIM_START_MODE",
    "AWSIM_START_COUNT_SECONDS",
    "AWSIM_VEHICLES",
    "AWSIM_LAPS",
    "AWSIM_TIMEOUT",
    "RACE_ARM_ON_VEHICLE_STATE",
    "AUTOWARE_RUN_MODE",
    "AUTOSTART_DEBUG_VISUALIZATION",
    "AUTOWARE_RUNTIME_IMAGE",
    "AIC_EXPECTED_AUTOWARE_RUNTIME_IMAGE_ID",
    "AIC_TEST_GATE2_REVIEW_ANCHOR_PATH",
    "AIC_TEST_GATE2_REVIEW_ANCHOR_SHA256",
    "AIC_TEST_GATE2_REVIEW_STATE_PATH",
    "AIC_TEST_GATE2_REVIEW_STATE_SHA256",
    "AIC_TEST_GATE2_REVIEW_RESULT_PATH",
    "AIC_TEST_GATE2_REVIEW_RESULT_SHA256",
    "AIC_TEST_GATE2_REVIEWED_EXECUTION_ID",
    "AIC_TEST_GATE2_REVIEW_ATTEMPT_ID",
    "AIC_TEST_GATE2_REVIEWED_HANDOFF_PATH",
    "AIC_TEST_GATE2_REVIEWED_HANDOFF_SHA256",
    "AIC_TEST_GATE2_REVIEW_PACKET_PATH",
    "AIC_TEST_GATE2_REVIEW_PACKET_SHA256",
    "AIC_TEST_GATE2_REVIEW_BUNDLE_PATH",
    "AIC_TEST_GATE2_REVIEW_BUNDLE_SHA256",
    "AIC_TEST_GATE2_REVIEW_IMAGE_ID",
    "AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256",
    "D1_STALL_TIMEOUT_SEC",
    "D1_STALL_ENTER_SPEED_MPS",
    "D1_STALL_EXIT_SPEED_MPS",
    "D1_VELOCITY_FRESHNESS_SEC",
    "D1_VELOCITY_EVIDENCE_FAILURE_SEC",
    "DEV_AUTO_START",
    "DEV_AUTO_START_EFFECTIVE_MODE",
    "AWSIM_READY_DOMAINS",
    "RUN_GATE_SCENARIO",
    "RUN_GATE_ARG",
    "OUTPUT_ROOT",
    "OUTPUT_HOST_ROOT",
)
HASHED_ENV_KEYS = (
    "AWSIM_EXTRA_ARGS",
    "GATE_EXTRA_ARGS",
)
GATE2_REVIEW_ANCHOR_PATH_KEYS = (
    "AIC_TEST_GATE2_REVIEW_ANCHOR_PATH",
    "AIC_TEST_GATE2_REVIEW_STATE_PATH",
    "AIC_TEST_GATE2_REVIEW_RESULT_PATH",
    "AIC_TEST_GATE2_REVIEWED_HANDOFF_PATH",
    "AIC_TEST_GATE2_REVIEW_PACKET_PATH",
    "AIC_TEST_GATE2_REVIEW_BUNDLE_PATH",
)
GATE2_REVIEW_ANCHOR_SHA_KEYS = (
    "AIC_TEST_GATE2_REVIEW_ANCHOR_SHA256",
    "AIC_TEST_GATE2_REVIEW_STATE_SHA256",
    "AIC_TEST_GATE2_REVIEW_RESULT_SHA256",
    "AIC_TEST_GATE2_REVIEWED_HANDOFF_SHA256",
    "AIC_TEST_GATE2_REVIEW_PACKET_SHA256",
    "AIC_TEST_GATE2_REVIEW_BUNDLE_SHA256",
    "AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256",
)
GATE2_NORMALIZED_SERVICE_KEYS = frozenset({
    "image", "command", "entrypoint", "environment", "mounts", "privileged",
    "network_mode", "security_opt", "cap_add", "read_only", "devices",
    "working_dir", "stop_signal", "stop_grace_period", "pull_policy",
})
GATE2_RENDERED_SERVICE_KEYS = frozenset({
    "image", "command", "entrypoint", "environment", "volumes", "privileged",
    "network_mode", "security_opt", "cap_add", "read_only", "devices",
    "working_dir", "stop_signal", "stop_grace_period", "pull_policy",
})


class FingerprintError(RuntimeError):
    """Raised when a complete, immutable fingerprint cannot be produced."""


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return _sha256_bytes(encoded)


def _gate2_review_launch_spec(
    repo_root: Path, plain: Mapping[str, Any], launch_spec_sha256: str,
) -> dict[str, Any]:
    """Load the fixed queue index's normalized launch contract."""
    raw_path = plain.get("AIC_TEST_GATE2_REVIEW_ANCHOR_PATH")
    if not isinstance(raw_path, str) or not Path(raw_path).is_absolute():
        raise FingerprintError("Gate 2 review launch index path is missing")
    anchor_sha256 = plain.get("AIC_TEST_GATE2_REVIEW_ANCHOR_SHA256")
    if not isinstance(anchor_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", anchor_sha256):
        raise FingerprintError("Gate 2 review launch index hash is missing")
    path = Path(raw_path)
    expected_root = repo_root.resolve(strict=True) / "analysis" / "aic_test" / "external_review_queue" / "gate2_authorizations"
    try:
        if path.resolve(strict=True).parent != expected_root or path.is_symlink() or stat.S_IMODE(path.stat().st_mode) != 0o644:
            raise FingerprintError("Gate 2 review launch index path is unsafe")
        if _sha256_file(path) != anchor_sha256:
            raise FingerprintError("Gate 2 review launch index hash mismatch")
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise FingerprintError("Gate 2 review launch index is unreadable") from error
    launch_spec = payload.get("launch_spec") if isinstance(payload, dict) else None
    if not isinstance(launch_spec, dict) or _canonical_sha256(launch_spec) != launch_spec_sha256:
        raise FingerprintError("Gate 2 review launch specification hash mismatch")
    return launch_spec


def _relative_path(path: Path, repo_root: Path) -> str:
    try:
        return path.relative_to(repo_root).as_posix()
    except ValueError as error:
        raise FingerprintError(f"path escapes repo root: {path}") from error


def _path_record(path: Path, repo_root: Path, *, required: bool = True) -> dict[str, Any]:
    relative = _relative_path(path, repo_root)
    if path.is_symlink():
        target = os.readlink(path)
        return {
            "path": relative,
            "type": "symlink",
            "mode": stat.S_IMODE(path.lstat().st_mode),
            "target": target,
            "sha256": _sha256_bytes(("symlink\0" + target).encode("utf-8")),
        }
    if not path.exists():
        if required:
            raise FingerprintError(f"required artifact is missing: {relative}")
        return {"path": relative, "type": "missing"}
    if not path.is_file():
        raise FingerprintError(f"required artifact is not a file: {relative}")
    file_stat = path.stat()
    return {
        "path": relative,
        "type": "file",
        "mode": stat.S_IMODE(file_stat.st_mode),
        "size": file_stat.st_size,
        "sha256": _sha256_file(path),
    }


def _is_excluded_source_path(relative: Path) -> bool:
    return any(part in EXCLUDED_SOURCE_PARTS for part in relative.parts) or relative.suffix == ".pyc"


def collect_source_records(repo_root: Path) -> list[dict[str, Any]]:
    source_root = repo_root / "aichallenge/workspace/src"
    if not source_root.is_dir():
        raise FingerprintError(f"source root is missing: {source_root}")
    records: list[dict[str, Any]] = []
    for path in sorted(source_root.rglob("*"), key=lambda item: item.as_posix()):
        relative_to_source = path.relative_to(source_root)
        if _is_excluded_source_path(relative_to_source):
            continue
        if path.is_file() or path.is_symlink():
            records.append(_path_record(path, repo_root))
    if not records:
        raise FingerprintError("source tree contains no fingerprintable files")
    return records


def _collect_stable_source_records(repo_root: Path) -> list[dict[str, Any]]:
    first = collect_source_records(repo_root)
    second = collect_source_records(repo_root)
    if first != second:
        raise FingerprintError("source tree changed while fingerprint was being captured")
    return second


def collect_named_paths(
    repo_root: Path,
    paths: Mapping[str, str],
    *,
    required: bool = True,
) -> dict[str, dict[str, Any]]:
    return {
        name: _path_record(repo_root / relative, repo_root, required=required)
        for name, relative in sorted(paths.items())
    }


def collect_tree_records(
    repo_root: Path,
    relative_root: str,
    *,
    excluded_parts: frozenset[str] = frozenset({"__pycache__"}),
    exclude_pyc: bool = True,
) -> dict[str, Any]:
    tree_root = repo_root / relative_root
    if not tree_root.is_dir():
        raise FingerprintError(f"required runtime tree is missing: {relative_root}")
    records: list[dict[str, Any]] = []
    for path in sorted(tree_root.rglob("*"), key=lambda item: item.as_posix()):
        relative_to_tree = path.relative_to(tree_root)
        if any(part in excluded_parts for part in relative_to_tree.parts):
            continue
        if exclude_pyc and relative_to_tree.suffix == ".pyc":
            continue
        if path.is_file() or path.is_symlink():
            records.append(_path_record(path, repo_root))
    if not records:
        raise FingerprintError(f"runtime tree contains no fingerprintable files: {relative_root}")
    return {
        "root": relative_root,
        "file_count": len(records),
        "sha256": _group_hash(records),
        "records": records,
    }


def collect_runtime_trees(repo_root: Path) -> dict[str, dict[str, Any]]:
    return {
        "workspace_build": collect_tree_records(
            repo_root, RUNTIME_TREE_INPUTS["workspace_build"]
        ),
        "workspace_install": collect_tree_records(
            repo_root,
            RUNTIME_TREE_INPUTS["workspace_install"],
            excluded_parts=frozenset({".venv", "__pycache__"}),
        ),
        "mpc_python_environment": collect_tree_records(
            repo_root, RUNTIME_TREE_INPUTS["mpc_python_environment"]
        ),
        "mpc_ros_install": collect_tree_records(
            repo_root,
            RUNTIME_TREE_INPUTS["mpc_ros_install"],
            excluded_parts=frozenset({".venv", "__pycache__"}),
        ),
        "mpc_message_runtime": collect_tree_records(
            repo_root, RUNTIME_TREE_INPUTS["mpc_message_runtime"]
        ),
    }


def collect_simulator_trees(repo_root: Path) -> dict[str, dict[str, Any]]:
    return {
        name: collect_tree_records(
            repo_root,
            relative,
            excluded_parts=frozenset(),
            exclude_pyc=False,
        )
        for name, relative in sorted(SIMULATOR_TREE_INPUTS.items())
    }


def _run_command(command: list[str], cwd: Path) -> bytes:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise FingerprintError(f"command failed: {' '.join(command)}: {error}") from error
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        raise FingerprintError(f"command failed: {' '.join(command)}: {detail}")
    return result.stdout


def collect_git_info(repo_root: Path) -> dict[str, Any]:
    commit = _run_command(["git", "rev-parse", "HEAD"], repo_root).decode().strip()
    branch = _run_command(["git", "branch", "--show-current"], repo_root).decode().strip()
    status = _run_command(
        ["git", "status", "--porcelain=v1", "-z", "--untracked-files=all"], repo_root
    )
    diff = _run_command(["git", "diff", "--binary", "HEAD", "--"], repo_root)
    return {
        "branch": branch or None,
        "commit": commit,
        "dirty": bool(status),
        "status_sha256": _sha256_bytes(status),
        "diff_sha256": _sha256_bytes(diff),
    }


def collect_docker_image_id(repo_root: Path, image: str) -> str:
    output = _run_command(
        ["docker", "image", "inspect", "--format", "{{.Id}}", image], repo_root
    ).decode("utf-8", errors="strict").strip()
    if not output.startswith("sha256:"):
        raise FingerprintError(f"unexpected Docker image id for {image}: {output!r}")
    return output


def collect_launch_context(environment: Mapping[str, str]) -> dict[str, Any]:
    plain = {key: environment.get(key, "") for key in PLAINTEXT_ENV_KEYS}
    hashed = {
        key: {
            "present": bool(environment.get(key, "")),
            "sha256": _sha256_bytes(environment.get(key, "").encode("utf-8")),
        }
        for key in HASHED_ENV_KEYS
    }
    return {"plain": plain, "hashed": hashed}


def _group_hash(records: Any) -> str:
    return _canonical_sha256(records)


def build_snapshot(
    repo_root: Path,
    launch_context: Mapping[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    source_records = _collect_stable_source_records(repo_root)
    source_tree_sha256 = _group_hash(source_records)
    runtime_artifacts = collect_named_paths(repo_root, RUNTIME_ARTIFACTS)
    runtime_install_links = collect_named_paths(repo_root, RUNTIME_INSTALL_LINKS)
    runtime_trees = collect_runtime_trees(repo_root)
    launch_config = collect_named_paths(repo_root, LAUNCH_CONFIG_INPUTS)
    harness = collect_named_paths(repo_root, HARNESS_INPUTS)
    simulator = collect_named_paths(repo_root, SIMULATOR_INPUTS)
    simulator_trees = collect_simulator_trees(repo_root)
    submission = collect_named_paths(repo_root, SUBMISSION_INPUTS)
    plain = launch_context.get("plain", {})
    runtime_image = plain.get("AUTOWARE_RUNTIME_IMAGE") if isinstance(plain, Mapping) else None
    if runtime_image not in {"aichallenge-2025-dev", "aichallenge-2025-eval"}:
        raise FingerprintError(f"unsupported AUTOWARE_RUNTIME_IMAGE: {runtime_image!r}")
    docker_image_id = collect_docker_image_id(repo_root, runtime_image)
    expected_image_id = plain.get("AIC_EXPECTED_AUTOWARE_RUNTIME_IMAGE_ID") if isinstance(plain, Mapping) else ""
    is_gate2_profile = isinstance(plain, Mapping) and plain.get("RUN_KIND") == "planner-pp-control-smoke"
    if is_gate2_profile:
        if not isinstance(expected_image_id, str) or not re.fullmatch(r"sha256:[0-9a-f]{64}", expected_image_id):
            raise FingerprintError("expected eval image id is missing or invalid")
        if docker_image_id != expected_image_id:
            raise FingerprintError("expected eval image id does not match tagged image")
        if any(
            not isinstance(plain.get(key), str)
            or not Path(plain[key]).is_absolute()
            for key in GATE2_REVIEW_ANCHOR_PATH_KEYS
        ):
            raise FingerprintError("Gate 2 review anchor path is missing or invalid")
        if any(
            not isinstance(plain.get(key), str)
            or not re.fullmatch(r"[0-9a-f]{64}", plain[key])
            for key in GATE2_REVIEW_ANCHOR_SHA_KEYS
        ):
            raise FingerprintError("Gate 2 review anchor hash is missing or invalid")
        for key in ("AIC_TEST_GATE2_REVIEWED_EXECUTION_ID", "AIC_TEST_GATE2_REVIEW_ATTEMPT_ID", "AIC_TEST_GATE2_REVIEW_IMAGE_ID"):
            if not isinstance(plain.get(key), str) or not plain[key].strip():
                raise FingerprintError(f"Gate 2 review anchor field is missing: {key}")
        launch_spec = _gate2_review_launch_spec(
            repo_root, plain, plain["AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256"],
        )
    else:
        launch_spec = None
    git_info = collect_git_info(repo_root)

    groups = {
        "autonomy_artifact_sha256": _group_hash(
            {
                "source_tree_sha256": source_tree_sha256,
                "runtime_artifacts": runtime_artifacts,
                "runtime_install_links": runtime_install_links,
                "runtime_trees": runtime_trees,
                "docker_image_id": docker_image_id,
            }
        ),
        "launch_config_sha256": _group_hash(launch_config),
        "harness_sha256": _group_hash(harness),
        "simulator_sha256": _group_hash(
            {"files": simulator, "trees": simulator_trees}
        ),
        "submission_sha256": _group_hash(submission),
    }
    experiment_sha256 = _group_hash(
        {
            "schema_version": SCHEMA_VERSION,
            "groups": groups,
            "launch_context": launch_context,
        }
    )
    snapshot = {
        "schema_version": SCHEMA_VERSION,
        "captured_at": datetime.now().astimezone().isoformat(),
        "git": git_info,
        "docker": {
            "image": runtime_image,
            "image_id": docker_image_id,
        },
        "runtime_image_admission": (
            {
                "expected_image_id": expected_image_id,
                "actual_image_id": docker_image_id,
                "review_anchor_sha256": plain.get("AIC_TEST_GATE2_REVIEW_ANCHOR_SHA256"),
                "launch_spec_sha256": plain.get("AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256"),
                "launch_spec": launch_spec,
            }
            if is_gate2_profile else None
        ),
        "source_tree": {
            "root": "aichallenge/workspace/src",
            "file_count": len(source_records),
            "sha256": source_tree_sha256,
            "manifest": "source-tree.sha256",
        },
        "runtime_artifacts": runtime_artifacts,
        "runtime_install_links": runtime_install_links,
        "runtime_tree_inputs": runtime_trees,
        "launch_config_inputs": launch_config,
        "harness_inputs": harness,
        "simulator_inputs": simulator,
        "simulator_tree_inputs": simulator_trees,
        "submission_inputs": submission,
        "launch_context": launch_context,
        "fingerprints": {
            **groups,
            "experiment_sha256": experiment_sha256,
        },
    }
    return snapshot, source_records


def attest_running_service(
    repo_root: Path,
    output_root: Path,
    run_dir: Path,
    services: list[str],
    environment: Mapping[str, str],
) -> Path:
    """Bind the live autonomy container to the prelaunch eval image before Start."""
    repo_root = repo_root.resolve()
    _, run_dir = validate_run_location(output_root, run_dir)
    if not services or len(set(services)) != len(services) or any(
        not re.fullmatch(r"[a-z0-9][a-z0-9-]*", service) for service in services
    ):
        raise FingerprintError(f"invalid compose services: {services!r}")
    runtime_image = environment.get("AUTOWARE_RUNTIME_IMAGE", "")
    if runtime_image not in {"aichallenge-2025-dev", "aichallenge-2025-eval"}:
        raise FingerprintError(f"unsupported AUTOWARE_RUNTIME_IMAGE: {runtime_image!r}")
    provenance = run_dir / PROVENANCE_DIRNAME
    prelaunch_path = provenance / "prelaunch-manifest.json"
    output_path = provenance / "runtime-container-attestation.json"
    if output_path.exists() or not prelaunch_path.is_file():
        raise FingerprintError("runtime attestation destination or prelaunch manifest is invalid")
    prelaunch = json.loads(prelaunch_path.read_text(encoding="utf-8"))
    expected_image_id = collect_docker_image_id(repo_root, runtime_image)
    if prelaunch.get("docker") != {"image": runtime_image, "image_id": expected_image_id}:
        raise FingerprintError("prelaunch Docker image binding does not match runtime request")
    admission = prelaunch.get("runtime_image_admission")
    if runtime_image == "aichallenge-2025-eval":
        if (
            not isinstance(admission, dict)
            or
            admission.get("expected_image_id") != expected_image_id
            or admission.get("actual_image_id") != expected_image_id
            or not re.fullmatch(r"[0-9a-f]{64}", admission.get("review_anchor_sha256", ""))
            or not re.fullmatch(r"[0-9a-f]{64}", admission.get("launch_spec_sha256", ""))
            or not isinstance(admission.get("launch_spec"), dict)
        ):
            raise FingerprintError("prelaunch expected eval image id does not match runtime request")
        launch_context = prelaunch.get("launch_context")
        plain = launch_context.get("plain", {}) if isinstance(launch_context, dict) else {}
        if _gate2_review_launch_spec(
            repo_root, plain, admission["launch_spec_sha256"],
        ) != admission["launch_spec"]:
            raise FingerprintError("prelaunch Gate 2 review launch index changed")
        if _rendered_launch_spec(repo_root) != admission["launch_spec"]:
            raise FingerprintError("rendered Gate 2 launch specification does not match review anchor")
    if set(services) != {"autoware-eval-runtime", "autoware-eval-command"}:
        raise FingerprintError("Gate 2 runtime service set is invalid")
    expected_project = f"aic-{run_dir.name.lower()}"
    records = [
        _inspect_running_service(repo_root, service, expected_image_id, expected_project)
        for service in services
    ]
    container_ids = [record.get("container_id") for record in records]
    if len(set(container_ids)) != len(container_ids):
        raise FingerprintError("Gate 2 runtime container IDs are not distinct")
    launch_context = prelaunch.get("launch_context")
    plain = launch_context.get("plain", {}) if isinstance(launch_context, dict) else {}
    payload = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_dir.name,
        "expected_image": runtime_image,
        "expected_image_id": expected_image_id,
        "review_anchor_sha256": plain.get("AIC_TEST_GATE2_REVIEW_ANCHOR_SHA256"),
        "launch_spec_sha256": plain.get("AIC_TEST_GATE2_REVIEW_LAUNCH_SPEC_SHA256"),
        "launch_spec": admission.get("launch_spec") if isinstance(admission, dict) else None,
        "services": records,
    }
    temporary = output_path.with_name(f".{output_path.name}.{os.getpid()}.tmp")
    _write_json(temporary, payload)
    os.replace(temporary, output_path)
    return output_path


def _inspect_running_service(
    repo_root: Path, service: str, expected_image_id: str,
    expected_project: str | None = None,
) -> dict[str, Any]:
    container_id = _run_command(["docker", "compose", "ps", "-q", service], repo_root).decode().strip()
    if not re.fullmatch(r"[0-9a-f]{12,64}", container_id):
        raise FingerprintError(f"running service container is not unique: {service}")
    inspected = json.loads(_run_command(["docker", "inspect", container_id], repo_root))
    if not isinstance(inspected, list) or len(inspected) != 1 or not isinstance(inspected[0], dict):
        raise FingerprintError("running service inspection has invalid schema")
    container = inspected[0]
    config = container.get("Config") if isinstance(container.get("Config"), dict) else {}
    host_config = container.get("HostConfig") if isinstance(container.get("HostConfig"), dict) else {}
    mounts = container.get("Mounts")
    labels = container.get("Config", {}).get("Labels") if isinstance(container.get("Config"), dict) else None
    if container.get("Image") != expected_image_id or not isinstance(mounts, list) or not isinstance(labels, dict):
        raise FingerprintError("running service image or metadata does not match prelaunch eval image")
    if labels.get("com.docker.compose.service") != service:
        raise FingerprintError("running container compose service label mismatch")
    compose_project = labels.get("com.docker.compose.project")
    if not isinstance(compose_project, str) or not compose_project:
        raise FingerprintError("running container compose project label is missing")
    if expected_project is not None and compose_project != expected_project:
        raise FingerprintError("running service compose project label mismatch")
    destinations: list[str] = []
    forbidden_root = PurePosixPath("/aichallenge")
    for mount in mounts:
        if not isinstance(mount, dict):
            raise FingerprintError("running service mount record is invalid")
        destination = mount.get("Destination")
        if not isinstance(destination, str) or not PurePosixPath(destination).is_absolute():
            raise FingerprintError("running service mount destination is invalid")
        normalized = PurePosixPath(destination)
        destinations.append(normalized.as_posix())
        if normalized == forbidden_root or forbidden_root in normalized.parents:
            raise FingerprintError("packaged runtime must not mount /aichallenge or its subtree")
    devices = host_config.get("Devices", [])
    if not isinstance(devices, list) or any(not isinstance(device, dict) for device in devices):
        raise FingerprintError("running service device records are invalid")
    stop_timeout = host_config.get("StopTimeout")
    if stop_timeout is not None and (type(stop_timeout) is not int or stop_timeout < 0):
        raise FingerprintError("running service stop timeout is invalid")
    stop_grace_period = None if stop_timeout is None else f"{stop_timeout}s"
    stop_signal = config.get("StopSignal")
    if stop_signal is not None and not isinstance(stop_signal, str):
        raise FingerprintError("running service stop signal is invalid")
    working_dir = config.get("WorkingDir")
    if working_dir is not None and not isinstance(working_dir, str):
        raise FingerprintError("running service working directory is invalid")
    return {
        "service": service,
        "container_id": container_id,
        "actual_image_id": container.get("Image"),
        "compose_project": compose_project,
        "mount_destinations": destinations,
        "mounts": mounts,
        "launch_spec": {
            "image": container.get("Image"),
            "command": config.get("Cmd"),
            "entrypoint": config.get("Entrypoint"),
            "environment": sorted(config.get("Env", []) or []),
            "mounts": mounts,
            "privileged": host_config.get("Privileged", False),
            "network_mode": host_config.get("NetworkMode", ""),
            "security_opt": sorted(host_config.get("SecurityOpt", []) or []),
            "cap_add": sorted(host_config.get("CapAdd", []) or []),
            "read_only": host_config.get("ReadonlyRootfs", False),
            "devices": sorted(
                json.dumps(device, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
                for device in devices
            ),
            "working_dir": working_dir,
            "stop_signal": stop_signal,
            "stop_grace_period": stop_grace_period,
            # Docker inspect does not expose Compose's pull policy after create;
            # the review-bound rendered Compose contract carries the authoritative value.
            "pull_policy": None,
        },
    }


def _normalize_compose_service(service: Mapping[str, Any]) -> dict[str, Any]:
    unknown = set(service) - GATE2_RENDERED_SERVICE_KEYS
    if unknown:
        raise FingerprintError(f"Gate 2 rendered service has unknown keys: {sorted(unknown)}")
    environment = service.get("environment", {})
    if isinstance(environment, list):
        environment = {
            item.split("=", 1)[0]: item.split("=", 1)[1] if "=" in item else None
            for item in environment if isinstance(item, str)
        }
    if not isinstance(environment, dict):
        raise FingerprintError("Gate 2 rendered service environment is invalid")
    volumes = service.get("volumes", [])
    if not isinstance(volumes, list):
        raise FingerprintError("Gate 2 rendered service volumes are invalid")
    normalized_volumes = [
        json.dumps(item, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        if isinstance(item, dict) else str(item)
        for item in volumes
    ]
    command = service.get("command", [])
    if isinstance(command, str):
        command = [command]
    if not isinstance(command, list):
        raise FingerprintError("Gate 2 rendered service command is invalid")
    entrypoint = service.get("entrypoint")
    if isinstance(entrypoint, str):
        entrypoint = [entrypoint]
    if entrypoint is not None and not isinstance(entrypoint, list):
        raise FingerprintError("Gate 2 rendered service entrypoint is invalid")
    image = service.get("image")
    if isinstance(image, str) and "@sha256:" in image:
        image = image[image.index("@") + 1:]
    devices = service.get("devices", [])
    if not isinstance(devices, list):
        raise FingerprintError("Gate 2 rendered service devices are invalid")
    security_opt = service.get("security_opt", [])
    cap_add = service.get("cap_add", [])
    if not isinstance(security_opt, list) or not isinstance(cap_add, list):
        raise FingerprintError("Gate 2 rendered service security options are invalid")
    for field in ("working_dir", "stop_signal", "stop_grace_period", "pull_policy"):
        value = service.get(field)
        if value is not None and not isinstance(value, str):
            raise FingerprintError(f"Gate 2 rendered service {field} is invalid")
    if type(service.get("privileged", False)) is not bool or type(service.get("read_only", False)) is not bool:
        raise FingerprintError("Gate 2 rendered service boolean field is invalid")
    network_mode = service.get("network_mode", "")
    if not isinstance(network_mode, str):
        raise FingerprintError("Gate 2 rendered service network mode is invalid")
    return {
        "image": image,
        "command": command,
        "entrypoint": entrypoint,
        "environment": {str(key): environment[key] for key in sorted(environment)},
        "mounts": sorted(normalized_volumes),
        "privileged": service.get("privileged", False),
        "network_mode": network_mode,
        "security_opt": sorted(security_opt),
        "cap_add": sorted(cap_add),
        "read_only": service.get("read_only", False),
        "devices": sorted(
            json.dumps(item, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
            if isinstance(item, dict) else str(item)
            for item in devices
        ),
        "working_dir": service.get("working_dir"),
        "stop_signal": service.get("stop_signal"),
        "stop_grace_period": service.get("stop_grace_period"),
        "pull_policy": service.get("pull_policy"),
    }


def _rendered_launch_spec(repo_root: Path) -> dict[str, Any]:
    try:
        payload = json.loads(_run_command(["docker", "compose", "config", "--format", "json"], repo_root))
    except (FingerprintError, json.JSONDecodeError) as error:
        raise FingerprintError("Gate 2 rendered launch specification is unavailable") from error
    services = payload.get("services") if isinstance(payload, dict) else None
    names = {"autoware-eval-command", "autoware-eval-runtime"}
    if (
        not isinstance(services, dict)
        or not names.issubset(services)
        or any(not isinstance(services[name], dict) for name in names)
    ):
        raise FingerprintError("Gate 2 rendered launch service set is invalid")
    return {
        "service_set": sorted(names),
        "services": {name: _normalize_compose_service(services[name]) for name in sorted(names)},
    }


def _canonical_mounts(record: Mapping[str, Any]) -> list[str]:
    """Return full Docker mount records with only mapping/list order normalized."""
    mounts = record.get("mounts")
    if not isinstance(mounts, list) or any(not isinstance(mount, dict) for mount in mounts):
        raise FingerprintError("runtime service mount records are invalid")
    # Keep every field and duplicate entry.  Sorting their canonical JSON only
    # removes Docker inspect's non-semantic array/key ordering differences.
    return sorted(
        json.dumps(mount, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        for mount in mounts
    )


def _canonical_runtime_launch_spec(record: Mapping[str, Any]) -> Any:
    """Normalize only the duplicated Docker mount ordering in a live launch spec."""
    launch_spec = record.get("launch_spec")
    if not isinstance(launch_spec, dict):
        return launch_spec
    mounts = launch_spec.get("mounts")
    if not isinstance(mounts, list) or any(not isinstance(mount, dict) for mount in mounts):
        return launch_spec
    canonical = dict(launch_spec)
    canonical["mounts"] = _canonical_mounts(launch_spec)
    return canonical


def _identity_differences(
    original_services: list[dict[str, Any]], observed_services: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    differences: list[dict[str, Any]] = []
    for original, observed in zip(original_services, observed_services):
        service = original["service"]
        observation_error = observed.get("observation_error")
        if isinstance(observation_error, str):
            differences.append({
                "service": service,
                "field": "observation_error",
                "attested": None,
                "observed": observation_error,
            })
            continue
        for field in ("service", "container_id", "actual_image_id", "compose_project"):
            if original.get(field) != observed.get(field):
                differences.append({
                    "service": service,
                    "field": field,
                    "attested": original.get(field),
                    "observed": observed.get(field),
                })
        original_mounts = _canonical_mounts(original)
        observed_mounts = _canonical_mounts(observed)
        if original_mounts != observed_mounts:
            differences.append({
                "service": service,
                "field": "mounts",
                "attested": original.get("mounts"),
                "observed": observed.get("mounts"),
                "attested_canonical": original_mounts,
                "observed_canonical": observed_mounts,
            })
        original_launch_spec = _canonical_runtime_launch_spec(original)
        observed_launch_spec = _canonical_runtime_launch_spec(observed)
        if original_launch_spec != observed_launch_spec:
            differences.append({
                "service": service,
                "field": "launch_spec",
                "attested": original.get("launch_spec"),
                "observed": observed.get("launch_spec"),
                "attested_canonical": original_launch_spec,
                "observed_canonical": observed_launch_spec,
            })
    return differences


def _write_identity_evidence(
    path: Path,
    *,
    run_id: str,
    identity_continuous: bool,
    original_services: list[dict[str, Any]],
    observed_services: list[dict[str, Any]],
    differences: list[dict[str, Any]],
    launch_spec: Mapping[str, Any] | None = None,
) -> None:
    payload = {
        "schema_version": SCHEMA_VERSION,
        "run_id": run_id,
        "identity_continuous": identity_continuous,
        "launch_spec": launch_spec,
        # Keep `services` for the existing postrun contract.  It deliberately
        # remains the immutable attested identity, never the later observation.
        "services": original_services,
        "attested_services": original_services,
        "observed_services": observed_services,
        "differences": differences,
    }
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    try:
        _write_json(temporary, payload)
        os.replace(temporary, path)
    except OSError as exc:
        raise FingerprintError(f"failed to persist runtime identity evidence: {path}") from exc


def verify_running_attestation(
    repo_root: Path, output_root: Path, run_dir: Path, *, seal: bool,
) -> Path:
    repo_root = repo_root.resolve()
    _, run_dir = validate_run_location(output_root, run_dir)
    provenance = run_dir / PROVENANCE_DIRNAME
    attestation_path = provenance / "runtime-container-attestation.json"
    if not attestation_path.is_file():
        raise FingerprintError("runtime container attestation is missing")
    prelaunch_path = provenance / "prelaunch-manifest.json"
    if prelaunch_path.is_file():
        try:
            prelaunch = json.loads(prelaunch_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise FingerprintError("prelaunch manifest is unavailable for attestation verification") from error
    else:
        # Legacy unit fixtures exercise identity continuity without a Gate2
        # prelaunch.  The production Gate2 path always has this manifest and
        # therefore takes the strict review-bound launch branch below.
        prelaunch = None
    attestation = json.loads(attestation_path.read_text(encoding="utf-8"))
    services = attestation.get("services") if isinstance(attestation, dict) else None
    expected_image_id = attestation.get("expected_image_id") if isinstance(attestation, dict) else None
    if (
        attestation.get("run_id") != run_dir.name
        or not isinstance(expected_image_id, str)
        or not isinstance(services, list)
        or len(services) != 2
        or {record.get("service") for record in services if isinstance(record, dict)} != {"autoware-eval-runtime", "autoware-eval-command"}
    ):
        raise FingerprintError("runtime container attestation schema or run binding is invalid")
    admission = prelaunch.get("runtime_image_admission") if isinstance(prelaunch, dict) else None
    if isinstance(prelaunch, dict) and prelaunch.get("docker", {}).get("image") == "aichallenge-2025-eval":
        if (
            not isinstance(admission, dict)
            or attestation.get("review_anchor_sha256") != admission.get("review_anchor_sha256")
            or attestation.get("launch_spec_sha256") != admission.get("launch_spec_sha256")
            or attestation.get("launch_spec") != admission.get("launch_spec")
            or _canonical_sha256(admission.get("launch_spec")) != admission.get("launch_spec_sha256")
        ):
            raise FingerprintError("runtime container attestation launch specification is not review-bound")
        launch_context = prelaunch.get("launch_context")
        plain = launch_context.get("plain", {}) if isinstance(launch_context, dict) else {}
        if _gate2_review_launch_spec(
            repo_root, plain, admission["launch_spec_sha256"],
        ) != admission["launch_spec"]:
            raise FingerprintError("runtime container attestation review index changed")
    if any(not isinstance(record, dict) for record in services):
        raise FingerprintError("runtime container attestation services are invalid")
    service_names = [record.get("service") for record in services]
    if (
        any(not isinstance(service, str) for service in service_names)
        or len(set(service_names)) != len(service_names)
        or len({record.get("container_id") for record in services}) != len(services)
        or any(not isinstance(record.get("container_id"), str) or not re.fullmatch(r"[0-9a-f]{12,64}", record["container_id"]) for record in services)
    ):
        raise FingerprintError("runtime container attestation service ownership is invalid")
    original_services = services
    current: list[dict[str, Any]] = []
    for service in service_names:
        try:
            current.append(_inspect_running_service(repo_root, service, expected_image_id))
        except Exception as exc:
            # Observation/parsing/schema failures are evidence too.  Preserve
            # them before failing closed; never catch BaseException signals.
            current.append({"service": service, "observation_error": str(exc)})
    differences = _identity_differences(original_services, current)
    output_path = provenance / (
        "runtime-container-continuity.json" if seal
        else "runtime-container-prestart-verification.json"
    )
    if seal and output_path.exists():
        raise FingerprintError("runtime container continuity seal already exists")
    _write_identity_evidence(
        output_path,
        run_id=run_dir.name,
        identity_continuous=not differences,
        original_services=original_services,
        observed_services=current,
        differences=differences,
        launch_spec=attestation.get("launch_spec") if isinstance(attestation.get("launch_spec"), dict) else None,
    )
    if differences:
        raise FingerprintError("runtime service identity changed after attestation")
    return output_path


def _validate_continuity_evidence(
    attestation: Mapping[str, Any], continuity: Mapping[str, Any], run_id: str,
) -> None:
    attested_services = attestation.get("services")
    observed_services = continuity.get("observed_services")
    differences = continuity.get("differences")
    if (
        continuity.get("run_id") != run_id
        or continuity.get("identity_continuous") is not True
        or continuity.get("services") != attested_services
        or continuity.get("attested_services") != attested_services
        or continuity.get("launch_spec") != attestation.get("launch_spec")
        or not isinstance(observed_services, list)
        or len(observed_services) != 2
        or differences != []
        or not isinstance(attested_services, list)
        or any(not isinstance(record, dict) for record in attested_services)
        or any(not isinstance(record, dict) for record in observed_services)
    ):
        raise FingerprintError("runtime container continuity seal is invalid")
    expected_names = [record.get("service") for record in attested_services]
    observed_names = [record.get("service") for record in observed_services]
    expected_ids = [record.get("container_id") for record in attested_services]
    observed_ids = [record.get("container_id") for record in observed_services]
    if (
        set(expected_names) != {"autoware-eval-runtime", "autoware-eval-command"}
        or len(set(expected_ids)) != len(expected_ids)
        or any(not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{12,64}", value) for value in expected_ids)
        or expected_names != observed_names
        or expected_ids != observed_ids
        or _identity_differences(attested_services, observed_services)
    ):
        raise FingerprintError("runtime container continuity seal is invalid")


def resolve_attested_service(
    repo_root: Path, output_root: Path, run_dir: Path, service: str,
) -> str:
    """Return the exact validated container ID; callers must exec this ID directly."""
    verify_running_attestation(repo_root, output_root, run_dir, seal=False)
    provenance = run_dir.resolve() / PROVENANCE_DIRNAME
    attestation = json.loads(
        (provenance / "runtime-container-attestation.json").read_text(encoding="utf-8")
    )
    matches = [
        record for record in attestation.get("services", [])
        if isinstance(record, dict) and record.get("service") == service
    ]
    if len(matches) != 1 or not re.fullmatch(r"[0-9a-f]{12,64}", matches[0].get("container_id", "")):
        raise FingerprintError("attested service container ID is unavailable")
    return matches[0]["container_id"]


def build_stable_snapshot(
    repo_root: Path,
    launch_context: Mapping[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    first, first_source_records = build_snapshot(repo_root, launch_context)
    second, second_source_records = build_snapshot(repo_root, launch_context)
    if first["fingerprints"] != second["fingerprints"]:
        raise FingerprintError("runtime inputs changed while fingerprint was being captured")
    if first_source_records != second_source_records:
        raise FingerprintError("source inputs changed while fingerprint was being captured")
    second["capture_stability"] = {"full_snapshot_passes": 2, "match": True}
    return second, second_source_records


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _write_source_manifest(path: Path, records: Iterable[Mapping[str, Any]]) -> None:
    lines = [f"{record['sha256']}  {record['path']}" for record in records]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_fingerprint_summary(path: Path, fingerprints: Mapping[str, str]) -> None:
    lines = [f"{name}={value}" for name, value in sorted(fingerprints.items())]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def validate_run_location(output_root: Path, run_dir: Path) -> tuple[Path, Path]:
    if run_dir.is_symlink():
        raise FingerprintError(f"run directory must not be a symlink: {run_dir}")
    resolved_output_root = output_root.resolve()
    resolved_run_dir = run_dir.resolve()
    if resolved_run_dir.parent != resolved_output_root:
        raise FingerprintError(
            f"run directory must be a direct child of output root: {resolved_run_dir}"
        )
    if not RUN_ID_PATTERN.fullmatch(resolved_run_dir.name):
        raise FingerprintError(f"invalid run id: {resolved_run_dir.name!r}")
    if resolved_run_dir.name in {"latest", ".", ".."}:
        raise FingerprintError(f"reserved run id: {resolved_run_dir.name!r}")
    return resolved_output_root, resolved_run_dir


def prepare_domain_artifact_dirs(run_dir: Path, domains_csv: str) -> list[Path]:
    """Create host-owned domain directories before any container writer starts."""
    raw_domains = [item.strip() for item in domains_csv.split(",")]
    if not raw_domains or any(not item for item in raw_domains):
        raise FingerprintError("artifact domains must be a non-empty CSV")
    if any(not DOMAIN_ID_PATTERN.fullmatch(item) for item in raw_domains):
        raise FingerprintError(f"invalid artifact domains: {domains_csv!r}")

    domain_ids = [int(item) for item in raw_domains]
    if len(set(domain_ids)) != len(domain_ids):
        raise FingerprintError(f"duplicate artifact domains: {domains_csv!r}")
    if any(domain_id > MAX_ARTIFACT_DOMAIN_ID for domain_id in domain_ids):
        raise FingerprintError(
            f"artifact domain exceeds supported maximum {MAX_ARTIFACT_DOMAIN_ID}: "
            f"{domains_csv!r}"
        )
    if run_dir.is_symlink() or not run_dir.is_dir():
        raise FingerprintError(f"run directory is not a real directory: {run_dir}")

    current_uid = os.geteuid()
    current_gid = os.getegid()
    run_stat = run_dir.stat()
    if run_stat.st_uid != current_uid or run_stat.st_gid != current_gid:
        raise FingerprintError(
            "run directory ownership does not match host process: "
            f"path={run_dir} owner={run_stat.st_uid}:{run_stat.st_gid} "
            f"process={current_uid}:{current_gid}"
        )

    domain_dirs = [run_dir / f"d{domain_id}" for domain_id in domain_ids]
    for domain_dir in domain_dirs:
        if domain_dir.is_symlink() or domain_dir.exists():
            raise FingerprintError(
                f"refusing existing domain artifact path: {domain_dir}"
            )

    created: list[Path] = []
    for domain_dir in domain_dirs:
        try:
            domain_dir.mkdir(mode=0o775, exist_ok=False)
            domain_dir.chmod(0o775)
        except OSError as exc:
            raise FingerprintError(
                f"failed to create domain artifact directory: {domain_dir}: {exc}"
            ) from exc
        domain_stat = domain_dir.stat()
        if (
            domain_stat.st_uid != current_uid
            or domain_stat.st_gid != current_gid
            or stat.S_IMODE(domain_stat.st_mode) != 0o775
        ):
            raise FingerprintError(
                "domain artifact ownership or mode mismatch: "
                f"path={domain_dir} owner={domain_stat.st_uid}:{domain_stat.st_gid} "
                f"mode={stat.S_IMODE(domain_stat.st_mode):04o}"
            )
        created.append(domain_dir)
    return created


def validate_output_mapping(
    output_root: Path,
    environment: Mapping[str, str],
) -> str:
    configured_host_root = environment.get("OUTPUT_HOST_ROOT", "")
    if not configured_host_root:
        raise FingerprintError("OUTPUT_HOST_ROOT is required")
    if Path(configured_host_root).resolve() != output_root.resolve():
        raise FingerprintError("OUTPUT_HOST_ROOT does not match --output-root")

    container_root = environment.get("OUTPUT_ROOT", "")
    container_path = PurePosixPath(container_root)
    if not container_root or not container_path.is_absolute():
        raise FingerprintError("OUTPUT_ROOT must be an absolute container path")
    if container_path == PurePosixPath("/") or ".." in container_path.parts:
        raise FingerprintError("OUTPUT_ROOT must be a scoped normalized container path")
    return container_path.as_posix()


def assert_runtime_stopped(repo_root: Path) -> None:
    commands = [["docker", "compose", "ps", "-aq"]]
    commands.extend(
        ["docker", "compose", "-p", str(project), "ps", "-aq"]
        for project in range(1, 5)
    )
    running: list[str] = []
    for command in commands:
        container_ids = _run_command(command, repo_root).decode().strip()
        if container_ids:
            running.append(" ".join(command))
    if running:
        raise FingerprintError(
            "run fingerprint requires stopped AWSIM/Autoware containers: "
            + ", ".join(running)
        )
    try:
        projects = json.loads(_run_command(["docker", "compose", "ls", "--all", "--format", "json"], repo_root))
    except (FingerprintError, json.JSONDecodeError) as error:
        raise FingerprintError("unable to enumerate all compose projects") from error
    if not isinstance(projects, list):
        raise FingerprintError("compose project enumeration has invalid schema")
    stale = [
        project.get("Name") for project in projects
        if isinstance(project, dict)
        and isinstance(project.get("Name"), str)
        and project["Name"].startswith("aic-")
    ]
    if stale:
        raise FingerprintError("run fingerprint requires no reserved aic-* compose projects: " + ", ".join(stale))


def _requires_rosbag(launch_context: Mapping[str, Any]) -> bool:
    plain = launch_context.get("plain", {})
    if not isinstance(plain, Mapping):
        return False
    return str(plain.get("ROSBAG", "")).strip().lower() in {"1", "true", "yes", "on"}


def validate_finalized_rosbags(run_dir: Path) -> list[str]:
    metadata_paths = sorted(run_dir.glob("d*/rosbag2_autoware/metadata.yaml"))
    if not metadata_paths:
        raise FingerprintError("rosbag metadata is missing or not finalized")
    validated: list[str] = []
    for metadata_path in metadata_paths:
        text = metadata_path.read_text(encoding="utf-8")
        count_match = re.search(r"(?m)^\s*message_count:\s*(\d+)\s*$", text)
        storage_matches = re.findall(
            r"(?m)^\s*-\s+([^\s]+\.(?:db3|mcap))\s*$", text
        )
        if count_match is None or int(count_match.group(1)) <= 0 or not storage_matches:
            raise FingerprintError(f"rosbag metadata is incomplete: {metadata_path}")
        for relative_storage in storage_matches:
            storage_path = PurePosixPath(relative_storage)
            if storage_path.is_absolute() or ".." in storage_path.parts:
                raise FingerprintError(
                    f"rosbag metadata contains an unsafe storage path: {metadata_path}"
                )
            absolute_storage = metadata_path.parent / Path(*storage_path.parts)
            if not absolute_storage.is_file() or absolute_storage.stat().st_size <= 0:
                raise FingerprintError(
                    f"rosbag storage is missing or empty: {absolute_storage}"
                )
        validated.append(metadata_path.relative_to(run_dir).as_posix())
    return validated


def capture_prelaunch(
    repo_root: Path,
    output_root: Path,
    run_dir: Path,
    environment: Mapping[str, str],
) -> Path:
    repo_root = repo_root.resolve()
    output_root, run_dir = validate_run_location(output_root, run_dir)
    container_output_root = validate_output_mapping(output_root, environment)
    assert_runtime_stopped(repo_root)
    if run_dir.exists() and any(run_dir.iterdir()):
        raise FingerprintError(f"refusing to reuse non-empty run directory: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    final_dir = run_dir / PROVENANCE_DIRNAME
    if final_dir.exists():
        raise FingerprintError(f"refusing to overwrite existing provenance: {final_dir}")

    launch_context = collect_launch_context(environment)
    snapshot, source_records = build_stable_snapshot(repo_root, launch_context)
    snapshot["run"] = {
        "run_id": run_dir.name,
        "host_output_root": output_root.as_posix(),
        "container_output_root": container_output_root,
    }
    temp_dir = Path(tempfile.mkdtemp(prefix=".provenance-", dir=run_dir))
    try:
        _write_json(temp_dir / "prelaunch-manifest.json", snapshot)
        _write_source_manifest(temp_dir / "source-tree.sha256", source_records)
        _write_fingerprint_summary(
            temp_dir / "artifact-fingerprint.sha256", snapshot["fingerprints"]
        )
        temp_dir.rename(final_dir)
    except Exception:
        shutil.rmtree(temp_dir, ignore_errors=True)
        raise
    return final_dir / "prelaunch-manifest.json"


def verify_postrun(repo_root: Path, output_root: Path, run_dir: Path) -> tuple[Path, bool]:
    repo_root = repo_root.resolve()
    _, run_dir = validate_run_location(output_root, run_dir)
    assert_runtime_stopped(repo_root)
    provenance = run_dir / PROVENANCE_DIRNAME
    prelaunch_path = provenance / "prelaunch-manifest.json"
    if not prelaunch_path.is_file():
        raise FingerprintError(f"prelaunch manifest is missing: {prelaunch_path}")
    postrun_path = provenance / "postrun-manifest.json"
    verification_path = provenance / "verification.json"
    if postrun_path.exists() or verification_path.exists():
        raise FingerprintError(f"refusing to overwrite postrun verification: {provenance}")

    prelaunch = json.loads(prelaunch_path.read_text(encoding="utf-8"))
    if prelaunch.get("docker", {}).get("image") == "aichallenge-2025-eval":
        admission = prelaunch.get("runtime_image_admission")
        launch_context = prelaunch.get("launch_context")
        plain = launch_context.get("plain", {}) if isinstance(launch_context, dict) else {}
        if (
            not isinstance(admission, dict)
            or not isinstance(admission.get("launch_spec_sha256"), str)
            or _gate2_review_launch_spec(
                repo_root, plain, admission["launch_spec_sha256"],
            ) != admission.get("launch_spec")
        ):
            raise FingerprintError("postrun Gate 2 review launch index changed")
        attestation_path = provenance / "runtime-container-attestation.json"
        continuity_path = provenance / "runtime-container-continuity.json"
        if not attestation_path.is_file():
            raise FingerprintError("runtime container attestation is missing")
        attestation = json.loads(attestation_path.read_text(encoding="utf-8"))
        if (
            not isinstance(attestation, dict)
            or attestation.get("schema_version") != SCHEMA_VERSION
            or attestation.get("run_id") != run_dir.name
            or attestation.get("expected_image") != prelaunch.get("docker", {}).get("image")
            or attestation.get("expected_image_id") != prelaunch.get("docker", {}).get("image_id")
            or attestation.get("review_anchor_sha256") != prelaunch.get("runtime_image_admission", {}).get("review_anchor_sha256")
            or attestation.get("launch_spec_sha256") != prelaunch.get("runtime_image_admission", {}).get("launch_spec_sha256")
            or attestation.get("launch_spec") != prelaunch.get("runtime_image_admission", {}).get("launch_spec")
            or not isinstance(attestation.get("services"), list)
            or len(attestation["services"]) != 2
            or {service.get("service") for service in attestation["services"] if isinstance(service, dict)} != {"autoware-eval-runtime", "autoware-eval-command"}
        ):
            raise FingerprintError("runtime container attestation does not match prelaunch binding")
        ids = [service.get("container_id") for service in attestation["services"] if isinstance(service, dict)]
        projects = [service.get("compose_project") for service in attestation["services"] if isinstance(service, dict)]
        if (
            any(not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{12,64}", value) for value in ids)
            or len(set(ids)) != len(ids)
            or projects != [f"aic-{run_dir.name.lower()}"] * len(projects)
            or any(service.get("actual_image_id") != prelaunch["docker"]["image_id"] for service in attestation["services"] if isinstance(service, dict))
        ):
            raise FingerprintError("runtime container attestation service identity is invalid")
        if not continuity_path.is_file():
            raise FingerprintError("runtime container continuity seal is missing")
        continuity = json.loads(continuity_path.read_text(encoding="utf-8"))
        if not isinstance(continuity, dict):
            raise FingerprintError("runtime container continuity seal is invalid")
        _validate_continuity_evidence(attestation, continuity, run_dir.name)
    launch_context = prelaunch.get("launch_context")
    if not isinstance(launch_context, dict):
        raise FingerprintError("prelaunch launch_context is invalid")
    rosbag_metadata: list[str] = []
    if _requires_rosbag(launch_context):
        rosbag_metadata = validate_finalized_rosbags(run_dir)
    postrun, _ = build_stable_snapshot(repo_root, launch_context)
    postrun["run"] = prelaunch.get("run")
    expected = prelaunch.get("fingerprints", {})
    actual = postrun.get("fingerprints", {})
    comparisons = {
        key: {"expected": expected.get(key), "actual": actual.get(key), "match": expected.get(key) == actual.get(key)}
        for key in sorted(set(expected) | set(actual))
    }
    verified = bool(comparisons) and all(item["match"] for item in comparisons.values())
    verification = {
        "schema_version": SCHEMA_VERSION,
        "verified_at": datetime.now().astimezone().isoformat(),
        "verified": verified,
        "rosbag_metadata": rosbag_metadata,
        "comparisons": comparisons,
    }

    for path, value in ((postrun_path, postrun), (verification_path, verification)):
        temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
        _write_json(temporary, value)
        os.replace(temporary, path)
    return verification_path, verified


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument(
        "--prepare-domains",
        default="",
        help="CSV of ROS domain artifact directories to create before runtime",
    )
    parser.add_argument("--verify-postrun", action="store_true")
    parser.add_argument("--attest-running-service", action="append", default=[])
    parser.add_argument("--verify-running-attestation", action="store_true")
    parser.add_argument("--seal-running-attestation", action="store_true")
    parser.add_argument("--resolve-attested-service", default="")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        modes = sum(bool(value) for value in (
            args.verify_postrun, args.attest_running_service,
            args.verify_running_attestation, args.seal_running_attestation,
            args.resolve_attested_service,
        ))
        if modes > 1:
            raise FingerprintError("fingerprint operation modes are exclusive")
        if args.attest_running_service:
            path = attest_running_service(
                args.repo_root, args.output_root, args.run_dir,
                args.attest_running_service, os.environ,
            )
            print(path)
            return 0
        if args.verify_running_attestation or args.seal_running_attestation:
            path = verify_running_attestation(
                args.repo_root, args.output_root, args.run_dir,
                seal=args.seal_running_attestation,
            )
            print(path)
            return 0
        if args.resolve_attested_service:
            print(resolve_attested_service(
                args.repo_root, args.output_root, args.run_dir,
                args.resolve_attested_service,
            ))
            return 0
        if args.verify_postrun:
            path, verified = verify_postrun(args.repo_root, args.output_root, args.run_dir)
            print(f"run fingerprint verification: {path} verified={str(verified).lower()}")
            return 0 if verified else 2
        path = capture_prelaunch(args.repo_root, args.output_root, args.run_dir, os.environ)
        if args.prepare_domains:
            prepare_domain_artifact_dirs(args.run_dir, args.prepare_domains)
        print(f"run fingerprint captured: {path}")
        return 0
    except (FingerprintError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"run fingerprint failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
