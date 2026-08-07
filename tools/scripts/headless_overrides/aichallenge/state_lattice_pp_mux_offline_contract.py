#!/usr/bin/env python3
"""Static/result contract for bounded private Planner -> PP -> Mux bag replay."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Any


PRIVATE_ROOT_PREFIX = "/test_only/offline_replay/"
SOURCE_TOPICS = {
    "/clock": ("rosgraph_msgs/msg/Clock", True),
    "/tf": ("tf2_msgs/msg/TFMessage", True),
    "/tf_static": ("tf2_msgs/msg/TFMessage", True),
    "/localization/kinematic_state": ("nav_msgs/msg/Odometry", True),
    "/v2x/vehicle_positions": (
        "v2x_msgs/msg/V2XVehiclePositionArray",
        True,
    ),
    "/planning/scenario_planning/trajectory": (
        "autoware_auto_planning_msgs/msg/Trajectory",
        True,
    ),
    "/overtake/race_armed": ("std_msgs/msg/Bool", True),
    "/awsim/state": ("std_msgs/msg/String", True),
    "/vehicle/status/steering_status": (
        "autoware_auto_vehicle_msgs/msg/SteeringReport",
        True,
    ),
    # POC29 records this topic with zero messages. Presence and type are
    # authoritative; replay must not invent health samples.
    "/mpc/speed_profile_debug": ("std_msgs/msg/String", False),
}


def validate_private_root(value: str) -> str:
    token = value.removeprefix(PRIVATE_ROOT_PREFIX)
    if not (
        value.startswith(PRIVATE_ROOT_PREFIX)
        and token
        and "/" not in token
        and all(character.isalnum() or character in "_-" for character in token)
    ):
        raise ValueError("invalid tokenized private root")
    return value.rstrip("/")


def parse_rosbag_metadata(path: Path) -> dict[str, dict[str, Any]]:
    """Parse the stable topic/type/count subset of rosbag2 metadata YAML."""
    topics: dict[str, dict[str, Any]] = {}
    current_name: str | None = None
    current_type: str | None = None
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        if stripped.startswith("name: "):
            current_name = stripped.removeprefix("name: ").strip()
            current_type = None
        elif current_name is not None and stripped.startswith("type: "):
            current_type = stripped.removeprefix("type: ").strip()
        elif (
            current_name is not None
            and current_type is not None
            and re.fullmatch(r"message_count: [0-9]+", stripped)
        ):
            if current_name in topics:
                raise ValueError(f"duplicate topic metadata: {current_name}")
            topics[current_name] = {
                "type": current_type,
                "message_count": int(stripped.split()[1]),
            }
            current_name = None
            current_type = None
    if not topics:
        raise ValueError("rosbag metadata contains no topics")
    return topics


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, delete=False
    ) as stream:
        temporary = Path(stream.name)
        os.fchmod(stream.fileno(), 0o644)
        json.dump(payload, stream, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def parse_named_paths(values: list[str]) -> dict[str, Path]:
    paths: dict[str, Path] = {}
    for value in values:
        name, separator, encoded_path = value.partition("=")
        if (
            not separator
            or not re.fullmatch(r"[A-Za-z0-9_.-]+", name)
            or not encoded_path
            or name in paths
        ):
            raise ValueError(f"invalid named path: {value}")
        paths[name] = Path(encoded_path)
    return paths


def validate_source(
    metadata_path: Path, mcap_path: Path, config_paths: dict[str, Path]
) -> dict[str, Any]:
    topics = parse_rosbag_metadata(metadata_path)
    errors: list[str] = []
    for topic, (expected_type, require_messages) in SOURCE_TOPICS.items():
        record = topics.get(topic)
        if record is None:
            errors.append(f"missing_source_topic:{topic}")
            continue
        if record["type"] != expected_type:
            errors.append(f"source_type_mismatch:{topic}")
        if require_messages and record["message_count"] <= 0:
            errors.append(f"source_topic_empty:{topic}")
    if not mcap_path.is_file() or mcap_path.stat().st_size <= 0:
        errors.append("source_mcap_missing_or_empty")
    hashes = {
        "metadata_sha256": sha256_file(metadata_path),
        "mcap_sha256": sha256_file(mcap_path) if mcap_path.is_file() else "",
        "configs": {
            name: {
                "path": str(path),
                "sha256": sha256_file(path),
            }
            for name, path in sorted(config_paths.items())
            if path.is_file()
        },
    }
    missing_configs = sorted(
        name for name, path in config_paths.items() if not path.is_file()
    )
    errors.extend(f"missing_config:{name}" for name in missing_configs)
    return {
        "source_topics": {
            name: topics.get(name) for name in sorted(SOURCE_TOPICS)
        },
        "source_hashes": hashes,
        "errors": errors,
    }


def output_topic_names(root: str) -> dict[str, str]:
    root = validate_private_root(root)
    return {
        "planner_reference_override": f"{root}/planner/reference_override",
        "planner_plan": f"{root}/planner/plan",
        "planner_safety_constraint": f"{root}/planner/safety_constraint",
        "pp_command_envelope": f"{root}/pp/command_envelope",
        "pp_execution_envelope": f"{root}/pp/execution_envelope",
        "pp_tracking_status": f"{root}/pp/tracking_status",
        "pp_execution_ack": f"{root}/pp/free_run_execution_ack",
        "mux_motion_authority_grant": f"{root}/mux/motion_authority_grant",
        "mux_final_control": f"{root}/output/control_cmd",
    }


def validate_output(
    metadata_path: Path, private_root: str, mode: str
) -> dict[str, Any]:
    topics = parse_rosbag_metadata(metadata_path)
    names = output_topic_names(private_root)
    counts = {
        label: int(topics.get(topic, {}).get("message_count", 0))
        for label, topic in names.items()
    }
    errors: list[str] = []
    if counts["planner_reference_override"] <= 0:
        errors.append("planner_reference_override_absent")
    if mode == "regenerated":
        for label in (
            "planner_plan",
            "planner_safety_constraint",
            "pp_command_envelope",
            "pp_execution_envelope",
            "pp_tracking_status",
            "mux_final_control",
        ):
            if counts[label] <= 0:
                errors.append(f"required_regenerated_output_absent:{label}")
    return {
        "output_topic_counts": counts,
        "output_topics": names,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("preflight", "final"), required=True)
    parser.add_argument("--mode", choices=("baseline", "regenerated"), required=True)
    parser.add_argument("--private-root", required=True)
    parser.add_argument("--source-metadata", type=Path, required=True)
    parser.add_argument("--source-mcap", type=Path, required=True)
    parser.add_argument("--preflight-result", type=Path)
    parser.add_argument("--output-metadata", type=Path)
    parser.add_argument("--config", action="append", default=[])
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()

    try:
        private_root = validate_private_root(args.private_root)
        config_paths = parse_named_paths(args.config)
        source = validate_source(
            args.source_metadata, args.source_mcap, config_paths
        )
        errors = list(source["errors"])
        output: dict[str, Any] = {}
        if args.phase == "final":
            if args.preflight_result is None or args.output_metadata is None:
                raise ValueError("final phase requires preflight and output metadata")
            preflight = json.loads(
                args.preflight_result.read_text(encoding="utf-8")
            )
            if preflight.get("source_hashes") != source["source_hashes"]:
                errors.append("immutable_source_or_config_hash_changed")
            output = validate_output(
                args.output_metadata, private_root, args.mode
            )
            errors.extend(output["errors"])
        payload = {
            "schema_version": 1,
            "phase": args.phase,
            "mode": args.mode,
            "private_root": private_root,
            "verdict": "PASS" if not errors else "HOLD",
            "no_synthetic_authority": True,
            "official_authority_proven": False,
            "vehicle_control_proven": False,
            "claim": (
                "COUNTERFACTUAL_PRODUCTION_ALGORITHM_CANDIDATE"
                if args.phase == "final"
                and args.mode == "regenerated"
                and not errors
                else "TEST_ONLY_NO_AUTHORITY"
            ),
            **source,
            **output,
            "errors": errors,
        }
        atomic_json(args.result, payload)
        return 0 if not errors else 1
    except Exception as error:
        atomic_json(
            args.result,
            {
                "schema_version": 1,
                "phase": args.phase,
                "mode": args.mode,
                "private_root": args.private_root,
                "verdict": "HOLD",
                "no_synthetic_authority": True,
                "official_authority_proven": False,
                "vehicle_control_proven": False,
                "claim": "TEST_ONLY_NO_AUTHORITY",
                "errors": [f"contract_error:{type(error).__name__}:{error}"],
            },
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
