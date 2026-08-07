#!/usr/bin/env python3
"""Deterministic process-backed vehicle readiness observer fixture."""

from __future__ import annotations

import argparse
import json
import os
import signal
import time
from pathlib import Path


def write_lifecycle(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, separators=(",", ":")) + "\n", encoding="utf-8")


def append_event(path: Path, domain_id: int, token: str, kind: str, value: str) -> dict:
    lines = path.read_text(encoding="utf-8").splitlines() if path.exists() else []
    sequence = len(lines) + 1
    record = {
        "domain_id": domain_id,
        "kind": kind,
        "monotonic_ns": time.monotonic_ns(),
        "seq": sequence,
        "token": token,
        "value": value,
    }
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, separators=(",", ":")) + "\n")
    return record


def latest_values(path: Path) -> tuple[str, int, str, int, int]:
    vehicle_state = "empty"
    vehicle_seq = 0
    initialization = "empty"
    initialization_seq = 0
    monotonic_ns = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        monotonic_ns = int(record["monotonic_ns"])
        if record["kind"] == "vehicle":
            vehicle_state = str(record["value"])
            vehicle_seq = int(record["seq"])
        elif record["kind"] == "initialization":
            initialization = str(record["value"])
            initialization_seq = int(record["seq"])
    return vehicle_state, vehicle_seq, initialization, initialization_seq, monotonic_ns


def desired_vehicle_state(domain_id: int) -> str:
    reset_marker = Path(os.environ["FAKE_RESET_OCCURRED"])
    if reset_marker.exists():
        return os.environ["FAKE_VEHICLE_STATE"].lower()
    behavior = os.environ["FAKE_START_BEHAVIOR"]
    publish_count = int(
        Path(os.environ["FAKE_START_PUBLISH_COUNT"]).read_text(encoding="utf-8")
    )
    post_start_states = {
        "start_then_ready_after_publish": "ready",
        "start_then_grounded_after_publish": "grounded",
        "start_then_finish_after_publish": "finish",
    }
    if behavior in post_start_states and publish_count >= 1:
        return post_start_states[behavior]
    admin_state = Path(os.environ["FAKE_ADMIN_STATE"]).read_text(encoding="utf-8")
    if Path(os.environ["FAKE_READY_DURING_PREFLIGHT"]).exists() and admin_state != "start":
        return "ready"
    if behavior in {"two_stage", "two_stage_lapcomplete_pre_final"} and publish_count == 1:
        return "ready"
    if behavior == "two_stage_no_final_start" and publish_count >= 1:
        return "ready"
    domain_behaviors = {
        "domain_start_admin_playstart",
        "domain_start_admin_waitstart",
        "domain_start_admin_waitstart_then_start",
        "domain_start_admin_waitstart_then_start_regress",
        "domain_start_admin_waitstart_pub_fail",
        "persistent_post_watermark_start",
    }
    if behavior in domain_behaviors and publish_count >= 1:
        if behavior == "domain_start_admin_waitstart_then_start_regress" and domain_id == 2 and admin_state == "start":
            return "grounded"
        domain_two = os.environ.get("FAKE_DOMAIN_2_VEHICLE_STATE_AFTER_START", "")
        if domain_id == 2 and domain_two:
            return domain_two.lower()
        return os.environ["FAKE_VEHICLE_STATE_AFTER_START"].lower()
    if admin_state == "start":
        return os.environ["FAKE_VEHICLE_STATE_AFTER_START"].lower()
    return os.environ["FAKE_VEHICLE_STATE"].lower()


def desired_initialization() -> str | None:
    if Path(os.environ["FAKE_RESET_OCCURRED"]).exists():
        return "true"
    if Path(os.environ["FAKE_POST_ARM_INITIALIZATION_FALSE"]).exists():
        return "false"
    publish_count = int(Path(os.environ["FAKE_START_PUBLISH_COUNT"]).read_text(encoding="utf-8"))
    admin_state = Path(os.environ["FAKE_ADMIN_STATE"]).read_text(encoding="utf-8")
    if publish_count >= 1 or admin_state == "start":
        value = os.environ["FAKE_INITIALIZATION_READY_AFTER_START"].lower()
        return None if value == "none" else value
    return "true"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--token", required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    parser.add_argument("--ready", type=Path, required=True)
    parser.add_argument("--vehicle-topic")
    parser.add_argument("--initialization-topic")
    parser.add_argument("--domain-id", type=int)
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--expected-pid", type=int)
    parser.add_argument("--expected-domain", type=int)
    parser.add_argument("--min-started-monotonic-ns", type=int)
    parser.add_argument("--history-watermark-seq", type=int)
    parser.add_argument("--initialization-watermark-seq", type=int)
    args = parser.parse_args()
    domain_id = args.expected_domain if args.snapshot else args.domain_id
    if domain_id is None:
        return 2

    calls = Path(os.environ["FAKE_ROS_CALLS"])
    with calls.open("a", encoding="utf-8") as stream:
        stream.write(f"{domain_id}|vehicle readiness observer snapshot={int(args.snapshot)}\n")

    if not args.snapshot:
        started_monotonic_ns = time.monotonic_ns()
        pid_dir = Path(os.environ["FAKE_VEHICLE_OBSERVER_PID_DIR"])
        (pid_dir / f"{domain_id}.pid").write_text(str(os.getpid()), encoding="utf-8")
        with Path(os.environ["FAKE_OBSERVER_DIRS"]).open("a", encoding="utf-8") as stream:
            stream.write(str(args.ready.parent) + "\n")
        args.jsonl.write_text("", encoding="utf-8")
        append_event(args.jsonl, domain_id, args.token, "vehicle", desired_vehicle_state(domain_id))
        append_event(args.jsonl, domain_id, args.token, "initialization", "true")

        def stop(*_unused: object) -> None:
            write_lifecycle(
                args.ready,
                {
                    "domain_id": domain_id,
                    "pid": os.getpid(),
                    "started_monotonic_ns": started_monotonic_ns,
                    "status": "stopped",
                    "stopped_monotonic_ns": time.monotonic_ns(),
                    "token": args.token,
                },
            )
            raise SystemExit(0)

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        write_lifecycle(
            args.ready,
            {
                "domain_id": domain_id,
                "pid": os.getpid(),
                "started_monotonic_ns": started_monotonic_ns,
                "status": "ready",
                "token": args.token,
            },
        )
        while True:
            time.sleep(0.05)

    lifecycle = json.loads(args.ready.read_text(encoding="utf-8"))
    if (
        lifecycle.get("domain_id") != domain_id
        or lifecycle.get("pid") != args.expected_pid
        or lifecycle.get("status") != "ready"
        or lifecycle.get("token") != args.token
    ):
        return 1
    publish_count = int(Path(os.environ["FAKE_START_PUBLISH_COUNT"]).read_text(encoding="utf-8"))
    behavior = os.environ["FAKE_START_BEHAVIOR"]
    if behavior == "vehicle_observer_exit_after_pulse" and publish_count >= 1:
        os.kill(args.expected_pid, signal.SIGKILL)
        return 1
    if behavior == "vehicle_observer_malformed_after_pulse" and publish_count >= 1:
        print("malformed")
        return 0

    vehicle_state, _, initialization, _, _ = latest_values(args.jsonl)
    post_start_states = {
        "start_then_ready_after_publish": "ready",
        "start_then_grounded_after_publish": "grounded",
        "start_then_finish_after_publish": "finish",
    }
    if behavior in post_start_states and publish_count >= 1:
        has_start = any(
            json.loads(line).get("kind") == "vehicle"
            and json.loads(line).get("value") == "start"
            for line in args.jsonl.read_text(encoding="utf-8").splitlines()
        )
        if not has_start:
            append_event(args.jsonl, domain_id, args.token, "vehicle", "start")
            append_event(
                args.jsonl,
                domain_id,
                args.token,
                "vehicle",
                post_start_states[behavior],
            )
            vehicle_state, _, initialization, _, _ = latest_values(args.jsonl)
    desired_vehicle = desired_vehicle_state(domain_id)
    if desired_vehicle != vehicle_state:
        if desired_vehicle == "start" and publish_count >= 1 and vehicle_state != "ready":
            append_event(args.jsonl, domain_id, args.token, "vehicle", "ready")
        append_event(args.jsonl, domain_id, args.token, "vehicle", desired_vehicle)
    desired_init = desired_initialization()
    if desired_init is not None and desired_init != initialization:
        append_event(args.jsonl, domain_id, args.token, "initialization", desired_init)

    lines = args.jsonl.read_text(encoding="utf-8").splitlines()
    vehicle_state, vehicle_seq, initialization, initialization_seq, monotonic_ns = latest_values(args.jsonl)
    saw_start = False
    saw_invalid = False
    saw_false = False
    if args.history_watermark_seq is not None:
        for line in lines:
            record = json.loads(line)
            if int(record["seq"]) > args.history_watermark_seq:
                saw_start = saw_start or (
                    record["kind"] == "vehicle" and record["value"] == "start"
                )
                saw_invalid = saw_invalid or (
                    record["kind"] == "vehicle"
                    and record["value"] not in {"ready", "start"}
                )
            if int(record["seq"]) > int(args.initialization_watermark_seq or 0):
                saw_false = saw_false or (
                    record["kind"] == "initialization" and record["value"] == "false"
                )
    print(
        len(lines),
        vehicle_state,
        vehicle_seq,
        initialization,
        initialization_seq,
        monotonic_ns,
        str(saw_start).lower(),
        str(saw_invalid).lower(),
        str(saw_false).lower(),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
