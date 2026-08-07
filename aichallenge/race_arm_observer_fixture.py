#!/usr/bin/env python3
"""Deterministic process-backed race-arm observer fixture for Helper tests."""

from __future__ import annotations

import argparse
import json
import os
import signal
import time
from pathlib import Path


def atomic_write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, separators=(",", ":")) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--token", required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    parser.add_argument("--ready", type=Path, required=True)
    parser.add_argument("--topic")
    parser.add_argument("--domain-id", type=int)
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--expected-pid", type=int)
    parser.add_argument("--expected-domain", type=int)
    parser.add_argument("--min-started-monotonic-ns", type=int)
    parser.add_argument("--history-watermark-seq", type=int)
    parser.add_argument("--forbid-state-after-watermark")
    args = parser.parse_args()

    domain_id = args.expected_domain if args.snapshot else args.domain_id
    if domain_id is None:
        return 2
    calls = Path(os.environ["FAKE_ROS_CALLS"])
    with calls.open("a", encoding="utf-8") as stream:
        stream.write(f"{domain_id}|race arm observer snapshot={int(args.snapshot)}\n")

    if args.snapshot:
        lifecycle = json.loads(args.ready.read_text(encoding="utf-8"))
        if (
            lifecycle.get("status") != "ready"
            or lifecycle.get("token") != args.token
            or lifecycle.get("domain_id") != domain_id
            or lifecycle.get("pid") != args.expected_pid
        ):
            return 1
        pid_dir = Path(os.environ["FAKE_RACE_OBSERVER_PID_DIR"])
        snapshot_count_path = pid_dir / f"{domain_id}.snapshot-count"
        snapshot_count = int(snapshot_count_path.read_text(encoding="utf-8")) + 1
        snapshot_count_path.write_text(str(snapshot_count), encoding="utf-8")
        startup_behavior = os.environ.get("FAKE_RACE_ARM_STARTUP_BEHAVIOR", "stable")
        if snapshot_count == 2:
            if startup_behavior == "observer_exit":
                os.kill(args.expected_pid, signal.SIGKILL)
                return 1
            if startup_behavior == "malformed":
                print("malformed")
                return 0
            if startup_behavior == "missing":
                args.jsonl.unlink(missing_ok=True)
                return 1
        records = args.jsonl.read_text(encoding="utf-8").splitlines()
        if not records:
            return 1
        record = json.loads(records[-1])
        generation = int(
            Path(os.environ["FAKE_RACE_ARM_EVENT_GENERATION_PATH"]).read_text(
                encoding="utf-8"
            )
        )
        delayed_states: list[str] = []
        if snapshot_count == 2 and startup_behavior == "delayed_false":
            delayed_states = ["false"]
        elif snapshot_count == 2 and startup_behavior == "delayed_true":
            delayed_states = ["true"]
        elif snapshot_count == 2 and startup_behavior == "burst_true_false":
            delayed_states = ["true", "false"]
        post_pulse_marker = pid_dir / f"{domain_id}.post-pulse-burst-recorded"
        if (
            startup_behavior == "post_pulse_burst_true_false"
            and int(
                Path(os.environ["FAKE_START_PUBLISH_COUNT"]).read_text(
                    encoding="utf-8"
                )
            )
            >= 1
            and not post_pulse_marker.exists()
        ):
            delayed_states = ["true", "false"]
            post_pulse_marker.write_text("recorded", encoding="utf-8")
        for delayed_state in delayed_states:
            generation = max(generation, int(record["fixture_generation"])) + 1
            Path(os.environ["FAKE_RACE_ARM_EVENT_GENERATION_PATH"]).write_text(
                str(generation), encoding="utf-8"
            )
            Path(os.environ["FAKE_RACE_ARM_STATE_PATH"]).write_text(
                delayed_state, encoding="utf-8"
            )
            record = {
                "domain_id": domain_id,
                "fixture_generation": generation,
                "monotonic_ns": time.monotonic_ns(),
                "seq": int(record["seq"]) + 1,
                "state": delayed_state,
                "token": args.token,
            }
            with args.jsonl.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, separators=(",", ":")) + "\n")
        if not delayed_states and generation > int(record["fixture_generation"]):
            record = {
                "domain_id": domain_id,
                "fixture_generation": generation,
                "monotonic_ns": time.monotonic_ns(),
                "seq": int(record["seq"]) + 1,
                "state": Path(os.environ["FAKE_RACE_ARM_STATE_PATH"]).read_text(
                    encoding="utf-8"
                ),
                "token": args.token,
            }
            with args.jsonl.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, separators=(",", ":")) + "\n")
        seen_true = False
        saw_forbidden = False
        if args.history_watermark_seq is not None:
            for encoded_record in args.jsonl.read_text(encoding="utf-8").splitlines():
                history_record = json.loads(encoded_record)
                if int(history_record["seq"]) > args.history_watermark_seq:
                    seen_true = seen_true or history_record["state"] == "true"
                    saw_forbidden = saw_forbidden or (
                        history_record["state"] == args.forbid_state_after_watermark
                    )
        print(
            record["seq"],
            record["state"],
            record["monotonic_ns"],
            str(seen_true).lower(),
            str(saw_forbidden).lower(),
        )
        return 0

    started_monotonic_ns = time.monotonic_ns()
    pid_dir = Path(os.environ["FAKE_RACE_OBSERVER_PID_DIR"])
    (pid_dir / f"{domain_id}.pid").write_text(str(os.getpid()), encoding="utf-8")
    (pid_dir / f"{domain_id}.snapshot-count").write_text("0", encoding="utf-8")
    with Path(os.environ["FAKE_OBSERVER_DIRS"]).open("a", encoding="utf-8") as stream:
        stream.write(str(args.ready.parent) + "\n")
    initial_record = {
        "domain_id": domain_id,
        "fixture_generation": int(
            Path(os.environ["FAKE_RACE_ARM_EVENT_GENERATION_PATH"]).read_text(
                encoding="utf-8"
            )
        ),
        "monotonic_ns": started_monotonic_ns,
        "seq": 1,
        "state": (
            os.environ.get("FAKE_RACE_ARM_DOMAIN_2_INITIAL_STATE", "")
            if domain_id == 2
            else ""
        )
        or Path(os.environ["FAKE_RACE_ARM_STATE_PATH"]).read_text(encoding="utf-8"),
        "token": args.token,
    }
    args.jsonl.write_text(
        json.dumps(initial_record, separators=(",", ":")) + "\n", encoding="utf-8"
    )

    def stop(*_unused: object) -> None:
        atomic_write(
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
    atomic_write(
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


if __name__ == "__main__":
    raise SystemExit(main())
