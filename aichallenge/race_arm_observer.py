#!/usr/bin/env python3
"""Bounded, read-only persistent observer for a vehicle-domain race-arm topic."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import sys
import time
from pathlib import Path


MAX_TOTAL_BYTES = 65536
MAX_RECORDS = 256
MAX_RECORD_BYTES = 512
VALID_STATES = {"false", "true"}


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def normalize_state(raw_state: object) -> str:
    if isinstance(raw_state, bool):
        return "true" if raw_state else "false"
    if isinstance(raw_state, str):
        normalized = raw_state.strip().lower()
        if normalized in VALID_STATES:
            return normalized
    raise ValueError("race-arm state must be exactly true or false")


class RaceArmObserver:
    def __init__(
        self, *, token: str, domain_id: int, jsonl_path: Path, lifecycle_path: Path
    ) -> None:
        self._token = token
        self._domain_id = domain_id
        self._jsonl_fd = os.open(
            jsonl_path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600
        )
        self._lifecycle_path = lifecycle_path
        self._sequence = 0
        self._stopping = False
        self._failed = False
        self._started_monotonic_ns = time.monotonic_ns()

    def stop(self, *_unused: object) -> None:
        self._stopping = True

    def record(self, raw_state: object) -> None:
        state = normalize_state(raw_state)
        self._sequence += 1
        record = {
            "domain_id": self._domain_id,
            "token": self._token,
            "seq": self._sequence,
            "monotonic_ns": time.monotonic_ns(),
            "state": state,
        }
        encoded = (
            json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n"
        ).encode("utf-8")
        if len(encoded) > MAX_RECORD_BYTES:
            raise ValueError("race-arm observer record exceeds bounded size")
        if os.write(self._jsonl_fd, encoded) != len(encoded):
            raise OSError("short JSONL write")

    def run(self, topic: str) -> int:
        try:
            import rclpy
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from std_msgs.msg import Bool

            rclpy.init(args=None)
            node = rclpy.create_node(f"race_arm_observer_d{self._domain_id}")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            def callback(message: Bool) -> None:
                try:
                    self.record(message.data)
                except Exception as exc:
                    atomic_json(
                        self._lifecycle_path,
                        {
                            "token": self._token,
                            "domain_id": self._domain_id,
                            "status": "error",
                            "error": str(exc),
                        },
                    )
                    self._failed = True
                    self._stopping = True

            node.create_subscription(Bool, topic, callback, qos)
            atomic_json(
                self._lifecycle_path,
                {
                    "token": self._token,
                    "domain_id": self._domain_id,
                    "status": "ready",
                    "pid": os.getpid(),
                    "started_monotonic_ns": self._started_monotonic_ns,
                },
            )
            while rclpy.ok() and not self._stopping:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
            rclpy.shutdown()
            return 0 if not self._stopping else 1
        except Exception as exc:
            atomic_json(
                self._lifecycle_path,
                {
                    "token": self._token,
                    "domain_id": self._domain_id,
                    "status": "error",
                    "error": str(exc),
                },
            )
            return 1
        finally:
            if self._stopping and not self._failed:
                atomic_json(
                    self._lifecycle_path,
                    {
                        "token": self._token,
                        "domain_id": self._domain_id,
                        "status": "stopped",
                        "pid": os.getpid(),
                        "started_monotonic_ns": self._started_monotonic_ns,
                        "stopped_monotonic_ns": time.monotonic_ns(),
                    },
                )
            os.close(self._jsonl_fd)


def read_snapshot(args: argparse.Namespace) -> int:
    try:
        lifecycle = json.loads(args.ready.read_text(encoding="utf-8"))
        if (
            lifecycle.get("token") != args.token
            or lifecycle.get("domain_id") != args.expected_domain
            or lifecycle.get("status") != "ready"
        ):
            return 1
        if args.expected_pid is not None and lifecycle.get("pid") != args.expected_pid:
            return 1
        started_monotonic_ns = lifecycle.get("started_monotonic_ns")
        if not isinstance(started_monotonic_ns, int):
            return 1
        if (
            args.min_started_monotonic_ns is not None
            and started_monotonic_ns < args.min_started_monotonic_ns
        ):
            return 1

        content = args.jsonl.read_bytes() if args.jsonl.exists() else b""
        if len(content) > MAX_TOTAL_BYTES or (content and not content.endswith(b"\n")):
            return 1

        previous_seq = 0
        previous_monotonic_ns = 0
        last_record: dict | None = None
        saw_required_state = False
        seen_true_after_history_watermark = False
        saw_forbidden_state = False
        for record_count, line in enumerate(content.splitlines(), 1):
            if record_count > MAX_RECORDS or not line or len(line) > MAX_RECORD_BYTES:
                return 1
            record = json.loads(line)
            if (
                set(record)
                != {"domain_id", "monotonic_ns", "seq", "state", "token"}
                or record["token"] != args.token
                or record["domain_id"] != args.expected_domain
            ):
                return 1
            if not isinstance(record["seq"], int) or not isinstance(
                record["monotonic_ns"], int
            ):
                return 1
            if (
                record["seq"] != previous_seq + 1
                or record["monotonic_ns"] <= previous_monotonic_ns
                or normalize_state(record["state"]) != record["state"]
            ):
                return 1
            previous_seq = record["seq"]
            previous_monotonic_ns = record["monotonic_ns"]
            last_record = record
            if (
                args.watermark_seq is not None
                and record["seq"] > args.watermark_seq
                and record["state"] == args.require_state_after_watermark
            ):
                saw_required_state = True
            if (
                args.history_watermark_seq is not None
                and record["seq"] > args.history_watermark_seq
            ):
                if record["state"] == "true":
                    seen_true_after_history_watermark = True
                if record["state"] == args.forbid_state_after_watermark:
                    saw_forbidden_state = True

        if args.watermark_seq is not None:
            if args.watermark_seq < 1 or args.require_state_after_watermark is None:
                return 1
            if not saw_required_state:
                return 1
        elif args.require_state_after_watermark is not None:
            return 1
        if args.history_watermark_seq is not None:
            if (
                args.history_watermark_seq < 0
                or args.history_watermark_seq > previous_seq
                or args.forbid_state_after_watermark is None
            ):
                return 1
        elif args.forbid_state_after_watermark is not None:
            return 1

        if last_record is None:
            print("0 empty 0 false false")
        else:
            print(
                f"{last_record['seq']} {last_record['state']} "
                f"{last_record['monotonic_ns']} "
                f"{str(seen_true_after_history_watermark).lower()} "
                f"{str(saw_forbidden_state).lower()}"
            )
        return 0
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--token", required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    parser.add_argument("--ready", type=Path, required=True)
    parser.add_argument("--topic", default="/overtake/race_armed")
    parser.add_argument("--domain-id", type=int)
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--expected-pid", type=int)
    parser.add_argument("--expected-domain", type=int)
    parser.add_argument("--min-started-monotonic-ns", type=int)
    parser.add_argument("--watermark-seq", type=int)
    parser.add_argument("--require-state-after-watermark", choices=sorted(VALID_STATES))
    parser.add_argument("--history-watermark-seq", type=int)
    parser.add_argument("--forbid-state-after-watermark", choices=sorted(VALID_STATES))
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]{16,128}", args.token):
        return 2
    if args.snapshot:
        if args.expected_domain is None or args.expected_domain <= 0:
            return 2
        return read_snapshot(args)
    if args.domain_id is None or args.domain_id <= 0:
        return 2
    observer = RaceArmObserver(
        token=args.token,
        domain_id=args.domain_id,
        jsonl_path=args.jsonl,
        lifecycle_path=args.ready,
    )
    signal.signal(signal.SIGINT, observer.stop)
    signal.signal(signal.SIGTERM, observer.stop)
    return observer.run(args.topic)


if __name__ == "__main__":
    sys.exit(main())
