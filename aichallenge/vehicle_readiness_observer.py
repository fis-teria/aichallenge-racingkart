#!/usr/bin/env python3
"""Persistent typed observer for vehicle state and initialization readiness."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import sys
import time
from pathlib import Path


MAX_TOTAL_BYTES = 131072
MAX_RECORDS = 512
MAX_RECORD_BYTES = 512
VALID_VEHICLE_STATES = {
    "spawned",
    "grounded",
    "ready",
    "start",
    "lapcomplete",
    "finish",
    "finished",
    "finishall",
    "finishedall",
    "terminate",
    "terminated",
}


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def normalize_vehicle_state(raw_state: object) -> str:
    if not isinstance(raw_state, str):
        raise ValueError("vehicle state must be a string")
    normalized = "".join(character for character in raw_state if character.isalnum()).lower()
    if normalized not in VALID_VEHICLE_STATES:
        raise ValueError(f"unsupported vehicle state: {raw_state!r}")
    return normalized


def normalize_initialization_ready(raw_value: object) -> str:
    if not isinstance(raw_value, bool):
        raise ValueError("initialization readiness must be a bool")
    return "true" if raw_value else "false"


class VehicleReadinessObserver:
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

    def record(self, kind: str, raw_value: object) -> None:
        if kind == "vehicle":
            value = normalize_vehicle_state(raw_value)
        elif kind == "initialization":
            value = normalize_initialization_ready(raw_value)
        else:
            raise ValueError(f"unsupported readiness event kind: {kind}")
        self._sequence += 1
        record = {
            "domain_id": self._domain_id,
            "kind": kind,
            "monotonic_ns": time.monotonic_ns(),
            "seq": self._sequence,
            "token": self._token,
            "value": value,
        }
        encoded = (
            json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n"
        ).encode("utf-8")
        if len(encoded) > MAX_RECORD_BYTES:
            raise ValueError("vehicle readiness observer record exceeds bounded size")
        if os.write(self._jsonl_fd, encoded) != len(encoded):
            raise OSError("short JSONL write")

    def run(self, vehicle_topic: str, initialization_topic: str) -> int:
        try:
            import rclpy
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
            )
            from std_msgs.msg import Bool, String

            rclpy.init(args=None)
            node = rclpy.create_node(f"vehicle_readiness_observer_d{self._domain_id}")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

            def fail(exc: Exception) -> None:
                atomic_json(
                    self._lifecycle_path,
                    {
                        "domain_id": self._domain_id,
                        "error": str(exc),
                        "status": "error",
                        "token": self._token,
                    },
                )
                self._failed = True
                self._stopping = True

            def vehicle_callback(message: String) -> None:
                try:
                    self.record("vehicle", message.data)
                except Exception as exc:  # noqa: BLE001
                    fail(exc)

            def initialization_callback(message: Bool) -> None:
                try:
                    self.record("initialization", message.data)
                except Exception as exc:  # noqa: BLE001
                    fail(exc)

            node.create_subscription(String, vehicle_topic, vehicle_callback, qos)
            node.create_subscription(Bool, initialization_topic, initialization_callback, qos)
            atomic_json(
                self._lifecycle_path,
                {
                    "domain_id": self._domain_id,
                    "pid": os.getpid(),
                    "started_monotonic_ns": self._started_monotonic_ns,
                    "status": "ready",
                    "token": self._token,
                },
            )
            while rclpy.ok() and not self._stopping:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
            rclpy.shutdown()
            return 0 if not self._stopping else 1
        except Exception as exc:  # noqa: BLE001
            atomic_json(
                self._lifecycle_path,
                {
                    "domain_id": self._domain_id,
                    "error": str(exc),
                    "status": "error",
                    "token": self._token,
                },
            )
            return 1
        finally:
            if self._stopping and not self._failed:
                atomic_json(
                    self._lifecycle_path,
                    {
                        "domain_id": self._domain_id,
                        "pid": os.getpid(),
                        "started_monotonic_ns": self._started_monotonic_ns,
                        "status": "stopped",
                        "stopped_monotonic_ns": time.monotonic_ns(),
                        "token": self._token,
                    },
                )
            os.close(self._jsonl_fd)


def read_snapshot(args: argparse.Namespace) -> int:
    try:
        lifecycle = json.loads(args.ready.read_text(encoding="utf-8"))
        if (
            lifecycle.get("domain_id") != args.expected_domain
            or lifecycle.get("status") != "ready"
            or lifecycle.get("token") != args.token
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
        vehicle_state = "empty"
        vehicle_seq = 0
        initialization_ready = "empty"
        initialization_seq = 0
        saw_start_after_watermark = False
        saw_invalid_vehicle_after_watermark = False
        saw_initialization_false_after_watermark = False
        for record_count, line in enumerate(content.splitlines(), 1):
            if record_count > MAX_RECORDS or not line or len(line) > MAX_RECORD_BYTES:
                return 1
            record = json.loads(line)
            if (
                set(record)
                != {"domain_id", "kind", "monotonic_ns", "seq", "token", "value"}
                or record["domain_id"] != args.expected_domain
                or record["token"] != args.token
            ):
                return 1
            if not isinstance(record["seq"], int) or not isinstance(
                record["monotonic_ns"], int
            ):
                return 1
            if (
                record["seq"] != previous_seq + 1
                or record["monotonic_ns"] <= previous_monotonic_ns
            ):
                return 1
            previous_seq = record["seq"]
            previous_monotonic_ns = record["monotonic_ns"]
            if record["kind"] == "vehicle":
                if normalize_vehicle_state(record["value"]) != record["value"]:
                    return 1
                vehicle_state = record["value"]
                vehicle_seq = record["seq"]
                if (
                    args.history_watermark_seq is not None
                    and record["seq"] > args.history_watermark_seq
                ):
                    saw_start_after_watermark = (
                        saw_start_after_watermark or record["value"] == "start"
                    )
                    saw_invalid_vehicle_after_watermark = (
                        saw_invalid_vehicle_after_watermark
                        or record["value"] not in {"ready", "start"}
                    )
            elif record["kind"] == "initialization":
                normalized = record["value"]
                if normalized not in {"false", "true"}:
                    return 1
                initialization_ready = normalized
                initialization_seq = record["seq"]
                if (
                    args.initialization_watermark_seq is not None
                    and record["seq"] > args.initialization_watermark_seq
                    and normalized == "false"
                ):
                    saw_initialization_false_after_watermark = True
            else:
                return 1

        if args.history_watermark_seq is not None:
            if args.history_watermark_seq < 0 or args.history_watermark_seq > previous_seq:
                return 1
        if args.initialization_watermark_seq is not None:
            if (
                args.initialization_watermark_seq < 1
                or args.initialization_watermark_seq > previous_seq
            ):
                return 1
        if (args.history_watermark_seq is None) != (
            args.initialization_watermark_seq is None
        ):
            return 1

        print(
            f"{previous_seq} {vehicle_state} {vehicle_seq} "
            f"{initialization_ready} {initialization_seq} {previous_monotonic_ns} "
            f"{str(saw_start_after_watermark).lower()} "
            f"{str(saw_invalid_vehicle_after_watermark).lower()} "
            f"{str(saw_initialization_false_after_watermark).lower()}"
        )
        return 0
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--token", required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    parser.add_argument("--ready", type=Path, required=True)
    parser.add_argument("--vehicle-topic", default="/awsim/state")
    parser.add_argument(
        "--initialization-topic", default="/autostart/initialization_ready"
    )
    parser.add_argument("--domain-id", type=int)
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--expected-pid", type=int)
    parser.add_argument("--expected-domain", type=int)
    parser.add_argument("--min-started-monotonic-ns", type=int)
    parser.add_argument("--history-watermark-seq", type=int)
    parser.add_argument("--initialization-watermark-seq", type=int)
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]{16,128}", args.token):
        return 2
    if args.snapshot:
        if args.expected_domain is None or args.expected_domain <= 0:
            return 2
        return read_snapshot(args)
    if args.domain_id is None or args.domain_id <= 0:
        return 2
    observer = VehicleReadinessObserver(
        token=args.token,
        domain_id=args.domain_id,
        jsonl_path=args.jsonl,
        lifecycle_path=args.ready,
    )
    signal.signal(signal.SIGINT, observer.stop)
    signal.signal(signal.SIGTERM, observer.stop)
    return observer.run(args.vehicle_topic, args.initialization_topic)


if __name__ == "__main__":
    sys.exit(main())
