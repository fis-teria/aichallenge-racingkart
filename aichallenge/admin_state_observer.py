#!/usr/bin/env python3
"""Bounded, read-only persistent observer for AWSIM's admin state topic."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import sys
import time
from pathlib import Path


MAX_STATE_BYTES = 64
MAX_TOTAL_BYTES = 65536
MAX_RECORDS = 256
NORMALIZED_STATE = re.compile(r"^[a-z0-9]+$")


def atomic_json(path: Path, payload: dict) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def normalize_state(raw_state: str) -> str:
    normalized = raw_state.strip().lower()
    if not normalized or len(normalized.encode("utf-8")) > MAX_STATE_BYTES:
        raise ValueError("admin state is empty or oversized")
    if not NORMALIZED_STATE.fullmatch(normalized):
        raise ValueError("admin state is malformed")
    return normalized


class AdminStateObserver:
    def __init__(self, *, token: str, jsonl_path: Path, lifecycle_path: Path) -> None:
        self._token = token
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

    def record(self, raw_state: str) -> None:
        normalized = normalize_state(raw_state)
        self._sequence += 1
        record = {
            "token": self._token,
            "seq": self._sequence,
            "monotonic_ns": time.monotonic_ns(),
            "state": normalized,
        }
        encoded = (json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n").encode("utf-8")
        if len(encoded) > 512:
            raise ValueError("admin observer record exceeds bounded size")
        # One append syscall makes every accepted record newline-complete.
        if os.write(self._jsonl_fd, encoded) != len(encoded):
            raise OSError("short JSONL write")

    def run(self, topic: str) -> int:
        try:
            import rclpy
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String

            rclpy.init(args=None)
            node = rclpy.create_node("admin_state_observer")
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            def callback(message: String) -> None:
                try:
                    self.record(message.data)
                except Exception as exc:  # callback failure must stop evidence production.
                    atomic_json(self._lifecycle_path, {"token": self._token, "status": "error", "error": str(exc)})
                    self._failed = True
                    self._stopping = True

            node.create_subscription(String, topic, callback, qos)
            atomic_json(self._lifecycle_path, {"token": self._token, "status": "ready", "pid": os.getpid(), "started_monotonic_ns": self._started_monotonic_ns})
            while rclpy.ok() and not self._stopping:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
            rclpy.shutdown()
            return 0 if not self._stopping else 1
        except Exception as exc:
            atomic_json(self._lifecycle_path, {"token": self._token, "status": "error", "error": str(exc)})
            return 1
        finally:
            if self._stopping and not self._failed:
                atomic_json(self._lifecycle_path, {"token": self._token, "status": "stopped", "pid": os.getpid(), "started_monotonic_ns": self._started_monotonic_ns, "stopped_monotonic_ns": time.monotonic_ns()})
            os.close(self._jsonl_fd)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--token", required=True)
    parser.add_argument("--jsonl", type=Path, required=True)
    parser.add_argument("--ready", type=Path, required=True)
    parser.add_argument("--topic", default="/admin/awsim/state")
    parser.add_argument("--snapshot", action="store_true")
    parser.add_argument("--expected-pid", type=int)
    parser.add_argument("--min-started-monotonic-ns", type=int)
    parser.add_argument("--watermark-seq", type=int)
    parser.add_argument("--require-start-after-watermark", action="store_true")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9_-]{16,128}", args.token):
        return 2
    if args.snapshot:
        try:
            lifecycle = json.loads(args.ready.read_text(encoding="utf-8"))
            if lifecycle.get("token") != args.token or lifecycle.get("status") != "ready":
                return 1
            if args.expected_pid is not None and lifecycle.get("pid") != args.expected_pid:
                return 1
            if not isinstance(lifecycle.get("started_monotonic_ns"), int):
                return 1
            if args.min_started_monotonic_ns is not None and lifecycle["started_monotonic_ns"] < args.min_started_monotonic_ns:
                return 1
            content = args.jsonl.read_bytes() if args.jsonl.exists() else b""
            if len(content) > MAX_TOTAL_BYTES:
                return 1
            if content and not content.endswith(b"\n"):
                return 1
            previous_seq = 0
            previous_monotonic_ns = 0
            record_count = 0
            last_record: dict | None = None
            saw_start_after_watermark = False
            for line in content.splitlines():
                record_count += 1
                if record_count > MAX_RECORDS:
                    return 1
                if not line or len(line) > 512:
                    return 1
                record = json.loads(line)
                if set(record) != {"monotonic_ns", "seq", "state", "token"} or record["token"] != args.token:
                    return 1
                if not isinstance(record["seq"], int) or not isinstance(record["monotonic_ns"], int):
                    return 1
                if record["seq"] != previous_seq + 1 or record["monotonic_ns"] <= previous_monotonic_ns:
                    return 1
                if normalize_state(record["state"]) != record["state"]:
                    return 1
                previous_seq = record["seq"]
                previous_monotonic_ns = record["monotonic_ns"]
                last_record = record
                if args.watermark_seq is not None and record["seq"] > args.watermark_seq:
                    if record["state"] == "start":
                        saw_start_after_watermark = True
                    elif record["state"] != "waitstart" or saw_start_after_watermark:
                        return 1
            if args.require_start_after_watermark and not saw_start_after_watermark:
                return 1
            if last_record is None:
                print("0 empty 0")
            else:
                print(f"{last_record['seq']} {last_record['state']} {last_record['monotonic_ns']}")
            return 0
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return 1
    observer = AdminStateObserver(token=args.token, jsonl_path=args.jsonl, lifecycle_path=args.ready)
    signal.signal(signal.SIGINT, observer.stop)
    signal.signal(signal.SIGTERM, observer.stop)
    return observer.run(args.topic)


if __name__ == "__main__":
    sys.exit(main())
