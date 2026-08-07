#!/usr/bin/env python3
"""End-to-end sealed-memfd collector fixture for C-002AY1."""

import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time


def main() -> int:
    collector_binary = Path(sys.argv[1])
    producer_binary = Path(sys.argv[2])
    with tempfile.TemporaryDirectory(prefix="c002ay1-collector-") as temp:
        root = Path(temp)
        socket_path = root / "collector.sock"
        ready_path = root / "ready"
        artifact_path = root / "observation.json"
        run_id = "collector-e2e"
        planner_nonce, planner_instance = 101, 102
        pp_nonce, pp_instance = 201, 202
        collector = subprocess.Popen(
            [
                str(collector_binary),
                "--socket",
                str(socket_path),
                "--ready-file",
                str(ready_path),
                "--artifact",
                str(artifact_path),
                "--run-id",
                run_id,
                "--planner-nonce",
                str(planner_nonce),
                "--planner-instance",
                str(planner_instance),
                "--pp-nonce",
                str(pp_nonce),
                "--pp-instance",
                str(pp_instance),
                "--planner-quota",
                "1",
                "--pp-quota",
                "1",
                "--timeout-sec",
                "5",
                "--drain-period-ms",
                "5",
                "--planner-max-batch",
                "4",
                "--pp-max-batch",
                "16",
                "--nice",
                "10",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 2.0
        while not ready_path.exists() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not ready_path.exists():
            collector.kill()
            collector.wait(timeout=1)
            raise AssertionError("collector did not become ready")

        producer = subprocess.Popen(
            [
                str(producer_binary),
                str(socket_path),
                run_id,
                str(planner_nonce),
                str(planner_instance),
                str(pp_nonce),
                str(pp_instance),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        producer_pid = producer.pid
        producer_stdout, producer_stderr = producer.communicate(timeout=2)
        collector_stdout, collector_stderr = collector.communicate(timeout=6)
        if producer.returncode != 0:
            raise AssertionError(
                f"producer exit={producer.returncode}: "
                f"{producer_stdout} {producer_stderr}"
            )
        if collector.returncode != 0:
            raise AssertionError(
                f"collector exit={collector.returncode}: "
                f"{collector_stdout} {collector_stderr}"
            )
        artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
        assert artifact["complete"] is True
        assert artifact["authority_eligible"] is False
        assert artifact["planner_count"] == 1
        assert artifact["pp_count"] == 1
        assert artifact["planner_drop_count"] == 0
        assert artifact["pp_drop_count"] == 0
        scheduler = artifact["collector_scheduler"]
        assert scheduler["pid"] > 0
        assert scheduler["tid"] > 0
        assert scheduler["policy"] == "SCHED_OTHER"
        assert scheduler["nice"] >= 10
        assert scheduler["affinity_cpus"]
        assert scheduler["initial_cpu"] >= 0
        assert scheduler["final_cpu"] >= 0
        drain = artifact["collector_drain"]
        assert drain["period_ms"] == 5
        assert drain["planner_max_batch_limit"] == 4
        assert drain["pp_max_batch_limit"] == 16
        assert 0 < drain["planner_max_batch_observed"] <= 4
        assert 0 < drain["pp_max_batch_observed"] <= 16
        assert drain["iterations"] > 0
        assert drain["thread_cpu_ns"] > 0
        assert drain["wall_ns"] > 0
        assert drain["timer_expiration_count"] > 0
        assert drain["catch_up_count"] == 0
        assert artifact["producer_peers"]["planner_pid"] == producer_pid
        assert artifact["producer_peers"]["pp_pid"] == producer_pid
        assert artifact["planner_records"][0]["sequence"] == 1
        assert artifact["planner_records"][0]["ros_sec"] == 1
        assert artifact["planner_records"][0]["ros_nanosec"] == 2
        assert artifact["planner_records"][0]["legacy_publish_count"] == 1
        assert artifact["planner_records"][0]["v2_publish_count"] == 0
        assert artifact["planner_records"][0]["selected_proposal_count"] == 1
        assert artifact["pp_records"][0]["sequence"] == 1
        assert artifact["pp_records"][0]["ros_sec"] == 3
        assert artifact["pp_records"][0]["ros_nanosec"] == 4
        assert artifact["pp_records"][0]["controller_role"] == 2

        oversized_socket = root / "oversized.sock"
        oversized_ready = root / "oversized.ready"
        oversized_artifact = root / "oversized.json"
        oversized_collector = subprocess.Popen(
            [
                str(collector_binary),
                "--socket",
                str(oversized_socket),
                "--ready-file",
                str(oversized_ready),
                "--artifact",
                str(oversized_artifact),
                "--run-id",
                run_id,
                "--planner-nonce",
                str(planner_nonce),
                "--planner-instance",
                str(planner_instance),
                "--pp-nonce",
                str(pp_nonce),
                "--pp-instance",
                str(pp_instance),
                "--planner-quota",
                "1",
                "--pp-quota",
                "1",
                "--timeout-sec",
                "1",
                "--drain-period-ms",
                "5",
                "--planner-max-batch",
                "4",
                "--pp-max-batch",
                "16",
                "--nice",
                "10",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 2.0
        while not oversized_ready.exists() and time.monotonic() < deadline:
            time.sleep(0.005)
        assert oversized_ready.exists()
        run_bytes = run_id.encode()
        oversized_handshake = struct.pack(
            "<QIB3xiIQQ64s",
            0x4330303241593148,
            1,
            1,
            os.getpid(),
            len(run_bytes),
            planner_nonce,
            planner_instance,
            run_bytes.ljust(64, b"\0"),
        ) + b"x"
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as peer:
            peer.connect(str(oversized_socket))
            peer.send(oversized_handshake)
        oversized_collector.communicate(timeout=3)
        assert oversized_collector.returncode == 5
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
