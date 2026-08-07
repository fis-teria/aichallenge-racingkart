#!/usr/bin/env python3
"""Diagnostic-only raw-CDR handoff feasibility microgate."""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import os
from pathlib import Path
import socket
import struct
import time
import zlib

from autoware_auto_planning_msgs.msg import TrajectoryPoint
from multi_purpose_mpc_ros_msgs.msg import (
    ControllerExecutionEnvelope,
    ExecutionSweepSample,
)
import rclpy
from rclpy.qos import QoSProfile
from rclpy.serialization import deserialize_message, serialize_message

from c002ay0_pp_runtime_measurement import (
    canonical_debug,
    canonical_payload_hash,
    json_safe_artifact,
    strict_json_text,
)


TOPIC_ID = 1
BARRIER_TOPIC_ID = 0
PAYLOAD_CAP_BYTES = 262_144
KERNEL_BUFFER_BYTES = 1_048_576
HEADER = struct.Struct("!IQII")
MIN_COMPLETED_SAMPLES = 1_000
MAX_COMPLETED_SAMPLES = MIN_COMPLETED_SAMPLES
P99_LIMIT_NS = 1_000_000
P999_LIMIT_NS = 2_000_000
WORKER_READY_TIMEOUT_SEC = 2.0
WORKER_RESULT_TIMEOUT_SEC = 5.0
WORKER_JOIN_TIMEOUT_SEC = 2.0


def representative_execution_envelope() -> ControllerExecutionEnvelope:
    message = ControllerExecutionEnvelope()
    message.header.stamp.sec = 100
    message.header.stamp.nanosec = 123_000_000
    message.schema_version = 2
    message.producer_instance_id = 123
    message.command_sequence = 456
    message.command_envelope.producer_instance_id = 123
    message.command_envelope.command_sequence = 456
    message.command_envelope.command_age_sec = 0.01
    message.witness.source_generation = 98
    message.witness.base_source_generation = 99
    message.witness.trajectory_progress_m = 12.5
    message.witness.required_spatial_horizon_m = 20.0
    message.witness.raw_steering_tire_angle_rad = 0.1
    for index in range(100):
        base_point = TrajectoryPoint()
        base_point.pose.position.x = float(index)
        base_point.pose.position.y = float(index) * 0.1
        message.witness.base_trajectory.points.append(base_point)
        applied_point = TrajectoryPoint()
        applied_point.pose.position.x = float(index) + 0.2
        applied_point.pose.position.y = float(index) * 0.1 + 0.3
        message.witness.applied_trajectory.points.append(applied_point)
        rollout = ExecutionSweepSample()
        rollout.elapsed_time_sec = float(index) * 0.01
        rollout.pose.position.x = float(index) + 0.1
        message.witness.rollout_samples.append(rollout)
    return message


def canonical_wire(message: ControllerExecutionEnvelope) -> str:
    return strict_json_text(
        json_safe_artifact(canonical_debug("execution_envelope", message)),
        sort_keys=True,
        separators=(",", ":"),
    )


def configure_seqpacket(sock: socket.socket) -> None:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, KERNEL_BUFFER_BYTES)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, KERNEL_BUFFER_BYTES)


def encode_header(
    topic_id: int,
    receive_index: int,
    payload: bytes,
) -> bytes:
    return HEADER.pack(
        topic_id,
        receive_index,
        len(payload),
        zlib.crc32(payload) & 0xFFFFFFFF,
    )


def send_owned_raw_packet(
    sock: socket.socket,
    raw: object,
    *,
    topic_id: int,
    receive_index: int,
) -> dict[str, int]:
    owned = memoryview(raw).tobytes()
    payload_len = len(owned)
    if payload_len > PAYLOAD_CAP_BYTES:
        raise AssertionError(
            "raw_handoff_oversize "
            f"payload_len={payload_len} cap={PAYLOAD_CAP_BYTES}"
        )
    header = encode_header(topic_id, receive_index, owned)
    try:
        sent = sock.sendmsg((header, owned))
    except BlockingIOError as error:
        raise AssertionError("raw_handoff_seqpacket_full") from error
    expected = len(header) + payload_len
    if sent != expected:
        raise AssertionError(
            f"raw_handoff_partial_send sent={sent} expected={expected}"
        )
    return {
        "topic_id": topic_id,
        "receive_index": receive_index,
        "payload_len": payload_len,
        "crc32": zlib.crc32(owned) & 0xFFFFFFFF,
    }


def decode_data_packet(
    packet: bytes,
    *,
    expected_index: int,
) -> tuple[int, bytes]:
    if len(packet) < HEADER.size:
        raise AssertionError("raw_handoff_short_header")
    topic_id, receive_index, payload_len, expected_crc = HEADER.unpack_from(
        packet
    )
    payload = packet[HEADER.size:]
    if topic_id != TOPIC_ID:
        raise AssertionError(f"raw_handoff_unknown_topic_id={topic_id}")
    if receive_index != expected_index:
        raise AssertionError(
            "raw_handoff_index_mismatch "
            f"expected={expected_index} actual={receive_index}"
        )
    if payload_len != len(payload):
        raise AssertionError(
            "raw_handoff_length_mismatch "
            f"declared={payload_len} actual={len(payload)}"
        )
    if payload_len > PAYLOAD_CAP_BYTES:
        raise AssertionError("raw_handoff_worker_oversize")
    actual_crc = zlib.crc32(payload) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise AssertionError(
            "raw_handoff_crc_mismatch "
            f"expected={expected_crc} actual={actual_crc}"
        )
    return receive_index, payload


def _worker_main(
    data_fd: int,
    result_connection: object,
    oracle_wire: str,
    oracle_hash: str,
) -> None:
    data_socket = socket.socket(fileno=data_fd)
    processed = 0
    try:
        result_connection.send({"ready": True, "pid": os.getpid()})
        expected_index = 1
        while True:
            packet = data_socket.recv(PAYLOAD_CAP_BYTES + HEADER.size)
            if not packet:
                raise AssertionError("raw_handoff_unexpected_eof")
            if len(packet) < HEADER.size:
                raise AssertionError("raw_handoff_short_header")
            topic_id, receive_index, payload_len, crc = HEADER.unpack_from(
                packet
            )
            if topic_id == BARRIER_TOPIC_ID:
                if (
                    receive_index != expected_index
                    or payload_len != 0
                    or crc != 0
                    or len(packet) != HEADER.size
                ):
                    raise AssertionError("raw_handoff_invalid_barrier")
                break
            _, payload = decode_data_packet(
                packet,
                expected_index=expected_index,
            )
            typed = deserialize_message(
                payload,
                ControllerExecutionEnvelope,
            )
            if canonical_wire(typed) != oracle_wire:
                raise AssertionError("raw_handoff_semantic_parity_mismatch")
            typed_hash = canonical_payload_hash(
                canonical_debug("execution_envelope", typed)
            )
            if typed_hash != oracle_hash:
                raise AssertionError("raw_handoff_hash_parity_mismatch")
            processed += 1
            expected_index += 1
            result_connection.send({"processed_index": processed})
        result_connection.send(
            {
                "processed": processed,
                "accepted": processed,
                "drop_count": 0,
                "parity_success": True,
            }
        )
    except BaseException as error:
        try:
            result_connection.send(
                {
                    "worker_error": type(error).__name__,
                    "message": str(error),
                    "processed": processed,
                }
            )
        finally:
            raise
    finally:
        data_socket.close()
        result_connection.close()


def stop_worker_bounded(process: multiprocessing.Process) -> None:
    if not process.is_alive():
        process.join(WORKER_JOIN_TIMEOUT_SEC)
        return
    process.terminate()
    process.join(WORKER_JOIN_TIMEOUT_SEC)
    if process.is_alive():
        process.kill()
        process.join(WORKER_JOIN_TIMEOUT_SEC)
    if process.is_alive():
        raise AssertionError("raw_handoff_worker_residue")


def _stop_worker_after_failure(
    process: multiprocessing.Process,
    primary_reason: str,
) -> None:
    try:
        stop_worker_bounded(process)
    except BaseException as cleanup_error:
        raise AssertionError(
            f"{primary_reason}; cleanup_failure={cleanup_error}"
        ) from cleanup_error
    raise AssertionError(primary_reason)


def wait_worker_ready(
    process: multiprocessing.Process,
    result_connection: object,
    *,
    timeout_sec: float,
) -> dict[str, object]:
    if not result_connection.poll(timeout_sec):
        _stop_worker_after_failure(
            process,
            "raw_handoff_worker_ready_timeout",
        )
    try:
        ready = result_connection.recv()
    except EOFError:
        _stop_worker_after_failure(
            process,
            "raw_handoff_worker_ready_eof",
        )
    if ready.get("ready") is not True:
        _stop_worker_after_failure(
            process,
            f"raw_handoff_worker_not_ready={ready}",
        )
    return ready


def wait_worker_progress(
    process: multiprocessing.Process,
    result_connection: object,
    *,
    expected_index: int,
    timeout_sec: float,
) -> dict[str, object]:
    if not result_connection.poll(timeout_sec):
        _stop_worker_after_failure(
            process,
            "raw_handoff_worker_progress_timeout",
        )
    try:
        progress = result_connection.recv()
    except EOFError:
        _stop_worker_after_failure(
            process,
            "raw_handoff_worker_progress_eof",
        )
    if progress.get("processed_index") != expected_index:
        _stop_worker_after_failure(
            process,
            "raw_handoff_worker_progress_invalid "
            f"expected={expected_index} actual={progress}",
        )
    return progress


def wait_worker_result(
    process: multiprocessing.Process,
    result_connection: object,
    *,
    timeout_sec: float,
) -> dict[str, object]:
    if not result_connection.poll(timeout_sec):
        _stop_worker_after_failure(
            process,
            "raw_handoff_worker_result_timeout",
        )
    try:
        result = result_connection.recv()
    except EOFError as error:
        process.join(WORKER_JOIN_TIMEOUT_SEC)
        if process.is_alive():
            stop_worker_bounded(process)
        raise AssertionError(
            "raw_handoff_worker_failed "
            f"exitcode={process.exitcode} result=eof"
        ) from error
    process.join(WORKER_JOIN_TIMEOUT_SEC)
    if process.is_alive():
        _stop_worker_after_failure(
            process,
            "raw_handoff_worker_join_timeout",
        )
    if process.exitcode != 0 or "worker_error" in result:
        raise AssertionError(
            "raw_handoff_worker_failed "
            f"exitcode={process.exitcode} result={result}"
        )
    return result


def percentile(values: list[int], probability: float) -> int:
    if not values:
        raise AssertionError("raw_handoff_empty_distribution")
    ordered = sorted(values)
    rank = max(1, math.ceil(probability * len(ordered)))
    return ordered[rank - 1]


def distribution(values: list[int]) -> dict[str, int]:
    return {
        "count": len(values),
        "min_ns": min(values),
        "max_ns": max(values),
        "p50_ns": percentile(values, 0.5),
        "p99_ns": percentile(values, 0.99),
        "p999_ns": percentile(values, 0.999),
    }


def reference_gate_criteria(
    *,
    sample_count: int,
    worker_result: dict[str, object],
    callback_wall: dict[str, int],
    callback_thread: dict[str, int],
    spin_wall: dict[str, int],
    spin_thread: dict[str, int],
    worker_residue: bool,
) -> dict[str, bool]:
    return {
        "raw_callback_wall_p99": (
            callback_wall["p99_ns"] <= P99_LIMIT_NS
        ),
        "raw_callback_wall_p999": (
            callback_wall["p999_ns"] <= P999_LIMIT_NS
        ),
        "raw_callback_thread_cpu_p99": (
            callback_thread["p99_ns"] <= P99_LIMIT_NS
        ),
        "raw_callback_thread_cpu_p999": (
            callback_thread["p999_ns"] <= P999_LIMIT_NS
        ),
        "full_spin_wall_p99": spin_wall["p99_ns"] <= P99_LIMIT_NS,
        "full_spin_wall_p999": (
            spin_wall["p999_ns"] <= P999_LIMIT_NS
        ),
        "full_spin_thread_cpu_p99": (
            spin_thread["p99_ns"] <= P99_LIMIT_NS
        ),
        "full_spin_thread_cpu_p999": (
            spin_thread["p999_ns"] <= P999_LIMIT_NS
        ),
        "accepted_exact": (
            worker_result.get("accepted") == sample_count
        ),
        "processed_exact": (
            worker_result.get("processed") == sample_count
        ),
        "drop_zero": worker_result.get("drop_count") == 0,
        "worker_residue_absent": not worker_residue,
        "typed_parity_success": (
            worker_result.get("parity_success") is True
        ),
    }


def run_microgate(
    artifact_root: Path,
    *,
    sample_count: int = MIN_COMPLETED_SAMPLES,
) -> dict[str, object]:
    if (
        sample_count < MIN_COMPLETED_SAMPLES
        or sample_count > MAX_COMPLETED_SAMPLES
    ):
        raise ValueError(
            "sample_count must be exactly "
            f"{MIN_COMPLETED_SAMPLES}"
        )
    if artifact_root.exists():
        raise FileExistsError(f"refusing to overwrite {artifact_root}")
    artifact_root.mkdir(parents=True)
    representative = representative_execution_envelope()
    serialized = serialize_message(representative)
    if len(serialized) > PAYLOAD_CAP_BYTES:
        raise AssertionError("representative exceeds fixed payload cap")
    representative = deserialize_message(
        serialized,
        ControllerExecutionEnvelope,
    )
    oracle_wire = canonical_wire(representative)
    oracle_hash = canonical_payload_hash(
        canonical_debug("execution_envelope", representative)
    )

    callback_wall_ns: list[int] = []
    callback_thread_cpu_ns: list[int] = []
    spin_wall_ns: list[int] = []
    spin_thread_cpu_ns: list[int] = []
    callback_records: list[dict[str, int]] = []
    receive_index = 0
    callback_failure: BaseException | None = None
    effective_kernel_buffers: dict[str, int] = {}
    worker_result: dict[str, object] = {}
    parent_socket: socket.socket | None = None
    child_socket: socket.socket | None = None
    parent_result: object | None = None
    child_result: object | None = None
    worker: multiprocessing.Process | None = None
    detached_child_fd: int | None = None
    node: object | None = None
    subscription: object | None = None
    publisher: object | None = None
    rclpy_initialized = False
    primary_error: BaseException | None = None
    cleanup_errors: list[str] = []

    def raw_callback(raw: object) -> None:
        nonlocal callback_failure, receive_index
        assert parent_socket is not None
        wall_started = time.monotonic_ns()
        thread_started = time.thread_time_ns()
        try:
            next_index = receive_index + 1
            record = send_owned_raw_packet(
                parent_socket,
                raw,
                topic_id=TOPIC_ID,
                receive_index=next_index,
            )
            receive_index = next_index
            callback_records.append(record)
        except BaseException as error:
            callback_failure = error
            raise
        finally:
            callback_wall_ns.append(time.monotonic_ns() - wall_started)
            callback_thread_cpu_ns.append(
                time.thread_time_ns() - thread_started
            )

    try:
        parent_socket, child_socket = socket.socketpair(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )
        configure_seqpacket(parent_socket)
        configure_seqpacket(child_socket)
        effective_kernel_buffers = {
            "parent_send": parent_socket.getsockopt(
                socket.SOL_SOCKET,
                socket.SO_SNDBUF,
            ),
            "parent_receive": parent_socket.getsockopt(
                socket.SOL_SOCKET,
                socket.SO_RCVBUF,
            ),
            "child_send": child_socket.getsockopt(
                socket.SOL_SOCKET,
                socket.SO_SNDBUF,
            ),
            "child_receive": child_socket.getsockopt(
                socket.SOL_SOCKET,
                socket.SO_RCVBUF,
            ),
        }
        if (
            effective_kernel_buffers["parent_send"]
            != effective_kernel_buffers["child_send"]
            or effective_kernel_buffers["parent_receive"]
            != effective_kernel_buffers["child_receive"]
        ):
            raise AssertionError(
                "raw_handoff_kernel_buffer_mismatch="
                f"{effective_kernel_buffers}"
            )
        parent_socket.setblocking(False)
        parent_result, child_result = multiprocessing.Pipe(duplex=False)
        detached_child_fd = child_socket.detach()
        child_socket = None
        worker = multiprocessing.Process(
            target=_worker_main,
            args=(
                detached_child_fd,
                child_result,
                oracle_wire,
                oracle_hash,
            ),
            name="c002ay0_raw_handoff_worker",
        )
        worker.start()
        os.close(detached_child_fd)
        detached_child_fd = None
        child_result.close()
        child_result = None
        wait_worker_ready(
            worker,
            parent_result,
            timeout_sec=WORKER_READY_TIMEOUT_SEC,
        )

        rclpy.init()
        rclpy_initialized = True
        node = rclpy.create_node("c002ay0_raw_handoff_microgate")
        subscription = node.create_subscription(
            ControllerExecutionEnvelope,
            "/c002ay0/raw_handoff",
            raw_callback,
            QoSProfile(depth=10),
            raw=True,
        )
        publisher = node.create_publisher(
            ControllerExecutionEnvelope,
            "/c002ay0/raw_handoff",
            QoSProfile(depth=10),
        )
        discovery_deadline = time.monotonic() + 3.0
        while (
            publisher.get_subscription_count() < 1
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.01)
        if publisher.get_subscription_count() < 1:
            raise AssertionError("raw_handoff_dds_discovery_timeout")
        for _ in range(sample_count):
            publisher.publish(representative)
            spin_wall_started = time.monotonic_ns()
            spin_thread_started = time.thread_time_ns()
            rclpy.spin_once(node, timeout_sec=0.1)
            spin_wall_ns.append(
                time.monotonic_ns() - spin_wall_started
            )
            spin_thread_cpu_ns.append(
                time.thread_time_ns() - spin_thread_started
            )
            if callback_failure is not None:
                raise callback_failure
            wait_worker_progress(
                worker,
                parent_result,
                expected_index=receive_index,
                timeout_sec=WORKER_RESULT_TIMEOUT_SEC,
            )
        if receive_index != sample_count:
            raise AssertionError(
                "raw_handoff_completed_sample_mismatch "
                f"expected={sample_count} actual={receive_index}"
            )
        barrier = HEADER.pack(
            BARRIER_TOPIC_ID,
            sample_count + 1,
            0,
            0,
        )
        try:
            sent = parent_socket.sendmsg((barrier,))
        except BlockingIOError as error:
            raise AssertionError("raw_handoff_barrier_full") from error
        if sent != len(barrier):
            raise AssertionError("raw_handoff_barrier_partial")
        worker_result = wait_worker_result(
            worker,
            parent_result,
            timeout_sec=WORKER_RESULT_TIMEOUT_SEC,
        )
    except BaseException as error:
        primary_error = error
    finally:
        for label, action in (
            (
                "subscription",
                lambda: (
                    node.destroy_subscription(subscription)
                    if node is not None and subscription is not None
                    else None
                ),
            ),
            (
                "publisher",
                lambda: (
                    node.destroy_publisher(publisher)
                    if node is not None and publisher is not None
                    else None
                ),
            ),
            (
                "node",
                lambda: node.destroy_node() if node is not None else None,
            ),
            (
                "rclpy",
                lambda: rclpy.shutdown() if rclpy_initialized else None,
            ),
            (
                "parent_socket",
                lambda: (
                    parent_socket.close()
                    if parent_socket is not None
                    else None
                ),
            ),
            (
                "child_socket",
                lambda: (
                    child_socket.close()
                    if child_socket is not None
                    else None
                ),
            ),
            (
                "detached_child_fd",
                lambda: (
                    os.close(detached_child_fd)
                    if detached_child_fd is not None
                    else None
                ),
            ),
            (
                "parent_result",
                lambda: (
                    parent_result.close()
                    if parent_result is not None
                    else None
                ),
            ),
            (
                "child_result",
                lambda: (
                    child_result.close()
                    if child_result is not None
                    else None
                ),
            ),
            (
                "worker",
                lambda: (
                    stop_worker_bounded(worker)
                    if worker is not None
                    else None
                ),
            ),
        ):
            try:
                action()
            except BaseException as cleanup_error:
                cleanup_errors.append(f"{label}={cleanup_error}")

    if primary_error is not None or cleanup_errors:
        failure = {
            "primary_failure": (
                {
                    "type": type(primary_error).__name__,
                    "message": str(primary_error),
                }
                if primary_error is not None
                else None
            ),
            "cleanup_failures": cleanup_errors,
        }
        (artifact_root / "failure.json").write_text(
            json.dumps(failure, indent=2, sort_keys=True) + "\n"
        )
        if primary_error is not None and cleanup_errors:
            raise AssertionError(
                "raw_handoff_primary_and_cleanup_failure "
                f"primary={primary_error} cleanup={cleanup_errors}"
            ) from primary_error
        if primary_error is not None:
            raise primary_error
        raise AssertionError(
            f"raw_handoff_cleanup_failure cleanup={cleanup_errors}"
        )

    callback_wall = distribution(callback_wall_ns)
    callback_thread = distribution(callback_thread_cpu_ns)
    spin_wall = distribution(spin_wall_ns)
    spin_thread = distribution(spin_thread_cpu_ns)
    worker_residue = worker.is_alive() if worker is not None else False
    gate_criteria = reference_gate_criteria(
        sample_count=sample_count,
        worker_result=worker_result,
        callback_wall=callback_wall,
        callback_thread=callback_thread,
        spin_wall=spin_wall,
        spin_thread=spin_thread,
        worker_residue=worker_residue,
    )
    gate_pass = all(gate_criteria.values())
    result: dict[str, object] = {
        "scope": "single_handoff_feasibility",
        "per_sample_worker_ack": True,
        "sustained_100hz_throughput_proven": False,
        "worker_canonical_throughput_included": False,
        "m4_acceptance_credit": False,
        "diagnostic_only": True,
        "not_acceptance": True,
        "m4_wcet_eligible": False,
        "no_retry": True,
        "sample_count": sample_count,
        "typed_serialized_payload_len": len(serialized),
        "raw_cdr_payload_len": callback_records[0]["payload_len"],
        "representative_shape": {
            "base_points": 100,
            "applied_points": 100,
            "rollout_samples": 100,
        },
        "payload_cap_bytes": PAYLOAD_CAP_BYTES,
        "kernel_buffer_bytes_requested": KERNEL_BUFFER_BYTES,
        "kernel_buffer_bytes_effective": effective_kernel_buffers,
        "callback_records": {
            "count": len(callback_records),
            "first": callback_records[0],
            "last": callback_records[-1],
            "records": callback_records,
        },
        "worker": {
            **worker_result,
            "residue": worker_residue,
        },
        "raw_callback_handoff_wall": callback_wall,
        "raw_callback_handoff_thread_cpu": callback_thread,
        "full_spin_wall": spin_wall,
        "full_spin_thread_cpu": spin_thread,
        "reference_gate": {
            "p99_limit_ns": P99_LIMIT_NS,
            "p999_limit_ns": P999_LIMIT_NS,
            "criteria": gate_criteria,
            "pass": gate_pass,
        },
        "remaining_risks": [
            (
                "unpaced producer previously reached hard-invalid EAGAIN; "
                "sustained 100 Hz queue non-saturation is not proven"
            ),
            (
                "per-sample worker acknowledgement is outside measured "
                "callback and full-spin distributions"
            ),
        ],
    }
    (artifact_root / "result.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument(
        "--samples",
        type=int,
        default=MIN_COMPLETED_SAMPLES,
    )
    args = parser.parse_args()
    result = run_microgate(args.artifact_root, sample_count=args.samples)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["reference_gate"]["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
