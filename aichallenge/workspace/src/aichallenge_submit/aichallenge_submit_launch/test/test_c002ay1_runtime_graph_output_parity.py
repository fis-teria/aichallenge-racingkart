#!/usr/bin/env python3
"""Actual-node Graph 3 and fixed-clock output parity for C-002AY1."""

from __future__ import annotations

import copy
from array import array
from collections import deque
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from multi_purpose_mpc_ros_msgs.msg import (
    ControllerCommandEnvelope,
    ControllerExecutionEnvelope,
    ControllerTrackingStatus,
    FreeRunExecutionAck,
    FreeRunSourceKey,
    MotionAuthorityGrant,
    OvertakePlan,
    SafetyConstraint,
    SafetyStopStatus,
)
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32MultiArray, String


PLANNER_NAME = "overtake_planner_node"
PP_NAME = "simple_pure_pursuit"
MUX_NAME = "hybrid_control_mux_node"
EXPECTED_NODES = {PLANNER_NAME, PP_NAME, MUX_NAME}
RUN_ID = "runtime-graph-parity"
PLANNER_NONCE = 301
PLANNER_INSTANCE = 302
PP_NONCE = 401
PP_INSTANCE = 402
RING_MAGIC = 0x4330303241593152
ABI_VERSION = 1
LAYOUT_VERSION = 1
HEADER_SIZE = 320
HARNESS_READY = 1


def command(*parts: str) -> list[str]:
    return list(parts)


def start_process(args: list[str]) -> subprocess.Popen[bytes]:
    return subprocess.Popen(
        args,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def stop_process(process: subprocess.Popen[bytes] | None) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=1)


def endpoint_signature(observer: rclpy.node.Node) -> tuple:
    signature = []
    for node_name in sorted(EXPECTED_NODES):
        publishers = observer.get_publisher_names_and_types_by_node(
            node_name, "/"
        )
        subscriptions = observer.get_subscriber_names_and_types_by_node(
            node_name, "/"
        )
        services = observer.get_service_names_and_types_by_node(node_name, "/")
        clients = observer.get_client_names_and_types_by_node(node_name, "/")
        signature.append(
            (
                node_name,
                tuple(sorted((name, tuple(types)) for name, types in publishers)),
                tuple(
                    sorted((name, tuple(types)) for name, types in subscriptions)
                ),
                tuple(sorted((name, tuple(types)) for name, types in services)),
                tuple(sorted((name, tuple(types)) for name, types in clients)),
            )
        )
    return tuple(signature)


def canonical_payload(topic: str, message: object) -> str:
    canonical = copy.deepcopy(message)
    if topic == "/overtake/plan":
        canonical.planner_instance_id = 1
        # This digest intentionally binds the real planner instance.  Once the
        # fixture normalizes that identity, normalize its derived field too;
        # digest correctness has separate contract tests.
        canonical.free_run_canonical_payload_sha256 = [0] * 32
    elif topic == "/pp/command_envelope":
        canonical.producer_instance_id = 1
    elif topic == "/pp/execution_envelope":
        canonical.producer_instance_id = 1
        canonical.command_envelope.producer_instance_id = 1
    return hashlib.sha256(bytes(serialize_message(canonical))).hexdigest()


class OutputCapture:
    def __init__(self, node: rclpy.node.Node) -> None:
        self.current: dict[str, list[str]] = {}
        self.evidence_counts = {"ack": 0, "source": 0}
        self.mux_exact_join_count = 0
        self.mux_record_count = 0
        self.mux_last_live_reasons: dict[str, object] = {}
        self.last_ack_state: dict[str, object] = {}
        self.last_pp_tracking_state: dict[str, object] = {}
        self.pp_command_events: deque[dict[str, object]] = deque(maxlen=256)
        self.pp_envelope_events: deque[dict[str, object]] = deque(maxlen=256)
        self.pp_ack_events: deque[dict[str, object]] = deque(maxlen=256)
        self.pp_tracking_events: deque[dict[str, object]] = deque(maxlen=256)
        self.mux_command_events: deque[dict[str, object]] = deque(maxlen=256)
        self.mux_tracking_events: deque[dict[str, object]] = deque(maxlen=256)
        self.mux_debug_events: deque[dict[str, object]] = deque(maxlen=256)
        self.mux_grant_events: deque[dict[str, object]] = deque(maxlen=256)
        topics = {
            "/overtake/reference_override": Float32MultiArray,
            "/overtake/safety_constraint": SafetyConstraint,
            "/overtake/plan": OvertakePlan,
            "/debug/overtake/mode": String,
            "/pp/control_cmd": AckermannControlCommand,
            "/pp/raw_control_cmd": AckermannControlCommand,
            "/pp/tracking_status": ControllerTrackingStatus,
            "/pp/command_envelope": ControllerCommandEnvelope,
            "/pp/execution_envelope": ControllerExecutionEnvelope,
            "/mux/control_cmd": AckermannControlCommand,
            "/mux/tracking_status": ControllerTrackingStatus,
        }
        self.subscriptions = [
            node.create_subscription(
                message_type,
                topic,
                lambda message, topic=topic: self._receive(topic, message),
                10,
            )
            for topic, message_type in topics.items()
        ]
        self.subscriptions.extend(
            (
                node.create_subscription(
                    FreeRunExecutionAck,
                    "/pp/free_run_execution_ack",
                    self._receive_ack,
                    10,
                ),
                node.create_subscription(
                    FreeRunSourceKey,
                    "/pp/free_run_source_key",
                    lambda _: self._count_evidence("source"),
                    10,
                ),
                node.create_subscription(
                    String,
                    "/mux/debug",
                    self._receive_mux_debug,
                    10,
                ),
                node.create_subscription(
                    MotionAuthorityGrant,
                    "/mux/motion_authority_grant",
                    self._receive_mux_grant,
                    10,
                ),
            )
        )

    def reset(self) -> None:
        self.current = {}
        self.evidence_counts = {"ack": 0, "source": 0}
        self.mux_exact_join_count = 0
        self.mux_record_count = 0
        self.mux_last_live_reasons = {}
        self.last_ack_state = {}
        self.last_pp_tracking_state = {}
        self.pp_command_events.clear()
        self.pp_envelope_events.clear()
        self.pp_ack_events.clear()
        self.pp_tracking_events.clear()
        self.mux_command_events.clear()
        self.mux_tracking_events.clear()
        self.mux_debug_events.clear()
        self.mux_grant_events.clear()

    def _receive_mux_grant(self, message: MotionAuthorityGrant) -> None:
        self.mux_grant_events.append(
            {
                "received_ns": time.monotonic_ns(),
                "valid": bool(message.valid),
                "reason": str(message.reason),
                "sequence": int(message.grant_sequence),
                "phase": int(message.phase),
                "plan_identity": (
                    int(message.plan_sample_key.race_arm_epoch),
                    int(message.plan_sample_key.planner_instance_id),
                    int(message.plan_sample_key.attempt_id),
                    str(message.plan_sample_key.target_vehicle_id),
                    int(message.plan_sample_key.pass_direction),
                    int(message.plan_sample_key.connector_transaction_id),
                    int(message.plan_sample_key.plan_stamp.sec) * 1_000_000_000
                    + int(message.plan_sample_key.plan_stamp.nanosec),
                    int(message.plan_sample_key.plan_generation),
                    int(message.candidate_revision),
                    tuple(int(value) for value in message.candidate_content_sha256),
                ),
                "warmup_identity": (
                    int(message.warmup_plan_sample_key.race_arm_epoch),
                    int(message.warmup_plan_sample_key.planner_instance_id),
                    int(message.warmup_plan_sample_key.attempt_id),
                    str(message.warmup_plan_sample_key.target_vehicle_id),
                    int(message.warmup_plan_sample_key.pass_direction),
                    int(message.warmup_plan_sample_key.connector_transaction_id),
                    int(message.warmup_plan_sample_key.plan_stamp.sec)
                    * 1_000_000_000
                    + int(message.warmup_plan_sample_key.plan_stamp.nanosec),
                    int(message.warmup_plan_sample_key.plan_generation),
                    int(message.warmup_candidate_revision),
                    tuple(
                        int(value)
                        for value in message.warmup_candidate_content_sha256
                    ),
                ),
                "warmup_token": int(
                    message.warmup_lateral_stop_authority_token
                ),
                "pp_identity": (
                    int(message.pp_producer_instance_id),
                    int(message.pp_command_sequence),
                    int(message.pp_command_stamp.sec) * 1_000_000_000
                    + int(message.pp_command_stamp.nanosec),
                ),
                "warmup_pp_identity": (
                    int(message.warmup_pp_producer_instance_id),
                    int(message.warmup_pp_command_sequence),
                    int(message.warmup_pp_command_stamp.sec) * 1_000_000_000
                    + int(message.warmup_pp_command_stamp.nanosec),
                ),
                "constraint_identity": (
                    int(message.constraint_generation),
                    int(message.constraint_stamp.sec) * 1_000_000_000
                    + int(message.constraint_stamp.nanosec),
                ),
                "command": (
                    float(message.speed_mps),
                    float(message.acceleration_mps2),
                    float(message.steering_tire_angle_rad),
                    float(message.steering_tire_rotation_rate_radps),
                ),
            }
        )

    def _count_evidence(self, kind: str) -> None:
        self.evidence_counts[kind] += 1

    def _receive_ack(self, message: FreeRunExecutionAck) -> None:
        self._count_evidence("ack")
        self.pp_ack_events.append(
            {
                "received_ns": time.monotonic_ns(),
                "eligible": bool(message.ack_eligible),
                "state": int(message.evidence_state),
                "reason": str(message.evidence_reason),
                "producer": int(message.source_key.controller_instance_id),
                "sequence": int(message.controller_sequence),
                "plan_generation": int(message.plan_key.plan_generation),
                "command_stamp_ns": (
                    int(message.controller_command_stamp.sec)
                    * 1_000_000_000
                    + int(message.controller_command_stamp.nanosec)
                ),
            }
        )
        self.last_ack_state = {
            "eligible": bool(message.ack_eligible),
            "state": int(message.evidence_state),
            "reason": str(message.evidence_reason),
            "raw_steering_rad": float(message.raw_steering_tire_angle_rad),
            "output_steering_rad": float(
                message.output_steering_tire_angle_rad
            ),
            "hard_steering_limit_rad": float(
                message.hard_steering_tire_angle_limit_rad
            ),
        }

    def _receive_mux_debug(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get("free_run_live_exact_selected_envelope_valid"):
            self.mux_exact_join_count += 1
        if payload.get("free_run_live_exact_published_record_valid"):
            self.mux_record_count += 1
        self.mux_last_live_reasons = {
            "ack_valid": payload.get("free_run_live_exact_ack_valid"),
            "ack_reason": payload.get("free_run_live_exact_ack_reason"),
            "join_reason": payload.get(
                "free_run_live_exact_selected_envelope_reason"
            ),
            "record_reason": payload.get(
                "free_run_live_exact_published_record_reason"
            ),
        }
        self.mux_debug_events.append(
            {
                "received_ns": time.monotonic_ns(),
                "join_reason": payload.get(
                    "free_run_live_exact_selected_envelope_reason"
                ),
                "forward_reason": payload.get(
                    "free_run_live_exact_forward_transition_reason"
                ),
                "source": payload.get("source"),
                "reason": payload.get("reason"),
                "authority_generation": payload.get(
                    "authority_plan_generation"
                ),
                "tracking_generation": payload.get(
                    "tracking_plan_generation"
                ),
                "tracking_delivery_gap_usable": payload.get(
                    "pp_tracking_delivery_gap_usable"
                ),
                "record_valid": bool(
                    payload.get(
                        "free_run_live_exact_published_record_valid"
                    )
                ),
                "record_reason": payload.get(
                    "free_run_live_exact_published_record_reason"
                ),
                "record_steering_rad": payload.get(
                    "free_run_live_exact_published_record_steering_rad"
                ),
                "record_age_sec": payload.get(
                    "free_run_live_exact_published_record_age_sec"
                ),
                "ack_identity": payload.get(
                    "free_run_live_exact_ack_identity"
                ),
                "selected_envelope_identity": payload.get(
                    "free_run_live_exact_selected_envelope_identity"
                ),
                "ack_age_sec": payload.get(
                    "free_run_live_exact_ack_age_sec"
                ),
                "source_age_sec": payload.get(
                    "free_run_live_exact_source_key_age_sec"
                ),
                "output_steering_rad": payload.get("output_steer_rad"),
                "steering_rate_limited": payload.get(
                    "steering_rate_limited"
                ),
                "source_valid": payload.get(
                    "free_run_live_exact_source_key_valid"
                ),
                "source_reason": payload.get(
                    "free_run_live_exact_source_key_reason"
                ),
                "envelope_valid": payload.get(
                    "pure_pursuit_envelope_shadow_valid"
                ),
                "envelope_reason": payload.get(
                    "pure_pursuit_envelope_shadow_reason"
                ),
                "envelope_candidate_count": payload.get(
                    "pure_pursuit_envelope_shadow_candidate_valid_count"
                ),
                "motion_first_false": payload.get(
                    "motion_pp_tracking_proof_first_false"
                ),
                "external_stop_latched": bool(
                    payload.get("external_stop_latched")
                ),
            }
        )

    def _receive(self, topic: str, message: object) -> None:
        if topic == "/pp/tracking_status":
            self.pp_tracking_events.append(
                {
                    "received_ns": time.monotonic_ns(),
                    "usable": bool(message.trajectory_tracking_usable),
                    "fresh": bool(message.pp_command_fresh),
                    "reason": str(message.reason),
                    "generation": int(message.plan_generation),
                    "stamp_ns": (
                        int(message.header.stamp.sec) * 1_000_000_000
                        + int(message.header.stamp.nanosec)
                    ),
                }
            )
            self.last_pp_tracking_state = {
                "usable": bool(message.trajectory_tracking_usable),
                "fresh": bool(message.pp_command_fresh),
                "reason": str(message.reason),
                "generation": int(message.plan_generation),
            }
        elif topic == "/pp/command_envelope":
            self.pp_envelope_events.append(
                {
                    "received_ns": time.monotonic_ns(),
                    "schema": int(message.schema_version),
                    "producer": int(message.producer_instance_id),
                    "sequence": int(message.command_sequence),
                    "plan_generation": int(message.plan_generation),
                    "plan_key": (
                        int(message.plan_sample_key.race_arm_epoch),
                        int(message.plan_sample_key.planner_instance_id),
                        int(message.plan_sample_key.attempt_id),
                        str(message.plan_sample_key.target_vehicle_id),
                        int(message.plan_sample_key.pass_direction),
                        int(message.plan_sample_key.connector_transaction_id),
                        int(message.plan_sample_key.plan_generation),
                    ),
                    "candidate_revision": int(message.candidate_revision),
                    "authority_kind": int(
                        message.lateral_stop_authority_kind
                    ),
                    "authority_direction": int(
                        message.lateral_stop_transaction_pass_direction
                    ),
                    "authority_token": int(
                        message.lateral_stop_authority_token
                    ),
                    "tracking_usable": bool(
                        message.trajectory_tracking_usable
                    ),
                }
            )
        elif topic in ("/pp/control_cmd", "/mux/control_cmd"):
            stamp = message.stamp
            event = {
                "received_ns": time.monotonic_ns(),
                "stamp_ns": int(stamp.sec) * 1_000_000_000
                + int(stamp.nanosec),
                "speed_mps": float(message.longitudinal.speed),
                "acceleration_mps2": float(
                    message.longitudinal.acceleration
                ),
                "steering_rad": float(
                    message.lateral.steering_tire_angle
                ),
                "steering_rate_radps": float(
                    message.lateral.steering_tire_rotation_rate
                ),
                "cdr_sha256": hashlib.sha256(
                    bytes(serialize_message(message))
                ).hexdigest(),
            }
            target = (
                self.pp_command_events
                if topic == "/pp/control_cmd"
                else self.mux_command_events
            )
            target.append(event)
        elif topic == "/mux/tracking_status":
            self.mux_tracking_events.append(
                {
                    "received_ns": time.monotonic_ns(),
                    "usable": bool(message.trajectory_tracking_usable),
                    "reason": str(message.reason),
                    "generation": int(message.plan_generation),
                    "pass_probe_exact_current_usable": bool(
                        message.pass_probe_exact_current_usable
                    ),
                }
            )
        payloads = self.current.setdefault(topic, [])
        if len(payloads) < 2:
            payloads.append(canonical_payload(topic, message))

    def signature(self) -> tuple:
        return tuple(
            (topic, tuple(payloads))
            for topic, payloads in sorted(self.current.items())
        )


class FreeRunAckRelay:
    """Relay one exact PP transport batch, then intentionally omit only ACK."""

    def __init__(self, node: rclpy.node.Node) -> None:
        self.forward_enabled = True
        self.command_only_enabled = False
        self.forward_count = 0
        # Mux選択に必要な連続envelopeを含むbounded delivery windowを残す。
        # baselineは後段で、このrelayが実際にforwardしたACK/command pairから
        # 取得する。PP subscriberのより新しいramp値とは混同しない。
        self.stop_after_forward_count = 2
        self.last_ack: FreeRunExecutionAck | None = None
        self.last_forwarded_ack: FreeRunExecutionAck | None = None
        self.last_forwarded_envelope_identity: tuple[int, int, int, int] | None = (
            None
        )
        self.last_forwarded_time_ns: int | None = None
        self.forwarded_envelope_producer_id: int | None = None
        self.last_forwarded_envelope_sequence = 0
        self.commands: dict[int, AckermannControlCommand] = {}
        self.statuses: dict[int, ControllerTrackingStatus] = {}
        self.envelopes: dict[int, ControllerCommandEnvelope] = {}
        self.acks: dict[int, FreeRunExecutionAck] = {}
        self.command_only_forwarded_stamps: set[int] = set()
        self.command_publisher = node.create_publisher(
            AckermannControlCommand, "/fixture/mux_pp_cmd", 10
        )
        self.status_publisher = node.create_publisher(
            ControllerTrackingStatus, "/fixture/mux_pp_tracking_status", 10
        )
        self.envelope_publisher = node.create_publisher(
            ControllerCommandEnvelope, "/fixture/mux_pp_command_envelope", 10
        )
        self.source_publisher = node.create_publisher(
            FreeRunSourceKey, "/fixture/mux_free_run_source_key", 10
        )
        self.ack_publisher = node.create_publisher(
            FreeRunExecutionAck,
            "/fixture/mux_free_run_execution_ack",
            10,
        )
        self.subscriptions = (
            node.create_subscription(
                AckermannControlCommand,
                "/pp/control_cmd",
                self._receive_command,
                10,
            ),
            node.create_subscription(
                ControllerTrackingStatus,
                "/pp/tracking_status",
                self._receive_status,
                10,
            ),
            node.create_subscription(
                ControllerCommandEnvelope,
                "/pp/command_envelope",
                self._receive_envelope,
                10,
            ),
            node.create_subscription(
                FreeRunSourceKey,
                "/pp/free_run_source_key",
                lambda _: None,
                10,
            ),
            node.create_subscription(
            FreeRunExecutionAck,
            "/pp/free_run_execution_ack",
            self._receive_ack,
            10,
            ),
        )

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _remember(self, cache: dict, stamp_ns: int, message: object) -> None:
        cache[stamp_ns] = copy.deepcopy(message)
        while len(cache) > 128:
            cache.pop(next(iter(cache)))

    def _receive_status(self, message: ControllerTrackingStatus) -> None:
        stamp_ns = self._stamp_ns(message.header.stamp)
        self._remember(self.statuses, stamp_ns, message)
        self._forward_exact_batch(stamp_ns)
        self._forward_command_only(stamp_ns)

    def _receive_envelope(self, message: ControllerCommandEnvelope) -> None:
        stamp_ns = self._stamp_ns(message.header.stamp)
        self._remember(self.envelopes, stamp_ns, message)
        self._forward_exact_batch(stamp_ns)
        self._forward_command_only(stamp_ns)

    def _receive_command(self, message: AckermannControlCommand) -> None:
        stamp_ns = self._stamp_ns(message.stamp)
        self._remember(self.commands, stamp_ns, message)
        self._forward_exact_batch(stamp_ns)
        self._forward_command_only(stamp_ns)

    def _forward_command_only(self, stamp_ns: int) -> None:
        if (
            not self.command_only_enabled
            or stamp_ns in self.command_only_forwarded_stamps
        ):
            return
        command = self.commands.get(stamp_ns)
        status = self.statuses.get(stamp_ns)
        envelope = self.envelopes.get(stamp_ns)
        if command is None or status is None or envelope is None:
            return
        if (
            int(envelope.command_sequence)
            <= self.last_forwarded_envelope_sequence
        ):
            return
        self.status_publisher.publish(status)
        self.command_publisher.publish(command)
        self.envelope_publisher.publish(envelope)
        self.command_only_forwarded_stamps.add(stamp_ns)
        while len(self.command_only_forwarded_stamps) > 128:
            self.command_only_forwarded_stamps.pop()

    def _receive_ack(self, message: FreeRunExecutionAck) -> None:
        self.last_ack = copy.deepcopy(message)
        stamp_ns = self._stamp_ns(message.controller_command_stamp)
        self._remember(self.acks, stamp_ns, message)
        self._forward_exact_batch(stamp_ns)

    def _forward_exact_batch(self, stamp_ns: int) -> None:
        if not self.forward_enabled:
            return
        command = self.commands.get(stamp_ns)
        status = self.statuses.get(stamp_ns)
        envelope = self.envelopes.get(stamp_ns)
        ack = self.acks.get(stamp_ns)
        if command is None or status is None or envelope is None or ack is None:
            return
        envelope_sequence = int(envelope.command_sequence)
        if envelope_sequence <= self.last_forwarded_envelope_sequence:
            return
        envelope_producer_id = int(envelope.producer_instance_id)
        if self.forward_count > 0:
            if envelope_producer_id != self.forwarded_envelope_producer_id:
                raise AssertionError(
                    "exact relay envelope producer changed within baseline pair"
                )
            if envelope_sequence != self.last_forwarded_envelope_sequence + 1:
                raise AssertionError(
                    "exact relay envelope sequence is not consecutive"
                )
        self.source_publisher.publish(ack.source_key)
        self.status_publisher.publish(status)
        self.command_publisher.publish(command)
        self.envelope_publisher.publish(envelope)
        self.ack_publisher.publish(ack)
        self.last_forwarded_time_ns = time.monotonic_ns()
        self.last_forwarded_ack = copy.deepcopy(ack)
        self.last_forwarded_envelope_identity = (
            envelope_producer_id,
            envelope_sequence,
            self._stamp_ns(envelope.header.stamp),
            int(envelope.plan_generation),
        )
        self.forwarded_envelope_producer_id = envelope_producer_id
        self.last_forwarded_envelope_sequence = envelope_sequence
        self.forward_count += 1
        # The transport contract requires two consecutive envelopes from one
        # producer before selection. Forward exactly that bounded pair, then
        # freeze it so the fixture does not consume the non-refreshable
        # SourceKey lease while waiting for baseline evidence.
        if self.forward_count >= self.stop_after_forward_count:
            self.forward_enabled = False

    def freeze(self) -> None:
        if self.forward_count <= 0 or self.last_forwarded_ack is None:
            raise AssertionError("cannot freeze ACK relay before a real ACK")
        self.forward_enabled = False
        self.command_only_enabled = True


def free_run_plan_digest(message: OvertakePlan) -> bytes:
    wire = bytearray()

    def append(fmt: str, value: int) -> None:
        wire.extend(struct.pack(fmt, value))

    def append_string(value: str) -> None:
        encoded = value.encode()
        append("<I", len(encoded))
        wire.extend(encoded)

    append_string("FREE_RUN_PLAN_PAYLOAD_V1")
    append_string(message.header.frame_id)
    append("<B", message.phase)
    append("<I", message.attempt_id)
    append_string(message.target_vehicle_id)
    append("<b", message.pass_direction)
    append("<B", int(message.trajectory_authorized))
    append("<B", int(message.lateral_maneuver_required))
    append_string(message.decision_reason)
    append("<I", message.authorization_failure_mask)
    append("<I", len(message.authorization_failure_reasons))
    for reason in message.authorization_failure_reasons:
        append_string(reason)
    append_string(message.candidate_reject_reason)
    append("<B", int(message.safety_inputs_complete))
    append("<B", int(message.tracking_usable))
    append("<B", int(message.trajectory_publishable))
    append_string(message.constraint_reason)
    append("<B", message.lateral_stop_authority_kind)
    append("<b", message.lateral_stop_transaction_pass_direction)
    append("<Q", message.lateral_stop_authority_token)
    append_string(message.trajectory.header.frame_id)
    append("<I", len(message.trajectory.points))
    append("<B", message.aw2_identity_schema_version)
    append("<Q", message.planner_instance_id)
    append("<Q", message.race_arm_epoch)
    append("<Q", message.connector_transaction_id)
    append("<I", message.candidate_revision)
    wire.extend(bytes(message.candidate_content_sha256))
    return hashlib.sha256(bytes(wire)).digest()


class FreeRunFixtureInputs:
    def __init__(self, node: rclpy.node.Node) -> None:
        self.sequence = 0
        self.clock_ns = 100_000_000_000
        self.clock_pub = node.create_publisher(Clock, "/clock", 10)
        self.odom_pub = node.create_publisher(
            Odometry, "/fixture/kinematics", 10
        )
        self.trajectory_pub = node.create_publisher(
            Trajectory, "/fixture/trajectory", 10
        )
        self.pp_plan_pub = node.create_publisher(
            OvertakePlan, "/fixture/pp/overtake_plan", 10
        )
        self.mux_plan_pub = node.create_publisher(
            OvertakePlan, "/fixture/mux/overtake_plan", 10
        )
        self.constraint_pub = node.create_publisher(
            SafetyConstraint, "/fixture/safety_constraint", 10
        )
        self.override_pub = node.create_publisher(
            Float32MultiArray, "/fixture/reference_override", 10
        )
        self.external_safety_pub = node.create_publisher(
            SafetyStopStatus, "/fixture/external_safety_status", 10
        )
        self.race_arm_pub = node.create_publisher(
            Bool,
            "/fixture/race_armed",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.clock = Clock()
        self.clock.clock.sec = 100
        self.odom = Odometry()
        self.odom.header.frame_id = "map"
        self.odom.header.stamp.sec = 100
        self.odom.pose.pose.orientation.w = 1.0
        self.odom.twist.twist.linear.x = 1.0
        self.trajectory = Trajectory()
        self.trajectory.header.frame_id = "map"
        self.trajectory.header.stamp.sec = 100
        for index in range(40):
            point = TrajectoryPoint()
            point.pose.position.x = 0.25 * index
            point.pose.orientation.w = 1.0
            point.longitudinal_velocity_mps = 2.0
            point.time_from_start = Duration(
                sec=0, nanosec=index * 10_000_000
            )
            self.trajectory.points.append(point)
        self.plan = OvertakePlan()
        self.plan.header.frame_id = "map"
        self.plan.header.stamp.sec = 100
        self.plan.aw2_identity_schema_version = 1
        self.plan.race_arm_epoch = 1
        self.plan.planner_instance_id = 1
        self.plan.plan_generation = 1
        self.plan.free_run_canonical_algorithm_version = (
            OvertakePlan.FREE_RUN_CANONICAL_ALGORITHM_V1
        )
        self.plan.free_run_canonical_payload_sha256 = list(
            free_run_plan_digest(self.plan)
        )
        self.constraint = SafetyConstraint()
        self.constraint.header.frame_id = "map"
        self.constraint.header.stamp.sec = 100
        self.constraint.constraint_generation = 1
        self.constraint.plan_generation = 1
        self.constraint.valid = True
        self.constraint.stop_requested = False
        self.constraint.release_authorized = True
        self.constraint.speed_limit_mps = 3.0
        self.constraint.required_brake_decel_mps2 = 0.0
        self.constraint.reason = "fixture_release"
        self.published_constraint_identities: set[tuple[int, int]] = set()
        self.override = Float32MultiArray()
        self.override.data = [1.0, 0.0, 0.0, 1.0, 1.0]

    def configure_passing_contract(self, *, warmup: bool) -> None:
        """Make the real PP typed schema-2 plan used by production scenario."""
        self.plan.phase = OvertakePlan.PASSING
        self.plan.attempt_id = 7
        self.plan.target_vehicle_id = "fixture_d2"
        self.plan.pass_direction = -1
        self.plan.trajectory_authorized = True
        self.plan.lateral_maneuver_required = True
        self.plan.race_arm_epoch = 1
        self.plan.planner_instance_id = 101
        self.plan.connector_transaction_id = 55
        self.plan.candidate_revision = 3 if warmup else 4
        self.plan.candidate_content_sha256 = [0x5A] * 32
        self.plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_PASS_WARMUP
            if warmup
            else OvertakePlan.LATERAL_STOP_NONE
        )
        self.plan.lateral_stop_transaction_pass_direction = -1 if warmup else 0
        self.plan.lateral_stop_authority_token = 0x700000009 if warmup else 0
        self.plan.trajectory = copy.deepcopy(self.trajectory)
        self.plan.free_run_canonical_payload_sha256 = list(
            free_run_plan_digest(self.plan)
        )
        # v1 lateral-and-speed override:
        # [valid, mode_id, point_count, lateral_m..., speed_mps..., version,
        #  generation]. Keep its generation exactly bound to the plan.
        self.override.data = [
            1.0,
            1.0,
            1.0,
            -0.20,
            2.0,
            1.0,
            float(self.plan.plan_generation),
        ]
        self.constraint.stop_requested = bool(warmup)
        self.constraint.release_authorized = not warmup
        self.constraint.speed_limit_mps = 0.5 if warmup else 3.0
        self.constraint.required_brake_decel_mps2 = 1.0 if warmup else 0.0
        self.constraint.reason = (
            "release_pending_safe_cycles" if warmup else "release_authorized"
        )

    def publish_race_armed(self, armed: bool) -> None:
        message = Bool()
        message.data = bool(armed)
        self.race_arm_pub.publish(message)

    def publish_contracts(
        self, *, trace_steady: bool = False
    ) -> list[dict[str, object]]:
        self.trajectory.header.stamp = self.clock.clock
        self.plan.header.stamp = self.clock.clock
        self.plan.trajectory.header.stamp = self.clock.clock
        self.constraint.header.stamp = self.clock.clock
        self.constraint.constraint_generation += 1
        self.published_constraint_identities.add(
            (
                int(self.constraint.constraint_generation),
                int(self.constraint.header.stamp.sec) * 1_000_000_000
                + int(self.constraint.header.stamp.nanosec),
            )
        )
        if not trace_steady:
            self.trajectory_pub.publish(self.trajectory)
            self.pp_plan_pub.publish(self.plan)
            self.mux_plan_pub.publish(self.plan)
            self.constraint_pub.publish(self.constraint)
            self.override_pub.publish(self.override)
            return []
        publications = (
            ("trajectory", self.trajectory_pub, self.trajectory),
            ("pp_plan", self.pp_plan_pub, self.plan),
            ("mux_plan", self.mux_plan_pub, self.plan),
            ("safety_constraint", self.constraint_pub, self.constraint),
            ("reference_override", self.override_pub, self.override),
        )
        trace: list[dict[str, object]] = []
        for kind, publisher, message in publications:
            started_ns = time.monotonic_ns()
            publisher.publish(message)
            trace.append(
                {
                    "kind": kind,
                    "publish_started_ns": started_ns,
                    "publish_completed_ns": time.monotonic_ns(),
                }
            )
        return trace

    def republish_current_authority_contract(self) -> None:
        """Publish a fresh safety evaluation for one immutable plan tuple."""
        self.constraint.constraint_generation += 1
        self.published_constraint_identities.add(
            (
                int(self.constraint.constraint_generation),
                int(self.constraint.header.stamp.sec) * 1_000_000_000
                + int(self.constraint.header.stamp.nanosec),
            )
        )
        self.pp_plan_pub.publish(self.plan)
        self.mux_plan_pub.publish(self.plan)
        self.constraint_pub.publish(self.constraint)
        self.override_pub.publish(self.override)

    def refresh_current_trajectory(self) -> None:
        """Refresh PP input freshness without mutating the plan tuple."""
        self.trajectory_pub.publish(self.trajectory)

    def publish_dynamic(self, delta_ns: int = 1_000_000) -> None:
        self.sequence += 1
        self.clock_ns += int(delta_ns)
        self.clock.clock.sec = self.clock_ns // 1_000_000_000
        self.clock.clock.nanosec = self.clock_ns % 1_000_000_000
        self.odom.header.stamp = self.clock.clock
        self.clock_pub.publish(self.clock)
        self.odom_pub.publish(self.odom)

    def set_lateral_offset(self, lateral_y_m: float) -> None:
        self.odom.pose.pose.position.y = float(lateral_y_m)

    def publish_mux_only_transition(self, stamp_ns: int) -> None:
        """Publish a bounded same-semantic N -> N+1 Mux contract."""
        self.plan.header.stamp.sec = int(stamp_ns) // 1_000_000_000
        self.plan.header.stamp.nanosec = int(stamp_ns) % 1_000_000_000
        self.plan.plan_generation += 1
        self.constraint.header.stamp = self.plan.header.stamp
        self.constraint.constraint_generation += 1
        self.constraint.plan_generation = self.plan.plan_generation
        self.mux_plan_pub.publish(self.plan)
        self.constraint_pub.publish(self.constraint)

    def publish_external_stop(self) -> None:
        status = SafetyStopStatus()
        status.header.stamp = self.clock.clock
        status.valid = False
        status.stop_requested = True
        status.source = "frl5_fixture"
        self.external_safety_pub.publish(status)


def wait_for_nodes(observer: rclpy.node.Node, processes: list, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for process in processes:
            if process.poll() is not None:
                raise AssertionError(
                    f"runtime node exited early with {process.returncode}"
                )
        names = {name for name, _ in observer.get_node_names_and_namespaces()}
        if EXPECTED_NODES <= names:
            # Node discovery can precede constructor completion.  Wait for the
            # production endpoint set to settle before taking the signature.
            settle_deadline = time.monotonic() + 0.3
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(observer, timeout_sec=0.02)
            return
        rclpy.spin_once(observer, timeout_sec=0.02)
    raise AssertionError("runtime nodes did not appear in graph")


def wait_for_nodes_gone(observer: rclpy.node.Node, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(observer, timeout_sec=0.02)
        names = {name for name, _ in observer.get_node_names_and_namespaces()}
        if EXPECTED_NODES.isdisjoint(names):
            return
    raise AssertionError("runtime nodes remained in graph after shutdown")


def wait_for_outputs(
    observer: rclpy.node.Node, capture: OutputCapture, timeout: float
) -> None:
    required = {
        "/overtake/safety_constraint",
        "/overtake/plan",
        "/debug/overtake/mode",
        "/pp/control_cmd",
        "/pp/raw_control_cmd",
        "/pp/tracking_status",
        "/mux/control_cmd",
        "/mux/tracking_status",
    }
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(observer, timeout_sec=0.02)
        if all(len(capture.current.get(topic, ())) >= 2 for topic in required):
            return
    missing = sorted(
        topic
        for topic in required
        if len(capture.current.get(topic, ())) < 2
    )
    raise AssertionError(f"runtime outputs missing: {missing}")


def run_hash(value: str) -> int:
    result = 1469598103934665603
    for byte in value.encode():
        result ^= byte
        result = (result * 1099511628211) & ((1 << 64) - 1)
    return result


def ring_shape(role: int) -> tuple[int, int]:
    return (56, 4096) if role == 1 else (40, 16384)


def make_ring(role: int, nonce: int, instance: int) -> int:
    record_size, capacity = ring_shape(role)
    mapping_size = HEADER_SIZE + record_size * capacity
    fd = os.memfd_create(
        "c002ay1-graph-parity", os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING
    )
    os.ftruncate(fd, mapping_size)
    header = bytearray(HEADER_SIZE)
    struct.pack_into(
        "<QIIIII",
        header,
        0,
        RING_MAGIC,
        ABI_VERSION,
        LAYOUT_VERSION,
        HEADER_SIZE,
        record_size,
        capacity,
    )
    struct.pack_into("<B", header, 28, role)
    struct.pack_into(
        "<QQQQQ",
        header,
        32,
        mapping_size,
        nonce,
        instance,
        run_hash(RUN_ID),
        HARNESS_READY,
    )
    os.pwrite(fd, header, 0)
    fcntl.fcntl(
        fd,
        fcntl.F_ADD_SEALS,
        fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL,
    )
    return fd


def start_collector(
    root: Path, force_full: bool
) -> tuple[threading.Thread, Path, list, list[int]]:
    socket_path = root / "collector.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    server.bind(str(socket_path))
    server.listen(2)
    errors: list[BaseException] = []
    full_drop_counts: list[int] = []

    def serve() -> None:
        ring_fds: list[int] = []
        try:
            for _ in range(2):
                peer, _ = server.accept()
                with peer:
                    handshake = peer.recv(256)
                    assert len(handshake) == 104
                    role = handshake[12]
                    assert role in (1, 2)
                    nonce, instance = struct.unpack_from("<QQ", handshake, 24)
                    expected = (
                        (PLANNER_NONCE, PLANNER_INSTANCE)
                        if role == 1
                        else (PP_NONCE, PP_INSTANCE)
                    )
                    assert (nonce, instance) == expected
                    fd = make_ring(role, nonce, instance)
                    ring_fds.append(fd)
                    peer.sendmsg(
                        [b"\x01"],
                        [
                            (
                                socket.SOL_SOCKET,
                                socket.SCM_RIGHTS,
                                array("i", [fd]),
                            )
                        ],
                    )
                    if force_full:
                        _, capacity = ring_shape(role)
                        os.pwrite(fd, struct.pack("<Q", capacity), 128)
            if force_full:
                time.sleep(0.15)
                full_drop_counts.extend(
                    struct.unpack("<Q", os.pread(fd, 8, 256))[0]
                    for fd in ring_fds
                )
        except BaseException as error:
            errors.append(error)
        finally:
            for fd in ring_fds:
                os.close(fd)
            server.close()

    thread = threading.Thread(target=serve)
    thread.start()
    return thread, socket_path, errors, full_drop_counts


def run_scenario(
    observer: rclpy.node.Node,
    capture: OutputCapture,
    root: Path,
    scenario: str,
) -> tuple[tuple, tuple, dict, dict[str, int]]:
    enabled = scenario in ("enabled", "unavailable", "full")
    ay0_enabled = scenario in ("ay0_healthy", "ay0_unavailable")
    active_hold_scenario = scenario == "free_run_live_hold_on"
    production_profile_scenario = scenario == "production_profile"
    free_run_live_enabled = scenario in (
        "free_run_live_on",
        "free_run_live_hold_on",
        "production_profile",
    )
    free_run_live_hold_enabled = scenario == "free_run_live_hold_on"
    free_run_live_measure = scenario in (
        "free_run_live_off",
        "free_run_live_on",
        "free_run_live_hold_on",
        "production_profile",
    )
    production_artifact_root = (
        Path(os.environ["C002AY1_PRODUCTION_ARTIFACT_DIR"])
        if production_profile_scenario
        and "C002AY1_PRODUCTION_ARTIFACT_DIR" in os.environ
        else root
    )
    if production_profile_scenario:
        production_artifact_root.mkdir(parents=True, exist_ok=True)
    mux_measurement_path = production_artifact_root / "mux-runtime.json"
    collector_thread = None
    collector_errors: list[BaseException] = []
    full_drop_counts: list[int] = []
    socket_path = root / "unavailable.sock"
    ack_relay = (
        FreeRunAckRelay(observer)
        if active_hold_scenario
        else None
    )
    if scenario in ("enabled", "full"):
        (
            collector_thread,
            socket_path,
            collector_errors,
            full_drop_counts,
        ) = start_collector(root, scenario == "full")

    planner_config = (
        Path(get_package_share_directory("overtake_planner"))
        / "config/overtake_planner.param.yaml"
    )
    mux_config = (
        Path(get_package_share_directory("hybrid_control_mux"))
        / "config/hybrid_control_mux.param.yaml"
    )
    common_observer = [
        "-p",
        f"c002ay1_prod_measure_enabled:={'true' if enabled else 'false'}",
        "-p",
        f"c002ay1_prod_measure_socket_path:={socket_path}",
        "-p",
        f"c002ay1_prod_measure_run_id:={RUN_ID}",
    ]
    planner = start_process(
        command(
            "ros2",
            "run",
            "overtake_planner",
            "overtake_planner_node",
            "--ros-args",
            "--params-file",
            str(planner_config),
            "-p",
            "use_sim_time:=true",
            *common_observer,
            "-p",
            f"c002ay1_prod_measure_session_nonce:={PLANNER_NONCE}",
            "-p",
            f"c002ay1_prod_measure_instance_id:={PLANNER_INSTANCE}",
        )
    )
    pp = start_process(
        command(
            "ros2",
            "run",
            "simple_pure_pursuit",
            "simple_pure_pursuit",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "aw2_shadow_transport_enabled:=false",
            "-p",
            "use_overtake_reference_override:=true",
            "-p",
            (
                "free_run_live_exact_ack_enabled:="
                + ("true" if free_run_live_enabled else "false")
            ),
            *(
                [
                    "-p",
                    "lookahead_gain:=0.0",
                    "-p",
                    "lookahead_min_distance:=1.0",
                    "-p",
                    "curvature_adaptive_lookahead_enabled:=false",
                    "-p",
                    "steering_tire_angle_gain:=1.0",
                ]
                if active_hold_scenario
                else []
            ),
            "-p",
            f"c002ay0_shadow_capture_enabled:={'true' if ay0_enabled else 'false'}",
            "-p",
            (
                "c002ay0_shadow_worker_path:="
                + (
                    str(
                        Path(get_package_prefix("overtake_transport_contract"))
                        / "lib/overtake_transport_contract/c002ay0_shadow_worker"
                    )
                    if scenario == "ay0_healthy"
                    else str(root / "missing-ay0-worker")
                )
            ),
            "-p",
            "c002ay0_shadow_session_generation:=501",
            "-p",
            "c002ay0_shadow_session_nonce:=502",
            *common_observer,
            "-p",
            f"c002ay1_prod_measure_session_nonce:={PP_NONCE}",
            "-p",
            f"c002ay1_prod_measure_instance_id:={PP_INSTANCE}",
            "-r",
            "output/control_cmd:=/pp/control_cmd",
            "-r",
            "output/raw_control_cmd:=/pp/raw_control_cmd",
            "-r",
            "output/controller_tracking_status:=/pp/tracking_status",
            "-r",
            "output/controller_command_envelope:=/pp/command_envelope",
            "-r",
            "output/controller_execution_envelope:=/pp/execution_envelope",
            "-r",
            "output/free_run_execution_ack:=/pp/free_run_execution_ack",
            "-r",
            "output/free_run_source_key:=/pp/free_run_source_key",
            "-r",
            "input/kinematics:=/fixture/kinematics",
            "-r",
            "input/trajectory:=/fixture/trajectory",
            "-r",
            "input/overtake_plan:=/fixture/pp/overtake_plan",
            "-r",
            "input/overtake_reference_override:=/fixture/reference_override",
            "-r",
            "input/race_armed:=/fixture/race_armed",
        )
    )
    mux = start_process(
        command(
            "ros2",
            "run",
            "hybrid_control_mux",
            "hybrid_control_mux_node.py",
            "--ros-args",
            "--params-file",
            str(mux_config),
            *(
                ["-p", "primary_source:=pure_pursuit"]
                if active_hold_scenario or production_profile_scenario
                else []
            ),
            "-p",
            "use_sim_time:=true",
            "-p",
            (
                "require_safety_constraint:="
                + ("true" if free_run_live_measure else "false")
            ),
            *(
                [
                    "-p",
                    "race_arm_required:=false",
                    "-p",
                    "ros_clock_stall_timeout_sec:=10.0",
                    "-p",
                    "control_loop_max_gap_sec:=10.0",
                ]
                if not production_profile_scenario
                else []
            ),
            *(
                [
                    "-p",
                    # The production-default diagnostic keeps normal 4 Hz
                    # debug publication; only the hold fixture needs 100 Hz.
                    "debug_publish_period_sec:="
                    + ("0.01" if active_hold_scenario else "0.25"),
                ]
                if active_hold_scenario or production_profile_scenario
                else []
            ),
            "-p",
            (
                "free_run_live_exact_observe_enabled:="
                + ("true" if free_run_live_enabled else "false")
            ),
            "-p",
            (
                "free_run_live_exact_pre_ack_hold_enabled:="
                + ("true" if free_run_live_hold_enabled else "false")
            ),
            "-p",
            (
                "mux_runtime_measurement_enabled:="
                + ("true" if free_run_live_measure else "false")
            ),
            "-p",
            f"mux_runtime_measurement_output_path:={mux_measurement_path}",
            "-r",
            (
                "input/free_run_execution_ack:="
                + (
                    "/fixture/mux_free_run_execution_ack"
                    if active_hold_scenario
                    else "/pp/free_run_execution_ack"
                )
            ),
            "-r",
            (
                "input/free_run_source_key:="
                + (
                    "/fixture/mux_free_run_source_key"
                    if active_hold_scenario
                    else "/pp/free_run_source_key"
                )
            ),
            "-r",
            (
                "input/pure_pursuit_cmd:="
                + (
                    "/fixture/mux_pp_cmd"
                    if active_hold_scenario
                    else "/pp/control_cmd"
                )
            ),
            "-r",
            (
                "input/pure_pursuit_tracking_status:="
                + (
                    "/fixture/mux_pp_tracking_status"
                    if active_hold_scenario
                    else "/pp/tracking_status"
                )
            ),
            "-r",
            (
                "input/pure_pursuit_command_envelope:="
                + (
                    "/fixture/mux_pp_command_envelope"
                    if active_hold_scenario
                    else "/pp/command_envelope"
                )
            ),
            "-r",
            "input/overtake_plan:=/fixture/mux/overtake_plan",
            "-r",
            "input/safety_constraint:=/fixture/safety_constraint",
            "-r",
            "input/external_safety_status:=/fixture/external_safety_status",
            "-r",
            "input/race_armed:=/fixture/race_armed",
            "-r",
            "output/control_cmd:=/mux/control_cmd",
            "-r",
            "output/controller_tracking_status:=/mux/tracking_status",
            "-r",
            "output/motion_authority_grant:=/mux/motion_authority_grant",
            "-r",
            "output/debug:=/mux/debug",
        )
    )
    processes = [planner, pp, mux]
    fixture_inputs = (
        FreeRunFixtureInputs(observer) if free_run_live_measure else None
    )
    active_evidence: dict[str, object] = {}
    successor_budget: dict[str, object] = {}

    def persist_successor_budget(
        *, status: str, failure: str = ""
    ) -> None:
        if not production_profile_scenario or not successor_budget:
            return
        publish_started_ns = int(
            successor_budget["publish_started_ns"]
        )

        def events_since(events) -> list[dict[str, object]]:
            return [
                dict(event)
                for event in events
                if int(event.get("received_ns", -1)) >= publish_started_ns
            ]

        payload = {
            "schema_version": 1,
            "clock": "CLOCK_MONOTONIC",
            "status": status,
            "failure": failure,
            **successor_budget,
            "observer_events": {
                "pp_command": events_since(capture.pp_command_events),
                "pp_envelope": events_since(capture.pp_envelope_events),
                "pp_ack": events_since(capture.pp_ack_events),
                "pp_tracking": events_since(capture.pp_tracking_events),
                "mux_debug": events_since(capture.mux_debug_events),
                "mux_grant": events_since(capture.mux_grant_events),
                "mux_command": events_since(capture.mux_command_events),
                "mux_tracking": events_since(capture.mux_tracking_events),
            },
        }
        path = production_artifact_root / "successor-budget-fixture.json"
        temporary_path = path.with_suffix(".json.tmp")
        temporary_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n"
        )
        temporary_path.replace(path)

    def authority_summary() -> dict[str, object]:
        """Summarize the fixture-only authority observation on one clock."""
        commit_boundary_ns = int(
            successor_budget.get("publish_completed_ns", 0)
        )
        production = active_evidence.get("production", {})
        revoke = production.get("revoke", {})
        outage_start_ns = int(production.get("outage_started_ns", 0))
        revoke_observed_ns = int(revoke.get("received_ns", 0))
        observation_end_ns = time.monotonic_ns()
        valid_grants = [
            dict(event)
            for event in capture.mux_grant_events
            if int(event["received_ns"]) >= commit_boundary_ns
            and bool(event["valid"])
        ]
        valid_positive_grants = [
            event for event in valid_grants if float(event["command"][0]) > 0.0
        ]
        observed_grants = [
            dict(event)
            for event in capture.mux_grant_events
            if commit_boundary_ns <= int(event["received_ns"])
            <= observation_end_ns
        ]
        post_revoke_valid_grants = [
            event
            for event in observed_grants
            if int(event["received_ns"]) >= revoke_observed_ns
            and bool(event["valid"])
        ]
        nonmonotonic_grant_sequences = sum(
            int(current["sequence"]) <= int(previous["sequence"])
            for previous, current in zip(observed_grants, observed_grants[1:])
        )
        exact_final_matches = [
            (dict(grant_event), dict(command_event))
            for grant_event in valid_positive_grants
            for command_event in capture.mux_command_events
            if (
                command_event["speed_mps"],
                command_event["acceleration_mps2"],
                command_event["steering_rad"],
                command_event["steering_rate_radps"],
            )
            == grant_event["command"]
        ]
        matched_grant_to_command_observer_delta_ns = (
            int(exact_final_matches[0][1]["received_ns"])
            - int(exact_final_matches[0][0]["received_ns"])
            if exact_final_matches
            else None
        )
        outage_commands = [
            dict(event)
            for event in capture.mux_command_events
            if outage_start_ns <= int(event["received_ns"])
            <= observation_end_ns
        ]
        post_revoke_positive_commands = [
            event
            for event in outage_commands
            if int(event["received_ns"]) >= revoke_observed_ns
            and float(event["speed_mps"]) > 0.0
        ]
        outage_stop_observed_ns = int(
            production.get("outage_stop", {}).get("received_ns", 0)
        )
        post_stop_positive_commands = [
            event
            for event in outage_commands
            if int(event["received_ns"]) > outage_stop_observed_ns
            and float(event["speed_mps"]) > 0.0
        ]
        revoke_to_stop_observer_delta_ns = (
            outage_stop_observed_ns - revoke_observed_ns
        )
        return {
            "schema_version": 1,
            "clock": "CLOCK_MONOTONIC",
            "scenario": scenario,
            "diagnostic_kind": "production_default_fixture_diagnostic",
            "commit_boundary_ns": commit_boundary_ns,
            "observation_end_ns": observation_end_ns,
            "valid_grant_count": len(valid_grants),
            "valid_positive_grant_count": len(valid_positive_grants),
            "observed_grant_count": len(observed_grants),
            "post_revoke_valid_grant_count": len(post_revoke_valid_grants),
            "nonmonotonic_grant_sequence_count": nonmonotonic_grant_sequences,
            "exact_final_command_count": len(exact_final_matches),
            "matched_grant_to_command_observer_delta_ns": (
                matched_grant_to_command_observer_delta_ns
            ),
            "matched_grant_to_command_observer_order": (
                "grant_before_or_same_command"
                if matched_grant_to_command_observer_delta_ns is not None
                and matched_grant_to_command_observer_delta_ns >= 0
                else "command_before_grant"
                if matched_grant_to_command_observer_delta_ns is not None
                else "unobserved"
            ),
            "committed_grant": production.get("grant", {}),
            "matched_final_command": production.get(
                "positive_command", {}
            ),
            "revoke": revoke,
            "revoke_observed_ns": revoke_observed_ns,
            "revoke_to_stop_observer_delta_ns": (
                revoke_to_stop_observer_delta_ns
            ),
            "revoke_to_stop_observer_order": (
                "revoke_before_or_same_stop"
                if revoke_to_stop_observer_delta_ns >= 0
                else "stop_before_revoke"
            ),
            "outage_started_ns": outage_start_ns,
            "outage_command_count": len(outage_commands),
            "outage_stop": production.get("outage_stop", {}),
            "outage_stop_observed_ns": outage_stop_observed_ns,
            "post_revoke_positive_command_count": len(
                post_revoke_positive_commands
            ),
            "post_stop_positive_command_count": len(
                post_stop_positive_commands
            ),
            "limitations": {
                "final_ackermann_identity": (
                    "The final Ackermann topic has no independent identity; "
                    "the exact 4-tuple join is diagnostic only."
                ),
                "cross_topic_receive_order": (
                    "CLOCK_MONOTONIC observer receive order across topics is "
                    "not causal publish or actuator ordering."
                ),
                "post_revoke_command_count": (
                    "The post-revoke positive-command count is cross-topic "
                    "diagnostic evidence only; the post-STOP command-topic "
                    "count is the hard fixture gate."
                ),
                "m4_credit": "DIAGNOSTIC_NOT_RACE_ACCEPTANCE",
            },
        }

    def persist_authority_summary(
        summary: dict[str, object], *, status: str, failure: str = ""
    ) -> None:
        if not production_profile_scenario:
            return
        payload = {"status": status, "failure": failure, **summary}
        path = (
            production_artifact_root
            / "c002ay1-production-default-fixture-diagnostic.json"
        )
        temporary_path = path.with_suffix(".json.tmp")
        temporary_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n"
        )
        temporary_path.replace(path)

    if active_hold_scenario:
        fixture_inputs.set_lateral_offset(-0.09)
    try:
        capture.reset()
        wait_for_nodes(observer, processes, 4)
        graph = endpoint_signature(observer)
        if fixture_inputs is not None and not production_profile_scenario:
            fixture_inputs.publish_contracts()
            for _ in range(10):
                fixture_inputs.publish_dynamic()
                rclpy.spin_once(observer, timeout_sec=0.01)
        if not production_profile_scenario:
            wait_for_outputs(observer, capture, 4)
        if production_profile_scenario:
            # Exercise the loaded production watchdog defaults, including the
            # false -> true arm epoch and post-arm progressing ROS clock.
            fixture_inputs.publish_race_armed(False)
            for _ in range(3):
                fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                rclpy.spin_once(observer, timeout_sec=0.01)
            if any(event["valid"] for event in capture.mux_grant_events):
                raise AssertionError("production profile granted while disarmed")
            disarmed_stop = next(
                (
                    event
                    for event in reversed(capture.mux_command_events)
                    if abs(float(event["speed_mps"])) <= 1.0e-9
                    and float(event["acceleration_mps2"]) < 0.0
                ),
                None,
            )
            if disarmed_stop is None:
                raise AssertionError("production profile omitted disarmed STOP")
            fixture_inputs.publish_race_armed(True)
            for _ in range(3):
                fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                rclpy.spin_once(observer, timeout_sec=0.01)

            # Stage A: PP itself must emit the typed PASS_WARMUP schema-2
            # evidence.  The fixture only supplies plan/constraint inputs; it
            # never constructs a grant or controller envelope.
            fixture_inputs.configure_passing_contract(warmup=True)
            fixture_inputs.publish_contracts()
            warmup_deadline = time.monotonic() + 0.50
            next_warmup_contract_publish = time.monotonic() + 0.04
            warmup_stop = None
            while time.monotonic() < warmup_deadline:
                if time.monotonic() >= next_warmup_contract_publish:
                    fixture_inputs.republish_current_authority_contract()
                    next_warmup_contract_publish += 0.04
                fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                rclpy.spin_once(observer, timeout_sec=0.005)
                warmup_stop = next(
                    (
                        event
                        for event in reversed(capture.mux_command_events)
                        if abs(float(event["speed_mps"])) <= 1.0e-9
                        and float(event["acceleration_mps2"]) < 0.0
                    ),
                    warmup_stop,
                )
                warmup_proof = next(
                    (
                        event
                        for event in reversed(capture.mux_tracking_events)
                        if event["pass_probe_exact_current_usable"]
                    ),
                    None,
                )
                if warmup_proof is not None and warmup_stop is not None:
                    break
            if warmup_proof is None or warmup_stop is None:
                raise AssertionError(
                    "production profile did not form real PP PASS_WARMUP "
                    f"evidence tracking={list(capture.mux_tracking_events)[-6:]} "
                    f"commands={list(capture.mux_command_events)[-6:]} "
                    f"pp_tracking={capture.last_pp_tracking_state} "
                    f"pp_envelopes={list(capture.pp_envelope_events)[-6:]} "
                    f"debug={list(capture.mux_debug_events)[-6:]}"
                )

            # Stage B: a direct N -> N+1 successor preserves the transaction,
            # digest, and trajectory, but advances only revision/release.
            warmup_plan = copy.deepcopy(fixture_inputs.plan)
            fixture_inputs.plan.plan_generation += 1
            fixture_inputs.configure_passing_contract(warmup=False)
            fixture_inputs.constraint.plan_generation = fixture_inputs.plan.plan_generation
            if (
                fixture_inputs.plan.connector_transaction_id
                != warmup_plan.connector_transaction_id
                or list(fixture_inputs.plan.candidate_content_sha256)
                != list(warmup_plan.candidate_content_sha256)
                or serialize_message(fixture_inputs.plan.trajectory)
                != serialize_message(warmup_plan.trajectory)
            ):
                raise AssertionError(
                    "production fixture changed immutable successor tuple"
                )
            successor_publications = fixture_inputs.publish_contracts(
                trace_steady=True
            )
            successor_start_ns = int(
                successor_publications[-1]["publish_completed_ns"]
            )
            successor_budget.update(
                {
                    "transition": "pass_warmup_to_passing",
                    "plan_generation": int(
                        fixture_inputs.plan.plan_generation
                    ),
                    "constraint_generation": int(
                        fixture_inputs.constraint.constraint_generation
                    ),
                    "publish_started_ns": int(
                        successor_publications[0]["publish_started_ns"]
                    ),
                    "publish_completed_ns": successor_start_ns,
                    "publications": successor_publications,
                }
            )
            grant = None
            positive_command = None
            successor_deadline = time.monotonic() + 0.50
            while time.monotonic() < successor_deadline:
                fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                rclpy.spin_once(observer, timeout_sec=0.005)
                grant = next(
                    (
                        event
                        for event in reversed(capture.mux_grant_events)
                        if event["received_ns"] >= successor_start_ns
                        and event["valid"]
                    ),
                    grant,
                )
                if grant is not None:
                    positive_command = next(
                        (
                            event
                            for event in reversed(capture.mux_command_events)
                            if event["received_ns"] >= successor_start_ns
                            and float(event["speed_mps"]) > 0.0
                            and (
                                event["speed_mps"],
                                event["acceleration_mps2"],
                                event["steering_rad"],
                                event["steering_rate_radps"],
                            )
                            == grant["command"]
                        ),
                        positive_command,
                    )
                if grant is not None and positive_command is not None:
                    break
            if grant is None or positive_command is None:
                persist_successor_budget(
                    status="authority_not_committed",
                    failure=(
                        "production profile did not commit exact PASS "
                        "successor"
                    ),
                )
                raise AssertionError(
                    "production profile did not commit exact PASS successor "
                    f"grant={grant} command={positive_command} "
                    f"debug={list(capture.mux_debug_events)[-6:]}"
                )
            successor_budget["grant_observed_ns"] = int(
                grant["received_ns"]
            )
            successor_budget["command_observed_ns"] = int(
                positive_command["received_ns"]
            )
            persist_successor_budget(status="authority_committed")
            expected_digest = tuple([0x5A] * 32)
            expected_transaction = (
                int(fixture_inputs.plan.race_arm_epoch),
                int(fixture_inputs.plan.planner_instance_id),
                int(fixture_inputs.plan.attempt_id),
                str(fixture_inputs.plan.target_vehicle_id),
                int(fixture_inputs.plan.pass_direction),
                int(fixture_inputs.plan.connector_transaction_id),
            )
            if (
                grant["warmup_identity"][:6] != expected_transaction
                or grant["plan_identity"][:6] != expected_transaction
                or grant["warmup_identity"][7] + 1 != grant["plan_identity"][7]
                or grant["warmup_identity"][8] >= grant["plan_identity"][8]
                or grant["warmup_identity"][9] != expected_digest
                or grant["plan_identity"][9] != expected_digest
                or grant["warmup_token"] != 0x700000009
                or grant["warmup_pp_identity"][0] != grant["pp_identity"][0]
                or grant["warmup_pp_identity"][1] >= grant["pp_identity"][1]
                or grant["constraint_identity"]
                not in fixture_inputs.published_constraint_identities
            ):
                raise AssertionError(f"production grant tuple mismatch: {grant}")
            if grant["command"] != (
                positive_command["speed_mps"],
                positive_command["acceleration_mps2"],
                positive_command["steering_rad"],
                positive_command["steering_rate_radps"],
            ):
                raise AssertionError(
                    "production grant final-command binding mismatch "
                    f"grant={grant} command={positive_command}"
                )

            # Outage: advance /clock but do not refresh plan/constraint or the
            # real PP exact evidence.  Production freshness must revoke first
            # (or in the same observer cycle) and remain revoked.
            outage_start_ns = time.monotonic_ns()
            revoke = None
            outage_stop = None
            outage_deadline = time.monotonic() + 0.45
            while time.monotonic() < outage_deadline:
                fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                rclpy.spin_once(observer, timeout_sec=0.005)
                revoke = next(
                    (
                        event
                        for event in reversed(capture.mux_grant_events)
                        if event["received_ns"] >= outage_start_ns
                        and not event["valid"]
                    ),
                    revoke,
                )
                outage_stop = next(
                    (
                        event
                        for event in reversed(capture.mux_command_events)
                        if event["received_ns"] >= outage_start_ns
                        and abs(float(event["speed_mps"])) <= 1.0e-9
                        and float(event["acceleration_mps2"]) < 0.0
                    ),
                    outage_stop,
                )
                if revoke is not None and outage_stop is not None:
                    break
            if revoke is None or outage_stop is None:
                raise AssertionError(
                    "production evidence outage did not revoke to STOP "
                    f"revoke={revoke} stop={outage_stop}"
                )
            if int(revoke["sequence"]) <= int(grant["sequence"]):
                raise AssertionError(
                    "production revoke did not advance grant sequence"
                )
            active_evidence["production"] = {
                "disarmed_stop": disarmed_stop,
                "warmup_stop": warmup_stop,
                "grant": grant,
                "positive_command": positive_command,
                "revoke": revoke,
                "outage_stop": outage_stop,
                "outage_started_ns": outage_start_ns,
                "transition_summary": {
                    "warmup_generation": grant["warmup_identity"][7],
                    "successor_generation": grant["plan_identity"][7],
                    "outage_revoke_reason": revoke["reason"],
                    "observer_saw_revoke_before_stop": (
                        int(revoke["received_ns"])
                        <= int(outage_stop["received_ns"])
                    ),
                },
            }
            # Keep the same outage fixture alive long enough to measure the
            # production timer distribution. No authority input is refreshed.
            measurement_deadline = time.monotonic() + 2.2
            while time.monotonic() < measurement_deadline:
                fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                rclpy.spin_once(observer, timeout_sec=0.01)
            summary = authority_summary()
            active_evidence["production"]["authority_summary"] = summary
            failures = []
            if int(summary["valid_positive_grant_count"]) == 0:
                failures.append(
                    "zero valid positive MotionAuthorityGrant observations"
                )
            if int(summary["exact_final_command_count"]) == 0:
                failures.append("zero exact post-commit final-command joins")
            if int(summary["revoke_observed_ns"]) == 0 or int(
                summary["outage_stop_observed_ns"]
            ) == 0:
                failures.append("missing revoke or outage STOP")
            if int(summary["post_stop_positive_command_count"]) != 0:
                failures.append("positive final command after outage STOP")
            if int(summary["post_revoke_valid_grant_count"]) != 0:
                failures.append("valid MotionAuthorityGrant after revoke")
            if int(summary["nonmonotonic_grant_sequence_count"]) != 0:
                failures.append("non-monotonic MotionAuthorityGrant sequence")
            if failures:
                failure = "; ".join(failures)
                persist_authority_summary(
                    summary, status="failed", failure=failure
                )
                raise AssertionError(
                    f"production-default fixture diagnostic: {failure}"
                )
            persist_authority_summary(summary, status="passed")
        elif active_hold_scenario:
            # Publish the source trajectory once.  Re-publishing it while
            # waiting would create a new source generation and make the real
            # ACK/current-source tuple intentionally fail exact matching.
            fixture_inputs.publish_contracts()
            baseline_deadline = time.monotonic() + 0.75
            while time.monotonic() < baseline_deadline:
                fixture_inputs.publish_dynamic(delta_ns=1_000_000)
                rclpy.spin_once(observer, timeout_sec=0.005)
                forwarded_ack = ack_relay.last_forwarded_ack
                forwarded_time_ns = ack_relay.last_forwarded_time_ns
                expected_ack_identity = (
                    (
                        int(forwarded_ack.plan_key.race_arm_epoch),
                        int(forwarded_ack.plan_key.planner_instance_id),
                        int(forwarded_ack.plan_key.plan_generation),
                        FreeRunAckRelay._stamp_ns(
                            forwarded_ack.plan_key.plan_stamp
                        ),
                        int(forwarded_ack.source_key.baseline_instance_id),
                        int(forwarded_ack.source_key.controller_instance_id),
                        int(forwarded_ack.source_key.source_generation),
                        FreeRunAckRelay._stamp_ns(
                            forwarded_ack.source_key.source_stamp
                        ),
                        int(forwarded_ack.controller_sequence),
                        FreeRunAckRelay._stamp_ns(
                            forwarded_ack.controller_command_stamp
                        ),
                    )
                    if forwarded_ack is not None
                    else None
                )
                forwarded_steering = (
                    float(
                        forwarded_ack.output_controller_command.lateral
                        .steering_tire_angle
                    )
                    if forwarded_ack is not None
                    else None
                )
                committed_debug = next(
                    (
                        event
                        for event in reversed(capture.mux_debug_events)
                        if event["record_valid"]
                        and event["record_age_sec"] is not None
                        and forwarded_time_ns is not None
                        and (
                            int(event["received_ns"])
                            - int(float(event["record_age_sec"]) * 1.0e9)
                        )
                        >= forwarded_time_ns - 2_000_000
                        and tuple(event["ack_identity"] or ())
                        == expected_ack_identity
                        and tuple(
                            event["selected_envelope_identity"] or ()
                        )
                        == ack_relay.last_forwarded_envelope_identity
                    ),
                    None,
                )
                if (
                    not ack_relay.forward_enabled
                    and forwarded_steering is not None
                    and committed_debug is not None
                    and any(
                        event["speed_mps"] > 0.0
                        and abs(
                            float(event["steering_rad"])
                            - forwarded_steering
                        )
                        <= 1.0e-6
                        for event in reversed(capture.mux_command_events)
                    )
                ):
                    break
            else:
                raise AssertionError(
                    f"{scenario}: exact baseline transport missing "
                    f"pp={list(capture.pp_command_events)[-4:]} "
                    f"mux={list(capture.mux_command_events)[-4:]} "
                    f"relay_forward={ack_relay.forward_count} "
                    f"relay_ack={ack_relay.last_ack is not None} "
                    f"relay_cache="
                    f"{(len(ack_relay.commands), len(ack_relay.statuses), len(ack_relay.envelopes))} "
                    f"tracking={list(capture.mux_tracking_events)[-4:]} "
                    f"debug={list(capture.mux_debug_events)[-4:]}"
                )
            baseline_ack = ack_relay.last_forwarded_ack
            if (
                baseline_ack is None
                or ack_relay.last_forwarded_time_ns is None
            ):
                raise AssertionError(
                    f"{scenario}: baseline forwarded ACK was not observed"
                )
            baseline_pp_steering = float(
                baseline_ack.output_controller_command.lateral
                .steering_tire_angle
            )
            baseline_command = next(
                (
                    event
                    for event in reversed(capture.mux_command_events)
                    if event["received_ns"]
                    >= ack_relay.last_forwarded_time_ns
                    and event["speed_mps"] > 0.0
                    and abs(
                        float(event["steering_rad"])
                        - baseline_pp_steering
                    )
                    <= 1.0e-6
                ),
                None,
            )
            if baseline_command is None:
                raise AssertionError(
                    f"{scenario}: Mux did not publish the forwarded ACK "
                    f"steering {baseline_pp_steering}; "
                    f"mux={list(capture.mux_command_events)[-8:]} "
                    f"tracking={list(capture.mux_tracking_events)[-4:]} "
                    f"debug={list(capture.mux_debug_events)[-4:]}"
                )

            # Duplicate source keys deliberately do not refresh the production
            # provenance lease.  Transition directly from this exact pair;
            # rearming another ACK for the same source only consumes lease time.
            record_steering = baseline_pp_steering

            old_plan_stamp = ack_relay.last_forwarded_ack.plan_key.plan_stamp
            old_plan_stamp_ns = (
                int(old_plan_stamp.sec) * 1_000_000_000
                + int(old_plan_stamp.nanosec)
            )
            transition_plan_stamp_ns = old_plan_stamp_ns + 50_000_000
            if fixture_inputs.clock_ns < transition_plan_stamp_ns:
                fixture_inputs.publish_dynamic(
                    delta_ns=(
                        transition_plan_stamp_ns - fixture_inputs.clock_ns
                    )
                )
                for _ in range(2):
                    rclpy.spin_once(observer, timeout_sec=0.005)
            # Start the observation window and omit ACK before publishing N+1.
            # Otherwise the first valid pre-ACK hold can occur during the
            # fixture's delivery spins and be discarded as pre-transition.
            ack_relay.freeze()
            transition_ns = time.monotonic_ns()
            committed_source_age_sec = committed_debug["source_age_sec"]
            if committed_source_age_sec is None:
                raise AssertionError(
                    f"{scenario}: committed baseline omitted SourceKey age"
                )
            source_age_at_transition_sec = float(
                committed_source_age_sec
            ) + (
                transition_ns - int(committed_debug["received_ns"])
            ) / 1.0e9
            if source_age_at_transition_sec > 0.045:
                raise AssertionError(
                    f"{scenario}: fixture exhausted the non-refreshable "
                    f"SourceKey transition budget "
                    f"source_age_sec={source_age_at_transition_sec:.6f}"
                )
            fixture_inputs.publish_mux_only_transition(
                stamp_ns=transition_plan_stamp_ns
            )
            fixture_inputs.publish_dynamic(delta_ns=1_000_000)
            # Keep the entire observed SourceKey age below 95 ms, leaving at
            # least 25 ms before the production 120 ms fail-closed deadline.
            transition_deadline = time.monotonic() + 0.050
            pp_transition_command = None
            hold_command = None
            hold_status = None
            hold_debug = None
            zero_stop_command = None
            while time.monotonic() < transition_deadline:
                fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                rclpy.spin_once(observer, timeout_sec=0.005)
                pp_transition_command = next(
                    (
                        event
                        for event in reversed(capture.pp_command_events)
                        if event["received_ns"] >= transition_ns
                        and math.isfinite(float(event["steering_rad"]))
                        and abs(float(event["steering_rad"]))
                        <= 0.64
                    ),
                    pp_transition_command,
                )
                hold_command = next(
                    (
                        event
                        for event in reversed(capture.mux_command_events)
                        if event["received_ns"] >= transition_ns
                        and abs(float(event["speed_mps"])) <= 1.0e-9
                        and float(event["acceleration_mps2"]) < 0.0
                        and abs(
                            float(event["steering_rad"]) - record_steering
                        )
                        <= 1.0e-9
                    ),
                    hold_command,
                )
                zero_stop_command = next(
                    (
                        event
                        for event in reversed(capture.mux_command_events)
                        if event["received_ns"] >= transition_ns
                        and abs(float(event["speed_mps"])) <= 1.0e-9
                        and float(event["acceleration_mps2"]) < 0.0
                        and abs(float(event["steering_rad"])) <= 1.0e-9
                    ),
                    zero_stop_command,
                )
                hold_status = next(
                    (
                        event
                        for event in reversed(capture.mux_tracking_events)
                        if event["received_ns"] >= transition_ns
                        and event["reason"]
                        == "free_run_live_exact_pre_ack_hold"
                        and not event["usable"]
                    ),
                    hold_status,
                )
                hold_debug = next(
                    (
                        event
                        for event in reversed(capture.mux_debug_events)
                        if event["received_ns"] >= transition_ns
                        and event["join_reason"]
                        == "forward_pre_ack_transition"
                        and event["record_valid"]
                    ),
                    hold_debug,
                )
                if pp_transition_command is not None and (
                    (
                        free_run_live_hold_enabled
                        and hold_command is not None
                        and hold_status is not None
                    )
                    or (
                        not free_run_live_hold_enabled
                        and zero_stop_command is not None
                    )
                ):
                    break
            if pp_transition_command is None:
                raise AssertionError(
                    f"{scenario}: PP did not produce a bounded transition "
                    f"steering; events={list(capture.pp_command_events)[-8:]}"
                )
            if free_run_live_hold_enabled:
                if any(
                    evidence is None
                    for evidence in (hold_command, hold_status)
                ):
                    raise AssertionError(
                        f"{scenario}: active hold missing command={hold_command} "
                        f"status={hold_status} debug={hold_debug} "
                        f"last_debug={list(capture.mux_debug_events)[-6:]} "
                        f"last_tracking={list(capture.mux_tracking_events)[-6:]} "
                        f"last_ack={capture.last_ack_state}"
                    )
                if any(
                    event["received_ns"] >= transition_ns
                    and event["valid"]
                    for event in capture.mux_grant_events
                ):
                    raise AssertionError(
                        f"{scenario}: active hold published motion authority "
                        f"{list(capture.mux_grant_events)[-6:]}"
                    )
                barrier_ns = time.monotonic_ns()
                fixture_inputs.publish_external_stop()
                barrier_deadline = time.monotonic() + 0.20
                barrier_command = None
                barrier_debug = None
                while time.monotonic() < barrier_deadline:
                    fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                    rclpy.spin_once(observer, timeout_sec=0.005)
                    barrier_command = next(
                        (
                            event
                            for event in reversed(capture.mux_command_events)
                            if event["received_ns"] >= barrier_ns
                            and abs(float(event["speed_mps"])) <= 1.0e-9
                            and float(event["acceleration_mps2"]) < 0.0
                            and abs(float(event["steering_rad"])) <= 1.0e-9
                        ),
                        barrier_command,
                    )
                    barrier_debug = next(
                        (
                            event
                            for event in reversed(capture.mux_debug_events)
                            if event["received_ns"] >= barrier_ns
                            and not event["record_valid"]
                            and event["external_stop_latched"]
                        ),
                        barrier_debug,
                    )
                    if barrier_command is not None:
                        break
                if barrier_command is None:
                    raise AssertionError(
                        f"{scenario}: E-stop did not publish zero-steer STOP "
                        f"command={barrier_command} debug={barrier_debug} "
                        f"last_debug={list(capture.mux_debug_events)[-8:]}"
                    )
                if any(
                    event["received_ns"] >= barrier_ns and event["valid"]
                    for event in capture.mux_grant_events
                ):
                    raise AssertionError(
                        f"{scenario}: E-stop published motion authority "
                        f"{list(capture.mux_grant_events)[-6:]}"
                    )
                active_evidence["barrier"] = {
                    "kind": "external_safety_stop",
                    "command": barrier_command,
                    "debug": barrier_debug,
                }
            else:
                if zero_stop_command is None or hold_status is not None:
                    raise AssertionError(
                        f"{scenario}: hold-OFF did not remain zero-steer STOP "
                        f"zero={zero_stop_command} hold_status={hold_status} "
                        f"commands={list(capture.mux_command_events)[-6:]} "
                        f"tracking={list(capture.mux_tracking_events)[-6:]} "
                        f"debug={list(capture.mux_debug_events)[-6:]}"
                    )
                stale_deadline = time.monotonic() + 0.20
                stale_debug = None
                while time.monotonic() < stale_deadline:
                    fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                    rclpy.spin_once(observer, timeout_sec=0.005)
                    stale_debug = next(
                        (
                            event
                            for event in reversed(capture.mux_debug_events)
                            if event["received_ns"] >= transition_ns
                            and not event["record_valid"]
                            and event["record_reason"]
                            == "live_record_evidence_expired_or_faulted"
                        ),
                        stale_debug,
                    )
                    if stale_debug is not None:
                        break
                if stale_debug is None:
                    raise AssertionError(
                        f"{scenario}: stale source/record did not fail closed"
                    )
                active_evidence["barrier"] = {
                    "kind": "stale_source_or_record",
                    "command": zero_stop_command,
                    "debug": stale_debug,
                }
            active_evidence.update(
                {
                    "baseline_command": baseline_command,
                    "record_steering_rad": record_steering,
                    "pp_transition_command": pp_transition_command,
                    "hold_command": hold_command,
                    "hold_status": hold_status,
                    "hold_debug": hold_debug,
                }
            )
            measurement_deadline = time.monotonic() + 2.5
            while time.monotonic() < measurement_deadline:
                fixture_inputs.publish_dynamic(delta_ns=10_000_000)
                rclpy.spin_once(observer, timeout_sec=0.02)
        elif free_run_live_measure:
            measurement_deadline = time.monotonic() + 2.5
            next_trajectory_publish = time.monotonic()
            while time.monotonic() < measurement_deadline:
                if time.monotonic() >= next_trajectory_publish:
                    fixture_inputs.publish_contracts()
                    next_trajectory_publish += 0.05
                fixture_inputs.publish_dynamic()
                rclpy.spin_once(observer, timeout_sec=0.02)
        if production_profile_scenario:
            artifact_root = Path(
                os.environ.get("C002AY1_PRODUCTION_ARTIFACT_DIR", str(root))
            )
            artifact_root.mkdir(parents=True, exist_ok=True)
            ledger = []
            production = active_evidence.get("production", {})
            warmup_boundary_ns = int(
                production.get("warmup_stop", {}).get("received_ns", 0)
            )
            pass_boundary_ns = int(
                production.get("grant", {}).get("received_ns", 0)
            )
            outage_boundary_ns = int(
                production.get("outage_started_ns", 0)
            )
            stop_boundary_ns = min(
                int(production.get("revoke", {}).get("received_ns", 0)),
                int(
                    production.get("outage_stop", {}).get(
                        "received_ns", 0
                    )
                ),
            )
            for kind, events in (
                ("debug", capture.mux_debug_events),
                ("grant", capture.mux_grant_events),
                ("command", capture.mux_command_events),
                ("tracking", capture.mux_tracking_events),
            ):
                for event in events:
                    received_ns = int(event["received_ns"])
                    if received_ns < warmup_boundary_ns:
                        phase = "pre_arm"
                    elif received_ns < pass_boundary_ns:
                        phase = "warmup_stop"
                    elif received_ns < stop_boundary_ns:
                        phase = "pass"
                    else:
                        phase = "outage_stop"
                    item = {
                        "kind": kind,
                        "fixture_phase": phase,
                        "input_outage": received_ns >= outage_boundary_ns,
                        **event,
                    }
                    if kind == "debug":
                        item["exact_available"] = (
                            event.get("motion_first_false") == "none"
                        )
                    ledger.append(item)
            ledger.sort(key=lambda event: int(event["received_ns"]))
            exact_availability = {}
            for phase in (
                "pre_arm",
                "warmup_stop",
                "pass",
                "outage_stop",
            ):
                cycles = [
                    event
                    for event in ledger
                    if event["kind"] == "debug"
                    and event["fixture_phase"] == phase
                ]
                exact_availability[phase] = {
                    "cycles": len(cycles),
                    "available_cycles": sum(
                        bool(event["exact_available"]) for event in cycles
                    ),
                }
            (artifact_root / "c002ay1-production-profile-ledger.json").write_text(
                json.dumps(
                    {
                        "scenario": scenario,
                        "exact_availability": exact_availability,
                        "transition_summary": active_evidence.get(
                            "production", {}
                        ).get("transition_summary", {}),
                        "events": ledger,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n"
            )
        outputs = capture.signature()
    finally:
        for process in reversed(processes):
            stop_process(process)
        if collector_thread is not None:
            collector_thread.join(timeout=2)
            if collector_thread.is_alive():
                raise AssertionError("collector thread did not terminate")
            if collector_errors:
                raise AssertionError(f"collector errors: {collector_errors}")
            if scenario == "full" and (
                len(full_drop_counts) != 2
                or any(count == 0 for count in full_drop_counts)
            ):
                raise AssertionError(
                    "full scenario did not observe producer drops: "
                    f"{full_drop_counts}"
                )
        wait_for_nodes_gone(observer, 2)
    measurement = {}
    if free_run_live_measure:
        if not mux_measurement_path.exists():
            raise AssertionError(
                f"{scenario}: Mux runtime measurement missing"
            )
        measurement = json.loads(mux_measurement_path.read_text())
        if production_profile_scenario:
            artifact_root = Path(
                os.environ.get("C002AY1_PRODUCTION_ARTIFACT_DIR", str(root))
            )
            artifact_root.mkdir(parents=True, exist_ok=True)
            (artifact_root / "mux-runtime.json").write_text(
                json.dumps(measurement, separators=(",", ":")) + "\n"
            )
    runtime_evidence = dict(capture.evidence_counts)
    runtime_evidence["exact_join"] = capture.mux_exact_join_count
    runtime_evidence["record"] = capture.mux_record_count
    runtime_evidence["reasons"] = dict(capture.mux_last_live_reasons)
    runtime_evidence["ack_state"] = dict(capture.last_ack_state)
    runtime_evidence["pp_tracking"] = dict(capture.last_pp_tracking_state)
    runtime_evidence["ack_forward_count"] = (
        ack_relay.forward_count if ack_relay is not None else 0
    )
    runtime_evidence["active_hold"] = active_evidence
    return graph, outputs, measurement, runtime_evidence


def main() -> int:
    rclpy.init()
    observer = rclpy.create_node("c002ay1_runtime_parity_observer")
    capture = OutputCapture(observer)
    try:
        results = {}
        production_only = (
            os.environ.get("C002AY1_PRODUCTION_ONLY", "0") == "1"
        )
        scenarios = (
            ("production_profile",)
            if production_only
            else (
                "disabled",
                "enabled",
                "unavailable",
                "full",
                "ay0_healthy",
                "ay0_unavailable",
                "free_run_live_off",
                "free_run_live_on",
                "free_run_live_hold_on",
                "production_profile",
            )
        )
        with tempfile.TemporaryDirectory(prefix="c002ay1-parity-") as temp:
            root = Path(temp)
            for scenario in scenarios:
                scenario_root = root / scenario
                scenario_root.mkdir()
                results[scenario] = run_scenario(
                    observer,
                    capture,
                    scenario_root,
                    scenario,
                )
        if production_only:
            measurement = results["production_profile"][2]
            records = measurement.get("records", [])
            drops = int(measurement.get("drops", -1))
            if len(records) < 100:
                raise AssertionError(
                    "production_profile: too few Mux timer records "
                    f"({len(records)})"
                )
            durations = [int(record[2]) for record in records]
            starts = [int(record[1]) for record in records]
            gaps = [
                current - previous
                for previous, current in zip(starts, starts[1:])
                if current >= previous
            ]
            p99 = sorted(durations)[
                min(len(durations) - 1, len(durations) * 99 // 100)
            ]
            print(
                f"production_profile samples={len(records)} "
                f"mux_callback_p99_ns={p99} "
                f"mux_callback_max_ns={max(durations)} "
                f"mux_timer_gap_max_ns={max(gaps)} drops={drops}"
            )
            if (
                p99 > 10_000_000
                or max(durations) > 100_000_000
                or max(gaps) > 100_000_000
                or drops != 0
            ):
                raise AssertionError(
                    "production_profile: runtime timing hard gate failed"
                )
            return 0
        baseline_graph, baseline_outputs, _, _ = results["disabled"]
        for scenario, (graph, outputs, _, _) in results.items():
            if scenario.startswith("free_run_live") or scenario == "production_profile":
                continue
            if graph != baseline_graph:
                baseline_by_node = {
                    entry[0]: entry[1:] for entry in baseline_graph
                }
                actual_by_node = {entry[0]: entry[1:] for entry in graph}
                changed_graph = {
                    node: (
                        baseline_by_node.get(node),
                        actual_by_node.get(node),
                    )
                    for node in sorted(
                        set(baseline_by_node) | set(actual_by_node)
                    )
                    if baseline_by_node.get(node) != actual_by_node.get(node)
                }
                raise AssertionError(
                    f"{scenario}: graph mismatch {changed_graph}"
                )
            if outputs != baseline_outputs:
                baseline_by_topic = dict(baseline_outputs)
                actual_by_topic = dict(outputs)
                changed = {
                    topic: (
                        baseline_by_topic.get(topic),
                        actual_by_topic.get(topic),
                    )
                    for topic in sorted(
                        set(baseline_by_topic) | set(actual_by_topic)
                    )
                    if baseline_by_topic.get(topic) != actual_by_topic.get(topic)
                }
                raise AssertionError(f"{scenario}: output mismatch {changed}")
        off_outputs = results["free_run_live_off"][1]
        on_outputs = results["free_run_live_on"][1]
        if off_outputs != on_outputs:
            raise AssertionError("Mux FREE_RUN live OFF/ON final output mismatch")
        off_evidence = results["free_run_live_off"][3]
        if any(
            off_evidence[key] for key in ("ack", "source", "exact_join", "record")
        ):
            raise AssertionError(
                "Mux FREE_RUN live OFF emitted exact evidence: "
                f"{results['free_run_live_off'][3]}"
            )
        if any(
            results["free_run_live_on"][3][key] <= 0
            for key in ("ack", "source", "exact_join", "record")
        ):
            raise AssertionError(
                "Mux FREE_RUN live ON exact evidence missing: "
                f"{results['free_run_live_on'][3]}"
            )
        hold_on = results["free_run_live_hold_on"][3]["active_hold"]
        if (
            hold_on["hold_command"] is None
            or hold_on["hold_status"] is None
        ):
            raise AssertionError(
                f"hold-ON allowed transition incomplete: {hold_on}"
            )
        if hold_on["barrier"]["kind"] != "external_safety_stop":
            raise AssertionError("hold-ON E-stop barrier was not observed")
        for scenario in (
            "free_run_live_off",
            "free_run_live_on",
            "free_run_live_hold_on",
            "production_profile",
        ):
            measurement = results[scenario][2]
            records = measurement.get("records", [])
            drops = int(measurement.get("drops", -1))
            if len(records) < 100:
                raise AssertionError(
                    f"{scenario}: too few Mux timer records ({len(records)})"
                )
            durations = [int(record[2]) for record in records]
            starts = [int(record[1]) for record in records]
            gaps = [
                current - previous
                for previous, current in zip(starts, starts[1:])
                if current >= previous
            ]
            p999 = sorted(durations)[
                min(len(durations) - 1, len(durations) * 999 // 1000)
            ]
            p99 = sorted(durations)[
                min(len(durations) - 1, len(durations) * 99 // 100)
            ]
            p95 = sorted(durations)[
                min(len(durations) - 1, len(durations) * 95 // 100)
            ]
            print(
                f"{scenario} samples={len(records)} "
                f"mux_callback_p95_ns={p95} "
                f"mux_callback_p99_ns={p99} "
                f"mux_callback_p999_ns={p999} "
                f"mux_callback_max_ns={max(durations)} "
                f"mux_timer_gap_max_ns={max(gaps)} drops={drops} "
                f"evidence={results[scenario][3]}"
            )
            if max(gaps) > 100_000_000:
                raise AssertionError(
                    f"{scenario}: Mux timer gap exceeds 100 ms"
                )
            if max(durations) > 100_000_000:
                raise AssertionError(
                    f"{scenario}: Mux callback exceeds 100 ms watchdog budget"
                )
            if p99 > 10_000_000:
                raise AssertionError(
                    f"{scenario}: Mux callback p99 exceeds 10 ms budget"
                )
            if drops != 0:
                raise AssertionError(
                    f"{scenario}: Mux runtime measurement dropped records"
                )
    finally:
        observer.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
