#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import secrets
import struct
import time
from collections import deque
from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Optional

import rclpy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from multi_purpose_mpc_ros_msgs.msg import (
    ControllerCommandEnvelope,
    ControllerTrackingStatus,
    FreeRunExecutionAck,
    FreeRunPlanKey,
    FreeRunSourceKey,
    MotionAuthorityGrant,
    OvertakePlan,
    RecoveryControlCommand,
    RecoveryPermit,
    RecoveryStatus,
    SafetyConstraint,
    SafetyStopStatus,
    StateLatticeControlCommand,
)
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import Bool, String

from hybrid_control_mux.core import (
    ControlLoopWatchdog,
    HybridMuxConfig,
    HybridMuxCore,
    MpcHealth,
    RaceFinishStopLatch,
    RecoveryMuxState,
    RosClockProgressWatchdog,
    RosClockProgressWatchdogResult,
    SafetyConstraintAuthority,
    SafetyConstraintDecision,
    SafetyConstraintState,
    SteeringLimitResult,
    SteeringLimiter,
    SteeringLimiterConfig,
    StateLatticeMuxState,
    apply_safety_constraint,
)


@dataclass(frozen=True)
class PurePursuitMotionSample:
    """Immutable PP source snapshot with separate authority/observed roles."""

    command: AckermannControlCommand
    command_stamp_ns: int
    command_receipt_time_sec: float
    authority_proof: Optional[ControllerTrackingStatus]
    authority_proof_receipt_time_sec: Optional[float]
    authority_proof_valid: bool
    authority_proof_identity: Optional[tuple[int, int]]
    observed_proof: Optional[ControllerTrackingStatus]
    observed_proof_receipt_time_sec: Optional[float]
    observed_proof_valid: bool
    envelope_identity: Optional[tuple[int, int, int, int]] = None


@dataclass(frozen=True)
class FreeRunPublishedCommandRecord:
    """Immutable provenance for one exact, already-published FREE_RUN command."""

    publish_steady_time_sec: float
    race_arm_epoch: int
    planner_instance_id: int
    plan_generation: int
    plan_stamp_ns: int
    canonical_plan_sha256: bytes
    source_identity: tuple[int, int, int, int]
    source_semantic_identity: tuple
    source_wire_sha256: bytes
    envelope_identity: tuple[int, int, int, int]
    envelope_command_sha256: bytes
    ack_identity: tuple[int, ...]
    ack_wire_sha256: bytes
    published_steering_rad: float
    tracking_soft_limit_rad: float
    actuator_hard_limit_rad: float
    final_command_cdr: bytes
    final_command_sha256: bytes


@dataclass(frozen=True)
class FreeRunSourceGapLease:
    """One bounded, zero-speed-only bridge to a direct SourceKey successor."""

    provenance_record: FreeRunPublishedCommandRecord
    successor_source_identity: tuple[int, int, int, int]
    acquired_steady_time_sec: float


@dataclass(frozen=True)
class MotionAuthorityDeliveryGapLease:
    """One committed PASSING cohort with a non-renewable gap hold."""

    plan_generation: int
    plan_identity: tuple
    constraint: SafetyConstraintState
    envelope: ControllerCommandEnvelope
    final_command: AckermannControlCommand
    acquired_steady_time_sec: float
    steering_result: Optional[SteeringLimitResult] = None
    # Preserve the original N receipt leases.  A delivery-gap acquisition is
    # not a new receipt for any member of the committed cohort.
    plan_receipt_time_sec: Optional[float] = None
    constraint_receipt_time_sec: Optional[float] = None
    tracking_receipt_time_sec: Optional[float] = None
    envelope_receipt_time_sec: Optional[float] = None
    command_receipt_time_sec: Optional[float] = None
    plan_header_stamp_ns: Optional[int] = None
    constraint_header_stamp_ns: Optional[int] = None
    tracking_header_stamp_ns: Optional[int] = None
    envelope_header_stamp_ns: Optional[int] = None
    command_header_stamp_ns: Optional[int] = None
    plan_lateral_stop_fingerprint: Optional[tuple] = None
    constraint_payload_fingerprint: Optional[tuple] = None
    # The committed N snapshot may exist for longer than the delivery gap.
    # Start the bounded hold only when direct N+1 partial delivery is first
    # observed; timer ticks and retransmissions must never renew this value.
    gap_started_steady_time_sec: Optional[float] = None
    # A valid direct N+1 Plan may describe newly replanned geometry. Freeze
    # that successor identity separately from the immutable committed N
    # cohort; it is binding state only and never grants authority.
    successor_plan_identity: Optional[tuple] = None
    successor_plan_lateral_stop_fingerprint: Optional[tuple] = None
    successor_envelope_preview_identity: Optional[tuple] = None


class DeliveryGapRevokeReason(IntEnum):
    """Stable diagnostic codes; these never participate in authority."""

    UNSPECIFIED = 0
    HIGHER_PRIORITY_STOP = 201
    ROS_CLOCK_STALLED = 203
    CONTROL_DEADLINE_MISSED = 204
    RACE_NOT_ARMED = 205
    PP_ENVELOPE_FAULT_LATCHED = 207
    SUCCESSOR_NOT_DIRECT = 102
    GAP_CLOCK_INVALID = 103
    GAP_TIMEOUT = 104
    N_INPUT_STALE = 300
    N_PLAN_STALE = 301
    N_CONSTRAINT_STALE = 302
    N_TRACKING_STALE = 303
    N_ENVELOPE_STALE = 304
    N_COMMAND_STALE = 305
    N_ENVELOPE_COMMAND_AGE_INVALID = 306
    N_STAMP_CHANGED = 307
    N_PLAN_IDENTITY_CHANGED = 313
    N_LATERAL_STOP_FINGERPRINT_CHANGED = 314
    N_CONSTRAINT_PAYLOAD_MUTATED = 315
    N1_PLAN_MISSING_OR_INVALID = 401
    N1_PLAN_INVALID = 402
    N1_PLAN_UNAUTHORIZED = 403
    N1_PLAN_PHASE_INVALID = 406
    N1_PLAN_STAMP_NOT_FORWARD = 407
    N1_IDENTITY_INVALID = 405
    N1_PLAN_RELATION_INVALID = 408
    N1_CONSTRAINT_MISSING_AFTER_OBSERVED = 501
    N1_CONSTRAINT_CACHE_MISS = 502
    N1_CONSTRAINT_INVALID = 503
    N1_CONSTRAINT_STOP_REQUESTED = 504
    N1_CONSTRAINT_RELEASE_WITHDRAWN = 505
    N1_CONSTRAINT_STALE = 506
    N1_CONSTRAINT_FRAME_OR_VALUE_INVALID = 507
    N1_CONSTRAINT_STAMP_MISMATCH = 508
    N1_CONSTRAINT_PAYLOAD_MUTATED = 509
    LATEST_CONSTRAINT_GENERATION_UNEXPECTED = 510
    N1_ENVELOPE_IDENTITY_MISMATCH = 601
    N1_ENVELOPE_SCHEMA_INVALID = 602
    N1_ENVELOPE_GENERATION_MISMATCH = 603
    N1_ENVELOPE_RELATION_INVALID = 604
    N1_ENVELOPE_PLAN_BINDING_MISMATCH = 605
    N1_ENVELOPE_UNUSABLE_OR_STALE = 606
    RENDEZVOUS_NOT_HOLDABLE = 704
    PLAN_CONSTRAINT_UNPAIRED = 705
    LIMITER_STATE_MISSING = 801
    LIMITER_SOURCE_MISMATCH = 802
    FINAL_STEERING_NONFINITE = 803
    LIMITER_FINAL_STEERING_MISMATCH = 804
    COMMITTED_STEERING_BINDING_INVALID = 805


class DeliveryGapPlanRelationSubreason(IntEnum):
    """Diagnostic-only detail for the composite DG408 predicate."""

    IDENTITY_PREFIX_MISMATCH = 1
    EXPECTED_CONSTRAINT_STAMP_MISMATCH = 2
    CANDIDATE_REVISION_MISMATCH = 3
    CANDIDATE_CONTENT_DIGEST_MISMATCH = 4
    AW2_IDENTITY_SCHEMA_INVALID = 5
    LATERAL_STOP_FINGERPRINT_MISSING = 6
    LATERAL_STOP_FINGERPRINT_MISMATCH = 7


@dataclass(frozen=True)
class DeliveryGapRevokeRecord:
    """First causal revoke observed before the one-shot STOP is consumed."""

    reason: DeliveryGapRevokeReason
    origin: str
    lease_generation: int
    observed_generation: Optional[int]
    gap_age_sec: Optional[float]
    plan_relation_subreason: Optional[DeliveryGapPlanRelationSubreason] = None


@dataclass(frozen=True)
class PendingFreeRunSource:
    """Observed SourceKey successor that has not received execution authority."""

    source_key: FreeRunSourceKey
    identity: tuple[int, int, int, int]
    semantic_identity: tuple
    wire_sha256: bytes
    receipt_steady_time_sec: float


class SafetyAuthorityRendezvousState(IntEnum):
    """Typed plan/constraint delivery state; never grants motion by itself."""

    UNKNOWN = 0
    EXACT_CURRENT = 1
    NORMAL_DELIVERY_GAP = 2
    STALE = 3
    PAYLOAD_MUTATION = 4
    REGRESSION_OR_GAP = 5


@dataclass(frozen=True)
class SafetyAuthoritySelection:
    constraint: Optional[SafetyConstraintState]
    receipt_time_sec: Optional[float]
    authority_plan_generation: Optional[int]
    rendezvous_state: SafetyAuthorityRendezvousState

    def __iter__(self):
        """Keep legacy test unpacking while exposing the typed state."""
        yield self.constraint
        yield self.receipt_time_sec
        yield self.authority_plan_generation


def retain_warmup_proof_in_stop_state(
    *, exact_pass_warmup_stop: bool,
    rendezvous_state: SafetyAuthorityRendezvousState,
) -> bool:
    """Keep Stage-A only for its exact STOP or a bounded topic rendezvous."""
    return bool(
        exact_pass_warmup_stop
        or rendezvous_state == SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
    )

class HybridControlMuxNode(Node):
    def __init__(self, **node_kwargs: object) -> None:
        super().__init__("hybrid_control_mux_node", **node_kwargs)

        self.enabled = self.declare_parameter("enabled", True).value
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 50.0).value)
        self.mpc_cmd_timeout_sec = float(
            self.declare_parameter("mpc_cmd_timeout_sec", 0.20).value
        )
        self.pure_pursuit_cmd_timeout_sec = float(
            self.declare_parameter("pure_pursuit_cmd_timeout_sec", 0.20).value
        )
        self.pure_pursuit_tracking_status_timeout_sec = float(
            self.declare_parameter(
                "pure_pursuit_tracking_status_timeout_sec", 0.20
            ).value
        )
        self.state_lattice_instant_control_enabled = bool(
            self.declare_parameter(
                "state_lattice_instant_control_enabled", False
            ).value
        )
        self.state_lattice_cmd_timeout_sec = float(
            self.declare_parameter(
                "state_lattice_cmd_timeout_sec", 0.12
            ).value
        )
        if (
            not math.isfinite(self.state_lattice_cmd_timeout_sec)
            or not 0.02 <= self.state_lattice_cmd_timeout_sec <= 0.20
        ):
            raise ValueError(
                "state_lattice_cmd_timeout_sec must be finite and in "
                "[0.02, 0.20]"
            )
        self.state_lattice_max_steering_angle_rad = float(
            self.declare_parameter(
                "state_lattice_max_steering_angle_rad", 0.5236
            ).value
        )
        self.state_lattice_max_steering_rate_radps = float(
            self.declare_parameter(
                "state_lattice_max_steering_rate_radps", 0.35
            ).value
        )
        if (
            not math.isfinite(self.state_lattice_max_steering_angle_rad)
            or not 0.0 < self.state_lattice_max_steering_angle_rad <= 0.64
        ):
            raise ValueError(
                "state_lattice_max_steering_angle_rad must be finite and in "
                "(0.0, 0.64]"
            )
        if (
            not math.isfinite(self.state_lattice_max_steering_rate_radps)
            or self.state_lattice_max_steering_rate_radps <= 0.0
        ):
            raise ValueError(
                "state_lattice_max_steering_rate_radps must be finite and positive"
            )
        self.pure_pursuit_envelope_timeout_sec = float(
            self.declare_parameter(
                "pure_pursuit_envelope_timeout_sec",
                self.pure_pursuit_tracking_status_timeout_sec,
            ).value
        )
        if (
            not math.isfinite(self.pure_pursuit_envelope_timeout_sec)
            or self.pure_pursuit_envelope_timeout_sec <= 0.0
        ):
            raise ValueError(
                "pure_pursuit_envelope_timeout_sec must be finite and positive"
            )
        self.pure_pursuit_envelope_future_stamp_tolerance_sec = float(
            self.declare_parameter(
                "pure_pursuit_envelope_future_stamp_tolerance_sec", 0.05
            ).value
        )
        if (
            not math.isfinite(
                self.pure_pursuit_envelope_future_stamp_tolerance_sec
            )
            or self.pure_pursuit_envelope_future_stamp_tolerance_sec < 0.0
        ):
            raise ValueError(
                "pure_pursuit_envelope_future_stamp_tolerance_sec must be finite "
                "and nonnegative"
            )
        self.free_run_live_exact_observe_enabled = bool(
            self.declare_parameter(
                "free_run_live_exact_observe_enabled", False
            ).value
        )
        self.free_run_live_exact_pre_ack_hold_enabled = bool(
            self.declare_parameter(
                "free_run_live_exact_pre_ack_hold_enabled", False
            ).value
        )
        if (
            self.free_run_live_exact_pre_ack_hold_enabled
            and not self.free_run_live_exact_observe_enabled
        ):
            raise ValueError(
                "free_run_live_exact_pre_ack_hold_enabled requires "
                "free_run_live_exact_observe_enabled"
            )
        self.mux_runtime_measurement_enabled = bool(
            self.declare_parameter(
                "mux_runtime_measurement_enabled", False
            ).value
        )
        self.latest_sample_observability_enabled = bool(
            self.declare_parameter("latest_sample_observability_enabled", False).value
        )
        self.mux_runtime_measurement_capacity = int(
            self.declare_parameter(
                "mux_runtime_measurement_capacity", 4096
            ).value
        )
        if not 128 <= self.mux_runtime_measurement_capacity <= 65536:
            raise ValueError(
                "mux_runtime_measurement_capacity must be in [128, 65536]"
            )
        self.mux_runtime_measurement_callback_capacity = int(
            self.declare_parameter(
                "mux_runtime_measurement_callback_capacity",
                self.mux_runtime_measurement_capacity,
            ).value
        )
        self.mux_runtime_measurement_cycle_capacity = int(
            self.declare_parameter(
                "mux_runtime_measurement_cycle_capacity",
                self.mux_runtime_measurement_capacity,
            ).value
        )
        if not (
            128 <= self.mux_runtime_measurement_callback_capacity <= 65536
            and 128 <= self.mux_runtime_measurement_cycle_capacity <= 65536
        ):
            raise ValueError("mux runtime measurement journal capacities must be in [128, 65536]")
        # Keep the diagnostic sidecar completely absent by default.  The ON
        # path owns fixed, non-overwriting slots; reaching capacity is a
        # diagnostic fault, never an eviction policy.
        self.mux_runtime_measurement_wcet_slots = (
            [None] * max(
                self.mux_runtime_measurement_capacity,
                self.mux_runtime_measurement_cycle_capacity,
            )
            if self.mux_runtime_measurement_enabled
            else None
        )
        # Legacy spelling retained for existing consumers; it is WCET-only.
        self.mux_runtime_measurement_records = self.mux_runtime_measurement_wcet_slots
        self.mux_runtime_measurement_cycle_records = (
            [None] * max(
                self.mux_runtime_measurement_capacity,
                self.mux_runtime_measurement_cycle_capacity,
            )
            if self.mux_runtime_measurement_enabled
            else None
        )
        self.mux_runtime_measurement_pending_cycle = None
        # Latest-sample evidence is deliberately separate from the legacy
        # WCET/cycle sidecar.  Slots are fixed scalars/tuples only.
        self.mux_latest_sample_callback_slots = (
            [None] * self.mux_runtime_measurement_callback_capacity
            if self.latest_sample_observability_enabled else None
        )
        self.mux_latest_sample_cycle_slots = (
            [None] * self.mux_runtime_measurement_cycle_capacity
            if self.latest_sample_observability_enabled else None
        )
        self.mux_runtime_measurement_drops = 0
        self.mux_runtime_measurement_faults = 0
        self.mux_runtime_measurement_sequence = 0
        self.mux_runtime_measurement_legacy_cycle_count = 0
        self.mux_latest_sample_cycle_sequence = 0
        self.mux_runtime_measurement_callback_sequence = 0
        self.mux_runtime_measurement_callback_count = 0
        self.mux_runtime_measurement_cycle_count = 0
        self.mux_runtime_measurement_callback_in_flight = 0
        self.mux_runtime_measurement_cycle_in_flight = 0
        self.mux_runtime_measurement_overflow = False
        self.mux_runtime_measurement_callback_overflow = False
        self.mux_runtime_measurement_cycle_overflow = False
        self.mux_runtime_measurement_callback_drops = 0
        self.mux_runtime_measurement_cycle_drops = 0
        self.mux_runtime_measurement_sealed = False
        self.mux_runtime_measurement_capture_epoch = 0
        self.mux_runtime_measurement_capture_armed_steady_ns = -1
        self.mux_runtime_measurement_capture_closed = False
        self.mux_runtime_measurement_capture_late_entry = False
        self.mux_runtime_measurement_executor_quiesced = False
        self.mux_runtime_measurement_callback_completions = 0
        self.mux_runtime_measurement_cycle_completions = 0
        self._mux_runtime_measurement_callback_context = None
        self._mux_latest_sample_cycle_context = None
        if self.latest_sample_observability_enabled:
            self.mux_runtime_measurement_capture_epoch = 1
            self.mux_runtime_measurement_capture_armed_steady_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self.mux_runtime_measurement_output_path = str(
            self.declare_parameter(
                "mux_runtime_measurement_output_path", ""
            ).value
        ).strip()
        # This hash binds the capture to a pre-run selector *rule*, never to a
        # PP identity.  PP identities are generated at runtime and are selected
        # only by the read-only offline classifier after the capture is sealed.
        self.mux_runtime_measurement_selector_manifest_sha256 = str(
            self.declare_parameter(
                "mux_runtime_measurement_selector_manifest_sha256", ""
            ).value
        ).strip()
        self.mux_runtime_measurement_capture_epoch_nonce = str(
            self.declare_parameter(
                "mux_runtime_measurement_capture_epoch_nonce", ""
            ).value
        ).strip()
        self.mux_runtime_measurement_expected_capture_epoch = int(
            self.declare_parameter(
                "mux_runtime_measurement_expected_capture_epoch", 0
            ).value
        )
        if (
            self.mux_runtime_measurement_selector_manifest_sha256
            and (
                len(self.mux_runtime_measurement_selector_manifest_sha256) != 64
                or any(
                    character not in "0123456789abcdef"
                    for character in self.mux_runtime_measurement_selector_manifest_sha256
                )
            )
        ):
            raise ValueError(
                "mux_runtime_measurement_selector_manifest_sha256 must be a "
                "lowercase SHA-256 digest when set"
            )
        if (
            self.mux_runtime_measurement_expected_capture_epoch < 0
            or self.mux_runtime_measurement_expected_capture_epoch > 1
        ):
            raise ValueError(
                "mux_runtime_measurement_expected_capture_epoch must be 0 or 1"
            )
        if (
            self.mux_runtime_measurement_capture_epoch_nonce
            and len(self.mux_runtime_measurement_capture_epoch_nonce) < 16
        ):
            raise ValueError(
                "mux_runtime_measurement_capture_epoch_nonce must be at least 16 "
                "characters when set"
            )
        self.free_run_live_exact_evidence_timeout_sec = float(
            self.declare_parameter(
                "free_run_live_exact_evidence_timeout_sec", 0.12
            ).value
        )
        if (
            not math.isfinite(
                self.free_run_live_exact_evidence_timeout_sec
            )
            or not 0.0 < self.free_run_live_exact_evidence_timeout_sec <= 0.12
        ):
            raise ValueError(
                "free_run_live_exact_evidence_timeout_sec must be finite "
                "and in (0.0, 0.12]"
            )
        self.free_run_source_provenance_timeout_sec = float(
            self.declare_parameter(
                "free_run_source_provenance_timeout_sec", 1.5
            ).value
        )
        if (
            not math.isfinite(self.free_run_source_provenance_timeout_sec)
            or not 0.0 < self.free_run_source_provenance_timeout_sec <= 1.5
        ):
            raise ValueError(
                "free_run_source_provenance_timeout_sec must be finite "
                "and in (0.0, 1.5]"
            )
        self.mpc_health_timeout_sec = float(
            self.declare_parameter("mpc_health_timeout_sec", 0.75).value
        )
        self.safety_constraint_enabled = bool(
            self.declare_parameter("safety_constraint_enabled", True).value
        )
        self.require_safety_constraint = bool(
            self.declare_parameter("require_safety_constraint", False).value
        )
        self.safety_constraint_timeout_sec = float(
            self.declare_parameter("safety_constraint_timeout_sec", 0.20).value
        )
        self.overtake_plan_timeout_sec = float(
            self.declare_parameter("overtake_plan_timeout_sec", 0.20).value
        )
        self.safety_constraint_max_speed_mps = float(
            self.declare_parameter("safety_constraint_max_speed_mps", 15.0).value
        )
        planner_stop_release_bootstrap_max_speed_mps = float(
            self.declare_parameter(
                "planner_stop_release_bootstrap_max_speed_mps", 0.20
            ).value
        )
        if (
            not math.isfinite(planner_stop_release_bootstrap_max_speed_mps)
            or planner_stop_release_bootstrap_max_speed_mps <= 0.0
        ):
            planner_stop_release_bootstrap_max_speed_mps = 0.20
        self.planner_stop_release_bootstrap_max_speed_mps = min(
            max(1.0e-3, planner_stop_release_bootstrap_max_speed_mps),
            max(1.0e-3, self.safety_constraint_max_speed_mps),
        )
        self.safety_constraint_max_brake_decel_mps2 = float(
            self.declare_parameter(
                "safety_constraint_max_brake_decel_mps2", 1.5
            ).value
        )
        self.fallback_speed_mps = float(
            self.declare_parameter("fallback_speed_mps", 2.0).value
        )
        self.fallback_accel_max_mps2 = float(
            self.declare_parameter("fallback_accel_max_mps2", 0.8).value
        )
        self.fallback_decel_min_mps2 = float(
            self.declare_parameter("fallback_decel_min_mps2", -1.5).value
        )
        self.recovery_enabled = bool(
            self.declare_parameter("recovery_enabled", False).value
        )
        self.recovery_cmd_timeout_sec = float(
            self.declare_parameter("recovery_cmd_timeout_sec", 0.15).value
        )
        self.recovery_status_timeout_sec = float(
            self.declare_parameter("recovery_status_timeout_sec", 0.20).value
        )
        self.recovery_permit_timeout_sec = float(
            self.declare_parameter("recovery_permit_timeout_sec", 0.30).value
        )
        self.external_safety_timeout_sec = float(
            self.declare_parameter("external_safety_timeout_sec", 0.20).value
        )
        self.require_recovery_external_safety_status = bool(
            self.declare_parameter("require_recovery_external_safety_status", True).value
        )
        self.recovery_speed_mps = float(
            self.declare_parameter("recovery_speed_mps", 0.7).value
        )
        self.recovery_accel_max_mps2 = float(
            self.declare_parameter("recovery_accel_max_mps2", 0.5).value
        )
        self.recovery_decel_min_mps2 = float(
            self.declare_parameter("recovery_decel_min_mps2", -1.5).value
        )
        self.stop_decel_mps2 = float(self.declare_parameter("stop_decel_mps2", -1.5).value)
        self.finish_stop_enabled = bool(
            self.declare_parameter("finish_stop_enabled", True).value
        )
        finish_stop_decel_mps2 = float(
            self.declare_parameter("finish_stop_decel_mps2", -3.2).value
        )
        # Finish通過時は実bagで約8.8 m/sあり、通常安全制約の1.5 m/s^2では
        # 停止距離が約26 mになる。公式Finish専用のterminal停止は、AWSIMで
        # 使用中の既知の縦加速度範囲(a_min=-3.2 m/s^2)内で独立にclampする。
        if not math.isfinite(finish_stop_decel_mps2):
            finish_stop_decel_mps2 = -3.2
        self.finish_stop_decel_mps2 = -min(
            3.2, max(1.0e-3, abs(finish_stop_decel_mps2))
        )
        finish_stop_max_steering_rad = float(
            self.declare_parameter("finish_stop_max_steering_rad", 0.20).value
        )
        if not math.isfinite(finish_stop_max_steering_rad):
            finish_stop_max_steering_rad = 0.20
        # Finish後はpath追従用の操舵を残すが、PASS/rejoin由来の急舵を
        # hard braking中へ持ち越さない。通常のsteering rate limiterも後段で効く。
        self.finish_stop_max_steering_rad = min(
            0.50, max(0.0, abs(finish_stop_max_steering_rad))
        )
        finish_stop_steering_guard_trigger_rad = float(
            self.declare_parameter(
                "finish_stop_steering_guard_trigger_rad", 0.35
            ).value
        )
        if not math.isfinite(finish_stop_steering_guard_trigger_rad):
            finish_stop_steering_guard_trigger_rad = 0.35
        self.finish_stop_steering_guard_trigger_rad = min(
            0.50, max(0.0, abs(finish_stop_steering_guard_trigger_rad))
        )
        self.debug_publish_period_sec = float(
            self.declare_parameter("debug_publish_period_sec", 0.25).value
        )
        self.steering_log_throttle_sec = float(
            self.declare_parameter("steering_log_throttle_sec", 1.0).value
        )
        self.last_steering_limit_log_sec = -1.0e9
        # Publication truth is deliberately separate from selector/limiter
        # state.  A delivery-gap replay may continue only the exact command
        # that actually left this node on the preceding control tick.
        self.last_published_control_command: Optional[
            AckermannControlCommand
        ] = None
        self.last_published_control_source = ""
        self.last_published_control_steady_time_sec: Optional[float] = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        config = HybridMuxConfig(
            primary_source=str(
                self.declare_parameter("primary_source", "mpc").value
            ),
            fallback_trigger_infeasible_count=int(
                self.declare_parameter("fallback_trigger_infeasible_count", 2).value
            ),
            fallback_release_solved_cycles=int(
                self.declare_parameter("fallback_release_solved_cycles", 3).value
            ),
            fallback_min_hold_sec=float(
                self.declare_parameter("fallback_min_hold_sec", 1.0).value
            ),
            use_pure_pursuit_on_mpc_cmd_timeout=bool(
                self.declare_parameter("use_pure_pursuit_on_mpc_cmd_timeout", True).value
            ),
            use_pure_pursuit_on_mpc_health_timeout=bool(
                self.declare_parameter("use_pure_pursuit_on_mpc_health_timeout", False).value
            ),
            use_mpc_on_pure_pursuit_cmd_timeout=bool(
                self.declare_parameter("use_mpc_on_pure_pursuit_cmd_timeout", False).value
            ),
            recovery_max_duration_sec=float(
                self.declare_parameter("recovery_max_duration_sec", 3.0).value
            ),
            state_lattice_instant_control_enabled=(
                self.state_lattice_instant_control_enabled
            ),
        )
        self.core = HybridMuxCore(config)
        self.safety_authority = SafetyConstraintAuthority(
            required=self.safety_constraint_enabled and self.require_safety_constraint,
            timeout_sec=self.safety_constraint_timeout_sec,
            maximum_speed_limit_mps=self.safety_constraint_max_speed_mps,
            maximum_brake_decel_mps2=self.safety_constraint_max_brake_decel_mps2,
        )
        period = 1.0 / max(1.0, self.control_rate_hz)
        self.control_loop_watchdog = ControlLoopWatchdog(
            float(
                self.declare_parameter(
                    "control_loop_max_gap_sec", max(0.10, period * 3.0)
                ).value
            )
        )
        self.ros_clock_watchdog = RosClockProgressWatchdog(
            float(
                self.declare_parameter(
                    "ros_clock_stall_timeout_sec", 0.20
                ).value
            )
        )
        self.race_arm_required = bool(
            # Fail closed for direct ros2 run and alternate launches too.
            # Legacy unit paths must explicitly opt out.
            self.declare_parameter("race_arm_required", True).value
        )
        self.ros_clock_motion_ready = not self.race_arm_required
        self.ros_clock_observation_reason = "race_not_armed"
        self.control_fault_clear_safe_cycles = max(
            1,
            int(
                self.declare_parameter(
                    "control_fault_clear_safe_cycles", 3
                ).value
            ),
        )
        self.control_fault_latched = False
        self.control_fault_clear_cycles = 0
        self.control_fault_reason = ""
        self.control_fault_last_release_stamp_ns: Optional[int] = None
        lateral_stop_steering_hold_timeout_sec = float(
            self.declare_parameter(
                "lateral_stop_steering_hold_timeout_sec", 0.12
            ).value
        )
        if not math.isfinite(lateral_stop_steering_hold_timeout_sec):
            lateral_stop_steering_hold_timeout_sec = 0.12
        self.lateral_stop_steering_hold_timeout_sec = min(
            0.25, max(0.02, lateral_stop_steering_hold_timeout_sec)
        )
        attack_follow_transport_continuity_timeout_sec = float(
            self.declare_parameter(
                "attack_follow_transport_continuity_timeout_sec", 0.12
            ).value
        )
        if not math.isfinite(attack_follow_transport_continuity_timeout_sec):
            attack_follow_transport_continuity_timeout_sec = 0.12
        self.attack_follow_transport_continuity_timeout_sec = min(
            0.12, max(0.02, attack_follow_transport_continuity_timeout_sec)
        )
        attack_follow_transport_max_point_delta_m = float(
            self.declare_parameter(
                "attack_follow_transport_max_point_delta_m", 0.15
            ).value
        )
        if not math.isfinite(attack_follow_transport_max_point_delta_m):
            attack_follow_transport_max_point_delta_m = 0.15
        self.attack_follow_transport_max_point_delta_m = min(
            0.25, max(0.01, attack_follow_transport_max_point_delta_m)
        )
        max_steering_angle_rad = float(
            self.declare_parameter(
                "max_steering_angle_rad", 0.64
            ).value
        )
        self.tracking_usable_max_steering_angle_rad = float(
            self.declare_parameter(
                "tracking_usable_max_steering_angle_rad",
                max_steering_angle_rad,
            ).value
        )
        if (
            not math.isfinite(max_steering_angle_rad)
            or max_steering_angle_rad <= 0.0
            or max_steering_angle_rad >= math.pi / 2.0
        ):
            raise ValueError(
                "max_steering_angle_rad must be finite and in (0, pi/2)"
            )
        if (
            not math.isfinite(
                self.tracking_usable_max_steering_angle_rad
            )
            or self.tracking_usable_max_steering_angle_rad <= 0.0
            or self.tracking_usable_max_steering_angle_rad
            > max_steering_angle_rad
        ):
            raise ValueError(
                "tracking_usable_max_steering_angle_rad must be finite, "
                "positive, and no greater than max_steering_angle_rad"
            )
        self.steering_limiter = SteeringLimiter(
            SteeringLimiterConfig(
                enabled=bool(self.declare_parameter("enable_steering_rate_limit", True).value),
                max_steering_angle_rad=max_steering_angle_rad,
                max_steering_rate_radps=float(
                    self.declare_parameter("max_steering_rate_radps", 128.0).value
                ),
                max_steering_delta_per_cycle=float(
                    self.declare_parameter("max_steering_delta_per_cycle", 0.0).value
                ),
                reset_dt_threshold_sec=float(
                    self.declare_parameter("steering_limiter_reset_dt_sec", 0.50).value
                ),
                reset_on_mode_change=bool(
                    self.declare_parameter("reset_steering_limiter_on_mode_change", False).value
                ),
            )
        )

        self.mpc_cmd: Optional[AckermannControlCommand] = None
        self.mpc_cmd_time_sec: Optional[float] = None
        self.mpc_cmd_stamp_ns: Optional[int] = None
        self.pure_pursuit_cmd: Optional[AckermannControlCommand] = None
        self.pure_pursuit_cmd_time_sec: Optional[float] = None
        self.pure_pursuit_cmd_stamp_ns: Optional[int] = None
        self.pure_pursuit_tracking_status: Optional[ControllerTrackingStatus] = None
        self.pure_pursuit_tracking_status_time_sec: Optional[float] = None
        self.pure_pursuit_tracking_status_stamp_ns: Optional[int] = None
        self.pure_pursuit_tracking_status_valid = False
        self.pure_pursuit_tracking_status_cache: dict[
            tuple[int, int], tuple[ControllerTrackingStatus, float, bool]
        ] = {}
        self.pure_pursuit_envelope_cache: dict[
            tuple[int, int, int, int],
            tuple[tuple, float, bool, str, ControllerCommandEnvelope],
        ] = {}
        # A bounded cache is only a rendezvous aid.  Keep one immutable
        # current-plan record separately so eviction cannot turn a received
        # unusable N sample into a false "missing" N sample and reopen N-1.
        self.pure_pursuit_envelope_watermark: Optional[
            tuple[
                tuple[int, int, int, int],
                tuple[tuple, float, bool, str, ControllerCommandEnvelope],
            ]
        ] = None
        self.pure_pursuit_envelope_active_producer_instance_id: Optional[int] = None
        self.pure_pursuit_envelope_active_sequence: Optional[int] = None
        self.pure_pursuit_envelope_active_stamp_ns: Optional[int] = None
        self.pure_pursuit_envelope_candidate_instance_id: Optional[int] = None
        self.pure_pursuit_envelope_candidate_sequence: Optional[int] = None
        self.pure_pursuit_envelope_candidate_stamp_ns: Optional[int] = None
        self.pure_pursuit_envelope_candidate_valid_count = 0
        self.pure_pursuit_envelope_quarantined_instances: set[int] = set()
        self.pure_pursuit_envelope_fault_latched = False
        self.pure_pursuit_envelope_fault_reason = ""
        self.pure_pursuit_envelope_last_valid = False
        self.pure_pursuit_envelope_last_reason = "missing"
        self.pure_pursuit_envelope_last_identity: Optional[
            tuple[int, int, int, int]
        ] = None
        self.pure_pursuit_envelope_last_message: Optional[
            ControllerCommandEnvelope
        ] = None
        self.pure_pursuit_envelope_last_receipt_time_sec: Optional[float] = None
        self.pure_pursuit_envelope_last_parity_valid = False
        self.pure_pursuit_envelope_last_parity_reason = "legacy_missing"
        self.state_lattice_cmd: Optional[StateLatticeControlCommand] = None
        self.state_lattice_cmd_time_sec: Optional[float] = None
        self.state_lattice_cmd_stamp_ns: Optional[int] = None
        self.state_lattice_cmd_valid = False
        self.state_lattice_cmd_reason = "missing"
        self.state_lattice_active_producer_instance_id: Optional[int] = None
        self.state_lattice_last_sequence: Optional[int] = None
        self.state_lattice_last_plan_generation: Optional[int] = None
        self.state_lattice_last_fingerprint: Optional[tuple] = None
        self.recovery_cmd: Optional[RecoveryControlCommand] = None
        self.recovery_cmd_time_sec: Optional[float] = None
        self.recovery_cmd_stamp_ns: Optional[int] = None
        self.recovery_status: Optional[RecoveryStatus] = None
        self.recovery_status_time_sec: Optional[float] = None
        self.recovery_status_stamp_ns: Optional[int] = None
        self.recovery_permit: Optional[RecoveryPermit] = None
        self.recovery_permit_time_sec: Optional[float] = None
        self.recovery_permit_stamp_ns: Optional[int] = None
        self.external_safety_status: Optional[SafetyStopStatus] = None
        self.external_safety_time_sec: Optional[float] = None
        self.external_safety_stamp_ns: Optional[int] = None
        self.external_stop_latched = False
        self.finish_stop_latch = RaceFinishStopLatch()
        self.finish_stop_lateral_guard_active = False
        self.last_tracking_steering_rad: Optional[float] = None
        # Terminal steering is intentionally independent from both the PP
        # transport cache and the short-lived lateral STOP proof.  It records
        # only the already limited command that was actually published before
        # the official Finish latch, then freezes for the rest of this race
        # epoch.
        finish_terminal_reference_timeout_sec = float(
            self.declare_parameter(
                "finish_terminal_reference_timeout_sec", 0.20
            ).value
        )
        if (
            not math.isfinite(finish_terminal_reference_timeout_sec)
            or not 0.02 <= finish_terminal_reference_timeout_sec <= 0.50
        ):
            raise ValueError(
                "finish_terminal_reference_timeout_sec must be finite and in "
                "[0.02, 0.50]"
            )
        self.finish_terminal_reference_timeout_sec = (
            finish_terminal_reference_timeout_sec
        )
        self.finish_terminal_candidate_steering_rad: Optional[float] = None
        self.finish_terminal_candidate_time_sec: Optional[float] = None
        self.finish_terminal_candidate_epoch: Optional[int] = None
        self.finish_terminal_snapshot_steering_rad: Optional[float] = None
        self.finish_terminal_snapshot_epoch: Optional[int] = None
        # Keep provenance for maneuver and nominal-path STOP steering separate.
        # A normal PP cycle or the other authority kind must not refresh the
        # bounded hold lifetime of a previously verified STOP bundle.
        self.last_verified_tracking_steering_rad: Optional[float] = None
        self.last_verified_tracking_steering_time_sec: Optional[float] = None
        self.last_verified_lateral_stop_steering_rad: Optional[float] = None
        self.last_verified_lateral_stop_steering_time_sec: Optional[float] = None
        self.last_verified_baseline_stop_steering_rad: Optional[float] = None
        self.last_verified_baseline_stop_steering_time_sec: Optional[float] = None
        self.last_verified_lateral_stop_plan_generation: Optional[int] = None
        self.last_verified_baseline_stop_plan_generation: Optional[int] = None
        # A normal FREE_RUN command may cross the conservative tracking
        # reserve by a tiny amount while remaining below the actuator hard
        # limit.  Never use that unverified current value as lateral authority.
        # Keep only the last soft-limit-compliant steering that was actually
        # published for the exact same baseline generation, so a longitudinal
        # STOP does not abruptly reset steering to zero.
        self.last_verified_baseline_free_run_steering_rad: Optional[float] = None
        self.last_verified_baseline_free_run_steering_time_sec: Optional[float] = None
        self.last_verified_baseline_free_run_plan_generation: Optional[int] = None
        self.last_verified_baseline_free_run_plan_stamp_ns: Optional[int] = None
        self.last_verified_baseline_free_run_epoch: Optional[int] = None
        # Observation-only FRL-3 cache. It cannot release longitudinal motion,
        # preserve steering, refresh legacy proof, or create a motion grant.
        self.free_run_live_exact_ack: Optional[FreeRunExecutionAck] = None
        self.free_run_live_exact_ack_time_sec: Optional[float] = None
        self.free_run_live_exact_ack_valid = False
        self.free_run_live_exact_ack_reason = "disabled"
        self.free_run_live_exact_ack_identity: Optional[tuple[int, ...]] = None
        self.free_run_live_exact_source_key: Optional[FreeRunSourceKey] = None
        self.free_run_live_exact_source_key_time_sec: Optional[float] = None
        self.free_run_live_exact_source_key_valid = False
        self.free_run_live_exact_source_key_reason = "disabled"
        self.free_run_live_exact_source_key_identity: Optional[
            tuple[int, ...]
        ] = None
        self.free_run_live_exact_source_wire_digest_cache: dict[
            tuple[int, ...], bytes
        ] = {}
        self.free_run_live_exact_tainted_sources: set[
            tuple[int, ...]
        ] = set()
        self.free_run_live_exact_plan_digest_cache: dict[
            tuple[int, int, int], bytes
        ] = {}
        self.free_run_live_exact_plan_delivery_cache: dict[
            tuple[int, int], tuple[bool, bytes, int]
        ] = {}
        self.free_run_live_exact_ack_wire_digest_cache: dict[
            tuple[int, ...], bytes
        ] = {}
        self.free_run_live_exact_tainted_generations: set[
            tuple[int, int, int]
        ] = set()
        self.free_run_live_exact_selected_envelope_valid = False
        self.free_run_live_exact_selected_envelope_reason = "disabled"
        self.free_run_live_exact_forward_transition_reason = "disabled"
        self.free_run_live_exact_selected_envelope_identity: Optional[
            tuple[int, int, int, int]
        ] = None
        # Provenance is matching evidence only.  It never by itself preserves
        # steering or grants motion after the exact current tuple is lost.
        self.free_run_source_provenance_record: Optional[
            FreeRunPublishedCommandRecord
        ] = None
        # Authority is a separate, non-refreshable lease.  Only a direct
        # SourceKey successor can mint it, and it composes a canonical STOP.
        self.free_run_source_gap_lease: Optional[FreeRunSourceGapLease] = None
        # Latest SourceKey is transport observation, not active provenance.
        # Keep its direct successor separate from the immutable old command
        # record until ACK + envelope + tracking + final publish all join.
        self.free_run_pending_source: Optional[PendingFreeRunSource] = None
        self.free_run_source_gap_expired_successor_identity: Optional[
            tuple[int, int, int, int]
        ] = None
        # Low-cadence debug publication can miss a 0.12 s delivery-gap lease.
        # Keep process-lifetime, saturating diagnostic counters in the control
        # node so the bag can prove hold, exact promotion, and expiry without
        # changing any authority decision or retaining an unbounded event log.
        self.free_run_source_gap_hold_episode_count = 0
        self.free_run_source_gap_exact_promotion_count = 0
        self.free_run_source_gap_lease_expiry_count = 0
        self.free_run_source_gap_last_counted_hold_identity: Optional[
            tuple[int, int, int, int, int]
        ] = None
        self.free_run_live_exact_record_reason = "disabled"
        self.safety_constraint: Optional[SafetyConstraintState] = None
        self.safety_constraint_time_sec: Optional[float] = None
        self.safety_constraint_header_stamp_ns: Optional[int] = None
        self.safety_constraint_timestamp_regressed = False
        self.safety_constraint_cache: dict[
            int, tuple[SafetyConstraintState, float]
        ] = {}
        # plan_generationだけをkeyにすると、同じgenerationの新stampが片topic
        # だけ先着した時に、直前の完全一致pairを上書きして失う。transport
        # rendezvous専用に(generation, header stamp)の短い履歴を別保持する。
        self.safety_constraint_contract_cache: dict[
            tuple[int, int], tuple[SafetyConstraintState, float]
        ] = {}
        self.overtake_plan_generation: Optional[int] = None
        self.overtake_plan_time_sec: Optional[float] = None
        self.overtake_plan_header_stamp_ns: Optional[int] = None
        self.overtake_plan_valid = False
        self.overtake_plan_trajectory_authorized = False
        self.overtake_plan_lateral_maneuver_required = False
        self.safety_authority_contract_unpaired = False
        self.overtake_plan_cache: dict[
            int, tuple[bool, float, int, bool, bool, int, int, str]
        ] = {}
        self.overtake_plan_attempt_cache: dict[int, int] = {}
        self.overtake_plan_motion_identity_cache: dict[int, tuple] = {}
        self.overtake_plan_trajectory_cache: dict[
            int, tuple[tuple[float, float], ...]
        ] = {}
        # STOP中の横操舵authorityはgeneric plan flagsから推測しない。
        # Planner宣言とPP実適用proofをexact generationで束縛する。
        self.overtake_plan_lateral_stop_authority_cache: dict[
            int, tuple[int, int, int, int, str, int, int, bool]
        ] = {}
        self.overtake_plan_lateral_stop_fingerprint_cache: dict[int, tuple] = {}
        self.overtake_plan_lateral_stop_taint_cache: dict[int, bool] = {}
        self.overtake_plan_contract_cache: dict[
            tuple[int, int], tuple[bool, float, int, bool, bool, int, int, str]
        ] = {}
        self.mpc_health = MpcHealth()
        self.mpc_health_time_sec: Optional[float] = None
        self.last_debug_publish_sec = -1.0e9
        self.last_motion_pp_tracking_proof_failure_signature = ""
        self.last_forced_motion_pp_tracking_debug_sec = -1.0e9
        self.forced_motion_pp_tracking_debug_min_period_sec = 0.10
        self.last_source = ""
        motion_authority_grant_lease_sec = float(
            self.declare_parameter(
                "motion_authority_grant_lease_sec", 0.04
            ).value
        )
        if (
            not math.isfinite(motion_authority_grant_lease_sec)
            or not 0.0 < motion_authority_grant_lease_sec <= 0.10
        ):
            raise ValueError(
                "motion_authority_grant_lease_sec must be finite and in "
                "(0.0, 0.10]"
            )
        self.motion_authority_grant_lease_sec = (
            motion_authority_grant_lease_sec
        )
        motion_authority_delivery_gap_lease_sec = float(
            self.declare_parameter(
                "motion_authority_delivery_gap_lease_sec", 0.04
            ).value
        )
        if (
            not math.isfinite(motion_authority_delivery_gap_lease_sec)
            or not 0.0 < motion_authority_delivery_gap_lease_sec <= 0.04
        ):
            raise ValueError(
                "motion_authority_delivery_gap_lease_sec must be finite and "
                "in (0.0, 0.04]"
            )
        # This is intentionally shorter than every message freshness bound.
        # The effective bound below remains fail-closed if a launch supplies a
        # shorter existing freshness timeout.
        self.motion_authority_delivery_gap_lease_sec = (
            motion_authority_delivery_gap_lease_sec
        )
        self.motion_authority_delivery_gap_lease: Optional[
            MotionAuthorityDeliveryGapLease
        ] = None
        # This is deliberately independent from the active lease.  Once an
        # exact cohort has expired or been revoked, timer republishes of that
        # same cohort must not mint a fresh delivery-gap window.
        self.motion_authority_delivery_gap_tombstone: Optional[
            MotionAuthorityDeliveryGapLease
        ] = None
        # A callback-side hard negative revokes the active lease immediately,
        # but its same-tick STOP must not be masked by a valid replacement
        # arriving before the timer callback.  The marker is consumed once by
        # the next timer and is cleared at a race-epoch boundary.
        self.motion_authority_delivery_gap_stop_pending = False
        self.motion_authority_delivery_gap_revoke_pending: Optional[
            DeliveryGapRevokeRecord
        ] = None
        self.motion_authority_delivery_gap_last_revoke: Optional[
            DeliveryGapRevokeRecord
        ] = None
        self.motion_authority_grant_issuer_instance_id = (
            secrets.randbits(64) or 1
        )
        self.motion_authority_grant_sequence = 0
        self.motion_authority_grant_active = False
        warmup_proof_timeout_sec = float(
            self.declare_parameter(
                "motion_authority_warmup_proof_timeout_sec", 0.50
            ).value
        )
        if (
            not math.isfinite(warmup_proof_timeout_sec)
            or not 0.0 < warmup_proof_timeout_sec <= 1.0
        ):
            raise ValueError(
                "motion_authority_warmup_proof_timeout_sec must be finite "
                "and in (0.0, 1.0]"
            )
        self.motion_authority_warmup_proof_timeout_sec = (
            warmup_proof_timeout_sec
        )
        self.motion_authority_warmup_proof: Optional[dict[str, object]] = None

        self.create_subscription(
            AckermannControlCommand,
            "input/mpc_control_cmd",
            self.on_mpc_cmd,
            1,
        )
        self.create_subscription(
            AckermannControlCommand,
            "input/pure_pursuit_control_cmd",
            self.on_pure_pursuit_cmd,
            1,
        )
        self.create_subscription(
            ControllerTrackingStatus,
            "input/pure_pursuit_tracking_status",
            self.on_pure_pursuit_tracking_status,
            1,
        )
        self.create_subscription(
            ControllerCommandEnvelope,
            "input/pure_pursuit_command_envelope",
            self.on_pure_pursuit_command_envelope,
            1,
        )
        self.create_subscription(
            StateLatticeControlCommand,
            "input/state_lattice_control_cmd",
            self.on_state_lattice_control_cmd,
            1,
        )
        if self.free_run_live_exact_observe_enabled:
            self.create_subscription(
                FreeRunSourceKey,
                "input/free_run_source_key",
                self.on_free_run_source_key,
                1,
            )
            self.create_subscription(
                FreeRunExecutionAck,
                "input/free_run_execution_ack",
                self.on_free_run_execution_ack,
                1,
            )
        self.create_subscription(
            RecoveryControlCommand,
            "input/recovery_control_cmd",
            self.on_recovery_cmd,
            1,
        )
        self.create_subscription(
            RecoveryStatus,
            "input/recovery_status",
            self.on_recovery_status,
            1,
        )
        self.create_subscription(
            RecoveryPermit,
            "input/recovery_permit",
            self.on_recovery_permit,
            1,
        )
        self.create_subscription(
            SafetyStopStatus,
            "input/external_safety_status",
            self.on_external_safety_status,
            1,
        )
        self.create_subscription(
            SafetyConstraint,
            "input/safety_constraint",
            self.on_safety_constraint,
            1,
        )
        self.create_subscription(
            OvertakePlan,
            "input/overtake_plan",
            self.on_overtake_plan,
            1,
        )
        self.create_subscription(String, "input/mpc_health", self.on_mpc_health, 1)
        self.create_subscription(Bool, "input/race_armed", self.on_race_armed, 1)
        self.create_subscription(String, "input/awsim_state", self.on_awsim_state, 1)
        self.control_pub = self.create_publisher(AckermannControlCommand, "output/control_cmd", 1)
        self.debug_pub = self.create_publisher(String, "output/debug", 1)
        self.tracking_status_pub = self.create_publisher(
            ControllerTrackingStatus, "output/controller_tracking_status", 1
        )
        self.motion_authority_grant_pub = self.create_publisher(
            MotionAuthorityGrant, "output/motion_authority_grant", 1
        )

        self.timer = self.create_timer(
            period, self.on_timer, clock=self.steady_clock
        )

    def now_sec(self) -> float:
        return self.steady_clock.now().nanoseconds / 1.0e9

    def _mux_measurement_store_callback(self, record: tuple) -> None:
        if self.mux_runtime_measurement_callback_count >= self.mux_runtime_measurement_callback_capacity:
            self.mux_runtime_measurement_callback_drops = min((1 << 64) - 1, self.mux_runtime_measurement_callback_drops + 1)
            self.mux_runtime_measurement_drops = min((1 << 64) - 1, self.mux_runtime_measurement_drops + 1)
            self.mux_runtime_measurement_overflow = True
            self.mux_runtime_measurement_callback_overflow = True
            return
        self.mux_latest_sample_callback_slots[self.mux_runtime_measurement_callback_count] = record
        self.mux_runtime_measurement_callback_count += 1

    def _mux_measurement_callback_entry(self) -> bool:
        """Begin one PP-envelope receipt without participating in its decision."""
        if self.mux_runtime_measurement_capture_closed:
            self.mux_runtime_measurement_capture_late_entry = True
            self.mux_runtime_measurement_faults = min((1 << 64) - 1, self.mux_runtime_measurement_faults + 1)
            return False
        self.mux_runtime_measurement_callback_sequence += 1
        self.mux_runtime_measurement_callback_in_flight += 1
        self._mux_runtime_measurement_callback_context = (
            self.mux_runtime_measurement_callback_sequence,
            time.clock_gettime_ns(time.CLOCK_MONOTONIC),
            None,
            self.pure_pursuit_envelope_watermark[0]
            if self.pure_pursuit_envelope_watermark is not None
            else None,
        )
        return True

    def mark_mux_runtime_measurement_executor_quiesced(self) -> None:
        """Close capture only after the executor has stopped dispatching work."""
        if not (self.mux_runtime_measurement_enabled or self.latest_sample_observability_enabled):
            return
        self.mux_runtime_measurement_capture_closed = True
        self.mux_runtime_measurement_executor_quiesced = True

    def _mux_measurement_callback_identity(self, identity: tuple[int, int, int, int]) -> None:
        context = self._mux_runtime_measurement_callback_context
        if context is not None:
            self._mux_runtime_measurement_callback_context = (
                context[0], context[1], tuple(identity), context[3]
            )

    def _mux_measurement_callback_finalize(self, exception: bool) -> None:
        context = self._mux_runtime_measurement_callback_context
        self._mux_runtime_measurement_callback_context = None
        self.mux_runtime_measurement_callback_in_flight -= 1
        self.mux_runtime_measurement_callback_completions += 1
        if context is None:
            self.mux_runtime_measurement_faults += 1
            return
        ordinal, steady_ns, identity, watermark_before = context
        cached = self.pure_pursuit_envelope_cache.get(identity) if identity else None
        watermark_after = (
            self.pure_pursuit_envelope_watermark[0]
            if self.pure_pursuit_envelope_watermark is not None
            else None
        )
        cache_action = (
            "rejected_before_insert" if cached is None else "inserted"
        )
        if cached is not None and self.pure_pursuit_envelope_last_reason == "duplicate":
            cache_action = "duplicate_same"
        elif cached is not None and self.pure_pursuit_envelope_last_reason == "duplicate_conflict":
            cache_action = "duplicate_conflict"
        watermark_action = (
            "advanced" if watermark_after == identity and watermark_after != watermark_before else "unchanged"
        )
        if exception:
            self.mux_runtime_measurement_faults += 1
        self._mux_measurement_store_callback((
            ordinal, steady_ns, identity, bool(cached and cached[2]),
            str(cached[3]) if cached is not None else "callback_exception",
            cache_action, watermark_before, watermark_after, watermark_action,
            bool(exception),
        ))

    @staticmethod
    def classify_latest_sample_trace(
        *, target_identity: tuple[int, int, int, int] | None,
        target_published_in_epoch: bool,
        sealed: bool,
        trace_loss: bool,
        callback_records: tuple[tuple, ...],
        cycle_records: tuple[tuple, ...],
    ) -> str:
        """Pure, fail-closed classifier for scalar latest-sample records."""
        if (
            target_identity is None or not sealed or trace_loss
            or not target_published_in_epoch
        ):
            return "INDETERMINATE_TRACE_LOSS"
        target = tuple(target_identity)
        target_receipt = next((record for record in callback_records if record[2] == target), None)
        target_admitted = bool(
            target_receipt is not None
            and target_receipt[5] == "inserted"
            and target_receipt[7] == target
        )
        callback_ordinals = [int(record[0]) for record in callback_records]
        cycle_ordinals = [int(record[0]) for record in cycle_records]
        if (
            (callback_ordinals and callback_ordinals[0] != 1)
            or (cycle_ordinals and cycle_ordinals[0] != 1)
            or any(
            current != previous + 1
            for previous, current in zip(
                callback_ordinals, callback_ordinals[1:]
            )
            )
            or any(
            current != previous + 1
            for previous, current in zip(cycle_ordinals, cycle_ordinals[1:])
            )
        ):
            return "INDETERMINATE_TRACE_LOSS"
        evaluated = bool(
            target_admitted
            and any(record[3] == target for record in cycle_records)
        )
        if evaluated:
            return "RECEIVED_AND_EVALUATED"
        if target_receipt is not None:
            successor = next(
                (
                    record for record in callback_records
                    if target_admitted
                    and record[8] == "advanced"
                    and record[6] == target
                    and record[7] != target
                    and record[7][0] == target[0]
                    and record[7][3] == target[3]
                    and record[7][1] > target[1]
                ),
                None,
            )
            if successor is not None and any(record[3] == successor[7] for record in cycle_records):
                return "RECEIVED_AND_SUPERSEDED"
            return "NONQUALIFYING"
        successor = next(
            (
                record for record in callback_records
                if record[2] is not None
                and record[2][0] == target[0]
                and record[2][3] == target[3]
                and record[2][1] > target[1]
            ),
            None,
        )
        if successor is not None and any(
            record[3] == successor[2] for record in cycle_records
        ):
            return "NOT_OBSERVED_IN_COMPLETE_MUX_TRACE"
        return "NONQUALIFYING"

    def destroy_node(self) -> bool:
        """Flush opt-in runtime measurements only during bounded shutdown."""
        if (
            (self.mux_runtime_measurement_enabled or self.latest_sample_observability_enabled)
            and self.mux_runtime_measurement_output_path
        ):
            output_path = self.mux_runtime_measurement_output_path
            temporary_path = output_path + ".tmp"
            quiescent = bool(
                self.mux_runtime_measurement_capture_closed
                and self.mux_runtime_measurement_executor_quiesced
                and self.mux_runtime_measurement_callback_in_flight == 0
                and self.mux_runtime_measurement_cycle_in_flight == 0
                and self.mux_runtime_measurement_callback_sequence
                == self.mux_runtime_measurement_callback_completions
                and self.mux_latest_sample_cycle_sequence
                == self.mux_runtime_measurement_cycle_completions
                and not self.mux_runtime_measurement_capture_late_entry
            )
            self.mux_runtime_measurement_sealed = quiescent
            if not quiescent:
                self.mux_runtime_measurement_faults += 1
            callback_records = tuple(
                self.mux_latest_sample_callback_slots[
                    :self.mux_runtime_measurement_callback_count
                ]
            ) if self.latest_sample_observability_enabled else ()
            cycle_records = tuple(
                self.mux_latest_sample_cycle_slots[
                    :self.mux_runtime_measurement_cycle_count
                ]
            ) if self.latest_sample_observability_enabled else ()
            trace_loss = bool(
                not quiescent
                or self.mux_runtime_measurement_callback_overflow
                or self.mux_runtime_measurement_cycle_overflow
                or self.mux_runtime_measurement_faults
            )
            payload = {
                "schema_version": 1,
                "legacy_measurement_enabled": bool(
                    self.mux_runtime_measurement_enabled
                ),
                "latest_sample_observability_enabled": bool(
                    self.latest_sample_observability_enabled
                ),
                "records": (
                    list(
                        self.mux_runtime_measurement_wcet_slots[
                            :self.mux_runtime_measurement_legacy_cycle_count
                        ]
                    )
                    if self.mux_runtime_measurement_enabled
                    else []
                ),
                "cycle_records_schema_version": 2,
                "cycle_records": (
                    list(
                        self.mux_runtime_measurement_cycle_records[
                            :self.mux_runtime_measurement_legacy_cycle_count
                        ]
                    )
                    if self.mux_runtime_measurement_enabled
                    else []
                ),
                "latest_sample_schema_version": 1,
                # Target selection is intentionally not a Mux responsibility:
                # the sealed target-agnostic trace is classified offline using
                # the independently recorded PP evidence and this manifest hash.
                "latest_sample_terminal": (
                    "UNCLASSIFIED_TARGET_AGNOSTIC_CAPTURE"
                    if not trace_loss
                    else "INDETERMINATE_TRACE_LOSS"
                ),
                "latest_sample_capture": {
                    "epoch": int(self.mux_runtime_measurement_capture_epoch),
                    "expected_capture_epoch": int(
                        self.mux_runtime_measurement_expected_capture_epoch
                    ),
                    "armed_steady_ns": int(self.mux_runtime_measurement_capture_armed_steady_ns),
                    "selector_manifest_sha256": str(
                        self.mux_runtime_measurement_selector_manifest_sha256
                    ),
                    "capture_epoch_nonce": str(
                        self.mux_runtime_measurement_capture_epoch_nonce
                    ),
                    "closed": bool(self.mux_runtime_measurement_capture_closed),
                    "late_entry": bool(self.mux_runtime_measurement_capture_late_entry),
                    "callback_entries": int(self.mux_runtime_measurement_callback_sequence),
                    "callback_completions": int(self.mux_runtime_measurement_callback_completions),
                    "cycle_entries": int(self.mux_latest_sample_cycle_sequence),
                    "cycle_completions": int(self.mux_runtime_measurement_cycle_completions),
                },
                "latest_sample_callback_records": (
                    list(callback_records)
                    if self.latest_sample_observability_enabled
                    else []
                ),
                "latest_sample_cycle_records": (
                    list(cycle_records)
                    if self.latest_sample_observability_enabled
                    else []
                ),
                "drops": int(self.mux_runtime_measurement_drops),
                "measurement_faults": int(
                    self.mux_runtime_measurement_faults
                ),
                "overflow": bool(self.mux_runtime_measurement_overflow),
                "callback_overflow": bool(self.mux_runtime_measurement_callback_overflow),
                "cycle_overflow": bool(self.mux_runtime_measurement_cycle_overflow),
                "callback_drops": int(self.mux_runtime_measurement_callback_drops),
                "cycle_drops": int(self.mux_runtime_measurement_cycle_drops),
                "sealed": bool(self.mux_runtime_measurement_sealed),
                "callback_in_flight": int(self.mux_runtime_measurement_callback_in_flight),
                "cycle_in_flight": int(self.mux_runtime_measurement_cycle_in_flight),
                "executor_quiesced": bool(
                    self.mux_runtime_measurement_executor_quiesced
                ),
            }
            try:
                with open(temporary_path, "w", encoding="utf-8") as stream:
                    json.dump(payload, stream, separators=(",", ":"))
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary_path, output_path)
            except OSError as error:
                self.mux_runtime_measurement_faults = min((1 << 64) - 1, self.mux_runtime_measurement_faults + 1)
                self.get_logger().error(
                    "failed to write mux runtime measurement: %s"
                    % error
                )
                raise
        return super().destroy_node()

    def _invalidate_verified_stop_steering(
        self, *, preserve_baseline_free_run: bool = False
    ) -> None:
        """Discard all short-lived lateral STOP proof across a safety barrier."""
        self.last_verified_tracking_steering_rad = None
        self.last_verified_tracking_steering_time_sec = None
        self.last_verified_lateral_stop_steering_rad = None
        self.last_verified_lateral_stop_steering_time_sec = None
        self.last_verified_baseline_stop_steering_rad = None
        self.last_verified_baseline_stop_steering_time_sec = None
        self.last_verified_lateral_stop_plan_generation = None
        self.last_verified_baseline_stop_plan_generation = None
        if not preserve_baseline_free_run:
            self._invalidate_verified_baseline_free_run_steering()

    def _invalidate_verified_baseline_free_run_steering(self) -> None:
        """Discard the short-lived nominal FREE_RUN steering reference."""
        self.last_verified_baseline_free_run_steering_rad = None
        self.last_verified_baseline_free_run_steering_time_sec = None
        self.last_verified_baseline_free_run_plan_generation = None
        self.last_verified_baseline_free_run_plan_stamp_ns = None
        self.last_verified_baseline_free_run_epoch = None

    def _invalidate_free_run_live_exact_record(self, reason: str) -> None:
        """Synchronously revoke provenance and its zero-speed gap lease."""
        self.free_run_source_provenance_record = None
        self.free_run_source_gap_lease = None
        self.free_run_pending_source = None
        self.free_run_source_gap_expired_successor_identity = None
        self.free_run_live_exact_record_reason = str(reason)
        self.free_run_live_exact_selected_envelope_valid = False
        self.free_run_live_exact_selected_envelope_reason = str(reason)
        self.free_run_live_exact_selected_envelope_identity = None

    # Kept as a private compatibility alias while the surrounding observer
    # code migrates to the explicitly split provenance/lease contract.
    @property
    def free_run_live_exact_published_record(
        self,
    ) -> Optional[FreeRunPublishedCommandRecord]:
        return self.free_run_source_provenance_record

    @free_run_live_exact_published_record.setter
    def free_run_live_exact_published_record(
        self, value: Optional[FreeRunPublishedCommandRecord]
    ) -> None:
        self.free_run_source_provenance_record = value

    def on_mpc_cmd(self, msg: AckermannControlCommand) -> None:
        receipt_time_sec, stamp_ns, timestamp_advanced = self._advance_receipt_time(
            self.mpc_cmd_time_sec,
            self.mpc_cmd_stamp_ns,
            self._stamp_ns(msg.stamp),
        )
        if not timestamp_advanced:
            return
        self.mpc_cmd = msg
        self.mpc_cmd_time_sec = receipt_time_sec
        self.mpc_cmd_stamp_ns = stamp_ns

    def on_pure_pursuit_cmd(self, msg: AckermannControlCommand) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.pure_pursuit_cmd_time_sec,
            self.pure_pursuit_cmd_stamp_ns,
            self._stamp_ns(msg.stamp),
        )
        if not timestamp_advanced:
            return
        self.pure_pursuit_cmd = msg
        self.pure_pursuit_cmd_time_sec = receipt_time_sec
        self.pure_pursuit_cmd_stamp_ns = stamp_ns
        self._refresh_pure_pursuit_envelope_parity()

    def on_state_lattice_control_cmd(
        self, msg: StateLatticeControlCommand
    ) -> None:
        """Accept one atomic State Lattice command without weakening identity."""
        stamp_ns = self._stamp_ns(msg.header.stamp)
        producer_instance_id = int(msg.producer_instance_id)
        command_sequence = int(msg.command_sequence)
        fingerprint = (
            int(msg.schema_version),
            producer_instance_id,
            command_sequence,
            int(msg.plan_generation),
            bool(msg.active),
            bool(msg.safety_evaluation_enabled),
            bool(msg.inputs_fresh),
            bool(msg.costmap_valid),
            bool(msg.trajectory_valid),
            bool(msg.trajectory_safe),
            bool(msg.stop_required),
            float(msg.command.longitudinal.speed),
            float(msg.command.longitudinal.acceleration),
            float(msg.command.longitudinal.jerk),
            float(msg.command.lateral.steering_tire_angle),
            float(msg.command.lateral.steering_tire_rotation_rate),
            str(msg.reason),
        )
        same_identity = bool(
            self.state_lattice_active_producer_instance_id
            == producer_instance_id
            and self.state_lattice_last_sequence == command_sequence
        )
        if same_identity:
            if fingerprint != self.state_lattice_last_fingerprint:
                self.state_lattice_cmd_valid = False
                self.state_lattice_cmd_reason = "duplicate_conflict"
            return

        valid = True
        reason = "valid"
        if int(msg.schema_version) != int(StateLatticeControlCommand.SCHEMA_V1):
            valid = False
            reason = "schema_invalid"
        elif (
            producer_instance_id <= 0
            or command_sequence <= 0
            or int(msg.plan_generation) <= 0
            or stamp_ns <= 0
        ):
            valid = False
            reason = "identity_invalid"
        elif str(msg.header.frame_id) != "map":
            valid = False
            reason = "frame_invalid"
        elif any(
            self._stamp_ns(candidate_stamp) != stamp_ns
            for candidate_stamp in (
                msg.command.stamp,
                msg.command.longitudinal.stamp,
                msg.command.lateral.stamp,
            )
        ):
            valid = False
            reason = "command_stamp_mismatch"
        elif (
            self.state_lattice_active_producer_instance_id is not None
            and producer_instance_id
            != self.state_lattice_active_producer_instance_id
            and self.core.state_lattice_episode_latched
        ):
            valid = False
            reason = "producer_changed_during_active_episode"
        elif (
            self.state_lattice_active_producer_instance_id
            == producer_instance_id
            and self.state_lattice_last_sequence is not None
            and command_sequence <= self.state_lattice_last_sequence
        ):
            valid = False
            reason = "sequence_regression"
        elif (
            self.state_lattice_active_producer_instance_id
            == producer_instance_id
            and self.state_lattice_last_plan_generation is not None
            and int(msg.plan_generation)
            not in (
                self.state_lattice_last_plan_generation,
                (
                    1
                    if self.state_lattice_last_plan_generation >= 16_777_215
                    else self.state_lattice_last_plan_generation + 1
                ),
            )
        ):
            valid = False
            reason = "plan_generation_discontinuity"
        elif (
            self.state_lattice_cmd_stamp_ns is not None
            and stamp_ns <= self.state_lattice_cmd_stamp_ns
        ):
            valid = False
            reason = "stamp_not_advanced"

        command_values = (
            float(msg.command.longitudinal.speed),
            float(msg.command.longitudinal.acceleration),
            float(msg.command.longitudinal.jerk),
            float(msg.command.lateral.steering_tire_angle),
            float(msg.command.lateral.steering_tire_rotation_rate),
        )
        if valid and not all(math.isfinite(value) for value in command_values):
            valid = False
            reason = "command_non_finite"
        elif valid and float(msg.command.longitudinal.speed) < 0.0:
            valid = False
            reason = "reverse_command_forbidden"
        elif valid and abs(float(msg.command.lateral.steering_tire_angle)) > (
            self.state_lattice_max_steering_angle_rad + 1.0e-6
        ):
            valid = False
            reason = "steering_angle_exceeded"
        elif valid and abs(
            float(msg.command.lateral.steering_tire_rotation_rate)
        ) > self.state_lattice_max_steering_rate_radps + 1.0e-6:
            valid = False
            reason = "steering_rate_exceeded"
        elif (
            valid
            and bool(msg.active)
            and self.state_lattice_cmd is not None
            and bool(self.state_lattice_cmd.active)
            and self.state_lattice_cmd_stamp_ns is not None
        ):
            sample_dt_sec = (
                stamp_ns - self.state_lattice_cmd_stamp_ns
            ) / 1.0e9
            steering_delta_rad = abs(
                float(msg.command.lateral.steering_tire_angle)
                - float(
                    self.state_lattice_cmd.command.lateral.steering_tire_angle
                )
            )
            if (
                not math.isfinite(sample_dt_sec)
                or sample_dt_sec <= 0.0
                or steering_delta_rad
                > self.state_lattice_max_steering_rate_radps
                * sample_dt_sec
                + 1.0e-4
            ):
                valid = False
                reason = "steering_transition_rate_exceeded"

        receipt_time_sec = self.now_sec()
        self.state_lattice_cmd = msg
        self.state_lattice_cmd_time_sec = receipt_time_sec
        self.state_lattice_cmd_stamp_ns = stamp_ns
        self.state_lattice_cmd_valid = valid
        self.state_lattice_cmd_reason = reason
        if valid:
            self.state_lattice_active_producer_instance_id = producer_instance_id
            self.state_lattice_last_sequence = command_sequence
            self.state_lattice_last_plan_generation = int(msg.plan_generation)
            self.state_lattice_last_fingerprint = fingerprint

    def on_pure_pursuit_tracking_status(
        self, msg: ControllerTrackingStatus
    ) -> None:
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        plan_generation = int(msg.plan_generation)
        status_key = (header_stamp_ns, plan_generation)
        maximum_stamp_ns = self.pure_pursuit_tracking_status_stamp_ns
        if (
            maximum_stamp_ns is not None
            and header_stamp_ns < maximum_stamp_ns
        ):
            return
        # A duplicate tuple is a retransmission, not a new proof.  In
        # particular, it must not extend freshness while /clock is frozen.
        if status_key in self.pure_pursuit_tracking_status_cache:
            return
        receipt_time_sec, stamp_ns, timestamp_advanced = self._advance_receipt_time(
            self.pure_pursuit_tracking_status_time_sec,
            self.pure_pursuit_tracking_status_stamp_ns,
            header_stamp_ns,
        )
        if not timestamp_advanced:
            # A new generation at an already observed command stamp is a
            # distinct transport proof.  Preserve the original receipt time:
            # accepting it must never hide an input timeout or clock stall.
            receipt_time_sec = self.pure_pursuit_tracking_status_time_sec
            stamp_ns = self.pure_pursuit_tracking_status_stamp_ns
        if receipt_time_sec is None or stamp_ns is None:
            return
        command_age_sec = float(msg.command_age_sec)
        lateral_stop_authority_kind = int(msg.lateral_stop_authority_kind)
        lateral_stop_transaction_pass_direction = int(
            msg.lateral_stop_transaction_pass_direction
        )
        lateral_stop_authority_token = int(msg.lateral_stop_authority_token)
        self.pure_pursuit_tracking_status = msg
        self.pure_pursuit_tracking_status_time_sec = receipt_time_sec
        self.pure_pursuit_tracking_status_stamp_ns = stamp_ns
        same_stamp_multiple_generations = any(
            cached_stamp_ns == header_stamp_ns
            and cached_generation != plan_generation
            for cached_stamp_ns, cached_generation in self.pure_pursuit_tracking_status_cache
        )
        status_valid = bool(
            str(msg.header.frame_id) == "base_link"
            and plan_generation > 0
            and math.isfinite(command_age_sec)
            and 0.0 <= command_age_sec
            <= self.pure_pursuit_tracking_status_timeout_sec
            and (
                not same_stamp_multiple_generations
                or self._tracking_status_binds_pure_pursuit_command(
                    msg, self.pure_pursuit_cmd
                )
            )
        )
        self.pure_pursuit_tracking_status_valid = status_valid
        self._remember_bounded(
            self.pure_pursuit_tracking_status_cache,
            status_key,
            (msg, receipt_time_sec, status_valid),
        )
        self._refresh_pure_pursuit_envelope_parity()

    def on_free_run_execution_ack(self, msg: FreeRunExecutionAck) -> None:
        """Observe one live exact FREE_RUN ACK without granting authority."""
        identity = (
            int(msg.plan_key.race_arm_epoch),
            int(msg.plan_key.planner_instance_id),
            int(msg.plan_key.plan_generation),
            self._stamp_ns(msg.plan_key.plan_stamp),
            int(msg.source_key.baseline_instance_id),
            int(msg.source_key.controller_instance_id),
            int(msg.source_key.source_generation),
            self._stamp_ns(msg.source_key.source_stamp),
            int(msg.controller_sequence),
            self._stamp_ns(msg.controller_command_stamp),
        )
        wire_digest = self._free_run_execution_ack_wire_digest(msg)
        if wire_digest is None:
            self._invalidate_free_run_live_exact_record(
                "ack_canonical_v2_invalid"
            )
            self.free_run_live_exact_ack = msg
            self.free_run_live_exact_ack_valid = False
            self.free_run_live_exact_ack_reason = "canonical_v2_invalid"
            return
        previous_wire_digest = (
            self.free_run_live_exact_ack_wire_digest_cache.get(identity)
        )
        generation_key = identity[:3]
        if previous_wire_digest is not None:
            if previous_wire_digest != wire_digest:
                self.free_run_live_exact_tainted_generations.add(
                    generation_key
                )
                self._invalidate_free_run_live_exact_record(
                    "ack_duplicate_identity_payload_mutation"
                )
                self.free_run_live_exact_ack = msg
                self.free_run_live_exact_ack_valid = False
                self.free_run_live_exact_ack_reason = (
                    "duplicate_identity_payload_mutation"
                )
            # Neither identical retransmission nor conflicting duplicate may
            # refresh the original steady-clock receipt lease.
            return
        record = self.free_run_live_exact_published_record
        source_identity = (
            int(msg.source_key.baseline_instance_id),
            int(msg.source_key.controller_instance_id),
            int(msg.source_key.source_generation),
            self._stamp_ns(msg.source_key.source_stamp),
        )
        source_is_record_or_direct_successor = bool(
            isinstance(record, FreeRunPublishedCommandRecord)
            and (
                source_identity == tuple(record.source_identity)
                or self._free_run_source_is_direct_semantic_successor(
                    msg.source_key, record
                )
            )
        )
        bounded_same_generation_successor = bool(
            isinstance(record, FreeRunPublishedCommandRecord)
            and int(msg.plan_key.plan_generation)
            == int(record.plan_generation)
            and int(msg.plan_key.race_arm_epoch) == int(record.race_arm_epoch)
            and int(msg.plan_key.planner_instance_id)
            == int(record.planner_instance_id)
            and bytes(msg.plan_key.canonical_plan_payload_sha256)
            == bytes(record.canonical_plan_sha256)
            and source_is_record_or_direct_successor
            and bool(msg.ack_eligible)
            and int(msg.evidence_state)
            == int(FreeRunExecutionAck.EVIDENCE_COMPLETE)
        )
        if (
            self.free_run_live_exact_ack_identity is not None
            and tuple(self.free_run_live_exact_ack_identity) != identity
            and self.free_run_live_exact_published_record is not None
            and not bounded_same_generation_successor
        ):
            self._invalidate_free_run_live_exact_record(
                "ack_identity_changed"
            )
        self._remember_bounded(
            self.free_run_live_exact_ack_wire_digest_cache,
            identity,
            wire_digest,
        )
        self.free_run_live_exact_ack = msg
        self.free_run_live_exact_ack_time_sec = self.now_sec()
        self.free_run_live_exact_ack_identity = identity
        self._refresh_free_run_live_exact_observation()

    def on_free_run_source_key(self, msg: FreeRunSourceKey) -> None:
        """Observe the current baseline source independently of an ACK."""
        identity = (
            int(msg.baseline_instance_id),
            int(msg.controller_instance_id),
            int(msg.source_generation),
            self._stamp_ns(msg.source_stamp),
        )
        source_wire = self._canonical_free_run_source_key_wire(msg)
        if source_wire is None:
            self._invalidate_free_run_live_exact_record(
                "source_canonical_invalid"
            )
            self.free_run_live_exact_source_key = msg
            self.free_run_live_exact_source_key_valid = False
            self.free_run_live_exact_source_key_reason = "canonical_invalid"
            return
        wire_digest = hashlib.sha256(source_wire).digest()
        previous_wire_digest = (
            self.free_run_live_exact_source_wire_digest_cache.get(identity)
        )
        if previous_wire_digest is not None:
            if previous_wire_digest != wire_digest:
                self.free_run_live_exact_tainted_sources.add(identity)
                self._invalidate_free_run_live_exact_record(
                    "source_duplicate_identity_payload_mutation"
                )
                self.free_run_live_exact_source_key = msg
                self.free_run_live_exact_source_key_valid = False
                self.free_run_live_exact_source_key_reason = (
                    "duplicate_identity_payload_mutation"
                )
            elif (
                self.free_run_pending_source is not None
                and identity != self.free_run_pending_source.identity
            ):
                # Once N+1 is pending, replaying an older, already-cached key
                # is a transport regression, not a harmless retransmission.
                # Do not replace the latest observation, but revoke both the
                # pending transaction and its old steering provenance.
                self._invalidate_free_run_live_exact_record(
                    "source_generation_regression"
                )
                self.free_run_live_exact_source_key_valid = False
                self.free_run_live_exact_source_key_reason = (
                    "generation_regression"
                )
            return
        record = self.free_run_live_exact_published_record
        now_sec = self.now_sec()
        direct_semantic_successor = bool(
            isinstance(record, FreeRunPublishedCommandRecord)
            and self._free_run_source_is_direct_semantic_successor(msg, record)
            and math.isfinite(now_sec - record.publish_steady_time_sec)
            and 0.0
            <= now_sec - record.publish_steady_time_sec
            <= self.free_run_source_provenance_timeout_sec
        )
        if (
            self.free_run_live_exact_source_key_identity is not None
            and tuple(self.free_run_live_exact_source_key_identity) != identity
            and not direct_semantic_successor
        ):
            self._invalidate_free_run_live_exact_record("source_identity_changed")
        elif direct_semantic_successor:
            self.free_run_live_exact_record_reason = (
                "source_direct_semantic_successor_pending"
            )
        self._remember_bounded(
            self.free_run_live_exact_source_wire_digest_cache,
            identity,
            wire_digest,
        )
        self.free_run_live_exact_source_key = msg
        self.free_run_live_exact_source_key_time_sec = now_sec
        self.free_run_live_exact_source_key_identity = identity
        digests = (
            msg.baseline_reference_sha256,
            msg.controller_implementation_sha256,
            msg.controller_config_sha256,
        )
        self.free_run_live_exact_source_key_valid = bool(
            int(msg.canonical_algorithm_version)
            == int(FreeRunSourceKey.CANONICAL_ALGORITHM_V1)
            and all(value > 0 for value in identity[:3])
            and identity[3] >= 0
            and int(msg.original_point_count) >= 2
            and all(
                len(digest) == 32
                and any(int(value) != 0 for value in digest)
                for digest in digests
            )
            and identity not in self.free_run_live_exact_tainted_sources
        )
        self.free_run_live_exact_source_key_reason = (
            "current"
            if self.free_run_live_exact_source_key_valid
            else "identity_or_digest_invalid"
        )
        if direct_semantic_successor and self.free_run_live_exact_source_key_valid:
            source_semantic_identity = (
                self._free_run_source_semantic_identity(msg)
            )
            if source_semantic_identity is None:
                self._invalidate_free_run_live_exact_record(
                    "source_semantic_identity_invalid"
                )
                self.free_run_live_exact_source_key_valid = False
                self.free_run_live_exact_source_key_reason = (
                    "semantic_identity_invalid"
                )
                return
            # This is the only point that mints the bounded steering-hold
            # authority. Keep N+1 as pending transport observation while the
            # N record remains immutable. Neither object is rewritten in place.
            self.free_run_pending_source = PendingFreeRunSource(
                source_key=copy.deepcopy(msg),
                identity=identity,
                semantic_identity=source_semantic_identity,
                wire_sha256=wire_digest,
                receipt_steady_time_sec=now_sec,
            )
            self.free_run_source_gap_lease = FreeRunSourceGapLease(
                provenance_record=record,
                successor_source_identity=identity,
                acquired_steady_time_sec=now_sec,
            )
            self.free_run_source_gap_expired_successor_identity = None
            self.free_run_live_exact_record_reason = (
                "source_direct_semantic_successor_gap_lease"
            )
        self._refresh_free_run_live_exact_observation()

    def _refresh_free_run_live_exact_observation(self) -> None:
        """Validate schema and exact local rendezvous; observation only."""
        msg = self.free_run_live_exact_ack
        self.free_run_live_exact_ack_valid = False
        if not self.free_run_live_exact_observe_enabled:
            self.free_run_live_exact_ack_reason = "disabled"
            return
        if msg is None:
            self.free_run_live_exact_ack_reason = "missing"
            return

        def digest_present(values: object) -> bool:
            return len(values) == 32 and any(int(value) != 0 for value in values)

        plan_key = msg.plan_key
        source_key = msg.source_key
        plan_stamp_ns = self._stamp_ns(plan_key.plan_stamp)
        source_stamp_ns = self._stamp_ns(source_key.source_stamp)
        command_stamp_ns = self._stamp_ns(msg.controller_command_stamp)
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        generation_key = (
            int(plan_key.race_arm_epoch),
            int(plan_key.planner_instance_id),
            int(plan_key.plan_generation),
        )
        plan_digest = bytes(plan_key.canonical_plan_payload_sha256)
        computed_ack_digest = self._free_run_execution_ack_wire_digest(msg)
        previous_digest = self.free_run_live_exact_plan_digest_cache.get(
            generation_key
        )
        if previous_digest is not None and previous_digest != plan_digest:
            self.free_run_live_exact_tainted_generations.add(generation_key)
        self._remember_bounded(
            self.free_run_live_exact_plan_digest_cache,
            generation_key,
            plan_digest,
        )

        scalar_values = (
            float(msg.raw_steering_tire_angle_rad),
            float(msg.output_steering_tire_angle_rad),
            float(msg.hard_steering_tire_angle_limit_rad),
            float(msg.required_spatial_horizon_m),
            float(msg.available_spatial_horizon_m),
            float(msg.trajectory_progress_m),
        )
        commands_finite = all(
            math.isfinite(float(value))
            for value in (
                msg.raw_controller_command.lateral.steering_tire_angle,
                msg.raw_controller_command.lateral.steering_tire_rotation_rate,
                msg.raw_controller_command.longitudinal.speed,
                msg.raw_controller_command.longitudinal.acceleration,
                msg.raw_controller_command.longitudinal.jerk,
                msg.output_controller_command.lateral.steering_tire_angle,
                msg.output_controller_command.lateral.steering_tire_rotation_rate,
                msg.output_controller_command.longitudinal.speed,
                msg.output_controller_command.longitudinal.acceleration,
                msg.output_controller_command.longitudinal.jerk,
            )
        )
        exact_plan = self.overtake_plan_contract_cache.get(
            (int(plan_key.plan_generation), plan_stamp_ns)
        )
        exact_plan_digest = self.free_run_live_exact_plan_delivery_cache.get(
            (int(plan_key.plan_generation), plan_stamp_ns)
        )
        exact_constraint = self.safety_constraint_contract_cache.get(
            (int(plan_key.plan_generation), plan_stamp_ns)
        )
        current_source_key = self.free_run_live_exact_source_key
        current_source_identity = (
            self.free_run_live_exact_source_key_identity
        )
        source_identity = (
            int(source_key.baseline_instance_id),
            int(source_key.controller_instance_id),
            int(source_key.source_generation),
            source_stamp_ns,
        )
        source_key_payload_matches = bool(
            current_source_key is not None
            and current_source_identity == source_identity
            and self._canonical_free_run_source_key_wire(current_source_key)
            == self._canonical_free_run_source_key_wire(source_key)
        )
        now_sec = self.now_sec()
        base = msg.base_geometry
        applied = msg.applied_geometry
        geometry_indices_valid = bool(
            int(base.original_point_count) == int(source_key.original_point_count)
            and int(applied.original_point_count)
            == int(source_key.original_point_count)
            and 0 <= int(base.first_source_index)
            <= int(base.nearest_source_index)
            <= int(base.last_source_index)
            < int(base.original_point_count)
            and 0 <= int(applied.first_source_index)
            <= int(applied.nearest_source_index)
            <= int(applied.last_source_index)
            < int(applied.original_point_count)
            and len(base.points)
            == int(base.last_source_index) - int(base.first_source_index) + 1
            and len(applied.points)
            == int(applied.last_source_index)
            - int(applied.first_source_index)
            + 1
            and 2 <= len(base.points) <= 100
            and 2 <= len(applied.points) <= 100
        )
        exact_normal_free_run = bool(
            exact_plan is not None
            and exact_plan[0]
            and int(exact_plan[5]) == int(OvertakePlan.FREE_RUN)
            and int(exact_plan[6]) == 0
            and not str(exact_plan[7])
            and not bool(exact_plan[3])
            and not bool(exact_plan[4])
            and self.overtake_plan_attempt_cache.get(
                int(plan_key.plan_generation)
            )
            == 0
        )
        exact_released_constraint = bool(
            exact_constraint is not None
            and exact_constraint[0].valid
            and exact_constraint[0].release_authorized
            and not exact_constraint[0].stop_requested
        )

        checks = (
            (
                int(msg.schema_version) == int(FreeRunExecutionAck.SCHEMA_V2)
                and bool(msg.ack_eligible)
                and int(msg.evidence_state)
                == int(FreeRunExecutionAck.EVIDENCE_COMPLETE),
                "ack_schema_unsupported_or_evidence_invalid",
            ),
            (
                str(msg.header.frame_id) == "base_link"
                and header_stamp_ns == command_stamp_ns
                and self._stamp_ns(msg.output_controller_command.stamp)
                == command_stamp_ns
                and self._stamp_ns(msg.raw_controller_command.stamp)
                == command_stamp_ns,
                "command_stamp_mismatch",
            ),
            (
                int(plan_key.canonical_algorithm_version) == 1
                and int(source_key.canonical_algorithm_version) == 1
                and all(value > 0 for value in generation_key)
                and int(source_key.baseline_instance_id) > 0
                and int(source_key.controller_instance_id) > 0
                and int(source_key.source_generation) > 0
                and int(msg.controller_sequence) > 0
                and int(source_key.original_point_count) >= 2,
                "identity_invalid",
            ),
            (
                generation_key
                not in self.free_run_live_exact_tainted_generations
                and digest_present(plan_digest)
                and digest_present(source_key.baseline_reference_sha256)
                and digest_present(source_key.controller_implementation_sha256)
                and digest_present(source_key.controller_config_sha256)
                and digest_present(msg.ack_sha256),
                "digest_missing_or_mutated",
            ),
            (
                computed_ack_digest is not None
                and bytes(msg.ack_sha256) == computed_ack_digest,
                "ack_digest_mismatch",
            ),
            (
                exact_normal_free_run and exact_released_constraint,
                "plan_constraint_exact_pair_missing",
            ),
            (
                exact_plan_digest is not None
                and exact_plan_digest[0]
                and int(exact_plan_digest[2])
                == int(FreeRunPlanKey.CANONICAL_ALGORITHM_V1)
                and bytes(exact_plan_digest[1]) == plan_digest,
                "planner_proposal_digest_mismatch",
            ),
            (
                current_source_identity
                not in self.free_run_live_exact_tainted_sources
                and self.free_run_live_exact_source_key_valid
                and source_key_payload_matches,
                "current_source_key_mismatch",
            ),
            (
                self.free_run_live_exact_ack_time_sec is not None
                and self.free_run_live_exact_source_key_time_sec is not None
                and exact_plan is not None
                and exact_constraint is not None
                and 0.0
                <= now_sec - self.free_run_live_exact_ack_time_sec
                <= self.free_run_live_exact_evidence_timeout_sec
                and 0.0
                <= now_sec - self.free_run_live_exact_source_key_time_sec
                <= self.free_run_live_exact_evidence_timeout_sec
                and 0.0
                <= now_sec - float(exact_plan[1])
                <= self.free_run_live_exact_evidence_timeout_sec
                and 0.0
                <= now_sec - float(exact_constraint[1])
                <= self.free_run_live_exact_evidence_timeout_sec,
                "live_evidence_stale",
            ),
            (
                str(base.frame_id) == str(applied.frame_id)
                and str(base.frame_id)
                and self._stamp_ns(base.source_stamp) == source_stamp_ns
                and self._stamp_ns(applied.source_stamp) == source_stamp_ns
                and geometry_indices_valid
                and base.points == applied.points
                and bytes(base.geometry_sha256)
                == bytes(applied.geometry_sha256)
                and digest_present(base.geometry_sha256)
                and int(base.full_source_digest_state)
                == int(base.FULL_SOURCE_DIGEST_COMPLETE)
                and int(applied.full_source_digest_state)
                == int(applied.FULL_SOURCE_DIGEST_COMPLETE)
                and bytes(base.full_source_sha256)
                == bytes(source_key.baseline_reference_sha256)
                and bytes(applied.full_source_sha256)
                == bytes(source_key.baseline_reference_sha256),
                "geometry_binding_invalid",
            ),
            (
                all(math.isfinite(value) for value in scalar_values)
                and commands_finite
                and math.isclose(
                    float(msg.raw_controller_command.lateral.steering_tire_angle),
                    float(msg.raw_steering_tire_angle_rad),
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                )
                and math.isclose(
                    float(
                        msg.output_controller_command.lateral.steering_tire_angle
                    ),
                    float(msg.output_steering_tire_angle_rad),
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                )
                and digest_present(msg.raw_controller_command_sha256)
                and digest_present(msg.output_controller_command_sha256),
                "command_binding_invalid",
            ),
            (
                bytes(msg.raw_controller_command_sha256)
                == self._canonical_controller_command_digest(
                    msg.raw_controller_command
                )
                and bytes(msg.output_controller_command_sha256)
                == self._canonical_controller_command_digest(
                    msg.output_controller_command
                ),
                "controller_command_digest_mismatch",
            ),
            (
                math.isclose(
                    float(msg.hard_steering_tire_angle_limit_rad),
                    float(self.steering_limiter.config.max_steering_angle_rad),
                    rel_tol=0.0,
                    abs_tol=1.0e-6,
                )
                and abs(float(msg.raw_steering_tire_angle_rad))
                <= float(msg.hard_steering_tire_angle_limit_rad)
                and abs(float(msg.output_steering_tire_angle_rad))
                <= float(msg.hard_steering_tire_angle_limit_rad),
                "actuator_limit_invalid",
            ),
            (
                float(msg.required_spatial_horizon_m) > 0.0
                and float(msg.available_spatial_horizon_m)
                >= float(msg.required_spatial_horizon_m)
                and not bool(base.lookahead_endpoint_fallback)
                and not bool(applied.lookahead_endpoint_fallback),
                "spatial_horizon_invalid",
            ),
        )
        for valid, reason in checks:
            if not valid:
                self.free_run_live_exact_ack_reason = reason
                return
        self.free_run_live_exact_ack_valid = True
        self.free_run_live_exact_ack_reason = "complete_observe_only"

    @staticmethod
    def _canonical_free_run_plan_digest(
        msg: OvertakePlan,
    ) -> Optional[bytes]:
        """Mirror FREE_RUN_PLAN_PAYLOAD_V1 for independent Mux verification."""
        reasons = tuple(str(reason) for reason in msg.authorization_failure_reasons)
        strings = (
            str(msg.header.frame_id),
            str(msg.decision_reason),
            str(msg.candidate_reject_reason),
            str(msg.constraint_reason),
            *reasons,
        )
        if (
            str(msg.header.frame_id) != "map"
            or any(len(value.encode()) > 256 for value in strings[1:])
            or len(strings[0].encode()) > 128
            or len(reasons) > 64
            or int(msg.phase) != int(OvertakePlan.FREE_RUN)
            or int(msg.attempt_id) != 0
            or str(msg.target_vehicle_id)
            or int(msg.pass_direction) != 0
            or bool(msg.trajectory_authorized)
            or bool(msg.lateral_maneuver_required)
            or int(msg.lateral_stop_authority_kind)
            != int(OvertakePlan.LATERAL_STOP_NONE)
            or int(msg.lateral_stop_transaction_pass_direction) != 0
            or int(msg.lateral_stop_authority_token) != 0
            or bool(msg.trajectory.points)
            or str(msg.trajectory.header.frame_id)
            not in ("", str(msg.header.frame_id))
        ):
            return None

        wire = bytearray()

        def append_u8(value: int) -> None:
            wire.extend(struct.pack("<B", value))

        def append_i8(value: int) -> None:
            wire.extend(struct.pack("<b", value))

        def append_u32(value: int) -> None:
            wire.extend(struct.pack("<I", value))

        def append_u64(value: int) -> None:
            wire.extend(struct.pack("<Q", value))

        def append_bool(value: bool) -> None:
            append_u8(1 if value else 0)

        def append_string(value: str) -> None:
            encoded = value.encode()
            append_u32(len(encoded))
            wire.extend(encoded)

        append_string("FREE_RUN_PLAN_PAYLOAD_V1")
        append_string(str(msg.header.frame_id))
        append_u8(int(msg.phase))
        append_u32(int(msg.attempt_id))
        append_string(str(msg.target_vehicle_id))
        append_i8(int(msg.pass_direction))
        append_bool(bool(msg.trajectory_authorized))
        append_bool(bool(msg.lateral_maneuver_required))
        append_string(str(msg.decision_reason))
        append_u32(int(msg.authorization_failure_mask))
        append_u32(len(reasons))
        for reason in reasons:
            append_string(reason)
        append_string(str(msg.candidate_reject_reason))
        append_bool(bool(msg.safety_inputs_complete))
        append_bool(bool(msg.tracking_usable))
        append_bool(bool(msg.trajectory_publishable))
        append_string(str(msg.constraint_reason))
        append_u8(int(msg.lateral_stop_authority_kind))
        append_i8(int(msg.lateral_stop_transaction_pass_direction))
        append_u64(int(msg.lateral_stop_authority_token))
        append_string(str(msg.trajectory.header.frame_id))
        append_u32(0)
        append_u8(int(msg.aw2_identity_schema_version))
        append_u64(int(msg.planner_instance_id))
        append_u64(int(msg.race_arm_epoch))
        append_u64(int(msg.connector_transaction_id))
        append_u32(int(msg.candidate_revision))
        wire.extend(bytes(msg.candidate_content_sha256))
        return hashlib.sha256(bytes(wire)).digest()

    @staticmethod
    def _canonical_free_run_source_key_wire(
        source_key: FreeRunSourceKey,
    ) -> Optional[bytes]:
        try:
            digests = (
                bytes(source_key.baseline_reference_sha256),
                bytes(source_key.controller_implementation_sha256),
                bytes(source_key.controller_config_sha256),
            )
            if (
                any(len(digest) != 32 for digest in digests)
                or int(source_key.source_stamp.sec) < 0
                or not 0
                <= int(source_key.source_stamp.nanosec)
                < 1_000_000_000
            ):
                return None
            wire = bytearray()
            wire.extend(
                struct.pack(
                    "<BQQIiII",
                    int(source_key.canonical_algorithm_version),
                    int(source_key.baseline_instance_id),
                    int(source_key.controller_instance_id),
                    int(source_key.source_generation),
                    int(source_key.source_stamp.sec),
                    int(source_key.source_stamp.nanosec),
                    int(source_key.original_point_count),
                )
            )
            for digest in digests:
                wire.extend(digest)
            return bytes(wire)
        except (OverflowError, struct.error, TypeError, ValueError):
            return None

    @staticmethod
    def _free_run_source_semantic_identity(
        source_key: FreeRunSourceKey,
    ) -> Optional[tuple]:
        """Return immutable source semantics, excluding delivery generation/stamp."""
        wire = HybridControlMuxNode._canonical_free_run_source_key_wire(
            source_key
        )
        if wire is None:
            return None
        digests = (
            bytes(source_key.baseline_reference_sha256),
            bytes(source_key.controller_implementation_sha256),
            bytes(source_key.controller_config_sha256),
        )
        return (
            int(source_key.canonical_algorithm_version),
            int(source_key.baseline_instance_id),
            int(source_key.controller_instance_id),
            int(source_key.original_point_count),
            *digests,
        )

    def _free_run_source_is_direct_semantic_successor(
        self,
        source_key: FreeRunSourceKey,
        record: FreeRunPublishedCommandRecord,
    ) -> bool:
        """Recognize one delivery successor without rewriting old provenance."""
        semantic_identity = self._free_run_source_semantic_identity(source_key)
        source_stamp_ns = self._stamp_ns(source_key.source_stamp)
        old_identity = tuple(record.source_identity)
        if len(old_identity) != 4:
            return False
        old_generation = int(old_identity[2])
        return bool(
            semantic_identity is not None
            and semantic_identity == tuple(record.source_semantic_identity)
            and old_generation < 0xFFFFFFFF
            and int(source_key.source_generation) == old_generation + 1
            and int(source_key.baseline_instance_id) == int(old_identity[0])
            and int(source_key.controller_instance_id) == int(old_identity[1])
            and source_stamp_ns > int(old_identity[3])
        )

    def _current_free_run_source_is_direct_semantic_successor(
        self,
        record: FreeRunPublishedCommandRecord,
    ) -> bool:
        source_key = self.free_run_live_exact_source_key
        return bool(
            source_key is not None
            and self.free_run_live_exact_source_key_valid
            and self.free_run_live_exact_source_key_identity
            not in self.free_run_live_exact_tainted_sources
            and self._free_run_source_is_direct_semantic_successor(
                source_key, record
            )
        )

    def _free_run_source_delivery_gap_pending(
        self,
        *,
        tracking_plan_generation: int,
        now_sec: float,
    ) -> bool:
        """Keep only old final steering while a direct SourceKey successor joins."""
        lease = self.free_run_source_gap_lease
        pending_source = self.free_run_pending_source
        record = (
            lease.provenance_record
            if isinstance(lease, FreeRunSourceGapLease)
            else None
        )
        current_plan_stamp_ns = self.overtake_plan_header_stamp_ns
        current_source_identity = tuple(
            self.free_run_live_exact_source_key_identity or ()
        )
        ack = self.free_run_live_exact_ack
        ack_source_identity = (
            (
                int(ack.source_key.baseline_instance_id),
                int(ack.source_key.controller_instance_id),
                int(ack.source_key.source_generation),
                self._stamp_ns(ack.source_key.source_stamp),
            )
            if ack is not None
            else ()
        )
        if (
            not isinstance(record, FreeRunPublishedCommandRecord)
            or not isinstance(lease, FreeRunSourceGapLease)
            or not isinstance(pending_source, PendingFreeRunSource)
            or current_plan_stamp_ns is None
            or not self._current_free_run_source_is_direct_semantic_successor(
                record
            )
        ):
            return False
        current_plan_delivery = self.free_run_live_exact_plan_delivery_cache.get(
            (int(tracking_plan_generation), int(current_plan_stamp_ns))
        )
        lease_age_sec = now_sec - lease.acquired_steady_time_sec
        return bool(
            int(tracking_plan_generation) == int(record.plan_generation)
            and int(current_plan_stamp_ns) == int(record.plan_stamp_ns)
            and current_plan_delivery is not None
            and bool(current_plan_delivery[0])
            and bytes(current_plan_delivery[1])
            == bytes(record.canonical_plan_sha256)
            and current_source_identity != ack_source_identity
            and current_source_identity
            == tuple(lease.successor_source_identity)
            and current_source_identity == tuple(pending_source.identity)
            and pending_source.source_key
            == self.free_run_live_exact_source_key
            and pending_source.wire_sha256
            == self.free_run_live_exact_source_wire_digest_cache.get(
                current_source_identity
            )
            and self._fresh(
                self.free_run_live_exact_source_key_time_sec,
                now_sec,
                self.free_run_live_exact_evidence_timeout_sec,
            )
            and math.isfinite(lease_age_sec)
            and 0.0
            <= lease_age_sec
            <= self.free_run_live_exact_evidence_timeout_sec
            and hashlib.sha256(record.final_command_cdr).digest()
            == record.final_command_sha256
        )

    @staticmethod
    def _canonical_free_run_execution_ack_v2(
        msg: FreeRunExecutionAck,
    ) -> Optional[bytes]:
        wire = bytearray(b"FREE_RUN_EXECUTION_ACK_CANONICAL_V2")

        def append_u8(value: int) -> None:
            wire.extend(struct.pack("<B", int(value)))

        def append_u32(value: int) -> None:
            wire.extend(struct.pack("<I", int(value)))

        def append_u64(value: int) -> None:
            wire.extend(struct.pack("<Q", int(value)))

        def append_f32(value: float) -> None:
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("non-finite float32")
            wire.extend(struct.pack("<f", value))

        def append_f64(value: float) -> None:
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("non-finite float64")
            wire.extend(struct.pack("<d", value))

        def append_time(stamp: object) -> None:
            sec = int(stamp.sec)
            nanosec = int(stamp.nanosec)
            if sec < 0 or not 0 <= nanosec < 1_000_000_000:
                raise ValueError("invalid time")
            wire.extend(struct.pack("<iI", sec, nanosec))

        def append_string(value: str, max_bytes: Optional[int] = None) -> None:
            encoded = str(value).encode("utf-8")
            if max_bytes is not None and len(encoded) > max_bytes:
                raise ValueError("bounded string exceeded")
            append_u32(len(encoded))
            wire.extend(encoded)

        def append_digest(value: object) -> None:
            digest = bytes(value)
            if len(digest) != 32:
                raise ValueError("invalid digest width")
            wire.extend(digest)

        def append_point(point: object) -> None:
            append_time(point.time_from_start)
            for value in (
                point.position_x_m,
                point.position_y_m,
                point.position_z_m,
                point.orientation_x,
                point.orientation_y,
                point.orientation_z,
                point.orientation_w,
            ):
                append_f64(value)
            quaternion_norm = math.sqrt(
                float(point.orientation_x) ** 2
                + float(point.orientation_y) ** 2
                + float(point.orientation_z) ** 2
                + float(point.orientation_w) ** 2
            )
            if abs(quaternion_norm - 1.0) > 1.0e-6:
                raise ValueError("invalid point quaternion")
            for value in (
                point.longitudinal_velocity_mps,
                point.lateral_velocity_mps,
                point.acceleration_mps2,
                point.heading_rate_rps,
                point.front_wheel_angle_rad,
                point.rear_wheel_angle_rad,
            ):
                append_f32(value)

        def append_geometry(geometry: object) -> None:
            point_count = len(geometry.points)
            if (
                not str(geometry.frame_id)
                or not 2 <= point_count <= 100
                or int(geometry.original_point_count) < 2
                or int(geometry.first_source_index)
                > int(geometry.nearest_source_index)
                or int(geometry.nearest_source_index)
                > int(geometry.last_source_index)
                or int(geometry.last_source_index)
                >= int(geometry.original_point_count)
                or any(
                    int(index) < int(geometry.first_source_index)
                    or int(index) > int(geometry.last_source_index)
                    for index in (
                        geometry.speed_cap_source_index,
                        geometry.curvature_last_read_source_index,
                        geometry.lookahead_selected_source_index,
                        geometry.required_horizon_end_source_index,
                    )
                )
                or point_count
                != int(geometry.last_source_index)
                - int(geometry.first_source_index)
                + 1
                or not math.isfinite(float(geometry.total_arc_length_m))
                or float(geometry.total_arc_length_m) < 0.0
                or not math.isfinite(
                    float(geometry.required_spatial_horizon_m)
                )
                or float(geometry.required_spatial_horizon_m) <= 0.0
                or any(
                    not any(int(value) != 0 for value in digest)
                    for digest in (
                        geometry.geometry_sha256,
                        geometry.start_point_sha256,
                        geometry.end_point_sha256,
                        geometry.full_source_sha256,
                    )
                )
            ):
                raise ValueError("invalid geometry bounds")
            append_string(geometry.frame_id, 128)
            append_time(geometry.source_stamp)
            for value in (
                geometry.original_point_count,
                geometry.first_source_index,
                geometry.last_source_index,
                geometry.nearest_source_index,
                geometry.speed_cap_source_index,
                geometry.curvature_last_read_source_index,
                geometry.lookahead_selected_source_index,
                geometry.required_horizon_end_source_index,
            ):
                append_u32(value)
            append_u8(1 if geometry.lookahead_endpoint_fallback else 0)
            append_u32(point_count)
            for point in geometry.points:
                append_point(point)
            append_digest(geometry.geometry_sha256)
            append_f64(geometry.total_arc_length_m)
            append_f64(geometry.required_spatial_horizon_m)
            append_digest(geometry.start_point_sha256)
            append_digest(geometry.end_point_sha256)
            append_u8(geometry.full_source_digest_state)
            append_digest(geometry.full_source_sha256)

        def append_pose(pose: object) -> None:
            for value in (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ):
                append_f64(value)
            quaternion_norm = math.sqrt(
                float(pose.orientation.x) ** 2
                + float(pose.orientation.y) ** 2
                + float(pose.orientation.z) ** 2
                + float(pose.orientation.w) ** 2
            )
            if abs(quaternion_norm - 1.0) > 1.0e-6:
                raise ValueError("invalid control pose quaternion")

        def append_command(command: AckermannControlCommand) -> None:
            append_time(command.stamp)
            append_time(command.lateral.stamp)
            append_f32(command.lateral.steering_tire_angle)
            append_f32(command.lateral.steering_tire_rotation_rate)
            append_time(command.longitudinal.stamp)
            append_f32(command.longitudinal.speed)
            append_f32(command.longitudinal.acceleration)
            append_f32(command.longitudinal.jerk)

        try:
            plan = msg.plan_key
            source = msg.source_key
            if (
                int(msg.schema_version)
                != int(FreeRunExecutionAck.SCHEMA_V2)
                or not bool(msg.ack_eligible)
                or int(msg.evidence_state)
                != int(FreeRunExecutionAck.EVIDENCE_COMPLETE)
                or not str(msg.header.frame_id)
                or int(plan.canonical_algorithm_version) == 0
                or int(plan.race_arm_epoch) <= 0
                or int(plan.planner_instance_id) <= 0
                or int(plan.plan_generation) <= 0
                or int(source.canonical_algorithm_version) == 0
                or int(source.baseline_instance_id) <= 0
                or int(source.controller_instance_id) <= 0
                or int(source.source_generation) <= 0
                or int(source.original_point_count) < 2
                or int(msg.controller_sequence) <= 0
                or not all(
                    any(int(value) != 0 for value in digest)
                    for digest in (
                        plan.canonical_plan_payload_sha256,
                        source.baseline_reference_sha256,
                        source.controller_implementation_sha256,
                        source.controller_config_sha256,
                        msg.raw_controller_command_sha256,
                        msg.output_controller_command_sha256,
                    )
                )
                or not math.isfinite(
                    float(msg.hard_steering_tire_angle_limit_rad)
                )
                or float(msg.hard_steering_tire_angle_limit_rad) <= 0.0
                or not math.isfinite(
                    float(msg.required_spatial_horizon_m)
                )
                or float(msg.required_spatial_horizon_m) <= 0.0
                or not math.isfinite(
                    float(msg.available_spatial_horizon_m)
                )
                or float(msg.available_spatial_horizon_m)
                < float(msg.required_spatial_horizon_m)
            ):
                return None
            append_time(msg.header.stamp)
            append_string(msg.header.frame_id)
            append_u8(msg.schema_version)
            append_u8(1 if msg.ack_eligible else 0)
            append_u8(msg.evidence_state)
            append_string(msg.evidence_reason, 256)

            append_u8(plan.canonical_algorithm_version)
            append_u64(plan.race_arm_epoch)
            append_u64(plan.planner_instance_id)
            append_u32(plan.plan_generation)
            append_time(plan.plan_stamp)
            append_digest(plan.canonical_plan_payload_sha256)

            source_wire = HybridControlMuxNode._canonical_free_run_source_key_wire(
                msg.source_key
            )
            if source_wire is None:
                raise ValueError("invalid source key")
            wire.extend(source_wire)

            append_u64(msg.controller_sequence)
            append_time(msg.controller_command_stamp)
            append_geometry(msg.base_geometry)
            append_geometry(msg.applied_geometry)
            append_pose(msg.control_pose)
            append_time(msg.control_pose_stamp)
            append_u32(msg.nearest_trajectory_index)
            append_f64(msg.trajectory_progress_m)
            append_command(msg.raw_controller_command)
            append_command(msg.output_controller_command)
            append_digest(msg.raw_controller_command_sha256)
            append_digest(msg.output_controller_command_sha256)
            append_f64(msg.raw_steering_tire_angle_rad)
            append_f64(msg.output_steering_tire_angle_rad)
            append_f64(msg.hard_steering_tire_angle_limit_rad)
            append_f64(msg.required_spatial_horizon_m)
            append_f64(msg.available_spatial_horizon_m)
            wire.extend(bytes(32))
        except (OverflowError, struct.error, TypeError, ValueError):
            return None
        return bytes(wire)

    @staticmethod
    def _free_run_execution_ack_wire_digest(
        msg: FreeRunExecutionAck,
    ) -> Optional[bytes]:
        if int(msg.schema_version) != int(FreeRunExecutionAck.SCHEMA_V2):
            return None
        wire = HybridControlMuxNode._canonical_free_run_execution_ack_v2(msg)
        return None if wire is None else hashlib.sha256(wire).digest()

    @staticmethod
    def _canonical_controller_command_digest(
        command: AckermannControlCommand,
    ) -> bytes:
        """Mirror AW2:CONTROLLER_COMMAND:v1 from simple_pure_pursuit."""
        stamps = (
            command.stamp,
            command.lateral.stamp,
            command.longitudinal.stamp,
        )
        values = (
            command.lateral.steering_tire_angle,
            command.lateral.steering_tire_rotation_rate,
            command.longitudinal.speed,
            command.longitudinal.acceleration,
            command.longitudinal.jerk,
        )
        if (
            any(
                int(stamp.sec) < 0
                or not 0 <= int(stamp.nanosec) < 1_000_000_000
                for stamp in stamps
            )
            or not all(math.isfinite(float(value)) for value in values)
        ):
            return bytes(32)
        wire = bytearray(b"AW2:CONTROLLER_COMMAND:v1")

        def append_stamp(stamp: object) -> None:
            wire.extend(
                struct.pack(
                    "<iI", int(stamp.sec), int(stamp.nanosec)
                )
            )

        append_stamp(command.stamp)
        append_stamp(command.lateral.stamp)
        wire.extend(
            struct.pack(
                "<ff",
                float(command.lateral.steering_tire_angle),
                float(command.lateral.steering_tire_rotation_rate),
            )
        )
        append_stamp(command.longitudinal.stamp)
        wire.extend(
            struct.pack(
                "<fff",
                float(command.longitudinal.speed),
                float(command.longitudinal.acceleration),
                float(command.longitudinal.jerk),
            )
        )
        return hashlib.sha256(bytes(wire)).digest()

    @staticmethod
    def _mux_runtime_cycle_record_v2_fields(
        *,
        selected_input_cmd: Optional[AckermannControlCommand],
        selected_pp_motion_sample: Optional[PurePursuitMotionSample],
        selected_plan_identity: Optional[tuple],
        grant_published: bool,
        grant_valid: bool,
        grant_sequence: Optional[int],
        grant_commit_steady_ns: Optional[int],
        grant_issuer_instance_id: Optional[int],
        final_command: AckermannControlCommand,
        steering_result: SteeringLimitResult,
        steering_limiter_config: SteeringLimiterConfig,
    ) -> dict[str, object]:
        """Return opt-in v2 provenance for the command just published."""
        pp_identity = (
            tuple(selected_pp_motion_sample.envelope_identity)
            if selected_pp_motion_sample is not None
            and selected_pp_motion_sample.envelope_identity is not None
            else None
        )
        validated_plan_identity = (
            tuple(selected_plan_identity)
            if selected_plan_identity is not None
            else None
        )
        final_command_cdr = bytes(serialize_message(final_command))
        final_command_canonical_sha256 = (
            HybridControlMuxNode._canonical_controller_command_digest(final_command)
        )
        final_command_cdr_sha256 = hashlib.sha256(final_command_cdr).digest()
        return {
            "selected_pp_identity": {
                "present": pp_identity is not None,
                "producer_instance_id": (
                    int(pp_identity[0]) if pp_identity is not None else None
                ),
                "command_sequence": (
                    int(pp_identity[1]) if pp_identity is not None else None
                ),
                "command_stamp_ns": (
                    int(pp_identity[2]) if pp_identity is not None else None
                ),
                "plan_generation": (
                    int(pp_identity[3]) if pp_identity is not None else None
                ),
            },
            "selected_plan_sample_identity": {
                "present": validated_plan_identity is not None,
                "race_arm_epoch": (
                    int(validated_plan_identity[0])
                    if validated_plan_identity is not None
                    else None
                ),
                "planner_instance_id": (
                    int(validated_plan_identity[1])
                    if validated_plan_identity is not None
                    else None
                ),
                "attempt_id": (
                    int(validated_plan_identity[2])
                    if validated_plan_identity is not None
                    else None
                ),
                "target_vehicle_id": (
                    str(validated_plan_identity[3])
                    if validated_plan_identity is not None
                    else None
                ),
                "pass_direction": (
                    int(validated_plan_identity[4])
                    if validated_plan_identity is not None
                    else None
                ),
                "connector_transaction_id": (
                    int(validated_plan_identity[5])
                    if validated_plan_identity is not None
                    else None
                ),
                "plan_stamp_ns": (
                    int(validated_plan_identity[6])
                    if validated_plan_identity is not None
                    else None
                ),
                "plan_generation": (
                    int(validated_plan_identity[7])
                    if validated_plan_identity is not None
                    else None
                ),
                "phase": (
                    int(validated_plan_identity[8])
                    if validated_plan_identity is not None
                    else None
                ),
                "candidate_revision": (
                    int(validated_plan_identity[9])
                    if validated_plan_identity is not None
                    else None
                ),
                "candidate_content_sha256": (
                    bytes(validated_plan_identity[10]).hex()
                    if validated_plan_identity is not None
                    else None
                ),
                "identity_valid": (
                    bool(validated_plan_identity[11])
                    if validated_plan_identity is not None
                    else None
                ),
            },
            "selected_input_command_canonical_sha256": (
                HybridControlMuxNode._canonical_controller_command_digest(
                    selected_input_cmd
                ).hex()
                if selected_input_cmd is not None
                else None
            ),
            "final_command_canonical_sha256": final_command_canonical_sha256.hex(),
            "final_command_cdr_sha256": final_command_cdr_sha256.hex(),
            "limiter": {
                "raw_steering_rad": float(steering_result.raw_steering_rad),
                "limited_steering_rad": float(
                    steering_result.limited_steering_rad
                ),
                "steering_delta_rad": float(steering_result.steering_delta_rad),
                "angle_limited": bool(steering_result.angle_limited),
                "rate_limited": bool(steering_result.rate_limited),
                "reset": bool(steering_result.limiter_reset),
                "enabled": bool(steering_limiter_config.enabled),
                "max_steering_angle_rad": float(
                    steering_limiter_config.max_steering_angle_rad
                ),
                "max_steering_rate_radps": float(
                    steering_limiter_config.max_steering_rate_radps
                ),
                "max_steering_delta_per_cycle_rad": float(
                    steering_limiter_config.max_steering_delta_per_cycle
                ),
                "reset_dt_threshold_sec": float(
                    steering_limiter_config.reset_dt_threshold_sec
                ),
                "reset_on_mode_change": bool(
                    steering_limiter_config.reset_on_mode_change
                ),
            },
            "grant": {
                "present": bool(grant_published),
                "issuer_instance_id": (
                    int(grant_issuer_instance_id) if grant_published else None
                ),
                "sequence": int(grant_sequence) if grant_published else None,
                "valid": bool(grant_valid) if grant_published else None,
                "commit_steady_ns": (
                    int(grant_commit_steady_ns)
                    if grant_published and grant_valid
                    and grant_commit_steady_ns is not None
                    and int(grant_commit_steady_ns) >= 0
                    else None
                ),
                "final_command_cdr_sha256": (
                    final_command_cdr_sha256.hex()
                    if grant_published and grant_valid
                    else None
                ),
            },
        }

    def _refresh_free_run_selected_envelope_join(
        self,
        sample: Optional[PurePursuitMotionSample],
        *,
        tracking_plan_generation: int,
        now_sec: float,
    ) -> Optional[ControllerCommandEnvelope]:
        """Join the current ACK to the exact envelope selected by Mux."""
        self.free_run_live_exact_selected_envelope_valid = False
        self.free_run_live_exact_selected_envelope_identity = None
        if not self.free_run_live_exact_observe_enabled:
            self.free_run_live_exact_selected_envelope_reason = "disabled"
            return None
        # A current N+1 plan/constraint pair can arrive one transport tick
        # before its exact PP envelope/proof.  This only identifies a bounded
        # zero-speed steering hold; it never supplies motion authority.
        if self._free_run_forward_pre_ack_transition(
            sample,
            tracking_plan_generation=tracking_plan_generation,
        ):
            self.free_run_live_exact_selected_envelope_reason = (
                "forward_pre_ack_transition"
            )
            self.free_run_live_exact_selected_envelope_identity = tuple(
                sample.envelope_identity
                if sample is not None and sample.envelope_identity is not None
                else self.free_run_live_exact_published_record.envelope_identity
            )
            return None
        if not self.free_run_live_exact_ack_valid:
            self.free_run_live_exact_selected_envelope_reason = (
                "ack_" + self.free_run_live_exact_ack_reason
            )
            return None
        ack = self.free_run_live_exact_ack
        if ack is None:
            self.free_run_live_exact_selected_envelope_reason = "ack_missing"
            return None
        envelope = self._exact_current_motion_envelope(
            sample,
            tracking_plan_generation=tracking_plan_generation,
            now_sec=now_sec,
        )
        if envelope is None or sample is None or sample.envelope_identity is None:
            self.free_run_live_exact_selected_envelope_reason = (
                "selected_envelope_missing"
            )
            if self.free_run_live_exact_published_record is not None:
                self._invalidate_free_run_live_exact_record(
                    "selected_envelope_missing"
                )
            return None
        identity = tuple(sample.envelope_identity)
        forward_pre_ack_transition = (
            self._free_run_forward_pre_ack_transition(
                sample,
                tracking_plan_generation=tracking_plan_generation,
            )
        )
        checks = (
            (
                int(ack.source_key.baseline_instance_id)
                == int(ack.source_key.controller_instance_id)
                == int(identity[0])
                == int(envelope.producer_instance_id),
                "producer_mismatch",
            ),
            (
                int(ack.controller_sequence)
                == int(identity[1])
                == int(envelope.command_sequence),
                "sequence_mismatch",
            ),
            (
                self._stamp_ns(ack.controller_command_stamp)
                == int(identity[2])
                == self._stamp_ns(envelope.header.stamp)
                == self._stamp_ns(envelope.command.stamp),
                "stamp_mismatch",
            ),
            (
                int(ack.plan_key.plan_generation)
                == int(identity[3])
                == int(envelope.plan_generation)
                == int(tracking_plan_generation),
                "generation_mismatch",
            ),
            (
                self._canonical_controller_command_digest(
                    ack.output_controller_command
                )
                == self._canonical_controller_command_digest(envelope.command)
                == self._canonical_controller_command_digest(sample.command)
                != bytes(32),
                "command_mismatch",
            ),
            (
                bytes(ack.output_controller_command_sha256)
                == self._canonical_controller_command_digest(
                    ack.output_controller_command
                )
                == self._canonical_controller_command_digest(envelope.command),
                "output_command_digest_mismatch",
            ),
        )
        for valid, reason in checks:
            if not valid:
                if forward_pre_ack_transition:
                    self.free_run_live_exact_selected_envelope_reason = (
                        "forward_pre_ack_transition"
                    )
                    self.free_run_live_exact_selected_envelope_identity = (
                        identity
                    )
                    return None
                self.free_run_live_exact_selected_envelope_reason = reason
                if self.free_run_live_exact_published_record is not None:
                    self._invalidate_free_run_live_exact_record(
                        "ack_selected_envelope_" + reason
                    )
                return None
        self.free_run_live_exact_selected_envelope_valid = True
        self.free_run_live_exact_selected_envelope_reason = "exact"
        self.free_run_live_exact_selected_envelope_identity = identity
        return envelope

    def _free_run_forward_pre_ack_transition(
        self,
        sample: Optional[PurePursuitMotionSample],
        *,
        tracking_plan_generation: int,
    ) -> bool:
        """Recognize only the bounded same-semantic N -> N+1 delivery gap."""
        record = self.free_run_live_exact_published_record
        if (
            record is None
            or self.overtake_plan_header_stamp_ns is None
            or self.free_run_live_exact_source_key_identity is None
        ):
            self.free_run_live_exact_forward_transition_reason = (
                "prerequisite_missing"
            )
            return False
        identity = (
            tuple(sample.envelope_identity)
            if sample is not None and sample.envelope_identity is not None
            else tuple(record.envelope_identity)
        )
        current_plan_stamp_ns = int(self.overtake_plan_header_stamp_ns)
        delivery = self.free_run_live_exact_plan_delivery_cache.get(
            (int(tracking_plan_generation), current_plan_stamp_ns)
        )
        plan_identity = self.overtake_plan_motion_identity_cache.get(
            int(tracking_plan_generation)
        )
        sample_steering_rad = (
            float(sample.command.lateral.steering_tire_angle)
            if sample is not None
            else None
        )
        plan_delta_ns = current_plan_stamp_ns - int(record.plan_stamp_ns)
        checks = (
            (len(identity) == 4, "identity_size"),
            (
                int(tracking_plan_generation)
                == int(record.plan_generation) + 1,
                "tracking_generation",
            ),
            (
                int(identity[0]) == int(record.envelope_identity[0])
                and int(identity[1]) >= int(record.envelope_identity[1])
                and int(identity[2]) >= int(record.envelope_identity[2]),
                "old_envelope_successor_mismatch",
            ),
            (
                int(identity[3]) == int(record.plan_generation),
                "identity_generation",
            ),
            (
                0 < plan_delta_ns
                <= int(
                    self.free_run_live_exact_evidence_timeout_sec * 1.0e9
                ),
                "plan_stamp_delta",
            ),
            (delivery is not None, "plan_delivery_missing"),
            (
                delivery is not None and bool(delivery[0]),
                "plan_delivery_invalid",
            ),
            (
                delivery is not None
                and bytes(delivery[1])
                == bytes(record.canonical_plan_sha256),
                "plan_digest_changed",
            ),
            (
                tuple(self.free_run_live_exact_source_key_identity)
                == tuple(record.source_identity)
                or self._current_free_run_source_is_direct_semantic_successor(
                    record
                ),
                "source_identity_changed",
            ),
            (
                plan_identity is not None
                and int(plan_identity[0]) == int(record.race_arm_epoch)
                and int(plan_identity[1]) == int(record.planner_instance_id)
                and int(plan_identity[7]) == int(tracking_plan_generation),
                "plan_identity_changed",
            ),
            (
                sample_steering_rad is None
                or (
                    math.isfinite(sample_steering_rad)
                    and abs(sample_steering_rad)
                    <= float(record.actuator_hard_limit_rad)
                ),
                "old_pp_steering_hard_limit",
            ),
        )
        first_false = next(
            (reason for valid, reason in checks if not valid), "eligible"
        )
        self.free_run_live_exact_forward_transition_reason = first_false
        return first_false == "eligible"

    def on_recovery_cmd(self, msg: RecoveryControlCommand) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.recovery_cmd_time_sec,
            self.recovery_cmd_stamp_ns,
            self._stamp_ns(msg.header.stamp),
        )
        if not timestamp_advanced:
            return
        self.recovery_cmd = msg
        self.recovery_cmd_time_sec = receipt_time_sec
        self.recovery_cmd_stamp_ns = stamp_ns

    def on_recovery_status(self, msg: RecoveryStatus) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.recovery_status_time_sec,
            self.recovery_status_stamp_ns,
            self._stamp_ns(msg.header.stamp),
        )
        if not timestamp_advanced:
            return
        self.recovery_status = msg
        self.recovery_status_time_sec = receipt_time_sec
        self.recovery_status_stamp_ns = stamp_ns

    def on_recovery_permit(self, msg: RecoveryPermit) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.recovery_permit_time_sec,
            self.recovery_permit_stamp_ns,
            self._stamp_ns(msg.header.stamp),
        )
        if not timestamp_advanced:
            return
        self.recovery_permit = msg
        self.recovery_permit_time_sec = receipt_time_sec
        self.recovery_permit_stamp_ns = stamp_ns

    def on_external_safety_status(self, msg: SafetyStopStatus) -> None:
        (
            receipt_time_sec,
            stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.external_safety_time_sec,
            self.external_safety_stamp_ns,
            self._stamp_ns(msg.header.stamp),
        )
        if not bool(msg.valid) or bool(msg.stop_requested):
            self.external_safety_status = msg
            self.external_stop_latched = True
            self.motion_authority_warmup_proof = None
            self._invalidate_motion_authority_delivery_gap_lease(
                reason=DeliveryGapRevokeReason.HIGHER_PRIORITY_STOP,
                origin="external_safety_callback",
                now_sec=self.now_sec(),
            )
            self._invalidate_verified_stop_steering()
            self._invalidate_free_run_live_exact_record("external_safety_stop")
            if timestamp_advanced:
                self.external_safety_time_sec = receipt_time_sec
                self.external_safety_stamp_ns = stamp_ns
            return
        if not timestamp_advanced:
            return
        self.external_safety_status = msg
        self.external_safety_time_sec = receipt_time_sec
        self.external_safety_stamp_ns = stamp_ns
        self.external_stop_latched = False

    def on_race_armed(self, msg: Bool) -> None:
        was_armed = bool(self.finish_stop_latch.armed)
        previous_epoch = self.finish_stop_latch.epoch
        self.finish_stop_latch.observe_race_armed(bool(msg.data))
        if not bool(msg.data):
            self._invalidate_free_run_live_exact_record("race_disarmed")
            self._invalidate_motion_authority_delivery_gap_lease(
                reason=DeliveryGapRevokeReason.RACE_NOT_ARMED,
                origin="race_arm_callback",
                now_sec=self.now_sec(),
            )
            self._reset_state_lattice_source("race_disarmed")
        if self.race_arm_required:
            if bool(msg.data) and not was_armed:
                self.ros_clock_watchdog.reset()
                self.ros_clock_motion_ready = False
                self.ros_clock_observation_reason = "ros_clock_epoch_reset"
            elif not bool(msg.data):
                self.ros_clock_motion_ready = False
                self.ros_clock_observation_reason = "race_not_armed"
        if self.finish_stop_latch.epoch != previous_epoch:
            self._invalidate_free_run_live_exact_record("race_epoch_changed")
            self.finish_stop_lateral_guard_active = False
            self.last_tracking_steering_rad = None
            self._reset_finish_terminal_reference()
            self._invalidate_verified_stop_steering()
            self._reset_pure_pursuit_envelope_epoch()
            # A new race epoch is a fresh identity boundary; do not carry a
            # prior same-stamp safety payload taint into the new cohort.
            self.safety_constraint_timestamp_regressed = False
            self.overtake_plan_lateral_stop_taint_cache.clear()
            self.overtake_plan_motion_identity_cache.clear()
            self.motion_authority_grant_active = False
            self.motion_authority_warmup_proof = None
            self._invalidate_motion_authority_delivery_gap_lease()
            self.motion_authority_delivery_gap_stop_pending = False
            self.motion_authority_delivery_gap_revoke_pending = None
            self.motion_authority_delivery_gap_last_revoke = None
            self.motion_authority_delivery_gap_tombstone = None
            self._reset_state_lattice_source("race_epoch_changed")

    def _reset_state_lattice_source(self, reason: str) -> None:
        self.state_lattice_cmd = None
        self.state_lattice_cmd_time_sec = None
        self.state_lattice_cmd_stamp_ns = None
        self.state_lattice_cmd_valid = False
        self.state_lattice_cmd_reason = str(reason)
        self.state_lattice_active_producer_instance_id = None
        self.state_lattice_last_sequence = None
        self.state_lattice_last_plan_generation = None
        self.state_lattice_last_fingerprint = None
        self.core.state_lattice_episode_latched = False

    def _reset_finish_terminal_reference(self) -> None:
        """Reset terminal-reference state only on a new race arm epoch."""
        self.finish_terminal_candidate_steering_rad = None
        self.finish_terminal_candidate_time_sec = None
        self.finish_terminal_candidate_epoch = None
        self.finish_terminal_snapshot_steering_rad = None
        self.finish_terminal_snapshot_epoch = None

    def _remember_finish_terminal_reference(
        self,
        cmd: AckermannControlCommand,
        source: str,
        now_sec: float,
        *,
        active_control_fault_reason: str,
        ros_clock_stalled: bool,
        deadline_missed: bool,
        steering_result: SteeringLimitResult,
    ) -> None:
        """Keep only the final already-limited published motion steering."""
        if (
            self.finish_stop_latch.latched
            or not self.finish_stop_latch.armed
            or int(self.finish_stop_latch.epoch) <= 0
            or str(source).strip().lower()
            not in {"mpc", "pure_pursuit", "state_lattice", "recovery"}
            or self.external_stop_latched
            or self.control_fault_latched
            or bool(active_control_fault_reason)
            or ros_clock_stalled
            or deadline_missed
        ):
            return
        actuation_values = (
            float(cmd.longitudinal.speed),
            float(cmd.longitudinal.acceleration),
            float(cmd.longitudinal.jerk),
            float(cmd.lateral.steering_tire_angle),
            float(cmd.lateral.steering_tire_rotation_rate),
        )
        if not all(math.isfinite(value) for value in actuation_values):
            return
        self.finish_terminal_candidate_steering_rad = float(
            cmd.lateral.steering_tire_angle
        )
        self.finish_terminal_candidate_time_sec = float(now_sec)
        self.finish_terminal_candidate_epoch = int(self.finish_stop_latch.epoch)

    def _snapshot_finish_terminal_reference(self) -> None:
        """Freeze the current-epoch, fresh published steering at Finish."""
        self.finish_terminal_snapshot_steering_rad = None
        self.finish_terminal_snapshot_epoch = int(self.finish_stop_latch.epoch)
        candidate_steering_rad = self.finish_terminal_candidate_steering_rad
        candidate_time_sec = self.finish_terminal_candidate_time_sec
        candidate_epoch = self.finish_terminal_candidate_epoch
        age_sec = (
            self.now_sec() - candidate_time_sec
            if candidate_time_sec is not None
            else float("inf")
        )
        if not (
            candidate_epoch == int(self.finish_stop_latch.epoch)
            and candidate_steering_rad is not None
            and math.isfinite(candidate_steering_rad)
            and math.isfinite(age_sec)
            and 0.0 <= age_sec <= self.finish_terminal_reference_timeout_sec
        ):
            self.finish_stop_lateral_guard_active = False
            return
        self.finish_terminal_snapshot_steering_rad = float(candidate_steering_rad)
        self.finish_stop_lateral_guard_active = bool(
            abs(candidate_steering_rad)
            > self.finish_stop_steering_guard_trigger_rad
        )

    def _reset_pure_pursuit_envelope_epoch(self) -> None:
        """Permit a new shadow producer only after a real race epoch reset."""
        self.pure_pursuit_envelope_cache.clear()
        self.pure_pursuit_envelope_watermark = None
        self.pure_pursuit_envelope_active_producer_instance_id = None
        self.pure_pursuit_envelope_active_sequence = None
        self.pure_pursuit_envelope_active_stamp_ns = None
        self.pure_pursuit_envelope_candidate_instance_id = None
        self.pure_pursuit_envelope_candidate_sequence = None
        self.pure_pursuit_envelope_candidate_stamp_ns = None
        self.pure_pursuit_envelope_candidate_valid_count = 0
        self.pure_pursuit_envelope_quarantined_instances.clear()
        self.pure_pursuit_envelope_fault_latched = False
        self.pure_pursuit_envelope_fault_reason = ""
        self.pure_pursuit_envelope_last_valid = False
        self.pure_pursuit_envelope_last_reason = "race_epoch_reset"
        self.pure_pursuit_envelope_last_identity = None
        self.pure_pursuit_envelope_last_message = None
        self.pure_pursuit_envelope_last_receipt_time_sec = None
        self.pure_pursuit_envelope_last_parity_valid = False
        self.pure_pursuit_envelope_last_parity_reason = "race_epoch_reset"

    def _clear_pure_pursuit_envelope_candidate(self) -> None:
        self.pure_pursuit_envelope_candidate_instance_id = None
        self.pure_pursuit_envelope_candidate_sequence = None
        self.pure_pursuit_envelope_candidate_stamp_ns = None
        self.pure_pursuit_envelope_candidate_valid_count = 0

    def _set_pure_pursuit_envelope_watermark(
        self,
        identity: tuple[int, int, int, int],
        record: tuple[tuple, float, bool, str, ControllerCommandEnvelope],
    ) -> None:
        """Pin one deep-copied current-plan envelope outside the short cache."""
        # A Plan-absent successor preview is immutable evidence only.  Never
        # let a later Plan or mutable producer state turn that same record into
        # authority; PP must provide a separately validated post-Plan record.
        if str(record[3]) == "delivery_gap_successor_preview":
            return
        self.pure_pursuit_envelope_watermark = (
            tuple(identity),
            (
                tuple(record[0]),
                float(record[1]),
                bool(record[2]),
                str(record[3]),
                copy.deepcopy(record[4]),
            ),
        )

    def _promote_pure_pursuit_envelope_watermark(self) -> None:
        """Promote the latest cached record for the active current plan, if any."""
        active_producer = self.pure_pursuit_envelope_active_producer_instance_id
        current_generation = self.overtake_plan_generation
        if active_producer is None or current_generation is None:
            self.pure_pursuit_envelope_watermark = None
            return
        candidates = [
            (identity, record)
            for identity, record in self.pure_pursuit_envelope_cache.items()
            if int(identity[0]) == int(active_producer)
            and int(identity[3]) == int(current_generation)
            and int(identity[1]) > 0
            and not self._pure_pursuit_envelope_is_unpromoted_successor_preview(
                identity
            )
        ]
        if not candidates:
            self.pure_pursuit_envelope_watermark = None
            return
        identity, record = max(candidates, key=lambda item: int(item[0][1]))
        self._set_pure_pursuit_envelope_watermark(identity, record)

    def _pure_pursuit_envelope_is_unpromoted_successor_preview(
        self,
        identity: tuple[int, int, int, int],
    ) -> bool:
        """Keep Plan-absent N+1 evidence outside normal authority selection."""
        record = self.pure_pursuit_envelope_cache.get(tuple(identity))
        staged_preview = bool(
            record is not None
            and str(record[3]) == "delivery_gap_successor_preview"
        )
        # A staged record never becomes authority merely because mutable
        # producer/sequence state later happens to equal part of its identity.
        # PP must publish a post-Plan Envelope that passes the normal validator;
        # that distinct record has reason ``valid`` and is selectable.
        return staged_preview

    def _update_pure_pursuit_envelope_watermark(
        self,
        identity: tuple[int, int, int, int],
        record: tuple[tuple, float, bool, str, ControllerCommandEnvelope],
    ) -> None:
        """Advance the current-plan watermark only with a newer active sequence."""
        if str(record[3]) == "delivery_gap_successor_preview":
            return
        active_producer = self.pure_pursuit_envelope_active_producer_instance_id
        if (
            active_producer is None
            or self.overtake_plan_generation is None
            or int(identity[0]) != int(active_producer)
            or int(identity[3]) != int(self.overtake_plan_generation)
            or int(identity[1]) <= 0
        ):
            return
        watermark = self.pure_pursuit_envelope_watermark
        if watermark is not None and int(identity[1]) <= int(watermark[0][1]):
            return
        self._set_pure_pursuit_envelope_watermark(identity, record)

    def on_pure_pursuit_command_envelope(
        self, msg: ControllerCommandEnvelope
    ) -> None:
        if not self.latest_sample_observability_enabled:
            self._on_pure_pursuit_command_envelope_impl(msg)
            return
        if not self._mux_measurement_callback_entry():
            self._on_pure_pursuit_command_envelope_impl(msg)
            return
        raised = True
        try:
            self._on_pure_pursuit_command_envelope_impl(msg)
            raised = False
        finally:
            # Do not catch production exceptions: accounting is exactly-once,
            # while callback failure remains visible to the executor unchanged.
            self._mux_measurement_callback_finalize(raised)

    def _on_pure_pursuit_command_envelope_impl(
        self, msg: ControllerCommandEnvelope
    ) -> None:
        """Validate the atomic PP transport sample without changing authority."""
        identity = self._pure_pursuit_envelope_identity(msg)
        if self.latest_sample_observability_enabled:
            self._mux_measurement_callback_identity(identity)
        fingerprint = self._pure_pursuit_envelope_fingerprint(msg)
        prevalidated: Optional[tuple[bool, str]] = None
        stage_successor_preview = False
        delivery_gap_lease = self.motion_authority_delivery_gap_lease
        if delivery_gap_lease is not None:
            valid, reason = self._validate_pure_pursuit_envelope(msg)
            direct_successor = (
                1
                if int(delivery_gap_lease.plan_generation) >= 16_777_215
                else int(delivery_gap_lease.plan_generation) + 1
            )
            incoming_generation = int(msg.plan_generation)
            if incoming_generation not in (
                int(delivery_gap_lease.plan_generation),
                direct_successor,
            ):
                valid = False
                reason = "delivery_gap_non_direct_generation"
                self._invalidate_motion_authority_delivery_gap_lease(
                    stop_pending=True,
                    reason=DeliveryGapRevokeReason.SUCCESSOR_NOT_DIRECT,
                    origin="envelope_callback",
                    observed_generation=incoming_generation,
                    now_sec=self.now_sec(),
                )
            elif incoming_generation == direct_successor:
                expected_successor_stamp_ns = None
                expected_successor_plan_identity = None
                cached_successor_identity = (
                    self.overtake_plan_motion_identity_cache.get(
                        direct_successor
                    )
                )
                if (
                    cached_successor_identity is not None
                    and bool(cached_successor_identity[11])
                ):
                    expected_successor_stamp_ns = int(
                        cached_successor_identity[6]
                    )
                    expected_successor_plan_identity = (
                        *tuple(cached_successor_identity[:8]),
                        int(cached_successor_identity[9]),
                        bytes(cached_successor_identity[10]),
                    )
                elif (
                    self.safety_constraint is not None
                    and int(self.safety_constraint.plan_generation)
                    == direct_successor
                ):
                    expected_successor_stamp_ns = int(
                        self.safety_constraint.header_stamp_ns
                    )
                callback_revoke_reason = (
                    self._delivery_gap_successor_envelope_revoke_reason(
                        delivery_gap_lease,
                        identity=identity,
                        record=(
                            fingerprint,
                            self.now_sec(),
                            valid,
                            reason,
                            msg,
                        ),
                        expected_generation=direct_successor,
                        expected_successor_stamp_ns=expected_successor_stamp_ns,
                        expected_plan_identity=expected_successor_plan_identity,
                        now_sec=self.now_sec(),
                    )
                )
                if callback_revoke_reason is not None:
                    valid = False
                    reason = (
                        "delivery_gap_successor_"
                        + callback_revoke_reason.name.lower()
                    )
                    self._invalidate_motion_authority_delivery_gap_lease(
                        stop_pending=True,
                        reason=callback_revoke_reason,
                        origin="envelope_callback",
                        observed_generation=incoming_generation,
                        now_sec=self.now_sec(),
                    )
                elif cached_successor_identity is None:
                    # A direct N+1 Envelope may arrive before Plan N+1.  The
                    # delivery-gap validator above has already checked its
                    # identity, stamp, freshness and usability.  Preserve it
                    # as a non-authoritative preview so the later exact Plan
                    # can complete the cohort without requiring retransmit.
                    valid = True
                    reason = "delivery_gap_successor_preview"
                    stage_successor_preview = True
                else:
                    # Plan N+1 already exists and the relation validator above
                    # accepted this Envelope.  Admit it through the ordinary
                    # valid path; never label a post-Plan record as preview.
                    valid = True
                    reason = "valid"
            elif not valid or not bool(msg.trajectory_tracking_usable):
                self._invalidate_motion_authority_delivery_gap_lease(
                    stop_pending=True,
                    reason=DeliveryGapRevokeReason.N1_ENVELOPE_UNUSABLE_OR_STALE,
                    origin="envelope_callback",
                    observed_generation=incoming_generation,
                    now_sec=self.now_sec(),
                )
            prevalidated = (valid, reason)
        cached = self.pure_pursuit_envelope_cache.get(identity)
        if cached is not None:
            if cached[0] != fingerprint:
                self._latch_pure_pursuit_envelope_fault("duplicate_conflict")
            self.pure_pursuit_envelope_last_identity = identity
            self.pure_pursuit_envelope_last_message = msg
            # A duplicate is not a fresh receipt. Restore the immutable first
            # receipt time for this exact identity rather than retaining the
            # later sample's age in shadow diagnostics.
            self.pure_pursuit_envelope_last_receipt_time_sec = cached[1]
            if prevalidated is not None and not prevalidated[0]:
                # Preserve the current callback's active-lease rejection in
                # diagnostics even when the transport identity was cached
                # before the lease existed. The cached receipt remains
                # immutable, while motion already follows pending STOP.
                self.pure_pursuit_envelope_last_valid = False
                self.pure_pursuit_envelope_last_reason = prevalidated[1]
            else:
                self.pure_pursuit_envelope_last_valid = bool(cached[2])
                self.pure_pursuit_envelope_last_reason = (
                    "duplicate"
                    if cached[0] == fingerprint
                    else "duplicate_conflict"
                )
            self._refresh_pure_pursuit_envelope_parity()
            return

        if prevalidated is None:
            valid, reason = self._validate_pure_pursuit_envelope(msg)
        else:
            valid, reason = prevalidated
        receipt_time_sec = self.now_sec()
        self._remember_bounded(
            self.pure_pursuit_envelope_cache,
            identity,
            (fingerprint, receipt_time_sec, valid, reason, copy.deepcopy(msg)),
        )
        record = self.pure_pursuit_envelope_cache[identity]
        self.pure_pursuit_envelope_last_identity = identity
        self.pure_pursuit_envelope_last_message = msg
        self.pure_pursuit_envelope_last_receipt_time_sec = receipt_time_sec
        self.pure_pursuit_envelope_last_valid = valid
        self.pure_pursuit_envelope_last_reason = reason
        self._refresh_pure_pursuit_envelope_parity()
        if not valid:
            self._update_pure_pursuit_envelope_watermark(identity, record)
            # Before bind, an invalid sample breaks the required consecutive
            # valid sequence. An already active producer is never released by
            # malformed traffic; only a new race epoch may rebind it.
            if self.pure_pursuit_envelope_active_producer_instance_id is None:
                self._clear_pure_pursuit_envelope_candidate()
            if reason == "sequence_zero":
                self._latch_pure_pursuit_envelope_fault("sequence_zero")
            return
        if stage_successor_preview:
            # Plan-absent N+1 is evidence only.  Do not advance the active
            # producer sequence/watermark, because committed N must remain
            # exactly selectable until the successor cohort completes.
            current_lease = self.motion_authority_delivery_gap_lease
            if current_lease is delivery_gap_lease:
                self.motion_authority_delivery_gap_lease = replace(
                    current_lease,
                    gap_started_steady_time_sec=(
                        current_lease.gap_started_steady_time_sec
                        if current_lease.gap_started_steady_time_sec is not None
                        else float(receipt_time_sec)
                    ),
                    successor_envelope_preview_identity=tuple(identity),
                )
            return
        if self.pure_pursuit_envelope_fault_latched:
            return

        producer_instance_id = int(msg.producer_instance_id)
        command_sequence = int(msg.command_sequence)
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        if producer_instance_id in self.pure_pursuit_envelope_quarantined_instances:
            self._latch_pure_pursuit_envelope_fault("producer_quarantined")
            return
        if self.pure_pursuit_envelope_active_producer_instance_id is not None:
            if producer_instance_id != self.pure_pursuit_envelope_active_producer_instance_id:
                self.pure_pursuit_envelope_quarantined_instances.add(
                    producer_instance_id
                )
                self._latch_pure_pursuit_envelope_fault("active_producer_conflict")
                return
            if (
                self.pure_pursuit_envelope_active_sequence is not None
                and command_sequence
                <= self.pure_pursuit_envelope_active_sequence
            ):
                self._latch_pure_pursuit_envelope_fault("sequence_regression")
                return
            if (
                self.pure_pursuit_envelope_active_stamp_ns is not None
                and header_stamp_ns < self.pure_pursuit_envelope_active_stamp_ns
            ):
                self._latch_pure_pursuit_envelope_fault("stamp_regression")
                return
            self.pure_pursuit_envelope_active_sequence = command_sequence
            self.pure_pursuit_envelope_active_stamp_ns = header_stamp_ns
            self._update_pure_pursuit_envelope_watermark(identity, record)
            return

        if self.pure_pursuit_envelope_candidate_instance_id is None:
            self.pure_pursuit_envelope_candidate_instance_id = producer_instance_id
            self.pure_pursuit_envelope_candidate_sequence = command_sequence
            self.pure_pursuit_envelope_candidate_stamp_ns = header_stamp_ns
            self.pure_pursuit_envelope_candidate_valid_count = 1
            return
        if producer_instance_id != self.pure_pursuit_envelope_candidate_instance_id:
            self.pure_pursuit_envelope_quarantined_instances.add(producer_instance_id)
            self._latch_pure_pursuit_envelope_fault("candidate_producer_conflict")
            return
        if (
            self.pure_pursuit_envelope_candidate_sequence is None
            or command_sequence <= self.pure_pursuit_envelope_candidate_sequence
        ):
            self._latch_pure_pursuit_envelope_fault("sequence_regression")
            return
        if (
            self.pure_pursuit_envelope_candidate_stamp_ns is not None
            and header_stamp_ns < self.pure_pursuit_envelope_candidate_stamp_ns
        ):
            self._latch_pure_pursuit_envelope_fault("stamp_regression")
            return
        self.pure_pursuit_envelope_candidate_sequence = command_sequence
        self.pure_pursuit_envelope_candidate_stamp_ns = header_stamp_ns
        self.pure_pursuit_envelope_candidate_valid_count += 1
        if self.pure_pursuit_envelope_candidate_valid_count >= 2:
            self.pure_pursuit_envelope_active_producer_instance_id = (
                producer_instance_id
            )
            self.pure_pursuit_envelope_active_sequence = command_sequence
            self.pure_pursuit_envelope_active_stamp_ns = header_stamp_ns
            self._promote_pure_pursuit_envelope_watermark()

    def _latch_pure_pursuit_envelope_fault(self, reason: str) -> None:
        self.pure_pursuit_envelope_fault_latched = True
        self.pure_pursuit_envelope_fault_reason = reason
        self.pure_pursuit_envelope_last_valid = False
        self.pure_pursuit_envelope_last_reason = reason
        self.motion_authority_warmup_proof = None

    def _refresh_pure_pursuit_envelope_parity(self) -> None:
        if self.pure_pursuit_envelope_last_message is None:
            self.pure_pursuit_envelope_last_parity_valid = False
            self.pure_pursuit_envelope_last_parity_reason = "envelope_missing"
            return
        (
            self.pure_pursuit_envelope_last_parity_valid,
            self.pure_pursuit_envelope_last_parity_reason,
        ) = self._pure_pursuit_envelope_legacy_parity(
            self.pure_pursuit_envelope_last_message
        )

    def _pure_pursuit_envelope_receipt_age_sec(self, now_sec: float) -> float:
        if self.pure_pursuit_envelope_last_receipt_time_sec is None:
            return math.inf
        return now_sec - self.pure_pursuit_envelope_last_receipt_time_sec

    def _pure_pursuit_envelope_receipt_fresh(self, now_sec: float) -> bool:
        return self._fresh(
            self.pure_pursuit_envelope_last_receipt_time_sec,
            now_sec,
            self.pure_pursuit_envelope_timeout_sec,
        )

    def _pure_pursuit_envelope_shadow_usable(
        self, now_sec: float, ros_clock_stalled: bool
    ) -> bool:
        identity = self.pure_pursuit_envelope_last_identity
        producer_bound_to_last_sample = bool(
            identity is not None
            and self.pure_pursuit_envelope_active_producer_instance_id
            == identity[0]
            and self.pure_pursuit_envelope_active_sequence == identity[1]
        )
        return bool(
            self.pure_pursuit_envelope_last_valid
            and not self.pure_pursuit_envelope_fault_latched
            and producer_bound_to_last_sample
            and self.pure_pursuit_envelope_last_parity_valid
            and self._pure_pursuit_envelope_receipt_fresh(now_sec)
            and not ros_clock_stalled
        )

    @staticmethod
    def _pure_pursuit_envelope_identity(
        msg: ControllerCommandEnvelope,
    ) -> tuple[int, int, int, int]:
        return (
            int(msg.producer_instance_id),
            int(msg.command_sequence),
            HybridControlMuxNode._stamp_ns(msg.header.stamp),
            int(msg.plan_generation),
        )

    @staticmethod
    def _pure_pursuit_envelope_plan_identity(
        msg: ControllerCommandEnvelope,
    ) -> tuple:
        key = msg.plan_sample_key
        return (
            int(key.race_arm_epoch),
            int(key.planner_instance_id),
            int(key.attempt_id),
            str(key.target_vehicle_id),
            int(key.pass_direction),
            int(key.connector_transaction_id),
            HybridControlMuxNode._stamp_ns(key.plan_stamp),
            int(key.plan_generation),
            int(msg.candidate_revision),
            bytes(msg.candidate_content_sha256),
        )

    def _motion_authority_successor_envelope_relation_valid(
        self,
        lease: MotionAuthorityDeliveryGapLease,
        msg: ControllerCommandEnvelope,
        *,
        expected_successor_stamp_ns: Optional[int] = None,
    ) -> bool:
        """Bind N+1 to stable transaction facts and a known N+1 Plan."""
        if len(lease.plan_identity) < 12:
            return False
        direct_successor = (
            1 if int(lease.plan_generation) >= 16_777_215
            else int(lease.plan_generation) + 1
        )
        envelope_identity = self._pure_pursuit_envelope_plan_identity(msg)
        envelope_stamp_ns = int(envelope_identity[6])
        successor_identity = lease.successor_plan_identity
        expected_successor_identity = (
            (
                *tuple(successor_identity[:8]),
                int(successor_identity[9]),
                bytes(successor_identity[10]),
            )
            if successor_identity is not None
            else None
        )
        return bool(
            int(msg.schema_version) == 2
            and bool(lease.plan_identity[11])
            and int(msg.plan_generation) == direct_successor
            and tuple(envelope_identity[:6]) == tuple(lease.plan_identity[:6])
            and envelope_stamp_ns > int(lease.plan_identity[6])
            and (
                expected_successor_stamp_ns is None
                or envelope_stamp_ns == int(expected_successor_stamp_ns)
            )
            and int(envelope_identity[7]) == direct_successor
            and int(envelope_identity[8]) == int(lease.plan_identity[9]) + 1
            and len(bytes(envelope_identity[9])) == 32
            and any(bytes(envelope_identity[9]))
            and (
                expected_successor_identity is None
                or tuple(envelope_identity)
                == tuple(expected_successor_identity)
            )
        )

    def _delivery_gap_successor_envelope_revoke_reason(
        self,
        lease: MotionAuthorityDeliveryGapLease,
        *,
        identity: tuple,
        record: tuple,
        expected_generation: int,
        expected_successor_stamp_ns: Optional[int],
        expected_plan_identity: Optional[tuple],
        now_sec: float,
    ) -> Optional[DeliveryGapRevokeReason]:
        """Classify the existing fail-closed successor-envelope predicate."""
        envelope = record[4]
        if tuple(self._pure_pursuit_envelope_identity(envelope)) != tuple(identity):
            return DeliveryGapRevokeReason.N1_ENVELOPE_IDENTITY_MISMATCH
        if int(envelope.schema_version) != 2:
            return DeliveryGapRevokeReason.N1_ENVELOPE_SCHEMA_INVALID
        if int(envelope.plan_generation) != int(expected_generation):
            return DeliveryGapRevokeReason.N1_ENVELOPE_GENERATION_MISMATCH
        if not self._motion_authority_successor_envelope_relation_valid(
            lease,
            envelope,
            expected_successor_stamp_ns=expected_successor_stamp_ns,
        ):
            return DeliveryGapRevokeReason.N1_ENVELOPE_RELATION_INVALID
        if (
            expected_plan_identity is not None
            and self._pure_pursuit_envelope_plan_identity(envelope)
            != expected_plan_identity
        ):
            return DeliveryGapRevokeReason.N1_ENVELOPE_PLAN_BINDING_MISMATCH
        if not bool(record[2]) or not self._fresh(
            record[1], now_sec, self.pure_pursuit_envelope_timeout_sec
        ):
            return DeliveryGapRevokeReason.N1_ENVELOPE_UNUSABLE_OR_STALE
        return None

    def _motion_authority_successor_plan_relation_valid(
        self,
        lease: MotionAuthorityDeliveryGapLease,
        msg: OvertakePlan,
        *,
        plan_valid: bool,
    ) -> bool:
        """Validate the direct N+1 Plan without changing shared authority."""
        if len(lease.plan_identity) < 12:
            return False
        direct_successor = (
            1 if int(lease.plan_generation) >= 16_777_215
            else int(lease.plan_generation) + 1
        )
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        incoming_identity = (
            int(msg.race_arm_epoch),
            int(msg.planner_instance_id),
            int(msg.attempt_id),
            str(msg.target_vehicle_id),
            int(msg.pass_direction),
            int(msg.connector_transaction_id),
        )
        successor_identity = lease.successor_plan_identity
        incoming_plan_identity = (
            *incoming_identity,
            int(header_stamp_ns),
            int(msg.plan_generation),
            int(msg.phase),
            int(msg.candidate_revision),
            bytes(msg.candidate_content_sha256),
            bool(msg.aw2_identity_schema_version == 1),
        )
        incoming_fingerprint = self._overtake_plan_lateral_stop_fingerprint(
            msg
        )
        expected_constraint_stamp_ns = (
            int(self.safety_constraint.header_stamp_ns)
            if (
                self.safety_constraint is not None
                and int(self.safety_constraint.plan_generation)
                == direct_successor
            )
            else None
        )
        return bool(
            plan_valid
            and bool(lease.plan_identity[11])
            and int(msg.plan_generation) == direct_successor
            and incoming_identity == tuple(lease.plan_identity[:6])
            and header_stamp_ns > int(lease.plan_identity[6])
            and (
                expected_constraint_stamp_ns is None
                or header_stamp_ns == expected_constraint_stamp_ns
            )
            and int(msg.phase) == int(OvertakePlan.PASSING)
            and int(msg.candidate_revision) == int(lease.plan_identity[9]) + 1
            and len(bytes(msg.candidate_content_sha256)) == 32
            and any(bytes(msg.candidate_content_sha256))
            and bool(msg.aw2_identity_schema_version == 1)
            and (
                successor_identity is None
                or tuple(incoming_plan_identity) == tuple(successor_identity)
            )
            and (
                successor_identity is None
                or (
                    lease.successor_plan_lateral_stop_fingerprint is not None
                    and incoming_fingerprint
                    == lease.successor_plan_lateral_stop_fingerprint
                )
            )
        )

    def _delivery_gap_successor_plan_revoke_reason(
        self,
        lease: MotionAuthorityDeliveryGapLease,
        msg: OvertakePlan,
        *,
        plan_valid: bool,
    ) -> Optional[DeliveryGapRevokeReason]:
        """Classify the existing direct-successor Plan predicate."""
        if not plan_valid:
            return DeliveryGapRevokeReason.N1_PLAN_INVALID
        if len(lease.plan_identity) < 12 or not bool(lease.plan_identity[11]):
            return DeliveryGapRevokeReason.N1_IDENTITY_INVALID
        direct_successor = (
            1 if int(lease.plan_generation) >= 16_777_215
            else int(lease.plan_generation) + 1
        )
        if int(msg.plan_generation) != direct_successor:
            return DeliveryGapRevokeReason.SUCCESSOR_NOT_DIRECT
        if int(msg.phase) != int(OvertakePlan.PASSING):
            return DeliveryGapRevokeReason.N1_PLAN_PHASE_INVALID
        if not bool(msg.trajectory_authorized) or not bool(
            msg.lateral_maneuver_required
        ):
            return DeliveryGapRevokeReason.N1_PLAN_UNAUTHORIZED
        if self._stamp_ns(msg.header.stamp) <= int(lease.plan_identity[6]):
            return DeliveryGapRevokeReason.N1_PLAN_STAMP_NOT_FORWARD
        if not self._motion_authority_successor_plan_relation_valid(
            lease, msg, plan_valid=plan_valid
        ):
            return DeliveryGapRevokeReason.N1_PLAN_RELATION_INVALID
        return None

    def _delivery_gap_successor_plan_relation_subreason(
        self,
        lease: MotionAuthorityDeliveryGapLease,
        msg: OvertakePlan,
    ) -> Optional[DeliveryGapPlanRelationSubreason]:
        """Identify the failed DG408 subpredicate without granting authority."""
        incoming_identity = (
            int(msg.race_arm_epoch),
            int(msg.planner_instance_id),
            int(msg.attempt_id),
            str(msg.target_vehicle_id),
            int(msg.pass_direction),
            int(msg.connector_transaction_id),
        )
        if incoming_identity != tuple(lease.plan_identity[:6]):
            return DeliveryGapPlanRelationSubreason.IDENTITY_PREFIX_MISMATCH
        direct_successor = (
            1
            if int(lease.plan_generation) >= 16_777_215
            else int(lease.plan_generation) + 1
        )
        expected_constraint_stamp_ns = (
            int(self.safety_constraint.header_stamp_ns)
            if (
                self.safety_constraint is not None
                and int(self.safety_constraint.plan_generation) == direct_successor
            )
            else None
        )
        if (
            expected_constraint_stamp_ns is not None
            and self._stamp_ns(msg.header.stamp) != expected_constraint_stamp_ns
        ):
            return (
                DeliveryGapPlanRelationSubreason.EXPECTED_CONSTRAINT_STAMP_MISMATCH
            )
        if int(msg.candidate_revision) != int(lease.plan_identity[9]) + 1:
            return DeliveryGapPlanRelationSubreason.CANDIDATE_REVISION_MISMATCH
        successor_identity = lease.successor_plan_identity
        if (
            successor_identity is not None
            and bytes(msg.candidate_content_sha256)
            != bytes(successor_identity[10])
        ):
            return (
                DeliveryGapPlanRelationSubreason.CANDIDATE_CONTENT_DIGEST_MISMATCH
            )
        if int(msg.aw2_identity_schema_version) != 1:
            return DeliveryGapPlanRelationSubreason.AW2_IDENTITY_SCHEMA_INVALID
        if (
            successor_identity is not None
            and lease.successor_plan_lateral_stop_fingerprint is None
        ):
            return (
                DeliveryGapPlanRelationSubreason.LATERAL_STOP_FINGERPRINT_MISSING
            )
        if (
            successor_identity is not None
            and self._overtake_plan_lateral_stop_fingerprint(msg)
            != lease.successor_plan_lateral_stop_fingerprint
        ):
            return (
                DeliveryGapPlanRelationSubreason.LATERAL_STOP_FINGERPRINT_MISMATCH
            )
        return None

    @staticmethod
    def _pure_pursuit_envelope_fingerprint(msg: ControllerCommandEnvelope) -> tuple:
        command = msg.command
        return (
            int(msg.schema_version),
            str(msg.header.frame_id),
            HybridControlMuxNode._stamp_ns(command.stamp),
            HybridControlMuxNode._stamp_ns(command.longitudinal.stamp),
            HybridControlMuxNode._stamp_ns(command.lateral.stamp),
            float(command.longitudinal.speed),
            float(command.longitudinal.acceleration),
            float(command.longitudinal.jerk),
            float(command.lateral.steering_tire_angle),
            float(command.lateral.steering_tire_rotation_rate),
            bool(msg.mpc_horizon_usable),
            bool(msg.pp_command_fresh),
            bool(msg.trajectory_tracking_usable),
            int(msg.lateral_stop_authority_kind),
            int(msg.lateral_stop_transaction_pass_direction),
            int(msg.lateral_stop_authority_token),
            float(msg.command_age_sec),
            str(msg.reason),
            int(msg.plan_sample_key.race_arm_epoch),
            int(msg.plan_sample_key.planner_instance_id),
            int(msg.plan_sample_key.attempt_id),
            str(msg.plan_sample_key.target_vehicle_id),
            int(msg.plan_sample_key.pass_direction),
            int(msg.plan_sample_key.connector_transaction_id),
            HybridControlMuxNode._stamp_ns(
                msg.plan_sample_key.plan_stamp
            ),
            int(msg.plan_sample_key.plan_generation),
            int(msg.candidate_revision),
            bytes(msg.candidate_content_sha256),
        )

    def _validate_pure_pursuit_envelope(
        self,
        msg: ControllerCommandEnvelope,
        *,
        allow_tracking_unusable: bool = False,
    ) -> tuple[bool, str]:
        command = msg.command
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        command_stamps = (
            self._stamp_ns(command.stamp),
            self._stamp_ns(command.longitudinal.stamp),
            self._stamp_ns(command.lateral.stamp),
        )
        values = (
            float(command.longitudinal.speed),
            float(command.longitudinal.acceleration),
            float(command.longitudinal.jerk),
            float(command.lateral.steering_tire_angle),
            float(command.lateral.steering_tire_rotation_rate),
        )
        command_age_sec = float(msg.command_age_sec)
        lateral_stop_authority_kind = int(msg.lateral_stop_authority_kind)
        lateral_stop_transaction_pass_direction = int(
            msg.lateral_stop_transaction_pass_direction
        )
        lateral_stop_authority_token = int(msg.lateral_stop_authority_token)
        if int(msg.schema_version) not in (1, 2):
            return False, "schema_version"
        if int(msg.producer_instance_id) == 0:
            return False, "producer_instance_zero"
        if int(msg.command_sequence) == 0:
            return False, "sequence_zero"
        if int(msg.plan_generation) == 0:
            return False, "plan_generation_zero"
        if lateral_stop_authority_kind not in (
            int(OvertakePlan.LATERAL_STOP_NONE),
            int(OvertakePlan.LATERAL_STOP_CURRENT_D_HOLD),
            int(OvertakePlan.LATERAL_STOP_PASS_WARMUP),
        ):
            return False, "lateral_stop_authority_kind"
        if lateral_stop_authority_kind == int(OvertakePlan.LATERAL_STOP_NONE):
            if (
                lateral_stop_transaction_pass_direction != 0
                or lateral_stop_authority_token != 0
            ):
                return False, "lateral_stop_none_payload"
        elif (
            lateral_stop_transaction_pass_direction not in (-1, 1)
            or lateral_stop_authority_token <= 0
        ):
            return False, "lateral_stop_authority_payload"
        if str(msg.header.frame_id) != "base_link":
            return False, "frame_id"
        if any(stamp_ns != header_stamp_ns for stamp_ns in command_stamps):
            return False, "stamp_mismatch"
        now_ros_ns = int(self.get_clock().now().nanoseconds)
        future_tolerance_ns = int(
            self.pure_pursuit_envelope_future_stamp_tolerance_sec * 1.0e9
        )
        if now_ros_ns == 0 and header_stamp_ns != 0:
            return False, "future_stamp_clock_zero"
        if now_ros_ns > 0 and header_stamp_ns > now_ros_ns + future_tolerance_ns:
            return False, "future_stamp"
        if now_ros_ns > 0:
            header_age_sec = (now_ros_ns - header_stamp_ns) * 1.0e-9
            if header_age_sec > self.pure_pursuit_envelope_timeout_sec:
                return False, "header_stamp_stale"
        if not all(math.isfinite(value) for value in values):
            return False, "command_nonfinite"
        if float(command.longitudinal.jerk) != 0.0:
            return False, "v1_jerk_nonzero"
        if float(command.lateral.steering_tire_rotation_rate) != 0.0:
            return False, "v1_steering_rate_nonzero"
        if (
            not math.isfinite(command_age_sec)
            or command_age_sec < 0.0
            or command_age_sec > self.pure_pursuit_envelope_timeout_sec
        ):
            return False, "command_age"
        if not bool(msg.pp_command_fresh):
            return False, "pp_command_not_fresh"
        if (
            not bool(msg.trajectory_tracking_usable)
            and not allow_tracking_unusable
        ):
            return False, "trajectory_tracking_unusable"
        if int(msg.schema_version) == 2:
            key = msg.plan_sample_key
            if (
                int(key.race_arm_epoch) <= 0
                or int(key.planner_instance_id) <= 0
                or int(key.attempt_id) <= 0
                or not str(key.target_vehicle_id).strip()
                or int(key.pass_direction) not in (-1, 1)
                or int(key.connector_transaction_id) <= 0
                or int(key.plan_generation) != int(msg.plan_generation)
                or self._stamp_ns(key.plan_stamp) <= 0
                or int(msg.candidate_revision) <= 0
                or len(bytes(msg.candidate_content_sha256)) != 32
                or not any(bytes(msg.candidate_content_sha256))
            ):
                return False, "planner_candidate_identity"
        return True, "valid"

    def _pure_pursuit_envelope_legacy_parity(
        self, msg: ControllerCommandEnvelope
    ) -> tuple[bool, str]:
        command = self.pure_pursuit_cmd
        status = self.pure_pursuit_tracking_status
        if command is None or status is None:
            return False, "legacy_missing"
        envelope_command = msg.command
        envelope_stamps = (
            self._stamp_ns(envelope_command.stamp),
            self._stamp_ns(envelope_command.longitudinal.stamp),
            self._stamp_ns(envelope_command.lateral.stamp),
        )
        legacy_stamps = (
            self._stamp_ns(command.stamp),
            self._stamp_ns(command.longitudinal.stamp),
            self._stamp_ns(command.lateral.stamp),
        )
        if envelope_stamps != legacy_stamps:
            return False, "legacy_command_stamp"
        envelope_values = (
            float(envelope_command.longitudinal.speed),
            float(envelope_command.longitudinal.acceleration),
            float(envelope_command.longitudinal.jerk),
            float(envelope_command.lateral.steering_tire_angle),
            float(envelope_command.lateral.steering_tire_rotation_rate),
        )
        legacy_values = (
            float(command.longitudinal.speed),
            float(command.longitudinal.acceleration),
            float(command.longitudinal.jerk),
            float(command.lateral.steering_tire_angle),
            float(command.lateral.steering_tire_rotation_rate),
        )
        if envelope_values != legacy_values:
            return False, "legacy_command_payload"
        if (
            self._stamp_ns(msg.header.stamp) != self._stamp_ns(status.header.stamp)
            or str(status.header.frame_id) != "base_link"
            or int(msg.plan_generation) != int(status.plan_generation)
            or bool(msg.mpc_horizon_usable) != bool(status.mpc_horizon_usable)
            or bool(msg.pp_command_fresh) != bool(status.pp_command_fresh)
            or bool(msg.trajectory_tracking_usable)
            != bool(status.trajectory_tracking_usable)
            or int(msg.lateral_stop_authority_kind)
            != int(status.lateral_stop_authority_kind)
            or int(msg.lateral_stop_transaction_pass_direction)
            != int(status.lateral_stop_transaction_pass_direction)
            or int(msg.lateral_stop_authority_token)
            != int(status.lateral_stop_authority_token)
            or float(msg.command_age_sec) != float(status.command_age_sec)
            or str(msg.reason) != str(status.reason)
        ):
            return False, "legacy_status_payload"
        return True, "valid"

    @staticmethod
    def _build_selected_pp_motion_sample(
        command: AckermannControlCommand,
        command_receipt_time_sec: float,
        authority_proof: Optional[ControllerTrackingStatus] = None,
        authority_proof_receipt_time_sec: Optional[float] = None,
        authority_proof_valid: bool = False,
        observed_proof: Optional[ControllerTrackingStatus] = None,
        observed_proof_receipt_time_sec: Optional[float] = None,
        observed_proof_valid: bool = False,
        envelope_identity: Optional[tuple[int, int, int, int]] = None,
    ) -> PurePursuitMotionSample:
        """Snapshot PP source inputs without elevating observed proof authority."""
        command_snapshot = copy.deepcopy(command)
        authority_proof_snapshot = (
            copy.deepcopy(authority_proof)
            if authority_proof is not None
            else None
        )
        command_stamp_ns = HybridControlMuxNode._stamp_ns(command_snapshot.stamp)
        return PurePursuitMotionSample(
            command=command_snapshot,
            command_stamp_ns=command_stamp_ns,
            command_receipt_time_sec=float(command_receipt_time_sec),
            authority_proof=authority_proof_snapshot,
            authority_proof_receipt_time_sec=(
                float(authority_proof_receipt_time_sec)
                if authority_proof_receipt_time_sec is not None
                else None
            ),
            authority_proof_valid=bool(authority_proof_valid),
            authority_proof_identity=(
                (
                    HybridControlMuxNode._stamp_ns(
                        authority_proof_snapshot.header.stamp
                    ),
                    int(authority_proof_snapshot.plan_generation),
                )
                if authority_proof_snapshot is not None
                else None
            ),
            observed_proof=(
                copy.deepcopy(observed_proof)
                if observed_proof is not None
                else None
            ),
            observed_proof_receipt_time_sec=(
                float(observed_proof_receipt_time_sec)
                if observed_proof_receipt_time_sec is not None
                else None
            ),
            observed_proof_valid=bool(observed_proof_valid),
            envelope_identity=(
                tuple(envelope_identity)
                if envelope_identity is not None
                else None
            ),
        )

    def _select_legacy_pp_motion_sample(
        self,
        *,
        tracking_plan_generation: int,
    ) -> Optional[PurePursuitMotionSample]:
        """Atomically snapshot legacy PP command and its selected proof.

        This is the only legacy-source reader for the motion authority path.
        Exact N or bounded N-1 proof is motion authority. Latest legacy status
        is retained solely as observed diagnostic/public-status evidence.
        """
        if (
            self.pure_pursuit_cmd is None
            or self.pure_pursuit_cmd_time_sec is None
        ):
            return None
        legacy_command = self.pure_pursuit_cmd
        command_stamp_ns = self._stamp_ns(legacy_command.stamp)
        active_producer = self.pure_pursuit_envelope_active_producer_instance_id
        legacy_previous_generation = (
            16_777_215
            if int(tracking_plan_generation) == 1
            else int(tracking_plan_generation) - 1
        )
        # Selector history must not turn arbitrary older samples into motion.
        # The sole direct-predecessor exception is handled by the immutable
        # committed-cohort lease at the timer boundary below.
        previous_generation = legacy_previous_generation
        current_entry = self.pure_pursuit_tracking_status_cache.get(
            (command_stamp_ns, tracking_plan_generation)
        )
        previous_entry = (
            self.pure_pursuit_tracking_status_cache.get(
                (command_stamp_ns, previous_generation)
            )
            if tracking_plan_generation > 0
            else None
        )
        authority_entry = (
            current_entry
            if current_entry is not None
            else previous_entry
        )
        authority_proof = (
            authority_entry[0] if authority_entry is not None else None
        )
        authority_proof_receipt_time_sec = (
            authority_entry[1] if authority_entry is not None else None
        )
        authority_proof_valid = bool(
            authority_entry is not None and authority_entry[2]
        )
        return self._build_selected_pp_motion_sample(
            legacy_command,
            self.pure_pursuit_cmd_time_sec,
            authority_proof=authority_proof,
            authority_proof_receipt_time_sec=authority_proof_receipt_time_sec,
            authority_proof_valid=authority_proof_valid,
            observed_proof=self.pure_pursuit_tracking_status,
            observed_proof_receipt_time_sec=(
                self.pure_pursuit_tracking_status_time_sec
            ),
            observed_proof_valid=self.pure_pursuit_tracking_status_valid,
        )

    def _select_bound_envelope_pp_motion_sample(
        self,
        *,
        tracking_plan_generation: int,
        now_sec: float,
        ros_clock_stalled: bool,
        debug_rejection_sink: Optional[list[dict[str, object]]] = None,
    ) -> Optional[PurePursuitMotionSample]:
        """Build authority only from a fresh BOUND immutable envelope record."""
        active_producer = self.pure_pursuit_envelope_active_producer_instance_id
        if (
            active_producer is None
            or self.pure_pursuit_envelope_fault_latched
            or ros_clock_stalled
            or tracking_plan_generation < 0
        ):
            return None
        legacy_previous_generation = (
            16_777_215
            if int(tracking_plan_generation) == 1
            else int(tracking_plan_generation) - 1
        )
        # Do not widen an N-1 bridge by scanning cache history.  A positive
        # PASSING command can cross this boundary only via a committed lease.
        previous_generation = legacy_previous_generation
        watermark = self.pure_pursuit_envelope_watermark
        if watermark is not None:
            watermark_identity, watermark_record = watermark
            if str(watermark_record[3]) == "delivery_gap_successor_preview":
                # Defense in depth for stale/injected watermark state.  A
                # preview reason is permanently non-authoritative.
                watermark = None
            if (
                watermark is not None
                and
                int(watermark_identity[0]) == int(active_producer)
                and int(watermark_identity[3]) == int(tracking_plan_generation)
            ):
                # A received unusable envelope is a hard negative.  It is not
                # a delivery gap and cannot reopen older cache history.
                watermark_delivery_gap = False
                if (
                    not bool(watermark_record[2])
                    or not self._fresh(
                        watermark_record[1],
                        now_sec,
                        self.pure_pursuit_envelope_timeout_sec,
                    )
                ) and not watermark_delivery_gap:
                    if debug_rejection_sink is not None:
                        envelope = watermark_record[4]
                        fresh = self._fresh(
                            watermark_record[1],
                            now_sec,
                            self.pure_pursuit_envelope_timeout_sec,
                        )
                        debug_rejection_sink.append(
                            {
                                "command_stamp_sec": int(envelope.command.stamp.sec),
                                "command_stamp_nanosec": int(
                                    envelope.command.stamp.nanosec
                                ),
                                "envelope_command_stamp_sec": int(
                                    envelope.command.stamp.sec
                                ),
                                "envelope_command_stamp_nanosec": int(
                                    envelope.command.stamp.nanosec
                                ),
                                "tracking_usable": bool(
                                    envelope.trajectory_tracking_usable
                                ),
                                "selection_rejected_for_invalid_tracking": bool(
                                    fresh
                                    and not bool(watermark_record[2])
                                    and str(watermark_record[3])
                                    == "trajectory_tracking_unusable"
                                ),
                                "selection_rejection_kind": (
                                    "invalid_tracking"
                                    if fresh
                                    and not bool(watermark_record[2])
                                    and str(watermark_record[3])
                                    == "trajectory_tracking_unusable"
                                    else (
                                        "current_envelope_stale"
                                        if not fresh
                                        else "current_envelope_invalid"
                                    )
                                ),
                            }
                        )
                    return None
                if watermark_delivery_gap:
                    watermark = None
                else:
                    envelope = copy.deepcopy(watermark_record[4])
                    proof = ControllerTrackingStatus()
                    proof.header.stamp = copy.deepcopy(envelope.header.stamp)
                    proof.header.frame_id = "base_link"
                    proof.plan_generation = int(envelope.plan_generation)
                    proof.mpc_horizon_usable = bool(envelope.mpc_horizon_usable)
                    proof.pp_command_fresh = bool(envelope.pp_command_fresh)
                    proof.trajectory_tracking_usable = bool(
                        envelope.trajectory_tracking_usable
                    )
                    proof.lateral_stop_authority_kind = int(
                        envelope.lateral_stop_authority_kind
                    )
                    proof.lateral_stop_transaction_pass_direction = int(
                        envelope.lateral_stop_transaction_pass_direction
                    )
                    proof.lateral_stop_authority_token = int(
                        envelope.lateral_stop_authority_token
                    )
                    proof.command_age_sec = float(envelope.command_age_sec)
                    proof.reason = str(envelope.reason)
                    return self._build_selected_pp_motion_sample(
                        envelope.command,
                        watermark_record[1],
                        authority_proof=proof,
                        authority_proof_receipt_time_sec=watermark_record[1],
                        authority_proof_valid=True,
                        observed_proof=proof,
                        observed_proof_receipt_time_sec=watermark_record[1],
                        observed_proof_valid=True,
                        envelope_identity=watermark_identity,
                    )
        # A received current-generation envelope is an authoritative hard
        # negative when its contract is unusable.  Never evade it by reusing
        # an older N-1 command; only a missing current record may bridge.
        current_records = [
            (identity, record)
            for identity, record in self.pure_pursuit_envelope_cache.items()
            if int(identity[0]) == int(active_producer)
            and int(identity[3]) == int(tracking_plan_generation)
            and not self._pure_pursuit_envelope_is_unpromoted_successor_preview(
                identity
            )
        ]
        if current_records:
            _, current_record = max(
                current_records,
                key=lambda item: int(item[0][1]),
            )
            current_delivery_gap = False
            if (
                not bool(current_record[2])
                or not self._fresh(
                    current_record[1],
                    now_sec,
                    self.pure_pursuit_envelope_timeout_sec,
                )
            ) and not current_delivery_gap:
                if debug_rejection_sink is not None:
                    envelope = current_record[4]
                    fresh = self._fresh(
                        current_record[1],
                        now_sec,
                        self.pure_pursuit_envelope_timeout_sec,
                    )
                    debug_rejection_sink.append(
                        {
                            "command_stamp_sec": int(envelope.command.stamp.sec),
                            "command_stamp_nanosec": int(
                                envelope.command.stamp.nanosec
                            ),
                            "envelope_command_stamp_sec": int(
                                envelope.command.stamp.sec
                            ),
                            "envelope_command_stamp_nanosec": int(
                                envelope.command.stamp.nanosec
                            ),
                            "tracking_usable": bool(
                                envelope.trajectory_tracking_usable
                            ),
                            "selection_rejected_for_invalid_tracking": bool(
                                fresh
                                and not bool(current_record[2])
                                and str(current_record[3])
                                == "trajectory_tracking_unusable"
                            ),
                            "selection_rejection_kind": (
                                "invalid_tracking"
                                if fresh
                                and not bool(current_record[2])
                                and str(current_record[3])
                                == "trajectory_tracking_unusable"
                                else (
                                    "current_envelope_stale"
                                    if not fresh
                                    else "current_envelope_invalid"
                                )
                            ),
                        }
                    )
                return None
            generations = (
                (previous_generation,)
                if current_delivery_gap
                else (tracking_plan_generation,)
            )
        else:
            # Preserve the existing direct N-1 selector contract for
            # zero-speed and non-PASSING control.  PASSING positive motion is
            # filtered at the timer boundary and may cross only via its
            # committed-cohort lease.
            generations = (previous_generation,)
        for generation in generations:
            candidates = [
                (identity, record)
                for identity, record in self.pure_pursuit_envelope_cache.items()
                if int(identity[0]) == int(active_producer)
                and int(identity[3]) == int(generation)
                and not self._pure_pursuit_envelope_is_unpromoted_successor_preview(
                    identity
                )
                and bool(record[2])
                and self._fresh(
                    record[1], now_sec, self.pure_pursuit_envelope_timeout_sec
                )
            ]
            if not candidates:
                continue
            identity, record = max(candidates, key=lambda item: int(item[0][1]))
            envelope = copy.deepcopy(record[4])
            proof = ControllerTrackingStatus()
            proof.header.stamp = copy.deepcopy(envelope.header.stamp)
            proof.header.frame_id = "base_link"
            proof.plan_generation = int(envelope.plan_generation)
            proof.mpc_horizon_usable = bool(envelope.mpc_horizon_usable)
            proof.pp_command_fresh = bool(envelope.pp_command_fresh)
            proof.trajectory_tracking_usable = bool(
                envelope.trajectory_tracking_usable
            )
            proof.lateral_stop_authority_kind = int(
                envelope.lateral_stop_authority_kind
            )
            proof.lateral_stop_transaction_pass_direction = int(
                envelope.lateral_stop_transaction_pass_direction
            )
            proof.lateral_stop_authority_token = int(
                envelope.lateral_stop_authority_token
            )
            proof.command_age_sec = float(envelope.command_age_sec)
            proof.reason = str(envelope.reason)
            return self._build_selected_pp_motion_sample(
                envelope.command,
                record[1],
                authority_proof=proof,
                authority_proof_receipt_time_sec=record[1],
                authority_proof_valid=True,
                observed_proof=proof,
                observed_proof_receipt_time_sec=record[1],
                observed_proof_valid=True,
                envelope_identity=identity,
            )
        return None

    def _select_bound_envelope_observed_proof(
        self, now_sec: float
    ) -> Optional[ControllerTrackingStatus]:
        """Latest structural envelope is telemetry only, never authority."""
        active = self.pure_pursuit_envelope_active_producer_instance_id
        if active is None:
            return None
        records = [
            record for identity, record in self.pure_pursuit_envelope_cache.items()
            if int(identity[0]) == int(active)
            and self._fresh(record[1], now_sec, self.pure_pursuit_envelope_timeout_sec)
        ]
        if not records:
            return None
        envelope = max(records, key=lambda record: int(record[4].command_sequence))[4]
        proof = ControllerTrackingStatus()
        proof.header.stamp = copy.deepcopy(envelope.header.stamp)
        proof.header.frame_id = "base_link"
        proof.plan_generation = int(envelope.plan_generation)
        proof.mpc_horizon_usable = bool(envelope.mpc_horizon_usable)
        proof.pp_command_fresh = bool(envelope.pp_command_fresh)
        proof.trajectory_tracking_usable = bool(envelope.trajectory_tracking_usable)
        proof.command_age_sec = float(envelope.command_age_sec)
        proof.reason = str(envelope.reason)
        return proof

    def _select_pass_warmup_acquisition_sample(
        self,
        *,
        tracking_plan_generation: int,
        now_sec: float,
        ros_clock_stalled: bool,
    ) -> Optional[PurePursuitMotionSample]:
        """Select a bound unusable command only for typed zero-speed warmup.

        This does not alter the normal motion selector.  The returned authority
        proof deliberately remains trajectory_tracking_usable=false, so it
        cannot satisfy any motion proof; it can only reach the later typed
        PASS_WARMUP zero-speed composition.
        """
        active = self.pure_pursuit_envelope_active_producer_instance_id
        if (
            self.pure_pursuit_envelope_fault_latched
            or ros_clock_stalled
            or tracking_plan_generation <= 0
        ):
            return None
        if active is None:
            bootstrap_records = []
            for identity, record in self.pure_pursuit_envelope_cache.items():
                envelope = record[4]
                structural, _ = self._validate_pure_pursuit_envelope(
                    envelope, allow_tracking_unusable=True
                )
                if (
                    structural
                    and not bool(envelope.trajectory_tracking_usable)
                    and int(envelope.schema_version) == 2
                    and int(envelope.lateral_stop_authority_kind)
                    == int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
                    and str(record[3]) == "trajectory_tracking_unusable"
                    and self._fresh(
                        record[1], now_sec, self.pure_pursuit_envelope_timeout_sec
                    )
                ):
                    bootstrap_records.append((identity, record))
            producers = {int(identity[0]) for identity, _ in bootstrap_records}
            if len(producers) != 1:
                return None
            active = next(iter(producers))
            ordered = sorted(
                (
                    (int(identity[1]), self._stamp_ns(record[4].header.stamp))
                    for identity, record in bootstrap_records
                    if int(identity[0]) == int(active)
                ),
                key=lambda item: item[0],
            )
            if len(ordered) < 2 or any(
                later_sequence <= earlier_sequence
                or later_stamp < earlier_stamp
                for (earlier_sequence, earlier_stamp),
                (later_sequence, later_stamp) in zip(ordered, ordered[1:])
            ):
                return None
        candidates = sorted(
            (
                (identity, record)
                for identity, record in self.pure_pursuit_envelope_cache.items()
                if int(identity[0]) == int(active)
                and int(identity[3]) == int(tracking_plan_generation)
                and str(record[3]) == "trajectory_tracking_unusable"
                and self._fresh(
                    record[1], now_sec, self.pure_pursuit_envelope_timeout_sec
                )
            ),
            key=lambda item: int(item[0][1]),
            reverse=True,
        )
        for identity, record in candidates:
            envelope = copy.deepcopy(record[4])
            structurally_valid, _ = self._validate_pure_pursuit_envelope(
                envelope, allow_tracking_unusable=True
            )
            if (
                not structurally_valid
                or bool(envelope.trajectory_tracking_usable)
                or int(envelope.schema_version) != 2
                or int(envelope.lateral_stop_authority_kind)
                != int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
            ):
                continue
            status_key = (
                self._stamp_ns(envelope.header.stamp),
                int(envelope.plan_generation),
            )
            status_record = self.pure_pursuit_tracking_status_cache.get(status_key)
            if (
                status_record is None
                or not bool(status_record[2])
                or not self._fresh(
                    status_record[1],
                    now_sec,
                    self.pure_pursuit_tracking_status_timeout_sec,
                )
            ):
                continue
            status = status_record[0]
            if (
                bool(status.trajectory_tracking_usable)
                or not bool(status.pass_warmup_steering_acquisition_active)
                or bool(status.pass_warmup_motion_ready)
                or int(status.lateral_stop_authority_kind)
                != int(envelope.lateral_stop_authority_kind)
                or int(status.lateral_stop_transaction_pass_direction)
                != int(envelope.lateral_stop_transaction_pass_direction)
                or int(status.lateral_stop_authority_token)
                != int(envelope.lateral_stop_authority_token)
                or not self._tracking_status_binds_pure_pursuit_command(
                    status, envelope.command
                )
            ):
                continue
            return self._build_selected_pp_motion_sample(
                envelope.command,
                record[1],
                authority_proof=status,
                authority_proof_receipt_time_sec=status_record[1],
                authority_proof_valid=True,
                observed_proof=status,
                observed_proof_receipt_time_sec=status_record[1],
                observed_proof_valid=True,
                envelope_identity=identity,
            )
        return None

    def _select_pp_motion_sample(
        self,
        *,
        tracking_plan_generation: int,
        now_sec: float,
        ros_clock_stalled: bool,
        debug_rejection_sink: Optional[list[dict[str, object]]] = None,
    ) -> Optional[PurePursuitMotionSample]:
        """Choose U2 envelope authority only for required safety contracts."""
        if self.require_safety_constraint:
            return self._select_bound_envelope_pp_motion_sample(
                tracking_plan_generation=tracking_plan_generation,
                now_sec=now_sec,
                ros_clock_stalled=ros_clock_stalled,
                debug_rejection_sink=debug_rejection_sink,
            )
        return self._select_legacy_pp_motion_sample(
            tracking_plan_generation=tracking_plan_generation
        )

    def _debug_pure_pursuit_evaluation_snapshot(
        self,
        selected_sample: Optional[PurePursuitMotionSample],
        *,
        selection_debug_snapshot: Optional[dict[str, object]],
    ) -> dict[str, object]:
        """Describe the PP inputs considered by this Mux cycle.

        The rejection snapshot is captured at the actual selector branch in
        this timer invocation.  This helper never re-selects or re-reads the
        mutable envelope cache after selection, so telemetry cannot invent a
        stop origin from a later callback.
        """
        if selection_debug_snapshot is not None:
            return dict(selection_debug_snapshot)
        if selected_sample is None:
            return {
                "command_stamp_sec": None,
                "command_stamp_nanosec": None,
                "envelope_command_stamp_sec": None,
                "envelope_command_stamp_nanosec": None,
                "tracking_usable": None,
                "selection_rejected_for_invalid_tracking": False,
                "selection_rejection_kind": "no_selector_rejection_snapshot",
            }
        command = selected_sample.command
        return {
            "command_stamp_sec": int(command.stamp.sec),
            "command_stamp_nanosec": int(command.stamp.nanosec),
            "envelope_command_stamp_sec": int(command.stamp.sec),
            "envelope_command_stamp_nanosec": int(command.stamp.nanosec),
            "tracking_usable": bool(
                selected_sample.authority_proof.trajectory_tracking_usable
            )
            if selected_sample.authority_proof is not None
            else None,
            "selection_rejected_for_invalid_tracking": False,
            "selection_rejection_kind": "none",
        }

    def _exact_current_motion_envelope(
        self,
        sample: Optional[PurePursuitMotionSample],
        *,
        tracking_plan_generation: int,
        now_sec: float,
    ) -> Optional[ControllerCommandEnvelope]:
        """Resolve only the exact current-generation immutable PP envelope."""
        if sample is None or sample.envelope_identity is None:
            return None
        identity = tuple(sample.envelope_identity)
        if (
            len(identity) != 4
            or self.pure_pursuit_envelope_fault_latched
            or self.pure_pursuit_envelope_active_producer_instance_id is None
            or self.pure_pursuit_envelope_active_sequence is None
            or int(identity[0])
            != int(self.pure_pursuit_envelope_active_producer_instance_id)
            or int(identity[1])
            != int(self.pure_pursuit_envelope_active_sequence)
            or int(identity[3]) != int(tracking_plan_generation)
        ):
            return None
        record = self.pure_pursuit_envelope_cache.get(identity)
        if record is None:
            watermark = self.pure_pursuit_envelope_watermark
            if watermark is not None and tuple(watermark[0]) == identity:
                record = watermark[1]
        if (
            record is None
            or not bool(record[2])
            or not self._fresh(
                record[1], now_sec, self.pure_pursuit_envelope_timeout_sec
            )
        ):
            return None
        return copy.deepcopy(record[4])

    def _motion_authority_final_steering_binding_valid(
        self,
        *,
        envelope_steering_rad: float,
        final_steering_rad: float,
        steering_result: SteeringLimitResult,
    ) -> bool:
        """Bind one PP steering input to the exact Mux-limited final output."""
        raw_steering_rad = float(steering_result.raw_steering_rad)
        limited_steering_rad = float(steering_result.limited_steering_rad)
        steering_delta_rad = float(steering_result.steering_delta_rad)
        values = (
            float(envelope_steering_rad),
            float(final_steering_rad),
            raw_steering_rad,
            limited_steering_rad,
            steering_delta_rad,
        )
        max_steering_angle_rad = float(
            self.steering_limiter.config.max_steering_angle_rad
        )
        if (
            not all(math.isfinite(value) for value in values)
            or not math.isfinite(max_steering_angle_rad)
            or max_steering_angle_rad < 0.0
            or float(envelope_steering_rad) != raw_steering_rad
            or float(final_steering_rad) != limited_steering_rad
            or bool(steering_result.angle_limited)
            or abs(limited_steering_rad) > max_steering_angle_rad + 1.0e-12
        ):
            return False
        rate_transform_observed = not math.isclose(
            limited_steering_rad,
            raw_steering_rad,
            abs_tol=1.0e-12,
        )
        return bool(
            bool(steering_result.rate_limited) == rate_transform_observed
            and not (
                bool(steering_result.rate_limited)
                and bool(steering_result.limiter_reset)
            )
        )

    def _motion_authority_committed_steering_binding_valid(
        self,
        *,
        envelope_steering_rad: float,
        final_steering_rad: float,
        steering_result: Optional[SteeringLimitResult],
    ) -> bool:
        """Validate stored limiter provenance, including legacy exact commits."""
        if steering_result is None:
            return bool(
                math.isfinite(float(envelope_steering_rad))
                and math.isfinite(float(final_steering_rad))
                and float(envelope_steering_rad) == float(final_steering_rad)
            )
        return self._motion_authority_final_steering_binding_valid(
            envelope_steering_rad=envelope_steering_rad,
            final_steering_rad=final_steering_rad,
            steering_result=steering_result,
        )

    def _motion_authority_expected_longitudinal_values(
        self,
        *,
        envelope_command: AckermannControlCommand,
        constraint_decision: SafetyConstraintDecision,
        now_sec: float,
    ) -> Optional[tuple[float, float, float, float]]:
        """Replay the deterministic Mux transforms used before grant binding."""
        envelope_values = (
            float(envelope_command.longitudinal.speed),
            float(envelope_command.longitudinal.acceleration),
            float(envelope_command.longitudinal.jerk),
            float(envelope_command.lateral.steering_tire_rotation_rate),
        )
        if (
            not all(math.isfinite(value) for value in envelope_values)
            or constraint_decision.stop_required
        ):
            return None
        expected = self._fallback_command(
            copy.deepcopy(envelope_command), now_sec
        )
        expected, expected_source, _ = self._apply_safety_constraint(
            expected,
            "pure_pursuit",
            "motion_authority_expected_transform",
            constraint_decision,
            now_sec,
        )
        expected_values = (
            float(expected.longitudinal.speed),
            float(expected.longitudinal.acceleration),
            float(expected.longitudinal.jerk),
            float(expected.lateral.steering_tire_rotation_rate),
        )
        if (
            expected_source != "pure_pursuit"
            or not all(math.isfinite(value) for value in expected_values)
        ):
            return None
        return expected_values

    def _motion_authority_grant_eligible(
        self,
        *,
        final_command: AckermannControlCommand,
        selected_sample: Optional[PurePursuitMotionSample],
        tracking_plan_generation: int,
        authority_plan_generation: Optional[int],
        authority_constraint: Optional[SafetyConstraintState],
        constraint_decision: SafetyConstraintDecision,
        decision_source: str,
        now_sec: float,
        active_control_fault_reason: str,
        ros_clock_stalled: bool,
        deadline_missed: bool,
        steering_result: SteeringLimitResult,
        delivery_gap_lease_active: bool = False,
    ) -> tuple[bool, str, Optional[ControllerCommandEnvelope], Optional[tuple]]:
        """Validate one exact PASS command immediately before final publish."""
        if str(decision_source) != "pure_pursuit":
            return False, "final_source", None, None
        if (
            self.external_stop_latched
            or self.control_fault_latched
            or bool(active_control_fault_reason)
            or bool(ros_clock_stalled)
            or bool(deadline_missed)
        ):
            return False, "higher_priority_stop", None, None
        if self.finish_stop_latch.latched or not self.finish_stop_latch.armed:
            return False, "race_authority", None, None
        if (
            authority_plan_generation is None
            or int(authority_plan_generation) != int(tracking_plan_generation)
        ):
            return False, "plan_generation", None, None
        if (
            authority_constraint is None
            or not authority_constraint.valid
            or authority_constraint.stop_requested
            or not authority_constraint.release_authorized
            or constraint_decision.stop_required
            or int(authority_constraint.plan_generation)
            != int(tracking_plan_generation)
        ):
            return False, "constraint", None, None
        plan_identity = self.overtake_plan_motion_identity_cache.get(
            int(tracking_plan_generation)
        )
        if plan_identity is None:
            return False, "plan_identity_missing", None, None
        (
            race_arm_epoch,
            planner_instance_id,
            attempt_id,
            target_vehicle_id,
            pass_direction,
            connector_transaction_id,
            plan_stamp_ns,
            plan_generation,
            phase,
            candidate_revision,
            candidate_digest,
            identity_valid,
        ) = plan_identity
        if (
            not identity_valid
            or int(phase) != int(OvertakePlan.PASSING)
            or int(pass_direction) not in (-1, 1)
            or int(race_arm_epoch) != int(self.finish_stop_latch.epoch)
            or int(authority_constraint.header_stamp_ns) != int(plan_stamp_ns)
            or int(plan_generation) != int(tracking_plan_generation)
            or not any(candidate_digest)
        ):
            return False, "plan_identity", None, plan_identity
        if delivery_gap_lease_active:
            lease = self.motion_authority_delivery_gap_lease
            envelope = (
                copy.deepcopy(lease.envelope)
                if lease is not None
                else None
            )
        else:
            envelope = self._exact_current_motion_envelope(
                selected_sample,
                tracking_plan_generation=tracking_plan_generation,
                now_sec=now_sec,
            )
        if envelope is None or int(envelope.schema_version) != 2:
            return False, "envelope_schema", envelope, plan_identity
        envelope_identity = self._pure_pursuit_envelope_plan_identity(
            envelope
        )
        expected_identity = (
            int(race_arm_epoch),
            int(planner_instance_id),
            int(attempt_id),
            str(target_vehicle_id),
            int(pass_direction),
            int(connector_transaction_id),
            int(plan_stamp_ns),
            int(plan_generation),
            int(candidate_revision),
            bytes(candidate_digest),
        )
        if envelope_identity != expected_identity:
            return False, "envelope_plan_binding", envelope, plan_identity
        envelope_command = envelope.command
        envelope_longitudinal_values = (
            float(envelope_command.longitudinal.speed),
            float(envelope_command.longitudinal.acceleration),
            float(envelope_command.longitudinal.jerk),
            float(envelope_command.lateral.steering_tire_rotation_rate),
        )
        final_longitudinal_values = (
            float(final_command.longitudinal.speed),
            float(final_command.longitudinal.acceleration),
            float(final_command.longitudinal.jerk),
            float(final_command.lateral.steering_tire_rotation_rate),
        )
        expected_final_longitudinal_values = (
            self._motion_authority_expected_longitudinal_values(
                envelope_command=envelope_command,
                constraint_decision=constraint_decision,
                now_sec=now_sec,
            )
        )
        if expected_final_longitudinal_values is None:
            return False, "final_command_binding", envelope, plan_identity
        if delivery_gap_lease_active:
            lease = self.motion_authority_delivery_gap_lease
            successor_constraint = self.safety_constraint
            direct_successor = (
                1
                if lease is not None
                and int(lease.plan_generation) >= 16_777_215
                else (
                    int(lease.plan_generation) + 1
                    if lease is not None
                    else 0
                )
            )
            constrained_lease_command = (
                copy.deepcopy(lease.final_command)
                if lease is not None
                else None
            )
            if (
                constrained_lease_command is not None
                and successor_constraint is not None
                and int(successor_constraint.plan_generation)
                == int(direct_successor)
            ):
                constrained_lease_command = (
                    self._delivery_gap_stricter_longitudinal_command(
                        constrained_lease_command,
                        successor_constraint,
                    )
                )
            if constrained_lease_command is not None:
                expected_final_longitudinal_values = (
                    float(constrained_lease_command.longitudinal.speed),
                    float(constrained_lease_command.longitudinal.acceleration),
                    float(constrained_lease_command.longitudinal.jerk),
                    float(
                        constrained_lease_command.lateral.steering_tire_rotation_rate
                    ),
                )
            steering_binding_valid = bool(
                lease is not None
                and constrained_lease_command is not None
                and selected_sample is not None
                and (
                    float(selected_sample.command.longitudinal.speed),
                    float(selected_sample.command.longitudinal.acceleration),
                    float(selected_sample.command.longitudinal.jerk),
                    float(
                        selected_sample.command.lateral.steering_tire_rotation_rate
                    ),
                )
                == expected_final_longitudinal_values
                and self._motion_authority_committed_steering_binding_valid(
                    envelope_steering_rad=float(
                        envelope_command.lateral.steering_tire_angle
                    ),
                    final_steering_rad=float(
                        lease.final_command.lateral.steering_tire_angle
                    ),
                    steering_result=lease.steering_result,
                )
                and float(selected_sample.command.lateral.steering_tire_angle)
                == float(lease.final_command.lateral.steering_tire_angle)
                and float(final_command.lateral.steering_tire_angle)
                == float(lease.final_command.lateral.steering_tire_angle)
                and self._motion_authority_final_steering_binding_valid(
                    envelope_steering_rad=float(
                        lease.final_command.lateral.steering_tire_angle
                    ),
                    final_steering_rad=float(
                        final_command.lateral.steering_tire_angle
                    ),
                    steering_result=steering_result,
                )
            )
        else:
            steering_binding_valid = (
                self._motion_authority_final_steering_binding_valid(
                    envelope_steering_rad=float(
                        envelope_command.lateral.steering_tire_angle
                    ),
                    final_steering_rad=float(
                        final_command.lateral.steering_tire_angle
                    ),
                    steering_result=steering_result,
                )
            )
        if (
            expected_final_longitudinal_values != final_longitudinal_values
            or not steering_binding_valid
        ):
            return False, "final_command_binding", envelope, plan_identity
        if (
            int(envelope.producer_instance_id) <= 0
            or int(envelope.command_sequence) <= 0
        ):
            return False, "command_identity", envelope, plan_identity
        # State has already authorized this safety-evaluated candidate and its
        # entry-speed cap.  The exact current-generation envelope and final
        # command binding above are the execution proof; no earlier zero-speed
        # warmup sample is required.
        return True, "ready", envelope, plan_identity

    def _invalidate_motion_authority_delivery_gap_lease(
        self,
        *,
        stop_pending: bool = False,
        reason: DeliveryGapRevokeReason = DeliveryGapRevokeReason.UNSPECIFIED,
        origin: str = "unspecified",
        observed_generation: Optional[int] = None,
        now_sec: Optional[float] = None,
        plan_relation_subreason: Optional[
            DeliveryGapPlanRelationSubreason
        ] = None,
    ) -> None:
        """Revoke the one-shot PASSING delivery-gap authority immediately."""
        lease = self.motion_authority_delivery_gap_lease
        if stop_pending and lease is not None:
            self.motion_authority_delivery_gap_stop_pending = True
        if lease is not None and reason != DeliveryGapRevokeReason.UNSPECIFIED:
            gap_age_sec: Optional[float] = None
            if now_sec is not None and lease.gap_started_steady_time_sec is not None:
                gap_age_sec = float(now_sec) - float(
                    lease.gap_started_steady_time_sec
                )
            record = DeliveryGapRevokeRecord(
                reason=reason,
                origin=str(origin),
                lease_generation=int(lease.plan_generation),
                observed_generation=(
                    int(observed_generation)
                    if observed_generation is not None
                    else None
                ),
                gap_age_sec=gap_age_sec,
                plan_relation_subreason=plan_relation_subreason,
            )
            # First reason wins until the timer consumes the corresponding STOP.
            if self.motion_authority_delivery_gap_revoke_pending is None:
                self.motion_authority_delivery_gap_revoke_pending = record
                self.motion_authority_delivery_gap_last_revoke = record
                subreason_code = (
                    self._delivery_gap_plan_relation_subreason_code(
                        plan_relation_subreason
                    )
                )
                subreason_name = (
                    plan_relation_subreason.name
                    if plan_relation_subreason is not None
                    else "NONE"
                )
                self.get_logger().warning(
                    "delivery_gap_revoke "
                    f"code=DG{int(reason):03d} name={reason.name} "
                    f"subcode={subreason_code} subname={subreason_name} "
                    f"origin={origin} n={int(lease.plan_generation)} "
                    f"observed={observed_generation} gap_age_sec={gap_age_sec}"
                )
        self.motion_authority_delivery_gap_lease = None

    @staticmethod
    def _delivery_gap_plan_relation_subreason_code(
        subreason: Optional[DeliveryGapPlanRelationSubreason],
    ) -> str:
        if subreason is None:
            return "NONE"
        return f"DG408-S{int(subreason):02d}"

    def _motion_release_constraint_valid(
        self,
        constraint: Optional[SafetyConstraintState],
        *,
        receipt_time_sec: Optional[float],
        now_sec: float,
        expected_generation: Optional[int] = None,
        expected_stamp_ns: Optional[int] = None,
    ) -> bool:
        """Check the complete non-STOP release contract without latching.

        Delivery-gap preview is deliberately side-effect free.  In particular,
        this helper must not call the live ``SafetyConstraintAuthority``: a
        successor preview is only evidence for deciding whether the committed
        cohort may remain held, never an authority transition for that cohort.
        """
        if constraint is None or not self._fresh(
            receipt_time_sec, now_sec, self.safety_constraint_timeout_sec
        ):
            return False
        if (
            not constraint.valid
            or constraint.stop_requested
            or not constraint.release_authorized
            or constraint.plan_generation < 0
            or constraint.constraint_generation < 0
            or constraint.header_stamp_ns < 0
            or str(constraint.frame_id) != "map"
            or not math.isfinite(float(constraint.speed_limit_mps))
            or not 0.0 < float(constraint.speed_limit_mps)
            <= float(self.safety_constraint_max_speed_mps)
            or not math.isfinite(float(constraint.required_brake_decel_mps2))
            or not 0.0 <= float(constraint.required_brake_decel_mps2)
            <= float(self.safety_constraint_max_brake_decel_mps2)
        ):
            return False
        if (
            expected_generation is not None
            and int(constraint.plan_generation) != int(expected_generation)
        ):
            return False
        if (
            expected_stamp_ns is not None
            and int(constraint.header_stamp_ns) != int(expected_stamp_ns)
        ):
            return False
        return True

    def _motion_release_constraint_revoke_reason(
        self,
        constraint: Optional[SafetyConstraintState],
        *,
        receipt_time_sec: Optional[float],
        now_sec: float,
        expected_generation: Optional[int] = None,
        expected_stamp_ns: Optional[int] = None,
    ) -> Optional[DeliveryGapRevokeReason]:
        """Explain the existing release predicate without changing it."""
        if constraint is None:
            return DeliveryGapRevokeReason.N1_CONSTRAINT_CACHE_MISS
        if not self._fresh(
            receipt_time_sec, now_sec, self.safety_constraint_timeout_sec
        ):
            return DeliveryGapRevokeReason.N1_CONSTRAINT_STALE
        if not constraint.valid:
            return DeliveryGapRevokeReason.N1_CONSTRAINT_INVALID
        if constraint.stop_requested:
            return DeliveryGapRevokeReason.N1_CONSTRAINT_STOP_REQUESTED
        if not constraint.release_authorized:
            return DeliveryGapRevokeReason.N1_CONSTRAINT_RELEASE_WITHDRAWN
        if (
            constraint.plan_generation < 0
            or constraint.constraint_generation < 0
            or constraint.header_stamp_ns < 0
            or str(constraint.frame_id) != "map"
            or not math.isfinite(float(constraint.speed_limit_mps))
            or not 0.0 < float(constraint.speed_limit_mps)
            <= float(self.safety_constraint_max_speed_mps)
            or not math.isfinite(float(constraint.required_brake_decel_mps2))
            or not 0.0 <= float(constraint.required_brake_decel_mps2)
            <= float(self.safety_constraint_max_brake_decel_mps2)
        ):
            return DeliveryGapRevokeReason.N1_CONSTRAINT_FRAME_OR_VALUE_INVALID
        if (
            expected_generation is not None
            and int(constraint.plan_generation) != int(expected_generation)
        ):
            return DeliveryGapRevokeReason.LATEST_CONSTRAINT_GENERATION_UNEXPECTED
        if (
            expected_stamp_ns is not None
            and int(constraint.header_stamp_ns) != int(expected_stamp_ns)
        ):
            return DeliveryGapRevokeReason.N1_CONSTRAINT_STAMP_MISMATCH
        return None

    def _coerce_safety_constraint_state(
        self, constraint: object
    ) -> SafetyConstraintState:
        """Normalize legacy test/fixture messages at the private lease edge."""
        if isinstance(constraint, SafetyConstraintState):
            return copy.deepcopy(constraint)
        header = getattr(constraint, "header", None)
        return SafetyConstraintState(
            constraint_generation=int(constraint.constraint_generation),
            plan_generation=int(constraint.plan_generation),
            valid=bool(constraint.valid),
            stop_requested=bool(constraint.stop_requested),
            release_authorized=bool(constraint.release_authorized),
            speed_limit_mps=float(constraint.speed_limit_mps),
            required_brake_decel_mps2=float(
                getattr(constraint, "required_brake_decel_mps2", 0.0)
            ),
            header_stamp_ns=self._stamp_ns(header.stamp) if header else 0,
            frame_id=str(header.frame_id) if header else "",
            reason=str(getattr(constraint, "reason", "")),
        )

    @staticmethod
    def _motion_constraint_payload_fingerprint(
        constraint: SafetyConstraintState,
    ) -> tuple:
        """Capture every committed safety-constraint payload field immutably."""
        return (
            int(constraint.constraint_generation),
            int(constraint.plan_generation),
            bool(constraint.valid),
            bool(constraint.stop_requested),
            bool(constraint.release_authorized),
            float(constraint.speed_limit_mps),
            float(constraint.required_brake_decel_mps2),
            int(constraint.header_stamp_ns),
            str(constraint.frame_id),
            str(constraint.reason),
        )

    @staticmethod
    def _delivery_gap_stricter_longitudinal_command(
        command: AckermannControlCommand,
        successor_constraint: SafetyConstraintState,
    ) -> AckermannControlCommand:
        """Apply only a stricter successor longitudinal bound to held N.

        The committed steering and every command field outside longitudinal
        speed/acceleration stay immutable.  This is never a relaxation: the
        successor constraint has already passed the exact release contract.
        """
        held = copy.deepcopy(command)
        held.longitudinal.speed = min(
            float(held.longitudinal.speed),
            float(successor_constraint.speed_limit_mps),
        )
        if float(successor_constraint.required_brake_decel_mps2) > 0.0:
            held.longitudinal.acceleration = min(
                float(held.longitudinal.acceleration),
                -float(successor_constraint.required_brake_decel_mps2),
            )
        return held

    @staticmethod
    def _lease_receipt_time(
        lease: MotionAuthorityDeliveryGapLease,
        field_name: str,
    ) -> float:
        """Read an original receipt, retaining compatibility with old fixtures."""
        value = getattr(lease, field_name, None)
        if value is None:
            return float(lease.acquired_steady_time_sec)
        return float(value)

    def _motion_authority_delivery_gap_lease_timeout_sec(self) -> float:
        """Bound the hold by the already-configured live-input freshness."""
        return min(
            float(self.motion_authority_delivery_gap_lease_sec),
            float(self.overtake_plan_timeout_sec),
            float(self.safety_constraint_timeout_sec),
            float(self.pure_pursuit_cmd_timeout_sec),
            float(self.pure_pursuit_tracking_status_timeout_sec),
            float(self.pure_pursuit_envelope_timeout_sec),
        )

    def _motion_authority_delivery_gap_envelope_fresh(
        self,
        lease: MotionAuthorityDeliveryGapLease,
        *,
        now_sec: float,
    ) -> bool:
        """Re-evaluate the committed envelope's age at every hold tick."""
        envelope = lease.envelope
        envelope_receipt_time_sec = self._lease_receipt_time(
            lease, "envelope_receipt_time_sec"
        )
        receipt_age_sec = now_sec - envelope_receipt_time_sec
        command_age_sec = float(envelope.command_age_sec)
        current_command_age_sec = command_age_sec + receipt_age_sec
        if (
            not math.isfinite(receipt_age_sec)
            or receipt_age_sec < 0.0
            or not math.isfinite(command_age_sec)
            or command_age_sec < 0.0
            or not math.isfinite(current_command_age_sec)
            or current_command_age_sec
            > float(self.pure_pursuit_envelope_timeout_sec)
        ):
            return False

        # Keep the same ROS-time rules as _validate_pure_pursuit_envelope:
        # clock-zero, future-stamp tolerance, and current header age are all
        # fail-closed while a delivery-gap lease is being held.
        header_stamp_ns = self._stamp_ns(envelope.header.stamp)
        now_ros_ns = int(self.get_clock().now().nanoseconds)
        future_tolerance_ns = int(
            self.pure_pursuit_envelope_future_stamp_tolerance_sec * 1.0e9
        )
        if now_ros_ns == 0 and header_stamp_ns != 0:
            return False
        if now_ros_ns > 0 and header_stamp_ns > now_ros_ns + future_tolerance_ns:
            return False
        if (
            now_ros_ns > 0
            and (now_ros_ns - header_stamp_ns) * 1.0e-9
            > float(self.pure_pursuit_envelope_timeout_sec)
        ):
            return False
        return True

    def _direct_motion_authority_successor_lease_sample(
        self,
        *,
        successor_generation: int,
        now_sec: float,
        authority_selection: SafetyAuthoritySelection,
        ros_clock_stalled: bool,
        active_control_fault_reason: str,
        deadline_missed: bool,
    ) -> Optional[PurePursuitMotionSample]:
        """Return only a committed N cohort during a bounded N+1 gap.

        This never treats an N+1 delivery as authority for N.  It merely keeps
        the immutable final command and the exact N contract alive until the
        direct successor is complete, and cannot renew itself from timer ticks
        or retransmissions.
        """
        lease = self.motion_authority_delivery_gap_lease
        if lease is None:
            return None

        def revoke(reason: DeliveryGapRevokeReason) -> None:
            self._invalidate_motion_authority_delivery_gap_lease(
                reason=reason,
                origin="timer_preview",
                observed_generation=int(successor_generation),
                now_sec=now_sec,
            )

        if (
            self.external_stop_latched
            or self.control_fault_latched
            or bool(active_control_fault_reason)
            or bool(ros_clock_stalled)
            or bool(deadline_missed)
            or self.finish_stop_latch.latched
            or not self.finish_stop_latch.armed
            or self.pure_pursuit_envelope_fault_latched
        ):
            reason = DeliveryGapRevokeReason.HIGHER_PRIORITY_STOP
            if ros_clock_stalled:
                reason = DeliveryGapRevokeReason.ROS_CLOCK_STALLED
            elif deadline_missed:
                reason = DeliveryGapRevokeReason.CONTROL_DEADLINE_MISSED
            elif not self.finish_stop_latch.armed:
                reason = DeliveryGapRevokeReason.RACE_NOT_ARMED
            elif self.pure_pursuit_envelope_fault_latched:
                reason = DeliveryGapRevokeReason.PP_ENVELOPE_FAULT_LATCHED
            revoke(reason)
            return None
        direct_successor = (
            1 if int(lease.plan_generation) >= 16_777_215
            else int(lease.plan_generation) + 1
        )
        if int(successor_generation) not in (
            int(lease.plan_generation),
            direct_successor,
        ):
            revoke(DeliveryGapRevokeReason.SUCCESSOR_NOT_DIRECT)
            return None
        if lease.gap_started_steady_time_sec is None:
            lease = replace(
                lease,
                gap_started_steady_time_sec=float(now_sec),
            )
            self.motion_authority_delivery_gap_lease = lease
        gap_started_steady_time_sec = float(
            lease.gap_started_steady_time_sec
        )
        lease_age_sec = now_sec - gap_started_steady_time_sec
        if (
            not math.isfinite(gap_started_steady_time_sec)
            or not math.isfinite(lease_age_sec)
            or not 0.0 <= lease_age_sec
            <= self._motion_authority_delivery_gap_lease_timeout_sec()
        ):
            revoke(
                DeliveryGapRevokeReason.GAP_CLOCK_INVALID
                if not math.isfinite(lease_age_sec) or lease_age_sec < 0.0
                else DeliveryGapRevokeReason.GAP_TIMEOUT
            )
            return None
        # Acquisition is not a receipt.  Every original N input must remain
        # fresh at this tick; otherwise a pre-aged cohort cannot gain a new
        # delivery-gap lifetime merely because the lease was minted later.
        original_receipts = (
            (
                "plan_receipt_time_sec",
                self.overtake_plan_timeout_sec,
                DeliveryGapRevokeReason.N_PLAN_STALE,
            ),
            (
                "constraint_receipt_time_sec",
                self.safety_constraint_timeout_sec,
                DeliveryGapRevokeReason.N_CONSTRAINT_STALE,
            ),
            (
                "tracking_receipt_time_sec",
                self.pure_pursuit_tracking_status_timeout_sec,
                DeliveryGapRevokeReason.N_TRACKING_STALE,
            ),
            (
                "envelope_receipt_time_sec",
                self.pure_pursuit_envelope_timeout_sec,
                DeliveryGapRevokeReason.N_ENVELOPE_STALE,
            ),
            (
                "command_receipt_time_sec",
                self.pure_pursuit_cmd_timeout_sec,
                DeliveryGapRevokeReason.N_COMMAND_STALE,
            ),
        )
        for receipt_name, timeout_sec, stale_reason in original_receipts:
            receipt = getattr(lease, receipt_name, None)
            if receipt is not None and not self._fresh(
                receipt,
                now_sec,
                timeout_sec,
            ):
                revoke(stale_reason)
                return None
        # Older in-tree fixtures constructed leases before the immutable
        # receipt fields existed.  Their acquisition time remains the only
        # defensible fallback and is still bounded by the short lease above.
        for receipt_name, timeout_sec, stale_reason in original_receipts:
            if getattr(lease, receipt_name, None) is None and not self._fresh(
                self._lease_receipt_time(lease, receipt_name),
                now_sec,
                timeout_sec,
            ):
                revoke(stale_reason)
                return None
        if (
            not math.isfinite(float(lease.envelope.command_age_sec))
            or float(lease.envelope.command_age_sec) < 0.0
            or float(lease.envelope.command_age_sec)
            > float(self.pure_pursuit_envelope_timeout_sec)
        ):
            revoke(DeliveryGapRevokeReason.N_ENVELOPE_COMMAND_AGE_INVALID)
            return None
        if (
            lease.envelope_header_stamp_ns is not None
            and int(lease.envelope_header_stamp_ns)
            != int(self._stamp_ns(lease.envelope.header.stamp))
        ) or (
            lease.command_header_stamp_ns is not None
            and int(lease.command_header_stamp_ns)
            != int(self._stamp_ns(lease.final_command.stamp))
        ):
            revoke(DeliveryGapRevokeReason.N_STAMP_CHANGED)
            return None
        if not self._motion_authority_delivery_gap_envelope_fresh(
            lease, now_sec=now_sec
        ):
            revoke(DeliveryGapRevokeReason.N_INPUT_STALE)
            return None
        current_generation_hold = bool(
            int(successor_generation) == int(lease.plan_generation)
        )
        successor_envelope_plan_identity: Optional[tuple] = None
        cached_successor_identity: Optional[tuple] = None
        if current_generation_hold:
            # Constraint-first delivery can leave the active plan at N while
            # a valid N+1 constraint (and possibly its envelope) is already
            # observed.  Keep the committed N identity immutable until Plan
            # N+1 joins; do not require a successor plan in this branch.
            current_identity = self.overtake_plan_motion_identity_cache.get(
                int(lease.plan_generation)
            )
            if (
                current_identity is None
                or tuple(current_identity) != tuple(lease.plan_identity)
                or (
                    lease.plan_lateral_stop_fingerprint is not None
                    and self.overtake_plan_lateral_stop_fingerprint_cache.get(
                        int(lease.plan_generation)
                    )
                    != lease.plan_lateral_stop_fingerprint
                )
            ):
                revoke(
                    DeliveryGapRevokeReason.N_PLAN_IDENTITY_CHANGED
                    if current_identity is None
                    or tuple(current_identity) != tuple(lease.plan_identity)
                    else DeliveryGapRevokeReason.N_LATERAL_STOP_FINGERPRINT_CHANGED
                )
                return None
            cached_successor_identity = (
                self.overtake_plan_motion_identity_cache.get(direct_successor)
            )
            if cached_successor_identity is not None and bool(
                cached_successor_identity[11]
            ):
                successor_envelope_plan_identity = (
                    *tuple(cached_successor_identity[:8]),
                    int(cached_successor_identity[9]),
                    bytes(cached_successor_identity[10]),
                )
            successor_identity = None
            successor_plan = None
        else:
            if int(successor_generation) != direct_successor:
                revoke(DeliveryGapRevokeReason.SUCCESSOR_NOT_DIRECT)
                return None
            successor_identity = self.overtake_plan_motion_identity_cache.get(
                int(successor_generation)
            )
            successor_plan = self.overtake_plan_cache.get(
                int(successor_generation)
            )
            if (
                successor_identity is None
                or successor_plan is None
                or not bool(successor_plan[0])
                or not bool(successor_plan[3])
                or not bool(successor_plan[4])
                or not bool(successor_identity[11])
                or int(successor_identity[8]) != int(OvertakePlan.PASSING)
                or int(successor_identity[6]) <= int(lease.plan_identity[6])
                or tuple(successor_identity[:6]) != tuple(lease.plan_identity[:6])
                # candidate_revision is intentionally a direct successor, not an
                # equality: State writes its wire generation into this field.
                or int(successor_identity[9]) != int(lease.plan_identity[9]) + 1
                or lease.successor_plan_identity is None
                or tuple(successor_identity)
                != tuple(lease.successor_plan_identity)
                or lease.successor_plan_lateral_stop_fingerprint is None
                or self.overtake_plan_lateral_stop_fingerprint_cache.get(
                    int(successor_generation)
                )
                != lease.successor_plan_lateral_stop_fingerprint
            ):
                revoke(DeliveryGapRevokeReason.N1_PLAN_MISSING_OR_INVALID)
                return None
            successor_envelope_plan_identity = (
                *tuple(successor_identity[:8]),
                int(successor_identity[9]),
                bytes(successor_identity[10]),
            )
        # A current successor constraint must either be wholly absent, or be
        # an exact fresh release with no tightening. Anything received but
        # malformed/STOP/stale is a hard negative, never a delivery gap.
        successor_stamp_ns = (
            int(self.safety_constraint.header_stamp_ns)
            if current_generation_hold and self.safety_constraint is not None
            else int(successor_identity[6])
        )
        if current_generation_hold and self.safety_constraint is None:
            revoke(
                DeliveryGapRevokeReason.N1_CONSTRAINT_MISSING_AFTER_OBSERVED
            )
            return None
        if current_generation_hold and cached_successor_identity is not None:
            if (
                not bool(cached_successor_identity[11])
                or int(cached_successor_identity[6]) != successor_stamp_ns
            ):
                revoke(DeliveryGapRevokeReason.N1_CONSTRAINT_STAMP_MISMATCH)
                return None
        successor_constraint_generation = (
            direct_successor if current_generation_hold else int(successor_generation)
        )
        successor_constraint = self.safety_constraint_contract_cache.get(
            (int(successor_constraint_generation), successor_stamp_ns)
        )
        latest_constraint = self.safety_constraint
        committed_constraint_state = self._coerce_safety_constraint_state(
            lease.constraint
        )
        committed_constraint_stamp_ns = (
            int(lease.constraint_header_stamp_ns)
            if lease.constraint_header_stamp_ns is not None
            else int(committed_constraint_state.header_stamp_ns)
        )
        committed_constraint_fingerprint = (
            lease.constraint_payload_fingerprint
            if lease.constraint_payload_fingerprint is not None
            else self._motion_constraint_payload_fingerprint(
                committed_constraint_state
            )
        )
        if (
            latest_constraint is not None
            and int(latest_constraint.plan_generation)
            not in (int(lease.plan_generation), int(successor_constraint_generation))
        ):
            revoke(
                DeliveryGapRevokeReason.LATEST_CONSTRAINT_GENERATION_UNEXPECTED
            )
            return None
        if (
            latest_constraint is not None
            and int(latest_constraint.plan_generation)
            == int(lease.plan_generation)
            and int(latest_constraint.header_stamp_ns)
            == committed_constraint_stamp_ns
            and self._motion_constraint_payload_fingerprint(latest_constraint)
            != committed_constraint_fingerprint
        ):
            revoke(DeliveryGapRevokeReason.N_CONSTRAINT_PAYLOAD_MUTATED)
            return None
        if (
            latest_constraint is not None
            and successor_constraint is not None
            and int(latest_constraint.plan_generation)
            == int(successor_constraint_generation)
            and self._motion_constraint_payload_fingerprint(latest_constraint)
            != self._motion_constraint_payload_fingerprint(
                successor_constraint[0]
            )
        ):
            revoke(DeliveryGapRevokeReason.N1_CONSTRAINT_PAYLOAD_MUTATED)
            return None
        if (
            latest_constraint is not None
            and int(latest_constraint.plan_generation)
            == int(successor_constraint_generation)
            and successor_constraint is None
        ):
            revoke(DeliveryGapRevokeReason.N1_CONSTRAINT_CACHE_MISS)
            return None
        if latest_constraint is not None:
            latest_constraint_receipt = self.safety_constraint_time_sec
            latest_expected_stamp = (
                successor_stamp_ns
                if int(latest_constraint.plan_generation)
                == int(successor_constraint_generation)
                else int(lease.plan_identity[6])
            )
            if not self._motion_release_constraint_valid(
                latest_constraint,
                receipt_time_sec=latest_constraint_receipt,
                now_sec=now_sec,
                expected_generation=int(latest_constraint.plan_generation),
                expected_stamp_ns=latest_expected_stamp,
            ):
                revoke(DeliveryGapRevokeReason.N1_CONSTRAINT_INVALID)
                return None
        if successor_constraint is not None:
            constraint, receipt_time_sec = successor_constraint
            if (
                not self._motion_release_constraint_valid(
                    constraint,
                    receipt_time_sec=receipt_time_sec,
                    now_sec=now_sec,
                    expected_generation=int(successor_constraint_generation),
                    expected_stamp_ns=successor_stamp_ns,
                )
            ):
                revoke(DeliveryGapRevokeReason.N1_CONSTRAINT_INVALID)
                return None
            # Constraint N+1 may arrive before PP N+1. A valid/fresh N+1
            # envelope may also arrive before the complete successor cohort;
            # retain exact N until the exact Constraint joins. Invalid,
            # unusable, stale, or identity-mutated successor data remains a
            # hard negative in the checks below.
        if authority_selection.rendezvous_state not in (
            SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP,
            SafetyAuthorityRendezvousState.EXACT_CURRENT,
        ):
            revoke(DeliveryGapRevokeReason.RENDEZVOUS_NOT_HOLDABLE)
            return None
        if current_generation_hold:
            for identity, record in self.pure_pursuit_envelope_cache.items():
                if int(identity[3]) != int(direct_successor):
                    continue
                envelope_revoke_reason = (
                    self._delivery_gap_successor_envelope_revoke_reason(
                        lease,
                        identity=identity,
                        record=record,
                        expected_generation=direct_successor,
                        expected_successor_stamp_ns=successor_stamp_ns,
                        expected_plan_identity=successor_envelope_plan_identity,
                        now_sec=now_sec,
                    )
                )
                if envelope_revoke_reason is not None:
                    revoke(envelope_revoke_reason)
                    return None
        for identity, record in self.pure_pursuit_envelope_cache.items():
            if not current_generation_hold and int(identity[3]) == int(
                successor_generation
            ):
                # A valid/fresh successor envelope is only a partial cohort:
                # Plan and PP may lead the exact N+1 Constraint.  Keep the
                # immutable N lease until the full successor authority joins.
                # Invalid, unusable, stale, or identity-mutated data remains a
                # hard negative and revokes immediately.
                envelope_revoke_reason = (
                    self._delivery_gap_successor_envelope_revoke_reason(
                        lease,
                        identity=identity,
                        record=record,
                        expected_generation=int(successor_generation),
                        expected_successor_stamp_ns=successor_stamp_ns,
                        expected_plan_identity=successor_envelope_plan_identity,
                        now_sec=now_sec,
                    )
                )
                if envelope_revoke_reason is not None:
                    revoke(envelope_revoke_reason)
                    return None
        if not math.isfinite(
            float(lease.final_command.lateral.steering_tire_angle)
        ):
            revoke(DeliveryGapRevokeReason.FINAL_STEERING_NONFINITE)
            return None
        # The N lease is an immutable, already-published command cohort.  The
        # process-wide limiter state is mutable and may legitimately advance
        # on a later control tick while N+1 is still incomplete.  Comparing
        # that latest state with N mixed two generations and revoked an
        # otherwise valid N hold.  Revalidate the frozen N raw/limited/final
        # binding below; the selected and actually published commands remain
        # exact-bound by the caller before any positive grant is emitted.
        if not self._motion_authority_committed_steering_binding_valid(
            envelope_steering_rad=float(
                lease.envelope.command.lateral.steering_tire_angle
            ),
            final_steering_rad=float(
                lease.final_command.lateral.steering_tire_angle
            ),
            steering_result=lease.steering_result,
        ):
            revoke(
                DeliveryGapRevokeReason.COMMITTED_STEERING_BINDING_INVALID
            )
            return None
        envelope = copy.deepcopy(lease.envelope)
        held_command = copy.deepcopy(lease.final_command)
        if successor_constraint is not None:
            held_command = self._delivery_gap_stricter_longitudinal_command(
                held_command,
                successor_constraint[0],
            )
        proof = ControllerTrackingStatus()
        proof.header.stamp = copy.deepcopy(envelope.header.stamp)
        proof.header.frame_id = "base_link"
        proof.plan_generation = int(lease.plan_generation)
        proof.mpc_horizon_usable = bool(envelope.mpc_horizon_usable)
        proof.pp_command_fresh = bool(envelope.pp_command_fresh)
        proof.trajectory_tracking_usable = bool(
            envelope.trajectory_tracking_usable
        )
        proof.command_age_sec = float(envelope.command_age_sec)
        proof.reason = "committed_direct_successor_delivery_gap"
        return self._build_selected_pp_motion_sample(
            held_command,
            self._lease_receipt_time(
                lease, "command_receipt_time_sec"
            ),
            authority_proof=proof,
            authority_proof_receipt_time_sec=self._lease_receipt_time(
                lease, "tracking_receipt_time_sec"
            ),
            authority_proof_valid=True,
            observed_proof=proof,
            observed_proof_receipt_time_sec=self._lease_receipt_time(
                lease, "tracking_receipt_time_sec"
            ),
            observed_proof_valid=True,
            envelope_identity=self._pure_pursuit_envelope_identity(envelope),
        )

    def _commit_motion_authority_delivery_gap_lease(
        self,
        *,
        plan_identity: tuple,
        constraint: SafetyConstraintState,
        envelope: ControllerCommandEnvelope,
        final_command: AckermannControlCommand,
        now_sec: float,
        selected_sample: Optional[PurePursuitMotionSample] = None,
        steering_result: Optional[SteeringLimitResult] = None,
    ) -> None:
        """Mint once per exact commit; timer republishes never extend it."""
        previous = self.motion_authority_delivery_gap_lease
        tombstone = self.motion_authority_delivery_gap_tombstone
        envelope_identity = self._pure_pursuit_envelope_identity(envelope)
        plan_generation = int(plan_identity[7])

        # The committed final command is part of the immutable envelope
        # provenance.  A helper caller must not mint a lease for a command
        # whose payload differs from the stored atomic PP command.
        envelope_command = envelope.command
        envelope_command_values = (
            float(envelope_command.longitudinal.speed),
            float(envelope_command.longitudinal.acceleration),
            float(envelope_command.longitudinal.jerk),
            float(envelope_command.lateral.steering_tire_rotation_rate),
        )
        final_command_values = (
            float(final_command.longitudinal.speed),
            float(final_command.longitudinal.acceleration),
            float(final_command.longitudinal.jerk),
            float(final_command.lateral.steering_tire_rotation_rate),
        )
        if (
            not all(
                math.isfinite(value)
                for value in (*envelope_command_values, *final_command_values)
            )
            or envelope_command_values != final_command_values
            or not self._motion_authority_committed_steering_binding_valid(
                envelope_steering_rad=float(
                    envelope_command.lateral.steering_tire_angle
                ),
                final_steering_rad=float(
                    final_command.lateral.steering_tire_angle
                ),
                steering_result=steering_result,
            )
        ):
            self._invalidate_motion_authority_delivery_gap_lease()
            return

        def same_commit(
            record: Optional[MotionAuthorityDeliveryGapLease],
        ) -> bool:
            return bool(
                record is not None
                and tuple(record.plan_identity) == tuple(plan_identity)
                and self._pure_pursuit_envelope_identity(record.envelope)
                == envelope_identity
            )

        # A revoked/expired cohort is a tombstone, not an invitation to mint
        # another 40 ms. Only a genuinely new exact command identity replaces
        # it; an explicit race epoch reset clears it elsewhere.
        if previous is None and same_commit(tombstone):
            return
        if (
            same_commit(previous)
        ):
            if tombstone is None:
                self.motion_authority_delivery_gap_tombstone = previous
            return
        if (
            previous is not None
            and int(previous.plan_generation) == plan_generation
        ):
            # A newer PP identity from the same committed generation cannot
            # renew or replace the held N snapshot.  The timer-side gap hold
            # continues to use ``previous`` until the exact successor tuple
            # completes.
            return
        constraint_state = self._coerce_safety_constraint_state(constraint)
        plan_stamp_ns = int(plan_identity[6])
        constraint_header_stamp_ns = (
            int(constraint_state.header_stamp_ns)
        )
        plan_entry = self.overtake_plan_contract_cache.get(
            (plan_generation, plan_stamp_ns)
        )
        constraint_entry = self.safety_constraint_contract_cache.get(
            (plan_generation, constraint_header_stamp_ns)
        )
        envelope_record = self.pure_pursuit_envelope_cache.get(
            envelope_identity
        )
        tracking_receipt_time_sec = (
            selected_sample.authority_proof_receipt_time_sec
            if selected_sample is not None
            else None
        )
        if (
            selected_sample is not None
            and selected_sample.authority_proof_identity is not None
        ):
            tracking_entry = self.pure_pursuit_tracking_status_cache.get(
                selected_sample.authority_proof_identity
            )
            if tracking_entry is not None:
                # Keep the PP tracking proof's own receipt lease separate from
                # the envelope receipt that happened to construct it.
                tracking_receipt_time_sec = float(tracking_entry[1])
        if tracking_receipt_time_sec is None and selected_sample is not None:
            tracking_receipt_time_sec = selected_sample.command_receipt_time_sec
        if tracking_receipt_time_sec is None:
            tracking_receipt_time_sec = self.pure_pursuit_tracking_status_time_sec
        command_receipt_time_sec = (
            selected_sample.command_receipt_time_sec
            if selected_sample is not None
            else None
        )
        if command_receipt_time_sec is None:
            command_receipt_time_sec = self.pure_pursuit_cmd_time_sec
        committed = MotionAuthorityDeliveryGapLease(
            plan_generation=plan_generation,
            plan_identity=tuple(plan_identity),
            constraint=constraint_state,
            envelope=copy.deepcopy(envelope),
            final_command=copy.deepcopy(final_command),
            acquired_steady_time_sec=float(now_sec),
            steering_result=copy.deepcopy(steering_result),
            plan_receipt_time_sec=(
                float(plan_entry[1])
                if plan_entry is not None
                else (
                    float(self.overtake_plan_time_sec)
                    if self.overtake_plan_time_sec is not None
                    else None
                )
            ),
            constraint_receipt_time_sec=(
                float(constraint_entry[1])
                if constraint_entry is not None
                else (
                    float(self.safety_constraint_time_sec)
                    if self.safety_constraint_time_sec is not None
                    else None
                )
            ),
            tracking_receipt_time_sec=(
                float(tracking_receipt_time_sec)
                if tracking_receipt_time_sec is not None
                else None
            ),
            envelope_receipt_time_sec=(
                float(envelope_record[1])
                if envelope_record is not None
                else (
                    float(self.pure_pursuit_envelope_last_receipt_time_sec)
                    if self.pure_pursuit_envelope_last_receipt_time_sec
                    is not None
                    else None
                )
            ),
            command_receipt_time_sec=(
                float(command_receipt_time_sec)
                if command_receipt_time_sec is not None
                else None
            ),
            plan_header_stamp_ns=plan_stamp_ns,
            constraint_header_stamp_ns=constraint_header_stamp_ns,
            tracking_header_stamp_ns=(
                int(selected_sample.authority_proof_identity[0])
                if selected_sample is not None
                and selected_sample.authority_proof_identity is not None
                else None
            ),
            envelope_header_stamp_ns=self._stamp_ns(envelope.header.stamp),
            command_header_stamp_ns=self._stamp_ns(final_command.stamp),
            plan_lateral_stop_fingerprint=(
                self.overtake_plan_lateral_stop_fingerprint_cache.get(
                    plan_generation
                )
            ),
            constraint_payload_fingerprint=(
                self._motion_constraint_payload_fingerprint(constraint_state)
            ),
        )
        self.motion_authority_delivery_gap_lease = committed
        self.motion_authority_delivery_gap_tombstone = committed

    def _ratchet_motion_authority_delivery_gap_lease_after_publish(
        self,
        *,
        plan_identity: tuple,
        constraint: SafetyConstraintState,
        envelope: ControllerCommandEnvelope,
        final_command: AckermannControlCommand,
        steering_result: Optional[SteeringLimitResult],
    ) -> None:
        """Advance only the published-command snapshot of an existing lease.

        A same-generation PP command must never mint or renew delivery-gap
        time.  It may, however, become the command that was actually applied
        last.  Record that newer exact command only after publication and only
        while every immutable Plan/Constraint/candidate binding remains equal.
        """
        lease = self.motion_authority_delivery_gap_lease
        published = self.last_published_control_command
        if (
            lease is None
            or published is None
            or len(plan_identity) < 12
            or len(lease.plan_identity) < 12
            or lease.constraint_header_stamp_ns is None
            or lease.constraint_payload_fingerprint is None
            or self.last_published_control_source != "pure_pursuit"
            or tuple(plan_identity) != tuple(lease.plan_identity)
            or int(plan_identity[7]) != int(lease.plan_generation)
        ):
            return

        published_payload = self._control_command_payload(published)
        final_payload = self._control_command_payload(final_command)
        if (
            not all(
                math.isfinite(value)
                for value in (*published_payload, *final_payload)
            )
            or published_payload != final_payload
            or self._stamp_ns(published.stamp)
            != self._stamp_ns(final_command.stamp)
            or self._stamp_ns(published.longitudinal.stamp)
            != self._stamp_ns(final_command.longitudinal.stamp)
            or self._stamp_ns(published.lateral.stamp)
            != self._stamp_ns(final_command.lateral.stamp)
        ):
            return

        expected_envelope_plan_identity = (
            int(plan_identity[0]),
            int(plan_identity[1]),
            int(plan_identity[2]),
            str(plan_identity[3]),
            int(plan_identity[4]),
            int(plan_identity[5]),
            int(plan_identity[6]),
            int(plan_identity[7]),
            int(plan_identity[9]),
            bytes(plan_identity[10]),
        )
        envelope_identity = self._pure_pursuit_envelope_identity(envelope)
        previous_envelope_identity = self._pure_pursuit_envelope_identity(
            lease.envelope
        )
        if (
            self._pure_pursuit_envelope_plan_identity(envelope)
            != expected_envelope_plan_identity
            or int(envelope_identity[0]) != int(previous_envelope_identity[0])
            or int(envelope_identity[1]) <= int(previous_envelope_identity[1])
            or int(envelope_identity[2]) <= int(previous_envelope_identity[2])
            or int(envelope_identity[3]) != int(lease.plan_generation)
            or not self._motion_authority_committed_steering_binding_valid(
                envelope_steering_rad=float(
                    envelope.command.lateral.steering_tire_angle
                ),
                final_steering_rad=float(
                    final_command.lateral.steering_tire_angle
                ),
                steering_result=steering_result,
            )
        ):
            return

        constraint_state = self._coerce_safety_constraint_state(constraint)
        constraint_fingerprint = self._motion_constraint_payload_fingerprint(
            constraint_state
        )
        if (
            int(constraint_state.plan_generation) != int(lease.plan_generation)
            or int(constraint_state.header_stamp_ns)
            != int(lease.constraint_header_stamp_ns)
            or constraint_fingerprint != lease.constraint_payload_fingerprint
        ):
            return

        # ``replace`` intentionally preserves every receipt/acquisition/gap
        # deadline and successor preview.  This is a published-command
        # ownership ratchet, never a temporal lease renewal.
        ratcheted = replace(
            lease,
            constraint=constraint_state,
            envelope=copy.deepcopy(envelope),
            final_command=copy.deepcopy(final_command),
            steering_result=copy.deepcopy(steering_result),
            envelope_header_stamp_ns=self._stamp_ns(envelope.header.stamp),
            command_header_stamp_ns=self._stamp_ns(final_command.stamp),
            constraint_payload_fingerprint=constraint_fingerprint,
        )
        self.motion_authority_delivery_gap_lease = ratcheted
        self.motion_authority_delivery_gap_tombstone = ratcheted

    def _capture_motion_authority_warmup_proof(
        self,
        *,
        selected_sample: Optional[PurePursuitMotionSample],
        tracking_plan_generation: int,
        authority_plan_generation: Optional[int],
        authority_constraint: Optional[SafetyConstraintState],
        now_sec: float,
    ) -> bool:
        """Capture one exact STOP/PASS_WARMUP ACK without granting motion."""
        if (
            authority_plan_generation is None
            or int(authority_plan_generation) != int(tracking_plan_generation)
            or authority_constraint is None
            or not authority_constraint.valid
            or not authority_constraint.stop_requested
            or authority_constraint.release_authorized
        ):
            return False
        plan_identity = self.overtake_plan_motion_identity_cache.get(
            int(tracking_plan_generation)
        )
        typed_authority = self.overtake_plan_lateral_stop_authority_cache.get(
            int(tracking_plan_generation)
        )
        if plan_identity is None or typed_authority is None:
            return False
        if (
            not bool(plan_identity[11])
            or int(plan_identity[8]) != int(OvertakePlan.PASSING)
            or int(typed_authority[0])
            != int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
            or int(typed_authority[2]) <= 0
            or not bool(typed_authority[7])
        ):
            return False
        envelope = self._exact_current_motion_envelope(
            selected_sample,
            tracking_plan_generation=tracking_plan_generation,
            now_sec=now_sec,
        )
        if envelope is None or int(envelope.schema_version) != 2:
            return False
        key = envelope.plan_sample_key
        envelope_identity = (
            int(key.race_arm_epoch),
            int(key.planner_instance_id),
            int(key.attempt_id),
            str(key.target_vehicle_id),
            int(key.pass_direction),
            int(key.connector_transaction_id),
            self._stamp_ns(key.plan_stamp),
            int(key.plan_generation),
            int(envelope.candidate_revision),
            bytes(envelope.candidate_content_sha256),
        )
        expected_identity = (
            *tuple(plan_identity[:8]),
            int(plan_identity[9]),
            bytes(plan_identity[10]),
        )
        if envelope_identity != expected_identity:
            return False
        if (
            int(envelope.lateral_stop_authority_kind)
            != int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
            or int(envelope.lateral_stop_transaction_pass_direction)
            != int(typed_authority[1])
            or int(envelope.lateral_stop_authority_token)
            != int(typed_authority[2])
        ):
            return False
        self.motion_authority_warmup_proof = {
            "plan_identity": tuple(plan_identity),
            "trajectory_xy": tuple(
                self.overtake_plan_trajectory_cache.get(
                    int(tracking_plan_generation), ()
                )
            ),
            "lateral_stop_authority_token": int(typed_authority[2]),
            "pp_producer_instance_id": int(envelope.producer_instance_id),
            "pp_command_sequence": int(envelope.command_sequence),
            "pp_command_stamp": copy.deepcopy(envelope.header.stamp),
            "receipt_time_sec": float(now_sec),
            "continuity_time_sec": float(now_sec),
        }
        return True

    def _publish_motion_authority_grant(
        self,
        *,
        valid: bool,
        reason: str,
        envelope: Optional[ControllerCommandEnvelope] = None,
        plan_identity: Optional[tuple] = None,
        constraint: Optional[SafetyConstraintState] = None,
        final_command: Optional[AckermannControlCommand] = None,
    ) -> None:
        """Publish observability for the Mux-owned internal commit/revocation."""
        if not valid and not self.motion_authority_grant_active:
            return
        self.motion_authority_grant_sequence += 1
        grant = MotionAuthorityGrant()
        grant.schema_version = 1
        grant.valid = bool(valid)
        grant.reason = str(reason)[:96]
        grant.grant_issuer_instance_id = int(
            self.motion_authority_grant_issuer_instance_id
        )
        grant.grant_sequence = int(self.motion_authority_grant_sequence)
        grant.lease_duration_sec = float(
            self.motion_authority_grant_lease_sec if valid else 0.0
        )
        warmup_proof = self.motion_authority_warmup_proof
        if warmup_proof is not None:
            warmup_identity = tuple(warmup_proof["plan_identity"])
            warmup_key = grant.warmup_plan_sample_key
            warmup_key.race_arm_epoch = int(warmup_identity[0])
            warmup_key.planner_instance_id = int(warmup_identity[1])
            warmup_key.attempt_id = int(warmup_identity[2])
            warmup_key.target_vehicle_id = str(warmup_identity[3])
            warmup_key.pass_direction = int(warmup_identity[4])
            warmup_key.connector_transaction_id = int(warmup_identity[5])
            warmup_stamp_ns = int(warmup_identity[6])
            warmup_key.plan_stamp.sec = warmup_stamp_ns // 1_000_000_000
            warmup_key.plan_stamp.nanosec = warmup_stamp_ns % 1_000_000_000
            warmup_key.plan_generation = int(warmup_identity[7])
            grant.warmup_candidate_revision = int(warmup_identity[9])
            grant.warmup_candidate_content_sha256 = [
                int(value) for value in bytes(warmup_identity[10])
            ]
            grant.warmup_lateral_stop_authority_token = int(
                warmup_proof["lateral_stop_authority_token"]
            )
            grant.warmup_pp_producer_instance_id = int(
                warmup_proof["pp_producer_instance_id"]
            )
            grant.warmup_pp_command_sequence = int(
                warmup_proof["pp_command_sequence"]
            )
            grant.warmup_pp_command_stamp = copy.deepcopy(
                warmup_proof["pp_command_stamp"]
            )
        if envelope is not None:
            grant.header = copy.deepcopy(envelope.header)
            grant.plan_sample_key = copy.deepcopy(envelope.plan_sample_key)
            grant.candidate_revision = int(envelope.candidate_revision)
            grant.candidate_content_sha256 = list(
                int(value)
                for value in envelope.candidate_content_sha256
            )
            grant.pp_producer_instance_id = int(
                envelope.producer_instance_id
            )
            grant.pp_command_sequence = int(envelope.command_sequence)
            grant.pp_command_stamp = copy.deepcopy(envelope.header.stamp)
            committed_command = (
                final_command if final_command is not None else envelope.command
            )
            grant.header.stamp = copy.deepcopy(committed_command.stamp)
            grant.speed_mps = float(committed_command.longitudinal.speed)
            grant.acceleration_mps2 = float(
                committed_command.longitudinal.acceleration
            )
            grant.steering_tire_angle_rad = float(
                committed_command.lateral.steering_tire_angle
            )
            grant.steering_tire_rotation_rate_radps = float(
                committed_command.lateral.steering_tire_rotation_rate
            )
        if plan_identity is not None:
            grant.phase = int(plan_identity[8])
        if constraint is not None:
            grant.constraint_generation = int(
                constraint.constraint_generation
            )
            constraint_stamp_ns = int(constraint.header_stamp_ns)
            grant.constraint_stamp.sec = constraint_stamp_ns // 1_000_000_000
            grant.constraint_stamp.nanosec = (
                constraint_stamp_ns % 1_000_000_000
            )
        self.motion_authority_grant_pub.publish(grant)
        self.motion_authority_grant_active = bool(valid)

    def on_awsim_state(self, msg: String) -> None:
        if self.finish_stop_enabled:
            was_latched = self.finish_stop_latch.latched
            self.finish_stop_latch.observe_vehicle_state(str(msg.data))
            if self.finish_stop_latch.latched and not was_latched:
                self.motion_authority_warmup_proof = None
                self._invalidate_motion_authority_delivery_gap_lease()
                self._invalidate_verified_stop_steering()
                self._snapshot_finish_terminal_reference()
                self._invalidate_free_run_live_exact_record("finish_latched")

    def on_safety_constraint(self, msg: SafetyConstraint) -> None:
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        constraint = SafetyConstraintState(
            constraint_generation=int(msg.constraint_generation),
            plan_generation=int(msg.plan_generation),
            valid=bool(msg.valid),
            stop_requested=bool(msg.stop_requested),
            release_authorized=bool(msg.release_authorized),
            speed_limit_mps=float(msg.speed_limit_mps),
            required_brake_decel_mps2=float(msg.required_brake_decel_mps2),
            header_stamp_ns=header_stamp_ns,
            frame_id=str(msg.header.frame_id),
            reason=str(msg.reason),
        )
        constraint_contract_key = (
            int(constraint.plan_generation),
            int(header_stamp_ns),
        )
        cached_contract_entry = self.safety_constraint_contract_cache.get(
            constraint_contract_key
        )
        same_stamp_payload_conflict = bool(
            cached_contract_entry is not None
            and self._motion_constraint_payload_fingerprint(
                cached_contract_entry[0]
            )
            != self._motion_constraint_payload_fingerprint(constraint)
        )
        if same_stamp_payload_conflict:
            # Same identity with a different payload is a tainted contract;
            # never leave the original cached release available for a later
            # exact PP switch.
            self.safety_constraint_contract_cache.pop(
                constraint_contract_key, None
            )
            cached_latest_entry = self.safety_constraint_cache.get(
                int(constraint.plan_generation)
            )
            if (
                cached_latest_entry is not None
                and int(cached_latest_entry[0].header_stamp_ns)
                == int(header_stamp_ns)
            ):
                self.safety_constraint_cache.pop(
                    int(constraint.plan_generation), None
                )
            self._invalidate_motion_authority_delivery_gap_lease(
                stop_pending=True,
                reason=DeliveryGapRevokeReason.N1_CONSTRAINT_PAYLOAD_MUTATED,
                origin="constraint_callback",
                observed_generation=int(constraint.plan_generation),
                now_sec=self.now_sec(),
            )
        previous_constraint_header_stamp_ns = (
            self.safety_constraint_header_stamp_ns
        )
        self.safety_constraint = constraint
        self.safety_constraint_timestamp_regressed = bool(
            same_stamp_payload_conflict
            or (
                previous_constraint_header_stamp_ns is not None
                and header_stamp_ns < previous_constraint_header_stamp_ns
            )
            or (
                self.safety_constraint_timestamp_regressed
                and previous_constraint_header_stamp_ns is not None
                and header_stamp_ns <= previous_constraint_header_stamp_ns
            )
        )
        (
            receipt_time_sec,
            maximum_stamp_ns,
            timestamp_advanced,
        ) = self._advance_receipt_time(
            self.safety_constraint_time_sec,
            self.safety_constraint_header_stamp_ns,
            header_stamp_ns,
        )
        self.safety_constraint_time_sec = receipt_time_sec
        self.safety_constraint_header_stamp_ns = maximum_stamp_ns
        delivery_gap_lease = self.motion_authority_delivery_gap_lease
        if delivery_gap_lease is not None:
            committed_constraint_state = self._coerce_safety_constraint_state(
                delivery_gap_lease.constraint
            )
            committed_constraint_stamp_ns = (
                int(delivery_gap_lease.constraint_header_stamp_ns)
                if delivery_gap_lease.constraint_header_stamp_ns is not None
                else int(committed_constraint_state.header_stamp_ns)
            )
            committed_constraint_fingerprint = (
                delivery_gap_lease.constraint_payload_fingerprint
                if delivery_gap_lease.constraint_payload_fingerprint is not None
                else self._motion_constraint_payload_fingerprint(
                    committed_constraint_state
                )
            )
            if (
                int(constraint.plan_generation)
                == int(delivery_gap_lease.plan_generation)
                and int(header_stamp_ns) == committed_constraint_stamp_ns
                and self._motion_constraint_payload_fingerprint(constraint)
                != committed_constraint_fingerprint
            ):
                self._invalidate_motion_authority_delivery_gap_lease(
                    stop_pending=True,
                    reason=DeliveryGapRevokeReason.N_CONSTRAINT_PAYLOAD_MUTATED,
                    origin="constraint_callback",
                    observed_generation=int(constraint.plan_generation),
                    now_sec=self.now_sec(),
                )
                delivery_gap_lease = None
        if delivery_gap_lease is not None:
            direct_successor = (
                1
                if int(delivery_gap_lease.plan_generation) >= 16_777_215
                else int(delivery_gap_lease.plan_generation) + 1
            )
            constraint_revoke_reason = None
            if int(constraint.plan_generation) in (
                int(delivery_gap_lease.plan_generation),
                direct_successor,
            ):
                constraint_revoke_reason = (
                    self._motion_release_constraint_revoke_reason(
                        constraint,
                        receipt_time_sec=receipt_time_sec,
                        now_sec=self.now_sec(),
                        expected_generation=int(constraint.plan_generation),
                        expected_stamp_ns=header_stamp_ns,
                    )
                )
            if constraint_revoke_reason is not None:
                self._invalidate_motion_authority_delivery_gap_lease(
                    stop_pending=True,
                    reason=constraint_revoke_reason,
                    origin="constraint_callback",
                    observed_generation=int(constraint.plan_generation),
                    now_sec=self.now_sec(),
                )
                delivery_gap_lease = None
        if (
            delivery_gap_lease is not None
            and int(constraint.plan_generation) == direct_successor
        ):
            cached_successor_identity = (
                self.overtake_plan_motion_identity_cache.get(
                    direct_successor
                )
            )
            if cached_successor_identity is not None and (
                not bool(cached_successor_identity[11])
                or int(cached_successor_identity[6]) != header_stamp_ns
            ):
                self._invalidate_motion_authority_delivery_gap_lease(
                    stop_pending=True,
                    reason=DeliveryGapRevokeReason.N1_CONSTRAINT_STAMP_MISMATCH,
                    origin="constraint_callback",
                    observed_generation=int(constraint.plan_generation),
                    now_sec=self.now_sec(),
                )
                delivery_gap_lease = None
        if (
            delivery_gap_lease is not None
            and int(constraint.plan_generation) == direct_successor
        ):
            for identity, record in self.pure_pursuit_envelope_cache.items():
                if int(identity[3]) != direct_successor:
                    continue
                envelope = record[4]
                if not self._motion_authority_successor_envelope_relation_valid(
                    delivery_gap_lease,
                    envelope,
                    expected_successor_stamp_ns=header_stamp_ns,
                ):
                    # Constraint and PP are independently delivered. Once both
                    # are observed, their known Plan stamp/identity relation is
                    # authoritative even while Plan N+1 itself is still absent.
                    self._invalidate_motion_authority_delivery_gap_lease(
                        stop_pending=True,
                        reason=DeliveryGapRevokeReason.N1_ENVELOPE_RELATION_INVALID,
                        origin="constraint_callback",
                        observed_generation=int(constraint.plan_generation),
                        now_sec=self.now_sec(),
                    )
                    delivery_gap_lease = None
                    break
        if (
            not constraint.valid
            or constraint.stop_requested
            or self.safety_constraint_timestamp_regressed
        ):
            barrier_reason = DeliveryGapRevokeReason.N1_CONSTRAINT_INVALID
            if constraint.stop_requested:
                barrier_reason = (
                    DeliveryGapRevokeReason.N1_CONSTRAINT_STOP_REQUESTED
                )
            elif self.safety_constraint_timestamp_regressed:
                barrier_reason = DeliveryGapRevokeReason.N1_CONSTRAINT_STAMP_MISMATCH
            self._invalidate_motion_authority_delivery_gap_lease(
                stop_pending=True,
                reason=barrier_reason,
                origin="constraint_callback",
                observed_generation=int(constraint.plan_generation),
                now_sec=self.now_sec(),
            )
            self._invalidate_free_run_live_exact_record(
                "safety_constraint_barrier"
            )
        if timestamp_advanced and receipt_time_sec is not None:
            self._remember_bounded(
                self.safety_constraint_cache,
                constraint.plan_generation,
                (constraint, receipt_time_sec),
            )
            self._remember_bounded(
                self.safety_constraint_contract_cache,
                (constraint.plan_generation, header_stamp_ns),
                (constraint, receipt_time_sec),
            )
        if self.free_run_live_exact_observe_enabled:
            self._refresh_free_run_live_exact_observation()

    def on_overtake_plan(self, msg: OvertakePlan) -> None:
        header_stamp_ns = self._stamp_ns(msg.header.stamp)
        delivery_gap_lease = self.motion_authority_delivery_gap_lease
        if (
            delivery_gap_lease is not None
            and int(msg.plan_generation) == int(delivery_gap_lease.plan_generation)
        ):
            incoming_identity = (
                int(msg.race_arm_epoch),
                int(msg.planner_instance_id),
                int(msg.attempt_id),
                str(msg.target_vehicle_id),
                int(msg.pass_direction),
                int(msg.connector_transaction_id),
                int(header_stamp_ns),
                int(msg.plan_generation),
                int(msg.phase),
                int(msg.candidate_revision),
                bytes(msg.candidate_content_sha256),
                bool(msg.aw2_identity_schema_version == 1),
            )
            committed_fingerprint = (
                delivery_gap_lease.plan_lateral_stop_fingerprint
                or self.overtake_plan_lateral_stop_fingerprint_cache.get(
                    int(msg.plan_generation)
                )
            )
            incoming_fingerprint = self._overtake_plan_lateral_stop_fingerprint(
                msg
            )
            if (
                tuple(incoming_identity) != tuple(delivery_gap_lease.plan_identity)
                or (
                    committed_fingerprint is not None
                    and incoming_fingerprint != committed_fingerprint
                )
                or str(msg.header.frame_id) != "map"
                or str(msg.trajectory.header.frame_id)
                != str(msg.header.frame_id)
                or self._stamp_ns(msg.trajectory.header.stamp)
                != header_stamp_ns
            ):
                # This check is intentionally before the normal monotonic
                # callback gate: a same-generation mutation arriving with an
                # older stamp must still revoke committed N immediately.
                self._invalidate_motion_authority_delivery_gap_lease(
                    stop_pending=True,
                    reason=DeliveryGapRevokeReason.N_PLAN_IDENTITY_CHANGED,
                    origin="plan_callback",
                    observed_generation=int(msg.plan_generation),
                    now_sec=self.now_sec(),
                )
        if self.free_run_live_exact_observe_enabled:
            computed_free_run_digest = self._canonical_free_run_plan_digest(msg)
            claimed_free_run_digest = bytes(
                msg.free_run_canonical_payload_sha256
            )
            free_run_digest_valid = bool(
                computed_free_run_digest is not None
                and int(msg.free_run_canonical_algorithm_version)
                == int(OvertakePlan.FREE_RUN_CANONICAL_ALGORITHM_V1)
                and claimed_free_run_digest == computed_free_run_digest
            )
            self._remember_bounded(
                self.free_run_live_exact_plan_delivery_cache,
                (int(msg.plan_generation), header_stamp_ns),
                (
                    free_run_digest_valid,
                    claimed_free_run_digest,
                    int(msg.free_run_canonical_algorithm_version),
                ),
            )
            if computed_free_run_digest is not None:
                generation_key = (
                    int(msg.race_arm_epoch),
                    int(msg.planner_instance_id),
                    int(msg.plan_generation),
                )
                previous_digest = (
                    self.free_run_live_exact_plan_digest_cache.get(
                        generation_key
                    )
                )
                if (
                    previous_digest is not None
                    and previous_digest != computed_free_run_digest
                ):
                    self.free_run_live_exact_tainted_generations.add(
                        generation_key
                    )
                self._remember_bounded(
                    self.free_run_live_exact_plan_digest_cache,
                    generation_key,
                    computed_free_run_digest,
                )
            record = self.free_run_live_exact_published_record
            if record is not None:
                pristine_same_semantic_generation = bool(
                    computed_free_run_digest is not None
                    and free_run_digest_valid
                    and int(msg.race_arm_epoch) == int(record.race_arm_epoch)
                    and int(msg.planner_instance_id)
                    == int(record.planner_instance_id)
                    and int(msg.plan_generation)
                    in (
                        int(record.plan_generation),
                        int(record.plan_generation) + 1,
                    )
                    and bytes(computed_free_run_digest)
                    == bytes(record.canonical_plan_sha256)
                    and header_stamp_ns >= int(record.plan_stamp_ns)
                )
                if not pristine_same_semantic_generation:
                    self._invalidate_free_run_live_exact_record(
                        "plan_semantic_identity_changed"
                    )
        previous_generation = self.overtake_plan_generation
        timestamp_monotonic = (
            self.overtake_plan_header_stamp_ns is None
            or header_stamp_ns >= self.overtake_plan_header_stamp_ns
        )
        self.overtake_plan_generation = int(msg.plan_generation)
        if self.overtake_plan_generation != previous_generation:
            self._promote_pure_pursuit_envelope_watermark()
        (
            self.overtake_plan_time_sec,
            advanced_stamp_ns,
            _,
        ) = self._advance_receipt_time(
            self.overtake_plan_time_sec,
            self.overtake_plan_header_stamp_ns,
            header_stamp_ns,
        )
        trajectory_header_matches = (
            str(msg.trajectory.header.frame_id) == str(msg.header.frame_id)
            and self._stamp_ns(msg.trajectory.header.stamp) == header_stamp_ns
        )
        trajectory_points_valid = bool(msg.trajectory.points) and all(
            math.isfinite(float(point.pose.position.x))
            and math.isfinite(float(point.pose.position.y))
            and math.isfinite(float(point.pose.position.z))
            and math.isfinite(float(point.pose.orientation.x))
            and math.isfinite(float(point.pose.orientation.y))
            and math.isfinite(float(point.pose.orientation.z))
            and math.isfinite(float(point.pose.orientation.w))
            and math.isfinite(float(point.longitudinal_velocity_mps))
            and math.isfinite(float(point.lateral_velocity_mps))
            and math.isfinite(float(point.acceleration_mps2))
            and math.isfinite(float(point.heading_rate_rps))
            and math.isfinite(float(point.front_wheel_angle_rad))
            and math.isfinite(float(point.rear_wheel_angle_rad))
            and float(point.longitudinal_velocity_mps) >= 0.0
            for point in msg.trajectory.points
        )
        trajectory_contract_required = bool(msg.trajectory_authorized) or bool(
            msg.lateral_maneuver_required
        )
        bootstrap_current_d_stop_hold = bool(
            int(msg.phase) == int(OvertakePlan.ATTACK_FOLLOW)
            and int(msg.pass_direction) == 0
            and bool(msg.lateral_maneuver_required)
            and not bool(msg.trajectory_authorized)
            and not bool(msg.trajectory.points)
            and bool(str(msg.target_vehicle_id).strip())
        )
        trajectory_contract_valid = (
            not trajectory_contract_required
            or (
                bool(msg.trajectory_authorized)
                and trajectory_header_matches
                and trajectory_points_valid
            )
            or bootstrap_current_d_stop_hold
        )
        self.overtake_plan_valid = (
            str(msg.header.frame_id) == "map"
            and header_stamp_ns >= 0
            and timestamp_monotonic
            and int(msg.phase)
            in (
                int(OvertakePlan.FREE_RUN),
                int(OvertakePlan.ATTACK_FOLLOW),
                int(OvertakePlan.PASSING),
                int(OvertakePlan.ABORT_HOLD),
            )
            and trajectory_contract_valid
        )
        delivery_gap_lease = self.motion_authority_delivery_gap_lease
        if delivery_gap_lease is not None:
            direct_successor = (
                1
                if int(delivery_gap_lease.plan_generation) >= 16_777_215
                else int(delivery_gap_lease.plan_generation) + 1
            )
            incoming_generation = int(msg.plan_generation)
            successor_plan_revoke_reason = None
            if incoming_generation not in (
                int(delivery_gap_lease.plan_generation), direct_successor
            ):
                successor_plan_revoke_reason = (
                    DeliveryGapRevokeReason.SUCCESSOR_NOT_DIRECT
                )
            elif incoming_generation == direct_successor:
                successor_plan_revoke_reason = (
                    self._delivery_gap_successor_plan_revoke_reason(
                        delivery_gap_lease,
                        msg,
                        plan_valid=self.overtake_plan_valid,
                    )
                )
            if successor_plan_revoke_reason is not None:
                # A callback hard negative must survive any valid replacement
                # delivered before the next timer. Reuse the existing one-shot
                # STOP marker; do not add a second authority or recovery state.
                self._invalidate_motion_authority_delivery_gap_lease(
                    stop_pending=True,
                    reason=successor_plan_revoke_reason,
                    origin="plan_callback",
                    observed_generation=incoming_generation,
                    now_sec=self.now_sec(),
                    plan_relation_subreason=(
                        self._delivery_gap_successor_plan_relation_subreason(
                            delivery_gap_lease, msg
                        )
                        if successor_plan_revoke_reason
                        == DeliveryGapRevokeReason.N1_PLAN_RELATION_INVALID
                        else None
                    ),
                )
            elif incoming_generation == direct_successor:
                # First valid N+1 Plan freezes the successor's own identity.
                # This does not alter committed N, refresh any N receipt, move
                # the gap clock, or grant N+1 authority. Later N+1 transport
                # members must bind exactly to these new-geometry bytes.
                successor_identity = (
                    int(msg.race_arm_epoch),
                    int(msg.planner_instance_id),
                    int(msg.attempt_id),
                    str(msg.target_vehicle_id),
                    int(msg.pass_direction),
                    int(msg.connector_transaction_id),
                    int(header_stamp_ns),
                    int(msg.plan_generation),
                    int(msg.phase),
                    int(msg.candidate_revision),
                    bytes(msg.candidate_content_sha256),
                    bool(msg.aw2_identity_schema_version == 1),
                )
                current_lease = self.motion_authority_delivery_gap_lease
                if current_lease is delivery_gap_lease:
                    self.motion_authority_delivery_gap_lease = replace(
                        delivery_gap_lease,
                        successor_plan_identity=successor_identity,
                        successor_plan_lateral_stop_fingerprint=(
                            self._overtake_plan_lateral_stop_fingerprint(msg)
                        ),
                    )
        self.overtake_plan_trajectory_authorized = bool(
            msg.trajectory_authorized
        )
        self.overtake_plan_lateral_maneuver_required = bool(
            msg.lateral_maneuver_required
        )
        if timestamp_monotonic:
            self.overtake_plan_header_stamp_ns = advanced_stamp_ns
            if self.overtake_plan_time_sec is not None:
                plan_entry = (
                    self.overtake_plan_valid,
                    self.overtake_plan_time_sec,
                    header_stamp_ns,
                    self.overtake_plan_trajectory_authorized,
                    self.overtake_plan_lateral_maneuver_required,
                    int(msg.phase),
                    int(msg.pass_direction),
                    str(msg.target_vehicle_id),
                )
                self._remember_bounded(
                    self.overtake_plan_cache,
                    self.overtake_plan_generation,
                    plan_entry,
                )
                self._remember_bounded(
                    self.overtake_plan_attempt_cache,
                    self.overtake_plan_generation,
                    int(msg.attempt_id),
                )
                motion_identity = (
                    int(msg.race_arm_epoch),
                    int(msg.planner_instance_id),
                    int(msg.attempt_id),
                    str(msg.target_vehicle_id),
                    int(msg.pass_direction),
                    int(msg.connector_transaction_id),
                    int(header_stamp_ns),
                    int(msg.plan_generation),
                    int(msg.phase),
                    int(msg.candidate_revision),
                    bytes(msg.candidate_content_sha256),
                    bool(msg.aw2_identity_schema_version == 1),
                )
                previous_motion_identity = (
                    self.overtake_plan_motion_identity_cache.get(
                        self.overtake_plan_generation
                    )
                )
                if (
                    previous_motion_identity is not None
                    and previous_motion_identity[:-1]
                    != motion_identity[:-1]
                ):
                    motion_identity = (*motion_identity[:-1], False)
                self._remember_bounded(
                    self.overtake_plan_motion_identity_cache,
                    self.overtake_plan_generation,
                    motion_identity,
                )
                trajectory_xy = tuple(
                    (
                        float(point.pose.position.x),
                        float(point.pose.position.y),
                    )
                    for point in msg.trajectory.points
                )
                lateral_stop_fingerprint = (
                    self._overtake_plan_lateral_stop_fingerprint(msg)
                )
                previous_trajectory_xy = self.overtake_plan_trajectory_cache.get(
                    self.overtake_plan_generation
                )
                previous_lateral_stop_fingerprint = (
                    self.overtake_plan_lateral_stop_fingerprint_cache.get(
                        self.overtake_plan_generation
                    )
                )
                if (
                    previous_lateral_stop_fingerprint is not None
                    and previous_lateral_stop_fingerprint
                    != lateral_stop_fingerprint
                ):
                    # A mutated generation must remain unusable for the whole
                    # race epoch. Unlike rendezvous payload caches, this set
                    # must never evict an older taint due to traffic volume.
                    self.overtake_plan_lateral_stop_taint_cache[
                        self.overtake_plan_generation
                    ] = True
                delivery_gap_lease = (
                    self.motion_authority_delivery_gap_lease
                )
                if (
                    delivery_gap_lease is not None
                    and int(delivery_gap_lease.plan_generation)
                    == int(self.overtake_plan_generation)
                    and (
                        not self.overtake_plan_valid
                        or
                        tuple(motion_identity)
                        != tuple(delivery_gap_lease.plan_identity)
                        or bool(
                            self.overtake_plan_lateral_stop_taint_cache.get(
                                self.overtake_plan_generation, False
                            )
                        )
                    )
                ):
                    # The committed N identity is immutable. A same-
                    # generation plan/trajectory mutation is a hard negative,
                    # even if the callback is otherwise structurally valid.
                    self._invalidate_motion_authority_delivery_gap_lease(
                        stop_pending=True,
                        reason=DeliveryGapRevokeReason.N_PLAN_IDENTITY_CHANGED,
                        origin="plan_callback",
                        observed_generation=int(self.overtake_plan_generation),
                        now_sec=self.now_sec(),
                    )
                self._remember_bounded(
                    self.overtake_plan_trajectory_cache,
                    self.overtake_plan_generation,
                    trajectory_xy,
                )
                self._remember_bounded(
                    self.overtake_plan_lateral_stop_fingerprint_cache,
                    self.overtake_plan_generation,
                    lateral_stop_fingerprint,
                )
                typed_authority = (
                    int(msg.lateral_stop_authority_kind),
                    int(msg.lateral_stop_transaction_pass_direction),
                    int(msg.lateral_stop_authority_token),
                    int(msg.attempt_id),
                    str(msg.target_vehicle_id),
                    int(msg.phase),
                    int(msg.pass_direction),
                    True,
                )
                previous_typed_authority = (
                    self.overtake_plan_lateral_stop_authority_cache.get(
                        self.overtake_plan_generation
                    )
                )
                if (
                    previous_typed_authority is not None
                    and previous_typed_authority[:-1]
                    != typed_authority[:-1]
                ) or (
                    previous_trajectory_xy is not None
                    and previous_trajectory_xy != trajectory_xy
                ) or bool(
                    self.overtake_plan_lateral_stop_taint_cache.get(
                        self.overtake_plan_generation, False
                    )
                ):
                    typed_authority = (*typed_authority[:-1], False)
                self._remember_bounded(
                    self.overtake_plan_lateral_stop_authority_cache,
                    self.overtake_plan_generation,
                    typed_authority,
                )
                self._remember_bounded(
                    self.overtake_plan_contract_cache,
                    (self.overtake_plan_generation, header_stamp_ns),
                    plan_entry,
                )
        if self.free_run_live_exact_observe_enabled:
            self._refresh_free_run_live_exact_observation()

    def on_mpc_health(self, msg: String) -> None:
        now_sec = self.now_sec()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.mpc_health = MpcHealth(valid=False, status="parse_error", age_sec=0.0)
            self.mpc_health_time_sec = now_sec
            return

        self.mpc_health = MpcHealth(
            valid=True,
            status=str(payload.get("mpc_status", "unknown")),
            infeasible_count=int(payload.get("mpc_infeasible_count", 0)),
            age_sec=0.0,
        )
        self.mpc_health_time_sec = now_sec

    def on_timer(self) -> None:
        """Run one control cycle, optionally recording test-only steady WCET."""
        if not (self.mux_runtime_measurement_enabled or self.latest_sample_observability_enabled):
            self._on_timer_impl()
            return
        started_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if self.mux_runtime_measurement_enabled:
            self.mux_runtime_measurement_pending_cycle = None
        if self.latest_sample_observability_enabled:
            self._mux_latest_sample_cycle_context = None
        if self.latest_sample_observability_enabled and self.mux_runtime_measurement_capture_closed:
            self.mux_runtime_measurement_capture_late_entry = True
            self.mux_runtime_measurement_faults = min((1 << 64) - 1, self.mux_runtime_measurement_faults + 1)
        if self.latest_sample_observability_enabled:
            self.mux_runtime_measurement_cycle_in_flight += 1
        callback_completed = False
        try:
            self._on_timer_impl()
            callback_completed = True
        finally:
            finished_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
            if self.mux_runtime_measurement_enabled:
                self.mux_runtime_measurement_sequence += 1
                legacy_sequence = self.mux_runtime_measurement_sequence
                legacy_index = self.mux_runtime_measurement_legacy_cycle_count
                if legacy_index >= self.mux_runtime_measurement_capacity:
                    self.mux_runtime_measurement_drops = min(
                        (1 << 64) - 1,
                        self.mux_runtime_measurement_drops + 1,
                    )
                else:
                    cycle = dict(self.mux_runtime_measurement_pending_cycle or {})
                    cycle["cycle_sequence"] = legacy_sequence
                    cycle["started_steady_ns"] = started_ns
                    cycle["duration_ns"] = max(0, finished_ns - started_ns)
                    cycle["callback_completed"] = callback_completed
                    self.mux_runtime_measurement_wcet_slots[legacy_index] = (
                        legacy_sequence,
                        started_ns,
                        max(0, finished_ns - started_ns),
                    )
                    self.mux_runtime_measurement_cycle_records[legacy_index] = cycle
                    self.mux_runtime_measurement_legacy_cycle_count += 1
            if self.latest_sample_observability_enabled:
                self.mux_runtime_measurement_cycle_in_flight -= 1
                self.mux_runtime_measurement_cycle_completions += 1
                self.mux_latest_sample_cycle_sequence += 1
                sequence = self.mux_latest_sample_cycle_sequence
                if not callback_completed:
                    self.mux_runtime_measurement_faults = min(
                        (1 << 64) - 1,
                        self.mux_runtime_measurement_faults + 1,
                    )
                if self.mux_runtime_measurement_cycle_count >= self.mux_runtime_measurement_cycle_capacity:
                    self.mux_runtime_measurement_cycle_drops = min((1 << 64) - 1, self.mux_runtime_measurement_cycle_drops + 1)
                    self.mux_runtime_measurement_overflow = True
                    self.mux_runtime_measurement_cycle_overflow = True
                else:
                    latest = self._mux_latest_sample_cycle_context or (
                        None, None, "nonqualifying", "unknown", "unknown", False, False
                    )
                    self.mux_latest_sample_cycle_slots[self.mux_runtime_measurement_cycle_count] = (
                        sequence,
                        started_ns,
                        *latest,
                    )
                    self.mux_runtime_measurement_cycle_count += 1

    def _on_timer_impl(self) -> None:
        now_sec = self.now_sec()
        measurement_steady_offset_ns = (
            time.clock_gettime_ns(time.CLOCK_MONOTONIC)
            - int(round(now_sec * 1.0e9))
            if self.mux_runtime_measurement_enabled
            else 0
        )
        if self.free_run_live_exact_observe_enabled:
            # Authority promotion must never consume a callback-stale boolean.
            # Keep the observer's lease fail-closed even when all inputs stop.
            self._refresh_free_run_live_exact_observation()
        watchdog = self.control_loop_watchdog.update(now_sec)
        observed_ros_clock_watchdog = self.ros_clock_watchdog.update(
            int(self.get_clock().now().nanoseconds), now_sec
        )
        self.ros_clock_observation_reason = observed_ros_clock_watchdog.reason
        if (
            self.race_arm_required
            and self.finish_stop_latch.armed
            and not self.ros_clock_motion_ready
            and observed_ros_clock_watchdog.reason == "ros_clock_progressing"
        ):
            self.ros_clock_motion_ready = True
        ros_clock_fault_operational = bool(
            (
                not self.race_arm_required
                or (
                    self.finish_stop_latch.armed
                    and self.ros_clock_motion_ready
                )
            )
            and observed_ros_clock_watchdog.stalled
        )
        ros_clock_watchdog = RosClockProgressWatchdogResult(
            stalled=ros_clock_fault_operational,
            stagnant_duration_sec=(
                observed_ros_clock_watchdog.stagnant_duration_sec
            ),
            reason=observed_ros_clock_watchdog.reason,
        )
        plan_fresh = self._fresh(
            self.overtake_plan_time_sec, now_sec, self.overtake_plan_timeout_sec
        )
        tracking_plan_generation = (
            self.overtake_plan_generation
            if plan_fresh
            and self.overtake_plan_valid
            and self.overtake_plan_generation is not None
            else -1
        )
        safety_authority_selection = self._select_safety_authority_contract(
            now_sec,
            tracking_plan_generation=tracking_plan_generation,
        )
        if self.safety_authority_contract_unpaired:
            self.get_logger().warning(
                "delivery_gap_diagnostic "
                f"code=DG{int(DeliveryGapRevokeReason.PLAN_CONSTRAINT_UNPAIRED):03d} "
                f"name={DeliveryGapRevokeReason.PLAN_CONSTRAINT_UNPAIRED.name} "
                "origin=rendezvous "
                f"tracking={tracking_plan_generation} "
                f"authority={safety_authority_selection.authority_plan_generation} "
                f"state={safety_authority_selection.rendezvous_state.name}"
            )
        authority_constraint = safety_authority_selection.constraint
        authority_constraint_time_sec = (
            safety_authority_selection.receipt_time_sec
        )
        authority_plan_generation = (
            safety_authority_selection.authority_plan_generation
        )
        active_control_fault_reason = ""
        if watchdog.deadline_missed:
            active_control_fault_reason = "control_loop_deadline_missed"
        elif ros_clock_watchdog.stalled:
            active_control_fault_reason = ros_clock_watchdog.reason

        # Validate a direct successor delivery gap before touching the shared
        # authority latch.  If this is the N+1 preview, evaluate committed N
        # below; the preview itself is pure and cannot advance the latch.
        lease_sample_candidate: Optional[PurePursuitMotionSample] = None
        lease_current_generation_hold = False
        lease_preview_attempted = False
        motion_authority_delivery_gap_lease_revoke_stop = bool(
            self.motion_authority_delivery_gap_stop_pending
        )
        # Callback-side hard negatives are consumed by exactly one timer. A
        # valid replacement arriving before this callback must not erase the
        # pending STOP, while a later complete cohort follows normal authority
        # arbitration after the marker has been cleared.
        self.motion_authority_delivery_gap_stop_pending = False
        if motion_authority_delivery_gap_lease_revoke_stop:
            self.motion_authority_delivery_gap_revoke_pending = None
        delivery_gap_lease = self.motion_authority_delivery_gap_lease
        if (
            delivery_gap_lease is not None
            and delivery_gap_lease.successor_envelope_preview_identity is not None
            and delivery_gap_lease.gap_started_steady_time_sec is not None
        ):
            preview_age_sec = now_sec - float(
                delivery_gap_lease.gap_started_steady_time_sec
            )
            if (
                not math.isfinite(preview_age_sec)
                or preview_age_sec < 0.0
                or preview_age_sec
                > self._motion_authority_delivery_gap_lease_timeout_sec()
            ):
                self._invalidate_motion_authority_delivery_gap_lease(
                    stop_pending=True,
                    reason=(
                        DeliveryGapRevokeReason.GAP_CLOCK_INVALID
                        if not math.isfinite(preview_age_sec)
                        or preview_age_sec < 0.0
                        else DeliveryGapRevokeReason.GAP_TIMEOUT
                    ),
                    origin="timer_preview",
                    observed_generation=int(tracking_plan_generation),
                    now_sec=now_sec,
                )
                motion_authority_delivery_gap_lease_revoke_stop = True
                delivery_gap_lease = None
        if (
            delivery_gap_lease is not None
            and tracking_plan_generation > 0
            and int(tracking_plan_generation)
            == int(delivery_gap_lease.plan_generation)
            and self.overtake_plan_generation is not None
            and int(self.overtake_plan_generation)
            == int(delivery_gap_lease.plan_generation)
            and self.safety_constraint is not None
            and int(self.safety_constraint.plan_generation)
            == (
                1
                if int(delivery_gap_lease.plan_generation) >= 16_777_215
                else int(delivery_gap_lease.plan_generation) + 1
            )
        ):
            lease_current_generation_hold = True
            lease_preview_attempted = True
            lease_sample_candidate = (
                self._direct_motion_authority_successor_lease_sample(
                    successor_generation=int(tracking_plan_generation),
                    now_sec=now_sec,
                    authority_selection=safety_authority_selection,
                    ros_clock_stalled=ros_clock_watchdog.stalled,
                    active_control_fault_reason=active_control_fault_reason,
                    deadline_missed=watchdog.deadline_missed,
                )
            )
        elif (
            delivery_gap_lease is not None
            and tracking_plan_generation > 0
            and int(tracking_plan_generation)
            == (
                1
                if int(delivery_gap_lease.plan_generation) >= 16_777_215
                else int(delivery_gap_lease.plan_generation) + 1
            )
        ):
            lease_preview_attempted = True
            lease_sample_candidate = (
                self._direct_motion_authority_successor_lease_sample(
                    successor_generation=int(tracking_plan_generation),
                    now_sec=now_sec,
                    authority_selection=safety_authority_selection,
                    ros_clock_stalled=ros_clock_watchdog.stalled,
                    active_control_fault_reason=active_control_fault_reason,
                    deadline_missed=watchdog.deadline_missed,
                )
            )
        elif (
            delivery_gap_lease is not None
            and int(tracking_plan_generation)
            not in (
                int(delivery_gap_lease.plan_generation),
                (
                    1
                    if int(delivery_gap_lease.plan_generation) >= 16_777_215
                    else int(delivery_gap_lease.plan_generation) + 1
                ),
            )
        ):
            # A committed N cohort may bridge only N itself or its direct
            # wrap-aware successor. Any other tracking generation is an
            # identity discontinuity and must revoke before selection can
            # fall back to that newer cached sample.
            lease_preview_attempted = True
            lease_sample_candidate = (
                self._direct_motion_authority_successor_lease_sample(
                    successor_generation=int(tracking_plan_generation),
                    now_sec=now_sec,
                    authority_selection=safety_authority_selection,
                    ros_clock_stalled=ros_clock_watchdog.stalled,
                    active_control_fault_reason=active_control_fault_reason,
                    deadline_missed=watchdog.deadline_missed,
                )
            )
        if (
            lease_preview_attempted
            and delivery_gap_lease is not None
            and lease_sample_candidate is None
            and self.motion_authority_delivery_gap_lease is None
        ):
            # The direct preview helper revokes the committed N cohort on any
            # hard negative (expiry, stale receipt, successor mismatch, or
            # final-command provenance failure).  Do not let the same timer
            # cycle fall back to a newer cached N sample and mint authority
            # again; the first post-revoke cycle is an explicit STOP.  This
            # applies to both the current-generation hold and the direct N+1
            # preview; merely having a lease without invoking the helper does
            # not trigger this guard.
            motion_authority_delivery_gap_lease_revoke_stop = True
            self.motion_authority_delivery_gap_revoke_pending = None
        selected_authority_constraint = authority_constraint
        selected_authority_constraint_time_sec = authority_constraint_time_sec
        selected_authority_plan_generation = authority_plan_generation
        if lease_sample_candidate is not None:
            delivery_gap_lease = self.motion_authority_delivery_gap_lease
            if delivery_gap_lease is not None:
                authority_constraint = copy.deepcopy(
                    self._coerce_safety_constraint_state(
                        delivery_gap_lease.constraint
                    )
                )
                authority_constraint_time_sec = self._lease_receipt_time(
                    delivery_gap_lease, "constraint_receipt_time_sec"
                )
                authority_plan_generation = int(
                    delivery_gap_lease.plan_generation
                )
        constraint_decision = self.safety_authority.evaluate(
            authority_constraint if self.safety_constraint_enabled else None,
            received_time_sec=(
                authority_constraint_time_sec
                if self.safety_constraint_enabled
                else None
            ),
            now_sec=now_sec,
            active_plan_generation=authority_plan_generation,
        )
        if (
            self.require_safety_constraint
            and safety_authority_selection.rendezvous_state
            == SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
            and lease_sample_candidate is None
            and not constraint_decision.stop_required
        ):
            # A normal cross-topic delivery gap is not a malformed safety
            # contract, so do not fault-latch the authority state.  Motion is
            # nevertheless forbidden until the latest plan's exact stamp peer
            # exists; only an older STOP pair may bridge this gap directly.
            constraint_decision = SafetyConstraintDecision(
                stop_required=True,
                speed_limit_mps=0.0,
                required_brake_decel_mps2=max(
                    0.0,
                    float(constraint_decision.required_brake_decel_mps2),
                ),
                constraint_generation=int(
                    constraint_decision.constraint_generation
                ),
                plan_generation=int(constraint_decision.plan_generation),
                reason="safety_authority_contract_unpaired",
            )
        if active_control_fault_reason:
            self.control_fault_latched = True
            self.control_fault_clear_cycles = 0
            self.control_fault_reason = active_control_fault_reason
            self.control_fault_last_release_stamp_ns = None
            self._invalidate_motion_authority_delivery_gap_lease()
        elif self.control_fault_latched:
            explicit_safe_release = (
                not constraint_decision.stop_required
                and authority_constraint is not None
                and authority_constraint.release_authorized
            )
            if explicit_safe_release:
                release_stamp_ns = authority_constraint.header_stamp_ns
                if (
                    self.control_fault_last_release_stamp_ns is None
                    or release_stamp_ns
                    > self.control_fault_last_release_stamp_ns
                ):
                    self.control_fault_clear_cycles += 1
                    self.control_fault_last_release_stamp_ns = release_stamp_ns
            else:
                self.control_fault_clear_cycles = 0
                self.control_fault_last_release_stamp_ns = None
            if (
                self.control_fault_clear_cycles
                >= self.control_fault_clear_safe_cycles
            ):
                self.control_fault_latched = False
                self.control_fault_clear_cycles = 0
                self.control_fault_reason = ""
                self.control_fault_last_release_stamp_ns = None
        normal_delivery_gap_baseline_hold_candidate = bool(
            self._normal_delivery_gap_baseline_hold_candidate(
                selection=safety_authority_selection,
                tracking_plan_generation=tracking_plan_generation,
                decision=constraint_decision,
                now_sec=now_sec,
            )
            and not active_control_fault_reason
            and not self.control_fault_latched
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not watchdog.deadline_missed
            and not ros_clock_watchdog.stalled
            and (
                not self.race_arm_required
                or (
                    self.finish_stop_latch.armed
                    and self.ros_clock_motion_ready
                )
            )
        )
        normal_delivery_gap_free_run_record_preservable = bool(
            normal_delivery_gap_baseline_hold_candidate
            and self._free_run_record_supports_normal_delivery_gap_hold(
                tracking_plan_generation=tracking_plan_generation,
                now_sec=now_sec,
            )
        )
        if (
            active_control_fault_reason
            or self.control_fault_latched
            or self.external_stop_latched
            or self.finish_stop_latch.latched
            or self.safety_constraint_timestamp_regressed
            or (
                self.race_arm_required
                and (
                    not self.finish_stop_latch.armed
                    or not self.ros_clock_motion_ready
                )
            )
            or constraint_decision.reason
            in (
                "safety_constraint_missing",
                "safety_constraint_stale",
                "safety_constraint_invalid",
                "safety_constraint_plan_generation_mismatch",
                "safety_constraint_timestamp_regression",
                "safety_constraint_generation_regression",
                "safety_constraint_plan_generation_regression",
                "safety_constraint_same_generation_conflict",
                "safety_constraint_fault_latched",
                "safety_authority_contract_unpaired",
            )
        ):
            self._invalidate_verified_stop_steering(
                preserve_baseline_free_run=(
                    normal_delivery_gap_baseline_hold_candidate
                )
            )
            if not normal_delivery_gap_free_run_record_preservable:
                self._invalidate_free_run_live_exact_record(
                    active_control_fault_reason
                    or self.control_fault_reason
                    or constraint_decision.reason
                    or "supervisory_safety_barrier"
                )
        elif self.free_run_live_exact_published_record is not None:
            record_age_sec = (
                now_sec
                - self.free_run_live_exact_published_record.publish_steady_time_sec
            )
            if (
                not math.isfinite(record_age_sec)
                or not 0.0
                <= record_age_sec
                <= self.free_run_source_provenance_timeout_sec
                or self.pure_pursuit_envelope_fault_latched
            ):
                self._invalidate_free_run_live_exact_record(
                    "source_provenance_expired_or_faulted"
                )
            elif self.free_run_source_gap_lease is not None:
                lease = self.free_run_source_gap_lease
                lease_age_sec = (
                    now_sec
                    - lease.acquired_steady_time_sec
                )
                if (
                    not math.isfinite(lease_age_sec)
                    or not 0.0
                    <= lease_age_sec
                    <= self.free_run_live_exact_evidence_timeout_sec
                    or not self._fresh(
                        self.free_run_live_exact_source_key_time_sec,
                        now_sec,
                        self.free_run_live_exact_evidence_timeout_sec,
                    )
                ):
                    # A stale pending successor can no longer be joined to the
                    # old final command. Revoke both objects so no later
                    # callback can recreate a hold from an expired transaction.
                    expired_successor_identity = (
                        lease.successor_source_identity
                    )
                    self._invalidate_free_run_live_exact_record(
                        "source_gap_lease_expired"
                    )
                    self.free_run_source_gap_expired_successor_identity = (
                        expired_successor_identity
                    )
                    self.free_run_source_gap_lease_expiry_count = min(
                        self.free_run_source_gap_lease_expiry_count + 1,
                        (1 << 64) - 1,
        )
        mpc_cmd_fresh = self._fresh(self.mpc_cmd_time_sec, now_sec, self.mpc_cmd_timeout_sec)
        pp_selection_debug_rejections: list[dict[str, object]] = []
        selected_pp_motion_sample = self._select_pp_motion_sample(
            tracking_plan_generation=tracking_plan_generation,
            now_sec=now_sec,
            ros_clock_stalled=ros_clock_watchdog.stalled,
            debug_rejection_sink=pp_selection_debug_rejections,
        )
        current_plan_entry = self.overtake_plan_cache.get(
            int(tracking_plan_generation)
        )
        passing_previous_sample = bool(
            selected_pp_motion_sample is not None
            and selected_pp_motion_sample.authority_proof is not None
            and current_plan_entry is not None
            and int(current_plan_entry[5]) == int(OvertakePlan.PASSING)
            and int(selected_pp_motion_sample.authority_proof.plan_generation)
            != int(tracking_plan_generation)
        )
        if passing_previous_sample:
            # Cache selection remains available to the historic STOP and
            # ATTACK_FOLLOW paths, but a PASSING predecessor cannot become
            # positive motion unless the dedicated committed lease below
            # accepts this exact successor gap.
            selected_pp_motion_sample = None
        if selected_pp_motion_sample is None:
            selected_pp_motion_sample = (
                self._select_pass_warmup_acquisition_sample(
                    tracking_plan_generation=tracking_plan_generation,
                    now_sec=now_sec,
                    ros_clock_stalled=ros_clock_watchdog.stalled,
                )
            )
        if motion_authority_delivery_gap_lease_revoke_stop:
            selected_pp_motion_sample = None
        motion_authority_delivery_gap_lease_active = False
        exact_successor_authority_complete = bool(
            lease_sample_candidate is not None
            and selected_pp_motion_sample is not None
            and selected_pp_motion_sample.authority_proof is not None
            and int(selected_pp_motion_sample.authority_proof.plan_generation)
            == int(tracking_plan_generation)
            and selected_authority_plan_generation is not None
            and int(selected_authority_plan_generation)
            == int(tracking_plan_generation)
            and selected_authority_constraint is not None
            and int(selected_authority_constraint.plan_generation)
            == int(tracking_plan_generation)
            and safety_authority_selection.rendezvous_state
            == SafetyAuthorityRendezvousState.EXACT_CURRENT
            and not constraint_decision.stop_required
        )
        if (
            lease_current_generation_hold
            and lease_sample_candidate is not None
            and self.motion_authority_delivery_gap_lease is not None
        ):
            lease = self.motion_authority_delivery_gap_lease
            # A constraint-first N+1 preview must keep the immutable N sample
            # ahead of any newer same-generation envelope in the cache.
            selected_pp_motion_sample = lease_sample_candidate
            tracking_plan_generation = int(lease.plan_generation)
            authority_plan_generation = int(lease.plan_generation)
            authority_constraint = self._coerce_safety_constraint_state(
                lease.constraint
            )
            authority_constraint_time_sec = self._lease_receipt_time(
                lease, "constraint_receipt_time_sec"
            )
            motion_authority_delivery_gap_lease_active = True
        elif (
            lease_sample_candidate is not None
            and not exact_successor_authority_complete
            and self.motion_authority_delivery_gap_lease is not None
        ):
            # Plan/PP can lead the exact N+1 safety authority.  Prefer the
            # immutable N cohort for this tick; no successor identity is
            # relabelled and the existing lease timeout remains authoritative.
            lease = self.motion_authority_delivery_gap_lease
            selected_pp_motion_sample = lease_sample_candidate
            tracking_plan_generation = int(lease.plan_generation)
            authority_plan_generation = int(lease.plan_generation)
            authority_constraint = self._coerce_safety_constraint_state(
                lease.constraint
            )
            authority_constraint_time_sec = self._lease_receipt_time(
                lease, "constraint_receipt_time_sec"
            )
            motion_authority_delivery_gap_lease_active = True
        elif (
            lease_sample_candidate is not None
            and selected_pp_motion_sample is not None
            and selected_pp_motion_sample.authority_proof is not None
            and int(selected_pp_motion_sample.authority_proof.plan_generation)
            == int(tracking_plan_generation)
            and selected_authority_plan_generation is not None
            and int(selected_authority_plan_generation)
            == int(tracking_plan_generation)
        ):
            # Exact N+1 PP completes the tuple.  Restore the selector's
            # authority inputs and advance the shared latch exactly once now;
            # the earlier preview evaluated neither N+1 nor any successor.
            authority_constraint = selected_authority_constraint
            authority_constraint_time_sec = selected_authority_constraint_time_sec
            authority_plan_generation = selected_authority_plan_generation
            constraint_decision = self.safety_authority.evaluate(
                authority_constraint if self.safety_constraint_enabled else None,
                received_time_sec=(
                    authority_constraint_time_sec
                    if self.safety_constraint_enabled
                    else None
                ),
                now_sec=now_sec,
                active_plan_generation=authority_plan_generation,
            )
        # Snapshot the selector's local result once.  The measurement record
        # below must not re-read the mutable envelope cache after arbitration.
        selector_watermark_identity = (
            tuple(self.pure_pursuit_envelope_watermark[0])
            if (
                self.mux_runtime_measurement_enabled
                or self.latest_sample_observability_enabled
            )
            and self.pure_pursuit_envelope_watermark is not None
            else None
        )
        selector_evaluated_identity = (
            tuple(selected_pp_motion_sample.envelope_identity)
            if (
                self.mux_runtime_measurement_enabled
                or self.latest_sample_observability_enabled
            )
            and selected_pp_motion_sample is not None
            and selected_pp_motion_sample.envelope_identity is not None
            else None
        )
        selected_free_run_envelope = (
            self._refresh_free_run_selected_envelope_join(
                selected_pp_motion_sample,
                tracking_plan_generation=tracking_plan_generation,
                now_sec=now_sec,
            )
            if self.free_run_live_exact_observe_enabled
            else None
        )
        expired_successor_exact_ack = bool(
            self.free_run_source_gap_expired_successor_identity is not None
            and self.free_run_live_exact_ack_valid
            and self.free_run_live_exact_ack is not None
            and self.free_run_live_exact_selected_envelope_valid
            and selected_free_run_envelope is not None
            and (
                int(self.free_run_live_exact_ack.source_key.baseline_instance_id),
                int(self.free_run_live_exact_ack.source_key.controller_instance_id),
                int(self.free_run_live_exact_ack.source_key.source_generation),
                self._stamp_ns(self.free_run_live_exact_ack.source_key.source_stamp),
            ) == self.free_run_source_gap_expired_successor_identity
        )
        selected_pp_command_fresh = bool(
            selected_pp_motion_sample is not None
            and self._fresh(
                selected_pp_motion_sample.command_receipt_time_sec,
                now_sec,
                self.pure_pursuit_cmd_timeout_sec,
            )
        )
        health = self._current_health(now_sec)
        recovery_state = self._current_recovery_state(now_sec)
        state_lattice_state = self._current_state_lattice_state(now_sec)

        if self.external_stop_latched:
            decision_source = "stop"
            reason = "external_safety_stop"
            fallback_active = True
            solved_cycles = 0
        elif self.control_fault_latched:
            decision_source = "stop"
            reason = self.control_fault_reason or "control_fault_latched"
            fallback_active = True
            solved_cycles = 0
        elif motion_authority_delivery_gap_lease_revoke_stop:
            decision_source = "stop"
            reason = "motion_authority_delivery_gap_lease_revoked"
            fallback_active = True
            solved_cycles = 0
        elif (
            self.race_arm_required
            and not self.finish_stop_latch.armed
            and not self.finish_stop_latch.latched
        ):
            decision_source = "stop"
            reason = "race_not_armed"
            fallback_active = True
            solved_cycles = 0
        elif (
            self.race_arm_required
            and self.finish_stop_latch.armed
            and not self.ros_clock_motion_ready
        ):
            decision_source = "stop"
            reason = "ros_clock_not_ready"
            fallback_active = True
            solved_cycles = 0
        elif not self.enabled:
            decision_source = "mpc" if mpc_cmd_fresh else "stop"
            reason = "disabled"
            fallback_active = False
            solved_cycles = 0
        else:
            decision = self.core.update(
                now_sec,
                mpc_cmd_fresh=mpc_cmd_fresh,
                pure_pursuit_cmd_fresh=selected_pp_command_fresh,
                mpc_health=health,
                recovery=recovery_state,
                state_lattice=state_lattice_state,
            )
            decision_source = decision.source
            reason = decision.reason
            fallback_active = decision.fallback_active
            solved_cycles = decision.solved_cycles

        selected_input_source = decision_source
        if (
            self.finish_stop_enabled
            and self.finish_stop_latch.latched
            and not self.external_stop_latched
            and not self.control_fault_latched
        ):
            # 公式の車両別Finishだけでterminal停止へ入る。操舵は直前の
            # fresh controller参照を保ち、減速中にコース外へ直進させない。
            decision_source = "finish_stop"
            reason = "official_vehicle_finish"
            fallback_active = True
            solved_cycles = 0

        selected_input_cmd = (
            selected_pp_motion_sample.command
            if selected_input_source == "pure_pursuit"
            and selected_pp_motion_sample is not None
            else (
                None
                if selected_input_source == "pure_pursuit"
                and self.require_safety_constraint
                else self._selected_input_command(selected_input_source)
            )
        )
        tracking_previous_generation = (
            16_777_215
            if int(tracking_plan_generation) == 1
            else int(tracking_plan_generation) - 1
        )
        if selected_pp_motion_sample is not None:
            if selected_input_source == "pure_pursuit":
                selected_input_cmd = selected_pp_motion_sample.command
            pp_tracking_status = selected_pp_motion_sample.authority_proof
            pp_tracking_status_time_sec = (
                selected_pp_motion_sample.authority_proof_receipt_time_sec
            )
            pp_tracking_status_valid = (
                selected_pp_motion_sample.authority_proof_valid
            )
            pp_tracking_status_fresh = self._fresh(
                selected_pp_motion_sample.authority_proof_receipt_time_sec,
                now_sec,
                self.pure_pursuit_tracking_status_timeout_sec,
            )
            pp_cmd_fresh = self._fresh(
                selected_pp_motion_sample.command_receipt_time_sec,
                now_sec,
                self.pure_pursuit_cmd_timeout_sec,
            )
            observed_pp_tracking_status = (
                selected_pp_motion_sample.observed_proof
            )
            observed_pp_tracking_status_valid = bool(
                selected_pp_motion_sample.observed_proof_valid
            )
            observed_pp_tracking_status_fresh = self._fresh(
                selected_pp_motion_sample.observed_proof_receipt_time_sec,
                now_sec,
                self.pure_pursuit_tracking_status_timeout_sec,
            )
        else:
            pp_tracking_status = None
            pp_tracking_status_time_sec = None
            pp_tracking_status_valid = False
            pp_tracking_status_fresh = False
            pp_cmd_fresh = False
            observed_pp_tracking_status = (
                self._select_bound_envelope_observed_proof(now_sec)
                if self.require_safety_constraint
                else None
            )
            observed_pp_tracking_status_valid = bool(
                observed_pp_tracking_status is not None
            )
            observed_pp_tracking_status_fresh = bool(
                observed_pp_tracking_status is not None
            )
        matched_current_tracking_entry = (
            (pp_tracking_status, pp_tracking_status_time_sec, pp_tracking_status_valid)
            if pp_tracking_status is not None
            and int(pp_tracking_status.plan_generation)
            == int(tracking_plan_generation)
            else None
        )
        selected_pp_command_valid = bool(
            selected_input_cmd is not None
            and math.isfinite(float(selected_input_cmd.longitudinal.speed))
            and math.isfinite(float(selected_input_cmd.longitudinal.acceleration))
            and math.isfinite(float(selected_input_cmd.lateral.steering_tire_angle))
        )
        tracking_usable_steering_limit_rad = float(
            self.tracking_usable_max_steering_angle_rad
        )
        tracking_usable_steering_limit_valid = bool(
            math.isfinite(tracking_usable_steering_limit_rad)
            and tracking_usable_steering_limit_rad > 0.0
            and math.isfinite(
                float(self.steering_limiter.config.max_steering_angle_rad)
            )
            and tracking_usable_steering_limit_rad
            <= float(self.steering_limiter.config.max_steering_angle_rad)
            + 1.0e-12
        )
        pure_pursuit_steering_command_valid = bool(
            selected_pp_motion_sample is not None
            and math.isfinite(
                float(
                    selected_pp_motion_sample.command.lateral.steering_tire_angle
                )
            )
        )
        pure_pursuit_command_within_tracking_steering_limit = bool(
            pure_pursuit_steering_command_valid
            and tracking_usable_steering_limit_valid
            and selected_pp_motion_sample is not None
            and abs(
                float(
                    selected_pp_motion_sample.command.lateral.steering_tire_angle
                )
            )
            <= tracking_usable_steering_limit_rad
        )
        pure_pursuit_command_exceeds_tracking_steering_limit = bool(
            pure_pursuit_steering_command_valid
            and tracking_usable_steering_limit_valid
            and selected_pp_motion_sample is not None
            and abs(
                float(
                    selected_pp_motion_sample.command.lateral.steering_tire_angle
                )
            )
            > tracking_usable_steering_limit_rad
        )
        pure_pursuit_command_within_actuator_hard_limit = bool(
            pure_pursuit_steering_command_valid
            and selected_pp_motion_sample is not None
            and math.isfinite(
                float(self.steering_limiter.config.max_steering_angle_rad)
            )
            and abs(
                float(
                    selected_pp_motion_sample.command.lateral.steering_tire_angle
                )
            )
            <= float(self.steering_limiter.config.max_steering_angle_rad)
        )
        pure_pursuit_command_violates_actuator_hard_limit = bool(
            selected_pp_motion_sample is not None
            and (
                not pure_pursuit_steering_command_valid
                or not pure_pursuit_command_within_actuator_hard_limit
            )
        )
        if pure_pursuit_command_violates_actuator_hard_limit:
            # Crossing the actuator boundary is a safety barrier.  A prior
            # nominal steering reference must not become usable again on the
            # next soft/hard-corridor sample without a newly verified,
            # soft-limit-compliant command being published first.
            self._invalidate_verified_baseline_free_run_steering()
            self._invalidate_free_run_live_exact_record(
                "pure_pursuit_actuator_hard_limit"
            )
        pp_tracking_proof_usable = bool(
            pp_tracking_status_fresh
            and pp_tracking_status_valid
            and pp_tracking_status is not None
            and bool(pp_tracking_status.pp_command_fresh)
            and bool(pp_tracking_status.trajectory_tracking_usable)
            and tracking_plan_generation >= 0
            and int(pp_tracking_status.plan_generation)
            == int(tracking_plan_generation)
        )
        tracking_evidence_generation = (
            int(pp_tracking_status.plan_generation)
            if pp_tracking_status is not None
            else 0
        )
        tracking_evidence_identity = (
            self.overtake_plan_motion_identity_cache.get(
                tracking_evidence_generation
            )
        )
        current_tracking_identity = (
            self.overtake_plan_motion_identity_cache.get(
                int(tracking_plan_generation)
            )
            if tracking_plan_generation > 0
            else None
        )
        tracking_generation_distance = (
            int(tracking_plan_generation) - tracking_evidence_generation
            if int(tracking_plan_generation) > tracking_evidence_generation
            else (16_777_215 - tracking_evidence_generation)
            + int(tracking_plan_generation)
        )
        # Replanning may change Cartesian samples while the same target/side
        # transaction continues. Current Plan/Constraint and command binding
        # validate that geometry independently; this delivery bridge therefore
        # follows maneuver identity instead of requiring byte-identical points.
        tracking_delivery_stable_successor = bool(
            tracking_evidence_identity is not None
            and current_tracking_identity is not None
            and tracking_evidence_generation != int(tracking_plan_generation)
            and 0 < tracking_generation_distance <= (16_777_215 // 2)
            and tuple(current_tracking_identity[:6])
            == tuple(tracking_evidence_identity[:6])
            and int(current_tracking_identity[6])
            > int(tracking_evidence_identity[6])
            and int(current_tracking_identity[8])
            == int(tracking_evidence_identity[8])
            and int(current_tracking_identity[9])
            > int(tracking_evidence_identity[9])
        )
        matched_pp_tracking_previous_generation_usable = bool(
            pp_tracking_status_fresh
            and pp_tracking_status_valid
            and pp_tracking_status is not None
            and bool(pp_tracking_status.pp_command_fresh)
            and bool(pp_tracking_status.trajectory_tracking_usable)
            and tracking_plan_generation > 0
            and (
                int(pp_tracking_status.plan_generation)
                == tracking_previous_generation
                or tracking_delivery_stable_successor
            )
        )
        pp_tracking_delivery_gap_usable = bool(
            not pp_tracking_proof_usable
            and matched_pp_tracking_previous_generation_usable
        )
        attack_follow_transport_continuity_usable = (
            self._maneuver_transport_continuity_authorized(
                tracking_plan_generation=tracking_plan_generation,
                authority_plan_generation=authority_plan_generation,
                authority_constraint=authority_constraint,
                authority_constraint_time_sec=authority_constraint_time_sec,
                constraint_decision=constraint_decision,
                selected_motion_sample=selected_pp_motion_sample,
                selected_input_source=selected_input_source,
                pp_tracking_delivery_gap_usable=(
                    pp_tracking_delivery_gap_usable
                ),
                active_control_fault_reason=active_control_fault_reason,
                now_sec=now_sec,
                expected_phase=OvertakePlan.ATTACK_FOLLOW,
            )
        )
        pass_transport_continuity_usable = (
            self._maneuver_transport_continuity_authorized(
                tracking_plan_generation=tracking_plan_generation,
                authority_plan_generation=authority_plan_generation,
                authority_constraint=authority_constraint,
                authority_constraint_time_sec=authority_constraint_time_sec,
                constraint_decision=constraint_decision,
                selected_motion_sample=selected_pp_motion_sample,
                selected_input_source=selected_input_source,
                pp_tracking_delivery_gap_usable=(
                    pp_tracking_delivery_gap_usable
                ),
                active_control_fault_reason=active_control_fault_reason,
                now_sec=now_sec,
                expected_phase=OvertakePlan.PASSING,
            )
        )
        same_stamp_tracking_generations = sorted(
            int(cached_generation)
            for cached_stamp_ns, cached_generation in (
                self.pure_pursuit_tracking_status_cache
            )
            if selected_pp_motion_sample is not None
            and cached_stamp_ns == selected_pp_motion_sample.command_stamp_ns
        )
        exact_tracking_status = (
            matched_current_tracking_entry[0]
            if matched_current_tracking_entry is not None
            else None
        )
        exact_tracking_status_fresh = self._fresh(
            (
                matched_current_tracking_entry[1]
                if matched_current_tracking_entry is not None
                else None
            ),
            now_sec,
            self.pure_pursuit_tracking_status_timeout_sec,
        )
        exact_tracking_status_valid = bool(
            matched_current_tracking_entry is not None
            and matched_current_tracking_entry[2]
        )
        exact_tracking_binding_required = bool(
            matched_current_tracking_entry is not None
            and len(same_stamp_tracking_generations) > 1
        )
        exact_tracking_binding_matches = bool(
            not exact_tracking_binding_required
            or (
                exact_tracking_status is not None
                and self._tracking_status_binds_pure_pursuit_command(
                    exact_tracking_status, selected_pp_motion_sample.command
                )
            )
        )
        attack_follow_stop_transport_release_ready = bool(
            pure_pursuit_command_within_tracking_steering_limit
            and self._maneuver_transport_continuity_authorized(
                tracking_plan_generation=tracking_plan_generation,
                authority_plan_generation=authority_plan_generation,
                authority_constraint=authority_constraint,
                authority_constraint_time_sec=authority_constraint_time_sec,
                constraint_decision=constraint_decision,
                selected_motion_sample=selected_pp_motion_sample,
                selected_input_source=selected_input_source,
                pp_tracking_delivery_gap_usable=(
                    pp_tracking_delivery_gap_usable
                ),
                active_control_fault_reason=active_control_fault_reason,
                now_sec=now_sec,
                expected_phase=OvertakePlan.ATTACK_FOLLOW,
                stop_release_bridge=True,
            )
        )
        motion_pp_tracking_proof_usable = bool(
            pure_pursuit_command_within_tracking_steering_limit
            and (
                pp_tracking_proof_usable
                or attack_follow_transport_continuity_usable
                or pass_transport_continuity_usable
            )
        )
        motion_pp_tracking_continuity_usable = bool(
            attack_follow_transport_continuity_usable
            or pass_transport_continuity_usable
        )
        tracking_plan_entry = self.overtake_plan_cache.get(
            int(tracking_plan_generation)
        )
        tracking_plan_phase = (
            int(tracking_plan_entry[5])
            if tracking_plan_entry is not None
            else None
        )
        motion_pp_tracking_continuity_applicable = bool(
            not pp_tracking_proof_usable
            and pp_tracking_delivery_gap_usable
            and tracking_plan_phase
            in (int(OvertakePlan.ATTACK_FOLLOW), int(OvertakePlan.PASSING))
        )
        motion_pp_tracking_proof_blockers = (
            self._motion_pp_tracking_proof_blockers(
                exact_tuple_present=(
                    matched_current_tracking_entry is not None
                ),
                same_stamp_other_generation_present=bool(
                    same_stamp_tracking_generations
                    and int(tracking_plan_generation)
                    not in same_stamp_tracking_generations
                ),
                exact_tuple_valid=exact_tracking_status_valid,
                binding_required=exact_tracking_binding_required,
                binding_matches=exact_tracking_binding_matches,
                status_fresh=exact_tracking_status_fresh,
                pp_command_fresh=pp_cmd_fresh,
                upstream_pp_command_fresh=bool(
                    exact_tracking_status is not None
                    and exact_tracking_status.pp_command_fresh
                ),
                steering_usable=(
                    pure_pursuit_command_within_tracking_steering_limit
                ),
                steering_blocked=bool(
                    pure_pursuit_command_exceeds_tracking_steering_limit
                    or (
                        selected_pp_motion_sample is not None
                        and (
                            not pure_pursuit_steering_command_valid
                            or not tracking_usable_steering_limit_valid
                        )
                    )
                ),
                exact_proof_usable=pp_tracking_proof_usable,
                continuity_applicable=(
                    motion_pp_tracking_continuity_applicable
                ),
                continuity_usable=motion_pp_tracking_continuity_usable,
            )
        )
        motion_pp_tracking_proof_first_false = (
            motion_pp_tracking_proof_blockers[0]
            if motion_pp_tracking_proof_blockers
            else "none"
        )
        motion_pp_tracking_proof_required = bool(
            self.safety_constraint_enabled
            and self.require_safety_constraint
            and decision_source == "pure_pursuit"
            and not constraint_decision.stop_required
        )
        motion_pp_tracking_proof_failure_active = bool(
            motion_pp_tracking_proof_required
            and not motion_pp_tracking_proof_usable
        )
        motion_pp_tracking_failure_signature = (
            motion_pp_tracking_proof_first_false
            if motion_pp_tracking_proof_failure_active
            else ""
        )
        force_motion_pp_tracking_debug_requested = bool(
            motion_pp_tracking_proof_failure_active
            and motion_pp_tracking_failure_signature
            != self.last_motion_pp_tracking_proof_failure_signature
        )
        force_motion_pp_tracking_debug = bool(
            force_motion_pp_tracking_debug_requested
            and now_sec - self.last_forced_motion_pp_tracking_debug_sec
            >= self.forced_motion_pp_tracking_debug_min_period_sec
        )
        if force_motion_pp_tracking_debug:
            self.last_forced_motion_pp_tracking_debug_sec = now_sec
            self.last_motion_pp_tracking_proof_failure_signature = (
                motion_pp_tracking_failure_signature
            )
        elif not motion_pp_tracking_proof_failure_active:
            self.last_motion_pp_tracking_proof_failure_signature = ""
        lateral_stop_plan_authorized = self._lateral_stop_plan_authorized(
            plan_generation=authority_plan_generation,
            constraint=authority_constraint,
            tracking_status=pp_tracking_status,
            now_sec=now_sec,
        )
        baseline_stop_plan_authorized = (
            self._baseline_stop_plan_authorized(
                plan_generation=authority_plan_generation,
                constraint=authority_constraint,
                now_sec=now_sec,
            )
        )
        baseline_free_run_plan_authorized = (
            self._baseline_free_run_plan_authorized(
                plan_generation=authority_plan_generation,
                constraint=authority_constraint,
                now_sec=now_sec,
            )
        )
        baseline_free_run_cache_contract_current = bool(
            baseline_free_run_plan_authorized
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pp_tracking_proof_usable
            and exact_tracking_status_fresh
            and exact_tracking_status_valid
            and exact_tracking_binding_matches
            and not self.safety_authority_contract_unpaired
        )
        free_run_source_delivery_gap_candidate = bool(
            self.free_run_live_exact_pre_ack_hold_enabled
            and baseline_free_run_cache_contract_current
            and decision_source == "pure_pursuit"
            and selected_input_source == "pure_pursuit"
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_actuator_hard_limit
            and self._free_run_source_delivery_gap_pending(
                tracking_plan_generation=tracking_plan_generation,
                now_sec=now_sec,
            )
            and not active_control_fault_reason
            and not self.control_fault_latched
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not watchdog.deadline_missed
            and not ros_clock_watchdog.stalled
        )
        # A plan/constraint pair for N may already be exact while PP still
        # reports the same command tuple for N-1.  This is a bounded transport
        # gap, not permission to use the N-1 proof for motion.  Preserve only
        # the final-published steering from the direct predecessor so the
        # STOP command does not discard lateral continuity before PP catches
        # up with exact N.
        pp_normal_delivery_gap_baseline_hold_candidate = bool(
            self._normal_delivery_gap_baseline_hold_candidate(
                selection=safety_authority_selection,
                tracking_plan_generation=tracking_plan_generation,
                decision=constraint_decision,
                now_sec=now_sec,
                pp_tracking_delivery_gap_usable=(
                    pp_tracking_delivery_gap_usable
                ),
            )
        )
        normal_delivery_gap_baseline_hold_candidate = bool(
            normal_delivery_gap_baseline_hold_candidate
            or pp_normal_delivery_gap_baseline_hold_candidate
        )
        if (
            self.last_verified_baseline_free_run_steering_rad is not None
            and not normal_delivery_gap_baseline_hold_candidate
            and (
                not baseline_free_run_cache_contract_current
                or self.last_verified_baseline_free_run_plan_generation is None
                or int(self.last_verified_baseline_free_run_plan_generation)
                != int(authority_plan_generation)
                or self.last_verified_baseline_free_run_plan_stamp_ns is None
                or self.overtake_plan_header_stamp_ns is None
                or int(self.last_verified_baseline_free_run_plan_stamp_ns)
                != int(self.overtake_plan_header_stamp_ns)
                or self.last_verified_baseline_free_run_epoch is None
                or int(self.last_verified_baseline_free_run_epoch)
                != int(self.finish_stop_latch.epoch)
            )
        ):
            self._invalidate_verified_baseline_free_run_steering()
        if (
            pp_normal_delivery_gap_baseline_hold_candidate
            or free_run_source_delivery_gap_candidate
        ):
            # Exact plan/constraint N cannot grant motion through an N-1 PP
            # tracking proof.  Convert only this direct-successor FREE_RUN
            # transport gap into the same typed zero-speed composition used
            # for a plan/constraint delivery gap.  The exact tuple remains
            # required on the next cycle to resume authority.
            constraint_decision = SafetyConstraintDecision(
                stop_required=True,
                speed_limit_mps=0.0,
                required_brake_decel_mps2=max(
                    0.0,
                    float(constraint_decision.required_brake_decel_mps2),
                ),
                constraint_generation=int(
                    constraint_decision.constraint_generation
                ),
                plan_generation=int(constraint_decision.plan_generation),
                reason=(
                    "free_run_source_normal_delivery_gap"
                    if free_run_source_delivery_gap_candidate
                    else "controller_tracking_normal_delivery_gap"
                ),
            )
        lateral_stop_contract_delivery_gap_authorized = (
            self._lateral_stop_contract_delivery_gap_authorized(
                plan_generation=authority_plan_generation,
                constraint=authority_constraint,
                constraint_decision=constraint_decision,
                now_sec=now_sec,
            )
        )
        baseline_stop_contract_delivery_gap_authorized = (
            self._baseline_stop_contract_delivery_gap_authorized(
                plan_generation=authority_plan_generation,
                constraint=authority_constraint,
                constraint_decision=constraint_decision,
                now_sec=now_sec,
            )
        )
        planner_stop_release_bootstrap_valid = (
            self._planner_stop_release_bootstrap_valid(
                plan_generation=authority_plan_generation,
                constraint=authority_constraint,
                now_sec=now_sec,
            )
        )
        pure_pursuit_cmd_valid_for_release = bool(
            selected_pp_motion_sample is not None
            and float(selected_pp_motion_sample.command.longitudinal.speed) >= 0.0
            and math.isfinite(
                float(selected_pp_motion_sample.command.longitudinal.speed)
            )
            and math.isfinite(
                float(selected_pp_motion_sample.command.longitudinal.acceleration)
            )
            and math.isfinite(
                float(
                    selected_pp_motion_sample.command.lateral.steering_tire_angle
                )
            )
            and pure_pursuit_command_within_tracking_steering_limit
        )
        # The planner STOP cap is a monotonic longitudinal authority and may
        # shrink with the nearly stationary ego speed.  The warm-up PP command
        # intentionally remains at the bounded bootstrap speed (normally
        # 0.20 m/s), while the final command is still composed to 0 m/s below.
        # Comparing the proof command with the shrinking STOP cap creates an
        # unrecoverable STOP -> no proof -> STOP cycle.  Bound the proof by its
        # dedicated contract cap; exact generation/stamp, STOP authority and
        # final zero-speed composition remain independently required.
        planner_stop_bootstrap_pp_speed_valid = bool(
            planner_stop_release_bootstrap_valid
            and selected_pp_motion_sample is not None
            and math.isfinite(
                float(selected_pp_motion_sample.command.longitudinal.speed)
            )
            and 0.0 <= float(selected_pp_motion_sample.command.longitudinal.speed)
            <= self.planner_stop_release_bootstrap_max_speed_mps + 1.0e-6
        )
        constraint_release_reason = (
            str(authority_constraint.reason).strip()
            if authority_constraint is not None
            else ""
        )
        authority_contract_matches_tracking = bool(
            authority_constraint is not None
            and authority_plan_generation is not None
            and bool(authority_constraint.valid)
            and bool(authority_constraint.stop_requested)
            and int(authority_constraint.plan_generation)
            == int(authority_plan_generation)
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
        )
        constraint_decision_matches_authority = bool(
            authority_plan_generation is not None
            and int(constraint_decision.plan_generation)
            == int(authority_plan_generation)
        )
        # Fault解除用ACKは縦STOPを解除するための証拠としてだけ残す。
        # lateral_stop_authority_usableには接続せず、横操舵authorityにはしない。
        historical_authority_fault_release_bridge = bool(
            constraint_decision.reason == "safety_constraint_fault_latched"
            and authority_contract_matches_tracking
        )
        historical_authority_release_lateral_bridge = False
        lateral_stop_fault_free = bool(
            constraint_decision.reason
            in (
                "safety_constraint_applied",
                "safety_constraint_retransmit",
                "safety_constraint_relaxation_not_authorized",
            )
            and not self.safety_authority_contract_unpaired
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
        )
        lateral_stop_authority_usable = bool(
            lateral_stop_fault_free
            and lateral_stop_plan_authorized
            and constraint_decision_matches_authority
        )
        pass_warmup_steering_acquisition_usable = bool(
            constraint_decision.stop_required
            and selected_input_source == "pure_pursuit"
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_tracking_steering_limit
            and pp_tracking_status is not None
            and bool(
                pp_tracking_status.pass_warmup_steering_acquisition_active
            )
            and not bool(pp_tracking_status.pass_warmup_motion_ready)
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and lateral_stop_authority_usable
            and not self.external_stop_latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
        )
        maneuver_lateral_stop_tracking_usable = bool(
            constraint_decision.stop_required
            and selected_input_source == "pure_pursuit"
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_tracking_steering_limit
            and pp_tracking_proof_usable
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and lateral_stop_authority_usable
            and not self.external_stop_latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
        )
        pass_probe_exact_current_usable = False
        pass_probe_lateral_stop_authority_token = 0
        if (
            self.external_stop_latched
            or self.finish_stop_latch.latched
            or self.control_fault_latched
            or bool(active_control_fault_reason)
            or ros_clock_watchdog.stalled
            or watchdog.deadline_missed
        ):
            self.motion_authority_warmup_proof = None
        elif (
            maneuver_lateral_stop_tracking_usable
            and authority_plan_generation is not None
            and int(
                self.overtake_plan_lateral_stop_authority_cache.get(
                    int(authority_plan_generation),
                    (OvertakePlan.LATERAL_STOP_NONE,),
                )[0]
            )
            == int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
        ):
            pass_probe_exact_current_usable = (
                self._capture_motion_authority_warmup_proof(
                    selected_sample=selected_pp_motion_sample,
                    tracking_plan_generation=tracking_plan_generation,
                    authority_plan_generation=authority_plan_generation,
                    authority_constraint=authority_constraint,
                    now_sec=now_sec,
                )
            )
            if pass_probe_exact_current_usable:
                typed_probe_authority = (
                    self.overtake_plan_lateral_stop_authority_cache.get(
                        int(authority_plan_generation)
                    )
                )
                if typed_probe_authority is not None:
                    pass_probe_lateral_stop_authority_token = int(
                        typed_probe_authority[2]
                    )
        elif (
            constraint_decision.stop_required
            or self.safety_authority_contract_unpaired
            or authority_constraint is None
            or not authority_constraint.valid
        ):
            # PASS_WARMUP above is the only STOP state allowed to mint or
            # retain a Stage-A proof. Any other Planner STOP (including
            # collision/wall/CBF rejection), invalid/stale constraint, or
            # unpaired plan/constraint revokes it immediately.
            typed_stop_authority = (
                self.overtake_plan_lateral_stop_authority_cache.get(
                    int(authority_plan_generation)
                )
                if authority_plan_generation is not None
                else None
            )
            exact_pass_warmup_stop = bool(
                not self.safety_authority_contract_unpaired
                and authority_constraint is not None
                and authority_constraint.valid
                and authority_constraint.stop_requested
                and not authority_constraint.release_authorized
                and typed_stop_authority is not None
                and int(typed_stop_authority[0])
                == int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
                and int(typed_stop_authority[2]) > 0
                and bool(typed_stop_authority[7])
            )
            if not retain_warmup_proof_in_stop_state(
                exact_pass_warmup_stop=exact_pass_warmup_stop,
                rendezvous_state=safety_authority_selection.rendezvous_state,
            ):
                self.motion_authority_warmup_proof = None
        baseline_stop_tracking_usable = bool(
            constraint_decision.stop_required
            and constraint_decision.reason
            in (
                "safety_constraint_applied",
                "safety_constraint_retransmit",
            )
            and selected_input_source == "pure_pursuit"
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_tracking_steering_limit
            and pp_tracking_proof_usable
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and baseline_stop_plan_authorized
            and constraint_decision_matches_authority
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
        )
        lateral_stop_tracking_usable = bool(
            maneuver_lateral_stop_tracking_usable
            or baseline_stop_tracking_usable
        )
        last_verified_tracking_steering_age_sec = (
            now_sec - self.last_verified_tracking_steering_time_sec
            if self.last_verified_tracking_steering_time_sec is not None
            else float("inf")
        )
        last_verified_lateral_stop_steering_age_sec = (
            now_sec - self.last_verified_lateral_stop_steering_time_sec
            if self.last_verified_lateral_stop_steering_time_sec is not None
            else float("inf")
        )
        last_verified_baseline_stop_steering_age_sec = (
            now_sec - self.last_verified_baseline_stop_steering_time_sec
            if self.last_verified_baseline_stop_steering_time_sec is not None
            else float("inf")
        )
        last_verified_baseline_free_run_steering_age_sec = (
            now_sec - self.last_verified_baseline_free_run_steering_time_sec
            if self.last_verified_baseline_free_run_steering_time_sec is not None
            else float("inf")
        )
        normal_delivery_gap_zero_speed_baseline_hold_usable = bool(
            normal_delivery_gap_baseline_hold_candidate
            and constraint_decision.stop_required
            and constraint_decision.reason
            in (
                "safety_authority_contract_unpaired",
                "controller_tracking_normal_delivery_gap",
            )
            and decision_source == "pure_pursuit"
            and selected_input_source == "pure_pursuit"
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_actuator_hard_limit
            and self.last_verified_baseline_free_run_steering_rad is not None
            and math.isfinite(
                self.last_verified_baseline_free_run_steering_rad
            )
            and abs(self.last_verified_baseline_free_run_steering_rad)
            <= tracking_usable_steering_limit_rad
            and math.isfinite(
                last_verified_baseline_free_run_steering_age_sec
            )
            and 0.0
            <= last_verified_baseline_free_run_steering_age_sec
            <= self.lateral_stop_steering_hold_timeout_sec
            and (
                not self.free_run_live_exact_observe_enabled
                or (
                    self.free_run_live_exact_published_record is not None
                    and self._fresh(
                        self.free_run_live_exact_source_key_time_sec,
                        now_sec,
                        self.free_run_source_provenance_timeout_sec,
                    )
                )
            )
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
            and not watchdog.deadline_missed
        )
        baseline_soft_steering_hold_usable = bool(
            decision_source == "pure_pursuit"
            and selected_input_source == "pure_pursuit"
            and not constraint_decision.stop_required
            and pure_pursuit_command_exceeds_tracking_steering_limit
            and pure_pursuit_command_within_actuator_hard_limit
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pp_tracking_proof_usable
            and exact_tracking_status_fresh
            and exact_tracking_status_valid
            and exact_tracking_binding_matches
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and baseline_free_run_plan_authorized
            and not self.safety_authority_contract_unpaired
            and self.last_verified_baseline_free_run_plan_generation is not None
            and int(self.last_verified_baseline_free_run_plan_generation)
            == int(authority_plan_generation)
            and self.last_verified_baseline_free_run_plan_stamp_ns is not None
            and self.overtake_plan_header_stamp_ns is not None
            and int(self.last_verified_baseline_free_run_plan_stamp_ns)
            == int(self.overtake_plan_header_stamp_ns)
            and self.last_verified_baseline_free_run_epoch is not None
            and int(self.last_verified_baseline_free_run_epoch)
            == int(self.finish_stop_latch.epoch)
            and self.last_verified_baseline_free_run_steering_rad is not None
            and math.isfinite(
                self.last_verified_baseline_free_run_steering_rad
            )
            and abs(self.last_verified_baseline_free_run_steering_rad)
            <= tracking_usable_steering_limit_rad
            and math.isfinite(last_verified_baseline_free_run_steering_age_sec)
            and 0.0 <= last_verified_baseline_free_run_steering_age_sec
            <= self.lateral_stop_steering_hold_timeout_sec
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
            and not watchdog.deadline_missed
        )
        free_run_live_record = self.free_run_live_exact_published_record
        current_plan_stamp_ns = self.overtake_plan_header_stamp_ns
        current_plan_delivery = (
            self.free_run_live_exact_plan_delivery_cache.get(
                (
                    int(tracking_plan_generation),
                    int(current_plan_stamp_ns),
                )
            )
            if current_plan_stamp_ns is not None
            else None
        )
        current_ack_is_complete = bool(
            self.free_run_live_exact_ack_valid
            and self.free_run_live_exact_ack is not None
            and current_plan_stamp_ns is not None
            and self._stamp_ns(
                self.free_run_live_exact_ack.plan_key.plan_stamp
            )
            == int(current_plan_stamp_ns)
        )
        free_run_live_record_age_sec = (
            now_sec - free_run_live_record.publish_steady_time_sec
            if free_run_live_record is not None
            else math.inf
        )
        free_run_live_exact_pre_ack_hold_usable = bool(
            self.free_run_live_exact_pre_ack_hold_enabled
            and free_run_live_record is not None
            and decision_source == "pure_pursuit"
            and selected_input_source == "pure_pursuit"
            and not constraint_decision.stop_required
            and not pp_tracking_proof_usable
            and pure_pursuit_command_within_actuator_hard_limit
            and pp_cmd_fresh
            and selected_pp_command_valid
            and matched_pp_tracking_previous_generation_usable
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and baseline_free_run_plan_authorized
            and not self.safety_authority_contract_unpaired
            and current_plan_stamp_ns is not None
            and 0
            < int(current_plan_stamp_ns)
            - int(free_run_live_record.plan_stamp_ns)
            <= int(
                self.free_run_live_exact_evidence_timeout_sec * 1.0e9
            )
            and current_plan_delivery is not None
            and bool(current_plan_delivery[0])
            and bytes(current_plan_delivery[1])
            == bytes(free_run_live_record.canonical_plan_sha256)
            and tuple(
                self.free_run_live_exact_source_key_identity or ()
            )
            == tuple(free_run_live_record.source_identity)
            and self._fresh(
                self.free_run_live_exact_source_key_time_sec,
                now_sec,
                self.free_run_live_exact_evidence_timeout_sec,
            )
            and self.free_run_live_exact_selected_envelope_reason
            == "forward_pre_ack_transition"
            and not current_ack_is_complete
            and math.isfinite(free_run_live_record_age_sec)
            and 0.0
            <= free_run_live_record_age_sec
            <= self.free_run_live_exact_evidence_timeout_sec
            and math.isfinite(
                free_run_live_record.published_steering_rad
            )
            and abs(free_run_live_record.published_steering_rad)
            <= min(
                float(free_run_live_record.tracking_soft_limit_rad),
                tracking_usable_steering_limit_rad,
            )
            and hashlib.sha256(
                free_run_live_record.final_command_cdr
            ).digest()
            == free_run_live_record.final_command_sha256
            and self.steering_limiter.has_last_steering
            and self.steering_limiter.last_source == "pure_pursuit"
            and math.isclose(
                float(self.steering_limiter.last_steering_rad),
                float(free_run_live_record.published_steering_rad),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
            and not watchdog.deadline_missed
        )
        free_run_source_delivery_gap_hold_usable = bool(
            free_run_source_delivery_gap_candidate
            and constraint_decision.stop_required
            and constraint_decision.reason
            == "free_run_source_normal_delivery_gap"
            and free_run_live_record is not None
            and self.free_run_source_gap_lease is not None
            and self.free_run_source_gap_lease.provenance_record
            is free_run_live_record
            and math.isfinite(free_run_live_record.published_steering_rad)
            and abs(free_run_live_record.published_steering_rad)
            <= min(
                float(free_run_live_record.tracking_soft_limit_rad),
                tracking_usable_steering_limit_rad,
            )
            and hashlib.sha256(
                free_run_live_record.final_command_cdr
            ).digest()
            == free_run_live_record.final_command_sha256
            and self.steering_limiter.has_last_steering
            and self.steering_limiter.last_source == "pure_pursuit"
            and math.isclose(
                float(self.steering_limiter.last_steering_rad),
                float(free_run_live_record.published_steering_rad),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
        )
        verified_lateral_generation = (
            int(self.last_verified_lateral_stop_plan_generation)
            if self.last_verified_lateral_stop_plan_generation is not None
            else 0
        )
        current_lateral_generation = (
            int(authority_plan_generation)
            if authority_plan_generation is not None
            else 0
        )
        verified_lateral_identity = self.overtake_plan_motion_identity_cache.get(
            verified_lateral_generation
        )
        current_lateral_identity = self.overtake_plan_motion_identity_cache.get(
            current_lateral_generation
        )
        verified_lateral_typed = (
            self.overtake_plan_lateral_stop_authority_cache.get(
                verified_lateral_generation
            )
        )
        current_lateral_typed = (
            self.overtake_plan_lateral_stop_authority_cache.get(
                current_lateral_generation
            )
        )
        lateral_generation_distance = (
            current_lateral_generation - verified_lateral_generation
            if current_lateral_generation > verified_lateral_generation
            else (16_777_215 - verified_lateral_generation)
            + current_lateral_generation
        )
        # This bridge can retain only an already verified steering angle while
        # longitudinal STOP remains asserted. Maneuver identity and typed token
        # continuity are required below; replanned point equality is not.
        lateral_hold_stable_successor = bool(
            verified_lateral_identity is not None
            and current_lateral_identity is not None
            and verified_lateral_generation != current_lateral_generation
            and 0 < lateral_generation_distance <= (16_777_215 // 2)
            and tuple(current_lateral_identity[:6])
            == tuple(verified_lateral_identity[:6])
            and int(current_lateral_identity[6])
            > int(verified_lateral_identity[6])
            and int(current_lateral_identity[8])
            == int(OvertakePlan.PASSING)
            and int(current_lateral_identity[9])
            > int(verified_lateral_identity[9])
        )
        lateral_hold_generation_continuous = bool(
            verified_lateral_generation == current_lateral_generation
            or lateral_hold_stable_successor
        )
        pass_warmup_acquisition_steering_hold_usable = bool(
            constraint_decision.stop_required
            and constraint_decision.reason
            in (
                "safety_constraint_applied",
                "safety_constraint_retransmit",
                "safety_constraint_relaxation_not_authorized",
            )
            and not self.safety_authority_contract_unpaired
            and authority_constraint is not None
            and authority_constraint.valid
            and authority_constraint.stop_requested
            and not authority_constraint.release_authorized
            and int(authority_constraint.plan_generation)
            == current_lateral_generation
            and current_lateral_identity is not None
            and int(authority_constraint.header_stamp_ns)
            == int(current_lateral_identity[6])
            and current_lateral_typed is not None
            and int(current_lateral_typed[0])
            == int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
            and int(current_lateral_typed[2]) > 0
            and bool(current_lateral_typed[7])
            and verified_lateral_typed is not None
            and int(verified_lateral_typed[0])
            == int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
            and int(verified_lateral_typed[1]) == int(current_lateral_typed[1])
            and int(verified_lateral_typed[2]) == int(current_lateral_typed[2])
            and lateral_hold_generation_continuous
            and self.last_verified_lateral_stop_steering_rad is not None
            and math.isfinite(self.last_verified_lateral_stop_steering_rad)
            and math.isfinite(last_verified_lateral_stop_steering_age_sec)
            and 0.0 <= last_verified_lateral_stop_steering_age_sec
            <= self.lateral_stop_steering_hold_timeout_sec
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
            and not watchdog.deadline_missed
        )
        maneuver_lateral_stop_steering_hold_usable = bool(
            constraint_decision.stop_required
            and not lateral_stop_tracking_usable
            and selected_input_source == "pure_pursuit"
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_tracking_steering_limit
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and (
                (
                    pp_tracking_delivery_gap_usable
                    and lateral_stop_authority_usable
                )
                or (
                    lateral_stop_contract_delivery_gap_authorized
                    and (
                        pp_tracking_proof_usable
                        or pp_tracking_delivery_gap_usable
                    )
                )
            )
            and self.last_verified_lateral_stop_plan_generation is not None
            and lateral_hold_generation_continuous
            and self.last_verified_lateral_stop_steering_rad is not None
            and math.isfinite(self.last_verified_lateral_stop_steering_rad)
            and math.isfinite(last_verified_lateral_stop_steering_age_sec)
            and 0.0 <= last_verified_lateral_stop_steering_age_sec
            <= self.lateral_stop_steering_hold_timeout_sec
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
        )
        baseline_stop_steering_hold_usable = bool(
            constraint_decision.stop_required
            and not lateral_stop_tracking_usable
            and selected_input_source == "pure_pursuit"
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_tracking_steering_limit
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and (
                (
                    # The current baseline plan/constraint bundle is exact,
                    # but its PP command/status peer may still be one planner
                    # generation behind for one control tick.  The transport
                    # proof below is already bounded to N-1/N; keep only the
                    # last exactly verified steering while longitudinal STOP
                    # remains active.  Requiring a non-zero plan/constraint
                    # stamp skew here incorrectly rejected this real delivery
                    # order and reset steering to zero.
                    baseline_stop_plan_authorized
                    and pp_tracking_delivery_gap_usable
                    and constraint_decision.reason
                    in (
                        "safety_constraint_applied",
                        "safety_constraint_retransmit",
                    )
                    and constraint_decision_matches_authority
                )
                or (
                    # The complementary case is a plan/constraint topic skew.
                    # It may use either the exact current PP proof or its
                    # bounded delivery-gap proof, but never authorizes motion.
                    baseline_stop_contract_delivery_gap_authorized
                    and (
                        pp_tracking_proof_usable
                        or pp_tracking_delivery_gap_usable
                    )
                )
            )
            and self.last_verified_baseline_stop_plan_generation
            is not None
            and int(self.last_verified_baseline_stop_plan_generation)
            in (
                int(authority_plan_generation) - 1,
                int(authority_plan_generation),
            )
            and self.last_verified_baseline_stop_steering_rad is not None
            and math.isfinite(self.last_verified_baseline_stop_steering_rad)
            and math.isfinite(last_verified_baseline_stop_steering_age_sec)
            and 0.0 <= last_verified_baseline_stop_steering_age_sec
            <= self.lateral_stop_steering_hold_timeout_sec
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
        )
        lateral_stop_steering_hold_usable = bool(
            maneuver_lateral_stop_steering_hold_usable
            or baseline_stop_steering_hold_usable
            or pass_warmup_acquisition_steering_hold_usable
        )
        planner_stop_release_contract_ready = bool(
            constraint_decision.stop_required
            and (
                constraint_decision_matches_authority
                or historical_authority_fault_release_bridge
            )
            and (
                (
                    lateral_stop_plan_authorized
                    and constraint_release_reason
                    in (
                        "maneuver_transaction_tracking_stop",
                        "release_pending_safe_cycles",
                    )
                )
                or (
                    planner_stop_release_bootstrap_valid
                    and planner_stop_bootstrap_pp_speed_valid
                    and constraint_release_reason
                    in (
                        "maneuver_transaction_tracking_stop",
                        "release_pending_safe_cycles",
                    )
                )
            )
        )
        historical_fault_release_clear_ready = bool(
            (
                constraint_decision.reason
                == "safety_constraint_fault_latched"
                or self.control_fault_latched
            )
            and self._authorized_pass_release_contract_valid(
                plan_generation=authority_plan_generation,
                constraint=authority_constraint,
                now_sec=now_sec,
            )
        )
        exact_safety_constraint_release_ready = bool(
            str(self.core.config.primary_source).strip().lower()
            == "pure_pursuit"
            and pp_cmd_fresh
            and pure_pursuit_cmd_valid_for_release
            and pp_tracking_proof_usable
            and authority_plan_generation is not None
            and int(tracking_plan_generation)
            == int(authority_plan_generation)
            and (
                planner_stop_release_contract_ready
                or historical_fault_release_clear_ready
            )
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not active_control_fault_reason
        )
        safety_constraint_release_ready = bool(
            exact_safety_constraint_release_ready
            or attack_follow_stop_transport_release_ready
        )
        if (
            decision_source == "pure_pursuit"
            and self.free_run_source_gap_expired_successor_identity is not None
            and not expired_successor_exact_ack
        ):
            decision_source = "stop"
            reason = "free_run_source_gap_lease_expired"
        cmd = self._select_command(
            decision_source, now_sec, selected_input_cmd)
        safety_constraint_non_stop_before_tracking_gate = bool(
            not constraint_decision.stop_required
        )
        if (
            self.safety_constraint_enabled
            and self.require_safety_constraint
            and decision_source == "pure_pursuit"
            and not constraint_decision.stop_required
            and not motion_pp_tracking_proof_usable
        ):
            # reference override, typed plan, safety constraint, PP command,
            # tracking status are separate topics.  A non-STOP pair alone must
            # not move the vehicle until the PP command proves the exact active
            # plan generation.  This also closes the raw-override-first window
            # before the typed plan/constraint peers arrive.
            constraint_decision = SafetyConstraintDecision(
                stop_required=True,
                speed_limit_mps=0.0,
                required_brake_decel_mps2=max(
                    0.0,
                    float(constraint_decision.required_brake_decel_mps2),
                ),
                constraint_generation=int(
                    constraint_decision.constraint_generation
                ),
                plan_generation=int(constraint_decision.plan_generation),
                reason="controller_tracking_generation_unverified",
            )
        if (
            pass_warmup_acquisition_steering_hold_usable
            and decision_source == "stop"
        ):
            cmd = self._stop_command(now_sec)
            cmd.lateral.steering_tire_angle = float(
                self.last_verified_lateral_stop_steering_rad
            )
            decision_source = "pure_pursuit"
            reason = "pass_warmup_acquisition_delivery_gap_hold"
        if self.safety_constraint_enabled:
            cmd, decision_source, reason = self._apply_safety_constraint(
                cmd,
                decision_source,
                reason,
                constraint_decision,
                now_sec,
                lateral_tracking_usable=(
                    lateral_stop_tracking_usable
                    or pass_warmup_steering_acquisition_usable
                ),
                lateral_hold_steering_rad=(
                    self.last_verified_lateral_stop_steering_rad
                    if (
                        maneuver_lateral_stop_steering_hold_usable
                        or pass_warmup_acquisition_steering_hold_usable
                    )
                    else (
                        self.last_verified_baseline_stop_steering_rad
                        if baseline_stop_steering_hold_usable
                        else (
                            free_run_live_record.published_steering_rad
                            if (
                                free_run_live_exact_pre_ack_hold_usable
                                or free_run_source_delivery_gap_hold_usable
                            )
                            and free_run_live_record is not None
                            else (
                                self.last_verified_baseline_free_run_steering_rad
                                if (
                                    baseline_soft_steering_hold_usable
                                    or normal_delivery_gap_zero_speed_baseline_hold_usable
                                )
                                else None
                            )
                        )
                    )
                ),
            )
            if (
                baseline_soft_steering_hold_usable
                and decision_source == "pure_pursuit"
            ):
                reason = "baseline_tracking_reserve_longitudinal_stop"
            elif (
                (
                    free_run_live_exact_pre_ack_hold_usable
                    or free_run_source_delivery_gap_hold_usable
                )
                and decision_source == "pure_pursuit"
            ):
                reason = (
                    "free_run_source_delivery_gap_zero_speed_steering_hold"
                    if free_run_source_delivery_gap_hold_usable
                    else "free_run_live_exact_pre_ack_hold"
                )
                if (
                    free_run_source_delivery_gap_hold_usable
                    and self.free_run_source_gap_lease is not None
                    and self.free_run_source_gap_last_counted_hold_identity
                    != (
                        int(
                            self.free_run_source_gap_lease.provenance_record.race_arm_epoch
                        ),
                        *self.free_run_source_gap_lease.successor_source_identity,
                    )
                ):
                    self.free_run_source_gap_hold_episode_count = min(
                        self.free_run_source_gap_hold_episode_count + 1,
                        (1 << 64) - 1,
                    )
                    self.free_run_source_gap_last_counted_hold_identity = (
                        int(
                            self.free_run_source_gap_lease.provenance_record.race_arm_epoch
                        ),
                        *self.free_run_source_gap_lease.successor_source_identity,
                    )
            elif (
                normal_delivery_gap_zero_speed_baseline_hold_usable
                and decision_source == "pure_pursuit"
            ):
                reason = "normal_delivery_gap_zero_speed_steering_hold"
        if (
            decision_source == "pure_pursuit"
            and pure_pursuit_command_exceeds_tracking_steering_limit
            and not (
                baseline_soft_steering_hold_usable
                or free_run_live_exact_pre_ack_hold_usable
                or free_run_source_delivery_gap_hold_usable
                or normal_delivery_gap_zero_speed_baseline_hold_usable
            )
        ):
            # A hard clamp only makes the command finite; it cannot prove that
            # the authorized trajectory is physically trackable.  Close the
            # final actuator authority independently of planner-constraint
            # configuration so legacy/diagnostic modes cannot move laterally
            # or accelerate on a tracking-unusable Pure Pursuit command.
            # Finish/watchdog/E-stop authorities have already selected a
            # stronger non-PP source and must not be overwritten here.
            cmd = self._stop_command(now_sec)
            decision_source = "stop"
            reason = "steering_command_exceeds_actuator_limit"
        steering_result = None
        if motion_authority_delivery_gap_lease_active:
            steering_result = (
                self._prepare_motion_authority_delivery_gap_replay(
                    command=cmd,
                    source=decision_source,
                    now_sec=now_sec,
                )
            )
        if steering_result is None:
            steering_result = self._limit_steering(
                cmd, decision_source, now_sec
            )
        if (
            decision_source == "pure_pursuit"
            and self.free_run_live_exact_observe_enabled
            and self.free_run_live_exact_selected_envelope_valid
            and selected_free_run_envelope is not None
            and not free_run_live_exact_pre_ack_hold_usable
            and not free_run_source_delivery_gap_hold_usable
            and (
                steering_result.angle_limited
                or steering_result.rate_limited
            )
        ):
            # The PP proof binds the command before the Mux actuator limiter.
            # If the final limiter changes that command, the exact FREE_RUN
            # execution tuple no longer proves the command that would move the
            # vehicle.  Keep the independent hard/rate clamp, but fail closed
            # instead of granting longitudinal motion to a different command.
            self._invalidate_free_run_live_exact_record(
                "free_run_final_limiter_mismatch"
            )
            cmd = self._stop_command(now_sec)
            decision_source = "stop"
            reason = "free_run_live_exact_final_limiter_rejected"
            steering_result = self._limit_steering(
                cmd, decision_source, now_sec
            )
        if (
            (
                free_run_live_exact_pre_ack_hold_usable
                or free_run_source_delivery_gap_hold_usable
                or normal_delivery_gap_zero_speed_baseline_hold_usable
            )
            and free_run_live_record is not None
            and (
                decision_source != "pure_pursuit"
                or not math.isclose(
                    float(steering_result.limited_steering_rad),
                    float(free_run_live_record.published_steering_rad),
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                )
                or steering_result.angle_limited
                or steering_result.rate_limited
            )
        ):
            self._invalidate_free_run_live_exact_record(
                "pre_ack_hold_final_limiter_mismatch"
            )
            cmd = self._stop_command(now_sec)
            decision_source = "stop"
            reason = "free_run_live_exact_pre_ack_hold_limiter_rejected"
            steering_result = self._limit_steering(
                cmd, decision_source, now_sec
            )
            free_run_live_exact_pre_ack_hold_usable = False
            free_run_source_delivery_gap_hold_usable = False
            normal_delivery_gap_zero_speed_baseline_hold_usable = False
        active_plan_identity = self.overtake_plan_motion_identity_cache.get(
            int(tracking_plan_generation)
        )
        pass_motion_context = bool(
            active_plan_identity is not None
            and int(active_plan_identity[8]) == int(OvertakePlan.PASSING)
        )
        positive_motion_requested = bool(
            float(cmd.longitudinal.speed) > 0.0
            or float(cmd.longitudinal.acceleration) > 0.0
        )
        pass_motion_grant_required = bool(
            pass_motion_context and positive_motion_requested
        )
        pass_motion_grant_valid = False
        pass_motion_grant_reason = "not_required"
        pass_motion_grant_envelope = None
        pass_motion_grant_identity = active_plan_identity
        motion_authority_grant_published = False
        motion_authority_grant_v2_published = False
        motion_authority_grant_published_valid = False
        motion_authority_grant_published_reason = ""
        pass_motion_grant_commit_steady_ns = -1
        if pass_motion_grant_required:
            grant_tracking_plan_generation = int(tracking_plan_generation)
            grant_authority_plan_generation = authority_plan_generation
            grant_authority_constraint = authority_constraint
            if (
                motion_pp_tracking_continuity_usable
                and selected_pp_motion_sample is not None
                and selected_pp_motion_sample.authority_proof is not None
            ):
                # The selected command remains the older fully coherent
                # Plan/Constraint/PP tuple.  The continuity predicate above
                # has independently required the latest exact released
                # successor as an additional safety guard; do not relabel the
                # older envelope as that newer generation.
                continuity_generation = int(
                    selected_pp_motion_sample.authority_proof.plan_generation
                )
                continuity_constraint_entry = (
                    self.safety_constraint_cache.get(continuity_generation)
                )
                if continuity_constraint_entry is not None:
                    grant_tracking_plan_generation = continuity_generation
                    grant_authority_plan_generation = continuity_generation
                    grant_authority_constraint = (
                        continuity_constraint_entry[0]
                    )
            (
                pass_motion_grant_valid,
                pass_motion_grant_reason,
                pass_motion_grant_envelope,
                pass_motion_grant_identity,
            ) = self._motion_authority_grant_eligible(
                final_command=cmd,
                selected_sample=selected_pp_motion_sample,
                tracking_plan_generation=grant_tracking_plan_generation,
                authority_plan_generation=grant_authority_plan_generation,
                authority_constraint=grant_authority_constraint,
                constraint_decision=constraint_decision,
                decision_source=decision_source,
                now_sec=now_sec,
                active_control_fault_reason=active_control_fault_reason,
                ros_clock_stalled=ros_clock_watchdog.stalled,
                deadline_missed=watchdog.deadline_missed,
                steering_result=steering_result,
                delivery_gap_lease_active=motion_authority_delivery_gap_lease_active,
            )
            if not pass_motion_grant_valid:
                cmd = self._stop_command(now_sec)
                decision_source = "stop"
                reason = (
                    "motion_authority_grant_" + pass_motion_grant_reason
                )
                steering_result = self._limit_steering(
                    cmd, decision_source, now_sec
                )
                grant_was_active = bool(self.motion_authority_grant_active)
                self._publish_motion_authority_grant(
                    valid=False,
                    reason=pass_motion_grant_reason,
                )
                motion_authority_grant_published = True
                motion_authority_grant_v2_published = grant_was_active
                motion_authority_grant_published_reason = str(
                    pass_motion_grant_reason
                )
        elif self.motion_authority_grant_active:
            self._publish_motion_authority_grant(
                valid=False,
                reason="positive_pass_motion_not_requested",
            )
            motion_authority_grant_published = True
            motion_authority_grant_v2_published = True
            motion_authority_grant_published_reason = (
                "positive_pass_motion_not_requested"
            )
        if (
            decision_source == "pure_pursuit"
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_tracking_steering_limit
            and (
                (
                    pp_tracking_proof_usable
                    and lateral_stop_tracking_usable
                )
                or pass_warmup_steering_acquisition_usable
            )
            and math.isfinite(steering_result.limited_steering_rad)
            and not self.external_stop_latched
            and not self.control_fault_latched
            and not active_control_fault_reason
        ):
            if pp_tracking_proof_usable and lateral_stop_tracking_usable:
                self.last_verified_tracking_steering_rad = (
                    steering_result.limited_steering_rad
                )
                self.last_verified_tracking_steering_time_sec = now_sec
            if (
                maneuver_lateral_stop_tracking_usable
                or pass_warmup_steering_acquisition_usable
            ):
                self.last_verified_lateral_stop_steering_rad = (
                    steering_result.limited_steering_rad
                )
                self.last_verified_lateral_stop_steering_time_sec = now_sec
                self.last_verified_lateral_stop_plan_generation = int(
                    authority_plan_generation
                )
            if baseline_stop_tracking_usable:
                self.last_verified_baseline_stop_steering_rad = (
                    steering_result.limited_steering_rad
                )
                self.last_verified_baseline_stop_steering_time_sec = now_sec
                self.last_verified_baseline_stop_plan_generation = int(
                    authority_plan_generation
                )
        self._remember_finish_terminal_reference(
            cmd,
            decision_source,
            now_sec,
            active_control_fault_reason=active_control_fault_reason,
            ros_clock_stalled=ros_clock_watchdog.stalled,
            deadline_missed=watchdog.deadline_missed,
            steering_result=steering_result,
        )
        if pass_motion_grant_valid:
            # The typed message is observability for an internal commit. It is
            # deliberately published before the exact bound positive command,
            # but is never consumed back into Mux as authority.
            if (
                not motion_authority_delivery_gap_lease_active
                and pass_motion_grant_envelope is not None
                and pass_motion_grant_identity is not None
                and grant_authority_constraint is not None
            ):
                # Mint only after all exact checks, including final limiter
                # binding, succeeded.  The helper preserves acquisition time
                # for a timer-republished identical commit.
                self._commit_motion_authority_delivery_gap_lease(
                    plan_identity=pass_motion_grant_identity,
                    constraint=grant_authority_constraint,
                    envelope=pass_motion_grant_envelope,
                    final_command=cmd,
                    now_sec=now_sec,
                    selected_sample=selected_pp_motion_sample,
                    steering_result=steering_result,
                )
            self._publish_motion_authority_grant(
                valid=True,
                reason="committed",
                envelope=pass_motion_grant_envelope,
                plan_identity=pass_motion_grant_identity,
                constraint=grant_authority_constraint,
                final_command=cmd,
            )
            if self.mux_runtime_measurement_enabled:
                pass_motion_grant_commit_steady_ns = time.clock_gettime_ns(
                    time.CLOCK_MONOTONIC
                )
            motion_authority_grant_published = True
            motion_authority_grant_v2_published = True
            motion_authority_grant_published_valid = True
            motion_authority_grant_published_reason = "committed"
        self.control_pub.publish(cmd)
        self.last_published_control_command = copy.deepcopy(cmd)
        self.last_published_control_source = str(decision_source)
        self.last_published_control_steady_time_sec = float(now_sec)
        if (
            pass_motion_grant_valid
            and not motion_authority_delivery_gap_lease_active
            and pass_motion_grant_envelope is not None
            and pass_motion_grant_identity is not None
            and grant_authority_constraint is not None
        ):
            # Publication is the ownership boundary.  A newer fully verified
            # same-generation PP command may replace only the frozen command
            # snapshot/provenance; the original lease and gap clocks remain
            # immutable.
            self._ratchet_motion_authority_delivery_gap_lease_after_publish(
                plan_identity=pass_motion_grant_identity,
                constraint=grant_authority_constraint,
                envelope=pass_motion_grant_envelope,
                final_command=cmd,
                steering_result=steering_result,
            )
        if self.latest_sample_observability_enabled:
            self._mux_latest_sample_cycle_context = (
                selector_watermark_identity,
                selector_evaluated_identity,
                (
                    "selected"
                    if selector_evaluated_identity is not None
                    else "nonqualifying"
                ),
                str(decision_source),
                str(reason),
                bool(watchdog.deadline_missed),
                bool(ros_clock_watchdog.stalled),
            )
        if self.mux_runtime_measurement_enabled:
            selected_plan_receipt_time_sec = None
            if tracking_plan_generation is not None:
                selected_plan_record = self.overtake_plan_cache.get(
                    int(tracking_plan_generation)
                )
                if selected_plan_record is not None:
                    selected_plan_receipt_time_sec = float(
                        selected_plan_record[1]
                    )
            selected_envelope_receipt_time_sec = None
            if (
                selected_pp_motion_sample is not None
                and selected_pp_motion_sample.envelope_identity is not None
            ):
                selected_envelope_record = (
                    self.pure_pursuit_envelope_cache.get(
                        tuple(selected_pp_motion_sample.envelope_identity)
                    )
                )
                if selected_envelope_record is not None:
                    selected_envelope_receipt_time_sec = float(
                        selected_envelope_record[1]
                    )
            selected_plan_sample_identity = (
                pass_motion_grant_identity if pass_motion_grant_valid else None
            )
            self.mux_runtime_measurement_pending_cycle = {
                "watermark_identity_at_eval": selector_watermark_identity,
                "evaluated_identity": selector_evaluated_identity,
                "selector_disposition": (
                    "selected" if selector_evaluated_identity is not None
                    else "nonqualifying"
                ),
                "final_source": str(decision_source),
                "final_reason": str(reason),
                "exact_available": bool(
                    pass_motion_grant_valid
                    and decision_source == "pure_pursuit"
                    and motion_pp_tracking_proof_first_false == "none"
                ),
                "motion_first_false": str(
                    motion_pp_tracking_proof_first_false
                ),
                "source": str(decision_source),
                "reason": str(reason),
                "authority_generation": (
                    int(authority_plan_generation)
                    if authority_plan_generation is not None
                    else -1
                ),
                "tracking_generation": (
                    int(tracking_plan_generation)
                    if tracking_plan_generation is not None
                    else -1
                ),
                "race_armed": bool(self.finish_stop_latch.armed),
                "ros_clock_motion_ready": bool(
                    self.ros_clock_motion_ready
                ),
                "ros_clock_stalled": bool(ros_clock_watchdog.stalled),
                "deadline_missed": bool(watchdog.deadline_missed),
                "control_fault_latched": bool(
                    self.control_fault_latched
                ),
                "control_fault_reason": str(self.control_fault_reason),
                "grant_required": bool(pass_motion_grant_required),
                "grant_eligible": bool(pass_motion_grant_valid),
                "grant_published": bool(
                    motion_authority_grant_published
                ),
                "grant_valid": bool(
                    motion_authority_grant_published_valid
                ),
                "grant_reason": str(
                    motion_authority_grant_published_reason
                ),
                "grant_sequence": int(
                    self.motion_authority_grant_sequence
                ),
                "grant_commit_steady_ns": int(
                    pass_motion_grant_commit_steady_ns
                ),
                "command": [
                    float(cmd.longitudinal.speed),
                    float(cmd.longitudinal.acceleration),
                    float(cmd.lateral.steering_tire_angle),
                    float(cmd.lateral.steering_tire_rotation_rate),
                ],
                "grant_command": (
                    [
                        float(cmd.longitudinal.speed),
                        float(cmd.longitudinal.acceleration),
                        float(cmd.lateral.steering_tire_angle),
                        float(
                            cmd.lateral.steering_tire_rotation_rate
                        ),
                    ]
                    if motion_authority_grant_published_valid
                    else []
                ),
                "timer_snapshot_steady_ns": (
                    int(round(now_sec * 1.0e9))
                    + measurement_steady_offset_ns
                ),
                "ros_steady_to_monotonic_offset_ns": int(
                    measurement_steady_offset_ns
                ),
                "input_receipt_steady_ns": {
                    "overtake_plan": (
                        int(
                            round(
                                selected_plan_receipt_time_sec * 1.0e9
                            )
                        )
                        + measurement_steady_offset_ns
                        if selected_plan_receipt_time_sec is not None
                        else -1
                    ),
                    "safety_constraint": (
                        int(round(authority_constraint_time_sec * 1.0e9))
                        + measurement_steady_offset_ns
                        if authority_constraint_time_sec is not None
                        else -1
                    ),
                    "pure_pursuit_command": (
                        int(
                            round(
                                selected_pp_motion_sample.command_receipt_time_sec
                                * 1.0e9
                            )
                        )
                        + measurement_steady_offset_ns
                        if selected_pp_motion_sample is not None
                        else -1
                    ),
                    "pure_pursuit_tracking": (
                        int(
                            round(
                                selected_pp_motion_sample.authority_proof_receipt_time_sec
                                * 1.0e9
                            )
                        )
                        + measurement_steady_offset_ns
                        if selected_pp_motion_sample is not None
                        and selected_pp_motion_sample.authority_proof_receipt_time_sec
                        is not None
                        else -1
                    ),
                    "pure_pursuit_envelope": (
                        int(
                            round(
                                selected_envelope_receipt_time_sec * 1.0e9
                            )
                        )
                        + measurement_steady_offset_ns
                        if selected_envelope_receipt_time_sec is not None
                        else -1
                    ),
                    "free_run_ack": (
                        int(
                            round(
                                self.free_run_live_exact_ack_time_sec * 1.0e9
                            )
                        )
                        + measurement_steady_offset_ns
                        if self.free_run_live_exact_ack_time_sec is not None
                        else -1
                    ),
                    "free_run_source_key": (
                        int(
                            round(
                                self.free_run_live_exact_source_key_time_sec
                                * 1.0e9
                            )
                        )
                        + measurement_steady_offset_ns
                        if self.free_run_live_exact_source_key_time_sec
                        is not None
                        else -1
                    ),
                },
                **self._mux_runtime_cycle_record_v2_fields(
                    selected_input_cmd=selected_input_cmd,
                    selected_pp_motion_sample=selected_pp_motion_sample,
                    selected_plan_identity=selected_plan_sample_identity,
                    grant_published=motion_authority_grant_v2_published,
                    grant_valid=motion_authority_grant_published_valid,
                    grant_sequence=(
                        self.motion_authority_grant_sequence
                        if motion_authority_grant_v2_published
                        else None
                    ),
                    grant_commit_steady_ns=(
                        pass_motion_grant_commit_steady_ns
                        if motion_authority_grant_published_valid
                        else None
                    ),
                    grant_issuer_instance_id=(
                        self.motion_authority_grant_issuer_instance_id
                        if motion_authority_grant_v2_published
                        else None
                    ),
                    final_command=cmd,
                    steering_result=steering_result,
                    steering_limiter_config=self.steering_limiter.config,
                ),
            }
        if (
            self.free_run_live_exact_observe_enabled
            and self.free_run_live_exact_selected_envelope_valid
            and selected_free_run_envelope is not None
            and self.free_run_live_exact_ack is not None
            and self.free_run_live_exact_ack_identity is not None
            and self.free_run_live_exact_source_key_identity is not None
            and decision_source == "pure_pursuit"
            and not constraint_decision.stop_required
            and baseline_free_run_cache_contract_current
            and pure_pursuit_command_within_tracking_steering_limit
            and math.isfinite(steering_result.limited_steering_rad)
            and abs(steering_result.limited_steering_rad)
            <= tracking_usable_steering_limit_rad
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
            and not watchdog.deadline_missed
        ):
            envelope_identity = tuple(
                self.free_run_live_exact_selected_envelope_identity or ()
            )
            previous_record = self.free_run_live_exact_published_record
            if (
                len(envelope_identity) == 4
                and (
                    previous_record is None
                    or previous_record.envelope_identity
                    != envelope_identity
                    or previous_record.source_identity
                    != tuple(self.free_run_live_exact_source_key_identity)
                    or previous_record.ack_identity
                    != tuple(self.free_run_live_exact_ack_identity)
                )
            ):
                ack = self.free_run_live_exact_ack
                source_identity = tuple(
                    self.free_run_live_exact_source_key_identity
                )
                source_wire_digest = (
                    self.free_run_live_exact_source_wire_digest_cache.get(
                        source_identity
                    )
                )
                ack_identity = tuple(self.free_run_live_exact_ack_identity)
                ack_wire_digest = (
                    self.free_run_live_exact_ack_wire_digest_cache.get(
                        ack_identity
                    )
                )
                final_command_cdr = bytes(serialize_message(cmd))
                source_semantic_identity = (
                    self._free_run_source_semantic_identity(
                        self.free_run_live_exact_source_key
                    )
                    if self.free_run_live_exact_source_key is not None
                    else None
                )
                pending_source_matches = bool(
                    self.free_run_pending_source is None
                    or (
                        self.free_run_pending_source.identity
                        == source_identity
                        and self.free_run_pending_source.semantic_identity
                        == source_semantic_identity
                        and self.free_run_pending_source.wire_sha256
                        == source_wire_digest
                    )
                )
                if (
                    source_wire_digest is not None
                    and ack_wire_digest is not None
                    and len(source_identity) == 4
                    and source_semantic_identity is not None
                    and pending_source_matches
                ):
                    self.free_run_live_exact_published_record = (
                        FreeRunPublishedCommandRecord(
                            publish_steady_time_sec=now_sec,
                            race_arm_epoch=int(
                                ack.plan_key.race_arm_epoch
                            ),
                            planner_instance_id=int(
                                ack.plan_key.planner_instance_id
                            ),
                            plan_generation=int(
                                ack.plan_key.plan_generation
                            ),
                            plan_stamp_ns=self._stamp_ns(
                                ack.plan_key.plan_stamp
                            ),
                            canonical_plan_sha256=bytes(
                                ack.plan_key.canonical_plan_payload_sha256
                            ),
                            source_identity=source_identity,
                            source_semantic_identity=(
                                source_semantic_identity
                            ),
                            source_wire_sha256=bytes(source_wire_digest),
                            envelope_identity=envelope_identity,
                            envelope_command_sha256=(
                                self._canonical_controller_command_digest(
                                    selected_free_run_envelope.command
                                )
                            ),
                            ack_identity=ack_identity,
                            ack_wire_sha256=bytes(ack_wire_digest),
                            published_steering_rad=float(
                                cmd.lateral.steering_tire_angle
                            ),
                            tracking_soft_limit_rad=float(
                                tracking_usable_steering_limit_rad
                            ),
                            actuator_hard_limit_rad=float(
                                self.steering_limiter.config.max_steering_angle_rad
                            ),
                            final_command_cdr=final_command_cdr,
                            final_command_sha256=hashlib.sha256(
                                final_command_cdr
                            ).digest(),
                        )
                    )
                    # An exact current ACK plus the final current gates has
                    # replaced the immutable provenance.  No old gap lease
                    # may survive that atomic post-publish commit.
                    if self.free_run_source_gap_lease is not None:
                        self.free_run_source_gap_exact_promotion_count = min(
                            self.free_run_source_gap_exact_promotion_count + 1,
                            (1 << 64) - 1,
                        )
                    self.free_run_pending_source = None
                    self.free_run_source_gap_lease = None
                    self.free_run_source_gap_expired_successor_identity = None
                    self.free_run_live_exact_record_reason = (
                        "exact_post_publish"
                    )
        if (
            decision_source == "pure_pursuit"
            and not constraint_decision.stop_required
            and baseline_free_run_cache_contract_current
            and pure_pursuit_command_within_tracking_steering_limit
            and math.isfinite(steering_result.limited_steering_rad)
            and abs(steering_result.limited_steering_rad)
            <= tracking_usable_steering_limit_rad
            and not self.external_stop_latched
            and not self.finish_stop_latch.latched
            and not self.control_fault_latched
            and not active_control_fault_reason
            and not ros_clock_watchdog.stalled
            and not watchdog.deadline_missed
        ):
            # Store only an already-published, exact FREE_RUN command that
            # remained inside the conservative tracking reserve.  An
            # over-limit sample can consume this bounded reference but can
            # never refresh it.
            self.last_verified_baseline_free_run_steering_rad = (
                steering_result.limited_steering_rad
            )
            self.last_verified_baseline_free_run_steering_time_sec = now_sec
            self.last_verified_baseline_free_run_plan_generation = int(
                authority_plan_generation
            )
            self.last_verified_baseline_free_run_plan_stamp_ns = int(
                self.overtake_plan_header_stamp_ns
            )
            self.last_verified_baseline_free_run_epoch = int(
                self.finish_stop_latch.epoch
            )
        tracking_status = ControllerTrackingStatus()
        tracking_status.header.stamp = (
            observed_pp_tracking_status.header.stamp
            if observed_pp_tracking_status is not None
            else self.get_clock().now().to_msg()
        )
        tracking_status.header.frame_id = "base_link"
        tracking_status.plan_generation = int(
            observed_pp_tracking_status.plan_generation
            if observed_pp_tracking_status_fresh
            and observed_pp_tracking_status_valid
            and observed_pp_tracking_status is not None
            else 0
        )
        tracking_status.mpc_horizon_usable = bool(
            observed_pp_tracking_status_fresh
            and observed_pp_tracking_status is not None
            and observed_pp_tracking_status.mpc_horizon_usable
        )
        tracking_status.pp_command_fresh = bool(pp_cmd_fresh)
        tracking_status.trajectory_tracking_usable = bool(
            str(self.core.config.primary_source).strip().lower() == "pure_pursuit"
            and decision_source == "pure_pursuit"
            and not constraint_decision.stop_required
            and pp_cmd_fresh
            and selected_pp_command_valid
            and pure_pursuit_command_within_tracking_steering_limit
            and motion_pp_tracking_proof_usable
            and (
                not pass_motion_grant_required
                or pass_motion_grant_valid
            )
        )
        tracking_status.safety_constraint_release_ready = bool(
            safety_constraint_release_ready
        )
        tracking_status.attack_follow_stop_transport_release_ready = bool(
            attack_follow_stop_transport_release_ready
        )
        pass_probe_transport_evidence = (
            safety_authority_selection.rendezvous_state
        )
        if "same_generation_conflict" in str(constraint_decision.reason):
            pass_probe_transport_evidence = (
                SafetyAuthorityRendezvousState.PAYLOAD_MUTATION
            )
        elif any(
            token in str(constraint_decision.reason)
            for token in ("regression", "generation_mismatch", "fault_latched")
        ):
            pass_probe_transport_evidence = (
                SafetyAuthorityRendezvousState.REGRESSION_OR_GAP
            )
        elif (
            pp_tracking_delivery_gap_usable
            and safety_authority_selection.rendezvous_state
            == SafetyAuthorityRendezvousState.EXACT_CURRENT
            and safety_constraint_non_stop_before_tracking_gate
            and not active_control_fault_reason
        ):
            # The Planner/SafetyConstraint exact N pair can arrive before PP's
            # atomic envelope N.  Keep final motion stopped, but classify the
            # fresh/valid N-1 proof as a bounded normal delivery gap so Planner
            # pauses an already-earned PASS probe instead of resetting it.
            # This does not promote N-1 to motion authority.
            pass_probe_transport_evidence = (
                SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
            )
        if (
            self.external_stop_latched
            or self.finish_stop_latch.latched
            or self.control_fault_latched
            or bool(active_control_fault_reason)
            or ros_clock_watchdog.stalled
            or watchdog.deadline_missed
        ):
            pass_probe_transport_evidence = (
                SafetyAuthorityRendezvousState.REGRESSION_OR_GAP
            )
        tracking_status.pass_probe_transport_evidence = int(
            pass_probe_transport_evidence
        )
        tracking_status.pass_probe_exact_current_usable = bool(
            pass_probe_exact_current_usable
        )
        tracking_status.pass_probe_lateral_stop_authority_token = int(
            pass_probe_lateral_stop_authority_token
            if pass_probe_exact_current_usable
            else 0
        )
        tracking_status.command_age_sec = float(
            now_sec - selected_pp_motion_sample.command_receipt_time_sec
            if selected_pp_motion_sample is not None
            else float("inf")
        )
        if decision_source == "finish_stop":
            # A terminal snapshot must never become a PASS/ACK proof.
            tracking_status.plan_generation = 0
            tracking_status.trajectory_tracking_usable = False
            tracking_status.safety_constraint_release_ready = False
            tracking_status.pass_probe_exact_current_usable = False
            tracking_status.pass_probe_lateral_stop_authority_token = 0
            tracking_status.attack_follow_stop_transport_release_ready = False
            tracking_status.reason = "official_vehicle_finish"
        primary_is_pure_pursuit = (
            str(self.core.config.primary_source).strip().lower() == "pure_pursuit"
        )
        upstream_tracking_reason = (
            str(observed_pp_tracking_status.reason).strip()
            if observed_pp_tracking_status is not None
            and observed_pp_tracking_status_fresh
            else ""
        )
        if decision_source == "finish_stop":
            tracking_status.reason = "official_vehicle_finish"
        elif attack_follow_stop_transport_release_ready:
            tracking_status.reason = (
                "attack_follow_stop_transport_continuity"
            )
        elif (
            tracking_status.trajectory_tracking_usable
            and attack_follow_transport_continuity_usable
            and not pp_tracking_proof_usable
        ):
            tracking_status.reason = "attack_follow_transport_continuity"
        elif (
            tracking_status.trajectory_tracking_usable
            and pass_transport_continuity_usable
            and not pp_tracking_proof_usable
        ):
            tracking_status.reason = "pass_transport_continuity"
        elif tracking_status.trajectory_tracking_usable:
            tracking_status.reason = "ready"
        elif not primary_is_pure_pursuit:
            tracking_status.reason = "primary_not_pure_pursuit"
        elif free_run_source_delivery_gap_hold_usable:
            tracking_status.reason = (
                "free_run_source_delivery_gap_zero_speed_steering_hold"
            )
        elif free_run_live_exact_pre_ack_hold_usable:
            tracking_status.reason = "free_run_live_exact_pre_ack_hold"
        elif normal_delivery_gap_zero_speed_baseline_hold_usable:
            tracking_status.reason = (
                "normal_delivery_gap_zero_speed_steering_hold"
            )
        elif baseline_soft_steering_hold_usable:
            tracking_status.reason = (
                "baseline_tracking_reserve_longitudinal_stop"
            )
        elif pure_pursuit_command_exceeds_tracking_steering_limit:
            tracking_status.reason = (
                "steering_command_exceeds_actuator_limit"
            )
        elif not tracking_usable_steering_limit_valid:
            tracking_status.reason = "actuator_steering_limit_invalid"
        elif decision_source != "pure_pursuit":
            tracking_status.reason = "final_source_not_pure_pursuit"
        elif constraint_decision.stop_required:
            tracking_status.reason = (
                "pass_warmup_zero_speed_steering_acquisition"
                if pass_warmup_steering_acquisition_usable
                else (
                    "longitudinal_safety_stop_with_lateral_tracking"
                    if lateral_stop_tracking_usable
                    else (
                        "longitudinal_safety_stop_with_lateral_hold"
                        if lateral_stop_steering_hold_usable
                        else "safety_constraint_stop"
                    )
                )
            )
        elif pp_tracking_status is None:
            tracking_status.reason = (
                "command_stamp_mismatch"
                if observed_pp_tracking_status_fresh
                else "pure_pursuit_tracking_missing"
            )
        elif not pp_cmd_fresh:
            tracking_status.reason = (
                upstream_tracking_reason or "pure_pursuit_command_stale"
            )
        elif not selected_pp_command_valid:
            tracking_status.reason = (
                upstream_tracking_reason or "selected_command_nonfinite"
            )
        elif not pp_tracking_status_fresh:
            tracking_status.reason = "pure_pursuit_tracking_stale"
        elif not pp_tracking_status_valid:
            tracking_status.reason = (
                upstream_tracking_reason or "pure_pursuit_tracking_invalid"
            )
        elif tracking_plan_generation < 0:
            tracking_status.reason = "active_plan_missing_or_stale"
        elif int(pp_tracking_status.plan_generation) != int(tracking_plan_generation):
            tracking_status.reason = "plan_generation_mismatch"
        elif not bool(pp_tracking_status.trajectory_tracking_usable):
            tracking_status.reason = (
                str(pp_tracking_status.reason).strip()
                or "pure_pursuit_tracking_unusable"
            )
        else:
            tracking_status.reason = "tracking_unusable"
        self.tracking_status_pub.publish(tracking_status)
        self._publish_debug(
            now_sec=now_sec,
            source=decision_source,
            reason=reason,
            fallback_active=fallback_active,
            solved_cycles=solved_cycles,
            mpc_cmd_fresh=mpc_cmd_fresh,
            pure_pursuit_cmd_fresh=pp_cmd_fresh,
            health=health,
            recovery_state=recovery_state,
            selected_input_cmd=selected_input_cmd,
            selected_pp_motion_sample=selected_pp_motion_sample,
            pp_selection_debug_snapshot=(
                dict(pp_selection_debug_rejections[-1])
                if pp_selection_debug_rejections
                else None
            ),
            output_cmd=cmd,
            steering_result=steering_result,
            constraint_decision=constraint_decision,
            control_loop_deadline_missed=watchdog.deadline_missed,
            control_publish_gap_sec=watchdog.publish_gap_sec,
            ros_clock_stalled=ros_clock_watchdog.stalled,
            ros_clock_stagnant_duration_sec=(
                ros_clock_watchdog.stagnant_duration_sec
            ),
            control_fault_latched=self.control_fault_latched,
            control_fault_clear_cycles=self.control_fault_clear_cycles,
            overtake_plan_fresh=plan_fresh,
            overtake_plan_valid=self.overtake_plan_valid,
            authority_plan_generation=authority_plan_generation,
            tracking_plan_generation=tracking_plan_generation,
            lateral_stop_plan_authorized=lateral_stop_plan_authorized,
            baseline_stop_plan_authorized=baseline_stop_plan_authorized,
            planner_stop_release_bootstrap_valid=(
                planner_stop_release_bootstrap_valid
            ),
            lateral_stop_tracking_usable=lateral_stop_tracking_usable,
            baseline_stop_tracking_usable=baseline_stop_tracking_usable,
            lateral_stop_steering_hold_usable=(
                lateral_stop_steering_hold_usable
            ),
            baseline_stop_steering_hold_usable=(
                baseline_stop_steering_hold_usable
            ),
            lateral_stop_contract_delivery_gap_authorized=(
                lateral_stop_contract_delivery_gap_authorized
            ),
            pp_tracking_delivery_gap_usable=(
                pp_tracking_delivery_gap_usable
            ),
            historical_authority_release_lateral_bridge=(
                historical_authority_release_lateral_bridge
            ),
            last_verified_tracking_steering_age_sec=(
                last_verified_tracking_steering_age_sec
            ),
            controller_tracking_status=tracking_status,
            pure_pursuit_tracking_status_fresh=pp_tracking_status_fresh,
            motion_pp_tracking_proof_usable=(
                motion_pp_tracking_proof_usable
            ),
            motion_pp_tracking_proof_first_false=(
                motion_pp_tracking_proof_first_false
            ),
            motion_pp_tracking_proof_blockers=(
                motion_pp_tracking_proof_blockers
            ),
            motion_pp_tracking_exact_tuple_present=(
                matched_current_tracking_entry is not None
            ),
            motion_pp_tracking_expected_stamp_ns=(
                selected_pp_motion_sample.command_stamp_ns
                if selected_pp_motion_sample is not None
                else None
            ),
            motion_pp_tracking_expected_generation=(
                tracking_plan_generation
            ),
            motion_pp_tracking_observed_stamp_ns=(
                self._stamp_ns(exact_tracking_status.header.stamp)
                if exact_tracking_status is not None
                else None
            ),
            motion_pp_tracking_observed_generation=(
                int(exact_tracking_status.plan_generation)
                if exact_tracking_status is not None
                else None
            ),
            motion_pp_tracking_same_stamp_generations=(
                same_stamp_tracking_generations
            ),
            motion_pp_tracking_exact_status_fresh=(
                exact_tracking_status_fresh
            ),
            motion_pp_tracking_exact_status_valid=(
                exact_tracking_status_valid
            ),
            motion_pp_tracking_pp_command_fresh=pp_cmd_fresh,
            motion_pp_tracking_upstream_pp_command_fresh=(
                bool(
                    exact_tracking_status is not None
                    and exact_tracking_status.pp_command_fresh
                )
            ),
            motion_pp_tracking_upstream_trajectory_usable=(
                bool(
                    exact_tracking_status is not None
                    and exact_tracking_status.trajectory_tracking_usable
                )
            ),
            motion_pp_tracking_binding_required=(
                exact_tracking_binding_required
            ),
            motion_pp_tracking_binding_matches=(
                exact_tracking_binding_matches
            ),
            motion_pp_tracking_steering_usable=(
                pure_pursuit_command_within_tracking_steering_limit
            ),
            motion_pp_tracking_continuity_applicable=(
                motion_pp_tracking_continuity_applicable
            ),
            motion_pp_tracking_continuity_usable=(
                motion_pp_tracking_continuity_usable
            ),
            motion_pp_tracking_attack_follow_continuity_usable=(
                attack_follow_transport_continuity_usable
            ),
            motion_pp_tracking_pass_continuity_usable=(
                pass_transport_continuity_usable
            ),
            force_publish=force_motion_pp_tracking_debug,
        )

        if decision_source != self.last_source:
            self.get_logger().warn(
                f"hybrid control source={decision_source} reason={reason} "
                f"mpc_status={health.status} mpc_infeasible={health.infeasible_count}"
            )
            self.last_source = decision_source

    def _select_safety_authority_contract(
        self,
        now_sec: float,
        *,
        tracking_plan_generation: int,
    ) -> SafetyAuthoritySelection:
        """Select one exact plan/constraint generation without hiding restrictions.

        The planner publishes the typed plan and safety constraint on separate
        topics.  A timer tick may therefore observe plan N+1 with constraint N.
        A fresh, fully matched generation may bridge only that transport gap.
        Invalid, regressed, STOP, and strictly tighter constraints bypass the
        rendezvous and are evaluated immediately.  A relaxation is never
        authorized until its matching valid plan is present.
        """
        self.safety_authority_contract_unpaired = False
        if not self.require_safety_constraint:
            return SafetyAuthoritySelection(
                self.safety_constraint,
                self.safety_constraint_time_sec,
                None,
                SafetyAuthorityRendezvousState.UNKNOWN,
            )

        latest_plan_fresh = self._fresh(
            self.overtake_plan_time_sec,
            now_sec,
            self.overtake_plan_timeout_sec,
        )
        latest_constraint_fresh = self._fresh(
            self.safety_constraint_time_sec,
            now_sec,
            self.safety_constraint_timeout_sec,
        )
        latest_plan_invalid = bool(
            latest_plan_fresh and not self.overtake_plan_valid
        )
        cached_plan_generations = [
            int(key[0]) for key in self.overtake_plan_contract_cache
        ]
        cached_constraint_plan_generations = [
            int(key[0]) for key in self.safety_constraint_contract_cache
        ]
        cached_constraint_generations = [
            int(entry[0].constraint_generation)
            for entry in self.safety_constraint_contract_cache.values()
        ]
        latest_constraint = self.safety_constraint
        latest_constraint_generation = (
            int(latest_constraint.plan_generation)
            if latest_constraint is not None
            else 0
        )
        latest_constraint_identity = (
            self.overtake_plan_motion_identity_cache.get(
                latest_constraint_generation
            )
        )
        tracking_plan_identity = (
            self.overtake_plan_motion_identity_cache.get(
                int(tracking_plan_generation)
            )
            if tracking_plan_generation >= 0
            else None
        )
        stable_release_delivery_successor = bool(
            latest_constraint is not None
            and latest_constraint.valid
            and not latest_constraint.stop_requested
            and latest_constraint.release_authorized
            and latest_constraint_identity is not None
            and tracking_plan_identity is not None
            and latest_constraint_generation != int(tracking_plan_generation)
            and tuple(tracking_plan_identity[:6])
            == tuple(latest_constraint_identity[:6])
            and int(tracking_plan_identity[6])
            > int(latest_constraint_identity[6])
            and int(tracking_plan_identity[8])
            == int(latest_constraint_identity[8])
            == int(OvertakePlan.PASSING)
            and int(tracking_plan_identity[9])
            > int(latest_constraint_identity[9])
            and bytes(tracking_plan_identity[10])
            == bytes(latest_constraint_identity[10])
            and tuple(
                self.overtake_plan_trajectory_cache.get(
                    int(tracking_plan_generation), ()
                )
            )
            == tuple(
                self.overtake_plan_trajectory_cache.get(
                    latest_constraint_generation, ()
                )
            )
        )
        plan_generation_regressed = bool(
            tracking_plan_generation >= 0
            and cached_plan_generations
            and tracking_plan_generation < max(cached_plan_generations)
        )
        constraint_generation_regressed = bool(
            latest_constraint is not None
            and (
                (
                    cached_constraint_plan_generations
                    and int(latest_constraint.plan_generation)
                    < max(cached_constraint_plan_generations)
                )
                or (
                    cached_constraint_generations
                    and int(latest_constraint.constraint_generation)
                    < max(cached_constraint_generations)
                )
            )
        )
        current_plan_payload_mutated = bool(
            tracking_plan_generation >= 0
            and self.overtake_plan_lateral_stop_taint_cache.get(
                int(tracking_plan_generation), False
            )
        )
        base_rendezvous_state = SafetyAuthorityRendezvousState.UNKNOWN
        if not latest_plan_fresh or not latest_constraint_fresh:
            base_rendezvous_state = SafetyAuthorityRendezvousState.STALE
        elif current_plan_payload_mutated:
            base_rendezvous_state = (
                SafetyAuthorityRendezvousState.PAYLOAD_MUTATION
            )
        elif (
            self.safety_constraint_timestamp_regressed
            or plan_generation_regressed
            or constraint_generation_regressed
            or latest_plan_invalid
            or (
                tracking_plan_generation >= 0
                and latest_constraint is not None
                and abs(
                    int(latest_constraint.plan_generation)
                    - int(tracking_plan_generation)
                )
                > 1
                and not stable_release_delivery_successor
            )
        ):
            base_rendezvous_state = (
                SafetyAuthorityRendezvousState.REGRESSION_OR_GAP
            )
        if latest_plan_invalid:
            # Preserve the historical fail-closed behavior for a fresh invalid
            # authoritative plan instead of masking it with an older pair.
            return SafetyAuthoritySelection(
                self.safety_constraint,
                self.safety_constraint_time_sec,
                tracking_plan_generation,
                base_rendezvous_state,
            )

        matched_entries: list[
            tuple[int, SafetyConstraintState, float, int]
        ] = []
        for contract_key, plan_entry in self.overtake_plan_contract_cache.items():
            plan_generation, contract_stamp_ns = contract_key
            (
                plan_valid,
                plan_time_sec,
                plan_stamp_ns,
                _,
                _,
                _,
                _,
                _,
            ) = plan_entry
            constraint_entry = self.safety_constraint_contract_cache.get(
                (plan_generation, contract_stamp_ns)
            )
            if constraint_entry is None:
                continue
            constraint, constraint_time_sec = constraint_entry
            if (
                not plan_valid
                or not constraint.valid
                or int(constraint.header_stamp_ns) != int(plan_stamp_ns)
            ):
                continue
            if not self._fresh(
                plan_time_sec, now_sec, self.overtake_plan_timeout_sec
            ) or not self._fresh(
                constraint_time_sec,
                now_sec,
                self.safety_constraint_timeout_sec,
            ):
                continue
            matched_entries.append(
                (
                    plan_stamp_ns,
                    constraint,
                    constraint_time_sec,
                    plan_generation,
                )
            )

        # Only the latest authoritative plan's exact (generation, stamp) peer
        # may release motion.  An older exact pair remains useful only when it
        # requests STOP; treating an old release pair as the active authority
        # would let a newly arrived PASS trajectory move before its constraint.
        matched_constraint: Optional[SafetyConstraintState] = None
        matched_constraint_time_sec: Optional[float] = None
        matched_plan_generation: Optional[int] = None
        current_plan_stamp_ns = self.overtake_plan_header_stamp_ns
        current_matches = [
            entry
            for entry in matched_entries
            if int(entry[3]) == int(tracking_plan_generation)
            and current_plan_stamp_ns is not None
            and int(entry[0]) == int(current_plan_stamp_ns)
        ]
        if current_matches:
            (
                _,
                matched_constraint,
                matched_constraint_time_sec,
                matched_plan_generation,
            ) = max(current_matches, key=lambda entry: (entry[0], entry[3]))
        matched_stop_entries = [
            entry for entry in matched_entries if bool(entry[1].stop_requested)
        ]
        matched_stop_entry = (
            max(matched_stop_entries, key=lambda entry: (entry[0], entry[3]))
            if matched_stop_entries
            else None
        )

        normal_delivery_gap = bool(
            base_rendezvous_state == SafetyAuthorityRendezvousState.UNKNOWN
            and tracking_plan_generation >= 0
            and latest_constraint is not None
            and latest_constraint.valid
            and not latest_constraint.stop_requested
            and latest_constraint.release_authorized
            and (
                abs(
                    int(latest_constraint.plan_generation)
                    - int(tracking_plan_generation)
                )
                <= 1
                or stable_release_delivery_successor
            )
            and (
                int(latest_constraint.plan_generation)
                != int(tracking_plan_generation)
                or matched_constraint is None
                or (
                    current_plan_stamp_ns is not None
                    and int(latest_constraint.header_stamp_ns)
                    != int(current_plan_stamp_ns)
                )
            )
        )
        selected_rendezvous_state = (
            SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
            if normal_delivery_gap
            else base_rendezvous_state
        )
        if latest_constraint is not None:
            if (
                self.safety_constraint_timestamp_regressed
                and matched_constraint is not None
            ):
                return SafetyAuthoritySelection(
                    matched_constraint,
                    matched_constraint_time_sec,
                    matched_plan_generation,
                    SafetyAuthorityRendezvousState.REGRESSION_OR_GAP,
                )
            if self.safety_constraint_timestamp_regressed and matched_entries:
                # A same-identity payload conflict is a supervisory STOP
                # barrier.  Keep the last exact historical cohort as the
                # authority input so evaluating this tainted successor cannot
                # advance the shared latch before a newer identity recovers.
                (
                    _,
                    historical_constraint,
                    historical_time_sec,
                    historical_plan_generation,
                ) = max(matched_entries, key=lambda entry: (entry[0], entry[3]))
                return SafetyAuthoritySelection(
                    historical_constraint,
                    historical_time_sec,
                    historical_plan_generation,
                    SafetyAuthorityRendezvousState.REGRESSION_OR_GAP,
                )
            must_apply_immediately = bool(
                self.safety_constraint_timestamp_regressed
                or not latest_constraint.valid
                or latest_constraint.stop_requested
                or not latest_constraint.release_authorized
            )
            if matched_constraint is not None:
                latest_constraint_well_formed = bool(
                    self.safety_authority._valid(latest_constraint)
                )
                strict_tightening = bool(
                    latest_constraint.speed_limit_mps
                    < matched_constraint.speed_limit_mps
                    or latest_constraint.required_brake_decel_mps2
                    > matched_constraint.required_brake_decel_mps2
                )
                unpaired_release_successor = bool(
                    normal_delivery_gap
                    and int(latest_constraint.plan_generation)
                    != int(tracking_plan_generation)
                )
                must_apply_immediately = bool(
                    must_apply_immediately
                    or (
                        latest_constraint.constraint_generation
                        > matched_constraint.constraint_generation
                        and (
                            not latest_constraint_well_formed
                            or strict_tightening
                            or not unpaired_release_successor
                        )
                    )
                )
            if must_apply_immediately:
                return SafetyAuthoritySelection(
                    latest_constraint,
                    self.safety_constraint_time_sec,
                    latest_constraint.plan_generation,
                    (
                        base_rendezvous_state
                        if base_rendezvous_state
                        != SafetyAuthorityRendezvousState.UNKNOWN
                        else SafetyAuthorityRendezvousState.REGRESSION_OR_GAP
                    ),
                )

        # A valid, non-tightening successor constraint may arrive before its
        # plan.  The current exact N pair remains authoritative in that
        # interval; do not label it as an unpaired STOP or advance the shared
        # latch to the constraint-only N+1 preview.
        if (
            matched_constraint is not None
            and normal_delivery_gap
            and latest_constraint is not None
            and int(latest_constraint.plan_generation)
            != int(tracking_plan_generation)
        ):
            if self.motion_authority_delivery_gap_lease is None:
                self.safety_authority_contract_unpaired = True
            return SafetyAuthoritySelection(
                matched_constraint,
                matched_constraint_time_sec,
                matched_plan_generation,
                (
                    SafetyAuthorityRendezvousState.EXACT_CURRENT
                    if self.motion_authority_delivery_gap_lease is not None
                    else SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
                ),
            )

        if matched_constraint is not None and not normal_delivery_gap:
            return SafetyAuthoritySelection(
                matched_constraint,
                matched_constraint_time_sec,
                matched_plan_generation,
                (
                    SafetyAuthorityRendezvousState.EXACT_CURRENT
                    if base_rendezvous_state
                    == SafetyAuthorityRendezvousState.UNKNOWN
                    else base_rendezvous_state
                ),
            )

        if matched_stop_entry is not None and not normal_delivery_gap:
            # STOP is monotonic/tighter and may bridge a topic delivery gap.
            # Its generation is returned verbatim so lateral-hold helpers can
            # decide independently whether steering proof is still eligible.
            _, stop_constraint, stop_time_sec, stop_plan_generation = (
                matched_stop_entry
            )
            return SafetyAuthoritySelection(
                stop_constraint,
                stop_time_sec,
                stop_plan_generation,
                SafetyAuthorityRendezvousState.REGRESSION_OR_GAP,
            )

        # A latest plan exists without its exact constraint peer.  Keep the
        # previous pair only as historical state for the authority latch and
        # mark this tick as unpaired; on_timer converts a typed normal
        # delivery-gap non-STOP decision to a transient, non-latching STOP.
        # This avoids both motion under an old release and a sticky fault on a
        # normal cross-topic delivery gap.
        self.safety_authority_contract_unpaired = True
        if matched_entries:
            (
                _,
                historical_constraint,
                historical_time_sec,
                historical_plan_generation,
            ) = max(matched_entries, key=lambda entry: (entry[0], entry[3]))
            return SafetyAuthoritySelection(
                historical_constraint,
                historical_time_sec,
                (
                    int(historical_constraint.plan_generation)
                    if normal_delivery_gap
                    else historical_plan_generation
                ),
                selected_rendezvous_state,
            )
        return SafetyAuthoritySelection(
            latest_constraint,
            self.safety_constraint_time_sec,
            (
                int(latest_constraint.plan_generation)
                if normal_delivery_gap and latest_constraint is not None
                else tracking_plan_generation
            ),
            selected_rendezvous_state,
        )

    def _lateral_stop_plan_authorized(
        self,
        *,
        plan_generation: Optional[int],
        constraint: Optional[SafetyConstraintState],
        tracking_status: Optional[ControllerTrackingStatus],
        now_sec: float,
    ) -> bool:
        """Accept only an exact typed plan/constraint/PP lateral STOP bundle."""
        if (
            plan_generation is None
            or constraint is None
            or tracking_status is None
        ):
            return False
        entry = self.overtake_plan_cache.get(int(plan_generation))
        if entry is None:
            return False
        (
            plan_valid,
            plan_time_sec,
            plan_stamp_ns,
            trajectory_authorized,
            lateral_maneuver_required,
            phase,
            published_pass_direction,
            target_vehicle_id,
        ) = entry
        typed = self.overtake_plan_lateral_stop_authority_cache.get(
            int(plan_generation)
        )
        if typed is None:
            return False
        (
            authority_kind,
            transaction_pass_direction,
            authority_token,
            attempt_id,
            typed_target_vehicle_id,
            typed_phase,
            typed_published_pass_direction,
            typed_immutable,
        ) = typed
        current_d_hold = bool(
            authority_kind == int(OvertakePlan.LATERAL_STOP_CURRENT_D_HOLD)
            and phase == int(OvertakePlan.ATTACK_FOLLOW)
            and published_pass_direction == 0
            and transaction_pass_direction in (-1, 1)
            and str(constraint.reason).strip()
            in (
                "maneuver_transaction_tracking_stop",
                "release_pending_safe_cycles",
            )
        )
        pass_warmup = bool(
            authority_kind == int(OvertakePlan.LATERAL_STOP_PASS_WARMUP)
            and phase == int(OvertakePlan.PASSING)
            and published_pass_direction in (-1, 1)
            and transaction_pass_direction == published_pass_direction
            and str(constraint.reason).strip() == "release_pending_safe_cycles"
        )
        tracking_or_warmup_acquisition = bool(
            tracking_status.trajectory_tracking_usable
            or (
                pass_warmup
                and tracking_status.pass_warmup_steering_acquisition_active
                and not tracking_status.pass_warmup_motion_ready
            )
        )
        checks = {
            "plan_valid": plan_valid,
            "trajectory_authorized": trajectory_authorized,
            "lateral_maneuver_required": lateral_maneuver_required,
            "typed_immutable": typed_immutable,
            "typed_phase": typed_phase == phase,
            "typed_direction": typed_published_pass_direction
            == published_pass_direction,
            "attempt": attempt_id > 0,
            "target": bool(str(target_vehicle_id).strip())
            and typed_target_vehicle_id == target_vehicle_id,
            "token": authority_token > 0,
            "stop_kind": current_d_hold or pass_warmup,
            "constraint_valid": constraint.valid,
            "constraint_stop": constraint.stop_requested
            and not constraint.release_authorized,
            "constraint_generation": int(constraint.plan_generation)
            == int(plan_generation),
            "constraint_stamp": int(constraint.header_stamp_ns)
            == int(plan_stamp_ns),
            "tracking_generation": int(tracking_status.plan_generation)
            == int(plan_generation),
            "tracking_or_acquisition": tracking_or_warmup_acquisition,
            "tracking_kind": int(tracking_status.lateral_stop_authority_kind)
            == authority_kind,
            "tracking_direction": int(
                tracking_status.lateral_stop_transaction_pass_direction
            )
            == transaction_pass_direction,
            "tracking_token": int(tracking_status.lateral_stop_authority_token)
            == authority_token,
            "fresh": self._fresh(
                plan_time_sec, now_sec, self.overtake_plan_timeout_sec
            ),
        }
        return all(checks.values())

    def _maneuver_transport_continuity_authorized(
        self,
        *,
        tracking_plan_generation: int,
        authority_plan_generation: Optional[int],
        authority_constraint: Optional[SafetyConstraintState],
        authority_constraint_time_sec: Optional[float],
        constraint_decision: SafetyConstraintDecision,
        selected_motion_sample: Optional[PurePursuitMotionSample],
        selected_input_source: str,
        pp_tracking_delivery_gap_usable: bool,
        active_control_fault_reason: str,
        now_sec: float,
        expected_phase: int,
        stop_release_bridge: bool = False,
    ) -> bool:
        """Bridge one bounded generation of the same authorized maneuver.

        Planner, typed plan/constraint, PP command and PP status are separate
        topics. A continuously regenerated ATTACK_FOLLOW or committed PASSING
        profile normally makes the PP proof trail by exactly one generation.
        This predicate may keep motion only when both generations are the same
        attempt, target, maneuver phase and pass direction, both exact
        plan/constraint bundles authorize motion, and their trajectories remain
        continuous. The ATTACK_FOLLOW stop-release branch never moves the
        vehicle; it only keeps an exact STOP tracking proof visible long enough
        for the planner's conservative release debounce to finish. No branch
        can authorize a target/attempt/side change or any fault interval.
        """
        expected_phase = int(expected_phase)
        if expected_phase not in (
            int(OvertakePlan.ATTACK_FOLLOW),
            int(OvertakePlan.PASSING),
        ) or (
            stop_release_bridge
            and expected_phase != int(OvertakePlan.ATTACK_FOLLOW)
        ):
            return False
        accepted_stop_reasons = (
            "maneuver_transaction_tracking_stop",
            "release_pending_safe_cycles",
        )
        current_constraint_mode_valid = bool(
            authority_constraint is not None
            and authority_constraint.valid
            and (
                (
                    stop_release_bridge
                    and constraint_decision.stop_required
                    and authority_constraint.stop_requested
                    and not authority_constraint.release_authorized
                    and str(authority_constraint.reason).strip()
                    in accepted_stop_reasons
                    and math.isfinite(
                        float(authority_constraint.speed_limit_mps)
                    )
                    and 0.0 < float(authority_constraint.speed_limit_mps)
                    <= self.planner_stop_release_bootstrap_max_speed_mps
                    + 1.0e-6
                )
                or (
                    not stop_release_bridge
                    and not constraint_decision.stop_required
                    and not authority_constraint.stop_requested
                    and authority_constraint.release_authorized
                )
            )
        )
        if (
            tracking_plan_generation <= 0
            or authority_plan_generation is None
            or int(authority_plan_generation) != int(tracking_plan_generation)
            or authority_constraint is None
            or not pp_tracking_delivery_gap_usable
            or self.safety_authority_contract_unpaired
            or bool(active_control_fault_reason)
            or self.control_fault_latched
            or self.external_stop_latched
            or self.finish_stop_latch.latched
            or self.safety_authority._fault_latched
            or not current_constraint_mode_valid
            or selected_motion_sample is None
            or selected_motion_sample.authority_proof is None
            or not bool(selected_motion_sample.authority_proof.pp_command_fresh)
            or not bool(
                selected_motion_sample.authority_proof.trajectory_tracking_usable
            )
            or str(selected_input_source).strip().lower() != "pure_pursuit"
            or not self._fresh(
                selected_motion_sample.command_receipt_time_sec,
                now_sec,
                self.attack_follow_transport_continuity_timeout_sec,
            )
            or not self._fresh(
                authority_constraint_time_sec,
                now_sec,
                self.attack_follow_transport_continuity_timeout_sec,
            )
        ):
            return False

        previous_generation = int(
            selected_motion_sample.authority_proof.plan_generation
        )
        if (
            previous_generation <= 0
            or previous_generation == int(tracking_plan_generation)
        ):
            return False
        command_values = (
            float(selected_motion_sample.command.longitudinal.speed),
            float(selected_motion_sample.command.longitudinal.acceleration),
            float(selected_motion_sample.command.lateral.steering_tire_angle),
        )
        if not all(math.isfinite(value) for value in command_values):
            return False
        if command_values[0] < 0.0:
            return False
        if (
            stop_release_bridge
            and authority_constraint is not None
            and command_values[0]
            > float(authority_constraint.speed_limit_mps) + 1.0e-6
        ):
            return False
        if (
            self._stamp_ns(selected_motion_sample.authority_proof.header.stamp)
            != int(selected_motion_sample.command_stamp_ns)
            or not self._fresh(
                selected_motion_sample.authority_proof_receipt_time_sec,
                now_sec,
                self.attack_follow_transport_continuity_timeout_sec,
            )
        ):
            return False

        current_plan = self.overtake_plan_cache.get(
            int(tracking_plan_generation)
        )
        previous_plan = self.overtake_plan_cache.get(previous_generation)
        current_attempt = self.overtake_plan_attempt_cache.get(
            int(tracking_plan_generation)
        )
        previous_attempt = self.overtake_plan_attempt_cache.get(
            previous_generation
        )
        current_trajectory = self.overtake_plan_trajectory_cache.get(
            int(tracking_plan_generation)
        )
        previous_trajectory = self.overtake_plan_trajectory_cache.get(
            previous_generation
        )
        previous_constraint_entry = self.safety_constraint_cache.get(
            previous_generation
        )
        if (
            current_plan is None
            or previous_plan is None
            or current_attempt is None
            or previous_attempt is None
            or current_attempt <= 0
            or current_attempt != previous_attempt
            or current_trajectory is None
            or previous_trajectory is None
            or previous_constraint_entry is None
        ):
            return False

        (
            current_valid,
            current_time_sec,
            current_stamp_ns,
            current_trajectory_authorized,
            current_lateral_maneuver_required,
            current_phase,
            current_direction,
            current_target,
        ) = current_plan
        (
            previous_valid,
            previous_time_sec,
            previous_stamp_ns,
            previous_trajectory_authorized,
            previous_lateral_maneuver_required,
            previous_phase,
            previous_direction,
            previous_target,
        ) = previous_plan
        previous_constraint, previous_constraint_time_sec = (
            previous_constraint_entry
        )
        previous_constraint_mode_valid = bool(
            previous_constraint.valid
            and (
                (
                    stop_release_bridge
                    and previous_constraint.stop_requested
                    and not previous_constraint.release_authorized
                    and str(previous_constraint.reason).strip()
                    in accepted_stop_reasons
                    and math.isfinite(
                        float(previous_constraint.speed_limit_mps)
                    )
                    and 0.0 < float(previous_constraint.speed_limit_mps)
                    <= self.planner_stop_release_bootstrap_max_speed_mps
                    + 1.0e-6
                )
                or (
                    not stop_release_bridge
                    and not previous_constraint.stop_requested
                    and previous_constraint.release_authorized
                )
            )
        )
        same_maneuver_direction = bool(
            (
                expected_phase == int(OvertakePlan.ATTACK_FOLLOW)
                and int(current_direction) in (-1, 0, 1)
                and int(previous_direction) == int(current_direction)
            )
            or (
                expected_phase == int(OvertakePlan.PASSING)
                and int(current_direction) in (-1, 1)
                and int(previous_direction) == int(current_direction)
            )
        )
        if not (
            current_valid
            and previous_valid
            and current_trajectory_authorized
            and previous_trajectory_authorized
            and current_lateral_maneuver_required
            and previous_lateral_maneuver_required
            and int(current_phase) == expected_phase
            and int(previous_phase) == expected_phase
            and same_maneuver_direction
            and bool(str(current_target).strip())
            and str(current_target) == str(previous_target)
            and int(authority_constraint.plan_generation)
            == int(tracking_plan_generation)
            and int(authority_constraint.header_stamp_ns)
            == int(current_stamp_ns)
            and previous_constraint_mode_valid
            and int(previous_constraint.plan_generation)
            == previous_generation
            and int(previous_constraint.header_stamp_ns)
            == int(previous_stamp_ns)
            and self._fresh(
                current_time_sec,
                now_sec,
                self.attack_follow_transport_continuity_timeout_sec,
            )
            and self._fresh(
                previous_time_sec,
                now_sec,
                self.attack_follow_transport_continuity_timeout_sec,
            )
            and self._fresh(
                previous_constraint_time_sec,
                now_sec,
                self.attack_follow_transport_continuity_timeout_sec,
            )
        ):
            return False

        if (
            len(current_trajectory) < 2
            or len(current_trajectory) != len(previous_trajectory)
        ):
            return False
        return all(
            math.hypot(current_x - previous_x, current_y - previous_y)
            <= self.attack_follow_transport_max_point_delta_m
            for (current_x, current_y), (previous_x, previous_y) in zip(
                current_trajectory, previous_trajectory
            )
        )

    def _free_run_record_supports_normal_delivery_gap_hold(
        self,
        *,
        tracking_plan_generation: int,
        now_sec: float,
    ) -> bool:
        """Validate immutable N-1 provenance without extending its hold lease.

        Source provenance is published at a lower cadence than actuation.  Its
        1.5 s bound therefore remains independent from the 0.12 s steering-hold
        lease enforced by `_normal_delivery_gap_baseline_hold_candidate`.
        """
        record = self.free_run_live_exact_published_record
        source_key = self.free_run_live_exact_source_key
        source_identity = tuple(
            self.free_run_live_exact_source_key_identity or ()
        )
        if (
            not isinstance(record, FreeRunPublishedCommandRecord)
            or source_key is None
            or not self.free_run_live_exact_source_key_valid
            or int(tracking_plan_generation) <= 0
            or int(record.plan_generation)
            != int(tracking_plan_generation) - 1
            or self.last_verified_baseline_free_run_plan_generation
            != int(record.plan_generation)
            or self.last_verified_baseline_free_run_plan_stamp_ns
            != int(record.plan_stamp_ns)
            or self.last_verified_baseline_free_run_steering_rad is None
            or not math.isclose(
                float(self.last_verified_baseline_free_run_steering_rad),
                float(record.published_steering_rad),
                rel_tol=0.0,
                abs_tol=1.0e-9,
            )
            or source_identity != tuple(record.source_identity)
            or source_identity in self.free_run_live_exact_tainted_sources
            or self._free_run_source_semantic_identity(source_key)
            != tuple(record.source_semantic_identity)
            or not self._fresh(
                self.free_run_live_exact_source_key_time_sec,
                now_sec,
                self.free_run_source_provenance_timeout_sec,
            )
        ):
            return False
        source_wire = self._canonical_free_run_source_key_wire(source_key)
        record_age_sec = now_sec - record.publish_steady_time_sec
        return bool(
            source_wire is not None
            and hashlib.sha256(source_wire).digest()
            == record.source_wire_sha256
            and hashlib.sha256(record.final_command_cdr).digest()
            == record.final_command_sha256
            and math.isfinite(record_age_sec)
            and 0.0
            <= record_age_sec
            <= self.free_run_source_provenance_timeout_sec
        )

    def _normal_delivery_gap_baseline_hold_candidate(
        self,
        *,
        selection: SafetyAuthoritySelection,
        tracking_plan_generation: int,
        decision: SafetyConstraintDecision,
        now_sec: float,
        pp_tracking_delivery_gap_usable: bool = False,
    ) -> bool:
        """Preserve only a verified FREE_RUN angle across one direct successor.

        This is not motion authority.  It recognizes one bounded, forward
        cross-topic delivery gap so the later STOP composition may keep only
        the last final-published steering angle instead of resetting it to
        zero.  The gap is either plan N / constraint N-1, or exact
        plan/constraint N with a selected PP tracking proof for N-1.
        """
        previous_generation = int(tracking_plan_generation) - 1
        plan_constraint_delivery_gap = bool(
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
            and decision.reason == "safety_authority_contract_unpaired"
            and selection.authority_plan_generation == previous_generation
        )
        pp_tracking_delivery_gap = bool(
            pp_tracking_delivery_gap_usable
            and selection.rendezvous_state
            == SafetyAuthorityRendezvousState.EXACT_CURRENT
            and not decision.stop_required
            and selection.authority_plan_generation
            == int(tracking_plan_generation)
        )
        if (
            not (plan_constraint_delivery_gap or pp_tracking_delivery_gap)
            or tracking_plan_generation <= 0
            or selection.constraint is None
        ):
            return False
        current_plan = self.overtake_plan_cache.get(
            int(tracking_plan_generation)
        )
        previous_plan = self.overtake_plan_cache.get(previous_generation)
        current_identity = self.overtake_plan_motion_identity_cache.get(
            int(tracking_plan_generation)
        )
        previous_identity = self.overtake_plan_motion_identity_cache.get(
            previous_generation
        )
        if (
            current_plan is None
            or previous_plan is None
            or current_identity is None
            or previous_identity is None
        ):
            return False
        (
            current_valid,
            current_time_sec,
            current_stamp_ns,
            current_trajectory_authorized,
            current_lateral_maneuver_required,
            current_phase,
            current_direction,
            current_target,
        ) = current_plan
        (
            previous_valid,
            previous_time_sec,
            previous_stamp_ns,
            previous_trajectory_authorized,
            previous_lateral_maneuver_required,
            previous_phase,
            previous_direction,
            previous_target,
        ) = previous_plan
        constraint = selection.constraint
        steering_age_sec = (
            now_sec - self.last_verified_baseline_free_run_steering_time_sec
            if self.last_verified_baseline_free_run_steering_time_sec
            is not None
            else math.inf
        )
        identities_are_same_nominal_source = bool(
            int(current_identity[0]) == int(previous_identity[0])
            and int(current_identity[1]) == int(previous_identity[1])
            and int(current_identity[2]) == int(previous_identity[2]) == 0
            and str(current_identity[3]) == str(previous_identity[3]) == ""
            and int(current_identity[4]) == int(previous_identity[4]) == 0
            and int(current_identity[5]) == int(previous_identity[5]) == 0
            and int(current_identity[8])
            == int(previous_identity[8])
            == int(OvertakePlan.FREE_RUN)
            and int(current_identity[9]) == int(previous_identity[9])
            and bytes(current_identity[10]) == bytes(previous_identity[10])
            and bool(current_identity[11])
            == bool(previous_identity[11])
        )
        return bool(
            current_valid
            and previous_valid
            and not current_trajectory_authorized
            and not previous_trajectory_authorized
            and not current_lateral_maneuver_required
            and not previous_lateral_maneuver_required
            and int(current_phase) == int(OvertakePlan.FREE_RUN)
            and int(previous_phase) == int(OvertakePlan.FREE_RUN)
            and int(current_direction) == int(previous_direction) == 0
            and not str(current_target)
            and not str(previous_target)
            and int(current_stamp_ns) > int(previous_stamp_ns)
            and self.overtake_plan_header_stamp_ns is not None
            and int(current_stamp_ns)
            == int(self.overtake_plan_header_stamp_ns)
            and constraint.valid
            and not constraint.stop_requested
            and constraint.release_authorized
            and (
                (
                    plan_constraint_delivery_gap
                    and int(constraint.plan_generation) == previous_generation
                    and int(constraint.header_stamp_ns) == int(previous_stamp_ns)
                )
                or (
                    pp_tracking_delivery_gap
                    and int(constraint.plan_generation)
                    == int(tracking_plan_generation)
                    and int(constraint.header_stamp_ns) == int(current_stamp_ns)
                )
            )
            and self._fresh(
                current_time_sec, now_sec, self.overtake_plan_timeout_sec
            )
            and self._fresh(
                previous_time_sec, now_sec, self.overtake_plan_timeout_sec
            )
            and identities_are_same_nominal_source
            and self.last_verified_baseline_free_run_plan_generation
            == previous_generation
            and self.last_verified_baseline_free_run_plan_stamp_ns
            == int(previous_stamp_ns)
            and self.last_verified_baseline_free_run_epoch
            == int(self.finish_stop_latch.epoch)
            and self.last_verified_baseline_free_run_steering_rad is not None
            and math.isfinite(
                self.last_verified_baseline_free_run_steering_rad
            )
            and abs(self.last_verified_baseline_free_run_steering_rad)
            <= float(self.tracking_usable_max_steering_angle_rad)
            and math.isfinite(steering_age_sec)
            and 0.0
            <= steering_age_sec
            <= self.lateral_stop_steering_hold_timeout_sec
        )

    def _baseline_free_run_plan_authorized(
        self,
        *,
        plan_generation: Optional[int],
        constraint: Optional[SafetyConstraintState],
        now_sec: float,
    ) -> bool:
        """Match one exact released baseline FREE_RUN plan/constraint pair."""
        if plan_generation is None or constraint is None:
            return False
        entry = self.overtake_plan_cache.get(int(plan_generation))
        if entry is None:
            return False
        (
            plan_valid,
            plan_time_sec,
            plan_stamp_ns,
            trajectory_authorized,
            lateral_maneuver_required,
            phase,
            pass_direction,
            _,
        ) = entry
        return bool(
            plan_valid
            and not trajectory_authorized
            and not lateral_maneuver_required
            and int(phase) == int(OvertakePlan.FREE_RUN)
            and int(pass_direction) == 0
            and constraint.valid
            and not constraint.stop_requested
            and constraint.release_authorized
            and int(constraint.plan_generation) == int(plan_generation)
            and int(constraint.header_stamp_ns) == int(plan_stamp_ns)
            and self._fresh(
                plan_time_sec, now_sec, self.overtake_plan_timeout_sec
            )
        )

    def _baseline_stop_plan_authorized(
        self,
        *,
        plan_generation: Optional[int],
        constraint: Optional[SafetyConstraintState],
        now_sec: float,
    ) -> bool:
        """Authorize only longitudinal STOP over an exact baseline PP plan.

        A planner STOP must not replace fresh nominal-path steering with zero
        while the vehicle is still moving.  This contract is intentionally
        disjoint from lateral maneuver authority: it accepts only an exact,
        fresh plan/constraint pair that explicitly requests no lateral
        maneuver.  E-stop, watchdog, stale/missing contracts, and PASS plans
        remain on the ordinary fail-closed zero-steering path.
        """
        if plan_generation is None or constraint is None:
            return False
        entry = self.overtake_plan_cache.get(int(plan_generation))
        if entry is None:
            return False
        (
            plan_valid,
            plan_time_sec,
            plan_stamp_ns,
            trajectory_authorized,
            lateral_maneuver_required,
            phase,
            pass_direction,
            _,
        ) = entry
        return bool(
            plan_valid
            and not trajectory_authorized
            and not lateral_maneuver_required
            and int(phase)
            in (
                int(OvertakePlan.FREE_RUN),
                int(OvertakePlan.ATTACK_FOLLOW),
                int(OvertakePlan.ABORT_HOLD),
            )
            and int(pass_direction) == 0
            and constraint.valid
            and constraint.stop_requested
            and int(constraint.plan_generation) == int(plan_generation)
            and int(constraint.header_stamp_ns) == int(plan_stamp_ns)
            and self._fresh(
                plan_time_sec, now_sec, self.overtake_plan_timeout_sec
            )
        )

    def _lateral_stop_contract_delivery_gap_authorized(
        self,
        *,
        plan_generation: Optional[int],
        constraint: Optional[SafetyConstraintState],
        constraint_decision: SafetyConstraintDecision,
        now_sec: float,
    ) -> bool:
        """Recognize only a bounded same-generation plan/constraint skew.

        The planner republishes a lateral STOP bundle on two topics. A timer
        may observe the new plan stamp one callback before the constraint peer
        (or the reverse). This predicate never authorizes motion or a new
        steering value; it only lets the caller keep a recently verified
        lateral-stop angle while longitudinal STOP remains active.
        """
        if plan_generation is None or constraint is None:
            return False
        entry = self.overtake_plan_cache.get(int(plan_generation))
        if entry is None:
            return False
        (
            plan_valid,
            plan_time_sec,
            plan_stamp_ns,
            trajectory_authorized,
            lateral_maneuver_required,
            _,
            _,
            _,
        ) = entry
        stamp_skew_sec = abs(
            int(plan_stamp_ns) - int(constraint.header_stamp_ns)
        ) / 1.0e9
        return bool(
            plan_valid
            and trajectory_authorized
            and lateral_maneuver_required
            and constraint.valid
            and constraint.stop_requested
            and constraint_decision.stop_required
            and constraint_decision.reason
            in (
                "safety_constraint_applied",
                "safety_constraint_retransmit",
            )
            and int(constraint.plan_generation) == int(plan_generation)
            and int(constraint_decision.plan_generation)
            == int(plan_generation)
            and 0.0 < stamp_skew_sec
            <= self.lateral_stop_steering_hold_timeout_sec
            and self._fresh(
                plan_time_sec, now_sec, self.overtake_plan_timeout_sec
            )
        )

    def _baseline_stop_contract_delivery_gap_authorized(
        self,
        *,
        plan_generation: Optional[int],
        constraint: Optional[SafetyConstraintState],
        constraint_decision: SafetyConstraintDecision,
        now_sec: float,
    ) -> bool:
        """Keep only a recently verified baseline steering across topic skew."""
        if plan_generation is None or constraint is None:
            return False
        entry = self.overtake_plan_cache.get(int(plan_generation))
        if entry is None:
            return False
        (
            plan_valid,
            plan_time_sec,
            plan_stamp_ns,
            trajectory_authorized,
            lateral_maneuver_required,
            phase,
            pass_direction,
            _,
        ) = entry
        stamp_skew_sec = abs(
            int(plan_stamp_ns) - int(constraint.header_stamp_ns)
        ) / 1.0e9
        return bool(
            plan_valid
            and not trajectory_authorized
            and not lateral_maneuver_required
            and int(phase)
            in (
                int(OvertakePlan.FREE_RUN),
                int(OvertakePlan.ATTACK_FOLLOW),
                int(OvertakePlan.ABORT_HOLD),
            )
            and int(pass_direction) == 0
            and constraint.valid
            and constraint.stop_requested
            and constraint_decision.stop_required
            and constraint_decision.reason
            in (
                "safety_constraint_applied",
                "safety_constraint_retransmit",
            )
            and int(constraint.plan_generation) == int(plan_generation)
            and int(constraint_decision.plan_generation)
            == int(plan_generation)
            and 0.0 < stamp_skew_sec
            <= self.lateral_stop_steering_hold_timeout_sec
            and self._fresh(
                plan_time_sec, now_sec, self.overtake_plan_timeout_sec
            )
        )

    def _planner_stop_release_bootstrap_valid(
        self,
        *,
        plan_generation: Optional[int],
        constraint: Optional[SafetyConstraintState],
        now_sec: float,
    ) -> bool:
        """Allow an exact stopped ATTACK_FOLLOW plan to start PP warm-up."""
        if plan_generation is None or constraint is None:
            return False
        entry = self.overtake_plan_cache.get(int(plan_generation))
        if entry is None:
            return False
        (
            plan_valid,
            plan_time_sec,
            plan_stamp_ns,
            trajectory_authorized,
            _lateral_maneuver_required,
            phase,
            pass_direction,
            target_vehicle_id,
        ) = entry
        stopped_follow_shape = bool(
            int(phase) == int(OvertakePlan.ATTACK_FOLLOW)
            and int(pass_direction) == 0
            and bool(str(target_vehicle_id).strip())
            and not trajectory_authorized
        )
        accepted_stop_reason = str(constraint.reason).strip() in (
            "maneuver_transaction_tracking_stop",
            "release_pending_safe_cycles",
        )
        return bool(
            plan_valid
            and stopped_follow_shape
            and constraint.valid
            and constraint.stop_requested
            and not constraint.release_authorized
            and accepted_stop_reason
            and math.isfinite(float(constraint.speed_limit_mps))
            and 0.0 < float(constraint.speed_limit_mps)
            <= self.planner_stop_release_bootstrap_max_speed_mps + 1.0e-6
            and int(constraint.plan_generation) == int(plan_generation)
            and int(constraint.header_stamp_ns) == int(plan_stamp_ns)
            and self._fresh(
                plan_time_sec, now_sec, self.overtake_plan_timeout_sec
            )
        )

    def _authorized_pass_release_contract_valid(
        self,
        *,
        plan_generation: Optional[int],
        constraint: Optional[SafetyConstraintState],
        now_sec: float,
    ) -> bool:
        """Keep an exact PASS release visible while historical latches clear."""
        if plan_generation is None or constraint is None:
            return False
        entry = self.overtake_plan_cache.get(int(plan_generation))
        if entry is None:
            return False
        (
            plan_valid,
            plan_time_sec,
            plan_stamp_ns,
            trajectory_authorized,
            lateral_maneuver_required,
            phase,
            pass_direction,
            _,
        ) = entry
        return bool(
            plan_valid
            and trajectory_authorized
            and lateral_maneuver_required
            and int(phase) == int(OvertakePlan.PASSING)
            and int(pass_direction) in (-1, 1)
            and constraint.valid
            and not constraint.stop_requested
            and constraint.release_authorized
            and int(constraint.plan_generation) == int(plan_generation)
            and int(constraint.header_stamp_ns) == int(plan_stamp_ns)
            and self._fresh(
                plan_time_sec, now_sec, self.overtake_plan_timeout_sec
            )
        )

    def _apply_safety_constraint(
        self,
        cmd: AckermannControlCommand,
        source: str,
        reason: str,
        decision: SafetyConstraintDecision,
        now_sec: float,
        *,
        lateral_tracking_usable: bool = False,
        lateral_hold_steering_rad: Optional[float] = None,
    ) -> tuple[AckermannControlCommand, str, str]:
        if not (
            math.isfinite(float(cmd.longitudinal.speed))
            and math.isfinite(float(cmd.longitudinal.acceleration))
        ):
            return self._stop_command(now_sec), "stop", "invalid_tracking_command"
        if source == "finish_stop":
            # 公式Finish後はすでにspeed=0のterminal停止指令であり、通常の
            # race_not_armed safety constraintより弱くない。ここで汎用stopへ
            # 置換すると操舵が0になり、高速のままコース外へ直進するため、
            # 縦の停止要求だけを保守的に合成してcontrollerの操舵を保持する。
            cmd.longitudinal.speed = 0.0
            if (
                decision.reason
                in {
                    "safety_constraint_applied",
                    "safety_constraint_retransmit",
                    "safety_constraint_relaxation_not_authorized",
                }
                and math.isfinite(decision.required_brake_decel_mps2)
                and decision.required_brake_decel_mps2 > 0.0
            ):
                cmd.longitudinal.acceleration = min(
                    float(cmd.longitudinal.acceleration),
                    -decision.required_brake_decel_mps2,
                )
            return cmd, source, reason
        if decision.stop_required:
            if lateral_tracking_usable and source == "pure_pursuit":
                # The exact plan/constraint bundle authorizes lateral tracking
                # while the independent longitudinal authority requests stop.
                # Keep the verified PP steering and compose only a stricter
                # longitudinal stop; E-stop/watchdog/control fault select
                # source=stop earlier and can never enter this branch.
                cmd = self._stamp(copy.deepcopy(cmd), now_sec)
                cmd.longitudinal.speed = 0.0
                cmd.longitudinal.acceleration = min(
                    float(cmd.longitudinal.acceleration),
                    float(self.stop_decel_mps2),
                )
                if decision.required_brake_decel_mps2 > 0.0:
                    cmd.longitudinal.acceleration = min(
                        float(cmd.longitudinal.acceleration),
                        -decision.required_brake_decel_mps2,
                    )
                return (
                    cmd,
                    source,
                    "safety_constraint_stop_with_lateral_tracking",
                )
            if (
                source == "pure_pursuit"
                and lateral_hold_steering_rad is not None
                and math.isfinite(lateral_hold_steering_rad)
            ):
                # A plan/status delivery skew must not reset a previously
                # verified steering command to zero. Keep only the bounded
                # last verified angle; longitudinal motion remains stopped.
                cmd = self._stamp(copy.deepcopy(cmd), now_sec)
                cmd.longitudinal.speed = 0.0
                cmd.longitudinal.acceleration = min(
                    float(cmd.longitudinal.acceleration),
                    float(self.stop_decel_mps2),
                )
                if decision.required_brake_decel_mps2 > 0.0:
                    cmd.longitudinal.acceleration = min(
                        float(cmd.longitudinal.acceleration),
                        -decision.required_brake_decel_mps2,
                    )
                cmd.lateral.steering_tire_angle = float(
                    lateral_hold_steering_rad
                )
                cmd.lateral.steering_tire_rotation_rate = 0.0
                return (
                    cmd,
                    source,
                    "safety_constraint_stop_with_lateral_hold",
                )
            stop_cmd = self._stop_command(now_sec)
            if decision.required_brake_decel_mps2 > 0.0:
                stop_cmd.longitudinal.acceleration = min(
                    float(stop_cmd.longitudinal.acceleration),
                    -decision.required_brake_decel_mps2,
                )
            return stop_cmd, "stop", decision.reason
        speed_mps, acceleration_mps2 = apply_safety_constraint(
            float(cmd.longitudinal.speed),
            float(cmd.longitudinal.acceleration),
            decision,
        )
        cmd.longitudinal.speed = speed_mps
        cmd.longitudinal.acceleration = acceleration_mps2
        if decision.reason == "safety_constraint_relaxation_not_authorized":
            reason = decision.reason
        return cmd, source, reason

    def _select_command(
        self,
        source: str,
        now_sec: float,
        selected_input_cmd: Optional[AckermannControlCommand],
    ) -> AckermannControlCommand:
        if source == "mpc" and selected_input_cmd is not None:
            return self._stamp(copy.deepcopy(selected_input_cmd), now_sec)
        if source == "pure_pursuit" and selected_input_cmd is not None:
            return self._fallback_command(copy.deepcopy(selected_input_cmd), now_sec)
        if source == "state_lattice" and selected_input_cmd is not None:
            return self._stamp(copy.deepcopy(selected_input_cmd), now_sec)
        if source == "recovery" and selected_input_cmd is not None:
            return self._recovery_command(copy.deepcopy(selected_input_cmd), now_sec)
        if source == "finish_stop":
            return self._finish_stop_command(now_sec)
        return self._stop_command(now_sec)

    def _selected_input_command(
        self, source: str
    ) -> Optional[AckermannControlCommand]:
        if source == "mpc":
            return self.mpc_cmd
        if source == "pure_pursuit":
            return self.pure_pursuit_cmd
        if source == "state_lattice" and self.state_lattice_cmd is not None:
            return self.state_lattice_cmd.command
        if source == "recovery" and self.recovery_cmd is not None:
            return self.recovery_cmd.command
        return None

    def _fallback_command(
        self, cmd: AckermannControlCommand, now_sec: float
    ) -> AckermannControlCommand:
        cmd = self._stamp(cmd, now_sec)
        cmd.longitudinal.speed = self._finite_clamp(
            cmd.longitudinal.speed, 0.0, max(0.0, self.fallback_speed_mps)
        )
        cmd.longitudinal.acceleration = self._finite_clamp(
            cmd.longitudinal.acceleration,
            self.fallback_decel_min_mps2,
            self.fallback_accel_max_mps2,
        )
        return cmd

    def _recovery_command(
        self, cmd: AckermannControlCommand, now_sec: float
    ) -> AckermannControlCommand:
        cmd = self._stamp(cmd, now_sec)
        cmd.longitudinal.speed = self._finite_clamp(
            cmd.longitudinal.speed, 0.0, max(0.0, self.recovery_speed_mps)
        )
        cmd.longitudinal.acceleration = self._finite_clamp(
            cmd.longitudinal.acceleration,
            self.recovery_decel_min_mps2,
            self.recovery_accel_max_mps2,
        )
        return cmd

    def _limit_steering(
        self, cmd: AckermannControlCommand, source: str, now_sec: float
    ) -> SteeringLimitResult:
        if source == "stop":
            result = self.steering_limiter.reset(0.0, now_sec, source)
            cmd.lateral.steering_tire_angle = 0.0
            return result

        result = self.steering_limiter.update(
            float(cmd.lateral.steering_tire_angle), now_sec, source
        )
        cmd.lateral.steering_tire_angle = result.limited_steering_rad
        if source in {"mpc", "pure_pursuit", "state_lattice", "recovery"} and math.isfinite(
            result.limited_steering_rad
        ):
            self.last_tracking_steering_rad = result.limited_steering_rad
        if result.angle_limited or result.rate_limited:
            if now_sec - self.last_steering_limit_log_sec >= self.steering_log_throttle_sec:
                self.get_logger().warn(
                    "hybrid steering limited "
                    f"source={source} raw={result.raw_steering_rad:.3f} "
                    f"limited={result.limited_steering_rad:.3f} "
                    f"delta={result.steering_delta_rad:.3f} "
                    f"angle_limited={result.angle_limited} "
                    f"rate_limited={result.rate_limited}"
                )
                self.last_steering_limit_log_sec = now_sec
        return result

    @staticmethod
    def _control_command_payload(command: AckermannControlCommand) -> tuple:
        """Canonical actuator payload, excluding the per-tick header stamp."""
        return (
            float(command.longitudinal.speed),
            float(command.longitudinal.acceleration),
            float(command.longitudinal.jerk),
            float(command.lateral.steering_tire_angle),
            float(command.lateral.steering_tire_rotation_rate),
        )

    def _prepare_motion_authority_delivery_gap_replay(
        self,
        *,
        command: AckermannControlCommand,
        source: str,
        now_sec: float,
    ) -> Optional[SteeringLimitResult]:
        """Realign the limiter only for the exact last-published frozen N."""
        lease = self.motion_authority_delivery_gap_lease
        published = self.last_published_control_command
        if (
            lease is None
            or published is None
            or source != "pure_pursuit"
            or self.last_published_control_source != "pure_pursuit"
        ):
            return None
        lease_payload = self._control_command_payload(lease.final_command)
        command_payload = self._control_command_payload(command)
        published_payload = self._control_command_payload(published)
        if (
            not all(
                math.isfinite(value)
                for value in (
                    *lease_payload,
                    *command_payload,
                    *published_payload,
                )
            )
            or command_payload != lease_payload
            or published_payload != lease_payload
        ):
            return None
        # Resetting here cannot create an actuator jump: the reset value is
        # byte-for-byte the last value published by this node.  It atomically
        # makes the limiter's next N+1 calculation start from that physical
        # output while leaving the lease/gap deadlines untouched.
        result = self.steering_limiter.reset(
            float(lease.final_command.lateral.steering_tire_angle),
            now_sec,
            source,
        )
        command.lateral.steering_tire_angle = result.limited_steering_rad
        if math.isfinite(result.limited_steering_rad):
            self.last_tracking_steering_rad = result.limited_steering_rad
        return result

    def _stop_command(self, now_sec: float) -> AckermannControlCommand:
        cmd = AckermannControlCommand()
        cmd = self._stamp(cmd, now_sec)
        cmd.longitudinal.speed = 0.0
        cmd.longitudinal.acceleration = self.stop_decel_mps2
        cmd.lateral.steering_tire_angle = 0.0
        return cmd

    def _finish_stop_command(self, now_sec: float) -> AckermannControlCommand:
        """Construct a canonical terminal command from the frozen snapshot."""
        cmd = AckermannControlCommand()
        cmd = self._stamp(cmd, now_sec)
        cmd.longitudinal.speed = 0.0
        cmd.longitudinal.acceleration = self.finish_stop_decel_mps2
        cmd.longitudinal.jerk = 0.0
        snapshot_steering_rad = self.finish_terminal_snapshot_steering_rad
        if snapshot_steering_rad is None or not math.isfinite(
            snapshot_steering_rad
        ):
            cmd.lateral.steering_tire_angle = 0.0
        elif self.finish_stop_lateral_guard_active:
            cmd.lateral.steering_tire_angle = self._finite_clamp(
                snapshot_steering_rad,
                -self.finish_stop_max_steering_rad,
                self.finish_stop_max_steering_rad,
            )
        else:
            cmd.lateral.steering_tire_angle = float(snapshot_steering_rad)
        cmd.lateral.steering_tire_rotation_rate = 0.0
        return cmd

    def _stamp(self, cmd: AckermannControlCommand, now_sec: float) -> AckermannControlCommand:
        stamp = self.get_clock().now().to_msg()
        cmd.stamp = stamp
        cmd.longitudinal.stamp = stamp
        cmd.lateral.stamp = stamp
        return cmd

    def _current_health(self, now_sec: float) -> MpcHealth:
        if self.mpc_health_time_sec is None:
            return MpcHealth(valid=False, status="missing", age_sec=float("inf"))
        age_sec = now_sec - self.mpc_health_time_sec
        if age_sec > self.mpc_health_timeout_sec:
            return MpcHealth(
                valid=False,
                status="stale",
                infeasible_count=self.mpc_health.infeasible_count,
                age_sec=age_sec,
            )
        return MpcHealth(
            valid=self.mpc_health.valid,
            status=self.mpc_health.status,
            infeasible_count=self.mpc_health.infeasible_count,
            age_sec=age_sec,
        )

    def _current_recovery_state(self, now_sec: float) -> RecoveryMuxState:
        status_fresh = self._fresh(
            self.recovery_status_time_sec, now_sec, self.recovery_status_timeout_sec
        )
        command_fresh = self._fresh(
            self.recovery_cmd_time_sec, now_sec, self.recovery_cmd_timeout_sec
        )
        permit_fresh = self._fresh(
            self.recovery_permit_time_sec, now_sec, self.recovery_permit_timeout_sec
        )
        safety_fresh = self._fresh(
            self.external_safety_time_sec, now_sec, self.external_safety_timeout_sec
        )
        if self.require_recovery_external_safety_status:
            external_safety_ok = (
                safety_fresh
                and self.external_safety_status is not None
                and bool(self.external_safety_status.valid)
                and not bool(self.external_safety_status.stop_requested)
            )
        else:
            external_safety_ok = (
                self.external_safety_status is None
                or not safety_fresh
                or (bool(self.external_safety_status.valid)
                    and not bool(self.external_safety_status.stop_requested))
            )

        status = self.recovery_status
        command = self.recovery_cmd
        permit = self.recovery_permit
        status_state = self._recovery_state_name(status.state) if status else "missing"
        ids_match = (
            status is not None
            and command is not None
            and permit is not None
            and status.attempt_id == command.attempt_id == permit.attempt_id
            and status.trajectory_generation
            == command.trajectory_generation
            == permit.trajectory_generation
        )
        trajectory_header_match = (
            status is not None
            and command is not None
            and status.trajectory_header.frame_id == command.trajectory_header.frame_id
            and status.trajectory_header.stamp.sec == command.trajectory_header.stamp.sec
            and status.trajectory_header.stamp.nanosec
            == command.trajectory_header.stamp.nanosec
        )
        command_valid = False
        command_forward_only = False
        if command is not None:
            cmd = command.command
            command_valid = (
                math.isfinite(float(cmd.longitudinal.speed))
                and math.isfinite(float(cmd.longitudinal.acceleration))
                and math.isfinite(float(cmd.lateral.steering_tire_angle))
            )
            command_forward_only = command_valid and float(cmd.longitudinal.speed) >= 0.0

        return RecoveryMuxState(
            enabled=self.recovery_enabled,
            status_state=status_state,
            status_fresh=status_fresh,
            command_fresh=command_fresh,
            permit_fresh=permit_fresh,
            external_safety_ok=external_safety_ok,
            input_complete=bool(status.input_complete) if status else False,
            trajectory_valid=bool(status.trajectory_valid) if status else False,
            trajectory_safe=bool(status.trajectory_safe) if status else False,
            stop_required=bool(status.stop_required) if status else False,
            handoff_ready=bool(status.handoff_ready) if status else False,
            recovery_allowed=bool(permit.recovery_allowed) if permit else False,
            handoff_allowed=bool(permit.handoff_allowed) if permit else False,
            ids_match=ids_match,
            trajectory_header_match=trajectory_header_match,
            command_valid=command_valid,
            command_forward_only=command_forward_only,
        )

    def _current_state_lattice_state(
        self, now_sec: float
    ) -> StateLatticeMuxState:
        message = self.state_lattice_cmd
        command_fresh = self._fresh(
            self.state_lattice_cmd_time_sec,
            now_sec,
            self.state_lattice_cmd_timeout_sec,
        )
        command_valid = bool(self.state_lattice_cmd_valid and message is not None)
        authority_valid = bool(
            command_valid
            and message is not None
            and bool(message.safety_evaluation_enabled)
            and bool(message.inputs_fresh)
            and bool(message.costmap_valid)
            and bool(message.trajectory_valid)
            and bool(message.trajectory_safe)
            and not bool(message.stop_required)
        )
        return StateLatticeMuxState(
            enabled=self.state_lattice_instant_control_enabled,
            active=bool(message.active) if message is not None else False,
            command_fresh=command_fresh,
            authority_valid=authority_valid,
            command_valid=command_valid,
            command_forward_only=bool(
                command_valid
                and message is not None
                and float(message.command.longitudinal.speed) >= 0.0
            ),
        )

    def _publish_debug(
        self,
        *,
        now_sec: float,
        source: str,
        reason: str,
        fallback_active: bool,
        solved_cycles: int,
        mpc_cmd_fresh: bool,
        pure_pursuit_cmd_fresh: bool,
        health: MpcHealth,
        recovery_state: RecoveryMuxState,
        selected_input_cmd: Optional[AckermannControlCommand],
        selected_pp_motion_sample: Optional[PurePursuitMotionSample],
        pp_selection_debug_snapshot: Optional[dict[str, object]],
        output_cmd: AckermannControlCommand,
        steering_result: SteeringLimitResult,
        constraint_decision: SafetyConstraintDecision,
        control_loop_deadline_missed: bool,
        control_publish_gap_sec: float,
        ros_clock_stalled: bool,
        ros_clock_stagnant_duration_sec: float,
        control_fault_latched: bool,
        control_fault_clear_cycles: int,
        overtake_plan_fresh: bool,
        overtake_plan_valid: bool,
        authority_plan_generation: Optional[int],
        tracking_plan_generation: int,
        lateral_stop_plan_authorized: bool,
        baseline_stop_plan_authorized: bool,
        planner_stop_release_bootstrap_valid: bool,
        lateral_stop_tracking_usable: bool,
        baseline_stop_tracking_usable: bool,
        lateral_stop_steering_hold_usable: bool,
        baseline_stop_steering_hold_usable: bool,
        lateral_stop_contract_delivery_gap_authorized: bool,
        pp_tracking_delivery_gap_usable: bool,
        historical_authority_release_lateral_bridge: bool,
        last_verified_tracking_steering_age_sec: float,
        controller_tracking_status: ControllerTrackingStatus,
        pure_pursuit_tracking_status_fresh: bool,
        motion_pp_tracking_proof_usable: bool,
        motion_pp_tracking_proof_first_false: str,
        motion_pp_tracking_proof_blockers: list[str],
        motion_pp_tracking_exact_tuple_present: bool,
        motion_pp_tracking_expected_stamp_ns: Optional[int],
        motion_pp_tracking_expected_generation: int,
        motion_pp_tracking_observed_stamp_ns: Optional[int],
        motion_pp_tracking_observed_generation: Optional[int],
        motion_pp_tracking_same_stamp_generations: list[int],
        motion_pp_tracking_exact_status_fresh: bool,
        motion_pp_tracking_exact_status_valid: bool,
        motion_pp_tracking_pp_command_fresh: bool,
        motion_pp_tracking_upstream_pp_command_fresh: bool,
        motion_pp_tracking_upstream_trajectory_usable: bool,
        motion_pp_tracking_binding_required: bool,
        motion_pp_tracking_binding_matches: bool,
        motion_pp_tracking_steering_usable: bool,
        motion_pp_tracking_continuity_applicable: bool,
        motion_pp_tracking_continuity_usable: bool,
        motion_pp_tracking_attack_follow_continuity_usable: bool,
        motion_pp_tracking_pass_continuity_usable: bool,
        force_publish: bool = False,
    ) -> None:
        if self.debug_publish_period_sec <= 0.0:
            return
        if (
            not force_publish
            and now_sec - self.last_debug_publish_sec
            < self.debug_publish_period_sec
        ):
            return
        self.last_debug_publish_sec = now_sec

        pp_evaluation = self._debug_pure_pursuit_evaluation_snapshot(
            selected_pp_motion_sample,
            selection_debug_snapshot=pp_selection_debug_snapshot,
        )
        if (
            source == "stop"
            and reason == "pure_pursuit_cmd_timeout"
            and bool(pp_evaluation["selection_rejected_for_invalid_tracking"])
            and pp_evaluation["selection_rejection_kind"] == "invalid_tracking"
        ):
            stop_context = "timeout_with_invalid_tracking_selector_rejection"
        elif source == "stop":
            stop_context = str(reason) or "other_stop"
        else:
            stop_context = "not_stop"
        # The top-level arbitration reason is authoritative.  Selector context
        # cannot retroactively turn a watchdog timeout into an invalid-input
        # causal proof.
        final_stop_origin = str(reason) if source == "stop" else "not_stop"
        final_stop_origin_proven = False

        msg = String()
        msg.data = json.dumps(
            {
                "controller": "hybrid_control_mux",
                "primary_source": self.core.config.primary_source,
                "source": source,
                "selected_source": source,
                "source_previous": self.last_source,
                "source_changed": source != self.last_source,
                "fallback_active": fallback_active,
                "reason": reason,
                "final_stop_origin": final_stop_origin,
                "final_stop_origin_proven": final_stop_origin_proven,
                "stop_context": stop_context,
                "solved_cycles": solved_cycles,
                "mpc_cmd_fresh": mpc_cmd_fresh,
                "pure_pursuit_cmd_fresh": pure_pursuit_cmd_fresh,
                "state_lattice_instant_control_enabled": (
                    self.state_lattice_instant_control_enabled
                ),
                "state_lattice_episode_latched": (
                    self.core.state_lattice_episode_latched
                ),
                "state_lattice_command_fresh": self._fresh(
                    self.state_lattice_cmd_time_sec,
                    now_sec,
                    self.state_lattice_cmd_timeout_sec,
                ),
                "state_lattice_command_valid": self.state_lattice_cmd_valid,
                "state_lattice_command_reason": self.state_lattice_cmd_reason,
                "mpc_health_valid": health.valid,
                "mpc_status": health.status,
                "mpc_infeasible_count": health.infeasible_count,
                "mpc_health_age_sec": health.age_sec if math.isfinite(health.age_sec) else None,
                "recovery_enabled": recovery_state.enabled,
                "recovery_latched": self.core.recovery_episode_latched,
                "recovery_status_state": recovery_state.status_state,
                "recovery_status_fresh": recovery_state.status_fresh,
                "recovery_command_fresh": recovery_state.command_fresh,
                "recovery_permit_fresh": recovery_state.permit_fresh,
                "recovery_external_safety_ok": recovery_state.external_safety_ok,
                "recovery_ids_match": recovery_state.ids_match,
                "recovery_trajectory_header_match": recovery_state.trajectory_header_match,
                "selected_input_stamp_sec": self._stamp_sec(selected_input_cmd),
                "selected_input_stamp_nanosec": self._stamp_nanosec(selected_input_cmd),
                # Keep the input snapshot separate from selected_input: an
                # invalid envelope intentionally has no selected motion sample,
                # but its exact cached command/envelope pair is still useful
                # read-only evidence for a fail-closed stop.
                "evaluated_pure_pursuit_command_stamp_sec": (
                    pp_evaluation["command_stamp_sec"]
                ),
                "evaluated_pure_pursuit_command_stamp_nanosec": (
                    pp_evaluation["command_stamp_nanosec"]
                ),
                "evaluated_pure_pursuit_envelope_command_stamp_sec": (
                    pp_evaluation["envelope_command_stamp_sec"]
                ),
                "evaluated_pure_pursuit_envelope_command_stamp_nanosec": (
                    pp_evaluation["envelope_command_stamp_nanosec"]
                ),
                "evaluated_pure_pursuit_tracking_usable": (
                    pp_evaluation["tracking_usable"]
                ),
                "evaluated_pure_pursuit_selection_rejected_for_invalid_tracking": (
                    pp_evaluation["selection_rejected_for_invalid_tracking"]
                ),
                "evaluated_pure_pursuit_rejection_kind": (
                    pp_evaluation["selection_rejection_kind"]
                ),
                "output_stamp_sec": self._stamp_sec(output_cmd),
                "output_stamp_nanosec": self._stamp_nanosec(output_cmd),
                "output_speed_mps": output_cmd.longitudinal.speed,
                "output_accel_mps2": output_cmd.longitudinal.acceleration,
                "output_steer_rad": output_cmd.lateral.steering_tire_angle,
                "raw_steer_rad": steering_result.raw_steering_rad,
                "limited_steer_rad": steering_result.limited_steering_rad,
                "steering_delta_rad": steering_result.steering_delta_rad,
                "steering_angle_limited": steering_result.angle_limited,
                "steering_rate_limited": steering_result.rate_limited,
                "steering_limiter_reset": steering_result.limiter_reset,
                "actuator_hard_steering_limit_rad": (
                    self.steering_limiter.config.max_steering_angle_rad
                ),
                "tracking_usable_max_steering_angle_rad": (
                    self.tracking_usable_max_steering_angle_rad
                ),
                "pure_pursuit_input_steer_rad": (
                    float(
                        self.pure_pursuit_cmd.lateral.steering_tire_angle
                    )
                    if self.pure_pursuit_cmd is not None
                    and math.isfinite(
                        float(
                            self.pure_pursuit_cmd.lateral.steering_tire_angle
                        )
                    )
                    else None
                ),
                "safety_constraint_enabled": self.safety_constraint_enabled,
                "safety_constraint_required": self.require_safety_constraint,
                "safety_constraint_reason": constraint_decision.reason,
                "safety_constraint_stop_required": constraint_decision.stop_required,
                "safety_constraint_generation": constraint_decision.constraint_generation,
                "safety_constraint_plan_generation": constraint_decision.plan_generation,
                "safety_constraint_speed_limit_mps": constraint_decision.speed_limit_mps,
                "safety_constraint_required_brake_decel_mps2": (
                    constraint_decision.required_brake_decel_mps2
                ),
                "overtake_plan_fresh": overtake_plan_fresh,
                "overtake_plan_valid": overtake_plan_valid,
                "authority_plan_generation": authority_plan_generation,
                "tracking_plan_generation": tracking_plan_generation,
                "lateral_stop_plan_authorized": lateral_stop_plan_authorized,
                "baseline_stop_plan_authorized": (
                    baseline_stop_plan_authorized
                ),
                "planner_stop_release_bootstrap_valid": (
                    planner_stop_release_bootstrap_valid
                ),
                "lateral_stop_tracking_usable": lateral_stop_tracking_usable,
                "baseline_stop_tracking_usable": (
                    baseline_stop_tracking_usable
                ),
                "lateral_stop_steering_hold_usable": (
                    lateral_stop_steering_hold_usable
                ),
                "baseline_stop_steering_hold_usable": (
                    baseline_stop_steering_hold_usable
                ),
                "lateral_stop_contract_delivery_gap_authorized": (
                    lateral_stop_contract_delivery_gap_authorized
                ),
                "pp_tracking_delivery_gap_usable": (
                    pp_tracking_delivery_gap_usable
                ),
                "historical_authority_release_lateral_bridge": (
                    historical_authority_release_lateral_bridge
                ),
                "last_verified_tracking_steering_rad": (
                    self.last_verified_tracking_steering_rad
                ),
                "last_verified_lateral_stop_plan_generation": (
                    self.last_verified_lateral_stop_plan_generation
                ),
                "last_verified_baseline_stop_plan_generation": (
                    self.last_verified_baseline_stop_plan_generation
                ),
                "last_verified_tracking_steering_age_sec": (
                    last_verified_tracking_steering_age_sec
                    if math.isfinite(last_verified_tracking_steering_age_sec)
                    else None
                ),
                "controller_tracking_plan_generation": int(
                    controller_tracking_status.plan_generation
                ),
                "controller_tracking_usable": bool(
                    controller_tracking_status.trajectory_tracking_usable
                ),
                "controller_tracking_safety_constraint_release_ready": bool(
                    controller_tracking_status.safety_constraint_release_ready
                ),
                "controller_tracking_attack_follow_stop_transport_release_ready": bool(
                    controller_tracking_status.attack_follow_stop_transport_release_ready
                ),
                "controller_tracking_reason": str(
                    controller_tracking_status.reason
                ),
                "pure_pursuit_tracking_received": (
                    self.pure_pursuit_tracking_status is not None
                ),
                "pure_pursuit_tracking_fresh": pure_pursuit_tracking_status_fresh,
                "pure_pursuit_tracking_valid": self.pure_pursuit_tracking_status_valid,
                "pure_pursuit_tracking_reason": (
                    str(self.pure_pursuit_tracking_status.reason)
                    if self.pure_pursuit_tracking_status is not None
                    else "missing"
                ),
                "pure_pursuit_envelope_shadow_received": (
                    self.pure_pursuit_envelope_last_identity is not None
                ),
                "pure_pursuit_envelope_shadow_valid": (
                    self.pure_pursuit_envelope_last_valid
                ),
                "pure_pursuit_envelope_shadow_reason": (
                    self.pure_pursuit_envelope_last_reason
                ),
                "pure_pursuit_envelope_shadow_identity": (
                    self.pure_pursuit_envelope_last_identity
                ),
                "pure_pursuit_envelope_shadow_active_producer_instance_id": (
                    self.pure_pursuit_envelope_active_producer_instance_id
                ),
                "pure_pursuit_envelope_shadow_candidate_valid_count": (
                    self.pure_pursuit_envelope_candidate_valid_count
                ),
                "pure_pursuit_envelope_shadow_fault_latched": (
                    self.pure_pursuit_envelope_fault_latched
                ),
                "pure_pursuit_envelope_shadow_fault_reason": (
                    self.pure_pursuit_envelope_fault_reason
                ),
                "pure_pursuit_envelope_shadow_cache_size": len(
                    self.pure_pursuit_envelope_cache
                ),
                "pure_pursuit_envelope_shadow_receipt_age_sec": (
                    self._pure_pursuit_envelope_receipt_age_sec(now_sec)
                    if math.isfinite(
                        self._pure_pursuit_envelope_receipt_age_sec(now_sec)
                    )
                    else None
                ),
                "pure_pursuit_envelope_shadow_receipt_fresh": (
                    self._pure_pursuit_envelope_receipt_fresh(now_sec)
                ),
                "pure_pursuit_envelope_shadow_ros_clock_usable": (
                    not ros_clock_stalled
                ),
                "pure_pursuit_envelope_shadow_usable": (
                    self._pure_pursuit_envelope_shadow_usable(
                        now_sec, ros_clock_stalled
                    )
                ),
                "pure_pursuit_envelope_shadow_parity_valid": (
                    self.pure_pursuit_envelope_last_parity_valid
                ),
                "pure_pursuit_envelope_shadow_parity_reason": (
                    self.pure_pursuit_envelope_last_parity_reason
                ),
                "free_run_live_exact_observe_enabled": (
                    self.free_run_live_exact_observe_enabled
                ),
                "free_run_live_exact_ack_received": (
                    self.free_run_live_exact_ack is not None
                ),
                "free_run_live_exact_ack_valid": (
                    self.free_run_live_exact_ack_valid
                ),
                "free_run_live_exact_ack_reason": (
                    self.free_run_live_exact_ack_reason
                ),
                "free_run_live_exact_ack_identity": (
                    self.free_run_live_exact_ack_identity
                ),
                "free_run_live_exact_ack_age_sec": (
                    now_sec - self.free_run_live_exact_ack_time_sec
                    if self.free_run_live_exact_ack_time_sec is not None
                    else None
                ),
                "free_run_live_exact_source_key_received": (
                    self.free_run_live_exact_source_key is not None
                ),
                "free_run_live_exact_source_key_valid": (
                    self.free_run_live_exact_source_key_valid
                ),
                "free_run_live_exact_source_key_reason": (
                    self.free_run_live_exact_source_key_reason
                ),
                "free_run_live_exact_source_key_identity": (
                    self.free_run_live_exact_source_key_identity
                ),
                "free_run_live_exact_source_key_age_sec": (
                    now_sec - self.free_run_live_exact_source_key_time_sec
                    if self.free_run_live_exact_source_key_time_sec is not None
                    else None
                ),
                "free_run_live_exact_selected_envelope_valid": (
                    self.free_run_live_exact_selected_envelope_valid
                ),
                "free_run_live_exact_selected_envelope_reason": (
                    self.free_run_live_exact_selected_envelope_reason
                ),
                "free_run_live_exact_selected_envelope_identity": (
                    self.free_run_live_exact_selected_envelope_identity
                ),
                "free_run_live_exact_forward_transition_reason": (
                    self.free_run_live_exact_forward_transition_reason
                ),
                "free_run_live_exact_published_record_valid": (
                    self.free_run_live_exact_published_record is not None
                ),
                "free_run_live_exact_published_record_reason": (
                    self.free_run_live_exact_record_reason
                ),
                "free_run_live_exact_published_record_age_sec": (
                    now_sec
                    - self.free_run_live_exact_published_record.publish_steady_time_sec
                    if self.free_run_live_exact_published_record is not None
                    else None
                ),
                "free_run_live_exact_published_record_steering_rad": (
                    self.free_run_live_exact_published_record.published_steering_rad
                    if self.free_run_live_exact_published_record is not None
                    else None
                ),
                "free_run_source_gap_hold_episode_count": (
                    self.free_run_source_gap_hold_episode_count
                ),
                "free_run_source_gap_exact_promotion_count": (
                    self.free_run_source_gap_exact_promotion_count
                ),
                "free_run_source_gap_lease_expiry_count": (
                    self.free_run_source_gap_lease_expiry_count
                ),
                "motion_pp_tracking_proof_usable": (
                    motion_pp_tracking_proof_usable
                ),
                "motion_pp_tracking_proof_first_false": (
                    motion_pp_tracking_proof_first_false
                ),
                "motion_pp_tracking_proof_blockers": (
                    motion_pp_tracking_proof_blockers
                ),
                "motion_pp_tracking_exact_tuple_present": (
                    motion_pp_tracking_exact_tuple_present
                ),
                "motion_pp_tracking_expected_stamp_ns": (
                    motion_pp_tracking_expected_stamp_ns
                ),
                "motion_pp_tracking_expected_generation": (
                    motion_pp_tracking_expected_generation
                ),
                "motion_pp_tracking_selected_envelope_producer_instance_id": (
                    selected_pp_motion_sample.envelope_identity[0]
                    if selected_pp_motion_sample is not None
                    and selected_pp_motion_sample.envelope_identity is not None
                    else None
                ),
                "motion_pp_tracking_selected_envelope_command_sequence": (
                    selected_pp_motion_sample.envelope_identity[1]
                    if selected_pp_motion_sample is not None
                    and selected_pp_motion_sample.envelope_identity is not None
                    else None
                ),
                "motion_pp_tracking_selected_envelope_receipt_age_sec": (
                    now_sec - selected_pp_motion_sample.command_receipt_time_sec
                    if selected_pp_motion_sample is not None
                    and selected_pp_motion_sample.envelope_identity is not None
                    else None
                ),
                "motion_pp_tracking_observed_stamp_ns": (
                    motion_pp_tracking_observed_stamp_ns
                ),
                "motion_pp_tracking_observed_generation": (
                    motion_pp_tracking_observed_generation
                ),
                "motion_pp_tracking_same_stamp_generations": (
                    motion_pp_tracking_same_stamp_generations
                ),
                "motion_pp_tracking_exact_status_fresh": (
                    motion_pp_tracking_exact_status_fresh
                ),
                "motion_pp_tracking_exact_status_valid": (
                    motion_pp_tracking_exact_status_valid
                ),
                "motion_pp_tracking_pp_command_fresh": (
                    motion_pp_tracking_pp_command_fresh
                ),
                "motion_pp_tracking_upstream_pp_command_fresh": (
                    motion_pp_tracking_upstream_pp_command_fresh
                ),
                "motion_pp_tracking_upstream_trajectory_usable": (
                    motion_pp_tracking_upstream_trajectory_usable
                ),
                "motion_pp_tracking_binding_required": (
                    motion_pp_tracking_binding_required
                ),
                "motion_pp_tracking_binding_matches": (
                    motion_pp_tracking_binding_matches
                ),
                "motion_pp_tracking_steering_usable": (
                    motion_pp_tracking_steering_usable
                ),
                "motion_pp_tracking_continuity_applicable": (
                    motion_pp_tracking_continuity_applicable
                ),
                "motion_pp_tracking_continuity_usable": (
                    motion_pp_tracking_continuity_usable
                ),
                "motion_pp_tracking_attack_follow_continuity_usable": (
                    motion_pp_tracking_attack_follow_continuity_usable
                ),
                "motion_pp_tracking_pass_continuity_usable": (
                    motion_pp_tracking_pass_continuity_usable
                ),
                "external_stop_latched": self.external_stop_latched,
                "finish_stop_enabled": self.finish_stop_enabled,
                "finish_stop_latched": self.finish_stop_latch.latched,
                "finish_stop_phase": self.finish_stop_latch.phase,
                "finish_stop_epoch": self.finish_stop_latch.epoch,
                "finish_stop_last_vehicle_state": (
                    self.finish_stop_latch.last_vehicle_state
                ),
                "finish_stop_pending_after_disarm": (
                    self.finish_stop_latch.finish_pending_after_disarm
                ),
                "finish_stop_ready_pending_before_arm": (
                    self.finish_stop_latch.ready_pending_before_arm
                ),
                "finish_stop_start_pending_before_arm": (
                    self.finish_stop_latch.start_pending_before_arm
                ),
                "finish_stop_decel_mps2": self.finish_stop_decel_mps2,
                "finish_stop_max_steering_rad": (
                    self.finish_stop_max_steering_rad
                ),
                "finish_stop_steering_guard_trigger_rad": (
                    self.finish_stop_steering_guard_trigger_rad
                ),
                "finish_stop_lateral_guard_active": (
                    self.finish_stop_lateral_guard_active
                ),
                "finish_terminal_reference_timeout_sec": (
                    self.finish_terminal_reference_timeout_sec
                ),
                "finish_terminal_candidate_steering_rad": (
                    self.finish_terminal_candidate_steering_rad
                ),
                "finish_terminal_candidate_epoch": (
                    self.finish_terminal_candidate_epoch
                ),
                "finish_terminal_snapshot_steering_rad": (
                    self.finish_terminal_snapshot_steering_rad
                ),
                "finish_terminal_snapshot_epoch": (
                    self.finish_terminal_snapshot_epoch
                ),
                "last_tracking_steering_rad": self.last_tracking_steering_rad,
                "control_loop_deadline_missed": control_loop_deadline_missed,
                "ros_clock_stalled": ros_clock_stalled,
                "race_arm_required": self.race_arm_required,
                "ros_clock_motion_ready": self.ros_clock_motion_ready,
                "ros_clock_observation_reason": (
                    self.ros_clock_observation_reason
                ),
                "ros_clock_stagnant_duration_sec": (
                    ros_clock_stagnant_duration_sec
                    if math.isfinite(ros_clock_stagnant_duration_sec)
                    else None
                ),
                "control_fault_latched": control_fault_latched,
                "control_fault_reason": self.control_fault_reason,
                "control_fault_clear_cycles": control_fault_clear_cycles,
                "control_publish_gap_sec": (
                    control_publish_gap_sec
                    if math.isfinite(control_publish_gap_sec)
                    else None
                ),
            },
            separators=(",", ":"),
        )
        self.debug_pub.publish(msg)

    @staticmethod
    def _fresh(last_time_sec: Optional[float], now_sec: float, timeout_sec: float) -> bool:
        if last_time_sec is None:
            return False
        age_sec = now_sec - last_time_sec
        return math.isfinite(age_sec) and 0.0 <= age_sec <= timeout_sec

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @classmethod
    def _overtake_plan_lateral_stop_fingerprint(
        cls, msg: OvertakePlan
    ) -> tuple:
        """Capture the immutable plan/trajectory payload used by lease joins."""
        return (
            bool(msg.trajectory_authorized),
            bool(msg.lateral_maneuver_required),
            int(msg.attempt_id),
            str(msg.target_vehicle_id),
            int(msg.phase),
            int(msg.pass_direction),
            int(msg.lateral_stop_authority_kind),
            int(msg.lateral_stop_transaction_pass_direction),
            int(msg.lateral_stop_authority_token),
            tuple(
                (
                    int(point.time_from_start.sec),
                    int(point.time_from_start.nanosec),
                    float(point.pose.position.x),
                    float(point.pose.position.y),
                    float(point.pose.position.z),
                    float(point.pose.orientation.x),
                    float(point.pose.orientation.y),
                    float(point.pose.orientation.z),
                    float(point.pose.orientation.w),
                    float(point.longitudinal_velocity_mps),
                    float(point.lateral_velocity_mps),
                    float(point.acceleration_mps2),
                    float(point.heading_rate_rps),
                    float(point.front_wheel_angle_rad),
                    float(point.rear_wheel_angle_rad),
                )
                for point in msg.trajectory.points
            ),
        )

    @staticmethod
    def _tracking_status_binds_pure_pursuit_command(
        status: ControllerTrackingStatus,
        command: Optional[AckermannControlCommand],
    ) -> bool:
        """Return true only when a same-stamp proof names this PP command.

        A command carries no plan generation.  Consequently, when two plan
        generations share a ROS stamp, a status must additionally bind its
        measured command values before it can prove the later generation.
        """
        if command is None or not bool(
            getattr(status, "pp_command_binding_valid", False)
        ):
            return False
        expected = (
            float(command.longitudinal.speed),
            float(command.longitudinal.acceleration),
            float(command.lateral.steering_tire_angle),
        )
        observed = (
            float(getattr(status, "pp_command_speed_mps", math.nan)),
            float(getattr(status, "pp_command_acceleration_mps2", math.nan)),
            float(
                getattr(
                    status, "pp_command_steering_tire_angle_rad", math.nan
                )
            ),
        )
        return all(
            math.isfinite(value)
            for value in (*expected, *observed)
        ) and expected == observed

    @staticmethod
    def _motion_pp_tracking_proof_blockers(
        *,
        exact_tuple_present: bool,
        same_stamp_other_generation_present: bool,
        exact_tuple_valid: bool,
        binding_required: bool,
        binding_matches: bool,
        status_fresh: bool,
        pp_command_fresh: bool,
        upstream_pp_command_fresh: bool,
        steering_usable: bool,
        steering_blocked: bool,
        exact_proof_usable: bool,
        continuity_applicable: bool,
        continuity_usable: bool,
    ) -> list[str]:
        """Classify why the PP motion proof is unavailable.

        The result is diagnostic-only.  Its order mirrors motion authority:
        steering is an outer mandatory gate, then an eligible N-1 continuity
        bridge, followed by the strict current `(stamp, generation)` proof.
        Multiple blockers are retained so a transport mismatch cannot hide a
        simultaneous timeout or actuator-limit violation.
        """
        if steering_usable and (exact_proof_usable or continuity_usable):
            return []

        blockers: list[str] = []
        if steering_blocked:
            blockers.append("steering")
        if continuity_applicable and not continuity_usable:
            blockers.append("continuity")
        if not exact_proof_usable:
            if not exact_tuple_present:
                blockers.append(
                    "generation"
                    if same_stamp_other_generation_present
                    else "exact_tuple"
                )
                if not pp_command_fresh:
                    blockers.append("freshness")
            else:
                if binding_required and not binding_matches:
                    blockers.append("binding")
                if (
                    not status_fresh
                    or not pp_command_fresh
                    or not upstream_pp_command_fresh
                ):
                    blockers.append("freshness")
                if (
                    not exact_tuple_valid
                    and "binding" not in blockers
                    and "freshness" not in blockers
                ):
                    blockers.append("exact_tuple")
                if not any(
                    blocker in blockers
                    for blocker in ("binding", "freshness", "exact_tuple")
                ):
                    # The exact peer exists but its upstream trajectory proof
                    # is unusable.  Keep that failure in the exact-proof stage.
                    blockers.append("exact_tuple")
        return blockers

    @staticmethod
    def _remember_bounded(
        cache: dict, contract_key: int | tuple[int, int], value: object
    ) -> None:
        """Keep a small ordered rendezvous window for independent topics."""
        cache[contract_key] = value
        while len(cache) > 8:
            # Generation wraps from 16_777_215 to 1. Numeric-min eviction
            # would delete the newest wrapped entry and force a permanent
            # fail-closed rendezvous gap. Python dicts preserve insertion
            # order, so evict the genuinely oldest receipt instead.
            del cache[next(iter(cache))]

    def _advance_receipt_time(
        self,
        previous_receipt_time_sec: Optional[float],
        maximum_stamp_ns: Optional[int],
        incoming_stamp_ns: int,
    ) -> tuple[Optional[float], Optional[int], bool]:
        if maximum_stamp_ns is None or incoming_stamp_ns > maximum_stamp_ns:
            return self.now_sec(), incoming_stamp_ns, True
        return previous_receipt_time_sec, maximum_stamp_ns, False

    @staticmethod
    def _stamp_sec(cmd: Optional[AckermannControlCommand]) -> Optional[int]:
        if cmd is None:
            return None
        return int(cmd.stamp.sec)

    @staticmethod
    def _stamp_nanosec(cmd: Optional[AckermannControlCommand]) -> Optional[int]:
        if cmd is None:
            return None
        return int(cmd.stamp.nanosec)

    @staticmethod
    def _finite_clamp(value: float, lower: float, upper: float) -> float:
        if not math.isfinite(value):
            return lower
        return min(max(value, lower), upper)

    @staticmethod
    def _recovery_state_name(state: int) -> str:
        mapping = {
            int(RecoveryStatus.INACTIVE): "inactive",
            int(RecoveryStatus.STUCK_CONFIRMING): "stuck_confirming",
            int(RecoveryStatus.STOP_HOLD): "stop_hold",
            int(RecoveryStatus.ACTIVE): "active",
            int(RecoveryStatus.HANDOFF_VERIFY): "handoff_verify",
            int(RecoveryStatus.COMPLETE): "complete",
            int(RecoveryStatus.ABORT): "abort",
            int(RecoveryStatus.LOCKOUT): "lockout",
        }
        return mapping.get(int(state), "unknown")


def main() -> None:
    rclpy.init()
    node = HybridControlMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A subscription take can race the context invalidation during an
        # external launch shutdown.  Preserve genuine runtime errors while ROS
        # is still live.
        if rclpy.ok():
            raise
    finally:
        node.mark_mux_runtime_measurement_executor_quiesced()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
