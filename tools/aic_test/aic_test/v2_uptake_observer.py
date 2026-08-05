#!/usr/bin/env python3
"""Read-only evidence observer for the V2 planner-to-PP uptake edge.

This module normally subscribes only to the V2 proposal, its PP binding
telemetry, and ``/clock``.  The dedicated final-fence profile additionally
owns one typed, one-shot diagnostic request publisher.  Neither profile has an
authority endpoint.  A PASS is evidence of the bounded transport/uptake edge
only; it is never evidence of command, Mux, or actuator authority.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
from dataclasses import dataclass, field
from typing import Any, Mapping


PROPOSAL_TOPIC = "/planning/overtake/state_lattice/v2_proposal"
STATUS_TOPIC = "/debug/overtake/state_lattice/v2_binding_status"
CLOCK_TOPIC = "/clock"
BASE_ATTESTATION_TOPIC = "/control/overtake/state_lattice/v2_base_attestation"
QUIESCE_REQUEST_TOPIC = "/test/m4/state_lattice/v2_quiesce"
FINAL_FENCE_TOPIC = "/test/m4/state_lattice/v2_final_fence"
MAX_LEDGER_ENTRIES = 2048
MAX_STATUS_EVENTS_PER_IDENTITY = 32
MAX_CYCLE_EVENTS = 11
MAX_AVAILABILITY_CHAIN_CYCLES = 256
V2_QOS_DEPTH = 17
INITIAL_UPTAKE_DEADLINE_NS = 200_000_000
SHADOW_SAFETY_CAP_NS = 50_000_000
REJECT_DEADLINE_MISS = 5
REJECT_STALE = 6
REJECT_CLOCK_FAULT = 9
AVAILABILITY_REPLACED = 3
# Mirrors AuthorizedCartesianTrajectory.{SOURCE_MPC_HORIZON,
# SOURCE_REFERENCE_TRAJECTORY}.  Keep this local observer free from ROS type
# imports so focused ledger tests remain runtime-independent.
VALID_BASE_SOURCE_KINDS = frozenset((1, 2))
LABEL = "PP_UPTAKE_AVAILABILITY_VERIFIED_NON_AUTHORITATIVE"
QUALIFIER = (
    "deadline_bounded_continuous_pp_cycle_availability_with_exact_replacement; "
    "pp_application_mux_m4_not_verified"
)

# This is intentionally broader than the three subscriptions owned by this
# observer.  The graph check makes an accidental future authority endpoint a
# terminal evidence failure rather than silently ignoring it.
FORBIDDEN_AUTHORITY_TOPICS = frozenset(
    {
        "/awsim/admin/command",
        "/awsim/control_cmd",
        "/control/command/actuation_cmd",
        "/control/command/control_cmd",
        "/control/command/emergency_cmd",
        "/control/trajectory",
        "/overtake/reference_override",
        "/planning/scenario_planning/trajectory",
        "/race/arm",
    }
)
REQUIRED_COMPONENT_NODES = frozenset(
    {
        "state_lattice_overtake_planner_node",
        "simple_pure_pursuit_node",
        "aic_test_v2_direct_input_driver",
    }
)
PROVENANCE_TOPICS = {
    CLOCK_TOPIC: "aic_test_v2_direct_input_driver",
    PROPOSAL_TOPIC: "state_lattice_overtake_planner_node",
    STATUS_TOPIC: "simple_pure_pursuit_node",
    BASE_ATTESTATION_TOPIC: "simple_pure_pursuit_node",
}
EXPECTED_SUBSCRIBERS = {
    BASE_ATTESTATION_TOPIC: "state_lattice_overtake_planner_node",
}


def _canonical_unsigned(value: int, width: int) -> bytes:
    return int(value).to_bytes(width, byteorder="big", signed=False)


def _canonical_string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return _canonical_unsigned(len(encoded), 4) + encoded


def quiesce_request_sha256(
    *,
    schema_version: int,
    execution_nonce: str,
    expected_producer_instance_id: str,
    expected_session_id: str,
    sealed_epoch_id: int,
    request_id: int,
) -> bytes:
    """Match the Planner's versioned, length-prefixed canonical request."""
    payload = b"".join(
        (
            _canonical_string("state_lattice_v2_quiesce_request/v1"),
            _canonical_unsigned(schema_version, 1),
            _canonical_string(execution_nonce),
            _canonical_string(expected_producer_instance_id),
            _canonical_string(expected_session_id),
            _canonical_unsigned(sealed_epoch_id, 8),
            _canonical_unsigned(request_id, 8),
        )
    )
    return hashlib.sha256(payload).digest()


def final_fence_sha256(message: Any) -> bytes:
    """Canonicalize a typed fence exactly as the emitter does."""
    identity = message.final_identity
    fields = [
        _canonical_string("state_lattice_v2_final_fence/v1"),
        _canonical_unsigned(message.schema_version, 1),
        _canonical_string(message.execution_nonce),
        _canonical_string(message.producer_instance_id),
        _canonical_string(message.session_id),
        _canonical_unsigned(message.sealed_epoch_id, 8),
        _canonical_unsigned(message.fence_id, 8),
        _canonical_unsigned(message.request_id, 8),
        bytes(message.request_canonical_sha256),
        _canonical_unsigned(message.final_committed_ordinal, 8),
        _canonical_unsigned(message.successful_emission_count, 8),
        _canonical_unsigned(1 if message.final_identity_present else 0, 1),
    ]
    if message.final_identity_present:
        fields.extend(
            (
                _canonical_string(identity.producer_instance_id),
                _canonical_string(identity.session_id),
                _canonical_unsigned(identity.proposal_sequence, 8),
                _canonical_unsigned(identity.plan_generation, 4),
                _canonical_unsigned(identity.source_generation, 4),
                _canonical_unsigned(int(identity.source_stamp.sec) & 0xFFFFFFFF, 4),
                _canonical_unsigned(identity.source_stamp.nanosec, 4),
                _canonical_string(identity.frame_id),
                bytes(identity.canonical_sha256),
                _canonical_unsigned(identity.publish_monotonic_ns, 8),
            )
        )
    fields.extend(
        (
            _canonical_string(message.proposal_qos_fingerprint),
            _canonical_unsigned(message.quiesced_monotonic_ns, 8),
            _canonical_unsigned(message.fence_publish_monotonic_ns, 8),
        )
    )
    return hashlib.sha256(b"".join(fields)).digest()


@dataclass
class FinalFenceDrainState:
    """Bounded, arrival-order-independent terminal evidence state.

    This state never changes Planner or PP decisions.  Its sticky fault only
    prevents evidence credit.  Original time predicates are recorded once and
    cannot be repaired by delayed delivery during the terminal drain.
    """

    execution_nonce: str
    producer_instance_id: str
    session_id: str
    sealed_epoch_id: int
    proposal_qos_fingerprint: str
    capacity: int = MAX_LEDGER_ENTRIES
    frozen_pp_cycle_sequence: int | None = None
    request_digest: bytes | None = None
    request_publish_count: int = 0
    fence_receive_count: int = 0
    final_committed_ordinal: int | None = None
    successful_emission_count: int | None = None
    fence_digest: bytes | None = None
    fence_final_identity_key: tuple[Any, ...] | None = None
    ordinals: dict[int, tuple[Any, ...]] = field(default_factory=dict)
    original_time_predicates: dict[int, bool] = field(default_factory=dict)
    pp_lifecycle_by_ordinal: dict[int, tuple[tuple[Any, ...], int]] = field(
        default_factory=dict
    )
    sticky_fault: str | None = None

    def _fault(self, reason: str) -> None:
        if self.sticky_fault is None:
            self.sticky_fault = reason

    def record_proposal(
        self,
        key: tuple[Any, ...],
        *,
        original_time_predicate_valid: bool,
    ) -> None:
        ordinal = int(key[2])
        valid = (
            valid_identity(key)
            and key[0] == self.producer_instance_id
            and key[1] == self.session_id
            and 0 < ordinal <= self.capacity
        )
        if not valid:
            self._fault("fence_proposal_identity_or_ordinal_invalid")
            return
        previous = self.ordinals.get(ordinal)
        if previous is not None:
            self._fault(
                "fence_proposal_duplicate"
                if previous == key
                else "fence_proposal_identity_conflict"
            )
            return
        if (
            self.final_committed_ordinal is not None
            and ordinal > self.final_committed_ordinal
        ):
            self._fault("proposal_ordinal_greater_than_fence")
            return
        self.ordinals[ordinal] = key
        self.original_time_predicates[ordinal] = bool(
            original_time_predicate_valid
        )

    def freeze_pp_boundary(self, pp_cycle_sequence: int) -> None:
        if self.frozen_pp_cycle_sequence is not None or pp_cycle_sequence <= 0:
            self._fault("terminal_pp_boundary_invalid")
            return
        self.frozen_pp_cycle_sequence = pp_cycle_sequence

    def record_pp_lifecycle(
        self, key: tuple[Any, ...], *, pp_cycle_sequence: int
    ) -> None:
        ordinal = int(key[2])
        if (
            not valid_identity(key)
            or key[0] != self.producer_instance_id
            or key[1] != self.session_id
            or not 0 < ordinal <= self.capacity
            or pp_cycle_sequence <= 0
        ):
            self._fault("pp_lifecycle_identity_or_cycle_invalid")
            return
        existing = self.pp_lifecycle_by_ordinal.get(ordinal)
        if existing is not None and existing[0] != key:
            self._fault("pp_lifecycle_identity_conflict")
            return
        if existing is None or pp_cycle_sequence < existing[1]:
            self.pp_lifecycle_by_ordinal[ordinal] = (key, pp_cycle_sequence)

    def make_request_fields(self) -> dict[str, Any] | None:
        if self.frozen_pp_cycle_sequence is None or self.request_publish_count != 0:
            self._fault("quiesce_request_order_or_multiplicity_invalid")
            return None
        digest = quiesce_request_sha256(
            schema_version=1,
            execution_nonce=self.execution_nonce,
            expected_producer_instance_id=self.producer_instance_id,
            expected_session_id=self.session_id,
            sealed_epoch_id=self.sealed_epoch_id,
            request_id=1,
        )
        self.request_digest = digest
        self.request_publish_count = 1
        return {
            "schema_version": 1,
            "execution_nonce": self.execution_nonce,
            "expected_producer_instance_id": self.producer_instance_id,
            "expected_session_id": self.session_id,
            "sealed_epoch_id": self.sealed_epoch_id,
            "request_id": 1,
            "canonical_sha256": digest,
        }

    def record_fence(self, message: Any) -> None:
        self.fence_receive_count += 1
        if self.fence_receive_count != 1:
            self._fault("final_fence_multiplicity_invalid")
            return
        try:
            final_ordinal = int(message.final_committed_ordinal)
            emission_count = int(message.successful_emission_count)
            valid = (
                self.request_publish_count == 1
                and self.request_digest is not None
                and message.schema_version == 1
                and message.execution_nonce == self.execution_nonce
                and message.producer_instance_id == self.producer_instance_id
                and message.session_id == self.session_id
                and message.sealed_epoch_id == self.sealed_epoch_id
                and message.fence_id == 1
                and message.request_id == 1
                and bytes(message.request_canonical_sha256) == self.request_digest
                and 0 <= final_ordinal <= self.capacity
                and emission_count == final_ordinal
                and message.quiesced_monotonic_ns > 0
                and message.fence_publish_monotonic_ns
                >= message.quiesced_monotonic_ns
                and message.proposal_qos_fingerprint
                == self.proposal_qos_fingerprint
                and bytes(message.canonical_sha256) == final_fence_sha256(message)
            )
            if final_ordinal == 0:
                valid = valid and not message.final_identity_present
            else:
                final_key = identity_key(message.final_identity)
                valid = (
                    valid
                    and message.final_identity_present
                    and valid_identity(final_key)
                    and final_key[0] == self.producer_instance_id
                    and final_key[1] == self.session_id
                    and final_key[2] == final_ordinal
                )
        except (AttributeError, OverflowError, TypeError, ValueError):
            valid = False
        if not valid:
            self._fault("final_fence_invalid")
            return
        self.final_committed_ordinal = final_ordinal
        self.successful_emission_count = emission_count
        self.fence_digest = bytes(message.canonical_sha256)
        self.fence_final_identity_key = (
            identity_key(message.final_identity)
            if message.final_identity_present
            else None
        )
        if any(ordinal > final_ordinal for ordinal in self.ordinals):
            self._fault("proposal_ordinal_greater_than_fence")

    def close(self, *, pp_lifecycle_closed_through: int) -> dict[str, Any]:
        final_ordinal = self.final_committed_ordinal
        expected_ordinals = (
            set(range(1, final_ordinal + 1))
            if final_ordinal is not None
            else set()
        )
        observed_ordinals = set(self.ordinals)
        if final_ordinal is None:
            self._fault("final_fence_missing")
        elif observed_ordinals != expected_ordinals:
            self._fault("fenced_ordinal_set_incomplete")
        if (
            self.sticky_fault is None
            and final_ordinal is not None
            and final_ordinal > 0
            and self.fence_final_identity_key
            != self.ordinals.get(final_ordinal)
        ):
            self._fault("final_fence_identity_proposal_mismatch")
        if (
            self.frozen_pp_cycle_sequence is None
            or pp_lifecycle_closed_through < self.frozen_pp_cycle_sequence
        ):
            self._fault("terminal_pp_lifecycle_unresolved")
        if self.sticky_fault is None and self.frozen_pp_cycle_sequence is not None:
            for ordinal in sorted(expected_ordinals):
                lifecycle = self.pp_lifecycle_by_ordinal.get(ordinal)
                if lifecycle is None:
                    self._fault("fenced_ordinal_pp_lifecycle_missing")
                    break
                if lifecycle[0] != self.ordinals.get(ordinal):
                    self._fault("fenced_ordinal_pp_identity_mismatch")
                    break
                if lifecycle[1] > self.frozen_pp_cycle_sequence:
                    self._fault(
                        "fenced_ordinal_pp_lifecycle_after_frozen_t"
                    )
                    break
        if any(not value for value in self.original_time_predicates.values()):
            self._fault("original_time_predicate_false")
        return {
            "valid": self.sticky_fault is None,
            "fault": self.sticky_fault,
            "request_publish_count": self.request_publish_count,
            "fence_receive_count": self.fence_receive_count,
            "final_committed_ordinal": final_ordinal,
            "successful_emission_count": self.successful_emission_count,
            "observed_ordinals": sorted(observed_ordinals),
            "pp_lifecycle_ordinals": sorted(self.pp_lifecycle_by_ordinal),
            "frozen_pp_cycle_sequence": self.frozen_pp_cycle_sequence,
            "pp_lifecycle_closed_through": pp_lifecycle_closed_through,
            "deadline_repair_permitted": False,
        }


def stamp_ns(stamp: Any) -> int:
    """Convert a ROS builtin time-like value to nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def identity_key(identity: Any) -> tuple[Any, ...]:
    """Return the full immutable identity tuple used by both message types."""
    digest = tuple(int(value) for value in identity.canonical_sha256)
    return (
        str(identity.producer_instance_id),
        str(identity.session_id),
        int(identity.proposal_sequence),
        int(identity.plan_generation),
        int(identity.source_generation),
        stamp_ns(identity.source_stamp),
        str(identity.frame_id),
        digest,
    )


def identity_json(key: tuple[Any, ...]) -> dict[str, Any]:
    return {
        "producer_instance_id": key[0],
        "session_id": key[1],
        "proposal_sequence": key[2],
        "plan_generation": key[3],
        "source_generation": key[4],
        "source_stamp_ns": key[5],
        "frame_id": key[6],
        "canonical_sha256": list(key[7]),
    }


def valid_identity(key: tuple[Any, ...]) -> bool:
    return (
        bool(key[0])
        and bool(key[1])
        and key[2] > 0
        and key[3] > 0
        and key[4] > 0
        and key[5] > 0
        and bool(key[6])
        and len(key[7]) == 32
        and any(key[7])
        and all(0 <= value <= 255 for value in key[7])
    )


def private_pp_binding_cycle_barrier_eligible(
    ledger: "UptakeLedger",
    *,
    status_recorded: bool,
    availability_summary: bool,
    pp_cycle_sequence: int,
    timer_entry_ros_ns: int,
    header_frame_id: str,
    timer_entry_monotonic_ns: int,
    cycle_event_count: int,
    run_invalid: bool,
    overflow_count: int,
    overflow_first_sequence: int,
    overflow_last_sequence: int,
) -> bool:
    """Accept only one clean, ordinary PP summary before Planner startup."""
    return (
        status_recorded
        and ledger.first_status_fault is None
        # Every ledger reason represents an observed evidence fault. Startup
        # has no allowed reason subset, so future reasons fail closed too.
        and not ledger.reasons
        and ledger.proposal_count == 0
        and availability_summary
        and pp_cycle_sequence > 0
        and timer_entry_ros_ns > 0
        and header_frame_id == "map"
        and timer_entry_monotonic_ns > 0
        # Before Planner starts, the ordinary PP cycle must be empty. A
        # positive count would be vulnerable to cross-topic callback order and
        # cannot prove that no proposal preceded this private barrier.
        and cycle_event_count == 0
        and not run_invalid
        and overflow_count == 0
        and overflow_first_sequence == 0
        and overflow_last_sequence == 0
    )


def private_pp_startup_clock_fault_summary_eligible(
    ledger: "UptakeLedger", key: tuple[Any, ...], status: Mapping[str, Any]
) -> bool:
    """Accept only the summary half of a PP-before-/clock fault pair."""
    stamp = int(status["header_stamp_ns"])
    sequence = int(status["pp_cycle_sequence"])
    return (
        ledger.pending_startup_clock_fault is None
        and not ledger.reasons
        and ledger.proposal_count == 0
        and status["availability_summary"]
        and not status["availability_present"]
        and not valid_identity(key)
        and not valid_identity(status["availability_identity"])
        and status["availability_safety_valid_until_ns"] == 0
        and status["availability_transition"] == 0
        and status["header_frame_id"] == "map"
        and stamp >= 0
        and (
            ledger.startup_last_header_stamp_ns is None
            or stamp >= ledger.startup_last_header_stamp_ns
        )
        and sequence > 0
        and (
            ledger.startup_last_pp_cycle_sequence is None
            or sequence > ledger.startup_last_pp_cycle_sequence
        )
        and not status["first_uptake_pass"]
        and status["hold_cycle_index"] == 0
        and status["reject_reason"] == 0
        and status["receive_monotonic_ns"] == 0
        and status["accepted_monotonic_ns"] == 0
        and status["run_invalid"] is True
        and status["overflow_count"] == 0
        and status["overflow_first_sequence"] == 0
        and status["overflow_last_sequence"] == 0
        and status["cycle_event_index"] == 0
        and status["cycle_event_count"] == 1
        and status["timer_entry_monotonic_ns"] > 0
    )


def private_pp_startup_clock_fault_event_eligible(
    ledger: "UptakeLedger", key: tuple[Any, ...], status: Mapping[str, Any]
) -> bool:
    """Match the sole permitted event to the pending startup summary."""
    pending = ledger.pending_startup_clock_fault
    if pending is None:
        return False
    summary = pending["status"]
    return (
        not status["availability_summary"]
        and not status["availability_present"]
        and not valid_identity(key)
        and not valid_identity(status["availability_identity"])
        and status["availability_safety_valid_until_ns"] == 0
        and status["availability_transition"] == 0
        and status["header_frame_id"] == "map"
        and status["header_stamp_ns"] == summary["header_stamp_ns"]
        and status["timer_entry_monotonic_ns"]
        == summary["timer_entry_monotonic_ns"]
        and status["pp_cycle_sequence"] == summary["pp_cycle_sequence"]
        and not status["first_uptake_pass"]
        and status["hold_cycle_index"] == 0
        and status["reject_reason"] == REJECT_CLOCK_FAULT
        and status["receive_monotonic_ns"] == 0
        and status["accepted_monotonic_ns"] == 0
        and status["run_invalid"] is True
        and status["overflow_count"] == 0
        and status["overflow_first_sequence"] == 0
        and status["overflow_last_sequence"] == 0
        and status["cycle_event_index"] == 0
        and status["cycle_event_count"] == 1
    )


def private_pp_startup_clean_stamp_eligible(
    ledger: "UptakeLedger", header_stamp_ns: int
) -> bool:
    """A clean epoch may begin only after the latest recovered clock fault."""
    return (
        ledger.startup_last_header_stamp_ns is None
        or header_stamp_ns > ledger.startup_last_header_stamp_ns
    )


def private_prelaunch_permit_eligible(ledger: "UptakeLedger") -> bool:
    """Freeze the private startup epoch immediately before Planner launch."""
    return (
        ledger.experimental_epoch_status_ordinal is not None
        and ledger.proposal_count == 0
        and ledger.first_status_fault is None
        and not ledger.reasons
    )


def process_private_prelaunch_request(
    request_file: Path,
    permit_file: Path,
    ledger: "UptakeLedger",
    *,
    expected_run_nonce: str,
) -> tuple[dict[str, Any] | None, str | None]:
    """Consume one request against the current ledger and atomically permit it."""
    if not request_file.is_file():
        return None, None
    # A permit is one-shot.  Reusing an old permit would move the epoch boundary
    # away from the ledger that was just re-evaluated.
    if permit_file.exists():
        return None, "prelaunch_permit_exists"
    try:
        request = json.loads(request_file.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None, "prelaunch_request_invalid"
    if request != {"schema_version": 1, "run_nonce": expected_run_nonce}:
        return None, "prelaunch_request_invalid"
    if not private_prelaunch_permit_eligible(ledger):
        return None, "prelaunch_permit_denied"
    permit = {
        "schema_version": 1,
        "run_nonce": expected_run_nonce,
        "epoch_boundary": "observer_prelaunch_permit",
        "proposal_count": ledger.proposal_count,
        "status_fault": False,
        "blocking_reasons": [],
        "experimental_epoch_status_ordinal": ledger.experimental_epoch_status_ordinal,
        "permit_status_ordinal": ledger.status_message_count,
    }
    _write_json_atomic(permit_file, permit)
    ledger.prelaunch_permit_status_ordinal = ledger.status_message_count
    return permit, None


@dataclass
class ProposalRecord:
    safety_valid_until_ns: int
    header_stamp_ros_ns: int
    payload_plan_stamp_ros_ns: int
    base_lease_valid_until_ns: int | None = None
    base_source_kind: int | None = None


@dataclass
class ResolutionState:
    proposal: ProposalRecord | None = None
    primary_resolution_count: int = 0
    first_uptake_count: int = 0
    first_uptake: dict[str, Any] | None = None
    first_uptake_cycle_summary: dict[str, Any] | None = None
    pre_uptake_reject: dict[str, Any] | None = None
    post_uptake_stale: dict[str, Any] | None = None
    post_uptake_stale_count: int = 0
    post_stale_reappearance_count: int = 0
    contradictory_event: dict[str, Any] | None = None
    orphan_continuation: dict[str, Any] | None = None
    resolved: bool = False


@dataclass
class CycleState:
    sequence: int
    summary: dict[str, Any]
    expected_event_count: int
    previous_summary: dict[str, Any] | None = None
    events: dict[int, dict[str, Any]] = field(default_factory=dict)
    first_uptake_count: int = 0


@dataclass
class UptakeLedger:
    """Small fail-closed in-memory ledger, independent of ROS runtime."""

    capacity: int = MAX_LEDGER_ENTRIES
    resolutions: dict[tuple[Any, ...], ResolutionState] = field(default_factory=dict)
    proposal_count: int = 0
    proposal_before_binding_cycle_count: int = 0
    status_identity_count: int = 0
    initial_eligible_key: tuple[Any, ...] | None = None
    anchor_key: tuple[Any, ...] | None = None
    anchor_sequence: int | None = None
    availability_window: dict[int, dict[str, Any]] = field(default_factory=dict)
    active_cycle: CycleState | None = None
    last_completed_cycle_summary: dict[str, Any] | None = None
    pending_cycle_sequence: int | None = None
    pending_cycle_events: dict[
        int, tuple[tuple[Any, ...], dict[str, Any]]
    ] = field(default_factory=dict)
    last_availability_sequence: int | None = None
    clock_sample_count: int = 0
    clock_first_ns: int | None = None
    clock_last_ns: int | None = None
    reasons: set[str] = field(default_factory=set)
    overflowed: bool = False
    status_message_count: int = 0
    # Startup PP timer callbacks can precede the first delivered /clock sample.
    # Keep their first zero/positive records for diagnosis, but do not let them
    # become M4 experimental-epoch faults.  The epoch begins only when a clean
    # current-run PP availability summary is recorded below.
    startup_zero_status: dict[str, Any] | None = None
    startup_first_positive_status: dict[str, Any] | None = None
    startup_last_status: dict[str, Any] | None = None
    startup_status_count: int = 0
    startup_last_header_stamp_ns: int | None = None
    startup_last_pp_cycle_sequence: int | None = None
    startup_clock_fault_pair_count: int = 0
    pending_startup_clock_fault: dict[str, Any] | None = None
    experimental_epoch_status_ordinal: int | None = None
    prelaunch_permit_status_ordinal: int | None = None
    first_status_fault: dict[str, Any] | None = None
    first_reject: dict[str, Any] | None = None
    first_lifecycle_classification: dict[str, Any] | None = None

    def _state(self, key: tuple[Any, ...], reason: str) -> ResolutionState | None:
        state = self.resolutions.get(key)
        if state is not None:
            return state
        if len(self.resolutions) >= self.capacity:
            self.overflowed = True
            self.reasons.add(reason)
            return None
        state = ResolutionState()
        self.resolutions[key] = state
        return state

    def record_clock(self, clock_ns: int) -> None:
        if clock_ns <= 0:
            self.reasons.add("clock_nonpositive")
        if self.clock_last_ns is not None and clock_ns <= self.clock_last_ns:
            self.reasons.add("clock_not_strictly_increasing")
        if self.clock_first_ns is None:
            self.clock_first_ns = clock_ns
        self.clock_last_ns = clock_ns
        self.clock_sample_count += 1

    def record_startup_clock_fault_pair(
        self, summary: dict[str, Any], event: dict[str, Any]
    ) -> None:
        """Commit one verified startup clock-fault pair with bounded evidence."""
        record = {"summary": summary, "event": event}
        self.startup_status_count += 1
        self.startup_last_status = record
        self.startup_last_header_stamp_ns = summary["header_stamp_ns"]
        self.startup_last_pp_cycle_sequence = summary["pp_cycle_sequence"]
        self.startup_clock_fault_pair_count += 1
        self.pending_startup_clock_fault = None
        if summary["header_stamp_ns"] <= 0:
            if self.startup_zero_status is None:
                self.startup_zero_status = record
        elif self.startup_first_positive_status is None:
            self.startup_first_positive_status = record

    def begin_experimental_epoch(self, *, first_cycle_sequence: int) -> None:
        """Freeze the M4 accounting start at the recorded clean PP summary."""
        if self.experimental_epoch_status_ordinal is not None:
            raise RuntimeError("experimental_epoch_already_started")
        if first_cycle_sequence <= 0:
            raise ValueError("experimental_epoch_requires_positive_cycle")
        # The startup summaries are deliberately outside the experimental
        # ledger, so continuity starts at the candidate rather than assuming
        # it must be PP cycle one.
        self.last_availability_sequence = first_cycle_sequence - 1

    def seal_experimental_epoch_start(self) -> None:
        if self.experimental_epoch_status_ordinal is not None:
            raise RuntimeError("experimental_epoch_already_started")
        self.experimental_epoch_status_ordinal = self.status_message_count

    def record_status_extraction_fault(
        self, *, observer_receive_monotonic_ns: int, error_type: str
    ) -> None:
        """Preserve an ordered fault even when typed status extraction fails."""
        self.status_message_count += 1
        self.reasons.add("malformed_status")
        if self.first_status_fault is None:
            self.first_status_fault = {
                "arrival_index": self.status_message_count,
                "classifications": ["observer_malformed_status"],
                "failed_observer_predicates": ["status_field_extraction"],
                "event": {
                    "observer_receive_monotonic_ns": observer_receive_monotonic_ns,
                    "extraction_error_type": error_type,
                    "raw_fields_unavailable": True,
                },
            }

    def record_proposal(
        self,
        key: tuple[Any, ...],
        safety_valid_until_ns: int,
        header_stamp_ros_ns: int,
        *,
        payload_plan_stamp_ros_ns: int | None = None,
        base_lease_valid_until_ns: int | None = None,
        base_source_kind: int | None = None,
        before_binding_cycle: bool = False,
    ) -> None:
        if before_binding_cycle:
            self.proposal_before_binding_cycle_count += 1
            self.reasons.add("proposal_before_pp_binding_cycle")
        # The Planner's source-authoritative issuance timestamp is the outer
        # V2 header stamp.  Production binds it exactly to proposal.plan_stamp;
        # identity.source_stamp is the upstream/base trajectory source time.
        if payload_plan_stamp_ros_ns is None:
            payload_plan_stamp_ros_ns = header_stamp_ros_ns
        if (
            not valid_identity(key)
            or safety_valid_until_ns <= 0
            or header_stamp_ros_ns <= 0
            or payload_plan_stamp_ros_ns <= 0
            or payload_plan_stamp_ros_ns != header_stamp_ros_ns
        ):
            self.reasons.add("malformed_proposal")
            return
        state = self._state(key, "proposal_ledger_overflow")
        if state is None:
            return
        if state.proposal is not None:
            self.reasons.add("duplicate_proposal_identity")
            return
        if (
            base_source_kind is not None
            and (
                isinstance(base_source_kind, bool)
                or not isinstance(base_source_kind, int)
                or base_source_kind not in VALID_BASE_SOURCE_KINDS
            )
        ):
            self.reasons.add("invalid_base_source_kind")
        state.proposal = ProposalRecord(
            safety_valid_until_ns,
            header_stamp_ros_ns,
            payload_plan_stamp_ros_ns,
            base_lease_valid_until_ns,
            base_source_kind,
        )
        self.proposal_count += 1
        if self.initial_eligible_key is None and not before_binding_cycle:
            # Same-topic proposal callbacks preserve the proposal stream order.
            # Lock once; status arrival or later valid proposals cannot re-anchor
            # the epoch's initial uptake candidate.
            self.initial_eligible_key = key
        self._resolve_first_uptake(key, state)

    def record_status(
        self,
        key: tuple[Any, ...],
        *,
        header_stamp_ns: int,
        first_uptake_pass: bool,
        hold_cycle_index: int,
        reject_reason: int,
        receive_monotonic_ns: int,
        accepted_monotonic_ns: int,
        run_invalid: bool,
        overflow_count: int,
        header_frame_id: str = "",
        overflow_first_sequence: int = 0,
        overflow_last_sequence: int = 0,
        observer_receive_monotonic_ns: int = 0,
        cycle_event_index: int = 0,
        cycle_event_count: int = 1,
        pp_cycle_sequence: int = 0,
        availability_summary: bool = False,
        availability_present: bool = False,
        availability_identity: tuple[Any, ...] | None = None,
        availability_safety_valid_until_ns: int = 0,
        availability_transition: int = 0,
        timer_entry_monotonic_ns: int = 0,
    ) -> None:
        self.status_message_count += 1
        terminal_identity_valid = valid_identity(key)
        availability_identity_valid = (
            availability_identity is not None and valid_identity(availability_identity)
        )
        failed_predicates = []
        if header_stamp_ns <= 0:
            failed_predicates.append("header_stamp_positive")
        if not terminal_identity_valid and not availability_summary:
            failed_predicates.append("terminal_identity_valid_for_terminal_event")
        if availability_summary and availability_present and not availability_identity_valid:
            failed_predicates.append("availability_identity_valid_when_present")
        if availability_summary and (
            terminal_identity_valid
            or first_uptake_pass
            or hold_cycle_index != 0
            or reject_reason != 0
            or receive_monotonic_ns != 0
            or accepted_monotonic_ns != 0
            or cycle_event_index != 0
        ):
            failed_predicates.append("summary_terminal_fields_neutral")
        classifications = []
        if failed_predicates:
            classifications.append("observer_malformed_status")
        if run_invalid:
            classifications.append("production_binding_run_invalid")
        if classifications and self.first_status_fault is None:
            self.first_status_fault = {
                "arrival_index": self.status_message_count,
                "classifications": classifications,
                "failed_observer_predicates": sorted(failed_predicates),
                "event": {
                    "header_stamp_ns": header_stamp_ns,
                    "header_frame_id": header_frame_id,
                    "identity": identity_json(key),
                    "terminal_identity_valid": terminal_identity_valid,
                    "first_uptake_pass": bool(first_uptake_pass),
                    "hold_cycle_index": hold_cycle_index,
                    "reject_reason": reject_reason,
                    "receive_monotonic_ns": receive_monotonic_ns,
                    "accepted_monotonic_ns": accepted_monotonic_ns,
                    "run_invalid": bool(run_invalid),
                    "overflow_count": overflow_count,
                    "overflow_first_sequence": overflow_first_sequence,
                    "overflow_last_sequence": overflow_last_sequence,
                    "availability_summary": bool(availability_summary),
                    "pp_cycle_sequence": pp_cycle_sequence,
                    "availability_present": bool(availability_present),
                    "availability_identity": (
                        identity_json(availability_identity)
                        if availability_identity is not None
                        else None
                    ),
                    "availability_identity_valid": availability_identity_valid,
                    "availability_safety_valid_until_ns": (
                        availability_safety_valid_until_ns
                    ),
                    "availability_transition": availability_transition,
                    "timer_entry_monotonic_ns": timer_entry_monotonic_ns,
                    "cycle_event_index": cycle_event_index,
                    "cycle_event_count": cycle_event_count,
                    "observer_receive_monotonic_ns": observer_receive_monotonic_ns,
                },
            }
        if run_invalid:
            self.reasons.add("binding_run_invalid")
        if failed_predicates:
            self.reasons.add("malformed_status")
            return
        if overflow_count != 0:
            self.reasons.add("binding_overflow")
        event = {
            "identity": key if terminal_identity_valid else None,
            "header_stamp_ns": header_stamp_ns,
            "first_uptake_pass": bool(first_uptake_pass),
            "hold_cycle_index": hold_cycle_index,
            "reject_reason": reject_reason,
            "receive_monotonic_ns": receive_monotonic_ns,
            "accepted_monotonic_ns": accepted_monotonic_ns,
            "cycle_event_index": cycle_event_index,
            "cycle_event_count": cycle_event_count,
            "pp_cycle_sequence": pp_cycle_sequence,
            "availability_summary": bool(availability_summary),
            "availability_present": bool(availability_present),
            "availability_identity": availability_identity,
            "availability_safety_valid_until_ns": availability_safety_valid_until_ns,
            "availability_transition": availability_transition,
            "timer_entry_monotonic_ns": timer_entry_monotonic_ns,
            # The production publisher stamps both the cycle summary and every
            # event with the ROS time captured at PP timer entry.  Keep the
            # derived name explicit so it is not confused with DDS latency.
            "timer_entry_ros_ns": header_stamp_ns,
            "observer_receive_monotonic_ns": observer_receive_monotonic_ns,
            "arrival_index": self.status_message_count,
        }
        if availability_summary:
            self._record_availability_summary(event)
            return
        self._record_terminal_event(key, event)

    def _record_availability_summary(self, event: dict[str, Any]) -> None:
        sequence = event["pp_cycle_sequence"]
        if sequence <= 0:
            self.reasons.add("malformed_status")
            return
        self._finish_active_cycle()
        if self.last_availability_sequence is None:
            if sequence != 1:
                self.reasons.add("availability_cycle_start_mismatch")
        else:
            if sequence == self.last_availability_sequence:
                self.reasons.add("availability_cycle_duplicate")
                return
            if sequence < self.last_availability_sequence:
                self.reasons.add("availability_cycle_out_of_order")
                return
            if sequence != self.last_availability_sequence + 1:
                self.reasons.add("pp_cycle_sequence_gap")
        self.last_availability_sequence = sequence
        expected_event_count = event["cycle_event_count"]
        if expected_event_count < 0 or expected_event_count > MAX_CYCLE_EVENTS:
            self.reasons.add("cycle_event_count_out_of_production_range")
            return
        self.active_cycle = CycleState(
            sequence,
            event,
            expected_event_count,
            self.last_completed_cycle_summary,
        )
        if self.anchor_sequence is not None and sequence >= self.anchor_sequence:
            if len(self.availability_window) >= MAX_AVAILABILITY_CHAIN_CYCLES:
                self.reasons.add("availability_chain_capacity_exceeded")
            else:
                self.availability_window[sequence] = event
        if self.pending_cycle_sequence == sequence:
            pending = [
                self.pending_cycle_events[index]
                for index in sorted(self.pending_cycle_events)
            ]
            self.pending_cycle_sequence = None
            self.pending_cycle_events = {}
            for key, pending_event in pending:
                self._record_terminal_event(key, pending_event)
        elif (
            self.pending_cycle_sequence is not None
            and self.pending_cycle_sequence < sequence
        ):
            self.reasons.add("pending_terminal_summary_missing")

    def _record_terminal_event(
        self, key: tuple[Any, ...], event: dict[str, Any]
    ) -> None:
        cycle = self.active_cycle
        if cycle is None or event["pp_cycle_sequence"] != cycle.sequence:
            self._buffer_terminal_event(key, event)
            return
        if event["cycle_event_count"] != cycle.expected_event_count:
            self.reasons.add("cycle_event_count_mismatch")
            return
        index = event["cycle_event_index"]
        if not 0 <= index < cycle.expected_event_count:
            self.reasons.add("cycle_event_index_invalid")
            return
        if index in cycle.events:
            self.reasons.add("cycle_event_duplicate")
            return
        for field_name in (
            "header_stamp_ns",
            "availability_present",
            "availability_identity",
            "availability_safety_valid_until_ns",
            "availability_transition",
            "timer_entry_monotonic_ns",
        ):
            if event[field_name] != cycle.summary[field_name]:
                self.reasons.add("cycle_event_summary_mismatch")
                return
        cycle.events[index] = event
        previous = cycle.previous_summary
        event["cycle_entry_previous_pp_cycle_sequence"] = (
            previous["pp_cycle_sequence"] if previous is not None else 0
        )
        event["cycle_entry_previous_availability_present"] = (
            bool(previous["availability_present"])
            if previous is not None
            else False
        )
        event["cycle_entry_previous_availability_identity"] = (
            previous["availability_identity"] if previous is not None else None
        )
        event["cycle_entry_previous_safety_valid_until_ns"] = (
            previous["availability_safety_valid_until_ns"]
            if previous is not None
            else 0
        )
        event["cycle_entry_previous_transition"] = (
            previous["availability_transition"] if previous is not None else 0
        )
        event["cycle_entry_previous_timer_entry_ros_ns"] = (
            previous["timer_entry_ros_ns"] if previous is not None else 0
        )
        event["cycle_entry_previous_timer_entry_monotonic_ns"] = (
            previous["timer_entry_monotonic_ns"] if previous is not None else 0
        )
        state = self._state(key, "status_identity_ledger_overflow")
        if state is None:
            return
        contradictory = event["first_uptake_pass"] and (
            event["hold_cycle_index"] != 1 or event["reject_reason"] != 0
        )
        if contradictory:
            state.primary_resolution_count += 1
            state.resolved = False
            self.reasons.add("contradictory_first_uptake_event")
            if state.contradictory_event is None:
                state.contradictory_event = event
            self._record_first_lifecycle_classification(
                "contradictory_event", key, event
            )
            return
        is_first_uptake = (
            event["first_uptake_pass"]
            and event["hold_cycle_index"] == 1
            and event["reject_reason"] == 0
        )
        is_hold_continuation = (
            event["hold_cycle_index"] > 1 and event["reject_reason"] == 0
        )
        is_reject = event["reject_reason"] != 0
        if is_first_uptake:
            cycle.first_uptake_count += 1
            state.primary_resolution_count += 1
            state.first_uptake_count += 1
            if cycle.first_uptake_count > 1:
                self.reasons.add("multiple_first_uptakes_in_pp_cycle")
            if state.post_uptake_stale is not None:
                state.post_stale_reappearance_count += 1
                self.reasons.add("retired_identity_reappeared")
            if state.pre_uptake_reject is not None:
                state.resolved = False
                self.reasons.add("pre_uptake_reject_then_first_uptake")
                return
            if state.first_uptake_count != 1 or state.first_uptake is not None:
                state.resolved = False
                self.reasons.add("duplicate_terminal_resolution")
                return
        if is_hold_continuation:
            # A hold is an event in its PP cycle, but never a second terminal
            # resolution.  In particular, a later stale hold must not mutate an
            # already accepted identity; summary/identity validity remains
            # independently checked by the availability window.
            if state.first_uptake is None:
                self.reasons.add("orphan_continuation")
                if state.orphan_continuation is None:
                    state.orphan_continuation = event
                self._record_first_lifecycle_classification(
                    "contradictory_event", key, event
                )
            elif state.post_uptake_stale is not None:
                state.post_stale_reappearance_count += 1
                self.reasons.add("retired_identity_reappeared")
            return
        if is_reject:
            if state.first_uptake is None:
                state.primary_resolution_count += 1
                state.resolved = False
                self.reasons.add("pre_uptake_reject")
                self.reasons.add("proposal_rejected")
                if state.pre_uptake_reject is None:
                    state.pre_uptake_reject = event
                else:
                    self.reasons.add("duplicate_pre_uptake_reject")
                self._record_first_reject("pre_uptake_reject", key, event)
                self._record_first_lifecycle_classification(
                    "pre_uptake_reject", key, event
                )
            elif event["reject_reason"] == REJECT_STALE:
                state.post_uptake_stale_count += 1
                if state.post_uptake_stale is None:
                    state.post_uptake_stale = event
                else:
                    state.post_stale_reappearance_count += 1
                    self.reasons.add("retired_identity_reappeared")
                    self.reasons.add("duplicate_post_uptake_stale")
            else:
                if state.post_uptake_stale is not None:
                    state.post_stale_reappearance_count += 1
                    self.reasons.add("retired_identity_reappeared")
                self.reasons.add("illegal_post_uptake_reject")
                self._record_first_reject(
                    "contradictory_event", key, event
                )
                self._record_first_lifecycle_classification(
                    "contradictory_event", key, event
                )
            return
        if not is_first_uptake:
            self.reasons.add("invalid_terminal_lifecycle")
            self._record_first_lifecycle_classification(
                "contradictory_event", key, event
            )
            return
        self.status_identity_count += 1
        state.first_uptake = event
        state.first_uptake_cycle_summary = cycle.summary
        self._resolve_first_uptake(key, state)

    def _buffer_terminal_event(
        self, key: tuple[Any, ...], event: dict[str, Any]
    ) -> None:
        sequence = event["pp_cycle_sequence"]
        if sequence <= 0:
            self.reasons.add("malformed_status")
            return
        if (
            self.last_availability_sequence is not None
            and sequence <= self.last_availability_sequence
        ):
            self.reasons.add("terminal_cycle_sequence_mismatch")
            return
        if self.pending_cycle_sequence is None:
            self.pending_cycle_sequence = sequence
        elif self.pending_cycle_sequence != sequence:
            self.reasons.add("pending_cycle_sequence_conflict")
            return
        index = event["cycle_event_index"]
        if not 0 <= index < MAX_CYCLE_EVENTS:
            self.reasons.add("cycle_event_index_invalid")
            return
        if index in self.pending_cycle_events:
            self.reasons.add("cycle_event_duplicate")
            return
        if len(self.pending_cycle_events) >= MAX_CYCLE_EVENTS:
            self.reasons.add("cycle_event_count_out_of_production_range")
            return
        self.pending_cycle_events[index] = (key, event)

    def _record_first_reject(
        self,
        classification: str,
        key: tuple[Any, ...],
        event: dict[str, Any],
    ) -> None:
        if self.first_reject is not None:
            return
        self.first_reject = {
            "classification_result": classification,
            "identity": identity_json(key),
            "reject_reason": event["reject_reason"],
            "event": self._event_json(event),
        }

    def _record_first_lifecycle_classification(
        self,
        classification: str,
        key: tuple[Any, ...],
        event: dict[str, Any],
    ) -> None:
        if self.first_lifecycle_classification is not None:
            return
        self.first_lifecycle_classification = {
            "classification_result": classification,
            "identity": identity_json(key),
            "event": self._event_json(event),
        }

    @staticmethod
    def _event_json(event: dict[str, Any]) -> dict[str, Any]:
        return {
            **event,
            "identity": (
                identity_json(event["identity"])
                if event.get("identity") is not None
                else None
            ),
            "availability_identity": (
                identity_json(event["availability_identity"])
                if event.get("availability_identity") is not None
                else None
            ),
            "cycle_entry_previous_availability_identity": (
                identity_json(
                    event["cycle_entry_previous_availability_identity"]
                )
                if event.get("cycle_entry_previous_availability_identity")
                is not None
                else None
            ),
        }

    def _finish_active_cycle(self) -> None:
        cycle = self.active_cycle
        if cycle is None:
            return
        cycle_complete = set(cycle.events) == set(
            range(cycle.expected_event_count)
        )
        for event in cycle.events.values():
            event["cycle_complete"] = cycle_complete
        if not cycle_complete:
            self.reasons.add("cycle_event_incomplete")
        self.last_completed_cycle_summary = cycle.summary
        self.active_cycle = None

    def _resolve_first_uptake(
        self, key: tuple[Any, ...], state: ResolutionState
    ) -> dict[str, Any] | None:
        if (
            state.resolved
            and state.primary_resolution_count == 1
            and state.first_uptake_count == 1
            and state.pre_uptake_reject is None
            and state.contradictory_event is None
        ):
            self._reserve_initial_anchor(key, state)
            return self._uptake_json(key, state)
        state.resolved = False
        proposal = state.proposal
        first = state.first_uptake
        if (
            proposal is None
            or first is None
            or state.primary_resolution_count != 1
            or state.first_uptake_count != 1
            or state.pre_uptake_reject is not None
            or state.contradictory_event is not None
        ):
            return None
        if (
            not first["first_uptake_pass"]
            or first["hold_cycle_index"] != 1
            or first["pp_cycle_sequence"] <= 0
            or first["reject_reason"] != 0
            or first["header_stamp_ns"] >= proposal.safety_valid_until_ns
            or first["receive_monotonic_ns"] <= 0
            or first["accepted_monotonic_ns"] < first["receive_monotonic_ns"]
        ):
            return None
        state.resolved = True
        self._reserve_initial_anchor(key, state)
        if (
            self.anchor_key is not None
            and key != self.anchor_key
            and first["pp_cycle_sequence"] < int(self.anchor_sequence)
        ):
            self.reasons.add("first_uptake_before_immutable_anchor")
        return self._uptake_json(key, state)

    def _reserve_initial_anchor(
        self, key: tuple[Any, ...], state: ResolutionState
    ) -> None:
        """Reserve S0 once, and only for a passing initial eligible proposal."""
        if key != self.initial_eligible_key or self.anchor_key is not None:
            return
        _, passes = self._initial_exact_uptake_evidence()
        if not passes:
            return
        first = state.first_uptake
        summary = state.first_uptake_cycle_summary
        if first is None or summary is None:
            return
        self.anchor_key = key
        self.anchor_sequence = first["pp_cycle_sequence"]
        self.availability_window[self.anchor_sequence] = summary

    def _initial_exact_uptake_evidence(self) -> tuple[dict[str, Any], bool]:
        """Return bounded G3 evidence and the direct initial-uptake predicate."""
        key = self.initial_eligible_key
        state = self.resolutions.get(key) if key is not None else None
        proposal = state.proposal if state is not None else None
        first = state.first_uptake if state is not None else None
        pre_reject = state.pre_uptake_reject if state is not None else None
        contradictory = state.contradictory_event if state is not None else None

        exact_primary = state is not None and state.primary_resolution_count == 1
        exact_first = (
            exact_primary
            and state is not None
            and state.first_uptake_count == 1
            and first is not None
            and pre_reject is None
            and contradictory is None
        )
        exact_pre_reject = (
            exact_primary
            and state is not None
            and state.first_uptake_count == 0
            and first is None
            and pre_reject is not None
            and contradictory is None
        )
        terminal = first if exact_first else pre_reject if exact_pre_reject else None
        timer_entry_ros_ns = (
            int(terminal["timer_entry_ros_ns"]) if terminal is not None else None
        )
        proposal_source_time_ns = (
            proposal.header_stamp_ros_ns if proposal is not None else None
        )
        delta_ns = (
            timer_entry_ros_ns - proposal_source_time_ns
            if timer_entry_ros_ns is not None
            and proposal_source_time_ns is not None
            else None
        )
        source_semantics_valid = (
            proposal is not None
            and proposal.header_stamp_ros_ns > 0
            and proposal.payload_plan_stamp_ros_ns
            == proposal.header_stamp_ros_ns
        )
        predicates = {
            "initial_eligible_proposal_fixed": key is not None,
            "full_exact_identity_valid": key is not None and valid_identity(key),
            "proposal_source_time_present": (
                proposal_source_time_ns is not None
                and proposal_source_time_ns > 0
            ),
            "outer_header_equals_payload_plan_stamp": source_semantics_valid,
            "safety_valid_until_present": (
                proposal is not None and proposal.safety_valid_until_ns > 0
            ),
            "primary_resolution_exactly_one": exact_primary,
            "primary_terminal_exact_first": exact_first,
            "first_uptake_pass_true": (
                first is not None and bool(first["first_uptake_pass"])
            ),
            "hold_cycle_index_is_one": (
                first is not None and first["hold_cycle_index"] == 1
            ),
            "reject_reason_is_zero": (
                first is not None and first["reject_reason"] == 0
            ),
            "pp_cycle_sequence_positive": (
                first is not None and first["pp_cycle_sequence"] > 0
            ),
            "status_header_equals_pp_timer_entry_ros": (
                first is not None
                and first["header_stamp_ns"] == first["timer_entry_ros_ns"]
            ),
            "timer_entry_not_before_proposal_source": (
                delta_ns is not None and delta_ns >= 0
            ),
            "timer_entry_within_200ms": (
                delta_ns is not None and delta_ns < INITIAL_UPTAKE_DEADLINE_NS
            ),
            "timer_entry_before_safety_deadline": (
                first is not None
                and proposal is not None
                and first["timer_entry_ros_ns"] < proposal.safety_valid_until_ns
            ),
        }
        passes = all(predicates.values())
        terminal_kind = (
            "FIRST_UPTAKE"
            if exact_first
            else "PRE_UPTAKE_REJECT"
            if exact_pre_reject
            else "MISSING_OR_CONTRADICTORY"
        )
        return (
            {
                "predicate": "G3_INITIAL_EXACT_UPTAKE_DEADLINE_AND_PROVENANCE",
                "valid": passes,
                "initial_eligible_identity": (
                    identity_json(key) if key is not None else None
                ),
                "proposal_provenance": (
                    {
                        "proposal_source_time_ns": proposal_source_time_ns,
                        "payload_plan_stamp_ros_ns": (
                            proposal.payload_plan_stamp_ros_ns
                        ),
                        "safety_valid_until_ns": proposal.safety_valid_until_ns,
                        "source_time_semantics": (
                            "authorized_v2.header.stamp_equals_"
                            "authorized_v2.proposal.plan_stamp"
                        ),
                        "identity_source_stamp_semantics": (
                            "base_trajectory_source_time_not_used_for_200ms"
                        ),
                        "identity_source_stamp_ns": key[5] if key is not None else None,
                    }
                    if proposal is not None
                    else None
                ),
                "primary_terminal": (
                    {
                        "kind": terminal_kind,
                        "pp_cycle_sequence": terminal["pp_cycle_sequence"],
                        "pp_timer_entry_ros_ns": terminal["timer_entry_ros_ns"],
                        "status_header_stamp_ns": terminal["header_stamp_ns"],
                        "first_uptake_pass": terminal["first_uptake_pass"],
                        "hold_cycle_index": terminal["hold_cycle_index"],
                        "reject_reason": terminal["reject_reason"],
                        "observer_receive_monotonic_ns": terminal[
                            "observer_receive_monotonic_ns"
                        ],
                        "observer_receive_used_for_200ms_predicate": False,
                    }
                    if terminal is not None
                    else None
                ),
                "pp_timer_entry_minus_proposal_source_ns": delta_ns,
                "strict_initial_uptake_limit_ns": INITIAL_UPTAKE_DEADLINE_NS,
                "predicates": predicates,
                "reason5_causal_provenance": self._reason5_causal_provenance(
                    key, state
                ),
            },
            passes,
        )

    def _reason5_causal_provenance(
        self, key: tuple[Any, ...] | None, state: ResolutionState | None
    ) -> dict[str, Any]:
        """Classify only an existing-telemetry reason-5 causal chain.

        The status payload exposes PP callback and timer times only in the
        CLOCK_MONOTONIC domain.  Its header stamp is the timer-entry ROS time.
        Keep those domains separate: this is deliberately not a reconstructed
        callback ROS timestamp or a driver-clock inference.
        """
        proposal = state.proposal if state is not None else None
        reject = state.pre_uptake_reject if state is not None else None
        plan_ns = proposal.header_stamp_ros_ns if proposal is not None else 0
        base_lease_ns = (
            proposal.base_lease_valid_until_ns if proposal is not None else None
        )
        source_kind = proposal.base_source_kind if proposal is not None else None
        final_deadline_ns = (
            proposal.safety_valid_until_ns if proposal is not None else 0
        )
        shadow_cap_ns = (
            plan_ns + SHADOW_SAFETY_CAP_NS if plan_ns > 0 else None
        )
        branch = "INVALID"
        deadline_operands_valid = (
            isinstance(base_lease_ns, int)
            and not isinstance(base_lease_ns, bool)
            and base_lease_ns > plan_ns
            and isinstance(source_kind, int)
            and not isinstance(source_kind, bool)
            and source_kind in VALID_BASE_SOURCE_KINDS
            and shadow_cap_ns is not None
            and final_deadline_ns > plan_ns
        )
        if deadline_operands_valid:
            if base_lease_ns < shadow_cap_ns and final_deadline_ns == base_lease_ns:
                branch = "BASE_LEASE"
            elif shadow_cap_ns < base_lease_ns and final_deadline_ns == shadow_cap_ns:
                branch = "SHADOW_CAP"
            elif base_lease_ns == shadow_cap_ns and final_deadline_ns == base_lease_ns:
                branch = "TIE"

        previous_sequence = (
            int(reject["cycle_entry_previous_pp_cycle_sequence"])
            if reject is not None
            else 0
        )
        previous_timer_ros_ns = (
            int(reject["cycle_entry_previous_timer_entry_ros_ns"])
            if reject is not None
            else 0
        )
        previous_timer_monotonic_ns = (
            int(reject["cycle_entry_previous_timer_entry_monotonic_ns"])
            if reject is not None
            else 0
        )
        reject_sequence = int(reject["pp_cycle_sequence"]) if reject else 0
        reject_timer_ros_ns = int(reject["timer_entry_ros_ns"]) if reject else 0
        reject_timer_monotonic_ns = (
            int(reject["timer_entry_monotonic_ns"]) if reject else 0
        )
        callback_monotonic_ns = (
            int(reject["receive_monotonic_ns"]) if reject else 0
        )
        predicates = {
            "initial_eligible_proposal_fixed": key is not None,
            "full_exact_identity_valid": key is not None and valid_identity(key),
            "exactly_one_primary_pre_uptake_reject": (
                state is not None
                and state.primary_resolution_count == 1
                and state.first_uptake_count == 0
                and state.first_uptake is None
                and reject is not None
                and state.contradictory_event is None
            ),
            "reject_reason_is_deadline_miss": (
                reject is not None
                and reject["reject_reason"] == REJECT_DEADLINE_MISS
            ),
            "deadline_operands_exact_joinable": deadline_operands_valid,
            "base_source_kind_recognized": (
                isinstance(source_kind, int)
                and not isinstance(source_kind, bool)
                and source_kind in VALID_BASE_SOURCE_KINDS
            ),
            "effective_deadline_selection_valid": branch != "INVALID",
            "adjacent_pp_cycles": (
                previous_sequence > 0 and reject_sequence == previous_sequence + 1
            ),
            "timer_ros_operands_present": (
                previous_timer_ros_ns > 0 and reject_timer_ros_ns > 0
            ),
            "same_pp_process_clock_monotonic_operands_present": (
                previous_timer_monotonic_ns > 0
                and callback_monotonic_ns > 0
                and reject_timer_monotonic_ns > 0
            ),
            "callback_after_preceding_timer": (
                callback_monotonic_ns > previous_timer_monotonic_ns
            ),
            "rejecting_timer_after_callback": (
                reject_timer_monotonic_ns > callback_monotonic_ns
            ),
            "preceding_timer_before_effective_deadline": (
                previous_timer_ros_ns < final_deadline_ns
            ),
            "rejecting_timer_at_or_after_effective_deadline": (
                reject_timer_ros_ns >= final_deadline_ns
            ),
        }
        causal = all(predicates.values())
        return {
            "classification": (
                "MISSED_PRECEDING_TIMER_THEN_DEADLINE_REJECTED_AT_NEXT_TIMER"
                if causal
                else "UNRESOLVED"
            ),
            "clock_domain_provenance": {
                "callback_and_timer_order": "CLOCK_MONOTONIC",
                "deadline_comparison": "ROS_TIME",
                "callback_ros_timestamp": None,
                "driver_tick_or_history_used": False,
            },
            "identity": identity_json(key) if key is not None else None,
            "deadline": {
                "plan_stamp_ros_ns": plan_ns or None,
                "base_lease_valid_until_ros_ns": base_lease_ns,
                "base_source_kind": source_kind,
                "shadow_50ms_cap_ros_ns": shadow_cap_ns,
                "safety_valid_until_ros_ns": final_deadline_ns or None,
                "effective_deadline_selection": branch,
                "effective_deadline_selection_detail": (
                    "SHADOW_50MS_CAP" if branch == "SHADOW_CAP" else branch
                ),
            },
            "preceding_timer": {
                "pp_cycle_sequence": previous_sequence or None,
                "timer_entry_ros_ns": previous_timer_ros_ns or None,
                "timer_entry_monotonic_ns": previous_timer_monotonic_ns or None,
            },
            "callback": {
                "receive_monotonic_ns": callback_monotonic_ns or None,
            },
            "rejecting_timer": {
                "pp_cycle_sequence": reject_sequence or None,
                "timer_entry_ros_ns": reject_timer_ros_ns or None,
                "timer_entry_monotonic_ns": reject_timer_monotonic_ns or None,
                "reject_reason": reject["reject_reason"] if reject else None,
            },
            "predicates": predicates,
        }

    def _uptake_json(
        self, key: tuple[Any, ...], state: ResolutionState
    ) -> dict[str, Any] | None:
        if not state.resolved or state.proposal is None or state.first_uptake is None:
            return None
        proposal = state.proposal
        first = state.first_uptake
        return {
            "identity": identity_json(key),
            "identity_key": key,
            "pp_cycle_sequence": first["pp_cycle_sequence"],
            "proposal_header_stamp_ros_ns": proposal.header_stamp_ros_ns,
            "proposal_payload_plan_stamp_ros_ns": (
                proposal.payload_plan_stamp_ros_ns
            ),
            "safety_valid_until_ns": proposal.safety_valid_until_ns,
            "first_uptake_header_stamp_ns": first["header_stamp_ns"],
            "first_uptake_timer_entry_ros_ns": first["timer_entry_ros_ns"],
            "receive_monotonic_ns": first["receive_monotonic_ns"],
            "accepted_monotonic_ns": first["accepted_monotonic_ns"],
            "observer_receive_monotonic_ns": first[
                "observer_receive_monotonic_ns"
            ],
            "event_count_for_identity": state.first_uptake_count,
        }

    def _post_stale_provenance(
        self,
        key: tuple[Any, ...],
        state: ResolutionState,
        accepted_keys: set[tuple[Any, ...]],
    ) -> dict[str, Any]:
        stale = state.post_uptake_stale
        assert stale is not None
        replacement_key = stale["availability_identity"]
        replacement_state = self.resolutions.get(replacement_key)
        replacement_proposal = (
            replacement_state.proposal if replacement_state is not None else None
        )
        replacement_first = (
            replacement_state.first_uptake
            if replacement_state is not None
            else None
        )
        proposal = state.proposal
        previous_key = stale[
            "cycle_entry_previous_availability_identity"
        ]
        predicates = {
            "stale_reject_reason": stale["reject_reason"] == REJECT_STALE,
            "stale_cycle_complete": bool(stale.get("cycle_complete", False)),
            "previous_cycle_is_immediate": (
                stale["cycle_entry_previous_pp_cycle_sequence"] + 1
                == stale["pp_cycle_sequence"]
            ),
            "previous_cycle_availability_present": bool(
                stale["cycle_entry_previous_availability_present"]
            ),
            "previous_active_identity_is_retired": previous_key == key,
            "previous_active_deadline_exact": (
                proposal is not None
                and stale["cycle_entry_previous_safety_valid_until_ns"]
                == proposal.safety_valid_until_ns
            ),
            "previous_active_was_before_deadline": (
                proposal is not None
                and stale["cycle_entry_previous_timer_entry_ros_ns"]
                < proposal.safety_valid_until_ns
            ),
            "retired_identity_expired_at_stale_cycle": (
                proposal is not None
                and stale["timer_entry_ros_ns"]
                >= proposal.safety_valid_until_ns
            ),
            "availability_present": bool(stale["availability_present"]),
            "transition_replaced": (
                stale["availability_transition"] == AVAILABILITY_REPLACED
            ),
            "replacement_identity_valid": (
                replacement_key is not None and valid_identity(replacement_key)
            ),
            "replacement_identity_changed": (
                replacement_key is not None and replacement_key != key
            ),
            "same_producer_session": (
                replacement_key is not None
                and replacement_key[0] == key[0]
                and replacement_key[1] == key[1]
            ),
            "proposal_sequence_strictly_newer": (
                replacement_key is not None and replacement_key[2] > key[2]
            ),
            "plan_generation_strictly_newer": (
                replacement_key is not None and replacement_key[3] > key[3]
            ),
            "replacement_exact_first_uptake": replacement_key in accepted_keys,
            "replacement_first_uptake_same_cycle": (
                replacement_first is not None
                and replacement_first["pp_cycle_sequence"]
                == stale["pp_cycle_sequence"]
            ),
            "replacement_summary_deadline_exact": (
                replacement_proposal is not None
                and stale["availability_safety_valid_until_ns"]
                == replacement_proposal.safety_valid_until_ns
            ),
            "replacement_available_before_deadline": (
                replacement_proposal is not None
                and stale["timer_entry_ros_ns"]
                < replacement_proposal.safety_valid_until_ns
            ),
        }
        legal = all(predicates.values())
        first = state.first_uptake
        return {
            "classification_result": (
                "legal_retirement" if legal else "availability_expired"
            ),
            "retired_identity": identity_json(key),
            "proposal_safety_valid_until_ns": (
                proposal.safety_valid_until_ns if proposal is not None else None
            ),
            "accepted_event": (
                {
                    "pp_cycle_sequence": first["pp_cycle_sequence"],
                    "timer_entry_ros_ns": first["timer_entry_ros_ns"],
                    "status_header_stamp_ns": first["header_stamp_ns"],
                    "receive_monotonic_ns": first["receive_monotonic_ns"],
                    "accepted_monotonic_ns": first["accepted_monotonic_ns"],
                    "observer_receive_monotonic_ns": first[
                        "observer_receive_monotonic_ns"
                    ],
                }
                if first is not None
                else None
            ),
            "stale_event": {
                "arrival_index": stale["arrival_index"],
                "pp_cycle_sequence": stale["pp_cycle_sequence"],
                "timer_entry_ros_ns": stale["timer_entry_ros_ns"],
                "status_header_stamp_ns": stale["header_stamp_ns"],
                "hold_cycle_index": stale["hold_cycle_index"],
                "reject_reason": stale["reject_reason"],
                "receive_monotonic_ns": stale["receive_monotonic_ns"],
                "accepted_monotonic_ns": stale["accepted_monotonic_ns"],
                "observer_receive_monotonic_ns": stale[
                    "observer_receive_monotonic_ns"
                ],
            },
            "stale_cycle_summary": {
                "cycle_complete": bool(stale.get("cycle_complete", False)),
                "availability_present": stale["availability_present"],
                "transition": stale["availability_transition"],
                "availability_identity": (
                    identity_json(replacement_key)
                    if replacement_key is not None
                    else None
                ),
                "summary_deadline_ns": stale[
                    "availability_safety_valid_until_ns"
                ],
                "timer_entry_ros_ns": stale["timer_entry_ros_ns"],
                "timer_entry_monotonic_ns": stale[
                    "timer_entry_monotonic_ns"
                ],
            },
            "cycle_entry_previous_summary": {
                "pp_cycle_sequence": stale[
                    "cycle_entry_previous_pp_cycle_sequence"
                ],
                "availability_present": stale[
                    "cycle_entry_previous_availability_present"
                ],
                "availability_identity": (
                    identity_json(previous_key)
                    if previous_key is not None
                    else None
                ),
                "summary_deadline_ns": stale[
                    "cycle_entry_previous_safety_valid_until_ns"
                ],
                "transition": stale[
                    "cycle_entry_previous_transition"
                ],
                "timer_entry_ros_ns": stale[
                    "cycle_entry_previous_timer_entry_ros_ns"
                ],
                "timer_entry_monotonic_ns": stale[
                    "cycle_entry_previous_timer_entry_monotonic_ns"
                ],
            },
            "same_cycle_replacement": {
                "valid": legal,
                "predicates": predicates,
            },
        }

    def _availability_cycle_valid(
        self, event: dict[str, Any], accepted_keys: set[tuple[Any, ...]]
    ) -> bool:
        key = event["availability_identity"]
        state = self.resolutions.get(key)
        proposal = state.proposal if state is not None else None
        return (
            event["availability_summary"]
            and event["pp_cycle_sequence"] > 0
            and event["reject_reason"] == 0
            and event["availability_present"]
            and event["timer_entry_monotonic_ns"] > 0
            and key in accepted_keys
            and state is not None
            and state.resolved
            and proposal is not None
            and event["availability_safety_valid_until_ns"]
            == proposal.safety_valid_until_ns
            and event["header_stamp_ns"] < proposal.safety_valid_until_ns
        )

    def _deadline_availability_chain(
        self, accepted_keys: set[tuple[Any, ...]]
    ) -> list[dict[str, Any]] | None:
        """Validate continuous availability until expiry or exact replacement.

        The cycle whose timer entry is equal to the active generation deadline
        is outside that generation's validity interval.  It is retained as the
        terminal boundary proof, but it is not credited as an availability
        cycle.  A different generation may continue the chain only when that
        very cycle reports an exact, strictly newer REPLACED uptake.
        """
        if self.anchor_key is None or self.anchor_sequence is None:
            return None
        current_key = self.anchor_key
        current_state = self.resolutions.get(current_key)
        current_proposal = (
            current_state.proposal if current_state is not None else None
        )
        if current_proposal is None:
            return None
        credited: list[dict[str, Any]] = []
        sequence = self.anchor_sequence
        first = True
        while True:
            event = self.availability_window.get(sequence)
            if event is None:
                return None
            timer_entry_ros_ns = event["header_stamp_ns"]
            event_key = event["availability_identity"]
            before_deadline = (
                timer_entry_ros_ns < current_proposal.safety_valid_until_ns
            )
            same_identity = event["availability_present"] and event_key == current_key
            replacement = event["availability_present"] and event_key != current_key

            if replacement:
                replacement_state = self.resolutions.get(event_key)
                replacement_proposal = (
                    replacement_state.proposal
                    if replacement_state is not None
                    else None
                )
                legal_replacement = (
                    event["availability_transition"] == AVAILABILITY_REPLACED
                    and self._availability_cycle_valid(event, accepted_keys)
                    and replacement_proposal is not None
                    and event_key[0] == current_key[0]
                    and event_key[1] == current_key[1]
                    and event_key[2] > current_key[2]
                    and event_key[3] > current_key[3]
                )
                if not before_deadline:
                    legal_replacement = legal_replacement and self._exact_expiry_event(
                        current_key,
                        sequence,
                        current_proposal.safety_valid_until_ns,
                    )
                if not legal_replacement:
                    return None
                credited.append(event)
                current_key = event_key
                current_proposal = replacement_proposal
                first = False
                sequence += 1
                continue

            if before_deadline:
                expected_transition = (
                    (1, AVAILABILITY_REPLACED) if first else (2,)
                )
                if (
                    not same_identity
                    or event["availability_transition"] not in expected_transition
                    or not self._availability_cycle_valid(event, accepted_keys)
                ):
                    return None
                credited.append(event)
                first = False
                sequence += 1
                continue

            # ``now >= deadline`` is the first cycle outside the obligation.
            # The expired identity must not remain available on that boundary.
            if event["availability_present"]:
                return None
            return (
                credited
                if self._exact_expiry_event(
                    current_key,
                    sequence,
                    current_proposal.safety_valid_until_ns,
                )
                else None
            )

    def _exact_expiry_event(
        self,
        key: tuple[Any, ...],
        sequence: int,
        deadline_ns: int,
    ) -> bool:
        state = self.resolutions.get(key)
        stale = state.post_uptake_stale if state is not None else None
        return (
            stale is not None
            and stale["reject_reason"] == REJECT_STALE
            and stale["pp_cycle_sequence"] == sequence
            and stale["header_stamp_ns"] >= deadline_ns
            and stale.get("cycle_complete") is True
        )

    def terminal(self) -> dict[str, Any]:
        self._finish_active_cycle()
        reasons = set(self.reasons)
        if self.pending_startup_clock_fault is not None:
            reasons.add("startup_clock_fault_event_pending")
        if self.pending_cycle_sequence is not None or self.pending_cycle_events:
            reasons.add("pending_terminal_summary_missing")
        if self.overflowed:
            reasons.add("observer_ledger_overflow")
        if (
            self.clock_sample_count < 2
            or self.clock_first_ns is None
            or self.clock_last_ns is None
            or self.clock_last_ns <= self.clock_first_ns
        ):
            reasons.add("clock_not_progressing")
        proposal_keys = {
            key for key, state in self.resolutions.items() if state.proposal
        }
        first_uptakes = {
            key: first_uptake
            for key in proposal_keys
            if (first_uptake := self._resolve_first_uptake(key, self.resolutions[key]))
        }
        accepted_keys = set(first_uptakes)
        pre_reject_observed_keys = {
            key
            for key, state in self.resolutions.items()
            if state.pre_uptake_reject is not None
        }
        pre_reject_keys = {
            key
            for key in pre_reject_observed_keys
            if (
                self.resolutions[key].primary_resolution_count == 1
                and self.resolutions[key].first_uptake is None
                and self.resolutions[key].contradictory_event is None
            )
        }
        post_stale_keys = {
            key
            for key, state in self.resolutions.items()
            if state.post_uptake_stale is not None
        }
        unresolved_keys = proposal_keys - accepted_keys - pre_reject_keys
        primary_multiplicity_keys = {
            key
            for key in proposal_keys
            if self.resolutions[key].primary_resolution_count > 1
        }
        if primary_multiplicity_keys:
            reasons.add("primary_resolution_multiplicity_invalid")
        status_keys = {
            key
            for key, state in self.resolutions.items()
            if (
                state.first_uptake is not None
                or state.pre_uptake_reject is not None
                or state.contradictory_event is not None
                or state.orphan_continuation is not None
            )
        }
        if proposal_keys != status_keys:
            reasons.add("proposal_status_identity_set_mismatch")
        partition_predicates = {
            "p_equals_a_union_rpre_union_u": (
                proposal_keys
                == accepted_keys | pre_reject_keys | unresolved_keys
            ),
            "a_rpre_u_pairwise_disjoint": not (
                accepted_keys & pre_reject_keys
                or accepted_keys & unresolved_keys
                or pre_reject_keys & unresolved_keys
            ),
            "rpost_subset_a": post_stale_keys <= accepted_keys,
            "u_is_no_valid_exactly_one_primary_resolution": all(
                not (
                    key in accepted_keys or key in pre_reject_keys
                )
                for key in unresolved_keys
            ),
        }
        if not all(partition_predicates.values()):
            reasons.add("proposal_terminal_partition_invalid")
        if unresolved_keys:
            reasons.add("proposal_without_exact_first_uptake")
        if not first_uptakes:
            reasons.add("no_exact_first_uptake")
        post_stale_provenance = [
            self._post_stale_provenance(
                key, self.resolutions[key], accepted_keys
            )
            for key in post_stale_keys
        ]
        for provenance in post_stale_provenance:
            if provenance["classification_result"] not in (
                "legal_retirement",
                "availability_expired",
            ):
                reasons.add("post_uptake_stale_without_legal_replacement")
        first_stale_provenance = min(
            post_stale_provenance,
            key=lambda item: item["stale_event"]["arrival_index"],
            default=None,
        )
        lifecycle_candidates: list[tuple[int, dict[str, Any]]] = []
        for key, state in self.resolutions.items():
            for classification, event in (
                ("pre_uptake_reject", state.pre_uptake_reject),
                ("contradictory_event", state.contradictory_event),
                ("contradictory_event", state.orphan_continuation),
            ):
                if event is not None:
                    lifecycle_candidates.append(
                        (
                            event["arrival_index"],
                            {
                                "classification_result": classification,
                                "identity": identity_json(key),
                                "event": self._event_json(event),
                            },
                        )
                    )
            if state.post_uptake_stale is not None:
                lifecycle_candidates.append(
                    (
                        state.post_uptake_stale["arrival_index"],
                        self._post_stale_provenance(
                            key, state, accepted_keys
                        ),
                    )
                )
        first_lifecycle_classification = (
            min(lifecycle_candidates, key=lambda item: item[0])[1]
            if lifecycle_candidates
            else None
        )
        initial_exact_uptake, initial_exact_uptake_valid = (
            self._initial_exact_uptake_evidence()
        )
        if not initial_exact_uptake_valid:
            reasons.add("initial_exact_uptake_predicate_failure")
        anchor = (
            self._uptake_json(self.anchor_key, self.resolutions[self.anchor_key])
            if self.anchor_key
            else None
        )
        window = self._deadline_availability_chain(accepted_keys)
        if anchor is not None and window is None:
            reasons.add("availability_window_predicate_failure")
        if window is None:
            reasons.add("deadline_availability_chain_incomplete")
        evidence = (
            {
                "first_uptakes": [
                    {
                        key: value
                        for key, value in uptake.items()
                        if key != "identity_key"
                    }
                    for uptake in first_uptakes.values()
                ],
                "availability_cycles": [
                    {
                        "identity": identity_json(event["availability_identity"]),
                        "pp_cycle_sequence": event["pp_cycle_sequence"],
                        "header_stamp_ns": event["header_stamp_ns"],
                        "availability_safety_valid_until_ns": event[
                            "availability_safety_valid_until_ns"
                        ],
                        "availability_transition": event["availability_transition"],
                        "timer_entry_monotonic_ns": event[
                            "timer_entry_monotonic_ns"
                        ],
                    }
                    for event in window
                ],
            }
            if window is not None
            else None
        )
        return {
            "outcome": "PASS" if not reasons else "INVALID_EVIDENCE",
            "label": LABEL if not reasons else None,
            "qualifier": QUALIFIER if not reasons else None,
            "reasons": sorted(reasons),
            "proposal_count": self.proposal_count,
            "epoch": {
                "proposal_before_pp_binding_cycle_count": (
                    self.proposal_before_binding_cycle_count
                ),
                "binding_cycle_before_all_proposals": (
                    self.proposal_before_binding_cycle_count == 0
                ),
            },
            "status_identity_count": self.status_identity_count,
            "clock": {
                "samples": self.clock_sample_count,
                "first_ns": self.clock_first_ns,
                "last_ns": self.clock_last_ns,
            },
            "initial_exact_uptake": initial_exact_uptake,
            "availability": evidence,
            "status_message_count": self.status_message_count,
            "startup_prepermit_status": {
                "first_zero": self.startup_zero_status,
                "first_positive": self.startup_first_positive_status,
                "last": self.startup_last_status,
                "count": self.startup_status_count,
            },
            "experimental_epoch": {
                "started_status_ordinal": self.experimental_epoch_status_ordinal,
                "permit_status_ordinal": self.prelaunch_permit_status_ordinal,
                "fault_accounting_starts_at_clean_pp_summary": True,
            },
            "first_status_fault": self.first_status_fault,
            "first_reject": self.first_reject,
            "first_lifecycle_classification": first_lifecycle_classification,
            "first_stale_provenance": first_stale_provenance,
            "proposal_terminal_partition": {
                "P": [identity_json(key) for key in sorted(proposal_keys)],
                "A": [identity_json(key) for key in sorted(accepted_keys)],
                "Rpre": [
                    identity_json(key) for key in sorted(pre_reject_keys)
                ],
                "Rpost": [
                    identity_json(key) for key in sorted(post_stale_keys)
                ],
                "U": [identity_json(key) for key in sorted(unresolved_keys)],
                "counts": {
                    "P": len(proposal_keys),
                    "A": len(accepted_keys),
                    "Rpre": len(pre_reject_keys),
                    "Rpost": len(post_stale_keys),
                    "U": len(unresolved_keys),
                },
                "predicates": partition_predicates,
                "primary_resolution_counts": [
                    {
                        "identity": identity_json(key),
                        "count": self.resolutions[
                            key
                        ].primary_resolution_count,
                    }
                    for key in sorted(proposal_keys)
                ],
                "observed_pre_reject_count": len(
                    pre_reject_observed_keys
                ),
                "primary_multiplicity_count": len(
                    primary_multiplicity_keys
                ),
            },
            "post_stale_latches": [
                {
                    "identity": identity_json(key),
                    "stale_event_count": self.resolutions[
                        key
                    ].post_uptake_stale_count,
                    "reappearance_count": self.resolutions[
                        key
                    ].post_stale_reappearance_count,
                    "reappeared": self.resolutions[
                        key
                    ].post_stale_reappearance_count
                    != 0,
                }
                for key in sorted(post_stale_keys)
            ],
            "stream_state": {
                "retained_identity_count": len(self.resolutions),
                "retained_s0_window_count": len(self.availability_window),
                "active_cycle_present": self.active_cycle is not None,
                "pending_cycle_sequence": self.pending_cycle_sequence,
                "pending_cycle_event_count": len(
                    self.pending_cycle_events
                ),
                "capacity": self.capacity,
                "capacity_overflowed": self.overflowed,
            },
        }


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False, suffix=".tmp"
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o644)
            json.dump(payload, stream, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        raise


def _terminal_payload_digest(payload: Mapping[str, Any]) -> str:
    canonical = json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def terminal_payload_hash_valid(payload: Mapping[str, Any]) -> bool:
    stored = payload.get("terminal_canonical_sha256")
    if not isinstance(stored, str) or len(stored) != 64:
        return False
    unsigned = dict(payload)
    del unsigned["terminal_canonical_sha256"]
    return stored == _terminal_payload_digest(unsigned)


def _rename_noreplace(source: Path, target: Path) -> None:
    """Linux atomic same-directory rename that refuses an existing target."""
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise OSError(errno.ENOSYS, "renameat2 unavailable")
    renameat2.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    )
    renameat2.restype = ctypes.c_int
    result = renameat2(
        -100,
        os.fsencode(source),
        -100,
        os.fsencode(target),
        1,
    )
    if result != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number), target)


def _write_terminal_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    """Persist, directory-sync, reread, and exact-validate one terminal result."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(path)
    final_payload = dict(payload)
    final_payload["terminal_canonical_sha256"] = _terminal_payload_digest(
        final_payload
    )
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            delete=False,
            suffix=".tmp",
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o644)
            json.dump(
                final_payload,
                stream,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        _rename_noreplace(temporary, path)
        temporary = None
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        reread = json.loads(path.read_text(encoding="utf-8"))
        if reread != final_payload or not terminal_payload_hash_valid(reread):
            raise ValueError("terminal_result_reread_validation_failed")
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def _component_graph_evidence(
    node: Any, *, final_fence_enabled: bool = False
) -> dict[str, Any]:
    graph_nodes = node.get_node_names_and_namespaces()
    component_endpoints = {}
    missing_component_nodes = []
    duplicate_component_nodes = []
    forbidden_component_endpoints = {}
    for component_name in sorted(REQUIRED_COMPONENT_NODES):
        matching_namespaces = [
            namespace for name, namespace in graph_nodes if name == component_name
        ]
        if not matching_namespaces:
            missing_component_nodes.append(component_name)
            continue
        if len(matching_namespaces) != 1:
            duplicate_component_nodes.append(component_name)
            continue
        namespace = matching_namespaces[0]
        publishers = {
            name: sorted(types)
            for name, types in node.get_publisher_names_and_types_by_node(
                component_name, namespace
            )
        }
        subscriptions = {
            name: sorted(types)
            for name, types in node.get_subscriber_names_and_types_by_node(
                component_name, namespace
            )
        }
        component_endpoints[component_name] = {
            "publishers": publishers,
            "subscriptions": subscriptions,
        }
        forbidden = sorted(
            FORBIDDEN_AUTHORITY_TOPICS.intersection(publishers)
            | FORBIDDEN_AUTHORITY_TOPICS.intersection(subscriptions)
        )
        if forbidden:
            forbidden_component_endpoints[component_name] = forbidden
    provenance_topics = dict(PROVENANCE_TOPICS)
    expected_subscribers = dict(EXPECTED_SUBSCRIBERS)
    if final_fence_enabled:
        provenance_topics.update(
            {
                QUIESCE_REQUEST_TOPIC: "aic_test_v2_uptake_observer",
                FINAL_FENCE_TOPIC: "state_lattice_overtake_planner_node",
            }
        )
        expected_subscribers[QUIESCE_REQUEST_TOPIC] = (
            "state_lattice_overtake_planner_node"
        )
    publisher_ownership = {
        topic: sorted(
            [
                f"{getattr(info, 'node_namespace', '/').rstrip('/')}/"
                f"{getattr(info, 'node_name', '')}"
                for info in node.get_publishers_info_by_topic(topic)
            ]
        )
        for topic in provenance_topics
    }
    expected_ownership = {
        topic: [f"/{node_name}"]
        for topic, node_name in provenance_topics.items()
    }
    provenance_failures = sorted(
        topic
        for topic, expected in expected_ownership.items()
        if publisher_ownership[topic] != expected
    )
    subscriber_ownership = {
        topic: sorted(
            [
                f"{getattr(info, 'node_namespace', '/').rstrip('/')}/"
                f"{getattr(info, 'node_name', '')}"
                for info in node.get_subscriptions_info_by_topic(topic)
            ]
        )
        for topic in expected_subscribers
    }
    expected_subscriber_ownership = {
        topic: [f"/{node_name}"]
        for topic, node_name in expected_subscribers.items()
    }
    subscriber_provenance_failures = sorted(
        topic
        for topic, expected in expected_subscriber_ownership.items()
        if subscriber_ownership[topic] != expected
    )
    return {
        "required_component_nodes": sorted(REQUIRED_COMPONENT_NODES),
        "missing_component_nodes": missing_component_nodes,
        "duplicate_component_nodes": duplicate_component_nodes,
        "component_endpoints": component_endpoints,
        "forbidden_component_endpoints": forbidden_component_endpoints,
        "proposal_and_status_publishers": publisher_ownership,
        "expected_proposal_and_status_publishers": expected_ownership,
        "publisher_ownership_failures": provenance_failures,
        "subscriber_ownership": subscriber_ownership,
        "expected_subscriber_ownership": expected_subscriber_ownership,
        "subscriber_ownership_failures": subscriber_provenance_failures,
        "ready": not (
            missing_component_nodes
            or duplicate_component_nodes
            or forbidden_component_endpoints
            or provenance_failures
            or subscriber_provenance_failures
        ),
    }


def _planner_disappearance_evidence(node: Any) -> dict[str, Any]:
    planner_nodes = [
        namespace
        for name, namespace in node.get_node_names_and_namespaces()
        if name == "state_lattice_overtake_planner_node"
    ]
    publishers = {
        topic: sorted(
            f"{getattr(info, 'node_namespace', '/').rstrip('/')}/"
            f"{getattr(info, 'node_name', '')}"
            for info in node.get_publishers_info_by_topic(topic)
        )
        for topic in (PROPOSAL_TOPIC, FINAL_FENCE_TOPIC)
    }
    ready = not planner_nodes and not any(publishers.values())
    return {
        "ready": ready,
        "planner_nodes": planner_nodes,
        "planner_publishers_after_stop": publishers,
        "cleanup_only_not_delivery_proof": True,
        "missing_component_nodes": [],
        "duplicate_component_nodes": [],
        "forbidden_component_endpoints": {},
        "publisher_ownership_failures": [] if ready else sorted(publishers),
    }


def observe(
    duration_s: float,
    *,
    ready_file: Path | None = None,
    sealed_file: Path | None = None,
    capture_complete_file: Path | None = None,
    terminal_graph_file: Path | None = None,
    first_fault_file: Path | None = None,
    pp_binding_cycle_file: Path | None = None,
    prelaunch_request_file: Path | None = None,
    prelaunch_permit_file: Path | None = None,
    expected_run_nonce: str | None = None,
    ready_timeout_s: float = 5.0,
    graph_seal_timeout_s: float = 5.0,
    final_fence_enabled: bool = False,
    final_fence_producer_instance_id: str = "",
    final_fence_session_id: str = "",
    final_fence_sealed_epoch_id: int = 0,
    final_fence_fixed_file: Path | None = None,
    planner_stopped_file: Path | None = None,
) -> dict[str, Any]:
    """Observe one bounded live run and return exactly one terminal JSON object."""
    import rclpy
    from multi_purpose_mpc_ros_msgs.msg import (
        AuthorizedCartesianTrajectoryV2,
        StateLatticeV2BindingStatus,
    )
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=None)
    node = Node("aic_test_v2_uptake_observer")
    v2_qos = QoSProfile(
        depth=V2_QOS_DEPTH,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    clock_qos = QoSProfile(
        depth=V2_QOS_DEPTH,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    ledger = UptakeLedger()
    proposal_qos_fingerprint = (
        "topic=/planning/overtake/state_lattice/v2_proposal;"
        "type=multi_purpose_mpc_ros_msgs/msg/"
        "AuthorizedCartesianTrajectoryV2;reliability=reliable;"
        "durability=volatile;history=keep_last;depth=8"
    )
    final_fence_state = (
        FinalFenceDrainState(
            execution_nonce=expected_run_nonce or "",
            producer_instance_id=final_fence_producer_instance_id,
            session_id=final_fence_session_id,
            sealed_epoch_id=final_fence_sealed_epoch_id,
            proposal_qos_fingerprint=proposal_qos_fingerprint,
        )
        if final_fence_enabled
        else None
    )
    quiesce_publisher = None
    pp_binding_cycle: dict[str, Any] | None = None
    prelaunch_permit: dict[str, Any] | None = None

    def maybe_write_prelaunch_permit() -> None:
        nonlocal prelaunch_permit
        if (
            prelaunch_permit is not None
            or prelaunch_request_file is None
            or prelaunch_permit_file is None
            or pp_binding_cycle is None
            or not prelaunch_request_file.is_file()
        ):
            return
        permit, denial_reason = process_private_prelaunch_request(
            prelaunch_request_file,
            prelaunch_permit_file,
            ledger,
            expected_run_nonce=expected_run_nonce,
        )
        if denial_reason is not None:
            ledger.reasons.add(denial_reason)
            return
        prelaunch_permit = permit

    def on_clock(message: Clock) -> None:
        ledger.record_clock(stamp_ns(message.clock))

    def on_proposal(message: AuthorizedCartesianTrajectoryV2) -> None:
        try:
            key = identity_key(message.identity)
            safety_valid_until_ns = stamp_ns(message.proposal.safety_valid_until)
            header_stamp_ros_ns = stamp_ns(message.header.stamp)
            ledger.record_proposal(
                key,
                safety_valid_until_ns,
                header_stamp_ros_ns=header_stamp_ros_ns,
                payload_plan_stamp_ros_ns=stamp_ns(
                    message.proposal.plan_stamp
                ),
                base_lease_valid_until_ns=stamp_ns(
                    message.proposal.base_lease_valid_until
                ),
                base_source_kind=int(message.proposal.base_source_kind),
                before_binding_cycle=(
                    expected_run_nonce is not None and pp_binding_cycle is None
                ),
            )
            if final_fence_state is not None:
                final_fence_state.record_proposal(
                    key,
                    original_time_predicate_valid=(
                        header_stamp_ros_ns > 0
                        and header_stamp_ros_ns < safety_valid_until_ns
                    ),
                )
        except (AttributeError, TypeError, ValueError):
            ledger.reasons.add("malformed_proposal")

    def on_status(message: StateLatticeV2BindingStatus) -> None:
        nonlocal pp_binding_cycle
        observer_receive_monotonic_ns = time.monotonic_ns()
        fault_before = ledger.first_status_fault
        status_recorded = False
        try:
            status_key = identity_key(message.identity)
            status_fields = {
                "header_stamp_ns": stamp_ns(message.header.stamp),
                "header_frame_id": str(message.header.frame_id),
                "first_uptake_pass": bool(message.first_uptake_pass),
                "hold_cycle_index": int(message.hold_cycle_index),
                "reject_reason": int(message.reject_reason),
                "receive_monotonic_ns": int(message.receive_monotonic_ns),
                "accepted_monotonic_ns": int(message.accepted_monotonic_ns),
                "run_invalid": bool(message.run_invalid),
                "overflow_count": int(message.overflow_count),
                "overflow_first_sequence": int(message.overflow_first_sequence),
                "overflow_last_sequence": int(message.overflow_last_sequence),
                "observer_receive_monotonic_ns": observer_receive_monotonic_ns,
                "cycle_event_index": int(message.cycle_event_index),
                "cycle_event_count": int(message.cycle_event_count),
                "pp_cycle_sequence": int(message.pp_cycle_sequence),
                "availability_summary": bool(message.availability_summary),
                "availability_present": bool(message.availability_present),
                "availability_identity": identity_key(message.availability_identity),
                "availability_safety_valid_until_ns": stamp_ns(
                    message.availability_safety_valid_until
                ),
                "availability_transition": int(message.availability_transition),
                "timer_entry_monotonic_ns": int(message.timer_entry_monotonic_ns),
            }
            # Before the test epoch, preserve exactly the PP-before-clock
            # fault *pairs* as bounded evidence.  It is not Planner-to-PP
            # acceptance evidence.
            if pp_binding_cycle is None:
                startup_candidate = False
                if ledger.pending_startup_clock_fault is not None:
                    if private_pp_startup_clock_fault_event_eligible(
                        ledger, status_key, status_fields
                    ):
                        ledger.record_startup_clock_fault_pair(
                            ledger.pending_startup_clock_fault["status"],
                            status_fields,
                        )
                        return
                    ledger.reasons.add("startup_clock_fault_pair_mismatch")
                elif private_pp_startup_clock_fault_summary_eligible(
                    ledger, status_key, status_fields
                ):
                    ledger.pending_startup_clock_fault = {
                        "status": status_fields,
                    }
                    return
                elif ledger.pending_startup_clock_fault is None:
                    startup_candidate = private_pp_binding_cycle_barrier_eligible(
                        ledger,
                        status_recorded=True,
                        availability_summary=status_fields["availability_summary"],
                        pp_cycle_sequence=status_fields["pp_cycle_sequence"],
                        timer_entry_ros_ns=status_fields["header_stamp_ns"],
                        header_frame_id=status_fields["header_frame_id"],
                        timer_entry_monotonic_ns=status_fields[
                            "timer_entry_monotonic_ns"
                        ],
                        cycle_event_count=status_fields["cycle_event_count"],
                        run_invalid=status_fields["run_invalid"],
                        overflow_count=status_fields["overflow_count"],
                        overflow_first_sequence=status_fields["overflow_first_sequence"],
                        overflow_last_sequence=status_fields["overflow_last_sequence"],
                    )
                # PP runs faster than /clock in this fixture.  Keep every
                # verified same-cycle clock-fault pair out of the experimental
                # ledger. Anything else is recorded normally and fails closed.
                if startup_candidate:
                    if not private_pp_startup_clean_stamp_eligible(
                        ledger, status_fields["header_stamp_ns"]
                    ):
                        ledger.reasons.add("startup_clock_recovery_not_advanced")
                    else:
                        ledger.begin_experimental_epoch(
                            first_cycle_sequence=status_fields["pp_cycle_sequence"]
                        )
            ledger.record_status(status_key, **status_fields)
            if (
                final_fence_state is not None
                and not status_fields["availability_summary"]
                and (
                    status_fields["first_uptake_pass"]
                    or status_fields["reject_reason"] != 0
                )
            ):
                final_fence_state.record_pp_lifecycle(
                    status_key,
                    pp_cycle_sequence=status_fields["pp_cycle_sequence"],
                )
            status_recorded = True
        except (AttributeError, TypeError, ValueError) as error:
            ledger.record_status_extraction_fault(
                observer_receive_monotonic_ns=observer_receive_monotonic_ns,
                error_type=type(error).__name__,
            )
        if (
            fault_before is None
            and ledger.first_status_fault is not None
            and first_fault_file is not None
        ):
            _write_json_atomic(first_fault_file, ledger.first_status_fault)
        if not status_recorded:
            return
        if pp_binding_cycle is None:
            try:
                timer_entry_ros_ns = status_fields["header_stamp_ns"]
                normal_status = private_pp_binding_cycle_barrier_eligible(
                    ledger,
                    status_recorded=status_recorded,
                    availability_summary=status_fields["availability_summary"],
                    pp_cycle_sequence=status_fields["pp_cycle_sequence"],
                    timer_entry_ros_ns=timer_entry_ros_ns,
                    header_frame_id=status_fields["header_frame_id"],
                    timer_entry_monotonic_ns=status_fields[
                        "timer_entry_monotonic_ns"
                    ],
                    cycle_event_count=status_fields["cycle_event_count"],
                    run_invalid=status_fields["run_invalid"],
                    overflow_count=status_fields["overflow_count"],
                    overflow_first_sequence=status_fields["overflow_first_sequence"],
                    overflow_last_sequence=status_fields["overflow_last_sequence"],
                )
            except (AttributeError, TypeError, ValueError):
                normal_status = False
                timer_entry_ros_ns = 0
            if normal_status and ledger.proposal_count == 0:
                ledger.seal_experimental_epoch_start()
                pp_binding_cycle = {
                    "schema_version": 1,
                    "run_nonce": expected_run_nonce,
                    "node_fqn": "/simple_pure_pursuit_node",
                    "status_topic": STATUS_TOPIC,
                    "pp_cycle_sequence": status_fields["pp_cycle_sequence"],
                    "timer_entry_ros_ns": timer_entry_ros_ns,
                    "header_frame_id": status_fields["header_frame_id"],
                    "timer_entry_monotonic_ns": status_fields[
                        "timer_entry_monotonic_ns"
                    ],
                    "cycle_event_count": status_fields["cycle_event_count"],
                    "observer_receive_monotonic_ns": observer_receive_monotonic_ns,
                    "proposal_count_at_barrier": ledger.proposal_count,
                    "experimental_epoch_status_ordinal": (
                        ledger.experimental_epoch_status_ordinal
                    ),
                    "normal_status_integrity": True,
                }
                if pp_binding_cycle_file is not None:
                    _write_json_atomic(pp_binding_cycle_file, pp_binding_cycle)

    node.create_subscription(Clock, CLOCK_TOPIC, on_clock, clock_qos)
    node.create_subscription(
        AuthorizedCartesianTrajectoryV2, PROPOSAL_TOPIC, on_proposal, v2_qos
    )
    node.create_subscription(
        StateLatticeV2BindingStatus, STATUS_TOPIC, on_status, v2_qos
    )
    if final_fence_state is not None:
        from multi_purpose_mpc_ros_msgs.msg import (
            StateLatticeV2FinalFence,
            StateLatticeV2QuiesceRequest,
        )

        final_fence_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        quiesce_publisher = node.create_publisher(
            StateLatticeV2QuiesceRequest,
            QUIESCE_REQUEST_TOPIC,
            final_fence_qos,
        )
        node.create_subscription(
            StateLatticeV2FinalFence,
            FINAL_FENCE_TOPIC,
            final_fence_state.record_fence,
            final_fence_qos,
        )
    if ready_file is not None:
        _write_json_atomic(ready_file, {"observer_ready": True})

    started_monotonic_ns = time.monotonic_ns()
    readiness_deadline = time.monotonic() + ready_timeout_s
    while (
        rclpy.ok()
        and expected_run_nonce is not None
        and pp_binding_cycle is None
        and time.monotonic() < readiness_deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.1)

    graph_seal_deadline = time.monotonic() + graph_seal_timeout_s
    sealed_graph_evidence: dict[str, Any] | None = None
    while rclpy.ok() and time.monotonic() < graph_seal_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        maybe_write_prelaunch_permit()
        candidate = _component_graph_evidence(
            node, final_fence_enabled=final_fence_enabled
        )
        if candidate["ready"] and (
            expected_run_nonce is None or pp_binding_cycle is not None
        ):
            sealed_graph_evidence = candidate
            if sealed_file is not None:
                _write_json_atomic(sealed_file, {"graph_sealed": True})
            break
    if sealed_graph_evidence is None:
        sealed_graph_evidence = _component_graph_evidence(
            node, final_fence_enabled=final_fence_enabled
        )

    # The proposal epoch begins at a received ordinary PP binding cycle. Its bounded capture/drain
    # interval starts after the complete Planner/PP/driver graph is sealed, so
    # startup time cannot consume that interval.
    capture_deadline = time.monotonic() + duration_s
    terminal_graph_evidence: dict[str, Any] | None = None
    quiesce_sent = False
    fence_fixed_written = False
    while rclpy.ok() and time.monotonic() < capture_deadline:
        rclpy.spin_once(
            node,
            timeout_sec=min(0.1, max(0.0, capture_deadline - time.monotonic())),
        )
        if (
            terminal_graph_evidence is None
            and capture_complete_file is not None
            and capture_complete_file.is_file()
        ):
            if final_fence_state is not None and not quiesce_sent:
                completed = ledger.last_completed_cycle_summary
                final_fence_state.freeze_pp_boundary(
                    int(completed["pp_cycle_sequence"])
                    if completed is not None
                    else 0
                )
                request_fields = final_fence_state.make_request_fields()
                if request_fields is not None and quiesce_publisher is not None:
                    request_message = StateLatticeV2QuiesceRequest()
                    for name, value in request_fields.items():
                        setattr(request_message, name, value)
                    quiesce_publisher.publish(request_message)
                quiesce_sent = True
            pp_closed_through = (
                int(ledger.last_completed_cycle_summary["pp_cycle_sequence"])
                if ledger.last_completed_cycle_summary is not None
                else 0
            )
            fence_closed = (
                final_fence_state is None
                or (
                    final_fence_state.final_committed_ordinal is not None
                    and set(final_fence_state.ordinals)
                    == set(
                        range(
                            1,
                            final_fence_state.final_committed_ordinal + 1,
                        )
                    )
                    and final_fence_state.frozen_pp_cycle_sequence is not None
                    and pp_closed_through
                    >= final_fence_state.frozen_pp_cycle_sequence
                    and final_fence_state.sticky_fault is None
                    and (
                        final_fence_state.final_committed_ordinal == 0
                        or final_fence_state.fence_final_identity_key
                        == final_fence_state.ordinals.get(
                            final_fence_state.final_committed_ordinal
                        )
                    )
                    and all(
                        ordinal in final_fence_state.pp_lifecycle_by_ordinal
                        and final_fence_state.pp_lifecycle_by_ordinal[ordinal][0]
                        == final_fence_state.ordinals[ordinal]
                        and final_fence_state.pp_lifecycle_by_ordinal[ordinal][1]
                        <= final_fence_state.frozen_pp_cycle_sequence
                        and final_fence_state.original_time_predicates[ordinal]
                        for ordinal in final_fence_state.ordinals
                    )
                )
            )
            if fence_closed:
                if final_fence_state is None:
                    terminal_graph_evidence = _component_graph_evidence(node)
                    if terminal_graph_file is not None:
                        _write_json_atomic(
                            terminal_graph_file, terminal_graph_evidence
                        )
                elif not fence_fixed_written:
                    if final_fence_fixed_file is not None:
                        marker = {
                            "schema_version": 1,
                            "execution_nonce": expected_run_nonce,
                            "sealed_epoch_id": final_fence_state.sealed_epoch_id,
                            "fence_canonical_sha256": (
                                final_fence_state.fence_digest.hex()
                                if final_fence_state.fence_digest is not None
                                else ""
                            ),
                            "final_committed_ordinal": (
                                final_fence_state.final_committed_ordinal
                            ),
                            "frozen_pp_cycle_sequence": (
                                final_fence_state.frozen_pp_cycle_sequence
                            ),
                            "fence_fixed": True,
                        }
                        validated_path = final_fence_fixed_file.with_name(
                            f"{final_fence_fixed_file.stem}.validated.json"
                        )
                        try:
                            _write_terminal_json_atomic(
                                final_fence_fixed_file, marker
                            )
                            persisted = json.loads(
                                final_fence_fixed_file.read_text(encoding="utf-8")
                            )
                            _write_terminal_json_atomic(
                                validated_path,
                                {
                                    "schema_version": 1,
                                    "execution_nonce": expected_run_nonce,
                                    "marker_terminal_canonical_sha256": persisted[
                                        "terminal_canonical_sha256"
                                    ],
                                    "marker_validated": True,
                                },
                            )
                            fence_fixed_written = True
                        except (OSError, TypeError, ValueError, json.JSONDecodeError):
                            final_fence_state._fault(
                                "fence_fixed_marker_persistence_failed"
                            )
        if (
            final_fence_state is not None
            and fence_fixed_written
            and planner_stopped_file is not None
            and planner_stopped_file.is_file()
        ):
            candidate = _planner_disappearance_evidence(node)
            if candidate["ready"]:
                terminal_graph_evidence = candidate
                if terminal_graph_file is not None:
                    _write_json_atomic(terminal_graph_file, candidate)
                break
    if terminal_graph_evidence is None:
        terminal_graph_evidence = (
            _planner_disappearance_evidence(node)
            if final_fence_state is not None and fence_fixed_written
            else _component_graph_evidence(
                node, final_fence_enabled=final_fence_enabled
            )
        )
        if terminal_graph_file is not None:
            _write_json_atomic(terminal_graph_file, terminal_graph_evidence)

    own_publishers = {
        name: sorted(types)
        for name, types in node.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace()
        )
    }
    own_subscriptions = {
        name: sorted(types)
        for name, types in node.get_subscriber_names_and_types_by_node(
            node.get_name(), node.get_namespace()
        )
    }
    node.destroy_node()
    rclpy.shutdown()
    if ledger.pending_startup_clock_fault is not None:
        ledger.reasons.add("startup_clock_fault_event_pending")
    payload = ledger.terminal()
    if final_fence_state is not None:
        pp_closed_through = (
            int(ledger.last_completed_cycle_summary["pp_cycle_sequence"])
            if ledger.last_completed_cycle_summary is not None
            else 0
        )
        final_fence = final_fence_state.close(
            pp_lifecycle_closed_through=pp_closed_through
        )
        payload["final_fence"] = final_fence
        if not final_fence["valid"]:
            payload["outcome"] = "INVALID_EVIDENCE"
            payload["label"] = None
            payload["reasons"] = sorted(
                set(payload["reasons"]) | {final_fence["fault"]}
            )
    if expected_run_nonce is not None and pp_binding_cycle is None:
        payload["outcome"] = "INVALID_EVIDENCE"
        payload["label"] = None
        payload["reasons"] = sorted(
            set(payload["reasons"]) | {"pp_binding_cycle_missing"}
        )
    authority_publishers = sorted(
        FORBIDDEN_AUTHORITY_TOPICS.intersection(own_publishers)
    )
    authority_subscriptions = sorted(
        FORBIDDEN_AUTHORITY_TOPICS.intersection(own_subscriptions)
    )
    if authority_publishers or authority_subscriptions:
        payload["outcome"] = "INVALID_EVIDENCE"
        payload["label"] = None
        payload["reasons"] = sorted(set(payload["reasons"]) | {"authority_topic_intersection"})
    if not sealed_graph_evidence["ready"] or not terminal_graph_evidence["ready"]:
        payload["outcome"] = "INVALID_EVIDENCE"
        payload["label"] = None
        invalid_graph_evidence = (
            sealed_graph_evidence
            if not sealed_graph_evidence["ready"]
            else terminal_graph_evidence
        )
        payload["reasons"] = sorted(
            set(payload["reasons"])
            | ({"required_component_node_missing"}
               if invalid_graph_evidence["missing_component_nodes"] else set())
            | ({"required_component_node_duplicate"}
               if invalid_graph_evidence["duplicate_component_nodes"] else set())
            | ({"component_authority_topic_intersection"}
               if invalid_graph_evidence["forbidden_component_endpoints"] else set())
            | ({"publisher_ownership_invalid"}
               if invalid_graph_evidence["publisher_ownership_failures"] else set())
        )
    payload.update(
        {
            "observer": "read_only_v2_uptake",
            "evidence_scope": "planner_v2_exact_publish_to_pp_availability_only",
            "pp_binding_cycle": pp_binding_cycle,
            "prelaunch_permit": prelaunch_permit,
            "run_nonce": expected_run_nonce,
            "started_monotonic_ns": started_monotonic_ns,
            "finished_monotonic_ns": time.monotonic_ns(),
            "graph": {
                "observer_publishers": own_publishers,
                "observer_subscriptions": own_subscriptions,
                "sealed": sealed_graph_evidence,
                "terminal": terminal_graph_evidence,
            },
            "authority": {
                "forbidden_topics": sorted(FORBIDDEN_AUTHORITY_TOPICS),
                "observer_authority_publishers": authority_publishers,
                "observer_authority_subscriptions": authority_subscriptions,
                "no_authority_topic_intersection": (
                    not authority_publishers and not authority_subscriptions
                ),
            },
        }
    )
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--ready-file")
    parser.add_argument("--sealed-file")
    parser.add_argument("--capture-complete-file")
    parser.add_argument("--terminal-graph-file")
    parser.add_argument("--first-fault-file")
    parser.add_argument("--pp-binding-cycle-file")
    parser.add_argument("--prelaunch-request-file")
    parser.add_argument("--prelaunch-permit-file")
    parser.add_argument("--expected-run-nonce")
    parser.add_argument("--ready-timeout", type=float, default=5.0)
    parser.add_argument("--graph-seal-timeout", type=float, default=5.0)
    parser.add_argument("--final-fence-enabled", action="store_true")
    parser.add_argument("--final-fence-producer-instance-id")
    parser.add_argument("--final-fence-session-id")
    parser.add_argument("--final-fence-sealed-epoch-id", type=int, default=0)
    parser.add_argument("--final-fence-fixed-file")
    parser.add_argument("--planner-stopped-file")
    parser.add_argument("--result-file")
    args = parser.parse_args()
    if not 1.0 <= args.duration <= 60.0:
        parser.error("--duration must be between 1 and 60 seconds")
    if not 0.1 <= args.ready_timeout <= 30.0:
        parser.error("--ready-timeout must be between 0.1 and 30 seconds")
    if not 0.1 <= args.graph_seal_timeout <= 30.0:
        parser.error("--graph-seal-timeout must be between 0.1 and 30 seconds")
    if args.final_fence_enabled and (
        not args.expected_run_nonce
        or not args.final_fence_producer_instance_id
        or not args.final_fence_session_id
        or args.final_fence_sealed_epoch_id <= 0
        or not args.final_fence_fixed_file
        or not args.planner_stopped_file
    ):
        parser.error("final-fence profile requires exact epoch identity")
    payload = observe(
        args.duration,
        ready_file=Path(args.ready_file) if args.ready_file else None,
        sealed_file=Path(args.sealed_file) if args.sealed_file else None,
        capture_complete_file=(
            Path(args.capture_complete_file) if args.capture_complete_file else None
        ),
        terminal_graph_file=(
            Path(args.terminal_graph_file) if args.terminal_graph_file else None
        ),
        first_fault_file=(
            Path(args.first_fault_file) if args.first_fault_file else None
        ),
        pp_binding_cycle_file=(
            Path(args.pp_binding_cycle_file) if args.pp_binding_cycle_file else None
        ),
        prelaunch_request_file=(
            Path(args.prelaunch_request_file) if args.prelaunch_request_file else None
        ),
        prelaunch_permit_file=(
            Path(args.prelaunch_permit_file) if args.prelaunch_permit_file else None
        ),
        expected_run_nonce=args.expected_run_nonce,
        ready_timeout_s=args.ready_timeout,
        graph_seal_timeout_s=args.graph_seal_timeout,
        final_fence_enabled=args.final_fence_enabled,
        final_fence_producer_instance_id=(
            args.final_fence_producer_instance_id or ""
        ),
        final_fence_session_id=args.final_fence_session_id or "",
        final_fence_sealed_epoch_id=args.final_fence_sealed_epoch_id,
        final_fence_fixed_file=(
            Path(args.final_fence_fixed_file)
            if args.final_fence_fixed_file
            else None
        ),
        planner_stopped_file=(
            Path(args.planner_stopped_file) if args.planner_stopped_file else None
        ),
    )
    if args.result_file:
        result_path = Path(args.result_file)
        if args.final_fence_enabled:
            _write_terminal_json_atomic(result_path, payload)
            payload = json.loads(result_path.read_text(encoding="utf-8"))
        else:
            _write_json_atomic(result_path, payload)
    print(json.dumps(payload, ensure_ascii=False, allow_nan=False))
    return 0 if payload["outcome"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
