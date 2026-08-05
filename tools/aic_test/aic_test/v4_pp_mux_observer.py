#!/usr/bin/env python3
"""Read-only bounded observer for the V4 -> PP -> Mux Stage-2 fixture."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any


PRIVATE_ROOT_PREFIX = "/aic_test/v4_pp_mux_stage2/"
CAUSAL_WINDOW_S = 2.5
COMMAND_EPSILON = 1.0e-5
MAX_STEERING_RAD = 0.64
MAX_FINAL_SPEED_MPS = 2.0
DIAGNOSTIC_COUNTER_MAX = (1 << 32) - 1


def _validate_private_root(root: str) -> bool:
    suffix = root.removeprefix(PRIVATE_ROOT_PREFIX)
    return bool(
        root.startswith(PRIVATE_ROOT_PREFIX)
        and suffix
        and all(character.isalnum() or character in "_-" for character in suffix)
        and "/" not in suffix
    )


def _stamp_key(stamp: object) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def _finite_command(command: object) -> bool:
    return all(
        math.isfinite(float(value))
        for value in (
            command.longitudinal.speed,
            command.longitudinal.acceleration,
            command.lateral.steering_tire_angle,
        )
    )


def _zero_command(command: object) -> bool:
    return bool(
        _finite_command(command)
        and abs(float(command.longitudinal.speed)) <= COMMAND_EPSILON
        and abs(float(command.lateral.steering_tire_angle)) <= COMMAND_EPSILON
    )


def _valid_v4_payload(data: list[float] | tuple[float, ...]) -> bool:
    if len(data) < 9 or not all(math.isfinite(float(value)) for value in data):
        return False
    count = int(data[2])
    if float(count) != float(data[2]) or count < 2:
        return False
    expected = 3 + 3 * count + 3
    if len(data) != expected or data[-3] != 4.0 or data[-2] < 1.0:
        return False
    distances = data[3 + 2 * count : 3 + 3 * count]
    return bool(
        abs(float(distances[0])) <= 1.0e-5
        and float(distances[-1]) > 0.0
        and all(float(lhs) <= float(rhs) for lhs, rhs in zip(distances, distances[1:]))
        and all(float(value) > 0.0 for value in data[3 + count : 3 + 2 * count])
    )


def _same_sign(lhs: float, rhs: float) -> bool:
    return abs(lhs) > COMMAND_EPSILON and abs(rhs) > COMMAND_EPSILON and lhs * rhs > 0.0


@dataclass(frozen=True)
class MuxStopPayload:
    output_stamp: tuple[int, int] | None
    reason: str
    final_stop_origin: str
    final_stop_origin_proven: bool
    stop_context: str
    evaluated_command_stamp: tuple[int, int] | None
    evaluated_envelope_stamp: tuple[int, int] | None
    selection_rejected_for_invalid_tracking: bool
    selection_rejection_kind: str


def _normalize_mux_stop_payload(payload: dict[str, Any]) -> MuxStopPayload | None:
    """Normalize optional debug fields without making missing data causal."""
    if (
        payload.get("selected_source") != "stop"
        or abs(float(payload.get("output_speed_mps", math.inf))) > COMMAND_EPSILON
    ):
        return None

    def stamp(prefix: str) -> tuple[int, int] | None:
        try:
            return int(payload[f"{prefix}_sec"]), int(payload[f"{prefix}_nanosec"])
        except (KeyError, TypeError, ValueError):
            return None

    return MuxStopPayload(
        output_stamp=stamp("output_stamp"),
        reason=str(payload.get("reason", "")),
        final_stop_origin=str(payload.get("final_stop_origin", "")),
        final_stop_origin_proven=bool(payload.get("final_stop_origin_proven", False)),
        stop_context=str(payload.get("stop_context", "")),
        evaluated_command_stamp=stamp("evaluated_pure_pursuit_command_stamp"),
        evaluated_envelope_stamp=stamp(
            "evaluated_pure_pursuit_envelope_command_stamp"
        ),
        selection_rejected_for_invalid_tracking=bool(
            payload.get(
                "evaluated_pure_pursuit_selection_rejected_for_invalid_tracking",
                False,
            )
        ),
        selection_rejection_kind=str(
            payload.get("evaluated_pure_pursuit_rejection_kind", "")
        ),
    )


def _saturating_increment(value: int) -> int:
    return min(DIAGNOSTIC_COUNTER_MAX, int(value) + 1)


def _saturating_increment_with_overflow(value: int) -> tuple[int, bool]:
    next_value = _saturating_increment(value)
    return next_value, bool(int(value) >= DIAGNOSTIC_COUNTER_MAX)


def _record_bounded_counter(evidence: Any, field: str) -> None:
    value, overflow = _saturating_increment_with_overflow(getattr(evidence, field))
    setattr(evidence, field, value)
    evidence.post_invalid_counter_overflow |= overflow


def _terminal_no_callback(invalid_v4_at: float | None, callback_count: int) -> bool:
    return bool(invalid_v4_at is not None and callback_count == 0)


def _latch_first_post_invalid_stop(evidence: Any, stop: MuxStopPayload, observed: float) -> None:
    if evidence.post_invalid_first_stop_receipt_at is not None:
        return
    evidence.post_invalid_first_stop_receipt_at = observed
    evidence.post_invalid_first_stop_output_stamp = stop.output_stamp
    evidence.post_invalid_first_stop_reason = stop.reason
    evidence.post_invalid_first_stop_context = stop.stop_context
    evidence.post_invalid_first_stop_origin = stop.final_stop_origin
    evidence.post_invalid_first_stop_origin_proven = stop.final_stop_origin_proven
    evidence.post_invalid_first_evaluated_command_stamp = stop.evaluated_command_stamp
    evidence.post_invalid_first_evaluated_envelope_stamp = stop.evaluated_envelope_stamp
    evidence.post_invalid_first_selector_flag = stop.selection_rejected_for_invalid_tracking
    evidence.post_invalid_first_rejection_kind = stop.selection_rejection_kind


def _latch_exact_mux_diagnostic_stop(evidence: Any, stop: MuxStopPayload, observed: float) -> None:
    if evidence.mux_diagnostic_stop_receipt_at is not None:
        return
    evidence.mux_diagnostic_stop_receipt_at = observed
    evidence.mux_diagnostic_stop_output_stamp = stop.output_stamp
    evidence.mux_diagnostic_stop_reason = stop.reason
    evidence.mux_diagnostic_stop_context = stop.stop_context
    evidence.mux_diagnostic_evaluated_command_stamp = stop.evaluated_command_stamp
    evidence.mux_diagnostic_evaluated_envelope_stamp = stop.evaluated_envelope_stamp
    evidence.mux_diagnostic_rejected_for_invalid_tracking = stop.selection_rejected_for_invalid_tracking
    evidence.mux_diagnostic_rejection_kind = stop.selection_rejection_kind
    evidence.mux_diagnostic_final_stop_origin = stop.final_stop_origin
    evidence.mux_diagnostic_final_stop_origin_proven = stop.final_stop_origin_proven


def _classify_post_invalid_stop_payload(
    payload: dict[str, Any], invalid_pp_stamp: tuple[int, int] | None
) -> str:
    """Pure diagnostic classifier; it grants no authority and changes no verdict."""
    stop = _normalize_mux_stop_payload(payload)
    if stop is None:
        return "stop_normalization_reject"
    if invalid_pp_stamp is None:
        return "invalid_pp_stamp_missing"
    if stop.evaluated_command_stamp != invalid_pp_stamp:
        return "command_stamp_mismatch"
    if stop.evaluated_envelope_stamp != invalid_pp_stamp:
        return "envelope_stamp_mismatch"
    if not stop.selection_rejected_for_invalid_tracking:
        return "selector_flag_mismatch"
    if stop.selection_rejection_kind != "invalid_tracking":
        return "rejection_kind_mismatch"
    return (
        "exact_candidate"
        if _is_exact_invalid_selector_context_candidate(stop, invalid_pp_stamp)
        else "selector_context_mismatch"
    )


def _is_direct_invalid_stop_candidate(
    stop: MuxStopPayload, invalid_pp_stamp: tuple[int, int]
) -> bool:
    """Pure O1->O2 transition guard; generic timeout candidates stay pending."""
    return bool(
        stop.evaluated_command_stamp == invalid_pp_stamp
        and stop.evaluated_envelope_stamp == invalid_pp_stamp
        and stop.selection_rejected_for_invalid_tracking
        and stop.selection_rejection_kind == "invalid_tracking"
        and stop.final_stop_origin == "invalid_tracking_selector_rejection"
        and stop.final_stop_origin_proven
    )


def _is_exact_invalid_selector_context_candidate(
    stop: MuxStopPayload, invalid_pp_stamp: tuple[int, int]
) -> bool:
    """Diagnostic only; never upgrades timeout context to direct causality."""
    return bool(
        stop.evaluated_command_stamp == invalid_pp_stamp
        and stop.evaluated_envelope_stamp == invalid_pp_stamp
        and stop.selection_rejected_for_invalid_tracking
        and stop.selection_rejection_kind == "invalid_tracking"
    )


def _select_direct_invalid_stop_candidate(
    candidates: tuple[MuxStopPayload, ...], invalid_pp_stamp: tuple[int, int]
) -> MuxStopPayload | None:
    """Return O2, not an earlier generic O1 timeout candidate."""
    for candidate in candidates:
        if _is_direct_invalid_stop_candidate(candidate, invalid_pp_stamp):
            return candidate
    return None


def _classify_invalid_fixture(
    *,
    base_fresh: bool,
    invalid_v4_count: int,
    pp_zero_seen: bool,
    envelope_seen: bool,
    reject_reason: str | None,
) -> str:
    if not base_fresh:
        return "FIXTURE_BASE_STREAM_ENDED"
    if (
        invalid_v4_count > 0
        and pp_zero_seen
        and envelope_seen
        and reject_reason == "state_lattice_v4_unavailable"
    ):
        return "PP_INVALID_CONFIRMED"
    return "NOT_EVALUATED"


def _is_first_zero_envelope_candidate(
    *,
    first_zero_stamp: tuple[int, int] | None,
    envelope_already_captured: bool,
    envelope_stamp: tuple[int, int],
    tracking_usable: bool,
    zero_command: bool,
) -> bool:
    return bool(
        first_zero_stamp is not None
        and not envelope_already_captured
        and envelope_stamp == first_zero_stamp
        and not tracking_usable
        and zero_command
    )


@dataclass
class Stage2Evidence:
    valid_v4_at: float | None = None
    invalid_v4_at: float | None = None
    pp_debug_at: float | None = None
    positive_envelope_at: float | None = None
    positive_envelope_steer: float | None = None
    positive_envelope_pp_stamp: tuple[int, int] | None = None
    positive_pp_command_at: float | None = None
    mux_pp_at: float | None = None
    positive_final_at: float | None = None
    positive_final_receipt_at: float | None = None
    mux_selected_input_stamp: tuple[int, int] | None = None
    mux_output_stamp: tuple[int, int] | None = None
    invalid_envelope_at: float | None = None
    invalid_pp_stamp: tuple[int, int] | None = None
    invalid_matched_pp_at: float | None = None
    zero_pp_command_at: float | None = None
    mux_stop_at: float | None = None
    mux_stop_receipt_at: float | None = None
    mux_stop_output_stamp: tuple[int, int] | None = None
    mux_final_exact_joined: bool = False
    mux_stop_reason: str | None = None
    mux_evaluated_pp_command_stamp: tuple[int, int] | None = None
    mux_evaluated_pp_envelope_stamp: tuple[int, int] | None = None
    mux_rejected_for_invalid_tracking: bool = False
    mux_rejection_kind: str | None = None
    mux_final_stop_origin: str | None = None
    mux_final_stop_origin_proven: bool = False
    mux_diagnostic_stop_receipt_at: float | None = None
    mux_diagnostic_stop_output_stamp: tuple[int, int] | None = None
    mux_diagnostic_stop_reason: str | None = None
    mux_diagnostic_stop_context: str | None = None
    mux_diagnostic_evaluated_command_stamp: tuple[int, int] | None = None
    mux_diagnostic_evaluated_envelope_stamp: tuple[int, int] | None = None
    mux_diagnostic_rejected_for_invalid_tracking: bool = False
    mux_diagnostic_rejection_kind: str | None = None
    mux_diagnostic_final_stop_origin: str | None = None
    mux_diagnostic_final_stop_origin_proven: bool = False
    first_watchdog_stop_at: float | None = None
    first_watchdog_stop_output_stamp: tuple[int, int] | None = None
    first_watchdog_stop_reason: str | None = None
    private_odom_last_at: float | None = None
    private_trajectory_last_at: float | None = None
    invalid_v4_first_at: float | None = None
    invalid_v4_last_at: float | None = None
    invalid_v4_count: int = 0
    first_pp_zero_stamp: tuple[int, int] | None = None
    first_pp_zero_envelope_reason: str | None = None
    pp_stale_debug_reason: str | None = None
    pp_stale_debug_at: float | None = None
    base_fresh_at_first_pp_zero: bool | None = None
    post_invalid_callback_count: int = 0
    post_invalid_json_parse_reject_count: int = 0
    post_invalid_stop_normalization_reject_count: int = 0
    pre_invalid_phase_drop_count: int = 0
    post_invalid_command_stamp_mismatch_count: int = 0
    post_invalid_envelope_stamp_mismatch_count: int = 0
    post_invalid_selector_flag_mismatch_count: int = 0
    post_invalid_rejection_kind_mismatch_count: int = 0
    post_invalid_exact_candidate_count: int = 0
    post_invalid_invalid_pp_stamp_missing_count: int = 0
    post_invalid_final_receipt_count: int = 0
    post_invalid_final_zero_count: int = 0
    post_invalid_first_classification: str | None = None
    post_invalid_first_stop_reason: str | None = None
    post_invalid_counter_overflow: bool = False
    mux_stop_candidate_evicted: bool = False
    post_invalid_no_callback: bool = False
    post_invalid_first_stop_receipt_at: float | None = None
    post_invalid_first_stop_output_stamp: tuple[int, int] | None = None
    post_invalid_first_stop_context: str | None = None
    post_invalid_first_stop_origin: str | None = None
    post_invalid_first_stop_origin_proven: bool = False
    post_invalid_first_evaluated_command_stamp: tuple[int, int] | None = None
    post_invalid_first_evaluated_envelope_stamp: tuple[int, int] | None = None
    post_invalid_first_selector_flag: bool = False
    post_invalid_first_rejection_kind: str | None = None
    odom_age_at_first_pp_zero_sec: float | None = None
    trajectory_age_at_first_pp_zero_sec: float | None = None
    zero_final_at: float | None = None
    zero_final_receipt_at: float | None = None
    blockers: list[str] = field(default_factory=list)

    def positive_complete(self) -> bool:
        times = (
            self.valid_v4_at,
            self.pp_debug_at,
            self.positive_pp_command_at,
            self.positive_envelope_at,
            self.mux_pp_at,
            self.positive_final_at,
        )
        if not all(value is not None for value in times):
            return False
        valid = float(self.valid_v4_at)
        debug = float(self.pp_debug_at)
        command = float(self.positive_pp_command_at)
        envelope = float(self.positive_envelope_at)
        mux = float(self.mux_pp_at)
        final = float(self.positive_final_at)
        return bool(
            valid <= debug <= min(command, envelope)
            and max(command, envelope) <= mux <= final
            and final - valid <= CAUSAL_WINDOW_S
            and self.mux_selected_input_stamp is not None
            and self.mux_output_stamp is not None
        )

    def invalid_complete(self) -> bool:
        times = (
            self.invalid_v4_at,
            self.invalid_envelope_at,
            self.zero_pp_command_at,
            self.mux_stop_at,
            self.zero_final_at,
        )
        if not self.positive_complete() or not all(value is not None for value in times):
            return False
        invalid = float(self.invalid_v4_at)
        envelope = float(self.invalid_envelope_at)
        command = float(self.zero_pp_command_at)
        mux = float(self.mux_stop_at)
        final = float(self.zero_final_at)
        return bool(
            invalid <= min(command, envelope)
            and max(command, envelope) <= mux
            # Final command and debug travel on independent DDS topics.  Their
            # receipt order is not a causal predicate; exact output stamp is.
            and invalid <= final
            and final - invalid <= CAUSAL_WINDOW_S
            and self.invalid_pp_stamp is not None
            and self.invalid_matched_pp_at == self.zero_pp_command_at
            and self.mux_stop_output_stamp is not None
            and self.mux_final_exact_joined
            and self.invalid_pp_stamp == self.mux_evaluated_pp_command_stamp
            and self.invalid_pp_stamp == self.mux_evaluated_pp_envelope_stamp
            and self.mux_rejected_for_invalid_tracking
            and self.mux_rejection_kind == "invalid_tracking"
            and self.mux_final_stop_origin == "invalid_tracking_selector_rejection"
            and self.mux_final_stop_origin_proven
        )

    def verdict(self) -> str:
        return "PASS" if self.invalid_complete() and not self.blockers and not self.post_invalid_counter_overflow else "HOLD"


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, delete=False
        ) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o644)
            json.dump(payload, stream, sort_keys=True, allow_nan=False)
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--private-root", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--ready", type=Path)
    args = parser.parse_args()
    if not (_validate_private_root(args.private_root) and 3.0 <= args.timeout <= 30.0):
        parser.error("bounded private Stage-2 observer configuration required")

    import rclpy
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from autoware_auto_planning_msgs.msg import Trajectory
    from multi_purpose_mpc_ros_msgs.msg import ControllerCommandEnvelope
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String

    rclpy.init(args=None)
    node = Node("aic_test_v4_pp_mux_observer")
    root = args.private_root.rstrip("/")
    evidence = Stage2Evidence()
    pp_commands: deque[tuple[float, tuple[int, int], float, float]] = deque(maxlen=256)
    final_commands: deque[tuple[float, tuple[int, int], float, float]] = deque(maxlen=256)
    positive_envelopes: deque[tuple[float, tuple[int, int], float]] = deque(maxlen=256)
    mux_stop_candidates: deque[
        tuple[
            float,
            tuple[int, int],
            str,
            tuple[int, int] | None,
            tuple[int, int] | None,
            bool,
            str,
            str,
            bool,
            str,
        ]
    ] = deque(maxlen=64)

    def now() -> float:
        return time.monotonic()

    def complete_invalid_chain() -> None:
        if (
            evidence.invalid_v4_at is None
            or evidence.invalid_envelope_at is None
            or evidence.zero_pp_command_at is None
            or evidence.invalid_pp_stamp is None
        ):
            return
        if not any(
            key == evidence.invalid_pp_stamp
            and abs(speed) <= COMMAND_EPSILON
            and abs(steer) <= COMMAND_EPSILON
            for _, key, speed, steer in pp_commands
        ):
            return
        normalized_candidates = tuple(
            MuxStopPayload(
                output_stamp=output_stamp,
                reason=reason,
                final_stop_origin=final_stop_origin,
                final_stop_origin_proven=final_stop_origin_proven,
                stop_context=stop_context,
                evaluated_command_stamp=evaluated_command_stamp,
                evaluated_envelope_stamp=evaluated_envelope_stamp,
                selection_rejected_for_invalid_tracking=(
                    rejected_for_invalid_tracking
                ),
                selection_rejection_kind=rejection_kind,
            )
            for (
                _,
                output_stamp,
                reason,
                evaluated_command_stamp,
                evaluated_envelope_stamp,
                rejected_for_invalid_tracking,
                rejection_kind,
                final_stop_origin,
                final_stop_origin_proven,
                stop_context,
            ) in mux_stop_candidates
        )
        if evidence.mux_diagnostic_stop_receipt_at is None:
            for candidate_index, candidate in enumerate(normalized_candidates):
                if not _is_exact_invalid_selector_context_candidate(
                    candidate, evidence.invalid_pp_stamp
                ):
                    continue
                raw = mux_stop_candidates[candidate_index]
                evidence.mux_diagnostic_stop_receipt_at = raw[0]
                evidence.mux_diagnostic_stop_output_stamp = candidate.output_stamp
                evidence.mux_diagnostic_stop_reason = candidate.reason
                evidence.mux_diagnostic_stop_context = candidate.stop_context
                evidence.mux_diagnostic_evaluated_command_stamp = (
                    candidate.evaluated_command_stamp
                )
                evidence.mux_diagnostic_evaluated_envelope_stamp = (
                    candidate.evaluated_envelope_stamp
                )
                evidence.mux_diagnostic_rejected_for_invalid_tracking = True
                evidence.mux_diagnostic_rejection_kind = (
                    candidate.selection_rejection_kind
                )
                evidence.mux_diagnostic_final_stop_origin = (
                    candidate.final_stop_origin
                )
                evidence.mux_diagnostic_final_stop_origin_proven = (
                    candidate.final_stop_origin_proven
                )
                break
        selected_candidate = _select_direct_invalid_stop_candidate(
            normalized_candidates, evidence.invalid_pp_stamp
        )
        if selected_candidate is None:
            return
        for (
            stop_received_at,
            output_stamp,
            reason,
            evaluated_command_stamp,
            evaluated_envelope_stamp,
            rejected_for_invalid_tracking,
            rejection_kind,
            final_stop_origin,
            final_stop_origin_proven,
            stop_context,
        ) in mux_stop_candidates:
            if stop_received_at < evidence.invalid_v4_at:
                continue
            candidate = MuxStopPayload(
                output_stamp=output_stamp,
                reason=reason,
                final_stop_origin=final_stop_origin,
                final_stop_origin_proven=final_stop_origin_proven,
                stop_context=stop_context,
                evaluated_command_stamp=evaluated_command_stamp,
                evaluated_envelope_stamp=evaluated_envelope_stamp,
                selection_rejected_for_invalid_tracking=(
                    rejected_for_invalid_tracking
                ),
                selection_rejection_kind=rejection_kind,
            )
            if candidate != selected_candidate:
                continue
            for final_received_at, key, speed, steer in final_commands:
                if (
                    key == output_stamp
                    and abs(speed) <= COMMAND_EPSILON
                    and abs(steer) <= COMMAND_EPSILON
                ):
                    evidence.mux_stop_receipt_at = stop_received_at
                    evidence.mux_stop_output_stamp = output_stamp
                    evidence.mux_final_exact_joined = True
                    evidence.mux_stop_reason = reason
                    evidence.mux_evaluated_pp_command_stamp = evaluated_command_stamp
                    evidence.mux_evaluated_pp_envelope_stamp = evaluated_envelope_stamp
                    evidence.mux_rejected_for_invalid_tracking = (
                        rejected_for_invalid_tracking
                    )
                    evidence.mux_rejection_kind = rejection_kind
                    evidence.mux_final_stop_origin = final_stop_origin
                    evidence.mux_final_stop_origin_proven = final_stop_origin_proven
                    evidence.zero_final_receipt_at = final_received_at
                    # Do not manufacture causality by moving a Mux decision
                    # after PP input receipts.  These are raw observer times;
                    # invalid_complete() checks their natural ordering.
                    evidence.mux_stop_at = stop_received_at
                    evidence.zero_final_at = final_received_at
                    return

    def on_v4(message: Float32MultiArray) -> None:
        observed = now()
        if _valid_v4_payload(list(message.data)):
            # Keep refreshing only until PP acknowledges the V4 geometry, then
            # freeze the source event which owns this causal chain.
            if evidence.pp_debug_at is None:
                evidence.valid_v4_at = observed
        elif evidence.positive_complete() and evidence.invalid_v4_at is None:
            evidence.invalid_v4_at = observed
            evidence.invalid_v4_first_at = observed
        if not _valid_v4_payload(list(message.data)):
            _record_bounded_counter(evidence, "invalid_v4_count")
            evidence.invalid_v4_last_at = observed

    def on_pp_debug(message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        observed = now()
        if (
            evidence.valid_v4_at is not None
            and payload.get("trajectory_source") == "trajectory_overtake_override"
            and payload.get("v4_poc_contract") is True
            and payload.get("v4_poc_geometry_applied") is True
        ):
            if evidence.pp_debug_at is None:
                evidence.pp_debug_at = observed
        stale_reason = str(payload.get("stale_reason", ""))
        if (
            evidence.invalid_v4_at is not None
            and observed >= evidence.invalid_v4_at
            and stale_reason
            and evidence.pp_stale_debug_reason is None
        ):
            evidence.pp_stale_debug_reason = stale_reason
            evidence.pp_stale_debug_at = observed

    def on_pp_command(message: AckermannControlCommand) -> None:
        observed = now()
        speed = float(message.longitudinal.speed)
        steer = float(message.lateral.steering_tire_angle)
        pp_commands.append((observed, _stamp_key(message.stamp), speed, steer))
        if (
            evidence.pp_debug_at is not None
            and observed >= evidence.pp_debug_at
            and _finite_command(message)
            and speed > 0.0
        ):
            if abs(steer) <= MAX_STEERING_RAD:
                if evidence.positive_pp_command_at is None:
                    evidence.positive_pp_command_at = observed
            else:
                if "pp_steering_out_of_bounds" not in evidence.blockers:
                    evidence.blockers.append("pp_steering_out_of_bounds")
        if evidence.invalid_v4_at is not None and observed >= evidence.invalid_v4_at:
            if _zero_command(message):
                if evidence.first_pp_zero_stamp is None:
                    evidence.first_pp_zero_stamp = _stamp_key(message.stamp)
                    if evidence.private_odom_last_at is not None:
                        evidence.odom_age_at_first_pp_zero_sec = (
                            observed - evidence.private_odom_last_at
                        )
                    if evidence.private_trajectory_last_at is not None:
                        evidence.trajectory_age_at_first_pp_zero_sec = (
                            observed - evidence.private_trajectory_last_at
                        )
                    ages = (
                        evidence.odom_age_at_first_pp_zero_sec,
                        evidence.trajectory_age_at_first_pp_zero_sec,
                    )
                    evidence.base_fresh_at_first_pp_zero = bool(
                        all(age is not None and 0.0 <= age <= 0.10 for age in ages)
                    )
                complete_invalid_chain()

    def on_envelope(message: ControllerCommandEnvelope) -> None:
        observed = now()
        command = message.command
        speed = float(command.longitudinal.speed)
        steer = float(command.lateral.steering_tire_angle)
        stamp = _stamp_key(command.stamp)
        matching_pp_times = [
            received_at
            for received_at, key, pp_speed, pp_steer in pp_commands
            if key == stamp
            and abs(pp_speed - speed) <= COMMAND_EPSILON
            and abs(pp_steer - steer) <= COMMAND_EPSILON
        ]
        matching_pp = bool(matching_pp_times)
        if (
            evidence.valid_v4_at is not None
            and matching_pp
            and observed >= max(matching_pp_times)
            and bool(message.pp_command_fresh)
            and bool(message.trajectory_tracking_usable)
            and _finite_command(command)
            and speed > 0.0
            and COMMAND_EPSILON < abs(steer) <= MAX_STEERING_RAD
        ):
            positive_envelopes.append((observed, stamp, steer))
        if (
            evidence.invalid_v4_at is not None
            and observed >= evidence.invalid_v4_at
            and matching_pp
            and _is_first_zero_envelope_candidate(
                first_zero_stamp=evidence.first_pp_zero_stamp,
                envelope_already_captured=evidence.invalid_envelope_at is not None,
                envelope_stamp=stamp,
                tracking_usable=bool(message.trajectory_tracking_usable),
                zero_command=_zero_command(command),
            )
        ):
            evidence.invalid_envelope_at = observed
            evidence.invalid_pp_stamp = stamp
            evidence.invalid_matched_pp_at = max(matching_pp_times)
            evidence.zero_pp_command_at = evidence.invalid_matched_pp_at
            evidence.first_pp_zero_envelope_reason = str(message.reason)
            complete_invalid_chain()

    def on_mux_debug(message: String) -> None:
        observed = now()
        if evidence.invalid_v4_at is not None:
            (
                evidence.post_invalid_callback_count,
                overflow,
            ) = _saturating_increment_with_overflow(
                evidence.post_invalid_callback_count
            )
            evidence.post_invalid_counter_overflow |= overflow
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            if evidence.invalid_v4_at is not None:
                (
                    evidence.post_invalid_json_parse_reject_count,
                    overflow,
                ) = _saturating_increment_with_overflow(
                    evidence.post_invalid_json_parse_reject_count
                )
                evidence.post_invalid_counter_overflow |= overflow
                if evidence.post_invalid_first_classification is None:
                    evidence.post_invalid_first_classification = "json_parse_reject"
            return
        if (
            evidence.mux_pp_at is None
            and evidence.pp_debug_at is not None
            and payload.get("selected_source") == "pure_pursuit"
            and float(payload.get("output_speed_mps", 0.0)) > 0.0
        ):
            selected_stamp = (
                int(payload.get("selected_input_stamp_sec", -1)),
                int(payload.get("selected_input_stamp_nanosec", -1)),
            )
            output_stamp = (
                int(payload.get("output_stamp_sec", -1)),
                int(payload.get("output_stamp_nanosec", -1)),
            )
            matching_commands = [
                item for item in pp_commands if item[1] == selected_stamp
            ]
            matching_envelopes = [
                item for item in positive_envelopes if item[1] == selected_stamp
            ]
            if matching_commands and matching_envelopes:
                command_received_at, _, _, _ = matching_commands[-1]
                envelope_received_at, _, envelope_steer = matching_envelopes[-1]
                if observed < max(command_received_at, envelope_received_at):
                    return
                evidence.positive_pp_command_at = command_received_at
                evidence.positive_envelope_at = envelope_received_at
                evidence.positive_envelope_steer = envelope_steer
                evidence.positive_envelope_pp_stamp = selected_stamp
                evidence.mux_pp_at = observed
                evidence.mux_selected_input_stamp = selected_stamp
                evidence.mux_output_stamp = output_stamp
                for received_at, key, final_speed, final_steer in final_commands:
                    if (
                        key == output_stamp
                        and math.isfinite(final_speed)
                        and math.isfinite(final_steer)
                        and final_speed > 0.0
                        and evidence.positive_envelope_steer is not None
                        and _same_sign(
                            final_steer, evidence.positive_envelope_steer
                        )
                    ):
                        # The final topic can be delivered before the debug
                        # topic from the same Mux cycle. Complete the join at
                        # the later receipt while retaining the raw receipt.
                        evidence.positive_final_receipt_at = received_at
                        evidence.positive_final_at = observed
                        break
        stop = _normalize_mux_stop_payload(payload)
        if evidence.invalid_v4_at is not None and observed >= evidence.invalid_v4_at:
            classification = _classify_post_invalid_stop_payload(
                payload, evidence.invalid_pp_stamp
            )
            field = f"post_invalid_{classification}_count"
            if hasattr(evidence, field):
                value, overflow = _saturating_increment_with_overflow(
                    getattr(evidence, field)
                )
                setattr(evidence, field, value)
                evidence.post_invalid_counter_overflow |= overflow
            if evidence.post_invalid_first_classification is None:
                evidence.post_invalid_first_classification = classification
            if stop is not None and evidence.post_invalid_first_stop_reason is None:
                evidence.post_invalid_first_stop_reason = stop.reason
            if stop is not None and evidence.post_invalid_first_stop_receipt_at is None:
                evidence.post_invalid_first_stop_receipt_at = observed
                evidence.post_invalid_first_stop_output_stamp = stop.output_stamp
                evidence.post_invalid_first_stop_context = stop.stop_context
                evidence.post_invalid_first_stop_origin = stop.final_stop_origin
                evidence.post_invalid_first_stop_origin_proven = stop.final_stop_origin_proven
                evidence.post_invalid_first_evaluated_command_stamp = stop.evaluated_command_stamp
                evidence.post_invalid_first_evaluated_envelope_stamp = stop.evaluated_envelope_stamp
                evidence.post_invalid_first_selector_flag = stop.selection_rejected_for_invalid_tracking
                evidence.post_invalid_first_rejection_kind = stop.selection_rejection_kind
            if (
                stop is not None
                and evidence.mux_diagnostic_stop_receipt_at is None
                and evidence.invalid_pp_stamp is not None
                and _is_exact_invalid_selector_context_candidate(
                    stop, evidence.invalid_pp_stamp
                )
            ):
                evidence.mux_diagnostic_stop_receipt_at = observed
                evidence.mux_diagnostic_stop_output_stamp = stop.output_stamp
                evidence.mux_diagnostic_stop_reason = stop.reason
                evidence.mux_diagnostic_stop_context = stop.stop_context
                evidence.mux_diagnostic_evaluated_command_stamp = stop.evaluated_command_stamp
                evidence.mux_diagnostic_evaluated_envelope_stamp = stop.evaluated_envelope_stamp
                evidence.mux_diagnostic_rejected_for_invalid_tracking = stop.selection_rejected_for_invalid_tracking
                evidence.mux_diagnostic_rejection_kind = stop.selection_rejection_kind
                evidence.mux_diagnostic_final_stop_origin = stop.final_stop_origin
                evidence.mux_diagnostic_final_stop_origin_proven = stop.final_stop_origin_proven
        elif stop is not None:
            _record_bounded_counter(evidence, "pre_invalid_phase_drop_count")
        if stop is None:
            return
        if (
            stop.reason == "pure_pursuit_cmd_timeout"
            and evidence.first_watchdog_stop_at is None
        ):
            evidence.first_watchdog_stop_at = observed
            evidence.first_watchdog_stop_output_stamp = stop.output_stamp
            evidence.first_watchdog_stop_reason = stop.reason
        if evidence.invalid_v4_at is not None and observed >= evidence.invalid_v4_at:
            if len(mux_stop_candidates) == mux_stop_candidates.maxlen:
                evidence.mux_stop_candidate_evicted = True
            mux_stop_candidates.append(
                (
                    observed,
                    stop.output_stamp or (-1, -1),
                    stop.reason,
                    stop.evaluated_command_stamp,
                    stop.evaluated_envelope_stamp,
                    stop.selection_rejected_for_invalid_tracking,
                    stop.selection_rejection_kind,
                    stop.final_stop_origin,
                    stop.final_stop_origin_proven,
                    stop.stop_context,
                )
            )
            complete_invalid_chain()

    def on_final_command(message: AckermannControlCommand) -> None:
        observed = now()
        speed = float(message.longitudinal.speed)
        steer = float(message.lateral.steering_tire_angle)
        final_commands.append((observed, _stamp_key(message.stamp), speed, steer))
        if evidence.invalid_v4_at is not None and observed >= evidence.invalid_v4_at:
            _record_bounded_counter(evidence, "post_invalid_final_receipt_count")
            if _zero_command(message):
                _record_bounded_counter(evidence, "post_invalid_final_zero_count")
        if (
            evidence.mux_pp_at is not None
            and observed >= evidence.mux_pp_at
            and evidence.mux_output_stamp == _stamp_key(message.stamp)
            and _finite_command(message)
            and 0.0 < speed <= MAX_FINAL_SPEED_MPS + COMMAND_EPSILON
            and abs(steer) <= MAX_STEERING_RAD
            and evidence.positive_envelope_steer is not None
            and _same_sign(steer, evidence.positive_envelope_steer)
        ):
            if evidence.positive_final_at is None:
                evidence.positive_final_at = observed
                evidence.positive_final_receipt_at = observed
        if evidence.invalid_v4_at is not None and observed >= evidence.invalid_v4_at:
            if _zero_command(message):
                complete_invalid_chain()

    def on_private_odom(message: object) -> None:
        del message
        evidence.private_odom_last_at = now()

    def on_private_trajectory(message: object) -> None:
        del message
        evidence.private_trajectory_last_at = now()

    # This observer deliberately owns no publishers; the driver is the sole
    # test authority publisher in this graph.
    subscriptions = (
        node.create_subscription(
            Float32MultiArray, f"{root}/input/reference_override", on_v4, 10
        ),
        node.create_subscription(String, f"{root}/pp/debug", on_pp_debug, 10),
        node.create_subscription(
            AckermannControlCommand, f"{root}/pp/control_cmd", on_pp_command, 10
        ),
        node.create_subscription(
            ControllerCommandEnvelope,
            f"{root}/pp/command_envelope",
            on_envelope,
            10,
        ),
        node.create_subscription(String, f"{root}/mux/debug", on_mux_debug, 10),
        node.create_subscription(Odometry, f"{root}/input/kinematics", on_private_odom, 10),
        node.create_subscription(Trajectory, f"{root}/input/trajectory", on_private_trajectory, 10),
        node.create_subscription(
            AckermannControlCommand,
            f"{root}/output/control_cmd",
            on_final_command,
            10,
        ),
    )
    del subscriptions

    if args.ready is not None:
        _write_json_atomic(
            args.ready,
            {"schema_version": 1, "private_root": root, "ready": True},
        )

    deadline = time.monotonic() + args.timeout
    try:
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and evidence.verdict() != "PASS"
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        base_fresh = bool(evidence.base_fresh_at_first_pp_zero)
        evidence.post_invalid_no_callback = bool(
            evidence.invalid_v4_at is not None
            and evidence.post_invalid_callback_count == 0
        )
        payload = {
            "schema_version": 1,
            "verdict": evidence.verdict(),
            "private_root": root,
            "causal_window_sec": CAUSAL_WINDOW_S,
            "observer_authority_publisher_count": 0,
            "evidence": {
                key: value
                for key, value in vars(evidence).items()
                if key != "blockers"
            },
            "blockers": evidence.blockers,
            "invalid_fixture_classification": _classify_invalid_fixture(
                base_fresh=base_fresh,
                invalid_v4_count=evidence.invalid_v4_count,
                pp_zero_seen=evidence.first_pp_zero_stamp is not None,
                envelope_seen=(
                    evidence.invalid_envelope_at is not None
                ),
                reject_reason=evidence.first_pp_zero_envelope_reason,
            ),
        }
        _write_json_atomic(args.result, payload)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if payload["verdict"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
