import copy
import hashlib
import json
import math
from pathlib import Path
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from autoware_auto_control_msgs.msg import AckermannControlCommand
from multi_purpose_mpc_ros_msgs.msg import (
    CandidateExecutionPoint,
    ControllerCommandEnvelope,
    ControllerTrackingStatus,
    FreeRunExecutionAck,
    FreeRunSourceKey,
    OvertakePlan,
    RecoveryControlCommand,
    RecoveryPermit,
    RecoveryStatus,
    SafetyConstraint,
    SafetyStopStatus,
    StateLatticeControlCommand,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String

from hybrid_control_mux.core import (
    SafetyConstraintDecision,
    SafetyConstraintState,
    SteeringLimiterConfig,
    SteeringLimitResult,
)
from hybrid_control_mux.hybrid_control_mux_node import (
    FreeRunPublishedCommandRecord,
    FreeRunSourceGapLease,
    HybridControlMuxNode,
    SafetyAuthorityRendezvousState,
)


def _set_stamp(stamp, sec: int) -> None:
    stamp.sec = sec
    stamp.nanosec = 0


def test_mux_runtime_measurement_is_default_off_without_cycle_side_effects():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        assert mux.mux_runtime_measurement_enabled is False
        mux.on_timer()
        assert mux.mux_runtime_measurement_sequence == 0
        assert not mux.mux_runtime_measurement_records
        assert not mux.mux_runtime_measurement_cycle_records
        assert mux.mux_runtime_measurement_pending_cycle is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


class _PublishRecorder:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def publish(self, message):
        payload = _canonical_ros_fields(message)
        reason = None
        if isinstance(message, String):
            reason = json.loads(message.data).get("reason")
        self.events.append((self.name, payload, reason))


def _canonical_ros_fields(value):
    """Full ROS field tuple; avoids CDR padding while retaining every field."""
    if hasattr(value, "get_fields_and_field_types"):
        return tuple(
            (name, _canonical_ros_fields(getattr(value, name)))
            for name in value.get_fields_and_field_types()
        )
    if isinstance(value, (list, tuple)):
        return tuple(_canonical_ros_fields(item) for item in value)
    # ROS generated fields may contain numpy arrays.  Preserve every element
    # but convert their non-bool-comparable container deterministically.
    if not isinstance(value, (str, bytes, bytearray)) and hasattr(value, "tolist"):
        return _canonical_ros_fields(value.tolist())
    return value


def _envelope_cache_fingerprint(cache):
    return tuple(
        (key, value[0], value[2], value[3], _canonical_ros_fields(value[4]))
        for key, value in sorted(cache.items())
    )


def _control_visible_equivalence_snapshot(mux, events):
    """Only capture slots/counters and seal metadata are diagnostic-only."""
    return {
        "published_event_order": tuple(event for event in events if event[0] != "debug"),
        "last_source": mux.last_source,
        "final_reason": next((reason for name, _, reason in reversed(events) if name == "debug"), None),
        "envelope_cache": _envelope_cache_fingerprint(mux.pure_pursuit_envelope_cache),
        "envelope_watermark": (
            mux.pure_pursuit_envelope_watermark[0],
            _canonical_ros_fields(mux.pure_pursuit_envelope_watermark[1][4]),
        ) if mux.pure_pursuit_envelope_watermark else None,
        "active_producer": mux.pure_pursuit_envelope_active_producer_instance_id,
        "active_sequence": mux.pure_pursuit_envelope_active_sequence,
        "active_stamp_ns": mux.pure_pursuit_envelope_active_stamp_ns,
        "fault": (mux.pure_pursuit_envelope_fault_latched, mux.pure_pursuit_envelope_fault_reason),
        "authority": (mux.motion_authority_grant_active, mux.motion_authority_grant_sequence),
        "race_finish": tuple(vars(mux.finish_stop_latch).items()),
        "external_stop": mux.external_stop_latched,
        "watchdogs": (tuple(vars(mux.control_loop_watchdog).items()), tuple(vars(mux.ros_clock_watchdog).items()), mux.ros_clock_motion_ready, mux.ros_clock_observation_reason),
        "limiter": tuple(vars(mux.steering_limiter).items()),
        "fallback": (mux.fallback_speed_mps, mux.fallback_accel_max_mps2, mux.fallback_decel_min_mps2),
    }


def _run_latest_sample_equivalence_trace(trace: str, enabled: bool):
    mux = HybridControlMuxNode(parameter_overrides=[
        Parameter("latest_sample_observability_enabled", value=enabled),
        # Match the private Stage-2 launch selection path; the node's public
        # default is MPC and would make every trace an irrelevant MPC timeout.
        Parameter("primary_source", value="pure_pursuit"),
        Parameter("require_safety_constraint", value=False),
        Parameter("race_arm_required", value=False),
        Parameter("pure_pursuit_cmd_timeout_sec", value=10.0),
        Parameter("pure_pursuit_tracking_status_timeout_sec", value=10.0),
        Parameter("control_loop_max_gap_sec", value=10.0),
        Parameter("ros_clock_stall_timeout_sec", value=10.0),
        Parameter("mux_runtime_measurement_callback_capacity", value=128),
    ])
    mux.timer.cancel()
    # Freeze the only control-time source used by callbacks/timer.  Publishers
    # are replaced after node setup, avoiding DDS scheduling/order variation.
    mux.now_sec = lambda: 10.0
    stamp = 120
    _set_mux_ros_time_for_envelope(mux, stamp)
    # Node construction observes host time; reset test-visible watchdog state
    # after the ROS-clock override so OFF/ON share identical logical history.
    mux.control_loop_watchdog._last_tick_sec = None
    mux.ros_clock_watchdog.reset()
    # The production grant issuer is intentionally per-process random.  Pin it
    # only in this same-input harness so full serialized grant output is a
    # deterministic control comparison rather than an identity comparison.
    mux.motion_authority_grant_issuer_instance_id = 1
    events = []
    recorders = [_PublishRecorder(name, events) for name in ("control", "tracking", "grant", "debug")]
    mux.control_pub, mux.tracking_status_pub, mux.motion_authority_grant_pub, mux.debug_pub = recorders
    try:
        command = _matching_legacy_command(stamp)
        status = _matching_legacy_tracking_status(stamp)
        mux.on_pure_pursuit_tracking_status(status)
        mux.on_pure_pursuit_cmd(command)
        envelope = _command_envelope(stamp_sec=stamp, command_sequence=1)
        if trace == "invalid_zero":
            envelope.trajectory_tracking_usable = False
            envelope.command.longitudinal.speed = 0.0
        elif trace == "rapid_supersession":
            _bind_envelope_from_legacy(mux, command, status, sequence=1)
            envelope = None
        elif trace == "duplicate_conflict":
            _bind_envelope_from_legacy(mux, command, status, sequence=1)
            envelope = _command_envelope(stamp_sec=stamp, command_sequence=2)
            envelope.command.longitudinal.speed = 2.0
        elif trace == "stale":
            envelope.command_age_sec = 99.0
        elif trace == "future":
            envelope.header.stamp.sec = 2_000_000_000
            envelope.command.stamp.sec = 2_000_000_000
            envelope.command.longitudinal.stamp.sec = 2_000_000_000
            envelope.command.lateral.stamp.sec = 2_000_000_000
        elif trace == "sequence_regression":
            _bind_envelope_from_legacy(mux, command, status, sequence=10)
            envelope = _command_envelope(stamp_sec=stamp, command_sequence=1)
        elif trace == "stamp_regression":
            _bind_envelope_from_legacy(mux, command, status, sequence=1)
            envelope = _command_envelope(stamp_sec=stamp - 1, command_sequence=3)
        elif trace == "missing_cache":
            envelope = None
        elif trace == "zero_reverse":
            command.longitudinal.speed = -1.0
            mux.on_pure_pursuit_cmd(command)
        elif trace == "steering_limit":
            command.lateral.steering_tire_angle = 9.0
            mux.on_pure_pursuit_cmd(command)
        elif trace == "watchdog":
            mux.control_loop_watchdog._last_tick_sec = -100.0
        elif trace == "clock_stall":
            mux.ros_clock_watchdog._maximum_ros_time_ns = 0
            mux.ros_clock_watchdog._last_progress_steady_sec = -100.0
        elif trace == "estop":
            mux.on_external_safety_status(SafetyStopStatus(valid=False))
        elif trace == "finish":
            mux.on_awsim_state(String(data="Ready"))
            mux.on_race_armed(Bool(data=True))
            mux.on_awsim_state(String(data="Start"))
            mux.on_awsim_state(String(data="Finish"))
        elif trace == "capacity_overflow":
            for sequence in range(1, 130):
                mux.on_pure_pursuit_command_envelope(
                    _command_envelope(stamp_sec=stamp, command_sequence=sequence)
                )
            envelope = None
        if envelope is not None:
            mux.on_pure_pursuit_command_envelope(envelope)
        mux.on_timer()
        # Every trace executes one deterministic timer cycle and its control
        # publisher; branch-specific input state must survive to the snapshot.
        assert any(event[0] == "control" for event in events)
        snapshot = _control_visible_equivalence_snapshot(mux, events)
        assert snapshot["final_reason"] != "mpc_cmd_timeout"
        if trace == "positive":
            assert mux.last_source == "pure_pursuit"
        elif trace == "rapid_supersession":
            assert mux.pure_pursuit_envelope_active_sequence == 2
        elif trace == "duplicate_conflict":
            assert mux.pure_pursuit_envelope_fault_reason == "duplicate_conflict"
        elif trace == "stale":
            assert mux.pure_pursuit_envelope_last_message.command_age_sec == 99.0
        elif trace == "future":
            assert mux.pure_pursuit_envelope_last_reason.startswith("future_stamp")
        elif trace == "sequence_regression":
            assert mux.pure_pursuit_envelope_fault_reason == "sequence_regression"
        elif trace == "stamp_regression":
            assert mux.pure_pursuit_envelope_fault_reason == "stamp_regression"
        elif trace == "missing_cache":
            assert mux.pure_pursuit_envelope_last_reason == "missing"
        elif trace == "estop":
            assert mux.external_stop_latched
        elif trace == "finish":
            assert mux.finish_stop_latch.latched
        elif trace == "capacity_overflow" and enabled:
            assert mux.mux_runtime_measurement_callback_overflow
        return snapshot
    finally:
        mux.destroy_node()


@pytest.mark.parametrize(
    "trace",
    [
        "positive", "invalid_zero", "rapid_supersession", "duplicate_conflict",
        "stale", "future", "sequence_regression", "stamp_regression", "missing_cache", "zero_reverse", "steering_limit",
        "watchdog", "clock_stall", "estop", "finish", "capacity_overflow",
    ],
)
def test_latest_sample_observability_on_off_is_control_functionally_equivalent(trace):
    rclpy.init()
    try:
        off = _run_latest_sample_equivalence_trace(trace, enabled=False)
        on = _run_latest_sample_equivalence_trace(trace, enabled=True)
        assert on == off
    finally:
        rclpy.shutdown()


def test_private_stage2_launch_is_the_only_explicit_latest_sample_enablement():
    launch = Path(__file__).resolve().parents[2] / (
        "aichallenge_submit_launch/launch/test_only/v4_pp_mux_stage2.launch.xml"
    )
    launch_text = launch.read_text(encoding="utf-8")
    assert 'name="latest_sample_observability_enabled" value="true"' in launch_text
    public_launch_root = launch.parents[1]
    assert all(
        "latest_sample_observability_enabled" not in candidate.read_text(encoding="utf-8")
        for candidate in public_launch_root.rglob("*.xml")
        if "test_only" not in candidate.parts
    )
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        assert mux.latest_sample_observability_enabled is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("legacy_enabled", "latest_enabled"),
    ((True, False), (False, True), (True, True)),
)
def test_mux_measurement_flags_are_independent(
    legacy_enabled: bool, latest_enabled: bool
):
    rclpy.init()
    mux = HybridControlMuxNode(
        parameter_overrides=[
            Parameter("mux_runtime_measurement_enabled", value=legacy_enabled),
            Parameter("latest_sample_observability_enabled", value=latest_enabled),
        ]
    )
    mux.timer.cancel()
    try:
        def run_cycle() -> None:
            if latest_enabled:
                mux._mux_latest_sample_cycle_context = (
                    None, None, "nonqualifying", "stop", "test", False, False
                )
            if legacy_enabled:
                mux.mux_runtime_measurement_pending_cycle = {"source": "stop"}

        mux._on_timer_impl = run_cycle
        mux.on_timer()

        assert (mux.mux_runtime_measurement_legacy_cycle_count == 1) is legacy_enabled
        assert (mux.mux_runtime_measurement_cycle_count == 1) is latest_enabled
        assert (mux.mux_runtime_measurement_cycle_records is not None) is legacy_enabled
        assert (mux.mux_latest_sample_cycle_slots is not None) is latest_enabled
        if latest_enabled and not legacy_enabled:
            assert mux.mux_runtime_measurement_pending_cycle is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("legacy_enabled", "latest_enabled"),
    ((True, False), (False, True), (True, True)),
)
def test_mux_measurement_never_suppresses_timer_exception(
    legacy_enabled: bool, latest_enabled: bool
):
    rclpy.init()
    mux = HybridControlMuxNode(
        parameter_overrides=[
            Parameter("mux_runtime_measurement_enabled", value=legacy_enabled),
            Parameter("latest_sample_observability_enabled", value=latest_enabled),
        ]
    )
    mux.timer.cancel()
    try:
        def raise_from_control() -> None:
            raise RuntimeError("control_failure")

        mux._on_timer_impl = raise_from_control
        with pytest.raises(RuntimeError, match="control_failure"):
            mux.on_timer()
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_mux_runtime_measurement_callback_finalizes_early_returns_once():
    rclpy.init()
    mux = HybridControlMuxNode(
        parameter_overrides=[
            Parameter("mux_runtime_measurement_enabled", value=True),
            Parameter("latest_sample_observability_enabled", value=True),
        ]
    )
    mux.timer.cancel()
    try:
        # The default envelope is invalid and returns through the early
        # sequence-zero branch.  Measurement must still finalize once.
        mux.on_pure_pursuit_command_envelope(ControllerCommandEnvelope())
        assert mux.mux_runtime_measurement_callback_sequence == 1
        assert mux.mux_runtime_measurement_callback_count == 1
        assert mux.mux_runtime_measurement_callback_in_flight == 0
        record = mux.mux_latest_sample_callback_slots[0]
        assert record[2] == (0, 0, 0, 0)
        assert record[5] == "inserted"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_latest_sample_trace_classifier_is_mutually_exclusive_and_fail_closed():
    target = (7, 10, 100, 3)
    successor = (7, 11, 110, 3)
    target_record = (1, 1, target, True, "ok", "inserted", None, target, "advanced", False)
    successor_record = (2, 2, successor, True, "ok", "inserted", target, successor, "advanced", False)
    omitted_target_successor_record = (
        1, 2, successor, True, "ok", "inserted", None, successor, "advanced", False
    )
    successor_evaluated = (1, 1, successor, successor, "selected", "stop", "timeout", False, False)
    classify = HybridControlMuxNode.classify_latest_sample_trace
    assert classify(target_identity=target, target_published_in_epoch=True, sealed=True, trace_loss=False, callback_records=(target_record,), cycle_records=((1, 1, target, target, "selected", "stop", "x", False, False),)) == "RECEIVED_AND_EVALUATED"
    assert classify(target_identity=target, target_published_in_epoch=True, sealed=True, trace_loss=False, callback_records=(target_record, successor_record), cycle_records=(successor_evaluated,)) == "RECEIVED_AND_SUPERSEDED"
    assert classify(target_identity=target, target_published_in_epoch=True, sealed=True, trace_loss=False, callback_records=(omitted_target_successor_record,), cycle_records=(successor_evaluated,)) == "NOT_OBSERVED_IN_COMPLETE_MUX_TRACE"
    assert classify(target_identity=target, target_published_in_epoch=True, sealed=True, trace_loss=False, callback_records=(), cycle_records=()) == "NONQUALIFYING"
    assert classify(target_identity=target, target_published_in_epoch=True, sealed=True, trace_loss=False, callback_records=((2, *target_record[1:]),), cycle_records=()) == "INDETERMINATE_TRACE_LOSS"
    assert classify(target_identity=target, target_published_in_epoch=True, sealed=True, trace_loss=False, callback_records=(target_record,), cycle_records=()) == "NONQUALIFYING"
    assert classify(target_identity=target, target_published_in_epoch=False, sealed=True, trace_loss=False, callback_records=(), cycle_records=()) == "INDETERMINATE_TRACE_LOSS"


def test_mux_runtime_measurement_callback_overflow_is_sticky_hold():
    rclpy.init()
    mux = HybridControlMuxNode(
        parameter_overrides=[
            Parameter("mux_runtime_measurement_enabled", value=True),
            Parameter("latest_sample_observability_enabled", value=True),
            Parameter("mux_runtime_measurement_capacity", value=128),
        ]
    )
    mux.timer.cancel()
    try:
        message = ControllerCommandEnvelope()
        for _ in range(129):
            mux.on_pure_pursuit_command_envelope(message)
        assert mux.mux_runtime_measurement_callback_count == 128
        assert mux.mux_runtime_measurement_drops == 1
        assert mux.mux_runtime_measurement_overflow is True
        assert mux.mux_runtime_measurement_callback_in_flight == 0
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_latest_sample_callback_exception_is_recorded_and_propagated():
    rclpy.init()
    mux = HybridControlMuxNode(
        parameter_overrides=[
            Parameter("latest_sample_observability_enabled", value=True),
        ]
    )
    mux.timer.cancel()
    try:
        def raise_from_callback(_message) -> None:
            raise RuntimeError("callback_failure")

        mux._on_pure_pursuit_command_envelope_impl = raise_from_callback
        with pytest.raises(RuntimeError, match="callback_failure"):
            mux.on_pure_pursuit_command_envelope(ControllerCommandEnvelope())
        assert mux.mux_runtime_measurement_callback_sequence == 1
        assert mux.mux_runtime_measurement_callback_completions == 1
        assert mux.mux_runtime_measurement_callback_in_flight == 0
        assert mux.mux_latest_sample_callback_slots[0][-1] is True
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_latest_sample_cycle_overflow_is_separate_and_sticky():
    rclpy.init()
    mux = HybridControlMuxNode(
        parameter_overrides=[
            Parameter("latest_sample_observability_enabled", value=True),
            Parameter("mux_runtime_measurement_cycle_capacity", value=128),
        ]
    )
    mux.timer.cancel()
    try:
        mux.mux_runtime_measurement_cycle_count = 128
        mux._on_timer_impl = lambda: None
        mux.on_timer()
        assert mux.mux_runtime_measurement_cycle_overflow is True
        assert mux.mux_runtime_measurement_cycle_drops == 1
        assert mux.mux_runtime_measurement_callback_drops == 0
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_latest_sample_late_callback_is_not_added_after_quiescent_close():
    rclpy.init()
    mux = HybridControlMuxNode(
        parameter_overrides=[
            Parameter("latest_sample_observability_enabled", value=True),
        ]
    )
    mux.timer.cancel()
    try:
        mux.mark_mux_runtime_measurement_executor_quiesced()
        mux.on_pure_pursuit_command_envelope(ControllerCommandEnvelope())
        assert mux.mux_runtime_measurement_capture_late_entry is True
        assert mux.mux_runtime_measurement_callback_sequence == 0
        assert mux.mux_runtime_measurement_callback_count == 0
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_latest_sample_shutdown_round_trip_seals_target_agnostic_capture(tmp_path):
    output_path = tmp_path / "latest-sample.json"
    successor = (7, 11, 110, 3)
    rclpy.init()
    mux = HybridControlMuxNode(
        parameter_overrides=[
            Parameter("latest_sample_observability_enabled", value=True),
            Parameter("mux_runtime_measurement_output_path", value=str(output_path)),
            Parameter(
                "mux_runtime_measurement_selector_manifest_sha256",
                value="a" * 64,
            ),
        ]
    )
    mux.timer.cancel()
    mux.mux_latest_sample_callback_slots[0] = (
        1, 2, successor, True, "ok", "inserted", None, successor, "advanced", False
    )
    mux.mux_latest_sample_cycle_slots[0] = (
        1, 3, successor, successor, "selected", "stop", "timeout", False, False
    )
    mux.mux_runtime_measurement_callback_count = 1
    mux.mux_runtime_measurement_callback_sequence = 1
    mux.mux_runtime_measurement_callback_completions = 1
    mux.mux_runtime_measurement_cycle_count = 1
    mux.mux_latest_sample_cycle_sequence = 1
    mux.mux_runtime_measurement_cycle_completions = 1
    mux.mark_mux_runtime_measurement_executor_quiesced()
    mux.destroy_node()
    rclpy.shutdown()

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["sealed"] is True
    assert payload["latest_sample_terminal"] == "UNCLASSIFIED_TARGET_AGNOSTIC_CAPTURE"
    capture = payload["latest_sample_capture"]
    assert capture["selector_manifest_sha256"] == "a" * 64
    assert payload["latest_sample_callback_records"][0][2] == list(successor)


def test_mux_runtime_cycle_record_v2_binds_same_cycle_grant_and_final_command():
    command = AckermannControlCommand()
    _set_stamp(command.stamp, 12)
    _set_stamp(command.longitudinal.stamp, 12)
    _set_stamp(command.lateral.stamp, 12)
    command.longitudinal.speed = 2.0
    command.longitudinal.acceleration = 0.2
    command.lateral.steering_tire_angle = 0.1
    sample = HybridControlMuxNode._build_selected_pp_motion_sample(
        command,
        command_receipt_time_sec=1.0,
        envelope_identity=(31, 47, 12_000_000_000, 9),
    )
    plan_identity = (
        4,
        101,
        7,
        "grid_d2",
        -1,
        19,
        12_000_000_000,
        9,
        OvertakePlan.PASSING,
        3,
        bytes([0xA5] * 32),
        True,
    )
    limiter = SteeringLimitResult(
        raw_steering_rad=0.1,
        limited_steering_rad=0.08,
        steering_delta_rad=-0.02,
        angle_limited=False,
        rate_limited=True,
        limiter_reset=False,
    )

    record = HybridControlMuxNode._mux_runtime_cycle_record_v2_fields(
        selected_input_cmd=command,
        selected_pp_motion_sample=sample,
        selected_plan_identity=plan_identity,
        grant_published=True,
        grant_valid=True,
        grant_sequence=55,
        grant_commit_steady_ns=987_654,
        grant_issuer_instance_id=77,
        final_command=command,
        steering_result=limiter,
        steering_limiter_config=SteeringLimiterConfig(
            max_steering_angle_rad=0.64,
            max_steering_rate_radps=3.0,
        ),
    )

    assert record["selected_pp_identity"] == {
        "present": True,
        "producer_instance_id": 31,
        "command_sequence": 47,
        "command_stamp_ns": 12_000_000_000,
        "plan_generation": 9,
    }
    assert record["selected_plan_sample_identity"]["present"] is True
    assert record["selected_plan_sample_identity"]["planner_instance_id"] == 101
    assert record["selected_plan_sample_identity"]["candidate_content_sha256"] == (
        bytes([0xA5] * 32).hex()
    )
    assert record["selected_input_command_canonical_sha256"] == (
        HybridControlMuxNode._canonical_controller_command_digest(command).hex()
    )
    assert record["final_command_canonical_sha256"] == (
        HybridControlMuxNode._canonical_controller_command_digest(command).hex()
    )
    assert record["final_command_cdr_sha256"] == hashlib.sha256(
        serialize_message(command)
    ).hexdigest()
    assert record["limiter"]["limited_steering_rad"] == pytest.approx(0.08)
    assert record["limiter"]["rate_limited"] is True
    assert record["grant"] == {
        "present": True,
        "issuer_instance_id": 77,
        "sequence": 55,
        "valid": True,
        "commit_steady_ns": 987_654,
        "final_command_cdr_sha256": record["final_command_cdr_sha256"],
    }


def test_mux_runtime_cycle_record_v2_revoke_stop_clears_exact_identity():
    stop = AckermannControlCommand()
    _set_stamp(stop.stamp, 13)
    _set_stamp(stop.longitudinal.stamp, 13)
    _set_stamp(stop.lateral.stamp, 13)
    stop.longitudinal.acceleration = -1.5
    record = HybridControlMuxNode._mux_runtime_cycle_record_v2_fields(
        selected_input_cmd=None,
        selected_pp_motion_sample=None,
        selected_plan_identity=None,
        grant_published=True,
        grant_valid=False,
        grant_sequence=56,
        grant_commit_steady_ns=None,
        grant_issuer_instance_id=77,
        final_command=stop,
        steering_result=SteeringLimitResult(
            raw_steering_rad=0.0,
            limited_steering_rad=0.0,
            steering_delta_rad=0.0,
            angle_limited=False,
            rate_limited=False,
            limiter_reset=True,
        ),
        steering_limiter_config=SteeringLimiterConfig(),
    )

    assert record["selected_pp_identity"]["present"] is False
    assert all(
        value is None
        for key, value in record["selected_pp_identity"].items()
        if key != "present"
    )
    assert record["selected_plan_sample_identity"]["present"] is False
    assert all(
        value is None
        for key, value in record["selected_plan_sample_identity"].items()
        if key != "present"
    )
    assert record["grant"] == {
        "present": True,
        "issuer_instance_id": 77,
        "sequence": 56,
        "valid": False,
        "commit_steady_ns": None,
        "final_command_cdr_sha256": None,
    }


def _state_lattice_command(
    *, stamp_sec: int, sequence: int, active: bool = True
) -> StateLatticeControlCommand:
    message = StateLatticeControlCommand()
    message.header.frame_id = "map"
    _set_stamp(message.header.stamp, stamp_sec)
    message.schema_version = StateLatticeControlCommand.SCHEMA_V1
    message.producer_instance_id = 101
    message.command_sequence = sequence
    message.plan_generation = 7
    message.active = active
    message.safety_evaluation_enabled = True
    message.inputs_fresh = True
    message.costmap_valid = True
    message.trajectory_valid = True
    message.trajectory_safe = True
    message.stop_required = False
    _set_stamp(message.command.stamp, stamp_sec)
    _set_stamp(message.command.longitudinal.stamp, stamp_sec)
    _set_stamp(message.command.lateral.stamp, stamp_sec)
    message.command.longitudinal.speed = 2.0
    message.command.longitudinal.acceleration = 0.2
    message.command.lateral.steering_tire_angle = 0.05
    message.command.lateral.steering_tire_rotation_rate = 0.1
    message.reason = "fixture"
    return message


def test_state_lattice_atomic_source_rejects_mutation_and_exposes_fresh_authority():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "state_lattice_instant_control_enabled:=true",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        message = _state_lattice_command(stamp_sec=10, sequence=1)
        mux.on_state_lattice_control_cmd(message)
        state = mux._current_state_lattice_state(mux.now_sec())
        assert state.command_fresh
        assert state.authority_valid
        assert state.command_valid

        conflicting = copy.deepcopy(message)
        conflicting.command.longitudinal.speed = 3.0
        mux.on_state_lattice_control_cmd(conflicting)
        state = mux._current_state_lattice_state(mux.now_sec())
        assert not state.command_valid
        assert not state.authority_valid
        assert mux.state_lattice_cmd_reason == "duplicate_conflict"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def _set_mux_ros_time_for_envelope(mux: HybridControlMuxNode, stamp_sec: int) -> None:
    """Test-only ROS clock setup; preserve the production clock-zero guard."""
    clock = mux.get_clock()
    clock._set_ros_time_is_active(True)
    clock.set_ros_time_override(Time(seconds=float(stamp_sec)))


def _bind_envelope_from_legacy(
    mux: HybridControlMuxNode,
    command: AckermannControlCommand,
    status: ControllerTrackingStatus,
    *,
    sequence: int,
) -> None:
    """Test-only same-value U2 transport peer for one legacy PP proof."""
    stamp_sec = int(command.stamp.sec)
    clock = mux.get_clock()
    clock._set_ros_time_is_active(True)
    # The envelope validator intentionally rejects a transport header that is
    # older than its bounded freshness window.  Keep this fixture's simulated
    # receipt inside that same window instead of relying on wall-clock time.
    clock.set_ros_time_override(Time(seconds=float(stamp_sec) + 0.05))
    for offset in (0, 1):
        envelope = _command_envelope(
            stamp_sec=stamp_sec,
            plan_generation=int(status.plan_generation),
            command_sequence=sequence + offset,
        )
        envelope.command = command
        _set_stamp(envelope.command.stamp, stamp_sec)
        _set_stamp(envelope.command.longitudinal.stamp, stamp_sec)
        _set_stamp(envelope.command.lateral.stamp, stamp_sec)
        envelope.mpc_horizon_usable = status.mpc_horizon_usable
        envelope.pp_command_fresh = status.pp_command_fresh
        envelope.trajectory_tracking_usable = status.trajectory_tracking_usable
        envelope.lateral_stop_authority_kind = (
            status.lateral_stop_authority_kind
        )
        envelope.lateral_stop_transaction_pass_direction = (
            status.lateral_stop_transaction_pass_direction
        )
        envelope.lateral_stop_authority_token = (
            status.lateral_stop_authority_token
        )
        envelope.command_age_sec = status.command_age_sec
        envelope.reason = status.reason
        mux.on_pure_pursuit_command_envelope(envelope)


def _spin_until(executor, predicate, timeout_sec: float = 0.5) -> bool:
    """Dispatch queued ROS callbacks until the expected sample is observed."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return True
    return bool(predicate())


def _valid_plan(*, stamp_sec: int, generation: int) -> OvertakePlan:
    plan = OvertakePlan()
    _set_stamp(plan.header.stamp, stamp_sec)
    plan.header.frame_id = "map"
    plan.phase = OvertakePlan.FREE_RUN
    plan.plan_generation = generation
    return plan


def _authorized_lateral_stop_plan(
    *, stamp_sec: int, generation: int
) -> OvertakePlan:
    plan = _valid_plan(stamp_sec=stamp_sec, generation=generation)
    plan.phase = OvertakePlan.ATTACK_FOLLOW
    plan.attempt_id = 1
    plan.target_vehicle_id = "grid_d2"
    plan.pass_direction = 0
    plan.trajectory_authorized = True
    plan.lateral_maneuver_required = True
    plan.lateral_stop_authority_kind = OvertakePlan.LATERAL_STOP_CURRENT_D_HOLD
    plan.lateral_stop_transaction_pass_direction = -1
    plan.lateral_stop_authority_token = (1 << 32) | generation
    plan.trajectory.header.frame_id = "map"
    _set_stamp(plan.trajectory.header.stamp, stamp_sec)
    point = TrajectoryPoint()
    point.pose.position.x = 1.0
    point.pose.position.y = 0.2
    point.longitudinal_velocity_mps = 0.5
    plan.trajectory.points.append(point)
    return plan


def _authorized_attack_follow_plan(
    *,
    stamp_sec: int,
    generation: int,
    attempt_id: int = 1,
    target_vehicle_id: str = "grid_d2",
    pass_direction: int = 0,
    x_offset_m: float = 0.0,
) -> OvertakePlan:
    plan = _valid_plan(stamp_sec=stamp_sec, generation=generation)
    plan.attempt_id = attempt_id
    plan.phase = OvertakePlan.ATTACK_FOLLOW
    plan.target_vehicle_id = target_vehicle_id
    plan.pass_direction = pass_direction
    plan.trajectory_authorized = True
    plan.lateral_maneuver_required = True
    plan.trajectory.header.frame_id = "map"
    _set_stamp(plan.trajectory.header.stamp, stamp_sec)
    for x_m in (1.0, 2.0):
        point = TrajectoryPoint()
        point.pose.position.x = x_m + x_offset_m
        point.pose.position.y = 0.2
        point.longitudinal_velocity_mps = 1.0
        plan.trajectory.points.append(point)
    return plan


def _authorized_passing_plan(
    *,
    stamp_sec: int,
    generation: int,
    attempt_id: int = 1,
    target_vehicle_id: str = "grid_d2",
    pass_direction: int = -1,
    x_offset_m: float = 0.0,
) -> OvertakePlan:
    plan = _authorized_attack_follow_plan(
        stamp_sec=stamp_sec,
        generation=generation,
        attempt_id=attempt_id,
        target_vehicle_id=target_vehicle_id,
        x_offset_m=x_offset_m,
    )
    plan.phase = OvertakePlan.PASSING
    plan.pass_direction = pass_direction
    return plan


def _bootstrap_current_d_stop_plan(
    *, stamp_sec: int, generation: int
) -> OvertakePlan:
    plan = _valid_plan(stamp_sec=stamp_sec, generation=generation)
    plan.phase = OvertakePlan.ATTACK_FOLLOW
    plan.target_vehicle_id = "grid_d2"
    plan.pass_direction = 0
    plan.trajectory_authorized = False
    plan.lateral_maneuver_required = True
    return plan


def _tracking_follow_stop_plan(
    *, stamp_sec: int, generation: int
) -> OvertakePlan:
    plan = _valid_plan(stamp_sec=stamp_sec, generation=generation)
    plan.phase = OvertakePlan.ATTACK_FOLLOW
    plan.target_vehicle_id = "grid_d2"
    plan.pass_direction = 0
    plan.trajectory_authorized = False
    plan.lateral_maneuver_required = False
    return plan


def _baseline_stop_plan(
    *, stamp_sec: int, generation: int
) -> OvertakePlan:
    plan = _valid_plan(stamp_sec=stamp_sec, generation=generation)
    plan.phase = OvertakePlan.ABORT_HOLD
    plan.pass_direction = 0
    plan.trajectory_authorized = False
    plan.lateral_maneuver_required = False
    return plan


def _constraint(
    *,
    stamp_sec: int,
    constraint_generation: int,
    plan_generation: int,
    speed_limit_mps: float = 3.0,
    stop_requested: bool = False,
    release_authorized: bool = True,
    reason: str = "",
) -> SafetyConstraint:
    constraint = SafetyConstraint()
    _set_stamp(constraint.header.stamp, stamp_sec)
    constraint.header.frame_id = "map"
    constraint.constraint_generation = constraint_generation
    constraint.plan_generation = plan_generation
    constraint.valid = True
    constraint.stop_requested = stop_requested
    constraint.release_authorized = release_authorized
    constraint.speed_limit_mps = speed_limit_mps
    constraint.reason = reason
    return constraint


def _free_run_execution_ack(*, stamp_sec: int, generation: int) -> FreeRunExecutionAck:
    ack = FreeRunExecutionAck()
    _set_stamp(ack.header.stamp, stamp_sec)
    ack.header.frame_id = "base_link"
    ack.schema_version = FreeRunExecutionAck.SCHEMA_V2
    ack.ack_eligible = True
    ack.evidence_state = FreeRunExecutionAck.EVIDENCE_COMPLETE
    ack.evidence_reason = "complete"
    ack.plan_key.canonical_algorithm_version = 1
    ack.plan_key.race_arm_epoch = 1
    ack.plan_key.planner_instance_id = 101
    ack.plan_key.plan_generation = generation
    _set_stamp(ack.plan_key.plan_stamp, stamp_sec)
    ack.plan_key.canonical_plan_payload_sha256 = [0x11] * 32
    ack.source_key.canonical_algorithm_version = 1
    ack.source_key.baseline_instance_id = 201
    ack.source_key.controller_instance_id = 201
    ack.source_key.source_generation = 3
    _set_stamp(ack.source_key.source_stamp, stamp_sec)
    ack.source_key.original_point_count = 10
    ack.source_key.baseline_reference_sha256 = [0x22] * 32
    ack.source_key.controller_implementation_sha256 = [0x33] * 32
    ack.source_key.controller_config_sha256 = [0x44] * 32
    ack.controller_sequence = 7
    _set_stamp(ack.controller_command_stamp, stamp_sec)
    ack.control_pose.orientation.w = 1.0
    _set_stamp(ack.control_pose_stamp, stamp_sec)
    ack.nearest_trajectory_index = 0
    ack.trajectory_progress_m = 0.0
    for command in (ack.raw_controller_command, ack.output_controller_command):
        _set_stamp(command.stamp, stamp_sec)
        _set_stamp(command.lateral.stamp, stamp_sec)
        _set_stamp(command.longitudinal.stamp, stamp_sec)
        command.longitudinal.speed = 2.0
        command.lateral.steering_tire_angle = 0.2
    ack.raw_controller_command_sha256 = [0x55] * 32
    ack.output_controller_command_sha256 = [0x66] * 32
    ack.raw_steering_tire_angle_rad = 0.2
    ack.output_steering_tire_angle_rad = 0.2
    ack.hard_steering_tire_angle_limit_rad = 0.3665191429188092
    ack.required_spatial_horizon_m = 0.5
    ack.available_spatial_horizon_m = 1.0
    geometry = ack.base_geometry
    geometry.frame_id = "map"
    _set_stamp(geometry.source_stamp, stamp_sec)
    geometry.original_point_count = 10
    geometry.first_source_index = 0
    geometry.last_source_index = 1
    geometry.nearest_source_index = 0
    geometry.speed_cap_source_index = 0
    geometry.curvature_last_read_source_index = 1
    geometry.lookahead_selected_source_index = 1
    geometry.required_horizon_end_source_index = 1
    geometry.required_spatial_horizon_m = 0.5
    geometry.total_arc_length_m = 1.0
    for x_m in (0.0, 1.0):
        point = TrajectoryPoint()
        point.pose.position.x = x_m
        point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 2.0
        geometry_point = CandidateExecutionPoint()
        geometry_point.position_x_m = point.pose.position.x
        geometry_point.orientation_w = 1.0
        geometry_point.longitudinal_velocity_mps = 2.0
        geometry.points.append(geometry_point)
    geometry.geometry_sha256 = [0x77] * 32
    geometry.start_point_sha256 = [0x78] * 32
    geometry.end_point_sha256 = [0x79] * 32
    geometry.full_source_digest_state = geometry.FULL_SOURCE_DIGEST_COMPLETE
    geometry.full_source_sha256 = [0x22] * 32
    ack.applied_geometry = copy.deepcopy(geometry)
    digest = HybridControlMuxNode._free_run_execution_ack_wire_digest(ack)
    assert digest is not None
    ack.ack_sha256 = list(digest)
    return ack


def _bind_free_run_plan_identity(mux, plan: OvertakePlan) -> bytes:
    digest = mux._canonical_free_run_plan_digest(plan)
    assert digest is not None
    plan.free_run_canonical_algorithm_version = (
        OvertakePlan.FREE_RUN_CANONICAL_ALGORITHM_V1
    )
    plan.free_run_canonical_payload_sha256 = list(digest)
    return digest


def _finalize_free_run_ack(
    mux, plan: OvertakePlan, ack: FreeRunExecutionAck
) -> FreeRunExecutionAck:
    ack.plan_key.canonical_plan_payload_sha256 = list(
        _bind_free_run_plan_identity(mux, plan)
    )
    ack.raw_controller_command_sha256 = list(
        mux._canonical_controller_command_digest(ack.raw_controller_command)
    )
    ack.output_controller_command_sha256 = list(
        mux._canonical_controller_command_digest(ack.output_controller_command)
    )
    digest = mux._free_run_execution_ack_wire_digest(ack)
    assert digest is not None
    ack.ack_sha256 = list(digest)
    return ack


def _source_key_from_ack(ack: FreeRunExecutionAck) -> FreeRunSourceKey:
    return copy.deepcopy(ack.source_key)


def _published_record_for_source(
    mux: HybridControlMuxNode,
    source: FreeRunSourceKey,
) -> FreeRunPublishedCommandRecord:
    command = AckermannControlCommand()
    command.lateral.steering_tire_angle = 0.2
    command_cdr = bytes(serialize_message(command))
    source_wire = mux._canonical_free_run_source_key_wire(source)
    assert source_wire is not None
    return FreeRunPublishedCommandRecord(
        publish_steady_time_sec=mux.now_sec(),
        race_arm_epoch=1,
        planner_instance_id=101,
        plan_generation=9,
        plan_stamp_ns=10_000_000_000,
        canonical_plan_sha256=bytes([0x11] * 32),
        source_identity=(
            int(source.baseline_instance_id),
            int(source.controller_instance_id),
            int(source.source_generation),
            mux._stamp_ns(source.source_stamp),
        ),
        source_semantic_identity=mux._free_run_source_semantic_identity(source),
        source_wire_sha256=hashlib.sha256(source_wire).digest(),
        envelope_identity=(201, 7, 10_000_000_000, 9),
        envelope_command_sha256=bytes([0x66] * 32),
        ack_identity=(1,) * 10,
        ack_wire_sha256=bytes([0x77] * 32),
        published_steering_rad=0.2,
        tracking_soft_limit_rad=0.35,
        actuator_hard_limit_rad=0.3665191429188092,
        final_command_cdr=command_cdr,
        final_command_sha256=hashlib.sha256(command_cdr).digest(),
    )


def test_free_run_execution_ack_canonical_v2_is_deterministic_and_rejects_v1():
    ack = _free_run_execution_ack(stamp_sec=10, generation=9)
    digest = HybridControlMuxNode._free_run_execution_ack_wire_digest(ack)

    assert digest is not None
    assert (
        digest.hex()
        == "c88364b6e367871eb4fa8f73a633450a"
        "99af2023a3a60d7a62aeab79c0e311d4"
    )
    assert all(
        HybridControlMuxNode._free_run_execution_ack_wire_digest(ack) == digest
        for _ in range(100)
    )
    with_claimed_digest = copy.deepcopy(ack)
    with_claimed_digest.ack_sha256 = list(digest)
    assert (
        HybridControlMuxNode._free_run_execution_ack_wire_digest(
            with_claimed_digest
        )
        == digest
    )
    hard_limit_mutation = copy.deepcopy(ack)
    hard_limit_mutation.hard_steering_tire_angle_limit_rad += 0.001
    assert (
        HybridControlMuxNode._free_run_execution_ack_wire_digest(
            hard_limit_mutation
        )
        != digest
    )
    v1 = copy.deepcopy(ack)
    v1.schema_version = FreeRunExecutionAck.SCHEMA_V1
    assert HybridControlMuxNode._free_run_execution_ack_wire_digest(v1) is None

    max_points = copy.deepcopy(ack)
    max_points.source_key.original_point_count = 100
    for geometry in (
        max_points.base_geometry,
        max_points.applied_geometry,
    ):
        geometry.original_point_count = 100
        geometry.last_source_index = 99
        geometry.curvature_last_read_source_index = 99
        geometry.lookahead_selected_source_index = 99
        geometry.required_horizon_end_source_index = 99
        geometry.total_arc_length_m = 99.0
        geometry.points.clear()
        for index in range(100):
            point = CandidateExecutionPoint()
            point.position_x_m = float(index)
            point.orientation_w = 1.0
            point.longitudinal_velocity_mps = 2.0
            geometry.points.append(point)
    max_digest = HybridControlMuxNode._free_run_execution_ack_wire_digest(
        max_points
    )
    assert max_digest is not None
    assert (
        max_digest.hex()
        == "1cc3c47ae9cc8361cec7855f78bd546a"
        "edc3b506afc2f3f0a0275bf5318d711d"
    )
    assert all(
        HybridControlMuxNode._free_run_execution_ack_wire_digest(max_points)
        == max_digest
        for _ in range(100)
    )

    invalid_time = copy.deepcopy(ack)
    invalid_time.control_pose_stamp.nanosec = 1_000_000_000
    assert (
        HybridControlMuxNode._free_run_execution_ack_wire_digest(invalid_time)
        is None
    )
    nonfinite = copy.deepcopy(ack)
    nonfinite.applied_geometry.points[1].position_x_m = math.inf
    assert (
        HybridControlMuxNode._free_run_execution_ack_wire_digest(nonfinite)
        is None
    )


def _command_envelope(
    *,
    stamp_sec: int,
    producer_instance_id: int = 17,
    command_sequence: int = 1,
    plan_generation: int = 9,
    command_age_sec: float = 0.0,
    trajectory_tracking_usable: bool = True,
    plan: OvertakePlan | None = None,
) -> ControllerCommandEnvelope:
    envelope = ControllerCommandEnvelope()
    _set_stamp(envelope.header.stamp, stamp_sec)
    envelope.header.frame_id = "base_link"
    envelope.schema_version = 2 if plan is not None else 1
    envelope.producer_instance_id = producer_instance_id
    envelope.command_sequence = command_sequence
    envelope.plan_generation = plan_generation
    _set_stamp(envelope.command.stamp, stamp_sec)
    _set_stamp(envelope.command.longitudinal.stamp, stamp_sec)
    envelope.command.longitudinal.speed = 1.5
    envelope.command.longitudinal.acceleration = -0.2
    envelope.command.longitudinal.jerk = 0.0
    _set_stamp(envelope.command.lateral.stamp, stamp_sec)
    envelope.command.lateral.steering_tire_angle = 0.1
    envelope.command.lateral.steering_tire_rotation_rate = 0.0
    envelope.mpc_horizon_usable = True
    envelope.pp_command_fresh = True
    envelope.trajectory_tracking_usable = trajectory_tracking_usable
    envelope.command_age_sec = command_age_sec
    envelope.reason = "ready" if trajectory_tracking_usable else "stale"
    if plan is not None:
        envelope.plan_sample_key.race_arm_epoch = plan.race_arm_epoch
        envelope.plan_sample_key.planner_instance_id = plan.planner_instance_id
        envelope.plan_sample_key.attempt_id = plan.attempt_id
        envelope.plan_sample_key.target_vehicle_id = plan.target_vehicle_id
        envelope.plan_sample_key.pass_direction = plan.pass_direction
        envelope.plan_sample_key.connector_transaction_id = (
            plan.connector_transaction_id
        )
        envelope.plan_sample_key.plan_stamp = plan.header.stamp
        envelope.plan_sample_key.plan_generation = plan.plan_generation
        envelope.candidate_revision = plan.candidate_revision
        envelope.candidate_content_sha256 = [
            int(value) for value in plan.candidate_content_sha256
        ]
    return envelope


def test_debug_pp_evaluation_snapshot_reports_invalid_cached_pair_without_selection():
    """An invalid PP envelope remains observable without becoming authority."""
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 100)
        invalid = _command_envelope(
            stamp_sec=100,
            trajectory_tracking_usable=False,
        )
        mux.pure_pursuit_cmd = copy.deepcopy(invalid.command)
        mux.on_pure_pursuit_command_envelope(invalid)
        mux.pure_pursuit_envelope_active_producer_instance_id = 17

        snapshot = mux._debug_pure_pursuit_evaluation_snapshot(
            None,
            selection_debug_snapshot={
                "command_stamp_sec": 100,
                "command_stamp_nanosec": 0,
                "envelope_command_stamp_sec": 100,
                "envelope_command_stamp_nanosec": 0,
                "tracking_usable": False,
                "selection_rejected_for_invalid_tracking": True,
                "selection_rejection_kind": "invalid_tracking",
            },
        )

        assert snapshot == {
            "command_stamp_sec": 100,
            "command_stamp_nanosec": 0,
            "envelope_command_stamp_sec": 100,
            "envelope_command_stamp_nanosec": 0,
            "tracking_usable": False,
            "selection_rejected_for_invalid_tracking": True,
            "selection_rejection_kind": "invalid_tracking",
        }
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_debug_pp_evaluation_snapshot_never_claims_invalid_rejection_on_stamp_mismatch():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 100)
        invalid = _command_envelope(
            stamp_sec=100,
            trajectory_tracking_usable=False,
        )
        mux.on_pure_pursuit_command_envelope(invalid)
        mux.pure_pursuit_envelope_active_producer_instance_id = 17
        mismatched = copy.deepcopy(invalid.command)
        _set_stamp(mismatched.stamp, 101)
        _set_stamp(mismatched.longitudinal.stamp, 101)
        _set_stamp(mismatched.lateral.stamp, 101)
        mux.pure_pursuit_cmd = mismatched

        snapshot = mux._debug_pure_pursuit_evaluation_snapshot(
            None,
            selection_debug_snapshot={
                "command_stamp_sec": 101,
                "command_stamp_nanosec": 0,
                "envelope_command_stamp_sec": 100,
                "envelope_command_stamp_nanosec": 0,
                "tracking_usable": False,
                "selection_rejected_for_invalid_tracking": False,
                "selection_rejection_kind": "command_envelope_stamp_mismatch",
            },
        )

        assert snapshot["tracking_usable"] is False
        assert snapshot["selection_rejected_for_invalid_tracking"] is False
        assert snapshot["selection_rejection_kind"] == "command_envelope_stamp_mismatch"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_debug_pp_evaluation_snapshot_preserves_selector_origin_for_timeout_cycle():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 100)
        invalid = _command_envelope(
            stamp_sec=100,
            trajectory_tracking_usable=False,
        )
        mux.pure_pursuit_cmd = copy.deepcopy(invalid.command)
        mux.on_pure_pursuit_command_envelope(invalid)
        mux.pure_pursuit_envelope_active_producer_instance_id = 17

        selector_origin: list[dict[str, object]] = []
        assert mux._select_bound_envelope_pp_motion_sample(
            tracking_plan_generation=9,
            now_sec=mux.now_sec(),
            ros_clock_stalled=False,
            debug_rejection_sink=selector_origin,
        ) is None
        snapshot = mux._debug_pure_pursuit_evaluation_snapshot(
            None, selection_debug_snapshot=selector_origin[-1]
        )

        assert snapshot["selection_rejected_for_invalid_tracking"] is True
        assert snapshot["selection_rejection_kind"] == "invalid_tracking"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_motion_authority_schema2_requires_complete_planner_identity():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 10)
        incomplete = _command_envelope(
            stamp_sec=10,
            trajectory_tracking_usable=True,
        )
        incomplete.schema_version = 2
        valid, reason = mux._validate_pure_pursuit_envelope(incomplete)
        assert not valid
        assert reason == "planner_candidate_identity"

        plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=9)
        plan.phase = OvertakePlan.PASSING
        plan.pass_direction = -1
        plan.aw2_identity_schema_version = 1
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        plan.attempt_id = 7
        plan.target_vehicle_id = "D2"
        plan.connector_transaction_id = 55
        plan.candidate_revision = 3
        plan.candidate_content_sha256 = [0x5A] * 32
        complete = _command_envelope(
            stamp_sec=10,
            trajectory_tracking_usable=True,
            plan=plan,
        )
        valid, reason = mux._validate_pure_pursuit_envelope(complete)
        assert valid
        assert reason == "valid"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_motion_authority_requires_exact_current_generation_envelope():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 10)
        armed = Bool()
        armed.data = True
        mux.on_race_armed(armed)
        plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=9)
        plan.phase = OvertakePlan.PASSING
        plan.pass_direction = -1
        plan.aw2_identity_schema_version = 1
        plan.race_arm_epoch = mux.finish_stop_latch.epoch
        plan.planner_instance_id = 101
        plan.attempt_id = 7
        plan.target_vehicle_id = "D2"
        plan.connector_transaction_id = 55
        plan.candidate_revision = 3
        plan.candidate_content_sha256 = [0x5A] * 32
        plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_PASS_WARMUP
        )
        plan.lateral_stop_transaction_pass_direction = -1
        plan.lateral_stop_authority_token = 0x700000009
        mux.on_overtake_plan(plan)

        envelope = _command_envelope(
            stamp_sec=10,
            command_sequence=4,
            plan_generation=9,
            plan=plan,
        )
        identity = mux._pure_pursuit_envelope_identity(envelope)
        now_sec = mux.now_sec()
        mux.pure_pursuit_envelope_cache[identity] = (
            mux._pure_pursuit_envelope_fingerprint(envelope),
            now_sec,
            True,
            "valid",
            envelope,
        )
        mux.pure_pursuit_envelope_active_producer_instance_id = int(
            envelope.producer_instance_id
        )
        mux.pure_pursuit_envelope_active_sequence = int(
            envelope.command_sequence
        )
        sample = mux._build_selected_pp_motion_sample(
            envelope.command,
            now_sec,
            envelope_identity=identity,
        )
        mux.motion_authority_warmup_proof = {
            "plan_identity": tuple(
                mux.overtake_plan_motion_identity_cache[9]
            ),
            "trajectory_xy": tuple(mux.overtake_plan_trajectory_cache[9]),
            "lateral_stop_authority_token": int(
                plan.lateral_stop_authority_token
            ),
            "pp_producer_instance_id": int(envelope.producer_instance_id),
            "pp_command_sequence": 3,
            "pp_command_stamp": envelope.header.stamp,
            "receipt_time_sec": float(now_sec),
        }
        constraint = SafetyConstraintState(
            constraint_generation=11,
            plan_generation=9,
            valid=True,
            stop_requested=False,
            release_authorized=True,
            speed_limit_mps=3.0,
            required_brake_decel_mps2=1.0,
            header_stamp_ns=10_000_000_000,
            frame_id="map",
            reason="release_authorized",
        )
        decision = SafetyConstraintDecision(
            stop_required=False,
            speed_limit_mps=3.0,
            required_brake_decel_mps2=1.0,
            constraint_generation=11,
            plan_generation=9,
            reason="release_authorized",
        )
        steering_result = mux.steering_limiter.reset(
            float(envelope.command.lateral.steering_tire_angle),
            now_sec,
            "pure_pursuit",
        )
        valid, reason, _, _ = mux._motion_authority_grant_eligible(
            final_command=envelope.command,
            selected_sample=sample,
            tracking_plan_generation=9,
            authority_plan_generation=9,
            authority_constraint=constraint,
            constraint_decision=decision,
            decision_source="pure_pursuit",
            now_sec=now_sec,
            active_control_fault_reason="",
            ros_clock_stalled=False,
            deadline_missed=False,
            steering_result=steering_result,
        )
        assert valid
        assert reason == "ready"

        saved_warmup_proof = mux.motion_authority_warmup_proof
        mux.motion_authority_warmup_proof = None
        valid, reason, _, _ = mux._motion_authority_grant_eligible(
            final_command=envelope.command,
            selected_sample=sample,
            tracking_plan_generation=9,
            authority_plan_generation=9,
            authority_constraint=constraint,
            constraint_decision=decision,
            decision_source="pure_pursuit",
            now_sec=now_sec,
            active_control_fault_reason="",
            ros_clock_stalled=False,
            deadline_missed=False,
            steering_result=steering_result,
        )
        assert not valid
        assert reason == "warmup_proof_missing"
        mux.motion_authority_warmup_proof = saved_warmup_proof

        stale_identity = (
            identity[0],
            identity[1],
            identity[2],
            8,
        )
        stale_sample = mux._build_selected_pp_motion_sample(
            envelope.command,
            now_sec,
            envelope_identity=stale_identity,
        )
        valid, reason, _, _ = mux._motion_authority_grant_eligible(
            final_command=envelope.command,
            selected_sample=stale_sample,
            tracking_plan_generation=9,
            authority_plan_generation=9,
            authority_constraint=constraint,
            constraint_decision=decision,
            decision_source="pure_pursuit",
            now_sec=now_sec,
            active_control_fault_reason="",
            ros_clock_stalled=False,
            deadline_missed=False,
            steering_result=steering_result,
        )
        assert not valid
        assert reason == "envelope_schema"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_motion_authority_captures_exact_pass_warmup_proof():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 10)
        plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=9)
        plan.phase = OvertakePlan.PASSING
        plan.pass_direction = -1
        plan.aw2_identity_schema_version = 1
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        plan.attempt_id = 7
        plan.target_vehicle_id = "D2"
        plan.connector_transaction_id = 55
        plan.candidate_revision = 3
        plan.candidate_content_sha256 = [0x5A] * 32
        plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_PASS_WARMUP
        )
        plan.lateral_stop_transaction_pass_direction = -1
        plan.lateral_stop_authority_token = 0x700000009
        mux.on_overtake_plan(plan)

        for sequence in (1, 2):
            envelope = _command_envelope(
                stamp_sec=10,
                command_sequence=sequence,
                plan_generation=9,
                plan=plan,
            )
            envelope.lateral_stop_authority_kind = (
                OvertakePlan.LATERAL_STOP_PASS_WARMUP
            )
            envelope.lateral_stop_transaction_pass_direction = -1
            envelope.lateral_stop_authority_token = (
                plan.lateral_stop_authority_token
            )
            mux.on_pure_pursuit_command_envelope(envelope)

        identity = mux._pure_pursuit_envelope_identity(envelope)
        now_sec = mux.now_sec()
        sample = mux._build_selected_pp_motion_sample(
            envelope.command,
            now_sec,
            envelope_identity=identity,
        )
        constraint = SafetyConstraintState(
            constraint_generation=10,
            plan_generation=9,
            valid=True,
            stop_requested=True,
            release_authorized=False,
            speed_limit_mps=0.0,
            required_brake_decel_mps2=1.0,
            header_stamp_ns=10_000_000_000,
            frame_id="map",
            reason="release_pending_safe_cycles",
        )

        record = mux.pure_pursuit_envelope_cache[identity]
        wrong_token_envelope = copy.deepcopy(record[4])
        wrong_token_envelope.lateral_stop_authority_token += 1
        mux.pure_pursuit_envelope_cache[identity] = (
            record[0],
            record[1],
            record[2],
            record[3],
            wrong_token_envelope,
        )
        assert not mux._capture_motion_authority_warmup_proof(
            selected_sample=sample,
            tracking_plan_generation=9,
            authority_plan_generation=9,
            authority_constraint=constraint,
            now_sec=now_sec,
        )
        mux.pure_pursuit_envelope_cache[identity] = record
        assert mux._capture_motion_authority_warmup_proof(
            selected_sample=sample,
            tracking_plan_generation=9,
            authority_plan_generation=9,
            authority_constraint=constraint,
            now_sec=now_sec,
        )
        proof = mux.motion_authority_warmup_proof
        assert proof is not None
        assert proof["lateral_stop_authority_token"] == 0x700000009
        assert proof["pp_producer_instance_id"] == 17
        assert proof["pp_command_sequence"] == 2
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_motion_authority_rejects_direct_successor_same_xy_changed_digest():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 10)
        armed = Bool()
        armed.data = True
        mux.on_race_armed(armed)
        warmup_plan = _authorized_lateral_stop_plan(
            stamp_sec=10, generation=9
        )
        warmup_plan.phase = OvertakePlan.PASSING
        warmup_plan.pass_direction = -1
        warmup_plan.aw2_identity_schema_version = 1
        warmup_plan.race_arm_epoch = mux.finish_stop_latch.epoch
        warmup_plan.planner_instance_id = 101
        warmup_plan.attempt_id = 7
        warmup_plan.target_vehicle_id = "D2"
        warmup_plan.connector_transaction_id = 55
        warmup_plan.candidate_revision = 3
        warmup_plan.candidate_content_sha256 = [0x5A] * 32
        warmup_plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_PASS_WARMUP
        )
        warmup_plan.lateral_stop_transaction_pass_direction = -1
        warmup_plan.lateral_stop_authority_token = 0x700000009
        mux.on_overtake_plan(warmup_plan)
        for sequence in (1, 2):
            warmup_envelope = _command_envelope(
                stamp_sec=10,
                command_sequence=sequence,
                plan_generation=9,
                plan=warmup_plan,
            )
            warmup_envelope.lateral_stop_authority_kind = (
                OvertakePlan.LATERAL_STOP_PASS_WARMUP
            )
            warmup_envelope.lateral_stop_transaction_pass_direction = -1
            warmup_envelope.lateral_stop_authority_token = (
                warmup_plan.lateral_stop_authority_token
            )
            mux.on_pure_pursuit_command_envelope(warmup_envelope)
        warmup_identity = mux._pure_pursuit_envelope_identity(
            warmup_envelope
        )
        now_sec = mux.now_sec()
        warmup_sample = mux._build_selected_pp_motion_sample(
            warmup_envelope.command,
            now_sec,
            envelope_identity=warmup_identity,
        )
        warmup_constraint = SafetyConstraintState(
            constraint_generation=1,
            plan_generation=9,
            valid=True,
            stop_requested=True,
            release_authorized=False,
            speed_limit_mps=0.0,
            required_brake_decel_mps2=1.0,
            header_stamp_ns=10_000_000_000,
            frame_id="map",
            reason="release_pending_safe_cycles",
        )
        assert mux._capture_motion_authority_warmup_proof(
            selected_sample=warmup_sample,
            tracking_plan_generation=9,
            authority_plan_generation=9,
            authority_constraint=warmup_constraint,
            now_sec=now_sec,
        )

        _set_mux_ros_time_for_envelope(mux, 11)
        motion_plan = copy.deepcopy(warmup_plan)
        _set_stamp(motion_plan.header.stamp, 11)
        _set_stamp(motion_plan.trajectory.header.stamp, 11)
        motion_plan.plan_generation = 10
        motion_plan.candidate_revision = 4
        motion_plan.candidate_content_sha256 = [0x5B] * 32
        motion_plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_NONE
        )
        motion_plan.lateral_stop_transaction_pass_direction = 0
        motion_plan.lateral_stop_authority_token = 0
        mux.on_overtake_plan(motion_plan)
        assert (
            tuple(mux.overtake_plan_trajectory_cache[10])
            == tuple(mux.overtake_plan_trajectory_cache[9])
        )
        motion_envelope = _command_envelope(
            stamp_sec=11,
            command_sequence=3,
            plan_generation=10,
            plan=motion_plan,
        )
        mux.on_pure_pursuit_command_envelope(motion_envelope)
        motion_identity = mux._pure_pursuit_envelope_identity(
            motion_envelope
        )
        motion_sample = mux._build_selected_pp_motion_sample(
            motion_envelope.command,
            mux.now_sec(),
            envelope_identity=motion_identity,
        )
        motion_constraint = SafetyConstraintState(
            constraint_generation=2,
            plan_generation=10,
            valid=True,
            stop_requested=False,
            release_authorized=True,
            speed_limit_mps=3.0,
            required_brake_decel_mps2=1.0,
            header_stamp_ns=11_000_000_000,
            frame_id="map",
            reason="release_authorized",
        )
        decision = SafetyConstraintDecision(
            stop_required=False,
            speed_limit_mps=3.0,
            required_brake_decel_mps2=1.0,
            constraint_generation=2,
            plan_generation=10,
            reason="release_authorized",
        )
        steering_result = mux.steering_limiter.reset(
            float(motion_envelope.command.lateral.steering_tire_angle),
            mux.now_sec(),
            "pure_pursuit",
        )
        valid, reason, _, _ = mux._motion_authority_grant_eligible(
            final_command=motion_envelope.command,
            selected_sample=motion_sample,
            tracking_plan_generation=10,
            authority_plan_generation=10,
            authority_constraint=motion_constraint,
            constraint_decision=decision,
            decision_source="pure_pursuit",
            now_sec=mux.now_sec(),
            active_control_fault_reason="",
            ros_clock_stalled=False,
            deadline_missed=False,
            steering_result=steering_result,
        )
        assert not valid
        assert reason == "warmup_candidate_mutation"
        assert mux.motion_authority_warmup_proof is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "mutation",
    [
        "warmup_plan",
        "warmup_candidate",
        "warmup_trajectory",
        "warmup_producer",
        "warmup_sequence",
        "motion_plan",
        "motion_candidate",
        "motion_producer",
        "motion_sequence",
        "motion_stamp",
    ],
)
def test_motion_authority_rejects_each_warmup_or_motion_binding_mutation(
    mutation,
):
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 10)
        armed = Bool()
        armed.data = True
        mux.on_race_armed(armed)
        plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=9)
        plan.phase = OvertakePlan.PASSING
        plan.pass_direction = -1
        plan.aw2_identity_schema_version = 1
        plan.race_arm_epoch = mux.finish_stop_latch.epoch
        plan.planner_instance_id = 101
        plan.attempt_id = 7
        plan.target_vehicle_id = "D2"
        plan.connector_transaction_id = 55
        plan.candidate_revision = 3
        plan.candidate_content_sha256 = [0x5A] * 32
        mux.on_overtake_plan(plan)

        envelope = _command_envelope(
            stamp_sec=10,
            command_sequence=4,
            plan_generation=9,
            plan=plan,
        )
        identity = mux._pure_pursuit_envelope_identity(envelope)
        now_sec = mux.now_sec()
        mux.pure_pursuit_envelope_cache[identity] = (
            mux._pure_pursuit_envelope_fingerprint(envelope),
            now_sec,
            True,
            "valid",
            envelope,
        )
        mux.pure_pursuit_envelope_active_producer_instance_id = int(
            envelope.producer_instance_id
        )
        mux.pure_pursuit_envelope_active_sequence = int(
            envelope.command_sequence
        )
        sample_identity = identity
        warmup_identity = tuple(
            mux.overtake_plan_motion_identity_cache[9]
        )
        warmup_trajectory = tuple(mux.overtake_plan_trajectory_cache[9])
        warmup_producer = int(envelope.producer_instance_id)
        warmup_sequence = 3
        if mutation == "warmup_plan":
            mutated = list(warmup_identity)
            mutated[3] = "D3"
            warmup_identity = tuple(mutated)
        elif mutation == "warmup_candidate":
            mutated = list(warmup_identity)
            mutated[9] += 1
            warmup_identity = tuple(mutated)
        elif mutation == "warmup_trajectory":
            warmup_trajectory = (*warmup_trajectory, (99.0, 99.0))
        elif mutation == "warmup_producer":
            warmup_producer += 1
        elif mutation == "warmup_sequence":
            warmup_sequence = int(envelope.command_sequence)
        elif mutation in ("motion_plan", "motion_candidate"):
            mutated_envelope = copy.deepcopy(envelope)
            if mutation == "motion_plan":
                mutated_envelope.plan_sample_key.target_vehicle_id = "D3"
            else:
                mutated_envelope.candidate_content_sha256[0] ^= 0xFF
            record = mux.pure_pursuit_envelope_cache[identity]
            mux.pure_pursuit_envelope_cache[identity] = (
                record[0],
                record[1],
                record[2],
                record[3],
                mutated_envelope,
            )
        elif mutation == "motion_producer":
            sample_identity = (
                identity[0] + 1,
                identity[1],
                identity[2],
                identity[3],
            )
        elif mutation == "motion_sequence":
            sample_identity = (
                identity[0],
                identity[1] - 1,
                identity[2],
                identity[3],
            )
        elif mutation == "motion_stamp":
            sample_identity = (
                identity[0],
                identity[1],
                identity[2] + 1,
                identity[3],
            )
        mux.motion_authority_warmup_proof = {
            "plan_identity": warmup_identity,
            "trajectory_xy": warmup_trajectory,
            "lateral_stop_authority_token": 0x700000009,
            "pp_producer_instance_id": warmup_producer,
            "pp_command_sequence": warmup_sequence,
            "pp_command_stamp": envelope.header.stamp,
            "receipt_time_sec": float(now_sec),
        }
        sample = mux._build_selected_pp_motion_sample(
            envelope.command,
            now_sec,
            envelope_identity=sample_identity,
        )
        constraint = SafetyConstraintState(
            constraint_generation=11,
            plan_generation=9,
            valid=True,
            stop_requested=False,
            release_authorized=True,
            speed_limit_mps=3.0,
            required_brake_decel_mps2=1.0,
            header_stamp_ns=10_000_000_000,
            frame_id="map",
            reason="release_authorized",
        )
        decision = SafetyConstraintDecision(
            stop_required=False,
            speed_limit_mps=3.0,
            required_brake_decel_mps2=1.0,
            constraint_generation=11,
            plan_generation=9,
            reason="release_authorized",
        )
        steering_result = mux.steering_limiter.reset(
            float(envelope.command.lateral.steering_tire_angle),
            now_sec,
            "pure_pursuit",
        )

        valid, _, _, _ = mux._motion_authority_grant_eligible(
            final_command=envelope.command,
            selected_sample=sample,
            tracking_plan_generation=9,
            authority_plan_generation=9,
            authority_constraint=constraint,
            constraint_decision=decision,
            decision_source="pure_pursuit",
            now_sec=now_sec,
            active_control_fault_reason="",
            ros_clock_stalled=False,
            deadline_missed=False,
            steering_result=steering_result,
        )
        assert not valid
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_motion_authority_publish_precedes_exact_positive_command():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    events = []

    class _Recorder:
        def __init__(self, name):
            self.name = name

        def publish(self, message):
            events.append((self.name, message))

    mux.motion_authority_grant_pub = _Recorder("grant")
    mux.tracking_status_pub = _Recorder("tracking")
    mux.control_pub = _Recorder("control")
    try:
        armed = Bool()
        armed.data = True
        mux.on_race_armed(armed)
        mux.ros_clock_motion_ready = True
        _set_mux_ros_time_for_envelope(mux, 10)

        plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=9)
        plan.phase = OvertakePlan.PASSING
        plan.pass_direction = -1
        plan.aw2_identity_schema_version = 1
        plan.race_arm_epoch = mux.finish_stop_latch.epoch
        plan.planner_instance_id = 101
        plan.attempt_id = 7
        plan.target_vehicle_id = "D2"
        plan.connector_transaction_id = 55
        plan.candidate_revision = 3
        plan.candidate_content_sha256 = [0x5A] * 32
        plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_PASS_WARMUP
        )
        plan.lateral_stop_transaction_pass_direction = -1
        plan.lateral_stop_authority_token = 0x700000009
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=1,
                plan_generation=9,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="release_pending_safe_cycles",
            )
        )

        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 10)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 9
        proof.mpc_horizon_usable = True
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_PASS_WARMUP
        )
        proof.lateral_stop_transaction_pass_direction = -1
        proof.lateral_stop_authority_token = plan.lateral_stop_authority_token
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        for sequence in (1, 2):
            warmup = _command_envelope(
                stamp_sec=10,
                command_sequence=sequence,
                plan_generation=9,
                plan=plan,
            )
            warmup.lateral_stop_authority_kind = (
                OvertakePlan.LATERAL_STOP_PASS_WARMUP
            )
            warmup.lateral_stop_transaction_pass_direction = -1
            warmup.lateral_stop_authority_token = (
                plan.lateral_stop_authority_token
            )
            mux.on_pure_pursuit_command_envelope(warmup)
        mux.on_pure_pursuit_cmd(warmup.command)
        mux.on_timer()
        assert mux.motion_authority_warmup_proof is not None
        warmup_control = next(
            msg for name, msg in reversed(events) if name == "control"
        )
        assert warmup_control.longitudinal.speed == pytest.approx(0.0)
        warmup_tracking = next(
            msg for name, msg in reversed(events) if name == "tracking"
        )
        assert warmup_tracking.pass_probe_exact_current_usable
        assert (
            warmup_tracking.pass_probe_lateral_stop_authority_token
            == plan.lateral_stop_authority_token
        )
        assert warmup_tracking.plan_generation == plan.plan_generation

        _set_mux_ros_time_for_envelope(mux, 11)
        motion_plan = _authorized_lateral_stop_plan(
            stamp_sec=11, generation=10
        )
        motion_plan.phase = OvertakePlan.PASSING
        motion_plan.pass_direction = -1
        motion_plan.aw2_identity_schema_version = 1
        motion_plan.race_arm_epoch = mux.finish_stop_latch.epoch
        motion_plan.planner_instance_id = 101
        motion_plan.attempt_id = 7
        motion_plan.target_vehicle_id = "D2"
        motion_plan.connector_transaction_id = 55
        motion_plan.candidate_revision = 4
        motion_plan.candidate_content_sha256 = [0x5A] * 32
        motion_plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_NONE
        )
        motion_plan.lateral_stop_transaction_pass_direction = 0
        motion_plan.lateral_stop_authority_token = 0
        mux.on_overtake_plan(motion_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=2,
                plan_generation=10,
                speed_limit_mps=3.0,
                stop_requested=False,
                release_authorized=True,
                reason="release_authorized",
            )
        )
        motion = _command_envelope(
            stamp_sec=11,
            command_sequence=3,
            plan_generation=10,
            plan=motion_plan,
        )
        mux.on_pure_pursuit_command_envelope(motion)
        mux.on_pure_pursuit_cmd(motion.command)
        _set_stamp(proof.header.stamp, 11)
        proof.plan_generation = 10
        proof.lateral_stop_authority_kind = OvertakePlan.LATERAL_STOP_NONE
        proof.lateral_stop_transaction_pass_direction = 0
        proof.lateral_stop_authority_token = 0
        mux.on_pure_pursuit_tracking_status(proof)
        events.clear()
        for _ in range(6):
            mux.on_timer()
            if any(name == "grant" and msg.valid for name, msg in events):
                break

        positive_index = next(
            index
            for index, (name, msg) in enumerate(events)
            if name == "control" and msg.longitudinal.speed > 0.0
        )
        grant_index = next(
            index
            for index, (name, msg) in enumerate(events)
            if name == "grant" and msg.valid
        )
        assert grant_index < positive_index
        grant = events[grant_index][1]
        command = events[positive_index][1]
        assert grant.pp_command_sequence == 3
        assert grant.warmup_pp_command_sequence == 2
        assert grant.speed_mps == pytest.approx(command.longitudinal.speed)
        assert grant.acceleration_mps2 == pytest.approx(
            command.longitudinal.acceleration
        )
        assert grant.steering_tire_angle_rad == pytest.approx(
            command.lateral.steering_tire_angle
        )
        assert grant.steering_tire_rotation_rate_radps == pytest.approx(
            command.lateral.steering_tire_rotation_rate
        )
        assert grant.header.stamp == command.stamp
        assert grant.pp_command_stamp == motion.header.stamp

        valid_grant_sequence = grant.grant_sequence
        _set_mux_ros_time_for_envelope(mux, 12)
        revoke_plan = _authorized_passing_plan(
            stamp_sec=12,
            generation=11,
            attempt_id=7,
            target_vehicle_id="D2",
            pass_direction=-1,
        )
        revoke_plan.aw2_identity_schema_version = 1
        revoke_plan.race_arm_epoch = mux.finish_stop_latch.epoch
        revoke_plan.planner_instance_id = 101
        revoke_plan.connector_transaction_id = 55
        revoke_plan.candidate_revision = 5
        revoke_plan.candidate_content_sha256 = [0x5C] * 32
        mux.on_overtake_plan(revoke_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=12,
                constraint_generation=3,
                plan_generation=11,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="safe_stop",
            )
        )
        events.clear()
        mux.on_timer()

        revoke_index = next(
            index
            for index, (name, msg) in enumerate(events)
            if name == "grant" and not msg.valid
        )
        stop_index = next(
            index
            for index, (name, msg) in enumerate(events)
            if name == "control"
            and msg.longitudinal.speed == pytest.approx(0.0)
        )
        revoke = events[revoke_index][1]
        assert revoke_index < stop_index
        assert revoke.lease_duration_sec == pytest.approx(0.0)
        assert revoke.grant_sequence == valid_grant_sequence + 1
        assert mux.motion_authority_warmup_proof is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_passing_positive_mpc_command_cannot_bypass_motion_grant():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=mpc",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "mpc_cmd_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    commands = []

    class _Recorder:
        def publish(self, message):
            commands.append(message)

    mux.control_pub = _Recorder()
    try:
        armed = Bool()
        armed.data = True
        mux.on_race_armed(armed)
        mux.ros_clock_motion_ready = True
        plan = _authorized_passing_plan(stamp_sec=10, generation=9)
        plan.aw2_identity_schema_version = 1
        plan.race_arm_epoch = mux.finish_stop_latch.epoch
        plan.planner_instance_id = 101
        plan.connector_transaction_id = 55
        plan.candidate_revision = 3
        plan.candidate_content_sha256 = [0x5A] * 32
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=1,
                plan_generation=9,
                speed_limit_mps=3.0,
                stop_requested=False,
                release_authorized=True,
                reason="release_authorized",
            )
        )
        command = AckermannControlCommand()
        _set_stamp(command.stamp, 10)
        command.longitudinal.speed = 1.5
        command.longitudinal.acceleration = 0.2
        command.lateral.steering_tire_angle = 0.1
        mux.on_mpc_cmd(command)

        mux.on_timer()

        assert commands
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= 0.0
        assert mux.motion_authority_grant_active is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def _matching_legacy_tracking_status(stamp_sec: int) -> ControllerTrackingStatus:
    status = ControllerTrackingStatus()
    _set_stamp(status.header.stamp, stamp_sec)
    status.header.frame_id = "base_link"
    status.plan_generation = 9
    status.mpc_horizon_usable = True
    status.pp_command_fresh = True
    status.trajectory_tracking_usable = True
    status.command_age_sec = 0.0
    status.reason = "ready"
    return status


def _matching_legacy_command(stamp_sec: int) -> AckermannControlCommand:
    command = AckermannControlCommand()
    _set_stamp(command.stamp, stamp_sec)
    _set_stamp(command.longitudinal.stamp, stamp_sec)
    command.longitudinal.speed = 1.5
    command.longitudinal.acceleration = -0.2
    command.longitudinal.jerk = 0.0
    _set_stamp(command.lateral.stamp, stamp_sec)
    command.lateral.steering_tire_angle = 0.1
    command.lateral.steering_tire_rotation_rate = 0.0
    return command


@pytest.mark.parametrize(
    ("authority_kind", "phase", "published_direction", "constraint_reason"),
    [
        (
            OvertakePlan.LATERAL_STOP_CURRENT_D_HOLD,
            OvertakePlan.ATTACK_FOLLOW,
            0,
            "maneuver_transaction_tracking_stop",
        ),
        (
            OvertakePlan.LATERAL_STOP_PASS_WARMUP,
            OvertakePlan.PASSING,
            -1,
            "release_pending_safe_cycles",
        ),
    ],
)
def test_typed_lateral_stop_authority_requires_exact_pp_proof(
    authority_kind, phase, published_direction, constraint_reason
):
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=7)
        plan.phase = phase
        plan.pass_direction = published_direction
        plan.lateral_stop_authority_kind = authority_kind
        plan.lateral_stop_transaction_pass_direction = -1
        plan.lateral_stop_authority_token = 0x700000001
        mux.on_overtake_plan(plan)
        constraint = SafetyConstraintState(
            constraint_generation=1,
            plan_generation=7,
            valid=True,
            stop_requested=True,
            release_authorized=False,
            speed_limit_mps=0.5,
            required_brake_decel_mps2=1.5,
            header_stamp_ns=10_000_000_000,
            reason=constraint_reason,
        )
        proof = ControllerTrackingStatus()
        proof.plan_generation = 7
        proof.trajectory_tracking_usable = True
        proof.lateral_stop_authority_kind = authority_kind
        proof.lateral_stop_transaction_pass_direction = -1
        proof.lateral_stop_authority_token = plan.lateral_stop_authority_token

        assert mux._lateral_stop_plan_authorized(
            plan_generation=7,
            constraint=constraint,
            tracking_status=proof,
            now_sec=mux.now_sec(),
        )

        proof.lateral_stop_authority_token += 1
        assert not mux._lateral_stop_plan_authorized(
            plan_generation=7,
            constraint=constraint,
            tracking_status=proof,
            now_sec=mux.now_sec(),
        )
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("mutation", ["position", "orientation", "velocity"])
def test_typed_lateral_stop_same_generation_mutation_is_sticky(mutation):
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan_a = _authorized_lateral_stop_plan(stamp_sec=10, generation=7)
        mux.on_overtake_plan(plan_a)
        proof = ControllerTrackingStatus()
        proof.plan_generation = 7
        proof.trajectory_tracking_usable = True
        proof.lateral_stop_authority_kind = plan_a.lateral_stop_authority_kind
        proof.lateral_stop_transaction_pass_direction = (
            plan_a.lateral_stop_transaction_pass_direction
        )
        proof.lateral_stop_authority_token = plan_a.lateral_stop_authority_token

        plan_b = _authorized_lateral_stop_plan(stamp_sec=11, generation=7)
        if mutation == "position":
            plan_b.trajectory.points[0].pose.position.x += 0.01
        elif mutation == "orientation":
            plan_b.trajectory.points[0].pose.orientation.z = 0.01
        else:
            plan_b.trajectory.points[0].longitudinal_velocity_mps += 0.01
        mux.on_overtake_plan(plan_b)
        # Bの再送で同generationのtaintを解除してはならない。
        mux.on_overtake_plan(plan_b)
        constraint = SafetyConstraintState(
            constraint_generation=1,
            plan_generation=7,
            valid=True,
            stop_requested=True,
            release_authorized=False,
            speed_limit_mps=0.5,
            required_brake_decel_mps2=1.5,
            header_stamp_ns=11_000_000_000,
            reason="maneuver_transaction_tracking_stop",
        )
        assert not mux._lateral_stop_plan_authorized(
            plan_generation=7,
            constraint=constraint,
            tracking_status=proof,
            now_sec=mux.now_sec(),
        )
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_typed_lateral_stop_taint_is_not_evicted_before_race_epoch():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        for generation in range(7, 16):
            stamp_sec = generation * 2
            plan_a = _authorized_lateral_stop_plan(
                stamp_sec=stamp_sec, generation=generation
            )
            mux.on_overtake_plan(plan_a)
            plan_b = _authorized_lateral_stop_plan(
                stamp_sec=stamp_sec + 1, generation=generation
            )
            plan_b.trajectory.points[0].pose.position.x += 0.01
            mux.on_overtake_plan(plan_b)

        assert len(mux.overtake_plan_lateral_stop_taint_cache) == 9
        assert mux.overtake_plan_lateral_stop_taint_cache[7]

        race_armed = Bool()
        race_armed.data = True
        mux.on_race_armed(race_armed)
        assert not mux.overtake_plan_lateral_stop_taint_cache
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "field_name",
    [
        "position_z",
        "orientation_w",
        "lateral_velocity",
        "acceleration",
        "heading_rate",
        "front_wheel_angle",
        "rear_wheel_angle",
    ],
)
def test_typed_lateral_stop_rejects_nonfinite_trajectory_fields(field_name):
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=7)
        point = plan.trajectory.points[0]
        if field_name == "position_z":
            point.pose.position.z = math.inf
        elif field_name == "orientation_w":
            point.pose.orientation.w = math.nan
        elif field_name == "lateral_velocity":
            point.lateral_velocity_mps = math.inf
        elif field_name == "acceleration":
            point.acceleration_mps2 = math.nan
        elif field_name == "heading_rate":
            point.heading_rate_rps = math.inf
        elif field_name == "front_wheel_angle":
            point.front_wheel_angle_rad = math.nan
        else:
            point.rear_wheel_angle_rad = math.inf

        mux.on_overtake_plan(plan)
        assert not mux.overtake_plan_valid
        assert not mux.overtake_plan_cache[7][0]
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_selected_pp_motion_sample_snapshots_legacy_command_and_proof_identity():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        command = _matching_legacy_command(7)
        proof = _matching_legacy_tracking_status(7)
        sample = mux._build_selected_pp_motion_sample(
            command,
            12.0,
            authority_proof=proof,
            authority_proof_receipt_time_sec=12.5,
            authority_proof_valid=True,
        )

        command.longitudinal.speed = 9.0
        proof.plan_generation = 99
        assert sample.command.longitudinal.speed == pytest.approx(1.5)
        assert sample.authority_proof is not None
        assert sample.authority_proof.plan_generation == 9
        assert sample.command_stamp_ns == 7_000_000_000
        assert sample.command_receipt_time_sec == pytest.approx(12.0)
        assert sample.authority_proof_receipt_time_sec == pytest.approx(12.5)
        assert sample.authority_proof_identity == (7_000_000_000, 9)
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_n1_transport_continuity_accepts_selected_motion_sample_seam():
    """N-1 continuity reads one immutable PP command/proof snapshot."""
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        sample = mux._build_selected_pp_motion_sample(
            _matching_legacy_command(7),
            12.0,
            authority_proof=_matching_legacy_tracking_status(7),
            authority_proof_receipt_time_sec=12.5,
            authority_proof_valid=True,
        )
        decision = SafetyConstraintDecision(
            stop_required=False,
            speed_limit_mps=3.0,
            required_brake_decel_mps2=0.0,
            constraint_generation=1,
            plan_generation=1,
            reason="ready",
        )

        # This deliberately incomplete authority bundle stays fail-closed.
        # It verifies only that the helper accepts the immutable seam, before
        # the existing ATTACK_FOLLOW/PASS integration tests prove its output.
        assert not mux._maneuver_transport_continuity_authorized(
            tracking_plan_generation=1,
            authority_plan_generation=None,
            authority_constraint=None,
            authority_constraint_time_sec=None,
            constraint_decision=decision,
            selected_motion_sample=sample,
            selected_input_source="pure_pursuit",
            pp_tracking_delivery_gap_usable=True,
            active_control_fault_reason="",
            now_sec=12.5,
            expected_phase=OvertakePlan.ATTACK_FOLLOW,
        )
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_observed_usable_proof_never_becomes_motion_authority():
    """A latest diagnostic proof cannot substitute for an exact PP binding."""
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        mux.on_overtake_plan(_valid_plan(stamp_sec=7, generation=7))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=7,
                constraint_generation=7,
                plan_generation=7,
            )
        )
        command = _matching_legacy_command(7)
        mux.on_pure_pursuit_cmd(command)
        observed_only = _matching_legacy_tracking_status(8)
        observed_only.plan_generation = 7
        mux.on_pure_pursuit_tracking_status(observed_only)

        sample = mux._select_legacy_pp_motion_sample(
            tracking_plan_generation=7
        )
        assert sample is not None
        assert sample.authority_proof is None
        assert sample.observed_proof is not None
        assert sample.observed_proof.trajectory_tracking_usable is True

        mux.on_timer()
        assert mux.last_source == "stop"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_u2_required_envelope_moves_without_legacy_and_legacy_only_stops():
    """U2 required authority never falls back from a bound envelope to raw PP."""
    rclpy.init(
        args=[
            "--ros-args", "-p", "primary_source:=pure_pursuit", "-p",
            "require_safety_constraint:=true", "-p",
            "safety_constraint_timeout_sec:=1.0", "-p",
            "overtake_plan_timeout_sec:=1.0", "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0", "-p",
            "pure_pursuit_envelope_timeout_sec:=1.0", "-p",
            "control_loop_max_gap_sec:=10.0", "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 11)
        mux.on_overtake_plan(_valid_plan(stamp_sec=10, generation=2))
        mux.on_safety_constraint(
            _constraint(stamp_sec=10, constraint_generation=2, plan_generation=2)
        )
        # Two consecutive envelope records bind producer 17.  No legacy
        # command/status is sent: U2 must still authorize this exact sample.
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=10, plan_generation=2, command_sequence=1)
        )
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=11, plan_generation=2, command_sequence=2)
        )
        assert mux.pure_pursuit_envelope_active_producer_instance_id == 17
        assert mux._select_bound_envelope_pp_motion_sample(
            tracking_plan_generation=2,
            now_sec=mux.now_sec(),
            ros_clock_stalled=False,
        ) is not None
        mux.on_timer()
        assert mux.last_source == "pure_pursuit"
        assert mux.pure_pursuit_cmd is None

        # A fresh legacy pair alone cannot restore required authority.
        legacy = _matching_legacy_command(12)
        proof = _matching_legacy_tracking_status(12)
        proof.plan_generation = 2
        mux.on_pure_pursuit_cmd(legacy)
        mux.on_pure_pursuit_tracking_status(proof)
        mux._reset_pure_pursuit_envelope_epoch()
        mux.on_timer()
        assert mux.last_source == "stop"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_u2_required_envelope_authority_works_over_actual_topics():
    """A bound envelope is the only PP motion source for the required path.

    This deliberately does not call a Mux input callback.  It protects the
    deployed topic wiring: a fresh `/clock`, exact plan/constraint, and two
    valid envelope samples must be sufficient to move.  After a real race
    epoch reset, raw legacy PP traffic alone must remain unable to restore
    motion authority.
    """
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "control_rate_hz:=50.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=10.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=10.0",
            "-p",
            "pure_pursuit_envelope_timeout_sec:=10.0",
            "-p",
            "safety_constraint_timeout_sec:=10.0",
            "-p",
            "overtake_plan_timeout_sec:=10.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    peer = Node("hybrid_control_mux_u2_actual_topic_peer")
    commands = []
    statuses = []
    peer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    peer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    clock_pub = peer.create_publisher(Clock, "/clock", 10)
    plan_pub = peer.create_publisher(OvertakePlan, "input/overtake_plan", 10)
    constraint_pub = peer.create_publisher(
        SafetyConstraint, "input/safety_constraint", 10
    )
    envelope_pub = peer.create_publisher(
        ControllerCommandEnvelope, "input/pure_pursuit_command_envelope", 10
    )
    legacy_command_pub = peer.create_publisher(
        AckermannControlCommand, "input/pure_pursuit_control_cmd", 10
    )
    legacy_status_pub = peer.create_publisher(
        ControllerTrackingStatus, "input/pure_pursuit_tracking_status", 10
    )
    race_armed_pub = peer.create_publisher(Bool, "input/race_armed", 10)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(peer)

    def publish_clock(stamp_sec: int) -> None:
        msg = Clock()
        _set_stamp(msg.clock, stamp_sec)
        clock_pub.publish(msg)

    def publish_until(predicate, publish, timeout_sec: float = 0.75) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            publish()
            executor.spin_once(timeout_sec=0.01)
            if predicate():
                return True
        return bool(predicate())

    try:
        # Establish simulator time before the validator receives stamped input.
        moved = publish_until(
            lambda: mux.get_clock().now().nanoseconds >= 100_000_000_000,
            lambda: publish_clock(100),
        )
        assert moved, {
            "active_producer": mux.pure_pursuit_envelope_active_producer_instance_id,
            "active_sequence": mux.pure_pursuit_envelope_active_sequence,
            "envelope_last_valid": mux.pure_pursuit_envelope_last_valid,
            "envelope_last_reason": mux.pure_pursuit_envelope_last_reason,
            "envelope_fault": mux.pure_pursuit_envelope_fault_reason,
            "plan_generation": mux.overtake_plan_generation,
            "last_source": mux.last_source,
        }

        plan = _valid_plan(stamp_sec=100, generation=9)
        constraint = _constraint(
            stamp_sec=100,
            constraint_generation=9,
            plan_generation=9,
        )
        first = _command_envelope(
            stamp_sec=100, plan_generation=9, command_sequence=1
        )
        second = _command_envelope(
            stamp_sec=100, plan_generation=9, command_sequence=2
        )

        # QoS depth is one at the Mux input.  Model two actual 50 Hz producer
        # ticks, rather than batch-publishing both records before the Mux has
        # dispatched the first callback; the latter intentionally drops the
        # older sample and cannot establish a consecutive producer bind.
        assert publish_until(
            lambda: (
                mux.overtake_plan_generation == 9
                and mux.safety_constraint is not None
                and mux.safety_constraint.plan_generation == 9
            ),
            lambda: (
                publish_clock(100),
                plan_pub.publish(plan),
                constraint_pub.publish(constraint),
            ),
        )
        assert publish_until(
            lambda: (
                mux.pure_pursuit_envelope_candidate_instance_id == 17
                and mux.pure_pursuit_envelope_candidate_sequence == 1
                and mux.pure_pursuit_envelope_candidate_valid_count == 1
            ),
            lambda: (
                publish_clock(100),
                envelope_pub.publish(first),
            ),
        )

        # No legacy command/status publisher is used on this motion path.
        moved = publish_until(
            lambda: any(
                msg.longitudinal.speed > 0.0
                and msg.lateral.steering_tire_angle == pytest.approx(0.1)
                for msg in commands
            )
            and any(
                msg.plan_generation == 9
                and msg.trajectory_tracking_usable
                and msg.reason == "ready"
                for msg in statuses
            )
            and mux.last_source == "pure_pursuit",
            lambda: (
                publish_clock(100),
                plan_pub.publish(plan),
                constraint_pub.publish(constraint),
                envelope_pub.publish(second),
            ),
        )
        assert moved, {
            "active_producer": mux.pure_pursuit_envelope_active_producer_instance_id,
            "active_sequence": mux.pure_pursuit_envelope_active_sequence,
            "envelope_last_valid": mux.pure_pursuit_envelope_last_valid,
            "envelope_last_reason": mux.pure_pursuit_envelope_last_reason,
            "envelope_fault": mux.pure_pursuit_envelope_fault_reason,
            "envelope_cache": tuple(mux.pure_pursuit_envelope_cache),
            "plan_generation": mux.overtake_plan_generation,
            "last_source": mux.last_source,
        }
        assert mux.pure_pursuit_cmd is None

        # Only a genuine false->true race arm transition clears an active U2
        # producer.  On the new epoch, publish plan/constraint and raw legacy
        # PP traffic but no envelope; required authority must stay STOP.
        reset_false = Bool(data=False)
        reset_true = Bool(data=True)
        legacy_plan = _valid_plan(stamp_sec=101, generation=10)
        legacy_constraint = _constraint(
            stamp_sec=101,
            constraint_generation=10,
            plan_generation=10,
        )
        legacy_command = _matching_legacy_command(101)
        legacy_status = _matching_legacy_tracking_status(101)
        legacy_status.plan_generation = 10
        command_count_before_legacy = len(commands)
        status_count_before_legacy = len(statuses)

        assert publish_until(
            lambda: mux.pure_pursuit_envelope_active_producer_instance_id is None,
            lambda: (
                publish_clock(101),
                race_armed_pub.publish(reset_false),
                race_armed_pub.publish(reset_true),
            ),
        )
        legacy_stopped = publish_until(
            lambda: any(
                msg.longitudinal.speed == pytest.approx(0.0)
                and msg.lateral.steering_tire_angle == pytest.approx(0.0)
                for msg in commands[command_count_before_legacy:]
            )
            and any(
                not msg.trajectory_tracking_usable
                and msg.reason == "final_source_not_pure_pursuit"
                for msg in statuses[status_count_before_legacy:]
            )
            and mux.last_source == "stop",
            lambda: (
                publish_clock(101),
                plan_pub.publish(legacy_plan),
                constraint_pub.publish(legacy_constraint),
                legacy_command_pub.publish(legacy_command),
                legacy_status_pub.publish(legacy_status),
            ),
        )
        assert legacy_stopped
    finally:
        executor.remove_node(peer)
        executor.remove_node(mux)
        peer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def _bind_envelope_producer_for_selector(
    mux: HybridControlMuxNode, *, previous_generation: int
) -> None:
    """Bind the U2 producer using two fresh N-1 records for selector tests."""
    mux.on_pure_pursuit_command_envelope(
        _command_envelope(
            stamp_sec=100,
            plan_generation=previous_generation,
            command_sequence=1,
        )
    )
    mux.on_pure_pursuit_command_envelope(
        _command_envelope(
            stamp_sec=100,
            plan_generation=previous_generation,
            command_sequence=2,
        )
    )
    assert mux.pure_pursuit_envelope_active_producer_instance_id == 17


def test_u2_selector_current_generation_invalid_or_stale_blocks_fresh_n_minus_1():
    """A received but unusable current record is a hard negative for N-1."""
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 100)
        _bind_envelope_producer_for_selector(mux, previous_generation=8)

        invalid_current = _command_envelope(
            stamp_sec=100,
            plan_generation=9,
            command_sequence=3,
            trajectory_tracking_usable=False,
        )
        mux.on_pure_pursuit_command_envelope(invalid_current)
        assert mux._select_bound_envelope_pp_motion_sample(
            tracking_plan_generation=9,
            now_sec=mux.now_sec(),
            ros_clock_stalled=False,
        ) is None

        mux._reset_pure_pursuit_envelope_epoch()
        _bind_envelope_producer_for_selector(mux, previous_generation=8)
        current = _command_envelope(
            stamp_sec=100,
            plan_generation=9,
            command_sequence=3,
        )
        mux.on_pure_pursuit_command_envelope(current)
        current_identity = mux._pure_pursuit_envelope_identity(current)
        fingerprint, _, valid, reason, immutable_current = (
            mux.pure_pursuit_envelope_cache[current_identity]
        )
        mux.pure_pursuit_envelope_cache[current_identity] = (
            fingerprint,
            mux.now_sec() - mux.pure_pursuit_envelope_timeout_sec - 0.01,
            valid,
            reason,
            immutable_current,
        )
        assert mux._select_bound_envelope_pp_motion_sample(
            tracking_plan_generation=9,
            now_sec=mux.now_sec(),
            ros_clock_stalled=False,
        ) is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_u2_selector_missing_current_generation_may_bridge_fresh_n_minus_1():
    """Only a wholly missing current record may use bounded N-1 continuity."""
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 100)
        _bind_envelope_producer_for_selector(mux, previous_generation=8)

        sample = mux._select_bound_envelope_pp_motion_sample(
            tracking_plan_generation=9,
            now_sec=mux.now_sec(),
            ros_clock_stalled=False,
        )
        assert sample is not None
        assert sample.authority_proof is not None
        assert sample.authority_proof.plan_generation == 8
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_u2_selector_evicted_current_hard_negative_cannot_reopen_n_minus_1():
    """Cache eviction must retain denial provenance instead of reopening N-1."""
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 100)
        _bind_envelope_producer_for_selector(mux, previous_generation=98)
        mux.on_overtake_plan(_valid_plan(stamp_sec=100, generation=100))

        current_negative = _command_envelope(
            stamp_sec=100,
            plan_generation=100,
            command_sequence=3,
            trajectory_tracking_usable=False,
        )
        mux.on_pure_pursuit_command_envelope(current_negative)
        # Put a fresh target N-1 record behind the hard-negative, then add
        # exactly enough distinct identities to evict the three older records
        # (bind sequence 1/2 and the current-generation negative) from the
        # bounded eight-entry cache while retaining N-1.
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(
                stamp_sec=100, plan_generation=99, command_sequence=4
            )
        )
        for sequence in range(5, 12):
            mux.on_pure_pursuit_command_envelope(
                _command_envelope(
                    stamp_sec=100,
                    plan_generation=100 + sequence,
                    command_sequence=sequence,
                )
            )

        assert len(mux.pure_pursuit_envelope_cache) == 8
        assert all(
            int(identity[3]) != 100
            for identity in mux.pure_pursuit_envelope_cache
        )
        assert any(
            int(identity[3]) == 99
            for identity in mux.pure_pursuit_envelope_cache
        )
        assert mux.pure_pursuit_envelope_watermark is not None
        assert mux.pure_pursuit_envelope_watermark[0][3] == 100
        assert mux._select_bound_envelope_pp_motion_sample(
            tracking_plan_generation=100,
            now_sec=mux.now_sec(),
            ros_clock_stalled=False,
        ) is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_u2_watermark_promotes_cached_current_generation_and_clears_on_plan_change():
    """Plan transitions select one current envelope record or clear the pin."""
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 100)
        _bind_envelope_producer_for_selector(mux, previous_generation=98)
        mux.on_overtake_plan(_valid_plan(stamp_sec=100, generation=100))
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(
                stamp_sec=100, plan_generation=101, command_sequence=3
            )
        )
        assert mux.pure_pursuit_envelope_watermark is None

        mux.on_overtake_plan(_valid_plan(stamp_sec=101, generation=101))
        assert mux.pure_pursuit_envelope_watermark is not None
        assert mux.pure_pursuit_envelope_watermark[0][3] == 101
        assert mux.pure_pursuit_envelope_watermark[0][1] == 3

        mux.on_overtake_plan(_valid_plan(stamp_sec=102, generation=102))
        assert mux.pure_pursuit_envelope_watermark is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_u2_watermark_rejects_older_sequence_and_clears_on_race_epoch_reset():
    """Only newer active sequences replace the pin; a new race clears it."""
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 100)
        _bind_envelope_producer_for_selector(mux, previous_generation=99)
        mux.on_overtake_plan(_valid_plan(stamp_sec=100, generation=100))
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(
                stamp_sec=100, plan_generation=100, command_sequence=4
            )
        )
        assert mux.pure_pursuit_envelope_watermark is not None
        assert mux.pure_pursuit_envelope_watermark[0][1] == 4

        mux.on_pure_pursuit_command_envelope(
            _command_envelope(
                stamp_sec=100, plan_generation=100, command_sequence=3
            )
        )
        assert mux.pure_pursuit_envelope_watermark is not None
        assert mux.pure_pursuit_envelope_watermark[0][1] == 4

        mux.on_race_armed(Bool(data=False))
        mux.on_race_armed(Bool(data=True))
        assert mux.pure_pursuit_envelope_watermark is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_binds_after_two_valid_samples_without_authority_change():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        legacy_status = ControllerTrackingStatus()
        _set_stamp(legacy_status.header.stamp, 10)
        legacy_status.header.frame_id = "base_link"
        legacy_status.plan_generation = 9
        legacy_status.mpc_horizon_usable = True
        legacy_status.pp_command_fresh = True
        legacy_status.trajectory_tracking_usable = True
        legacy_status.command_age_sec = 0.0
        legacy_status.reason = "ready"
        mux.on_pure_pursuit_tracking_status(legacy_status)
        _set_mux_ros_time_for_envelope(mux, 10)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=10, command_sequence=1)
        )
        assert mux.pure_pursuit_envelope_last_parity_valid is False

        legacy_command = AckermannControlCommand()
        _set_stamp(legacy_command.stamp, 10)
        _set_stamp(legacy_command.longitudinal.stamp, 10)
        legacy_command.longitudinal.speed = 1.5
        legacy_command.longitudinal.acceleration = -0.2
        legacy_command.longitudinal.jerk = 0.0
        _set_stamp(legacy_command.lateral.stamp, 10)
        legacy_command.lateral.steering_tire_angle = 0.1
        legacy_command.lateral.steering_tire_rotation_rate = 0.0
        # PP publishes status -> envelope -> command. The final callback must
        # refresh parity rather than leaving the envelope-receipt result stale.
        mux.on_pure_pursuit_cmd(legacy_command)
        assert mux.pure_pursuit_cmd is legacy_command
        assert mux.pure_pursuit_envelope_last_parity_valid is True
        assert mux.pure_pursuit_envelope_active_producer_instance_id is None
        _set_mux_ros_time_for_envelope(mux, 11)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=11, command_sequence=2)
        )

        assert mux.pure_pursuit_envelope_active_producer_instance_id == 17
        assert mux.pure_pursuit_cmd is legacy_command
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("mutate", "expected_reason"),
    [
        (lambda envelope: setattr(envelope, "schema_version", 3), "schema_version"),
        (lambda envelope: setattr(envelope, "producer_instance_id", 0), "producer_instance_zero"),
        (lambda envelope: setattr(envelope, "command_sequence", 0), "sequence_zero"),
        (lambda envelope: setattr(envelope, "plan_generation", 0), "plan_generation_zero"),
        (lambda envelope: setattr(envelope.header, "frame_id", "map"), "frame_id"),
        (
            lambda envelope: setattr(
                envelope.command.lateral.stamp, "sec", envelope.header.stamp.sec + 1
            ),
            "stamp_mismatch",
        ),
        (
            lambda envelope: setattr(
                envelope.command.stamp, "sec", envelope.header.stamp.sec + 1
            ),
            "stamp_mismatch",
        ),
        (
            lambda envelope: setattr(
                envelope.command.longitudinal.stamp,
                "sec",
                envelope.header.stamp.sec + 1,
            ),
            "stamp_mismatch",
        ),
        (
            lambda envelope: setattr(envelope.command.longitudinal, "jerk", 0.1),
            "v1_jerk_nonzero",
        ),
        (
            lambda envelope: setattr(
                envelope.command.lateral, "steering_tire_rotation_rate", 0.1
            ),
            "v1_steering_rate_nonzero",
        ),
        (lambda envelope: setattr(envelope, "command_age_sec", 1.0), "command_age"),
        (lambda envelope: setattr(envelope, "pp_command_fresh", False), "pp_command_not_fresh"),
        (
            lambda envelope: setattr(envelope, "trajectory_tracking_usable", False),
            "trajectory_tracking_unusable",
        ),
    ],
)
def test_command_envelope_shadow_rejects_invalid_v1_contracts(
    mutate, expected_reason
):
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 20)
        envelope = _command_envelope(stamp_sec=20)
        mutate(envelope)
        mux.on_pure_pursuit_command_envelope(envelope)

        assert mux.pure_pursuit_envelope_last_valid is False
        assert mux.pure_pursuit_envelope_last_reason == expected_reason
        assert mux.pure_pursuit_envelope_active_producer_instance_id is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_rejects_stale_header_even_when_reported_age_is_zero():
    """Mux-computed ROS age, not the publisher claim, owns stale rejection."""
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "pure_pursuit_envelope_timeout_sec:=1.0",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        _set_mux_ros_time_for_envelope(mux, 12)
        envelope = _command_envelope(
            stamp_sec=10,
            command_sequence=1,
            command_age_sec=0.0,
        )

        valid, reason = mux._validate_pure_pursuit_envelope(envelope)

        assert valid is False
        assert reason == "header_stamp_stale"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_latches_conflict_and_sequence_regression():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 30)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=30, command_sequence=1)
        )
        _set_mux_ros_time_for_envelope(mux, 31)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=31, command_sequence=2)
        )
        assert mux.pure_pursuit_envelope_active_producer_instance_id == 17

        conflict = _command_envelope(stamp_sec=31, command_sequence=2)
        conflict.command.longitudinal.speed = 2.0
        mux.on_pure_pursuit_command_envelope(conflict)
        assert mux.pure_pursuit_envelope_fault_latched is True
        assert mux.pure_pursuit_envelope_fault_reason == "duplicate_conflict"

        mux._reset_pure_pursuit_envelope_epoch()
        _set_mux_ros_time_for_envelope(mux, 40)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=40, command_sequence=2)
        )
        _set_mux_ros_time_for_envelope(mux, 41)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=41, command_sequence=3)
        )
        _set_mux_ros_time_for_envelope(mux, 42)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=42, command_sequence=2)
        )
        assert mux.pure_pursuit_envelope_fault_latched is True
        assert mux.pure_pursuit_envelope_fault_reason == "sequence_regression"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_invalid_breaks_unbound_candidate_but_not_active_producer():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        _set_mux_ros_time_for_envelope(mux, 35)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=35, command_sequence=1)
        )
        assert mux.pure_pursuit_envelope_candidate_valid_count == 1
        invalid = _command_envelope(stamp_sec=36, command_sequence=2)
        invalid.trajectory_tracking_usable = False
        _set_mux_ros_time_for_envelope(mux, 36)
        mux.on_pure_pursuit_command_envelope(invalid)
        assert mux.pure_pursuit_envelope_candidate_valid_count == 0
        _set_mux_ros_time_for_envelope(mux, 37)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=37, command_sequence=3)
        )
        assert mux.pure_pursuit_envelope_active_producer_instance_id is None
        assert mux.pure_pursuit_envelope_candidate_valid_count == 1
        _set_mux_ros_time_for_envelope(mux, 38)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=38, command_sequence=4)
        )
        assert mux.pure_pursuit_envelope_active_producer_instance_id == 17

        active_invalid = _command_envelope(stamp_sec=39, command_sequence=5)
        active_invalid.trajectory_tracking_usable = False
        _set_mux_ros_time_for_envelope(mux, 39)
        mux.on_pure_pursuit_command_envelope(active_invalid)
        assert mux.pure_pursuit_envelope_active_producer_instance_id == 17
        assert mux.pure_pursuit_envelope_active_sequence == 4
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_clock_zero_requires_zero_stamp():
    rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
    mux = HybridControlMuxNode()
    try:
        assert mux.get_clock().now().nanoseconds == 0
        valid, reason = mux._validate_pure_pursuit_envelope(
            _command_envelope(stamp_sec=0)
        )
        assert valid is True
        assert reason == "valid"
        valid, reason = mux._validate_pure_pursuit_envelope(
            _command_envelope(stamp_sec=1)
        )
        assert valid is False
        assert reason == "future_stamp_clock_zero"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_parity_requires_base_link_status_frame():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        mux.on_pure_pursuit_cmd(_matching_legacy_command(43))
        status = _matching_legacy_tracking_status(43)
        status.header.frame_id = "map"
        mux.on_pure_pursuit_tracking_status(status)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=43, command_sequence=1)
        )

        assert mux.pure_pursuit_envelope_last_parity_valid is False
        assert mux.pure_pursuit_envelope_last_parity_reason == "legacy_status_payload"
        assert mux.pure_pursuit_cmd is not None
        assert mux.pure_pursuit_tracking_status is status
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("envelope_case", ["none", "valid", "malformed", "faulted"])
def test_command_envelope_shadow_never_changes_legacy_timer_authority(envelope_case):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=false",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )

    def run_once(case: str):
        mux = HybridControlMuxNode()
        mux.timer.cancel()
        observer = Node(f"hybrid_control_mux_envelope_authority_{case}")
        commands = []
        statuses = []
        observer.create_subscription(
            AckermannControlCommand,
            "output/control_cmd",
            lambda msg: commands.append(msg),
            10,
        )
        observer.create_subscription(
            ControllerTrackingStatus,
            "output/controller_tracking_status",
            lambda msg: statuses.append(msg),
            10,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(mux)
        executor.add_node(observer)
        try:
            mux.on_pure_pursuit_tracking_status(_matching_legacy_tracking_status(120))
            mux.on_pure_pursuit_cmd(_matching_legacy_command(120))
            if case == "valid":
                mux.on_pure_pursuit_command_envelope(
                    _command_envelope(stamp_sec=120, command_sequence=1)
                )
            elif case == "malformed":
                malformed = _command_envelope(stamp_sec=120, command_sequence=1)
                malformed.schema_version = 2
                mux.on_pure_pursuit_command_envelope(malformed)
            elif case == "faulted":
                first = _command_envelope(stamp_sec=120, command_sequence=1)
                second = _command_envelope(stamp_sec=121, command_sequence=2)
                conflict = _command_envelope(stamp_sec=121, command_sequence=2)
                conflict.command.longitudinal.speed = 2.0
                mux.on_pure_pursuit_command_envelope(first)
                mux.on_pure_pursuit_command_envelope(second)
                mux.on_pure_pursuit_command_envelope(conflict)

            mux.on_timer()
            assert _spin_until(
                executor, lambda: bool(commands) and bool(statuses)
            )
            command = commands[-1]
            status = statuses[-1]
            return (
                mux.last_source,
                (
                    command.longitudinal.speed,
                    command.longitudinal.acceleration,
                    command.longitudinal.jerk,
                    command.lateral.steering_tire_angle,
                    command.lateral.steering_tire_rotation_rate,
                ),
                (
                    status.plan_generation,
                    status.mpc_horizon_usable,
                    status.pp_command_fresh,
                    status.trajectory_tracking_usable,
                    status.safety_constraint_release_ready,
                    status.attack_follow_stop_transport_release_ready,
                    status.reason,
                ),
            )
        finally:
            executor.remove_node(observer)
            executor.remove_node(mux)
            observer.destroy_node()
            mux.destroy_node()

    try:
        baseline = run_once("none")
        observed = baseline if envelope_case == "none" else run_once(envelope_case)
        assert observed == baseline
    finally:
        rclpy.shutdown()


def test_command_envelope_shadow_duplicate_restores_its_first_receipt_time():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        first = _command_envelope(stamp_sec=45, command_sequence=1)
        second = _command_envelope(stamp_sec=46, command_sequence=2)
        mux.on_pure_pursuit_command_envelope(first)
        first_identity = mux.pure_pursuit_envelope_last_identity
        assert first_identity is not None
        mux.on_pure_pursuit_command_envelope(second)
        newer_receipt_time_sec = mux.pure_pursuit_envelope_last_receipt_time_sec
        old_receipt_time_sec = (
            mux.now_sec() - mux.pure_pursuit_envelope_timeout_sec - 1.0e-3
        )
        fingerprint, _, valid, reason, cached_message = (
            mux.pure_pursuit_envelope_cache[first_identity]
        )
        mux.pure_pursuit_envelope_cache[first_identity] = (
            fingerprint,
            old_receipt_time_sec,
            valid,
            reason,
            cached_message,
        )

        mux.on_pure_pursuit_command_envelope(first)

        assert mux.pure_pursuit_envelope_last_identity == first_identity
        assert mux.pure_pursuit_envelope_last_receipt_time_sec == pytest.approx(
            old_receipt_time_sec
        )
        assert (
            mux.pure_pursuit_envelope_last_receipt_time_sec
            < newer_receipt_time_sec - 0.1
        )
        assert mux._pure_pursuit_envelope_receipt_fresh(mux.now_sec()) is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_faults_second_producer_until_race_epoch_reset():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        for sequence, stamp_sec in ((1, 50), (2, 51)):
            _set_mux_ros_time_for_envelope(mux, stamp_sec)
            mux.on_pure_pursuit_command_envelope(
                _command_envelope(stamp_sec=stamp_sec, command_sequence=sequence)
            )
        _set_mux_ros_time_for_envelope(mux, 52)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(
                stamp_sec=52,
                producer_instance_id=18,
                command_sequence=1,
            )
        )
        assert mux.pure_pursuit_envelope_fault_reason == "active_producer_conflict"

        mux.on_race_armed(Bool(data=True))
        assert mux.pure_pursuit_envelope_fault_latched is False
        assert mux.pure_pursuit_envelope_active_producer_instance_id is None
        for sequence, stamp_sec in ((1, 53), (2, 54)):
            _set_mux_ros_time_for_envelope(mux, stamp_sec)
            mux.on_pure_pursuit_command_envelope(
                _command_envelope(
                    stamp_sec=stamp_sec,
                    producer_instance_id=18,
                    command_sequence=sequence,
                )
            )
        assert mux.pure_pursuit_envelope_active_producer_instance_id == 18
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_bounds_cache_and_rejects_stale_stop_without_reason_parity():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        for sequence in range(1, 10):
            mux.on_pure_pursuit_command_envelope(
                _command_envelope(stamp_sec=60 + sequence, command_sequence=sequence)
            )
        assert len(mux.pure_pursuit_envelope_cache) == 8

        stale_status = ControllerTrackingStatus()
        _set_stamp(stale_status.header.stamp, 80)
        stale_status.header.frame_id = "base_link"
        stale_status.plan_generation = 0
        stale_status.pp_command_fresh = False
        stale_status.trajectory_tracking_usable = False
        stale_status.command_age_sec = float("inf")
        stale_status.reason = "legacy_stale_reason"
        mux.on_pure_pursuit_tracking_status(stale_status)
        stale_envelope = _command_envelope(
            stamp_sec=80,
            command_sequence=10,
            plan_generation=0,
            command_age_sec=float("inf"),
            trajectory_tracking_usable=False,
        )
        stale_envelope.pp_command_fresh = False
        stale_envelope.reason = "envelope_stale_reason"
        mux.on_pure_pursuit_command_envelope(stale_envelope)

        assert stale_status.plan_generation == 0
        assert stale_status.trajectory_tracking_usable is False
        assert stale_envelope.plan_generation == 0
        assert stale_envelope.trajectory_tracking_usable is False
        assert mux.pure_pursuit_envelope_last_valid is False
        assert mux.pure_pursuit_envelope_last_parity_valid is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_max_sequence_is_valid_but_stale_or_clock_stalled_is_unusable():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        maximum_sequence = (2**64) - 1
        _set_mux_ros_time_for_envelope(mux, 90)
        valid, reason = mux._validate_pure_pursuit_envelope(
            _command_envelope(stamp_sec=90, command_sequence=maximum_sequence)
        )
        assert valid is True
        assert reason == "valid"
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=90, command_sequence=maximum_sequence)
        )
        _set_mux_ros_time_for_envelope(mux, 91)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=91, command_sequence=0)
        )
        assert mux.pure_pursuit_envelope_fault_latched is True
        assert mux.pure_pursuit_envelope_fault_reason == "sequence_zero"

        mux._reset_pure_pursuit_envelope_epoch()

        for sequence, stamp_sec in ((1, 92), (2, 93)):
            _set_mux_ros_time_for_envelope(mux, stamp_sec)
            mux.on_pure_pursuit_tracking_status(
                _matching_legacy_tracking_status(stamp_sec)
            )
            mux.on_pure_pursuit_command_envelope(
                _command_envelope(stamp_sec=stamp_sec, command_sequence=sequence)
            )
            mux.on_pure_pursuit_cmd(_matching_legacy_command(stamp_sec))
        now_sec = mux.now_sec()
        assert mux._pure_pursuit_envelope_shadow_usable(now_sec, False) is True
        mux.pure_pursuit_envelope_last_receipt_time_sec = (
            now_sec - mux.pure_pursuit_envelope_timeout_sec - 1.0e-3
        )
        assert mux._pure_pursuit_envelope_receipt_fresh(now_sec) is False
        assert mux._pure_pursuit_envelope_shadow_usable(now_sec, False) is False
        assert mux._pure_pursuit_envelope_shadow_usable(now_sec, True) is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_command_envelope_shadow_rejects_future_stamp_and_latches_stamp_regression():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        # Integer-second truncation must not make this land inside the
        # production future-stamp tolerance near a second boundary.
        future_sec = int(mux.get_clock().now().nanoseconds // 1_000_000_000) + 2
        future = _command_envelope(stamp_sec=future_sec)
        mux.on_pure_pursuit_command_envelope(future)
        assert mux.pure_pursuit_envelope_last_valid is False
        assert mux.pure_pursuit_envelope_last_reason == "future_stamp"

        mux._reset_pure_pursuit_envelope_epoch()
        _set_mux_ros_time_for_envelope(mux, 100)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=100, command_sequence=1)
        )
        _set_mux_ros_time_for_envelope(mux, 101)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=101, command_sequence=2)
        )
        _set_mux_ros_time_for_envelope(mux, 100)
        mux.on_pure_pursuit_command_envelope(
            _command_envelope(stamp_sec=100, command_sequence=3)
        )
        assert mux.pure_pursuit_envelope_fault_latched is True
        assert mux.pure_pursuit_envelope_fault_reason == "stamp_regression"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_safety_authority_rendezvous_bridges_only_release_transport_gap():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_rendezvous_gap_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_overtake_plan(_valid_plan(stamp_sec=10, generation=10))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
            )
        )
        now_sec = mux.now_sec()
        selection = mux._select_safety_authority_contract(
            now_sec, tracking_plan_generation=10
        )
        constraint, receipt_time_sec, plan_generation = selection
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.EXACT_CURRENT
        )
        decision = mux.safety_authority.evaluate(
            constraint,
            received_time_sec=receipt_time_sec,
            now_sec=now_sec,
            active_plan_generation=plan_generation,
        )
        assert decision.stop_required is False
        assert plan_generation == 10

        # Prime one final-published, exact FREE_RUN steering command for N.
        # A later N -> N+1 plan/constraint delivery gap may retain only this
        # verified angle while longitudinal authority remains STOP.
        verified_command = AckermannControlCommand()
        _set_stamp(verified_command.stamp, 19)
        verified_command.longitudinal.speed = 2.0
        verified_command.longitudinal.acceleration = 0.4
        verified_command.lateral.steering_tire_angle = -0.21
        mux.on_pure_pursuit_cmd(verified_command)
        verified_proof = ControllerTrackingStatus()
        _set_stamp(verified_proof.header.stamp, 19)
        verified_proof.header.frame_id = "base_link"
        verified_proof.plan_generation = 10
        verified_proof.pp_command_fresh = True
        verified_proof.trajectory_tracking_usable = True
        verified_proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(verified_proof)
        _bind_envelope_from_legacy(
            mux, verified_command, verified_proof, sequence=19
        )
        mux.on_timer()
        assert _spin_until(
            executor, lambda: len(commands) >= 1 and len(statuses) >= 1
        )
        assert commands[-1].longitudinal.speed == pytest.approx(2.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(-0.21)
        assert statuses[-1].trajectory_tracking_usable is True
        assert mux.last_verified_baseline_free_run_steering_rad == pytest.approx(
            -0.21
        )

        # If plan N+1 arrives before its constraint peer, the historical pair
        # remains cached but is marked unpaired. on_timer must convert that
        # historical non-STOP decision to a transient STOP.  The current PP
        # command is not authority; only the previously final-published angle
        # may bridge this one bounded delivery gap.
        mux.on_overtake_plan(_valid_plan(stamp_sec=11, generation=11))
        now_sec = mux.now_sec()
        selection = mux._select_safety_authority_contract(
            now_sec, tracking_plan_generation=11
        )
        constraint, receipt_time_sec, plan_generation = selection
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
        )
        decision = mux.safety_authority.evaluate(
            constraint,
            received_time_sec=receipt_time_sec,
            now_sec=now_sec,
            active_plan_generation=plan_generation,
        )
        assert decision.stop_required is False
        assert plan_generation == 10
        assert constraint.plan_generation == 10
        assert mux.safety_authority_contract_unpaired is True
        gap_stop_decision = SafetyConstraintDecision(
            stop_required=True,
            speed_limit_mps=0.0,
            required_brake_decel_mps2=0.0,
            constraint_generation=10,
            plan_generation=10,
            reason="safety_authority_contract_unpaired",
        )
        assert mux._normal_delivery_gap_baseline_hold_candidate(
            selection=selection,
            tracking_plan_generation=11,
            decision=gap_stop_decision,
            now_sec=mux.now_sec(),
        )

        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        command.longitudinal.speed = 2.0
        command.longitudinal.acceleration = 0.4
        command.lateral.steering_tire_angle = -0.3
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 11
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=21)
        command_count = len(commands)
        status_count = len(statuses)
        mux.on_timer()
        assert mux.last_source == "pure_pursuit"
        assert mux.safety_authority._fault_latched is False
        assert _spin_until(
            executor,
            lambda: len(commands) > command_count
            and len(statuses) > status_count,
        )
        assert commands[-1].longitudinal.speed == 0.0
        assert commands[-1].longitudinal.acceleration <= 0.0
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(-0.21)
        assert statuses[-1].trajectory_tracking_usable is False
        assert (
            statuses[-1].reason
            == "normal_delivery_gap_zero_speed_steering_hold"
        )
        assert mux.motion_authority_grant_active is False
        assert (
            statuses[-1].pass_probe_transport_evidence
            == ControllerTrackingStatus.PASS_PROBE_TRANSPORT_NORMAL_DELIVERY_GAP
        )

        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
                speed_limit_mps=4.0,
            )
        )
        now_sec = mux.now_sec()
        selection = mux._select_safety_authority_contract(
            now_sec, tracking_plan_generation=11
        )
        constraint, receipt_time_sec, plan_generation = selection
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.EXACT_CURRENT
        )
        decision = mux.safety_authority.evaluate(
            constraint,
            received_time_sec=receipt_time_sec,
            now_sec=now_sec,
            active_plan_generation=plan_generation,
        )
        assert decision.stop_required is False
        assert plan_generation == 11

        # A supervisory barrier always revokes the preserved angle
        # synchronously; the gap carveout must never survive E-stop.
        external_stop = SafetyStopStatus()
        external_stop.valid = False
        external_stop.stop_requested = True
        mux.on_external_safety_status(external_stop)
        command_count = len(commands)
        status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > command_count and len(statuses) > status_count,
        )
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert mux.last_verified_baseline_free_run_steering_rad is None
        assert mux.motion_authority_grant_active is False

        # The reverse delivery order is also a normal gap when the new
        # constraint is a non-STOP relaxation exactly one generation ahead.
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=12,
                constraint_generation=12,
                plan_generation=12,
                speed_limit_mps=5.0,
            )
        )
        selection = mux._select_safety_authority_contract(
            mux.now_sec(), tracking_plan_generation=11
        )
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
        )
        assert mux.safety_authority_contract_unpaired is True
        mux.on_overtake_plan(_valid_plan(stamp_sec=12, generation=12))
        selection = mux._select_safety_authority_contract(
            mux.now_sec(), tracking_plan_generation=12
        )
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.EXACT_CURRENT
        )

        # A tighter cap is safe to apply before its plan peer arrives; STOP is
        # likewise never delayed by the rendezvous cache.
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=13,
                constraint_generation=13,
                plan_generation=13,
                speed_limit_mps=2.0,
            )
        )
        now_sec = mux.now_sec()
        constraint, _, plan_generation = mux._select_safety_authority_contract(
            now_sec, tracking_plan_generation=12
        )
        assert plan_generation == 13
        assert constraint.speed_limit_mps == 2.0

        mux.on_safety_constraint(
            _constraint(
                stamp_sec=14,
                constraint_generation=14,
                plan_generation=14,
                speed_limit_mps=2.0,
                stop_requested=True,
            )
        )
        now_sec = mux.now_sec()
        constraint, receipt_time_sec, plan_generation = (
            mux._select_safety_authority_contract(
                now_sec, tracking_plan_generation=12
            )
        )
        decision = mux.safety_authority.evaluate(
            constraint,
            received_time_sec=receipt_time_sec,
            now_sec=now_sec,
            active_plan_generation=plan_generation,
        )
        assert plan_generation == 14
        assert decision.stop_required is True
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_exact_authority_pp_n_minus_1_gap_holds_only_primed_free_run_steering():
    """A direct N-1 PP proof pauses steering authority without losing angle."""
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_pp_generation_gap_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        # No historical final command exists at startup, so a delivery gap
        # must remain a zero-steer STOP rather than manufacture a reference.
        assert mux.last_verified_baseline_free_run_steering_rad is None

        plan_n = _valid_plan(stamp_sec=10, generation=10)
        mux.on_overtake_plan(plan_n)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
            )
        )
        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        _set_stamp(command.longitudinal.stamp, 20)
        command.longitudinal.speed = 2.0
        command.longitudinal.acceleration = 0.4
        _set_stamp(command.lateral.stamp, 20)
        command.lateral.steering_tire_angle = -0.21
        command.lateral.steering_tire_rotation_rate = 0.0
        mux.on_pure_pursuit_cmd(command)

        def proof_for(generation: int, *, bind_command: bool):
            proof = ControllerTrackingStatus()
            _set_stamp(proof.header.stamp, 20)
            proof.header.frame_id = "base_link"
            proof.plan_generation = generation
            proof.pp_command_fresh = True
            proof.trajectory_tracking_usable = True
            proof.command_age_sec = 0.0
            if bind_command:
                proof.pp_command_binding_valid = True
                proof.pp_command_speed_mps = 2.0
                proof.pp_command_acceleration_mps2 = 0.4
                proof.pp_command_steering_tire_angle_rad = -0.21
            return proof

        # Exact N publishes the only steering that can be reused later.
        proof_n = proof_for(10, bind_command=False)
        mux.on_pure_pursuit_tracking_status(proof_n)
        _bind_envelope_from_legacy(mux, command, proof_n, sequence=20)
        mux.on_timer()
        assert _spin_until(
            executor, lambda: len(commands) >= 1 and len(statuses) >= 1
        )
        assert commands[-1].longitudinal.speed == pytest.approx(2.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(-0.21)
        assert statuses[-1].trajectory_tracking_usable is True
        assert mux.last_verified_baseline_free_run_steering_rad == pytest.approx(
            -0.21
        )

        # Plan/constraint N+1 are exact, but PP still proves N for the same
        # command stamp. It is a bounded pause, never a motion grant.
        mux.on_overtake_plan(_valid_plan(stamp_sec=11, generation=11))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
            )
        )
        selection = mux._select_safety_authority_contract(
            mux.now_sec(), tracking_plan_generation=11
        )
        assert selection.rendezvous_state == SafetyAuthorityRendezvousState.EXACT_CURRENT
        current_decision = mux.safety_authority.evaluate(
            selection.constraint,
            received_time_sec=selection.receipt_time_sec,
            now_sec=mux.now_sec(),
            active_plan_generation=selection.authority_plan_generation,
        )
        assert current_decision.stop_required is False
        assert mux._normal_delivery_gap_baseline_hold_candidate(
            selection=selection,
            tracking_plan_generation=11,
            decision=current_decision,
            now_sec=mux.now_sec(),
            pp_tracking_delivery_gap_usable=True,
        )
        command_count = len(commands)
        status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > command_count and len(statuses) > status_count,
        )
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= 0.0
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(-0.21)
        assert commands[-1].lateral.steering_tire_rotation_rate == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].reason == "normal_delivery_gap_zero_speed_steering_hold"
        assert mux.motion_authority_grant_active is False

        # Only the exact N+1 proof resumes normal PP authority. Because both
        # generations share a command stamp, N+1 must bind the command values.
        proof_n_plus_1 = proof_for(11, bind_command=True)
        mux.on_pure_pursuit_tracking_status(proof_n_plus_1)
        _bind_envelope_from_legacy(
            mux, command, proof_n_plus_1, sequence=22
        )
        command_count = len(commands)
        status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > command_count and len(statuses) > status_count,
        )
        assert commands[-1].longitudinal.speed == pytest.approx(2.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(-0.21)
        assert statuses[-1].trajectory_tracking_usable is True
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_dev3_135724_unprimed_n63_n64_gap_remains_zero_steer_stop():
    """Replay D1's first 20260727-135724 N-1/N seam without a verified record.

    The bag had authority generation 64 while PP still reported 63, with
    ``last_verified_tracking_steering_rad=null``.  A normal delivery gap may
    preserve an already final-published immutable record, but it must not
    invent one at startup or reinterpret stale ACK/source-key evidence.
    """
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("dev3_135724_unprimed_gap_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_overtake_plan(_valid_plan(stamp_sec=64, generation=64))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=64,
                constraint_generation=64,
                plan_generation=64,
            )
        )
        command = AckermannControlCommand()
        _set_stamp(command.stamp, 63)
        _set_stamp(command.longitudinal.stamp, 63)
        command.longitudinal.speed = 2.0
        command.longitudinal.acceleration = 0.4
        _set_stamp(command.lateral.stamp, 63)
        command.lateral.steering_tire_angle = -0.21
        mux.on_pure_pursuit_cmd(command)
        proof_n_minus_1 = ControllerTrackingStatus()
        _set_stamp(proof_n_minus_1.header.stamp, 63)
        proof_n_minus_1.header.frame_id = "base_link"
        proof_n_minus_1.plan_generation = 63
        proof_n_minus_1.pp_command_fresh = True
        proof_n_minus_1.trajectory_tracking_usable = True
        proof_n_minus_1.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof_n_minus_1)
        _bind_envelope_from_legacy(
            mux, command, proof_n_minus_1, sequence=63
        )

        assert mux.last_verified_baseline_free_run_steering_rad is None
        mux.on_timer()
        assert _spin_until(
            executor, lambda: bool(commands) and bool(statuses)
        )
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= 0.0
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert mux.last_verified_baseline_free_run_steering_rad is None
        assert mux.motion_authority_grant_active is False
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_prearm_exact_tuple_cannot_grant_motion():
    """An exact executable FREE_RUN tuple is still powerless before race arm."""
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "race_arm_required:=true",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("prearm_exact_tuple_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        generation = 71
        mux.on_overtake_plan(
            _valid_plan(stamp_sec=generation, generation=generation)
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=generation,
                constraint_generation=generation,
                plan_generation=generation,
                speed_limit_mps=3.0,
                stop_requested=False,
                release_authorized=True,
            )
        )
        command = AckermannControlCommand()
        _set_stamp(command.stamp, generation)
        _set_stamp(command.longitudinal.stamp, generation)
        command.longitudinal.speed = 2.0
        command.longitudinal.acceleration = 0.4
        _set_stamp(command.lateral.stamp, generation)
        command.lateral.steering_tire_angle = 0.24
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, generation)
        proof.header.frame_id = "base_link"
        proof.plan_generation = generation
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=generation)

        mux.on_timer()
        assert _spin_until(
            executor, lambda: bool(commands) and bool(statuses)
        )
        assert mux.finish_stop_latch.armed is False
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= 0.0
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert mux.motion_authority_grant_active is False
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_dev3_d1_n43_n44_unpaired_gap_preserves_exact_free_run_record():
    """Replay the 20260727-070846 D1 N43/N44 transport ordering.

    Plan N+1 may arrive before constraint N+1, followed by the exact
    plan/constraint N+1 pair while PP still reports N.  Neither delivery seam
    grants motion, but both must preserve the immutable, final-published N
    steering record until the exact N+1 PP tuple arrives.
    """
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "free_run_live_exact_pre_ack_hold_enabled:=true",
            "-p",
            "race_arm_required:=false",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=10.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=10.0",
            "-p",
            "pure_pursuit_envelope_timeout_sec:=10.0",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "max_steering_angle_rad:=0.64",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.64",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("dev3_d1_n43_n44_gap_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        plan_n = _valid_plan(stamp_sec=43, generation=43)
        plan_n.race_arm_epoch = 1
        plan_n.planner_instance_id = 101
        ack_n = _free_run_execution_ack(stamp_sec=43, generation=43)
        ack_n.hard_steering_tire_angle_limit_rad = 0.64
        ack_n = _finalize_free_run_ack(mux, plan_n, ack_n)
        mux.on_overtake_plan(plan_n)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=43,
                constraint_generation=43,
                plan_generation=43,
            )
        )
        mux.on_free_run_source_key(_source_key_from_ack(ack_n))
        mux.on_free_run_execution_ack(ack_n)
        mux.on_pure_pursuit_cmd(copy.deepcopy(ack_n.output_controller_command))
        proof_n = ControllerTrackingStatus()
        proof_n.header = copy.deepcopy(ack_n.header)
        proof_n.plan_generation = 43
        proof_n.mpc_horizon_usable = True
        proof_n.pp_command_fresh = True
        proof_n.trajectory_tracking_usable = True
        proof_n.command_age_sec = 0.0
        proof_n.reason = "ready"
        mux.on_pure_pursuit_tracking_status(proof_n)
        _set_mux_ros_time_for_envelope(mux, 43)
        for sequence in (6, 7):
            envelope = _command_envelope(
                stamp_sec=43,
                producer_instance_id=201,
                command_sequence=sequence,
                plan_generation=43,
            )
            envelope.command = copy.deepcopy(ack_n.output_controller_command)
            mux.on_pure_pursuit_command_envelope(envelope)
        mux.on_timer()
        assert _spin_until(
            executor, lambda: bool(commands) and bool(statuses)
        )
        assert commands[-1].longitudinal.speed > 0.0
        assert statuses[-1].trajectory_tracking_usable is True
        record_n = mux.free_run_live_exact_published_record
        assert record_n is not None, (
            mux.free_run_live_exact_record_reason,
            mux.free_run_live_exact_ack_reason,
            mux.free_run_live_exact_selected_envelope_reason,
            mux.free_run_live_exact_source_key_reason,
        )

        # Real bag order: plan 44 is visible for one callback before its
        # constraint peer.  The current implementation used to invalidate the
        # exact N43 record at this unpaired STOP barrier.
        plan_n_plus_1 = _valid_plan(stamp_sec=43, generation=44)
        plan_n_plus_1.header.stamp.nanosec = 50_000_000
        plan_n_plus_1.race_arm_epoch = 1
        plan_n_plus_1.planner_instance_id = 101
        _bind_free_run_plan_identity(mux, plan_n_plus_1)
        mux.on_overtake_plan(plan_n_plus_1)
        gap_selection = mux._select_safety_authority_contract(
            mux.now_sec(), tracking_plan_generation=44
        )
        gap_decision = mux.safety_authority.evaluate(
            gap_selection.constraint,
            received_time_sec=gap_selection.receipt_time_sec,
            now_sec=mux.now_sec(),
            active_plan_generation=gap_selection.authority_plan_generation,
        )
        gap_stop_decision = SafetyConstraintDecision(
            stop_required=True,
            speed_limit_mps=0.0,
            required_brake_decel_mps2=0.0,
            constraint_generation=int(gap_decision.constraint_generation),
            plan_generation=int(gap_decision.plan_generation),
            reason="safety_authority_contract_unpaired",
        )
        assert mux._normal_delivery_gap_baseline_hold_candidate(
            selection=gap_selection,
            tracking_plan_generation=44,
            decision=gap_stop_decision,
            now_sec=mux.now_sec(),
        )
        assert mux._free_run_record_supports_normal_delivery_gap_hold(
            tracking_plan_generation=44,
            now_sec=mux.now_sec(),
        )
        command_count = len(commands)
        status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > command_count
            and len(statuses) > status_count,
        )
        assert mux.free_run_live_exact_published_record is record_n, (
            mux.free_run_live_exact_record_reason,
            mux.free_run_live_exact_selected_envelope_reason,
            mux.free_run_live_exact_forward_transition_reason,
            mux.last_verified_baseline_free_run_steering_rad,
        )
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(
            record_n.published_steering_rad
        )
        assert statuses[-1].trajectory_tracking_usable is False
        assert (
            statuses[-1].reason
            == "normal_delivery_gap_zero_speed_steering_hold"
        )
        assert mux.motion_authority_grant_active is False
        assert mux.free_run_live_exact_published_record is record_n

        # Constraint 44 closes the authority pair, but PP envelope/tracking 43
        # still cannot move the vehicle.  It may only retain the same record.
        constraint_n_plus_1 = _constraint(
            stamp_sec=43,
            constraint_generation=44,
            plan_generation=44,
        )
        constraint_n_plus_1.header.stamp.nanosec = 50_000_000
        mux.on_safety_constraint(constraint_n_plus_1)
        command_count = len(commands)
        status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > command_count
            and len(statuses) > status_count,
        )
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(
            record_n.published_steering_rad
        )
        assert statuses[-1].trajectory_tracking_usable is False
        assert (
            statuses[-1].reason
            == "normal_delivery_gap_zero_speed_steering_hold"
        )
        assert mux.motion_authority_grant_active is False
        assert mux.free_run_live_exact_published_record is record_n

        # Only the exact N44 ACK/envelope/tracking tuple may atomically replace
        # N43 and restore positive longitudinal authority.
        ack_n_plus_1 = _free_run_execution_ack(
            stamp_sec=44, generation=44
        )
        ack_n_plus_1.hard_steering_tire_angle_limit_rad = 0.64
        ack_n_plus_1.source_key = copy.deepcopy(ack_n.source_key)
        ack_n_plus_1.controller_sequence = 9
        ack_n_plus_1.plan_key.plan_stamp = copy.deepcopy(
            plan_n_plus_1.header.stamp
        )
        ack_n_plus_1.base_geometry.source_stamp = copy.deepcopy(
            ack_n.source_key.source_stamp
        )
        ack_n_plus_1.applied_geometry.source_stamp = copy.deepcopy(
            ack_n.source_key.source_stamp
        )
        ack_n_plus_1 = _finalize_free_run_ack(
            mux, plan_n_plus_1, ack_n_plus_1
        )
        mux.on_free_run_execution_ack(ack_n_plus_1)
        mux.on_pure_pursuit_cmd(
            copy.deepcopy(ack_n_plus_1.output_controller_command)
        )
        proof_n_plus_1 = copy.deepcopy(proof_n)
        proof_n_plus_1.header = copy.deepcopy(ack_n_plus_1.header)
        proof_n_plus_1.plan_generation = 44
        proof_n_plus_1.pp_command_binding_valid = True
        proof_n_plus_1.pp_command_speed_mps = float(
            ack_n_plus_1.output_controller_command.longitudinal.speed
        )
        proof_n_plus_1.pp_command_acceleration_mps2 = float(
            ack_n_plus_1.output_controller_command.longitudinal.acceleration
        )
        proof_n_plus_1.pp_command_steering_tire_angle_rad = float(
            ack_n_plus_1.output_controller_command.lateral.steering_tire_angle
        )
        mux.on_pure_pursuit_tracking_status(proof_n_plus_1)
        _set_mux_ros_time_for_envelope(mux, 44)
        for sequence in (8, 9):
            envelope = _command_envelope(
                stamp_sec=44,
                producer_instance_id=201,
                command_sequence=sequence,
                plan_generation=44,
            )
            envelope.command = copy.deepcopy(
                ack_n_plus_1.output_controller_command
            )
            mux.on_pure_pursuit_command_envelope(envelope)
        command_count = len(commands)
        status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > command_count
            and len(statuses) > status_count,
        )
        assert commands[-1].longitudinal.speed > 0.0
        assert statuses[-1].trajectory_tracking_usable is True
        record_n_plus_1 = mux.free_run_live_exact_published_record
        assert record_n_plus_1 is not None
        assert record_n_plus_1 is not record_n
        assert int(record_n_plus_1.plan_generation) == 44
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_safety_authority_normal_delivery_gap_from_startup_constraint_stops_without_fault_latch():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    observer = Node("hybrid_control_mux_startup_normal_gap_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        # At startup a constraint can arrive before the first plan.  When the
        # first observed plan is N+1, the valid non-STOP constraint N is a
        # transport gap, not an authority-contract fault.
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
            )
        )
        mux.on_overtake_plan(_valid_plan(stamp_sec=11, generation=11))
        now_sec = mux.now_sec()
        selection = mux._select_safety_authority_contract(
            now_sec, tracking_plan_generation=11
        )
        constraint, receipt_time_sec, authority_plan_generation = selection
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.NORMAL_DELIVERY_GAP
        )
        assert constraint is not None
        assert authority_plan_generation == constraint.plan_generation == 10
        decision = mux.safety_authority.evaluate(
            constraint,
            received_time_sec=receipt_time_sec,
            now_sec=now_sec,
            active_plan_generation=authority_plan_generation,
        )
        assert decision.stop_required is False
        assert mux.safety_authority._fault_latched is False

        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        command.longitudinal.speed = 2.0
        command.longitudinal.acceleration = 0.4
        command.lateral.steering_tire_angle = -0.3
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 11
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)

        mux.on_timer()
        assert mux.last_source == "stop"
        assert mux.safety_authority._fault_latched is False
        assert _spin_until(executor, lambda: commands and statuses)
        assert commands[-1].longitudinal.speed == 0.0
        assert commands[-1].longitudinal.acceleration <= 0.0
        assert commands[-1].lateral.steering_tire_angle == 0.0
        assert statuses[-1].trajectory_tracking_usable is False
        assert (
            statuses[-1].pass_probe_transport_evidence
            == ControllerTrackingStatus.PASS_PROBE_TRANSPORT_NORMAL_DELIVERY_GAP
        )

        # The same PP proof cannot release motion by itself.  Only the exact
        # N+1 constraint peer transitions authority back to normal evaluation.
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
            )
        )
        now_sec = mux.now_sec()
        selection = mux._select_safety_authority_contract(
            now_sec, tracking_plan_generation=11
        )
        constraint, receipt_time_sec, authority_plan_generation = selection
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.EXACT_CURRENT
        )
        assert authority_plan_generation == constraint.plan_generation == 11
        decision = mux.safety_authority.evaluate(
            constraint,
            received_time_sec=receipt_time_sec,
            now_sec=now_sec,
            active_plan_generation=authority_plan_generation,
        )
        assert decision.stop_required is False
        assert mux.safety_authority._fault_latched is False
        mux.on_timer()
        assert mux.safety_authority._fault_latched is False
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "rejection_case",
    (
        "expired",
        "n_plus_2",
        "passing",
        "attack_follow",
        "abort_hold",
        "target_changed",
        "lateral_required",
        "stale_plan",
    ),
)
def test_normal_delivery_gap_baseline_hold_rejects_non_nominal_boundaries(
    rejection_case,
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "race_arm_required:=false",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        mux.on_overtake_plan(_valid_plan(stamp_sec=10, generation=10))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
            )
        )
        mux.last_verified_baseline_free_run_steering_rad = -0.21
        mux.last_verified_baseline_free_run_steering_time_sec = mux.now_sec()
        mux.last_verified_baseline_free_run_plan_generation = 10
        mux.last_verified_baseline_free_run_plan_stamp_ns = 10_000_000_000
        mux.last_verified_baseline_free_run_epoch = (
            mux.finish_stop_latch.epoch
        )

        generation = 12 if rejection_case == "n_plus_2" else 11
        current_plan = _valid_plan(stamp_sec=11, generation=generation)
        if rejection_case == "passing":
            current_plan = _authorized_passing_plan(
                stamp_sec=11, generation=generation
            )
        elif rejection_case == "attack_follow":
            current_plan = _authorized_attack_follow_plan(
                stamp_sec=11, generation=generation
            )
        elif rejection_case == "abort_hold":
            current_plan = _baseline_stop_plan(
                stamp_sec=11, generation=generation
            )
        elif rejection_case == "target_changed":
            current_plan.target_vehicle_id = "grid_d2"
        elif rejection_case == "lateral_required":
            current_plan = _authorized_lateral_stop_plan(
                stamp_sec=11, generation=generation
            )
        mux.on_overtake_plan(current_plan)
        if rejection_case == "expired":
            mux.last_verified_baseline_free_run_steering_time_sec = (
                mux.now_sec()
                - mux.lateral_stop_steering_hold_timeout_sec
                - 0.01
            )
        elif rejection_case == "stale_plan":
            entry = list(mux.overtake_plan_cache[generation])
            entry[1] = mux.now_sec() - mux.overtake_plan_timeout_sec - 0.01
            mux.overtake_plan_cache[generation] = tuple(entry)

        selection = mux._select_safety_authority_contract(
            mux.now_sec(), tracking_plan_generation=generation
        )
        decision = SafetyConstraintDecision(
            stop_required=True,
            speed_limit_mps=0.0,
            required_brake_decel_mps2=0.0,
            constraint_generation=10,
            plan_generation=10,
            reason="safety_authority_contract_unpaired",
        )
        assert not mux._normal_delivery_gap_baseline_hold_candidate(
            selection=selection,
            tracking_plan_generation=generation,
            decision=decision,
            now_sec=mux.now_sec(),
        )
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_safety_authority_rendezvous_keeps_exact_pair_across_same_generation_stamp_skew(
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
        ]
    )
    mux = HybridControlMuxNode()
    try:
        # generation 10/stamp 10 is a complete, non-stop authority pair.
        mux.on_overtake_plan(_valid_plan(stamp_sec=10, generation=10))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
            )
        )

        # A newer same-generation stamp must not erase historical cache, but it
        # also must not authorize motion. A still newer generation without its
        # exact peer leaves the selector in unpaired STOP state.
        mux.on_overtake_plan(_valid_plan(stamp_sec=11, generation=10))
        mux.on_overtake_plan(_valid_plan(stamp_sec=12, generation=12))

        now_sec = mux.now_sec()
        constraint, receipt_time_sec, plan_generation = (
            mux._select_safety_authority_contract(
                now_sec, tracking_plan_generation=12
            )
        )
        assert constraint is not None
        assert constraint.plan_generation == 10
        assert constraint.header_stamp_ns == 10 * 1_000_000_000
        assert plan_generation == 10
        assert mux.safety_authority_contract_unpaired is True
        decision = mux.safety_authority.evaluate(
            constraint,
            received_time_sec=receipt_time_sec,
            now_sec=now_sec,
            active_plan_generation=plan_generation,
        )
        assert decision.stop_required is False
        assert decision.reason != "safety_constraint_plan_generation_mismatch"
        assert decision.reason != "safety_constraint_fault_latched"

        # Exact PP proof for N+1 is insufficient while the plan/constraint pair
        # is still unpaired. Longitudinal output remains STOP without latching
        # a malformed-contract fault.
        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        command.longitudinal.speed = 2.0
        command.longitudinal.acceleration = 0.4
        command.lateral.steering_tire_angle = -0.3
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 12
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        mux.on_timer()
        assert mux.last_source == "stop"
        assert mux.safety_authority._fault_latched is False

        # Once the matching generation-12 constraint arrives, authority moves
        # atomically to the new exact pair.
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=12,
                constraint_generation=12,
                plan_generation=12,
                speed_limit_mps=4.0,
            )
        )
        now_sec = mux.now_sec()
        constraint, receipt_time_sec, plan_generation = (
            mux._select_safety_authority_contract(
                now_sec, tracking_plan_generation=12
            )
        )
        assert constraint is not None
        assert constraint.plan_generation == 12
        assert plan_generation == 12
        decision = mux.safety_authority.evaluate(
            constraint,
            received_time_sec=receipt_time_sec,
            now_sec=now_sec,
            active_plan_generation=plan_generation,
        )
        assert decision.stop_required is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_safety_authority_rendezvous_does_not_classify_generation_gap_or_regression_as_normal(
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    try:
        mux.on_overtake_plan(_valid_plan(stamp_sec=10, generation=10))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
            )
        )

        mux.overtake_plan_lateral_stop_taint_cache[10] = True
        selection = mux._select_safety_authority_contract(
            mux.now_sec(), tracking_plan_generation=10
        )
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.PAYLOAD_MUTATION
        )
        mux.overtake_plan_lateral_stop_taint_cache.clear()

        # N+2 is a missing generation, not a bounded delivery-order gap.
        mux.on_overtake_plan(_valid_plan(stamp_sec=12, generation=12))
        selection = mux._select_safety_authority_contract(
            mux.now_sec(), tracking_plan_generation=12
        )
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.REGRESSION_OR_GAP
        )

        # Once generation 12 has been observed, generation 11 is a regression
        # even though it is only one away from the latest constraint.
        mux.on_overtake_plan(_valid_plan(stamp_sec=13, generation=11))
        selection = mux._select_safety_authority_contract(
            mux.now_sec(), tracking_plan_generation=11
        )
        assert (
            selection.rendezvous_state
            == SafetyAuthorityRendezvousState.REGRESSION_OR_GAP
        )
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_safety_authority_rendezvous_caches_are_bounded():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        for generation in range(1, 13):
            mux.on_overtake_plan(
                _valid_plan(stamp_sec=generation, generation=generation)
            )
            mux.on_safety_constraint(
                _constraint(
                    stamp_sec=generation,
                    constraint_generation=generation,
                    plan_generation=generation,
                )
            )
        assert len(mux.overtake_plan_cache) == 8
        assert len(mux.overtake_plan_attempt_cache) == 8
        assert len(mux.overtake_plan_trajectory_cache) == 8
        assert len(mux.safety_constraint_cache) == 8
        assert len(mux.overtake_plan_contract_cache) == 8
        assert len(mux.safety_constraint_contract_cache) == 8
        assert min(mux.overtake_plan_cache) == 5
        assert min(mux.overtake_plan_attempt_cache) == 5
        assert min(mux.overtake_plan_trajectory_cache) == 5
        assert min(mux.safety_constraint_cache) == 5

        # Generation wraps after 16_777_215. The newest wrapped generation 1
        # must remain in the rendezvous window; numeric-min eviction used to
        # delete it immediately and force a fail-closed STOP at every wrap.
        for stamp_sec, generation in ((13, 16_777_215), (14, 1)):
            mux.on_overtake_plan(
                _valid_plan(stamp_sec=stamp_sec, generation=generation)
            )
            mux.on_safety_constraint(
                _constraint(
                    stamp_sec=stamp_sec,
                    constraint_generation=generation,
                    plan_generation=generation,
                )
            )
        assert len(mux.overtake_plan_cache) == 8
        assert len(mux.safety_constraint_cache) == 8
        assert 16_777_215 in mux.overtake_plan_cache
        assert 1 in mux.overtake_plan_cache
        assert 16_777_215 in mux.safety_constraint_cache
        assert 1 in mux.safety_constraint_cache
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_new_pp_generation_cannot_move_before_exact_plan_constraint_pair():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    try:
        mux.on_overtake_plan(_valid_plan(stamp_sec=10, generation=10))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
            )
        )

        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        command.longitudinal.speed = 2.0
        command.longitudinal.acceleration = 0.4
        command.lateral.steering_tire_angle = -0.3
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 11
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=1)

        # Raw override/PP generation 11 arrived first. The exact release pair
        # is still generation 10, so the new PP command must not move.
        mux.on_timer()
        assert mux.last_source == "stop"
        assert mux.safety_authority._fault_latched is False

        # Constraint first, then typed plan, matches planner publication order.
        # Motion releases only after the exact generation-11 triple exists.
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
            )
        )
        mux.on_overtake_plan(_valid_plan(stamp_sec=11, generation=11))
        mux.on_timer()
        assert mux.last_source == "pure_pursuit"
        assert mux.safety_authority._fault_latched is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("pass_direction", [-1, 0, 1])
def test_attack_follow_keeps_motion_across_one_generation_transport_gap(
    pass_direction,
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_attack_follow_transport_observer")
    statuses = []
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)

    try:
        mux.on_overtake_plan(
            _authorized_attack_follow_plan(
                stamp_sec=10,
                generation=10,
                pass_direction=pass_direction,
                x_offset_m=0.0,
            )
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
                speed_limit_mps=3.0,
            )
        )

        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        command.longitudinal.speed = 1.0
        command.longitudinal.acceleration = 0.2
        command.lateral.steering_tire_angle = 0.1
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 10
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)

        _bind_envelope_from_legacy(mux, command, proof, sequence=1)

        # Planner generation N arrives while the PP command/status peer still
        # proves N-1.  The same target/attempt/direction and a continuous
        # ATTACK_FOLLOW trajectory may keep moving for this bounded gap.
        mux.on_overtake_plan(
            _authorized_attack_follow_plan(
                stamp_sec=11,
                generation=11,
                pass_direction=pass_direction,
                x_offset_m=0.02,
            )
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
                speed_limit_mps=2.5,
            )
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(statuses))
        assert mux.last_source == "pure_pursuit"
        assert statuses[-1].trajectory_tracking_usable is True
        assert statuses[-1].reason == "attack_follow_transport_continuity"
        assert statuses[-1].plan_generation == 10
        assert statuses[-1].safety_constraint_release_ready is False

        # Once the exact N proof arrives, the ordinary strict path owns the
        # command again and reports ready.
        _set_stamp(command.stamp, 21)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 21)
        proof.plan_generation = 11
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=3)
        previous_count = len(statuses)
        mux.on_timer()
        assert _spin_until(executor, lambda: len(statuses) > previous_count)
        assert mux.last_source == "pure_pursuit"
        assert statuses[-1].trajectory_tracking_usable is True
        assert statuses[-1].reason == "ready"
        assert statuses[-1].plan_generation == 11
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_passing_keeps_motion_across_same_transaction_generation_gap():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_passing_transport_observer")
    statuses = []
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)

    try:
        mux.on_overtake_plan(
            _authorized_passing_plan(stamp_sec=10, generation=10)
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
                speed_limit_mps=3.0,
            )
        )

        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        command.longitudinal.speed = 1.0
        command.longitudinal.acceleration = 0.2
        command.lateral.steering_tire_angle = -0.1
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 10
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=1)

        # 最新Gate2 bagと同じ順序。current plan/constraintはNへ進むが、
        # commandとそのtyped proofだけは同じPASSのN-1を1 control tick追う。
        mux.on_overtake_plan(
            _authorized_passing_plan(
                stamp_sec=11, generation=11, x_offset_m=0.02
            )
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
                speed_limit_mps=2.5,
            )
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(statuses))
        # N-1 continuity remains useful diagnostics, but schema 2 exact-current
        # binding is mandatory for a PASS motion grant.
        assert mux.last_source == "stop"
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].reason == "final_source_not_pure_pursuit"
        assert statuses[-1].plan_generation == 10

        # target、attempt、sideのどれかが変われば、過去PASS proofを流用しない。
        _set_stamp(command.stamp, 21)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 21)
        proof.plan_generation = 11
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=3)
        mux.on_overtake_plan(
            _authorized_passing_plan(
                stamp_sec=12,
                generation=12,
                target_vehicle_id="grid_d3",
                x_offset_m=0.04,
            )
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=12,
                constraint_generation=12,
                plan_generation=12,
                speed_limit_mps=2.5,
            )
        )
        previous_count = len(statuses)
        mux.on_timer()
        assert _spin_until(executor, lambda: len(statuses) > previous_count)
        assert mux.last_source == "stop"
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].reason != "pass_transport_continuity"
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("phase", "expected_reason"),
    [
        (OvertakePlan.ATTACK_FOLLOW, "attack_follow_transport_continuity"),
        (OvertakePlan.PASSING, "pass_transport_continuity"),
    ],
)
@pytest.mark.parametrize("observed_usable", [False, True])
def test_observed_proof_never_changes_n1_maneuver_authority(
    phase, expected_reason, observed_usable
):
    """N-1 maneuver authority is invariant to latest observed proof payload."""
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    observer = Node("hybrid_control_mux_observed_n1_observer")
    statuses = []
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        plan_factory = (
            _authorized_attack_follow_plan
            if phase == OvertakePlan.ATTACK_FOLLOW
            else _authorized_passing_plan
        )
        mux.on_overtake_plan(plan_factory(stamp_sec=10, generation=10))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
                speed_limit_mps=3.0,
            )
        )
        command = _matching_legacy_command(20)
        mux.on_pure_pursuit_cmd(command)
        authority_proof = _matching_legacy_tracking_status(20)
        authority_proof.plan_generation = 10
        mux.on_pure_pursuit_tracking_status(authority_proof)
        _bind_envelope_from_legacy(mux, command, authority_proof, sequence=1)

        # This newer status cannot bind to command stamp 20, so it is observed
        # telemetry only. Its usable bit must never alter N-1 authorization.
        observed_only = _matching_legacy_tracking_status(21)
        observed_only.plan_generation = 99
        observed_only.trajectory_tracking_usable = observed_usable
        observed_only.pp_command_fresh = observed_usable
        mux.on_pure_pursuit_tracking_status(observed_only)

        mux.on_overtake_plan(
            plan_factory(stamp_sec=11, generation=11, x_offset_m=0.02)
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
                speed_limit_mps=2.5,
            )
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(statuses))
        if phase == OvertakePlan.PASSING:
            assert mux.last_source == "stop"
            assert statuses[-1].trajectory_tracking_usable is False
            assert statuses[-1].reason == "final_source_not_pure_pursuit"
        else:
            assert mux.last_source == "pure_pursuit"
            assert statuses[-1].reason == expected_reason
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("observed_usable", [False, True])
def test_observed_proof_never_changes_n1_stop_release_authority(
    observed_usable,
):
    """STOP release bridge also consumes only the selected authority proof."""
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_observed_stop_release_observer")
    statuses = []
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_overtake_plan(
            _authorized_attack_follow_plan(
                stamp_sec=10, generation=10, x_offset_m=0.0
            )
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
                speed_limit_mps=0.2,
                stop_requested=True,
                release_authorized=False,
                reason="maneuver_transaction_tracking_stop",
            )
        )
        command = _matching_legacy_command(20)
        command.longitudinal.speed = 0.2
        mux.on_pure_pursuit_cmd(command)
        authority_proof = _matching_legacy_tracking_status(20)
        authority_proof.plan_generation = 10
        mux.on_pure_pursuit_tracking_status(authority_proof)
        _bind_envelope_from_legacy(mux, command, authority_proof, sequence=1)
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(statuses))

        observed_only = _matching_legacy_tracking_status(21)
        observed_only.plan_generation = 99
        observed_only.pp_command_fresh = observed_usable
        observed_only.trajectory_tracking_usable = observed_usable
        mux.on_pure_pursuit_tracking_status(observed_only)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
                speed_limit_mps=0.2,
                stop_requested=True,
                release_authorized=False,
                reason="release_pending_safe_cycles",
            )
        )
        mux.on_overtake_plan(
            _authorized_attack_follow_plan(
                stamp_sec=11, generation=11, x_offset_m=0.02
            )
        )
        previous_count = len(statuses)
        mux.on_timer()
        assert _spin_until(executor, lambda: len(statuses) > previous_count)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is True
        assert statuses[-1].attack_follow_stop_transport_release_ready is True
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_attack_follow_stop_transport_keeps_release_probe_without_motion():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_attack_follow_stop_transport_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)

    def spin_publications(previous_count: int) -> None:
        assert _spin_until(
            executor,
            lambda: len(statuses) > previous_count and bool(commands),
        )

    try:
        plan = _authorized_attack_follow_plan(
            stamp_sec=10, generation=10, x_offset_m=0.0
        )
        plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_CURRENT_D_HOLD
        )
        plan.lateral_stop_transaction_pass_direction = -1
        plan.lateral_stop_authority_token = (1 << 32) | 10
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
                speed_limit_mps=0.2,
                stop_requested=True,
                release_authorized=False,
                reason="maneuver_transaction_tracking_stop",
            )
        )

        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        command.longitudinal.speed = 0.2
        command.longitudinal.acceleration = 0.1
        command.lateral.steering_tire_angle = 0.1
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 10
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.lateral_stop_authority_kind = plan.lateral_stop_authority_kind
        proof.lateral_stop_transaction_pass_direction = (
            plan.lateral_stop_transaction_pass_direction
        )
        proof.lateral_stop_authority_token = plan.lateral_stop_authority_token
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=1)

        mux.on_timer()
        spin_publications(0)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert statuses[-1].safety_constraint_release_ready is True
        assert (
            statuses[-1].attack_follow_stop_transport_release_ready is False
        )

        # Plannerの実publish順はconstraint -> typed plan。新constraintだけが
        # 先着したtickでは、古いplanへ組み合わせず縦STOPを維持する。
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=11,
                plan_generation=11,
                speed_limit_mps=0.2,
                stop_requested=True,
                release_authorized=False,
                reason="release_pending_safe_cycles",
            )
        )
        previous_count = len(statuses)
        mux.on_timer()
        spin_publications(previous_count)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert statuses[-1].safety_constraint_release_ready is False
        assert mux.safety_authority._fault_latched is False

        # 続いて同stampのtyped planが届いてもPP command/statusはまだN-1。
        # 同じtarget/attempt/directionのATTACK_FOLLOWだけは、走行を許可せず
        # release proofだけを1世代搬送して循環STOPを切る。
        next_plan = _authorized_attack_follow_plan(
            stamp_sec=11, generation=11, x_offset_m=0.02
        )
        next_plan.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_CURRENT_D_HOLD
        )
        next_plan.lateral_stop_transaction_pass_direction = -1
        next_plan.lateral_stop_authority_token = (1 << 32) | 11
        mux.on_overtake_plan(next_plan)
        previous_count = len(statuses)
        mux.on_timer()
        spin_publications(previous_count)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= mux.stop_decel_mps2
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is True
        assert statuses[-1].attack_follow_stop_transport_release_ready is True
        assert statuses[-1].plan_generation == 10
        assert (
            statuses[-1].reason
            == "attack_follow_stop_transport_continuity"
        )
        assert mux.safety_authority._fault_latched is False

        # PP実装はstatus -> commandの順にpublishする。status Nだけが先着した
        # tickも古いcommandを新proofへ結び付けず、縦STOPを維持する。
        next_proof = ControllerTrackingStatus()
        _set_stamp(next_proof.header.stamp, 21)
        next_proof.header.frame_id = "base_link"
        next_proof.plan_generation = 11
        next_proof.pp_command_fresh = True
        next_proof.trajectory_tracking_usable = True
        next_proof.lateral_stop_authority_kind = (
            next_plan.lateral_stop_authority_kind
        )
        next_proof.lateral_stop_transaction_pass_direction = (
            next_plan.lateral_stop_transaction_pass_direction
        )
        next_proof.lateral_stop_authority_token = (
            next_plan.lateral_stop_authority_token
        )
        next_proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(next_proof)
        previous_count = len(statuses)
        mux.on_timer()
        spin_publications(previous_count)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].attack_follow_stop_transport_release_ready is True

        # 同stamp commandが揃った後は通常のexact proofへ戻る。constraintが
        # STOPなので、この時点でも走行は開始しない。
        _set_stamp(command.stamp, 21)
        mux.on_pure_pursuit_cmd(command)
        _bind_envelope_from_legacy(mux, command, next_proof, sequence=3)
        previous_count = len(statuses)
        mux.on_timer()
        spin_publications(previous_count)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is True
        assert (
            statuses[-1].attack_follow_stop_transport_release_ready is False
        )
        assert statuses[-1].plan_generation == 11
        assert (
            statuses[-1].reason
            != "attack_follow_stop_transport_continuity"
        )

        # Identity changes cannot reuse the prior STOP proof.
        mux.on_overtake_plan(
            _authorized_attack_follow_plan(
                stamp_sec=12,
                generation=12,
                target_vehicle_id="grid_d3",
                x_offset_m=0.04,
            )
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=12,
                constraint_generation=12,
                plan_generation=12,
                speed_limit_mps=0.2,
                stop_requested=True,
                release_authorized=False,
                reason="release_pending_safe_cycles",
            )
        )
        previous_count = len(statuses)
        mux.on_timer()
        spin_publications(previous_count)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert statuses[-1].safety_constraint_release_ready is False
        assert (
            statuses[-1].attack_follow_stop_transport_release_ready is False
        )
        assert (
            statuses[-1].reason
            != "attack_follow_stop_transport_continuity"
        )
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_attack_follow_stop_transport_rejects_over_limit_steering_proof():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
            "-p",
            "max_steering_angle_rad:=0.6",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.5",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_attack_follow_limit_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )

    mux.on_overtake_plan(
        _authorized_attack_follow_plan(
            stamp_sec=10, generation=10, x_offset_m=0.0
        )
    )
    mux.on_safety_constraint(
        _constraint(
            stamp_sec=10,
            constraint_generation=10,
            plan_generation=10,
            speed_limit_mps=0.2,
            stop_requested=True,
            release_authorized=False,
            reason="maneuver_transaction_tracking_stop",
        )
    )
    command = AckermannControlCommand()
    _set_stamp(command.stamp, 20)
    command.longitudinal.speed = 0.2
    command.longitudinal.acceleration = 0.1
    command.lateral.steering_tire_angle = 0.500001
    mux.on_pure_pursuit_cmd(command)
    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 20)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 10
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)
    _bind_envelope_from_legacy(mux, command, proof, sequence=1)

    mux.on_safety_constraint(
        _constraint(
            stamp_sec=11,
            constraint_generation=11,
            plan_generation=11,
            speed_limit_mps=0.2,
            stop_requested=True,
            release_authorized=False,
            reason="release_pending_safe_cycles",
        )
    )
    mux.on_overtake_plan(
        _authorized_attack_follow_plan(
            stamp_sec=11, generation=11, x_offset_m=0.02
        )
    )

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(commands) and bool(statuses))
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is False
        assert (
            statuses[-1].attack_follow_stop_transport_release_ready is False
        )
        assert statuses[-1].reason == "steering_command_exceeds_actuator_limit"
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    (
        "next_target",
        "next_attempt",
        "next_phase",
        "next_direction",
        "next_generation",
        "next_x_offset_m",
    ),
    [
        ("grid_d3", 1, OvertakePlan.ATTACK_FOLLOW, -1, 11, 0.02),
        ("grid_d2", 2, OvertakePlan.ATTACK_FOLLOW, -1, 11, 0.02),
        ("grid_d2", 1, OvertakePlan.PASSING, -1, 11, 0.02),
        ("grid_d2", 1, OvertakePlan.ATTACK_FOLLOW, -1, 11, 0.30),
        ("grid_d2", 1, OvertakePlan.ATTACK_FOLLOW, 1, 11, 0.02),
        ("grid_d2", 1, OvertakePlan.ATTACK_FOLLOW, 0, 11, 0.02),
        ("grid_d2", 1, OvertakePlan.ATTACK_FOLLOW, 2, 11, 0.02),
        ("grid_d2", 1, OvertakePlan.ATTACK_FOLLOW, -1, 12, 0.02),
    ],
)
def test_attack_follow_transport_gap_rejects_identity_or_trajectory_change(
    next_target,
    next_attempt,
    next_phase,
    next_direction,
    next_generation,
    next_x_offset_m,
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        mux.on_overtake_plan(
            _authorized_attack_follow_plan(
                stamp_sec=10, generation=10, pass_direction=-1
            )
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=10,
                plan_generation=10,
            )
        )
        next_plan = _authorized_attack_follow_plan(
            stamp_sec=11,
            generation=next_generation,
            attempt_id=next_attempt,
            target_vehicle_id=next_target,
            pass_direction=next_direction,
            x_offset_m=next_x_offset_m,
        )
        next_plan.phase = next_phase
        mux.on_overtake_plan(next_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=next_generation,
                plan_generation=next_generation,
            )
        )

        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        command.longitudinal.speed = 1.0
        command.longitudinal.acceleration = 0.2
        command.lateral.steering_tire_angle = 0.1
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 10
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)

        mux.on_timer()
        assert mux.last_source == "stop"
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_nonadvancing_input_stamps_cannot_overwrite_newer_payloads():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        newer_pp = AckermannControlCommand()
        _set_stamp(newer_pp.stamp, 2)
        newer_pp.longitudinal.speed = 1.0
        mux.on_pure_pursuit_cmd(newer_pp)
        pp_receipt_time = mux.pure_pursuit_cmd_time_sec

        stale_pp = AckermannControlCommand()
        _set_stamp(stale_pp.stamp, 1)
        stale_pp.longitudinal.speed = 9.0
        mux.on_pure_pursuit_cmd(stale_pp)
        same_stamp_pp = AckermannControlCommand()
        _set_stamp(same_stamp_pp.stamp, 2)
        same_stamp_pp.longitudinal.speed = 8.0
        mux.on_pure_pursuit_cmd(same_stamp_pp)
        assert mux.pure_pursuit_cmd.longitudinal.speed == 1.0
        assert mux.pure_pursuit_cmd_time_sec == pp_receipt_time

        newer_mpc = AckermannControlCommand()
        _set_stamp(newer_mpc.stamp, 4)
        newer_mpc.longitudinal.speed = 2.0
        mux.on_mpc_cmd(newer_mpc)
        stale_mpc = AckermannControlCommand()
        _set_stamp(stale_mpc.stamp, 3)
        stale_mpc.longitudinal.speed = 10.0
        mux.on_mpc_cmd(stale_mpc)
        assert mux.mpc_cmd.longitudinal.speed == 2.0

        newer_recovery = RecoveryControlCommand()
        _set_stamp(newer_recovery.header.stamp, 6)
        newer_recovery.command.longitudinal.speed = 0.5
        mux.on_recovery_cmd(newer_recovery)
        stale_recovery = RecoveryControlCommand()
        _set_stamp(stale_recovery.header.stamp, 5)
        stale_recovery.command.longitudinal.speed = 4.0
        mux.on_recovery_cmd(stale_recovery)
        assert mux.recovery_cmd.command.longitudinal.speed == 0.5

        newer_status = RecoveryStatus()
        _set_stamp(newer_status.header.stamp, 8)
        newer_status.state = RecoveryStatus.ACTIVE
        mux.on_recovery_status(newer_status)
        stale_status = RecoveryStatus()
        _set_stamp(stale_status.header.stamp, 7)
        stale_status.state = RecoveryStatus.INACTIVE
        mux.on_recovery_status(stale_status)
        assert mux.recovery_status.state == RecoveryStatus.ACTIVE

        newer_permit = RecoveryPermit()
        _set_stamp(newer_permit.header.stamp, 10)
        newer_permit.recovery_allowed = False
        mux.on_recovery_permit(newer_permit)
        stale_permit = RecoveryPermit()
        _set_stamp(stale_permit.header.stamp, 9)
        stale_permit.recovery_allowed = True
        mux.on_recovery_permit(stale_permit)
        assert mux.recovery_permit.recovery_allowed is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_same_stamp_tracking_proofs_are_keyed_by_plan_generation_and_bound_command():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    try:
        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        _set_stamp(command.longitudinal.stamp, 20)
        command.longitudinal.speed = 1.0
        command.longitudinal.acceleration = 0.2
        command.longitudinal.jerk = 0.0
        _set_stamp(command.lateral.stamp, 20)
        command.lateral.steering_tire_angle = 0.1
        command.lateral.steering_tire_rotation_rate = 0.0
        mux.on_pure_pursuit_cmd(command)

        def proof_for(generation: int, *, steering_rad: float = 0.1):
            proof = ControllerTrackingStatus()
            _set_stamp(proof.header.stamp, 20)
            proof.header.frame_id = "base_link"
            proof.plan_generation = generation
            proof.pp_command_fresh = True
            proof.trajectory_tracking_usable = True
            proof.pp_command_binding_valid = True
            proof.pp_command_speed_mps = 1.0
            proof.pp_command_acceleration_mps2 = 0.2
            proof.pp_command_steering_tire_angle_rad = steering_rad
            proof.command_age_sec = 0.0
            return proof

        mux.on_pure_pursuit_tracking_status(proof_for(11))
        stamp_ns = mux.pure_pursuit_cmd_stamp_ns
        first_receipt_time = mux.pure_pursuit_tracking_status_time_sec
        mux.on_pure_pursuit_tracking_status(proof_for(12))
        # U2 authority is the immutable envelope, while the legacy cache
        # below remains the diagnostic/binding regression surface.
        _bind_envelope_from_legacy(mux, command, proof_for(12), sequence=1)

        assert (stamp_ns, 11) in mux.pure_pursuit_tracking_status_cache
        assert (stamp_ns, 12) in mux.pure_pursuit_tracking_status_cache
        assert mux.pure_pursuit_tracking_status_cache[(stamp_ns, 12)][2] is True
        # A new generation at the same stamp does not refresh timeout age.
        assert mux.pure_pursuit_tracking_status_time_sec == first_receipt_time

        mux.on_overtake_plan(_valid_plan(stamp_sec=10, generation=12))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=12,
                plan_generation=12,
            )
        )
        mux.on_timer()
        assert mux.last_source == "pure_pursuit"

        changed_command = AckermannControlCommand()
        _set_stamp(changed_command.stamp, 20)
        _set_stamp(changed_command.longitudinal.stamp, 20)
        changed_command.longitudinal.speed = 1.0
        changed_command.longitudinal.acceleration = 0.2
        changed_command.longitudinal.jerk = 0.0
        _set_stamp(changed_command.lateral.stamp, 20)
        changed_command.lateral.steering_tire_angle = 0.25
        changed_command.lateral.steering_tire_rotation_rate = 0.0
        mux.on_pure_pursuit_cmd(changed_command)
        mux.on_pure_pursuit_tracking_status(proof_for(13, steering_rad=0.25))

        # AckermannControlCommand has no generation.  A different same-stamp
        # payload therefore cannot be proven by the retained command.
        assert mux.pure_pursuit_tracking_status_cache[(stamp_ns, 13)][2] is False

        duplicate_receipt_time = mux.pure_pursuit_tracking_status_time_sec
        mux.on_pure_pursuit_tracking_status(proof_for(12))
        assert mux.pure_pursuit_tracking_status_time_sec == duplicate_receipt_time

        stale = proof_for(10)
        _set_stamp(stale.header.stamp, 19)
        mux.on_pure_pursuit_tracking_status(stale)
        assert (19_000_000_000, 10) not in mux.pure_pursuit_tracking_status_cache
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("overrides", "expected_blockers"),
    [
        ({}, ["exact_tuple"]),
        (
            {"same_stamp_other_generation_present": True},
            ["generation"],
        ),
        (
            {
                "exact_tuple_present": True,
                "binding_required": True,
                "binding_matches": False,
            },
            ["binding"],
        ),
        (
            {
                "exact_tuple_present": True,
                "exact_tuple_valid": True,
                "status_fresh": False,
            },
            ["freshness"],
        ),
        (
            {
                "exact_tuple_present": True,
                "exact_tuple_valid": True,
                "status_fresh": True,
                "steering_usable": False,
                "steering_blocked": True,
                "exact_proof_usable": True,
            },
            ["steering"],
        ),
        (
            {
                "same_stamp_other_generation_present": True,
                "continuity_applicable": True,
            },
            ["continuity", "generation"],
        ),
        (
            {
                "exact_tuple_present": True,
                "exact_tuple_valid": True,
                "status_fresh": True,
                "exact_proof_usable": True,
            },
            [],
        ),
    ],
)
def test_motion_pp_tracking_proof_diagnostic_classifies_first_false(
    overrides, expected_blockers
):
    inputs = {
        "exact_tuple_present": False,
        "same_stamp_other_generation_present": False,
        "exact_tuple_valid": False,
        "binding_required": False,
        "binding_matches": True,
        "status_fresh": True,
        "pp_command_fresh": True,
        "upstream_pp_command_fresh": True,
        "steering_usable": True,
        "steering_blocked": False,
        "exact_proof_usable": False,
        "continuity_applicable": False,
        "continuity_usable": False,
    }
    inputs.update(overrides)

    blockers = HybridControlMuxNode._motion_pp_tracking_proof_blockers(
        **inputs
    )

    assert blockers == expected_blockers


def test_motion_pp_tracking_failure_forces_exact_tuple_debug_event():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
            "-p",
            "debug_publish_period_sec:=100.0",
        ]
    )
    mux = HybridControlMuxNode()
    mux.now_sec = lambda: 100.0
    observer = Node("hybrid_control_mux_proof_diagnostic_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    commands = []
    statuses = []
    debug = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    observer.create_subscription(
        String,
        "output/debug",
        lambda msg: debug.append(json.loads(msg.data)),
        10,
    )
    try:
        command = AckermannControlCommand()
        _set_stamp(command.stamp, 20)
        _set_stamp(command.longitudinal.stamp, 20)
        command.longitudinal.speed = 1.0
        command.longitudinal.acceleration = 0.2
        command.longitudinal.jerk = 0.0
        _set_stamp(command.lateral.stamp, 20)
        command.lateral.steering_tire_angle = 0.1
        command.lateral.steering_tire_rotation_rate = 0.0
        mux.on_pure_pursuit_cmd(command)

        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, 20)
        proof.header.frame_id = "base_link"
        proof.plan_generation = 12
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        # In U2 the immutable envelope, not the raw legacy PP command, is
        # motion authority.  Keep the legacy proof so the JSON diagnostic
        # remains checked against its established tuple cache as well.
        _bind_envelope_from_legacy(mux, command, proof, sequence=1)
        mux.on_overtake_plan(_valid_plan(stamp_sec=10, generation=12))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=12,
                plan_generation=12,
            )
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(debug) >= 1)
        assert mux.last_source == "pure_pursuit"
        assert debug[-1]["motion_pp_tracking_proof_usable"] is True
        assert debug[-1]["motion_pp_tracking_proof_first_false"] == "none"
        assert debug[-1]["motion_pp_tracking_selected_envelope_producer_instance_id"] == 17
        assert debug[-1]["motion_pp_tracking_selected_envelope_command_sequence"] == 2
        assert debug[-1]["motion_pp_tracking_selected_envelope_receipt_age_sec"] == pytest.approx(0.0)

        next_command = AckermannControlCommand()
        _set_stamp(next_command.stamp, 21)
        _set_stamp(next_command.longitudinal.stamp, 21)
        next_command.longitudinal.speed = 1.0
        next_command.longitudinal.acceleration = 0.2
        next_command.longitudinal.jerk = 0.0
        _set_stamp(next_command.lateral.stamp, 21)
        next_command.lateral.steering_tire_angle = 0.1
        next_command.lateral.steering_tire_rotation_rate = 0.0
        mux.on_pure_pursuit_cmd(next_command)
        next_proof = ControllerTrackingStatus()
        _set_stamp(next_proof.header.stamp, 21)
        next_proof.header.frame_id = "base_link"
        next_proof.plan_generation = 13
        next_proof.pp_command_fresh = True
        next_proof.trajectory_tracking_usable = True
        next_proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(next_proof)
        # Advance the release contract but intentionally omit the matching
        # generation-13 envelope. U2 may observe legacy traffic, but cannot
        # authorize its raw command. The already published generation-12
        # steering may only be held at zero speed while the exact N tuple is
        # pending.
        mux.on_overtake_plan(_valid_plan(stamp_sec=11, generation=13))
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=13,
                plan_generation=13,
            )
        )
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(debug) >= 2
            and len(statuses) >= 2
            and len(commands) >= 2
            and commands[-1].longitudinal.speed == pytest.approx(0.0)
            and statuses[-1].trajectory_tracking_usable is False,
        )

        failure = debug[-1]
        assert failure["source"] == "pure_pursuit"
        assert failure["reason"] == "normal_delivery_gap_zero_speed_steering_hold"
        assert failure["motion_pp_tracking_proof_usable"] is False
        assert failure["motion_pp_tracking_proof_first_false"] == "generation"
        assert failure["motion_pp_tracking_proof_blockers"] == ["generation"]
        # The last bound generation-12 envelope is allowed to be selected as
        # a bounded candidate, but cannot prove the new FREE_RUN generation.
        assert failure["motion_pp_tracking_expected_stamp_ns"] == 20_000_000_000
        assert failure["motion_pp_tracking_expected_generation"] == 13
        assert failure["motion_pp_tracking_observed_stamp_ns"] is None
        assert failure["motion_pp_tracking_observed_generation"] is None
        assert failure["motion_pp_tracking_exact_tuple_present"] is False
        assert failure["source_previous"] == "pure_pursuit"
        assert failure["source_changed"] is False
        assert failure["control_fault_reason"] == ""
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.1)
        assert commands[-1].lateral.steering_tire_rotation_rate == pytest.approx(
            0.0
        )
        assert statuses[-1].trajectory_tracking_usable is False
        assert (
            statuses[-1].reason
            == "normal_delivery_gap_zero_speed_steering_hold"
        )
        assert (
            statuses[-1].pass_probe_transport_evidence
            == ControllerTrackingStatus.PASS_PROBE_TRANSPORT_EXACT_CURRENT
        )

        forced_debug_time = mux.last_forced_motion_pp_tracking_debug_sec
        steering_blocked_command = AckermannControlCommand()
        _set_stamp(steering_blocked_command.stamp, 22)
        _set_stamp(steering_blocked_command.longitudinal.stamp, 22)
        steering_blocked_command.longitudinal.speed = 1.0
        steering_blocked_command.longitudinal.acceleration = 0.2
        steering_blocked_command.longitudinal.jerk = 0.0
        _set_stamp(steering_blocked_command.lateral.stamp, 22)
        steering_blocked_command.lateral.steering_tire_angle = (
            mux.tracking_usable_max_steering_angle_rad + 0.01
        )
        steering_blocked_command.lateral.steering_tire_rotation_rate = 0.0
        mux.on_pure_pursuit_cmd(steering_blocked_command)
        mux.on_timer()
        # A changing failure label cannot bypass the dedicated 10 Hz cap.
        # The pending signature remains different and can be emitted once the
        # cap interval has elapsed.
        assert mux.last_forced_motion_pp_tracking_debug_sec == forced_debug_time
        assert mux.last_motion_pp_tracking_proof_failure_signature == "generation"

        # Bag-derived 20260726-131156 seam: standalone status N arrived while
        # the atomic envelope was still N-1. Once the exact generation-13
        # envelope arrives, authority recovers from N itself without promoting
        # the generation-12 sample or latching the transient delivery gap.
        _bind_envelope_from_legacy(mux, next_command, next_proof, sequence=3)
        mux.last_debug_publish_sec = -1.0e9
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(debug) >= 3
            and len(statuses) >= 3
            and statuses[-1].trajectory_tracking_usable is True,
        )
        recovered = debug[-1]
        assert recovered["source"] == "pure_pursuit"
        assert recovered["source_previous"] == "pure_pursuit"
        assert recovered["source_changed"] is False
        assert recovered["motion_pp_tracking_expected_generation"] == 13
        assert recovered["motion_pp_tracking_observed_generation"] == 13
        assert recovered["motion_pp_tracking_selected_envelope_command_sequence"] == 4
        assert (
            statuses[-1].pass_probe_transport_evidence
            == ControllerTrackingStatus.PASS_PROBE_TRANSPORT_EXACT_CURRENT
        )
        assert mux.safety_authority._fault_latched is False
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_mux_publishes_stop_while_sim_clock_is_frozen():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "control_rate_hz:=50.0",
        ]
    )
    mux = HybridControlMuxNode()
    observer = Node("hybrid_control_mux_steady_timer_test_observer")
    received = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: received.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 0.40
        while time.monotonic() < deadline and len(received) < 3:
            executor.spin_once(timeout_sec=0.02)

        assert len(received) >= 2
        assert mux.race_arm_required is True
        assert mux.finish_stop_latch.armed is False
        assert mux.ros_clock_motion_ready is False
        assert mux.control_fault_latched is False
        assert all(msg.longitudinal.speed == 0.0 for msg in received)
        assert all(msg.longitudinal.acceleration < 0.0 for msg in received)
        assert all(msg.lateral.steering_tire_angle == 0.0 for msg in received)
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_official_finish_latch_stops_only_current_domain_and_preserves_steering():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=false",
            "-p",
            "finish_stop_enabled:=true",
            "-p",
            "finish_stop_steering_guard_trigger_rad:=0.3",
            "-p",
            "max_steering_angle_rad:=0.6",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.35",
            "-p",
            "enable_steering_rate_limit:=false",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_finish_stop_observer")
    received = []
    received_tracking = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: received.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: received_tracking.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        command = AckermannControlCommand()
        _set_stamp(command.stamp, 1)
        command.longitudinal.speed = 6.0
        command.longitudinal.acceleration = 1.0
        command.lateral.steering_tire_angle = 0.34
        mux.on_pure_pursuit_cmd(command)

        # 実bagと同じくReadyがrace_armed=trueよりわずかに先に届く。
        ready = String()
        ready.data = "Ready"
        mux.on_awsim_state(ready)
        armed = Bool()
        armed.data = True
        mux.on_race_armed(armed)
        state = String()
        state.data = "Start"
        mux.on_awsim_state(state)
        # Finish直前の実出力操舵を記録してrejoin級の急舵guardを成立させる。
        previous_count = len(received)
        previous_tracking_count = len(received_tracking)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(received) > previous_count
            and len(received_tracking) > previous_tracking_count,
        )
        assert received[-1].lateral.steering_tire_angle == pytest.approx(0.34)

        # 別subscriptionのcallback順が実bagで逆転した条件を再現する。
        armed.data = False
        mux.on_race_armed(armed)
        state.data = "Finish"
        mux.on_awsim_state(state)
        assert mux.finish_terminal_snapshot_steering_rad == pytest.approx(0.34)

        # Finish後のraw PPはterminal snapshotを更新できない。guardはlatch時の
        # final limited steeringだけをboundedに持つ。
        _set_stamp(command.stamp, 2)
        command.lateral.steering_tire_angle = 0.4
        mux.on_pure_pursuit_cmd(command)

        previous_count = len(received)
        previous_tracking_count = len(received_tracking)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(received) > previous_count
            and len(received_tracking) > previous_tracking_count,
        )
        assert received[-1].longitudinal.speed == 0.0
        assert received[-1].longitudinal.acceleration == pytest.approx(-3.2)
        assert received[-1].lateral.steering_tire_angle == pytest.approx(0.2)
        assert received_tracking[-1].plan_generation == 0
        assert not received_tracking[-1].trajectory_tracking_usable
        assert not received_tracking[-1].safety_constraint_release_ready
        assert not received_tracking[-1].attack_follow_stop_transport_release_ready
        assert received_tracking[-1].reason == "official_vehicle_finish"

        # 実bagと同じFinish直後のdisarmでもterminal停止を解除しない。
        mux.on_race_armed(armed)
        previous_count = len(received)
        previous_tracking_count = len(received_tracking)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(received) > previous_count
            and len(received_tracking) > previous_tracking_count,
        )
        assert mux.finish_stop_latch.latched
        assert received[-1].longitudinal.speed == 0.0
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_finish_stop_command_is_canonical_and_uses_frozen_snapshot_only():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        mux.finish_terminal_snapshot_steering_rad = 0.12
        mux.finish_stop_lateral_guard_active = False

        output = mux._finish_stop_command(mux.now_sec())

        assert output.longitudinal.speed == 0.0
        assert output.longitudinal.acceleration == pytest.approx(-3.2)
        assert output.longitudinal.jerk == 0.0
        assert output.lateral.steering_tire_angle == pytest.approx(0.12)
        assert output.lateral.steering_tire_rotation_rate == 0.0
        assert output.stamp == output.longitudinal.stamp == output.lateral.stamp
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_finish_snapshot_rejects_stale_or_nonfinite_candidate_to_zero_steering():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        mux.finish_stop_latch.epoch = 3
        mux.finish_terminal_candidate_epoch = 3
        mux.finish_terminal_candidate_steering_rad = 0.18
        mux.finish_terminal_candidate_time_sec = (
            mux.now_sec() - mux.finish_terminal_reference_timeout_sec - 0.01
        )
        mux._snapshot_finish_terminal_reference()
        assert mux.finish_terminal_snapshot_steering_rad is None
        assert not mux.finish_stop_lateral_guard_active
        assert mux._finish_stop_command(mux.now_sec()).lateral.steering_tire_angle == 0.0

        mux.finish_terminal_candidate_time_sec = mux.now_sec()
        mux.finish_terminal_candidate_steering_rad = float("nan")
        mux._snapshot_finish_terminal_reference()
        assert mux.finish_terminal_snapshot_steering_rad is None
        assert mux._finish_stop_command(mux.now_sec()).lateral.steering_tire_angle == 0.0
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_finish_stop_applies_only_valid_stronger_constraint_brake():
    rclpy.init(args=["--ros-args", "-p", "finish_stop_decel_mps2:=-1.0"])
    mux = HybridControlMuxNode()
    try:
        output = mux._finish_stop_command(mux.now_sec())
        valid_decision = SafetyConstraintDecision(
            stop_required=True,
            speed_limit_mps=0.5,
            required_brake_decel_mps2=1.5,
            constraint_generation=2,
            plan_generation=2,
            reason="safety_constraint_applied",
        )
        output, source, _ = mux._apply_safety_constraint(
            output,
            "finish_stop",
            "official_vehicle_finish",
            valid_decision,
            mux.now_sec(),
        )
        assert source == "finish_stop"
        assert output.longitudinal.speed == 0.0
        assert output.longitudinal.acceleration == pytest.approx(-1.5)

        invalid_decision = SafetyConstraintDecision(
            stop_required=True,
            speed_limit_mps=0.0,
            required_brake_decel_mps2=9.0,
            constraint_generation=0,
            plan_generation=0,
            reason="safety_constraint_stale",
        )
        output = mux._finish_stop_command(mux.now_sec())
        output, source, _ = mux._apply_safety_constraint(
            output,
            "finish_stop",
            "official_vehicle_finish",
            invalid_decision,
            mux.now_sec(),
        )
        assert source == "finish_stop"
        assert output.longitudinal.acceleration == pytest.approx(-1.0)
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_new_race_epoch_is_the_only_finish_snapshot_reset():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        mux.finish_terminal_snapshot_steering_rad = 0.16
        mux.finish_terminal_snapshot_epoch = 1
        mux.finish_stop_latch.latched = True
        disarmed = Bool()
        disarmed.data = False
        mux.on_race_armed(disarmed)
        assert mux.finish_terminal_snapshot_steering_rad == pytest.approx(0.16)

        armed = Bool()
        armed.data = True
        mux.on_race_armed(armed)
        assert mux.finish_terminal_snapshot_steering_rad is None
        assert mux.finish_terminal_snapshot_epoch is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_external_stop_and_control_fault_override_finish_snapshot():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_finish_priority_observer")
    received = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: received.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.finish_stop_latch.latched = True
        mux.finish_terminal_snapshot_steering_rad = 0.20
        mux.finish_stop_lateral_guard_active = True
        mux.external_stop_latched = True
        previous_count = len(received)
        mux.on_timer()
        assert _spin_until(executor, lambda: len(received) > previous_count)
        assert received[-1].longitudinal.acceleration == pytest.approx(
            mux.stop_decel_mps2
        )
        assert received[-1].lateral.steering_tire_angle == 0.0

        mux.external_stop_latched = False
        mux.control_fault_latched = True
        mux.control_fault_reason = "control_loop_deadline_missed"
        previous_count = len(received)
        mux.on_timer()
        assert _spin_until(executor, lambda: len(received) > previous_count)
        assert received[-1].longitudinal.acceleration == pytest.approx(
            mux.stop_decel_mps2
        )
        assert received[-1].lateral.steering_tire_angle == 0.0
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_deadline_watchdog_overrides_frozen_finish_snapshot():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=false",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=0.02",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_finish_deadline_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        command = AckermannControlCommand()
        _set_stamp(command.stamp, 1)
        command.longitudinal.speed = 5.0
        command.longitudinal.acceleration = 0.5
        command.lateral.steering_tire_angle = 0.18
        mux.on_pure_pursuit_cmd(command)

        ready = String()
        ready.data = "Ready"
        mux.on_awsim_state(ready)
        armed = Bool()
        armed.data = True
        mux.on_race_armed(armed)
        start = String()
        start.data = "Start"
        mux.on_awsim_state(start)
        previous_command_count = len(commands)
        previous_status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > previous_command_count
            and len(statuses) > previous_status_count,
        )
        assert mux.finish_terminal_candidate_steering_rad == pytest.approx(0.18)

        finish = String()
        finish.data = "Finish"
        mux.on_awsim_state(finish)
        assert mux.finish_stop_latch.latched
        assert mux.finish_terminal_snapshot_steering_rad == pytest.approx(0.18)

        # A real steady-clock gap, not a synthetic latch, must beat Finish.
        time.sleep(0.03)
        previous_command_count = len(commands)
        previous_status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > previous_command_count
            and len(statuses) > previous_status_count,
        )
        assert mux.last_source == "stop"
        assert commands[-1].longitudinal.speed == 0.0
        assert commands[-1].longitudinal.acceleration == pytest.approx(
            mux.stop_decel_mps2
        )
        assert commands[-1].lateral.steering_tire_angle == 0.0
        assert not statuses[-1].trajectory_tracking_usable
        assert not statuses[-1].safety_constraint_release_ready
        assert not statuses[-1].attack_follow_stop_transport_release_ready
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_finish_stop_keeps_lateral_tracking_when_disarm_constraint_requests_stop():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "finish_stop_enabled:=true",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    try:
        command = AckermannControlCommand()
        command.longitudinal.speed = 0.0
        command.longitudinal.acceleration = mux.finish_stop_decel_mps2
        command.lateral.steering_tire_angle = 0.27
        decision = SafetyConstraintDecision(
            stop_required=True,
            speed_limit_mps=0.0,
            required_brake_decel_mps2=1.5,
            constraint_generation=1,
            plan_generation=1,
            reason="safety_constraint_applied",
        )

        output, source, reason = mux._apply_safety_constraint(
            command,
            "finish_stop",
            "official_vehicle_finish",
            decision,
            mux.now_sec(),
        )

        assert source == "finish_stop"
        assert reason == "official_vehicle_finish"
        assert output.longitudinal.speed == 0.0
        assert output.longitudinal.acceleration == pytest.approx(-3.2)
        assert output.lateral.steering_tire_angle == pytest.approx(0.27)
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_constraint_stop_keeps_only_exact_authorized_pp_lateral_tracking():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_lateral_stop_observer")
    received = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: received.append(msg),
        10,
    )

    plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=7)
    mux.on_overtake_plan(plan)
    constraint = _constraint(
        stamp_sec=10,
        constraint_generation=1,
        plan_generation=7,
        speed_limit_mps=0.5,
        stop_requested=True,
        release_authorized=False,
        reason="maneuver_transaction_tracking_stop",
    )
    mux.on_safety_constraint(constraint)

    command = AckermannControlCommand()
    _set_stamp(command.stamp, 20)
    command.longitudinal.speed = 2.0
    command.longitudinal.acceleration = 0.4
    command.lateral.steering_tire_angle = 0.31
    mux.on_pure_pursuit_cmd(command)
    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 20)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 7
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.lateral_stop_authority_kind = plan.lateral_stop_authority_kind
    proof.lateral_stop_transaction_pass_direction = (
        plan.lateral_stop_transaction_pass_direction
    )
    proof.lateral_stop_authority_token = plan.lateral_stop_authority_token
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)
    _bind_envelope_from_legacy(mux, command, proof, sequence=1)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(received))
        assert received[-1].longitudinal.speed == pytest.approx(0.0)
        assert received[-1].longitudinal.acceleration <= -1.5
        assert received[-1].lateral.steering_tire_angle == pytest.approx(0.31)
        assert mux.steering_limiter.last_steering_rad == pytest.approx(0.31)

        # A same-generation plan with a different stamp is not the exact
        # SafetyEvaluator bundle and must fall back to the ordinary stop.
        mismatched_plan = _authorized_lateral_stop_plan(
            stamp_sec=11, generation=7
        )
        mux.on_overtake_plan(mismatched_plan)
        _set_stamp(command.stamp, 21)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 21)
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=3)
        previous_count = len(received)
        mux.on_timer()
        assert _spin_until(executor, lambda: len(received) > previous_count)
        assert received[-1].longitudinal.speed == pytest.approx(0.0)
        assert received[-1].lateral.steering_tire_angle == pytest.approx(0.0)

        # Even a new exact plan/constraint bundle cannot override an unusable
        # tracking proof.
        next_plan = _authorized_lateral_stop_plan(stamp_sec=12, generation=8)
        mux.on_overtake_plan(next_plan)
        next_constraint = _constraint(
            stamp_sec=12,
            constraint_generation=2,
            plan_generation=8,
            speed_limit_mps=0.5,
            stop_requested=True,
            release_authorized=False,
            reason="maneuver_transaction_tracking_stop",
        )
        next_constraint.required_brake_decel_mps2 = 1.5
        mux.on_safety_constraint(next_constraint)
        _set_stamp(command.stamp, 22)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 22)
        proof.plan_generation = 8
        proof.lateral_stop_authority_token = next_plan.lateral_stop_authority_token
        proof.trajectory_tracking_usable = False
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=5)
        previous_count = len(received)
        mux.on_timer()
        assert _spin_until(executor, lambda: len(received) > previous_count)
        assert received[-1].lateral.steering_tire_angle == pytest.approx(0.0)

        # External safety remains a higher-priority fail-closed authority even
        # when every planner/tracking proof is otherwise valid.
        final_plan = _authorized_lateral_stop_plan(stamp_sec=13, generation=9)
        mux.on_overtake_plan(final_plan)
        final_constraint = _constraint(
            stamp_sec=13,
            constraint_generation=3,
            plan_generation=9,
            speed_limit_mps=0.5,
            stop_requested=True,
            release_authorized=False,
            reason="maneuver_transaction_tracking_stop",
        )
        final_constraint.required_brake_decel_mps2 = 1.5
        mux.on_safety_constraint(final_constraint)
        _set_stamp(command.stamp, 23)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 23)
        proof.plan_generation = 9
        proof.lateral_stop_authority_token = final_plan.lateral_stop_authority_token
        proof.trajectory_tracking_usable = True
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=7)
        mux.external_stop_latched = True
        previous_count = len(received)
        mux.on_timer()
        assert _spin_until(executor, lambda: len(received) > previous_count)
        assert received[-1].longitudinal.speed == pytest.approx(0.0)
        assert received[-1].lateral.steering_tire_angle == pytest.approx(0.0)
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_planner_stop_keeps_exact_baseline_pp_steering_while_braking():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_baseline_stop_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )

    plan = _baseline_stop_plan(stamp_sec=30, generation=10)
    mux.on_overtake_plan(plan)
    constraint = _constraint(
        stamp_sec=30,
        constraint_generation=1,
        plan_generation=10,
        speed_limit_mps=0.5,
        stop_requested=True,
        release_authorized=False,
        reason="safe_stop",
    )
    mux.on_safety_constraint(constraint)

    command = AckermannControlCommand()
    _set_stamp(command.stamp, 30)
    command.longitudinal.speed = 6.0
    command.longitudinal.acceleration = 0.2
    command.lateral.steering_tire_angle = 0.28
    mux.on_pure_pursuit_cmd(command)
    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 30)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 10
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)
    _bind_envelope_from_legacy(mux, command, proof, sequence=1)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)

    def spin_publications() -> None:
        command_count = len(commands)
        status_count = len(statuses)
        deadline = time.monotonic() + 0.20
        while (
            len(commands) == command_count or len(statuses) == status_count
        ) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    try:
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= -1.5
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.28)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is False
        assert (
            statuses[-1].reason
            == "longitudinal_safety_stop_with_lateral_tracking"
        )
        assert mux.last_verified_baseline_stop_plan_generation == 10

        # plan/constraintの片topicだけが50 ms先行しても、縦停止を維持したまま
        # 直前にexact検証済みの操舵だけを短時間保持する。
        skewed_plan = _baseline_stop_plan(stamp_sec=30, generation=10)
        skewed_plan.header.stamp.nanosec = 50_000_000
        mux.on_overtake_plan(skewed_plan)
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.28)
        assert (
            statuses[-1].reason
            == "longitudinal_safety_stop_with_lateral_hold"
        )
        # 実bagのABORT_HOLD列では、次generationのexact plan/constraintが
        # 同時に届いた一方、PP command/statusだけがN-1で1 control tick遅れた。
        # muxがpp_tracking_delivery_gap_usableと判定できるこの順序では、縦STOPを
        # 維持しながら直前のexact baseline steeringだけを短時間保持する。
        next_plan = _baseline_stop_plan(stamp_sec=31, generation=11)
        mux.on_overtake_plan(next_plan)
        next_constraint = _constraint(
            stamp_sec=31,
            constraint_generation=2,
            plan_generation=11,
            speed_limit_mps=0.5,
            stop_requested=True,
            release_authorized=False,
            reason="safe_stop",
        )
        mux.on_safety_constraint(next_constraint)
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= -1.5
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.28)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is False
        assert (
            statuses[-1].reason
            == "longitudinal_safety_stop_with_lateral_hold"
        )
        assert mux.last_verified_baseline_stop_plan_generation == 10

        # generation 11の同stamp command/statusを揃え、一度exact trackingへ
        # 戻してから、次のtransport gapの期限切れを独立に検証する。
        _set_stamp(command.stamp, 31)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 31)
        proof.plan_generation = 11
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=3)
        mux.on_timer()
        spin_publications()
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.28)
        assert mux.last_verified_baseline_stop_plan_generation == 11

        # bounded holdの期限切れは従来どおりzero-steer STOPへfail-closedする。
        # 停止要求そのものは一度も解除しない。
        mux.last_verified_baseline_stop_steering_time_sec = (
            mux.now_sec() - mux.lateral_stop_steering_hold_timeout_sec - 0.01
        )
        expired_plan = _baseline_stop_plan(stamp_sec=32, generation=12)
        mux.on_overtake_plan(expired_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=32,
                constraint_generation=3,
                plan_generation=12,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="safe_stop",
            )
        )
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        # SteeringLimiterはsource切替直後の不連続を抑えるため、同一process内で
        # 即時連続実行したテストでは角度が1周期だけ残り得る。authority自体が
        # ordinary stopへ戻ったことをfinal source/statusで検証する。
        assert mux.last_source == "stop"
        assert statuses[-1].reason == "final_source_not_pure_pursuit"

        # External safetyはPlanner由来の縦停止より上位であり、exactな基準軌道
        # proofが揃っていても全commandを従来どおり停止へ置換する。
        final_plan = _baseline_stop_plan(stamp_sec=33, generation=13)
        mux.on_overtake_plan(final_plan)
        final_constraint = _constraint(
            stamp_sec=33,
            constraint_generation=4,
            plan_generation=13,
            speed_limit_mps=0.5,
            stop_requested=True,
            release_authorized=False,
            reason="safe_stop",
        )
        mux.on_safety_constraint(final_constraint)
        _set_stamp(command.stamp, 33)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 33)
        proof.plan_generation = 13
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=5)
        mux.external_stop_latched = True
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)

        # A SafetyConstraintAuthority fault latch is a safety barrier, not a
        # normal one-generation transport gap.  Even with an exact current
        # baseline STOP bundle and fresh N-1 PP proof, it must invalidate the
        # prior steering proof and keep the complete command fail-closed.
        mux.external_stop_latched = False
        verified_plan = _baseline_stop_plan(stamp_sec=34, generation=14)
        mux.on_overtake_plan(verified_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=34,
                constraint_generation=5,
                plan_generation=14,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="safe_stop",
            )
        )
        _set_stamp(command.stamp, 34)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 34)
        proof.plan_generation = 14
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=7)
        mux.on_timer()
        spin_publications()
        restored_baseline_steering = commands[-1].lateral.steering_tire_angle
        assert 0.0 < restored_baseline_steering
        assert restored_baseline_steering == pytest.approx(0.28)
        assert mux.last_verified_baseline_stop_steering_rad == pytest.approx(
            restored_baseline_steering
        )
        assert mux.last_verified_baseline_stop_plan_generation == 14

        mux.safety_authority._fault_latched = True
        fault_plan = _baseline_stop_plan(stamp_sec=35, generation=15)
        mux.on_overtake_plan(fault_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=35,
                constraint_generation=6,
                plan_generation=15,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="safe_stop",
            )
        )
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].reason == "final_source_not_pure_pursuit"
        assert mux.last_verified_baseline_stop_plan_generation is None
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_authority_fault_zeros_current_and_historical_pp_steering():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    # This test drives the control tick explicitly. Disable the wall timer so
    # observer spin cannot interleave a second publication with each asserted
    # contract transition.
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_historical_lateral_stop_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )

    # First establish the older authority generation that will remain latched.
    old_plan = _authorized_lateral_stop_plan(stamp_sec=40, generation=20)
    mux.on_overtake_plan(old_plan)
    mux.on_safety_constraint(
        _constraint(
            stamp_sec=40,
            constraint_generation=1,
            plan_generation=20,
            speed_limit_mps=0.5,
            stop_requested=True,
            release_authorized=False,
            reason="maneuver_transaction_tracking_stop",
        )
    )

    command = AckermannControlCommand()
    _set_stamp(command.stamp, 40)
    command.longitudinal.speed = 0.75
    command.longitudinal.acceleration = 0.2
    command.lateral.steering_tire_angle = -0.34
    mux.on_pure_pursuit_cmd(command)
    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 40)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 20
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.lateral_stop_authority_kind = old_plan.lateral_stop_authority_kind
    proof.lateral_stop_transaction_pass_direction = (
        old_plan.lateral_stop_transaction_pass_direction
    )
    proof.lateral_stop_authority_token = old_plan.lateral_stop_authority_token
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)
    _bind_envelope_from_legacy(mux, command, proof, sequence=1)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)

    def spin_publications() -> None:
        command_count = len(commands)
        status_count = len(statuses)
        deadline = time.monotonic() + 0.20
        while (
            len(commands) == command_count or len(statuses) == status_count
        ) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    try:
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(-0.34)

        # 同一generationのSTOP bundleをplannerが再送すると、plan topicだけが
        # constraint peerより1 callback先に届くことがある。直前に同じ横STOPを
        # exact tracking済みでstamp差も50 msだけなら、縦停止を維持したまま
        # 最後の検証済み操舵だけを保持する。
        skewed_republish = _authorized_lateral_stop_plan(
            stamp_sec=40, generation=20
        )
        skewed_republish.header.stamp.nanosec = 50_000_000
        skewed_republish.trajectory.header.stamp.nanosec = 50_000_000
        mux.on_overtake_plan(skewed_republish)
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(-0.34)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is False
        assert (
            statuses[-1].reason
            == "longitudinal_safety_stop_with_lateral_hold"
        )

        # A prior authority fault keeps longitudinal motion stopped and reports
        # the old generation.  The current generation may supply steering only
        # through its exact plan/constraint/PP proof bundle; this does not clear
        # or bypass the authority latch.
        mux.safety_authority._fault_latched = True
        current_plan = _authorized_lateral_stop_plan(
            stamp_sec=41, generation=21
        )
        mux.on_overtake_plan(current_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=41,
                constraint_generation=2,
                plan_generation=21,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="maneuver_transaction_tracking_stop",
            )
        )
        _set_stamp(command.stamp, 41)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 41)
        proof.plan_generation = 21
        proof.lateral_stop_authority_token = current_plan.lateral_stop_authority_token
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=3)
        mux.on_timer()
        spin_publications()

        assert mux.safety_authority._fault_latched is True
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= mux.stop_decel_mps2
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert mux.steering_limiter.last_steering_rad == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        # Fault解除用ACKは残すが、最終出力はzero-steer STOPのまま。
        assert statuses[-1].safety_constraint_release_ready is True
        assert statuses[-1].plan_generation == 21
        assert (
            statuses[-1].reason
            != "longitudinal_safety_stop_with_lateral_tracking"
        )

        # Authorityの過去fault latchを解除する明示PASS bundleでも、実解除に
        # 必要な連続release周期の間は縦速度を0に保つ。その1周期目で現在の
        # exact PP proofがあるなら、操舵だけは0へresetしない。
        release_plan = _authorized_lateral_stop_plan(
            stamp_sec=42, generation=21
        )
        release_plan.phase = OvertakePlan.PASSING
        release_plan.target_vehicle_id = "grid_d2"
        release_plan.pass_direction = -1
        mux.on_overtake_plan(release_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=42,
                constraint_generation=3,
                plan_generation=21,
                speed_limit_mps=0.75,
                stop_requested=False,
                release_authorized=True,
                reason="normal_limit",
            )
        )
        _set_stamp(command.stamp, 42)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 42)
        proof.plan_generation = 21
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=5)
        mux.on_timer()
        spin_publications()
        assert mux.safety_authority._fault_latched is True
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        # Fault解除用ACKは診断上trueでも、操舵authorityには昇格しない。
        assert statuses[-1].safety_constraint_release_ready is True
        assert (
            statuses[-1].reason
            != "longitudinal_safety_stop_with_lateral_tracking"
        )

        # A SafetyConstraintAuthority fault latch is a proof barrier. Even if
        # the next exact planner generation arrives one timer tick before its
        # PP peer, do not carry the pre-fault steering bundle across it. The
        # vehicle remains longitudinally stopped and waits for a new exact PP
        # proof under the current authority generation.
        hold_plan = _authorized_lateral_stop_plan(
            stamp_sec=43, generation=22
        )
        mux.on_overtake_plan(hold_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=43,
                constraint_generation=4,
                plan_generation=22,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="release_pending_safe_cycles",
            )
        )
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert mux.safety_authority._fault_latched is True
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is False
        assert statuses[-1].reason == "final_source_not_pure_pursuit"

        assert mux.last_verified_lateral_stop_plan_generation is None

        # generation 22の同stamp proofを揃えて一度exact trackingへ戻す。
        # 次のtransport gapの有効期限だけを独立に検証できる状態にする。
        _set_stamp(command.stamp, 43)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 43)
        proof.plan_generation = 22
        proof.lateral_stop_authority_token = hold_plan.lateral_stop_authority_token
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=7)
        mux.on_timer()
        spin_publications()
        # The fault barrier reset the steering limiter to zero. A new exact
        # bundle may restore tracking, but only through the configured rate
        # limit; it must not jump directly back to the pre-fault angle.
        restored_steering = commands[-1].lateral.steering_tire_angle
        assert restored_steering == pytest.approx(0.0)
        assert mux.last_verified_lateral_stop_steering_rad is None
        assert mux.last_verified_lateral_stop_plan_generation is None

        # The hold is not an open-ended fallback. Once its short validity
        # expires, an unmatched proof returns to the ordinary fail-closed stop.
        mux.last_verified_lateral_stop_steering_time_sec = (
            mux.now_sec() - mux.lateral_stop_steering_hold_timeout_sec - 0.01
        )
        expired_plan = _authorized_lateral_stop_plan(
            stamp_sec=44, generation=23
        )
        mux.on_overtake_plan(expired_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=44,
                constraint_generation=5,
                plan_generation=23,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="release_pending_safe_cycles",
            )
        )
        _set_stamp(command.stamp, 44)
        mux.on_pure_pursuit_cmd(command)
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is False

        # External safety still owns the complete command and must zero the
        # steering even if the same planner proof remains available.
        mux.external_stop_latched = True
        _set_stamp(command.stamp, 45)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 45)
        proof.plan_generation = 23
        mux.on_pure_pursuit_tracking_status(proof)
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_maneuver_stop_hold_rejects_steering_older_than_previous_generation():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_maneuver_hold_generation_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)

    def spin_publications() -> None:
        command_count = len(commands)
        status_count = len(statuses)
        deadline = time.monotonic() + 0.20
        while (
            len(commands) == command_count or len(statuses) == status_count
        ) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    def send_pp_pair(stamp_sec: int, generation: int) -> None:
        nonlocal pp_sequence
        command = AckermannControlCommand()
        _set_stamp(command.stamp, stamp_sec)
        command.longitudinal.speed = 0.75
        command.longitudinal.acceleration = 0.2
        command.lateral.steering_tire_angle = 0.26
        mux.on_pure_pursuit_cmd(command)

        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, stamp_sec)
        proof.header.frame_id = "base_link"
        proof.plan_generation = generation
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.lateral_stop_authority_kind = (
            OvertakePlan.LATERAL_STOP_CURRENT_D_HOLD
        )
        proof.lateral_stop_transaction_pass_direction = -1
        proof.lateral_stop_authority_token = (1 << 32) | generation
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(
            mux, command, proof, sequence=pp_sequence
        )
        pp_sequence += 2

    def send_stop_authority(stamp_sec: int, generation: int) -> None:
        mux.on_overtake_plan(
            _authorized_lateral_stop_plan(
                stamp_sec=stamp_sec, generation=generation
            )
        )
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=stamp_sec,
                constraint_generation=generation - 29,
                plan_generation=generation,
                speed_limit_mps=0.5,
                stop_requested=True,
                release_authorized=False,
                reason="maneuver_transaction_tracking_stop",
            )
        )

    pp_sequence = 1
    try:
        # Exact maneuver N establishes the only steering proof eligible for a
        # bounded delivery-gap hold.
        send_stop_authority(stamp_sec=50, generation=30)
        send_pp_pair(stamp_sec=50, generation=30)
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.26)
        assert mux.last_verified_lateral_stop_plan_generation == 30

        # Authority N+1 with PP N is stale for lateral authority. Longitudinal
        # STOP remains authoritative, but N steering must not cross the gap.
        send_stop_authority(stamp_sec=51, generation=31)
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration <= mux.stop_decel_mps2
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].reason == "final_source_not_pure_pursuit"
        assert statuses[-1].safety_constraint_release_ready is False
        assert statuses[-1].plan_generation == 30

        # Receiving PP N+1 without an exact control tick does not verify it.
        # Once authority advances to N+2, exact N is outside the N-1/N window
        # and the mux must fall back to a complete zero-steering STOP.
        send_pp_pair(stamp_sec=51, generation=31)
        send_stop_authority(stamp_sec=52, generation=32)
        mux.on_timer()
        spin_publications()
        assert mux.last_verified_lateral_stop_plan_generation == 30
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].safety_constraint_release_ready is False
        assert statuses[-1].reason == "final_source_not_pure_pursuit"

        # Authority N+2自身のfresh command/proofが揃った制御tickでだけ解除proofを
        # publishする。二世代前のACKでfalseだった状態から、現在世代のexact
        # bundleでtrueへ変わることを同じテストで固定する。
        send_pp_pair(stamp_sec=52, generation=32)
        mux.on_timer()
        spin_publications()
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert statuses[-1].safety_constraint_release_ready is True
        assert statuses[-1].plan_generation == 32
        assert mux.last_verified_lateral_stop_plan_generation == 32
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_same_stamp_republication_cannot_hide_ros_clock_stall():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "control_rate_hz:=50.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "ros_clock_stall_timeout_sec:=0.15",
            "-p",
            "race_arm_required:=true",
            "-p",
            "control_fault_clear_safe_cycles:=1",
            "-p",
            "debug_publish_period_sec:=0.01",
        ]
    )
    mux = HybridControlMuxNode()
    # Keep this fixture independent from any /clock publisher left on the
    # shared ROS graph by a prior simulator run.
    _set_mux_ros_time_for_envelope(mux, 0)
    mux.timer.cancel()
    peer = Node("hybrid_control_mux_clock_stall_peer")
    commands = []
    debug = []
    peer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    peer.create_subscription(
        String,
        "output/debug",
        lambda msg: debug.append(json.loads(msg.data)),
        10,
    )
    command_pub = peer.create_publisher(
        AckermannControlCommand, "input/pure_pursuit_control_cmd", 10
    )
    plan_pub = peer.create_publisher(OvertakePlan, "input/overtake_plan", 10)
    constraint_pub = peer.create_publisher(
        SafetyConstraint, "input/safety_constraint", 10
    )
    tracking_pub = peer.create_publisher(
        ControllerTrackingStatus,
        "input/pure_pursuit_tracking_status",
        10,
    )

    command = AckermannControlCommand()
    command.longitudinal.speed = 2.0
    command.longitudinal.acceleration = 0.5
    plan = OvertakePlan()
    plan.header.frame_id = "map"
    plan.plan_generation = 1
    constraint = SafetyConstraint()
    constraint.header.frame_id = "map"
    constraint.constraint_generation = 1
    constraint.plan_generation = 1
    constraint.valid = True
    constraint.release_authorized = True
    constraint.speed_limit_mps = 3.0
    proof = ControllerTrackingStatus()
    proof.header.frame_id = "base_link"
    proof.plan_generation = 1
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_cmd(command)
    mux.on_pure_pursuit_tracking_status(proof)
    # Keep the U2 authority sample at stamp zero as well: this test proves
    # that repeated legacy traffic cannot keep a frozen ROS clock usable.
    for sequence in (1, 2):
        envelope = _command_envelope(
            stamp_sec=0,
            plan_generation=1,
            command_sequence=sequence,
        )
        envelope.command = command
        envelope.mpc_horizon_usable = proof.mpc_horizon_usable
        envelope.pp_command_fresh = proof.pp_command_fresh
        envelope.trajectory_tracking_usable = proof.trajectory_tracking_usable
        envelope.command_age_sec = proof.command_age_sec
        mux.on_pure_pursuit_command_envelope(envelope)
    mux.on_overtake_plan(plan)
    mux.on_safety_constraint(constraint)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(peer)
    try:
        # Frozen simulator time before the official arm is readiness only:
        # motion stays fail-closed, but no operational clock fault is latched.
        deadline = time.monotonic() + 0.20
        while time.monotonic() < deadline:
            command_pub.publish(command)
            tracking_pub.publish(proof)
            plan_pub.publish(plan)
            constraint_pub.publish(constraint)
            executor.spin_once(timeout_sec=0.01)
            _set_mux_ros_time_for_envelope(mux, 0)
            mux.on_timer()

        assert commands
        assert all(msg.longitudinal.speed == 0.0 for msg in commands)
        assert mux.control_fault_latched is False
        assert mux.ros_clock_motion_ready is False
        assert all(item.get("ros_clock_stalled") is False for item in debug)

        mux.on_race_armed(Bool(data=True))
        mux.on_timer()
        assert mux.last_source == "stop"
        assert mux.ros_clock_motion_ready is False

        # Even beyond the stall budget, a race epoch that has never observed
        # post-arm clock progress remains a readiness STOP rather than an
        # operational fault latch.
        no_progress_deadline = time.monotonic() + 0.20
        while time.monotonic() < no_progress_deadline:
            command_pub.publish(command)
            tracking_pub.publish(proof)
            plan_pub.publish(plan)
            constraint_pub.publish(constraint)
            executor.spin_once(timeout_sec=0.01)
            _set_mux_ros_time_for_envelope(mux, 0)
            mux.on_timer()
        assert mux.last_source == "stop"
        assert mux.ros_clock_motion_ready is False
        assert mux.control_fault_latched is False

        # A strictly newer post-arm clock sample establishes readiness. The
        # arm epoch reset invalidates prior envelopes, so submit a new exact
        # pair before motion can resume.
        mux.get_clock()._set_ros_time_is_active(True)
        mux.get_clock().set_ros_time_override(Time(nanoseconds=1))
        for sequence in (3, 4):
            envelope = _command_envelope(
                stamp_sec=0,
                plan_generation=1,
                command_sequence=sequence,
            )
            envelope.command = command
            envelope.mpc_horizon_usable = proof.mpc_horizon_usable
            envelope.pp_command_fresh = proof.pp_command_fresh
            envelope.trajectory_tracking_usable = proof.trajectory_tracking_usable
            envelope.command_age_sec = proof.command_age_sec
            mux.on_pure_pursuit_command_envelope(envelope)
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(constraint)
        mux.on_timer()
        assert mux.ros_clock_motion_ready is True

        motion_deadline = time.monotonic() + 0.10
        while time.monotonic() < motion_deadline:
            command_pub.publish(command)
            tracking_pub.publish(proof)
            plan_pub.publish(plan)
            constraint_pub.publish(constraint)
            executor.spin_once(timeout_sec=0.01)
            mux.get_clock().set_ros_time_override(Time(nanoseconds=1))
            mux.on_timer()
        assert any(msg.longitudinal.speed > 0.0 for msg in commands)

        # All headers and /clock now intentionally remain unchanged. Repeated
        # receipt traffic must not hide a post-readiness clock stall.
        stall_deadline = time.monotonic() + 0.30
        while time.monotonic() < stall_deadline:
            command_pub.publish(command)
            tracking_pub.publish(proof)
            plan_pub.publish(plan)
            constraint_pub.publish(constraint)
            executor.spin_once(timeout_sec=0.01)
            mux.get_clock().set_ros_time_override(Time(nanoseconds=1))
            mux.on_timer()

        assert commands[-1].longitudinal.speed == 0.0
        assert any(item.get("ros_clock_stalled") is True for item in debug)
        assert mux.control_fault_latched is True

        # A later race epoch must baseline its own clock even if AWSIM reset
        # makes the new stamp smaller than the prior epoch. Existing fault
        # latches remain fail-closed and are deliberately not cleared here.
        mux.on_race_armed(Bool(data=False))
        mux.get_clock().set_ros_time_override(Time(nanoseconds=0))
        mux.on_race_armed(Bool(data=True))
        assert mux.control_fault_latched is True
        mux.on_timer()
        assert mux.ros_clock_observation_reason == "ros_clock_initial"
        assert mux.ros_clock_motion_ready is False
    finally:
        executor.remove_node(peer)
        executor.remove_node(mux)
        peer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_tracking_status_exposes_release_ready_behind_planner_stop_only():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    observer = Node("hybrid_control_mux_release_ready_observer")
    received = []
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: received.append(msg),
        10,
    )

    # 実走のcurrent-d SAFE_STOPは停止しているためtrajectoryの空間長が0になる。
    # ATTACK_FOLLOW・direction=0・targetありの厳密なbootstrapだけを許可する。
    plan = _bootstrap_current_d_stop_plan(stamp_sec=30, generation=11)
    mux.on_overtake_plan(plan)
    constraint = _constraint(
        stamp_sec=30,
        constraint_generation=1,
        plan_generation=11,
        speed_limit_mps=0.2,
        stop_requested=True,
        release_authorized=False,
        reason="maneuver_transaction_tracking_stop",
    )
    mux.on_safety_constraint(constraint)

    command = AckermannControlCommand()
    _set_stamp(command.stamp, 31)
    command.longitudinal.speed = 0.2
    command.longitudinal.acceleration = -1.0
    command.lateral.steering_tire_angle = 0.15
    mux.on_pure_pursuit_cmd(command)
    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 31)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 11
    proof.mpc_horizon_usable = True
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)
    _bind_envelope_from_legacy(mux, command, proof, sequence=1)

    # 過去のclock stall latchは残っているが、現在のwatchdog原因は解消済み。
    # final sourceはstopのままでも、planner自身の停止constraintを解除した後に
    # 追従できる同generation PP proofだけをrelease-readyとして返す。
    mux.control_fault_latched = True
    mux.control_fault_reason = "ros_clock_stalled"
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: received
            and received[-1].plan_generation == 11
            and received[-1].header.stamp.sec == 31,
        )
        assert received[-1].trajectory_tracking_usable is False
        assert received[-1].safety_constraint_release_ready is True
        assert received[-1].plan_generation == 11
        assert received[-1].reason == "final_source_not_pure_pursuit"

        # 起動直後のclock stall等でSafetyConstraintAuthority自身が古い世代を
        # latchしていても、車両のstopは維持したまま、現在の厳密な
        # plan/constraint/PP proof bundleだけを次世代release-readyへ橋渡しする。
        # 実解除はplanner・authority・control faultの各連続安全周期が別途必要。
        mux.safety_authority._fault_latched = True
        next_plan = _tracking_follow_stop_plan(
            stamp_sec=32, generation=12
        )
        mux.on_overtake_plan(next_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=32,
                constraint_generation=2,
                plan_generation=12,
                speed_limit_mps=0.2,
                stop_requested=True,
                release_authorized=False,
                reason="release_pending_safe_cycles",
            )
        )
        _set_stamp(command.stamp, 32)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 32)
        proof.plan_generation = 12
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=3)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: received
            and received[-1].plan_generation == 12
            and received[-1].header.stamp.sec == 32,
        )
        assert received[-1].trajectory_tracking_usable is False
        assert received[-1].safety_constraint_release_ready is True
        assert received[-1].plan_generation == 12

        # planner gateがreleaseを出した後も、Authorityとcontrol fault latchは
        # それぞれ3つの新しい明示releaseを直列に確認する。Authorityの3回目が
        # control側の1回目にもなるため、合計5回目まで実commandを停止する。
        # その解除確認中にreadyを落としてstopへ戻す自己デッドロックを防ぐ。
        for release_index, stamp_sec in enumerate(
            (33, 34, 35, 36, 37), start=1
        ):
            release_plan = _authorized_lateral_stop_plan(
                stamp_sec=stamp_sec, generation=12
            )
            release_plan.phase = OvertakePlan.PASSING
            release_plan.target_vehicle_id = "grid_d2"
            release_plan.pass_direction = -1
            mux.on_overtake_plan(release_plan)
            mux.on_safety_constraint(
                _constraint(
                    stamp_sec=stamp_sec,
                    constraint_generation=3,
                    plan_generation=12,
                    speed_limit_mps=0.75,
                    stop_requested=False,
                    release_authorized=True,
                    reason="normal_limit",
                )
            )
            _set_stamp(command.stamp, stamp_sec)
            mux.on_pure_pursuit_cmd(command)
            _set_stamp(proof.header.stamp, stamp_sec)
            mux.on_pure_pursuit_tracking_status(proof)
            _bind_envelope_from_legacy(
                mux,
                command,
                proof,
                sequence=5 + (release_index - 1) * 2,
            )
            mux.on_timer()
            assert _spin_until(
                executor,
                lambda: received
                and received[-1].header.stamp.sec == stamp_sec,
            )
            if release_index < 5:
                assert received[-1].trajectory_tracking_usable is False
                assert received[-1].safety_constraint_release_ready is True
            else:
                # A legacy schema 1 command may clear the old fault latches,
                # but cannot become PASS motion authority.
                assert received[-1].trajectory_tracking_usable is False
                assert received[-1].safety_constraint_release_ready is False
                assert mux.safety_authority._fault_latched is False
                assert mux.control_fault_latched is False

        # 外部停止はplanner constraintの自己解除ではないため、同じPP proofが
        # あってもrelease-readyを絶対に出さない。
        mux.external_stop_latched = True
        _set_stamp(command.stamp, 38)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 38)
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=15)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: received and received[-1].header.stamp.sec == 38,
        )
        assert received[-1].safety_constraint_release_ready is False

        # 空trajectory例外を実PASSへ広げない。PASSING/direction指定済み軌道は
        # 完全trajectory認可が無ければ、同generation PP proofでも解除不可。
        mux.external_stop_latched = False
        invalid_pass = _bootstrap_current_d_stop_plan(
            stamp_sec=39, generation=13
        )
        invalid_pass.phase = OvertakePlan.PASSING
        invalid_pass.pass_direction = -1
        mux.on_overtake_plan(invalid_pass)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=39,
                constraint_generation=3,
                plan_generation=13,
                speed_limit_mps=0.2,
                stop_requested=True,
                release_authorized=False,
                reason="maneuver_transaction_tracking_stop",
            )
        )
        _set_stamp(command.stamp, 39)
        mux.on_pure_pursuit_cmd(command)
        _set_stamp(proof.header.stamp, 39)
        proof.plan_generation = 13
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(mux, command, proof, sequence=17)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: received
            and received[-1].plan_generation == 13
            and received[-1].header.stamp.sec == 39,
        )
        assert received[-1].safety_constraint_release_ready is False
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_planner_stop_bootstrap_rejects_relaxed_or_over_cap_contracts():
    rclpy.init()
    mux = HybridControlMuxNode()
    try:
        plan = _bootstrap_current_d_stop_plan(stamp_sec=50, generation=21)
        mux.on_overtake_plan(plan)
        now_sec = mux.now_sec()

        def state(
            *,
            release_authorized: bool = False,
            speed_limit_mps: float = 0.20,
            reason: str = "maneuver_transaction_tracking_stop",
        ) -> SafetyConstraintState:
            return SafetyConstraintState(
                constraint_generation=1,
                plan_generation=21,
                valid=True,
                stop_requested=True,
                release_authorized=release_authorized,
                speed_limit_mps=speed_limit_mps,
                required_brake_decel_mps2=1.0,
                header_stamp_ns=50_000_000_000,
                frame_id="map",
                reason=reason,
            )

        assert mux._planner_stop_release_bootstrap_valid(
            plan_generation=21,
            constraint=state(),
            now_sec=now_sec,
        )
        assert not mux._planner_stop_release_bootstrap_valid(
            plan_generation=21,
            constraint=state(release_authorized=True),
            now_sec=now_sec,
        )
        assert mux._planner_stop_release_bootstrap_valid(
            plan_generation=21,
            constraint=state(reason="release_pending_safe_cycles"),
            now_sec=now_sec,
        )
        assert not mux._planner_stop_release_bootstrap_valid(
            plan_generation=21,
            constraint=state(speed_limit_mps=0.201),
            now_sec=now_sec,
        )

        missing_target = _bootstrap_current_d_stop_plan(
            stamp_sec=51, generation=22
        )
        missing_target.target_vehicle_id = ""
        mux.on_overtake_plan(missing_target)
        assert not mux._planner_stop_release_bootstrap_valid(
            plan_generation=22,
            constraint=SafetyConstraintState(
                constraint_generation=2,
                plan_generation=22,
                valid=True,
                stop_requested=True,
                release_authorized=False,
                speed_limit_mps=0.20,
                required_brake_decel_mps2=1.0,
                header_stamp_ns=51_000_000_000,
                frame_id="map",
                reason="maneuver_transaction_tracking_stop",
            ),
            now_sec=mux.now_sec(),
        )

        follow_plan = _tracking_follow_stop_plan(
            stamp_sec=52, generation=23
        )
        mux.on_overtake_plan(follow_plan)
        assert mux._planner_stop_release_bootstrap_valid(
            plan_generation=23,
            constraint=SafetyConstraintState(
                constraint_generation=3,
                plan_generation=23,
                valid=True,
                stop_requested=True,
                release_authorized=False,
                speed_limit_mps=0.01,
                required_brake_decel_mps2=1.0,
                header_stamp_ns=52_000_000_000,
                frame_id="map",
                reason="release_pending_safe_cycles",
            ),
            now_sec=mux.now_sec(),
        )
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("stop_cap_mps", "pp_speed_mps", "expected_release_ready"),
    [
        (0.20, 0.21, False),
        # The planner's STOP cap follows the nearly stationary ego speed and
        # may therefore be lower than the fixed 0.20 m/s PP warm-up command.
        # Motion remains stopped; the configured bootstrap cap bounds only
        # the tracking proof used to break the STOP/release handshake cycle.
        (0.01, 0.20, True),
    ],
)
def test_planner_stop_bootstrap_bounds_pp_command_by_bootstrap_cap(
    stop_cap_mps, pp_speed_mps, expected_release_ready
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    observer = Node("hybrid_control_mux_over_cap_bootstrap_observer")
    received = []
    commands = []
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: received.append(msg),
        10,
    )
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )

    mux.on_overtake_plan(
        _bootstrap_current_d_stop_plan(stamp_sec=60, generation=31)
    )
    mux.on_safety_constraint(
        _constraint(
            stamp_sec=60,
            constraint_generation=1,
            plan_generation=31,
            speed_limit_mps=stop_cap_mps,
            stop_requested=True,
            release_authorized=False,
            reason="maneuver_transaction_tracking_stop",
        )
    )
    command = AckermannControlCommand()
    _set_stamp(command.stamp, 61)
    command.longitudinal.speed = pp_speed_mps
    command.longitudinal.acceleration = -1.0
    command.lateral.steering_tire_angle = 0.1
    mux.on_pure_pursuit_cmd(command)
    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 61)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 31
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)
    _bind_envelope_from_legacy(mux, command, proof, sequence=1)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(received) and bool(commands))
        assert received[-1].plan_generation == 31
        assert (
            received[-1].safety_constraint_release_ready
            is expected_release_ready
        )
        assert received[-1].trajectory_tracking_usable is False
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_tracking_usable_requires_pp_generation_proof_and_finite_raw_command():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=false",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_tracking_proof_observer")
    received = []
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: received.append(msg),
        10,
    )

    plan = OvertakePlan()
    _set_stamp(plan.header.stamp, 1)
    plan.header.frame_id = "map"
    plan.phase = OvertakePlan.FREE_RUN
    plan.plan_generation = 7
    mux.on_overtake_plan(plan)

    constraint = SafetyConstraint()
    _set_stamp(constraint.header.stamp, 1)
    constraint.header.frame_id = "map"
    constraint.constraint_generation = 1
    constraint.plan_generation = 7
    constraint.valid = True
    constraint.release_authorized = True
    constraint.speed_limit_mps = 3.0
    mux.on_safety_constraint(constraint)

    command = AckermannControlCommand()
    _set_stamp(command.stamp, 1)
    command.longitudinal.speed = 2.0
    command.longitudinal.acceleration = 0.5
    command.lateral.steering_tire_angle = 0.1
    mux.on_pure_pursuit_cmd(command)

    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 1)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 7
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)

    def tick_and_receive_status() -> None:
        previous_count = len(received)
        mux.on_timer()
        deadline = time.monotonic() + 0.20
        while len(received) == previous_count and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert len(received) > previous_count

    try:
        tick_and_receive_status()
        assert received[-1].trajectory_tracking_usable is True
        assert received[-1].plan_generation == 7
        assert received[-1].reason == "ready"

        invalid = AckermannControlCommand()
        _set_stamp(invalid.stamp, 2)
        invalid.longitudinal.speed = 2.0
        invalid.longitudinal.acceleration = 0.5
        invalid.lateral.steering_tire_angle = float("nan")
        mux.on_pure_pursuit_cmd(invalid)
        _set_stamp(proof.header.stamp, 2)
        proof.reason = "command_missing_or_nonfinite"
        mux.on_pure_pursuit_tracking_status(proof)
        tick_and_receive_status()
        assert received[-1].trajectory_tracking_usable is False
        assert received[-1].reason == "command_missing_or_nonfinite"

        valid = AckermannControlCommand()
        _set_stamp(valid.stamp, 3)
        valid.longitudinal.speed = 2.0
        valid.longitudinal.acceleration = 0.5
        valid.lateral.steering_tire_angle = 0.1
        mux.on_pure_pursuit_cmd(valid)
        _set_stamp(proof.header.stamp, 3)
        proof.plan_generation = 6
        proof.reason = ""
        mux.on_pure_pursuit_tracking_status(proof)
        tick_and_receive_status()
        assert received[-1].trajectory_tracking_usable is False

        _set_stamp(valid.stamp, 4)
        mux.on_pure_pursuit_cmd(valid)
        _set_stamp(proof.header.stamp, 5)
        proof.plan_generation = 7
        mux.on_pure_pursuit_tracking_status(proof)
        tick_and_receive_status()
        assert received[-1].trajectory_tracking_usable is False
        assert received[-1].reason == "command_stamp_mismatch"

        # The status for stamp 5 arrived before its command. Once the matching
        # command arrives, the exact-stamp cache must reconcile the pair.
        _set_stamp(valid.stamp, 5)
        mux.on_pure_pursuit_cmd(valid)
        tick_and_receive_status()
        assert received[-1].trajectory_tracking_usable is True
        assert received[-1].plan_generation == 7
        assert received[-1].header.stamp.sec == 5

        # The reverse delivery order must also remain fail-closed until the
        # exact matching status arrives; the stamp-5 proof cannot prove cmd 6.
        _set_stamp(valid.stamp, 6)
        mux.on_pure_pursuit_cmd(valid)
        tick_and_receive_status()
        assert received[-1].trajectory_tracking_usable is False
        assert received[-1].reason == "command_stamp_mismatch"

        _set_stamp(proof.header.stamp, 6)
        mux.on_pure_pursuit_tracking_status(proof)
        tick_and_receive_status()
        assert received[-1].trajectory_tracking_usable is True
        assert received[-1].header.stamp.sec == 6
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    ("steering_rad", "expected_usable", "expected_reason"),
    [
        (0.49, True, "ready"),
        (0.50, True, "ready"),
        (
            math.nextafter(0.50, math.inf),
            False,
            "steering_command_exceeds_actuator_limit",
        ),
        (
            math.nextafter(-0.50, -math.inf),
            False,
            "steering_command_exceeds_actuator_limit",
        ),
        (0.500001, False, "steering_command_exceeds_actuator_limit"),
        (-0.500001, False, "steering_command_exceeds_actuator_limit"),
    ],
)
def test_tracking_usable_requires_raw_steering_within_absolute_limit(
    steering_rad, expected_usable, expected_reason
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=false",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
            "-p",
            "max_steering_angle_rad:=0.6",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.5",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_steering_limit_status_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )

    plan = _valid_plan(stamp_sec=1, generation=7)
    mux.on_overtake_plan(plan)
    constraint = _constraint(
        stamp_sec=1,
        constraint_generation=1,
        plan_generation=7,
        speed_limit_mps=3.0,
        stop_requested=False,
        release_authorized=True,
    )
    mux.on_safety_constraint(constraint)

    command = AckermannControlCommand()
    _set_stamp(command.stamp, 1)
    command.longitudinal.speed = 2.0
    command.longitudinal.acceleration = 0.5
    command.lateral.steering_tire_angle = steering_rad
    mux.on_pure_pursuit_cmd(command)

    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 1)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 7
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(commands) and bool(statuses))
        assert statuses[-1].trajectory_tracking_usable is expected_usable
        assert statuses[-1].reason == expected_reason
        if expected_usable:
            assert commands[-1].longitudinal.speed == pytest.approx(2.0)
            assert commands[-1].lateral.steering_tire_angle == pytest.approx(
                steering_rad
            )
        else:
            assert commands[-1].longitudinal.speed == pytest.approx(0.0)
            assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_tracking_reserve_excess_holds_only_last_verified_steering():
    """The dev3 corner boundary must brake without publishing new lateral authority."""
    tracking_limit_rad = 0.35
    actuator_hard_limit_rad = 0.3665191429188092
    dev3_raw_steering_rad = -0.213792
    steering_gain = 1.639
    dev3_post_gain_steering_rad = dev3_raw_steering_rad * steering_gain
    assert abs(dev3_post_gain_steering_rad) > tracking_limit_rad
    assert abs(dev3_post_gain_steering_rad) < actuator_hard_limit_rad

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
            "-p",
            f"max_steering_angle_rad:={actuator_hard_limit_rad}",
            "-p",
            f"tracking_usable_max_steering_angle_rad:={tracking_limit_rad}",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("hybrid_control_mux_free_run_reserve_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)

    def publish_pp_sample(*, stamp_sec: int, generation: int, steering_rad: float):
        command = AckermannControlCommand()
        _set_stamp(command.stamp, stamp_sec)
        command.longitudinal.speed = 2.0
        command.longitudinal.acceleration = 0.5
        command.lateral.steering_tire_angle = steering_rad
        mux.on_pure_pursuit_cmd(command)
        proof = ControllerTrackingStatus()
        _set_stamp(proof.header.stamp, stamp_sec)
        proof.header.frame_id = "base_link"
        proof.plan_generation = generation
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        mux.on_pure_pursuit_tracking_status(proof)
        _bind_envelope_from_legacy(
            mux, command, proof, sequence=stamp_sec * 2
        )

    try:
        plan = _valid_plan(stamp_sec=10, generation=7)
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=1,
                plan_generation=7,
                speed_limit_mps=3.0,
                stop_requested=False,
                release_authorized=True,
            )
        )

        verified_steering_rad = -0.34
        publish_pp_sample(
            stamp_sec=20,
            generation=7,
            steering_rad=verified_steering_rad,
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(commands) >= 1 and len(statuses) >= 1)
        assert commands[-1].longitudinal.speed == pytest.approx(2.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(
            verified_steering_rad
        )
        assert statuses[-1].trajectory_tracking_usable is True
        assert mux.last_verified_baseline_free_run_steering_rad == pytest.approx(
            verified_steering_rad
        )

        publish_pp_sample(
            stamp_sec=21,
            generation=7,
            steering_rad=dev3_post_gain_steering_rad,
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(commands) >= 2 and len(statuses) >= 2)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration < 0.0
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(
            verified_steering_rad
        )
        assert statuses[-1].trajectory_tracking_usable is False
        assert (
            statuses[-1].reason
            == "baseline_tracking_reserve_longitudinal_stop"
        )

        # A hard-limit violation is a cache barrier.  Returning to the narrow
        # soft/hard corridor must not resurrect steering published before the
        # violation.
        publish_pp_sample(
            stamp_sec=22,
            generation=7,
            steering_rad=-(actuator_hard_limit_rad + 0.01),
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(commands) >= 3 and len(statuses) >= 3)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert mux.last_verified_baseline_free_run_steering_rad is None
        publish_pp_sample(
            stamp_sec=23,
            generation=7,
            steering_rad=dev3_post_gain_steering_rad,
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(commands) >= 4 and len(statuses) >= 4)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)

        # Re-prime after the safety barrier before checking bounded expiry.
        publish_pp_sample(
            stamp_sec=24,
            generation=7,
            steering_rad=verified_steering_rad,
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(commands) >= 5 and len(statuses) >= 5)

        # An expired baseline reference cannot turn into persistent lateral
        # authority.  The same over-reserve input must return to canonical
        # zero-steer STOP.
        mux.last_verified_baseline_free_run_steering_time_sec = (
            mux.now_sec() - mux.lateral_stop_steering_hold_timeout_sec - 0.01
        )
        publish_pp_sample(
            stamp_sec=25,
            generation=7,
            steering_rad=dev3_post_gain_steering_rad,
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(commands) >= 6 and len(statuses) >= 6)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert (
            statuses[-1].reason
            == "steering_command_exceeds_actuator_limit"
        )

        # Re-prime the baseline cache, then change to a typed PASSING plan.
        # The baseline reference must be invalidated and must never bridge
        # into lateral maneuver authority.
        publish_pp_sample(
            stamp_sec=26,
            generation=7,
            steering_rad=verified_steering_rad,
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(commands) >= 7 and len(statuses) >= 7)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(
            verified_steering_rad
        )
        passing_plan = _authorized_attack_follow_plan(
            stamp_sec=11, generation=8, pass_direction=-1
        )
        passing_plan.phase = OvertakePlan.PASSING
        mux.on_overtake_plan(passing_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=2,
                plan_generation=8,
                speed_limit_mps=3.0,
                stop_requested=False,
                release_authorized=True,
            )
        )
        publish_pp_sample(
            stamp_sec=27,
            generation=8,
            steering_rad=dev3_post_gain_steering_rad,
        )
        mux.on_timer()
        assert _spin_until(executor, lambda: len(commands) >= 8 and len(statuses) >= 8)
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert mux.last_verified_baseline_free_run_steering_rad is None
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_lateral_stop_tracking_rejects_absolute_steering_limit_excess():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=1.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "race_arm_required:=false",
            "-p",
            "max_steering_angle_rad:=0.6",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.5",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    mux.last_verified_lateral_stop_steering_rad = 0.2
    mux.last_verified_lateral_stop_steering_time_sec = mux.now_sec()
    mux.last_verified_lateral_stop_plan_generation = 6
    observer = Node("hybrid_control_mux_lateral_stop_limit_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )

    plan = _authorized_lateral_stop_plan(stamp_sec=10, generation=7)
    mux.on_overtake_plan(plan)
    mux.on_safety_constraint(
        _constraint(
            stamp_sec=10,
            constraint_generation=1,
            plan_generation=7,
            speed_limit_mps=0.5,
            stop_requested=True,
            release_authorized=False,
        )
    )
    command = AckermannControlCommand()
    _set_stamp(command.stamp, 20)
    command.longitudinal.speed = 0.2
    command.longitudinal.acceleration = 0.0
    command.lateral.steering_tire_angle = 0.500001
    mux.on_pure_pursuit_cmd(command)
    proof = ControllerTrackingStatus()
    _set_stamp(proof.header.stamp, 20)
    proof.header.frame_id = "base_link"
    proof.plan_generation = 7
    proof.pp_command_fresh = True
    proof.trajectory_tracking_usable = True
    proof.command_age_sec = 0.0
    mux.on_pure_pursuit_tracking_status(proof)
    _bind_envelope_from_legacy(mux, command, proof, sequence=1)

    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        mux.on_timer()
        assert _spin_until(executor, lambda: bool(commands) and bool(statuses))
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].reason == "steering_command_exceeds_actuator_limit"
        assert mux.last_verified_lateral_stop_steering_rad == pytest.approx(0.2)
        assert mux.last_verified_lateral_stop_plan_generation == 6
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "limit_overrides",
    [
        ["-p", "max_steering_angle_rad:=0.0"],
        ["-p", "tracking_usable_max_steering_angle_rad:=0.0"],
        [
            "-p",
            "max_steering_angle_rad:=0.5",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.6",
        ],
    ],
)
def test_invalid_absolute_steering_limit_configuration_fails_fast(
    limit_overrides,
):
    rclpy.init(args=["--ros-args", *limit_overrides])
    try:
        with pytest.raises(ValueError):
            HybridControlMuxNode()
    finally:
        rclpy.shutdown()


def test_free_run_live_exact_observer_is_disabled_by_default():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        assert mux.free_run_live_exact_observe_enabled is False
        assert mux.free_run_live_exact_ack is None
        assert mux.free_run_live_exact_ack_valid is False
        assert mux.free_run_live_exact_ack_reason == "disabled"
        assert (
            mux.get_subscriptions_info_by_topic(
                "/input/free_run_execution_ack"
            )
            == []
        )
        assert (
            mux.get_subscriptions_info_by_topic("/input/free_run_source_key")
            == []
        )
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_live_exact_observer_joins_plan_constraint_and_ack_without_authority():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "max_steering_angle_rad:=0.3665191429188092",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.35",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan = _valid_plan(stamp_sec=10, generation=9)
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        ack = _finalize_free_run_ack(
            mux, plan, _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=4,
                plan_generation=9,
            )
        )
        mux.on_free_run_source_key(_source_key_from_ack(ack))
        mux.on_free_run_execution_ack(ack)

        assert mux.free_run_live_exact_ack_valid is True, (
            mux.free_run_live_exact_ack_reason
        )
        assert mux.free_run_live_exact_ack_reason == "complete_observe_only"
        assert mux.motion_authority_grant_active is False
        assert mux.last_verified_baseline_free_run_steering_rad is None

        mutated_plan = _valid_plan(stamp_sec=11, generation=9)
        mutated_plan.race_arm_epoch = 1
        mutated_plan.planner_instance_id = 101
        _bind_free_run_plan_identity(mux, mutated_plan)
        mux.on_overtake_plan(mutated_plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=11,
                constraint_generation=5,
                plan_generation=9,
            )
        )
        mutated_ack = _free_run_execution_ack(stamp_sec=11, generation=9)
        mutated_ack.plan_key.canonical_plan_payload_sha256 = [0x12] * 32
        mux.on_free_run_execution_ack(mutated_ack)

        assert mux.free_run_live_exact_ack_valid is False
        assert mux.free_run_live_exact_ack_reason == "digest_missing_or_mutated"
        assert mux.motion_authority_grant_active is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_live_exact_observer_requires_independent_current_source_key():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "max_steering_angle_rad:=0.3665191429188092",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan = _valid_plan(stamp_sec=10, generation=9)
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        ack = _finalize_free_run_ack(
            mux, plan, _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=4,
                plan_generation=9,
            )
        )
        mux.on_free_run_execution_ack(ack)

        assert mux.free_run_live_exact_ack_valid is False
        assert (
            mux.free_run_live_exact_ack_reason
            == "current_source_key_mismatch"
        )
        assert mux.motion_authority_grant_active is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_live_exact_observer_recomputes_planner_proposal_digest():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "max_steering_angle_rad:=0.3665191429188092",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan = _valid_plan(stamp_sec=10, generation=9)
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        ack = _finalize_free_run_ack(
            mux, plan, _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        plan.free_run_canonical_payload_sha256[0] ^= 0x01
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=4,
                plan_generation=9,
            )
        )
        mux.on_free_run_source_key(_source_key_from_ack(ack))
        mux.on_free_run_execution_ack(ack)

        assert mux.free_run_live_exact_ack_valid is False
        assert (
            mux.free_run_live_exact_ack_reason
            == "planner_proposal_digest_mismatch"
        )
        assert mux.motion_authority_grant_active is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_live_exact_observer_expires_without_new_callbacks_on_timer():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "max_steering_angle_rad:=0.3665191429188092",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan = _valid_plan(stamp_sec=10, generation=9)
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        ack = _finalize_free_run_ack(
            mux, plan, _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=4,
                plan_generation=9,
            )
        )
        mux.on_free_run_source_key(_source_key_from_ack(ack))
        mux.on_free_run_execution_ack(ack)
        assert mux.free_run_live_exact_ack_valid is True

        expired_time = (
            mux.now_sec()
            - mux.free_run_live_exact_evidence_timeout_sec
            - 0.01
        )
        mux.free_run_live_exact_ack_time_sec = expired_time
        mux.free_run_live_exact_source_key_time_sec = expired_time
        mux.on_timer()

        assert mux.free_run_live_exact_ack_valid is False
        assert mux.free_run_live_exact_ack_reason == "live_evidence_stale"
        assert mux.motion_authority_grant_active is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_live_exact_observer_rejects_source_rollover_and_mutation():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "max_steering_angle_rad:=0.3665191429188092",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan = _valid_plan(stamp_sec=10, generation=9)
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        ack = _finalize_free_run_ack(
            mux, plan, _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        source_key = _source_key_from_ack(ack)
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=4,
                plan_generation=9,
            )
        )
        mux.on_free_run_source_key(source_key)
        mux.on_free_run_execution_ack(ack)
        assert mux.free_run_live_exact_ack_valid is True

        source_rollover = copy.deepcopy(source_key)
        source_rollover.source_generation += 1
        _set_stamp(source_rollover.source_stamp, 11)
        mux.on_free_run_source_key(source_rollover)
        assert mux.free_run_live_exact_ack_valid is False
        assert (
            mux.free_run_live_exact_ack_reason
            == "current_source_key_mismatch"
        )

        mux.on_free_run_source_key(source_key)
        original_receipt = mux.free_run_live_exact_source_key_time_sec
        mutated_source = copy.deepcopy(source_key)
        mutated_source.baseline_reference_sha256[0] ^= 0x01
        mux.on_free_run_source_key(mutated_source)
        assert mux.free_run_live_exact_source_key_valid is False
        assert (
            mux.free_run_live_exact_source_key_reason
            == "duplicate_identity_payload_mutation"
        )
        assert mux.free_run_live_exact_source_key_time_sec == original_receipt
        assert mux.motion_authority_grant_active is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_live_exact_observer_taints_same_identity_payload_mutation():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "max_steering_angle_rad:=0.3665191429188092",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.35",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        plan = _valid_plan(stamp_sec=10, generation=9)
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        ack = _finalize_free_run_ack(
            mux, plan, _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=4,
                plan_generation=9,
            )
        )
        mux.on_free_run_source_key(_source_key_from_ack(ack))
        mux.on_free_run_execution_ack(ack)
        assert mux.free_run_live_exact_ack_valid is True
        original_receipt = mux.free_run_live_exact_ack_time_sec

        conflicting_duplicate = copy.deepcopy(ack)
        conflicting_duplicate.output_controller_command.longitudinal.speed = 3.0
        conflicting_digest = mux._free_run_execution_ack_wire_digest(
            conflicting_duplicate
        )
        assert conflicting_digest is not None
        conflicting_duplicate.ack_sha256 = list(conflicting_digest)
        mux.on_free_run_execution_ack(conflicting_duplicate)

        assert mux.free_run_live_exact_ack_valid is False
        assert (
            mux.free_run_live_exact_ack_reason
            == "duplicate_identity_payload_mutation"
        )
        assert mux.free_run_live_exact_ack_time_sec == original_receipt
        assert (1, 101, 9) in mux.free_run_live_exact_tainted_generations
        assert mux.motion_authority_grant_active is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "force_final_limiter_mismatch,generation_delta,force_initial_limiter_mismatch",
    [(False, 1, False), (True, 1, False), (False, 2, False), (False, 1, True)],
)
def test_free_run_live_exact_ack_binds_selected_envelope_and_records_published_command(
    force_final_limiter_mismatch,
    generation_delta,
    force_initial_limiter_mismatch,
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "primary_source:=pure_pursuit",
            "-p",
            "require_safety_constraint:=true",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "free_run_live_exact_pre_ack_hold_enabled:=true",
            "-p",
            "race_arm_required:=false",
            "-p",
            "pure_pursuit_cmd_timeout_sec:=10.0",
            "-p",
            "pure_pursuit_tracking_status_timeout_sec:=10.0",
            "-p",
            "pure_pursuit_envelope_timeout_sec:=10.0",
            "-p",
            "safety_constraint_timeout_sec:=1.0",
            "-p",
            "overtake_plan_timeout_sec:=1.0",
            "-p",
            "control_loop_max_gap_sec:=10.0",
            "-p",
            "ros_clock_stall_timeout_sec:=10.0",
            "-p",
            "max_steering_angle_rad:=0.3665191429188092",
            "-p",
            "tracking_usable_max_steering_angle_rad:=0.35",
            "-p",
            "enable_steering_rate_limit:=false",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("free_run_live_exact_record_observer")
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        plan = _valid_plan(stamp_sec=10, generation=9)
        plan.race_arm_epoch = 1
        plan.planner_instance_id = 101
        ack = _finalize_free_run_ack(
            mux, plan, _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        mux.on_overtake_plan(plan)
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=10,
                constraint_generation=4,
                plan_generation=9,
            )
        )
        mux.on_free_run_source_key(_source_key_from_ack(ack))
        mux.on_free_run_execution_ack(ack)
        mux.on_pure_pursuit_cmd(copy.deepcopy(ack.output_controller_command))
        proof = ControllerTrackingStatus()
        proof.header = copy.deepcopy(ack.header)
        proof.plan_generation = 9
        proof.mpc_horizon_usable = True
        proof.pp_command_fresh = True
        proof.trajectory_tracking_usable = True
        proof.command_age_sec = 0.0
        proof.reason = "ready"
        mux.on_pure_pursuit_tracking_status(proof)
        _set_mux_ros_time_for_envelope(mux, 10)
        for sequence in (6, 7):
            envelope = _command_envelope(
                stamp_sec=10,
                producer_instance_id=201,
                command_sequence=sequence,
                plan_generation=9,
            )
            envelope.command = copy.deepcopy(ack.output_controller_command)
            mux.on_pure_pursuit_command_envelope(envelope)

        if force_initial_limiter_mismatch:
            original_limit_steering = mux._limit_steering

            def force_initial_mismatch(cmd, source, now_sec):
                result = original_limit_steering(cmd, source, now_sec)
                if source == "pure_pursuit":
                    result.limited_steering_rad = 0.0
                    result.rate_limited = True
                    cmd.lateral.steering_tire_angle = 0.0
                return result

            mux._limit_steering = force_initial_mismatch
        mux.on_timer()
        assert _spin_until(
            executor, lambda: bool(commands) and bool(statuses)
        )
        if force_initial_limiter_mismatch:
            assert mux.free_run_live_exact_selected_envelope_valid is False
            assert commands[-1].longitudinal.speed == pytest.approx(0.0)
            assert commands[-1].longitudinal.acceleration < 0.0
            assert commands[-1].lateral.steering_tire_angle == pytest.approx(
                0.0
            )
            assert statuses[-1].trajectory_tracking_usable is False
            assert statuses[-1].reason == "final_source_not_pure_pursuit"
            assert mux.motion_authority_grant_active is False
            assert mux.free_run_live_exact_published_record is None
            assert (
                mux.free_run_live_exact_record_reason
                == "free_run_final_limiter_mismatch"
            )
            return
        assert mux.free_run_live_exact_selected_envelope_valid is True, (
            mux.free_run_live_exact_selected_envelope_reason
        )
        assert mux.free_run_live_exact_selected_envelope_identity == (
            201,
            7,
            10_000_000_000,
            9,
        )
        record = mux.free_run_live_exact_published_record
        assert record is not None
        assert record.envelope_identity == (201, 7, 10_000_000_000, 9)
        assert record.published_steering_rad == pytest.approx(
            commands[-1].lateral.steering_tire_angle
        )
        assert record.final_command_cdr == bytes(
            serialize_message(commands[-1])
        )
        assert record.final_command_sha256 == hashlib.sha256(
            record.final_command_cdr
        ).digest()

        selected_sample = mux._select_pp_motion_sample(
            tracking_plan_generation=9,
            now_sec=mux.now_sec(),
            ros_clock_stalled=False,
        )
        mutations = []
        producer_mismatch = copy.deepcopy(ack)
        producer_mismatch.source_key.baseline_instance_id = 202
        producer_mismatch.source_key.controller_instance_id = 202
        mutations.append(producer_mismatch)
        sequence_mismatch = copy.deepcopy(ack)
        sequence_mismatch.controller_sequence = 8
        mutations.append(sequence_mismatch)
        stamp_mismatch = copy.deepcopy(ack)
        stamp_mismatch.controller_command_stamp.nanosec = 1
        mutations.append(stamp_mismatch)
        generation_mismatch = copy.deepcopy(ack)
        generation_mismatch.plan_key.plan_generation = 10
        mutations.append(generation_mismatch)
        command_mismatch = copy.deepcopy(ack)
        command_mismatch.output_controller_command.longitudinal.speed = 2.5
        command_mismatch.output_controller_command_sha256 = list(
            mux._canonical_controller_command_digest(
                command_mismatch.output_controller_command
            )
        )
        mutations.append(command_mismatch)
        digest_mismatch = copy.deepcopy(ack)
        digest_mismatch.output_controller_command_sha256[0] ^= 0x01
        mutations.append(digest_mismatch)
        for mutated_ack in mutations:
            mux.free_run_live_exact_ack = mutated_ack
            mux.free_run_live_exact_ack_valid = True
            mux.free_run_live_exact_published_record = record
            assert (
                mux._refresh_free_run_selected_envelope_join(
                    selected_sample,
                    tracking_plan_generation=9,
                    now_sec=mux.now_sec(),
                )
                is None
            )
            assert mux.free_run_live_exact_published_record is None

        mux.free_run_live_exact_ack = ack
        mux.free_run_live_exact_ack_valid = True
        mux.free_run_live_exact_published_record = record
        # Provenance may be older than the 0.12s live-evidence lease while
        # still remaining inside the formal trajectory bound.
        record = FreeRunPublishedCommandRecord(
            **{
                **record.__dict__,
                "publish_steady_time_sec": mux.now_sec() - 0.9,
            }
        )
        mux.free_run_live_exact_published_record = record

        # Latest Dev3 fixture: the reference publisher delivers the exact same
        # canonical trajectory again.  Only source generation/stamp advance.
        source_successor = _source_key_from_ack(ack)
        source_successor.source_generation += 1
        source_successor.source_stamp.nanosec = 50_000_000
        mux.on_free_run_source_key(source_successor)
        assert mux.free_run_live_exact_published_record is record
        pending_source = mux.free_run_pending_source
        assert pending_source is not None
        assert pending_source.identity == (
            int(source_successor.baseline_instance_id),
            int(source_successor.controller_instance_id),
            int(source_successor.source_generation),
            mux._stamp_ns(source_successor.source_stamp),
        )
        assert pending_source.source_key == source_successor
        assert record.source_identity != pending_source.identity
        assert mux.free_run_source_gap_lease is not None
        mux.on_timer()
        assert _spin_until(
            executor, lambda: len(commands) >= 2 and len(statuses) >= 2
        )
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration < 0.0
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(
            record.published_steering_rad
        )
        assert (
            commands[-1].lateral.steering_tire_rotation_rate
            == pytest.approx(0.0)
        )
        assert statuses[-1].trajectory_tracking_usable is False
        assert statuses[-1].reason == (
            "free_run_source_delivery_gap_zero_speed_steering_hold"
        )
        assert mux.motion_authority_grant_active is False
        assert mux.free_run_live_exact_published_record is record
        assert mux.free_run_source_gap_hold_episode_count == 1

        if not force_final_limiter_mismatch and generation_delta == 1:
            # Isolated expiry assertion: terminate this parameter case here so
            # the later N+1 transport fixture retains its original state.
            lease = mux.free_run_source_gap_lease
            assert lease is not None
            mux.free_run_source_gap_lease = FreeRunSourceGapLease(
                provenance_record=lease.provenance_record,
                successor_source_identity=lease.successor_source_identity,
                acquired_steady_time_sec=(
                    mux.now_sec()
                    - mux.free_run_live_exact_evidence_timeout_sec
                    - 0.01
                ),
            )
            previous_command_count = len(commands)
            mux.on_timer()
            assert _spin_until(
                executor, lambda: len(commands) > previous_command_count
            )
            assert mux.last_source == "stop"
            assert commands[-1].longitudinal.speed == pytest.approx(0.0)
            assert commands[-1].longitudinal.acceleration < 0.0
            assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
            assert commands[-1].lateral.steering_tire_rotation_rate == pytest.approx(0.0)
            assert statuses[-1].trajectory_tracking_usable is False
            assert mux.motion_authority_grant_active is False
            assert mux.free_run_source_gap_lease_expiry_count == 1
            assert mux.free_run_pending_source is None
            assert mux.free_run_source_provenance_record is None
            return

        # The exact ACK for the pending SourceKey atomically replaces the old
        # immutable record.  The old tuple is never rewritten in place.
        successor_ack = copy.deepcopy(ack)
        successor_ack.source_key = copy.deepcopy(source_successor)
        successor_ack.base_geometry.source_stamp = copy.deepcopy(
            source_successor.source_stamp
        )
        successor_ack.applied_geometry.source_stamp = copy.deepcopy(
            source_successor.source_stamp
        )
        successor_digest = mux._free_run_execution_ack_wire_digest(
            successor_ack
        )
        assert successor_digest is not None
        successor_ack.ack_sha256 = list(successor_digest)
        mux.on_free_run_execution_ack(successor_ack)
        assert mux.free_run_live_exact_ack_valid is True, (
            mux.free_run_live_exact_ack_reason
        )
        assert mux.free_run_pending_source is pending_source
        previous_command_count = len(commands)
        previous_status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > previous_command_count
            and len(statuses) > previous_status_count,
        )
        assert commands[-1].longitudinal.speed > 0.0
        transitioned_record = mux.free_run_live_exact_published_record
        assert transitioned_record is not None
        assert transitioned_record is not record
        assert transitioned_record.source_identity == (
            int(source_successor.baseline_instance_id),
            int(source_successor.controller_instance_id),
            int(source_successor.source_generation),
            mux._stamp_ns(source_successor.source_stamp),
        )
        assert record.source_identity != transitioned_record.source_identity
        assert mux.free_run_source_provenance_record is transitioned_record
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_pending_source is None
        assert mux.free_run_source_gap_exact_promotion_count == 1
        record = transitioned_record
        ack = successor_ack

        # D1 transport seam: plan/constraint advance to N+1 while the only
        # fresh PP envelope/proof is the exact, already-published N sample.
        next_plan = _valid_plan(stamp_sec=10, generation=9 + generation_delta)
        next_plan.header.stamp.nanosec = 100_000_000
        next_plan.race_arm_epoch = 1
        next_plan.planner_instance_id = 101
        _bind_free_run_plan_identity(mux, next_plan)
        mux.on_overtake_plan(next_plan)
        next_constraint = _constraint(
            stamp_sec=10,
            constraint_generation=5,
            plan_generation=9 + generation_delta,
        )
        next_constraint.header.stamp.nanosec = 100_000_000
        mux.on_safety_constraint(next_constraint)
        if generation_delta == 1:
            # N+1 plan/constraint直後にselected PP sampleが1周期欠けても、
            # その周期はzero-steerのまま旧recordだけを保持する。次のfreshな
            # N envelope/proofが到着した周期に限りbounded holdを評価できる。
            assert (
                mux._refresh_free_run_selected_envelope_join(
                    None,
                    tracking_plan_generation=10,
                    now_sec=mux.now_sec(),
                )
                is None
            )
            assert (
                mux.free_run_live_exact_selected_envelope_reason
                == "forward_pre_ack_transition"
            )
            assert mux.free_run_live_exact_published_record is record
        if force_final_limiter_mismatch:
            original_limit_steering = mux._limit_steering

            def force_mismatch_after_hold_eligibility(cmd, source, now_sec):
                result = original_limit_steering(cmd, source, now_sec)
                if source == "pure_pursuit":
                    result.limited_steering_rad = 0.0
                    result.rate_limited = True
                    cmd.lateral.steering_tire_angle = 0.0
                return result

            mux._limit_steering = force_mismatch_after_hold_eligibility
        previous_command_count = len(commands)
        previous_status_count = len(statuses)
        mux.on_timer()
        assert _spin_until(
            executor,
            lambda: len(commands) > previous_command_count
            and len(statuses) > previous_status_count,
        )
        if force_final_limiter_mismatch or generation_delta != 1:
            assert commands[-1].longitudinal.speed == pytest.approx(0.0)
            assert commands[-1].longitudinal.acceleration < 0.0
            assert commands[-1].lateral.steering_tire_angle == pytest.approx(
                0.0
            )
            assert statuses[-1].trajectory_tracking_usable is False
            assert mux.motion_authority_grant_active is False
            assert mux.free_run_live_exact_published_record is None
            if force_final_limiter_mismatch:
                assert (
                    mux.free_run_live_exact_record_reason
                    == "pre_ack_hold_final_limiter_mismatch"
                )
            else:
                assert (
                    mux.free_run_live_exact_record_reason
                    == "plan_semantic_identity_changed"
                )
            return

        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration < 0.0
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(
            record.published_steering_rad
        )
        assert statuses[-1].trajectory_tracking_usable is False
        assert (
            statuses[-1].reason
            == "normal_delivery_gap_zero_speed_steering_hold"
        )
        assert mux.motion_authority_grant_active is False
        assert mux.free_run_live_exact_published_record is record

        mux.free_run_live_exact_source_key_time_sec = (
            mux.now_sec()
            - mux.free_run_live_exact_evidence_timeout_sec
            - 0.01
        )
        previous_command_count = len(commands)
        mux.on_timer()
        assert _spin_until(
            executor, lambda: len(commands) > previous_command_count
        )
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        # ACK/plan/constraint/PP evidence still expires at 0.12s, but that
        # must only revoke the gap authority.  Matching provenance remains
        # available until its independent formal-trajectory bound (1.5s).
        assert mux.free_run_source_provenance_record is record
        assert mux.free_run_source_gap_lease is None

        mux.on_external_safety_status(SafetyStopStatus(valid=False))
        assert mux.free_run_live_exact_published_record is None
        assert mux.free_run_live_exact_record_reason == "external_safety_stop"
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_source_direct_successor_rejects_nonsemantic_or_nonmonotonic_updates():
    rclpy.init()
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        source = _source_key_from_ack(
            _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        record = _published_record_for_source(mux, source)

        valid_successor = copy.deepcopy(source)
        valid_successor.source_generation += 1
        valid_successor.source_stamp.nanosec = 50_000_000
        assert mux._free_run_source_is_direct_semantic_successor(
            valid_successor, record
        )

        invalid_updates = {}
        same_generation = copy.deepcopy(valid_successor)
        same_generation.source_generation = source.source_generation
        invalid_updates["same_generation"] = same_generation
        skipped_generation = copy.deepcopy(valid_successor)
        skipped_generation.source_generation += 1
        invalid_updates["skipped_generation"] = skipped_generation
        equal_stamp = copy.deepcopy(valid_successor)
        equal_stamp.source_stamp = copy.deepcopy(source.source_stamp)
        invalid_updates["equal_stamp"] = equal_stamp
        regressed_stamp = copy.deepcopy(valid_successor)
        regressed_stamp.source_stamp.sec = 9
        invalid_updates["regressed_stamp"] = regressed_stamp
        changed_baseline = copy.deepcopy(valid_successor)
        changed_baseline.baseline_reference_sha256[0] ^= 0xFF
        invalid_updates["changed_baseline_digest"] = changed_baseline
        changed_controller = copy.deepcopy(valid_successor)
        changed_controller.controller_implementation_sha256[0] ^= 0xFF
        invalid_updates["changed_controller_digest"] = changed_controller
        changed_config = copy.deepcopy(valid_successor)
        changed_config.controller_config_sha256[0] ^= 0xFF
        invalid_updates["changed_config_digest"] = changed_config
        changed_instance = copy.deepcopy(valid_successor)
        changed_instance.controller_instance_id += 1
        invalid_updates["changed_controller_instance"] = changed_instance
        changed_point_count = copy.deepcopy(valid_successor)
        changed_point_count.original_point_count += 1
        invalid_updates["changed_point_count"] = changed_point_count

        for case_name, update in invalid_updates.items():
            assert not mux._free_run_source_is_direct_semantic_successor(
                update, record
            ), case_name
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "fault",
    ["regression", "payload_mutation", "generation_skip"],
)
def test_pending_free_run_source_fault_revokes_pending_and_old_record(fault):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        source = _source_key_from_ack(
            _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        mux.on_free_run_source_key(source)
        record = _published_record_for_source(mux, source)
        mux.free_run_source_provenance_record = record

        successor = copy.deepcopy(source)
        successor.source_generation += 1
        successor.source_stamp.nanosec = 50_000_000
        mux.on_free_run_source_key(successor)
        assert mux.free_run_pending_source is not None
        assert mux.free_run_source_gap_lease is not None
        assert mux.free_run_source_provenance_record is record

        if fault == "regression":
            invalid = copy.deepcopy(source)
        elif fault == "payload_mutation":
            invalid = copy.deepcopy(successor)
            invalid.baseline_reference_sha256[0] ^= 0x01
        else:
            invalid = copy.deepcopy(successor)
            invalid.source_generation += 1
            invalid.source_stamp.nanosec = 100_000_000
        mux.on_free_run_source_key(invalid)

        assert mux.free_run_pending_source is None
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_source_provenance_record is None
        assert mux.motion_authority_grant_active is False
    finally:
        mux.destroy_node()
        rclpy.shutdown()


def test_aged_free_run_source_provenance_mints_only_a_bounded_gap_lease():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    try:
        source = _source_key_from_ack(
            _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        mux.on_free_run_source_key(source)
        record = _published_record_for_source(mux, source)
        record = FreeRunPublishedCommandRecord(
            **{
                **record.__dict__,
                "publish_steady_time_sec": mux.now_sec() - 0.9,
            }
        )
        mux.free_run_source_provenance_record = record

        successor = copy.deepcopy(source)
        successor.source_generation += 1
        successor.source_stamp.nanosec = 50_000_000
        mux.on_free_run_source_key(successor)

        assert mux.free_run_source_provenance_record is record
        lease = mux.free_run_source_gap_lease
        assert isinstance(lease, FreeRunSourceGapLease)
        assert lease.provenance_record is record
        assert lease.successor_source_identity == (
            int(successor.baseline_instance_id),
            int(successor.controller_instance_id),
            int(successor.source_generation),
            mux._stamp_ns(successor.source_stamp),
        )
        assert mux.free_run_live_exact_record_reason == (
            "source_direct_semantic_successor_gap_lease"
        )

        expired = FreeRunPublishedCommandRecord(
            **{
                **record.__dict__,
                "publish_steady_time_sec": mux.now_sec() - 1.51,
            }
        )
        mux.free_run_source_provenance_record = expired
        mux.free_run_source_gap_lease = None
        later_successor = copy.deepcopy(successor)
        later_successor.source_generation += 1
        later_successor.source_stamp.nanosec = 100_000_000
        mux.on_free_run_source_key(later_successor)
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_source_provenance_record is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "fault", ["deadline", "ros_clock_stall", "external_stop", "finish"]
)
def test_free_run_gap_lease_faults_clear_provenance_and_publish_canonical_stop(
    fault,
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
            "-p",
            "race_arm_required:=false",
            "-p",
            "control_loop_max_gap_sec:=0.01",
            "-p",
            "ros_clock_stall_timeout_sec:=0.01",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    observer = Node("free_run_gap_lease_fault_observer_" + fault)
    commands = []
    statuses = []
    observer.create_subscription(
        AckermannControlCommand,
        "output/control_cmd",
        lambda msg: commands.append(msg),
        10,
    )
    observer.create_subscription(
        ControllerTrackingStatus,
        "output/controller_tracking_status",
        lambda msg: statuses.append(msg),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(mux)
    executor.add_node(observer)
    try:
        source = _source_key_from_ack(
            _free_run_execution_ack(stamp_sec=10, generation=9)
        )
        record = _published_record_for_source(mux, source)
        mux.free_run_source_provenance_record = record
        mux.free_run_source_gap_lease = FreeRunSourceGapLease(
            provenance_record=record,
            successor_source_identity=(1, 1, 2, 10_050_000_000),
            acquired_steady_time_sec=mux.now_sec(),
        )
        now_sec = mux.now_sec()
        if fault == "deadline":
            mux.control_loop_watchdog._last_tick_sec = now_sec - 0.02
        elif fault == "ros_clock_stall":
            _set_mux_ros_time_for_envelope(mux, 10)
            ros_time_ns = int(mux.get_clock().now().nanoseconds)
            mux.ros_clock_watchdog._maximum_ros_time_ns = ros_time_ns
            mux.ros_clock_watchdog._last_progress_steady_sec = now_sec - 0.02
        elif fault == "external_stop":
            mux.on_external_safety_status(SafetyStopStatus(valid=False))
        else:
            # Finish has no frozen terminal steering in this active gap lease
            # fixture, so its timer command must also be canonical zero steer.
            mux.on_awsim_state(String(data="Ready"))
            mux.on_race_armed(Bool(data=True))
            mux.on_awsim_state(String(data="Start"))
            mux.on_awsim_state(String(data="Finish"))

        mux.on_timer()
        assert _spin_until(
            executor, lambda: bool(commands) and bool(statuses)
        )
        assert mux.free_run_source_provenance_record is None
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_pending_source is None
        assert commands[-1].longitudinal.speed == pytest.approx(0.0)
        assert commands[-1].longitudinal.acceleration < 0.0
        assert commands[-1].lateral.steering_tire_angle == pytest.approx(0.0)
        assert commands[-1].lateral.steering_tire_rotation_rate == pytest.approx(0.0)
        assert statuses[-1].trajectory_tracking_usable is False
        assert mux.motion_authority_grant_active is False
    finally:
        executor.remove_node(observer)
        executor.remove_node(mux)
        observer.destroy_node()
        mux.destroy_node()
        rclpy.shutdown()


def test_free_run_live_exact_record_callback_barriers_clear_synchronously():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "free_run_live_exact_observe_enabled:=true",
        ]
    )
    mux = HybridControlMuxNode()
    mux.timer.cancel()
    marker = object()
    try:
        mux.free_run_live_exact_published_record = marker
        mux.free_run_source_gap_lease = marker
        mux.free_run_pending_source = marker
        mux.on_external_safety_status(SafetyStopStatus(valid=False))
        assert mux.free_run_live_exact_published_record is None
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_pending_source is None

        mux.free_run_live_exact_published_record = marker
        mux.free_run_source_gap_lease = marker
        mux.free_run_pending_source = marker
        mux.on_race_armed(Bool(data=False))
        assert mux.free_run_live_exact_published_record is None
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_pending_source is None

        source = FreeRunSourceKey()
        source.canonical_algorithm_version = 1
        source.baseline_instance_id = 1
        source.controller_instance_id = 1
        source.source_generation = 1
        source.original_point_count = 2
        source.baseline_reference_sha256 = [1] * 32
        source.controller_implementation_sha256 = [2] * 32
        source.controller_config_sha256 = [3] * 32
        mux.on_free_run_source_key(source)
        record = _published_record_for_source(mux, source)
        mux.free_run_live_exact_published_record = record
        mux.free_run_source_gap_lease = marker
        rolled_source = copy.deepcopy(source)
        rolled_source.source_generation = 2
        rolled_source.source_stamp.sec = 1
        mux.on_free_run_source_key(rolled_source)
        assert mux.free_run_live_exact_published_record is record
        assert mux.free_run_pending_source is not None

        skipped_source = copy.deepcopy(rolled_source)
        skipped_source.source_generation = 3
        skipped_source.source_stamp.sec = 2
        mux.on_free_run_source_key(skipped_source)
        assert mux.free_run_live_exact_published_record is None
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_pending_source is None
        assert mux.free_run_live_exact_record_reason == "source_identity_changed"

        mux.free_run_live_exact_ack_identity = (1,) * 10
        mux.free_run_live_exact_published_record = marker
        mux.free_run_source_gap_lease = marker
        mux.free_run_pending_source = marker
        changed_ack = FreeRunExecutionAck()
        changed_ack.plan_key.race_arm_epoch = 1
        changed_ack.plan_key.planner_instance_id = 1
        changed_ack.plan_key.plan_generation = 1
        changed_ack.source_key.baseline_instance_id = 1
        changed_ack.source_key.controller_instance_id = 1
        changed_ack.source_key.source_generation = 1
        changed_ack.controller_sequence = 2
        mux.on_free_run_execution_ack(changed_ack)
        assert mux.free_run_live_exact_published_record is None
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_pending_source is None

        mux.free_run_live_exact_published_record = marker
        mux.free_run_source_gap_lease = marker
        mux.free_run_pending_source = marker
        mux.on_safety_constraint(
            _constraint(
                stamp_sec=20,
                constraint_generation=20,
                plan_generation=20,
                stop_requested=True,
                release_authorized=False,
            )
        )
        assert mux.free_run_live_exact_published_record is None
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_pending_source is None

        mux.finish_stop_latch.armed = True
        mux.finish_stop_latch.race_started = True
        mux.free_run_live_exact_published_record = marker
        mux.free_run_source_gap_lease = marker
        mux.free_run_pending_source = marker
        mux.on_awsim_state(String(data="Finish"))
        assert mux.free_run_live_exact_published_record is None
        assert mux.free_run_source_gap_lease is None
        assert mux.free_run_pending_source is None
    finally:
        mux.destroy_node()
        rclpy.shutdown()
