#!/usr/bin/env python3
"""Two-run normal-load timing and fault parity measurement for AY0 capture."""

from __future__ import annotations

from contextlib import nullcontext
from array import array
import copy
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

from ament_index_python.packages import get_package_prefix
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from builtin_interfaces.msg import Duration
from multi_purpose_mpc_ros_msgs.msg import (
    AuthorizedCartesianTrajectory,
    AuthorizedCartesianTrajectoryV2,
    ControllerCommandEnvelope,
    ControllerExecutionEnvelope,
    ControllerBaseTrajectorySnapshot,
    ControllerTrackingStatus,
    FreeRunExecutionAck,
    FreeRunSourceKey,
    OvertakePlan,
    StateLatticeV2BindingStatus,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray


RING_MAGIC = 0x4330303241593152
HANDSHAKE_MAGIC = 0x4330303241593148
ABI_VERSION = 1
LAYOUT_VERSION = 1
HEADER_SIZE = 320
RECORD_SIZE = 40
CAPACITY = 16384
RING_SIZE = HEADER_SIZE + RECORD_SIZE * CAPACITY
HARNESS_READY = 1
RUN_ID = "c002ay0-pp-runtime"
SESSION_NONCE = 0xA201
OBSERVER_INSTANCE = 0xA202
FIXED_SEC = 100
CAPTURE_COUNT = 32
CAPTURE_WINDOW_COUNT = CAPTURE_COUNT + 1
CAPTURE_TOPICS = frozenset(
    {
        "command",
        "raw_command",
        "tracking",
        "command_envelope",
        "execution_envelope",
    }
)
STALE_INPUT_REASONS = frozenset(
    {
        "missing_odom",
        "missing_trajectory",
        "stale_odom",
        "stale_trajectory",
        "nonfinite_odom",
        "empty_trajectory",
    }
)
PP_WALL_TIMER_PERIOD_MS = 10
INPUT_DISCOVERY_TIMEOUT_SEC = 4.0
BINDING_DISCOVERY_TIMEOUT_SEC = 4.0
RECORDER_DISCOVERY_TIMEOUT_SEC = 10.0
RACE_ARM_DISCOVERY_TIMEOUT_SEC = 2.0
FIXED_DISARM_DURATION_SEC = 0.1
FIXED_ARM_DURATION_SEC = 0.1
OUTPUT_BOUNDARY_TIMEOUT_SEC = 4.0
E2E_DISARM_DURATION_SEC = 0.12
E2E_SAFETY_TIMELINE_LIMIT = 1024
E2E_INPUT_PUBLISH_TICK_LEDGER_LIMIT = 1024
E2E_PROPOSAL_AUDIT_LEDGER_LIMIT = 1024
E2E_OUT_OF_COHORT_SAMPLE_LIMIT = 8
V2_UPTAKE_PLANNER_PRODUCER_INSTANCE_ID = "7101"
V2_UPTAKE_PP_ATTESTATION_PRODUCER_INSTANCE_ID = "7201"
V2_UPTAKE_SESSION_ID = "1"
V2_UPTAKE_TIMEOUT_SEC = 4.0
V2_UPTAKE_MIN_PROPOSAL_COUNT = 6
V2_UPTAKE_COHORT_QUIET_SEC = 0.1
E2E_BINDING_DISPOSITIONS = {
    "exact_current",
    "deferred_predecessor",
    "deferred_unpaired_delivery",
    "replan_required_nearest_advanced",
    "replan_required_nearest_regressed",
    "rejected_source_mutation",
    "rejected_identity",
    "rejected_stale",
    "rejected_invalid_current",
    "rejected_invalid_proposal",
}
E2E_REARM_TIMEOUT_SEC = 2.0
SCHEDULING_EVIDENCE_MAX_SEC = 1.0
SCHEDULING_REANCHOR_TIMEOUT_SEC = 1.0
FIXTURE_INPUT_PERIOD_SEC = PP_WALL_TIMER_PERIOD_MS / 1_000.0
FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK = 64
FIXTURE_DRAIN_SPIN_WAIT_SEC = 0.0005
FIXTURE_DRAIN_DEADLINE_GUARD_SEC = 0.0005
FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY = 8
PREARM_STORAGE_MARGIN_MS = 1_000
PREARM_STORAGE_PHASE_BUDGET_MS = sum(
    round(duration_sec * 1_000)
    for duration_sec in (
        INPUT_DISCOVERY_TIMEOUT_SEC,
        BINDING_DISCOVERY_TIMEOUT_SEC,
        RECORDER_DISCOVERY_TIMEOUT_SEC,
        RACE_ARM_DISCOVERY_TIMEOUT_SEC,
        FIXED_DISARM_DURATION_SEC,
        FIXED_ARM_DURATION_SEC,
        OUTPUT_BOUNDARY_TIMEOUT_SEC,
        E2E_DISARM_DURATION_SEC,
        E2E_REARM_TIMEOUT_SEC,
        SCHEDULING_EVIDENCE_MAX_SEC,
        SCHEDULING_REANCHOR_TIMEOUT_SEC,
    )
)
PREARM_STORAGE_BUDGET_MS = (
    PREARM_STORAGE_PHASE_BUDGET_MS + PREARM_STORAGE_MARGIN_MS
)
PREARM_STORAGE_REQUIRED_MESSAGES = (
    math.ceil(PREARM_STORAGE_BUDGET_MS / PP_WALL_TIMER_PERIOD_MS)
    * len(CAPTURE_TOPICS)
)
PREARM_CAPTURE_CAPACITY_MESSAGES = 16_384
if PREARM_STORAGE_REQUIRED_MESSAGES >= PREARM_CAPTURE_CAPACITY_MESSAGES:
    raise RuntimeError(
        "pre-arm capture capacity does not cover the declared phase budget"
    )
STARTUP_LEDGER_MAX_MESSAGES = PREARM_CAPTURE_CAPACITY_MESSAGES
ALIGNMENT_STAGE_MAX_MESSAGES = PREARM_CAPTURE_CAPACITY_MESSAGES
NONFINITE_FLOAT_TAG = "__c002ay0_nonfinite_float__"
TIMING_DIAGNOSTIC_BLOCKS = 5
TIMING_DIAGNOSTIC_RECORD_COUNT = 1024
TIMING_ATTRIBUTION_RECORD_COUNT = 10_240
TIMING_DIAGNOSTIC_PROBE_MARGIN_NS = 109_788
SIM_CLOCK_QUANTUM_NS = 1_000_000
D2_EGO_X_M = 89633.30679316094
D2_EGO_Y_M = 43131.17568487456
D2_EGO_YAW_RAD = 2.216058185742633
D2_OPPONENTS = (
    ("d1", 89631.15625, 43127.80859375),
    ("d3", 89628.8046875, 43131.41796875),
)
RECORDER_TOPICS = (
    "/clock",
    "/ay0/input/kinematics",
    "/ay0/input/v2x",
    "/ay0/input/trajectory",
    "/ay0/input/overtake_plan",
    "/ay0/input/race_armed",
    "/debug/overtake/state_lattice/authorized_cartesian_trajectory",
    "/debug/overtake/state_lattice/source_binding",
)
DDS_ENVIRONMENT_CONTRACT = {
    "fixture_cyclonedds_uri_present": True,
    "fixture_cyclonedds_uri": "",
    "fixture_verified_before_arm": True,
    "isolated_child_cyclonedds_uri": "",
    "isolated_child_environment_source": (
        "explicit_copy_of_verified_fixture_environment"
    ),
    "isolated_child_preinit_check": "required_before_ready_ack",
    "isolated_child_preinit_verified_by_ready_ack": True,
}


def require_explicit_empty_cyclonedds_uri() -> None:
    if "CYCLONEDDS_URI" not in os.environ:
        raise AssertionError("fixture_cyclonedds_uri_missing")
    if os.environ["CYCLONEDDS_URI"] != "":
        raise AssertionError("fixture_cyclonedds_uri_not_empty")


def require_scheduling_evidence_elapsed(elapsed_sec: float) -> None:
    if (
        not math.isfinite(elapsed_sec)
        or elapsed_sec < 0.0
        or elapsed_sec > SCHEDULING_EVIDENCE_MAX_SEC
    ):
        raise AssertionError(
            "scheduling evidence exceeded "
            f"{SCHEDULING_EVIDENCE_MAX_SEC:.3f}s "
            f"elapsed={elapsed_sec:.6f}s"
        )


def initial_scheduling_evidence(
    scenario: str,
    profile: str,
) -> dict[str, object]:
    return {
        "scenario": scenario,
        "profile": profile,
        "primary_first_false": None,
        "collection_status": {
            "process_snapshot": {
                "status": "not_reached",
                "reason": "scenario_setup_or_execution_not_completed",
            },
            "reanchor": {
                "status": "not_reached",
                "reason": "process_snapshot_not_completed",
            },
        },
    }


def primary_first_false_record(
    error: BaseException,
    drain_statistics: dict[str, object],
) -> dict[str, object]:
    fatal_tick = (
        drain_statistics.get("tick_observation", {}).get("last", {})
        or drain_statistics.get("step_observation", {}).get("last", {})
    )
    return {
        "exception_type": type(error).__name__,
        "message": str(error),
        "fatal_phase": (
            fatal_tick.get("fatal_phase")
            if isinstance(fatal_tick, dict)
            else ""
        ),
        "spin_attempts": (
            fatal_tick.get("spin_attempts")
            if isinstance(fatal_tick, dict)
            else None
        ),
    }


def scheduling_executor_lifecycle_valid(
    scheduling_required: bool,
    lifecycle: dict[str, object],
) -> bool:
    return (
        not scheduling_required
        or (
            lifecycle.get("dedicated") is True
            and lifecycle.get("created") is True
            and lifecycle.get("add_node_attempts") == 1
            and lifecycle.get("add_node_succeeded") is True
            and lifecycle.get("remove_node_succeeded") is True
            and lifecycle.get("shutdown_succeeded") is True
            and not lifecycle.get("cleanup_failure")
        )
    )


def drive_paced_fixture_tick(
    publish_action: object,
    spin_once_action: object,
    *,
    period_sec: float = FIXTURE_INPUT_PERIOD_SEC,
    max_callbacks: int = FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK,
    spin_wait_sec: float = FIXTURE_DRAIN_SPIN_WAIT_SEC,
    deadline_guard_sec: float = FIXTURE_DRAIN_DEADLINE_GUARD_SEC,
    statistics: dict[str, object] | None = None,
    break_on_first_zero_idle: bool = False,
    monotonic: object = time.monotonic,
    sleep: object = time.sleep,
    monotonic_ns: object = time.monotonic_ns,
    thread_time_ns: object = time.thread_time_ns,
) -> int:
    """Publish once, then drain callbacks within one bounded 100 Hz tick."""
    if (
        not math.isfinite(period_sec)
        or period_sec <= 0.0
        or max_callbacks <= 0
        or not math.isfinite(spin_wait_sec)
        or spin_wait_sec <= 0.0
        or not math.isfinite(deadline_guard_sec)
        or deadline_guard_sec < 0.0
        or deadline_guard_sec >= period_sec
    ):
        raise AssertionError("fixture callback drain budget invalid")
    started = monotonic()
    deadline = started + period_sec
    callback_attempts = 0
    blocking_wait_attempts = 0
    zero_wait_attempts = 0
    statistics_updated = False
    tick_observation: dict[str, object] = {
        "tick_index": (
            int(statistics.get("tick_count", 0)) + 1
            if statistics is not None
            else 1
        ),
        "publish": {
            "wall_elapsed_ns": 0,
            "thread_cpu_elapsed_ns": 0,
        },
        "blocking_wait": {
            "count": 0,
            "wall_elapsed_ns_sum": 0,
            "thread_cpu_elapsed_ns_sum": 0,
            "max_wall_elapsed_ns": 0,
            "max_thread_cpu_elapsed_ns": 0,
        },
        "zero_wait": {
            "count": 0,
            "wall_elapsed_ns_sum": 0,
            "thread_cpu_elapsed_ns_sum": 0,
            "max_wall_elapsed_ns": 0,
            "max_thread_cpu_elapsed_ns": 0,
        },
        "callback": {
            "count": 0,
            "wall_elapsed_ns_sum": 0,
            "thread_cpu_elapsed_ns_sum": 0,
            "max_wall_elapsed_ns": 0,
            "max_thread_cpu_elapsed_ns": 0,
        },
        "idle": {
            "count": 0,
            "wall_elapsed_ns_sum": 0,
            "thread_cpu_elapsed_ns_sum": 0,
            "max_wall_elapsed_ns": 0,
            "max_thread_cpu_elapsed_ns": 0,
        },
        "fatal_phase": "",
        "idle_break_enabled": break_on_first_zero_idle,
        "idle_break_count": 0,
        "break_phase": "",
        "deferred_not_proven": True,
    }

    def aggregate_spin(
        observation: object,
        wait_kind: str,
    ) -> None:
        if not isinstance(observation, dict):
            return
        wall_elapsed_ns = int(observation.get("wall_elapsed_ns", 0))
        thread_cpu_elapsed_ns = int(
            observation.get("thread_cpu_elapsed_ns", 0)
        )
        callback_generation_delta = int(
            observation.get("callback_generation_delta", 0)
        )
        bucket_names = [wait_kind]
        if callback_generation_delta == 1:
            bucket_names.append("callback")
        elif callback_generation_delta == 0:
            bucket_names.append("idle")
        for bucket_name in bucket_names:
            bucket = tick_observation[bucket_name]
            assert isinstance(bucket, dict)
            bucket["count"] = int(bucket["count"]) + 1
            bucket["wall_elapsed_ns_sum"] = (
                int(bucket["wall_elapsed_ns_sum"]) + wall_elapsed_ns
            )
            bucket["thread_cpu_elapsed_ns_sum"] = (
                int(bucket["thread_cpu_elapsed_ns_sum"])
                + thread_cpu_elapsed_ns
            )
            bucket["max_wall_elapsed_ns"] = max(
                int(bucket["max_wall_elapsed_ns"]),
                wall_elapsed_ns,
            )
            bucket["max_thread_cpu_elapsed_ns"] = max(
                int(bucket["max_thread_cpu_elapsed_ns"]),
                thread_cpu_elapsed_ns,
            )

    def validated_callback_generation_delta(
        observation: object,
        phase: str,
    ) -> int | None:
        if not break_on_first_zero_idle:
            return None
        if not isinstance(observation, dict):
            raise AssertionError(
                "paced_drain_idle_break_observation_invalid "
                f"phase={phase} reason=not_dict"
            )
        callback_generation_delta = observation.get(
            "callback_generation_delta"
        )
        if type(callback_generation_delta) is not int:
            raise AssertionError(
                "paced_drain_idle_break_observation_invalid "
                f"phase={phase} reason=delta_not_int"
            )
        if callback_generation_delta < 0 or callback_generation_delta > 1:
            raise AssertionError(
                "paced_drain_idle_break_observation_invalid "
                f"phase={phase} callback_generation_delta="
                f"{callback_generation_delta}"
            )
        return callback_generation_delta

    def update_statistics(
        *,
        elapsed_sec: float,
        deadline_missed: bool,
        guard_exhausted: bool = False,
    ) -> None:
        nonlocal statistics_updated
        if statistics_updated:
            return
        statistics_updated = True
        tick_observation["elapsed_sec"] = elapsed_sec
        tick_observation["deadline_missed"] = deadline_missed
        tick_observation["guard_exhausted"] = guard_exhausted
        tick_observation["spin_attempts"] = callback_attempts
        publish_record = tick_observation["publish"]
        blocking_record = tick_observation["blocking_wait"]
        zero_record = tick_observation["zero_wait"]
        assert isinstance(publish_record, dict)
        assert isinstance(blocking_record, dict)
        assert isinstance(zero_record, dict)
        tick_observation["wall_elapsed_ns"] = sum(
            int(record["wall_elapsed_ns"])
            if "wall_elapsed_ns" in record
            else int(record["wall_elapsed_ns_sum"])
            for record in (
                publish_record,
                blocking_record,
                zero_record,
            )
        )
        tick_observation["thread_cpu_elapsed_ns"] = sum(
            int(record["thread_cpu_elapsed_ns"])
            if "thread_cpu_elapsed_ns" in record
            else int(record["thread_cpu_elapsed_ns_sum"])
            for record in (
                publish_record,
                blocking_record,
                zero_record,
            )
        )
        if statistics is None:
            return
        statistics.setdefault("deadline_miss_count", 0)
        statistics.setdefault("guard_exhaustion_count", 0)
        statistics["tick_count"] = int(statistics.get("tick_count", 0)) + 1
        statistics["max_elapsed_sec"] = max(
            float(statistics.get("max_elapsed_sec", 0.0)),
            elapsed_sec,
        )
        statistics["max_spin_attempts"] = max(
            int(statistics.get("max_spin_attempts", 0)),
            callback_attempts,
        )
        statistics["max_blocking_wait_attempts"] = max(
            int(statistics.get("max_blocking_wait_attempts", 0)),
            blocking_wait_attempts,
        )
        statistics["max_zero_wait_attempts"] = max(
            int(statistics.get("max_zero_wait_attempts", 0)),
            zero_wait_attempts,
        )
        if deadline_missed:
            statistics["deadline_miss_count"] = (
                int(statistics.get("deadline_miss_count", 0)) + 1
            )
        if guard_exhausted:
            statistics["guard_exhaustion_count"] = (
                int(statistics.get("guard_exhaustion_count", 0)) + 1
            )
        tick_statistics = statistics.setdefault(
            "tick_observation",
            {
                "count": 0,
                "tail_capacity": (
                    FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY
                ),
                "tail": [],
            },
        )
        assert isinstance(tick_statistics, dict)
        record_fixture_spin_observation(
            tick_statistics,
            tick_observation,
        )

    active_completed_at = started
    try:
        tick_observation["fatal_phase"] = "publish"
        publish_wall_started_ns = monotonic_ns()
        publish_thread_cpu_started_ns = thread_time_ns()
        try:
            publish_action()
        finally:
            publish_record = tick_observation["publish"]
            assert isinstance(publish_record, dict)
            publish_record["wall_elapsed_ns"] = (
                monotonic_ns() - publish_wall_started_ns
            )
            publish_record["thread_cpu_elapsed_ns"] = (
                thread_time_ns() - publish_thread_cpu_started_ns
            )
        publish_completed_at = monotonic()
        active_completed_at = publish_completed_at
        if publish_completed_at > deadline:
            tick_observation["fatal_phase"] = "publish"
            elapsed_sec = publish_completed_at - started
            update_statistics(
                elapsed_sec=elapsed_sec,
                deadline_missed=True,
            )
            raise AssertionError(
                "paced_drain_deadline_miss "
                f"period_sec={period_sec:.9f} "
                f"elapsed_sec={elapsed_sec:.9f} "
                "spin_attempts=0 phase=publish"
            )

        remaining_sec = deadline - monotonic()
        if remaining_sec <= deadline_guard_sec:
            tick_observation["fatal_phase"] = "publish"
            elapsed_sec = monotonic() - started
            update_statistics(
                elapsed_sec=elapsed_sec,
                deadline_missed=False,
                guard_exhausted=True,
            )
            raise AssertionError(
                "paced_drain_deadline_guard_exhausted "
                f"period_sec={period_sec:.9f} "
                f"guard_sec={deadline_guard_sec:.9f} "
                f"elapsed_sec={elapsed_sec:.9f} "
                "spin_attempts=0 phase=publish"
            )

        blocking_wait_attempts = 1
        callback_attempts += 1
        tick_observation["fatal_phase"] = "blocking_wait"
        try:
            observation = spin_once_action(
                min(spin_wait_sec, remaining_sec - deadline_guard_sec)
            )
        except BaseException as error:
            aggregate_spin(
                getattr(error, "_fixture_spin_observation", None),
                "blocking_wait",
            )
            raise
        validated_callback_generation_delta(
            observation,
            "blocking_wait",
        )
        aggregate_spin(observation, "blocking_wait")
        tick_observation["fatal_phase"] = ""
        completed_at = monotonic()
        active_completed_at = completed_at
        if completed_at > deadline:
            tick_observation["fatal_phase"] = "blocking_wait"
            elapsed_sec = completed_at - started
            update_statistics(
                elapsed_sec=elapsed_sec,
                deadline_missed=True,
            )
            raise AssertionError(
                "paced_drain_deadline_miss "
                f"period_sec={period_sec:.9f} "
                f"elapsed_sec={elapsed_sec:.9f} "
                f"spin_attempts={callback_attempts} phase=blocking_wait"
            )

        while callback_attempts < max_callbacks:
            remaining_sec = deadline - monotonic()
            if remaining_sec <= deadline_guard_sec:
                break
            tick_observation["fatal_phase"] = "zero_wait"
            callback_attempts += 1
            zero_wait_attempts += 1
            try:
                observation = spin_once_action(0.0)
            except BaseException as error:
                aggregate_spin(
                    getattr(error, "_fixture_spin_observation", None),
                    "zero_wait",
                )
                raise
            callback_generation_delta = (
                validated_callback_generation_delta(
                    observation,
                    "zero_wait",
                )
            )
            aggregate_spin(observation, "zero_wait")
            tick_observation["fatal_phase"] = ""
            completed_at = monotonic()
            active_completed_at = completed_at
            if completed_at > deadline:
                tick_observation["fatal_phase"] = "zero_wait"
                elapsed_sec = completed_at - started
                update_statistics(
                    elapsed_sec=elapsed_sec,
                    deadline_missed=True,
                )
                raise AssertionError(
                    "paced_drain_deadline_miss "
                    f"period_sec={period_sec:.9f} "
                    f"elapsed_sec={elapsed_sec:.9f} "
                    f"spin_attempts={callback_attempts} phase=zero_wait"
                )
            if callback_generation_delta == 0:
                tick_observation["idle_break_count"] = 1
                tick_observation["break_phase"] = "zero_wait"
                break
        active_completed_at = monotonic()
        remaining_sec = deadline - active_completed_at
        if remaining_sec > 0.0:
            sleep(remaining_sec)
        update_statistics(
            elapsed_sec=active_completed_at - started,
            deadline_missed=False,
        )
        return callback_attempts
    finally:
        if not statistics_updated:
            active_completed_at = monotonic()
            update_statistics(
                elapsed_sec=active_completed_at - started,
                deadline_missed=active_completed_at > deadline,
            )


def drive_isolated_observer_drain_step(
    spin_once_action: object,
    *,
    max_callbacks: int = FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK,
    spin_wait_sec: float = FIXTURE_DRAIN_SPIN_WAIT_SEC,
    statistics: dict[str, object] | None = None,
    monotonic_ns: object = time.monotonic_ns,
    thread_time_ns: object = time.thread_time_ns,
) -> int:
    """Drain fixture evidence while the isolated child owns 100 Hz release."""
    if (
        max_callbacks <= 0
        or not math.isfinite(spin_wait_sec)
        or spin_wait_sec <= 0.0
    ):
        raise AssertionError("isolated observer drain budget invalid")
    started_wall_ns = monotonic_ns()
    started_thread_cpu_ns = thread_time_ns()
    callback_attempts = 0
    callback_count = 0
    idle_count = 0
    break_phase = ""
    fatal_phase = ""
    saturated = False
    try:
        for index in range(max_callbacks):
            fatal_phase = "blocking_wait" if index == 0 else "zero_wait"
            callback_attempts += 1
            observation = spin_once_action(
                spin_wait_sec if index == 0 else 0.0
            )
            if not isinstance(observation, dict):
                raise AssertionError(
                    "isolated_observer_drain_observation_invalid "
                    f"phase={fatal_phase} reason=not_dict"
                )
            callback_generation_delta = observation.get(
                "callback_generation_delta"
            )
            if (
                type(callback_generation_delta) is not int
                or callback_generation_delta not in (0, 1)
            ):
                raise AssertionError(
                    "isolated_observer_drain_observation_invalid "
                    f"phase={fatal_phase} callback_generation_delta="
                    f"{callback_generation_delta}"
                )
            if callback_generation_delta == 0:
                idle_count += 1
                break_phase = fatal_phase
                fatal_phase = ""
                break
            callback_count += 1
            fatal_phase = ""
        else:
            saturated = True
            fatal_phase = "callback_limit"
            raise AssertionError(
                "isolated_observer_drain_saturated "
                f"max_callbacks={max_callbacks}"
            )
        return callback_attempts
    finally:
        completed_wall_ns = monotonic_ns()
        completed_thread_cpu_ns = thread_time_ns()
        if statistics is not None:
            statistics.setdefault("saturation_count", 0)
            step = {
                "step_index": int(statistics.get("step_count", 0)) + 1,
                "wall_elapsed_ns": completed_wall_ns - started_wall_ns,
                "thread_cpu_elapsed_ns": (
                    completed_thread_cpu_ns - started_thread_cpu_ns
                ),
                "spin_attempts": callback_attempts,
                "callback_count": callback_count,
                "idle_count": idle_count,
                "break_phase": break_phase,
                "fatal_phase": fatal_phase,
                "saturated": saturated,
                "input_release_deadline_applies": False,
            }
            statistics["step_count"] = int(
                statistics.get("step_count", 0)
            ) + 1
            statistics["max_elapsed_ns"] = max(
                int(statistics.get("max_elapsed_ns", 0)),
                int(step["wall_elapsed_ns"]),
            )
            statistics["max_spin_attempts"] = max(
                int(statistics.get("max_spin_attempts", 0)),
                callback_attempts,
            )
            statistics["callback_count"] = int(
                statistics.get("callback_count", 0)
            ) + callback_count
            statistics["idle_count"] = int(
                statistics.get("idle_count", 0)
            ) + idle_count
            if saturated:
                statistics["saturation_count"] = int(
                    statistics.get("saturation_count", 0)
                ) + 1
            step_statistics = statistics.setdefault(
                "step_observation",
                {
                    "count": 0,
                    "tail_capacity": (
                        FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY
                    ),
                    "tail": [],
                },
            )
            assert isinstance(step_statistics, dict)
            step_statistics["count"] = int(
                step_statistics.get("count", 0)
            ) + 1
            step_statistics["last"] = step
            tail = list(step_statistics.get("tail", []))
            tail.append(step)
            if len(tail) > FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY:
                tail = tail[-FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY:]
            step_statistics["tail"] = tail


ISOLATED_INPUT_ACK_PHASE = 2
ISOLATED_INPUT_TICK_PERIOD_NS = 10_000_000


def synchronize_isolated_input_phase(
    publisher: object | None,
    desired_armed: bool,
    current_armed: bool,
    *,
    force: bool = False,
) -> tuple[bool, dict[str, object] | None]:
    """Commit an isolated phase before entering a paced publish callback."""
    if publisher is None:
        return current_armed, None
    publisher.assert_alive()
    if not force and desired_armed == current_armed:
        return current_armed, None
    previous_ack = publisher.current_ack
    ack = publisher.phase(desired_armed)
    expected_sequence = int(previous_ack["sequence"]) + 1
    expected_generation = int(previous_ack["generation"]) + 1
    if ack.get("kind") != ISOLATED_INPUT_ACK_PHASE:
        raise AssertionError("isolated_phase_fence_ack_kind_invalid")
    if (
        ack.get("sequence") != expected_sequence
        or ack.get("generation") != expected_generation
    ):
        raise AssertionError("isolated_phase_fence_ack_identity_invalid")
    tick_index = ack.get("tick_index")
    watermark = ack.get("publish_complete_watermark")
    stamp_ns = ack.get("stamp_ns")
    if (
        type(tick_index) is not int
        or type(watermark) is not int
        or type(stamp_ns) is not int
        or tick_index <= int(previous_ack["tick_index"])
        or watermark != tick_index
        or watermark
        <= int(previous_ack["publish_complete_watermark"])
        or stamp_ns <= int(previous_ack["stamp_ns"])
    ):
        raise AssertionError("isolated_phase_fence_apply_invalid")
    return desired_armed, dict(ack)


def validate_post_ack_exact_cohort(
    anchor_stamp_ns: int,
    ack: dict[str, object],
) -> None:
    """Prove an exact output cohort follows the acknowledged child publish."""
    ack_stamp_ns = int(ack["stamp_ns"])
    applied_tick = int(ack["tick_index"])
    watermark = int(ack["publish_complete_watermark"])
    if watermark != applied_tick:
        raise AssertionError("isolated_post_ack_watermark_invalid")
    stamp_delta_ns = anchor_stamp_ns - FIXED_SEC * 1_000_000_000
    if (
        anchor_stamp_ns <= ack_stamp_ns
        or stamp_delta_ns <= 0
        or stamp_delta_ns % ISOLATED_INPUT_TICK_PERIOD_NS != 0
    ):
        raise AssertionError("isolated_post_ack_cohort_stamp_invalid")
    anchor_child_tick = stamp_delta_ns // ISOLATED_INPUT_TICK_PERIOD_NS
    if anchor_child_tick <= applied_tick or anchor_child_tick <= watermark:
        raise AssertionError("isolated_post_ack_cohort_tick_invalid")


def record_fixture_spin_observation(
    statistics: dict[str, object],
    observation: dict[str, object],
) -> None:
    """Keep bounded evidence for executor-call cost without logging each poll."""
    record = dict(observation)
    statistics["count"] = int(statistics.get("count", 0)) + 1
    statistics["last"] = record
    tail = list(statistics.get("tail", []))
    tail.append(record)
    if len(tail) > FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY:
        tail = tail[-FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY:]
    statistics["tail"] = tail
    wall_elapsed_ns = int(record.get("wall_elapsed_ns", 0))
    thread_cpu_elapsed_ns = int(
        record.get("thread_cpu_elapsed_ns", 0)
    )
    max_wall = statistics.get("max_wall")
    if (
        not isinstance(max_wall, dict)
        or wall_elapsed_ns > int(max_wall.get("wall_elapsed_ns", 0))
    ):
        statistics["max_wall"] = record
    max_thread_cpu = statistics.get("max_thread_cpu")
    if (
        not isinstance(max_thread_cpu, dict)
        or thread_cpu_elapsed_ns
        > int(max_thread_cpu.get("thread_cpu_elapsed_ns", 0))
    ):
        statistics["max_thread_cpu"] = record
    callback = record.get("last_callback")
    decomposition = record.get("callback_wrapper_decomposition")
    attribution = record.get("callback_wrapper_attribution")
    if (
        isinstance(callback, dict)
        and isinstance(callback.get("topic"), str)
        and callback["topic"]
        and isinstance(decomposition, dict)
        and isinstance(attribution, dict)
        and attribution.get("valid") is True
    ):
        topic = callback["topic"]
        by_topic = statistics.setdefault("callback_timing_by_topic", {})
        assert isinstance(by_topic, dict)
        topic_statistics = by_topic.setdefault(
            topic,
            {
                "count": 0,
                "wall_elapsed_ns_sum": 0,
                "thread_cpu_elapsed_ns_sum": 0,
                "max_wall_elapsed_ns": 0,
                "max_thread_cpu_elapsed_ns": 0,
                "wrapper_body_wall_elapsed_ns_sum": 0,
                "wrapper_body_thread_cpu_elapsed_ns_sum": 0,
                "pre_wrapper_wall_elapsed_ns_sum": 0,
                "pre_wrapper_thread_cpu_elapsed_ns_sum": 0,
                "wrapper_completion_wall_elapsed_ns_sum": 0,
                "wrapper_completion_thread_cpu_elapsed_ns_sum": 0,
                "post_wrapper_wall_elapsed_ns_sum": 0,
                "post_wrapper_thread_cpu_elapsed_ns_sum": 0,
            },
        )
        assert isinstance(topic_statistics, dict)
        topic_statistics["count"] = int(topic_statistics["count"]) + 1
        for field in (
            "wall_elapsed_ns",
            "thread_cpu_elapsed_ns",
        ):
            sum_field = f"{field}_sum"
            topic_statistics[sum_field] = int(
                topic_statistics[sum_field]
            ) + int(record.get(field, 0))
            max_field = f"max_{field}"
            topic_statistics[max_field] = max(
                int(topic_statistics[max_field]),
                int(record.get(field, 0)),
            )
        for field in (
            "wrapper_body_wall_elapsed_ns",
            "wrapper_body_thread_cpu_elapsed_ns",
            "pre_wrapper_wall_elapsed_ns",
            "pre_wrapper_thread_cpu_elapsed_ns",
            "wrapper_completion_wall_elapsed_ns",
            "wrapper_completion_thread_cpu_elapsed_ns",
            "post_wrapper_wall_elapsed_ns",
            "post_wrapper_thread_cpu_elapsed_ns",
        ):
            sum_field = f"{field}_sum"
            topic_statistics[sum_field] = int(
                topic_statistics[sum_field]
            ) + int(decomposition.get(field, 0))


def spin_fixture_once_with_observation(
    node: object,
    timeout_sec: float,
    *,
    scheduling_required: bool,
    capture: object,
    statistics: dict[str, object],
    spin_once_action: object = rclpy.spin_once,
) -> dict[str, object] | None:
    if not scheduling_required:
        spin_once_action(node, timeout_sec=timeout_sec)
        return None
    spin_start_wall_ns = time.monotonic_ns()
    spin_start_thread_cpu_ns = time.thread_time_ns()
    thread_native_id_before = threading.get_native_id()
    callback_wrapper_entry_generation_before = getattr(
        capture,
        "callback_wrapper_entry_generation",
        None,
    )
    callback_generation_before = capture.callback_completion_generation
    spin_completed = False
    spin_error: BaseException | None = None
    try:
        spin_once_action(node, timeout_sec=timeout_sec)
        spin_completed = True
    except BaseException as error:
        spin_error = error
        raise
    finally:
        spin_end_wall_ns = time.monotonic_ns()
        spin_end_thread_cpu_ns = time.thread_time_ns()
        thread_native_id_after = threading.get_native_id()
        callback_wrapper_entry_generation_after = getattr(
            capture,
            "callback_wrapper_entry_generation",
            None,
        )
        callback_generation_after = capture.callback_completion_generation
        callback_generation_delta = (
            callback_generation_after - callback_generation_before
        )
        callback_wrapper_entry_generation_delta = (
            callback_wrapper_entry_generation_after
            - callback_wrapper_entry_generation_before
            if (
                type(callback_wrapper_entry_generation_before) is int
                and type(callback_wrapper_entry_generation_after) is int
            )
            else None
        )
        observation: dict[str, object] = {
            "call_index": int(statistics.get("count", 0)) + 1,
            "timeout_requested_ns": int(timeout_sec * 1_000_000_000),
            "wall_elapsed_ns": spin_end_wall_ns - spin_start_wall_ns,
            "thread_cpu_elapsed_ns": (
                spin_end_thread_cpu_ns - spin_start_thread_cpu_ns
            ),
            "spin_start_wall_ns": spin_start_wall_ns,
            "spin_start_thread_cpu_ns": spin_start_thread_cpu_ns,
            "spin_end_wall_ns": spin_end_wall_ns,
            "spin_end_thread_cpu_ns": spin_end_thread_cpu_ns,
            "thread_native_id_before": thread_native_id_before,
            "thread_native_id_after": thread_native_id_after,
            "callback_wrapper_entry_generation_before": (
                callback_wrapper_entry_generation_before
            ),
            "callback_wrapper_entry_generation_after": (
                callback_wrapper_entry_generation_after
            ),
            "callback_wrapper_entry_generation_delta": (
                callback_wrapper_entry_generation_delta
            ),
            "callback_generation_before": callback_generation_before,
            "callback_generation_after": callback_generation_after,
            "callback_generation_delta": callback_generation_delta,
        }
        decomposition_error = ""
        if callback_generation_after != callback_generation_before:
            observation["last_callback"] = dict(
                capture.last_callback_observation
            )
        if (
            callback_generation_delta == 1
            and thread_native_id_after == thread_native_id_before
        ):
            callback_observation = observation.get("last_callback")
            if not isinstance(callback_observation, dict):
                decomposition_error = "callback_observation_missing"
            else:
                wall_timestamps = {
                    "spin_start": spin_start_wall_ns,
                    "wrapper_entry": callback_observation.get(
                        "wrapper_entry_wall_ns"
                    ),
                    "callback_body_end": callback_observation.get(
                        "callback_body_end_wall_ns"
                    ),
                    "wrapper_completion": callback_observation.get(
                        "wrapper_completion_wall_ns"
                    ),
                    "spin_end": spin_end_wall_ns,
                }
                thread_cpu_timestamps = {
                    "spin_start": spin_start_thread_cpu_ns,
                    "wrapper_entry": callback_observation.get(
                        "wrapper_entry_thread_cpu_ns"
                    ),
                    "callback_body_end": callback_observation.get(
                        "callback_body_end_thread_cpu_ns"
                    ),
                    "wrapper_completion": callback_observation.get(
                        "wrapper_completion_thread_cpu_ns"
                    ),
                    "spin_end": spin_end_thread_cpu_ns,
                }
                event_id = callback_observation.get("wrapper_event_id")
                expected_event_id = (
                    callback_wrapper_entry_generation_before + 1
                    if type(callback_wrapper_entry_generation_before) is int
                    else None
                )
                callback_thread_ids = (
                    callback_observation.get("thread_native_id_before"),
                    callback_observation.get(
                        "thread_native_id_at_body_end"
                    ),
                    callback_observation.get("thread_native_id_after"),
                )
                if (
                    callback_wrapper_entry_generation_delta != 1
                    or callback_wrapper_entry_generation_before
                    != callback_generation_before
                    or callback_wrapper_entry_generation_after
                    != callback_generation_after
                    or type(event_id) is not int
                    or event_id != expected_event_id
                    or event_id != callback_generation_after
                    or callback_observation.get(
                        "wrapper_entry_generation"
                    )
                    != event_id
                    or callback_observation.get(
                        "wrapper_completion_generation"
                    )
                    != callback_generation_after
                    or callback_generation_after
                    != callback_generation_before + 1
                    or any(
                        thread_id != thread_native_id_before
                        for thread_id in callback_thread_ids
                    )
                    or any(
                        type(timestamp) is not int
                        for timestamp in (
                            *wall_timestamps.values(),
                            *thread_cpu_timestamps.values(),
                        )
                    )
                    or list(wall_timestamps.values())
                    != sorted(wall_timestamps.values())
                    or list(thread_cpu_timestamps.values())
                    != sorted(thread_cpu_timestamps.values())
                ):
                    decomposition_error = (
                        "callback_wrapper_decomposition_invalid"
                    )
                else:
                    wall_values = list(wall_timestamps.values())
                    thread_cpu_values = list(
                        thread_cpu_timestamps.values()
                    )
                    wall_durations = [
                        later - earlier
                        for earlier, later in zip(
                            wall_values,
                            wall_values[1:],
                        )
                    ]
                    thread_cpu_durations = [
                        later - earlier
                        for earlier, later in zip(
                            thread_cpu_values,
                            thread_cpu_values[1:],
                        )
                    ]
                    observation["callback_wrapper_decomposition"] = {
                        "scope": (
                            "fixture_spin_and_instrumented_"
                            "callback_wrapper"
                        ),
                        "dds_subcomponent_attribution": False,
                        "executor_internal_attribution": False,
                        "expected_wrapper_event_count": 1,
                        "observed_wrapper_entry_count": (
                            callback_wrapper_entry_generation_delta
                        ),
                        "observed_wrapper_completion_count": (
                            callback_generation_delta
                        ),
                        "wrapper_event_id": event_id,
                        "wall_timestamps_ns": wall_timestamps,
                        "thread_cpu_timestamps_ns": (
                            thread_cpu_timestamps
                        ),
                        "spin_outer_wrapper_wall_elapsed_ns": (
                            observation["wall_elapsed_ns"]
                        ),
                        "spin_outer_wrapper_thread_cpu_elapsed_ns": (
                            observation["thread_cpu_elapsed_ns"]
                        ),
                        "pre_wrapper_wall_elapsed_ns": wall_durations[0],
                        "wrapper_body_wall_elapsed_ns": wall_durations[1],
                        "wrapper_completion_wall_elapsed_ns": (
                            wall_durations[2]
                        ),
                        "post_wrapper_wall_elapsed_ns": wall_durations[3],
                        "post_body_wall_elapsed_ns": sum(
                            wall_durations[2:]
                        ),
                        "pre_wrapper_thread_cpu_elapsed_ns": (
                            thread_cpu_durations[0]
                        ),
                        "wrapper_body_thread_cpu_elapsed_ns": (
                            thread_cpu_durations[1]
                        ),
                        "wrapper_completion_thread_cpu_elapsed_ns": (
                            thread_cpu_durations[2]
                        ),
                        "post_wrapper_thread_cpu_elapsed_ns": (
                            thread_cpu_durations[3]
                        ),
                        "post_body_thread_cpu_elapsed_ns": sum(
                            thread_cpu_durations[2:]
                        ),
                        "outer_minus_body_wall_elapsed_ns": (
                            observation["wall_elapsed_ns"]
                            - wall_durations[1]
                        ),
                        "outer_minus_body_thread_cpu_elapsed_ns": (
                            observation["thread_cpu_elapsed_ns"]
                            - thread_cpu_durations[1]
                        ),
                    }
                    observation["callback_wrapper_attribution"] = {
                        "valid": True,
                        "reason": "",
                        "wrapper_event_id": event_id,
                    }
        elif callback_generation_delta == 1:
            decomposition_error = "thread_native_id"
        elif (
            callback_generation_delta == 0
            and callback_wrapper_entry_generation_delta not in (None, 0)
        ):
            decomposition_error = "callback_wrapper_event_count_invalid"
        elif callback_generation_delta not in (0, 1):
            decomposition_error = "callback_generation_delta"
        if decomposition_error:
            observation["callback_wrapper_attribution"] = {
                "valid": False,
                "reason": decomposition_error,
            }
        record_fixture_spin_observation(statistics, observation)
        if spin_error is not None:
            try:
                setattr(
                    spin_error,
                    "_fixture_spin_observation",
                    observation,
                )
            except Exception:
                pass
        if (
            spin_completed
            and callback_generation_delta not in (0, 1)
        ):
            statistics["integrity_failure_count"] = (
                int(statistics.get("integrity_failure_count", 0)) + 1
            )
            statistics["last_integrity_failure"] = (
                "callback_generation_delta"
            )
            error = AssertionError(
                "fixture_spin_integrity_failure "
                f"callback_generation_delta={callback_generation_delta}"
            )
            setattr(error, "_fixture_spin_observation", observation)
            raise error
        if (
            spin_completed
            and thread_native_id_after != thread_native_id_before
        ):
            statistics["integrity_failure_count"] = (
                int(statistics.get("integrity_failure_count", 0)) + 1
            )
            statistics["last_integrity_failure"] = "thread_native_id"
            error = AssertionError(
                "fixture_spin_integrity_failure "
                f"thread_native_id_before={thread_native_id_before} "
                f"thread_native_id_after={thread_native_id_after}"
            )
            setattr(error, "_fixture_spin_observation", observation)
            raise error
        if spin_completed and decomposition_error:
            statistics["integrity_failure_count"] = (
                int(statistics.get("integrity_failure_count", 0)) + 1
            )
            statistics["last_integrity_failure"] = decomposition_error
            error = AssertionError(
                "fixture_spin_integrity_failure "
                f"{decomposition_error}"
            )
            setattr(error, "_fixture_spin_observation", observation)
            raise error
    return observation


def missing_recorder_subscriptions(
    node: rclpy.node.Node,
    recorder_node_name: str,
) -> list[str]:
    """Return topics without a subscription owned by the named recorder."""
    missing = []
    for topic in RECORDER_TOPICS:
        endpoints = node.get_subscriptions_info_by_topic(topic)
        if not any(endpoint.node_name == recorder_node_name for endpoint in endpoints):
            missing.append(topic)
    return missing


def run_hash(value: str) -> int:
    result = 1469598103934665603
    for byte in value.encode():
        result ^= byte
        result = (result * 1099511628211) & ((1 << 64) - 1)
    return result


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class RuntimeRing:
    def __init__(
        self,
        root: Path,
        socket_path_override: Path | None = None,
    ) -> None:
        self.socket_path = socket_path_override or (root / "observer.sock")
        self.fd = os.memfd_create(
            "c002ay0-pp-runtime",
            os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING,
        )
        os.ftruncate(self.fd, RING_SIZE)
        header = bytearray(HEADER_SIZE)
        struct.pack_into(
            "<QIIIII",
            header,
            0,
            RING_MAGIC,
            ABI_VERSION,
            LAYOUT_VERSION,
            HEADER_SIZE,
            RECORD_SIZE,
            CAPACITY,
        )
        struct.pack_into("<B", header, 28, 2)
        struct.pack_into(
            "<QQQQQ",
            header,
            32,
            RING_SIZE,
            SESSION_NONCE,
            OBSERVER_INSTANCE,
            run_hash(RUN_ID),
            HARNESS_READY,
        )
        os.pwrite(self.fd, header, 0)
        fcntl.fcntl(
            self.fd,
            fcntl.F_ADD_SEALS,
            fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL,
        )
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.server.bind(str(self.socket_path))
        self.server.listen(1)
        self.error: BaseException | None = None
        self.thread = threading.Thread(target=self._serve)
        self.thread.start()

    def _serve(self) -> None:
        try:
            peer, _ = self.server.accept()
            with peer:
                handshake = peer.recv(256)
                if len(handshake) != 104:
                    raise AssertionError("invalid observation handshake size")
                magic, abi = struct.unpack_from("<QI", handshake, 0)
                role = handshake[12]
                nonce, instance = struct.unpack_from("<QQ", handshake, 24)
                if (
                    magic != HANDSHAKE_MAGIC
                    or abi != ABI_VERSION
                    or role != 2
                    or nonce != SESSION_NONCE
                    or instance != OBSERVER_INSTANCE
                ):
                    raise AssertionError("invalid observation handshake")
                peer.sendmsg(
                    [b"\x01"],
                    [
                        (
                            socket.SOL_SOCKET,
                            socket.SCM_RIGHTS,
                            array("i", [self.fd]),
                        )
                    ],
                )
        except BaseException as error:
            self.error = error
        finally:
            self.server.close()

    def state(self) -> tuple[int, int]:
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise AssertionError("observation handshake remained blocked")
        if self.error is not None:
            raise self.error
        write_index = struct.unpack("<Q", os.pread(self.fd, 8, 128))[0]
        drop_count = struct.unpack("<Q", os.pread(self.fd, 8, 256))[0]
        return write_index, drop_count

    def records(self) -> tuple[list[tuple[int, int, int]], int]:
        write_index, drop_count = self.state()
        records = []
        for index in range(min(write_index, CAPACITY)):
            payload = os.pread(
                self.fd,
                RECORD_SIZE,
                HEADER_SIZE + (index % CAPACITY) * RECORD_SIZE,
            )
            sequence, steady_start_ns, duration_ns = struct.unpack_from(
                "<QQQ", payload, 0
            )
            records.append((sequence, steady_start_ns, duration_ns))
        return records, drop_count

    def close(self) -> None:
        os.close(self.fd)
        try:
            self.socket_path.unlink()
        except FileNotFoundError:
            pass


def canonical_payload_hash(semantic: dict) -> str:
    semantic_wire = json.dumps(
        semantic,
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(semantic_wire).hexdigest()


def json_safe_artifact(value: object) -> object:
    if isinstance(value, dict):
        return {
            key: json_safe_artifact(nested)
            for key, nested in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [json_safe_artifact(nested) for nested in value]
    if isinstance(value, float) and not math.isfinite(value):
        if math.isnan(value):
            kind = "nan"
        elif value > 0.0:
            kind = "positive_infinity"
        else:
            kind = "negative_infinity"
        return {NONFINITE_FLOAT_TAG: kind}
    return value


def restore_json_safe_artifact(value: object) -> object:
    if isinstance(value, dict):
        if set(value) == {NONFINITE_FLOAT_TAG}:
            kind = value[NONFINITE_FLOAT_TAG]
            if kind == "nan":
                return math.nan
            if kind == "positive_infinity":
                return math.inf
            if kind == "negative_infinity":
                return -math.inf
            raise ValueError(f"unknown nonfinite float tag: {kind}")
        return {
            key: restore_json_safe_artifact(nested)
            for key, nested in value.items()
        }
    if isinstance(value, list):
        return [restore_json_safe_artifact(nested) for nested in value]
    return value


def strict_json_text(value: object, **kwargs: object) -> str:
    return json.dumps(
        json_safe_artifact(value),
        allow_nan=False,
        **kwargs,
    )


def reject_nonstandard_json_constant(value: str) -> object:
    raise ValueError(f"non-standard JSON constant: {value}")


def canonical_payload(topic: str, message: object) -> str:
    return canonical_payload_hash(canonical_debug(topic, message))


def canonical_debug(topic: str, message: object) -> dict:
    semantic = message_to_ordereddict(message)
    if topic == "tracking":
        semantic["command_age_sec"] = 0.0
    elif topic == "command_envelope":
        semantic["producer_instance_id"] = 1
        semantic["command_sequence"] = 1
        semantic["command_age_sec"] = 0.0
    elif topic == "execution_envelope":
        semantic["producer_instance_id"] = 1
        semantic["command_sequence"] = 1
        semantic["command_envelope"]["producer_instance_id"] = 1
        semantic["command_envelope"]["command_sequence"] = 1
        semantic["command_envelope"]["command_age_sec"] = 0.0
        semantic["witness"]["source_generation"] = 1
        semantic["witness"]["base_source_generation"] = 1
    normalize_timing_fields(semantic)
    return semantic


def normalize_timing_fields(value: object) -> None:
    if isinstance(value, dict):
        if set(value) == {"sec", "nanosec"}:
            value["sec"] = 0
            value["nanosec"] = 0
            return
        for nested in value.values():
            normalize_timing_fields(nested)
    elif isinstance(value, list):
        for nested in value:
            normalize_timing_fields(nested)


class Capture:
    def __init__(
        self,
        node: rclpy.node.Node,
        expected_session_generation: int,
        expected_session_nonce: int,
        cohort_alignment_enabled: bool = False,
        callback_instrumentation_enabled: bool = False,
    ) -> None:
        self.expected_session_generation = expected_session_generation
        self.expected_session_nonce = expected_session_nonce
        self.node = node
        self.enabled = False
        self.latest_sequence = 0
        self.values: dict[str, list[str]] = {}
        self.identities: dict[str, list[tuple]] = {}
        self.canonical_semantics: dict[str, list[dict]] = {}
        self.first_debug: dict[str, dict] = {}
        self.cohort_alignment_enabled = cohort_alignment_enabled
        self.capture_after_stamp_ns: int | None = None
        self.startup_anchor_stamp_ns: int | None = None
        self.startup_anchor_sequence: int | None = None
        self.startup_anchor_producer_instance_id: int | None = None
        self.startup_ledger_overflow = False
        self.startup_ledger: list[dict[str, object]] = []
        self.alignment_stage_overflow = False
        self.alignment_stage_message_count = 0
        self.alignment_stage_max_message_count = 0
        self.alignment_stage_mismatch_reason = ""
        self.reanchor_fence_invalid: dict[str, object] | None = None
        self.alignment_cycles: dict[
            int, dict[str, list[dict[str, object]]]
        ] = {}
        self.next_expected_sequence: int | None = None
        self.materialized_sequences: list[int] = []
        self.max_abs_steering: dict[str, float] = {}
        self.shadow_snapshot_count = 0
        self.shadow_snapshot_invalid = False
        self.authorized_proposal_count = 0
        self.binding_count = 0
        self.exact_binding_count = 0
        self.binding_invalid = False
        self.binding_dispositions: dict[str, int] = {}
        self.snapshot_controller_instance_ids: set[int] = set()
        self.max_snapshot_race_arm_epoch = 0
        self.expected_e2e_race_arm_epoch = 0
        self.e2e_active = False
        self.e2e_proposals: dict[tuple[int, ...], dict[str, int]] = {}
        self.e2e_observed_binding_keys: set[tuple[int, ...]] = set()
        self.e2e_terminals: dict[tuple[int, ...], dict[str, int | str]] = {}
        self.e2e_proposal_audits: dict[tuple[int, ...], dict[str, int]] = {}
        self.e2e_planner_input_audits: dict[tuple[int, ...], dict[str, int]] = {}
        self.e2e_proposal_audit_overflow = False
        self.e2e_proposal_audit_invalid = False
        self.e2e_planner_input_audit_overflow = False
        self.e2e_planner_input_audit_invalid = False
        self.e2e_duplicate_proposal_audits = 0
        self.e2e_duplicate_planner_input_audits = 0
        self.e2e_out_of_cohort_proposals = 0
        self.e2e_out_of_cohort_bindings = 0
        self.e2e_out_of_cohort_binding_audits = 0
        self.e2e_out_of_cohort_proposal_audits = 0
        self.e2e_out_of_cohort_planner_input_audits = 0
        self.e2e_out_of_cohort_samples: dict[str, list[dict[str, object]]] = {
            "proposal": [],
            "binding": [],
            "binding_audit": [],
            "proposal_audit": [],
            "planner_input_audit": [],
        }
        self.e2e_out_of_cohort_sample_overflow = False
        self.e2e_duplicate_proposals = 0
        self.e2e_duplicate_bindings = 0
        self.e2e_input_publish_ticks: dict[int, dict[str, int]] = {}
        self.e2e_input_publish_tick_overflow = False
        self.e2e_input_publish_tick_invalid = False
        self.free_run_ack_count = 0
        self.free_run_source_key_count = 0
        self.callback_instrumentation_enabled = (
            callback_instrumentation_enabled
        )
        self.callback_wrapper_entry_generation = 0
        self.callback_completion_generation = 0
        self.last_callback_observation: dict[str, object] = {}
        topics = {
            "command": ("/ay0/output/control_cmd", AckermannControlCommand),
            "raw_command": (
                "/ay0/output/raw_control_cmd",
                AckermannControlCommand,
            ),
            "tracking": (
                "/ay0/output/tracking_status",
                ControllerTrackingStatus,
            ),
            "command_envelope": (
                "/ay0/output/command_envelope",
                ControllerCommandEnvelope,
            ),
            "execution_envelope": (
                "/ay0/output/execution_envelope",
                ControllerExecutionEnvelope,
            ),
        }
        self.subscriptions = [
            node.create_subscription(
                message_type,
                topic,
                self._instrument_callback(
                    key,
                    lambda message, key=key: self._receive(key, message),
                ),
                100,
            )
            for key, (topic, message_type) in topics.items()
        ]
        self.subscriptions.append(
            node.create_subscription(
                String,
                "/test/c002ay0/state_lattice/planner_input_audit",
                self._receive_planner_input_audit,
                QoSProfile(
                    depth=E2E_PROPOSAL_AUDIT_LEDGER_LIMIT,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                String,
                "/test/c002ay0/state_lattice/proposal_publish_audit",
                self._receive_proposal_publish_audit,
                QoSProfile(
                    depth=E2E_PROPOSAL_AUDIT_LEDGER_LIMIT,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                ControllerBaseTrajectorySnapshot,
                "/control/overtake/base_trajectory_snapshot",
                self._instrument_callback(
                    "shadow_snapshot",
                    self._receive_shadow_snapshot,
                ),
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
        )

        self.subscriptions.append(
            node.create_subscription(
                String,
                "/test/c002ay0/input_publish_tick",
                self._receive_input_publish_tick,
                QoSProfile(
                    depth=E2E_INPUT_PUBLISH_TICK_LEDGER_LIMIT,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                AuthorizedCartesianTrajectory,
                "/debug/overtake/state_lattice/"
                "authorized_cartesian_trajectory",
                self._instrument_callback(
                    "authorized_proposal",
                    self._receive_authorized_proposal,
                ),
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                String,
                "/debug/overtake/state_lattice/source_binding",
                self._instrument_callback(
                    "source_binding",
                    self._receive_binding,
                ),
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                String,
                "/test/c002ay0/state_lattice/binding_callback_terminal",
                self._instrument_callback(
                    "binding_callback_terminal",
                    self._receive_audited_binding,
                ),
                QoSProfile(
                    depth=64,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                FreeRunExecutionAck,
                "/ay0/output/free_run_execution_ack",
                self._instrument_callback(
                    "free_run_execution_ack",
                    lambda _: setattr(
                        self,
                        "free_run_ack_count",
                        self.free_run_ack_count + 1,
                    ),
                ),
                100,
            )
        )
        self.subscriptions.append(
            node.create_subscription(
                FreeRunSourceKey,
                "/ay0/output/free_run_source_key",
                self._instrument_callback(
                    "free_run_source_key",
                    lambda _: setattr(
                        self,
                        "free_run_source_key_count",
                        self.free_run_source_key_count + 1,
                    ),
                ),
                100,
            )
        )

    def _receive_input_publish_tick(self, message: String) -> None:
        if not self.e2e_active:
            return
        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.e2e_input_publish_tick_invalid = True
            return
        required = (
            "schema_version",
            "tick_index",
            "clock_stamp_ns",
            "trajectory_stamp_ns",
            "plan_stamp_ns",
            "clock_publish_steady_ns",
            "plan_publish_steady_ns",
            "covered_clock_first_stamp_ns",
            "covered_clock_last_stamp_ns",
            "covered_clock_tick_period_ns",
            "covered_clock_tick_count",
        )
        if (
            not isinstance(value, dict)
            or value.get("schema_version") != 1
            or any(type(value.get(field)) is not int for field in required[1:])
        ):
            self.e2e_input_publish_tick_invalid = True
            return
        plan_stamp_ns = int(value["plan_stamp_ns"])
        covered_first_ns = int(value["covered_clock_first_stamp_ns"])
        covered_last_ns = int(value["covered_clock_last_stamp_ns"])
        covered_period_ns = int(value["covered_clock_tick_period_ns"])
        covered_count = int(value["covered_clock_tick_count"])
        if (
            covered_period_ns != SIM_CLOCK_QUANTUM_NS * 10
            or covered_count != 10
            or covered_first_ns > covered_last_ns
            or covered_first_ns
            + (covered_count - 1) * covered_period_ns
            != covered_last_ns
        ):
            self.e2e_input_publish_tick_invalid = True
            return
        if (
            plan_stamp_ns in self.e2e_input_publish_ticks
            or len(self.e2e_input_publish_ticks)
            >= E2E_INPUT_PUBLISH_TICK_LEDGER_LIMIT
        ):
            if plan_stamp_ns in self.e2e_input_publish_ticks:
                self.e2e_input_publish_tick_invalid = True
            else:
                self.e2e_input_publish_tick_overflow = True
            return
        self.e2e_input_publish_ticks[plan_stamp_ns] = {
            field: int(value[field]) for field in required[1:]
        }

    def _instrument_callback(self, topic: str, callback: object) -> object:
        if not self.callback_instrumentation_enabled:
            return callback

        def instrumented(message: object) -> None:
            wrapper_entry_wall_ns = time.monotonic_ns()
            wrapper_entry_thread_cpu_ns = time.thread_time_ns()
            thread_native_id_before = threading.get_native_id()
            wrapper_event_id = (
                int(
                    getattr(
                        self,
                        "callback_wrapper_entry_generation",
                        0,
                    )
                )
                + 1
            )
            self.callback_wrapper_entry_generation = wrapper_event_id
            try:
                callback(message)
            finally:
                callback_body_end_wall_ns = time.monotonic_ns()
                callback_body_end_thread_cpu_ns = time.thread_time_ns()
                thread_native_id_at_body_end = threading.get_native_id()
                wrapper_completion_generation = (
                    self.callback_completion_generation + 1
                )
                callback_observation = {
                    "topic": topic,
                    "wrapper_event_id": wrapper_event_id,
                    "wrapper_entry_generation": wrapper_event_id,
                    "wrapper_completion_generation": (
                        wrapper_completion_generation
                    ),
                    "wrapper_entry_wall_ns": wrapper_entry_wall_ns,
                    "wrapper_entry_thread_cpu_ns": (
                        wrapper_entry_thread_cpu_ns
                    ),
                    "callback_body_end_wall_ns": (
                        callback_body_end_wall_ns
                    ),
                    "callback_body_end_thread_cpu_ns": (
                        callback_body_end_thread_cpu_ns
                    ),
                    "wall_elapsed_ns": (
                        callback_body_end_wall_ns
                        - wrapper_entry_wall_ns
                    ),
                    "thread_cpu_elapsed_ns": (
                        callback_body_end_thread_cpu_ns
                        - wrapper_entry_thread_cpu_ns
                    ),
                    "wrapper_body_wall_elapsed_ns": (
                        callback_body_end_wall_ns
                        - wrapper_entry_wall_ns
                    ),
                    "wrapper_body_thread_cpu_elapsed_ns": (
                        callback_body_end_thread_cpu_ns
                        - wrapper_entry_thread_cpu_ns
                    ),
                    "thread_native_id_before": thread_native_id_before,
                    "thread_native_id_at_body_end": (
                        thread_native_id_at_body_end
                    ),
                }
                wrapper_completion_wall_ns = time.monotonic_ns()
                wrapper_completion_thread_cpu_ns = time.thread_time_ns()
                thread_native_id_after = threading.get_native_id()
                callback_observation.update(
                    {
                        "wrapper_completion_wall_ns": (
                            wrapper_completion_wall_ns
                        ),
                        "wrapper_completion_thread_cpu_ns": (
                            wrapper_completion_thread_cpu_ns
                        ),
                        "thread_native_id_after": thread_native_id_after,
                    }
                )
                self.last_callback_observation = callback_observation
                self.callback_completion_generation = (
                    wrapper_completion_generation
                )

        return instrumented

    @staticmethod
    def _proposal_key_from_message(
        message: AuthorizedCartesianTrajectory,
    ) -> tuple[int, ...]:
        return (
            int(message.plan_sample_key.race_arm_epoch),
            int(message.plan_sample_key.planner_instance_id),
            int(message.plan_sample_key.attempt_id),
            int(message.plan_sample_key.connector_transaction_id),
            int(message.plan_sample_key.plan_generation),
            int(message.candidate_revision),
            int(message.authority_token),
            int(message.safety_snapshot_id),
            int(message.source_controller_instance_id),
            int(message.source_controller_sequence),
            int(message.base_lease_id),
        )

    @staticmethod
    def _proposal_key_from_binding(value: dict) -> tuple[int, ...] | None:
        names = (
            "proposal_race_arm_epoch",
            "proposal_planner_instance_id",
            "proposal_attempt_id",
            "proposal_connector_transaction_id",
            "proposal_plan_generation",
            "proposal_candidate_revision",
            "proposal_authority_token",
            "proposal_safety_snapshot_id",
            "proposal_controller_instance_id",
            "proposal_controller_sequence",
            "proposal_base_lease_id",
        )
        if any(
            not isinstance(value.get(name), int)
            or isinstance(value.get(name), bool)
            for name in names
        ):
            return None
        return tuple(int(value[name]) for name in names)

    @staticmethod
    def _stamp_ns(stamp: object) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _json_stamp_ns(value: dict, prefix: str) -> int | None:
        sec = value.get(f"{prefix}_sec")
        nanosec = value.get(f"{prefix}_nanosec")
        if not isinstance(sec, int) or not isinstance(nanosec, int):
            return None
        if nanosec < 0 or nanosec >= 1_000_000_000:
            return None
        return sec * 1_000_000_000 + nanosec

    def begin_e2e_measurement(self, expected_race_arm_epoch: int) -> None:
        if expected_race_arm_epoch <= self.max_snapshot_race_arm_epoch:
            raise AssertionError("E2E race epoch did not advance")
        self.shadow_snapshot_count = 0
        self.shadow_snapshot_invalid = False
        self.authorized_proposal_count = 0
        self.binding_count = 0
        self.exact_binding_count = 0
        self.binding_invalid = False
        self.binding_dispositions.clear()
        self.expected_e2e_race_arm_epoch = expected_race_arm_epoch
        self.e2e_active = True
        self.e2e_proposals.clear()
        self.e2e_observed_binding_keys.clear()
        self.e2e_terminals.clear()
        self.e2e_proposal_audits.clear()
        self.e2e_planner_input_audits.clear()
        self.e2e_proposal_audit_overflow = False
        self.e2e_proposal_audit_invalid = False
        self.e2e_planner_input_audit_overflow = False
        self.e2e_planner_input_audit_invalid = False
        self.e2e_duplicate_proposal_audits = 0
        self.e2e_duplicate_planner_input_audits = 0
        self.e2e_out_of_cohort_proposals = 0
        self.e2e_out_of_cohort_bindings = 0
        self.e2e_out_of_cohort_binding_audits = 0
        self.e2e_out_of_cohort_proposal_audits = 0
        self.e2e_out_of_cohort_planner_input_audits = 0
        self.e2e_out_of_cohort_samples = {
            "proposal": [],
            "binding": [],
            "binding_audit": [],
            "proposal_audit": [],
            "planner_input_audit": [],
        }
        self.e2e_out_of_cohort_sample_overflow = False
        self.e2e_duplicate_proposals = 0
        self.e2e_duplicate_bindings = 0
        self.e2e_input_publish_ticks = {}
        self.e2e_input_publish_tick_overflow = False
        self.e2e_input_publish_tick_invalid = False

    def finalize_e2e_measurement(self) -> dict[str, object]:
        self.e2e_active = False
        if self.e2e_proposal_audit_overflow:
            raise AssertionError(
                "proposal publish audit ledger capacity exceeded "
                f"capacity={E2E_PROPOSAL_AUDIT_LEDGER_LIMIT}"
            )
        if self.e2e_proposal_audit_invalid:
            raise AssertionError("proposal publish audit malformed or order-inverted")
        if self.e2e_planner_input_audit_overflow:
            raise AssertionError(
                "planner input audit ledger capacity exceeded "
                f"capacity={E2E_PROPOSAL_AUDIT_LEDGER_LIMIT}"
            )
        if self.e2e_planner_input_audit_invalid:
            raise AssertionError("planner input audit malformed or order-inverted")
        if self.e2e_duplicate_proposal_audits != 0:
            raise AssertionError(
                "proposal publish audit duplicate identity "
                f"count={self.e2e_duplicate_proposal_audits}"
            )
        if self.e2e_duplicate_planner_input_audits != 0:
            raise AssertionError(
                "planner input audit duplicate identity "
                f"count={self.e2e_duplicate_planner_input_audits}"
            )
        audit_only_keys = set(self.e2e_proposal_audits) - set(self.e2e_terminals)
        planner_input_only_keys = (
            set(self.e2e_planner_input_audits) - set(self.e2e_proposal_audits)
        )
        complete_causal_keys = (
            set(self.e2e_proposals)
            & set(self.e2e_terminals)
            & set(self.e2e_proposal_audits)
            & set(self.e2e_planner_input_audits)
        )
        for key in sorted(complete_causal_keys):
            proposal = self.e2e_proposals[key]
            terminal = self.e2e_terminals[key]
            audit = self.e2e_proposal_audits[key]
            planner_input = self.e2e_planner_input_audits[key]
            terminal_source = terminal.get("binding_snapshot_evidence_v1")
            if not isinstance(terminal_source, dict) or (
                planner_input["proposal_source_stamp_sec"],
                planner_input["proposal_source_stamp_nanosec"],
                planner_input["proposal_source_generation"],
            ) != (
                terminal_source["proposal"]["source_stamp_sec"],
                terminal_source["proposal"]["source_stamp_nanosec"],
                terminal_source["proposal"]["source_generation"],
            ):
                continue
            required_proposal_fields = (
                "plan_stamp_ns",
                "safety_evaluation_stamp_ns",
                "safety_valid_until_ns",
            )
            required_terminal_fields = (
                "binding_ros_now_ns",
                "binding_callback_monotonic_ns",
                "safety_valid_until_ns",
            )
            if not all(field in proposal for field in required_proposal_fields) or not all(
                field in terminal for field in required_terminal_fields
            ):
                continue
            plan_stamp_ns = int(proposal["plan_stamp_ns"])
            safety_evaluation_stamp_ns = int(
                proposal["safety_evaluation_stamp_ns"]
            )
            safety_valid_until_ns = int(proposal["safety_valid_until_ns"])
            binding_ros_now_ns = int(terminal["binding_ros_now_ns"])
            if not (
                plan_stamp_ns == safety_evaluation_stamp_ns
                and safety_valid_until_ns > plan_stamp_ns
                and int(terminal["safety_valid_until_ns"])
                == safety_valid_until_ns
                and binding_ros_now_ns >= plan_stamp_ns
                and binding_ros_now_ns >= safety_valid_until_ns
            ):
                continue
            planning_monotonic_ns = int(
                planner_input["planning_timer_entry_monotonic_ns"]
            )
            binding_monotonic_ns = int(
                terminal["binding_callback_monotonic_ns"]
            )
            if binding_monotonic_ns < planning_monotonic_ns:
                raise AssertionError(
                    "safety lifetime monotonic direct comparison invalid "
                    f"key={key} planning_monotonic_ns={planning_monotonic_ns} "
                    f"binding_monotonic_ns={binding_monotonic_ns}"
                )
            diagnostic_lifetime_ns = safety_valid_until_ns - plan_stamp_ns
            planning_to_binding_ns = (
                binding_monotonic_ns - planning_monotonic_ns
            )
            diagnostic_monotonic_margin_ns = (
                diagnostic_lifetime_ns - planning_to_binding_ns
            )
            direct_evidence = {
                "schema_version": 1,
                "clock": "CLOCK_MONOTONIC",
                "full_terminal_key": key,
                "plan_stamp_ns": plan_stamp_ns,
                "binding_ros_now_ns": binding_ros_now_ns,
                "safety_valid_until_ns": safety_valid_until_ns,
                "planning_timer_entry_monotonic_ns": planning_monotonic_ns,
                "fixed_record_capture_monotonic_ns": int(
                    audit["capture_monotonic_ns"]
                ),
                "publish_call_entry_monotonic_ns": int(
                    audit["publish_call_entry_monotonic_ns"]
                ),
                "publish_call_return_monotonic_ns": int(
                    audit["publish_call_return_monotonic_ns"]
                ),
                "binding_callback_monotonic_ns": binding_monotonic_ns,
                "planning_to_binding_ns": planning_to_binding_ns,
                "diagnostic_lifetime_ns": diagnostic_lifetime_ns,
                "diagnostic_monotonic_margin_ns": (
                    diagnostic_monotonic_margin_ns
                ),
                "ledger_close_mismatch": {
                    "proposal_audit_only": sorted(audit_only_keys)[:5],
                    "planner_input_only": sorted(planner_input_only_keys)[:5],
                },
            }
            raise AssertionError(
                "safety lifetime monotonic direct comparison "
                f"binding_before_deadline=false key={key} evidence="
                f"{json.dumps(direct_evidence, sort_keys=True)}"
            )
        if audit_only_keys:
            raise AssertionError(
                "proposal publish audit terminal exact-join mismatch "
                f"audit_only={sorted(audit_only_keys)[:5]}"
            )
        if planner_input_only_keys:
            raise AssertionError(
                "planner input audit proposal publish exact-join mismatch "
                f"planner_input_only={sorted(planner_input_only_keys)[:5]}"
            )
        for key, audit in self.e2e_proposal_audits.items():
            planner_input = self.e2e_planner_input_audits.get(key)
            if planner_input is None:
                raise AssertionError(
                    "planner input audit proposal publish exact-join mismatch "
                    f"key={key}"
                )
            terminal = self.e2e_terminals[key]
            terminal_source = terminal["binding_snapshot_evidence_v1"]
            if not isinstance(terminal_source, dict) or (
                planner_input["proposal_source_stamp_sec"],
                planner_input["proposal_source_stamp_nanosec"],
                planner_input["proposal_source_generation"],
            ) != (
                terminal_source["proposal"]["source_stamp_sec"],
                terminal_source["proposal"]["source_stamp_nanosec"],
                terminal_source["proposal"]["source_generation"],
            ):
                raise AssertionError(
                    "planner input audit authoritative source identity mismatch "
                    f"key={key}"
                )
            binding_callback_ns = int(terminal["binding_callback_monotonic_ns"])
            publish_entry_ns = int(audit["publish_call_entry_monotonic_ns"])
            publish_return_ns = int(audit["publish_call_return_monotonic_ns"])
            if binding_callback_ns < publish_entry_ns:
                raise AssertionError(
                    "proposal publish audit binding order inversion "
                    f"key={key} binding_callback_ns={binding_callback_ns} "
                    f"publish_entry_ns={publish_entry_ns}"
                )
            terminal["proposal_publish_audit_v1"] = {
                "classification": (
                    "binding_callback_during_publish_call"
                    if binding_callback_ns < publish_return_ns
                    else "binding_callback_after_publish_return"
                ),
                **audit,
                "binding_callback_monotonic_ns": binding_callback_ns,
                "publish_entry_to_binding_callback_ns": (
                    binding_callback_ns - publish_entry_ns
                ),
                "publish_return_to_binding_callback_ns": (
                    binding_callback_ns - publish_return_ns
                ),
                "planner_input_audit_v1": {
                    "classification": "input_then_planning_then_capture",
                    **planner_input,
                    "input_to_planning_timer_ns": (
                        planner_input["planning_timer_entry_monotonic_ns"]
                        - planner_input[
                            "base_snapshot_callback_entry_monotonic_ns"
                        ]
                    ),
                    "planning_timer_to_capture_ns": (
                        planner_input["capture_monotonic_ns"]
                        - planner_input["planning_timer_entry_monotonic_ns"]
                    ),
                },
            }
        if self.e2e_out_of_cohort_sample_overflow:
            raise AssertionError(
                "out-of-cohort sample capacity exceeded "
                f"per_stream_capacity={E2E_OUT_OF_COHORT_SAMPLE_LIMIT}"
            )
        out_of_cohort_counts = {
            "proposal": self.e2e_out_of_cohort_proposals,
            "binding": self.e2e_out_of_cohort_bindings,
            "binding_audit": self.e2e_out_of_cohort_binding_audits,
            "proposal_audit": self.e2e_out_of_cohort_proposal_audits,
            "planner_input_audit": self.e2e_out_of_cohort_planner_input_audits,
        }
        if any(
            out_of_cohort_counts[stream] != len(samples)
            for stream, samples in self.e2e_out_of_cohort_samples.items()
        ):
            raise AssertionError(
                "out-of-cohort sample count mismatch "
                f"counts={out_of_cohort_counts} "
                "samples="
                f"{ {k: len(v) for k, v in self.e2e_out_of_cohort_samples.items()} }"
            )
        if self.e2e_input_publish_tick_overflow:
            raise AssertionError(
                "input publish tick ledger capacity exceeded "
                f"capacity={E2E_INPUT_PUBLISH_TICK_LEDGER_LIMIT}"
            )
        if self.e2e_input_publish_tick_invalid:
            raise AssertionError("input publish tick ledger invalid")
        safety_lifetime_timeline = []
        safety_lifetime_timeline_missing = 0
        joined_timeline_keys = (
            self.e2e_proposals.keys() & self.e2e_terminals.keys()
        )
        if len(joined_timeline_keys) > E2E_SAFETY_TIMELINE_LIMIT:
            raise AssertionError(
                "safety lifetime timeline capacity exceeded "
                f"count={len(joined_timeline_keys)} "
                f"capacity={E2E_SAFETY_TIMELINE_LIMIT}"
            )
        for key in joined_timeline_keys:
            proposal = self.e2e_proposals[key]
            terminal = self.e2e_terminals[key]
            if (
                "plan_stamp_ns" not in proposal
                or "safety_evaluation_stamp_ns" not in proposal
                or "safety_valid_until_ns" not in proposal
                or "received_sim_ns" not in proposal
                or "binding_ros_now_ns" not in terminal
                or "safety_valid_until_ns" not in terminal
            ):
                raise AssertionError(
                    "safety lifetime timeline provenance missing "
                    f"key={key}"
                )
            plan_stamp_ns = int(proposal["plan_stamp_ns"])
            safety_evaluation_stamp_ns = int(
                proposal["safety_evaluation_stamp_ns"]
            )
            safety_valid_until_ns = int(
                proposal["safety_valid_until_ns"]
            )
            proposal_received_sim_ns = int(proposal["received_sim_ns"])
            binding_ros_now_ns = int(terminal["binding_ros_now_ns"])
            terminal_safety_valid_until_ns = int(
                terminal["safety_valid_until_ns"]
            )
            publish_tick_candidates = [
                audit
                for audit in self.e2e_input_publish_ticks.values()
                if int(audit["covered_clock_first_stamp_ns"])
                <= plan_stamp_ns
                <= int(audit["covered_clock_last_stamp_ns"])
                and (
                    plan_stamp_ns
                    - int(audit["covered_clock_first_stamp_ns"])
                )
                % int(audit["covered_clock_tick_period_ns"])
                == 0
            ]
            publish_tick = (
                publish_tick_candidates[0]
                if len(publish_tick_candidates) == 1
                else None
            )
            if not publish_tick_candidates:
                observer_classification = (
                    "publisher_tick_audit_unavailable"
                )
            elif len(publish_tick_candidates) != 1:
                observer_classification = (
                    "publisher_tick_audit_ambiguous"
                )
            elif (
                int(publish_tick["clock_stamp_ns"])
                != int(publish_tick["covered_clock_last_stamp_ns"])
                or int(publish_tick["trajectory_stamp_ns"])
                != int(publish_tick["covered_clock_last_stamp_ns"])
                or int(publish_tick["plan_stamp_ns"])
                != int(publish_tick["covered_clock_last_stamp_ns"])
                or int(publish_tick["covered_clock_tick_period_ns"])
                != SIM_CLOCK_QUANTUM_NS * 10
                or int(publish_tick["covered_clock_tick_count"]) != 10
                or int(publish_tick["clock_publish_steady_ns"])
                > int(publish_tick["plan_publish_steady_ns"])
            ):
                observer_classification = (
                    "publisher_stamp_boundary_mismatch"
                )
            elif proposal_received_sim_ns < plan_stamp_ns:
                observer_classification = (
                    "publisher_aligned_observer_clock_before_plan"
                )
            else:
                observer_classification = (
                    "publisher_aligned_observer_clock_current"
                )
            observer_receipt_diagnostic = {
                "schema_version": 1,
                "classification": observer_classification,
                "observer_received_sim_ns": proposal_received_sim_ns,
                "plan_stamp_ns": plan_stamp_ns,
                "observer_minus_plan_ns": (
                    proposal_received_sim_ns - plan_stamp_ns
                ),
                "observer_before_plan": (
                    proposal_received_sim_ns < plan_stamp_ns
                ),
                "proposal_callback_steady_ns": int(
                    proposal.get("received_steady_ns", 0)
                ),
                "publisher_tick_audit": publish_tick,
                "publisher_tick_audit_count": len(
                    self.e2e_input_publish_ticks
                ),
                "publisher_tick_audit_stamp_tail": sorted(
                    self.e2e_input_publish_ticks
                )[-8:],
            }
            provenance_predicates = {
                "plan_equals_safety_evaluation": (
                    plan_stamp_ns == safety_evaluation_stamp_ns
                ),
                "proposal_deadline_equals_terminal_deadline": (
                    safety_valid_until_ns
                    == terminal_safety_valid_until_ns
                ),
                "proposal_deadline_after_plan": (
                    safety_valid_until_ns > plan_stamp_ns
                ),
                "binding_not_before_plan": (
                    binding_ros_now_ns >= plan_stamp_ns
                ),
                "binding_before_deadline": (
                    binding_ros_now_ns < safety_valid_until_ns
                ),
            }
            failed_predicates = [
                name
                for name, passed in provenance_predicates.items()
                if not passed
            ]
            if failed_predicates:
                timestamp_values_ns = {
                    "plan_stamp_ns": plan_stamp_ns,
                    "safety_evaluation_stamp_ns": (
                        safety_evaluation_stamp_ns
                    ),
                    "proposal_safety_valid_until_ns": (
                        safety_valid_until_ns
                    ),
                    "observer_received_sim_ns": (
                        proposal_received_sim_ns
                    ),
                    "binding_ros_now_ns": binding_ros_now_ns,
                    "terminal_safety_valid_until_ns": (
                        terminal_safety_valid_until_ns
                    ),
                }
                monotonic_causal_evidence = {
                    "schema_version": 1,
                    "clock": "CLOCK_MONOTONIC",
                    "full_terminal_key": key,
                    "proposal_callback_steady_ns": int(
                        proposal.get("received_steady_ns", 0)
                    ),
                    "binding_callback_monotonic_ns": int(
                        terminal["binding_callback_monotonic_ns"]
                    ),
                    "proposal_publish_audit_v1": terminal.get(
                        "proposal_publish_audit_v1"
                    ),
                }
                raise AssertionError(
                    "safety lifetime timeline provenance invalid "
                    f"key={key} "
                    f"failed_predicates={json.dumps(failed_predicates)} "
                    "predicates="
                    f"{json.dumps(provenance_predicates, sort_keys=True)} "
                    "timestamps_ns="
                    f"{json.dumps(timestamp_values_ns, sort_keys=True)} "
                    "observer_receipt_diagnostic="
                    f"{json.dumps(observer_receipt_diagnostic, sort_keys=True)} "
                    "monotonic_causal_evidence="
                    f"{json.dumps(monotonic_causal_evidence, sort_keys=True)}"
                )
            safety_lifetime_timeline.append(
                {
                    "full_terminal_key": key,
                    "disposition": str(terminal["disposition"]),
                    "plan_stamp_ns": plan_stamp_ns,
                    "safety_valid_until_ns": safety_valid_until_ns,
                    "observer_received_sim_ns": proposal_received_sim_ns,
                    "binding_ros_now_ns": binding_ros_now_ns,
                    "generation_budget_ns": (
                        safety_valid_until_ns - plan_stamp_ns
                    ),
                    "observer_delivery_elapsed_ns": (
                        proposal_received_sim_ns - plan_stamp_ns
                    ),
                    "observer_reception_margin_ns": (
                        safety_valid_until_ns - proposal_received_sim_ns
                    ),
                    "producer_binding_elapsed_ns": (
                        binding_ros_now_ns - plan_stamp_ns
                    ),
                    "observer_binding_time_delta_ns": (
                        binding_ros_now_ns - proposal_received_sim_ns
                    ),
                    "binding_margin_ns": (
                        safety_valid_until_ns - binding_ros_now_ns
                    ),
                    "observer_receipt_diagnostic": (
                        observer_receipt_diagnostic
                    ),
                }
            )
        safety_lifetime_timeline.sort(
            key=lambda entry: (
                int(entry["binding_margin_ns"]),
                tuple(entry["full_terminal_key"]),
            )
        )
        safety_lifetime_timeline_count = len(safety_lifetime_timeline)
        terminal_provenance = [
            {
                "full_terminal_key": key,
                "terminal_ordinal": ordinal,
                "disposition": str(self.e2e_terminals[key]["disposition"]),
                "binding_callback_monotonic_ns": int(
                    self.e2e_terminals[key]["binding_callback_monotonic_ns"]
                ),
                **(
                    {"proposal_publish_audit_v1": self.e2e_terminals[key][
                        "proposal_publish_audit_v1"]}
                    if "proposal_publish_audit_v1" in self.e2e_terminals[key]
                    else {}
                ),
                **(
                    {
                        "binding_snapshot_evidence_v1": (
                            self.e2e_terminals[key][
                                "binding_snapshot_evidence_v1"
                            ]
                        )
                    }
                    if self.e2e_terminals[key].get(
                        "binding_snapshot_evidence_v1"
                    )
                    is not None
                    else {}
                ),
                "lateral_authority_eligible": (
                    self.e2e_terminals[key]["lateral_authority_eligible"]
                    is True
                ),
                "observed_exact_key": (
                    key if key in self.e2e_observed_binding_keys else None
                ),
                "observer_missing": key not in self.e2e_observed_binding_keys,
                "binding_only": key not in self.e2e_proposals,
                "out_of_cohort": (
                    key[0] != self.expected_e2e_race_arm_epoch
                ),
                **(
                    {
                        "proposal_source_identity_relation": relation
                    }
                    if (
                        relation := self._proposal_source_identity_relation(
                            self.e2e_terminals[key].get(
                                "binding_snapshot_evidence_v1"
                            )
                        )
                    )
                    is not None
                    else {}
                ),
            }
            for ordinal, key in enumerate(sorted(self.e2e_terminals), start=1)
        ]
        if self.e2e_duplicate_bindings != 0:
            raise AssertionError(
                "terminal provenance duplicate binding callbacks "
                f"count={self.e2e_duplicate_bindings}"
            )
        if len(terminal_provenance) != len(self.e2e_terminals):
            raise AssertionError("terminal provenance count mismatch")
        recomputed_dispositions: dict[str, int] = {}
        for entry in terminal_provenance:
            disposition = str(entry["disposition"])
            recomputed_dispositions[disposition] = (
                recomputed_dispositions.get(disposition, 0) + 1
            )
        if recomputed_dispositions != self.binding_dispositions:
            raise AssertionError(
                "terminal provenance disposition mismatch "
                f"ledger={recomputed_dispositions} "
                f"binding={self.binding_dispositions}"
            )
        exact_terminals = [
            terminal
            for terminal in self.e2e_terminals.values()
            if terminal["disposition"] == "exact_current"
        ]
        exact_callback_monotonic_ns = sorted(
            int(terminal["binding_callback_monotonic_ns"])
            for terminal in exact_terminals
        )
        deferred_predecessor_nonauthority_count = sum(
            terminal["disposition"] == "deferred_predecessor"
            and terminal["lateral_authority_eligible"] is False
            for terminal in self.e2e_terminals.values()
        )
        deferred_predecessor_authority_violation = any(
            terminal["disposition"] == "deferred_predecessor"
            and terminal["lateral_authority_eligible"] is True
            for terminal in self.e2e_terminals.values()
        )
        nonexact_terminal_authority_violation = any(
            terminal["disposition"] != "exact_current"
            and terminal["lateral_authority_eligible"] is True
            for terminal in self.e2e_terminals.values()
        )
        if nonexact_terminal_authority_violation:
            raise AssertionError(
                "non-exact terminal claims lateral authority"
            )

        # M4 delivery is proposal-centric: a 20 Hz planner proposal must be
        # taken up once by a later PP callback before its safety deadline.  It
        # does not require a new planner identity on every 100 Hz PP cycle.
        # Repeated use of an already accepted identity is a separate sustained
        # availability gate and is not inferred from this terminal stream.
        proposal_uptake_records: list[dict] = []
        proposal_uptake_keys = (
            set(self.e2e_proposals)
            | set(self.e2e_proposal_audits)
            | set(self.e2e_terminals)
        )
        for key in sorted(proposal_uptake_keys):
            terminal = self.e2e_terminals.get(key)
            failure_reasons: list[str] = []
            if key not in self.e2e_proposal_audits:
                failure_reasons.append("missing_proposal_publish_audit")
            if terminal is None:
                failure_reasons.append("missing_terminal")
            else:
                publish_audit = terminal.get("proposal_publish_audit_v1")
                publish_return_ns = (
                    publish_audit.get("publish_call_return_monotonic_ns")
                    if isinstance(publish_audit, dict)
                    else None
                )
                callback_ns = terminal.get("binding_callback_monotonic_ns")
                if terminal.get("disposition") != "exact_current":
                    failure_reasons.append("not_exact_current")
                if terminal.get("lateral_authority_eligible") is not False:
                    failure_reasons.append("unexpected_authority")
                if self._proposal_source_identity_relation(
                    terminal.get("binding_snapshot_evidence_v1")
                ) != "exact_current":
                    failure_reasons.append("source_identity_not_current")
                if (
                    not isinstance(publish_return_ns, int)
                    or not isinstance(callback_ns, int)
                    or callback_ns < publish_return_ns
                ):
                    failure_reasons.append("binding_before_publish_return")
                if int(terminal.get("safety_margin_at_bind_ns", 0)) <= 0:
                    failure_reasons.append(
                        "binding_not_strictly_before_deadline"
                    )
            proposal_uptake_records.append(
                {
                    "full_proposal_key": key,
                    "plan_generation": key[4],
                    "status": (
                        "accepted_first_uptake"
                        if not failure_reasons
                        else "rejected_or_missing"
                    ),
                    "failure_reasons": failure_reasons,
                }
            )
        accepted_first_uptake_count = sum(
            record["status"] == "accepted_first_uptake"
            for record in proposal_uptake_records
        )
        proposal_only_keys = [
            key for key in self.e2e_proposals if key not in self.e2e_terminals
        ]
        binding_only_keys = [
            key
            for key in self.e2e_terminals
            if key not in self.e2e_proposals
        ]
        binding_observer_missing_keys = [
            key
            for key in self.e2e_terminals
            if key not in self.e2e_observed_binding_keys
        ]
        binding_observer_without_audit_keys = [
            key
            for key in self.e2e_observed_binding_keys
            if key not in self.e2e_terminals
        ]
        if sum(entry["observer_missing"] for entry in terminal_provenance) != len(
            binding_observer_missing_keys
        ):
            raise AssertionError("terminal provenance observer aggregate mismatch")
        if sum(entry["binding_only"] for entry in terminal_provenance) != len(
            binding_only_keys
        ):
            raise AssertionError("terminal provenance binding-only aggregate mismatch")
        latencies = [
            int(terminal["callback_to_audit_observer_ns"])
            for terminal in self.e2e_terminals.values()
            if int(terminal["callback_to_audit_observer_ns"]) >= 0
        ]
        source_ages_at_bind = [
            int(terminal["source_age_at_bind_ns"])
            for terminal in self.e2e_terminals.values()
            if int(terminal["source_age_at_bind_ns"]) >= 0
        ]
        safety_margins_at_bind = [
            int(terminal["safety_margin_at_bind_ns"])
            for terminal in self.e2e_terminals.values()
        ]
        same_base_divergence_count = sum(
            1
            for binding in binding_only_keys
            if any(
                binding[8:] == proposal[8:]
                for proposal in proposal_only_keys
            )
        )

        def selected(values: list[int], numerator: int, denominator: int) -> int:
            if not values:
                return 0
            ordered = sorted(values)
            return ordered[
                min(
                    len(ordered) - 1,
                    len(ordered) * numerator // denominator,
                )
            ]

        return {
            "expected_race_arm_epoch": self.expected_e2e_race_arm_epoch,
            "fixture_proposal_seen": len(self.e2e_proposals),
            "binding_callback_entries": len(self.e2e_terminals),
            "terminal_bindings": len(self.e2e_terminals),
            "duplicate_proposals": self.e2e_duplicate_proposals,
            "duplicate_bindings": self.e2e_duplicate_bindings,
            "proposal_publish_audit_entries": len(self.e2e_proposal_audits),
            "planner_input_audit_entries": len(self.e2e_planner_input_audits),
            "duplicate_proposal_publish_audits": (
                self.e2e_duplicate_proposal_audits
            ),
            "duplicate_planner_input_audits": (
                self.e2e_duplicate_planner_input_audits
            ),
            "out_of_cohort_proposals": self.e2e_out_of_cohort_proposals,
            "out_of_cohort_bindings": self.e2e_out_of_cohort_bindings,
            "out_of_cohort_binding_audits": (
                self.e2e_out_of_cohort_binding_audits
            ),
            "out_of_cohort_proposal_audits": (
                self.e2e_out_of_cohort_proposal_audits
            ),
            "out_of_cohort_planner_input_audits": (
                self.e2e_out_of_cohort_planner_input_audits
            ),
            "out_of_cohort_samples_v1": {
                "schema_version": 1,
                "per_stream_limit": E2E_OUT_OF_COHORT_SAMPLE_LIMIT,
                "overflow": self.e2e_out_of_cohort_sample_overflow,
                "samples": {
                    stream: list(samples)
                    for stream, samples in (
                        self.e2e_out_of_cohort_samples.items()
                    )
                },
            },
            "fixture_only_identities": len(proposal_only_keys),
            "binding_only_identities": len(binding_only_keys),
            "intersection_identities": len(
                self.e2e_proposals.keys() & self.e2e_terminals.keys()
            ),
            "binding_observer_missing": len(
                binding_observer_missing_keys
            ),
            "binding_observer_without_audit": len(
                binding_observer_without_audit_keys
            ),
            "callback_to_audit_observer_p90_ns": selected(
                latencies, 90, 100
            ),
            "callback_to_audit_observer_p99_ns": selected(
                latencies, 99, 100
            ),
            "callback_to_audit_observer_max_ns": max(
                latencies, default=0
            ),
            "source_age_at_bind_max_ns": max(
                source_ages_at_bind, default=0
            ),
            "safety_margin_at_bind_min_ns": min(
                safety_margins_at_bind, default=0
            ),
            "safety_lifetime_timeline_count": (
                safety_lifetime_timeline_count
            ),
            "safety_lifetime_timeline_missing": (
                safety_lifetime_timeline_missing
            ),
            "safety_lifetime_timeline_truncated": (
                False
            ),
            "safety_lifetime_timeline": safety_lifetime_timeline,
            "same_base_divergence_count": same_base_divergence_count,
            "exact_current_terminal_count": len(exact_terminals),
            "deferred_predecessor_nonauthority_count": (
                deferred_predecessor_nonauthority_count
            ),
            "deferred_predecessor_authority_violation": (
                deferred_predecessor_authority_violation
            ),
            "nonexact_terminal_authority_violation": False,
            "proposal_uptake_contract_v1": {
                "schema_version": 1,
                "expected_proposal_count": len(proposal_uptake_records),
                "accepted_first_uptake_count": accepted_first_uptake_count,
                "failed_or_missing_count": (
                    len(proposal_uptake_records)
                    - accepted_first_uptake_count
                ),
                "all_proposals_taken_up_once_before_deadline": (
                    bool(proposal_uptake_records)
                    and accepted_first_uptake_count
                    == len(proposal_uptake_records)
                ),
                "pp_hold_cycles_evaluated": False,
                "sustained_10_cycle_gate": "not_evaluated",
                "records": proposal_uptake_records,
            },
            "exact_current_terminal_max_monotonic_gap_ns": max(
                (
                    current - previous
                    for previous, current in zip(
                        exact_callback_monotonic_ns,
                        exact_callback_monotonic_ns[1:],
                    )
                ),
                default=0,
            ),
            "fixture_only_keys": proposal_only_keys[:5],
            "binding_only_keys": binding_only_keys[:5],
            "binding_observer_missing_keys": (
                binding_observer_missing_keys[:5]
            ),
            "binding_observer_without_audit_keys": (
                binding_observer_without_audit_keys[:5]
            ),
            # One-to-one terminal mapping.  Unlike the diagnostic key samples
            # above, retain every deferred terminal target identity.
            "terminal_provenance": terminal_provenance,
        }

    def _receive_shadow_snapshot(
        self, message: ControllerBaseTrajectorySnapshot
    ) -> None:
        self.shadow_snapshot_count += 1
        self.max_snapshot_race_arm_epoch = max(
            self.max_snapshot_race_arm_epoch,
            int(message.race_arm_epoch),
        )
        self.snapshot_controller_instance_ids.add(
            message.controller_instance_id
        )
        if (
            message.schema_version
            != ControllerBaseTrajectorySnapshot.SCHEMA_V1_SHADOW
            or message.authority_eligible
        ):
            self.shadow_snapshot_invalid = True

    def _append_out_of_cohort_sample(
        self,
        stream: str,
        key: tuple[int, ...],
        *,
        received_steady_ns: int,
        received_sim_ns: int,
        disposition: str = "",
        lateral_authority_eligible: bool = False,
        binding_callback_monotonic_ns: int = 0,
        binding_ros_now_ns: int = 0,
    ) -> None:
        samples = self.e2e_out_of_cohort_samples[stream]
        if len(samples) >= E2E_OUT_OF_COHORT_SAMPLE_LIMIT:
            self.e2e_out_of_cohort_sample_overflow = True
            return
        samples.append(
            {
                "schema_version": 1,
                "stream": stream,
                "event_ordinal": len(samples) + 1,
                "expected_race_arm_epoch": (
                    self.expected_e2e_race_arm_epoch
                ),
                "observed_race_arm_epoch": key[0],
                "full_proposal_key": key,
                "disposition": disposition,
                "lateral_authority_eligible": (
                    lateral_authority_eligible
                ),
                "received_steady_ns": received_steady_ns,
                "received_sim_ns": received_sim_ns,
                "binding_callback_monotonic_ns": (
                    binding_callback_monotonic_ns
                ),
                "binding_ros_now_ns": binding_ros_now_ns,
            }
        )

    def _receive_authorized_proposal(
        self, message: AuthorizedCartesianTrajectory
    ) -> None:
        if self.e2e_active:
            proposal_epoch = int(message.plan_sample_key.race_arm_epoch)
            if proposal_epoch != self.expected_e2e_race_arm_epoch:
                self._append_out_of_cohort_sample(
                    "proposal",
                    self._proposal_key_from_message(message),
                    received_steady_ns=time.monotonic_ns(),
                    received_sim_ns=(
                        self.node.get_clock().now().nanoseconds
                    ),
                    lateral_authority_eligible=(
                        message.authority_eligible is True
                    ),
                )
                if message.authority_eligible:
                    self.binding_invalid = True
                self.e2e_out_of_cohort_proposals += 1
                return
        self.authorized_proposal_count += 1
        if (
            message.schema_version
            != AuthorizedCartesianTrajectory.SCHEMA_V1_SHADOW
            or message.authority_eligible
        ):
            self.binding_invalid = True
        if not self.e2e_active:
            return
        key = self._proposal_key_from_message(message)
        if key in self.e2e_proposals:
            self.e2e_duplicate_proposals += 1
            return
        received_steady_ns = time.monotonic_ns()
        received_sim_ns = self.node.get_clock().now().nanoseconds
        plan_stamp_ns = self._stamp_ns(message.plan_stamp)
        safety_evaluation_stamp_ns = self._stamp_ns(
            message.safety_evaluation_stamp
        )
        source_stamp_ns = self._stamp_ns(message.base_source_stamp)
        safety_valid_until_ns = self._stamp_ns(message.safety_valid_until)
        source_age_ns = received_sim_ns - source_stamp_ns
        safety_margin_ns = safety_valid_until_ns - received_sim_ns
        if source_age_ns < 0:
            self.binding_invalid = True
        self.e2e_proposals[key] = {
            "received_steady_ns": received_steady_ns,
            "received_sim_ns": received_sim_ns,
            "plan_stamp_ns": plan_stamp_ns,
            "safety_evaluation_stamp_ns": (
                safety_evaluation_stamp_ns
            ),
            "source_stamp_ns": source_stamp_ns,
            "source_age_ns": source_age_ns,
            "safety_valid_until_ns": safety_valid_until_ns,
            "safety_margin_ns": safety_margin_ns,
        }

    def _receive_binding(self, message: String) -> None:
        received_steady_ns = time.monotonic_ns()
        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.binding_invalid = True
            return
        if self.e2e_active:
            proposal_epoch = value.get("proposal_race_arm_epoch")
            key = self._proposal_key_from_binding(value)
            if key is None:
                self.binding_invalid = True
                return
            if proposal_epoch != self.expected_e2e_race_arm_epoch:
                binding_sim_ns = self._json_stamp_ns(
                    value, "binding_ros_now"
                )
                if (
                    value.get("schema_version") != 1
                    or value.get("shadow_only") is not True
                    or value.get("lateral_authority_eligible") is True
                    or value.get("disposition")
                    not in E2E_BINDING_DISPOSITIONS
                    or binding_sim_ns is None
                ):
                    self.binding_invalid = True
                self._append_out_of_cohort_sample(
                    "binding",
                    key,
                    received_steady_ns=received_steady_ns,
                    received_sim_ns=(
                        self.node.get_clock().now().nanoseconds
                    ),
                    disposition=str(value.get("disposition", "")),
                    lateral_authority_eligible=(
                        value.get("lateral_authority_eligible") is True
                    ),
                    binding_callback_monotonic_ns=(
                        value["binding_callback_monotonic_ns"]
                        if isinstance(
                            value.get("binding_callback_monotonic_ns"), int
                        )
                        and not isinstance(
                            value.get("binding_callback_monotonic_ns"), bool
                        )
                        else 0
                    ),
                    binding_ros_now_ns=(
                        binding_sim_ns if binding_sim_ns is not None else 0
                    ),
                )
                self.e2e_out_of_cohort_bindings += 1
                return
            self.binding_count += 1
            self.e2e_observed_binding_keys.add(key)
            return
        self.binding_count += 1
        self._record_legacy_binding(value)

    def _receive_proposal_publish_audit(self, message: String) -> None:
        if not self.e2e_active:
            return
        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.e2e_proposal_audit_invalid = True
            return
        key = self._proposal_key_from_binding(value)
        timing_names = (
            "capture_monotonic_ns",
            "worker_dequeue_monotonic_ns",
            "worker_build_begin_monotonic_ns",
            "publish_call_entry_monotonic_ns",
            "publish_call_return_monotonic_ns",
        )
        if (
            key is None
            or value.get("schema_version") != 1
            or any(
                not isinstance(value.get(name), int)
                or isinstance(value.get(name), bool)
                or int(value[name]) <= 0
                for name in timing_names
            )
        ):
            self.e2e_proposal_audit_invalid = True
            return
        timings = [int(value[name]) for name in timing_names]
        if timings != sorted(timings):
            self.e2e_proposal_audit_invalid = True
            return
        if key[0] != self.expected_e2e_race_arm_epoch:
            self._append_out_of_cohort_sample(
                "proposal_audit",
                key,
                received_steady_ns=time.monotonic_ns(),
                received_sim_ns=self.node.get_clock().now().nanoseconds,
            )
            self.e2e_out_of_cohort_proposal_audits += 1
            return
        if key in self.e2e_proposal_audits:
            self.e2e_duplicate_proposal_audits += 1
            self.e2e_proposal_audit_invalid = True
            return
        if len(self.e2e_proposal_audits) >= E2E_PROPOSAL_AUDIT_LEDGER_LIMIT:
            self.e2e_proposal_audit_overflow = True
            self.e2e_proposal_audit_invalid = True
            return
        self.e2e_proposal_audits[key] = {
            name: int(value[name]) for name in timing_names
        }

    def _receive_planner_input_audit(self, message: String) -> None:
        if not self.e2e_active:
            return
        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.e2e_planner_input_audit_invalid = True
            return
        key = self._proposal_key_from_binding(value)
        timing_names = (
            "base_snapshot_callback_entry_monotonic_ns",
            "planning_timer_entry_monotonic_ns",
            "planner_update_begin_monotonic_ns",
            "capture_monotonic_ns",
        )
        source_names = (
            "proposal_source_stamp_sec",
            "proposal_source_stamp_nanosec",
            "proposal_source_generation",
        )
        if (
            key is None
            or value.get("schema_version") != 1
            or any(
                not isinstance(value.get(name), int)
                or isinstance(value.get(name), bool)
                or int(value[name]) <= 0
                for name in timing_names
            )
            or any(
                not isinstance(value.get(name), int)
                or isinstance(value.get(name), bool)
                for name in source_names
            )
            or value["proposal_source_stamp_nanosec"] < 0
            or value["proposal_source_stamp_nanosec"] >= 1_000_000_000
            or value["proposal_source_generation"] <= 0
        ):
            self.e2e_planner_input_audit_invalid = True
            return
        timings = [int(value[name]) for name in timing_names]
        if timings != sorted(timings):
            self.e2e_planner_input_audit_invalid = True
            return
        if key[0] != self.expected_e2e_race_arm_epoch:
            self._append_out_of_cohort_sample(
                "planner_input_audit",
                key,
                received_steady_ns=time.monotonic_ns(),
                received_sim_ns=self.node.get_clock().now().nanoseconds,
            )
            self.e2e_out_of_cohort_planner_input_audits += 1
            return
        if key in self.e2e_planner_input_audits:
            self.e2e_duplicate_planner_input_audits += 1
            self.e2e_planner_input_audit_invalid = True
            return
        if len(self.e2e_planner_input_audits) >= E2E_PROPOSAL_AUDIT_LEDGER_LIMIT:
            self.e2e_planner_input_audit_overflow = True
            self.e2e_planner_input_audit_invalid = True
            return
        self.e2e_planner_input_audits[key] = {
            name: int(value[name]) for name in (*timing_names, *source_names)
        }

    def _receive_audited_binding(self, message: String) -> None:
        received_steady_ns = time.monotonic_ns()
        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.binding_invalid = True
            return
        if not self.e2e_active:
            return
        proposal_epoch = value.get("proposal_race_arm_epoch")
        key = self._proposal_key_from_binding(value)
        snapshot_evidence = self._binding_snapshot_evidence(value)
        binding_sim_ns = self._json_stamp_ns(value, "binding_ros_now")
        callback_monotonic_ns = value.get("binding_callback_monotonic_ns")
        if (
            key is None
            or snapshot_evidence is None
            or binding_sim_ns is None
            or not isinstance(callback_monotonic_ns, int)
            or isinstance(callback_monotonic_ns, bool)
            or callback_monotonic_ns <= 0
            or value.get("schema_version") != 1
            or value.get("shadow_only") is not True
            or value.get("disposition") not in E2E_BINDING_DISPOSITIONS
        ):
            self.binding_invalid = True
            return
        if proposal_epoch != self.expected_e2e_race_arm_epoch:
            self._append_out_of_cohort_sample(
                "binding_audit",
                key,
                received_steady_ns=received_steady_ns,
                received_sim_ns=self.node.get_clock().now().nanoseconds,
                disposition=str(value.get("disposition", "")),
                lateral_authority_eligible=(
                    value.get("lateral_authority_eligible") is True
                ),
                binding_callback_monotonic_ns=callback_monotonic_ns,
                binding_ros_now_ns=binding_sim_ns,
            )
            if value.get("lateral_authority_eligible") is True:
                self.binding_invalid = True
            self.e2e_out_of_cohort_binding_audits += 1
            return
        if key in self.e2e_terminals:
            self.e2e_duplicate_bindings += 1
            self.binding_invalid = True
            return
        self._record_e2e_terminal(key, value, received_steady_ns)

    @staticmethod
    def _binding_snapshot_evidence(
        value: dict,
    ) -> dict[str, object] | None:
        proposal_names = (
            "proposal_source_stamp_sec",
            "proposal_source_stamp_nanosec",
            "proposal_source_generation",
        )
        current_names = (
            "current_race_arm_epoch",
            "current_controller_instance_id",
            "current_controller_sequence",
            "current_base_lease_id",
            "current_source_stamp_sec",
            "current_source_stamp_nanosec",
            "current_source_generation",
            "current_first_source_index",
            "current_last_source_index",
            "current_nearest_source_index",
        )
        if (
            value.get("test_audit_schema_version") != 1
            or (
                value.get("previous_present") is not True
                and value.get("previous_present") is not False
            )
            or any(
                not isinstance(value.get(name), int)
                or isinstance(value.get(name), bool)
                for name in (*proposal_names, *current_names)
            )
        ):
            return None
        previous_present = value["previous_present"]
        previous_names = (
            "previous_race_arm_epoch",
            "previous_controller_instance_id",
            "previous_controller_sequence",
            "previous_base_lease_id",
            "previous_source_stamp_sec",
            "previous_source_stamp_nanosec",
            "previous_source_generation",
            "previous_first_source_index",
            "previous_last_source_index",
            "previous_nearest_source_index",
        )
        if previous_present and any(
            not isinstance(value.get(name), int)
            or isinstance(value.get(name), bool)
            for name in previous_names
        ):
            return None
        if (
            value["proposal_source_stamp_nanosec"] < 0
            or value["proposal_source_stamp_nanosec"] >= 1_000_000_000
            or value["current_source_stamp_nanosec"] < 0
            or value["current_source_stamp_nanosec"] >= 1_000_000_000
            or value["current_controller_sequence"] <= 0
            or value["current_base_lease_id"] <= 0
            or (
                previous_present
                and (
                    value["previous_source_stamp_nanosec"] < 0
                    or value["previous_source_stamp_nanosec"]
                    >= 1_000_000_000
                    or value["previous_controller_sequence"] <= 0
                    or value["previous_base_lease_id"] <= 0
                )
            )
        ):
            return None
        return {
            "schema_version": 1,
            "proposal": {
                name.removeprefix("proposal_"): value[name]
                for name in proposal_names
            },
            "current": {
                name.removeprefix("current_"): value[name]
                for name in current_names
            },
            "previous": (
                {
                    name.removeprefix("previous_"): value[name]
                    for name in previous_names
                }
                if previous_present
                else None
            ),
        }

    @staticmethod
    def _proposal_source_identity_relation(
        snapshot_evidence: object,
    ) -> str | None:
        if not isinstance(snapshot_evidence, dict):
            return None
        proposal = snapshot_evidence.get("proposal")
        current = snapshot_evidence.get("current")
        previous = snapshot_evidence.get("previous")
        if not isinstance(proposal, dict) or not isinstance(current, dict):
            return None

        def source_identity_matches(candidate: object) -> bool:
            return isinstance(candidate, dict) and all(
                proposal.get(name) == candidate.get(name)
                for name in (
                    "source_stamp_sec",
                    "source_stamp_nanosec",
                    "source_generation",
                )
            )

        if source_identity_matches(current):
            return "exact_current"
        if source_identity_matches(previous):
            return "exact_previous"
        return "neither_current_nor_previous"

    def _record_e2e_terminal(
        self, key: tuple[int, ...], value: dict, received_steady_ns: int
    ) -> None:
        callback_monotonic_ns = value.get("binding_callback_monotonic_ns")
        binding_sim_ns = self._json_stamp_ns(value, "binding_ros_now")
        source_stamp_ns = self._json_stamp_ns(value, "proposal_source_stamp")
        safety_valid_until_ns = self._json_stamp_ns(
            value, "proposal_safety_valid_until"
        )
        if (
            not isinstance(callback_monotonic_ns, int)
            or callback_monotonic_ns <= 0
            or binding_sim_ns is None
            or source_stamp_ns is None
            or safety_valid_until_ns is None
        ):
            self.binding_invalid = True
            callback_monotonic_ns = 0
            binding_sim_ns = 0
            source_stamp_ns = 0
            safety_valid_until_ns = 0
        exact = self._binding_is_exact(value)
        disposition = str(value.get("disposition", "missing"))
        self.binding_dispositions[disposition] = (
            self.binding_dispositions.get(disposition, 0) + 1
        )
        if exact:
            self.exact_binding_count += 1
        else:
            self.binding_invalid = True
        source_age_at_bind_ns = binding_sim_ns - source_stamp_ns
        if source_age_at_bind_ns < 0:
            self.binding_invalid = True
        callback_to_audit_observer_ns = (
            received_steady_ns - callback_monotonic_ns
        )
        if callback_to_audit_observer_ns < 0:
            self.binding_invalid = True
        snapshot_evidence = self._binding_snapshot_evidence(value)
        self.e2e_terminals[key] = {
            "disposition": disposition,
            "binding_callback_monotonic_ns": callback_monotonic_ns,
            "binding_ros_now_ns": binding_sim_ns,
            "safety_valid_until_ns": safety_valid_until_ns,
            "lateral_authority_eligible": (
                value.get("lateral_authority_eligible") is True
            ),
            # The test-only RELIABLE audit is emitted after callback entry and
            # carries the terminal result.  This duration is diagnostic only;
            # it is not folded into the PP callback WCET.
            "callback_to_audit_observer_ns": callback_to_audit_observer_ns,
            "source_age_at_bind_ns": source_age_at_bind_ns,
            "safety_margin_at_bind_ns": (
                safety_valid_until_ns - binding_sim_ns
            ),
            "binding_snapshot_evidence_v1": snapshot_evidence,
        }

    def _binding_is_exact(self, value: dict) -> bool:
        exact_tuple = (
            value.get("current_source_stamp_sec")
            == value.get("proposal_source_stamp_sec")
            and value.get("current_source_stamp_nanosec")
            == value.get("proposal_source_stamp_nanosec")
            and value.get("current_source_generation")
            == value.get("proposal_source_generation")
            and value.get("current_nearest_source_index")
            == value.get("proposal_nearest_source_index")
        )
        exact = (
            value.get("schema_version") == 1
            and value.get("shadow_only") is True
            and value.get("lateral_authority_eligible") is False
            and value.get("disposition") == "exact_current"
            and value.get("validation_error") == 0
            and value.get("session_generation")
            == self.expected_session_generation
            and value.get("session_nonce") == self.expected_session_nonce
            and value.get("current_race_arm_epoch")
            == (
                self.expected_e2e_race_arm_epoch
                if self.e2e_active
                else 1
            )
            and value.get("proposal_race_arm_epoch")
            == (
                self.expected_e2e_race_arm_epoch
                if self.e2e_active
                else value.get("proposal_race_arm_epoch")
            )
            and value.get("controller_instance_id")
            in self.snapshot_controller_instance_ids
            and value.get("current_controller_sequence", 0) > 0
            and exact_tuple
        )
        return exact

    def _record_legacy_binding(self, value: dict) -> None:
        exact = self._binding_is_exact(value)
        disposition = str(value.get("disposition", "missing"))
        self.binding_dispositions[disposition] = (
            self.binding_dispositions.get(disposition, 0) + 1
        )
        if exact:
            self.exact_binding_count += 1
        else:
            self.binding_invalid = True

    def _receive(self, key: str, message: object) -> None:
        if key == "command_envelope":
            self.latest_sequence = message.command_sequence
        sample_identity = self._sample_identity(message)
        if self.cohort_alignment_enabled:
            if (
                self.enabled
                and len(self.materialized_sequences)
                >= CAPTURE_WINDOW_COUNT
            ):
                return
            semantic = canonical_debug(key, message)
            record = {
                "topic": key,
                "identity": sample_identity,
                "semantic": semantic,
                "payload_hash": canonical_payload_hash(semantic),
                "steering_abs": (
                    abs(message.lateral.steering_tire_angle)
                    if key in ("command", "raw_command")
                    else None
                ),
            }
            if not self.enabled:
                self._record_startup_sample(record)
            self._stage_alignment_sample(record)
            if self.enabled:
                self._materialize_ready_cycles()
            return
        if not self.enabled:
            return
        if key in ("command", "raw_command"):
            self.max_abs_steering[key] = max(
                self.max_abs_steering.get(key, 0.0),
                abs(message.lateral.steering_tire_angle),
            )
        values = self.values.setdefault(key, [])
        if len(values) < CAPTURE_WINDOW_COUNT:
            semantic = canonical_debug(key, message)
            self.first_debug.setdefault(key, semantic)
            values.append(canonical_payload_hash(semantic))
            if key in ("command_envelope", "execution_envelope"):
                self.canonical_semantics.setdefault(key, []).append(semantic)
            identities = self.identities.setdefault(key, [])
            identities.append(sample_identity)

    def _record_startup_sample(
        self,
        record: dict[str, object],
    ) -> None:
        if len(self.startup_ledger) >= STARTUP_LEDGER_MAX_MESSAGES:
            self.startup_ledger_overflow = True
            return
        self.startup_ledger.append(
            {
                "topic": record["topic"],
                "identity": record["identity"],
                "semantic": record["semantic"],
            }
        )

    def _stage_alignment_sample(
        self,
        record: dict[str, object],
    ) -> None:
        identity = record["identity"]
        topic = str(record["topic"])
        if not isinstance(identity, tuple):
            raise AssertionError("alignment sample identity invalid")
        stamp_ns = int(identity[0])
        if (
            self.capture_after_stamp_ns is not None
            and stamp_ns <= self.capture_after_stamp_ns
        ):
            return
        by_topic = self.alignment_cycles.setdefault(stamp_ns, {})
        if topic in by_topic:
            records = by_topic[topic]
            identity_bearing = topic in (
                "command_envelope",
                "execution_envelope",
            )
            if identity_bearing and any(
                existing["identity"] == identity for existing in records
            ):
                self.alignment_stage_mismatch_reason = (
                    "duplicate identity-bearing sample "
                    f"topic={topic} identity={identity}"
                )
                raise AssertionError(
                    "alignment stage duplicate sample "
                    + self.alignment_stage_mismatch_reason
                )
            if (
                self.alignment_stage_message_count
                >= ALIGNMENT_STAGE_MAX_MESSAGES
            ):
                self.alignment_stage_overflow = True
                raise AssertionError(
                    "alignment stage overflow "
                    f"capacity={ALIGNMENT_STAGE_MAX_MESSAGES}"
                )
            records.append(record)
            self.alignment_stage_message_count += 1
            self.alignment_stage_max_message_count = max(
                self.alignment_stage_max_message_count,
                self.alignment_stage_message_count,
            )
            return
        if (
            self.alignment_stage_message_count
            >= ALIGNMENT_STAGE_MAX_MESSAGES
        ):
            self.alignment_stage_overflow = True
            raise AssertionError(
                "alignment stage overflow "
                f"capacity={ALIGNMENT_STAGE_MAX_MESSAGES}"
            )
        by_topic[topic] = [record]
        self.alignment_stage_message_count += 1
        self.alignment_stage_max_message_count = max(
            self.alignment_stage_max_message_count,
            self.alignment_stage_message_count,
        )

    def _materialize_ready_cycles(self) -> None:
        while (
            self.next_expected_sequence is not None
            and len(self.materialized_sequences) < CAPTURE_WINDOW_COUNT
        ):
            candidates = []
            for stamp_ns, by_topic in self.alignment_cycles.items():
                envelopes = by_topic.get("command_envelope", ())
                matching_envelopes = [
                    record
                    for record in envelopes
                    if (
                        isinstance(record.get("identity"), tuple)
                        and int(record["identity"][2])
                        == self.next_expected_sequence
                    )
                ]
                if len(matching_envelopes) == 1:
                    candidates.append((stamp_ns, by_topic))
                elif len(matching_envelopes) > 1:
                    raise AssertionError(
                        "alignment stage ambiguous expected envelope "
                        f"sequence={self.next_expected_sequence} "
                        f"stamp={stamp_ns}"
                    )
            if len(candidates) > 1:
                self.alignment_stage_mismatch_reason = (
                    "multiple stamps for expected sequence "
                    f"{self.next_expected_sequence}: "
                    f"{sorted(stamp for stamp, _ in candidates)}"
                )
                raise AssertionError(
                    "alignment stage ambiguous expected cohort "
                    + self.alignment_stage_mismatch_reason
                )
            if not candidates:
                return
            stamp_ns, by_topic = candidates[0]
            if set(by_topic) != CAPTURE_TOPICS:
                return
            selected_records = self._select_expected_cycle_records(
                by_topic,
                self.next_expected_sequence,
            )
            if selected_records is None:
                return
            selected_by_topic = {
                topic: [record]
                for topic, record in selected_records.items()
            }
            if not self._completed_cycle_is_exact(selected_by_topic):
                self.alignment_stage_mismatch_reason = (
                    "semantic mismatch for expected sequence "
                    f"{self.next_expected_sequence} stamp={stamp_ns}"
                )
                raise AssertionError(
                    "alignment stage expected cohort invalid "
                    + self.alignment_stage_mismatch_reason
                )
            if len(by_topic["command_envelope"]) == 1:
                for topic in ("command", "raw_command", "tracking"):
                    selected_semantic = selected_records[topic]["semantic"]
                    if any(
                        record.get("semantic") != selected_semantic
                        for record in by_topic[topic]
                    ):
                        raise AssertionError(
                            "alignment stage unmatched mirror variant "
                            f"topic={topic} "
                            f"sequence={self.next_expected_sequence}"
                        )
            first_records = selected_records
            identities = {
                topic: record["identity"]
                for topic, record in first_records.items()
            }
            if any(
                not isinstance(identity, tuple)
                or int(identity[0]) != stamp_ns
                for identity in identities.values()
            ):
                raise AssertionError(
                    "alignment stage expected cohort stamp mismatch"
                )
            envelope_identity = identities["command_envelope"]
            execution_identity = identities["execution_envelope"]
            tracking_identity = identities["tracking"]
            if (
                int(envelope_identity[2])
                != self.next_expected_sequence
                or not self._execution_identity_matches(
                    execution_identity,
                    envelope_identity,
                )
                or int(tracking_identity[3]) != int(envelope_identity[3])
                or (
                    self.startup_anchor_producer_instance_id is not None
                    and int(envelope_identity[1])
                    != self.startup_anchor_producer_instance_id
                )
            ):
                self.alignment_stage_mismatch_reason = (
                    "identity mismatch for expected sequence "
                    f"{self.next_expected_sequence} "
                    f"envelope={envelope_identity} "
                    f"execution={execution_identity} "
                    f"tracking={tracking_identity}"
                )
                raise AssertionError(
                    "alignment stage expected cohort identity invalid "
                    + self.alignment_stage_mismatch_reason
                )
            for topic in sorted(CAPTURE_TOPICS):
                record = first_records[topic]
                semantic = record["semantic"]
                payload_hash = record["payload_hash"]
                identity = record["identity"]
                if not isinstance(semantic, dict) or not isinstance(
                    payload_hash, str
                ):
                    raise AssertionError(
                        "alignment stage canonical record invalid"
                    )
                self.values.setdefault(topic, []).append(payload_hash)
                self.identities.setdefault(topic, []).append(identity)
                self.first_debug.setdefault(topic, semantic)
                if topic in ("command_envelope", "execution_envelope"):
                    self.canonical_semantics.setdefault(topic, []).append(
                        semantic
                    )
                steering_abs = record["steering_abs"]
                if steering_abs is not None:
                    self.max_abs_steering[topic] = max(
                        self.max_abs_steering.get(topic, 0.0),
                        float(steering_abs),
                    )
            materialized_sequence = self.next_expected_sequence
            self.materialized_sequences.append(materialized_sequence)
            self.next_expected_sequence += 1
            for topic in CAPTURE_TOPICS:
                by_topic[topic].remove(first_records[topic])
                self.alignment_stage_message_count -= 1
            if not by_topic["command_envelope"]:
                removed_count = sum(
                    len(records) for records in by_topic.values()
                )
                del self.alignment_cycles[stamp_ns]
                self.alignment_stage_message_count -= removed_count

    @staticmethod
    def _execution_identity_matches(
        execution_identity: object,
        envelope_identity: object,
    ) -> bool:
        return (
            isinstance(execution_identity, tuple)
            and isinstance(envelope_identity, tuple)
            and execution_identity == envelope_identity
        )

    @staticmethod
    def _select_expected_cycle_records(
        by_topic: dict[str, list[dict[str, object]]],
        expected_sequence: int,
    ) -> dict[str, dict[str, object]] | None:
        envelope_matches = [
            record
            for record in by_topic.get("command_envelope", ())
            if (
                isinstance(record.get("identity"), tuple)
                and int(record["identity"][2]) == expected_sequence
            )
        ]
        if len(envelope_matches) != 1:
            if len(envelope_matches) > 1:
                raise AssertionError(
                    "alignment stage ambiguous command envelope "
                    f"sequence={expected_sequence}"
                )
            return None
        command_envelope_record = envelope_matches[0]
        envelope_identity = command_envelope_record["identity"]
        command_envelope = command_envelope_record["semantic"]
        if not isinstance(envelope_identity, tuple) or not isinstance(
            command_envelope, dict
        ):
            raise AssertionError("alignment stage envelope record invalid")

        execution_matches = [
            record
            for record in by_topic.get("execution_envelope", ())
            if (
                Capture._execution_identity_matches(
                    record.get("identity"),
                    envelope_identity,
                )
                and isinstance(record.get("semantic"), dict)
                and record["semantic"].get("command_envelope")
                == command_envelope
            )
        ]
        if len(execution_matches) != 1:
            if len(execution_matches) > 1:
                raise AssertionError(
                    "alignment stage ambiguous execution envelope "
                    f"sequence={expected_sequence}"
                )
            partial_identity_matches = [
                record
                for record in by_topic.get("execution_envelope", ())
                if (
                    isinstance(record.get("identity"), tuple)
                    and record["identity"][1:4]
                    == envelope_identity[1:4]
                )
            ]
            if partial_identity_matches:
                raise AssertionError(
                    "alignment stage execution envelope full identity "
                    f"mismatch sequence={expected_sequence}"
                )
            return None

        command_matches = [
            record
            for record in by_topic.get("command", ())
            if record.get("semantic") == command_envelope.get("command")
        ]
        if not command_matches:
            return None

        tracking_fields = (
            "plan_generation",
            "mpc_horizon_usable",
            "pp_command_fresh",
            "trajectory_tracking_usable",
            "lateral_stop_authority_kind",
            "lateral_stop_transaction_pass_direction",
            "lateral_stop_authority_token",
            "reason",
        )
        tracking_matches = [
            record
            for record in by_topic.get("tracking", ())
            if (
                isinstance(record.get("semantic"), dict)
                and all(
                    record["semantic"].get(field)
                    == command_envelope.get(field)
                    for field in tracking_fields
                )
            )
        ]
        if not tracking_matches:
            return None

        raw_matches = [
            record
            for record in by_topic.get("raw_command", ())
            if record.get("semantic") == command_envelope.get("command")
        ]
        if not raw_matches:
            return None
        expected_command_digest = canonical_payload_hash(
            command_envelope["command"]
        )
        raw_hashes = {
            canonical_payload_hash(record["semantic"])
            for record in raw_matches
            if isinstance(record.get("semantic"), dict)
        }
        if raw_hashes != {expected_command_digest}:
            raise AssertionError(
                "alignment stage raw command mirror digest mismatch "
                f"sequence={expected_sequence} hashes={sorted(raw_hashes)}"
            )

        return {
            "command": command_matches[0],
            "raw_command": raw_matches[0],
            "tracking": tracking_matches[0],
            "command_envelope": command_envelope_record,
            "execution_envelope": execution_matches[0],
        }

    @staticmethod
    def _completed_cycle_is_exact(
        by_topic: dict[str, list[dict[str, object]]],
    ) -> bool:
        if set(by_topic) != CAPTURE_TOPICS or any(
            len(records) != 1 for records in by_topic.values()
        ):
            return False
        tracking = by_topic["tracking"][0]["semantic"]
        command = by_topic["command"][0]["semantic"]
        command_envelope = by_topic["command_envelope"][0]["semantic"]
        execution_envelope = by_topic["execution_envelope"][0]["semantic"]
        if (
            not isinstance(tracking, dict)
            or not isinstance(command, dict)
            or not isinstance(command_envelope, dict)
            or not isinstance(execution_envelope, dict)
        ):
            return False
        if execution_envelope.get("command_envelope") != command_envelope:
            return False
        if command_envelope.get("command") != command:
            return False
        tracking_fields = (
            "plan_generation",
            "mpc_horizon_usable",
            "pp_command_fresh",
            "trajectory_tracking_usable",
            "lateral_stop_authority_kind",
            "lateral_stop_transaction_pass_direction",
            "lateral_stop_authority_token",
            "reason",
        )
        return all(
            command_envelope.get(field) == tracking.get(field)
            for field in tracking_fields
        )

    def validate_fresh_reanchor_interval(
        self,
        minimum_sequence: int,
        *,
        minimum_stamp_exclusive_ns: int,
        expected_producer_instance_id: int,
    ) -> None:
        self.reanchor_fence_invalid = None

        def fail(
            kind: str,
            stamp_ns: int,
            by_topic: dict[str, list[dict[str, object]]],
            **details: object,
        ) -> None:
            issue = {
                "kind": kind,
                "stamp_ns": stamp_ns,
                "topics": sorted(by_topic),
                **details,
            }
            self.reanchor_fence_invalid = issue
            self.alignment_stage_mismatch_reason = (
                f"fresh re-anchor {kind} {issue}"
            )
            raise AssertionError(self.alignment_stage_mismatch_reason)

        for stamp_ns, by_topic in sorted(self.alignment_cycles.items()):
            if stamp_ns <= minimum_stamp_exclusive_ns:
                continue
            envelope_records = by_topic.get("command_envelope", ())
            if not envelope_records:
                continue
            for envelope_record in envelope_records:
                envelope_identity = envelope_record["identity"]
                if (
                    not isinstance(envelope_identity, tuple)
                    or len(envelope_identity) < 4
                    or int(envelope_identity[0]) != stamp_ns
                ):
                    fail(
                        "command_envelope_identity",
                        stamp_ns,
                        by_topic,
                        identity=envelope_identity,
                    )
                sequence = int(envelope_identity[2])
                producer_instance_id = int(envelope_identity[1])
                if sequence < minimum_sequence:
                    fail(
                        "sequence_regression",
                        stamp_ns,
                        by_topic,
                        sequence=sequence,
                        minimum_sequence=minimum_sequence,
                    )
                if producer_instance_id != expected_producer_instance_id:
                    fail(
                        "producer_mismatch",
                        stamp_ns,
                        by_topic,
                        sequence=sequence,
                        expected_producer_instance_id=(
                            expected_producer_instance_id
                        ),
                        observed_producer_instance_id=producer_instance_id,
                    )
                execution_records = by_topic.get(
                    "execution_envelope", ()
                )
                exact_executions = [
                    record
                    for record in execution_records
                    if self._execution_identity_matches(
                        record.get("identity"),
                        envelope_identity,
                    )
                ]
                partial_executions = [
                    record
                    for record in execution_records
                    if (
                        isinstance(record.get("identity"), tuple)
                        and record["identity"][1:3]
                        == envelope_identity[1:3]
                    )
                ]
                if partial_executions and not exact_executions:
                    fail(
                        "cohort_identity_mismatch",
                        stamp_ns,
                        by_topic,
                        sequence=sequence,
                    )
                if exact_executions and not any(
                    isinstance(record.get("semantic"), dict)
                    and record["semantic"].get("command_envelope")
                    == envelope_record.get("semantic")
                    for record in exact_executions
                ):
                    fail(
                        "semantic_mismatch",
                        stamp_ns,
                        by_topic,
                        sequence=sequence,
                    )
            self._completed_cycle_records(
                by_topic,
                minimum_sequence,
                expected_producer_instance_id,
            )

    @classmethod
    def _completed_cycle_records(
        cls,
        by_topic: dict[str, list[dict[str, object]]],
        minimum_sequence: int,
        expected_producer_instance_id: int | None,
    ) -> list[tuple[int, int, dict[str, dict[str, object]]]]:
        working = {
            topic: list(records)
            for topic, records in by_topic.items()
        }
        envelope_identities = [
            record.get("identity")
            for record in working.get("command_envelope", ())
        ]
        sequences = sorted(
            {
                int(identity[2])
                for identity in envelope_identities
                if (
                    isinstance(identity, tuple)
                    and len(identity) >= 4
                    and int(identity[2]) >= minimum_sequence
                    and (
                        expected_producer_instance_id is None
                        or int(identity[1])
                        == expected_producer_instance_id
                    )
                )
            }
        )
        completed = []
        for sequence in sequences:
            try:
                selected = cls._select_expected_cycle_records(
                    working, sequence
                )
            except AssertionError as error:
                if "execution envelope full identity mismatch" in str(error):
                    # Pre-arm discovery is observational: a malformed partial
                    # cohort cannot anchor, while strict fresh validation above
                    # reports the concrete mismatch.
                    return []
                raise
            if selected is None:
                continue
            selected_by_topic = {
                topic: [record] for topic, record in selected.items()
            }
            if not cls._completed_cycle_is_exact(selected_by_topic):
                raise AssertionError(
                    "fresh re-anchor semantic mismatch "
                    f"sequence={sequence}"
                )
            envelope_identity = selected["command_envelope"]["identity"]
            if not isinstance(envelope_identity, tuple):
                raise AssertionError(
                    "fresh re-anchor envelope identity invalid"
                )
            completed.append(
                (sequence, int(envelope_identity[1]), selected)
            )
            for topic, record in selected.items():
                working[topic].remove(record)
        if any(records for records in working.values()):
            # Identity-less mirrors cannot be assigned by arrival order.  Keep
            # the stamp pending until the full envelope cardinality arrives.
            return []
        return completed

    def _startup_stale_stop_evidence(self) -> list[dict[str, object]]:
        evidence = []
        for record in self.startup_ledger:
            if record["topic"] != "command_envelope":
                continue
            semantic = record["semantic"]
            if (
                not isinstance(semantic, dict)
                or semantic.get("reason") not in STALE_INPUT_REASONS
            ):
                continue
            command = semantic.get("command", {})
            longitudinal = (
                command.get("longitudinal", {})
                if isinstance(command, dict)
                else {}
            )
            safe_stop = (
                semantic.get("pp_command_fresh") is False
                and semantic.get("trajectory_tracking_usable") is False
                and semantic.get("lateral_stop_authority_kind") == 0
                and semantic.get("lateral_stop_authority_token") == 0
                and longitudinal.get("speed") == 0.0
                and isinstance(longitudinal.get("acceleration"), (int, float))
                and longitudinal["acceleration"] < 0.0
            )
            evidence.append(
                {
                    "identity": record["identity"],
                    "reason": semantic.get("reason"),
                    "speed_mps": longitudinal.get("speed"),
                    "acceleration_mps2": longitudinal.get("acceleration"),
                    "safe_stop": safe_stop,
                }
            )
        return evidence

    def arm_after_completed_cycle(
        self,
        minimum_sequence: int,
        *,
        minimum_stamp_exclusive_ns: int | None = None,
        expected_producer_instance_id: int | None = None,
    ) -> tuple[int, int, int] | None:
        if not self.cohort_alignment_enabled:
            self.enabled = True
            return None
        if self.startup_ledger_overflow:
            raise AssertionError("startup alignment ledger overflow")
        if (
            self.alignment_stage_overflow
            or self.alignment_stage_message_count
            > ALIGNMENT_STAGE_MAX_MESSAGES
        ):
            raise AssertionError("startup alignment stage overflow")
        stale_stop_evidence = self._startup_stale_stop_evidence()
        if not stale_stop_evidence or not all(
            bool(entry["safe_stop"]) for entry in stale_stop_evidence
        ):
            raise AssertionError(
                "startup stale-input fail-closed evidence invalid "
                f"count={len(stale_stop_evidence)}"
            )
        candidates = self._completed_cycle_candidates(
            minimum_sequence,
            minimum_stamp_exclusive_ns=minimum_stamp_exclusive_ns,
            expected_producer_instance_id=expected_producer_instance_id,
        )
        if not candidates:
            raise AssertionError(
                "startup completed output cohort unavailable "
                f"minimum_sequence={minimum_sequence}"
            )
        anchor_stamp_ns, anchor_sequence, anchor_producer_instance_id = max(
            candidates, key=lambda value: (value[1], value[0])
        )
        anchor_records = self.alignment_cycles[anchor_stamp_ns]
        anchor_envelopes = [
            record
            for record in anchor_records["command_envelope"]
            if (
                isinstance(record.get("identity"), tuple)
                and int(record["identity"][2]) == anchor_sequence
            )
        ]
        if len(anchor_envelopes) != 1:
            raise AssertionError("startup alignment anchor envelope invalid")
        anchor_identity = anchor_envelopes[0]["identity"]
        if (
            not isinstance(anchor_identity, tuple)
            or int(anchor_identity[1]) != anchor_producer_instance_id
        ):
            raise AssertionError("startup alignment anchor identity invalid")
        self.capture_after_stamp_ns = anchor_stamp_ns
        self.startup_anchor_stamp_ns = anchor_stamp_ns
        self.startup_anchor_sequence = anchor_sequence
        self.startup_anchor_producer_instance_id = (
            anchor_producer_instance_id
        )
        self.next_expected_sequence = anchor_sequence + 1
        stale_stamps = [
            stamp_ns
            for stamp_ns in self.alignment_cycles
            if stamp_ns <= anchor_stamp_ns
        ]
        for stamp_ns in stale_stamps:
            removed = self.alignment_cycles.pop(stamp_ns)
            self.alignment_stage_message_count -= sum(
                len(records) for records in removed.values()
            )
        self.enabled = True
        self._materialize_ready_cycles()
        return (
            anchor_stamp_ns,
            anchor_sequence,
            anchor_producer_instance_id,
        )

    def _completed_cycle_candidates(
        self,
        minimum_sequence: int,
        *,
        minimum_stamp_exclusive_ns: int | None = None,
        expected_producer_instance_id: int | None = None,
    ) -> list[tuple[int, int, int]]:
        candidates = []
        for stamp_ns, by_topic in self.alignment_cycles.items():
            if (
                minimum_stamp_exclusive_ns is not None
                and stamp_ns <= minimum_stamp_exclusive_ns
            ):
                continue
            for sequence, producer_instance_id, _ in (
                self._completed_cycle_records(
                    by_topic,
                    minimum_sequence,
                    expected_producer_instance_id,
                )
            ):
                candidates.append(
                    (stamp_ns, sequence, producer_instance_id)
                )
        return candidates

    def has_completed_cycle(
        self,
        minimum_sequence: int,
        *,
        minimum_stamp_exclusive_ns: int | None = None,
        expected_producer_instance_id: int | None = None,
    ) -> bool:
        return bool(
            self._completed_cycle_candidates(
                minimum_sequence,
                minimum_stamp_exclusive_ns=minimum_stamp_exclusive_ns,
                expected_producer_instance_id=(
                    expected_producer_instance_id
                ),
            )
        )

    def latest_completed_cycle_candidate(
        self,
        minimum_sequence: int,
    ) -> tuple[int, int, int]:
        candidates = self._completed_cycle_candidates(minimum_sequence)
        if not candidates:
            raise AssertionError(
                "completed output cohort boundary unavailable "
                f"minimum_sequence={minimum_sequence}"
            )
        return max(candidates, key=lambda value: (value[1], value[0]))

    def validate_aligned_capture(self) -> None:
        if not self.cohort_alignment_enabled:
            return
        if (
            self.startup_anchor_stamp_ns is None
            or self.startup_anchor_sequence is None
        ):
            raise AssertionError("startup alignment anchor unavailable")
        identities = {
            key: values
            for key, values in self.identities.items()
        }
        if set(identities) != CAPTURE_TOPICS:
            raise AssertionError(
                f"aligned capture topics invalid: {sorted(identities)}"
            )
        expected_sequences = list(
            range(
                self.startup_anchor_sequence + 1,
                self.startup_anchor_sequence
                + CAPTURE_WINDOW_COUNT
                + 1,
            )
        )
        if self.materialized_sequences != expected_sequences:
            raise AssertionError(
                "aligned capture materialized sequence invalid "
                f"anchor={self.startup_anchor_sequence} "
                f"materialized={self.materialized_sequences}"
            )
        previous_stamp_ns = self.startup_anchor_stamp_ns
        for ordinal, expected_sequence in enumerate(expected_sequences):
            cohort_identities = {
                key: values[ordinal]
                for key, values in identities.items()
            }
            cohort_stamps = {
                int(identity[0])
                for identity in cohort_identities.values()
            }
            if (
                len(cohort_stamps) != 1
                or next(iter(cohort_stamps)) < previous_stamp_ns
            ):
                raise AssertionError(
                    "aligned capture cohort stamp invalid "
                    f"ordinal={ordinal} previous={previous_stamp_ns} "
                    f"current={sorted(cohort_stamps)}"
                )
            previous_stamp_ns = next(iter(cohort_stamps))
            envelope_identity = cohort_identities["command_envelope"]
            execution_identity = cohort_identities["execution_envelope"]
            tracking_identity = cohort_identities["tracking"]
            if (
                int(envelope_identity[2]) != expected_sequence
                or not self._execution_identity_matches(
                    execution_identity,
                    envelope_identity,
                )
                or int(tracking_identity[3]) != int(envelope_identity[3])
            ):
                raise AssertionError(
                    "aligned capture cohort identity invalid "
                    f"ordinal={ordinal} expected={expected_sequence} "
                    f"envelope={envelope_identity} "
                    f"execution={execution_identity} "
                    f"tracking={tracking_identity}"
                )

    def startup_alignment_summary(self) -> dict[str, object]:
        return {
            "diagnostic_only": True,
            "prearm_storage_budget_ms": PREARM_STORAGE_BUDGET_MS,
            "prearm_storage_required_messages": (
                PREARM_STORAGE_REQUIRED_MESSAGES
            ),
            "ledger_overflow": self.startup_ledger_overflow,
            "ledger_capacity": STARTUP_LEDGER_MAX_MESSAGES,
            "message_count": len(self.startup_ledger),
            "ledger_max_message_count": len(self.startup_ledger),
            "anchor_stamp_ns": self.startup_anchor_stamp_ns,
            "anchor_sequence": self.startup_anchor_sequence,
            "anchor_producer_instance_id": (
                self.startup_anchor_producer_instance_id
            ),
            "alignment_stage_capacity": ALIGNMENT_STAGE_MAX_MESSAGES,
            "alignment_stage_overflow": self.alignment_stage_overflow,
            "alignment_stage_max_message_count": (
                self.alignment_stage_max_message_count
            ),
            "alignment_stage_pending_message_count": (
                self.alignment_stage_message_count
            ),
            "alignment_stage_mismatch_reason": (
                self.alignment_stage_mismatch_reason
            ),
            "materialized_sequences": self.materialized_sequences,
            "next_expected_sequence": self.next_expected_sequence,
            "stale_input_stop_evidence": (
                self._startup_stale_stop_evidence()
            ),
            "ledger": self.startup_ledger,
        }

    def complete(self) -> bool:
        return len(self.values) == 5 and all(
            len(values) >= CAPTURE_WINDOW_COUNT
            for values in self.values.values()
        )

    def alignment_pending_summary(self) -> dict[str, object]:
        pending = []
        for stamp_ns, by_topic in sorted(self.alignment_cycles.items())[:10]:
            pending.append(
                {
                    "stamp_ns": stamp_ns,
                    "topics": sorted(by_topic),
                    "command_sequences": [
                        int(record["identity"][2])
                        for record in by_topic.get(
                            "command_envelope", ()
                        )
                    ],
                }
            )
        return {
            "anchor_stamp_ns": self.startup_anchor_stamp_ns,
            "anchor_sequence": self.startup_anchor_sequence,
            "next_expected_sequence": self.next_expected_sequence,
            "materialized_count": len(self.materialized_sequences),
            "stage_message_count": self.alignment_stage_message_count,
            "stage_overflow": self.alignment_stage_overflow,
            "mismatch_reason": self.alignment_stage_mismatch_reason,
            "pending": pending,
        }

    def fresh_reanchor_observation_summary(
        self,
        minimum_sequence: int,
        *,
        minimum_stamp_exclusive_ns: int,
        expected_producer_instance_id: int,
    ) -> dict[str, object]:
        topic_record_counts = {
            topic: 0 for topic in sorted(CAPTURE_TOPICS)
        }
        topic_max_stamp_ns: dict[str, int | None] = {
            topic: None for topic in sorted(CAPTURE_TOPICS)
        }
        sequences: list[int] = []
        producer_instance_ids: set[int] = set()
        fresh_stamp_count = 0
        exact_complete_count = 0
        eligible_exact_complete_count = 0
        incomplete_stamp_count = 0
        duplicate_topic_count = 0
        newest = []
        for stamp_ns, by_topic in sorted(self.alignment_cycles.items()):
            if stamp_ns <= minimum_stamp_exclusive_ns:
                continue
            fresh_stamp_count += 1
            duplicate_topics = sorted(
                topic
                for topic, records in by_topic.items()
                if len(records) != 1
            )
            duplicate_topic_count += len(duplicate_topics)
            for topic, records in by_topic.items():
                if topic not in topic_record_counts:
                    continue
                topic_record_counts[topic] += len(records)
                topic_max_stamp_ns[topic] = max(
                    stamp_ns,
                    topic_max_stamp_ns[topic] or stamp_ns,
                )
            envelope_records = by_topic.get("command_envelope", ())
            stamp_sequences = []
            for record in envelope_records:
                identity = record["identity"]
                if not isinstance(identity, tuple) or len(identity) < 4:
                    continue
                sequence = int(identity[2])
                producer_instance_id = int(identity[1])
                sequences.append(sequence)
                stamp_sequences.append(sequence)
                producer_instance_ids.add(producer_instance_id)
            exact_complete = self._completed_cycle_is_exact(by_topic)
            if exact_complete:
                exact_complete_count += 1
                identity = envelope_records[0]["identity"]
                if (
                    int(identity[2]) >= minimum_sequence
                    and int(identity[1])
                    == expected_producer_instance_id
                ):
                    eligible_exact_complete_count += 1
            else:
                incomplete_stamp_count += 1
            newest.append(
                {
                    "stamp_ns": stamp_ns,
                    "topics": sorted(by_topic),
                    "record_counts": {
                        topic: len(records)
                        for topic, records in sorted(by_topic.items())
                    },
                    "command_sequences": stamp_sequences,
                    "duplicate_topics": duplicate_topics,
                    "exact_complete": exact_complete,
                }
            )
        return {
            "minimum_sequence": minimum_sequence,
            "minimum_stamp_exclusive_ns": minimum_stamp_exclusive_ns,
            "expected_producer_instance_id": (
                expected_producer_instance_id
            ),
            "fresh_stamp_count": fresh_stamp_count,
            "exact_complete_count": exact_complete_count,
            "eligible_exact_complete_count": (
                eligible_exact_complete_count
            ),
            "incomplete_stamp_count": incomplete_stamp_count,
            "duplicate_topic_count": duplicate_topic_count,
            "topic_record_counts": topic_record_counts,
            "topic_max_stamp_ns": topic_max_stamp_ns,
            "minimum_observed_sequence": min(sequences) if sequences else None,
            "maximum_observed_sequence": max(sequences) if sequences else None,
            "observed_producer_instance_ids": sorted(producer_instance_ids),
            "newest": newest[-10:],
        }

    def signature(self) -> tuple:
        return tuple(
            (key, tuple(values[:CAPTURE_WINDOW_COUNT]))
            for key, values in sorted(self.values.items())
        )

    def identity_signature(self) -> tuple:
        return tuple(
            (key, tuple(values[:CAPTURE_WINDOW_COUNT]))
            for key, values in sorted(self.identities.items())
        )

    def canonical_semantic_signature(self) -> tuple:
        return tuple(
            (key, tuple(values[:CAPTURE_WINDOW_COUNT]))
            for key, values in sorted(self.canonical_semantics.items())
        )

    @staticmethod
    def _sample_identity(message: object) -> tuple:
        header = getattr(message, "header", None)
        stamp = (
            getattr(header, "stamp", None)
            if header is not None
            else getattr(message, "stamp", None)
        )
        stamp_ns = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if stamp is not None
            else 0
        )
        plan_key = getattr(message, "plan_sample_key", None)
        witness = getattr(message, "witness", None)
        return (
            stamp_ns,
            int(getattr(message, "producer_instance_id", 0)),
            int(getattr(message, "command_sequence", 0)),
            int(getattr(message, "plan_generation", 0)),
            int(getattr(plan_key, "race_arm_epoch", 0))
            if plan_key is not None
            else 0,
            int(getattr(plan_key, "plan_generation", 0))
            if plan_key is not None
            else 0,
            int(getattr(witness, "source_generation", 0))
            if witness is not None
            else 0,
            int(getattr(witness, "base_source_generation", 0))
            if witness is not None
            else 0,
            str(getattr(header, "frame_id", ""))
            if header is not None
            else "",
        )


def fixed_clock() -> Clock:
    message = Clock()
    message.clock.sec = FIXED_SEC
    return message


def trajectory(
    curved: bool = False, state_lattice_fixture: bool = False
) -> Trajectory:
    message = Trajectory()
    message.header.frame_id = "map"
    message.header.stamp.sec = FIXED_SEC
    message.points = []
    point_count = 64 if state_lattice_fixture else 40
    for index in range(point_count):
        point = message.points.add() if hasattr(message.points, "add") else None
        if point is None:
            from autoware_auto_planning_msgs.msg import TrajectoryPoint

            point = TrajectoryPoint()
            message.points.append(point)
        if state_lattice_fixture:
            distance_m = 0.20 * index
            point.pose.position.x = (
                D2_EGO_X_M + math.cos(D2_EGO_YAW_RAD) * distance_m
            )
            point.pose.position.y = (
                D2_EGO_Y_M + math.sin(D2_EGO_YAW_RAD) * distance_m
            )
            point.pose.orientation.z = math.sin(D2_EGO_YAW_RAD * 0.5)
            point.pose.orientation.w = math.cos(D2_EGO_YAW_RAD * 0.5)
        elif curved:
            theta = 0.1 * index
            point.pose.position.x = math.sin(theta)
            point.pose.position.y = 1.0 - math.cos(theta)
            point.pose.orientation.z = math.sin(theta * 0.5)
            point.pose.orientation.w = math.cos(theta * 0.5)
        else:
            point.pose.position.x = 0.25 * index
            point.pose.orientation.w = 1.0
        point.longitudinal_velocity_mps = 2.0
        point.time_from_start = Duration(
            sec=0, nanosec=index * 10_000_000
        )
    return message


def odometry(state_lattice_fixture: bool = False) -> Odometry:
    message = Odometry()
    message.header.frame_id = "map"
    message.header.stamp.sec = FIXED_SEC
    if state_lattice_fixture:
        message.pose.pose.position.x = D2_EGO_X_M
        message.pose.pose.position.y = D2_EGO_Y_M
        message.pose.pose.orientation.z = math.sin(D2_EGO_YAW_RAD * 0.5)
        message.pose.pose.orientation.w = math.cos(D2_EGO_YAW_RAD * 0.5)
        message.twist.twist.linear.x = 0.0
    else:
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = 1.0
    return message


def v2x_fixture(v2_uptake_only: bool = False) -> V2XVehiclePositionArray:
    message = V2XVehiclePositionArray()
    message.header.frame_id = "map"
    message.header.stamp.sec = FIXED_SEC
    opponents = D2_OPPONENTS
    if v2_uptake_only:
        # The archived D2 snapshot is intentionally a blocked-output
        # regression fixture.  The V2 transport smoke instead needs one
        # deterministic, map-valid and permission-valid safe proposal without
        # weakening planner safety gates.
        target_distance_m = 9.0
        opponents = ((
            "d3",
            D2_EGO_X_M + math.cos(D2_EGO_YAW_RAD) * target_distance_m,
            D2_EGO_Y_M + math.sin(D2_EGO_YAW_RAD) * target_distance_m,
        ),)
    for vehicle_id, x_m, y_m in opponents:
        vehicle = V2XVehiclePosition()
        vehicle.header.frame_id = "map"
        vehicle.header.stamp.sec = FIXED_SEC
        vehicle.vehicle_id = vehicle_id
        vehicle.position.x = x_m
        vehicle.position.y = y_m
        vehicle.covariance.x = 0.0
        vehicle.covariance.y = 0.0
        message.vehicles.append(vehicle)
    return message


def overtake_plan() -> OvertakePlan:
    message = OvertakePlan()
    message.header.frame_id = "map"
    message.header.stamp.sec = FIXED_SEC
    message.aw2_identity_schema_version = 1
    message.race_arm_epoch = 1
    message.planner_instance_id = 1
    message.plan_generation = 1
    message.free_run_canonical_algorithm_version = (
        OvertakePlan.FREE_RUN_CANONICAL_ALGORITHM_V1
    )
    message.free_run_canonical_payload_sha256 = list(
        free_run_plan_digest(message)
    )
    return message


def free_run_plan_digest(message: OvertakePlan) -> bytes:
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
    append_string(message.header.frame_id)
    append_u8(message.phase)
    append_u32(message.attempt_id)
    append_string(message.target_vehicle_id)
    append_i8(message.pass_direction)
    append_bool(message.trajectory_authorized)
    append_bool(message.lateral_maneuver_required)
    append_string(message.decision_reason)
    append_u32(message.authorization_failure_mask)
    append_u32(len(message.authorization_failure_reasons))
    for reason in message.authorization_failure_reasons:
        append_string(reason)
    append_string(message.candidate_reject_reason)
    append_bool(message.safety_inputs_complete)
    append_bool(message.tracking_usable)
    append_bool(message.trajectory_publishable)
    append_string(message.constraint_reason)
    append_u8(message.lateral_stop_authority_kind)
    append_i8(message.lateral_stop_transaction_pass_direction)
    append_u64(message.lateral_stop_authority_token)
    append_string(message.trajectory.header.frame_id)
    append_u32(0)
    append_u8(message.aw2_identity_schema_version)
    append_u64(message.planner_instance_id)
    append_u64(message.race_arm_epoch)
    append_u64(message.connector_transaction_id)
    append_u32(message.candidate_revision)
    wire.extend(bytes(message.candidate_content_sha256))
    return hashlib.sha256(bytes(wire)).digest()


def child_pids(parent: int) -> list[int]:
    path = Path(f"/proc/{parent}/task/{parent}/children")
    try:
        return [int(value) for value in path.read_text().split()]
    except (FileNotFoundError, ProcessLookupError):
        return []


def worker_pids(parent: int) -> list[int]:
    pending = child_pids(parent)
    descendants = []
    while pending:
        pid = pending.pop()
        pending.extend(child_pids(pid))
        try:
            command = Path(f"/proc/{pid}/cmdline").read_bytes()
        except (FileNotFoundError, ProcessLookupError):
            continue
        if b"c002ay0_shadow_worker" in command:
            descendants.append(pid)
    return descendants


def all_descendant_pids(parent: int) -> list[int]:
    pending = child_pids(parent)
    descendants = []
    while pending:
        pid = pending.pop()
        if pid in descendants:
            continue
        descendants.append(pid)
        pending.extend(child_pids(pid))
    return sorted(descendants)


def parse_cpu_list(value: str) -> set[int]:
    cpus: set[int] = set()
    for item in value.split(","):
        bounds = item.strip().split("-", 1)
        try:
            first = int(bounds[0])
            last = int(bounds[-1])
        except (IndexError, ValueError) as error:
            raise AssertionError(f"invalid CPU list: {value}") from error
        if first < 0 or last < first:
            raise AssertionError(f"invalid CPU range: {item}")
        cpus.update(range(first, last + 1))
    if not cpus:
        raise AssertionError("CPU list must not be empty")
    return cpus


def cgroup_cpu_stat(pid: int) -> dict[str, object]:
    try:
        cgroup_lines = Path(f"/proc/{pid}/cgroup").read_text().splitlines()
    except OSError as error:
        return {"available": False, "error": str(error)}
    candidates: list[Path] = []
    for line in cgroup_lines:
        fields = line.split(":", 2)
        if len(fields) != 3:
            continue
        controllers, relative_path = fields[1], fields[2].lstrip("/")
        if not controllers:
            candidates.append(
                Path("/sys/fs/cgroup") / relative_path / "cpu.stat"
            )
        elif "cpu" in controllers.split(","):
            candidates.append(
                Path("/sys/fs/cgroup/cpu") / relative_path / "cpu.stat"
            )
    for path in candidates:
        try:
            content = path.read_text()
        except OSError:
            continue
        values: dict[str, int] = {}
        for line in content.splitlines():
            fields = line.split()
            if len(fields) == 2:
                try:
                    values[fields[0]] = int(fields[1])
                except ValueError:
                    continue
        return {
            "available": True,
            "path": str(path),
            "cgroup": cgroup_lines,
            "values": values,
        }
    return {"available": False, "cgroup": cgroup_lines}


def fixture_process_identity_snapshot(pid: int) -> dict[str, object]:
    process_root = Path(f"/proc/{pid}")
    try:
        content = (process_root / "stat").read_text()
        right_parenthesis = content.rfind(")")
        if right_parenthesis < 0:
            raise ValueError("stat comm terminator missing")
        comm = content[content.find("(") + 1 : right_parenthesis]
        fields = content[right_parenthesis + 2 :].split()
        if len(fields) <= 19:
            raise ValueError("stat fields incomplete")
        state = fields[0]
        ppid = int(fields[1])
        session_id = int(fields[3])
        starttime_ticks = int(fields[19])
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError) as error:
        return {
            "pid": pid,
            "readable": False,
            "classification": "unreadable",
            "error": f"{type(error).__name__}: {error}",
        }
    try:
        cmdline = (
            (process_root / "cmdline")
            .read_bytes()
            .replace(b"\0", b" ")
            .decode(errors="replace")
            .strip()
        )
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        cmdline = ""
    try:
        cgroup = (process_root / "cgroup").read_text().splitlines()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        cgroup = None
    classification = (
        "zombie"
        if state == "Z"
        else "live"
        if state in {"R", "S", "D", "T", "t", "I", "W", "P"}
        else "unknown"
    )
    return {
        "pid": pid,
        "readable": True,
        "classification": classification,
        "state": state,
        "ppid": ppid,
        "sid": session_id,
        "session_id": session_id,
        "starttime": starttime_ticks,
        "starttime_ticks": starttime_ticks,
        "cmdline": cmdline,
        "comm": comm,
        "cgroup": cgroup,
    }


def process_scheduler_snapshot(pid: int, role: str) -> dict[str, object]:
    task_root = Path(f"/proc/{pid}/task")
    tids = []
    for task_path in sorted(task_root.iterdir(), key=lambda path: int(path.name)):
        tid = int(task_path.name)
        tids.append(
            {
                "tid": tid,
                "affinity": sorted(os.sched_getaffinity(tid)),
                "scheduler_policy": os.sched_getscheduler(tid),
                "scheduler_priority": os.sched_getparam(tid).sched_priority,
                "nice": os.getpriority(os.PRIO_PROCESS, tid),
            }
        )
    if not tids:
        raise AssertionError(f"{role}: no task IDs for PID {pid}")
    return {
        "role": role,
        "pid": pid,
        "command": (
            Path(f"/proc/{pid}/cmdline")
            .read_bytes()
            .replace(b"\0", b" ")
            .decode(errors="replace")
            .strip()
        ),
        "cgroup": cgroup_cpu_stat(pid),
        "tids": tids,
    }


def verify_process_affinity(
    snapshot: dict[str, object],
    expected_cpus: set[int],
) -> None:
    mismatches = [
        {
            "tid": task["tid"],
            "affinity": task["affinity"],
        }
        for task in snapshot["tids"]
        if set(task["affinity"]) != expected_cpus
    ]
    if mismatches:
        raise AssertionError(
            f"{snapshot['role']}: effective affinity mismatch "
            f"expected={sorted(expected_cpus)} actual={mismatches}"
        )


def collect_scheduling_evidence(
    *,
    pp_pid: int,
    state_lattice_pid: int | None,
    proposal_relay_pid: int | None,
    expected_pp_cpus: set[int],
    expected_fixture_cpus: set[int],
) -> dict[str, object]:
    processes = [
        process_scheduler_snapshot(os.getpid(), "fixture"),
        process_scheduler_snapshot(pp_pid, "pure_pursuit"),
    ]
    for descendant in all_descendant_pids(pp_pid):
        processes.append(
            process_scheduler_snapshot(descendant, "pure_pursuit_descendant")
        )
    if state_lattice_pid is not None:
        processes.append(
            process_scheduler_snapshot(state_lattice_pid, "state_lattice")
        )
        for descendant in all_descendant_pids(state_lattice_pid):
            processes.append(
                process_scheduler_snapshot(
                    descendant,
                    "state_lattice_descendant",
                )
            )
    if proposal_relay_pid is not None:
        processes.append(
            process_scheduler_snapshot(proposal_relay_pid, "proposal_relay")
        )
    for snapshot in processes:
        expected = (
            expected_pp_cpus
            if snapshot["role"].startswith("pure_pursuit")
            else expected_fixture_cpus
        )
        verify_process_affinity(snapshot, expected)
        cgroup = snapshot["cgroup"]
        if not bool(cgroup["available"]):
            raise AssertionError(
                f"{snapshot['role']}: cgroup cpu.stat unavailable {cgroup}"
            )
    return {
        "requested": {
            "fixture_cpus": sorted(expected_fixture_cpus),
            "pure_pursuit_cpus": sorted(expected_pp_cpus),
        },
        "processes": processes,
    }


def stop_process(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    os.kill(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        os.kill(process.pid, signal.SIGKILL)
        process.wait(timeout=1)


def refresh_isolated_publisher_lifecycle(
    lifecycle: dict[str, object],
    final_status: dict[str, object],
) -> None:
    lifecycle.update(
        {
            "alive": final_status["alive"],
            "exitcode": final_status["exitcode"],
            "target_process_lifecycle": copy.deepcopy(
                final_status.get("target_process_lifecycle")
            ),
            "target_cleanup_failure": str(
                final_status.get("target_cleanup_failure", "")
            ),
            "associated_child_lifecycle": copy.deepcopy(
                final_status.get("associated_child_lifecycle", [])
            ),
            "associated_child_cleanup_failure": str(
                final_status.get(
                    "associated_child_cleanup_failure",
                    "",
                )
            ),
            "associated_child_sample_failures": list(
                final_status.get(
                    "associated_child_sample_failures",
                    [],
                )
            ),
            "unresolved": bool(final_status["alive"]),
        }
    )


def cleanup_pre_run_case_failure(
    primary_error: BaseException,
    *,
    node: object,
    capture: object,
    isolated_input_publisher: object | None,
    proposal_relay: object | None,
    state_lattice_process: object | None,
    process: object | None,
    state_lattice_log: object | None,
    pp_log: object | None,
    ring: object | None,
    stop_process_action: object = stop_process,
) -> list[str]:
    cleanup_errors = []
    if isolated_input_publisher is not None:
        try:
            isolated_input_publisher.close()
        except BaseException as error:
            cleanup_errors.append(
                "isolated_input_publisher="
                f"{type(error).__name__}: {error}"
            )
    for label, handle in (
        ("proposal_relay", proposal_relay),
        ("state_lattice_process", state_lattice_process),
        ("process", process),
    ):
        if handle is None:
            continue
        try:
            stop_process_action(handle)
        except BaseException as error:
            cleanup_errors.append(
                f"{label}={type(error).__name__}: {error}"
            )
    for label, stream in (
        ("state_lattice_log", state_lattice_log),
        ("pp_log", pp_log),
    ):
        if stream is None:
            continue
        try:
            stream.close()
        except BaseException as error:
            cleanup_errors.append(
                f"{label}={type(error).__name__}: {error}"
            )
    try:
        subscriptions = list(getattr(capture, "subscriptions", []))
    except BaseException as error:
        cleanup_errors.append(
            "capture_subscriptions="
            f"{type(error).__name__}: {error}"
        )
        subscriptions = []
    for index, subscription in enumerate(subscriptions):
        try:
            node.destroy_subscription(subscription)
        except BaseException as error:
            cleanup_errors.append(
                "capture_subscription"
                f"[{index}]={type(error).__name__}: {error}"
            )
    if ring is not None:
        try:
            ring.close()
        except BaseException as error:
            cleanup_errors.append(
                f"ring={type(error).__name__}: {error}"
            )
    try:
        setattr(
            primary_error,
            "_pre_run_case_cleanup_failures",
            list(cleanup_errors),
        )
        if cleanup_errors:
            original_message = str(primary_error)
            primary_error.args = (
                f"{original_message}; pre_run_cleanup_failures="
                f"{'; '.join(cleanup_errors)}",
            )
    except BaseException:
        # Cleanup diagnostics must never replace the exception that triggered
        # this pre-main-try recovery path.
        pass
    return cleanup_errors


def wait_process_gone(pid: int, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if not Path(f"/proc/{pid}").exists():
            return True
        time.sleep(0.01)
    return not Path(f"/proc/{pid}").exists()


class V2UptakeCapture:
    """Fixture-only observer for one non-authoritative V2 uptake edge."""

    def __init__(self, node: rclpy.node.Node) -> None:
        self.proposals: dict[tuple[object, ...], dict[str, object]] = {}
        self.statuses: list[dict[str, object]] = []
        self.last_proposal_receive_monotonic_ns = 0
        qos = QoSProfile(
            depth=17,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscriptions = [
            node.create_subscription(
                AuthorizedCartesianTrajectoryV2,
                "/planning/overtake/state_lattice/v2_proposal",
                self._receive_proposal,
                qos,
            ),
            node.create_subscription(
                StateLatticeV2BindingStatus,
                "/debug/overtake/state_lattice/v2_binding_status",
                self._receive_status,
                qos,
            ),
        ]

    @staticmethod
    def _identity_key(identity: object) -> tuple[object, ...]:
        return (
            str(identity.producer_instance_id),
            str(identity.session_id),
            int(identity.proposal_sequence),
            int(identity.plan_generation),
            int(identity.source_generation),
            V2UptakeCapture._stamp_ns(identity.source_stamp),
            str(identity.frame_id),
            tuple(int(value) for value in identity.canonical_sha256),
        )

    @staticmethod
    def _stamp_ns(stamp: object) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _receive_proposal(self, message: AuthorizedCartesianTrajectoryV2) -> None:
        self.last_proposal_receive_monotonic_ns = time.monotonic_ns()
        key = self._identity_key(message.identity)
        if key in self.proposals:
            raise AssertionError(f"V2 duplicate proposal identity={key}")
        self.proposals[key] = {
            "schema_version": int(message.schema_version),
            "frame_id": str(message.header.frame_id),
            "proposal_authority_eligible": bool(message.proposal.authority_eligible),
            "proposal_safety_evaluation_result": int(
                message.proposal.safety_evaluation_result
            ),
            "proposal_safety_valid_until_ns": self._stamp_ns(
                message.proposal.safety_valid_until
            ),
            "identity": {
                "producer_instance_id": key[0],
                "session_id": key[1],
                "proposal_sequence": key[2],
                "plan_generation": key[3],
                "source_generation": int(message.identity.source_generation),
                "source_stamp_ns": self._stamp_ns(message.identity.source_stamp),
                "frame_id": str(message.identity.frame_id),
                "canonical_sha256": [
                    int(value) for value in message.identity.canonical_sha256
                ],
            },
        }

    def _receive_status(self, message: StateLatticeV2BindingStatus) -> None:
        self.statuses.append(
            {
                "identity": self._identity_key(message.identity),
                "first_uptake_pass": bool(message.first_uptake_pass),
                "reject_reason": int(message.reject_reason),
                "run_invalid": bool(message.run_invalid),
                "overflow_count": int(message.overflow_count),
                "receive_monotonic_ns": int(message.receive_monotonic_ns),
                "accepted_monotonic_ns": int(message.accepted_monotonic_ns),
                "timer_entry_ros_ns": self._stamp_ns(message.header.stamp),
                "timer_entry_monotonic_ns": int(message.timer_entry_monotonic_ns),
                "hold_cycle_index": int(message.hold_cycle_index),
                "cycle_event_index": int(message.cycle_event_index),
                "cycle_event_count": int(message.cycle_event_count),
                "pp_cycle_sequence": int(message.pp_cycle_sequence),
                "availability_summary": bool(message.availability_summary),
                "availability_present": bool(message.availability_present),
                "availability_identity": self._identity_key(
                    message.availability_identity
                ),
                "availability_safety_valid_until_ns": self._stamp_ns(
                    message.availability_safety_valid_until
                ),
                "availability_transition": int(message.availability_transition),
            }
        )

    def successful_uptake(self) -> dict[str, object] | None:
        for status in self.statuses:
            key = status["identity"]
            if (
                status["first_uptake_pass"]
                and status["reject_reason"]
                == StateLatticeV2BindingStatus.REJECT_NONE
                and not status["run_invalid"]
                and status["overflow_count"] == 0
                and status["hold_cycle_index"] == 1
                and status["pp_cycle_sequence"] > 0
                and status["cycle_event_count"] > 0
                and 0 <= status["cycle_event_index"] < status["cycle_event_count"]
                and key in self.proposals
            ):
                return {"key": key, "status": status}
        return None

    def _accepted_first_uptake_keys(self) -> set[tuple[object, ...]]:
        return {
            item["identity"]
            for item in self.statuses
            if (
                item["first_uptake_pass"]
                and item["reject_reason"]
                == StateLatticeV2BindingStatus.REJECT_NONE
                and not item["run_invalid"]
                and item["overflow_count"] == 0
                and item["hold_cycle_index"] == 1
                and item["pp_cycle_sequence"] > 0
                and item["cycle_event_count"] > 0
                and 0
                <= item["cycle_event_index"]
                < item["cycle_event_count"]
                and item["identity"] in self.proposals
                and item["receive_monotonic_ns"] > 0
                and item["accepted_monotonic_ns"]
                >= item["receive_monotonic_ns"]
                and item["timer_entry_ros_ns"]
                < self.proposals[item["identity"]][
                    "proposal_safety_valid_until_ns"
                ]
            )
        }

    def ready(self, expected_count: int) -> bool:
        if len(self.proposals) > expected_count:
            raise AssertionError(
                "V2 unexpected proposal count "
                f"expected={expected_count} observed={len(self.proposals)}"
            )
        return (
            len(self.proposals) == expected_count
            and set(self.proposals) == self._accepted_first_uptake_keys()
        )

    def cohort_quiescent(self, quiet_sec: float) -> bool:
        if len(self.proposals) < V2_UPTAKE_MIN_PROPOSAL_COUNT:
            return False
        return (
            time.monotonic_ns() - self.last_proposal_receive_monotonic_ns
            >= int(quiet_sec * 1_000_000_000)
        )

    def validate(self, expected_count: int) -> dict[str, object]:
        if len(self.proposals) != expected_count:
            raise AssertionError(
                "V2 proposal count mismatch "
                f"expected={expected_count} observed={len(self.proposals)}"
            )
        for proposal_key, proposal_item in self.proposals.items():
            if proposal_key[0] != V2_UPTAKE_PLANNER_PRODUCER_INSTANCE_ID:
                raise AssertionError(
                    f"V2 planner producer mismatch key={proposal_key}"
                )
            if proposal_key[1] != V2_UPTAKE_SESSION_ID:
                raise AssertionError(
                    f"V2 session/race epoch mismatch key={proposal_key}"
                )
            if (
                proposal_item["schema_version"]
                != AuthorizedCartesianTrajectoryV2.SCHEMA_V2_NON_AUTHORITATIVE
            ):
                raise AssertionError(
                    f"V2 proposal schema invalid key={proposal_key}"
                )
            if proposal_item["proposal_authority_eligible"]:
                raise AssertionError(
                    f"V2 proposal became authoritative key={proposal_key}"
                )
            if (
                proposal_item["proposal_safety_evaluation_result"]
                != AuthorizedCartesianTrajectory.SAFETY_PASSED
            ):
                raise AssertionError(
                    f"V2 proposal is not safety-passed key={proposal_key}"
                )
        uptake = self.successful_uptake()
        if uptake is None:
            raise AssertionError(
                "V2 first uptake missing "
                f"proposals={len(self.proposals)} statuses={self.statuses[-8:]}"
            )
        key = uptake["key"]
        proposal = self.proposals[key]
        status = uptake["status"]
        if key[0] != V2_UPTAKE_PLANNER_PRODUCER_INSTANCE_ID:
            raise AssertionError(f"V2 planner producer mismatch key={key}")
        if key[1] != V2_UPTAKE_SESSION_ID:
            raise AssertionError(f"V2 session/race epoch mismatch key={key}")
        if proposal["schema_version"] != AuthorizedCartesianTrajectoryV2.SCHEMA_V2_NON_AUTHORITATIVE:
            raise AssertionError(f"V2 proposal schema invalid key={key}")
        if proposal["proposal_authority_eligible"]:
            raise AssertionError(f"V2 proposal became authoritative key={key}")
        if proposal["proposal_safety_evaluation_result"] != AuthorizedCartesianTrajectory.SAFETY_PASSED:
            raise AssertionError(f"V2 proposal is not safety-passed key={key}")
        if proposal["proposal_safety_valid_until_ns"] <= status["timer_entry_ros_ns"]:
            raise AssertionError(f"V2 uptake missed safety deadline key={key}")
        accepted_keys = self._accepted_first_uptake_keys()
        if set(self.proposals) != accepted_keys:
            raise AssertionError(
                "V2 proposal first-uptake completeness failed "
                f"proposals={set(self.proposals)} accepted={accepted_keys}"
            )
        summaries = [
            item for item in self.statuses if item["availability_summary"]
        ]
        anchor_sequence = int(status["pp_cycle_sequence"])
        summaries_by_sequence: dict[int, list[dict[str, object]]] = {}
        for item in summaries:
            summaries_by_sequence.setdefault(
                int(item["pp_cycle_sequence"]), []
            ).append(item)
        availability_window: list[dict[str, object]] | None = None
        candidate_window: list[dict[str, object]] = []
        current_key = key
        sequence = anchor_sequence
        first_cycle = True
        while True:
            matches = summaries_by_sequence.get(sequence, [])
            if len(matches) != 1:
                candidate_window = []
                break
            item = matches[0]
            current_deadline_ns = self.proposals[current_key][
                "proposal_safety_valid_until_ns"
            ]
            if item["timer_entry_ros_ns"] >= current_deadline_ns:
                if not item["availability_present"]:
                    availability_window = candidate_window
                    break
            item_key = item["availability_identity"]
            common_valid = (
                item["reject_reason"]
                == StateLatticeV2BindingStatus.REJECT_NONE
                and not item["run_invalid"]
                and item["overflow_count"] == 0
                and item["availability_present"]
                and item["pp_cycle_sequence"] == sequence
                and item["timer_entry_monotonic_ns"] > 0
                and item_key in accepted_keys
                and item_key in self.proposals
                and item["availability_safety_valid_until_ns"]
                == self.proposals[item_key]["proposal_safety_valid_until_ns"]
                and item["timer_entry_ros_ns"]
                < self.proposals[item_key]["proposal_safety_valid_until_ns"]
            )
            if item_key == current_key:
                valid_transition = item["availability_transition"] in (
                    (
                        StateLatticeV2BindingStatus.AVAILABILITY_FIRST,
                        StateLatticeV2BindingStatus.AVAILABILITY_REPLACED,
                    )
                    if first_cycle
                    else (StateLatticeV2BindingStatus.AVAILABILITY_HELD,)
                )
            else:
                valid_transition = (
                    item["availability_transition"]
                    == StateLatticeV2BindingStatus.AVAILABILITY_REPLACED
                    and item_key[0] == current_key[0]
                    and item_key[1] == current_key[1]
                    and item_key[2] > current_key[2]
                    and item_key[3] > current_key[3]
                )
                if valid_transition:
                    current_key = item_key
            if not common_valid or not valid_transition:
                candidate_window = []
                break
            candidate_window.append(item)
            first_cycle = False
            sequence += 1
        if availability_window is None:
            raise AssertionError(
                "V2 deadline-bounded continuous availability chain missing "
                f"statuses={self.statuses[-32:]}"
            )
        return {
            "classification": "PP_UPTAKE_AVAILABILITY_VERIFIED_NON_AUTHORITATIVE",
            "qualifier": (
                "deadline_bounded_continuous_pp_cycle_availability_with_exact_replacement; "
                "pp_application_mux_m4_not_verified"
            ),
            "planner_live_control_output": False,
            "planner_instant_control": False,
            "pp_accepted_payload": "structurally_discarded",
            "planner_producer_instance_id": V2_UPTAKE_PLANNER_PRODUCER_INSTANCE_ID,
            "pp_attestation_producer_instance_id": V2_UPTAKE_PP_ATTESTATION_PRODUCER_INSTANCE_ID,
            "session_id_and_race_epoch": V2_UPTAKE_SESSION_ID,
            "proposal": proposal,
            "first_uptake": status,
            "availability_cycles": availability_window,
            "observed_proposal_count": len(self.proposals),
            "observed_status_count": len(self.statuses),
        }


def run_case(
    node: rclpy.node.Node,
    root: Path,
    scenario: str,
    session_value: int,
    curved: bool = False,
    free_run_live: bool = False,
    measure_long: bool = False,
    binding_enabled: bool = False,
    proposal_relay_path: Path | None = None,
    warmup_sequence: int = 20,
    real_state_lattice: bool = False,
    pp_executable_override: Path | None = None,
    callback_span_path: Path | None = None,
    callback_span_enabled: bool | None = None,
    runtime_socket_path: Path | None = None,
    timing_diagnostic: bool = False,
    timing_record_count: int = TIMING_DIAGNOSTIC_RECORD_COUNT,
    v2_uptake_only: bool = False,
) -> tuple[
    tuple,
    list[tuple[int, int, int]],
    int,
    int,
    dict[str, float],
    int,
    bool,
    int,
    int,
    int,
    int,
    int,
    bool,
    dict[str, object],
    tuple,
]:
    scheduling_required = (
        os.environ.get("C002AY0_SCHEDULING_EVIDENCE_REQUIRED") == "1"
    )
    scheduling_evidence_root_text = os.environ.get(
        "C002AY0_SCHEDULING_EVIDENCE_ROOT", ""
    )
    expected_fixture_cpu_text = os.environ.get(
        "C002AY0_EXPECT_FIXTURE_CPU_LIST", ""
    )
    pp_cpu_text = os.environ.get("C002AY0_PP_CPU_LIST", "")
    scheduling_profile = os.environ.get("C002AY0_SCHEDULING_PROFILE", "")
    idle_break_diagnostic = (
        scheduling_required
        and os.environ.get("C002AY0_IDLE_BREAK_DIAGNOSTIC") == "1"
    )
    isolated_input_publisher_diagnostic = (
        scheduling_required
        and os.environ.get(
            "C002AY0_ISOLATED_INPUT_PUBLISHER_DIAGNOSTIC"
        )
        == "1"
    )
    if isolated_input_publisher_diagnostic:
        require_explicit_empty_cyclonedds_uri()
    if scheduling_required and (
        not scheduling_evidence_root_text
        or not expected_fixture_cpu_text
        or not pp_cpu_text
        or not scheduling_profile
    ):
        raise AssertionError("incomplete scheduling evidence configuration")
    expected_fixture_cpus = (
        parse_cpu_list(expected_fixture_cpu_text)
        if scheduling_required
        else set()
    )
    expected_pp_cpus = (
        parse_cpu_list(pp_cpu_text) if scheduling_required else set()
    )
    if scheduling_required and set(os.sched_getaffinity(0)) != expected_fixture_cpus:
        raise AssertionError(
            "fixture effective affinity mismatch "
            f"expected={sorted(expected_fixture_cpus)} "
            f"actual={sorted(os.sched_getaffinity(0))}"
        )
    scheduling_evidence_path = (
        Path(scheduling_evidence_root_text) / f"{scenario}.json"
        if scheduling_required
        else None
    )
    scheduling_evidence: dict[str, object] | None = (
        initial_scheduling_evidence(scenario, scheduling_profile)
        if scheduling_required
        else None
    )
    scheduling_failure = ""
    paced_drain_statistics: dict[str, object] = {
        "tick_count": 0,
        "max_elapsed_sec": 0.0,
        "max_spin_attempts": 0,
        "max_blocking_wait_attempts": 0,
        "max_zero_wait_attempts": 0,
        "deadline_miss_count": 0,
        "guard_exhaustion_count": 0,
    }
    isolated_observer_drain_statistics: dict[str, object] = {
        "step_count": 0,
        "max_elapsed_ns": 0,
        "max_spin_attempts": 0,
        "callback_count": 0,
        "idle_count": 0,
        "saturation_count": 0,
    }
    spin_observation_statistics: dict[str, object] = {
        "count": 0,
        "tail_capacity": FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY,
        "tail": [],
        "integrity_failure_count": 0,
        "last_integrity_failure": "",
    }
    scheduling_executor: SingleThreadedExecutor | None = None
    scheduling_executor_spin_active = False
    scheduling_executor_lifecycle: dict[str, object] = {
        "dedicated": scheduling_required,
        "created": False,
        "add_node_attempts": 0,
        "add_node_succeeded": False,
        "remove_node_succeeded": False,
        "shutdown_succeeded": False,
        "cleanup_failure": "",
    }
    cgroup_before = (
        cgroup_cpu_stat(os.getpid()) if scheduling_required else None
    )
    if scheduling_required and not bool(cgroup_before["available"]):
        raise AssertionError(f"cgroup cpu.stat unavailable: {cgroup_before}")
    ring = RuntimeRing(root, runtime_socket_path)
    worker_path = (
        Path(get_package_prefix("overtake_transport_contract"))
        / "lib/overtake_transport_contract/c002ay0_shadow_worker"
    )
    enabled = scenario != "off"
    configured_worker = (
        root / "missing-worker" if scenario == "unavailable" else worker_path
    )
    pp_executable = pp_executable_override or (
        Path(get_package_prefix("simple_pure_pursuit"))
        / "lib/simple_pure_pursuit/simple_pure_pursuit"
    )
    pp_log_path = root / "pure_pursuit.log"
    pp_log = pp_log_path.open("wb")
    pp_env = os.environ.copy()
    if callback_span_path is not None:
        pp_env["C002AY0_PP_CALLBACK_SPAN_PATH"] = str(callback_span_path)
    if callback_span_enabled is not None:
        pp_env["C002AY0_PP_CALLBACK_SPAN_ENABLED"] = (
            "1" if callback_span_enabled else "0"
        )
    if binding_enabled and real_state_lattice:
        pp_env["C002AY0_TEST_RELIABLE_BINDING_AUDIT"] = "1"
    if v2_uptake_only:
        pp_command_v2_params = [
            "state_lattice_v2_live_proposal_accept_enabled:=true",
            "state_lattice_v2_expected_producer_instance_id:="
            f"'{V2_UPTAKE_PLANNER_PRODUCER_INSTANCE_ID}'",
            "state_lattice_v2_base_attestation_publish_enabled:=true",
            "state_lattice_v2_base_attestation_producer_instance_id:="
            f"'{V2_UPTAKE_PP_ATTESTATION_PRODUCER_INSTANCE_ID}'",
            "state_lattice_v2_base_attestation_session_id:="
            f"'{V2_UPTAKE_SESSION_ID}'",
        ]
    else:
        pp_command_v2_params = []
    pp_command = [
        str(pp_executable),
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-p",
        "aw2_shadow_transport_enabled:=false",
        "-p",
        "use_mpc_predicted_horizon:=false",
        "-p",
        f"max_trajectory_age_sec:={20.0 if real_state_lattice else 0.2}",
        "-p",
        "use_overtake_reference_override:=false",
        "-p",
        "c002ay1_prod_measure_enabled:=true",
        "-p",
        f"c002ay1_prod_measure_socket_path:={ring.socket_path}",
        "-p",
        f"c002ay1_prod_measure_run_id:={RUN_ID}",
        "-p",
        f"c002ay1_prod_measure_session_nonce:={SESSION_NONCE}",
        "-p",
        f"c002ay1_prod_measure_instance_id:={OBSERVER_INSTANCE}",
        "-p",
        f"c002ay0_shadow_capture_enabled:={'true' if enabled else 'false'}",
        "-p",
        "state_lattice_source_binding_shadow_enabled:="
        f"{'true' if binding_enabled else 'false'}",
        "-p",
        f"c002ay0_shadow_worker_path:={configured_worker}",
        "-p",
        f"c002ay0_shadow_session_generation:={session_value}",
        "-p",
        f"c002ay0_shadow_session_nonce:={session_value + 1}",
        "-p",
        f"free_run_live_exact_ack_enabled:={'true' if free_run_live else 'false'}",
        "-r",
        "input/kinematics:=/ay0/input/kinematics",
        "-r",
        "input/trajectory:=/ay0/input/trajectory",
        "-r",
        "input/overtake_plan:=/ay0/input/overtake_plan",
        "-r",
        "input/race_armed:=/ay0/input/race_armed",
        "-r",
        "output/control_cmd:=/ay0/output/control_cmd",
        "-r",
        "output/raw_control_cmd:=/ay0/output/raw_control_cmd",
        "-r",
        "output/controller_tracking_status:=/ay0/output/tracking_status",
        "-r",
        "output/controller_command_envelope:=/ay0/output/command_envelope",
        "-r",
        "output/controller_execution_envelope:=/ay0/output/execution_envelope",
        "-r",
        "output/free_run_execution_ack:=/ay0/output/free_run_execution_ack",
        "-r",
        "output/free_run_source_key:=/ay0/output/free_run_source_key",
    ]
    for parameter in pp_command_v2_params:
        pp_command.extend(["-p", parameter])
    if scheduling_required:
        pp_command = ["taskset", "-c", pp_cpu_text, *pp_command]
    process = subprocess.Popen(
        pp_command,
        stdin=subprocess.DEVNULL,
        stdout=pp_log,
        stderr=subprocess.STDOUT,
        env=pp_env,
    )
    proposal_relay = None
    state_lattice_process = None
    state_lattice_log = None
    if (binding_enabled or v2_uptake_only) and real_state_lattice:
        state_lattice_prefix = Path(
            get_package_prefix("state_lattice_overtake_planner")
        )
        state_lattice_executable = (
            state_lattice_prefix
            / "lib/state_lattice_overtake_planner/"
            "state_lattice_overtake_planner_node"
        )
        state_lattice_worker = (
            state_lattice_prefix
            / "lib/state_lattice_overtake_planner/"
            "c002ay0_state_lattice_shadow_worker"
        )
        state_lattice_params = (
            state_lattice_prefix
            / "share/state_lattice_overtake_planner/config/"
            "state_lattice_overtake_planner.param.yaml"
        )
        state_lattice_log = (root / "state_lattice.log").open("wb")
        state_lattice_env = os.environ.copy()
        state_lattice_env["C002AY0_TEST_RELIABLE_PROPOSAL_AUDIT"] = "1"
        state_lattice_env["C002AY0_TEST_RELIABLE_PLANNER_INPUT_AUDIT"] = "1"
        state_lattice_process = subprocess.Popen(
            [
                str(state_lattice_executable),
                "--ros-args",
                "--params-file",
                str(state_lattice_params),
                "-p",
                "use_sim_time:=true",
                "-p",
                "own_vehicle_id:=d2",
                "-p",
                "ego_state_topic:=/ay0/input/kinematics",
                "-p",
                "opponent_topic:=/ay0/input/v2x",
                "-p",
                "live_control_output_enabled:=false",
                "-p",
                "instant_control_enabled:=false",
                "-p",
                "controller_trackability_profile:=shadow_only",
                "-p",
                "safety_evaluation_enabled:=true",
                "-p",
                "c002ay0_state_lattice_shadow_enabled:=true",
                "-p",
                f"c002ay0_state_lattice_shadow_worker_path:={state_lattice_worker}",
                "-p",
                f"c002ay0_state_lattice_shadow_session_generation:={session_value + 100}",
                "-p",
                f"c002ay0_state_lattice_shadow_session_nonce:={session_value + 101}",
                "-p",
                "reference_override_topic:=/ay0/debug/state_lattice/"
                "reference_override",
                *(
                    [
                        "-p",
                        "state_lattice_v2_live_proposal_publish_enabled:=true",
                        "-p",
                        "state_lattice_v2_producer_instance_id:="
                        f"{V2_UPTAKE_PLANNER_PRODUCER_INSTANCE_ID}",
                        "-p",
                        "state_lattice_v2_base_attestation_accept_enabled:=true",
                        "-p",
                        "state_lattice_v2_expected_pp_producer_instance_id:="
                        f"'{V2_UPTAKE_PP_ATTESTATION_PRODUCER_INSTANCE_ID}'",
                        "-p",
                        "state_lattice_v2_expected_pp_session_id:="
                        f"'{V2_UPTAKE_SESSION_ID}'",
                    ]
                    if v2_uptake_only
                    else []
                ),
            ],
            stdin=subprocess.DEVNULL,
            stdout=state_lattice_log,
            stderr=subprocess.STDOUT,
            env=state_lattice_env,
        )
    elif binding_enabled:
        if proposal_relay_path is None:
            raise AssertionError("binding ON requires exact proposal relay")
        proposal_relay = subprocess.Popen(
            [str(proposal_relay_path)],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    capture = Capture(
        node,
        session_value,
        session_value + 1,
        cohort_alignment_enabled=True,
        callback_instrumentation_enabled=scheduling_required,
    )
    v2_uptake_capture = V2UptakeCapture(node) if v2_uptake_only else None
    isolated_input_publisher = None
    isolated_input_publisher_markers: dict[str, object] | None = None
    isolated_initial_ack: dict[str, object] | None = None
    isolated_armed_state = False
    isolated_phase_acks: list[dict[str, object]] = []
    race_armed_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    if isolated_input_publisher_diagnostic:
        try:
            from c002ay0_isolated_input_publisher import (
                IsolatedInputPublisher,
                MARKERS as ISOLATED_INPUT_PUBLISHER_MARKERS,
            )

            isolated_input_publisher_markers = dict(
                ISOLATED_INPUT_PUBLISHER_MARKERS
            )
            isolated_input_publisher = IsolatedInputPublisher(
                curved=curved,
                real_state_lattice=real_state_lattice,
            )
            (
                isolated_armed_state,
                isolated_initial_ack,
            ) = synchronize_isolated_input_phase(
                isolated_input_publisher,
                False,
                isolated_armed_state,
                force=True,
            )
            assert isolated_initial_ack is not None
            isolated_phase_acks.append(
                {
                    **isolated_initial_ack,
                    "desired_armed": False,
                    "request_stage": "initial_disarmed_phase",
                }
            )
            clock_pub = None
            odom_pub = None
            v2x_pub = None
            trajectory_pub = None
            plan_pub = None
            race_armed_pub = None
        except BaseException as error:
            cleanup_pre_run_case_failure(
                error,
                node=node,
                capture=capture,
                isolated_input_publisher=isolated_input_publisher,
                proposal_relay=proposal_relay,
                state_lattice_process=state_lattice_process,
                process=process,
                state_lattice_log=state_lattice_log,
                pp_log=pp_log,
                ring=ring,
            )
            raise
    else:
        clock_pub = node.create_publisher(Clock, "/clock", 10)
        odom_pub = node.create_publisher(
            Odometry,
            "/ay0/input/kinematics",
            10,
        )
        v2x_pub = node.create_publisher(
            V2XVehiclePositionArray,
            "/ay0/input/v2x",
            10,
        )
        trajectory_pub = node.create_publisher(
            Trajectory,
            "/ay0/input/trajectory",
            10,
        )
        plan_pub = node.create_publisher(
            OvertakePlan,
            "/ay0/input/overtake_plan",
            10,
        )
        race_armed_pub = node.create_publisher(
            Bool,
            "/ay0/input/race_armed",
            race_armed_qos,
        )
    clock_message = fixed_clock()
    odom_message = odometry(real_state_lattice)
    trajectory_message = trajectory(curved, real_state_lattice)
    v2x_message = v2x_fixture(v2_uptake_only)
    plan_message = overtake_plan()
    next_contract_publish = 0.0
    trajectory_published = False
    race_armed = False
    sim_time_ns = FIXED_SEC * 1_000_000_000
    if isolated_initial_ack is not None:
        sim_time_ns = int(isolated_initial_ack["stamp_ns"])
    sim_start_monotonic: float | None = None

    def input_subscription_count(
        topic: str,
        publisher: object | None,
    ) -> int:
        if isolated_input_publisher is not None:
            return node.count_subscribers(topic)
        assert publisher is not None
        return publisher.get_subscription_count()

    def set_stamp(stamp: object, value_ns: int) -> None:
        stamp.sec = value_ns // 1_000_000_000
        stamp.nanosec = value_ns % 1_000_000_000

    def synchronize_isolated_phase(
        armed: bool,
        request_stage: str,
        *,
        force: bool = False,
    ) -> dict[str, object] | None:
        nonlocal isolated_armed_state, sim_time_ns
        isolated_armed_state, ack = synchronize_isolated_input_phase(
            isolated_input_publisher,
            armed,
            isolated_armed_state,
            force=force,
        )
        if ack is not None:
            sim_time_ns = int(ack["stamp_ns"])
            isolated_phase_acks.append(
                {
                    **ack,
                    "desired_armed": armed,
                    "request_stage": request_stage,
                }
            )
        return ack

    def publish_clock_and_arm(armed: bool, *, advance: bool = True) -> None:
        nonlocal sim_start_monotonic, sim_time_ns
        if isolated_input_publisher is not None:
            if armed != isolated_armed_state:
                raise AssertionError(
                    "isolated phase was not synchronized before publish"
                )
            return
        if advance:
            if sim_start_monotonic is None:
                sim_start_monotonic = time.monotonic()
            elapsed_ns = int(
                (time.monotonic() - sim_start_monotonic) * 1_000_000_000
            )
            sim_time_ns = (
                FIXED_SEC * 1_000_000_000
                + (elapsed_ns // SIM_CLOCK_QUANTUM_NS)
                * SIM_CLOCK_QUANTUM_NS
            )
        set_stamp(clock_message.clock, sim_time_ns)
        clock_pub.publish(clock_message)
        race_armed_pub.publish(Bool(data=armed))

    def publish_inputs() -> None:
        """Drive 100 Hz inputs without unrealistically flooding 20 Hz contracts."""
        nonlocal next_contract_publish, trajectory_published
        if isolated_input_publisher is not None:
            return
        now = time.monotonic()
        publish_clock_and_arm(race_armed)
        set_stamp(odom_message.header.stamp, sim_time_ns)
        odom_pub.publish(odom_message)
        if real_state_lattice:
            set_stamp(v2x_message.header.stamp, sim_time_ns)
            for vehicle in v2x_message.vehicles:
                set_stamp(vehicle.header.stamp, sim_time_ns)
            v2x_pub.publish(v2x_message)
        if now < next_contract_publish:
            return
        set_stamp(trajectory_message.header.stamp, sim_time_ns)
        set_stamp(plan_message.header.stamp, sim_time_ns)
        if not real_state_lattice or not trajectory_published:
            trajectory_pub.publish(trajectory_message)
            trajectory_published = True
        plan_pub.publish(plan_message)
        next_contract_publish = now + (0.1 if real_state_lattice else 0.05)

    def spin_fixture_once(
        timeout_sec: float,
    ) -> dict[str, object] | None:
        def spin_once_action(
            observed_node: object,
            *,
            timeout_sec: float,
        ) -> None:
            nonlocal scheduling_executor_spin_active
            if scheduling_required:
                if (
                    scheduling_executor is None
                    or scheduling_executor_lifecycle.get(
                        "add_node_succeeded"
                    )
                    is not True
                    or observed_node is not node
                ):
                    raise AssertionError(
                        "dedicated scheduling executor unavailable"
                    )
                if scheduling_executor_spin_active:
                    raise AssertionError(
                        "concurrent scheduling executor spin rejected"
                    )
                scheduling_executor_spin_active = True
                try:
                    scheduling_executor.spin_once(timeout_sec=timeout_sec)
                finally:
                    scheduling_executor_spin_active = False
                return
            rclpy.spin_once(observed_node, timeout_sec=timeout_sec)

        return spin_fixture_once_with_observation(
            node,
            timeout_sec,
            scheduling_required=scheduling_required,
            capture=capture,
            statistics=spin_observation_statistics,
            spin_once_action=spin_once_action,
        )

    def drive_input_tick(*, legacy_timeout_sec: float = 0.005) -> int:
        synchronize_isolated_phase(
            race_armed,
            "drive_input_tick",
        )
        if scheduling_required:
            if isolated_input_publisher is not None:
                return drive_isolated_observer_drain_step(
                    spin_fixture_once,
                    statistics=isolated_observer_drain_statistics,
                )
            return drive_paced_fixture_tick(
                publish_inputs,
                spin_fixture_once,
                statistics=paced_drain_statistics,
                break_on_first_zero_idle=idle_break_diagnostic,
            )
        publish_inputs()
        spin_fixture_once(legacy_timeout_sec)
        return 1

    def drive_clock_arm_tick(
        armed: bool,
        *,
        advance: bool = False,
        legacy_timeout_sec: float = 0.005,
    ) -> int:
        synchronize_isolated_phase(
            armed,
            "drive_clock_arm_tick",
        )
        publish_action = lambda: publish_clock_and_arm(
            armed,
            advance=advance,
        )
        if scheduling_required:
            if isolated_input_publisher is not None:
                return drive_isolated_observer_drain_step(
                    spin_fixture_once,
                    statistics=isolated_observer_drain_statistics,
                )
            return drive_paced_fixture_tick(
                publish_action,
                spin_fixture_once,
                statistics=paced_drain_statistics,
                break_on_first_zero_idle=idle_break_diagnostic,
            )
        publish_action()
        spin_fixture_once(legacy_timeout_sec)
        return 1

    fault_worker = -1
    fault_injected = False
    graph_counts: dict[str, int] = {}
    e2e_summary: dict[str, object] = {}
    measurement_start_ring_sequence: int | None = None
    measurement_start_write_index: int | None = None
    try:
        if scheduling_required:
            scheduling_executor = SingleThreadedExecutor(
                context=node.context
            )
            scheduling_executor_lifecycle["created"] = True
            scheduling_executor_lifecycle["add_node_attempts"] = 1
            if scheduling_executor.add_node(node) is not True:
                raise AssertionError(
                    "dedicated scheduling executor add_node failed"
                )
            scheduling_executor_lifecycle["add_node_succeeded"] = True
        if real_state_lattice:
            input_discovery_deadline = (
                time.monotonic() + INPUT_DISCOVERY_TIMEOUT_SEC
            )
            while time.monotonic() < input_discovery_deadline:
                drive_clock_arm_tick(False, legacy_timeout_sec=0.01)
                minimum_odom_subscriptions = (
                    2 if (binding_enabled or v2_uptake_only) else 1
                )
                if (
                    input_subscription_count(
                        "/ay0/input/kinematics",
                        odom_pub,
                    )
                    >= minimum_odom_subscriptions
                    and input_subscription_count(
                        "/ay0/input/trajectory",
                        trajectory_pub,
                    )
                    >= 1
                ):
                    break
            if (
                input_subscription_count(
                    "/ay0/input/kinematics",
                    odom_pub,
                )
                < (2 if (binding_enabled or v2_uptake_only) else 1)
                or input_subscription_count(
                    "/ay0/input/trajectory",
                    trajectory_pub,
                )
                < 1
            ):
                raise AssertionError(
                    f"{scenario}: PP/State Lattice input discovery incomplete"
                )
        if real_state_lattice and binding_enabled:
            discovery_deadline = (
                time.monotonic() + BINDING_DISCOVERY_TIMEOUT_SEC
            )
            while time.monotonic() < discovery_deadline:
                drive_clock_arm_tick(False, legacy_timeout_sec=0.01)
                if (
                    input_subscription_count(
                        "/ay0/input/race_armed",
                        race_armed_pub,
                    )
                    >= 1
                    and node.count_publishers(
                        "/control/overtake/base_trajectory_snapshot"
                    )
                    >= 1
                    and node.count_publishers(
                        "/debug/overtake/state_lattice/"
                        "authorized_cartesian_trajectory"
                    )
                    >= 1
                    and node.count_publishers(
                        "/test/c002ay0/state_lattice/"
                        "binding_callback_terminal"
                    )
                    >= 1
                ):
                    break
            if input_subscription_count(
                "/ay0/input/race_armed",
                race_armed_pub,
            ) < 1:
                raise AssertionError(f"{scenario}: race-arm subscriber missing")
            if (
                node.count_publishers(
                    "/control/overtake/base_trajectory_snapshot"
                )
                < 1
            ):
                raise AssertionError(f"{scenario}: snapshot publisher missing")
            if (
                node.count_publishers(
                    "/debug/overtake/state_lattice/"
                    "authorized_cartesian_trajectory"
                )
                < 1
            ):
                raise AssertionError(f"{scenario}: proposal publisher missing")
            if (
                node.count_publishers(
                    "/test/c002ay0/state_lattice/"
                    "binding_callback_terminal"
                )
                < 1
            ):
                raise AssertionError(
                    f"{scenario}: reliable binding audit publisher missing"
                )
            if os.environ.get("C002AY0_REQUIRE_RECORDER_DISCOVERY") == "1":
                recorder_node_name = os.environ.get(
                    "C002AY0_RECORDER_NODE_NAME",
                    "rosbag2_recorder",
                )
                recorder_deadline = (
                    time.monotonic() + RECORDER_DISCOVERY_TIMEOUT_SEC
                )
                missing_recorder_topics = list(RECORDER_TOPICS)
                while time.monotonic() < recorder_deadline:
                    drive_clock_arm_tick(False, legacy_timeout_sec=0.01)
                    missing_recorder_topics = missing_recorder_subscriptions(
                        node,
                        recorder_node_name,
                    )
                    if not missing_recorder_topics:
                        break
                if missing_recorder_topics:
                    raise AssertionError(
                        f"{scenario}: recorder discovery incomplete "
                        f"node={recorder_node_name} "
                        f"missing={missing_recorder_topics}"
                    )
                print(
                    f"{scenario} C002AY0_RECORDER_DISCOVERY=PASS "
                    f"node={recorder_node_name} topics={len(RECORDER_TOPICS)}"
                )
        if v2_uptake_only:
            v2_discovery_deadline = time.monotonic() + BINDING_DISCOVERY_TIMEOUT_SEC
            while time.monotonic() < v2_discovery_deadline:
                drive_clock_arm_tick(False, legacy_timeout_sec=0.01)
                if (
                    node.count_publishers(
                        "/planning/overtake/state_lattice/v2_proposal"
                    ) >= 1
                    and node.count_publishers(
                        "/debug/overtake/state_lattice/v2_binding_status"
                    ) >= 1
                ):
                    break
            if node.count_publishers(
                "/planning/overtake/state_lattice/v2_proposal"
            ) < 1:
                raise AssertionError(f"{scenario}: V2 proposal publisher missing")
            if node.count_publishers(
                "/debug/overtake/state_lattice/v2_binding_status"
            ) < 1:
                raise AssertionError(f"{scenario}: V2 binding status publisher missing")
        race_arm_discovery_deadline = (
            time.monotonic() + RACE_ARM_DISCOVERY_TIMEOUT_SEC
        )
        while (
            input_subscription_count(
                "/ay0/input/race_armed",
                race_armed_pub,
            )
            < 1
            and time.monotonic() < race_arm_discovery_deadline
        ):
            drive_clock_arm_tick(False, legacy_timeout_sec=0.01)
        if input_subscription_count(
            "/ay0/input/race_armed",
            race_armed_pub,
        ) < 1:
            raise AssertionError(f"{scenario}: race-arm subscriber missing")
        # Every capture cohort starts in one explicit race epoch.  Production
        # shadow publication is fail-closed while disarmed; leaving ordinary
        # PP scenarios at epoch zero makes their snapshot assertion impossible
        # and gives them a different authority fixture from the E2E scenario.
        disarmed_until = (
            time.monotonic() + FIXED_DISARM_DURATION_SEC
        )
        while time.monotonic() < disarmed_until:
            drive_clock_arm_tick(False)
        race_armed = True
        armed_until = time.monotonic() + FIXED_ARM_DURATION_SEC
        while time.monotonic() < armed_until:
            drive_clock_arm_tick(True)
        minimum_capture_anchor_sequence = warmup_sequence
        # The preceding fixed-clock arm phase can end in the middle of a
        # fail-closed stale-input output cycle.  Require one later,
        # five-topic-complete cycle as an observation boundary, while
        # retaining every pre-boundary sample in the startup ledger.
        minimum_capture_anchor_sequence = max(
            warmup_sequence,
            int(capture.latest_sequence) + 1,
        )
        deadline = time.monotonic() + OUTPUT_BOUNDARY_TIMEOUT_SEC
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError(
                    f"{scenario}: PP exited early {process.returncode}"
                )
            if (
                state_lattice_process is not None
                and state_lattice_process.poll() is not None
            ):
                raise AssertionError(
                    f"{scenario}: State Lattice exited early "
                    f"{state_lattice_process.returncode}; log={root / 'state_lattice.log'}"
                )
            drive_input_tick(legacy_timeout_sec=0.01)
            if scenario in ("full", "hang", "crash") and not fault_injected:
                workers = worker_pids(process.pid)
                if workers:
                    fault_worker = workers[0]
                    os.kill(
                        fault_worker,
                        (
                            signal.SIGSTOP
                            if scenario in ("full", "hang")
                            else signal.SIGKILL
                        ),
                    )
                    fault_injected = True
            output_boundary_complete = (
                capture.has_completed_cycle(
                    minimum_capture_anchor_sequence
                )
            )
            if output_boundary_complete and (
                scenario not in ("full", "hang", "crash") or fault_injected
            ):
                break
        if scenario in ("full", "hang", "crash") and not fault_injected:
            raise AssertionError(f"{scenario}: worker fault was not injected")
        if real_state_lattice and binding_enabled:
            if capture.max_snapshot_race_arm_epoch == 0:
                raise AssertionError(f"{scenario}: warmup race epoch missing")
            expected_epoch = capture.max_snapshot_race_arm_epoch + 1
            capture.begin_e2e_measurement(expected_epoch)

            # Fence all warmup DDS traffic behind a new race-arm epoch.  Keep
            # simulation time advancing so old safety evidence expires
            # naturally instead of being rescued by a frozen clock.
            race_armed = False
            disarm_deadline = (
                time.monotonic() + E2E_DISARM_DURATION_SEC
            )
            while time.monotonic() < disarm_deadline:
                drive_input_tick()
            race_armed = True
            rearm_deadline = time.monotonic() + E2E_REARM_TIMEOUT_SEC
            while (
                capture.max_snapshot_race_arm_epoch < expected_epoch
                and time.monotonic() < rearm_deadline
            ):
                drive_input_tick()
            if capture.max_snapshot_race_arm_epoch < expected_epoch:
                raise AssertionError(
                    f"{scenario}: fenced race epoch {expected_epoch} missing"
                )
        if scheduling_required:
            if isolated_input_publisher is not None:
                evidence_publish_ack = synchronize_isolated_phase(
                    isolated_armed_state,
                    "pre_scheduling_evidence",
                    force=True,
                )
                assert evidence_publish_ack is not None
                sim_time_ns = int(evidence_publish_ack["stamp_ns"])
            (
                _,
                _,
                evidence_producer_instance_id,
            ) = capture.latest_completed_cycle_candidate(
                minimum_capture_anchor_sequence
            )
            evidence_latest_sequence = int(capture.latest_sequence)
            evidence_sim_stamp_ns = int(sim_time_ns)
            scheduling_evidence_started = time.monotonic()
            assert scheduling_evidence is not None
            scheduling_evidence["collection_status"][
                "process_snapshot"
            ] = {
                "status": "started",
                "reason": "",
            }
            collected_scheduling_evidence = collect_scheduling_evidence(
                pp_pid=process.pid,
                state_lattice_pid=(
                    state_lattice_process.pid
                    if state_lattice_process is not None
                    else None
                ),
                proposal_relay_pid=(
                    proposal_relay.pid if proposal_relay is not None else None
                ),
                expected_pp_cpus=expected_pp_cpus,
                expected_fixture_cpus=expected_fixture_cpus,
            )
            scheduling_evidence.update(collected_scheduling_evidence)
            scheduling_evidence["collection_status"][
                "process_snapshot"
            ] = {
                "status": "completed",
                "reason": "",
            }
            scheduling_evidence_elapsed_sec = (
                time.monotonic() - scheduling_evidence_started
            )
            scheduling_evidence.update(
                {
                    "scenario": scenario,
                    "profile": scheduling_profile,
                    "cgroup_before": cgroup_before,
                    "evidence_elapsed_sec": (
                        scheduling_evidence_elapsed_sec
                    ),
                    "evidence_max_sec": SCHEDULING_EVIDENCE_MAX_SEC,
                    "prearm_storage_budget_ms": (
                        PREARM_STORAGE_BUDGET_MS
                    ),
                    "prearm_storage_required_messages": (
                        PREARM_STORAGE_REQUIRED_MESSAGES
                    ),
                    "prearm_storage_capacity_messages": (
                        PREARM_CAPTURE_CAPACITY_MESSAGES
                    ),
                }
            )
            try:
                require_scheduling_evidence_elapsed(
                    scheduling_evidence_elapsed_sec
                )
            except AssertionError as error:
                raise AssertionError(f"{scenario}: {error}") from error
            post_evidence_minimum_sequence = evidence_latest_sequence + 1
            reanchor_started = time.monotonic()
            scheduling_evidence["collection_status"]["reanchor"] = {
                "status": "started",
                "reason": "",
            }
            scheduling_evidence["reanchor_fence"] = {
                "completed": False,
                "deadline_sec": SCHEDULING_REANCHOR_TIMEOUT_SEC,
                "evidence_latest_sequence": evidence_latest_sequence,
                "evidence_sim_stamp_ns": evidence_sim_stamp_ns,
                "evidence_producer_instance_id": (
                    evidence_producer_instance_id
                ),
                "isolated_phase_ack": (
                    evidence_publish_ack
                    if isolated_input_publisher is not None
                    else None
                ),
                "minimum_anchor_sequence": (
                    post_evidence_minimum_sequence
                ),
                "callback_drain": {
                    "mode": (
                        "isolated_observer"
                        if isolated_input_publisher is not None
                        else "paced_publish_and_drain"
                    ),
                    "input_period_sec": FIXTURE_INPUT_PERIOD_SEC,
                    "max_callbacks_per_tick": (
                        FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK
                    ),
                    "spin_wait_sec": FIXTURE_DRAIN_SPIN_WAIT_SEC,
                    "deadline_guard_sec": (
                        FIXTURE_DRAIN_DEADLINE_GUARD_SEC
                    ),
                    "input_release_deadline_applies": (
                        isolated_input_publisher is None
                    ),
                    "statistics": (
                        isolated_observer_drain_statistics
                        if isolated_input_publisher is not None
                        else paced_drain_statistics
                    ),
                },
                "observation_at_start": (
                    capture.fresh_reanchor_observation_summary(
                        post_evidence_minimum_sequence,
                        minimum_stamp_exclusive_ns=evidence_sim_stamp_ns,
                        expected_producer_instance_id=(
                            evidence_producer_instance_id
                        ),
                    )
                ),
            }
            reanchor_deadline = (
                reanchor_started + SCHEDULING_REANCHOR_TIMEOUT_SEC
            )

            def validate_reanchor_boundary() -> None:
                try:
                    capture.validate_fresh_reanchor_interval(
                        post_evidence_minimum_sequence,
                        minimum_stamp_exclusive_ns=evidence_sim_stamp_ns,
                        expected_producer_instance_id=(
                            evidence_producer_instance_id
                        ),
                    )
                except AssertionError as error:
                    scheduling_evidence["reanchor_fence"].update(
                        {
                            "elapsed_sec": (
                                time.monotonic() - reanchor_started
                            ),
                            "invalid": capture.reanchor_fence_invalid,
                            "pending": capture.alignment_pending_summary(),
                            "observation_at_end": (
                                capture.fresh_reanchor_observation_summary(
                                    post_evidence_minimum_sequence,
                                    minimum_stamp_exclusive_ns=(
                                        evidence_sim_stamp_ns
                                    ),
                                    expected_producer_instance_id=(
                                        evidence_producer_instance_id
                                    ),
                                )
                            ),
                            "timed_out": False,
                        }
                    )
                    raise AssertionError(
                        f"{scenario}: scheduling re-anchor invalid: {error}"
                    ) from error

            while time.monotonic() < reanchor_deadline:
                validate_reanchor_boundary()
                if capture.has_completed_cycle(
                    post_evidence_minimum_sequence,
                    minimum_stamp_exclusive_ns=evidence_sim_stamp_ns,
                    expected_producer_instance_id=(
                        evidence_producer_instance_id
                    ),
                ):
                    break
                if process.poll() is not None:
                    raise AssertionError(
                        f"{scenario}: PP exited during scheduling re-anchor "
                        f"{process.returncode}"
                    )
                if (
                    state_lattice_process is not None
                    and state_lattice_process.poll() is not None
                ):
                    raise AssertionError(
                        f"{scenario}: State Lattice exited during scheduling "
                        f"re-anchor {state_lattice_process.returncode}; "
                        f"log={root / 'state_lattice.log'}"
                    )
                drive_input_tick()
            validate_reanchor_boundary()
            if not capture.has_completed_cycle(
                post_evidence_minimum_sequence,
                minimum_stamp_exclusive_ns=evidence_sim_stamp_ns,
                expected_producer_instance_id=(
                    evidence_producer_instance_id
                ),
            ):
                scheduling_evidence["reanchor_fence"].update(
                    {
                        "elapsed_sec": (
                            time.monotonic() - reanchor_started
                        ),
                        "pending": capture.alignment_pending_summary(),
                        "observation_at_end": (
                            capture.fresh_reanchor_observation_summary(
                                post_evidence_minimum_sequence,
                                minimum_stamp_exclusive_ns=(
                                    evidence_sim_stamp_ns
                                ),
                                expected_producer_instance_id=(
                                    evidence_producer_instance_id
                                ),
                            )
                        ),
                        "timed_out": True,
                    }
                )
                raise AssertionError(
                    f"{scenario}: scheduling re-anchor cohort unavailable "
                    f"minimum_sequence={post_evidence_minimum_sequence} "
                    f"minimum_stamp_exclusive_ns={evidence_sim_stamp_ns} "
                    f"alignment={capture.alignment_pending_summary()}"
                )
            validate_reanchor_boundary()
            reanchor_observation_at_end = (
                capture.fresh_reanchor_observation_summary(
                    post_evidence_minimum_sequence,
                    minimum_stamp_exclusive_ns=evidence_sim_stamp_ns,
                    expected_producer_instance_id=(
                        evidence_producer_instance_id
                    ),
                )
            )
            selected_anchor = capture.arm_after_completed_cycle(
                post_evidence_minimum_sequence,
                minimum_stamp_exclusive_ns=evidence_sim_stamp_ns,
                expected_producer_instance_id=(
                    evidence_producer_instance_id
                ),
            )
            if selected_anchor is None:
                raise AssertionError(
                    f"{scenario}: scheduling re-anchor identity missing"
                )
            (
                selected_anchor_stamp_ns,
                selected_anchor_sequence,
                selected_anchor_producer_instance_id,
            ) = selected_anchor
            if isolated_input_publisher is not None:
                validate_post_ack_exact_cohort(
                    selected_anchor_stamp_ns,
                    evidence_publish_ack,
                )
            scheduling_evidence["reanchor_fence"].update(
                {
                    "completed": True,
                    "elapsed_sec": (
                        time.monotonic() - reanchor_started
                    ),
                    "selected_anchor_stamp_ns": (
                        selected_anchor_stamp_ns
                    ),
                    "selected_anchor_sequence": (
                        selected_anchor_sequence
                    ),
                    "selected_anchor_producer_instance_id": (
                        selected_anchor_producer_instance_id
                    ),
                    "isolated_post_ack_exact_cohort_valid": (
                        True
                        if isolated_input_publisher is not None
                        else None
                    ),
                    "observation_at_end": reanchor_observation_at_end,
                    "timed_out": False,
                }
            )
            scheduling_evidence["collection_status"]["reanchor"] = {
                "status": "completed",
                "reason": "",
            }
        else:
            capture.arm_after_completed_cycle(
                minimum_capture_anchor_sequence
            )
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and not capture.complete():
            if process.poll() is not None:
                raise AssertionError(
                    f"{scenario}: PP exited while capturing"
                )
            drive_input_tick()
        if not capture.complete():
            raise AssertionError(
                f"{scenario}: incomplete PP outputs "
                f"alignment={capture.alignment_pending_summary()}"
            )
        capture.validate_aligned_capture()
        if v2_uptake_only:
            assert v2_uptake_capture is not None
            v2_deadline = time.monotonic() + V2_UPTAKE_TIMEOUT_SEC
            while (
                len(v2_uptake_capture.proposals)
                < V2_UPTAKE_MIN_PROPOSAL_COUNT
                and time.monotonic() < v2_deadline
            ):
                if process.poll() is not None:
                    raise AssertionError(
                        f"{scenario}: PP exited before V2 uptake "
                        f"{process.returncode}"
                    )
                if (
                    state_lattice_process is not None
                    and state_lattice_process.poll() is not None
                ):
                    raise AssertionError(
                        f"{scenario}: State Lattice exited before V2 uptake "
                        f"{state_lattice_process.returncode}"
                    )
                drive_input_tick()
            while (
                not v2_uptake_capture.cohort_quiescent(
                    V2_UPTAKE_COHORT_QUIET_SEC
                )
                and time.monotonic() < v2_deadline
            ):
                if process.poll() is not None:
                    raise AssertionError(
                        f"{scenario}: PP exited before V2 cohort freeze "
                        f"{process.returncode}"
                    )
                if (
                    state_lattice_process is not None
                    and state_lattice_process.poll() is not None
                ):
                    raise AssertionError(
                        f"{scenario}: State Lattice exited before V2 cohort freeze "
                        f"{state_lattice_process.returncode}"
                    )
                spin_fixture_once(0.005)
            expected_v2_proposal_count = len(v2_uptake_capture.proposals)
            while (
                not v2_uptake_capture.ready(expected_v2_proposal_count)
                and time.monotonic() < v2_deadline
            ):
                if process.poll() is not None:
                    raise AssertionError(
                        f"{scenario}: PP exited before V2 uptake "
                        f"{process.returncode}"
                    )
                if (
                    state_lattice_process is not None
                    and state_lattice_process.poll() is not None
                ):
                    raise AssertionError(
                        f"{scenario}: State Lattice exited before V2 uptake "
                        f"{state_lattice_process.returncode}"
                    )
                spin_fixture_once(0.005)
            v2_summary = v2_uptake_capture.validate(
                expected_v2_proposal_count
            )
            (root / "v2_uptake_summary.json").write_text(
                json.dumps(v2_summary, indent=2, sort_keys=True) + "\n"
            )
        if timing_diagnostic:
            pre_measurement_records, pre_measurement_drops = ring.records()
            if pre_measurement_drops != 0 or not pre_measurement_records:
                raise AssertionError(
                    f"{scenario}: invalid timing measurement fence "
                    f"records={len(pre_measurement_records)} "
                    f"drops={pre_measurement_drops}"
                )
            if (
                len(pre_measurement_records) + timing_record_count
                > CAPACITY
            ):
                raise AssertionError(
                    f"{scenario}: timing ring capacity insufficient "
                    f"pre_measurement={len(pre_measurement_records)} "
                    f"target={timing_record_count} capacity={CAPACITY}"
                )
            measurement_start_ring_sequence = (
                pre_measurement_records[-1][0] + 1
            )
            measurement_start_write_index, _ = ring.state()
        if timing_diagnostic:
            measurement_deadline = (
                time.monotonic()
                + max(10.5, timing_record_count / 100.0 + 5.0)
            )
            while time.monotonic() < measurement_deadline:
                current_write_index, current_drops = ring.state()
                if current_drops != 0:
                    raise AssertionError(
                        f"{scenario}: timing measurement dropped records "
                        f"drops={current_drops}"
                    )
                if (
                    current_write_index - measurement_start_write_index
                    >= timing_record_count
                ):
                    break
                drive_input_tick()
            else:
                raise AssertionError(
                    f"{scenario}: timing measurement target incomplete "
                    f"target={timing_record_count}"
                )
        elif scenario == "healthy" or measure_long:
            # p99.9 must not collapse to max; run longer than 1000 samples at
            # 100 Hz before treating this percentile as a tail measurement.
            measurement_deadline = time.monotonic() + 10.5
            while time.monotonic() < measurement_deadline:
                drive_input_tick()
        if real_state_lattice and binding_enabled:
            # Stop creating new eligible records, but keep /clock progressing
            # for a bounded 500 ms drain.  Every proposal observed in the
            # fenced epoch must reach one terminal binding result.
            race_armed = False
            drain_deadline = time.monotonic() + 0.5
            while time.monotonic() < drain_deadline:
                drive_clock_arm_tick(False, advance=True)
            e2e_summary = capture.finalize_e2e_measurement()
        if scenario in ("full", "hang", "crash"):
            time.sleep(0.1)
            if len(worker_pids(process.pid)) > 1:
                raise AssertionError(f"{scenario}: worker auto-respawned")
        signature = capture.signature()
    finally:
        active_primary_error = sys.exc_info()[1]
        if (
            scheduling_evidence is not None
            and active_primary_error is not None
            and scheduling_evidence.get("primary_first_false") is None
        ):
            scheduling_evidence["primary_first_false"] = (
                primary_first_false_record(
                    active_primary_error,
                    (
                        isolated_observer_drain_statistics
                        if isolated_input_publisher is not None
                        else paced_drain_statistics
                    ),
                )
            )
            collection_status = scheduling_evidence.get(
                "collection_status",
                {},
            )
            if isinstance(collection_status, dict):
                for stage in ("process_snapshot", "reanchor"):
                    stage_status = collection_status.get(stage)
                    if (
                        isinstance(stage_status, dict)
                        and stage_status.get("status") != "completed"
                    ):
                        stage_status["reason"] = (
                            "primary_first_false_before_completion"
                        )
        executor_cleanup_errors = []
        if scheduling_executor is not None:
            if scheduling_executor_lifecycle.get(
                "add_node_succeeded"
            ) is True:
                try:
                    scheduling_executor.remove_node(node)
                    scheduling_executor_lifecycle[
                        "remove_node_succeeded"
                    ] = True
                except BaseException as error:
                    executor_cleanup_errors.append(
                        f"remove_node={error}"
                    )
            try:
                shutdown_result = scheduling_executor.shutdown(
                    timeout_sec=1.0
                )
                if shutdown_result is not True:
                    raise AssertionError(
                        "dedicated executor shutdown timed out"
                    )
                scheduling_executor_lifecycle[
                    "shutdown_succeeded"
                ] = True
            except BaseException as error:
                executor_cleanup_errors.append(f"shutdown={error}")
        if executor_cleanup_errors:
            scheduling_executor_lifecycle["cleanup_failure"] = "; ".join(
                executor_cleanup_errors
            )
            if not scheduling_failure:
                scheduling_failure = (
                    f"{scenario}: dedicated scheduling executor cleanup failed "
                    f"{scheduling_executor_lifecycle['cleanup_failure']}"
                )
        isolated_publisher_cleanup_failure = ""
        isolated_publisher_lifecycle: dict[str, object] | None = None
        isolated_associated_child_lifecycle: list[
            dict[str, object]
        ] = []
        if isolated_input_publisher is not None:
            cleanup_errors = []
            isolated_publisher_lifecycle = {
                **isolated_input_publisher.process_status,
                "provenance": "measurement_spawn_isolated_input_publisher",
                "cleanup_stage": "shutdown_requested",
                "cleanup_failure": "",
                "cleanup_ack": None,
                "identity_before_cleanup": (
                    fixture_process_identity_snapshot(
                        isolated_input_publisher.pid
                    )
                ),
            }
            try:
                isolated_publisher_lifecycle["cleanup_ack"] = (
                    isolated_input_publisher.shutdown()
                )
                isolated_publisher_lifecycle["cleanup_stage"] = (
                    "shutdown_acknowledged"
                )
            except BaseException as error:
                cleanup_errors.append(f"shutdown={error}")
                isolated_publisher_lifecycle["cleanup_stage"] = (
                    "shutdown_failed"
                )
            try:
                isolated_input_publisher.close()
            except BaseException as error:
                cleanup_errors.append(f"close={error}")
                isolated_publisher_lifecycle["cleanup_stage"] = (
                    "close_failed"
                )
            final_isolated_status = (
                isolated_input_publisher.process_status
            )
            isolated_associated_child_lifecycle = list(
                final_isolated_status.get(
                    "associated_child_lifecycle",
                    [],
                )
            )
            associated_cleanup_failure = str(
                final_isolated_status.get(
                    "associated_child_cleanup_failure",
                    "",
                )
            )
            associated_sample_failures = list(
                final_isolated_status.get(
                    "associated_child_sample_failures",
                    [],
                )
            )
            if (
                associated_cleanup_failure
                and not any(
                    associated_cleanup_failure in error
                    for error in cleanup_errors
                )
            ):
                cleanup_errors.append(
                    "associated_children="
                    f"{associated_cleanup_failure}"
                )
            refresh_isolated_publisher_lifecycle(
                isolated_publisher_lifecycle,
                final_isolated_status,
            )
            target_cleanup_failure = str(
                final_isolated_status.get(
                    "target_cleanup_failure",
                    "",
                )
            )
            if (
                target_cleanup_failure
                and not any(
                    target_cleanup_failure in error
                    for error in cleanup_errors
                )
            ):
                cleanup_errors.append(
                    f"target={target_cleanup_failure}"
                )
            if not cleanup_errors:
                isolated_publisher_lifecycle["cleanup_stage"] = (
                    "closed_no_residue"
                )
            if cleanup_errors:
                isolated_publisher_cleanup_failure = "; ".join(
                    cleanup_errors
                )
                isolated_publisher_lifecycle["cleanup_failure"] = (
                    isolated_publisher_cleanup_failure
                )
                if active_primary_error is not None:
                    setattr(
                        active_primary_error,
                        "_isolated_publisher_cleanup_failure",
                        isolated_publisher_cleanup_failure,
                    )
        if (
            scheduling_required
            and scheduling_evidence is not None
            and process.poll() is None
            and (
                state_lattice_process is None
                or state_lattice_process.poll() is None
            )
        ):
            try:
                final_scheduling_evidence = collect_scheduling_evidence(
                    pp_pid=process.pid,
                    state_lattice_pid=(
                        state_lattice_process.pid
                        if state_lattice_process is not None
                        else None
                    ),
                    proposal_relay_pid=(
                        proposal_relay.pid
                        if proposal_relay is not None
                        else None
                    ),
                    expected_pp_cpus=expected_pp_cpus,
                    expected_fixture_cpus=expected_fixture_cpus,
                )
                scheduling_evidence["final_processes"] = (
                    final_scheduling_evidence["processes"]
                )
            except (AssertionError, OSError, ProcessLookupError) as error:
                scheduling_failure = (
                    f"{scenario}: final scheduling evidence failed: {error}"
                )
        pp_descendants = all_descendant_pids(process.pid)
        state_lattice_descendants = (
            all_descendant_pids(state_lattice_process.pid)
            if state_lattice_process is not None
            else []
        )
        proposal_relay_pid = (
            proposal_relay.pid if proposal_relay is not None else None
        )
        state_lattice_pid = (
            state_lattice_process.pid
            if state_lattice_process is not None
            else None
        )
        pp_pid = process.pid
        owned_process_identities = {
            pid: fixture_process_identity_snapshot(pid)
            for pid in {
                pp_pid,
                *pp_descendants,
                *state_lattice_descendants,
                *(
                    (state_lattice_pid,)
                    if state_lattice_pid is not None
                    else ()
                ),
                *(
                    (proposal_relay_pid,)
                    if proposal_relay_pid is not None
                    else ()
                ),
            }
        }
        cgroup_after = (
            cgroup_cpu_stat(os.getpid()) if scheduling_required else None
        )
        if real_state_lattice:
            graph_counts = {
                "race_armed_subscriptions": input_subscription_count(
                    "/ay0/input/race_armed",
                    race_armed_pub,
                ),
                "odom_subscriptions": input_subscription_count(
                    "/ay0/input/kinematics",
                    odom_pub,
                ),
                "trajectory_subscriptions": (
                    input_subscription_count(
                        "/ay0/input/trajectory",
                        trajectory_pub,
                    )
                ),
                "v2x_subscriptions": input_subscription_count(
                    "/ay0/input/v2x",
                    v2x_pub,
                ),
                "snapshot_publishers": node.count_publishers(
                    "/control/overtake/base_trajectory_snapshot"
                ),
                "proposal_publishers": node.count_publishers(
                    "/debug/overtake/state_lattice/"
                    "authorized_cartesian_trajectory"
                ),
                "binding_audit_publishers": node.count_publishers(
                    "/test/c002ay0/state_lattice/"
                    "binding_callback_terminal"
                ),
                "binding_publishers": node.count_publishers(
                    "/debug/overtake/state_lattice/source_binding"
                ),
            }
        if proposal_relay is not None:
            stop_process(proposal_relay)
        if state_lattice_process is not None:
            stop_process(state_lattice_process)
        if state_lattice_log is not None:
            state_lattice_log.close()
        stop_process(process)
        pp_log.close()
        if scheduling_required:
            process_lifecycle: list[dict[str, object]] = []

            def append_lifecycle(
                pid: int,
                role: str,
                provenance: str,
                cleanup_stage: str,
                exitcode: int | None,
            ) -> None:
                alive = Path(f"/proc/{pid}").exists()
                process_lifecycle.append(
                    {
                        "pid": pid,
                        "role": role,
                        "provenance": provenance,
                        "cleanup_stage": cleanup_stage,
                        "alive": alive,
                        "exitcode": exitcode,
                        "unresolved": alive,
                        "identity_before_cleanup": (
                            owned_process_identities.get(pid)
                        ),
                    }
                )

            append_lifecycle(
                pp_pid,
                "pure_pursuit",
                "measurement_subprocess_handle",
                "stop_process_complete",
                process.poll(),
            )
            for descendant in pp_descendants:
                append_lifecycle(
                    descendant,
                    "pure_pursuit_descendant",
                    f"descendant_of_pure_pursuit_pid_{pp_pid}",
                    "parent_stop_reconciled",
                    None,
                )
            if state_lattice_pid is not None:
                append_lifecycle(
                    state_lattice_pid,
                    "state_lattice",
                    "measurement_subprocess_handle",
                    "stop_process_complete",
                    state_lattice_process.poll(),
                )
            for descendant in state_lattice_descendants:
                append_lifecycle(
                    descendant,
                    "state_lattice_descendant",
                    f"descendant_of_state_lattice_pid_{state_lattice_pid}",
                    "parent_stop_reconciled",
                    None,
                )
            if proposal_relay_pid is not None:
                append_lifecycle(
                    proposal_relay_pid,
                    "proposal_relay",
                    "measurement_subprocess_handle",
                    "stop_process_complete",
                    proposal_relay.poll(),
                )
            if isolated_publisher_lifecycle is not None:
                process_lifecycle.append(isolated_publisher_lifecycle)
            process_lifecycle.extend(
                isolated_associated_child_lifecycle
            )
            if scheduling_evidence is None:
                scheduling_evidence = {
                    "scenario": scenario,
                    "profile": scheduling_profile,
                }
            scheduling_evidence.update(
                {
                    "diagnostic_only": True,
                    "not_acceptance": True,
                    "m4_wcet_eligible": False,
                    "no_retry": True,
                    "idle_break_diagnostic": idle_break_diagnostic,
                    "isolated_input_publisher_diagnostic": (
                        isolated_input_publisher_diagnostic
                    ),
                    "isolated_input_publisher_markers": (
                        isolated_input_publisher_markers
                        if isolated_input_publisher_diagnostic
                        else None
                    ),
                    "dds_environment_contract": (
                        dict(DDS_ENVIRONMENT_CONTRACT)
                        if isolated_input_publisher_diagnostic
                        else None
                    ),
                    "isolated_input_publisher_cleanup_failure": (
                        isolated_publisher_cleanup_failure
                    ),
                    "isolated_phase_acks": isolated_phase_acks,
                    "process_lifecycle": process_lifecycle,
                    "scheduling_executor_lifecycle": (
                        scheduling_executor_lifecycle
                    ),
                }
            )
            scheduling_evidence["paced_callback_drain"] = {
                "applicable": isolated_input_publisher is None,
                "input_period_sec": FIXTURE_INPUT_PERIOD_SEC,
                "max_callbacks_per_tick": (
                    FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK
                ),
                "spin_wait_sec": FIXTURE_DRAIN_SPIN_WAIT_SEC,
                "deadline_guard_sec": FIXTURE_DRAIN_DEADLINE_GUARD_SEC,
                "elapsed_scope": "publish_and_callback_drain_before_pacing_sleep",
                "idle_break_diagnostic": idle_break_diagnostic,
                "spin_observation": spin_observation_statistics,
                **paced_drain_statistics,
            }
            scheduling_evidence["isolated_observer_drain"] = {
                "applicable": isolated_input_publisher is not None,
                "diagnostic_only": True,
                "not_acceptance": True,
                "m4_wcet_eligible": False,
                "input_release_authority": (
                    "isolated_input_publisher"
                    if isolated_input_publisher is not None
                    else "not_applicable"
                ),
                "input_release_period_sec": FIXTURE_INPUT_PERIOD_SEC,
                "input_release_gap_gate_sec": 0.012,
                "input_release_deadline_applies": False,
                "observer_processing_scope": (
                    "fixture_only_canonical_hash_alignment_and_ledger"
                ),
                "observer_processing_in_input_release_deadline": False,
                "canonical_hash_ledger_deferred": False,
                "max_callbacks_per_step": (
                    FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK
                ),
                "spin_wait_sec": FIXTURE_DRAIN_SPIN_WAIT_SEC,
                "spin_observation": spin_observation_statistics,
                **isolated_observer_drain_statistics,
            }
            residue_candidates = [
                pp_pid,
                *pp_descendants,
                *state_lattice_descendants,
            ]
            if state_lattice_pid is not None:
                residue_candidates.append(state_lattice_pid)
            if proposal_relay_pid is not None:
                residue_candidates.append(proposal_relay_pid)
            residue = [
                pid
                for pid in sorted(set(residue_candidates))
                if not wait_process_gone(pid, 1.0)
            ]
            unresolved_lifecycle_pids = sorted(
                int(record["pid"])
                for record in process_lifecycle
                if bool(record["unresolved"])
            )
            residue = sorted(
                set(residue) | set(unresolved_lifecycle_pids)
            )
            if residue:
                scheduling_failure = (
                    f"{scenario}: fixture-owned process residue {residue}"
                )
            if not bool(cgroup_after and cgroup_after["available"]):
                scheduling_failure = (
                    f"{scenario}: final cgroup cpu.stat unavailable "
                    f"{cgroup_after}"
                )
            if cgroup_before and cgroup_after:
                before_values = cgroup_before.get("values", {})
                after_values = cgroup_after.get("values", {})
                throttle_delta = {
                    key: int(after_values.get(key, 0))
                    - int(before_values.get(key, 0))
                    for key in ("nr_throttled", "throttled_usec")
                }
                if any(value > 0 for value in throttle_delta.values()):
                    scheduling_failure = (
                        f"{scenario}: cgroup CPU throttling increased "
                        f"{throttle_delta}"
                    )
                if scheduling_evidence is not None:
                    scheduling_evidence.update(
                        {
                            "cgroup_after": cgroup_after,
                            "cgroup_throttle_delta": throttle_delta,
                            "residue_pids": residue,
                        }
                    )
            if scheduling_evidence is not None:
                isolated_lifecycle_valid = (
                    not isolated_input_publisher_diagnostic
                    or (
                        isolated_publisher_lifecycle is not None
                        and isolated_publisher_lifecycle.get(
                            "cleanup_stage"
                        )
                        == "closed_no_residue"
                        and isolated_publisher_lifecycle.get("alive")
                        is False
                        and isolated_publisher_lifecycle.get(
                            "exitcode"
                        )
                        == 0
                        and isolated_publisher_lifecycle.get(
                            "unresolved"
                        )
                        is False
                    )
                )
                scheduling_evidence["scenario_integrity_valid"] = (
                    not scheduling_failure
                    and not residue
                    and not isolated_publisher_cleanup_failure
                    and scheduling_executor_lifecycle_valid(
                        scheduling_required,
                        scheduling_executor_lifecycle,
                    )
                    and isolated_lifecycle_valid
                    and scheduling_evidence.get(
                        "reanchor_fence",
                        {},
                    ).get("completed")
                    is True
                    and not any(
                        bool(record["unresolved"])
                        for record in process_lifecycle
                    )
                )
            if scheduling_evidence_path is not None:
                scheduling_evidence_path.write_text(
                    json.dumps(
                        scheduling_evidence
                        or {
                            "scenario": scenario,
                            "profile": scheduling_profile,
                            "error": "evidence collection did not complete",
                        },
                        indent=2,
                        sort_keys=True,
                    )
                    + "\n"
                )
        if (
            real_state_lattice
            and binding_enabled
            and capture.shadow_snapshot_count == 0
        ):
            pp_tail = pp_log_path.read_text(errors="replace")[-4000:]
            state_lattice_tail = (
                (root / "state_lattice.log").read_text(errors="replace")[-4000:]
                if (root / "state_lattice.log").exists()
                else ""
            )
            print(
                f"{scenario} zero-snapshot diagnostics "
                f"graph={graph_counts}:\n"
                f"--- pure_pursuit.log ---\n{pp_tail}\n"
                f"--- state_lattice.log ---\n{state_lattice_tail}"
            )
        if scenario in ("full", "hang") and fault_worker > 0:
            if not wait_process_gone(fault_worker, 1.0):
                raise AssertionError("hung worker remained after PP shutdown")
        if isolated_input_publisher is None:
            node.destroy_publisher(clock_pub)
            node.destroy_publisher(odom_pub)
            node.destroy_publisher(v2x_pub)
            node.destroy_publisher(trajectory_pub)
            node.destroy_publisher(plan_pub)
            node.destroy_publisher(race_armed_pub)
        for subscription in [
            *capture.subscriptions,
            *(v2_uptake_capture.subscriptions if v2_uptake_capture else []),
        ]:
            node.destroy_subscription(subscription)
        if (
            isolated_publisher_cleanup_failure
            and active_primary_error is None
        ):
            raise AssertionError(
                "isolated input publisher cleanup failed: "
                f"{isolated_publisher_cleanup_failure}"
            )
    if scheduling_failure:
        raise AssertionError(scheduling_failure)
    records, drops = ring.records()
    ring.close()
    if measurement_start_ring_sequence is not None:
        selected_records = [
            record
            for record in records
            if record[0] >= measurement_start_ring_sequence
        ][:timing_record_count]
        expected_sequences = list(
            range(
                measurement_start_ring_sequence,
                measurement_start_ring_sequence
                + timing_record_count,
            )
        )
        if (
            drops != 0
            or len(selected_records) != timing_record_count
            or [record[0] for record in selected_records]
            != expected_sequences
        ):
            raise AssertionError(
                f"{scenario}: timing cohort invalid "
                f"start={measurement_start_ring_sequence} "
                f"records={len(selected_records)} drops={drops}"
            )
        records = selected_records
        (root / "runtime_records.json").write_text(
            json.dumps(
                {
                    "measurement_start_ring_sequence": (
                        measurement_start_ring_sequence
                    ),
                    "target_record_count": timing_record_count,
                    "record_count": len(records),
                    "drop_count": drops,
                    "records": records,
                },
                sort_keys=True,
            )
            + "\n"
        )
        (root / "canonical_output.json").write_text(
            strict_json_text(
                {
                    "signature": signature,
                    "identity_signature": capture.identity_signature(),
                    "canonical_semantic": (
                        capture.canonical_semantic_signature()
                    ),
                    "startup_alignment": (
                        capture.startup_alignment_summary()
                    ),
                },
                sort_keys=True,
            )
            + "\n"
        )
    return (
        signature,
        records,
        drops,
        fault_worker,
        capture.max_abs_steering,
        capture.shadow_snapshot_count,
        capture.shadow_snapshot_invalid,
        capture.free_run_ack_count,
        capture.free_run_source_key_count,
        capture.authorized_proposal_count,
        capture.binding_count,
        capture.exact_binding_count,
        capture.binding_invalid,
        capture.binding_dispositions,
        capture.first_debug,
        e2e_summary,
        capture.identity_signature(),
        (
            parse_callback_spans(callback_span_path)
            if callback_span_path is not None
            else None
        ),
    )


def parse_callback_spans(
    path: Path,
) -> tuple[list[tuple[int, int, int, int, int]], bool]:
    if not path.exists():
        raise AssertionError(f"callback span output missing: {path}")
    lines = path.read_text().splitlines()
    if not lines:
        raise AssertionError("callback span output is empty")
    header = lines[0].split()
    if len(header) != 4 or header[0] != "count" or header[2] != "overflow":
        raise AssertionError("callback span header is invalid")
    expected_count = int(header[1])
    overflowed = bool(int(header[3]))
    records = []
    for line in lines[1:]:
        values = tuple(int(value) for value in line.split())
        if len(values) != 5:
            raise AssertionError("callback span record is invalid")
        records.append(values)
    if len(records) != expected_count:
        raise AssertionError(
            "callback span count mismatch "
            f"header={expected_count} records={len(records)}"
        )
    return records, overflowed


def callback_window_attribution(
    spans: list[tuple[int, int, int, int, int]],
    window_start_ns: int,
    window_end_ns: int,
) -> dict[str, object]:
    by_kind: dict[int, int] = {}
    known_busy_ns = 0
    for _, kind, entry_ns, exit_ns, _ in spans:
        overlap_ns = max(
            0,
            min(exit_ns, window_end_ns) - max(entry_ns, window_start_ns),
        )
        if overlap_ns:
            by_kind[kind] = by_kind.get(kind, 0) + overlap_ns
            known_busy_ns += overlap_ns
    window_ns = max(0, window_end_ns - window_start_ns)
    if known_busy_ns > window_ns:
        raise AssertionError("callback spans overlap inside attribution window")
    return {
        "window_ns": window_ns,
        "known_busy_ns": known_busy_ns,
        "unattributed_residual_ns": window_ns - known_busy_ns,
        "known_busy_by_kind_ns": by_kind,
    }


def exact_field_differences(
    lhs: object,
    rhs: object,
    path: str = "$",
) -> list[dict[str, object]]:
    differences: list[dict[str, object]] = []
    if type(lhs) is not type(rhs):
        return [
            {
                "path": path,
                "kind": "type",
                "off_type": type(lhs).__name__,
                "on_type": type(rhs).__name__,
                "off": lhs,
                "on": rhs,
            }
        ]
    if isinstance(lhs, dict):
        lhs_keys = set(lhs)
        rhs_keys = set(rhs)
        for key in sorted(lhs_keys | rhs_keys):
            child_path = f"{path}.{key}"
            if key not in lhs:
                differences.append(
                    {
                        "path": child_path,
                        "kind": "missing_off",
                        "on": rhs[key],
                    }
                )
            elif key not in rhs:
                differences.append(
                    {
                        "path": child_path,
                        "kind": "missing_on",
                        "off": lhs[key],
                    }
                )
            else:
                differences.extend(
                    exact_field_differences(
                        lhs[key],
                        rhs[key],
                        child_path,
                    )
                )
        return differences
    if isinstance(lhs, list):
        if len(lhs) != len(rhs):
            differences.append(
                {
                    "path": path,
                    "kind": "length",
                    "off": len(lhs),
                    "on": len(rhs),
                }
            )
        for index, (lhs_value, rhs_value) in enumerate(zip(lhs, rhs)):
            differences.extend(
                exact_field_differences(
                    lhs_value,
                    rhs_value,
                    f"{path}[{index}]",
                )
            )
        return differences
    if lhs != rhs:
        differences.append(
            {
                "path": path,
                "kind": "value",
                "off": lhs,
                "on": rhs,
            }
        )
    return differences


def validate_parity_field_capture(
    results: dict[str, tuple],
    artifact_root: Path,
) -> None:
    off_name = "timing_diagnostic_off_0_a"
    on_name = "timing_diagnostic_on_0_a"
    if tuple(results) != (off_name, on_name):
        raise AssertionError(
            f"parity field capture scenarios invalid: {tuple(results)}"
        )

    documents: dict[str, dict] = {}
    for scenario in (off_name, on_name):
        result = results[scenario]
        if (
            result[2] != 0
            or len(result[1]) != TIMING_DIAGNOSTIC_RECORD_COUNT
        ):
            raise AssertionError(
                f"{scenario}: parity field capture ring invalid "
                f"records={len(result[1])} drops={result[2]}"
            )
        span_result = result[17]
        if span_result is None:
            raise AssertionError(
                f"{scenario}: parity field capture spans unavailable"
            )
        spans, overflowed = span_result
        probe_enabled = "_on_" in scenario
        if overflowed or bool(spans) != probe_enabled:
            raise AssertionError(
                f"{scenario}: parity field recorder invalid "
                f"enabled={probe_enabled} overflow={overflowed} "
                f"records={len(spans)}"
            )

        document_path = artifact_root / scenario / "canonical_output.json"
        document = json.loads(
            document_path.read_text(),
            parse_constant=reject_nonstandard_json_constant,
        )
        signatures = dict(document.get("signature", ()))
        semantics = dict(document.get("canonical_semantic", ()))
        startup_alignment = document.get("startup_alignment")
        if (
            not isinstance(startup_alignment, dict)
            or startup_alignment.get("ledger_overflow") is not False
            or not isinstance(
                startup_alignment.get("anchor_stamp_ns"), int
            )
            or not isinstance(
                startup_alignment.get("anchor_sequence"), int
            )
        ):
            raise AssertionError(
                f"{scenario}: startup alignment artifact invalid"
            )
        stale_stop_evidence = startup_alignment.get(
            "stale_input_stop_evidence"
        )
        if (
            not isinstance(stale_stop_evidence, list)
            or not stale_stop_evidence
            or not all(
                isinstance(entry, dict)
                and entry.get("safe_stop") is True
                for entry in stale_stop_evidence
            )
        ):
            raise AssertionError(
                f"{scenario}: startup fail-closed ledger invalid"
            )
        if set(semantics) != {
            "command_envelope",
            "execution_envelope",
        }:
            raise AssertionError(
                f"{scenario}: canonical semantic channels invalid "
                f"{sorted(semantics)}"
            )
        for topic, values in semantics.items():
            hashes = signatures.get(topic, ())
            if (
                len(values) != CAPTURE_WINDOW_COUNT
                or len(hashes) != CAPTURE_WINDOW_COUNT
                or [
                    canonical_payload_hash(
                        restore_json_safe_artifact(value)
                    )
                    for value in values
                ]
                != hashes
            ):
                raise AssertionError(
                    f"{scenario}: canonical semantic rehash invalid "
                    f"topic={topic} values={len(values)} "
                    f"hashes={len(hashes)}"
                )
        documents[scenario] = document

    off_signatures = dict(documents[off_name]["signature"])
    on_signatures = dict(documents[on_name]["signature"])
    signatures_exact = off_signatures == on_signatures
    identity_alignment_valid = (
        signatures_equal_with_single_leading_sample_alignment(
            results[on_name][0],
            results[off_name][0],
            results[on_name][16],
            results[off_name][16],
        )
    )

    off_semantics = dict(documents[off_name]["canonical_semantic"])
    on_semantics = dict(documents[on_name]["canonical_semantic"])
    field_differences: dict[str, list[dict[str, object]]] = {}
    for topic in ("command_envelope", "execution_envelope"):
        topic_differences = []
        for ordinal, (off_value, on_value) in enumerate(
            zip(off_semantics[topic], on_semantics[topic])
        ):
            if (
                off_signatures[topic][ordinal]
                == on_signatures[topic][ordinal]
            ):
                continue
            differences = exact_field_differences(off_value, on_value)
            if differences:
                topic_differences.append(
                    {
                        "capture_ordinal": ordinal,
                        "difference_count": len(differences),
                        "differences": differences,
                    }
                )
        field_differences[topic] = topic_differences

    summary = {
        "diagnostic_only": True,
        "classification": (
            "FIRST_FALSE_NOT_REPRODUCED_INCONCLUSIVE_NO_RETRY"
            if signatures_exact and identity_alignment_valid
            else "PARITY_OR_IDENTITY_FIRST_FALSE_CAPTURED_NOT_ACCEPTANCE"
        ),
        "scenario_order": [off_name, on_name],
        "signatures_exact_same_ordinal": signatures_exact,
        "identity_alignment_valid": identity_alignment_valid,
        "alignment_report": signature_alignment_report(
            results[on_name][0],
            results[off_name][0],
        ),
        "field_differences": field_differences,
    }
    (artifact_root / "parity_field_capture_summary.json").write_text(
        strict_json_text(summary, indent=2, sort_keys=True) + "\n"
    )
    if not signatures_exact or not identity_alignment_valid:
        raise AssertionError(
            "parity field capture exact cohort mismatch "
            f"signatures_exact={signatures_exact} "
            f"identity_alignment_valid={identity_alignment_valid} "
            f"overlap={summary['alignment_report']}"
        )
    print(f"C002AY0_PP_PARITY_FIELD_CAPTURE={summary['classification']}")


def validate_callback_span_diagnostic(
    results: dict[str, tuple],
    artifact_root: Path,
) -> None:
    baseline_name = "timing_diagnostic_off_0_a"
    baseline = results[baseline_name]
    run_summaries: dict[str, dict[str, object]] = {}

    for scenario, result in results.items():
        if not signatures_equal_with_single_leading_sample_alignment(
            result[0], baseline[0], result[16], baseline[16]
        ):
            overlap = signature_alignment_report(result[0], baseline[0])
            raise AssertionError(
                f"{scenario}: timing diagnostic canonical output parity "
                f"mismatch overlap={overlap}"
            )
        timer_records = result[1]
        if result[2] != 0 or len(timer_records) != TIMING_DIAGNOSTIC_RECORD_COUNT:
            raise AssertionError(
                f"{scenario}: timing diagnostic ring invalid "
                f"records={len(timer_records)} drops={result[2]}"
            )
        expected_timer_sequence = timer_records[0][0]
        for sequence, _, duration_ns in timer_records:
            if sequence != expected_timer_sequence or duration_ns < 0:
                raise AssertionError(
                    f"{scenario}: timing diagnostic timer sequence invalid"
                )
            expected_timer_sequence += 1

        span_result = result[17]
        if span_result is None:
            raise AssertionError(
                f"{scenario}: timing diagnostic spans are unavailable"
            )
        spans, overflowed = span_result
        probe_enabled = "_on_" in scenario
        if overflowed or (probe_enabled and not spans) or (
            not probe_enabled and spans
        ):
            raise AssertionError(
                f"{scenario}: timing diagnostic recorder invalid "
                f"enabled={probe_enabled} overflow={overflowed} "
                f"records={len(spans)}"
            )
        expected_span_sequence = 1
        thread_ids = set()
        previous_exit_ns = 0
        for sequence, _, entry_ns, exit_ns, thread_id in spans:
            if sequence != expected_span_sequence:
                raise AssertionError(
                    f"{scenario}: timing diagnostic span sequence invalid"
                )
            if entry_ns > exit_ns or entry_ns < previous_exit_ns:
                raise AssertionError(
                    f"{scenario}: timing diagnostic spans overlap"
                )
            expected_span_sequence += 1
            previous_exit_ns = exit_ns
            thread_ids.add(thread_id)
        if probe_enabled and len(thread_ids) != 1:
            raise AssertionError(
                f"{scenario}: timing diagnostic callbacks changed thread"
            )

        gaps = [
            current[1] - previous[1]
            for previous, current in zip(timer_records, timer_records[1:])
        ]
        if not gaps or min(gaps) < 0:
            raise AssertionError(
                f"{scenario}: timing diagnostic timer clock invalid"
            )
        durations = [record[2] for record in timer_records]
        late_gaps = []
        if probe_enabled:
            for previous, current in zip(timer_records, timer_records[1:]):
                previous_start_ns = previous[1]
                current_start_ns = current[1]
                gap_ns = current_start_ns - previous_start_ns
                if gap_ns <= 12_000_000:
                    continue
                late_gaps.append(
                    {
                        "gap_ns": gap_ns,
                        "w10": callback_window_attribution(
                            spans,
                            previous_start_ns + 10_000_000,
                            current_start_ns,
                        ),
                        "w12": callback_window_attribution(
                            spans,
                            previous_start_ns + 12_000_000,
                            current_start_ns,
                        ),
                    }
                )
        run_summaries[scenario] = {
            "probe_enabled": probe_enabled,
            "record_count": len(timer_records),
            "span_count": len(spans),
            "overflow": overflowed,
            "gap_p99_ns": percentile_99(gaps),
            "gap_p999_ns": percentile_999(gaps),
            "gap_max_ns": max(gaps),
            "gap_over_12ms_count": sum(
                gap_ns > 12_000_000 for gap_ns in gaps
            ),
            "duration_p99_ns": percentile_99(durations),
            "duration_p999_ns": percentile_999(durations),
            "duration_max_ns": max(durations),
            "late_gap_attribution": late_gaps,
        }

    block_summaries = []
    p999_shifts_ns = []
    for block in range(TIMING_DIAGNOSTIC_BLOCKS):
        off_names = (
            f"timing_diagnostic_off_{block}_a",
            f"timing_diagnostic_off_{block}_b",
        )
        on_names = (
            f"timing_diagnostic_on_{block}_a",
            f"timing_diagnostic_on_{block}_b",
        )
        off_p999_sum = sum(
            int(run_summaries[name]["gap_p999_ns"]) for name in off_names
        )
        on_p999_sum = sum(
            int(run_summaries[name]["gap_p999_ns"]) for name in on_names
        )
        p999_shift_ns = (on_p999_sum - off_p999_sum) // 2
        p999_shifts_ns.append(p999_shift_ns)
        block_summaries.append(
            {
                "block": block,
                "order": [off_names[0], on_names[0], on_names[1], off_names[1]],
                "p999_shift_on_minus_off_ns": p999_shift_ns,
            }
        )

    upper_tail_shift_ns = max(0, *p999_shifts_ns)
    unprobed_reproduction = any(
        not bool(summary["probe_enabled"])
        and int(summary["gap_over_12ms_count"]) > 0
        for summary in run_summaries.values()
    )
    if unprobed_reproduction:
        classification = "UNPROBED_REPRODUCTION_NOT_ACCEPTANCE"
    elif upper_tail_shift_ns >= TIMING_DIAGNOSTIC_PROBE_MARGIN_NS:
        classification = "PROBE_SENSITIVE_CANDIDATE_NOT_ACCEPTANCE"
    else:
        classification = "INCONCLUSIVE_NOT_ACCEPTANCE"
    campaign_summary = {
        "classification": classification,
        "diagnostic_only": True,
        "record_count_per_run": TIMING_DIAGNOSTIC_RECORD_COUNT,
        "probe_margin_ns": TIMING_DIAGNOSTIC_PROBE_MARGIN_NS,
        "upper_tail_shift_ns": upper_tail_shift_ns,
        "unprobed_reproduction": unprobed_reproduction,
        "blocks": block_summaries,
        "runs": run_summaries,
    }
    (artifact_root / "campaign_summary.json").write_text(
        json.dumps(campaign_summary, indent=2, sort_keys=True) + "\n"
    )
    print(
        "C002AY0_PP_PROBE_SENSITIVITY_DIAGNOSTIC="
        + json.dumps(campaign_summary, sort_keys=True)
    )
    print(f"C002AY0_PP_PROBE_SENSITIVITY_DIAGNOSTIC={classification}")


def validate_timing_attribution_only(
    results: dict[str, tuple],
    artifact_root: Path,
    baseline_path: Path,
) -> None:
    scenario = "timing_diagnostic_attribution_on_0"
    if tuple(results) != (scenario,):
        raise AssertionError(
            f"attribution-only scenarios invalid: {tuple(results)}"
        )
    result = results[scenario]
    timer_records = result[1]
    if result[2] != 0 or len(timer_records) != TIMING_ATTRIBUTION_RECORD_COUNT:
        raise AssertionError(
            f"{scenario}: attribution timer ring invalid "
            f"records={len(timer_records)} drops={result[2]}"
        )
    expected_timer_sequence = timer_records[0][0]
    for sequence, _, duration_ns in timer_records:
        if sequence != expected_timer_sequence or duration_ns < 0:
            raise AssertionError(
                f"{scenario}: attribution timer sequence invalid"
            )
        expected_timer_sequence += 1

    span_result = result[17]
    if span_result is None:
        raise AssertionError(f"{scenario}: attribution spans unavailable")
    spans, overflowed = span_result
    if overflowed or not spans:
        raise AssertionError(
            f"{scenario}: attribution span recorder invalid "
            f"overflow={overflowed} records={len(spans)}"
        )
    expected_span_sequence = 1
    previous_exit_ns = 0
    thread_ids = set()
    for sequence, _, entry_ns, exit_ns, thread_id in spans:
        if sequence != expected_span_sequence:
            raise AssertionError(
                f"{scenario}: attribution span sequence invalid"
            )
        if entry_ns > exit_ns or entry_ns < previous_exit_ns:
            raise AssertionError(f"{scenario}: attribution spans overlap")
        expected_span_sequence += 1
        previous_exit_ns = exit_ns
        thread_ids.add(thread_id)
    if len(thread_ids) != 1:
        raise AssertionError(
            f"{scenario}: attribution callbacks changed thread"
        )

    current_path = artifact_root / scenario / "canonical_output.json"
    current_document = json.loads(
        current_path.read_text(),
        parse_constant=reject_nonstandard_json_constant,
    )
    baseline_document = json.loads(
        baseline_path.read_text(),
        parse_constant=reject_nonstandard_json_constant,
    )
    if current_document.get("signature") != baseline_document.get("signature"):
        raise AssertionError(
            f"{scenario}: frozen common output signature mismatch"
        )
    signatures = dict(current_document.get("signature", ()))
    semantics = dict(current_document.get("canonical_semantic", ()))
    if set(semantics) != {"command_envelope", "execution_envelope"}:
        raise AssertionError(
            f"{scenario}: canonical semantic channels invalid"
        )
    for topic, values in semantics.items():
        hashes = signatures.get(topic, ())
        if (
            len(values) != CAPTURE_WINDOW_COUNT
            or len(hashes) != CAPTURE_WINDOW_COUNT
            or [
                canonical_payload_hash(restore_json_safe_artifact(value))
                for value in values
            ]
            != hashes
        ):
            raise AssertionError(
                f"{scenario}: canonical semantic rehash invalid "
                f"topic={topic}"
            )

    gaps = [
        current[1] - previous[1]
        for previous, current in zip(timer_records, timer_records[1:])
    ]
    if not gaps or min(gaps) < 0:
        raise AssertionError(f"{scenario}: attribution timer clock invalid")
    late_gaps = []
    late_gap_classes = []
    for index, (previous, current) in enumerate(
        zip(timer_records, timer_records[1:])
    ):
        gap_ns = current[1] - previous[1]
        if gap_ns <= 12_000_000:
            continue
        w10 = callback_window_attribution(
            spans, previous[1] + 10_000_000, current[1]
        )
        w12 = callback_window_attribution(
            spans, previous[1] + 12_000_000, current[1]
        )
        if int(w10["unattributed_residual_ns"]) == 0:
            gap_class = "KNOWN_SUBSCRIPTION_OCCUPANCY_REPRODUCED"
        elif int(w12["known_busy_ns"]) == 0:
            gap_class = "UNATTRIBUTED_EXECUTOR_RMW_OS_REPRODUCED"
        else:
            gap_class = "MIXED_PRE_ENTRY_DELAY"
        late_gap_classes.append(gap_class)
        late_gaps.append(
            {
                "previous_sequence": previous[0],
                "current_sequence": current[0],
                "gap_ns": gap_ns,
                "gap_excess_over_12ms_ns": gap_ns - 12_000_000,
                "due_lateness_ns": gap_ns - 10_000_000,
                "post_previous_timer_residual_ns": (
                    current[1] - (previous[1] + previous[2])
                ),
                "two_period_phase_error_ns": (
                    timer_records[index + 2][1]
                    - (previous[1] + 20_000_000)
                    if index + 2 < len(timer_records)
                    else None
                ),
                "timer_overlap_at_due_ns": max(
                    0, previous[2] - 10_000_000
                ),
                "w10": w10,
                "w12": w12,
                "classification": gap_class,
            }
        )
    if not late_gap_classes:
        classification = "NO_LATE_GAP_INCONCLUSIVE_NO_RETRY"
    elif "UNATTRIBUTED_EXECUTOR_RMW_OS_REPRODUCED" in late_gap_classes:
        classification = "UNATTRIBUTED_EXECUTOR_RMW_OS_REPRODUCED"
    elif "MIXED_PRE_ENTRY_DELAY" in late_gap_classes:
        classification = "MIXED_PRE_ENTRY_DELAY"
    else:
        classification = "KNOWN_SUBSCRIPTION_OCCUPANCY_REPRODUCED"

    summary = {
        "classification": classification,
        "diagnostic_only": True,
        "not_acceptance": True,
        "no_retry": True,
        "record_count": len(timer_records),
        "drop_count": result[2],
        "span_count": len(spans),
        "span_overflow": overflowed,
        "single_callback_thread": len(thread_ids) == 1,
        "frozen_baseline_path": str(baseline_path),
        "frozen_baseline_sha256": sha256_file(baseline_path),
        "gap_p99_ns": percentile_99(gaps),
        "gap_p999_ns": percentile_999(gaps),
        "gap_max_ns": max(gaps),
        "gap_over_12ms_count": len(late_gaps),
        "late_gap_attribution": late_gaps,
    }
    summary_path = artifact_root / "attribution_only_summary.json"
    summary_path.write_text(
        strict_json_text(summary, indent=2, sort_keys=True) + "\n"
    )
    json.loads(
        summary_path.read_text(),
        parse_constant=reject_nonstandard_json_constant,
    )
    print(f"C002AY0_PP_TIMING_ATTRIBUTION_ONLY={classification}")


def percentile_999(values: list[int]) -> int:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, len(ordered) * 999 // 1000)]


def percentile_99(values: list[int]) -> int:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, len(ordered) * 99 // 100)]


def percentile_90(values: list[int]) -> int:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, len(ordered) * 90 // 100)]


def signatures_equal_with_bounded_alignment(lhs: tuple, rhs: tuple) -> bool:
    """Require 16 contiguous equal samples despite subscriber start skew."""
    lhs_by_topic = dict(lhs)
    rhs_by_topic = dict(rhs)
    if lhs_by_topic.keys() != rhs_by_topic.keys():
        return False
    for topic, lhs_values in lhs_by_topic.items():
        rhs_values = rhs_by_topic[topic]
        if lhs_values == rhs_values:
            continue
        aligned = False
        for lhs_start in range(len(lhs_values) - 15):
            for rhs_start in range(len(rhs_values) - 15):
                if (
                    lhs_values[lhs_start : lhs_start + 16]
                    == rhs_values[rhs_start : rhs_start + 16]
                ):
                    aligned = True
                    break
            if aligned:
                break
        if not aligned:
            return False
    return True


def signature_alignment_report(lhs: tuple, rhs: tuple) -> dict[str, int]:
    """Return the longest exact contiguous overlap for each output topic."""
    report = {}
    lhs_by_topic = dict(lhs)
    rhs_by_topic = dict(rhs)
    for topic, lhs_values in lhs_by_topic.items():
        rhs_values = rhs_by_topic.get(topic, ())
        best = 0
        for lhs_start in range(len(lhs_values)):
            for rhs_start in range(len(rhs_values)):
                overlap = 0
                while (
                    lhs_start + overlap < len(lhs_values)
                    and rhs_start + overlap < len(rhs_values)
                    and lhs_values[lhs_start + overlap]
                    == rhs_values[rhs_start + overlap]
                ):
                    overlap += 1
                best = max(best, overlap)
        report[topic] = best
    return report


def signatures_equal_with_single_leading_sample_alignment(
    lhs: tuple,
    rhs: tuple,
    lhs_identities: tuple,
    rhs_identities: tuple,
) -> bool:
    """Require 32 exact consecutive samples after at most one leading skew."""
    lhs_by_topic = dict(lhs)
    rhs_by_topic = dict(rhs)
    lhs_identity_by_topic = dict(lhs_identities)
    rhs_identity_by_topic = dict(rhs_identities)
    if (
        lhs_by_topic.keys() != rhs_by_topic.keys()
        or lhs_by_topic.keys() != lhs_identity_by_topic.keys()
        or lhs_by_topic.keys() != rhs_identity_by_topic.keys()
    ):
        return False

    allowed_offsets = ((0, 0), (1, 0), (0, 1))
    for topic, lhs_values in lhs_by_topic.items():
        rhs_values = rhs_by_topic[topic]
        lhs_ids = lhs_identity_by_topic[topic]
        rhs_ids = rhs_identity_by_topic[topic]
        if any(
            len(values) != CAPTURE_WINDOW_COUNT
            for values in (lhs_values, rhs_values, lhs_ids, rhs_ids)
        ):
            return False

        def stream_valid(identities: tuple) -> bool:
            stamps = [identity[0] for identity in identities]
            if any(
                current < previous
                for previous, current in zip(stamps, stamps[1:])
            ):
                return False
            sequences = [identity[2] for identity in identities]
            if any(sequences):
                if any(sequence <= 0 for sequence in sequences):
                    return False
                if any(
                    current != previous + 1
                    for previous, current in zip(
                        sequences, sequences[1:]
                    )
                ):
                    return False
                producers = {identity[1] for identity in identities}
                if len(producers) != 1 or next(iter(producers)) <= 0:
                    return False
            return True

        if not stream_valid(lhs_ids) or not stream_valid(rhs_ids):
            return False

        matched = False
        for lhs_start, rhs_start in allowed_offsets:
            lhs_window = lhs_values[
                lhs_start : lhs_start + CAPTURE_COUNT
            ]
            rhs_window = rhs_values[
                rhs_start : rhs_start + CAPTURE_COUNT
            ]
            if lhs_window != rhs_window:
                continue
            lhs_identity_window = lhs_ids[
                lhs_start : lhs_start + CAPTURE_COUNT
            ]
            rhs_identity_window = rhs_ids[
                rhs_start : rhs_start + CAPTURE_COUNT
            ]
            # Absolute producer IDs, sequence origins, and stamps differ
            # between isolated PP processes.  Safety-relevant generation,
            # plan-key, source-generation, and frame identity must not.
            if any(
                lhs_identity[3:] != rhs_identity[3:]
                for lhs_identity, rhs_identity in zip(
                    lhs_identity_window, rhs_identity_window
                )
            ):
                continue
            lhs_sequences = [
                identity[2] for identity in lhs_identity_window
            ]
            rhs_sequences = [
                identity[2] for identity in rhs_identity_window
            ]
            if any(lhs_sequences) and (
                [
                    sequence - lhs_sequences[0]
                    for sequence in lhs_sequences
                ]
                != [
                    sequence - rhs_sequences[0]
                    for sequence in rhs_sequences
                ]
            ):
                continue
            if lhs_start == 1 and (
                lhs_ids[0][0] >= lhs_ids[1][0]
                or (
                    lhs_ids[1][2] > 0
                    and lhs_ids[0][2] + 1 != lhs_ids[1][2]
                )
            ):
                continue
            if rhs_start == 1 and (
                rhs_ids[0][0] >= rhs_ids[1][0]
                or (
                    rhs_ids[1][2] > 0
                    and rhs_ids[0][2] + 1 != rhs_ids[1][2]
                )
            ):
                continue
            matched = True
            break
        if not matched:
            return False
    return True


def validate_snapshot_worker_wcet(results: dict[str, tuple]) -> None:
    baseline = results["off"][0]
    for scenario in ("healthy_a", "healthy_b"):
        if not signatures_equal_with_bounded_alignment(
            results[scenario][0], baseline
        ):
            overlap = signature_alignment_report(
                results[scenario][0], baseline
            )
            raise AssertionError(
                f"{scenario}: PP command parity mismatch overlap={overlap}"
            )
        records = results[scenario][1]
        drops = results[scenario][2]
        if len(records) < 1000:
            raise AssertionError(
                f"{scenario}: too few callback records ({len(records)})"
            )
        durations = [record[2] for record in records]
        starts = [record[1] for record in records]
        gaps = [
            current - previous
            for previous, current in zip(starts, starts[1:])
            if current >= previous
        ]
        duration_p999 = percentile_999(durations)
        gap_max = max(gaps)
        print(
            f"{scenario} samples={len(records)} "
            f"callback_p99_ns={percentile_99(durations)} "
            f"callback_p999_ns={duration_p999} "
            f"callback_max_ns={max(durations)} "
            f"timer_gap_max_ns={gap_max} observer_drops={drops}"
        )
        if duration_p999 > 1_000_000:
            raise AssertionError(f"{scenario}: callback p99.9 exceeds 1 ms")
        if gap_max > 12_000_000:
            raise AssertionError(f"{scenario}: timer gap exceeds 12 ms")
        if drops != 0:
            raise AssertionError(f"{scenario}: observer dropped records")
        if results[scenario][5] == 0:
            raise AssertionError(f"{scenario}: no shadow snapshot published")
        if results[scenario][5] > len(records) // 4 + 2:
            raise AssertionError(
                f"{scenario}: shadow publish cadence exceeded 20 Hz"
            )
        if results[scenario][6]:
            raise AssertionError(f"{scenario}: invalid shadow authority")
    print("C002AY0_SNAPSHOT_WORKER_WCET=PASS")


def validate_binding_e2e_wcet(results: dict[str, tuple]) -> None:
    baseline = results["off"][0]
    failures = []
    stretch_failures = []
    legacy_tail_failures = []
    production_observer_coverage_warnings = []
    for scenario in ("binding_on_a", "binding_on_b"):
        if not signatures_equal_with_bounded_alignment(
            results[scenario][0], baseline
        ):
            overlap = signature_alignment_report(
                results[scenario][0], baseline
            )
            for topic in ("tracking", "command_envelope", "execution_envelope"):
                baseline_debug = results["off"][14].get(topic, {})
                scenario_debug = results[scenario][14].get(topic, {})
                differing = sorted(
                    key
                    for key in set(baseline_debug) | set(scenario_debug)
                    if baseline_debug.get(key) != scenario_debug.get(key)
                )
                print(f"{scenario} {topic} differing_fields={differing}")
            failures.append(
                f"{scenario}: PP command parity mismatch overlap={overlap}"
            )
        records = results[scenario][1]
        drops = results[scenario][2]
        if len(records) < 1000:
            raise AssertionError(
                f"{scenario}: too few callback records ({len(records)})"
            )
        durations = [record[2] for record in records]
        starts = [record[1] for record in records]
        gaps = [
            current - previous
            for previous, current in zip(starts, starts[1:])
            if current >= previous
        ]
        duration_p999 = percentile_999(durations)
        gap_max = max(gaps)
        e2e_summary = results[scenario][15]
        print(
            f"{scenario} samples={len(records)} "
            f"callback_p90_ns={percentile_90(durations)} "
            f"callback_p99_ns={percentile_99(durations)} "
            f"callback_p999_ns={duration_p999} "
            f"callback_max_ns={max(durations)} "
            f"timer_gap_max_ns={gap_max} observer_drops={drops} "
            f"snapshots={results[scenario][5]} "
            f"proposals={results[scenario][9]} "
            f"bindings={results[scenario][10]} "
            f"exact_bindings={results[scenario][11]} "
            f"dispositions={results[scenario][13]} "
            f"legacy_p999_le_1ms={duration_p999 <= 1_000_000} "
            f"e2e_terminal={json.dumps(e2e_summary, sort_keys=True)}"
        )
        if percentile_99(durations) > 1_000_000:
            failures.append(f"{scenario}: callback p99 exceeds 1 ms")
        if duration_p999 > 2_000_000:
            failures.append(f"{scenario}: callback p99.9 exceeds 2 ms")
        if gap_max > 12_000_000:
            stretch_failures.append(f"{scenario}: timer gap exceeds 12 ms")
        if duration_p999 > 1_000_000:
            legacy_tail_failures.append(
                f"{scenario}: legacy callback p99.9 exceeds 1 ms"
            )
        if drops != 0:
            failures.append(f"{scenario}: observer dropped records")
        if results[scenario][5] == 0:
            failures.append(f"{scenario}: no base snapshot published")
        if results[scenario][9] == 0:
            production_observer_coverage_warnings.append(
                f"{scenario}: proposal observer empty"
            )
        if results[scenario][10] == 0:
            production_observer_coverage_warnings.append(
                f"{scenario}: binding observer empty"
            )
        if not e2e_summary:
            failures.append(f"{scenario}: terminal audit missing")
        else:
            if e2e_summary.get("expected_race_arm_epoch", 0) <= 1:
                failures.append(f"{scenario}: fenced race epoch missing")
            if e2e_summary.get("binding_callback_entries", 0) == 0:
                failures.append(f"{scenario}: binding callback cohort empty")
            if e2e_summary.get("binding_callback_entries") != e2e_summary.get(
                "terminal_bindings"
            ):
                failures.append(
                    f"{scenario}: binding callback cohort did not terminalize"
                )
            proposal_uptake = e2e_summary.get(
                "proposal_uptake_contract_v1"
            )
            if not isinstance(proposal_uptake, dict):
                failures.append(
                    f"{scenario}: proposal uptake contract missing"
                )
            elif not proposal_uptake.get(
                "all_proposals_taken_up_once_before_deadline", False
            ):
                failures.append(
                    f"{scenario}: proposal uptake incomplete "
                    f"accepted={proposal_uptake.get('accepted_first_uptake_count')} "
                    f"expected={proposal_uptake.get('expected_proposal_count')} "
                    f"failed={proposal_uptake.get('failed_or_missing_count')}"
                )
            if e2e_summary.get("duplicate_bindings", 0) != 0:
                failures.append(
                    f"{scenario}: duplicate binding callback audit="
                    f"{e2e_summary.get('duplicate_bindings')}"
                )
            for field in (
                "duplicate_proposals",
                "out_of_cohort_proposals",
                "out_of_cohort_bindings",
                "out_of_cohort_binding_audits",
            ):
                if e2e_summary.get(field, 0) != 0:
                    failures.append(
                        f"{scenario}: {field}={e2e_summary.get(field)}"
                    )
            if e2e_summary.get("safety_margin_at_bind_min_ns", 0) <= 0:
                failures.append(
                    f"{scenario}: exact binding lacks positive safety margin"
                )
            for field in (
                "fixture_only_identities",
                "binding_observer_without_audit",
            ):
                if e2e_summary.get(field, 0) != 0:
                    failures.append(
                        f"{scenario}: {field}={e2e_summary.get(field)}"
                    )
            if e2e_summary.get("fixture_proposal_seen", 0) == 0:
                production_observer_coverage_warnings.append(
                    f"{scenario}: proposal observer cohort empty"
                )
            if e2e_summary.get("binding_only_identities", 0) != 0:
                production_observer_coverage_warnings.append(
                    f"{scenario}: binding_only_identities="
                    f"{e2e_summary.get('binding_only_identities')}"
                )
            if e2e_summary.get("binding_observer_missing", 0) != 0:
                production_observer_coverage_warnings.append(
                    f"{scenario}: binding_observer_missing="
                    f"{e2e_summary.get('binding_observer_missing')}"
                )
            terminal_provenance = e2e_summary.get("terminal_provenance")
            if (
                not isinstance(terminal_provenance, list)
                or len(terminal_provenance)
                != e2e_summary.get("terminal_bindings")
                or any(
                    not isinstance(terminal, dict)
                    or terminal.get("out_of_cohort") is not False
                    for terminal in terminal_provenance
                )
            ):
                failures.append(
                    f"{scenario}: terminal audit is incomplete or out-of-cohort"
                )
        if results[scenario][5] > len(records) // 4 + 2:
            failures.append(
                f"{scenario}: shadow publish cadence exceeded 20 Hz"
            )
        if results[scenario][6]:
            failures.append(f"{scenario}: invalid shadow authority")
    if results["off"][10] != 0:
        failures.append("binding OFF published a binding result")
    if failures:
        raise AssertionError("; ".join(failures))
    if stretch_failures:
        print(
            "C002AY0_PP_BINDING_E2E_STRETCH=FAIL "
            + "; ".join(stretch_failures)
        )
    else:
        print("C002AY0_PP_BINDING_E2E_STRETCH=PASS")
    if legacy_tail_failures:
        print(
            "C002AY0_PP_BINDING_E2E_LEGACY_TAIL=FAIL "
            + "; ".join(legacy_tail_failures)
        )
    else:
        print("C002AY0_PP_BINDING_E2E_LEGACY_TAIL=PASS")
    if production_observer_coverage_warnings:
        print(
            "C002AY0_PP_BINDING_PRODUCTION_OBSERVER_COVERAGE=FAIL "
            + "; ".join(production_observer_coverage_warnings)
        )
        print("C002AY0_PP_BINDING_FULL_DELIVERY_E2E=FAIL")
    else:
        print("C002AY0_PP_BINDING_PRODUCTION_OBSERVER_COVERAGE=PASS")
        print("C002AY0_PP_BINDING_FULL_DELIVERY_E2E=PASS")
    print(
        "C002AY0_PP_BINDING_OBSERVED_TERMINAL_AUDIT_INTEGRITY=PASS"
    )
    print("C002AY0_PP_BINDING_CALLBACK_COHORT_WCET=PASS")


def create_measurement_node() -> object:
    return rclpy.create_node(
        "c002ay0_pp_runtime_measurement",
        parameter_overrides=[
            Parameter("use_sim_time", value=True),
        ],
    )


def main() -> int:
    rclpy.init()
    node = create_measurement_node()
    results = {}
    snapshot_worker_wcet_only = (
        os.environ.get("C002AY0_SNAPSHOT_WORKER_WCET_ONLY") == "1"
    )
    v2_uptake_only = os.environ.get("C002AY0_V2_UPTAKE_ONLY") == "1"
    v2_uptake_artifact_parent_text = os.environ.get(
        "C002AY0_V2_UPTAKE_ARTIFACT_PARENT", ""
    )
    real_state_lattice_e2e = (
        os.environ.get("C002AY0_REAL_STATE_LATTICE_E2E_WCET_ONLY") == "1"
    )
    binding_e2e_wcet_only = (
        os.environ.get("C002AY0_BINDING_E2E_WCET_ONLY") == "1"
        or real_state_lattice_e2e
    )
    timing_diagnostic_fixture_text = os.environ.get(
        "C002AY0_PP_TIMING_DIAGNOSTIC_FIXTURE", ""
    )
    timing_diagnostic_fixture = (
        Path(timing_diagnostic_fixture_text)
        if timing_diagnostic_fixture_text
        else None
    )
    timing_diagnostic_parity_only = (
        os.environ.get("C002AY0_PP_TIMING_DIAGNOSTIC_PARITY_ONLY") == "1"
    )
    timing_attribution_only = (
        os.environ.get("C002AY0_M4_ATTRIBUTION_ONLY") == "1"
    )
    timing_attribution_baseline_text = os.environ.get(
        "C002AY0_M4_ATTRIBUTION_BASELINE", ""
    )
    timing_attribution_baseline = (
        Path(timing_attribution_baseline_text)
        if timing_attribution_baseline_text
        else None
    )
    timing_diagnostic_artifact_parent_text = os.environ.get(
        "C002AY0_PP_TIMING_DIAGNOSTIC_ARTIFACT_PARENT", ""
    )
    proposal_relay_path_text = os.environ.get(
        "C002AY0_EXACT_PROPOSAL_RELAY", ""
    )
    proposal_relay_path = (
        Path(proposal_relay_path_text)
        if proposal_relay_path_text
        else None
    )
    try:
        if v2_uptake_only and (
            snapshot_worker_wcet_only
            or real_state_lattice_e2e
            or timing_diagnostic_fixture is not None
            or timing_diagnostic_parity_only
            or timing_attribution_only
        ):
            raise AssertionError("V2 uptake fixture conflicts with another mode")
        if v2_uptake_only and not v2_uptake_artifact_parent_text:
            raise AssertionError("V2 uptake artifact parent is required")
        if timing_diagnostic_parity_only and timing_attribution_only:
            raise AssertionError(
                "timing parity-only and attribution-only modes conflict"
            )
        if timing_attribution_only and (
            timing_diagnostic_fixture is None
            or timing_attribution_baseline is None
        ):
            raise AssertionError(
                "attribution-only requires timing fixture and baseline"
            )
        if v2_uptake_only:
            artifact_parent = Path(v2_uptake_artifact_parent_text)
            artifact_parent.mkdir(parents=True, exist_ok=True)
            temp_context = nullcontext(
                tempfile.mkdtemp(
                    prefix="c002ay0-v2-uptake-",
                    dir=artifact_parent,
                )
            )
        elif timing_diagnostic_fixture is not None:
            if not timing_diagnostic_artifact_parent_text:
                raise AssertionError(
                    "timing diagnostic artifact parent is required"
                )
            artifact_parent = Path(timing_diagnostic_artifact_parent_text)
            artifact_parent.mkdir(parents=True, exist_ok=True)
            temp_context = nullcontext(
                tempfile.mkdtemp(
                    prefix="c002ay0-pp-timing-diagnostic-",
                    dir=artifact_parent,
                )
            )
        else:
            temp_context = tempfile.TemporaryDirectory(
                prefix="c002ay0-pp-runtime-"
            )
        with temp_context as temp:
            root = Path(temp)
            if v2_uptake_only:
                print(f"C002AY0_V2_UPTAKE_ARTIFACT={root}")
                (root / "fixture_manifest.json").write_text(
                    json.dumps(
                        {
                            "classification": "fixture_only",
                            "scenario_order": ["v2_uptake"],
                            "planner_live_control_output": False,
                            "planner_instant_control": False,
                            "pp_accepted_payload": "structurally_discarded",
                            "planner_producer_instance_id": (
                                V2_UPTAKE_PLANNER_PRODUCER_INSTANCE_ID
                            ),
                            "pp_attestation_producer_instance_id": (
                                V2_UPTAKE_PP_ATTESTATION_PRODUCER_INSTANCE_ID
                            ),
                            "session_id_and_race_epoch": V2_UPTAKE_SESSION_ID,
                            "harness_sha256": sha256_file(Path(__file__)),
                        },
                        indent=2,
                        sort_keys=True,
                    )
                    + "\n"
                )
            if timing_diagnostic_fixture is not None:
                print(f"C002AY0_PP_TIMING_DIAGNOSTIC_ARTIFACT={root}")
                timing_diagnostic_scenarios = (
                    ("timing_diagnostic_attribution_on_0",)
                    if timing_attribution_only
                    else
                    (
                        "timing_diagnostic_off_0_a",
                        "timing_diagnostic_on_0_a",
                    )
                    if timing_diagnostic_parity_only
                    else tuple(
                        scenario
                        for block in range(TIMING_DIAGNOSTIC_BLOCKS)
                        for scenario in (
                            f"timing_diagnostic_off_{block}_a",
                            f"timing_diagnostic_on_{block}_a",
                            f"timing_diagnostic_on_{block}_b",
                            f"timing_diagnostic_off_{block}_b",
                        )
                    )
                )
                production_pp = (
                    Path(get_package_prefix("simple_pure_pursuit"))
                    / "lib/simple_pure_pursuit/simple_pure_pursuit"
                )
                manifest = {
                    "diagnostic_only": True,
                    "attribution_only": timing_attribution_only,
                    "parity_field_capture_only": (
                        timing_diagnostic_parity_only
                    ),
                    "started_unix_ns": time.time_ns(),
                    "scenario_order": timing_diagnostic_scenarios,
                    "blocks": TIMING_DIAGNOSTIC_BLOCKS,
                    "record_count_per_run": (
                        TIMING_ATTRIBUTION_RECORD_COUNT
                        if timing_attribution_only
                        else TIMING_DIAGNOSTIC_RECORD_COUNT
                    ),
                    "probe_margin_ns": (
                        TIMING_DIAGNOSTIC_PROBE_MARGIN_NS
                    ),
                    "fixture_path": str(timing_diagnostic_fixture),
                    "fixture_sha256": sha256_file(
                        timing_diagnostic_fixture
                    ),
                    "production_pp_path": str(production_pp),
                    "production_pp_sha256": sha256_file(production_pp),
                    "harness_sha256": sha256_file(Path(__file__)),
                    "frozen_baseline_path": (
                        str(timing_attribution_baseline)
                        if timing_attribution_baseline is not None
                        else None
                    ),
                    "frozen_baseline_sha256": (
                        sha256_file(timing_attribution_baseline)
                        if timing_attribution_baseline is not None
                        else None
                    ),
                    "cpu_affinity": sorted(os.sched_getaffinity(0)),
                    "environment": {
                        key: os.environ.get(key, "")
                        for key in (
                            "ROS_DOMAIN_ID",
                            "RMW_IMPLEMENTATION",
                            "ROS_LOCALHOST_ONLY",
                            "CYCLONEDDS_URI",
                            "ROS_AUTOMATIC_DISCOVERY_RANGE",
                        )
                    },
                    "hostname": os.uname().nodename,
                }
                (root / "campaign_manifest.json").write_text(
                    json.dumps(manifest, indent=2, sort_keys=True) + "\n"
                )
            else:
                timing_diagnostic_scenarios = ()
            scenarios = (
                timing_diagnostic_scenarios
                if timing_diagnostic_fixture is not None
                else
                ("v2_uptake",)
                if v2_uptake_only
                else
                ("off", "binding_on_a", "binding_on_b")
                if binding_e2e_wcet_only
                else ("off", "healthy_a", "healthy_b")
                if snapshot_worker_wcet_only
                else (
                    "off",
                    "healthy_a",
                    "unavailable",
                    "full",
                    "hang",
                    "crash",
                    "restart",
                    "healthy_b",
                    "hard_clamp_off",
                    "hard_clamp_on",
                    "free_run_live_off",
                    "free_run_live_on",
                )
            )
            for index, scenario in enumerate(
                scenarios
            ):
                scenario_root = root / scenario
                scenario_root.mkdir()
                if scenario in (
                    "healthy_a",
                    "healthy_b",
                    "restart",
                    "binding_on_a",
                    "binding_on_b",
                ):
                    effective = "healthy"
                elif scenario.startswith("hard_clamp"):
                    effective = (
                        "off" if scenario.endswith("_off") else "healthy"
                    )
                elif scenario.startswith("free_run_live"):
                    effective = "off"
                elif scenario.startswith("timing_diagnostic"):
                    effective = "off"
                else:
                    effective = scenario
                results[scenario] = run_case(
                    node,
                    scenario_root,
                    effective,
                    0xB000 + index * 2,
                    curved=scenario.startswith("hard_clamp"),
                    free_run_live=(
                        scenario == "free_run_live_on"
                        or scenario.startswith("timing_diagnostic")
                    ),
                    measure_long=(
                        scenario.startswith("free_run_live")
                        or scenario.startswith("binding_on")
                        or scenario.startswith("timing_diagnostic")
                    ),
                    binding_enabled=scenario.startswith("binding_on"),
                    proposal_relay_path=proposal_relay_path,
                    warmup_sequence=100 if binding_e2e_wcet_only else 20,
                    real_state_lattice=(
                        real_state_lattice_e2e or v2_uptake_only
                    ),
                    pp_executable_override=(
                        timing_diagnostic_fixture
                        if scenario.startswith("timing_diagnostic")
                        else None
                    ),
                    callback_span_path=(
                        scenario_root / "callback_spans.txt"
                        if scenario.startswith("timing_diagnostic")
                        else None
                    ),
                    callback_span_enabled=(
                        "_on_" in scenario
                        if scenario.startswith("timing_diagnostic")
                        else None
                    ),
                    runtime_socket_path=(
                        Path(
                            f"/tmp/ay0r-{os.getpid()}-"
                            f"{0xB000 + index * 2:x}.sock"
                        )
                        if scenario.startswith("timing_diagnostic")
                        else None
                    ),
                    timing_diagnostic=scenario.startswith(
                        "timing_diagnostic"
                    ),
                    timing_record_count=(
                        TIMING_ATTRIBUTION_RECORD_COUNT
                        if timing_attribution_only
                        else TIMING_DIAGNOSTIC_RECORD_COUNT
                    ),
                    v2_uptake_only=v2_uptake_only,
                )
        if v2_uptake_only:
            summary_path = root / "v2_uptake" / "v2_uptake_summary.json"
            if not summary_path.exists():
                raise AssertionError("V2 uptake summary missing")
            summary = json.loads(summary_path.read_text())
            if (
                summary.get("classification")
                != "PP_UPTAKE_AVAILABILITY_VERIFIED_NON_AUTHORITATIVE"
            ):
                raise AssertionError("V2 uptake summary classification invalid")
            print("C002AY0_V2_UPTAKE_FIXTURE=COMPLETE")
            return 0
        if timing_diagnostic_fixture is not None:
            if timing_attribution_only:
                validate_timing_attribution_only(
                    results,
                    root,
                    timing_attribution_baseline,
                )
                return 0
            if timing_diagnostic_parity_only:
                validate_parity_field_capture(results, root)
                return 0
            validate_callback_span_diagnostic(
                results,
                root,
            )
            return 0
        if binding_e2e_wcet_only:
            validate_binding_e2e_wcet(results)
            return 0
        if snapshot_worker_wcet_only:
            validate_snapshot_worker_wcet(results)
            return 0
        baseline = results["off"][0]
        for scenario, result in results.items():
            signature = result[0]
            if scenario.startswith("hard_clamp"):
                continue
            if not signatures_equal_with_single_leading_sample_alignment(
                signature,
                baseline,
                result[16],
                results["off"][16],
            ):
                overlap = signature_alignment_report(signature, baseline)
                raise AssertionError(
                    f"{scenario}: PP command parity mismatch "
                    f"overlap={overlap}"
                )
        if not signatures_equal_with_bounded_alignment(
            results["hard_clamp_on"][0], results["hard_clamp_off"][0]
        ):
            overlap = signature_alignment_report(
                results["hard_clamp_on"][0],
                results["hard_clamp_off"][0],
            )
            raise AssertionError(
                f"hard clamp: PP command parity mismatch overlap={overlap}"
            )
        if not signatures_equal_with_single_leading_sample_alignment(
            results["free_run_live_on"][0],
            results["free_run_live_off"][0],
            results["free_run_live_on"][16],
            results["free_run_live_off"][16],
        ):
            overlap = signature_alignment_report(
                results["free_run_live_on"][0],
                results["free_run_live_off"][0],
            )
            raise AssertionError(
                "FREE_RUN live exact: PP output parity mismatch "
                f"overlap={overlap}"
            )
        # Source-key publication also supports the PP-core exact-snapshot
        # diagnostic path, so ACK OFF does not imply source-key OFF.  Motion
        # execution evidence itself must remain absent.
        if results["free_run_live_off"][7] != 0:
            raise AssertionError("FREE_RUN live exact OFF published ACK")
        if (
            results["free_run_live_on"][7] <= 0
            or results["free_run_live_on"][8] <= 0
        ):
            raise AssertionError("FREE_RUN live exact ON evidence missing")
        clamp_metrics = results["hard_clamp_on"][4]
        if clamp_metrics.get("raw_command", 0.0) <= 0.35:
            raise AssertionError(
                f"hard clamp fixture did not exceed raw limit: {clamp_metrics}"
            )
        for scenario in ("healthy_a", "restart", "healthy_b"):
            records = results[scenario][1]
            drops = results[scenario][2]
            if len(records) < 100:
                raise AssertionError(
                    f"{scenario}: too few callback records ({len(records)})"
                )
            durations = [record[2] for record in records]
            starts = [record[1] for record in records]
            gaps = [
                current - previous
                for previous, current in zip(starts, starts[1:])
                if current >= previous
            ]
            duration_p999 = percentile_999(durations)
            gap_max = max(gaps)
            print(
                f"{scenario} samples={len(records)} "
                f"callback_p99_ns={percentile_99(durations)} "
                f"callback_p999_ns={duration_p999} "
                f"callback_max_ns={max(durations)} "
                f"timer_gap_max_ns={gap_max} observer_drops={drops}"
            )
            if duration_p999 > 1_000_000:
                raise AssertionError(f"{scenario}: callback p99.9 exceeds 1 ms")
            if gap_max > 12_000_000:
                raise AssertionError(f"{scenario}: timer gap exceeds 12 ms")
            if drops != 0:
                raise AssertionError(f"{scenario}: observer dropped records")
            if results[scenario][5] == 0:
                raise AssertionError(f"{scenario}: no shadow snapshot published")
            if results[scenario][5] > len(records) // 4 + 2:
                raise AssertionError(
                    f"{scenario}: shadow publish cadence exceeded 20 Hz"
                )
            if results[scenario][6]:
                raise AssertionError(f"{scenario}: invalid shadow authority")
        for scenario in ("free_run_live_off", "free_run_live_on"):
            records = results[scenario][1]
            drops = results[scenario][2]
            if len(records) < 100:
                raise AssertionError(
                    f"{scenario}: too few callback records ({len(records)})"
                )
            durations = [record[2] for record in records]
            starts = [record[1] for record in records]
            gaps = [
                current - previous
                for previous, current in zip(starts, starts[1:])
                if current >= previous
            ]
            print(
                f"{scenario} samples={len(records)} "
                f"callback_p99_ns={percentile_99(durations)} "
                f"callback_p999_ns={percentile_999(durations)} "
                f"callback_max_ns={max(durations)} "
                f"timer_gap_max_ns={max(gaps)} "
                f"ack_count={results[scenario][7]} "
                f"source_key_count={results[scenario][8]}"
            )
            if percentile_999(durations) > 1_000_000:
                raise AssertionError(
                    f"{scenario}: live callback p99.9 exceeds 1 ms"
                )
            if max(gaps) > 12_000_000:
                raise AssertionError(
                    f"{scenario}: live timer gap exceeds 12 ms"
                )
            if drops != 0:
                raise AssertionError(
                    f"{scenario}: live observer dropped records"
                )
        print("C002AY0_PP_RUNTIME_PARITY=PASS")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
