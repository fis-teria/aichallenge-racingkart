import pytest

from hybrid_control_mux.core import (
    ControlLoopWatchdog,
    HybridMuxConfig,
    HybridMuxCore,
    MpcHealth,
    RecoveryMuxState,
    RosClockProgressWatchdog,
    SafetyConstraintAuthority,
    SafetyConstraintState,
    SteeringLimiter,
    SteeringLimiterConfig,
    apply_safety_constraint,
)


def safety_constraint(
    constraint_generation: int,
    *,
    plan_generation: int = 7,
    valid: bool = True,
    stop_requested: bool = False,
    release_authorized: bool = False,
    speed_limit_mps: float = 2.0,
    required_brake_decel_mps2: float = 0.0,
) -> SafetyConstraintState:
    return SafetyConstraintState(
        constraint_generation=constraint_generation,
        plan_generation=plan_generation,
        valid=valid,
        stop_requested=stop_requested,
        release_authorized=release_authorized,
        speed_limit_mps=speed_limit_mps,
        required_brake_decel_mps2=required_brake_decel_mps2,
    )


def safety_authority(*, required: bool = True) -> SafetyConstraintAuthority:
    return SafetyConstraintAuthority(
        required=required,
        timeout_sec=0.2,
        maximum_speed_limit_mps=15.0,
        maximum_brake_decel_mps2=6.0,
    )


def test_speed_limit_cannot_increase_without_explicit_release():
    authority = safety_authority()
    first = authority.evaluate(
        safety_constraint(10), received_time_sec=1.0, now_sec=1.0
    )
    denied = authority.evaluate(
        safety_constraint(11, speed_limit_mps=6.0),
        received_time_sec=1.1,
        now_sec=1.1,
    )

    assert first.speed_limit_mps == 2.0
    assert denied.speed_limit_mps == 2.0
    assert denied.reason == "safety_constraint_relaxation_not_authorized"


def test_new_generation_explicit_release_can_raise_speed_limit():
    authority = safety_authority()
    authority.evaluate(safety_constraint(10), received_time_sec=1.0, now_sec=1.0)
    released = authority.evaluate(
        safety_constraint(11, speed_limit_mps=6.0, release_authorized=True),
        received_time_sec=1.1,
        now_sec=1.1,
    )

    assert not released.stop_required
    assert released.speed_limit_mps == 6.0


def test_same_generation_changed_payload_forces_stop():
    authority = safety_authority()
    authority.evaluate(safety_constraint(10), received_time_sec=1.0, now_sec=1.0)
    conflict = authority.evaluate(
        safety_constraint(10, speed_limit_mps=6.0, release_authorized=True),
        received_time_sec=1.1,
        now_sec=1.1,
    )

    assert conflict.stop_required
    assert conflict.reason == "safety_constraint_same_generation_conflict"


def test_older_generation_and_plan_generation_regression_force_stop():
    authority = safety_authority()
    authority.evaluate(safety_constraint(10), received_time_sec=1.0, now_sec=1.0)

    old_constraint = authority.evaluate(
        safety_constraint(9), received_time_sec=1.1, now_sec=1.1
    )
    old_plan = authority.evaluate(
        safety_constraint(11, plan_generation=6),
        received_time_sec=1.1,
        now_sec=1.1,
    )

    assert old_constraint.stop_required
    assert old_constraint.reason == "safety_constraint_generation_regression"
    assert old_plan.stop_required
    assert old_plan.reason == "safety_constraint_plan_generation_regression"


@pytest.mark.parametrize(
    ("constraint", "received_time_sec", "now_sec", "reason"),
    [
        (None, None, 1.0, "safety_constraint_missing"),
        (safety_constraint(1), 1.0, 1.3, "safety_constraint_stale"),
        (
            safety_constraint(1, valid=False),
            1.0,
            1.0,
            "safety_constraint_invalid",
        ),
        (
            safety_constraint(1, speed_limit_mps=float("nan")),
            1.0,
            1.0,
            "safety_constraint_invalid",
        ),
        (
            safety_constraint(1, required_brake_decel_mps2=float("inf")),
            1.0,
            1.0,
            "safety_constraint_invalid",
        ),
    ],
)
def test_missing_stale_invalid_or_nonfinite_constraint_forces_stop(
    constraint, received_time_sec, now_sec, reason
):
    decision = safety_authority().evaluate(
        constraint, received_time_sec=received_time_sec, now_sec=now_sec
    )

    assert decision.stop_required
    assert decision.reason == reason


def test_plan_generation_mismatch_forces_stop_when_active_plan_is_known():
    decision = safety_authority().evaluate(
        safety_constraint(1, plan_generation=7),
        received_time_sec=1.0,
        now_sec=1.0,
        active_plan_generation=8,
    )

    assert decision.stop_required
    assert decision.reason == "safety_constraint_plan_generation_mismatch"


def test_timestamp_regression_and_wrong_frame_force_stop():
    authority = safety_authority()
    first = safety_constraint(1)
    first = SafetyConstraintState(
        **{**first.__dict__, "header_stamp_ns": 100}
    )
    authority.evaluate(
        first, received_time_sec=1.0, now_sec=1.0
    )
    regressed = safety_constraint(2)
    regressed = SafetyConstraintState(
        **{**regressed.__dict__, "header_stamp_ns": 99}
    )
    wrong_frame = safety_constraint(2)
    wrong_frame = SafetyConstraintState(
        **{**wrong_frame.__dict__, "frame_id": "odom"}
    )

    regression_decision = authority.evaluate(
        regressed, received_time_sec=1.1, now_sec=1.1
    )
    assert regression_decision.stop_required
    assert regression_decision.reason == "safety_constraint_timestamp_regression"
    assert authority.evaluate(
        wrong_frame, received_time_sec=1.1, now_sec=1.1
    ).stop_required


def test_stale_fault_requires_consecutive_explicit_safe_release():
    authority = safety_authority()
    authority.evaluate(
        safety_constraint(1), received_time_sec=1.0, now_sec=1.0
    )
    stale = authority.evaluate(
        safety_constraint(1), received_time_sec=1.0, now_sec=1.3
    )
    assert stale.stop_required

    releases = [
        SafetyConstraintState(
            **{
                **safety_constraint(2, release_authorized=True).__dict__,
                "header_stamp_ns": stamp_ns,
            }
        )
        for stamp_ns in (1, 2, 3)
    ]
    first = authority.evaluate(releases[0], received_time_sec=1.31, now_sec=1.31)
    second = authority.evaluate(releases[1], received_time_sec=1.32, now_sec=1.32)
    third = authority.evaluate(releases[2], received_time_sec=1.33, now_sec=1.33)

    assert first.stop_required
    assert second.stop_required
    assert not third.stop_required


def test_startup_missing_does_not_block_first_valid_low_constraint():
    authority = safety_authority()
    assert authority.evaluate(
        None, received_time_sec=None, now_sec=1.0
    ).stop_required

    first_valid = authority.evaluate(
        safety_constraint(1, speed_limit_mps=0.5),
        received_time_sec=1.01,
        now_sec=1.01,
    )

    assert not first_valid.stop_required
    assert first_valid.speed_limit_mps == 0.5


def test_required_brake_overrides_only_weaker_acceleration():
    authority = safety_authority()
    decision = authority.evaluate(
        safety_constraint(1, required_brake_decel_mps2=1.2),
        received_time_sec=1.0,
        now_sec=1.0,
    )

    assert apply_safety_constraint(8.0, 2.0, decision) == (2.0, -1.2)
    assert apply_safety_constraint(8.0, -0.5, decision) == (2.0, -1.2)
    assert apply_safety_constraint(8.0, -2.0, decision) == (2.0, -2.0)


def test_stop_constraint_also_enforces_required_braking():
    authority = safety_authority()
    decision = authority.evaluate(
        safety_constraint(
            1, stop_requested=True, required_brake_decel_mps2=1.2
        ),
        received_time_sec=1.0,
        now_sec=1.0,
    )

    assert apply_safety_constraint(8.0, -0.5, decision) == (0.0, -1.2)


def test_constraint_latch_is_independent_of_controller_source_switch():
    authority = safety_authority()
    decision = authority.evaluate(
        safety_constraint(10), received_time_sec=1.0, now_sec=1.0
    )

    for _source in ("mpc", "pure_pursuit", "mpc"):
        speed_mps, _ = apply_safety_constraint(8.0, 1.0, decision)
        assert speed_mps == 2.0


def test_control_loop_watchdog_uses_supplied_steady_time():
    watchdog = ControlLoopWatchdog(maximum_gap_sec=0.06)

    first = watchdog.update(1.0)
    on_time = watchdog.update(1.02)
    late = watchdog.update(1.10)

    assert not first.deadline_missed
    assert not on_time.deadline_missed
    assert late.deadline_missed
    assert late.publish_gap_sec == pytest.approx(0.08)


def test_ros_clock_progress_watchdog_detects_stall_and_regression():
    watchdog = RosClockProgressWatchdog(maximum_stall_sec=0.2)

    assert not watchdog.update(100, 1.0).stalled
    assert not watchdog.update(100, 1.1).stalled
    assert watchdog.update(100, 1.21).stalled
    assert watchdog.update(99, 1.22).reason == "ros_clock_regression"
    assert not watchdog.update(101, 1.23).stalled


def test_uses_mpc_when_healthy():
    core = HybridMuxCore(HybridMuxConfig())
    decision = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
    )

    assert decision.source == "mpc"
    assert not decision.fallback_active


def test_pure_pursuit_primary_uses_pure_pursuit_even_when_mpc_is_healthy():
    core = HybridMuxCore(HybridMuxConfig(primary_source="pure_pursuit"))
    decision = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
    )

    assert decision.source == "pure_pursuit"
    assert not decision.fallback_active
    assert decision.reason == "primary_pure_pursuit"


def test_pure_pursuit_primary_stops_when_pure_pursuit_command_times_out():
    core = HybridMuxCore(HybridMuxConfig(primary_source="pure_pursuit"))
    decision = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=False,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
    )

    assert decision.source == "stop"
    assert decision.fallback_active
    assert decision.reason == "pure_pursuit_cmd_timeout"


def test_pure_pursuit_primary_can_use_mpc_when_pure_pursuit_command_times_out():
    core = HybridMuxCore(
        HybridMuxConfig(
            primary_source="pure_pursuit",
            use_mpc_on_pure_pursuit_cmd_timeout=True,
        )
    )
    decision = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=False,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
    )

    assert decision.source == "mpc"
    assert decision.fallback_active
    assert decision.reason == "pure_pursuit_cmd_timeout"


def test_switches_to_pure_pursuit_after_infeasible_threshold():
    core = HybridMuxCore(HybridMuxConfig(fallback_trigger_infeasible_count=2))
    decision = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "infeasible", 2, 0.1),
    )

    assert decision.source == "pure_pursuit"
    assert decision.fallback_active
    assert decision.reason == "mpc_infeasible"


def test_holds_fallback_until_mpc_is_stably_solved():
    core = HybridMuxCore(
        HybridMuxConfig(
            fallback_trigger_infeasible_count=1,
            fallback_release_solved_cycles=2,
            fallback_min_hold_sec=1.0,
        )
    )
    first = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "infeasible", 1, 0.1),
    )
    assert first.source == "pure_pursuit"

    held = core.update(
        1.5,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
    )
    assert held.source == "pure_pursuit"

    released = core.update(
        2.1,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
    )
    assert released.source == "mpc"
    assert not released.fallback_active


def test_stops_when_fallback_requested_without_pure_pursuit_command():
    core = HybridMuxCore(HybridMuxConfig(fallback_trigger_infeasible_count=1))
    decision = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=False,
        mpc_health=MpcHealth(True, "infeasible", 1, 0.1),
    )

    assert decision.source == "stop"
    assert decision.fallback_active
    assert decision.reason == "fallback_without_pure_pursuit_cmd"

    timeout_decision = core.update(
        1.1,
        mpc_cmd_fresh=False,
        pure_pursuit_cmd_fresh=False,
        mpc_health=MpcHealth(False, "stale", 1, 1.0),
    )
    assert timeout_decision.source == "stop"


def test_recovery_inactive_does_not_change_normal_mpc_selection():
    core = HybridMuxCore(HybridMuxConfig())

    decision = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
        recovery=RecoveryMuxState(enabled=True, status_state="inactive", status_fresh=True),
    )

    assert decision.source == "mpc"
    assert not core.recovery_episode_latched


def test_recovery_active_wins_when_all_typed_gates_match():
    core = HybridMuxCore(HybridMuxConfig(recovery_max_duration_sec=3.0))
    recovery = RecoveryMuxState(
        enabled=True,
        status_state="active",
        status_fresh=True,
        command_fresh=True,
        permit_fresh=True,
        external_safety_ok=True,
        input_complete=True,
        trajectory_valid=True,
        trajectory_safe=True,
        recovery_allowed=True,
        ids_match=True,
        trajectory_header_match=True,
        command_valid=True,
        command_forward_only=True,
    )

    decision = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
        recovery=recovery,
    )

    assert decision.source == "recovery"
    assert core.recovery_episode_latched


def test_recovery_latch_stops_instead_of_falling_back_when_status_times_out():
    core = HybridMuxCore(HybridMuxConfig())
    stop_hold = RecoveryMuxState(
        enabled=True,
        status_state="stop_hold",
        status_fresh=True,
        external_safety_ok=True,
    )
    first = core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
        recovery=stop_hold,
    )
    assert first.source == "stop"
    assert core.recovery_episode_latched

    timed_out = core.update(
        1.1,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
        recovery=RecoveryMuxState(enabled=True, status_fresh=False),
    )

    assert timed_out.source == "stop"
    assert timed_out.reason == "recovery_status_timeout"
    assert core.recovery_episode_latched


def test_recovery_complete_releases_latch_to_normal_source():
    core = HybridMuxCore(HybridMuxConfig())
    core.update(
        1.0,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
        recovery=RecoveryMuxState(
            enabled=True,
            status_state="active",
            status_fresh=True,
            command_fresh=True,
            permit_fresh=True,
            external_safety_ok=True,
            input_complete=True,
            trajectory_valid=True,
            trajectory_safe=True,
            recovery_allowed=True,
            ids_match=True,
            trajectory_header_match=True,
            command_valid=True,
            command_forward_only=True,
        ),
    )

    released = core.update(
        1.2,
        mpc_cmd_fresh=True,
        pure_pursuit_cmd_fresh=True,
        mpc_health=MpcHealth(True, "solved", 0, 0.1),
        recovery=RecoveryMuxState(
            enabled=True,
            status_state="complete",
            status_fresh=True,
            permit_fresh=True,
            external_safety_ok=True,
            handoff_ready=True,
            handoff_allowed=True,
        ),
    )

    assert released.source == "mpc"
    assert not core.recovery_episode_latched


def test_steering_limiter_clamps_absolute_angle():
    limiter = SteeringLimiter(
        SteeringLimiterConfig(
            enabled=True,
            max_steering_angle_rad=0.5,
            max_steering_rate_radps=10.0,
        )
    )

    result = limiter.update(1.0, 1.0, "mpc")

    assert result.limited_steering_rad == pytest.approx(0.5)
    assert result.angle_limited
    assert not result.rate_limited


def test_steering_limiter_rate_limits_across_source_switch_by_default():
    limiter = SteeringLimiter(
        SteeringLimiterConfig(
            enabled=True,
            max_steering_angle_rad=1.0,
            max_steering_rate_radps=1.0,
            reset_on_mode_change=False,
        )
    )
    first = limiter.update(0.0, 1.00, "mpc")
    second = limiter.update(0.50, 1.02, "pure_pursuit")

    assert first.limiter_reset
    assert second.limited_steering_rad == pytest.approx(0.02)
    assert second.rate_limited
    assert not second.limiter_reset


def test_steering_limiter_can_reset_on_source_switch():
    limiter = SteeringLimiter(
        SteeringLimiterConfig(
            enabled=True,
            max_steering_angle_rad=1.0,
            max_steering_rate_radps=1.0,
            reset_on_mode_change=True,
        )
    )
    limiter.update(0.0, 1.00, "mpc")
    result = limiter.update(0.50, 1.02, "pure_pursuit")

    assert result.limited_steering_rad == pytest.approx(0.50)
    assert not result.rate_limited
    assert result.limiter_reset


def test_steering_limiter_treats_nonfinite_input_as_zero():
    limiter = SteeringLimiter(
        SteeringLimiterConfig(
            enabled=True,
            max_steering_angle_rad=0.5,
            max_steering_rate_radps=1.0,
        )
    )

    result = limiter.update(float("nan"), 1.0, "pure_pursuit")

    assert result.raw_steering_rad == pytest.approx(0.0)
    assert result.limited_steering_rad == pytest.approx(0.0)


def test_steering_limiter_reset_bypasses_rate_limit_for_stop():
    limiter = SteeringLimiter(
        SteeringLimiterConfig(
            enabled=True,
            max_steering_angle_rad=1.0,
            max_steering_rate_radps=0.1,
        )
    )
    limiter.update(0.8, 1.00, "mpc")

    result = limiter.reset(0.0, 1.01, "stop")

    assert result.limited_steering_rad == pytest.approx(0.0)
    assert not result.rate_limited
    assert result.limiter_reset
