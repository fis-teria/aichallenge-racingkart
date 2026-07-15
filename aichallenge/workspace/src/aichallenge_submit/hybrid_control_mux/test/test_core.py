import pytest

from hybrid_control_mux.core import (
    HybridMuxConfig,
    HybridMuxCore,
    MpcHealth,
    RecoveryMuxState,
    SteeringLimiter,
    SteeringLimiterConfig,
)


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
