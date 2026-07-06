from hybrid_control_mux.core import HybridMuxConfig, HybridMuxCore, MpcHealth


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
