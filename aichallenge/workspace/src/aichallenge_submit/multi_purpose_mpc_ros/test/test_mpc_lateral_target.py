import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.modules.setdefault("osqp", SimpleNamespace(OSQP=lambda: object()))

from multi_purpose_mpc_ros.core.MPC import MPC
from multi_purpose_mpc_ros.overtake_contract import (
    OvertakeSpeedOnlyFailClosedLatch,
    parse_overtake_reference_override,
)


class ModelStub:
    n_states = 3
    safety_margin = 0.0


def make_mpc(*, lateral_target_mode="center_of_corridor", wall_margin_m=0.0):
    return MPC(
        ModelStub(),
        N=2,
        Q=None,
        R=None,
        QN=None,
        StateConstraints={
            "xmin": np.array([-np.inf, -np.inf, -np.inf]),
            "xmax": np.array([np.inf, np.inf, np.inf]),
        },
        InputConstraints={
            "umin": np.array([0.0, -1.0]),
            "umax": np.array([10.0, 1.0]),
        },
        ay_max=10.0,
        max_steering_rate=1.0,
        wp_id_offset=0,
        use_obstacle_avoidance=True,
        use_path_constraints_topic=True,
        lateral_target_mode=lateral_target_mode,
        wall_margin_m=wall_margin_m,
    )


def test_reference_path_lateral_target_uses_zero_when_inside_corridor():
    mpc = make_mpc(lateral_target_mode="reference_path")

    target = mpc._lateral_reference(np.array([1.2, 0.8]), np.array([-0.5, -1.0]))

    assert target == pytest.approx([0.0, 0.0])


def test_reference_path_lateral_target_clips_to_wall_margin_corridor():
    mpc = make_mpc(lateral_target_mode="reference_path", wall_margin_m=0.25)
    ub, lb = mpc._apply_wall_margin(np.array([0.1]), np.array([-1.0]))

    target = mpc._lateral_reference(ub, lb)

    assert ub == pytest.approx([-0.15])
    assert lb == pytest.approx([-0.75])
    assert target == pytest.approx([-0.15])


def test_center_of_corridor_lateral_target_preserves_previous_behavior():
    mpc = make_mpc(lateral_target_mode="center_of_corridor", wall_margin_m=0.25)
    ub, lb = mpc._apply_wall_margin(np.array([1.0]), np.array([-0.5]))

    target = mpc._lateral_reference(ub, lb)

    assert ub == pytest.approx([0.75])
    assert lb == pytest.approx([-0.25])
    assert target == pytest.approx([0.25])


def test_overtake_lateral_override_clips_to_corridor():
    mpc = make_mpc(lateral_target_mode="reference_path")
    mpc.set_overtake_reference_override([0.2, 2.0], [3.0, 4.0], mode_id=3)

    target = mpc._overtake_lateral_reference(
        2, np.array([1.0, 1.0]), np.array([-1.0, -1.0]))

    assert target == pytest.approx([0.2, 1.0])


def test_overtake_lateral_override_uses_snapshot_if_callback_clears(monkeypatch):
    mpc = make_mpc(lateral_target_mode="reference_path")
    mpc.set_overtake_reference_override([0.2, 0.4], [3.0, 4.0], mode_id=3)
    original_lateral_reference = mpc._lateral_reference

    def clearing_lateral_reference(ub, lb):
        mpc.clear_overtake_reference_override()
        return original_lateral_reference(ub, lb)

    monkeypatch.setattr(mpc, "_lateral_reference", clearing_lateral_reference)

    target = mpc._overtake_lateral_reference(
        2, np.array([1.0, 1.0]), np.array([-1.0, -1.0]))

    assert target == pytest.approx([0.2, 0.4])
    assert mpc._overtake_lateral_reference(
        2, np.array([1.0, 1.0]), np.array([-1.0, -1.0])) is None


def test_overtake_override_snapshot_keeps_lateral_and_speed_same_generation():
    mpc = make_mpc(lateral_target_mode="reference_path")
    mpc.set_overtake_reference_override([0.2, 0.4], [3.0, 4.0], mode_id=3)
    mpc.set_overtake_reference_override(
        [0.2, 0.4], [3.0, 4.0], mode_id=3, generation=12)
    mode_id, generation, lateral_offsets, speed_caps, speed_only, _authorized, _mandatory = (
        mpc._overtake_override_snapshot())

    mpc.set_overtake_reference_override([-0.5, -0.6], [1.0, 1.5], mode_id=4)

    assert mode_id == 3
    assert generation == 12
    assert not speed_only
    assert mpc._overtake_lateral_reference(
        2, np.array([1.0, 1.0]), np.array([-1.0, -1.0]),
        lateral_offsets) == pytest.approx([0.2, 0.4])
    assert mpc._overtake_speed_cap(0, speed_caps) == pytest.approx(3.0)
    assert mpc._overtake_speed_cap(1, speed_caps) == pytest.approx(4.0)


def test_prediction_contract_snapshot_keeps_horizon_authorization_together():
    mpc = make_mpc()
    mpc.set_overtake_reference_override(
        [0.3], [2.0], mode_id=7, generation=42,
        solver_horizon_authorized=True, mandatory_lateral_avoidance=True)

    (mode_id, generation, _offsets, _speed_caps, _speed_only, authorized,
     mandatory) = (
        mpc._overtake_override_snapshot())
    mpc._last_problem_prediction_contract = (
        mode_id, generation, authorized, mandatory)
    mpc.set_overtake_reference_override(
        [-0.3], [1.0], mode_id=4, generation=43)
    mpc.current_prediction_contract = mpc._last_problem_prediction_contract

    assert mpc.current_prediction_contract == (7, 42, True, True)


def test_overtake_speed_cap_returns_none_for_invalid_values():
    mpc = make_mpc()
    mpc.set_overtake_reference_override([0.0], [4.0, -1.0], mode_id=1)

    assert mpc._overtake_speed_cap(0) == pytest.approx(4.0)
    assert mpc._overtake_speed_cap(1) is None
    assert mpc._overtake_speed_cap(2) is None


def test_speed_only_override_keeps_baseline_lateral_and_caps_every_horizon_point():
    mpc = make_mpc(lateral_target_mode="reference_path")
    mpc.set_overtake_reference_override(
        [], [0.5], mode_id=11, generation=42, speed_only=True)

    mode_id, generation, lateral_offsets, speed_caps, speed_only, _authorized, _mandatory = (
        mpc._overtake_override_snapshot())

    assert mode_id == 11
    assert generation == 42
    assert speed_only
    assert lateral_offsets is not None
    assert lateral_offsets.size == 0
    assert mpc._overtake_lateral_reference(
        2, np.array([1.0, 1.0]), np.array([-1.0, -1.0]),
        lateral_offsets) is None
    assert mpc._overtake_speed_cap(0, speed_caps, speed_only) == pytest.approx(0.5)
    assert mpc._overtake_speed_cap(1, speed_caps, speed_only) == pytest.approx(0.5)
    assert mpc._overtake_speed_cap(50, speed_caps, speed_only) == pytest.approx(0.5)


def test_speed_only_cap_latches_after_malformed_payload_without_lateral_override():
    mpc = make_mpc(lateral_target_mode="reference_path")
    latch = OvertakeSpeedOnlyFailClosedLatch()
    valid_v2 = parse_overtake_reference_override([1.0, 11.0, 0.0, 2.0, 42.0, 0.5])
    assert valid_v2 is not None
    latch.observe_valid(valid_v2)
    mpc.set_overtake_reference_override(
        valid_v2.lateral_offsets, valid_v2.speed_caps, valid_v2.mode_id,
        valid_v2.generation, speed_only=valid_v2.speed_only)

    malformed = parse_overtake_reference_override(
        [1.0, 11.0, 0.0, 2.0, 42.0, 0.0])
    assert malformed is None
    retained = latch.retained()
    assert retained is not None
    mpc.set_overtake_reference_override(
        retained.lateral_offsets, retained.speed_caps, retained.mode_id,
        retained.generation, speed_only=retained.speed_only)

    mode_id, generation, lateral_offsets, speed_caps, speed_only, _authorized, _mandatory = (
        mpc._overtake_override_snapshot())
    assert (mode_id, generation, speed_only) == (11, 42, True)
    assert lateral_offsets is not None and lateral_offsets.size == 0
    assert mpc._overtake_lateral_reference(
        2, np.array([1.0, 1.0]), np.array([-1.0, -1.0]),
        lateral_offsets) is None
    assert mpc._overtake_speed_cap(0, speed_caps, speed_only) == pytest.approx(0.5)
    assert mpc._overtake_speed_cap(1, speed_caps, speed_only) == pytest.approx(0.5)


def test_speed_only_cap_latches_after_timeout_without_lateral_override():
    mpc = make_mpc(lateral_target_mode="reference_path")
    latch = OvertakeSpeedOnlyFailClosedLatch()
    valid_v2 = parse_overtake_reference_override([1.0, 11.0, 0.0, 2.0, 43.0, 0.4])
    assert valid_v2 is not None
    latch.observe_valid(valid_v2)

    # No new payload arrives before the timeout. The v2-only latch is the
    # controller's retained input, so it continues to cap every horizon point.
    retained = latch.retained()
    assert retained is not None
    mpc.set_overtake_reference_override(
        retained.lateral_offsets, retained.speed_caps, retained.mode_id,
        retained.generation, speed_only=retained.speed_only)

    _mode_id, _generation, lateral_offsets, speed_caps, speed_only, _authorized, _mandatory = (
        mpc._overtake_override_snapshot())
    assert lateral_offsets is not None and lateral_offsets.size == 0
    assert mpc._overtake_lateral_reference(
        2, np.array([1.0, 1.0]), np.array([-1.0, -1.0]),
        lateral_offsets) is None
    assert mpc._overtake_speed_cap(0, speed_caps, speed_only) == pytest.approx(0.4)
    assert mpc._overtake_speed_cap(1, speed_caps, speed_only) == pytest.approx(0.4)


def test_clear_overtake_reference_override_restores_no_override():
    mpc = make_mpc()
    mpc.set_overtake_reference_override([0.3], [2.0], mode_id=1)

    mpc.clear_overtake_reference_override()

    assert mpc._overtake_lateral_reference(
        1, np.array([1.0]), np.array([-1.0])) is None
    assert mpc._overtake_speed_cap(0) is None
