import math

import numpy as np
import pytest

from multi_purpose_mpc_ros.overtake_contract import (
    OvertakeSpeedOnlyFailClosedLatch,
    parse_overtake_reference_override,
)


def test_v1_override_contract_preserves_exact_generation() -> None:
    override = parse_overtake_reference_override(
        [np.float32(1.0), 7.0, 2.0, 0.4, 0.5, 3.0, 2.0, 1.0, 42.0])

    assert override is not None
    assert override.mode_id == 7
    assert override.generation == 42
    assert override.lateral_offsets == pytest.approx((0.4, 0.5))
    assert not override.speed_only
    assert not override.solver_horizon_authorized


def test_v3_override_contract_binds_solver_horizon_authorization_to_generation() -> None:
    override = parse_overtake_reference_override(
        [1.0, 7.0, 2.0, 0.4, 0.5, 3.0, 2.0, 3.0, 42.0, 2.0])

    assert override is not None
    assert override.mode_id == 7
    assert override.generation == 42
    assert override.solver_horizon_authorized
    assert override.mandatory_lateral_avoidance


def test_v2_speed_only_contract_has_no_lateral_points() -> None:
    override = parse_overtake_reference_override(
        [np.float32(1.0), 11.0, 0.0, 2.0, 42.0, 0.5])

    assert override is not None
    assert override.mode_id == 11
    assert override.point_count == 0
    assert override.generation == 42
    assert override.speed_only
    assert override.lateral_offsets == ()
    assert override.speed_caps == pytest.approx((0.5,))


def test_explicit_inactive_v1_is_not_reinterpreted_as_speed_only() -> None:
    override = parse_overtake_reference_override([1.0, 0.0, 0.0, 1.0, 42.0])

    assert override is not None
    assert override.mode_id == 0
    assert override.point_count == 0
    assert not override.speed_only


def test_speed_only_latch_retains_cap_after_malformed_payload() -> None:
    latch = OvertakeSpeedOnlyFailClosedLatch()
    valid_v2 = parse_overtake_reference_override(
        [1.0, 11.0, 0.0, 2.0, 42.0, 0.5])
    assert valid_v2 is not None
    latch.observe_valid(valid_v2)

    assert parse_overtake_reference_override(
        [1.0, 11.0, 0.0, 2.0, 42.0, 0.0]) is None
    retained = latch.retained()
    assert retained is not None
    assert retained.lateral_offsets == ()
    assert retained.speed_caps == pytest.approx((0.5,))


def test_speed_only_latch_retains_cap_after_timeout_until_explicit_inactive() -> None:
    latch = OvertakeSpeedOnlyFailClosedLatch()
    valid_v2 = parse_overtake_reference_override(
        [1.0, 11.0, 0.0, 2.0, 43.0, 0.4])
    assert valid_v2 is not None
    latch.observe_valid(valid_v2)

    # timeout itself supplies no replacement payload, so the retained v2 cap
    # remains available to the controller.
    retained = latch.retained()
    assert retained is not None
    assert retained.lateral_offsets == ()
    assert retained.speed_caps == pytest.approx((0.4,))

    explicit_inactive = parse_overtake_reference_override(
        [1.0, 0.0, 0.0, 1.0, 44.0])
    assert explicit_inactive is not None
    latch.observe_valid(explicit_inactive)
    assert latch.retained() is None


@pytest.mark.parametrize(
    "payload",
    [
        [math.nan, 7.0, 1.0, 0.2, 3.0],
        [1.0, 7.5, 1.0, 0.2, 3.0],
        [1.0, 7.0, 1001.0],
        [1.0, 7.0, 1.0, math.inf, 3.0],
        [1.0, 7.0, 1.0, 0.2, 3.0, 1.5, 42.0],
        [1.0, 7.0, 1.0, 0.2, 3.0, 1.0, 42.5],
        [1.0, 7.0, 1.0, 0.2, 3.0, 1.0, math.nan],
        [1.0, 7.0, 1.0, 0.2, 3.0, 3.0, 42.0, 3.0],
        [1.0, 7.0, 1.0, 0.2, 3.0, 3.0, 42.0, 2.5],
        [1.0, 0.0, 0.0, 2.0, 42.0, 0.5],
        [1.0, 7.0, 0.0, 2.0, 0.0, 0.5],
        [1.0, 7.0, 0.0, 2.0, 42.0, 0.0],
        [1.0, 7.0, 0.0, 2.0, 42.0, math.inf],
        [1.0, 7.0, 0.0, 1.0, 42.0, 0.5],
        [1.0, 7.0, 1.0, 0.2, 0.0],
    ],
)
def test_malformed_override_contract_fails_closed(payload: list[float]) -> None:
    assert parse_overtake_reference_override(payload) is None
