import math

import numpy as np
import pytest

from multi_purpose_mpc_ros.overtake_contract import (
    parse_overtake_reference_override,
)


def test_v1_override_contract_preserves_exact_generation() -> None:
    override = parse_overtake_reference_override(
        [np.float32(1.0), 7.0, 2.0, 0.4, 0.5, 3.0, 2.0, 1.0, 42.0])

    assert override is not None
    assert override.mode_id == 7
    assert override.generation == 42
    assert override.lateral_offsets == pytest.approx((0.4, 0.5))


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
    ],
)
def test_malformed_override_contract_fails_closed(payload: list[float]) -> None:
    assert parse_overtake_reference_override(payload) is None
