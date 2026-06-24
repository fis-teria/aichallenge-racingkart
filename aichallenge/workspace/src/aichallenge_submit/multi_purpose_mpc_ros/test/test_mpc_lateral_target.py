import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.modules.setdefault("osqp", SimpleNamespace(OSQP=lambda: object()))

from multi_purpose_mpc_ros.core.MPC import MPC


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
