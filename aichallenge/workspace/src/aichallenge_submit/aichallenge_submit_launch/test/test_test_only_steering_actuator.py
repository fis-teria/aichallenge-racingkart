import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/test_only_steering_actuator.py"
spec = importlib.util.spec_from_file_location("test_only_steering_actuator", SCRIPT)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)


def test_bounded_step_tracks_both_directions_without_overshoot() -> None:
    step = module.bounded_steering_step
    assert step(0.0, 0.4, 0.75, 0.01, 0.48) == pytest.approx(0.0075)
    assert step(0.4, -0.4, 0.75, 0.01, 0.48) == pytest.approx(0.3925)
    assert step(0.397, 0.4, 0.75, 0.01, 0.48) == pytest.approx(0.4)


def test_bounded_step_clamps_physical_angle_and_rejects_invalid_input() -> None:
    step = module.bounded_steering_step
    assert step(0.47, 1.0, 2.0, 0.1, 0.48) == pytest.approx(0.48)
    with pytest.raises(ValueError):
        step(0.0, float("nan"), 0.75, 0.01, 0.48)
