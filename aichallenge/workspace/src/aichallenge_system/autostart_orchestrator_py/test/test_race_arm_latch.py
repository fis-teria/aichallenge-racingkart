from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest


def _load_latch_class():
    node_path = (
        Path(__file__).resolve().parents[1]
        / "autostart_orchestrator_py/autostart_orchestrator_node.py"
    )
    spec = spec_from_file_location("autostart_orchestrator_node", node_path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._RaceArmLatch


_RaceArmLatch = _load_latch_class()


def _latch():
    return _RaceArmLatch(
        ["Start"],
        ["Spawned", "Grounded", "Finish"],
        ["Ready"],
        ["Finish"],
    )


def _ready_arm_latch():
    return _RaceArmLatch(
        ["Ready", "Start"],
        ["Spawned", "Grounded", "Finish"],
        ["Ready"],
        ["Finish"],
    )


def test_arm_states_require_a_neutral_qualification_state():
    with pytest.raises(ValueError, match="neutral_states must not be empty"):
        _RaceArmLatch(
            ["Start"],
            ["Spawned", "Grounded", "Finish"],
            [],
            ["Finish"],
        )


def test_ready_waits_for_successful_initialization_then_arms():
    latch = _ready_arm_latch()
    assert latch.observe_state("Grounded") is False
    accepted, generation = latch.begin_initialization()
    assert accepted is True

    assert latch.observe_state("Ready") is False
    assert latch.initialization_active(generation) is True
    assert latch.complete_initialization(generation, True) == (True, True)
    assert latch.armed is True


def test_ready_never_arms_after_failed_initialization():
    latch = _ready_arm_latch()
    assert latch.observe_state("Grounded") is False
    accepted, generation = latch.begin_initialization()
    assert accepted is True
    assert latch.observe_state("Ready") is False

    assert latch.complete_initialization(generation, False) == (True, False)
    assert latch.observe_state("Ready") is False
    assert latch.armed is False


def test_early_start_waits_for_ready_arm_generation():
    latch = _ready_arm_latch()
    assert latch.observe_state("Grounded") is False
    accepted, generation = latch.begin_initialization()
    assert accepted is True

    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_before_neutral"
    assert latch.initialization_active(generation) is True
    assert latch.complete_initialization(generation, True) == (True, False)
    assert latch.observe_state("Ready") is True


def test_start_before_ready_does_not_invalidate_initialization_generation():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation

    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_before_neutral"
    assert latch.initialization_active(generation) is True
    assert latch.observe_state("Ready") is False
    accepted, armed = latch.complete_initialization(generation, True)

    assert accepted is True
    assert armed is False
    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_requires_official_start"
    assert latch.observe_official_start() is True


def test_initialization_before_start_requires_official_start_service():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.complete_initialization(generation, True) == (True, False)
    assert latch.initialization_complete is True

    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_requires_official_start"
    assert latch.observe_official_start() is True
    assert latch.observe_state("Ready") is True


def test_pre_ready_start_after_initialization_waits_for_official_start():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.complete_initialization(generation, True) == (True, False)

    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_before_neutral"
    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_requires_official_start"
    assert latch.observe_official_start() is True


def test_repeated_pre_ready_start_preserves_initialization_and_generation():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.complete_initialization(generation, True) == (True, False)

    for _ in range(3):
        assert latch.observe_state("Start") is False
        assert latch.last_observation_reason == "arm_state_before_neutral"
        assert latch.generation == generation
        assert latch.initialization_complete is True

    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_requires_official_start"
    assert latch.observe_official_start() is True


def test_reset_clears_ready_qualification_before_next_start():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.complete_initialization(generation, True) == (True, False)
    assert latch.observe_state("Ready") is False

    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    reset_generation = latch.generation
    assert latch.complete_initialization(reset_generation, True) == (True, False)
    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_before_neutral"


def test_duplicate_grounded_preserves_completed_initialization():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.complete_initialization(generation, True) == (True, False)

    assert latch.observe_state("Grounded") is False
    assert latch.last_observation_reason == "duplicate_reset_state"
    assert latch.generation == generation
    assert latch.initialization_complete is True
    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_requires_official_start"
    assert latch.observe_official_start() is True


def test_failed_initialization_never_arms_from_start_or_ready():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.observe_state("Start") is False
    assert latch.complete_initialization(generation, False) == (True, False)

    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_requires_official_start"
    assert latch.observe_official_start() is False
    assert latch.last_observation_reason == "official_start_before_initialization"


def test_finish_invalidates_stale_initialization_and_delayed_start():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.observe_state("Finish") is False
    assert latch.complete_initialization(generation, True) == (False, False)
    assert latch.observe_state("Start") is False
    initialization_accepted, delayed_generation = latch.begin_initialization()
    assert initialization_accepted is False
    assert latch.complete_initialization(delayed_generation, True) == (False, False)
    assert latch.observe_state("Start") is False


def test_unknown_state_fails_closed_until_explicit_reset_and_reinitialization():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.complete_initialization(generation, True) == (True, False)
    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.observe_official_start() is True

    assert latch.observe_state("CORRUPTED_STATE") is False
    assert latch.observe_state("Start") is False

    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    new_generation = latch.generation
    assert latch.complete_initialization(new_generation, True) == (True, False)
    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.observe_official_start() is True


@pytest.mark.parametrize("invalid_state", ["", "   ", "---"])
def test_empty_or_non_normalizable_state_fails_closed(invalid_state):
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.complete_initialization(generation, True) == (True, False)
    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.observe_official_start() is True

    assert latch.observe_state(invalid_state) is False
    assert latch.begin_initialization()[0] is False


@pytest.mark.parametrize("reset_state", ["Finish", "Spawned", "Grounded"])
def test_explicit_reset_disarms_latched_race(reset_state):
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    generation = latch.generation
    assert latch.complete_initialization(generation, True) == (True, False)
    assert latch.observe_state("Ready") is False
    assert latch.observe_state("Start") is False
    assert latch.observe_official_start() is True
    assert latch.observe_state("Ready") is True

    assert latch.observe_state(reset_state) is False
    assert latch.armed is False
    # A delayed Start from the completed generation cannot re-arm without a
    # fresh initialization result.
    assert latch.observe_state("Start") is False


def test_official_start_arms_only_after_ready_and_initialization():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    accepted, generation = latch.begin_initialization()
    assert accepted is True
    assert latch.complete_initialization(generation, True) == (True, False)

    assert latch.observe_official_start() is False
    assert latch.last_observation_reason == "official_start_before_neutral"
    assert latch.generation == generation
    assert latch.initialization_complete is True
    assert latch.observe_state("Ready") is False
    vehicle_state_before_service = latch._last_state

    assert latch.observe_official_start() is True
    assert latch.last_observation_reason == "official_start"
    assert latch._last_state == vehicle_state_before_service
    assert latch.observe_official_start() is True
    assert latch.last_observation_reason == "already_armed"
    assert latch.generation == generation


def test_official_start_before_initialization_rejects_delayed_completion():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    accepted, generation = latch.begin_initialization()
    assert accepted is True
    assert latch.observe_state("Ready") is False
    vehicle_state_before_service = latch._last_state

    assert latch.observe_official_start() is False
    assert latch.last_observation_reason == "official_start_before_initialization"
    assert latch._last_state == vehicle_state_before_service
    assert latch.generation == generation + 1
    assert latch.complete_initialization(generation, True) == (False, False)
    assert latch.observe_official_start() is False
    assert latch.last_observation_reason == "terminal_blocked"


def test_gate_ready_arm_makes_official_start_idempotent():
    latch = _ready_arm_latch()
    assert latch.observe_state("Grounded") is False
    accepted, generation = latch.begin_initialization()
    assert accepted is True
    assert latch.complete_initialization(generation, True) == (True, False)
    assert latch.observe_state("Ready") is True
    vehicle_state_before_service = latch._last_state

    assert latch.observe_official_start() is True
    assert latch.last_observation_reason == "already_armed"
    assert latch._last_state == vehicle_state_before_service
    assert latch.generation == generation


def test_reset_blocks_delayed_official_start_until_current_ready():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    assert latch.begin_initialization()[0] is True
    first_generation = latch.generation
    assert latch.complete_initialization(first_generation, True) == (True, False)
    assert latch.observe_state("Ready") is False
    assert latch.observe_official_start() is True

    assert latch.observe_state("Grounded") is False
    reset_generation = latch.generation
    assert latch.observe_official_start() is False
    assert latch.last_observation_reason == "official_start_before_neutral"
    assert latch.generation == reset_generation
    assert latch.armed is False


def test_initialization_active_waits_through_vehicle_start_until_finish():
    latch = _latch()
    assert latch.observe_state("Grounded") is False
    accepted, generation = latch.begin_initialization()
    assert accepted is True
    assert latch.initialization_active(generation) is True
    assert latch.observe_state("Ready") is False
    assert latch.initialization_active(generation) is True

    assert latch.observe_state("Start") is False
    assert latch.last_observation_reason == "arm_state_requires_official_start"
    assert latch.initialization_active(generation) is True
    assert latch.observe_state("Finish") is False
    assert latch.initialization_active(generation) is False
