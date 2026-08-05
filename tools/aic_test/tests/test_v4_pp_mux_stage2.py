from __future__ import annotations

import ast
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest

from aic_test.v4_pp_mux_driver import (
    DEFAULT_POST_INVALID_DWELL_S,
    _effective_duration_s,
    _phase,
    _v4_payload,
    _validate_private_root as driver_private_root_valid,
)
from aic_test.v4_pp_mux_observer import (
    CAUSAL_WINDOW_S,
    DIAGNOSTIC_COUNTER_MAX,
    MuxStopPayload,
    Stage2Evidence,
    _is_direct_invalid_stop_candidate,
    _is_exact_invalid_selector_context_candidate,
    _classify_invalid_fixture,
    _classify_post_invalid_stop_payload,
    _saturating_increment,
    _record_bounded_counter,
    _terminal_no_callback,
    _latch_first_post_invalid_stop,
    _latch_exact_mux_diagnostic_stop,
    _is_first_zero_envelope_candidate,
    _select_direct_invalid_stop_candidate,
    _normalize_mux_stop_payload,
    _same_sign,
    _valid_v4_payload,
    _validate_private_root as observer_private_root_valid,
)
from aic_test.v4_pp_mux_stage2_runner import (
    _setsid_command,
    _terminate_owned_group,
    _validate_fixture_commands,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
DRIVER_SOURCE = PACKAGE_ROOT / "aic_test/v4_pp_mux_driver.py"
OBSERVER_SOURCE = PACKAGE_ROOT / "aic_test/v4_pp_mux_observer.py"
LAUNCH_PATH = REPOSITORY_ROOT / (
    "aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/"
    "launch/test_only/v4_pp_mux_stage2.launch.xml"
)


@pytest.mark.parametrize(
    "root, expected",
    [
        ("/aic_test/v4_pp_mux_stage2/run_01", True),
        ("/aic_test/v4_pp_mux_stage2/run-01", True),
        ("/aic_test/v4_pp_mux_stage2/", False),
        ("/aic_test/v4_pp_mux_stage2/a/b", False),
        ("/overtake", False),
    ],
)
def test_private_root_is_narrow_and_shared(root: str, expected: bool) -> None:
    assert driver_private_root_valid(root) is expected
    assert observer_private_root_valid(root) is expected


def test_driver_builds_valid_v4_then_representative_invalid() -> None:
    valid = _v4_payload(41, valid=True)
    assert _valid_v4_payload(valid)
    assert valid[-3:] == [4.0, 41.0, 2.0]
    assert not _valid_v4_payload(_v4_payload(41, valid=False))
    assert _phase(0.49, 0.5, 3.5) == "baseline"
    assert _phase(0.50, 0.5, 3.5) == "valid"
    assert _phase(4.0, 0.5, 3.5) == "invalid"
    assert DEFAULT_POST_INVALID_DWELL_S == 1.0
    assert _effective_duration_s(0.5, 3.5, 1.0) == 5.0


def test_invalid_fixture_classifier_requires_fresh_base_and_pp_zero_envelope() -> None:
    assert _classify_invalid_fixture(
        base_fresh=True, invalid_v4_count=2, pp_zero_seen=True, envelope_seen=True,
        reject_reason="state_lattice_v4_unavailable",
    ) == "PP_INVALID_CONFIRMED"
    assert _classify_invalid_fixture(
        base_fresh=False, invalid_v4_count=2, pp_zero_seen=True, envelope_seen=True,
        reject_reason="state_lattice_v4_unavailable",
    ) == "FIXTURE_BASE_STREAM_ENDED"
    assert _classify_invalid_fixture(
        base_fresh=True, invalid_v4_count=2, pp_zero_seen=False, envelope_seen=True,
        reject_reason="state_lattice_v4_unavailable",
    ) == "NOT_EVALUATED"


def test_post_invalid_diagnostic_classifies_each_rejection_without_verdict_credit() -> None:
    base = {
        "selected_source": "stop", "output_speed_mps": 0.0,
        "output_stamp_sec": 20, "output_stamp_nanosec": 20,
        "evaluated_pure_pursuit_command_stamp_sec": 20,
        "evaluated_pure_pursuit_command_stamp_nanosec": 10,
        "evaluated_pure_pursuit_envelope_command_stamp_sec": 20,
        "evaluated_pure_pursuit_envelope_command_stamp_nanosec": 10,
        "evaluated_pure_pursuit_selection_rejected_for_invalid_tracking": True,
        "evaluated_pure_pursuit_rejection_kind": "invalid_tracking",
        "final_stop_origin": "invalid_tracking_selector_rejection",
        "final_stop_origin_proven": True,
    }
    assert _classify_post_invalid_stop_payload({}, (20, 10)) == "stop_normalization_reject"
    assert _classify_post_invalid_stop_payload(base, None) == "invalid_pp_stamp_missing"
    assert _classify_post_invalid_stop_payload({**base, "evaluated_pure_pursuit_command_stamp_nanosec": 11}, (20, 10)) == "command_stamp_mismatch"
    assert _classify_post_invalid_stop_payload({**base, "evaluated_pure_pursuit_envelope_command_stamp_nanosec": 11}, (20, 10)) == "envelope_stamp_mismatch"
    assert _classify_post_invalid_stop_payload({**base, "evaluated_pure_pursuit_selection_rejected_for_invalid_tracking": False}, (20, 10)) == "selector_flag_mismatch"
    assert _classify_post_invalid_stop_payload({**base, "evaluated_pure_pursuit_rejection_kind": "stale"}, (20, 10)) == "rejection_kind_mismatch"
    assert _classify_post_invalid_stop_payload({**base, "final_stop_origin_proven": False}, (20, 10)) == "exact_candidate"
    assert _classify_post_invalid_stop_payload(base, (20, 10)) == "exact_candidate"
    assert _saturating_increment(DIAGNOSTIC_COUNTER_MAX) == DIAGNOSTIC_COUNTER_MAX


def test_bounded_counter_sets_sticky_overflow_and_terminal_no_callback_is_pure() -> None:
    evidence = Stage2Evidence(post_invalid_callback_count=DIAGNOSTIC_COUNTER_MAX)
    _record_bounded_counter(evidence, "post_invalid_callback_count")
    assert evidence.post_invalid_callback_count == DIAGNOSTIC_COUNTER_MAX
    assert evidence.post_invalid_counter_overflow
    assert not _terminal_no_callback(None, 0)
    assert _terminal_no_callback(1.0, 0)
    assert not _terminal_no_callback(1.0, 1)


def test_overflow_fails_closed_even_when_direct_evidence_is_otherwise_complete() -> None:
    evidence = Stage2Evidence(
        valid_v4_at=1.0, pp_debug_at=1.1, positive_pp_command_at=1.15,
        positive_envelope_at=1.2, mux_pp_at=1.25, positive_final_at=1.3,
        mux_selected_input_stamp=(1, 1), mux_output_stamp=(1, 2),
        invalid_v4_at=2.0, invalid_envelope_at=2.1, invalid_pp_stamp=(2, 1),
        invalid_matched_pp_at=2.1, zero_pp_command_at=2.1, mux_stop_at=2.2,
        mux_stop_output_stamp=(2, 2), mux_final_exact_joined=True,
        mux_evaluated_pp_command_stamp=(2, 1), mux_evaluated_pp_envelope_stamp=(2, 1),
        mux_rejected_for_invalid_tracking=True, mux_rejection_kind="invalid_tracking",
        mux_final_stop_origin="invalid_tracking_selector_rejection",
        mux_final_stop_origin_proven=True, zero_final_at=2.15,
    )
    assert evidence.verdict() == "PASS"
    evidence.post_invalid_counter_overflow = True
    assert evidence.verdict() == "HOLD"


def test_first_stop_latch_is_immutable_while_later_exact_diagnostic_latches() -> None:
    evidence = Stage2Evidence()
    first = MuxStopPayload((1, 1), "timeout", "timeout", False, "", (1, 1), (1, 1), False, "")
    exact = MuxStopPayload((2, 2), "direct", "invalid_tracking_selector_rejection", True, "", (2, 2), (2, 2), True, "invalid_tracking")
    _latch_first_post_invalid_stop(evidence, first, 1.0)
    _latch_first_post_invalid_stop(evidence, exact, 2.0)
    _latch_exact_mux_diagnostic_stop(evidence, exact, 2.0)
    assert evidence.post_invalid_first_stop_output_stamp == (1, 1)
    assert evidence.post_invalid_first_stop_receipt_at == 1.0
    assert evidence.mux_diagnostic_stop_output_stamp == (2, 2)
    assert evidence.mux_diagnostic_stop_receipt_at == 2.0
    assert _classify_invalid_fixture(
        base_fresh=True, invalid_v4_count=2, pp_zero_seen=True, envelope_seen=True,
        reject_reason="trajectory_stale",
    ) == "NOT_EVALUATED"


def test_runner_wraps_only_its_owned_command_in_setsid() -> None:
    command = ["python3", "-m", "aic_test.v4_pp_mux_observer"]
    assert _setsid_command(command) == ["setsid", *command]


def test_invalid_envelope_is_exactly_bound_to_first_zero_once() -> None:
    assert _is_first_zero_envelope_candidate(
        first_zero_stamp=(4, 2), envelope_already_captured=False,
        envelope_stamp=(4, 2), tracking_usable=False, zero_command=True,
    )
    assert not _is_first_zero_envelope_candidate(
        first_zero_stamp=(4, 2), envelope_already_captured=False,
        envelope_stamp=(4, 3), tracking_usable=False, zero_command=True,
    )
    assert not _is_first_zero_envelope_candidate(
        first_zero_stamp=(4, 2), envelope_already_captured=True,
        envelope_stamp=(4, 2), tracking_usable=False, zero_command=True,
    )


def test_runner_accepts_only_bound_private_fixture_commands() -> None:
    observer = ["python3", "/repo/tools/aic_test/aic_test/v4_pp_mux_observer.py"]
    driver = ["python3", "/repo/tools/aic_test/aic_test/v4_pp_mux_driver.py"]
    launch = [
        "ros2", "launch", "aichallenge_submit_launch",
        "v4_pp_mux_stage2.launch.xml", "topic_token:=run_03",
    ]
    assert _validate_fixture_commands(observer, launch, driver, "run_03")
    assert not _validate_fixture_commands(
        observer, [*launch[:-1], "topic_token:=other"], driver, "run_03"
    )


def test_runner_terminates_owned_group_with_child() -> None:
    process = subprocess.Popen(
        ["setsid", "sh", "-c", "sleep 30 & wait"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        time.sleep(0.05)
        pgid = os.getpgid(process.pid)
        assert pgid == process.pid
        assert _terminate_owned_group(process, pgid, timeout_s=0.5) in {
            "sigint", "sigterm", "sigkill"
        }
        with pytest.raises(ProcessLookupError):
            os.killpg(pgid, 0)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, 9)
            process.wait(timeout=1.0)


def test_observer_requires_bounded_positive_and_invalid_chains() -> None:
    evidence = Stage2Evidence(
        valid_v4_at=1.00,
        pp_debug_at=1.10,
        positive_pp_command_at=1.15,
        positive_envelope_at=1.20,
        positive_envelope_steer=0.10,
        mux_pp_at=1.25,
        positive_final_at=1.30,
        mux_selected_input_stamp=(10, 20),
        mux_output_stamp=(10, 30),
    )
    assert evidence.positive_complete()
    assert evidence.verdict() == "HOLD"

    evidence.mux_pp_at = 1.05
    assert not evidence.positive_complete()
    evidence.mux_pp_at = 1.25

    evidence.invalid_v4_at = 2.00
    evidence.zero_pp_command_at = 2.10
    evidence.invalid_matched_pp_at = 2.10
    evidence.invalid_envelope_at = 2.12
    evidence.invalid_pp_stamp = (20, 10)
    evidence.mux_stop_at = 2.15
    evidence.mux_stop_output_stamp = (20, 20)
    evidence.mux_final_exact_joined = True
    evidence.zero_final_at = 2.14
    evidence.mux_evaluated_pp_command_stamp = (20, 10)
    evidence.mux_evaluated_pp_envelope_stamp = (20, 10)
    evidence.mux_rejected_for_invalid_tracking = True
    evidence.mux_rejection_kind = "invalid_tracking"
    evidence.mux_final_stop_origin = "invalid_tracking_selector_rejection"
    evidence.mux_final_stop_origin_proven = True
    evidence.mux_stop_reason = "pure_pursuit_tracking_unusable"
    assert evidence.invalid_complete()
    assert evidence.verdict() == "PASS"

    evidence.zero_final_at = 2.00 + CAUSAL_WINDOW_S + 0.01
    assert not evidence.invalid_complete()
    assert evidence.verdict() == "HOLD"


@pytest.mark.parametrize(
    "mutation",
    [
        pytest.param(
            lambda evidence: (
                setattr(evidence, "mux_stop_reason", "pure_pursuit_cmd_timeout"),
                setattr(evidence, "mux_rejected_for_invalid_tracking", False),
            ),
            id="watchdog-timeout-without-selector-origin",
        ),
        pytest.param(
            lambda evidence: (
                setattr(evidence, "mux_stop_reason", "pure_pursuit_cmd_timeout"),
                setattr(
                    evidence,
                    "mux_rejection_kind",
                    "current_envelope_stale",
                ),
            ),
            id="stale-timeout-is-not-invalid-tracking",
        ),
        pytest.param(
            lambda evidence: (
                setattr(
                    evidence,
                    "mux_final_stop_origin",
                    "pure_pursuit_cmd_timeout",
                ),
                setattr(evidence, "mux_final_stop_origin_proven", False),
                setattr(
                    evidence,
                    "mux_stop_context",
                    "timeout_with_invalid_tracking_selector_rejection",
                ),
            ),
            id="all-exact-selector-context-with-unproven-timeout-origin",
        ),
        pytest.param(
            lambda evidence: setattr(
                evidence, "mux_evaluated_pp_command_stamp", (20, 11)
            ),
            id="evaluated-command-stamp-mismatch",
        ),
        pytest.param(
            lambda evidence: (
                setattr(evidence, "mux_stop_reason", "pure_pursuit_cmd_timeout"),
                setattr(evidence, "mux_evaluated_pp_envelope_stamp", None),
            ),
            id="null-timeout-evaluated-envelope-stamp",
        ),
        pytest.param(
            lambda evidence: setattr(evidence, "mux_final_exact_joined", False),
            id="final-output-stamp-does-not-match",
        ),
    ],
)
def test_observer_rejects_adversarial_invalid_causality(
    mutation,
) -> None:
    """Never turn timeout/reordered/null evidence into invalid-PP PASS."""
    evidence = Stage2Evidence(
        valid_v4_at=1.00,
        pp_debug_at=1.10,
        positive_pp_command_at=1.15,
        positive_envelope_at=1.20,
        positive_envelope_steer=0.10,
        mux_pp_at=1.25,
        positive_final_at=1.30,
        mux_selected_input_stamp=(10, 20),
        mux_output_stamp=(10, 30),
        invalid_v4_at=2.00,
        invalid_envelope_at=2.12,
        invalid_pp_stamp=(20, 10),
        invalid_matched_pp_at=2.10,
        zero_pp_command_at=2.10,
        mux_stop_at=2.15,
        mux_stop_output_stamp=(20, 20),
        mux_final_exact_joined=True,
        mux_stop_reason="pure_pursuit_tracking_unusable",
        mux_evaluated_pp_command_stamp=(20, 10),
        mux_evaluated_pp_envelope_stamp=(20, 10),
        mux_rejected_for_invalid_tracking=True,
        mux_rejection_kind="invalid_tracking",
        mux_final_stop_origin="invalid_tracking_selector_rejection",
        mux_final_stop_origin_proven=True,
        zero_final_at=2.14,
    )
    assert evidence.invalid_complete()
    mutation(evidence)
    assert not evidence.invalid_complete()
    assert evidence.verdict() == "HOLD"


def test_stop_payload_normalizer_retains_watchdog_with_null_evaluated_stamps() -> None:
    payload = _normalize_mux_stop_payload(
        {
            "selected_source": "stop",
            "output_speed_mps": 0.0,
            "reason": "pure_pursuit_cmd_timeout",
            "final_stop_origin": "pure_pursuit_cmd_timeout",
        }
    )
    assert payload == MuxStopPayload(
        output_stamp=None,
        reason="pure_pursuit_cmd_timeout",
        final_stop_origin="pure_pursuit_cmd_timeout",
        final_stop_origin_proven=False,
        stop_context="",
        evaluated_command_stamp=None,
        evaluated_envelope_stamp=None,
        selection_rejected_for_invalid_tracking=False,
        selection_rejection_kind="",
    )


def test_stop_payload_normalizer_requires_explicit_final_invalid_origin() -> None:
    base = {
        "selected_source": "stop",
        "output_speed_mps": 0.0,
        "reason": "pure_pursuit_cmd_timeout",
        "output_stamp_sec": 20,
        "output_stamp_nanosec": 20,
        "evaluated_pure_pursuit_command_stamp_sec": 20,
        "evaluated_pure_pursuit_command_stamp_nanosec": 10,
        "evaluated_pure_pursuit_envelope_command_stamp_sec": 20,
        "evaluated_pure_pursuit_envelope_command_stamp_nanosec": 10,
        "evaluated_pure_pursuit_selection_rejected_for_invalid_tracking": True,
        "evaluated_pure_pursuit_rejection_kind": "invalid_tracking",
        "final_stop_origin_proven": True,
    }
    generic = _normalize_mux_stop_payload(
        {**base, "final_stop_origin": "pure_pursuit_cmd_timeout"}
    )
    explicit = _normalize_mux_stop_payload(
        {**base, "final_stop_origin": "invalid_tracking_selector_rejection"}
    )
    assert generic is not None
    assert generic.final_stop_origin != "invalid_tracking_selector_rejection"
    assert explicit is not None
    assert explicit.final_stop_origin == "invalid_tracking_selector_rejection"
    assert _is_direct_invalid_stop_candidate(generic, (20, 10)) is False
    assert _is_direct_invalid_stop_candidate(explicit, (20, 10)) is True


def test_timeout_selector_context_is_not_a_direct_origin_transition() -> None:
    stop = MuxStopPayload(
        output_stamp=(20, 20),
        reason="pure_pursuit_cmd_timeout",
        final_stop_origin="pure_pursuit_cmd_timeout",
        final_stop_origin_proven=False,
        stop_context="timeout_with_invalid_tracking_selector_rejection",
        evaluated_command_stamp=(20, 10),
        evaluated_envelope_stamp=(20, 10),
        selection_rejected_for_invalid_tracking=True,
        selection_rejection_kind="invalid_tracking",
    )
    assert _is_direct_invalid_stop_candidate(stop, (20, 10)) is False
    assert _is_exact_invalid_selector_context_candidate(stop, (20, 10)) is True

    evidence = Stage2Evidence(
        mux_diagnostic_stop_reason="pure_pursuit_cmd_timeout",
        mux_diagnostic_stop_context=(
            "timeout_with_invalid_tracking_selector_rejection"
        ),
        mux_diagnostic_evaluated_command_stamp=(20, 10),
        mux_diagnostic_evaluated_envelope_stamp=(20, 10),
        mux_diagnostic_rejected_for_invalid_tracking=True,
        mux_diagnostic_rejection_kind="invalid_tracking",
        mux_diagnostic_final_stop_origin="pure_pursuit_cmd_timeout",
        mux_diagnostic_final_stop_origin_proven=False,
    )
    assert evidence.verdict() == "HOLD"


def test_direct_candidate_scan_skips_generic_o1_and_selects_future_explicit_o2() -> None:
    generic_o1 = MuxStopPayload(
        output_stamp=(20, 20),
        reason="pure_pursuit_cmd_timeout",
        final_stop_origin="pure_pursuit_cmd_timeout",
        final_stop_origin_proven=False,
        stop_context="timeout_with_invalid_tracking_selector_rejection",
        evaluated_command_stamp=(20, 10),
        evaluated_envelope_stamp=(20, 10),
        selection_rejected_for_invalid_tracking=True,
        selection_rejection_kind="invalid_tracking",
    )
    explicit_o2 = MuxStopPayload(
        output_stamp=(20, 30),
        reason="future_explicit_invalid_tracking",
        final_stop_origin="invalid_tracking_selector_rejection",
        final_stop_origin_proven=True,
        stop_context="",
        evaluated_command_stamp=(20, 10),
        evaluated_envelope_stamp=(20, 10),
        selection_rejected_for_invalid_tracking=True,
        selection_rejection_kind="invalid_tracking",
    )
    assert (
        _select_direct_invalid_stop_candidate((generic_o1, explicit_o2), (20, 10))
        == explicit_o2
    )


def test_observer_preserves_nonzero_steering_sign() -> None:
    assert _same_sign(0.1, 0.2)
    assert _same_sign(-0.1, -0.2)
    assert not _same_sign(0.1, -0.2)
    assert not _same_sign(0.0, 0.2)


def _create_publisher_count(path: Path) -> int:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return sum(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_publisher"
        for node in ast.walk(tree)
    )


def test_driver_is_only_fixture_publisher() -> None:
    assert _create_publisher_count(DRIVER_SOURCE) == 7
    assert _create_publisher_count(OBSERVER_SOURCE) == 0
    observer_text = OBSERVER_SOURCE.read_text(encoding="utf-8")
    assert "observer_authority_publisher_count\": 0" in observer_text


def test_test_only_launch_is_private_and_enables_exact_stage2_gate() -> None:
    ET.parse(LAUNCH_PATH)
    text = LAUNCH_PATH.read_text(encoding="utf-8")
    assert 'value="/aic_test/v4_pp_mux_stage2/$(var topic_token)"' in text
    assert '<param name="primary_source" value="pure_pursuit"/>' in text
    assert '<param name="require_safety_constraint" value="true"/>' in text
    assert (
        '<arg name="state_lattice_v4_poc_command_activation_enabled" value="true"/>'
        in text
    )
    assert (
        '<arg name="state_lattice_v4_poc_identity_gate_enabled" value="false"/>'
        in text
    )
    assert "/control/command/control_cmd" not in text
    assert "/overtake/plan" not in text
    assert "/overtake/safety_constraint" not in text
    assert "/awsim/state" not in text
    assert "/mpc/predicted_horizon" not in text
    assert "/vehicle/status/steering_status" not in text
    assert "/wall_recovery" not in text
