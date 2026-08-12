from __future__ import annotations

import ast
import hashlib
import json
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
    CAPTURE_CLOSE_QUIET_SEC,
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
    _capture_close_ready,
    _first_valid_marker_observed_at,
    classify_latest_sample_capture,
    classify_latest_sample_artifacts,
    write_latest_sample_selector_manifest,
    _validate_private_root as observer_private_root_valid,
)
from aic_test.v4_pp_mux_stage2_runner import (
    ACCEPTABLE_LATEST_SAMPLE_TERMINALS,
    _claim_fields,
    _build_observer_command,
    _setsid_command,
    _terminate_owned_group,
    _validate_fixture_commands,
)
from aic_test.installed_artifact_attestation import (
    REQUIRED_STAGE2_ROLES,
    InstalledRoleSpec,
    attest_stage2_installed_snapshot,
    attest_installed_roles,
    inspect_host_installed_snapshot,
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
    capture_path = Path("/tmp/aic-test/mux.json")
    assert _validate_fixture_commands(
        observer,
        [
            *launch,
            f"mux_capture_output_path:={capture_path}",
            "mux_selector_manifest_sha256:=" + "a" * 64,
            "mux_capture_epoch_nonce:=" + "b" * 32,
            "mux_capture_epoch:=1",
        ],
        driver,
        "run_03",
        capture_path,
        "a" * 64,
        "b" * 32,
        1,
    )


def test_observer_command_keeps_nonce_and_close_marker_distinct(tmp_path: Path) -> None:
    nonce = "a" * 32
    marker = tmp_path / "driver-complete.json"
    command = _build_observer_command(
        ["python3", "/repo/v4_pp_mux_observer.py"],
        "/aic_test/v4_pp_mux_stage2/run_03",
        20.0,
        tmp_path / "observer.json",
        tmp_path / "ready.json",
        nonce,
        marker,
    )
    assert command[command.index("--capture-epoch-nonce") + 1] == nonce
    assert command[command.index("--capture-close-marker") + 1] == str(marker)


def test_capture_close_drain_waits_for_pending_envelopes_after_marker() -> None:
    marker_at = 10.0
    assert _first_valid_marker_observed_at(
        None, marker_valid=True, now_sec=marker_at
    ) == marker_at
    assert _first_valid_marker_observed_at(
        marker_at, marker_valid=True, now_sec=10.05
    ) == marker_at
    assert not _capture_close_ready(
        marker_observed_at=marker_at, last_envelope_at=None, now_sec=marker_at
    )
    assert not _capture_close_ready(
        marker_observed_at=marker_at, last_envelope_at=10.08, now_sec=10.15
    )
    assert _capture_close_ready(
        marker_observed_at=marker_at,
        last_envelope_at=10.08,
        now_sec=10.08 + CAPTURE_CLOSE_QUIET_SEC + 0.001,
    )
    assert _capture_close_ready(
        marker_observed_at=marker_at,
        last_envelope_at=None,
        now_sec=marker_at + CAPTURE_CLOSE_QUIET_SEC + 0.001,
    )


def test_target_agnostic_capture_classification_is_manifest_hash_bound(tmp_path: Path) -> None:
    manifest_path = tmp_path / "selector.json"
    mux_path = tmp_path / "mux.json"
    pp_path = tmp_path / "pp.json"
    root = "/aic_test/v4_pp_mux_stage2/run_03"
    nonce = "a" * 32
    manifest_sha256 = write_latest_sample_selector_manifest(
        manifest_path, root, nonce
    )
    target = [7, 10, 100, 3]
    successor = [7, 11, 110, 3]
    mux_path.write_text(
        json.dumps(
            {
                "latest_sample_schema_version": 1,
                "latest_sample_observability_enabled": True,
                "sealed": True,
                "measurement_faults": 0,
                "overflow": False,
                "callback_overflow": False,
                "cycle_overflow": False,
                "latest_sample_capture": {
                    "closed": True,
                    "late_entry": False,
                    "selector_manifest_sha256": manifest_sha256,
                    "capture_epoch_nonce": nonce,
                    "epoch": 1,
                    "expected_capture_epoch": 1,
                },
                "latest_sample_callback_records": [
                    [1, 1, target, True, "ok", "inserted", None, target, "advanced", False],
                    [2, 2, successor, True, "ok", "inserted", target, successor, "advanced", False],
                ],
                "latest_sample_cycle_records": [
                    [1, 3, successor, successor, "selected", "stop", "timeout", False, False]
                ],
            }
        )
    )
    pp_path.write_text(
        json.dumps(
            {
                "private_root": root,
                "capture_epoch_nonce": nonce,
                "evidence": {
                    "selector_candidate_window_closed": True,
                    "selector_capture_close_marker_observed": True,
                    "selector_candidate_window_opened_at": 1.0,
                    "target_published_after_arm": True,
                    "selector_candidate_overflow": False,
                    "selector_candidate_duplicate_conflict": False,
                    "selector_eligible_candidate_count": 1,
                    "selector_eligible_candidates": [
                        {
                            "identity": target,
                            "payload_sha256": "b" * 64,
                            "published_after_arm": True,
                        }
                    ],
                },
            }
        )
    )
    result = classify_latest_sample_artifacts(
        selector_manifest_path=manifest_path, mux_capture_path=mux_path, pp_result_path=pp_path
    )
    assert result["terminal"] == "HOLD_SOURCE_HASH_UNBOUND"
    assert result["selector_manifest_sha256"] == manifest_sha256

    manifest = json.loads(manifest_path.read_text())
    manifest["expected_source_hashes"] = {"fixture": "c" * 64}
    manifest_path.write_text(json.dumps(manifest))
    capture = json.loads(mux_path.read_text())
    capture["latest_sample_capture"]["selector_manifest_sha256"] = (
        hashlib.sha256(
            json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
    )
    mux_path.write_text(json.dumps(capture))
    pp_result = json.loads(pp_path.read_text())
    assert classify_latest_sample_capture(
        selector_manifest=manifest, mux_capture=capture, pp_result=pp_result
    ) == "RECEIVED_AND_SUPERSEDED"

    capture["latest_sample_capture"]["selector_manifest_sha256"] = "0" * 64
    mux_path.write_text(json.dumps(capture))
    assert classify_latest_sample_artifacts(
        selector_manifest_path=manifest_path, mux_capture_path=mux_path, pp_result_path=pp_path
    )["terminal"] == "INDETERMINATE_TRACE_LOSS"


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        (lambda manifest, capture, result: result["evidence"].update({"selector_eligible_candidate_count": 0, "selector_eligible_candidates": []}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: result["evidence"].update({"selector_eligible_candidate_count": 2, "selector_eligible_candidates": result["evidence"]["selector_eligible_candidates"] * 2}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: result["evidence"].update({"selector_candidate_duplicate_conflict": True}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: result.update({"private_root": "/aic_test/v4_pp_mux_stage2/other"}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: capture["latest_sample_capture"].update({"capture_epoch_nonce": "b" * 32}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: capture["latest_sample_capture"].update({"epoch": 2}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: capture.update({"sealed": False}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: result["evidence"].update({"selector_capture_close_marker_observed": False}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: capture.update({"latest_sample_callback_records": [[2, 1, [7, 10, 100, 3], True, "ok", "inserted", None, [7, 10, 100, 3], "advanced", False]]}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: capture.update({"latest_sample_callback_records": [[1, 1]]}), "INDETERMINATE_TRACE_LOSS"),
        (lambda manifest, capture, result: capture.update({"latest_sample_callback_records": [[1, 1, [7, 10, 100, 3], True, "duplicate_conflict", "duplicate_conflict", None, [7, 10, 100, 3], "unchanged", False]]}), "INDETERMINATE_TRACE_LOSS"),
    ],
)
def test_target_selector_fails_closed_for_ambiguous_or_incomplete_artifacts(mutation, expected) -> None:
    root = "/aic_test/v4_pp_mux_stage2/run_04"
    nonce = "d" * 32
    target = [7, 10, 100, 3]
    manifest = {
        "schema_version": 1, "private_root": root,
        "selector_rule": "first_invalid_zero_envelope_after_invalid_v4",
        "selector_source": "pp_observer", "capture_epoch_nonce": nonce,
    }
    capture = {
        "latest_sample_schema_version": 1, "latest_sample_observability_enabled": True,
        "sealed": True, "measurement_faults": 0, "overflow": False,
        "callback_overflow": False, "cycle_overflow": False,
        "latest_sample_capture": {"closed": True, "late_entry": False, "selector_manifest_sha256": "a" * 64, "capture_epoch_nonce": nonce, "epoch": 1, "expected_capture_epoch": 1},
        "latest_sample_callback_records": [[1, 1, target, True, "ok", "inserted", None, target, "advanced", False]],
        "latest_sample_cycle_records": [[1, 2, target, target, "selected", "stop", "x", False, False]],
    }
    result = {
        "private_root": root, "capture_epoch_nonce": nonce,
        "evidence": {
            "selector_candidate_window_closed": True, "target_published_after_arm": True,
            "selector_capture_close_marker_observed": True,
            "selector_candidate_window_opened_at": 1.0,
            "selector_candidate_overflow": False, "selector_candidate_duplicate_conflict": False,
            "selector_eligible_candidate_count": 1,
            "selector_eligible_candidates": [{"identity": target, "payload_sha256": "e" * 64, "published_after_arm": True}],
        },
    }
    mutation(manifest, capture, result)
    assert classify_latest_sample_capture(
        selector_manifest=manifest, mux_capture=capture, pp_result=result
    ) == expected


def test_selector_is_pp_only_and_runner_terminal_allowlist_is_conservative(tmp_path: Path) -> None:
    manifest_path = tmp_path / "selector.json"
    write_latest_sample_selector_manifest(
        manifest_path, "/aic_test/v4_pp_mux_stage2/run_05", "f" * 32
    )
    manifest = json.loads(manifest_path.read_text())
    assert manifest["selector_source"] == "pp_observer"
    assert "mux" not in manifest["selector_rule"]
    assert ACCEPTABLE_LATEST_SAMPLE_TERMINALS == {"RECEIVED_AND_EVALUATED"}
    assert "NOT_OBSERVED_IN_COMPLETE_MUX_TRACE" not in ACCEPTABLE_LATEST_SAMPLE_TERMINALS
    assert "NONQUALIFYING" not in ACCEPTABLE_LATEST_SAMPLE_TERMINALS


@pytest.mark.parametrize(
    ("terminal", "runner_success", "evaluated"),
    [
        ("RECEIVED_AND_EVALUATED", True, True),
        ("RECEIVED_AND_SUPERSEDED", False, False),
        ("NONQUALIFYING", False, False),
    ],
)
def test_runner_claims_never_upgrade_superseded_receipt(
    terminal: str, runner_success: bool, evaluated: bool
) -> None:
    claims = _claim_fields(terminal)
    assert claims["runner_success"] is runner_success
    assert claims["pp_command_evaluated"] is evaluated
    assert claims["final_mux_use_proven"] is False
    assert claims["vehicle_control_proven"] is False
    assert claims["stage2_style_pass"] is False


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize(
    ("configure", "expected"),
    [
        (lambda root, path, digest: ([InstalledRoleSpec("role", path, root, "unknown", digest, "installed_bytes")]), "HOLD_UNKNOWN_ARTIFACT_KIND"),
        (lambda root, path, digest: ([InstalledRoleSpec("role", path, root, "installed_script", None, None)]), "HOLD_SOURCE_HASH_UNBOUND"),
        (lambda root, path, digest: ([InstalledRoleSpec("role", path, root, "installed_script", digest, "workspace_source")]), "HOLD_SOURCE_HASH_UNBOUND"),
        (lambda root, path, digest: ([InstalledRoleSpec("role", root / "missing", root, "installed_script", digest, "installed_bytes")]), "HOLD_MISSING_ARTIFACT"),
        (lambda root, path, digest: ([InstalledRoleSpec("role", root / "escape", root, "installed_script", digest, "installed_bytes")]), "HOLD_ROOT_ESCAPE"),
        (lambda root, path, digest: ([InstalledRoleSpec("role", root / "cycle", root, "installed_script", digest, "installed_bytes")]), "HOLD_SYMLINK_CYCLE"),
        (lambda root, path, digest: ([InstalledRoleSpec("role", path, root, "installed_script", "0" * 64, "installed_bytes")]), "HOLD_DIGEST_MISMATCH"),
        (lambda root, path, digest: ([InstalledRoleSpec("role_a", path, root, "installed_script", digest, "installed_bytes"), InstalledRoleSpec("role_b", path, root, "installed_script", digest, "installed_bytes")]), "HOLD_DUPLICATE_CANONICAL_ROLE"),
        (lambda root, path, digest: ([InstalledRoleSpec("role", path, root, "executable", digest, "installed_bytes")]), "HOLD_BINARY_KIND_MISUSE"),
    ],
)
def test_installed_attestation_rejects_unbound_or_ambiguous_roles(
    tmp_path: Path, configure, expected: str
) -> None:
    root = tmp_path / "install"
    root.mkdir()
    artifact = root / "entry.py"
    artifact.write_text("#!/usr/bin/env python3\n", encoding="utf-8")
    (root / "escape").symlink_to(tmp_path / "outside")
    (root / "cycle").symlink_to("cycle")
    result = attest_installed_roles(configure(root, artifact, _digest(artifact)))
    assert result["terminal"] == expected
    assert all(role["terminal"] == "MATCH" or role["terminal"].startswith("HOLD_") for role in result["roles"])


def test_installed_attestation_resolves_bounded_alias_to_real_host_bytes(tmp_path: Path) -> None:
    root = tmp_path / "install"
    root.mkdir()
    artifact = root / "entry.py"
    artifact.write_text("#!/usr/bin/env python3\nprint('installed')\n", encoding="utf-8")
    alias = root / "entry"
    alias.symlink_to(artifact.name)
    spec = InstalledRoleSpec("mux_entry", alias, root, "installed_script", _digest(artifact), "installed_bytes")
    result = inspect_host_installed_snapshot([spec])
    assert result["read_only"] is True
    assert result["terminal"] == "READY_FOR_EXTERNAL_REVIEW_ONLY"
    assert result["roles"][0]["canonical_path"] == str(artifact)


def test_workspace_source_is_recorded_but_cannot_attest_an_installed_role(tmp_path: Path) -> None:
    source = tmp_path / "source.py"
    source.write_text("print('source')\n", encoding="utf-8")
    result = inspect_host_installed_snapshot([
        InstalledRoleSpec("mux_source", source, tmp_path, "workspace_source", _digest(source), "workspace_source")
    ])
    assert result["terminal"] == "HOLD_SOURCE_HASH_UNBOUND"
    assert result["roles"][0]["terminal"] == "SOURCE_ONLY_MATCH"
    assert result["roles"][0]["binding_scope"] == "workspace_source_only"


def test_unbound_role_retains_observed_host_digest_and_terminal_order_is_stable(tmp_path: Path) -> None:
    root = tmp_path / "install"
    root.mkdir()
    artifact = root / "entry.py"
    artifact.write_text("print('installed')\n", encoding="utf-8")
    unbound = InstalledRoleSpec("script", artifact, root, "installed_script", None, None)
    mismatch = InstalledRoleSpec("mismatch", artifact, root, "installed_script", "0" * 64, "installed_bytes")
    forward = attest_installed_roles([unbound, mismatch])
    reverse = attest_installed_roles([mismatch, unbound])
    assert forward["roles"][0]["canonical_path"] == str(artifact)
    assert forward["roles"][0]["observed_digest"] == _digest(artifact)
    assert forward["terminal"] == reverse["terminal"] == "HOLD_DUPLICATE_CANONICAL_ROLE"


def test_stage2_preflight_requires_complete_role_set_and_preserves_missing_primary(tmp_path: Path) -> None:
    artifact = tmp_path / "artifact.py"
    artifact.write_text("x\n", encoding="utf-8")
    subset = [InstalledRoleSpec("mux_entry", artifact, tmp_path, "installed_script", _digest(artifact), "installed_bytes")]
    assert attest_stage2_installed_snapshot(subset)["terminal"] == "HOLD_SOURCE_HASH_UNBOUND"
    specs = [InstalledRoleSpec(role, tmp_path / role, tmp_path, "installed_script", "0" * 64, "installed_bytes") for role in REQUIRED_STAGE2_ROLES]
    assert attest_stage2_installed_snapshot(specs)["terminal"] == "HOLD_MISSING_ARTIFACT"


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
    assert '<param name="mux_runtime_measurement_enabled" value="false"/>' in text
    assert '<param name="latest_sample_observability_enabled" value="true"/>' in text
    assert "latest_sample_target_" not in text
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
