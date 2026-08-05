from __future__ import annotations

import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from aic_test import cli
from aic_test import v2_uptake_observer as observer
from aic_test.v2_uptake_observer import (
    FinalFenceDrainState,
    _write_terminal_json_atomic,
    final_fence_sha256,
    identity_key,
    quiesce_request_sha256,
    terminal_payload_hash_valid,
)


QOS_FINGERPRINT = (
    "topic=/planning/overtake/state_lattice/v2_proposal;"
    "type=multi_purpose_mpc_ros_msgs/msg/AuthorizedCartesianTrajectoryV2;"
    "reliability=reliable;durability=volatile;history=keep_last;depth=8"
)

# Cross-language canonical known-answer contract, mirrored verbatim by
# test_state_lattice_v2_final_fence.cpp:
# v1, execution-1, producer 7101, session 9, epoch 9, request 1.
QUIESCE_REQUEST_KNOWN_ANSWER_SHA256 = (
    "754b853638c404fa4fdb10677e56fb445e4632873088a05432760d7441a52a06"
)

# Cross-language nonzero-final-identity fence known-answer contract, mirrored
# verbatim by test_state_lattice_v2_final_fence.cpp. Fields are:
# execution-1, producer 7101, session/epoch 9, F/count 1, identity sequence 1,
# generation 1/1, source stamp 1s+0ns, map, digest 01+31x00, publish mono 101,
# quiesced/fence mono 200/201, and QOS_FINGERPRINT.
FINAL_FENCE_KNOWN_ANSWER_SHA256 = (
    "6fb43a62105c9e338bfb506c04c9326dcd69e4025a96d7dcf30cabce8495abe4"
)


def identity(ordinal: int) -> SimpleNamespace:
    return SimpleNamespace(
        producer_instance_id="7101",
        session_id="1",
        proposal_sequence=ordinal,
        plan_generation=ordinal,
        source_generation=1,
        source_stamp=SimpleNamespace(sec=1, nanosec=ordinal),
        frame_id="map",
        canonical_sha256=bytes([(ordinal % 255) or 1]) + bytes(31),
        publish_monotonic_ns=100 + ordinal,
    )


def state() -> FinalFenceDrainState:
    return FinalFenceDrainState(
        execution_nonce="execution-1",
        producer_instance_id="7101",
        session_id="1",
        sealed_epoch_id=1,
        proposal_qos_fingerprint=QOS_FINGERPRINT,
    )


def fence(target: FinalFenceDrainState, ordinal: int) -> SimpleNamespace:
    message = SimpleNamespace(
        schema_version=1,
        execution_nonce="execution-1",
        producer_instance_id="7101",
        session_id="1",
        sealed_epoch_id=1,
        fence_id=1,
        request_id=1,
        request_canonical_sha256=target.request_digest,
        final_committed_ordinal=ordinal,
        successful_emission_count=ordinal,
        final_identity_present=ordinal != 0,
        final_identity=identity(ordinal if ordinal else 1),
        proposal_qos_fingerprint=QOS_FINGERPRINT,
        quiesced_monotonic_ns=1000,
        fence_publish_monotonic_ns=1001,
        canonical_sha256=bytes(32),
    )
    message.canonical_sha256 = final_fence_sha256(message)
    return message


def arm(target: FinalFenceDrainState, terminal_pp_cycle: int = 40) -> None:
    target.freeze_pp_boundary(terminal_pp_cycle)
    request = target.make_request_fields()
    assert request is not None
    assert request["request_id"] == 1


def test_request_canonicalization_is_deterministic_and_field_bound() -> None:
    fields = {
        "schema_version": 1,
        "execution_nonce": "execution-1",
        "expected_producer_instance_id": "7101",
        "expected_session_id": "1",
        "sealed_epoch_id": 1,
        "request_id": 1,
    }
    assert quiesce_request_sha256(**fields) == quiesce_request_sha256(**fields)
    assert quiesce_request_sha256(**fields) != quiesce_request_sha256(
        **{**fields, "sealed_epoch_id": 2}
    )


def test_cross_language_quiesce_request_known_answer() -> None:
    digest = quiesce_request_sha256(
        schema_version=1,
        execution_nonce="execution-1",
        expected_producer_instance_id="7101",
        expected_session_id="9",
        sealed_epoch_id=9,
        request_id=1,
    )
    assert digest.hex() == QUIESCE_REQUEST_KNOWN_ANSWER_SHA256


def test_cross_language_nonzero_identity_final_fence_known_answer() -> None:
    final_identity = SimpleNamespace(
        producer_instance_id="7101",
        session_id="9",
        proposal_sequence=1,
        plan_generation=1,
        source_generation=1,
        source_stamp=SimpleNamespace(sec=1, nanosec=0),
        frame_id="map",
        canonical_sha256=bytes([1]) + bytes(31),
        publish_monotonic_ns=101,
    )
    message = SimpleNamespace(
        schema_version=1,
        execution_nonce="execution-1",
        producer_instance_id="7101",
        session_id="9",
        sealed_epoch_id=9,
        fence_id=1,
        request_id=1,
        request_canonical_sha256=bytes.fromhex(
            QUIESCE_REQUEST_KNOWN_ANSWER_SHA256
        ),
        final_committed_ordinal=1,
        successful_emission_count=1,
        final_identity_present=True,
        final_identity=final_identity,
        proposal_qos_fingerprint=QOS_FINGERPRINT,
        quiesced_monotonic_ns=200,
        fence_publish_monotonic_ns=201,
    )
    assert final_fence_sha256(message).hex() == FINAL_FENCE_KNOWN_ANSWER_SHA256


def test_fence_before_delayed_in_scope_ordinal_drains_by_ordinal() -> None:
    target = state()
    target.record_pp_lifecycle(identity_key(identity(1)), pp_cycle_sequence=30)
    target.record_pp_lifecycle(identity_key(identity(2)), pp_cycle_sequence=31)
    target.record_proposal(
        identity_key(identity(1)), original_time_predicate_valid=True
    )
    arm(target)
    target.record_fence(fence(target, 2))
    target.record_proposal(
        identity_key(identity(2)), original_time_predicate_valid=True
    )
    result = target.close(pp_lifecycle_closed_through=40)
    assert result["valid"] is True
    assert result["observed_ordinals"] == [1, 2]
    assert result["frozen_pp_cycle_sequence"] == 40
    assert result["deadline_repair_permitted"] is False


@pytest.mark.parametrize(
    ("mutation", "expected_fault"),
    (
        ("greater_than_fence", "proposal_ordinal_greater_than_fence"),
        ("missing_ordinal", "fenced_ordinal_set_incomplete"),
        ("duplicate_fence", "final_fence_multiplicity_invalid"),
        ("wrong_nonce", "final_fence_invalid"),
        ("pp_unresolved", "terminal_pp_lifecycle_unresolved"),
    ),
)
def test_terminal_faults_are_sticky_and_fail_closed(
    mutation: str, expected_fault: str
) -> None:
    target = state()
    target.record_pp_lifecycle(identity_key(identity(1)), pp_cycle_sequence=30)
    target.record_proposal(
        identity_key(identity(1)), original_time_predicate_valid=True
    )
    arm(target)
    message = fence(target, 1 if mutation != "missing_ordinal" else 2)
    if mutation == "wrong_nonce":
        message.execution_nonce = "wrong"
        message.canonical_sha256 = final_fence_sha256(message)
    target.record_fence(message)
    if mutation == "greater_than_fence":
        target.record_proposal(
            identity_key(identity(2)), original_time_predicate_valid=True
        )
    if mutation == "duplicate_fence":
        target.record_fence(message)
    pp_closed = 39 if mutation == "pp_unresolved" else 40
    result = target.close(pp_lifecycle_closed_through=pp_closed)
    assert result["valid"] is False
    assert result["fault"] == expected_fault


def test_delayed_drain_cannot_repair_original_time_predicate() -> None:
    target = state()
    target.record_pp_lifecycle(identity_key(identity(1)), pp_cycle_sequence=30)
    arm(target)
    target.record_fence(fence(target, 1))
    target.record_proposal(
        identity_key(identity(1)), original_time_predicate_valid=False
    )
    result = target.close(pp_lifecycle_closed_through=40)
    assert result["fault"] == "original_time_predicate_false"
    assert result["deadline_repair_permitted"] is False


def test_proposal_resolved_only_after_frozen_t_gets_no_credit() -> None:
    target = state()
    target.record_proposal(
        identity_key(identity(1)), original_time_predicate_valid=True
    )
    arm(target, terminal_pp_cycle=40)
    target.record_fence(fence(target, 1))
    target.record_pp_lifecycle(identity_key(identity(1)), pp_cycle_sequence=41)
    result = target.close(pp_lifecycle_closed_through=41)
    assert result["fault"] == "fenced_ordinal_pp_lifecycle_after_frozen_t"


def test_missing_pp_terminal_cannot_close_fenced_ordinal() -> None:
    target = state()
    target.record_proposal(
        identity_key(identity(1)), original_time_predicate_valid=True
    )
    arm(target)
    target.record_fence(fence(target, 1))
    assert target.close(pp_lifecycle_closed_through=40)["fault"] == (
        "fenced_ordinal_pp_lifecycle_missing"
    )


def test_wrong_pp_identity_at_t_fails_exact_join() -> None:
    target = state()
    proposal_identity = identity(1)
    wrong_identity = identity(1)
    wrong_identity.plan_generation = 2
    target.record_proposal(
        identity_key(proposal_identity), original_time_predicate_valid=True
    )
    target.record_pp_lifecycle(
        identity_key(wrong_identity), pp_cycle_sequence=30
    )
    arm(target)
    target.record_fence(fence(target, 1))
    assert target.close(pp_lifecycle_closed_through=40)["fault"] == (
        "fenced_ordinal_pp_identity_mismatch"
    )


def test_zero_fence_closes_empty_set_but_grants_no_availability_credit() -> None:
    target = state()
    arm(target)
    target.record_fence(fence(target, 0))
    result = target.close(pp_lifecycle_closed_through=40)
    assert result["valid"] is True
    assert result["final_committed_ordinal"] == 0
    assert result["observed_ordinals"] == []


@pytest.mark.parametrize(
    "mutation",
    (
        "producer",
        "session",
        "epoch",
        "request_digest",
        "qos",
        "hash",
        "final_identity",
        "capacity_plus_one",
    ),
)
def test_fence_mutation_matrix_fails_closed(mutation: str) -> None:
    target = state()
    if mutation != "capacity_plus_one":
        target.record_proposal(
            identity_key(identity(1)), original_time_predicate_valid=True
        )
        target.record_pp_lifecycle(
            identity_key(identity(1)), pp_cycle_sequence=30
        )
    arm(target)
    message = fence(target, 1)
    if mutation == "producer":
        message.producer_instance_id = "7102"
    elif mutation == "session":
        message.session_id = "2"
    elif mutation == "epoch":
        message.sealed_epoch_id = 2
    elif mutation == "request_digest":
        message.request_canonical_sha256 = bytes([7]) * 32
    elif mutation == "qos":
        message.proposal_qos_fingerprint = "wrong"
    elif mutation == "hash":
        message.canonical_sha256 = bytes([9]) * 32
    elif mutation == "final_identity":
        message.final_identity.plan_generation = 2
    elif mutation == "capacity_plus_one":
        message.final_committed_ordinal = target.capacity + 1
        message.successful_emission_count = target.capacity + 1
        message.final_identity = identity(target.capacity + 1)
    if mutation != "hash":
        message.canonical_sha256 = final_fence_sha256(message)
    target.record_fence(message)
    result = target.close(pp_lifecycle_closed_through=40)
    assert result["valid"] is False
    assert result["fault"] in {
        "final_fence_invalid",
        "final_fence_identity_proposal_mismatch",
    }


def test_duplicate_and_conflicting_proposal_ordinals_fail_closed() -> None:
    duplicate = state()
    key = identity_key(identity(1))
    duplicate.record_proposal(key, original_time_predicate_valid=True)
    duplicate.record_proposal(key, original_time_predicate_valid=True)
    assert duplicate.sticky_fault == "fence_proposal_duplicate"

    conflict = state()
    changed = identity(1)
    changed.plan_generation = 2
    conflict.record_proposal(key, original_time_predicate_valid=True)
    conflict.record_proposal(
        identity_key(changed), original_time_predicate_valid=True
    )
    assert conflict.sticky_fault == "fence_proposal_identity_conflict"


def test_terminal_result_is_exclusive_synced_and_hash_valid(tmp_path: Path) -> None:
    target = tmp_path / "result.json"
    payload = {
        "outcome": "INVALID_EVIDENCE",
        "run_nonce": "execution-1",
        "final_fence": {"final_committed_ordinal": 2},
    }
    _write_terminal_json_atomic(target, payload)
    reread = json.loads(target.read_text(encoding="utf-8"))
    assert terminal_payload_hash_valid(reread)
    reread["run_nonce"] = "changed"
    assert not terminal_payload_hash_valid(reread)
    with pytest.raises(FileExistsError):
        _write_terminal_json_atomic(target, payload)


def test_dedicated_runner_profile_is_explicit_and_normal_profile_stays_off() -> None:
    parser = cli.build_parser()
    dedicated = parser.parse_args(["run", "v2-uptake-final-fence-smoke"])
    normal = parser.parse_args(["run", "v2-uptake-smoke"])
    assert dedicated.scenario == "v2-uptake-final-fence-smoke"
    assert normal.scenario == "v2-uptake-smoke"

    source = Path(cli.__file__).read_text(encoding="utf-8")
    assert 'args.scenario == "v2-uptake-final-fence-smoke"' in source
    assert "__FINAL_FENCE_OBSERVER_ARGS__" in source
    assert "__FINAL_FENCE_PLANNER_ARGS__" in source


def test_runner_cannot_stop_before_canonical_marker_and_sidecar_validation() -> None:
    source = Path(cli.__file__).read_text(encoding="utf-8")
    block = source[
        source.index('if [[ "__FINAL_FENCE_PROFILE_BOOL__" == true ]]') :
    ]
    marker_wait = block.index("final_fence_fixed.validated.json")
    marker_hash = block.index("final_fence_marker_hash_invalid")
    sidecar_binding = block.index("final_fence_sidecar_binding_invalid")
    planner_stop = block.index("shutdown_component planner")
    assert marker_wait < marker_hash < sidecar_binding < planner_stop
    assert "quiet" not in block[:planner_stop].lower()
    assert "subscriber" not in block[:planner_stop].lower()
    assert "cleanup_only_not_delivery_proof" in Path(
        observer.__file__
    ).read_text(encoding="utf-8")


def test_cpp_terminal_fence_regressions_are_present_in_focused_source() -> None:
    repo = Path(cli.__file__).resolve().parents[3]
    cpp_test = (
        repo
        / "aichallenge/workspace/src/aichallenge_submit/"
        "state_lattice_overtake_planner/test/"
        "test_state_lattice_v2_final_fence.cpp"
    ).read_text(encoding="utf-8")
    for name in (
        "RequestAfterFenceIsTerminalAndCannotReplaceFence",
        "PostFenceAdmissionIsRejectedAndFaultsEvidence",
        "DuplicateOrConflictingRequestCannotQuiesceTwice",
        "AmbiguousFencePublicationRemainsFencedWithoutReplacement",
    ):
        assert name in cpp_test


def _assert_no_validated_sidecar(target: Path) -> None:
    assert not target.with_name(f"{target.stem}.validated.json").exists()


def test_terminal_persistence_file_fsync_failure_has_no_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "marker.json"

    def fail_fsync(_fd: int) -> None:
        raise OSError("file fsync failed")

    monkeypatch.setattr(os, "fsync", fail_fsync)
    with pytest.raises(OSError, match="file fsync failed"):
        _write_terminal_json_atomic(target, {"outcome": "PASS"})
    assert not target.exists()
    _assert_no_validated_sidecar(target)


def test_terminal_persistence_rename_failure_has_no_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "marker.json"

    def fail_rename(_source: Path, _target: Path) -> None:
        raise OSError("rename failed")

    monkeypatch.setattr(observer, "_rename_noreplace", fail_rename)
    with pytest.raises(OSError, match="rename failed"):
        _write_terminal_json_atomic(target, {"outcome": "PASS"})
    assert not target.exists()
    _assert_no_validated_sidecar(target)


def test_terminal_persistence_directory_fsync_failure_has_no_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "marker.json"
    real_fsync = os.fsync
    calls = 0

    def fail_second_fsync(fd: int) -> None:
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError("directory fsync failed")
        real_fsync(fd)

    monkeypatch.setattr(os, "fsync", fail_second_fsync)
    with pytest.raises(OSError, match="directory fsync failed"):
        _write_terminal_json_atomic(target, {"outcome": "PASS"})
    _assert_no_validated_sidecar(target)


def test_terminal_persistence_reread_failure_has_no_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "marker.json"
    monkeypatch.setattr(Path, "read_text", lambda *_args, **_kwargs: "{")
    with pytest.raises(json.JSONDecodeError):
        _write_terminal_json_atomic(target, {"outcome": "PASS"})
    _assert_no_validated_sidecar(target)


def test_terminal_persistence_hash_failure_has_no_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "marker.json"
    monkeypatch.setattr(observer, "terminal_payload_hash_valid", lambda _value: False)
    with pytest.raises(ValueError, match="reread_validation_failed"):
        _write_terminal_json_atomic(target, {"outcome": "PASS"})
    _assert_no_validated_sidecar(target)
