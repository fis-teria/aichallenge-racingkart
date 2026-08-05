from __future__ import annotations

import json
import os
import re
import stat

import pytest

from aic_test.v2_uptake_observer import (
    FORBIDDEN_AUTHORITY_TOPICS,
    LABEL,
    QUALIFIER,
    REJECT_CLOCK_FAULT,
    REQUIRED_COMPONENT_NODES,
    UptakeLedger,
    VALID_BASE_SOURCE_KINDS,
    _component_graph_evidence,
    _write_json_atomic,
    private_pp_binding_cycle_barrier_eligible,
    private_pp_startup_clean_stamp_eligible,
    private_pp_startup_clock_fault_event_eligible,
    private_pp_startup_clock_fault_summary_eligible,
    private_prelaunch_permit_eligible,
    process_private_prelaunch_request,
)


def test_required_component_graph_contract_is_explicit_and_fail_closed() -> None:
    assert REQUIRED_COMPONENT_NODES == {
        "state_lattice_overtake_planner_node",
        "simple_pure_pursuit_node",
        "aic_test_v2_direct_input_driver",
    }
    assert "/control/command/control_cmd" in FORBIDDEN_AUTHORITY_TOPICS


def test_startup_clock_fault_reason_matches_production_message_contract() -> None:
    message = (
        __import__("pathlib").Path(__file__).parents[3]
        / "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros_msgs/msg/"
        "StateLatticeV2BindingStatus.msg"
    ).read_text(encoding="utf-8")
    match = re.search(
        r"^\s*uint8\s+REJECT_CLOCK_FAULT\s*=\s*(\d+)\s*$",
        message,
        flags=re.MULTILINE,
    )
    assert match is not None
    assert int(match.group(1)) == REJECT_CLOCK_FAULT


class _Endpoint:
    def __init__(self, node_name: str, node_namespace: str = "/") -> None:
        self.node_name = node_name
        self.node_namespace = node_namespace


class _GraphNode:
    def __init__(
        self,
        extra_proposal_publisher: bool = False,
        forbidden_planner_endpoint: bool = False,
        duplicate_planner_node: bool = False,
        missing_attestation_subscriber: bool = False,
    ) -> None:
        self.extra_proposal_publisher = extra_proposal_publisher
        self.forbidden_planner_endpoint = forbidden_planner_endpoint
        self.duplicate_planner_node = duplicate_planner_node
        self.missing_attestation_subscriber = missing_attestation_subscriber

    def get_node_names_and_namespaces(self) -> list[tuple[str, str]]:
        nodes = [(name, "/") for name in REQUIRED_COMPONENT_NODES]
        if self.duplicate_planner_node:
            nodes.append(("state_lattice_overtake_planner_node", "/other"))
        return nodes

    def get_publisher_names_and_types_by_node(
        self, name: str, _: str
    ) -> list[tuple[str, list[str]]]:
        if name == "state_lattice_overtake_planner_node" and self.forbidden_planner_endpoint:
            return [("/control/command/control_cmd", ["example/msg/Command"])]
        return []

    def get_subscriber_names_and_types_by_node(
        self, name: str, _: str
    ) -> list[tuple[str, list[str]]]:
        return []

    def get_publishers_info_by_topic(self, topic: str) -> list[_Endpoint]:
        if topic == "/clock":
            return [_Endpoint("aic_test_v2_direct_input_driver")]
        if topic.endswith("v2_proposal"):
            endpoints = [_Endpoint("state_lattice_overtake_planner_node")]
            if self.extra_proposal_publisher:
                endpoints.append(_Endpoint("unexpected"))
            return endpoints
        return [_Endpoint("simple_pure_pursuit_node")]

    def get_subscriptions_info_by_topic(self, topic: str) -> list[_Endpoint]:
        if self.missing_attestation_subscriber:
            return []
        return [_Endpoint("state_lattice_overtake_planner_node")]


def test_graph_provenance_requires_exactly_one_expected_publisher() -> None:
    assert _component_graph_evidence(_GraphNode())["ready"] is True
    invalid = _component_graph_evidence(_GraphNode(extra_proposal_publisher=True))
    assert invalid["ready"] is False
    assert invalid["publisher_ownership_failures"] == [
        "/planning/overtake/state_lattice/v2_proposal"
    ]


def test_graph_provenance_requires_base_attestation_subscriber() -> None:
    invalid = _component_graph_evidence(
        _GraphNode(missing_attestation_subscriber=True)
    )
    assert invalid["ready"] is False
    assert invalid["subscriber_ownership_failures"] == [
        "/control/overtake/state_lattice/v2_base_attestation"
    ]


def test_observer_source_has_ready_seal_and_terminal_graph_lifecycle() -> None:
    source = __import__("pathlib").Path(__file__).parents[1] / "aic_test" / "v2_uptake_observer.py"
    text = source.read_text(encoding="utf-8")
    assert '"observer_ready": True' in text
    assert '"graph_sealed": True' in text
    assert "terminal_graph_file" in text
    assert "sealed_graph_evidence" in text
    assert "terminal_graph_evidence" in text
    assert 'STATUS_TOPIC = "/debug/overtake/state_lattice/v2_binding_status"' in text
    assert 'CLOCK_TOPIC: "aic_test_v2_direct_input_driver"' in text
    assert '"proposal_count_at_barrier": ledger.proposal_count' in text
    assert '"normal_status_integrity": True' in text
    assert '"pp_binding_cycle_missing"' in text
    assert "private_pp_binding_cycle_barrier_eligible(" in text
    assert text.count("and not ledger.reasons") >= 2
    assert "node.create_subscription(\n        StateLatticeV2BindingStatus, STATUS_TOPIC, on_status, v2_qos\n    )" in text
    assert "node.create_subscription(String" not in text
    assert "graph_seal_deadline = time.monotonic() + graph_seal_timeout_s" in text
    assert text.index("graph_seal_deadline =") < text.index("capture_deadline =")


def _clean_binding_cycle_barrier(ledger: UptakeLedger, **overrides: object) -> bool:
    fields: dict[str, object] = {
        "status_recorded": True,
        "availability_summary": True,
        "pp_cycle_sequence": 1,
        "timer_entry_ros_ns": 100,
        "header_frame_id": "map",
        "timer_entry_monotonic_ns": 200,
        "cycle_event_count": 0,
        "run_invalid": False,
        "overflow_count": 0,
        "overflow_first_sequence": 0,
        "overflow_last_sequence": 0,
    }
    fields.update(overrides)
    return private_pp_binding_cycle_barrier_eligible(ledger, **fields)  # type: ignore[arg-type]


def test_private_binding_cycle_barrier_rejects_run_invalid_and_overflow_tuple() -> None:
    assert _clean_binding_cycle_barrier(UptakeLedger()) is True
    assert _clean_binding_cycle_barrier(UptakeLedger(), cycle_event_count=1) is False
    assert _clean_binding_cycle_barrier(UptakeLedger(), run_invalid=True) is False
    assert _clean_binding_cycle_barrier(UptakeLedger(), overflow_count=1) is False
    assert (
        _clean_binding_cycle_barrier(UptakeLedger(), overflow_first_sequence=1)
        is False
    )
    assert (
        _clean_binding_cycle_barrier(UptakeLedger(), overflow_last_sequence=1)
        is False
    )


def test_private_binding_cycle_barrier_is_sticky_after_malformed_summary() -> None:
    ledger = UptakeLedger()
    ledger.record_status(
        _identity(),
        header_stamp_ns=0,
        header_frame_id="map",
        first_uptake_pass=False,
        hold_cycle_index=0,
        reject_reason=0,
        receive_monotonic_ns=0,
        accepted_monotonic_ns=0,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=1,
        availability_summary=True,
        availability_present=False,
        timer_entry_monotonic_ns=1,
    )
    assert ledger.first_status_fault is not None
    assert "malformed_status" in ledger.reasons
    assert _clean_binding_cycle_barrier(ledger) is False


def test_private_prelaunch_permit_rejects_late_fault_after_clean_barrier() -> None:
    ledger = UptakeLedger()
    _record_clean_epoch_summary(ledger)
    assert private_prelaunch_permit_eligible(ledger) is True
    ledger.reasons.add("binding_overflow")
    assert private_prelaunch_permit_eligible(ledger) is False


@pytest.mark.parametrize(
    "fault",
    (
        "malformed_status",
        "binding_run_invalid",
        "binding_overflow",
        "clock_nonpositive",
        "clock_not_strictly_increasing",
    ),
)
def test_private_prelaunch_permit_full_loop_denies_fault_before_request(
    fault: str,
) -> None:
    """Fixture loop: clean barrier, late PP fault, then valid request."""
    ledger = UptakeLedger()
    _record_clean_epoch_summary(ledger)
    assert _clean_binding_cycle_barrier(ledger) is True
    ledger.reasons.add(fault)
    assert private_prelaunch_permit_eligible(ledger) is False


def test_private_prelaunch_permit_boundary_is_immutable_after_permit() -> None:
    ledger = UptakeLedger()
    _record_clean_epoch_summary(ledger)
    assert private_prelaunch_permit_eligible(ledger) is True
    permit = {"run_nonce": "exact", "epoch_boundary": "observer_prelaunch_permit"}
    ledger.reasons.add("binding_run_invalid")
    assert permit == {"run_nonce": "exact", "epoch_boundary": "observer_prelaunch_permit"}
    assert private_prelaunch_permit_eligible(ledger) is False


@pytest.mark.parametrize(
    "fault",
    (
        "malformed_status",
        "binding_run_invalid",
        "binding_overflow",
        "clock_nonpositive",
        "clock_not_strictly_increasing",
    ),
)
def test_private_prelaunch_request_file_denies_fault_before_permit(
    tmp_path: "Path", fault: str
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    ledger.reasons.add(fault)
    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert payload is None
    assert reason == "prelaunch_permit_denied"
    assert not permit.exists()


@pytest.mark.parametrize("clock_samples", ((0,), (2, 1)))
def test_private_prelaunch_request_file_denies_observed_clock_fault(
    tmp_path: "Path", clock_samples: tuple[int, ...]
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    for sample in clock_samples:
        ledger.record_clock(sample)
    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert payload is None
    assert reason == "prelaunch_permit_denied"
    assert not permit.exists()


@pytest.mark.parametrize(
    "request_text",
    ("{", '{"schema_version": 1, "run_nonce": "wrong"}'),
)
def test_private_prelaunch_request_file_rejects_invalid_binding(
    tmp_path: "Path", request_text: str
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(request_text, encoding="utf-8")
    payload, reason = process_private_prelaunch_request(
        request, permit, UptakeLedger(), expected_run_nonce="exact"
    )
    assert payload is None
    assert reason == "prelaunch_request_invalid"
    assert not permit.exists()


def test_private_prelaunch_request_file_atomically_seals_epoch(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    _record_clean_epoch_summary(ledger)
    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert reason is None
    assert json.loads(permit.read_text(encoding="utf-8")) == payload
    assert stat.S_IMODE(permit.stat().st_mode) == 0o644
    sealed = permit.read_bytes()
    ledger.reasons.add("binding_run_invalid")
    assert private_prelaunch_permit_eligible(ledger) is False
    assert permit.read_bytes() == sealed


def test_private_prelaunch_request_allows_clean_increasing_clock(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    ledger.record_clock(1)
    ledger.record_clock(2)
    _record_clean_epoch_summary(ledger)
    assert not ledger.reasons
    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert reason is None
    assert payload is not None
    assert json.loads(permit.read_text(encoding="utf-8")) == payload


def test_private_prelaunch_rejects_any_future_ledger_reason() -> None:
    ledger = UptakeLedger()
    ledger.reasons.add("future_startup_fault")
    assert private_prelaunch_permit_eligible(ledger) is False
    assert _clean_binding_cycle_barrier(ledger) is False


def _empty_identity() -> tuple[object, ...]:
    return ("", "", 0, 0, 0, 0, "", (0,) * 32)


def _startup_fault_status(
    *, stamp: int, cycle: int, summary: bool, reject_reason: int, run_invalid: bool
) -> dict[str, object]:
    return {
        "header_stamp_ns": stamp,
        "header_frame_id": "map",
        "first_uptake_pass": False,
        "hold_cycle_index": 0,
        "reject_reason": reject_reason,
        "receive_monotonic_ns": 0,
        "accepted_monotonic_ns": 0,
        "run_invalid": run_invalid,
        "overflow_count": 0,
        "overflow_first_sequence": 0,
        "overflow_last_sequence": 0,
        "cycle_event_index": 0,
        "cycle_event_count": 1,
        "pp_cycle_sequence": cycle,
        "availability_summary": summary,
        "availability_present": False,
        "availability_identity": _empty_identity(),
        "availability_safety_valid_until_ns": 0,
        "availability_transition": 0,
        "timer_entry_monotonic_ns": 200,
    }


def _startup_clock_fault_summary(
    ledger: UptakeLedger,
    *,
    stamp: int,
    cycle: int,
    run_invalid: bool = True,
) -> bool:
    return private_pp_startup_clock_fault_summary_eligible(
        ledger,
        _empty_identity(),
        _startup_fault_status(
            stamp=stamp,
            cycle=cycle,
            summary=True,
            reject_reason=0,
            run_invalid=run_invalid,
        ),
    )


def _record_startup_clock_fault_pair(
    ledger: UptakeLedger, *, stamp: int, cycle: int
) -> None:
    summary = _startup_fault_status(
        stamp=stamp, cycle=cycle, summary=True, reject_reason=0, run_invalid=True
    )
    assert private_pp_startup_clock_fault_summary_eligible(
        ledger, _empty_identity(), summary
    )
    ledger.pending_startup_clock_fault = {"status": summary}
    event = _startup_fault_status(
        stamp=stamp,
        cycle=cycle,
        summary=False,
        reject_reason=9,
        run_invalid=True,
    )
    assert private_pp_startup_clock_fault_event_eligible(
        ledger, _empty_identity(), event
    )
    ledger.record_startup_clock_fault_pair(summary, event)


def _record_clean_epoch_summary(
    ledger: UptakeLedger, *, cycle: int = 2, stamp: int = 100
) -> None:
    ledger.begin_experimental_epoch(first_cycle_sequence=cycle)
    ledger.record_status(
        _empty_identity(),
        header_stamp_ns=stamp,
        header_frame_id="map",
        first_uptake_pass=False,
        hold_cycle_index=0,
        reject_reason=0,
        receive_monotonic_ns=0,
        accepted_monotonic_ns=0,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=cycle,
        availability_summary=True,
        availability_present=False,
        availability_identity=_empty_identity(),
        timer_entry_monotonic_ns=200,
        cycle_event_count=0,
    )
    ledger.seal_experimental_epoch_start()


def test_private_prelaunch_keeps_full_neutral_clock_recovery_outside_epoch(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    _record_startup_clock_fault_pair(ledger, stamp=0, cycle=1)
    _record_startup_clock_fault_pair(ledger, stamp=100, cycle=2)
    _record_startup_clock_fault_pair(ledger, stamp=100, cycle=3)
    assert ledger.reasons == set()
    assert ledger.status_message_count == 0

    _record_clean_epoch_summary(ledger, cycle=4)
    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )

    assert reason is None
    assert payload is not None
    assert ledger.startup_status_count == 3
    assert ledger.startup_zero_status is not None
    assert ledger.startup_first_positive_status is not None
    assert ledger.startup_last_status == {
        "summary": _startup_fault_status(
            stamp=100, cycle=3, summary=True, reject_reason=0, run_invalid=True
        ),
        "event": _startup_fault_status(
            stamp=100, cycle=3, summary=False, reject_reason=9, run_invalid=True
        ),
    }
    assert payload["experimental_epoch_status_ordinal"] == 1


def test_private_prelaunch_clock_recovery_does_not_allow_generic_run_invalid() -> None:
    ledger = UptakeLedger()
    bad_reject = _startup_fault_status(
        stamp=0, cycle=1, summary=True, reject_reason=1, run_invalid=True
    )
    assert not private_pp_startup_clock_fault_summary_eligible(
        ledger, _empty_identity(), bad_reject
    )
    bad_event_count = _startup_fault_status(
        stamp=0, cycle=1, summary=True, reject_reason=0, run_invalid=True
    )
    bad_event_count["cycle_event_count"] = 0
    assert not private_pp_startup_clock_fault_summary_eligible(
        ledger, _empty_identity(), bad_event_count
    )


def test_private_prelaunch_clock_fault_pair_rejects_orphan_reorder_and_pending() -> None:
    ledger = UptakeLedger()
    orphan_event = _startup_fault_status(
        stamp=0, cycle=1, summary=False, reject_reason=9, run_invalid=True
    )
    assert not private_pp_startup_clock_fault_event_eligible(
        ledger, _empty_identity(), orphan_event
    )
    summary = _startup_fault_status(
        stamp=0, cycle=1, summary=True, reject_reason=0, run_invalid=True
    )
    assert private_pp_startup_clock_fault_summary_eligible(
        ledger, _empty_identity(), summary
    )
    ledger.pending_startup_clock_fault = {"status": summary}
    reordered_summary = _startup_fault_status(
        stamp=0, cycle=2, summary=True, reject_reason=0, run_invalid=True
    )
    assert not private_pp_startup_clock_fault_event_eligible(
        ledger, _empty_identity(), reordered_summary
    )
    assert "startup_clock_fault_event_pending" in ledger.terminal()["reasons"]


def test_private_prelaunch_accepts_positive_first_clock_fault_pair_then_later_clean(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    _record_startup_clock_fault_pair(ledger, stamp=100, cycle=1)
    assert private_pp_startup_clean_stamp_eligible(ledger, 100) is False
    assert private_pp_startup_clean_stamp_eligible(ledger, 101) is True
    _record_clean_epoch_summary(ledger, cycle=2, stamp=101)

    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert reason is None
    assert payload is not None


def test_private_prelaunch_file_protocol_accepts_clean_epoch_after_startup_records(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    _record_startup_clock_fault_pair(ledger, stamp=0, cycle=1)
    _record_startup_clock_fault_pair(ledger, stamp=100, cycle=2)
    _record_clean_epoch_summary(ledger, cycle=3, stamp=200)

    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )

    assert reason is None
    assert payload is not None
    assert payload["experimental_epoch_status_ordinal"] == 1
    assert payload["permit_status_ordinal"] == 1
    assert ledger.startup_zero_status is not None
    assert ledger.startup_first_positive_status is not None
    assert ledger.first_status_fault is None
    assert json.loads(permit.read_text(encoding="utf-8")) == payload


def test_private_prelaunch_file_protocol_denies_fault_before_permit(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    _record_clean_epoch_summary(ledger)
    ledger.reasons.add("binding_run_invalid")

    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )

    assert payload is None
    assert reason == "prelaunch_permit_denied"
    assert not permit.exists()


def test_private_prelaunch_file_protocol_keeps_post_permit_fault_terminal(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    _record_clean_epoch_summary(ledger)
    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert reason is None
    assert payload is not None

    ledger.record_status(
        _empty_identity(),
        header_stamp_ns=0,
        first_uptake_pass=False,
        hold_cycle_index=0,
        reject_reason=0,
        receive_monotonic_ns=0,
        accepted_monotonic_ns=0,
        run_invalid=True,
        overflow_count=0,
        pp_cycle_sequence=3,
        availability_summary=True,
        availability_present=False,
        availability_identity=_empty_identity(),
    )

    assert ledger.prelaunch_permit_status_ordinal == 1
    assert ledger.first_status_fault is not None
    assert "binding_run_invalid" in ledger.reasons
    assert permit.exists()


def test_private_prelaunch_file_protocol_rejects_proposal_before_epoch(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    ledger.record_proposal(
        _identity(),
        safety_valid_until_ns=2_000,
        header_stamp_ros_ns=1_000,
        before_binding_cycle=True,
    )

    assert _clean_binding_cycle_barrier(ledger) is False
    payload, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert payload is None
    assert reason == "prelaunch_permit_denied"
    assert not permit.exists()


def test_private_prelaunch_file_protocol_rejects_second_permit(
    tmp_path: "Path",
) -> None:
    from pathlib import Path

    request = Path(tmp_path) / "request.json"
    permit = Path(tmp_path) / "permit.json"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": "exact"}), encoding="utf-8"
    )
    ledger = UptakeLedger()
    _record_clean_epoch_summary(ledger)
    first, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert first is not None
    assert reason is None

    second, reason = process_private_prelaunch_request(
        request, permit, ledger, expected_run_nonce="exact"
    )
    assert second is None
    assert reason == "prelaunch_permit_exists"


def test_g3_source_timestamp_contract_is_bound_to_planner_and_pp_timer_entry() -> None:
    repo = __import__("pathlib").Path(__file__).parents[3]
    planner = (
        repo
        / "aichallenge/workspace/src/aichallenge_submit/"
        "state_lattice_overtake_planner/src/"
        "state_lattice_overtake_planner_node.cpp"
    ).read_text(encoding="utf-8")
    pp = (
        repo
        / "aichallenge/workspace/src/aichallenge_submit/"
        "simple_pure_pursuit/src/simple_pure_pursuit.cpp"
    ).read_text(encoding="utf-8")
    binding = (
        repo
        / "aichallenge/workspace/src/aichallenge_submit/"
        "overtake_transport_contract/src/state_lattice_v2_binding.cpp"
    ).read_text(encoding="utf-8")
    assert "const auto plan_stamp = now();" in planner
    assert "message.header.stamp = proposal.trajectory.plan_stamp;" in planner
    assert (
        "message.identity.source_stamp = proposal.trajectory.base_source_stamp;"
        in planner
    )
    assert "message.proposal = proposal.trajectory;" in planner
    shadow = (
        repo
        / "aichallenge/workspace/src/aichallenge_submit/"
        "state_lattice_overtake_planner/src/c002ay0_shadow_proposal.cpp"
    ).read_text(encoding="utf-8")
    assert "std::min(lease_ns, plan_ns + kShadowSafetyLifetimeNs)" in shadow
    assert "trajectory.base_lease_valid_until = base.lease_valid_until;" in shadow
    assert "trajectory.base_source_kind = base.base_source_kind;" in shadow
    message = (
        repo
        / "aichallenge/workspace/src/aichallenge_submit/"
        "multi_purpose_mpc_ros_msgs/msg/AuthorizedCartesianTrajectory.msg"
    ).read_text(encoding="utf-8")
    assert "uint8 SOURCE_MPC_HORIZON=1" in message
    assert "uint8 SOURCE_REFERENCE_TRAJECTORY=2" in message
    assert VALID_BASE_SOURCE_KINDS == frozenset((1, 2))
    assert "timeNs(proposal.header.stamp) != timeNs(payload.plan_stamp)" in binding
    assert "const auto stamp = get_clock()->now();" in pp
    assert "beginCycle(\n        stamp_msg, steadyNowNanoseconds())" in pp
    assert "publishStateLatticeV2BindingStatus(stamp_msg, v2_result);" in pp
    assert "status.header.stamp = stamp;" in pp
    assert "state_lattice_v2_test_clock_readiness" not in pp


def test_graph_provenance_rejects_forbidden_component_endpoint() -> None:
    invalid = _component_graph_evidence(_GraphNode(forbidden_planner_endpoint=True))
    assert invalid["ready"] is False
    assert invalid["forbidden_component_endpoints"] == {
        "state_lattice_overtake_planner_node": ["/control/command/control_cmd"]
    }


def test_graph_provenance_rejects_duplicate_required_component_node() -> None:
    invalid = _component_graph_evidence(_GraphNode(duplicate_planner_node=True))
    assert invalid["ready"] is False
    assert invalid["duplicate_component_nodes"] == [
        "state_lattice_overtake_planner_node"
    ]


def _identity(
    sequence: int = 1,
    generation: int = 3,
    *,
    producer: str = "42",
    session: str = "7",
) -> tuple[object, ...]:
    return (
        producer,
        session,
        sequence,
        generation,
        2,
        1_000,
        "map",
        (1,) + (0,) * 31,
    )


def _record_first_uptake(ledger: UptakeLedger, key: tuple[object, ...]) -> None:
    _record_summary(
        ledger,
        key,
        cycle=6,
        hold_cycle_index=1,
        transition=1,
        event_count=1,
    )
    ledger.record_status(
        key,
        header_stamp_ns=201,
        first_uptake_pass=True,
        hold_cycle_index=1,
        reject_reason=0,
        receive_monotonic_ns=10,
        accepted_monotonic_ns=11,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=6,
        availability_present=True,
        availability_identity=key,
        availability_safety_valid_until_ns=500,
        availability_transition=1,
        timer_entry_monotonic_ns=106,
        cycle_event_count=1,
    )


def _record_summary(
    ledger: UptakeLedger,
    key: tuple[object, ...],
    *,
    cycle: int,
    hold_cycle_index: int,
    transition: int,
    event_count: int = 0,
    valid_until_ns: int = 500,
    availability_present: bool = True,
    availability_key: tuple[object, ...] | None = None,
    header_stamp_ns: int | None = None,
    timer_entry_monotonic_ns: int | None = None,
) -> None:
    summary_identity = (
        availability_key
        if availability_key is not None
        else key
        if availability_present
        else ("", "", 0, 0, 0, 0, "", (0,) * 32)
    )
    ledger.record_status(
        ("", "", 0, 0, 0, 0, "", (0,) * 32),
        header_stamp_ns=(
            header_stamp_ns if header_stamp_ns is not None else 200 + cycle
        ),
        first_uptake_pass=False,
        hold_cycle_index=0,
        reject_reason=0,
        receive_monotonic_ns=0,
        accepted_monotonic_ns=0,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=cycle,
        availability_summary=True,
        availability_present=availability_present,
        availability_identity=summary_identity,
        availability_safety_valid_until_ns=valid_until_ns,
        availability_transition=transition,
        timer_entry_monotonic_ns=(
            timer_entry_monotonic_ns
            if timer_entry_monotonic_ns is not None
            else 100 + cycle
        ),
        cycle_event_count=event_count,
    )


def _record_cycle_event(
    ledger: UptakeLedger,
    key: tuple[object, ...],
    *,
    cycle: int,
    first_uptake: bool,
    hold_cycle_index: int,
    reject_reason: int,
    availability_present: bool,
    availability_key: tuple[object, ...],
    valid_until_ns: int,
    transition: int,
    event_index: int = 0,
    event_count: int = 1,
    receive_monotonic_ns: int = 10,
    accepted_monotonic_ns: int = 11,
    observer_receive_monotonic_ns: int = 1_000,
    header_stamp_ns: int | None = None,
    timer_entry_monotonic_ns: int | None = None,
) -> None:
    ledger.record_status(
        key,
        header_stamp_ns=(
            header_stamp_ns if header_stamp_ns is not None else 200 + cycle
        ),
        first_uptake_pass=first_uptake,
        hold_cycle_index=hold_cycle_index,
        reject_reason=reject_reason,
        receive_monotonic_ns=receive_monotonic_ns,
        accepted_monotonic_ns=accepted_monotonic_ns,
        run_invalid=False,
        overflow_count=0,
        observer_receive_monotonic_ns=observer_receive_monotonic_ns,
        pp_cycle_sequence=cycle,
        availability_present=availability_present,
        availability_identity=availability_key,
        availability_safety_valid_until_ns=valid_until_ns,
        availability_transition=transition,
        timer_entry_monotonic_ns=(
            timer_entry_monotonic_ns
            if timer_entry_monotonic_ns is not None
            else 100 + cycle
        ),
        cycle_event_index=event_index,
        cycle_event_count=event_count,
    )


def _availability_ledger(
    *,
    include_expiry_boundary: bool = True,
    hold_cycles: int = 10,
    safety_valid_until_ns: int = 500,
) -> UptakeLedger:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(
        key,
        safety_valid_until_ns=safety_valid_until_ns,
        header_stamp_ros_ns=120,
    )
    for cycle in range(1, hold_cycles + 1):
        _record_summary(
            ledger,
            key,
            cycle=cycle,
            hold_cycle_index=cycle,
            transition=1 if cycle == 1 else 2,
            valid_until_ns=safety_valid_until_ns,
            event_count=1 if cycle == 1 else 0,
        )
        if cycle == 1:
            ledger.record_status(
                key,
                header_stamp_ns=201,
                first_uptake_pass=True,
                hold_cycle_index=1,
                reject_reason=0,
                receive_monotonic_ns=10,
                accepted_monotonic_ns=11,
                run_invalid=False,
                overflow_count=0,
                pp_cycle_sequence=1,
                availability_present=True,
                availability_identity=key,
                availability_safety_valid_until_ns=safety_valid_until_ns,
                availability_transition=1,
                timer_entry_monotonic_ns=101,
                cycle_event_count=1,
            )
    if include_expiry_boundary:
        _record_summary(
            ledger,
            key,
            cycle=hold_cycles + 1,
            hold_cycle_index=0,
            transition=0,
            availability_present=False,
            valid_until_ns=0,
            header_stamp_ns=safety_valid_until_ns,
            event_count=1,
        )
        _record_cycle_event(
            ledger,
            key,
            cycle=hold_cycles + 1,
            first_uptake=False,
            hold_cycle_index=hold_cycles,
            reject_reason=6,
            availability_present=False,
            availability_key=("", "", 0, 0, 0, 0, "", (0,) * 32),
            valid_until_ns=0,
            transition=0,
            header_stamp_ns=safety_valid_until_ns,
        )
    return ledger


def _g3_single_terminal_ledger(
    *,
    proposal_source_time_ns: int,
    pp_timer_entry_ros_ns: int,
    safety_valid_until_ns: int,
    first_uptake_pass: bool = True,
    hold_cycle_index: int = 1,
    reject_reason: int = 0,
    payload_plan_stamp_ros_ns: int | None = None,
) -> tuple[UptakeLedger, tuple[object, ...]]:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(
        key,
        safety_valid_until_ns,
        proposal_source_time_ns,
        payload_plan_stamp_ros_ns=(
            proposal_source_time_ns
            if payload_plan_stamp_ros_ns is None
            else payload_plan_stamp_ros_ns
        ),
    )
    _record_summary(
        ledger,
        key,
        cycle=1,
        hold_cycle_index=hold_cycle_index,
        transition=1,
        event_count=1,
        valid_until_ns=safety_valid_until_ns,
        header_stamp_ns=pp_timer_entry_ros_ns,
    )
    _record_cycle_event(
        ledger,
        key,
        cycle=1,
        first_uptake=first_uptake_pass,
        hold_cycle_index=hold_cycle_index,
        reject_reason=reject_reason,
        availability_present=True,
        availability_key=key,
        valid_until_ns=safety_valid_until_ns,
        transition=1,
        header_stamp_ns=pp_timer_entry_ros_ns,
    )
    return ledger, key


def test_terminal_passes_for_every_cycle_before_safety_deadline() -> None:
    result = _availability_ledger().terminal()
    assert result["outcome"] == "PASS"
    assert result["label"] == LABEL
    assert result["qualifier"] == QUALIFIER
    assert [
        event["pp_cycle_sequence"] for event in result["availability"]["availability_cycles"]
    ] == list(range(1, 11))
    assert (
        result["availability"]["first_uptakes"][0]["proposal_header_stamp_ros_ns"]
        == 120
    )
    g3 = result["initial_exact_uptake"]
    assert g3["valid"] is True
    assert g3["primary_terminal"]["kind"] == "FIRST_UPTAKE"
    assert g3["primary_terminal"]["pp_cycle_sequence"] == 1
    assert g3["primary_terminal"]["pp_timer_entry_ros_ns"] == 201
    assert g3["primary_terminal"]["status_header_stamp_ns"] == 201
    assert g3["proposal_provenance"]["proposal_source_time_ns"] == 120
    assert g3["proposal_provenance"]["identity_source_stamp_ns"] == 1_000
    assert g3["pp_timer_entry_minus_proposal_source_ns"] == 81
    assert (
        g3["primary_terminal"]["observer_receive_used_for_200ms_predicate"]
        is False
    )


@pytest.mark.parametrize("hold_cycles", [3, 17])
def test_deadline_contract_is_not_a_fixed_cycle_count(hold_cycles: int) -> None:
    deadline_ns = 400 + hold_cycles
    result = _availability_ledger(
        hold_cycles=hold_cycles,
        safety_valid_until_ns=deadline_ns,
    ).terminal()
    assert result["outcome"] == "PASS"
    assert [
        event["pp_cycle_sequence"]
        for event in result["availability"]["availability_cycles"]
    ] == list(range(1, hold_cycles + 1))


def test_deadline_summary_without_exact_stale_terminal_cannot_pass() -> None:
    ledger = _availability_ledger()
    state = ledger.resolutions[_identity()]
    state.post_uptake_stale = None
    state.post_uptake_stale_count = 0
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "deadline_availability_chain_incomplete" in result["reasons"]


def test_g3_pre_uptake_reject_is_exact_terminal_but_not_first_uptake() -> None:
    ledger, key = _g3_single_terminal_ledger(
        proposal_source_time_ns=100,
        pp_timer_entry_ros_ns=200,
        safety_valid_until_ns=500,
        first_uptake_pass=False,
        hold_cycle_index=0,
        reject_reason=7,
    )
    result = ledger.terminal()
    g3 = result["initial_exact_uptake"]
    assert g3["initial_eligible_identity"]["proposal_sequence"] == key[2]
    assert g3["primary_terminal"]["kind"] == "PRE_UPTAKE_REJECT"
    assert g3["primary_terminal"]["reject_reason"] == 7
    assert g3["predicates"]["primary_resolution_exactly_one"] is True
    assert g3["predicates"]["primary_terminal_exact_first"] is False
    assert g3["valid"] is False


def test_g3_rejects_negative_source_to_pp_timer_delta() -> None:
    ledger, _ = _g3_single_terminal_ledger(
        proposal_source_time_ns=300,
        pp_timer_entry_ros_ns=200,
        safety_valid_until_ns=500,
    )
    g3 = ledger.terminal()["initial_exact_uptake"]
    assert g3["pp_timer_entry_minus_proposal_source_ns"] == -100
    assert g3["predicates"]["timer_entry_not_before_proposal_source"] is False
    assert g3["valid"] is False


@pytest.mark.parametrize(
    ("delta_ns", "expected"),
    [(199_999_999, True), (200_000_000, False)],
)
def test_g3_uses_strict_200ms_boundary(delta_ns: int, expected: bool) -> None:
    source_ns = 100
    timer_ns = source_ns + delta_ns
    ledger, _ = _g3_single_terminal_ledger(
        proposal_source_time_ns=source_ns,
        pp_timer_entry_ros_ns=timer_ns,
        safety_valid_until_ns=timer_ns + 1,
    )
    g3 = ledger.terminal()["initial_exact_uptake"]
    assert g3["pp_timer_entry_minus_proposal_source_ns"] == delta_ns
    assert g3["predicates"]["timer_entry_within_200ms"] is expected
    assert g3["valid"] is expected


@pytest.mark.parametrize(
    ("timer_ns", "expected"),
    [(499, True), (500, False)],
)
def test_g3_uses_strict_safety_deadline_boundary(
    timer_ns: int, expected: bool
) -> None:
    ledger, _ = _g3_single_terminal_ledger(
        proposal_source_time_ns=100,
        pp_timer_entry_ros_ns=timer_ns,
        safety_valid_until_ns=500,
    )
    g3 = ledger.terminal()["initial_exact_uptake"]
    assert g3["predicates"]["timer_entry_before_safety_deadline"] is expected
    assert g3["valid"] is expected


def test_g3_missing_plan_stamp_provenance_fails_closed() -> None:
    ledger, _ = _g3_single_terminal_ledger(
        proposal_source_time_ns=100,
        pp_timer_entry_ros_ns=200,
        safety_valid_until_ns=500,
        payload_plan_stamp_ros_ns=0,
    )
    result = ledger.terminal()
    g3 = result["initial_exact_uptake"]
    assert g3["initial_eligible_identity"] is None
    assert g3["proposal_provenance"] is None
    assert g3["predicates"]["proposal_source_time_present"] is False
    assert g3["valid"] is False
    assert "malformed_proposal" in result["reasons"]


def test_g3_initial_pre_reject_cannot_reanchor_to_later_first_uptake() -> None:
    ledger = UptakeLedger()
    first_key = _identity(sequence=1, generation=3)
    later_key = _identity(sequence=2, generation=4)
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(first_key, 1_000, 100)
    _record_summary(
        ledger,
        first_key,
        cycle=1,
        hold_cycle_index=0,
        transition=1,
        event_count=1,
        valid_until_ns=1_000,
        header_stamp_ns=200,
    )
    _record_cycle_event(
        ledger,
        first_key,
        cycle=1,
        first_uptake=False,
        hold_cycle_index=0,
        reject_reason=7,
        availability_present=True,
        availability_key=first_key,
        valid_until_ns=1_000,
        transition=1,
        header_stamp_ns=200,
    )
    ledger.record_proposal(later_key, 1_000, 150)
    _record_summary(
        ledger,
        later_key,
        cycle=2,
        hold_cycle_index=1,
        transition=3,
        event_count=1,
        valid_until_ns=1_000,
        header_stamp_ns=250,
    )
    _record_cycle_event(
        ledger,
        later_key,
        cycle=2,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=0,
        availability_present=True,
        availability_key=later_key,
        valid_until_ns=1_000,
        transition=3,
        header_stamp_ns=250,
    )
    result = ledger.terminal()
    assert ledger.initial_eligible_key == first_key
    assert ledger.anchor_key is None
    assert result["initial_exact_uptake"]["primary_terminal"]["kind"] == (
        "PRE_UPTAKE_REJECT"
    )
    assert result["initial_exact_uptake"]["initial_eligible_identity"][
        "proposal_sequence"
    ] == 1


def test_terminal_fails_closed_for_cycle_sequence_gap() -> None:
    ledger = _availability_ledger()
    ledger.availability_window.pop(6)
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "deadline_availability_chain_incomplete" in result["reasons"]
    assert "availability_window_predicate_failure" in result["reasons"]
    assert "pp_cycle_sequence_gap" not in result["reasons"]


def test_later_valid_window_cannot_hide_gap_after_first_uptake() -> None:
    ledger = _availability_ledger()
    ledger.availability_window[2]["availability_present"] = False
    key = _identity()
    for cycle in range(12, 21):
        _record_summary(
            ledger,
            key,
            cycle=cycle,
            hold_cycle_index=cycle,
            transition=2,
        )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "deadline_availability_chain_incomplete" in result["reasons"]


def test_terminal_fails_closed_for_expired_availability_identity() -> None:
    ledger = _availability_ledger()
    ledger.availability_window[5]["header_stamp_ns"] = 500
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "deadline_availability_chain_incomplete" in result["reasons"]


def test_terminal_fails_closed_for_reject_only_cycle() -> None:
    ledger = _availability_ledger()
    ledger.availability_window[4]["reject_reason"] = 6
    ledger.availability_window[4]["availability_present"] = False
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "deadline_availability_chain_incomplete" in result["reasons"]


def test_terminal_fails_closed_for_clock_or_run_invalid() -> None:
    ledger = _availability_ledger()
    ledger.record_clock(150)
    ledger.reasons.add("binding_run_invalid")
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "clock_not_strictly_increasing" in result["reasons"]
    assert "binding_run_invalid" in result["reasons"]


def test_identity_transition_requires_exact_newer_accepted_generation() -> None:
    ledger = _availability_ledger()
    old_key = _identity()
    invalid_key = _identity(sequence=2, generation=3)
    ledger.record_proposal(
        invalid_key, safety_valid_until_ns=500, header_stamp_ros_ns=121
    )
    _record_first_uptake(ledger, invalid_key)
    for event in (ledger.availability_window[sequence] for sequence in range(6, 11)):
        event["availability_identity"] = invalid_key
        event["availability_transition"] = 3
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "deadline_availability_chain_incomplete" in result["reasons"]
    assert old_key != invalid_key


def test_same_identity_requires_held_transition() -> None:
    ledger = _availability_ledger()
    ledger.availability_window[6]["availability_transition"] = 3
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "deadline_availability_chain_incomplete" in result["reasons"]


def test_every_proposal_must_have_exact_first_uptake() -> None:
    ledger = _availability_ledger()
    ledger.record_proposal(
        _identity(sequence=2, generation=4),
        safety_valid_until_ns=500,
        header_stamp_ros_ns=121,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "proposal_without_exact_first_uptake" in result["reasons"]


def test_proposal_before_binding_cycle_is_sticky_invalid() -> None:
    ledger = _availability_ledger()
    early_key = _identity(sequence=2, generation=4)
    ledger.record_proposal(
        early_key,
        safety_valid_until_ns=500,
        header_stamp_ros_ns=121,
        before_binding_cycle=True,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "proposal_before_pp_binding_cycle" in result["reasons"]
    assert result["epoch"] == {
        "proposal_before_pp_binding_cycle_count": 1,
        "binding_cycle_before_all_proposals": False,
    }


def test_first_status_fault_preserves_raw_fields_and_arrival_order() -> None:
    ledger = UptakeLedger()
    invalid = ("", "", 0, 0, 0, 0, "", (0,) * 32)
    ledger.record_status(
        invalid,
        header_stamp_ns=0,
        header_frame_id="map",
        first_uptake_pass=False,
        hold_cycle_index=0,
        reject_reason=9,
        receive_monotonic_ns=101,
        accepted_monotonic_ns=0,
        run_invalid=True,
        overflow_count=2,
        overflow_first_sequence=41,
        overflow_last_sequence=42,
        observer_receive_monotonic_ns=1_001,
        pp_cycle_sequence=7,
        availability_summary=True,
        availability_present=True,
        availability_identity=invalid,
        availability_safety_valid_until_ns=0,
        availability_transition=0,
        timer_entry_monotonic_ns=99,
        cycle_event_index=0,
        cycle_event_count=1,
    )
    first = ledger.first_status_fault
    assert first is not None
    assert first["arrival_index"] == 1
    assert first["classifications"] == [
        "observer_malformed_status",
        "production_binding_run_invalid",
    ]
    assert first["failed_observer_predicates"] == [
        "availability_identity_valid_when_present",
        "header_stamp_positive",
        "summary_terminal_fields_neutral",
    ]
    assert first["event"]["reject_reason"] == 9
    assert first["event"]["overflow_count"] == 2
    assert first["event"]["overflow_first_sequence"] == 41
    assert first["event"]["overflow_last_sequence"] == 42
    assert first["event"]["observer_receive_monotonic_ns"] == 1_001

    ledger.record_status(
        _identity(),
        header_stamp_ns=200,
        header_frame_id="map",
        first_uptake_pass=False,
        hold_cycle_index=0,
        reject_reason=0,
        receive_monotonic_ns=102,
        accepted_monotonic_ns=0,
        run_invalid=True,
        overflow_count=3,
        overflow_first_sequence=41,
        overflow_last_sequence=43,
        observer_receive_monotonic_ns=1_002,
        pp_cycle_sequence=8,
    )
    assert ledger.status_message_count == 2
    assert ledger.first_status_fault == first
    result = ledger.terminal()
    assert result["first_status_fault"] == first
    assert "binding_run_invalid" in result["reasons"]
    assert "malformed_status" in result["reasons"]


def test_status_extraction_failure_is_ordered_and_immutable() -> None:
    ledger = UptakeLedger()
    ledger.record_status_extraction_fault(
        observer_receive_monotonic_ns=2_001, error_type="ValueError"
    )
    first = ledger.first_status_fault
    assert first == {
        "arrival_index": 1,
        "classifications": ["observer_malformed_status"],
        "failed_observer_predicates": ["status_field_extraction"],
        "event": {
            "observer_receive_monotonic_ns": 2_001,
            "extraction_error_type": "ValueError",
            "raw_fields_unavailable": True,
        },
    }
    ledger.record_status_extraction_fault(
        observer_receive_monotonic_ns=2_002, error_type="TypeError"
    )
    assert ledger.status_message_count == 2
    assert ledger.first_status_fault == first
    assert ledger.terminal()["first_status_fault"] == first


def test_overlong_unfinished_chain_fails_closed_at_bounded_capacity() -> None:
    ledger = _availability_ledger(include_expiry_boundary=False)
    key = _identity()
    for cycle in range(11, 5502):
        _record_summary(
            ledger,
            key,
            cycle=cycle,
            hold_cycle_index=cycle,
            transition=2,
        )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert result["status_message_count"] > 5481
    assert result["stream_state"]["retained_s0_window_count"] == 256
    assert result["stream_state"]["retained_identity_count"] == 1
    assert result["availability"] is None
    assert "observer_ledger_overflow" not in result["reasons"]
    assert "availability_chain_capacity_exceeded" in result["reasons"]


@pytest.mark.parametrize(
    ("second_sequence", "expected_reason"),
    [
        (1, "availability_cycle_duplicate"),
        (3, "pp_cycle_sequence_gap"),
        (0, "malformed_status"),
    ],
)
def test_streaming_sequence_fault_is_sticky(
    second_sequence: int, expected_reason: str
) -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    _record_summary(ledger, key, cycle=1, hold_cycle_index=1, transition=1)
    _record_summary(
        ledger,
        key,
        cycle=second_sequence,
        hold_cycle_index=2,
        transition=2,
    )
    for cycle in range(11, 21):
        _record_summary(
            ledger, key, cycle=cycle, hold_cycle_index=cycle, transition=2
        )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert expected_reason in result["reasons"]


def test_positive_out_of_order_cycle_is_sticky() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    _record_summary(ledger, key, cycle=1, hold_cycle_index=1, transition=1)
    _record_summary(ledger, key, cycle=2, hold_cycle_index=2, transition=2)
    _record_summary(ledger, key, cycle=1, hold_cycle_index=1, transition=1)
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "availability_cycle_out_of_order" in result["reasons"]


def test_first_cycle_must_close_the_observed_epoch_start() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    _record_summary(ledger, key, cycle=2, hold_cycle_index=1, transition=1)
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "availability_cycle_start_mismatch" in result["reasons"]


def test_s0_is_fixed_by_first_eligible_proposal_not_status_arrival_order() -> None:
    ledger = UptakeLedger()
    first_key = _identity(sequence=1, generation=3)
    later_key = _identity(sequence=2, generation=4)
    ledger.record_clock(100)
    ledger.record_clock(200)
    _record_summary(
        ledger, first_key, cycle=1, hold_cycle_index=1, transition=1, event_count=1
    )
    ledger.record_status(
        first_key,
        header_stamp_ns=201,
        first_uptake_pass=True,
        hold_cycle_index=1,
        reject_reason=0,
        receive_monotonic_ns=10,
        accepted_monotonic_ns=11,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=1,
        availability_present=True,
        availability_identity=first_key,
        availability_safety_valid_until_ns=500,
        availability_transition=1,
        timer_entry_monotonic_ns=101,
        cycle_event_count=1,
    )
    ledger.record_proposal(later_key, 500, 121)
    _record_summary(
        ledger, later_key, cycle=2, hold_cycle_index=1, transition=3, event_count=1
    )
    ledger.record_status(
        later_key,
        header_stamp_ns=202,
        first_uptake_pass=True,
        hold_cycle_index=1,
        reject_reason=0,
        receive_monotonic_ns=12,
        accepted_monotonic_ns=13,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=2,
        availability_present=True,
        availability_identity=later_key,
        availability_safety_valid_until_ns=500,
        availability_transition=3,
        timer_entry_monotonic_ns=102,
        cycle_event_count=1,
    )
    ledger.record_proposal(first_key, 500, 120)
    result = ledger.terminal()
    assert ledger.initial_eligible_key == later_key
    assert ledger.anchor_key == later_key
    assert ledger.anchor_sequence == 2
    assert result["initial_exact_uptake"]["initial_eligible_identity"][
        "proposal_sequence"
    ] == 2
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "deadline_availability_chain_incomplete" in result["reasons"]


def test_late_fault_survives_long_clean_tail() -> None:
    ledger = _availability_ledger()
    key = _identity()
    ledger.record_clock(150)
    for cycle in range(11, 5502):
        _record_summary(
            ledger, key, cycle=cycle, hold_cycle_index=cycle, transition=2
        )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "clock_not_strictly_increasing" in result["reasons"]


def test_cycle_event_multiplicity_is_exact_and_sticky() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    _record_summary(
        ledger,
        key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=2,
    )
    ledger.record_status(
        key,
        header_stamp_ns=201,
        first_uptake_pass=True,
        hold_cycle_index=1,
        reject_reason=0,
        receive_monotonic_ns=10,
        accepted_monotonic_ns=11,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=1,
        availability_present=True,
        availability_identity=key,
        availability_safety_valid_until_ns=500,
        availability_transition=1,
        timer_entry_monotonic_ns=101,
        cycle_event_index=0,
        cycle_event_count=2,
    )
    _record_summary(
        ledger, key, cycle=2, hold_cycle_index=2, transition=2
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "cycle_event_incomplete" in result["reasons"]


def test_cycle_event_count_above_production_maximum_is_invalid() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    _record_summary(
        ledger,
        key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=12,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "cycle_event_count_out_of_production_range" in result["reasons"]


def test_cycle_event_count_at_production_maximum_is_accepted() -> None:
    ledger = UptakeLedger()
    key = _identity()
    _record_summary(
        ledger,
        key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=11,
    )
    assert "cycle_event_count_out_of_production_range" not in ledger.reasons
    assert ledger.active_cycle is not None
    assert ledger.active_cycle.expected_event_count == 11


def test_duplicate_terminal_event_index_is_invalid() -> None:
    ledger = _availability_ledger(include_expiry_boundary=False)
    key = _identity(sequence=2, generation=4)
    ledger.record_proposal(key, 600, 220)
    _record_summary(
        ledger,
        key,
        cycle=11,
        hold_cycle_index=1,
        transition=3,
        event_count=2,
    )
    for _ in range(2):
        ledger.record_status(
            key,
            header_stamp_ns=211,
            first_uptake_pass=True,
            hold_cycle_index=1,
            reject_reason=0,
            receive_monotonic_ns=20,
            accepted_monotonic_ns=21,
            run_invalid=False,
            overflow_count=0,
            pp_cycle_sequence=11,
            availability_present=True,
            availability_identity=key,
            availability_safety_valid_until_ns=500,
            availability_transition=3,
            timer_entry_monotonic_ns=111,
            cycle_event_index=0,
            cycle_event_count=2,
        )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "cycle_event_duplicate" in result["reasons"]


def test_multiple_first_uptakes_in_one_cycle_are_invalid() -> None:
    ledger = UptakeLedger()
    first_key = _identity()
    second_key = _identity(sequence=2, generation=4)
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(first_key, 500, 120)
    ledger.record_proposal(second_key, 500, 121)
    _record_summary(
        ledger,
        first_key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=2,
    )
    for index, key in enumerate((first_key, second_key)):
        ledger.record_status(
            key,
            header_stamp_ns=201,
            first_uptake_pass=True,
            hold_cycle_index=1,
            reject_reason=0,
            receive_monotonic_ns=10 + index,
            accepted_monotonic_ns=12 + index,
            run_invalid=False,
            overflow_count=0,
            pp_cycle_sequence=1,
            availability_present=True,
            availability_identity=first_key,
            availability_safety_valid_until_ns=500,
            availability_transition=1,
            timer_entry_monotonic_ns=101,
            cycle_event_index=index,
            cycle_event_count=2,
        )
    for cycle in range(2, 11):
        _record_summary(
            ledger,
            first_key,
            cycle=cycle,
            hold_cycle_index=cycle,
            transition=2,
        )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "multiple_first_uptakes_in_pp_cycle" in result["reasons"]


def test_hybrid_summary_with_terminal_fields_is_malformed() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    ledger.record_status(
        key,
        header_stamp_ns=201,
        first_uptake_pass=True,
        hold_cycle_index=1,
        reject_reason=0,
        receive_monotonic_ns=10,
        accepted_monotonic_ns=11,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=1,
        availability_summary=True,
        availability_present=True,
        availability_identity=key,
        availability_safety_valid_until_ns=500,
        availability_transition=1,
        timer_entry_monotonic_ns=101,
        cycle_event_count=1,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "malformed_status" in result["reasons"]
    assert result["first_status_fault"]["failed_observer_predicates"] == [
        "summary_terminal_fields_neutral"
    ]


def test_realistic_long_stream_with_terminal_events_stays_bounded() -> None:
    ledger = UptakeLedger()
    ledger.record_clock(100)
    ledger.record_clock(200)
    active_key = _identity()
    generation = 3
    valid_until_ns = 10_000
    for cycle in range(1, 5501):
        has_new_proposal = cycle == 1 or cycle % 5 == 1
        if has_new_proposal:
            sequence = 1 if cycle == 1 else (cycle - 1) // 5 + 1
            generation += 0 if cycle == 1 else 1
            active_key = _identity(sequence=sequence, generation=generation)
            ledger.record_proposal(active_key, valid_until_ns, 100 + cycle)
        _record_summary(
            ledger,
            active_key,
            cycle=cycle,
            hold_cycle_index=1 if has_new_proposal else cycle,
            transition=(1 if cycle == 1 else 3 if has_new_proposal else 2),
            event_count=1 if has_new_proposal else 0,
            valid_until_ns=valid_until_ns,
        )
        if has_new_proposal:
            ledger.record_status(
                active_key,
                header_stamp_ns=200 + cycle,
                first_uptake_pass=True,
                hold_cycle_index=1,
                reject_reason=0,
                receive_monotonic_ns=10 + cycle,
                accepted_monotonic_ns=11 + cycle,
                run_invalid=False,
                overflow_count=0,
                pp_cycle_sequence=cycle,
                availability_present=True,
                availability_identity=active_key,
                availability_safety_valid_until_ns=valid_until_ns,
                availability_transition=(1 if cycle == 1 else 3),
                timer_entry_monotonic_ns=100 + cycle,
                cycle_event_index=0,
                cycle_event_count=1,
            )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert result["status_message_count"] > 5481
    assert result["stream_state"]["retained_identity_count"] < 2048
    assert result["stream_state"]["retained_s0_window_count"] == 256
    assert "availability_chain_capacity_exceeded" in result["reasons"]


def test_accepted_first_followed_by_holds_is_one_resolution_and_complete_cycles() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 1_000, 120)
    for cycle in range(1, 11):
        _record_summary(
            ledger,
            key,
            cycle=cycle,
            hold_cycle_index=cycle,
            transition=1 if cycle == 1 else 2,
            event_count=1,
            valid_until_ns=1_000,
        )
        ledger.record_status(
            key,
            header_stamp_ns=200 + cycle,
            first_uptake_pass=cycle == 1,
            hold_cycle_index=cycle,
            reject_reason=0,
            receive_monotonic_ns=10 if cycle == 1 else 0,
            accepted_monotonic_ns=11 if cycle == 1 else 0,
            run_invalid=False,
            overflow_count=0,
            pp_cycle_sequence=cycle,
            availability_present=True,
            availability_identity=key,
            availability_safety_valid_until_ns=1_000,
            availability_transition=1 if cycle == 1 else 2,
            timer_entry_monotonic_ns=100 + cycle,
            cycle_event_count=1,
        )
    _record_summary(
        ledger,
        key,
        cycle=11,
        hold_cycle_index=0,
        transition=0,
        availability_present=False,
        valid_until_ns=0,
        header_stamp_ns=1_000,
        event_count=1,
    )
    _record_cycle_event(
        ledger,
        key,
        cycle=11,
        first_uptake=False,
        hold_cycle_index=10,
        reject_reason=6,
        availability_present=False,
        availability_key=("", "", 0, 0, 0, 0, "", (0,) * 32),
        valid_until_ns=0,
        transition=0,
        header_stamp_ns=1_000,
    )
    result = ledger.terminal()
    assert result["outcome"] == "PASS"
    assert result["status_identity_count"] == 1
    assert result["availability"]["first_uptakes"][0]["event_count_for_identity"] == 1
    assert "duplicate_terminal_resolution" not in result["reasons"]
    assert "proposal_rejected" not in result["reasons"]


def test_first_reject_is_persisted_without_overwriting_later_events() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 1_000, 120)
    _record_summary(
        ledger,
        key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=1,
        valid_until_ns=1_000,
    )
    ledger.record_status(
        key,
        header_stamp_ns=201,
        first_uptake_pass=False,
        hold_cycle_index=1,
        reject_reason=7,
        receive_monotonic_ns=10,
        accepted_monotonic_ns=0,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=1,
        availability_present=True,
        availability_identity=key,
        availability_safety_valid_until_ns=1_000,
        availability_transition=1,
        timer_entry_monotonic_ns=101,
        cycle_event_count=1,
    )
    _record_summary(
        ledger,
        key,
        cycle=2,
        hold_cycle_index=2,
        transition=2,
        event_count=1,
        valid_until_ns=1_000,
    )
    ledger.record_status(
        key,
        header_stamp_ns=202,
        first_uptake_pass=False,
        hold_cycle_index=2,
        reject_reason=8,
        receive_monotonic_ns=0,
        accepted_monotonic_ns=0,
        run_invalid=False,
        overflow_count=0,
        pp_cycle_sequence=2,
        availability_present=True,
        availability_identity=key,
        availability_safety_valid_until_ns=1_000,
        availability_transition=2,
        timer_entry_monotonic_ns=102,
        cycle_event_count=1,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "proposal_rejected" in result["reasons"]
    assert result["first_reject"]["identity"] == {
        "producer_instance_id": "42",
        "session_id": "7",
        "proposal_sequence": 1,
        "plan_generation": 3,
        "source_generation": 2,
        "source_stamp_ns": 1_000,
        "frame_id": "map",
        "canonical_sha256": [1] + [0] * 31,
    }
    assert result["first_reject"]["reject_reason"] == 7
    assert result["first_reject"]["event"]["pp_cycle_sequence"] == 1


def _reason5_ledger(
    *,
    base_lease_ns: int = 1_090_000_000,
    final_deadline_ns: int = 1_050_000_000,
    previous_cycle: int = 1,
    rejecting_cycle: int = 2,
    previous_timer_ros_ns: int = 1_040_000_000,
    rejecting_timer_ros_ns: int = 1_060_000_000,
    previous_timer_monotonic_ns: int = 1_000,
    callback_monotonic_ns: int = 1_010,
    rejecting_timer_monotonic_ns: int = 1_020,
    source_kind: object = 1,
) -> UptakeLedger:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(
        key,
        final_deadline_ns,
        1_000_000_000,
        base_lease_valid_until_ns=base_lease_ns,
        base_source_kind=source_kind,
    )
    _record_summary(
        ledger,
        key,
        cycle=previous_cycle,
        hold_cycle_index=0,
        transition=0,
        event_count=0,
        availability_present=False,
        valid_until_ns=0,
        header_stamp_ns=previous_timer_ros_ns,
        timer_entry_monotonic_ns=previous_timer_monotonic_ns,
    )
    _record_summary(
        ledger,
        key,
        cycle=rejecting_cycle,
        hold_cycle_index=0,
        transition=0,
        event_count=1,
        availability_present=False,
        valid_until_ns=0,
        header_stamp_ns=rejecting_timer_ros_ns,
        timer_entry_monotonic_ns=rejecting_timer_monotonic_ns,
    )
    _record_cycle_event(
        ledger,
        key,
        cycle=rejecting_cycle,
        first_uptake=False,
        hold_cycle_index=0,
        reject_reason=5,
        availability_present=False,
        availability_key=("", "", 0, 0, 0, 0, "", (0,) * 32),
        valid_until_ns=0,
        transition=0,
        header_stamp_ns=rejecting_timer_ros_ns,
        timer_entry_monotonic_ns=rejecting_timer_monotonic_ns,
        receive_monotonic_ns=callback_monotonic_ns,
        accepted_monotonic_ns=0,
    )
    return ledger


@pytest.mark.parametrize(
    ("base_lease_ns", "final_deadline_ns", "previous_timer_ros_ns", "expected_branch"),
    [
        (1_040_000_000, 1_040_000_000, 1_030_000_000, "BASE_LEASE"),
        (1_090_000_000, 1_050_000_000, 1_040_000_000, "SHADOW_CAP"),
        (1_050_000_000, 1_050_000_000, 1_040_000_000, "TIE"),
    ],
)
def test_reason5_causal_provenance_classifies_effective_deadline_minimum(
    base_lease_ns: int,
    final_deadline_ns: int,
    previous_timer_ros_ns: int,
    expected_branch: str,
) -> None:
    ledger = _reason5_ledger(
        base_lease_ns=base_lease_ns,
        final_deadline_ns=final_deadline_ns,
        previous_timer_ros_ns=previous_timer_ros_ns,
    )
    causal = ledger.terminal()["initial_exact_uptake"]["reason5_causal_provenance"]
    assert causal["deadline"]["effective_deadline_selection"] == expected_branch
    assert causal["classification"] == (
        "MISSED_PRECEDING_TIMER_THEN_DEADLINE_REJECTED_AT_NEXT_TIMER"
    )
    assert causal["clock_domain_provenance"] == {
        "callback_and_timer_order": "CLOCK_MONOTONIC",
        "deadline_comparison": "ROS_TIME",
        "callback_ros_timestamp": None,
        "driver_tick_or_history_used": False,
    }


@pytest.mark.parametrize(
    "kwargs",
    [
        {"base_lease_ns": 1_040_000_000, "final_deadline_ns": 1_050_000_000},
        {"source_kind": None},
    ],
)
def test_reason5_causal_provenance_invalid_deadline_operands_are_unresolved(
    kwargs: dict[str, int | None]
) -> None:
    causal = _reason5_ledger(**kwargs).terminal()["initial_exact_uptake"][
        "reason5_causal_provenance"
    ]
    assert causal["deadline"]["effective_deadline_selection"] == "INVALID"
    assert causal["classification"] == "UNRESOLVED"


@pytest.mark.parametrize(
    ("source_kind", "expected_sticky_fault"),
    [
        (None, False),
        (False, True),
        (True, True),
        (0, True),
        (-1, True),
        (3, True),
        (255, True),
    ],
)
def test_reason5_causal_provenance_rejects_unknown_or_nonenum_source_kind(
    source_kind: object, expected_sticky_fault: bool
) -> None:
    ledger = _reason5_ledger(source_kind=source_kind)
    result = ledger.terminal()
    causal = result["initial_exact_uptake"]["reason5_causal_provenance"]
    assert causal["deadline"]["effective_deadline_selection"] == "INVALID"
    assert causal["classification"] == "UNRESOLVED"
    assert causal["predicates"]["base_source_kind_recognized"] is False
    assert ("invalid_base_source_kind" in result["reasons"]) is expected_sticky_fault


@pytest.mark.parametrize("source_kind", [1, 2])
def test_reason5_causal_provenance_accepts_only_production_source_kind_enum(
    source_kind: int,
) -> None:
    causal = _reason5_ledger(source_kind=source_kind).terminal()[
        "initial_exact_uptake"
    ]["reason5_causal_provenance"]
    assert causal["predicates"]["base_source_kind_recognized"] is True
    assert causal["classification"] == (
        "MISSED_PRECEDING_TIMER_THEN_DEADLINE_REJECTED_AT_NEXT_TIMER"
    )


def test_reason5_causal_provenance_requires_callback_between_consecutive_timers() -> None:
    for kwargs in (
        {"callback_monotonic_ns": 1_000},
        {"callback_monotonic_ns": 1_020},
        {"previous_cycle": 1, "rejecting_cycle": 3},
    ):
        causal = _reason5_ledger(**kwargs).terminal()["initial_exact_uptake"][
            "reason5_causal_provenance"
        ]
        assert causal["classification"] == "UNRESOLVED"


def test_reason5_causal_provenance_requires_next_timer_at_deadline() -> None:
    causal = _reason5_ledger(rejecting_timer_ros_ns=1_049_999_999).terminal()[
        "initial_exact_uptake"
    ]["reason5_causal_provenance"]
    assert causal["predicates"]["preceding_timer_before_effective_deadline"]
    assert not causal["predicates"][
        "rejecting_timer_at_or_after_effective_deadline"
    ]
    assert causal["classification"] == "UNRESOLVED"


def test_reason5_causal_provenance_stays_fixed_to_first_proposal() -> None:
    ledger = _reason5_ledger()
    later_key = _identity(sequence=2, generation=4)
    ledger.record_proposal(
        later_key,
        250,
        110,
        base_lease_valid_until_ns=250,
        base_source_kind=1,
    )
    causal = ledger.terminal()["initial_exact_uptake"]["reason5_causal_provenance"]
    assert causal["identity"]["proposal_sequence"] == 1
    assert causal["classification"] == (
        "MISSED_PRECEDING_TIMER_THEN_DEADLINE_REJECTED_AT_NEXT_TIMER"
    )


def test_resolution_capacity_never_evicts_to_pass() -> None:
    ledger = UptakeLedger(capacity=2)
    for sequence in range(1, 4):
        ledger.record_proposal(
            _identity(sequence=sequence, generation=sequence + 2), 500, 120
        )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert result["stream_state"]["retained_identity_count"] == 2
    assert result["stream_state"]["capacity_overflowed"] is True
    assert "proposal_ledger_overflow" in result["reasons"]
    assert "observer_ledger_overflow" in result["reasons"]


def test_atomic_json_is_host_readable(tmp_path) -> None:
    target = tmp_path / "observer.json"
    _write_json_atomic(target, {"outcome": "INVALID_EVIDENCE"})
    assert json.loads(target.read_text()) == {"outcome": "INVALID_EVIDENCE"}
    assert stat.S_IMODE(target.stat().st_mode) == 0o644
    assert list(tmp_path.glob("*.tmp")) == []


def test_atomic_replace_failure_preserves_old_result_and_cleans_temp(
    tmp_path, monkeypatch
) -> None:
    target = tmp_path / "observer.json"
    target.write_text('{"old": true}\n')

    def fail_replace(_source, _target) -> None:
        raise OSError("replace failed")

    monkeypatch.setattr(os, "replace", fail_replace)
    with pytest.raises(OSError, match="replace failed"):
        _write_json_atomic(target, {"new": True})
    assert json.loads(target.read_text()) == {"old": True}
    assert list(tmp_path.glob("*.tmp")) == []


def test_atomic_fsync_failure_cleans_temp(tmp_path, monkeypatch) -> None:
    target = tmp_path / "observer.json"

    def fail_fsync(_file_descriptor: int) -> None:
        raise OSError("fsync failed")

    monkeypatch.setattr(os, "fsync", fail_fsync)
    with pytest.raises(OSError, match="fsync failed"):
        _write_json_atomic(target, {"new": True})
    assert not target.exists()
    assert list(tmp_path.glob("*.tmp")) == []


def _accepted_then_stale(
    replacement_key: tuple[object, ...],
    *,
    accept_replacement: bool,
    availability_present: bool = True,
    transition: int = 3,
) -> UptakeLedger:
    ledger = UptakeLedger()
    old_key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(old_key, 202, 120)
    _record_summary(
        ledger,
        old_key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=1,
        valid_until_ns=202,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=1,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=0,
        availability_present=True,
        availability_key=old_key,
        valid_until_ns=202,
        transition=1,
        observer_receive_monotonic_ns=1_001,
    )
    if accept_replacement:
        ledger.record_proposal(replacement_key, 500, 121)
    summary_key = (
        replacement_key
        if availability_present
        else ("", "", 0, 0, 0, 0, "", (0,) * 32)
    )
    _record_summary(
        ledger,
        summary_key,
        cycle=2,
        hold_cycle_index=1,
        transition=transition,
        event_count=2 if accept_replacement else 1,
        valid_until_ns=500 if availability_present else 0,
        availability_present=availability_present,
        availability_key=summary_key,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=2,
        first_uptake=False,
        hold_cycle_index=2,
        reject_reason=6,
        availability_present=availability_present,
        availability_key=summary_key,
        valid_until_ns=500 if availability_present else 0,
        transition=transition,
        event_count=2 if accept_replacement else 1,
        accepted_monotonic_ns=11,
        observer_receive_monotonic_ns=1_002,
    )
    if accept_replacement:
        _record_cycle_event(
            ledger,
            replacement_key,
            cycle=2,
            first_uptake=True,
            hold_cycle_index=1,
            reject_reason=0,
            availability_present=True,
            availability_key=replacement_key,
            valid_until_ns=500,
            transition=transition,
            event_index=1,
            event_count=2,
            receive_monotonic_ns=20,
            accepted_monotonic_ns=21,
            observer_receive_monotonic_ns=1_003,
        )
    return ledger


def test_unresolved_reject_is_pre_uptake_reject_and_partitioned() -> None:
    ledger = UptakeLedger()
    key = _identity()
    none_key = ("", "", 0, 0, 0, 0, "", (0,) * 32)
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    _record_summary(
        ledger,
        none_key,
        cycle=1,
        hold_cycle_index=0,
        transition=0,
        event_count=1,
        valid_until_ns=0,
        availability_present=False,
        availability_key=none_key,
    )
    _record_cycle_event(
        ledger,
        key,
        cycle=1,
        first_uptake=False,
        hold_cycle_index=0,
        reject_reason=7,
        availability_present=False,
        availability_key=none_key,
        valid_until_ns=0,
        transition=0,
        accepted_monotonic_ns=0,
        observer_receive_monotonic_ns=2_001,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "pre_uptake_reject" in result["reasons"]
    assert result["first_reject"]["classification_result"] == "pre_uptake_reject"
    partition = result["proposal_terminal_partition"]
    assert partition["counts"] == {
        "P": 1,
        "A": 0,
        "Rpre": 1,
        "Rpost": 0,
        "U": 0,
    }
    assert all(partition["predicates"].values())


def test_post_uptake_stale_with_same_cycle_exact_replacement_is_legal() -> None:
    replacement_key = _identity(sequence=2, generation=4)
    ledger = _accepted_then_stale(replacement_key, accept_replacement=True)
    for cycle in range(3, 11):
        _record_summary(
            ledger,
            replacement_key,
            cycle=cycle,
            hold_cycle_index=cycle,
            transition=2,
            valid_until_ns=500,
        )
    _record_summary(
        ledger,
        replacement_key,
        cycle=11,
        hold_cycle_index=0,
        transition=0,
        availability_present=False,
        valid_until_ns=0,
        header_stamp_ns=500,
        event_count=1,
    )
    _record_cycle_event(
        ledger,
        replacement_key,
        cycle=11,
        first_uptake=False,
        hold_cycle_index=10,
        reject_reason=6,
        availability_present=False,
        availability_key=("", "", 0, 0, 0, 0, "", (0,) * 32),
        valid_until_ns=0,
        transition=0,
        header_stamp_ns=500,
    )
    result = ledger.terminal()
    assert result["outcome"] == "PASS"
    provenance = result["first_stale_provenance"]
    assert provenance["classification_result"] == "legal_retirement"
    assert provenance["proposal_safety_valid_until_ns"] == 202
    assert provenance["accepted_event"] == {
        "pp_cycle_sequence": 1,
        "timer_entry_ros_ns": 201,
        "status_header_stamp_ns": 201,
        "receive_monotonic_ns": 10,
        "accepted_monotonic_ns": 11,
        "observer_receive_monotonic_ns": 1_001,
    }
    assert provenance["stale_event"]["pp_cycle_sequence"] == 2
    assert provenance["stale_event"]["timer_entry_ros_ns"] == 202
    assert provenance["stale_event"]["observer_receive_monotonic_ns"] == 1_002
    assert provenance["stale_cycle_summary"]["transition"] == 3
    assert provenance["stale_cycle_summary"]["summary_deadline_ns"] == 500
    predecessor = provenance["cycle_entry_previous_summary"]
    assert predecessor["pp_cycle_sequence"] == 1
    assert predecessor["availability_identity"] == provenance["retired_identity"]
    assert predecessor["summary_deadline_ns"] == 202
    assert provenance["same_cycle_replacement"]["valid"] is True
    assert provenance["same_cycle_replacement"]["predicates"][
        "stale_cycle_complete"
    ] is True
    partition = result["proposal_terminal_partition"]
    assert partition["counts"] == {
        "P": 2,
        "A": 2,
        "Rpre": 0,
        "Rpost": 2,
        "U": 0,
    }
    assert all(partition["predicates"].values())
    assert "post_uptake_stale_without_legal_replacement" not in result["reasons"]


def test_post_uptake_stale_with_summary_none_is_availability_expired() -> None:
    ledger = _accepted_then_stale(
        _identity(sequence=2, generation=4),
        accept_replacement=False,
        availability_present=False,
        transition=0,
    )
    result = ledger.terminal()
    provenance = result["first_stale_provenance"]
    assert result["outcome"] == "PASS"
    assert provenance["classification_result"] == "availability_expired"
    assert provenance["stale_cycle_summary"]["availability_present"] is False
    predicates = provenance["same_cycle_replacement"]["predicates"]
    assert predicates["transition_replaced"] is False
    assert "post_uptake_stale_without_legal_replacement" not in result["reasons"]


@pytest.mark.parametrize(
    ("replacement_key", "accept_replacement", "failed_predicate"),
    [
        (
            _identity(sequence=2, generation=4),
            False,
            "replacement_exact_first_uptake",
        ),
        (
            _identity(sequence=2, generation=4, producer="other"),
            True,
            "same_producer_session",
        ),
        (
            _identity(sequence=2, generation=4, session="other"),
            True,
            "same_producer_session",
        ),
        (
            _identity(sequence=2, generation=3),
            True,
            "plan_generation_strictly_newer",
        ),
    ],
)
def test_post_uptake_stale_rejects_illegal_replacement(
    replacement_key: tuple[object, ...],
    accept_replacement: bool,
    failed_predicate: str,
) -> None:
    result = _accepted_then_stale(
        replacement_key, accept_replacement=accept_replacement
    ).terminal()
    provenance = result["first_stale_provenance"]
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert provenance["classification_result"] == "availability_expired"
    assert provenance["same_cycle_replacement"]["valid"] is False
    predicates = provenance["same_cycle_replacement"]["predicates"]
    assert predicates[failed_predicate] is False


def test_orphan_continuation_is_sticky_invalid() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    _record_summary(
        ledger,
        key,
        cycle=1,
        hold_cycle_index=2,
        transition=2,
        event_count=1,
    )
    _record_cycle_event(
        ledger,
        key,
        cycle=1,
        first_uptake=False,
        hold_cycle_index=2,
        reject_reason=0,
        availability_present=True,
        availability_key=key,
        valid_until_ns=500,
        transition=2,
        observer_receive_monotonic_ns=3_001,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "orphan_continuation" in result["reasons"]
    classification = result["first_lifecycle_classification"]
    assert classification["classification_result"] == "contradictory_event"


def test_first_uptake_with_reject_is_contradictory_and_sticky_invalid() -> None:
    ledger = UptakeLedger()
    key = _identity()
    none_key = ("", "", 0, 0, 0, 0, "", (0,) * 32)
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    _record_summary(
        ledger,
        none_key,
        cycle=1,
        hold_cycle_index=0,
        transition=0,
        event_count=1,
        valid_until_ns=0,
        availability_present=False,
        availability_key=none_key,
    )
    _record_cycle_event(
        ledger,
        key,
        cycle=1,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=6,
        availability_present=False,
        availability_key=none_key,
        valid_until_ns=0,
        transition=0,
        accepted_monotonic_ns=0,
        observer_receive_monotonic_ns=4_001,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "contradictory_first_uptake_event" in result["reasons"]
    classification = result["first_lifecycle_classification"]
    assert classification["classification_result"] == "contradictory_event"
    partition = result["proposal_terminal_partition"]
    assert partition["counts"]["P"] == 1
    assert partition["counts"]["A"] == 0
    assert partition["counts"]["Rpre"] == 0
    assert partition["counts"]["U"] == 1
    assert partition["primary_resolution_counts"][0]["count"] == 1


def test_post_stale_provenance_remains_bounded_after_long_tail() -> None:
    replacement_key = _identity(sequence=2, generation=4)
    ledger = _accepted_then_stale(replacement_key, accept_replacement=True)
    for cycle in range(3, 5_502):
        _record_summary(
            ledger,
            replacement_key,
            cycle=cycle,
            hold_cycle_index=cycle,
            transition=2,
            valid_until_ns=10_000,
        )
    result = ledger.terminal()
    assert result["stream_state"]["retained_identity_count"] == 2
    assert result["stream_state"]["retained_s0_window_count"] == 256
    assert "availability_chain_capacity_exceeded" in result["reasons"]
    assert result["proposal_terminal_partition"]["counts"]["Rpost"] == 1
    assert isinstance(result["first_stale_provenance"], dict)


def test_unrelated_old_stale_cannot_piggyback_on_separate_replacement() -> None:
    ledger = UptakeLedger()
    old_key = _identity(sequence=1, generation=3)
    active_key = _identity(sequence=2, generation=4)
    replacement_key = _identity(sequence=3, generation=5)
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(old_key, 203, 120)
    _record_summary(
        ledger,
        old_key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=1,
        valid_until_ns=203,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=1,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=0,
        availability_present=True,
        availability_key=old_key,
        valid_until_ns=203,
        transition=1,
    )
    ledger.record_proposal(active_key, 500, 121)
    _record_summary(
        ledger,
        active_key,
        cycle=2,
        hold_cycle_index=1,
        transition=3,
        event_count=1,
        valid_until_ns=500,
    )
    _record_cycle_event(
        ledger,
        active_key,
        cycle=2,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=0,
        availability_present=True,
        availability_key=active_key,
        valid_until_ns=500,
        transition=3,
    )
    ledger.record_proposal(replacement_key, 600, 122)
    _record_summary(
        ledger,
        replacement_key,
        cycle=3,
        hold_cycle_index=1,
        transition=3,
        event_count=2,
        valid_until_ns=600,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=3,
        first_uptake=False,
        hold_cycle_index=3,
        reject_reason=6,
        availability_present=True,
        availability_key=replacement_key,
        valid_until_ns=600,
        transition=3,
        event_count=2,
        observer_receive_monotonic_ns=5_001,
    )
    _record_cycle_event(
        ledger,
        replacement_key,
        cycle=3,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=0,
        availability_present=True,
        availability_key=replacement_key,
        valid_until_ns=600,
        transition=3,
        event_index=1,
        event_count=2,
        observer_receive_monotonic_ns=5_002,
    )
    result = ledger.terminal()
    provenance = result["first_stale_provenance"]
    predicates = provenance["same_cycle_replacement"]["predicates"]
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert provenance["classification_result"] == "availability_expired"
    assert predicates["previous_active_identity_is_retired"] is False
    assert provenance["cycle_entry_previous_summary"][
        "availability_identity"
    ] != provenance["retired_identity"]


def test_stale_before_summary_and_new_proposal_resolves_after_cycle_closure() -> None:
    ledger = UptakeLedger()
    old_key = _identity(sequence=1, generation=3)
    replacement_key = _identity(sequence=2, generation=4)
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(old_key, 202, 120)
    _record_summary(
        ledger,
        old_key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=1,
        valid_until_ns=202,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=1,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=0,
        availability_present=True,
        availability_key=old_key,
        valid_until_ns=202,
        transition=1,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=2,
        first_uptake=False,
        hold_cycle_index=2,
        reject_reason=6,
        availability_present=True,
        availability_key=replacement_key,
        valid_until_ns=500,
        transition=3,
        event_count=2,
        observer_receive_monotonic_ns=6_001,
    )
    assert ledger.pending_cycle_sequence == 2
    _record_summary(
        ledger,
        replacement_key,
        cycle=2,
        hold_cycle_index=1,
        transition=3,
        event_count=2,
        valid_until_ns=500,
    )
    _record_cycle_event(
        ledger,
        replacement_key,
        cycle=2,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=0,
        availability_present=True,
        availability_key=replacement_key,
        valid_until_ns=500,
        transition=3,
        event_index=1,
        event_count=2,
        observer_receive_monotonic_ns=6_002,
    )
    ledger.record_proposal(replacement_key, 500, 121)
    for cycle in range(3, 11):
        _record_summary(
            ledger,
            replacement_key,
            cycle=cycle,
            hold_cycle_index=cycle,
            transition=2,
            valid_until_ns=500,
        )
    _record_summary(
        ledger,
        replacement_key,
        cycle=11,
        hold_cycle_index=0,
        transition=0,
        availability_present=False,
        valid_until_ns=0,
        header_stamp_ns=500,
        event_count=1,
    )
    _record_cycle_event(
        ledger,
        replacement_key,
        cycle=11,
        first_uptake=False,
        hold_cycle_index=10,
        reject_reason=6,
        availability_present=False,
        availability_key=("", "", 0, 0, 0, 0, "", (0,) * 32),
        valid_until_ns=0,
        transition=0,
        header_stamp_ns=500,
    )
    result = ledger.terminal()
    assert result["outcome"] == "PASS"
    assert result["first_stale_provenance"][
        "classification_result"
    ] == "legal_retirement"
    assert result["first_stale_provenance"]["stale_cycle_summary"][
        "cycle_complete"
    ] is True
    assert result["stream_state"]["pending_cycle_event_count"] == 0


def test_unclosed_pending_stale_is_sticky_invalid_at_terminal() -> None:
    ledger = UptakeLedger()
    old_key = _identity(sequence=1, generation=3)
    replacement_key = _identity(sequence=2, generation=4)
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(old_key, 202, 120)
    _record_summary(
        ledger,
        old_key,
        cycle=1,
        hold_cycle_index=1,
        transition=1,
        event_count=1,
        valid_until_ns=202,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=1,
        first_uptake=True,
        hold_cycle_index=1,
        reject_reason=0,
        availability_present=True,
        availability_key=old_key,
        valid_until_ns=202,
        transition=1,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=2,
        first_uptake=False,
        hold_cycle_index=2,
        reject_reason=6,
        availability_present=True,
        availability_key=replacement_key,
        valid_until_ns=500,
        transition=3,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "pending_terminal_summary_missing" in result["reasons"]
    assert result["stream_state"]["pending_cycle_event_count"] == 1


@pytest.mark.parametrize(
    ("reject_reason", "hold_cycle_index"),
    [(0, 3), (6, 3), (7, 3)],
)
def test_retired_identity_reappearance_is_counted_and_sticky_invalid(
    reject_reason: int, hold_cycle_index: int
) -> None:
    old_key = _identity(sequence=1, generation=3)
    replacement_key = _identity(sequence=2, generation=4)
    ledger = _accepted_then_stale(replacement_key, accept_replacement=True)
    _record_summary(
        ledger,
        replacement_key,
        cycle=3,
        hold_cycle_index=2,
        transition=2,
        event_count=1,
        valid_until_ns=500,
    )
    _record_cycle_event(
        ledger,
        old_key,
        cycle=3,
        first_uptake=False,
        hold_cycle_index=hold_cycle_index,
        reject_reason=reject_reason,
        availability_present=True,
        availability_key=replacement_key,
        valid_until_ns=500,
        transition=2,
        observer_receive_monotonic_ns=7_001,
    )
    result = ledger.terminal()
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "retired_identity_reappeared" in result["reasons"]
    latch = result["post_stale_latches"][0]
    assert latch["reappeared"] is True
    assert latch["reappearance_count"] == 1


def test_duplicate_primary_resolution_remains_in_u_with_count() -> None:
    ledger = UptakeLedger()
    key = _identity()
    ledger.record_clock(100)
    ledger.record_clock(200)
    ledger.record_proposal(key, 500, 120)
    for cycle in (1, 2):
        _record_summary(
            ledger,
            key,
            cycle=cycle,
            hold_cycle_index=1,
            transition=1 if cycle == 1 else 2,
            event_count=1,
        )
        _record_cycle_event(
            ledger,
            key,
            cycle=cycle,
            first_uptake=True,
            hold_cycle_index=1,
            reject_reason=0,
            availability_present=True,
            availability_key=key,
            valid_until_ns=500,
            transition=1 if cycle == 1 else 2,
        )
    result = ledger.terminal()
    partition = result["proposal_terminal_partition"]
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "primary_resolution_multiplicity_invalid" in result["reasons"]
    assert partition["counts"]["A"] == 0
    assert partition["counts"]["U"] == 1
    assert partition["primary_resolution_counts"][0]["count"] == 2
