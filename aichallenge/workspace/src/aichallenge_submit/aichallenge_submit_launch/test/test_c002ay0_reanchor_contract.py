#!/usr/bin/env python3

import ast
import copy
import inspect
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import TrajectoryPoint
from multi_purpose_mpc_ros_msgs.msg import (
    AuthorizedCartesianTrajectory,
    ControllerCommandEnvelope,
    ControllerExecutionEnvelope,
    ControllerTrackingStatus,
    ExecutionSweepSample,
)
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String

from c002ay0_pp_runtime_measurement import (
    ALIGNMENT_STAGE_MAX_MESSAGES,
    CAPTURE_TOPICS,
    E2E_INPUT_PUBLISH_TICK_LEDGER_LIMIT,
    E2E_PROPOSAL_AUDIT_LEDGER_LIMIT,
    E2E_OUT_OF_COHORT_SAMPLE_LIMIT,
    E2E_SAFETY_TIMELINE_LIMIT,
    PREARM_CAPTURE_CAPACITY_MESSAGES,
    PREARM_STORAGE_BUDGET_MS,
    PREARM_STORAGE_PHASE_BUDGET_MS,
    PREARM_STORAGE_REQUIRED_MESSAGES,
    FIXTURE_DRAIN_DEADLINE_GUARD_SEC,
    FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK,
    FIXTURE_DRAIN_SPIN_WAIT_SEC,
    FIXTURE_INPUT_PERIOD_SEC,
    FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY,
    FIXED_SEC,
    SCHEDULING_EVIDENCE_MAX_SEC,
    STARTUP_LEDGER_MAX_MESSAGES,
    Capture,
    canonical_debug,
    canonical_payload_hash,
    cleanup_pre_run_case_failure,
    create_measurement_node,
    drive_isolated_observer_drain_step,
    drive_paced_fixture_tick,
    json_safe_artifact,
    initial_scheduling_evidence,
    normalize_timing_fields,
    primary_first_false_record,
    record_fixture_spin_observation,
    refresh_isolated_publisher_lifecycle,
    require_scheduling_evidence_elapsed,
    run_case,
    scheduling_executor_lifecycle_valid,
    spin_fixture_once_with_observation,
    stop_process,
    strict_json_text,
    synchronize_isolated_input_phase,
    validate_binding_e2e_wcet,
    validate_post_ack_exact_cohort,
)
from run_c002ay0_formal_wcet import (
    DDS_ENVIRONMENT_CONTRACT,
    MARKERS,
    classify_single_scheduling_result,
    configure_formal_dds_environment,
    configure_idle_break_diagnostic_environment,
    configure_isolated_input_publisher_environment,
    diagnostic_mode_markers,
    inspect_scheduling_evidence,
    isolated_publisher_lifecycle_is_closed,
    parse_args,
    process_identity_snapshot,
    reconcile_fixture_session_processes,
    require_formal_dds_environment,
    session_process_snapshots,
    validate_isolated_input_publisher_cli,
    validate_domains,
)


def legacy_canonical_debug(topic: str, message: object) -> dict:
    canonical = copy.deepcopy(message)
    if topic == "tracking":
        canonical.command_age_sec = 0.0
    elif topic == "command_envelope":
        canonical.producer_instance_id = 1
        canonical.command_sequence = 1
        canonical.command_age_sec = 0.0
    elif topic == "execution_envelope":
        canonical.producer_instance_id = 1
        canonical.command_sequence = 1
        canonical.command_envelope.producer_instance_id = 1
        canonical.command_envelope.command_sequence = 1
        canonical.command_envelope.command_age_sec = 0.0
        canonical.witness.source_generation = 1
        canonical.witness.base_source_generation = 1
    semantic = message_to_ordereddict(canonical)
    normalize_timing_fields(semantic)
    return semantic


def valid_isolated_publisher_lifecycle(
    pid: int = 1234,
) -> dict[str, object]:
    identity = {
        "pid": pid,
        "readable": True,
        "classification": "live",
        "state": "S",
        "ppid": 1,
        "session_id": 1230,
        "starttime_ticks": 99,
        "cmdline": "synthetic",
        "comm": "synthetic",
        "cgroup": ["0::/synthetic"],
    }
    return {
        "pid": pid,
        "role": "isolated_input_publisher",
        "provenance": "measurement_spawn_isolated_input_publisher",
        "cleanup_stage": "closed_no_residue",
        "alive": False,
        "exitcode": 0,
        "unresolved": False,
        "identity_before_cleanup": identity,
        "target_process_lifecycle": {
            "pid": pid,
            "provenance": "exact_direct_popen_child_delta",
            "cleanup_stage": "absent_no_residue",
            "alive": False,
            "exitcode": 0,
            "unresolved": False,
            "cleanup_failure": "",
            "descendants_before_stop": [],
            "descendants_poststop": [],
            "identity_poststop": {
                "pid": pid,
                "readable": False,
                "classification": "absent",
            },
        },
        "target_cleanup_failure": "",
        "associated_child_lifecycle": [],
        "associated_child_cleanup_failure": "",
        "associated_child_sample_failures": [],
    }


class ReanchorContractTest(unittest.TestCase):
    def test_measurement_node_uses_ros_sim_clock_without_wall_fallback(
        self,
    ) -> None:
        expected_node = object()
        with patch(
            "c002ay0_pp_runtime_measurement.rclpy.create_node",
            return_value=expected_node,
        ) as create_node:
            node = create_measurement_node()

        self.assertIs(node, expected_node)
        create_node.assert_called_once()
        self.assertEqual(
            create_node.call_args.args,
            ("c002ay0_pp_runtime_measurement",),
        )
        overrides = create_node.call_args.kwargs["parameter_overrides"]
        self.assertEqual(len(overrides), 1)
        self.assertEqual(overrides[0].name, "use_sim_time")
        self.assertIs(overrides[0].value, True)

    def setUp(self) -> None:
        self.capture = Capture.__new__(Capture)
        self.capture.cohort_alignment_enabled = True
        self.capture.enabled = False
        self.capture.capture_after_stamp_ns = None
        self.capture.startup_anchor_stamp_ns = None
        self.capture.startup_anchor_sequence = None
        self.capture.startup_anchor_producer_instance_id = None
        self.capture.startup_ledger_overflow = False
        self.capture.startup_ledger = []
        self.capture.alignment_stage_overflow = False
        self.capture.alignment_stage_message_count = 0
        self.capture.alignment_stage_max_message_count = 0
        self.capture.alignment_stage_mismatch_reason = ""
        self.capture.reanchor_fence_invalid = None
        self.capture.alignment_cycles = {}
        self.capture.next_expected_sequence = None
        self.capture.materialized_sequences = []
        self.capture.values = {}
        self.capture.identities = {}
        self.capture.canonical_semantics = {}
        self.capture.first_debug = {}
        self.capture.max_abs_steering = {}

    @staticmethod
    def e2e_ledger_capture() -> Capture:
        capture = Capture.__new__(Capture)
        capture.max_snapshot_race_arm_epoch = 1
        capture.binding_dispositions = {}
        capture.e2e_proposals = {}
        capture.e2e_observed_binding_keys = set()
        capture.e2e_terminals = {}
        capture.e2e_proposal_audits = {}
        capture.e2e_planner_input_audits = {}
        capture.e2e_proposal_audit_overflow = False
        capture.e2e_proposal_audit_invalid = False
        capture.e2e_planner_input_audit_overflow = False
        capture.e2e_planner_input_audit_invalid = False
        capture.e2e_duplicate_proposal_audits = 0
        capture.e2e_duplicate_planner_input_audits = 0
        capture.begin_e2e_measurement(2)
        return capture

    @staticmethod
    def e2e_key(
        attempt_id: int,
        controller_sequence: int | None = None,
    ) -> tuple[int, ...]:
        if controller_sequence is None:
            controller_sequence = attempt_id
        return (
            2,
            7001,
            attempt_id,
            attempt_id,
            11,
            attempt_id,
            9001,
            8001,
            6001,
            controller_sequence,
            5001,
        )

    @staticmethod
    def e2e_terminal(
        disposition: str,
        *,
        callback_monotonic_ns: int = 1,
        lateral_authority_eligible: bool = False,
    ) -> dict[str, int | str | bool]:
        return {
            "disposition": disposition,
            "binding_callback_monotonic_ns": callback_monotonic_ns,
            "lateral_authority_eligible": lateral_authority_eligible,
            "callback_to_audit_observer_ns": 1,
            "source_age_at_bind_ns": 1,
            "safety_margin_at_bind_ns": 40_000_000,
            "binding_ros_now_ns": 160_000_000,
            "safety_valid_until_ns": 200_000_000,
            "binding_snapshot_evidence_v1": {
                "proposal": {
                    "source_stamp_sec": 1,
                    "source_stamp_nanosec": 0,
                    "source_generation": 11,
                }
            },
        }

    @staticmethod
    def e2e_proposal() -> dict[str, int]:
        return {
            "plan_stamp_ns": 100_000_000,
            "safety_evaluation_stamp_ns": 100_000_000,
            "safety_valid_until_ns": 200_000_000,
            "received_sim_ns": 130_000_000,
        }

    @classmethod
    def proposal_audit_payload(cls, attempt_id: int = 77, *, epoch: int = 2) -> dict[str, int]:
        key = cls.e2e_key(attempt_id)
        return {
            "schema_version": 1,
            "proposal_race_arm_epoch": epoch,
            "proposal_planner_instance_id": key[1],
            "proposal_attempt_id": key[2],
            "proposal_connector_transaction_id": key[3],
            "proposal_plan_generation": key[4],
            "proposal_candidate_revision": key[5],
            "proposal_authority_token": key[6],
            "proposal_safety_snapshot_id": key[7],
            "proposal_controller_instance_id": key[8],
            "proposal_controller_sequence": key[9],
            "proposal_base_lease_id": key[10],
            "capture_monotonic_ns": 10,
            "worker_dequeue_monotonic_ns": 20,
            "worker_build_begin_monotonic_ns": 30,
            "publish_call_entry_monotonic_ns": 40,
            "publish_call_return_monotonic_ns": 50,
        }

    @classmethod
    def planner_input_audit_payload(
        cls, attempt_id: int = 77, *, epoch: int = 2
    ) -> dict[str, int]:
        payload = cls.proposal_audit_payload(attempt_id, epoch=epoch)
        payload.update(
            {
                "proposal_source_stamp_sec": 1,
                "proposal_source_stamp_nanosec": 0,
                "proposal_source_generation": 11,
                "base_snapshot_callback_entry_monotonic_ns": 10,
                "planning_timer_entry_monotonic_ns": 20,
                "planner_update_begin_monotonic_ns": 30,
                "capture_monotonic_ns": 40,
            }
        )
        return payload

    @classmethod
    def binding_audit_payload(
        cls,
        *,
        epoch: int = 2,
        attempt_id: int = 77,
        disposition: str = "exact_current",
        lateral_authority_eligible: bool = False,
    ) -> dict[str, int | str | bool]:
        key = list(cls.e2e_key(attempt_id))
        key[0] = epoch
        return {
            "schema_version": 1,
            "test_audit_schema_version": 1,
            "shadow_only": True,
            "lateral_authority_eligible": lateral_authority_eligible,
            "disposition": disposition,
            "validation_error": 0,
            "proposal_race_arm_epoch": key[0],
            "proposal_planner_instance_id": key[1],
            "proposal_attempt_id": key[2],
            "proposal_connector_transaction_id": key[3],
            "proposal_plan_generation": key[4],
            "proposal_candidate_revision": key[5],
            "proposal_authority_token": key[6],
            "proposal_safety_snapshot_id": key[7],
            "proposal_controller_instance_id": key[8],
            "proposal_controller_sequence": key[9],
            "proposal_base_lease_id": key[10],
            "current_race_arm_epoch": epoch,
            "current_controller_instance_id": key[8],
            "current_controller_sequence": key[9],
            "current_base_lease_id": key[10],
            "current_source_stamp_sec": 1,
            "current_source_stamp_nanosec": 0,
            "current_source_generation": 11,
            "current_first_source_index": 10,
            "current_last_source_index": 20,
            "current_nearest_source_index": 12,
            "previous_present": True,
            "previous_race_arm_epoch": epoch,
            "previous_controller_instance_id": key[8],
            "previous_controller_sequence": key[9] - 1,
            "previous_base_lease_id": key[10] - 1,
            "previous_source_stamp_sec": 0,
            "previous_source_stamp_nanosec": 990_000_000,
            "previous_source_generation": 11,
            "previous_first_source_index": 10,
            "previous_last_source_index": 20,
            "previous_nearest_source_index": 11,
            "proposal_source_stamp_sec": 1,
            "proposal_source_stamp_nanosec": 0,
            "proposal_source_generation": 11,
            "proposal_nearest_source_index": 12,
            "proposal_safety_valid_until_sec": 2,
            "proposal_safety_valid_until_nanosec": 0,
            "binding_ros_now_sec": 1,
            "binding_ros_now_nanosec": 100_000_000,
            "binding_callback_monotonic_ns": 10_000,
        }

    @staticmethod
    def authorized_proposal_message(
        plan_stamp_ns: int,
    ) -> AuthorizedCartesianTrajectory:
        message = AuthorizedCartesianTrajectory()
        message.schema_version = (
            AuthorizedCartesianTrajectory.SCHEMA_V1_SHADOW
        )
        message.authority_eligible = False
        message.plan_sample_key.race_arm_epoch = 2
        message.plan_sample_key.planner_instance_id = 7001
        message.plan_sample_key.attempt_id = 77
        message.plan_sample_key.connector_transaction_id = 77
        message.plan_sample_key.plan_generation = 11
        message.candidate_revision = 77
        message.authority_token = 9001
        message.safety_snapshot_id = 8001
        message.source_controller_instance_id = 6001
        message.source_controller_sequence = 77
        message.base_lease_id = 5001
        message.plan_stamp.nanosec = plan_stamp_ns
        message.safety_evaluation_stamp.nanosec = plan_stamp_ns
        message.base_source_stamp.nanosec = plan_stamp_ns - 10_000_000
        message.safety_valid_until.nanosec = plan_stamp_ns + 50_000_000
        return message

    @staticmethod
    def binding_e2e_result(e2e_summary: dict[str, object]) -> tuple:
        signature = (("command", tuple(range(16))),)
        records = [(0, index * 1_000_000, 1) for index in range(1_000)]
        return (
            signature,
            records,
            0,
            None,
            {},
            1,
            False,
            0,
            0,
            int(e2e_summary.get("fixture_proposal_seen", 0)),
            int(e2e_summary.get("binding_callback_entries", 0)),
            int(e2e_summary.get("terminal_bindings", 0)),
            False,
            {},
            {},
            e2e_summary,
        )

    @staticmethod
    def exact_terminal_summary(
        *,
        binding_only: int = 0,
        fixture_only: int = 0,
        observer_missing: int = 0,
        observer_without_audit: int = 0,
    ) -> dict[str, object]:
        terminal_count = max(1, binding_only)
        return {
            "expected_race_arm_epoch": 2,
            "fixture_proposal_seen": terminal_count - binding_only,
            "binding_callback_entries": terminal_count,
            "terminal_bindings": terminal_count,
            "duplicate_proposals": 0,
            "duplicate_bindings": 0,
            "out_of_cohort_proposals": 0,
            "out_of_cohort_bindings": 0,
            "out_of_cohort_binding_audits": 0,
            "fixture_only_identities": fixture_only,
            "binding_only_identities": binding_only,
            "binding_observer_missing": observer_missing,
            "binding_observer_without_audit": observer_without_audit,
            "safety_margin_at_bind_min_ns": 1,
            "proposal_uptake_contract_v1": {
                "schema_version": 1,
                "expected_proposal_count": terminal_count,
                "accepted_first_uptake_count": terminal_count,
                "failed_or_missing_count": 0,
                "all_proposals_taken_up_once_before_deadline": True,
                "pp_hold_cycles_evaluated": False,
                "sustained_10_cycle_gate": "not_evaluated",
                "records": [],
            },
            "terminal_provenance": [
                {"disposition": "exact_current", "out_of_cohort": False}
                for _ in range(terminal_count)
            ],
        }

    def test_binding_only_terminal_audits_are_coverage_warning(self) -> None:
        summary = self.exact_terminal_summary(
            binding_only=6,
            observer_missing=1,
        )
        on_result = self.binding_e2e_result(summary)
        off_result = list(on_result)
        off_result[10] = 0
        with patch("builtins.print") as output:
            validate_binding_e2e_wcet(
                {
                    "off": tuple(off_result),
                    "binding_on_a": on_result,
                    "binding_on_b": on_result,
                }
            )

        lines = [str(call.args[0]) for call in output.call_args_list]
        self.assertIn("C002AY0_PP_BINDING_FULL_DELIVERY_E2E=FAIL", lines)
        self.assertIn(
            "C002AY0_PP_BINDING_OBSERVED_TERMINAL_AUDIT_INTEGRITY=PASS",
            lines,
        )

    def test_missing_terminal_audit_remains_hard_failure(self) -> None:
        for field, expected in (
            ("fixture_only", "fixture_only_identities=1"),
            (
                "observer_without_audit",
                "binding_observer_without_audit=1",
            ),
        ):
            with self.subTest(field=field):
                summary = self.exact_terminal_summary(**{field: 1})
                on_result = self.binding_e2e_result(summary)
                off_result = list(on_result)
                off_result[10] = 0
                with self.assertRaisesRegex(AssertionError, expected):
                    validate_binding_e2e_wcet(
                        {
                            "off": tuple(off_result),
                            "binding_on_a": on_result,
                            "binding_on_b": on_result,
                        }
                    )

    def test_formal_runner_requires_terminal_audit_marker(self) -> None:
        self.assertIn(
            "C002AY0_PP_BINDING_OBSERVED_TERMINAL_AUDIT_INTEGRITY",
            MARKERS,
        )

    def test_e2e_deferred_predecessor_has_no_delivery_mismatch(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(1)
        capture.e2e_proposals[key] = self.e2e_proposal()
        capture.e2e_observed_binding_keys.add(key)
        capture.e2e_terminals[key] = self.e2e_terminal(
            "deferred_predecessor"
        )
        capture.binding_dispositions["deferred_predecessor"] = 1

        summary = capture.finalize_e2e_measurement()

        self.assertEqual(
            capture.binding_dispositions,
            {"deferred_predecessor": 1},
        )
        self.assertEqual(summary["fixture_proposal_seen"], 1)
        self.assertEqual(summary["terminal_bindings"], 1)
        self.assertEqual(summary["intersection_identities"], 1)
        self.assertEqual(summary["fixture_only_identities"], 0)
        self.assertEqual(summary["binding_only_identities"], 0)
        self.assertEqual(summary["binding_observer_missing"], 0)
        self.assertEqual(summary["binding_observer_without_audit"], 0)
        self.assertEqual(summary["out_of_cohort_proposals"], 0)
        self.assertEqual(summary["out_of_cohort_bindings"], 0)
        self.assertEqual(summary["out_of_cohort_binding_audits"], 0)
        self.assertEqual(
            summary["deferred_predecessor_nonauthority_count"], 1
        )
        self.assertFalse(
            summary["deferred_predecessor_authority_violation"]
        )
        uptake = summary["proposal_uptake_contract_v1"]
        self.assertFalse(
            uptake["all_proposals_taken_up_once_before_deadline"]
        )
        self.assertEqual(
            uptake["records"][0]["failure_reasons"],
            [
                "missing_proposal_publish_audit",
                "not_exact_current",
                "source_identity_not_current",
                "binding_before_publish_return",
            ],
        )
        self.assertEqual(
            summary["terminal_provenance"],
            [
                {
                    "full_terminal_key": key,
                    "terminal_ordinal": 1,
                    "disposition": "deferred_predecessor",
                    "binding_callback_monotonic_ns": 1,
                    "binding_snapshot_evidence_v1": {
                        "proposal": {
                            "source_stamp_sec": 1,
                            "source_stamp_nanosec": 0,
                            "source_generation": 11,
                        }
                    },
                    "lateral_authority_eligible": False,
                    "observed_exact_key": key,
                    "observer_missing": False,
                    "binding_only": False,
                    "out_of_cohort": False,
                }
            ],
        )

    def test_binding_audit_snapshot_evidence_preserves_both_identities(
        self,
    ) -> None:
        evidence = Capture._binding_snapshot_evidence(
            self.binding_audit_payload()
        )
        self.assertIsNotNone(evidence)
        assert evidence is not None
        self.assertEqual(evidence["schema_version"], 1)
        self.assertEqual(
            evidence["current"]["controller_sequence"],
            77,
        )
        self.assertEqual(
            evidence["previous"]["controller_sequence"],
            76,
        )
        self.assertEqual(evidence["current"]["base_lease_id"], 5001)
        self.assertEqual(evidence["previous"]["base_lease_id"], 5000)
        self.assertEqual(
            evidence["proposal"],
            {
                "source_stamp_sec": 1,
                "source_stamp_nanosec": 0,
                "source_generation": 11,
            },
        )
        self.assertEqual(
            Capture._proposal_source_identity_relation(evidence),
            "exact_current",
        )

    def test_binding_audit_source_identity_relation_distinguishes_previous_and_neither(
        self,
    ) -> None:
        previous = self.binding_audit_payload()
        previous["proposal_source_stamp_sec"] = 0
        previous["proposal_source_stamp_nanosec"] = 990_000_000
        evidence = Capture._binding_snapshot_evidence(previous)
        self.assertIsNotNone(evidence)
        self.assertEqual(
            Capture._proposal_source_identity_relation(evidence),
            "exact_previous",
        )

        neither = self.binding_audit_payload()
        neither["proposal_source_generation"] = 12
        evidence = Capture._binding_snapshot_evidence(neither)
        self.assertIsNotNone(evidence)
        self.assertEqual(
            Capture._proposal_source_identity_relation(evidence),
            "neither_current_nor_previous",
        )

    def test_out_of_cohort_binding_audit_preserves_bounded_sample(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        capture.node = Mock()
        capture.node.get_clock.return_value.now.return_value.nanoseconds = (
            1_200_000_000
        )
        payload = self.binding_audit_payload(epoch=3)

        capture._receive_audited_binding(
            String(data=json.dumps(payload))
        )

        self.assertEqual(capture.e2e_out_of_cohort_binding_audits, 1)
        self.assertFalse(capture.binding_invalid)
        self.assertEqual(capture.e2e_terminals, {})
        sample = capture.e2e_out_of_cohort_samples["binding_audit"][0]
        self.assertEqual(sample["schema_version"], 1)
        self.assertEqual(sample["stream"], "binding_audit")
        self.assertEqual(sample["expected_race_arm_epoch"], 2)
        self.assertEqual(sample["observed_race_arm_epoch"], 3)
        self.assertEqual(sample["full_proposal_key"][0], 3)
        self.assertEqual(sample["disposition"], "exact_current")
        self.assertFalse(sample["lateral_authority_eligible"])
        self.assertEqual(sample["binding_ros_now_ns"], 1_100_000_000)

    def test_proposal_audit_exact_join_persists_monotonic_classification(self) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        capture.e2e_terminals[key] = self.e2e_terminal(
            "exact_current", callback_monotonic_ns=60
        )
        capture.e2e_terminals[key]["binding_snapshot_evidence_v1"].update(
            {
                "current": dict(
                    capture.e2e_terminals[key][
                        "binding_snapshot_evidence_v1"
                    ]["proposal"]
                ),
                "previous": None,
            }
        )
        capture.binding_dispositions["exact_current"] = 1
        capture._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        capture._receive_planner_input_audit(
            String(data=json.dumps(self.planner_input_audit_payload(77)))
        )
        summary = capture.finalize_e2e_measurement()
        audit = summary["terminal_provenance"][0]["proposal_publish_audit_v1"]
        self.assertEqual(
            audit["classification"],
            "binding_callback_after_publish_return",
        )
        self.assertEqual(audit["publish_entry_to_binding_callback_ns"], 20)
        self.assertEqual(audit["publish_return_to_binding_callback_ns"], 10)
        self.assertEqual(
            audit["planner_input_audit_v1"]["classification"],
            "input_then_planning_then_capture",
        )
        self.assertEqual(
            audit["planner_input_audit_v1"]["input_to_planning_timer_ns"], 10
        )
        uptake = summary["proposal_uptake_contract_v1"]
        self.assertEqual(uptake["expected_proposal_count"], 1)
        self.assertEqual(uptake["accepted_first_uptake_count"], 1)
        self.assertEqual(uptake["failed_or_missing_count"], 0)
        self.assertTrue(
            uptake["all_proposals_taken_up_once_before_deadline"]
        )
        self.assertFalse(uptake["pp_hold_cycles_evaluated"])
        self.assertEqual(
            uptake["sustained_10_cycle_gate"], "not_evaluated"
        )

        during = self.e2e_ledger_capture()
        during.e2e_terminals[key] = self.e2e_terminal(
            "exact_current", callback_monotonic_ns=45
        )
        during.e2e_terminals[key]["binding_snapshot_evidence_v1"].update(
            {
                "current": dict(
                    during.e2e_terminals[key][
                        "binding_snapshot_evidence_v1"
                    ]["proposal"]
                ),
                "previous": None,
            }
        )
        during.binding_dispositions["exact_current"] = 1
        during._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        during._receive_planner_input_audit(
            String(data=json.dumps(self.planner_input_audit_payload(77)))
        )
        during_summary = during.finalize_e2e_measurement()
        during_audit = during_summary["terminal_provenance"][0][
            "proposal_publish_audit_v1"
        ]
        self.assertEqual(
            during_audit["classification"],
            "binding_callback_during_publish_call",
        )
        self.assertEqual(
            during_audit["publish_entry_to_binding_callback_ns"], 5
        )
        self.assertEqual(
            during_audit["publish_return_to_binding_callback_ns"], -5
        )
        during_uptake = during_summary["proposal_uptake_contract_v1"]
        self.assertFalse(
            during_uptake["all_proposals_taken_up_once_before_deadline"]
        )
        self.assertEqual(
            during_uptake["records"][0]["failure_reasons"],
            ["binding_before_publish_return"],
        )

    def test_proposal_uptake_rejects_deferred_predecessor(self) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        terminal = self.e2e_terminal(
            "deferred_predecessor", callback_monotonic_ns=60
        )
        terminal["binding_snapshot_evidence_v1"]["current"] = dict(
            terminal["binding_snapshot_evidence_v1"]["proposal"]
        )
        terminal["binding_snapshot_evidence_v1"]["current"][
            "source_stamp_sec"
        ] = 2
        terminal["binding_snapshot_evidence_v1"]["previous"] = dict(
            terminal["binding_snapshot_evidence_v1"]["proposal"]
        )
        capture.e2e_terminals[key] = terminal
        capture.binding_dispositions["deferred_predecessor"] = 1
        capture._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        capture._receive_planner_input_audit(
            String(data=json.dumps(self.planner_input_audit_payload(77)))
        )

        summary = capture.finalize_e2e_measurement()

        uptake = summary["proposal_uptake_contract_v1"]
        self.assertEqual(uptake["accepted_first_uptake_count"], 0)
        self.assertEqual(uptake["failed_or_missing_count"], 1)
        self.assertEqual(
            uptake["records"][0]["failure_reasons"],
            ["not_exact_current", "source_identity_not_current"],
        )

    def test_proposal_audit_rejects_missing_duplicate_and_order_inversion(self) -> None:
        capture = self.e2e_ledger_capture()
        capture._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        with self.assertRaisesRegex(AssertionError, "exact-join mismatch"):
            capture.finalize_e2e_measurement()

        duplicate = self.e2e_ledger_capture()
        duplicate._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        duplicate._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        with self.assertRaisesRegex(AssertionError, "malformed or order-inverted"):
            duplicate.finalize_e2e_measurement()

        inverted = self.e2e_ledger_capture()
        payload = self.proposal_audit_payload(77)
        payload["worker_build_begin_monotonic_ns"] = 19
        inverted._receive_proposal_publish_audit(String(data=json.dumps(payload)))
        with self.assertRaisesRegex(AssertionError, "malformed or order-inverted"):
            inverted.finalize_e2e_measurement()

        binding_inverted = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        binding_inverted.e2e_terminals[key] = self.e2e_terminal(
            "exact_current", callback_monotonic_ns=39
        )
        binding_inverted.binding_dispositions["exact_current"] = 1
        binding_inverted._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        binding_inverted._receive_planner_input_audit(
            String(data=json.dumps(self.planner_input_audit_payload(77)))
        )
        with self.assertRaisesRegex(AssertionError, "binding order inversion"):
            binding_inverted.finalize_e2e_measurement()

        overflow = self.e2e_ledger_capture()
        overflow.e2e_proposal_audits = {
            self.e2e_key(index): {} for index in range(E2E_PROPOSAL_AUDIT_LEDGER_LIMIT)
        }
        overflow._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(2_000)))
        )
        with self.assertRaisesRegex(AssertionError, "capacity exceeded"):
            overflow.finalize_e2e_measurement()

    def test_planner_input_audit_rejects_order_source_and_join_failures(self) -> None:
        audit_only = self.e2e_ledger_capture()
        audit_only._receive_planner_input_audit(
            String(data=json.dumps(self.planner_input_audit_payload(77)))
        )
        with self.assertRaisesRegex(AssertionError, "planner input audit proposal publish exact-join"):
            audit_only.finalize_e2e_measurement()

        capture = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        capture.e2e_terminals[key] = self.e2e_terminal(
            "exact_current", callback_monotonic_ns=60
        )
        capture.binding_dispositions["exact_current"] = 1
        capture._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        payload = self.planner_input_audit_payload(77)
        payload["planning_timer_entry_monotonic_ns"] = 9
        capture._receive_planner_input_audit(String(data=json.dumps(payload)))
        with self.assertRaisesRegex(AssertionError, "planner input audit malformed"):
            capture.finalize_e2e_measurement()

        source_mismatch = self.e2e_ledger_capture()
        source_mismatch.e2e_terminals[key] = self.e2e_terminal(
            "exact_current", callback_monotonic_ns=60
        )
        source_mismatch.binding_dispositions["exact_current"] = 1
        source_mismatch._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77)))
        )
        payload = self.planner_input_audit_payload(77)
        payload["proposal_source_generation"] = 12
        source_mismatch._receive_planner_input_audit(
            String(data=json.dumps(payload))
        )
        with self.assertRaisesRegex(AssertionError, "authoritative source identity"):
            source_mismatch.finalize_e2e_measurement()

    def test_out_of_cohort_streams_remain_separate(self) -> None:
        capture = self.e2e_ledger_capture()
        capture.node = Mock()
        capture.node.get_clock.return_value.now.return_value.nanoseconds = (
            1_200_000_000
        )
        proposal = self.authorized_proposal_message(100_000_000)
        proposal.plan_sample_key.race_arm_epoch = 3
        binding = self.binding_audit_payload(epoch=3)

        capture._receive_authorized_proposal(proposal)
        capture._receive_binding(String(data=json.dumps(binding)))
        capture._receive_audited_binding(
            String(data=json.dumps(binding))
        )
        capture._receive_proposal_publish_audit(
            String(data=json.dumps(self.proposal_audit_payload(77, epoch=3)))
        )
        capture._receive_planner_input_audit(
            String(data=json.dumps(self.planner_input_audit_payload(77, epoch=3)))
        )
        summary = capture.finalize_e2e_measurement()

        self.assertEqual(summary["out_of_cohort_proposals"], 1)
        self.assertEqual(summary["out_of_cohort_bindings"], 1)
        self.assertEqual(summary["out_of_cohort_binding_audits"], 1)
        self.assertEqual(summary["out_of_cohort_proposal_audits"], 1)
        self.assertEqual(summary["out_of_cohort_planner_input_audits"], 1)
        samples = summary["out_of_cohort_samples_v1"]
        self.assertEqual(samples["schema_version"], 1)
        self.assertEqual(
            samples["per_stream_limit"],
            E2E_OUT_OF_COHORT_SAMPLE_LIMIT,
        )
        self.assertFalse(samples["overflow"])
        self.assertEqual(
            set(samples["samples"]),
            {
                "proposal",
                "binding",
                "binding_audit",
                "proposal_audit",
                "planner_input_audit",
            },
        )
        for stream in (
            "proposal",
            "binding",
            "binding_audit",
            "proposal_audit",
            "planner_input_audit",
        ):
            self.assertEqual(len(samples["samples"][stream]), 1)
            sample = samples["samples"][stream][0]
            self.assertEqual(sample["stream"], stream)
            self.assertEqual(sample["event_ordinal"], 1)
            self.assertEqual(sample["expected_race_arm_epoch"], 2)
            self.assertEqual(sample["observed_race_arm_epoch"], 3)
            self.assertEqual(sample["full_proposal_key"][0], 3)
        self.assertEqual(capture.e2e_proposals, {})
        self.assertEqual(capture.e2e_observed_binding_keys, set())
        self.assertEqual(capture.e2e_terminals, {})

    def test_out_of_cohort_binding_audit_invalid_schema_fails_closed(
        self,
    ) -> None:
        invalid_overrides = (
            {"test_audit_schema_version": 2},
            {"proposal_attempt_id": True},
            {"previous_source_stamp_nanosec": 1_000_000_000},
            {"disposition": "unknown"},
            {"lateral_authority_eligible": True},
        )
        for override in invalid_overrides:
            with self.subTest(override=override):
                capture = self.e2e_ledger_capture()
                capture.node = Mock()
                capture.node.get_clock.return_value.now.return_value.nanoseconds = (
                    1_200_000_000
                )
                payload = self.binding_audit_payload(epoch=3)
                payload.update(override)
                capture._receive_audited_binding(
                    String(data=json.dumps(payload))
                )
                self.assertTrue(capture.binding_invalid)
                if override != {"lateral_authority_eligible": True}:
                    self.assertEqual(
                        capture.e2e_out_of_cohort_samples[
                            "binding_audit"
                        ],
                        [],
                    )

    def test_out_of_cohort_sample_overflow_fails_closed(self) -> None:
        capture = self.e2e_ledger_capture()
        capture.node = Mock()
        capture.node.get_clock.return_value.now.return_value.nanoseconds = (
            1_200_000_000
        )
        for attempt_id in range(
            10,
            10 + E2E_OUT_OF_COHORT_SAMPLE_LIMIT + 1,
        ):
            payload = self.binding_audit_payload(
                epoch=3,
                attempt_id=attempt_id,
            )
            capture._receive_audited_binding(
                String(data=json.dumps(payload))
            )

        self.assertEqual(
            len(capture.e2e_out_of_cohort_samples["binding_audit"]),
            E2E_OUT_OF_COHORT_SAMPLE_LIMIT,
        )
        self.assertTrue(capture.e2e_out_of_cohort_sample_overflow)
        with self.assertRaisesRegex(
            AssertionError,
            "out-of-cohort sample capacity exceeded",
        ):
            capture.finalize_e2e_measurement()

    def test_e2e_exact_current_terminal_cadence_is_measured(self) -> None:
        capture = self.e2e_ledger_capture()
        first_key = self.e2e_key(70)
        second_key = self.e2e_key(71)
        for key, callback_monotonic_ns in (
            (first_key, 100),
            (second_key, 145),
        ):
            capture.e2e_proposals[key] = self.e2e_proposal()
            capture.e2e_observed_binding_keys.add(key)
            capture.e2e_terminals[key] = self.e2e_terminal(
                "exact_current",
                callback_monotonic_ns=callback_monotonic_ns,
            )
        capture.binding_dispositions["exact_current"] = 2

        summary = capture.finalize_e2e_measurement()

        self.assertEqual(summary["exact_current_terminal_count"], 2)
        self.assertEqual(
            summary["exact_current_terminal_max_monotonic_gap_ns"], 45
        )

    def test_e2e_safety_lifetime_decomposes_by_full_identity(self) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(73)
        capture.e2e_proposals[key] = {
            "plan_stamp_ns": 100,
            "safety_evaluation_stamp_ns": 100,
            "safety_valid_until_ns": 200,
            "received_sim_ns": 130,
        }
        capture.e2e_observed_binding_keys.add(key)
        terminal = self.e2e_terminal("exact_current")
        terminal.update(
            {
                "binding_ros_now_ns": 170,
                "safety_valid_until_ns": 200,
                "safety_margin_at_bind_ns": 30,
            }
        )
        capture.e2e_terminals[key] = terminal
        capture.binding_dispositions["exact_current"] = 1

        summary = capture.finalize_e2e_measurement()

        self.assertEqual(summary["safety_lifetime_timeline_count"], 1)
        self.assertEqual(summary["safety_lifetime_timeline_missing"], 0)
        self.assertFalse(summary["safety_lifetime_timeline_truncated"])
        timeline = summary["safety_lifetime_timeline"][0]
        self.assertEqual(timeline["full_terminal_key"], key)
        self.assertEqual(timeline["generation_budget_ns"], 100)
        self.assertEqual(timeline["observer_delivery_elapsed_ns"], 30)
        self.assertEqual(timeline["observer_reception_margin_ns"], 70)
        self.assertEqual(timeline["producer_binding_elapsed_ns"], 70)
        self.assertEqual(timeline["observer_binding_time_delta_ns"], 40)
        self.assertEqual(timeline["binding_margin_ns"], 30)
        self.assertEqual(
            timeline["generation_budget_ns"],
            timeline["producer_binding_elapsed_ns"]
            + timeline["binding_margin_ns"],
        )

    def test_e2e_proposal_callback_retains_safety_deadline(self) -> None:
        capture = self.e2e_ledger_capture()
        capture.authorized_proposal_count = 0
        capture.binding_invalid = False
        capture.node = Mock()
        capture.node.get_clock.return_value.now.return_value.nanoseconds = (
            130_000_000
        )
        message = AuthorizedCartesianTrajectory()
        message.schema_version = (
            AuthorizedCartesianTrajectory.SCHEMA_V1_SHADOW
        )
        message.authority_eligible = False
        message.plan_sample_key.race_arm_epoch = 2
        message.plan_sample_key.planner_instance_id = 7001
        message.plan_sample_key.attempt_id = 77
        message.plan_sample_key.connector_transaction_id = 77
        message.plan_sample_key.plan_generation = 11
        message.candidate_revision = 77
        message.authority_token = 9001
        message.safety_snapshot_id = 8001
        message.source_controller_instance_id = 6001
        message.source_controller_sequence = 77
        message.base_lease_id = 5001
        message.plan_stamp.nanosec = 100_000_000
        message.safety_evaluation_stamp.nanosec = 100_000_000
        message.base_source_stamp.nanosec = 90_000_000
        message.safety_valid_until.nanosec = 150_000_000

        steady_ns = 1_785_000_000_000_000_000
        with patch(
            "c002ay0_pp_runtime_measurement.time.monotonic_ns",
            return_value=steady_ns,
        ):
            capture._receive_authorized_proposal(message)

        key = self.e2e_key(77)
        self.assertEqual(
            capture.e2e_proposals[key]["received_sim_ns"],
            130_000_000,
        )
        self.assertEqual(
            capture.e2e_proposals[key]["received_steady_ns"],
            steady_ns,
        )
        self.assertEqual(
            capture.e2e_proposals[key]["safety_valid_until_ns"],
            150_000_000,
        )

    def test_e2e_input_publish_tick_records_stamp_boundary(self) -> None:
        capture = self.e2e_ledger_capture()
        capture._receive_input_publish_tick(
            String(
                data=json.dumps(
                    {
                        "schema_version": 1,
                        "tick_index": 11,
                        "clock_stamp_ns": 110_000_000,
                        "trajectory_stamp_ns": 110_000_000,
                        "plan_stamp_ns": 110_000_000,
                        "clock_publish_steady_ns": 8_000,
                        "plan_publish_steady_ns": 9_000,
                        "covered_clock_first_stamp_ns": 20_000_000,
                        "covered_clock_last_stamp_ns": 110_000_000,
                        "covered_clock_tick_period_ns": 10_000_000,
                        "covered_clock_tick_count": 10,
                    }
                )
            )
        )

        self.assertEqual(
            capture.e2e_input_publish_ticks[110_000_000],
            {
                "tick_index": 11,
                "clock_stamp_ns": 110_000_000,
                "trajectory_stamp_ns": 110_000_000,
                "plan_stamp_ns": 110_000_000,
                "clock_publish_steady_ns": 8_000,
                "plan_publish_steady_ns": 9_000,
                "covered_clock_first_stamp_ns": 20_000_000,
                "covered_clock_last_stamp_ns": 110_000_000,
                "covered_clock_tick_period_ns": 10_000_000,
                "covered_clock_tick_count": 10,
            },
        )

    def test_e2e_publisher_stamp_boundary_mismatch_is_explicit(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        capture.e2e_proposals[key] = {
            "plan_stamp_ns": 110_000_000,
            "safety_evaluation_stamp_ns": 110_000_000,
            "safety_valid_until_ns": 160_000_000,
            "received_sim_ns": 100_000_000,
        }
        capture.e2e_terminals[key] = {
            **self.e2e_terminal("exact_current"),
            "binding_ros_now_ns": 120_000_000,
            "safety_valid_until_ns": 160_000_000,
        }
        capture.binding_dispositions["exact_current"] = 1
        capture.e2e_input_publish_ticks[110_000_000] = {
            "tick_index": 11,
            "clock_stamp_ns": 100_000_000,
            "trajectory_stamp_ns": 110_000_000,
            "plan_stamp_ns": 110_000_000,
            "clock_publish_steady_ns": 8_000,
            "plan_publish_steady_ns": 9_000,
            "covered_clock_first_stamp_ns": 20_000_000,
            "covered_clock_last_stamp_ns": 110_000_000,
            "covered_clock_tick_period_ns": 10_000_000,
            "covered_clock_tick_count": 10,
        }

        summary = capture.finalize_e2e_measurement()
        diagnostic = summary["safety_lifetime_timeline"][0][
            "observer_receipt_diagnostic"
        ]
        self.assertTrue(diagnostic["observer_before_plan"])
        self.assertEqual(
            diagnostic["classification"],
            "publisher_stamp_boundary_mismatch",
        )

    def test_e2e_delayed_aligned_audit_classifies_observer_clock(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        capture.e2e_proposals[key] = {
            "plan_stamp_ns": 80_000_000,
            "safety_evaluation_stamp_ns": 80_000_000,
            "safety_valid_until_ns": 130_000_000,
            "received_sim_ns": 70_000_000,
            "received_steady_ns": 10_000,
        }
        capture.e2e_terminals[key] = {
            **self.e2e_terminal("exact_current"),
            "binding_ros_now_ns": 90_000_000,
            "safety_valid_until_ns": 130_000_000,
        }
        capture.binding_dispositions["exact_current"] = 1
        capture._receive_input_publish_tick(
            String(
                data=json.dumps(
                    {
                        "schema_version": 1,
                        "tick_index": 11,
                        "clock_stamp_ns": 110_000_000,
                        "trajectory_stamp_ns": 110_000_000,
                        "plan_stamp_ns": 110_000_000,
                        "clock_publish_steady_ns": 8_000,
                        "plan_publish_steady_ns": 9_000,
                        "covered_clock_first_stamp_ns": 20_000_000,
                        "covered_clock_last_stamp_ns": 110_000_000,
                        "covered_clock_tick_period_ns": 10_000_000,
                        "covered_clock_tick_count": 10,
                    }
                )
            )
        )

        summary = capture.finalize_e2e_measurement()
        diagnostic = summary["safety_lifetime_timeline"][0][
            "observer_receipt_diagnostic"
        ]
        self.assertEqual(diagnostic["schema_version"], 1)
        self.assertEqual(diagnostic["observer_minus_plan_ns"], -10_000_000)
        self.assertTrue(diagnostic["observer_before_plan"])
        self.assertEqual(
            diagnostic["classification"],
            "publisher_aligned_observer_clock_before_plan",
        )

    def test_e2e_current_aligned_audit_is_diagnostic_only(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        capture.e2e_proposals[key] = {
            "plan_stamp_ns": 110_000_000,
            "safety_evaluation_stamp_ns": 110_000_000,
            "safety_valid_until_ns": 160_000_000,
            "received_sim_ns": 110_000_000,
            "received_steady_ns": 10_000,
        }
        capture.e2e_terminals[key] = {
            **self.e2e_terminal("exact_current"),
            "binding_ros_now_ns": 120_000_000,
            "safety_valid_until_ns": 160_000_000,
        }
        capture.binding_dispositions["exact_current"] = 1
        capture.e2e_input_publish_ticks[110_000_000] = {
            "tick_index": 11,
            "clock_stamp_ns": 110_000_000,
            "trajectory_stamp_ns": 110_000_000,
            "plan_stamp_ns": 110_000_000,
            "clock_publish_steady_ns": 8_000,
            "plan_publish_steady_ns": 9_000,
            "covered_clock_first_stamp_ns": 20_000_000,
            "covered_clock_last_stamp_ns": 110_000_000,
            "covered_clock_tick_period_ns": 10_000_000,
            "covered_clock_tick_count": 10,
        }

        summary = capture.finalize_e2e_measurement()
        diagnostic = summary["safety_lifetime_timeline"][0][
            "observer_receipt_diagnostic"
        ]
        self.assertFalse(diagnostic["observer_before_plan"])
        self.assertEqual(diagnostic["observer_minus_plan_ns"], 0)
        self.assertEqual(
            diagnostic["classification"],
            "publisher_aligned_observer_clock_current",
        )

    def test_e2e_missing_audit_is_diagnostic_only(self) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        capture.e2e_proposals[key] = {
            "plan_stamp_ns": 100_000_000,
            "safety_evaluation_stamp_ns": 100_000_000,
            "safety_valid_until_ns": 150_000_000,
            "received_sim_ns": 110_000_000,
            "received_steady_ns": 10_000,
        }
        capture.e2e_terminals[key] = {
            **self.e2e_terminal("exact_current"),
            "binding_ros_now_ns": 120_000_000,
            "safety_valid_until_ns": 150_000_000,
        }
        capture.binding_dispositions["exact_current"] = 1

        summary = capture.finalize_e2e_measurement()
        diagnostic = summary["safety_lifetime_timeline"][0][
            "observer_receipt_diagnostic"
        ]
        self.assertFalse(diagnostic["observer_before_plan"])
        self.assertEqual(
            diagnostic["observer_minus_plan_ns"],
            10_000_000,
        )
        self.assertEqual(
            diagnostic["classification"],
            "publisher_tick_audit_unavailable",
        )

    def test_e2e_overlapping_tick_audits_are_ambiguous(self) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(77)
        capture.e2e_proposals[key] = {
            "plan_stamp_ns": 80_000_000,
            "safety_evaluation_stamp_ns": 80_000_000,
            "safety_valid_until_ns": 130_000_000,
            "received_sim_ns": 70_000_000,
            "received_steady_ns": 10_000,
        }
        capture.e2e_terminals[key] = {
            **self.e2e_terminal("exact_current"),
            "binding_ros_now_ns": 90_000_000,
            "safety_valid_until_ns": 130_000_000,
        }
        capture.binding_dispositions["exact_current"] = 1
        for first_ns, last_ns in (
            (20_000_000, 110_000_000),
            (30_000_000, 120_000_000),
        ):
            capture._receive_input_publish_tick(
                String(
                    data=json.dumps(
                        {
                            "schema_version": 1,
                            "tick_index": last_ns // 10_000_000,
                            "clock_stamp_ns": last_ns,
                            "trajectory_stamp_ns": last_ns,
                            "plan_stamp_ns": last_ns,
                            "clock_publish_steady_ns": 8_000,
                            "plan_publish_steady_ns": 9_000,
                            "covered_clock_first_stamp_ns": first_ns,
                            "covered_clock_last_stamp_ns": last_ns,
                            "covered_clock_tick_period_ns": 10_000_000,
                            "covered_clock_tick_count": 10,
                        }
                    )
                )
            )

        summary = capture.finalize_e2e_measurement()
        diagnostic = summary["safety_lifetime_timeline"][0][
            "observer_receipt_diagnostic"
        ]
        self.assertEqual(
            diagnostic["classification"],
            "publisher_tick_audit_ambiguous",
        )

    def test_e2e_input_publish_tick_invalid_duplicate_and_overflow_fail(
        self,
    ) -> None:
        def tick_message(stamp_ns: int) -> String:
            return String(
                data=json.dumps(
                    {
                        "schema_version": 1,
                        "tick_index": stamp_ns,
                        "clock_stamp_ns": stamp_ns,
                        "trajectory_stamp_ns": stamp_ns,
                        "plan_stamp_ns": stamp_ns,
                        "clock_publish_steady_ns": stamp_ns,
                        "plan_publish_steady_ns": stamp_ns,
                        "covered_clock_first_stamp_ns": (
                            stamp_ns - 90_000_000
                        ),
                        "covered_clock_last_stamp_ns": stamp_ns,
                        "covered_clock_tick_period_ns": 10_000_000,
                        "covered_clock_tick_count": 10,
                    }
                )
            )

        malformed = self.e2e_ledger_capture()
        malformed._receive_input_publish_tick(String(data="{"))
        with self.assertRaisesRegex(
            AssertionError,
            "input publish tick ledger invalid",
        ):
            malformed.finalize_e2e_measurement()

        invalid_coverages = (
            {
                "covered_clock_tick_period_ns": 0,
            },
            {
                "covered_clock_tick_period_ns": -10_000_000,
            },
            {
                "covered_clock_tick_count": 0,
            },
            {
                "covered_clock_tick_count": -1,
            },
            {
                "covered_clock_first_stamp_ns": 120_000_000,
                "covered_clock_last_stamp_ns": 110_000_000,
            },
            {
                "covered_clock_first_stamp_ns": 110_000_000,
                "covered_clock_last_stamp_ns": 110_000_000,
            },
        )
        for override in invalid_coverages:
            with self.subTest(coverage_override=override):
                value = json.loads(tick_message(110_000_000).data)
                value.update(override)
                invalid = self.e2e_ledger_capture()
                invalid._receive_input_publish_tick(
                    String(data=json.dumps(value))
                )
                with self.assertRaisesRegex(
                    AssertionError,
                    "input publish tick ledger invalid",
                ):
                    invalid.finalize_e2e_measurement()

        duplicate = self.e2e_ledger_capture()
        duplicate._receive_input_publish_tick(tick_message(1))
        duplicate._receive_input_publish_tick(tick_message(1))
        with self.assertRaisesRegex(
            AssertionError,
            "input publish tick ledger invalid",
        ):
            duplicate.finalize_e2e_measurement()

        overflow = self.e2e_ledger_capture()
        for stamp_ns in range(
            1,
            E2E_INPUT_PUBLISH_TICK_LEDGER_LIMIT + 2,
        ):
            overflow._receive_input_publish_tick(
                tick_message(stamp_ns)
            )
        with self.assertRaisesRegex(
            AssertionError,
            "input publish tick ledger capacity exceeded",
        ):
            overflow.finalize_e2e_measurement()

    def test_e2e_safety_lifetime_invalid_provenance_fails_closed(
        self,
    ) -> None:
        invalid_values = (
            (
                "evaluation mismatch",
                ("plan_equals_safety_evaluation",),
                101,
                200,
                99,
                170,
                200,
            ),
            (
                "nonpositive budget",
                (
                    "proposal_deadline_after_plan",
                    "binding_before_deadline",
                ),
                100,
                100,
                100,
                100,
                100,
            ),
            (
                "binding before plan",
                ("binding_not_before_plan",),
                100,
                200,
                99,
                99,
                200,
            ),
            (
                "binding at deadline",
                ("binding_before_deadline",),
                100,
                200,
                130,
                200,
                200,
            ),
            (
                "terminal expiry mismatch",
                ("proposal_deadline_equals_terminal_deadline",),
                100,
                200,
                130,
                170,
                201,
            ),
        )
        for (
            reason,
            failed_predicates,
            evaluation_ns,
            proposal_expiry_ns,
            received_ns,
            binding_ns,
            terminal_expiry_ns,
        ) in invalid_values:
            with self.subTest(reason=reason):
                capture = self.e2e_ledger_capture()
                key = self.e2e_key(74)
                capture.e2e_proposals[key] = {
                    "plan_stamp_ns": 100,
                    "safety_evaluation_stamp_ns": evaluation_ns,
                    "safety_valid_until_ns": proposal_expiry_ns,
                    "received_sim_ns": received_ns,
                    "received_steady_ns": 55,
                }
                terminal = self.e2e_terminal(
                    "exact_current", callback_monotonic_ns=60
                )
                terminal.update(
                    {
                        "binding_ros_now_ns": binding_ns,
                        "safety_valid_until_ns": terminal_expiry_ns,
                    }
                )
                capture.e2e_terminals[key] = terminal
                capture.binding_dispositions["exact_current"] = 1
                capture._receive_proposal_publish_audit(
                    String(data=json.dumps(self.proposal_audit_payload(74)))
                )
                capture._receive_planner_input_audit(
                    String(data=json.dumps(self.planner_input_audit_payload(74)))
                )

                with self.assertRaises(AssertionError) as raised:
                    capture.finalize_e2e_measurement()
                message = str(raised.exception)
                if reason == "binding at deadline":
                    self.assertIn(
                        "safety lifetime monotonic direct comparison",
                        message,
                    )
                    evidence = json.loads(
                        message.split(" evidence=", maxsplit=1)[1]
                    )
                    self.assertEqual(evidence["clock"], "CLOCK_MONOTONIC")
                    self.assertEqual(
                        evidence["full_terminal_key"], list(key)
                    )
                    self.assertEqual(
                        evidence["planning_timer_entry_monotonic_ns"], 20
                    )
                    self.assertEqual(
                        evidence["binding_callback_monotonic_ns"], 60
                    )
                    self.assertEqual(evidence["planning_to_binding_ns"], 40)
                    self.assertEqual(evidence["diagnostic_lifetime_ns"], 100)
                    self.assertEqual(
                        evidence["diagnostic_monotonic_margin_ns"], 60
                    )
                    self.assertEqual(
                        evidence["ledger_close_mismatch"],
                        {
                            "planner_input_only": [],
                            "proposal_audit_only": [],
                        },
                    )
                    continue
                self.assertIn(
                    "safety lifetime timeline provenance invalid",
                    message,
                )
                self.assertIn(
                    f"failed_predicates={json.dumps(list(failed_predicates))}",
                    message,
                )
                predicates_text, diagnostic_text = message.split(
                    " predicates=", maxsplit=1
                )[1].split(" timestamps_ns=", maxsplit=1)
                timestamps_text, observer_diagnostic_text = (
                    diagnostic_text.split(
                        " observer_receipt_diagnostic=",
                        maxsplit=1,
                    )
                )
                predicates = json.loads(predicates_text)
                timestamps_ns = json.loads(timestamps_text)
                observer_text, monotonic_text = observer_diagnostic_text.split(
                    " monotonic_causal_evidence=", maxsplit=1
                )
                observer_diagnostic = json.loads(observer_text)
                monotonic_evidence = json.loads(monotonic_text)
                for failed_predicate in failed_predicates:
                    self.assertFalse(predicates[failed_predicate])
                self.assertEqual(
                    observer_diagnostic["classification"],
                    "publisher_tick_audit_unavailable",
                )
                self.assertEqual(
                    sum(not passed for passed in predicates.values()),
                    len(failed_predicates),
                )
                self.assertEqual(
                    timestamps_ns,
                    {
                        "binding_ros_now_ns": binding_ns,
                        "observer_received_sim_ns": received_ns,
                        "plan_stamp_ns": 100,
                        "proposal_safety_valid_until_ns": (
                            proposal_expiry_ns
                        ),
                        "safety_evaluation_stamp_ns": evaluation_ns,
                        "terminal_safety_valid_until_ns": (
                            terminal_expiry_ns
                        ),
                    },
                )
                self.assertEqual(monotonic_evidence["schema_version"], 1)
                self.assertEqual(
                    monotonic_evidence["clock"], "CLOCK_MONOTONIC"
                )
                self.assertEqual(
                    monotonic_evidence["full_terminal_key"], list(key)
                )
                self.assertEqual(
                    monotonic_evidence["proposal_callback_steady_ns"], 55
                )
                self.assertEqual(
                    monotonic_evidence["binding_callback_monotonic_ns"], 60
                )
                causal = monotonic_evidence["proposal_publish_audit_v1"]
                self.assertEqual(causal["capture_monotonic_ns"], 10)
                self.assertEqual(causal["publish_call_entry_monotonic_ns"], 40)
                self.assertEqual(causal["publish_call_return_monotonic_ns"], 50)
                self.assertEqual(
                    causal["planner_input_audit_v1"][
                        "base_snapshot_callback_entry_monotonic_ns"
                    ],
                    10,
                )
                self.assertEqual(
                    causal["planner_input_audit_v1"]["capture_monotonic_ns"],
                    40,
                )

    def test_e2e_safety_lifetime_allows_binding_before_observer(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(76)
        capture.e2e_proposals[key] = self.e2e_proposal()
        terminal = self.e2e_terminal("exact_current")
        terminal.update(
            {
                "binding_ros_now_ns": 120_000_000,
                "safety_valid_until_ns": 200_000_000,
            }
        )
        capture.e2e_terminals[key] = terminal
        capture.binding_dispositions["exact_current"] = 1

        summary = capture.finalize_e2e_measurement()

        timeline = summary["safety_lifetime_timeline"][0]
        self.assertEqual(
            timeline["observer_binding_time_delta_ns"], -10_000_000
        )
        self.assertEqual(
            timeline["producer_binding_elapsed_ns"], 20_000_000
        )

    def test_e2e_safety_lifetime_missing_provenance_fails_closed(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(75)
        capture.e2e_proposals[key] = self.e2e_proposal()
        del capture.e2e_proposals[key]["received_sim_ns"]
        capture.e2e_terminals[key] = self.e2e_terminal("exact_current")
        capture.binding_dispositions["exact_current"] = 1

        with self.assertRaisesRegex(
            AssertionError, "safety lifetime timeline provenance missing"
        ):
            capture.finalize_e2e_measurement()

    def test_e2e_safety_lifetime_capacity_fails_closed(self) -> None:
        capture = self.e2e_ledger_capture()
        for attempt_id in range(1, E2E_SAFETY_TIMELINE_LIMIT + 2):
            key = self.e2e_key(attempt_id)
            capture.e2e_proposals[key] = self.e2e_proposal()
            capture.e2e_terminals[key] = self.e2e_terminal("exact_current")
        capture.binding_dispositions["exact_current"] = (
            E2E_SAFETY_TIMELINE_LIMIT + 1
        )

        with self.assertRaisesRegex(
            AssertionError, "safety lifetime timeline capacity exceeded"
        ):
            capture.finalize_e2e_measurement()

    def test_e2e_deferred_authority_claim_is_hard_invalid(self) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(72)
        capture.e2e_proposals[key] = self.e2e_proposal()
        capture.e2e_observed_binding_keys.add(key)
        capture.e2e_terminals[key] = self.e2e_terminal(
            "deferred_predecessor",
            lateral_authority_eligible=True,
        )
        capture.binding_dispositions["deferred_predecessor"] = 1

        with self.assertRaisesRegex(
            AssertionError, "non-exact terminal claims lateral authority"
        ):
            capture.finalize_e2e_measurement()

    def test_e2e_terminal_observer_missing_has_no_deferred_disposition(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(2)
        capture.e2e_proposals[key] = self.e2e_proposal()
        capture.e2e_terminals[key] = self.e2e_terminal("exact_current")
        capture.binding_dispositions["exact_current"] = 1

        summary = capture.finalize_e2e_measurement()

        self.assertEqual(
            capture.binding_dispositions,
            {"exact_current": 1},
        )
        self.assertEqual(summary["fixture_proposal_seen"], 1)
        self.assertEqual(summary["terminal_bindings"], 1)
        self.assertEqual(summary["intersection_identities"], 1)
        self.assertEqual(summary["fixture_only_identities"], 0)
        self.assertEqual(summary["binding_only_identities"], 0)
        self.assertEqual(summary["binding_observer_missing"], 1)
        self.assertEqual(summary["binding_observer_without_audit"], 0)
        self.assertEqual(
            summary["binding_observer_missing_keys"],
            [key],
        )
        self.assertEqual(summary["out_of_cohort_proposals"], 0)
        self.assertEqual(summary["out_of_cohort_bindings"], 0)
        self.assertEqual(summary["out_of_cohort_binding_audits"], 0)
        self.assertEqual(
            summary["terminal_provenance"][0]["observed_exact_key"],
            None,
        )
        self.assertTrue(summary["terminal_provenance"][0]["observer_missing"])

    def test_e2e_mixed_failures_remain_separately_counted(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        deferred_key = self.e2e_key(3)
        missing_key = self.e2e_key(4)
        capture.e2e_proposals[deferred_key] = self.e2e_proposal()
        capture.e2e_proposals[missing_key] = self.e2e_proposal()
        capture.e2e_observed_binding_keys.add(deferred_key)
        capture.e2e_terminals[deferred_key] = self.e2e_terminal(
            "deferred_predecessor"
        )
        capture.e2e_terminals[missing_key] = self.e2e_terminal(
            "exact_current"
        )
        capture.binding_dispositions.update(
            {
                "deferred_predecessor": 1,
                "exact_current": 1,
            }
        )

        summary = capture.finalize_e2e_measurement()

        self.assertEqual(
            capture.binding_dispositions,
            {
                "deferred_predecessor": 1,
                "exact_current": 1,
            },
        )
        self.assertEqual(summary["fixture_proposal_seen"], 2)
        self.assertEqual(summary["terminal_bindings"], 2)
        self.assertEqual(summary["intersection_identities"], 2)
        self.assertEqual(summary["fixture_only_identities"], 0)
        self.assertEqual(summary["binding_only_identities"], 0)
        self.assertEqual(summary["binding_observer_missing"], 1)
        self.assertEqual(
            summary["binding_observer_missing_keys"],
            [missing_key],
        )
        self.assertEqual(summary["binding_observer_without_audit"], 0)
        self.assertEqual(summary["out_of_cohort_proposals"], 0)
        self.assertEqual(summary["out_of_cohort_bindings"], 0)
        self.assertEqual(summary["out_of_cohort_binding_audits"], 0)

    def test_e2e_full_identity_observer_mismatch_is_not_reanchored(
        self,
    ) -> None:
        capture = self.e2e_ledger_capture()
        terminal_key = self.e2e_key(44)
        observer_key = self.e2e_key(44, controller_sequence=45)
        capture.e2e_proposals[terminal_key] = self.e2e_proposal()
        capture.e2e_terminals[terminal_key] = self.e2e_terminal(
            "exact_current"
        )
        capture.e2e_observed_binding_keys.add(observer_key)
        capture.binding_dispositions["exact_current"] = 1

        summary = capture.finalize_e2e_measurement()

        self.assertEqual(summary["binding_observer_missing_keys"], [terminal_key])
        self.assertEqual(
            summary["binding_observer_without_audit_keys"], [observer_key]
        )
        self.assertEqual(
            summary["terminal_provenance"][0]["full_terminal_key"],
            terminal_key,
        )
        self.assertIsNone(
            summary["terminal_provenance"][0]["observed_exact_key"]
        )

    def test_e2e_duplicate_terminal_count_fails_closed(self) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(7)
        capture.e2e_proposals[key] = self.e2e_proposal()
        capture.e2e_terminals[key] = self.e2e_terminal("exact_current")
        capture.e2e_observed_binding_keys.add(key)
        capture.binding_dispositions["exact_current"] = 1
        capture.e2e_duplicate_bindings = 1

        with self.assertRaisesRegex(
            AssertionError, "terminal provenance duplicate binding callbacks"
        ):
            capture.finalize_e2e_measurement()

    def test_e2e_disposition_aggregate_mismatch_fails_closed(self) -> None:
        capture = self.e2e_ledger_capture()
        key = self.e2e_key(8)
        capture.e2e_proposals[key] = self.e2e_proposal()
        capture.e2e_terminals[key] = self.e2e_terminal("exact_current")
        capture.e2e_observed_binding_keys.add(key)
        capture.binding_dispositions["deferred_predecessor"] = 1

        with self.assertRaisesRegex(
            AssertionError, "terminal provenance disposition mismatch"
        ):
            capture.finalize_e2e_measurement()

    def test_final_isolated_lifecycle_refreshes_nested_evidence(
        self,
    ) -> None:
        stale = {
            "alive": True,
            "exitcode": None,
            "target_process_lifecycle": {
                "cleanup_stage": "spawn_observed",
                "unresolved": True,
            },
            "associated_child_lifecycle": [{"pid": 96}],
        }
        final = {
            "alive": False,
            "exitcode": 0,
            "target_process_lifecycle": (
                valid_isolated_publisher_lifecycle()[
                    "target_process_lifecycle"
                ]
            ),
            "target_cleanup_failure": "",
            "associated_child_lifecycle": [],
            "associated_child_cleanup_failure": "",
            "associated_child_sample_failures": [],
        }
        refresh_isolated_publisher_lifecycle(stale, final)
        self.assertFalse(stale["alive"])
        self.assertEqual(stale["exitcode"], 0)
        self.assertEqual(
            stale["target_process_lifecycle"]["cleanup_stage"],
            "absent_no_residue",
        )
        self.assertEqual(stale["associated_child_lifecycle"], [])
        final["target_process_lifecycle"]["cleanup_stage"] = "mutated"
        self.assertEqual(
            stale["target_process_lifecycle"]["cleanup_stage"],
            "absent_no_residue",
        )

    def test_pre_run_cleanup_uses_only_known_handles_in_fixed_order(
        self,
    ) -> None:
        events: list[str] = []
        processes = [
            type("ProcessHandle", (), {"label": label})()
            for label in ("proposal", "state_lattice", "pure_pursuit")
        ]
        state_log = Mock()
        state_log.close.side_effect = lambda: events.append("state_log")
        pp_log = Mock()
        pp_log.close.side_effect = lambda: events.append("pp_log")
        subscriptions = [object(), object()]
        capture = type(
            "CaptureHandle",
            (),
            {"subscriptions": subscriptions},
        )()
        node = Mock()
        node.destroy_subscription.side_effect = (
            lambda subscription: events.append(
                f"subscription[{subscriptions.index(subscription)}]"
            )
        )
        isolated_publisher = Mock()
        isolated_publisher.close.side_effect = (
            lambda: events.append("isolated_publisher")
        )
        ring = Mock()
        ring.close.side_effect = lambda: events.append("ring")
        primary = RuntimeError("publisher startup failed")

        cleanup_errors = cleanup_pre_run_case_failure(
            primary,
            node=node,
            capture=capture,
            isolated_input_publisher=isolated_publisher,
            proposal_relay=processes[0],
            state_lattice_process=processes[1],
            process=processes[2],
            state_lattice_log=state_log,
            pp_log=pp_log,
            ring=ring,
            stop_process_action=(
                lambda handle: events.append(f"stop:{handle.label}")
            ),
        )

        self.assertEqual(cleanup_errors, [])
        self.assertEqual(
            events,
            [
                "isolated_publisher",
                "stop:proposal",
                "stop:state_lattice",
                "stop:pure_pursuit",
                "state_log",
                "pp_log",
                "subscription[0]",
                "subscription[1]",
                "ring",
            ],
        )
        self.assertEqual(
            primary._pre_run_case_cleanup_failures,
            [],
        )

    def test_pre_run_cleanup_continues_and_preserves_primary_error(
        self,
    ) -> None:
        events: list[str] = []
        proposal = object()
        state_lattice = object()
        pure_pursuit = object()

        def stop_action(handle: object) -> None:
            events.append(
                {
                    id(proposal): "proposal",
                    id(state_lattice): "state_lattice",
                    id(pure_pursuit): "pure_pursuit",
                }[id(handle)]
            )
            if handle is state_lattice:
                raise RuntimeError("state stop failed")

        state_log = Mock()
        state_log.close.side_effect = RuntimeError("state log failed")
        pp_log = Mock()
        pp_log.close.side_effect = lambda: events.append("pp_log")
        subscriptions = [object(), object()]
        capture = type(
            "CaptureHandle",
            (),
            {"subscriptions": subscriptions},
        )()
        node = Mock()

        def destroy_subscription(subscription: object) -> None:
            index = subscriptions.index(subscription)
            events.append(f"subscription[{index}]")
            if index == 0:
                raise RuntimeError("subscription failed")

        node.destroy_subscription.side_effect = destroy_subscription
        isolated_publisher = Mock()

        def close_isolated_publisher() -> None:
            events.append("isolated_publisher")
            raise RuntimeError("isolated publisher close failed")

        isolated_publisher.close.side_effect = close_isolated_publisher
        ring = Mock()

        def close_ring() -> None:
            events.append("ring")
            raise RuntimeError("ring failed")

        ring.close.side_effect = close_ring
        primary = ValueError("primary constructor failure")
        original_primary = primary

        cleanup_errors = cleanup_pre_run_case_failure(
            primary,
            node=node,
            capture=capture,
            isolated_input_publisher=isolated_publisher,
            proposal_relay=proposal,
            state_lattice_process=state_lattice,
            process=pure_pursuit,
            state_lattice_log=state_log,
            pp_log=pp_log,
            ring=ring,
            stop_process_action=stop_action,
        )

        self.assertIs(primary, original_primary)
        self.assertEqual(
            events,
            [
                "isolated_publisher",
                "proposal",
                "state_lattice",
                "pure_pursuit",
                "pp_log",
                "subscription[0]",
                "subscription[1]",
                "ring",
            ],
        )
        self.assertEqual(
            cleanup_errors,
            primary._pre_run_case_cleanup_failures,
        )
        self.assertIn("primary constructor failure", str(primary))
        self.assertIn("isolated_input_publisher=RuntimeError", str(primary))
        self.assertIn("state_lattice_process=RuntimeError", str(primary))
        self.assertIn("state_lattice_log=RuntimeError", str(primary))
        self.assertIn("capture_subscription[0]=RuntimeError", str(primary))
        self.assertIn("ring=RuntimeError", str(primary))

    def test_pre_run_cleanup_accepts_none_and_already_stopped_process(
        self,
    ) -> None:
        process = Mock()
        process.poll.return_value = 0
        capture = type("CaptureHandle", (), {"subscriptions": []})()
        primary = RuntimeError("primary")

        with patch(
            "c002ay0_pp_runtime_measurement.os.kill"
        ) as kill:
            cleanup_errors = cleanup_pre_run_case_failure(
                primary,
                node=Mock(),
                capture=capture,
                isolated_input_publisher=None,
                proposal_relay=None,
                state_lattice_process=None,
                process=process,
                state_lattice_log=None,
                pp_log=None,
                ring=None,
                stop_process_action=stop_process,
            )

        self.assertEqual(cleanup_errors, [])
        process.poll.assert_called_once_with()
        process.wait.assert_not_called()
        kill.assert_not_called()
        self.assertEqual(str(primary), "primary")

    def test_isolated_startup_is_locally_guarded_before_main_run_try(
        self,
    ) -> None:
        tree = ast.parse(inspect.getsource(run_case))
        guarded_blocks = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.If)
            and isinstance(node.test, ast.Name)
            and node.test.id == "isolated_input_publisher_diagnostic"
            and node.body
            and isinstance(node.body[0], ast.Try)
        ]

        self.assertEqual(len(guarded_blocks), 1)
        startup_try = guarded_blocks[0].body[0]
        self.assertTrue(
            any(
                isinstance(call.func, ast.Name)
                and call.func.id == "cleanup_pre_run_case_failure"
                for handler in startup_try.handlers
                for call in ast.walk(handler)
                if isinstance(call, ast.Call)
            )
        )
        guarded_calls = {
            call.func.id
            for call in ast.walk(startup_try)
            if isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
        }
        self.assertIn("IsolatedInputPublisher", guarded_calls)
        self.assertIn("synchronize_isolated_input_phase", guarded_calls)

    @staticmethod
    def complete_single_result_contract(
        result: dict[str, object],
    ) -> None:
        result.setdefault("isolated_input_publisher_diagnostic", False)
        if result["isolated_input_publisher_diagnostic"] is True:
            result.setdefault(
                "dds_environment_contract",
                copy.deepcopy(DDS_ENVIRONMENT_CONTRACT),
            )
        result.setdefault("fixture_session_residue", [])
        result.setdefault(
            "fixture_session_reconciliation",
            {"valid": True},
        )
        evidence = result["scheduling_evidence"]["evidence"]
        for item in evidence.values():
            item.setdefault("scenario_integrity_valid", True)
            item.setdefault("residue_pids", [])
            item.setdefault(
                "collection_status",
                {
                    "process_snapshot": {"status": "completed"},
                    "reanchor": {"status": "completed"},
                },
            )
            item.setdefault(
                "process_lifecycle",
                (
                    [valid_isolated_publisher_lifecycle()]
                    if result["isolated_input_publisher_diagnostic"] is True
                    else [
                        {
                            "pid": 1234,
                            "role": "fixture_helper",
                            "provenance": "synthetic_test",
                            "cleanup_stage": "closed_no_residue",
                            "alive": False,
                            "exitcode": 0,
                            "unresolved": False,
                            "identity_before_cleanup": {
                                "pid": 1234,
                                "readable": True,
                                "classification": "live",
                                "state": "S",
                                "ppid": 1,
                                "session_id": 1230,
                                "starttime_ticks": 99,
                                "cmdline": "synthetic",
                                "comm": "synthetic",
                                "cgroup": ["0::/synthetic"],
                            },
                        }
                    ]
                ),
            )
            if result["isolated_input_publisher_diagnostic"] is True:
                item.setdefault(
                    "dds_environment_contract",
                    copy.deepcopy(DDS_ENVIRONMENT_CONTRACT),
                )
                phase_ack = {
                    "kind": 2,
                    "sequence": 1,
                    "generation": 1,
                    "tick_index": 1,
                    "publish_complete_watermark": 1,
                    "stamp_ns": FIXED_SEC * 1_000_000_000 + 10_000_000,
                    "request_stage": "synthetic_test",
                }
                item.setdefault("isolated_phase_acks", [phase_ack])
                item.setdefault(
                    "reanchor_fence",
                    {
                        "completed": True,
                        "isolated_post_ack_exact_cohort_valid": True,
                        "isolated_phase_ack": {
                            key: value
                            for key, value in phase_ack.items()
                            if key != "request_stage"
                        },
                    },
                )
                item.setdefault(
                    "isolated_observer_drain",
                    {
                        "applicable": True,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "input_release_authority": (
                            "isolated_input_publisher"
                        ),
                        "input_release_period_sec": 0.01,
                        "input_release_gap_gate_sec": 0.012,
                        "input_release_deadline_applies": False,
                        "observer_processing_in_input_release_deadline": (
                            False
                        ),
                        "canonical_hash_ledger_deferred": False,
                        "saturation_count": 0,
                        "step_count": 1,
                        "spin_observation": {
                            "integrity_failure_count": 0,
                        },
                    },
                )
            drain = item.setdefault(
                (
                    "isolated_observer_drain"
                    if result["isolated_input_publisher_diagnostic"] is True
                    else "paced_callback_drain"
                ),
                {},
            )
            tick_observation = drain.setdefault(
                "tick_observation",
                {},
            )
            deadline_miss_count = int(
                drain.get("deadline_miss_count", 0)
            )
            tick_observation.setdefault(
                "last",
                {
                    "fatal_phase": (
                        "zero_wait" if deadline_miss_count else ""
                    ),
                    "spin_attempts": 2 if deadline_miss_count else 1,
                },
            )

    @staticmethod
    def canonical_wire(value: object) -> str:
        return strict_json_text(
            json_safe_artifact(value),
            sort_keys=True,
            separators=(",", ":"),
        )

    @staticmethod
    def representative_messages() -> list[tuple[str, object]]:
        command = AckermannControlCommand()
        command.stamp.sec = 12
        command.stamp.nanosec = 34
        command.longitudinal.acceleration = float("nan")

        tracking = ControllerTrackingStatus()
        tracking.header.stamp.sec = 56
        tracking.header.stamp.nanosec = 78
        tracking.command_age_sec = float("inf")

        command_envelope = ControllerCommandEnvelope()
        command_envelope.header.stamp.sec = 90
        command_envelope.header.stamp.nanosec = 12
        command_envelope.producer_instance_id = 123
        command_envelope.command_sequence = 456
        command_envelope.command_age_sec = float("-inf")
        command_envelope.command.longitudinal.acceleration = float("nan")

        execution_envelope = ControllerExecutionEnvelope()
        execution_envelope.header.stamp.sec = 34
        execution_envelope.header.stamp.nanosec = 56
        execution_envelope.schema_version = 2
        execution_envelope.producer_instance_id = 123
        execution_envelope.command_sequence = 456
        execution_envelope.command_envelope.producer_instance_id = 123
        execution_envelope.command_envelope.command_sequence = 456
        execution_envelope.command_envelope.command_age_sec = float("inf")
        execution_envelope.witness.source_generation = 98
        execution_envelope.witness.base_source_generation = 99
        execution_envelope.witness.trajectory_progress_m = float("nan")
        execution_envelope.witness.required_spatial_horizon_m = float("inf")
        execution_envelope.witness.raw_steering_tire_angle_rad = float("-inf")
        for index in range(100):
            base_point = TrajectoryPoint()
            base_point.pose.position.x = float(index)
            base_point.pose.position.y = float(index) * 0.1
            execution_envelope.witness.base_trajectory.points.append(
                base_point
            )
            applied_point = TrajectoryPoint()
            applied_point.pose.position.x = float(index) + 0.2
            applied_point.pose.position.y = float(index) * 0.1 + 0.3
            execution_envelope.witness.applied_trajectory.points.append(
                applied_point
            )
            rollout = ExecutionSweepSample()
            rollout.elapsed_time_sec = float(index) * 0.01
            rollout.pose.position.x = float(index) + 0.1
            execution_envelope.witness.rollout_samples.append(rollout)

        return [
            ("command", command),
            ("tracking", tracking),
            ("command_envelope", command_envelope),
            ("execution_envelope", execution_envelope),
        ]

    def test_dict_first_canonicalization_matches_legacy_wire_and_hash(
        self,
    ) -> None:
        for topic, message in self.representative_messages():
            with self.subTest(topic=topic):
                before = self.canonical_wire(
                    message_to_ordereddict(message)
                )
                legacy = legacy_canonical_debug(topic, message)
                current = canonical_debug(topic, message)
                after = self.canonical_wire(
                    message_to_ordereddict(message)
                )
                self.assertEqual(
                    self.canonical_wire(current),
                    self.canonical_wire(legacy),
                )
                self.assertEqual(
                    canonical_payload_hash(current),
                    canonical_payload_hash(legacy),
                )
                self.assertEqual(after, before)

    @staticmethod
    def cohort(
        stamp_ns: int,
        sequence: int,
        producer_instance_id: int = 42,
    ) -> dict[str, dict[str, object]]:
        tracking = {
            "plan_generation": 7,
            "mpc_horizon_usable": False,
            "pp_command_fresh": False,
            "trajectory_tracking_usable": False,
            "lateral_stop_authority_kind": 0,
            "lateral_stop_transaction_pass_direction": 0,
            "lateral_stop_authority_token": 0,
            "reason": "missing_odom",
        }
        command = {
            "longitudinal": {
                "speed": 0.0,
                "acceleration": -1.5,
            }
        }
        command_envelope = {
            **tracking,
            "command": command,
        }
        semantics = {
            "command": command,
            "raw_command": command,
            "tracking": tracking,
            "command_envelope": command_envelope,
            "execution_envelope": {
                "command_envelope": command_envelope,
            },
        }
        identities = {
            "command": (
                stamp_ns,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                "",
            ),
            "raw_command": (
                stamp_ns,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                "",
            ),
            "tracking": (
                stamp_ns,
                0,
                0,
                7,
                0,
                0,
                0,
                0,
                "",
            ),
            "command_envelope": (
                stamp_ns,
                producer_instance_id,
                sequence,
                7,
                0,
                0,
                0,
                0,
                "",
            ),
            "execution_envelope": (
                stamp_ns,
                producer_instance_id,
                sequence,
                7,
                0,
                0,
                0,
                0,
                "",
            ),
        }
        return {
            topic: {
                "topic": topic,
                "identity": identities[topic],
                "semantic": semantics[topic],
                "payload_hash": f"{topic}-{stamp_ns}-{sequence}",
                "steering_abs": None,
            }
            for topic in CAPTURE_TOPICS
        }

    def stage(
        self,
        records: dict[str, dict[str, object]],
        *,
        startup: bool = True,
    ) -> None:
        for record in records.values():
            if startup:
                self.capture._record_startup_sample(record)
            self.capture._stage_alignment_sample(record)
            if self.capture.enabled:
                self.capture._materialize_ready_cycles()

    def test_fresh_watermark_reanchors_then_captures_exact_next(self) -> None:
        evidence_stamp_ns = 100_000_000_000
        old = self.cohort(evidence_stamp_ns, 10)
        self.stage(old)
        self.capture._record_startup_sample(old["command"])
        self.capture._stage_alignment_sample(old["command"])

        fresh = self.cohort(evidence_stamp_ns + 1_000_000, 11)
        self.stage(fresh)

        self.assertFalse(
            self.capture.has_completed_cycle(
                11,
                minimum_stamp_exclusive_ns=(
                    evidence_stamp_ns + 1_000_000
                ),
                expected_producer_instance_id=42,
            )
        )
        self.assertFalse(
            self.capture.has_completed_cycle(
                11,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=99,
            )
        )
        self.assertTrue(
            self.capture.has_completed_cycle(
                11,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=42,
            )
        )

        anchor = self.capture.arm_after_completed_cycle(
            11,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )
        self.assertEqual(
            anchor,
            (evidence_stamp_ns + 1_000_000, 11, 42),
        )
        self.assertEqual(self.capture.next_expected_sequence, 12)
        self.assertEqual(
            self.capture.capture_after_stamp_ns,
            evidence_stamp_ns + 1_000_000,
        )
        self.assertNotIn(evidence_stamp_ns, self.capture.alignment_cycles)
        self.assertGreater(len(self.capture.startup_ledger), 0)

        self.stage(
            self.cohort(evidence_stamp_ns + 2_000_000, 12),
            startup=False,
        )
        self.assertEqual(self.capture.materialized_sequences, [12])

    def test_prearm_storage_budget_is_finite_and_campaign_wide(self) -> None:
        self.assertEqual(PREARM_STORAGE_PHASE_BUDGET_MS, 28_320)
        self.assertEqual(PREARM_STORAGE_BUDGET_MS, 29_320)
        self.assertEqual(PREARM_STORAGE_REQUIRED_MESSAGES, 14_660)
        self.assertEqual(PREARM_CAPTURE_CAPACITY_MESSAGES, 16_384)
        self.assertEqual(
            STARTUP_LEDGER_MAX_MESSAGES,
            PREARM_CAPTURE_CAPACITY_MESSAGES,
        )
        self.assertEqual(
            ALIGNMENT_STAGE_MAX_MESSAGES,
            PREARM_CAPTURE_CAPACITY_MESSAGES,
        )
        self.assertLess(
            PREARM_STORAGE_REQUIRED_MESSAGES,
            PREARM_CAPTURE_CAPACITY_MESSAGES,
        )
        summary = self.capture.startup_alignment_summary()
        self.assertEqual(
            summary["prearm_storage_budget_ms"],
            PREARM_STORAGE_BUDGET_MS,
        )
        self.assertEqual(
            summary["prearm_storage_required_messages"],
            PREARM_STORAGE_REQUIRED_MESSAGES,
        )
        self.assertEqual(
            summary["ledger_capacity"],
            PREARM_CAPTURE_CAPACITY_MESSAGES,
        )
        self.assertEqual(
            summary["alignment_stage_capacity"],
            PREARM_CAPTURE_CAPACITY_MESSAGES,
        )

    def test_prearm_required_messages_fit_and_are_observable(self) -> None:
        template = self.cohort(1, 1)["command"]
        for index in range(PREARM_STORAGE_REQUIRED_MESSAGES):
            record = dict(template)
            record["identity"] = (
                index + 1,
                *template["identity"][1:],
            )
            self.capture._record_startup_sample(record)
            self.capture._stage_alignment_sample(record)

        summary = self.capture.startup_alignment_summary()
        self.assertFalse(summary["ledger_overflow"])
        self.assertEqual(
            summary["ledger_max_message_count"],
            PREARM_STORAGE_REQUIRED_MESSAGES,
        )
        self.assertFalse(summary["alignment_stage_overflow"])
        self.assertEqual(
            summary["alignment_stage_max_message_count"],
            PREARM_STORAGE_REQUIRED_MESSAGES,
        )
        self.assertEqual(
            summary["alignment_stage_pending_message_count"],
            PREARM_STORAGE_REQUIRED_MESSAGES,
        )

    def test_paced_tick_drains_fresh_exact_cohort_with_a_hard_bound(
        self,
    ) -> None:
        class FakeClock:
            def __init__(self) -> None:
                self.now = 0.0

            def monotonic(self) -> float:
                return self.now

            def sleep(self, duration_sec: float) -> None:
                self.now += duration_sec

        fake_clock = FakeClock()
        fresh = self.cohort(2_000_000, 2)
        queue: list[dict[str, object] | None] = [
            None for _ in range(20)
        ]
        queue.extend(fresh.values())
        publish_count = 0

        def publish() -> None:
            nonlocal publish_count
            publish_count += 1

        def spin_once(timeout_sec: float) -> None:
            if queue:
                record = queue.pop(0)
                if record is not None:
                    self.capture._stage_alignment_sample(record)
                fake_clock.now += 0.0001
            else:
                fake_clock.now += timeout_sec

        spin_once(0.0)
        self.assertFalse(
            self.capture.has_completed_cycle(
                2,
                minimum_stamp_exclusive_ns=1_000_000,
                expected_producer_instance_id=42,
            )
        )
        callback_attempts = drive_paced_fixture_tick(
            publish,
            spin_once,
            monotonic=fake_clock.monotonic,
            sleep=fake_clock.sleep,
        )
        self.assertEqual(publish_count, 1)
        self.assertEqual(
            callback_attempts,
            FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK,
        )
        self.assertGreaterEqual(
            fake_clock.now,
            FIXTURE_INPUT_PERIOD_SEC,
        )
        self.assertTrue(
            self.capture.has_completed_cycle(
                2,
                minimum_stamp_exclusive_ns=1_000_000,
                expected_producer_instance_id=42,
            )
        )

    def test_paced_tick_stops_at_callback_limit_and_keeps_cadence(
        self,
    ) -> None:
        now = 0.0
        spin_count = 0

        def monotonic() -> float:
            return now

        def sleep(duration_sec: float) -> None:
            nonlocal now
            now += duration_sec

        def spin_once(_: float) -> None:
            nonlocal spin_count
            spin_count += 1

        callback_attempts = drive_paced_fixture_tick(
            lambda: None,
            spin_once,
            monotonic=monotonic,
            sleep=sleep,
        )
        self.assertEqual(
            callback_attempts,
            FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK,
        )
        self.assertEqual(
            spin_count,
            FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK,
        )
        self.assertEqual(now, FIXTURE_INPUT_PERIOD_SEC)

    def test_paced_tick_waits_once_then_uses_zero_time_polls(
        self,
    ) -> None:
        now = 0.0
        observed_timeouts: list[float] = []
        statistics: dict[str, object] = {}

        def monotonic() -> float:
            return now

        def sleep(duration_sec: float) -> None:
            nonlocal now
            now += duration_sec

        def spin_once(timeout_sec: float) -> None:
            nonlocal now
            observed_timeouts.append(timeout_sec)
            now += timeout_sec

        callback_attempts = drive_paced_fixture_tick(
            lambda: None,
            spin_once,
            statistics=statistics,
            monotonic=monotonic,
            sleep=sleep,
        )
        self.assertEqual(
            callback_attempts,
            FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK,
        )
        self.assertEqual(
            observed_timeouts,
            [0.0005]
            + [0.0]
            * (FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK - 1),
        )
        self.assertEqual(statistics["max_blocking_wait_attempts"], 1)
        self.assertEqual(
            statistics["max_zero_wait_attempts"],
            FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK - 1,
        )
        self.assertEqual(statistics["deadline_miss_count"], 0)
        self.assertEqual(statistics["guard_exhaustion_count"], 0)
        self.assertLess(
            statistics["max_elapsed_sec"],
            FIXTURE_INPUT_PERIOD_SEC
            - FIXTURE_DRAIN_DEADLINE_GUARD_SEC,
        )
        tick = statistics["tick_observation"]["last"]
        self.assertFalse(tick["idle_break_enabled"])
        self.assertEqual(tick["idle_break_count"], 0)
        self.assertEqual(tick["break_phase"], "")
        self.assertTrue(tick["deferred_not_proven"])
        self.assertEqual(now, FIXTURE_INPUT_PERIOD_SEC)

    def test_paced_tick_aggregates_publish_wait_and_callback_costs(
        self,
    ) -> None:
        now = 0.0
        statistics: dict[str, object] = {}
        observations = iter(
            (
                {
                    "wall_elapsed_ns": 10,
                    "thread_cpu_elapsed_ns": 4,
                    "callback_generation_delta": 1,
                },
                {
                    "wall_elapsed_ns": 20,
                    "thread_cpu_elapsed_ns": 6,
                    "callback_generation_delta": 0,
                },
                {
                    "wall_elapsed_ns": 30,
                    "thread_cpu_elapsed_ns": 8,
                    "callback_generation_delta": 1,
                },
            )
        )
        wall_clock = iter((100, 130))
        thread_cpu_clock = iter((200, 207))

        def sleep(duration_sec: float) -> None:
            nonlocal now
            now += duration_sec

        drive_paced_fixture_tick(
            lambda: None,
            lambda _: next(observations),
            max_callbacks=3,
            statistics=statistics,
            monotonic=lambda: now,
            sleep=sleep,
            monotonic_ns=lambda: next(wall_clock),
            thread_time_ns=lambda: next(thread_cpu_clock),
        )

        tick_statistics = statistics["tick_observation"]
        self.assertEqual(
            tick_statistics["tail_capacity"],
            FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY,
        )
        self.assertEqual(len(tick_statistics["tail"]), 1)
        tick = tick_statistics["last"]
        self.assertEqual(
            tick["publish"],
            {
                "wall_elapsed_ns": 30,
                "thread_cpu_elapsed_ns": 7,
            },
        )
        self.assertEqual(tick["blocking_wait"]["count"], 1)
        self.assertEqual(tick["zero_wait"]["count"], 2)
        self.assertEqual(tick["callback"]["count"], 2)
        self.assertEqual(tick["callback"]["wall_elapsed_ns_sum"], 40)
        self.assertEqual(
            tick["callback"]["thread_cpu_elapsed_ns_sum"],
            12,
        )
        self.assertEqual(tick["callback"]["max_wall_elapsed_ns"], 30)
        self.assertEqual(tick["idle"]["count"], 1)
        self.assertEqual(tick["idle"]["wall_elapsed_ns_sum"], 20)
        self.assertEqual(tick["idle"]["thread_cpu_elapsed_ns_sum"], 6)
        self.assertEqual(tick["wall_elapsed_ns"], 90)
        self.assertEqual(tick["thread_cpu_elapsed_ns"], 25)
        self.assertEqual(tick_statistics["max_wall"], tick)
        self.assertEqual(tick_statistics["max_thread_cpu"], tick)

    def test_idle_break_stops_after_first_zero_wait_idle(self) -> None:
        now = 0.0
        timeouts: list[float] = []
        statistics: dict[str, object] = {}
        deltas = iter((1, 0))

        def spin_once(timeout_sec: float) -> dict[str, object]:
            timeouts.append(timeout_sec)
            return {
                "wall_elapsed_ns": 1,
                "thread_cpu_elapsed_ns": 1,
                "callback_generation_delta": next(deltas),
            }

        callback_attempts = drive_paced_fixture_tick(
            lambda: None,
            spin_once,
            break_on_first_zero_idle=True,
            statistics=statistics,
            monotonic=lambda: now,
            sleep=lambda _: None,
        )
        self.assertEqual(callback_attempts, 2)
        self.assertEqual(timeouts, [FIXTURE_DRAIN_SPIN_WAIT_SEC, 0.0])
        tick = statistics["tick_observation"]["last"]
        self.assertTrue(tick["idle_break_enabled"])
        self.assertEqual(tick["idle_break_count"], 1)
        self.assertEqual(tick["break_phase"], "zero_wait")
        self.assertTrue(tick["deferred_not_proven"])
        self.assertEqual(tick["callback"]["count"], 1)
        self.assertEqual(tick["idle"]["count"], 1)

    def test_isolated_observer_drain_is_bounded_without_input_deadline(
        self,
    ) -> None:
        observations = iter((1, 1, 0))
        statistics: dict[str, object] = {}
        wall_clock = iter((1_000, 12_000_000))
        cpu_clock = iter((2_000, 11_000_000))

        attempts = drive_isolated_observer_drain_step(
            lambda _: {
                "callback_generation_delta": next(observations),
            },
            max_callbacks=4,
            statistics=statistics,
            monotonic_ns=lambda: next(wall_clock),
            thread_time_ns=lambda: next(cpu_clock),
        )

        self.assertEqual(attempts, 3)
        self.assertEqual(statistics["step_count"], 1)
        self.assertEqual(statistics["callback_count"], 2)
        self.assertEqual(statistics["idle_count"], 1)
        self.assertEqual(statistics["saturation_count"], 0)
        step = statistics["step_observation"]["last"]
        self.assertGreater(step["wall_elapsed_ns"], 10_000_000)
        self.assertFalse(step["input_release_deadline_applies"])
        self.assertFalse(step["saturated"])
        self.assertEqual(step["break_phase"], "zero_wait")

    def test_isolated_observer_drain_saturation_is_hard_invalid(
        self,
    ) -> None:
        statistics: dict[str, object] = {}
        with self.assertRaisesRegex(
            AssertionError,
            "isolated_observer_drain_saturated",
        ):
            drive_isolated_observer_drain_step(
                lambda _: {"callback_generation_delta": 1},
                max_callbacks=2,
                statistics=statistics,
                monotonic_ns=lambda: 0,
                thread_time_ns=lambda: 0,
            )
        self.assertEqual(statistics["saturation_count"], 1)
        self.assertTrue(
            statistics["step_observation"]["last"]["saturated"]
        )

    def test_spin_observation_aggregates_exact_topic_boundaries(
        self,
    ) -> None:
        statistics: dict[str, object] = {
            "count": 0,
            "tail": [],
        }
        observation = {
            "wall_elapsed_ns": 1_000,
            "thread_cpu_elapsed_ns": 900,
            "last_callback": {"topic": "execution_envelope"},
            "callback_wrapper_attribution": {"valid": True},
            "callback_wrapper_decomposition": {
                "wrapper_body_wall_elapsed_ns": 400,
                "wrapper_body_thread_cpu_elapsed_ns": 390,
                "pre_wrapper_wall_elapsed_ns": 500,
                "pre_wrapper_thread_cpu_elapsed_ns": 450,
                "wrapper_completion_wall_elapsed_ns": 40,
                "wrapper_completion_thread_cpu_elapsed_ns": 30,
                "post_wrapper_wall_elapsed_ns": 60,
                "post_wrapper_thread_cpu_elapsed_ns": 30,
            },
        }
        record_fixture_spin_observation(statistics, observation)
        record_fixture_spin_observation(statistics, observation)
        command_observation = copy.deepcopy(observation)
        command_observation["wall_elapsed_ns"] = 700
        command_observation["last_callback"]["topic"] = "command"
        record_fixture_spin_observation(statistics, command_observation)

        topic = statistics["callback_timing_by_topic"][
            "execution_envelope"
        ]
        self.assertEqual(topic["count"], 2)
        self.assertEqual(topic["wall_elapsed_ns_sum"], 2_000)
        self.assertEqual(topic["max_wall_elapsed_ns"], 1_000)
        self.assertEqual(
            topic["wrapper_body_wall_elapsed_ns_sum"],
            800,
        )
        self.assertEqual(topic["pre_wrapper_wall_elapsed_ns_sum"], 1_000)
        self.assertEqual(
            topic["wrapper_completion_wall_elapsed_ns_sum"],
            80,
        )
        self.assertEqual(topic["post_wrapper_wall_elapsed_ns_sum"], 120)
        command_topic = statistics["callback_timing_by_topic"]["command"]
        self.assertEqual(command_topic["count"], 1)
        self.assertEqual(command_topic["wall_elapsed_ns_sum"], 700)
        self.assertEqual(topic["count"], 2)

    def test_idle_break_keeps_callbacks_until_zero_wait_idle(self) -> None:
        deltas = iter((1, 1, 0))
        statistics: dict[str, object] = {}

        callback_attempts = drive_paced_fixture_tick(
            lambda: None,
            lambda _: {
                "wall_elapsed_ns": 1,
                "thread_cpu_elapsed_ns": 1,
                "callback_generation_delta": next(deltas),
            },
            break_on_first_zero_idle=True,
            statistics=statistics,
            monotonic=lambda: 0.0,
            sleep=lambda _: None,
        )
        self.assertEqual(callback_attempts, 3)
        tick = statistics["tick_observation"]["last"]
        self.assertEqual(tick["callback"]["count"], 2)
        self.assertEqual(tick["idle"]["count"], 1)
        self.assertEqual(tick["idle_break_count"], 1)

    def test_idle_break_defers_late_callback_to_next_tick_once(
        self,
    ) -> None:
        deltas = iter((0, 0, 1, 0))
        processed_callbacks = 0
        statistics: dict[str, object] = {}

        def spin_once(_: float) -> dict[str, object]:
            nonlocal processed_callbacks
            delta = next(deltas)
            processed_callbacks += delta
            return {
                "wall_elapsed_ns": 1,
                "thread_cpu_elapsed_ns": 1,
                "callback_generation_delta": delta,
            }

        for _ in range(2):
            drive_paced_fixture_tick(
                lambda: None,
                spin_once,
                break_on_first_zero_idle=True,
                statistics=statistics,
                monotonic=lambda: 0.0,
                sleep=lambda _: None,
            )
        self.assertEqual(processed_callbacks, 1)
        self.assertEqual(statistics["tick_count"], 2)
        self.assertEqual(
            [tick["idle_break_count"] for tick in statistics[
                "tick_observation"
            ]["tail"]],
            [1, 1],
        )

    def test_idle_break_does_not_stop_sixty_four_callbacks(self) -> None:
        statistics: dict[str, object] = {}
        callback_attempts = drive_paced_fixture_tick(
            lambda: None,
            lambda _: {
                "wall_elapsed_ns": 1,
                "thread_cpu_elapsed_ns": 1,
                "callback_generation_delta": 1,
            },
            break_on_first_zero_idle=True,
            statistics=statistics,
            monotonic=lambda: 0.0,
            sleep=lambda _: None,
        )
        self.assertEqual(
            callback_attempts,
            FIXTURE_DRAIN_MAX_CALLBACKS_PER_TICK,
        )
        tick = statistics["tick_observation"]["last"]
        self.assertEqual(tick["callback"]["count"], 64)
        self.assertEqual(tick["idle_break_count"], 0)
        self.assertEqual(tick["break_phase"], "")

    def test_idle_break_rejects_missing_and_invalid_observations(
        self,
    ) -> None:
        invalid_observations = (
            None,
            {},
            {"callback_generation_delta": "0"},
            {"callback_generation_delta": -1},
            {"callback_generation_delta": 2},
        )
        for observation in invalid_observations:
            with self.subTest(observation=observation):
                with self.assertRaisesRegex(
                    AssertionError,
                    "paced_drain_idle_break_observation_invalid",
                ):
                    drive_paced_fixture_tick(
                        lambda: None,
                        lambda _: observation,
                        break_on_first_zero_idle=True,
                        statistics={},
                        monotonic=lambda: 0.0,
                        sleep=lambda _: None,
                    )

    def test_paced_tick_deadline_miss_is_hard_invalid_and_recorded(
        self,
    ) -> None:
        now = 0.0
        statistics: dict[str, object] = {}

        def monotonic() -> float:
            return now

        def spin_once(timeout_sec: float) -> None:
            nonlocal now
            self.assertLessEqual(
                timeout_sec,
                FIXTURE_INPUT_PERIOD_SEC,
            )
            now += FIXTURE_INPUT_PERIOD_SEC + 0.001

        with self.assertRaisesRegex(
            AssertionError,
            "paced_drain_deadline_miss",
        ):
            drive_paced_fixture_tick(
                lambda: None,
                spin_once,
                statistics=statistics,
                monotonic=monotonic,
                sleep=lambda _: None,
            )
        self.assertEqual(statistics["tick_count"], 1)
        self.assertEqual(statistics["max_spin_attempts"], 1)
        self.assertEqual(statistics["deadline_miss_count"], 1)
        self.assertGreater(
            statistics["max_elapsed_sec"],
            FIXTURE_INPUT_PERIOD_SEC,
        )
        tick = statistics["tick_observation"]["last"]
        self.assertEqual(tick["fatal_phase"], "blocking_wait")
        self.assertTrue(tick["deadline_missed"])
        self.assertEqual(tick["spin_attempts"], 1)

    def test_paced_tick_publish_deadline_miss_is_hard_invalid(
        self,
    ) -> None:
        now = 0.0
        spin_count = 0
        statistics: dict[str, object] = {}

        def monotonic() -> float:
            return now

        def publish() -> None:
            nonlocal now
            now += FIXTURE_INPUT_PERIOD_SEC + 0.001

        def spin_once(_: float) -> None:
            nonlocal spin_count
            spin_count += 1

        with self.assertRaisesRegex(
            AssertionError,
            "paced_drain_deadline_miss.*phase=publish",
        ):
            drive_paced_fixture_tick(
                publish,
                spin_once,
                statistics=statistics,
                monotonic=monotonic,
                sleep=lambda _: None,
            )
        self.assertEqual(spin_count, 0)
        self.assertEqual(statistics["tick_count"], 1)
        self.assertEqual(statistics["max_spin_attempts"], 0)
        self.assertEqual(statistics["deadline_miss_count"], 1)
        tick = statistics["tick_observation"]["last"]
        self.assertEqual(tick["fatal_phase"], "publish")
        self.assertTrue(tick["deadline_missed"])
        self.assertEqual(tick["spin_attempts"], 0)
        self.assertGreaterEqual(tick["publish"]["wall_elapsed_ns"], 0)

    def test_paced_tick_publish_guard_exhaustion_is_hard_invalid(
        self,
    ) -> None:
        now = 0.0
        spin_count = 0
        statistics: dict[str, object] = {}

        def monotonic() -> float:
            return now

        def publish() -> None:
            nonlocal now
            now += (
                FIXTURE_INPUT_PERIOD_SEC
                - FIXTURE_DRAIN_DEADLINE_GUARD_SEC / 2.0
            )

        def spin_once(_: float) -> None:
            nonlocal spin_count
            spin_count += 1

        with self.assertRaisesRegex(
            AssertionError,
            "paced_drain_deadline_guard_exhausted.*phase=publish",
        ):
            drive_paced_fixture_tick(
                publish,
                spin_once,
                statistics=statistics,
                monotonic=monotonic,
                sleep=lambda _: None,
            )
        self.assertEqual(spin_count, 0)
        self.assertEqual(statistics["tick_count"], 1)
        self.assertEqual(statistics["deadline_miss_count"], 0)
        self.assertEqual(statistics["guard_exhaustion_count"], 1)
        tick = statistics["tick_observation"]["last"]
        self.assertEqual(tick["fatal_phase"], "publish")
        self.assertFalse(tick["deadline_missed"])
        self.assertTrue(tick["guard_exhausted"])

    def test_paced_tick_preserves_spin_exception_and_records_fatal_tick(
        self,
    ) -> None:
        statistics: dict[str, object] = {}

        def fail_spin(_: float) -> None:
            raise RuntimeError("synthetic paced spin failure")

        with self.assertRaisesRegex(
            RuntimeError,
            "synthetic paced spin failure",
        ):
            drive_paced_fixture_tick(
                lambda: None,
                fail_spin,
                statistics=statistics,
                monotonic=lambda: 0.0,
                sleep=lambda _: None,
            )
        tick = statistics["tick_observation"]["last"]
        self.assertEqual(tick["fatal_phase"], "blocking_wait")
        self.assertEqual(tick["spin_attempts"], 1)
        self.assertEqual(tick["blocking_wait"]["count"], 0)

    def test_paced_tick_aggregates_observation_from_fatal_spin(
        self,
    ) -> None:
        paced_statistics: dict[str, object] = {}
        spin_statistics: dict[str, object] = {"count": 0, "tail": []}

        class SyntheticCapture:
            callback_completion_generation = 0
            last_callback_observation: dict[str, object] = {}

        def fail_spin(_node: object, *, timeout_sec: float) -> None:
            del timeout_sec
            raise RuntimeError("synthetic observed spin failure")

        def observed_spin(timeout_sec: float) -> object:
            return spin_fixture_once_with_observation(
                object(),
                timeout_sec,
                scheduling_required=True,
                capture=SyntheticCapture(),
                statistics=spin_statistics,
                spin_once_action=fail_spin,
            )

        with self.assertRaisesRegex(
            RuntimeError,
            "synthetic observed spin failure",
        ):
            drive_paced_fixture_tick(
                lambda: None,
                observed_spin,
                statistics=paced_statistics,
                monotonic=lambda: 0.0,
                sleep=lambda _: None,
            )
        tick = paced_statistics["tick_observation"]["last"]
        observation = spin_statistics["last"]
        self.assertEqual(tick["fatal_phase"], "blocking_wait")
        self.assertEqual(tick["blocking_wait"]["count"], 1)
        self.assertEqual(tick["idle"]["count"], 1)
        self.assertEqual(
            tick["blocking_wait"]["wall_elapsed_ns_sum"],
            observation["wall_elapsed_ns"],
        )
        self.assertEqual(
            tick["idle"]["thread_cpu_elapsed_ns_sum"],
            observation["thread_cpu_elapsed_ns"],
        )

    def test_paced_tick_zero_wait_callback_overrun_is_hard_invalid(
        self,
    ) -> None:
        now = 0.0
        observed_timeouts: list[float] = []
        statistics: dict[str, object] = {}

        def monotonic() -> float:
            return now

        def spin_once(timeout_sec: float) -> None:
            nonlocal now
            observed_timeouts.append(timeout_sec)
            if len(observed_timeouts) == 1:
                now += timeout_sec
            else:
                now += FIXTURE_INPUT_PERIOD_SEC

        with self.assertRaisesRegex(
            AssertionError,
            "paced_drain_deadline_miss.*phase=zero_wait",
        ):
            drive_paced_fixture_tick(
                lambda: None,
                spin_once,
                statistics=statistics,
                monotonic=monotonic,
                sleep=lambda _: None,
            )
        self.assertEqual(observed_timeouts, [0.0005, 0.0])
        self.assertEqual(statistics["max_blocking_wait_attempts"], 1)
        self.assertEqual(statistics["max_zero_wait_attempts"], 1)
        self.assertEqual(statistics["deadline_miss_count"], 1)

    def test_spin_observation_is_bounded_and_keeps_cost_maxima(
        self,
    ) -> None:
        statistics: dict[str, object] = {
            "count": 0,
            "tail_capacity": FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY,
            "tail": [],
        }
        total = FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY + 3
        for index in range(total):
            record_fixture_spin_observation(
                statistics,
                {
                    "call_index": index + 1,
                    "timeout_requested_ns": 0,
                    "wall_elapsed_ns": index * 10,
                    "thread_cpu_elapsed_ns": (total - index) * 5,
                    "callback_generation_delta": index % 2,
                },
            )
        self.assertEqual(statistics["count"], total)
        self.assertEqual(
            len(statistics["tail"]),
            FIXTURE_SPIN_OBSERVATION_TAIL_CAPACITY,
        )
        self.assertEqual(statistics["tail"][0]["call_index"], 4)
        self.assertEqual(statistics["last"]["call_index"], total)
        self.assertEqual(
            statistics["max_wall"]["call_index"],
            total,
        )
        self.assertEqual(
            statistics["max_thread_cpu"]["call_index"],
            1,
        )

    def test_capture_callback_instrumentation_records_one_completion(
        self,
    ) -> None:
        capture = Capture.__new__(Capture)
        capture.callback_instrumentation_enabled = True
        capture.callback_wrapper_entry_generation = 0
        capture.callback_completion_generation = 0
        capture.last_callback_observation = {}
        received: list[object] = []
        callback = capture._instrument_callback(
            "synthetic_topic",
            received.append,
        )
        message = object()
        callback(message)
        self.assertEqual(received, [message])
        self.assertEqual(capture.callback_wrapper_entry_generation, 1)
        self.assertEqual(capture.callback_completion_generation, 1)
        self.assertEqual(
            capture.last_callback_observation["topic"],
            "synthetic_topic",
        )
        self.assertGreaterEqual(
            capture.last_callback_observation["wall_elapsed_ns"],
            0,
        )
        self.assertGreaterEqual(
            capture.last_callback_observation["thread_cpu_elapsed_ns"],
            0,
        )

    def test_spin_wrapper_decomposition_is_bounded_and_nonnegative(
        self,
    ) -> None:
        capture = Capture.__new__(Capture)
        capture.callback_instrumentation_enabled = True
        capture.callback_wrapper_entry_generation = 0
        capture.callback_completion_generation = 0
        capture.last_callback_observation = {}
        callback = capture._instrument_callback(
            "synthetic_topic",
            lambda _: None,
        )
        wall_clock = iter((100, 120, 150, 160, 180))
        thread_cpu_clock = iter((200, 210, 225, 230, 250))

        def spin_once(
            _node: object,
            *,
            timeout_sec: float,
        ) -> None:
            del timeout_sec
            callback(object())

        with (
            patch(
                "c002ay0_pp_runtime_measurement.time.monotonic_ns",
                side_effect=lambda: next(wall_clock),
            ),
            patch(
                "c002ay0_pp_runtime_measurement.time.thread_time_ns",
                side_effect=lambda: next(thread_cpu_clock),
            ),
        ):
            observation = spin_fixture_once_with_observation(
                object(),
                0.0,
                scheduling_required=True,
                capture=capture,
                statistics={"count": 0, "tail": []},
                spin_once_action=spin_once,
            )
        decomposition = observation["callback_wrapper_decomposition"]
        self.assertEqual(
            decomposition["scope"],
            "fixture_spin_and_instrumented_callback_wrapper",
        )
        self.assertFalse(decomposition["dds_subcomponent_attribution"])
        self.assertFalse(decomposition["executor_internal_attribution"])
        self.assertEqual(decomposition["expected_wrapper_event_count"], 1)
        self.assertEqual(decomposition["observed_wrapper_entry_count"], 1)
        self.assertEqual(
            decomposition["observed_wrapper_completion_count"],
            1,
        )
        self.assertEqual(decomposition["wrapper_event_id"], 1)
        self.assertEqual(
            decomposition["wall_timestamps_ns"],
            {
                "spin_start": 100,
                "wrapper_entry": 120,
                "callback_body_end": 150,
                "wrapper_completion": 160,
                "spin_end": 180,
            },
        )
        self.assertEqual(
            decomposition["thread_cpu_timestamps_ns"],
            {
                "spin_start": 200,
                "wrapper_entry": 210,
                "callback_body_end": 225,
                "wrapper_completion": 230,
                "spin_end": 250,
            },
        )
        self.assertEqual(
            decomposition["spin_outer_wrapper_wall_elapsed_ns"],
            80,
        )
        self.assertEqual(
            decomposition["wrapper_body_wall_elapsed_ns"],
            30,
        )
        self.assertEqual(decomposition["pre_wrapper_wall_elapsed_ns"], 20)
        self.assertEqual(
            decomposition["wrapper_completion_wall_elapsed_ns"],
            10,
        )
        self.assertEqual(decomposition["post_wrapper_wall_elapsed_ns"], 20)
        self.assertEqual(decomposition["post_body_wall_elapsed_ns"], 30)
        self.assertEqual(
            decomposition["outer_minus_body_wall_elapsed_ns"],
            50,
        )
        self.assertEqual(
            decomposition["spin_outer_wrapper_thread_cpu_elapsed_ns"],
            50,
        )
        self.assertEqual(
            decomposition["wrapper_body_thread_cpu_elapsed_ns"],
            15,
        )
        self.assertEqual(
            decomposition["pre_wrapper_thread_cpu_elapsed_ns"],
            10,
        )
        self.assertEqual(
            decomposition["wrapper_completion_thread_cpu_elapsed_ns"],
            5,
        )
        self.assertEqual(
            decomposition["post_wrapper_thread_cpu_elapsed_ns"],
            20,
        )
        self.assertEqual(
            decomposition["post_body_thread_cpu_elapsed_ns"],
            25,
        )
        self.assertEqual(
            decomposition["outer_minus_body_thread_cpu_elapsed_ns"],
            35,
        )
        self.assertTrue(
            all(
                value >= 0
                for key, value in decomposition.items()
                if key.endswith("_elapsed_ns")
            )
        )
        self.assertTrue(
            observation["callback_wrapper_attribution"]["valid"]
        )

    def test_spin_wrapper_attribution_fails_closed_when_ambiguous(
        self,
    ) -> None:
        def wrong_event_id(capture: Capture) -> None:
            capture.last_callback_observation["wrapper_event_id"] = 2

        def extra_wrapper_entry(capture: Capture) -> None:
            capture.callback_wrapper_entry_generation += 1

        def nonmonotonic_timestamp(capture: Capture) -> None:
            capture.last_callback_observation[
                "wrapper_completion_wall_ns"
            ] = 149

        def nonmonotonic_thread_cpu_timestamp(capture: Capture) -> None:
            capture.last_callback_observation[
                "wrapper_completion_thread_cpu_ns"
            ] = 224

        def wrong_native_thread(capture: Capture) -> None:
            capture.last_callback_observation[
                "thread_native_id_at_body_end"
            ] += 1

        for name, mutate in (
            ("event_identity", wrong_event_id),
            ("event_count", extra_wrapper_entry),
            ("timestamp_order", nonmonotonic_timestamp),
            (
                "thread_cpu_timestamp_order",
                nonmonotonic_thread_cpu_timestamp,
            ),
            ("native_thread", wrong_native_thread),
        ):
            with self.subTest(name=name):
                capture = Capture.__new__(Capture)
                capture.callback_instrumentation_enabled = True
                capture.callback_wrapper_entry_generation = 0
                capture.callback_completion_generation = 0
                capture.last_callback_observation = {}
                callback = capture._instrument_callback(
                    "synthetic_topic",
                    lambda _: None,
                )
                wall_clock = iter((100, 120, 150, 160, 180))
                thread_cpu_clock = iter((200, 210, 225, 230, 250))

                def spin_once(
                    _node: object,
                    *,
                    timeout_sec: float,
                ) -> None:
                    del timeout_sec
                    callback(object())
                    mutate(capture)

                statistics: dict[str, object] = {
                    "count": 0,
                    "tail": [],
                }
                with (
                    patch(
                        "c002ay0_pp_runtime_measurement.time.monotonic_ns",
                        side_effect=lambda: next(wall_clock),
                    ),
                    patch(
                        "c002ay0_pp_runtime_measurement.time.thread_time_ns",
                        side_effect=lambda: next(thread_cpu_clock),
                    ),
                    self.assertRaisesRegex(
                        AssertionError,
                        "callback_wrapper_decomposition_invalid",
                    ),
                ):
                    spin_fixture_once_with_observation(
                        object(),
                        0.0,
                        scheduling_required=True,
                        capture=capture,
                        statistics=statistics,
                        spin_once_action=spin_once,
                    )
                attribution = statistics["last"][
                    "callback_wrapper_attribution"
                ]
                self.assertFalse(attribution["valid"])
                self.assertEqual(
                    attribution["reason"],
                    "callback_wrapper_decomposition_invalid",
                )
                self.assertNotIn(
                    "callback_wrapper_decomposition",
                    statistics["last"],
                )

    def test_instrumented_callback_exception_preserves_original(
        self,
    ) -> None:
        capture = Capture.__new__(Capture)
        capture.callback_instrumentation_enabled = True
        capture.callback_wrapper_entry_generation = 0
        capture.callback_completion_generation = 0
        capture.last_callback_observation = {}

        def fail_callback(_: object) -> None:
            raise RuntimeError("synthetic callback body failure")

        callback = capture._instrument_callback(
            "synthetic_topic",
            fail_callback,
        )
        statistics: dict[str, object] = {"count": 0, "tail": []}
        wall_clock = iter((100, 120, 150, 160, 180))
        thread_cpu_clock = iter((200, 210, 225, 230, 250))

        with (
            patch(
                "c002ay0_pp_runtime_measurement.time.monotonic_ns",
                side_effect=lambda: next(wall_clock),
            ),
            patch(
                "c002ay0_pp_runtime_measurement.time.thread_time_ns",
                side_effect=lambda: next(thread_cpu_clock),
            ),
            self.assertRaisesRegex(
                RuntimeError,
                "synthetic callback body failure",
            ) as raised,
        ):
            spin_fixture_once_with_observation(
                object(),
                0.0,
                scheduling_required=True,
                capture=capture,
                statistics=statistics,
                spin_once_action=(
                    lambda _node, *, timeout_sec: callback(object())
                ),
            )

        self.assertEqual(capture.callback_wrapper_entry_generation, 1)
        self.assertEqual(capture.callback_completion_generation, 1)
        callback_observation = capture.last_callback_observation
        self.assertEqual(callback_observation["wrapper_event_id"], 1)
        self.assertEqual(callback_observation["wrapper_entry_wall_ns"], 120)
        self.assertEqual(
            callback_observation["callback_body_end_wall_ns"],
            150,
        )
        self.assertEqual(
            callback_observation["wrapper_completion_wall_ns"],
            160,
        )
        attached = getattr(
            raised.exception,
            "_fixture_spin_observation",
        )
        self.assertEqual(attached, statistics["last"])
        self.assertEqual(attached["callback_generation_delta"], 1)
        self.assertTrue(
            attached["callback_wrapper_attribution"]["valid"]
        )

    def test_two_actual_instrumented_callbacks_fail_closed(self) -> None:
        capture = Capture.__new__(Capture)
        capture.callback_instrumentation_enabled = True
        capture.callback_wrapper_entry_generation = 0
        capture.callback_completion_generation = 0
        capture.last_callback_observation = {}
        first_callback = capture._instrument_callback(
            "first_topic",
            lambda _: None,
        )
        second_callback = capture._instrument_callback(
            "second_topic",
            lambda _: None,
        )
        statistics: dict[str, object] = {"count": 0, "tail": []}
        wall_clock = iter((100, 110, 120, 130, 140, 150, 160, 170))
        thread_cpu_clock = iter(
            (200, 210, 220, 230, 240, 250, 260, 270)
        )

        def spin_once(
            _node: object,
            *,
            timeout_sec: float,
        ) -> None:
            del timeout_sec
            first_callback(object())
            second_callback(object())

        with (
            patch(
                "c002ay0_pp_runtime_measurement.time.monotonic_ns",
                side_effect=lambda: next(wall_clock),
            ),
            patch(
                "c002ay0_pp_runtime_measurement.time.thread_time_ns",
                side_effect=lambda: next(thread_cpu_clock),
            ),
            self.assertRaisesRegex(
                AssertionError,
                "callback_generation_delta=2",
            ),
        ):
            spin_fixture_once_with_observation(
                object(),
                0.0,
                scheduling_required=True,
                capture=capture,
                statistics=statistics,
                spin_once_action=spin_once,
            )

        observation = statistics["last"]
        self.assertEqual(
            observation["callback_wrapper_entry_generation_delta"],
            2,
        )
        self.assertEqual(observation["callback_generation_delta"], 2)
        self.assertEqual(
            observation["last_callback"]["topic"],
            "second_topic",
        )
        self.assertFalse(
            observation["callback_wrapper_attribution"]["valid"]
        )
        self.assertEqual(
            observation["callback_wrapper_attribution"]["reason"],
            "callback_generation_delta",
        )

    def test_spin_outer_native_thread_mismatch_fails_closed(self) -> None:
        capture = Capture.__new__(Capture)
        capture.callback_instrumentation_enabled = True
        capture.callback_wrapper_entry_generation = 0
        capture.callback_completion_generation = 0
        capture.last_callback_observation = {}
        callback = capture._instrument_callback(
            "synthetic_topic",
            lambda _: None,
        )
        statistics: dict[str, object] = {"count": 0, "tail": []}
        wall_clock = iter((100, 120, 150, 160, 180))
        thread_cpu_clock = iter((200, 210, 225, 230, 250))
        native_threads = iter((41, 41, 41, 41, 42))

        with (
            patch(
                "c002ay0_pp_runtime_measurement.time.monotonic_ns",
                side_effect=lambda: next(wall_clock),
            ),
            patch(
                "c002ay0_pp_runtime_measurement.time.thread_time_ns",
                side_effect=lambda: next(thread_cpu_clock),
            ),
            patch(
                "c002ay0_pp_runtime_measurement.threading.get_native_id",
                side_effect=lambda: next(native_threads),
            ),
            self.assertRaisesRegex(
                AssertionError,
                "thread_native_id_before=41 thread_native_id_after=42",
            ),
        ):
            spin_fixture_once_with_observation(
                object(),
                0.0,
                scheduling_required=True,
                capture=capture,
                statistics=statistics,
                spin_once_action=(
                    lambda _node, *, timeout_sec: callback(object())
                ),
            )

        observation = statistics["last"]
        self.assertEqual(observation["thread_native_id_before"], 41)
        self.assertEqual(observation["thread_native_id_after"], 42)
        self.assertFalse(
            observation["callback_wrapper_attribution"]["valid"]
        )
        self.assertEqual(
            observation["callback_wrapper_attribution"]["reason"],
            "thread_native_id",
        )

    def test_callback_instrumentation_is_disabled_by_default(self) -> None:
        capture = Capture.__new__(Capture)
        capture.callback_instrumentation_enabled = False
        callback = lambda _: None
        self.assertIs(
            capture._instrument_callback("synthetic_topic", callback),
            callback,
        )

    def test_non_scheduling_spin_keeps_single_spin_without_observation(
        self,
    ) -> None:
        statistics: dict[str, object] = {"count": 0, "tail": []}
        calls: list[tuple[object, float]] = []
        node = object()

        class SyntheticCapture:
            callback_completion_generation = 0
            last_callback_observation: dict[str, object] = {}

        def spin_once_action(
            observed_node: object,
            *,
            timeout_sec: float,
        ) -> None:
            calls.append((observed_node, timeout_sec))

        observation = spin_fixture_once_with_observation(
            node,
            0.0125,
            scheduling_required=False,
            capture=SyntheticCapture(),
            statistics=statistics,
            spin_once_action=spin_once_action,
        )
        self.assertIsNone(observation)
        self.assertEqual(calls, [(node, 0.0125)])
        self.assertEqual(statistics, {"count": 0, "tail": []})

    def test_run_case_spin_wrapper_returns_observation_to_paced_tick(
        self,
    ) -> None:
        syntax = ast.parse(inspect.getsource(run_case))
        wrappers = [
            node
            for node in ast.walk(syntax)
            if isinstance(node, ast.FunctionDef)
            and node.name == "spin_fixture_once"
        ]
        self.assertEqual(len(wrappers), 1)
        wrapper = wrappers[0]
        self.assertEqual(
            ast.unparse(wrapper.returns),
            "dict[str, object] | None",
        )
        self.assertIsInstance(wrapper.body[-1], ast.Return)
        returned = wrapper.body[-1].value
        self.assertIsInstance(returned, ast.Call)
        self.assertIsInstance(returned.func, ast.Name)
        self.assertEqual(
            returned.func.id,
            "spin_fixture_once_with_observation",
        )

    def test_scheduling_run_case_owns_one_dedicated_executor(self) -> None:
        syntax = ast.parse(inspect.getsource(run_case))
        calls = [
            node
            for node in ast.walk(syntax)
            if isinstance(node, ast.Call)
        ]

        def named_call(name: str) -> list[ast.Call]:
            return [
                call
                for call in calls
                if isinstance(call.func, ast.Name)
                and call.func.id == name
            ]

        def attribute_call(
            owner: str,
            name: str,
        ) -> list[ast.Call]:
            return [
                call
                for call in calls
                if isinstance(call.func, ast.Attribute)
                and call.func.attr == name
                and isinstance(call.func.value, ast.Name)
                and call.func.value.id == owner
            ]

        executor_constructors = named_call("SingleThreadedExecutor")
        self.assertEqual(len(executor_constructors), 1)
        self.assertEqual(
            ast.unparse(executor_constructors[0].keywords[0].value),
            "node.context",
        )
        for method in ("add_node", "spin_once", "remove_node", "shutdown"):
            with self.subTest(method=method):
                self.assertEqual(
                    len(attribute_call("scheduling_executor", method)),
                    1,
                )
        add_node_call = attribute_call(
            "scheduling_executor",
            "add_node",
        )[0]
        self.assertEqual(ast.unparse(add_node_call.args[0]), "node")
        shutdown_call = attribute_call(
            "scheduling_executor",
            "shutdown",
        )[0]
        self.assertEqual(
            {
                keyword.arg: ast.literal_eval(keyword.value)
                for keyword in shutdown_call.keywords
            },
            {"timeout_sec": 1.0},
        )
        self.assertEqual(len(attribute_call("rclpy", "spin_once")), 1)

        nested = {
            node.name: node
            for node in ast.walk(syntax)
            if isinstance(node, ast.FunctionDef)
        }
        for helper_name in (
            "spin_fixture_once",
            "drive_input_tick",
            "drive_clock_arm_tick",
            "publish_inputs",
            "publish_clock_and_arm",
        ):
            helper_calls = [
                node
                for node in ast.walk(nested[helper_name])
                if isinstance(node, ast.Call)
            ]
            self.assertFalse(
                any(
                    isinstance(call.func, ast.Name)
                    and call.func.id == "SingleThreadedExecutor"
                    for call in helper_calls
                ),
                helper_name,
            )
            self.assertFalse(
                any(
                    isinstance(call.func, ast.Attribute)
                    and call.func.attr == "add_node"
                    for call in helper_calls
                ),
                helper_name,
            )

        valid_lifecycle = {
            "dedicated": True,
            "created": True,
            "add_node_attempts": 1,
            "add_node_succeeded": True,
            "remove_node_succeeded": True,
            "shutdown_succeeded": True,
            "cleanup_failure": "",
        }
        self.assertTrue(
            scheduling_executor_lifecycle_valid(
                True,
                valid_lifecycle,
            )
        )
        self.assertTrue(
            scheduling_executor_lifecycle_valid(False, {})
        )
        for field, invalid_value in (
            ("dedicated", False),
            ("created", False),
            ("add_node_attempts", 2),
            ("add_node_succeeded", False),
            ("remove_node_succeeded", False),
            ("shutdown_succeeded", False),
            ("cleanup_failure", "shutdown timed out"),
        ):
            with self.subTest(field=field):
                invalid_lifecycle = dict(valid_lifecycle)
                invalid_lifecycle[field] = invalid_value
                self.assertFalse(
                    scheduling_executor_lifecycle_valid(
                        True,
                        invalid_lifecycle,
                    )
                )
        self.assertIn(
            "concurrent scheduling executor spin rejected",
            inspect.getsource(run_case),
        )

    def test_scheduling_spin_records_stable_native_thread_identity(
        self,
    ) -> None:
        statistics: dict[str, object] = {"count": 0, "tail": []}

        class SyntheticCapture:
            callback_completion_generation = 0
            last_callback_observation: dict[str, object] = {}

        returned_observation = spin_fixture_once_with_observation(
            object(),
            0.0,
            scheduling_required=True,
            capture=SyntheticCapture(),
            statistics=statistics,
            spin_once_action=lambda _node, *, timeout_sec: None,
        )
        observation = statistics["last"]
        self.assertEqual(returned_observation, observation)
        self.assertEqual(
            observation["thread_native_id_before"],
            observation["thread_native_id_after"],
        )
        self.assertEqual(observation["callback_generation_delta"], 0)

    def test_scheduling_spin_rejects_multiple_callback_completions(
        self,
    ) -> None:
        statistics: dict[str, object] = {"count": 0, "tail": []}

        class SyntheticCapture:
            callback_completion_generation = 0
            last_callback_observation: dict[str, object] = {}

        capture = SyntheticCapture()

        def complete_two_callbacks(
            _node: object,
            *,
            timeout_sec: float,
        ) -> None:
            del timeout_sec
            capture.callback_completion_generation += 2
            capture.last_callback_observation = {"topic": "second"}

        with self.assertRaisesRegex(
            AssertionError,
            "fixture_spin_integrity_failure callback_generation_delta=2",
        ):
            spin_fixture_once_with_observation(
                object(),
                0.0,
                scheduling_required=True,
                capture=capture,
                statistics=statistics,
                spin_once_action=complete_two_callbacks,
            )
        self.assertEqual(statistics["count"], 1)
        self.assertEqual(
            statistics["last"]["callback_generation_delta"],
            2,
        )
        self.assertEqual(
            statistics["last"]["last_callback"]["topic"],
            "second",
        )
        self.assertEqual(statistics["integrity_failure_count"], 1)
        self.assertEqual(
            statistics["last_integrity_failure"],
            "callback_generation_delta",
        )

    def test_scheduling_spin_preserves_spin_exception_after_observation(
        self,
    ) -> None:
        statistics: dict[str, object] = {"count": 0, "tail": []}

        class SyntheticCapture:
            callback_completion_generation = 0
            last_callback_observation: dict[str, object] = {}

        def fail_spin(_node: object, *, timeout_sec: float) -> None:
            del timeout_sec
            raise RuntimeError("synthetic spin failure")

        with self.assertRaisesRegex(
            RuntimeError,
            "synthetic spin failure",
        ) as raised:
            spin_fixture_once_with_observation(
                object(),
                0.0,
                scheduling_required=True,
                capture=SyntheticCapture(),
                statistics=statistics,
                spin_once_action=fail_spin,
            )
        self.assertEqual(statistics["count"], 1)
        self.assertEqual(
            statistics["last"]["callback_generation_delta"],
            0,
        )
        self.assertEqual(
            raised.exception._fixture_spin_observation,
            statistics["last"],
        )

    def test_single_scheduling_result_classifies_deadline_after_integrity(
        self,
    ) -> None:
        result = {
            "integrity_valid": False,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "evidence": {
                    "off": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 1,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        self.assertEqual(
            classify_single_scheduling_result(result),
            "ZERO_WAIT_OVERRUN_REPRODUCED_DIAGNOSTIC_STOP",
        )

    def test_single_scheduling_result_is_inconclusive_without_miss(
        self,
    ) -> None:
        result = {
            "integrity_valid": True,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "valid": True,
                "evidence": {
                    "off": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 0,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                    "binding_on_a": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 0,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                    "binding_on_b": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 0,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        self.assertEqual(
            classify_single_scheduling_result(result),
            "NO_ZERO_WAIT_OVERRUN_INCONCLUSIVE_NO_RETRY",
        )

    def test_single_scheduling_result_prioritizes_integrity_failure(
        self,
    ) -> None:
        result = {
            "integrity_valid": False,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "evidence": {
                    "off": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 1,
                            "spin_observation": {
                                "integrity_failure_count": 1,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        self.assertEqual(
            classify_single_scheduling_result(result),
            "INVALID_NOT_INTERPRETABLE",
        )

    def test_single_scheduling_result_rejects_marker_mismatch_first(
        self,
    ) -> None:
        result = {
            "integrity_valid": False,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "evidence": {
                    "off": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": True,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 1,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        self.assertEqual(
            classify_single_scheduling_result(result),
            "INVALID_NOT_INTERPRETABLE",
        )

    def test_single_scheduling_result_rejects_missing_integrity_count(
        self,
    ) -> None:
        result = {
            "integrity_valid": False,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "evidence": {
                    "off": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 1,
                            "spin_observation": {},
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        self.assertEqual(
            classify_single_scheduling_result(result),
            "INVALID_NOT_INTERPRETABLE",
        )

    def test_single_scheduling_result_rejects_nonprefix_evidence(
        self,
    ) -> None:
        result = {
            "integrity_valid": False,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "evidence": {
                    "binding_on_a": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 1,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        self.assertEqual(
            classify_single_scheduling_result(result),
            "INVALID_NOT_INTERPRETABLE",
        )

    def test_single_scheduling_result_requires_idle_break_marker(
        self,
    ) -> None:
        result = {
            "integrity_valid": False,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "evidence": {
                    "off": {
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 1,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        self.assertEqual(
            classify_single_scheduling_result(result),
            "INVALID_NOT_INTERPRETABLE",
        )
        result["scheduling_evidence"]["evidence"]["off"][
            "idle_break_diagnostic"
        ] = False
        self.assertEqual(
            classify_single_scheduling_result(result),
            "INVALID_NOT_INTERPRETABLE",
        )

    def test_isolated_input_classifier_requires_exact_exclusion_markers(
        self,
    ) -> None:
        isolated_markers = {
            "diagnostic_only": True,
            "not_acceptance": True,
            "m4_wcet_eligible": False,
            "m4_acceptance_credit": False,
            "parent_lease_timeout_sec": 30.0,
            "launcher_diagnostic_baseline": "direct_popen_exec_v1",
            "launcher_timing_parity": False,
        }
        result = {
            "integrity_valid": True,
            "isolated_input_publisher_diagnostic": True,
            "diagnostic_only": True,
            "not_acceptance": True,
            "m4_wcet_eligible": False,
            "m4_acceptance_credit": False,
            "no_retry": True,
            "parent_contact_lease_timeout_sec": 30.0,
            "scheduling_evidence": {
                "valid": True,
                "isolated_input_publisher_diagnostic": True,
                "diagnostic_only": True,
                "not_acceptance": True,
                "m4_wcet_eligible": False,
                "m4_acceptance_credit": False,
                "no_retry": True,
                "parent_contact_lease_timeout_sec": 30.0,
                "evidence": {
                    "off": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": True,
                        "isolated_input_publisher_markers": isolated_markers,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "isolated_observer_drain": {
                            "applicable": True,
                            "diagnostic_only": True,
                            "not_acceptance": True,
                            "m4_wcet_eligible": False,
                            "input_release_authority": (
                                "isolated_input_publisher"
                            ),
                            "input_release_period_sec": 0.01,
                            "input_release_gap_gate_sec": 0.012,
                            "input_release_deadline_applies": False,
                            "observer_processing_in_input_release_deadline": (
                                False
                            ),
                            "canonical_hash_ledger_deferred": False,
                            "saturation_count": 0,
                            "step_count": 1,
                            "max_callbacks_per_step": 64,
                            "max_spin_attempts": 6,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        self.assertEqual(
            classify_single_scheduling_result(result),
            "NO_ZERO_WAIT_OVERRUN_INCONCLUSIVE_NO_RETRY",
        )
        self.assertTrue(
            isolated_publisher_lifecycle_is_closed(
                result["scheduling_evidence"]["evidence"]["off"][
                    "process_lifecycle"
                ][0]
            )
        )
        for container, key in (
            (result, "m4_acceptance_credit"),
            (result, "dds_environment_contract"),
            (result["scheduling_evidence"], "no_retry"),
            (
                result["scheduling_evidence"]["evidence"]["off"][
                    "isolated_input_publisher_markers"
                ],
                "parent_lease_timeout_sec",
            ),
            (
                result["scheduling_evidence"]["evidence"]["off"][
                    "isolated_input_publisher_markers"
                ],
                "launcher_diagnostic_baseline",
            ),
            (
                result["scheduling_evidence"]["evidence"]["off"][
                    "isolated_input_publisher_markers"
                ],
                "launcher_timing_parity",
            ),
        ):
            mismatched = copy.deepcopy(result)
            if container is result:
                mismatched[key] = None
            elif container is result["scheduling_evidence"]:
                mismatched["scheduling_evidence"][key] = None
            else:
                mismatched["scheduling_evidence"]["evidence"]["off"][
                    "isolated_input_publisher_markers"
                ][key] = None
            self.assertEqual(
                classify_single_scheduling_result(mismatched),
                "INVALID_NOT_INTERPRETABLE",
            )
        for field, value in (
            ("fixture_cyclonedds_uri_present", False),
            ("fixture_cyclonedds_uri", "file:///bad.xml"),
            ("fixture_verified_before_arm", False),
            ("isolated_child_cyclonedds_uri", "file:///bad.xml"),
            ("isolated_child_environment_source", "implicit"),
            ("isolated_child_preinit_verified_by_ready_ack", False),
        ):
            mismatched = copy.deepcopy(result)
            mismatched["scheduling_evidence"]["evidence"]["off"][
                "dds_environment_contract"
            ][field] = value
            self.assertEqual(
                classify_single_scheduling_result(mismatched),
                "INVALID_NOT_INTERPRETABLE",
            )
        for field, value in (
            ("saturation_count", 1),
            ("step_count", 0),
            ("input_release_deadline_applies", True),
            ("canonical_hash_ledger_deferred", True),
            ("input_release_gap_gate_sec", 0.013),
            ("max_callbacks_per_step", 63),
            ("max_spin_attempts", 65),
        ):
            mismatched = copy.deepcopy(result)
            mismatched["scheduling_evidence"]["evidence"]["off"][
                "isolated_observer_drain"
            ][field] = value
            self.assertEqual(
                classify_single_scheduling_result(mismatched),
                "INVALID_NOT_INTERPRETABLE",
            )
        mismatched = copy.deepcopy(result)
        mismatched["scheduling_evidence"]["evidence"]["off"][
            "isolated_input_publisher_diagnostic"
        ] = False
        self.assertEqual(
            classify_single_scheduling_result(mismatched),
            "INVALID_NOT_INTERPRETABLE",
        )
        for field, value in (
            ("cleanup_stage", "spawn_observed"),
            ("identity_poststop", {"pid": 1234, "classification": "live"}),
            ("descendants_before_stop", [{"pid": 4321}]),
            ("descendants_poststop", [{"pid": 4321}]),
            ("cleanup_failure", "synthetic cleanup failure"),
        ):
            mismatched = copy.deepcopy(result)
            lifecycle = mismatched["scheduling_evidence"]["evidence"][
                "off"
            ]["process_lifecycle"][0]
            lifecycle["target_process_lifecycle"][field] = value
            self.assertFalse(
                isolated_publisher_lifecycle_is_closed(lifecycle)
            )
            self.assertEqual(
                classify_single_scheduling_result(mismatched),
                "INVALID_NOT_INTERPRETABLE",
            )
        for field, value in (
            ("target_cleanup_failure", "synthetic target failure"),
            ("associated_child_lifecycle", [{"pid": 4321}]),
            (
                "associated_child_cleanup_failure",
                "synthetic associated failure",
            ),
            (
                "associated_child_sample_failures",
                ["synthetic sample failure"],
            ),
        ):
            mismatched = copy.deepcopy(result)
            lifecycle = mismatched["scheduling_evidence"]["evidence"][
                "off"
            ]["process_lifecycle"][0]
            lifecycle[field] = value
            self.assertFalse(
                isolated_publisher_lifecycle_is_closed(lifecycle)
            )
            self.assertEqual(
                classify_single_scheduling_result(mismatched),
                "INVALID_NOT_INTERPRETABLE",
            )

    def test_isolated_phase_ack_is_outside_both_paced_publish_paths(
        self,
    ) -> None:
        callback_active = False
        test_case = self

        class FakePublisher:
            def __init__(self) -> None:
                self.current_ack = {
                    "kind": 1,
                    "sequence": 0,
                    "generation": 0,
                    "tick_index": 0,
                    "publish_complete_watermark": 0,
                    "stamp_ns": FIXED_SEC * 1_000_000_000,
                }

            def assert_alive(self) -> None:
                return

            def phase(self, armed: bool) -> dict[str, object]:
                test_case.assertFalse(callback_active)
                tick = int(self.current_ack["tick_index"]) + 1
                self.current_ack = {
                    "kind": 2,
                    "sequence": int(self.current_ack["sequence"]) + 1,
                    "generation": int(self.current_ack["generation"]) + 1,
                    "tick_index": tick,
                    "publish_complete_watermark": tick,
                    "stamp_ns": (
                        FIXED_SEC * 1_000_000_000 + tick * 10_000_000
                    ),
                    "armed": armed,
                }
                return dict(self.current_ack)

        publisher = FakePublisher()
        armed = False
        for helper_path, desired_armed in (
            ("drive_clock_arm_tick", True),
            ("drive_input_tick", False),
        ):
            armed, ack = synchronize_isolated_input_phase(
                publisher,
                desired_armed,
                armed,
            )
            self.assertIsNotNone(ack, helper_path)

            def paced_publish() -> None:
                nonlocal callback_active
                callback_active = True
                try:
                    self.assertEqual(armed, desired_armed)
                finally:
                    callback_active = False

            drive_paced_fixture_tick(
                paced_publish,
                lambda _: {
                    "callback_generation_delta": 0,
                    "wall_elapsed_ns": 0,
                    "thread_cpu_elapsed_ns": 0,
                },
                max_callbacks=1,
                monotonic=lambda: 0.0,
                sleep=lambda _: None,
            )
            validate_post_ack_exact_cohort(
                int(ack["stamp_ns"]) + 10_000_000,
                ack,
            )
            with self.assertRaisesRegex(
                AssertionError,
                "cohort_stamp_invalid",
            ):
                validate_post_ack_exact_cohort(
                    int(ack["stamp_ns"]),
                    ack,
                )

        syntax = ast.parse(inspect.getsource(run_case))
        nested = {
            node.name: node
            for node in ast.walk(syntax)
            if isinstance(node, ast.FunctionDef)
        }
        publish_calls = [
            node
            for node in ast.walk(nested["publish_clock_and_arm"])
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "phase"
        ]
        self.assertEqual(publish_calls, [])
        for helper_name in ("drive_input_tick", "drive_clock_arm_tick"):
            calls = [
                node
                for node in ast.walk(nested[helper_name])
                if isinstance(node, ast.Call)
            ]
            names = [
                (
                    call.func.id
                    if isinstance(call.func, ast.Name)
                    else call.func.attr
                    if isinstance(call.func, ast.Attribute)
                    else ""
                )
                for call in calls
            ]
            self.assertIn("synchronize_isolated_phase", names)
            self.assertIn("drive_paced_fixture_tick", names)
            self.assertIn("drive_isolated_observer_drain_step", names)
            self.assertLess(
                min(
                    call.lineno
                    for call, name in zip(calls, names)
                    if name == "synchronize_isolated_phase"
                ),
                min(
                    call.lineno
                    for call, name in zip(calls, names)
                    if name
                    in (
                        "drive_paced_fixture_tick",
                        "drive_isolated_observer_drain_step",
                    )
                ),
            )

    def test_single_classifier_requires_exact_zero_wait_terminal(self) -> None:
        base = {
            "integrity_valid": False,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "evidence": {
                    "off": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 1,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(base)
        self.assertEqual(
            classify_single_scheduling_result(base),
            "ZERO_WAIT_OVERRUN_REPRODUCED_DIAGNOSTIC_STOP",
        )
        cases = {
            "publish_phase": lambda value: value[
                "scheduling_evidence"
            ]["evidence"]["off"]["paced_callback_drain"][
                "tick_observation"
            ]["last"].update({"fatal_phase": "publish"}),
            "spin_zero": lambda value: value[
                "scheduling_evidence"
            ]["evidence"]["off"]["paced_callback_drain"][
                "tick_observation"
            ]["last"].update({"spin_attempts": 0}),
            "integrity_invalid": lambda value: value[
                "scheduling_evidence"
            ]["evidence"]["off"]["paced_callback_drain"][
                "spin_observation"
            ].update({"integrity_failure_count": 1}),
            "evidence_invalid": lambda value: value[
                "scheduling_evidence"
            ]["evidence"]["off"].update(
                {"scenario_integrity_valid": False}
            ),
            "residue_invalid": lambda value: value.update(
                {
                    "fixture_session_residue": [4321],
                    "fixture_session_reconciliation": {"valid": False},
                }
            ),
        }
        for name, mutate in cases.items():
            with self.subTest(name=name):
                candidate = copy.deepcopy(base)
                mutate(candidate)
                self.assertEqual(
                    classify_single_scheduling_result(candidate),
                    "INVALID_NOT_INTERPRETABLE",
                )

    def test_classifier_rejects_early_zero_wait_collection_not_reached(
        self,
    ) -> None:
        result = {
            "integrity_valid": False,
            "isolated_input_publisher_diagnostic": False,
            "scheduling_evidence": {
                "evidence": {
                    "off": {
                        "idle_break_diagnostic": True,
                        "isolated_input_publisher_diagnostic": False,
                        "diagnostic_only": True,
                        "not_acceptance": True,
                        "m4_wcet_eligible": False,
                        "no_retry": True,
                        "paced_callback_drain": {
                            "deadline_miss_count": 1,
                            "spin_observation": {
                                "integrity_failure_count": 0,
                            },
                        },
                    },
                },
            },
        }
        self.complete_single_result_contract(result)
        result["scheduling_evidence"]["evidence"]["off"][
            "collection_status"
        ] = {
            "process_snapshot": {
                "status": "not_reached",
                "reason": "primary_first_false_before_completion",
            },
            "reanchor": {
                "status": "not_reached",
                "reason": "primary_first_false_before_completion",
            },
        }
        self.assertEqual(
            classify_single_scheduling_result(result),
            "INVALID_NOT_INTERPRETABLE",
        )

    def test_early_zero_wait_preserves_first_false_without_fake_collection(
        self,
    ) -> None:
        evidence = initial_scheduling_evidence(
            "off",
            "SHARED_CONTROL_0_3",
        )
        evidence.update(
            {
                "primary_first_false": primary_first_false_record(
                    AssertionError("early zero wait"),
                    {
                        "tick_observation": {
                            "last": {
                                "fatal_phase": "zero_wait",
                                "spin_attempts": 2,
                            }
                        }
                    },
                ),
                "idle_break_diagnostic": True,
                "isolated_input_publisher_diagnostic": True,
                "isolated_input_publisher_markers": {
                    "diagnostic_only": True,
                    "not_acceptance": True,
                    "m4_wcet_eligible": False,
                    "m4_acceptance_credit": False,
                    "parent_lease_timeout_sec": 30.0,
                    "launcher_diagnostic_baseline": "direct_popen_exec_v1",
                    "launcher_timing_parity": False,
                },
                "diagnostic_only": True,
                "not_acceptance": True,
                "m4_wcet_eligible": False,
                "no_retry": True,
                "scenario_integrity_valid": False,
                "residue_pids": [],
                "process_lifecycle": [
                    {
                        "pid": 3001,
                        "role": "isolated_input_publisher",
                        "provenance": "measurement_spawn",
                        "cleanup_stage": "closed_no_residue",
                        "alive": False,
                        "exitcode": 0,
                        "unresolved": False,
                        "identity_before_cleanup": {
                            "pid": 3001,
                            "readable": True,
                            "classification": "live",
                            "state": "S",
                            "ppid": 3000,
                            "session_id": 3000,
                            "starttime_ticks": 10,
                            "cmdline": "publisher",
                            "comm": "python3",
                            "cgroup": ["0::/fixture"],
                        },
                    }
                ],
            }
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "off.json").write_text(
                json.dumps(evidence)
            )
            inspected = inspect_scheduling_evidence(
                root,
                "SHARED_CONTROL_0_3",
                {0, 1, 2, 3},
                {0, 1, 2, 3},
                True,
                True,
            )
        errors = " ".join(inspected["errors"])
        self.assertFalse(inspected["valid"])
        self.assertEqual(
            evidence["primary_first_false"]["fatal_phase"],
            "zero_wait",
        )
        self.assertIn("process evidence collection not reached", errors)
        self.assertIn("reanchor collection not reached", errors)
        self.assertNotIn("CPU request mismatch", errors)
        self.assertNotIn("post-ACK", errors)

    def test_fixture_session_pid_reconciliation_is_fail_closed(self) -> None:
        identity = {
            "pid": 2002,
            "readable": True,
            "classification": "live",
            "state": "S",
            "ppid": 2001,
            "session_id": 2001,
            "starttime_ticks": 123,
            "cmdline": "isolated publisher",
            "comm": "python3",
            "cgroup": ["0::/fixture"],
        }
        lifecycle = {
            "pid": 2002,
            "role": "isolated_input_publisher",
            "provenance": "measurement_spawn_isolated_input_publisher",
            "cleanup_stage": "closed_no_residue",
            "alive": False,
            "exitcode": 0,
            "unresolved": False,
            "identity_before_cleanup": identity,
        }
        evidence = {
            "evidence": {
                "off": {"process_lifecycle": [lifecycle]},
            }
        }
        reconciled = reconcile_fixture_session_processes(
            fixture_pid=2001,
            fixture_exit_code=0,
            fixture_timed_out=False,
            fixture_session_residue=[],
            fixture_session_residue_snapshots=[],
            scheduling_evidence=evidence,
        )
        self.assertTrue(reconciled["valid"])
        self.assertEqual(
            {
                record["pid"]: record["role"]
                for record in reconciled["known_processes"]
            },
            {
                2001: "fixture_session_leader",
                2002: "isolated_input_publisher",
            },
        )
        for classification, mutate_identity, role_is_known in (
            ("live", {}, True),
            ("zombie", {"state": "Z"}, True),
            (
                "unreadable",
                {"readable": False, "starttime_ticks": None},
                False,
            ),
            ("unknown", {"state": "?", "classification": "unknown"}, False),
            (
                "identity_changed",
                {"starttime_ticks": 124},
                False,
            ),
        ):
            with self.subTest(classification=classification):
                unresolved_lifecycle = copy.deepcopy(lifecycle)
                unresolved_lifecycle.update(
                    {
                        "cleanup_stage": "close_failed",
                        "alive": True,
                        "exitcode": None,
                        "unresolved": True,
                    }
                )
                residue_identity = {
                    **identity,
                    **mutate_identity,
                    "classification": classification,
                }
                unresolved = reconcile_fixture_session_processes(
                    fixture_pid=2001,
                    fixture_exit_code=1,
                    fixture_timed_out=True,
                    fixture_session_residue=[2002],
                    fixture_session_residue_snapshots=[
                        residue_identity
                    ],
                    scheduling_evidence={
                        "evidence": {
                            "off": {
                                "process_lifecycle": [
                                    unresolved_lifecycle
                                ],
                            },
                        }
                    },
                )
                self.assertFalse(unresolved["valid"])
                errors = " ".join(unresolved["errors"])
                self.assertEqual(
                    "role=isolated_input_publisher" in errors,
                    role_is_known,
                )
                self.assertEqual(
                    "role unresolved" in errors,
                    not role_is_known,
                )

    def test_runner_preserves_unclassified_spawn_child_identity(
        self,
    ) -> None:
        identity = {
            "pid": 2071,
            "readable": True,
            "classification": "zombie",
            "state": "Z",
            "ppid": 2001,
            "session_id": 2001,
            "starttime_ticks": 7100,
            "cmdline": "",
            "comm": "python3",
            "cgroup": ["0::/fixture"],
        }
        lifecycle = {
            "pid": 2071,
            "role": "isolated_spawn_associated_child_unclassified",
            "provenance": (
                "direct_child_delta_around_multiprocessing_spawn"
            ),
            "cleanup_stage": "zombie_not_reaped",
            "alive": True,
            "exitcode": None,
            "unresolved": True,
            "identity_before_cleanup": identity,
        }
        zombie = reconcile_fixture_session_processes(
            fixture_pid=2001,
            fixture_exit_code=1,
            fixture_timed_out=False,
            fixture_session_residue=[2071],
            fixture_session_residue_snapshots=[identity],
            scheduling_evidence={
                "evidence": {
                    "off": {"process_lifecycle": [lifecycle]},
                }
            },
        )
        self.assertFalse(zombie["valid"])
        known = {
            record["pid"]: record
            for record in zombie["known_processes"]
        }
        self.assertEqual(
            known[2071]["role"],
            "isolated_spawn_associated_child_unclassified",
        )
        self.assertEqual(
            known[2071]["provenance"],
            "direct_child_delta_around_multiprocessing_spawn",
        )
        self.assertIn(
            "role=isolated_spawn_associated_child_unclassified",
            " ".join(zombie["errors"]),
        )

        absent_lifecycle = {
            **lifecycle,
            "cleanup_stage": "absent_no_residue",
            "alive": False,
            "unresolved": False,
        }
        absent = reconcile_fixture_session_processes(
            fixture_pid=2001,
            fixture_exit_code=0,
            fixture_timed_out=False,
            fixture_session_residue=[],
            fixture_session_residue_snapshots=[],
            scheduling_evidence={
                "evidence": {
                    "off": {
                        "process_lifecycle": [absent_lifecycle]
                    },
                }
            },
        )
        self.assertTrue(absent["valid"])
        self.assertEqual(
            {
                record["pid"]: record["role"]
                for record in absent["known_processes"]
            }[2071],
            "isolated_spawn_associated_child_unclassified",
        )
        late_residue = reconcile_fixture_session_processes(
            fixture_pid=2001,
            fixture_exit_code=0,
            fixture_timed_out=False,
            fixture_session_residue=[2071],
            fixture_session_residue_snapshots=[identity],
            scheduling_evidence={
                "evidence": {
                    "off": {
                        "process_lifecycle": [absent_lifecycle]
                    },
                }
            },
        )
        self.assertFalse(late_residue["valid"])
        self.assertIn(
            "session residue role unresolved",
            " ".join(late_residue["errors"]),
        )
        self.assertEqual(
            {
                record["pid"]: record["role"]
                for record in late_residue["known_processes"]
            }[2071],
            "isolated_spawn_associated_child_unclassified",
        )
        run_case_source = inspect.getsource(run_case)
        self.assertIn(
            "process_lifecycle.extend(\n"
            "                isolated_associated_child_lifecycle\n"
            "            )",
            run_case_source,
        )
        self.assertIn(
            'isolated_publisher_lifecycle["cleanup_failure"]',
            run_case_source,
        )
        self.assertIn(
            '"associated_child_sample_failures"',
            run_case_source,
        )

    def test_session_identity_snapshot_detects_state_and_reuse(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            proc_root = Path(temporary)
            process_root = proc_root / "2002"
            process_root.mkdir()

            def write_stat(state: str, starttime_ticks: int) -> None:
                fields = [
                    state,
                    "1",
                    "2002",
                    "2001",
                    *(["0"] * 15),
                    str(starttime_ticks),
                ]
                (process_root / "stat").write_text(
                    f"2002 (helper proc) {' '.join(fields)}\n"
                )

            write_stat("S", 123)
            (process_root / "cmdline").write_bytes(
                b"python3\0helper.py\0"
            )
            (process_root / "cgroup").write_text("0::/fixture\n")
            live = process_identity_snapshot(
                2002,
                proc_root=proc_root,
            )
            self.assertEqual(live["classification"], "live")
            self.assertEqual(live["session_id"], 2001)
            self.assertEqual(live["starttime_ticks"], 123)
            self.assertEqual(live["cmdline"], "python3 helper.py")
            self.assertEqual(live["cgroup"], ["0::/fixture"])

            write_stat("Z", 123)
            zombie = process_identity_snapshot(
                2002,
                proc_root=proc_root,
            )
            self.assertEqual(zombie["classification"], "zombie")

            write_stat("?", 123)
            unknown = process_identity_snapshot(
                2002,
                proc_root=proc_root,
            )
            self.assertEqual(unknown["classification"], "unknown")

            write_stat("S", 124)
            changed = session_process_snapshots(
                2001,
                known_identities={2002: live},
                proc_root=proc_root,
            )
            self.assertEqual(changed[0]["classification"], "identity_changed")

            (process_root / "stat").write_text("unreadable-stat\n")
            unreadable = session_process_snapshots(
                2001,
                known_identities={2002: live},
                proc_root=proc_root,
            )
            self.assertEqual(unreadable[0]["classification"], "unreadable")

    def test_isolated_input_cli_is_single_scheduling_only(self) -> None:
        with patch(
            "sys.argv",
            [
                "run_c002ay0_formal_wcet.py",
                "--artifact-root",
                "/tmp/not-created-by-parser-test",
                "--isolated-input-publisher-diagnostic",
            ],
        ):
            parsed = parse_args()
        self.assertTrue(parsed.isolated_input_publisher_diagnostic)
        with self.assertRaisesRegex(
            ValueError,
            "--scheduling-diagnostic",
        ):
            validate_isolated_input_publisher_cli(
                enabled=parsed.isolated_input_publisher_diagnostic,
                scheduling_diagnostic=parsed.scheduling_diagnostic,
                single_shared_control_off_domain=(
                    parsed.single_shared_control_off_domain
                ),
            )
        with self.assertRaisesRegex(
            ValueError,
            "--single-shared-control-off-domain",
        ):
            validate_isolated_input_publisher_cli(
                enabled=True,
                scheduling_diagnostic=True,
                single_shared_control_off_domain=None,
            )
        validate_isolated_input_publisher_cli(
            enabled=True,
            scheduling_diagnostic=True,
            single_shared_control_off_domain=229,
        )
        validate_isolated_input_publisher_cli(
            enabled=False,
            scheduling_diagnostic=True,
            single_shared_control_off_domain=None,
        )

    def test_isolated_input_environment_is_exact_arm_only(self) -> None:
        inherited = {
            "C002AY0_ISOLATED_INPUT_PUBLISHER_DIAGNOSTIC": "1",
            "OTHER": "x",
        }
        normal_or_fixed_four = (
            configure_isolated_input_publisher_environment(
                inherited,
                False,
            )
        )
        exact_single_arm = configure_isolated_input_publisher_environment(
            inherited,
            True,
        )
        self.assertNotIn(
            "C002AY0_ISOLATED_INPUT_PUBLISHER_DIAGNOSTIC",
            normal_or_fixed_four,
        )
        self.assertEqual(
            exact_single_arm[
                "C002AY0_ISOLATED_INPUT_PUBLISHER_DIAGNOSTIC"
            ],
            "1",
        )
        self.assertEqual(normal_or_fixed_four["OTHER"], "x")
        self.assertEqual(exact_single_arm["OTHER"], "x")

    def test_formal_dds_environment_overwrites_host_and_is_fail_closed(
        self,
    ) -> None:
        configured = configure_formal_dds_environment(
            {
                "CYCLONEDDS_URI": "file:///opt/autoware/cyclonedds.xml",
                "OTHER": "preserved",
            }
        )
        self.assertIn("CYCLONEDDS_URI", configured)
        self.assertEqual(configured["CYCLONEDDS_URI"], "")
        self.assertEqual(configured["OTHER"], "preserved")
        require_formal_dds_environment(configured)
        for invalid in ({}, {"CYCLONEDDS_URI": "file:///bad.xml"}):
            with self.assertRaisesRegex(
                RuntimeError,
                "must be explicit empty",
            ):
                require_formal_dds_environment(invalid)

    def test_idle_break_environment_is_single_arm_only(self) -> None:
        inherited = {"C002AY0_IDLE_BREAK_DIAGNOSTIC": "1", "OTHER": "x"}
        fixed_four = configure_idle_break_diagnostic_environment(
            inherited,
            False,
        )
        single_arm = configure_idle_break_diagnostic_environment(
            inherited,
            True,
        )
        self.assertNotIn(
            "C002AY0_IDLE_BREAK_DIAGNOSTIC",
            fixed_four,
        )
        self.assertEqual(
            single_arm["C002AY0_IDLE_BREAK_DIAGNOSTIC"],
            "1",
        )
        self.assertEqual(fixed_four["OTHER"], "x")
        self.assertEqual(single_arm["OTHER"], "x")

    def test_idle_break_runner_mode_markers_are_fail_closed(self) -> None:
        self.assertEqual(
            diagnostic_mode_markers(True, True),
            {
                "diagnostic_only": True,
                "not_acceptance": True,
                "m4_wcet_eligible": False,
                "no_retry": True,
                "idle_break_diagnostic": True,
            },
        )
        self.assertFalse(
            diagnostic_mode_markers(True, False)[
                "idle_break_diagnostic"
            ]
        )
        self.assertFalse(
            diagnostic_mode_markers(False, False)[
                "idle_break_diagnostic"
            ]
        )
        with self.assertRaisesRegex(ValueError, "diagnostic-only"):
            diagnostic_mode_markers(False, True)

    def test_single_scheduling_domain_validation_is_fail_closed(
        self,
    ) -> None:
        self.assertEqual(validate_domains((229,), expected_count=1), (229,))
        with self.assertRaisesRegex(ValueError, "reserved CTest domains"):
            validate_domains((223,), expected_count=1)

    def test_fresh_reanchor_observation_exposes_topic_high_watermarks(
        self,
    ) -> None:
        evidence_stamp_ns = 10_000_000
        self.stage(
            self.cohort(evidence_stamp_ns + 1_000_000, 5),
            startup=False,
        )
        summary = self.capture.fresh_reanchor_observation_summary(
            5,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )
        self.assertEqual(summary["fresh_stamp_count"], 1)
        self.assertEqual(summary["exact_complete_count"], 1)
        self.assertEqual(summary["eligible_exact_complete_count"], 1)
        self.assertEqual(summary["incomplete_stamp_count"], 0)
        self.assertEqual(summary["duplicate_topic_count"], 0)
        self.assertEqual(summary["minimum_observed_sequence"], 5)
        self.assertEqual(summary["maximum_observed_sequence"], 5)
        self.assertEqual(
            set(summary["topic_record_counts"].values()),
            {1},
        )
        self.assertEqual(
            set(summary["topic_max_stamp_ns"].values()),
            {evidence_stamp_ns + 1_000_000},
        )

    def test_alignment_stage_capacity_plus_one_hard_fails(self) -> None:
        template = self.cohort(1, 1)["command"]
        for index in range(ALIGNMENT_STAGE_MAX_MESSAGES):
            record = dict(template)
            record["identity"] = (
                index + 1,
                *template["identity"][1:],
            )
            self.capture._stage_alignment_sample(record)
        self.assertEqual(
            self.capture.alignment_stage_message_count,
            ALIGNMENT_STAGE_MAX_MESSAGES,
        )
        self.assertEqual(
            self.capture.alignment_stage_max_message_count,
            ALIGNMENT_STAGE_MAX_MESSAGES,
        )
        overflow = dict(template)
        overflow["identity"] = (
            ALIGNMENT_STAGE_MAX_MESSAGES + 1,
            *template["identity"][1:],
        )
        with self.assertRaisesRegex(
            AssertionError,
            "alignment stage overflow",
        ):
            self.capture._stage_alignment_sample(overflow)
        self.assertTrue(self.capture.alignment_stage_overflow)
        self.assertFalse(self.capture.enabled)
        summary = self.capture.startup_alignment_summary()
        self.assertTrue(summary["alignment_stage_overflow"])
        self.assertEqual(
            summary["alignment_stage_max_message_count"],
            ALIGNMENT_STAGE_MAX_MESSAGES,
        )
        self.assertEqual(
            summary["alignment_stage_pending_message_count"],
            ALIGNMENT_STAGE_MAX_MESSAGES,
        )

    def test_scheduling_evidence_elapsed_gate_is_fail_closed(self) -> None:
        require_scheduling_evidence_elapsed(SCHEDULING_EVIDENCE_MAX_SEC)
        with self.assertRaisesRegex(
            AssertionError,
            "scheduling evidence exceeded",
        ):
            require_scheduling_evidence_elapsed(
                SCHEDULING_EVIDENCE_MAX_SEC + 0.000_001
            )
        with self.assertRaisesRegex(
            AssertionError,
            "scheduling evidence exceeded",
        ):
            require_scheduling_evidence_elapsed(float("nan"))
        with self.assertRaisesRegex(
            AssertionError,
            "scheduling evidence exceeded",
        ):
            require_scheduling_evidence_elapsed(-0.000_001)

    def test_startup_ledger_capacity_plus_one_blocks_arm(self) -> None:
        record = self.cohort(1, 1)["command"]
        for _ in range(STARTUP_LEDGER_MAX_MESSAGES + 1):
            self.capture._record_startup_sample(record)
        self.assertEqual(
            len(self.capture.startup_ledger),
            STARTUP_LEDGER_MAX_MESSAGES,
        )
        self.assertTrue(self.capture.startup_ledger_overflow)
        summary = self.capture.startup_alignment_summary()
        self.assertTrue(summary["ledger_overflow"])
        self.assertEqual(
            summary["ledger_max_message_count"],
            STARTUP_LEDGER_MAX_MESSAGES,
        )
        with self.assertRaisesRegex(
            AssertionError,
            "startup alignment ledger overflow",
        ):
            self.capture.arm_after_completed_cycle(1)
        self.assertFalse(self.capture.enabled)

    def test_latest_exact_prearm_candidate_is_the_anchor(self) -> None:
        evidence_stamp_ns = 200_000_000_000
        self.stage(self.cohort(evidence_stamp_ns, 20))
        self.stage(self.cohort(evidence_stamp_ns + 1_000_000, 21))
        self.stage(self.cohort(evidence_stamp_ns + 2_000_000, 22))

        anchor = self.capture.arm_after_completed_cycle(
            21,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )
        self.assertEqual(
            anchor,
            (evidence_stamp_ns + 2_000_000, 22, 42),
        )
        self.assertEqual(self.capture.next_expected_sequence, 23)

    def test_fresh_partial_cohort_remains_pending(self) -> None:
        evidence_stamp_ns = 250_000_000_000
        partial = self.cohort(evidence_stamp_ns + 1_000_000, 26)
        del partial["execution_envelope"]
        self.stage(partial)

        self.capture.validate_fresh_reanchor_interval(
            26,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )
        self.assertFalse(
            self.capture.has_completed_cycle(
                26,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=42,
            )
        )
        self.assertFalse(self.capture.enabled)

    def test_fresh_producer_mismatch_is_a_hard_failure(self) -> None:
        evidence_stamp_ns = 260_000_000_000
        self.stage(
            self.cohort(
                evidence_stamp_ns + 1_000_000,
                27,
                producer_instance_id=99,
            )
        )

        with self.assertRaisesRegex(
            AssertionError,
            "producer_mismatch",
        ):
            self.capture.validate_fresh_reanchor_interval(
                27,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=42,
            )
        self.assertFalse(self.capture.enabled)
        self.assertIsNone(self.capture.startup_anchor_sequence)

    def test_fresh_sequence_regression_is_a_hard_failure(self) -> None:
        evidence_stamp_ns = 265_000_000_000
        self.stage(self.cohort(evidence_stamp_ns + 1_000_000, 26))

        with self.assertRaisesRegex(
            AssertionError,
            "sequence_regression",
        ):
            self.capture.validate_fresh_reanchor_interval(
                27,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=42,
            )
        self.assertFalse(self.capture.enabled)
        self.assertIsNone(self.capture.startup_anchor_sequence)

    def test_fresh_duplicate_mirror_without_envelope_remains_pending(
        self,
    ) -> None:
        evidence_stamp_ns = 270_000_000_000
        cohort = self.cohort(evidence_stamp_ns + 1_000_000, 28)
        self.stage(cohort)
        self.capture._stage_alignment_sample(cohort["command"])

        self.capture.validate_fresh_reanchor_interval(
            28,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )
        self.assertFalse(
            self.capture.has_completed_cycle(
                28,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=42,
            )
        )
        self.assertFalse(self.capture.enabled)
        self.assertIsNone(self.capture.startup_anchor_sequence)

    def test_fresh_reanchor_accepts_equal_stamp_distinct_sequences(
        self,
    ) -> None:
        evidence_stamp_ns = 275_000_000_000
        shared_stamp_ns = evidence_stamp_ns + 1_000_000
        self.stage(self.cohort(shared_stamp_ns, 31))
        self.stage(self.cohort(shared_stamp_ns, 32))

        self.capture.validate_fresh_reanchor_interval(
            31,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )
        anchor = self.capture.arm_after_completed_cycle(
            31,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )

        self.assertEqual(anchor, (shared_stamp_ns, 32, 42))
        self.assertEqual(self.capture.next_expected_sequence, 33)

    def test_equal_stamp_fresh_cycles_cannot_reuse_one_mirror_set(
        self,
    ) -> None:
        evidence_stamp_ns = 277_000_000_000
        shared_stamp_ns = evidence_stamp_ns + 1_000_000
        first = self.cohort(shared_stamp_ns, 33)
        second = self.cohort(shared_stamp_ns, 34)
        self.stage(first)
        self.stage(
            {
                "command_envelope": second["command_envelope"],
                "execution_envelope": second["execution_envelope"],
            }
        )

        self.capture.validate_fresh_reanchor_interval(
            33,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )
        self.assertFalse(
            self.capture.has_completed_cycle(
                33,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=42,
            )
        )

    def test_fresh_identity_mismatch_is_a_hard_failure(self) -> None:
        evidence_stamp_ns = 280_000_000_000
        cohort = self.cohort(evidence_stamp_ns + 1_000_000, 29)
        identity = cohort["execution_envelope"]["identity"]
        cohort["execution_envelope"]["identity"] = (
            *identity[:3],
            int(identity[3]) + 1,
            *identity[4:],
        )
        self.stage(cohort)

        with self.assertRaisesRegex(
            AssertionError,
            "cohort_identity_mismatch",
        ):
            self.capture.validate_fresh_reanchor_interval(
                29,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=42,
            )
        self.assertFalse(self.capture.enabled)
        self.assertIsNone(self.capture.startup_anchor_sequence)

    def test_fresh_semantic_mismatch_is_a_hard_failure(self) -> None:
        evidence_stamp_ns = 290_000_000_000
        cohort = self.cohort(evidence_stamp_ns + 1_000_000, 30)
        cohort["execution_envelope"]["semantic"] = {
            "command_envelope": {"reason": "not-the-command-envelope"}
        }
        self.stage(cohort)

        with self.assertRaisesRegex(AssertionError, "semantic_mismatch"):
            self.capture.validate_fresh_reanchor_interval(
                30,
                minimum_stamp_exclusive_ns=evidence_stamp_ns,
                expected_producer_instance_id=42,
            )
        self.assertFalse(self.capture.enabled)
        self.assertIsNone(self.capture.startup_anchor_sequence)

    def test_post_arm_repeated_ackermann_is_a_non_authoritative_mirror(
        self,
    ) -> None:
        evidence_stamp_ns = 300_000_000_000
        self.stage(self.cohort(evidence_stamp_ns, 30))
        self.stage(self.cohort(evidence_stamp_ns + 1_000_000, 31))
        self.capture.arm_after_completed_cycle(
            31,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )

        duplicate = self.cohort(
            evidence_stamp_ns + 2_000_000,
            32,
        )["command"]
        self.capture._stage_alignment_sample(duplicate)
        self.capture._stage_alignment_sample(duplicate)
        self.assertEqual(
            len(
                self.capture.alignment_cycles[
                    evidence_stamp_ns + 2_000_000
                ]["command"]
            ),
            2,
        )

    def test_post_arm_duplicate_envelope_identity_remains_hard_failure(
        self,
    ) -> None:
        evidence_stamp_ns = 310_000_000_000
        self.stage(self.cohort(evidence_stamp_ns, 30))
        self.stage(self.cohort(evidence_stamp_ns + 1_000_000, 31))
        self.capture.arm_after_completed_cycle(
            31,
            minimum_stamp_exclusive_ns=evidence_stamp_ns,
            expected_producer_instance_id=42,
        )

        duplicate = self.cohort(
            evidence_stamp_ns + 2_000_000,
            32,
        )["command_envelope"]
        self.capture._stage_alignment_sample(duplicate)
        with self.assertRaisesRegex(
            AssertionError,
            "duplicate identity-bearing sample",
        ):
            self.capture._stage_alignment_sample(duplicate)

    def test_equal_stamp_consecutive_envelope_sequences_materialize(
        self,
    ) -> None:
        evidence_stamp_ns = 320_000_000_000
        self.stage(self.cohort(evidence_stamp_ns, 30))
        self.capture.arm_after_completed_cycle(
            30,
            minimum_stamp_exclusive_ns=evidence_stamp_ns - 1,
            expected_producer_instance_id=42,
        )

        self.stage(
            self.cohort(evidence_stamp_ns + 1_000_000, 31),
            startup=False,
        )
        self.stage(
            self.cohort(evidence_stamp_ns + 1_000_000, 32),
            startup=False,
        )

        self.assertEqual(self.capture.materialized_sequences, [31, 32])

    def test_equal_stamp_envelopes_cannot_reuse_one_mirror_set(
        self,
    ) -> None:
        evidence_stamp_ns = 330_000_000_000
        self.stage(self.cohort(evidence_stamp_ns, 30))
        self.capture.arm_after_completed_cycle(
            30,
            minimum_stamp_exclusive_ns=evidence_stamp_ns - 1,
            expected_producer_instance_id=42,
        )
        stamp_ns = evidence_stamp_ns + 1_000_000
        first = self.cohort(stamp_ns, 31)
        second = self.cohort(stamp_ns, 32)
        for topic in ("command", "raw_command", "tracking"):
            self.capture._stage_alignment_sample(first[topic])
        for topic in ("command_envelope", "execution_envelope"):
            self.capture._stage_alignment_sample(first[topic])
            self.capture._stage_alignment_sample(second[topic])

        self.capture._materialize_ready_cycles()
        self.assertEqual(self.capture.materialized_sequences, [31])
        self.assertIsNone(
            self.capture._select_expected_cycle_records(
                self.capture.alignment_cycles[stamp_ns],
                32,
            )
        )

    def test_execution_envelope_requires_full_cycle_identity(self) -> None:
        evidence_stamp_ns = 340_000_000_000
        self.stage(self.cohort(evidence_stamp_ns, 30))
        self.capture.arm_after_completed_cycle(
            30,
            minimum_stamp_exclusive_ns=evidence_stamp_ns - 1,
            expected_producer_instance_id=42,
        )
        cohort = self.cohort(evidence_stamp_ns + 1_000_000, 31)
        execution_identity = list(
            cohort["execution_envelope"]["identity"]
        )
        execution_identity[4] = 99
        cohort["execution_envelope"]["identity"] = tuple(
            execution_identity
        )

        with self.assertRaisesRegex(
            AssertionError,
            "execution envelope full identity mismatch",
        ):
            self.stage(cohort, startup=False)

    def test_prearm_execution_identity_mismatch_cannot_anchor(self) -> None:
        evidence_stamp_ns = 345_000_000_000
        cohort = self.cohort(evidence_stamp_ns, 30)
        execution_identity = list(
            cohort["execution_envelope"]["identity"]
        )
        execution_identity[6] = 99
        cohort["execution_envelope"]["identity"] = tuple(
            execution_identity
        )
        self.stage(cohort)

        self.assertFalse(
            self.capture.has_completed_cycle(
                30,
                minimum_stamp_exclusive_ns=evidence_stamp_ns - 1,
                expected_producer_instance_id=42,
            )
        )
        with self.assertRaisesRegex(
            AssertionError,
            "startup completed output cohort unavailable",
        ):
            self.capture.arm_after_completed_cycle(
                30,
                minimum_stamp_exclusive_ns=evidence_stamp_ns - 1,
                expected_producer_instance_id=42,
            )

    def test_raw_command_must_match_selected_command_digest(self) -> None:
        evidence_stamp_ns = 350_000_000_000
        self.stage(self.cohort(evidence_stamp_ns, 30))
        self.capture.arm_after_completed_cycle(
            30,
            minimum_stamp_exclusive_ns=evidence_stamp_ns - 1,
            expected_producer_instance_id=42,
        )
        cohort = self.cohort(evidence_stamp_ns + 1_000_000, 31)
        cohort["raw_command"]["semantic"] = {
            "longitudinal": {
                "speed": 9.0,
                "acceleration": 9.0,
            }
        }

        self.stage(cohort, startup=False)
        self.assertEqual(self.capture.materialized_sequences, [])

    def test_mixed_valid_and_divergent_raw_mirrors_fail(self) -> None:
        evidence_stamp_ns = 360_000_000_000
        self.stage(self.cohort(evidence_stamp_ns, 30))
        self.capture.arm_after_completed_cycle(
            30,
            minimum_stamp_exclusive_ns=evidence_stamp_ns - 1,
            expected_producer_instance_id=42,
        )
        cohort = self.cohort(evidence_stamp_ns + 1_000_000, 31)
        for record in cohort.values():
            self.capture._stage_alignment_sample(record)
        divergent_raw = dict(cohort["raw_command"])
        divergent_raw["semantic"] = {
            "longitudinal": {
                "speed": 9.0,
                "acceleration": 9.0,
            }
        }
        self.capture._stage_alignment_sample(divergent_raw)

        with self.assertRaisesRegex(
            AssertionError,
            "unmatched mirror variant",
        ):
            self.capture._materialize_ready_cycles()


if __name__ == "__main__":
    unittest.main()
