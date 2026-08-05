from __future__ import annotations

import ast
import hashlib
import json
import errno
import os
import shlex
import stat
import subprocess
import sys
import time
from pathlib import Path

import pytest

from aic_test import cli
from aic_test.cli import (
    _read_host_readable_json,
    _write_host_readable_json_atomic,
    build_parser,
    private_prelaunch_start_decision,
    private_prelaunch_start_files_valid,
)
from aic_test.model import OUTCOMES, read_result, write_json_atomic
from aic_test.observer import FORBIDDEN_AUTHORITY_TOPICS
from aic_test.v2_direct_input_driver import (
    INPUT_PERIODS_S,
    _due_input_channels,
    _input_stamp_ns,
    _validate_clock_step,
    _process_is_alive,
    _schedule_due,
    _write_json_atomic,
)


def test_cli_exposes_control_smoke() -> None:
    args = build_parser().parse_args(["run", "control-smoke"])
    assert args.scenario == "control-smoke"
    assert args.observe_duration == 20.0


def test_cli_exposes_read_only_safegate2_analysis() -> None:
    args = build_parser().parse_args(["analyze", "safegate2-stopped-overtake", "run-1"])
    assert args.run_id == "run-1"


def _bounded_deadline_chain_payload(retained_count: int = 11) -> dict[str, object]:
    return {
        "stream_state": {
            "retained_s0_window_count": retained_count,
            "retained_identity_count": 2,
            "capacity": 8,
            "active_cycle_present": False,
            "pending_cycle_sequence": None,
            "pending_cycle_event_count": 0,
            "capacity_overflowed": False,
        },
        "evidence": {"availability_cycles": [{"pp_cycle_sequence": 1}]},
    }


def test_observer_bounded_deadline_chain_accepts_non_ten_valid_window() -> None:
    assert cli._observer_bounded_deadline_chain_valid(
        _bounded_deadline_chain_payload(11)
    )


@pytest.mark.parametrize(
    "payload",
    [
        {},
        _bounded_deadline_chain_payload(0),
        _bounded_deadline_chain_payload(257),
        {
            **_bounded_deadline_chain_payload(),
            "stream_state": {
                **_bounded_deadline_chain_payload()["stream_state"],
                "capacity_overflowed": True,
            },
        },
        {
            **_bounded_deadline_chain_payload(),
            "stream_state": {
                **_bounded_deadline_chain_payload()["stream_state"],
                "pending_cycle_event_count": 1,
            },
        },
    ],
)
def test_observer_bounded_deadline_chain_rejects_malformed_or_unbounded_state(
    payload: object,
) -> None:
    assert not cli._observer_bounded_deadline_chain_valid(payload)


def test_gate2_metrics_timeline_repairs_only_legacy_preventive_generation_quote() -> None:
    timeline, error = cli._gate2_metrics_timeline([
        {"topic": "/debug/overtake/mode", "timestamp_ns": 0, "data": "PASS"},
        {"topic": "/debug/overtake/metrics", "timestamp_ns": 1, "data": '{"mode":"PASS","preventive_side_role_generation":12"}'},
    ])
    assert error is None
    assert timeline["warnings"] == [{"code": "legacy_preventive_side_role_generation_quote_repaired", "count": 1}]


def test_gate2_metrics_timeline_rejects_other_malformed_json() -> None:
    timeline, error = cli._gate2_metrics_timeline([
        {"topic": "/debug/overtake/metrics", "timestamp_ns": 1, "data": '{"mode":"PASS","candidate_cost":1.2"}'},
    ])
    assert timeline is None
    assert error == "metrics_json_invalid"


def test_gate2_metrics_timeline_reports_ordered_transitions_and_costs() -> None:
    timeline, error = cli._gate2_metrics_timeline([
        {"topic": "/debug/overtake/mode", "timestamp_ns": 1, "data": "FOLLOW"},
        {"topic": "/debug/overtake/metrics", "timestamp_ns": 2, "data": '{"mode":"FOLLOW","reason":"blocked","candidate_cost":3.0}'},
        {"topic": "/debug/overtake/metrics", "timestamp_ns": 3, "data": '{"mode":"OVERTAKE_LEFT","reason":"clear","selected_cost":2.0}'},
        {"topic": "/debug/overtake/metrics", "timestamp_ns": 4, "data": '{"mode":"SAFE_STOP","reason":"cost_stop_threshold"}'},
    ])
    assert error is None
    assert timeline["topic_counts"] == {"/debug/overtake/mode": 1, "/debug/overtake/metrics": 3}
    assert [row["mode"] for row in timeline["mode_reason_transitions"]] == ["FOLLOW", "OVERTAKE_LEFT", "SAFE_STOP"]
    assert timeline["first_transition"]["fields"]["candidate_cost"] == 3.0
    failure = timeline["first_stop_or_blocked_transition_after_overtake_mode"]
    assert failure["reason"] == "cost_stop_threshold"
    assert failure["transition_index"] == 2
    assert failure["anchor_transition_index"] == 1
    assert timeline["overtake_anchor_transition"]["source_topic"] == "/debug/overtake/metrics"


def test_gate2_metrics_timeline_retains_fixed_v2_attestation_scalars() -> None:
    timeline, error = cli._gate2_metrics_timeline([
        {"topic": "/debug/overtake/mode", "timestamp_ns": 1, "data": "FOLLOW"},
        {"topic": "/debug/overtake/metrics", "timestamp_ns": 2, "data": json.dumps({
            "mode": "FOLLOW",
            "reason": "blocked",
            "state_lattice_v2_base_attestation_present": False,
            "state_lattice_v2_base_attestation_receive_count": 0,
            "state_lattice_v2_last_base_attestation_validation_reason": 255,
        })},
    ])
    assert error is None
    fields = timeline["first_transition"]["fields"]
    assert fields["state_lattice_v2_base_attestation_present"] is False
    assert fields["state_lattice_v2_base_attestation_receive_count"] == 0
    assert fields["state_lattice_v2_last_base_attestation_validation_reason"] == 255


def test_gate2_metrics_timeline_reports_v4_geometry_application() -> None:
    timeline, error = cli._gate2_metrics_timeline([
        {"topic": "/debug/overtake/mode", "timestamp_ns": 1, "data": "OVERTAKE_RIGHT"},
        {"topic": "/debug/overtake/metrics", "timestamp_ns": 2, "data": '{"mode":"OVERTAKE_RIGHT","reason":"selected_right_candidate"}'},
        {"topic": "/pure_pursuit/debug", "timestamp_ns": 3, "data": json.dumps({
            "controller": "simple_pure_pursuit",
            "stale_input": False,
            "overtake_override_applied": True,
            "overtake_override_generation": 474,
            "overtake_override_apply_reason": "applied",
            "trajectory_source": "trajectory_overtake_override",
            "v4_poc_contract": True,
            "v4_poc_identity_required": True,
            "v4_poc_identity_matched": True,
            "v4_poc_geometry_applied": True,
            "v4_poc_generation": 474,
            "steering_tire_angle_rad": 0.12,
            "overtake_lateral_offset_m": -2.4,
            "overtake_spatial_horizon_arc_m": 7.681,
        })},
    ])
    assert error is None
    application = timeline["v4_pp_application"]
    assert application["observation"] == "PP_DEBUG_REPORTED_V4_POC_GEOMETRY_APPLIED"
    assert application["applied_record_count"] == 1
    assert application["first_applied_record"]["generation"] == 474
    assert application["first_applied_record"]["trajectory_source"] == "trajectory_overtake_override"
    assert application["byte_exact_payload_identity_proven"] is False
    assert application["m4_credit"] is False


def test_gate2_metrics_timeline_reports_bounded_base_attestation_stages() -> None:
    status_topic = "/debug/overtake/state_lattice/v2_binding_status"
    base = {
        "availability_summary": True,
        "base_attestation_stage": 5,
        "base_attestation_build_result": 255,
        "base_attestation_build_diagnostic": 0,
        "base_attestation_validation_reason": 0,
        "base_attestation_point_invalid_field": 0,
        "base_attestation_failure_window_index": 4294967295,
        "base_attestation_failure_source_index": 4294967295,
        "base_attestation_snapshot_result": 255,
        "base_attestation_local_reject_reason": 255,
        "base_attestation_attempt_count": 12,
        "base_attestation_publish_count": 0,
        "base_attestation_race_arm_epoch": 0,
    }
    published = {
        **base,
        "base_attestation_stage": 11,
        "base_attestation_build_result": 0,
        "base_attestation_build_diagnostic": 0,
        "base_attestation_validation_reason": 1,
        "base_attestation_point_invalid_field": 1,
        "base_attestation_snapshot_result": 0,
        "base_attestation_local_reject_reason": 0,
        "base_attestation_attempt_count": 13,
        "base_attestation_publish_count": 1,
        "base_attestation_race_arm_epoch": 1,
    }
    timeline, error = cli._gate2_metrics_timeline([
        {"topic": "/debug/overtake/mode", "timestamp_ns": 1, "data": "FOLLOW"},
        {"topic": "/debug/overtake/metrics", "timestamp_ns": 2,
         "data": '{"mode":"FOLLOW","reason":"clear"}'},
        {"topic": status_topic, "timestamp_ns": 3, "data": json.dumps(base)},
        {"topic": status_topic, "timestamp_ns": 4, "data": json.dumps(published)},
    ])
    assert error is None
    diagnostic = timeline["v2_base_attestation"]
    assert diagnostic["summary_record_count"] == 2
    assert [row["base_attestation_stage"] for row in diagnostic["transitions"]] == [5, 11]
    assert diagnostic["last"]["base_attestation_publish_count"] == 1
    assert diagnostic["last"]["base_attestation_validation_reason"] == 1


def test_safegate2_analysis_uses_mcap_read_only_eval_container(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    bag = tmp_path / "output" / "run-1" / "d1" / "rosbag2_autoware"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  storage_identifier: mcap\n", encoding="utf-8")
    commands: list[list[str]] = []

    class Result:
        returncode = 0
        stdout = 'RTNETLINK answers: Operation not permitted\nAIC_TEST_ANALYSIS_JSON=[{"topic":"/debug/overtake/mode","timestamp_ns":1,"data":"FOLLOW"},{"topic":"/debug/overtake/metrics","timestamp_ns":2,"data":"{\\"mode\\":\\"FOLLOW\\"}"}]\n'

    def fake_run(command: object, **_: object) -> Result:
        commands.append(list(command))
        result = Result()
        if list(command)[:3] == ["docker", "container", "inspect"]:
            result.returncode = 1
        if list(command)[:4] == ["docker", "container", "ls", "-a"]:
            result.stdout = ""
        return result

    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(["--repo-root", str(tmp_path), "analyze", "safegate2-stopped-overtake", "run-1"])
    assert cli.analyze_safegate2_stopped_overtake(args) == 0
    output = json.loads(capsys.readouterr().out)
    assert "outcome" not in output
    assert output["analysis_status"] == "COMPLETE"
    assert output["claim_cap"] == "READ_ONLY_PLANNER_PP_APPLICATION_DIAGNOSTIC_ONLY"
    assert output["physical_outcome"] == "NOT_EVALUATED"
    assert output["planner_pp_authority_outcome"] == "V4_POC_GEOMETRY_APPLICATION_NOT_OBSERVED"
    assert output["m4_credit"] is False and output["m5_credit"] is False
    assert output["mode_topic_usage"] == "COUNT_ONLY_NOT_JOINED"
    assert output["behavior_classification"] == "NO_OVERTAKE_OBSERVED"
    assert all("failure" not in key and "unsafe" not in key and "cause" not in key and "policy" not in key for key in output)
    assert "aichallenge-2025-eval:latest" in commands[0]
    assert "--network" in commands[0] and "none" in commands[0]
    assert "--read-only" in commands[0]
    assert "--pull=never" in commands[0]
    assert "--name" in commands[0]
    assert "65534:65534" in commands[0]
    assert commands[0][-2] == "aic-test-analyzer"
    assert "storage_id='mcap'" in commands[0][-1]
    assert "'/pure_pursuit/debug'" in commands[0][-1]
    assert "\\n" not in commands[0][-1]


def test_safegate2_analysis_rejects_oversized_metadata_before_container(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    bag = tmp_path / "output" / "run-1" / "d1" / "rosbag2_autoware"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_bytes(
        b"storage_identifier: mcap\n"
        + b"x" * cli.GATE2_METADATA_MAX_BYTES
    )
    monkeypatch.setattr(
        cli, "_run",
        lambda *_args, **_kwargs: pytest.fail("container must not run"),
    )
    args = build_parser().parse_args([
        "--repo-root", str(tmp_path), "analyze",
        "safegate2-stopped-overtake", "run-1",
    ])
    assert cli.analyze_safegate2_stopped_overtake(args) == 4


@pytest.mark.parametrize("stdout", ["noise\n", "AIC_TEST_ANALYSIS_JSON=[]\nAIC_TEST_ANALYSIS_JSON=[]\n"])
def test_safegate2_analysis_rejects_absent_or_duplicate_marker(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, stdout: str
) -> None:
    bag = tmp_path / "output" / "run-1" / "d1" / "rosbag2_autoware"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("storage_identifier: mcap\n", encoding="utf-8")

    class Result:
        returncode = 0

        def __init__(self) -> None:
            self.stdout = stdout

    def fake_run(command: object, **_kwargs: object) -> Result:
        result = Result()
        if list(command)[:3] == ["docker", "container", "inspect"]:
            result.returncode = 1
        if list(command)[:4] == ["docker", "container", "ls", "-a"]:
            result.stdout = ""
        return result

    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(["--repo-root", str(tmp_path), "analyze", "safegate2-stopped-overtake", "run-1"])
    assert cli.analyze_safegate2_stopped_overtake(args) == 4


def _analysis_args_with_mcap_bag(tmp_path: Path) -> argparse.Namespace:
    bag = tmp_path / "output" / "run-1" / "d1" / "rosbag2_autoware"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("storage_identifier: mcap\n", encoding="utf-8")
    return build_parser().parse_args(
        ["--repo-root", str(tmp_path), "analyze", "safegate2-stopped-overtake", "run-1"]
    )


def test_safegate2_analysis_timeout_cleans_only_exact_container(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    commands: list[list[str]] = []

    class Result:
        def __init__(self, returncode: int) -> None:
            self.returncode = returncode
            self.stdout = ""

    def fake_run(command: object, **kwargs: object) -> Result:
        rendered = list(command)
        commands.append(rendered)
        if rendered[:2] == ["docker", "run"]:
            raise subprocess.TimeoutExpired(rendered, kwargs["timeout"])
        if rendered[:3] == ["docker", "container", "inspect"]:
            return Result(1)
        return Result(0)

    monkeypatch.setattr(cli, "_run", fake_run)
    assert cli.analyze_safegate2_stopped_overtake(_analysis_args_with_mcap_bag(tmp_path)) == 3
    assert json.loads(capsys.readouterr().out)["reason"] == "rosbag_analysis_exception:TimeoutExpired"
    assert not any(command[:3] == ["docker", "rm", "-f"] for command in commands)
    assert all("unrelated-container" not in command for command in commands)


def test_safegate2_analysis_nonzero_residual_is_cleaned_and_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    commands: list[list[str]] = []
    inspect_calls = 0

    class Result:
        def __init__(self, returncode: int) -> None:
            self.returncode = returncode
            self.stdout = ""

    def fake_run(command: object, **_: object) -> Result:
        nonlocal inspect_calls
        rendered = list(command)
        commands.append(rendered)
        if rendered[:2] == ["docker", "run"]:
            return Result(7)
        if rendered[:3] == ["docker", "container", "inspect"]:
            inspect_calls += 1
            return Result(0 if inspect_calls == 1 else 1)
        return Result(0)

    monkeypatch.setattr(cli, "_run", fake_run)
    assert cli.analyze_safegate2_stopped_overtake(_analysis_args_with_mcap_bag(tmp_path)) == 3
    assert json.loads(capsys.readouterr().out)["reason"] == "rosbag_analysis_container_owner_mismatch"
    assert not any(command[:3] == ["docker", "rm", "-f"] for command in commands)


def test_safegate2_analysis_cleanup_failure_is_not_success(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    class Result:
        def __init__(self, returncode: int) -> None:
            self.returncode = returncode
            self.stdout = ""

    def fake_run(command: object, **kwargs: object) -> Result:
        rendered = list(command)
        if rendered[:2] == ["docker", "run"]:
            raise subprocess.TimeoutExpired(rendered, kwargs["timeout"])
        if rendered[:3] == ["docker", "rm", "-f"]:
            return Result(1)
        return Result(1)

    monkeypatch.setattr(cli, "_run", fake_run)
    assert cli.analyze_safegate2_stopped_overtake(_analysis_args_with_mcap_bag(tmp_path)) == 3
    assert json.loads(capsys.readouterr().out)["reason"] == "rosbag_analysis_timeout_cleanup_failed"


def test_gate2_analysis_container_names_are_nonce_unique_and_bounded() -> None:
    first = cli._gate2_analysis_container_name("Run-1", "0123456789abcdef")
    second = cli._gate2_analysis_container_name("run-1", "fedcba9876543210")
    assert first != second
    assert len(cli._gate2_analysis_container_name("r" * 300, "0123456789abcdef")) <= 63


def test_gate2_analysis_timeout_own_label_cleans_exact_name(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    nonce = "0123456789abcdef"
    commands: list[list[str]] = []
    inspect_count = 0

    class Result:
        def __init__(self, returncode: int, stdout: str = "") -> None:
            self.returncode, self.stdout = returncode, stdout

    def fake_run(command: object, **kwargs: object) -> Result:
        nonlocal inspect_count
        rendered = list(command); commands.append(rendered)
        if rendered[:2] == ["docker", "run"]:
            raise subprocess.TimeoutExpired(rendered, kwargs["timeout"])
        if rendered[:3] == ["docker", "container", "inspect"]:
            inspect_count += 1
            return Result(0, nonce) if inspect_count == 1 else Result(1)
        return Result(0)

    monkeypatch.setattr(cli.secrets, "token_hex", lambda _: nonce)
    monkeypatch.setattr(cli, "_run", fake_run)
    assert cli.analyze_safegate2_stopped_overtake(_analysis_args_with_mcap_bag(tmp_path)) == 3
    name = cli._gate2_analysis_container_name("run-1", nonce)
    assert ["docker", "rm", "-f", name] in commands
    assert json.loads(capsys.readouterr().out)["reason"] == "rosbag_analysis_exception:TimeoutExpired"


@pytest.mark.parametrize("inspect_stdout,expected_rm,reason", [
    ("", False, "rosbag_analysis_exception:TimeoutExpired"),
    ("wrong-owner", False, "rosbag_analysis_timeout_cleanup_failed"),
])
def test_gate2_analysis_timeout_absent_or_wrong_owner_never_deletes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str],
    inspect_stdout: str, expected_rm: bool, reason: str,
) -> None:
    nonce = "0123456789abcdef"; commands: list[list[str]] = []

    class Result:
        def __init__(self, returncode: int, stdout: str = "") -> None:
            self.returncode, self.stdout = returncode, stdout

    def fake_run(command: object, **kwargs: object) -> Result:
        rendered = list(command); commands.append(rendered)
        if rendered[:2] == ["docker", "run"]:
            raise subprocess.TimeoutExpired(rendered, kwargs["timeout"])
        if rendered[:3] == ["docker", "container", "inspect"]:
            return Result(1) if not inspect_stdout else Result(0, inspect_stdout)
        return Result(0)

    monkeypatch.setattr(cli.secrets, "token_hex", lambda _: nonce)
    monkeypatch.setattr(cli, "_run", fake_run)
    assert cli.analyze_safegate2_stopped_overtake(_analysis_args_with_mcap_bag(tmp_path)) == 3
    assert any(command[:3] == ["docker", "rm", "-f"] for command in commands) is expected_rm
    assert json.loads(capsys.readouterr().out)["reason"] == reason


@pytest.mark.parametrize("list_returncode,list_stdout,raises", [
    (0, "", False),
    (1, "", True),
    (0, "aic-test-analysis-run-1-0123456789abcdef\n", True),
    (0, "unexpected-container\n", True),
])
def test_gate2_analysis_inspect_failure_requires_exact_absence_confirmation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
    list_returncode: int, list_stdout: str, raises: bool,
) -> None:
    name = "aic-test-analysis-run-1-0123456789abcdef"

    class Result:
        def __init__(self, returncode: int, stdout: str = "") -> None:
            self.returncode, self.stdout = returncode, stdout

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        if rendered[:3] == ["docker", "container", "inspect"]:
            return Result(1)
        if rendered[:4] == ["docker", "container", "ls", "-a"]:
            return Result(list_returncode, list_stdout)
        pytest.fail(f"unexpected command: {rendered}")

    monkeypatch.setattr(cli, "_run", fake_run)
    if raises:
        with pytest.raises(OSError):
            cli._gate2_analysis_container_ownership(tmp_path, name, "0123456789abcdef")
    else:
        assert cli._gate2_analysis_container_ownership(tmp_path, name, "0123456789abcdef") is None


def test_safegate2_analysis_rejects_symlink_escape(tmp_path: Path) -> None:
    run_root = tmp_path / "output" / "run-1" / "d1"
    run_root.mkdir(parents=True)
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "metadata.yaml").write_text("storage_identifier: mcap\n", encoding="utf-8")
    (run_root / "rosbag2_autoware").symlink_to(outside, target_is_directory=True)
    bag, error = cli._gate2_analysis_bag_path(tmp_path, "run-1")
    assert bag is None
    assert error == "rosbag_symlink_component_rejected"


def test_safegate2_analysis_rejects_output_ancestor_symlink(tmp_path: Path) -> None:
    outside = tmp_path / "outside-output"
    outside.mkdir()
    (tmp_path / "output").symlink_to(outside, target_is_directory=True)
    bag, error = cli._gate2_analysis_bag_path(tmp_path, "run-1")
    assert bag is None
    assert error == "rosbag_output_ancestor_symlink_rejected"


def test_gate2_metrics_timeline_rejects_empty_or_non_list_payload() -> None:
    for payload in ([], {"topic": "/debug/overtake/metrics"}):
        _, error = cli._gate2_metrics_timeline(payload)  # type: ignore[arg-type]
        assert error == "metrics_records_empty_or_invalid"


def test_gate2_metrics_timeline_rejects_bool_or_out_of_order_timestamps() -> None:
    for records in (
        [{"topic": "/debug/overtake/metrics", "timestamp_ns": True, "data": "{}"}],
        [{"topic": "/debug/overtake/mode", "timestamp_ns": 2, "data": "FOLLOW"}, {"topic": "/debug/overtake/metrics", "timestamp_ns": 1, "data": "{}"}],
    ):
        _, error = cli._gate2_metrics_timeline(records)
        assert error in {"metrics_record_schema_invalid", "metrics_timestamp_out_of_order"}


def test_private_prelaunch_start_decision_exercises_request_permit_and_liveness() -> None:
    nonce = "exact"
    request = json.dumps({"schema_version": 1, "run_nonce": nonce})
    permit = {
        "schema_version": 1, "run_nonce": nonce,
        "epoch_boundary": "observer_prelaunch_permit", "proposal_count": 0,
        "status_fault": False, "blocking_reasons": [],
        "experimental_epoch_status_ordinal": 1, "permit_status_ordinal": 1,
    }
    sentinel = []
    if private_prelaunch_start_decision(request, permit, expected_nonce=nonce, observer_alive=True, pp_alive=True):
        sentinel.append("planner")
    assert sentinel == ["planner"]
    for bad_request, bad_permit, observer_alive, pp_alive in (
        ("{", permit, True, True),
        (json.dumps({"schema_version": 1, "run_nonce": "wrong"}), permit, True, True),
        (request, {**permit, "run_nonce": "stale"}, True, True),
        (request, None, True, True),
        (request, permit, False, True),
        (request, permit, True, False),
    ):
        assert not private_prelaunch_start_decision(bad_request, bad_permit, expected_nonce=nonce, observer_alive=observer_alive, pp_alive=pp_alive)


def test_private_prelaunch_runtime_file_gate_is_fail_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    request = tmp_path / "request.json"
    permit = tmp_path / "permit.json"
    nonce = "exact"
    request.write_text(
        json.dumps({"schema_version": 1, "run_nonce": nonce}), encoding="utf-8"
    )
    permit_payload = {
        "schema_version": 1,
        "run_nonce": nonce,
        "epoch_boundary": "observer_prelaunch_permit",
        "proposal_count": 0,
        "status_fault": False,
        "blocking_reasons": [],
        "experimental_epoch_status_ordinal": 1,
        "permit_status_ordinal": 1,
    }
    permit.write_text(json.dumps(permit_payload), encoding="utf-8")
    request.chmod(0o644)
    permit.chmod(0o644)
    live_pids = {101, 202}

    def kill(pid: int, _signal: int) -> None:
        if pid not in live_pids:
            raise ProcessLookupError(pid)

    monkeypatch.setattr(os, "kill", kill)
    assert private_prelaunch_start_files_valid(
        request, permit, expected_nonce=nonce, observer_pid=101, pp_pid=202
    )

    permit.write_text("{", encoding="utf-8")
    assert not private_prelaunch_start_files_valid(
        request, permit, expected_nonce=nonce, observer_pid=101, pp_pid=202
    )
    permit.write_text(json.dumps({"schema_version": 1, "run_nonce": "stale"}), encoding="utf-8")
    assert not private_prelaunch_start_files_valid(
        request, permit, expected_nonce=nonce, observer_pid=101, pp_pid=202
    )
    permit.unlink()
    assert not private_prelaunch_start_files_valid(
        request, permit, expected_nonce=nonce, observer_pid=101, pp_pid=202
    )
    permit.write_text(json.dumps(permit_payload), encoding="utf-8")
    permit.chmod(0o644)
    live_pids.remove(202)
    assert not private_prelaunch_start_files_valid(
        request, permit, expected_nonce=nonce, observer_pid=101, pp_pid=202
    )


def test_cli_exposes_isolated_v2_uptake_smoke() -> None:
    args = build_parser().parse_args(["run", "v2-uptake-smoke"])
    assert args.scenario == "v2-uptake-smoke"
    assert args.ros_domain_id == 231
    assert args.runtime_timeout == 90.0


def test_cli_keeps_gate_timeout_internal() -> None:
    args = build_parser().parse_args(["run", "safegate2-stopped-overtake"])
    assert args.runtime_timeout == 90.0
    assert not hasattr(args, "gate_runtime_timeout")


def _write_valid_gate2_provenance(root: Path, run_id: str) -> None:
    provenance = root / "provenance"
    provenance.mkdir(exist_ok=True)
    (provenance / "prelaunch-manifest.json").write_text(json.dumps({
        "schema_version": 2,
        "run": {"run_id": run_id, "container_output_root": "/output", "host_output_root": str(root.parent.resolve())},
        "capture_stability": {"match": True},
        "launch_context": {"plain": {
            "RUN_GATE_SCENARIO": "SafetyGate/scenario2.yaml", "CONTROL_METHOD": "state_lattice_pure_pursuit",
            "RUN_KIND": "planner-pp-control-smoke", "PLANNER_PP_CONTROL_SMOKE_LIVE_SPATIAL": "true",
            "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED": "true",
            "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED": "true",
            "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID": "4101",
            "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID": "4201",
            "STATE_LATTICE_V2_SESSION_ID": "1",
            "ROSBAG": "true", "AWSIM_VEHICLES": "4",
            "AUTOWARE_RUN_MODE": "awsim-no-viz",
            "AUTOSTART_DEBUG_VISUALIZATION": "false",
        }, "hashed": {"GATE_EXTRA_ARGS": {
            "present": True,
            "sha256": hashlib.sha256(b"-batchmode -nographics --camera false --lidar false").hexdigest(),
        }}},
    }), encoding="utf-8")
    (provenance / "source-tree.sha256").write_text("source\n", encoding="utf-8")
    (provenance / "artifact-fingerprint.sha256").write_text("artifact\n", encoding="utf-8")


def _write_complete_gate2_evidence(root: Path, run_id: str) -> None:
    (root / "d1" / "rosbag2_autoware").mkdir(parents=True)
    (root / "d1" / "autoware.log").write_text("d1", encoding="utf-8")
    (root / "d1" / "rosbag2_autoware" / "metadata.yaml").write_text("metadata", encoding="utf-8")
    _write_valid_gate2_provenance(root, run_id)
    (root / "safety-gate-result.json").write_text(json.dumps({
        "schema_version": "v1", "gate_arg": "test2", "scenario_folder": "SafetyGate",
        "all_passed": True, "tests": [{"test_name": "test2", "passed": True}],
    }), encoding="utf-8")


def _write_valid_gate2_postrun(root: Path, run_id: str) -> None:
    provenance = root / "provenance"
    (provenance / "postrun-manifest.json").write_text(json.dumps({
        "schema_version": 2, "run": {"run_id": run_id},
    }), encoding="utf-8")
    (provenance / "verification.json").write_text(json.dumps({
        "schema_version": 2,
        "verified_at": "2026-08-02T00:00:00+09:00",
        "verified": True,
        "rosbag_metadata": [{"path": str(root / "d1" / "rosbag2_autoware" / "metadata.yaml")}],
        "comparisons": {"source_tree": {"match": True}},
    }), encoding="utf-8")


@pytest.mark.parametrize("launch_context", ["invalid", [], 1])
def test_gate2_manifest_scalar_launch_context_is_invalid(
    tmp_path: Path, launch_context: object
) -> None:
    path = tmp_path / "prelaunch-manifest.json"
    path.write_text(json.dumps({"schema_version": 2, "launch_context": launch_context}), encoding="utf-8")
    assert cli._gate2_manifest_error(path, tmp_path, "run-1", time.time_ns()) == "prelaunch_manifest_launch_context_invalid"


@pytest.mark.parametrize("run", ["invalid", [], 1])
def test_gate2_postrun_scalar_run_is_invalid(tmp_path: Path, run: object) -> None:
    path = tmp_path / "postrun-manifest.json"
    path.write_text(json.dumps({"schema_version": 2, "run": run}), encoding="utf-8")
    assert cli._gate2_postrun_error(path, "run-1", time.time_ns()) == "postrun-manifest_run_invalid"


def test_safegate2_wrapper_uses_fixed_target_and_official_result_only(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    commands: list[list[str]] = []
    launch_environment: dict[str, str] = {}
    preflight_environment: dict[str, str] = {}

    class Result:
        def __init__(self, returncode: int = 0) -> None:
            self.returncode = returncode
            self.stdout = ""

    def fake_run(command: object, **kwargs: object) -> Result:
        rendered = list(command)
        commands.append(rendered)
        if rendered[:2] == ["make", "planner-pp-control-smoke"]:
            launch_environment.update(kwargs["env"])
            output = tmp_path / "output" / "fixture-gate2"
            (output / "d1" / "rosbag2_autoware").mkdir(parents=True)
            (output / "d1" / "autoware.log").write_text("d1", encoding="utf-8")
            (output / "d1" / "rosbag2_autoware" / "metadata.yaml").write_text(
                "rosbag2_bagfile_information:\n", encoding="utf-8"
            )
            _write_valid_gate2_provenance(output, "fixture-gate2")
            (output / "safety-gate-result.json").write_text(
                json.dumps(
                    {
                        "schema_version": "v1",
                        "gate_arg": "test2",
                        "scenario_folder": "SafetyGate",
                        "all_passed": True,
                        "tests": [{"test_name": "test2", "passed": True}],
                    }
                ),
                encoding="utf-8",
            )
        if rendered[:2] == ["make", "verify-run-fingerprint"]:
            _write_valid_gate2_postrun(tmp_path / "output" / "fixture-gate2", "fixture-gate2")
        return Result()

    monkeypatch.setenv("AWSIM_EXTRA_ARGS", "--unsafe-fixture")
    monkeypatch.setenv("MAKEFLAGS", "--unsafe-fixture")
    monkeypatch.setenv("COMPOSE_PROJECT_NAME", "unsafe-fixture")

    def fake_preflight(_repo: Path, env: dict[str, str]) -> tuple[None, None]:
        preflight_environment.update(env)
        return None, None

    monkeypatch.setattr(cli, "_gate2_compose_preflight", fake_preflight)
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "fixture-gate2"]
    )
    assert cli.run_scenario(args) == 0
    result = read_result(tmp_path / "analysis" / "aic_test" / "runs" / "fixture-gate2" / "result.json")
    assert result["outcome"] == "PASS"
    assert result["maximum_claim"] == "OFFICIAL_GATE2_PHYSICAL_OUTCOME_ONLY"
    assert result["artifacts"]["d1_rosbag"]["exists"] is True
    assert result["artifacts"]["provenance_manifests"][0]["sha256"]
    assert "AWSIM_EXTRA_ARGS" not in launch_environment
    assert "MAKEFLAGS" not in launch_environment
    assert "COMPOSE_PROJECT_NAME" not in launch_environment
    assert preflight_environment == launch_environment
    assert commands == [
        [
            "make",
            "planner-pp-control-smoke",
            "RUN_ID=fixture-gate2",
                "GATE_EXTRA_ARGS=-batchmode -nographics --camera false --lidar false",
                "AWSIM_EXTRA_ARGS=",
                "AUTOWARE_RUN_MODE=awsim-no-viz",
                "AUTOSTART_DEBUG_VISUALIZATION=false",
                "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED=true",
                "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED=true",
                "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID=4101",
                "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID=4201",
                "STATE_LATTICE_V2_SESSION_ID=1",
                "OUTPUT_HOST_ROOT=./output",
            "OUTPUT_ROOT=/output",
        ],
        ["make", "down"],
        [
            "make",
            "verify-run-fingerprint",
            "VERIFY_RUN_ID=fixture-gate2",
            "OUTPUT_HOST_ROOT=./output",
            "OUTPUT_ROOT=/output",
        ],
    ]


def test_safegate2_wrapper_rejects_malformed_official_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class Result:
        returncode = 0
        stdout = ""

    def fake_run(command: object, **_: object) -> Result:
        if list(command)[:2] == ["make", "planner-pp-control-smoke"]:
            output = tmp_path / "output" / "invalid-gate2"
            output.mkdir(parents=True)
            (output / "safety-gate-result.json").write_text(
                '{"schema_version":"v1","gate_arg":"test1","all_passed":true,"tests":[]}',
                encoding="utf-8",
            )
        return Result()

    monkeypatch.setattr(cli, "_gate2_compose_preflight", lambda _repo, _env: (None, None))
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "invalid-gate2"]
    )
    assert cli.run_scenario(args) == 4
    result = read_result(tmp_path / "analysis" / "aic_test" / "runs" / "invalid-gate2" / "result.json")
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert "safety_gate_result_not_test2" in result["reasons"]


def test_safegate2_timeout_without_result_is_infrastructure_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    commands: list[tuple[list[str], float]] = []

    class Result:
        returncode = 0
        stdout = ""

    def fake_run(command: object, **kwargs: object) -> Result:
        commands.append((list(command), float(kwargs["timeout"])))
        if list(command)[:2] == ["make", "planner-pp-control-smoke"]:
            raise subprocess.TimeoutExpired(list(command), kwargs["timeout"])
        return Result()

    monkeypatch.setattr(cli, "_gate2_compose_preflight", lambda _repo, _env: (None, None))
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "timeout-gate2"]
    )
    assert cli.run_scenario(args) == 3
    result = read_result(tmp_path / "analysis" / "aic_test" / "runs" / "timeout-gate2" / "result.json")
    assert result["outcome"] == "INFRASTRUCTURE_FAILED"
    assert commands == [
        (["make", "planner-pp-control-smoke", "RUN_ID=timeout-gate2", "GATE_EXTRA_ARGS=-batchmode -nographics --camera false --lidar false", "AWSIM_EXTRA_ARGS=", "AUTOWARE_RUN_MODE=awsim-no-viz", "AUTOSTART_DEBUG_VISUALIZATION=false", "STATE_LATTICE_V2_LIVE_PROPOSAL_PUBLISH_ENABLED=true", "STATE_LATTICE_V2_LIVE_PROPOSAL_ACCEPT_ENABLED=true", "STATE_LATTICE_V2_PRODUCER_INSTANCE_ID=4101", "STATE_LATTICE_V2_PP_PRODUCER_INSTANCE_ID=4201", "STATE_LATTICE_V2_SESSION_ID=1", "OUTPUT_HOST_ROOT=./output", "OUTPUT_ROOT=/output"], 240.0),
        (["make", "down"], 210.0),
    ]


def test_safegate2_failed_target_without_result_is_infrastructure_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class Result:
        stdout = ""

        def __init__(self, returncode: int) -> None:
            self.returncode = returncode

    def fake_run(command: object, **_: object) -> Result:
        return Result(7 if list(command)[:2] == ["make", "planner-pp-control-smoke"] else 0)

    monkeypatch.setattr(cli, "_gate2_compose_preflight", lambda _repo, _env: (None, None))
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "failed-no-result"]
    )
    assert cli.run_scenario(args) == 3
    result = read_result(tmp_path / "analysis" / "aic_test" / "runs" / "failed-no-result" / "result.json")
    assert result["outcome"] == "INFRASTRUCTURE_FAILED"
    assert "fixed_gate2_target_failed" in result["reasons"]


def test_safegate2_official_failure_records_first_reason(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class Result:
        returncode = 0
        stdout = ""

    def fake_run(command: object, **_: object) -> Result:
        if list(command)[:2] == ["make", "planner-pp-control-smoke"]:
            output = tmp_path / "output" / "failed-gate2"
            output.mkdir(parents=True)
            (output / "safety-gate-result.json").write_text(
                json.dumps({
                    "schema_version": "v1", "gate_arg": "test2", "scenario_folder": "SafetyGate", "all_passed": False,
                    "tests": [{"test_name": "test2", "passed": False, "fail_reason": "pass_zone_not_entered"}],
                }),
                encoding="utf-8",
            )
            _write_valid_gate2_provenance(output, "failed-gate2")
            (output / "d1" / "rosbag2_autoware").mkdir(parents=True)
            (output / "d1" / "autoware.log").write_text("d1", encoding="utf-8")
            (output / "d1" / "rosbag2_autoware" / "metadata.yaml").write_text("metadata", encoding="utf-8")
        if list(command)[:2] == ["make", "verify-run-fingerprint"]:
            _write_valid_gate2_postrun(tmp_path / "output" / "failed-gate2", "failed-gate2")
        return Result()

    monkeypatch.setattr(cli, "_gate2_compose_preflight", lambda _repo, _env: (None, None))
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "failed-gate2"]
    )
    assert cli.run_scenario(args) == 1
    result = read_result(tmp_path / "analysis" / "aic_test" / "runs" / "failed-gate2" / "result.json")
    assert result["outcome"] == "ASSERTION_FAILED"
    assert result["official_gate2"] == {"all_passed": False, "test2_fail_reason": "pass_zone_not_entered"}


def test_safegate2_existing_run_dir_is_precondition_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_dir = tmp_path / "analysis" / "aic_test" / "runs" / "duplicate-gate2"
    run_dir.mkdir(parents=True)
    monkeypatch.setattr(cli, "_gate2_compose_preflight", lambda _repo, _env: (None, None))
    monkeypatch.setattr(cli, "_run", lambda *_args, **_kwargs: pytest.fail("must not run"))
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "duplicate-gate2"]
    )
    assert cli.run_scenario(args) == 3
    assert not (run_dir / "result.json").exists()


def test_safegate2_active_compose_is_rejected_without_cleanup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    commands: list[list[str]] = []

    class Result:
        returncode = 0
        stdout = "container-id\n"

    def fake_run(command: object, **_: object) -> Result:
        commands.append(list(command))
        return Result()

    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "active-compose"]
    )
    assert cli.run_scenario(args) == 3
    assert commands == [["docker", "compose", "ps", "-q"]]


@pytest.mark.parametrize("down_failed,residual", [(True, False), (False, True)])
def test_safegate2_cleanup_failure_cannot_pass(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, down_failed: bool, residual: bool
) -> None:
    compose_calls = 0

    class Result:
        stdout = ""

        def __init__(self, returncode: int = 0) -> None:
            self.returncode = returncode

    def fake_preflight(_repo: Path, _env: dict[str, str]) -> tuple[str | None, str | None]:
        nonlocal compose_calls
        compose_calls += 1
        return ("managed_compose_project_active:default", None) if residual and compose_calls == 2 else (None, None)

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        if rendered[:2] == ["make", "planner-pp-control-smoke"]:
            _write_complete_gate2_evidence(tmp_path / "output" / "cleanup-failure", "cleanup-failure")
        if rendered == ["make", "down"]:
            return Result(1 if down_failed else 0)
        return Result()

    monkeypatch.setattr(cli, "_gate2_compose_preflight", fake_preflight)
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "cleanup-failure"]
    )
    assert cli.run_scenario(args) == 3
    result = read_result(tmp_path / "analysis" / "aic_test" / "runs" / "cleanup-failure" / "result.json")
    assert result["outcome"] == "INFRASTRUCTURE_FAILED"


def test_safegate2_verified_false_is_invalid_evidence(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class Result:
        returncode = 0
        stdout = ""

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        root = tmp_path / "output" / "verified-false"
        if rendered[:2] == ["make", "planner-pp-control-smoke"]:
            _write_complete_gate2_evidence(root, "verified-false")
        if rendered[:2] == ["make", "verify-run-fingerprint"]:
            _write_valid_gate2_postrun(root, "verified-false")
            (root / "provenance" / "verification.json").write_text(
                json.dumps({"schema_version": 2, "verified": False}), encoding="utf-8"
            )
        return Result()

    monkeypatch.setattr(cli, "_gate2_compose_preflight", lambda _repo, _env: (None, None))
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "verified-false"]
    )
    assert cli.run_scenario(args) == 4


@pytest.mark.parametrize("comparisons", [{}, {"source_tree": {"match": False}}])
def test_safegate2_empty_or_false_verification_comparison_is_invalid(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, comparisons: dict[str, dict[str, bool]]
) -> None:
    class Result:
        returncode = 0
        stdout = ""

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        root = tmp_path / "output" / "comparison-invalid"
        if rendered[:2] == ["make", "planner-pp-control-smoke"]:
            _write_complete_gate2_evidence(root, "comparison-invalid")
        if rendered[:2] == ["make", "verify-run-fingerprint"]:
            _write_valid_gate2_postrun(root, "comparison-invalid")
            verification_path = root / "provenance" / "verification.json"
            verification = json.loads(verification_path.read_text(encoding="utf-8"))
            verification["comparisons"] = comparisons
            verification_path.write_text(json.dumps(verification), encoding="utf-8")
        return Result()

    monkeypatch.setattr(cli, "_gate2_compose_preflight", lambda _repo, _env: (None, None))
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "comparison-invalid"]
    )
    assert cli.run_scenario(args) == 4


@pytest.mark.parametrize("stale,wrong_run,missing", [(True, False, False), (False, True, False), (False, False, True)])
def test_safegate2_rejects_stale_or_mismatched_provenance(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, stale: bool, wrong_run: bool, missing: bool
) -> None:
    class Result:
        returncode = 0
        stdout = ""

    def fake_run(command: object, **_: object) -> Result:
        if list(command)[:2] == ["make", "planner-pp-control-smoke"]:
            root = tmp_path / "output" / "provenance-invalid"
            _write_complete_gate2_evidence(root, "wrong-run" if wrong_run else "provenance-invalid")
            if stale:
                stale_ns = 1
                os.utime(root / "safety-gate-result.json", ns=(stale_ns, stale_ns))
                os.utime(root / "provenance" / "prelaunch-manifest.json", ns=(stale_ns, stale_ns))
            if missing:
                (root / "provenance" / "prelaunch-manifest.json").unlink()
        return Result()

    monkeypatch.setattr(cli, "_gate2_compose_preflight", lambda _repo, _env: (None, None))
    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "provenance-invalid"]
    )
    assert cli.run_scenario(args) == 4
    result = read_result(tmp_path / "analysis" / "aic_test" / "runs" / "provenance-invalid" / "result.json")
    assert result["outcome"] == "INVALID_EVIDENCE"
    assert any("stale" in reason or "run_id_mismatch" in reason or "unreadable" in reason for reason in result["reasons"])


def test_safegate2_existing_output_is_rejected_before_compose(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (tmp_path / "output" / "occupied-output").mkdir(parents=True)
    monkeypatch.setattr(cli, "_run", lambda *_args, **_kwargs: pytest.fail("must not run"))
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", "occupied-output"]
    )
    assert cli.run_scenario(args) == 3


@pytest.mark.parametrize("run_id", ["", "../escape", "/absolute", "bad space", "a/b"])
def test_safegate2_invalid_run_id_is_precondition_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, run_id: str
) -> None:
    monkeypatch.setattr(cli, "_run", lambda *_args, **_kwargs: pytest.fail("must not run"))
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "safegate2-stopped-overtake", "--run-id", run_id]
    )
    assert cli.run_scenario(args) == 3


def _write_cyclonedds_config(path: Path, discovery: str = "") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        """<CycloneDDS xmlns=\"https://cdds.io/config\">
  <Domain Id=\"any\">
    <General><AllowMulticast>default</AllowMulticast>
      <Interfaces><NetworkInterface name=\"lo\" multicast=\"default\" /></Interfaces>
    </General>
    %s
  </Domain>
</CycloneDDS>
"""
        % discovery,
        encoding="utf-8",
    )


def test_current_cyclonedds_config_domain_port_boundaries_are_config_derived() -> None:
    repo = Path(__file__).resolve().parents[3]
    current = cli._cyclonedds_domain_admissibility(repo, 231)
    assert current["domain_config_binding"] == "DOMAIN_CONFIG_BOUND"
    assert current["domain_port_admissibility"] == "DOMAIN_PORT_ADMISSIBLE"
    assert current["participant_index"] == "default"
    assert current["computed_domain_base"] == 65_150
    assert current["participant_index_branches"] == {
        "none": {"admissible": True, "candidate_count": 1, "first_candidate_ports": {"multicast_meta": 65_150, "multicast_data": 65_151}},
        "auto": {"admissible": True, "candidate_count": 100, "first_candidate_ports": {"multicast_meta": 65_150, "multicast_data": 65_151, "unicast_meta": 65_160, "unicast_data": 65_161}},
    }
    for domain, base in ((232, 65_400), (233, 65_650), (235, 66_150)):
        invalid = cli._cyclonedds_domain_admissibility(repo, domain)
        assert invalid["domain_config_binding"] == "DOMAIN_CONFIG_BOUND"
        assert invalid["domain_port_admissibility"] == "DOMAIN_PORT_INADMISSIBLE"
        assert invalid["computed_domain_base"] == base
        assert invalid["error"] == "cyclonedds_domain_port_out_of_range_or_collision"


def test_v2_udp_socket_preflight_checks_all_auto_candidate_ports(
    monkeypatch: pytest.MonkeyPatch
) -> None:
    evidence = cli._cyclonedds_domain_admissibility(Path(__file__).resolve().parents[3], 227)
    assert evidence["admissible"] is True

    class Result:
        returncode = 0
        stdout = "UNCONN 0 0 0.0.0.0:64359 0.0.0.0:* users:((\"holder\",pid=1,fd=3))\n"
        stderr = ""

    monkeypatch.setattr(subprocess, "run", lambda *_args, **_kwargs: Result())
    preflight, error = cli._v2_uptake_udp_socket_preflight(evidence)
    assert preflight["candidate_ports"][0:2] == [64150, 64151]
    assert preflight["candidate_ports"][-1] == 64359
    assert error == "udp_socket_candidate_port_bound"
    assert preflight["bound_socket_lines"] == [Result.stdout.strip()]


def test_v2_udp_socket_preflight_fails_closed_when_inspection_unavailable(
    monkeypatch: pytest.MonkeyPatch
) -> None:
    evidence = cli._cyclonedds_domain_admissibility(Path(__file__).resolve().parents[3], 227)
    monkeypatch.setattr(subprocess, "run", lambda *_args, **_kwargs: (_ for _ in ()).throw(FileNotFoundError()))
    preflight, error = cli._v2_uptake_udp_socket_preflight(evidence)
    assert error == "udp_socket_inspection_unavailable"
    assert preflight["inspection_available"] is False


def test_v2_udp_socket_preflight_failure_never_consumes_budget_or_starts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    config = tmp_path / "vehicle" / "cyclonedds.xml"
    config.parent.mkdir(parents=True)
    config.write_text("config", encoding="utf-8")
    monkeypatch.setattr(cli, "_cyclonedds_domain_admissibility", lambda *_: {
        "admissible": True, "config_path": str(config),
        "container_cyclonedds_uri": "file:///opt/autoware/cyclonedds.xml",
        "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
    })
    monkeypatch.setattr(cli, "_cyclonedds_config_hash_matches", lambda *_: True)
    monkeypatch.setattr(cli, "_v2_uptake_udp_socket_preflight", lambda *_: (
        {"inspection_available": False, "bound_socket_lines": []},
        "udp_socket_inspection_unavailable",
    ))
    monkeypatch.setattr(cli, "_run", lambda *_args, **_kwargs: pytest.fail("must not start"))
    before = (tmp_path / "review-state.json").read_bytes()
    args = build_parser().parse_args([
        "--repo-root", str(tmp_path), "run", "v2-uptake-smoke", "--run-id", "udp-blocked",
    ])
    assert cli.run_v2_uptake_smoke(args) == 3
    assert (tmp_path / "review-state.json").read_bytes() == before
    result = json.loads((tmp_path / "analysis/aic_test/runs/udp-blocked/result.json").read_text())
    assert result["reasons"] == ["udp_socket_inspection_unavailable"]


def test_cyclonedds_port_overrides_and_explicit_participant_indexes(tmp_path: Path) -> None:
    _write_cyclonedds_config(
        tmp_path / "vehicle" / "cyclonedds.xml",
        """<Discovery><ParticipantIndex>3</ParticipantIndex><Ports>
      <Base>100</Base><DomainGain>20</DomainGain><MulticastMetaOffset>4</MulticastMetaOffset>
      <MulticastDataOffset>5</MulticastDataOffset><ParticipantGain>7</ParticipantGain>
      <UnicastMetaOffset>8</UnicastMetaOffset><UnicastDataOffset>9</UnicastDataOffset>
    </Ports></Discovery>""",
    )
    numeric = cli._cyclonedds_domain_admissibility(tmp_path, 4)
    assert numeric["admissible"] is True
    assert numeric["participant_index"] == "3"
    assert numeric["computed_domain_base"] == 180
    assert numeric["computed_ports"]["3"]["first_candidate_ports"] == {
        "multicast_meta": 184, "multicast_data": 185,
        "unicast_meta": 209, "unicast_data": 210,
    }

    _write_cyclonedds_config(
        tmp_path / "vehicle" / "cyclonedds.xml",
        "<Discovery><ParticipantIndex>none</ParticipantIndex></Discovery>",
    )
    none = cli._cyclonedds_domain_admissibility(tmp_path, 232)
    assert none["admissible"] is True
    assert none["computed_ports"]["none"]["first_candidate_ports"] == {
        "multicast_meta": 65_400,
        "multicast_data": 65_401,
    }


def test_cyclonedds_hash_mismatch_is_detected_before_launch(tmp_path: Path) -> None:
    path = tmp_path / "vehicle" / "cyclonedds.xml"
    _write_cyclonedds_config(path)
    evidence = cli._cyclonedds_domain_admissibility(tmp_path, 232)
    assert cli._cyclonedds_config_hash_matches(evidence) is True
    evidence["config_sha256"] = evidence["config_sha256"].upper()
    assert cli._cyclonedds_config_hash_matches(evidence) is True
    evidence["config_sha256"] = "a" * 63
    assert cli._cyclonedds_config_hash_matches(evidence) is False
    path.write_text("<CycloneDDS />", encoding="utf-8")
    evidence["config_sha256"] = "a" * 64
    assert cli._cyclonedds_config_hash_matches(evidence) is False


def test_cyclonedds_namespace_root_and_normal_attributes_are_strict(tmp_path: Path) -> None:
    path = tmp_path / "vehicle" / "cyclonedds.xml"
    path.parent.mkdir(parents=True)
    path.write_text(
        """<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xsi:schemaLocation="https://cdds.io/config ignored"><!-- normal comment -->
        <Domain Id="any"><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain>
        </CycloneDDS>""",
        encoding="utf-8",
    )
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["admissible"] is True
    for config, error in (
        ("<CycloneDDS><Domain Id=\"any\" /></CycloneDDS>", "cyclonedds_root_namespace_invalid"),
        ("<x:CycloneDDS xmlns:x=\"urn:wrong\"><x:Domain Id=\"any\" /></x:CycloneDDS>", "cyclonedds_root_namespace_invalid"),
        ("<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain Id=\"any\"><Ports xmlns=\"urn:wrong\"><Base>1</Base></Ports></Domain></CycloneDDS>", "cyclonedds_semantic_namespace_invalid"),
    ):
        path.write_text(config, encoding="utf-8")
        assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == error


@pytest.mark.parametrize(
    "domain_attributes,error",
    [
        ("", None),
        (" Id=\"any\"", None),
        (" xmlns:x=\"urn:attr\" x:Id=\"any\"", "cyclonedds_domain_attribute_invalid"),
        (" xmlns:x=\"urn:attr\" Id=\"any\" x:Id=\"any\"", "cyclonedds_domain_attribute_invalid"),
        (" xmlns:x=\"urn:attr\" x:Other=\"1\"", "cyclonedds_domain_attribute_invalid"),
        (" id=\"any\"", "cyclonedds_domain_attribute_invalid"),
    ],
)
def test_selected_domain_allows_only_unqualified_id_attribute(
    tmp_path: Path, domain_attributes: str, error: str | None
) -> None:
    path = tmp_path / "vehicle" / "cyclonedds.xml"
    path.parent.mkdir(parents=True)
    path.write_text(
        f"<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain{domain_attributes}><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>",
        encoding="utf-8",
    )
    result = cli._cyclonedds_domain_admissibility(tmp_path, 1)
    if error is None:
        assert result["admissible"] is True
    else:
        assert result["error"] == error


def test_cyclonedds_uri_rejects_final_symlink_missing_and_nonregular(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "target.xml"
    _write_cyclonedds_config(target)
    symlink = tmp_path / "link.xml"
    symlink.symlink_to(target)
    monkeypatch.setenv("CYCLONEDDS_URI", f"file://{symlink}")
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == (
        "cyclonedds_config_final_component_symlink"
    )
    monkeypatch.setenv("CYCLONEDDS_URI", f"file://{tmp_path / 'missing.xml'}")
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == "cyclonedds_config_missing"
    directory = tmp_path / "directory.xml"
    directory.mkdir()
    monkeypatch.setenv("CYCLONEDDS_URI", f"file://{directory}")
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == (
        "cyclonedds_config_not_regular_file"
    )


def test_default_config_path_rejects_final_symlink_before_preflight_side_effects(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    vehicle = tmp_path / "vehicle"
    vehicle.mkdir()
    target = tmp_path / "target.xml"
    _write_cyclonedds_config(target)
    (vehicle / "cyclonedds.xml").symlink_to(target)
    monkeypatch.setattr(
        cli, "_run", lambda *_args, **_kwargs: pytest.fail("launch helper must not run")
    )
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "v2-uptake-smoke", "--run-id", "default-symlink"]
    )
    assert cli.run_v2_uptake_smoke(args) == 3
    assert not (tmp_path / "analysis").exists()


@pytest.mark.parametrize(
    "value,expected",
    [
        ("A" * 64, "a" * 64),
        ("a" * 64, "a" * 64),
        ("a" * 63, None),
        ("g" * 64, None),
    ],
)
def test_cyclonedds_sha256_normalization_and_format(value: str, expected: str | None) -> None:
    assert cli._normalized_sha256(value) == expected


def test_cyclonedds_external_domain_auto_and_fixed_port_contracts(tmp_path: Path) -> None:
    path = tmp_path / "vehicle" / "cyclonedds.xml"
    _write_cyclonedds_config(
        path,
        """<Discovery><ExternalDomainId>2</ExternalDomainId><ParticipantIndex>auto</ParticipantIndex>
        <MaxAutoParticipantIndex>1</MaxAutoParticipantIndex><Ports><Base>100</Base><DomainGain>20</DomainGain>
        <MulticastMetaOffset>0</MulticastMetaOffset><MulticastDataOffset>1</MulticastDataOffset>
        <ParticipantGain>2</ParticipantGain><UnicastMetaOffset>10</UnicastMetaOffset>
        <UnicastDataOffset>11</UnicastDataOffset></Ports></Discovery>""",
    )
    auto = cli._cyclonedds_domain_admissibility(tmp_path, 1)
    assert auto["admissible"] is True
    assert auto["effective_domain_id"] == 2
    assert auto["computed_domain_base"] == 140
    assert auto["max_auto_participant_index"] == 1
    assert auto["computed_ports"]["auto"]["first_candidate_ports"] == {
        "multicast_meta": 140,
        "multicast_data": 141,
        "unicast_meta": 150,
        "unicast_data": 151,
    }

    _write_cyclonedds_config(
        path,
        "<Discovery><ExternalDomainId>-1</ExternalDomainId></Discovery>",
    )
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == (
        "cyclonedds_external_domain_id_unsupported"
    )
    assert cli._cyclonedds_domain_admissibility(tmp_path, -1)["error"] == (
        "ros_domain_id_negative"
    )

    _write_cyclonedds_config(
        path,
        "<Discovery><ParticipantIndex>none</ParticipantIndex><Ports><MulticastMetaOffset>0</MulticastMetaOffset><MulticastDataOffset>0</MulticastDataOffset></Ports></Discovery>",
    )
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == (
        "cyclonedds_domain_port_out_of_range_or_collision"
    )


def test_auto_participant_index_candidates_are_alternatives(tmp_path: Path) -> None:
    _write_cyclonedds_config(
        tmp_path / "vehicle" / "cyclonedds.xml",
        """<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>1</MaxAutoParticipantIndex>
        <Ports><Base>100</Base><DomainGain>1</DomainGain><MulticastMetaOffset>0</MulticastMetaOffset>
        <MulticastDataOffset>1</MulticastDataOffset><ParticipantGain>0</ParticipantGain>
        <UnicastMetaOffset>10</UnicastMetaOffset><UnicastDataOffset>11</UnicastDataOffset></Ports></Discovery>""",
    )
    result = cli._cyclonedds_domain_admissibility(tmp_path, 1)
    assert result["admissible"] is True
    assert result["participant_index_branches"]["auto"]["candidate_count"] == 2


@pytest.mark.parametrize(
    "field",
    ["Base", "DomainGain", "MulticastMetaOffset", "MulticastDataOffset", "ParticipantGain", "UnicastMetaOffset", "UnicastDataOffset"],
)
def test_empty_arithmetic_scalar_is_fail_closed(tmp_path: Path, field: str) -> None:
    _write_cyclonedds_config(
        tmp_path / "vehicle" / "cyclonedds.xml",
        f"<Discovery><Ports><{field}></{field}></Ports></Discovery>",
    )
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == (
        "cyclonedds_port_value_malformed"
    )


@pytest.mark.parametrize(
    "discovery,error",
    [
        (
            "<Discovery><ParticipantIndex>none</ParticipantIndex><ParticipantIndex>0</ParticipantIndex></Discovery>",
            "cyclonedds_duplicate_participant_index",
        ),
        (
            "<Discovery><ExternalDomainId>1</ExternalDomainId><ExternalDomainId>2</ExternalDomainId></Discovery>",
            "cyclonedds_duplicate_external_domain_id",
        ),
        (
            "<Discovery><Ports><Base>1</Base><Base>2</Base></Ports></Discovery>",
            "cyclonedds_duplicate_port_field",
        ),
        (
            "<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>1</MaxAutoParticipantIndex><MaxAutoParticipantIndex>2</MaxAutoParticipantIndex></Discovery>",
            "cyclonedds_duplicate_max_auto_participant_index",
        ),
    ],
)
def test_cyclonedds_duplicate_matching_scalars_fail_closed(
    tmp_path: Path, discovery: str, error: str
) -> None:
    _write_cyclonedds_config(tmp_path / "vehicle" / "cyclonedds.xml", discovery)
    result = cli._cyclonedds_domain_admissibility(tmp_path, 1)
    assert result["admissible"] is False
    assert result["error"] == error


def test_cyclonedds_relative_uri_and_duplicate_matching_domain_fail_closed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("CYCLONEDDS_URI", "file:relative.xml")
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == (
        "unsupported_or_relative_cyclonedds_uri"
    )
    monkeypatch.delenv("CYCLONEDDS_URI")
    path = tmp_path / "vehicle" / "cyclonedds.xml"
    path.parent.mkdir(parents=True)
    path.write_text(
        "<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain Id=\"any\" /><Domain Id=\"1\" /></CycloneDDS>",
        encoding="utf-8",
    )
    assert cli._cyclonedds_domain_admissibility(tmp_path, 1)["error"] == (
        "cyclonedds_domain_binding_ambiguous"
    )


@pytest.mark.parametrize(
    "config,error",
    [
        ("<CycloneDDS>", "cyclonedds_config_unreadable_or_malformed:ParseError"),
        (
            "<CycloneDDS xmlns=\"https://cdds.io/config\"><Domain Id=\"any\"><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>-1</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>",
            "cyclonedds_max_auto_participant_index_invalid",
        ),
    ],
)
def test_cyclonedds_invalid_or_ambiguous_config_fails_closed(
    tmp_path: Path, config: str, error: str
) -> None:
    path = tmp_path / "vehicle" / "cyclonedds.xml"
    path.parent.mkdir(parents=True)
    path.write_text(config, encoding="utf-8")
    result = cli._cyclonedds_domain_admissibility(tmp_path, 1)
    assert result["admissible"] is False
    assert result["error"] == error


def test_inadmissible_domain_stops_before_run_dir_or_launch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    _write_cyclonedds_config(tmp_path / "vehicle" / "cyclonedds.xml")
    launched = False

    def fail_if_called(*_args: object, **_kwargs: object) -> object:
        nonlocal launched
        launched = True
        raise AssertionError("launch helper must not run")

    monkeypatch.setattr(cli, "_run", fail_if_called)
    args = build_parser().parse_args(
        [
            "--repo-root",
            str(tmp_path),
            "run",
            "v2-uptake-smoke",
            "--run-id",
            "ineligible-domain",
            "--ros-domain-id",
            "233",
        ]
    )
    assert cli.run_v2_uptake_smoke(args) == 3
    result = json.loads(capsys.readouterr().out)
    assert result["outcome"] == "PRECONDITION_NOT_MET"
    assert result["cyclonedds"]["computed_domain_base"] == 65_650
    assert launched is False
    assert not (tmp_path / "analysis").exists()


def test_hash_drift_stops_before_run_dir_or_launch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = tmp_path / "vehicle" / "cyclonedds.xml"
    _write_cyclonedds_config(config)
    evidence = cli._cyclonedds_domain_admissibility(tmp_path, 1)
    config.write_text("<CycloneDDS />", encoding="utf-8")
    monkeypatch.setattr(cli, "_cyclonedds_domain_admissibility", lambda *_: evidence)
    monkeypatch.setattr(
        cli,
        "_run",
        lambda *_args, **_kwargs: pytest.fail("launch helper must not run"),
    )
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "v2-uptake-smoke", "--run-id", "hash-drift"]
    )
    assert cli.run_v2_uptake_smoke(args) == 3
    assert not (tmp_path / "analysis").exists()


def test_v2_uptake_runner_is_direct_fixture_free_and_private() -> None:
    source = Path(__file__).resolve().parents[1] / "aic_test" / "cli.py"
    text = source.read_text(encoding="utf-8")
    assert "c002ay0_pp_runtime_measurement.py" not in text
    assert "fixture_root" not in text
    assert "ros2 run state_lattice_overtake_planner" in text
    assert "ros2 run simple_pure_pursuit" in text
    assert "hybrid_control_mux" not in text[text.index("def run_v2_uptake_smoke"):]
    assert "readonly OBSERVER_LIFETIME_S=36" in text
    assert "readonly CAPTURE_LIFETIME_S=30" in text
    assert "${PRIVATE_INPUT}" in text and "${PRIVATE_OUTPUT}" in text
    assert "live_control_output_enabled:=false" in text
    assert "instant_control_enabled:=false" in text
    assert "controller_trackability_profile:=shadow_only" in text
    assert "state_lattice_v2_live_proposal_publish_enabled:=true" in text
    assert "kill -TERM --" in text
    assert "kill -KILL --" in text
    assert 'if kill -0 -- "-${pid}" 2>/dev/null; then' in text
    assert "for name in planner pure_pursuit; do" not in text
    assert "no_process_residue" in text
    assert "--params-file /aichallenge/workspace/install/state_lattice_overtake_planner" in text
    assert "own_vehicle_id:=d2" in text
    assert "state_lattice_v2_producer_instance_id:=7101" in text
    for string_parameter, value in (
        ("state_lattice_v2_expected_pp_producer_instance_id", "7201"),
        ("state_lattice_v2_expected_pp_session_id", "1"),
        ("state_lattice_v2_expected_producer_instance_id", "7101"),
        ("state_lattice_v2_base_attestation_producer_instance_id", "7201"),
        ("state_lattice_v2_base_attestation_session_id", "1"),
    ):
        assert f'-p "{string_parameter}:=\'{value}\'"' in text
    assert "input/race_armed:=${PRIVATE_INPUT}/race_armed" in text
    assert "for name in input_driver planner pure_pursuit observer" in text
    assert '"--name",' in text
    assert "--ready-file /evidence/observer_ready.json" in text
    assert "--sealed-file /evidence/observer_sealed.json" in text
    assert "--terminal-graph-file /evidence/observer_terminal_graph.json" in text
    assert "--first-fault-file /evidence/observer_first_fault.json" in text
    assert "--start-gate-file /evidence/capture_start.json" in text
    assert '--supervisor-pid "$$"' in text
    assert "--terminal-graph-timeout 5" in text
    assert "--primed-file /evidence/input_primed.json" in text
    assert "--pp-binding-cycle-file /evidence/pp_binding_cycle.json" in text
    assert '--expected-run-nonce "${AIC_TEST_RUN_NONCE}"' in text
    assert "state_lattice_v2_test_clock_readiness" not in text
    assert "output/state_lattice_v2_clock_ready" not in text
    assert "test -s /evidence/pp_binding_cycle.json" in text
    assert 'evidence.get("proposal_count_at_barrier") == 0' in text
    assert 'evidence.get("normal_status_integrity") is True' in text
    assert text.index('evidence.get("normal_status_integrity") is True') < text.index(
        "start_component planner"
    )
    assert "--prelaunch-permit-file /evidence/pp_prelaunch_permit.json" in text
    assert text.index("test -s /evidence/pp_prelaunch_permit.json") < text.index(
        "start_component planner"
    )
    prelaunch_gate = text[
        text.index('python3 - "${AIC_TEST_RUN_NONCE}" "${component_pid[observer]}"'):
        text.index("start_component planner")
    ]
    assert "from aic_test.cli" not in prelaunch_gate
    assert "import json" in prelaunch_gate and "import os" in prelaunch_gate
    assert 'require(stat.S_ISREG(permit_stat.st_mode), "permit_not_regular")' in prelaunch_gate
    assert 'require(stat.S_IMODE(permit_stat.st_mode) == 0o644, "permit_mode_invalid")' in prelaunch_gate
    assert 'require(request == {"schema_version": 1, "run_nonce": expected_nonce}, "request_invalid")' in prelaunch_gate
    assert 'require(isinstance(permit, dict) and set(permit) == required_keys, "permit_shape_invalid")' in prelaunch_gate
    assert 'require(permit["proposal_count"] == 0 and permit["status_fault"] is False, "permit_status_invalid")' in prelaunch_gate
    assert "assert " not in prelaunch_gate
    assert "for pid in (observer_pid, pp_pid):" in prelaunch_gate
    v2_source = text[text.index("def run_v2_uptake_smoke"):]
    assert "unset CYCLONEDDS_URI" not in v2_source
    assert "readonly EXPECTED_CYCLONEDDS_URI" in v2_source
    assert "readonly EXPECTED_CYCLONEDDS_SHA256" in v2_source
    assert "test -f /opt/autoware/cyclonedds.xml" in v2_source
    assert "test ! -L /opt/autoware/cyclonedds.xml" in v2_source
    assert "actual_cyclonedds_sha256" in v2_source
    assert v2_source.index("test \"${actual_cyclonedds_sha256}\"") < v2_source.index(
        "source /autoware/install/setup.bash"
    ) < v2_source.index("start_component observer")
    assert "attest_cyclonedds_after_sources()" in v2_source
    assert v2_source.index("source /aichallenge/workspace/install/setup.bash") < v2_source.rindex(
        "attest_cyclonedds_after_sources"
    ) < v2_source.index("start_component observer")
    assert "if source /autoware/install/setup.bash" not in v2_source
    assert "if source /aichallenge/workspace/install/setup.bash" not in v2_source
    assert v2_source.count("set +u\nsource /autoware/install/setup.bash\nbuiltin set -euo pipefail") == 1
    assert v2_source.count(
        "set +u\nsource /aichallenge/workspace/install/setup.bash\nbuiltin set -euo pipefail"
    ) == 1
    assert v2_source.count("builtin set -euo pipefail\narm_cleanup_traps\nverify_strict_mode") == 2
    assert "COLCON_TRACE" not in v2_source
    final_gate = v2_source.rindex(
        "verify_strict_mode\nverify_cleanup_traps\nattest_cyclonedds_after_sources"
    )
    assert final_gate < v2_source.index("start_component observer")
    assert "arm_cleanup_traps()" in v2_source
    assert "verify_cleanup_traps()" in v2_source
    assert v2_source.index("arm_cleanup_traps\nverify_cleanup_traps\n\nset +u") < v2_source.index(
        "source /autoware/install/setup.bash"
    )
    assert "set -euo pipefail" in text
    assert "ros2 param get /state_lattice_overtake_planner_node use_sim_time" in text
    assert "ros2 param get /simple_pure_pursuit_node use_sim_time" in text
    assert text.count('test "${planner_use_sim_time}" = "Boolean value is: True"') == 1
    assert text.count('test "${pp_use_sim_time}" = "Boolean value is: True"') == 1
    assert "/evidence/effective_time_parameters.json" in text
    assert '"effective_time_parameters_verified"' in text
    assert '"observer_bounded_stream_state"' in text
    assert '"observer_epoch_order_valid"' in text
    assert '--run-nonce "${AIC_TEST_RUN_NONCE}"' in text
    assert "test -s /evidence/input_primed.json" in text
    assert 'evidence.get("sample_count", 0) >= 3' in text
    assert 'evidence.get("race_armed") is False' in text
    assert text.index("start_component input_driver") < text.index(
        "start_component pure_pursuit"
    )
    assert text.index("test -s /evidence/pp_binding_cycle.json") < text.index(
        "start_component planner"
    )
    assert "--terminal-graph-file /evidence/observer_terminal_graph.json" in text
    assert "touch /evidence/capture_start.json" in text
    assert text.index("test -s /evidence/pp_binding_cycle.json") < text.index(
        "touch /evidence/capture_start.json"
    )
    assert text.index("start_component planner") < text.index(
        "touch /evidence/capture_start.json"
    )
    assert text.index("/evidence/effective_time_parameters.json") < text.index(
        "touch /evidence/capture_start.json"
    )
    assert "test -f /evidence/observer_terminal_graph.json" in text
    assert "test -s /evidence/observer.json" in text
    assert text.index("shutdown_component planner") < text.index(
        "shutdown_component pure_pursuit"
    )


def test_v2_generated_python_heredocs_compile_after_outer_string_evaluation() -> None:
    source = Path(__file__).resolve().parents[1] / "aic_test" / "cli.py"
    module = ast.parse(source.read_text(encoding="utf-8"))
    assignment = next(
        node
        for node in ast.walk(module)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "container_script"
            for target in node.targets
        )
    )
    expression = assignment.value
    while isinstance(expression, ast.Call) and isinstance(expression.func, ast.Attribute):
        expression = expression.func.value
    assert isinstance(expression, ast.Constant) and isinstance(expression.value, str)
    evaluated_script = expression.value

    heredoc_bodies: list[str] = []
    lines = evaluated_script.splitlines()
    for index, line in enumerate(lines):
        if not line.startswith("python3 ") or not line.endswith("<<'PY'"):
            continue
        end = lines.index("PY", index + 1)
        heredoc_bodies.append("\n".join(lines[index + 1 : end]) + "\n")

    assert len(heredoc_bodies) == 5
    for index, body in enumerate(heredoc_bodies):
        compile(body, f"generated-v2-heredoc-{index}.py", "exec")


def _run_v2_setup_boundary_fixture(
    tmp_path: Path, setup1_body: str, setup2_body: str = 'printf reached > "${SETUP2_MARKER}"\n'
) -> tuple[subprocess.CompletedProcess[str], Path, Path, Path, Path]:
    setup1 = tmp_path / "setup1.bash"
    setup2 = tmp_path / "setup2.bash"
    script = tmp_path / "launcher_fixture.bash"
    cleanup_marker = tmp_path / "cleanup"
    setup2_marker = tmp_path / "setup2_reached"
    observer_marker = tmp_path / "observer_reached"
    strict_marker = tmp_path / "strict_restored"
    setup1.write_text(setup1_body, encoding="utf-8")
    setup2.write_text(setup2_body, encoding="utf-8")

    cli_source = (Path(__file__).resolve().parents[1] / "aic_test" / "cli.py").read_text(
        encoding="utf-8"
    )
    script_start = cli_source.index('container_script = """') + len('container_script = """')
    script_end = cli_source.index('""".replace(', script_start)
    container_script = cli_source[script_start:script_end]
    boundary_start = container_script.index("cleanup_traps_armed=false")
    boundary_end = container_script.index("start_component observer", boundary_start)
    boundary = container_script[boundary_start:boundary_end]
    boundary = boundary.replace(
        "/autoware/install/setup.bash", shlex.quote(str(setup1))
    ).replace(
        "/aichallenge/workspace/install/setup.bash", shlex.quote(str(setup2))
    )

    script.write_text(
        f'''set -euo pipefail
readonly EXPECTED_CYCLONEDDS_URI="fixture://expected"
export CYCLONEDDS_URI="${{EXPECTED_CYCLONEDDS_URI}}"
readonly CLEANUP_MARKER={shlex.quote(str(cleanup_marker))}
readonly SETUP2_MARKER={shlex.quote(str(setup2_marker))}
readonly OBSERVER_MARKER={shlex.quote(str(observer_marker))}
readonly STRICT_MARKER={shlex.quote(str(strict_marker))}
finish() {{ : > "${{CLEANUP_MARKER}}"; }}
verify_strict_mode() {{
  case "$-" in *e*) ;; *) return 1;; esac
  [[ -o nounset ]] && [[ -o pipefail ]]
}}
attest_cyclonedds_after_sources() {{
  test "${{CYCLONEDDS_URI:-}}" = "${{EXPECTED_CYCLONEDDS_URI}}"
}}
{boundary}[[ "$-" == *e* ]] && [[ -o nounset ]] && [[ -o pipefail ]]
: > "${{STRICT_MARKER}}"
: > "${{OBSERVER_MARKER}}"
''',
        encoding="utf-8",
    )
    completed = subprocess.run(
        ["bash", str(script)], text=True, capture_output=True, check=False
    )
    return completed, cleanup_marker, setup2_marker, observer_marker, strict_marker


@pytest.mark.parametrize(
    "setup1_body",
    ["false\ntrue\n", "false | true\ntrue\n", "return 7\n"],
    ids=["false_then_true", "pipeline_false_then_true", "return_7"],
)
def test_v2_setup1_errexit_reaches_cleanup_without_setup2_or_observer(
    tmp_path: Path, setup1_body: str
) -> None:
    completed, cleanup_marker, setup2_marker, observer_marker, strict_marker = (
        _run_v2_setup_boundary_fixture(tmp_path, setup1_body)
    )
    assert completed.returncode != 0, completed.stderr
    assert cleanup_marker.exists()
    assert not setup2_marker.exists()
    assert not observer_marker.exists()
    assert not strict_marker.exists()


def test_v2_setup_optional_unset_succeeds_and_reaches_observer(tmp_path: Path) -> None:
    completed, cleanup_marker, setup2_marker, observer_marker, strict_marker = (
        _run_v2_setup_boundary_fixture(tmp_path, "unset OPTIONAL_SETUP_VALUE\ntrue\n")
    )
    assert completed.returncode == 0, completed.stderr
    assert cleanup_marker.exists()
    assert setup2_marker.exists()
    assert observer_marker.exists()
    assert strict_marker.exists()


def test_v2_setup_option_weakening_is_normalized_before_next_setup(tmp_path: Path) -> None:
    completed, cleanup_marker, setup2_marker, observer_marker, strict_marker = (
        _run_v2_setup_boundary_fixture(tmp_path, "set +e\nset +o pipefail\ntrue\n")
    )
    assert completed.returncode == 0, completed.stderr
    assert cleanup_marker.exists()
    assert setup2_marker.exists()
    assert observer_marker.exists()
    assert strict_marker.exists()


def test_v2_setup_dds_uri_mutation_fails_final_attestation(tmp_path: Path) -> None:
    completed, cleanup_marker, setup2_marker, observer_marker, strict_marker = (
        _run_v2_setup_boundary_fixture(tmp_path, "CYCLONEDDS_URI=fixture://mutated\ntrue\n")
    )
    assert completed.returncode != 0, completed.stderr
    assert cleanup_marker.exists()
    assert setup2_marker.exists()
    assert not observer_marker.exists()
    assert not strict_marker.exists()


def test_result_outcomes_are_stable() -> None:
    assert OUTCOMES == {
        "PASS",
        "ASSERTION_FAILED",
        "PRECONDITION_NOT_MET",
        "INFRASTRUCTURE_FAILED",
        "INVALID_EVIDENCE",
    }


def test_observer_forbidden_topics_cover_control_and_admin_authority() -> None:
    assert "/control/command/control_cmd" in FORBIDDEN_AUTHORITY_TOPICS
    assert "/control/command/actuation_cmd" in FORBIDDEN_AUTHORITY_TOPICS
    assert "/race/arm" in FORBIDDEN_AUTHORITY_TOPICS
    assert "/awsim/admin/command" in FORBIDDEN_AUTHORITY_TOPICS
    assert "/overtake/reference_override" in FORBIDDEN_AUTHORITY_TOPICS


def test_direct_input_driver_is_standalone_and_disarms_private_topic() -> None:
    driver = Path(__file__).resolve().parents[1] / "aic_test" / "v2_direct_input_driver.py"
    text = driver.read_text(encoding="utf-8")
    assert "c002ay0" not in text.lower()
    assert "create_publisher(Clock, \"/clock\"" in text
    assert 'f"{args.input_root}/kinematics"' in text
    assert 'f"{args.input_root}/race_armed"' in text
    assert "disarm_pub.publish(Bool(data=False))" in text
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in text
    assert "remaining_s > 0.25" in text
    assert "for _ in range(3)" in text
    assert 'trajectory.header.frame_id = "map"' in text
    assert "89633.30679316094" in text
    assert "43131.17568487456" in text
    assert "2.216058185742633" in text
    assert "for index in range(64)" in text
    assert 'opponent.vehicle_id = "d3"' in text
    assert "* 9.0" in text
    assert "now_s - started >= 1.0" in text
    assert "point.time_from_start.nanosec = index * 10_000_000" in text
    assert "mpc_health" not in text
    assert "_write_json_atomic" in text
    assert "CLOCK_PERIOD_S = 0.001" in text
    assert "--clock-period-s" not in text
    assert "ODOMETRY_PERIOD_S = 0.010" in text
    assert "TRAJECTORY_PERIOD_S = 0.050" in text
    assert "V2X_PERIOD_S = 0.050" in text
    assert "RACE_STATE_PERIOD_S = 0.050" in text
    assert "def _schedule_due(" in text and "now_s + period_s" in text
    assert "def _due_input_channels(" in text
    assert "publish_odometry(input_stamp_ns)" in text
    assert "publish_trajectory(input_stamp_ns)" in text
    assert "publish_v2x(input_stamp_ns)" in text
    assert "publish_race_state(armed=armed)" in text
    assert "_input_stamp_ns(clock_ns, last_published_clock_ns)" in text
    assert '"sample_count": len(priming_clock_ns)' in text
    assert '"race_armed": False' in text
    assert "gate_deadline" not in text
    assert "not _process_is_alive(args.supervisor_pid)" in text
    assert "terminal_deadline = time.monotonic() + args.terminal_graph_timeout" in text
    assert "while not terminal_file.is_file()" in text
    assert text.index("_write_json_atomic(\n                Path(args.capture_complete_file)") < text.index(
        "while not terminal_file.is_file()"
    ) < text.index("node.destroy_node()")
    assert "os.fchmod(stream.fileno(), 0o644)" in text
    assert '"capture_complete": True' in text


def test_direct_input_driver_scheduler_uses_independent_bounded_cadences() -> None:
    assert INPUT_PERIODS_S == {
        "odometry": 0.010,
        "trajectory": 0.050,
        "v2x": 0.050,
        "race_state": 0.050,
    }
    due, rescheduled = _schedule_due(1.0, 0.5, 0.001)
    assert due is True
    assert rescheduled == 1.001
    due, unchanged = _schedule_due(1.0, 1.001, 0.001)
    assert due is False
    assert unchanged == 1.001

    next_due = {channel: 0.0 for channel in INPUT_PERIODS_S}
    due_channels, next_due = _due_input_channels(0.0, next_due)
    assert due_channels == ("odometry", "trajectory", "v2x", "race_state")
    due_channels, next_due = _due_input_channels(0.010, next_due)
    assert due_channels == ("odometry",)
    due_channels, _ = _due_input_channels(0.050, next_due)
    assert due_channels == ("odometry", "trajectory", "v2x", "race_state")
    assert _input_stamp_ns(101, 100) == 101
    assert _input_stamp_ns(None, 101) == 101
    with pytest.raises(RuntimeError, match="input_publish_without_clock"):
        _input_stamp_ns(None, 0)
    _validate_clock_step(100, 100 + 10_000_000)
    with pytest.raises(RuntimeError, match="clock_step_out_of_bounds"):
        _validate_clock_step(100, 100 + 10_000_001)


def test_direct_input_driver_supervisor_liveness_is_fail_closed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("os.kill", lambda _pid, _signal: None)
    assert _process_is_alive(123) is True

    def missing(_pid: int, _signal: int) -> None:
        raise ProcessLookupError(errno.ESRCH, "missing")

    monkeypatch.setattr("os.kill", missing)
    assert _process_is_alive(123) is False


def test_direct_container_name_and_timeout_cleanup_are_scoped(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    assert cli._direct_container_name("Run / One") == "aic-test-v2-run---one"
    commands: list[list[str]] = []

    class Result:
        returncode = 0

    def fake_run(command: object, **_: object) -> Result:
        commands.append(list(command))
        if list(command)[:3] == ["docker", "container", "inspect"]:
            Result.returncode = 1
        else:
            Result.returncode = 0
        return Result()

    monkeypatch.setattr(cli, "_run", fake_run)
    cleanup = cli._cleanup_direct_container(tmp_path, "aic-test-v2-run-one")
    assert cleanup["absent_after_cleanup"] is True
    assert commands == [
        ["docker", "stop", "--time", "5", "aic-test-v2-run-one"],
        ["docker", "rm", "--force", "aic-test-v2-run-one"],
        ["docker", "container", "inspect", "aic-test-v2-run-one"],
    ]


def test_result_is_written_atomically_and_rejects_unknown_outcome(
    tmp_path: Path,
) -> None:
    path = tmp_path / "result.json"
    write_json_atomic(
        path,
        {
            "run_id": "fixture-001",
            "scenario": "control-smoke",
            "outcome": "PASS",
        },
    )
    assert read_result(path)["run_id"] == "fixture-001"
    assert not list(tmp_path.glob("*.tmp"))

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["outcome"] = "UNKNOWN"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="unknown result outcome"):
        read_result(path)


@pytest.mark.parametrize(
    "writer,payload",
    [
        (
            _write_host_readable_json_atomic,
            {"run_id": "fixture-001", "outcome": "PASS"},
        ),
        (
            _write_json_atomic,
            {"schema_version": 1, "run_nonce": "fixture-001", "capture_complete": True},
        ),
    ],
)
def test_v2_handoff_json_writers_are_atomic_host_readable_0644(
    tmp_path: Path, writer, payload: dict[str, object]
) -> None:
    path = tmp_path / "handoff.json"
    writer(path, payload)
    assert json.loads(path.read_text(encoding="utf-8"))
    assert stat.S_IMODE(path.stat().st_mode) == 0o644
    assert not list(tmp_path.glob("*.tmp"))


def test_v2_handoff_writer_failure_leaves_no_partial_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    path = tmp_path / "handoff.json"

    def fail_fsync(_file_descriptor: int) -> None:
        raise OSError("fsync failed")

    monkeypatch.setattr(os, "fsync", fail_fsync)
    with pytest.raises(OSError, match="fsync failed"):
        _write_json_atomic(path, {"schema_version": 1})
    assert not path.exists()
    assert not list(tmp_path.glob("*.tmp"))


def test_v2_handoff_reader_rejects_missing_or_mode_mismatched_json(
    tmp_path: Path,
) -> None:
    path = tmp_path / "handoff.json"
    assert _read_host_readable_json(path) is None
    path.write_text('{"schema_version": 1}\n', encoding="utf-8")
    os.chmod(path, 0o600)
    assert _read_host_readable_json(path) is None
    os.chmod(path, 0o644)
    assert _read_host_readable_json(path) == {"schema_version": 1}


def _write_v2_uptake_prearm_inputs(
    root: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    paths = {
        "archive_sha256": root / "submit" / "aichallenge_submit.tar.gz",
        "planner_source_sha256": root / "aichallenge/workspace/src/aichallenge_submit/state_lattice_overtake_planner/src/state_lattice_overtake_planner_node.cpp",
        "pp_source_sha256": root / "aichallenge/workspace/src/aichallenge_submit/simple_pure_pursuit/src/simple_pure_pursuit.cpp",
        "planner_binary_sha256": root / "aichallenge/workspace/install/state_lattice_overtake_planner/lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node",
        "pp_binary_sha256": root / "aichallenge/workspace/install/simple_pure_pursuit/lib/simple_pure_pursuit/simple_pure_pursuit",
        "observer_sha256": root / "tools/aic_test/aic_test/v2_uptake_observer.py",
        "driver_sha256": root / "tools/aic_test/aic_test/v2_direct_input_driver.py",
    }
    for name, path in paths.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(name, encoding="utf-8")
        monkeypatch.setenv(cli.V2_UPTAKE_EXPECTED_HASH_ENV[name], hashlib.sha256(path.read_bytes()).hexdigest())
    monkeypatch.setenv(cli.V2_UPTAKE_IMAGE_REFERENCE_ENV, "aichallenge-2025-eval@sha256:" + "1" * 64)
    monkeypatch.setenv(cli.V2_UPTAKE_IMAGE_ID_ENV, "sha256:" + "2" * 64)
    monkeypatch.setenv(cli.V2_UPTAKE_EXECUTION_ID_ENV, "reviewed-edge-1")
    handoff = root / "reviewed-handoff.json"
    handoff.write_text("handoff\n", encoding="utf-8")
    os.chmod(handoff, 0o644)
    handoff_sha = hashlib.sha256(handoff.read_bytes()).hexdigest()
    review = {
        "schema_version": 1,
        "external_review_gate": {
            "completed": True, "selected_model": "GPT-5.6 Sol Pro",
            "terminal_status": "COMPLETED", "workflow_advance_allowed": True,
            "reviewed_execution_id": "execution-1", "reviewed_handoff_sha256": handoff_sha,
            "reviewed_handoff_path": str(handoff),
        },
        "current_owning_edge": {
            "id": "runtime-arm-edge-1", "permits_runtime_arm": True,
            "authorized_runtime_execution_id": "reviewed-edge-1",
            "runtime_budget": {"fresh_runtime_max": 1, "fresh_runtime_used": 0,
                               "retry_allowed": False},
        },
    }
    state = root / "review-state.json"
    state.write_text(json.dumps(review), encoding="utf-8")
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEW_STATE_PATH_ENV, str(state))
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEW_STATE_SHA_ENV, hashlib.sha256(state.read_bytes()).hexdigest())
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEWED_EXECUTION_ID_ENV, "execution-1")
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEWED_HANDOFF_SHA_ENV, handoff_sha)
    monkeypatch.setenv(cli.V2_UPTAKE_CURRENT_EDGE_ID_ENV, "runtime-arm-edge-1")
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEWED_HANDOFF_PATH_ENV, str(handoff))


def test_v2_static_prearm_rejects_invalid_hash_without_docker(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    monkeypatch.setenv(cli.V2_UPTAKE_EXPECTED_HASH_ENV["archive_sha256"], "invalid")
    prearm, error = cli._v2_uptake_static_prearm(tmp_path)
    assert prearm is None
    assert error == "expected_archive_sha256_invalid"


def test_v2_runner_rejects_python_optimized_before_repository_side_effects(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    state = tmp_path / "review-state.json"
    before = state.read_bytes()
    environment = os.environ.copy()
    environment.pop("PYTHONOPTIMIZE", None)
    environment["PYTHONPATH"] = str(Path(__file__).resolve().parents[1])
    command = [
        sys.executable, "-O", "-c",
        "from aic_test.cli import main; raise SystemExit(main(['--repo-root', r'%s', 'run', 'v2-uptake-smoke', '--run-id', 'optimized']))" % tmp_path,
    ]
    completed = subprocess.run(command, env=environment, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               check=False)
    assert completed.returncode != 0
    assert "python_optimize_enabled" in completed.stdout
    assert state.read_bytes() == before
    assert not (tmp_path / "analysis").exists()


def test_v2_static_prearm_binds_expected_and_actual_hashes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    prearm, error = cli._v2_uptake_static_prearm(tmp_path)
    assert error is None and prearm is not None
    assert prearm["expected_image_reference"].startswith("aichallenge-2025-eval@sha256:")
    assert all(prearm["actual_hashes"][name] == prearm["expected_hashes"][name]
               for name in prearm["actual_hashes"])


def _set_deferred_connection_unavailable_gate(
    root: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    state = root / "review-state.json"
    payload = json.loads(state.read_text(encoding="utf-8"))
    handoff_sha = os.environ[cli.V2_UPTAKE_REVIEWED_HANDOFF_SHA_ENV]
    packet = root / "review-packet.json"
    packet.write_text('{"packet":"immutable"}\n', encoding="utf-8")
    os.chmod(packet, 0o644)
    packet_sha = hashlib.sha256(packet.read_bytes()).hexdigest()
    fresh_result = root / "fresh-04-result.json"
    fresh_result_payload = {
        "outcome": "INFRASTRUCTURE_FAILED",
        "prearm": {"execution_id": "m4-v2-planner-generation-fresh-runtime-04"},
        "review_state_budget_transition": {"before_sha256": "a", "after_sha256": "b"},
        "one_shot_budget_guard": "/tmp/fresh-04-guard.json",
        "docker_create_returncode": 0,
        "observer": None,
        "container_cleanup": {"absent_after_cleanup": True},
    }
    fresh_result.write_text(json.dumps(fresh_result_payload), encoding="utf-8")
    os.chmod(fresh_result, 0o644)
    fresh_result_sha = hashlib.sha256(fresh_result.read_bytes()).hexdigest()
    fresh_handoff = root / "fresh-04-handoff.json"
    fresh_handoff_payload = {
        "execution_id": "m4-v2-planner-generation-fresh-runtime-04",
        "runtime": {
            "outcome": "INFRASTRUCTURE_FAILED",
            "first_reason": "image_planner_binary_sha256_mismatch",
            "capture_started": False,
        },
        "runtime_budget": {"fresh_runtime_max": 1, "fresh_runtime_used": 1, "retry_allowed": False},
        "binding": {"result_path": str(fresh_result), "result_sha256": fresh_result_sha},
    }
    fresh_handoff.write_text(json.dumps(fresh_handoff_payload), encoding="utf-8")
    os.chmod(fresh_handoff, 0o644)
    fresh_04_evidence = {
        "handoff_path": str(fresh_handoff),
        "handoff_sha256": hashlib.sha256(fresh_handoff.read_bytes()).hexdigest(),
        "execution_id": "m4-v2-planner-generation-fresh-runtime-04",
        "outcome": "INFRASTRUCTURE_FAILED",
        "first_reason": "image_planner_binary_sha256_mismatch",
        "capture_started": False,
        "runtime_budget": {"fresh_runtime_max": 1, "fresh_runtime_used": 1, "retry_allowed": False},
        "result_path": str(fresh_result), "result_sha256": fresh_result_sha,
    }
    fresh_06_result = root / "fresh-06-result.json"
    fresh_06_result_payload = dict(fresh_result_payload)
    fresh_06_result_payload["prearm"] = {"execution_id": "m4-v2-planner-generation-fresh-runtime-06"}
    fresh_06_result.write_text(json.dumps(fresh_06_result_payload), encoding="utf-8")
    os.chmod(fresh_06_result, 0o644)
    fresh_06_result_sha = hashlib.sha256(fresh_06_result.read_bytes()).hexdigest()
    fresh_06_handoff = root / "fresh-06-handoff.json"
    fresh_06_handoff_payload = dict(fresh_handoff_payload)
    fresh_06_handoff_payload["execution_id"] = "m4-v2-planner-generation-fresh-runtime-06"
    fresh_06_handoff_payload["runtime"] = {
        **fresh_06_handoff_payload["runtime"],
        "first_reason": "image_pp_binary_sha256_mismatch",
    }
    fresh_06_handoff_payload["binding"] = {"result_path": str(fresh_06_result), "result_sha256": fresh_06_result_sha}
    fresh_06_handoff.write_text(json.dumps(fresh_06_handoff_payload), encoding="utf-8")
    os.chmod(fresh_06_handoff, 0o644)
    fresh_06_evidence = {**fresh_04_evidence,
        "handoff_path": str(fresh_06_handoff),
        "handoff_sha256": hashlib.sha256(fresh_06_handoff.read_bytes()).hexdigest(),
        "execution_id": "m4-v2-planner-generation-fresh-runtime-06",
        "first_reason": "image_pp_binary_sha256_mismatch",
        "result_path": str(fresh_06_result), "result_sha256": fresh_06_result_sha,
    }
    monkeypatch.setenv(cli.V2_UPTAKE_EXECUTION_ID_ENV, "m4-v2-planner-generation-fresh-runtime-07")
    payload["current_owning_edge"]["authorized_runtime_execution_id"] = "m4-v2-planner-generation-fresh-runtime-07"
    authorization = {
        "user_authorization": "EXPLICIT",
        "authorization_record_path": str(root / "runtime-authorization.json"),
        "authorization_record_sha256": "0" * 64,
        "authorized_runtime_execution_id": "m4-v2-planner-generation-fresh-runtime-07",
        "ros_domain_id": 231,
        "expected_image_reference": os.environ[cli.V2_UPTAKE_IMAGE_REFERENCE_ENV],
        "expected_image_id": os.environ[cli.V2_UPTAKE_IMAGE_ID_ENV],
        "expected_hashes": {
            name: os.environ[environment_name].lower()
            for name, environment_name in cli.V2_UPTAKE_EXPECTED_HASH_ENV.items()
        },
        "outer_timeout_s": 90,
        "runtime_timeout_s": 90,
        "fresh_runtime_max": 1,
        "fresh_runtime_used": 0,
        "retry_allowed": False,
        "prior_consumed_attempts": [fresh_04_evidence, fresh_06_evidence],
    }
    record = root / "runtime-authorization.json"
    # The record's digest is intentionally over the same immutable binding with
    # the digest field blanked, avoiding an impossible self-referential hash.
    authorization_record = dict(authorization)
    authorization_record["authorization_record_sha256"] = ""
    record.write_text(json.dumps(authorization_record), encoding="utf-8")
    os.chmod(record, 0o644)
    authorization["authorization_record_sha256"] = hashlib.sha256(record.read_bytes()).hexdigest()
    payload["external_review_gate"] = {
        "completed": False,
        "terminal_status": "DEFERRED_CONNECTION_UNAVAILABLE",
        "workflow_advance_allowed": True,
        "reviewed_execution_id": "execution-1",
        "reviewed_handoff_sha256": handoff_sha,
        "reviewed_handoff_path": str(root / "reviewed-handoff.json"),
        "external_review_status": "DEFERRED_CONNECTION_UNAVAILABLE",
        "review_packet_path": str(packet),
        "review_packet_sha256": packet_sha,
        "connection_attempts": [
            {"timestamp": "2026-08-04T00:00:00Z", "outcome": "CONNECTION_UNAVAILABLE", "timeout_s": 30},
            {"timestamp": "2026-08-04T00:01:00Z", "outcome": "CONNECTION_UNAVAILABLE", "timeout_s": 30},
            {"timestamp": "2026-08-04T00:02:00Z", "outcome": "CONNECTION_UNAVAILABLE", "timeout_s": 30},
        ],
        "terminal_connection_error": None,
        "local_safety_review": {"disposition": "GO", "blocker_count": 0, "major_count": 0},
        "retrospective_review": {
            "status": "QUEUED", "review_packet_path": str(packet),
            "review_packet_sha256": packet_sha, "handoff_sha256": handoff_sha,
        },
        "authorization": authorization,
    }
    state.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEW_STATE_SHA_ENV, hashlib.sha256(state.read_bytes()).hexdigest())


def test_v2_static_prearm_accepts_strict_deferred_connection_unavailable_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    _set_deferred_connection_unavailable_gate(tmp_path, monkeypatch)
    state = tmp_path / "review-state.json"
    payload = json.loads(state.read_text(encoding="utf-8"))
    authorization = payload["external_review_gate"]["authorization"]
    record = Path(authorization["authorization_record_path"])
    authorization_record = dict(authorization)
    authorization_record["authorization_record_sha256"] = ""
    record.write_text(json.dumps(authorization_record), encoding="utf-8")
    os.chmod(record, 0o644)
    authorization["authorization_record_sha256"] = hashlib.sha256(record.read_bytes()).hexdigest()
    state.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEW_STATE_SHA_ENV, hashlib.sha256(state.read_bytes()).hexdigest())
    prearm, error = cli._v2_uptake_static_prearm(tmp_path, 231, 90.0)
    assert error is None and prearm is not None
    assert prearm["external_review_status"] == "DEFERRED_CONNECTION_UNAVAILABLE"


@pytest.mark.parametrize("mutation", ["completed", "approved", "attempts", "attempts_with_terminal_error", "safety", "queue", "packet", "missing_auth", "execution", "domain", "hash", "timeout", "retry", "prior_missing", "prior_duplicate", "prior_reordered", "fresh04_path", "fresh04_sha", "fresh04_execution", "fresh04_outcome", "fresh04_budget", "fresh04_retry", "fresh06_path", "fresh06_sha", "fresh06_reason", "fresh06_budget"])
def test_v2_static_prearm_rejects_weak_or_mislabeled_deferred_gate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mutation: str
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    _set_deferred_connection_unavailable_gate(tmp_path, monkeypatch)
    state = tmp_path / "review-state.json"
    payload = json.loads(state.read_text(encoding="utf-8"))
    gate = payload["external_review_gate"]
    if mutation == "completed":
        gate["completed"] = True
    elif mutation == "approved":
        gate["terminal_status"] = "APPROVED"
    elif mutation == "attempts":
        gate["connection_attempts"] = gate["connection_attempts"][:2]
    elif mutation == "attempts_with_terminal_error":
        gate["connection_attempts"] = gate["connection_attempts"][:2]
        gate["terminal_connection_error"] = "browser profile already in use"
    elif mutation == "safety":
        gate["local_safety_review"]["major_count"] = 1
    elif mutation == "queue":
        gate["retrospective_review"]["status"] = "COMPLETED"
    elif mutation == "missing_auth":
        gate.pop("authorization")
    elif mutation == "execution":
        gate["authorization"]["authorized_runtime_execution_id"] = "wrong"
    elif mutation == "domain":
        gate["authorization"]["ros_domain_id"] = 227
    elif mutation == "hash":
        gate["authorization"]["expected_hashes"]["archive_sha256"] = "0" * 64
    elif mutation == "timeout":
        gate["authorization"]["outer_timeout_s"] = 91
    elif mutation == "retry":
        gate["authorization"]["retry_allowed"] = True
    elif mutation == "prior_missing":
        gate["authorization"]["prior_consumed_attempts"] = gate["authorization"]["prior_consumed_attempts"][:1]
    elif mutation == "prior_duplicate":
        gate["authorization"]["prior_consumed_attempts"][1] = gate["authorization"]["prior_consumed_attempts"][0]
    elif mutation == "prior_reordered":
        gate["authorization"]["prior_consumed_attempts"].reverse()
    elif mutation == "fresh04_path":
        gate["authorization"]["prior_consumed_attempts"][0]["handoff_path"] = str(tmp_path / "wrong.json")
    elif mutation == "fresh04_sha":
        gate["authorization"]["prior_consumed_attempts"][0]["handoff_sha256"] = "0" * 64
    elif mutation == "fresh04_execution":
        gate["authorization"]["prior_consumed_attempts"][0]["execution_id"] = "wrong"
    elif mutation == "fresh04_outcome":
        gate["authorization"]["prior_consumed_attempts"][0]["outcome"] = "PASS"
    elif mutation == "fresh04_budget":
        gate["authorization"]["prior_consumed_attempts"][0]["runtime_budget"]["fresh_runtime_used"] = 0
    elif mutation == "fresh04_retry":
        gate["authorization"]["prior_consumed_attempts"][0]["runtime_budget"]["retry_allowed"] = True
    elif mutation == "fresh06_path":
        gate["authorization"]["prior_consumed_attempts"][1]["handoff_path"] = str(tmp_path / "wrong-fresh06.json")
    elif mutation == "fresh06_sha":
        gate["authorization"]["prior_consumed_attempts"][1]["handoff_sha256"] = "0" * 64
    elif mutation == "fresh06_reason":
        gate["authorization"]["prior_consumed_attempts"][1]["first_reason"] = "image_planner_binary_sha256_mismatch"
    elif mutation == "fresh06_budget":
        gate["authorization"]["prior_consumed_attempts"][1]["runtime_budget"]["fresh_runtime_used"] = 0
    else:
        (tmp_path / "review-packet.json").write_text('{"packet":"drift"}\n', encoding="utf-8")
    state.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEW_STATE_SHA_ENV, hashlib.sha256(state.read_bytes()).hexdigest())
    assert cli._v2_uptake_static_prearm(tmp_path, 231, 90.0)[1] == "review_state_gate_not_satisfied"


@pytest.mark.parametrize("requested_timeout_s", [89.0, 91.0, float("nan"), float("inf")])
def test_v2_static_prearm_rejects_deferred_runtime_timeout_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, requested_timeout_s: float
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    _set_deferred_connection_unavailable_gate(tmp_path, monkeypatch)
    assert cli._v2_uptake_static_prearm(
        tmp_path, 231, requested_timeout_s
    )[1] == "review_state_gate_not_satisfied"


def test_v2_static_prearm_rejects_review_state_hash_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    (tmp_path / "review-state.json").write_text("{}", encoding="utf-8")
    prearm, error = cli._v2_uptake_static_prearm(tmp_path)
    assert prearm is None
    assert error == "review_state_hash_mismatch"


def test_v2_static_prearm_rejects_flat_synthetic_review_state(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    state = tmp_path / "review-state.json"
    state.write_text(json.dumps({"external_review_gate": "completed"}), encoding="utf-8")
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEW_STATE_SHA_ENV, hashlib.sha256(state.read_bytes()).hexdigest())
    prearm, error = cli._v2_uptake_static_prearm(tmp_path)
    assert prearm is None
    assert error == "review_state_schema_invalid"


@pytest.mark.parametrize("schema_version", [None, 2, True])
def test_v2_static_prearm_rejects_missing_wrong_or_boolean_state_schema(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, schema_version: object
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    state = tmp_path / "review-state.json"
    payload = json.loads(state.read_text(encoding="utf-8"))
    if schema_version is None:
        payload.pop("schema_version")
    else:
        payload["schema_version"] = schema_version
    state.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setenv(cli.V2_UPTAKE_REVIEW_STATE_SHA_ENV, hashlib.sha256(state.read_bytes()).hexdigest())
    assert cli._v2_uptake_static_prearm(tmp_path)[1] == "review_state_schema_invalid"


def test_v2_state_budget_transition_fsyncs_parent_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    prearm, error = cli._v2_uptake_static_prearm(tmp_path)
    assert error is None and prearm is not None
    original_fsync = os.fsync
    fsynced: list[int] = []

    def track_fsync(fd: int) -> None:
        fsynced.append(fd)
        original_fsync(fd)

    monkeypatch.setattr(os, "fsync", track_fsync)
    before, after = cli._consume_v2_uptake_state_budget(prearm)
    assert before is not None and after is not None and before != after
    assert len(fsynced) >= 2


def test_host_readable_binding_writer_fsyncs_parent_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    original_fsync = os.fsync
    calls: list[int] = []
    def track(fd: int) -> None:
        calls.append(fd); original_fsync(fd)
    monkeypatch.setattr(os, "fsync", track)
    cli._write_host_readable_json_atomic(tmp_path / "binding.json", {"run_id": "x"})
    assert len(calls) >= 2


def test_v2_one_shot_budget_is_durable_and_never_restored(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    prearm, error = cli._v2_uptake_static_prearm(tmp_path)
    assert error is None and prearm is not None
    guard, error = cli._consume_v2_uptake_one_shot_budget(tmp_path, "one", prearm)
    assert error is None and guard is not None and guard.is_file()
    assert json.loads(guard.read_text(encoding="utf-8"))["run_id"] == "one"
    second_guard, second_error = cli._consume_v2_uptake_one_shot_budget(tmp_path, "two", prearm)
    assert second_guard is None
    assert second_error == "v2_uptake_one_shot_budget_already_consumed"
    monkeypatch.setenv(cli.V2_UPTAKE_EXECUTION_ID_ENV, "reviewed-edge-2")
    future_prearm, future_error = cli._v2_uptake_static_prearm(tmp_path)
    assert future_prearm is None
    assert future_error == "review_state_gate_not_satisfied"


def _planner_binary_attestation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, objects: dict[str, tuple[str, str]]
) -> tuple[str | None, dict[str, object], str | None, list[list[str]]]:
    class Result:
        def __init__(self, returncode: int = 0) -> None:
            self.returncode, self.stdout = returncode, ""

    commands: list[list[str]] = []

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        commands.append(rendered)
        source = rendered[2].split(":", 1)[1]
        target = Path(rendered[-1])
        entry = objects.get(source)
        if entry is None:
            return Result(1)
        kind, value = entry
        if kind == "symlink":
            os.symlink(value, target)
        elif kind == "regular":
            target.write_text(value, encoding="utf-8")
        elif kind == "directory":
            target.mkdir()
        else:
            raise AssertionError(f"unknown fixture object kind: {kind}")
        return Result()

    monkeypatch.setattr(cli, "_run", fake_run)
    destination = tmp_path / "image-attestation"
    destination.mkdir()
    expected = hashlib.sha256(b"planner").hexdigest()
    actual, observation, error = cli._container_planner_binary_attestation(
        tmp_path, "created", destination, expected
    )
    return actual, observation, error, commands


def test_planner_binary_attestation_resolves_absolute_install_symlink(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    requested = str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node")
    actual, observation, error, _ = _planner_binary_attestation(
        tmp_path, monkeypatch, {
            requested: ("symlink", str(cli.PLANNER_BUILD_TARGET)),
            str(cli.PLANNER_BUILD_TARGET): ("regular", "planner"),
        }
    )
    assert error is None
    assert actual == hashlib.sha256(b"planner").hexdigest()
    assert observation == {
        "requested_path": requested, "initial_type": "symlink",
        "normalized_chain": [requested, str(cli.PLANNER_BUILD_TARGET)],
        "copy_outcomes": [
            {"normalized_path": requested, "returncode": 0, "local_type": "symlink"},
            {"normalized_path": str(cli.PLANNER_BUILD_TARGET), "returncode": 0,
             "local_type": "regular"},
        ],
        "resolved_path": str(cli.PLANNER_BUILD_TARGET), "terminal_type": "regular",
        "size_bytes": 7, "actual_sha256": actual,
    }


def test_planner_binary_attestation_accepts_direct_regular_install_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    requested = str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node")
    actual, observation, error, _ = _planner_binary_attestation(
        tmp_path, monkeypatch, {requested: ("regular", "planner")}
    )
    assert error is None and actual == observation["actual_sha256"]
    assert observation["initial_type"] == observation["terminal_type"] == "regular"
    assert observation["resolved_path"] == requested


def test_planner_binary_attestation_resolves_relative_install_symlink(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    requested = str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node")
    resolved = str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/real_node")
    actual, observation, error, _ = _planner_binary_attestation(
        tmp_path, monkeypatch, {
            requested: ("symlink", "real_node"), resolved: ("regular", "planner"),
        }
    )
    assert error is None and actual is not None
    assert observation["normalized_chain"] == [requested, resolved]


def test_planner_binary_attestation_persists_actual_hash_on_numeric_mismatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    requested = str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node")
    actual, observation, error, _ = _planner_binary_attestation(
        tmp_path, monkeypatch, {requested: ("regular", "different")}
    )
    assert actual is None and error == "image_planner_binary_sha256_mismatch"
    assert observation["actual_sha256"] == hashlib.sha256(b"different").hexdigest()
    assert observation["size_bytes"] == len("different")


@pytest.mark.parametrize(
    ("objects", "expected_error"),
    [
        ({}, "image_planner_binary_resolution_failed:copy_failed"),
        ({
            str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"): ("symlink", "loop"),
            str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/loop"): ("symlink", "state_lattice_overtake_planner_node"),
        }, "image_planner_binary_resolution_failed:symlink_cycle"),
        ({
            str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"): ("symlink", "/tmp/escape"),
        }, "image_planner_binary_resolution_failed:path_escape"),
        ({
            str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"): ("directory", ""),
        }, "image_planner_binary_resolution_failed:terminal_not_regular"),
    ],
    ids=["broken", "cycle", "escape", "non_regular"],
)
def test_planner_binary_attestation_rejects_resolution_or_type_failures(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
    objects: dict[str, tuple[str, str]], expected_error: str,
) -> None:
    actual, observation, error, commands = _planner_binary_attestation(
        tmp_path, monkeypatch, objects
    )
    assert actual is None and error == expected_error
    assert all(command[:3] != ["docker", "start", "-a"] for command in commands)
    if expected_error.endswith("copy_failed"):
        assert observation["initial_type"] == "unobserved"
        assert observation["copy_outcomes"] == [{
            "normalized_path": observation["requested_path"], "returncode": 1,
        }]
    else:
        assert observation["terminal_type"] != "regular"


def test_planner_binary_attestation_rejects_excessive_symlink_hops(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    requested = str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node")
    objects: dict[str, tuple[str, str]] = {}
    current = requested
    for index in range(cli.PLANNER_BINARY_ATTESTATION_MAX_HOPS + 1):
        next_path = str(cli.PLANNER_INSTALL_ROOT / f"hop_{index}")
        objects[current] = ("symlink", next_path)
        current = next_path
    actual, observation, error, commands = _planner_binary_attestation(
        tmp_path, monkeypatch, objects
    )
    assert actual is None
    assert error == "image_planner_binary_resolution_failed:too_many_symlink_hops"
    assert len(observation["normalized_chain"]) == cli.PLANNER_BINARY_ATTESTATION_MAX_HOPS + 1
    assert all(command[:3] != ["docker", "start", "-a"] for command in commands)


def _pp_binary_attestation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, objects: dict[str, tuple[str, str]],
    expected_text: str = "pure_pursuit",
) -> tuple[str | None, dict[str, object], str | None, list[list[str]]]:
    class Result:
        def __init__(self, returncode: int = 0) -> None:
            self.returncode, self.stdout = returncode, ""

    commands: list[list[str]] = []

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command); commands.append(rendered)
        source = rendered[2].split(":", 1)[1]; target = Path(rendered[-1])
        entry = objects.get(source)
        if entry is None:
            return Result(1)
        kind, value = entry
        if kind == "symlink": os.symlink(value, target)
        elif kind == "regular": target.write_text(value, encoding="utf-8")
        elif kind == "directory": target.mkdir()
        else: raise AssertionError(kind)
        return Result()

    monkeypatch.setattr(cli, "_run", fake_run)
    destination = tmp_path / "image-attestation"; destination.mkdir()
    actual, observation, error = cli._container_pp_binary_attestation(
        tmp_path, "created", destination, hashlib.sha256(expected_text.encode()).hexdigest()
    )
    return actual, observation, error, commands


def test_pp_binary_attestation_resolves_direct_and_allowlisted_symlinks(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    requested = str(cli.PP_INSTALL_ROOT / "lib/simple_pure_pursuit/simple_pure_pursuit")
    for index, objects in enumerate((
        {requested: ("regular", "pure_pursuit")},
        {requested: ("symlink", str(cli.PP_BUILD_TARGET)), str(cli.PP_BUILD_TARGET): ("regular", "pure_pursuit")},
        {requested: ("symlink", "real_pp"), str(cli.PP_INSTALL_ROOT / "lib/simple_pure_pursuit/real_pp"): ("regular", "pure_pursuit")},
    )):
        case = tmp_path / str(index); case.mkdir()
        actual, observation, error, commands = _pp_binary_attestation(case, monkeypatch, objects)
        assert error is None and actual == observation["actual_sha256"]
        assert all(command[:3] != ["docker", "start", "-a"] for command in commands)


@pytest.mark.parametrize(
    ("objects", "expected_error"),
    [
        ({}, "image_pp_binary_resolution_failed:copy_failed"),
        ({str(cli.PP_INSTALL_ROOT / "lib/simple_pure_pursuit/simple_pure_pursuit"): ("symlink", "/tmp/escape")}, "image_pp_binary_resolution_failed:path_escape"),
        ({str(cli.PP_INSTALL_ROOT / "lib/simple_pure_pursuit/simple_pure_pursuit"): ("directory", "")}, "image_pp_binary_resolution_failed:terminal_not_regular"),
        ({str(cli.PP_INSTALL_ROOT / "lib/simple_pure_pursuit/simple_pure_pursuit"): ("symlink", "loop"), str(cli.PP_INSTALL_ROOT / "lib/simple_pure_pursuit/loop"): ("symlink", "simple_pure_pursuit")}, "image_pp_binary_resolution_failed:symlink_cycle"),
    ], ids=["copy_fail", "escape", "nonregular", "cycle"],
)
def test_pp_binary_attestation_rejects_unsafe_resolution_before_start(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, objects: dict[str, tuple[str, str]], expected_error: str,
) -> None:
    actual, _, error, commands = _pp_binary_attestation(tmp_path, monkeypatch, objects)
    assert actual is None and error == expected_error
    assert all(command[:3] != ["docker", "start", "-a"] for command in commands)


def test_pp_binary_attestation_rejects_hop_limit_and_persists_numeric_mismatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    requested = str(cli.PP_INSTALL_ROOT / "lib/simple_pure_pursuit/simple_pure_pursuit")
    hops: dict[str, tuple[str, str]] = {}
    current = requested
    for index in range(cli.PLANNER_BINARY_ATTESTATION_MAX_HOPS + 1):
        next_path = str(cli.PP_INSTALL_ROOT / f"hop_{index}")
        hops[current] = ("symlink", next_path); current = next_path
    actual, _, error, commands = _pp_binary_attestation(tmp_path, monkeypatch, hops)
    assert actual is None and error == "image_pp_binary_resolution_failed:too_many_symlink_hops"
    assert all(command[:3] != ["docker", "start", "-a"] for command in commands)

    mismatch_dir = tmp_path / "mismatch"; mismatch_dir.mkdir()
    actual, observation, error, commands = _pp_binary_attestation(
        mismatch_dir, monkeypatch, {requested: ("regular", "different")}
    )
    assert actual is None and error == "image_pp_binary_sha256_mismatch"
    assert observation["actual_sha256"] == hashlib.sha256(b"different").hexdigest()
    assert all(command[:3] != ["docker", "start", "-a"] for command in commands)


def test_v2_image_hash_mismatch_fails_before_start(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    prearm, error = cli._v2_uptake_static_prearm(tmp_path)
    assert error is None and prearm is not None
    commands: list[list[str]] = []

    class Result:
        returncode = 0
        stdout = ""

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        commands.append(rendered)
        Path(rendered[-1]).write_text("mismatch", encoding="utf-8")
        return Result()

    monkeypatch.setattr(cli, "_run", fake_run)
    hashes, observation, pp_observation, mismatch = cli._v2_uptake_image_hashes(
        tmp_path, "created", tmp_path, prearm
    )
    assert hashes is None
    assert observation == {}
    assert pp_observation == {}
    assert mismatch == "image_planner_source_sha256_mismatch"
    assert all(command[:3] != ["docker", "start", "-a"] for command in commands)


def test_v2_planner_attestation_failure_never_resets_budget_or_starts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    config = tmp_path / "vehicle/cyclonedds.xml"
    config.parent.mkdir(parents=True)
    config.write_text("config", encoding="utf-8")
    monkeypatch.setattr(cli, "_cyclonedds_domain_admissibility", lambda *_: {
        "admissible": True, "config_path": str(config),
        "container_cyclonedds_uri": "file:///opt/autoware/cyclonedds.xml",
        "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
    })
    monkeypatch.setattr(cli, "_cyclonedds_config_hash_matches", lambda *_: True)
    monkeypatch.setattr(cli, "_v2_uptake_udp_socket_preflight", lambda *_: ({
        "inspection_available": True, "bound_socket_lines": []}, None))
    commands: list[list[str]] = []

    class Result:
        def __init__(self, returncode: int = 0, stdout: str = "") -> None:
            self.returncode, self.stdout = returncode, stdout

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        commands.append(rendered)
        if rendered[:2] == ["git", "rev-parse"]:
            return Result(stdout="head\n")
        if rendered[:3] == ["docker", "container", "inspect"] and "--format" in rendered:
            return Result(stdout="sha256:" + "2" * 64 + "\n")
        if rendered[:3] == ["docker", "container", "inspect"]:
            return Result(returncode=1)
        if rendered[:2] == ["docker", "cp"]:
            source = rendered[2].split(":", 1)[1]
            if source == str(
                cli.PLANNER_INSTALL_ROOT
                / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"
            ):
                return Result(returncode=17)
            Path(rendered[-1]).write_text(Path(rendered[-1]).name, encoding="utf-8")
            return Result()
        return Result()

    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "v2-uptake-smoke", "--run-id", "attestation-fail"]
    )
    assert cli.run_v2_uptake_smoke(args) == 3
    state = json.loads((tmp_path / "review-state.json").read_text(encoding="utf-8"))
    assert state["current_owning_edge"]["runtime_budget"]["fresh_runtime_used"] == 1
    assert all(command[:3] != ["docker", "start", "-a"] for command in commands)
    result = json.loads(
        (tmp_path / "analysis/aic_test/runs/attestation-fail/result.json").read_text(
            encoding="utf-8"
        )
    )
    assert "image_planner_binary_resolution_failed:copy_failed" in result["reasons"]
    assert result["planner_binary_attestation"] == {
        "requested_path": str(
            cli.PLANNER_INSTALL_ROOT
            / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"
        ),
        "initial_type": "unobserved",
        "normalized_chain": [str(
            cli.PLANNER_INSTALL_ROOT
            / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"
        )],
        "copy_outcomes": [{
            "normalized_path": str(
                cli.PLANNER_INSTALL_ROOT
                / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"
            ),
            "returncode": 17,
        }],
        "resolved_path": None,
        "terminal_type": None,
        "size_bytes": None,
        "actual_sha256": None,
    }


def test_v2_runner_consumes_then_creates_inspects_binds_and_starts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    config = tmp_path / "vehicle" / "cyclonedds.xml"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text("config", encoding="utf-8")
    evidence = {
        "admissible": True, "config_path": str(config),
        "container_cyclonedds_uri": "file:///opt/autoware/cyclonedds.xml",
        "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
    }
    monkeypatch.setattr(cli, "_cyclonedds_domain_admissibility", lambda *_: evidence)
    monkeypatch.setattr(cli, "_cyclonedds_config_hash_matches", lambda *_: True)
    monkeypatch.setattr(cli, "_v2_uptake_udp_socket_preflight", lambda *_: ({
        "inspection_available": True, "bound_socket_lines": []}, None))
    commands: list[list[str]] = []

    class Result:
        def __init__(self, returncode: int = 0, stdout: str = "") -> None:
            self.returncode, self.stdout = returncode, stdout

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        commands.append(rendered)
        if rendered[:2] == ["git", "status"] or rendered[:2] == ["docker", "compose"]:
            return Result()
        if rendered[:2] == ["git", "rev-parse"]:
            return Result(stdout="head\n")
        if rendered[:3] == ["docker", "container", "inspect"]:
            if "--format" in rendered:
                return Result(stdout="sha256:" + "2" * 64 + "\n")
            return Result(returncode=1)
        if rendered[:4] == ["docker", "container", "ls", "-a"]:
            return Result()
        if rendered[:2] == ["ps", "-eo"]:
            return Result()
        if rendered[:2] == ["docker", "create"]:
            return Result()
        if rendered[:2] == ["docker", "cp"]:
            source = rendered[2].split(":", 1)[1]
            target = Path(rendered[-1])
            if source == str(cli.PLANNER_INSTALL_ROOT / "lib/state_lattice_overtake_planner/state_lattice_overtake_planner_node"):
                os.symlink(str(cli.PLANNER_BUILD_TARGET), target)
            elif source == str(cli.PLANNER_BUILD_TARGET):
                target.write_text("planner_binary_sha256", encoding="utf-8")
            elif source == str(cli.PP_INSTALL_ROOT / "lib/simple_pure_pursuit/simple_pure_pursuit"):
                target.write_text("pp_binary_sha256", encoding="utf-8")
            else:
                target.write_text(target.name, encoding="utf-8")
            return Result()
        if rendered[:3] == ["docker", "start", "-a"]:
            binding = tmp_path / "analysis/aic_test/runs/ordered/prelaunch-binding.json"
            assert binding.is_file()
            assert json.loads(binding.read_text(encoding="utf-8"))["actual_image_id"] == "sha256:" + "2" * 64
            return Result(returncode=1, stdout="runtime failed")
        return Result(returncode=1)

    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "v2-uptake-smoke", "--run-id", "ordered"]
    )
    assert cli.run_v2_uptake_smoke(args) == 3
    create_index = next(index for index, command in enumerate(commands) if command[:2] == ["docker", "create"])
    inspect_index = next(index for index, command in enumerate(commands) if command[:3] == ["docker", "container", "inspect"] and "--format" in command)
    start_index = next(index for index, command in enumerate(commands) if command[:3] == ["docker", "start", "-a"])
    assert create_index < inspect_index < start_index
    assert commands[create_index][2] == "--pull=never"


def test_v2_doctor_residue_detects_all_direct_containers_and_exact_processes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    class Result:
        def __init__(self, stdout: str = "", returncode: int = 0) -> None:
            self.stdout, self.returncode = stdout, returncode

    def fake_run(command: object, **_: object) -> Result:
        rendered = list(command)
        if rendered[:2] == ["docker", "compose"]:
            return Result()
        if rendered[:3] == ["docker", "container", "inspect"]:
            return Result(returncode=1)
        if rendered[:4] == ["docker", "container", "ls", "-a"]:
            return Result(stdout="aic-test-v2-other-run\n")
        if rendered[:2] == ["ps", "-eo"]:
            return Result(stdout="123 ros2 run simple_pure_pursuit simple_pure_pursuit\n")
        raise AssertionError(rendered)

    monkeypatch.setattr(cli, "_run", fake_run)
    evidence, error = cli._v2_uptake_doctor_residue(tmp_path, "aic-test-v2-new")
    assert error == "v2_uptake_doctor_residue_present"
    assert evidence["active_v2_containers"] == ["aic-test-v2-other-run"]
    assert evidence["relevant_host_processes"][0]["pid"] == "123"


def test_v2_doctor_failure_does_not_consume_budget_or_create(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _write_v2_uptake_prearm_inputs(tmp_path, monkeypatch)
    config = tmp_path / "vehicle" / "cyclonedds.xml"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text("config", encoding="utf-8")
    evidence = {"admissible": True, "config_path": str(config),
                "container_cyclonedds_uri": "file:///opt/autoware/cyclonedds.xml",
                "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest()}
    monkeypatch.setattr(cli, "_cyclonedds_domain_admissibility", lambda *_: evidence)
    monkeypatch.setattr(cli, "_cyclonedds_config_hash_matches", lambda *_: True)
    monkeypatch.setattr(cli, "_v2_uptake_udp_socket_preflight", lambda *_: ({
        "inspection_available": True, "bound_socket_lines": []}, None))
    monkeypatch.setattr(cli, "_v2_uptake_doctor_residue", lambda *_: (
        {"active_v2_containers": ["aic-test-v2-old"]},
        "v2_uptake_doctor_residue_present",
    ))
    commands: list[list[str]] = []

    class Result:
        returncode = 0
        stdout = ""

    def fake_run(command: object, **_: object) -> Result:
        commands.append(list(command))
        return Result()

    monkeypatch.setattr(cli, "_run", fake_run)
    args = build_parser().parse_args(
        ["--repo-root", str(tmp_path), "run", "v2-uptake-smoke", "--run-id", "blocked"]
    )
    assert cli.run_v2_uptake_smoke(args) == 3
    assert not list((tmp_path / "analysis" / "aic_test").glob("v2_uptake_one_shot_*.json"))
    assert all(command[:2] != ["docker", "create"] for command in commands)
