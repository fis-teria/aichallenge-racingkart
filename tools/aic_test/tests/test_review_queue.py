from __future__ import annotations

import hashlib
import json
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

import pytest

from aic_test.cli import build_parser
import aic_test.review_queue as review_queue_module
from aic_test.review_queue import ReviewQueue, ReviewQueueError


def _file(path: Path, text: str) -> Path:
    path.write_text(text, encoding="utf-8")
    return path


def test_production_cli_rejects_queue_root_override() -> None:
    with pytest.raises(SystemExit):
        build_parser().parse_args([
            "external-review", "--queue-root", "/tmp/forked-queue", "status",
            "--execution-id", "exec-1",
        ])


def test_terminal_failure_cli_requires_and_parses_bound_evidence_arguments() -> None:
    arguments = [
        "external-review", "terminal-failure", "--execution-id", "exec-1",
        "--review-attempt-id", "attempt-1", "--worker-id", "transport",
        "--evidence", "/tmp/evidence.json", "--evidence-sha256", "a" * 64,
    ]
    parsed = build_parser().parse_args(arguments)
    assert parsed.func.__name__ == "external_review"
    assert (parsed.execution_id, parsed.review_attempt_id, parsed.worker_id) == (
        "exec-1", "attempt-1", "transport")
    assert (parsed.evidence, parsed.evidence_sha256) == ("/tmp/evidence.json", "a" * 64)
    for option in ("--execution-id", "--review-attempt-id", "--worker-id", "--evidence", "--evidence-sha256"):
        missing = arguments.copy()
        index = missing.index(option)
        del missing[index:index + 2]
        with pytest.raises(SystemExit):
            build_parser().parse_args(missing)


def test_retire_pending_cli_requires_audit_identity_and_reason() -> None:
    arguments = [
        "external-review", "retire-pending", "--execution-id", "exec-1",
        "--review-attempt-id", "attempt-1", "--retired-by", "user-authorized-codex",
        "--request-sha256", "a" * 64,
        "--reason", "Obsolete invalid retry; preserve immutable request bytes.",
    ]
    parsed = build_parser().parse_args(arguments)
    assert parsed.func.__name__ == "external_review"
    assert parsed.review_command == "retire-pending"
    for option in (
        "--execution-id", "--review-attempt-id", "--retired-by",
        "--request-sha256", "--reason",
    ):
        missing = arguments.copy()
        index = missing.index(option)
        del missing[index:index + 2]
        with pytest.raises(SystemExit):
            build_parser().parse_args(missing)


def _request(queue: ReviewQueue, tmp_path: Path, *, attempt: str = "attempt-1",
             retry_of: str | None = None) -> dict:
    return queue.enqueue(
        execution_id="exec-1", review_attempt_id=attempt,
        handoff=_file(tmp_path / "handoff.md", "handoff\n"),
        packet=_file(tmp_path / "packet.md", "packet\n"),
        objective="Review without changing technical authority.", requested_by="bridge",
        retry_of=retry_of,
    )


def _response(queue: ReviewQueue, tmp_path: Path, attempt: str = "attempt-1") -> Path:
    request = queue.status("exec-1", attempt)["request"]
    path = tmp_path / f"{attempt}-response.json"
    path.write_text(json.dumps({
        "schema_version": 2, "execution_id": "exec-1", "review_attempt_id": attempt,
        "handoff_sha256": request["handoff_sha256"], "model": "GPT-5.6 Sol Pro",
        "model_fallback_reason": None, "project_url": request["approved_project_url"],
        "submission_id": "submission-1", "submission_verified": True, "severity": "None",
        "disposition": "No issue.", "minimum_changes": [], "transition_permission": "GO",
        "review_text": "External review completed for the bound attempt.",
    }), encoding="utf-8")
    return path


def _submit(queue: ReviewQueue, tmp_path: Path, attempt: str = "attempt-1",
            worker: str = "transport") -> dict:
    request = queue.status("exec-1", attempt)["request"]
    queue.begin_submission(
        execution_id="exec-1", review_attempt_id=attempt, worker_id=worker,
        model="GPT-5.6 Sol Pro", project_url=request["approved_project_url"],
        submission_id="submission-1",
    )
    return queue.record_submission(
        execution_id="exec-1", review_attempt_id=attempt, worker_id=worker,
        model="GPT-5.6 Sol Pro", project_url=request["approved_project_url"],
        submission_id="submission-1",
    )


def _evidence(tmp_path: Path, attempts: int = 3, outcome: str = "DISCONNECTED") -> Path:
    return _file(tmp_path / "connection-evidence.json", json.dumps({"connection_attempts": [
        {"timestamp": f"2026-08-05T00:00:0{i}Z", "outcome": outcome, "timeout_s": 30}
        for i in range(attempts)
    ]}))


def _terminal_failure_evidence(tmp_path: Path, *, retryable: bool = True) -> Path:
    return _file(tmp_path / "terminal-failure-evidence.json", json.dumps({
        "schema_version": 1, "submission_id": "submission-1",
        "terminal_error_kind": "RATE_LIMIT", "terminal_error_text": "Too many requests.",
        "observed_at": "2026-08-05T00:00:01Z", "retryable": retryable,
    }))


def test_round_trip_binds_execution_and_immutable_attempt(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    pending = _request(queue, tmp_path)
    assert pending["state"] == "REVIEW_PENDING"
    assert pending["queue_root"] == str((tmp_path / "queue").resolve())
    assert queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")["state"] == "CLAIMED"
    _submit(queue, tmp_path)
    completed = queue.complete(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
        model="GPT-5.6 Sol Pro", severity="None", disposition="No issue.",
        transition_permission="GO", response=_response(queue, tmp_path), minimum_changes=[],
        submission_id="submission-1", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"],
    )
    assert completed["state"] == "COMPLETED"
    assert completed["workflow_advance_allowed"] is False


def test_only_one_transport_can_claim_an_attempt(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    def claim(worker: str) -> str:
        try:
            queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id=worker)
            return "won"
        except ReviewQueueError:
            return "lost"
    with ThreadPoolExecutor(max_workers=2) as pool:
        assert sorted(pool.map(claim, ("worker-a", "worker-b"))) == ["lost", "won"]


def test_deferral_preserves_claim_and_allows_hash_bound_retry(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    deferred = queue.defer(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
        evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
    )
    assert deferred["state"] == "DEFERRED_CONNECTION_UNAVAILABLE"
    assert deferred["deferral"]["resume_action"] == "REVIEW_SAME_IMMUTABLE_EXECUTION"
    assert deferred["workflow_advance_allowed"] is False
    retry = _request(queue, tmp_path, attempt="attempt-2", retry_of="attempt-1")
    assert retry["request"]["execution_id"] == "exec-1"
    assert queue.claim(execution_id="exec-1", review_attempt_id="attempt-2", worker_id="transport")["state"] == "CLAIMED"
    assert (tmp_path / "queue/claims/attempt-1.json").exists()


def test_retry_rejects_missing_deferral_and_different_immutable_inputs(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    with pytest.raises(ReviewQueueError, match="retry_prior_deferral_invalid"):
        _request(queue, tmp_path, attempt="attempt-2", retry_of="attempt-1")
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
                evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    with pytest.raises(ReviewQueueError, match="retry_source_hash_mismatch"):
        queue.enqueue(execution_id="exec-1", review_attempt_id="attempt-3",
                      handoff=_file(tmp_path / "other-handoff.md", "other\n"),
                      packet=tmp_path / "packet.md", objective="same", requested_by="bridge", retry_of="attempt-1")


def test_defer_requires_claim_owner_hash_and_connection_evidence(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="owner")
    evidence = _evidence(tmp_path, attempts=2)
    with pytest.raises(ReviewQueueError, match="claim_owner_mismatch"):
        queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="other", evidence=evidence,
                    evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    with pytest.raises(ReviewQueueError, match="connection_attempts_insufficient"):
        queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="owner", evidence=evidence,
                    evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    with pytest.raises(ReviewQueueError, match="connection_attempts_insufficient"):
        queue.defer(
            execution_id="exec-1", review_attempt_id="attempt-1", worker_id="owner",
            evidence=evidence,
            evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
            terminal_connection_error="browser profile already in use",
        )
    with pytest.raises(ReviewQueueError, match="connection_evidence_hash_mismatch"):
        queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="owner", evidence=evidence,
                    evidence_sha256="0" * 64, terminal_connection_error="browser disconnected")


def test_complete_rejects_response_for_a_different_attempt(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    response = _response(queue, tmp_path)
    payload = json.loads(response.read_text())
    payload["review_attempt_id"] = "attempt-other"
    response.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ReviewQueueError, match="response_disposition_or_provenance_mismatch"):
        queue.complete(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
                       model="GPT-5.6 Sol Pro", severity="None", disposition="No issue.", transition_permission="GO",
                       response=response, minimum_changes=[], submission_id="submission-1",
                       project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"])


def test_status_backwards_reads_legacy_poc03_without_mutating_it(tmp_path: Path) -> None:
    root = tmp_path / "queue"
    request_path = root / "requests/poc-03.json"
    request_path.parent.mkdir(parents=True)
    handoff, packet = _file(tmp_path / "legacy-handoff.md", "handoff\n"), _file(tmp_path / "legacy-packet.md", "packet\n")
    request_path.write_text(json.dumps({
        "schema_version": 1, "execution_id": "poc-03", "status": "REVIEW_PENDING", "requested_by": "bridge",
        "objective": "legacy", "handoff_path": str(handoff), "handoff_sha256": hashlib.sha256(handoff.read_bytes()).hexdigest(),
        "packet_path": str(packet), "packet_sha256": hashlib.sha256(packet.read_bytes()).hexdigest(),
        "approved_models": ["GPT-5.6 Sol Pro", "GPT-5.6 Pro"],
        "approved_project_url": "https://chatgpt.com/g/g-p-6a67ef32dd9c8191af3d8ee937ef668f-codexyong-wakusuhesu/project",
        "transport": "Playwright MCP only", "created_at": "2026-08-05T00:00:00Z",
    }), encoding="utf-8")
    status = ReviewQueue(root).status("poc-03")
    assert status["legacy_read_only"] is True
    assert status["state"] == "REVIEW_PENDING"
    assert status["workflow_advance_allowed"] is False


def test_legacy_claim_can_defer_without_mutating_legacy_records_and_retry(tmp_path: Path) -> None:
    root, execution_id = tmp_path / "queue", "poc-03"
    request_path = root / f"requests/{execution_id}.json"
    request_path.parent.mkdir(parents=True)
    handoff, packet = _file(tmp_path / "legacy-handoff.md", "handoff\n"), _file(tmp_path / "legacy-packet.md", "packet\n")
    request_path.write_text(json.dumps({
        "schema_version": 1, "execution_id": execution_id, "status": "REVIEW_PENDING", "requested_by": "bridge",
        "objective": "legacy", "handoff_path": str(handoff), "handoff_sha256": hashlib.sha256(handoff.read_bytes()).hexdigest(),
        "packet_path": str(packet), "packet_sha256": hashlib.sha256(packet.read_bytes()).hexdigest(),
        "approved_models": ["GPT-5.6 Sol Pro", "GPT-5.6 Pro"],
        "approved_project_url": "https://chatgpt.com/g/g-p-6a67ef32dd9c8191af3d8ee937ef668f-codexyong-wakusuhesu/project",
        "transport": "Playwright MCP only", "created_at": "2026-08-05T00:00:00Z",
    }), encoding="utf-8")
    claim_path = root / f"claims/{execution_id}.json"
    claim_path.parent.mkdir(parents=True)
    claim_path.write_text(json.dumps({
        "schema_version": 1, "execution_id": execution_id, "status": "CLAIMED", "worker_id": "transport",
        "request_sha256": hashlib.sha256(request_path.read_bytes()).hexdigest(),
        "handoff_sha256": hashlib.sha256(handoff.read_bytes()).hexdigest(), "claimed_at": "2026-08-05T00:00:01Z",
    }), encoding="utf-8")
    original_request, original_claim = request_path.read_bytes(), claim_path.read_bytes()
    queue, evidence = ReviewQueue(root), _evidence(tmp_path)
    deferred = queue.defer(execution_id=execution_id, review_attempt_id=execution_id, worker_id="transport",
                           evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    assert deferred["state"] == "DEFERRED_CONNECTION_UNAVAILABLE"
    assert deferred["deferral"]["legacy_claim"] is True
    assert request_path.read_bytes() == original_request
    assert claim_path.read_bytes() == original_claim
    retry = queue.enqueue(execution_id=execution_id, review_attempt_id="poc-03-retry-01", handoff=handoff,
                          packet=packet, objective="retry", requested_by="bridge", retry_of=execution_id)
    assert retry["state"] == "REVIEW_PENDING"


def test_historical_legacy_browser_profile_deferral_allows_exact_retry(tmp_path: Path) -> None:
    root, execution_id = tmp_path / "queue", "poc-03"
    request_path = root / f"requests/{execution_id}.json"
    request_path.parent.mkdir(parents=True)
    handoff = _file(tmp_path / "legacy-handoff.md", "handoff\n")
    packet = _file(tmp_path / "legacy-packet.md", "packet\n")
    request_path.write_text(json.dumps({
        "schema_version": 1, "execution_id": execution_id, "status": "REVIEW_PENDING",
        "requested_by": "bridge", "objective": "legacy",
        "handoff_path": str(handoff),
        "handoff_sha256": hashlib.sha256(handoff.read_bytes()).hexdigest(),
        "packet_path": str(packet),
        "packet_sha256": hashlib.sha256(packet.read_bytes()).hexdigest(),
        "approved_models": ["GPT-5.6 Sol Pro", "GPT-5.6 Pro"],
        "approved_project_url": (
            "https://chatgpt.com/g/g-p-6a67ef32dd9c8191af3d8ee937ef668f-"
            "codexyong-wakusuhesu/project"
        ),
        "transport": "Playwright MCP only", "created_at": "2026-08-05T00:00:00Z",
    }), encoding="utf-8")
    claim_path = root / f"claims/{execution_id}.json"
    claim_path.parent.mkdir(parents=True)
    claim_path.write_text(json.dumps({
        "schema_version": 1, "execution_id": execution_id, "status": "CLAIMED",
        "worker_id": "transport",
        "request_sha256": hashlib.sha256(request_path.read_bytes()).hexdigest(),
        "handoff_sha256": hashlib.sha256(handoff.read_bytes()).hexdigest(),
        "claimed_at": "2026-08-05T00:00:01Z",
    }), encoding="utf-8")
    legacy_outcome = "Browser profile already in use; Playwright could not enumerate tabs"
    attempts = [
        {"timestamp": f"2026-08-05T00:00:0{i}Z", "outcome": legacy_outcome, "timeout_s": 15}
        for i in range(3)
    ]
    evidence = _file(
        tmp_path / "connection-evidence.json",
        json.dumps({"connection_attempts": attempts}),
    )
    deferral_path = root / f"deferrals/{execution_id}.json"
    deferral_path.parent.mkdir(parents=True)
    deferral_path.write_text(json.dumps({
        "schema_version": 2, "execution_id": execution_id,
        "review_attempt_id": execution_id, "status": "DEFERRED_CONNECTION_UNAVAILABLE",
        "resume_action": "REVIEW_SAME_IMMUTABLE_EXECUTION",
        "request_sha256": hashlib.sha256(request_path.read_bytes()).hexdigest(),
        "handoff_sha256": hashlib.sha256(handoff.read_bytes()).hexdigest(),
        "packet_sha256": hashlib.sha256(packet.read_bytes()).hexdigest(),
        "worker_id": "transport",
        "claim_sha256": hashlib.sha256(claim_path.read_bytes()).hexdigest(),
        "connection_evidence_path": str(evidence),
        "connection_evidence_sha256": hashlib.sha256(evidence.read_bytes()).hexdigest(),
        "connection_attempts": attempts, "terminal_connection_error": None,
        "workflow_advance_allowed": False, "deferred_at": "2026-08-05T00:01:00Z",
        "legacy_claim": True,
    }), encoding="utf-8")

    queue = ReviewQueue(root)
    status = queue.status(execution_id)
    assert status["state"] == "DEFERRED_CONNECTION_UNAVAILABLE"
    assert status["integrity_valid"] is True
    retry = queue.enqueue(
        execution_id=execution_id, review_attempt_id="poc-03-retry-01",
        handoff=handoff, packet=packet, objective="retry", requested_by="bridge",
        retry_of=execution_id,
    )
    assert retry["state"] == "REVIEW_PENDING"
    assert retry["integrity_valid"] is True


def test_historical_legacy_browser_profile_compatibility_is_exact() -> None:
    outcome = "Browser profile already in use; Playwright could not enumerate tabs"

    def attempts(count: int = 3, *, value: str = outcome) -> list[dict]:
        return [
            {"timestamp": f"2026-08-05T00:00:0{i}Z", "outcome": value, "timeout_s": 15}
            for i in range(count)
        ]

    assert ReviewQueue._legacy_connection_attempts_valid(attempts()) is True
    assert ReviewQueue._legacy_connection_attempts_valid(attempts(2)) is False
    assert ReviewQueue._legacy_connection_attempts_valid(attempts(4)) is False
    assert ReviewQueue._legacy_connection_attempts_valid(
        attempts(value=outcome + " ")
    ) is False
    invalid_timeout = attempts()
    invalid_timeout[0]["timeout_s"] = 0
    assert ReviewQueue._legacy_connection_attempts_valid(invalid_timeout) is False
    extra_key = attempts()
    extra_key[0]["detail"] = "profile busy"
    assert ReviewQueue._legacy_connection_attempts_valid(extra_key) is False


def test_current_defer_rejects_historical_legacy_browser_profile_outcome(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(
        tmp_path,
        outcome="Browser profile already in use; Playwright could not enumerate tabs",
    )
    with pytest.raises(ReviewQueueError, match="connection_attempts_invalid"):
        queue.defer(
            execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
            evidence=evidence,
            evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
        )


def test_stored_current_deferral_rejects_historical_legacy_browser_profile_outcome(
    tmp_path: Path,
) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    queue.defer(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
        evidence=evidence,
        evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
    )
    evidence_payload = json.loads(evidence.read_text(encoding="utf-8"))
    for attempt in evidence_payload["connection_attempts"]:
        attempt["outcome"] = (
            "Browser profile already in use; Playwright could not enumerate tabs"
        )
    evidence.write_text(json.dumps(evidence_payload), encoding="utf-8")
    marker_path = tmp_path / "queue/terminals/attempt-1.json"
    marker = json.loads(marker_path.read_text(encoding="utf-8"))
    marker["connection_attempts"] = evidence_payload["connection_attempts"]
    marker["connection_evidence_sha256"] = hashlib.sha256(evidence.read_bytes()).hexdigest()
    marker_path.write_text(json.dumps(marker), encoding="utf-8")

    status = queue.status("exec-1", "attempt-1")
    assert status["state"] == "INVALID_EVIDENCE"
    assert "deferral_schema_or_binding_invalid" in status["integrity_errors"]
    with pytest.raises(ReviewQueueError, match="retry_prior_deferral_invalid"):
        _request(queue, tmp_path, attempt="attempt-2", retry_of="attempt-1")


def test_status_fails_closed_for_forged_result_and_tampered_response(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    response = _response(queue, tmp_path)
    request_path = tmp_path / "queue/requests/attempt-1.json"
    result_path = tmp_path / "queue/results/attempt-1.json"
    result_path.parent.mkdir()
    result_path.write_text(json.dumps({"schema_version": 2, "execution_id": "exec-1", "status": "COMPLETED"}), encoding="utf-8")
    assert queue.status("exec-1", "attempt-1")["state"] == "INVALID_EVIDENCE"
    result_path.unlink()
    queue.complete(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
                   model="GPT-5.6 Sol Pro", severity="None", disposition="No issue.", transition_permission="GO",
                   response=response, minimum_changes=[], submission_id="submission-1",
                   project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"])
    response.write_bytes(b"\xff\xfe")
    assert queue.status("exec-1", "attempt-1")["state"] == "INVALID_EVIDENCE"


def test_claim_and_complete_fail_closed_for_tampered_source_and_invalid_completion(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    pending = _request(queue, tmp_path)
    Path(pending["request"]["packet_path"]).write_text("changed\n", encoding="utf-8")
    with pytest.raises(ReviewQueueError, match="request_source_integrity_invalid"):
        queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    queue = ReviewQueue(tmp_path / "queue-2")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    with pytest.raises(ReviewQueueError, match="completion_value_invalid"):
        queue.complete(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", model="other",
                       severity="None", disposition="No issue.", transition_permission="GO", response=_response(queue, tmp_path),
                       minimum_changes=[], submission_id="submission-1", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"])
    with pytest.raises(ReviewQueueError, match="response_schema_invalid"):
        bad = _response(queue, tmp_path)
        payload = json.loads(bad.read_text())
        payload.pop("review_attempt_id")
        bad.write_text(json.dumps(payload), encoding="utf-8")
        queue.complete(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", model="GPT-5.6 Sol Pro",
                       severity="None", disposition="No issue.", transition_permission="GO", response=bad, minimum_changes=[],
                       submission_id="submission-1", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"])


def test_defer_rejects_existing_completed_result(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    queue.complete(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", model="GPT-5.6 Sol Pro",
                   severity="None", disposition="No issue.", transition_permission="GO", response=_response(queue, tmp_path),
                   minimum_changes=[], submission_id="submission-1", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"])
    evidence = _evidence(tmp_path)
    with pytest.raises(ReviewQueueError, match="review_already_completed"):
        queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
                    evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())


def test_complete_rejects_existing_deferral(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
                evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    with pytest.raises(ReviewQueueError, match="review_already_deferred"):
        queue.complete(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", model="GPT-5.6 Sol Pro",
                       severity="None", disposition="No issue.", transition_permission="GO", response=_response(queue, tmp_path),
                       minimum_changes=[], submission_id="submission-1", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"])


def test_dual_terminal_status_and_retry_fail_closed(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
                evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    request = queue.status("exec-1", "attempt-1")["request"]
    result_path = tmp_path / "queue/results/attempt-1.json"
    result_path.parent.mkdir(exist_ok=True)
    result_path.write_text(json.dumps({
        "schema_version": 2, "execution_id": "exec-1", "review_attempt_id": "attempt-1", "status": "COMPLETED",
        "request_sha256": hashlib.sha256((tmp_path / "queue/requests/attempt-1.json").read_bytes()).hexdigest(),
        "handoff_sha256": request["handoff_sha256"], "packet_sha256": request["packet_sha256"], "worker_id": "transport",
        "model": "GPT-5.6 Sol Pro", "model_fallback_reason": None, "project_url": request["approved_project_url"],
        "submission_id": "submission-1", "submission_verified": True, "severity": "None", "disposition": "No issue.",
        "transition_permission": "GO", "minimum_changes": [], "response_path": str(_response(queue, tmp_path)),
        "response_sha256": hashlib.sha256(_response(queue, tmp_path).read_bytes()).hexdigest(), "advisory_only": True,
        "workflow_advance_allowed": False, "completed_at": "2026-08-05T00:00:00Z",
    }), encoding="utf-8")
    status = queue.status("exec-1", "attempt-1")
    assert status["state"] == "INVALID_EVIDENCE"
    assert "terminal_records_conflict" in status["integrity_errors"]
    with pytest.raises(ReviewQueueError, match="retry_prior_terminal_conflict"):
        _request(queue, tmp_path, attempt="attempt-2", retry_of="attempt-1")


def test_forged_retry_lineage_fails_closed_in_status_and_claim(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
                evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    _request(queue, tmp_path, attempt="attempt-2", retry_of="attempt-1")
    request_path = tmp_path / "queue/requests/attempt-2.json"
    forged = json.loads(request_path.read_text())
    forged["retry_of"] = "missing-attempt"
    request_path.write_text(json.dumps(forged), encoding="utf-8")
    status = queue.status("exec-1", "attempt-2")
    assert status["state"] == "INVALID_EVIDENCE"
    assert "retry_lineage_invalid" in status["integrity_errors"]
    with pytest.raises(ReviewQueueError, match="request_source_integrity_invalid"):
        queue.claim(execution_id="exec-1", review_attempt_id="attempt-2", worker_id="transport")


def test_forged_retry_hash_mismatch_fails_closed_in_status(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
                evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    _request(queue, tmp_path, attempt="attempt-2", retry_of="attempt-1")
    request_path = tmp_path / "queue/requests/attempt-2.json"
    forged = json.loads(request_path.read_text())
    forged["packet_sha256"] = "0" * 64
    request_path.write_text(json.dumps(forged), encoding="utf-8")
    assert queue.status("exec-1", "attempt-2")["state"] == "INVALID_EVIDENCE"


def test_concurrent_defer_and_complete_have_one_terminal_winner(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    evidence, response = _terminal_failure_evidence(tmp_path), _response(queue, tmp_path)
    barrier = threading.Barrier(2)

    def defer() -> str:
        barrier.wait()
        try:
            queue.terminal_failure(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
                                   evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
            return "won"
        except ReviewQueueError as exc:
            return str(exc)

    def complete() -> str:
        barrier.wait()
        try:
            queue.complete(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", model="GPT-5.6 Sol Pro",
                           severity="None", disposition="No issue.", transition_permission="GO", response=response,
                           minimum_changes=[], submission_id="submission-1", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"])
            return "won"
        except ReviewQueueError as exc:
            return str(exc)

    with ThreadPoolExecutor(max_workers=2) as pool:
        outcomes = list(pool.map(lambda fn: fn(), (defer, complete)))
    assert outcomes.count("won") == 1
    assert any(outcome in {"terminal_conflict", "review_already_terminal_failure", "review_already_completed"}
               for outcome in outcomes if outcome != "won")


def test_atomic_marker_is_the_readable_v2_terminal_record(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
                evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    status = queue.status("exec-1", "attempt-1")
    assert status["state"] == "DEFERRED_CONNECTION_UNAVAILABLE"
    assert (tmp_path / "queue/terminals/attempt-1.json").is_file()
    assert not (tmp_path / "queue/deferrals/attempt-1.json").exists()


def test_retryable_terminal_failure_binds_claim_and_allows_hash_bound_retry(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    evidence = _terminal_failure_evidence(tmp_path)
    failed = queue.terminal_failure(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
        evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
    )
    assert failed["state"] == "RETRYABLE_UI_TERMINAL_FAILURE"
    assert failed["terminal_failure"]["resume_action"] == "REVIEW_SAME_IMMUTABLE_EXECUTION"
    assert failed["terminal_failure"]["workflow_advance_allowed"] is False
    retry = _request(queue, tmp_path, attempt="attempt-2", retry_of="attempt-1")
    assert retry["request"]["handoff_sha256"] == failed["request"]["handoff_sha256"]
    failed_at = datetime.fromisoformat(
        failed["terminal_failure"]["failed_at"].replace("Z", "+00:00")
    )
    not_before = datetime.fromisoformat(
        retry["request"]["not_before"].replace("Z", "+00:00")
    )
    assert (not_before - failed_at).total_seconds() == 600
    with pytest.raises(ReviewQueueError, match="review_queue_cooldown_until"):
        queue.claim(execution_id="exec-1", review_attempt_id="attempt-2", worker_id="transport")

    request_path = tmp_path / "queue/requests/attempt-2.json"
    request_payload = json.loads(request_path.read_text(encoding="utf-8"))
    request_payload["not_before"] = "2000-01-01T00:00:00Z"
    request_path.write_text(json.dumps(request_payload), encoding="utf-8")
    terminal_path = tmp_path / "queue/terminals/attempt-1.json"
    terminal_payload = json.loads(terminal_path.read_text(encoding="utf-8"))
    terminal_payload["failed_at"] = "2000-01-01T00:00:00Z"
    terminal_path.write_text(json.dumps(terminal_payload), encoding="utf-8")
    assert queue.claim(
        execution_id="exec-1", review_attempt_id="attempt-2", worker_id="transport"
    )["state"] == "CLAIMED"


def test_connection_deferral_retry_is_not_rate_limited(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path)
    queue.defer(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
        evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
    )
    retry = _request(queue, tmp_path, attempt="attempt-2", retry_of="attempt-1")
    assert "not_before" not in retry["request"]
    assert queue.claim(
        execution_id="exec-1", review_attempt_id="attempt-2", worker_id="transport"
    )["state"] == "CLAIMED"


def test_terminal_failure_rejects_wrong_owner_tampering_and_nonretryable_evidence(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="owner")
    _submit(queue, tmp_path, worker="owner")
    evidence = _terminal_failure_evidence(tmp_path, retryable=False)
    with pytest.raises(ReviewQueueError, match="claim_owner_mismatch"):
        queue.terminal_failure(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="other",
                               evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    with pytest.raises(ReviewQueueError, match="terminal_failure_evidence_invalid"):
        queue.terminal_failure(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="owner",
                               evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    evidence = _terminal_failure_evidence(tmp_path)
    with pytest.raises(ReviewQueueError, match="terminal_failure_evidence_hash_mismatch"):
        queue.terminal_failure(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="owner",
                               evidence=evidence, evidence_sha256="0" * 64)
    payload = json.loads(evidence.read_text())
    payload["terminal_error_kind"] = "SOMETHING_ELSE"
    evidence.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ReviewQueueError, match="terminal_failure_evidence_invalid"):
        queue.terminal_failure(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="owner",
                               evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())


def test_defer_rejects_rate_limit_as_a_connection_failure(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path, outcome="RATE_LIMIT")
    with pytest.raises(ReviewQueueError, match="connection_attempts_invalid"):
        queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
                    evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())


def test_defer_rejects_ambiguous_terminal_error_with_rate_limit_text(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    evidence = _evidence(tmp_path, outcome="TERMINAL_ERROR")
    with pytest.raises(ReviewQueueError, match="connection_attempts_invalid"):
        queue.defer(
            execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
            evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest(),
            terminal_connection_error="rate limit",
        )


def test_terminal_failure_race_has_one_terminal_winner(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    evidence, response = _terminal_failure_evidence(tmp_path), _response(queue, tmp_path)
    barrier = threading.Barrier(2)

    def fail() -> str:
        barrier.wait()
        try:
            queue.terminal_failure(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
                                   evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
            return "won"
        except ReviewQueueError as exc:
            return str(exc)

    def complete() -> str:
        barrier.wait()
        try:
            queue.complete(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
                           model="GPT-5.6 Sol Pro", severity="None", disposition="No issue.",
                           transition_permission="GO", response=response, minimum_changes=[], submission_id="submission-1",
                           project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"])
            return "won"
        except ReviewQueueError as exc:
            return str(exc)

    with ThreadPoolExecutor(max_workers=2) as pool:
        outcomes = list(pool.map(lambda fn: fn(), (fail, complete)))
    assert outcomes.count("won") == 1
    assert any(outcome in {"terminal_conflict", "review_already_terminal_failure", "review_already_completed"}
               for outcome in outcomes if outcome != "won")


def test_terminal_failure_status_fails_closed_for_tampered_marker(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    evidence = _terminal_failure_evidence(tmp_path)
    queue.terminal_failure(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
                           evidence=evidence, evidence_sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())
    marker = tmp_path / "queue/terminals/attempt-1.json"
    payload = json.loads(marker.read_text())
    payload["terminal_error_text"] = "forged"
    marker.write_text(json.dumps(payload), encoding="utf-8")
    status = queue.status("exec-1", "attempt-1")
    assert status["state"] == "INVALID_EVIDENCE"
    assert "terminal_failure_schema_or_binding_invalid" in status["integrity_errors"]


def test_cross_attempt_claims_are_queue_singleflight(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path, attempt="attempt-a")
    _request(queue, tmp_path, attempt="attempt-b")

    def claim(attempt: str) -> str:
        try:
            queue.claim(execution_id="exec-1", review_attempt_id=attempt, worker_id=f"worker-{attempt[-1]}")
            return "won"
        except ReviewQueueError as exc:
            return str(exc)

    with ThreadPoolExecutor(max_workers=2) as pool:
        outcomes = list(pool.map(claim, ("attempt-a", "attempt-b")))
    assert outcomes.count("won") == 1
    assert sum(outcome.startswith(("review_singleflight_busy:", "review_not_oldest_pending:"))
               for outcome in outcomes) == 1
    assert len(list((tmp_path / "queue/singleflight_leases").glob("*.json"))) == 1


def test_claim_next_is_fifo_and_skips_only_future_rate_limit_retry(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    timestamps = iter(("2026-01-01T00:00:00Z", "2026-01-01T00:00:01Z", "2026-01-01T00:00:02Z"))
    monkeypatch.setattr(review_queue_module, "_now_utc", lambda: next(timestamps))
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path, attempt="attempt-old")
    _request(queue, tmp_path, attempt="attempt-new")
    claimed = queue.claim_next(worker_id="transport")
    assert claimed is not None
    assert claimed["review_attempt_id"] == "attempt-old"
    assert queue.status("exec-1", "attempt-new")["state"] == "REVIEW_PENDING"


def test_terminal_cooldown_is_global_but_connection_deferral_is_not(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path, attempt="attempt-1")
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path, attempt="attempt-1")
    queue.complete(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
        model="GPT-5.6 Sol Pro", severity="None", disposition="No issue.",
        transition_permission="GO", response=_response(queue, tmp_path), minimum_changes=[],
        submission_id="submission-1", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"],
    )
    _request(queue, tmp_path, attempt="attempt-2")
    with pytest.raises(ReviewQueueError, match="review_queue_cooldown_until"):
        queue.claim_next(worker_id="transport")
    marker_path = tmp_path / "queue/terminals/attempt-1.json"
    marker = json.loads(marker_path.read_text(encoding="utf-8"))
    marker["completed_at"] = "2000-01-01T00:00:00Z"
    marker_path.write_text(json.dumps(marker), encoding="utf-8")
    assert queue.claim_next(worker_id="transport")["review_attempt_id"] == "attempt-2"


def test_submitting_and_submitted_restart_never_resend(tmp_path: Path) -> None:
    root = tmp_path / "queue"
    queue = ReviewQueue(root)
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    submitting = queue.begin_submission(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
        model="GPT-5.6 Sol Pro", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"],
        submission_id="submission-1",
    )
    assert submitting["state"] == "SUBMITTING"
    restarted = ReviewQueue(root)
    assert restarted.status("exec-1", "attempt-1")["state"] == "SUBMITTING"
    with pytest.raises(ReviewQueueError, match="review_singleflight_busy"):
        restarted.claim_next(worker_id="transport")
    submitted = restarted.record_submission(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
        model="GPT-5.6 Sol Pro", submission_id="submission-1",
        project_url=restarted.status("exec-1", "attempt-1")["request"]["approved_project_url"],
    )
    assert submitted["state"] == "SUBMITTED"
    with pytest.raises(ReviewQueueError, match="submission_already_recorded"):
        restarted.record_submission(
            execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
            model="GPT-5.6 Sol Pro", submission_id="submission-1",
            project_url=submitted["request"]["approved_project_url"],
        )


def test_completion_requires_exact_submission_binding(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
    _submit(queue, tmp_path)
    response = _response(queue, tmp_path)
    with pytest.raises(ReviewQueueError, match="completion_submission_mismatch"):
        queue.complete(
            execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
            model="GPT-5.6 Sol Pro", severity="None", disposition="No issue.",
            transition_permission="GO", response=response, minimum_changes=[],
            submission_id="different-submission", project_url=queue.status("exec-1", "attempt-1")["request"]["approved_project_url"],
        )
    assert queue.status("exec-1", "attempt-1")["state"] == "SUBMITTED"


def test_fifo_oldest_invalid_request_blocks_newer_claim(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path, attempt="attempt-old")
    _request(queue, tmp_path, attempt="attempt-new")
    old_path = tmp_path / "queue/requests/attempt-old.json"
    payload = json.loads(old_path.read_text(encoding="utf-8"))
    payload.pop("objective")
    old_path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ReviewQueueError, match="pending_request_invalid:attempt-old"):
        queue.claim_next(worker_id="transport")
    assert not (tmp_path / "queue/singleflight_leases/attempt-new.json").exists()


def test_retire_pending_sidecar_skips_invalid_unclaimed_request_without_mutating_it(
    tmp_path: Path,
) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path, attempt="attempt-old")
    old_path = tmp_path / "queue/requests/attempt-old.json"
    old = json.loads(old_path.read_text(encoding="utf-8"))
    old["retry_of"] = "missing-attempt"
    old_path.write_text(json.dumps(old), encoding="utf-8")
    original_request = old_path.read_bytes()
    _request(queue, tmp_path, attempt="attempt-new")

    retired = queue.retire_pending(
        execution_id="exec-1", review_attempt_id="attempt-old",
        retired_by="user-authorized-codex",
        expected_request_sha256=hashlib.sha256(original_request).hexdigest(),
        reason="Obsolete invalid retry; preserve immutable request bytes.",
    )
    assert retired["state"] == "RETIRED_PENDING"
    assert retired["integrity_valid"] is True
    assert retired["workflow_advance_allowed"] is False
    assert retired["retirement"]["reason_code"] == "ABANDONED_INVALID_RETRY_LINEAGE"
    assert old_path.read_bytes() == original_request
    with pytest.raises(ReviewQueueError, match="review_retired"):
        queue.claim(
            execution_id="exec-1", review_attempt_id="attempt-old", worker_id="transport",
        )
    claimed = queue.claim_next(worker_id="transport")
    assert claimed is not None
    assert claimed["review_attempt_id"] == "attempt-new"
    with pytest.raises(ReviewQueueError, match="retry_prior_retired"):
        _request(queue, tmp_path, attempt="attempt-retry", retry_of="attempt-old")


def test_retire_pending_rejects_claimed_or_source_tampered_request(tmp_path: Path) -> None:
    claimed_queue = ReviewQueue(tmp_path / "claimed-queue")
    _request(claimed_queue, tmp_path)
    claimed_queue.claim(
        execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport",
    )
    claimed_path = tmp_path / "claimed-queue/requests/attempt-1.json"
    claimed_request = json.loads(claimed_path.read_text(encoding="utf-8"))
    claimed_request["retry_of"] = "missing-attempt"
    claimed_path.write_text(json.dumps(claimed_request), encoding="utf-8")
    with pytest.raises(ReviewQueueError, match="retire_pending_not_unclaimed"):
        claimed_queue.retire_pending(
            execution_id="exec-1", review_attempt_id="attempt-1",
            retired_by="user-authorized-codex",
            expected_request_sha256=hashlib.sha256(claimed_path.read_bytes()).hexdigest(),
            reason="No longer required.",
        )

    tampered_queue = ReviewQueue(tmp_path / "tampered-queue")
    pending = _request(tampered_queue, tmp_path, attempt="attempt-tampered")
    Path(pending["request"]["packet_path"]).write_text("tampered\n", encoding="utf-8")
    with pytest.raises(ReviewQueueError, match="request_source_integrity_invalid"):
        tampered_queue.retire_pending(
            execution_id="exec-1", review_attempt_id="attempt-tampered",
            retired_by="user-authorized-codex",
            expected_request_sha256=hashlib.sha256(
                (tmp_path / "tampered-queue/requests/attempt-tampered.json").read_bytes()
            ).hexdigest(),
            reason="No longer required.",
        )


def test_retire_pending_rejects_valid_pending_and_wrong_expected_hash(tmp_path: Path) -> None:
    valid_queue = ReviewQueue(tmp_path / "valid-queue")
    _request(valid_queue, tmp_path)
    valid_path = tmp_path / "valid-queue/requests/attempt-1.json"
    with pytest.raises(ReviewQueueError, match="retire_pending_not_invalid_retry"):
        valid_queue.retire_pending(
            execution_id="exec-1", review_attempt_id="attempt-1",
            retired_by="user-authorized-codex",
            expected_request_sha256=hashlib.sha256(valid_path.read_bytes()).hexdigest(),
            reason="No longer required.",
        )

    invalid_queue = ReviewQueue(tmp_path / "invalid-queue")
    _request(invalid_queue, tmp_path)
    invalid_path = tmp_path / "invalid-queue/requests/attempt-1.json"
    invalid_request = json.loads(invalid_path.read_text(encoding="utf-8"))
    invalid_request["retry_of"] = "missing-attempt"
    invalid_path.write_text(json.dumps(invalid_request), encoding="utf-8")
    with pytest.raises(ReviewQueueError, match="retire_pending_request_sha256_mismatch"):
        invalid_queue.retire_pending(
            execution_id="exec-1", review_attempt_id="attempt-1",
            retired_by="user-authorized-codex", expected_request_sha256="0" * 64,
            reason="No longer required.",
        )


def test_tampered_retirement_remains_fifo_barrier(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path, attempt="attempt-old")
    old_path = tmp_path / "queue/requests/attempt-old.json"
    old = json.loads(old_path.read_text(encoding="utf-8"))
    old["retry_of"] = "missing-attempt"
    old_path.write_text(json.dumps(old), encoding="utf-8")
    queue.retire_pending(
        execution_id="exec-1", review_attempt_id="attempt-old",
        retired_by="user-authorized-codex",
        expected_request_sha256=hashlib.sha256(old_path.read_bytes()).hexdigest(),
        reason="No longer required.",
    )
    retirement_path = tmp_path / "queue/retirements/attempt-old.json"
    retirement = json.loads(retirement_path.read_text(encoding="utf-8"))
    retirement["request_sha256"] = "0" * 64
    retirement_path.write_text(json.dumps(retirement), encoding="utf-8")
    _request(queue, tmp_path, attempt="attempt-new")

    status = queue.status("exec-1", "attempt-old")
    assert status["state"] == "INVALID_EVIDENCE"
    assert "retirement_schema_or_binding_invalid" in status["integrity_errors"]
    with pytest.raises(ReviewQueueError, match="pending_retirement_invalid:attempt-old"):
        queue.claim_next(worker_id="transport")
    assert not (tmp_path / "queue/claims/attempt-new.json").exists()


def test_exact_retirement_sidecar_cannot_skip_valid_pending_request(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    pending = _request(queue, tmp_path, attempt="attempt-old")
    request_path = tmp_path / "queue/requests/attempt-old.json"
    retirement_path = tmp_path / "queue/retirements/attempt-old.json"
    retirement_path.parent.mkdir(parents=True)
    retirement_path.write_text(json.dumps({
        "schema_version": 1,
        "execution_id": "exec-1",
        "review_attempt_id": "attempt-old",
        "status": "RETIRED_PENDING",
        "request_sha256": hashlib.sha256(request_path.read_bytes()).hexdigest(),
        "handoff_sha256": pending["request"]["handoff_sha256"],
        "packet_sha256": pending["request"]["packet_sha256"],
        "reason_code": "ABANDONED_INVALID_RETRY_LINEAGE",
        "reason": "Forged retirement of a valid request.",
        "retired_by": "forged-actor",
        "retired_at": "2026-08-09T00:00:00Z",
        "workflow_advance_allowed": False,
        "retry_eligible": False,
    }), encoding="utf-8")
    _request(queue, tmp_path, attempt="attempt-new")

    status = queue.status("exec-1", "attempt-old")
    assert status["state"] == "INVALID_EVIDENCE"
    assert "retirement_schema_or_binding_invalid" in status["integrity_errors"]
    with pytest.raises(ReviewQueueError, match="pending_retirement_invalid:attempt-old"):
        queue.claim_next(worker_id="transport")


def test_duplicate_retire_pending_race_has_one_winner(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    request_path = tmp_path / "queue/requests/attempt-1.json"
    request = json.loads(request_path.read_text(encoding="utf-8"))
    request["retry_of"] = "missing-attempt"
    request_path.write_text(json.dumps(request), encoding="utf-8")
    request_sha256 = hashlib.sha256(request_path.read_bytes()).hexdigest()
    barrier = threading.Barrier(2)

    def retire(worker: str) -> str:
        barrier.wait()
        try:
            queue.retire_pending(
                execution_id="exec-1", review_attempt_id="attempt-1",
                retired_by=worker, expected_request_sha256=request_sha256,
                reason="No longer required.",
            )
            return "retired"
        except ReviewQueueError as exc:
            return str(exc)

    with ThreadPoolExecutor(max_workers=2) as pool:
        outcomes = list(pool.map(retire, ("retirer-a", "retirer-b")))
    assert outcomes.count("retired") == 1
    assert sum(outcome == "review_already_retired" for outcome in outcomes) == 1


def test_orphan_lease_fails_closed_instead_of_claiming_next(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path, attempt="attempt-old")
    _request(queue, tmp_path, attempt="attempt-new")
    lease_path = tmp_path / "queue/singleflight_leases/attempt-old.json"
    lease_path.parent.mkdir(parents=True)
    lease_path.write_text(json.dumps({"schema_version": 2, "status": "CLAIMED"}), encoding="utf-8")
    with pytest.raises(ReviewQueueError, match="singleflight_lease_invalid"):
        queue.claim_next(worker_id="transport")
