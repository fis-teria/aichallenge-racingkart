from __future__ import annotations

import hashlib
import json
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

from aic_test.cli import build_parser
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


def _evidence(tmp_path: Path, attempts: int = 3) -> Path:
    return _file(tmp_path / "connection-evidence.json", json.dumps({"connection_attempts": [
        {"timestamp": f"2026-08-05T00:00:0{i}Z", "outcome": "DISCONNECTED", "timeout_s": 30}
        for i in range(attempts)
    ]}))


def test_round_trip_binds_execution_and_immutable_attempt(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    pending = _request(queue, tmp_path)
    assert pending["state"] == "REVIEW_PENDING"
    assert pending["queue_root"] == str((tmp_path / "queue").resolve())
    assert queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")["state"] == "CLAIMED"
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


def test_status_fails_closed_for_forged_result_and_tampered_response(tmp_path: Path) -> None:
    queue = ReviewQueue(tmp_path / "queue")
    _request(queue, tmp_path)
    queue.claim(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport")
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
    evidence, response = _evidence(tmp_path), _response(queue, tmp_path)
    barrier = threading.Barrier(2)

    def defer() -> str:
        barrier.wait()
        try:
            queue.defer(execution_id="exec-1", review_attempt_id="attempt-1", worker_id="transport", evidence=evidence,
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
    assert outcomes.count("terminal_conflict") == 1


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
