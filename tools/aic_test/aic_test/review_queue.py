from __future__ import annotations

import hashlib
import json
import os
import re
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 2
LEGACY_SCHEMA_VERSION = 1
ALLOWED_MODELS = ("GPT-5.6 Sol Pro", "GPT-5.6 Pro")
APPROVED_PROJECT_URL = "https://chatgpt.com/g/g-p-6a67ef32dd9c8191af3d8ee937ef668f-codexyong-wakusuhesu/project"
ALLOWED_SEVERITIES = ("Blocker", "Major", "Minor", "None")
ALLOWED_TRANSITIONS = ("GO", "HOLD")
CANONICAL_REVIEW_QUEUE_ROOT = (
    Path(__file__).resolve().parents[3] / "analysis/aic_test/external_review_queue"
)
_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}\Z")


class ReviewQueueError(RuntimeError):
    pass


def _now_utc() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _validate_id(value: str, field: str) -> str:
    if not _ID_RE.fullmatch(value):
        raise ReviewQueueError(f"invalid_{field}")
    return value


def _regular_file(path: Path, field: str) -> Path:
    resolved = path.resolve(strict=True)
    if path.is_symlink() or not resolved.is_file():
        raise ReviewQueueError(f"{field}_not_regular_file")
    return resolved


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json_bytes(payload: dict[str, Any]) -> bytes:
    try:
        return (json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2,
                           allow_nan=False) + "\n").encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise ReviewQueueError("json_payload_invalid") from exc


def _write_exclusive(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = _json_bytes(payload)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(mode="wb", prefix=f".{path.name}.",
                                         suffix=".tmp", dir=path.parent, delete=False) as stream:
            temporary_path = Path(stream.name)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_path, 0o644)
        try:
            os.link(temporary_path, path, follow_symlinks=False)
        except FileExistsError as exc:
            raise ReviewQueueError(f"record_already_exists:{path.name}") from exc
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def _read_json(path: Path, field: str) -> dict[str, Any]:
    resolved = _regular_file(path, field)
    try:
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ReviewQueueError(f"{field}_invalid_json") from exc
    if not isinstance(payload, dict):
        raise ReviewQueueError(f"{field}_invalid_schema")
    return payload


def _review_envelope(path: Path) -> dict[str, Any]:
    payload = _read_json(path, "response")
    required = {"schema_version", "execution_id", "review_attempt_id", "handoff_sha256",
                "model", "model_fallback_reason", "project_url", "submission_id",
                "submission_verified", "severity", "disposition", "minimum_changes",
                "transition_permission", "review_text"}
    if set(payload) != required or payload.get("schema_version") != SCHEMA_VERSION:
        raise ReviewQueueError("response_schema_invalid")
    if not isinstance(payload.get("review_text"), str) or not payload["review_text"].strip():
        raise ReviewQueueError("response_review_text_missing")
    return payload


class ReviewQueue:
    def __init__(self, root: Path):
        self.root = root.resolve()

    def _record(self, collection: str, review_attempt_id: str) -> Path:
        return self.root / collection / f"{_validate_id(review_attempt_id, 'review_attempt_id')}.json"

    def _legacy_record(self, collection: str, execution_id: str) -> Path:
        return self.root / collection / f"{_validate_id(execution_id, 'execution_id')}.json"

    def enqueue(self, *, execution_id: str, review_attempt_id: str, handoff: Path, packet: Path,
                objective: str, requested_by: str, retry_of: str | None = None) -> dict[str, Any]:
        execution_id = _validate_id(execution_id, "execution_id")
        review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
        requested_by = _validate_id(requested_by, "requested_by")
        if not objective.strip() or len(objective) > 4096:
            raise ReviewQueueError("invalid_objective")
        handoff = _regular_file(handoff, "handoff")
        packet = _regular_file(packet, "packet")
        handoff_sha256, packet_sha256 = _sha256(handoff), _sha256(packet)
        if retry_of is not None:
            retry_of = _validate_id(retry_of, "retry_of")
            self._validate_retry(retry_of, execution_id, handoff_sha256, packet_sha256)
        payload = {
            "schema_version": SCHEMA_VERSION, "execution_id": execution_id,
            "review_attempt_id": review_attempt_id, "retry_of": retry_of,
            "status": "REVIEW_PENDING", "requested_by": requested_by, "objective": objective,
            "handoff_path": str(handoff), "handoff_sha256": handoff_sha256,
            "packet_path": str(packet), "packet_sha256": packet_sha256,
            "approved_models": list(ALLOWED_MODELS), "approved_project_url": APPROVED_PROJECT_URL,
            "transport": "Playwright MCP only", "created_at": _now_utc(),
        }
        _write_exclusive(self._record("requests", review_attempt_id), payload)
        return self.status(execution_id, review_attempt_id)

    def _validate_retry(self, retry_of: str, execution_id: str, handoff_sha256: str,
                        packet_sha256: str) -> None:
        try:
            request = _read_json(self._record("requests", retry_of), "prior_request")
            terminal_path = self._record("terminals", retry_of)
            deferral_path = terminal_path if terminal_path.exists() else self._record("deferrals", retry_of)
            deferral = _read_json(deferral_path, "prior_deferral")
        except (OSError, ReviewQueueError) as exc:
            raise ReviewQueueError("retry_prior_deferral_invalid") from exc
        if self._record("results", retry_of).exists():
            raise ReviewQueueError("retry_prior_terminal_conflict")
        if terminal_path.exists() and deferral.get("terminal_kind") == "COMPLETED":
            raise ReviewQueueError("retry_prior_terminal_conflict")
        is_legacy = request.get("schema_version") == LEGACY_SCHEMA_VERSION
        valid = (not self._legacy_request_errors(request, execution_id, retry_of)
                 and self._legacy_deferral_valid(deferral, request, deferral_path, retry_of)) if is_legacy else (
                     not self._request_schema_errors(request, execution_id, retry_of)
                     and self._deferral_valid(deferral, request, deferral_path))
        if not valid:
            raise ReviewQueueError("retry_prior_deferral_invalid")
        if request["handoff_sha256"] != handoff_sha256 or request["packet_sha256"] != packet_sha256:
            raise ReviewQueueError("retry_source_hash_mismatch")

    def claim(self, *, execution_id: str, review_attempt_id: str, worker_id: str) -> dict[str, Any]:
        worker_id = _validate_id(worker_id, "worker_id")
        request_path = self._record("requests", review_attempt_id)
        request = _read_json(request_path, "request")
        if (self._request_schema_errors(request, execution_id, review_attempt_id)
                or not self._retry_lineage_valid(request) or self._source_integrity_errors(request)):
            raise ReviewQueueError("request_source_integrity_invalid")
        if self._record("results", review_attempt_id).exists() or self._record("deferrals", review_attempt_id).exists():
            raise ReviewQueueError("review_already_terminal")
        payload = {"schema_version": SCHEMA_VERSION, "execution_id": execution_id,
                   "review_attempt_id": review_attempt_id, "request_sha256": _sha256(request_path),
                   "handoff_sha256": request["handoff_sha256"], "worker_id": worker_id,
                   "status": "CLAIMED", "claimed_at": _now_utc()}
        _write_exclusive(self._record("claims", review_attempt_id), payload)
        return self.status(execution_id, review_attempt_id)

    def defer(self, *, execution_id: str, review_attempt_id: str, worker_id: str,
              evidence: Path, evidence_sha256: str, terminal_connection_error: str | None = None) -> dict[str, Any]:
        if self._record("results", review_attempt_id).exists():
            raise ReviewQueueError("review_already_completed")
        self._reject_existing_terminal(review_attempt_id, "DEFERRED_CONNECTION_UNAVAILABLE")
        try:
            request_path, request, claim = self._bound_claim(execution_id, review_attempt_id, worker_id)
            legacy = False
        except ReviewQueueError as exc:
            if str(exc) != "request_source_integrity_invalid":
                raise
            request_path, request, claim = self._legacy_bound_claim(execution_id, review_attempt_id, worker_id)
            legacy = True
        evidence = _regular_file(evidence, "connection_evidence")
        if not re.fullmatch(r"[0-9a-f]{64}", evidence_sha256) or _sha256(evidence) != evidence_sha256:
            raise ReviewQueueError("connection_evidence_hash_mismatch")
        evidence_payload = _read_json(evidence, "connection_evidence")
        attempts = evidence_payload.get("connection_attempts")
        if not isinstance(attempts, list) or not all(self._connection_attempt_valid(item) for item in attempts):
            raise ReviewQueueError("connection_attempts_invalid")
        if len(attempts) < 3:
            raise ReviewQueueError("connection_attempts_insufficient")
        if terminal_connection_error is not None and not terminal_connection_error.strip():
            raise ReviewQueueError("terminal_connection_error_invalid")
        payload = {"schema_version": SCHEMA_VERSION, "execution_id": execution_id,
                   "review_attempt_id": review_attempt_id, "status": "DEFERRED_CONNECTION_UNAVAILABLE",
                   "resume_action": "REVIEW_SAME_IMMUTABLE_EXECUTION", "request_sha256": _sha256(request_path),
                   "handoff_sha256": request["handoff_sha256"], "packet_sha256": request["packet_sha256"],
                   "worker_id": worker_id, "claim_sha256": _sha256(self._record("claims", review_attempt_id)),
                   "connection_evidence_path": str(evidence), "connection_evidence_sha256": evidence_sha256,
                   "connection_attempts": attempts, "terminal_connection_error": terminal_connection_error,
                   "workflow_advance_allowed": False, "deferred_at": _now_utc(),
                   "legacy_claim": legacy, "terminal_kind": "DEFERRED_CONNECTION_UNAVAILABLE"}
        payload["claim_sha256"] = _sha256(self._legacy_record("claims", execution_id) if legacy else self._record("claims", review_attempt_id))
        self._write_terminal_marker(payload)
        return self.status(execution_id, review_attempt_id)

    @staticmethod
    def _connection_attempt_valid(item: Any) -> bool:
        return (isinstance(item, dict) and set(item) == {"timestamp", "outcome", "timeout_s"}
                and isinstance(item["timestamp"], str) and bool(item["timestamp"].strip())
                and isinstance(item["outcome"], str) and bool(item["outcome"].strip())
                and type(item["timeout_s"]) in (int, float) and 0 < item["timeout_s"] <= 1800)

    def _bound_claim(self, execution_id: str, review_attempt_id: str, worker_id: str) -> tuple[Path, dict[str, Any], dict[str, Any]]:
        request_path = self._record("requests", review_attempt_id)
        request, claim = _read_json(request_path, "request"), _read_json(self._record("claims", review_attempt_id), "claim")
        if (self._request_schema_errors(request, execution_id, review_attempt_id)
                or not self._retry_lineage_valid(request) or self._source_integrity_errors(request)):
            raise ReviewQueueError("request_source_integrity_invalid")
        if not self._claim_valid(claim, request, request_path, review_attempt_id) or claim["worker_id"] != worker_id:
            raise ReviewQueueError("claim_owner_mismatch")
        return request_path, request, claim

    def _legacy_bound_claim(self, execution_id: str, review_attempt_id: str, worker_id: str) -> tuple[Path, dict[str, Any], dict[str, Any]]:
        if review_attempt_id != execution_id:
            raise ReviewQueueError("legacy_review_attempt_id_mismatch")
        request_path = self._legacy_record("requests", execution_id)
        claim_path = self._legacy_record("claims", execution_id)
        request, claim = _read_json(request_path, "request"), _read_json(claim_path, "claim")
        if self._legacy_request_errors(request, execution_id, review_attempt_id) or self._source_integrity_errors(request):
            raise ReviewQueueError("request_source_integrity_invalid")
        if not self._legacy_claim_valid(claim, request, request_path) or claim.get("worker_id") != worker_id:
            raise ReviewQueueError("claim_owner_mismatch")
        return request_path, request, claim

    def complete(self, *, execution_id: str, review_attempt_id: str, worker_id: str, model: str,
                 severity: str, disposition: str, transition_permission: str, response: Path,
                 minimum_changes: list[str], submission_id: str, project_url: str,
                 fallback_reason: str | None = None) -> dict[str, Any]:
        if model not in ALLOWED_MODELS or severity not in ALLOWED_SEVERITIES or transition_permission not in ALLOWED_TRANSITIONS:
            raise ReviewQueueError("completion_value_invalid")
        if not disposition.strip() or len(disposition) > 4096 or not all(isinstance(i, str) and i.strip() for i in minimum_changes):
            raise ReviewQueueError("disposition_or_minimum_changes_invalid")
        submission_id, worker_id = _validate_id(submission_id, "submission_id"), _validate_id(worker_id, "worker_id")
        if project_url != APPROVED_PROJECT_URL or (model == "GPT-5.6 Pro") != bool(fallback_reason and fallback_reason.strip()):
            raise ReviewQueueError("completion_provenance_invalid")
        if self._record("deferrals", review_attempt_id).exists():
            raise ReviewQueueError("review_already_deferred")
        self._reject_existing_terminal(review_attempt_id, "COMPLETED")
        request_path, request, _ = self._bound_claim(execution_id, review_attempt_id, worker_id)
        response = _regular_file(response, "response")
        envelope = _review_envelope(response)
        expected = {"execution_id": execution_id, "review_attempt_id": review_attempt_id,
                    "handoff_sha256": request["handoff_sha256"], "model": model,
                    "model_fallback_reason": fallback_reason, "project_url": project_url,
                    "submission_id": submission_id, "submission_verified": True, "severity": severity,
                    "disposition": disposition, "minimum_changes": minimum_changes,
                    "transition_permission": transition_permission}
        if any(envelope.get(key) != value for key, value in expected.items()):
            raise ReviewQueueError("response_disposition_or_provenance_mismatch")
        payload = {"schema_version": SCHEMA_VERSION, "execution_id": execution_id,
                   "review_attempt_id": review_attempt_id, "status": "COMPLETED",
                   "request_sha256": _sha256(request_path), "handoff_sha256": request["handoff_sha256"],
                   "packet_sha256": request["packet_sha256"], "worker_id": worker_id, "model": model,
                   "model_fallback_reason": fallback_reason, "project_url": project_url,
                   "submission_id": submission_id, "submission_verified": True, "severity": severity,
                   "disposition": disposition, "transition_permission": transition_permission,
                   "minimum_changes": minimum_changes, "response_path": str(response),
                   "response_sha256": _sha256(response), "advisory_only": True,
                   "workflow_advance_allowed": False, "completed_at": _now_utc(),
                   "terminal_kind": "COMPLETED", "claim_sha256": _sha256(self._record("claims", review_attempt_id))}
        self._write_terminal_marker(payload)
        return self.status(execution_id, review_attempt_id)

    def _write_terminal_marker(self, payload: dict[str, Any]) -> None:
        review_attempt_id = str(payload.get("review_attempt_id", ""))
        try:
            _write_exclusive(self._record("terminals", review_attempt_id), payload)
        except ReviewQueueError as exc:
            if str(exc).startswith("record_already_exists:"):
                raise ReviewQueueError("terminal_conflict") from exc
            raise

    def _reject_existing_terminal(self, review_attempt_id: str, requested_kind: str) -> None:
        marker_path = self._record("terminals", review_attempt_id)
        if not marker_path.exists():
            return
        try:
            existing = _read_json(marker_path, "terminal_marker")
            kind = existing.get("terminal_kind")
        except ReviewQueueError:
            raise ReviewQueueError("terminal_conflict")
        if kind == "COMPLETED":
            raise ReviewQueueError("review_already_completed")
        if kind == "DEFERRED_CONNECTION_UNAVAILABLE":
            raise ReviewQueueError("review_already_deferred")
        raise ReviewQueueError("terminal_conflict")

    @staticmethod
    def _source_integrity_errors(request: dict[str, Any]) -> list[str]:
        errors: list[str] = []
        for field in ("handoff", "packet"):
            try:
                path = _regular_file(Path(str(request[f"{field}_path"])), field)
                if _sha256(path) != request.get(f"{field}_sha256"):
                    errors.append(f"{field}_hash_mismatch")
            except (KeyError, OSError, ReviewQueueError):
                errors.append(f"{field}_unavailable")
        return errors

    def _retry_lineage_valid(self, request: dict[str, Any]) -> bool:
        """Validate only the direct predecessor; do not recurse through retry chains."""
        retry_of = request.get("retry_of")
        if retry_of is None:
            return True
        if not isinstance(retry_of, str) or not _ID_RE.fullmatch(retry_of):
            return False
        try:
            prior_request = _read_json(self._record("requests", retry_of), "prior_request")
            terminal_path = self._record("terminals", retry_of)
            prior_deferral = _read_json(terminal_path if terminal_path.exists() else self._record("deferrals", retry_of), "prior_deferral")
        except (OSError, ReviewQueueError):
            return False
        if self._record("results", retry_of).exists():
            return False
        if terminal_path.exists() and prior_deferral.get("terminal_kind") == "COMPLETED":
            return False
        is_legacy = prior_request.get("schema_version") == LEGACY_SCHEMA_VERSION
        if is_legacy:
            prior_valid = (not self._legacy_request_errors(prior_request, request.get("execution_id", ""), retry_of)
                           and self._legacy_deferral_valid(prior_deferral, prior_request,
                                                           self._record("deferrals", retry_of), retry_of))
        else:
            prior_valid = (not self._request_schema_errors(prior_request, request.get("execution_id", ""), retry_of)
                           and self._deferral_valid(prior_deferral, prior_request,
                                                    self._record("deferrals", retry_of)))
        return (prior_valid
                and prior_request.get("handoff_sha256") == request.get("handoff_sha256")
                and prior_request.get("packet_sha256") == request.get("packet_sha256"))

    def _prior_deferral_marker_valid(self, request: dict[str, Any], review_attempt_id: str,
                                     deferral: dict[str, Any], legacy: bool) -> bool:
        marker_path = self._record("terminals", review_attempt_id)
        if not marker_path.exists():
            return legacy  # Existing schema-v1 deferrals predate terminal markers.
        try:
            marker = _read_json(marker_path, "terminal_marker")
            claim_path = self._legacy_record("claims", request["execution_id"]) if legacy else self._record("claims", review_attempt_id)
            claim = _read_json(claim_path, "claim")
        except (KeyError, OSError, ReviewQueueError):
            return False
        return self._terminal_marker_valid(marker, request,
                                           self._legacy_record("requests", request["execution_id"]) if legacy else self._record("requests", review_attempt_id),
                                           claim, claim_path, request["execution_id"], review_attempt_id,
                                           "DEFERRED_CONNECTION_UNAVAILABLE")

    @staticmethod
    def _request_schema_errors(request: dict[str, Any], execution_id: str, review_attempt_id: str) -> list[str]:
        required = {"schema_version", "execution_id", "review_attempt_id", "retry_of", "status", "requested_by", "objective", "handoff_path", "handoff_sha256", "packet_path", "packet_sha256", "approved_models", "approved_project_url", "transport", "created_at"}
        valid = (set(request) == required and request.get("schema_version") == SCHEMA_VERSION
                 and request.get("execution_id") == execution_id and request.get("review_attempt_id") == review_attempt_id
                 and request.get("status") == "REVIEW_PENDING" and request.get("requested_by") in ("vscode", "bridge")
                 and isinstance(request.get("objective"), str) and bool(request["objective"].strip())
                 and request.get("approved_models") == list(ALLOWED_MODELS)
                 and request.get("approved_project_url") == APPROVED_PROJECT_URL and request.get("transport") == "Playwright MCP only")
        return [] if valid else ["request_schema_or_binding_invalid"]

    @staticmethod
    def _legacy_request_errors(request: dict[str, Any], execution_id: str, review_attempt_id: str) -> list[str]:
        required = {"schema_version", "execution_id", "status", "requested_by", "objective", "handoff_path", "handoff_sha256", "packet_path", "packet_sha256", "approved_models", "approved_project_url", "transport", "created_at"}
        valid = (review_attempt_id == execution_id and set(request) == required
                 and request.get("schema_version") == LEGACY_SCHEMA_VERSION and request.get("execution_id") == execution_id
                 and request.get("status") == "REVIEW_PENDING" and request.get("requested_by") in ("vscode", "bridge")
                 and isinstance(request.get("objective"), str) and bool(request["objective"].strip())
                 and request.get("approved_models") == list(ALLOWED_MODELS)
                 and request.get("approved_project_url") == APPROVED_PROJECT_URL and request.get("transport") == "Playwright MCP only")
        return [] if valid else ["legacy_request_schema_invalid"]

    @staticmethod
    def _claim_valid(claim: dict[str, Any], request: dict[str, Any], request_path: Path, review_attempt_id: str) -> bool:
        return (claim.get("schema_version") == SCHEMA_VERSION and claim.get("execution_id") == request.get("execution_id")
                and claim.get("review_attempt_id") == review_attempt_id and claim.get("status") == "CLAIMED"
                and claim.get("request_sha256") == _sha256(request_path) and claim.get("handoff_sha256") == request.get("handoff_sha256")
                and isinstance(claim.get("worker_id"), str) and bool(_ID_RE.fullmatch(claim["worker_id"])))

    @staticmethod
    def _legacy_claim_valid(claim: dict[str, Any], request: dict[str, Any], request_path: Path) -> bool:
        required = {"schema_version", "execution_id", "request_sha256", "handoff_sha256", "worker_id", "status", "claimed_at"}
        return (set(claim) == required and claim.get("schema_version") == LEGACY_SCHEMA_VERSION
                and claim.get("execution_id") == request.get("execution_id") and claim.get("status") == "CLAIMED"
                and claim.get("request_sha256") == _sha256(request_path)
                and claim.get("handoff_sha256") == request.get("handoff_sha256")
                and isinstance(claim.get("worker_id"), str) and bool(_ID_RE.fullmatch(claim["worker_id"])))

    def _deferral_valid(self, deferral: dict[str, Any], request: dict[str, Any], path: Path) -> bool:
        try:
            evidence = _regular_file(Path(str(deferral["connection_evidence_path"])), "connection_evidence")
            evidence_payload = _read_json(evidence, "connection_evidence")
            evidence_valid = (_sha256(evidence) == deferral["connection_evidence_sha256"]
                              and evidence_payload.get("connection_attempts") == deferral.get("connection_attempts"))
            claim_path = self._record("claims", request["review_attempt_id"])
            claim = _read_json(claim_path, "claim")
            claim_valid = (self._claim_valid(claim, request, self._record("requests", request["review_attempt_id"]),
                                             request["review_attempt_id"])
                           and claim.get("worker_id") == deferral.get("worker_id")
                           and _sha256(claim_path) == deferral.get("claim_sha256"))
        except (KeyError, OSError, ReviewQueueError):
            evidence_valid = claim_valid = False
        required = {"schema_version", "execution_id", "review_attempt_id", "status", "resume_action",
                    "request_sha256", "handoff_sha256", "packet_sha256", "worker_id", "claim_sha256",
                    "connection_evidence_path", "connection_evidence_sha256", "connection_attempts",
                    "terminal_connection_error", "workflow_advance_allowed", "deferred_at", "legacy_claim", "terminal_kind"}
        return (set(deferral) == required and deferral.get("schema_version") == SCHEMA_VERSION and deferral.get("execution_id") == request.get("execution_id")
                and deferral.get("review_attempt_id") == request.get("review_attempt_id")
                and deferral.get("status") == "DEFERRED_CONNECTION_UNAVAILABLE"
                and deferral.get("resume_action") == "REVIEW_SAME_IMMUTABLE_EXECUTION"
                and deferral.get("request_sha256") == _sha256(self._record("requests", request["review_attempt_id"]))
                and deferral.get("handoff_sha256") == request.get("handoff_sha256") and deferral.get("packet_sha256") == request.get("packet_sha256")
                and deferral.get("workflow_advance_allowed") is False and deferral.get("legacy_claim") is False
                and deferral.get("terminal_kind") == "DEFERRED_CONNECTION_UNAVAILABLE" and evidence_valid and claim_valid
                and isinstance(deferral.get("connection_attempts"), list)
                and all(self._connection_attempt_valid(item) for item in deferral["connection_attempts"])
                and len(deferral["connection_attempts"]) >= 3)

    def _legacy_deferral_valid(self, deferral: dict[str, Any], request: dict[str, Any], path: Path,
                               review_attempt_id: str) -> bool:
        try:
            evidence = _regular_file(Path(str(deferral["connection_evidence_path"])), "connection_evidence")
            evidence_payload = _read_json(evidence, "connection_evidence")
            claim_path = self._legacy_record("claims", request["execution_id"])
            claim = _read_json(claim_path, "claim")
            bindings_valid = (_sha256(evidence) == deferral["connection_evidence_sha256"]
                              and evidence_payload.get("connection_attempts") == deferral.get("connection_attempts")
                              and self._legacy_claim_valid(claim, request, self._legacy_record("requests", request["execution_id"]))
                              and claim.get("worker_id") == deferral.get("worker_id")
                              and _sha256(claim_path) == deferral.get("claim_sha256"))
        except (KeyError, OSError, ReviewQueueError):
            bindings_valid = False
        required = {"schema_version", "execution_id", "review_attempt_id", "status", "resume_action",
                    "request_sha256", "handoff_sha256", "packet_sha256", "worker_id", "claim_sha256",
                    "connection_evidence_path", "connection_evidence_sha256", "connection_attempts",
                    "terminal_connection_error", "workflow_advance_allowed", "deferred_at", "legacy_claim"}
        terminal_required = required | {"terminal_kind"}
        return (set(deferral) in (required, terminal_required) and deferral.get("schema_version") == SCHEMA_VERSION
                and deferral.get("execution_id") == request.get("execution_id")
                and deferral.get("review_attempt_id") == review_attempt_id
                and deferral.get("status") == "DEFERRED_CONNECTION_UNAVAILABLE"
                and deferral.get("resume_action") == "REVIEW_SAME_IMMUTABLE_EXECUTION"
                and deferral.get("request_sha256") == _sha256(self._legacy_record("requests", request["execution_id"]))
                and deferral.get("handoff_sha256") == request.get("handoff_sha256")
                and deferral.get("packet_sha256") == request.get("packet_sha256")
                and deferral.get("workflow_advance_allowed") is False and deferral.get("legacy_claim") is True
                and deferral.get("terminal_kind", "DEFERRED_CONNECTION_UNAVAILABLE") == "DEFERRED_CONNECTION_UNAVAILABLE"
                and bindings_valid and isinstance(deferral.get("connection_attempts"), list)
                and all(self._connection_attempt_valid(item) for item in deferral["connection_attempts"])
                and len(deferral["connection_attempts"]) >= 3)

    def _terminal_marker_valid(self, marker: dict[str, Any], request: dict[str, Any], request_path: Path,
                               claim: dict[str, Any], claim_path: Path, execution_id: str,
                               review_attempt_id: str, terminal_kind: str) -> bool:
        return (marker.get("schema_version") == SCHEMA_VERSION
                and marker.get("execution_id") == execution_id and marker.get("review_attempt_id") == review_attempt_id
                and marker.get("terminal_kind") == terminal_kind and marker.get("request_sha256") == _sha256(request_path)
                and marker.get("handoff_sha256") == request.get("handoff_sha256")
                and marker.get("claim_sha256") == _sha256(claim_path) and marker.get("worker_id") == claim.get("worker_id"))

    def _result_valid(self, result: dict[str, Any], request: dict[str, Any], request_path: Path,
                      claim: dict[str, Any], execution_id: str, review_attempt_id: str) -> bool:
        try:
            response = _regular_file(Path(str(result["response_path"])), "response")
            envelope = _review_envelope(response)
        except (KeyError, OSError, UnicodeError, ReviewQueueError):
            return False
        required = {"schema_version", "execution_id", "review_attempt_id", "status", "request_sha256",
                    "handoff_sha256", "packet_sha256", "worker_id", "model", "model_fallback_reason",
                    "project_url", "submission_id", "submission_verified", "severity", "disposition",
                    "transition_permission", "minimum_changes", "response_path", "response_sha256",
                    "advisory_only", "workflow_advance_allowed", "completed_at", "terminal_kind", "claim_sha256"}
        fields = ("execution_id", "review_attempt_id", "handoff_sha256", "model", "model_fallback_reason",
                  "project_url", "submission_id", "submission_verified", "severity", "disposition",
                  "minimum_changes", "transition_permission")
        return (set(result) == required and result.get("schema_version") == SCHEMA_VERSION
                and result.get("execution_id") == execution_id and result.get("review_attempt_id") == review_attempt_id
                and result.get("status") == "COMPLETED" and result.get("request_sha256") == _sha256(request_path)
                and result.get("handoff_sha256") == request.get("handoff_sha256")
                and result.get("packet_sha256") == request.get("packet_sha256") and result.get("worker_id") == claim.get("worker_id")
                and result.get("claim_sha256") == _sha256(self._record("claims", review_attempt_id))
                and result.get("model") in ALLOWED_MODELS and result.get("severity") in ALLOWED_SEVERITIES
                and result.get("transition_permission") in ALLOWED_TRANSITIONS and result.get("project_url") == APPROVED_PROJECT_URL
                and result.get("submission_verified") is True and isinstance(result.get("submission_id"), str)
                and bool(_ID_RE.fullmatch(result.get("submission_id", ""))) and result.get("advisory_only") is True
                and result.get("workflow_advance_allowed") is False and result.get("terminal_kind") == "COMPLETED" and isinstance(result.get("disposition"), str)
                and bool(result["disposition"].strip()) and isinstance(result.get("minimum_changes"), list)
                and all(isinstance(item, str) and item.strip() for item in result["minimum_changes"])
                and ((result.get("model") == "GPT-5.6 Pro") == bool(result.get("model_fallback_reason")))
                and _sha256(response) == result.get("response_sha256")
                and all(envelope.get(field) == result.get(field) for field in fields))

    def status(self, execution_id: str, review_attempt_id: str | None = None) -> dict[str, Any]:
        execution_id = _validate_id(execution_id, "execution_id")
        if review_attempt_id is None:
            return self._legacy_status(execution_id)
        review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
        request_path = self._record("requests", review_attempt_id)
        request = _read_json(request_path, "request")
        if request.get("schema_version") == LEGACY_SCHEMA_VERSION:
            return self._legacy_status(execution_id, review_attempt_id)
        errors = self._request_schema_errors(request, execution_id, review_attempt_id) + self._source_integrity_errors(request)
        if not self._retry_lineage_valid(request):
            errors.append("retry_lineage_invalid")
        claim_path, result_path, deferral_path = (self._record(c, review_attempt_id) for c in ("claims", "results", "deferrals"))
        marker_path = self._record("terminals", review_attempt_id)
        claim = _read_json(claim_path, "claim") if claim_path.exists() else None
        result = _read_json(result_path, "result") if result_path.exists() else None
        deferral = _read_json(deferral_path, "deferral") if deferral_path.exists() else None
        marker = _read_json(marker_path, "terminal_marker") if marker_path.exists() else None
        if marker and marker.get("terminal_kind") == "COMPLETED":
            if result or deferral:
                errors.append("terminal_records_conflict")
            result = marker
        elif marker and marker.get("terminal_kind") == "DEFERRED_CONNECTION_UNAVAILABLE":
            if result or deferral:
                errors.append("terminal_records_conflict")
            deferral = marker
        elif marker:
            errors.append("terminal_marker_invalid")
        if claim and not self._claim_valid(claim, request, request_path, review_attempt_id): errors.append("claim_schema_or_binding_invalid")
        if deferral and (not claim or not self._deferral_valid(deferral, request, deferral_path)): errors.append("deferral_schema_or_binding_invalid")
        if result and (not claim or not self._result_valid(result, request, request_path, claim, execution_id, review_attempt_id)):
            errors.append("result_schema_or_binding_invalid")
        if result and deferral:
            errors.append("terminal_records_conflict")
        if (result or deferral) and marker is None:
            errors.append("terminal_marker_missing")
        if marker is not None:
            if not claim:
                errors.append("terminal_marker_invalid")
            elif result:
                if not self._terminal_marker_valid(marker, request, request_path, claim, claim_path, execution_id, review_attempt_id, "COMPLETED"):
                    errors.append("terminal_marker_invalid")
            elif deferral:
                if not self._terminal_marker_valid(marker, request, request_path, claim, claim_path, execution_id, review_attempt_id, "DEFERRED_CONNECTION_UNAVAILABLE"):
                    errors.append("terminal_marker_invalid")
        state = "COMPLETED" if result else ("DEFERRED_CONNECTION_UNAVAILABLE" if deferral else ("CLAIMED" if claim else "REVIEW_PENDING"))
        return {"schema_version": SCHEMA_VERSION, "queue_root": str(self.root), "execution_id": execution_id, "review_attempt_id": review_attempt_id,
                "state": "INVALID_EVIDENCE" if errors else state, "integrity_valid": not errors,
                "integrity_errors": errors, "request": request, "claim": claim, "result": result,
                "deferral": deferral, "workflow_advance_allowed": False}

    def _legacy_status(self, execution_id: str, review_attempt_id: str | None = None) -> dict[str, Any]:
        """Read the immutable poc-03 format without permitting it to be claimed or changed."""
        request_path = self._legacy_record("requests", execution_id)
        request = _read_json(request_path, "request")
        attempt_id = review_attempt_id or execution_id
        errors = self._legacy_request_errors(request, execution_id, attempt_id)
        errors.extend(self._source_integrity_errors(request))
        claim_path, result_path = self._legacy_record("claims", execution_id), self._legacy_record("results", execution_id)
        claim = _read_json(claim_path, "claim") if claim_path.exists() else None
        result = _read_json(result_path, "result") if result_path.exists() else None
        deferral_path = self._record("deferrals", attempt_id)
        marker_path = self._record("terminals", attempt_id)
        deferral = _read_json(deferral_path, "deferral") if deferral_path.exists() else None
        marker = _read_json(marker_path, "terminal_marker") if marker_path.exists() else None
        if marker and marker.get("terminal_kind") == "COMPLETED":
            if result or deferral:
                errors.append("terminal_records_conflict")
            result = marker
        elif marker and marker.get("terminal_kind") == "DEFERRED_CONNECTION_UNAVAILABLE":
            if result or deferral:
                errors.append("terminal_records_conflict")
            deferral = marker
        elif marker:
            errors.append("terminal_marker_invalid")
        if claim and not self._legacy_claim_valid(claim, request, request_path):
            errors.append("legacy_claim_schema_or_binding_invalid")
        if deferral and not self._legacy_deferral_valid(deferral, request, deferral_path, attempt_id):
            errors.append("deferral_schema_or_binding_invalid")
        if result and deferral:
            errors.append("terminal_records_conflict")
        if marker is not None:
            if not claim:
                errors.append("terminal_marker_invalid")
            elif deferral:
                if not self._terminal_marker_valid(marker, request, request_path, claim, claim_path, execution_id, attempt_id, "DEFERRED_CONNECTION_UNAVAILABLE"):
                    errors.append("terminal_marker_invalid")
            elif result:
                if not self._terminal_marker_valid(marker, request, request_path, claim, claim_path, execution_id, attempt_id, "COMPLETED"):
                    errors.append("terminal_marker_invalid")
        state = "COMPLETED" if result else ("DEFERRED_CONNECTION_UNAVAILABLE" if deferral else ("CLAIMED" if claim else "REVIEW_PENDING"))
        return {"schema_version": LEGACY_SCHEMA_VERSION, "queue_root": str(self.root), "execution_id": execution_id, "review_attempt_id": attempt_id,
                "state": "INVALID_EVIDENCE" if errors else state, "integrity_valid": not errors,
                "integrity_errors": errors, "request": request, "claim": claim, "result": result,
                "deferral": deferral, "workflow_advance_allowed": False, "legacy_read_only": True}
