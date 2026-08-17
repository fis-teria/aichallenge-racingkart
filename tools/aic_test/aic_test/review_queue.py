from __future__ import annotations

import hashlib
import json
import os
import re
import stat
import tempfile
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Iterator

import fcntl


SCHEMA_VERSION = 2
LEGACY_SCHEMA_VERSION = 1
ALLOWED_MODELS = ("GPT-5.6 Sol Pro", "GPT-5.6 Pro")
APPROVED_PROJECT_URL = "https://chatgpt.com/g/g-p-6a67ef32dd9c8191af3d8ee937ef668f-codexyong-wakusuhesu/project"
ALLOWED_SEVERITIES = ("Blocker", "Major", "Minor", "None")
ALLOWED_TRANSITIONS = ("GO", "HOLD")
RATE_LIMIT_RETRY_DELAY = timedelta(minutes=10)
QUEUE_TERMINAL_COOLDOWN = timedelta(seconds=60)
ALLOWED_CONNECTION_UNAVAILABLE_OUTCOMES = (
    "BROWSER_UNAVAILABLE", "CONNECTION_UNAVAILABLE", "DISCONNECTED",
    "PROJECT_UNAVAILABLE",
)
LEGACY_BROWSER_PROFILE_IN_USE_OUTCOME = (
    "Browser profile already in use; Playwright could not enumerate tabs"
)
CANONICAL_REVIEW_QUEUE_ROOT = (
    Path(__file__).resolve().parents[3] / "analysis/aic_test/external_review_queue"
)
SINGLEFLIGHT_LEASE_COLLECTION = "singleflight_leases"
SUBMISSION_INTENT_COLLECTION = "submission_intents"
RETIREMENTS_COLLECTION = "retirements"
RETIREMENT_REASON_CODE = "ABANDONED_INVALID_RETRY_LINEAGE"
QUEUE_LOCK_NAME = "queue.lock"
GATE2_AUTHORIZATION_COLLECTION = "gate2_authorizations"
GATE2_RUNTIME_BUDGET = {
    "fresh_runtime_max": 1,
    "fresh_runtime_used": 0,
    "process_group_timeout_s": 120,
    "retry_allowed": False,
}
GATE2_LAUNCH_SERVICE_KEYS = frozenset({
    "image", "command", "entrypoint", "environment", "mounts", "privileged",
    "network_mode", "security_opt", "cap_add", "read_only", "devices",
    "working_dir", "stop_signal", "stop_grace_period", "pull_policy",
})
_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}\Z")


class ReviewQueueError(RuntimeError):
    pass


def _now_utc() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _parse_utc(value: Any) -> datetime | None:
    if not isinstance(value, str) or not value.endswith("Z"):
        return None
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError:
        return None
    return parsed if parsed.tzinfo is not None else None


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

    @contextmanager
    def _queue_lock(self) -> Iterator[None]:
        """Serialize queue arbitration and terminal transitions.

        The lock is deliberately a small, queue-local file lock.  It protects
        the arbitration decision across independent Bridge/VS Code processes;
        the immutable JSON records remain the durable state and recovery
        surface if a transport process exits while a request is submitted.
        """
        self.root.mkdir(parents=True, exist_ok=True)
        lock_path = self.root / QUEUE_LOCK_NAME
        with lock_path.open("a+") as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

    def _record(self, collection: str, review_attempt_id: str) -> Path:
        return self.root / collection / f"{_validate_id(review_attempt_id, 'review_attempt_id')}.json"

    def _legacy_record(self, collection: str, execution_id: str) -> Path:
        return self.root / collection / f"{_validate_id(execution_id, 'execution_id')}.json"

    def _singleflight_lease_path(self, review_attempt_id: str) -> Path:
        return self._record(SINGLEFLIGHT_LEASE_COLLECTION, review_attempt_id)

    def _submission_intent_path(self, review_attempt_id: str) -> Path:
        return self._record(SUBMISSION_INTENT_COLLECTION, review_attempt_id)

    def _retirement_path(self, review_attempt_id: str) -> Path:
        return self._record(RETIREMENTS_COLLECTION, review_attempt_id)

    def gate2_authorization_path(self, review_attempt_id: str) -> Path:
        """Return the fixed Gate2 authorization-index path for a review attempt.

        The path is derived from the canonical queue root and the validated attempt
        id.  Callers never choose a state/result path or hash; those are recovered
        from the queue records below.
        """
        return self._record(GATE2_AUTHORIZATION_COLLECTION, review_attempt_id)

    def gate2_preflight_bundle_path(self, run_id: str) -> Path:
        """Return the deterministic preflight bundle path for one Gate2 run."""
        return self.root / "preflight_bundles" / f"{_validate_id(run_id, 'run_id')}.json"

    def _canonical_gate2_preflight_bundle_path(
        self, preflight_bundle: Path, expected_run_id: str,
    ) -> Path:
        """Reject caller-selected bundle locations before reading their bytes."""
        try:
            if preflight_bundle.is_symlink():
                raise ReviewQueueError("preflight_bundle_path_invalid")
            resolved = preflight_bundle.resolve(strict=False)
            canonical_directory = self.root / "preflight_bundles"
            if (
                canonical_directory.is_symlink()
                or (canonical_directory.exists() and not canonical_directory.is_dir())
            ):
                raise ReviewQueueError("preflight_bundle_path_invalid")
            canonical_directory = canonical_directory.resolve(strict=False)
            if (
                resolved.parent != canonical_directory
                or resolved.suffix != ".json"
            ):
                raise ReviewQueueError("preflight_bundle_path_invalid")
            bundle_run_id = _validate_id(resolved.stem, "run_id")
            canonical = self.gate2_preflight_bundle_path(bundle_run_id).resolve()
            if resolved != canonical or bundle_run_id != expected_run_id:
                raise ReviewQueueError("preflight_bundle_path_invalid")
        except (OSError, ReviewQueueError) as exc:
            raise ReviewQueueError("preflight_bundle_path_invalid") from exc
        return preflight_bundle

    def resolve_completed_gate2(
        self, *, execution_id: str, review_attempt_id: str, run_id: str,
    ) -> dict[str, Any]:
        """Resolve one completed advisory review and its owner-created Gate2 index.

        ``status()`` is the authoritative queue validator for request/claim/
        terminal/result provenance.  The review verdict remains advisory; the
        exclusive Gate2 index is the local owner transition that adds the
        runtime-specific image, artifact, bundle and launch contract.  Every
        review identity and source hash must still exactly match the completed
        queue records.  This keeps arbitrary bytes and paths out of the runtime
        caller's input surface without turning reviewer severity into authority.
        """
        execution_id = _validate_id(execution_id, "execution_id")
        review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
        run_id = _validate_id(run_id, "run_id")
        status = self.status(execution_id, review_attempt_id)
        if (
            status.get("state") != "COMPLETED"
            or status.get("integrity_valid") is not True
            or not isinstance(status.get("request"), dict)
            or not isinstance(status.get("result"), dict)
        ):
            raise ReviewQueueError("gate2_review_not_completed")
        request = status["request"]
        result = status["result"]
        request_path = self._record("requests", review_attempt_id)
        result_path = self._record("terminals", review_attempt_id)
        try:
            request_sha256 = _sha256(request_path)
            result_sha256 = _sha256(result_path)
        except OSError as exc:
            raise ReviewQueueError("gate2_review_record_unreadable") from exc
        if (
            result.get("terminal_kind") != "COMPLETED"
            or result.get("execution_id") != execution_id
            or result.get("review_attempt_id") != review_attempt_id
            or result.get("request_sha256") != request_sha256
            or result.get("advisory_only") is not True
            or result.get("workflow_advance_allowed") is not False
        ):
            raise ReviewQueueError("gate2_review_result_contract_invalid")
        index_path = self.gate2_authorization_path(review_attempt_id)
        index = _read_json(index_path, "gate2_authorization_index")
        required = {
            "schema_version", "run_id", "execution_id", "review_attempt_id",
            "review_result_path", "review_result_sha256", "reviewed_handoff_path",
            "reviewed_handoff_sha256", "review_packet_path", "review_packet_sha256",
            "review_bundle_path", "review_bundle_sha256", "image_id", "artifact_sha256",
            "authority_profile", "runtime_budget", "transition_permission", "launch_spec",
        }
        if set(index) != required or index.get("schema_version") != 1:
            raise ReviewQueueError("gate2_authorization_index_schema_invalid")
        if (
            index.get("run_id") != run_id
            or index.get("execution_id") != execution_id
            or index.get("review_attempt_id") != review_attempt_id
            or index.get("review_result_path") != str(result_path.resolve())
            or index.get("review_result_sha256") != result_sha256
            or index.get("reviewed_handoff_path") != request.get("handoff_path")
            or index.get("reviewed_handoff_sha256") != request.get("handoff_sha256")
            or index.get("review_packet_path") != request.get("packet_path")
            or index.get("review_packet_sha256") != request.get("packet_sha256")
            or index.get("transition_permission") != "GO_RUNTIME_ONCE"
        ):
            raise ReviewQueueError("gate2_authorization_index_binding_invalid")
        indexed_bundle_path = index.get("review_bundle_path")
        canonical_bundle_path = self.gate2_preflight_bundle_path(run_id).resolve(
            strict=False
        )
        if (
            not isinstance(indexed_bundle_path, str)
            or not Path(indexed_bundle_path).is_absolute()
            or indexed_bundle_path != str(canonical_bundle_path)
        ):
            raise ReviewQueueError("gate2_authorization_index_path_invalid")
        try:
            canonical_indexed_bundle_path = (
                self._canonical_gate2_preflight_bundle_path(
                    Path(indexed_bundle_path), run_id
                ).resolve(strict=False)
            )
        except (OSError, ReviewQueueError) as exc:
            raise ReviewQueueError(
                "gate2_authorization_index_path_invalid"
            ) from exc
        if canonical_indexed_bundle_path != canonical_bundle_path:
            raise ReviewQueueError("gate2_authorization_index_path_invalid")
        try:
            index_stat = index_path.lstat()
            if stat.S_IMODE(index_stat.st_mode) != 0o644 or index_stat.st_uid != os.getuid():
                raise ReviewQueueError("gate2_authorization_index_unsafe")
        except OSError as exc:
            raise ReviewQueueError("gate2_authorization_index_unsafe") from exc
        for path_key, sha_key in (
            ("review_result_path", "review_result_sha256"),
            ("reviewed_handoff_path", "reviewed_handoff_sha256"),
            ("review_packet_path", "review_packet_sha256"),
            ("review_bundle_path", "review_bundle_sha256"),
        ):
            path_raw = index.get(path_key)
            sha = index.get(sha_key)
            if (
                not isinstance(path_raw, str) or not Path(path_raw).is_absolute()
                or not isinstance(sha, str) or not re.fullmatch(r"[0-9a-f]{64}", sha)
            ):
                raise ReviewQueueError("gate2_authorization_index_path_invalid")
            try:
                path = Path(path_raw)
                if _sha256(_regular_file(path, path_key)) != sha:
                    raise ReviewQueueError("gate2_authorization_index_hash_mismatch")
            except (OSError, ReviewQueueError) as exc:
                if isinstance(exc, ReviewQueueError):
                    raise
                raise ReviewQueueError("gate2_authorization_index_path_invalid") from exc
        image_id = index.get("image_id")
        artifact_sha256 = index.get("artifact_sha256")
        if (
            not isinstance(image_id, str)
            or not re.fullmatch(r"sha256:[0-9a-f]{64}", image_id)
            or not isinstance(artifact_sha256, dict)
            or set(artifact_sha256) != {"mux_node", "mux_config", "planner_node"}
            or any(not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value)
                   for value in artifact_sha256.values())
            or not isinstance(index.get("authority_profile"), dict)
            or not isinstance(index.get("runtime_budget"), dict)
            or not isinstance(index.get("launch_spec"), dict)
        ):
            raise ReviewQueueError("gate2_authorization_index_contract_invalid")
        return {
            **index,
            "state": "COMPLETED",
            "integrity_valid": True,
            "authorization_index_path": str(index_path.resolve()),
            "authorization_index_sha256": _sha256(index_path),
            "queue_root": str(self.root),
            "request_path": str(request_path.resolve()),
            "request_sha256": request_sha256,
            "review_result_path": str(result_path.resolve()),
            "review_result_sha256": result_sha256,
            "review_response_path": result.get("response_path"),
            "review_response_sha256": result.get("response_sha256"),
            "review_model": result.get("model"),
            "review_severity": result.get("severity"),
            "review_disposition": result.get("disposition"),
            "review_transition_permission": result.get("transition_permission"),
        }

    def authorize_gate2(
        self, *, execution_id: str, review_attempt_id: str,
        preflight_bundle: Path, expected_run_id: str,
    ) -> dict[str, Any]:
        """Create one owner-admitted Gate2 index from a reviewed machine bundle.

        The bundle is an immutable, exact-schema JSON artifact produced by the
        rebuild/preflight workflow.  No runtime identity, hash, or launch field
        is accepted as a CLI value; all of them are recovered from that bundle
        and cross-bound to the already completed queue request/terminal.  The
        external verdict is retained as advisory provenance; this explicit,
        exclusive method call is the local owner transition to one runtime.
        """
        execution_id = _validate_id(execution_id, "execution_id")
        review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
        if not isinstance(expected_run_id, str):
            raise ReviewQueueError("preflight_bundle_path_invalid")
        expected_run_id = _validate_id(expected_run_id, "run_id")
        preflight_bundle = self._canonical_gate2_preflight_bundle_path(
            preflight_bundle, expected_run_id
        )
        status = self.status(execution_id, review_attempt_id)
        if (
            status.get("state") != "COMPLETED"
            or status.get("integrity_valid") is not True
            or not isinstance(status.get("request"), dict)
            or not isinstance(status.get("result"), dict)
        ):
            raise ReviewQueueError("gate2_review_not_completed")
        request = status["request"]
        result = status["result"]
        request_path = self._record("requests", review_attempt_id)
        result_path = self._record("terminals", review_attempt_id)
        request_sha256 = _sha256(request_path)
        result_sha256 = _sha256(result_path)
        if (
            result.get("terminal_kind") != "COMPLETED"
            or result.get("execution_id") != execution_id
            or result.get("review_attempt_id") != review_attempt_id
            or result.get("request_sha256") != request_sha256
            or result.get("advisory_only") is not True
            or result.get("workflow_advance_allowed") is not False
        ):
            raise ReviewQueueError("gate2_review_result_contract_invalid")

        bundle = _regular_file(preflight_bundle, "preflight_bundle")
        try:
            bundle_stat = bundle.lstat()
            if stat.S_IMODE(bundle_stat.st_mode) != 0o644 or bundle_stat.st_uid != os.getuid():
                raise ReviewQueueError("preflight_bundle_unsafe")
            raw = bundle.read_bytes()
            payload = json.loads(raw.decode("utf-8"), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
            if isinstance(exc, ReviewQueueError):
                raise
            raise ReviewQueueError("preflight_bundle_invalid_json") from exc
        if not isinstance(payload, dict):
            raise ReviewQueueError("preflight_bundle_schema_invalid")
        required = {
            "schema_version", "run_id", "execution_id", "review_attempt_id",
            "request_sha256", "reviewed_handoff_path", "reviewed_handoff_sha256",
            "review_packet_path", "review_packet_sha256", "image_id", "artifact_sha256",
            "authority_profile", "runtime_budget", "launch_spec",
        }
        if set(payload) != required or payload.get("schema_version") != 1:
            raise ReviewQueueError("preflight_bundle_schema_invalid")
        if not isinstance(payload.get("run_id"), str):
            raise ReviewQueueError("preflight_bundle_run_id_invalid")
        run_id = _validate_id(payload["run_id"], "run_id")
        if run_id != expected_run_id:
            raise ReviewQueueError("preflight_bundle_run_id_mismatch")
        if (
            payload.get("execution_id") != execution_id
            or payload.get("review_attempt_id") != review_attempt_id
            or payload.get("request_sha256") != request_sha256
            or payload.get("reviewed_handoff_path") != request.get("handoff_path")
            or payload.get("reviewed_handoff_sha256") != request.get("handoff_sha256")
            or payload.get("review_packet_path") != request.get("packet_path")
            or payload.get("review_packet_sha256") != request.get("packet_sha256")
        ):
            raise ReviewQueueError("preflight_bundle_request_binding_invalid")
        for path_key, sha_key in (
            ("reviewed_handoff_path", "reviewed_handoff_sha256"),
            ("review_packet_path", "review_packet_sha256"),
        ):
            path_raw, expected_sha = payload.get(path_key), payload.get(sha_key)
            if (
                not isinstance(path_raw, str) or not Path(path_raw).is_absolute()
                or not isinstance(expected_sha, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_sha)
            ):
                raise ReviewQueueError("preflight_bundle_source_invalid")
            if _sha256(_regular_file(Path(path_raw), path_key)) != expected_sha:
                raise ReviewQueueError("preflight_bundle_source_hash_mismatch")
        image_id = payload.get("image_id")
        artifacts = payload.get("artifact_sha256")
        if (
            not isinstance(image_id, str) or not re.fullmatch(r"sha256:[0-9a-f]{64}", image_id)
            or not isinstance(artifacts, dict) or set(artifacts) != {"mux_node", "mux_config", "planner_node"}
            or any(not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value) for value in artifacts.values())
            or payload.get("runtime_budget") != GATE2_RUNTIME_BUDGET
            or not isinstance(payload.get("authority_profile"), dict)
            or not payload["authority_profile"]
            or not isinstance(payload.get("launch_spec"), dict)
        ):
            raise ReviewQueueError("preflight_bundle_contract_invalid")
        launch_spec = payload["launch_spec"]
        if set(launch_spec) != {"service_set", "services"}:
            raise ReviewQueueError("preflight_bundle_launch_spec_invalid")
        services = launch_spec.get("services")
        if (
            launch_spec.get("service_set") != ["autoware-eval-command", "autoware-eval-runtime"]
            or not isinstance(services, dict)
            or set(services) != {"autoware-eval-command", "autoware-eval-runtime"}
            or any(not isinstance(service, dict) or set(service) != GATE2_LAUNCH_SERVICE_KEYS for service in services.values())
        ):
            raise ReviewQueueError("preflight_bundle_launch_spec_invalid")
        index = {
            "schema_version": 1,
            "run_id": run_id,
            "execution_id": execution_id,
            "review_attempt_id": review_attempt_id,
            "review_result_path": str(result_path.resolve()),
            "review_result_sha256": result_sha256,
            "reviewed_handoff_path": request["handoff_path"],
            "reviewed_handoff_sha256": request["handoff_sha256"],
            "review_packet_path": request["packet_path"],
            "review_packet_sha256": request["packet_sha256"],
            "review_bundle_path": str(bundle.resolve()),
            "review_bundle_sha256": _sha256(bundle),
            "image_id": image_id,
            "artifact_sha256": artifacts,
            "authority_profile": payload["authority_profile"],
            "runtime_budget": payload["runtime_budget"],
            "transition_permission": "GO_RUNTIME_ONCE",
            "launch_spec": launch_spec,
        }
        _write_exclusive(self.gate2_authorization_path(review_attempt_id), index)
        return self.resolve_completed_gate2(
            execution_id=execution_id, review_attempt_id=review_attempt_id, run_id=run_id,
        )

    def enqueue(self, *, execution_id: str, review_attempt_id: str, handoff: Path, packet: Path,
                objective: str, requested_by: str, retry_of: str | None = None) -> dict[str, Any]:
        with self._queue_lock():
            execution_id = _validate_id(execution_id, "execution_id")
            review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
            requested_by = _validate_id(requested_by, "requested_by")
            if not objective.strip() or len(objective) > 4096:
                raise ReviewQueueError("invalid_objective")
            handoff = _regular_file(handoff, "handoff")
            packet = _regular_file(packet, "packet")
            handoff_sha256, packet_sha256 = _sha256(handoff), _sha256(packet)
            retry_terminal: dict[str, Any] | None = None
            if retry_of is not None:
                retry_of = _validate_id(retry_of, "retry_of")
                retry_terminal = self._validate_retry(
                    retry_of, execution_id, handoff_sha256, packet_sha256
                )
            payload = {
                "schema_version": SCHEMA_VERSION, "execution_id": execution_id,
                "review_attempt_id": review_attempt_id, "retry_of": retry_of,
                "status": "REVIEW_PENDING", "requested_by": requested_by, "objective": objective,
                "handoff_path": str(handoff), "handoff_sha256": handoff_sha256,
                "packet_path": str(packet), "packet_sha256": packet_sha256,
                "approved_models": list(ALLOWED_MODELS), "approved_project_url": APPROVED_PROJECT_URL,
                "transport": "Playwright MCP only", "created_at": _now_utc(),
            }
            if (retry_terminal is not None
                    and retry_terminal.get("terminal_kind") == "RETRYABLE_UI_TERMINAL_FAILURE"
                    and retry_terminal.get("terminal_error_kind") == "RATE_LIMIT"):
                failed_at = _parse_utc(retry_terminal.get("failed_at"))
                if failed_at is None:
                    raise ReviewQueueError("retry_prior_failure_time_invalid")
                payload["not_before"] = (
                    failed_at + RATE_LIMIT_RETRY_DELAY
                ).isoformat().replace("+00:00", "Z")
            _write_exclusive(self._record("requests", review_attempt_id), payload)
        return self.status(execution_id, review_attempt_id)

    def retire_pending(
        self, *, execution_id: str, review_attempt_id: str, retired_by: str,
        expected_request_sha256: str, reason: str,
    ) -> dict[str, Any]:
        """Retire one unclaimed retry whose immutable lineage is invalid."""
        execution_id = _validate_id(execution_id, "execution_id")
        review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
        retired_by = _validate_id(retired_by, "retired_by")
        if not re.fullmatch(r"[0-9a-f]{64}", expected_request_sha256):
            raise ReviewQueueError("retire_pending_request_sha256_invalid")
        if not isinstance(reason, str) or not reason.strip() or len(reason) > 4096:
            raise ReviewQueueError("retire_pending_reason_invalid")
        with self._queue_lock():
            request_path = self._record("requests", review_attempt_id)
            request = _read_json(request_path, "request")
            if (self._request_schema_errors(request, execution_id, review_attempt_id)
                    or self._source_integrity_errors(request)):
                raise ReviewQueueError("request_source_integrity_invalid")
            if _sha256(request_path) != expected_request_sha256:
                raise ReviewQueueError("retire_pending_request_sha256_mismatch")
            retry_of = request.get("retry_of")
            if (not isinstance(retry_of, str) or not _ID_RE.fullmatch(retry_of)
                    or self._retry_lineage_valid(request)):
                raise ReviewQueueError("retire_pending_not_invalid_retry")
            retirement_path = self._retirement_path(review_attempt_id)
            if retirement_path.exists():
                raise ReviewQueueError("review_already_retired")
            activity_paths = (
                self._record("claims", review_attempt_id),
                self._record("results", review_attempt_id),
                self._record("deferrals", review_attempt_id),
                self._record("terminals", review_attempt_id),
                self._record("submissions", review_attempt_id),
                self._submission_intent_path(review_attempt_id),
                self._singleflight_lease_path(review_attempt_id),
            )
            if any(path.exists() for path in activity_paths):
                raise ReviewQueueError("retire_pending_not_unclaimed")
            payload = {
                "schema_version": 1,
                "execution_id": execution_id,
                "review_attempt_id": review_attempt_id,
                "status": "RETIRED_PENDING",
                "request_sha256": expected_request_sha256,
                "handoff_sha256": request["handoff_sha256"],
                "packet_sha256": request["packet_sha256"],
                "reason_code": RETIREMENT_REASON_CODE,
                "reason": reason,
                "retired_by": retired_by,
                "retired_at": _now_utc(),
                "workflow_advance_allowed": False,
                "retry_eligible": False,
            }
            _write_exclusive(retirement_path, payload)
        return self.status(execution_id, review_attempt_id)

    def _validate_retry(self, retry_of: str, execution_id: str, handoff_sha256: str,
                        packet_sha256: str) -> dict[str, Any]:
        if self._retirement_path(retry_of).exists():
            raise ReviewQueueError("retry_prior_retired")
        try:
            request = _read_json(self._record("requests", retry_of), "prior_request")
            terminal_path = self._record("terminals", retry_of)
            terminal_path_or_deferral = terminal_path if terminal_path.exists() else self._record("deferrals", retry_of)
            terminal = _read_json(terminal_path_or_deferral, "prior_terminal")
        except (OSError, ReviewQueueError) as exc:
            raise ReviewQueueError("retry_prior_deferral_invalid") from exc
        if self._record("results", retry_of).exists():
            raise ReviewQueueError("retry_prior_terminal_conflict")
        terminal_kind = terminal.get("terminal_kind", "DEFERRED_CONNECTION_UNAVAILABLE")
        if terminal_kind == "COMPLETED":
            raise ReviewQueueError("retry_prior_terminal_conflict")
        is_legacy = request.get("schema_version") == LEGACY_SCHEMA_VERSION
        valid = (not self._legacy_request_errors(request, execution_id, retry_of)
                 and self._legacy_deferral_valid(terminal, request, terminal_path_or_deferral, retry_of)) if is_legacy else (
                     not self._request_schema_errors(request, execution_id, retry_of)
                     and (self._deferral_valid(terminal, request, terminal_path_or_deferral)
                          or self._retryable_terminal_failure_valid(terminal, request)))
        if not valid:
            raise ReviewQueueError("retry_prior_deferral_invalid")
        if request["handoff_sha256"] != handoff_sha256 or request["packet_sha256"] != packet_sha256:
            raise ReviewQueueError("retry_source_hash_mismatch")
        return terminal

    @staticmethod
    def _terminal_timestamp(marker: dict[str, Any]) -> datetime | None:
        for field in ("completed_at", "deferred_at", "failed_at"):
            timestamp = _parse_utc(marker.get(field))
            if timestamp is not None:
                return timestamp
        return None

    def _queue_cooldown_until_locked(self) -> datetime | None:
        """Return the queue-wide cooldown deadline from the newest terminal.

        Terminal records are immutable and are intentionally retained.  The
        newest timestamp is authoritative; malformed timestamps fail closed so
        an unsafe parallel submission cannot be hidden by a bad record.
        """
        terminals = self.root / "terminals"
        latest: datetime | None = None
        if not terminals.exists():
            return None
        for path in sorted(terminals.glob("*.json")):
            marker = _read_json(path, "terminal_marker")
            if marker.get("schema_version") != SCHEMA_VERSION:
                continue
            # A connection-unavailable deferral did not submit anything to
            # the external service, so it does not consume the global lane.
            if marker.get("terminal_kind") == "DEFERRED_CONNECTION_UNAVAILABLE":
                continue
            timestamp = self._terminal_timestamp(marker)
            if timestamp is None:
                raise ReviewQueueError("terminal_cooldown_timestamp_invalid")
            if latest is None or timestamp > latest:
                latest = timestamp
        if latest is None:
            return None
        return latest + QUEUE_TERMINAL_COOLDOWN

    def _singleflight_lease_valid(self, lease: dict[str, Any], request: dict[str, Any],
                                  request_path: Path, review_attempt_id: str) -> bool:
        required = {
            "schema_version", "lease_version", "execution_id", "review_attempt_id",
            "request_sha256", "handoff_sha256", "packet_sha256", "claim_sha256",
            "worker_id", "status", "claimed_at",
        }
        lease_path = self._singleflight_lease_path(review_attempt_id)
        claim_path = self._record("claims", review_attempt_id)
        try:
            claim = _read_json(claim_path, "claim")
            claimed_at = _parse_utc(lease.get("claimed_at"))
        except (OSError, ReviewQueueError):
            return False
        return (
            set(lease) == required
            and lease.get("schema_version") == SCHEMA_VERSION
            and lease.get("lease_version") == 1
            and lease.get("execution_id") == request.get("execution_id")
            and lease.get("review_attempt_id") == review_attempt_id
            and lease.get("request_sha256") == _sha256(request_path)
            and lease.get("handoff_sha256") == request.get("handoff_sha256")
            and lease.get("packet_sha256") == request.get("packet_sha256")
            and lease.get("claim_sha256") == _sha256(claim_path)
            and lease.get("worker_id") == claim.get("worker_id")
            and lease.get("status") == "CLAIMED"
            and isinstance(lease.get("worker_id"), str)
            and bool(_ID_RE.fullmatch(lease["worker_id"]))
            and claimed_at is not None
            and lease_path.is_file()
        )

    def _active_singleflight_lease_locked(self) -> tuple[str, dict[str, Any]] | None:
        """Find the one active lease created by the new arbitration path.

        ``claims/`` deliberately is not consulted here.  It contains records
        from older queue versions, including historical claim-without-terminal
        records that remain monitoring-only and must not deadlock new work.
        """
        collection = self.root / SINGLEFLIGHT_LEASE_COLLECTION
        if not collection.exists():
            return None
        active: list[tuple[str, dict[str, Any]]] = []
        for lease_path in sorted(collection.glob("*.json")):
            attempt = _validate_id(lease_path.stem, "review_attempt_id")
            lease = _read_json(lease_path, "singleflight_lease")
            request_path = self._record("requests", attempt)
            request = _read_json(request_path, "request")
            if not self._singleflight_lease_valid(lease, request, request_path, attempt):
                raise ReviewQueueError("singleflight_lease_invalid")
            terminal_path = self._record("terminals", attempt)
            if terminal_path.exists():
                marker = _read_json(terminal_path, "terminal_marker")
                request_path = self._record("requests", attempt)
                claim_path = self._record("claims", attempt)
                try:
                    terminal_request = _read_json(request_path, "request")
                    terminal_claim = _read_json(claim_path, "claim")
                    kind = marker.get("terminal_kind")
                    marker_bound = self._terminal_marker_valid(
                        marker, terminal_request, request_path, terminal_claim,
                        claim_path, terminal_request.get("execution_id", ""), attempt, kind,
                    )
                    content_bound = (
                        kind == "COMPLETED" and self._result_valid(
                            marker, terminal_request, request_path, terminal_claim,
                            terminal_request.get("execution_id", ""), attempt,
                        )
                    ) or (
                        kind == "DEFERRED_CONNECTION_UNAVAILABLE" and self._deferral_valid(
                            marker, terminal_request, terminal_path,
                        )
                    ) or (
                        kind == "RETRYABLE_UI_TERMINAL_FAILURE" and self._retryable_terminal_failure_valid(
                            marker, terminal_request,
                        )
                    )
                except (KeyError, OSError, ReviewQueueError):
                    marker_bound = content_bound = False
                if not marker_bound or not content_bound:
                    raise ReviewQueueError("singleflight_lease_terminal_invalid")
                continue
            active.append((attempt, lease))
        # Intents/submission markers are also new-system records.  An orphan
        # marker (for example, a crash before the second atomic write) is
        # ambiguous and therefore blocks the lane rather than allowing a
        # newer request to send in parallel.
        intents = self.root / SUBMISSION_INTENT_COLLECTION
        if intents.exists():
            for intent_path in sorted(intents.glob("*.json")):
                attempt = _validate_id(intent_path.stem, "review_attempt_id")
                lease_path = self._singleflight_lease_path(attempt)
                if not lease_path.exists():
                    raise ReviewQueueError("singleflight_lease_missing_for_submission")
                if not self._record("claims", attempt).exists():
                    raise ReviewQueueError("singleflight_claim_missing_for_submission")
        submissions = self.root / "submissions"
        if submissions.exists():
            for submission_path in sorted(submissions.glob("*.json")):
                attempt = _validate_id(submission_path.stem, "review_attempt_id")
                if not self._submission_intent_path(attempt).exists():
                    raise ReviewQueueError("submission_intent_missing_for_submission")
                if not self._singleflight_lease_path(attempt).exists():
                    raise ReviewQueueError("singleflight_lease_missing_for_submission")
        if len(active) > 1:
            raise ReviewQueueError("singleflight_multiple_active_leases")
        return active[0] if active else None

    def _eligible_pending_locked(self, execution_id: str | None = None) -> list[tuple[datetime, str, dict[str, Any], Path]]:
        requests = self.root / "requests"
        if not requests.exists():
            return []
        now = datetime.now(timezone.utc)
        records: list[tuple[datetime, str, dict[str, Any], Path]] = []
        for request_path in sorted(requests.glob("*.json")):
            attempt = _validate_id(request_path.stem, "review_attempt_id")
            try:
                request = _read_json(request_path, "request")
            except ReviewQueueError as exc:
                raise ReviewQueueError(f"pending_request_invalid:{attempt}") from exc
            # Schema-v1 records are grandfathered monitoring-only and never
            # enter the new FIFO arbitration lane.
            if request.get("schema_version") != SCHEMA_VERSION:
                continue
            if execution_id is not None and request.get("execution_id") != execution_id:
                continue
            created_at = _parse_utc(request.get("created_at"))
            if created_at is None:
                raise ReviewQueueError(f"pending_request_invalid:{attempt}")
            records.append((created_at, attempt, request, request_path))
        records.sort(key=lambda item: (item[0], item[1]))
        candidates: list[tuple[datetime, str, dict[str, Any], Path]] = []
        for created_at, attempt, request, request_path in records:
            artifacts = tuple(
                self._record(collection, attempt).exists()
                for collection in ("claims", "results", "deferrals", "terminals", "submissions", SUBMISSION_INTENT_COLLECTION)
            )
            retirement_path = self._retirement_path(attempt)
            if retirement_path.exists():
                try:
                    retirement = _read_json(retirement_path, "retirement")
                    retirement_valid = self._retirement_valid(
                        retirement, request, request_path, attempt,
                    )
                except ReviewQueueError:
                    retirement_valid = False
                if (self._request_schema_errors(
                        request, request.get("execution_id", ""), attempt)
                        or self._source_integrity_errors(request)
                        or not retirement_valid
                        or any(artifacts)
                        or self._singleflight_lease_path(attempt).exists()):
                    raise ReviewQueueError(f"pending_retirement_invalid:{attempt}")
                continue
            if request.get("status") != "REVIEW_PENDING":
                if not any(artifacts):
                    raise ReviewQueueError(f"pending_request_invalid:{attempt}")
                continue
            # A malformed or tampered unclaimed pending request is a queue
            # barrier.  Never skip it and submit a newer packet out of order.
            if (self._request_schema_errors(request, request.get("execution_id", ""), attempt)
                    or not self._retry_lineage_valid(request)
                    or self._source_integrity_errors(request)):
                if not any(artifacts):
                    raise ReviewQueueError(f"pending_request_invalid:{attempt}")
                continue
            if any(self._record(collection, attempt).exists() for collection in (
                    "claims", "results", "deferrals", "terminals", "submissions", SUBMISSION_INTENT_COLLECTION)):
                continue
            not_before = request.get("not_before")
            if not_before is not None:
                parsed_not_before = _parse_utc(not_before)
                if parsed_not_before is None:
                    raise ReviewQueueError(f"pending_request_invalid:{attempt}")
                if now < parsed_not_before:
                    continue
            candidates.append((created_at, attempt, request, request_path))
        return candidates

    def _claim_locked(self, *, execution_id: str, review_attempt_id: str,
                      worker_id: str) -> None:
        """Claim one already-selected request while holding ``_queue_lock``."""
        cooldown_until = self._queue_cooldown_until_locked()
        now = datetime.now(timezone.utc)
        if cooldown_until is not None and now < cooldown_until:
            raise ReviewQueueError(
                f"review_queue_cooldown_until:{cooldown_until.isoformat().replace('+00:00', 'Z')}"
            )
        request_path = self._record("requests", review_attempt_id)
        request = _read_json(request_path, "request")
        retirement_path = self._retirement_path(review_attempt_id)
        if retirement_path.exists():
            retirement = _read_json(retirement_path, "retirement")
            conflicting_paths = (
                self._record("claims", review_attempt_id),
                self._record("results", review_attempt_id),
                self._record("deferrals", review_attempt_id),
                self._record("terminals", review_attempt_id),
                self._record("submissions", review_attempt_id),
                self._submission_intent_path(review_attempt_id),
                self._singleflight_lease_path(review_attempt_id),
            )
            if (self._request_schema_errors(request, execution_id, review_attempt_id)
                    or self._source_integrity_errors(request)
                    or not self._retirement_valid(
                        retirement, request, request_path, review_attempt_id)
                    or any(path.exists() for path in conflicting_paths)):
                raise ReviewQueueError("retirement_schema_or_binding_invalid")
            raise ReviewQueueError("review_retired")
        if (self._request_schema_errors(request, execution_id, review_attempt_id)
                or not self._retry_lineage_valid(request) or self._source_integrity_errors(request)):
            raise ReviewQueueError("request_source_integrity_invalid")
        not_before = request.get("not_before")
        if not_before is not None:
            parsed_not_before = _parse_utc(not_before)
            if parsed_not_before is None:
                raise ReviewQueueError("request_not_before_invalid")
            if now < parsed_not_before:
                raise ReviewQueueError(f"review_not_ready_until:{not_before}")
        if (self._record("results", review_attempt_id).exists()
                or self._record("deferrals", review_attempt_id).exists()
                or self._record("terminals", review_attempt_id).exists()):
            raise ReviewQueueError("review_already_terminal")
        if self._record("submissions", review_attempt_id).exists():
            raise ReviewQueueError("review_already_submitted")
        if self._submission_intent_path(review_attempt_id).exists():
            raise ReviewQueueError("review_already_submitting")
        claim_path = self._record("claims", review_attempt_id)
        if claim_path.exists():
            raise ReviewQueueError("review_already_claimed")
        active = self._active_singleflight_lease_locked()
        if active is not None and active[0] != review_attempt_id:
            raise ReviewQueueError(f"review_singleflight_busy:{active[0]}")
        candidates = self._eligible_pending_locked()
        if not candidates or candidates[0][1] != review_attempt_id:
            oldest = candidates[0][1] if candidates else "none"
            raise ReviewQueueError(f"review_not_oldest_pending:{oldest}")
        claim_payload = {
            "schema_version": SCHEMA_VERSION, "execution_id": execution_id,
            "review_attempt_id": review_attempt_id, "request_sha256": _sha256(request_path),
            "handoff_sha256": request["handoff_sha256"], "worker_id": worker_id,
            "status": "CLAIMED", "claimed_at": _now_utc(),
        }
        # Write the lease first.  Its claim hash is the hash of the exact
        # immutable claim bytes we are about to write.  If the process exits
        # between these writes, arbitration sees an incomplete lease and
        # fails closed instead of allowing another submission.
        lease_payload = {
            "schema_version": SCHEMA_VERSION, "lease_version": 1,
            "execution_id": execution_id, "review_attempt_id": review_attempt_id,
            "request_sha256": _sha256(request_path),
            "handoff_sha256": request["handoff_sha256"], "packet_sha256": request["packet_sha256"],
            "claim_sha256": hashlib.sha256(_json_bytes(claim_payload)).hexdigest(),
            "worker_id": worker_id,
            "status": "CLAIMED", "claimed_at": claim_payload["claimed_at"],
        }
        _write_exclusive(self._singleflight_lease_path(review_attempt_id), lease_payload)
        _write_exclusive(claim_path, claim_payload)

    def claim(self, *, execution_id: str, review_attempt_id: str, worker_id: str) -> dict[str, Any]:
        execution_id = _validate_id(execution_id, "execution_id")
        review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
        worker_id = _validate_id(worker_id, "worker_id")
        with self._queue_lock():
            self._claim_locked(execution_id=execution_id, review_attempt_id=review_attempt_id,
                               worker_id=worker_id)
        return self.status(execution_id, review_attempt_id)

    def claim_next(self, *, worker_id: str, execution_id: str | None = None) -> dict[str, Any] | None:
        """Atomically claim the oldest eligible pending request, if any."""
        worker_id = _validate_id(worker_id, "worker_id")
        if execution_id is not None:
            execution_id = _validate_id(execution_id, "execution_id")
        with self._queue_lock():
            cooldown_until = self._queue_cooldown_until_locked()
            now = datetime.now(timezone.utc)
            if cooldown_until is not None and now < cooldown_until:
                raise ReviewQueueError(
                    f"review_queue_cooldown_until:{cooldown_until.isoformat().replace('+00:00', 'Z')}"
                )
            active = self._active_singleflight_lease_locked()
            if active is not None:
                raise ReviewQueueError(f"review_singleflight_busy:{active[0]}")
            candidates = self._eligible_pending_locked(execution_id)
            if not candidates:
                return None
            _, attempt, request, _ = candidates[0]
            self._claim_locked(execution_id=request["execution_id"], review_attempt_id=attempt,
                               worker_id=worker_id)
            claimed_execution = request["execution_id"]
        return self.status(claimed_execution, attempt)

    def _submission_intent_valid(self, intent: dict[str, Any], request: dict[str, Any],
                                 request_path: Path, claim: dict[str, Any],
                                 review_attempt_id: str) -> bool:
        required = {
            "schema_version", "execution_id", "review_attempt_id", "request_sha256",
            "handoff_sha256", "packet_sha256", "claim_sha256", "lease_sha256",
            "worker_id", "model", "model_fallback_reason", "project_url",
            "submission_id", "status", "started_at",
        }
        lease_path = self._singleflight_lease_path(review_attempt_id)
        started_at = _parse_utc(intent.get("started_at"))
        expected_submission_id = intent.get("submission_id")
        try:
            lease_sha = _sha256(_regular_file(lease_path, "singleflight_lease"))
        except (OSError, ReviewQueueError):
            return False
        return (
            set(intent) == required
            and intent.get("schema_version") == SCHEMA_VERSION
            and intent.get("execution_id") == request.get("execution_id")
            and intent.get("review_attempt_id") == review_attempt_id
            and intent.get("request_sha256") == _sha256(request_path)
            and intent.get("handoff_sha256") == request.get("handoff_sha256")
            and intent.get("packet_sha256") == request.get("packet_sha256")
            and intent.get("claim_sha256") == _sha256(self._record("claims", review_attempt_id))
            and intent.get("lease_sha256") == lease_sha
            and intent.get("worker_id") == claim.get("worker_id")
            and intent.get("model") in ALLOWED_MODELS
            and intent.get("project_url") == APPROVED_PROJECT_URL
            and (expected_submission_id is None or (
                isinstance(expected_submission_id, str)
                and bool(_ID_RE.fullmatch(expected_submission_id))
            ))
            and intent.get("status") == "SUBMITTING"
            and started_at is not None
            and ((intent.get("model") == "GPT-5.6 Pro") == bool(
                intent.get("model_fallback_reason")
            ))
        )

    def _submission_valid(self, submission: dict[str, Any], request: dict[str, Any],
                          request_path: Path, claim: dict[str, Any],
                          review_attempt_id: str) -> bool:
        required = {
            "schema_version", "execution_id", "review_attempt_id", "request_sha256",
            "handoff_sha256", "packet_sha256", "claim_sha256", "lease_sha256",
            "worker_id", "model", "model_fallback_reason", "project_url",
            "submission_id", "submission_verified", "submitted_at",
        }
        lease_path = self._singleflight_lease_path(review_attempt_id)
        try:
            expected_lease_sha = _sha256(_regular_file(lease_path, "singleflight_lease"))
        except (OSError, ReviewQueueError):
            return False
        submitted_at = _parse_utc(submission.get("submitted_at"))
        return (
            set(submission) == required
            and submission.get("schema_version") == SCHEMA_VERSION
            and submission.get("execution_id") == request.get("execution_id")
            and submission.get("review_attempt_id") == review_attempt_id
            and submission.get("request_sha256") == _sha256(request_path)
            and submission.get("handoff_sha256") == request.get("handoff_sha256")
            and submission.get("packet_sha256") == request.get("packet_sha256")
            and submission.get("claim_sha256") == _sha256(self._record("claims", review_attempt_id))
            and submission.get("lease_sha256") == expected_lease_sha
            and submission.get("worker_id") == claim.get("worker_id")
            and submission.get("model") in ALLOWED_MODELS
            and submission.get("project_url") == APPROVED_PROJECT_URL
            and isinstance(submission.get("submission_id"), str)
            and bool(_ID_RE.fullmatch(submission.get("submission_id", "")))
            and submission.get("submission_verified") is True
            and ((submission.get("model") == "GPT-5.6 Pro") == bool(
                submission.get("model_fallback_reason")
            ))
            and submitted_at is not None
        )

    def begin_submission(self, *, execution_id: str, review_attempt_id: str,
                         worker_id: str, model: str, project_url: str,
                         submission_id: str | None = None,
                         fallback_reason: str | None = None) -> dict[str, Any]:
        """Record the irreversible-send intent before touching the UI.

        The intent is immutable.  A restart that finds ``SUBMITTING`` must
        reconcile the existing UI attempt rather than send the packet again.
        """
        execution_id = _validate_id(execution_id, "execution_id")
        review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
        worker_id = _validate_id(worker_id, "worker_id")
        if model not in ALLOWED_MODELS:
            raise ReviewQueueError("submission_model_invalid")
        if project_url != APPROVED_PROJECT_URL:
            raise ReviewQueueError("submission_project_invalid")
        if (model == "GPT-5.6 Pro") != bool(fallback_reason and fallback_reason.strip()):
            raise ReviewQueueError("submission_provenance_invalid")
        if submission_id is not None:
            submission_id = _validate_id(submission_id, "submission_id")
        with self._queue_lock():
            self._reject_existing_terminal(review_attempt_id, "SUBMITTING")
            request_path, request, claim = self._bound_claim(execution_id, review_attempt_id, worker_id)
            lease_path = self._singleflight_lease_path(review_attempt_id)
            if not lease_path.exists():
                raise ReviewQueueError("submission_lease_missing")
            lease = _read_json(lease_path, "singleflight_lease")
            if not self._singleflight_lease_valid(lease, request, request_path, review_attempt_id):
                raise ReviewQueueError("submission_lease_invalid")
            intent_path = self._submission_intent_path(review_attempt_id)
            if intent_path.exists():
                raise ReviewQueueError("submission_intent_already_recorded")
            payload = {
                "schema_version": SCHEMA_VERSION, "execution_id": execution_id,
                "review_attempt_id": review_attempt_id, "request_sha256": _sha256(request_path),
                "handoff_sha256": request["handoff_sha256"], "packet_sha256": request["packet_sha256"],
                "claim_sha256": _sha256(self._record("claims", review_attempt_id)),
                "lease_sha256": _sha256(lease_path), "worker_id": claim["worker_id"],
                "model": model, "model_fallback_reason": fallback_reason,
                "project_url": project_url, "submission_id": submission_id,
                "status": "SUBMITTING", "started_at": _now_utc(),
            }
            _write_exclusive(intent_path, payload)
        return self.status(execution_id, review_attempt_id)

    def _bound_submission(self, execution_id: str, review_attempt_id: str,
                          worker_id: str) -> tuple[Path, dict[str, Any], dict[str, Any], Path]:
        request_path, request, claim = self._bound_claim(execution_id, review_attempt_id, worker_id)
        lease_path = self._singleflight_lease_path(review_attempt_id)
        if not lease_path.exists():
            raise ReviewQueueError("submission_lease_missing")
        lease = _read_json(lease_path, "singleflight_lease")
        if not self._singleflight_lease_valid(lease, request, request_path, review_attempt_id):
            raise ReviewQueueError("submission_lease_invalid")
        intent_path = self._submission_intent_path(review_attempt_id)
        if not intent_path.exists():
            raise ReviewQueueError("submission_intent_missing")
        intent = _read_json(intent_path, "submission_intent")
        if not self._submission_intent_valid(intent, request, request_path, claim, review_attempt_id):
            raise ReviewQueueError("submission_intent_invalid")
        submission_path = self._record("submissions", review_attempt_id)
        if not submission_path.exists():
            raise ReviewQueueError("submission_record_missing")
        submission = _read_json(submission_path, "submission")
        if not self._submission_valid(submission, request, request_path, claim, review_attempt_id):
            raise ReviewQueueError("submission_schema_or_binding_invalid")
        return request_path, request, claim, submission_path

    def record_submission(self, *, execution_id: str, review_attempt_id: str,
                          worker_id: str, model: str, submission_id: str,
                          project_url: str, fallback_reason: str | None = None) -> dict[str, Any]:
        """Persist the one immutable UI-submission marker for an attempt.

        This is intentionally separate from ``complete`` so a restarted
        transport can observe ``SUBMITTED`` and monitor the same UI response
        without sending the packet again.
        """
        execution_id = _validate_id(execution_id, "execution_id")
        review_attempt_id = _validate_id(review_attempt_id, "review_attempt_id")
        worker_id = _validate_id(worker_id, "worker_id")
        submission_id = _validate_id(submission_id, "submission_id")
        if model not in ALLOWED_MODELS:
            raise ReviewQueueError("submission_model_invalid")
        if project_url != APPROVED_PROJECT_URL:
            raise ReviewQueueError("submission_project_invalid")
        if (model == "GPT-5.6 Pro") != bool(fallback_reason and fallback_reason.strip()):
            raise ReviewQueueError("submission_provenance_invalid")
        with self._queue_lock():
            self._reject_existing_terminal(review_attempt_id, "SUBMITTED")
            request_path, request, claim = self._bound_claim(execution_id, review_attempt_id, worker_id)
            submission_path = self._record("submissions", review_attempt_id)
            if submission_path.exists():
                raise ReviewQueueError("submission_already_recorded")
            lease_path = self._singleflight_lease_path(review_attempt_id)
            if not lease_path.exists():
                raise ReviewQueueError("submission_lease_missing")
            lease = _read_json(lease_path, "singleflight_lease")
            if not self._singleflight_lease_valid(lease, request, request_path, review_attempt_id):
                raise ReviewQueueError("submission_lease_invalid")
            intent_path = self._submission_intent_path(review_attempt_id)
            if not intent_path.exists():
                raise ReviewQueueError("submission_intent_missing")
            intent = _read_json(intent_path, "submission_intent")
            if not self._submission_intent_valid(
                    intent, request, request_path, claim, review_attempt_id):
                raise ReviewQueueError("submission_intent_invalid")
            if any(intent.get(key) != value for key, value in {
                "model": model, "project_url": project_url,
                "model_fallback_reason": fallback_reason,
            }.items()):
                raise ReviewQueueError("submission_intent_mismatch")
            if intent.get("submission_id") is not None and intent.get("submission_id") != submission_id:
                raise ReviewQueueError("submission_intent_mismatch")
            payload = {
                "schema_version": SCHEMA_VERSION, "execution_id": execution_id,
                "review_attempt_id": review_attempt_id, "request_sha256": _sha256(request_path),
                "handoff_sha256": request["handoff_sha256"], "packet_sha256": request["packet_sha256"],
                "claim_sha256": _sha256(self._record("claims", review_attempt_id)),
                "lease_sha256": _sha256(lease_path) if lease_path.exists() else None,
                "worker_id": claim["worker_id"], "model": model,
                "model_fallback_reason": fallback_reason, "project_url": project_url,
                "submission_id": submission_id, "submission_verified": True,
                "submitted_at": _now_utc(),
            }
            _write_exclusive(submission_path, payload)
        return self.status(execution_id, review_attempt_id)

    def defer(self, *, execution_id: str, review_attempt_id: str, worker_id: str,
              evidence: Path, evidence_sha256: str, terminal_connection_error: str | None = None) -> dict[str, Any]:
        with self._queue_lock():
            result = self._defer_locked(
                execution_id=execution_id, review_attempt_id=review_attempt_id, worker_id=worker_id,
                evidence=evidence, evidence_sha256=evidence_sha256,
                terminal_connection_error=terminal_connection_error,
            )
        return result

    def _defer_locked(self, *, execution_id: str, review_attempt_id: str, worker_id: str,
                      evidence: Path, evidence_sha256: str,
                      terminal_connection_error: str | None = None) -> dict[str, Any]:
        if self._record("results", review_attempt_id).exists():
            raise ReviewQueueError("review_already_completed")
        self._reject_existing_terminal(review_attempt_id, "DEFERRED_CONNECTION_UNAVAILABLE")
        if self._record("submissions", review_attempt_id).exists():
            raise ReviewQueueError("review_already_submitted")
        if self._submission_intent_path(review_attempt_id).exists():
            raise ReviewQueueError("review_already_submitting")
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

    def terminal_failure(self, *, execution_id: str, review_attempt_id: str, worker_id: str,
                         evidence: Path, evidence_sha256: str) -> dict[str, Any]:
        """Record a submitted review's explicit retryable UI terminal failure."""
        with self._queue_lock():
            result = self._terminal_failure_locked(
                execution_id=execution_id, review_attempt_id=review_attempt_id,
                worker_id=worker_id, evidence=evidence, evidence_sha256=evidence_sha256,
            )
        return result

    def _terminal_failure_locked(self, *, execution_id: str, review_attempt_id: str,
                                 worker_id: str, evidence: Path,
                                 evidence_sha256: str) -> dict[str, Any]:
        """Implementation of :meth:`terminal_failure` under the queue lock."""
        if self._record("results", review_attempt_id).exists():
            raise ReviewQueueError("review_already_completed")
        if self._record("deferrals", review_attempt_id).exists():
            raise ReviewQueueError("review_already_deferred")
        self._reject_existing_terminal(review_attempt_id, "RETRYABLE_UI_TERMINAL_FAILURE")
        request_path, request, _, submission_path = self._bound_submission(
            execution_id, review_attempt_id, worker_id
        )
        evidence = _regular_file(evidence, "terminal_failure_evidence")
        if not re.fullmatch(r"[0-9a-f]{64}", evidence_sha256) or _sha256(evidence) != evidence_sha256:
            raise ReviewQueueError("terminal_failure_evidence_hash_mismatch")
        evidence_payload = _read_json(evidence, "terminal_failure_evidence")
        if not self._retryable_terminal_failure_evidence_valid(evidence_payload):
            raise ReviewQueueError("terminal_failure_evidence_invalid")
        submission_payload = _read_json(submission_path, "submission")
        if submission_payload.get("submission_id") != evidence_payload.get("submission_id"):
            raise ReviewQueueError("terminal_failure_submission_mismatch")
        if (evidence_payload.get("terminal_error_kind") == "INCOMPLETE_RESPONSE_AFTER_30M"
                and submission_payload.get("submitted_at") != evidence_payload.get("submitted_at")):
            raise ReviewQueueError("terminal_failure_submission_time_mismatch")
        payload = {
            "schema_version": SCHEMA_VERSION, "execution_id": execution_id,
            "review_attempt_id": review_attempt_id, "status": "RETRYABLE_UI_TERMINAL_FAILURE",
            "resume_action": "REVIEW_SAME_IMMUTABLE_EXECUTION",
            "request_sha256": _sha256(request_path), "handoff_sha256": request["handoff_sha256"],
            "packet_sha256": request["packet_sha256"], "worker_id": worker_id,
            "claim_sha256": _sha256(self._record("claims", review_attempt_id)),
            "terminal_failure_evidence_path": str(evidence),
            "terminal_failure_evidence_sha256": evidence_sha256,
            "submission_sha256": _sha256(submission_path),
            "submission_id": evidence_payload["submission_id"],
            "terminal_error_kind": evidence_payload["terminal_error_kind"],
            "terminal_error_text": evidence_payload["terminal_error_text"],
            "observed_at": evidence_payload["observed_at"], "retryable": True,
            "workflow_advance_allowed": False, "failed_at": _now_utc(),
            "terminal_kind": "RETRYABLE_UI_TERMINAL_FAILURE",
        }
        self._write_terminal_marker(payload)
        return self.status(execution_id, review_attempt_id)

    @staticmethod
    def _retryable_terminal_failure_evidence_valid(evidence: Any) -> bool:
        common = {"schema_version", "submission_id", "terminal_error_kind",
                  "terminal_error_text", "observed_at", "retryable"}
        if (not isinstance(evidence, dict) or evidence.get("schema_version") != 1
                or not isinstance(evidence.get("submission_id"), str)
                or not _ID_RE.fullmatch(evidence["submission_id"])
                or not all(isinstance(evidence.get(field), str) and evidence[field].strip()
                           and len(evidence[field]) <= 4096
                           for field in ("terminal_error_kind", "terminal_error_text", "observed_at"))
                or _parse_utc(evidence.get("observed_at")) is None
                or evidence.get("retryable") is not True):
            return False
        if evidence.get("terminal_error_kind") in {
            "RATE_LIMIT", "NETWORK_DISCONNECTED",
        }:
            return set(evidence) == common
        if evidence.get("terminal_error_kind") != "INCOMPLETE_RESPONSE_AFTER_30M":
            return False
        required = common | {"submitted_at", "checkpoint_observations", "missing_required_fields"}
        submitted_at = _parse_utc(evidence.get("submitted_at"))
        observed_at = _parse_utc(evidence.get("observed_at"))
        required_missing = {
            "execution_or_handoff_binding", "severity", "disposition",
            "minimum_changes", "transition_permission",
        }
        checkpoints = evidence.get("checkpoint_observations")
        if (set(evidence) != required or submitted_at is None or observed_at is None
                or observed_at < submitted_at + timedelta(minutes=30)
                or set(evidence.get("missing_required_fields", [])) != required_missing
                or not isinstance(checkpoints, list) or len(checkpoints) != 3):
            return False
        for expected_minute, checkpoint in zip((10, 20, 30), checkpoints, strict=True):
            if (not isinstance(checkpoint, dict)
                    or set(checkpoint) != {"elapsed_minutes", "observed_at", "reloaded",
                                           "response_text", "generating", "ui_errors"}
                    or checkpoint.get("elapsed_minutes") != expected_minute
                    or checkpoint.get("reloaded") is not True
                    or checkpoint.get("generating") is not False
                    or checkpoint.get("ui_errors") != []
                    or checkpoint.get("response_text") != evidence["terminal_error_text"]):
                return False
            checkpoint_at = _parse_utc(checkpoint.get("observed_at"))
            if checkpoint_at is None or checkpoint_at < submitted_at + timedelta(minutes=expected_minute):
                return False
        return True

    @staticmethod
    def _connection_attempt_valid(item: Any) -> bool:
        return (isinstance(item, dict) and set(item) == {"timestamp", "outcome", "timeout_s"}
                and isinstance(item["timestamp"], str) and bool(item["timestamp"].strip())
                and item.get("outcome") in ALLOWED_CONNECTION_UNAVAILABLE_OUTCOMES
                and type(item["timeout_s"]) in (int, float) and 0 < item["timeout_s"] <= 1800)

    @staticmethod
    def _legacy_connection_attempts_valid(attempts: Any) -> bool:
        strict = (isinstance(attempts, list) and len(attempts) >= 3
                  and all(ReviewQueue._connection_attempt_valid(item) for item in attempts))
        historical_profile_busy = (
            isinstance(attempts, list) and len(attempts) == 3
            and all(
                isinstance(item, dict)
                and set(item) == {"timestamp", "outcome", "timeout_s"}
                and isinstance(item["timestamp"], str) and bool(item["timestamp"].strip())
                and item.get("outcome") == LEGACY_BROWSER_PROFILE_IN_USE_OUTCOME
                and type(item["timeout_s"]) in (int, float)
                and 0 < item["timeout_s"] <= 1800
                for item in attempts
            )
        )
        return strict or historical_profile_busy

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
        with self._queue_lock():
            result = self._complete_locked(
                execution_id=execution_id, review_attempt_id=review_attempt_id,
                worker_id=worker_id, model=model, severity=severity,
                disposition=disposition, transition_permission=transition_permission,
                response=response, minimum_changes=minimum_changes,
                submission_id=submission_id, project_url=project_url,
                fallback_reason=fallback_reason,
            )
        return result

    def _complete_locked(self, *, execution_id: str, review_attempt_id: str,
                         worker_id: str, model: str, severity: str,
                         disposition: str, transition_permission: str,
                         response: Path, minimum_changes: list[str],
                         submission_id: str, project_url: str,
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
        request_path, request, _, submission_path = self._bound_submission(
            execution_id, review_attempt_id, worker_id
        )
        submission = _read_json(submission_path, "submission")
        if any(submission.get(key) != value for key, value in {
            "model": model, "project_url": project_url,
            "submission_id": submission_id, "model_fallback_reason": fallback_reason,
        }.items()):
            raise ReviewQueueError("completion_submission_mismatch")
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
                   "submission_sha256": _sha256(submission_path),
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
        if kind == "RETRYABLE_UI_TERMINAL_FAILURE":
            raise ReviewQueueError("review_already_terminal_failure")
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
        if self._retirement_path(retry_of).exists():
            return False
        try:
            prior_request = _read_json(self._record("requests", retry_of), "prior_request")
            terminal_path = self._record("terminals", retry_of)
            prior_terminal = _read_json(terminal_path if terminal_path.exists() else self._record("deferrals", retry_of), "prior_terminal")
        except (OSError, ReviewQueueError):
            return False
        if self._record("results", retry_of).exists():
            return False
        terminal_kind = prior_terminal.get("terminal_kind", "DEFERRED_CONNECTION_UNAVAILABLE")
        if terminal_kind == "COMPLETED":
            return False
        is_legacy = prior_request.get("schema_version") == LEGACY_SCHEMA_VERSION
        if is_legacy:
            prior_valid = (not self._legacy_request_errors(prior_request, request.get("execution_id", ""), retry_of)
                           and self._legacy_deferral_valid(prior_terminal, prior_request,
                                                           self._record("deferrals", retry_of), retry_of))
        else:
            prior_valid = (not self._request_schema_errors(prior_request, request.get("execution_id", ""), retry_of)
                           and (self._deferral_valid(prior_terminal, prior_request,
                                                     self._record("deferrals", retry_of))
                                or self._retryable_terminal_failure_valid(prior_terminal, prior_request)))
        return (prior_valid
                and prior_request.get("handoff_sha256") == request.get("handoff_sha256")
                and prior_request.get("packet_sha256") == request.get("packet_sha256"))

    def _retirement_valid(
        self, retirement: dict[str, Any], request: dict[str, Any], request_path: Path,
        review_attempt_id: str,
    ) -> bool:
        required = {
            "schema_version", "execution_id", "review_attempt_id", "status",
            "request_sha256", "handoff_sha256", "packet_sha256", "reason_code",
            "reason", "retired_by", "retired_at", "workflow_advance_allowed",
            "retry_eligible",
        }
        return (
            set(retirement) == required
            and retirement.get("schema_version") == 1
            and retirement.get("execution_id") == request.get("execution_id")
            and retirement.get("review_attempt_id") == review_attempt_id
            and retirement.get("status") == "RETIRED_PENDING"
            and retirement.get("request_sha256") == _sha256(request_path)
            and retirement.get("handoff_sha256") == request.get("handoff_sha256")
            and retirement.get("packet_sha256") == request.get("packet_sha256")
            and isinstance(request.get("retry_of"), str)
            and bool(_ID_RE.fullmatch(request["retry_of"]))
            and not self._retry_lineage_valid(request)
            and retirement.get("reason_code") == RETIREMENT_REASON_CODE
            and isinstance(retirement.get("reason"), str)
            and bool(retirement["reason"].strip())
            and len(retirement["reason"]) <= 4096
            and isinstance(retirement.get("retired_by"), str)
            and bool(_ID_RE.fullmatch(retirement["retired_by"]))
            and _parse_utc(retirement.get("retired_at")) is not None
            and retirement.get("workflow_advance_allowed") is False
            and retirement.get("retry_eligible") is False
        )

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
        valid_fields = set(request) in (required, required | {"not_before"})
        not_before_valid = ("not_before" not in request
                            or _parse_utc(request.get("not_before")) is not None)
        valid = (valid_fields and not_before_valid and request.get("schema_version") == SCHEMA_VERSION
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

    def _retryable_terminal_failure_valid(self, failure: dict[str, Any], request: dict[str, Any]) -> bool:
        try:
            evidence = _regular_file(Path(str(failure["terminal_failure_evidence_path"])), "terminal_failure_evidence")
            evidence_payload = _read_json(evidence, "terminal_failure_evidence")
            claim_path = self._record("claims", request["review_attempt_id"])
            claim = _read_json(claim_path, "claim")
            bindings_valid = (
                _sha256(evidence) == failure["terminal_failure_evidence_sha256"]
                and self._retryable_terminal_failure_evidence_valid(evidence_payload)
                and all(failure.get(field) == evidence_payload[field] for field in (
                    "submission_id", "terminal_error_kind", "terminal_error_text", "observed_at", "retryable"))
                and self._claim_valid(claim, request, self._record("requests", request["review_attempt_id"]),
                                      request["review_attempt_id"])
                and claim.get("worker_id") == failure.get("worker_id")
                and _sha256(claim_path) == failure.get("claim_sha256")
                and _sha256(self._record("submissions", request["review_attempt_id"])) == failure.get("submission_sha256")
                and self._submission_valid(
                    _read_json(self._record("submissions", request["review_attempt_id"]), "submission"),
                    request, self._record("requests", request["review_attempt_id"]), claim,
                    request["review_attempt_id"],
                )
                and _read_json(
                    self._record("submissions", request["review_attempt_id"]), "submission"
                ).get("submission_id") == failure.get("submission_id"))
        except (KeyError, OSError, ReviewQueueError):
            bindings_valid = False
        required = {
            "schema_version", "execution_id", "review_attempt_id", "status", "resume_action",
            "request_sha256", "handoff_sha256", "packet_sha256", "worker_id", "claim_sha256",
            "terminal_failure_evidence_path", "terminal_failure_evidence_sha256", "submission_id",
            "submission_sha256",
            "terminal_error_kind", "terminal_error_text", "observed_at", "retryable",
            "workflow_advance_allowed", "failed_at", "terminal_kind",
        }
        return (set(failure) == required and failure.get("schema_version") == SCHEMA_VERSION
                and failure.get("execution_id") == request.get("execution_id")
                and failure.get("review_attempt_id") == request.get("review_attempt_id")
                and failure.get("status") == "RETRYABLE_UI_TERMINAL_FAILURE"
                and failure.get("resume_action") == "REVIEW_SAME_IMMUTABLE_EXECUTION"
                and failure.get("request_sha256") == _sha256(self._record("requests", request["review_attempt_id"]))
                and failure.get("handoff_sha256") == request.get("handoff_sha256")
                and failure.get("packet_sha256") == request.get("packet_sha256")
                and failure.get("retryable") is True
                and failure.get("workflow_advance_allowed") is False
                and failure.get("terminal_kind") == "RETRYABLE_UI_TERMINAL_FAILURE"
                and bindings_valid)

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
                and bindings_valid
                and self._legacy_connection_attempts_valid(deferral.get("connection_attempts")))

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
                    "advisory_only", "workflow_advance_allowed", "completed_at", "terminal_kind",
                    "claim_sha256", "submission_sha256"}
        legacy_required = required - {"submission_sha256"}
        legacy_result = (
            "submission_sha256" not in result
            and not self._singleflight_lease_path(review_attempt_id).exists()
            and set(result) == legacy_required
        )
        fields = ("execution_id", "review_attempt_id", "handoff_sha256", "model", "model_fallback_reason",
                  "project_url", "submission_id", "submission_verified", "severity", "disposition",
                  "minimum_changes", "transition_permission")
        submission: dict[str, Any] = {}
        try:
            submission = _read_json(self._record("submissions", review_attempt_id), "submission")
            submission_valid = self._submission_valid(
                submission, request, request_path, claim, review_attempt_id
            )
        except (OSError, ReviewQueueError):
            submission_valid = False
        return ((set(result) == required or legacy_result) and result.get("schema_version") == SCHEMA_VERSION
                and result.get("execution_id") == execution_id and result.get("review_attempt_id") == review_attempt_id
                and result.get("status") == "COMPLETED" and result.get("request_sha256") == _sha256(request_path)
                and result.get("handoff_sha256") == request.get("handoff_sha256")
                and result.get("packet_sha256") == request.get("packet_sha256") and result.get("worker_id") == claim.get("worker_id")
                and result.get("claim_sha256") == _sha256(self._record("claims", review_attempt_id))
                and (legacy_result or (
                    result.get("submission_sha256") == _sha256(self._record("submissions", review_attempt_id))
                    and submission_valid
                    and submission.get("submission_id") == result.get("submission_id")
                    and submission.get("model") == result.get("model")
                    and submission.get("project_url") == result.get("project_url")
                ))
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
        retirement_path = self._retirement_path(review_attempt_id)
        retirement = _read_json(retirement_path, "retirement") if retirement_path.exists() else None
        if retirement is None and not self._retry_lineage_valid(request):
            errors.append("retry_lineage_invalid")
        claim_path, result_path, deferral_path = (self._record(c, review_attempt_id) for c in ("claims", "results", "deferrals"))
        intent_path = self._submission_intent_path(review_attempt_id)
        submission_path = self._record("submissions", review_attempt_id)
        marker_path = self._record("terminals", review_attempt_id)
        claim = _read_json(claim_path, "claim") if claim_path.exists() else None
        result = _read_json(result_path, "result") if result_path.exists() else None
        deferral = _read_json(deferral_path, "deferral") if deferral_path.exists() else None
        intent = _read_json(intent_path, "submission_intent") if intent_path.exists() else None
        submission = _read_json(submission_path, "submission") if submission_path.exists() else None
        terminal_failure = None
        marker = _read_json(marker_path, "terminal_marker") if marker_path.exists() else None
        if marker and marker.get("terminal_kind") == "COMPLETED":
            if result or deferral:
                errors.append("terminal_records_conflict")
            result = marker
        elif marker and marker.get("terminal_kind") == "DEFERRED_CONNECTION_UNAVAILABLE":
            if result or deferral:
                errors.append("terminal_records_conflict")
            deferral = marker
        elif marker and marker.get("terminal_kind") == "RETRYABLE_UI_TERMINAL_FAILURE":
            if result or deferral:
                errors.append("terminal_records_conflict")
            terminal_failure = marker
        elif marker:
            errors.append("terminal_marker_invalid")
        if retirement:
            retirement_conflict = any(
                item is not None for item in (
                    claim, result, deferral, intent, submission, marker, terminal_failure,
                )
            ) or self._singleflight_lease_path(review_attempt_id).exists()
            if (errors or retirement_conflict
                    or not self._retirement_valid(
                        retirement, request, request_path, review_attempt_id)):
                errors.append("retirement_schema_or_binding_invalid")
        if claim and not self._claim_valid(claim, request, request_path, review_attempt_id): errors.append("claim_schema_or_binding_invalid")
        if intent and (not claim or not self._submission_intent_valid(intent, request, request_path, claim, review_attempt_id)):
            errors.append("submission_intent_schema_or_binding_invalid")
        if submission and (not claim or not self._submission_valid(submission, request, request_path, claim, review_attempt_id)):
            errors.append("submission_schema_or_binding_invalid")
        if submission and not intent:
            errors.append("submission_intent_missing")
        if deferral and (not claim or not self._deferral_valid(deferral, request, deferral_path)): errors.append("deferral_schema_or_binding_invalid")
        if result and (not claim or not self._result_valid(result, request, request_path, claim, execution_id, review_attempt_id)):
            errors.append("result_schema_or_binding_invalid")
        if result and deferral:
            errors.append("terminal_records_conflict")
        if terminal_failure and not claim:
            errors.append("terminal_failure_schema_or_binding_invalid")
        elif terminal_failure and not self._retryable_terminal_failure_valid(terminal_failure, request):
            errors.append("terminal_failure_schema_or_binding_invalid")
        if (result or deferral or terminal_failure) and marker is None:
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
            elif terminal_failure:
                if not self._terminal_marker_valid(marker, request, request_path, claim, claim_path, execution_id, review_attempt_id, "RETRYABLE_UI_TERMINAL_FAILURE"):
                    errors.append("terminal_marker_invalid")
        state = "RETIRED_PENDING" if retirement else ("COMPLETED" if result else ("DEFERRED_CONNECTION_UNAVAILABLE" if deferral else ("RETRYABLE_UI_TERMINAL_FAILURE" if terminal_failure else ("SUBMITTED" if submission else ("SUBMITTING" if intent else ("CLAIMED" if claim else "REVIEW_PENDING"))))))
        return {"schema_version": SCHEMA_VERSION, "queue_root": str(self.root), "execution_id": execution_id, "review_attempt_id": review_attempt_id,
                "state": "INVALID_EVIDENCE" if errors else state, "integrity_valid": not errors,
                "integrity_errors": errors, "request": request, "claim": claim, "result": result,
                "deferral": deferral, "terminal_failure": terminal_failure,
                "submission_intent": intent, "submission": submission, "retirement": retirement,
                "workflow_advance_allowed": False}

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
