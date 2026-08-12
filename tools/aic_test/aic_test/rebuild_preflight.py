"""Fail-closed source/archive binding before an eval-image rebuild.

This module deliberately does not create an archive, inspect Docker, or start a
runtime.  It only establishes that the five reviewed source files and, when
provided, their archive members are the exact bytes named by the manifest.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import stat
import tarfile
from pathlib import Path, PurePosixPath
from typing import Any


SCHEMA_VERSION = 1
_HEX = frozenset("0123456789abcdef")
_PACKAGE_ROOT = "aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/state_lattice_overtake_planner"
_ARCHIVE_ROOT = "aichallenge_submit/state_lattice_overtake_planner"
_FIXED_FILES = {
    f"{_PACKAGE_ROOT}/include/state_lattice_overtake_planner/types.hpp":
        f"{_ARCHIVE_ROOT}/include/state_lattice_overtake_planner/types.hpp",
    f"{_PACKAGE_ROOT}/src/lattice_planner.cpp":
        f"{_ARCHIVE_ROOT}/src/lattice_planner.cpp",
    f"{_PACKAGE_ROOT}/src/state_lattice_overtake_planner_node.cpp":
        f"{_ARCHIVE_ROOT}/src/state_lattice_overtake_planner_node.cpp",
    f"{_PACKAGE_ROOT}/test/test_contract_and_state.cpp":
        f"{_ARCHIVE_ROOT}/test/test_contract_and_state.cpp",
    f"{_PACKAGE_ROOT}/test/test_frenet_and_lattice.cpp":
        f"{_ARCHIVE_ROOT}/test/test_frenet_and_lattice.cpp",
}
_FIXED_INSTALLED_EXECUTABLE = (
    "/aichallenge/workspace/install/state_lattice_overtake_planner/lib/"
    "state_lattice_overtake_planner/state_lattice_overtake_planner_node"
)
_FIXED_ARCHIVE_PATH = "aichallenge-racingkart/submit/aichallenge_submit.tar.gz"


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _valid_sha256(value: object) -> bool:
    return isinstance(value, str) and len(value) == 64 and set(value) <= _HEX


def _normalized_relative(value: object) -> str | None:
    if not isinstance(value, str) or not value:
        return None
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or "." in path.parts:
        return None
    normalized = str(path)
    return normalized if normalized == value else None


def _regular_bytes(path: Path) -> tuple[bytes | None, str | None]:
    try:
        mode = path.lstat().st_mode
        if stat.S_ISLNK(mode):
            return None, "symlink_rejected"
        if not stat.S_ISREG(mode):
            return None, "not_regular_file"
        return path.read_bytes(), None
    except OSError:
        return None, "missing_or_unreadable"


def _hold(reason: str, **extra: Any) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "outcome": "HOLD",
        "reason": reason,
        **extra,
    }


def _hold_from_ready(reason: str, ready: dict[str, Any], **extra: Any) -> dict[str, Any]:
    """Retain stable binding fields without accidentally retaining READY state."""
    context = {
        key: value for key, value in ready.items()
        if key not in {"outcome", "reason"}
    }
    return _hold(reason, **context, **extra)


def _manifest_spec(manifest: object) -> tuple[dict[str, Any] | None, str | None]:
    if not isinstance(manifest, dict) or set(manifest) != {
        "schema_version", "execution_id", "immutable_handoff", "files",
        "installed_planner_executable_path", "archive_path",
    }:
        return None, "manifest_schema_invalid"
    if type(manifest["schema_version"]) is not int or manifest["schema_version"] != SCHEMA_VERSION:
        return None, "manifest_schema_version_invalid"
    if not isinstance(manifest["execution_id"], str) or not manifest["execution_id"]:
        return None, "manifest_execution_id_invalid"
    handoff = manifest["immutable_handoff"]
    if (
        not isinstance(handoff, dict) or set(handoff) != {"path", "sha256"}
        or _normalized_relative(handoff["path"]) is None or not _valid_sha256(handoff["sha256"])
    ):
        return None, "manifest_handoff_invalid"
    if manifest["installed_planner_executable_path"] != _FIXED_INSTALLED_EXECUTABLE:
        return None, "manifest_installed_executable_invalid"
    if (
        _normalized_relative(manifest["archive_path"]) != _FIXED_ARCHIVE_PATH
        or manifest["archive_path"] != _FIXED_ARCHIVE_PATH
    ):
        return None, "manifest_archive_path_invalid"
    files = manifest["files"]
    if not isinstance(files, list) or len(files) != len(_FIXED_FILES):
        return None, "manifest_files_count_invalid"
    observed: dict[str, str] = {}
    for entry in files:
        if not isinstance(entry, dict) or set(entry) != {"host_path", "archive_member", "sha256"}:
            return None, "manifest_file_entry_invalid"
        host_path = _normalized_relative(entry["host_path"])
        archive_member = _normalized_relative(entry["archive_member"])
        if host_path is None or archive_member is None or not _valid_sha256(entry["sha256"]):
            return None, "manifest_file_value_invalid"
        if host_path in observed:
            return None, "manifest_duplicate_host_path"
        if host_path not in _FIXED_FILES:
            return None, "manifest_unexpected_host_path"
        if _FIXED_FILES[host_path] != archive_member:
            return None, "manifest_archive_member_invalid"
        observed[host_path] = entry["sha256"]
    if set(observed) != set(_FIXED_FILES):
        return None, "manifest_fixed_file_set_invalid"
    return manifest, None


def _workspace_root(manifest_path: Path) -> Path:
    """The immutable manifest is deliberately stored under workspace-root/docs."""
    return manifest_path.resolve().parent.parent


def verify_rebuild_preflight(manifest_path: Path, archive_path: Path | None = None) -> dict[str, Any]:
    """Read-only validation of reviewed host bytes and optional archive bytes."""
    raw, error = _regular_bytes(manifest_path)
    if error is not None or raw is None:
        return _hold(f"manifest_{error}")
    try:
        manifest_data = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError):
        return _hold("manifest_json_invalid", manifest_sha256=_sha256_bytes(raw))
    manifest, error = _manifest_spec(manifest_data)
    if error is not None or manifest is None:
        return _hold(error or "manifest_invalid", manifest_sha256=_sha256_bytes(raw))

    root = _workspace_root(manifest_path)
    handoff = manifest["immutable_handoff"]
    handoff_path = root / handoff["path"]
    handoff_bytes, handoff_error = _regular_bytes(handoff_path)
    if handoff_error is not None or handoff_bytes is None:
        return _hold(f"handoff_{handoff_error}", manifest_sha256=_sha256_bytes(raw))
    if _sha256_bytes(handoff_bytes) != handoff["sha256"]:
        return _hold("handoff_sha256_mismatch", manifest_sha256=_sha256_bytes(raw))

    expected = {entry["host_path"]: entry for entry in manifest["files"]}
    host_hashes: dict[str, str] = {}
    for host_path in sorted(expected):
        host_bytes, host_error = _regular_bytes(root / host_path)
        if host_error is not None or host_bytes is None:
            return _hold(f"host_{host_error}:{host_path}", manifest_sha256=_sha256_bytes(raw),
                         handoff_sha256=handoff["sha256"], host_hashes=host_hashes)
        digest = _sha256_bytes(host_bytes)
        host_hashes[host_path] = digest
        if digest != expected[host_path]["sha256"]:
            return _hold(f"host_sha256_mismatch:{host_path}", manifest_sha256=_sha256_bytes(raw),
                         handoff_sha256=handoff["sha256"], host_hashes=host_hashes)

    result: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "outcome": "READY",
        "reason": "host_bytes_match",
        "execution_id": manifest["execution_id"],
        "manifest_sha256": _sha256_bytes(raw),
        "handoff_sha256": handoff["sha256"],
        "host_hashes": host_hashes,
        "installed_planner_executable_path": manifest["installed_planner_executable_path"],
    }
    if archive_path is None:
        return result
    expected_archive_path = root / manifest["archive_path"]
    try:
        if archive_path.resolve(strict=False) != expected_archive_path.resolve(strict=False):
            return _hold_from_ready("archive_path_invalid", result)
    except OSError:
        return _hold_from_ready("archive_path_invalid", result)
    if archive_path.is_symlink():
        return _hold_from_ready("archive_path_invalid", result)
    archive_bytes, archive_error = _regular_bytes(archive_path)
    if archive_error is not None or archive_bytes is None:
        return _hold_from_ready(f"archive_{archive_error}", result)
    archive_hashes: dict[str, str] = {}
    members_by_name: dict[str, list[tarfile.TarInfo]] = {}
    try:
        with tarfile.open(archive_path, "r:*") as archive:
            for member in archive.getmembers():
                if member.name in {entry["archive_member"] for entry in expected.values()}:
                    members_by_name.setdefault(member.name, []).append(member)
            for host_path in sorted(expected):
                entry = expected[host_path]
                member_name = entry["archive_member"]
                members = members_by_name.get(member_name, [])
                if not members:
                    return _hold_from_ready(
                        f"archive_member_missing:{member_name}", result,
                        archive_sha256=_sha256_bytes(archive_bytes), archive_hashes=archive_hashes)
                if len(members) != 1:
                    return _hold_from_ready(
                        f"archive_member_duplicate:{member_name}", result,
                        archive_sha256=_sha256_bytes(archive_bytes), archive_hashes=archive_hashes)
                member = members[0]
                if not member.isreg() or member.issym() or member.islnk():
                    return _hold_from_ready(
                        f"archive_member_not_regular:{member_name}", result,
                        archive_sha256=_sha256_bytes(archive_bytes), archive_hashes=archive_hashes)
                extracted = archive.extractfile(member)
                if extracted is None:
                    return _hold_from_ready(
                        f"archive_member_unreadable:{member_name}", result,
                        archive_sha256=_sha256_bytes(archive_bytes), archive_hashes=archive_hashes)
                with extracted:
                    digest = _sha256_bytes(extracted.read())
                archive_hashes[member_name] = digest
                if digest != entry["sha256"]:
                    return _hold_from_ready(
                        f"archive_member_sha256_mismatch:{member_name}", result,
                        archive_sha256=_sha256_bytes(archive_bytes), archive_hashes=archive_hashes)
    except (OSError, tarfile.TarError):
        return _hold_from_ready(
            "archive_invalid", result, archive_sha256=_sha256_bytes(archive_bytes),
            archive_hashes=archive_hashes)
    result.update({
        "reason": "host_and_archive_bytes_match",
        "archive_sha256": _sha256_bytes(archive_bytes),
        "archive_hashes": archive_hashes,
    })
    return result


def run_rebuild_preflight(args: argparse.Namespace) -> int:
    result = verify_rebuild_preflight(
        Path(args.manifest), Path(args.archive) if args.archive else None
    )
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, allow_nan=False))
    return 0 if result["outcome"] == "READY" else 4
