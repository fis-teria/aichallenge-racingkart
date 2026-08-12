from __future__ import annotations

import hashlib
import io
import json
import tarfile
from pathlib import Path

import pytest

from aic_test.cli import build_parser
from aic_test.rebuild_preflight import (
    _FIXED_ARCHIVE_PATH,
    _FIXED_FILES,
    _FIXED_INSTALLED_EXECUTABLE,
    verify_rebuild_preflight,
)


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _workspace(tmp_path: Path) -> tuple[Path, Path, dict[str, object], dict[str, bytes]]:
    root, docs = tmp_path / "workspace", tmp_path / "workspace" / "docs"
    docs.mkdir(parents=True)
    handoff = docs / "handoff.md"
    handoff.write_text("immutable handoff\n", encoding="utf-8")
    contents: dict[str, bytes] = {}
    entries: list[dict[str, str]] = []
    for index, (host_path, archive_member) in enumerate(sorted(_FIXED_FILES.items())):
        data = f"reviewed-{index}\n".encode()
        (root / host_path).parent.mkdir(parents=True, exist_ok=True)
        (root / host_path).write_bytes(data)
        contents[archive_member] = data
        entries.append({"host_path": host_path, "archive_member": archive_member, "sha256": _sha(data)})
    manifest: dict[str, object] = {
        "schema_version": 1,
        "execution_id": "test-rebuild-preflight",
        "archive_path": _FIXED_ARCHIVE_PATH,
        "immutable_handoff": {"path": "docs/handoff.md", "sha256": _sha(handoff.read_bytes())},
        "files": entries,
        "installed_planner_executable_path": _FIXED_INSTALLED_EXECUTABLE,
    }
    path = docs / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return root, path, manifest, contents


def _write_manifest(path: Path, manifest: dict[str, object]) -> None:
    path.write_text(json.dumps(manifest), encoding="utf-8")


def _archive(path: Path, contents: dict[str, bytes], *, omit: str | None = None,
             duplicate: str | None = None, nonregular: str | None = None) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(path, "w:gz") as archive:
        for name, data in contents.items():
            if name == omit:
                continue
            info = tarfile.TarInfo(name)
            info.size = len(data)
            if name == nonregular:
                info.type = tarfile.SYMTYPE
                info.linkname = "elsewhere"
                info.size = 0
                archive.addfile(info)
            else:
                archive.addfile(info, io.BytesIO(data))
            if name == duplicate:
                duplicate_info = tarfile.TarInfo(name)
                duplicate_info.size = len(data)
                archive.addfile(duplicate_info, io.BytesIO(data))
    return path


def test_cli_exposes_rebuild_preflight() -> None:
    args = build_parser().parse_args(["rebuild-preflight", "--manifest", "docs/manifest.json"])
    assert args.archive is None


def test_success_for_host_and_archive_bytes(tmp_path: Path) -> None:
    root, manifest_path, _, contents = _workspace(tmp_path)
    archive = _archive(root / _FIXED_ARCHIVE_PATH, contents)
    result = verify_rebuild_preflight(manifest_path, archive)
    assert result["outcome"] == "READY"
    assert result["reason"] == "host_and_archive_bytes_match"
    assert result["host_hashes"]
    assert result["archive_hashes"]


def test_host_hash_mismatch_holds(tmp_path: Path) -> None:
    root, manifest_path, manifest, _ = _workspace(tmp_path)
    first = manifest["files"][0]
    (root / first["host_path"]).write_text("mutated\n", encoding="utf-8")
    result = verify_rebuild_preflight(manifest_path)
    assert result["outcome"] == "HOLD"
    assert result["reason"].startswith("host_sha256_mismatch:")


def test_manifest_schema_version_holds(tmp_path: Path) -> None:
    _, manifest_path, manifest, _ = _workspace(tmp_path)
    manifest["schema_version"] = 2
    _write_manifest(manifest_path, manifest)
    assert verify_rebuild_preflight(manifest_path)["reason"] == "manifest_schema_version_invalid"


@pytest.mark.parametrize("mutate, reason", [
    (lambda manifest: manifest["files"].pop(), "manifest_files_count_invalid"),
    (lambda manifest: manifest["files"].append(dict(manifest["files"][0])), "manifest_files_count_invalid"),
    (lambda manifest: manifest["files"].__setitem__(0, {**manifest["files"][0], "host_path": "unexpected.cpp"}), "manifest_unexpected_host_path"),
])
def test_manifest_missing_duplicate_or_unexpected_entry_holds(tmp_path: Path, mutate, reason: str) -> None:
    _, manifest_path, manifest, _ = _workspace(tmp_path)
    mutate(manifest)
    _write_manifest(manifest_path, manifest)
    assert verify_rebuild_preflight(manifest_path)["reason"] == reason


def test_unsafe_archive_target_member_holds(tmp_path: Path) -> None:
    root, manifest_path, manifest, contents = _workspace(tmp_path)
    target = manifest["files"][0]["archive_member"]
    result = verify_rebuild_preflight(
        manifest_path, _archive(root / _FIXED_ARCHIVE_PATH, contents, nonregular=target)
    )
    assert result["outcome"] == "HOLD"
    assert result["reason"] == f"archive_member_not_regular:{target}"


@pytest.mark.parametrize("kind", ["mismatch", "missing", "duplicate"])
def test_archive_mismatch_missing_or_duplicate_holds(tmp_path: Path, kind: str) -> None:
    root, manifest_path, manifest, contents = _workspace(tmp_path)
    target = manifest["files"][0]["archive_member"]
    if kind == "mismatch":
        contents[target] = b"wrong bytes\n"
        archive = _archive(root / _FIXED_ARCHIVE_PATH, contents)
        expected = f"archive_member_sha256_mismatch:{target}"
    elif kind == "missing":
        archive = _archive(root / _FIXED_ARCHIVE_PATH, contents, omit=target)
        expected = f"archive_member_missing:{target}"
    else:
        archive = _archive(root / _FIXED_ARCHIVE_PATH, contents, duplicate=target)
        expected = f"archive_member_duplicate:{target}"
    result = verify_rebuild_preflight(manifest_path, archive)
    assert result["outcome"] == "HOLD"
    assert result["reason"] == expected


def test_other_archive_path_holds_even_when_its_bytes_match(tmp_path: Path) -> None:
    _, manifest_path, _, contents = _workspace(tmp_path)
    result = verify_rebuild_preflight(manifest_path, _archive(tmp_path / "other.tar.gz", contents))
    assert result["outcome"] == "HOLD"
    assert result["reason"] == "archive_path_invalid"


def test_boolean_schema_version_is_rejected(tmp_path: Path) -> None:
    _, manifest_path, manifest, _ = _workspace(tmp_path)
    manifest["schema_version"] = True
    _write_manifest(manifest_path, manifest)
    result = verify_rebuild_preflight(manifest_path)
    assert result["outcome"] == "HOLD"
    assert result["reason"] == "manifest_schema_version_invalid"


def test_canonical_archive_symlink_is_rejected(tmp_path: Path) -> None:
    root, manifest_path, _, contents = _workspace(tmp_path)
    real_archive = _archive(tmp_path / "real.tar.gz", contents)
    canonical_archive = root / _FIXED_ARCHIVE_PATH
    canonical_archive.parent.mkdir(parents=True, exist_ok=True)
    canonical_archive.symlink_to(real_archive)
    result = verify_rebuild_preflight(manifest_path, canonical_archive)
    assert result["outcome"] == "HOLD"
    assert result["reason"] == "archive_path_invalid"
