"""Read-only, fail-closed installed-artifact binding for private Stage 2."""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Iterable


_KINDS = {"workspace_source", "installed_script", "executable", "shared_library", "launch"}
_BINARY_KINDS = {"executable", "shared_library"}
_MAX_SYMLINK_HOPS = 16
REQUIRED_STAGE2_ROLES = frozenset({
    "mux_entry", "mux_installed_module", "private_stage2_launch", "observer",
    "runner", "offline_classifier",
})
_TERMINAL_PRECEDENCE = {
    "HOLD_ROOT_ESCAPE": 90, "HOLD_SYMLINK_CYCLE": 90, "HOLD_SYMLINK_HOP_LIMIT": 90,
    "HOLD_MISSING_ARTIFACT": 80, "HOLD_NOT_REGULAR_FILE": 80,
    "HOLD_DUPLICATE_CANONICAL_ROLE": 70, "HOLD_BINARY_KIND_MISUSE": 70,
    "HOLD_DIGEST_MISMATCH": 60, "HOLD_INVALID_EXPECTED_DIGEST": 60,
    "HOLD_BUILD_PROVENANCE_UNBOUND": 50, "HOLD_SOURCE_HASH_UNBOUND": 40,
}


def _more_severe(current: str | None, candidate: str | None) -> str | None:
    if candidate is None:
        return current
    if current is None or _TERMINAL_PRECEDENCE.get(candidate, 100) > _TERMINAL_PRECEDENCE.get(current, 100):
        return candidate
    return current


@dataclass(frozen=True)
class InstalledRoleSpec:
    role: str
    path: Path
    allowed_root: Path
    kind: str
    expected_digest: str | None
    expected_origin: str | None


def _within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _resolve_bounded(path: Path, allowed_root: Path) -> tuple[Path | None, str | None]:
    """Resolve links explicitly so an escaping intermediate link is rejected."""
    root = allowed_root.resolve(strict=False)
    current = path if path.is_absolute() else root / path
    seen: set[Path] = set()
    for _ in range(_MAX_SYMLINK_HOPS + 1):
        lexical = Path(os.path.abspath(current))
        if not _within(lexical, root):
            return None, "HOLD_ROOT_ESCAPE"
        if lexical in seen:
            return None, "HOLD_SYMLINK_CYCLE"
        if not lexical.is_symlink():
            if not lexical.exists():
                return None, "HOLD_MISSING_ARTIFACT"
            if not lexical.is_file():
                return None, "HOLD_NOT_REGULAR_FILE"
            canonical = lexical.resolve(strict=True)
            if not _within(canonical, root):
                return None, "HOLD_ROOT_ESCAPE"
            return canonical, None
        seen.add(lexical)
        target = os.readlink(lexical)
        current = Path(target) if os.path.isabs(target) else lexical.parent / target
    return None, "HOLD_SYMLINK_HOP_LIMIT"


def _kind_error(path: Path, kind: str) -> str | None:
    if kind == "executable" and not os.access(path, os.X_OK):
        return "HOLD_BINARY_KIND_MISUSE"
    if kind == "shared_library" and ".so" not in path.name:
        return "HOLD_BINARY_KIND_MISUSE"
    return None


def _missing_binding_terminal(kind: str, origin: str | None) -> str:
    if kind in _BINARY_KINDS:
        return "HOLD_BUILD_PROVENANCE_UNBOUND"
    return "HOLD_SOURCE_HASH_UNBOUND"


def attest_installed_roles(specs: Iterable[InstalledRoleSpec]) -> dict[str, Any]:
    """Attest actual host files only; never copies or treats fixtures as installs."""
    results: list[dict[str, Any]] = []
    canonical_roles: dict[Path, str] = {}
    terminal: str | None = None
    for spec in specs:
        result: dict[str, Any] = {
            "role": spec.role, "requested_path": str(spec.path),
            "allowed_root": str(spec.allowed_root), "kind": spec.kind,
            "expected_digest": spec.expected_digest, "expected_origin": spec.expected_origin,
        }
        error: str | None = None
        if spec.kind not in _KINDS:
            error = "HOLD_UNKNOWN_ARTIFACT_KIND"
        elif not spec.role:
            error = "HOLD_INVALID_ROLE"
        elif spec.expected_digest is None or spec.expected_origin is None:
            error = _missing_binding_terminal(spec.kind, spec.expected_origin)
        elif spec.kind == "workspace_source" and spec.expected_origin != "workspace_source":
            error = "HOLD_SOURCE_HASH_UNBOUND"
        elif spec.kind != "workspace_source" and spec.expected_origin == "workspace_source":
            # A source hash/copy cannot attest bytes a later launch will execute.
            error = _missing_binding_terminal(spec.kind, spec.expected_origin)
        elif spec.kind in _BINARY_KINDS and spec.expected_origin != "installed_binary":
            error = "HOLD_BUILD_PROVENANCE_UNBOUND"
        elif (
            spec.kind not in _BINARY_KINDS
            and spec.kind != "workspace_source"
            and spec.expected_origin != "installed_bytes"
        ):
            error = "HOLD_SOURCE_HASH_UNBOUND"
        elif len(spec.expected_digest) != 64 or any(c not in "0123456789abcdef" for c in spec.expected_digest):
            error = "HOLD_INVALID_EXPECTED_DIGEST"
        canonical, resolve_error = _resolve_bounded(spec.path, spec.allowed_root)
        error = _more_severe(error, resolve_error)
        if canonical is not None:
            kind_error = _kind_error(canonical, spec.kind)
            error = _more_severe(error, kind_error)
            if canonical in canonical_roles:
                error = _more_severe(error, "HOLD_DUPLICATE_CANONICAL_ROLE")
                result["duplicate_of"] = canonical_roles[canonical]
            else:
                canonical_roles[canonical] = spec.role
                observed = hashlib.sha256(canonical.read_bytes()).hexdigest()
                result["canonical_path"] = str(canonical)
                result["observed_digest"] = observed
                if spec.expected_digest is not None and observed != spec.expected_digest:
                    error = _more_severe(error, "HOLD_DIGEST_MISMATCH")
        if error is None and spec.kind == "workspace_source":
            result["binding_scope"] = "workspace_source_only"
            result["terminal"] = "SOURCE_ONLY_MATCH"
            terminal = _more_severe(terminal, "HOLD_SOURCE_HASH_UNBOUND")
        else:
            result["terminal"] = error or "MATCH"
        terminal = _more_severe(terminal, error)
        results.append(result)
    return {"schema_version": 1, "read_only": True, "terminal": terminal or "READY_FOR_EXTERNAL_REVIEW_ONLY", "roles": results}


def inspect_host_installed_snapshot(specs: Iterable[InstalledRoleSpec]) -> dict[str, Any]:
    """Alias documenting that this inspection neither starts containers nor mutates host state."""
    return attest_installed_roles(specs)


def attest_stage2_installed_snapshot(specs: Iterable[InstalledRoleSpec]) -> dict[str, Any]:
    """Fail closed unless the future private launch's complete role set is bound."""
    spec_list = list(specs)
    result = inspect_host_installed_snapshot(spec_list)
    roles = [spec.role for spec in spec_list]
    if set(roles) != REQUIRED_STAGE2_ROLES or len(roles) != len(set(roles)):
        if result["terminal"] == "READY_FOR_EXTERNAL_REVIEW_ONLY":
            result["terminal"] = "HOLD_SOURCE_HASH_UNBOUND"
        result["required_roles"] = sorted(REQUIRED_STAGE2_ROLES)
        result["role_set_complete"] = False
    else:
        result["required_roles"] = sorted(REQUIRED_STAGE2_ROLES)
        result["role_set_complete"] = True
    return result


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description="read-only Stage-2 installed-byte preflight")
    parser.add_argument("--specs", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()
    try:
        raw = json.loads(args.specs.read_text(encoding="utf-8"))
        specs = [InstalledRoleSpec(
            role=item["role"], path=Path(item["path"]), allowed_root=Path(item["allowed_root"]),
            kind=item["kind"], expected_digest=item.get("expected_digest"),
            expected_origin=item.get("expected_origin"),
        ) for item in raw["roles"]]
    except (OSError, ValueError, KeyError, TypeError) as error:
        _write_json_atomic(args.result, {"schema_version": 1, "terminal": "HOLD_SOURCE_HASH_UNBOUND", "error": str(error)})
        return 1
    result = attest_stage2_installed_snapshot(specs)
    _write_json_atomic(args.result, result)
    return 0 if result["terminal"] == "READY_FOR_EXTERNAL_REVIEW_ONLY" else 1


if __name__ == "__main__":
    raise SystemExit(main())
