from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any

from . import SCHEMA_VERSION


OUTCOMES = {
    "PASS",
    "ASSERTION_FAILED",
    "PRECONDITION_NOT_MET",
    "INFRASTRUCTURE_FAILED",
    "INVALID_EVIDENCE",
}


def write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    """Write a result without exposing a partially-written JSON file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"schema_version": SCHEMA_VERSION, **payload}
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        delete=False,
    ) as stream:
        temporary = Path(stream.name)
        json.dump(payload, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def read_result(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("outcome") not in OUTCOMES:
        raise ValueError(f"unknown result outcome: {payload.get('outcome')!r}")
    return payload
