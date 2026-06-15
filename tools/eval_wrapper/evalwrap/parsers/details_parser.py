from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .summary_parser import extract_lap_times, load_json


@dataclass
class ParsedDetails:
    raw_json: dict[str, Any] = field(default_factory=dict)
    lap_times: list[float] = field(default_factory=list)
    penalty_count: int | None = None
    collision_count: int | None = None
    warnings: list[str] = field(default_factory=list)


def parse_details(path: Path) -> ParsedDetails:
    data, warnings = load_json(path)
    parsed = ParsedDetails(raw_json=data, warnings=warnings)
    if not data:
        return parsed
    parsed.lap_times = extract_lap_times(data)
    parsed.penalty_count = _first_exact_int(data, {"penalty_count", "penaltycount", "penalties"})
    if parsed.penalty_count is None:
        parsed.penalty_count = _event_list_count(data, {"penalty_events", "penaltyevents"})
    parsed.collision_count = _first_exact_int(data, {"collision_count", "collisioncount", "crash_count", "crashcount"})
    if parsed.collision_count is None:
        parsed.collision_count = _event_list_count(data, {"collision_events", "collisionevents", "crash_events", "crashevents"})
    return parsed


def _first_exact_int(data: Any, keys: set[str]) -> int | None:
    if isinstance(data, dict):
        for key, value in data.items():
            normalized = _norm(str(key))
            if normalized in keys and isinstance(value, (int, float)) and not isinstance(value, bool):
                return int(value)
            found = _first_exact_int(value, keys)
            if found is not None:
                return found
    elif isinstance(data, list):
        for item in data:
            found = _first_exact_int(item, keys)
            if found is not None:
                return found
    return None


def _event_list_count(data: Any, keys: set[str]) -> int | None:
    if isinstance(data, dict):
        for key, value in data.items():
            normalized = _norm(str(key))
            if normalized in keys and isinstance(value, list):
                return len(value)
            found = _event_list_count(value, keys)
            if found is not None:
                return found
    elif isinstance(data, list):
        for item in data:
            found = _event_list_count(item, keys)
            if found is not None:
                return found
    return None


def _norm(key: str) -> str:
    return "".join(char for char in key.lower() if char.isalnum())
