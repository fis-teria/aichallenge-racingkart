from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class ParsedSummary:
    raw_json: dict[str, Any] = field(default_factory=dict)
    finish: bool | None = None
    total_time_sec: float | None = None
    lap_count: int | None = None
    best_lap_sec: float | None = None
    avg_lap_sec: float | None = None
    penalty_count: int | None = None
    collision_count: int | None = None
    warnings: list[str] = field(default_factory=list)


def load_json(path: Path) -> tuple[dict[str, Any], list[str]]:
    if not path.exists():
        return {}, [f"missing json: {path.name}"]
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except Exception as exc:  # noqa: BLE001 - keep partial runs alive
        return {}, [f"failed to parse {path.name}: {exc}"]
    if not isinstance(data, dict):
        return {}, [f"json root is not an object: {path.name}"]
    return data, []


def parse_summary(path: Path) -> ParsedSummary:
    data, warnings = load_json(path)
    parsed = ParsedSummary(raw_json=data, warnings=warnings)
    if not data:
        return parsed

    parsed.finish = _find_bool(data, {"finish", "finished", "completed", "success"})
    if parsed.finish is None:
        status = _find_string(data, {"status", "state", "result"})
        if status:
            lowered = status.lower()
            if any(token in lowered for token in ("finish", "success", "complete")):
                parsed.finish = True
            elif any(token in lowered for token in ("fail", "timeout", "abort")):
                parsed.finish = False

    parsed.total_time_sec = _find_number(data, ("totaltime", "totalelapsed", "elapsedtime", "racetime"))
    parsed.penalty_count = _find_int(data, ("penaltycount", "penalties", "violationcount", "violations"))
    parsed.collision_count = _find_int(data, ("collisioncount", "collisions", "crashcount", "crashes"))

    lap_times = extract_lap_times(data)
    if lap_times:
        parsed.lap_count = len(lap_times)
        parsed.best_lap_sec = min(lap_times)
        parsed.avg_lap_sec = sum(lap_times) / len(lap_times)
    else:
        parsed.lap_count = _find_int(data, ("lapcount", "lapscompleted", "lapnum", "laps"))
        parsed.best_lap_sec = _find_number(data, ("bestlap", "bestlaptime"))
        parsed.avg_lap_sec = _find_number(data, ("avglap", "averagelap", "avglaptime"))

    return parsed


def extract_lap_times(data: Any) -> list[float]:
    candidates: list[list[float]] = []

    def walk(node: Any, parent_key: str = "") -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                norm = _norm(key)
                if "lap" in norm and isinstance(value, list):
                    lap_times = _lap_times_from_list(value)
                    if lap_times:
                        candidates.append(lap_times)
                        continue
                walk(value, norm)
        elif isinstance(node, list):
            if "lap" in parent_key:
                lap_times = _lap_times_from_list(node)
                if lap_times:
                    candidates.append(lap_times)
                    return
            for item in node:
                walk(item, parent_key)

    walk(data)
    unique_candidates: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    for candidate in candidates:
        signature = tuple(candidate)
        if signature in seen:
            continue
        seen.add(signature)
        unique_candidates.append(candidate)
    if not unique_candidates:
        return []
    return max(unique_candidates, key=len)


def _lap_times_from_list(items: list[Any]) -> list[float]:
    values: list[float] = []
    for item in items:
        lap_time = _lap_time_from_item(item)
        if lap_time is not None:
            values.append(lap_time)
    return values


def _lap_time_from_item(item: Any) -> float | None:
    if isinstance(item, (int, float)):
        return float(item)
    if not isinstance(item, dict):
        return None
    for key, value in item.items():
        norm = _norm(key)
        if ("lap" in norm and "time" in norm) or norm in {"time", "duration", "durationsec"}:
            if isinstance(value, (int, float)):
                return float(value)
    return None


def _norm(key: str) -> str:
    return re.sub(r"[^a-z0-9]", "", key.lower())


def _walk(data: Any):
    if isinstance(data, dict):
        for key, value in data.items():
            yield _norm(str(key)), value
            yield from _walk(value)
    elif isinstance(data, list):
        for item in data:
            yield from _walk(item)


def _find_number(data: Any, candidates: tuple[str, ...]) -> float | None:
    for key, value in _walk(data):
        if any(candidate in key for candidate in candidates) and isinstance(value, (int, float)):
            return float(value)
    return None


def _find_int(data: Any, candidates: tuple[str, ...]) -> int | None:
    number = _find_number(data, candidates)
    return int(number) if number is not None else None


def _find_bool(data: Any, candidates: set[str]) -> bool | None:
    for key, value in _walk(data):
        if key in candidates and isinstance(value, bool):
            return value
    return None


def _find_string(data: Any, candidates: set[str]) -> str | None:
    for key, value in _walk(data):
        if key in candidates and isinstance(value, str):
            return value
    return None
