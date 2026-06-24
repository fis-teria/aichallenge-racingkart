from __future__ import annotations

from dataclasses import dataclass, asdict

from evalwrap.parsers.log_parser import LogExcerpt


@dataclass
class Event:
    run_id: str
    domain_id: str
    time_sec: float | None
    lap: int | None
    section: int | None
    event_type: str
    severity: str
    description: str

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def events_from_counts(run_id: str, domain_id: str, penalty_count: int, collision_count: int) -> list[Event]:
    events: list[Event] = []
    if penalty_count > 0:
        events.append(
            Event(run_id, domain_id, None, None, None, "penalty", "confirmed", f"penalty count: {penalty_count}")
        )
    if collision_count > 0:
        events.append(
            Event(run_id, domain_id, None, None, None, "collision", "confirmed", f"collision count: {collision_count}")
        )
    return events


def events_from_log_excerpts(run_id: str, domain_id: str, excerpts: list[LogExcerpt]) -> list[Event]:
    events: list[Event] = []
    for excerpt in excerpts:
        text = excerpt.text
        lowered = text.lower()
        event_type = "node_error" if "error" in lowered or "exception" in lowered or "died" in lowered else "log_warning"
        severity = "error" if event_type == "node_error" else "warn"
        events.append(
            Event(
                run_id=run_id,
                domain_id=domain_id,
                time_sec=None,
                lap=None,
                section=None,
                event_type=event_type,
                severity=severity,
                description=f"{excerpt.path}:{excerpt.line_no}: {text}",
            )
        )
    return events


def events_from_rosbag(run_id: str, domain_id: str, items: list[dict[str, object]]) -> list[Event]:
    events: list[Event] = []
    for item in items:
        events.append(
            Event(
                run_id=run_id,
                domain_id=domain_id,
                time_sec=_optional_float(item.get("time_sec")),
                lap=_optional_int(item.get("lap")),
                section=_optional_int(item.get("section")),
                event_type=str(item.get("event_type") or "rosbag_event"),
                severity=str(item.get("severity") or "info"),
                description=str(item.get("description") or ""),
            )
        )
    return events


def _optional_float(value: object) -> float | None:
    return float(value) if isinstance(value, (int, float)) else None


def _optional_int(value: object) -> int | None:
    return int(value) if isinstance(value, (int, float)) else None
