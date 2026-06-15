from __future__ import annotations

import json

from evalwrap.parsers.details_parser import parse_details


def test_details_parser_uses_explicit_penalty_count(tmp_path) -> None:
    details = {
        "finished": True,
        "laps": [52.7, 51.8],
        "penalty_count": 13,
        "penalty_total_seconds": 207.94,
        "penalty_events": [{"kind": "wall"}, {"kind": "wall"}],
        "penalty_by_kind": {"wall": {"count": 13, "total_seconds": 207.96}},
    }
    path = tmp_path / "d1-result-details.json"
    path.write_text(json.dumps(details), encoding="utf-8")

    parsed = parse_details(path)

    assert parsed.lap_times == [52.7, 51.8]
    assert parsed.penalty_count == 13
    assert parsed.collision_count is None
