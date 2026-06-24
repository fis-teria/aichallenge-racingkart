from __future__ import annotations

from pathlib import Path

from evalwrap.parsers.summary_parser import extract_lap_times, parse_summary


def test_summary_parser_extracts_common_fields() -> None:
    fixture = Path(__file__).parent / "fixtures" / "sample_output_latest" / "d1" / "result-summary.json"
    parsed = parse_summary(fixture)

    assert parsed.finish is True
    assert parsed.total_time_sec == 421.35
    assert parsed.lap_count == 6
    assert parsed.best_lap_sec == 68.9
    assert parsed.penalty_count == 0
    assert parsed.collision_count == 0


def test_summary_parser_deduplicates_schema_v2_lap_lists() -> None:
    data = {
        "vehicles": [
            {
                "finished": True,
                "laps": [52.7, 52.6, 51.8],
            }
        ],
        "laps": [52.7, 52.6, 51.8],
    }

    assert extract_lap_times(data) == [52.7, 52.6, 51.8]
