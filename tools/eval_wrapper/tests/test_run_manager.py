from __future__ import annotations

from datetime import datetime, timezone

from evalwrap.run_manager import make_run_id


def test_make_run_id_slugifies_label() -> None:
    run_id = make_run_id("curve speed v1", datetime(2026, 6, 12, 0, 0, 1, tzinfo=timezone.utc))
    assert run_id == "20260612_000001_curve_speed_v1"
