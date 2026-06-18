from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

from evalwrap.run_manager import _wait_for_single_eval_output, make_run_id


def test_make_run_id_slugifies_label() -> None:
    run_id = make_run_id("curve speed v1", datetime(2026, 6, 12, 0, 0, 1, tzinfo=timezone.utc))
    assert run_id == "20260612_000001_curve_speed_v1"


def test_wait_for_single_eval_output_rejects_unfinalized_rosbag(tmp_path: Path) -> None:
    domain = tmp_path / "output" / "20260618-232702" / "d1"
    bag_dir = domain / "rosbag2_autoware"
    bag_dir.mkdir(parents=True)
    (domain / "result-summary.json").write_text("{}", encoding="utf-8")
    (domain / "d1-result-details.json").write_text("{}", encoding="utf-8")
    (bag_dir / "rosbag2_autoware_0.mcap").write_bytes(b"not-finalized")

    source, warnings = _wait_for_single_eval_output(
        tmp_path,
        [1],
        datetime(1970, 1, 1, tzinfo=timezone.utc),
        timeout_sec=0,
        poll_sec=0,
    )

    assert source == tmp_path / "output" / "20260618-232702"
    assert warnings == ["timed out waiting for eval output to finish: d1: rosbag2_autoware metadata.yaml not ready"]


def test_wait_for_single_eval_output_accepts_finalized_rosbag(tmp_path: Path) -> None:
    domain = tmp_path / "output" / "20260618-232702" / "d1"
    bag_dir = domain / "rosbag2_autoware"
    bag_dir.mkdir(parents=True)
    (domain / "result-summary.json").write_text("{}", encoding="utf-8")
    (domain / "d1-result-details.json").write_text("{}", encoding="utf-8")
    (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information: {}", encoding="utf-8")
    (bag_dir / "rosbag2_autoware_0.mcap").write_bytes(b"finalized")

    source, warnings = _wait_for_single_eval_output(
        tmp_path,
        [1],
        datetime(1970, 1, 1, tzinfo=timezone.utc),
        timeout_sec=0,
        poll_sec=0,
    )

    assert source == tmp_path / "output" / "20260618-232702"
    assert warnings == []
