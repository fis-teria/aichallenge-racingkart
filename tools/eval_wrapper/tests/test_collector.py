from __future__ import annotations

from pathlib import Path
from shutil import copytree

from evalwrap.collector import collect_output
from evalwrap.utils import rosbag_utils


def test_collector_copies_existing_domain(tmp_path: Path) -> None:
    fixture = Path(__file__).parent / "fixtures" / "sample_output_latest"
    run_dir = tmp_path / "run"

    result = collect_output(fixture, run_dir, [1, 2, 3, 4])

    assert result.domains == ["d1"]
    assert (run_dir / "raw" / "d1" / "result-summary.json").exists()
    assert not result.warnings


def test_collector_resolves_empty_output_latest_to_latest_timestamped_run(tmp_path: Path) -> None:
    fixture = Path(__file__).parent / "fixtures" / "sample_output_latest"
    output_root = tmp_path / "output"
    (output_root / "latest").mkdir(parents=True)
    copytree(fixture, output_root / "20260612-155040")
    run_dir = tmp_path / "analysis" / "run"

    result = collect_output(output_root / "latest", run_dir, [1, 2, 3, 4])

    assert result.domains == ["d1"]
    assert (run_dir / "raw" / "d1" / "result-summary.json").exists()
    assert not result.warnings


def test_collector_reindexes_mcap_bag_missing_metadata(tmp_path: Path, monkeypatch) -> None:
    output_run = tmp_path / "output" / "20260704-212818"
    bag_dir = output_run / "d1" / "rosbag2_autoware"
    bag_dir.mkdir(parents=True)
    (bag_dir / "rosbag2_autoware_0.mcap").write_bytes(b"mcap")
    run_dir = tmp_path / "analysis" / "run"
    calls: list[Path] = []

    def fake_ensure_rosbag_metadata(bag_path: Path, timeout_sec: float = 120.0):
        calls.append(bag_path)
        (bag_path / "metadata.yaml").write_text(
            "rosbag2_bagfile_information: {}\n", encoding="utf-8"
        )
        return True, None

    monkeypatch.setattr(rosbag_utils, "ensure_rosbag_metadata", fake_ensure_rosbag_metadata)

    result = collect_output(output_run, run_dir, [1])

    copied_bag = run_dir / "raw" / "d1" / "rosbag2_autoware"
    assert result.domains == ["d1"]
    assert (copied_bag / "metadata.yaml").exists()
    assert calls == [copied_bag]
    assert not result.warnings
