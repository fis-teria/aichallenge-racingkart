from __future__ import annotations

from pathlib import Path
from shutil import copytree

from evalwrap.collector import collect_output


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
