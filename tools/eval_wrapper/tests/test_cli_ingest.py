from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def test_cli_ingest_fixture(tmp_path: Path) -> None:
    package_root = Path(__file__).parents[1]
    repo_root = tmp_path / "repo"
    repo_root.mkdir()
    for name in ("create_submit_file.bash", "docker_build.sh", "Makefile"):
        (repo_root / name).write_text("# fixture\n", encoding="utf-8")
    (repo_root / "output" / "latest").mkdir(parents=True)
    fixture = package_root / "tests" / "fixtures" / "sample_output_latest"

    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "evalwrap",
            "--repo-root",
            str(repo_root),
            "ingest",
            "--label",
            "fixture",
            "--path",
            str(fixture),
        ],
        cwd=package_root,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    run_dirs = list((repo_root / "analysis" / "runs").glob("*_fixture"))
    assert len(run_dirs) == 1
    assert (run_dirs[0] / "manifest.yaml").exists()
    assert (run_dirs[0] / "processed" / "metrics.json").exists()
    assert (run_dirs[0] / "report" / "index.html").exists()
