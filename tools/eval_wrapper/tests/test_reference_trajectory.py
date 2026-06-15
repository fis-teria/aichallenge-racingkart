from __future__ import annotations

from pathlib import Path

from evalwrap.reference_trajectory import load_reference_trajectory


def test_load_reference_trajectory_resolves_mpc_csv_and_drops_duplicate_endpoint(tmp_path: Path) -> None:
    repo_root = tmp_path
    package_root = repo_root / "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros"
    config_dir = package_root / "config"
    csv_dir = package_root / "env/final_ver3"
    config_dir.mkdir(parents=True)
    csv_dir.mkdir(parents=True)

    (config_dir / "config.yaml").write_text(
        "\n".join(
            [
                "reference_path:",
                "  csv_path: env/final_ver3/ref.csv",
                "  circular: true",
                "  update_by_topic: false",
            ]
        ),
        encoding="utf-8",
    )
    (csv_dir / "ref.csv").write_text(
        "\n".join(
            [
                "s_m,x_m,y_m,kappa_radpm",
                "0.0,0.0,0.0,9999.0",
                "1.0,1.0,0.0,0.0",
                "2.0,1.0,1.0,0.0",
                "3.0,0.0,0.0,-9999.0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    reference = load_reference_trajectory(repo_root, {"enabled": True, "source": "mpc_config"})

    assert reference is not None
    assert reference.source == "mpc_csv"
    assert reference.csv_path == csv_dir / "ref.csv"
    assert reference.circular
    assert reference.points == [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
