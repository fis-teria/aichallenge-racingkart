from __future__ import annotations

import subprocess
from pathlib import Path

from evalwrap.utils import rosbag_utils


def test_rosbag_storage_files_exist_accepts_db3(tmp_path: Path) -> None:
    bag_dir = tmp_path / "rosbag2_autoware"
    bag_dir.mkdir()
    (bag_dir / "rosbag2_autoware_0.db3").write_bytes(b"sqlite")

    assert rosbag_utils.rosbag_storage_files_exist(bag_dir)


def test_ensure_rosbag_metadata_reports_missing_ros2(tmp_path: Path, monkeypatch) -> None:
    bag_dir = tmp_path / "rosbag2_autoware"
    bag_dir.mkdir()
    (bag_dir / "rosbag2_autoware_0.mcap").write_bytes(b"mcap")
    monkeypatch.setattr(rosbag_utils.shutil, "which", lambda name: None)

    ready, reason = rosbag_utils.ensure_rosbag_metadata(bag_dir)

    assert not ready
    assert reason == "metadata.yaml missing; ros2 not found for reindex"


def test_ensure_rosbag_metadata_reports_reindex_failure(tmp_path: Path, monkeypatch) -> None:
    bag_dir = tmp_path / "rosbag2_autoware"
    bag_dir.mkdir()
    (bag_dir / "rosbag2_autoware_0.mcap").write_bytes(b"mcap")
    monkeypatch.setattr(rosbag_utils.shutil, "which", lambda name: "/usr/bin/ros2")

    def fake_run(*args, **kwargs):
        return subprocess.CompletedProcess(args[0], 1, stdout="", stderr="bad bag")

    monkeypatch.setattr(rosbag_utils.subprocess, "run", fake_run)

    ready, reason = rosbag_utils.ensure_rosbag_metadata(bag_dir)

    assert not ready
    assert reason == "ros2 bag reindex failed: bad bag"


def test_ensure_rosbag_metadata_reports_timeout(tmp_path: Path, monkeypatch) -> None:
    bag_dir = tmp_path / "rosbag2_autoware"
    bag_dir.mkdir()
    (bag_dir / "rosbag2_autoware_0.mcap").write_bytes(b"mcap")
    monkeypatch.setattr(rosbag_utils.shutil, "which", lambda name: "/usr/bin/ros2")

    def fake_run(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], timeout=1.0)

    monkeypatch.setattr(rosbag_utils.subprocess, "run", fake_run)

    ready, reason = rosbag_utils.ensure_rosbag_metadata(bag_dir, timeout_sec=1.0)

    assert not ready
    assert reason is not None
    assert reason.startswith("failed to reindex:")
