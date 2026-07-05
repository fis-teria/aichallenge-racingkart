from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


def rosbag_storage_files_exist(bag_dir: Path) -> bool:
    return bag_dir.is_dir() and any(path.suffix in {".mcap", ".db3"} for path in bag_dir.iterdir())


def ensure_rosbag_metadata(bag_dir: Path, timeout_sec: float = 120.0) -> tuple[bool, str | None]:
    if not rosbag_storage_files_exist(bag_dir):
        return True, None
    if (bag_dir / "metadata.yaml").exists():
        return True, None

    ros2 = shutil.which("ros2")
    if ros2 is None:
        return False, "metadata.yaml missing; ros2 not found for reindex"

    try:
        completed = subprocess.run(
            [ros2, "bag", "reindex", str(bag_dir)],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except Exception as exc:  # noqa: BLE001 - repair should not crash reporting
        return False, f"failed to reindex: {exc}"

    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        suffix = f": {detail}" if detail else ""
        return False, f"ros2 bag reindex failed{suffix}"
    if not (bag_dir / "metadata.yaml").exists():
        return False, "ros2 bag reindex finished but metadata.yaml is still missing"
    return True, None
