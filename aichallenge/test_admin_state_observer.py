import json
import subprocess
import sys
from pathlib import Path


OBSERVER = Path(__file__).with_name("admin_state_observer.py")
TOKEN = "0123456789abcdef"


def snapshot(tmp_path: Path, records: list[dict], *, watermark: int | None = None) -> subprocess.CompletedProcess[str]:
    ready = tmp_path / "ready.json"
    events = tmp_path / "events.jsonl"
    ready.write_text(json.dumps({"token": TOKEN, "status": "ready", "pid": 1, "started_monotonic_ns": 1}))
    events.write_text("".join(json.dumps(record) + "\n" for record in records))
    command = [sys.executable, str(OBSERVER), "--snapshot", "--token", TOKEN, "--ready", str(ready), "--jsonl", str(events)]
    if watermark is not None:
        command += ["--watermark-seq", str(watermark)]
    return subprocess.run(command, text=True, capture_output=True, check=False)


def records(states: list[str]) -> list[dict]:
    return [{"token": TOKEN, "seq": index, "monotonic_ns": index, "state": state} for index, state in enumerate(states, 1)]


def test_snapshot_accepts_256_records_and_rejects_257(tmp_path: Path) -> None:
    assert snapshot(tmp_path, records(["waitstart"] * 256)).returncode == 0
    assert snapshot(tmp_path, records(["waitstart"] * 257)).returncode != 0


def test_snapshot_transition_rejects_terminal_between_watermark_and_start(tmp_path: Path) -> None:
    assert snapshot(tmp_path, records(["waitstart", "waitstart", "start"]), watermark=1).returncode == 0
    assert snapshot(tmp_path, records(["waitstart", "finish", "start"]), watermark=1).returncode != 0
    assert snapshot(tmp_path, records(["waitstart", "start", "ready"]), watermark=1).returncode != 0
