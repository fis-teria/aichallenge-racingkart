import json
import subprocess
import sys
from pathlib import Path


OBSERVER = Path(__file__).with_name("race_arm_observer.py")
TOKEN = "0123456789abcdef"
DOMAIN_ID = 3


def records(states: list[str]) -> list[dict]:
    return [
        {
            "token": TOKEN,
            "domain_id": DOMAIN_ID,
            "seq": index,
            "monotonic_ns": index,
            "state": state,
        }
        for index, state in enumerate(states, 1)
    ]


def snapshot(
    tmp_path: Path,
    rows: list[dict],
    *,
    watermark: int | None = None,
    required_state: str | None = None,
    history_watermark: int | None = None,
    forbidden_state: str | None = None,
) -> subprocess.CompletedProcess[str]:
    ready = tmp_path / "ready.json"
    events = tmp_path / "events.jsonl"
    ready.write_text(
        json.dumps(
            {
                "token": TOKEN,
                "domain_id": DOMAIN_ID,
                "status": "ready",
                "pid": 17,
                "started_monotonic_ns": 11,
            }
        ),
        encoding="utf-8",
    )
    events.write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )
    command = [
        sys.executable,
        str(OBSERVER),
        "--snapshot",
        "--token",
        TOKEN,
        "--expected-pid",
        "17",
        "--expected-domain",
        str(DOMAIN_ID),
        "--min-started-monotonic-ns",
        "10",
        "--ready",
        str(ready),
        "--jsonl",
        str(events),
    ]
    if watermark is not None:
        command += ["--watermark-seq", str(watermark)]
    if required_state is not None:
        command += ["--require-state-after-watermark", required_state]
    if history_watermark is not None:
        command += ["--history-watermark-seq", str(history_watermark)]
    if forbidden_state is not None:
        command += ["--forbid-state-after-watermark", forbidden_state]
    return subprocess.run(command, text=True, capture_output=True, check=False)


def test_snapshot_accepts_exact_bool_sequence_and_reports_latest(tmp_path: Path) -> None:
    result = snapshot(tmp_path, records(["false", "false", "true"]))
    assert result.returncode == 0
    assert result.stdout.strip() == "3 true 3 false false"


def test_snapshot_reports_true_and_forbidden_history_even_when_latest_is_false(
    tmp_path: Path,
) -> None:
    result = snapshot(
        tmp_path,
        records(["false", "true", "false"]),
        history_watermark=1,
        forbidden_state="true",
    )
    assert result.returncode == 0
    assert result.stdout.strip() == "3 false 3 true true"


def test_snapshot_requires_new_true_after_false_watermark(tmp_path: Path) -> None:
    rows = records(["true", "false", "true"])
    assert snapshot(tmp_path, rows, watermark=2, required_state="true").returncode == 0
    assert snapshot(tmp_path, rows[:2], watermark=2, required_state="true").returncode != 0
    assert snapshot(tmp_path, rows[:2], watermark=1, required_state="true").returncode != 0


def test_snapshot_rejects_invalid_or_nonmonotonic_records(tmp_path: Path) -> None:
    invalid_state = records(["false"])
    invalid_state[0]["state"] = "False "
    assert snapshot(tmp_path, invalid_state).returncode != 0

    duplicate = records(["false", "true"])
    duplicate[1]["seq"] = 1
    assert snapshot(tmp_path, duplicate).returncode != 0

    reversed_time = records(["false", "true"])
    reversed_time[1]["monotonic_ns"] = 1
    assert snapshot(tmp_path, reversed_time).returncode != 0


def test_snapshot_bounds_record_count_and_rejects_partial_tail(tmp_path: Path) -> None:
    assert snapshot(tmp_path, records(["false"] * 256)).returncode == 0
    assert snapshot(tmp_path, records(["false"] * 257)).returncode != 0

    ready = tmp_path / "partial-ready.json"
    events = tmp_path / "partial-events.jsonl"
    ready.write_text(
        json.dumps(
            {
                "token": TOKEN,
                "domain_id": DOMAIN_ID,
                "status": "ready",
                "pid": 17,
                "started_monotonic_ns": 11,
            }
        ),
        encoding="utf-8",
    )
    events.write_text(json.dumps(records(["false"])[0]), encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            str(OBSERVER),
            "--snapshot",
            "--token",
            TOKEN,
            "--expected-pid",
            "17",
            "--expected-domain",
            str(DOMAIN_ID),
            "--ready",
            str(ready),
            "--jsonl",
            str(events),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0


def test_snapshot_rejects_lifecycle_identity_mismatch(tmp_path: Path) -> None:
    ready = tmp_path / "ready.json"
    events = tmp_path / "events.jsonl"
    ready.write_text(
        json.dumps(
            {
                "token": "fedcba9876543210",
                "domain_id": DOMAIN_ID,
                "status": "ready",
                "pid": 17,
                "started_monotonic_ns": 11,
            }
        ),
        encoding="utf-8",
    )
    events.write_text("", encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            str(OBSERVER),
            "--snapshot",
            "--token",
            TOKEN,
            "--expected-pid",
            "17",
            "--expected-domain",
            str(DOMAIN_ID),
            "--ready",
            str(ready),
            "--jsonl",
            str(events),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0


def test_snapshot_rejects_cross_domain_lifecycle_and_records(tmp_path: Path) -> None:
    rows = records(["false", "true"])
    rows[1]["domain_id"] = DOMAIN_ID + 1
    assert snapshot(tmp_path, rows).returncode != 0

    ready = tmp_path / "ready.json"
    events = tmp_path / "events.jsonl"
    ready.write_text(
        json.dumps(
            {
                "token": TOKEN,
                "domain_id": DOMAIN_ID + 1,
                "status": "ready",
                "pid": 17,
                "started_monotonic_ns": 11,
            }
        ),
        encoding="utf-8",
    )
    events.write_text("", encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable,
            str(OBSERVER),
            "--snapshot",
            "--token",
            TOKEN,
            "--expected-pid",
            "17",
            "--expected-domain",
            str(DOMAIN_ID),
            "--ready",
            str(ready),
            "--jsonl",
            str(events),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
