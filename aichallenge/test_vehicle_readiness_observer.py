import json
import subprocess
import sys
from pathlib import Path


OBSERVER = Path(__file__).with_name("vehicle_readiness_observer.py")
TOKEN = "0123456789abcdef"
DOMAIN_ID = 3


def records(events: list[tuple[str, str]]) -> list[dict]:
    return [
        {
            "domain_id": DOMAIN_ID,
            "kind": kind,
            "monotonic_ns": index,
            "seq": index,
            "token": TOKEN,
            "value": value,
        }
        for index, (kind, value) in enumerate(events, 1)
    ]


def snapshot(
    tmp_path: Path,
    rows: list[dict],
    *,
    watermark: int | None = None,
    initialization_watermark: int | None = None,
) -> subprocess.CompletedProcess[str]:
    ready = tmp_path / "ready.json"
    events = tmp_path / "events.jsonl"
    ready.write_text(
        json.dumps(
            {
                "domain_id": DOMAIN_ID,
                "pid": 17,
                "started_monotonic_ns": 11,
                "status": "ready",
                "token": TOKEN,
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
        command += ["--history-watermark-seq", str(watermark)]
    if initialization_watermark is not None:
        command += [
            "--initialization-watermark-seq",
            str(initialization_watermark),
        ]
    return subprocess.run(command, text=True, capture_output=True, check=False)


def test_snapshot_preserves_ready_to_start_burst_after_watermark(
    tmp_path: Path,
) -> None:
    result = snapshot(
        tmp_path,
        records(
            [
                ("vehicle", "grounded"),
                ("initialization", "true"),
                ("vehicle", "ready"),
                ("vehicle", "start"),
            ]
        ),
        watermark=2,
        initialization_watermark=2,
    )
    assert result.returncode == 0
    assert result.stdout.strip() == "4 start 4 true 2 4 true false false"


def test_snapshot_accepts_start_then_ready_but_reports_grounded_or_finish(
    tmp_path: Path,
) -> None:
    start_ready = snapshot(
        tmp_path,
        records(
            [
                ("vehicle", "grounded"),
                ("initialization", "true"),
                ("vehicle", "start"),
                ("vehicle", "ready"),
            ]
        ),
        watermark=2,
        initialization_watermark=2,
    )
    assert start_ready.returncode == 0
    assert start_ready.stdout.strip().endswith("true false false")

    for terminal_state in ("grounded", "finish"):
        invalid = snapshot(
            tmp_path,
            records(
                [
                    ("vehicle", "ready"),
                    ("initialization", "true"),
                    ("vehicle", "start"),
                    ("vehicle", terminal_state),
                ]
            ),
            watermark=2,
            initialization_watermark=2,
        )
        assert invalid.returncode == 0
        assert invalid.stdout.strip().endswith("true true false")


def test_snapshot_accepts_pre_pulse_true_with_no_later_false(tmp_path: Path) -> None:
    result = snapshot(
        tmp_path,
        records(
            [
                ("vehicle", "ready"),
                ("initialization", "true"),
                ("vehicle", "start"),
            ]
        ),
        watermark=2,
        initialization_watermark=2,
    )
    assert result.returncode == 0
    assert result.stdout.strip().endswith("true false false")


def test_snapshot_reports_initialization_false_after_true_watermark(
    tmp_path: Path,
) -> None:
    result = snapshot(
        tmp_path,
        records(
            [
                ("vehicle", "ready"),
                ("initialization", "true"),
                ("initialization", "false"),
                ("vehicle", "start"),
            ]
        ),
        watermark=2,
        initialization_watermark=2,
    )
    assert result.returncode == 0
    assert result.stdout.strip().endswith("true false true")


def test_snapshot_rejects_missing_pre_pulse_initialization_watermark(
    tmp_path: Path,
) -> None:
    rows = records([("vehicle", "ready")])
    assert snapshot(tmp_path, rows, watermark=1).returncode != 0
    assert snapshot(
        tmp_path, rows, watermark=1, initialization_watermark=0
    ).returncode != 0


def test_snapshot_rejects_malformed_or_regressed_records(tmp_path: Path) -> None:
    malformed = records([("vehicle", "Ready ")])
    assert snapshot(tmp_path, malformed).returncode != 0

    regressed = records([("vehicle", "ready"), ("initialization", "true")])
    regressed[1]["seq"] = 1
    assert snapshot(tmp_path, regressed).returncode != 0

    cross_domain = records([("vehicle", "ready")])
    cross_domain[0]["domain_id"] = DOMAIN_ID + 1
    assert snapshot(tmp_path, cross_domain).returncode != 0


def test_snapshot_bounds_history_and_rejects_partial_tail(tmp_path: Path) -> None:
    bounded = records([("vehicle", "ready")] * 512)
    assert snapshot(tmp_path, bounded).returncode == 0
    assert snapshot(tmp_path, bounded + [bounded[-1]]).returncode != 0

    ready = tmp_path / "partial-ready.json"
    events = tmp_path / "partial-events.jsonl"
    ready.write_text(
        json.dumps(
            {
                "domain_id": DOMAIN_ID,
                "pid": 17,
                "started_monotonic_ns": 11,
                "status": "ready",
                "token": TOKEN,
            }
        ),
        encoding="utf-8",
    )
    events.write_text(json.dumps(records([("vehicle", "ready")])[0]), encoding="utf-8")
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
