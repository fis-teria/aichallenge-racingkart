from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
import traceback
import uuid
from pathlib import Path
from typing import Any

from .config import load_config
from .evaluator import Ros2Evaluator


def atomic_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + f".{uuid.uuid4().hex}.tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
    temporary.replace(path)


def controller_ready(timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "1"
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["ros2", "service", "list"],
            env=environment,
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        if "/simple_pure_pursuit_node/ga/set_enabled" in result.stdout:
            return True
        time.sleep(1.0)
    return False


def run_worker(worker_id: str, config_path: Path, ipc_root: Path) -> int:
    config = load_config(config_path)
    evaluator_config = config["evaluator"]
    root = ipc_root / worker_id
    requests = root / "requests"
    running = root / "running"
    results = root / "results"
    failed = root / "failed"
    for directory in (requests, running, results, failed):
        directory.mkdir(parents=True, exist_ok=True)

    print(f"worker={worker_id} waiting for Pure Pursuit", flush=True)
    if not controller_ready(float(evaluator_config.get("worker_ready_timeout_sec", 120.0))):
        raise RuntimeError("Pure Pursuit GA services did not become ready")
    print(f"worker={worker_id} ready", flush=True)

    poll_interval = float(evaluator_config.get("worker_poll_interval_sec", 0.1))
    episode_timeout = float(evaluator_config["episode_timeout_sec"])
    while True:
        request_paths = sorted(requests.glob("*.json"), key=lambda path: path.stat().st_mtime)
        if not request_paths:
            time.sleep(poll_interval)
            continue
        source = request_paths[0]
        claimed = running / source.name
        try:
            source.replace(claimed)
        except FileNotFoundError:
            continue
        request: dict[str, Any] = json.loads(claimed.read_text(encoding="utf-8"))
        job_id = str(request["job_id"])
        try:
            run_dir = Path(request["run_dir"])
            evaluator = Ros2Evaluator(evaluator_config, episode_timeout, run_dir)
            metrics = evaluator.evaluate(
                request["candidate_id"],
                request["parameter_hash"],
                request["parameters"],
                int(request["repeat"]),
            )
            atomic_json(results / f"{job_id}.json", metrics)
            print(
                f"worker={worker_id} completed candidate={request['candidate_id']} "
                f"reason={metrics.get('exit_reason')}",
                flush=True,
            )
        except Exception as error:  # Worker boundary must report every failure.
            atomic_json(
                failed / f"{job_id}.json",
                {"error": str(error), "traceback": traceback.format_exc()},
            )
            print(
                f"worker={worker_id} failed candidate={request.get('candidate_id')}: {error}",
                flush=True,
            )
        finally:
            claimed.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker-id", required=True)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--ipc-root", required=True, type=Path)
    args = parser.parse_args()
    raise SystemExit(run_worker(args.worker_id, args.config, args.ipc_root))


if __name__ == "__main__":
    main()
