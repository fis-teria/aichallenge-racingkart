#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def environments(count: int, stride: int = 10) -> list[dict[str, object]]:
    if count < 1:
        raise ValueError("environment count must be positive")
    return [
        {
            "name": f"env{index}",
            "admin_domain_id": (index - 1) * stride,
            "vehicle_domain_ids": [
                (index - 1) * stride + vehicle for vehicle in range(1, 5)
            ],
        }
        for index in range(1, count + 1)
    ]


def write_evaluator_config(source: Path, target: Path, count: int) -> None:
    data = json.loads(source.read_text(encoding="utf-8"))
    pool = environments(count)
    data["run"]["parallel_workers"] = count * 4
    data["evaluator"].pop("vehicle_domain_ids", None)
    data["evaluator"]["environments"] = pool
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def write_race_config(source: Path, target: Path, base_domain: int, name: str) -> None:
    lines = source.read_text(encoding="utf-8").splitlines()
    output = []
    for line in lines:
        if line.startswith("name:"):
            output.append(f"name: {name}")
        elif line.startswith("ros2-base-domain:"):
            output.append(f"ros2-base-domain: {base_domain}")
        else:
            output.append(line)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text("\n".join(output) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-count", type=int, required=True)
    parser.add_argument("--config-source", type=Path, required=True)
    parser.add_argument("--config-output", type=Path, required=True)
    parser.add_argument("--race-template", type=Path, required=True)
    parser.add_argument("--race-config-dir", type=Path, required=True)
    args = parser.parse_args()
    write_evaluator_config(
        args.config_source, args.config_output, args.env_count
    )
    for environment in environments(args.env_count):
        name = f"ga-pool-{environment['name']}"
        write_race_config(
            args.race_template,
            args.race_config_dir / f"{name}.yaml",
            int(environment["vehicle_domain_ids"][0]),
            name,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
