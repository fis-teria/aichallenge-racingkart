from __future__ import annotations

import json
from pathlib import Path

from .storage import Storage


def export_best(run_dir: Path, output: Path) -> dict:
    storage = Storage(run_dir / "run.sqlite3")
    best = storage.best(1)[0]
    storage.close()
    lines = [
        f"# GA run: {run_dir.name}",
        f"# candidate: {best['candidate_id']}",
        f"# parameter hash: {best['parameter_hash']}",
        "/**:",
        "  ros__parameters:",
        "    ga_experiment_mode: false",
        "    use_external_target_vel: true",
        "    external_target_vel: 9.722222222222  # 35 km/h",
    ]
    for name, value in sorted(best["parameters"].items()):
        lines.append(f"    {name}: {float(value):.12g}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    report = output.with_suffix(".json")
    report.write_text(json.dumps(best, indent=2, sort_keys=True), encoding="utf-8")
    return best
