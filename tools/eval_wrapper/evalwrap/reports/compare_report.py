from __future__ import annotations

import html
import json
from pathlib import Path


def generate_compare_report(analysis_dir: Path, base_run: str, target_run: str) -> Path:
    base_metrics = _load_metrics(analysis_dir, base_run)
    target_metrics = _load_metrics(analysis_dir, target_run)
    base_overtake = _load_overtake_metrics(analysis_dir, base_run)
    target_overtake = _load_overtake_metrics(analysis_dir, target_run)
    report_dir = analysis_dir / "comparisons" / f"{base_run}_vs_{target_run}"
    report_dir.mkdir(parents=True, exist_ok=True)
    index = report_dir / "index.html"
    rows = []
    for domain_id, target in target_metrics.get("domains", {}).items():
        base = base_metrics.get("domains", {}).get(domain_id, {})
        for metric in (
            "total_time_sec",
            "best_lap_sec",
            "avg_lap_sec",
            "lap_count",
            "penalty_count",
            "collision_count",
            "stuck_count",
            "low_speed_time_sec",
            "avg_speed_mps",
            "max_speed_mps",
            "max_abs_steer_rad",
            "steer_oscillation_score",
            "max_accel_mps2",
            "max_decel_mps2",
            "max_command_accel_mps2",
            "max_command_decel_mps2",
            "max_command_abs_steer_rad",
            "avg_path_error_m",
            "max_path_error_m",
            ):
            rows.append((domain_id, metric, base.get(metric), target.get(metric), _delta(base.get(metric), target.get(metric))))
    overtake_path = _write_overtake_compare(report_dir, base_run, target_run, base_overtake, target_overtake)
    index.write_text(_html(base_run, target_run, rows, overtake_path), encoding="utf-8")
    return index


def _load_metrics(analysis_dir: Path, run_id: str) -> dict:
    path = analysis_dir / "runs" / run_id / "processed" / "metrics.json"
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_overtake_metrics(analysis_dir: Path, run_id: str) -> dict:
    path = analysis_dir / "runs" / run_id / "processed" / "overtake_metrics.json"
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _delta(base, target):
    if isinstance(base, (int, float)) and isinstance(target, (int, float)):
        return target - base
    return ""


def _html(base_run: str, target_run: str, rows: list[tuple[str, str, object, object, object]], overtake_path: Path | None) -> str:
    body = [
        "<!doctype html><html><head><meta charset='utf-8'><title>evalwrap compare</title>",
        "<style>body{font-family:system-ui,sans-serif;margin:32px}table{border-collapse:collapse;width:100%}th,td{border-bottom:1px solid #ddd;padding:8px;text-align:left}.good{color:#047857}.bad{color:#b91c1c}</style>",
        "</head><body>",
        f"<h1>{html.escape(base_run)} vs {html.escape(target_run)}</h1>",
        "<p><a href='overtake_compare.html'>Overtake compare report</a></p>" if overtake_path else "",
        "<table><thead><tr><th>domain</th><th>metric</th><th>base</th><th>target</th><th>delta</th></tr></thead><tbody>",
    ]
    for domain_id, metric, base, target, delta in rows:
        cls = ""
        lower_is_better = {
            "total_time_sec",
            "best_lap_sec",
            "avg_lap_sec",
            "penalty_count",
            "collision_count",
            "stuck_count",
            "low_speed_time_sec",
            "max_abs_steer_rad",
            "steer_oscillation_score",
            "max_decel_mps2",
            "max_command_decel_mps2",
            "max_command_abs_steer_rad",
            "avg_path_error_m",
            "max_path_error_m",
        }
        higher_is_better = {"avg_speed_mps", "max_speed_mps", "max_accel_mps2", "max_command_accel_mps2"}
        if isinstance(delta, (int, float)) and metric in lower_is_better:
            cls = "good" if delta < 0 else "bad" if delta > 0 else ""
        elif isinstance(delta, (int, float)) and metric in higher_is_better:
            cls = "good" if delta > 0 else "bad" if delta < 0 else ""
        body.append(
            f"<tr><td>{html.escape(domain_id)}</td><td>{html.escape(metric)}</td><td>{base}</td><td>{target}</td><td class='{cls}'>{delta}</td></tr>"
        )
    body.append("</tbody></table></body></html>")
    return "\n".join(body)


def _write_overtake_compare(
    report_dir: Path,
    base_run: str,
    target_run: str,
    base_overtake: dict,
    target_overtake: dict,
) -> Path | None:
    if not base_overtake and not target_overtake:
        return None
    path = report_dir / "overtake_compare.html"
    rows = []
    target_domains = target_overtake.get("domains", {}) if target_overtake else {}
    base_domains = base_overtake.get("domains", {}) if base_overtake else {}
    metrics = [
        "attempt_count",
        "success_rate",
        "failure_rate",
        "blocked_time_sec",
        "missed_overtake_chance_count",
        "collision_count",
        "penalty_count",
        "min_vehicle_distance_m",
        "min_cbf_h",
        "max_cbf_slack",
        "mpc_infeasible_count",
    ]
    for domain_id in sorted(set(base_domains) | set(target_domains)):
        base = base_domains.get(domain_id, {})
        target = target_domains.get(domain_id, {})
        for metric in metrics:
            rows.append((domain_id, metric, base.get(metric), target.get(metric), _delta(base.get(metric), target.get(metric))))
    path.write_text(_overtake_html(base_run, target_run, rows), encoding="utf-8")
    return path


def _overtake_html(base_run: str, target_run: str, rows: list[tuple[str, str, object, object, object]]) -> str:
    body = [
        "<!doctype html><html><head><meta charset='utf-8'><title>overtake compare</title>",
        "<style>body{font-family:system-ui,sans-serif;margin:32px}table{border-collapse:collapse;width:100%}th,td{border-bottom:1px solid #ddd;padding:8px;text-align:left}.good{color:#047857}.bad{color:#b91c1c}</style>",
        "</head><body>",
        f"<h1>Overtake Compare: {html.escape(base_run)} vs {html.escape(target_run)}</h1>",
        "<table><thead><tr><th>domain</th><th>metric</th><th>base</th><th>target</th><th>delta</th></tr></thead><tbody>",
    ]
    lower_is_better = {
        "failure_rate",
        "blocked_time_sec",
        "missed_overtake_chance_count",
        "collision_count",
        "penalty_count",
        "max_cbf_slack",
        "mpc_infeasible_count",
    }
    higher_is_better = {"attempt_count", "success_rate", "min_vehicle_distance_m", "min_cbf_h"}
    for domain_id, metric, base, target, delta in rows:
        cls = ""
        if isinstance(delta, (int, float)) and metric in lower_is_better:
            cls = "good" if delta < 0 else "bad" if delta > 0 else ""
        elif isinstance(delta, (int, float)) and metric in higher_is_better:
            cls = "good" if delta > 0 else "bad" if delta < 0 else ""
        body.append(
            f"<tr><td>{html.escape(domain_id)}</td><td>{html.escape(metric)}</td><td>{base}</td><td>{target}</td><td class='{cls}'>{delta}</td></tr>"
        )
    body.append("</tbody></table></body></html>")
    return "\n".join(body)
