from __future__ import annotations

import csv
import html
from pathlib import Path
from typing import Any


def generate_run_report(run_dir: Path, manifest: dict[str, Any], metrics: dict[str, Any]) -> Path:
    report_dir = run_dir / "report"
    report_dir.mkdir(parents=True, exist_ok=True)
    index = report_dir / "index.html"
    domains = metrics.get("domains", {})
    log_excerpts = metrics.get("log_excerpts", {})

    parts = [
        "<!doctype html>",
        "<html><head><meta charset='utf-8'>",
        "<title>evalwrap report</title>",
        _style(),
        "</head><body>",
        f"<h1>{_e(manifest.get('run_id', 'run'))}</h1>",
        "<section><h2>Run Overview</h2>",
        _kv_table(
            {
                "label": manifest.get("label"),
                "status": manifest.get("status"),
                "created_at": manifest.get("created_at"),
                "git_branch": manifest.get("repo", {}).get("branch"),
                "git_commit": manifest.get("repo", {}).get("commit"),
                "dirty": manifest.get("repo", {}).get("dirty"),
                "diff_hash": manifest.get("repo", {}).get("diff_hash"),
                "submit_sha256": manifest.get("submission", {}).get("tar_sha256"),
                "eval_mode": manifest.get("eval", {}).get("mode"),
            }
        ),
        "</section>",
        "<section><h2>Result Summary</h2>",
        _domain_table(domains),
        "</section>",
        "<section><h2>Section Splits</h2>",
        _split_table(run_dir),
        "</section>",
        "<section><h2>Target Speed Profile</h2>",
        _target_speed_profile_section(run_dir),
        "</section>",
        "<section><h2>Grade & Acceleration Profile</h2>",
        _grade_profile_section(run_dir),
        "</section>",
        "<section><h2>Artifacts</h2>",
        _artifact_links(run_dir, domains.keys()),
        "</section>",
        "<section><h2>Processed Files</h2>",
        "<ul>",
        _link("../processed/metrics.json", "metrics.json"),
        _link("../processed/lap_summary.csv", "lap_summary.csv"),
        _link("../processed/events.csv", "events.csv"),
        _link("../processed/section_summary.csv", "section_summary.csv"),
        _link("../processed/awsim_section_summary.csv", "awsim_section_summary.csv"),
        _link("../processed/corner_summary.csv", "corner_summary.csv"),
        _link("../processed/trajectory_reference.csv", "trajectory_reference.csv"),
        _link("../processed/vehicle_timeseries.csv", "vehicle_timeseries.csv"),
        _link("../processed/control_timeseries.csv", "control_timeseries.csv"),
        _link("../processed/delay_aware_debug.csv", "delay_aware_debug.csv"),
        _link("../processed/speed_profile_debug.csv", "speed_profile_debug.csv"),
        _link("../processed/grade_profile.csv", "grade_profile.csv"),
        _link("../processed/motion_log.csv", "motion_log.csv"),
        "</ul></section>",
        "<section><h2>Log Excerpts</h2>",
        _log_excerpts(log_excerpts),
        "</section>",
        "</body></html>",
    ]
    index.write_text("\n".join(parts), encoding="utf-8")
    return index


def _style() -> str:
    return """
<style>
body { font-family: system-ui, sans-serif; margin: 32px; color: #202124; background: #f8fafc; }
h1, h2 { color: #111827; }
section { background: white; border: 1px solid #d8dee9; border-radius: 8px; padding: 16px; margin: 16px 0; }
table { border-collapse: collapse; width: 100%; font-size: 14px; }
th, td { border-bottom: 1px solid #e5e7eb; padding: 8px; text-align: left; vertical-align: top; }
th { background: #f1f5f9; }
.status-success { color: #047857; font-weight: 700; }
.status-partial, .status-failed { color: #b45309; font-weight: 700; }
.split-main { font-weight: 700; white-space: nowrap; }
.split-sub { color: #64748b; font-size: 12px; white-space: nowrap; }
.corner-splits-layout { display: grid; grid-template-columns: minmax(0, 1fr) 280px; gap: 16px; align-items: start; }
.corner-splits-table { overflow-x: auto; }
.corner-split-cell, .corner-split-header { cursor: default; }
.corner-split-cell.is-active, .corner-split-header.is-active { background: #fff7ed; }
.corner-map-panel { position: sticky; top: 16px; }
.corner-map-title { margin: 0 0 8px; font-size: 14px; color: #334155; }
.corner-map-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
.corner-map-card { border: 1px solid #d8dee9; border-radius: 8px; padding: 8px; background: #f8fafc; transition: border-color 120ms ease, box-shadow 120ms ease, background 120ms ease; }
.corner-map-card.is-active { border-color: #f97316; background: #fff7ed; box-shadow: 0 0 0 2px rgba(249, 115, 22, 0.18); }
.corner-map-label { margin: 0 0 4px; font-size: 12px; font-weight: 700; color: #334155; }
.corner-map-svg { display: block; width: 100%; height: auto; background: #ffffff; border-radius: 6px; }
.corner-map-track { fill: none; stroke: #94a3b8; stroke-width: 2.2; stroke-linecap: round; stroke-linejoin: round; }
.corner-map-highlight { fill: none; stroke: #f97316; stroke-width: 5; stroke-linecap: round; stroke-linejoin: round; }
	.corner-map-start { fill: #0f172a; stroke: white; stroke-width: 1.2; }
	.corner-map-empty { color: #64748b; font-size: 12px; padding: 8px; }
	.speed-profile-domain { margin-top: 18px; }
	.speed-profile-domain:first-child { margin-top: 0; }
	.speed-profile-title { margin: 0 0 10px; font-size: 15px; color: #334155; }
	.speed-profile-layout { display: grid; grid-template-columns: minmax(0, 1.1fr) minmax(320px, 0.9fr); gap: 16px; align-items: start; }
	.speed-panel { min-width: 0; }
	.speed-map-svg, .speed-chart-svg { display: block; width: 100%; height: auto; background: #ffffff; border: 1px solid #e5e7eb; border-radius: 8px; }
	.speed-map-base { fill: none; stroke: #cbd5e1; stroke-width: 2; stroke-linecap: round; stroke-linejoin: round; }
	.speed-map-segment { fill: none; stroke-width: 4; stroke-linecap: round; stroke-linejoin: round; }
	.speed-drop-marker { fill: #ef4444; stroke: #ffffff; stroke-width: 1.5; }
	.speed-chart-axis { stroke: #94a3b8; stroke-width: 1; }
	.speed-chart-grid { stroke: #e5e7eb; stroke-width: 1; }
	.speed-chart-target { fill: none; stroke: #2563eb; stroke-width: 2.2; stroke-linejoin: round; }
	.speed-chart-actual { fill: none; stroke: #f97316; stroke-width: 1.8; stroke-linejoin: round; }
	.speed-chart-label { fill: #475569; font-size: 11px; }
	.grade-profile-domain { margin-top: 18px; }
	.grade-profile-domain:first-child { margin-top: 0; }
	.grade-chart-svg { display: block; width: 100%; height: auto; background: #ffffff; border: 1px solid #e5e7eb; border-radius: 8px; }
	.grade-chart-grade { fill: none; stroke: #16a34a; stroke-width: 2.1; stroke-linejoin: round; }
	.grade-chart-accel { fill: none; stroke: #f97316; stroke-width: 1.8; stroke-linejoin: round; }
	.grade-chart-command { fill: none; stroke: #7c3aed; stroke-width: 1.8; stroke-linejoin: round; }
	.grade-chart-zero { stroke: #cbd5e1; stroke-width: 1; stroke-dasharray: 4 4; }
	.grade-extreme-table { margin-top: 12px; overflow-x: auto; }
	.speed-profile-legend { display: flex; flex-wrap: wrap; gap: 10px 14px; margin: 8px 0 0; color: #475569; font-size: 12px; }
	.speed-swatch { display: inline-block; width: 28px; height: 3px; margin-right: 6px; vertical-align: middle; border-radius: 999px; }
	.speed-drop-table { margin-top: 12px; overflow-x: auto; }
	.mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 12px; }
	a { color: #2563eb; }
	@media (max-width: 900px) {
	  .corner-splits-layout { grid-template-columns: 1fr; }
	  .corner-map-panel { position: static; }
	  .corner-map-grid { grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); }
	  .speed-profile-layout { grid-template-columns: 1fr; }
	}
	</style>
	"""


def _kv_table(values: dict[str, Any]) -> str:
    rows = ["<table><tbody>"]
    for key, value in values.items():
        rows.append(f"<tr><th>{_e(key)}</th><td class='mono'>{_e(value)}</td></tr>")
    rows.append("</tbody></table>")
    return "\n".join(rows)


def _domain_table(domains: dict[str, Any]) -> str:
    headers = [
        "domain",
        "finish",
        "total_time_sec",
        "lap_count",
        "best_lap_sec",
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
        "trajectory_source",
        "rosbag_available",
        "judgement",
    ]
    rows = ["<table><thead><tr>", *[f"<th>{_e(header)}</th>" for header in headers], "</tr></thead><tbody>"]
    for domain_id, metrics in domains.items():
        rows.append("<tr>")
        rows.append(f"<td>{_e(domain_id)}</td>")
        for header in headers[1:]:
            rows.append(f"<td>{_e(metrics.get(header))}</td>")
        rows.append("</tr>")
    if not domains:
        rows.append(f"<tr><td colspan='{len(headers)}'>No domain metrics were generated.</td></tr>")
    rows.append("</tbody></table>")
    return "\n".join(rows)


def _split_table(run_dir: Path) -> str:
    processed_dir = run_dir / "processed"
    awsim_rows = _read_awsim_section_summary(processed_dir / "awsim_section_summary.csv")
    if awsim_rows:
        return _awsim_section_split_table(awsim_rows)
    return "\n".join(
        [
            "<p>AWSIM section data was not recorded for this run. Showing fallback Corner Splits.</p>",
            _corner_split_table(run_dir),
        ]
    )


def _awsim_section_split_table(split_rows: list[dict[str, str]]) -> str:
    sections = sorted({str(row["section"]) for row in split_rows}, key=_section_sort_key)
    grouped: dict[tuple[str, int], dict[str, dict[str, str]]] = {}
    for row in split_rows:
        key = (str(row["domain_id"]), int(float(row["lap"])))
        grouped.setdefault(key, {})[str(row["section"])] = row

    header = ["domain", "lap", *[f"section_{section}" for section in sections]]
    rows = ["<table><thead><tr>", *[f"<th>{_e(item)}</th>" for item in header], "</tr></thead><tbody>"]
    for domain_id, lap in sorted(grouped):
        by_section = grouped[(domain_id, lap)]
        rows.append("<tr>")
        rows.append(f"<td>{_e(domain_id)}</td>")
        rows.append(f"<td>{_e(lap)}</td>")
        for section in sections:
            rows.append(f"<td>{_awsim_section_cell(by_section.get(section))}</td>")
        rows.append("</tr>")
    rows.append("</tbody></table>")
    return "\n".join(rows)


def _corner_split_table(run_dir: Path) -> str:
    processed_dir = run_dir / "processed"
    split_rows = _read_corner_summary(processed_dir / "corner_summary.csv")
    if not split_rows:
        return "<p>No corner split rows were generated.</p>"

    corner_ids = sorted({str(row["corner_id"]) for row in split_rows}, key=_corner_sort_key)
    grouped: dict[tuple[str, int], dict[str, dict[str, str]]] = {}
    for row in split_rows:
        key = (str(row["domain_id"]), int(row["pass"]))
        grouped.setdefault(key, {})[str(row["corner_id"])] = row

    header = ["domain", "pass", *corner_ids]
    rows = ["<table><thead><tr>", *[f"<th>{_e(item)}</th>" for item in header], "</tr></thead><tbody>"]
    for domain_id, pass_number in sorted(grouped):
        by_corner = grouped[(domain_id, pass_number)]
        rows.append("<tr>")
        rows.append(f"<td>{_e(domain_id)}</td>")
        rows.append(f"<td>{_e(pass_number)}</td>")
        for corner_id in corner_ids:
            rows.append(
                f"<td class='corner-split-cell' data-corner-id='{_attr(corner_id)}'>"
                f"{_corner_split_cell(by_corner.get(corner_id))}</td>"
            )
        rows.append("</tr>")
    rows.append("</tbody></table>")
    table = "\n".join(rows)
    map_panel = _corner_map_panel(corner_ids, _read_trajectory_reference(processed_dir / "trajectory_reference.csv"))
    return "\n".join(
        [
            "<div class='corner-splits-layout'>",
            f"<div class='corner-splits-table'>{table}</div>",
            map_panel,
            "</div>",
            _corner_map_script(),
        ]
    )


def _target_speed_profile_section(run_dir: Path) -> str:
    processed_dir = run_dir / "processed"
    vehicle_rows = _read_csv_rows(processed_dir / "vehicle_timeseries.csv")
    control_rows = _read_csv_rows(processed_dir / "control_timeseries.csv")
    if not vehicle_rows or not control_rows:
        return "<p>No target speed profile data was generated.</p>"

    domain_ids = sorted({row.get("domain_id", "") for row in vehicle_rows if row.get("domain_id")})
    parts: list[str] = []
    for domain_id in domain_ids:
        profile_rows = _target_speed_profile_rows(
            [row for row in vehicle_rows if row.get("domain_id") == domain_id],
            [row for row in control_rows if row.get("domain_id") == domain_id],
        )
        if len(profile_rows) < 2:
            continue
        drops = _target_speed_drops(profile_rows)
        parts.append(
            "\n".join(
                [
                    "<article class='speed-profile-domain'>",
                    f"<h3 class='speed-profile-title'>{_e(domain_id)}</h3>",
                    "<div class='speed-profile-layout'>",
                    f"<div class='speed-panel'>{_target_speed_map_svg(profile_rows, drops)}</div>",
                    f"<div class='speed-panel'>{_target_speed_chart_svg(profile_rows)}</div>",
                    "</div>",
                    _target_speed_drop_table(drops),
                    "</article>",
                ]
            )
        )
    if not parts:
        return "<p>No target speed profile rows could be matched to vehicle position.</p>"
    return "\n".join(parts)


def _grade_profile_section(run_dir: Path) -> str:
    processed_dir = run_dir / "processed"
    grade_rows = _read_csv_rows(processed_dir / "grade_profile.csv")
    if not grade_rows:
        return "<p>No grade profile data was generated.</p>"

    domain_ids = sorted({row.get("domain_id", "") for row in grade_rows if row.get("domain_id")})
    parts: list[str] = []
    for domain_id in domain_ids:
        rows = _grade_profile_rows([row for row in grade_rows if row.get("domain_id") == domain_id])
        if len(rows) < 2:
            continue
        parts.append(
            "\n".join(
                [
                    "<article class='grade-profile-domain'>",
                    f"<h3 class='speed-profile-title'>{_e(domain_id)}</h3>",
                    "<div class='speed-profile-layout'>",
                    f"<div class='speed-panel'>{_grade_map_svg(rows)}</div>",
                    f"<div class='speed-panel'>{_grade_chart_svg(rows)}</div>",
                    "</div>",
                    _grade_extreme_table(rows),
                    "</article>",
                ]
            )
        )
    if not parts:
        return "<p>Grade profile rows were generated, but no chartable grade samples were found.</p>"
    return "\n".join(parts)


def _read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _target_speed_profile_rows(
    vehicle_rows: list[dict[str, str]],
    control_rows: list[dict[str, str]],
) -> list[dict[str, Any]]:
    vehicles = sorted(vehicle_rows, key=lambda row: _to_float(row.get("time_sec")) or 0.0)
    controls = sorted(control_rows, key=lambda row: _to_float(row.get("time_sec")) or 0.0)
    if not vehicles or not controls:
        return []

    control_index = 0
    profile_rows: list[dict[str, Any]] = []
    for vehicle in vehicles:
        time_sec = _to_float(vehicle.get("time_sec"))
        x_m = _to_float(vehicle.get("x_m"))
        y_m = _to_float(vehicle.get("y_m"))
        if time_sec is None or x_m is None or y_m is None:
            continue
        while control_index + 1 < len(controls):
            current_delta = abs((_to_float(controls[control_index].get("time_sec")) or 0.0) - time_sec)
            next_delta = abs((_to_float(controls[control_index + 1].get("time_sec")) or 0.0) - time_sec)
            if next_delta > current_delta:
                break
            control_index += 1
        control = controls[control_index]
        control_time = _to_float(control.get("time_sec"))
        if control_time is None or abs(control_time - time_sec) > 0.25:
            continue
        target_speed = _to_float(control.get("target_speed_mps"))
        if target_speed is None:
            continue
        profile_rows.append(
            {
                "time_sec": time_sec,
                "x_m": x_m,
                "y_m": y_m,
                "distance_m": _to_float(vehicle.get("distance_m")),
                "track_s_m": _to_float(vehicle.get("track_s_m")),
                "corner_id": vehicle.get("corner_id") or "",
                "speed_mps": _to_float(vehicle.get("speed_mps")),
                "acceleration_mps2": _to_float(vehicle.get("acceleration_mps2")),
                "target_speed_mps": target_speed,
                "command_accel_mps2": _to_float(control.get("accel_mps2")),
                "command_steer_rad": _to_float(control.get("steer_rad")),
            }
        )
    return profile_rows


def _grade_profile_rows(rows: list[dict[str, str]]) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for row in sorted(rows, key=lambda item: _to_float(item.get("time_sec")) or 0.0):
        distance = _to_float(row.get("distance_m"))
        track_s = _to_float(row.get("track_s_m"))
        x_axis = distance if distance is not None else track_s
        if x_axis is None:
            continue
        output.append(
            {
                "time_sec": _to_float(row.get("time_sec")),
                "distance_m": distance,
                "track_s_m": track_s,
                "x_axis_m": x_axis,
                "x_m": _to_float(row.get("x_m")),
                "y_m": _to_float(row.get("y_m")),
                "z_m": _to_float(row.get("z_m")),
                "trajectory_z_m": _to_float(row.get("trajectory_z_m")),
                "grade_percent": _to_float(row.get("grade_percent")),
                "grade_rad": _to_float(row.get("grade_rad")),
                "grade_source": row.get("grade_source") or "",
                "speed_mps": _to_float(row.get("speed_mps")),
                "target_speed_mps": _to_float(row.get("target_speed_mps")),
                "acceleration_mps2": _to_float(row.get("acceleration_mps2")),
                "command_accel_mps2": _to_float(row.get("command_accel_mps2")),
                "command_steer_rad": _to_float(row.get("command_steer_rad")),
            }
        )
    return output


def _grade_chart_svg(rows: list[dict[str, Any]]) -> str:
    sampled = _sample_rows(rows, 900)
    grade_rows = [
        row
        for row in sampled
        if isinstance(row.get("x_axis_m"), (int, float)) and isinstance(row.get("grade_percent"), (int, float))
    ]
    accel_rows = [
        row
        for row in sampled
        if isinstance(row.get("x_axis_m"), (int, float))
        and (
            isinstance(row.get("acceleration_mps2"), (int, float))
            or isinstance(row.get("command_accel_mps2"), (int, float))
        )
    ]
    if len(grade_rows) < 2:
        return "<div class='corner-map-empty'>No grade chart data.</div>"

    width = 760.0
    height = 340.0
    pad_left = 48.0
    pad_right = 18.0
    pad_top = 18.0
    pad_bottom = 34.0
    gap = 28.0
    panel_height = (height - pad_top - pad_bottom - gap) / 2.0
    grade_top = pad_top
    grade_bottom = grade_top + panel_height
    accel_top = grade_bottom + gap
    accel_bottom = accel_top + panel_height
    min_x = min(float(row["x_axis_m"]) for row in sampled if isinstance(row.get("x_axis_m"), (int, float)))
    max_x = max(float(row["x_axis_m"]) for row in sampled if isinstance(row.get("x_axis_m"), (int, float)))

    grade_values = [float(row["grade_percent"]) for row in grade_rows]
    min_grade, max_grade = _padded_range([*grade_values, 0.0], minimum_span=0.5)
    accel_values = [
        float(value)
        for row in accel_rows
        for value in (row.get("acceleration_mps2"), row.get("command_accel_mps2"))
        if isinstance(value, (int, float))
    ]
    min_accel, max_accel = _padded_range([*accel_values, 0.0], minimum_span=0.5)

    def project_x(value: float) -> float:
        return pad_left + (value - min_x) / max(max_x - min_x, 1e-6) * (width - pad_left - pad_right)

    def project_y(value: float, min_value: float, max_value: float, top: float, bottom: float) -> float:
        return bottom - (value - min_value) / max(max_value - min_value, 1e-6) * (bottom - top)

    grade_points = []
    accel_points = []
    command_points = []
    for row in sampled:
        x_axis = row.get("x_axis_m")
        if not isinstance(x_axis, (int, float)):
            continue
        x = project_x(float(x_axis))
        grade = row.get("grade_percent")
        accel = row.get("acceleration_mps2")
        command = row.get("command_accel_mps2")
        if isinstance(grade, (int, float)):
            grade_points.append(f"{x:.2f},{project_y(float(grade), min_grade, max_grade, grade_top, grade_bottom):.2f}")
        if isinstance(accel, (int, float)):
            accel_points.append(f"{x:.2f},{project_y(float(accel), min_accel, max_accel, accel_top, accel_bottom):.2f}")
        if isinstance(command, (int, float)):
            command_points.append(f"{x:.2f},{project_y(float(command), min_accel, max_accel, accel_top, accel_bottom):.2f}")

    grade_zero = project_y(0.0, min_grade, max_grade, grade_top, grade_bottom)
    accel_zero = project_y(0.0, min_accel, max_accel, accel_top, accel_bottom)
    return "\n".join(
        [
            f"<svg class='grade-chart-svg' viewBox='0 0 {width:.0f} {height:.0f}' role='img' aria-label='grade and acceleration chart'>",
            f"<line class='speed-chart-axis' x1='{pad_left:.1f}' y1='{grade_bottom:.1f}' x2='{width - pad_right:.1f}' y2='{grade_bottom:.1f}' />",
            f"<line class='speed-chart-axis' x1='{pad_left:.1f}' y1='{grade_top:.1f}' x2='{pad_left:.1f}' y2='{grade_bottom:.1f}' />",
            f"<line class='grade-chart-zero' x1='{pad_left:.1f}' y1='{grade_zero:.1f}' x2='{width - pad_right:.1f}' y2='{grade_zero:.1f}' />",
            f"<polyline class='grade-chart-grade' points='{_attr(' '.join(grade_points))}' />",
            f"<line class='speed-chart-axis' x1='{pad_left:.1f}' y1='{accel_bottom:.1f}' x2='{width - pad_right:.1f}' y2='{accel_bottom:.1f}' />",
            f"<line class='speed-chart-axis' x1='{pad_left:.1f}' y1='{accel_top:.1f}' x2='{pad_left:.1f}' y2='{accel_bottom:.1f}' />",
            f"<line class='grade-chart-zero' x1='{pad_left:.1f}' y1='{accel_zero:.1f}' x2='{width - pad_right:.1f}' y2='{accel_zero:.1f}' />",
            f"<polyline class='grade-chart-accel' points='{_attr(' '.join(accel_points))}' />",
            f"<polyline class='grade-chart-command' points='{_attr(' '.join(command_points))}' />",
            f"<text class='speed-chart-label' x='{pad_left:.1f}' y='13'>grade %</text>",
            f"<text class='speed-chart-label' x='{pad_left:.1f}' y='{accel_top - 6:.1f}'>accel m/s^2</text>",
            f"<text class='speed-chart-label' x='{width - 106:.1f}' y='{height - 9:.1f}'>distance m</text>",
            "</svg>",
            "<div class='speed-profile-legend'>",
            "<span><span class='speed-swatch' style='background:#16a34a'></span>grade_percent</span>",
            "<span><span class='speed-swatch' style='background:#f97316'></span>acceleration_mps2</span>",
            "<span><span class='speed-swatch' style='background:#7c3aed'></span>command_accel_mps2</span>",
            "</div>",
        ]
    )


def _grade_extreme_table(rows: list[dict[str, Any]]) -> str:
    grade_rows = [row for row in rows if isinstance(row.get("grade_percent"), (int, float))]
    extremes = sorted(grade_rows, key=lambda row: abs(float(row["grade_percent"])), reverse=True)[:12]
    headers = [
        "time_sec",
        "distance_m",
        "track_s_m",
        "grade_percent",
        "grade_source",
        "speed_mps",
        "target_speed_mps",
        "acceleration_mps2",
        "command_accel_mps2",
    ]
    table = ["<div class='grade-extreme-table'><table><thead><tr>", *[f"<th>{_e(header)}</th>" for header in headers], "</tr></thead><tbody>"]
    for row in extremes:
        table.append("<tr>")
        table.append(f"<td>{_e(_format_number(row.get('time_sec'), 3))}</td>")
        table.append(f"<td>{_e(_format_number(row.get('distance_m'), 1))}</td>")
        table.append(f"<td>{_e(_format_number(row.get('track_s_m'), 1))}</td>")
        table.append(f"<td>{_e(_format_number(row.get('grade_percent'), 2))}</td>")
        table.append(f"<td>{_e(row.get('grade_source'))}</td>")
        table.append(f"<td>{_e(_format_number(row.get('speed_mps'), 2))}</td>")
        table.append(f"<td>{_e(_format_number(row.get('target_speed_mps'), 2))}</td>")
        table.append(f"<td>{_e(_format_number(row.get('acceleration_mps2'), 2))}</td>")
        table.append(f"<td>{_e(_format_number(row.get('command_accel_mps2'), 2))}</td>")
        table.append("</tr>")
    if not extremes:
        table.append(f"<tr><td colspan='{len(headers)}'>No non-empty grade samples found.</td></tr>")
    table.append("</tbody></table></div>")
    return "\n".join(table)


def _padded_range(values: list[float], minimum_span: float) -> tuple[float, float]:
    if not values:
        return -minimum_span / 2.0, minimum_span / 2.0
    min_value = min(values)
    max_value = max(values)
    span = max(max_value - min_value, minimum_span)
    padding = span * 0.08
    center = (min_value + max_value) / 2.0
    if max_value - min_value < minimum_span:
        min_value = center - minimum_span / 2.0
        max_value = center + minimum_span / 2.0
    return min_value - padding, max_value + padding


def _target_speed_drops(rows: list[dict[str, Any]], min_drop_mps: float = 0.2) -> list[dict[str, Any]]:
    drops: list[dict[str, Any]] = []
    previous: dict[str, Any] | None = None
    for row in rows:
        target_speed = row.get("target_speed_mps")
        if not isinstance(target_speed, (int, float)):
            continue
        if previous is not None:
            previous_speed = previous.get("target_speed_mps")
            if isinstance(previous_speed, (int, float)) and previous_speed - target_speed >= min_drop_mps:
                drop = dict(row)
                drop["from_target_speed_mps"] = previous_speed
                drop["drop_mps"] = previous_speed - target_speed
                speed = row.get("speed_mps")
                drop["speed_error_mps"] = (
                    float(speed) - float(target_speed) if isinstance(speed, (int, float)) else None
                )
                drops.append(drop)
        previous = row
    return drops


def _target_speed_map_svg(rows: list[dict[str, Any]], drops: list[dict[str, Any]]) -> str:
    sampled = _sample_rows(rows, 900)
    points = [(float(row["x_m"]), float(row["y_m"])) for row in sampled]
    if len(points) < 2:
        return "<div class='corner-map-empty'>No map data.</div>"
    width = 620.0
    height = 420.0
    pad = 18.0
    project = _projector(points, width, height, pad)
    speed_values = [float(row["speed_mps"]) for row in rows if isinstance(row.get("speed_mps"), (int, float))]
    if not speed_values:
        return "<div class='corner-map-empty'>No actual speed map data.</div>"
    min_speed = min(speed_values)
    max_speed = max(speed_values)
    base_points = " ".join(f"{x:.2f},{y:.2f}" for x, y in (project(point) for point in points))
    segments: list[str] = []
    for first, second in zip(sampled, sampled[1:]):
        first_point = (float(first["x_m"]), float(first["y_m"]))
        second_point = (float(second["x_m"]), float(second["y_m"]))
        x1, y1 = project(first_point)
        x2, y2 = project(second_point)
        if abs(x2 - x1) < 0.05 and abs(y2 - y1) < 0.05:
            continue
        speed = first.get("speed_mps")
        if not isinstance(speed, (int, float)):
            continue
        color = _speed_color(float(speed), min_speed, max_speed)
        title = (
            f"speed {_format_speed_kmh(speed)} / "
            f"target {_format_speed_kmh(first.get('target_speed_mps'))}"
        )
        segments.append(
            "<line class='speed-map-segment' "
            f"x1='{x1:.2f}' y1='{y1:.2f}' x2='{x2:.2f}' y2='{y2:.2f}' stroke='{_attr(color)}'>"
            f"<title>{_e(title)}</title></line>"
        )
    drop_markers = []
    for drop in drops[:80]:
        x_m = drop.get("x_m")
        y_m = drop.get("y_m")
        if not isinstance(x_m, (int, float)) or not isinstance(y_m, (int, float)):
            continue
        x, y = project((float(x_m), float(y_m)))
        label = (
            f"{_format_speed_kmh(drop.get('from_target_speed_mps'))} -> "
            f"{_format_speed_kmh(drop.get('target_speed_mps'))}, "
            f"actual {_format_speed_kmh(drop.get('speed_mps'))}"
        )
        drop_markers.append(f"<circle class='speed-drop-marker'><title>{_e(label)}</title></circle>".replace("<circle", f"<circle cx='{x:.2f}' cy='{y:.2f}' r='4.2'"))
    return "\n".join(
        [
            f"<svg class='speed-map-svg' viewBox='0 0 {width:.0f} {height:.0f}' role='img' aria-label='actual speed map'>",
            f"<polyline class='speed-map-base' points='{_attr(base_points)}' />",
            *segments,
            *drop_markers,
            "</svg>",
            "<div class='speed-profile-legend'>",
            f"<span><span class='speed-swatch' style='background:{_speed_color(min_speed, min_speed, max_speed)}'></span>actual {_e(_format_speed_kmh(min_speed))}</span>",
            f"<span><span class='speed-swatch' style='background:{_speed_color(max_speed, min_speed, max_speed)}'></span>actual {_e(_format_speed_kmh(max_speed))}</span>",
            "<span><span class='speed-swatch' style='background:#ef4444'></span>target drop</span>",
            "</div>",
        ]
    )


def _grade_map_svg(rows: list[dict[str, Any]]) -> str:
    sampled = _sample_rows(
        [
            row
            for row in rows
            if isinstance(row.get("x_m"), (int, float))
            and isinstance(row.get("y_m"), (int, float))
            and isinstance(row.get("grade_percent"), (int, float))
        ],
        900,
    )
    points = [(float(row["x_m"]), float(row["y_m"])) for row in sampled]
    if len(points) < 2:
        return "<div class='corner-map-empty'>No grade map data.</div>"
    width = 620.0
    height = 420.0
    pad = 18.0
    project = _projector(points, width, height, pad)
    grade_values = [float(row["grade_percent"]) for row in sampled]
    min_grade = min(grade_values)
    max_grade = max(grade_values)
    max_abs_grade = max(abs(min_grade), abs(max_grade), 1e-6)
    base_points = " ".join(f"{x:.2f},{y:.2f}" for x, y in (project(point) for point in points))
    segments: list[str] = []
    for first, second in zip(sampled, sampled[1:]):
        first_point = (float(first["x_m"]), float(first["y_m"]))
        second_point = (float(second["x_m"]), float(second["y_m"]))
        x1, y1 = project(first_point)
        x2, y2 = project(second_point)
        if abs(x2 - x1) < 0.05 and abs(y2 - y1) < 0.05:
            continue
        grade = first.get("grade_percent")
        if not isinstance(grade, (int, float)):
            continue
        color = _grade_color(float(grade), max_abs_grade)
        segments.append(
            "<line class='speed-map-segment' "
            f"x1='{x1:.2f}' y1='{y1:.2f}' x2='{x2:.2f}' y2='{y2:.2f}' stroke='{_attr(color)}'>"
            f"<title>grade {_e(_format_percent(grade))}</title></line>"
        )
    return "\n".join(
        [
            f"<svg class='speed-map-svg' viewBox='0 0 {width:.0f} {height:.0f}' role='img' aria-label='grade map'>",
            f"<polyline class='speed-map-base' points='{_attr(base_points)}' />",
            *segments,
            "</svg>",
            "<div class='speed-profile-legend'>",
            f"<span><span class='speed-swatch' style='background:{_grade_color(min_grade, max_abs_grade)}'></span>{_e(_format_percent(min_grade))}</span>",
            f"<span><span class='speed-swatch' style='background:{_grade_color(0.0, max_abs_grade)}'></span>{_e(_format_percent(0.0))}</span>",
            f"<span><span class='speed-swatch' style='background:{_grade_color(max_grade, max_abs_grade)}'></span>{_e(_format_percent(max_grade))}</span>",
            "</div>",
        ]
    )


def _target_speed_chart_svg(rows: list[dict[str, Any]]) -> str:
    sampled = _sample_rows(rows, 850)
    distance_values = [row.get("distance_m") for row in sampled if isinstance(row.get("distance_m"), (int, float))]
    target_values = [row.get("target_speed_mps") for row in sampled if isinstance(row.get("target_speed_mps"), (int, float))]
    actual_values = [row.get("speed_mps") for row in sampled if isinstance(row.get("speed_mps"), (int, float))]
    if len(distance_values) < 2 or not target_values:
        return "<div class='corner-map-empty'>No speed chart data.</div>"
    width = 520.0
    height = 260.0
    pad_left = 40.0
    pad_right = 14.0
    pad_top = 14.0
    pad_bottom = 30.0
    min_distance = min(distance_values)
    max_distance = max(distance_values)
    max_speed_kmh = max([float(value) * 3.6 for value in target_values + actual_values] or [1.0])
    max_speed_kmh = max(max_speed_kmh, 1.0)

    def project(distance_m: float, speed_mps: float) -> tuple[float, float]:
        x = pad_left + (distance_m - min_distance) / max(max_distance - min_distance, 1e-6) * (
            width - pad_left - pad_right
        )
        y = height - pad_bottom - (speed_mps * 3.6) / max_speed_kmh * (height - pad_top - pad_bottom)
        return x, y

    target_points = []
    actual_points = []
    for row in sampled:
        distance = row.get("distance_m")
        target = row.get("target_speed_mps")
        actual = row.get("speed_mps")
        if isinstance(distance, (int, float)) and isinstance(target, (int, float)):
            x, y = project(float(distance), float(target))
            target_points.append(f"{x:.2f},{y:.2f}")
        if isinstance(distance, (int, float)) and isinstance(actual, (int, float)):
            x, y = project(float(distance), max(0.0, float(actual)))
            actual_points.append(f"{x:.2f},{y:.2f}")

    grid_y = pad_top + (height - pad_top - pad_bottom) * 0.5
    return "\n".join(
        [
            f"<svg class='speed-chart-svg' viewBox='0 0 {width:.0f} {height:.0f}' role='img' aria-label='target and actual speed chart'>",
            f"<line class='speed-chart-axis' x1='{pad_left:.1f}' y1='{height - pad_bottom:.1f}' x2='{width - pad_right:.1f}' y2='{height - pad_bottom:.1f}' />",
            f"<line class='speed-chart-axis' x1='{pad_left:.1f}' y1='{pad_top:.1f}' x2='{pad_left:.1f}' y2='{height - pad_bottom:.1f}' />",
            f"<line class='speed-chart-grid' x1='{pad_left:.1f}' y1='{grid_y:.1f}' x2='{width - pad_right:.1f}' y2='{grid_y:.1f}' />",
            f"<polyline class='speed-chart-target' points='{_attr(' '.join(target_points))}' />",
            f"<polyline class='speed-chart-actual' points='{_attr(' '.join(actual_points))}' />",
            f"<text class='speed-chart-label' x='{pad_left:.1f}' y='12'>speed km/h</text>",
            f"<text class='speed-chart-label' x='{width - 96:.1f}' y='{height - 9:.1f}'>distance m</text>",
            "</svg>",
            "<div class='speed-profile-legend'>",
            "<span><span class='speed-swatch' style='background:#2563eb'></span>target_speed_mps</span>",
            "<span><span class='speed-swatch' style='background:#f97316'></span>speed_mps</span>",
            "</div>",
        ]
    )


def _target_speed_drop_table(drops: list[dict[str, Any]]) -> str:
    headers = [
        "time_sec",
        "distance_m",
        "track_s_m",
        "corner_id",
        "target_speed",
        "speed_mps",
        "speed_error_mps",
        "acceleration_mps2",
        "command_accel_mps2",
    ]
    rows = ["<div class='speed-drop-table'><table><thead><tr>", *[f"<th>{_e(header)}</th>" for header in headers], "</tr></thead><tbody>"]
    for drop in drops[:40]:
        rows.append("<tr>")
        rows.append(f"<td>{_e(_format_number(drop.get('time_sec'), 3))}</td>")
        rows.append(f"<td>{_e(_format_number(drop.get('distance_m'), 1))}</td>")
        rows.append(f"<td>{_e(_format_number(drop.get('track_s_m'), 1))}</td>")
        rows.append(f"<td>{_e(drop.get('corner_id'))}</td>")
        target_text = f"{_format_speed_kmh(drop.get('from_target_speed_mps'))} -> {_format_speed_kmh(drop.get('target_speed_mps'))}"
        rows.append(f"<td>{_e(target_text)}</td>")
        rows.append(f"<td>{_e(_format_number(drop.get('speed_mps'), 2))}</td>")
        rows.append(f"<td>{_e(_format_number(drop.get('speed_error_mps'), 2))}</td>")
        rows.append(f"<td>{_e(_format_number(drop.get('acceleration_mps2'), 2))}</td>")
        rows.append(f"<td>{_e(_format_number(drop.get('command_accel_mps2'), 2))}</td>")
        rows.append("</tr>")
    if not drops:
        rows.append(f"<tr><td colspan='{len(headers)}'>No target speed drops detected.</td></tr>")
    elif len(drops) > 40:
        rows.append(f"<tr><td colspan='{len(headers)}'>{len(drops) - 40} additional drops omitted.</td></tr>")
    rows.append("</tbody></table></div>")
    return "\n".join(rows)


def _sample_rows(rows: list[dict[str, Any]], max_count: int) -> list[dict[str, Any]]:
    if len(rows) <= max_count:
        return rows
    step = max(1, len(rows) // max_count)
    sampled = rows[::step]
    if sampled[-1] is not rows[-1]:
        sampled.append(rows[-1])
    return sampled


def _projector(
    points: list[tuple[float, float]],
    width: float,
    height: float,
    pad: float,
):
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)

    def project(point: tuple[float, float]) -> tuple[float, float]:
        x, y = point
        px = pad + (x - min_x) / span_x * (width - pad * 2.0)
        py = height - pad - (y - min_y) / span_y * (height - pad * 2.0)
        return px, py

    return project


def _speed_color(value: float, min_value: float, max_value: float) -> str:
    ratio = (value - min_value) / max(max_value - min_value, 1e-6)
    if ratio < 0.25:
        return "#ef4444"
    if ratio < 0.5:
        return "#f59e0b"
    if ratio < 0.75:
        return "#22c55e"
    return "#2563eb"


def _grade_color(value: float, max_abs_value: float) -> str:
    ratio = min(abs(value) / max(max_abs_value, 1e-6), 1.0)
    if abs(value) <= max_abs_value * 0.08:
        return "#64748b"
    if value < 0.0:
        if ratio < 0.45:
            return "#38bdf8"
        if ratio < 0.75:
            return "#2563eb"
        return "#1d4ed8"
    if ratio < 0.45:
        return "#facc15"
    if ratio < 0.75:
        return "#f97316"
    return "#dc2626"


def _read_corner_summary(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    rows: list[dict[str, str]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if not row.get("domain_id") or not row.get("corner_id") or not row.get("pass"):
                continue
            rows.append(row)
    return rows


def _read_awsim_section_summary(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    rows: list[dict[str, str]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if not row.get("domain_id") or not row.get("lap") or not row.get("section"):
                continue
            if not row.get("duration_sec"):
                continue
            rows.append(row)
    return rows


def _read_trajectory_reference(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    rows: list[dict[str, str]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if not row.get("domain_id") or not row.get("x_m") or not row.get("y_m"):
                continue
            rows.append(row)
    return rows


def _corner_split_cell(row: dict[str, str] | None) -> str:
    if row is None:
        return ""
    elapsed = _format_duration(row.get("exit_time_sec"))
    duration = _format_seconds(row.get("duration_sec"))
    if elapsed and duration:
        return f"<div class='split-main'>{_e(elapsed)}</div><div class='split-sub'>+{_e(duration)}</div>"
    return _e(elapsed or duration)


def _awsim_section_cell(row: dict[str, str] | None) -> str:
    if row is None:
        return ""
    duration = _format_seconds(row.get("duration_sec"))
    lap_exit = _format_duration(row.get("exit_lap_time_sec"))
    min_speed = _format_speed_kmh(row.get("min_speed_mps"))
    max_speed = _format_speed_kmh(row.get("max_speed_mps"))
    subparts = []
    if lap_exit:
        subparts.append(f"lap {lap_exit}")
    if min_speed or max_speed:
        subparts.append(f"{min_speed}..{max_speed}")
    sub = " / ".join(part for part in subparts if part)
    if duration and sub:
        return f"<div class='split-main'>{_e(duration)}</div><div class='split-sub'>{_e(sub)}</div>"
    return _e(duration or sub)


def _corner_map_panel(corner_ids: list[str], reference_rows: list[dict[str, str]]) -> str:
    if not reference_rows:
        return "<aside class='corner-map-panel'><p class='corner-map-empty'>No reference trajectory map data.</p></aside>"
    domain_rows = _first_domain_reference_rows(reference_rows)
    cards = [
        "<aside class='corner-map-panel'>",
        "<h3 class='corner-map-title'>Corner Map</h3>",
        "<div class='corner-map-grid'>",
    ]
    for corner_id in corner_ids:
        cards.append(
            "<article class='corner-map-card' "
            f"data-corner-map='{_attr(corner_id)}'>"
            f"<h4 class='corner-map-label'>{_e(corner_id)}</h4>"
            f"{_mini_map_svg(domain_rows, corner_id)}"
            "</article>"
        )
    cards.extend(["</div>", "</aside>"])
    return "\n".join(cards)


def _first_domain_reference_rows(reference_rows: list[dict[str, str]]) -> list[dict[str, str]]:
    first_domain = reference_rows[0].get("domain_id")
    return [row for row in reference_rows if row.get("domain_id") == first_domain]


def _mini_map_svg(reference_rows: list[dict[str, str]], corner_id: str) -> str:
    points = _map_points(reference_rows)
    if len(points) < 2:
        return "<div class='corner-map-empty'>No map data.</div>"

    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    width = 160.0
    height = 110.0
    pad = 8.0

    def project(point: tuple[float, float, str | None]) -> tuple[float, float]:
        x, y, _corner = point
        px = pad + (x - min_x) / span_x * (width - pad * 2.0)
        py = height - pad - (y - min_y) / span_y * (height - pad * 2.0)
        return px, py

    full_polyline = _polyline(points, project)
    highlight_segments = _corner_segments(points, corner_id)
    highlight = "\n".join(
        f"<polyline class='corner-map-highlight' points='{_attr(_polyline(segment, project))}' />"
        for segment in highlight_segments
        if len(segment) >= 2
    )
    first_x, first_y = project(points[0])
    return "\n".join(
        [
            f"<svg class='corner-map-svg' viewBox='0 0 {width:.0f} {height:.0f}' role='img' "
            f"aria-label='{_attr(corner_id)} map'>",
            f"<polyline class='corner-map-track' points='{_attr(full_polyline)}' />",
            highlight,
            f"<circle class='corner-map-start' cx='{first_x:.2f}' cy='{first_y:.2f}' r='2.8' />",
            "</svg>",
        ]
    )


def _map_points(reference_rows: list[dict[str, str]]) -> list[tuple[float, float, str | None]]:
    points: list[tuple[float, float, str | None]] = []
    for row in reference_rows:
        x = _to_float(row.get("x_m"))
        y = _to_float(row.get("y_m"))
        if x is None or y is None:
            continue
        points.append((x, y, row.get("corner_id") or None))
    return points


def _corner_segments(
    points: list[tuple[float, float, str | None]],
    corner_id: str,
) -> list[list[tuple[float, float, str | None]]]:
    segments: list[list[tuple[float, float, str | None]]] = []
    current: list[tuple[float, float, str | None]] = []
    for point in points:
        if point[2] == corner_id:
            current.append(point)
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)
    return segments


def _polyline(
    points: list[tuple[float, float, str | None]],
    project,
) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in (project(point) for point in points))


def _corner_map_script() -> str:
    return """
<script>
(() => {
  const cells = document.querySelectorAll("[data-corner-id]");
  const maps = document.querySelectorAll("[data-corner-map]");
  const setActive = (cornerId, active) => {
    cells.forEach((cell) => {
      if (cell.dataset.cornerId === cornerId) cell.classList.toggle("is-active", active);
    });
    maps.forEach((map) => {
      if (map.dataset.cornerMap === cornerId) map.classList.toggle("is-active", active);
    });
  };
  cells.forEach((cell) => {
    const cornerId = cell.dataset.cornerId;
    cell.addEventListener("mouseenter", () => setActive(cornerId, true));
    cell.addEventListener("mouseleave", () => setActive(cornerId, false));
  });
  maps.forEach((map) => {
    const cornerId = map.dataset.cornerMap;
    map.addEventListener("mouseenter", () => setActive(cornerId, true));
    map.addEventListener("mouseleave", () => setActive(cornerId, false));
  });
})();
</script>
"""


def _format_duration(value: str | float | None) -> str:
    seconds = _to_float(value)
    if seconds is None:
        return ""
    minutes = int(seconds // 60)
    remainder = seconds - minutes * 60
    return f"{minutes}:{remainder:06.3f}"


def _format_seconds(value: str | float | None) -> str:
    seconds = _to_float(value)
    if seconds is None:
        return ""
    return f"{seconds:.3f}s"


def _format_number(value: Any, digits: int) -> str:
    number = _to_float(value)
    if number is None:
        return ""
    return f"{number:.{digits}f}"


def _format_speed_kmh(value: Any) -> str:
    number = _to_float(value)
    if number is None:
        return ""
    return f"{number * 3.6:.1f} km/h"


def _format_percent(value: Any) -> str:
    number = _to_float(value)
    if number is None:
        return ""
    return f"{number:.2f}%"


def _to_float(value: str | float | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _corner_sort_key(value: str) -> tuple[str, int | str]:
    prefix, _, suffix = value.rpartition("_")
    if suffix.isdigit():
        return prefix, int(suffix)
    return value, value


def _section_sort_key(value: str) -> tuple[int, str]:
    if value.isdigit():
        return int(value), value
    try:
        return int(float(value)), value
    except ValueError:
        return 10**9, value


def _artifact_links(run_dir: Path, domain_ids) -> str:
    rows = ["<ul>"]
    for domain_id in domain_ids:
        raw_domain = run_dir / "raw" / str(domain_id)
        rows.append(f"<li><strong>{_e(domain_id)}</strong><ul>")
        for rel in [
            "result-summary.json",
            f"{domain_id}-result-details.json",
            "autoware.log",
            "rosbag2_autoware",
            "capture",
        ]:
            if (raw_domain / rel).exists():
                rows.append(_link(f"../raw/{domain_id}/{rel}", rel))
        for motion in sorted(raw_domain.glob("motion_analytics-*.html")):
            rows.append(_link(f"../raw/{domain_id}/{motion.name}", motion.name))
        rows.append("</ul></li>")
    if not list(domain_ids):
        rows.append("<li>No raw domain artifacts copied.</li>")
    rows.append("</ul>")
    return "\n".join(rows)


def _log_excerpts(excerpts: dict[str, Any]) -> str:
    rows = ["<table><thead><tr><th>domain</th><th>path</th><th>line</th><th>text</th></tr></thead><tbody>"]
    count = 0
    for domain_id, items in excerpts.items():
        for item in items:
            rows.append(
                "<tr>"
                f"<td>{_e(domain_id)}</td>"
                f"<td class='mono'>{_e(item.get('path'))}</td>"
                f"<td>{_e(item.get('line_no'))}</td>"
                f"<td>{_e(item.get('text'))}</td>"
                "</tr>"
            )
            count += 1
    if count == 0:
        rows.append("<tr><td colspan='4'>No warning/error excerpts found.</td></tr>")
    rows.append("</tbody></table>")
    return "\n".join(rows)


def _link(href: str, label: str) -> str:
    return f"<li><a href='{html.escape(href, quote=True)}'>{_e(label)}</a></li>"


def _e(value: Any) -> str:
    if value is None:
        return ""
    return html.escape(str(value))


def _attr(value: Any) -> str:
    if value is None:
        return ""
    return html.escape(str(value), quote=True)
