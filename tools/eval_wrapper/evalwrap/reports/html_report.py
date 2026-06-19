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
        "<section><h2>Corner Splits</h2>",
        _corner_split_table(run_dir),
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
        _link("../processed/corner_summary.csv", "corner_summary.csv"),
        _link("../processed/trajectory_reference.csv", "trajectory_reference.csv"),
        _link("../processed/vehicle_timeseries.csv", "vehicle_timeseries.csv"),
        _link("../processed/control_timeseries.csv", "control_timeseries.csv"),
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
.mono { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 12px; }
a { color: #2563eb; }
@media (max-width: 900px) {
  .corner-splits-layout { grid-template-columns: 1fr; }
  .corner-map-panel { position: static; }
  .corner-map-grid { grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); }
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
