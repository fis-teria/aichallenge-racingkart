from __future__ import annotations

import csv
import html
import json
from pathlib import Path
from typing import Any


def generate_overtake_report(run_dir: Path) -> Path | None:
    processed_dir = run_dir / "processed"
    metrics_path = processed_dir / "overtake_metrics.json"
    if not metrics_path.exists():
        return None
    metrics = _read_json(metrics_path)
    attempts = _read_csv(processed_dir / "overtake_attempts.csv")
    timeseries = _read_csv(processed_dir / "overtake_timeseries.csv")
    bins = _read_csv(processed_dir / "overtake_map_bins.csv")
    report_dir = run_dir / "report"
    report_dir.mkdir(parents=True, exist_ok=True)
    path = report_dir / "overtake.html"
    path.write_text(_html(metrics, attempts, timeseries, bins), encoding="utf-8")
    return path


def overtake_index_section(run_dir: Path, metrics: dict[str, Any]) -> str:
    overtake = metrics.get("overtake", {})
    domains = overtake.get("domains", {}) if isinstance(overtake, dict) else {}
    if not domains:
        return "<p>No overtake analysis was generated.</p>"
    rows = [
        "<table><thead><tr><th>domain</th><th>available</th><th>attempts</th><th>success_rate</th><th>blocked_time_sec</th><th>missed</th><th>judgement</th></tr></thead><tbody>"
    ]
    for domain_id, data in domains.items():
        rows.append(
            "<tr>"
            f"<td>{_e(domain_id)}</td>"
            f"<td>{_e(data.get('analysis_available'))}</td>"
            f"<td>{_e(data.get('attempt_count'))}</td>"
            f"<td>{_fmt(data.get('success_rate'))}</td>"
            f"<td>{_fmt(data.get('blocked_time_sec'))}</td>"
            f"<td>{_e(data.get('missed_overtake_chance_count'))}</td>"
            f"<td>{_e(data.get('judgement'))}</td>"
            "</tr>"
        )
    rows.append("</tbody></table>")
    link = "<p><a href='overtake.html'>Open overtake report</a></p>" if (run_dir / "report" / "overtake.html").exists() else ""
    warnings = overtake.get("warnings", []) if isinstance(overtake, dict) else []
    warning_html = "".join(f"<li>{_e(warning)}</li>" for warning in warnings)
    if warning_html:
        warning_html = f"<ul>{warning_html}</ul>"
    return "\n".join([*rows, link, warning_html])


def _html(
    metrics: dict[str, Any],
    attempts: list[dict[str, str]],
    timeseries: list[dict[str, str]],
    bins: list[dict[str, str]],
) -> str:
    return "\n".join(
        [
            "<!doctype html><html><head><meta charset='utf-8'><title>overtake report</title>",
            _style(),
            "</head><body>",
            f"<h1>Overtake Report: {_e(metrics.get('run_id', 'run'))}</h1>",
            "<section><h2>Summary</h2>",
            _summary_table(metrics),
            "</section>",
            "<section><h2>Decision Summary</h2>",
            _decision_summary(metrics, timeseries, attempts),
            "</section>",
            "<section><h2>Event Map</h2>",
            _event_map(attempts, timeseries),
            "</section>",
            "<section><h2>Map Bins</h2>",
            _table(bins[:80], ["domain_id", "bin_id", "section", "attempt_count", "success_rate", "blocked_time_sec", "missed_chance_count", "max_cbf_slack"]),
            "</section>",
            "<section><h2>Timeline</h2>",
            _state_timeline(timeseries),
            "</section>",
            "<section><h2>CBF / MPC</h2>",
            _cbf_chart(timeseries),
            "</section>",
            "<section><h2>Abort Reasons</h2>",
            _abort_bars(metrics),
            "</section>",
            "<section><h2>Attempts</h2>",
            _table(attempts, ["domain_id", "attempt_id", "result", "abort_reason", "start_time_sec", "end_time_sec", "time_to_pass_sec", "min_vehicle_distance_m", "min_cbf_h", "max_cbf_slack"]),
            "</section>",
            "<section><h2>Processed Files</h2>",
            "<ul>",
            "<li><a href='../processed/overtake_metrics.json'>overtake_metrics.json</a></li>",
            "<li><a href='../processed/overtake_attempts.csv'>overtake_attempts.csv</a></li>",
            "<li><a href='../processed/overtake_timeseries.csv'>overtake_timeseries.csv</a></li>",
            "<li><a href='../processed/overtake_map_bins.csv'>overtake_map_bins.csv</a></li>",
            "</ul></section>",
            "</body></html>",
        ]
    )


def _decision_summary(
    metrics: dict[str, Any],
    timeseries: list[dict[str, str]],
    attempts: list[dict[str, str]],
) -> str:
    parts = [_decision_overview(metrics, timeseries)]
    if not _has_overtake_scene(metrics, timeseries, attempts):
        parts.append(
            "<p class='notice muted'>No overtake scene observed. "
            "The debug topic was parsed, but no front/side vehicle, blocked state, "
            "side-by-side state, or active override sample was found.</p>"
        )
        return "\n".join(parts)

    samples = _decision_samples(timeseries)
    parts.extend(
        [
            "<h3>Decision Samples</h3>",
            _table(
                samples[:120],
                [
                    "domain_id",
                    "timestamp_sec",
                    "s",
                    "overtake_state",
                    "selected",
                    "blocked",
                    "side_by_side",
                    "corner_side_by_side",
                    "corner_abs_curvature",
                    "front_vehicle_id",
                    "front_distance_m",
                    "front_delta_d",
                    "can_pass_left",
                    "can_pass_right",
                    "pass_gap_reason",
                    "active_override",
                    "reason",
                ],
            ),
        ]
    )
    if len(samples) > 120:
        parts.append(f"<p class='notice'>Showing 120 of {len(samples)} decision-change samples.</p>")
    return "\n".join(parts)


def _decision_overview(metrics: dict[str, Any], timeseries: list[dict[str, str]]) -> str:
    domains = metrics.get("domains", {})
    domain_ids = set(domains) if isinstance(domains, dict) else set()
    domain_ids.update(str(row.get("domain_id") or "") for row in timeseries if row.get("domain_id"))
    rows = []
    for domain_id in sorted(domain_ids):
        domain_rows = [row for row in timeseries if str(row.get("domain_id") or "") == domain_id]
        domain_metrics = domains.get(domain_id, {}) if isinstance(domains, dict) else {}
        rows.append(
            {
                "domain_id": domain_id,
                "samples": len(domain_rows),
                "attempts": domain_metrics.get("attempt_count"),
                "blocked_samples": sum(1 for row in domain_rows if _truthy(row.get("blocked"))),
                "side_by_side_samples": sum(1 for row in domain_rows if _truthy(row.get("side_by_side"))),
                "corner_side_by_side_samples": sum(
                    1 for row in domain_rows if _truthy(row.get("corner_side_by_side"))
                ),
                "active_override_samples": sum(1 for row in domain_rows if _truthy(row.get("active_override"))),
                "front_vehicle_samples": sum(1 for row in domain_rows if _filled(row.get("front_vehicle_id"))),
                "selected_counts": _top_counts(domain_rows, "selected"),
                "pass_gap_reasons": _top_counts(domain_rows, "pass_gap_reason"),
                "reasons": _top_counts(domain_rows, "reason"),
            }
        )
    return _table(
        rows,
        [
            "domain_id",
            "samples",
            "attempts",
            "blocked_samples",
            "side_by_side_samples",
            "corner_side_by_side_samples",
            "active_override_samples",
            "front_vehicle_samples",
            "selected_counts",
            "pass_gap_reasons",
            "reasons",
        ],
    )


def _has_overtake_scene(
    metrics: dict[str, Any],
    timeseries: list[dict[str, str]],
    attempts: list[dict[str, str]],
) -> bool:
    if attempts:
        return True
    for domain in metrics.get("domains", {}).values():
        if _int(domain.get("attempt_count")) > 0:
            return True
        if (_float(domain.get("blocked_time_sec")) or 0.0) > 0.0:
            return True
        if _int(domain.get("missed_overtake_chance_count")) > 0:
            return True
    return any(_is_interesting_decision_row(row) for row in timeseries)


def _decision_samples(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    samples = []
    last_signature_by_domain: dict[str, tuple[object, ...]] = {}
    for row in sorted(rows, key=lambda item: (str(item.get("domain_id") or ""), _float(item.get("timestamp_sec")) or 0.0)):
        if not _is_interesting_decision_row(row):
            continue
        domain_id = str(row.get("domain_id") or "")
        signature = (
            row.get("overtake_state"),
            row.get("selected"),
            row.get("blocked"),
            row.get("side_by_side"),
            row.get("corner_side_by_side"),
            row.get("active_override"),
            row.get("front_vehicle_id"),
            row.get("can_pass_left"),
            row.get("can_pass_right"),
            row.get("pass_gap_reason"),
            row.get("reason"),
        )
        if signature == last_signature_by_domain.get(domain_id):
            continue
        samples.append(row)
        last_signature_by_domain[domain_id] = signature
    return samples


def _is_interesting_decision_row(row: dict[str, str]) -> bool:
    selected = str(row.get("selected") or "").strip().upper()
    pass_gap_reason = str(row.get("pass_gap_reason") or "").strip()
    return (
        _truthy(row.get("blocked"))
        or _truthy(row.get("side_by_side"))
        or _truthy(row.get("corner_side_by_side"))
        or _truthy(row.get("active_override"))
        or _filled(row.get("front_vehicle_id"))
        or _filled(row.get("side_vehicle_id"))
        or selected not in {"", "FASTEST"}
        or pass_gap_reason not in {"", "no_target"}
        or _filled(row.get("reason"))
    )


def _summary_table(metrics: dict[str, Any]) -> str:
    rows = []
    for domain_id, data in metrics.get("domains", {}).items():
        row = {"domain_id": domain_id, **data}
        rows.append(row)
    return _table(
        rows,
        [
            "domain_id",
            "analysis_available",
            "attempt_count",
            "success_count",
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
            "judgement",
        ],
    )


def _event_map(attempts: list[dict[str, str]], timeseries: list[dict[str, str]]) -> str:
    points = []
    for row in timeseries:
        x = _float(row.get("ego_x"))
        y = _float(row.get("ego_y"))
        if x is not None and y is not None:
            points.append((x, y))
    if len(points) < 2:
        return "<p>No map coordinates were available.</p>"
    min_x, max_x = min(x for x, _ in points), max(x for x, _ in points)
    min_y, max_y = min(y for _, y in points), max(y for _, y in points)
    width, height, pad = 720.0, 420.0, 24.0

    def project(x: float, y: float) -> tuple[float, float]:
        px = pad + (x - min_x) / max(max_x - min_x, 1e-9) * (width - pad * 2)
        py = height - pad - (y - min_y) / max(max_y - min_y, 1e-9) * (height - pad * 2)
        return px, py

    polyline = " ".join(f"{project(x, y)[0]:.1f},{project(x, y)[1]:.1f}" for x, y in points[:: max(1, len(points) // 600)])
    markers = []
    colors = {"success": "#16a34a", "unsafe_success": "#f97316", "aborted": "#f97316", "failed": "#dc2626", "unknown": "#64748b"}
    for attempt in attempts:
        x = _float(attempt.get("start_x"))
        y = _float(attempt.get("start_y"))
        if x is None or y is None:
            continue
        px, py = project(x, y)
        color = colors.get(str(attempt.get("result")), "#2563eb")
        markers.append(f"<circle cx='{px:.1f}' cy='{py:.1f}' r='5' fill='{color}' stroke='white' stroke-width='1.5'><title>{_attr(attempt.get('result'))}</title></circle>")
    return (
        f"<svg class='overtake-map' viewBox='0 0 {width:.0f} {height:.0f}' role='img' aria-label='overtake event map'>"
        f"<polyline points='{polyline}' fill='none' stroke='#94a3b8' stroke-width='2'/>"
        + "\n".join(markers)
        + "</svg>"
    )


def _state_timeline(rows: list[dict[str, str]]) -> str:
    if not rows:
        return "<p>No overtake time-series rows were generated.</p>"
    state_order = ["NORMAL", "FOLLOWING", "OVERTAKE_PREP", "OVERTAKING", "RETURNING", "ABORTED", "UNKNOWN"]
    values = []
    for row in rows:
        t = _float(row.get("timestamp_sec"))
        if t is None:
            continue
        state = row.get("overtake_state") or "UNKNOWN"
        values.append((t, state_order.index(state) if state in state_order else len(state_order) - 1))
    if len(values) < 2:
        return "<p>Not enough timeline samples.</p>"
    return _line_chart(values, "state timeline", y_labels=state_order)


def _cbf_chart(rows: list[dict[str, str]]) -> str:
    values = []
    for row in rows:
        t = _float(row.get("timestamp_sec"))
        h = _float(row.get("min_cbf_h"))
        if t is not None and h is not None:
            values.append((t, h))
    if len(values) < 2:
        return "<p>No CBF margin samples were available.</p>"
    return _line_chart(values, "CBF margin", color="#dc2626")


def _abort_bars(metrics: dict[str, Any]) -> str:
    counts: dict[str, int] = {}
    for domain in metrics.get("domains", {}).values():
        for reason, count in (domain.get("abort_reason_counts") or {}).items():
            counts[str(reason)] = counts.get(str(reason), 0) + int(count)
    if not counts:
        return "<p>No abort reasons were recorded.</p>"
    max_count = max(counts.values())
    rows = ["<div class='bars'>"]
    for reason, count in sorted(counts.items()):
        width = 100.0 * count / max(max_count, 1)
        rows.append(f"<div class='bar-row'><span>{_e(reason)}</span><div><i style='width:{width:.1f}%'></i></div><b>{count}</b></div>")
    rows.append("</div>")
    return "\n".join(rows)


def _line_chart(values: list[tuple[float, float]], label: str, *, color: str = "#2563eb", y_labels: list[str] | None = None) -> str:
    width, height, pad = 720.0, 220.0, 28.0
    min_t, max_t = values[0][0], values[-1][0]
    ys = [y for _, y in values]
    min_y, max_y = min(ys), max(ys)
    if y_labels:
        min_y, max_y = 0.0, float(len(y_labels) - 1)

    def project(t: float, y: float) -> tuple[float, float]:
        x = pad + (t - min_t) / max(max_t - min_t, 1e-9) * (width - pad * 2)
        py = height - pad - (y - min_y) / max(max_y - min_y, 1e-9) * (height - pad * 2)
        return x, py

    points = " ".join(f"{project(t, y)[0]:.1f},{project(t, y)[1]:.1f}" for t, y in values[:: max(1, len(values) // 700)])
    labels = ""
    if y_labels:
        label_items = []
        for idx, state in enumerate(y_labels):
            _, y = project(min_t, float(idx))
            label_items.append(f"<text x='4' y='{y + 3:.1f}' font-size='9' fill='#64748b'>{_e(state)}</text>")
        labels = "\n".join(label_items)
    return (
        f"<svg class='overtake-chart' viewBox='0 0 {width:.0f} {height:.0f}' role='img' aria-label='{_attr(label)}'>"
        f"{labels}<polyline points='{points}' fill='none' stroke='{color}' stroke-width='2'/>"
        "</svg>"
    )


def _table(rows: list[dict[str, Any]], headers: list[str]) -> str:
    body = ["<table><thead><tr>", *[f"<th>{_e(header)}</th>" for header in headers], "</tr></thead><tbody>"]
    for row in rows:
        body.append("<tr>")
        for header in headers:
            body.append(f"<td>{_fmt(row.get(header))}</td>")
        body.append("</tr>")
    if not rows:
        body.append(f"<tr><td colspan='{len(headers)}'>No rows.</td></tr>")
    body.append("</tbody></table>")
    return "\n".join(body)


def _style() -> str:
    return """
<style>
body{font-family:system-ui,sans-serif;margin:32px;color:#202124;background:#f8fafc}
section{background:white;border:1px solid #d8dee9;border-radius:8px;padding:16px;margin:16px 0}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{border-bottom:1px solid #e5e7eb;padding:7px;text-align:left}th{background:#f1f5f9}
h3{margin:18px 0 10px;font-size:16px}
.overtake-map,.overtake-chart{display:block;width:100%;height:auto;background:white;border:1px solid #e5e7eb;border-radius:8px}
.bars{display:grid;gap:8px}.bar-row{display:grid;grid-template-columns:180px 1fr 48px;gap:10px;align-items:center}.bar-row div{height:12px;background:#e5e7eb;border-radius:999px;overflow:hidden}.bar-row i{display:block;height:100%;background:#f97316}
.notice{padding:10px 12px;border-radius:8px;background:#eff6ff;border:1px solid #bfdbfe;color:#1e3a8a}
.notice.muted{background:#f8fafc;border-color:#e2e8f0;color:#475569}
a{color:#2563eb}
</style>
"""


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _int(value: object) -> int:
    parsed = _float(value)
    if parsed is None:
        return 0
    return int(parsed)


def _truthy(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    text = str(value).strip().lower()
    return text in {"1", "true", "yes", "y", "on"}


def _filled(value: object) -> bool:
    return value not in (None, "")


def _top_counts(rows: list[dict[str, str]], key: str, *, limit: int = 3) -> str:
    counts: dict[str, int] = {}
    for row in rows:
        value = row.get(key)
        if not _filled(value):
            continue
        text = str(value)
        counts[text] = counts.get(text, 0) + 1
    if not counts:
        return ""
    items = sorted(counts.items(), key=lambda item: (-item[1], item[0]))[:limit]
    return ", ".join(f"{name}:{count}" for name, count in items)


def _fmt(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.3f}"
    parsed = _float(value)
    if parsed is not None and isinstance(value, str) and any(ch in value for ch in ".eE"):
        return f"{parsed:.3f}"
    return _e(value)


def _e(value: object) -> str:
    return html.escape("" if value is None else str(value))


def _attr(value: object) -> str:
    return _e(value).replace("'", "&#x27;")
