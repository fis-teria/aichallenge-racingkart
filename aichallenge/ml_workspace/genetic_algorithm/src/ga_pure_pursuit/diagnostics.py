from __future__ import annotations

import argparse
import html
import json
import math
import sqlite3
from pathlib import Path
from typing import Any


SUMMARY_FIELDS = (
    "lap_time_seconds",
    "elapsed_seconds",
    "distance_traveled_m",
    "initial_speed_mps",
    "actual_speed_mean_mps",
    "actual_speed_max_mps",
    "target_speed_mean_mps",
    "absolute_speed_error_mean_mps",
    "absolute_speed_error_p95_mps",
    "commanded_acceleration_mean_mps2",
    "commanded_acceleration_max_mps2",
    "actual_acceleration_mean_mps2",
    "actual_acceleration_max_mps2",
    "speed_limited_ratio",
    "speed_limited_estimated_seconds",
    "time_to_10_kmh_seconds",
    "time_to_20_kmh_seconds",
    "time_to_30_kmh_seconds",
    "time_to_34_kmh_seconds",
    "lap_completion_speed_mps",
)


def load_candidate_metrics(database: Path, candidate_id: str | None) -> tuple[str, dict[str, Any]]:
    connection = sqlite3.connect(database)
    try:
        if candidate_id is None:
            row = connection.execute(
                """
                SELECT c.candidate_id, e.metrics_json
                FROM candidates AS c
                JOIN episodes AS e ON e.candidate_id = c.candidate_id
                WHERE c.status = 'complete'
                ORDER BY c.fitness ASC
                LIMIT 1
                """
            ).fetchone()
        else:
            row = connection.execute(
                """
                SELECT c.candidate_id, e.metrics_json
                FROM candidates AS c
                JOIN episodes AS e ON e.candidate_id = c.candidate_id
                WHERE c.candidate_id = ?
                ORDER BY e.repeat_index ASC
                LIMIT 1
                """,
                (candidate_id,),
            ).fetchone()
    finally:
        connection.close()
    if row is None:
        raise ValueError(f"candidate not found: {candidate_id or '<best>'}")
    return str(row[0]), json.loads(row[1])


def _format_value(name: str, value: Any) -> str:
    if value is None:
        return "not reached"
    if isinstance(value, float):
        if name.endswith("_mps"):
            return f"{value:.3f} m/s ({value * 3.6:.2f} km/h)"
        if name.endswith("_ratio"):
            return f"{value:.3f} ({value * 100.0:.1f}%)"
        if name.endswith("_seconds"):
            return f"{value:.3f} s"
        if name.endswith("_m"):
            return f"{value:.3f} m"
        return f"{value:.6f}"
    return str(value)


def render_html(candidate_id: str, metrics: dict[str, Any]) -> str:
    trace = list(metrics.get("diagnostic_trace", []))
    width, height = 1200, 560
    left, right, top, bottom = 70, 30, 35, 65
    plot_width = width - left - right
    plot_height = height - top - bottom
    duration = max(
        (float(item.get("elapsed_seconds") or 0.0) for item in trace),
        default=float(metrics.get("elapsed_seconds") or 1.0),
    )
    duration = max(duration, 1.0)
    visible_speeds = [
        float(value) * 3.6
        for item in trace
        for value in (item.get("actual_speed_mps"), item.get("target_speed_mps"))
        if value is not None and math.isfinite(float(value))
    ]
    y_max = max(40.0, math.ceil(max(visible_speeds, default=40.0) / 5.0) * 5.0)

    def x_position(seconds: float) -> float:
        return left + plot_width * max(0.0, min(duration, seconds)) / duration

    def y_position(speed_kmh: float) -> float:
        clipped = max(0.0, min(y_max, speed_kmh))
        return top + plot_height * (1.0 - clipped / y_max)

    def polyline(field: str, color: str, dash: str = "") -> str:
        points = []
        for item in trace:
            value = item.get(field)
            if value is None:
                continue
            points.append(
                f"{x_position(float(item['elapsed_seconds'])):.1f},"
                f"{y_position(float(value) * 3.6):.1f}"
            )
        dash_attribute = f' stroke-dasharray="{dash}"' if dash else ""
        return (
            f'<polyline fill="none" stroke="{color}" stroke-width="2"'
            f'{dash_attribute} points="{" ".join(points)}"/>'
        )

    visible_accelerations = [
        abs(float(value))
        for item in trace
        for value in (
            item.get("commanded_acceleration_mps2"),
            item.get("actual_acceleration_mps2"),
        )
        if value is not None and math.isfinite(float(value))
    ]
    acceleration_max = max(
        5.0, math.ceil(max(visible_accelerations, default=5.0))
    )

    def acceleration_y_position(acceleration: float) -> float:
        clipped = max(-acceleration_max, min(acceleration_max, acceleration))
        return top + plot_height * (
            1.0 - (clipped + acceleration_max) / (2.0 * acceleration_max)
        )

    def acceleration_polyline(field: str, color: str) -> str:
        points = []
        for item in trace:
            value = item.get(field)
            if value is None:
                continue
            points.append(
                f"{x_position(float(item['elapsed_seconds'])):.1f},"
                f"{acceleration_y_position(float(value)):.1f}"
            )
        return (
            f'<polyline fill="none" stroke="{color}" stroke-width="2" '
            f'points="{" ".join(points)}"/>'
        )

    limited_regions = []
    for index, item in enumerate(trace):
        if not item.get("speed_limited"):
            continue
        start = float(item.get("elapsed_seconds") or 0.0)
        end = (
            float(trace[index + 1].get("elapsed_seconds") or start)
            if index + 1 < len(trace)
            else duration
        )
        limited_regions.append(
            f'<rect x="{x_position(start):.1f}" y="{top}" '
            f'width="{max(1.0, x_position(end) - x_position(start)):.1f}" '
            f'height="{plot_height}" fill="#ef4444" opacity="0.08"/>'
        )

    grid = []
    for speed in range(0, int(y_max) + 1, 5):
        y = y_position(float(speed))
        grid.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" '
            'stroke="#d1d5db" stroke-width="1"/>'
            f'<text x="{left-10}" y="{y+4:.1f}" text-anchor="end">{speed}</text>'
        )
    for tick in range(0, 11):
        seconds = duration * tick / 10.0
        x = x_position(seconds)
        grid.append(
            f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{height-bottom}" '
            'stroke="#e5e7eb" stroke-width="1"/>'
            f'<text x="{x:.1f}" y="{height-bottom+24}" text-anchor="middle">'
            f"{seconds:.0f}</text>"
        )

    rows = "".join(
        f"<tr><th>{html.escape(name)}</th>"
        f"<td>{html.escape(_format_value(name, metrics.get(name)))}</td></tr>"
        for name in SUMMARY_FIELDS
    )
    split_rows = "".join(
        "<tr>"
        f"<td>{split.get('lap')}</td><td>{split.get('section')}</td>"
        f"<td>{_format_value('lap_time_seconds', split.get('lap_time_seconds'))}</td>"
        f"<td>{_format_value('section_duration_seconds', split.get('section_duration_seconds'))}</td>"
        "</tr>"
        for split in metrics.get("section_splits", [])
    )
    return f"""<!doctype html>
<html lang="en">
<meta charset="utf-8">
<title>Pure Pursuit diagnostics - {html.escape(candidate_id)}</title>
<style>
body {{ font-family: sans-serif; margin: 24px; color: #111827; }}
svg {{ width: 100%; max-width: {width}px; border: 1px solid #d1d5db; }}
table {{ border-collapse: collapse; margin: 18px 0; }}
th, td {{ border: 1px solid #d1d5db; padding: 6px 10px; text-align: left; }}
.legend span {{ margin-right: 20px; }}
</style>
<h1>Pure Pursuit diagnostics: {html.escape(candidate_id)}</h1>
<div class="legend">
  <span style="color:#2563eb">Actual speed</span>
  <span style="color:#f59e0b">Target speed</span>
  <span style="color:#7c3aed">Curvature limit</span>
  <span style="color:#ef4444">Red background: speed limited</span>
</div>
<svg viewBox="0 0 {width} {height}" role="img">
  <rect width="{width}" height="{height}" fill="white"/>
  {''.join(limited_regions)}
  {''.join(grid)}
  {polyline('actual_speed_mps', '#2563eb')}
  {polyline('target_speed_mps', '#f59e0b')}
  {polyline('curvature_speed_limit_mps', '#7c3aed', '6 4')}
  <text x="{width/2}" y="{height-12}" text-anchor="middle">Elapsed time (s)</text>
  <text x="18" y="{height/2}" text-anchor="middle"
        transform="rotate(-90 18 {height/2})">Speed (km/h)</text>
</svg>
<h2>Longitudinal acceleration</h2>
<div class="legend">
  <span style="color:#dc2626">Commanded acceleration</span>
  <span style="color:#059669">Measured acceleration</span>
</div>
<svg viewBox="0 0 {width} {height}" role="img">
  <rect width="{width}" height="{height}" fill="white"/>
  <line x1="{left}" y1="{acceleration_y_position(0.0):.1f}"
        x2="{width-right}" y2="{acceleration_y_position(0.0):.1f}"
        stroke="#9ca3af" stroke-width="1"/>
  {acceleration_polyline('commanded_acceleration_mps2', '#dc2626')}
  {acceleration_polyline('actual_acceleration_mps2', '#059669')}
  <text x="{left-10}" y="{acceleration_y_position(acceleration_max)+4:.1f}"
        text-anchor="end">{acceleration_max:.0f}</text>
  <text x="{left-10}" y="{acceleration_y_position(0.0)+4:.1f}"
        text-anchor="end">0</text>
  <text x="{left-10}" y="{acceleration_y_position(-acceleration_max)+4:.1f}"
        text-anchor="end">-{acceleration_max:.0f}</text>
  <text x="{width/2}" y="{height-12}" text-anchor="middle">Elapsed time (s)</text>
  <text x="18" y="{height/2}" text-anchor="middle"
        transform="rotate(-90 18 {height/2})">Acceleration (m/s²)</text>
</svg>
<h2>Summary</h2>
<table>{rows}</table>
<h2>Section splits</h2>
<table>
<tr><th>Lap</th><th>Section</th><th>Lap time</th><th>Section duration</th></tr>
{split_rows}
</table>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description="render one GA episode speed diagnostic")
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--candidate-id")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    candidate_id, metrics = load_candidate_metrics(
        args.run_dir / "run.sqlite3", args.candidate_id
    )
    if not metrics.get("diagnostic_trace"):
        raise SystemExit(
            "selected episode has no diagnostic_trace; run it with the updated monitor first"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render_html(candidate_id, metrics), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
