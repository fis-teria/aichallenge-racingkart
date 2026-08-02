from __future__ import annotations

import argparse
import csv
import json
import sqlite3
import statistics
import time
from pathlib import Path
from typing import Any


def is_valid_timed_lap(metrics: dict[str, Any]) -> bool:
    if not metrics.get("completed") or metrics.get("lap_time_seconds") is None:
        return False
    first_section = next(
        (
            split for split in metrics.get("section_splits", [])
            if int(split.get("section", -1)) == 1
        ),
        None,
    )
    # A timer close to zero at section 1 means AWSIM reset the lap clock after
    # the vehicle had already covered the standing-start section.
    return (
        first_section is not None
        and float(first_section.get("lap_time_seconds") or 0.0) >= 1.0
    )


def validated_lap_time(candidate: dict[str, Any]) -> float | None:
    values = [
        float(episode["lap_time_seconds"])
        for episode in candidate["episodes"]
        if is_valid_timed_lap(episode)
    ]
    return statistics.median(values) if values else None


def validated_metric(candidate: dict[str, Any], name: str) -> float | None:
    values = [
        float(episode[name])
        for episode in candidate["episodes"]
        if is_valid_timed_lap(episode) and episode.get(name) is not None
    ]
    return statistics.median(values) if values else None


def _read_rows(
    database: Path,
) -> tuple[list[dict[str, Any]], dict[str, Any] | None, dict[str, Any] | None]:
    connection = sqlite3.connect(database, timeout=10)
    connection.row_factory = sqlite3.Row
    try:
        rows = connection.execute(
            """
            SELECT c.candidate_id, c.generation, c.genome_json, c.fitness, c.status,
                   e.metrics_json, e.repeat_index
            FROM candidates c
            LEFT JOIN episodes e ON e.candidate_id = c.candidate_id
            ORDER BY c.generation, c.candidate_id, e.repeat_index
            """
        ).fetchall()
    finally:
        connection.close()

    candidates: dict[str, dict[str, Any]] = {}
    for row in rows:
        item = candidates.setdefault(
            row["candidate_id"],
            {
                "id": row["candidate_id"],
                "generation": row["generation"],
                "fitness": row["fitness"],
                "status": row["status"],
                "genome": json.loads(row["genome_json"]),
                "episodes": [],
            },
        )
        if row["metrics_json"]:
            item["episodes"].append(json.loads(row["metrics_json"]))
    completed = [
        item for item in candidates.values()
        if item["fitness"] is not None and item["episodes"]
    ]
    fitness_best = min(completed, key=lambda item: item["fitness"]) if completed else None
    lap_candidates = [item for item in completed if validated_lap_time(item) is not None]
    lap_best = min(
        lap_candidates,
        key=lambda item: validated_lap_time(item) or float("inf"),
    ) if lap_candidates else None
    return list(candidates.values()), fitness_best, lap_best


def _resolve_base_path(run_dir: Path, config: dict[str, Any]) -> Path | None:
    raw = config.get("evaluator", {}).get("path_optimization", {}).get("base_csv_path")
    if not raw:
        return None
    path = Path(raw)
    if path.exists():
        return path
    if str(path).startswith("/aichallenge/"):
        repository = run_dir.resolve().parents[4]
        translated = repository / str(path).removeprefix("/")
        if translated.exists():
            return translated
    return None


def _load_xy(path: Path | None) -> list[list[float]]:
    if path is None or not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return [[float(row["x"]), float(row["y"])] for row in csv.DictReader(stream)]


def build_data(run_dir: Path) -> dict[str, Any]:
    config = json.loads((run_dir / "experiment_resolved.json").read_text(encoding="utf-8"))
    candidates, fitness_best, lap_best = _read_rows(run_dir / "run.sqlite3")
    search_space = config.get("search_space", {})
    generation_rows: list[dict[str, Any]] = []
    for generation in sorted({item["generation"] for item in candidates}):
        current = [
            item for item in candidates
            if item["generation"] == generation and item["fitness"] is not None
        ]
        lap_times = [
            value for item in current
            if (value := validated_lap_time(item)) is not None
        ]
        standing_times = [
            value for item in current
            if (value := validated_metric(item, "standing_lap_time_seconds")) is not None
        ]
        flying_times = [
            value for item in current
            if (value := validated_metric(item, "flying_lap_time_seconds")) is not None
        ]
        fitnesses = [float(item["fitness"]) for item in current]
        generation_rows.append(
            {
                "generation": generation,
                "complete": len(current),
                "total": sum(item["generation"] == generation for item in candidates),
                "best_fitness": min(fitnesses) if fitnesses else None,
                "mean_fitness": sum(fitnesses) / len(fitnesses) if fitnesses else None,
                "best_lap": min(lap_times) if lap_times else None,
                "mean_lap": sum(lap_times) / len(lap_times) if lap_times else None,
                "best_standing_lap": min(standing_times) if standing_times else None,
                "best_flying_lap": min(flying_times) if flying_times else None,
            }
        )

    gene_names = list(search_space)
    gene_heatmap = []
    for generation in sorted({item["generation"] for item in candidates}):
        current = [
            item for item in candidates
            if item["generation"] == generation and item["fitness"] is not None
        ]
        values = []
        for name in gene_names:
            bounds = search_space[name]
            width = float(bounds["max"]) - float(bounds["min"])
            normalized = [
                (float(item["genome"][name]) - float(bounds["min"])) / width
                for item in current
            ]
            values.append(sum(normalized) / len(normalized) if normalized else None)
        gene_heatmap.append({"generation": generation, "values": values})

    def representative_metrics(candidate: dict[str, Any] | None) -> tuple[float | None, dict[str, Any]]:
        lap = validated_lap_time(candidate) if candidate else None
        metrics = min(
            (
                episode for episode in candidate["episodes"]
                if is_valid_timed_lap(episode)
            ),
            key=lambda episode: abs(float(episode["lap_time_seconds"]) - float(lap)),
            default={},
        ) if candidate else {}
        return lap, metrics

    def trace_data(metrics: dict[str, Any]) -> list[dict[str, Any]]:
        return [
            {
                "t": item.get("elapsed_seconds"),
                "actual": None if item.get("actual_speed_mps") is None else item["actual_speed_mps"] * 3.6,
                "target": None if item.get("target_speed_mps") is None else item["target_speed_mps"] * 3.6,
                "limit": None if item.get("curvature_speed_limit_mps") is None else item["curvature_speed_limit_mps"] * 3.6,
                "commanded_acceleration": item.get("commanded_acceleration_mps2"),
                "actual_acceleration": item.get("actual_acceleration_mps2"),
                "accel_cmd": item.get("accel_cmd"),
                "brake_cmd": item.get("brake_cmd"),
            }
            for item in metrics.get("diagnostic_trace", [])
        ]

    best_lap, best_metrics = representative_metrics(lap_best)
    completed = [item for item in candidates if item["fitness"] is not None]
    latest_generation = max((item["generation"] for item in completed), default=None)
    latest_candidates = [
        item for item in completed
        if item["generation"] == latest_generation and validated_lap_time(item) is not None
    ]
    latest_lap_best = min(
        latest_candidates,
        key=lambda item: validated_lap_time(item) or float("inf"),
        default=None,
    )
    latest_lap, latest_metrics = representative_metrics(latest_lap_best)
    comparison_generations = (
        [value for value in (latest_generation - 1, latest_generation) if value >= 0]
        if latest_generation is not None else []
    )
    generation_comparison = []
    for generation in comparison_generations:
        rows = []
        for item in candidates:
            if item["generation"] != generation:
                continue
            rows.append({
                "id": item["id"],
                "status": item["status"],
                "fitness": item["fitness"],
                "standing_lap": validated_metric(item, "standing_lap_time_seconds"),
                "flying_lap": validated_metric(item, "flying_lap_time_seconds"),
                "genes": {
                    name: item["genome"].get(name)
                    for name in gene_names
                },
            })
        rows.sort(key=lambda item: (
            item["flying_lap"] is None,
            item["flying_lap"] if item["flying_lap"] is not None else float("inf"),
            item["id"],
        ))
        generation_comparison.append({"generation": generation, "rows": rows})
    if len(generation_comparison) == 2:
        previous_rows = [
            row for row in generation_comparison[0]["rows"]
            if row["flying_lap"] is not None
        ]
        for row in generation_comparison[1]["rows"]:
            row["reference_id"] = None
            row["lap_delta"] = None
            row["gene_deltas"] = {}
            row["top_changed_genes"] = []
            if row["flying_lap"] is None or not previous_rows:
                continue

            def normalized_gene_distance(reference: dict[str, Any]) -> float:
                differences = []
                for name in gene_names:
                    current_value = row["genes"].get(name)
                    reference_value = reference["genes"].get(name)
                    bounds = search_space.get(name, {})
                    width = float(bounds.get("max", 0.0)) - float(bounds.get("min", 0.0))
                    if current_value is None or reference_value is None or width <= 0.0:
                        continue
                    differences.append(
                        ((float(current_value) - float(reference_value)) / width) ** 2
                    )
                return sum(differences) / max(1, len(differences))

            reference = min(previous_rows, key=normalized_gene_distance)
            row["reference_id"] = reference["id"]
            row["lap_delta"] = float(row["flying_lap"]) - float(reference["flying_lap"])
            normalized_changes = []
            for name in gene_names:
                value = row["genes"].get(name)
                reference_value = reference["genes"].get(name)
                if value is None or reference_value is None:
                    continue
                delta = float(value) - float(reference_value)
                row["gene_deltas"][name] = delta
                bounds = search_space.get(name, {})
                width = float(bounds.get("max", 0.0)) - float(bounds.get("min", 0.0))
                if width > 0.0:
                    normalized_changes.append((abs(delta) / width, name))
            if row["lap_delta"] < 0.0:
                row["top_changed_genes"] = [
                    name for _, name in sorted(normalized_changes, reverse=True)[:3]
                ]
    candidate_path = Path(best_metrics.get("candidate_path", "")) if lap_best else None
    if candidate_path and not candidate_path.exists() and str(candidate_path).startswith("/aichallenge/"):
        candidate_path = run_dir.resolve().parents[4] / str(candidate_path).removeprefix("/")
    trace = trace_data(best_metrics)
    return {
        "run_id": run_dir.name,
        "updated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "generations": generation_rows,
        "gene_names": gene_names,
        "gene_heatmap": gene_heatmap,
        "generation_comparison": generation_comparison,
        "baseline_path": _load_xy(_resolve_base_path(run_dir, config)),
        "best_path": _load_xy(candidate_path),
        "trace": trace,
        "latest_trace": trace_data(latest_metrics),
        "latest_best": {
            "generation": latest_generation,
            "id": latest_lap_best["id"] if latest_lap_best else None,
            "lap": latest_lap,
            "standing_lap": latest_metrics.get("standing_lap_time_seconds"),
            "flying_lap": latest_metrics.get("flying_lap_time_seconds"),
        },
        "best": {
            "id": lap_best["id"] if lap_best else None,
            "fitness": lap_best["fitness"] if lap_best else None,
            "lap": best_lap,
            "standing_lap": best_metrics.get("standing_lap_time_seconds"),
            "flying_lap": best_metrics.get("flying_lap_time_seconds"),
            "valid_repeats": (
                sum(is_valid_timed_lap(episode) for episode in lap_best["episodes"])
                if lap_best else 0
            ),
            "excluded_laps": sum(
                episode.get("completed", False) and not is_valid_timed_lap(episode)
                for item in candidates for episode in item["episodes"]
            ),
            "elapsed": best_metrics.get("elapsed_seconds"),
            "fitness_best_id": fitness_best["id"] if fitness_best else None,
            "fitness_best": fitness_best["fitness"] if fitness_best else None,
            "max_speed": (
                best_metrics.get("actual_speed_max_mps", 0) * 3.6 if lap_best else None
            ),
            "mean_speed": (
                best_metrics.get("distance_average_speed_mps", 0) * 3.6 if lap_best else None
            ),
            "speed_error": (
                best_metrics.get("absolute_speed_error_mean_mps", 0) * 3.6 if lap_best else None
            ),
            "path_penalty": (
                lap_best["fitness"] - float(best_metrics.get("lap_time_seconds", lap_best["fitness"]))
                if lap_best else None
            ),
        },
    }


def render_html(data: dict[str, Any], refresh_seconds: int = 15) -> str:
    payload = json.dumps(data, ensure_ascii=False).replace("</", "<\\/")
    return f"""<!doctype html>
<html lang="ja"><meta charset="utf-8">
<meta http-equiv="refresh" content="{refresh_seconds}">
<title>GA progress - {data['run_id']}</title>
<style>
:root {{ color-scheme:dark; --bg:#08111f; --card:#101c2e; --grid:#29405f; --text:#e5eefc; --muted:#91a4bf; }}
* {{ box-sizing:border-box }} body {{ margin:0; padding:20px; background:var(--bg); color:var(--text); font-family:system-ui,sans-serif }}
h1 {{ margin:0 0 4px }} .sub {{ color:var(--muted); margin-bottom:14px }} .cards {{ display:grid;grid-template-columns:repeat(6,minmax(120px,1fr));gap:10px;margin-bottom:12px }}
.metric,.panel {{ background:var(--card);border:1px solid #1f3552;border-radius:10px;padding:12px }} .metric b {{ display:block;font-size:1.35rem;margin-top:4px }}
.grid {{ display:grid;grid-template-columns:1fr 1fr;gap:12px }} .panel h2 {{ font-size:1rem;margin:0 0 8px }} .panel h2.trace-subtitle {{margin-top:16px;padding-top:12px;border-top:1px solid #29405f}} canvas {{ width:100%;height:310px }} canvas.compact {{height:150px;margin-top:8px}}
#speed,#latestSpeed {{height:220px}} #actuation,#latestActuation {{height:90px;margin-top:4px}}
.help {{ color:var(--muted);font-size:.88rem;line-height:1.5;margin-top:6px }} @media(max-width:900px) {{.cards{{grid-template-columns:repeat(2,1fr)}}.grid{{grid-template-columns:1fr}}}}
.wide {{grid-column:1/-1}} .table-wrap {{overflow:auto;max-height:560px;border:1px solid #29405f;border-radius:7px}}
table {{border-collapse:separate;border-spacing:0;width:max-content;min-width:100%;font-size:.78rem}}
th,td {{padding:6px 8px;border-right:1px solid #233955;border-bottom:1px solid #233955;white-space:nowrap;text-align:right}}
th {{position:sticky;top:0;background:#17263b;color:#bcd0ea;z-index:2}} td:first-child,th:first-child {{position:sticky;left:0;text-align:left;background:#142238;z-index:1}}
th:first-child {{z-index:3}} tr.generation-start td {{border-top:3px solid #60a5fa}} td.pending {{color:var(--muted)}}
tr.improved td:first-child {{color:#6ee7b7}} td.gene-candidate {{background:#123d38;color:#a7f3d0;font-weight:650}} .delta-good {{color:#6ee7b7}} .delta-bad {{color:#fca5a5}} .gene-delta {{display:block;color:#93a9c5;font-size:.68rem}}
</style>
<h1>GA Progress Dashboard</h1><div class="sub" id="subtitle"></div>
<div class="cards" id="cards"></div>
<div class="grid">
 <section class="panel"><h2>世代ごとの改善</h2><canvas id="progress"></canvas><div class="help">紫がFitness、橙が停止スタートの1周目、緑が加速済みの2周目。各世代の検証済み最小値を表示。</div></section>
 <section class="panel">
  <h2>最速ラップ個体の速度追従</h2>
  <canvas id="speed"></canvas><canvas id="actuation" class="compact"></canvas>
  <div class="help">上段は速度、下段は目標加速度と実測加速度（m/s²）。横軸はどちらも評価開始からの経過秒数。</div>
  <h2 id="latestTitle" class="trace-subtitle">探索中世代の最速個体</h2>
  <canvas id="latestSpeed"></canvas><canvas id="latestActuation" class="compact"></canvas>
  <div class="help">現在評価が完了している最新世代だけから最速個体を選択。新しい診断項目の確認に使用。</div>
 </section>
 <section class="panel"><h2>経路の変化</h2><canvas id="path"></canvas><div class="help">灰色が元経路、緑が最良経路。大きくギザギザせず、コーナーを滑らかに外→内→外へ通るのが理想。</div></section>
 <section class="panel"><h2>遺伝子の収束</h2><canvas id="genes"></canvas><div class="help">横が世代、縦が遺伝子。色が世代間で変わらなくなると収束。早すぎる収束は探索不足のサイン。</div></section>
 <section class="panel wide"><h2>N-1 / N世代 個体別Lapタイムと遺伝子</h2><div class="table-wrap" id="generationTableWrap"><table id="generationTable"></table></div><div class="help">N世代は遺伝子的に最も近いN-1個体と比較。Δ2周目が負なら短縮。短縮行の★は正規化変化量が大きい遺伝子上位3つで、改善要因の候補（因果の確定ではない）。</div></section>
</div>
<script>
const D={payload};
const fmt=(v,n=2)=>v==null?'—':Number(v).toFixed(n);
subtitle.textContent=`run ${{D.run_id}} / 更新 ${{D.updated}} / 完了 ${{D.generations.reduce((a,g)=>a+g.complete,0)}} 個体 / タイマー欠測除外 ${{D.best.excluded_laps||0}} ラップ`;
const b=D.best; cards.innerHTML=[
 ['最速ラップ個体',b.id||'—'],['重み付きタイム',fmt(b.lap)+' s'],['2周目',fmt(b.flying_lap)+' s'],
 ['1周目',fmt(b.standing_lap)+' s'],['最高速度',fmt(b.max_speed,1)+' km/h'],['Fitness最良',fmt(b.fitness_best)]
].map(x=>`<div class="metric">${{x[0]}}<b>${{x[1]}}</b></div>`).join('');
const lb=D.latest_best||{{}}; latestTitle.textContent=`探索中 第${{lb.generation??'—'}}世代の最速: ${{lb.id||'—'}} / 2周目 ${{fmt(lb.flying_lap)}} s`;
function setup(id){{const c=document.getElementById(id),r=devicePixelRatio||1;c.width=c.clientWidth*r;c.height=c.clientHeight*r;const x=c.getContext('2d');x.scale(r,r);return [x,c.clientWidth,c.clientHeight]}}
function chart(id,series,xKey){{const [c,w,h]=setup(id),m={{l:48,r:18,t:18,b:35}},vals=series.flatMap(s=>s.data.map(p=>p[1])).filter(Number.isFinite),xvals=series.flatMap(s=>s.data.map(p=>p[0])).filter(Number.isFinite);if(!vals.length)return;
 const lo=Math.min(...vals),hi=Math.max(...vals),pad=Math.max((hi-lo)*.12,.05),y0=lo-pad,y1=hi+pad,x0=Math.min(0,...xvals),x1=Math.max(...xvals,x0+1);
 c.strokeStyle='#29405f';c.fillStyle='#91a4bf';c.font='11px sans-serif';for(let i=0;i<6;i++){{let y=m.t+(h-m.t-m.b)*i/5;c.beginPath();c.moveTo(m.l,y);c.lineTo(w-m.r,y);c.stroke();c.fillText((y1-(y1-y0)*i/5).toFixed(1),4,y+4)}}
 series.forEach(s=>{{c.strokeStyle=s.color;c.lineWidth=2;c.beginPath();s.data.forEach((p,i)=>{{let x=m.l+(w-m.l-m.r)*(p[0]-x0)/(x1-x0),y=m.t+(h-m.t-m.b)*(y1-p[1])/(y1-y0);i?c.lineTo(x,y):c.moveTo(x,y)}});c.stroke()}});
 series.forEach((s,i)=>{{c.fillStyle=s.color;c.fillRect(m.l+i*145,h-18,12,3);c.fillStyle='#e5eefc';c.fillText(s.name,m.l+18+i*145,h-14)}})}}
const G=D.generations;
chart('progress',[{{name:'Fitness',color:'#c084fc',data:G.filter(g=>g.best_fitness!=null).map(g=>[g.generation,g.best_fitness])}},{{name:'1周目(s)',color:'#fb923c',data:G.filter(g=>g.best_standing_lap!=null).map(g=>[g.generation,g.best_standing_lap])}},{{name:'2周目(s)',color:'#34d399',data:G.filter(g=>g.best_flying_lap!=null).map(g=>[g.generation,g.best_flying_lap])}}]);
chart('speed',[{{name:'実速度',color:'#38bdf8',data:D.trace.filter(x=>x.actual!=null).map(x=>[x.t,x.actual])}},{{name:'目標',color:'#fb923c',data:D.trace.filter(x=>x.target!=null).map(x=>[x.t,x.target])}}]);
chart('actuation',[{{name:'目標加速度',color:'#fb923c',data:D.trace.filter(x=>x.commanded_acceleration!=null).map(x=>[x.t,x.commanded_acceleration])}},{{name:'実測加速度',color:'#34d399',data:D.trace.filter(x=>x.actual_acceleration!=null).map(x=>[x.t,x.actual_acceleration])}}]);
const LT=D.latest_trace||[];
chart('latestSpeed',[{{name:'実速度',color:'#38bdf8',data:LT.filter(x=>x.actual!=null).map(x=>[x.t,x.actual])}},{{name:'目標',color:'#fb923c',data:LT.filter(x=>x.target!=null).map(x=>[x.t,x.target])}}]);
chart('latestActuation',[{{name:'目標加速度',color:'#fb923c',data:LT.filter(x=>x.commanded_acceleration!=null).map(x=>[x.t,x.commanded_acceleration])}},{{name:'実測加速度',color:'#34d399',data:LT.filter(x=>x.actual_acceleration!=null).map(x=>[x.t,x.actual_acceleration])}}]);
function pathChart(){{const [c,w,h]=setup('path'),all=D.baseline_path.concat(D.best_path);if(!all.length)return;let xs=all.map(p=>p[0]),ys=all.map(p=>p[1]),x0=Math.min(...xs),x1=Math.max(...xs),y0=Math.min(...ys),y1=Math.max(...ys),pad=20,s=Math.min((w-2*pad)/(x1-x0),(h-2*pad)/(y1-y0));function draw(a,color,width){{c.strokeStyle=color;c.lineWidth=width;c.beginPath();a.forEach((p,i)=>{{let x=pad+(p[0]-x0)*s,y=h-pad-(p[1]-y0)*s;i?c.lineTo(x,y):c.moveTo(x,y)}});c.stroke()}}draw(D.baseline_path,'#64748b',4);draw(D.best_path,'#34d399',2)}}pathChart();
function heat(){{const [c,w,h]=setup('genes'),R=D.gene_heatmap,N=D.gene_names.length;if(!R.length)return;let l=185,t=12,cw=(w-l-8)/R.length,ch=(h-t-8)/N;c.font='9px sans-serif';D.gene_names.forEach((n,i)=>{{c.fillStyle='#91a4bf';c.fillText(n.replace('path_offset_','path_'),3,t+(i+.75)*ch);}});R.forEach((r,x)=>r.values.forEach((v,y)=>{{if(v==null)return;c.fillStyle=`hsl(${{220-v*190}} 80% 55%)`;c.fillRect(l+x*cw,t+y*ch,Math.max(cw-1,1),Math.max(ch-1,1))}}))}}heat();
function generationTable(){{const table=document.getElementById('generationTable'),groups=D.generation_comparison||[],names=D.gene_names||[];
 const head=['世代 / 個体','状態','N-1比較元','Δ2周目(s)','Fitness','1周目(s)','2周目(s)',...names];
 const rows=groups.flatMap(group=>group.rows.map((row,index)=>{{const status=row.flying_lap==null?(row.status==='complete'?'Lap欠測':'評価中'):'完了',improved=row.lap_delta!=null&&row.lap_delta<0,top=new Set(row.top_changed_genes||[]);
  const geneCells=names.map(name=>{{const delta=(row.gene_deltas||{{}})[name],star=top.has(name),cls=star?'gene-candidate':'';return `<td class="${{cls}}">${{star?'★ ':''}}${{fmt(row.genes[name],4)}}${{delta==null?'':`<span class="gene-delta">Δ ${{delta>=0?'+':''}}${{fmt(delta,4)}}</span>`}}</td>`}}).join('');
  return `<tr class="${{index===0?'generation-start ':''}}${{improved?'improved':''}}"><td>N=${{group.generation}} / ${{row.id}}</td><td class="${{status==='完了'?'':'pending'}}">${{status}}</td><td>${{row.reference_id||'—'}}</td><td class="${{row.lap_delta==null?'':improved?'delta-good':'delta-bad'}}">${{row.lap_delta==null?'—':`${{row.lap_delta>=0?'+':''}}${{fmt(row.lap_delta,3)}}`}}</td><td>${{fmt(row.fitness,3)}}</td><td>${{fmt(row.standing_lap)}}</td><td>${{fmt(row.flying_lap)}}</td>${{geneCells}}</tr>`}}));
 table.innerHTML=`<thead><tr>${{head.map(value=>`<th>${{value}}</th>`).join('')}}</tr></thead><tbody>${{rows.join('')}}</tbody>`;
}}generationTable();
const scrollKey=`ga-dashboard-scroll:${{D.run_id}}`;
function saveScrollPosition(){{const wrap=document.getElementById('generationTableWrap'),state={{windowY:window.scrollY,tableX:wrap?.scrollLeft||0,tableY:wrap?.scrollTop||0}};try{{sessionStorage.setItem(scrollKey,JSON.stringify(state))}}catch(_error){{window.name=JSON.stringify({{key:scrollKey,state}})}}}}
function restoreScrollPosition(){{let state=null;try{{state=JSON.parse(sessionStorage.getItem(scrollKey)||'null')}}catch(_error){{try{{const saved=JSON.parse(window.name||'null');if(saved?.key===scrollKey)state=saved.state}}catch(_ignored){{}}}}if(!state)return;const wrap=document.getElementById('generationTableWrap');if(wrap){{wrap.scrollLeft=state.tableX||0;wrap.scrollTop=state.tableY||0}}window.scrollTo(0,state.windowY||0)}}
history.scrollRestoration='manual';window.addEventListener('pagehide',saveScrollPosition);window.addEventListener('beforeunload',saveScrollPosition);requestAnimationFrame(()=>requestAnimationFrame(restoreScrollPosition));
</script></html>"""


def write_dashboard(run_dir: Path, output: Path, refresh_seconds: int) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(render_html(build_data(run_dir), refresh_seconds), encoding="utf-8")
    temporary.replace(output)


def main() -> None:
    parser = argparse.ArgumentParser(description="render a live GA progress dashboard")
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--interval", type=int, default=15)
    args = parser.parse_args()
    output = args.output or args.run_dir / "dashboard.html"
    while True:
        write_dashboard(args.run_dir, output, args.interval)
        print(output, flush=True)
        if not args.watch:
            break
        time.sleep(max(args.interval, 2))


if __name__ == "__main__":
    main()
