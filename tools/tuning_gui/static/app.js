const state = {
  app: null,
  currentFile: null,
  currentContent: "",
  currentKind: "text",
  editorMode: "text",
  structuredRows: [],
  structuredDirty: false,
  structuredDescriptionsDirty: false,
  commandTimer: null,
  motion: {
    reports: [],
    selectedRunId: null,
    selectedDomain: null,
    data: null,
  },
  pathEditor: {
    data: null,
    points: [],
    selectedIndex: null,
    mode: "move",
    image: null,
    view: { scale: 1, offsetX: 0, offsetY: 0 },
    draggingPoint: false,
    panning: false,
    lastPointer: null,
    dirty: false,
    undoPoints: null,
    selectedRangeIndices: [],
    draggingRange: false,
    rangeStart: null,
    rangeCurrent: null,
    draggingRangeMove: false,
    rangeMoveStartWorld: null,
    rangeMoveOriginalPoints: null,
  },
};

const $ = (id) => document.getElementById(id);
const THEME_KEY = "tuning-gui-theme";
const XML_PRIORITY_COLUMNS = [
  "name",
  "default",
  "value",
  "to",
  "from",
  "args",
  "file",
  "pkg",
  "exec",
  "type",
  "if",
  "unless",
  "output",
  "namespace",
  "plugin",
  "cwd",
  "cmd",
];

function applyTheme(theme) {
  const normalized = theme === "dark" ? "dark" : "light";
  document.documentElement.dataset.theme = normalized;
  localStorage.setItem(THEME_KEY, normalized);
  const toggle = $("themeToggle");
  const label = $("themeLabel");
  if (toggle) toggle.checked = normalized === "dark";
  if (label) label.textContent = normalized === "dark" ? "Dark" : "Light";
}

function currentTheme() {
  return document.documentElement.dataset.theme || localStorage.getItem(THEME_KEY) || "light";
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.error || response.statusText);
  }
  return data;
}

function toast(message) {
  const el = $("toast");
  el.textContent = message;
  el.hidden = false;
  clearTimeout(el._timer);
  el._timer = setTimeout(() => {
    el.hidden = true;
  }, 4200);
}

function fmtSec(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "-";
  return `${Number(value).toFixed(3)}s`;
}

function fmtDate(value) {
  if (!value) return "-";
  return String(value).replace("T", " ").replace(/\+.*/, "");
}

function fmtNum(value, digits = 3, suffix = "") {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "-";
  return `${Number(value).toFixed(digits)}${suffix}`;
}

function valueOr(value, fallback) {
  return value === null || value === undefined ? fallback : value;
}

async function loadState() {
  const data = await api("/api/state");
  state.app = data;
  renderState();
  if (!state.currentFile && data.files.length) {
    await openFile(data.files[0].path);
  }
  await refreshHistory();
  await refreshCommand();
}

function renderState() {
  const data = state.app;
  $("repoRoot").textContent = data.repo_root;
  $("xmlDefault").textContent = data.xml_default_control_method;
  $("lastBuild").textContent = fmtDate(data.last_build_at);
  $("dirtySince").textContent = fmtDate(data.dirty_since);

  const methodSelect = $("controlMethod");
  methodSelect.innerHTML = "";
  for (const method of data.methods) {
    const option = document.createElement("option");
    option.value = method;
    option.textContent = method;
    option.selected = method === data.selected_control_method;
    methodSelect.appendChild(option);
  }

  const fileList = $("fileList");
  fileList.innerHTML = "";
  for (const file of data.files) {
    const button = document.createElement("button");
    button.className = `file-item ${state.currentFile === file.path ? "active" : ""}`;
    button.innerHTML = `<strong>${escapeHtml(file.label)}</strong><small>${escapeHtml(file.path)}</small>`;
    button.addEventListener("click", () => openFile(file.path));
    fileList.appendChild(button);
  }

  const presetSelect = $("presetSelect");
  presetSelect.innerHTML = "";
  for (const preset of data.presets) {
    const option = document.createElement("option");
    option.value = preset.name;
    option.textContent = `${preset.name} (${preset.control_method})`;
    option.title = preset.note || "";
    presetSelect.appendChild(option);
  }

  renderStatus(data.command);
}

function renderStatus(commandState) {
  const running = commandState && commandState.running;
  const dirty = Boolean(state.app && state.app.dirty_since);
  const pill = $("statusPill");
  pill.className = "status-pill";
  if (running) {
    pill.textContent = "running";
    pill.classList.add("running");
  } else if (dirty) {
    pill.textContent = "needs build";
    pill.classList.add("dirty");
  } else {
    pill.textContent = "ready";
    pill.classList.add("ready");
  }
  const locked = Boolean(running);
  const pathActive = state.editorMode === "path";
  for (const id of ["saveControl", "saveControlBuild", "restorePreset", "pathSave"]) {
    $(id).disabled = locked;
  }
  for (const id of ["validateFile", "diffFile", "saveFile", "saveFileBuild"]) {
    $(id).disabled = locked || pathActive;
  }
  $("stopCommand").disabled = !locked;
  $("runHeadless").disabled = locked;
  $("npcCount").disabled = locked;
}

function setCommandLog(text) {
  const log = $("commandLog");
  log.textContent = text;
  log.scrollTop = log.scrollHeight;
  requestAnimationFrame(() => {
    log.scrollTop = log.scrollHeight;
  });
}

async function openFile(path) {
  const data = await api(`/api/file?path=${encodeURIComponent(path)}`);
  state.currentFile = data.path;
  state.currentContent = data.content;
  state.currentKind = fileKind(data.path);
  state.structuredDirty = false;
  state.structuredDescriptionsDirty = false;
  $("fileTitle").textContent = fileLabel(data.path);
  $("filePath").textContent = data.path;
  $("fileEditor").value = data.content;
  $("diffOutput").textContent = "";
  if (state.currentKind === "yaml" || state.currentKind === "xml") {
    state.editorMode = "table";
    await loadStructuredRows();
  } else {
    state.editorMode = "text";
    state.structuredRows = [];
  }
  renderEditorMode();
  renderState();
}

function fileLabel(path) {
  const files = state.app && state.app.files ? state.app.files : [];
  const file = files.find((item) => item.path === path);
  return file ? file.label : path.split("/").pop();
}

function fileKind(path) {
  const lower = path.toLowerCase();
  if (lower.endsWith(".yaml") || lower.endsWith(".yml")) return "yaml";
  if (lower.endsWith(".xml")) return "xml";
  if (lower.endsWith(".csv")) return "csv";
  return "text";
}

async function loadStructuredRows() {
  const data = await api("/api/structured/parse", {
    method: "POST",
    body: JSON.stringify({ path: state.currentFile, content: $("fileEditor").value }),
  });
  state.currentKind = data.kind || state.currentKind;
  state.structuredRows = data.rows || [];
  renderStructuredRows();
}

function renderEditorMode() {
  const tableAvailable = state.currentKind === "yaml" || state.currentKind === "xml";
  const pathActive = state.editorMode === "path";
  $("tableMode").disabled = !tableAvailable || pathActive;
  $("applyStructured").disabled = pathActive || !tableAvailable || !state.structuredRows.length;
  $("validateFile").disabled = pathActive;
  $("diffFile").disabled = pathActive;
  $("saveFile").disabled = pathActive;
  $("saveFileBuild").disabled = pathActive;
  $("structuredEditor").hidden = !(tableAvailable && state.editorMode === "table");
  $("pathEditor").hidden = !pathActive;
  $("fileEditor").classList.toggle("hidden-editor", pathActive || (tableAvailable && state.editorMode === "table"));
  $("diffOutput").classList.toggle("hidden-editor", pathActive);
  $("tableMode").classList.toggle("active", tableAvailable && state.editorMode === "table");
  $("textMode").classList.toggle("active", !pathActive && (!tableAvailable || state.editorMode === "text"));
  $("pathMode").classList.toggle("active", pathActive);
  if (pathActive) {
    requestAnimationFrame(() => {
      fitPathView(false);
      drawPathEditor();
    });
  }
}

function renderStructuredRows() {
  if (state.currentKind === "xml") {
    renderXmlStructuredRows();
    return;
  }
  renderScalarStructuredRows();
}

function renderScalarStructuredRows() {
  $("structuredHead").innerHTML = `<tr>
    <th>line</th>
    <th>path</th>
    <th>name</th>
    <th>type</th>
    <th>description</th>
    <th>value</th>
  </tr>`;
  const tbody = $("structuredRows");
  const filter = $("structuredFilter").value.trim().toLowerCase();
  const rows = state.structuredRows.filter((row) => {
    const haystack = `${row.path || ""} ${row.name || ""} ${row.value || ""} ${row.label || ""} ${row.description || ""}`.toLowerCase();
    return !filter || haystack.includes(filter);
  });
  tbody.innerHTML = rows
    .map((row) => {
      return `<tr data-row-id="${escapeHtml(row.id)}">
        <td>${escapeHtml(valueOr(row.line, ""))}</td>
        <td>${escapeHtml(row.path || row.label || "")}</td>
        <td>${escapeHtml(row.name || "")}</td>
        <td class="structured-type">${escapeHtml(row.type || "")}</td>
        <td><input class="structured-description" data-row-id="${escapeHtml(row.id)}" value="${escapeHtml(valueOr(row.description, ""))}"></td>
        <td><input class="structured-value" data-row-id="${escapeHtml(row.id)}" value="${escapeHtml(valueOr(row.value, ""))}"></td>
      </tr>`;
    })
    .join("");
  $("structuredHint").textContent = state.structuredRows.length
    ? `${rows.length}/${state.structuredRows.length} rows - 表の値は保存前に本文へ反映されます`
    : "このファイルで表編集できる値は見つかりません";
  for (const input of tbody.querySelectorAll(".structured-value")) {
    input.addEventListener("input", () => {
      const row = state.structuredRows.find((item) => item.id === input.dataset.rowId);
      if (!row) return;
      row.value = input.value;
      state.structuredDirty = true;
    });
  }
  bindStructuredDescriptionInputs(tbody);
}

function renderXmlStructuredRows() {
  const tbody = $("structuredRows");
  const filter = $("structuredFilter").value.trim().toLowerCase();
  const columns = xmlColumns(state.structuredRows);
  $("structuredHead").innerHTML = `<tr>
    <th>line</th>
    <th>tag</th>
    <th>description</th>
    ${columns.map((column) => `<th>${escapeHtml(column)}</th>`).join("")}
  </tr>`;
  const rows = state.structuredRows.filter((row) => {
    const attrs = row.attrs || {};
    const haystack = `${row.path || ""} ${row.tag || ""} ${row.label || ""} ${row.description || ""} ${Object.keys(attrs).join(" ")} ${Object.values(attrs).join(" ")}`.toLowerCase();
    return !filter || haystack.includes(filter);
  });
  tbody.innerHTML = rows
    .map((row) => {
      const attrs = row.attrs || {};
      const cells = columns
        .map((column) => {
          if (!Object.prototype.hasOwnProperty.call(attrs, column)) {
            return "<td class=\"structured-empty\"></td>";
          }
          return `<td><input class="structured-value xml-attr-value" data-row-id="${escapeHtml(row.id)}" data-attr="${escapeHtml(column)}" value="${escapeHtml(valueOr(attrs[column], ""))}"></td>`;
        })
        .join("");
      return `<tr data-row-id="${escapeHtml(row.id)}">
        <td>${escapeHtml(valueOr(row.line, ""))}</td>
        <td class="structured-tag">${escapeHtml(row.tag || "")}</td>
        <td><input class="structured-description" data-row-id="${escapeHtml(row.id)}" value="${escapeHtml(valueOr(row.description, ""))}"></td>
        ${cells}
      </tr>`;
    })
    .join("");
  $("structuredHint").textContent = state.structuredRows.length
    ? `${rows.length}/${state.structuredRows.length} elements - XMLは要素1つを1行で表示しています`
    : "このXMLで表編集できる属性は見つかりません";
  for (const input of tbody.querySelectorAll(".xml-attr-value")) {
    input.addEventListener("input", () => {
      const row = state.structuredRows.find((item) => item.id === input.dataset.rowId);
      if (!row) return;
      row.attrs = row.attrs || {};
      row.attrs[input.dataset.attr] = input.value;
      state.structuredDirty = true;
    });
  }
  bindStructuredDescriptionInputs(tbody);
}

function bindStructuredDescriptionInputs(tbody) {
  for (const input of tbody.querySelectorAll(".structured-description")) {
    input.addEventListener("input", () => {
      const row = state.structuredRows.find((item) => item.id === input.dataset.rowId);
      if (!row) return;
      row.description = input.value;
      state.structuredDescriptionsDirty = true;
    });
  }
}

function xmlColumns(rows) {
  const seen = new Set();
  const attrs = [];
  for (const row of rows) {
    for (const attr of row.attr_order || Object.keys(row.attrs || {})) {
      if (!seen.has(attr)) {
        seen.add(attr);
        attrs.push(attr);
      }
    }
  }
  const priority = XML_PRIORITY_COLUMNS.filter((column) => seen.has(column));
  const extras = attrs.filter((column) => !XML_PRIORITY_COLUMNS.includes(column));
  return [...priority, ...extras];
}

async function applyStructuredRows(silent = false) {
  if (!(state.currentKind === "yaml" || state.currentKind === "xml")) return null;
  const data = await api("/api/structured/apply", {
    method: "POST",
    body: JSON.stringify({
      path: state.currentFile,
      content: $("fileEditor").value,
      rows: state.structuredRows,
    }),
  });
  $("fileEditor").value = data.content;
  state.currentContent = data.content;
  state.structuredDirty = false;
  state.structuredDescriptionsDirty = false;
  await loadStructuredRows();
  if (!silent) {
    toast(data.descriptions_changed ? "表の値とdescriptionを保存したよ" : "表の値を本文へ反映したよ");
  }
  return data;
}

async function openPathEditor(path = "") {
  state.editorMode = "path";
  renderEditorMode();
  const url = path ? `/api/path-editor?path=${encodeURIComponent(path)}` : "/api/path-editor";
  const data = await api(url);
  state.pathEditor.data = data;
  state.pathEditor.points = clonePathPoints(data.source.points || []);
  state.pathEditor.selectedIndex = null;
  state.pathEditor.dirty = false;
  state.pathEditor.undoPoints = null;
  clearPathRangeSelection(false);
  renderPathControls();
  await loadPathImage(data.map.image_url);
  fitPathView(true);
  renderPathStats();
  drawPathEditor();
}

function clonePathPoints(points) {
  return points.map((point, index) => ({
    index,
    s_m: Number(valueOr(point.s_m, 0)),
    x_m: Number(point.x_m),
    y_m: Number(point.y_m),
    psi_rad: Number(valueOr(point.psi_rad, 0)),
    kappa_radpm: Number(valueOr(point.kappa_radpm, 0)),
    vx_mps: Number(valueOr(point.vx_mps, 0)),
    ax_mps2: Number(valueOr(point.ax_mps2, 0)),
  }));
}

function renderPathControls() {
  const data = state.pathEditor.data;
  if (!data) return;
  const source = $("pathSource");
  source.innerHTML = "";
  for (const item of data.available_paths || []) {
    const option = document.createElement("option");
    option.value = item.path;
    option.textContent = item.name;
    option.selected = item.path === data.source.path;
    source.appendChild(option);
  }
  $("pathTarget").value = data.default_target_path || data.source.path;
  $("pathSwitchConfig").checked = true;
  $("pathAutoBuild").checked = false;
  renderPathModeButtons();
  renderPathSmoothButtons();
}

async function loadPathImage(url) {
  const image = new Image();
  image.src = `${url}${url.includes("?") ? "&" : "?"}t=${Date.now()}`;
  await new Promise((resolve, reject) => {
    image.onload = resolve;
    image.onerror = () => reject(new Error("map image load failed"));
  });
  state.pathEditor.image = image;
}

function resizePathCanvas() {
  const canvas = $("pathCanvas");
  const wrap = canvas.parentElement;
  const rect = wrap.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(rect.width));
  const height = Math.max(1, Math.floor(rect.height));
  if (canvas.width !== Math.floor(width * dpr) || canvas.height !== Math.floor(height * dpr)) {
    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(height * dpr);
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { width, height, ctx };
}

function fitPathView(force = true) {
  if (state.editorMode !== "path" || !state.pathEditor.image) return;
  const canvas = $("pathCanvas");
  const rect = canvas.parentElement.getBoundingClientRect();
  if (!force && rect.width <= 0) return;
  const image = state.pathEditor.image;
  const scale = Math.min(rect.width / image.width, rect.height / image.height) * 0.94;
  state.pathEditor.view = {
    scale,
    offsetX: (rect.width - image.width * scale) / 2,
    offsetY: (rect.height - image.height * scale) / 2,
  };
}

function zoomPathView(factor) {
  const canvas = $("pathCanvas");
  const rect = canvas.parentElement.getBoundingClientRect();
  const center = { x: rect.width / 2, y: rect.height / 2 };
  const before = canvasToImage(center);
  const view = state.pathEditor.view;
  view.scale = Math.max(0.05, Math.min(20, view.scale * factor));
  view.offsetX = center.x - before.x * view.scale;
  view.offsetY = center.y - before.y * view.scale;
  drawPathEditor();
}

function drawPathEditor() {
  if (state.editorMode !== "path") return;
  const { width, height, ctx } = resizePathCanvas();
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = getComputedStyle(document.documentElement).getPropertyValue("--panel-alt").trim();
  ctx.fillRect(0, 0, width, height);
  const image = state.pathEditor.image;
  if (!image) return;
  const { scale, offsetX, offsetY } = state.pathEditor.view;
  ctx.imageSmoothingEnabled = false;
  ctx.globalAlpha = currentTheme() === "dark" ? 0.72 : 0.9;
  ctx.drawImage(image, offsetX, offsetY, image.width * scale, image.height * scale);
  ctx.globalAlpha = 1;
  const canvasPoints = state.pathEditor.points.map((point) => imageToCanvas(worldToImage(point)));
  drawPathLine(ctx, canvasPoints);
  drawSelectedRangeLine(ctx, canvasPoints);
  drawPathPoints(ctx, canvasPoints);
  drawPathRangeRect(ctx);
}

function drawPathLine(ctx, canvasPoints) {
  if (canvasPoints.length < 2) return;
  ctx.save();
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#e44262";
  ctx.beginPath();
  ctx.moveTo(canvasPoints[0].x, canvasPoints[0].y);
  for (const point of canvasPoints.slice(1)) {
    ctx.lineTo(point.x, point.y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawPathPoints(ctx, canvasPoints) {
  ctx.save();
  const rangeIndices = new Set(state.pathEditor.selectedRangeIndices);
  canvasPoints.forEach((point, index) => {
    const selected = index === state.pathEditor.selectedIndex;
    const inRange = rangeIndices.has(index);
    ctx.beginPath();
    ctx.arc(point.x, point.y, selected ? 5 : inRange ? 4.5 : 3.5, 0, Math.PI * 2);
    ctx.fillStyle = selected ? "#ffcc4d" : inRange ? "#20b36b" : "#1d78ff";
    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = selected ? "#523600" : inRange ? "#063f26" : "#ffffff";
    ctx.stroke();
  });
  ctx.restore();
}

function drawSelectedRangeLine(ctx, canvasPoints) {
  const runs = selectedRangeRuns();
  if (!runs.length) return;
  ctx.save();
  ctx.lineWidth = 4;
  ctx.strokeStyle = "#20b36b";
  ctx.globalAlpha = 0.82;
  for (const run of runs) {
    if (run.length < 2) continue;
    ctx.beginPath();
    const first = canvasPoints[run[0]];
    ctx.moveTo(first.x, first.y);
    for (const index of run.slice(1)) {
      const point = canvasPoints[index];
      ctx.lineTo(point.x, point.y);
    }
    ctx.stroke();
  }
  ctx.restore();
}

function drawPathRangeRect(ctx) {
  if (!state.pathEditor.draggingRange || !state.pathEditor.rangeStart || !state.pathEditor.rangeCurrent) return;
  const rect = normalizedCanvasRect(state.pathEditor.rangeStart, state.pathEditor.rangeCurrent);
  ctx.save();
  ctx.fillStyle = "rgba(32, 179, 107, 0.12)";
  ctx.strokeStyle = "#20b36b";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 4]);
  ctx.fillRect(rect.x, rect.y, rect.width, rect.height);
  ctx.strokeRect(rect.x, rect.y, rect.width, rect.height);
  ctx.restore();
}

function worldToImage(point) {
  const map = state.pathEditor.data.map;
  const origin = map.origin;
  const resolution = map.resolution;
  const x = (Number(point.x_m) - origin[0]) / resolution;
  const y = map.height - (Number(point.y_m) - origin[1]) / resolution;
  return { x, y };
}

function imageToWorld(point) {
  const map = state.pathEditor.data.map;
  const origin = map.origin;
  const resolution = map.resolution;
  return {
    x_m: origin[0] + point.x * resolution,
    y_m: origin[1] + (map.height - point.y) * resolution,
  };
}

function imageToCanvas(point) {
  const view = state.pathEditor.view;
  return {
    x: point.x * view.scale + view.offsetX,
    y: point.y * view.scale + view.offsetY,
  };
}

function canvasToImage(point) {
  const view = state.pathEditor.view;
  return {
    x: (point.x - view.offsetX) / view.scale,
    y: (point.y - view.offsetY) / view.scale,
  };
}

function pathPointerPosition(event) {
  const rect = $("pathCanvas").getBoundingClientRect();
  return { x: event.clientX - rect.left, y: event.clientY - rect.top };
}

function nearestPathPoint(canvasPoint, maxDistance = 14) {
  let best = { index: null, distance: Infinity };
  state.pathEditor.points.forEach((point, index) => {
    const canvas = imageToCanvas(worldToImage(point));
    const distance = Math.hypot(canvas.x - canvasPoint.x, canvas.y - canvasPoint.y);
    if (distance < best.distance) best = { index, distance };
  });
  return best.distance <= maxDistance ? best.index : null;
}

function nearestPathSegment(canvasPoint) {
  const points = state.pathEditor.points;
  if (points.length < 2) return { index: points.length - 1, distance: Infinity };
  let best = { index: 0, distance: Infinity };
  for (let index = 0; index < points.length - 1; index += 1) {
    const a = imageToCanvas(worldToImage(points[index]));
    const b = imageToCanvas(worldToImage(points[index + 1]));
    const distance = distanceToSegment(canvasPoint, a, b);
    if (distance < best.distance) best = { index, distance };
  }
  return best;
}

function distanceToSegment(point, a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length2 = dx * dx + dy * dy;
  if (length2 === 0) return Math.hypot(point.x - a.x, point.y - a.y);
  const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2));
  const projection = { x: a.x + t * dx, y: a.y + t * dy };
  return Math.hypot(point.x - projection.x, point.y - projection.y);
}

function onPathPointerDown(event) {
  if (state.editorMode !== "path") return;
  event.preventDefault();
  const point = pathPointerPosition(event);
  if (shouldPanPathView(event)) {
    state.pathEditor.panning = true;
    state.pathEditor.lastPointer = point;
    $("pathCanvas").setPointerCapture(event.pointerId);
    return;
  }
  if (state.pathEditor.mode === "range") {
    beginPathRangeSelection(point, event.pointerId);
    return;
  }
  if (state.pathEditor.mode === "add") {
    insertPathPoint(point);
    return;
  }
  if (selectedPathRangeHit(point)) {
    beginPathRangeMove(point, event.pointerId);
    return;
  }
  const index = nearestPathPoint(point);
  if (index !== null) {
    state.pathEditor.selectedIndex = index;
    state.pathEditor.draggingPoint = true;
    $("pathCanvas").setPointerCapture(event.pointerId);
  } else {
    beginPathRangeSelection(point, event.pointerId);
  }
  renderPathStats();
  drawPathEditor();
}

function onPathPointerMove(event) {
  if (state.editorMode !== "path") return;
  const point = pathPointerPosition(event);
  if (state.pathEditor.draggingRange) {
    state.pathEditor.rangeCurrent = point;
    drawPathEditor();
  } else if (state.pathEditor.draggingRangeMove && state.pathEditor.rangeMoveStartWorld) {
    moveSelectedPathRange(point);
  } else if (state.pathEditor.draggingPoint && state.pathEditor.selectedIndex !== null) {
    clearPathSmoothUndo();
    const world = imageToWorld(canvasToImage(point));
    const selected = state.pathEditor.points[state.pathEditor.selectedIndex];
    selected.x_m = world.x_m;
    selected.y_m = world.y_m;
    state.pathEditor.dirty = true;
    renderPathStats();
    drawPathEditor();
  } else if (state.pathEditor.panning && state.pathEditor.lastPointer) {
    const previous = state.pathEditor.lastPointer;
    state.pathEditor.view.offsetX += point.x - previous.x;
    state.pathEditor.view.offsetY += point.y - previous.y;
    state.pathEditor.lastPointer = point;
    drawPathEditor();
  }
}

function onPathPointerUp(event) {
  if (state.editorMode !== "path") return;
  if (state.pathEditor.draggingRange) {
    state.pathEditor.rangeCurrent = pathPointerPosition(event);
    finishPathRangeSelection();
  }
  state.pathEditor.draggingPoint = false;
  state.pathEditor.panning = false;
  state.pathEditor.draggingRange = false;
  state.pathEditor.draggingRangeMove = false;
  state.pathEditor.rangeMoveStartWorld = null;
  state.pathEditor.rangeMoveOriginalPoints = null;
  state.pathEditor.lastPointer = null;
  try {
    $("pathCanvas").releasePointerCapture(event.pointerId);
  } catch {
    // pointer capture may already be released by the browser
  }
}

function shouldPanPathView(event) {
  return state.pathEditor.mode === "move" && (event.button === 1 || event.altKey || event.shiftKey);
}

function onPathWheel(event) {
  if (state.editorMode !== "path") return;
  event.preventDefault();
  const point = pathPointerPosition(event);
  const before = canvasToImage(point);
  const factor = event.deltaY < 0 ? 1.12 : 0.89;
  const view = state.pathEditor.view;
  view.scale = Math.max(0.05, Math.min(20, view.scale * factor));
  view.offsetX = point.x - before.x * view.scale;
  view.offsetY = point.y - before.y * view.scale;
  drawPathEditor();
}

function insertPathPoint(canvasPoint) {
  clearPathSmoothUndo();
  clearPathRangeSelection(false);
  const imagePoint = canvasToImage(canvasPoint);
  const world = imageToWorld(imagePoint);
  const segment = nearestPathSegment(canvasPoint);
  const insertAt = Math.max(0, segment.index + 1);
  const before = state.pathEditor.points[Math.max(0, insertAt - 1)] || {};
  const after = state.pathEditor.points[Math.min(state.pathEditor.points.length - 1, insertAt)] || before;
  const point = {
    index: insertAt,
    s_m: 0,
    x_m: world.x_m,
    y_m: world.y_m,
    psi_rad: 0,
    kappa_radpm: 0,
    vx_mps: averageNumeric(before.vx_mps, after.vx_mps),
    ax_mps2: averageNumeric(before.ax_mps2, after.ax_mps2),
  };
  state.pathEditor.points.splice(insertAt, 0, point);
  state.pathEditor.selectedIndex = insertAt;
  state.pathEditor.dirty = true;
  reindexPathPoints();
  renderPathStats();
  drawPathEditor();
}

function averageNumeric(a, b) {
  const av = Number.isFinite(Number(a)) ? Number(a) : 0;
  const bv = Number.isFinite(Number(b)) ? Number(b) : av;
  return (av + bv) / 2;
}

function deleteSelectedPathPoint() {
  const index = state.pathEditor.selectedIndex;
  if (index === null) return;
  if (state.pathEditor.points.length <= 2) {
    toast("2点未満にはできないよ");
    return;
  }
  clearPathSmoothUndo();
  clearPathRangeSelection(false);
  state.pathEditor.points.splice(index, 1);
  state.pathEditor.selectedIndex = Math.min(index, state.pathEditor.points.length - 1);
  state.pathEditor.dirty = true;
  reindexPathPoints();
  renderPathStats();
  drawPathEditor();
}

function reindexPathPoints() {
  state.pathEditor.points.forEach((point, index) => {
    point.index = index;
  });
}

function setPathMode(mode) {
  state.pathEditor.mode = mode;
  state.pathEditor.draggingRange = false;
  state.pathEditor.rangeStart = null;
  state.pathEditor.rangeCurrent = null;
  renderPathModeButtons();
  drawPathEditor();
}

function renderPathModeButtons() {
  $("pathSelectMode").classList.toggle("active", state.pathEditor.mode === "move");
  $("pathAddMode").classList.toggle("active", state.pathEditor.mode === "add");
  const rangeButton = $("pathRangeMode");
  if (rangeButton) rangeButton.classList.toggle("active", state.pathEditor.mode === "range");
}

function pathIsCircular() {
  const data = state.pathEditor.data;
  return Boolean(data && data.circular !== null && data.circular !== undefined ? data.circular : true);
}

function renderPathSmoothButtons() {
  $("pathUndoSmooth").disabled = !state.pathEditor.undoPoints;
  $("pathClearRange").disabled = !state.pathEditor.selectedRangeIndices.length;
}

function clearPathSmoothUndo() {
  if (!state.pathEditor.undoPoints) return;
  state.pathEditor.undoPoints = null;
  renderPathSmoothButtons();
}

function beginPathRangeSelection(point, pointerId) {
  state.pathEditor.rangeStart = point;
  state.pathEditor.rangeCurrent = point;
  state.pathEditor.draggingRange = true;
  state.pathEditor.selectedIndex = null;
  $("pathCanvas").setPointerCapture(pointerId);
  renderPathStats();
  drawPathEditor();
}

function finishPathRangeSelection() {
  const start = state.pathEditor.rangeStart;
  const current = state.pathEditor.rangeCurrent;
  state.pathEditor.draggingRange = false;
  state.pathEditor.rangeStart = null;
  state.pathEditor.rangeCurrent = null;
  if (!start || !current) return;
  const rect = normalizedCanvasRect(start, current);
  if (rect.width < 4 && rect.height < 4) {
    clearPathRangeSelection(true);
    return;
  }
  const indices = [];
  state.pathEditor.points.forEach((point, index) => {
    const canvas = imageToCanvas(worldToImage(point));
    if (
      canvas.x >= rect.x &&
      canvas.x <= rect.x + rect.width &&
      canvas.y >= rect.y &&
      canvas.y <= rect.y + rect.height
    ) {
      indices.push(index);
    }
  });
  state.pathEditor.selectedRangeIndices = indices;
  renderPathSmoothButtons();
  renderPathStats();
  drawPathEditor();
  toast(indices.length ? `${indices.length}点をrange選択したよ` : "range内に点がなかったよ");
}

function clearPathRangeSelection(showToast = true) {
  state.pathEditor.selectedRangeIndices = [];
  state.pathEditor.draggingRange = false;
  state.pathEditor.draggingRangeMove = false;
  state.pathEditor.rangeStart = null;
  state.pathEditor.rangeCurrent = null;
  state.pathEditor.rangeMoveStartWorld = null;
  state.pathEditor.rangeMoveOriginalPoints = null;
  renderPathSmoothButtons();
  if (showToast) toast("rangeを解除したよ");
  renderPathStats();
  drawPathEditor();
}

function selectedPathRangeHit(canvasPoint) {
  const selected = new Set(state.pathEditor.selectedRangeIndices);
  if (!selected.size) return false;
  const pointIndex = nearestPathPoint(canvasPoint, 16);
  if (pointIndex !== null && selected.has(pointIndex)) return true;

  const canvasPoints = state.pathEditor.points.map((point) => imageToCanvas(worldToImage(point)));
  for (const run of selectedRangeRuns()) {
    if (run.length < 2) continue;
    for (let cursor = 0; cursor < run.length - 1; cursor += 1) {
      const a = canvasPoints[run[cursor]];
      const b = canvasPoints[run[cursor + 1]];
      if (distanceToSegment(canvasPoint, a, b) <= 12) return true;
    }
  }
  return false;
}

function beginPathRangeMove(canvasPoint, pointerId) {
  if (!state.pathEditor.selectedRangeIndices.length) return;
  state.pathEditor.draggingRangeMove = true;
  state.pathEditor.selectedIndex = null;
  state.pathEditor.rangeMoveStartWorld = imageToWorld(canvasToImage(canvasPoint));
  state.pathEditor.rangeMoveOriginalPoints = clonePathPoints(state.pathEditor.points);
  state.pathEditor.undoPoints = clonePathPoints(state.pathEditor.points);
  $("pathCanvas").setPointerCapture(pointerId);
  renderPathSmoothButtons();
  renderPathStats();
  drawPathEditor();
}

function moveSelectedPathRange(canvasPoint) {
  const start = state.pathEditor.rangeMoveStartWorld;
  const original = state.pathEditor.rangeMoveOriginalPoints;
  if (!start || !original) return;
  const world = imageToWorld(canvasToImage(canvasPoint));
  const dx = world.x_m - start.x_m;
  const dy = world.y_m - start.y_m;
  const selected = new Set(state.pathEditor.selectedRangeIndices);
  state.pathEditor.points = original.map((point, index) => {
    if (!selected.has(index)) return { ...point };
    return {
      ...point,
      x_m: Number(point.x_m) + dx,
      y_m: Number(point.y_m) + dy,
    };
  });
  state.pathEditor.dirty = true;
  reindexPathPoints();
  renderPathStats();
  drawPathEditor();
}

function normalizedCanvasRect(a, b) {
  const x = Math.min(a.x, b.x);
  const y = Math.min(a.y, b.y);
  return {
    x,
    y,
    width: Math.abs(a.x - b.x),
    height: Math.abs(a.y - b.y),
  };
}

function applyPathSmoothing() {
  const points = state.pathEditor.points;
  if (points.length < 3) {
    toast("3点以上でsmoothできるよ");
    return;
  }
  const strength = clampNumber(Number($("pathSmoothStrength").value), 0.05, 0.85, 0.35);
  const passes = Math.round(clampNumber(Number($("pathSmoothPasses").value), 1, 8, 2));
  const selectedIndices = [...state.pathEditor.selectedRangeIndices];
  if (selectedIndices.length > 0 && selectedIndices.length < 3) {
    toast("range smoothは3点以上選んでね");
    return;
  }
  state.pathEditor.undoPoints = clonePathPoints(points);
  state.pathEditor.points = smoothPathPoints(
    points,
    strength,
    passes,
    pathIsCircular(),
    selectedIndices
  );
  if (state.pathEditor.selectedIndex !== null) {
    state.pathEditor.selectedIndex = Math.min(state.pathEditor.selectedIndex, state.pathEditor.points.length - 1);
  }
  state.pathEditor.dirty = true;
  reindexPathPoints();
  renderPathSmoothButtons();
  renderPathStats();
  drawPathEditor();
  toast(selectedIndices.length ? `range smooth ${selectedIndices.length}点 / ${passes} pass` : `smooth ${passes} pass`);
}

function undoPathSmoothing() {
  if (!state.pathEditor.undoPoints) return;
  state.pathEditor.points = clonePathPoints(state.pathEditor.undoPoints);
  state.pathEditor.undoPoints = null;
  state.pathEditor.dirty = true;
  reindexPathPoints();
  renderPathSmoothButtons();
  renderPathStats();
  drawPathEditor();
  toast("変更を戻したよ");
}

function smoothPathPoints(points, strength, passes, circular, selectedIndices = []) {
  let current = clonePathPoints(points);
  const count = current.length;
  const selectedSet = new Set(selectedIndices);
  const fullPath = selectedSet.size === 0 || selectedSet.size === count;
  const runs = fullPath ? [] : selectedRangeRuns(selectedIndices, count, circular);
  for (let pass = 0; pass < passes; pass += 1) {
    const previous = current;
    current = previous.map((point, index) => {
      if (fullPath) {
        if (!circular && (index === 0 || index === count - 1)) {
          return { ...point };
        }
        const previousPoint = previous[(index - 1 + count) % count];
        const nextPoint = previous[(index + 1) % count];
        const targetX = (previousPoint.x_m + nextPoint.x_m) / 2;
        const targetY = (previousPoint.y_m + nextPoint.y_m) / 2;
        return {
          ...point,
          x_m: point.x_m + (targetX - point.x_m) * strength,
          y_m: point.y_m + (targetY - point.y_m) * strength,
        };
      }
      const range = runs.find((run) => run.includes(index));
      if (!range || range.length < 3) {
        return { ...point };
      }
      const position = range.indexOf(index);
      if (position === 0 || position === range.length - 1) {
        return { ...point };
      }
      const previousPoint = previous[range[position - 1]];
      const nextPoint = previous[range[position + 1]];
      const targetX = (previousPoint.x_m + nextPoint.x_m) / 2;
      const targetY = (previousPoint.y_m + nextPoint.y_m) / 2;
      return {
        ...point,
        x_m: point.x_m + (targetX - point.x_m) * strength,
        y_m: point.y_m + (targetY - point.y_m) * strength,
      };
    });
  }
  return current;
}

function selectedRangeRuns(indices = state.pathEditor.selectedRangeIndices, count = state.pathEditor.points.length, circular = pathIsCircular()) {
  const sorted = [...new Set(indices)].filter((index) => index >= 0 && index < count).sort((a, b) => a - b);
  if (!sorted.length) return [];
  const runs = [];
  let current = [sorted[0]];
  for (const index of sorted.slice(1)) {
    if (index === current[current.length - 1] + 1) {
      current.push(index);
    } else {
      runs.push(current);
      current = [index];
    }
  }
  runs.push(current);
  const lastRun = runs[runs.length - 1];
  if (circular && runs.length > 1 && runs[0][0] === 0 && lastRun[lastRun.length - 1] === count - 1) {
    const first = runs.shift();
    const last = runs.pop();
    runs.push([...last, ...first]);
  }
  return runs;
}

function clampNumber(value, min, max, fallback) {
  if (!Number.isFinite(value)) return fallback;
  return Math.max(min, Math.min(max, value));
}

function renderPathStats() {
  const points = state.pathEditor.points;
  const stats = calculatePathStats(points);
  $("pathPointCount").textContent = points.length ? String(points.length) : "-";
  $("pathLength").textContent = Number.isFinite(stats.length) ? `${stats.length.toFixed(1)} m` : "-";
  $("pathMinSegment").textContent = Number.isFinite(stats.minSegment) ? `${stats.minSegment.toFixed(2)} m` : "-";
  const selected = state.pathEditor.selectedIndex;
  $("pathSelected").textContent = selected === null ? "-" : String(selected);
  const rangeCount = state.pathEditor.selectedRangeIndices.length;
  $("pathRangeCount").textContent = rangeCount ? `${rangeCount} pts` : "-";
  const point = selected === null ? null : points[selected];
  $("pathPointX").value = point ? Number(point.x_m).toFixed(3) : "";
  $("pathPointY").value = point ? Number(point.y_m).toFixed(3) : "";
  const warnings = [];
  if (state.pathEditor.dirty) warnings.push("unsaved changes");
  if (rangeCount) warnings.push(`range ${rangeCount} points`);
  if (Number.isFinite(stats.minSegment) && stats.minSegment < 0.2) warnings.push(`min segment ${stats.minSegment.toFixed(3)} m`);
  $("pathWarnings").textContent = warnings.join("\n");
}

function calculatePathStats(points) {
  let length = 0;
  let minSegment = Infinity;
  for (let i = 0; i < points.length - 1; i += 1) {
    const segment = Math.hypot(points[i + 1].x_m - points[i].x_m, points[i + 1].y_m - points[i].y_m);
    length += segment;
    minSegment = Math.min(minSegment, segment);
  }
  return { length, minSegment };
}

function updateSelectedPathPointFromInputs() {
  const index = state.pathEditor.selectedIndex;
  if (index === null) return;
  const x = Number($("pathPointX").value);
  const y = Number($("pathPointY").value);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return;
  const point = state.pathEditor.points[index];
  clearPathSmoothUndo();
  point.x_m = x;
  point.y_m = y;
  state.pathEditor.dirty = true;
  renderPathStats();
  drawPathEditor();
}

async function loadSelectedPath() {
  await openPathEditor($("pathSource").value);
}

async function savePathEditor() {
  const source = state.pathEditor.data && state.pathEditor.data.source ? state.pathEditor.data.source : {};
  const data = await api("/api/path-editor/save", {
    method: "POST",
    body: JSON.stringify({
      config_path: state.pathEditor.data ? state.pathEditor.data.config_path : null,
      source_path: source.path,
      target_path: $("pathTarget").value,
      switch_config: $("pathSwitchConfig").checked,
      auto_rebuild: $("pathAutoBuild").checked,
      points: state.pathEditor.points,
    }),
  });
  state.pathEditor.points = clonePathPoints(data.points || state.pathEditor.points);
  state.pathEditor.dirty = false;
  state.pathEditor.undoPoints = null;
  renderPathSmoothButtons();
  $("pathWarnings").textContent = (data.warnings || []).join("\n");
  const backupText = data.backup ? " バックアップ作成済み" : "";
  toast(data.config_changed ? `経路CSVを保存してconfigも切り替えたよ${backupText}` : `経路CSVを保存したよ${backupText}`);
  await loadState();
  await openPathEditor(data.path);
}

async function saveControl(autoRebuild) {
  const method = $("controlMethod").value;
  const data = await api("/api/control-method", {
    method: "POST",
    body: JSON.stringify({ method, auto_rebuild: autoRebuild }),
  });
  toast(data.changed.length ? `XML defaultを ${method} に更新したよ` : `control_methodは ${method} のまま`);
  await loadState();
}

async function rememberSelectedControlMethod(method) {
  const data = await api("/api/selected-control-method", {
    method: "POST",
    body: JSON.stringify({ method }),
  });
  state.app.selected_control_method = data.method;
  state.app.files = data.files || (state.app.catalog && state.app.catalog[data.method]) || [];
  renderState();
  if (!state.app.files.some((file) => file.path === state.currentFile) && state.app.files.length) {
    await openFile(state.app.files[0].path);
  }
}

async function validateFile() {
  if (state.structuredDirty || state.structuredDescriptionsDirty) await applyStructuredRows(true);
  await api("/api/validate", {
    method: "POST",
    body: JSON.stringify({ path: state.currentFile, content: $("fileEditor").value }),
  });
  toast("構文チェックOK");
}

async function diffFile() {
  if (state.structuredDirty || state.structuredDescriptionsDirty) await applyStructuredRows(true);
  const data = await api("/api/diff", {
    method: "POST",
    body: JSON.stringify({ path: state.currentFile, content: $("fileEditor").value }),
  });
  $("diffOutput").textContent = data.diff || "差分なし";
}

async function saveFile(autoRebuild) {
  let structuredApply = null;
  if (state.structuredDirty || state.structuredDescriptionsDirty) {
    structuredApply = await applyStructuredRows(true);
  }
  const data = await api("/api/file", {
    method: "POST",
    body: JSON.stringify({
      path: state.currentFile,
      content: $("fileEditor").value,
      auto_rebuild: autoRebuild,
    }),
  });
  state.currentContent = $("fileEditor").value;
  if (data.changed) {
    toast("保存したよ。バックアップも作成済み");
  } else if (structuredApply && structuredApply.descriptions_changed) {
    toast("descriptionを保存したよ");
  } else {
    toast("変更なし");
  }
  await loadState();
}

async function run(action) {
  const method = $("controlMethod").value;
  const buildFirst = $("buildFirst").checked;
  const note = $("runNote").value;
  const headless = $("runHeadless").checked;
  const npcCount = Number($("npcCount").value || 0);
  const data = await api("/api/run", {
    method: "POST",
    body: JSON.stringify({
      action,
      control_method: method,
      build_first: buildFirst,
      note,
      headless,
      npc_count: npcCount,
    }),
  });
  toast(`${action} を開始したよ`);
  setCommandLog(`$ ${data.command}\n`);
  startCommandPolling();
  await loadState();
}

async function stopCommand() {
  await api("/api/stop", { method: "POST", body: "{}" });
  toast("停止リクエストを送ったよ");
  await refreshCommand();
}

async function refreshCommand() {
  const data = await api("/api/command");
  renderStatus(data);
  if (data.command) {
    const commandLine = data.command.command ? `$ ${data.command.command}\n` : "";
    setCommandLog(`${commandLine}${data.log_tail || ""}`);
  } else {
    setCommandLog("");
  }
  if (data.running) {
    startCommandPolling();
  } else {
    stopCommandPolling();
  }
}

function startCommandPolling() {
  if (state.commandTimer) return;
  state.commandTimer = setInterval(refreshCommand, 2000);
}

function stopCommandPolling() {
  if (!state.commandTimer) return;
  clearInterval(state.commandTimer);
  state.commandTimer = null;
}

async function refreshDocker() {
  const data = await api("/api/docker-ps");
  $("dockerPs").textContent = data.output || "";
}

async function savePreset() {
  const name = $("presetName").value;
  const note = $("presetNote").value;
  const method = $("controlMethod").value;
  const data = await api("/api/presets", {
    method: "POST",
    body: JSON.stringify({ name, note, control_method: method }),
  });
  toast(`プリセット ${data.name} を保存したよ`);
  await loadState();
}

async function restorePreset() {
  const name = $("presetSelect").value;
  if (!name) {
    toast("復元するプリセットを選んでね");
    return;
  }
  const data = await api("/api/presets/restore", {
    method: "POST",
    body: JSON.stringify({ name, auto_rebuild: $("buildFirst").checked }),
  });
  toast(`プリセット ${name} を復元したよ`);
  await loadState();
  if (data.changed.length) {
    await openFile(data.changed[0]);
  }
}

async function refreshHistory() {
  const data = await api("/api/history");
  renderLapHistory(data.outputs || []);
  renderReports(data.reports || []);
  await renderMotionRunPicker(data.reports || []);
}

function renderLapHistory(rows) {
  const root = $("lapHistory");
  if (!rows.length) {
    root.innerHTML = "<p>まだresult-summaryがないよ。</p>";
    return;
  }
  const body = rows
    .slice(0, 20)
    .map((row) => {
      const laps = (row.laps || []).map((lap) => Number(lap).toFixed(1)).join(" / ");
      const logHref = `/files?path=${encodeURIComponent(row.autoware_log)}`;
      return `<tr>
        <td>${escapeHtml(row.run_dir)}</td>
        <td>${row.finished ? "yes" : "no"}</td>
        <td>${fmtSec(row.best_lap_sec)}</td>
        <td>${fmtSec(row.avg_lap_sec)}</td>
        <td>${escapeHtml(laps)}</td>
        <td><a href="${logHref}" target="_blank" rel="noreferrer">log</a></td>
      </tr>`;
    })
    .join("");
  root.innerHTML = `<table>
    <thead><tr><th>run</th><th>finish</th><th>best</th><th>avg</th><th>laps</th><th></th></tr></thead>
    <tbody>${body}</tbody>
  </table>`;
}

function renderReports(rows) {
  const root = $("reportList");
  if (!rows.length) {
    root.innerHTML = "<p>まだレポートがないよ。</p>";
    return;
  }
  root.innerHTML = rows
    .slice(0, 20)
    .map((row) => {
      const href = `/files?path=${encodeURIComponent(row.report_path)}`;
      return `<a class="report-item" href="${href}" target="_blank" rel="noreferrer">
        <strong>${escapeHtml(row.run_id)}</strong>
        <small>best ${fmtSec(row.best_lap_sec)} / avg ${fmtSec(row.avg_lap_sec)} / ${escapeHtml(row.judgement || "-")}</small>
      </a>`;
    })
    .join("");
}

async function renderMotionRunPicker(rows) {
  state.motion.reports = rows.filter((row) => row.motion_log_available);
  const select = $("motionRun");
  const previous = state.motion.selectedRunId || select.value;
  select.innerHTML = "";
  for (const row of state.motion.reports) {
    const option = document.createElement("option");
    option.value = row.run_id;
    option.textContent = row.run_id;
    option.selected = row.run_id === previous;
    select.appendChild(option);
  }
  if (!state.motion.reports.length) {
    state.motion.selectedRunId = null;
    state.motion.selectedDomain = null;
    state.motion.data = null;
    renderMotionEmpty("まだmotion logがないよ。EVAL後かDEV後にlog化してね。");
    return;
  }
  state.motion.selectedRunId = select.value || state.motion.reports[0].run_id;
  select.value = state.motion.selectedRunId;
  if (!state.motion.data || state.motion.data.run_id !== state.motion.selectedRunId) {
    await loadMotionLog();
  }
}

async function loadMotionLog() {
  const runId = $("motionRun").value;
  if (!runId) {
    renderMotionEmpty("表示するrunを選んでね。");
    return;
  }
  const domain = $("motionDomain").value || state.motion.selectedDomain || "";
  const params = new URLSearchParams({ run_id: runId });
  if (domain) params.set("domain", domain);
  const data = await api(`/api/motion-log?${params.toString()}`);
  state.motion.selectedRunId = data.run_id;
  state.motion.selectedDomain = data.domain_id || "";
  state.motion.data = data;
  renderMotionDomainPicker(data);
  renderMotionLog(data);
}

function renderMotionDomainPicker(data) {
  const select = $("motionDomain");
  const previous = state.motion.selectedDomain || data.domain_id || "";
  select.innerHTML = "";
  for (const domain of data.domains || []) {
    const option = document.createElement("option");
    option.value = domain;
    option.textContent = domain;
    option.selected = domain === previous;
    select.appendChild(option);
  }
  select.disabled = !(data.domains || []).length;
  if (data.domain_id) {
    select.value = data.domain_id;
    state.motion.selectedDomain = data.domain_id;
  }
}

function renderMotionLog(data) {
  if (!(data.points && data.points.length)) {
    renderMotionEmpty("このrunには表示できるmotion sampleがないよ。");
    return;
  }
  drawMotionChart(data.points);
  const stats = data.stats || {};
  $("motionStats").innerHTML = `
    <dt>duration</dt><dd>${fmtSec(stats.duration_sec)}</dd>
    <dt>samples</dt><dd>${escapeHtml(valueOr(stats.samples, "-"))}</dd>
    <dt>max speed</dt><dd>${fmtNum(stats.max_speed_mps, 3, " m/s")}</dd>
    <dt>max accel</dt><dd>${fmtNum(stats.max_abs_acceleration_mps2, 3, " m/s²")}</dd>
    <dt>max steer</dt><dd>${fmtNum(valueOr(stats.max_abs_steering_rad, stats.max_abs_command_steering_rad), 3, " rad")}</dd>
  `;
  const links = Object.entries(data.paths || {})
    .filter(([, path]) => path)
    .map(([label, path]) => {
      const href = `/files?path=${encodeURIComponent(path)}`;
      return `<a href="${href}" target="_blank" rel="noreferrer">${escapeHtml(label)}</a>`;
    });
  $("motionLinks").innerHTML = links.length ? links.join("") : "";
}

function renderMotionEmpty(message) {
  const canvas = $("motionChart");
  const ctx = canvas.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || canvas.width;
  const height = canvas.clientHeight || canvas.height;
  if (canvas.width !== Math.floor(width * dpr) || canvas.height !== Math.floor(height * dpr)) {
    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(height * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = chartColor("--muted");
  ctx.font = "13px system-ui, sans-serif";
  ctx.fillText(message, 14, 28);
  $("motionStats").innerHTML = "";
  $("motionLinks").innerHTML = "";
  $("motionDomain").innerHTML = "";
  $("motionDomain").disabled = true;
}

function drawMotionChart(points) {
  const canvas = $("motionChart");
  const ctx = canvas.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || canvas.width;
  const height = canvas.clientHeight || canvas.height;
  if (canvas.width !== Math.floor(width * dpr) || canvas.height !== Math.floor(height * dpr)) {
    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(height * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  const panels = [
    {
      label: "speed [m/s]",
      keys: [
        { key: "speed_mps", label: "actual", color: "#2457a6" },
        { key: "target_speed_mps", label: "target", color: "#718096" },
      ],
    },
    {
      label: "accel [m/s²]",
      keys: [
        { key: "acceleration_mps2", label: "actual", color: "#956218" },
        { key: "command_accel_mps2", label: "cmd", color: "#c05621" },
      ],
    },
    {
      label: "steer [rad]",
      keys: [
        { key: "steering_rad", label: "actual", color: "#177245" },
        { key: "command_steer_rad", label: "cmd", color: "#2f855a" },
      ],
    },
  ];
  const plotLeft = 54;
  const plotRight = 12;
  const plotTop = 16;
  const plotGap = 12;
  const panelHeight = (height - plotTop * 2 - plotGap * (panels.length - 1)) / panels.length;
  const maxTime = Math.max(...points.map((point) => Number(point.time_sec || 0)), 1e-6);

  ctx.font = "11px system-ui, sans-serif";
  panels.forEach((panel, index) => {
    const top = plotTop + index * (panelHeight + plotGap);
    drawMotionPanel(ctx, points, panel, {
      left: plotLeft,
      top,
      width: width - plotLeft - plotRight,
      height: panelHeight,
      maxTime,
    });
  });
}

function drawMotionPanel(ctx, points, panel, rect) {
  const text = chartColor("--text");
  const muted = chartColor("--muted");
  const line = chartColor("--line");
  const values = [];
  for (const point of points) {
    for (const series of panel.keys) {
      const value = point[series.key];
      if (Number.isFinite(Number(value))) values.push(Number(value));
    }
  }
  if (!values.length) {
    ctx.fillStyle = muted;
    ctx.fillText(`${panel.label}: no data`, rect.left, rect.top + 16);
    return;
  }
  let minY = Math.min(...values);
  let maxY = Math.max(...values);
  if (minY === maxY) {
    minY -= 1;
    maxY += 1;
  }
  const pad = (maxY - minY) * 0.08;
  minY -= pad;
  maxY += pad;
  const xFor = (time) => rect.left + (Number(time || 0) / rect.maxTime) * rect.width;
  const yFor = (value) => rect.top + rect.height - ((Number(value) - minY) / (maxY - minY)) * rect.height;

  ctx.strokeStyle = line;
  ctx.lineWidth = 1;
  ctx.strokeRect(rect.left, rect.top, rect.width, rect.height);
  ctx.beginPath();
  for (let i = 1; i < 4; i += 1) {
    const y = rect.top + (rect.height / 4) * i;
    ctx.moveTo(rect.left, y);
    ctx.lineTo(rect.left + rect.width, y);
  }
  ctx.stroke();

  ctx.fillStyle = text;
  ctx.fillText(panel.label, 8, rect.top + 14);
  ctx.fillStyle = muted;
  ctx.fillText(maxY.toFixed(2), 8, rect.top + 28);
  ctx.fillText(minY.toFixed(2), 8, rect.top + rect.height - 4);

  for (const series of panel.keys) {
    const seriesPoints = points.filter((point) => Number.isFinite(Number(point[series.key])));
    if (!seriesPoints.length) continue;
    ctx.strokeStyle = series.color;
    ctx.lineWidth = series.label === "cmd" || series.label === "target" ? 1.4 : 2;
    ctx.setLineDash(series.label === "cmd" || series.label === "target" ? [4, 4] : []);
    ctx.beginPath();
    seriesPoints.forEach((point, index) => {
      const x = xFor(point.time_sec);
      const y = yFor(point[series.key]);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.fillStyle = muted;
  const legend = panel.keys.map((series) => series.label).join(" / ");
  ctx.fillText(legend, rect.left + rect.width - 90, rect.top + 14);
}

function chartColor(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || "#64748b";
}

function escapeHtml(value) {
  return String(valueOr(value, ""))
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function bind() {
  applyTheme(currentTheme());
  $("themeToggle").addEventListener("change", () => {
    applyTheme($("themeToggle").checked ? "dark" : "light");
  });
  $("tableMode").addEventListener("click", async () => {
    if (!(state.currentKind === "yaml" || state.currentKind === "xml")) return;
    state.editorMode = "table";
    await loadStructuredRows();
    renderEditorMode();
  });
  $("textMode").addEventListener("click", () => {
    state.editorMode = "text";
    renderEditorMode();
  });
  $("pathMode").addEventListener("click", () => openPathEditor().catch((e) => toast(e.message)));
  $("applyStructured").addEventListener("click", () => applyStructuredRows(false).catch((e) => toast(e.message)));
  $("structuredFilter").addEventListener("input", renderStructuredRows);
  $("controlMethod").addEventListener("change", async () => {
    const method = $("controlMethod").value;
    await rememberSelectedControlMethod(method);
  });
  $("saveControl").addEventListener("click", () => saveControl(false).catch((e) => toast(e.message)));
  $("saveControlBuild").addEventListener("click", () => saveControl(true).catch((e) => toast(e.message)));
  $("validateFile").addEventListener("click", () => validateFile().catch((e) => toast(e.message)));
  $("diffFile").addEventListener("click", () => diffFile().catch((e) => toast(e.message)));
  $("saveFile").addEventListener("click", () => saveFile(false).catch((e) => toast(e.message)));
  $("saveFileBuild").addEventListener("click", () => saveFile(true).catch((e) => toast(e.message)));
  $("runBuild").addEventListener("click", () => run("build").catch((e) => toast(e.message)));
  $("runDev").addEventListener("click", () => run("dev").catch((e) => toast(e.message)));
  $("runEval").addEventListener("click", () => run("eval").catch((e) => toast(e.message)));
  $("runQuickEval").addEventListener("click", () => run("quick-eval").catch((e) => toast(e.message)));
  $("runIngest").addEventListener("click", () => run("ingest").catch((e) => toast(e.message)));
  $("runDown").addEventListener("click", () => run("down").catch((e) => toast(e.message)));
  $("stopCommand").addEventListener("click", () => stopCommand().catch((e) => toast(e.message)));
  $("refreshDocker").addEventListener("click", () => refreshDocker().catch((e) => toast(e.message)));
  $("refreshMotion").addEventListener("click", () => loadMotionLog().catch((e) => toast(e.message)));
  $("motionRun").addEventListener("change", () => {
    state.motion.selectedRunId = $("motionRun").value;
    state.motion.selectedDomain = null;
    loadMotionLog().catch((e) => toast(e.message));
  });
  $("motionDomain").addEventListener("change", () => {
    state.motion.selectedDomain = $("motionDomain").value;
    loadMotionLog().catch((e) => toast(e.message));
  });
  $("savePreset").addEventListener("click", () => savePreset().catch((e) => toast(e.message)));
  $("restorePreset").addEventListener("click", () => restorePreset().catch((e) => toast(e.message)));
  $("pathLoad").addEventListener("click", () => loadSelectedPath().catch((e) => toast(e.message)));
  $("pathSave").addEventListener("click", () => savePathEditor().catch((e) => toast(e.message)));
  $("pathFit").addEventListener("click", () => {
    fitPathView(true);
    drawPathEditor();
  });
  $("pathZoomIn").addEventListener("click", () => zoomPathView(1.16));
  $("pathZoomOut").addEventListener("click", () => zoomPathView(0.86));
  $("pathSelectMode").addEventListener("click", () => setPathMode("move"));
  $("pathAddMode").addEventListener("click", () => setPathMode("add"));
  const pathRangeMode = $("pathRangeMode");
  if (pathRangeMode) pathRangeMode.addEventListener("click", () => setPathMode("range"));
  $("pathDeletePoint").addEventListener("click", deleteSelectedPathPoint);
  $("pathSmooth").addEventListener("click", applyPathSmoothing);
  $("pathUndoSmooth").addEventListener("click", undoPathSmoothing);
  $("pathClearRange").addEventListener("click", () => clearPathRangeSelection(true));
  $("pathPointX").addEventListener("change", updateSelectedPathPointFromInputs);
  $("pathPointY").addEventListener("change", updateSelectedPathPointFromInputs);
  $("pathCanvas").addEventListener("pointerdown", onPathPointerDown);
  $("pathCanvas").addEventListener("pointermove", onPathPointerMove);
  $("pathCanvas").addEventListener("pointerup", onPathPointerUp);
  $("pathCanvas").addEventListener("pointercancel", onPathPointerUp);
  $("pathCanvas").addEventListener("wheel", onPathWheel, { passive: false });
  window.addEventListener("resize", () => {
    if (state.editorMode === "path") {
      drawPathEditor();
    }
    if (state.motion.data && state.motion.data.points && state.motion.data.points.length) {
      drawMotionChart(state.motion.data.points);
    }
  });
}

bind();
loadState().catch((e) => toast(e.message));
setInterval(refreshHistory, 15000);
