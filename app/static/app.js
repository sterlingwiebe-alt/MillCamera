/* Vision Inspection UI */
"use strict";

const $ = (id) => document.getElementById(id);
const api = async (path, opts = {}) => {
  const r = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...opts,
    body: opts.body ? JSON.stringify(opts.body) : undefined,
  });
  if (!r.ok) {
    let msg = r.statusText;
    try { msg = (await r.json()).detail || msg; } catch {}
    throw new Error(msg);
  }
  return r.json();
};
const toast = (msg) => { $("draw-hint").textContent = msg; };

/* Full sensor size -- used to map canvas coords to image pixels. Updated
   from the first frame's natural size (stream is served at scale=0.5). */
const STREAM_SCALE = 0.5;

/* ------------------------------------------------------------ tabs */
document.querySelectorAll(".tab").forEach((b) => {
  b.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach((x) => x.classList.remove("active"));
    document.querySelectorAll(".tabpane").forEach((x) => x.classList.remove("active"));
    b.classList.add("active");
    $("tab-" + b.dataset.tab).classList.add("active");
    if (b.dataset.tab === "run") {
      $("run-img").src = "/stream?annotated=1&scale=0.5&fps=10&_=" + Date.now();
    } else {
      $("run-img").src = "";
    }
  });
});

/* ------------------------------------------------- recipe + ROI state */
let recipe = null;          // full recipe dict being edited
let selectedRoi = -1;       // index into recipe.rois
let drawMode = null;        // null | 'region' | 'teach'
let viewSource = "live";    // 'live' | 'reference' (what the Setup image shows)
let refAnnotated = false;   // reference currently showing server-side overlay

function updateSetupSrc() {
  refAnnotated = false;
  const live = viewSource === "live";
  $("btn-src-live").classList.toggle("active", live);
  $("btn-src-ref").classList.toggle("active", !live);
  if (live) {
    $("setup-img").src = "/stream?annotated=0&scale=0.5&fps=8&_=" + Date.now();
    $("src-note").textContent = "";
  } else {
    $("setup-img").src = `/api/recipes/${encodeURIComponent(recipe.name)}` +
      `/reference.jpg?annotated=0&scale=0.5&_=${Date.now()}`;
    $("src-note").textContent = "showing the recipe's stored reference image";
  }
  setTimeout(syncCanvas, 400);
}

$("btn-src-live").addEventListener("click", () => { viewSource = "live"; updateSetupSrc(); });
$("btn-src-ref").addEventListener("click", () => {
  if (!recipe || !recipe.reference_image) { toast("This recipe has no reference image — click “Set reference…” first."); return; }
  viewSource = "reference"; updateSetupSrc();
});
$("btn-ref-set").addEventListener("click", async () => {
  if (!recipe) { toast("Select a recipe first."); return; }
  const path = prompt("Image path (relative to the app folder):",
    recipe.reference_image || "Sample Images\\LogDeck\\ScrewFeeder.jpeg");
  if (path === null) return;
  try {
    await api(`/api/recipes/${encodeURIComponent(recipe.name)}/reference`,
      { method: "POST", body: { path } });
    await loadRecipe(recipe.name);
    toast(path ? "Reference image set." : "Reference image cleared.");
  } catch (e) { toast("Set reference failed: " + e.message); }
});

async function refreshRecipeList() {
  const d = await api("/api/recipes");
  const sel = $("recipe-select");
  sel.innerHTML = "";
  d.recipes.forEach((n) => {
    const o = document.createElement("option");
    o.value = o.textContent = n;
    if (n === d.active) o.textContent = n + " (active)";
    sel.appendChild(o);
  });
  if (d.active) sel.value = d.active;
  if (sel.value) await loadRecipe(sel.value);
}

async function loadRecipe(name) {
  recipe = await api("/api/recipes/" + encodeURIComponent(name));
  selectedRoi = recipe.rois.length ? 0 : -1;
  renderRoiList();
  // recipes authored against a stored photo open in reference view so the
  // zones are visible even when the camera looks at something else
  viewSource = recipe.reference_image ? "reference" : "live";
  updateSetupSrc();
  drawOverlay();
}

function defaultRoi(kind) {
  const n = recipe.rois.length + 1;
  return {
    name: (kind === "template" ? "Match" : "Zone") + n, kind, enabled: true,
    rect: [200, 200, 600, 400],
    template_file: "", score_threshold: 0.8, multi_match: false, max_matches: 5,
    color_space: "gray", lower: [0, 0, 0], upper: [255, 255, 255],
    min_percent: 50, max_percent: 100,
  };
}

function renderRoiList() {
  const list = $("roi-list");
  list.innerHTML = "";
  if (!recipe) return;
  recipe.rois.forEach((roi, i) => {
    const card = document.createElement("div");
    card.className = "roi-card" + (i === selectedRoi ? " selected" : "");
    card.addEventListener("click", () => { selectedRoi = i; renderRoiList(); drawOverlay(); });

    const r1 = document.createElement("div");
    r1.className = "row";
    r1.innerHTML = `<span class="tag">${roi.kind}</span>`;
    const name = document.createElement("input");
    name.type = "text"; name.value = roi.name;
    name.addEventListener("change", () => { roi.name = name.value; });
    r1.appendChild(name);
    const en = document.createElement("input");
    en.type = "checkbox"; en.checked = roi.enabled; en.title = "enabled";
    en.addEventListener("change", () => { roi.enabled = en.checked; drawOverlay(); });
    r1.appendChild(en);
    const del = document.createElement("button");
    del.textContent = "✕"; del.className = "danger"; del.title = "delete ROI";
    del.addEventListener("click", (e) => {
      e.stopPropagation();
      recipe.rois.splice(i, 1);
      selectedRoi = Math.min(selectedRoi, recipe.rois.length - 1);
      renderRoiList(); drawOverlay();
    });
    r1.appendChild(del);
    card.appendChild(r1);

    const r2 = document.createElement("div");
    r2.className = "row";
    r2.innerHTML = `<span class="hint">rect [${roi.rect.map(Math.round).join(", ")}]</span>`;
    const btnRegion = document.createElement("button");
    btnRegion.textContent = "Draw region";
    btnRegion.addEventListener("click", (e) => {
      e.stopPropagation(); selectedRoi = i; drawMode = "region";
      renderRoiList(); toast(`Drag on the image to set the ${roi.kind === "template" ? "SEARCH region" : "zone"} for “${roi.name}”.`);
    });
    r2.appendChild(btnRegion);
    card.appendChild(r2);

    if (roi.kind === "template") {
      const r3 = document.createElement("div");
      r3.className = "row";
      r3.append("score ≥ ");
      const th = document.createElement("input");
      th.type = "number"; th.min = 0; th.max = 1; th.step = 0.01; th.value = roi.score_threshold;
      th.addEventListener("change", () => { roi.score_threshold = parseFloat(th.value); });
      r3.appendChild(th);
      const multi = document.createElement("input");
      multi.type = "checkbox"; multi.checked = roi.multi_match; multi.title = "find multiple matches";
      multi.addEventListener("change", () => { roi.multi_match = multi.checked; });
      r3.append(multi, " multi ≤ ");
      const mm = document.createElement("input");
      mm.type = "number"; mm.min = 1; mm.max = 8; mm.value = roi.max_matches;
      mm.addEventListener("change", () => { roi.max_matches = parseInt(mm.value); });
      r3.appendChild(mm);
      card.appendChild(r3);

      const r4 = document.createElement("div");
      r4.className = "row";
      const teach = document.createElement("button");
      teach.textContent = "Teach template";
      teach.addEventListener("click", (e) => {
        e.stopPropagation(); selectedRoi = i; drawMode = "teach";
        renderRoiList(); toast(`Drag on the image around the FEATURE to teach for “${roi.name}” (save the recipe first if the ROI is new).`);
      });
      r4.appendChild(teach);
      card.appendChild(r4);
      if (roi.template_file) {
        const img = document.createElement("img");
        img.className = "tpl";
        img.src = `/api/recipes/${encodeURIComponent(recipe.name)}/template/${encodeURIComponent(roi.name)}?_=${Date.now()}`;
        card.appendChild(img);
      }
    } else {
      const r3 = document.createElement("div");
      r3.className = "row";
      const cs = document.createElement("select");
      ["gray", "hsv"].forEach((v) => {
        const o = document.createElement("option");
        o.value = o.textContent = v; cs.appendChild(o);
      });
      cs.value = roi.color_space;
      cs.addEventListener("change", () => { roi.color_space = cs.value; renderRoiList(); });
      r3.appendChild(cs);
      card.appendChild(r3);

      const r4 = document.createElement("div");
      r4.className = "row";
      const mkNum = (val, cb, min, max) => {
        const inp = document.createElement("input");
        inp.type = "number"; inp.min = min; inp.max = max; inp.value = val;
        inp.addEventListener("change", () => cb(parseInt(inp.value)));
        return inp;
      };
      if (roi.color_space === "gray") {
        r4.append("gray ");
        r4.appendChild(mkNum(roi.lower[0], (v) => (roi.lower[0] = v), 0, 255));
        r4.append(" – ");
        r4.appendChild(mkNum(roi.upper[0], (v) => (roi.upper[0] = v), 0, 255));
      } else {
        r4.append("H ");
        r4.appendChild(mkNum(roi.lower[0], (v) => (roi.lower[0] = v), 0, 179));
        r4.append("–");
        r4.appendChild(mkNum(roi.upper[0], (v) => (roi.upper[0] = v), 0, 179));
        r4.append(" S ");
        r4.appendChild(mkNum(roi.lower[1], (v) => (roi.lower[1] = v), 0, 255));
        r4.append("–");
        r4.appendChild(mkNum(roi.upper[1], (v) => (roi.upper[1] = v), 0, 255));
        r4.append(" V ");
        r4.appendChild(mkNum(roi.lower[2], (v) => (roi.lower[2] = v), 0, 255));
        r4.append("–");
        r4.appendChild(mkNum(roi.upper[2], (v) => (roi.upper[2] = v), 0, 255));
      }
      card.appendChild(r4);

      const r5 = document.createElement("div");
      r5.className = "row";
      r5.append("in-range % ");
      const mn = document.createElement("input");
      mn.type = "number"; mn.min = 0; mn.max = 100; mn.value = roi.min_percent;
      mn.addEventListener("change", () => { roi.min_percent = parseFloat(mn.value); });
      const mx = document.createElement("input");
      mx.type = "number"; mx.min = 0; mx.max = 100; mx.value = roi.max_percent;
      mx.addEventListener("change", () => { roi.max_percent = parseFloat(mx.value); });
      r5.append(mn, " – ", mx);
      card.appendChild(r5);
    }
    list.appendChild(card);
  });
}

/* ------------------------------------------------------- ROI drawing */
const canvas = $("setup-canvas");
const setupImg = $("setup-img");
let dragStart = null;
let dragCur = null;

function imgScale() {
  /* canvas css px -> full-image px */
  const w = setupImg.clientWidth;
  const natural = setupImg.naturalWidth || 1224;      // stream at 0.5 scale
  const fullWidth = natural / STREAM_SCALE;
  return fullWidth / w;
}

function syncCanvas() {
  canvas.width = setupImg.clientWidth;
  canvas.height = setupImg.clientHeight;
  drawOverlay();
}
new ResizeObserver(syncCanvas).observe(setupImg);

function drawOverlay() {
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!recipe || refAnnotated) return;   // server overlay already on the image
  const s = imgScale();
  recipe.rois.forEach((roi, i) => {
    if (!roi.enabled) return;
    const [x, y, w, h] = roi.rect.map((v) => v / s);
    ctx.lineWidth = i === selectedRoi ? 3 : 2;
    ctx.strokeStyle = roi.kind === "template" ? "#38bdf8" : "#facc15";
    ctx.strokeRect(x, y, w, h);
    ctx.font = "13px sans-serif";
    ctx.fillStyle = ctx.strokeStyle;
    ctx.fillText(roi.name, x + 4, Math.max(y - 5, 12));
  });
  if (dragStart && dragCur) {
    ctx.strokeStyle = drawMode === "teach" ? "#f472b6" : "#4ade80";
    ctx.setLineDash([6, 4]);
    ctx.strokeRect(dragStart.x, dragStart.y,
                   dragCur.x - dragStart.x, dragCur.y - dragStart.y);
    ctx.setLineDash([]);
  }
}

canvas.addEventListener("mousedown", (e) => {
  if (drawMode === null || selectedRoi < 0) return;
  const r = canvas.getBoundingClientRect();
  dragStart = { x: e.clientX - r.left, y: e.clientY - r.top };
  dragCur = { ...dragStart };
});
canvas.addEventListener("mousemove", (e) => {
  if (!dragStart) return;
  const r = canvas.getBoundingClientRect();
  dragCur = { x: e.clientX - r.left, y: e.clientY - r.top };
  drawOverlay();
});
window.addEventListener("mouseup", async () => {
  if (!dragStart || !dragCur || selectedRoi < 0) { dragStart = dragCur = null; return; }
  const s = imgScale();
  const x = Math.min(dragStart.x, dragCur.x) * s;
  const y = Math.min(dragStart.y, dragCur.y) * s;
  const w = Math.abs(dragCur.x - dragStart.x) * s;
  const h = Math.abs(dragCur.y - dragStart.y) * s;
  const rect = [x, y, w, h].map(Math.round);
  const roi = recipe.rois[selectedRoi];
  const mode = drawMode;
  dragStart = dragCur = null;
  drawMode = null;
  if (w < 4 || h < 4) { drawOverlay(); return; }

  if (mode === "region") {
    roi.rect = rect;
    toast(`Region for “${roi.name}” set to [${rect.join(", ")}]. Save the recipe to keep it.`);
    renderRoiList(); drawOverlay();
  } else if (mode === "teach") {
    try {
      await saveRecipe(true);
      const d = await api(`/api/recipes/${encodeURIComponent(recipe.name)}/teach`,
        { method: "POST", body: { roi_name: roi.name, rect, source: viewSource } });
      toast(`Template taught for “${roi.name}” (${d.size[0]}×${d.size[1]} px).`);
      await loadRecipe(recipe.name);
    } catch (err) { toast("Teach failed: " + err.message); }
  }
});

/* ------------------------------------------------------ setup buttons */
$("btn-roi-add").addEventListener("click", () => {
  if (!recipe) { toast("Create/select a recipe first."); return; }
  const kind = confirm("OK = template match ROI, Cancel = threshold zone ROI")
    ? "template" : "threshold";
  recipe.rois.push(defaultRoi(kind));
  selectedRoi = recipe.rois.length - 1;
  renderRoiList(); drawOverlay();
});

async function saveRecipe(silent) {
  if (!recipe) return;
  await api("/api/recipes/" + encodeURIComponent(recipe.name),
    { method: "PUT", body: { rois: recipe.rois } });
  if (!silent) toast("Recipe saved.");
}
$("btn-recipe-save").addEventListener("click", () => saveRecipe(false).catch((e) => toast(e.message)));

$("btn-recipe-new").addEventListener("click", async () => {
  const name = $("recipe-new-name").value.trim();
  if (!name) return;
  try {
    await api("/api/recipes/" + encodeURIComponent(name), { method: "POST" });
    await api("/api/recipes/" + encodeURIComponent(name) + "/activate", { method: "POST" });
    $("recipe-new-name").value = "";
    await refreshRecipeList();
  } catch (e) { toast(e.message); }
});

$("btn-recipe-delete").addEventListener("click", async () => {
  const name = $("recipe-select").value;
  if (!name || !confirm(`Delete recipe “${name}”?`)) return;
  await api("/api/recipes/" + encodeURIComponent(name), { method: "DELETE" });
  recipe = null; selectedRoi = -1;
  await refreshRecipeList();
  renderRoiList(); drawOverlay();
});

$("btn-recipe-activate").addEventListener("click", async () => {
  const name = $("recipe-select").value;
  if (!name) return;
  await api("/api/recipes/" + encodeURIComponent(name) + "/activate", { method: "POST" });
  await refreshRecipeList();
});

$("recipe-select").addEventListener("change", (e) => loadRecipe(e.target.value));

$("btn-test").addEventListener("click", async () => {
  try {
    await saveRecipe(true);
    await api("/api/recipes/" + encodeURIComponent(recipe.name) + "/activate", { method: "POST" });
    const d = await api("/api/inspect/test?source=" + viewSource, { method: "POST" });
    $("test-result").textContent = JSON.stringify(d, null, 2);
    toast(d.passed ? "TEST: PASS" : "TEST: FAIL");
    if (viewSource === "reference") {
      // show the server-annotated verdict on the reference image
      refAnnotated = true;
      $("setup-img").src = `/api/recipes/${encodeURIComponent(recipe.name)}` +
        `/reference.jpg?annotated=1&scale=0.5&_=${Date.now()}`;
      drawOverlay();
    }
  } catch (e) { toast("Test failed: " + e.message); }
});

$("btn-swtrig").addEventListener("click", () =>
  api("/api/camera/trigger", { method: "POST" }).catch((e) => toast(e.message)));

$("exposure").addEventListener("change", () =>
  api("/api/camera/exposure", { method: "POST", body: { exposure_us: parseFloat($("exposure").value) } })
    .catch((e) => toast(e.message)));
$("gain").addEventListener("change", () =>
  api("/api/camera/exposure", { method: "POST", body: { gain_db: parseFloat($("gain").value) } })
    .catch((e) => toast(e.message)));

/* -------------------------------------------------------------- run */
let running = false;
$("btn-run-toggle").addEventListener("click", async () => {
  try {
    const d = await api("/api/run", { method: "POST", body: { enabled: !running } });
    running = d.run_mode;
    updateRunButton();
  } catch (e) { alert(e.message); }
});
function updateRunButton() {
  const b = $("btn-run-toggle");
  b.textContent = running ? "STOP RUN" : "START RUN";
  b.style.background = running ? "#991b1b" : "";
}
$("btn-counters-reset").addEventListener("click", () =>
  api("/api/counters/reset", { method: "POST" }));

$("btn-log-apply").addEventListener("click", async () => {
  const mode = $("log-mode").value;
  const d = await api("/api/logging", { method: "POST",
    body: { mode, count: parseInt($("log-count").value) || 0 } });
  $("log-status").textContent = JSON.stringify(d);
});

/* ------------------------------------------------------------ review */
let reviewData = null;
let labelsData = null;      // {categories: [{name, expect}], labels: {file: cat}}
let selectedFile = null;
let reviewFolder = "";

function catByName(name) {
  return (labelsData?.categories || []).find((c) => c.name === name);
}

function rowAgreement(r) {
  /* '' = unlabeled/no expectation, true = recipe agrees with label */
  const cat = catByName(labelsData?.labels?.[r.file]);
  if (!cat || !cat.expect || r.error) return "";
  return (cat.expect === "pass") === !!r.passed;
}

function renderReviewList() {
  const list = $("review-list");
  list.innerHTML = "";
  if (!reviewData) return;
  let agree = 0, disagree = 0;
  reviewData.results.forEach((r) => {
    const row = document.createElement("div");
    const ag = rowAgreement(r);
    if (ag === true) agree++;
    if (ag === false) disagree++;
    row.className = "review-row" + (r.file === selectedFile ? " selected" : "")
      + (ag === false ? " disagree" : "");
    const label = labelsData?.labels?.[r.file];
    row.innerHTML = `<span>${r.file}</span>` +
      (label ? `<span class="labelchip">${label}</span>` : "") +
      `<span class="v ${r.passed ? "pass" : "fail"}">${r.error ? "ERR" : r.passed ? "PASS" : "FAIL"}</span>`;
    row.addEventListener("click", () => selectReviewRow(r));
    list.appendChild(row);
  });
  const pass = reviewData.results.filter((r) => r.passed).length;
  let txt = `${reviewData.count} images — ${pass} pass, ${reviewData.count - pass} fail`;
  if (agree + disagree > 0) {
    txt += ` · vs labels: ${agree} agree, ${disagree} DISAGREE`;
  }
  $("review-summary").textContent = txt;
}

function renderCatButtons() {
  const box = $("cat-buttons");
  box.innerHTML = "";
  (labelsData?.categories || []).forEach((c) => {
    const b = document.createElement("button");
    b.className = "catbtn" +
      (selectedFile && labelsData.labels[selectedFile] === c.name ? " assigned" : "");
    b.textContent = c.name + (c.expect ? (c.expect === "pass" ? " ✓" : " ✗") : "");
    b.title = c.expect ? `images in this category should ${c.expect.toUpperCase()}`
                       : "no pass/fail expectation";
    b.addEventListener("click", async () => {
      if (!selectedFile) { $("label-current").textContent = "select an image first"; return; }
      const cur = labelsData.labels[selectedFile];
      labelsData = await api("/api/labels", { method: "POST", body: {
        folder: reviewFolder,
        set_label: { file: selectedFile, category: cur === c.name ? "" : c.name },
      }});
      renderCatButtons(); renderReviewList();
    });
    box.appendChild(b);
  });
}

function selectReviewRow(r) {
  selectedFile = r.file;
  $("label-current").textContent = r.file +
    (labelsData?.labels?.[r.file] ? ` — labeled “${labelsData.labels[r.file]}”` : " — unlabeled");
  $("review-img").src = `/api/review/image?folder=${encodeURIComponent(reviewData.folder)}` +
    `&file=${encodeURIComponent(r.file)}&annotated=1&scale=0.5&_=${Date.now()}`;
  renderCatButtons(); renderReviewList();
}

$("btn-cat-add").addEventListener("click", async () => {
  const name = $("cat-new").value.trim();
  if (!name || !reviewFolder) return;
  const cats = (labelsData?.categories || []).filter((c) => c.name !== name);
  cats.push({ name, expect: $("cat-expect").value });
  labelsData = await api("/api/labels", { method: "POST",
    body: { folder: reviewFolder, categories: cats } });
  $("cat-new").value = "";
  renderCatButtons();
});

$("btn-review-run").addEventListener("click", async () => {
  const folder = $("review-folder").value.trim();
  if (!folder) return;
  $("review-summary").textContent = "Running…";
  try {
    reviewData = await api("/api/review/run", { method: "POST", body: { folder } });
    reviewFolder = folder;
    labelsData = await api("/api/labels?folder=" + encodeURIComponent(folder));
    selectedFile = null;
    $("label-panel").style.display = "";
    $("label-current").textContent = "select an image below";
    renderCatButtons();
    renderReviewList();
  } catch (e) { $("review-summary").textContent = "Error: " + e.message; }
});

/* ---------------------------------------------------------- settings */
async function loadSettings() {
  const s = await api("/api/status");
  applyStatus(s);
  try {
    const exp = await api("/api/camera/exposure");
    if (exp.exposure_us) $("exposure").value = Math.round(exp.exposure_us);
    if (exp.gain_db !== undefined) $("gain").value = exp.gain_db;
  } catch {}
}

$("btn-cam-apply").addEventListener("click", async () => {
  const d = await api("/api/camera/config", { method: "POST", body: {
    source: $("cfg-cam-source").value,
    serial: $("cfg-cam-serial").value,
    trigger_line: $("cfg-cam-line").value,
    sim_folder: $("cfg-cam-simfolder").value,
    rtsp_url: $("cfg-cam-rtsp").value,
  }});
  $("cam-info").textContent = JSON.stringify(d);
});

$("btn-plc-apply").addEventListener("click", async () => {
  const d = await api("/api/plc/config", { method: "POST", body: {
    backend: $("cfg-plc-backend").value,
    ams_net_id: $("cfg-plc-ams").value,
    ip_address: $("cfg-plc-ip").value,
    port: parseInt($("cfg-plc-port").value) || 851,
    slot: parseInt($("cfg-plc-slot").value) || 0,
  }});
  $("plc-info").textContent = JSON.stringify(d, null, 2);
});

$("btn-plc-test").addEventListener("click", async () => {
  try {
    const d = await api("/api/plc/test", { method: "POST" });
    $("plc-info").textContent = JSON.stringify(d, null, 2);
  } catch (e) { $("plc-info").textContent = "FAILED: " + e.message; }
});

/* ------------------------------------------------------ status + ws */
function applyStatus(s) {
  const cam = $("h-camera");
  cam.className = "pill " + (s.camera.connected ? "ok" : "bad");
  cam.textContent = "camera " + (s.camera.connected ? s.camera.mode : "offline");
  const plc = $("h-plc");
  const mock = s.plc.type === "MockPlc";
  plc.className = "pill " + (s.plc.connected ? (mock ? "" : "ok") : "bad");
  plc.textContent = mock ? "plc: mock" : "plc " + (s.plc.connected ? "ok" : "offline");
  const rec = $("h-recipe");
  rec.className = "pill " + (s.recipe ? "ok" : "bad");
  rec.textContent = s.recipe || "no recipe";
  running = s.run_mode;
  updateRunButton();
  $("c-total").textContent = s.counters.total;
  $("c-pass").textContent = s.counters.pass;
  $("c-fail").textContent = s.counters.fail;
  $("log-status").textContent = JSON.stringify(s.logging);
}

function applyResult(msg) {
  const r = msg.data;
  $("run-verdict").textContent = r.passed ? "PASS" : "FAIL";
  $("run-verdict").className = "verdict " + (r.passed ? "pass" : "fail");
  $("c-time").textContent = r.elapsed_ms + " ms";
  $("c-rid").textContent = r.result_id;
  $("run-result").textContent = JSON.stringify(r, null, 2);
  if (msg.counters) {
    $("c-total").textContent = msg.counters.total;
    $("c-pass").textContent = msg.counters.pass;
    $("c-fail").textContent = msg.counters.fail;
  }
}

function connectWs() {
  const ws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws");
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "status") applyStatus(msg.data);
    else if (msg.type === "result") applyResult(msg);
  };
  ws.onclose = () => setTimeout(connectWs, 2000);
}

/* --------------------------------------------------------------- boot */
(async function boot() {
  await refreshRecipeList().catch(() => {});
  await loadSettings().catch(() => {});
  connectWs();
  setTimeout(syncCanvas, 800);
})();
