/* Robot car web panel.  Talks only to web_panel.py: /events (Server-Sent
   Events: hello / status / log / clear_log) and POST /api/<action>.
   web_panel.py keeps all the car's state; this page only shows it. */
"use strict";

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

function el(tag, props = {}, ...kids) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(props)) {
    if (k === "class") n.className = v;
    else if (k === "text") n.textContent = v;
    else if (k === "style") n.style.cssText = v;
    else if (k.startsWith("on")) n.addEventListener(k.slice(2), v);
    else n.setAttribute(k, v);
  }
  for (const kid of kids) if (kid != null) n.append(kid);
  return n;
}

function setText(sel, text) {
  const n = typeof sel === "string" ? $(sel) : sel;
  if (n.textContent !== text) n.textContent = text;
}

/* ---- preferences (this browser only) ------------------------------------ */
const PREF_KEY = "robotcar-panel-v1";
const prefs = Object.assign(
  { tab: "drive", follow: true, stamp: false, raw: false, hideHb: true, show: {} },
  (() => { try { return JSON.parse(localStorage.getItem(PREF_KEY)) || {}; } catch { return {}; } })());
function savePrefs() {
  try { localStorage.setItem(PREF_KEY, JSON.stringify(prefs)); } catch { /* private mode */ }
}

/* ---- talking to web_panel.py --------------------------------------------- */
async function post(action, body = {}) {
  try {
    const r = await fetch("/api/" + action, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const j = await r.json().catch(() => ({ ok: false, error: "Bad answer from the panel." }));
    if (!j.ok) toast(j.error || "Refused.", "bad");
    return j.ok;
  } catch {
    toast("The panel program is not reachable: is web_panel.py still running?", "bad");
    return false;
  }
}
const cmd = (word) => post("cmd", { word });

function toast(text, tone = "info") {
  const t = el("div", { class: "toast " + tone, text });
  $("#toasts").append(t);
  while ($("#toasts").children.length > 4) $("#toasts").firstChild.remove();
  setTimeout(() => { t.classList.add("out"); setTimeout(() => t.remove(), 350); },
             tone === "bad" ? 7000 : 3500);
}

function flash(btn) {
  btn.classList.remove("flash");
  void btn.offsetWidth;
  btn.classList.add("flash");
}

/* confirm dialog: resolves true on OK */
function confirmBox(title, html, okText = "OK") {
  const dlg = $("#confirm");
  setText("#confirm-title", title);
  $("#confirm-body").innerHTML = html;          // only fixed text from this file
  setText("#confirm-ok", okText);
  dlg.returnValue = "";
  dlg.showModal();
  $("#confirm-ok").focus();
  return new Promise((resolve) => {
    dlg.addEventListener("close", () => resolve(dlg.returnValue === "ok"), { once: true });
  });
}

/* ---- names ------------------------------------------------------------------ */
const STATE_TEXT = {
  init: "Init", idle: "Idle", calibrate: "Calibrating", line_follow: "Line following",
  junction: "Junction", hump: "Hump", obstacle: "Obstacle bypass",
  line_search: "Line search", finished: "Finished", fault: "FAULT",
};
const RUNNING = new Set(["line_follow", "junction", "hump", "obstacle", "line_search"]);
const SOFT_FAULTS = new Set(["OBST_STOP", "WIFI_DOWN"]);   // report / hold only
const CAT_SHORT = {
  left: "L line", right: "R line", barcode: "Barcode", line: "Line", tilt: "IMU",
  compass: "Compass", ultrasonic: "Obstacle", encoders: "Motors", state: "State",
  events: "Map", net: "Net", diag: "Diag", other: "Other",
};

let CFG = null;           // fixed values from the server
let ST = null;            // latest status
let built = false;        // forms built and filled once
let lastReplyId = null;   // replies up to this one have been toasted

/* ---- live updates ------------------------------------------------------------- */
function connect() {
  const es = new EventSource("/events");
  es.addEventListener("hello", (e) => {
    const d = JSON.parse(e.data);
    $("#offline-banner").hidden = true;
    CFG = d.config;
    if (!built) { buildForms(d.status); built = true; }
    const ids = d.status.replies.map((r) => r.id);
    lastReplyId = ids.length ? Math.max(...ids) : 0;     // don't toast old replies
    records = d.records;
    renderLog();
    applyStatus(d.status);
  });
  es.addEventListener("status", (e) => applyStatus(JSON.parse(e.data)));
  es.addEventListener("log", (e) => appendLog(JSON.parse(e.data)));
  es.addEventListener("clear_log", () => { records = []; renderLog(); });
  es.onerror = () => { $("#offline-banner").hidden = false; };
}

/* ---- status ------------------------------------------------------------------- */
function pill(sel, info) {
  const p = $(sel);
  p.className = "pill " + info.tone;
  setText($("b", p), info.text);
}

function applyStatus(s) {
  ST = s;
  pill("#conn", s.conn);
  pill("#online", s.online);

  const stp = $("#state");
  stp.className = "pill state" + (s.state === "fault" ? " fault" : RUNNING.has(s.state) ? " run" : "");
  setText(stp, "State: " + (STATE_TEXT[s.state] || s.state || "?"));

  const bi = $("#broker");
  if (document.activeElement !== bi && !bi.dataset.edited) bi.value = s.broker;

  // why stopped
  const w = s.why;
  const card = $("#why");
  card.className = "card why kind-" + w.kind + (w.stale ? " stale" : "");
  setText("#why-head", w.head);
  setText("#why-detail", w.detail);
  setText("#why-last", w.last);
  setText("#why-since", w.since ? "Since " + w.since : "");
  $("#why-stale").hidden = !w.stale;
  setText("#why-stale", w.stale || "");
  const chips = $("#why-faults");
  const key = s.faults.join(",");
  if (chips.dataset.key !== key) {
    chips.dataset.key = key;
    chips.replaceChildren(...s.faults.map((f) =>
      el("span", { class: "chip" + (SOFT_FAULTS.has(f) ? " soft" : ""), text: f })));
  }

  // speed
  const sp = s.speed;
  const v = $("#spd-v");
  setText(v, sp.v == null ? "\u2013" : sp.v + " mm/s");
  v.className = "big " + (sp.v == null ? "" : sp.stale != null ? "dim" : sp.v ? "live" : "");
  const lvl = $("#spd-level");
  const lvlKey = sp.level || "";
  if (lvl.dataset.key !== lvlKey) {
    lvl.dataset.key = lvlKey;
    lvl.replaceChildren(sp.level ? el("span", { class: "chip level", text: "speed level: " + sp.level }) : "");
  }
  setText("#spd-stats", sp.v == null ? "no telemetry yet"
    : `this run: max ${sp.max}, avg ${sp.avg} mm/s, ${sp.dist_m.toFixed(2)} m in ${sp.time_s} s`);
  setText("#spd-at", sp.stale != null ? `no telemetry for ${sp.stale} s` : sp.at ? "at " + sp.at : "");

  // barcode
  const bc = $("#bc-letter");
  setText(bc, s.barcode.letter || "\u2013");
  bc.className = "big letter" + (s.barcode.letter ? " gold" : "");
  setText("#bc-detail", s.barcode.detail);
  setText("#bc-at", s.barcode.at);

  // hump
  const hh = $("#hump-h");
  setText(hh, s.hump.text || "\u2013");
  hh.className = "big" + (s.hump.climbing ? " hot" : s.hump.text ? " good" : "");
  setText("#hump-info", s.hump.info);
  setText("#hump-at", s.hump.at);
  setText("#wheel-now", s.wheel_now);
  setText("#acc-raw", s.acc_raw);

  // obstacle width
  const ow = $("#ow-mm");
  if (s.width.measuring) {
    setText(ow, "measuring\u2026");
    ow.className = "big hot";
    setText("#ow-note", "stopped to sweep both edges (about 6 s)");
  } else {
    setText(ow, s.width.mm ? s.width.mm + " mm" : "\u2013");
    ow.className = "big" + (s.width.mm ? " good" : "");
    setText("#ow-note", s.width.mm ? "last obstacle" : "measured about 40 cm before an obstacle");
  }

  // bypass side and depth
  for (const b of $$("#avoid-seg button")) {
    b.setAttribute("aria-pressed", String(b.dataset.side === s.avoid.side));
  }
  note("#avoid-note", "Car has: " + (s.avoid.side ? s.avoid.side.toUpperCase() : "?") +
       " \u2014 " + s.avoid.note, s.avoid.tone);
  note("#depth-note", "Car has: " + (s.depth.mm ? s.depth.mm + " mm" : "?") +
       " \u2014 " + s.depth.note, s.depth.tone);
  note("#map-note", s.map_note.text, s.map_note.tone);

  renderReplies(s.replies);
  renderEdge(s.edge);
}

function note(sel, text, tone) {
  const n = $(sel);
  n.className = "note small " + (tone || "");
  setText(n, text);
}

function renderReplies(list) {
  const ol = $("#replies");
  const key = list.length ? list[list.length - 1].id : 0;
  if (ol.dataset.key !== String(key)) {
    ol.dataset.key = String(key);
    if (!list.length) {
      ol.replaceChildren(el("li", { class: "empty", text: "Nothing yet. Replies from the car appear here." }));
    } else {
      ol.replaceChildren(...list.slice().reverse().map((r) =>
        el("li", { class: r.tone }, el("time", { text: r.t }), el("span", { text: r.text }))));
    }
  }
  if (lastReplyId == null) return;
  for (const r of list) {
    if (r.id > lastReplyId) {
      toast(r.text, r.tone);
      lastReplyId = r.id;
    }
  }
}

/* ---- forms ---------------------------------------------------------------- */
function buildForms(s) {
  const L = CFG.limits;

  // tabs
  for (const b of $$(".tabs button")) b.addEventListener("click", () => showTab(b.dataset.tab));
  for (const a of $$("[data-goto]")) a.addEventListener("click", (e) => { e.preventDefault(); showTab(a.dataset.goto); });
  showTab(prefs.tab);

  // drive
  $("#btn-start").addEventListener("click", (e) => { flash(e.currentTarget); cmd("start"); });
  $("#btn-stop").addEventListener("click", stop);
  $("#btn-mode").addEventListener("click", () => cmd("mode"));
  $("#btn-calibrate").addEventListener("click", calibrate);
  $("#btn-snapshot").addEventListener("click", () => post("snapshot"));
  $("#btn-clear-readings").addEventListener("click", () => post("clear_readings"));

  $("#broker").addEventListener("input", (e) => { e.target.dataset.edited = "1"; });
  $("#broker-form").addEventListener("submit", async (e) => {
    e.preventDefault();
    const bi = $("#broker");
    if (await post("connect", { broker: bi.value.trim() })) delete bi.dataset.edited;
  });

  // mapping
  const rows = $("#map-rows");
  for (const [label, word] of CFG.moves) {
    const sel = el("select", { id: "map-" + word, "data-word": word });
    for (const l of CFG.letters) sel.append(el("option", { value: l, text: l }));
    sel.addEventListener("change", checkMap);
    rows.append(el("div", { class: "map-cell" }, el("label", { for: "map-" + word, text: label }), sel));
  }
  const seg = $("#map-default");
  for (const [label, word] of CFG.moves) {
    seg.append(el("button", { type: "button", "data-word": word, "aria-pressed": "false", text: label,
                              onclick: () => setDefaultMove(word) }));
  }
  fillMap(s.mapping);
  $("#btn-map-send").addEventListener("click", sendMap);
  $("#btn-map-reset").addEventListener("click", () => fillMap(Object.assign({}, CFG.default_map,
                                                                       { default: CFG.default_move })));

  // bypass
  for (const b of $$("#avoid-seg button")) b.addEventListener("click", () => cmd("avoid " + b.dataset.side));
  const [dmin, dmax, ddef] = L.depth;
  const dr = $("#depth-range"), di = $("#depth-in");
  for (const n of [dr, di]) { n.min = dmin; n.max = dmax; }
  dr.value = di.value = s.depth.mm || ddef;
  dr.addEventListener("input", () => { di.value = dr.value; di.classList.remove("invalid"); });
  di.addEventListener("input", () => { if (+di.value >= dmin && +di.value <= dmax) dr.value = di.value; });
  setText("#depth-help", `${dmin} to ${dmax} mm. Shorter turns back to the line sooner after the box. The car starts with ${ddef} mm.`);
  $("#depth-form").addEventListener("submit", (e) => {
    e.preventDefault();
    const mm = intIn(di, dmin, dmax);
    if (mm == null) return toast(`Box depth: a whole number of mm, ${dmin} to ${dmax}.`, "bad");
    cmd("depth " + mm);
  });

  // width calibration
  const wi = $("#wcal-in");
  wi.min = L.wcal[0]; wi.max = L.wcal[1];
  $("#wcal-form").addEventListener("submit", calibrateWidth);
  $("#btn-wcal-clear").addEventListener("click", clearRuns);

  // servo trim
  const [tmin, tmax, tstep] = L.trim;
  const ti = $("#trim-in");
  ti.min = tmin; ti.max = tmax; ti.value = s.trim;
  const nudge = (d) => {
    const cur = Number.isInteger(+ti.value) && ti.value !== "" ? +ti.value : 0;
    ti.value = Math.max(tmin, Math.min(tmax, cur + d));
    ti.classList.remove("invalid");
    cmd("servo " + ti.value);
  };
  $("#trim-left").addEventListener("click", () => nudge(+tstep));
  $("#trim-right").addEventListener("click", () => nudge(-tstep));
  $("#trim-form").addEventListener("submit", (e) => {
    e.preventDefault();
    const us = intIn(ti, tmin, tmax);
    if (us == null) return toast(`Servo trim: a whole number of us, ${tmin} to ${tmax}.`, "bad");
    cmd("servo " + us);
  });

  // TEMPORARY (19 Nov): servo end calibration
  const [emin, emax, estep, edeg] = L.servo_end;
  setText("#send-deg", String(edeg));
  for (const [side, id] of [["left", "#send-l"], ["right", "#send-r"]]) {
    const n = $(id);
    n.min = emin; n.max = emax; n.step = estep;
    const go = () => {
      const us = intIn(n, emin, emax);
      if (us == null) return toast(`Servo ${side} end: a whole number of us, ${emin} to ${emax}.`, "bad");
      cmd(`servoend ${side} ${us}`);
    };
    $(id + "-go").addEventListener("click", go);
    n.addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); go(); } });
  }
  $("#send-form").addEventListener("submit", (e) => e.preventDefault());
  $("#send-ahead").addEventListener("click", () => cmd("servoend ahead"));

  // TEMPORARY (20 Nov): raw scan
  $("#btn-rscan").addEventListener("click", () => cmd("rscan"));

  // edge calibration
  setText("#spread", String(L.spread));
  for (const id of ["#edge-l", "#edge-r"]) {
    const n = $(id);
    n.min = L.edge[0]; n.max = L.edge[1];
    n.addEventListener("input", edgeSum);
  }
  edgeSum();
  $("#btn-esweep").addEventListener("click", () => cmd("esweep"));
  $("#edge-form").addEventListener("submit", addEdgeRun);
  $("#btn-edge-clear").addEventListener("click", clearRuns);
  $("#btn-copy-consts").addEventListener("click", copyConsts);

  buildLogControls();

  // Esc = STOP from anywhere
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") stop();
  });
}

function showTab(name) {
  if (!$(`.panel[data-panel="${name}"]`)) name = "drive";
  for (const b of $$(".tabs button")) b.setAttribute("aria-selected", String(b.dataset.tab === name));
  for (const p of $$(".panel")) p.hidden = p.dataset.panel !== name;
  prefs.tab = name;
  savePrefs();
}

function intIn(input, lo, hi) {
  const t = input.value.trim();
  const ok = /^-?\d+$/.test(t) && +t >= lo && +t <= hi;
  input.classList.toggle("invalid", !ok);
  return ok ? +t : null;
}

function stop() {
  flash($("#btn-stop"));
  cmd("stop");
}

async function calibrate() {
  const ok = await confirmBox("Calibrate",
    `<p>Put the car <b>on the line</b>, with about 25 cm free all round.</p>
     <p>After OK, don't touch it. It stays still for about 1 s, then spins a full circle by itself
     (about 4 s). It ends by itself with a beep and a blue LED.</p>
     <p class="muted small">Firmware built with CAL_AUTO_SPIN 0: turn it a full circle by hand
     across the line within about 14 s instead.</p>`, "Calibrate");
  if (ok) cmd("calibrate");
}

/* mapping */
function fillMap(m) {
  for (const sel of $$("#map-rows select")) sel.value = m[sel.dataset.word];
  setDefaultMove(m.default || CFG.default_move);
  checkMap();
}
function setDefaultMove(word) {
  for (const b of $$("#map-default button")) b.setAttribute("aria-pressed", String(b.dataset.word === word));
}
function checkMap() {
  const sels = $$("#map-rows select");
  const count = {};
  for (const s of sels) count[s.value] = (count[s.value] || 0) + 1;
  let dup = false;
  for (const s of sels) {
    const bad = count[s.value] > 1;
    s.classList.toggle("invalid", bad);
    dup = dup || bad;
  }
  $("#map-error").hidden = !dup;
  setText("#map-error", "Each movement needs a different letter.");
  $("#btn-map-send").disabled = dup;
  return !dup;
}
function sendMap() {
  if (!checkMap()) return;
  const body = {};
  for (const s of $$("#map-rows select")) body[s.dataset.word] = s.value;
  const d = $("#map-default button[aria-pressed=true]");
  body.default = d ? d.dataset.word : CFG.default_move;
  post("map", body);
}

/* width calibration */
async function calibrateWidth(e) {
  e.preventDefault();
  const [lo, hi] = CFG.limits.wcal;
  const mm = intIn($("#wcal-in"), lo, hi);
  const err = $("#wcal-error");
  err.hidden = mm != null;
  if (mm == null) { setText(err, `Enter the box width as a whole number of mm, ${lo} to ${hi}.`); return; }
  const ok = await confirmBox("Calibrate width",
    `<ul>
       <li>Car in <b>Idle</b> (blue LED), not moving.</li>
       <li>A box exactly <b>${mm} mm</b> wide in front of it, centred on the car, flat face square to the car,
           about <b>40 cm</b> from the ultrasonic sensor.</li>
     </ul>
     <p>After OK the servo sweeps for about 6 s and the run is added to the ones since power-up
     (Clear runs starts again). Each side is fitted on its own once its edge positions are
     ${CFG.limits.spread} mm apart: use boxes of different widths, or Edge calibration, which lets you
     shift one box sideways.</p>
     <p>The log then shows <code>set OW_SL_E4 .. OW_KR_E4 ..</code>: copy all four into
     <code>app_program/subsystem5/obst_width.h</code> and rebuild. The car uses them straight away,
     until power-off.</p>`, "Start sweep");
  if (ok) cmd("wcal " + mm);
}

async function clearRuns() {
  const ok = await confirmBox("Clear runs",
    "<p>Forget every width calibration run on the car? The constants it uses now stay.</p>", "Clear runs");
  if (ok) cmd("wcal clear");
}

/* edge calibration */
function edgeSum() {
  const l = $("#edge-l").value, r = $("#edge-r").value;
  const [bmin, bmax] = CFG.limits.box;
  const ok = /^-?\d+$/.test(l) && /^-?\d+$/.test(r);
  const sum = ok ? +l + +r : null;
  const n = $("#edge-sum");
  setText(n, sum == null ? "\u2013" : sum + " mm");
  n.classList.toggle("error", sum != null && (sum < bmin || sum > bmax));
}

function addEdgeRun(e) {
  e.preventDefault();
  const [lo, hi] = CFG.limits.edge;
  const [bmin, bmax] = CFG.limits.box;
  const l = intIn($("#edge-l"), lo, hi), r = intIn($("#edge-r"), lo, hi);
  const err = $("#edge-error");
  let msg = null;
  if (l == null || r == null) msg = `Each edge must be a whole number of mm, ${lo} to ${hi}.`;
  else if (l + r < bmin || l + r > bmax) msg = `Left + right is the box width: it must be ${bmin} to ${bmax} mm.`;
  err.hidden = !msg;
  if (msg) { setText(err, msg); return; }
  cmd(`wcal ${l} ${r}`);
}

function copyConsts() {
  const text = $("#edge-consts").textContent;
  const done = () => toast("Copied: " + text, "ok");
  if (navigator.clipboard && window.isSecureContext) {
    navigator.clipboard.writeText(text).then(done, () => toast("Copy failed: select the text instead.", "bad"));
  } else {
    const r = document.createRange();
    r.selectNodeContents($("#edge-consts"));
    getSelection().removeAllRanges();
    getSelection().addRange(r);
    document.execCommand("copy") ? done() : toast("Select the text and press Ctrl+C.", "info");
  }
}

let edgeKey = "";
const SVGNS = "http://www.w3.org/2000/svg";
function svg(tag, attrs, text) {
  const n = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs)) n.setAttribute(k, v);
  if (text != null) n.textContent = text;
  return n;
}

function renderEdge(d) {
  const key = JSON.stringify(d);
  if (key === edgeKey) return;
  edgeKey = key;

  setText("#edge-status", d.status);
  const consts = $("#edge-consts");
  if (d.fit) {
    const [, sl, kl, sr, kr] = d.fit;
    setText(consts, `OW_SL_E4 ${sl}  OW_KL_E4 ${kl}  OW_SR_E4 ${sr}  OW_KR_E4 ${kr}`);
  } else {
    setText(consts, "no fit yet");
  }
  $("#btn-copy-consts").disabled = !d.fit;

  const tb = $("#edge-runs");
  if (!d.runs.length) {
    tb.replaceChildren(el("tr", {}, el("td", { colspan: "8", class: "muted", text: "No calibration runs yet." })));
  } else {
    tb.replaceChildren(...d.runs.map((r) => el("tr", {}, ...[r.n, r.face, r.ldeg, r.lknown, r.lreads,
      r.rdeg, r.rknown, r.rreads].map((v) => el("td", { text: String(v) })))));
  }

  // drawing, seen from above
  const W = 560, H = 430, S = 0.75, OX = 280, OY = 400;
  const { pivot, side_max } = CFG.edge_geom;
  const pt = (deg, r) => {
    const a = deg * Math.PI / 180, rr = r + pivot;
    return [OX - rr * Math.sin(a) * S, OY - rr * Math.cos(a) * S];
  };
  const latX = (lat) => OX - lat * S;
  const g = $("#edge-svg");
  const kids = [];
  const line = (x1, y1, x2, y2, stroke, extra = {}) =>
    kids.push(svg("line", Object.assign({ x1, y1, x2, y2, stroke }, extra)));
  const text = (x, y, t, fill, extra = {}) =>
    kids.push(svg("text", Object.assign({ x, y, fill, "font-size": 15 }, extra), t));

  for (let f = 100; f < 600; f += 100) {
    const y = OY - f * S;
    line(0, y, W, y, "#1a222c");
    text(4, y - 4, f + " mm", "#5d6874", { "font-size": 13 });
  }
  for (let lat = -300; lat <= 300; lat += 100) line(latX(lat), 0, latX(lat), OY, "#1a222c");
  line(OX, 0, OX, OY, "#3a4654", { "stroke-dasharray": "4 3" });
  for (const side of [1, -1]) {
    const [x, y] = pt(side * side_max, 520);
    line(OX, OY, x, y, "#2a3440", { "stroke-dasharray": "2 4" });
  }
  text(8, 16, "LEFT", "#8b98a5", { "font-weight": 700 });
  text(W - 8, 16, "RIGHT", "#8b98a5", { "font-weight": 700, "text-anchor": "end" });
  kids.push(svg("rect", { x: latX(75), y: OY + 4, width: 150 * S, height: H - OY - 4,
                          fill: "#1c242d", stroke: "#3a4654" }));
  kids.push(svg("circle", { cx: OX, cy: OY, r: 4, fill: "#e6edf3" }));

  for (const [deg, r, face] of d.looks) {
    const [x, y] = pt(deg, Math.min(r, 600));
    const c = face ? "#2fbfa8" : "#59636e";
    line(OX, OY, x, y, c, { "stroke-width": face ? 2 : 1 });
    kids.push(svg("circle", { cx: x, cy: y, r: 3, fill: c }));
  }
  if (d.face) {
    const yf = OY - (d.face + pivot) * S;
    line(0, yf, W, yf, "#2fbfa8", { "stroke-dasharray": "6 3" });
    text(W - 6, yf - 6, `face ${d.face} mm`, "#2fbfa8", { "text-anchor": "end" });
    if (d.known) {
      const xl = latX(d.known[0]), xr = latX(-d.known[1]);
      line(xl, yf, xr, yf, "#4c9aff", { "stroke-width": 5 });
      text((xl + xr) / 2, yf + 18, `real ${d.known[0] + d.known[1]} mm`, "#4c9aff",
           { "text-anchor": "middle", "font-weight": 700 });
    }
    if (d.edges) {
      for (const [lat, oov] of [[d.edges[0], d.oov[0]], [-d.edges[1], d.oov[1]]]) {
        const x = latX(lat);
        kids.push(svg("polygon", { points: `${x},${yf + 2} ${x - 6},${yf + 14} ${x + 6},${yf + 14}`,
                                   fill: oov ? "#f85149" : "#f0883e" }));
      }
      if (d.edges[2] > 0) {
        text(OX, yf - 22, `measured ${d.edges[2]} mm`, "#f0883e",
             { "text-anchor": "middle", "font-size": 18, "font-weight": 700 });
      }
    }
  }
  if (d.bound) {
    for (const [deg, oov] of [[d.bound[0], d.oov[0]], [-d.bound[1], d.oov[1]]]) {
      const [x, y] = pt(deg, 560);
      line(OX, OY, x, y, oov ? "#f85149" : "#f0883e", { "stroke-dasharray": "5 3" });
      text(x, y - 6, `${Math.abs(deg).toFixed(1)}\u00b0${oov ? " (out of view)" : ""}`, "#f0883e",
           { "text-anchor": "middle" });
    }
  }
  if (!d.looks.length) {
    text(OX, 140, "No sweep yet: car in Idle, box in front, then Measure only.", "#8b98a5",
         { "text-anchor": "middle", "font-size": 16 });
  }
  text(OX, H - 6, "car (sensor at the dot)", "#8b98a5", { "text-anchor": "middle", "font-size": 13 });
  g.replaceChildren(...kids);
}

/* ---- console log -------------------------------------------------------- */
let records = [];
let findText = "";
let catBtns = {};

function buildLogControls() {
  const cats = $("#cats");
  for (const [key, label] of CFG.categories) {
    if (!(key in prefs.show)) prefs.show[key] = true;
    const b = el("button", { class: "cat", type: "button", title: label, "aria-pressed": String(prefs.show[key]) },
      el("i", { style: "background:" + CFG.colours[key] }), label);
    b.addEventListener("click", (e) => {
      if (e.ctrlKey || e.metaKey) {                 // Ctrl+click: this one only
        for (const k of Object.keys(prefs.show)) prefs.show[k] = k === key;
      } else {
        prefs.show[key] = !prefs.show[key];
      }
      syncCats();
      renderLog();
    });
    catBtns[key] = b;
    cats.append(b);
  }
  cats.title = "Click to show or hide. Ctrl+click shows only that one.";
  $("#cats-all").addEventListener("click", () => setAllCats(true));
  $("#cats-none").addEventListener("click", () => setAllCats(false));

  const opts = { "#opt-follow": "follow", "#opt-stamp": "stamp", "#opt-raw": "raw", "#opt-hb": "hideHb" };
  for (const [sel, key] of Object.entries(opts)) {
    const c = $(sel);
    c.checked = prefs[key];
    c.addEventListener("change", () => {
      prefs[key] = c.checked;
      savePrefs();
      if (key === "follow") { if (c.checked) toBottom(); }
      else renderLog();
    });
  }
  let t = null;
  $("#log-find").addEventListener("input", (e) => {
    clearTimeout(t);
    t = setTimeout(() => { findText = e.target.value.trim().toLowerCase(); renderLog(); }, 120);
  });
  $("#log").addEventListener("scroll", onLogScroll);
  $("#btn-jump").addEventListener("click", () => setFollow(true));
  $("#btn-log-clear").addEventListener("click", () => post("clear_log"));
  $("#btn-log-save").addEventListener("click", downloadLog);
}

function syncCats() {
  for (const [k, b] of Object.entries(catBtns)) b.setAttribute("aria-pressed", String(!!prefs.show[k]));
  savePrefs();
}
function setAllCats(on) {
  for (const k of Object.keys(prefs.show)) prefs.show[k] = on;
  syncCats();
  renderLog();
}

function isFault(text) {
  const low = text.toLowerCase();
  return (low.includes("fault") && !low.includes("faults: none")) || low.includes("error") || low.includes("fail");
}

/* the rows one record shows with the current filters: [cat, text] */
function rowsFor(rec) {
  if (prefs.hideHb && rec.hb) return [];
  let rows;
  if (prefs.raw) {
    if (!rec.e.some(([c]) => prefs.show[c])) return [];
    rows = [[rec.e.length ? rec.e[0][0] : "other", rec.raw]];
  } else {
    rows = rec.e.filter(([c]) => prefs.show[c]);
  }
  if (findText) rows = rows.filter(([, t]) => t.toLowerCase().includes(findText));
  return rows;
}

function textWithMarks(text) {
  const span = el("span", { class: "txt" });
  if (!findText) { span.textContent = text; return span; }
  const low = text.toLowerCase();
  let i = 0;
  for (;;) {
    const j = low.indexOf(findText, i);
    if (j < 0) break;
    span.append(text.slice(i, j), el("mark", { text: text.slice(j, j + findText.length) }));
    i = j + findText.length;
  }
  span.append(text.slice(i));
  return span;
}

function recNode(rec) {
  const rows = rowsFor(rec);
  if (!rows.length) return null;
  const box = el("div", { class: "rec", "data-id": String(rec.id) });
  for (const [cat, text] of rows) {
    const cls = "row" + (text.includes("BARCODE READ:") ? " bcread" : isFault(text) ? " fault" : "");
    const row = el("div", { class: cls, style: "--c:" + (CFG.colours[cat] || "#ddd") });
    if (prefs.stamp) row.append(el("span", { class: "ts", text: rec.t }));
    row.append(el("span", { class: "tag", style: "color:" + (CFG.colours[cat] || "#ddd"), text: CAT_SHORT[cat] || cat }));
    row.append(textWithMarks(text));
    box.append(row);
  }
  return box;
}

let shown = 0;
function renderLog() {
  const log = $("#log");
  const frag = document.createDocumentFragment();
  shown = 0;
  for (const rec of records) {
    const n = recNode(rec);
    if (n) { frag.append(n); shown++; }
  }
  log.replaceChildren(frag);
  if (!shown) {
    log.append(el("div", { class: "empty", text: records.length
      ? "No lines match the filters."
      : "No console lines yet. They appear here once the car is connected to the broker." }));
  }
  logCount();
  if (prefs.follow) toBottom();
}

function appendLog(recs) {
  const log = $("#log");
  const empty = $(".empty", log);
  if (empty) empty.remove();
  const frag = document.createDocumentFragment();
  for (const rec of recs) {
    records.push(rec);
    const n = recNode(rec);
    if (n) { frag.append(n); shown++; }
  }
  log.append(frag);
  const extra = records.length - CFG.log_max;
  if (extra > 0) {
    records.splice(0, extra);
    const minId = records[0].id;
    while (log.firstChild && log.firstChild.dataset && +log.firstChild.dataset.id < minId) {
      log.firstChild.remove();
      shown--;
    }
  }
  if (!shown && !$(".empty", log)) {
    log.append(el("div", { class: "empty", text: "No lines match the filters." }));
  }
  logCount();
  if (prefs.follow) toBottom();
  else if (recs.length) $("#btn-jump").hidden = false;
}

function logCount() {
  setText("#log-count", shown === records.length && !findText
    ? `${records.length} lines`
    : `${shown} of ${records.length} lines shown`);
}

let autoScrolling = false;
function toBottom() {
  const log = $("#log");
  autoScrolling = true;
  log.scrollTop = log.scrollHeight;
  requestAnimationFrame(() => { autoScrolling = false; });
  $("#btn-jump").hidden = true;
}
function onLogScroll() {
  if (autoScrolling) return;
  const log = $("#log");
  const atEnd = log.scrollTop + log.clientHeight >= log.scrollHeight - 24;
  if (prefs.follow && !atEnd) setFollow(false);          // scrolled up: stay there
  else if (!prefs.follow && atEnd) setFollow(true);      // back at the end
}
function setFollow(on) {
  prefs.follow = on;
  $("#opt-follow").checked = on;
  savePrefs();
  if (on) toBottom();
}

function downloadLog() {
  const label = Object.fromEntries(CFG.categories);
  const lines = [];
  for (const rec of records) {
    if (prefs.raw) { lines.push(`${rec.t}  ${rec.raw}`); continue; }
    for (const [c, t] of rec.e) lines.push(`${rec.t}  ${(label[c] + ":").padEnd(26)} ${t}`);
  }
  const blob = new Blob([lines.join("\r\n") + "\r\n"], { type: "text/plain" });
  const a = el("a", { href: URL.createObjectURL(blob),
                      download: "car_log_" + new Date().toISOString().slice(0, 19).replace(/[-:T]/g, "") + ".txt" });
  document.body.append(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
}

connect();
