// The live page: the bird flying in real time on the flight model (src/physics: strip model, 6-DOF, controller),
// or playing back a flight log, over the Bow Valley; one button per subsystem opens it in the studio with
// exploded and cutaway views and its own live numbers; the gates as they stand, failures included. Left alone for
// 25 s it tours itself with a cursor; any touch, click, drag, wheel or key hands control back.
import * as THREE from 'three';
import { model, DATA } from './data.js';
import { makeMaterials, ACCENT } from './materials.js';
import { buildBird, SUBSYSTEMS } from './bird.js';
import { createStage, loadValley } from './stage.js';
import { numbers, gateLine } from './numbers.js';
import { setupFlight } from '../../src/physics/scenarios.js';
import { eulerFromQuat } from '../../src/physics/flight.js';

const DEG = 180 / Math.PI;
const N = numbers(model, DATA);
const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
const fmt = (v, d = 0) => (v == null || !Number.isFinite(v) ? '—' : v.toLocaleString('en-US', { minimumFractionDigits: d, maximumFractionDigits: d }));

const LABEL = { wing: 'Wing', drive: 'Drive', twist: 'Twist', power: 'Power', avionics: 'Avionics', structure: 'Structure' };
const ICON = {
  wing: '<path d="M2 14c5-1 9-5 11-9 2 4 6 8 9 9-5 0-8-2-9-4-1 2-5 4-11 4z"/>',
  drive: '<circle cx="12" cy="12" r="4"/><path d="M12 2v4M12 18v4M2 12h4M18 12h4M5 5l3 3M16 16l3 3M5 19l3-3M16 8l3-3"/>',
  twist: '<path d="M4 12a8 8 0 0 1 14-5M20 12a8 8 0 0 1-14 5"/><path d="M18 3v4h-4M6 21v-4h4"/>',
  power: '<rect x="3" y="7" width="16" height="10" rx="2"/><path d="M21 10v4M8 12h6M11 9v6"/>',
  avionics: '<rect x="6" y="6" width="12" height="12" rx="2"/><path d="M9 2v4M15 2v4M9 18v4M15 18v4M2 9h4M2 15h4M18 9h4M18 15h4"/>',
  structure: '<path d="M3 12h18M7 8l-4 4 4 4M17 8l4 4-4 4"/>',
};

const CSS = `
:root{color-scheme:dark;--bg:hsl(212 22% 6%);--card:hsl(212 20% 10% / .74);--line:hsl(212 15% 18%);--ink:hsl(210 25% 95%);--muted:hsl(212 12% 68%);--acc:${ACCENT};--bad:#ff8a75;--ok:#b9e3c6;
  --r:16px;--spring:cubic-bezier(.34,1.56,.64,1);--out:cubic-bezier(.22,1,.36,1)}
@media (prefers-color-scheme: light){:root{--card:hsl(212 20% 10% / .8)}}
html,body{margin:0;height:100%;background:var(--bg);color:var(--ink);overflow:hidden;-webkit-font-smoothing:antialiased}
#app{position:fixed;inset:0;font-family:Inter,system-ui,sans-serif}
#app *{font-family:inherit}
#gl{position:absolute;inset:0;width:100%;height:100%;display:block;touch-action:none}
.glass{background:var(--card);backdrop-filter:blur(20px) saturate(1.4);-webkit-backdrop-filter:blur(20px) saturate(1.4);border-radius:22px;box-shadow:0 18px 40px #0007,inset 0 1px 0 hsl(0 0% 100% / .05)}
.top{position:absolute;left:20px;top:18px;right:20px;display:flex;align-items:flex-start;gap:16px;pointer-events:none}
.brand{pointer-events:auto}
.brand h1{margin:0;font-size:30px;font-weight:800;letter-spacing:-.03em;line-height:1}
.brand p{margin:6px 0 0;font-size:14px;color:var(--muted)}
.honest{display:inline-flex;align-items:center;gap:8px;margin-top:10px;padding:6px 12px;border-radius:999px;font-size:13px;font-weight:600;letter-spacing:.01em;background:hsl(212 20% 10% / .7);backdrop-filter:blur(12px)}
.honest i{width:8px;height:8px;border-radius:4px;background:var(--acc)}
.seg{margin-left:auto;display:flex;padding:4px;gap:4px;pointer-events:auto}
.seg button,.dock button,.tog button,select{font:inherit;color:var(--ink);background:none;border:0;cursor:pointer}
.seg button{padding:9px 16px;border-radius:999px;font-size:14px;font-weight:600;color:var(--muted);min-height:40px;transition:background .2s,color .2s}
.seg button[aria-pressed=true]{background:hsl(0 0% 100% / .1);color:var(--ink)}
select{padding:9px 12px;border-radius:999px;font-size:14px;font-weight:600;background:hsl(0 0% 100% / .06);min-height:40px}
.panel{position:absolute;right:20px;top:92px;width:300px;padding:20px 22px;pointer-events:auto}
.panel h2{margin:0;font-size:20px;font-weight:700;letter-spacing:-.01em}
.panel .sub{margin:4px 0 12px;font-size:13px;color:var(--muted);line-height:1.4}
.stat{display:flex;justify-content:space-between;align-items:baseline;padding:9px 0;border-top:1px solid var(--line)}
.stat span{font-size:14px;color:var(--muted)}
.stat b{font-family:'JetBrains Mono',ui-monospace,monospace!important;font-size:19px;font-weight:600;font-variant-numeric:tabular-nums}
.stat b.acc{color:var(--acc)}
.stat b.bad{color:var(--bad)}
.note{margin-top:10px;font-size:12.5px;color:var(--muted);line-height:1.45}
.note b{color:var(--bad);font-weight:600}
.tog{display:flex;gap:6px;margin-top:12px}
.tog button{flex:1;padding:9px 0;border-radius:12px;font-size:13px;font-weight:600;background:hsl(0 0% 100% / .06);min-height:40px;transition:background .2s}
.tog button[aria-pressed=true]{background:var(--acc);color:#1b1206}
.dock{position:absolute;left:50%;bottom:20px;transform:translateX(-50%);display:flex;gap:6px;padding:6px;max-width:calc(100vw - 40px);overflow-x:auto;scrollbar-width:none}
.dock button{display:flex;flex-direction:column;align-items:center;gap:4px;padding:9px 14px;border-radius:16px;min-width:72px;min-height:56px;font-size:12.5px;font-weight:600;color:var(--muted);transition:background .25s,color .25s,transform .4s var(--spring)}
.dock button svg{width:22px;height:22px;fill:none;stroke:currentColor;stroke-width:1.7;stroke-linecap:round;stroke-linejoin:round}
.dock button[aria-pressed=true]{background:hsl(36 100% 64% / .14);color:var(--acc)}
.dock button:active{transform:scale(.94)}
.gates{position:absolute;left:20px;bottom:20px;width:300px;padding:16px 18px;pointer-events:auto}
.gates summary{list-style:none;cursor:pointer;display:flex;align-items:center;gap:12px;justify-content:space-between;font-size:14px;font-weight:600;min-height:28px;white-space:nowrap}
.gsum{font-size:13px}
.gates summary::-webkit-details-marker{display:none}
.dots{display:flex;gap:5px;margin-top:10px;flex-wrap:wrap}
.dots span{width:15px;height:15px;border-radius:5px;background:hsl(212 12% 30%)}
.dots span.ok{background:var(--ok)}.dots span.no{background:var(--bad)}
.glist{margin-top:10px;max-height:38vh;overflow:auto}
.gl{display:flex;gap:10px;padding:7px 0;border-top:1px solid var(--line);font-size:12.5px;line-height:1.35}
.gl em{font-style:normal;font-family:'JetBrains Mono',monospace!important;font-weight:700;width:26px;color:var(--muted)}
.gl b{margin-left:auto;font-weight:700;font-family:'JetBrains Mono',monospace!important}
.gl b.ok{color:var(--ok)}.gl b.no{color:var(--bad)}
.cursor{position:absolute;width:26px;height:26px;margin:-13px 0 0 -13px;border-radius:13px;border:2px solid var(--acc);box-shadow:0 0 0 6px hsl(36 100% 64% / .18);pointer-events:none;opacity:0;transition:left .9s var(--out),top .9s var(--out),opacity .3s,transform .25s var(--spring);z-index:9}
.cursor.press{transform:scale(.7)}
.loading{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);font-size:14px;color:var(--muted);display:flex;gap:10px;align-items:center}
.loading i{width:14px;height:14px;border-radius:7px;border:2px solid var(--muted);border-top-color:transparent;animation:spin 1s linear infinite}
@keyframes spin{to{transform:rotate(1turn)}}
.hint{position:absolute;left:50%;bottom:96px;transform:translateX(-50%);font-size:13px;color:var(--muted);opacity:.9;transition:opacity .5s}
@media (max-width:700px){
  .top{left:14px;right:14px;top:12px}
  .brand h1{font-size:24px}.brand p{display:none}
  .honest{font-size:12px;padding:5px 10px;margin-top:8px}
  .seg button{padding:8px 11px;font-size:13px}
  .panel{left:14px;right:14px;width:auto;top:auto;bottom:96px;padding:14px 16px;max-height:34vh;overflow:auto}
  .panel .sub{margin-bottom:6px}
  .stat{padding:7px 0}.stat b{font-size:17px}
  .gates{left:14px;right:auto;top:96px;bottom:auto;width:auto;padding:8px 12px;border-radius:16px}
  .gates .dots{display:none}
  .dock{bottom:12px;left:14px;right:14px;transform:none;max-width:none;justify-content:flex-start}
  .dock{gap:2px;padding:4px}
  .dock button{min-width:0;flex:1;padding:8px 2px;font-size:11px}
  .hint{display:none}
}
@media (prefers-reduced-motion: reduce){*{transition:none!important;animation:none!important}}
`;

export function runLive() {
  const style = document.createElement('style');
  style.textContent = CSS;
  document.head.appendChild(style);
  const app = document.getElementById('app');
  const canvas = document.createElement('canvas');
  canvas.id = 'gl';
  app.appendChild(canvas);
  const ui = document.createElement('div');
  app.appendChild(ui);
  ui.innerHTML = `
    <div class="top">
      <div class="brand"><h1>Avis</h1><p>${N.name} · ${N.reference ?? ''}</p><div class="honest"><i></i>simulated flight · real parts</div></div>
      <div class="seg glass" role="group" aria-label="Source">
        <button data-src="live" aria-pressed="true">Live</button>
        <button data-src="log" aria-pressed="false">Flight log</button>
        <select aria-label="Flight log" hidden></select>
      </div>
    </div>
    <section class="panel glass" aria-live="polite"></section>
    <details class="gates glass"><summary><span>Gates</span><span class="gsum"></span></summary><div class="dots"></div><div class="glist"></div></details>
    <nav class="dock glass" aria-label="Subsystems">${SUBSYSTEMS.map((s) => `<button data-sub="${s}" aria-pressed="false"><svg viewBox="0 0 24 24">${ICON[s]}</svg>${LABEL[s]}</button>`).join('')}</nav>
    <div class="hint">Drag to look around · pick a part of the bird to open it</div>
    <div class="loading"><i></i>Loading the Bow Valley…</div>
    <div class="cursor"></div>`;
  const $ = (q) => ui.querySelector(q);
  const panel = $('.panel');

  // ---- gates, as they are -----------------------------------------------------------------------------------------
  const ORDER = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M7', 'M8', 'V1', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7'];
  const G = ORDER.map((g) => ({ g, r: DATA.gates.find((x) => x.gate === g) }));
  $('.gsum').innerHTML = `<span style="color:var(--muted);font-weight:500">${G.filter((x) => x.r?.passed).length} pass · ${G.filter((x) => x.r && !x.r.passed).length} fail · ${G.filter((x) => !x.r).length} not run</span>`;
  $('.dots').innerHTML = G.map((x) => `<span class="${!x.r ? '' : x.r.passed ? 'ok' : 'no'}" title="${x.g}"></span>`).join('');
  $('.glist').innerHTML = G.map((x) => `<div class="gl"><em>${x.g}</em><span>${x.r ? gateLine(x.r) : 'no result yet'}</span><b class="${!x.r ? '' : x.r.passed ? 'ok' : 'no'}">${!x.r ? '—' : x.r.passed ? 'pass' : 'fail'}</b></div>`).join('') +
    `<div class="note">Design v${DATA.design_version}. Gate files in out/verification, shown as written.</div>`;

  // ---- scene --------------------------------------------------------------------------------------------------
  const lowTier = /Mobi|Android/i.test(navigator.userAgent) || innerWidth < 700;
  const stage = createStage(canvas, { quality: lowTier ? 'low' : 'high' });
  const mats = makeMaterials({ textures: true });
  const bird = buildBird(model, mats);
  const holder = new THREE.Group();
  holder.add(bird.root);
  stage.scene.add(holder);
  const cam = stage.camera;
  let valley = null;
  const sunLight = new THREE.DirectionalLight('#ffd9b0', 3.0);
  sunLight.castShadow = false;
  const skyFill = new THREE.HemisphereLight('#bcd4f0', '#3a3426', 0.9);

  // ---- flight sources -----------------------------------------------------------------------------------------
  const D = model.design;
  let origin = new THREE.Vector3(0, 0, 0);
  const live = { sim: null, ctl: null, tick: null, t0: 0 };
  function startLive() {
    const f = setupFlight(D, { seed: 7 });
    live.sim = f.sim;
    live.ctl = f.ctl;
    live.tick = f.tick;
  }
  startLive();
  let source = 'live';
  let log = null;
  let logT = 0;
  const logs = [];

  function logSample(t) {
    const n = log.t.length;
    const tt = log.t[0] + (t % (log.t[n - 1] - log.t[0]));
    let k = Math.min(n - 2, Math.max(0, Math.floor((tt - log.t[0]) * log.hz)));
    while (k < n - 2 && log.t[k + 1] < tt) k++;
    while (k > 0 && log.t[k] > tt) k--;
    const f = Math.min(1, Math.max(0, (tt - log.t[k]) / (log.t[k + 1] - log.t[k] || 1)));
    const L = (a) => a[k] + f * (a[k + 1] - a[k]);
    const q0 = new THREE.Quaternion(...log.quat[k]), q1 = new THREE.Quaternion(...log.quat[k + 1]);
    return {
      t: tt, pos: log.pos[k].map((v, i) => v + f * (log.pos[k + 1][i] - v)), quat: q0.slerp(q1, f),
      phase: L(log.phase), amp: L(log.amp), twistL: L(log.twistL), twistR: L(log.twistR), tailL: L(log.tailL), tailR: L(log.tailR),
      airspeed: L(log.airspeed), power: L(log.power), battery: L(log.battery),
    };
  }

  // ---- interaction ----------------------------------------------------------------------------------------------
  let focus = null;
  const view = { explode: false, cut: false, slow: false };
  let az = 0, el = 0, zoom = 1;
  let idleAt = performance.now();
  let touring = false;
  const touch = () => {
    idleAt = performance.now();
    if (touring) stopTour();
  };
  let drag = null;
  canvas.addEventListener('pointerdown', (e) => {
    touch();
    drag = { x: e.clientX, y: e.clientY, az, el };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!drag) return;
    az = drag.az - (e.clientX - drag.x) * 0.006;
    el = Math.max(-1.2, Math.min(1.2, drag.el + (e.clientY - drag.y) * 0.004));
  });
  canvas.addEventListener('pointerup', () => (drag = null));
  canvas.addEventListener('wheel', (e) => {
    touch();
    zoom = Math.max(0.35, Math.min(2.5, zoom * Math.exp(e.deltaY * 0.001)));
  }, { passive: true });
  addEventListener('keydown', (e) => {
    touch();
    if (source === 'live' && live.ctl) {
      if (e.key === 'ArrowLeft') live.ctl.guidance.course -= 0.35;
      if (e.key === 'ArrowRight') live.ctl.guidance.course += 0.35;
      if (e.key === 'ArrowUp') live.ctl.guidance.altitude += 5;
      if (e.key === 'ArrowDown') live.ctl.guidance.altitude -= 5;
    }
    if (e.key === 'Escape') setFocus(null);
  });

  function setFocus(sub) {
    focus = focus === sub ? null : sub;
    for (const b of ui.querySelectorAll('.dock button')) b.setAttribute('aria-pressed', String(b.dataset.sub === focus));
    view.explode = false;
    view.cut = focus === 'drive' || focus === 'power' || focus === 'avionics';
    view.slow = focus === 'drive' || focus === 'twist';
    az = 0;
    el = 0;
    zoom = 1;
    if (navigator.vibrate) navigator.vibrate(10);
  }
  ui.querySelector('.dock').addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    touch();
    setFocus(b.dataset.sub);
  });
  panel.addEventListener('click', (e) => {
    const b = e.target.closest('button[data-t]');
    if (!b) return;
    touch();
    view[b.dataset.t] = !view[b.dataset.t];
  });
  const sel = $('select');
  ui.querySelector('.seg').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-src]');
    if (!b) return;
    touch();
    setSource(b.dataset.src);
  });
  sel.addEventListener('change', () => loadLog(sel.value));
  function setSource(s) {
    source = s;
    for (const b of ui.querySelectorAll('.seg button')) b.setAttribute('aria-pressed', String(b.dataset.src === s));
    sel.hidden = s !== 'log';
    if (s === 'log' && !log && logs.length) loadLog(sel.value || logs[0].file);
  }
  async function loadLog(file) {
    try {
      log = await (await fetch(file)).json();
      logT = 0;
      sel.value = file;
    } catch {
      log = null;
    }
  }
  fetch('data/flights.json').then((r) => r.json()).then((list) => {
    logs.push(...list);
    sel.innerHTML = list.map((l) => `<option value="${l.file}">${l.scenario} · ${fmt(l.seconds, 0)} s</option>`).join('');
    const cor = list.find((l) => /corridor/.test(l.scenario));
    if (cor) sel.value = cor.file;
  }).catch(() => {
    ui.querySelector('[data-src=log]').disabled = true;
  });

  loadValley(stage, 'env').then((v) => {
    $('.loading').remove();
    if (!v) return;
    valley = v;
    v.group.visible = false;
    stage.scene.add(v.group);
    const cor = v.manifest;
    // the live flight starts over the corridor's first stretch, 60 m above the terrain origin height
    origin = new THREE.Vector3(1150, -40, -780);
    sunLight.position.copy(v.sun).multiplyScalar(500);
    stage.scene.add(sunLight, sunLight.target, skyFill);
    void cor;
  });

  // ---- tour ------------------------------------------------------------------------------------------------------
  const cursor = $('.cursor');
  const TOUR = [[null, 7], ['wing', 6], ['drive', 8, 'explode'], ['twist', 6], ['power', 6], ['avionics', 5], ['structure', 6, 'explode'], [null, 6]];
  let tourStep = -1, tourNext = 0;
  function stopTour() {
    touring = false;
    cursor.style.opacity = 0;
  }
  function tourTick(now) {
    if (!touring) {
      if (now - idleAt > 25000) {
        touring = true;
        tourStep = -1;
        tourNext = now;
      } else return;
    }
    if (now < tourNext) return;
    tourStep = (tourStep + 1) % TOUR.length;
    const [sub, secs, extra] = TOUR[tourStep];
    const btn = sub ? ui.querySelector(`.dock button[data-sub="${sub}"]`) : ui.querySelector(`.dock button[data-sub="${focus}"]`);
    if (btn && !reduce) {
      const r = btn.getBoundingClientRect();
      cursor.style.opacity = 1;
      cursor.style.left = `${r.left + r.width / 2}px`;
      cursor.style.top = `${r.top + r.height / 2}px`;
      setTimeout(() => cursor.classList.add('press'), 950);
      setTimeout(() => {
        cursor.classList.remove('press');
        if (touring) {
          if (sub !== focus) setFocus(sub ?? focus);
          if (extra) view[extra] = true;
          idleAt = performance.now() - 26000;
        }
      }, 1150);
    } else {
      setFocus(sub ?? focus);
      if (extra) view[extra] = true;
    }
    tourNext = now + secs * 1000;
  }

  // ---- panel --------------------------------------------------------------------------------------------------
  const panelHtml = (s) => (PANELS[focus] ?? PANELS.overview)(s, { view, source, log, has: !!s.tel, tel: s.tel ?? {}, fit: DATA.fit.issues });

  // ---- loop -------------------------------------------------------------------------------------------------------
  const resize = () => stage.resize(innerWidth, innerHeight);
  addEventListener('resize', resize);
  resize();
  let last = performance.now();
  let panelAt = 0;
  const chase = { pos: new THREE.Vector3(), tgt: new THREE.Vector3(), init: false };
  let slowPhase = 0, lastPhase = null;

  function frame(now) {
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    tourTick(now);
    // advance the source
    let s;
    if (source === 'log' && log) {
      logT += dt;
      const L = logSample(logT);
      s = { ...L, pos: new THREE.Vector3(...L.pos), crankHz: null, tel: null };
      if (!/corridor/.test(log.scenario)) s.pos.add(origin);
    } else s = liveState(live, origin, D, dt);
    if (s.crankHz == null && lastPhase != null) s.crankHz = Math.max(0, (s.phase - lastPhase) / (2 * Math.PI) / Math.max(dt, 1e-3));
    lastPhase = s.phase;
    // in the studio at quarter speed, the drive runs its own clock from the same phase rate
    let phase = s.phase;
    if (focus && view.slow) {
      slowPhase += 0.25 * 2 * Math.PI * (s.crankHz || N.flapHz) * dt;
      phase = slowPhase;
    } else slowPhase = phase;
    const ex = {};
    if (focus && view.explode) for (const k of SUBSYSTEMS) ex[k] = k === focus ? 0.0 : 0.6;
    if (focus === 'structure' && view.explode) for (const k of SUBSYSTEMS) ex[k] = 0.8;
    const dp = bird.update({ phase, amp: s.amp, twistL: s.twistL, twistR: s.twistR, tailL: s.tailL, tailR: s.tailR, focus, accents: !!focus, explode: ex, shell: view.cut ? 0 : 1 });
    s.phi = dp.right.phi;
    s.mu = dp.right.mu;
    // place and frame
    const studio = !!focus || !valley;
    stage.setStudio(studio);
    if (valley) valley.group.visible = !studio;
    stage.scene.background = studio ? null : valley?.sky ?? null;
    if (studio) {
      holder.position.set(0, 0, 0);
      bird.root.quaternion.identity();
      studioCamera(cam, stage, FRAMING[focus ?? 'overview'], az, el, zoom);
    } else {
      holder.position.copy(s.pos);
      bird.root.quaternion.copy(s.quat);
      chaseCamera(cam, stage, chase, s, az, el, zoom, dt);
      sunLight.target.position.copy(s.pos);
      sunLight.position.copy(s.pos).add(valley.sun.clone().multiplyScalar(50));
    }
    cam.updateProjectionMatrix();
    stage.render();
    if (now - panelAt > 120) {
      panelAt = now;
      panel.innerHTML = panelHtml(s);
    }
    if (!window.READY) window.READY = true;
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
  window.AVIS = { setFocus, setSource, view, get focus() { return focus; } };
}

// ---- panels: one per subsystem, each reading the live state s (or the log), never a typed number --------------
const stat = (k, v, cls = '') => `<div class="stat"><span>${k}</span><b class="${cls}">${v}</b></div>`;
const toggles = (view) => `<div class="tog"><button data-t="explode" aria-pressed="${view.explode}">Exploded</button><button data-t="cut" aria-pressed="${view.cut}">Cutaway</button><button data-t="slow" aria-pressed="${view.slow}">¼ speed</button></div>`;
const loadOr = (c, v) => (c.has ? v : '—');
const fitNotes = (items) => items.map((i) => `<b>Fit:</b> ${i.text}.`).join('<br>');
const PANELS = {
  wing: (s, c) => `<h2>Wing</h2><p class="sub">Two carbon spars per wing, ${model.ribs.length} balsa ribs, film both faces. The outer panel moves ${fmt(1 + model.wing.segments[1].gain, 1)}× the shoulder.</p>${stat('stroke angle', `${fmt(s.phi * DEG, 1)}°`, 'acc')}${stat('amplitude', `±${fmt(s.amp * DEG, 1)}°`)}${stat('root bending', loadOr(c, `${fmt(c.tel.rootBend, 1)} N·m`))}${stat('load factor', loadOr(c, `${fmt(c.tel.loadFactor, 2)} g`))}${toggles(c.view)}`,
  drive: (s, c) => `<h2>Drive</h2><p class="sub">${N.stages.join(' × ')} teeth, ${fmt(N.ratio, 2)} : 1 to a crank-rocker. Gears turn at the true ratio.</p>${stat('motor', `${fmt(s.crankHz * 60 * N.ratio, 0)} rpm`)}${stat('crank', `${fmt(s.crankHz, 2)} Hz`, 'acc')}${stat('crank torque', loadOr(c, `${fmt(c.tel.crankTorque, 2)} N·m`))}${stat('coupler force', loadOr(c, `${fmt(c.tel.couplerForce, 0)} N`))}${stat('transmission angle', `${fmt(s.mu * DEG, 0)}°`)}${toggles(c.view)}<p class="note">${fitNotes(c.fit.filter((i) => /crank|stage|gear|motor/.test(i.part + i.text)).slice(0, 2))}</p>`,
  twist: (s, c) => `<h2>Twist</h2><p class="sub">A servo at the elbow turns the hand wing through a ${fmt(model.design.servos?.twist?.linkage_ratio ?? 1, 1)} : 1 horn linkage and a torque rod: nose down on the downstroke, up on the upstroke.</p>${stat('twist, right', `${fmt(s.twistR * DEG, 1)}°`, 'acc')}${stat('twist, left', `${fmt(s.twistL * DEG, 1)}°`)}${stat('servo torque', loadOr(c, `${fmt(c.tel.twistTorque, 3)} N·m`))}${stat('servo current', loadOr(c, `${fmt(c.tel.servo_I, 2)} A`))}${toggles(c.view)}`,
  power: (s, c) => `<h2>Power</h2><p class="sub">${N.batteryCells}S ${fmt(N.batteryMah, 0)} mAh, ${fmt(N.batteryWh, 1)} Wh. Its ESC's BEC runs the servos and avionics.</p>${stat('electrical power', `${fmt(s.power, 0)} W`, 'acc')}${stat('battery', `${fmt(s.battery, 0)}%`)}${stat('battery current', loadOr(c, `${fmt(c.tel.batt_I, 1)} A`))}${stat('motor current', loadOr(c, `${fmt(c.tel.motor_I, 1)} A`))}${toggles(c.view)}`,
  avionics: (s, c) => `<h2>Avionics</h2><p class="sub">Flight controller with IMU and baro, GPS, airspeed and receiver; two servos move the V-tail.</p>${stat('airspeed', `${fmt(s.airspeed, 1)} m/s`, 'acc')}${stat('roll', `${fmt(s.euler ? s.euler.roll * DEG : null, 0)}°`)}${stat('pitch', `${fmt(s.euler ? s.euler.pitch * DEG : null, 0)}°`)}${stat('tail L / R', `${fmt(s.tailL * DEG, 1)}° / ${fmt(s.tailR * DEG, 1)}°`)}${toggles(c.view)}`,
  structure: (s, c) => `<h2>Structure</h2><p class="sub">${fmt(N.mass_g, 0)} g: parts ${fmt(N.parts_g, 0)} g, structure ${fmt(N.structure_g, 0)} g, 15% margin ${fmt(N.margin_g, 0)} g.</p>${stat('span', `${fmt(N.span_m, 2)} m`)}${stat('wing area', `${fmt(N.area_m2, 3)} m²`)}${stat('wing loading', `${fmt(N.wingLoading, 1)} N/m²`)}${toggles(c.view)}<p class="note">${fitNotes(c.fit.slice(0, 4))}</p>`,
  overview: (s, c) => `<h2>${c.source === 'live' ? 'Flying now' : 'Flight log'}</h2><p class="sub">${c.source === 'live' ? 'The flight model, stepped in real time.' : `${c.log?.scenario ?? ''}, from out/flight, as recorded.`}</p>${stat('airspeed', `${fmt(s.airspeed, 1)} m/s`)}${stat('stroke angle', `${fmt(s.phi * DEG, 1)}°`, 'acc')}${stat('flap frequency', `${fmt(s.crankHz, 2)} Hz`)}${stat('electrical power', `${fmt(s.power, 0)} W`)}${stat('battery', `${fmt(s.battery, 0)}%`)}`,
};

/** One real-time step of the flight model (several ticks of its own dt) and the state the page shows. */
function liveState(live, origin, D, dt) {
  const steps = Math.min(60, Math.round(dt / live.sim.dt));
  for (let i = 0; i < steps; i++) live.tick();
  // gentle S-turns down the valley so the flight has something to do
  live.ctl.guidance.course = 0.45 * Math.sin(live.sim.t / 18);
  const snap = live.sim.snapshot();
  const tel = live.sim.tel;
  const e = eulerFromQuat(snap.quat);
  return {
    pos: new THREE.Vector3(...snap.pos).add(origin), quat: new THREE.Quaternion(...snap.quat),
    phase: snap.phase, amp: snap.amp, twistL: snap.twistL, twistR: snap.twistR, tailL: snap.tailL, tailR: snap.tailR,
    airspeed: tel.airspeed, power: tel.power_w, battery: 100 * (1 - snap.charge / (D.battery.capacity_Ah * 3600)), crankHz: snap.crank / (2 * Math.PI), tel,
    euler: { roll: e.phi, pitch: e.theta },
  };
}

/** Studio framing of a subsystem, offset by the visitor's drag and zoom. */
function studioCamera(cam, stage, f, az, el, zoom) {
  const t = new THREE.Vector3(...f.target);
  const d = f.dist * zoom * (innerWidth < 700 ? 1.45 : 1);
  const A = f.az + az, E = f.el + el;
  cam.fov = 30;
  cam.position.set(t.x + d * Math.cos(E) * Math.cos(A), t.y + d * Math.sin(E), t.z + d * Math.cos(E) * Math.sin(A));
  cam.lookAt(t);
  stage.aimLights(t, 1.3);
}

/** A chase camera behind and above the bird, eased so it glides over the wingbeat. */
function chaseCamera(cam, stage, chase, s, az, el, zoom, dt) {
  const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(s.quat);
  fwd.y = 0;
  fwd.normalize();
  const k = innerWidth < 700 ? 1.5 : 1;
  const back = fwd.clone().applyAxisAngle(new THREE.Vector3(0, 1, 0), az + 0.35).multiplyScalar(-2.5 * zoom * k);
  const want = s.pos.clone().add(back).add(new THREE.Vector3(0, 0.55 * zoom * k + el * 2, 0));
  if (!chase.init) {
    chase.pos.copy(want);
    chase.tgt.copy(s.pos);
    chase.init = true;
  }
  chase.pos.lerp(want, 1 - Math.exp(-dt * 2.5));
  chase.tgt.lerp(s.pos, 1 - Math.exp(-dt * 8));
  cam.fov = 40;
  cam.position.copy(chase.pos);
  cam.lookAt(chase.tgt);
  stage.aimLights(s.pos, 1.3);
}

const FRAMING = {
  overview: { target: [-0.2, 0.02, 0], az: 0.65, el: 0.3, dist: 3.4 },
  wing: { target: [-0.02, 0.06, 0.5], az: 0.55, el: 0.55, dist: 1.7 },
  drive: { target: [0.015, -0.025, 0], az: 0.75, el: -0.12, dist: 0.55 },
  twist: { target: [-0.02, 0.08, 0.62], az: 0.35, el: 0.18, dist: 1.05 },
  power: { target: [0.14, -0.01, 0], az: 0.6, el: -0.35, dist: 0.8 },
  avionics: { target: [0.0, 0.0, 0], az: 0.9, el: 0.45, dist: 0.85 },
  structure: { target: [-0.25, 0.0, 0], az: 0.8, el: 0.4, dist: 2.6 },
};
