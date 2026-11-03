// The film as one page: render(t) is a pure function of time. Studio sequences (assembly, drive, twist, power,
// cards) pose the bird from the same model as the page; flight shots are Blender plates (PNG frames rendered from
// the flight logs) shown behind a HUD that reads the same log. The edit (film/edit.json, resolved by
// film/render.mjs into site/data/edit.json) decides the order and timing; the score reads the same edit.
import * as THREE from 'three';
import { makeMaterials, ACCENT, livery as liveryOf, liveryInBom, defaultLivery } from './materials.js';
import { buildBird } from './bird.js';
import { createStage } from './stage.js';
import { numbers, gateLine, gateFailures, gateVerdict } from './numbers.js';
import { drivePose } from './model.js';

const DEG = 180 / Math.PI;
const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const ease = (x) => (x < 0.5 ? 4 * x * x * x : 1 - (-2 * x + 2) ** 3 / 2);
const easeOut = (x) => 1 - (1 - clamp(x)) ** 3;
const span = (u, a, b) => clamp((u - a) / (b - a));
/** The flight shots' readout from the log at plate frame j of segment data d (render.mjs writeSegData): airspeed,
 * flap-cycle mean power and climb, battery. Empty without data. */
export function flightHud(d, j) {
  if (!d || j < 0) return '';
  const climb = d.climb_m_s ? `<div>climb<b>${signed(d.climb_m_s[j], 1)} m/s</b></div>` : '';
  return `<div class="fhud"><div>airspeed<b>${fmt(d.airspeed[j], 1)} m/s</b></div><div>electrical power, flap-cycle mean<b>${fmt(d.power_w[j], 0)} W</b></div>${climb}<div>battery<b>${fmt(d.battery_pct[j], 0)}%</b></div></div>`;
}

/** A caption's design numbers, written in edit.json as {altitude_m} and {temperature_C}, filled from the design. */
export function fillDesign(text, design) {
  const atm = design?.atmosphere ?? {};
  return text.replace(/\{altitude_m\}/g, fmt(atm.altitude_m, 0)).replace(/\{temperature_C\}/g, fmt(atm.temperature_C, 0));
}

/**
 * The drive sequence's words that depend on the design: the crank wheel's teeth, and the part the second callout
 * points at, the stroke servo's sliding pin, or on a fixed-stroke drive the crank angle encoder its governor reads.
 */
export function driveLabels(model) {
  const { ampMin, ampMax } = model.law;
  const fixed = !(ampMax - ampMin > 1e-6);
  return {
    crankWheel: `Crank wheel ${model.train.stages[model.train.stages.length - 1].wheel.z}T`,
    // where each label's end sits on screen (a label left of its part ends there): clear ground left of the gears (the
    // encoder, low on the crank shaft) or above them (the pin swings through the gears), off the Coupler and Motor labels
    fixed,
    late: fixed ? { node: 'encoder_readhead', text: 'Crank angle encoder', at: [450, 640] } : { node: 'slot_pin_R', text: 'Stroke servo slides the pin', at: [700, 195] },
  };
}

/** Where the power sequence's labels sit on screen, one per powerParts row: clear ground left of the parts. */
export const POWER_LABELS = [[330, 330], [330, 430], [330, 530], [330, 630], [330, 730]];
// The power sequence's camera: an orbit round the power parts' sphere, held at `fill` of the frame height, shifted
// so every part stays right of the label column (POWER_LABELS' x) and each line runs away from its label.
export const POWER_CAM = { az0: 30, azSpan: 25, el0: -22, elSpan: 8, distSpan: -0.08, fov: 30, fill: 0.72, shift: 160 };

/** Place `cam` on an orbit round `target` (degrees, metres), looking at it. */
export function orbitCamera(cam, target, azDeg, elDeg, dist, fov = 30) {
  const az = azDeg / DEG, el = elDeg / DEG;
  cam.fov = fov;
  cam.position.set(target.x + dist * Math.cos(el) * Math.cos(az), target.y + dist * Math.sin(el), target.z + dist * Math.cos(el) * Math.sin(az));
  cam.lookAt(target);
  cam.updateProjectionMatrix();
}

/** Segments [p, q] and [r, s] (screen points) cross each other, end points excluded. */
export function segmentsCross([p, q], [r, s]) {
  const o = (a, b, c) => Math.sign((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]));
  return o(p, q, r) * o(p, q, s) < 0 && o(r, s, p) * o(r, s, q) < 0;
}

/** Every ordering of the items of `a`. */
function permutations(a) {
  if (a.length <= 1) return [a];
  return a.flatMap((x, i) => permutations([...a.slice(0, i), ...a.slice(i + 1)]).map((r) => [x, ...r]));
}

/** The slot for each callout: tracks[k][j] is part k's screen point at sample j (null off screen), slots the label
 * points. Of every assignment of parts to slots, the one whose lines (part to slot) cross least over all samples;
 * ties go to the fewest parts moved from the listed order. Held for the whole sequence, so no label jumps. */
export function assignSlots(tracks, slots) {
  const lines = (perm, j) => tracks.map((tr, k) => (tr[j] ? [tr[j], slots[perm[k]]] : null)).filter(Boolean);
  const nS = Math.max(0, ...tracks.map((t) => t.length));
  let best = null;
  for (const perm of permutations([...slots.keys()]).map((p) => p.slice(0, tracks.length))) {
    let cross = 0;
    for (let j = 0; j < nS; j++) {
      const L = lines(perm, j);
      for (let a = 0; a < L.length; a++) for (let b = a + 1; b < L.length; b++) cross += segmentsCross(L[a], L[b]);
    }
    const moved = perm.reduce((m, v, k) => m + (v !== k), 0);
    if (!best || cross < best.cross || (cross === best.cross && moved < best.moved)) best = { perm, cross, moved };
  }
  return best;
}

/** The title camera's orbit at u (0..1 through the sequence): [target, azimuth deg, elevation deg, distance, fov]. */
export function titleOrbit(u) {
  const c = TITLE_CAM;
  return [new THREE.Vector3(...c.target), c.az0 + c.azSpan * u, c.el0 + c.elSpan * u, c.dist0 + c.distSpan * ease(u), c.fov];
}

/** The power camera's orbit at u round the power parts' bounding sphere, as titleOrbit. */
export function powerOrbit(sphere, u) {
  const c = POWER_CAM;
  const fr = framing(sphere.c, sphere.r, c.fov, c.fill);
  return [fr.target, c.az0 + c.azSpan * ease(u), c.el0 + c.elSpan * u, fr.dist * (1 + c.distSpan * ease(u)), c.fov];
}

/** Screen tracks of the power parts' callout points over the power sequence: the camera SEQ.power flies, the bird
 * flapping as it does, `samples` times over `dur` seconds. */
export function powerTracks({ bird, cam, sphere, parts, flap, dur, samples = 48 }) {
  const c = POWER_CAM;
  const tracks = parts.map(() => []);
  for (let j = 0; j < samples; j++) {
    const t = (j / (samples - 1)) * dur, u = t / dur;
    orbitCamera(cam, ...powerOrbit(sphere, u));
    cam.setViewOffset(1920, 1080, c.shift, 0, 1920, 1080);
    cam.updateMatrixWorld(true);
    bird.update({ ...flap(t), focus: 'power' });
    bird.root.updateMatrixWorld(true);
    parts.forEach((p, k) => {
      const o = bird.root.getObjectByName(p.node);
      if (!o) return tracks[k].push(null);
      const v = o.getWorldPosition(new THREE.Vector3()).project(cam);
      const x = (v.x * 0.5 + 0.5) * 1920, y = (-v.y * 0.5 + 0.5) * 1080;
      tracks[k].push(onScreen(x, y, v.z) ? [x, y] : null);
    });
  }
  return tracks;
}

/** The twist sequence's orbit: aim point, azimuth sweep (deg), elevation (deg), distance (m), lens fov (deg). */
// The title's camera and the title block's top edge: the bird flaps in the upper right while the block sits lower
// left, so no wing crosses "Avis", the subtitle or the rule over the slow push-in (film/test/title-camera.test.mjs).
export const TITLE_CAM = { target: [-0.22, -0.2, 0], az0: 32, azSpan: -10, el0: 16, elSpan: -4, dist0: 4.1, distSpan: -0.3, fov: 30, shift: -300, top: 640 };
export const TWIST_CAM = { target: [-0.06, 0.06, 0], az0: 10, azSpan: 24, el: 12, dist0: 3.6, distSpan: -0.1, fov: 33, shift: 260 };

const fmt = (v, d = 0) => (v == null || Number.isNaN(v) ? '—' : v.toLocaleString('en-US', { minimumFractionDigits: d, maximumFractionDigits: d }));
/** Signed with a true minus, so a climb and a sink line up in tabular figures. */
export const signed = (v, d = 0) => {
  if (v == null || Number.isNaN(v)) return '—';
  const r = Math.round(v * 10 ** d) / 10 ** d; // no "−0.0"
  return `${r < 0 ? '\u2212' : '+'}${fmt(Math.abs(r), d)}`;
};

const CSS = `
#film{position:absolute;left:0;top:0;width:1920px;height:1080px;overflow:hidden;background:#0b0e12;font-family:Inter,system-ui,sans-serif;color:#e9eef3;text-align:left}
.frow b,.fhud b,.g .v,.panel h3{font-variant-numeric:tabular-nums}
#film *{text-align:left}
#film canvas,#film img.plate{position:absolute;inset:0;width:1920px;height:1080px}
#film img.plate{object-fit:cover}
#film .ov{position:absolute;inset:0;pointer-events:none}
#film .fade{position:absolute;inset:0;background:#07090c;opacity:0}
.honest{position:absolute;left:64px;bottom:52px;font-size:30px;letter-spacing:.02em;color:#c9d3dc;opacity:.92;font-weight:500;text-shadow:0 1px 12px #0008}
.honest b{color:${ACCENT};font-weight:600}
.mark{position:absolute;border:2px solid ${ACCENT};border-radius:50%;box-shadow:0 0 0 1px rgba(0,0,0,.35),0 0 10px rgba(0,0,0,.4)}
.markl{position:absolute;font-size:26px;font-weight:600;white-space:nowrap;color:#f3f6f9;text-shadow:0 1px 10px #000c}
.livtag{position:absolute;right:64px;bottom:52px;font-size:28px;font-weight:600;color:#0b0e12;background:${ACCENT};padding:6px 16px;border-radius:8px}
.titleblock{position:absolute;left:120px;top:${TITLE_CAM.top}px}
.titleblock h1{font-size:200px;line-height:.9;margin:0;font-weight:800;letter-spacing:-.04em}
.titleblock h2{font-size:46px;margin:28px 0 0;font-weight:500;color:#c9d3dc;letter-spacing:-.01em}
.titleblock .rule{width:96px;height:4px;background:${ACCENT};border-radius:2px;margin-top:36px}
.panel{position:absolute;right:72px;top:96px;width:560px;padding:34px 38px;border-radius:24px;background:rgba(14,18,24,.97);backdrop-filter:blur(20px) saturate(1.4);box-shadow:0 30px 60px #0007,inset 0 1px 0 rgba(255,255,255,.06)}
.panel h3{margin:0 0 6px;font-size:40px;font-weight:700;letter-spacing:-.02em}
.panel .sub{font-size:28px;color:#a9b5c1;margin-bottom:22px;line-height:1.3}
.frow{display:flex;justify-content:space-between;align-items:baseline;padding:12px 0;border-top:1px solid rgba(255,255,255,.07)}
.frow span{font-size:30px;color:#b9c4cf}
.frow b{font-family:'JetBrains Mono',monospace;font-size:38px;font-weight:600;color:#f3f6f9}
.frow b.acc{color:${ACCENT}}
.frow b.bad{color:#ff8a75}
.fcap{position:absolute;left:64px;top:56px;font-size:34px;font-weight:600;letter-spacing:-.01em;text-shadow:0 2px 14px #000a}
.fcap small{display:block;font-size:30px;font-weight:500;color:#b9c4cf;margin-top:6px}
.call{position:absolute;font-size:34px;font-weight:600;white-space:nowrap;text-shadow:0 2px 12px #000c}
.call i{position:absolute;width:10px;height:10px;border-radius:5px;background:${ACCENT};box-shadow:0 0 0 4px rgba(255,181,71,.25)}
.fhud{position:absolute;left:64px;bottom:120px;display:flex;gap:44px}
.fhud div{font-size:28px;color:#c9d3dc}
.fhud b{display:block;font-family:'JetBrains Mono',monospace;font-size:44px;color:#fff;font-weight:600}
.card{position:absolute;left:120px;right:120px;top:120px;bottom:150px}
.card h2{font-size:64px;margin:0 0 10px;font-weight:800;letter-spacing:-.03em}
.card p{font-size:32px;color:#b9c4cf;margin:0 0 30px}
.gates{display:grid;grid-template-columns:1fr 1fr;column-gap:64px}
.g{display:flex;gap:20px;align-items:baseline;padding:11px 0;border-top:1px solid rgba(255,255,255,.07);font-size:30px;color:#d5dde5}
.g em{font-style:normal;font-family:'JetBrains Mono',monospace;font-weight:700;width:64px;color:#8d99a6}
.g .ok{color:#cfe8d7}.g .no{color:#ff8a75}
.g .v{margin-left:auto;font-family:'JetBrains Mono',monospace;font-size:28px;font-weight:600}
.g .v.nf{color:#f0b35a;white-space:nowrap}
.g.wide{grid-column:1/-1}
.g .why{display:block;font-size:26px;color:#ffb3a6;margin-top:4px}
.frow.dim{opacity:.38}
.frow b{white-space:nowrap;margin-left:16px}
.frow.s{padding:7px 0}.frow.s span{font-size:25px}.frow.s b{font-size:28px}
.g{font-size:24px;padding:5px 0}.g .why{font-size:21px;margin-top:2px}
.card.gc h2{font-size:48px}.card.gc p{font-size:26px;margin-bottom:10px}
.credits{columns:2;column-gap:80px;font-size:30px;line-height:1.45;color:#c9d3dc}
.credits b{color:#fff;font-weight:600}
`;

/** The subsystem a BOM role or structure item belongs to, by its words. */
export function subsystemOf(s) {
  if (/motor|gear|bearing|bushing|stroke|crank|coupler|rocker|horn pin|shoulder (pin|housing)|^key,|encoder/i.test(s)) return 'drive';
  if (/battery|ESC|regulator/i.test(s)) return 'power';
  if (/flight controller|GPS|airspeed|receiver|tail servo/i.test(s)) return 'avionics';
  if (/twist servo|twist torque|spar|edge rod|wing film|ribs/i.test(s)) return 'wing';
  return 'structure';
}

/** Mass per subsystem from the BOM, by the role or item name. */
export function subsystemMass(design) {
  const sub = subsystemOf;
  const m = { drive: 0, power: 0, avionics: 0, wing: 0, structure: 0 };
  for (const p of design.bom.parts) m[sub(p.role)] += p.mass_g;
  for (const s of design.bom.structure) m[sub(s.name)] += s.mass_g;
  return m;
}

/** A catalogue part's short name: its maker and the first word of its name ("Pololu D36V50F7"). */
export function shortName(part) {
  if (!part) return '';
  const maker = part.manufacturer ?? '';
  const rest = (part.name ?? '').startsWith(maker) ? part.name.slice(maker.length).trim() : part.name ?? '';
  return `${maker} ${rest.split(/[\s,(]/)[0]}`.trim();
}

/** The gear train as the BOM lists it, motor first: each stage's catalogue numbers, teeth, ratio and BOM masses. */
export function driveStages(model) {
  const bom = (id) => model.design.bom.parts.find((p) => p.id === id);
  // the part number: the catalogue's, else the first word of the BOM line's name (the page's catalogue copy is terse)
  const pn = (id) => model.parts[id]?.specs?.part_number ?? (bom(id)?.name ?? model.parts[id]?.name)?.split(' ')[0] ?? id;
  return model.train.stages.map((st) => ({
    pinion: pn(st.pinion.id), wheel: pn(st.wheel.id), zp: st.pinion.z, zw: st.wheel.z, ratio: st.wheel.z / st.pinion.z,
    mass_g: (bom(st.pinion.id)?.unit_mass_g ?? 0) + (bom(st.wheel.id)?.unit_mass_g ?? 0),
  }));
}

/** The power sequence's parts, from the BOM by role: battery, ESC, servo regulator and the servos it feeds, with masses. */
export function powerParts(model) {
  const D = model.design;
  const rows = [];
  const add = (label, re, node) => {
    const ps = D.bom.parts.filter((p) => re.test(p.role));
    if (!ps.length) return;
    const n = ps.reduce((a, p) => a + p.count, 0);
    const part = { ...model.parts[ps[0].id], name: ps[0].name ?? model.parts[ps[0].id]?.name };
    rows.push({ label, name: `${n > 1 ? `${n} × ` : ''}${shortName(part)}`, mass_g: ps.reduce((a, p) => a + p.mass_g, 0), node });
  };
  add(`${D.battery.cells}S LiPo`, /^battery$/, 'part_battery');
  add('ESC', /\bESC\b/, 'part_esc');
  add('regulator', /regulator/, 'part_regulator');
  add('tail servos', /^tail servo$/, 'part_servo_tail_R');
  add('twist servos', /twist servo$/, 'twist_servo_R');
  return rows;
}

/** The bounding sphere (centre c, radius r) of the bird's meshes that pass `pred`, posed level at amplitude amp. */
export function sphereOf(bird, amp, pred) {
  const b = new THREE.Box3();
  bird.update({ phase: 0, amp });
  bird.root.updateMatrixWorld(true);
  bird.root.traverse((o) => o.isMesh && pred(o) && b.expandByObject(o));
  return { c: b.getCenter(new THREE.Vector3()), r: b.getSize(new THREE.Vector3()).length() / 2 };
}

/** A projected point (pixels, NDC depth) inside the 1920 x 1080 frame and in front of the camera. */
export const onScreen = (x, y, z) => z <= 1 && x >= 0 && x <= 1920 && y >= 0 && y <= 1080;

/** Aim point and distance that hold a bounding sphere (centre c, radius r) at fraction k of the frame height. */
export function framing(c, r, fovDeg = 30, k = 0.62) {
  return { target: c, dist: r / (k * Math.tan((fovDeg * Math.PI) / 360)) };
}

/** A ring round a bird too small to read in a wide shot, at its place on screen [x, y, distance m] from the log and
 * the shot's camera (film/bird_px.py), labelled with what it is and how far away. Nothing when it is off screen. */
export function birdMarker(px, span_m, name) {
  if (!px || !onScreen(px[0], px[1], 0) || !(px[2] > 0)) return '';
  const [x, y, d] = px;
  const R = 26;
  const left = x > 1920 - 560; // label on the side with room
  const far = d >= 1000 ? `${fmt(d / 1000, 1)} km` : `${fmt(d, 0)} m`;
  return `<div class="mark" style="left:${x - R}px;top:${y - R}px;width:${2 * R}px;height:${2 * R}px"></div>
    <div class="markl" style="left:${left ? x - R - 14 : x + R + 14}px;top:${y - R - 34}px;${left ? 'transform:translateX(-100%)' : ''}">${name} · ${fmt(span_m, 2)} m span, ${far} away</div>`;
}

/** A marker segment's ring at plate frame j of its data d (render.mjs birdOnScreen); nothing for other segments. */
export function segMarker(seg, d, j, N) {
  return seg.marker ? birdMarker(d?.px?.[j], N.span_m, N.name) : '';
}

/** The edit's livery (null: the clear film; absent: the white the BOM records, else the clear film) and, when the
 * BOM does not record its colours, the label every frame carries so a preview never passes for the bird as built. */
export function filmLivery(edit, parts, bom = null) {
  const liv = liveryOf(edit.livery === undefined ? defaultLivery(parts, bom) : edit.livery);
  return { liv, tag: liv && !liveryInBom(liv, parts, bom) ? `<div class="livtag">preview, livery not yet in BOM · ${liv.name}</div>` : '' };
}

/** A studio sequence's duration in the edit (the first segment that plays it), or `d` when the edit has none. */
export function seqDur(edit, seq, d = 5.5) {
  return edit.segments.find((s) => s.seq === seq)?.dur ?? d;
}

/** One panel row: a label and its value. */
const row = (k, v, cls = '', rowCls = '') => `<div class="frow ${rowCls}"><span>${k}</span><b class="${cls}">${v}</b></div>`;

/** The gates card's rows: every gate, its own line and verdict; a failure spans the card with what failed beneath. */
export function gatesCardItems(gates) {
  return gateOrder(gates).map((g) => {
    const r = gates.find((x) => x.gate === g);
    if (!r) return `<div class="g"><em>${g}</em><span>no result yet</span><span class="v" style="color:#8d99a6">not run</span></div>`;
    const v = gateVerdict(r);
    if (v === 'pass' || v === 'pass?') return `<div class="g"><em>${g}</em><span>${gateLine(r)}</span><span class="v ok">${v}</span></div>`;
    // not final: both runs' own numbers, and why it is not a pass
    if (v === 'NOT FINAL') return `<div class="g wide"><em>${g}</em><span>${gateLine(r)} on the CFD suction table; ${gateLine(r.at06).replace(/^[^:]*:\s*/, '')} at the published η<sub>s</sub> 0.6<span class="why">not final, so not a pass: the verdict differs at η<sub>s</sub> 0.6${(r.passed ? r.at06.failures ?? gateFailures(r.at06) : r.failures ?? gateFailures(r)).length ? ` (${(r.passed ? r.at06.failures ?? gateFailures(r.at06) : r.failures ?? gateFailures(r)).join('; ')})` : ''}</span></span><span class="v nf">NOT FINAL</span></div>`;
    return `<div class="g wide"><em>${g}</em><span>${gateLine(r)}<span class="why">${(r.failures ?? gateFailures(r)).join('; ')}</span></span><span class="v no">FAIL</span></div>`;
  }).join('');
}

/** The drive panel's stage rows, motor first: catalogue numbers, teeth and ratio. */
export const stageRows = (stages) => stages.map((x) => row(`${x.pinion} → ${x.wheel}`, `${x.zp}:${x.zw} · ${fmt(x.ratio, 2)}`, '', 's')).join('');

/**
 * The assembly at u: build progress (bird.js steps, one after another, evenly over u 0..0.86) and each subsystem's
 * exploded fraction for the panel's rows; a bird without steps (no physics layout) explodes by subsystem windows.
 */
export function assemblyExplode(bird, u) {
  const build = span(u, 0.0, 0.86);
  const bx = bird.buildExplode(build);
  if (bx) return { build, ex: Object.fromEntries(['structure', 'drive', 'power', 'avionics', 'wing'].map((k) => [k, bx[k] ?? 0])) };
  const w = { structure: [0.0, 0.42], drive: [0.12, 0.55], power: [0.22, 0.62], avionics: [0.3, 0.7], wing: [0.4, 0.86] };
  return { build: null, ex: Object.fromEntries(Object.entries(w).map(([k, [a, b]]) => [k, 1 - ease(span(u, a, b))])) };
}

/** The assembly camera at u: [target, azimuth deg, elevation deg, distance m, fov deg]; on the gearbox (between
 * the plates, about the crank axis) while it and its frame go together, out to the whole bird from u 0.68 to 0.84. */
export function assemblyOrbit(u) {
  // from the right side, in the key light, so the slides along x read across the frame; then round to the old path
  const out = ease(span(u, 0.68, 0.84));
  return [new THREE.Vector3(0.045 - 0.195 * out, -0.025 * (1 - out), 0), 75 + (75 + 45 * ease(u)) * out, 24 - 6 * u, 0.6 + 1.85 * out - 0.25 * u, 30];
}

/** The assembly panel's subsystem rows: each subsystem's BOM mass, dimmed until it is in place. */
export const massRows = (ex, massBy) => Object.keys(ex).map((k) => `<div class="frow${ex[k] < 0.02 ? '' : ' dim'}"><span>${k}</span><b>${fmt(massBy[k], 0)} g</b></div>`).join('');

/** The SPEC's gates, then any other gate file present, M before V, by number. */
export function gateOrder(gates) {
  const declared = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M7', 'M8', 'V1', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7'];
  const key = (g) => [g[0] === 'M' ? 0 : 1, +g.slice(1) || 0];
  return [...new Set([...declared, ...gates.map((g) => g.gate)])].sort((a, b) => key(a)[0] - key(b)[0] || key(a)[1] - key(b)[1] || a.localeCompare(b));
}

export function createFilm(host, { model, data, edit, plateBase = 'plates', segData = {} }) {
  const N = numbers(model, data);
  const style = document.createElement('style');
  style.textContent = CSS;
  document.head.appendChild(style);
  const film = document.createElement('div');
  film.id = 'film';
  host.appendChild(film);
  const plate = document.createElement('img');
  plate.className = 'plate';
  plate.style.opacity = 0;
  const canvas = document.createElement('canvas');
  const ov = document.createElement('div');
  ov.className = 'ov';
  const fade = document.createElement('div');
  fade.className = 'fade';
  film.append(plate, canvas, ov, fade);

  const stage = createStage(canvas, { capture: true, pr: 1 });
  stage.resize(1920, 1080);
  // the covering's livery: none (the clear film) unless the edit names one; one the catalogue does not record yet is
  // labelled on every frame, so a preview never passes for the bird as built
  const { liv, tag: liveryTag } = filmLivery(edit, model.parts, model.design.bom);
  const mats = makeMaterials({ textures: true, livery: liv });
  const bird = buildBird(model, mats, { livery: liv });
  stage.scene.add(bird.root);
  const massBy = subsystemMass(model.design);
  const stages = driveStages(model), power = powerParts(model);
  // each studio shot is framed on the parts it shows, as built from this design, so a new design re-frames itself
  const servoIds = new Set(model.design.bom.parts.filter((p) => /servo$/.test(p.role)).map((p) => p.id));
  const driveSphere = sphereOf(bird, N.ampDeg / DEG, (o) => o.userData.subsystem === 'drive' && !/^(hinge|shoulder|rocker_socket)/.test(o.name));
  const powerSphere = sphereOf(bird, N.ampDeg / DEG, (o) => o.userData.subsystem === 'power' && !/^wire/.test(o.name));
  const cam = stage.camera;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);

  const orbit = (target, azDeg, elDeg, dist, fov = 30) => {
    orbitCamera(cam, target, azDeg, elDeg, dist, fov);
    stage.aimLights(target, 1.3);
  };
  const flap = (t, speed = 1) => {
    const ph = 2 * Math.PI * N.flapHz * t * speed;
    return { phase: ph, amp: N.ampDeg / DEG, twistL: N.twistAt(ph, -1), twistR: N.twistAt(ph, 1) };
  };
  // the power callouts' label slots, held for the sequence: the assignment whose lines never (or least) cross
  const powerSlots = assignSlots(powerTracks({ bird, cam: cam.clone(), sphere: powerSphere, parts: power, flap, dur: seqDur(edit, 'power') }), POWER_LABELS).perm;
  const screenOf = (obj, local = V(0, 0, 0)) => {
    const p = local.clone();
    obj.updateMatrixWorld(true);
    p.applyMatrix4(obj.matrixWorld).project(cam);
    return [(p.x * 0.5 + 0.5) * 1920, (-p.y * 0.5 + 0.5) * 1080, p.z];
  };
  const callout = (text, obj, local, dx, dy, alpha = 1) => {
    if (!obj || alpha <= 0.01) return '';
    const [x, y] = screenOf(obj, local);
    const lx = x + dx, ly = y + dy;
    const len = Math.hypot(dx, dy), ang = Math.atan2(dy, dx) * DEG;
    return `<div style="position:absolute;left:${x}px;top:${y}px;width:${len}px;height:2px;background:rgba(255,181,71,.8);transform-origin:0 50%;transform:rotate(${ang}deg);opacity:${alpha}"></div>
      <div class="call" style="left:${lx + (dx < 0 ? -12 : 12)}px;top:${ly - 22}px;opacity:${alpha};${dx < 0 ? 'transform:translateX(-100%)' : ''}">${text}</div>
      <div class="call" style="left:${x - 5}px;top:${y - 5}px;opacity:${alpha}"><i style="left:0;top:0"></i></div>`;
  };
  /** A callout whose label sits at a fixed clear place on screen [x, y], its line running to the part. */
  const calloutAt = (text, obj, at, alpha = 1) => {
    if (!obj || !at) return '';
    const [x, y, z] = screenOf(obj, V(0, 0, 0));
    if (!onScreen(x, y, z)) return ''; // the part is out of shot: no line to nowhere
    return callout(text, obj, V(0, 0, 0), at[0] - x, at[1] - y, alpha);
  };
  const honest = (a = 1) => `<div class="honest" style="opacity:${0.92 * a}"><b>●</b>&nbsp; simulated flight · real parts</div>${liveryTag}`;
  const node = (n) => bird.root.getObjectByName(n);

  // ---- studio sequences: (u in 0..1, t seconds into the sequence, dur) -> { state, html } with the camera set ---
  const SEQ = {
    title(u, t) {
      const c = TITLE_CAM;
      orbit(...titleOrbit(u));
      const a = easeOut(span(u, 0.05, 0.3));
      return {
        shift: c.shift,
        state: flap(t),
        html: () => `<div class="titleblock" style="opacity:${a};transform:translateY(${(1 - a) * 24}px)"><h1>Avis</h1><h2>${N.name} · a gull built from catalogue parts</h2><div class="rule"></div></div>${honest(a)}`,
      };
    },
    assembly(u, t) {
      // the parts go together one step after another in an order a person could follow with them (bird.js steps:
      // the physics' layout through solids.js assemblyPlan, then the packed parts, the pod and the wings); each
      // subsystem's row lights when its last step is in. A design without the physics' solids explodes by subsystem.
      const { build, ex } = assemblyExplode(bird, u);
      // close on the gearbox while it goes together (the steps up to the trays), then out to the whole bird for the
      // packed parts, the pod and the wings
      orbit(...assemblyOrbit(u));
      const fl = span(u, 0.88, 1);
      const st = fl > 0 ? flap(t - 0.88 * this._dur) : { phase: this._level, amp: N.ampDeg / DEG };
      let g = 0;
      for (const k of Object.keys(ex)) g += massBy[k] * (1 - ex[k]);
      g += N.margin_g * span(u, 0.86, 0.95);
      return {
        shift: 260,
        state: { ...st, explode: ex, build, shell: 1 },
        html: () => `<div class="fcap">Every part from a catalogue<small>${N.bomLines} BOM lines, ${N.partCount} bought parts, each with a source</small></div>
          <div class="panel" style="top:auto;bottom:120px"><h3>${fmt(g, 0)} g</h3><div class="sub">parts ${fmt(N.parts_g, 0)} g · structure ${fmt(N.structure_g, 0)} g · 15% margin ${fmt(N.margin_g, 0)} g</div>
          ${massRows(ex, massBy)}${row('span', `${fmt(N.span_m, 2)} m`)}</div>${honest()}`,
      };
    },
    drive(u, t) {
      const fr = framing(driveSphere.c, driveSphere.r, 30, 0.85);
      orbit(fr.target, 205 - 30 * ease(u), -6 + 8 * u, fr.dist * (1 - 0.1 * ease(u)), 30);
      const speed = 0.25;
      const st = flap(t, speed);
      // a stroke servo walks the amplitude through its range in the second half; a fixed-stroke drive holds it
      const s = span(u, 0.5, 0.95);
      const { ampMin, ampMax } = model.law;
      const labels = driveLabels(model);
      const nom = st.amp;
      let amp = nom;
      if (s > 0) amp = s < 1 / 3 ? nom + (ampMax - nom) * ease(s * 3) : s < 2 / 3 ? ampMax + (ampMin - ampMax) * ease(s * 3 - 1) : ampMin + (nom - ampMin) * ease(s * 3 - 2);
      const dp = drivePose(model, st.phase, amp);
      const a = easeOut(span(u, 0.04, 0.2));
      const pinNode = node(labels.late.node);
      const pinAlpha = easeOut(span(u, 0.5, 0.6));
      // the pin callout's label is pinned to a fixed clear patch of background (between the caption and the gear
      // train, left of the panel) rather than a fixed offset from the pin: the pin itself swings through the gear
      // cluster over the walk, so a constant dx/dy would drag the label back and forth across the gears with it.
      // Computed inside html() (not here) so the camera's matrixWorld is current for this frame (set after the
      // caller's bird.update()/cam.updateMatrixWorld(), not before, same as every other callout's lazy screenOf()).
      const pinCallout = () => {
        if (!pinNode || pinAlpha <= 0.01) return '';
        const [px, py] = screenOf(pinNode, V(0, 0, 0));
        return callout(labels.late.text, pinNode, V(0, 0, 0), labels.late.at[0] - px, labels.late.at[1] - py, pinAlpha);
      };
      return {
        shift: 280,
        state: { ...st, amp, focus: 'drive', accents: true },
        html: () => `<div class="fcap">The drive<small>shown at ¼ speed</small></div>
          <div class="panel" style="opacity:${a};width:620px"><h3>${fmt(N.ratio, 2)} : 1</h3><div class="sub">${['one', 'two', 'three', 'four', 'five'][stages.length - 1] ?? stages.length} spur stages · gears ${fmt(stages.reduce((g, x) => g + x.mass_g, 0), 0)} g${labels.fixed ? ` · fixed stroke ±${fmt(amp * DEG, 1)}°` : ''}</div>
          ${stageRows(stages)}
          ${row('motor', `${fmt(N.motorRpm, 0)} rpm`)}${row('crank', `${fmt(N.flapHz, 2)} Hz`)}${row('stroke angle', `${fmt(dp.right.phi * DEG, 1)}°`, 'acc')}${labels.fixed ? '' : row('amplitude', `±${fmt(amp * DEG, 1)}°`)}${row('transmission angle', `${fmt(dp.right.mu * DEG, 0)}°`)}</div>
          ${callout('Motor', node('motor'), V(0, 0.018, 0), -240, 40, a)}
          ${callout(labels.crankWheel, node('gear_crank'), V(0, -0.021, 0), -200, 80, a)}
          ${callout('Coupler', node('coupler_R'), V(0, 0.045, 0), 200, -170, a)}
          ${pinCallout()}${honest()}`,
      };
    },
    twist(u, t) {
      // target and orbit chosen so both wings (the twist focus draws both wings' ribs and film), root to tip, stay
      // inside the frame and off the caption, the panel and the claim over a full stroke cycle at any point in the
      // sequence: film/test/twist-camera.test.mjs pins this against the wingStations()/sectionPoint() geometry.
      const c = TWIST_CAM;
      orbit(V(...c.target), c.az0 + c.azSpan * ease(u), c.el, c.dist0 + c.distSpan * ease(u), c.fov);
      const st = flap(t, 0.25);
      const dp = drivePose(model, st.phase, st.amp);
      const tip = st.twistR * DEG;
      const a = easeOut(span(u, 0.05, 0.2));
      return {
        shift: TWIST_CAM.shift,
        state: { ...st, focus: 'twist', accents: true },
        html: () => `<div class="fcap">Active twist<small>¼ speed · nose down on the downstroke, up on the upstroke</small></div>
          <div class="panel" style="opacity:${a}"><h3>${tip >= 0 ? '+' : '−'}${fmt(Math.abs(tip), 1)}°</h3><div class="sub">commanded twist at the tip, ±${fmt(N.twistDynDeg, 0)}° per stroke</div>
          ${row('stroke angle', `${fmt(dp.right.phi * DEG, 1)}°`, 'acc')}${row('twist servo', model.parts[model.design.servos.twist.id]?.name?.split(' ')[0] ?? 'servo')}${row('ribs per wing', `${model.ribs.length}`)}</div>
          ${callout('Twist servo', node('twist_servo_R'), V(0, 0, 0), 120, -150, a)}${honest()}`,
      };
    },
    power(u, t) {
      const c = POWER_CAM;
      orbit(...powerOrbit(powerSphere, u));
      const st = flap(t);
      const a = easeOut(span(u, 0.05, 0.2));
      const m2 = data.gates.find((g) => g.gate === 'M2');
      return {
        shift: c.shift,
        state: { ...st, focus: 'power', also: servoIds, accents: true },
        html: () => `<div class="fcap">Power<small>${N.batteryCells}S ${fmt(N.batteryMah, 0)} mAh ${model.design.battery.chemistry ?? ''}, ${fmt(N.batteryWh, 1)} Wh${model.design.battery.charge_limit_V_per_cell ? ` · charged to ${fmt(model.design.battery.charge_limit_V_per_cell, 2)} V a cell` : ''}</small></div>
          <div class="panel" style="opacity:${a}"><h3>${fmt(N.cruisePowerW, 1)} W</h3><div class="sub">electrical power in cruise at ${fmt(N.cruiseV, 1)} m/s (M2)</div>
          ${row('endurance', `${fmt(N.enduranceMin, 1)} min`)}${row('gate M2', m2 ? gateVerdict(m2) : 'not run', m2 && gateVerdict(m2) !== 'pass' ? 'bad' : '')}
          ${power.map((p) => row(`${p.label} · ${p.name}`, `${fmt(p.mass_g, 0)} g`, '', 's')).join('')}</div>
          ${power.map((p, k) => calloutAt(p.label === 'ESC' ? 'ESC' : p.label[0].toUpperCase() + p.label.slice(1), node(p.node), POWER_LABELS[powerSlots[k]], a)).join('')}${honest()}`,
      };
    },
    gates(u, t) {
      orbit(V(-0.22, 0.0, 0), 210 + 20 * u, 14, 3.6, 30);
      const a = easeOut(span(u, 0.03, 0.18));
      const items = gatesCardItems(data.gates);
      return {
        state: { ...flap(t), shell: 1 },
        html: () => `<div style="position:absolute;inset:0;background:rgba(9,11,15,${0.78 * a})"></div><div class="card gc" style="opacity:${a}"><h2>Where it stands</h2><p>${N.passCount} of ${data.gates.length} gates pass for ${N.name}, design v${data.design_version}${((k) => (k ? `, ${k} not final` : ''))(data.gates.filter((g) => gateVerdict(g) === 'NOT FINAL').length)}. Failures stay on the record.</p><div class="gates">${items}</div></div>${honest(a)}`,
      };
    },
    credits(u, t) {
      orbit(V(-0.22, 0.0, 0), 230 + 15 * u, 12, 4.2, 30);
      const a = easeOut(span(u, 0.03, 0.2));
      return {
        state: { ...flap(t), shell: 1 },
        html: () => `<div style="position:absolute;inset:0;background:rgba(9,11,15,${0.84 * a})"></div><div class="card" style="opacity:${a}"><h2>Avis</h2><p>No physical bird exists. Every flight here is a simulation; every part is a real catalogue part, read from its listing.</p>
          <div class="credits">${(edit.credits ?? []).map((c) => `<div><b>${c[0]}</b> ${c[1]}</div>`).join('')}</div></div>${honest(a)}`,
      };
    },
  };
  SEQ.assembly._level = 0;
  {
    // the level phase for a still pose while parts assemble: right wing at phi = 0
    let best = 0, bv = Infinity;
    for (let i = 0; i < 360; i++) {
      const p = (i / 360) * 2 * Math.PI;
      const v = Math.abs(drivePose(model, p, N.ampDeg / DEG).right.phi);
      if (v < bv && p < Math.PI) {
        bv = v;
        best = p;
      }
    }
    SEQ.assembly._level = best;
  }

  // ---- flight plates: Blender frames behind a HUD that reads the same log -------------------------------------
  function plateFrame(seg, tl) {
    const p = seg.plate;
    const k = Math.min(p.n - 1, Math.max(0, Math.floor(tl * p.fps + 1e-6)));
    const idx = p.i0 + k;
    const d = segData[seg.id];
    const j = d ? Math.min(d.t.length - 1, k) : -1;
    const hud = flightHud(d, j);
    const cap = `<div class="fcap">${fillDesign(seg.caption ?? '', model.design)}<small>${fillDesign(seg.sub ?? '', model.design)}${p.slow > 1 ? ` · ${p.slow}× slow motion` : ''}</small></div>`;
    const scrim = '<div style="position:absolute;inset:0;background:linear-gradient(180deg,rgba(6,9,13,.55) 0%,rgba(6,9,13,0) 22%,rgba(6,9,13,0) 70%,rgba(6,9,13,.6) 100%)"></div>';
    const mark = segMarker(seg, d, j, N);
    return { src: `${plateBase}/${p.dir}/f_${String(idx).padStart(5, '0')}.png`, html: scrim + mark + cap + (seg.hud === false ? '' : hud) + honest() };
  }

  const segs = edit.segments;
  const duration = segs.reduce((m, s) => Math.max(m, s.start + s.dur), 0);
  let lastSrc = '';

  async function frame(t) {
    const seg = segs.find((s) => t >= s.start && t < s.start + s.dur) ?? segs[segs.length - 1];
    const tl = t - seg.start;
    const u = clamp(tl / seg.dur);
    let fa = 0;
    if (seg.fadeIn && tl < seg.fadeIn) fa = 1 - tl / seg.fadeIn;
    if (seg.fadeOut && tl > seg.dur - seg.fadeOut) fa = Math.max(fa, (tl - (seg.dur - seg.fadeOut)) / seg.fadeOut);
    fade.style.opacity = clamp(fa);
    if (seg.kind === 'bl') {
      const pf = plateFrame(seg, tl);
      canvas.style.opacity = 0;
      plate.style.opacity = 1;
      if (pf.src !== lastSrc) {
        plate.src = pf.src;
        lastSrc = pf.src;
        await plate.decode().catch(() => {});
      }
      ov.innerHTML = pf.html;
      return { seg: seg.id };
    }
    plate.style.opacity = 0;
    canvas.style.opacity = 1;
    const f = SEQ[seg.seq];
    f._dur = seg.dur;
    const r = f.call(f, u, tl);
    // frame the subject clear of the panel (right) or the title (left)
    const shift = r.shift ?? 0;
    if (shift) cam.setViewOffset(1920, 1080, shift, 0, 1920, 1080);
    else cam.clearViewOffset();
    bird.update({ accents: false, ...r.state });
    bird.root.updateMatrixWorld(true);
    cam.updateMatrixWorld(); // callouts project through this frame's camera, not the last one rendered
    ov.innerHTML = typeof r.html === 'function' ? r.html() : r.html;
    stage.render();
    return { seg: seg.id };
  }
  return { frame, duration, fps: edit.fps ?? 30, stage, bird };
}
