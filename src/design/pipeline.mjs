#!/usr/bin/env node
// Design pipeline for one candidate: coarse screen -> top designs -> trim, controller, trim by flight -> quick
// missions -> the best feasible design is written to data/designs/<name>.json. Checkpoints in out/sweep/.
//   node src/design/pipeline.mjs larus [--top 6]
import { readFileSync, writeFileSync, existsSync, mkdirSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { screen } from './screen.js';
import { buildDesign, CANDIDATES } from './build.js';
import { buildControl, refineTrimByFlight, trimLevel } from './trim.js';
import { buildBird } from '../physics/flight.js';
import { validateDesign } from './schema.js';
import { runM1, runM2, runM3, runM4Seed, runM5 } from '../physics/scenarios.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const DEG = Math.PI / 180;
const cand = process.argv[2];
const top = +(process.argv[process.argv.indexOf('--top') + 1] || 6);
if (!CANDIDATES[cand]) {
  console.error('usage: pipeline.mjs larus|falco|columba');
  process.exit(2);
}
const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
mkdirSync(join(ROOT, 'out/sweep'), { recursive: true });
const ckpt = join(ROOT, `out/sweep/${cand}-pipeline.json`);
const state = existsSync(ckpt) && process.argv.includes('--resume') ? JSON.parse(readFileSync(ckpt, 'utf8')) : { cand, evaluated: [] };
const save = () => writeFileSync(ckpt, JSON.stringify(state, null, 1));

const t0 = Date.now();
const ranked = screen(cand, catalog);
state.screen = { count: ranked.length, seconds: (Date.now() - t0) / 1000, best: ranked.slice(0, 20).map((r) => ({ minMargin: r.minMargin, mass_g: r.mass_g, margins: r.margins, span: r.params.span_m, AR: r.params.AR, motor: r.params.parts.motor, battery: r.params.parts.battery, twist: r.params.parts.twistServo, G: r.params.gearRatio })) };
console.log(`${cand}: screened ${ranked.length} designs in ${state.screen.seconds.toFixed(1)} s`);
save();

// Pick distinct designs: different span/AR/battery/motor combinations from the top of the ranking.
// The screen's min-margin favours the lightest parts; rough air (M4) rewards power headroom and wing loading, which
// the closed form cannot judge. So from the designs within 0.15 of the best margin, take distinct motor and
// battery pairs with the most power headroom.
const bestM = ranked[0]?.minMargin ?? 0;
const pool = ranked.filter((r) => r.minMargin >= Math.min(bestM, 0) - 0.15);
pool.sort((a, b) => b.margins.power + b.margins.endurance + 0.02 * b.ws - (a.margins.power + a.margins.endurance + 0.02 * a.ws));
const picks = [];
const seen = new Set();
for (const r of [...pool, ...ranked]) {
  const key = [r.params.parts.battery, r.params.parts.motor].join('|');
  if (seen.has(key)) continue;
  seen.add(key);
  picks.push(r);
  if (picks.length >= top) break;
}

function quickMissions(d) {
  const out = {};
  const keep = (r) => ({ passed: r.passed, measured: r.measured, peaks: r.peaks });
  out.M1 = keep(runM1(d, {}));
  out.M2 = keep(runM2(d, { tEnd: 60 }));
  out.M3 = keep(runM3(d, {}));
  out.M5 = keep(runM5(d, {}));
  const m4 = [];
  for (let s = 1; s <= 8; s++) m4.push(runM4Seed(d, s, {}));
  out.M4 = { passed: m4.filter((r) => r.passed).length, of: m4.length, worstAlt: Math.max(...m4.map((r) => r.measured.altitude_dev_m)), worstRoll: Math.max(...m4.map((r) => r.measured.max_roll_deg)), worstPitch: Math.max(...m4.map((r) => r.measured.max_pitch_deg)) };
  return out;
}

for (const [k, pick] of picks.entries()) {
  if (state.evaluated.find((e) => e.k === k)) continue;
  const p = { ...pick.params };
  const tStart = Date.now();
  const d = buildDesign(p, catalog);
  const bird = buildBird(d);
  // Cruise schedule: twist amplitude chosen for least trimmed power at the screen's cruise speed.
  const Vc = Math.max(6, Math.round(pick.V * 2) / 2);
  let best = null;
  for (const dyn of [8, 14, 20]) {
    const t = trimLevel(d, bird, { V: Vc, amp: p.amp_deg * DEG, dyn: dyn * DEG });
    if (t && t.converged && (!best || t.power < best.t.power)) best = { dyn, t };
  }
  if (!best) {
    state.evaluated.push({ k, params: p, failed: 'no trim at cruise speed', minutes: (Date.now() - tStart) / 60000 });
    save();
    continue;
  }
  const speeds = [];
  for (let v = Math.max(5, Math.round(pick.Vs * 1.1 * 2) / 2); v <= Math.max(Vc + 4, pick.Vs * 2.4); v += 0.5) speeds.push(+v.toFixed(2));
  const ctl = buildControl(d, { amp: p.amp_deg * DEG, sym: 0, dyn: best.dyn * DEG, lead: 0, V: Vc }, { speeds, bird });
  if (!ctl) {
    state.evaluated.push({ k, params: p, failed: 'no trim table', minutes: (Date.now() - tStart) / 60000 });
    save();
    continue;
  }
  d.control = ctl;
  await refineTrimByFlight(d);
  // Cruise at the least flight power, rough air at the fastest trimmed speed up to cruise + 3 m/s.
  const tf = (d.control.trimFlight ?? []).filter((r) => !r.unflyable);
  if (tf.length) {
    const minP = tf.reduce((a, b) => (b.flight_power_w < a.flight_power_w ? b : a));
    d.control.Vcruise = minP.V;
    d.control.Vrough = Math.min(tf[tf.length - 1].V, minP.V + 3);
  }
  d.cruise = { V: d.control.Vcruise, amp: p.amp_deg * DEG, dyn: best.dyn * DEG, sym: 0, lead: 0 };
  const errs = validateDesign(d);
  const q = quickMissions(d);
  const passCount = ['M1', 'M2', 'M3', 'M5'].filter((m) => q[m].passed).length + (q.M4.passed === q.M4.of ? 1 : 0);
  state.evaluated.push({ k, params: p, screen: { minMargin: pick.minMargin, mass_g: pick.mass_g }, valid: errs, quick: q, passCount, m4: q.M4.passed / q.M4.of, mass_g: d.mass.total_g, minutes: (Date.now() - tStart) / 60000, design: d });
  console.log(`${cand} #${k}: ${d.mass.total_g.toFixed(0)} g, quick ${passCount}/5 (M4 ${q.M4.passed}/${q.M4.of}) in ${((Date.now() - tStart) / 60000).toFixed(1)} min`);
  save();
}

// Choose: most quick passes, then the M4 rate, then lighter.
const ok = state.evaluated.filter((e) => e.design);
ok.sort((a, b) => b.m4 - a.m4 || b.passCount - a.passCount || a.mass_g - b.mass_g);
if (ok.length) {
  const win = ok[0].design;
  win.selection = { pipeline: 'src/design/pipeline.mjs', screened: state.screen.count, refined: ok.length, rank: ok.map((e) => ({ k: e.k, passCount: e.passCount, m4: e.m4, mass_g: e.mass_g })), quick: ok[0].quick };
  writeFileSync(join(ROOT, `data/designs/${win.name}.json`), JSON.stringify(win, null, 1));
  console.log(`${cand}: wrote data/designs/${win.name}.json`);
}
for (const e of state.evaluated) delete e.design;
state.chosen = ok.length ? ok[0].k : null;
save();
