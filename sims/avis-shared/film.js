// The film as one page: render(t) is a pure function of time. Studio sequences (assembly, drive, twist, power,
// cards) pose the bird from the same model as the page; flight shots are Blender plates (PNG frames rendered from
// the flight logs) shown behind a HUD that reads the same log. The edit (film/edit.json, resolved by
// film/render.mjs into site/data/edit.json) decides the order and timing; the score reads the same edit.
import * as THREE from 'three';
import { makeMaterials, ACCENT } from './materials.js';
import { buildBird } from './bird.js';
import { createStage } from './stage.js';
import { numbers, gateLine } from './numbers.js';
import { drivePose } from './model.js';

const DEG = 180 / Math.PI;
const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const ease = (x) => (x < 0.5 ? 4 * x * x * x : 1 - (-2 * x + 2) ** 3 / 2);
const easeOut = (x) => 1 - (1 - clamp(x)) ** 3;
const span = (u, a, b) => clamp((u - a) / (b - a));
const fmt = (v, d = 0) => (v == null || Number.isNaN(v) ? '—' : v.toLocaleString('en-US', { minimumFractionDigits: d, maximumFractionDigits: d }));

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
.titleblock{position:absolute;left:120px;top:330px}
.titleblock h1{font-size:200px;line-height:.9;margin:0;font-weight:800;letter-spacing:-.04em}
.titleblock h2{font-size:46px;margin:28px 0 0;font-weight:500;color:#c9d3dc;letter-spacing:-.01em}
.titleblock .rule{width:96px;height:4px;background:${ACCENT};border-radius:2px;margin-top:36px}
.panel{position:absolute;right:72px;top:96px;width:560px;padding:34px 38px;border-radius:24px;background:rgba(14,18,24,.66);backdrop-filter:blur(20px) saturate(1.4);box-shadow:0 30px 60px #0007,inset 0 1px 0 rgba(255,255,255,.06)}
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
.credits{columns:2;column-gap:80px;font-size:30px;line-height:1.45;color:#c9d3dc}
.credits b{color:#fff;font-weight:600}
`;

/** Mass per subsystem from the BOM, by the role or item name. */
export function subsystemMass(design) {
  const sub = (s) => (/motor|gear|bearing|stroke|crank|coupler/i.test(s) ? 'drive' : /battery|ESC/i.test(s) ? 'power' : /flight controller|GPS|airspeed|receiver|tail servo/i.test(s) ? 'avionics' : /twist servo|spar|edge rod|wing film|ribs/i.test(s) ? 'wing' : 'structure');
  const m = { drive: 0, power: 0, avionics: 0, wing: 0, structure: 0 };
  for (const p of design.bom.parts) m[sub(p.role)] += p.mass_g;
  for (const s of design.bom.structure) m[sub(s.name)] += s.mass_g;
  return m;
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
  const mats = makeMaterials({ textures: true });
  const bird = buildBird(model, mats);
  stage.scene.add(bird.root);
  const massBy = subsystemMass(model.design);
  const cam = stage.camera;
  const V = (x, y, z) => new THREE.Vector3(x, y, z);

  const orbit = (target, azDeg, elDeg, dist, fov = 30) => {
    const az = azDeg / DEG, el = elDeg / DEG;
    cam.fov = fov;
    cam.position.set(target.x + dist * Math.cos(el) * Math.cos(az), target.y + dist * Math.sin(el), target.z + dist * Math.cos(el) * Math.sin(az));
    cam.lookAt(target);
    cam.updateProjectionMatrix();
    stage.aimLights(target, 1.3);
  };
  const flap = (t, speed = 1) => {
    const ph = 2 * Math.PI * N.flapHz * t * speed;
    return { phase: ph, amp: N.ampDeg / DEG, twistL: N.twistAt(ph, -1), twistR: N.twistAt(ph, 1) };
  };
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
  const honest = (a = 1) => `<div class="honest" style="opacity:${0.92 * a}"><b>●</b>&nbsp; simulated flight · real parts</div>`;
  const row = (k, v, cls = '') => `<div class="frow"><span>${k}</span><b class="${cls}">${v}</b></div>`;
  const node = (n) => bird.root.getObjectByName(n);

  // ---- studio sequences: (u in 0..1, t seconds into the sequence, dur) -> { state, html } with the camera set ---
  const SEQ = {
    title(u, t) {
      orbit(V(-0.22, 0.02, 0), 32 - 10 * u, 16 - 4 * u, 3.3 - 0.4 * ease(u), 30);
      const a = easeOut(span(u, 0.05, 0.3));
      return {
        shift: -300,
        state: flap(t),
        html: () => `<div class="titleblock" style="opacity:${a};transform:translateY(${(1 - a) * 24}px)"><h1>Avis</h1><h2>${N.name} · a gull built from catalogue parts</h2><div class="rule"></div></div>${honest(a)}`,
      };
    },
    assembly(u, t) {
      const ex = {
        structure: 1 - ease(span(u, 0.0, 0.42)),
        drive: 1 - ease(span(u, 0.12, 0.55)),
        power: 1 - ease(span(u, 0.22, 0.62)),
        avionics: 1 - ease(span(u, 0.3, 0.7)),
        wing: 1 - ease(span(u, 0.4, 0.86)),
      };
      orbit(V(-0.15, 0.0, 0), 150 + 45 * ease(u), 24 - 6 * u, 2.7 - 0.25 * u, 30);
      const fl = span(u, 0.88, 1);
      const st = fl > 0 ? flap(t - 0.88 * this._dur) : { phase: this._level, amp: N.ampDeg / DEG };
      let g = 0;
      for (const k of Object.keys(ex)) g += massBy[k] * (1 - ex[k]);
      const done = Object.values(ex).filter((v) => v < 0.02).length;
      g += N.margin_g * span(u, 0.86, 0.95);
      return {
        shift: 260,
        state: { ...st, explode: ex, shell: 1 },
        html: () => `<div class="fcap">Every part from a catalogue<small>${N.bomLines} BOM lines, ${N.partCount} bought parts, each with a source</small></div>
          <div class="panel" style="top:auto;bottom:120px"><h3>${fmt(g, 0)} g</h3><div class="sub">parts ${fmt(N.parts_g, 0)} g · structure ${fmt(N.structure_g, 0)} g · 15% margin ${fmt(N.margin_g, 0)} g</div>
          ${row('subsystems in place', `${done} / 5`)}${row('span', `${fmt(N.span_m, 2)} m`)}${row('wing area', `${fmt(N.area_m2, 3)} m²`)}</div>${honest()}`,
      };
    },
    drive(u, t) {
      const target = V(0.004, -0.028, 0);
      orbit(target, 205 - 30 * ease(u), -6 + 8 * u, 0.36 - 0.04 * ease(u), 30);
      const speed = 0.25;
      const st = flap(t, speed);
      // the stroke servo walks the amplitude through its range in the second half
      const s = span(u, 0.5, 0.95);
      const { ampMin, ampMax } = model.law;
      const nom = st.amp;
      let amp = nom;
      if (s > 0) amp = s < 1 / 3 ? nom + (ampMax - nom) * ease(s * 3) : s < 2 / 3 ? ampMax + (ampMin - ampMax) * ease(s * 3 - 1) : ampMin + (nom - ampMin) * ease(s * 3 - 2);
      const dp = drivePose(model, st.phase, amp);
      const a = easeOut(span(u, 0.04, 0.2));
      return {
        shift: 280,
        state: { ...st, amp, focus: 'drive', accents: true },
        html: () => `<div class="fcap">The drive<small>shown at ¼ speed</small></div>
          <div class="panel" style="opacity:${a}"><h3>${fmt(N.ratio, 2)} : 1</h3><div class="sub">${['one', 'two', 'three', 'four'][N.stages.length - 1] ?? N.stages.length} spur stages, ${N.stages.join(' × ')} teeth</div>
          ${row('motor', `${fmt(N.motorRpm, 0)} rpm`)}${row('crank', `${fmt(N.flapHz, 2)} Hz`)}${row('stroke angle', `${fmt(dp.right.phi * DEG, 1)}°`, 'acc')}${row('amplitude', `±${fmt(amp * DEG, 1)}°`)}${row('transmission angle', `${fmt(dp.right.mu * DEG, 0)}°`)}</div>
          ${callout('Motor', node('motor'), V(0, 0.018, 0), -240, 40, a)}
          ${callout('Crank wheel 40T', node('gear_crank'), V(0, -0.021, 0), -200, 80, a)}
          ${callout('Coupler', node('coupler_R'), V(0, 0.045, 0), 200, -170, a)}
          ${s > 0 ? callout('Stroke servo slides the pin', node('slot_pin_R'), V(0, 0, 0), 140, 300, easeOut(span(u, 0.5, 0.6))) : ''}${honest()}`,
      };
    },
    twist(u, t) {
      const target = V(-0.06, 0.12, 0.5);
      orbit(target, 22 + 26 * ease(u), 12, 1.8 - 0.15 * ease(u), 32);
      const st = flap(t, 0.25);
      const dp = drivePose(model, st.phase, st.amp);
      const tip = st.twistR * DEG;
      const a = easeOut(span(u, 0.05, 0.2));
      return {
        shift: 260,
        state: { ...st, focus: 'twist', accents: true },
        html: () => `<div class="fcap">Active twist<small>¼ speed · nose down on the downstroke, up on the upstroke</small></div>
          <div class="panel" style="opacity:${a}"><h3>${tip >= 0 ? '+' : '−'}${fmt(Math.abs(tip), 1)}°</h3><div class="sub">commanded twist at the tip, ±${fmt(N.twistDynDeg, 0)}° per stroke</div>
          ${row('stroke angle', `${fmt(dp.right.phi * DEG, 1)}°`, 'acc')}${row('twist servo', model.parts[model.design.servos.twist.id]?.name?.split(' ')[0] ?? 'servo')}${row('ribs per wing', `${model.ribs.length}`)}</div>
          ${callout('Twist servo', node('twist_servo_R'), V(0, 0, 0), 120, -150, a)}${honest()}`,
      };
    },
    power(u, t) {
      const target = V(0.15, -0.008, 0);
      orbit(target, 30 + 25 * ease(u), -22 + 8 * u, 0.6 - 0.08 * ease(u), 30);
      const st = flap(t);
      const a = easeOut(span(u, 0.05, 0.2));
      const m2 = data.gates.find((g) => g.gate === 'M2');
      return {
        shift: 280,
        state: { ...st, focus: 'power', accents: true },
        html: () => `<div class="fcap">Power<small>${N.batteryCells}S ${fmt(N.batteryMah, 0)} mAh LiPo, ${fmt(N.batteryWh, 1)} Wh</small></div>
          <div class="panel" style="opacity:${a}"><h3>${fmt(N.cruisePowerW, 1)} W</h3><div class="sub">electrical power in cruise at ${fmt(N.cruiseV, 1)} m/s (M2)</div>
          ${row('endurance', `${fmt(N.enduranceMin, 1)} min`)}${row('power available / required', `${fmt(N.powerRatio, 1)}`)}${row('gate M2', m2 ? (m2.passed ? 'pass' : 'FAIL') : 'not run', m2 && !m2.passed ? 'bad' : '')}</div>
          ${callout('Battery', node('part_battery'), V(0, 0, 0), -170, 110, a)}${callout('ESC', node('part_esc'), V(0, 0, 0), -120, 170, a)}${honest()}`,
      };
    },
    gates(u, t) {
      orbit(V(-0.22, 0.0, 0), 210 + 20 * u, 14, 3.6, 30);
      const a = easeOut(span(u, 0.03, 0.18));
      const order = ['M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M7', 'M8', 'V1', 'V2', 'V3', 'V4', 'V5', 'V6', 'V7'];
      const items = order.map((g) => {
        const r = data.gates.find((x) => x.gate === g);
        const v = !r ? '<span class="v" style="color:#8d99a6">not run</span>' : r.passed ? '<span class="v ok">pass</span>' : '<span class="v no">FAIL</span>';
        return `<div class="g"><em>${g}</em><span>${r ? gateLine(r) : 'no result yet'}</span>${v}</div>`;
      }).join('');
      return {
        state: { ...flap(t), shell: 1 },
        html: () => `<div style="position:absolute;inset:0;background:rgba(9,11,15,${0.78 * a})"></div><div class="card" style="opacity:${a}"><h2>Where it stands</h2><p>${N.passCount} of ${data.gates.length} gates pass for ${N.name}, design v${data.design_version}. Failures stay on the record.</p><div class="gates">${items}</div></div>${honest(a)}`,
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
    const hud = d && j >= 0 ? `<div class="fhud"><div>airspeed<b>${fmt(d.airspeed[j], 1)} m/s</b></div><div>electrical power, flap-cycle mean<b>${fmt(d.power_w[j], 0)} W</b></div><div>battery<b>${fmt(d.battery_pct[j], 0)}%</b></div></div>` : '';
    const cap = `<div class="fcap">${seg.caption ?? ''}<small>${seg.sub ?? ''}${p.slow > 1 ? ` · ${p.slow}× slow motion` : ''}</small></div>`;
    const scrim = '<div style="position:absolute;inset:0;background:linear-gradient(180deg,rgba(6,9,13,.55) 0%,rgba(6,9,13,0) 22%,rgba(6,9,13,0) 70%,rgba(6,9,13,.6) 100%)"></div>';
    return { src: `${plateBase}/${p.dir}/f_${String(idx).padStart(5, '0')}.png`, html: scrim + cap + (seg.hud === false ? '' : hud) + honest() };
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
