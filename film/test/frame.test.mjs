// Run: node --import ./film/lib/three-hooks.mjs --test film/test/frame.test.mjs
// The airframe frame.js draws (src/design/airframe.js's load path): every number checked here is parsed, in this
// file, from the design's own BOM basis text (or the catalogue spec for ties) - never against frame.js's own
// output - then compared to the actual drawn geometry (its local bounding box, and for hollow shapes the inner
// radius read straight off the vertices, the same technique film/test/bird.test.mjs uses for bearing rings).
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird } from '../lib/bird-node.mjs';
import { prepare } from '../../sims/avis-shared/model.js';
import { buildBird } from '../../sims/avis-shared/bird.js';
import { makeMaterials } from '../../sims/avis-shared/materials.js';
import {
  offCount, parseTube, parseRing, parseBlocks, parsePlate, parseBracket, parseChannel, parsePanel,
  splitEven, scatter, screwGeo, nutGeo, tieGeo, toM,
} from '../../sims/avis-shared/frame.js';
import { lockFlangeCircles } from '../../src/design/airframe.js';

const { bird, design, model } = nodeBird('larus-gl-2');
const EPS = 1e-6;
const planeMid = (model.planes.plateAft + model.planes.plateFwd) / 2;

const part = (name) => bird.parts.find((p) => p.name === name);
const size = (o) => new THREE.Box3().setFromBufferAttribute(o.geometry.attributes.position).getSize(new THREE.Vector3());
const sortedDims = (o) => [size(o).x, size(o).y, size(o).z].sort((a, b) => a - b);
function minRadius(o) {
  const p = o.geometry.attributes.position.array;
  let r = Infinity;
  for (let i = 0; i < p.length; i += 3) r = Math.min(r, Math.hypot(p[i + 1], p[i + 2]));
  return r;
}
const close = (a, b, eps = EPS) => assert.ok(Math.abs(a - b) < eps, `${a} vs ${b}`);
const closeDims = (o, mmList) => {
  const expect = mmList.map((v) => v / 1000).sort((a, b) => a - b);
  sortedDims(o).forEach((v, i) => close(v, expect[i]));
};

const FRAME_NAME_RE = /^(shoulder rail|shoulder bracket|tail boom clamp blocks|forward tray|aft tray|motor doubler|gearbox panels)/;
const FRAME_MATERIAL_IDS = ['screw-iso4762-m3x8-10.9', 'nut-iso4032-m3-10', 'tie-hellermanntyton-t18r'];

// v22 on, the rails, brackets, clamps, trays, panels and boom are the physics' own solids (film/test/physics-solids.
// test.mjs holds the drawing to them). Here the physics' solids are checked against the BOM's basis text, which the
// physics also writes: they must describe the same part. Disagreements found are reported to the physics, listed in
// REPORTED with both numbers; each must still disagree (so the list is pruned when the physics corrects it).
const ph = bird.phys;
const sol = (id, name) => { const s = ph.solid(id, name); assert.ok(s, `${id}/${name}`); return s; };
const xLen = (s) => s.x[1] - s.x[0];
// (v24 corrected both v23 entries: the boom's BOM length is its solid's, the panels' the sheet between the plates)
const REPORTED = {
  // BOM "less a 2.5 mm minor-diameter bore" (the nut's mass); the physics bores it to the 3 mm shank it carries (the
  // thread is not drawn): reported, the film draws the physics' bore
  'M3 nut bore': { bom: 2.5, physics: 3 },
};
const found = {};
const agree = (what, bom, physics, tol = 0.05) => {
  if (Math.abs(bom - physics) <= tol) return assert.ok(!REPORTED[what], `${what}: reported as ${JSON.stringify(REPORTED[what])}, now agrees (${bom} mm)`);
  found[what] = true;
  assert.ok(REPORTED[what] && Math.abs(REPORTED[what].bom - bom) < 0.05 && Math.abs(REPORTED[what].physics - physics) < 0.05, `${what}: BOM ${bom} mm vs physics ${physics.toFixed(2)} mm`);
};

test('shoulder rails: the physics\' tube and lock flange at the BOM\'s OD, ID, length and flange thickness, at the design\'s rail position', () => {
  for (const side of ['right', 'left']) {
    const item = design.bom.structure.find((s) => s.name === `shoulder rail ${side}`);
    const [, od, id, len] = item.basis.match(/([\d.]+)\/([\d.]+)\s*mm tube x\s*([\d.]+)\s*mm/).map(Number);
    const tube = sol(`shoulder rail ${side}`, 'tube'), lock = sol(`shoulder rail ${side}`, 'lock flange');
    agree(`rail ${side} OD`, od, 2000 * tube.shape.ro, 1e-6);
    agree(`rail ${side} ID`, id, 2000 * tube.shape.ri, 1e-6);
    agree(`rail ${side} length`, len, 1000 * xLen(tube));
    agree(`rail ${side} lock flange`, +item.basis.match(/([\d.]+) mm lock flange/)[1], 1000 * xLen(lock), 1e-6);
    assert.deepEqual(tube.shape.c, design.structure.airframe.parts.rails[side]);
    // the lock flange where the design says it is screwed: v22 the forward plate's aft face, v24 ('aft outer') the
    // aft plate's aft face, else the aft plate's forward face
    const at = design.params.linkage.airframe.rail.lock_at, PL = design.params.linkage.plates;
    if (at === 'forward') close(lock.x[1], model.planes.plateFwd - PL.t_fwd_mm / 2000, 1e-9);
    else if (at === 'aft outer') close(lock.x[1], model.planes.plateAft - PL.t_mm / 2000, 1e-9);
    else close(lock.x[0], model.planes.plateAft + PL.t_mm / 2000, 1e-9);
    assert.deepEqual(part(item.name).objects.map((o) => o.name).sort(), [`frame_rail_${side}`, `frame_rail_lock_${side}`].sort());
  }
});

test('shoulder rail aft seats: each rail through its seat in the aft plate, its seat boss drawn where the BOM lists one', () => {
  // v23 drew each rail's aft seat boss from its BOM ring; v24 removed the bosses (the rail locks moved to the aft
  // plate's aft face, params.linkage.airframe.rail.aft_boss_mm 0). The promise is the same: the rail is seated in the
  // aft plate as the design says: through a hole in the plate's own solid, centred on the rail, clear of its tube
  const R = design.params.linkage.airframe.rail, plate = sol('gearbox plate aft', 'plate');
  for (const side of ['right', 'left']) {
    const boss = design.bom.structure.find((s) => s.name === `shoulder rail ${side} aft seat boss`);
    const pos = design.structure.airframe.parts.rails[side], tube = sol(`shoulder rail ${side}`, 'tube');
    if (boss) {
      const [, od, id, t] = boss.basis.match(/ring\s*([\d.]+)\/([\d.]+)\s*x\s*([\d.]+)\s*mm/).map(Number);
      const o = part(boss.name).objects[0];
      close(size(o).x, t / 1000); close(size(o).y, od / 1000); close(2 * minRadius(o), id / 1000);
      continue;
    }
    assert.ok(!R.aft_boss_mm, 'no boss in the BOM, none in the design');
    assert.ok(!ph.parts.some((p) => new RegExp(`seat boss`).test(p.id)), 'and none in the physics');
    const seat = plate.shape.holes.find((h) => Math.hypot(h.c[0] - pos[0], h.c[1] - pos[1]) < 1e-9);
    assert.ok(seat, `${side} rail's seat in the aft plate`);
    assert.ok(seat.r >= tube.shape.ro - 1e-9, `${side} seat ${seat.r} round the tube ${tube.shape.ro}`);
    assert.ok(tube.x[0] < plate.x[0] && tube.x[1] > plate.x[1], `${side} rail through the plate`);
  }
});

test('shoulder brackets: the physics\' sleeves and arms at the BOM\'s sizes, a sleeve on the rail and an arm to each housing', () => {
  for (const [side, tag] of [['right', 'R'], ['left', 'L']]) {
    const item = design.bom.structure.find((s) => s.name === `shoulder bracket ${side}`);
    const b = parseBracket(item.basis);
    const rail = design.structure.airframe.parts.rails[side];
    for (const nm of ['aft', 'forward']) {
      const sl = sol(`shoulder bracket ${tag} ${nm}`, 'sleeve'), arm = sol(`shoulder bracket ${tag} ${nm}`, 'arm');
      agree('bracket sleeve OD', b.sOd, 2000 * sl.shape.ro, 1e-6);
      agree('bracket sleeve ID', b.sId, 2000 * sl.shape.ri, 1e-6);
      agree('bracket sleeve length', b.sLen, 1000 * xLen(sl), 1e-6);
      assert.deepEqual(sl.shape.c, rail);
      agree('bracket arm thickness', b.aH, 1000 * xLen(arm), 1e-6);
      // the arm a bar of the BOM's width; its length the BOM's (v24: "from the bushing to the rail", the bar's own
      // length, which the physics draws from its housing's ring to its sleeve's)
      const pts = arm.shape.pts, w = Math.hypot(pts[1][0] - pts[2][0], pts[1][1] - pts[2][1]);
      agree('bracket arm width', b.aB, 1000 * w, 1e-6);
      agree('bracket arm length', b.aLen, 1000 * Math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1]));
      const hous = sol(`shoulder bracket ${tag} ${nm}`, 'housing'), end = (i, j) => [(pts[i][0] + pts[j][0]) / 2, (pts[i][1] + pts[j][1]) / 2];
      const d = (p, c) => Math.hypot(p[0] - c[0], p[1] - c[1]);
      assert.ok(d(end(0, 3), hous.shape.c) <= hous.shape.ro && d(end(1, 2), sl.shape.c) <= sl.shape.ro, 'the arm from its housing to its sleeve');
    }
    assert.equal(part(item.name).objects.length, 4);
  }
});

test('tail boom clamp blocks: 2 off, the physics\' blocks at the BOM\'s section and length, bored for the boom, clamp_spacing_m apart', () => {
  const item = design.bom.structure.find((s) => /^tail boom clamp blocks/.test(s.name));
  const d = parseBlocks(item.basis);
  const bore = +item.basis.match(/bored\s*([\d.]+)\s*mm/)[1];
  const blocks = ['aft', 'forward'].map((nm) => sol(`boom clamp ${nm}`, 'block'));
  // v24: each block a square collar with a screw flange below it, at the BOM's width and depth
  if (d.flange) for (const nm of ['aft', 'forward']) {
    const f = sol(`boom clamp ${nm}`, 'flange'), zs = f.shape.pts.map((p) => p[0]), ys = f.shape.pts.map((p) => p[1]);
    agree('clamp flange width', d.flange.w, 1000 * (Math.max(...zs) - Math.min(...zs)));
    agree('clamp flange depth', d.flange.h, 1000 * (Math.max(...ys) - Math.min(...ys)));
    agree('clamp flange length', d.len, 1000 * xLen(f), 1e-6);
  }
  for (const b of blocks) {
    const zs = b.shape.pts.map((p) => p[0]), ys = b.shape.pts.map((p) => p[1]);
    agree('clamp block width', d.a, 1000 * (Math.max(...zs) - Math.min(...zs)));
    agree('clamp block height', d.b, 1000 * (Math.max(...ys) - Math.min(...ys)));
    agree('clamp block length', d.len, 1000 * xLen(b), 1e-6);
    agree('clamp bore', bore, 2000 * b.shape.holes[0].r, 1e-6);
    assert.deepEqual(b.shape.holes[0].c, [0, design.structure.airframe.parts.boom_y_m]);
  }
  close((blocks[1].x[0] + blocks[1].x[1]) / 2 - (blocks[0].x[0] + blocks[0].x[1]) / 2, design.structure.airframe.parts.clamp_spacing_m, 1e-6);
  assert.equal(part(item.name).objects.length, 2);
});

test('forward and aft trays: the physics\' channels at the BOM\'s width, flange height, sheet and length from the plate', () => {
  for (const tag of ['forward', 'aft']) {
    const item = design.bom.structure.find((s) => s.name === `${tag} tray`);
    // the channel's width, flange height and sheet, then its length (v22: "to N mm from the plate"; v23: "x N mm")
    const m = item.basis.match(/channel\s*([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/).slice(1).map(Number);
    const c = { w: m[0], flangeH: m[1], t: m[2] };
    const fromPlate = +(item.basis.match(/to\s*([\d.]+)\s*mm from the plate/) ?? item.basis.match(/mm x\s*([\d.]+)\s*mm/))[1];
    const base = sol(`${tag} tray`, 'base'), fl = sol(`${tag} tray`, 'flange right');
    // the BOM's channel by its sheet's centreline (its area is (w + 2 h) t): base and flanges as capsules along it
    agree(`${tag} tray width`, c.w, 1000 * Math.abs(base.shape.b[0] - base.shape.a[0]), 1e-6);
    agree(`${tag} tray flange height`, c.flangeH, 1000 * Math.abs(fl.shape.b[1] - fl.shape.a[1]), 1e-6);
    agree(`${tag} tray sheet`, c.t, 2000 * base.shape.r, 1e-6);
    agree(`${tag} tray length from the plate`, fromPlate, 1000 * xLen(base));
    close(base.shape.a[1], design.structure.airframe.parts.trays[tag].y_m, 1e-12);
  }
});

test('gearbox panels and the boom: the physics\' sheets at the BOM\'s height and thickness where the design places them; lengths against the BOM', () => {
  const item = design.bom.structure.find((s) => /^gearbox panels/.test(s.name));
  // v23 "each N mm plus two M mm flanges"; v24 "each between the plates' inner faces (N mm)", the flanges by area
  const gap = parsePanel(item.basis, 'top panel').gap;
  const list = design.params.linkage.airframe.panels.list;
  for (const nm of ['side panel right', 'side panel left', 'top panel']) {
    const [, a, t] = item.basis.match(new RegExp(`${nm}\\s+([\\d.]+)\\s*x\\s*([\\d.]+)\\s*mm`)).map(Number);
    const s = sol(`gearbox ${nm}`, 'sheet'), e = list.find((p) => p.name === nm);
    agree(`${nm} height`, a, 1000 * Math.hypot(s.shape.b[0] - s.shape.a[0], s.shape.b[1] - s.shape.a[1]), 1e-6);
    agree(`${nm} thickness`, t, 2000 * s.shape.r, 1e-6);
    if (nm === 'top panel') close(s.shape.a[1], e.y_mm / 1000, 1e-12);
    else { close(s.shape.a[0], e.z_mm / 1000, 1e-12); close(s.shape.a[1], e.y0_mm / 1000, 1e-12); }
    agree('gearbox panel length between the plates', gap, 1000 * xLen(s));
    // v24: its flanges, each end's area as the BOM states it
    const fa = item.basis.match(new RegExp(`${nm}[^;]*?flanges\\s*([\\d.]+)\\s*mm2 each end`));
    if (fa) {
      const p = ph.part(`gearbox ${nm}`), area = (sh) => Math.abs(sh.pts.reduce((A, q, i) => A + q[0] * sh.pts[(i + 1) % sh.pts.length][1] - sh.pts[(i + 1) % sh.pts.length][0] * q[1], 0)) / 2;
      for (const end of ['aft', 'fwd']) agree(`${nm} flanges ${end}`, +fa[1], 1e6 * p.solids.filter((x) => x.name.startsWith(`flange ${end}`)).reduce((A, x) => A + area(x.shape), 0), 0.5);
    }
  }
  assert.equal(part(item.name).objects.length, 3);
  const keel = design.bom.structure.find((s) => /keel/.test(s.name));
  agree('keel and tail boom length', 1000 * +keel.basis.match(/x\s*([\d.]+)\s*m\b/)[1], 1000 * xLen(sol('tail boom', 'tube')));
});


// The BOM-text path frame.js keeps for a design the physics does not lay out (no params.layout): its defaults.
const withoutLayout = (mutate) => {
  const d = structuredClone(design);
  delete d.params.layout;
  mutate(d);
  const m = prepare(d, model.parts);
  return { m, b: buildBird(m, makeMaterials({ textures: false })) };
};

test('gearbox panels (no physics layout): without design.params.linkage.airframe.panels.list, fall back to default offsets', () => {
  const { b: b2 } = withoutLayout((d2) => delete d2.params.linkage.airframe.panels);
  const objs = b2.parts.find((p) => p.name === 'gearbox panels, 3 off').objects;
  const right = objs.find((o) => o.name === 'frame_panel_side_panel_right');
  const left = objs.find((o) => o.name === 'frame_panel_side_panel_left');
  const top = objs.find((o) => o.name === 'frame_panel_top_panel');
  close(right.position.z, 31 / 1000, 1e-9);
  close(left.position.z, -31 / 1000, 1e-9);
  close(top.position.y, 56.5 / 1000, 1e-9);
});

test('M3 screws: the BOM count, each the head diameter over the head-plus-shank length', () => {
  // every screw line (v24: the M3 x 8s and the boom clamps' M3 x 12s), each screw along its own axis (v24: the trays'
  // and the riser's along y)
  for (const item of design.bom.structure.filter((s) => /^(M3 screws|ISO 4762 M3)/.test(s.name))) {
    const n = +item.name.match(/,\s*(\d+)\s*off/)[1];
    const [, headD, headH] = item.basis.match(/head\s*([\d.]+)\s*mm x\s*([\d.]+)\s*mm/).map(Number);
    const [, , shankL] = item.basis.match(/([\d.]+)\s*mm x\s*([\d.]+)\s*mm shank/).map(Number);
    const objs = part(item.name).objects;
    assert.equal(objs.length, n, item.name);
    for (const o of objs) sortedDims(o).forEach((v, i) => close(v, [headD, headD, headH + shankL].map((x) => x / 1000).sort((a, b) => a - b)[i], 1e-5));
  }
});

test('M3 nuts: the BOM count, each a hexagon across its flats and its corners, with its bore', () => {
  // v24: the physics' nut is the ring round its corners; the film draws the hexagon in it
  const item = design.bom.structure.find((s) => /^M3 nuts/.test(s.name));
  const n = +item.name.match(/,\s*(\d+)\s*off/)[1];
  const [, flats, high] = item.basis.match(/([\d.]+)\s*mm across flats x\s*([\d.]+)\s*mm high/).map(Number);
  const [, bore] = item.basis.match(/([\d.]+)\s*mm minor-diameter bore/).map(Number);
  const objs = part(item.name).objects;
  assert.equal(objs.length, n);
  const corners = flats / Math.cos(Math.PI / 6);
  let hex = 0;
  for (const o of objs) {
    const sh = ph.solid(o.userData.physics[0].split('/')[0], 'nut').shape;
    if (sh.kind === 'ring') {
      // on an axis along x: the hexagon, and its bore against the BOM's
      sortedDims(o).forEach((v, i) => close(v, [high, flats, corners].map((x) => x / 1000)[i], 1e-5));
      agree('M3 nut bore', bore, 2000 * minRadius(o), 1e-3);
      hex++;
    } else sortedDims(o).forEach((v, i) => close(v, [high, corners, corners].map((x) => x / 1000)[i], 1e-5)); // across x: round in the physics' square
  }
  assert.ok(hex > n / 2, `${hex} of ${n} drawn hexagonal`);
});

test('cable ties: every BOM tie line drawn to its count, each round its part and carrier where the design states it', () => {
  // v24: every tie is the physics' (the packed parts', the formers', the tail's and the pitot's), one mesh a tie
  const S = design.structure.airframe.parts.support;
  for (const item of design.bom.structure.filter((x) => /^cable ties/.test(x.name))) {
    const n = +item.name.match(/,\s*(\d+)\s*off/)[1];
    const objs = part(item.name).objects;
    assert.equal(objs.length, n, item.name);
    for (const t of (S?.ties ?? []).filter((x) => x.tie === item.material_id)) {
      const b = model.layout.parts.find((p) => p.name === t.part).box;
      const loops = objs.filter((o) => o.userData.physics.every((g) => g.startsWith(`packed ${t.part} ties/`)));
      assert.equal(loops.length, t.n, t.part);
      for (const o of loops) {
        const w = new THREE.Box3().setFromObject(o);
        assert.ok(w.min.x > b.x0 && w.max.x < b.x1, `${t.part} tie along its part`);
        close(w.max.x - w.min.x, t.band_mm[0] / 1000, 1e-7);
        assert.ok(w.max.z - w.min.z >= b.z1 - b.z0 && w.max.y - w.min.y >= b.y1 - b.y0, `${t.part} tie round it`);
      }
    }
  }
});

test('M3 screws and nuts land at their joints: every fixing the airframe states, its screw and its nut on its axis at its place', () => {
  // v23 placed its own fasteners at the joints (the rail lock lands, the bracket holes, two each elsewhere); v24 states
  // every fixing (structure.airframe.parts.fixings: its joint, axis, place, screw and nut) and draws it as solids, so
  // the same promise is checked against that record: each fixing has a drawn screw (and nut) of its BOM line, centred
  // on its axis at its place, and every drawn M3 screw and nut is one of them
  const F = design.structure.airframe.parts.fixings;
  const lineOf = (id) => design.bom.structure.find((s) => s.material_id === id).name;
  const centre = (o) => new THREE.Box3().setFromObject(o).getCenter(new THREE.Vector3());
  const across = (f, c) => (f.axis === 'x' ? [c.z, c.y] : f.axis === 'y' ? [c.x, c.z] : [c.x, c.y]);
  const used = new Set();
  for (const f of F) {
    for (const [id, what] of [[f.screw, 'screw'], [f.nut, 'nut']]) {
      if (!id) continue;
      const hit = part(lineOf(id)).objects.filter((o) => !used.has(o) && Math.hypot(...across(f, centre(o)).map((v, k) => v - f.c[k])) < 1e-6);
      assert.ok(hit.length >= 1, `${f.joint} ${what} at ${f.c}`);
      used.add(hit[0]);
    }
  }
  for (const id of new Set(F.flatMap((f) => [f.screw, f.nut]).filter(Boolean))) {
    const objs = part(lineOf(id)).objects;
    assert.equal(objs.filter((o) => used.has(o)).length, objs.length, `${lineOf(id)}: every one at a fixing`);
    assert.equal(objs.length, +lineOf(id).match(/(\d+) off/)[1], lineOf(id));
  }
  assert.equal(design.structure.clearance.holes.passed, true, 'and every hole lies in metal (the physics\' hole record)');
});

test('every frame BOM line is registered with its own material_id', () => {
  const ids = new Set(bird.parts.map((p) => p.id).filter(Boolean));
  for (const s of design.bom.structure) if (FRAME_NAME_RE.test(s.name) || FRAME_MATERIAL_IDS.includes(s.material_id)) assert.ok(ids.has(s.material_id), s.name);
});

test('buildFrame records its placement assumptions on model.layout.assumptions', () => {
  assert.ok(model.layout.assumptions.some((a) => /shoulder rail/.test(a)));
  assert.ok(model.layout.assumptions.some((a) => /M3 screw/.test(a)));
  assert.ok(model.layout.assumptions.some((a) => /cable tie/.test(a)));
});

test('M3 screws: the design gives the rail\'s lock-screw count, so no "assumed" note is recorded for it', () => {
  assert.ok(!model.layout.assumptions.some((a) => /assumed \d+ shoulder rail lock screws/.test(a)));
});

test('M3 screws: without a design-given rail lock-screw count, falls back to 2 per side and says so', () => {
  // the film places its own screws only for a design the physics does not lay out (v24 states every fixing)
  const screwName = design.bom.structure.find((s) => /^M3 screws/.test(s.name)).name;
  const n = +screwName.match(/,\s*(\d+)\s*off/)[1];
  const { m: m2, b: b2 } = withoutLayout((d2) => delete d2.params.linkage.airframe.rail.lock_screws);
  assert.ok(m2.layout.assumptions.some((a) => /assumed 2 shoulder rail lock screws/.test(a)));
  const screws = b2.parts.find((p) => p.name === screwName).objects;
  assert.equal(screws.length, n);
});

test('with no design.params.linkage.airframe at all (no physics layout), panels and screws still fall back to their defaults', () => {
  const { m: m3, b: b3 } = withoutLayout((d3) => delete d3.params.linkage.airframe);
  const right = b3.parts.find((p) => p.name === 'gearbox panels, 3 off').objects.find((o) => o.name === 'frame_panel_side_panel_right');
  close(right.position.z, 31 / 1000, 1e-9);
  assert.ok(m3.layout.assumptions.some((a) => /assumed 2 shoulder rail lock screws/.test(a)));
});

test('with no rails in design.structure.airframe.parts, no rail, boss or bracket is drawn, and screws drop by the rails\' lock-screw share', () => {
  const d4 = structuredClone(design);
  delete d4.structure.airframe.parts.rails;
  const m4 = prepare(d4, model.parts);
  const b4 = buildBird(m4, makeMaterials({ textures: false }));
  assert.ok(!b4.parts.some((p) => /^shoulder (rail|bracket)/.test(p.name)));
  // the screw count: the film's own share-out, for a design the physics does not lay out (v24 states every fixing)
  const { b: b5 } = withoutLayout((d5) => delete d5.structure.airframe.parts.rails);
  const nScrew = +design.bom.structure.find((s) => /^M3 screws/.test(s.name)).name.match(/,\s*(\d+)\s*off/)[1];
  const lock = design.params.linkage.airframe.rail.lock_screws;
  assert.equal(b5.parts.find((p) => /^M3 screws/.test(p.name)).objects.length, nScrew - 2 * lock);
});

test('with no trays in design.structure.airframe.parts (no physics layout: a v23 design needs its carriers), neither tray nor its doubler is drawn, and nothing else breaks', () => {
  const { b: b5 } = withoutLayout((d5) => { delete d5.structure.airframe.parts.trays; delete d5.structure.airframe.parts.carriers; });
  assert.ok(!b5.parts.some((p) => /^(forward|aft) tray/.test(p.name)));
  assert.ok(!b5.parts.some((p) => /^forward tray flange doubler/.test(p.name)));
});

test('no NaN in any frame part\'s geometry or world matrix after a pose update', () => {
  bird.update({ phase: 1.1, amp: model.law.ampNominal, twistL: 0.2, twistR: -0.2 });
  bird.root.updateMatrixWorld(true);
  let checked = 0;
  for (const p of bird.parts) {
    if (!(FRAME_NAME_RE.test(p.name) || FRAME_MATERIAL_IDS.includes(p.id))) continue;
    for (const o of p.objects) {
      for (const v of o.geometry.attributes.position.array) assert.ok(Number.isFinite(v), `${p.name} geometry`);
      for (const v of o.matrixWorld.elements) assert.ok(Number.isFinite(v), `${p.name} matrix`);
      checked++;
    }
  }
  assert.ok(checked > 40, `only checked ${checked} frame objects`);
});

test('a design with no structure.airframe draws no frame part, and does not throw', () => {
  const { bird: b2, design: d2 } = nodeBird('falco-pf-2');
  assert.equal(d2.structure?.airframe, undefined);
  assert.ok(!b2.parts.some((p) => FRAME_NAME_RE.test(p.name) || FRAME_MATERIAL_IDS.includes(p.id)));
});

// ---- unit tests for the basis parsers and small helpers, so their branches are pinned directly rather than only
// through the one real design that happens to have every field they can read -------------------------------------

test('offCount: parses "N off" from a BOM item name, or 0 without one', () => {
  assert.equal(offCount({ name: 'M3 screws, 16 off' }), 16);
  assert.equal(offCount({ name: 'forward tray' }), 0);
  assert.equal(offCount(null), 0);
});

test('parseTube, parseRing, parseBlocks, parsePlate: pull their numbers from basis text, or null without a match', () => {
  assert.deepEqual(parseTube('12/8 mm tube x 96.0 mm plus a 2 mm lock flange'), { od: 12, id: 8, len: 96 });
  assert.equal(parseTube('nothing here'), null);
  assert.deepEqual(parseRing('ring 29/12 x 3 mm on the aft plate'), { od: 29, id: 12, t: 3 });
  assert.equal(parseRing('no ring here'), null);
  assert.deepEqual(parseBlocks('13 x 19 mm blocks x 6 mm, bored 9.982 mm'), { a: 13, b: 19, len: 6 });
  assert.equal(parseBlocks('no blocks here'), null);
  assert.deepEqual(parsePlate('22 mm x 2 mm 6061-T6 under the flange screws'), { w: 22, t: 2 });
  assert.equal(parsePlate('no plate here'), null);
});

test('parseBracket: needs both the sleeves text and the arms text, or returns null', () => {
  const full = 'two sleeves 16/12 x 13 mm and two arms 10 x 8 x 12.2 mm to the housings';
  assert.deepEqual(parseBracket(full), { sOd: 16, sId: 12, sLen: 13, aB: 10, aH: 8, aLen: 12.2 });
  assert.equal(parseBracket('two sleeves 16/12 x 13 mm only'), null);
  assert.equal(parseBracket('two arms 10 x 8 x 12.2 mm only'), null);
  assert.equal(parseBracket('neither'), null);
});

test('parseChannel: the channel width, flange height, sheet thickness and overall length', () => {
  const basis = 'channel 22 x 7 x 0.8 mm sheet (28.8 mm2) x 184 mm (to 174 mm from the plate, with a 10 mm root flange)';
  assert.deepEqual(parseChannel(basis), { w: 22, flangeH: 7, t: 0.8, len: 184 });
  assert.equal(parseChannel('no channel here'), null);
});

test('parsePanel: needs both the named panel\'s own line and the shared "each ... flanges" line', () => {
  const basis = 'side panel right 21.5 x 1.2 mm, top panel 20 x 1 mm, each 68.0 mm plus two 24 mm flanges';
  assert.deepEqual(parsePanel(basis, 'side panel right'), { a: 21.5, t: 1.2, gap: 68, flange: 24 });
  assert.equal(parsePanel(basis, 'side panel left'), null); // not named in this basis
  assert.equal(parsePanel('side panel right 21.5 x 1.2 mm', 'side panel right'), null); // no shared flanges line
});

test('splitEven: splits a total into n whole shares that always sum back to it, clamped at 0', () => {
  assert.deepEqual(splitEven(10, 5), [2, 2, 2, 2, 2]);
  assert.deepEqual(splitEven(7, 3), [2, 2, 3]);
  assert.deepEqual(splitEven(0, 4), [0, 0, 0, 0]);
  assert.deepEqual(splitEven(-5, 3), [0, 0, 0]);
  for (const [t, n] of [[11, 4], [1, 6], [23, 7]]) assert.equal(splitEven(t, n).reduce((a, b) => a + b, 0), t);
});

test('scatter: n points on a circle of radius r around a centre, in the y/z plane; nothing without a centre', () => {
  const arr = [];
  scatter(null, 3, 0.01, arr);
  assert.equal(arr.length, 0);
  scatter([1, 2, 3], 1, 5, arr);
  assert.deepEqual(arr, [[1, 7, 3]]); // k=0: angle 0, cos=1, sin=0
  const arr4 = [];
  scatter([0, 0, 0], 4, 10, arr4);
  assert.equal(arr4.length, 4);
  for (const p of arr4) assert.ok(Math.abs(Math.hypot(p[1], p[2]) - 10) < 1e-9);
  // k=1 of 4: angle pi/2, cos 0 sin 1 - pins the sign of both the cos and the sin term
  assert.ok(Math.abs(arr4[1][1] - 0) < 1e-9 && Math.abs(arr4[1][2] - 10) < 1e-9, arr4[1]);
});

test('screwGeo and nutGeo: null without both halves of their basis text; the right numbers otherwise reach the geometry builder', () => {
  const fakeCtx = { geo: { cylX: (r, len) => ({ r, len }), ringX: (ro, ri, w) => ({ ro, ri, w }) } };
  const screwItem = { basis: 'ISO 4762 M3 maximum head 5.5 mm x 3.0 mm plus a 3 mm x 8 mm shank at nominal diameter' };
  assert.deepEqual(screwGeo(fakeCtx, screwItem), { r: 5.5 / 2000, len: (3 + 8) / 1000 });
  assert.equal(screwGeo(fakeCtx, { basis: 'head 5.5 mm x 3.0 mm only' }), null);
  assert.equal(screwGeo(fakeCtx, { basis: '3 mm x 8 mm shank only' }), null);
  const nutItem = { basis: 'ISO 4032 M3 maximum 5.5 mm across flats x 2.4 mm high less a 2.5 mm minor-diameter bore' };
  assert.deepEqual(nutGeo(fakeCtx, nutItem), { ro: 5.5 / 2000, ri: 2.5 / 2000, w: 2.4 / 1000 });
  assert.equal(nutGeo(fakeCtx, { basis: '5.5 mm across flats x 2.4 mm high only' }), null);
  assert.equal(nutGeo(fakeCtx, { basis: '2.5 mm minor-diameter bore only' }), null);
});

test('tieGeo: a box at the catalogue\'s own dimensions; null without them', () => {
  const fakeCtx = { spec: (id) => (id === 'has' ? { dimensions: '100 x 2.5 x 1.0 mm, bundle 1.5-22 mm' } : {}) };
  const g = tieGeo(fakeCtx, { material_id: 'has' });
  assert.equal(+g.parameters.width.toFixed(4), 0.1);
  assert.equal(+g.parameters.height.toFixed(4), 0.0025);
  assert.equal(+g.parameters.depth.toFixed(4), 0.001);
  assert.equal(tieGeo(fakeCtx, { material_id: 'missing' }), null);
});

test('motor doubler (film-placed, no physics solid): the BOM\'s square and thickness on the forward plate\'s outer face, round the motor\'s shaft, bored clear of its outboard bearing', () => {
  const item = design.bom.structure.find((s) => s.name === 'motor doubler');
  if (!item) return assert.equal(bird.root.getObjectByName('frame_motor_doubler'), undefined, 'none without its BOM line');
  const d = parsePlate(item.basis), o = part(item.name).objects[0];
  const can = sol('motor', 'can'), ob = bird.phys.parts.find((p) => /^motor shaft outboard journal/.test(p.id)).solids[0];
  const PL = design.params.linkage.plates, face = model.planes.plateFwd + (PL.t_fwd_mm ?? PL.t_mm) / 2000;
  bird.root.updateMatrixWorld(true);
  const b = new THREE.Box3().setFromObject(o, true);
  close(b.min.x, face, 1e-7);
  close(b.max.x - b.min.x, d.t / 1000, 1e-7);
  close(b.max.z - b.min.z, d.w / 1000, 1e-7);
  close(b.max.y - b.min.y, d.w / 1000, 1e-7);
  close((b.max.y + b.min.y) / 2, can.shape.c[1], 1e-7);
  close((b.max.z + b.min.z) / 2, can.shape.c[0], 1e-7);
  // its bore: the nearest vertex to the shaft axis is the bearing's outside radius and the running clearance
  const pos = o.geometry.attributes.position, v = new THREE.Vector3();
  let r = Infinity;
  for (let i = 0; i < pos.count; i++) { v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld); r = Math.min(r, Math.hypot(v.z - can.shape.c[0], v.y - can.shape.c[1])); }
  close(r, ob.shape.ro + 0.0005, 1e-7);
  assert.equal(o.userData.joint, 'motor');
});

// last, once every test above has compared its numbers
test('every reported BOM / physics disagreement was found', () => {
  assert.deepEqual(Object.keys(found).sort(), Object.keys(REPORTED).sort());
});
