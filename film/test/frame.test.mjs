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

test('shoulder rails: OD, ID and length from the basis, at the design\'s rail position', () => {
  for (const side of ['right', 'left']) {
    const item = design.bom.structure.find((s) => s.name === `shoulder rail ${side}`);
    const [, od, id, len] = item.basis.match(/([\d.]+)\/([\d.]+)\s*mm tube x\s*([\d.]+)\s*mm/).map(Number);
    const o = part(item.name).objects[0];
    close(size(o).x, len / 1000);
    close(size(o).y, od / 1000);
    close(size(o).z, od / 1000);
    close(2 * minRadius(o), id / 1000);
    const pos = design.structure.airframe.parts.rails[side];
    close(o.position.y, pos[1], 1e-9);
    close(o.position.z, pos[0], 1e-9);
    close(o.position.x, planeMid, 1e-9);
  }
});

test('shoulder rail aft seat bosses: ring OD, ID and thickness from the basis, on the aft plate', () => {
  for (const side of ['right', 'left']) {
    const item = design.bom.structure.find((s) => s.name === `shoulder rail ${side} aft seat boss`);
    const [, od, id, t] = item.basis.match(/ring\s*([\d.]+)\/([\d.]+)\s*x\s*([\d.]+)\s*mm/).map(Number);
    const o = part(item.name).objects[0];
    close(size(o).x, t / 1000);
    close(size(o).y, od / 1000);
    close(size(o).z, od / 1000);
    close(2 * minRadius(o), id / 1000);
    close(o.position.x, model.planes.plateAft, 1e-9);
    const pos = design.structure.airframe.parts.rails[side];
    close(o.position.y, pos[1], 1e-9);
    close(o.position.z, pos[0], 1e-9);
  }
});

test('shoulder brackets: two sleeves and two arms, each sized from the basis', () => {
  const railItem = {};
  for (const side of ['right', 'left']) railItem[side] = design.bom.structure.find((s) => s.name === `shoulder rail ${side}`);
  for (const side of ['right', 'left']) {
    const item = design.bom.structure.find((s) => s.name === `shoulder bracket ${side}`);
    const [, sOd, sId, sLen] = item.basis.match(/two sleeves\s*([\d.]+)\/([\d.]+)\s*x\s*([\d.]+)\s*mm/).map(Number);
    const [, aB, aH, aLen] = item.basis.match(/two arms\s*([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/).map(Number);
    const railLen = parseTube(railItem[side].basis).len / 1000;
    const objs = part(item.name).objects;
    assert.equal(objs.length, 4);
    const sleeves = objs.filter((o) => /sleeve/.test(o.name));
    const arms = objs.filter((o) => /arm/.test(o.name));
    assert.equal(sleeves.length, 2);
    assert.equal(arms.length, 2);
    const railPos = design.structure.airframe.parts.rails[side];
    const railXs = [planeMid - railLen / 2, planeMid + railLen / 2].sort((p, q) => p - q);
    const sleeveXs = sleeves.map((o) => o.position.x).sort((p, q) => p - q);
    sleeveXs.forEach((x, i) => close(x, railXs[i], 1e-9));
    for (const o of sleeves) {
      close(size(o).x, sLen / 1000); close(size(o).y, sOd / 1000); close(size(o).z, sOd / 1000); close(2 * minRadius(o), sId / 1000);
      close(o.position.y, railPos[1], 1e-9); close(o.position.z, railPos[0], 1e-9);
    }
    // each arm's own centre: from the rail toward the shoulder housing (model.fourBar.O4), at the basis's own length
    const housing = side === 'right' ? model.fourBar.O4 : [-model.fourBar.O4[0], model.fourBar.O4[1]];
    const n = Math.hypot(housing[1] - railPos[1], housing[0] - railPos[0]);
    const expectArm = [railPos[1] + ((housing[1] - railPos[1]) / n) * (aLen / 1000) / 2, railPos[0] + ((housing[0] - railPos[0]) / n) * (aLen / 1000) / 2];
    for (const o of arms) {
      closeDims(o, [aB, aH, aLen]);
      close(o.position.y, expectArm[0]);
      close(o.position.z, expectArm[1]);
    }
    const armXs = arms.map((o) => o.position.x).sort((p, q) => p - q);
    armXs.forEach((x, i) => close(x, railXs[i], 1e-9));
  }
});

test('shoulder bracket arms: their actual world rotation carries the local "up" axis from the rail toward the housing', () => {
  for (const side of ['right', 'left']) {
    const item = design.bom.structure.find((s) => s.name === `shoulder bracket ${side}`);
    const aLen = toM(parseBracket(item.basis).aLen);
    const railPos = design.structure.airframe.parts.rails[side];
    const housing = side === 'right' ? model.fourBar.O4 : [-model.fourBar.O4[0], model.fourBar.O4[1]];
    const n = Math.hypot(housing[1] - railPos[1], housing[0] - railPos[0]);
    const unit = [(housing[1] - railPos[1]) / n, (housing[0] - railPos[0]) / n];
    const arms = part(item.name).objects.filter((o) => /arm/.test(o.name));
    for (const o of arms) {
      const near = new THREE.Vector3(0, -aLen / 2, 0).applyQuaternion(o.quaternion).add(o.position);
      const far = new THREE.Vector3(0, aLen / 2, 0).applyQuaternion(o.quaternion).add(o.position);
      close(near.y, railPos[1]); close(near.z, railPos[0]);
      close(far.y, railPos[1] + unit[0] * aLen); close(far.z, railPos[0] + unit[1] * aLen);
    }
  }
});

test('tail boom clamp blocks: 2 off, cross-section and length from the basis, clamp_spacing_m apart', () => {
  const item = design.bom.structure.find((s) => /^tail boom clamp blocks/.test(s.name));
  const [, a, b, len] = item.basis.match(/([\d.]+)\s*x\s*([\d.]+)\s*mm blocks x\s*([\d.]+)\s*mm/).map(Number);
  const objs = part(item.name).objects;
  assert.equal(objs.length, 2);
  for (const o of objs) {
    closeDims(o, [a, b, len]);
    close(o.position.y, design.structure.airframe.parts.boom_y_m, 1e-9);
    close(o.position.z, 0, 1e-9);
  }
  const xs = objs.map((o) => o.position.x).sort((p, q) => p - q);
  close(xs[1] - xs[0], design.structure.airframe.parts.clamp_spacing_m, 1e-6);
  const expectXs = [model.planes.plateAft - model.planes.plateT / 2 - len / 2000, model.planes.plateFwd + model.planes.plateT / 2 + len / 2000];
  xs.forEach((x, i) => close(x, expectXs[i], 1e-9));
});

test('forward and aft trays: channel width, flange height and overall length from the basis, outward from the plate', () => {
  for (const [tag, outward] of [['forward', 1], ['aft', -1]]) {
    const item = design.bom.structure.find((s) => s.name === `${tag} tray`);
    const m = item.basis.match(/channel\s*([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm sheet.*?x\s*([\d.]+)\s*mm/).map(Number);
    const [, w, flangeH, , len] = m;
    const o = part(item.name).objects[0];
    closeDims(o, [w, flangeH, len]);
    close(o.position.y, design.structure.airframe.parts.trays[tag].y_m, 1e-9);
    close(o.position.z, 0, 1e-9);
    const plateFace = tag === 'forward' ? model.planes.plateFwd + model.planes.plateT / 2 : model.planes.plateAft - model.planes.plateT / 2;
    close(o.position.x, plateFace + (outward * len) / 2000, 1e-9);
  }
});

test('forward tray flange doubler and motor doubler: width and thickness from the basis, as a square plate, on the forward plate', () => {
  const fwdFace = model.planes.plateFwd + model.planes.plateT / 2;
  const motorBox = model.layout.parts.find((p) => p.name === 'motor').box;
  const expect = { 'forward tray flange doubler': [fwdFace, design.structure.airframe.parts.trays.forward.y_m, 0], 'motor doubler': [fwdFace, motorBox.cy, motorBox.cz] };
  for (const name of ['forward tray flange doubler', 'motor doubler']) {
    const item = design.bom.structure.find((s) => s.name === name);
    const [, w, t] = item.basis.match(/([\d.]+)\s*mm x\s*([\d.]+)\s*mm/).map(Number);
    const o = part(name).objects[0];
    closeDims(o, [t, w, w]);
    close(o.position.x, expect[name][0], 1e-9);
    close(o.position.y, expect[name][1], 1e-9);
    close(o.position.z, expect[name][2], 1e-9);
  }
});

test('gearbox panels: 3 off, each panel\'s h/w and thickness from the basis, over the plate gap plus two flanges', () => {
  const item = design.bom.structure.find((s) => /^gearbox panels/.test(s.name));
  const [, gap, flange] = item.basis.match(/each\s*([\d.]+)\s*mm plus two\s*([\d.]+)\s*mm flanges/).map(Number);
  const len = gap + 2 * flange;
  const objs = part(item.name).objects;
  assert.equal(objs.length, 3);
  const list = design.params.linkage.airframe.panels.list;
  for (const nm of ['side panel right', 'side panel left', 'top panel']) {
    const [, a, t] = item.basis.match(new RegExp(`${nm}\\s+([\\d.]+)\\s*x\\s*([\\d.]+)\\s*mm`)).map(Number);
    const o = objs.find((oo) => oo.name === `frame_panel_${nm.replace(/\s+/g, '_')}`);
    assert.ok(o, nm);
    closeDims(o, [a, t, len]);
    close(o.position.x, planeMid, 1e-9);
    const entry = list.find((p) => p.name === nm);
    if (nm === 'top panel') { close(o.position.y, entry.y_mm / 1000, 1e-9); close(o.position.z, 0, 1e-9); }
    else { close(o.position.y, (entry.y0_mm + a / 2) / 1000, 1e-9); close(o.position.z, entry.z_mm / 1000, 1e-9); }
  }
});

test('gearbox panels: without design.params.linkage.airframe.panels.list, fall back to default offsets', () => {
  const d2 = structuredClone(design);
  delete d2.params.linkage.airframe.panels;
  const m2 = prepare(d2, model.parts);
  const b2 = buildBird(m2, makeMaterials({ textures: false }));
  const objs = b2.parts.find((p) => p.name === 'gearbox panels, 3 off').objects;
  const right = objs.find((o) => o.name === 'frame_panel_side_panel_right');
  const left = objs.find((o) => o.name === 'frame_panel_side_panel_left');
  const top = objs.find((o) => o.name === 'frame_panel_top_panel');
  close(right.position.z, 31 / 1000, 1e-9);
  close(left.position.z, -31 / 1000, 1e-9);
  close(top.position.y, 56.5 / 1000, 1e-9);
});

test('M3 screws: the BOM count, each the head diameter over the head-plus-shank length', () => {
  const item = design.bom.structure.find((s) => /^M3 screws/.test(s.name));
  const n = +item.name.match(/,\s*(\d+)\s*off/)[1];
  const [, headD, headH] = item.basis.match(/head\s*([\d.]+)\s*mm x\s*([\d.]+)\s*mm/).map(Number);
  const [, , shankL] = item.basis.match(/([\d.]+)\s*mm x\s*([\d.]+)\s*mm shank/).map(Number);
  const objs = part(item.name).objects;
  assert.equal(objs.length, n);
  for (const o of objs) { close(size(o).x, (headH + shankL) / 1000); close(size(o).y, headD / 1000); close(size(o).z, headD / 1000); }
});

test('M3 nuts: the BOM count, each an across-flats ring with its bore', () => {
  const item = design.bom.structure.find((s) => /^M3 nuts/.test(s.name));
  const n = +item.name.match(/,\s*(\d+)\s*off/)[1];
  const [, flats, high] = item.basis.match(/([\d.]+)\s*mm across flats x\s*([\d.]+)\s*mm high/).map(Number);
  const [, bore] = item.basis.match(/([\d.]+)\s*mm minor-diameter bore/).map(Number);
  const objs = part(item.name).objects;
  assert.equal(objs.length, n);
  for (const o of objs) { close(size(o).x, high / 1000); close(size(o).y, flats / 1000); close(size(o).z, flats / 1000); close(2 * minRadius(o), bore / 1000); }
});

test('cable ties: the BOM count, each the catalogue\'s own strap dimensions', () => {
  const item = design.bom.structure.find((s) => /^cable ties/.test(s.name));
  const n = +item.name.match(/,\s*(\d+)\s*off/)[1];
  const dims = model.parts[item.material_id].specs.dimensions.match(/([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/).slice(1).map(Number);
  const objs = part(item.name).objects;
  assert.equal(objs.length, n);
  for (const o of objs) closeDims(o, dims);
});

test('M3 screws and nuts land at their joints: the rail\'s own lock-screw count per side, 2 each at the boom clamps, the trays and the motor (no nut there)', () => {
  const A = design.structure.airframe.parts;
  const lock = design.params.linkage.airframe.rail.lock_screws;
  const clampLen = +design.bom.structure.find((s) => /^tail boom clamp blocks/.test(s.name)).basis.match(/mm blocks x\s*([\d.]+)\s*mm/)[1];
  const motorBox = model.layout.parts.find((p) => p.name === 'motor').box;
  const joints = {
    'rail right': [model.planes.plateAft, A.rails.right[1], A.rails.right[0]],
    'rail left': [model.planes.plateAft, A.rails.left[1], A.rails.left[0]],
    'boom aft': [model.planes.plateAft - model.planes.plateT / 2 - clampLen / 2000, A.boom_y_m, 0],
    'boom fwd': [model.planes.plateFwd + model.planes.plateT / 2 + clampLen / 2000, A.boom_y_m, 0],
    'tray fwd': [model.planes.plateFwd + model.planes.plateT / 2, A.trays.forward.y_m, 0],
    'tray aft': [model.planes.plateAft - model.planes.plateT / 2, A.trays.aft.y_m, 0],
    motor: [motorBox.cx, motorBox.cy, motorBox.cz],
  };
  const screws = part(design.bom.structure.find((s) => /^M3 screws/.test(s.name)).name).objects;
  const nuts = part(design.bom.structure.find((s) => /^M3 nuts/.test(s.name)).name).objects;
  const near = (objs, c, r = 0.02) => objs.filter((o) => Math.hypot(o.position.x - c[0], o.position.y - c[1], o.position.z - c[2]) < r).length;
  const expectScrew = { 'rail right': lock, 'rail left': lock, 'boom aft': 2, 'boom fwd': 2, 'tray fwd': 2, 'tray aft': 2, motor: 2 };
  const expectNut = { 'rail right': lock, 'rail left': lock, 'boom aft': 2, 'boom fwd': 2, 'tray fwd': 2, 'tray aft': 2, motor: 0 };
  for (const [key, c] of Object.entries(joints)) {
    assert.equal(near(screws, c), expectScrew[key], `${key} screws`);
    assert.equal(near(nuts, c), expectNut[key], `${key} nuts`);
  }
  assert.equal(screws.length, Object.values(expectScrew).reduce((a, b) => a + b, 0));
  assert.equal(nuts.length, Object.values(expectNut).reduce((a, b) => a + b, 0));
});

test('cable ties: clustered at the forward tray, aft tray and motor joints, none left over', () => {
  const A = design.structure.airframe.parts;
  const motorBox = model.layout.parts.find((p) => p.name === 'motor').box;
  const joints = [
    [model.planes.plateFwd + model.planes.plateT / 2, A.trays.forward.y_m, 0],
    [model.planes.plateAft - model.planes.plateT / 2, A.trays.aft.y_m, 0],
    [motorBox.cx, motorBox.cy, motorBox.cz],
  ];
  const ties = part(design.bom.structure.find((s) => /^cable ties/.test(s.name)).name).objects;
  const near = (c, r = 0.02) => ties.filter((o) => Math.hypot(o.position.x - c[0], o.position.y - c[1], o.position.z - c[2]) < r).length;
  const counts = joints.map((c) => near(c));
  assert.equal(counts.reduce((a, b) => a + b, 0), ties.length);
  assert.ok(counts.every((n) => n > 0), counts);
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
  const screwName = design.bom.structure.find((s) => /^M3 screws/.test(s.name)).name;
  const n = +screwName.match(/,\s*(\d+)\s*off/)[1];
  const d2 = structuredClone(design);
  delete d2.params.linkage.airframe.rail.lock_screws;
  const m2 = prepare(d2, model.parts);
  const b2 = buildBird(m2, makeMaterials({ textures: false }));
  assert.ok(m2.layout.assumptions.some((a) => /assumed 2 shoulder rail lock screws/.test(a)));
  const screws = b2.parts.find((p) => p.name === screwName).objects;
  assert.equal(screws.length, n);
});

test('with no design.params.linkage.airframe at all, panels and screws still fall back to their defaults', () => {
  const d3 = structuredClone(design);
  delete d3.params.linkage.airframe;
  const m3 = prepare(d3, model.parts);
  const b3 = buildBird(m3, makeMaterials({ textures: false }));
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
  const nScrew = +design.bom.structure.find((s) => /^M3 screws/.test(s.name)).name.match(/,\s*(\d+)\s*off/)[1];
  const lock = design.params.linkage.airframe.rail.lock_screws;
  assert.equal(b4.parts.find((p) => /^M3 screws/.test(p.name)).objects.length, nScrew - 2 * lock);
});

test('with no trays in design.structure.airframe.parts, neither tray nor its doubler is drawn, and nothing else breaks', () => {
  const d5 = structuredClone(design);
  delete d5.structure.airframe.parts.trays;
  const m5 = prepare(d5, model.parts);
  const b5 = buildBird(m5, makeMaterials({ textures: false }));
  assert.ok(!b5.parts.some((p) => /^(forward|aft) tray/.test(p.name)));
  assert.ok(b5.parts.some((p) => /^motor doubler/.test(p.name)));
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
