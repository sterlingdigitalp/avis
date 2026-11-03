// Run: node --import ./film/lib/three-hooks.mjs --test film/test/
// The three.js bird: every BOM line is drawn, counted parts appear as many times as the BOM says, the gears and
// bearings match the BOM counts, and the pose leaves no NaN anywhere.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird, posedNodes } from '../lib/bird-node.mjs';
import { stationsAt, sineSeries, membraneDip } from '../../sims/avis-shared/model.js';

const { bird, design, model } = nodeBird('larus-gl-2');

test('every BOM part and structure item is drawn', () => {
  const ids = new Set(bird.parts.map((p) => p.id).filter(Boolean));
  for (const p of design.bom.parts) assert.ok(ids.has(p.id), `${p.role} ${p.id}`);
  for (const s of design.bom.structure) assert.ok(ids.has(s.material_id), `${s.name} ${s.material_id}`);
});

test('gears, bearings, bushings and servos appear exactly as often as the BOM lists them', () => {
  const count = (id) => {
    let n = 0;
    bird.root.traverse((o) => {
      if (o.isMesh && o.userData.partId === id && /teeth|bearing|bushing|_case/.test(o.name)) n++;
    });
    return n;
  };
  const bom = (role) => design.bom.parts.filter((p) => p.role === role).reduce((a, p) => a + p.count, 0);
  const gearIds = design.bom.parts.filter((p) => p.role === 'gear');
  let gearsDrawn = 0;
  for (const id of new Set(gearIds.map((g) => g.id))) gearsDrawn += count(id);
  assert.equal(gearsDrawn, bom('gear'));
  // every bearing and bushing line, whether one pooled "bearing" or one per joint ("bushing: crank shaft (2)")
  const journals = design.bom.parts.filter((p) => /^(bearing|bushing)\b/.test(p.role));
  assert.ok(journals.length);
  for (const id of new Set(journals.map((p) => p.id))) {
    assert.equal(count(id), journals.filter((p) => p.id === id).reduce((a, p) => a + p.count, 0), id);
  }
  const servoId = design.bom.parts.find((p) => /servo$/.test(p.role)).id;
  const servos = design.bom.parts.filter((p) => p.id === servoId).reduce((a, p) => a + p.count, 0);
  assert.equal(count(servoId), servos);
});

test('ribs: the BOM count, shared between the wings', () => {
  let n = 0;
  bird.root.traverse((o) => {
    if (/^rib_[LR]_\d+$/.test(o.name)) n++;
  });
  const listed = +design.bom.structure.find((s) => /^ribs/.test(s.name)).name.match(/(\d+)\s*off/)[1];
  assert.equal(n, listed);
});

test('a pose from the log leaves every matrix and vertex finite', () => {
  bird.update({ phase: 2.2, amp: model.law.ampMax, twistL: 0.3, twistR: -0.3, tailL: 0.2, tailR: -0.1 });
  bird.root.updateMatrixWorld(true);
  bird.root.traverse((o) => {
    for (const v of o.matrixWorld.elements) assert.ok(Number.isFinite(v), o.name);
  });
  for (const m of bird.deform) for (const v of m.geometry.attributes.position.array) assert.ok(Number.isFinite(v), m.name);
});

test('wing segment nodes are the GLB contract: wing_R_0..1 and wing_L_0..1, chained, the only wing_ nodes', () => {
  const names = [];
  bird.root.traverse((o) => names.push(o.name));
  assert.deepEqual(names.filter((n) => /^wing_/.test(n)).sort(), ['wing_L_0', 'wing_L_1', 'wing_R_0', 'wing_R_1']);
  assert.equal(bird.root.getObjectByName('wing_R_1').parent.name, 'wing_R_0');
  assert.equal(bird.root.name, 'body');
  assert.ok(posedNodes(bird).length >= 4);
});

test('the right wing tip rises with the stroke angle and the left mirrors it', () => {
  const tipY = (phase) => {
    const dp = bird.update({ phase, amp: model.law.ampNominal });
    bird.root.updateMatrixWorld(true);
    const r = bird.root.getObjectByName('rib_R_08').getWorldPosition(new bird.root.position.constructor());
    const l = bird.root.getObjectByName('rib_L_08').getWorldPosition(new bird.root.position.constructor());
    return { r, l, dp };
  };
  const top = tipY(0), bottom = tipY(Math.PI);
  assert.ok(top.r.y > bottom.r.y + 0.2);
  assert.ok(top.l.y > bottom.l.y + 0.2);
  assert.ok(top.r.z > 0 && top.l.z < 0);
});

test('wing film: on the rib profile at every rib, a shallow dip between them, faces never cross', () => {
  bird.update({ phase: 0.7, amp: model.law.ampNominal ?? design.cruise.amp, twistL: 0.2, twistR: -0.2 });
  const f = bird.root.getObjectByName('film_R_0');
  const pos = f.geometry.attributes.position.array;
  const rows = f.userData.rows;
  const ring = pos.length / 3 / rows.length;
  const nSec = (ring + 1) / 2;
  const P = (r, c) => [pos[3 * (r * ring + c)], pos[3 * (r * ring + c) + 1], pos[3 * (r * ring + c) + 2]];
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  // thickness at 30% chord, upper face point vs lower face point of the same row
  const k30 = model.section.reduce((best, q, j) => (Math.abs(q.xi - 0.3) < Math.abs(model.section[best].xi - 0.3) ? j : best), 0);
  const thick = rows.map((_, r) => dist(P(r, nSec - 1 - k30), P(r, nSec - 1 + k30)));
  // rows run rib, then three between, then the next rib: the mid-bay row is thinner than both ribs, but not by much
  let bays = 0;
  for (let r = 0; r + 4 <= rows.length - 1; r += 4) {
    const mid = thick[r + 2], ribA = thick[r], ribB = thick[r + 4];
    assert.ok(mid < Math.min(ribA, ribB), `bay ${r / 4}: mid ${mid} ribs ${ribA} ${ribB}`);
    assert.ok(mid > 0.8 * Math.min(ribA, ribB), `bay ${r / 4}: film sags too far (${mid} vs ${ribA})`);
    bays++;
  }
  assert.ok(bays >= 3);
  for (let r = 0; r < rows.length; r++) for (let j = 1; j < nSec - 1; j++) {
    const up = P(r, nSec - 1 - j), lo = P(r, nSec - 1 + j);
    assert.ok(dist(up, lo) > 0.0012, `row ${r} chord point ${j}: faces ${dist(up, lo)} m apart`);
  }
});

test('wing film between ribs: model.membraneDip at the bay midpoint, never inside a spar the ribs clear, faces a rod apart', () => {
  bird.update({ phase: 1.9, amp: model.law.ampNominal ?? design.cruise.amp, twistL: -0.3, twistR: 0.3 });
  const sec = model.section, nSec = sec.length, W = model.wing, ribs = model.ribs;
  const xs = sec.map((q) => q.xi);
  const bUp = sineSeries(xs, sec.map((q) => q.cam + q.half));
  const bLo = sineSeries(xs, sec.map((q) => q.cam - q.half));
  const teR = model._teR;
  const sparR = [/inner spar/, /outer spar/].map((re) => model.parts[design.bom.structure.find((x) => re.test(x.name)).material_id].specs.od_mm / 2000);
  const fsList = ribs.flatMap((s, k) => (k < ribs.length - 1 ? [0, 1, 2, 3].map((r) => s + ((ribs[k + 1] - s) * r) / 4) : [s]));
  let checked = 0;
  for (const tag of ['R', 'L']) for (const k of [0, 1]) {
    const f = bird.root.getObjectByName(`film_${tag}_${k}`);
    const pos = f.geometry.attributes.position.array;
    const P = (r, c) => [pos[3 * (r * (2 * nSec - 1) + c)], pos[3 * (r * (2 * nSec - 1) + c) + 1], pos[3 * (r * (2 * nSec - 1) + c) + 2]];
    const fromSpar = (v) => Math.hypot(v[0] - W.spar_x, v[1]); // the segment frame: the spar runs along z at (spar_x, 0)
    f.userData.rows.forEach((i, r) => {
      const s = fsList[i];
      const b = Math.max(0, ribs.findIndex((x) => x > s + 1e-9) - 1);
      const gap = ribs[b + 1] - ribs[b], u = (s - ribs[b]) / gap;
      const st = stationsAt(W, [s]);
      const c = st.chord[0], rad = sparR[st.seg[0]];
      for (let j = 1; j < nSec - 1; j++) {
        const up = P(r, nSec - 1 - j), lo = P(r, nSec - 1 + j);
        const apart = Math.hypot(up[0] - lo[0], up[1] - lo[1], up[2] - lo[2]);
        assert.ok(apart >= 2 * teR - 1e-7, `${tag}${k} row ${r} point ${j}: faces ${apart} m apart`);
        if (Math.abs(u - 0.5) > 1e-6) continue;
        // at the bay midpoint: never inside a spar where the rib rows on either side are outside it
        const ribA = P(r - 2, nSec - 1 - j), ribB = P(r + 2, nSec - 1 - j);
        if (fromSpar(ribA) >= rad && fromSpar(ribB) >= rad) assert.ok(fromSpar(up) >= rad - 1e-7, `${tag}${k} row ${r} point ${j} inside the spar`);
        const xSpar = (st.xle[0] - W.spar_x) / c;
        const half = Math.max(sec[j].half, teR / c);
        const dUp = membraneDip(bUp, sec[j].xi, 0.5, gap / c), dLo = membraneDip(bLo, sec[j].xi, 0.5, gap / c);
        if (Math.abs(sec[j].xi - xSpar) < rad / c + 0.02 || dUp - dLo > 2 * (half - teR / c)) continue; // spar or rod holds it
        assert.ok(Math.abs(apart - c * (2 * half - dUp + dLo)) < 1e-6, `${tag}${k} row ${r} point ${j}: ${apart} vs ${c * (2 * half - dUp + dLo)}`);
        checked++;
      }
    });
  }
  assert.ok(checked > 100, `${checked} mid-bay points checked`);
});

test('a pooled "bearing" line (older designs) is drawn two per shoulder hinge, two per gearbox shaft, the rest at the elbows', async () => {
  const { prepare } = await import('../../sims/avis-shared/model.js');
  const { makeMaterials } = await import('../../sims/avis-shared/materials.js');
  const { buildBird } = await import('../../sims/avis-shared/bird.js');
  const shafts = model.train.shafts.length - 1; // the crank's and each gear shaft, not the motor's
  const id = design.bom.parts.find((p) => /^bearing/.test(p.role))?.id ?? design.bom.parts.find((p) => /^bushing/.test(p.role)).id;
  for (const N of [4 + 2 * shafts + 4, 4 + 2 * shafts - 1]) {
    const parts = [...design.bom.parts.filter((p) => !/^(bearing|bushing)\b/.test(p.role)), { role: 'bearing', id, count: N }];
    const m = prepare({ ...design, bom: { ...design.bom, parts } }, model.parts);
    const b = buildBird(m, makeMaterials({ textures: false }));
    const n = (re) => b.meshes.filter((o) => re.test(o.name)).length;
    assert.equal(n(/^hinge_bearing_/), 4);
    assert.equal(n(/^gearbox_bearing_/), Math.min(2 * shafts, N - 4));
    assert.equal(n(/^elbow_bearing_/), N - 4 - 2 * shafts >= 4 ? 4 : 0);
    assert.equal(b.meshes.filter((o) => o.userData.partId === id && /bearing/.test(o.name)).length, Math.min(N, 4 + 2 * shafts + 4));
  }
});

test('journals are drawn at catalogue size on their shafts; the encoder rings weigh what the catalogue says', async () => {
  const { journalDims } = await import('../../sims/avis-shared/model.js');
  bird.update({ phase: 0.3, amp: model.law.ampNominal ?? design.cruise.amp });
  bird.root.updateMatrixWorld(true);
  const size = (o) => new THREE.Box3().setFromBufferAttribute(o.geometry.attributes.position).getSize(new THREE.Vector3());
  const P = model.planes, shafts = model.train.shafts;
  let n = 0;
  for (const o of bird.meshes.filter((m) => /^(gearbox|hinge|coupler|elbow)_(bearing|bushing)_/.test(m.name))) {
    const d = journalDims(model.parts[o.userData.partId].specs);
    const s = size(o);
    assert.ok(Math.abs(s.x - d.B) < 1e-6 && Math.abs(s.y - Math.max(d.D, d.flangeD)) < 2e-4, `${o.name}: ${s.x} x ${s.y}`);
    const g = o.name.match(/^gearbox_\w+_(\d+)_(aft|fwd)$/);
    if (g) {
      // flush with the plate's outer face, reaching into the gearbox (the plate's thickness from the design when it
      // gives one), so a journal longer than its plate never pokes out of the aft plate into the crank web
      const PL = design.params?.linkage?.plates;
      const t = g[2] === 'aft' ? (PL ? PL.t_mm / 1000 : P.plateT) : (PL ? (PL.t_fwd_mm ?? PL.t_mm) / 1000 : P.plateT);
      const x = g[2] === 'aft' ? P.plateAft - t / 2 + d.B / 2 : P.plateFwd + t / 2 - d.B / 2;
      assert.ok(Math.abs(o.position.x - x) < 1e-12, `${o.name} x`);
      assert.deepEqual([o.position.y, o.position.z], [shafts[+g[1]][1], shafts[+g[1]][0]]);
      n++;
    }
  }
  assert.ok(n >= 2);
  const ring = (name, id, rho) => {
    const o = bird.root.getObjectByName(name);
    if (!o) return;
    const s = size(o);
    const pos = o.geometry.attributes.position.array;
    let ri = Infinity;
    for (let i = 0; i < pos.length; i += 3) ri = Math.min(ri, Math.hypot(pos[i + 1], pos[i + 2]));
    const vol = Math.PI * ((s.y / 2) ** 2 - ri ** 2) * s.x;
    assert.ok(Math.abs(vol * rho * 1000 - model.parts[id].mass_g) < 0.05 * model.parts[id].mass_g, `${name}: ${vol * rho * 1000} g`);
    assert.ok(Math.abs(o.position.y - shafts[0][1]) < 1e-9 && Math.abs(o.position.z - shafts[0][0]) < 1e-9, `${name} on the crank shaft`);
    return ri;
  };
  const enc = design.bom.parts.find((p) => /encoder/.test(p.role) && !/magnet/.test(p.role));
  const mag = design.bom.parts.find((p) => /encoder magnet/.test(p.role));
  if (mag) {
    const ri = ring('encoder_magnet', mag.id, 7500);
    assert.ok(Math.abs(2 * ri - (model.parts[mag.id].specs.inner_diameter_mm ?? 8) / 1000) < 2e-4);
  }
  if (enc) ring('encoder_readhead', enc.id, 1850);
  if (enc && mag) {
    // the readhead clears the magnet, and both sit between the gearbox plates on the crank shaft
    const rh = bird.root.getObjectByName('encoder_readhead'), mg = bird.root.getObjectByName('encoder_magnet');
    const p = rh.geometry.attributes.position.array;
    let inner = Infinity;
    for (let i = 0; i < p.length; i += 3) inner = Math.min(inner, Math.hypot(p[i + 1], p[i + 2]));
    assert.ok(inner > size(mg).y / 2, 'readhead bore clears the magnet');
    for (const o of [rh, mg]) assert.ok(o.position.x - size(o).x / 2 > P.plateAft && o.position.x + size(o).x / 2 < P.plateFwd, `${o.name} between the plates`);
  }
  if (enc || mag) assert.ok(model.layout.assumptions.some((a) => /encoder/.test(a)));
});

test('rocker track: a band either side of the slot, or a round hole where a fixed stroke leaves no slot', async () => {
  const { offsetBand, slotPath } = await import('../../sims/avis-shared/bird.js');
  assert.deepEqual(offsetBand([[0, 0], [2, 0]], 0.5), [[[0, 0.5], [0, -0.5]], [[2, 0.5], [2, -0.5]]]);
  const diag = offsetBand([[0, 0], [1, 1], [2, 2]], Math.SQRT2);
  assert.ok(diag.every(([l, r], i) => Math.abs(l[0] - (i - 1)) < 1e-12 && Math.abs(l[1] - (i + 1)) < 1e-12 && Math.abs(r[0] - (i + 1)) < 1e-12));
  for (const side of [1, -1]) {
    const hole = slotPath([[0.02, 0.01], [0.02, 0.01]], side, 0.003).getPoints(24);
    assert.ok(hole.every((p) => Math.abs(Math.hypot(p.x + side * 0.02, p.y - 0.01) - 0.003) < 1e-9));
    const band = slotPath([[0, 0], [0.01, 0]], side, 0.001).getPoints(1);
    assert.ok(band.some((p) => Math.abs(p.y - 0.001) < 1e-9) && band.some((p) => Math.abs(p.y + 0.001) < 1e-9));
    assert.ok(band.every((p) => Math.abs(p.x) <= 0.01 + 1e-9 && p.x * side <= 1e-12), 'mirrored for the left wing');
  }
});
