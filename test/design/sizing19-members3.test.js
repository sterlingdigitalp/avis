// Round 19 (members3: the drive box's last members at the sealed design's rough-air row, 14 m/s at 3.291 Hz, x3's
// trim; the crank's shaft and pin are not sized here: the larger bushings they need do not fit the box, see members3's
// report). Each member is sized by the rule it already uses (src/physics/members.js; src/design/round17.mjs SIZING19
// carries the numbers) and pinned on the v25 build (v24's params with round17Set, as the quick probe builds it): the
// build's member carries the new section, the old section is refused at the 14 m/s row's peak loads, the new one passes,
// and each FoS is held to a hand calculation from the published figures (the catalogue's design values), not to this
// code's own formulas. The peak loads are the flight model's at that row (x3's mass and trim, the members rebuilt from
// the v25 code: r19 members3/logs/s3.json); they are the load case, not the expectation.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { checkMembers } from '../../src/physics/members.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = () => JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
const FOS = 2.5; // the members' FoS 1.5 under the envelope's member allowance 0.6
const rel = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol * Math.abs(b), `${a} vs ${b} (rel ${tol})`);
const spec = (id) => catalog.find((p) => p.id === id).specs;
// design values as the catalogue entries state them (MPa): EN 485-2 / EN 754-2 6061-T6 (Rp0.2
// 240, Rm 260 for every form, the ASM bearing 386 at e/D 2 counted at half); Araldite 2011 on aluminium at half
const D6061 = spec('metal-al-6061-t6').design_values;
const LAP = spec('adhesive-araldite-2011-aluminium').design_values.shear_MPa;

let built = null;
async function v25() {
  if (built) return built;
  const { buildDesign } = await import('../../src/design/build.js');
  const { round17Set, mergeDeep } = await import('../../src/design/round17.mjs');
  const { solidMassParams } = await import('../../src/design/round11.mjs');
  const set = round17Set({}, catalog);
  let p = mergeDeep(v24().params, set);
  for (const k of ['massFromSolids', 'lumpPos']) if (k in set) p[k] = set[k];
  if (p.massFromSolids) p = solidMassParams(p, catalog);
  built = buildDesign(p, catalog);
  return built;
}
const member = (d, name) => { const m = d.structure.members.find((q) => q.name === name); assert.ok(m, `no member ${name}`); return m; };
function fosAt(m, load) {
  const peaks = {};
  for (const [k, names] of Object.entries(m.uses)) for (const n of [].concat(names)) peaks[`${m.name}/${n}`] = load[k] ?? 0;
  return checkMembers([m], peaks).members[0].fos;
}
const withDims = (m, dims) => ({ ...m, dims: { ...m.dims, ...dims } });

// ---- hand references -------------------------------------------------------------------------------------------
// EN 1999-1-1 Table 8.5 bolt bearing on a ring of outside D round a hole d (e1 = e2 = D / 2, over d), gamma_M2 1.25,
// against the half-counted published bearing (whichever is less); null below EN's 1.2 d0 edge distance
function enBearing(D, d, fu = D6061.ultimate_MPa) {
  const e = D / 2 / d;
  if (e < 1.2) return null;
  const en = (Math.min(2.8 * e - 1.7, 2.5) * Math.min(e / 3, 1) * fu) / 1.25;
  return Math.min(en, D6061.bearing_yield_MPa);
}
// a ring lug d / D / t under F: bearing (above), net tension and shear-out against the plate's yield and shear yield
function lugFos(d, D, t, F) {
  const b = enBearing(D, d);
  if (b === null) return null;
  const w = D - d;
  return Math.min(b / (F / (d * t)), D6061.yield_MPa / (F / (w * t)), D6061.shear_yield_MPa / (F / (2 * t * (w / 2))));
}

test('the rails\' lock screws bear in the aft plate and a 0.8 mm pad (2.8 mm): in the 2 mm plate alone they are refused', async () => {
  const d = await v25(), ra = d.params.linkage.airframe.rail;
  const { lockPadCircles } = await import('../../src/design/airframe.js');
  // an M3 screw's seat at e/d 2 in 6061-T6: the published bearing (386 MPa at e/D 2) counted at half
  const hand = (F, t) => D6061.bearing_yield_MPa / (F / (3 * t));
  for (const [side, F] of [['right', 560.289], ['left', 512.6011]]) {
    const m = member(d, `shoulder rail ${side}, torsion lock screws in the plate`);
    assert.equal(m.dims.t_mm, 2 + 0.8); assert.equal(m.dims.e_over_d, 2);
    rel(fosAt(m, { F }), hand(F, 2.8), 5e-3);
    assert.ok(hand(F, 2.8) >= FOS, `${side} 2.8 mm at ${hand(F, 2.8)}`);
    if (side === 'right') assert.ok(hand(F, 2) < FOS);
  }
  // the pad gives each screw the e/d 2 the seat is rated at: lock_edge_mm (6) round each M3, 2 d
  for (const q of lockPadCircles(ra, [0, 0], 1).slice(1)) assert.ok(q.r * 1000 >= 2 * 3 - 1e-9);
  // and it is no ring of seat_D round the rail, so the rail's seat lug keeps the plate's own 2 mm
  assert.equal(member(d, 'plate seat shoulder rail right aft').dims.t_mm, 2);
});

test('the rails\' aft plate seats 34.2 mm across (e/d 1.315): 31.5 mm is refused at 14 m/s; the 7 mm collar leaves 0.5 mm', async () => {
  const d = await v25(), ra = d.params.linkage.airframe.rail;
  for (const [side, F] of [['left', 1673.2183], ['right', 1665.2062]]) {
    const m = member(d, `plate seat shoulder rail ${side} aft`);
    assert.equal(m.dims.d_mm, 13); assert.equal(m.dims.D_mm, 34.2); assert.equal(m.dims.t_mm, 2);
    rel(fosAt(m, { F }), lugFos(13, 34.2, 2, F), 5e-3);
    assert.ok(lugFos(13, 34.2, 2, F) >= FOS, `${side} 34.2 at ${lugFos(13, 34.2, 2, F)}`);
    assert.ok(lugFos(13, 31.5, 2, F) < FOS, `${side} 31.5 at ${lugFos(13, 31.5, 2, F)}`);
  }
  // the seat ring beside the shoulder pin's collar: the rail sits dz, dy off the hinge (the pin's axis); its ring and the
  // 7 mm collar keep the 0.5 mm running clearance; the 9 mm collar would cut into the ring
  const gap = (collar) => Math.hypot(ra.dz_mm, ra.dy_mm) - ra.seat_D_mm / 2 - collar / 2;
  assert.equal(d.params.linkage.rocker.hub_collar_od_mm, 7);
  assert.ok(gap(7) >= 0.5 - 1e-9, `7 mm collar gap ${gap(7)}`);
  assert.ok(gap(9) < 0.5);
  // the aft plate holds the whole ring: every point of it lies in the plate's outline or the rail's lock ear (the
  // outlines are stored to 0.01 mm, so the ring is sampled 0.02 mm inside its edge, at the outline's own 5 deg vertices)
  const pl = d.structure.airframe.parts.plates.aft, polys = [pl.outline_m, ...(pl.ears ?? []).map((e) => e.outline_m)];
  const inside = (P, p) => { let s = 0; for (let i = 0; i < P.length; i++) { const a = P[i], b = P[(i + 1) % P.length], c = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]); if (Math.abs(c) < 1e-12) continue; if (!s) s = Math.sign(c); else if (Math.sign(c) !== s) return false; } return true; };
  const rail = d.structure.airframe.parts.rails;
  for (const side of ['right', 'left']) for (let i = 0; i < 72; i++) {
    const a = (i * Math.PI) / 36, r = ra.seat_D_mm / 2000 - 2e-5, p = [rail[side][0] + r * Math.cos(a), rail[side][1] + r * Math.sin(a)];
    assert.ok(polys.some((P) => inside(P, p)), `${side} seat ring point at ${(a * 180 / Math.PI).toFixed(0)} deg outside the aft plate`);
  }
});

test('the side panels\' end flange bonds: a third rectangle at each end (327 mm2), the two-rectangle 278 mm2 refused', async () => {
  const d = await v25(), list = d.params.linkage.airframe.panels.list;
  const area = (rs) => rs.reduce((a, r) => a + (r.z[1] - r.z[0]) * (r.y[1] - r.y[0]), 0);
  const F = 1551.8537; // N, half the forward plate's net force plus the box torsion over the sides' spacing
  for (const side of ['right', 'left']) {
    const q = list.find((p) => p.name === `side panel ${side}`), m = member(d, `gearbox side panel ${side} flange bond`);
    const A = Math.min(area(q.flanges), area(q.flanges_fwd ?? q.flanges)); // each end carries the panel's whole force
    rel(m.dims.w_mm * m.dims.L_mm, A, 1e-3);
    rel(fosAt(m, { F }), (LAP * A) / F, 5e-3);
    assert.ok((LAP * A) / F >= FOS, `${side} ${A} mm2 at ${(LAP * A) / F}`);
    assert.ok((LAP * area(q.flanges.slice(0, 2))) / F < FOS, `${side} two rectangles`);
  }
});
