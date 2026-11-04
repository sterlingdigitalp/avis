// Round 19 (sizing to the FoS rule at the rough-air row): the three members round 18 left short at 14 m/s / 3.023 Hz
// (the fastest kept trim row of round 18's x1 build) are sized by the rules they already use: the tail boom on Rock
// West's T-RND-472, the twist joiners' outer sleeves 24 mm long, the crank shaft solid. Each is pinned on the v25 build
// (v24's params with round17Set, as the quick probe builds it): the build's member carries the new section, the old
// section is refused at the rough row's peak loads, the new one passes, and each FoS is held to a hand calculation from
// the published figures (psi as the pages print them), not to this code's own numbers. The peak loads are the flight
// model's at that row (round 18's x1 design, its members rebuilt from the v25 code: r19 members/logs/set1-fly.log); they
// are the load case, not the expectation.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { checkMembers } from '../../src/physics/members.js';
import { TWIST_DRIVE } from '../../src/design/twist-drive.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const byId = (id) => catalog.find((p) => p.id === id);
const v24 = () => JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
const PSI = 0.00689475729; // MPa per psi
const IN = 25.4, LB = 453.59237; // mm per inch, g per lb
const FOS = 2.5; // the members' FoS 1.5 under the envelope's member allowance 0.6
const rel = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol * Math.abs(b), `${a} vs ${b} (rel ${tol})`);

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
const member = (d, name) => d.structure.members.find((m) => m.name === name);
// a member's FoS at given section peaks ({ M, T, V, F } on the sections its `uses` names)
function fosAt(m, load) {
  const peaks = {};
  for (const [k, names] of Object.entries(m.uses)) for (const n of [].concat(names)) peaks[`${m.name}/${n}`] = load[k] ?? 0;
  return checkMembers([m], peaks).members[0].fos;
}

// ---- the tail boom at the aft clamp ----------------------------------------------------------------------------------
// the rough row's peaks on the T-RND-472 boom (its inertia included): N m, N m, N
const BOOM = { M: 8.99, T: 1.357, V: 10.746 };
// the cfrp-tube rule by hand: bending over the published compressive strength at half, torsion and shear over the CLT XY
// shear strength at half (Rock West's pages: X compressive 244,000 psi, XY shear 13,992 / 13,996 psi, reference only)
function boomFos(D, d, shearPsi, { M, T, V }) {
  const Z = (Math.PI * (D ** 4 - d ** 4)) / (32 * D), A = (Math.PI * (D * D - d * d)) / 4; // mm3, mm2
  const sigma = (M * 1000) / Z, tau = (T * 1000) / (2 * Z) + (2 * V) / A; // MPa
  return 1 / (sigma / ((244000 * PSI) / 2) + tau / ((shearPsi * PSI) / 2));
}

test('the T-RND-472 entries are the page\'s published section, weight and CLT figures', () => {
  const t = byId('tube-rockwest-pultruded-12x10'), tr = byId('composite-rockwest-t-rnd-472-transverse');
  assert.equal(t.specs.part_number, 'T-RND-472');
  rel(t.specs.od_mm, 0.465 * IN, 1e-4); rel(t.specs.id_mm, 0.39 * IN, 1e-4); rel(t.specs.wall_mm, 0.038 * IN, 1e-3);
  // 0.216 lb for the 78 in piece
  rel(t.mass_g, (0.216 * LB) / (78 * IN / 1000), 1e-3);
  rel(t.specs.compressive_strength_mpa, 244000 * PSI, 1e-3);
  rel(tr.specs.design_values.xy_shear_MPa, (13996 * PSI) / 2, 2e-3);
  assert.ok(tr.specs.design_values.xy_shear_MPa <= (13996 * PSI) / 2, 'rounded down');
  rel(tr.specs.design_values.yield_MPa, (38600 * PSI) / 2, 1e-3);
});

test('the v25 boom is T-RND-472; the T-RND-393 section is refused at the rough row\'s boom moment, the new one passes', async () => {
  const d = await v25(), m = member(d, 'tail boom at the aft clamp');
  assert.equal(d.params.parts.keel, 'tube-rockwest-pultruded-12x10');
  assert.equal(m.dims.d_mm, 11.811); assert.equal(m.dims.di_mm, 9.906);
  const now = fosAt(m, BOOM);
  rel(now, boomFos(0.465 * IN, 0.39 * IN, 13996, BOOM), 5e-3);
  assert.ok(now >= FOS, `T-RND-472 at ${now}`);
  // the old boom, on its own published section and its own CLT record, at the same moment
  const old = byId('tube-rockwest-pultruded-10x8'), oldT = byId('composite-rockwest-t-rnd-393-transverse');
  const m0 = { ...m, dims: { d_mm: old.specs.od_mm, di_mm: old.specs.id_mm }, material: { ...m.material, id: old.id, axial_MPa: old.specs.compressive_strength_mpa / 2, shear_MPa: oldT.specs.design_values.xy_shear_MPa } };
  const was = fosAt(m0, BOOM);
  rel(was, boomFos(0.393 * IN, 0.314 * IN, 13992, BOOM), 5e-3);
  assert.ok(was < FOS, `T-RND-393 at ${was}`);
});

test('the boom clamps\' M3 shanks lie in their flange below the 14.81 mm collar; at round 18\'s 8 mm offset they would not', async () => {
  const { AIRFRAME_DEFAULTS, clampFlange } = await import('../../src/design/airframe.js');
  const d = await v25(), bo = { ...AIRFRAME_DEFAULTS.boom, ...d.params.linkage.airframe.boom }, yB = bo.y_mm / 1000;
  const blk = (0.465 * IN + 2 * bo.clamp_wall_mm) / 1000, r = 0.0015; // the collar on T-RND-472; an M3 shank's radius
  assert.equal(bo.screw_offset_mm, 9); assert.equal(bo.screw_edge_mm, 5);
  const top = (b) => clampFlange(b, blk, yB).y[1]; // the flange's upper edge: the collar's lower face
  rel(top(bo), yB - blk / 2, 1e-9);
  assert.ok(yB - bo.screw_offset_mm / 1000 + r <= top(bo), 'the shank\'s upper side inside the flange');
  assert.ok(yB - 0.008 + r > top({ ...bo, screw_offset_mm: 8 }), 'at 8 mm the shank would stand out of the flange into the open corner');
  // its lower edge where round 18's was (8 + 6 mm below the axis): at 15 mm the right horn boss's sweep cut it by 0.36 mm
  // (probed)
  rel(clampFlange(bo, blk, yB).y[0], yB - 0.014, 1e-9);
});

// ---- the twist joiners' outer sleeve bonds ---------------------------------------------------------------------------
// the rough row's peak axial shear force on the right joiner's outer bond (the elbow moment as 4 M / d, and the hand
// wing's spanwise force), N; the bond rule by hand: tau = F / (pi d L) against Araldite 2011 on CFRP, 2770 psi
// (ISO 4587, typical) at half
const JOINER_F = 2010.077;
const bondFos = (dmm, Lmm, F) => ((2770 * PSI) / 2) / (F / (Math.PI * dmm * Lmm));

// (round 19 members2: the sealed design's own rough row sized them on to 30 mm, sizing19-sealed.test.js; at least 24 here)
test('the twist joiners\' outer sleeves are at least 24 mm: the 20 mm bond is refused at the rough row\'s force, theirs passes', async () => {
  const Lo = TWIST_DRIVE.joiner.outer_mm;
  assert.ok(Lo >= 24, `outer sleeve ${Lo} mm`);
  const d = await v25();
  for (const t of ['R', 'L']) {
    const m = member(d, `twist joiner ${t} outer sleeve bond`);
    assert.equal(m.dims.L_mm, Lo);
    const now = fosAt(m, { F: JOINER_F, T: 0 });
    rel(now, bondFos(m.dims.d_mm, Lo, JOINER_F), 2e-3);
    assert.ok(now >= FOS, `${t} at ${now}`);
    const was = fosAt({ ...m, dims: { ...m.dims, L_mm: 20 } }, { F: JOINER_F, T: 0 });
    rel(was, bondFos(m.dims.d_mm, 20, JOINER_F), 2e-3);
    assert.ok(was < FOS, `${t} 20 mm at ${was}`);
  }
});

// ---- the crank shaft -------------------------------------------------------------------------------------------------
// the rough row's peaks on the crank shaft: bending N m, torque N m, shear N
const CRANK = { M: 30.274, T: 11.048, V: 2449.169 };
// the round-member rule by hand: von Mises at the outer fibre and at the neutral axis (shear 4/3 V/A solid, 2 V/A for
// a tube), against ASTM B348 Grade 5's minimum yield, 120 ksi
function crankFos(D, d, { M, T, V }) {
  const Z = (Math.PI * (D ** 4 - d ** 4)) / (32 * D), A = (Math.PI * (D * D - d * d)) / 4;
  const s = (M * 1000) / Z, tT = (T * 1000) / (2 * Z), tV = ((d > 0 ? 2 : 4 / 3) * V) / A;
  return (120000 * PSI) / Math.max(Math.sqrt(s * s + 3 * tT * tT), Math.sqrt(3) * (tT + tV));
}

test('the v25 crank shaft is solid: the 4 mm bore is refused at the rough row\'s loads, the solid shaft passes', async () => {
  const d = await v25(), m = member(d, 'crank shaft');
  assert.equal(m.dims.d_mm, 10); assert.equal(m.dims.di_mm, 0);
  const now = fosAt(m, CRANK);
  rel(now, crankFos(10, 0, CRANK), 2e-3);
  assert.ok(now >= FOS, `solid at ${now}`);
  const was = fosAt({ ...m, dims: { ...m.dims, di_mm: 4 } }, CRANK);
  rel(was, crankFos(10, 4, CRANK), 2e-3);
  assert.ok(was < FOS, `4 mm bore at ${was}`);
  // the BOM's shaft is the solid one (its basis names no bore)
  const it = d.bom.structure.find((q) => q.name === 'crank shaft');
  assert.match(it.basis, /^10 mm shaft x /);
});
