// Round 19 (members2: sizing to the FoS rule at the sealed design's own trims). With the pod sealed and the CG at the
// 0.10 margin the free-flight trims fly faster strokes: the exploration build x3 (the current design) trimmed 11-14 m/s
// at 2.850-3.291 Hz, the rough-air row 14 m/s at 3.291 Hz (round 18's x1: 3.023 Hz there), and these members fell below
// FoS 2.5 there. Each is sized by the rule it
// already uses (src/design/round17.mjs SIZING19, src/design/twist-drive.js TWIST_DRIVE.joiner, the shoulder pin's
// stepped section in src/design/linkage.js). Each is pinned on the v25 build (v24's params with round17Set, as the quick
// probe builds it): the build's member carries the new section, the old section is refused at the 13 m/s row's peak
// loads, the new one passes, and each FoS is held to a hand calculation from the published figures (sections by strip
// summation, not this code's section formulas). The peak loads are the flight model's at the 14 m/s row (x3's mass and
// trim, the members rebuilt from the v25 code: r19 members2/logs/s4.json); they are the load case, not the expectation.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { checkMembers, channelSection } from '../../src/physics/members.js';
import { TWIST_DRIVE } from '../../src/design/twist-drive.js';
import { rockerHub } from '../../src/design/linkage.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = () => JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
const FOS = 2.5; // the members' FoS 1.5 under the envelope's member allowance 0.6
const rel = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol * Math.abs(b), `${a} vs ${b} (rel ${tol})`);
// design yields as the catalogue entries state them (MPa): EN 754-2 / EN 485-2 minimums; ASTM B348 Gr 5
const Y6061 = 240, TAU6061 = 138.6, Y7075 = 485, YTI = 827;

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
// a member's FoS at given section peaks ({ M, T, V, F } on the sections its `uses` names)
function fosAt(m, load) {
  const peaks = {};
  for (const [k, names] of Object.entries(m.uses)) for (const n of [].concat(names)) peaks[`${m.name}/${n}`] = load[k] ?? 0;
  return checkMembers([m], peaks).members[0].fos;
}
const withDims = (m, dims) => ({ ...m, dims: { ...m.dims, ...dims } });

// ---- sections by strip summation (independent of members.js channelSection / twist-drive.js uSection) ---------------
// rectangles [z0, z1, y0, y1] (mm), each cut into n x n cells: area and the smaller section modulus about each centroidal axis
function stripSection(rects, n = 400) {
  let A = 0, Sz = 0, Sy = 0;
  const cells = [];
  for (const [z0, z1, y0, y1] of rects) {
    const dz = (z1 - z0) / n, dy = (y1 - y0) / n;
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) { const z = z0 + (i + 0.5) * dz, y = y0 + (j + 0.5) * dy, a = dz * dy; cells.push([z, y, a]); A += a; Sz += a * z; Sy += a * y; }
  }
  const zc = Sz / A, yc = Sy / A;
  let Iz = 0, Iy = 0, cy = 0, cz = 0;
  for (const [z, y, a] of cells) { Iz += a * (y - yc) ** 2; Iy += a * (z - zc) ** 2; }
  for (const [z0, z1, y0, y1] of rects) { cy = Math.max(cy, Math.abs(y0 - yc), Math.abs(y1 - yc)); cz = Math.max(cz, Math.abs(z0 - zc), Math.abs(z1 - zc)); }
  return { A, Zout: Iz / cy, Zin: Iy / cz };
}
// a bent-sheet channel as members.js idealises it: the web w x t, the flanges t thick centred on +-w/2 and reaching h
// above the web's mid-plane (mm); the strips leave out the corners' overlap the thin-wall formula counts twice, so the
// two agree to about 1 %
const channelRects = (w, h, t) => [[-w / 2, w / 2, 0, t], [-w / 2 - t / 2, -w / 2 + t / 2, t, h + t / 2], [w / 2 - t / 2, w / 2 + t / 2, t, h + t / 2]];

test('the nose tray\'s flanges 7 mm: the 5 mm channel is refused at the 14 m/s root moment, 7 mm passes (strip-summed section)', async () => {
  const d = await v25(), m = member(d, 'nose tray at the root');
  assert.match(m.dims.section, /^channel 37\.6 x 7 x 0\.6 mm$/);
  const M = 1.1715; // N m, both ways at full value (the inertia moment of everything beyond the root)
  const hand = (h) => { const s = stripSection(channelRects(37.6, h, 0.6)); return Y6061 / ((M * 1000) / s.Zin + (M * 1000) / s.Zout); };
  // the old section as the rule states it (members.js channelSection), at the same moment
  const c5 = channelSection(37.6, 5, 0.6);
  const now = fosAt(m, { M, Mo: M }), was = fosAt(withDims(m, { A_mm2: c5.A_mm2, Zi_mm3: c5.Zi_mm3, Zo_mm3: c5.Zo_mm3 }), { M, Mo: M });
  rel(now, hand(7), 0.02);
  rel(was, hand(5), 0.02);
  assert.ok(now >= FOS, `7 mm flanges at ${now}`);
  assert.ok(was < FOS, `5 mm flanges at ${was}`);
});

test('the shoulder rails 13/6.5 mm: 13/8 is refused at the 14 m/s torsion and shear (neutral axis), 13/6.5 passes', async () => {
  const d = await v25();
  // roundMemberCheck by hand: von Mises at the outer fibre and, for a tube, torsion plus 2 V / A at the neutral axis
  const hand = (D, di, { M, T, V }) => {
    const J = (Math.PI / 32) * (D ** 4 - di ** 4), A = (Math.PI / 4) * (D * D - di * di);
    const s = (M * 1000 * (D / 2)) / (J / 2), tT = (T * 1000 * (D / 2)) / J, tV = (2 * V) / A;
    return Y7075 / Math.max(Math.hypot(s, Math.sqrt(3) * tT), Math.sqrt(3) * (tT + tV));
  };
  for (const [side, load] of [['right', { M: 22.0361, T: 29.135, V: 1665.2062 }], ['left', { M: 27.0573, T: 26.6553, V: 1673.2183 }]]) {
    const m = member(d, `shoulder rail ${side}`);
    assert.equal(m.dims.d_mm, 13); assert.equal(m.dims.di_mm, 6.5);
    const now = fosAt(m, load), was = fosAt(withDims(m, { di_mm: 8 }), load);
    rel(now, hand(13, 6.5, load), 5e-3); rel(was, hand(13, 8, load), 5e-3);
    assert.ok(now >= FOS, `${side} 13/6.5 at ${now}`);
    assert.ok(was < FOS, `${side} 13/8 at ${was}`);
  }
});

test('the shoulder brackets\' arms 12.5 mm wide: 11 mm is refused in in-plane bending at 14 m/s, 12.5 passes', async () => {
  const d = await v25();
  const hand = (b, h, lever, F) => Y6061 / ((6 * F * lever) / (h * b * b));
  for (const [nm, F] of [['shoulder bracket left aft, arm', 822.4101], ['shoulder bracket right forward, arm', 761.929]]) {
    const m = member(d, nm);
    assert.equal(m.dims.b_mm, 12.5); assert.equal(m.dims.h_mm, 8);
    rel(fosAt(m, { F }), hand(12.5, 8, m.dims.lever_mm, F), 5e-3);
    assert.ok(hand(12.5, 8, m.dims.lever_mm, F) >= FOS);
    if (/left aft/.test(nm)) assert.ok(hand(11, 8, m.dims.lever_mm, F) < FOS);
  }
});

test('the gearbox side panels 1.1 mm: 0.9 mm is refused in shear at the 14 m/s box force, 1.1 passes', async () => {
  const d = await v25(), p = member(d, 'gearbox side panel right');
  const V = 1551.8537; // N, half the forward plate's net force plus the box torsion over the sides' spacing
  assert.equal(p.dims.t_mm, 1.1); assert.equal(p.dims.h_mm, 29);
  // shear yield (von Mises from EN 485-2's 240 MPa) governs; the plate's buckling stress, k pi^2 E / (12 (1 - nu^2)) (t /
  // h)^2 with k = 5.35 + 4 (h / b)^2, is far above it
  const k = 5.35 + 4 * (29 / p.dims.b_mm) ** 2, tauCr = (k * Math.PI ** 2 * 68900) / (12 * (1 - 0.33 ** 2)) * (1.1 / 29) ** 2;
  assert.ok(tauCr > TAU6061);
  rel(fosAt(p, { V }), TAU6061 / (V / (29 * 1.1)), 5e-3);
  assert.ok(TAU6061 / (V / (29 * 1.1)) >= FOS);
  assert.ok(TAU6061 / (V / (29 * 0.9)) < FOS);
});

test('the crank web 16 mm wide: 14 mm is refused at the 14 m/s pin moment and coupler force, 16 passes', async () => {
  const d = await v25(), m = member(d, 'crank web');
  const L = { M: 17.9259, F: 2368.4543 };
  const hand = (b, h, lever) => YTI / ((6 * L.M * 1000) / (b * h * h) + (6 * L.F * lever) / (h * b * b));
  assert.equal(m.dims.b_mm, 16); assert.equal(m.dims.h_mm, 6);
  rel(fosAt(m, L), hand(16, 6, m.dims.lever_mm), 5e-3);
  assert.ok(hand(16, 6, m.dims.lever_mm) >= FOS);
  assert.ok(hand(14, 6, m.dims.lever_mm) < FOS);
});

test('the rocker arms 12.5 mm wide and the left horn boss 30 mm across: 11 mm and 25 mm refused at 14 m/s', async () => {
  const d = await v25();
  const arm = (b, h, lever, F) => Y7075 / ((6 * F * lever) / (h * b * b));
  for (const [side, F] of [['left', 1149.1281], ['right', 1234.9263]]) {
    const m = member(d, `rocker arm ${side}`);
    assert.equal(m.dims.b_mm, 12.5);
    rel(fosAt(m, { F }), arm(12.5, m.dims.h_mm, m.dims.lever_mm, F), 5e-3);
    assert.ok(arm(12.5, m.dims.h_mm, m.dims.lever_mm, F) >= FOS);
    assert.ok(arm(11, m.dims.h_mm, m.dims.lever_mm, F) < FOS);
  }
  // the horn boss: bearing F / (d t) + 6 M / (d t^2) against EN 1999-1-1's bolt bearing k1 alpha_b fu / gamma_M2 at half
  // (the standard does not list 7075): k1 = min(2.8 e/d - 1.7, 2.5), alpha_b = e / (3 d), fu 540 MPa, gamma_M2 1.25
  const m = member(d, 'horn boss left'), L = { F: 1149.1281, M: 4.0219 };
  assert.equal(m.dims.e_over_d, 15 / 8);
  const boss = (D) => {
    const e = D / 2 / 8, allow = (Math.min(2.8 * e - 1.7, 2.5) * (e / 3) * 540) / 1.25 / 2;
    return allow / (L.F / (8 * m.dims.t_mm) + (6 * L.M * 1000) / (8 * m.dims.t_mm ** 2));
  };
  rel(fosAt(m, L), boss(30), 5e-3);
  assert.ok(boss(30) >= FOS); assert.ok(boss(25) < FOS);
});

// members3: the collar 9 -> 7 mm across (rocker.hub_collar_od_mm), so the rails' aft plate seats can reach 34.2 mm beside it
test('the shoulder pin is a stepped shaft at its collar: its own 6 mm section at the collar\'s end, the 7 mm collar at the hub face', async () => {
  const d = await v25();
  const r = d.params.linkage.rocker, span = d.params.joints.shoulderSpan_mm / 1000, B = 0.006;
  const planes = d.structure.joints_layout.planes_m;
  const hub = rockerHub(d, r, span, B, planes, B);
  // the forward bushing's centre is half its length past its flange, which the collar stops the running clearance short
  // of: the arm of the pin's own section is B / 2 + 0.5 mm whatever the plate; the collar's is the whole gap to the hub
  rel(hub.levers.collar * 1000, 3 + 0.5, 1e-6);
  rel(hub.levers.forward * 1000, (span / 2 - hub.x[1]) * 1000, 1e-6);
  const Ffwd = 761.929; // N, the forward bushing's peak radial force at 14 m/s
  const pin = (dd, lever) => Y7075 / Math.max((Ffwd * lever * 32) / (Math.PI * dd ** 3), (Math.sqrt(3) * 4 / 3 * Ffwd) / ((Math.PI / 4) * dd * dd));
  const own = member(d, 'shoulder pin right'), col = member(d, 'shoulder pin right at its collar');
  assert.equal(own.dims.d_mm, 6); assert.equal(col.dims.d_mm, 7);
  rel(fosAt(own, { M: Ffwd * hub.levers.collar, V: Ffwd }), pin(6, hub.levers.collar * 1000), 5e-3);
  rel(fosAt(col, { M: Ffwd * hub.levers.forward, V: Ffwd }), pin(7, hub.levers.forward * 1000), 5e-3);
  assert.ok(pin(6, hub.levers.collar * 1000) >= FOS && pin(7, hub.levers.forward * 1000) >= FOS);
  // the one-section reading (the 6 mm pin over the whole arm to the hub face) is refused at the same force
  assert.ok(pin(6, hub.levers.forward * 1000) < FOS);
});

test('the twist joiners: inner sleeve 20 mm, outer 30 mm, bay 1.0 mm; 15 / 24 / 0.8 refused at 14 m/s with the lagging chain', async () => {
  const d = await v25();
  assert.equal(TWIST_DRIVE.joiner.inner_mm, 20); assert.equal(TWIST_DRIVE.joiner.outer_mm, 30); assert.equal(TWIST_DRIVE.joiner.u_t_mm, 1.0);
  // bondCheck by hand: the torque as a ring force 2 T / d with the axial force, over pi d L, against Araldite 2011 on
  // CFRP (2770 psi typical, at half: 9.55 MPa)
  const bond = (dd, L, { T = 0, F = 0 }) => 9.55 / (Math.hypot((2 * T * 1000) / dd, F) / (Math.PI * dd * L));
  for (const [nm, L, Lold, load] of [['twist joiner R inner sleeve bond', 20, 15, { T: 1.2968, F: 2075.7622 }], ['twist joiner R outer sleeve bond', 30, 24, { F: 2589.6436 }]]) {
    const m = member(d, nm);
    assert.equal(m.dims.L_mm, L);
    rel(fosAt(m, load), bond(m.dims.d_mm, L, load), 5e-3);
    assert.ok(bond(m.dims.d_mm, L, load) >= FOS, `${nm} at ${L} mm`);
    assert.ok(bond(m.dims.d_mm, Lold, load) < FOS, `${nm} at ${Lold} mm`);
  }
  // the bay: its floor and two low walls (od 12.4, walls to 2.5 / 2 mm above the axis), strip-summed
  const ro = 6.2, U = (t) => stripSection([[-(ro - t), ro - t, -ro, -ro + t], [-ro, -ro + t, -ro, 2.5], [ro - t, ro, -ro, 2]]);
  const bay = member(d, 'twist joiner R bay'), Lb = { M: 5.1763, F: 2.979 };
  const hand = (t) => { const s = U(t); return Y7075 / (Lb.F / s.A + (Lb.M * 1000) / Math.min(s.Zout, s.Zin)); };
  rel(fosAt(bay, Lb), hand(1.0), 5e-3);
  assert.ok(hand(1.0) >= FOS); assert.ok(hand(0.8) < FOS);
});
