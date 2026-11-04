// Round 18: three bugs found by the trim diagnosis (docs/status/diagnose-trim-v25.md), each pinned against an
// independent reference, never against this code's own output.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { outsideDrag } from '../../src/design/round11.mjs';
import { TWIST_SECTION, twistMembers, twistLoads } from '../../src/design/twist-drive.js';
import { solidProps } from '../../src/design/massprops.js';
import { materialOf } from '../../src/design/linkage.js';
import { adhesiveOf } from '../../src/design/airframe.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''}: ${a} vs ${b} (tol ${tol})`);

// ---- item 2: protrusion drag on both wings (round11.mjs outsideDrag, diagnosis 1.3/4.1) ----------------------------

test('outsideDrag counts the wing-protrusion CdA on both wings, from an independent sum over the section record\'s rows', () => {
  // twist-drive.js drawnLines draws one wing only (twistWing(ax, 'R', frame)), so sectionRecord's protrusions and
  // CdA_m2 are the RIGHT wing's; both wings protrude the same way, so the body's added CdA must be twice that.
  const protrusions = [
    { part: 'twist joiner R', frontal_mm2: 12.3, declared: true, why: 'x', Cd: 1.2, CdA_m2: 1.2 * 12.3e-6 },
    { part: 'twist frame pins R', frontal_mm2: 4.1, declared: true, why: 'x', Cd: 1.2, CdA_m2: 1.2 * 4.1e-6 },
    { part: 'servo arm R', declared: false }, // undeclared rows have no CdA_m2 at all
  ];
  // Computed here, over the rows, NOT read from a pre-summed field: an independent sum.
  const independentSum = protrusions.reduce((a, p) => a + (p.CdA_m2 ?? 0), 0);
  const section = { rule: 'x', sparN_mm: 1, travel_rad: 0.6, rows: [], protrusions, undeclared: [], CdA_m2: independentSum, passed: true };
  const frontal = { CdA_m2: 5e-4, area_mm2: 400, Cd: 1.2, basis: 'x' };
  const d = { body: { CdA_front_pod_m2: 0.002 }, structure: { clearance: { containment: { frontal }, section } } };
  outsideDrag(d);
  // outsideDrag rounds CdA_front_wing_protrusions_m2 to 7 decimals (toFixed(7)); the tolerance covers that rounding.
  near(d.body.CdA_front_wing_protrusions_m2, 2 * independentSum, 1e-7, 'both wings, from the independent sum');
  near(d.body.CdA_front_m2, d.body.CdA_front_pod_m2 + frontal.CdA_m2 + 2 * independentSum, 1e-7, 'total body CdA');
});

// ---- item 3: the rib-on-rod member's load (twist-drive.js, diagnosis 2.5(a)) ----------------------------------------

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const base = JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
/** The official design with the drive stated, as test/design/twist-drive.test.js builds it. */
function withDrive(over = {}) {
  const d = structuredClone(base);
  d.params = { ...d.params, tubeSpecSheet: true, parts: { ...d.params.parts, twistServo: 'servo-hitec-md89mw' }, twistDrive: { section: TWIST_SECTION, ...over } };
  return d;
}
function buildTwistMembers(d) {
  const members = [];
  twistMembers({ d, catalog, members, mat: (id) => materialOf(catalog, id), glueAl: adhesiveOf(catalog, 'adhesive-araldite-2011-aluminium'), glueCf: adhesiveOf(catalog, 'adhesive-araldite-2011') });
  return members;
}
/** A member's load from its first section's radial terms, against a channel-value map (as compileSupports combines them: sum |k| x channel). */
const loadOf = (m, chv) => m.sections[0].radial.reduce((acc, t) => acc + Math.abs(t.k) * (chv[t.ch] ?? 0), 0);

test('wing ribs on the edge rods: loaded by the flown twist ANGLE at R_max, not torque over a soft aerodynamic stiffness', () => {
  const D = withDrive();
  const members = buildTwistMembers(D);
  const L = twistLoads(D, catalog);
  // R_max computed here from the drawn rods' own R_N_per_rad (independent of twistLoads' returned Rrod field).
  const Rmax = Math.max(...L.stiff.rods.flatMap((r) => r.R_N_per_rad.map(Math.abs)));
  const m = members.find((mm) => mm.name === 'wing ribs R on the edge rods');
  assert.ok(m, 'the member is drawn');
  const term = m.sections[0].radial.find((t) => t.ch === 'twist_angle_R');
  assert.ok(term, 'carries the twist_angle_R channel (the hardware-bounded twist, not twist_R torque)');
  near(term.k, Rmax, 1e-6, 'the member\'s coefficient is R_max'); // the member rounds k to 6 decimals (twist-drive.js r6)
  assert.ok(!m.sections[0].radial.some((t) => t.ch === 'twist_R'), 'no longer loaded by the raw twist torque');
  // At a known twist angle (well inside the chain's travel_rad, about 0.675 rad): load = R_max x theta.
  const theta = 0.2;
  near(loadOf(m, { twist_angle_R: theta, root_yz_R: 0 }), Rmax * theta, 1e-6, 'load at a known twist angle');
  // The old bug read a large twist TORQUE over a soft stiffness as a large twist; a large torque at a small angle
  // must no longer load this member at all (it carries no twist_R term any more).
  near(loadOf(m, { twist_angle_R: 0.01, twist_R: 1000, root_yz_R: 0 }), Rmax * 0.01, 1e-6, 'a large torque on the unused channel does not add load');
});

// ---- item 4: tube product-of-inertia sign (massprops.js, diagnosis 1.3) ---------------------------------------------

test('solidProps: a tube\'s off-diagonal inertia (y-z, 45 degrees) matches the analytic rotated-tube tensor', () => {
  // A 3/2 mm (outer/inner radius) x 100 mm tube lying at 45 deg between y and z (body axes): only Iyz is nonzero
  // (the tube's axis has no x-component, diagnosis 1.3). Iu (about its own axis) and Ip (about a transverse axis
  // through its centre) are the standard hollow-cylinder formulas, independent of massprops.js.
  const ro = 0.003, ri = 0.002, L = 0.1;
  const s = { tube: { ro, ri, L }, x: [0, 0], shape: { kind: 'cap', a: [0, 0], b: [1, 1], r: ro } };
  const p = solidProps(s, null);
  const V = Math.PI * (ro * ro - ri * ri) * L;
  const Iu = ((ro * ro + ri * ri) / 2) * V, Ip = ((ro * ro + ri * ri) / 4 + (L * L) / 12) * V;
  // The inertia-tensor off-diagonal element (already the negative of the raw product integral, the accumulator's own
  // sign) for a cylinder of axis unit vector a: I_ij = Ip d_ij + (Iu - Ip) a_i a_j. At 45 deg, a_y = a_z = 1/sqrt(2).
  const expectedIyz = (Iu - Ip) * 0.5;
  // near(), not strict equal: a[0] = 0 exactly, so (Iu - Ip) * a[0] * a[1] can land on -0 rather than +0 (same value).
  near(p.J[3], 0, 1e-20, 'no x-y product for a tube with no x-component to its axis');
  near(p.J[4], 0, 1e-20, 'no x-z product for a tube with no x-component to its axis');
  near(p.J[5], expectedIyz, 1e-18, 'stored y-z product (diagnosis: was +6.47e-10, should be -6.48e-10 in that check)');
  // The diagonal terms (Ixx = Ip, Iyy = Izz = Ip + (Iu - Ip) a_y^2 at 45 deg) and the centroid are untouched by this
  // fix (only the off-diagonal sign changed); pinned too so a mutation there is not missed as a side effect.
  near(p.J[0], Ip, 1e-18, 'Ixx = Ip (the tube has no x-component to its axis)');
  near(p.J[1], Ip + (Iu - Ip) * 0.5, 1e-18, 'Iyy at 45 deg');
  near(p.J[2], Ip + (Iu - Ip) * 0.5, 1e-18, 'Izz at 45 deg');
  near(p.c[0], 0, 1e-18, 'centroid x = xm');
  near(p.c[1], 0.5, 1e-18, 'centroid y = (a[1]+b[1])/2');
  near(p.c[2], 0.5, 1e-18, 'centroid z = (a[0]+b[0])/2');
});

test('solidProps: a prism\'s products are unchanged (a rotated rectangle against an independent rotation-of-axes reference)', () => {
  // A 40 x 20 mm rectangle (z x y), 50 mm long along x, rotated 45 deg in the (z, y) plane. For an axis-aligned
  // rectangle Szz0 = A Wz^2/12, Syy0 = A Wy^2/12, Szy0 = 0; rotating by 45 deg (sin 2*45=1, cos 2*45=0) gives
  // Szy = (Szz0 - Syy0)/2 = A (Wz^2 - Wy^2) / 24 -- a standard rotation-of-axes (Mohr's-circle-style) result for a
  // second-moment tensor, independent of massprops.js's own shoelace polygon integration.
  const Wz = 0.04, Wy = 0.02, Lx = 0.05, th = Math.PI / 4;
  const cz = Math.cos(th), sn = Math.sin(th);
  const rot = ([z, y]) => [z * cz - y * sn, z * sn + y * cz];
  const pts = [[Wz / 2, Wy / 2], [-Wz / 2, Wy / 2], [-Wz / 2, -Wy / 2], [Wz / 2, -Wy / 2]].map(rot);
  const s = { x: [0, Lx], shape: { kind: 'poly', pts } };
  const p = solidProps(s, null);
  const A = Wz * Wy, SzyExpected = (A * (Wz * Wz - Wy * Wy)) / 24;
  near(p.J[3], 0, 1e-20);
  near(p.J[4], 0, 1e-20);
  near(p.J[5], -Lx * SzyExpected, 1e-15, 'prism y-z product (unaffected by the tube fix)');
});
