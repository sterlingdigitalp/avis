// Assembly clearance (SPEC "Design freeze", "Assembly clearance"): the check's geometry against hand calculations,
// one test per v21 clash the SPEC lists (each fails the 0.5 mm rule on v21's own numbers, frozen in
// test/fixtures/larus-gl-2-v21.json), the further clashes the check found, and the v22 layout rule that clears the
// four (hand calculations from the catalogue dimensions and the design's parameters).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { gap2, gap3, ring, cap, poly, polySd, segSegDist, checkClearance, plateJournalX, bossBeyond, CLEARANCE_M } from '../../src/design/clearance.js';
import { driveLayout, driveJoints } from '../../src/design/joints.js';
import { driveMembers, plateSeatLength, rockerHubLength, jointHousings } from '../../src/design/linkage.js';
import { round14Set, round14Note } from '../../src/design/round14.mjs';
import { fittedBuild, recordClearance } from '../../src/design/round11.mjs';
import { drivePose, linkageAt, motorOf, partsIndex } from '../../sims/avis-shared/model.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v21 = () => JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v21.json', import.meta.url), 'utf8'));
const near = (a, b, tol = 1e-6) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);

/** The check on a design with its airframe re-derived from its own parameters (the fixture predates outline_m). */
function run(d) {
  const lay = driveLayout(d, catalog);
  const jp = driveJoints(d, catalog, d.params.joints);
  const lm = driveMembers(d, catalog, d.params.linkage, lay, jp.joints);
  return { lay, r: checkClearance(d, catalog, d.params.linkage, lay, lm.airframe, { all: true }) };
}
const pair = (r, a, b) => {
  const p = r.all_pairs[`${a} | ${b}`] ?? r.all_pairs[`${b} | ${a}`];
  assert.ok(p, `no pair ${a} / ${b}`);
  return p.gap_mm; // rounded to 0.001 mm in the record
};
/**
 * Round 16: a coupler eye is drawn as its body and, at its outer face, the counterbore for its bushing's 1 mm flange
 * (the catalogue's d3 x b2): the eye's gap to a part is the smaller of the two solids'.
 */
const eyePair = (r, a, b) => {
  const g = [pair(r, a, b)];
  for (const [x, y] of [[`${a} counterbore`, b], [a, `${b} counterbore`]]) if (r.all_pairs[`${x} | ${y}`] ?? r.all_pairs[`${y} | ${x}`]) g.push(pair(r, x, y));
  return Math.min(...g);
};
const R3 = 6e-4; // tolerance for a value read from the record
const asV22 = (d) => { d.params.layout = { rule: 'clearance', running_mm: 0.5 }; d.params.linkage.plates.flange = 'face'; return d; };

let V21, V22;
const once = () => (V21 ??= run(v21()));
const once22 = () => (V22 ??= run(asV22(v21())));

test('plane geometry: gaps of rings, capsules and polygons against hand values', () => {
  near(gap2(ring([0, 0], 1), ring([5, 0], 2)), 2); // 5 - 1 - 2
  near(gap2(ring([0, 0], 5, 3), ring([0, 0], 2)), 1); // a disc in an annulus's bore
  near(gap2(ring([0, 0], 3), ring([4, 0], 2)), -1); // overlap 1
  near(gap2(cap([0, 0], [10, 0], 1), ring([5, 4], 1)), 2);
  near(gap2(cap([0, 0], [10, 0], 0.5), cap([0, 3], [10, 3], 0.5)), 2);
  near(gap2(cap([0, -1], [0, 1], 0.5), cap([-1, 0], [1, 0], 0.5)), -1); // crossing bars
  near(segSegDist([0, 0], [1, 0], [2, 1], [3, 1]), Math.SQRT2);
  const sq = [[0, 0], [1, 0], [1, 1], [0, 1]];
  near(polySd(sq, [2, 0.5]), 1);
  near(polySd(sq, [0.5, 0.5]), -0.5);
  near(polySd([...sq].reverse(), [0.5, 0.2]), -0.2); // either winding
  near(gap2(poly(sq), poly([[2, 0], [3, 0], [3, 1], [2, 1]])), 1);
  near(gap2(poly(sq), poly([[0.75, 0], [2, 0], [2, 1], [0.75, 1]])), -0.25);
  near(gap2(cap([-1, 3], [2, 3], 0.5), poly(sq)), 1.5);
  near(gap2(ring([0.5, 0.5], 0.1), poly([[-2, -2], [2, -2], [2, 2], [-2, 2]], [{ c: [0.5, 0.5], r: 0.4 }])), 0.3); // a shaft in a plate's hole
  near(gap3([0, 1], [4, 5], 4), 5); // x gap 3, in-plane gap 4
  near(gap3([0, 1], [3, 5], -1), 2);
  near(gap3([0, 2], [1, 3], -2), -1); // overlapping: the smaller penetration
});

test('v21 clash 1: the two coupler eyes on the crank pin overlap by 1 mm (planes 5 mm apart, eyes b1 6 mm)', () => {
  const { r, lay } = once();
  const b1 = catalog.find((p) => p.id === 'bushing-igus-GFM-0810-06').specs.b1_mm;
  const hand = -(b1 - (lay.planes.couplerR - lay.planes.couplerL) * 1000);
  near(hand, -1, 1e-9);
  near(pair(r, 'coupler R/crank eye', 'coupler L/crank eye'), hand, R3);
  near(pair(r, 'coupler R crank bushing/body', 'coupler L crank bushing/body'), hand, R3);
  assert.ok(hand < CLEARANCE_M * 1000);
  assert.ok(r.clashes.some((c) => /coupler R/.test(c.a + c.b) && /coupler L/.test(c.a + c.b)));
});

test('v21 clash 2: the left rocker-end eye cuts 2.4 mm radially into the left aft shoulder housing, all stroke long', () => {
  const d = v21(), { r } = once();
  const L = d.params.linkage;
  const c = linkageAt(driveLayout(d, catalog).law, d.drive.ampNominal).c;
  const hand = (c * 1000) - L.coupler.eye_od_mm / 2 - L.shoulderHousing.od_mm / 2; // the eye's centre is the rocker length from the housing's
  near(hand, -2.4164, 1e-3);
  assert.ok(hand < CLEARANCE_M * 1000);
  near(eyePair(r, 'coupler L/horn eye', 'shoulder bracket L aft/housing'), hand, R3);
});

test('v21 clash 3: the crank web is 0.2 mm from the aft plate, so the crank bushing\'s 1 mm flange has no room', () => {
  const d = v21(), { r, lay } = once();
  const P = lay.planes, tA = d.params.linkage.plates.t_mm / 1000, tw = d.params.linkage.crank.web_t_mm / 1000;
  const hand = ((P.plateAft - tA / 2) - (P.crankArm + tw / 2)) * 1000; // 4.7 - 4.5 mm
  near(hand, 0.2, 1e-9);
  assert.ok(hand < CLEARANCE_M * 1000, 'under the 0.5 mm running clearance');
  // v21's seats hold the whole part (flange counterbored at the plate's face): 0.2 mm, under the 0.5 mm running clearance
  near(pair(r, 'crank/web', 'crank shaft journal aft (bushing-igus-GFM-1012-10)/flange'), hand, R3);
  // with the flange on the face, as the SPEC reads it, the flange is 0.8 mm into the web
  const dims = { d: 0.01, D: 0.012, B: 0.01, flangeD: 0.018, flangeB: 0.001 };
  const xs = plateJournalX({ flange: 'face' }, 'aft', P.plateAft, tA, dims);
  near((P.crankArm + tw / 2 - xs.flange[0]) * 1000, 0.8, 1e-9);
});

test('v21 clash 4: the aft-plate bosses reach 7 mm into the crank wheel and 3.2 mm into the stage-4 pinion', () => {
  const d = v21(), { r, lay } = once();
  const P = lay.planes, PL = d.params.linkage.plates;
  // crank wheel MSGB1-45, bore 15 mm, on the crank axis; the crank boss 12 + 2 x 8.5 = 29 mm: 14.5 - 7.5 = 7 mm radially
  const wheelGap = pair(r, 'stage 4 wheel gear-khk-msgb1-45/teeth', 'gearbox plate aft/boss crank shaft/aft');
  near(wheelGap, -(12 / 2 + PL.boss_wall_mm - 15 / 2), R3);
  assert.ok(wheelGap < 0);
  // the shaft-1 boss (GFM-0810-06, b1 6 - 1.6 mm plate = 4.4 mm beyond it) against the pinion's face at the stage plane
  const xBoss = P.plateAft + PL.t_mm / 2000 + 0.0044, xFace = P.stage[3] - 0.005;
  near(pair(r, 'stage 4 pinion gear-khk-msga1-18/teeth', 'gearbox plate aft/boss gear shaft 1/aft'), -(xBoss - xFace) * 1000, R3);
});

test('v21: the check also finds the right and left horns, gear shaft 2, the rails and the rocker hubs in each other', () => {
  const d = v21(), { r, lay } = once();
  // gear shaft 2 runs plate to plate through stage 1's DS1-60 (tip 31 mm) at the stage-2 centre distance (40 + 18) / 2
  near(pair(r, 'gear shaft 2/shaft', 'stage 1 wheel gear-khk-ds1-60/teeth'), 29 - 31 - 8 / 2, R3);
  // the rail 4 mm outboard and 11.5 mm above the hinge (12 mm tube) beside the 14.5 mm rocker hub
  near(pair(r, 'rocker R/hub', 'shoulder rail right/tube'), Math.hypot(4, 11.5) - 6 - 7.25, R3);
  // the rocker hub fills the gap between its bushings: no running clearance
  near(pair(r, 'rocker R/hub', 'shoulder bushing R aft/body'), 0, R3);
  // the horn pins come within |B_R - B_L| of each other: the right horn pin (8 mm) in the left horn boss (23 mm)
  let dmin = Infinity;
  for (let i = 0; i < 3600; i++) {
    const q = drivePose({ law: lay.law, fourBar: lay.fb }, (i / 3600) * 2 * Math.PI, d.drive.ampNominal);
    dmin = Math.min(dmin, Math.hypot(q.right.B[0] - q.left.B[0], q.right.B[1] - q.left.B[1]));
  }
  near(pair(r, 'rocker R/horn pin', 'rocker L/horn boss'), dmin * 1000 - 4 - 11.5, 0.02);
  assert.equal(r.passed, false);
});

test('v22 layout: hand-calculated planes from the catalogue and the design (0.5 mm running clearance)', () => {
  const { lay } = once22();
  const P = lay.planes;
  near(P.couplerR * 1000, 1.5 - 3 - 0.5 - 3);
  near(P.couplerL * 1000, -5 - 6 - 0.5);
  near(P.plateAft * 1000, 1.5 + 3 + 0.5 + 1 + 0.8); // web face, run, GFM-1012-10 flange, half the 1.6 mm plate
  near(P.stage[3] * 1000, 6.8 + 0.8 + (10 - 1 - 1.6) + 0.5 + 5); // past the crank boss (body 9 mm less the plate)
  near(P.stage[0] * 1000, 15.5 + (10 + 5 + 1.5) * 3 + 3);
  near(P.plateFwd * 1000, 68 + 3 + 8 + (5 - 1.2) + 0.5 + 0.6); // past the 628/8 (5 mm) in the 1.2 mm forward plate
});

test('v22 layout clears the four clashes and the hub stacks by the 0.5 mm running clearance', () => {
  const { r } = once22();
  near(eyePair(r, 'coupler R/crank eye', 'coupler L/crank eye'), 0.5, R3);
  near(eyePair(r, 'crank/web', 'coupler R/crank eye'), 0.5, R3);
  near(pair(r, 'crank/web', 'crank shaft journal aft (bushing-igus-GFM-1012-10)/flange'), 0.5, R3);
  near(pair(r, 'stage 4 wheel gear-khk-msgb1-45/teeth', 'gearbox plate aft/boss crank shaft/aft'), 0.5, R3);
  near(pair(r, 'rocker R/hub', 'shoulder bushing R aft/body'), 0.5, R3);
  near(pair(r, 'rocker L/hub', 'shoulder bushing L forward/body'), 0.5, R3);
  near(pair(r, 'stage 1 wheel gear-khk-ds1-60/hub', 'gear shaft 3 journal forward (bearing-skf-628-8-2z)/body'), 0.5, R3);
  // clash 2 is not a plane question: the eye and housing radii exceed the rocker length whatever the planes
  assert.ok(eyePair(r, 'coupler L/horn eye', 'shoulder bracket L aft/housing') < 0);
});

test('seat rules under the flange-on-face convention (v22) and v21 unchanged', () => {
  const pl = { t_mm: 1.6, t_fwd_mm: 1.2, bossed: ['crank shaft/aft'] };
  assert.deepEqual(plateSeatLength(pl, 'crank shaft', 'aft', 10, 1), { plate_mm: 1.6, bossed: true, length_mm: 10 });
  assert.deepEqual(plateSeatLength({ ...pl, flange: 'face' }, 'crank shaft', 'aft', 10, 1), { plate_mm: 1.6, bossed: true, length_mm: 9 });
  const dims = { d: 0.01, D: 0.012, B: 0.01, flangeD: 0.018, flangeB: 0.001 };
  near(bossBeyond(pl, 0.0016, dims), 0.0084, 1e-12);
  near(bossBeyond({ flange: 'face' }, 0.0016, dims), 0.0074, 1e-12);
  near(rockerHubLength({ params: {} }, {}, 0.024, 0.006), 0.018, 1e-12);
  near(rockerHubLength({ params: { layout: { rule: 'clearance', running_mm: 0.5 } } }, {}, 0.024, 0.006), 0.017, 1e-12);
  near(rockerHubLength({ params: {} }, { hub_len_mm: 16 }, 0.024, 0.006), 0.016, 1e-12);
  const d = asV22(v21());
  const j = jointHousings(driveJoints(d, catalog, d.params.joints).joints, d.params.linkage, catalog).find((q) => q.name === 'crank shaft');
  assert.deepEqual(j.supports.map((s) => s.housing_mm), [9, 9]); // GFM-1012-10: b1 10 less its 1 mm flange
  const s = round14Set();
  assert.equal(s.layout.rule, 'clearance');
  assert.deepEqual(s.linkage.plates, { edge_mm: 11.3, flange: 'face' });
});

test('the record: v21 fails with the four clashes listed per pair of parts, the sampling stated', () => {
  const { r } = once();
  const has = (a, b) => r.clashes.some((c) => (c.a.startsWith(a) && c.b.startsWith(b)) || (c.a.startsWith(b) && c.b.startsWith(a)));
  assert.ok(has('coupler R/', 'coupler L/'));
  assert.ok(has('crank/', 'crank shaft journal aft')); // led by the wheel seat in the bushing (-1 mm), the web 0.2 mm from it
  assert.ok(has('stage 4 wheel', 'gearbox plate aft/boss'));
  assert.ok(r.clashes.some((c) => /coupler L/.test(c.a + c.b) && /shoulder bracket L aft/.test(c.a + c.b)));
  assert.equal(r.sampling.crank_phases_per_amplitude, 360);
  assert.equal(r.required_mm, 0.5);
  for (let i = 1; i < r.clashes.length; i++) assert.ok(r.clashes[i].gap_mm >= r.clashes[i - 1].gap_mm);
});

test('the build records the check: fittedBuild on v21\'s own parameters writes structure.clearance, failed', () => {
  const { d } = fittedBuild(v21().params, catalog, false);
  const c = d.structure.clearance;
  assert.equal(c.passed, false);
  assert.ok(c.clashes.some((q) => /coupler R/.test(q.a + q.b) && /coupler L/.test(q.a + q.b)));
  assert.ok(Array.isArray(d.structure.airframe.parts.plates.aft.outline_m) && d.structure.airframe.parts.plates.aft.outline_m.length > 3);
  assert.equal(c.pairs_checked + c.pairs_exempt, (c.solids * (c.solids - 1)) / 2);
});

test('the round-14 assumption line states the check\'s verdict', () => {
  const lay = { running_mm: 0.5 };
  assert.match(round14Note(lay, { passed: true, clashes: [], required_mm: 0.5 }), /check passed/);
  assert.match(round14Note(lay, { passed: false, clashes: [{}, {}], required_mm: 0.5, pod_shell: { clashes: ['packed battery'] } }), /FAILED: 2 part pairs under 0.5 mm, 1 against the pod/);
  assert.match(round14Note(lay, null), /check not run/);
});

test('a design without a linkage gets no clearance record', () => {
  const d = { structure: {} };
  assert.equal(recordClearance(d, { joints: {} }, catalog).structure.clearance, undefined);
  assert.equal(recordClearance(d, { linkage: {} }, catalog).structure.clearance, undefined);
});

test('the gear fold reads the motor from the catalogue ("37.5 dia x 33.75" mm) and its lump, else 35 x 40 mm at x 40 mm', () => {
  const d = v21();
  const m = motorOf(d, partsIndex(d, catalog));
  near(m.md[0], 0.0375, 1e-12); near(m.md[1], 0.03375, 1e-12);
  assert.deepEqual(m.lumpM, d.mass.lumps.find((l) => l.name === 'flap motor').pos);
  assert.deepEqual(motorOf({ params: {}, mass: { lumps: [] } }, {}), { md: [0.035, 0.04], lumpM: [0.04, 0, 0] });
  assert.deepEqual(motorOf({ params: { parts: { motor: 'x' } }, mass: { lumps: [] } }, { x: { specs: { dimensions_text_mm: 'none' } } }).md, [0.035, 0.04]);
});

// ---- Round 14: the layout that assembles, and the check's fixes ------------------------------------------------
import { hornStack, sleeveX, lockFlangeShape, MESH } from '../../src/design/clearance.js';
import { rockerHub } from '../../src/design/linkage.js';
import { ROUND14 } from '../../src/design/round14.mjs';
import { encoderDims, journalDims } from '../../sims/avis-shared/model.js';
import { protrusions } from '../../src/design/fit.js';
import { lockFlangeCircles, hullOf } from '../../src/design/airframe.js';

/** v21's own parameters with round 14's layout, joints, parts and linkage blocks (as round14.mjs builds them). */
function asR14(d) {
  const S = ROUND14.set, p = d.params, A = p.linkage.airframe;
  p.layout = { ...S.layout };
  p.joints = { ...p.joints, ...S.joints };
  p.parts = { ...p.parts, ...S.parts };
  const L = S.linkage;
  p.linkage = { ...p.linkage, plates: { ...p.linkage.plates, ...L.plates }, rocker: { ...p.linkage.rocker, ...L.rocker }, airframe: { ...A, ...L.airframe, plates: { ...A.plates, ...L.airframe.plates }, rail: { ...A.rail, ...L.airframe.rail } } };
  d.bom.parts.find((q) => q.role === 'crank encoder magnet').id = S.parts.encoderMagnet; // as buildDesign lists it
  return d;
}
let V14;
const once14 = () => (V14 ??= run(asR14(v21())));

test('round 14 planes by hand: eyes, web, plates, stages, overhung stage 1, the motor inside, the encoder stub (mm)', () => {
  const { lay } = once14();
  const P = lay.planes, mmv = (v) => v * 1000;
  // web centre 8.7, 6 mm thick; right eye 0.5 aft of it (6 mm GFM-0810-06), left eye 0.5 aft of that
  near(mmv(P.crankArm), 8.7); near(mmv(P.couplerR), 8.7 - 3 - 0.5 - 3); near(mmv(P.couplerL), 2.2 - 6 - 0.5);
  // aft plate past the 1 mm flange of GFM-1012-10, half its 1.6 mm
  near(mmv(P.plateAft), 11.7 + 0.5 + 1 + 0.8);
  // crank stage past the crank boss (body 9 mm less the plate), MSGB1-45 face 10, hub 5; 1.5 mm between stages
  near(mmv(P.stage[3]), 14.8 + 7.4 + 0.5 + 5); near(mmv(P.stage[2]), 37.7 + 1.5 + 5); near(mmv(P.stage[1]), 54.2 + 1.5 + 5);
  // forward plate past stage 2's hub and the forward journals (5 mm less the 1.2 mm plate), half the plate
  near(mmv(P.plateFwd), 70.7 + 3.8 + 0.5 + 0.6);
  // stage 1 past the 619/5's outward 4 mm (from the inner face) and the running gap; the motor 1.5 mm aft of the plate
  near(mmv(P.stage[0]), 75.6 - 0.6 + 4 + 0.5 + 3); assert.equal(P.stage1, 'overhung');
  near(mmv(P.motorFace), 75.6 - 1 - 1.5); near(mmv(P.motorX[0]), 73.1 - 33.75);
  // the encoder forward of stage 1's 6 mm teeth: BR10 6.9 mm, ride 4 mm, BA100 9.5 mm; the crank shaft to its end
  near(mmv(P.encoder.readhead[0]), 82.5 + 3 + 0.5); near(mmv(P.encoder.readhead[1]), 92.9); near(mmv(P.encoder.actuator[0]), 96.9); near(mmv(P.encoder.actuator[1]), 106.4);
  near(mmv(P.crankEnd), 106.4);
});

test('round 14 horn stacking and bracket sleeves, by hand', () => {
  const Ly = ROUND14.set.layout;
  const L = hornStack(Ly, 'L', -0.0043, 0.006, 0.007), R = hornStack(Ly, 'R', 0.0022, 0.006, 0.007);
  [[L.boss, [-14.8, -7.8]], [L.pin, [-14.8, -1.3]], [R.boss, [5.7, 12.7]], [R.pin, [-0.8, 12.7]]].forEach(([a, b]) => { near(a[0] * 1000, b[0], 1e-9); near(a[1] * 1000, b[1], 1e-9); });
  // v21: boss 1 mm forward of the eye, the pin to 1 mm past it
  const v = hornStack({}, 'R', -0.0045, 0.006, 0.008);
  near(v.boss[0] * 1000, -0.5, 1e-9); near(v.boss[1] * 1000, 7.5, 1e-9); near(v.pin[0] * 1000, -8.5, 1e-9);
  // a sleeve centred on a housing forward of the aft plate would sit in the plate: it starts 0.5 mm clear of it instead
  const ax = { P: { plateAft: 0.014, plateFwd: 0.0965 }, tA: 0.0016, tF: 0.0012, Ly };
  const f = sleeveX(ax, 0.0183, 0.013), a = sleeveX(ax, -0.0183, 0.013);
  near(f[0] * 1000, 15.3, 1e-9); near(f[1] * 1000, 28.3, 1e-9); near(a[0] * 1000, -24.8, 1e-9); near(a[1] * 1000, -11.8, 1e-9);
});

test('round 14: the left eye clears the left aft housing by stacking along x (clash 2), the horns clear each other', () => {
  const { r } = once14();
  // the left eye (-7.3 to -1.3 mm) 8 mm forward of the left aft housing's counterbore (-16.3 to -15.3 mm)
  near(eyePair(r, 'coupler L/horn eye', 'shoulder bracket L aft/housing counterbore'), 8, R3);
  assert.ok(eyePair(r, 'coupler R/horn eye', 'rocker L/horn boss') >= 0.5);
  assert.ok(eyePair(r, 'rocker R/horn pin', 'coupler L/horn eye') >= 0.5);
  near(pair(r, 'rocker L/horn boss', 'shoulder bracket L aft/housing counterbore'), 0.5, R3); // boss from -14.8 mm
  // the hub turned down to 13.6 mm where the eyes pass: the eye's inner edge is the rocker length less 12.25 mm from
  // the axis, 0.53 mm off the relief
  const c = linkageAt(driveLayout(asR14(v21()), catalog).law, v21().drive.ampNominal).c * 1000;
  near(eyePair(r, 'rocker R/hub relief', 'coupler R/horn eye'), c - 24.5 / 2 - 13.6 / 2, 0.02);
});

test('round 14 rocker hub by hand: relief at the eyes, forward end clear of the aft plate, the seat at full diameter', () => {
  const d = asR14(v21());
  const { lay } = once14();
  const h = rockerHub(d, d.params.linkage.rocker, 0.0366, 0.006, lay.planes, 0.006);
  // 36.6 - 6 - 2 x 0.5 = 29.6 mm centred on the hinge; forward end 0.5 mm aft of the plate's aft face (14.0 - 0.8)
  near(h.x[0] * 1000, -14.8, 1e-9); near(h.x[1] * 1000, 12.7, 1e-9);
  // relief from 0.5 mm aft of the left eye (-7.3) to 0.5 mm forward of the right eye (5.2)
  near(h.relief[0] * 1000, -7.8, 1e-9); near(h.relief[1] * 1000, 5.7, 1e-9); near(h.seat_m * 1000, 27.5 - 13.5, 1e-9);
  // the shoulder pin's arms: bushing centres at -18.3 and 18.3 mm
  near(h.levers.aft * 1000, 3.5, 1e-9); near(h.levers.forward * 1000, 5.6, 1e-9);
  // the pin's collar from the hub's face to 0.5 mm short of the forward bushing's flange (18.3 - 3 = 15.3 mm)
  near(h.collar[0] * 1000, 12.7, 1e-9); near(h.collar[1] * 1000, 14.8, 1e-9);
  const { r } = once14();
  near(pair(r, 'rocker R/pin collar', 'shoulder bushing R forward/flange'), 0.5, R3);
  assert.ok(pair(r, 'rocker R/pin collar', 'gearbox plate aft/plate') >= 0.5 - R3);
  const hub = once14().lay && run(asR14(v21())); // members on the round-14 layout
  const lm = driveMembers(asR14(v21()), catalog, asR14(v21()).params.linkage, lay, driveJoints(asR14(v21()), catalog, asR14(v21()).params.joints).joints);
  const m = lm.members.find((q) => q.name === 'rocker hub right');
  near(m.dims.t_mm, 14, 1e-9); assert.ok(m.dims.e_over_d >= 1.2);
  assert.ok(hub.r.passed !== undefined);
});

test('round 14: joined parts are checked at zero gap and never interpenetrate; gears in mesh are exempt', () => {
  const { r } = once14();
  const rows = Object.values(r.all_pairs);
  const joined = rows.filter((q) => q.joined);
  assert.ok(joined.length > 50);
  for (const q of joined) assert.ok(q.gap_mm >= -1e-4, `${q.a} / ${q.b} ${q.gap_mm}`);
  assert.ok(!rows.some((q) => q.joined === MESH));
  assert.ok(!rows.some((q) => /^stage 4 wheel/.test(q.a) && /^stage 4 pinion/.test(q.b)));
  // a bushing flange sits in its housing's counterbore, touching; a bracket arm stops at the rail's surface
  near(pair(r, 'shoulder bushing R aft/flange', 'shoulder bracket R aft/housing counterbore'), 0, R3);
  near(pair(r, 'shoulder bracket R aft/arm', 'shoulder rail right/tube'), 0, R3);
});

test('round 14: zero clashes through the stroke on v21\'s own parameters with the round-14 layout', () => {
  const { r } = once14();
  assert.deepEqual(r.clashes, []);
  assert.equal(r.required_mm, 0.5);
});

test('the boom passes the plates and posts through a running-clearance hole; the clamp blocks are bored for it', () => {
  const { r } = once14();
  const rows = Object.values(r.all_pairs).filter((q) => /^tail boom\/tube$/.test(q.a) || /^tail boom\/tube$/.test(q.b));
  const plate = rows.filter((q) => /gearbox plate/.test(q.a + q.b));
  assert.ok(plate.length >= 4);
  for (const q of plate) assert.ok(q.gap_mm >= 0.5 - R3);
  near(Math.min(...plate.map((q) => q.gap_mm)), 0.5, R3);
  const clamps = rows.filter((x) => /^boom clamp/.test(x.a) || /^boom clamp/.test(x.b));
  assert.equal(clamps.length, 2);
  for (const q of clamps) near(q.gap_mm, 0, R3);
});

test('round 14 parts: the encoder\'s catalogue sizes, the outward motor bearing, the lock flange, the strict shell fit', () => {
  const d = asR14(v21());
  const e = encoderDims(d, partsIndex(d, catalog));
  assert.deepEqual([e.readhead_od, e.readhead_id, e.readhead_t, e.ride, e.actuator_od, e.actuator_id, e.actuator_h].map((v) => +(v * 1000).toFixed(3)), [45, 15.8, 6.9, 4, 22, 10, 9.5]);
  // the actuator's bore is the crank shaft's diameter (the 8 mm one it replaces was not)
  assert.equal(catalog.find((p) => p.id === 'sensor-rls-orbis-actuator-10mm').specs.inner_diameter_mm, d.params.linkage.crank.shaft_od_mm);
  assert.equal(catalog.find((p) => p.id === 'sensor-rls-orbis-actuator-8mm').specs.inner_diameter_mm, 8);
  assert.ok(Object.values(encoderDims({ params: {} }, {})).every((v) => v === 0)); // no encoder in the design: nothing placed
  const dims = journalDims(catalog.find((p) => p.id === 'bearing-skf-619-5-2z').specs);
  const x = plateJournalX({ flange: 'face' }, 'fwd', 0.0965, 0.0012, dims, true);
  near(x.body[0] * 1000, 95.9, 1e-9); near(x.body[1] * 1000, 99.9, 1e-9);
  // six M3 lands on a 15 mm arc from 40 to 172 degrees (6.5 mm or more apart, an M3 head 5.5 mm): by hand
  const cs = lockFlangeCircles({ ...d.params.linkage.airframe.rail, ...ROUND14.set.linkage.airframe.rail }, [0, 0], 1);
  assert.equal(cs.length, 7); near(cs[1].c[0] * 1000, 15 * Math.cos((40 * Math.PI) / 180), 1e-9); near(cs[6].c[0] * 1000, 15 * Math.cos((172 * Math.PI) / 180), 1e-9);
  for (let i = 2; i < 7; i++) assert.ok(Math.hypot(cs[i].c[0] - cs[i - 1].c[0], cs[i].c[1] - cs[i - 1].c[1]) >= 0.0065);
  assert.ok(hullOf(cs).area_m2 > 0);
  assert.ok(lockFlangeShape({ ...ROUND14.set.linkage.airframe.rail }, [0.0265, 0.001], 1, 0.0065).kind === 'poly');
  const fit = { issues: [{ mm: 0.3 }, { mm: 0 }] };
  assert.equal(protrusions(fit, d).length, 1); assert.equal(protrusions(fit, v21()).length, 0);
});

test('plane geometry, mutation follow-up: asymmetric crossings, scaled and small polygons, corners, V-shaped minima', () => {
  // two bars crossing off-centre: the crossing test must see it (endpoint distances alone give 0.2)
  near(segSegDist([0, 0], [4, 1], [1, -1], [1.5, 3]), 0);
  near(gap2(cap([0, 0], [4, 1], 0.1), cap([1, -1], [1.5, 3], 0.1)), -0.2);
  // apart, no crossing: the nearest endpoint to the other segment
  near(segSegDist([0, 0], [1, 0], [2, -1], [2, 3]), 1);
  // a 3 x 2 box: depth inside is scaled by the edge lengths, and outside a corner it is the corner distance
  const box = [[0, 0], [3, 0], [3, 2], [0, 2]];
  near(polySd(box, [1.5, 0.5]), -0.5); near(polySd(box, [2.5, 1]), -0.5);
  near(polySd(box, [4, 3]), Math.SQRT2); near(polySd(box, [1, -0.25]), 0.25);
  // a millimetre-sized polygon of either winding (the check's own scale): its sign comes from its area alone
  const small = [[0, 0], [0.003, 0], [0.003, 0.002], [0, 0.002]];
  near(polySd([...small].reverse(), [0.0015, 0.0005]), -0.0005, 1e-12); near(polySd(small, [0.0015, 0.0005]), -0.0005, 1e-12);
  // a bar passing a box's corner at a slant: the nearest point lies inside the bar, found by the golden-section search
  // (the bar's line x + y = 2 passes sqrt(2) from the corner (2, 2), its nearest point the bar's middle)
  near(gap2(cap([-1, 3], [3, -1], 0.1), poly(box)), -1.1, 1e-6); // through the 3 x 2 box: deepest at (1, 1), 1 from the edges, less r
  near(gap2(cap([-1, 3], [3, -1], 0.1), poly([[2, 2], [3, 2], [3, 3], [2, 3]])), Math.hypot(1, 1) - 0.1, 1e-6);
  // gaps of prisms: touching in x and overlapping in the plane is zero, not a clearance
  near(gap3([0, 1], [1, 2], -1), 0); near(gap3([0, 1], [1.5, 2], 0), 0.5); near(gap3([0, 1], [0.5, 2], 0.3), 0.3);
  // a ring in a polygon's hole, and a ring whose bore holds a capsule or a polygon
  near(gap2(poly(box, [{ c: [1.5, 1], r: 0.5 }]), ring([1.5, 1], 0.3)), 0.2);
  near(gap2(cap([1.4, 1], [1.6, 1], 0.1), ring([1.5, 1], 2, 0.5)), 0.5 - 0.1 - 0.1);
  near(gap2(poly([[1.4, 0.9], [1.6, 0.9], [1.6, 1.1], [1.4, 1.1]]), ring([1.5, 1], 2, 0.5)), 0.5 - Math.hypot(0.1, 0.1));
});

test('the round-14 assumption line states the layout it describes, only for the round-14 layout', () => {
  const line = round14Note(ROUND14.set.layout, { passed: true, clashes: [], required_mm: 0.5 });
  assert.match(line, /horn stacks R fwd, L aft/); assert.match(line, /stage 1 overhung/);
  assert.match(line, new RegExp(JSON.stringify(ROUND14.set.layout.links_deg).replace(/[[\]]/g, '\\$&')));
  assert.doesNotMatch(round14Note({ running_mm: 0.5 }, { passed: true, clashes: [], required_mm: 0.5 }), /overhung/);
});
