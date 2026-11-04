// Round 17b (module "retain"): the coupler-eye pins' retaining washers and the eye gap that makes room for them, the
// plate bosses as long as their parts (the outward-seat boss bug), the spacer compression check, the washer rating, and
// the drive's interface rules. Each refusal is pinned against a deliberately broken, hand-built input.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { pinEndGroove, plateJournalX, bossSpan, plateBoss, bossBeyond } from '../../src/design/clearance.js';
import { pinWasher, crankPinEnd, hornStack, rockerArm, washerRating } from '../../src/design/linkage.js';
import { ringCompressionCheck, checkMembers } from '../../src/physics/members.js';
import { interfaceLedger, RETAIN_INTERFACE_RULES } from '../../src/design/interfaces.js';
import { RETENTION } from '../../src/design/round17-retention.mjs';

const near = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);
const W6 = { groove_width_m_mm: 0.74, min_groove_to_shaft_end_n_mm: 1.2, thickness_s_mm: 0.7, outer_diameter_d3_mm: 12.3, groove_diameter_d2_mm: 6, load_columns: { FN_kN: 1.1, Fs_kN: 1.35, Fsg_kN: 0.7 } };
const cat = [{ id: 'washer-din6799-6', specs: W6 }];
const Ret = { washer8mm: 'washer-din6799-6', hornPinWasher: true, crankPinWasher: false };

test('a pin washer: its groove and land past the eye, read off the catalogue; off unless asked for', () => {
  const pw = pinWasher(cat, Ret, 'horn');
  near(pw.past * 1000, 1.94); near(pw.ro * 1000, 6.15); near(pw.ri * 1000, 3);
  assert.equal(pinWasher(cat, Ret, 'crank'), null);
  assert.equal(pinWasher(cat, { ...Ret, washer8mm: null }, 'horn'), null);
  // the groove against the eye's face (so the washer bears on it), the land beyond to the tip
  const g = pinEndGroove(-0.0008, -1, pw);
  near(g.groove[0] * 1000, -1.54); near(g.groove[1] * 1000, -0.8); near(g.tip[0] * 1000, -2.74); near(g.tip[1] * 1000, -1.54);
  near(g.washer[0] * 1000, -1.5); near(g.washer[1] * 1000, -0.8); near(g.end * 1000, -2.74);
  const f = pinEndGroove(0.001, 1, pw);
  near(f.groove[0] * 1000, 1); near(f.tip[1] * 1000, 2.94);
  // the crank pin: 1 mm past the left eye without a washer, the land with one
  near(crankPinEnd({ couplerL: -0.0043 }, 0.006, null) * 1000, -8.3);
  near(crankPinEnd({ couplerL: -0.0043 }, 0.006, pw) * 1000, -9.24);
});

test('the eye gap is the horn washer\'s groove and land plus the running clearance', () => {
  // both horn pins' tips point into the gap between the eyes (each boss on its eye's outer side), and each tip must
  // clear the other coupler's eye, which overlaps it in the plane, by the running clearance
  assert.equal(RETENTION.linkage.retention.hornPinWasher, true);
  near(RETENTION.layout.eye_gap_mm, W6.groove_width_m_mm + W6.min_groove_to_shaft_end_n_mm + 0.5, 1e-12);
});

test('a horn boss kept within the hub, and the arm within both, no thicker than they overlap', () => {
  const Ly = { horn_gap_mm: 0.5, horns: { R: 'fwd', L: 'aft' } };
  // the left eye moved aft by the wider eye gap: its 7 mm boss would reach 1.54 mm past the hub's aft face
  const H = hornStack(Ly, 'L', -0.00624, 0.006, 0.007, 0.00194, [-0.0152, 0.0127]);
  near(H.boss[0] * 1000, -15.2); near(H.boss[1] * 1000, -9.74); near(H.pin[0] * 1000, -15.2); near(H.pin[1] * 1000, -1.3);
  const a = rockerArm(H, [-0.0152, 0.0127], 0.006);
  near(a.t * 1000, 5.46); near(a.x[0] * 1000, -15.2); near(a.x[1] * 1000, -9.74);
  // without the hub the v21 behaviour (the boss its full length)
  const v = hornStack(Ly, 'L', -0.0043, 0.006, 0.007);
  near(v.boss[0] * 1000, -14.8); near(v.boss[1] * 1000, -7.8);
  // an arm that fits is left centred on its boss
  const R = hornStack(Ly, 'R', 0.0022, 0.006, 0.007, null, [-0.0152, 0.0127]), ar = rockerArm(R, [-0.0152, 0.0127], 0.006);
  near(ar.t * 1000, 6); near((ar.x[0] + ar.x[1]) * 500, 9.2);
});

test('a boss covers the journal body beyond its plate, on the side the body runs (the outward-seat bug)', () => {
  const plates = { flange: 'face' }, dims = { d: 0.005, D: 0.013, B: 0.004, flangeD: 0, flangeB: 0 };
  // the forward plate at x 0.1, 1.2 mm thick; an ordinary bearing seat runs inward (aft) from the outer face
  const xin = plateJournalX(plates, 'fwd', 0.1, 0.0012, dims);
  const bin = bossSpan(xin.body, 0.1, 0.0012);
  near(bin[0] * 1000, 96.6); near(bin[1] * 1000, 99.4);
  // the motor's outboard bearing is seated from the inner face outward: its boss must grow forward, not into the can
  const xout = plateJournalX(plates, 'fwd', 0.1, 0.0012, dims, true);
  const bout = bossSpan(xout.body, 0.1, 0.0012);
  near(bout[0] * 1000, 100.6); near(bout[1] * 1000, 103.4);
  near((bout[1] - bout[0]), bossBeyond(plates, 0.0012, dims), 1e-12);
  // a body no longer than the plate has no boss
  assert.equal(bossSpan([0.0994, 0.1006], 0.1, 0.0012), null);
  // bossed seats: by 'joint/support', the motor's under its single support name 'plate'; a seat's own wall wins
  const PL = { bossed: ['gear shaft 3/aft', 'motor shaft outboard/plate'], boss_wall_mm: 8.5, boss_walls_mm: { 'gear shaft 3/aft': 1.5 } };
  near(plateBoss(PL, 'gear shaft 3', 'aft').wall * 1000, 1.5);
  near(plateBoss(PL, 'motor shaft outboard', 'forward').wall * 1000, 8.5);
  assert.equal(plateBoss(PL, 'gear shaft 3', 'forward'), null);
});

test('a spacer in compression: rated against its design yield, refused without one', () => {
  const r = ringCompressionCheck({ D_mm: 12, d_mm: 8 }, { yield_MPa: 240 }, 100);
  near(r.stress_MPa, 100 / (Math.PI / 4 * 80), 1e-9); near(r.fos, 240 / r.stress_MPa, 1e-9); assert.equal(r.ok, true);
  const bad = ringCompressionCheck({ D_mm: 12, d_mm: 8 }, { yield_MPa: null }, 100);
  assert.equal(bad.fos, null); assert.equal(bad.ok, false);
  assert.equal(ringCompressionCheck({ D_mm: 8, d_mm: 8 }, { yield_MPa: 240 }, 100).ok, false);
  // a light load (under 1 MPa) is still a finite FoS; no load, an infinite one
  near(ringCompressionCheck({ D_mm: 12, d_mm: 8 }, { yield_MPa: 240 }, 10).fos, 240 / (10 / (Math.PI / 4 * 80)), 1e-6);
  assert.equal(ringCompressionCheck({ D_mm: 12, d_mm: 8 }, { yield_MPa: 240 }, 0).fos, Infinity);
  const m = checkMembers([{ name: 's', check: 'ring compression', material: { id: 'x', yield_MPa: 240 }, dims: { D_mm: 12, d_mm: 8 }, uses: { F: 'F' } }], { 's/F': 30000 });
  assert.equal(m.ok, false);
});

test('a DIN 6799 washer is rated at the lowest of its table\'s load columns', () => {
  const r = washerRating('washer-din6799-6', W6, 'metal-al-7075-t6');
  assert.equal(r.rating_N, 700);
  assert.match(r.rating_basis, /Fsg 0.7 kN/);
  assert.equal(washerRating('w', { load_columns: { FN_kN: 1.1, Fs_kN: 1.35 } }, 'm').rating_N, 1100);
});

test('the retain rules: a gear shaft is held only when a rated member stops it each way', () => {
  // round 17b (retain2): no spacer tubes; the shaft between its two end rings at the bushings' flanges
  const members = ['gear shaft 1 washer aft', 'gear shaft 1 washer forward', 'gear shaft 1 bushing flanges, axial thrust'].map((name) => ({ name, check: 'rated' }));
  const d = { params: {}, structure: { members, joints: [{ name: 'gear shaft 1' }] } };
  const joins = [['gear shaft 1 journal aft (bushing-igus-GFM-0810-06)', 'gearbox plate aft', 'pressed in its plate seat'], ['gear shaft 1 journal aft (bushing-igus-GFM-0810-06)', 'gear shaft 1/shaft', 'journal on its shaft']];
  const ok = interfaceLedger({ parts: [], joins }, d, RETAIN_INTERFACE_RULES);
  assert.equal(ok.failed.length, 0, JSON.stringify(ok.failed.map((r) => r.why_failed)));
  // broken: the forward ring gone (as on gear shaft 2 in round 17): one way rests on a press fit, so both rows fail
  const d2 = { ...d, structure: { ...d.structure, members: members.filter((m) => m.name !== 'gear shaft 1 washer forward') } };
  const bad = interfaceLedger({ parts: [], joins }, d2, RETAIN_INTERFACE_RULES);
  assert.equal(bad.failed.length, 2);
  assert.ok(bad.failed.every((r) => /axial retention not rated/.test(r.why_failed)));
  // a circlip at that end serves as well as a washer
  const d3 = { ...d, structure: { ...d.structure, members: [...d2.structure.members, { name: 'gear shaft 1 circlip forward', check: 'rated' }] } };
  assert.equal(interfaceLedger({ parts: [], joins }, d3, RETAIN_INTERFACE_RULES).failed.length, 0);
  // a crank-shaft journal: held only with every member of its two stops named
  const cj = [['crank shaft journal forward (bushing-igus-GFM-1012-10)', 'crank/shaft', 'journal on its shaft']];
  const dc = { ...d, structure: { ...d.structure, joints: [{ name: 'crank shaft' }] } };
  assert.equal(interfaceLedger({ parts: [], joins: cj }, dc, RETAIN_INTERFACE_RULES).failed.length, 1);
  const crank = ['crank shaft circlip forward', 'crank shaft bushing flanges, axial thrust', 'crank web axial screw', 'crank web end lip under the screw head'];
  assert.equal(interfaceLedger({ parts: [], joins: cj }, { ...dc, structure: { ...dc.structure, members: crank.map((name) => ({ name, check: 'rated' })) } }, RETAIN_INTERFACE_RULES).failed.length, 0);
  assert.equal(interfaceLedger({ parts: [], joins: cj }, { ...dc, structure: { ...dc.structure, members: crank.slice(1).map((name) => ({ name, check: 'rated' })) } }, RETAIN_INTERFACE_RULES).failed.length, 1);
});

test('the retain rules: a pin washer and its eye bushing are rated by the members the design names', () => {
  const names = ['horn pin washer right', "horn pin right at its washer's groove", "horn pin washer right on the bushing's end", 'coupler eye right bushing flanges, axial thrust'];
  const d = { params: {}, structure: { members: names.map((name) => ({ name, check: 'rated' })), joints: [{ name: 'coupler eye right, rocker horn' }] } };
  const joins = [['horn pin washer right', 'rocker R', 'in its groove, cut to its own published diameter'], ['horn pin washer right', 'coupler R', "against the eye's free face"], ['coupler R horn bushing', 'coupler R', 'pressed in the eye'], ['coupler R horn bushing', 'rocker R/horn pin', 'journal on its pin']];
  const L = interfaceLedger({ parts: [], joins }, d, RETAIN_INTERFACE_RULES);
  assert.equal(L.failed.length, 0, JSON.stringify(L.failed.map((r) => r.why_failed)));
  // broken: the washer's member missing from the design: every row naming it fails
  const d2 = { ...d, structure: { ...d.structure, members: d.structure.members.filter((m) => m.name !== 'horn pin washer right') } };
  assert.equal(interfaceLedger({ parts: [], joins }, d2, RETAIN_INTERFACE_RULES).failed.length, 4);
});

test('the round-17b retention set reads its washer from the catalogue at the size it is tabulated for', () => {
  const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
  const list = Array.isArray(catalog) ? catalog : catalog.parts;
  const w = list.find((p) => p.id === RETENTION.linkage.retention.washer8mm);
  assert.ok(w, 'washer-din6799-6 in the catalogue');
  near(RETENTION.layout.eye_gap_mm, w.specs.groove_width_m_mm + w.specs.min_groove_to_shaft_end_n_mm + 0.5, 1e-12);
});

test('on v24 with the round-17b retention: the pins carry their washers, the eyes clear, every seat bossed', async () => {
  const { fullAssembly } = await import('../../src/design/clearance.js');
  const { seatWidths } = await import('../../src/design/interfaces.js');
  const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
  const d = JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
  const R = RETENTION.linkage;
  d.params = { ...d.params, layout: { ...d.params.layout, ...RETENTION.layout }, linkage: { ...d.params.linkage, retention: R.retention, plates: { ...d.params.linkage.plates, ...R.plates }, rocker: { ...d.params.linkage.rocker, ...R.rocker } } };
  const { asm } = fullAssembly(d, catalog);
  const part = (id) => asm.parts.find((p) => p.id === id), sol = (id, nm) => part(id)?.solids.find((s) => s.name === nm);
  const mmx = (s) => s.x.map((v) => +(v * 1000).toFixed(2));
  // the crank pin: groove against the left eye's free face, its land beyond, the washer in the groove
  const eyeL = sol('coupler L', 'crank eye counterbore'), cw = sol('crank pin washer', 'washer');
  assert.ok(cw && sol('crank web', 'pin groove') && sol('crank web', 'pin tip'));
  near(cw.x[1], eyeL.x[0], 1e-12); near(cw.x[1] - cw.x[0], 0.0007, 1e-12);
  near(sol('crank web', 'pin groove').x[1], eyeL.x[0], 1e-12); near(sol('crank web', 'pin tip').x[0], eyeL.x[0] - 0.00194, 1e-12);
  // each horn pin's tip clears the other coupler's eye by the running clearance
  const tipR = sol('rocker R', 'horn pin tip'), tipL = sol('rocker L', 'horn pin tip');
  const eR = sol('coupler R', 'horn eye'), eL = sol('coupler L', 'horn eye');
  near(tipR.x[0] - eL.x[1], 0.0005, 1e-9); near(eR.x[0] - tipL.x[1], 0.0005, 1e-9);
  assert.deepEqual(mmx(sol('horn pin washer right', 'washer')), [eR.x[0] * 1000 - 0.7, eR.x[0] * 1000].map((v) => +v.toFixed(2)));
  // the left boss cut to the hub, the arm within it; the joins the ledger reads
  const hub = sol('rocker L', 'hub'), boss = sol('rocker L', 'horn boss'), arm = sol('rocker L', 'arm');
  near(boss.x[0], hub.x[0], 1e-12); assert.ok(arm.x[0] >= hub.x[0] - 1e-12 && arm.x[1] <= boss.x[1] + 1e-12);
  for (const [a, b] of [['crank pin washer', 'crank web'], ['horn pin washer right', 'rocker R'], ['horn pin washer left', 'coupler L horn bushing']]) assert.ok(asm.joins.some((j) => j[0] === a && j[1] === b), `${a} | ${b}`);
  // every plate seat is as wide as its part, the motor's outboard boss forward of the forward plate
  assert.deepEqual(seatWidths(asm), []);
  const mb = sol('gearbox plate forward', 'boss motor shaft outboard/forward'), plF = sol('gearbox plate forward', 'plate');
  near(mb.x[0], plF.x[1], 1e-12); near(mb.x[1] - mb.x[0], 0.0028, 1e-9);
  const g3 = sol('gearbox plate aft', 'boss gear shaft 3/aft'), plA = sol('gearbox plate aft', 'plate');
  near(g3.x[0], plA.x[1], 1e-12); near(g3.shape.ro - g3.shape.ri, 0.0015, 1e-12);
});

test('mutation gaps: the checks and helpers at their edges', async () => {
  const { hornSide, pinWasherMembers } = await import('../../src/design/linkage.js');
  const { retentionContainment } = await import('../../src/design/round17-retention.mjs');
  // ring compression: no material, a sub-mm2 annulus, and the reason it names
  assert.equal(ringCompressionCheck({ D_mm: 12, d_mm: 8 }, null, 10).ok, false);
  const tiny = ringCompressionCheck({ D_mm: 1, d_mm: 0.5 }, { yield_MPa: 240 }, 0.01);
  near(tiny.stress_MPa, 0.01 / (Math.PI / 4 * 0.75), 1e-12); assert.equal(tiny.ok, true);
  assert.equal(ringCompressionCheck({ D_mm: 8, d_mm: 8 }, { yield_MPa: 240 }, 1).why, 'no annulus');
  assert.equal(ringCompressionCheck({ D_mm: 12, d_mm: 8 }, {}, 1).why, 'no design yield');
  // a horn stack the hub does not clip, both ways, and an arm that has to move to stay within the hub
  const Ly = { horn_gap_mm: 0.5, horns: { L: 'aft' } }, wide = [-1, 1];
  const L1 = hornStack(Ly, 'L', 0, 0.006, 0.007, 0.002, wide), R1 = hornStack(Ly, 'R', 0, 0.006, 0.007, 0.002, wide);
  near(L1.boss[0] * 1000, -10.5); near(L1.boss[1] * 1000, -3.5); near(L1.pin[1] * 1000, 5);
  near(R1.boss[0] * 1000, 3.5); near(R1.boss[1] * 1000, 10.5); near(R1.pin[0] * 1000, -5);
  const R2 = hornStack(Ly, 'R', 0, 0.006, 0.007, null, [-1, 0.008]);
  near(R2.boss[1] * 1000, 8); near(R2.pin[1] * 1000, 8);
  const sh = rockerArm({ boss: [0, 0.01] }, [0.004, 1], 0.004);
  near(sh.x[0] * 1000, 4); near(sh.x[1] * 1000, 8); near(sh.t * 1000, 4);
  const hi = rockerArm({ boss: [0, 0.01] }, [-1, 0.006], 0.004);
  near(hi.x[0] * 1000, 2); near(hi.x[1] * 1000, 6);
  // hornSide: with a washer the stack runs on and keeps within the hub; a side's own boss diameter; without, v21
  const L = { rocker: { boss_len_mm: 7, arm_t_mm: 6, boss_od_mm: 23, boss_od_side_mm: { L: 25 } }, retention: Ret };
  const hub = { x: [-0.0152, 0.0127] };
  const a = hornSide(Ly, cat, L, hub, -0.00624, 0.006, 'L');
  near(a.H.boss[0] * 1000, -15.2); near(a.arm.t * 1000, 5.46); assert.equal(a.boss_od_mm, 25); near(a.pw.past * 1000, 1.94); near(a.H.pin[1] * 1000, -1.3);
  const b = hornSide(Ly, cat, L, hub, 0.0022, 0.006, 'R');
  assert.equal(b.boss_od_mm, 23); near(b.H.pin[0] * 1000, -2.74);
  const c = hornSide(Ly, cat, { ...L, retention: null }, hub, -0.0043, 0.006, 'L');
  assert.equal(c.pw, null); near(c.H.boss[0] * 1000, -14.8); near(c.arm.x[0] * 1000, -14.3); near(c.arm.x[1] * 1000, -8.3); near(c.arm.t * 1000, 6);
  // the pin washer's members: the washer at its lowest column, the pin at the groove, the bushing end it bears on
  const cx = { eye: { id: 'bushing-igus-GFM-0810-06', specs: { static_surface_pressure_mpa: 80 } }, eyeD: { d: 0.008 } };
  const pw = pinWasher(cat, Ret, 'horn');
  const ms = pinWasherMembers(cx, 'horn pin washer left', 'horn pin left', pw, { id: 'metal-al-7075-t6' }, 0.05, [0, 0.02, -0.025], [{ ch: 'coupler_L', k: 1 }], 'why', 10);
  assert.deepEqual(ms.map((m) => m.name), ['horn pin washer left', "horn pin washer left on the bushing's end", "horn pin left at its washer's groove"]);
  assert.equal(ms[0].dims.rating_N, 700); assert.equal(ms[0].check, 'rated');
  assert.deepEqual(ms[1].dims, { D_mm: 10, d_mm: 8 }); assert.equal(ms[1].material.static_surface_pressure_mpa, 80);
  assert.equal(pinWasherMembers(cx, 'w', 'p', pw, { id: 'x' }, 0.05, [0, 0, 0], [], 'why', 15)[1].dims.D_mm, 12.3);
  assert.deepEqual(ms[2].dims, { d_mm: 6, planes: 1 }); assert.deepEqual(ms[2].sections[0].radial, [{ ch: 'coupler_L', k: 1 }]);
  assert.ok(ms[0].sections[0].radial.some((t) => t.ch === 'sf_origin' && Math.abs(t.k - 0.05) < 1e-12));
  // a boss wall with no per-seat list falls back to boss_wall_mm
  near(plateBoss({ bossed: ['gear shaft 1/aft'], boss_wall_mm: 8.5 }, 'gear shaft 1', 'aft').wall * 1000, 8.5);
  // containment: the pins' new solids join the horn slots; nothing else changes; no declarations pass through
  const c0 = { openings: [{ name: 'horn slots', solids: ['rocker R/horn pin'] }, { name: 'wing root, right', solids: ['x'] }], outside: [1] };
  const c1 = retentionContainment(c0);
  assert.deepEqual(c1.openings[0].solids, ['rocker R/horn pin', 'rocker R/horn pin groove', 'rocker R/horn pin tip', 'horn pin washer right/washer', 'rocker L/horn pin groove', 'rocker L/horn pin tip', 'horn pin washer left/washer']);
  assert.deepEqual(c1.openings[1], c0.openings[1]); assert.deepEqual(c1.outside, [1]); assert.equal(c0.openings[0].solids.length, 1);
  assert.equal(retentionContainment(null), null); assert.deepEqual(retentionContainment({ on_skin: [] }), { on_skin: [] });
  assert.deepEqual(retentionContainment({ openings: [{ name: 'horn slots' }] }).openings[0].solids.length, 6);
});

test('mutation gaps: the retain rules name nothing when a direction or a part is missing', () => {
  const members = ['gear shaft 1 washer aft', 'gear shaft 1 bushing flanges, axial thrust'].map((name) => ({ name, check: 'rated' }));
  const d = { params: {}, structure: { members, joints: [{ name: 'gear shaft 1' }] } };
  for (const ms of [members, [{ name: 'gear shaft 1 washer forward', check: 'rated' }, { name: 'gear shaft 1 washer aft', check: 'rated' }]]) {
    const L = interfaceLedger({ parts: [], joins: [['gear shaft 1 journal forward (bushing-igus-GFM-0810-06)', 'gear shaft 1/shaft', 'journal on its shaft']] }, { ...d, structure: { ...d.structure, members: ms } }, RETAIN_INTERFACE_RULES);
    assert.deepEqual(L.rows[0].axial.rating, []);
  }
  // a coupler bushing id with no end, or no side, names nothing for its axial stops
  const e = interfaceLedger({ parts: [], joins: [['coupler R horn bushing', 'coupler R', 'pressed in the eye'], ['coupler L crank bushing', 'crank web/pin', 'journal on its pin']] }, d, RETAIN_INTERFACE_RULES);
  assert.deepEqual(e.rows[0].axial.rating, ['coupler eye right bushing flanges, axial thrust', 'horn pin washer right']);
  assert.deepEqual(e.rows[1].axial.rating, ['coupler eye left bushing flanges, axial thrust', 'crank pin washer']);
});
