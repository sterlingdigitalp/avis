// Round 17b (module "retain2"): the drive's and frame's axial retention finished: each shaft end's ring, the crank's
// forward circlip, keyed gears and bearing rings bonded with LOCTITE 638 (RC practice on titanium), the pins bonded in
// their 7075 bosses, the interface rules that name them, the keys turning with their shafts (swept envelopes), and the
// nuts drawn as bored hexagons. Each refusal is pinned against a deliberately broken, hand-built input.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { shaftEndRetainer, ringRating, retainingBond, driveMembers } from '../../src/design/linkage.js';
import { turningKey, sampleMoving, SWEEP_SKIP, ring, poly, nutSection, fullAssembly, GROOVE, KEYWAY, THREAD } from '../../src/design/clearance.js';
import { interfaceLedger, RETAIN_INTERFACE_RULES } from '../../src/design/interfaces.js';
import { rcPracticeCheck, checkMembers } from '../../src/physics/members.js';
import { fastenerGeometry, hexOf } from '../../src/design/completeness.js';
import { RETENTION, RETENTION_SEQUENCE } from '../../src/design/round17-retention.mjs';
import { driveLayout } from '../../src/design/joints.js';
import { drivePose, gearAngles } from '../../sims/avis-shared/model.js';

const near = (a, b, tol = 1e-9) => assert.ok(Math.abs(a - b) <= tol, `${a} vs ${b}`);
const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = () => JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
/** v24 with the round-17b retention merged as round17.mjs merges it (and the crank's bore change). */
function withRetention() {
  const d = v24(), R = RETENTION.linkage;
  d.params = { ...d.params, layout: { ...d.params.layout, ...RETENTION.layout }, linkage: { ...d.params.linkage, crank: { ...d.params.linkage.crank, ...R.crank }, retention: R.retention, plates: { ...d.params.linkage.plates, ...R.plates }, rocker: { ...d.params.linkage.rocker, ...R.rocker } } };
  return d;
}

test('each shaft end takes its own ring: the washer, a stated circlip, nothing where excluded', () => {
  const Ret = { washer8mm: 'w', forwardCirclip: { 2: 'c8' }, washer8mmForwardExclude: [2, 3], gearShaft3Circlip: null };
  assert.equal(shaftEndRetainer(Ret, 1, 'aft', false), 'w');
  assert.equal(shaftEndRetainer(Ret, 1, 'forward', false), 'w');
  assert.equal(shaftEndRetainer(Ret, 2, 'forward', false), 'c8');
  assert.equal(shaftEndRetainer(Ret, 3, 'forward', false), null);
  assert.equal(shaftEndRetainer(Ret, 3, 'aft', true), null);
  assert.equal(shaftEndRetainer({ ...Ret, gearShaft3Circlip: 'c' }, 3, 'aft', true), 'c');
  assert.equal(shaftEndRetainer({ ...Ret, gearShaft3Circlip: 'c' }, 3, 'forward', true), null);
  assert.equal(shaftEndRetainer(null, 1, 'aft', false), null);
  assert.equal(shaftEndRetainer({}, 1, 'forward', false), null);
});

test('a DIN 471 circlip is rated at the lowest of its load columns; a DIN 6799 washer as before', () => {
  const c = catalog.find((p) => p.id === 'circlip-din471-10');
  assert.ok(c, 'circlip-din471-10 merged into the catalogue, its table as staged');
  const r = ringRating(c.id, c.specs, 'metal-ti-6al-4v-annealed-bar');
  assert.equal(r.rating_N, 1010); assert.match(r.rating_basis, /FN 1.01 kN, the lowest of the table's FN 1.01, FR 4, FRg 2.4 kN/);
  assert.equal(ringRating('circlip-x', { load_columns: { FN_kN: 3, FR_kN: 2 } }, 'm').rating_N, 2000);
  const w = catalog.find((p) => p.id === 'washer-din6799-6');
  assert.equal(ringRating(w.id, w.specs, 'm').rating_N, 700);
});

test('LOCTITE 638 rates a steel part on aluminium at half its TDS figure, and nothing on titanium', () => {
  const a = retainingBond(catalog, 'adhesive-loctite-638', 'aluminium');
  near(a.shear_MPa, 8.5); assert.match(a.basis, /aluminium pins and collars/);
  assert.equal(retainingBond(catalog, 'adhesive-loctite-638', null), null);
  assert.equal(retainingBond(catalog, null, 'aluminium'), null);
  // broken: an adhesive with no aluminium figure is not rated on aluminium
  assert.equal(retainingBond([{ id: 'x', name: 'x', specs: { compressive_shear_iso10123_72h_mpa: { steel_pins_collars: 29 } } }], 'x', 'aluminium'), null);
});

test('the RC-practice check reports a fixing\'s peak axial force beside its torque', () => {
  const r = rcPracticeCheck({ rc: { method: 'bond' } }, 0, 12.5);
  assert.equal(r.peak_F_N, 12.5); assert.equal(r.ok, true); assert.equal(r.fos, null); assert.match(r.why, /peak axial force 12.50 N/);
  assert.equal('peak_F_N' in rcPracticeCheck({ rc: {} }, 1), false);
  const m = checkMembers([{ name: 'g', check: 'rc practice', rc: { method: 'm' }, uses: { F: 'F' } }], { 'g/F': 3 });
  assert.equal(m.members[0].peak_F_N, 3);
});

test('on v24 with the round-17b retention: every gear and bearing ring held by a rated member, the crank by RC practice', () => {
  const d = withRetention(), L = d.params.linkage, lay = driveLayout(d, catalog);
  const { members, items } = driveMembers(d, catalog, L, lay, d.structure.joints);
  const by = (n) => members.find((m) => m.name === n);
  // the crank's wheel on its titanium seat: RC practice (LOCTITE publishes no titanium figure), its axial force reported
  const cw = by('gear-khk-msgb1-45 on the crank, axial bond');
  assert.equal(cw.check, 'rc practice'); assert.equal(cw.rc.published_Nm, null); assert.deepEqual(cw.uses, { F: 'F' });
  // a pinion on a 7075 shaft: its face and hub, rated; a wheel on its seat: its face only
  const pn = by('gear-khk-msga1-18 on gear shaft 1, axial bond'), wh = by('gear-khk-msgb1-45 on gear shaft 1, axial bond');
  assert.equal(pn.check, 'bond'); near(pn.material.shear_MPa, 8.5); assert.equal(pn.dims.d_mm, 8); assert.equal(pn.dims.L_mm, 15);
  assert.equal(wh.dims.d_mm, 15); assert.equal(wh.dims.L_mm, 10);
  // gear shaft 3: both 628/8s bonded, inner and outer rings, each with its axial capacity at ISO 76
  for (const n of ['gear shaft 3 bearing aft, inner ring bond', 'gear shaft 3 bearing aft, outer ring bond', 'gear shaft 3 bearing forward, inner ring bond', 'gear shaft 3 bearing forward, outer ring bond']) assert.equal(by(n)?.check, 'bond', n);
  assert.equal(by('gear shaft 3 bearing forward, axial (ISO 76)').dims.rating_N, 1140);
  // the motor's outboard bearing: its outer ring only (its inner ring slides on the motor shaft)
  assert.equal(by('motor shaft outboard bearing forward, inner ring bond'), undefined);
  assert.deepEqual([by('gear shaft 3 bearing aft, inner ring bond').dims, by('gear shaft 3 bearing aft, outer ring bond').dims], [{ d_mm: 8, L_mm: 5 }, { d_mm: 16, L_mm: 5 }]);
  assert.equal(by('gear shaft 3 bearing aft, axial (ISO 76)').dims.rating_N, 1140);
  assert.equal(by('motor shaft outboard bearing forward, outer ring bond').dims.d_mm, 13);
  // the end rings: gear shaft 2's forward circlip, the crank's, both flanges of each
  assert.equal(by('gear shaft 2 circlip forward').dims.rating_N, 810);
  assert.equal(by('crank shaft circlip forward').dims.rating_N, 1010);
  assert.ok(by('gear shaft 2 bushing flanges, axial thrust') && by('crank shaft bushing flanges, axial thrust'));
  assert.equal(members.filter((m) => /spacer/.test(m.name)).length, 0, 'no spacer tubes');
  // the pins bonded in their bosses: Araldite 2011 on aluminium, axial and the bushings' friction torque
  const hp = by('horn pin right, bond in the horn boss'), sp = by('shoulder pin left, bond in the rocker hub');
  near(hp.material.shear_MPa, 13); assert.ok(hp.sections.find((s) => s.name === 'T').radial[0].k > 0);
  assert.equal(sp.dims.d_mm, 6); assert.ok(sp.sections.find((s) => s.name === 'T').radial.length >= 2);
  // the rings counted by catalogue id: 6 washers (2 horn, 1 crank pin, shaft 1 both ends, shaft 2 aft), 1 each circlip
  const count = (re) => items.find((i) => re.test(i.name))?.count;
  assert.equal(count(/DIN 6799/), 6); assert.equal(count(/DIN 471.*8 mm/), 1); assert.equal(count(/DIN 471.*10 mm/), 1);
  // the crank shaft's bore (item 9)
  assert.equal(by('crank shaft').dims.di_mm, 4);
});

test('on v24 with the round-17b retention: the rings drawn where their members say, the keys turning inside their sweeps', () => {
  const d = withRetention(), { asm } = fullAssembly(d, catalog);
  const part = (id) => asm.parts.find((p) => p.id === id), sol = (id, nm) => part(id)?.solids.find((s) => s.name === nm);
  const joined = (a, b, w) => asm.joins.some((j) => j[0] === a && j[1] === b && j[2] === w);
  // gear shaft 2's forward circlip in its groove at the forward flange, the shaft's end 0.5 mm or more short of the DS1-60
  const c2 = sol('gear shaft 2 circlip forward', 'circlip'), fl2 = sol('gear shaft 2 journal forward (bushing-igus-GFM-0810-06)', 'flange');
  near(c2.x[0], fl2.x[1], 1e-12); assert.ok(joined('gear shaft 2 circlip forward', 'gear shaft 2', GROOVE));
  const tip2 = sol('gear shaft 2', 'shaft forward tip'), ds60 = asm.parts.find((p) => /^stage 1 wheel gear-khk-ds1-60/.test(p.id)).solids.find((s) => s.name === 'teeth');
  assert.ok(ds60.x[0] - tip2.x[1] >= 0.0005 - 1e-9, `${(ds60.x[0] - tip2.x[1]) * 1000} mm`);
  // the crank's forward circlip at its flange, the groove at the ring's published diameter
  const cc = sol('crank shaft circlip forward', 'circlip'), g = sol('crank', 'shaft forward groove'), flc = sol('crank shaft journal forward (bushing-igus-GFM-1012-10)', 'flange');
  near(cc.x[0], flc.x[1], 1e-12); near(g.shape.ro, 0.0048, 1e-12); near(g.x[1] - g.x[0], 0.0011, 1e-12);
  // gear shaft 3 carries no ring or spacer (its bearing rings bonded); no spacer anywhere
  assert.equal(asm.parts.filter((p) => /spacer|gear shaft 3 circlip/.test(p.id) && !/^readhead/.test(p.id)).length, 0);
  // the keys: moving, inside their swept rings at every sampled pose, at +y at the reference pose, joined to their gears
  const lay = driveLayout(d, catalog), poseAt = (ph) => drivePose({ law: lay.law, fourBar: lay.fb }, ph, d.drive.ampNominal);
  const keys = asm.parts.filter((p) => /^key, /.test(p.id));
  assert.ok(keys.length >= 7);
  for (const k of keys) {
    const s = k.solids[0];
    assert.ok(s.at && s.sweep, k.id);
    for (let i = 0; i < 24; i++) for (const p of s.at(poseAt((i / 24) * 2 * Math.PI)).pts) { const r = Math.hypot(p[0] - s.sweep.c[0], p[1] - s.sweep.c[1]); assert.ok(r <= s.sweep.ro + 1e-12 && r >= s.sweep.ri - 1e-12, k.id); }
    const ref = s.at(poseAt(0)).pts;
    near(Math.min(...ref.map((p) => p[0])) + Math.max(...ref.map((p) => p[0])), 2 * s.sweep.c[0], 1e-9); // centred on its shaft in z
    assert.ok(Math.min(...ref.map((p) => p[1])) > s.sweep.c[1], `${k.id} at +y at the reference pose`);
    const gear = asm.joins.find((j) => j[0] === k.id && j[2] === KEYWAY && /^(stage|crank web)/.test(j[1]))?.[1];
    const gs = part(gear)?.solids.find((x) => x.name === 'teeth' || x.name === 'web');
    if (gs?.shape) assert.ok(s.sweep.ro < gs.shape.ro, `${k.id}'s sweep inside ${gear}`);
  }
  // a gear-shaft key turns at its shaft's tooth ratio: a quarter crank turn moves it by that shaft's angle
  const kk = keys.find((p) => /on gear shaft 3$/.test(p.id)).solids[0], q1 = poseAt(0.4), q2 = poseAt(0.9);
  const ang = (q) => { const pts = kk.at(q).pts, m = [(pts[0][0] + pts[2][0]) / 2 - kk.sweep.c[0], (pts[0][1] + pts[2][1]) / 2 - kk.sweep.c[1]]; return Math.atan2(m[1], m[0]); };
  const ga = (q) => gearAngles(lay.train, Math.atan2(q.A[1] - q.O2[1], q.A[0] - q.O2[0]))[3];
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  near(wrap(ang(q2) - ang(q1)), wrap(ga(q2) - ga(q1)), 1e-9);
  // the readhead's nuts: bored hexagons on their shanks' thread
  assert.ok(joined('readhead screw 1/shank', 'readhead nut 1', THREAD));
  assert.equal(sol('readhead nut 1', 'nut').shape.pts.length, 6);
});

test('a swept solid far from another is recorded at its envelope gap, a lower bound; a near one is sampled', () => {
  const key = { ...turningKey({ links: [], stages: [], n: 0 }, 0, [0, 0], 0.005, { b_mm: 3, h_mm: 3, shaft_depth_mm: 1.5 }, [0, 0.01]), id: 'k', part: 'k' };
  const far = { name: 'f', x: [0, 0.01], shape: ring([0.05, 0], 0.002), id: 'f', part: 'f' }, close = { name: 'c', x: [0, 0.01], shape: ring([0.0095, 0], 0.002), id: 'c', part: 'c' };
  const pairs = [{ a: key, b: far, min: Infinity, at: null, required: 0.0005 }, { a: key, b: close, min: Infinity, at: null, required: 0.0005 }];
  const poseAt = (amp, ph) => ({ A: [Math.cos(ph), Math.sin(ph)], O2: [0, 0] });
  sampleMoving(pairs, [0.3], 36, poseAt, poseAt(0.3, 0));
  assert.equal(pairs[0].swept, true); near(pairs[0].min, 0.05 - 0.002 - key.sweep.ro, 1e-12);
  assert.ok(pairs[0].min >= SWEEP_SKIP);
  assert.equal(pairs[1].swept, undefined); assert.ok(pairs[1].min < 0.0095 - 0.002 - key.sweep.ri);
  // the bound is never above the gap at any pose
  for (let i = 0; i < 36; i++) { const sh = key.at(poseAt(0.3, (i / 36) * 2 * Math.PI)); assert.ok(Math.min(...sh.pts.map((p) => Math.hypot(p[0] - 0.05, p[1]) - 0.002)) >= pairs[0].min - 1e-12); }
});

test('the retain2 rules: bonded bearings, the crank, the motor\'s own location and threads, RC practice in scope', () => {
  const M = (names, check = 'rated') => names.map((name) => ({ name, check }));
  const base = { params: {}, structure: { members: [], joints: [{ name: 'gear shaft 3' }, { name: 'motor shaft outboard' }], joints_unchecked: [{ name: 'motor shaft' }] } };
  const withM = (ms) => ({ ...base, structure: { ...base.structure, members: ms } });
  const g3 = (end) => `gear shaft 3 journal ${end} (bearing-skf-628-8-2z)`;
  const joins = [[g3('aft'), 'gearbox plate aft', 'pressed in its plate seat'], [g3('aft'), 'gear shaft 3/shaft', 'journal on its shaft'], [g3('forward'), 'gearbox plate forward', 'pressed in its plate seat'], [g3('forward'), 'gear shaft 3/shaft', 'journal on its shaft']];
  const full = M(['gear shaft 3 bearing aft, inner ring bond', 'gear shaft 3 bearing aft, outer ring bond', 'gear shaft 3 bearing forward, outer ring bond', 'gear shaft 3 bearing aft, axial (ISO 76)'], 'bond');
  const ok3 = interfaceLedger({ parts: [], joins }, withM(full), RETAIN_INTERFACE_RULES);
  assert.equal(ok3.failed.length, 0);
  // a floating forward bearing names the locating aft one; with its own inner ring bonded it names its own
  assert.deepEqual(ok3.rows[3].axial.rating, ['gear shaft 3 bearing aft, inner ring bond', 'gear shaft 3 bearing aft, outer ring bond', 'gear shaft 3 bearing aft, axial (ISO 76)', 'gear shaft 3 bearing forward, outer ring bond']);
  const both = [...full, ...M(['gear shaft 3 bearing forward, inner ring bond', 'gear shaft 3 bearing forward, axial (ISO 76)'], 'bond')];
  assert.deepEqual(interfaceLedger({ parts: [], joins }, withM(both), RETAIN_INTERFACE_RULES).rows[3].axial.rating, ['gear shaft 3 bearing forward, inner ring bond', 'gear shaft 3 bearing forward, outer ring bond', 'gear shaft 3 bearing forward, axial (ISO 76)']);
  // broken: no inner ring bonded (no bearing locates the shaft): both journals' shaft rows fail, the seats still hold
  const noLoc = interfaceLedger({ parts: [], joins }, withM(full.filter((m) => !/inner/.test(m.name))), RETAIN_INTERFACE_RULES);
  assert.deepEqual(noLoc.failed.map((r) => r.why), ['journal on its shaft', 'journal on its shaft']);
  // broken: the forward bearing's seat bond missing: its seat row and its shaft row fail
  assert.equal(interfaceLedger({ parts: [], joins }, withM(full.filter((m) => !/forward/.test(m.name))), RETAIN_INTERFACE_RULES).failed.length, 2);
  // the motor's outboard bearing: its seat held by its bond; on the shaft it names the motor's own location, uncredited
  const mo = 'motor shaft outboard journal forward (bearing-skf-619-5-2z)';
  const L = interfaceLedger({ parts: [], joins: [[mo, 'gearbox plate forward', 'pressed in its plate seat'], [mo, 'motor/shaft', 'journal on its shaft']] }, withM(M(['motor shaft outboard bearing forward, outer ring bond'], 'bond')), RETAIN_INTERFACE_RULES);
  assert.equal(L.failed.length, 1); assert.deepEqual(L.failed[0].axial.rating, ['motor shaft outboard bearing forward, outer ring bond', 'motor shaft']);
  // the motor bracket's screws in the motor's back face: their thread is unrated (no depth published)
  const T = interfaceLedger({ parts: [], joins: [['motor bracket', 'motor', 'screwed into its back-face threads']] }, withM(M(['motor mount screws', 'motor mount screws in the bracket'])), RETAIN_INTERFACE_RULES);
  assert.equal(T.failed.length, 1); assert.match(T.failed[0].why_failed, /axial retention not rated/); assert.match(T.failed[0].note, /m3\*4/);
  // a keyed gear: its key and its bond; the crank's wheel by an RC-practice bond, in scope (a gear)
  const keyed = M(['key, crank wheel gear-khk-msgb1-45', 'key, crank wheel gear-khk-msgb1-45, gear side'], 'key');
  const gj = [['stage 4 wheel gear-khk-msgb1-45', 'crank/wheel seat', 'gear on its shaft (bore on the shaft or seat)']];
  const rcL = interfaceLedger({ parts: [], joins: gj }, withM([...keyed, ...M(['gear-khk-msgb1-45 on the crank, axial bond'], 'rc practice')]), RETAIN_INTERFACE_RULES);
  assert.equal(rcL.failed.length, 0); assert.equal(rcL.rc_practice.length, 1); // listed with the RC-practice rows
  assert.equal(interfaceLedger({ parts: [], joins: gj }, withM(keyed), RETAIN_INTERFACE_RULES).failed.length, 1);
  // broken: an RC-practice member named for a joint that is not a gear, pulley or coupling fails on scope
  const rcRule = [{ why: /^on its widget$/, part: null, kind: 'bonded', load: { by: 'x', rating: () => ['w bond'], rule: 'r' }, axial: { by: 'x', rating: () => ['w bond'], rule: 'r' } }];
  const sc = interfaceLedger({ parts: [], joins: [['crank web', 'crank', 'on its widget']] }, withM(M(['w bond'], 'rc practice')), rcRule);
  assert.equal(sc.failed.length, 1); assert.match(sc.failed[0].why_failed, /covers only a gear, pulley or coupling/);
  assert.equal(interfaceLedger({ parts: [], joins: [['stage 1 wheel gear-x', 'crank', 'on its widget']] }, withM(M(['w bond'], 'rc practice')), rcRule).failed.length, 0);
});

test('a nut is its hexagon bored to the catalogue\'s minor diameter, measured back the same way', () => {
  const nut = catalog.find((p) => p.id === 'nut-iso4032-m3-10'), ng = fastenerGeometry(nut.specs, nut.id);
  near(ng.bore, 0.0025); near(ng.s, 0.0055);
  const sh = nutSection([0.01, -0.02], ng, { d: 0.003 });
  const h = hexOf({ name: 'nut', x: [0, ng.m], shape: sh }, null);
  near(h.flats, 0.0055, 1e-12); near(h.bore, 0.0025, 1e-12); near(h.L, ng.m, 1e-12);
  // no minor diameter stated: the screw's own; a ring (the old corner circle) or a hexagon with no bore is not one
  near(nutSection([0, 0], { s: 0.004 }, { d: 0.002 }).holes[0].r, 0.001);
  assert.equal(hexOf({ x: [0, 1], shape: ring([0, 0], 0.003) }, null), null);
  assert.equal(hexOf({ x: [0, 1], shape: poly(sh.pts) }, null), null);
});

test('the sequence fits every new ring after its plate, and no step names a spacer', () => {
  const names = RETENTION_SEQUENCE.flatMap((q) => q.steps.map((s) => s.name));
  for (const n of ['gear shaft 2 circlip forward', 'crank shaft circlip forward', 'gear shaft 1 washer forward']) assert.ok(RETENTION_SEQUENCE.find((q) => q.after === 'forward plate with its journals').steps.some((s) => s.name === n), n);
  assert.ok(!names.some((n) => /spacer|gear shaft 3 circlip/.test(n)));
  assert.ok(!RETENTION_SEQUENCE.some((q) => q.replace));
});

test('the round-17 crank (its web screwed on its stub): stopped forward by the web on the aft flange, aft by its circlip', () => {
  const d = withRetention();
  // round 17's crank web: a hub boss, an end lip and one countersunk M3 into the stub's tapped end (round17.mjs ROUND17.crank)
  d.params.linkage.crank = { ...d.params.linkage.crank, web: { ...d.params.linkage.crank.web, key: { ...d.params.linkage.crank.web.key, L_mm: 4.3, shaft_depth_mm: 1.5 }, hub_od_mm: 20, lip_mm: 1.7, axial: { screw: 'screw-iso10642-m3x8-10.9', countersunk: true, clearance_mm: 0.2, solid_mm: 2.5 } } };
  const lay = driveLayout(d, catalog), { members } = driveMembers(d, catalog, d.params.linkage, lay, d.structure.joints);
  const by = (n) => members.find((m) => m.name === n);
  // the screw holds the larger side: the shaft, its wheel and the actuator (held forward by the web), not only the web
  const screwF = by('crank web axial screw').sections[0].radial.find((t) => t.ch === 'sf_origin').k;
  const heldF = by('crank shaft circlip forward').sections[0].radial.find((t) => t.ch === 'sf_origin').k;
  near(screwF, heldF, 1e-9); assert.ok(heldF > 0.13, `${heldF} kg`);
  for (const n of ['crank web axial screw thread in the stub', 'crank web end lip under the screw head', 'crank shaft bushing flanges, axial thrust']) assert.ok(by(n), n);
  const { asm } = fullAssembly(d, catalog);
  const sol = (id, nm) => asm.parts.find((p) => p.id === id)?.solids.find((s) => s.name === nm);
  // the shaft split at the circlip's groove; the web's key turning with the crank, inside its sweep
  assert.ok(sol('crank', 'shaft') && sol('crank', 'shaft forward groove') && sol('crank', 'shaft forward') && sol('crank', 'shaft solid end'));
  near(sol('crank', 'shaft').x[1], sol('crank shaft circlip forward', 'circlip').x[0], 1e-12);
  const wk = sol('key, crank web', 'key');
  assert.ok(wk.at && wk.sweep); near(wk.sweep.ri, 0.005 - 0.0015, 1e-12);
  // the actuator joined to the shaft piece it sits on (beyond the groove), with its set screws' row
  assert.ok(asm.joins.some(([a, b]) => a === 'crank encoder actuator' && b === 'crank/shaft forward'));
});

test('a crank web screwed by a socket-head screw, no lip or solid end stated, and no circlip: the defaults and the refusals', () => {
  const d = withRetention();
  d.params.linkage.crank = { ...d.params.linkage.crank, web: { ...d.params.linkage.crank.web, axial: { screw: 'screw-iso4762-m3x8-10.9' } } };
  const lay = driveLayout(d, catalog), { members } = driveMembers(d, catalog, d.params.linkage, lay, d.structure.joints);
  const by = (n) => members.find((m) => m.name === n);
  // no lip: the lip's shear area is nil (its check then refuses); the thread engaged over the web alone
  assert.equal(by('crank web end lip under the screw head').dims.area_mm2, 0);
  near(by('crank web axial screw thread in the stub').dims.Le_mm, Math.min(8, d.params.linkage.crank.web_t_mm), 1e-9);
  const r = checkMembers([by('crank web end lip under the screw head')], { 'crank web end lip under the screw head/F': 1 });
  assert.equal(r.ok, false);
  const { asm } = fullAssembly(d, catalog);
  // its head on the web's aft face (screwX), the shaft not split by a solid end
  const sc = asm.parts.find((p) => p.id === 'crank web axial screw');
  assert.deepEqual(sc.solids.map((s) => s.name), ['head', 'shank']);
  assert.equal(asm.parts.find((p) => p.id === 'crank').solids.some((s) => s.name === 'shaft solid end'), false);
  // without the crank circlip: no ring, no groove, and the crank's journals name nothing for their axial stop
  const d2 = withRetention();
  d2.params.linkage.retention = { ...d2.params.linkage.retention, crankCirclip: null };
  const { asm: a2 } = fullAssembly(d2, catalog);
  assert.equal(a2.parts.some((p) => p.id === 'crank shaft circlip forward'), false);
  assert.deepEqual(a2.parts.find((p) => p.id === 'crank').solids.map((s) => s.name).filter((n) => /groove|forward/.test(n)), []);
  const m2 = driveMembers(d2, catalog, d2.params.linkage, driveLayout(d2, catalog), d2.structure.joints).members;
  assert.equal(m2.some((m) => /^crank shaft (circlip|bushing flanges)/.test(m.name)), false);
  const L = interfaceLedger({ parts: [], joins: [['crank shaft journal aft (bushing-igus-GFM-1012-10)', 'crank/shaft', 'journal on its shaft']] }, { params: {}, structure: { members: m2, joints: d2.structure.joints } }, RETAIN_INTERFACE_RULES);
  assert.equal(L.failed.length, 1);
});

test('each journal, gear and key is joined to the shaft pieces under it or touching its ends, and no others', () => {
  const d = withRetention(), { asm } = fullAssembly(d, catalog);
  const to = (a, w) => asm.joins.filter((j) => j[0] === a && j[2] === w).map((j) => j[1]).sort();
  // gear shaft 2's forward journal: the shaft, and the circlip's groove that starts at its flange's outer face
  assert.deepEqual(to('gear shaft 2 journal forward (bushing-igus-GFM-0810-06)', 'journal on its shaft'), ['gear shaft 2/shaft', 'gear shaft 2/shaft forward tip groove']);
  assert.deepEqual(to('gear shaft 2 journal aft (bushing-igus-GFM-0810-06)', 'journal on its shaft'), ['gear shaft 2/shaft', 'gear shaft 2/shaft aft tip groove']);
  // a pinion on the plain shaft; a wheel on its seat (and the shaft under the seat)
  const pin = asm.parts.find((p) => /^stage 3 pinion /.test(p.id)).id, wh = asm.parts.find((p) => /^stage 2 wheel /.test(p.id)).id;
  assert.deepEqual(to(pin, 'gear on its shaft (bore on the shaft or seat)'), ['gear shaft 2/shaft']);
  assert.deepEqual(to(wh, 'gear on its shaft (bore on the shaft or seat)'), ['gear shaft 2/shaft', 'gear shaft 2/wheel seat']);
  // the crank's forward journal: the shaft up to the groove, and the groove at its flange's face; not the piece beyond
  assert.deepEqual(to('crank shaft journal forward (bushing-igus-GFM-1012-10)', 'journal on its shaft'), ['crank/shaft', 'crank/shaft forward groove']);
});

test('mutation gaps: the held loads, the drawn lengths, the rings\' and keys\' sizes, the screw\'s thread and lip, exactly', async () => {
  const { inertialTerms } = await import('../../src/design/airframe.js');
  const d = withRetention();
  d.params.linkage.crank = { ...d.params.linkage.crank, web: { ...d.params.linkage.crank.web, key: { ...d.params.linkage.crank.web.key, L_mm: 4.3, shaft_depth_mm: 1.5 }, hub_od_mm: 20, lip_mm: 1.7, axial: { screw: 'screw-iso10642-m3x8-10.9', countersunk: true, clearance_mm: 0.2, solid_mm: 2.5 } } };
  const L = d.params.linkage, lay = driveLayout(d, catalog), P = lay.planes, { members, items } = driveMembers(d, catalog, L, lay, d.structure.joints);
  const by = (n) => members.find((m) => m.name === n), item = (n) => items.find((i) => i.name === n);
  const mass = (id) => catalog.find((p) => p.id === id).mass_g;
  const terms = (m, s = 'F') => m.sections.find((q) => q.name === s).radial;
  const same = (a, b) => { assert.equal(a.length, b.length); a.forEach((t, i) => { assert.equal(t.ch, b[i].ch); near(t.k, b[i].k, 1e-9); }); };
  // gear shaft 2's rings and flanges: its own mass and its two gears, at its wheel's plane and its own axis
  const g2 = item('gear shaft 2').mass_g, sh2 = lay.train.shafts[2], st = lay.train.stages;
  const F2 = inertialTerms((g2 + mass(st[1].wheel.id) + mass(st[2].pinion.id)) / 1000, [P.stage[1], sh2[1], sh2[0]]);
  for (const n of ['gear shaft 2 washer aft', 'gear shaft 2 circlip forward', 'gear shaft 2 bushing flanges, axial thrust']) same(terms(by(n)), F2);
  assert.deepEqual(by('gear shaft 2 bushing flanges, axial thrust').dims, { D_mm: 15, d_mm: 10 });
  // a gear's bond under its own inertia at its plane; the bearings' bonds under the shaft's held mass
  same(terms(by('gear-khk-msga1-18 on gear shaft 2, axial bond')), inertialTerms(mass('gear-khk-msga1-18') / 1000, [P.stage[2], sh2[1], sh2[0]]));
  same(terms(by('gear-khk-msgb1-45 on the crank, axial bond')), inertialTerms(mass('gear-khk-msgb1-45') / 1000, [P.stage[3], lay.fb.O2[1], lay.fb.O2[0]]));
  const g3 = item('gear shaft 3').mass_g, sh3 = lay.train.shafts[3];
  const F3 = inertialTerms((g3 + mass(st[0].wheel.id) + mass(st[1].pinion.id)) / 1000, [P.stage[0], sh3[1], sh3[0]]);
  for (const n of ['gear shaft 3 bearing aft, inner ring bond', 'gear shaft 3 bearing forward, outer ring bond', 'gear shaft 3 bearing aft, axial (ISO 76)']) same(terms(by(n)), F3);
  // the crank: everything that slides with it, at the crank wheel's plane on the crank axis
  const eye = catalog.find((p) => p.id === d.params.joints.couplerEyes), web = item('crank web and pin').mass_g, cs = item('crank shaft').mass_g;
  const coup = item('coupler links, 2 off').mass_g;
  const held = (cs + web + mass('gear-khk-msgb1-45') + mass('sensor-rls-orbis-actuator-10mm') + coup + 4 * eye.mass_g) / 1000;
  const Fc = terms(by('crank shaft circlip forward')), sf = Fc.find((t) => t.ch === 'sf_origin').k;
  assert.ok(Math.abs(sf - held) / held < 0.02, `${sf} vs ${held}`); // the crank's one-piece volume (v) vs its two items' masses
  same(terms(by('crank shaft bushing flanges, axial thrust')), Fc);
  assert.deepEqual(by('crank shaft bushing flanges, axial thrust').dims, { D_mm: 18, d_mm: 12 });
  // the screw's thread and the lip: engaged 6.3 mm (8 less the 1.7 mm lip), pitch diameter 2.675, the head's mean 4.5 mm
  const th = by('crank web axial screw thread in the stub').dims, lip = by('crank web end lip under the screw head').dims;
  near(th.Le_mm, 6.3, 1e-9); near(th.area_mm2, +(Math.PI * (3 - 0.6495 * 0.5) * 6.3 / 2).toFixed(3), 1e-9);
  near(lip.area_mm2, +(Math.PI * 4.5 * 1.7).toFixed(3), 1e-9);
  same(terms(by('crank web axial screw thread in the stub')), terms(by('crank web axial screw')));
  // each gear shaft's BOM length is its drawn length, its rings' lands included
  const { asm } = fullAssembly(d, catalog);
  for (const j of [1, 2, 3]) {
    const ss = asm.parts.find((p) => p.id === `gear shaft ${j}`).solids.filter((s) => s.name !== 'wheel seat');
    const drawn = Math.max(...ss.map((s) => s.x[1])) - Math.min(...ss.map((s) => s.x[0]));
    near(+item(`gear shaft ${j}`).basis.match(/x ([\d.]+) mm/)[1] / 1000, drawn, 6e-5);
  }
  // the rings' drawn sizes: thickness on the flange's outer side, swept and groove radii from the catalogue
  const sol = (id, nm) => asm.parts.find((p) => p.id === id)?.solids.find((s) => s.name === nm);
  const wa = sol('gear shaft 2 washer aft', 'washer'), fa = sol('gear shaft 2 journal aft (bushing-igus-GFM-0810-06)', 'flange');
  near(wa.x[1], fa.x[0], 1e-12); near(wa.x[1] - wa.x[0], 0.0007, 1e-12); near(wa.shape.ro, 0.00615, 1e-12); near(wa.shape.ri, 0.003, 1e-12);
  const c2 = sol('gear shaft 2 circlip forward', 'circlip');
  near(c2.x[1] - c2.x[0], 0.0008, 1e-12); near(c2.shape.ro, 0.00735, 1e-12); near(c2.shape.ri, 0.0038, 1e-12);
  const cc = sol('crank shaft circlip forward', 'circlip');
  near(cc.x[1] - cc.x[0], 0.001, 1e-12); near(cc.shape.ro, 0.0085, 1e-12); near(cc.shape.ri, 0.0048, 1e-12);
  // each key's sweep from its own seat: its bottom shaft_depth_mm into the seat or shaft it is keyed to
  const K = (re) => asm.parts.find((p) => re.test(p.id)).solids[0].sweep;
  near(K(/^key, wheel .* on gear shaft 1$/).ri, L.gearShafts[0].seat_od_mm / 2000 - L.gearShafts[0].wheelKey.shaft_depth_mm / 1000, 1e-12);
  near(K(/^key, pinion .* on gear shaft 2$/).ri, L.gearShafts[1].d_mm / 2000 - L.gearShafts[1].pinionKey.shaft_depth_mm / 1000, 1e-12);
  near(K(/^key, wheel .* on gear shaft 2$/).ri, L.gearShafts[1].seat_od_mm / 2000 - L.gearShafts[1].wheelKey.shaft_depth_mm / 1000, 1e-12);
  near(K(/^key, crank wheel /).ri, L.crank.seat_od_mm / 2000 - L.crank.key.shaft_depth_mm / 1000, 1e-12);
});

test('a ring against its flange is rated by the ring and that shaft\'s flange member; in its groove by the ring', () => {
  const ms = ['gear shaft 2 circlip forward', 'gear shaft 2 bushing flanges, axial thrust', 'crank shaft circlip forward', 'crank shaft bushing flanges, axial thrust'].map((name) => ({ name, check: 'rated' }));
  const d = { params: {}, structure: { members: ms, joints: [] } };
  const joins = [['gear shaft 2 circlip forward', 'gear shaft 2 journal forward (bushing-igus-GFM-0810-06)', 'against its flange'], ['crank shaft circlip forward', 'crank shaft journal forward (bushing-igus-GFM-1012-10)', 'against its flange'], ['crank shaft circlip forward', 'crank', GROOVE]];
  const L = interfaceLedger({ parts: [], joins }, d, RETAIN_INTERFACE_RULES);
  assert.equal(L.failed.length, 0, JSON.stringify(L.failed.map((r) => r.why_failed)));
  assert.deepEqual(L.rows[0].load.rating, ['gear shaft 2 circlip forward', 'gear shaft 2 bushing flanges, axial thrust']);
  assert.deepEqual(L.rows[1].load.rating, ['crank shaft circlip forward', 'crank shaft bushing flanges, axial thrust']);
  // broken: the flange member missing: the ring's row names nothing and fails
  const bad = interfaceLedger({ parts: [], joins }, { ...d, structure: { ...d.structure, members: ms.filter((m) => !/^gear shaft 2 bushing/.test(m.name)) } }, RETAIN_INTERFACE_RULES);
  assert.equal(bad.failed.length, 1); assert.deepEqual(bad.failed[0].load.rating, []);
});
