// The wing twist drive (round 17b, src/design/twist-drive.js): its geometry, its law, its records and their refusals.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import {
  TWIST_DRIVE, TWIST_SECTION, TWIST_INTERFACE_RULES, TWIST_SEQUENCE, twistParams, twistContext, twistKinematics, twistGeometry, ribPieces, drawnLines, twistBom,
  twistStiffness, twistChainFigures, twistChains, sectionRecord, twistSupportRows, twistMembers, twistLoads, servoTravel,
} from '../../src/design/twist-drive.js';
import { sectionNAt } from '../../src/design/section.js';
import { checkMembers } from '../../src/physics/members.js';
import { materialOf } from '../../src/design/linkage.js';
import { adhesiveOf } from '../../src/design/airframe.js';
import { THROUGH, THREAD, MESH, KEYWAY } from '../../src/design/clearance.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const base = JSON.parse(readFileSync(new URL('../../data/designs/larus-gl-2.json', import.meta.url), 'utf8'));
/** The official design with the drive stated (and its MD89MW, the spec-sheet sections), as round 17 builds it. */
const withDrive = (over = {}) => {
  const d = structuredClone(base);
  d.params = { ...d.params, tubeSpecSheet: true, parts: { ...d.params.parts, twistServo: 'servo-hitec-md89mw' }, twistDrive: { section: TWIST_SECTION, ...over } };
  return d;
};
const D = withDrive();

test('the twist drive is off unless the design states it', () => {
  assert.equal(twistParams(base.params), null);
  assert.equal(twistParams(D.params).e_mm, TWIST_DRIVE.e_mm);
  assert.equal(twistParams({ twistDrive: { e_mm: 100 } }).e_mm, 100);
});

test('the four-bar is parallel at its centre: its ratio is r_servo / r_lever, and the delivered law is exact at xi 0, 1/2 and 1', () => {
  const { G } = twistContext(D, catalog), K = twistKinematics(G);
  assert.ok(Math.abs(K.ratio(0) - G.rs / G.rl) < 1e-3, `${K.ratio(0)} vs ${G.rs / G.rl}`);
  assert.ok(Math.abs(K.rodAt(0)) < 1e-9);
  for (const th of [0.3, 0.6, 0.9]) {
    assert.ok(Math.abs(K.law(0, th)) < 1e-12);
    assert.ok(Math.abs(K.law(0.5, th) - th / 2) < 1e-12);
    assert.ok(Math.abs(K.law(1, th) - th) < 1e-12);
  }
  // the four-bar turns the rod the same way as the servo, more at the ends of travel
  assert.ok(K.rodAt(0.4) > 0.4 && K.rodAt(-0.4) < -0.4);
});

test('a rib is chordwise pieces inside the real section at both its faces, and its BOM volume is its drawn volume', () => {
  const { G, ribs, surf } = twistContext(D, catalog), rLe = 0.001, rTe = 0.00075, t = 0.002;
  for (const s of [ribs[0], ribs[6], ribs.at(-1)]) {
    const rp = ribPieces(D.wing, s, rLe, rTe, surf, G.sparN, { t_mm: 2 });
    assert.ok(rp.pieces.length > 4);
    for (const p of rp.pieces) for (const sf of [s - t / 2, s + t / 2]) for (const x of [p.x[0], (p.x[0] + p.x[1]) / 2, p.x[1]]) {
      const q = sectionNAt(D.wing, sf, x, surf, { sparN: G.sparN });
      assert.ok(p.n[0] >= q.lo - 1e-9 && p.n[1] <= q.hi + 1e-9, `rib at ${s}: piece ${JSON.stringify(p)} outside at ${sf}, ${x}`);
    }
  }
  const tw = twistBom(D, catalog), rho = catalog.find((e) => e.id === 'balsa-wood-database').specs.density_kg_m3;
  assert.ok(Math.abs(tw.by.ribs.V_m3 - tw.lines.ribsV_m3) < 1e-15);
  assert.ok(tw.lines.ribsV_m3 * rho * 1000 > 2 && tw.lines.ribsV_m3 * rho * 1000 < 12, 'a wing\'s ribs weigh a few grams');
});

test('the BOM holds every machined line at its drawn volume and every stock line at its drawn length', () => {
  const tw = twistBom(D, catalog), { by } = drawnLines(D, catalog);
  for (const q of tw.structure) {
    const tag = q.name.replace(/, \d+ off$/, ''), b = by[tag];
    assert.ok(b, `no solids for ${q.name}`);
    if (/twist (bars|tip arms)/.test(tag)) assert.ok(Math.abs(q.mass_g - 2 * 13.3 * b.L_m) < 1e-9);
    else assert.ok(Math.abs(q.mass_g - 2 * b.V_m3 * q.material.specs.density_g_cm3 * 1e6) < 1e-9, q.name);
    assert.equal(q.count, 2 * b.items);
  }
  for (const p of tw.parts) assert.equal(p.count, 2 * by[p.id].items, p.id);
});

test('the twist stiffness is the edge rods\' bending energy: positive, and scaling with each rod\'s EI', () => {
  const k = twistStiffness(D, catalog);
  assert.ok(k.K_Nm_rad > 0.01 && k.K_Nm_rad < 0.5, `${k.K_Nm_rad}`);
  for (const r of k.rods) assert.ok(r.K_Nm_rad > 0 && Number.isFinite(r.K_Nm_rad / r.EI_Nm2));
  assert.ok(Math.abs(k.rods.reduce((a, r) => a + r.K_Nm_rad, 0) - k.K_Nm_rad) < 1e-12);
});

test('the section record holds the drawn wing inside the E387 with its declared protrusions, and refuses an undeclared one', () => {
  const ok = sectionRecord(D, catalog, TWIST_SECTION);
  assert.ok(ok.passed, `undeclared: ${ok.undeclared.join(', ')}`);
  for (const p of ok.protrusions) assert.ok(p.declared && Number.isFinite(p.CdA_m2));
  // the servo raised 10 mm: its case stands out of the upper surface, and nothing declares it
  const up = withDrive({ servo: { spline_n_mm: 12 } });
  const bad = sectionRecord(up, catalog, TWIST_SECTION);
  assert.equal(bad.passed, false);
  assert.ok(bad.undeclared.includes('twist servo R'), bad.undeclared.join(', '));
  // and with no declarations at all, the arm that stands out at the top of its sweep fails it
  assert.equal(sectionRecord(D, catalog, {}).passed, false);
});

test('the twist servo\'s support rows: touching its frame\'s strip, and refused when the strip stands off it', () => {
  const rows = twistSupportRows(D, catalog);
  assert.equal(rows.length, 2);
  assert.ok(rows.every((r) => r.passed && r.gap_mm === 0));
  const off = twistSupportRows(withDrive({ hanger: { gap_mm: 0.3 } }), catalog);
  assert.ok(off.every((r) => !r.passed && Math.abs(r.gap_mm - 0.3) < 1e-6));
});

test('every join of the drawn wing meets a twist rule whose named members exist and are rated (or honestly unrated)', () => {
  const { joins } = drawnLines(D, catalog), members = [];
  const ax = { d: D, catalog, members, mat: (id) => materialOf(catalog, id), glueAl: adhesiveOf(catalog, 'adhesive-araldite-2011-aluminium'), glueCf: adhesiveOf(catalog, 'adhesive-araldite-2011') };
  twistMembers(ax);
  const names = new Set(members.map((m) => m.name)), skip = new Set([THROUGH, THREAD, MESH, KEYWAY]);
  const unmatched = [], missing = new Set();
  for (const [a, b, why] of joins) {
    if (skip.has(why) || /^rocker /.test(b)) continue;
    const r = TWIST_INTERFACE_RULES.find((q) => q.why.test(why) && (!q.part || q.part.test(a) || q.part.test(b)));
    if (!r) { unmatched.push(`${a} | ${b} | ${why}`); continue; }
    for (const n of [...r.load.rating(D, a, b), ...r.axial.rating(D, a, b)]) if (!names.has(n)) missing.add(n);
  }
  assert.deepEqual(unmatched, []);
  assert.deepEqual([...missing].filter((n) => !/^spar root (socket|bond) (right|left)$/.test(n)), []);
  // round 17b (lead): no member is left unrated: the maker's centre screw is held with the maker's arm to the servo's
  // stall torque (the orchestrator's decision for the R-ML25), disclosed in the arm's rating basis
  const res = checkMembers(members, Object.fromEntries(members.flatMap((m) => m.sections.map((s) => [`${m.name}/${s.name}`, 0.0]))));
  assert.deepEqual(res.members.filter((m) => m.check === 'unrated').map((m) => m.name), []);
  assert.match(members.find((m) => m.name === 'twist servo arm R').dims.rating_basis, /no rating of its own is published/);
});

test('a reason no twist rule knows is not matched (the ledger fails it)', () => {
  assert.equal(TWIST_INTERFACE_RULES.find((q) => q.why.test('held there by hope')), undefined);
});

test('the members carry their loads on the twist and wing channels, and the chain\'s loads follow the drawn geometry', () => {
  const members = [];
  twistMembers({ d: D, catalog, members, mat: (id) => materialOf(catalog, id), glueAl: adhesiveOf(catalog, 'adhesive-araldite-2011-aluminium'), glueCf: adhesiveOf(catalog, 'adhesive-araldite-2011') });
  assert.ok(members.length > 60);
  const chans = new Set(members.flatMap((m) => m.sections.flatMap((s) => s.radial.map((q) => q.ch))));
  for (const c of ['twist_R', 'twist_L', 'root_yz_R', 'root_x_L', 'elbow_R', 'elbow_L']) assert.ok(chans.has(c), c);
  const L = twistLoads(D, catalog);
  // the rod carries the tip's torque, the servo it over the four-bar's smallest ratio, and the ribs' shares sum to Q
  assert.ok(Math.abs(L.kTs - L.kq / L.kRod) < 1e-12 && L.kRod >= L.kRod0 - 1e-9);
  const w = (s) => (s - D.wing.segments[0].length) / D.wing.segments[1].length;
  assert.ok(Math.abs(L.a.reduce((acc, a, j) => acc + a * w(L.hand[j]), 0) - 1) < 1e-9);
  // at v24's flown peaks (twist 0.584 N m, elbow 5.913 N m, M7) and generous root loads every rated member holds FoS 1.5
  const peaks = Object.fromEntries(members.flatMap((m) => m.sections.map((s) => [`${m.name}/${s.name}`, s.radial.reduce((a, q) => a + q.k * ({ twist_R: 0.6, twist_L: 0.6, root_yz_R: 45, root_yz_L: 45, root_x_R: 8, root_x_L: 8, elbow_R: 5.95, elbow_L: 5.95 }[q.ch] ?? 0), 0)])));
  const res = checkMembers(members, peaks).members.filter((m) => m.check !== 'unrated');
  const weak = res.filter((m) => !m.ok).map((m) => `${m.name} ${m.fos?.toFixed?.(2)}`);
  assert.deepEqual(weak, []);
});

test('the chain record: the path is joined pair by pair, its figures are the drawn chain\'s, and a shortfall is a finding', () => {
  const { joins } = drawnLines(D, catalog), pairs = new Set(joins.map(([a, b]) => [a.split('/')[0], b.split('/')[0]].sort().join('|')));
  const F = twistChainFigures(D, catalog), d = structuredClone(D);
  d.servos.twist = { ...d.servos.twist, linkage_ratio: F.ratio, stall_Nm: 0.60604 / F.ratio, speed_rad_s: 7.588 * F.ratio, servo_stall_Nm: 0.60604, volts: 5.76 };
  const [r] = twistChains(null, d, catalog, { twistTorque_Nm: 0.584, travel_rad: 0.977, twistRate_rad_s: 9, basis: 'test' });
  assert.equal(r.name, 'twist R');
  for (let i = 0; i + 1 < r.path.length; i++) assert.ok(pairs.has([r.path[i], r.path[i + 1]].sort().join('|')), `${r.path[i]} | ${r.path[i + 1]}`);
  assert.ok(Math.abs(r.figures.ratio.drawn - r.figures.ratio.flown) <= r.figures.ratio.tol);
  assert.ok(r.figures.law.max_err <= r.figures.law.tol);
  assert.ok(r.figures.axis.err <= r.figures.axis.tol);
  assert.ok(Math.abs(r.figures.travel.delivered_rad - servoTravel(F.G, F.P) * F.kq) < 1e-4);
  // round 17b (lead): the torque and travel shortfalls are limits the flight model flies: reported as { text, flown }
  const txt = (f) => (typeof f === 'string' ? f : f.text);
  assert.ok(r.findings.some((f) => /^torque/.test(txt(f)) && f.flown) && r.findings.some((f) => /^travel/.test(txt(f)) && f.flown));
  const [ok] = twistChains(null, d, catalog, { twistTorque_Nm: 0.2, travel_rad: 0.5, twistRate_rad_s: 5, basis: 'test' });
  assert.ok(!ok.findings.some((f) => /^(torque|travel|speed)/.test(typeof f === 'string' ? f : f.text)));
});

test('the sequence puts each wing on with its drive as one piece', () => {
  assert.equal(TWIST_SEQUENCE.length, 2);
  const re = TWIST_SEQUENCE[0].steps[0].members.map((m) => new RegExp(m));
  for (const id of ['twist joiner R', 'twist rod end R lever', 'rib R 6', 'inner spar R', 'twist rib ring R 6 in']) assert.ok(re.some((q) => q.test(id)), id);
  assert.ok(!re.some((q) => q.test('twist joiner L')));
});

test('the geometry: the spars coaxial in the section, the bar within the travel\'s slot in every hand-wing rib', () => {
  const { G, ribs } = twistContext(D, catalog), P = twistParams(D.params);
  assert.ok(G.jA > ribs[G.k5] && G.jO1 < ribs[G.k5 + 1]);
  assert.ok(G.sOe < G.capF1 && G.capF1 <= G.sH0 && G.sTp1 <= G.L + 1e-12);
  assert.ok(G.e * Math.cos(servoTravel(G, P) / 2) > G.bar.ro);
  assert.throws(() => twistGeometry(D.wing, { ...D.params.parts, twistRod: 'no-such-part' }, ribs, P, catalog));
});
