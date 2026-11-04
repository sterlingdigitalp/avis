// SPEC "Function completeness": "The covering spans only what is supported. Every surface the loft draws has structure
// under it. Film spans stay within a stated rule with its source." (audit v24 item 4, findings A1, F9, D5). These tests
// pin src/design/pod-structure.js: the loft cut to its structure, the covering record's refusals on deliberately broken
// assemblies (a removed former, a removed longeron, a part outside the film, an unframed cut, no hatch, a sharp bend,
// the v24 tube), the BOM against the drawn solids, the interface rules, the assembly order and the official refusal.
// The design is round 17's pod on the v24 file's parameters, with v24's tied tail (tailMech null) so the tests do not
// depend on the tail's work in progress.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildDesign } from '../../src/design/build.js';
import { solidMassParams } from '../../src/design/round11.mjs';
import { round17Set, mergeDeep, officialRefusal } from '../../src/design/round17.mjs';
import { fullAssembly, poly, ring, S } from '../../src/design/clearance.js';
import { interfaceLedger, RULE_HELPERS } from '../../src/design/interfaces.js';
import { sequenceRecord } from '../../src/design/sequence.js';
import { completenessRecord } from '../../src/design/completeness.js';
import { podAndBoomLoft, loftWettedArea } from '../../src/physics/geometry.js';
import { drivePose } from '../../sims/avis-shared/model.js';
import * as PS from '../../src/design/pod-structure.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const v24 = () => JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v24.json', import.meta.url) /* round 18: v24 frozen (the live file becomes v25) */, 'utf8'));

/** Round 17's set over v24's parameters (v24's tied tail), mass from solids as quick17 builds it. */
function build(over = {}) {
  const set = round17Set({ tailMech: null, ...over }, catalog);
  let p = mergeDeep(v24().params, set);
  for (const k of ['massFromSolids', 'lumpPos', 'tailMech', 'pod']) if (k in set) p[k] = set[k];
  p = solidMassParams(p, catalog);
  const d = buildDesign(p, catalog), { asm, lay } = fullAssembly(d, catalog);
  return { d, asm, lay };
}
let cached = null;
const base = () => (cached ??= build());
/** The assembly without the hatch (build-needs.js: its frame, cover, arcs and fixing): the window left open. */
const noHatch = (asm) => ({ ...asm, parts: asm.parts.filter((p) => !/^hatch /.test(p.id)) });
/** The assembly with these parts in place of its own (the round-17 build's hatch among them). */
const withParts = (asm, parts) => ({ ...asm, parts });
const record = (asm, d, lay, opts = {}) => PS.coveringRecord(asm, d, catalog, { lay, phases: 6, ...opts });

// ---- geometry ------------------------------------------------------------------------------------------------------

test('the loft ends where the structure does: the nose at its end disc, no tube aft of the cone', () => {
  const raw = podAndBoomLoft({ noseX_m: 0.2578, podLength_m: 0.573, width_m: 0.12, height_m: 0.14, boomOd_m: 0.01, tailEndX_m: -1.0888, y_m: -0.027, boomY_m: 0 });
  assert.equal(raw.sections.at(-1).x_m, -1.0888, 'the v24 loft runs a 10 mm tube to the tail');
  const cut = PS.trimPodLoft(raw, { ...PS.POD, nose_end_w_mm: 12 });
  assert.ok(Math.abs(cut.sections[0].width_m - 0.012) < 1e-12, 'the nose starts at the end disc\'s width');
  assert.ok(cut.sections.at(-1).x_m > -0.32 && cut.sections.at(-1).x_m < -0.31, 'the loft ends at the cone\'s end');
  assert.ok(loftWettedArea(raw) - loftWettedArea(cut) > 0.024, 'the tube\'s 0.024 m2 of wetted area is gone');
  assert.throws(() => PS.trimPodLoft(cut), /nose point/, 'a loft already cut has no nose point to cut');
  const { d } = base();
  assert.equal(d.body.loft.sections.length, cut.sections.length, 'the build cuts its loft the same way');
});

test('the nose stretch: the ogive\'s sections move about its first full section, so the station named moves as stated', () => {
  const raw = podAndBoomLoft({ noseX_m: 0.2578, podLength_m: 0.573, width_m: 0.12, height_m: 0.14, boomOd_m: 0.01, tailEndX_m: NaN, y_m: -0.027, boomY_m: 0 });
  const st = PS.noseStretch(raw, { x_mm: 231.2, by_mm: 40 }), x0 = raw.sections[6].x_m, e = 0.04 / (0.2312 - x0);
  // the first full section (120 x 140 mm) is the 7th, at 257.8 - 6 x 573 / 14 mm
  assert.ok(Math.abs(x0 - (0.2578 - (6 * 0.573) / 14)) < 1e-12 && Math.abs(st.stretch.e - e) < 1e-12);
  raw.sections.forEach((q, i) => {
    const r = st.sections[i];
    assert.equal(r.width_m, q.width_m); assert.equal(r.height_m, q.height_m);
    assert.ok(Math.abs(r.x_m - (i < 6 ? x0 + (q.x_m - x0) * (1 + e) : q.x_m)) < 1e-12, `section ${i}`);
  });
  assert.ok(Math.abs(PS.stretched(st, 231.2) - 271.2) < 1e-9 && PS.stretched(st, -50) === -50, 'a station aft of the ogive does not move');
  assert.equal(PS.noseStretch(raw, null), raw);
  assert.throws(() => PS.noseStretch(raw, { x_mm: -50, by_mm: 10 }), /forward, from a station in the ogive/);
});

test('the catenoid bound and its neck (MathWorld, Minimal Surface of Revolution, Eq. 26)', () => {
  assert.equal(PS.CATENOID_2H_OVER_R, 1.3254868386);
  assert.equal(PS.catenoidNeck(0.663), null, 'past the bound: no film between the rings');
  const near = PS.catenoidNeck(0.6627);
  assert.ok(near > 0.40 && near < 0.45, `at the bound the neck is about 0.45 R (got ${near})`);
  const small = PS.catenoidNeck(0.1);
  assert.ok(Math.abs(small - 0.1 ** 2 / 2) < 2e-4, 'a short span necks by about h^2 / 2 R');
});

test('a crossing is where the solid meets the skin\'s band: wholly outside or inside by more than the clearance, none', () => {
  const loft = { kind: 'sections', sections: [0.1, -0.1].map((x) => ({ x_m: x, y_m: 0, width_m: 0.1, height_m: 0.1 })) };
  const at = (z, r) => S('p', [-0.01, 0.01], ring([z, 0], r));
  assert.equal(PS.crossing(loft, at(0.06, 0.002), [null], 0.0005), null, 'a rod 8 mm outside the skin does not cut the film');
  assert.equal(PS.crossing(loft, at(0.04, 0.002), [null], 0.0005), null, 'nor one 8 mm inside');
  // a 2 mm rod centred on the 50 mm skin meets it at polar +-asin(2 / 50) = +-2.29 deg
  const through = PS.crossing(loft, at(0.05, 0.002), [null], 0.0005);
  assert.ok(through && PS.arcWithin(through.polar, [357.5, 2.5]) && PS.arcWithin([358, 2], through.polar) && through.x[0] === -0.01 && through.x[1] === 0.01, JSON.stringify(through));
  assert.ok(PS.crossing(loft, at(0.0523, 0.002), [null], 0.0005), 'a rod 0.3 mm outside the skin touches the film: a crossing');
  // a 12 mm square across the 50 mm skin, its outline drawn every 3 mm along each edge: no outline point lies within
  // 0.5 mm of the skin (47.98 mm in, 50.95 mm out at the top edge), yet the edges between them cross it
  const sq = S('sq', [-0.01, 0.01], poly([[0.0446, -0.006], [0.0566, -0.006], [0.0566, 0.006], [0.0446, 0.006]]));
  const cs = PS.crossing(loft, sq, [null], 0.0005);
  assert.ok(cs && PS.arcWithin(cs.polar, [350, 10]) && PS.arcWithin([357, 3], cs.polar), JSON.stringify(cs));
});

test('a solid that passes through the skin between two of its x samples crosses it there', () => {
  // a cone from 50 mm (x 0.1) to 30 mm (x -0.1) radius; a 1 mm rod along x at z 41.2 mm: at x 0 (radius 40) it is 0.7
  // to 1.7 mm outside, at x 0.025 (radius 45) 3.3 to 4.3 mm inside: it goes through the skin between
  const loft = { kind: 'sections', sections: [{ x_m: 0.1, y_m: 0, width_m: 0.1, height_m: 0.1 }, { x_m: -0.1, y_m: 0, width_m: 0.06, height_m: 0.06 }] };
  const c = PS.crossing(loft, S('p', [-0.1, 0.1], ring([0.0412, 0], 0.0005)), [null], 0.0005);
  assert.ok(c && Math.abs(c.x[0]) < 1e-12 && Math.abs(c.x[1] - 0.025) < 1e-12 && PS.arcWithin(c.polar, [358, 2]), JSON.stringify(c));
});

test('chords and arcs: the sign of a chord depth, arcs across 0 deg', () => {
  const A = [1, 0], B = [0, 1];
  assert.ok(PS.chordDepth(A, B, [0.2, 0.2]) > 0, 'a point toward the centre is inside the chord');
  assert.ok(PS.chordDepth(A, B, [Math.SQRT1_2, Math.SQRT1_2]) < 0, 'the arc between A and B lies outside its chord');
  assert.ok(PS.inArc(350, [330, 20]) && PS.inArc(10, [330, 20]) && !PS.inArc(90, [330, 20]));
  assert.ok(!PS.inArc(330, [330, 20], true), 'strictly: the ends are not in');
  assert.ok(PS.arcWithin([340, 10], [330, 20]) && !PS.arcWithin([320, 10], [330, 20]));
});

test('the hatch window is read from params.layout.hatch: its formers and the rods across it follow it', () => {
  const { d } = base(), G = PS.podGeometry(d, catalog), h = d.params.layout.hatch;
  const aft = G.formers.find((f) => f.name === 'hatch aft'), fwd = G.formers.find((f) => f.name === 'hatch forward');
  assert.ok(Math.abs(aft.x[1] - h.x_mm[0] / 1000) < 1e-12 && Math.abs(fwd.x[0] - h.x_mm[1] / 1000) < 1e-12);
  // round 19: the window moved to stations of the unstretched nose (x 160 and 238 mm) as the nose stretch takes them
  // (pod-structure.js stretched: round17.mjs's PACK_FWD_MM and NOSE_EXTRA_MM), so it holds the nose kink at any stretch
  const moved = structuredClone(d), xs = [160, 238].map((x) => PS.stretched(d.body.loft, x));
  moved.params.layout.hatch = { x_mm: xs, polar_deg: [30, 150] };
  const G2 = PS.podGeometry(moved, catalog), kinkMid = G.formers.find((f) => f.name === 'nose kink').mid;
  assert.ok(xs[0] / 1000 < kinkMid && xs[1] / 1000 > kinkMid, 'the moved window spans the nose kink');
  assert.ok(Math.abs(G2.formers.find((f) => f.name === 'hatch aft').x[1] - xs[0] / 1000) < 1e-12);
  const r70 = G2.runs.filter((r) => /^pod longeron mid 70/.test(r.id));
  assert.ok(r70.some((r) => Math.abs(r.x[1] - xs[0] / 1000) < 1e-12), 'the top rods stop at the moved window\'s edge');
  const kink = G2.formers.find((f) => f.name === 'nose kink');
  assert.deepEqual(kink.arcs, [[150, 30]], 'a former across the window is an arc ending at its edges');
});

// ---- the covering record ---------------------------------------------------------------------------------------------

test('the covering record passes on the round-17 pod with a hatch, and refuses it without one', () => {
  const { d, asm, lay } = base();
  const ok = record(asm, d, lay);
  assert.equal(ok.passed, true, ok.failed.join('; '));
  assert.ok(ok.rows.some((r) => r.kind === 'panel') && ok.rows.some((r) => r.kind === 'rib panels'));
  assert.ok(ok.film_clearance_min_mm >= 0.5, 'every enclosed part clears the film by the containment\'s 0.5 mm');
  assert.ok(ok.assumptions.some((a) => /1\.3254868386/.test(a) && /stated assumption/.test(a)), 'the rule and its source are stated');
  const no = record(noHatch(asm), d, lay);
  assert.equal(no.passed, false);
  assert.deepEqual(no.failed.filter((f) => /window/.test(f)), ['window: hatch @ 20..160']);
});

test('the rules\' figures come from their catalogue references; without them the record refuses', () => {
  const { d, asm, lay } = base();
  assert.deepEqual(PS.coveringSources(catalog), { catenoid: 1.3254868386, suction: 'peak design suction on the covering = 2 x wing loading x n (G) x 1.5', adhesive: 280 });
  const bare = catalog.filter((q) => !/^covering-ref-/.test(q.id));
  const r = PS.coveringRecord(asm, d, bare, { lay, phases: 6 });
  assert.equal(r.passed, false);
  assert.ok(['rule source: catenoid', 'rule source: suction', 'rule source: adhesive'].every((f) => r.failed.includes(f)));
  assert.ok(r.rows.some((q) => q.kind === 'rib panels' && !q.ok), 'with no adhesive figure no rib panel passes');
});

test('a removed former leaves its bays unsupported (the deliberately broken design)', () => {
  const { d, asm, lay } = base();
  const broken = withParts(asm, asm.parts.filter((p) => p.id !== 'pod former mid aft'));
  const r = record(broken, d, lay);
  assert.equal(r.passed, false);
  const bays = r.rows.filter((q) => q.kind === 'bay' && !q.ok);
  assert.equal(bays.length, 2, 'both bays on the missing former fail');
  assert.ok(bays.every((q) => q.formers_drawn === false && q.between.includes('mid aft')));
  // an end disc removed: the loft no longer ends on structure
  const noEnd = record(withParts(asm, asm.parts.filter((p) => p.id !== 'pod former tail end')), d, lay);
  assert.ok(noEnd.failed.includes('loft end: tail'));
});

test('the pod\'s bonds on the gearbox posts are drawn: a former off its post\'s face fails', () => {
  const { d, asm, lay } = base(), r = record(asm, d, lay);
  const rows = r.rows.filter((q) => q.kind === 'post bond');
  assert.deepEqual(rows.map((q) => q.name).sort(), ['pod former aft plate L', 'pod former aft plate R', 'pod former forward plate']);
  assert.ok(rows.every((q) => q.ok && q.faces_touch && q.overlap_mm2 >= q.rated_mm2 * 0.95));
  const moved = asm.parts.map((p) => (p.id === 'pod former forward plate' ? { ...p, solids: p.solids.map((s) => ({ ...s, x: s.x.map((v) => v + 0.001) })) } : p));
  const r2 = record(withParts(asm, moved), d, lay);
  assert.ok(r2.rows.some((q) => q.kind === 'post bond' && q.name === 'pod former forward plate' && !q.ok && !q.faces_touch));
  const narrow = asm.parts.map((p) => (p.id === 'gearbox plate aft' ? { ...p, solids: p.solids.map((s) => (/^post/.test(s.name) ? { ...s, shape: { ...s.shape, pts: s.shape.pts.map(([z, y]) => [z * 0.25, y]) } } : s)) } : p));
  const r3 = record(withParts(asm, narrow), d, lay);
  assert.ok(r3.rows.some((q) => q.kind === 'post bond' && /aft plate/.test(q.name) && !q.ok && q.faces_touch), 'a post a quarter as wide holds a quarter of the bond');
});

test('a removed longeron fails its panels and bays', () => {
  const { d, asm, lay } = base();
  const r = record(withParts(asm, asm.parts.filter((p) => p.id !== 'pod longeron mid 270')), d, lay);
  assert.equal(r.passed, false);
  assert.ok(r.rows.some((q) => q.kind === 'panel' && !q.ok && q.longerons.includes('pod longeron mid 270')));
  assert.ok(r.rows.some((q) => q.kind === 'longeron ends' && q.run === 'pod longeron mid 270' && !q.ok));
});

test('a part between the film\'s chord and the loft fails the film clearance; deeper in it passes', () => {
  const { d, asm, lay } = base(), G = PS.podGeometry(d, catalog), loft = d.body.loft;
  const at = (depth) => {
    const x = -0.047, h = PS.sectionAt(loft, x), a = 261 * Math.PI / 180, c = PS.insetPoint(h, a, depth + 0.0005);
    return { id: 'probe block', solids: [S('block', [x - 0.002, x + 0.002], ring(c, 0.0005))] };
  };
  assert.ok(G.runs.some((r) => r.deg === 252) && G.runs.some((r) => r.deg === 270), 'the probe sits mid-panel between the 252 and 270 deg rods');
  const near = record(withParts(asm, [...asm.parts, at(0.0006)]), d, lay);
  assert.ok(near.rows.some((q) => q.kind === 'film clearance' && q.solid === 'probe block/block' && !q.ok));
  const deep = record(withParts(asm, [...asm.parts, at(0.008)]), d, lay);
  assert.ok(!deep.rows.some((q) => q.kind === 'film clearance' && q.solid === 'probe block/block' && !q.ok));
});

test('an opening whose crossing lies in no window, on no part that holds the film, fails', () => {
  const { d, asm, lay } = base(), loft = d.body.loft, h = PS.sectionAt(loft, -0.05);
  const post = { id: 'stray post', solids: [S('post', [-0.052, -0.048], ring(PS.surfacePoint(h, 270 * Math.PI / 180), 0.002))] };
  const dd = structuredClone(d);
  dd.params.layout.containment.openings = [...dd.params.layout.containment.openings, { name: 'stray', solids: ['stray post'], why: 'a test' }];
  const r = record(withParts(asm, [...asm.parts, post]), dd, lay);
  assert.ok(r.rows.some((q) => q.kind === 'opening' && q.opening === 'stray' && q.ok === false && q.in === null));
  assert.ok(r.rows.some((q) => q.kind === 'opening' && q.opening === 'wing root, right' && q.ok && /slots/.test(q.in)), 'the wing roots lie in the slots\' window');
  assert.ok(r.rows.some((q) => q.kind === 'opening' && q.opening === 'boom posts' && q.ok && /held by the part/.test(q.in)), 'the forward post holds the film\'s edge');
});

test('a window whose polar edge is on no longeron is not framed', () => {
  const { d, lay } = base();
  const dd = structuredClone(d);
  // round 19: the tail pushrod exits and the pitot exit are closed; a test window there with its lower edge off any rod
  dd.params.pod = { ...dd.params.pod, windows: [...PS.POD.windows, { name: 'test window', between: ['hatch forward', 'nose kink'], polar_deg: [310, 345], openings: [], why: 'a test' }] };
  const { asm } = fullAssembly(dd, catalog);
  const r = record(asm, dd, lay);
  const row = r.rows.find((q) => q.kind === 'window' && q.name === 'test window');
  assert.equal(row.ok, false);
  assert.deepEqual(row.framed_polar, [false, true]);
});

test('a longeron run over a sharp kink of the loft fails its bend; the drawn sets bend within the allowance', () => {
  const { d, asm, lay } = base(), r = record(asm, d, lay);
  const bends = r.rows.filter((q) => q.kind === 'longeron bend');
  assert.ok(bends.length > 20 && bends.every((q) => q.ok));
  // the nose's rods run on past the nose kink to the next former (the mid set starting there)
  const dd = structuredClone(d);
  dd.params.pod = { ...dd.params.pod, longerons: PS.POD.longerons.filter((s) => s.name !== 'nose corner' && s.name !== 'nose sides').map((s) => (s.name === 'nose' ? { ...s, to: 'nose 176' } : s.name === 'mid' ? { ...s, from: 'nose 176' } : s)) };
  // round 19: at the nose kink's former wherever the nose stretch puts it; the allowance from the rod's catalogue figures
  // (its lower compressive strength over its upper modulus, at FoS 1.5: rod-ec-1-5mm)
  const G = PS.podGeometry(dd, catalog), run = G.runs.find((q) => q.id === 'pod longeron nose 270'), xk = G.formers.find((f) => f.name === 'nose kink').mid;
  const k = PS.runKinks(dd.body.loft, run).find((q) => Math.abs(q.x - xk) < 1e-9);
  const lr = catalog.find((q) => q.id === 'rod-ec-1-5mm').specs, allow = lr.compressive_strength_lengthways_mpa_range[0] / (lr.tensile_modulus_lengthways_gpa_range.at(-1) * 1000) / 1.5;
  assert.ok(k && k.angle > 10 * Math.PI / 180 && 0.00075 / k.radius > allow, `the nose kink (${k && (k.angle * 180 / Math.PI).toFixed(1)} deg) bends the rod past its strain allowance`);
});

test('the wing and tail films: Myers\' suction on each rib bond within the film\'s adhesive power; one rib holds no panel', () => {
  const { d, asm, lay } = base(), r = record(asm, d, lay);
  const rows = r.rows.filter((q) => q.kind === 'rib panels');
  assert.deepEqual(rows.map((q) => q.surface).sort(), ['tail L', 'tail R', 'wing L', 'wing R']);
  assert.ok(rows.every((q) => q.ok && q.adhesive_power_N_per_m === 280));
  const oneRib = asm.parts.filter((p) => !/^rib R (?!1$)\d+$/.test(p.id));
  const r1 = record(withParts(asm, oneRib), d, lay);
  assert.ok(r1.rows.some((q) => q.kind === 'rib panels' && q.surface === 'wing R' && !q.ok));
  // ribs 2 m apart: the panel's load on its bonds passes the film's adhesive power
  const far = asm.parts.filter((p) => !/^tail rib L [34]$/.test(p.id)).map((p) => (p.id === 'tail rib L 2' ? { ...p, solids: p.solids.map((s) => ({ ...s, shape: { ...s.shape, pts: s.shape.pts.map(([z, y]) => [z - 2, y + 2]) } })) } : p));
  const r2 = record(withParts(asm, far), d, lay);
  assert.ok(r2.rows.some((q) => q.kind === 'rib panels' && q.surface === 'tail L' && !q.ok));
});

// ---- the BOM, the joints, the order and the refusal ----------------------------------------------------------------

test('the BOM\'s pod lines are their drawn solids: formers by volume, longerons by length, the film over the cut loft', () => {
  const { d, asm, lay } = base();
  const q0 = drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampMin);
  const c = completenessRecord(asm, d, catalog, q0);
  for (const re of [/^fuselage formers, \d+ off$/, /^fuselage longerons, \d+ off$/, /^fuselage film shell$/]) {
    const row = c.lines.find((l) => re.test(l.line));
    assert.ok(row && row.passed, `${re}: ${JSON.stringify(row?.size ?? row)}`);
  }
  const b = PS.podBom(d, catalog), lines = Object.fromEntries(b.structure.map((s) => [s.name.replace(/, \d+ off$/, ''), s]));
  assert.ok(Math.abs(lines['fuselage longerons'].mass_g - 2.7 * b.rod_length_m) < 1e-9);
  assert.ok(Math.abs(lines['fuselage formers'].mass_g - b.former_volume_m3 * 1.33e6) < 1e-9);
  assert.ok(b.film_area_m2 < loftWettedArea(d.body.loft), 'the film is cut at the windows');
  assert.match(lines['fuselage film shell'].basis, /hotmelt adhesive is brushed on/);
  const lump = d.mass.lumps.find((l) => l.name === 'fuselage shell and formers');
  assert.deepEqual(lump.lines.sort(), b.structure.map((s) => s.name).sort(), 'the pod\'s lump carries its own lines');
});

test('every pod joint has a rated rule; the rule texts are interfaces.js\'s own', () => {
  const { d, asm } = base();
  const L = interfaceLedger(asm, d, PS.POD_INTERFACE_RULES);
  const pod = L.rows.filter((r) => /^pod /.test(r.a) || /^pod /.test(r.b));
  assert.ok(pod.length > 100);
  assert.deepEqual(pod.filter((r) => !r.ok).map((r) => `${r.a} | ${r.b} | ${r.why_failed}`), []);
  const without = interfaceLedger(asm, d).rows.filter((r) => /^pod /.test(r.a) || /^pod /.test(r.b));
  assert.ok(without.some((r) => !r.ok), 'without the pod\'s rules its joins fail');
  for (const r of PS.POD_INTERFACE_RULES) for (const side of [r.load, r.axial]) assert.ok([RULE_HELPERS.BOND_RULE, RULE_HELPERS.TIE_RULE].includes(side.rule));
  // a member the rule names but the design lacks: the row fails
  const dd = { ...d, structure: { ...d.structure, members: d.structure.members.filter((m) => m.name !== 'pod former bonds on the gearbox posts') } };
  assert.ok(interfaceLedger(asm, dd, PS.POD_INTERFACE_RULES).rows.some((r) => r.why === 'bonded on the post\'s face' && !r.ok));
});

test('the members carry the pod\'s inertia and its drag', () => {
  const { d } = base(), names = d.structure.members.map((m) => m.name);
  for (const n of ['pod former bonds on the gearbox posts', 'pod former bands at the posts', 'pod longerons carrying the cone', 'pod longeron bonds carrying the cone', 'pod longerons carrying the pod aft of the slots', 'pod longerons carrying the nose']) assert.ok(names.includes(n), n);
  assert.ok(!names.includes('pod formers ties'), 'v24\'s tie member is gone with its rings');
  const bond = d.structure.members.find((m) => m.name === 'pod former bonds on the gearbox posts');
  const lump = d.mass.lumps.find((l) => l.name === 'fuselage shell and formers');
  const kSf = bond.sections[0].radial.filter((t) => t.ch === 'sf_origin').reduce((a, t) => a + t.k, 0);
  const D = 0.5 * (d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1) * 25 ** 2 * d.body.CdA_front_m2;
  // round 19: statics, the pod's centroid against its two plate formers (inside them with the nose stretched, outside
  // without): the more loaded support carries max(lever) / span of its inertia, never less than half
  const xs = PS.podGeometry(d, catalog).formers.filter((f) => f.plate).map((f) => f.mid).sort((a, b) => a - b);
  const kSup = Math.max(xs[1] - lump.pos[0], lump.pos[0] - xs[0]) / (xs[1] - xs[0]);
  // (members.js terms are stated to 8 decimals: two rounded terms, so within 2e-8 kg)
  assert.ok(kSup >= 0.5 && kSf >= lump.mass_kg * kSup + D / 9.80665 - 2e-8, 'each post bond: the pod\'s inertia on the more loaded support and its whole drag');
});

test('every pod solid is in an assembly step, and none passes through another; the covering goes on after its structure', () => {
  const { d, asm, lay } = base();
  const sq = sequenceRecord(asm, d, lay);
  assert.deepEqual(sq.missing.filter((m) => /^pod /.test(m)), []);
  assert.deepEqual(sq.failures.filter((f) => /^pod /.test(f)), []);
  const names = d.params.layout.sequence.steps.map((s) => s.name), cover = names.findIndex((n) => /^pod covering/.test(n));
  assert.ok(cover > names.findIndex((n) => /^pod tail end former$/.test(n)) && cover < names.findIndex((n) => /^wing R$/.test(n)));
  assert.ok(!names.some((n) => /^former \d/.test(n)), 'round 16\'s ring steps are replaced');
});

test('--official refuses a design whose covering record is missing or fails', () => {
  const { d, asm, lay } = base();
  const pass = { massprops: { passed: true }, covering: record(asm, d, lay) };
  const fail = { ...pass, covering: record(noHatch(asm), d, lay) };
  const why = (c) => officialRefusal({ ...c }) ?? '';
  assert.doesNotMatch(why(pass), /covering/);
  assert.match(why(fail), /covering: window: hatch/);
  assert.match(why({ massprops: { passed: true } }), /no covering record/);
});

// ---- the numbers behind the rows and members, recomputed from the drawn solids -----------------------------------------

const polyArea = (pts) => Math.abs(pts.reduce((a, p, i) => a + p[0] * pts[(i + 1) % pts.length][1] - pts[(i + 1) % pts.length][0] * p[1], 0)) / 2;
/** The mass (kg) and centroid of the pod's drawn structure (rods by length, formers by volume) and film over x0..x1. */
function drawnShare(asm, d, x0, x1) {
  let m = 0; const s = [0, 0, 0];
  const add = (mm, p) => { m += mm; for (let k = 0; k < 3; k++) s[k] += mm * p[k]; };
  for (const p of asm.parts.filter((q) => /^pod (longeron|former) /.test(q.id))) for (const sol of p.solids) {
    const xm = (sol.x[0] + sol.x[1]) / 2;
    if (xm < x0 || xm > x1) continue;
    if (sol.tube) add(0.0027 * sol.tube.L, [xm, sol.shape.c[1], sol.shape.c[0]]);
    else { const c = sol.shape.pts.reduce((a, q) => [a[0] + q[0] / sol.shape.pts.length, a[1] + q[1] / sol.shape.pts.length], [0, 0]); add(polyArea(sol.shape.pts) * (sol.x[1] - sol.x[0]) * 1330, [xm, c[1], c[0]]); }
  }
  const sec = d.body.loft.sections, lo = Math.max(x0, sec.at(-1).x_m), hi = Math.min(x1, sec[0].x_m);
  if (hi > lo) { const h = PS.sectionAt(d.body.loft, (lo + hi) / 2); add(0.017 * PS.loftArea(d.body.loft, lo, hi, () => true, 60, 72), [(lo + hi) / 2, h.y, 0]); }
  return { m, r: s.map((v) => v / m) };
}
/** The drawn rods at x: their number, the group's smaller second moment (per rod area) and its farthest rod. */
function drawnGroup(asm, x) {
  const cs = asm.parts.filter((p) => /^pod longeron /.test(p.id)).map((p) => p.solids.find((s) => s.x[0] <= x && s.x[1] >= x)).filter(Boolean).map((s) => s.shape.c);
  const c = cs.reduce((a, p) => [a[0] + p[0] / cs.length, a[1] + p[1] / cs.length], [0, 0]);
  return { n: cs.length, sum: Math.min(cs.reduce((a, p) => a + (p[1] - c[1]) ** 2, 0), cs.reduce((a, p) => a + (p[0] - c[0]) ** 2, 0)), cmax: Math.max(...cs.map((p) => Math.hypot(p[0] - c[0], p[1] - c[1]))) };
}
const kOf = (m, ch) => m.sections[0].radial.filter((t) => t.ch === ch).reduce((a, t) => a + t.k, 0);
const near = (a, b, rel, what) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${what}: ${a} against ${b}`);

test('the cantilever members: each part\'s mass, lever and rod group as drawn, its longest bay', () => {
  const { d, asm } = base(), G = PS.podGeometry(d, catalog), f = (n) => G.formers.find((q) => q.name === n);
  const D = 0.5 * (d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1) * 25 ** 2 * d.body.CdA_front_m2, g = 9.80665;
  const win = G.windows.find((w) => w.name === 'wing roots and horn slots');
  const cases = [
    // the cone's longest bay: from the cone start's former to the pushrod exits' forward one
    ['the cone', f('cone start'), -Infinity, f('cone start').mid, f('cone start').mid - 1e-4, (f('cone start').mid - f('pushrod exits forward').mid) * 1000],
    // its longest bay: the slots' (the aft plate's former to the one at the slots' aft edge) or one of the two aft of it
    ['the pod aft of the slots', f('aft plate'), -Infinity, f('aft plate').x[0], (win.x[0] + win.x[1]) / 2, Math.max(f('aft plate').mid - f('slots aft').mid, f('slots aft').mid - f('mid aft').mid, f('mid aft').mid - f('cone start').mid) * 1000],
    // round 19: the nose's longest bay forward of the forward plate's former, from the formers (the nose stretch moves them)
    ['the nose', f('forward plate'), f('forward plate').x[1], Infinity, f('forward plate').mid + 1e-4, (() => { const m = G.formers.map((q) => q.mid).filter((v) => v >= f('forward plate').mid - 1e-9).sort((a, b) => a - b); return Math.max(...m.slice(1).map((v, i) => v - m[i])) * 1000; })()],
  ];
  for (const [nm, fs, x0, x1, xg, L] of cases) {
    const strut = d.structure.members.find((m) => m.name === `pod longerons carrying ${nm}`), sh = drawnShare(asm, d, x0, x1), gr = drawnGroup(asm, xg);
    const k = 1 / gr.n + (Math.abs(sh.r[0] - fs.mid) * gr.cmax) / gr.sum;
    near(kOf(strut, 'sf_origin'), sh.m * k + D / g / gr.n, 0.06, `${nm}: rod force per g`);
    near(kOf(strut, 'wdot') / (kOf(strut, 'sf_origin') - D / g / gr.n), Math.hypot(...sh.r), 0.05, `${nm}: the lever of its rotation terms`);
    assert.ok(Math.abs(strut.dims.L_mm - L) < 0.15, `${nm}: longest bay ${strut.dims.L_mm}`);
    assert.equal(strut.dims.d_mm, 1.5);
    const bond = d.structure.members.find((m) => m.name === `pod longeron bonds carrying ${nm}`);
    assert.ok(bond.dims.w_mm === 1.5 && [0.5, 1].includes(bond.dims.L_mm));
    assert.ok(kOf(bond, 'sf_origin') > sh.m / 40, `${nm}: its bond carries its share`);
  }
  assert.equal(d.structure.members.find((m) => m.name === 'pod longeron bonds carrying the cone').dims.L_mm, 0.5, 'the cone\'s rods end on the cone start: half its thickness');
  assert.equal(d.structure.members.find((m) => m.name === 'pod longeron bonds carrying the nose').dims.L_mm, 1, 'the mid rods run on over the forward plate\'s former');
});

test('the post bonds carry the pod as a beam on its two plate formers, and its drag', () => {
  const { d } = base(), G = PS.podGeometry(d, catalog), lump = d.mass.lumps.find((l) => l.name === 'fuselage shell and formers');
  const xs = G.formers.filter((f) => f.plate).map((f) => f.mid).sort((a, b) => a - b), xc = lump.pos[0];
  const kSup = Math.max(xs[1] - xc, xc - xs[0]) / (xs[1] - xs[0]);
  // round 19: with the nose stretched the centroid lies between the two supports (kSup < 1), without it ahead of them
  assert.ok(kSup >= 0.5 && (kSup > 1) === (xc < xs[0] || xc > xs[1]), `the more loaded support's share ${kSup}`);
  const D = 0.5 * (d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1) * 25 ** 2 * d.body.CdA_front_m2;
  const bond = d.structure.members.find((m) => m.name === 'pod former bonds on the gearbox posts');
  near(kOf(bond, 'sf_origin'), lump.mass_kg * kSup + D / 9.80665, 1e-6, 'post bond per g');
  near(kOf(bond, 'wdot'), lump.mass_kg * Math.hypot(...lump.pos) * kSup, 1e-5, 'post bond rotation');
  assert.deepEqual(bond.dims, { w_mm: 16, L_mm: 4 });
  const band = d.structure.members.find((m) => m.name === 'pod former bands at the posts'), arm = kOf(band, 'sf_origin') / kOf(bond, 'sf_origin');
  assert.ok(arm > 0.012 && arm < 0.02, `the band's arm from the post to the 70/110 deg rods (${arm})`);
  assert.deepEqual(band.dims, { b_mm: 4, t_mm: 1 });
  const beside = d.structure.members.find((m) => m.name === 'pod former bands beside the posts'), armT = kOf(beside, 'sf_origin') / kOf(bond, 'sf_origin');
  assert.ok(armT >= 0.001 && armT < 0.004, `the thin band from the deep arc's end (72 deg) to the 70 deg rod (${armT})`);
  assert.deepEqual(beside.dims, { b_mm: 1.5, t_mm: 1 });
});

test('the rib-panel rows: Myers\' suction, the widest panel between the drawn ribs, the load on its bonds', () => {
  const { d, asm, lay } = base(), r = record(asm, d, lay), rho = d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1;
  const q0 = drivePose({ law: lay.law, fourBar: lay.fb }, 0, d.drive.ampMin);
  const widest = (re) => { const cs = asm.parts.filter((p) => re.test(p.id)).map((p) => { const s = p.solids[0], sh = s.at ? s.at(q0) : s.shape; return sh.pts.reduce((a, q) => [a[0] + q[0] / sh.pts.length, a[1] + q[1] / sh.pts.length], [0, 0]); }).sort((a, b) => Math.hypot(...a) - Math.hypot(...b)); return Math.max(...cs.slice(1).map((c, k) => Math.hypot(c[0] - cs[k][0], c[1] - cs[k][1]))); };
  const wing = r.rows.find((q) => q.kind === 'rib panels' && q.surface === 'wing R'), tail = r.rows.find((q) => q.kind === 'rib panels' && q.surface === 'tail L');
  near(wing.peak_suction_Pa, 2 * d.geometry.wing_loading_N_m2 * 5 * 1.5, 1e-3, 'wing suction');
  near(tail.peak_suction_Pa, 2 * 0.5 * rho * 25 ** 2 * 1.5, 1e-3, 'tail suction');
  near(wing.widest_mm, widest(/^rib R \d+$/) * 1000, 1e-4, 'the wing\'s widest panel');
  near(tail.widest_mm, widest(/^tail rib L \d+$/) * 1000, 1e-4, 'the tail\'s widest panel');
  for (const q of [wing, tail]) { near(q.bond_load_N_per_m, (q.peak_suction_Pa * q.widest_mm) / 2000, 1e-3, `${q.surface} bond load`); near(q.ratio, 280 / q.bond_load_N_per_m, 1e-2, `${q.surface} ratio`); }
  assert.equal(wing.ribs, 10);
  // a heavier design: the wing's own loading enters
  const dd = structuredClone(d); dd.geometry.wing_loading_N_m2 *= 20;
  assert.ok(!record(asm, dd, lay).rows.find((q) => q.kind === 'rib panels' && q.surface === 'wing R').ok, 'at 20 x the loading the wing\'s film pulls off its ribs');
});

test('bays: 2h/R from the loft, the longerons holding them; with no longerons a bay is a ring span within the bound or not', () => {
  const { d, asm, lay } = base(), r = record(asm, d, lay), loft = d.body.loft;
  for (const b of r.rows.filter((q) => q.kind === 'bay')) {
    const L = (b.x_mm[1] - b.x_mm[0]) / 1000, R = Math.min(...[b.x_mm[1] / 1000 - 1e-9, b.x_mm[0] / 1000 + 1e-9].map((x) => { const h = PS.sectionAt(loft, x); return Math.min(h.hw, h.hh); }));
    // (the record states 2h/R to 4 decimals: round 19, with the nose stretched, the 'nose 94' former stands ~1 mm from the
    // hatch's aft one, a bay whose 2h/R is ~0.02, where that rounding alone is 0.3%)
    assert.ok(Math.abs(b.two_h_over_R - L / R) <= 1e-3 * (L / R) + 5e-5, `bay ${b.between}: ${b.two_h_over_R} against ${L / R}`);
    assert.equal(b.held_by, 'panels between longerons');
  }
  const over = r.rows.filter((q) => q.kind === 'bay' && q.two_h_over_R > 1.3255);
  assert.ok(over.length >= 4 && over.every((q) => q.ok && q.held_by === 'panels between longerons'), 'several bays (the nose\'s end and the cone\'s) hold only because of their longerons');
  assert.ok(r.rows.filter((q) => q.kind === 'panel').every((q) => q.max_sag_mm > 0 && q.max_sag_mm < 6), 'the film sags between rods by less than 6 mm');
  // no nose longerons: the nose's bays are ring spans, past the catenoid's bound
  const dd = structuredClone(d); dd.params.pod = { ...dd.params.pod, longerons: PS.POD.longerons.filter((s) => !/^nose/.test(s.name)) };
  const { asm: a2 } = fullAssembly(dd, catalog), r2 = record(a2, dd, lay);
  const nose = r2.rows.filter((q) => q.kind === 'bay' && (q.between[0] === 'nose end' || q.between[0] === 'hatch forward'));
  assert.equal(nose.length, 2);
  assert.ok(nose.every((q) => q.held_by === 'rings only'));
  // the nose end's bay is past the bound (no film between its rings); round 19: the one aft of the hatch's forward
  // former (17.6 mm long unstretched, 1.45 x that with the pack 97 mm forward) by the rule, whichever side of it the
  // nose stretch puts it: within the bound it holds, its neck stated; past it, it fails with no neck
  const [end, aft] = nose;
  assert.ok(!end.ok && end.two_h_over_R > 1.3255 && end.catenoid_neck_mm === null, JSON.stringify(end));
  const within = aft.two_h_over_R < 1.3255;
  assert.ok(aft.ok === within && (within ? aft.catenoid_neck_mm > 0 : aft.catenoid_neck_mm === null), JSON.stringify(aft));
});

test('the bend rows are the circle through three supports; the loft ends on its discs', () => {
  const { d, asm, lay } = base(), G = PS.podGeometry(d, catalog), run = G.runs.find((q) => q.id === 'pod longeron mid 270');
  // round 19: at the 'nose 176' former wherever the nose stretch puts it
  const x176 = G.formers.find((f) => f.name === 'nose 176').mid;
  const k = PS.runKinks(d.body.loft, run), at = k.find((q) => Math.abs(q.x - x176) < 1e-9), L = run.line, i = L.findIndex((v) => Math.abs(v.x - at.x) < 1e-9);
  const P = (v) => [v.x, ...v.p], u = P(L[i]).map((c, j) => c - P(L[i - 1])[j]), v = P(L[i + 1]).map((c, j) => c - P(L[i])[j]);
  const ang = Math.acos(u.reduce((a, c, j) => a + c * v[j], 0) / (Math.hypot(...u) * Math.hypot(...v)));
  near(at.angle, ang, 1e-9, 'the bend\'s angle');
  near(at.radius, Math.hypot(...P(L[i + 1]).map((c, j) => c - P(L[i - 1])[j])) / (2 * Math.sin(ang)), 1e-9, 'the bend\'s radius');
  // round 19: a real bend (the ogive's), its row the record's: the strain r / R at it, within the allowance
  const r = record(asm, d, lay), ends = r.rows.filter((q) => q.kind === 'loft end');
  const row = r.rows.find((q) => q.kind === 'longeron bend' && q.run === run.id && Math.abs(q.x_mm - at.x * 1000) < 1e-3);
  assert.ok(at.angle > 0.01 && row && Math.abs(row.strain - 0.00075 / at.radius) <= 1e-3 * row.strain && row.ok === (0.00075 / at.radius <= row.allowable), JSON.stringify({ at, row }));
  assert.deepEqual(ends.map((q) => [q.what, q.ok, q.loft_x_mm === q.structure_x_mm, q.drawn]), [['nose', true, true, true], ['tail', true, true, true]]);
  const noNose = record(withParts(asm, asm.parts.filter((p) => p.id !== 'pod former nose end')), d, lay);
  assert.ok(noNose.failed.includes('loft end: nose') && !noNose.failed.includes('loft end: tail'));
  const dd = structuredClone(d); dd.body.loft.sections[0].x_m += 0.002;
  assert.ok(record(asm, dd, lay).failed.includes('loft end: nose'), 'a loft past its nose disc');
});

test('openings and windows: the crossings as measured, a hatch only where the window is', () => {
  const { d, asm, lay } = base(), r = record(asm, d, lay);
  // round 19: the pitot exit closed: the film's cut round the probe, the probe holding its edge; its crossing on the
  // nose between the nose kink's former and the nose end, at the probe's own polar angles
  const pit = r.rows.find((q) => q.kind === 'opening' && q.opening === 'pitot'), G = PS.podGeometry(d, catalog), fx = (n) => G.formers.find((q) => q.name === n).x;
  assert.ok(pit.ok && /^held by the part/.test(pit.in) && !G.windows.some((w) => /pitot/.test(w.name)), JSON.stringify(pit));
  assert.ok(pit.x_mm[0] > fx('nose kink')[1] * 1000 && pit.x_mm[1] < fx('nose end')[0] * 1000 && pit.polar_deg[0] > 320 && pit.polar_deg[1] < 342, JSON.stringify(pit));
  const sock = r.rows.find((q) => q.kind === 'opening' && q.solid === 'rocker R/spar socket');
  assert.ok(sock.polar_deg[0] > 20 && sock.polar_deg[1] < 60 && sock.x_mm[0] < 0 && sock.x_mm[1] > 0);
  const hatch = r.rows.find((q) => q.kind === 'window' && q.name === 'hatch');
  assert.ok(hatch.covered_by.includes('hatch cover') && hatch.covered_by.includes('hatch cover arcs') && hatch.covered_by.every((id) => /^hatch /.test(id)), JSON.stringify(hatch.covered_by));
  const away = { ...noHatch(asm), parts: [...noHatch(asm).parts, { id: 'hatch far', solids: [S('cover', [-0.1, -0.09], poly([[0, 0], [0.001, 0], [0.001, 0.001]]))] }] };
  assert.ok(!record(away, d, lay).rows.find((q) => q.kind === 'window' && q.name === 'hatch').ok, 'a hatch part outside the window covers nothing');
  for (const w of r.rows.filter((q) => q.kind === 'window')) assert.deepEqual([w.framed_x, w.framed_polar], [[true, true], [true, true]], w.name);
});

test('arcs and the catenoid at their edges', () => {
  assert.ok(PS.inArc(0, [0, 360]) && PS.inArc(359.9, [0, 0]) && PS.inArc(30, [30, 30]));
  assert.ok(PS.inArc(30, [30, 60]) && PS.inArc(60, [30, 60]) && !PS.inArc(60, [30, 60], true) && PS.inArc(45, [30, 60], true));
  assert.ok(PS.arcWithin([40, 140], [40, 140]) && !PS.arcWithin([39, 140], [40, 140]) && !PS.arcWithin([40, 141], [40, 140]));
  assert.ok(PS.arcWithin([350, 10], [330, 210]) && !PS.arcWithin([200, 220], [330, 210]));
  assert.equal(PS.catenoidNeck(0), 0);
  assert.ok(PS.catenoidNeck(0.3) > PS.catenoidNeck(0.2) && PS.catenoidNeck(0.6627) < 0.4476 + 1e-3);
  assert.equal(PS.catenoidNeck(PS.CATENOID_2H_OVER_R / 2), null);
});

test('the bonds\' force factors and the bands\' arms, recomputed from the drawn rods and the posts', () => {
  const { d, asm } = base(), G = PS.podGeometry(d, catalog), f = (n) => G.formers.find((q) => q.name === n);
  const D = 0.5 * (d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1) * 25 ** 2 * d.body.CdA_front_m2, g = 9.80665;
  const cases = [['the cone', f('cone start').mid - 1e-4, -Infinity, f('cone start').mid, f('cone start'), 0.5], ['the pod aft of the slots', f('aft plate').mid, -Infinity, f('aft plate').x[0], f('aft plate'), 0.5], ['the nose', f('forward plate').mid, f('forward plate').x[1], Infinity, f('forward plate'), 1]];
  for (const [nm, xg, x0, x1, fs, L] of cases) {
    const bond = d.structure.members.find((m) => m.name === `pod longeron bonds carrying ${nm}`), sh = drawnShare(asm, d, x0, x1), gr = drawnGroup(asm, xg);
    const kB = 1 / gr.n + (Math.abs(sh.r[0] - fs.mid) * gr.cmax) / gr.sum;
    near(kOf(bond, 'sf_origin'), sh.m * kB + D / g / gr.n, 0.06, `${nm}: bond force per g`);
    assert.equal(bond.dims.L_mm, L, `${nm}: bonded length`);
  }
  // the bands: from each post's edges to the nearest rod on its former (the larger semi-axis), and beyond the deep arc
  const bond = d.structure.members.find((m) => m.name === 'pod former bonds on the gearbox posts');
  const arcs = (fm, from) => { const h = PS.sectionAt(d.body.loft, fm.mid), rods = G.runs.filter((r) => r.x[0] <= fm.mid + 1e-9 && r.x[1] >= fm.mid - 1e-9).map((r) => r.deg); return Math.min(...rods.flatMap((a) => from.map((e) => Math.min(Math.abs(a - e), 360 - Math.abs(a - e))))) * Math.PI / 180 * Math.max(h.hw, h.hh); };
  const held = G.formers.filter((q) => q.plate);
  const arm = Math.max(...held.map((fm) => { const e = Math.acos(0.008 / PS.sectionAt(d.body.loft, fm.mid).hw) * 180 / Math.PI; return Math.max(arcs(fm, [e, 180 - e]), 0.008); }));
  near(kOf(d.structure.members.find((m) => m.name === 'pod former bands at the posts'), 'sf_origin') / kOf(bond, 'sf_origin'), arm, 1e-5, 'the band\'s arm at the posts');
  const armT = Math.max(...held.map((fm) => Math.max(arcs(fm, [72, 108]), 0.001)));
  // members.js terms are stated to 8 decimals: the band's terms within half of 1e-8 each of the bond's times the arm
  const beside = d.structure.members.find((m) => m.name === 'pod former bands beside the posts'), nT = (m) => m.sections[0].radial.filter((t) => t.ch === 'sf_origin').length;
  assert.ok(Math.abs(kOf(beside, 'sf_origin') - kOf(bond, 'sf_origin') * armT) <= 0.5e-8 * (nT(beside) + nT(bond) * armT) + 1e-15, `the band's arm beyond the deep arc: ${kOf(beside, 'sf_origin') / kOf(bond, 'sf_origin')} against ${armT}`);
});

// round 18 (lead): the open windows' drag from its two catalogue sources, on the real round-17 pod's own windows
test('the open windows\' drag: the cavity source\'s Cd on every open window, the through-flow bound on the smaller side', () => {
  const { d, asm, lay } = base();
  const o = record(asm, d, lay).open;
  const A = o.windows.reduce((s, w) => s + w.area_m2, 0), Amax = Math.max(...o.windows.map((w) => w.area_m2));
  // round 19: the pushrod exits closed (with v24's tied tail there are no pushrods, so no guides and no leaks either)
  assert.deepEqual(o.windows.map((w) => w.name), ['wing roots and horn slots'], 'round 19: the pitot exit closed too');
  assert.ok(!o.windows.some((w) => /pushrod/.test(w.name)) && o.leaks.length === 0);
  assert.ok(!o.windows.some((w) => /hatch/.test(w.name)), 'the hatch is closed by its cover');
  assert.equal(o.Cd, 0.108);
  assert.ok(Math.abs(o.cavity_CdA_m2 - 0.108 * A) < 2e-6);
  assert.ok(Math.abs(o.through_CdA_m2 - 0.85 * Math.min(Amax, A - Amax)) < 2e-6);
  assert.ok(Math.abs(o.CdA_m2 - (o.cavity_CdA_m2 + o.through_CdA_m2)) < 2e-6);
  // without the sources the record falls back to Cd 1.0 on the cuts, no through-flow term
  const bare = PS.coveringRecord(asm, d, catalog.filter((q) => !/^drag-ref-/.test(q.id)), { lay, phases: 6 }).open;
  assert.equal(bare.Cd, 1.0);
  assert.equal(bare.through_CdA_m2, 0);
  assert.ok(Math.abs(bare.CdA_m2 - A) < 2e-6);
});
