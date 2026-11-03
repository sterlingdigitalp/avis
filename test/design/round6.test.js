// Round 12: the design-file fixture is v20 (test/fixtures/larus-gl-2-v20.json, the committed HEAD design), the design these hand calculations were written for; v21 has its own tests (round12).
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { buildDesign } from '../../src/design/build.js';
import { driveLayout } from '../../src/design/joints.js';
import { materialOf } from '../../src/design/linkage.js';
import { round6Params, ROUND6 } from '../../src/design/round6.mjs';
import { LOAD_CHANNELS } from '../../src/physics/flight.js';
import { linkageAt } from '../../sims/avis-shared/model.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
const base = () => JSON.parse(readFileSync(join(ROOT, 'test/fixtures/larus-gl-2-v20.json'), 'utf8'));
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);
const built = (() => { let d = null; return () => (d ??= buildDesign(round6Params(base(), {}, catalog), catalog)); })();

test('materials (round 7 ruling): standard minimums in full, values without a stated basis at half', () => {
  const ti = materialOf(catalog, 'metal-ti-6al-4v-annealed-bar');
  near(ti.yield_MPa, 827, 0.5, 'ASTM B348 Gr 5 minimum 120 ksi');
  const al6 = materialOf(catalog, 'metal-al-6061-t6');
  near(al6.yield_MPa, 240, 1e-9, '6061-T6 Rp0.2 minimum, EN 754-2 / 755-2 / 485-2');
  near(al6.bearing_yield_MPa, 386 / 2, 1e-9, 'ASM sheet bearing yield, no basis stated');
  near(materialOf(catalog, 'metal-ti-6al-4v-annealed-bar').bearing_yield_MPa, 1480 / 2, 1e-9);
  near(materialOf(catalog, 'composite-rockwest-t-rnd-393-transverse').yield_MPa, 38600 * 0.00689476 / 2, 0.1);
  near(materialOf(catalog, 'adhesive-araldite-2011').shear_MPa, 2770 * 0.00689476 / 2, 0.01);
  assert.equal(al6.bearing_e_over_d_min, 2);
  const al7 = materialOf(catalog, 'metal-al-7075-t6');
  near(al7.yield_MPa, 485, 1e-9, '7075-T6 Rp0.2 minimum, EN 754-2 drawn bar');
  assert.equal(al7.bearing_yield_MPa, null);
  assert.throws(() => materialOf(catalog, 'plate-ec-2mm'), /no design yield/);
});

test('round 6 linkage: every loaded member in the BOM by mass, on known load channels, with a published material', () => {
  const d = built();
  const members = d.structure.members;
  assert.ok(members.length >= 30);
  for (const m of members) {
    if (m.check === 'unrated') { assert.ok(m.why, `${m.name} says why it is unrated`); continue; }
    assert.ok(Number.isFinite(m.material.yield_MPa ?? m.material.shear_MPa), `${m.name} material`);
    for (const s of m.sections) for (const t of s.radial) assert.ok(LOAD_CHANNELS.includes(t.ch) && t.k >= 0, `${m.name}/${s.name} ${t.ch}`);
    for (const names of Object.values(m.uses)) for (const n of [].concat(names)) assert.ok(m.sections.some((s) => s.name === n), `${m.name} uses ${n}`);
  }
  // The parts the ruling names are all there: shafts, pins, plate seats, end fittings (coupler eyes), keys.
  for (const re of [/^spar root socket left$/, /^rocker arm right$/, /gear side$/, /^gearbox plates between seats$/, /ds1-64 on gear shaft 2, fixing/, /^crank shaft$/, /^crank pin$/, /^gear shaft 1$/, /^gear shaft 3$/, /^shoulder pin right$/, /^horn pin left$/, /^plate seat crank shaft aft$/, /^coupler eye right$/, /^key, crank wheel/, /^shoulder housing left aft$/]) assert.ok(members.some((m) => re.test(m.name)), String(re));
  // Each mass item is in the structure list with its material's source; carbon crank, rockers, plates and couplers gone.
  const names = d.bom.structure.map((s) => s.name);
  for (const n of ['crank: shaft, web and pin in one piece', 'gear shaft 1', 'rocker right', 'coupler links, 2 off', 'gearbox plates, 2 off']) assert.ok(names.includes(n), n);
  assert.ok(!names.includes('crank, rockers and gearbox plates') && !names.includes('couplers, 2 off'));
  for (const s of d.bom.structure) assert.ok(s.source_url, s.name);
  // M6 arithmetic still holds: margin 15% of parts plus structure.
  near(d.bom.margin_g, 0.15 * (d.bom.parts_g + d.bom.structure_g), 1e-9);
  // The outboard motor bearing is a BOM part and a joint.
  assert.ok(d.bom.parts.some((p) => p.id === 'bearing-skf-619-5-2z' && /motor outboard/.test(p.role)));
  assert.ok(d.structure.joints.some((j) => j.name === 'motor shaft outboard'));
  // The coupler is the metal link, judged at its material's design yield and full modulus.
  assert.equal(d.structure.coupler.kind, 'metal link');
  near(d.structure.coupler.allowable_MPa, 240, 1e-9);
  near(d.structure.coupler.E_GPa, 68.9, 1e-9);
  // The FC line carries the corrected-PDB requirement.
  assert.match(d.bom.parts.find((p) => p.id === 'fc-matek-f405-wmo').hardware, /not marked "3m0"/i);
});

test('round 6 linkage geometry by hand: crank shaft and pin moment arms, outboard bearing lever, crank mass', () => {
  const d = built();
  const lay = driveLayout(d, catalog);
  const pl = lay.planes;
  const crank = d.structure.members.find((m) => m.name === 'crank shaft');
  // At the aft support both couplers overhang: arms plateAft - couplerR and plateAft - couplerL.
  const atAft = crank.sections.find((s) => Math.abs(s.radial.find((t) => t.ch === 'coupler_R')?.k - (pl.plateAft - pl.couplerR)) < 1e-6);
  assert.ok(atAft, 'a section at the aft support');
  near(atAft.radial.find((t) => t.ch === 'coupler_L').k, pl.plateAft - pl.couplerL, 1e-6);
  // Crank pin: from the web's aft face (crank arm plane less half the 5 mm web).
  const pin = d.structure.members.find((m) => m.name === 'crank pin');
  const face = pl.crankArm - ROUND6.linkage.crank.web_t_mm / 2000;
  near(pin.sections[0].radial.find((t) => t.ch === 'coupler_R').k, face - pl.couplerR, 1e-6);
  near(pin.sections[0].radial.find((t) => t.ch === 'coupler_L').k, face - pl.couplerL, 1e-6);
  // Outboard bearing: the stage 1 mesh force times (a + s) / s.
  const ob = d.structure.joints.find((j) => j.name === 'motor shaft outboard').supports[0].radial[0];
  const a = pl.plateFwd - pl.stage[0], s = pl.motorAft - pl.plateFwd + 0.0025;
  near(ob.k, lay.meshPerNm(0) * (a + s) / s, 1e-5);
  // Crank mass: tube, web, pin and wheel seat in Ti-6Al-4V at 4.43 g/cm3.
  const c = ROUND6.linkage.crank;
  const eyeD = 0.008, eyeB = 0.006;
  const lenS = pl.plateFwd + pl.plateT / 2 + 0.001 - (pl.crankArm + c.web_t_mm / 2000);
  const pinL = face - (pl.couplerL - eyeB / 2 - 0.001);
  const v = (Math.PI / 4) * (0.01 ** 2 - 0.007 ** 2) * lenS + 0.014 * 0.005 * (d.drive.a + (0.01 + eyeD) / 2) + (Math.PI / 4) * eyeD ** 2 * pinL + (Math.PI / 4) * (0.015 ** 2 - 0.01 ** 2) * 0.01;
  near(d.bom.structure.find((q) => q.name.startsWith('crank:')).mass_g, v * 1e6 * 4.43, 1e-6);
});

test('round 6 parameters: parts and linkage merged over the base; a null linkage leaves round 5 carbon parts', () => {
  const p = round6Params(base(), {}, catalog);
  assert.equal(p.parts.battery, 'batt-nexus-2s1p-p30b');
  assert.equal(p.parts.gears[5], 'gear-khk-ssag1-40j10');
  assert.equal(p.joints.motorOutboard, 'bearing-skf-619-5-2z');
  assert.equal(p._linkageItems, undefined);
  const q = round6Params(base(), { linkage: null }, catalog);
  assert.equal(q.linkage, undefined);
  const d = buildDesign(q, catalog);
  assert.ok(d.bom.structure.some((s) => s.name === 'crank, rockers and gearbox plates'));
  assert.equal(d.structure.members, undefined);
});

test('round 6 linkage masses by hand: shafts, pins, rockers, housings, couplers, plates, bosses and keys', () => {
  const d = built();
  const lay = driveLayout(d, catalog);
  const pl = lay.planes;
  const L = ROUND6.linkage;
  const g = (name) => d.bom.structure.find((q) => q.name === name).mass_g;
  const rod = (dd, l) => (Math.PI / 4) * dd * dd * l;
  const ring = (D, dd, l) => (Math.PI / 4) * (D * D - dd * dd) * l;
  const TI = 4.43e6, A7 = 2.81e6, A6 = 2.7e6; // g/m3
  const shaftLen = pl.plateFwd - pl.plateAft + pl.plateT + 0.002;
  near(g('gear shaft 1'), (rod(0.008, shaftLen) + ring(0.010, 0.008, 0.010)) * TI, 1e-6);
  near(g('gear shaft 2'), rod(0.008, shaftLen) * A7, 1e-6);
  near(g('gear shaft 3'), rod(0.006, shaftLen) * A7, 1e-6);
  // Shoulder pins: 6 mm, 24 mm between bushing centres plus one 6 mm bushing.
  near(g('shoulder pin left'), rod(0.006, 0.030) * TI, 1e-6);
  // Horn pins: 8 mm through the 8 mm boss, the 6 mm eye and 2 mm.
  near(g('horn pin right'), rod(0.008, 0.016) * TI, 1e-6);
  // Rockers: hub 11/6 x 18 mm, horn arm c x 8 x 5 mm, boss 13/8 x 8 mm, spar socket 13/10 x 20 mm, in 7075.
  const c = linkageAt(lay.law, d.drive.ampNominal).c;
  near(g('rocker left'), (ring(0.011, 0.006, 0.018) + c * 0.008 * 0.005 + ring(0.013, 0.008, 0.008) + ring(0.013, 0.010, 0.020)) * A7, 1e-6);
  near(g('rocker left'), g('rocker right'), 1e-12);
  // Shoulder housings: two 12/8 x 6 mm rings a side, 6061.
  near(g('shoulder housings right, 2 off'), 2 * ring(0.012, 0.008, 0.006) * A6, 1e-6);
  // Coupler links: 5 mm body over the length less one eye diameter, two 15/10 x 6 mm eyes, 6061, two links.
  const bL = d.params.coupler_mm / 1000;
  near(g('coupler links, 2 off'), 2 * (rod(0.005, bL - 0.015) + 2 * ring(0.015, 0.010, 0.006)) * A6, 1e-6);
  // Plates: the stated area x 2 mm of 6061; bosses (3 mm wall, as long as the bushing) at the three named seats.
  near(g('gearbox plates, 2 off'), L.plates.area_m2 * 0.002 * A6, 1e-6);
  near(g('gearbox plate bosses'), (2 * ring(0.018, 0.012, 0.008) + ring(0.016, 0.010, 0.004)) * A6, 1e-6);
  // Keys: b x h x L of Ti-6Al-4V.
  near(g('key, crank wheel gear-khk-msgb1-40'), 0.005 * 0.005 * 0.010 * TI, 1e-9);
  near(g('key, pinion gear-khk-msga1-18 on gear shaft 2'), 0.003 * 0.003 * 0.010 * TI, 1e-9);
  // The seat checks use the part's outside diameter: a bossed seat 3 mm wall, the others a 5 mm ring in the plate.
  const seat = (n) => d.structure.members.find((m) => m.name === n).dims;
  assert.deepEqual(seat('plate seat crank shaft aft'), { d_mm: 12, D_mm: 18, t_mm: 10 });
  assert.deepEqual(seat('plate seat gear shaft 2 aft'), { d_mm: 10, D_mm: 20, t_mm: 2 });
  assert.deepEqual(seat('plate seat motor shaft outboard plate'), { d_mm: 13, D_mm: 23, t_mm: 2 });
});

test('cruise effective-angle amplitudes: every strip of both wings, half the range, the hook restored after', async () => {
  const { cruiseEffectiveAngles } = await import('../../src/physics/gates/suction-sensitivity.mjs');
  const { setupFlight } = await import('../../src/physics/scenarios.js');
  const d = base();
  const r = cruiseEffectiveAngles(d, { settle_s: 1, cycles: 2 });
  assert.equal(r.stations.length, 16);
  assert.equal(r.cycles, 2);
  assert.ok(r.flap_hz_measured > 0.5 && r.flap_hz_measured < 5);
  for (const s of r.stations) {
    assert.ok(s.alpha_eff_max_deg >= s.alpha_eff_min_deg);
    near(s.alpha_eff_amplitude_deg, (s.alpha_eff_max_deg - s.alpha_eff_min_deg) / 2, 0.006);
    assert.ok(s.frac_semispan > 0 && s.frac_semispan < 1);
  }
  assert.deepEqual(r.stations.filter((s) => s.wing === 'left').map((s) => s.station), [0, 1, 2, 3, 4, 5, 6, 7]);
  // A fresh flight's strip model is the module's own evaluate (nothing left patched).
  const { sim } = setupFlight(d, { init: { y: 0 } });
  assert.equal(sim.bird.strip.evaluate.name, 'evaluate');
});

test('round 7 keyways, gear side: depth from the gear maker\'s keyway, KHK\'s steel, a finite rating', () => {
  const d = built();
  const ks = d.structure.members.filter((m) => /gear side$/.test(m.name));
  assert.equal(ks.length, 4);
  const depth = Object.fromEntries(ks.map((k) => [k.name, k.dims.h_mm]));
  assert.equal(depth['key, crank wheel gear-khk-msgb1-40, gear side'], 2.3);
  assert.equal(depth['key, pinion gear-khk-msga1-18 on gear shaft 2, gear side'], 1.4);
  assert.equal(depth['key, wheel gear-khk-ssag1-40j10 on gear shaft 1, gear side'], 1.8);
  assert.equal(ks.find((k) => /msgb1-40/.test(k.name)).side_material.id, 'metal-khk-scm415');
  assert.equal(ks.find((k) => /ssag1-40j10/.test(k.name)).side_material.id, 'metal-khk-s45c');
});
