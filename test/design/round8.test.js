// Round 8 design: the airframe load path and the keyway materials, against hand calculations.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { buildDesign } from '../../src/design/build.js';
import { materialOf } from '../../src/design/linkage.js';
import { inertialTerms, addTerms, scaleTerms, rowGroup, circleGroup, hullOf, hullWidthAt, fastenerOf } from '../../src/design/airframe.js';
import { round8Params } from '../../src/design/round8.mjs';
import { LOAD_CHANNELS } from '../../src/physics/flight.js';

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const catalog = JSON.parse(readFileSync(join(ROOT, 'data/parts/catalog.json'), 'utf8'));
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);
const base = () => JSON.parse(readFileSync(join(ROOT, 'data/designs/larus-gl-2.json'), 'utf8'));
const built = (() => { let d = null; return () => (d ??= buildDesign(round8Params(base(), {}, catalog), catalog)); })();

test('keyway gear side (round 7 crushing ruling): JIS reference values, the lowest stated condition, halved', () => {
  near(materialOf(catalog, 'metal-khk-s45c').yield_MPa, 345 / 2, 1e-9, 'S45C N yield 345 (reference) at half');
  near(materialOf(catalog, 'metal-khk-scm415').yield_MPa, 830 / 4, 1e-9, 'SCM415 tensile 830 only (reference): half, and half again');
  near(materialOf(catalog, 'metal-al-6061-t6').yield_MPa, 240, 1e-9, 'EN 485-2 T6 sheet 1.5-3 mm');
});

test('fasteners and ties: ISO 898-1 M3 10.9 proof load, shear by von Mises on it; ties at the maker minimum', () => {
  const s = fastenerOf(catalog, 'screw-iso4762-m3x8-10.9');
  assert.equal(s.tension_N, 4180);
  near(s.shear_N, 4180 / Math.sqrt(3), 0.1);
  // Mass from ISO 4762 maximum head 5.5 x 3.0 mm and an 8 mm shank of 3 mm at 7.85 g/cm3.
  near(s.mass_g, (Math.PI / 4 * 5.5 ** 2 * 3 + Math.PI / 4 * 9 * 8) * 7.85e-3, 0.01);
  assert.equal(fastenerOf(catalog, 'tie-hellermanntyton-t18r').loop_N, 80);
  near(materialOf(catalog, 'adhesive-araldite-2011-aluminium').shear_MPa, 3771 * 0.00689476 / 2, 0.01);
  assert.throws(() => fastenerOf(catalog, 'nut-iso4032-m3-10'), /no rated load/);
});

test('airframe helpers by hand: inertial bound, term sums, fastener groups, hull area and width', () => {
  // 0.1 kg at 0.5 m: 0.1 sf_origin + 0.05 wdot + 0.05 w2.
  assert.deepEqual(inertialTerms(0.1, [0.3, 0.4, 0]), [{ ch: 'sf_origin', k: 0.1 }, { ch: 'wdot', k: 0.05 }, { ch: 'w2', k: 0.05 }]);
  assert.deepEqual(addTerms([{ ch: 'a', k: 1 }], [{ ch: 'a', k: -2 }, { ch: 'b', k: 0.5 }], [{ ch: 'c', k: 0 }]), [{ ch: 'a', k: 3 }, { ch: 'b', k: 0.5 }]);
  assert.deepEqual(scaleTerms([{ ch: 'a', k: 2 }], 0.25), [{ ch: 'a', k: 0.5 }]);
  assert.deepEqual(rowGroup(2, 14), { n: 2, r_max_mm: 7, sum_r2_mm2: 98 });
  assert.deepEqual(rowGroup(3, 10), { n: 3, r_max_mm: 10, sum_r2_mm2: 200 });
  assert.deepEqual(circleGroup(4, 8), { n: 4, r_max_mm: 8, sum_r2_mm2: 256 });
  // Two circles r 10 mm, 100 mm apart: a stadium, pi r^2 + 2 r L (each circle as its inscribed 72-gon).
  const h = hullOf([{ c: [0, 0], r: 0.01 }, { c: [0.1, 0], r: 0.01 }]);
  near(h.area_m2, 36 * 0.01 ** 2 * Math.sin(2 * Math.PI / 72) + 2 * 0.01 * 0.1, 1e-9);
  near(hullWidthAt(h.poly, 0), 0.12, 1e-9);
});

test('airframe in the built design: nothing unrated but the DS gear fixings, every member on known channels, parts in the BOM', () => {
  const d = built();
  const ms = d.structure.members;
  assert.ok(!ms.some((m) => m.name === 'gearbox plates between seats'), 'the plates now have their load path');
  const unrated = ms.filter((m) => m.check === 'unrated').map((m) => m.name);
  assert.ok(unrated.every((n) => /^gear-khk-ds1-\d+ on .*fixing$/.test(n)), unrated.join(', '));
  for (const m of ms) for (const s of m.sections) for (const t of s.radial) assert.ok(LOAD_CHANNELS.includes(t.ch), `${m.name}/${s.name}: ${t.ch}`);
  for (const nm of ['shoulder rail right', 'shoulder rail left', 'shoulder bracket right', 'tail boom clamp blocks, 2 off', 'forward tray', 'aft tray']) assert.ok(d.structure.items.some((i) => i.name === nm && i.mass_g > 0), nm);
  const screws = d.structure.items.find((i) => /^M3 screws/.test(i.name));
  const n = d.structure.airframe.parts.counts.screws;
  near(screws.mass_g, n * catalog.find((p) => p.id === 'screw-iso4762-m3x8-10.9').mass_g, 1e-9);
  // Every check kind the airframe uses has its rating attached.
  for (const m of ms.filter((q) => q.check === 'bolts')) assert.ok(m.fastener.tension_N > 0 && m.fastener.shear_N > 0, m.name);
  for (const m of ms.filter((q) => q.check === 'tie')) assert.ok(m.tie.loop_N > 0, m.name);
  // The boom: the moment couple between the clamps, by hand from the member's own terms.
  const fa = ms.find((q) => q.name === 'tail boom in the aft clamp, bearing').sections[0].radial;
  const ff = ms.find((q) => q.name === 'tail boom in the forward clamp, bearing').sections[0].radial;
  const Mb = ms.find((q) => q.name === 'tail boom at the aft clamp').sections.find((s) => s.name === 'M').radial;
  const s = d.structure.airframe.parts.clamp_spacing_m;
  const k = (ts, ch) => ts.find((t) => t.ch === ch)?.k ?? 0;
  near(k(ff, 'tail_F'), k(Mb, 'tail_F') / s, 1e-6);
  near(k(fa, 'tail_F'), 1 + k(Mb, 'tail_F') / s, 1e-6);
  near(k(Mb, 'tail_F'), d.structure.airframe.parts.tail_lever_m, 1e-6);
});
