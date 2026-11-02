import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { buildDesign, gearRatioOf, MARGIN } from '../../src/design/build.js';
import { efficientBandTop, motorContinuous, tubeAllowable, isEligible, servoAt } from '../../src/design/catalog.js';
import { eligibleParts } from '../../src/design/screen.js';
import { validateDesign } from '../../src/design/schema.js';
import { trimGlide } from '../../src/design/trim.js';
import { buildBird } from '../../src/physics/flight.js';
import { designAtmosphere, G0 } from '../../src/physics/atmosphere.js';
import { POLARS, polarEval } from '../../src/physics/polars.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url)));
const has = (id) => catalog.some((p) => p.id === id);
const gears = ['gear-khk-ds0.8-12', 'gear-khk-ds0.8-60', 'gear-khk-ds0.8-20', 'gear-khk-ds0.8-80'];
const params = {
  candidate: 'larus', span_m: 1.8, AR: 8, taper: 0.55, elbowFrac: 0.5, elbowGain: 0.5, amp_deg: 15, ampMax_deg: 24, phiMean_deg: 5,
  gearRatio: 20, staticMargin: 0.1, sparFrac: 0.35, crank_mm: 8, coupler_mm: 70,
  parts: { motor: 'motor-eflite-park370-1080', esc: 'esc-hobbywing-skywalker-20a-v2-mini', battery: 'batt-gensace-1000-3s-25c', twistServo: 'servo-kst-x06-v6', tailServo: 'servo-kst-x06-v6', strokeServo: 'servo-kst-x06-v6', fc: 'fc-ark-v6x', gps: 'gps-holybro-m10', airspeed: 'aspd-matek-4525', receiver: 'rx-matek-elrs-r24-p6', sparInner: 'tube-ec-woven-16x14', sparOuter: 'tube-ec-woven-8x6', leRod: 'rod-ec-2mm', teRod: 'rod-ec-1-5mm', ribMaterial: 'balsa-wood-database', film: 'film-oracover-air-light', keel: 'tube-ec-woven-8x6', gears, bearing: 'bearing-skf-624-2z' },
};
const ready = Object.values(params.parts).flat().every(has);

test('SPEC rules on ratings and strength', () => {
  assert.equal(efficientBandTop('4 - 7 A (>74%) |'), 7);
  assert.equal(efficientBandTop('2,5 - 7 A (>75%) |'), 7);
  assert.equal(efficientBandTop('8-12A'), 12);
  // Published continuous wins; otherwise min(band top, 50% burst)
  assert.equal(motorContinuous({ specs: { continuous_current_a: 9, max_efficiency_current_text: '1 - 20 A' } }).amps, 9);
  assert.equal(motorContinuous({ specs: { continuous_current_a: null, max_efficiency_current_text: '4 - 7 A', current_capacity_a: 10 } }).amps, 5);
  assert.equal(motorContinuous({ specs: { continuous_current_a: null } }).amps, null);
  // Woven tube 650 MPa; rods the lower of their range; roll-wrapped none
  assert.equal(tubeAllowable({ specs: { tensile_strength_lengthways_mpa: 650, tensile_modulus_lengthways_gpa: 64 } }).sigma_MPa, 650);
  assert.equal(tubeAllowable({ specs: { tensile_strength_lengthways_mpa_range: [400, 500] } }).sigma_MPa, 400);
  assert.equal(tubeAllowable({ specs: { tensile_strength_mpa: null } }).sigma_MPa, null);
  assert.equal(isEligible({ specs: { stock_note: 'Page read shows Sold Out flags.' } }), false);
  assert.equal(isEligible({ specs: { mass_variants_g: { 'Model A legacy (sold out)': 39 } } }), true);
  // Only servos with a published stall current are eligible
  for (const s of eligibleParts(catalog).servos) assert.ok(s.specs.stall_current?.length);
});

test('servo performance interpolates between published voltage rows', () => {
  const p = { specs: { performance: [{ voltage_v: 4, torque_nm: 0.1, speed_s_per_60deg: 0.2 }, { voltage_v: 6, torque_nm: 0.2, speed_s_per_60deg: 0.1 }] } };
  const s = servoAt(p, 5);
  assert.ok(Math.abs(s.stall_Nm - 0.15) < 1e-12 && Math.abs(s.speed_rad_s - Math.PI / 3 / 0.15) < 1e-12);
});

test('hand calculation: mass from the BOM and the structure, plus 15%', { skip: !ready && 'catalogue lacks a fixture part' }, () => {
  const d = buildDesign(params, catalog);
  assert.deepEqual(validateDesign(d), []);
  // Parts: catalogue masses times counts, summed by hand here from the catalogue directly
  const m = (id) => catalog.find((p) => p.id === id).mass_g;
  const P = params.parts;
  const parts = m(P.motor) + m(P.esc) + m(P.battery) + 2 * m(P.twistServo) + 2 * m(P.tailServo) + m(P.strokeServo) + m(P.fc) + m(P.gps) + m(P.airspeed) + m(P.receiver) + gears.reduce((a, g) => a + m(g), 0) + 8 * m(P.bearing);
  assert.ok(Math.abs(d.bom.parts_g - parts) < 1e-9);
  // Spars: grams per metre (mass_basis per_metre) times length, two wings
  const semi = params.span_m / 2 - 0.025;
  const inner = d.wing.segments[0].length;
  assert.equal(catalog.find((p) => p.id === P.sparInner).specs.mass_basis, 'per_metre');
  const sparItem = d.bom.structure.find((s) => s.name.startsWith('inner spar'));
  assert.ok(Math.abs(sparItem.mass_g - 2 * m(P.sparInner) * inner) < 1e-9);
  // Film: g/m2 times both faces of the wing area with 3% allowance
  const film = d.bom.structure.find((s) => s.name.startsWith('wing film'));
  assert.ok(Math.abs(film.mass_g - catalog.find((p) => p.id === P.film).specs.areal_density_g_m2 * 2 * d.geometry.area_m2 * 1.03) < 1e-9);
  // Margin and total
  assert.ok(Math.abs(d.bom.margin_g - MARGIN * (d.bom.parts_g + d.bom.structure_g)) < 1e-9);
  assert.ok(Math.abs(d.mass.total_g - 1.15 * (d.bom.parts_g + d.bom.structure_g)) < 1e-6);
  // Wing loading W/S by hand
  assert.ok(Math.abs(d.geometry.wing_loading_N_m2 - (d.mass.total_kg * G0) / d.geometry.area_m2) < 1e-9);
  // Span and area by hand: S = b^2/AR exactly as asked
  assert.ok(Math.abs(d.geometry.area_m2 - (1.8 * 1.8) / 8) < 1e-9);
  assert.ok(semi > 0.8 && d.geometry.span_m === 1.8);
});

test('hand calculation: level-flight power in a parked glide equals W x sink, close to D V by drag polar', { skip: !ready && 'catalogue lacks a fixture part' }, () => {
  const d = buildDesign(params, catalog);
  const bird = buildBird(d);
  const V = 8;
  const g = trimGlide(d, bird, { V, amp: 15 * Math.PI / 180 });
  assert.ok(g.converged);
  const W = d.mass.total_kg * G0;
  const Pmodel = W * g.sink;
  // Hand: D = q S (cd0 + CL^2 / (pi AR e)) + body; cd0 from the section polar at the mean chord Reynolds number
  const atm = designAtmosphere();
  const q = 0.5 * atm.rho * V * V;
  const S = d.geometry.area_m2;
  const CL = W / (q * S);
  const out = new Float64Array(4);
  const Re = (V * d.geometry.mac_m) / atm.nu;
  polarEval(POLARS[d.wing.airfoil], CL / 5.5 - 3.2 * Math.PI / 180, Re, 1, 1, out);
  const tailS = d.tail.area_total_m2;
  const D = q * S * (out[1] + (CL * CL) / (Math.PI * d.geometry.AR * 0.9)) + q * tailS * 0.025 + q * d.body.CdA_front_m2;
  const Phand = D * V;
  // Same physics, different route: a strip model with stall, lag and tail versus a drag polar by hand.
  assert.ok(Math.abs(Pmodel / Phand - 1) < 0.35, `model ${Pmodel.toFixed(2)} W, hand ${Phand.toFixed(2)} W`);
});
