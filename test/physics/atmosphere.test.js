import test from 'node:test';
import assert from 'node:assert/strict';
import { isa, designAtmosphere } from '../../src/physics/atmosphere.js';

test('ISA sea level matches ICAO table', () => {
  const a = isa(0);
  assert.ok(Math.abs(a.rho - 1.225) < 0.001);
  assert.ok(Math.abs(a.a - 340.3) < 0.1);
  assert.ok(Math.abs(a.mu - 1.789e-5) < 0.01e-5);
});

test('ISA 2000 m matches the published table (79495 Pa, 1.0066 kg/m3, 275.15 K)', () => {
  const a = isa(2000);
  assert.ok(Math.abs(a.p - 79495) < 30, `p=${a.p}`);
  assert.ok(Math.abs(a.rho - 1.0066) < 0.001);
  assert.ok(Math.abs(a.T - 275.15) < 0.01);
});

test('hand calculation of the design condition: 1800 m, 10 C', () => {
  // p = 101325 (276.45/288.15)^5.25588 = 81.5 kPa ; rho = p / (287.05 * 283.15)
  const hand_p = 101325 * Math.pow(276.45 / 288.15, 5.25588);
  const hand_rho = hand_p / (287.05 * 283.15);
  const d = designAtmosphere();
  assert.ok(Math.abs(d.T - 283.15) < 1e-9);
  assert.ok(Math.abs(d.p - hand_p) < 5);
  assert.ok(Math.abs(d.rho - hand_rho) < 1e-3);
  const loss = 1 - d.rho / 1.225;
  assert.ok(loss > 0.17 && loss < 0.19, `density loss ${loss}, SPEC says about 18%`);
});
