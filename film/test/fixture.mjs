// A synthetic flight log in the SPEC format, so the tests do not depend on files under out/.
import { writeFileSync, mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

export function syntheticLog(n = 240, dt = 1 / 120) {
  const r4 = (v) => +v.toFixed(4);
  const c = { t: [], pos: [], quat: [], vel: [], phase: [], amplitude: [], twist_l: [], twist_r: [], tail: [], airspeed: [], power_w: [], battery_pct: [], wind: [] };
  for (let k = 0; k < n; k++) {
    const t = 0.002 + k * dt;
    c.t.push(r4(t));
    c.pos.push([r4(9.5 * t), 0, 0]);
    c.quat.push([0, 0, 0, 1]);
    c.vel.push([9.5, 0, 0]);
    c.phase.push(r4((2 * Math.PI * 3 * t) % (2 * Math.PI)));
    c.amplitude.push(r4(0.35 + 0.05 * Math.sin(t)));
    c.twist_l.push(r4(0.3 * Math.sin(2 * Math.PI * 3 * t)));
    c.twist_r.push(r4(-0.3 * Math.sin(2 * Math.PI * 3 * t)));
    c.tail.push([r4(0.05 * t), r4(-0.05 * t)]);
    c.airspeed.push(9.5);
    c.power_w.push(40);
    c.battery_pct.push(100 - t);
    c.wind.push([0, 0, 0]);
  }
  return { design: 'larus-gl-2', scenario: 'synthetic', dt, columns: c };
}

export function syntheticLogFile(n, dt) {
  const dir = mkdtempSync(join(tmpdir(), 'avis-film-test-'));
  const f = join(dir, 'log.json');
  writeFileSync(f, JSON.stringify(syntheticLog(n, dt)));
  return f;
}
