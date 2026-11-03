// The film's corridor log is the physics corridor scenario, flown again from the current design and flight model:
// the same seed, speed and wind as src/physics/gates/corridor.mjs (read from that script's text), and a log on disk
// counts as current only when it was flown from exactly the design, flight model and corridor there now.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { CORRIDOR_SCENARIO, corridorInputs, isCurrent, readInputs, physicsSources, ensureCorridorLog } from '../flight_log.mjs';

const ROOT = path.join(path.dirname(new URL(import.meta.url).pathname), '..', '..');
const src = fs.readFileSync(path.join(ROOT, 'src/physics/gates/corridor.mjs'), 'utf8');

test('the scenario is corridor.mjs\'s: seed, speed, mean wind, turbulence and its scale height', () => {
  const wind = src.match(/createWind\(\{\s*mean:\s*\[([^\]]+)\],\s*uRms:\s*([\d.]+),\s*agl:\s*([\d.]+)[^)]*\},\s*(\d+)\)/);
  const run = src.match(/runCorridor\(d,\s*corridor,\s*\{\s*seed:\s*(\d+),\s*wind:\s*w,\s*speed:\s*([\d.]+)\s*\}\)/);
  assert.ok(wind && run, 'corridor.mjs no longer reads as expected: compare the scenario by hand');
  assert.deepEqual(wind[1].split(',').map(Number), CORRIDOR_SCENARIO.wind.mean);
  assert.equal(+wind[2], CORRIDOR_SCENARIO.wind.uRms);
  assert.equal(+wind[3], CORRIDOR_SCENARIO.wind.agl);
  assert.equal(+wind[4], CORRIDOR_SCENARIO.seed);
  assert.equal(+run[1], CORRIDOR_SCENARIO.seed);
  assert.equal(+run[2], CORRIDOR_SCENARIO.speed);
});

test('a log is current only for the inputs it was flown from', () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'avis-flightlog-'));
  try {
    const inputs = corridorInputs('larus-gl-2');
    const f = path.join(dir, 'log.json');
    assert.equal(isCurrent(f, inputs), false, 'missing');
    fs.writeFileSync(f, JSON.stringify({ inputs, design: 'larus-gl-2', columns: { t: [0, 1] } }, null, 1));
    assert.equal(isCurrent(f, inputs), true);
    assert.deepEqual(readInputs(f), inputs);
    for (const k of ['design_sha256', 'physics_sha256', 'corridor_sha256']) assert.equal(isCurrent(f, { ...inputs, [k]: 'x' }), false, k);
    assert.equal(isCurrent(f, { ...inputs, scenario: { ...inputs.scenario, seed: 8 } }), false, 'scenario');
    fs.writeFileSync(f, JSON.stringify({ design: 'larus-gl-2', columns: { t: [0, 1] } }));
    assert.equal(isCurrent(f, inputs), false, 'a log without inputs (as corridor.mjs writes it) is stale');
  } finally {
    fs.rmSync(dir, { recursive: true, force: true });
  }
});

test('the flight model\'s sources are every module of src/physics, not the gate scripts', () => {
  const names = physicsSources().map((f) => path.basename(f));
  assert.ok(names.includes('flight.js') && names.includes('scenarios.js') && names.includes('kinematics.js'));
  assert.ok(names.every((n) => n.endsWith('.js')));
  assert.deepEqual(names, [...names].sort());
});

test('a stale log is flown again from the design and the flight model, and is then current', async () => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'avis-flightroot-'));
  try {
    fs.mkdirSync(path.join(root, 'src'), { recursive: true });
    fs.symlinkSync(path.join(ROOT, 'src', 'physics'), path.join(root, 'src', 'physics'));
    for (const f of ['data/designs/larus-gl-2.json', 'data/env/corridor.json']) {
      fs.mkdirSync(path.dirname(path.join(root, f)), { recursive: true });
      fs.copyFileSync(path.join(ROOT, f), path.join(root, f));
    }
    assert.equal((await ensureCorridorLog('larus-gl-2', { check: true, root })).status, 'stale');
    const r = await ensureCorridorLog('larus-gl-2', { root });
    assert.equal(r.status, 'written');
    assert.ok(r.completed && r.span_s > 60 && r.fly_s > 0 && r.fly_s < 600, `${r.span_s} s, ${r.fly_s} s to fly`);
    const log = JSON.parse(fs.readFileSync(r.file, 'utf8'));
    assert.ok(Math.abs(r.span_s - (log.columns.t.at(-1) - log.columns.t[0])) < 1e-9);
    assert.equal(Object.keys(log)[0], 'inputs');
    assert.equal(log.scenario, 'larus-gl-2-corridor');
    assert.equal(log.columns.t.length, log.columns.pos.length);
    assert.ok(!fs.existsSync(`${r.file}.tmp`));
    assert.equal((await ensureCorridorLog('larus-gl-2', { root })).status, 'current');
    assert.equal((await ensureCorridorLog('larus-gl-2', { root, check: true, force: true })).status, 'stale');
  } finally {
    fs.rmSync(root, { recursive: true, force: true });
  }
});

test('the command line: --check says current or stale, with the design and flight-model hashes, and exits 0 or 3', () => {
  const r = spawnSync(process.execPath, [path.join(ROOT, 'film', 'flight_log.mjs'), 'larus-gl-2', '--check'], { encoding: 'utf8' });
  const m = r.stdout.match(/larus-gl-2-corridor\.json: (current|stale) \(design ([0-9a-f]{12}), physics ([0-9a-f]{12})\)/);
  assert.ok(m, r.stdout + r.stderr);
  const inputs = corridorInputs('larus-gl-2');
  assert.equal(m[2], inputs.design_sha256.slice(0, 12));
  assert.equal(m[3], inputs.physics_sha256.slice(0, 12));
  assert.equal(r.status, m[1] === 'current' ? 0 : 3);
});
