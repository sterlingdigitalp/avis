// The film puts the bird together in the physics' own assembly sequence (SPEC "Completeness and assembly order": "The
// film's assembly sequence is this one"): params.layout.sequence, resolved on the physics' solids by
// src/design/sequence.js. Through v23 the film kept its own plan (solids.js assemblyPlan) and this file slid every
// step of it along its line against the parts in place; v24 retires that plan, so the same promise (the order on
// screen is one a person could follow with these parts) is now:
//  - the film's steps are the physics' steps: the same order, names, lines, distances and solids, every solid in
//    exactly one, each drawn mesh moving with the step of the solids it draws;
//  - the sequence arrives clear: the physics' check of it (sequence.js sequenceRecord) passes on the solids the film
//    draws, as the design's record says, and every step along x is swept again here, exactly, against everything
//    already placed (touching allowed, the pairs the clearance check exempts aside);
//  - on screen each step moves along its own line, in the physics' order, beat by beat (solids.js screenBeats,
//    stepTimes): nothing is reordered and no step is skipped.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird } from '../lib/bird-node.mjs';
import { fullAssembly, gap2, gap3, EXEMPT_JOINTS } from '../../src/design/clearance.js';
import { resolveSteps, sequenceRecord } from '../../src/design/sequence.js';
import { drivePose } from '../../sims/avis-shared/model.js';
import { screenBeats, stepTimes, stepFractions } from '../../sims/avis-shared/solids.js';

const { bird, model, design } = nodeBird('larus-gl-2');
const phys = bird.phys;
const seq = design.params.layout.sequence;
const N = seq.steps.length;
const catalog = Object.values(model.parts);
const { asm, lay } = fullAssembly(design, catalog);
const own = resolveSteps(asm, seq);
const keyOf = (a, b) => (a < b ? `${a}|${b}` : `${b}|${a}`);

test('the film\'s steps are the physics\' sequence: its order, names, lines, distances and solids, every solid in one step', () => {
  assert.equal(own.missing.length, 0, `solids in no step: ${own.missing.slice(0, 5)}`);
  assert.equal(own.empty.length, 0, `steps naming nothing: ${own.empty}`);
  assert.equal(phys.steps.length, N);
  assert.equal(bird.steps.length, N + 1, 'and the pod\'s covering, which the sequence exempts, last');
  assert.ok(bird.steps[N].pod, 'the covering beat');
  own.steps.forEach((st, k) => {
    assert.equal(bird.steps[k].name, seq.steps[k].name);
    assert.deepEqual(phys.steps[k].solids, st.solids, st.name);
    if (st.threaded) assert.ok(bird.steps[k].threaded && bird.steps[k].line.every((v) => v === 0), `${st.name}: threaded, not slid`);
    else bird.steps[k].line.forEach((v, i) => assert.ok(Math.abs(v - st.dir[i] * st.dist) < 1e-12, `${st.name}: its line`));
  });
  // each drawn mesh goes on with the step of the solids it draws
  const stepOf = new Map(own.steps.flatMap((st, k) => st.solids.map((t) => [t, k])));
  let checked = 0;
  for (const o of bird.meshes) {
    const tags = (o.userData.physics ?? []).flatMap((t) => (t.includes('/') ? [t] : phys.part(t).solids.map((s) => `${t}/${s.name}`)));
    if (!tags.length) continue;
    const ks = new Set(tags.map((t) => stepOf.get(t)));
    assert.ok(ks.has(bird.meshStep.get(o)), `${o.name}: drawn in step ${bird.meshStep.get(o)}, its solids in ${[...ks]}`);
    checked++;
  }
  assert.ok(checked > 200, `${checked} meshes`);
});

test('the sequence arrives clear: the physics\' own check passes on the solids the film draws, as the design records', () => {
  const rec = sequenceRecord(asm, design, lay);
  assert.deepEqual(rec.failures, []);
  assert.deepEqual(rec.missing, []);
  assert.equal(rec.passed, true);
  const saved = design.structure.clearance.assembly;
  assert.equal(saved.passed, true);
  assert.deepEqual(saved.steps.map((s) => s.name), rec.steps.map((s) => s.name));
  // the film draws exactly these solids (physics-solids.test.mjs): the same parts, the same solid names
  assert.deepEqual(phys.parts.map((p) => `${p.id}:${p.solids.map((s) => s.name).join(',')}`), asm.parts.map((p) => `${p.id}:${p.solids.map((s) => s.name).join(',')}`));
});

test('every step along x sweeps clear of everything already placed (exactly: its section over its swept x range)', () => {
  const pose = seq.pose ?? { phase_rad: 0 };
  const q = drivePose({ law: lay.law, fourBar: lay.fb }, pose.phase_rad, pose.amp ?? design.drive.ampNominal);
  const byId = new Map(asm.parts.flatMap((p) => p.solids.map((s) => [`${p.id}/${s.name}`, { part: p.id, id: `${p.id}/${s.name}`, x: s.x, sec: s.at ? s.at(q) : s.shape }])));
  const joins = new Map(asm.joins.map(([a, b, w]) => [keyOf(a, b), w]));
  const exempt = (a, b) => a.part === b.part || [keyOf(a.id, b.id), keyOf(a.id, b.part), keyOf(a.part, b.id), keyOf(a.part, b.part)].some((k) => EXEMPT_JOINTS.includes(joins.get(k)));
  const placed = [], bad = [];
  let swept = 0;
  for (const st of own.steps) {
    const moving = st.solids.map((t) => byId.get(t));
    if (!st.threaded && Math.abs(st.dir[1]) < 1e-12 && Math.abs(st.dir[2]) < 1e-12) {
      const dx = st.dir[0] * st.dist;
      for (const m of moving) {
        const xs = [Math.min(m.x[0], m.x[0] + dx), Math.max(m.x[1], m.x[1] + dx)];
        for (const t of placed) {
          if (exempt(m, t) || xs[1] < t.x[0] || t.x[1] < xs[0]) continue;
          const g = gap3(xs, t.x, gap2(m.sec, t.sec));
          if (g < -1e-7) bad.push(`${st.name}: ${m.id} through ${t.id} by ${(-g * 1000).toFixed(3)} mm`);
        }
        swept++;
      }
    }
    placed.push(...moving);
  }
  assert.deepEqual(bad, []);
  assert.ok(swept > 100, `${swept} solids swept`);
});

test('on screen: the beats are runs of the physics\' steps in order; every step moves in its turn, along its line', () => {
  const { beats, times } = bird.assembly;
  assert.deepEqual(beats.slice(0, -1).flatMap((b) => b.steps), Array.from({ length: N }, (_, k) => k), 'every step once, in order');
  assert.ok(beats.at(-1).film && beats.at(-1).steps[0] === N, 'the covering\'s beat last');
  assert.ok(beats.length < N / 2, `${beats.length} beats for ${N} steps`);
  // each step lands after the one before it and starts no earlier
  for (let k = 1; k < times.length; k++) {
    assert.ok(times[k][1] > times[k - 1][1] && times[k][0] >= times[k - 1][0], `step ${k} in its turn`);
    assert.ok(times[k][0] < times[k][1]);
  }
  assert.equal(times[0][0], 0);
  assert.equal(times.at(-1)[1], 1);
  // the motor's step: out along its line before it, home after it
  const k = bird.steps.findIndex((s) => s.nodes.some((n) => n.name === 'motor'));
  const motor = bird.root.getObjectByName('motor');
  bird.update({ phase: 0, amp: model.law.ampNominal, build: 1 });
  const home = motor.position.clone();
  bird.update({ phase: 0, amp: model.law.ampNominal, build: times[k][0] });
  assert.ok(motor.position.clone().sub(home).distanceTo(new THREE.Vector3(...bird.steps[k].line)) < 1e-12, 'the motor fully out as its step begins');
  bird.update({ phase: 0, amp: model.law.ampNominal, build: times[k][1] });
  assert.ok(motor.position.distanceTo(home) < 1e-12, 'and home as it ends');
  const ex = bird.buildExplode(0.5);
  for (const sub of ['structure', 'drive', 'wing']) assert.ok(ex[sub] >= 0 && ex[sub] <= 1, sub);
  assert.equal(bird.buildExplode(1).drive, 0);
  bird.update({ phase: 0, amp: model.law.ampNominal });
});

test('the beats group a part with its fixings, a run of one kind, one side\'s linkage and the parts onto one carrier', () => {
  const { beats } = bird.assembly;
  const beatOf = (name) => beats.findIndex((b) => b.steps.some((k) => seq.steps[k]?.name === name));
  const together = (...names) => names.every((n) => beatOf(n) === beatOf(names[0]) && beatOf(n) >= 0);
  assert.ok(together('aft tray', 'aft tray screws', 'aft tray nuts'), 'a tray with its screws and nuts');
  assert.ok(together('rail lock screws', 'rail lock nuts') && beatOf('rail lock screws') !== beatOf('side panel left'), 'a run of fixings of one kind, its own beat when it fixes nothing of the beat before');
  assert.ok(together('former 2', 'former 1', 'former 5', 'former 1 tie', 'former 5 tie'), 'the formers and their ties');
  assert.ok(together('wing R', 'wing L') && together('tail surface R', 'tail surface L'), 'mirrored pairs');
  assert.ok(together('coupler R with its bushings', 'rocker R on its shoulder pin', 'horn pin R') && beatOf('coupler L with its bushings') !== beatOf('horn pin R'), 'one side\'s linkage');
  assert.ok(together('esc', 'esc ties', 'battery', 'battery ties') && beatOf('gps') !== beatOf('esc'), 'the parts onto one carrier');
  assert.ok(!together('aft plate with its journals', 'crank shaft with its wheel and key'), 'different parts apart');
  // the rules on made-up steps: a later design's crank-web screw goes on with the web it fixes
  const fake = {
    steps: [{ name: 'web', solids: ['web/w'] }, { name: 'web screw', solids: ['screw/head'] }, { name: 'plate', solids: ['plate/p'] }],
    lines: [{ key: 'web', kind: 'structure', id: 'metal' }, { key: 'M3 screws', kind: 'structure', id: 'screw-x' }, { key: 'plate', kind: 'structure', id: 'metal' }],
    joins: [['screw', 'web', 'through']],
  };
  const tags = { 'web/w': 'web', 'screw/head': 'M3 screws', 'plate/p': 'plate' };
  Object.assign(fake, { solid: (id, name) => ({ bom: tags[`${id}/${name}`] }), line: (t) => fake.lines.find((l) => l.key === t) ?? null });
  assert.deepEqual(screenBeats(fake, { 'screw-x': { category: 'fastener' }, metal: { category: 'metal_bar' } }).map((b) => b.steps), [[0, 1], [2]]);
});

test('timing: beats one after another, the steps of a beat staggered; each step eased out of its line', () => {
  const t = stepTimes([{ steps: [0] }, { steps: [1, 2] }], 1);
  assert.deepEqual(t[0], [0, 1 / 3]);
  assert.ok(Math.abs(t[1][0] - 1 / 3) < 1e-12 && Math.abs(t[1][1] - (1 / 3 + 2 / 9)) < 1e-12);
  assert.ok(Math.abs(t[2][0] - (1 / 3 + 1 / 9)) < 1e-12 && Math.abs(t[2][1] - 2 / 3) < 1e-12);
  assert.deepEqual(t[3], [2 / 3, 1]);
  assert.deepEqual(stepFractions(t, 0), [1, 1, 1, 1]);
  assert.deepEqual(stepFractions(t, 1), [0, 0, 0, 0]);
  const mid = stepFractions([[0, 1]], 0.25)[0];
  assert.ok(Math.abs(mid - (1 - 0.25 * 0.25 * 2.5)) < 1e-12, 'smoothstep in the step');
});

test('the film\'s assembly: the rows lit as their steps land; the camera on the gearbox, out as the boom goes on', async () => {
  const { assemblyExplode, assemblyOrbit, assemblyWide, assemblyStep, assemblyHold, stepLine } = await import('../../sims/avis-shared/film.js');
  const a0 = assemblyExplode(bird, 0), a1 = assemblyExplode(bird, 0.9);
  assert.equal(a0.build, 0);
  assert.deepEqual(Object.keys(a0.ex), ['structure', 'drive', 'power', 'avionics', 'wing']);
  for (const v of Object.values(a0.ex)) assert.ok(v > 0.9, 'all out at the start');
  for (const v of Object.values(a1.ex)) assert.equal(v, 0, 'all in by u 0.86');
  const old = assemblyExplode({ buildExplode: () => null }, 0.3);
  assert.equal(old.build, null);
  assert.ok(old.ex.structure < old.ex.wing);
  // out as the first step whose part reaches beyond the gearbox's close-up begins: the tail boom's
  const wide = assemblyWide(bird), times = bird.assembly.times;
  const k = seq.steps.findIndex((s) => s.name === 'tail boom');
  assert.ok(Math.abs(wide - 0.86 * times[k][0]) < 1e-12, `out at ${wide}`);
  const P = model.planes;
  const [t0, , , d0] = assemblyOrbit(0, wide), [t1, , , d1] = assemblyOrbit(Math.min(1, wide + 0.17), wide);
  assert.ok(t0.x > P.plateAft && t0.x < P.plateFwd && d0 < 1, `${t0.x} at ${d0} m`);
  assert.ok(Math.abs(t1.x + 0.15) < 1e-12 && t1.y === 0 && d1 > 2, `${t1.x} at ${d1} m`);
  assert.equal(assemblyWide(null), 0.68);
  // the panel names the step going on, numbered in the physics' order
  assert.deepEqual(assemblyStep(bird, times[5][0] + 1e-9), { k: 6, n: N, name: seq.steps[5].name });
  assert.equal(assemblyStep(bird, 1), null);
  assert.equal(assemblyStep({ assembly: null }, 0.5), null);
  assert.ok(stepLine(assemblyStep(bird, times[5][0] + 1e-9)).includes(`step 6 of ${N} · ${seq.steps[5].name}`) && stepLine(null) === '');
  // held at the pose the sequence states while it goes together; a bird without one, level
  assert.deepEqual(assemblyHold(bird, 9, 0.5), { phase: seq.pose.phase_rad, amp: seq.pose.amp ?? model.law.ampNominal });
  assert.deepEqual(assemblyHold({ phys: null }, 9, 0.5), { phase: 9, amp: 0.5 });
});

test('building: a part is drawn from its step on, moved along its line; a tie threaded in place, never slid; all drawn when not building', () => {
  const { times } = bird.assembly, amp = model.law.ampNominal;
  const seen = (o) => o.visible;
  const at = (name) => seq.steps.findIndex((s) => s.name === name);
  const meshOf = (tag) => bird.meshes.find((o) => (o.userData.physics ?? []).some((t) => t === tag || t.startsWith(`${tag}/`)));
  const k = at('gear shaft 1 with its gears and keys');
  bird.update({ phase: 0, amp, build: (times[k][0] + times[k][1]) / 2 });
  bird.root.updateMatrixWorld(true);
  assert.ok(seen(bird.root.getObjectByName('motor_bell_can')) && seen(bird.root.getObjectByName('shaft_1')), 'before it and itself drawn');
  assert.ok(!seen(bird.root.getObjectByName('plate_fwd')) && !seen(bird.root.getObjectByName('shell')), 'after it not yet');
  const g = bird.root.getObjectByName('gear_s3_wheel'), f = stepFractions([times[k]], (times[k][0] + times[k][1]) / 2)[0];
  const out = new THREE.Vector3().setFromMatrixPosition(g.matrixWorld);
  bird.update({ phase: 0, amp, build: 1 });
  bird.root.updateMatrixWorld(true);
  const home = new THREE.Vector3().setFromMatrixPosition(g.matrixWorld);
  assert.ok(out.clone().sub(home).distanceTo(new THREE.Vector3(...bird.steps[k].line).multiplyScalar(f)) < 1e-9, `posed gear out by ${out.x - home.x}`);
  // a threaded tie: not drawn before its step, drawn from its start, where it is tied
  const kt = at('esc ties'), tie = meshOf('packed esc ties');
  bird.update({ phase: 0, amp, build: times[kt][0] - 1e-6 });
  assert.ok(!seen(tie), 'not before its step');
  const pos = tie.position.clone();
  bird.update({ phase: 0, amp, build: (times[kt][0] + times[kt][1]) / 2 });
  assert.ok(seen(tie) && tie.position.distanceTo(pos) < 1e-12, 'threaded in place');
  // the wings only in their step; the pod's covering only in its own beat, last
  const kw = at('wing R');
  bird.update({ phase: 0, amp, build: times[kw][0] - 1e-6 });
  assert.ok(bird.meshes.filter((o) => o.userData.subsystem === 'wing').every((o) => !o.visible), 'the wings not yet');
  bird.update({ phase: 0, amp, build: times[N][0] - 1e-6 });
  assert.ok(!seen(bird.root.getObjectByName('shell')), 'no covering before its beat');
  bird.update({ phase: 0, amp, build: (times[N][0] + times[N][1]) / 2 });
  assert.ok(seen(bird.root.getObjectByName('shell')), 'the covering going on');
  // not building: every part drawn, the structure under a pigmented covering aside (bird.js markUnder)
  bird.update({ phase: 0, amp, explode: { drive: 1, structure: 1, wing: 1 } });
  assert.ok(seen(bird.root.getObjectByName('frame_rail_right')) && seen(bird.root.getObjectByName('shaft_1')) && seen(meshOf('rail lock right screw 1')) && seen(bird.root.getObjectByName('pushrod_R')));
  bird.update({ phase: 0, amp });
});
