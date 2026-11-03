// The order the assembly sequence puts the bird together (sims/avis-shared/solids.js assemblyPlan) must be one a
// person could follow with these parts: every step slides in along its own straight line, and on the way it may touch
// (a shaft through its journal, a gear onto its seat) but never pass through a part already in place. Checked on the
// physics' own solids (src/design/clearance.js), the moving ones at the pose the film holds the assembly in, sixteen
// points along each line. A conflict the design itself leaves (no order can avoid it) is listed in REPORTED with its
// reason, and must still occur, so the list is pruned when the design is corrected.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird, levelPhase } from '../lib/bird-node.mjs';
import { gap2, gap3 } from '../../src/design/clearance.js';
import { drivePose } from '../../sims/avis-shared/model.js';
import { assemblyPlan, shiftSection, stepOf, stepFractions } from '../../sims/avis-shared/solids.js';

const { bird, model } = nodeBird('larus-gl-2');
const phys = bird.phys;
const amp = model.law.ampNominal;
const q = drivePose({ law: phys.lay.law, fourBar: phys.lay.fb }, levelPhase(model, amp), amp);
const plan = assemblyPlan(phys);
// v22: the one-piece crank (web aft of the aft plate, a 15 mm wheel seat inside the box) cannot pass the aft plate's
// 12 mm journal seat, nor its bushing, in either direction: reported to the physics (no order fits it)
const REPORTED = new Set(['crank | gearbox plate aft', 'crank | crank shaft journal aft (bushing-igus-GFM-1012-10)']);

const sectionAt = (s) => (s.at ? s.at(q) : s.shape);
function solidsOf(tag) {
  const [id, name] = [tag.split('/')[0], tag.includes('/') ? tag.slice(tag.indexOf('/') + 1) : null];
  return phys.part(id).solids.filter((s) => !name || s.name === name).map((s) => ({ part: id, name: s.name, x: s.x, sec: sectionAt(s) }));
}
function bbox(sec) {
  const pts = sec.kind === 'ring' ? [[sec.c[0] - sec.ro, sec.c[1] - sec.ro], [sec.c[0] + sec.ro, sec.c[1] + sec.ro]]
    : sec.kind === 'cap' ? [[Math.min(sec.a[0], sec.b[0]) - sec.r, Math.min(sec.a[1], sec.b[1]) - sec.r], [Math.max(sec.a[0], sec.b[0]) + sec.r, Math.max(sec.a[1], sec.b[1]) + sec.r]]
      : [[Math.min(...sec.pts.map((p) => p[0])), Math.min(...sec.pts.map((p) => p[1]))], [Math.max(...sec.pts.map((p) => p[0])), Math.max(...sec.pts.map((p) => p[1]))]];
  return pts;
}
const apart = (a, b) => a[1][0] < b[0][0] || b[1][0] < a[0][0] || a[1][1] < b[0][1] || b[1][1] < a[0][1];

test('every physics part arrives in exactly one step, every step names parts', () => {
  const seen = new Map();
  for (const p of phys.parts) for (const s of p.solids) {
    const k = stepOf(plan, `${p.id}/${s.name}`);
    assert.ok(k >= 0, `${p.id}/${s.name} in no step`);
    seen.set(`${p.id}/${s.name}`, k);
  }
  for (const s of plan) assert.ok(s.members.length, s.name);
  // the order the brief's v22 changes need: the motor inside the box before the forward plate closes it, stage 1
  // (overhung) after it, the rails before the forward plate, each coupler before its own aft shoulder bracket
  const at = (n) => plan.findIndex((s) => s.name === n);
  assert.ok(at('motor') < at('forward plate') && at('forward plate') < at('stage 1 and encoder') && at('rails') < at('forward plate'));
  assert.ok(at('motor') < at('gear train'), 'the motor in before the gears it would pass');
  for (const t of ['R', 'L']) assert.ok(at(`coupler ${t}`) < at(`shoulder brackets ${t}`) && at(`shoulder brackets ${t}`) < at(`rocker ${t}`) && at(`rocker ${t}`) < at(`shoulder pin ${t}`));
});

test('no step slides through a part already in place (touching allowed: a shaft in its journal, a gear onto its seat)', () => {
  const placed = [], found = new Set(), bad = [];
  plan.forEach((step) => {
    const moving = step.members.flatMap(solidsOf);
    for (const m of moving) {
      const d = step.dirOf(m.part);
      for (const t of placed) {
        if (t.part === m.part) continue;
        for (let i = 1; i <= 16; i++) {
          const f = i / 16, x = [m.x[0] + d[0] * f, m.x[1] + d[0] * f];
          if (x[1] <= t.x[0] || x[0] >= t.x[1]) continue;
          const sec = shiftSection(m.sec, [d[2] * f, d[1] * f]);
          if (apart(bbox(sec), bbox(t.sec))) continue;
          const g = gap3(x, t.x, gap2(sec, t.sec));
          if (g < -1e-7) {
            const key = `${m.part} | ${t.part}`;
            if (REPORTED.has(key)) found.add(key);
            else bad.push(`${step.name}: ${m.part}/${m.name} through ${t.part}/${t.name} by ${(-g * 1000).toFixed(2)} mm at ${(f * 100).toFixed(0)}% out`);
            break;
          }
        }
      }
    }
    placed.push(...moving);
  });
  assert.deepEqual([...new Set(bad)], []);
  assert.deepEqual([...found].sort(), [...REPORTED].sort(), 'every reported conflict still occurs');
});

test('the film plays the plan: at build progress p each step is out by stepFractions, the drive and frame nodes along their lines', () => {
  assert.ok(bird.steps.length > plan.length, 'the packed parts, the pod and the wings after the physics\' steps');
  plan.forEach((s, k) => assert.equal(bird.steps[k].name, s.name));
  const f = stepFractions(bird.steps.length, 0.5);
  assert.ok(f.some((v) => v === 0) && f.some((v) => v === 1), 'half way: some steps in, some out');
  const motor = bird.root.getObjectByName('motor');
  bird.update({ phase: 0, amp, build: 1 });
  const home = motor.position.clone();
  const k = plan.findIndex((s) => s.name === 'motor');
  bird.update({ phase: 0, amp, build: k / bird.steps.length });
  assert.ok(Math.abs(motor.position.x - home.x - plan[k].dir[0]) < 1e-12, 'the motor fully out before its step');
  bird.update({ phase: 0, amp, build: (k + 1) / bird.steps.length });
  assert.ok(motor.position.distanceTo(home) < 1e-12, 'and home after it');
  const ex = bird.buildExplode(0.5);
  for (const sub of ['structure', 'drive', 'wing']) assert.ok(ex[sub] >= 0 && ex[sub] <= 1, sub);
  assert.equal(bird.buildExplode(1).drive, 0);
  bird.update({ phase: 0, amp });
});

test('the film\'s assembly: the steps evenly over the sequence, the rows lit as their steps land, the camera on the gearbox first', async () => {
  const { assemblyExplode, assemblyOrbit } = await import('../../sims/avis-shared/film.js');
  const a0 = assemblyExplode(bird, 0), a1 = assemblyExplode(bird, 0.9);
  assert.equal(a0.build, 0);
  assert.deepEqual(Object.keys(a0.ex), ['structure', 'drive', 'power', 'avionics', 'wing']);
  for (const v of Object.values(a0.ex)) assert.ok(v > 0.9, 'all out at the start');
  for (const v of Object.values(a1.ex)) assert.equal(v, 0, 'all in by u 0.86');
  // the drive's row lights once its last step lands, before the wings' (the gearbox goes together first)
  const last = bird.steps.map((s) => s.sub).lastIndexOf('drive');
  const mid = assemblyExplode(bird, (0.86 * (last + 1)) / bird.steps.length + 0.005);
  assert.ok(mid.ex.drive < 0.02 && mid.ex.wing > 0.5, JSON.stringify(mid.ex));
  // a bird without steps: the subsystem windows
  const old = assemblyExplode({ buildExplode: () => null }, 0.3);
  assert.equal(old.build, null);
  assert.ok(old.ex.structure < old.ex.wing);
  // camera: on the gearbox (between the plates, about the crank's height) while it goes together, then the whole bird
  const P = model.planes;
  const [t0, , , d0] = assemblyOrbit(0), [t1, , , d1] = assemblyOrbit(0.85);
  assert.ok(t0.x > P.plateAft && t0.x < P.plateFwd && d0 < 1, `${t0.x} at ${d0} m`);
  assert.ok(Math.abs(t1.x + 0.15) < 1e-12 && t1.y === 0 && d1 > 2, `${t1.x} at ${d1} m`);
});

test('every step arrives along one body axis, from the side its parts can come from', () => {
  for (const s of plan) for (const m of s.members) {
    const d = s.dirOf(m.split('/')[0]), nz = d.filter((v) => v !== 0);
    assert.equal(nz.length, 1, `${s.name}: along one axis, ${d}`);
    assert.ok(Math.abs(nz[0]) >= 0.03 && Math.abs(nz[0]) <= 0.15, `${s.name}: ${nz[0]} m out`);
  }
  const d = (n, id) => plan.find((s) => s.name === n).dirOf(id ?? plan.find((s) => s.name === n).members[0].split('/')[0]);
  // the crank's web and pin are aft of the aft plate: it comes from aft; everything inside the box comes in from
  // forward before the forward plate closes it; stage 1 and the encoder onto the stubs forward of it; the couplers
  // onto the crank pin from its free (aft) end; the pins from aft; brackets and rockers from their own side
  assert.ok(d('crank')[0] < 0);
  for (const n of ['gear train', 'motor', 'rails', 'forward plate', 'stage 1 and encoder']) assert.ok(d(n)[0] > 0, n);
  for (const t of ['R', 'L']) {
    for (const n of [`coupler ${t}`, `shoulder pin ${t}`, `horn pin ${t}`]) assert.ok(d(n)[0] < 0, n);
    for (const n of [`shoulder brackets ${t}`, `rocker ${t}`]) assert.ok(Math.sign(d(n)[2]) === (t === 'R' ? 1 : -1), n);
  }
  assert.ok(d('side panels', 'gearbox side panel right')[2] > 0 && d('side panels', 'gearbox side panel left')[2] < 0);
  assert.ok(d('floor')[1] < 0);
  assert.ok(d('boom clamps', 'boom clamp aft')[0] < 0 && d('boom clamps', 'boom clamp forward')[0] > 0);
  assert.ok(d('trays', 'aft tray')[1] < 0 && d('trays', 'forward tray')[1] < 0, 'the trays up onto the plates\' pads');
});

test('stepOf: a part/solid in a step that names it, else the step that names its part; stepFractions eases each step', () => {
  const p = [{ members: ['rocker R/hub'] }, { members: ['rocker R'] }, { members: ['rocker R/horn pin'] }];
  assert.equal(stepOf(p, 'rocker R/horn pin'), 2);
  assert.equal(stepOf(p, 'rocker R/hub'), 0);
  assert.equal(stepOf(p, 'rocker R/arm'), 1);
  assert.equal(stepOf(p, 'crank'), -1);
  assert.deepEqual(stepFractions(4, 0), [1, 1, 1, 1]);
  assert.deepEqual(stepFractions(4, 1), [0, 0, 0, 0]);
  assert.deepEqual(stepFractions(4, 0.375), [0, 0.5, 1, 1]);
  assert.ok(Math.abs(stepFractions(4, 0.3125)[1] - (1 - 0.25 * 0.25 * 2.5)) < 1e-12, 'smoothstep in the step');
});

test('building: a part is drawn from the moment its step begins, moved along its line; posed parts too; all drawn when not building', () => {
  const N = bird.steps.length, at = (n) => bird.steps.findIndex((s) => s.name === n);
  const seen = (name) => bird.root.getObjectByName(name).visible;
  const k = at('gear train');
  // half way through the gear train's step: the motor (before it) in, the gears out along +x, the rails (after) not drawn
  bird.update({ phase: 0, amp, build: (k + 0.5) / N });
  bird.root.updateMatrixWorld(true);
  assert.ok(seen('motor_bell_can') && seen('shaft_1') && !seen('frame_rail_right') && !seen('plate_fwd') && !seen('shell'));
  const tie = bird.meshes.find((o) => /^frame_tie_/.test(o.name)).name;
  assert.ok(!seen('frame_screw_0') && !seen(tie), 'film-only fasteners and ties with their joints\' steps');
  const g = bird.root.getObjectByName('gear_crank'), f = 1 - 0.5 * 0.5 * 2; // smoothstep at the step's middle
  const home = new THREE.Vector3(), out = new THREE.Vector3();
  out.setFromMatrixPosition(g.matrixWorld);
  bird.update({ phase: 0, amp, build: 1 });
  bird.root.updateMatrixWorld(true);
  home.setFromMatrixPosition(g.matrixWorld);
  assert.ok(Math.abs(out.x - home.x - plan[k].dir[0] * f) < 1e-9 && Math.abs(out.y - home.y) < 1e-12, `posed gear out by ${out.x - home.x}`);
  // the wings, the tail and its pushrods only in their steps
  bird.update({ phase: 0, amp, build: (at('pod') + 0.2) / N });
  assert.ok(seen('pushrod_R') && seen('former_0') && seen('rocker_socket_R'), 'the pod going on, the rocker long in');
  assert.ok(bird.meshes.filter((o) => o.userData.subsystem === 'wing').every((o) => !o.visible), 'the wings not yet');
  bird.update({ phase: 0, amp, build: (at('pod') - 0.01) / N });
  assert.ok(!seen('pushrod_R') && !seen('former_0'));
  bird.update({ phase: 0, amp, build: 1 });
  assert.ok(bird.meshes.filter((o) => o.userData.subsystem === 'wing').every((o) => o.visible || /^(film_|le_rod|te_rod)/.test(o.name)));
  // not building: every part drawn, wherever an explode puts it
  bird.update({ phase: 0, amp, explode: { drive: 1, structure: 1, wing: 1 } });
  assert.ok(seen('frame_rail_right') && seen('shaft_1') && seen('frame_screw_0') && seen('pushrod_R'));
  bird.update({ phase: 0, amp });
});
