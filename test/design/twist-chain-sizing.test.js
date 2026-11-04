// Round 19: the hand wing's twist chain sized for the rough-air row (src/design/twist-drive.js). The old chain (the
// R-ML25's 16 mm hole, a 12.8 mm lever, two KBRM-02) is refused at the rough row's twist torque, an arm the maker does
// not publish (a hole it does not have, an M3 rod end on its M2 tapped hole) is refused, and a supply above a servo's top
// published row is refused (SPEC "Servo voltage covers current too").
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { TWIST_SECTION, twistContext, twistChainFigures, twistChains, twistMembers, armFitFindings, twistParams } from '../../src/design/twist-drive.js';
import { checkMembers } from '../../src/physics/members.js';
import { materialOf } from '../../src/design/linkage.js';
import { adhesiveOf } from '../../src/design/airframe.js';
import { chainRow, ratingsRecord } from '../../src/design/chains.js';

const catalog = JSON.parse(readFileSync(new URL('../../data/parts/catalog.json', import.meta.url), 'utf8'));
const base = JSON.parse(readFileSync(new URL('../fixtures/larus-gl-2-v24.json', import.meta.url), 'utf8'));
const withDrive = (over = {}) => {
  const d = structuredClone(base);
  d.params = { ...d.params, tubeSpecSheet: true, parts: { ...d.params.parts, twistServo: 'servo-hitec-md89mw' }, twistDrive: { section: TWIST_SECTION, ...over } };
  return d;
};
const OLD = { servo: { hole_mm: 16 }, lever: { r_mm: 12.8 } };
const KBRM02_N = 97.9; // igus igubal catalogue: KBRM-02, maximum static tensile load, long term
// the rough-air row's twist torque at the wing (x1, 14 m/s, the right wing's peak; round 18 $S18/sizing/x1fly-14.log
// on the old chain, round 19 $S19/twist/logs/fly-p1-185-ref.log on the new one at its own trim frequency)
const Q_OLD = 0.4625, Q_NEW = 0.5097;

/**
 * The link's force by statics on the drawn four-bar, solved here independently of twist-drive.js's kinematics: for each
 * servo angle a over +-30 deg the lever angle b where the link keeps its drawn length (bisection), the transmission
 * angle's sine between the lever and the link; F = Tr / (r_l sin gamma) at the worst one, Tr = kq Q.
 */
function linkForce(G, kq, Q) {
  const ell = Math.hypot(G.Hp[0] - G.Lp[0], G.Hp[1] - G.Lp[1]);
  const H = (a) => [G.S[0] + G.rs * Math.cos(G.alpha + a), G.S[1] + G.rs * Math.sin(G.alpha + a)];
  const Lv = (b) => [G.rl * Math.cos(G.alpha + b), G.rl * Math.sin(G.alpha + b)];
  let sinMin = 1;
  for (let i = 0; i <= 60; i++) {
    const a = (-30 + i) * (Math.PI / 180), h = H(a), f = (b) => Math.hypot(h[0] - Lv(b)[0], h[1] - Lv(b)[1]) - ell;
    let lo = a * (G.rs / G.rl) - 0.4, hi = a * (G.rs / G.rl) + 0.4;
    if (f(lo) * f(hi) > 0) throw new Error('no bracket');
    for (let k = 0; k < 80; k++) { const m = (lo + hi) / 2; if (f(lo) * f(m) <= 0) hi = m; else lo = m; }
    const b = (lo + hi) / 2, l = Lv(b), dv = [h[0] - l[0], h[1] - l[1]], n = Math.hypot(...dv), u = [Math.cos(G.alpha + b), Math.sin(G.alpha + b)];
    sinMin = Math.min(sinMin, Math.abs(u[0] * dv[1] - u[1] * dv[0]) / n);
  }
  return (kq * Q) / (G.rl * sinMin);
}

const membersOf = (d) => {
  const members = [];
  twistMembers({ d, catalog, members, mat: (id) => materialOf(catalog, id), glueAl: adhesiveOf(catalog, 'adhesive-araldite-2011-aluminium'), glueCf: adhesiveOf(catalog, 'adhesive-araldite-2011') });
  return members;
};
const fosAt = (members, name, Q) => {
  const m = members.find((q) => q.name === name);
  const peaks = Object.fromEntries(m.sections.map((s) => [`${m.name}/${s.name}`, s.radial.reduce((a, q) => a + q.k * (q.ch === 'twist_R' ? Q : 0), 0)]));
  return checkMembers([m], peaks).members[0].fos;
};

test('the old chain\'s KBRM-02 at the rough row\'s twist torque is under the 2.5 the envelope holds members to; the round-19 chain is over it', () => {
  for (const [over, Q, refused] of [[OLD, Q_OLD, true], [{}, Q_NEW, false]]) {
    const d = withDrive(over), { G } = twistContext(d, catalog), kq = twistChainFigures(d, catalog).kq;
    const F = linkForce(G, kq, Q), fosRef = KBRM02_N / F;
    const fos = fosAt(membersOf(d), 'twist link R rod ends', Q);
    assert.ok(Math.abs(fos - fosRef) / fosRef < 2e-3, `member ${fos} vs statics ${fosRef}`);
    // the flight envelope's members ratio: (1.5 / fos) over its 0.6 member allowance
    assert.equal((1.5 / fos) / 0.6 > 1, refused, `${JSON.stringify(over)}: FoS ${fos.toFixed(3)}`);
  }
  // the old chain: FoS 2.07 (round 18's finding), the link 47.2 N
  const d = withDrive(OLD), { G } = twistContext(d, catalog);
  assert.ok(Math.abs(linkForce(G, twistChainFigures(d, catalog).kq, Q_OLD) - 47.2) < 0.2);
});

test('the case\'s torque reaction is the servo\'s torque, the rod\'s times r_s / r_l at the centre (statics), and the frame pins carry it', () => {
  for (const over of [OLD, {}]) {
    const d = withDrive(over), { G } = twistContext(d, catalog), kq = twistChainFigures(d, catalog).kq;
    const pins = membersOf(d).find((m) => m.name === 'twist frame pins R'), P = twistParams(d.params);
    const span = G.caseDims[0] + 2 * P.hanger.lip_mm / 1000;
    // the pin's moment per N m of twist torque: the case's torque over its length, simply supported over the lips
    const kM = pins.sections[0].radial.find((q) => q.ch === 'twist_R').k, Ts = (kM * 4 / span) * G.caseDims[0];
    assert.ok(Ts >= kq * (G.rs / G.rl) * (1 - 1e-3), `${JSON.stringify(over)}: ${Ts} vs ${kq * G.rs / G.rl}`);
  }
});

test('an arm the maker does not publish is refused: a hole it does not have, or an M3 screw and rod end on its M2 tapped hole', () => {
  assert.deepEqual(armFitFindings(twistParams(withDrive().params), catalog), []);
  assert.deepEqual(armFitFindings(twistParams(withDrive(OLD).params), catalog), []);
  const odd = armFitFindings(twistParams(withDrive({ servo: { hole_mm: 17.5 } }).params), catalog);
  assert.ok(odd.some((f) => /17\.5 mm .* publishes holes at 16, 19 mm/.test(f)), odd.join('; '));
  const m3 = armFitFindings(twistParams(withDrive({ screw: 'screw-iso4762-m3x8-10.9' }).params), catalog);
  assert.ok(m3.some((f) => /M3 .*M2\.0x0\.4/.test(f)) && m3.some((f) => /^rod end: .*2 mm bore on an M3 screw/.test(f)), m3.join('; '));
  // and the chain record refuses it (a refusing finding, not a flown one)
  const d = withDrive({ servo: { hole_mm: 17.5 } });
  const [r] = twistChains(null, d, catalog, { twistTorque_Nm: 0.1, travel_rad: 0.1, twistRate_rad_s: 1, basis: 'test' });
  assert.ok(r.findings.some((f) => typeof f === 'string' && /^arm:/.test(f)));
  assert.equal(chainRow({ parts: [], joins: [] }, r).passed, false);
});

test('a supply whose highest output is above a servo\'s top published row is refused, current included (SPEC "Servo voltage covers current too")', () => {
  const d = (lo, hi) => ({ servos: { twist: { id: 'servo-hitec-md89mw' }, tail: { id: 'servo-hitec-md89mw' } }, electrical: { servo_supply_V: lo, servo_supply: { performance_V: lo, current_V: hi } } });
  // the Pololu D36V50F6, 6.0 V +-4%: inside the MD89MW's 4.8-7.4 V rows
  assert.equal(ratingsRecord(d(5.76, 6.24), catalog).passed, true);
  // the D36V50F7, 7.5 V +-4%: its lowest 7.2 V is inside the rows and its highest 7.8 V inside the 3.5-8.4 V range, but
  // above the 7.4 V top row, where no stall current is published
  const r = ratingsRecord(d(7.2, 7.8), catalog);
  assert.equal(r.passed, false);
  assert.ok(r.rows.some((x) => !x.ok && /highest supply 7\.8 V/.test(x.what) && /above the published top row 7\.4 V/.test(x.why)), JSON.stringify(r.rows.filter((x) => !x.ok)));
});
