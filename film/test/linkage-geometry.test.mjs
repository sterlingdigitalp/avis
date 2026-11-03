// The machined linkage and gearbox as drawn, against the design's own numbers (params.linkage, the BOM's basis text,
// the catalogue's journals) and the kinematics: every coupler eye on its pin over a whole crank turn, every member at
// its size, and the focus rules of the studio sequences.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { nodeBird } from '../lib/bird-node.mjs';
import { linkageAt, journalDims } from '../../sims/avis-shared/model.js';
import { drawnBox, drawnRadii, poseOf, meshesOf } from './drawn.mjs';

const { bird, design, model } = nodeBird('larus-gl-2');
const L = design.params.linkage;
const mm = (v) => v / 1000;
const node = (n) => {
  const o = bird.root.getObjectByName(n);
  assert.ok(o, n);
  return o;
};
const centre = (n) => new THREE.Box3().setFromObject(node(n)).getCenter(new THREE.Vector3());
/** Local size and inner radius (about the x axis) of a mesh's own geometry. */
function dims(n) {
  const g = node(n).geometry;
  g.computeBoundingBox();
  const s = g.boundingBox.getSize(new THREE.Vector3());
  const p = g.attributes.position.array;
  let ri = Infinity;
  for (let i = 0; i < p.length; i += 3) ri = Math.min(ri, Math.hypot(p[i + 1], p[i + 2]));
  return { x: s.x, y: s.y, z: s.z, ri };
}
const near = (a, b, tol = 1e-6) => Math.abs(a - b) < tol;
/** World x length and radii about the centre c ([z, y]) of a ring drawn anywhere: v22's fixed parts are drawn in body coordinates. */
function ringAbout(n, c) {
  bird.root.updateMatrixWorld(true);
  const o = node(n), pos = o.geometry.attributes.position, v = new THREE.Vector3();
  let x0 = Infinity, x1 = -Infinity, ri = Infinity, ro = 0;
  for (let i = 0; i < pos.count; i++) {
    v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld);
    const r = Math.hypot(v.z - c[0], v.y - c[1]);
    x0 = Math.min(x0, v.x); x1 = Math.max(x1, v.x); ri = Math.min(ri, r); ro = Math.max(ro, r);
  }
  return { x: x1 - x0, x0, x1, ri, ro };
}
/** The BOM's length (mm, as printed) and its printed precision. */
const printed = (text) => ({ v: +text / 1000, tol: (text.includes('.') ? 0.5 * 10 ** -text.split('.')[1].length : 0.5) / 1000 + 1e-9 });
const jd = (role) => journalDims(model.parts[design.bom.parts.find((p) => role.test(p.role)).id].specs);
const sh = jd(/shoulder pivots/), eye = jd(/coupler eyes/);
const span = design.params.joints.shoulderSpan_mm / 1000;
const item = (re) => design.bom.structure.find((s) => re.test(s.name));
// v24 on, the crank, the couplers' eyes and bushings and the keys are drawn from the physics' solids: what is drawn of
// one solid (drawn.mjs) at the pose the bird is in
const amp = model.law.ampNominal;
const dcentre = (tag, q) => drawnBox(bird, tag, q).getCenter(new THREE.Vector3());
const dsize = (tag, q) => drawnBox(bird, tag, q).getSize(new THREE.Vector3());

test('over a whole crank turn each coupler\'s eyes sit on the crank pin and on its rocker\'s horn pin', () => {
  for (let i = 0; i < 24; i++) {
    const phase = (i / 24) * 2 * Math.PI, q = poseOf(bird, phase, amp);
    bird.update({ phase, amp });
    bird.root.updateMatrixWorld(true);
    const pin = dcentre('crank web/pin', q);
    for (const tag of ['R', 'L']) {
      const bot = dcentre(`coupler ${tag}/crank eye`, q), top = dcentre(`coupler ${tag}/horn eye`, q), horn = centre(`horn_pin_${tag}`);
      assert.ok(near(bot.y, pin.y) && near(bot.z, pin.z), `${tag} bottom eye off the crank pin at ${i}/24`);
      assert.ok(near(top.y, horn.y, 1e-5) && near(top.z, horn.z, 1e-5), `${tag} top eye off the horn pin at ${i}/24: ${top.y - horn.y}, ${top.z - horn.z}`);
      assert.ok(near(top.distanceTo(bot) ** 2 - (top.x - bot.x) ** 2, model.law.b ** 2, 1e-8), `${tag} coupler length`);
    }
  }
});

test('rockers, pins and housings at the design\'s and the BOM\'s sizes', () => {
  const r = L.rocker;
  const c = linkageAt(model.law, model.law.ampNominal).c;
  bird.update({ phase: 0, amp: model.law.ampNominal });
  const O4 = model.fourBar.O4;
  for (const [tag, side] of [['R', 'right'], ['L', 'left']]) {
    const at = [tag === 'R' ? O4[0] : -O4[0], O4[1]];
    // the hub: the BOM's length (as printed) over its pieces, at the design's diameters; turned down to the relief
    // diameter where the coupler eyes pass, a collar on the shoulder pin as the BOM lists it
    const hubBOM = item(new RegExp(`^rocker ${side}$`)).basis.match(/hub ([\d.]+)\/([\d.]+) x ([\d.]+) mm/);
    const pieces = bird.meshes.filter((o) => new RegExp(`^rocker_${tag}(_relief|_forward)?$`).test(o.name)).map((o) => ringAbout(o.name, at));
    const hubLen = pieces.reduce((a, p) => a + p.x, 0), want = printed(hubBOM[3]);
    assert.ok(Math.abs(hubLen - want.v) <= want.tol, `hub ${hubLen} vs the BOM's ${hubBOM[3]} mm`);
    for (const p of pieces) assert.ok(near(p.ri, sh.d / 2, 1e-5) && (near(p.ro, mm(r.hub_od_mm) / 2, 1e-5) || near(p.ro, mm(r.hub_relief_od_mm) / 2, 1e-5)), 'hub diameters');
    if (r.hub_relief_od_mm) {
      const rel = ringAbout(`rocker_${tag}_relief`, at);
      // the relief spans both coupler eyes' planes and the running clearance either side
      const run = design.params.layout.running_mm / 1000;
      assert.ok(rel.x0 <= model.planes.couplerL - eye.B / 2 - run + 1e-9 && rel.x1 >= model.planes.couplerR + eye.B / 2 + run - 1e-9, 'relief under the eyes');
    }
    const pinBOM = item(new RegExp(`^shoulder pin ${side}$`)).basis.match(/(\d[\d.]*) mm x ([\d.]+) mm.*?([\d.]+) mm collar x ([\d.]+) mm/);
    const col = ringAbout(`rocker_${tag}_pincollar`, at), cw = printed(pinBOM[4]);
    assert.ok(near(2 * col.ro, +pinBOM[3] / 1000, 1e-5) && Math.abs(col.x - cw.v) <= cw.tol, `collar ${col.x}`);
    const boss = dims(`rocker_boss_${tag}`), arm = dims(`rocker_arm_${tag}`), sock = dims(`rocker_socket_${tag}`);
    assert.ok(near(boss.y, mm(r.boss_od_mm), 2e-4) && near(boss.x, mm(r.boss_len_mm)) && near(boss.ri, eye.d / 2, 1e-5), 'boss');
    assert.ok(near(arm.x, mm(r.arm_t_mm)) && near(arm.y, c, 1e-6) && near(arm.z, mm(r.arm_w_mm)), `arm ${arm.y} vs ${c}`);
    assert.ok(near(sock.x, mm(r.spar_od_mm) + 2 * mm(r.socket_wall_mm), 2e-4) && near(sock.z, mm(r.socket_len_mm)), 'socket');
    const hp = dims(`horn_pin_${tag}`), sp = dims(`hinge_pin_${tag}`);
    const hornBOM = item(new RegExp(`^horn pin ${side}$`)).basis.match(/([\d.]+) mm x ([\d.]+) mm/), hl = printed(hornBOM[2]);
    assert.ok(near(hp.y, eye.d, 2e-4) && Math.abs(hp.x - hl.v) <= hl.tol, `horn pin ${hp.x} vs the BOM's ${hornBOM[2]} mm`);
    const sl = printed(pinBOM[2]);
    assert.ok(near(sp.y, sh.d, 2e-4) && near(sp.x, span + sh.B) && Math.abs(sp.x - sl.v) <= sl.tol, `shoulder pin ${sp.x} vs the BOM's ${pinBOM[2]} mm`);
    for (const end of ['a', 'f']) {
      // the housing round the bushing, counterbored for its flange on the hub's side
      const h = ringAbout(`shoulder_housing_${tag}_${end}`, at);
      assert.ok(near(2 * h.ro, mm(L.shoulderHousing.od_mm), 1e-5) && near(h.x, sh.B) && near(h.ri, sh.D / 2, 1e-5), 'housing');
      assert.ok(near(Math.abs((h.x0 + h.x1) / 2 - design.wing.hinge[0]), span / 2), 'housing at the bushing');
      const b = ringAbout(`hinge_bushing_${tag}_${end}`, at);
      assert.ok(near(b.x0, h.x0) && near(b.x1, h.x1) && near(b.ri, sh.d / 2, 1e-5) && near(b.ro, sh.flangeD / 2, 1e-5), 'bushing in its housing');
    }
    // the eye (v24: counterbored for its bushing's flange) round its bushing, as long as the bushing
    const q = poseOf(bird, 0, amp), ce = `coupler ${tag}/crank eye`, eb = drawnBox(bird, ce, q).union(drawnBox(bird, `${ce} counterbore`, q));
    const ec = bird.phys.solid(`coupler ${tag}`, 'crank eye').at(q).c, er = drawnRadii(bird, ce, q, ec);
    assert.ok(near(dsize(ce, q).y, mm(L.coupler.eye_od_mm), 2e-4) && near(eb.max.x - eb.min.x, eye.B) && near(er.ri, eye.D / 2, 1e-5), 'coupler eye');
    const rod = dims(`coupler_rod_${tag}`);
    assert.ok(near(rod.x, mm(L.coupler.body_d_mm), 2e-4), 'coupler body');
  }
});

test('the crank, shafts, keys, plates and bosses at the design\'s sizes; the crank pin and shafts as long as the BOM says', () => {
  const C = L.crank;
  // v24: the crank in two parts, the shaft (with its stub and wheel seat) and the web and pin keyed on the stub, each
  // its own BOM line; at phase 0, where the crank's throw points at the pin
  bird.update({ phase: 0, amp });
  const q = poseOf(bird, 0, amp), O2 = model.train.shafts[0];
  const webBOM = item(/^crank web and pin$/).basis, shaftBOM = item(/^crank shaft$/).basis;
  const pinL = +webBOM.match(/([\d.]+) mm pin x ([\d.]+) mm/)[2] / 1000;
  const pin = dsize('crank web/pin', q), web = drawnBox(bird, 'crank web/web', q), shaft = dsize('crank/shaft', q);
  assert.ok(near(pin.x, pinL) && near(pin.y, eye.d, 2e-4), `crank pin ${pin.x} vs the BOM's ${pinL}`);
  // the web: its thickness along x, its width across the throw, its length the throw plus its width
  const [, wb, throwMm, wt] = webBOM.match(/web ([\d.]+) mm wide round the ([\d.]+) mm throw x ([\d.]+) mm/).map(Number);
  const ws = web.getSize(new THREE.Vector3());
  assert.ok(near(ws.x, mm(wt)) && near(mm(wt), mm(C.web_t_mm)) && near(mm(throwMm), model.law.a, 1e-9), 'web thickness and throw');
  const p = bird.phys.solid('crank web', 'pin').at(q).c, u = [(p[0] - O2[0]) / model.law.a, (p[1] - O2[1]) / model.law.a];
  const r = drawnRadii(bird, 'crank web/web', q, O2);
  assert.ok(near(r.ri, mm(C.shaft_od_mm) / 2, 1e-5), 'bored on the stub');
  const ext = (dir) => { let m = -Infinity; const v = new THREE.Vector3(); for (const o of meshesOf(bird, 'crank web/web')) { const pos = o.geometry.attributes.position; for (let i = 0; i < pos.count; i++) { v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld); m = Math.max(m, (v.z - O2[0]) * dir[0] + (v.y - O2[1]) * dir[1]); } } return m; };
  assert.ok(near(ext(u) + ext([-u[0], -u[1]]), model.law.a + mm(wb), 2e-4) && near(ext([-u[1], u[0]]) + ext([u[1], -u[0]]), mm(wb), 2e-4), 'web as long as the throw and its width, as wide as its width');
  const sr = drawnRadii(bird, 'crank/shaft', q, O2);
  assert.ok(near(shaft.y, mm(C.shaft_od_mm), 2e-4) && near(sr.ri, mm(C.shaft_id_mm) / 2, 1e-4), 'crank shaft');
  // the crank shaft as long as the BOM says (to the end of the encoder's stub forward of stage 1)
  const shaftLen = +shaftBOM.match(/shaft x ([\d.]+) mm/)[1] / 1000;
  assert.ok(near(shaft.x, shaftLen), `crank shaft ${shaft.x} vs the BOM's ${shaftLen}`);
  L.gearShafts.forEach((g, k) => {
    const s = dims(`shaft_${k + 1}`);
    const len = +item(new RegExp(`^gear shaft ${k + 1}$`)).basis.match(/x ([\d.]+) mm/)[1] / 1000;
    assert.ok(near(s.y, mm(g.d_mm), 2e-4) && near(s.x, len), `gear shaft ${k + 1}: ${s.x} vs ${len}`);
    if (g.seat_od_mm) assert.ok(near(dims(`shaft_${k + 1}_wheel_seat`).y, mm(g.seat_od_mm), 2e-4), 'seat');
  });
  // the crank wheel's key, the BOM's b x h x L
  const wheelKey = bird.phys.parts.find((x) => /^key, crank wheel /.test(x.id));
  const [kb, kh, kl] = item(new RegExp(`^${wheelKey.id}$`)).basis.match(/([\d.]+) x ([\d.]+) x ([\d.]+) mm/).slice(1).map(Number);
  const ks = dsize(`${wheelKey.id}/key`, q);
  assert.ok(near(ks.x, mm(kl)) && [ks.y, ks.z].sort().every((v, i) => near(v, [mm(kb), mm(kh)].sort()[i])) && near(mm(kb), mm(C.key.b_mm)) && near(mm(kh), mm(C.key.h_mm)), 'crank wheel key');
  const P = L.plates;
  assert.ok(near(dims('plate_aft').x, mm(P.t_mm)) && near(dims('plate_fwd').x, mm(P.t_fwd_mm)), 'plates');
  const crankJ = jd(/^bushing: crank shaft/);
  // a boss as long as the journal's body in its seat, less the plate (v22, plates.flange 'face': the flange on the
  // plate's outer face, so the body is the bushing's length less its flange)
  const body = P.flange === 'face' && crankJ.flangeB > 0 ? crankJ.B - crankJ.flangeB : crankJ.B;
  const b = ringAbout('plate_boss_0_aft', model.train.shafts[0]);
  assert.ok(near(2 * b.ro, crankJ.D + 2 * mm(P.boss_wall_mm), 1e-5) && near(b.x, body - mm(P.t_mm)) && near(b.ri, crankJ.D / 2, 1e-5), `crank boss ${b.x}`);
  assert.equal(bird.root.children.filter((o) => /^plate_boss_/.test(o.name)).length, P.bossed.length);
});

test('a focus shows its subsystem and the parts named with it, and ghosts the rest', () => {
  const servoIds = new Set(design.bom.parts.filter((p) => /servo$/.test(p.role)).map((p) => p.id));
  bird.update({ phase: 0, amp: model.law.ampNominal, focus: 'power', also: servoIds });
  const ghost = node('part_battery').children[0].material.constructor === THREE.MeshBasicMaterial;
  assert.equal(ghost, false, 'the battery is drawn as itself');
  const mat = (n) => node(n).material;
  const web = meshesOf(bird, 'crank web/web')[0];
  assert.ok(mat('servo_tail_L_case') !== web.material && web.material.transparent && web.material.opacity < 0.1, 'the crank is ghosted, a tail servo is not');
  bird.update({ phase: 0, amp: model.law.ampNominal, focus: 'twist' });
  assert.ok(!mat('servo_tail_L_case').opacity || mat('servo_tail_L_case').opacity < 0.1, 'without also, a tail servo is ghosted');
  bird.update({ phase: 0, amp: model.law.ampNominal });
  assert.ok(!(web.material.transparent && web.material.opacity < 0.1), 'no focus: all drawn');
});

test('keys in their keyways, bosses on the plates\' inner faces, the motor bearing in the forward plate', () => {
  bird.update({ phase: 0, amp: model.law.ampNominal });
  bird.root.updateMatrixWorld(true);
  const k = L.crank.key, bore = model.train.stages.at(-1).wheel.bore, O2 = model.train.shafts[0];
  const wheelKey = bird.phys.parts.find((x) => /^key, crank wheel /.test(x.id));
  const kc = dcentre(`${wheelKey.id}/key`, poseOf(bird, 0, amp));
  assert.ok(near(Math.hypot(kc.z - O2[0], kc.y - O2[1]), bore / 2 - mm(k.shaft_depth_mm) + mm(k.h_mm) / 2), 'crank wheel key half in the shaft');
  const P = model.planes, PL = L.plates;
  const box = (n) => new THREE.Box3().setFromObject(node(n));
  for (const key of PL.bossed) {
    const [joint, support] = key.split('/');
    const n = `plate_boss_${joint === 'crank shaft' ? 0 : joint.match(/\d+/)[0]}_${support}`;
    const b = box(n);
    if (support === 'aft') assert.ok(near(b.min.x, P.plateAft + mm(PL.t_mm) / 2), `${n} on the aft plate's inner face`);
    else assert.ok(near(b.max.x, P.plateFwd - mm(PL.t_fwd_mm) / 2), `${n} on the forward plate's inner face`);
  }
  const mo = design.bom.parts.find((p) => /motor outboard/.test(p.role));
  const d = journalDims(model.parts[mo.id].specs), m = box(`motor_outboard_${mo.role.split(':')[0]}`);
  const c = model.train.shafts.at(-1);
  // in the forward plate: flush with its outer face (the motor forward of the box), or (v22, stage 1 overhung, the
  // motor inside the box) seated from its inner face outward, its pinion forward of the plate
  if (design.params.layout?.stage1 === 'overhung') assert.ok(near(m.min.x, P.plateFwd - mm(PL.t_fwd_mm) / 2) && near(m.max.x - m.min.x, d.B), 'from the plate\'s inner face outward');
  else assert.ok(near(m.max.x, P.plateFwd + mm(PL.t_fwd_mm) / 2) && near(m.max.x - m.min.x, d.B), 'flush with the plate\'s outer face');
  assert.ok(near((m.min.y + m.max.y) / 2, c[1]) && near((m.min.z + m.max.z) / 2, c[0]), 'on the motor shaft');
});

test('a symmetric pose (as a flight log flies it): the left wing at the right stroke, its coupler still on the crank pin', () => {
  // The flight model strokes both wings alike; one shared crank pin cannot (6.6 deg apart at v21's amplitude). The
  // left coupler stays on the pin and aims at the mirrored rocker pin, so its far eye misses that pin by a few mm.
  // the coupler's eyes are drawn in its node (drive-solids.js drawPosed), at its origin and at the coupler's length
  // along its y: first, that the drawn eyes are there
  const eyeAt = (tag, y) => node(`coupler_${tag}`).localToWorld(new THREE.Vector3(0, y, 0));
  bird.update({ phase: 1.1, amp });
  const q = poseOf(bird, 1.1, amp);
  for (const [e, y] of [['crank', 0], ['horn', model.law.b]]) {
    const d = dcentre(`coupler L/${e} eye`, q), a = eyeAt('L', y);
    assert.ok(near(d.y, a.y) && near(d.z, a.z), `the ${e} eye at its node's ${y}`);
  }
  bird.update({ phase: 1.1, amp, symmetric: true });
  bird.root.updateMatrixWorld(true);
  const pin = dcentre('crank web/pin', q), bot = eyeAt('L', 0);
  assert.ok(near(bot.y, pin.y) && near(bot.z, pin.z), 'on the crank pin');
  const r = centre('horn_pin_R'), l = centre('horn_pin_L');
  assert.ok(near(l.y, r.y) && near(l.z, -r.z), 'the left rocker mirrors the right one');
  const top = eyeAt('L', model.law.b);
  assert.ok(Math.hypot(top.y - l.y, top.z - l.z) < 0.003, `left eye ${Math.hypot(top.y - l.y, top.z - l.z) * 1000} mm off its horn pin`);
  bird.update({ phase: 1.1, amp: model.law.ampNominal });
});

test('the twist focus draws the wings\' ribs and film and ghosts their spars', () => {
  bird.update({ phase: 0, amp: model.law.ampNominal, focus: 'twist' });
  let rib = null, spar = null;
  bird.root.traverse((o) => {
    if (!o.isMesh || o.userData.subsystem !== 'wing') return;
    if (!rib && /rib/.test(o.name)) rib = o;
    if (!spar && /spar/.test(o.name) && !/rib|film/.test(o.name)) spar = o;
  });
  assert.ok(rib && spar, 'a rib and a spar');
  assert.ok(!(rib.material.transparent && rib.material.opacity < 0.1), 'rib drawn');
  assert.ok(spar.material.transparent && spar.material.opacity < 0.1, 'spar ghosted');
  bird.update({ phase: 0, amp: model.law.ampNominal });
});
