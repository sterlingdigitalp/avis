// The airframe the design's load path names (src/design/airframe.js): shoulder rails and brackets, boom clamp
// blocks, trays, doublers, gearbox panels, screws, nuts and cable ties, each drawn from the design file's own
// numbers (design.structure.airframe.parts, design.params.linkage.airframe, and each BOM line's own basis text).
// Called by bird.js buildBird with its build context. A design with no structure.airframe (an older design) draws
// nothing: this is the only generic contract this module keeps.
import * as THREE from 'three';
import { lockFlangeCircles } from '../../src/design/airframe.js';

const TWO_PI = 2 * Math.PI;
export const toM = (mm) => mm / 1000;
export const offCount = (item) => +(item?.name.match(/,\s*(\d+)\s*off/)?.[1] ?? 0);

// ---- basis-text parsers: one per BOM line shape, each returning the numbers (mm) it names, or null -------------
// Exported so the test can hit their branches directly, with crafted basis text, instead of only through the one
// real design that happens to have every field this module can read.
export const parseTube = (basis) => { const m = basis.match(/([\d.]+)\/([\d.]+)\s*mm tube x\s*([\d.]+)\s*mm/); return m && { od: +m[1], id: +m[2], len: +m[3] }; };
export const parseRing = (basis) => { const m = basis.match(/ring\s*([\d.]+)\/([\d.]+)\s*x\s*([\d.]+)\s*mm/); return m && { od: +m[1], id: +m[2], t: +m[3] }; };
export const parseBlocks = (basis) => { const m = basis.match(/([\d.]+)\s*x\s*([\d.]+)\s*mm blocks x\s*([\d.]+)\s*mm/); return m && { a: +m[1], b: +m[2], len: +m[3] }; };
export const parsePlate = (basis) => { const m = basis.match(/([\d.]+)\s*mm x\s*([\d.]+)\s*mm/); return m && { w: +m[1], t: +m[2] }; };
export function parseBracket(basis) {
  const s = basis.match(/two sleeves\s*([\d.]+)\/([\d.]+)\s*x\s*([\d.]+)\s*mm/);
  const a = basis.match(/two arms\s*([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/);
  return s && a ? { sOd: +s[1], sId: +s[2], sLen: +s[3], aB: +a[1], aH: +a[2], aLen: +a[3] } : null;
}
export function parseChannel(basis) {
  const m = basis.match(/channel\s*([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm sheet.*?x\s*([\d.]+)\s*mm/);
  return m && { w: +m[1], flangeH: +m[2], t: +m[3], len: +m[4] };
}
export function parsePanel(basis, name) {
  const m = basis.match(new RegExp(`${name}\\s+([\\d.]+)\\s*x\\s*([\\d.]+)\\s*mm`));
  const g = basis.match(/each\s*([\d.]+)\s*mm plus two\s*([\d.]+)\s*mm flanges/);
  return m && g ? { a: +m[1], t: +m[2], gap: +g[1], flange: +g[2] } : null;
}
export function screwGeo(ctx, item) {
  const h = item.basis.match(/head\s*([\d.]+)\s*mm x\s*([\d.]+)\s*mm/);
  const s = item.basis.match(/([\d.]+)\s*mm x\s*([\d.]+)\s*mm shank/);
  return h && s ? ctx.geo.cylX(toM(+h[1]) / 2, toM(+h[2] + +s[2])) : null;
}
export function nutGeo(ctx, item) {
  const m = item.basis.match(/([\d.]+)\s*mm across flats x\s*([\d.]+)\s*mm high/);
  const b = item.basis.match(/([\d.]+)\s*mm minor-diameter bore/);
  return m && b ? ring(ctx, +m[1], +b[1], +m[2]) : null;
}
export function tieGeo(ctx, item) {
  const d = ctx.spec(item.material_id).dimensions?.match(/([\d.]+)\s*x\s*([\d.]+)\s*x\s*([\d.]+)\s*mm/);
  return d ? new THREE.BoxGeometry(toM(+d[1]), toM(+d[2]), toM(+d[3])) : null;
}

// ---- geometry helpers --------------------------------------------------------------------------------------------
export const ring = (ctx, od, id, w) => ctx.geo.ringX(toM(od) / 2, toM(id) / 2, toM(w));
export const box = (sx, sy, sz) => new THREE.BoxGeometry(toM(sx), toM(sy), toM(sz));
export function splitEven(total, n) {
  const t = Math.max(0, total), base = Math.floor(t / n);
  return Array.from({ length: n }, (_, i) => (i === n - 1 ? t - base * (n - 1) : base));
}
export function scatter(center, n, r, arr) {
  if (!center) return;
  for (let k = 0; k < n; k++) {
    const a = (TWO_PI * k) / Math.max(n, 1);
    arr.push([center[0], center[1] + r * Math.cos(a), center[2] + r * Math.sin(a)]);
  }
}

/** Every joint a fastener or tie can sit at, by name, from whatever of the frame has already been drawn. */
function fastenerJoints(fx) {
  const { ctx, A, railX, clampX, trayX, xA } = fx;
  const flat = {};
  // a rail's lock screws at its lock flange (v22: on the forward plate's aft face), else at the aft plate
  for (const side of ['right', 'left']) if (railX[side] && A.rails?.[side]) flat[`rail ${side}`] = [fx.lockX ?? xA, A.rails[side][1], A.rails[side][0]];
  if (clampX) { flat['boom aft'] = [clampX.aft, A.boom_y_m, 0]; flat['boom fwd'] = [clampX.fwd, A.boom_y_m, 0]; }
  if (trayX.forward) flat['tray fwd'] = [trayX.forward.face, trayX.forward.y ?? A.trays.forward.y_m, 0];
  if (trayX.aft) flat['tray aft'] = [trayX.aft.face, trayX.aft.y ?? A.trays.aft.y_m, 0];
  const mp = ctx.model.layout.parts.find((p) => p.name === 'motor');
  if (mp && !fx.v23) flat.motor = fx.motorAt ?? [mp.box.cx, mp.box.cy, mp.box.cz];
  // v23: the motor hangs by its back face on the bracket (its screws into the motor's own threads), the riser's foot
  // on the forward tray and its tab under the nose tray
  if (fx.v23) Object.assign(flat, fx.v23);
  return flat;
}

// ---- shoulder rails, their aft seat boss and their bracket -------------------------------------------------------
function frameRail(fx, side) {
  const { ctx, A, xMid } = fx;
  const pos = A.rails?.[side];
  const item = ctx.structItem(new RegExp(`^shoulder rail ${side}$`));
  const d = pos && item && parseTube(item.basis);
  if (!d) return;
  const m = ctx.mesh(`frame_rail_${side}`, ring(ctx, d.od, d.id, d.len), 'alu_machined', ctx.root, 'structure', item.material_id);
  m.position.set(xMid, pos[1], pos[0]);
  ctx.register(item.name, item.material_id, item.name, 'structure', m);
  fx.railX[side] = { x0: xMid - toM(d.len) / 2, x1: xMid + toM(d.len) / 2 };
  frameRailBoss(fx, side, pos);
  frameBracket(fx, side, pos);
}

function frameRailBoss(fx, side, pos) {
  const { ctx, xA } = fx;
  const item = ctx.structItem(new RegExp(`^shoulder rail ${side} aft seat boss$`));
  const d = item && parseRing(item.basis);
  if (!d) return;
  const m = ctx.mesh(`frame_rail_boss_${side}`, ring(ctx, d.od, d.id, d.t), 'alu_machined', ctx.root, 'structure', item.material_id);
  m.userData.joint = 'aft plate';
  m.position.set(xA, pos[1], pos[0]);
  ctx.register(item.name, item.material_id, item.name, 'structure', m);
}

/** Two sleeves at the rail's own ends, and two arms pointing from the rail toward the shoulder housing. */
function frameBracket(fx, side, railPos) {
  const { ctx, assumptions, railX } = fx;
  const item = ctx.structItem(new RegExp(`^shoulder bracket ${side}$`));
  const d = item && parseBracket(item.basis);
  if (!d) return;
  const O4 = ctx.model.fourBar.O4;
  const housing = side === 'right' ? O4 : [-O4[0], O4[1]];
  const n = Math.hypot(housing[1] - railPos[1], housing[0] - railPos[0]) || 1;
  const dir = new THREE.Vector3(0, (housing[1] - railPos[1]) / n, (housing[0] - railPos[0]) / n);
  const objs = [];
  for (const [tag, x] of [['aft', railX[side].x0], ['fwd', railX[side].x1]]) {
    const sl = ctx.mesh(`frame_bracket_sleeve_${side}_${tag}`, ring(ctx, d.sOd, d.sId, d.sLen), 'alu_machined', ctx.root, 'structure', item.material_id);
    sl.position.set(x, railPos[1], railPos[0]);
    const arm = ctx.mesh(`frame_bracket_arm_${side}_${tag}`, box(d.aB, d.aLen, d.aH), 'alu_machined', ctx.root, 'structure', item.material_id);
    arm.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    arm.position.set(x, railPos[1] + dir.y * toM(d.aLen) / 2, railPos[0] + dir.z * toM(d.aLen) / 2);
    objs.push(sl, arm);
  }
  ctx.register(item.name, item.material_id, item.name, 'structure', objs);
  assumptions.push(`${item.name}: the two sleeves sit at the rail's own ends; each arm points from the rail toward the shoulder housing (model.fourBar.O4) at the basis's own length — the design gives no z/y split for that offset.`);
}

// ---- tail boom clamp blocks -----------------------------------------------------------------------------------
function frameBoom(fx) {
  const { ctx, A, assumptions, xA, xF, plateT } = fx;
  const item = ctx.structItem(/^tail boom clamp blocks/);
  const d = item && parseBlocks(item.basis);
  if (!d || A.boom_y_m == null) return;
  const len = toM(d.len);
  fx.clampX = { aft: xA - plateT / 2 - len / 2, fwd: xF + plateT / 2 + len / 2 };
  const objs = [['aft', fx.clampX.aft], ['fwd', fx.clampX.fwd]].map(([tag, x]) => {
    const m = ctx.mesh(`frame_boom_clamp_${tag}`, box(d.len, d.a, d.b), 'alu_machined', ctx.root, 'structure', item.material_id);
    m.position.set(x, A.boom_y_m, 0);
    return m;
  });
  ctx.register(item.name, item.material_id, item.name, 'structure', objs);
  assumptions.push(`${item.name}: the boom's bore is not modelled as a hole; the blocks sit flush outside each gearbox plate.`);
}

// ---- forward and aft trays, and their flange doublers ----------------------------------------------------------
function frameTray(fx, tag, plateFace, outward) {
  const { ctx, A, assumptions } = fx;
  const item = ctx.structItem(new RegExp(`^${tag} tray$`));
  const d = item && parseChannel(item.basis);
  const yM = A.trays?.[tag]?.y_m;
  if (!d || yM == null) return;
  const len = toM(d.len);
  const m = ctx.mesh(`frame_tray_${tag}`, box(d.len, d.flangeH, d.w), 'alu_machined', ctx.root, 'structure', item.material_id);
  m.position.set(plateFace + outward * len / 2, yM, 0);
  ctx.register(item.name, item.material_id, item.name, 'structure', m);
  fx.trayX[tag] = { face: plateFace };
  assumptions.push(`${item.name}: drawn as a solid box over the channel's width, flange height and overall length; the 10 mm root flange is not modelled as a separate bent tab, and the whole length is drawn outward from the plate.`);
}

function frameDoubler(fx, name, x, y, z) {
  const { ctx, assumptions } = fx;
  const item = ctx.structItem(new RegExp(`^${name}$`));
  const d = item && parsePlate(item.basis);
  if (!d) return;
  const m = ctx.mesh(`frame_${name.replace(/\s+/g, '_')}`, box(d.t, d.w, d.w), 'alu_machined', ctx.root, 'structure', item.material_id);
  m.userData.joint = /tray/.test(name) ? 'tray fwd' : 'motor';
  m.position.set(x, y, z);
  ctx.register(item.name, item.material_id, item.name, 'structure', m);
  assumptions.push(`${item.name}: drawn as a ${d.w} x ${d.w} mm square (the design gives only the flange's width and thickness, not its depth).`);
}

function frameTrays(fx) {
  const { xF, xA, plateT, A } = fx;
  frameTray(fx, 'forward', xF + plateT / 2, 1);
  frameTray(fx, 'aft', xA - plateT / 2, -1);
  if (fx.trayX.forward && A.trays?.forward) frameDoubler(fx, 'forward tray flange doubler', fx.trayX.forward.face, A.trays.forward.y_m, 0);
}

function frameMotorDoubler(fx) {
  const { ctx, xF, plateT } = fx;
  const mp = ctx.model.layout.parts.find((p) => p.name === 'motor');
  if (mp) frameDoubler(fx, 'motor doubler', xF + plateT / 2, mp.box.cy, mp.box.cz);
}

// ---- gearbox side and top panels -----------------------------------------------------------------------------
function framePanel(fx, item, name, list, center) {
  const { ctx } = fx;
  const d = parsePanel(item.basis, name);
  if (!d) return null;
  const isTop = name === 'top panel';
  const entry = list?.find((p) => p.name === name);
  const geo = box(d.gap + 2 * d.flange, isTop ? d.t : d.a, isTop ? d.a : d.t);
  const m = ctx.mesh(`frame_panel_${name.replace(/\s+/g, '_')}`, geo, 'alu_machined', ctx.root, 'structure', item.material_id);
  const y = isTop ? toM(entry?.y_mm ?? 56.5) : toM((entry?.y0_mm ?? 3) + d.a / 2);
  const z = isTop ? 0 : toM(entry?.z_mm ?? (name.includes('right') ? 31 : -31));
  m.position.set(center, y, z);
  return m;
}

function framePanels(fx) {
  const { ctx, assumptions, xA, xF } = fx;
  const item = ctx.structItem(/^gearbox panels/);
  if (!item) return;
  const list = ctx.D.params?.linkage?.airframe?.panels?.list;
  const center = (xA + xF) / 2;
  const objs = ['side panel right', 'side panel left', 'top panel'].map((nm) => framePanel(fx, item, nm, list, center)).filter(Boolean);
  if (!objs.length) return;
  ctx.register(item.name, item.material_id, item.name, 'structure', objs);
  assumptions.push(`${item.name}: each panel's z (or y) position comes from design.params.linkage.airframe.panels.list when present, else a default offset; its h/w, thickness and length come from the BOM's own basis text.`);
}

// ---- M3 screws and nuts ---------------------------------------------------------------------------------------
function drawFasteners(fx, item, points, kind, geo) {
  if (!item || !geo || !points.length) return;
  const { ctx } = fx;
  const objs = points.map((p, i) => {
    const m = ctx.mesh(`frame_${kind}_${i}`, geo, 'steel_ground', ctx.root, 'structure', item.material_id);
    m.position.set(...p);
    const j = fx.fastenerJoint?.[kind]?.[i];
    if (j) m.userData.joint = j;
    return m;
  });
  ctx.register(item.name, item.material_id, item.name, 'structure', objs);
}

function frameScrewsAndNuts(fx, screwItem, nutItem) {
  const { ctx, assumptions } = fx;
  const nScrew = offCount(screwItem), nNut = nutItem ? offCount(nutItem) : 0;
  const given = ctx.D.params?.linkage?.airframe?.rail?.lock_screws;
  const lock = given ?? 2, railN = 2 * lock;
  if (given == null) assumptions.push(`${screwItem.name}: assumed ${lock} shoulder rail lock screws per side (the design gives only the total screw count).`);
  const joints = fastenerJoints(fx);
  const holes = fx.bracketHoles ?? [];
  const keys = fx.v23 ? ['rail right', 'rail left', 'motor bracket', 'boom aft', 'boom fwd', 'tray fwd', 'tray aft', 'riser foot', 'riser tab'] : ['rail right', 'rail left', 'boom aft', 'boom fwd', 'tray fwd', 'tray aft', 'motor'];
  const screwN = fx.v23 ? [lock, lock, holes.length, ...splitEven(nScrew - railN - holes.length, 6)] : [lock, lock, ...splitEven(nScrew - railN, 5)];
  const nutN = fx.v23 ? [lock, lock, 0, ...splitEven(nNut - railN, 6)] : [lock, lock, ...splitEven(nNut - railN, 4), 0];
  const screws = [], nuts = [], at = { screw: [], nut: [] };
  keys.forEach((key, i) => {
    // a rail's lock screws on its flange's lands where the physics states them (v22: lock_arc_deg), else on a circle
    const lands = key === 'motor bracket' ? holes : fx.lockLands?.[key.replace(/^rail /, '')];
    if (lands && screwN[i] === lands.length) {
      if (key === 'motor bracket') { for (const c of lands) screws.push([joints[key][0], c[1], c[0]]); return; }
      for (const c of lands) { screws.push([joints[key][0], c[1], c[0]]); nuts.push([joints[key][0], c[1], c[0]]); }
      return;
    }
    scatter(joints[key], screwN[i], 0.01, screws);
    scatter(joints[key], nutN[i], 0.012, nuts);
  });
  // each fastener's joint, in the order they were placed (bird.js puts it in the assembly step making that joint)
  keys.forEach((key, i) => {
    if (!joints[key]) return;
    at.screw.push(...Array(screwN[i]).fill(key));
    at.nut.push(...Array(nutN[i]).fill(key));
  });
  fx.fastenerJoint = at;
  drawFasteners(fx, screwItem, screws, 'screw', screwGeo(ctx, screwItem));
  drawFasteners(fx, nutItem, nuts, 'nut', nutGeo(ctx, nutItem));
  assumptions.push(`${screwItem.name} and ${nutItem?.name ?? 'M3 nuts'}: beyond the rails' lock screws, the rest is split evenly across the two boom clamps, the two trays and the motor (no nut there — it threads into its own case), each on a small illustrative circle; the design gives no per-joint count or bolt-circle radius.`);
  assumptions.push(`${screwItem.name}: drawn as a single cylinder at the head diameter over the head-plus-shank length; the narrower shank is not modelled as a separate step.`);
  if (nutItem) assumptions.push(`${nutItem.name}: drawn as a round ring at the across-flats width and the bore, not the true hexagonal outline.`);
}

// ---- cable ties -------------------------------------------------------------------------------------------------
function frameTies(fx, tieItem) {
  const { ctx, assumptions } = fx;
  const nTie = offCount(tieItem);
  const geo = tieGeo(ctx, tieItem);
  if (!nTie || !geo) return;
  const joints = fastenerJoints(fx);
  const clusters = [joints['tray fwd'], joints['tray aft'], joints.motor].filter(Boolean);
  if (!clusters.length) return;
  const points = [];
  splitEven(nTie, clusters.length).forEach((n, i) => scatter(clusters[i], n, 0.008, points));
  const objs = points.map((p, i) => { const m = ctx.mesh(`frame_tie_${i}`, geo, 'acetal', ctx.root, 'structure', tieItem.material_id); m.position.set(...p); m.userData.joint = 'ties'; return m; });
  ctx.register(tieItem.name, tieItem.material_id, tieItem.name, 'structure', objs);
  assumptions.push(`${tieItem.name}: all ${nTie} are grouped at the forward tray, aft tray and motor/crank joints for illustration; the design gives a total count but no per-tie position (battery, ESC, receiver, airspeed, tail servos, FC, GPS, the crank encoder, the pitot probe, the fuselage formers and the tail surfaces each account for some of it, per the design's load-path note).`);
}

function frameFasteners(fx) {
  const { ctx } = fx;
  const screwItem = ctx.structItem(/^M3 screws/), nutItem = ctx.structItem(/^M3 nuts/);
  if (screwItem) frameScrewsAndNuts(fx, screwItem, nutItem);
  const ties = ctx.D.bom.structure.filter((s) => /^cable ties/.test(s.name));
  if (ctx.D.structure?.airframe?.parts?.support?.ties) for (const t of ties) supportTies(fx, t);
  else if (ties[0]) frameTies(fx, ties[0]);
}

/**
 * v23 (airframe.parts.support.ties): each packed part's ties round it and its carrier, at the BOM band, spaced along
 * the part; the pitot's ties are the physics' own solids (drive-solids.js). What the BOM line counts beyond those has no
 * stated place: drawn at the trays as before, and said so.
 */
function supportTies(fx, item) {
  const { ctx, assumptions } = fx;
  const S = ctx.D.structure.airframe.parts.support, objs = [];
  for (const t of S.ties.filter((x) => x.tie === item.material_id)) {
    const b = ctx.model.layout.parts.find((p) => p.name === t.part)?.box;
    if (!b) continue;
    const [w, th] = t.band_mm.map(toM), car = ctx.D.structure.airframe.parts.carriers?.[t.carrier.replace(/ tray$/, '')];
    const yb = Math.min(b.y0, (car?.y ?? b.y0) - (car?.t ?? 0)) - th;
    for (let k = 0; k < t.n; k++) {
      const m = ctx.mesh(`frame_tie_${t.part}_${k}`, loopGeo(ctx, b.z1 - b.z0 + 2 * th, b.y1 - yb + th, th, w), 'acetal', ctx.root, 'structure', item.material_id);
      m.position.set(b.x0 + ((k + 1) * (b.x1 - b.x0)) / (t.n + 1), (b.y1 + th + yb) / 2, (b.z0 + b.z1) / 2);
      m.userData.joint = 'ties';
      objs.push(m);
    }
  }
  const pitot = S.pitot?.tie === item.material_id ? S.pitot.n : 0;
  const rest = offCount(item) - objs.length - pitot;
  const geo = tieGeo(ctx, item), joints = fastenerJoints(fx);
  const at = [joints['tray fwd'], joints['tray aft']].filter(Boolean), pts = [];
  if (rest > 0 && geo && at.length) splitEven(rest, at.length).forEach((n, i) => scatter(at[i], n, 0.008, pts));
  pts.forEach((p, i) => { const m = ctx.mesh(`frame_tie_${item.material_id}_${i}`, geo, 'acetal', ctx.root, 'structure', item.material_id); m.position.set(...p); m.userData.joint = 'ties'; objs.push(m); });
  if (objs.length || pitot) ctx.register(item.name, item.material_id, item.name, 'structure', objs);
  if (rest > 0) assumptions.push(`${item.name}: ${objs.length - pts.length} round their parts and carriers (support.ties), ${pitot} round the pitot (the physics' solids), ${rest} with no stated place drawn at the trays.`);
}

/** A tie's loop: a rectangle w x h (z, y) outside, of band thickness t, w_x wide along x. */
function loopGeo(ctx, w, h, t, wx) {
  const s = new THREE.Shape();
  s.moveTo(-w / 2, -h / 2); s.lineTo(w / 2, -h / 2); s.lineTo(w / 2, h / 2); s.lineTo(-w / 2, h / 2); s.closePath();
  const hole = new THREE.Path();
  hole.moveTo(-w / 2 + t, -h / 2 + t); hole.lineTo(-w / 2 + t, h / 2 - t); hole.lineTo(w / 2 - t, h / 2 - t); hole.lineTo(w / 2 - t, -h / 2 + t); hole.closePath();
  s.holes.push(hole);
  const g = new THREE.ExtrudeGeometry(s, { depth: wx, bevelEnabled: false });
  g.translate(0, 0, -wx / 2);
  return ctx.geo.alongX(g);
}

/**
 * A design the physics lays out (ctx.phys): the rails, brackets, panels, boom, clamps and trays are its solids
 * (drive-solids.js). Here only the BOM lines the physics' check has no solid for: the rails' aft seat bosses, the two
 * doublers, the M3 screws and nuts and the cable ties, at the joints those solids put them.
 */
function physicsFrame(fx) {
  const { ctx, A } = fx;
  physicsJoints(fx);
  for (const side of ['right', 'left']) if (fx.railX[side]) frameRailBoss(fx, side, A.rails[side]);
  if (fx.trayX.forward) frameDoubler(fx, 'forward tray flange doubler', fx.trayX.forward.face, A.trays.forward.y_m, 0);
  motorDoubler(fx);
  frameFasteners(fx);
  fx.assumptions.push("frame (v22 on): shoulder rails, brackets, panels, boom, clamps and trays are the physics' own solids (drive-solids.js); the rails' aft seat bosses, the doublers, the screws, nuts and ties are not in the physics' clearance check and are placed by the film.");
  ctx.model.layout?.assumptions?.push(...fx.assumptions);
  return { assumptions: fx.assumptions };
}

/** The joints the fasteners and doublers sit at, from the physics' solids: rails, lock lands, clamps, trays, motor. */
function physicsJoints(fx) {
  const { ctx, A } = fx;
  const ph = ctx.phys, mid = (v) => (v[0] + v[1]) / 2;
  for (const side of ['right', 'left']) {
    const t = ph.solid(`shoulder rail ${side}`, 'tube');
    if (t && A.rails?.[side]) fx.railX[side] = { x0: t.x[0], x1: t.x[1] };
  }
  const lock = ph.solid('shoulder rail right', 'lock flange');
  if (lock) fx.lockX = mid(lock.x);
  fx.lockLands = lockLands(ctx.D.params?.linkage?.airframe?.rail, A.rails);
  const ca = ph.solid('boom clamp aft', 'block'), cf = ph.solid('boom clamp forward', 'block');
  if (ca && cf) fx.clampX = { aft: mid(ca.x), fwd: mid(cf.x) };
  const tf = ph.solid('forward tray', 'base'), ta = ph.solid('aft tray', 'base');
  if (tf) fx.trayX.forward = { face: tf.x[0], y: tf.shape.a[1] };
  if (ta) fx.trayX.aft = { face: ta.x[1], y: ta.shape.a[1] };
  // the motor's flange screws: through the forward plate into its face (v22: the motor inside the box on that plate)
  const can = ph.solid('motor', 'can');
  if (can) fx.motorAt = [ctx.P.stage1 === 'overhung' ? fx.xF : can.x[0], can.shape.c[1], can.shape.c[0]];
  // v23: the motor bracket's screws at its holes (into the motor's back face), the riser's foot and tab
  const mb = A.motor_bracket, ri = A.carriers?.riser;
  if (mb && ri) {
    fx.bracketHoles = mb.holes;
    const xr = (ri.x[0] + ri.x[1]) / 2;
    fx.v23 = { 'motor bracket': [(mb.x[0] + mb.x[1]) / 2, 0, 0], 'riser foot': [xr, ri.y[0], 0], 'riser tab': [xr, ri.y[1], 0] };
  }
}

/** Each rail's lock-screw lands as the physics states them (src/design/airframe.js lockFlangeCircles), or null. */
export function lockLands(R, rails) {
  if (!R?.lock_arc_deg || !R.lock_screws || !rails) return null;
  const out = {};
  for (const side of ['right', 'left']) if (rails[side]) out[side] = lockFlangeCircles(R, rails[side], side === 'right' ? 1 : -1).slice(1).map((q) => q.c);
  return out;
}

/** The motor doubler: on the forward plate's outer face round the motor's shaft, bored clear of its outboard bearing. */
function motorDoubler(fx) {
  const { ctx } = fx;
  const item = ctx.structItem(/^motor doubler$/), d = item && parsePlate(item.basis);
  if (!d || !fx.motorAt) return;
  const ob = ctx.phys.parts.find((p) => /^motor shaft outboard journal/.test(p.id))?.solids[0];
  const r = (ob ? ob.shape.ro : 0.004) + 0.0005, t = toM(d.t);
  const PL = ctx.D.params.linkage.plates, xf = ctx.P.plateFwd + toM(PL.t_fwd_mm ?? PL.t_mm) / 2;
  const m = ctx.mesh('frame_motor_doubler', squareWithHole(ctx, toM(d.w), t, r), 'alu_machined', ctx.root, 'structure', item.material_id);
  m.userData.joint = 'motor';
  m.position.set(ctx.P.stage1 === 'overhung' ? xf + t / 2 : fx.motorAt[0] - t / 2, fx.motorAt[1], fx.motorAt[2]);
  ctx.register(item.name, item.material_id, item.name, 'structure', m);
  fx.assumptions.push(`${item.name}: a ${d.w} mm square, ${d.t} mm, on the forward plate's outer face round the motor's shaft (the physics has no solid for it).`);
}

/** A square plate w x w, t thick along x, with a round hole of radius r at its centre. */
function squareWithHole(ctx, w, t, r) {
  const s = new THREE.Shape();
  s.moveTo(-w / 2, -w / 2);
  s.lineTo(w / 2, -w / 2);
  s.lineTo(w / 2, w / 2);
  s.lineTo(-w / 2, w / 2);
  s.closePath();
  const h = new THREE.Path();
  h.absarc(0, 0, r, 0, TWO_PI, true);
  s.holes.push(h);
  const g = new THREE.ExtrudeGeometry(s, { depth: t, bevelEnabled: false, curveSegments: 32 });
  g.translate(0, 0, -t / 2);
  return ctx.geo.alongX(g);
}

/** Draws and registers the frame parts of the design's BOM; returns the placement assumptions it had to make. */
export function buildFrame(ctx) {
  const AF = ctx.D.structure?.airframe;
  if (!AF) return { assumptions: [] };
  const A = AF.parts ?? {};
  const { P } = ctx;
  const fx = { ctx, A, assumptions: [], xA: P.plateAft, xF: P.plateFwd, plateT: P.plateT, xMid: (P.plateAft + P.plateFwd) / 2, railX: {}, trayX: {}, clampX: null };
  if (ctx.phys) return physicsFrame(fx);
  fx.assumptions.push("shoulder rails: centred fore-aft on the gearbox plates' midpoint (the design gives each rail's z/y position but not its x span); the 2 mm lock flange is not modelled as a separate step, it is part of the overall length.");
  for (const side of ['right', 'left']) frameRail(fx, side);
  frameBoom(fx);
  frameTrays(fx);
  frameMotorDoubler(fx);
  framePanels(fx);
  frameFasteners(fx);
  ctx.model.layout?.assumptions?.push(...fx.assumptions);
  return { assumptions: fx.assumptions };
}
