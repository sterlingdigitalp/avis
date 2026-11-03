// The drive's and the frame's parts drawn from the physics' own solids (solids.js physicsAssembly): the gearbox
// plates with their posts, pads and bosses, every plate journal, the gear shafts, the crank encoder, the shoulder
// bushings and brackets, the rails with their lock flanges, the panels, the tail boom and its clamps, the trays. Each
// mesh is the solid's prism, so it is exactly where and as big as the clearance check has it, and carries
// userData.physics (the part/solid it draws) for film/test/physics-solids.test.mjs. Every other solid (v24: the
// fixings, ties, keys, formers, the crank's web, the couplers' eyes, the wings' and tail's structure, and any part a
// later design adds) is drawn from its solids by its BOM tag (drawRest, drawPosed): registered under the line the tag
// names, in the material and look its catalogue entry gives, so a new BOM line needs no film work.
import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { solidGeometry } from './solids.js';

const mergeAll = (ctx, solids) => (solids.length === 1 ? solidGeometry(solids[0]) : ctx.geo.mergeGeos(solids.map((s) => solidGeometry(s))));

/** One mesh drawing some of a physics part's solids (all of them by default), tagged with each. */
export function partMesh(ctx, name, partId, { solids = null, mat = 'alu_machined', sub = 'structure', id = null, parent = ctx.root } = {}) {
  const p = ctx.phys.part(partId);
  const list = (solids ?? p.solids.map((s) => s.name)).map((n) => p.solids.find((s) => s.name === n)).filter((s) => s && !s.at);
  if (!list.length) return null;
  const o = ctx.mesh(name, mergeAll(ctx, list), mat, parent, sub, id);
  o.userData.physics = list.map((s) => `${partId}/${s.name}`);
  return o;
}

const kindOf = (ctx, id) => (/bushing/.test(ctx.model.parts[id]?.category ?? id) ? 'bushing' : 'bearing');

/** A boss's shaft index from its solid's name ('boss crank shaft/aft' -> 0, 'boss gear shaft 2/forward' -> 2). */
export const bossShaft = (name) => (/crank shaft/.test(name) ? 0 : +(name.match(/gear shaft (\d+)/)?.[1] ?? -1));

/** One gearbox plate: its outline, posts and pads as one mesh, each boss its own. -> { plate, bosses } */
function plate(ctx, side, tag, ids) {
  const id = `gearbox plate ${side}`, p = ctx.phys.part(id);
  if (!p) return { plate: null, bosses: [] };
  const isBoss = (s) => /^boss /.test(s.name);
  const body = p.solids.filter((s) => !isBoss(s)).map((s) => s.name);
  return {
    plate: partMesh(ctx, `plate_${tag}`, id, { solids: body, sub: 'drive', id: ids.plate }),
    bosses: p.solids.filter(isBoss).map((s) => partMesh(ctx, `plate_boss_${bossShaft(s.name)}_${side}`, id, { solids: [s.name], sub: 'drive', id: ids.boss })),
  };
}

/** Gearbox plates (outline, posts and pads) and their bosses. */
function plates(ctx) {
  const item = ctx.structItem(/^gearbox plates/), bossItem = ctx.structItem(/plate bosses/);
  const ids = { plate: item?.material_id, boss: bossItem?.material_id ?? item?.material_id };
  const both = [plate(ctx, 'aft', 'aft', ids), plate(ctx, 'forward', 'fwd', ids)];
  const objs = both.map((b) => b.plate).filter(Boolean), bosses = both.flatMap((b) => b.bosses);
  if (objs.length) ctx.register('gearbox plates', ids.plate, 'gearbox plates', 'drive', objs);
  if (bosses.length) ctx.register('gearbox plate bosses', ids.boss, 'gearbox plate bosses', 'drive', bosses);
}

/** Every plate journal the physics seats, by its shaft and plate, with the catalogue part it is. */
function journalsOf(ctx) {
  const byId = new Map();
  for (const p of ctx.phys.parts) {
    const m = p.id.match(/^(crank shaft|gear shaft (\d+)|motor shaft outboard) journal (aft|forward) \((.+)\)$/);
    if (!m) continue;
    const id = m[4], kind = kindOf(ctx, id);
    const name = m[1] === 'motor shaft outboard' ? `motor_outboard_${kind}` : `gearbox_${kind}_${m[1] === 'crank shaft' ? 0 : m[2]}_${m[3] === 'aft' ? 'aft' : 'fwd'}`;
    const o = partMesh(ctx, name, p.id, { mat: kind === 'bushing' ? 'acetal' : 'steel_ground', sub: 'drive', id });
    if (!byId.has(id)) byId.set(id, { kind, meshes: [] });
    byId.get(id).meshes.push(o);
  }
  for (const [id, { kind, meshes }] of byId) ctx.register(`gearbox ${kind}s`, id, kind, 'drive', meshes);
}

/** Gear shafts plate to plate (round 14: to the far face of a gear outside the plates), their wheel seats. */
function gearShafts(ctx) {
  const gs = ctx.D.params.linkage.gearShafts ?? [];
  for (let j = 1; ctx.phys.part(`gear shaft ${j}`); j++) {
    const g = gs[j - 1] ?? {}, id = `gear shaft ${j}`;
    const mat = g.material ? 'alu_machined' : 'steel_shaft';
    const objs = [partMesh(ctx, `shaft_${j}`, id, { solids: ['shaft'], mat, sub: 'drive', id: g.material })];
    const seat = partMesh(ctx, `shaft_${j}_wheel_seat`, id, { solids: ['wheel seat'], mat, sub: 'drive', id: g.material });
    if (seat) objs.push(seat);
    ctx.register(id, g.material, 'gear shaft', 'drive', objs);
  }
}

/** The crank encoder's readhead and actuator at the physics' planes and catalogue sizes. */
function encoder(ctx) {
  const P = ctx.D.params?.parts ?? {};
  for (const [pid, name, mat, id, what] of [['crank encoder readhead', 'encoder_readhead', 'pcb', P.crankEncoder, 'crank angle encoder'], ['crank encoder actuator', 'encoder_magnet', 'magnet', P.encoderMagnet, 'crank encoder magnet']]) {
    if (!ctx.phys.part(pid)) continue;
    const o = partMesh(ctx, name, pid, { mat, sub: 'avionics', id });
    ctx.register(what, id, what, 'avionics', o);
  }
  encoderFixing(ctx);
}

/** v23: the readhead's fixing (the maker's M2 screws, spacers and nuts, airframe.parts.support.readhead), each under its BOM line. */
function encoderFixing(ctx) {
  const fx = ctx.phys.part('crank encoder readhead fixing');
  if (!fx) return;
  for (const [kind, re, mat] of [['head', /^ISO 4762.* M2 /, 'steel_ground'], ['spacer', /encoder spacers/, 'alu_machined'], ['nut', /^ISO 4032.* M2 /, 'steel_ground']]) {
    const it = ctx.structItem(re) ?? { name: `readhead ${kind}s`, material_id: null };
    const names = fx.solids.filter((s) => s.name.startsWith(kind)).map((s) => s.name);
    if (names.length) ctx.register(it.name, it.material_id, it.name, 'avionics', partMesh(ctx, `encoder_fixing_${kind}s`, fx.id, { solids: names, mat, sub: 'avionics', id: it.material_id }));
  }
}

/** One end's shoulder bushing, its housing (counterbored for the flange) and its bracket's arm and sleeve, into `out`. */
function shoulderEnd(ctx, tag, side, nm, out) {
  const { phys, journal } = ctx;
  const jh = journal('shoulder'), e = nm === 'aft' ? 'a' : 'f';
  const b = `shoulder bushing ${tag} ${nm}`, h = `shoulder bracket ${tag} ${nm}`;
  if (phys.part(b) && jh) out.bush.push(partMesh(ctx, `hinge_${jh.kind}_${tag}_${e}`, b, { mat: jh.kind === 'bearing' ? 'steel_ground' : 'acetal', sub: 'drive', id: jh.id }));
  if (!phys.part(h)) return;
  out.hous.push(partMesh(ctx, `shoulder_housing_${tag}_${e}`, h, { solids: ['housing', 'housing counterbore'], sub: 'drive', id: out.housingId }));
  for (const s of ['arm', 'sleeve']) out.brk.push(partMesh(ctx, `frame_bracket_${s}_${side}_${nm === 'aft' ? 'aft' : 'fwd'}`, h, { solids: [s], id: out.bracket?.material_id }));
}

/** Shoulder bushings and brackets, each side: registered as the BOM lists them (bushings, housings, bracket). */
function shoulders(ctx) {
  const jh = ctx.journal('shoulder'), housingId = ctx.D.params.linkage.shoulderHousing?.material;
  for (const [tag, side] of [['R', 'right'], ['L', 'left']]) {
    const out = { bush: [], hous: [], brk: [], housingId, bracket: ctx.structItem(new RegExp(`^shoulder bracket ${side}$`)) };
    for (const nm of ['aft', 'forward']) shoulderEnd(ctx, tag, side, nm, out);
    const brk = out.brk.filter(Boolean);
    if (out.bush.length) ctx.register(`hinge ${jh.kind}s ${tag}`, jh.id, jh.kind, 'drive', out.bush);
    if (out.hous.length) ctx.register(`shoulder housings ${tag}`, housingId, 'shoulder housings', 'drive', out.hous);
    if (brk.length) ctx.register(out.bracket?.name ?? `shoulder bracket ${side}`, out.bracket?.material_id, out.bracket?.name ?? 'shoulder bracket', 'structure', brk);
  }
}

/** Rails (tube and lock flange), panels, the tail boom and its clamps, the trays. */
function frame(ctx) {
  const { phys, structItem, register } = ctx;
  const reg = (re, objs) => {
    const it = structItem(re);
    const list = objs.filter(Boolean);
    if (list.length) register(it?.name ?? String(re), it?.material_id, it?.name ?? 'frame', 'structure', list);
    return it;
  };
  for (const side of ['right', 'left']) {
    const id = `shoulder rail ${side}`, it = structItem(new RegExp(`^${id}$`));
    if (!phys.part(id)) continue;
    reg(new RegExp(`^${id}$`), [partMesh(ctx, `frame_rail_${side}`, id, { solids: ['tube'], id: it?.material_id }), partMesh(ctx, `frame_rail_lock_${side}`, id, { solids: ['lock flange'], id: it?.material_id })]);
  }
  const pan = structItem(/^gearbox panels/);
  reg(/^gearbox panels/, phys.parts.filter((p) => /^gearbox (side|top) panel/.test(p.id)).map((p) => partMesh(ctx, `frame_panel_${p.id.replace(/^gearbox /, '').replace(/\s+/g, '_')}`, p.id, { id: pan?.material_id })));
  const clamps = structItem(/^tail boom clamp blocks/);
  reg(/^tail boom clamp blocks/, ['aft', 'forward'].map((nm) => phys.part(`boom clamp ${nm}`) && partMesh(ctx, `frame_boom_clamp_${nm === 'aft' ? 'aft' : 'fwd'}`, `boom clamp ${nm}`, { id: clamps?.material_id })));
  const keel = structItem(/keel/);
  if (phys.part('tail boom')) reg(/keel/, [partMesh(ctx, 'keel', 'tail boom', { mat: 'carbon_woven', id: keel?.material_id })]);
  for (const nm of ['forward', 'aft']) {
    const it = structItem(new RegExp(`^${nm} tray$`));
    if (phys.part(`${nm} tray`)) reg(new RegExp(`^${nm} tray$`), [partMesh(ctx, `frame_tray_${nm}`, `${nm} tray`, { id: it?.material_id })]);
  }
}

// ---- every other solid, keyed by its BOM tag ---------------------------------------------------------------------

/**
 * The film's material for a BOM line, read from its catalogue entry (its id, name, category and stated material), so a
 * new line draws in its own material with no film work: the first rule whose words the entry carries, else machined
 * aluminium. A part the film details (a gear, a bearing) keeps its builder's material.
 */
const MATERIAL_RULES = [
  [/(^|\s)tie-|\bcable tie\b|\bPA ?66\b|\bnylon\b|\bpolyamide\b/i, 'acetal'],
  [/\bbalsa\b|\bbasswood\b|\bplywood\b|\bwood\b/i, 'balsa'],
  [/\bmagnet\b|\bneodymium\b|\bNdFeB\b/i, 'magnet'],
  [/\b(lead|leads|wire|wiring)\b|\bcables?\b(?! ties?\b)/i, 'wire'],
  [/\bconnector\b|\bXT\d+\b|\bJST\b/i, 'pcb'],
  [/\bcarbon\b.*\b(plate|sheet)\b|\b(plate|sheet)\b.*\bcarbon\b|(^|\s)plate-ec-/i, 'carbon_plate'],
  [/\bcarbon\b|\bCFRP\b|(^|\s)(tube|rod)-(ec|rockwest|cst)-|\bpultruded\b/i, 'carbon_pultruded'],
  [/\btitanium\b|\bTi-\d|\bTi\d|-ti-|tigertitanium/i, 'titanium'],
  [/\bS45C\b|\bsteel\b.*\bbar\b/i, 'steel_shaft'],
  [/^fastener\b|\b(screw|nut|washer|circlip|retaining ring|bolt|dowel)\b|\bstainless\b|\bsteel\b/i, 'steel_ground'],
  [/\baluminium\b|\baluminum\b|-al-|\b(6061|7075|2024)\b/i, 'alu_machined'],
  [/\bacetal\b|\bPOM\b|\bdelrin\b|\bbushing\b|\bABS\b|\bPETG\b|\bPLA\b|\bfoam\b|\bEPP\b|\bEPS\b|\brubber\b|\bplastic\b|\bvelcro\b|\bhook and loop\b/i, 'acetal'],
  [/^servo\b/i, 'servo_case'],
  [/^battery\b/i, 'battery_wrap'],
  [/^esc\b/i, 'esc_wrap'],
  [/^gps\b/i, 'gps_case'],
  [/^(flight_controller|receiver|sensor|regulator|airspeed|barometer)\b/i, 'pcb'],
];
export function materialOf(parts, line) {
  const c = line && parts[line.id];
  if (!c) return 'alu_machined';
  const text = `${c.category ?? ''} ${line.id} ${c.name ?? ''} ${c.specs?.material ?? ''} ${c.specs?.alloy ?? ''}`;
  return MATERIAL_RULES.find(([re]) => re.test(text))?.[1] ?? 'alu_machined';
}

/**
 * How a BOM line is registered: a structure line under its own name and material, a bought part under its role and
 * catalogue id; an untagged solid under its part's id with no BOM id. -> { name, id }
 */
export function registryOf(ctx, line, partId) {
  if (!line) return { name: partId, id: null };
  if (line.kind === 'structure') return { name: line.key, id: line.id };
  return { name: ctx.D?.bom?.parts?.find((b) => b.id === line.id)?.role ?? line.name, id: line.id };
}

/**
 * The solids of a part no film mesh draws yet, grouped by the BOM line each is tagged with and the item of that line
 * it belongs to (solid.item: one tie of a part's several), so each mesh is one item as the BOM counts it.
 */
function undrawnByLine(ctx, drawn, p) {
  const groups = new Map();
  for (const s of p.solids) {
    if (drawn.has(`${p.id}/${s.name}`)) continue;
    const line = ctx.phys.line?.(s.bom) ?? null, key = `${line?.key ?? ''}|${s.item ?? ''}`;
    if (!groups.has(key)) groups.set(key, { line, solids: [] });
    groups.get(key).solids.push(s);
  }
  return [...groups.values()];
}

/** A mesh moved to the centre of its geometry (its vertices where they were): its position is where the part is. */
function centred(o) {
  if (!o.geometry?.attributes?.position) return o;
  o.geometry.computeBoundingBox();
  const c = o.geometry.boundingBox.getCenter(new THREE.Vector3());
  o.geometry.translate(-c.x, -c.y, -c.z);
  o.position.add(c);
  return o;
}

/** Every physics solid some mesh draws, as 'part/solid'. */
const drawnSolids = (ctx) => new Set(ctx.meshes.flatMap((o) => o.userData.physics ?? []).flatMap((t) => (t.includes('/') ? [t] : (ctx.phys.part(t)?.solids ?? []).map((s) => `${t}/${s.name}`))));

/** A mesh name for a part (and its line, when the part has solids of more than one; its one solid's name, if one). */
const meshName = (p, line, many, solids) => {
  const item = solids[0].item && solids[0].item !== p.id ? solids[0].item : null;
  return `phys_${item ?? p.id}${!item && many && line && solids.length > 1 ? `_${line.key}` : ''}${solids.length === 1 ? `_${solids[0].name}` : ''}`.replace(/[^\w]+/g, '_').replace(/_+$/, '');
};

/**
 * What a BOM line's part is, from its catalogue entry (category and name), for how it is drawn: a 'nut', a 'screw' or
 * a 'tie' (a catalogue fastener), else null.
 */
export function lookOf(parts, line) {
  const c = line && parts[line.id];
  if (c?.category !== 'fastener') return null;
  return /\bnut\b/i.test(c.name) ? 'nut' : /\b(screw|bolt)\b/i.test(c.name) ? 'screw' : /\btie\b/i.test(c.name) ? 'tie' : null;
}

/** A rectangle's (z, y) box, if a polygon is one with its sides along the axes: [[z0, y0], [z1, y1]], else null. */
function axisRect(sh) {
  if (sh.kind !== 'poly' || sh.pts.length !== 4 || sh.holes?.length) return null;
  const zs = [...new Set(sh.pts.map((p) => +p[0].toFixed(12)))], ys = [...new Set(sh.pts.map((p) => +p[1].toFixed(12)))];
  return zs.length === 2 && ys.length === 2 ? [[Math.min(...zs), Math.min(...ys)], [Math.max(...zs), Math.max(...ys)]] : null;
}

/**
 * A socket head cap screw's head on an axis along x: the disc, its outer half (the end away from the shank) with the
 * hexagon socket in it, across flats the ISO 4762 proportion of the head (2.5 of 5.5 mm at M3).
 */
function socketHead(s, sh, shank) {
  const outerAft = Math.abs(shank.x[0] - s.x[1]) < Math.abs(shank.x[1] - s.x[0]); // the shank forward of the head
  const h = s.x[1] - s.x[0], d = h / 2, af = (2.5 / 5.5) * 2 * sh.ro, rc = af / Math.sqrt(3);
  const disc = (x0, x1, socket) => {
    const shape = new THREE.Shape();
    shape.absarc(-sh.c[0], sh.c[1], sh.ro, 0, 2 * Math.PI, false);
    if (socket) {
      const hex = new THREE.Path();
      for (let k = 0; k < 6; k++) { const a = (k * Math.PI) / 3; hex[k ? 'lineTo' : 'moveTo'](-(sh.c[0] + rc * Math.cos(a)), sh.c[1] + rc * Math.sin(a)); }
      hex.closePath();
      shape.holes.push(hex);
    }
    const g = new THREE.ExtrudeGeometry(shape, { depth: x1 - x0, bevelEnabled: false, curveSegments: 28 });
    g.rotateY(Math.PI / 2);
    g.translate(x0, 0, 0);
    return g;
  };
  const parts = outerAft ? [disc(s.x[0], s.x[0] + d, true), disc(s.x[0] + d, s.x[1], false)] : [disc(s.x[0], s.x[1] - d, false), disc(s.x[1] - d, s.x[1], true)];
  return mergeGeometries(parts);
}

/**
 * How a solid is drawn, inside it and reaching its x range and outer section (film/test/physics-solids.test.mjs):
 *  - a tube along the span (solid.tube.axis 'span': the physics' capsule round its axis over the tube's own diameter in
 *    x) round: the capsule swept about its axis, a cylinder with round ends;
 *  - a nut (lookOf) on an axis along x: the hexagon in its ring (the physics' ring is round its corners), with its bore;
 *  - a screw's head on an axis along x: a socket head (socketHead);
 *  - a screw's or nut's piece on an axis across x (the physics' square prism across x and z, along y): the round piece
 *    in it;
 *  - anything else: its prism.
 */
export function hardwareGeometry(s, sh = s.shape, look = null, siblings = []) {
  const head = look === 'screw' && sh.kind === 'ring' && !(sh.ri > 0) && siblings.find((o) => o !== s && o.shape?.kind === 'ring' && o.shape.ro < sh.ro && Math.hypot(o.shape.c[0] - sh.c[0], o.shape.c[1] - sh.c[1]) < 1e-9);
  if (head) return socketHead(s, sh, head);
  if (sh.kind === 'cap' && s.tube?.axis === 'span' && Math.abs(s.x[1] - s.x[0] - 2 * sh.r) < 1e-9) {
    const L = Math.hypot(sh.b[0] - sh.a[0], sh.b[1] - sh.a[1]);
    const g = new THREE.CapsuleGeometry(sh.r, L, 6, 20);
    // CapsuleGeometry lies along +y: turn it onto the (z, y) direction a -> b, centred on the segment and the x range
    const dir = new THREE.Vector3(0, sh.b[1] - sh.a[1], sh.b[0] - sh.a[0]).normalize();
    if (L > 1e-12) g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir));
    g.translate((s.x[0] + s.x[1]) / 2, (sh.a[1] + sh.b[1]) / 2, (sh.a[0] + sh.b[0]) / 2);
    return g;
  }
  if (look === 'nut' && sh.kind === 'ring') {
    const hex = new THREE.Shape();
    for (let k = 0; k < 6; k++) { const a = (k * Math.PI) / 3; hex[k ? 'lineTo' : 'moveTo'](-(sh.c[0] + sh.ro * Math.cos(a)), sh.c[1] + sh.ro * Math.sin(a)); }
    hex.closePath();
    if (sh.ri > 0) { const h = new THREE.Path(); h.absarc(-sh.c[0], sh.c[1], sh.ri, 0, 2 * Math.PI, true); hex.holes.push(h); }
    const g = new THREE.ExtrudeGeometry(hex, { depth: s.x[1] - s.x[0], bevelEnabled: false, curveSegments: 24 });
    g.rotateY(Math.PI / 2);
    g.translate(s.x[0], 0, 0);
    return g;
  }
  const r = (look === 'nut' || look === 'screw') && axisRect(sh);
  if (r && Math.abs(r[1][0] - r[0][0] - (s.x[1] - s.x[0])) < 1e-9) {
    const d = r[1][0] - r[0][0], h = r[1][1] - r[0][1];
    const g = new THREE.CylinderGeometry(d / 2, d / 2, h, 28);
    g.translate((s.x[0] + s.x[1]) / 2, (r[0][1] + r[1][1]) / 2, (r[0][0] + r[1][0]) / 2);
    return g;
  }
  return solidGeometry(s, sh);
}

/**
 * Every fixed physics solid no builder drew (the fixings, ties, keys, formers, the tail's structure, a later design's
 * new part), from its own solids: one mesh per part and BOM line, registered under the line its tag names
 * (completeness.js resolveTag), in its catalogue material. -> the part ids drawn here
 */
export function drawRest(ctx) {
  const drawn = drawnSolids(ctx), out = [];
  for (const p of ctx.phys.parts) {
    const groups = undrawnByLine(ctx, drawn, p).map((g) => ({ ...g, solids: g.solids.filter((s) => !s.at) })).filter((g) => g.solids.length);
    for (const g of groups) {
      const reg = registryOf(ctx, g.line, p.id), look = lookOf(ctx.model.parts ?? {}, g.line);
      const geo = g.solids.length === 1 ? hardwareGeometry(g.solids[0], g.solids[0].shape, look) : ctx.geo.mergeGeos(g.solids.map((s) => hardwareGeometry(s, s.shape, look, g.solids)));
      const o = centred(ctx.mesh(meshName(p, g.line, groups.length > 1, g.solids), geo, materialOf(ctx.model.parts ?? {}, g.line), ctx.root, subsystemOf(ctx, p), reg.id));
      o.userData.physics = g.solids.map((s) => `${p.id}/${s.name}`);
      ctx.register(reg.name, reg.id, reg.name, subsystemOf(ctx, p), o);
    }
    if (groups.length) out.push(p.id);
  }
  if (out.length) ctx.model.layout?.assumptions?.push(`drawn from the physics' solids alone, under the BOM lines their tags name (no film builder names them): ${out.join(', ')}`);
  return out;
}

/** A part's subsystem: the drive's when it is keyed (a key in its keyseat) or turns with the drive, else the structure's. */
function subsystemOf(ctx, p) {
  if (ctx.phys.joins?.some(([a, b, w]) => /keyseat/.test(w ?? '') && (a.split('/')[0] === p.id || b.split('/')[0] === p.id))) return 'drive';
  return 'structure';
}

/**
 * Material points of solids at a pose, in body coordinates, at both ends of each x range: a ring's centre, a
 * capsule's two ends, a polygon's corners (points that move with the part, whatever turns it).
 */
function solidPoints(solids, q) {
  return solids.flatMap((s) => {
    const sh = s.at ? s.at(q) : s.shape;
    const zy = sh.kind === 'ring' ? [sh.c] : sh.kind === 'cap' ? [sh.a, sh.b] : sh.pts;
    return s.x.flatMap((x) => zy.map(([z, y]) => new THREE.Vector3(x, y, z)));
  });
}

/** A node's matrix relative to the bird's root. */
function inRoot(ctx, n) {
  ctx.root.updateMatrixWorld(true);
  return new THREE.Matrix4().copy(ctx.root.matrixWorld).invert().multiply(n.matrixWorld);
}

/**
 * The posed node some moving solids of a part move with: of the ctx.posed nodes in whose frame all their material
 * points stay put between two drive poses (pose(phase) poses the bird and returns the physics' drive pose there), the
 * first that carries a part this one is joined to and goes on with (the same step of the physics' sequence: a bushing
 * with the eye it sits in, though its centre also rides the pin), else the first. -> node, or null
 */
function nodeOf(ctx, partId, solids, pose, nodes) {
  const at = (phase) => {
    const q = pose(phase);
    return nodes.map((n) => { const inv = inRoot(ctx, n).invert(); return solidPoints(solids, q).map((v) => v.applyMatrix4(inv)); });
  };
  const a = at(0.4), b = at(2.3);
  const fits = nodes.filter((n, i) => a[i].every((v, k) => v.distanceTo(b[i][k]) < 1e-7));
  const stepOf = (id) => (ctx.phys.steps ?? []).findIndex((st) => st.solids.some((t) => t.split('/')[0] === id));
  const mine = stepOf(partId);
  const joined = new Set(ctx.phys.joins.flatMap(([x, y]) => { const [px, py] = [x.split('/')[0], y.split('/')[0]]; return px === partId ? [py] : py === partId ? [px] : []; }).filter((id) => stepOf(id) === mine));
  const carries = (n) => n.children.some((o) => (o.userData.physics ?? []).some((t) => joined.has(t.split('/')[0])));
  return fits.find(carries) ?? fits[0] ?? null;
}

/**
 * Every moving physics solid no builder drew (the crank's web and pin, the wings' structure, a later design's moving
 * part): each in the frame of the posed node it moves with (nodeOf), drawn at the reference pose and carried by that
 * node, so it is where the physics has it at every pose. pose(phase) poses the bird; ref is the phase to draw at.
 * -> the part ids drawn here; a solid no node carries is reported in the layout's assumptions.
 */
export function drawPosed(ctx, pose, ref = 0) {
  const drawn = drawnSolids(ctx), out = [], lost = [];
  const nodes = [...new Set(ctx.posed)].filter((n) => n.isObject3D);
  for (const p of ctx.phys.parts) {
    // one mesh per part and BOM line: one item, as the BOM counts it
    const groups = undrawnByLine(ctx, drawn, p).map((g) => ({ ...g, solids: g.solids.filter((s) => s.at) })).filter((g) => g.solids.length);
    for (const g of groups) {
      const node = nodeOf(ctx, p.id, g.solids, pose, nodes);
      if (!node) { lost.push(`${p.id}`); continue; }
      const q = pose(ref), inv = inRoot(ctx, node).invert();
      const look = lookOf(ctx.model.parts ?? {}, g.line);
      const geo = (g.solids.length === 1 ? hardwareGeometry(g.solids[0], g.solids[0].at(q), look) : ctx.geo.mergeGeos(g.solids.map((s) => hardwareGeometry(s, s.at(q), look)))).applyMatrix4(inv);
      const reg = registryOf(ctx, g.line, p.id), sub = node.userData.subsystem ?? 'drive';
      const o = centred(ctx.mesh(meshName(p, g.line, groups.length > 1, g.solids), geo, materialOf(ctx.model.parts ?? {}, g.line), node, sub, reg.id));
      o.userData.physics = g.solids.map((s) => `${p.id}/${s.name}`);
      ctx.register(reg.name, reg.id, reg.name, sub, o);
      if (!out.includes(p.id)) out.push(p.id);
    }
  }
  if (lost.length) ctx.model.layout?.assumptions?.push(`not drawn: no posed node carries the moving solids of ${lost.join(', ')}`);
  return out;
}

/** The fixed drive and frame parts of a design the physics lays out (solids.js physicsAssembly). */
export function buildFixedFromPhysics(ctx) {
  plates(ctx);
  journalsOf(ctx);
  gearShafts(ctx);
  encoder(ctx);
  shoulders(ctx);
  frame(ctx);
}
