// The drive's and the frame's fixed parts drawn from the physics' own solids (solids.js physicsAssembly): the
// gearbox plates with their posts, pads and bosses, every plate journal, the gear shafts, the crank encoder, the
// shoulder bushings and brackets, the rails with their lock flanges, the panels, the tail boom and its clamps, the
// trays. Each mesh is the solid's prism, so it is exactly where and as big as the clearance check has it, and carries
// userData.physics (the part/solid it draws) for film/test/physics-solids.test.mjs. A part the physics adds that no
// builder here names is drawn from its solids too (drawRest), so a later design's new parts appear with no film work.
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

/** Every physics part no builder drew (a later design's new part), from its fixed solids. -> the ids drawn here */
export function drawRest(ctx) {
  const drawn = new Set();
  for (const o of ctx.meshes) for (const t of o.userData.physics ?? []) drawn.add(t.split('/')[0]);
  const rest = ctx.phys.parts.filter((p) => !drawn.has(p.id) && p.solids.some((s) => !s.at));
  for (const p of rest) {
    // its BOM line: the one the physics names (p.bom / p.material_id), else the structure line of its own name
    const sup = Object.values(ctx.D?.structure?.airframe?.parts?.support ?? {}).find((s) => s?.part && `${s.part} ties` === p.id);
    const id = p.bom ?? p.material_id ?? ctx.D?.bom?.structure?.find((s) => s.name === p.id)?.material_id ?? sup?.tie ?? null;
    const o = partMesh(ctx, `phys_${p.id.replace(/[^\w]+/g, '_')}`, p.id, { id });
    ctx.register(p.id, id, p.id, 'structure', o);
  }
  if (rest.length) ctx.model.layout?.assumptions?.push(`drawn from the physics' solids alone (no film builder names them): ${rest.map((p) => p.id).join(', ')}`);
  return rest.map((p) => p.id);
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
