// Round 16 (SPEC "Completeness and assembly order", 2026-11-03): every screw and nut the airframe counts, at a stated
// place. The airframe's bolt checks (src/design/airframe.js) count the M3 screws and nuts of each joint; this module
// says where each one is: its axis, the face its head bears on, the parts it clamps and the face its nut bears on,
// and picks its length (the shortest catalogue screw of the design's thread that takes the stack and a full nut). The
// BOM counts the screws by length from this layout, and the assembly check (src/design/clearance.js fixingParts)
// draws each screw with its head and shank in its hole, and each nut.
import { byId } from './catalog.js';
import { fastenerGeometry } from './completeness.js';
import { lockFlangeCircles, panelFlanges } from './airframe.js';

const m = (v) => v / 1000;

/**
 * The shortest screw in `screws` (catalogue ids, one thread) whose shank takes the stack plus a full nut (or, tapped,
 * the stack plus at least 1.5 d of thread): -> { id, g } or null.
 */
export function pickScrew(catalog, screws, stack, nutM, tapped = false) {
  const opts = screws.map((id) => ({ id, g: fastenerGeometry(byId(catalog, id).specs ?? {}, id) })).sort((a, b) => a.g.L - b.g.L);
  return opts.find((o) => o.g.L >= stack + (tapped ? 1.5 * o.g.d : nutM) - 1e-9) ?? null;
}

/**
 * A row of n screws at pitch s (m) along one axis, centred at c0: their offsets.
 */
const row = (n, s) => Array.from({ length: n }, (_, i) => (i - (n - 1) / 2) * s);

/**
 * The bolt layout of an airframe context (airframe.js ax, after its members): one entry per screw. Each entry:
 * { joint, screw, g (its geometry), nut, ng, axis 'x' | 'y', c ([z, y] for an x screw, [x, z] for a y screw), face
 * (the head's bearing face along the axis), dir (the shank's direction, +1 or -1), stack_m, nut_face, through (the
 * parts it clamps, head side first), tapped (the part it threads into) }.
 */
export function fixingLayout(ax) {
  const k = layoutKit(ax);
  lockBolts(ax, k);
  clampBolts(ax, k);
  if (ax.carriers?.geometry) carrierBolts(ax, k);
  // the motor on its bracket: four screws from the bracket's aft face into the motor's back-face threads
  const mb = ax.motorBracket;
  if (mb) for (const c of mb.holes) k.bolt('motor bracket', 'x', c, mb.x[0], 1, mb.t_m, ['motor bracket'], { tapped: 'motor' });
  return k.out;
}

/** What every joint's bolts need: the plates' faces, the catalogue's screws and nut, and the bolt maker. */
function layoutKit(ax) {
  const { catalog, A, tFwd, tAft } = ax;
  const tF = m(tFwd), tAm = m(tAft), screws = A.screws ?? [A.fastener], nut = A.nut;
  const ng = fastenerGeometry(byId(catalog, nut).specs ?? {}, nut);
  const out = [];
  // round 16: a panel flange on a plate's inner face under a screw is clamped with the plate (panelFlanges)
  const flanges = panelFlanges(A, ax.planes, tAm, tF);
  const layer = (plate, c) => flanges.filter((f) => f.end === plate && f.rects.some((r) => c[0] >= r.z[0] && c[0] <= r.z[1] && c[1] >= r.y[0] && c[1] <= r.y[1]));
  const withInner = (plate, c, face, dir, stack, through, headInside) => {
    const L = layer(plate, c), t = L.reduce((a, f) => a + f.t, 0);
    // the head inside: it bears on the flange (its face moves in); the nut inside: the flange adds to the stack
    return { face: headInside ? face + dir * -t : face, stack: stack + t, through: headInside ? [...L.map((f) => f.part), ...through] : [...through, ...L.map((f) => f.part)] };
  };
  const bolt = (joint, axis, c, face0, dir, stack0, through0, opt = {}) => {
    const tapped = opt.tapped ?? null;
    const { face, stack, through } = opt.plate ? withInner(opt.plate, c, face0, dir, stack0, through0, opt.headInside) : { face: face0, stack: stack0, through: through0 };
    const s = pickScrew(catalog, screws, stack, ng.m, !!tapped);
    if (!s) throw new Error(`${joint}: no catalogue screw takes a ${(stack * 1000).toFixed(2)} mm stack`);
    out.push({ joint, screw: s.id, g: s.g, nut: tapped ? null : nut, ng: tapped ? null : ng, axis, c, face, dir, stack_m: stack, nut_face: tapped ? null : face + dir * stack, through, tapped });
  };
  return { tF, tAm, out, bolt };
}

/**
 * The rails' torsion locks, each land's screw through its plate and the rail's lock flange: lock_at 'forward' from the
 * forward plate's outer face aft (the flange on its aft face); 'aft outer' (round 16) from inside the box on the aft
 * plate's front face aft through the plate (and an aft seat boss) to the flange on its aft face; else (v21) from the
 * aft plate's aft face forward to the flange on its front face.
 */
function lockBolts(ax, { tF, tAm, bolt }) {
  const { A, xA, xF, railPos } = ax, ra = A.rail, lt = m(ra.lock_t_mm ?? 2);
  if (!ra.lock_at || !ra.lock_arc_deg) return;
  const outerAft = ra.lock_at === 'aft outer', boss = outerAft && ra.aft_boss_at === 'aft face' ? m(ra.aft_boss_mm ?? 0) : 0;
  for (const [side, sg] of [['right', 1], ['left', -1]]) for (const q of lockFlangeCircles(ra, railPos[side], sg).slice(1)) {
    const joint = `rail lock ${side}`, rail = `shoulder rail ${side}`;
    if (ra.lock_at === 'forward') bolt(joint, 'x', q.c, xF + tF / 2, -1, tF + lt, ['gearbox plate forward', rail]);
    else if (outerAft) bolt(joint, 'x', q.c, xA + tAm / 2, -1, tAm + boss + lt, ['gearbox plate aft', ...(boss ? [`${rail} aft seat boss`] : []), rail], { plate: 'aft', headInside: true });
    else bolt(joint, 'x', q.c, xA - tAm / 2, 1, tAm + lt, ['gearbox plate aft', rail]);
  }
}

/** The boom clamps: a row through each clamp's screw flange and its plate's pad, heads outboard, nuts inside. */
function clampBolts(ax, { tF, tAm, bolt }) {
  const { A, xA, xF } = ax, bo = A.boom, yB = ax.yB, cl = m(bo.clamp_len_mm), y = yB - m(bo.screw_offset_mm);
  const [hA, hF] = ax.d.params.layout?.plate_faces === 'actual' ? [tAm / 2, tF / 2] : [ax.plateT / 2, ax.plateT / 2];
  for (const z of row(bo.screws, m(bo.screw_pitch_mm))) {
    bolt('boom clamp aft', 'x', [z, y], xA - hA - cl, 1, cl + tAm, ['boom clamp aft', 'gearbox plate aft']);
    bolt('boom clamp forward', 'x', [z, y], xF + hF + cl, -1, cl + tF, ['boom clamp forward', 'gearbox plate forward']);
  }
}

/**
 * The carriers' screws: the forward tray's root tab on the forward plate (its doubler between them on the plate's
 * outer face, tray.doubler_at 'outer'), heads in the channel, nuts inside the box; the aft tray's on the aft plate; the
 * riser's foot on the forward tray's base and the nose tray's base on the riser's tab, vertical, heads up.
 */
function carrierBolts(ax, { tF, tAm, bolt }) {
  const { A } = ax, G = ax.carriers.geometry, F = G.forward, AT = G.aft, R = G.riser, N = G.nose;
  const dbl = m(A.tray.doubler_mm ?? 0), outer = A.tray.doubler_at === 'outer', yTab = (c) => c.y + c.t / 2 + c.tab / 2, xm = (a, b) => (a + b) / 2;
  const fwdStack = ['forward tray', ...(dbl && outer ? ['forward tray flange doubler'] : []), 'gearbox plate forward', ...(dbl && !outer ? ['forward tray flange doubler'] : [])];
  for (const z of row(A.tray.screws, m(A.tray.screw_pitch_mm))) bolt('forward tray', 'x', [z, yTab(F)], F.x[0] + F.t, -1, F.t + tF + dbl, fwdStack, { plate: 'fwd' });
  for (const z of row(A.aftTray.screws, m(A.aftTray.screw_pitch_mm))) bolt('aft tray', 'x', [z, yTab(AT)], AT.x[1] - AT.t, 1, AT.t + tAm, ['aft tray', 'gearbox plate aft'], { plate: 'aft' });
  for (const z of row(A.noseTray.riser.screws, m(A.noseTray.riser.screw_pitch_mm))) bolt('nose tray riser', 'y', [xm(R.x[0] - R.foot, R.x[0]), z], R.y[0] + R.t, -1, R.t + F.t, ['nose tray riser', 'forward tray']);
  for (const z of row(A.noseTray.screws, m(A.noseTray.screw_pitch_mm))) bolt('nose tray', 'y', [xm(R.x[1], R.x[1] + R.foot), z], N.y + N.t / 2, -1, N.t + R.t, ['nose tray', 'nose tray riser']);
}

/** The layout's screws and nuts by catalogue id: -> { screws: Map(id -> n), nuts: Map(id -> n) }. */
export function fixingCounts(layout) {
  const screws = new Map(), nuts = new Map();
  for (const b of layout) {
    screws.set(b.screw, (screws.get(b.screw) ?? 0) + 1);
    if (b.nut) nuts.set(b.nut, (nuts.get(b.nut) ?? 0) + 1);
  }
  return { screws, nuts };
}
