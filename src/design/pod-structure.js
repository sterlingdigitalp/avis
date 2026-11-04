// The pod's structure and covering (round 17, SPEC "Function completeness": "The covering spans only what is supported";
// audit v24 item 4, findings A1, F9, D5). v24 drew five balsa rings on 2 x 2 mm tabs under a film the loft drew from a
// nose point to a 10 mm tube at x -1089 mm that enclosed nothing. Here:
//  - Longerons (Easy Composites 1.5 mm pultruded rod) lie on the loft at fixed polar angles: the film rests on them.
//    Three sets, each straight along the loft's lines between its sections: the nose (its end former to the loft's
//    first kink), the mid body (that kink to where the cone starts) and the cone (to the tail's end former). The
//    loft's kinks at those two stations are too sharp for the rod to bend round (the bend rows), so the sets meet on a
//    former there. A set's rod is cut where a window (the hatch, the wing-root and horn slots, a pushrod's exit) lies
//    across its angle; each cut lands on the former at the window's edge.
//  - Formers (Easy Composites 1 mm carbon plate) are bands under the longerons (their outside the rod's diameter in
//    from the loft, so the rods rest on them), arcs where a window lies across them, end discs where the loft closes.
//    The two beside the gearbox plates are bonded on the plates' boom posts; the rest hang on the rods.
//  - The loft ends where the structure does: the nose at its end disc (balsa has no bond rating, and no catalogued
//    screw is long enough to hold a block, so the nose closes on carbon), the cone at its end disc (the v24 tube aft of
//    it is gone).
//  - The film (ORACOVER AIR LIGHT with the maker's brushed-on hotmelt) is stated over the loft, less the windows.
// The covering record (coveringRecord) holds every film span to the rule it states, with its source.
// Frame: x forward, y up, z right (body); the polar angle as clearance.js containmentRecord measures it (the elliptic
// section's parametric angle: 0 deg at +z, 90 at the top).
import { S, poly, ring, declaredOpening } from './clearance.js';
import { byId } from './catalog.js';
import { loftHalf, drivePose } from '../../sims/avis-shared/model.js';

const mm = (v) => v / 1000;
const DEG = Math.PI / 180;
const r3 = (v) => +(v * 1000).toFixed(3);
const norm360 = (a) => ((a % 360) + 360) % 360;

/** The catenoid's existence bound between two coaxial rings (covering-ref-catenoid-critical-ratio, MathWorld Eq. 26). */
export const CATENOID_2H_OVER_R = 1.3254868386;

/** The catalogue's reference entries the covering record takes its figures from (parts-v25/covering.json). */
export const COVERING_SOURCES = { catenoid: 'covering-ref-catenoid-critical-ratio', suction: 'covering-ref-myers-1992-rib-suction', adhesive: 'covering-ref-oracover-air-light-application' };

/**
 * The rules' figures from the catalogue: the catenoid's 2h/R bound, Myers' suction rule (its text), the film's adhesive
 * power (N/m, parsed from the maker's "Adhesive power: min. 7 N / 25 mm"); null where the entry or figure is missing.
 */
export function coveringSources(catalog) {
  const at = (id) => (catalog ?? []).find((q) => q.id === id)?.specs ?? null;
  const c = at(COVERING_SOURCES.catenoid), m = at(COVERING_SOURCES.suction), o = at(COVERING_SOURCES.adhesive);
  const a = /Adhesive power: min\. ([\d.]+) N \/ ([\d.]+) mm/.exec(o?.general_text ?? '');
  return { catenoid: Number.isFinite(c?.critical_separation_over_radius) ? c.critical_separation_over_radius : null, suction: m?.rule ?? null, adhesive: a ? +a[1] / (+a[2] / 1000) : null };
}

/**
 * The pod's structure (mm, deg). params.pod merges over it (formers, longerons and windows replace whole lists).
 *  - formers: each { name, x_mm } (its faces), { name, edge: [window, 'fwd'|'aft'] } (outside that window's edge),
 *    { name, section_near_mm } (centred on the loft's section nearest there) or { name, end: 'nose'|'tail'} (a disc
 *    closing the loft); split: in two halves (it is trapped between the gearbox plates' posts); plate: { part, post,
 *    w_mm } (its band bonded flat on that gearbox plate's post, w_mm wide, where it crosses the band's top); deep:
 *    { arc, band_mm } (the band deeper over that polar arc: round the post, where it carries the pod to it).
 *  - longerons: sets { name, from, to (formers), deg }.
 *  - windows: { name, x_mm, polar_deg: [from, to] counter-clockwise, openings } where the film is cut, framed by the
 *    formers at its x edges and the longerons at its polar edges; the hatch's window (params.layout.hatch) is added.
 *  - holders: declared openings whose fixed part holds the film's cut edge (bonded round it).
 */
export const POD = {
  former: 'plate-ec-1mm', longeron: 'rod-ec-1-5mm', adhesive: 'adhesive-araldite-2011',
  band_mm: 2, gap_mm: 0.02, piece_err_mm: 2.5, piece_max_mm: 200, nose_end_w_mm: 12, V_drag_m_s: 25, step_deg: 20,
  formers: [
    { name: 'nose end', end: 'nose' },
    { name: 'hatch forward', edge: ['hatch', 'fwd'] },
    { name: 'nose kink', section_near_mm: 216.9 },
    { name: 'nose 176', section_near_mm: 176 },
    { name: 'hatch aft', edge: ['hatch', 'aft'] },
    { name: 'nose 135', section_near_mm: 135 },
    { name: 'nose 94', section_near_mm: 94.1 },
    { name: 'forward plate', x_mm: [76.2, 77.2], plate: { part: 'gearbox plate forward', post: 'post 1 (boom clamp)', w_mm: 16 }, deep: { arc: [72, 108], band_mm: 4 } },
    { name: 'aft plate', edge: ['wing roots and horn slots', 'fwd'], split: true, band_mm: 1.5, step_deg: 10, plate: { part: 'gearbox plate aft', post: 'post 1 (boom clamp)', w_mm: 16 }, deep: { arc: [72, 108], band_mm: 4 } },
    { name: 'slots aft', edge: ['wing roots and horn slots', 'aft'] },
    { name: 'mid aft', x_mm: [-68.5, -67.5] },
    { name: 'cone start', section_near_mm: -110.5 },
    // round 18: the pushrod exits run from this former aft to the tail kink's (see windows); round 19: the exits closed,
    // the exit guides' lugs bonded on its aft face (tail-mech.js exitGuide)
    { name: 'pushrod exits forward', x_mm: [-208, -207] },
    { name: 'tail kink', section_near_mm: -274.2 },
    { name: 'tail end', end: 'tail' },
  ],
  longerons: [
    // 20 and 160 deg: the hatch window's polar edges (params.layout.hatch, round 18: the cover's lands and the pack lift
    // clear of them at the window's narrow forward end)
    { name: 'nose', from: 'nose end', to: 'nose kink', deg: [345, 20, 90, 160, 195, 270], piece_err_mm: 2 },
    // the two beside the battery's and the nose tray's front corners: drawn closer to the true rod
    { name: 'nose sides', from: 'nose end', to: 'nose kink', deg: [226, 314], piece_err_mm: 0.75 },
    // the battery's and the nose tray's front left corners stand within 1.3 mm of the loft at x 224-231 (polar 215-219
    // deg): the film clears them between this short rod and the nose's 226 deg rod; the right ones lie in the probe's window
    { name: 'nose corner', from: 'hatch forward', to: 'nose kink', deg: [210], piece_err_mm: 0.3 },
    // the belly's five (round 18: 236/252/270/288/304 deg, in place of 240/270/300): the twist-servo leads' pod runs and
    // the harness ties lie along the belly 0.3 mm inside the 30 deg panels' chords; on these the film clears them by 0.88 mm
    { name: 'mid', from: 'nose kink', to: 'cone start', deg: [330, 0, 20, 40, 70, 110, 140, 160, 180, 210, 236, 252, 270, 288, 304] },
    // 324/5 and 175/216 deg: the tail pushrod exits' polar edges (round 18: from the drawn pushrods, whose skin crossing
    // reaches 357.0 / 183.0 deg; was 355 / 185); 35 and 145 (were 30 and 150) so the rods beside them stay 0.5 mm apart
    // where the cone closes on its 10 mm end disc (round 19: the exits closed, the film spans 324-5 and 175-216 deg over
    // the exit guides, its cut bonded round each)
    { name: 'cone', from: 'cone start', to: 'tail end', deg: [324, 5, 35, 70, 110, 145, 175, 216, 250, 290], piece_err_mm: 4 },
  ],
  windows: [
    { name: 'wing roots and horn slots', x_mm: [-28, 15.2], polar_deg: [-30, 210], openings: ['wing root, right', 'wing root, left', 'horn slots'], why: 'the spars, rocker sockets and horn eyes sweep through the slots; the bridges between the slots are too narrow to frame, and the aft shoulder brackets stand within 3 mm of the skin, so the cut runs from the aft plate to clear of the brackets (round 18: aft to x -28, past where the twist-servo leads\' pod runs come up under the skin at x -27.2)' },
    // round 19 (decision 2): the pitot exit (round 18: a window from the nose kink's former to the nose end, polar
    // 314-345 deg) is closed: the probe leaves the nose through a cut of its own size in the film, bonded round it
    // (holders). The window was there because the battery's front right corner stood 0.71 mm inside the loft, 0.14 mm
    // under the film's chord from 314 to 345 deg; the nose is now stretched NOSE_EXTRA_MM further than the pack moves
    // (round17.mjs), which puts the corner deeper under that chord, and no rod fits between the corner and the probe
    // (the nose tray's flange and base corners leave none clear of both)
    // round 19 (decision 2): the tail pushrod exits (round 18: windows from the 'pushrod exits forward' former to the
    // tail kink's, polar 324-5 and 175-216 deg) are closed: each pushrod runs out through an exit guide (tail-mech.js
    // exitGuide) whose cut in the film is bonded round it (holders), so the film spans those panels again
  ],
  holders: [
    { opening: 'boom posts', parts: '^gearbox plate (aft|forward)$', why: 'the gearbox plates\' posts stand through the top to the boom clamps; the film\'s cut edge is bonded round each post (the aft post lies in the slots\' window)' },
    // round 19: the exit guides hold the film's edge round them (the film carries no load: the guide is held by its lug
    // on the former); each pushrod meets the skin only inside its guide's crossing (through)
    // round 19: the pitot probe holds the film's edge round it ("a hole of its own diameter", round 15's declaration)
    { opening: 'pitot', parts: '^pitot-static probe$', why: 'the film\'s cut edge is closed and bonded round the probe where it leaves the nose (the probe is held by its ties on the nose tray\'s flange; the film carries no load)' },
    { opening: 'tail pushrod guides', parts: '^tail pushrod guide [RL]$', why: 'the film\'s cut edge is closed and bonded round each exit guide where it crosses the skin (the guide is held by its lug on the former; the film carries no load)' },
    { opening: 'tail pushrod exits', through: '^tail pushrod guide [RL]$', why: 'each pushrod meets the skin only inside its exit guide, which runs through the skin with the film closed round it' },
  ],
};

/** The design's pod parameters: POD with params.pod over it. */
export function podParams(d) {
  return { ...POD, ...(d.params?.pod ?? {}) };
}

// ---- the loft --------------------------------------------------------------------------------------------------

/** The loft's half-section at x: { y, hw, hh } (m), or null outside it. */
export const sectionAt = (loft, x) => loftHalf(loft, x);
/** The point on the section's ellipse at polar angle a (rad). */
export const surfacePoint = (h, a) => [h.hw * Math.cos(a), h.y + h.hh * Math.sin(a)];
/** The outward unit normal of the section's ellipse at polar angle a. */
export function surfaceNormal(h, a) {
  const n = [Math.cos(a) / h.hw, Math.sin(a) / h.hh], l = Math.hypot(n[0], n[1]);
  return [n[0] / l, n[1] / l];
}
/** The point `depth` (m) in from the section's ellipse along its normal at polar angle a. */
export function insetPoint(h, a, depth) {
  const p = surfacePoint(h, a), n = surfaceNormal(h, a);
  return [p[0] - depth * n[0], p[1] - depth * n[1]];
}
/** A point's polar angle (deg, 0..360) in a section, as the containment record measures it. */
export const polarOf = (h, p) => norm360(Math.atan2((p[1] - h.y) / h.hh, p[0] / h.hw) / DEG);
/** The containment record's elliptic gap from a point to the section's skin (m, > 0 inside). */
export const loftGap = (h, p) => (h ? (1 - Math.hypot((p[1] - h.y) / h.hh, p[0] / h.hw)) * Math.min(h.hh, h.hw) : -Infinity);

/** Whether polar angle a (deg) lies in the counter-clockwise arc [a0, a1]; `strict` excludes its ends. */
export function inArc(a, [a0, a1], strict = false) {
  const span = norm360(a1 - a0) || 360, t = norm360(a - a0), e = 1e-9;
  return strict ? t > e && t < span - e : t <= span + e || t >= 360 - e;
}

/** The loft's section on the line from section a to b at x (both { x_m, y_m, width_m, height_m }). */
function lerpSection(a, b, x) {
  const f = (a.x_m - x) / (a.x_m - b.x_m);
  return { x_m: x, y_m: a.y_m + f * (b.y_m - a.y_m), width_m: a.width_m + f * (b.width_m - a.width_m), height_m: a.height_m + f * (b.height_m - a.height_m) };
}

/**
 * The loft cut to the pod's structure (round 17): its nose at the end disc (where its first segment is
 * params.pod.nose_end_w_mm wide), and nothing aft of the cone's end (v24's 10 mm tube to the tail enclosed nothing).
 */
export function trimPodLoft(loft0, P = POD) {
  const loft = noseStretch(loft0, P.nose_stretch);
  const s = loft.sections, w = mm(P.nose_end_w_mm);
  if (!(s.length > 2 && s[0].width_m === 0)) throw new Error('trimPodLoft: the loft must start at a nose point');
  const f = w / s[1].width_m, xNose = s[0].x_m - f * (s[0].x_m - s[1].x_m);
  // the cone's end: the last pod section (the tube aft of it is a repeat of the boom's 10 mm section)
  let last = s.length - 1;
  while (last > 1 && s[last].width_m === s[last - 1].width_m && s[last].height_m === s[last - 1].height_m) last--;
  return { ...loft, sections: [lerpSection(s[0], s[1], xNose), ...s.slice(1, last + 1)] };
}

/**
 * Round 18 (the CG): the loft's nose lengthened. Its sections forward of the first full-size one (the ogive) are
 * stretched along x about that one, so that the section at st.x_mm moves st.by_mm forward (each section keeps its
 * size; everything aft of the ogive is unchanged). -> the loft, with stretch: { x_m, e } (the map x -> x0 + (x - x0)(1 + e)).
 */
export function noseStretch(loft, st) {
  if (!st?.by_mm) return loft;
  const s = loft.sections, area = (q) => q.width_m * q.height_m, full = Math.max(...s.map(area));
  const i0 = s.findIndex((q) => area(q) >= full - 1e-12), x0 = s[i0].x_m, e = mm(st.by_mm) / (mm(st.x_mm) - x0);
  if (!(e > 0) || !(mm(st.x_mm) > x0)) throw new Error('noseStretch: the stretch must be forward, from a station in the ogive');
  return { ...loft, sections: s.map((q, i) => (i < i0 ? { ...q, x_m: x0 + (q.x_m - x0) * (1 + e) } : q)), stretch: { x_m: x0, e } };
}

/** A station (mm) on the unstretched loft mapped onto the stretched one (loft.stretch, noseStretch). */
export const stretched = (loft, x_mm) => (loft.stretch && mm(x_mm) > loft.stretch.x_m ? (loft.stretch.x_m + (mm(x_mm) - loft.stretch.x_m) * (1 + loft.stretch.e)) * 1000 : x_mm);

/** The loft's surface area (m2) over x in [x0, x1] and the polar arc `keep(a)` says is covered. */
export function loftArea(loft, x0, x1, keep = () => true, nx = 400, na = 360) {
  let A = 0;
  const dx = (x1 - x0) / nx, da = (2 * Math.PI) / na;
  for (let i = 0; i < nx; i++) {
    const x = x0 + (i + 0.5) * dx, h = sectionAt(loft, x), ha = sectionAt(loft, Math.min(x + 1e-4, x1)), hb = sectionAt(loft, Math.max(x - 1e-4, x0));
    if (!h || !ha || !hb) continue;
    const dw = (ha.hw - hb.hw) / (2e-4), dh = (ha.hh - hb.hh) / (2e-4), dy = (ha.y - hb.y) / (2e-4);
    for (let j = 0; j < na; j++) {
      const a = (j + 0.5) * da;
      if (!keep(x, a / DEG)) continue;
      const Px = [1, dw * Math.cos(a), dy + dh * Math.sin(a)], Pa = [0, -h.hw * Math.sin(a), h.hh * Math.cos(a)];
      const c = [Px[1] * Pa[2] - Px[2] * Pa[1], Px[2] * Pa[0] - Px[0] * Pa[2], Px[0] * Pa[1] - Px[1] * Pa[0]];
      A += Math.hypot(...c) * dx * da;
    }
  }
  return A;
}

// ---- the structure's geometry ------------------------------------------------------------------------------------

/**
 * The windows the film is cut for: params.pod.windows and the hatch's (params.layout.hatch), m and deg. A window
 * stated `between` two formers takes its x from their faces (podGeometry sets it; null here).
 */
export function podWindows(d, P = podParams(d)) {
  const out = P.windows.map((w) => ({ ...w, x: w.x_mm ? w.x_mm.map(mm) : null, polar: w.polar_deg.map(norm360), covered_by: w.covered_by ?? null }));
  const h = d.params?.layout?.hatch;
  if (h) out.push({ name: 'hatch', x: h.x_mm.map(mm), polar: h.polar_deg.map(norm360), openings: [], covered_by: '^hatch', why: 'the access hatch (params.layout.hatch): its own frame, cover and film close it' });
  return out;
}

/**
 * The pod's structure on a design's loft: formers (x faces, kind band | disc, the polar arcs a band covers, halves,
 * ties), longeron runs (each straight between the loft's sections, its pieces), the windows and the sets.
 * -> { loft, P, t, rod, formers, runs, windows }.
 */
export function podGeometry(d, catalog) {
  const P = podParams(d), loft = d.body.loft, sec = loft.sections;
  const t = mm(byId(catalog, P.former).specs.thickness_mm_nominal ?? 1), rod = mm(byId(catalog, P.longeron).specs.od_mm);
  const windows = podWindows(d, P), win = (n) => { const w = windows.find((q) => q.name === n); if (!w) throw new Error(`pod former: no window '${n}'`); return w; };
  const xNose = sec[0].x_m, xTail = sec.at(-1).x_m;
  const placed = P.formers.map((f) => {
    let x;
    if (f.end === 'nose') x = [xNose - t, xNose];
    else if (f.end === 'tail') x = [xTail, xTail + t];
    else if (f.edge) { const w = win(f.edge[0]); x = f.edge[1] === 'fwd' ? [w.x[1], w.x[1] + t] : [w.x[0] - t, w.x[0]]; }
    else if (f.section_near_mm != null) { const xs = mm(stretched(loft, f.section_near_mm)), s = sec.reduce((a, b) => (Math.abs(b.x_m - xs) < Math.abs(a.x_m - xs) ? b : a)); x = [s.x_m - t / 2, s.x_m + t / 2]; }
    else x = f.x_mm.map(mm);
    return { ...f, x };
  });
  // a window between two formers: from the aft one's forward face to the forward one's aft face
  for (const w of windows.filter((q) => q.between)) {
    const [fw, af] = w.between.map((n) => placed.find((f) => f.name === n));
    if (!fw || !af) throw new Error(`pod window ${w.name}: no former ${w.between.join(' / ')}`);
    w.x = [af.x[1], fw.x[0]];
  }
  const formers = placed.map((f) => {
    // the arcs it covers: all round, less every window whose x range holds it
    const cut = windows.filter((w) => w.x[0] <= f.x[0] + 1e-9 && w.x[1] >= f.x[1] - 1e-9);
    if (cut.length > 1) throw new Error(`pod former ${f.name}: more than one window across it`);
    const arcs = cut.length ? [[cut[0].polar[1], cut[0].polar[0]]] : [[0, 360]];
    return { ...f, id: `pod former ${f.name}`, kind: f.end ? 'disc' : 'band', arcs, mid: (f.x[0] + f.x[1]) / 2 };
  });
  const byName = (n) => { const f = formers.find((q) => q.name === n); if (!f) throw new Error(`pod longerons: no former '${n}'`); return f; };
  const runs = [];
  for (const set of P.longerons) {
    const fF = byName(set.from), fA = byName(set.to);
    const xs = fF.kind === 'disc' ? fF.x[1] : fF.mid, xe = fA.kind === 'disc' ? fA.x[0] : fA.mid;
    for (const deg0 of set.deg) {
      const deg = norm360(deg0);
      // the set's span less every window lying across this angle
      let spans = [[xe, xs]];
      for (const w of windows.filter((q) => inArc(deg, q.polar, true))) spans = spans.flatMap(([a, b]) => [[a, Math.min(b, w.x[0])], [Math.max(a, w.x[1]), b]]).filter(([a, b]) => b - a > 1e-6);
      spans.sort((p, q) => q[1] - p[1]).forEach(([a, b], k, all) => {
        const id = `pod longeron ${set.name} ${deg0}${all.length > 1 ? ` ${'abcdef'[k]}` : ''}`;
        const line = runLine(loft, formers, deg * DEG, [a, b]);
        runs.push({ id, set: set.name, deg, a: deg * DEG, x: [a, b], line, pieces: runPieces(loft, formers, line, deg * DEG, [a, b], rod / 2, { ...P, piece_err_mm: set.piece_err_mm ?? P.piece_err_mm }) });
      });
    }
  }
  return { loft, P, t, rod, formers, runs, windows };
}

/**
 * A run's line: the rod rests on the formers under it and runs straight between them (the film pulls it in; only the
 * formers hold it out), so its vertices are the loft's surface at each former's mid-plane under the run, and its ends
 * where the run ends (on the line through the last two vertices). -> [{ x, p: [z, y] }] from aft to forward.
 */
export function runLine(loft, formers, a, [x0, x1]) {
  const xs = [...new Set(formers.filter((f) => f.mid >= x0 - 1e-9 && f.mid <= x1 + 1e-9).map((f) => f.mid))].sort((p, q) => p - q);
  const at = (x) => surfacePoint(sectionAt(loft, Math.min(Math.max(x, loft.sections.at(-1).x_m), loft.sections[0].x_m)), a);
  const v = xs.map((x) => ({ x, p: at(x) }));
  if (v.length < 2) return [{ x: x0, p: at(x0) }, { x: x1, p: at(x1) }];
  const ext = (q0, q1, x) => ({ x, p: q0.p.map((c, k) => c + ((q1.p[k] - c) * (x - q0.x)) / (q1.x - q0.x)) });
  // a run ending on the far face of a former (a window's edge, an end disc) runs on along its last segment to it
  if (x0 < v[0].x - 1e-9) v.unshift(ext(v[0], v[1], x0));
  if (x1 > v.at(-1).x + 1e-9) v.push(ext(v.at(-2), v.at(-1), x1));
  return v;
}

/** The point on a run's line at x (the rod's outside, on its formers). */
export function linePoint(line, x) {
  let k = 0;
  while (k + 2 < line.length && x > line[k + 1].x) k++;
  const [q0, q1] = [line[k], line[k + 1]], f = (x - q0.x) / (q1.x - q0.x);
  return [q0.p[0] + f * (q1.p[0] - q0.p[0]), q0.p[1] + f * (q1.p[1] - q0.p[1])];
}

/**
 * A run's pieces along x: split at the formers' faces, each at most piece_max_mm and short enough that the rod's slope
 * moves it at most piece_err_mm over it. Each piece is a ring of the rod's radius in from its line (along the loft's
 * normal) at the piece's end nearer the axis: inside the line, and so inside the loft, over the whole piece, and at
 * most piece_err_mm in from the true rod at its other end; `L` is the true length of the rod over the piece.
 */
export function runPieces(loft, formers, line, a, [x0, x1], r, P = POD) {
  const cuts = new Set([x0, x1, ...line.map((q) => q.x)]);
  // at a former's faces where the rod slopes (so the piece on the former is drawn at the former's own section; where it
  // does not slope, one ring serves the whole bay)
  const slopes = (x) => { const d0 = linePoint(line, x - 1e-4), d1 = linePoint(line, x + 1e-4); return Math.hypot(d1[0] - d0[0], d1[1] - d0[1]) / 2e-4 > 1e-4; };
  for (const f of formers) for (const xf of f.x) if (xf > x0 + 1e-9 && xf < x1 - 1e-9 && slopes(xf)) cuts.add(xf);
  const xs = [...cuts].filter((x) => x >= x0 - 1e-12 && x <= x1 + 1e-12).sort((p, q) => p - q), out = [];
  const c = (x) => { const h = sectionAt(loft, x), n = surfaceNormal(h, a), p = linePoint(line, x); return [p[0] - r * n[0], p[1] - r * n[1]]; };
  for (let i = 0; i + 1 < xs.length; i++) {
    const [u, v] = [xs[i], xs[i + 1]], slope = Math.hypot(...linePoint(line, v).map((q, k) => q - linePoint(line, u)[k])) / (v - u);
    const n = Math.max(1, Math.ceil((v - u) / Math.min(mm(P.piece_max_mm), slope > 0 ? mm(P.piece_err_mm) / slope : Infinity)));
    for (let k = 0; k < n; k++) {
      const p0 = u + ((v - u) * k) / n, p1 = u + ((v - u) * (k + 1)) / n, c0 = c(p0 + 1e-9), c1 = c(p1 - 1e-9);
      const h0 = sectionAt(loft, p0 + 1e-9), h1 = sectionAt(loft, p1 - 1e-9);
      // at the end with the smaller section: inside it, so inside the larger (the sections nest) over the whole piece
      out.push({ x: [p0, p1], c: h0.hw * h0.hh <= h1.hw * h1.hh ? c0 : c1, L: Math.hypot(p1 - p0, c1[0] - c0[0], c1[1] - c0[1]) });
    }
  }
  return out;
}

/** A former's section: the smaller of the loft's sections across its thickness. */
function formerSection(loft, f) {
  const a = sectionAt(loft, f.x[0] + 1e-9), b = sectionAt(loft, f.x[1] - 1e-9);
  return a.hw * a.hh <= b.hw * b.hh ? a : b;
}

/** The polar angles (deg) a former's outline is drawn through: every `step` round its arcs, its arcs' ends and every rod angle on it. */
function formerAngles(f, runs, step = 15) {
  const out = [];
  for (const [a0, a1] of f.arcs) {
    const span = norm360(a1 - a0) || 360, n = Math.ceil(span / step);
    for (let k = 0; k <= n; k++) out.push(norm360(a0 + (span * k) / n));
    for (const r of runs) if (inArc(r.deg, [a0, a1])) out.push(r.deg);
    for (const e of f.deep?.arc ?? []) if (inArc(e, [a0, a1])) out.push(norm360(e));
  }
  return [...new Set(out.map((v) => +v.toFixed(9)))];
}

/**
 * A former's solids: a band as convex quadrilaterals between the inset curves (the rods' diameter plus gap_mm in from
 * the loft, and band_mm further in), a disc as convex slices from the section's centre to the inset curve. A split
 * former's halves are returned apart (R: -90..90, L: 90..270).
 */
export function formerSolids(loft, f, runs, P, rodD) {
  const h = formerSection(loft, f), o = rodD + mm(P.gap_mm), i = o + mm(f.band_mm ?? P.band_mm);
  const angs = formerAngles(f, runs, f.step_deg ?? P.step_deg);
  const pieces = [];
  for (const arc of f.arcs) {
    const span = norm360(arc[1] - arc[0]) || 360, full = span === 360;
    // the angles on the arc in order from its start (its end last, apart from its start when the arc is not full)
    const off = (a) => { const o = norm360(a - arc[0]); return !full && o < 1e-9 && inArc(a, arc) && Math.abs(norm360(arc[1] - a)) < 1e-9 ? span : o; };
    const as = [...new Set(angs.filter((a) => inArc(a, arc)).map((a) => +off(a).toFixed(9)))].sort((p, q) => p - q).map((o) => norm360(arc[0] + o));
    if (!full && Math.abs(norm360(as.at(-1) - arc[1])) > 1e-9) as.push(arc[1]);
    const list = full ? [...as, as[0]] : as;
    for (let k = 0; k + 1 < list.length; k++) {
      const [p, q] = [list[k] * DEG, list[k + 1] * DEG], midDeg = norm360(list[k] + norm360(list[k + 1] - list[k]) / 2);
      const ii = f.deep && inArc(midDeg, f.deep.arc) ? o + mm(f.deep.band_mm) : i;
      const pts = f.kind === 'disc' ? [[0, h.y], insetPoint(h, p, o), insetPoint(h, q, o)] : [insetPoint(h, p, o), insetPoint(h, q, o), insetPoint(h, q, ii), insetPoint(h, p, ii)];
      pieces.push({ a0: list[k], a1: list[k + 1], sol: S(`${f.kind === 'disc' ? 'slice' : 'band'} ${pieces.length + 1}`, f.x, poly(pts)) });
    }
  }
  if (!f.split) return { [f.id]: pieces.map((q) => q.sol) };
  const mid = (q) => norm360((q.a0 + norm360(q.a1 - q.a0) / 2));
  const half = (arc) => pieces.filter((q) => inArc(mid(q), arc)).map((q) => q.sol);
  return { [`${f.id} R`]: half([270, 90]), [`${f.id} L`]: half([90, 270]) };
}

// ---- the assembly's parts -------------------------------------------------------------------------------------

/**
 * The pod's structure as solids of the assembly (clearance.js assembly, in place of round 16's formers): ax as
 * clearance.js's assembly context ({ d, catalog, af, add, join, assumptions }).
 */
export function podParts(ax) {
  const { d, catalog, add, join, assumptions } = ax;
  const G = podGeometry(d, catalog), { loft, P, rod, formers, runs } = G;
  for (const f of formers) {
    const parts = formerSolids(loft, f, runs, P, rod);
    for (const [id, sols] of Object.entries(parts)) add(id, sols, { bom: 'fuselage formers' });
    if (f.plate) for (const id of Object.keys(parts)) join(id, f.plate.part, 'bonded on the post\'s face');
    if (f.split) join(`${f.id} R`, `${f.id} L`, 'butted to its other half, both on the longerons');
  }
  for (const r of runs) {
    add(r.id, r.pieces.map((q, k) => ({ ...S(`piece ${k + 1}`, q.x, ring(q.c, rod / 2)), tube: { ro: rod / 2, ri: 0, L: q.L, axis: 'x' } })), { bom: 'fuselage longerons' });
    for (const f of formers.filter((q) => q.x[1] > r.x[0] + 1e-9 && q.x[0] < r.x[1] - 1e-9)) {
      // a split former's rods on their half (a rod on the butt, at 90 or 270 deg, on both)
      const ids = f.split ? (Math.abs(Math.cos(r.a)) < 0.05 ? [`${f.id} R`, `${f.id} L`] : [`${f.id} ${Math.cos(r.a) > 0 ? 'R' : 'L'}`]) : [f.id];
      for (const id of ids) join(r.id, id, f.kind === 'disc' ? 'bonded on the end former\'s edge' : 'bonded on the former\'s edge');
    }
  }
  // rods of two sets at the same angle meet end to end on the former between the sets
  for (const r of runs) for (const q of runs) if (r !== q && r.set !== q.set && Math.abs(r.deg - q.deg) < 1e-9 && Math.abs(r.x[0] - q.x[1]) < 1e-9) join(r.id, q.id, 'end to end on the former between the sets');
  assumptions.push(`pod (round 17, params.pod, src/design/pod-structure.js): ${runs.length} longeron runs of ${P.longeron} on the loft (sets ${P.longerons.map((s) => `${s.name} ${s.deg.join('/')} deg`).join('; ')}), on ${formers.length} formers of ${P.former} (bands ${P.band_mm} mm deep under the rods, end discs where the loft closes), ${formers.filter((f) => f.plate).map((f) => f.name).join(' and ')} bonded by their bands on the gearbox plates' boom posts (nothing else holds the pod); windows cut in the film: ${G.windows.map((w) => w.name).join(', ')}; each rod piece drawn at its end with the smaller section (at most ${P.piece_err_mm} mm in from the true rod)`);
  return G;
}

// ---- the BOM -----------------------------------------------------------------------------------------------------

/** The film's area (m2): the loft's surface less every window (podGeometry's), plus the two end discs' faces. */
export function podFilmArea(loft, W) {
  const s = loft.sections;
  const keep = (x, a) => !W.some((w) => x > w.x[0] && x < w.x[1] && inArc(a, w.polar, true));
  const disc = (q) => (Math.PI / 4) * q.width_m * q.height_m;
  return loftArea(loft, s.at(-1).x_m, s[0].x_m, keep) + disc(s[0]) + disc(s.at(-1));
}

/**
 * The pod's BOM lines from its geometry (round 17): its formers (their drawn volume, tabs included, at the plate's
 * density) and longerons (their rods' true length), and the film over the loft less the windows, its adhesive
 * disclosed. carriers: the airframe's tray geometry (the tabs reach its flanges).
 * -> { structure: [{ name, material_id, material, mass_g, basis, source_url, count }], ties, G }.
 */
export function podBom(d, catalog) {
  const G = podGeometry(d, catalog), { loft, P, rod, formers, runs } = G;
  const pl = byId(catalog, P.former), lr = byId(catalog, P.longeron), film = byId(catalog, d.params.parts.film);
  const vol = formers.reduce((a, f) => a + Object.values(formerSolids(loft, f, runs, P, rod)).flat().reduce((v, s) => v + polyArea(s.shape.pts) * (s.x[1] - s.x[0]), 0), 0);
  const nF = formers.reduce((a, f) => a + (f.split ? 2 : 1), 0), Lr = runs.reduce((a, r) => a + r.pieces.reduce((v, q) => v + q.L, 0), 0);
  const Af = podFilmArea(loft, G.windows), rhoF = film.specs.areal_density_g_m2 ?? film.mass_g;
  const structure = [
    { name: `fuselage formers, ${nF} off`, material_id: pl.id, material: pl.name, mass_g: vol * pl.specs.density_g_cm3 * 1e6, basis: `${nF} pieces (${formers.length} formers, ${formers.filter((f) => f.split).length} in halves): ${P.band_mm} mm bands under the longerons (${formers.filter((f) => f.band_mm).map((f) => `${f.name} ${f.band_mm} mm`).join(', ')}), the plate formers' bands bonded on the gearbox plates' posts, end discs where the loft closes; their drawn ${(vol * 1e9).toFixed(0)} mm3 at the datasheet's ${pl.specs.density_g_cm3} g/cm3 (its ${pl.specs.areal_density_kg_m2} kg/m2 gives ${(pl.specs.areal_density_kg_m2 / pl.specs.density_g_cm3 * 100).toFixed(0)}% of this: the heavier figure is used)`, source_url: pl.source_url, count: nF },
    { name: `fuselage longerons, ${runs.length} off`, material_id: lr.id, material: lr.name, mass_g: lr.mass_g * Lr, basis: `${runs.length} runs, ${(Lr * 1000).toFixed(1)} mm of ${lr.id} in all (${lr.mass_g} g/m; each run its rod's true length along the loft)`, source_url: lr.source_url, count: runs.length },
    { name: 'fuselage film shell', material_id: film.id, material: film.name, mass_g: rhoF * Af, basis: `${rhoF} g/m2 x ${Af.toFixed(4)} m2: the loft's surface and its two end faces, less the windows (${G.windows.map((w) => w.name).join(', ')}; the hatch's own film is the hatch's); ORACOVER AIR has no adhesive layer: the maker's hotmelt adhesive is brushed on the structure (no mass published: in the 15% unlisted margin with the other glues)`, source_url: film.source_url },
  ];
  return { structure, G, film_area_m2: Af, rod_length_m: Lr, former_volume_m3: vol };
}

// ---- the M7 members ----------------------------------------------------------------------------------------------

const G0 = 9.80665;
const polyArea = (pts) => Math.abs(pts.reduce((a, p, i) => a + p[0] * pts[(i + 1) % pts.length][1] - pts[(i + 1) % pts.length][0] * p[1], 0)) / 2;
const sect = (name, radial) => ({ name, radial, axial: [] });
const inertial = (m, r) => { const R = Math.hypot(...r); return [{ ch: 'sf_origin', k: +m.toFixed(8) }, { ch: 'wdot', k: +(m * R).toFixed(8) }, { ch: 'w2', k: +(m * R).toFixed(8) }]; };
const scaleT = (terms, f) => terms.map((q) => ({ ch: q.ch, k: +(q.k * f).toFixed(8) }));
/**
 * The pod's drag D as a load term (only the tie check carries a fixed extra force): D / g on the specific-force
 * channel, which bounds D whenever the load factor is at least 1 and overstates it in proportion above that.
 */
const dragTerm = (D) => [{ ch: 'sf_origin', k: +(D / G0).toFixed(8) }];

/** The mass (kg) and centroid (m) of the structure and film over x in [x0, x1] (from the BOM's own geometry). */
function podShare(G, d, catalog, x0, x1) {
  const { loft, P, rod, formers, runs } = G, pl = byId(catalog, P.former), lr = byId(catalog, P.longeron), film = byId(catalog, d.params.parts.film);
  const acc = [0, 0, 0, 0], addm = (m, p) => { acc[0] += m; for (let k = 0; k < 3; k++) acc[k + 1] += m * p[k]; };
  for (const r of runs) for (const q of r.pieces) { const xm = (q.x[0] + q.x[1]) / 2; if (xm >= x0 && xm <= x1) addm((lr.mass_g / 1000) * q.L, [xm, q.c[1], q.c[0]]); }
  for (const f of formers) if (f.mid >= x0 && f.mid <= x1) {
    for (const s of Object.values(formerSolids(loft, f, runs, P, rod)).flat()) {
      const pts = s.shape.pts, c = pts.reduce((a, p) => [a[0] + p[0] / pts.length, a[1] + p[1] / pts.length], [0, 0]);
      addm(polyArea(pts) * (s.x[1] - s.x[0]) * pl.specs.density_g_cm3 * 1000, [f.mid, c[1], c[0]]);
    }
  }
  const s = loft.sections, lo = Math.max(x0, s.at(-1).x_m), hi = Math.min(x1, s[0].x_m), rhoF = (film.specs.areal_density_g_m2 ?? film.mass_g) / 1000;
  if (hi > lo) { const A = loftArea(loft, lo, hi, () => true, 60, 72), h = sectionAt(loft, (lo + hi) / 2); addm(rhoF * A, [(lo + hi) / 2, h.y, 0]); }
  return { m: acc[0], r: acc[0] > 0 ? [acc[1] / acc[0], acc[2] / acc[0], acc[3] / acc[0]] : [0, 0, 0] };
}

/** The rods at a station: their number, their (z, y) group's smaller second moment (m2 per rod area) and its farthest rod. */
function rodGroup(G, x) {
  const h = sectionAt(G.loft, x), rs = G.runs.filter((r) => r.x[0] <= x + 1e-9 && r.x[1] >= x - 1e-9);
  const ps = rs.map((r) => insetPoint(h, r.a, G.rod / 2)), c = ps.reduce((a, p) => [a[0] + p[0] / ps.length, a[1] + p[1] / ps.length], [0, 0]);
  const Iy = ps.reduce((a, p) => a + (p[1] - c[1]) ** 2, 0), Iz = ps.reduce((a, p) => a + (p[0] - c[0]) ** 2, 0);
  return { n: rs.length, sum_r2: Math.min(Iy, Iz), c_max: Math.max(...ps.map((p) => Math.hypot(p[0] - c[0], p[1] - c[1]))) };
}

/**
 * The pod's M7 members (round 17; audit F9: the formers carry the pod's drag at 25 m/s and its inertia): the two plate
 * formers' bonds on the gearbox plates' posts and their bands from the post to the rods, and for each part of the pod
 * that hangs beyond them on its longerons alone (the cone on its first former, the pod aft of the slots, the nose) the
 * rods in compression and their bonds. Every one carries the pod's whole drag (as dragTerm). ax: airframe.js's member
 * context.
 */
export function podMembers(ax) {
  const { d, catalog, members, glueCf, lump } = ax;
  const G = podGeometry(d, catalog), { P, formers, runs, rod, t, loft } = G;
  const rho = d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1.0, drag = 0.5 * rho * P.V_drag_m_s ** 2 * (d.body?.CdA_front_m2 ?? 0.003);
  const pod = lump('fuselage shell and formers'), held = formers.filter((f) => f.plate);
  const pl = byId(catalog, P.former), lr = byId(catalog, P.longeron);
  const plate = { id: pl.id, yield_MPa: pl.specs.tensile_strength_0_90_mpa / 2, basis: `${pl.specs.tensile_strength_0_90_mpa} MPa tensile (0/90) with no compressive figure, not stated as a minimum: counted at half (SPEC round 7, values with no basis)` };
  const rodMat = { id: lr.id, E_GPa: lr.specs.tensile_modulus_lengthways_gpa_range[0], yield_MPa: lr.specs.compressive_strength_lengthways_mpa_range[0] };
  // the pod (structure and film) on its two plate formers as a beam on two supports: its centroid lies outside them,
  // so the nearer former carries more than the whole pod's weight (statics); the drag (along x) all on one former
  const xs = held.map((f) => f.mid).sort((p, q) => p - q), xc = pod.r[0], span = xs.at(-1) - xs[0];
  const kSup = span > 0 ? Math.max(Math.abs(xs.at(-1) - xc), Math.abs(xc - xs[0])) / span : 1;
  const share = [...scaleT(inertial(pod.m, pod.r), kSup), ...scaleT(dragTerm(drag), 1)];
  const w = Math.min(...held.map((f) => f.plate.w_mm)), bandL = Math.min(...held.map((f) => f.deep?.band_mm ?? f.band_mm ?? P.band_mm)), bandT = Math.min(...held.map((f) => f.band_mm ?? P.band_mm));
  members.push({ name: 'pod former bonds on the gearbox posts', kind: 'bond', check: 'lap', material: glueCf, dims: { w_mm: w, L_mm: bandL }, sections: [sect('F', share)], uses: { F: 'F' }, note: `Araldite 2011, carbon band on 6061 at its published CFRP figure (typical, half): each plate former's band flat on its gearbox plate's boom post (${w} mm wide, the band ${bandL} mm deep: the record measures the drawn overlap); the pod (${(pod.m * 1000).toFixed(1)} g, centroid x ${(xc * 1000).toFixed(1)} mm) on the formers ${held.map((f) => f.name).join(' and ')} (x ${xs.map((v) => (v * 1000).toFixed(1)).join(', ')} mm) as a beam on two supports: the more loaded one carries ${kSup.toFixed(2)} x its inertia, and one the whole drag at ${P.V_drag_m_s} m/s (${drag.toFixed(2)} N, as dragTerm); nothing else holds the pod to the frame` });
  // the band, in its plane, from the post's edge to the nearest rod on that former (deep round the post), and beyond the
  // deep arc's ends to the nearest rod (at its own depth)
  const arcTo = (f, from) => { const h = sectionAt(loft, f.mid); return Math.min(...runs.filter((r) => r.x[0] <= f.mid + 1e-9 && r.x[1] >= f.mid - 1e-9).map((r) => Math.min(...from.map((e) => { const g = norm360(r.deg - e); return Math.min(g, 360 - g); })))) * DEG * Math.max(h.hw, h.hh); };
  const edgeOf = (f) => { const e = Math.acos(Math.min(1, mm(f.plate.w_mm) / 2 / sectionAt(loft, f.mid).hw)) / DEG; return [e, 180 - e]; }; // the post's edges on the band, near 90 deg
  const arm = Math.max(...held.map((f) => Math.max(arcTo(f, edgeOf(f)), mm(f.plate.w_mm) / 2))); // at least half the post
  members.push({ name: 'pod former bands at the posts', kind: 'plate', check: 'plate', material: plate, dims: { b_mm: bandL, t_mm: t * 1000 }, sections: [sect('M', scaleT(share, arm))], uses: { M: 'M' }, note: `the plate former's load bending its ${bandL} x ${t * 1000} mm band (deep round the post) in its plane over the ${(arm * 1000).toFixed(1)} mm from the post's edge to the nearest longeron` });
  const armT = Math.max(...held.map((f) => Math.max(arcTo(f, f.deep?.arc ?? edgeOf(f)), 0.001)));
  members.push({ name: 'pod former bands beside the posts', kind: 'plate', check: 'plate', material: plate, dims: { b_mm: bandT, t_mm: t * 1000 }, sections: [sect('M', scaleT(share, armT))], uses: { M: 'M' }, note: `the same load on the ${bandT} x ${t * 1000} mm band beyond the deep arc, over the ${(armT * 1000).toFixed(1)} mm (at least 1 mm) from its end to the nearest longeron` });
  cantileverMembers({ d, catalog, members, glueCf, G, held, drag, rodMat, lr });
  return { G, drag };
}

/**
 * The members of each part of the pod that hangs beyond its two plate formers on its longerons alone (the cone on its
 * first former, the pod aft of the slots on the rods under the slots' window, the nose): the most loaded rod in
 * compression over the longest bay on the span, and its bond on the support former.
 */
function cantileverMembers({ d, catalog, members, glueCf, G, held, drag, rodMat, lr }) {
  const { formers, runs, rod, t } = G;
  // what hangs beyond the plate formers on the longerons: the cone on its first former (its rods end there), the pod
  // aft of the slots (on the five rods under the slots' window), the nose forward of the forward plate's former
  const by = (n) => formers.find((f) => f.name === n), win = G.windows.find((w0) => w0.name === 'wing roots and horn slots');
  const fwdPlate = held.reduce((a, f) => (f.mid > a.mid ? f : a)), aftPlate = held.reduce((a, f) => (f.mid < a.mid ? f : a)), cone = by('cone start');
  const spans = [
    { nm: 'the cone', f: cone, x: [-Infinity, cone.mid], xg: cone.mid - 1e-4, bays: [-Infinity, cone.mid], bondOnSpan: true },
    { nm: 'the pod aft of the slots', f: aftPlate, x: [-Infinity, aftPlate.x[0]], xg: win ? (win.x[0] + win.x[1]) / 2 : aftPlate.x[0] - 1e-4, bays: [cone.mid, aftPlate.mid] },
    { nm: 'the nose', f: fwdPlate, x: [fwdPlate.x[1], Infinity], xg: fwdPlate.mid + 1e-4, bays: [fwdPlate.mid, Infinity] },
  ];
  for (const { nm, f, x, xg, bays, bondOnSpan } of spans) {
    const sh = podShare(G, d, catalog, x[0], x[1]), lever = Math.abs(sh.r[0] - f.mid), load = inertial(sh.m, sh.r);
    // the rods in compression: the most loaded rod of the thinnest group on the span (its share and the moment over it)
    const g = rodGroup(G, xg), kRod = 1 / g.n + (lever * g.c_max) / g.sum_r2;
    const st = formers.map((q) => q.mid).filter((v) => v >= bays[0] - 1e-9 && v <= bays[1] + 1e-9).sort((p, q) => p - q);
    const Lb = Math.max(...st.slice(1).map((v, k) => v - st[k]));
    members.push({ name: `pod longerons carrying ${nm}`, kind: 'strut', check: 'strut', material: rodMat, dims: { L_mm: +(Lb * 1000).toFixed(1), d_mm: rod * 1000, di_mm: 0 }, sections: [sect('F', [...scaleT(load, kRod), ...scaleT(dragTerm(drag), 1 / g.n)])], uses: { F: 'F' }, note: `${nm} (${(sh.m * 1000).toFixed(1)} g at x ${(sh.r[0] * 1000).toFixed(0)} mm, ${(lever * 1000).toFixed(0)} mm from the former ${f.name}) on the ${g.n} rods at x ${(xg * 1000).toFixed(1)} mm: the most loaded rod's share and the moment over the group, and its share of the whole pod's drag, pinned over the longest bay on the span (${(Lb * 1000).toFixed(1)} mm); ${lr.id} at the lower of its published modulus (${rodMat.E_GPa} GPa) and compressive strength (${rodMat.yield_MPa} MPa)` });
    // the bonds at the support former: a rod ending there passes its whole force through its bond (over half the
    // former), the group being the span's own rods where they all end on it (the cone's), else every rod at the former
    const gf = bondOnSpan ? g : rodGroup(G, f.mid), kB = 1 / gf.n + (lever * gf.c_max) / gf.sum_r2, ends = runs.some((r) => Math.abs(r.x[0] - f.mid) < 1e-6 || Math.abs(r.x[1] - f.mid) < 1e-6 || Math.abs(r.x[0] - f.x[0]) < 1e-6 || Math.abs(r.x[1] - f.x[1]) < 1e-6);
    members.push({ name: `pod longeron bonds carrying ${nm}`, kind: 'bond', check: 'lap', material: glueCf, dims: { w_mm: rod * 1000, L_mm: ((ends ? 0.5 : 1) * t * 1000) }, sections: [sect('F', [...scaleT(load, kB), ...scaleT(dragTerm(drag), 1 / gf.n)])], uses: { F: 'F' }, note: `Araldite 2011 on CFRP (typical, half): the most loaded of the ${gf.n} rods at the former ${f.name} passing its whole force through its bond, over the rod's width and ${ends ? 'half the former\'s thickness (a run ending on it rests on half)' : 'the former\'s thickness'}` });
  }
}

// ---- the interface rules, the sequence and the containment declarations -------------------------------------------

const BOND_RULE = "the adhesive maker's published lap-shear strength on that substrate (SPEC Rulings after physics round 7, 'Every part reaches the airframe through a rated path')";
const TIE_RULE = "the cable tie maker's published loop/tensile strength (SPEC round 15 carriers; src/design/airframe.js tieFor)";
const spec = (names, by, rule) => ({ by, rating: () => names, rule });
const isText = (txt) => ({ test: (s) => s === txt });
const bonds = (d) => (d.structure.members ?? []).filter((m) => /^pod longeron bonds carrying /.test(m.name)).map((m) => m.name);
const struts = (d) => (d.structure.members ?? []).filter((m) => /^pod longerons carrying /.test(m.name)).map((m) => m.name);
/** The pod's rows for the interface ledger (src/design/interfaces.js format), ahead of INTERFACE_RULES. */
export const POD_INTERFACE_RULES = [
  { why: /^bonded on the (end )?former's edge$/, part: /^pod longeron /, kind: 'bonded', load: { by: 'the rod\'s bond on the former (Araldite 2011 on CFRP)', rating: bonds, rule: BOND_RULE }, axial: { by: 'the same bond', rating: bonds, rule: BOND_RULE }, note: 'a longeron on the edge of the former under it; the most loaded bonds (at the two stations where the cone and the nose hang on the rods) rate every one' },
  { why: isText('end to end on the former between the sets'), part: /^pod longeron /, kind: 'bonded', load: { by: 'each rod\'s own bond on the former they meet on', rating: bonds, rule: BOND_RULE }, axial: { by: 'the same bonds', rating: bonds, rule: BOND_RULE }, note: 'two sets\' rods at one angle meet on a former; each is bonded to it' },
  { why: isText('butted to its other half, both on the longerons'), part: /^pod former /, kind: 'bonded', load: { by: 'the longerons bonded across both halves', rating: (d) => [...bonds(d), ...struts(d)], rule: BOND_RULE }, axial: { by: 'the same bonds', rating: bonds, rule: BOND_RULE }, note: 'a former in halves (it is trapped between the gearbox plates\' posts): each half on its own rods' },
  { why: isText('bonded on the post\'s face'), part: /^pod former /, kind: 'bonded', load: spec(['pod former bonds on the gearbox posts', 'pod former bands at the posts', 'pod former bands beside the posts'], 'the band\'s bond on the gearbox plate\'s post, and the band to it', BOND_RULE), axial: spec(['pod former bonds on the gearbox posts'], 'the same bond', BOND_RULE), note: 'the pod\'s only fixing to the frame: a plate former\'s band bonded flat on the gearbox plate\'s boom post' },
];

const X = (s) => [s, 0, 0], Z = (s) => [0, 0, s];
/**
 * Each set's rods at one angle as one step, onto their formers from outside along their own outward direction (along
 * x, a rod would sweep its inner face through every former it does not rest on in that place).
 */
export function podLongeronSteps(P = POD) {
  return P.longerons.flatMap((set) => set.deg.map((deg) => step(`pod longerons, ${set.name} ${deg} deg`, [`^pod longeron ${set.name} ${deg}( [a-f])?$`], [0, +Math.sin(deg * DEG).toFixed(9), +Math.cos(deg * DEG).toFixed(9)], 0.05, 'from outside onto their formers')));
}
const step = (name, members, dir, dist_m, why) => ({ name, members, dir, dist_m, ...(why ? { why } : {}) });
/**
 * The pod's assembly steps (round17.mjs sequence17): in place of round 16's formers, after the packed parts and the
 * probe: the formers from the nose and from the tail (the one trapped between the plates' posts in halves from the
 * sides), their ties, the rods of each set along x, then the covering. The film has no solids: its step is stated
 * and matches none (sequence.js lists it under `empty`).
 */
export const POD_SEQUENCE = [
  { replace: 'former 2', steps: [
    step('pod formers from the nose', ['^pod former (forward plate|nose 94|nose 135|hatch aft|nose 176|nose kink|hatch forward)$'], X(1), 0.35, 'the rings together along x from forward, over the nose\'s smaller sections; the forward plate\'s former onto its post'),
    step('pod formers from the tail', ['^pod former (slots aft|mid aft|cone start|pushrod exits forward|tail kink)$'], X(-1), 0.45, 'the rings together along x from aft'),
    step('pod former aft plate R', ['^pod former aft plate R$'], Z(1), 0.08, 'the half from the right (the ring is trapped between the gearbox plates\' posts), its band onto the aft plate\'s post'),
    step('pod former aft plate L', ['^pod former aft plate L$'], Z(-1), 0.08, 'the half from the left'),
  ] },
  // round 18: the rods and end discs after round 16's 'former 1' (so a step inserted after 'former 2', the tail
  // pushrods fed in through their exits, comes between the rings and the rods)
  { replace: 'former 1', steps: [] },
  { after: 'former 1', steps: [
    ...podLongeronSteps(POD),
    step('pod nose end former', ['^pod former nose end$'], X(1), 0.1, 'onto the nose rods\' ends'),
    step('pod tail end former', ['^pod former tail end$'], X(-1), 0.1, 'onto the cone rods\' ends'),
  ] },
  { replace: 'former 3', steps: [] }, { replace: 'former 4', steps: [] }, { replace: 'former 5', steps: [] },
  ...[1, 2, 3, 4, 5].map((k) => ({ replace: `former ${k} tie`, steps: [] })),
  { after: 'former 1', steps: [{ name: 'pod covering: ORACOVER AIR LIGHT over the formers, longerons and end discs (the hotmelt brushed on first), cut at the windows', members: ['^pod covering$'], threaded: true, why: 'the film has no solids; it goes on after every part it covers and before the wings go through the slots\' window' }] },
];

/** Containment declarations: the longerons carry the film (their outside on the loft: at least zero, not 0.5 mm). */
export const POD_CONTAINMENT = {
  on_skin: [{ re: '^pod longeron ', why: 'the longerons carry the covering: each rod\'s outside on the loft (touching it)' }],
};

// ---- the covering record ------------------------------------------------------------------------------------------

const outline = (sh, n = 48) => {
  if (sh.kind === 'ring') return Array.from({ length: n }, (_, k) => [sh.c[0] + sh.ro * Math.cos((2 * Math.PI * k) / n), sh.c[1] + sh.ro * Math.sin((2 * Math.PI * k) / n)]);
  if (sh.kind === 'cap') return [sh.a, sh.b].flatMap((c) => Array.from({ length: n }, (_, k) => [c[0] + sh.r * Math.cos((2 * Math.PI * k) / n), c[1] + sh.r * Math.sin((2 * Math.PI * k) / n)]));
  return sh.pts.flatMap((a, i) => { const b = sh.pts[(i + 1) % sh.pts.length]; return Array.from({ length: 4 }, (_, k) => [a[0] + ((b[0] - a[0]) * k) / 4, a[1] + ((b[1] - a[1]) * k) / 4]); });
};

/** The bays between the formers' mid-planes, each with the rods over it and its film panels and windows. */
export function podBays(G) {
  const xs = G.formers.map((f) => f.mid).sort((p, q) => q - p), out = [];
  for (let k = 0; k + 1 < xs.length; k++) {
    const [x1, x0] = [xs[k], xs[k + 1]];
    const fF = G.formers.find((f) => f.mid === x1), fA = G.formers.find((f) => f.mid === x0);
    const rods = G.runs.filter((r) => r.x[0] <= x0 + 1e-9 && r.x[1] >= x1 - 1e-9).sort((p, q) => p.deg - q.deg);
    const wins = G.windows.filter((w) => w.x[0] <= fA.x[1] + 1e-9 && w.x[1] >= fF.x[0] - 1e-9);
    const panels = rods.map((r, i) => {
      const q = rods[(i + 1) % rods.length], arc = [r.deg, q.deg], w = wins.find((v) => inArc(norm360(r.deg + norm360(q.deg - r.deg) / 2), v.polar, true));
      return { a: r, b: q, arc, span_deg: norm360(q.deg - r.deg) || 360, window: w ?? null };
    });
    out.push({ x: [x0, x1], fwd: fF, aft: fA, rods, panels, windows: wins });
  }
  return out;
}

/** The catenoid's neck (fraction of R in from the rings) for rings of radius 1 at half-separation h, or null past the bound. */
export function catenoidNeck(h) {
  if (h >= CATENOID_2H_OVER_R / 2) return null;
  // a cosh(h / a) = 1 on the outer (stable) branch: a from 1 down
  let lo = 0.5524, hi = 1;
  for (let i = 0; i < 80; i++) { const a = (lo + hi) / 2; if (a * Math.cosh(h / a) > 1) hi = a; else lo = a; }
  return 1 - (lo + hi) / 2;
}

/** A rib part's span station: its section's centre in (z, y) at pose q (m) and its chord (its x range). */
function ribStation(part, q) {
  const s = part.solids.find((v) => v.name === 'rib') ?? part.solids[0], sh = s.at ? s.at(q) : s.shape;
  const pts = sh.kind === 'poly' ? sh.pts : [sh.c ?? sh.a];
  return { id: part.id, c: pts.reduce((a, p) => [a[0] + p[0] / pts.length, a[1] + p[1] / pts.length], [0, 0]), chord: s.x[1] - s.x[0] };
}

/**
 * The covering record (round 17, SPEC "The covering spans only what is supported"; audit item 4): every surface the
 * loft draws has structure under it, and every film span stays within its stated rule.
 *  - Pod: the loft ends where the structure does (an end disc at each end); every bay between formers is held either
 *    as panels between longerons or, with no longerons, as a tube between two rings that must satisfy the catenoid's
 *    bound (2h/R < 1.3255, R the smaller ring's smaller semi-axis); each panel's film lies on the chord between its two
 *    rods at every x, and every solid inside the pod clears that chord by the containment's 0.5 mm; every window is
 *    framed (its x edges on formers, its polar edges on longerons) and every declared opening's crossing lies in a
 *    window or on a part that holds the film's edge; the hatch's window is closed by parts whose id starts 'hatch';
 *    every rod bend within its strain allowance; every run's ends on a former.
 *  - Wing and tail: their film panels between the assembly's ribs (and the edge rods along them): the film's load on
 *    each rib bond by Myers' rule against the film's published adhesive power.
 * opts: { required (m), phases, n_design, tail_CL } -> { rule, rows, failed, assumptions, passed }.
 */
export function coveringRecord(asm, d, catalog, opts = {}) {
  const G = podGeometry(d, catalog), byPart = new Map(asm.parts.map((p) => [p.id, p]));
  // the rules' figures from their catalogue references (parts-v25/covering.json); a missing one fails its row
  const src = coveringSources(catalog);
  const cx = {
    G, asm, d, byPart, src, bound: src.catenoid ?? NaN, required: opts.required ?? 5e-4, bays: podBays(G),
    poses: posesOf(d, opts.lay, opts.phases ?? 12), decl: d.params.layout?.containment ?? {},
    drawn: (f) => (f.split ? [`${f.id} R`, `${f.id} L`] : [f.id]).every((id) => byPart.has(id)), runDrawn: (r) => byPart.has(r.id),
  };
  const clear = filmClearance(asm, d, G, cx.bays, cx.poses, cx.required, cx.decl);
  const rows = [
    ...Object.entries(COVERING_SOURCES).map(([k, id]) => ({ kind: 'rule source', what: k, id, ok: src[k] != null })),
    ...loftEndRows(cx), ...cx.bays.flatMap((b) => bayRows(cx, b)), ...G.windows.map((w) => windowRow(cx, w)), ...openingRows(cx),
    ...clear.worst.map((c) => ({ kind: 'film clearance', ...c, ok: c.clear_mm >= cx.required * 1000 - 1e-6 })),
    ...rodRows(cx, byId(catalog, G.P.longeron)), ...postBondRows(cx), ...sealRows(cx),
    ...ribPanels(asm, d, cx.poses[0], { ...opts, adhesive_N_per_m: src.adhesive }),
  ];
  const failed = rows.filter((r) => !r.ok).map(rowLabel);
  return {
    rule: 'SPEC Design freeze, Function completeness, "The covering spans only what is supported" (2026-11-03): every surface the loft draws has structure under it; the pod film between rings within the catenoid bound 2h/R < 1.3255 (stated assumption, MathWorld), between longerons on the chord, clear of every enclosed part by 0.5 mm; every cut framed; the wing and tail films\' load on their ribs by Myers (Model Aviation 1992) against the film\'s published adhesive power; src/design/pod-structure.js',
    rows, failed, assumptions: coveringAssumptions(byId(catalog, G.P.longeron)), film_clearance_min_mm: clear.min_mm, open: openDrag(G, openDragSources(catalog), sealLeaks(asm)), passed: failed.length === 0,
  };
}

/** The drag coefficient on an open window's area when no cavity-drag source is in the catalogue: the whole flow over
 * it stopped (an upper bound; stated). */
export const CD_OPEN = 1.0;

/** The catalogue's references for the open windows' drag (round 18): the cavity and the internal through-flow. */
export const OPEN_DRAG_SOURCES = { cavity: 'drag-ref-gharib-roshko-1985-axisymmetric-cavity', through: 'drag-ref-rogallo-naca-713-internal-flow' };

/**
 * The open-window drag figures from the catalogue (null where an entry or figure is missing): the cavity's Cd on the
 * opening's area (Gharib, Roshko & Sarohia 1985, the wake-mode ceiling converted to opening area: the top of the
 * published low-speed range), and the through-flow's drag on the openings the interior draws air in through (Rogallo,
 * NACA Report 713 Fig. 10: C_Di 0.85 on the hole's area at internal p/q -0.95, a conservative reading for an interior
 * set by the wing-root window; the Q/AV <= 1 bound, 2.0, if that reading is missing).
 */
export function openDragSources(catalog) {
  const at = (id) => (catalog ?? []).find((q) => q.id === id)?.specs ?? null;
  const r = at(OPEN_DRAG_SOURCES.through), c = at(OPEN_DRAG_SOURCES.cavity)?.recommended_cd_open_conservative, t = r?.through_flow_cd_conservative ?? r?.through_flow_cda_over_area_max;
  return { cavity: Number.isFinite(c) ? c : null, through: Number.isFinite(t) ? t : null };
}

/**
 * The windows left open (no part closes them: not the hatch's) and their drag. Each one's area on the loft:
 *  - as a cavity, at the cavity source's Cd on the opening's area (CD_OPEN, an upper bound, if no source is given);
 *  - the pod's interior joins them, so air can pass through: the interior sits at the largest window's pressure (it has
 *    about 4x the others' area), so the others draw air in; their drag is the through-flow source's coefficient on their
 *    area (the smaller side: the largest window against all the others). Summed with the cavity term: conservative.
 *  - round 19: a sealed exit's leak (leaks: [{ name, area_m2 }], the annulus an exit guide leaves round its pushrod) is a
 *    path of its own for the through-flow (on the smaller side with the windows), not a cavity (it has no depth to
 *    trap a vortex: the flow passes along the annulus). The cooling inlet and outlet are not here: the build-needs
 *    record counts them, at Cd 1.0 on their area (above this term's C_Di).
 * -> { windows: [{ name, area_m2 }], leaks, Cd, cavity_CdA_m2, through_CdA_m2, CdA_m2, basis }.
 */
export function openDrag(G, src = {}, leaks = []) {
  const windows = G.windows.filter((w) => !w.covered_by).map((w) => ({ name: w.name, area_m2: +loftArea(G.loft, w.x[0], w.x[1], (x, a) => inArc(a, w.polar, true)).toFixed(6) }));
  const paths = [...windows, ...leaks], A = windows.reduce((s, w) => s + w.area_m2, 0), Ap = paths.reduce((s, w) => s + w.area_m2, 0), Amax = Math.max(0, ...paths.map((w) => w.area_m2));
  const Cd = src.cavity ?? CD_OPEN, cav = Cd * A, thr = src.through != null ? src.through * Math.min(Amax, Ap - Amax) : 0;
  const lk = leaks.length ? `; the seals' leaks (${leaks.map((l) => `${l.name} ${(l.area_m2 * 1e6).toFixed(2)} mm2`).join(', ')}) as through-flow paths on the smaller side, not cavities` : '';
  const basis = src.cavity != null
    ? `each open window as a cavity at Cd ${Cd} on its opening's area (${OPEN_DRAG_SOURCES.cavity}: the top of the published low-speed range, a conservative ceiling), plus the through-flow the pod's interior allows between them at C_Di ${src.through ?? 0} on the smaller side's area (${OPEN_DRAG_SOURCES.through} Fig. 10: the inlet holes' drag at internal p/q -0.95, conservative against the body-alone -0.055 at the window, drag-ref-freeman-naca-443-akron-hull-cp, since the window may also feel the wing root's suction)${lk}. Not in the containment record's frontal area (a cut has no material)`
    : `the open windows' area on the loft at Cd ${CD_OPEN}: an upper bound (stated assumption: the flow over a cut is taken as wholly stopped; no cavity-drag source in the catalogue). Not in the containment record's frontal area (a cut has no material)`;
  return { windows, leaks, Cd, cavity_CdA_m2: +cav.toFixed(6), through_CdA_m2: +thr.toFixed(6), CdA_m2: +(cav + thr).toFixed(6), basis };
}

/** A failed row's label: its kind and what it is about. */
const rowLabel = (r) => `${r.kind}: ${r.name ?? r.solid ?? r.run ?? r.what ?? (r.x_mm ? r.x_mm.join('..') : '')}${r.polar_deg ? ` @ ${[].concat(r.polar_deg).join('..')}` : ''}`;

/** The loft's two ends, each on its drawn end disc (the drawn disc's outer face at the loft's end section). */
function loftEndRows({ G, byPart }) {
  const sec = G.loft.sections, face = (f, k) => { const xs = (byPart.get(f?.id)?.solids ?? []).map((q) => q.x[k]); return xs.length ? (k ? Math.max(...xs) : Math.min(...xs)) : null; };
  const nose = G.formers.find((f) => f.end === 'nose'), tail = G.formers.find((f) => f.end === 'tail'), xn = face(nose, 1), xt = face(tail, 0);
  return [
    { kind: 'loft end', what: 'nose', loft_x_mm: r3(sec[0].x_m), structure_x_mm: xn == null ? null : r3(xn), drawn: xn != null, ok: xn != null && Math.abs(xn - sec[0].x_m) < 1e-9 && sec[0].width_m > 0 },
    { kind: 'loft end', what: 'tail', loft_x_mm: r3(sec.at(-1).x_m), structure_x_mm: xt == null ? null : r3(xt), drawn: xt != null, ok: xt != null && Math.abs(xt - sec.at(-1).x_m) < 1e-9 },
  ];
}

/** A bay's row (held as panels between longerons, or else a ring span within the catenoid's bound) and its panels' rows. */
function bayRows({ G, bound, drawn, runDrawn }, b) {
  const loft = G.loft, L = b.x[1] - b.x[0], hs = [sectionAt(loft, b.x[1] - 1e-9), sectionAt(loft, b.x[0] + 1e-9)], R = Math.min(...hs.map((h) => Math.min(h.hw, h.hh)));
  const rods = b.rods.filter(runDrawn), formersOk = drawn(b.fwd) && drawn(b.aft);
  const ratio = L / R, held = rods.length >= 3 && b.panels.every((p) => p.window || p.span_deg < 180), neck = catenoidNeck(ratio / 2);
  const bay = { kind: 'bay', x_mm: b.x.map(r3), between: [b.fwd.name, b.aft.name], span_mm: r3(L), R_mm: r3(R), two_h_over_R: +ratio.toFixed(4), bound, longerons: rods.length, held_by: held ? 'panels between longerons' : 'rings only', ...(held ? {} : { catenoid_neck_mm: neck == null ? null : r3(neck * R) }), formers_drawn: formersOk, ok: formersOk && (held || ratio < bound) && rods.length === b.rods.length };
  const panels = b.panels.filter((p) => !p.window).map((p) => {
    const mid = norm360(p.a.deg + p.span_deg / 2), sag = Math.max(...[0.02, 0.25, 0.5, 0.75, 0.98].map((f) => { const x = b.x[0] + f * L; return -chordDepth(linePoint(p.a.line, x), linePoint(p.b.line, x), surfacePoint(sectionAt(loft, x), mid * DEG)); }));
    return { kind: 'panel', x_mm: b.x.map(r3), polar_deg: p.arc, span_deg: +p.span_deg.toFixed(2), longerons: [p.a.id, p.b.id], max_sag_mm: r3(sag), ok: p.span_deg < 180 && runDrawn(p.a) && runDrawn(p.b) };
  });
  return [bay, ...panels];
}

/** A window: framed (its x edges on drawn formers, its polar edges on drawn longerons) and covered where it says so. */
function windowRow({ G, asm, bays, drawn, runDrawn }, w) {
  const edgesX = w.x.map((x, k) => G.formers.some((f) => Math.abs(f.x[k === 0 ? 1 : 0] - x) < 1e-9 && drawn(f)));
  const inBays = bays.filter((b) => b.windows.includes(w)), edgesP = w.polar.map((a) => inBays.length > 0 && inBays.every((b) => b.rods.some((r) => Math.abs(norm360(r.deg - a)) < 1e-6 && runDrawn(r))));
  const cover = w.covered_by ? asm.parts.filter((pt) => new RegExp(w.covered_by).test(pt.id) && pt.solids.some((s) => s.x[1] > w.x[0] && s.x[0] < w.x[1])).map((pt) => pt.id) : null;
  return { kind: 'window', name: w.name, x_mm: w.x.map(r3), polar_deg: w.polar, framed_x: edgesX, framed_polar: edgesP, ...(w.covered_by ? { covered_by: cover } : { open: true }), why: w.why, ok: edgesX.every(Boolean) && edgesP.every(Boolean) && (!w.covered_by || cover.length > 0) };
}

/**
 * Each declared opening's solids: its crossing of the skin in a window, on a part that holds the film's edge, or (a
 * holder stating `through`) inside the crossing of a part that runs through the skin and holds the film's edge round it
 * (round 19: a pushrod inside its exit guide's crossing, in x and polar angle).
 */
function openingRows({ G, byPart, poses, required, decl }) {
  const rows = [], cache = new Map();
  const crossOf = (pid) => {
    if (!cache.has(pid)) cache.set(pid, unionCrossing((byPart.get(pid)?.solids ?? []).map((s) => crossing(G.loft, s, poses, required)).filter(Boolean)));
    return cache.get(pid);
  };
  for (const o of decl.openings ?? []) {
    const holder = G.P.holders.find((h) => h.opening === o.name);
    for (const id of o.solids ?? [o.part]) {
      const [pid, sname] = id.includes('/') ? id.split('/') : [id, null];
      for (const s of (byPart.get(pid)?.solids ?? []).filter((v) => !sname || v.name === sname)) {
        const c = crossing(G.loft, s, poses, required);
        if (!c) continue;
        const where = openingIn(G, holder, pid, c, [...byPart.keys()], crossOf);
        rows.push({ kind: 'opening', opening: o.name, solid: `${pid}/${s.name}`, x_mm: c.x.map(r3), polar_deg: c.polar.map((v) => +v.toFixed(1)), in: where, ok: where != null });
      }
    }
  }
  return rows;
}

/** Where a crossing c of part pid meets the film: a window, the part holding its edge, or another part it runs through. */
function openingIn(G, holder, pid, c, ids, crossOf) {
  const within = (g) => !!g && c.x[0] >= g.x[0] - 1e-9 && c.x[1] <= g.x[1] + 1e-9 && arcWithin(c.polar, g.polar);
  const inWin = G.windows.find((w) => within({ x: w.x, polar: w.polar }));
  if (inWin) return `window: ${inWin.name}`;
  if (holder?.parts && new RegExp(holder.parts).test(pid)) return `held by the part (${holder.why})`;
  const via = holder?.through ? ids.filter((q) => q !== pid && new RegExp(holder.through).test(q)).find((q) => within(crossOf(q))) : null;
  return via ? `through ${via} (${holder.why})` : null;
}

/** The skin crossings of a part's solids as one: their x range and the shortest polar arc holding them all, or null. */
export function unionCrossing(cs) {
  if (!cs.length) return null;
  const as = cs.flatMap((c) => { const span = norm360(c.polar[1] - c.polar[0]); return Array.from({ length: 9 }, (_, k) => norm360(c.polar[0] + (span * k) / 8)); });
  const srt = [...as].sort((p, q) => p - q);
  let gap = -1, at = 0;
  for (let i = 0; i < srt.length; i++) { const g = norm360(srt[(i + 1) % srt.length] - srt[i]); if (g > gap) { gap = g; at = i; } }
  return { x: [Math.min(...cs.map((c) => c.x[0])), Math.max(...cs.map((c) => c.x[1]))], polar: [srt[(at + 1) % srt.length], srt[at]] };
}

/**
 * Round 19: each exit seal (a part stating `seal`: an exit guide) as the covering record checks it: the window it
 * replaces is not open, its leak (the annulus) stated, the pushrod clear of its bore over the commanded travel (the
 * smallest radial clearance against the rod's largest excursion), and its lug's face on the former's band over at least
 * the area its bond member rates (the drawn overlap, sampled at 0.05 mm).
 */
function sealRows({ G, asm, byPart }) {
  return asm.parts.filter((p) => p.seal).map((p) => {
    const s = p.seal, lug = byPart.get(s.lug), fm = byPart.get(s.former);
    const face = lug && fm ? fm.solids.filter((q) => Math.abs(q.x[0] - lug.solids[0].x[1]) < 1e-9 && q.shape.kind === 'poly') : [];
    const overlap = face.length ? overlapArea(face.map((q) => q.shape), lug.solids[0].shape) : 0;
    const open = G.windows.some((w) => w.name === s.replaces);
    return { kind: 'exit seal', name: p.id, replaces: s.replaces, window_open: open, leak_mm2: +(s.leak_m2 * 1e6).toFixed(2), leak_basis: s.leak_basis, clearance_mm: r3(s.clearance_m), excursion_mm: r3(s.excursion_m), travel_basis: s.travel_basis, lug_overlap_mm2: +(overlap * 1e6).toFixed(2), lug_bond_rated_mm2: s.lug_bond_mm2, ok: !open && s.leak_m2 > 0 && s.excursion_m < s.clearance_m && overlap * 1e6 >= s.lug_bond_mm2 };
  });
}

/** The leaks the seals leave (each sealing part's stated `seal.leak_m2`), for the through-flow term. */
export const sealLeaks = (asm) => asm.parts.filter((p) => p.seal && p.seal.leak_m2 > 0).map((p) => ({ name: `${p.id} annulus`, area_m2: +p.seal.leak_m2.toFixed(9) }));

/**
 * Each plate former's bond on its gearbox plate's post as drawn: the former's face on the post's face, and the band's
 * overlap with the post (sampled at 0.05 mm) at least the bond the member rates (the post's width by the band's depth;
 * a former in halves shares it).
 */
function postBondRows({ G, byPart }) {
  return G.formers.filter((f) => f.plate).flatMap((f) => {
    const post = byPart.get(f.plate.part)?.solids.find((q) => q.name === f.plate.post), ids = f.split ? [`${f.id} R`, `${f.id} L`] : [f.id];
    const want = (mm(f.plate.w_mm) * mm(f.deep?.band_mm ?? f.band_mm ?? G.P.band_mm)) / ids.length;
    return ids.map((id) => {
      const bands = byPart.get(id)?.solids.filter((q) => /^band/.test(q.name)) ?? [];
      // the drawn band's face on the drawn post's face
      const touch = !!post && bands.length > 0 && bands.every((q) => Math.abs(post.x[1] - q.x[0]) < 1e-9 || Math.abs(post.x[0] - q.x[1]) < 1e-9);
      const area = post && touch ? overlapArea(bands.map((q) => q.shape), post.shape) : 0;
      return { kind: 'post bond', name: id, post: post ? `${f.plate.part}/${f.plate.post}` : null, faces_touch: touch, overlap_mm2: +(area * 1e6).toFixed(2), rated_mm2: +(want * 1e6).toFixed(2), ok: touch && area >= want * 0.95 };
    });
  });
}

/** The area (m2) of the union of shapes `a` inside shape b, sampled on a 0.05 mm grid over b's box. */
function overlapArea(a, b, st = 5e-5) {
  const pts = b.pts, z = pts.map((q) => q[0]), y = pts.map((q) => q[1]);
  let n = 0;
  for (let zz = Math.min(...z) + st / 2; zz < Math.max(...z); zz += st) for (let yy = Math.min(...y) + st / 2; yy < Math.max(...y); yy += st) if (inPoly(pts, [zz, yy]) && a.some((sh) => inPoly(sh.pts, [zz, yy]))) n++;
  return n * st * st;
}

/** Whether point p lies inside the polygon pts (even-odd). */
function inPoly(pts, [z, y]) {
  let inside = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    const [zi, yi] = pts[i], [zj, yj] = pts[j];
    if ((yi > y) !== (yj > y) && z < ((zj - zi) * (y - yi)) / (yj - yi) + zi) inside = !inside;
  }
  return inside;
}

/** The longerons' bends (strain against the rod's allowance) and ends (each on a drawn former). */
function rodRows({ G, drawn, runDrawn }, lr) {
  const Emax = lr.specs.tensile_modulus_lengthways_gpa_range.at(-1), sc = lr.specs.compressive_strength_lengthways_mpa_range[0], epsAllow = sc / (Emax * 1000) / 1.5;
  return G.runs.flatMap((r) => {
    const bends = runKinks(G.loft, r).map((k) => ({ kind: 'longeron bend', run: r.id, x_mm: r3(k.x), angle_deg: +(k.angle / DEG).toFixed(3), radius_mm: r3(k.radius), strain: +(G.rod / 2 / k.radius).toExponential(3), allowable: +epsAllow.toExponential(3), ok: G.rod / 2 / k.radius <= epsAllow }));
    const ends = r.x.map((x) => G.formers.find((f) => x >= f.x[0] - 1e-9 && x <= f.x[1] + 1e-9));
    return [...bends, { kind: 'longeron ends', run: r.id, x_mm: r.x.map(r3), on: ends.map((f) => f?.name ?? null), ok: ends.every((f) => f && drawn(f)) && runDrawn(r) }];
  });
}

/** The record's stated assumptions. */
function coveringAssumptions(lr) {
  const Emax = lr.specs.tensile_modulus_lengthways_gpa_range.at(-1), sc = lr.specs.compressive_strength_lengthways_mpa_range[0];
  return [
    `pod film: a membrane under uniform tension with no pressure difference (the soap-film model of covering-ref-catenoid-critical-ratio, MathWorld, Minimal Surface of Revolution, Eq. 26: a film between two coaxial rings exists only for 2h/R < ${CATENOID_2H_OVER_R}); a stated assumption, not a covering maker's rule (none publishes a span: SOURCES-covering.md). Elliptic or unequal rings: R is the smaller semi-axis of the smaller ring. Shrink film's non-isotropic stress and the flight's pressure difference are not modelled`,
    'pod panels: between two longerons the film is taken to lie on the straight chord between them at every x (a plane is the zero-pressure film between two straight edges; the formers under the chord only lift it, and are not credited); every solid inside the pod must clear that chord by the containment\'s 0.5 mm',
    `longeron bends: a rod is straight between the formers it rests on and bends through each, on the circle through that former and its neighbours; its strain against the lower published compressive strength over the upper published modulus (${sc} MPa / ${Emax} GPa) at FoS 1.5`,
    'windows: the film is cut round each one and its edge bonded to the formers and longerons framing it; a fixed part crossing the skin (a gearbox post, round 19 a pushrod\'s exit guide) holds the film\'s edge bonded round it',
    'round 19, the exit guides: the film\'s edge bonded round each guide (the maker\'s hotmelt on the woven carbon) carries no load: the guide is held by its lug\'s two rated bonds (tail-mech.js guideMembers), and the film\'s pull on it is its unpublished shrink tension, as at every framed edge; it seals, and the annulus between the guide\'s bore and the pushrod is the exit\'s only leak (counted in the open-window drag\'s through-flow)',
    'the film\'s shrink tension is not published: the longerons\' bow between formers under it is not quantified',
  ];
}

/**
 * How far M lies on the section's centre side of the chord from A to B (m), A to B counter-clockwise round the section
 * (its inside on the left): > 0 inside the chord, < 0 between the chord and the arc.
 */
export function chordDepth(A, B, M) {
  const u = [B[0] - A[0], B[1] - A[1]], w = [M[0] - A[0], M[1] - A[1]];
  return (u[0] * w[1] - u[1] * w[0]) / Math.hypot(u[0], u[1]);
}

/** Whether the counter-clockwise arc c lies within the counter-clockwise arc w (deg). */
export function arcWithin(c, w) {
  const span = norm360(w[1] - w[0]) || 360, o0 = norm360(c[0] - w[0]), o1 = o0 + (norm360(c[1] - c[0]));
  return o0 <= span + 1e-9 && o1 <= span + 1e-9;
}

/** The drive poses the record samples (crank phases at each amplitude; the drive's layout, joints.js driveLayout). */
function posesOf(d, lay, phases) {
  if (!lay || !d.drive) return [null];
  return [d.drive.ampMin, d.drive.ampNominal, d.drive.ampMax].filter(Number.isFinite).flatMap((amp) => Array.from({ length: phases }, (_, i) => drivePose({ law: lay.law, fourBar: lay.fb }, (i / phases) * 2 * Math.PI, amp)));
}

/**
 * A solid's crossing of the skin: x and polar range (deg, the short way) where it comes within `required` of it, or
 * null. Its outline meets the skin where an edge of it (between two outline points) reaches into the band within
 * `required` of the skin on either side, both its ends counted; where it passes through the skin between two of its x
 * samples (wholly inside the band at one, wholly outside at the next), both samples' outlines count. A solid wholly
 * inside by more than `required`, or wholly outside by more (a pushrod run out bare past the skin), does not meet the
 * film there.
 */
export function crossing(loft, s, poses, required) {
  const shapes = s.at ? poses.filter(Boolean).map((q) => s.at(q)) : [s.shape];
  let x0 = Infinity, x1 = -Infinity; const as = [];
  const take = (x, angs) => { x0 = Math.min(x0, x); x1 = Math.max(x1, x); as.push(...angs); };
  let prev = null;
  for (let k = 0; k <= 8; k++) {
    const x = s.x[0] + ((s.x[1] - s.x[0]) * k) / 8, h = sectionAt(loft, x);
    if (!h) { prev = null; continue; }
    const cur = { x, angs: [], inside: true, outside: true };
    for (const sh of shapes) {
      const pts = outline(sh, 24), g = pts.map((p) => loftGap(h, p));
      pts.forEach((p, i) => {
        const j = (i + 1) % pts.length;
        cur.angs.push(polarOf(h, p));
        if (g[i] < required) cur.inside = false;
        if (g[i] > -required) cur.outside = false;
        if (Math.min(g[i], g[j]) < required && Math.max(g[i], g[j]) > -required) take(x, [polarOf(h, p), polarOf(h, pts[j])]);
      });
    }
    if (prev && ((prev.inside && cur.outside) || (prev.outside && cur.inside))) { take(prev.x, prev.angs); take(x, cur.angs); }
    prev = cur;
  }
  if (!as.length) return null;
  // the polar range as the shortest arc holding every angle
  const srt = [...as].sort((p, q) => p - q);
  let gap = -1, at = 0;
  for (let i = 0; i < srt.length; i++) { const g = norm360(srt[(i + 1) % srt.length] - srt[i]); if (g > gap) { gap = g; at = i; } }
  return { x: [x0, x1], polar: [srt[(at + 1) % srt.length], srt[at]] };
}

/** The film's clearance over every panel: each enclosed solid's points at x inside the pod against the panel's chord. */
function filmClearance(asm, d, G, bays, poses, required, decl) {
  const { loft } = G, outsideRe = (decl.outside ?? []).map((o) => (o.re ? new RegExp(o.re) : null)), outsideId = new Set((decl.outside ?? []).filter((o) => !o.re).map((o) => o.part ?? o.name));
  const own = /^pod (longeron|former) /, skip = (pid) => own.test(pid) && !/ tie [RL]$/.test(pid) || outsideId.has(pid) || outsideRe.some((re) => re?.test(pid));
  const worst = new Map(); let min = Infinity;
  for (const part of asm.parts) {
    if (skip(part.id)) continue;
    for (const s of part.solids) {
      if (declaredOpening(decl.openings, part.id, s.name)) continue;
      const shapes = s.at ? poses.filter(Boolean).map((q) => s.at(q)) : [s.shape];
      for (let k = 0; k <= 6; k++) {
        const x = s.x[0] + ((s.x[1] - s.x[0]) * k) / 6, h = sectionAt(loft, x), b = bays.find((q) => x >= q.x[0] - 1e-12 && x <= q.x[1] + 1e-12);
        if (!h || !b || b.panels.length < 3) continue;
        for (const sh of shapes) for (const p of outline(sh, 32)) {
          if (loftGap(h, p) < 0) continue; // outside the loft: the containment record's
          const a = polarOf(h, p), pn = b.panels.find((q) => inArc(a, q.arc));
          if (!pn || pn.window) continue;
          const A = linePoint(pn.a.line, x), B = linePoint(pn.b.line, x), c = chordDepth(A, B, p);
          min = Math.min(min, c);
          const key = `${b.x[0]}|${pn.arc.join(',')}`;
          if (!worst.has(key) || worst.get(key).c > c) worst.set(key, { c, solid: `${part.id}/${s.name}`, x, arc: pn.arc, bay: b.x });
        }
      }
    }
  }
  return { min_mm: Number.isFinite(min) ? r3(min) : null, worst: [...worst.values()].sort((p, q) => p.c - q.c).filter((w, k) => k < 12 || w.c < required).map((w) => ({ solid: w.solid, x_mm: r3(w.x), polar_deg: w.arc, bay_mm: w.bay.map(r3), clear_mm: r3(w.c) })) };
}

/** A run's bends at its interior vertices (the formers it passes over): { x, angle, radius } (the circle through the three supports). */
export function runKinks(loft, r) {
  const out = [], L = r.line;
  for (let k = 1; k + 1 < L.length; k++) {
    const u = [L[k].x - L[k - 1].x, L[k].p[0] - L[k - 1].p[0], L[k].p[1] - L[k - 1].p[1]], v = [L[k + 1].x - L[k].x, L[k + 1].p[0] - L[k].p[0], L[k + 1].p[1] - L[k].p[1]];
    const lu = Math.hypot(...u), lv = Math.hypot(...v), ang = Math.acos(Math.max(-1, Math.min(1, u.reduce((s0, q, i) => s0 + q * v[i], 0) / (lu * lv))));
    if (lu < 1e-6 || lv < 1e-6) continue;
    // the rod's radius through its three supports: the circle through them (chord from the one before to the one after)
    const chord = Math.hypot(L[k + 1].x - L[k - 1].x, L[k + 1].p[0] - L[k - 1].p[0], L[k + 1].p[1] - L[k - 1].p[1]);
    out.push({ x: L[k].x, angle: ang, radius: ang > 1e-12 ? chord / (2 * Math.sin(ang)) : Infinity });
  }
  return out;
}

/**
 * The wing's and tail's film panels between the assembly's ribs, per side, at pose q (any rib layout another module
 * draws: parts 'rib R|L k' and 'tail rib R|L k'). Myers (covering-ref-myers-1992-rib-suction): peak suction on the
 * covering = 2 x the surface's loading x n x 1.5; each rib bond carries half of each neighbouring panel, p s / 2 per
 * unit length, against the film's published adhesive power (ORACOVER 2010: "Adhesive power: min. 7 N / 25 mm").
 */
function ribPanels(asm, d, q, opts) {
  const rows = [], WL = d.geometry?.wing_loading_N_m2, n = opts.n_design ?? 5, rho = d.atmosphere?.rho_kg_m3 ?? d.atmosphere?.rho ?? 1.0;
  const qTail = 0.5 * rho * (podParams(d).V_drag_m_s) ** 2 * (opts.tail_CL ?? 1.0);
  const adh = opts.adhesive_N_per_m ?? NaN; // the ORACOVER 2010 product information's adhesive power (coveringSources)
  const groups = [['wing', /^rib ([RL]) (\d+)$/, 2 * (WL ?? NaN) * n * 1.5, `2 x wing loading ${WL?.toFixed(2)} N/m2 x n ${n} x 1.5`], ['tail', /^tail rib ([RL]) (\d+)$/, 2 * qTail * 1.5, `2 x q ${qTail.toFixed(0)} Pa at ${podParams(d).V_drag_m_s} m/s x CL ${opts.tail_CL ?? 1.0} x 1.5 (the tail's own loading, stated)`]];
  for (const [surf, re, p, basis] of groups) for (const side of ['R', 'L']) {
    const ribs = asm.parts.filter((pt) => re.test(pt.id) && pt.id.match(re)[1] === side).map((pt) => ribStation(pt, q));
    if (ribs.length < 2) { rows.push({ kind: 'rib panels', surface: `${surf} ${side}`, ribs: ribs.length, ok: false, why: 'fewer than two ribs: no film panel can be held' }); continue; }
    // order along the span from the root (nearest the body's centre line)
    ribs.sort((a, b) => Math.hypot(...a.c) - Math.hypot(...b.c));
    let worst = null;
    for (let k = 0; k + 1 < ribs.length; k++) {
      const s = Math.hypot(ribs[k + 1].c[0] - ribs[k].c[0], ribs[k + 1].c[1] - ribs[k].c[1]), load = (p * s) / 2;
      if (!worst || load > worst.load) worst = { s, load, between: [ribs[k].id, ribs[k + 1].id] };
    }
    rows.push({ kind: 'rib panels', surface: `${surf} ${side}`, ribs: ribs.length, widest_mm: r3(worst.s), between: worst.between, peak_suction_Pa: +p.toFixed(1), suction_basis: basis, bond_load_N_per_m: +worst.load.toFixed(2), adhesive_power_N_per_m: adh, ratio: +(adh / worst.load).toFixed(2), ok: Number.isFinite(worst.load) && worst.load <= adh });
  }
  return rows;
}
