// The assembly order (SPEC "Completeness and assembly order", 2026-11-03): a bird that cannot be put together is not
// built. The design states its sequence (params.layout.sequence): each step one part or a sub-assembly put together
// first away from the bird, with the straight line it moves along to reach its place (a unit direction in the body
// frame, from its place outward, and how far out it starts). This module checks that every step arrives without
// passing through anything already placed: touching is allowed, and the pairs the clearance check exempts (a key in
// its keyway, a shank in its hole, gears in mesh, a rod through its rib, a tube in another's bore) may meet.
//  - The drive is held at the sequence's pose (its phase and amplitude) throughout.
//  - A step along x is checked exactly: a prism moving along x sweeps its own section over the union of its x ranges.
//  - A step across x is sampled by conservative advancement: from each position the step advances by the gap to the
//    nearest placed solid (a gap that is a true distance cannot close faster than the step moves), and never by more
//    than STEP_MAX_M nor less than STEP_MIN_M, so no wall of 0.5 mm can be jumped.
//  - A cable tie is a flexible band threaded through its slots and closed in place, not a rigid loop: a step marked
//    threaded does not slide; it must come after every part it binds (so there is something to thread round) and its
//    closed solids are the clearance check's to clear. A tie that can slide on along its band's width does so.
//  - Every solid of the assembly is in exactly one step.
import { gap2, gap3, EXEMPT_JOINTS } from './clearance.js';
import { drivePose } from '../../sims/avis-shared/model.js';

/** The largest and smallest advance across x (m): a quarter and a fifth of the 0.5 mm clearance. */
export const STEP_MAX_M = 1.25e-4, STEP_MIN_M = 1e-4;
/** Overlap counted as passing through (m): below it, rounding. */
export const TOL_M = 1e-7;

const keyOf = (a, b) => (a < b ? `${a}|${b}` : `${b}|${a}`);

/** A section moved by [dz, dy] in its plane. */
export function shiftShape(sh, d) {
  const m = (p) => [p[0] + d[0], p[1] + d[1]];
  if (sh.kind === 'ring') return { ...sh, c: m(sh.c) };
  if (sh.kind === 'cap') return { ...sh, a: m(sh.a), b: m(sh.b) };
  return { ...sh, pts: sh.pts.map(m), holes: (sh.holes ?? []).map((h) => ({ ...h, c: m(h.c) })) };
}

/** A section's (z, y) bounding box: [[z0, y0], [z1, y1]]. */
function bbox(sh) {
  if (sh.kind === 'ring') return [[sh.c[0] - sh.ro, sh.c[1] - sh.ro], [sh.c[0] + sh.ro, sh.c[1] + sh.ro]];
  if (sh.kind === 'cap') return [[Math.min(sh.a[0], sh.b[0]) - sh.r, Math.min(sh.a[1], sh.b[1]) - sh.r], [Math.max(sh.a[0], sh.b[0]) + sh.r, Math.max(sh.a[1], sh.b[1]) + sh.r]];
  const z = sh.pts.map((p) => p[0]), y = sh.pts.map((p) => p[1]);
  return [[Math.min(...z), Math.min(...y)], [Math.max(...z), Math.max(...y)]];
}

/**
 * The steps of a stated sequence resolved against an assembly: each member pattern (a regular expression on part ids,
 * or 'part/solid' for one solid) picks the parts or solids it names, in order, each solid taken by the first step that
 * names it. -> { steps: [{ name, dir, dist, solids: [ids] }], missing: [ids in no step], empty: [step names] }.
 */
export function resolveSteps(asm, seq) {
  const all = asm.parts.flatMap((p) => p.solids.map((s) => `${p.id}/${s.name}`));
  const taken = new Set(), steps = [], empty = [];
  for (const st of seq.steps) {
    const res = st.members.map((m) => new RegExp(m));
    const ids = all.filter((id) => !taken.has(id) && res.some((re) => re.test(id.split('/')[0]) || re.test(id)));
    ids.forEach((id) => taken.add(id));
    if (!ids.length) { empty.push(st.name); continue; }
    if (st.threaded) { steps.push({ name: st.name, threaded: true, solids: ids, ...(st.why ? { why: st.why } : {}) }); continue; }
    const n = Math.hypot(...st.dir);
    steps.push({ name: st.name, dir: st.dir.map((v) => v / n), dist: st.dist_m ?? 0.08, solids: ids, ...(st.why ? { why: st.why } : {}) });
  }
  return { steps, missing: all.filter((id) => !taken.has(id)), empty };
}

/**
 * The slide of one moving solid (x range and section at the hold pose) against one placed solid, along unit dir over
 * dist (it starts dist out along dir and moves to its place). -> the smallest gap met on the way (m) and where (0 at
 * its place, 1 at its start).
 */
export function slideGap(m, t, dir, dist) {
  if (Math.abs(dir[1]) < 1e-12 && Math.abs(dir[2]) < 1e-12) {
    // along x: the section is unchanged, the x range sweeps
    const dx = dir[0] * dist, xs = [Math.min(m.x[0], m.x[0] + dx), Math.max(m.x[1], m.x[1] + dx)];
    const g2 = gap2(m.sec, t.sec), g = gap3(xs, t.x, g2), home = gap3(m.x, t.x, g2);
    return g < home ? { gap: g, at: null } : { gap: home, at: 0 };
  }
  let f = 0, worst = Infinity, at = 0;
  const L = dist;
  while (f <= 1 + 1e-12) {
    const o = [dir[0] * f * L, dir[1] * f * L, dir[2] * f * L];
    const g = gap3([m.x[0] + o[0], m.x[1] + o[0]], t.x, gap2(shiftShape(m.sec, [o[2], o[1]]), t.sec));
    if (g < worst) { worst = g; at = f; }
    if (g < -TOL_M) break;
    f += Math.min(STEP_MAX_M, Math.max(STEP_MIN_M, 0.9 * g)) / L;
  }
  return { gap: worst, at };
}

/** Whether two solids may meet: one part's, or a pair the clearance check exempts (EXEMPT_JOINTS). */
function exemptOf(asm) {
  const joins = new Map(asm.joins.map(([a, b, w]) => [keyOf(a, b), w]));
  return (a, b) => {
    if (a.part === b.part) return true;
    const w = joins.get(keyOf(a.id, b.id)) ?? joins.get(keyOf(a.id, b.part)) ?? joins.get(keyOf(a.part, b.id)) ?? joins.get(keyOf(a.part, b.part));
    return EXEMPT_JOINTS.includes(w);
  };
}

/** A threaded step: it binds the parts it is joined to, which must all be placed already. */
function threadedRow(asm, st, moving, placedParts) {
  const mine = new Set(moving.map((m) => m.part));
  const binds = [...new Set(asm.joins.flatMap(([a, b]) => { const [pa, pb] = [a.split('/')[0], b.split('/')[0]]; return mine.has(pa) && !mine.has(pb) ? [pb] : mine.has(pb) && !mine.has(pa) ? [pa] : []; }))];
  const absent = binds.filter((p) => !placedParts.has(p));
  return { name: st.name, threaded: true, parts: [...mine], solids: moving.length, binds, ...(absent.length ? { binds_not_yet_placed: absent } : {}), ...(st.why ? { why: st.why } : {}), failed: absent.length > 0 };
}

/** A sliding step: every moving solid against every placed solid it could meet (its swept box overlapping). */
function slideRow(st, moving, placed, exempt) {
  let worst = Infinity;
  const through = [];
  for (const m of moving) {
    const bm = bbox(m.sec), dz = st.dir[2] * st.dist, dy = st.dir[1] * st.dist, dx = st.dir[0] * st.dist;
    const sw = [[Math.min(bm[0][0], bm[0][0] + dz), Math.min(bm[0][1], bm[0][1] + dy)], [Math.max(bm[1][0], bm[1][0] + dz), Math.max(bm[1][1], bm[1][1] + dy)]];
    const sx = [Math.min(m.x[0], m.x[0] + dx), Math.max(m.x[1], m.x[1] + dx)];
    for (const t of placed) {
      if (exempt(m, t)) continue;
      const bt = bbox(t.sec);
      // a placed solid wholly off the moving solid's swept box is skipped
      if (sw[1][0] < bt[0][0] - 0.001 || bt[1][0] < sw[0][0] - 0.001 || sw[1][1] < bt[0][1] - 0.001 || bt[1][1] < sw[0][1] - 0.001 || sx[1] < t.x[0] - 0.001 || t.x[1] < sx[0] - 0.001) continue;
      const g = slideGap(m, t, st.dir, st.dist);
      worst = Math.min(worst, g.gap);
      if (g.gap < -TOL_M) through.push({ moving: m.id, placed: t.id, by_mm: +(-g.gap * 1000).toFixed(3), ...(g.at != null ? { at_frac: +g.at.toFixed(3) } : {}) });
    }
  }
  const parts = [...new Set(moving.map((m) => m.part))];
  return { name: st.name, dir: st.dir.map((v) => +v.toFixed(4)), dist_mm: +(st.dist * 1000).toFixed(1), parts, solids: moving.length, worst_gap_mm: Number.isFinite(worst) ? +(worst * 1000).toFixed(3) : null, ...(through.length ? { through: through.slice(0, 12), n_through: through.length } : {}), ...(st.why ? { why: st.why } : {}), failed: through.length > 0 };
}

/**
 * The sequence's check on an assembly (asm: clearance.js assembly with the packed parts) at the drive pose the
 * sequence holds. -> { rule, pose, steps: [{ name, dir, dist_mm, parts, solids, worst_gap_mm, through }], missing,
 * empty, failures, passed }.
 */
export function sequenceRecord(asm, d, lay) {
  const seq = d.params.layout.sequence;
  const pose = seq.pose ?? { phase_rad: 0, amp: d.drive.ampNominal };
  const moves = asm.parts.some((p) => p.solids.some((s) => s.at));
  const q = moves ? drivePose({ law: lay.law, fourBar: lay.fb }, pose.phase_rad, pose.amp ?? d.drive.ampNominal) : null;
  const byId = new Map(asm.parts.flatMap((p) => p.solids.map((s) => [`${p.id}/${s.name}`, { part: p.id, id: `${p.id}/${s.name}`, x: s.x, sec: s.at ? s.at(q) : s.shape }])));
  const exempt = exemptOf(asm);
  const { steps, missing, empty } = resolveSteps(asm, seq);
  const placed = [], placedParts = new Set(), rows = [];
  for (const st of steps) {
    const moving = st.solids.map((id) => byId.get(id));
    const { failed, ...r } = st.threaded ? threadedRow(asm, st, moving, placedParts) : slideRow(st, moving, placed, exempt);
    placed.push(...moving);
    moving.forEach((m) => placedParts.add(m.part));
    rows.push({ ...r, ...(failed ? { failed: true } : {}) });
  }
  const failures = rows.filter((r) => r.failed).map((r) => r.name);
  return {
    rule: 'SPEC Completeness and assembly order (2026-11-03): every step arrives along its straight line without passing through anything already placed (touching allowed; the pairs the clearance check exempts may meet); a threaded tie comes after every part it binds; every solid in exactly one step; a step along x checked exactly by its swept x range, a step across x by conservative advancement between 0.1 and 0.125 mm; src/design/sequence.js',
    pose: { phase_rad: pose.phase_rad, amp_rad: pose.amp ?? d.drive.ampNominal, ...(seq.pose?.why ? { why: seq.pose.why } : {}) },
    steps: rows,
    missing,
    empty,
    failures,
    passed: failures.length === 0 && missing.length === 0,
  };
}
