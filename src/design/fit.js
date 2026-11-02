// Round 4: the body sized to hold the parts. The packing check is the film workstream's (sims/avis-shared/model.js
// prepare, layoutParts, fitReport), used as it is: this file only chooses the pod's size and centre line, and moves the
// design's mass lumps to where the packing put the real parts, so drag, shell mass, CG and trim all follow.
import { prepare, partsIndex, leftRightAsymmetry } from '../../sims/avis-shared/model.js';
import { buildDesign } from './build.js';

/** Fit issues that are real protrusions (mm > 0.5); slides alone are not issues, they move the CG. */
export function protrusions(fit) {
  return fit.issues.filter((i) => i.mm > 0.5);
}

/** Packing of a built design: { model, issues, cgShift_m }. */
export function packDesign(design, catalog) {
  const model = prepare(design, partsIndex(design, catalog));
  return { model, issues: protrusions(model.fit), cgShift_m: model.fit.cgShift_m };
}

/** Lump positions from a packing: each lump at the mass-weighted centre of the parts the packing placed for it. */
export function lumpPositionsFrom(model) {
  const L = model.layout.parts;
  const centre = (names) => {
    const ps = L.filter((p) => names.includes(p.name));
    if (!ps.length) return null;
    const c = [0, 0, 0];
    for (const p of ps) { c[0] += p.box.cx / ps.length; c[1] += p.box.cy / ps.length; c[2] += p.box.cz / ps.length; }
    return c.map((v) => +v.toFixed(4));
  };
  const out = {
    'flap motor': centre(['motor']),
    ESC: centre(['esc']),
    'flight controller': centre(['fc']),
    GPS: centre(['gps']),
    'receiver and airspeed': centre(['receiver', 'airspeed']),
    'tail servos': centre(['servo_tail_L', 'servo_tail_R']),
    'stroke servo': centre(['servo_stroke']),
  };
  // Gears and their bearings: the mass-weighted centre of the gears at their planes and shafts.
  const t = model.train;
  if (t?.links?.length) {
    let m = 0;
    const c = [0, 0, 0];
    for (let j = 0; j < t.links.length; j++) {
      const si = t.n - 1 - j;
      const st = t.stages[si];
      const x = t.planes.stage[si];
      for (const [g, sh] of [[st.wheel, t.shafts[j]], [st.pinion, t.shafts[j + 1]]]) {
        const gm = g.mass_g ?? 0;
        m += gm; c[0] += gm * x; c[1] += gm * sh[1]; c[2] += gm * sh[0];
      }
    }
    if (m > 0) out['gears and bearings'] = c.map((v) => +(v / m).toFixed(4));
    out['crank, rockers, plates'] = [+t.planes.crankArm.toFixed(4), +model.fourBar.O2[1].toFixed(4), 0];
  }
  for (const k of Object.keys(out)) if (!out[k]) delete out[k];
  return out;
}

/**
 * The smallest pod (frontal area W x H) over a grid that holds every part, with its centre line lowered as needed,
 * and the lumps moved to the packed positions (two passes, so the battery is re-placed for the CG target with the
 * other lumps where they really are). Returns { params, packing, tried } or { failed, best }.
 */
export function fitBody(params, catalog, { widths = [0.06, 0.07, 0.08, 0.09, 0.1, 0.11], heights = [0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14], ys = [0, -0.01, -0.02, -0.03, -0.04] } = {}) {
  const cands = [];
  for (const W of widths) for (const H of heights) for (const Y of ys) if (H / 2 + Y >= 0.03) cands.push({ W, H, Y, area: W * H });
  cands.sort((a, b) => a.area - b.area || b.Y - a.Y);
  let tried = 0;
  let best = null;
  for (const c of cands) {
    let p = { ...params, bodyWidth_m: c.W, bodyHeight_m: c.H, bodyY_m: c.Y };
    let pack = null;
    for (let pass = 0; pass < 3; pass++) {
      const d = buildDesign(p, catalog);
      pack = packDesign(d, catalog);
      tried++;
      p = { ...p, lumpPos: { ...(p.lumpPos ?? {}), ...lumpPositionsFrom(pack.model) } };
      if (pack.issues.length) break;
    }
    const worst = Math.max(0, ...pack.issues.map((i) => i.mm));
    if (!best || worst < best.worst) best = { c, worst, issues: pack.issues };
    if (!pack.issues.length) {
      const d = buildDesign(p, catalog);
      const finalPack = packDesign(d, catalog);
      if (!finalPack.issues.length) return { params: p, packing: finalPack, body: c, tried };
    }
  }
  return { failed: 'no pod in the grid holds every part', best, tried };
}

/** Left-right stroke difference (rad) at the drive's minimum, nominal and maximum amplitudes, one crank pin. */
export function strokeAsymmetry(design, catalog) {
  const model = prepare(design, partsIndex(design, catalog));
  const d = design.drive;
  return { min: leftRightAsymmetry(model, d.ampMin), nominal: leftRightAsymmetry(model, d.ampNominal), max: leftRightAsymmetry(model, d.ampMax) };
}
