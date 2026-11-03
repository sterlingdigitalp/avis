// Round 9: the gear train's ratings under KHK's stated conditions (SPEC ruling after physics round 8), from the
// catalogue parts, the drive layout (gear planes between the plates) and the design's stated running conditions
// (p.gearRating: the rating crank frequency, the lubricant on each kind of mesh, the gearbox temperature limit and
// whether the load reverses). src/physics/gears.js holds the corrections; this file only gathers their inputs.
import { jgmaGearRating, lewisDuraconRating, pitchLineSpeed } from '../physics/gears.js';

const byId = (catalog, id) => catalog.find((p) => p.id === id);
const isDuracon = (part) => /duracon/i.test(part.specs?.material ?? '');

/**
 * Support class of a gear at plane x between bearings at xA and xB (KHK Table 10.11). KHK states no criterion for its
 * "rugged" and "weak" shaft columns, so a gear that is not at mid-span (within tol of the span) is taken as on a weak
 * shaft, the more conservative column; outside the span it is overhung.
 */
export function supportClass(x, xA, xB, tol = 0.05) {
  const lo = Math.min(xA, xB), hi = Math.max(xA, xB);
  if (x < lo || x > hi) return 'overhung';
  return Math.abs((x - lo) / (hi - lo) - 0.5) <= tol ? 'equidistant' : 'weak';
}

/** Each stage's pinion and wheel speeds (rpm) from the crank's rpm: stage k's wheel turns at the crank's times the later stages' ratios. */
export function stageSpeeds(stages, crankRpm) {
  return stages.map((s, k) => {
    const wheel = crankRpm * stages.slice(k + 1).reduce((a, q) => a * q.ratio, 1);
    return { wheel_rpm: wheel, pinion_rpm: wheel * s.ratio };
  });
}

/**
 * Re-rate every stage (structure.gear_stages) under the design's conditions. Returns new stage objects whose
 * pinion_allowable_Nm and gear_allowable_Nm are the corrected values, with the catalogue values and every condition
 * check kept beside them (rating_check).
 */
export function rateStages(stages, catalog, layout, gr) {
  const crankRpm = 60 * gr.crank_Hz;
  const sp = stageSpeeds(stages, crankRpm);
  const planes = layout?.planes_m ?? {};
  const xA = planes.plateAft, xB = planes.plateFwd;
  return stages.map((s, k) => {
    const pin = byId(catalog, s.pinion), gear = byId(catalog, s.gear);
    const ps = pin.specs, gs = gear.specs;
    const m = ps.module_mm;
    const v = pitchLineSpeed(m, ps.teeth, sp[k].pinion_rpm);
    const x = planes.stage?.[k];
    const support = Number.isFinite(x) && Number.isFinite(xA) ? supportClass(x, xA, xB) : 'weak';
    const one = (part, z, zMate, mate) => {
      const sp_ = part.specs;
      const cat = { bending_Nm: sp_.allowable_torque_bending_nm ?? null, surface_Nm: sp_.allowable_torque_surface_durability_nm ?? sp_.allowable_torque_surface_nm ?? null };
      if (isDuracon(part)) return lewisDuraconRating(cat, { m, z, v_m_s: v, mateDuracon: isDuracon(mate), reversing: gr.reversing, grease: gr.plastic_lubricant?.grease === true, temperature_C: gr.max_temperature_C });
      const hardened = /harden|carburi/i.test(sp_.heat_treatment ?? '');
      const lub = gr.metal_lubricant;
      return jgmaGearRating(cat, { m, z, zMate, b_mm: Math.min(ps.face_width_mm ?? Infinity, gs.face_width_mm ?? Infinity), v_m_s: v, support, lubricant_cSt_50C: lub?.cSt_50C ?? null, reversing: gr.reversing, hardened });
    };
    const rp = one(pin, ps.teeth, gs.teeth, gear), rg = one(gear, gs.teeth, ps.teeth, pin);
    return {
      ...s,
      pinion_allowable_Nm: rp.allowable_Nm, gear_allowable_Nm: rg.allowable_Nm,
      catalogue_pinion_allowable_Nm: s.pinion_allowable_Nm, catalogue_gear_allowable_Nm: s.gear_allowable_Nm,
      rating_check: { crank_Hz: gr.crank_Hz, pinion_rpm: sp[k].pinion_rpm, wheel_rpm: sp[k].wheel_rpm, pitch_line_m_s: v, support, plane_x_m: x ?? null, pinion: rp, gear: rg },
    };
  });
}
