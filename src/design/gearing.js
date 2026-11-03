// Round 9: the gear train's ratings under KHK's stated conditions (SPEC ruling after physics round 8), from the
// catalogue parts, the drive layout (gear planes between the plates) and the design's stated running conditions
// (p.gearRating: the rating crank frequency, the lubricant on each kind of mesh, the gearbox temperature limit and
// whether the load reverses). src/physics/gears.js holds the corrections; this file only gathers their inputs.
import { jgmaGearRating, lewisDuraconRating, pitchLineSpeed, kvRow } from '../physics/gears.js';

const byId = (catalog, id) => catalog.find((p) => p.id === id);
// Duracon is Polyplastics' acetal (POM); KHK's DSF gears state 'Polyacetal' and take the same plastic method (round 10 fix:
// they were rated by the metal method before).
const isDuracon = (part) => /duracon|acetal/i.test(part.specs?.material ?? '');

/**
 * Round 10: a metal gear's row in JGMA's KV table (KHK Table 10.3, old JIS B 1702 grades) from its catalogue precision
 * ("JIS grade N5 (JIS B1702-1:1998)") through the design's stated old-grade equivalence (gr.accuracy.old_grade_of,
 * e.g. { N5: 1 }, with its basis) and tooth modification (gr.accuracy.modified, false unless the maker states it).
 * No stated grade or no equivalence: null (the rating then takes the band-only rule).
 */
export function kvRowOf(part, accuracy) {
  const n = /N(\d+)/.exec(part?.specs?.precision ?? '')?.[1];
  const g = n !== undefined ? accuracy?.old_grade_of?.[`N${n}`] : undefined;
  return Number.isFinite(g) ? kvRow(g, accuracy?.modified === true) : null;
}

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
/**
 * Round 12 (SPEC ruling after physics round 11): a gear's load cycles over the design life life = { flights,
 * flight_s } at rpm (the gear's own speed at the rating crank frequency, which bounds the flown crank rate): flights x
 * flight_s x rpm / 60, every cycle counted at the peak. Null without a stated life (no life credit).
 */
export function lifeCycles(life, rpm) {
  if (!life || !Number.isFinite(life.flights) || !Number.isFinite(life.flight_s) || !Number.isFinite(rpm)) return null;
  return (life.flights * life.flight_s * rpm) / 60;
}

/**
 * The JGMA 401-01 K_L column for a metal gear: carburized or nitrided teeth; otherwise a steel gear on its core
 * hardness, HB 120-220 (induction-hardened teeth take the core; no KHK steel gear here states a core over HB 220);
 * null (no life credit) for brass, stainless and anything else the table does not cover.
 */
export function lifeColumnOf(part) {
  const ht = String(part.specs?.heat_treatment ?? ''), mat = String(part.specs?.material ?? '');
  if (/carburi|nitrid/i.test(ht) || /carburi/i.test(mat)) return 'carburized_nitrided';
  if (/S45C|SCM\d|SCM4|AISI 1045|AISI 41/i.test(mat)) return 'hb120_220';
  return null;
}

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
      // Round 12: this gear's own revolutions over the design life (gr.life), counted at the rating crank frequency.
      const rpm = part === pin ? sp[k].pinion_rpm : sp[k].wheel_rpm;
      const cycles = lifeCycles(gr.life, rpm);
      if (isDuracon(part)) return lewisDuraconRating(cat, { m, z, v_m_s: v, mateDuracon: isDuracon(mate), reversing: gr.reversing, grease: gr.plastic_lubricant?.grease === true, temperature_C: gr.max_temperature_C, cycles });
      const hardened = /harden|carburi/i.test(sp_.heat_treatment ?? '');
      const lub = gr.metal_lubricant;
      const column = lifeColumnOf(part);
      return jgmaGearRating(cat, { m, z, zMate, b_mm: Math.min(ps.face_width_mm ?? Infinity, gs.face_width_mm ?? Infinity), v_m_s: v, support, lubricant_cSt_50C: lub?.cSt_50C ?? null, reversing: gr.reversing, hardened, kv_row: kvRowOf(part, gr.accuracy), ...(column ? { cycles, lifeColumn: column } : {}) });
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
