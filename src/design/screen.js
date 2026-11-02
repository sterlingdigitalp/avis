// Coarse screen: thousands of parameterised designs scored in seconds with closed-form estimates, using real
// catalogue parts only. It ranks; the flight model decides. Nothing here is evidence for a gate.

import { ofCategory, isEligible, motorModel, batteryModel, servoAt, tubeAllowable, tubeSection } from './catalog.js';
import { buildDesign, minRating, CANDIDATES } from './build.js';
import { designAtmosphere, G0 } from '../physics/atmosphere.js';

const DEG = Math.PI / 180;

/** Eligible parts by role, with the SPEC rules applied (continuous rating, servo current, published strength). */
export function eligibleParts(catalog) {
  const ok = (p) => isEligible(p);
  const motors = ofCategory(catalog, 'motor').filter(ok).filter((p) => motorModel(p).I_cont !== null);
  const batteries = ofCategory(catalog, 'battery').filter(ok).filter((p) => p.specs.chemistry === 'LiPo' && p.specs.cells_series >= 2 && p.specs.cells_series <= 4 && p.specs.capacity_mah > 0);
  const escs = ofCategory(catalog, 'esc').filter(ok).filter((p) => Number.isFinite(p.specs.continuous_current_a));
  const servos = ofCategory(catalog, 'servo').filter(ok).filter((p) => Array.isArray(p.specs.stall_current) && p.specs.stall_current.length);
  const tubes = [...ofCategory(catalog, 'carbon_tube'), ...ofCategory(catalog, 'carbon_rod')].filter(ok).filter((p) => tubeAllowable(p).sigma_MPa !== null);
  const fcs = ofCategory(catalog, 'flight_controller').filter(ok).filter((p) => p.specs.imu_onboard && p.specs.baro_onboard);
  const gps = ofCategory(catalog, 'gps').filter(ok).filter((p) => p.specs.compass || p.specs.compass_conflict || p.specs.magnetometer_onboard);
  const aspd = ofCategory(catalog, 'airspeed').filter(ok);
  const rx = ofCategory(catalog, 'receiver').filter(ok);
  const gears = ofCategory(catalog, 'gear').filter(ok).filter((p) => minRating(p) !== null);
  const bearings = ofCategory(catalog, 'bearing').filter(ok);
  const films = ofCategory(catalog, 'wing_film').filter(ok);
  const ribs = ofCategory(catalog, 'rib_material').filter(ok).filter((p) => p.id.includes('balsa'));
  const lightest = (arr) => arr.slice().sort((a, b) => a.mass_g - b.mass_g)[0];
  return { motors, batteries, escs, servos, tubes, fcs, gps, aspd, rx, gears, bearings, films, ribs, lightest };
}

/** Two-stage spur reductions from same-module pairs, pinion 12-20 teeth. */
export function gearTrains(gears) {
  const byModule = new Map();
  for (const g of gears) {
    const k = `${g.specs.module_mm}-${g.specs.material}`;
    if (!byModule.has(k)) byModule.set(k, []);
    byModule.get(k).push(g);
  }
  const stagesList = [];
  for (const list of byModule.values()) {
    for (const p of list) for (const w of list) {
      if (p.specs.teeth >= 12 && p.specs.teeth <= 20 && w.specs.teeth >= 2 * p.specs.teeth) stagesList.push({ p, w, ratio: w.specs.teeth / p.specs.teeth, mass: p.mass_g + w.mass_g, module: p.specs.module_mm });
    }
  }
  const trains = [];
  for (const a of stagesList) for (const b of stagesList) {
    if (b.module < a.module) continue; // the slow stage is at least as coarse
    trains.push({ ids: [a.p.id, a.w.id, b.p.id, b.w.id], ratio: a.ratio * b.ratio, mass: a.mass + b.mass, crankRating: Math.min(minRating(b.w), minRating(b.p) * b.ratio, minRating(a.p) * a.ratio * b.ratio, minRating(a.w) * b.ratio), stage1: a, stage2: b });
  }
  return trains;
}

/** Closed-form estimate for one parameter set. Returns margins (>= 0 good) and headline numbers. */
export function coarse(params, catalog) {
  const d = buildDesign(params, catalog);
  const atm = designAtmosphere();
  const rho = atm.rho;
  const W = d.mass.total_kg * G0;
  const S = d.geometry.area_m2;
  const AR = d.geometry.AR;
  const semi = d.geometry.span_m / 2;
  const CLmax = 1.0; // film-covered E387-like wing, conservative of the 1.2 section maximum
  const Vs = Math.sqrt((2 * W) / (rho * S * CLmax));
  const V = Math.max(1.3 * Vs, 6);
  const q = 0.5 * rho * V * V;
  const CL = W / (q * S);
  const Cd0 = 0.03;
  const D = q * S * (Cd0 + (CL * CL) / (Math.PI * AR * 0.85)) + q * d.body.CdA_front_m2 * 1.5;
  const etaTotal = 0.22; // electrical to useful power for a flapping drive (SmartBird: 0.25)
  const P = (D * V) / etaTotal + 5 * 0.4;
  const bt = d.battery;
  const E = bt.capacity_Ah * bt.cells * 3.7 * 3600 * 0.8;
  const endurance = E / P / 60;
  const Icont = Math.min(d.motor.I_cont ?? 0, d.esc.I_cont ?? 0, bt.I_cont ?? 0);
  const Vbat = bt.cells * 3.6;
  const ratio = (Icont * Vbat) / P;
  // Stroke frequency for Strouhal 0.3 at the 3/4 span
  const A = params.amp_deg * DEG;
  const f = (0.3 * V) / (2 * A * 0.75 * semi * 1.2);
  const w = 2 * Math.PI * f;
  // Crank torque: aero lift moment plus wing inertia, through the stroke ratio A
  const mw = d.mass.wing.one_wing_kg;
  const Iw = mw * (0.45 * semi) ** 2;
  const Qaero = (W / 2) * 0.45 * semi * 1.6;
  const Qin = Iw * A * w * w;
  const Tcrank = 2 * A * Math.hypot(Qaero, Qin) * 1.3;
  const G = d.drive.gearRatio;
  const crankRating = params.crankRating;
  // Motor voltage needed at that crank speed and torque (gear efficiency 0.85)
  const Kv = d.motor.kv_rad_per_v_s;
  const Iavg = ((Tcrank / 1.6) / (G * 0.85)) * Kv + d.motor.I0;
  const Vneed = (w * G) / Kv + Iavg * d.motor.R;
  // Twist servo load: section moment about the spar plus apparent mass offset
  const tw = servoAt(catalog.find((p) => p.id === params.parts.twistServo), 5);
  const c = d.geometry.mac_m;
  const Mtw = 0.5 * rho * (V * 1.3) ** 2 * c * c * 0.12 * semi * 0.5;
  const sparAllow = d.structure.spar_inner.allowable_MPa * 1e6;
  const Mroot = (W / 2) * 0.45 * semi * 3.0; // 3 g at the root, flap and gust
  const sigma = (Mroot * d.structure.spar_inner.c) / d.structure.spar_inner.I;
  const margins = {
    launch: (7.0 - Vs) / 7.0, // stall below 7 m/s leaves flapping margin to 8 m/s
    endurance: (endurance - 12) / 12,
    power: (ratio - 1.6) / 1.6,
    gear: (crankRating - Tcrank) / crankRating,
    voltage: (0.85 * Vbat - Vneed) / Vbat,
    spar: (sparAllow / 1.5 - sigma) / (sparAllow / 1.5),
    servo: (tw.stall_Nm - Mtw * 1.5) / tw.stall_Nm,
    freq: (8 - f) / 8,
  };
  const minMargin = Math.min(...Object.values(margins));
  return { design: d, Vs, V, P, endurance, ratio, f, Tcrank, Vneed, margins, minMargin, mass_g: d.mass.total_g };
}

const CAND_GRID = {
  larus: { span: [1.5, 1.6, 1.7, 1.8, 1.9, 2.0], AR: [7, 8, 9, 10], taper: [0.5, 0.65], amp: [14, 18] },
  falco: { span: [1.0, 1.1, 1.2], AR: [6, 7, 8], taper: [0.45, 0.6], amp: [14, 18] },
  columba: { span: [0.7, 0.8, 0.9], AR: [4.5, 5.5, 6.5], taper: [0.6, 0.75], amp: [16, 20] },
};

/** Score many designs for one candidate. Returns all results sorted best first. */
export function screen(candidate, catalog, { limit = Infinity } = {}) {
  const E = eligibleParts(catalog);
  const L = E.lightest;
  const trains = gearTrains(E.gears).filter((t) => t.ratio >= 8 && t.ratio <= 60);
  const g = CAND_GRID[candidate];
  const out = [];
  const tubeBy = (od) => E.tubes.filter((t) => t.category === 'carbon_tube').sort((a, b) => Math.abs(a.specs.od_mm - od) - Math.abs(b.specs.od_mm - od))[0];
  const inner = E.tubes.filter((t) => t.category === 'carbon_tube');
  const servos = E.servos;
  const avionics = { fc: L(E.fcs).id, gps: L(E.gps).id, airspeed: L(E.aspd).id, receiver: L(E.rx).id };
  const rods = E.tubes.filter((t) => t.category === 'carbon_rod').sort((a, b) => a.specs.od_mm - b.specs.od_mm);
  let count = 0;
  for (const span of g.span) for (const AR of g.AR) for (const taper of g.taper) for (const amp of g.amp) {
    for (const motor of E.motors) for (const battery of E.batteries) {
      const esc = E.escs.filter((e) => cellsOk(e, battery)).sort((a, b) => a.mass_g - b.mass_g).find((e) => e.specs.continuous_current_a >= Math.min(motorModel(motor).I_cont, 1e9));
      if (!esc) continue;
      for (const sparIn of inner) for (const tw of servos) {
        // Gear ratio aimed at the motor speed needed for the estimated flap frequency at 70% battery voltage
        const semi = span / 2;
        const V0 = 8;
        const f0 = (0.3 * V0) / (2 * amp * DEG * 0.75 * semi * 1.2);
        const Gwant = (0.65 * battery.specs.cells_series * 3.7 * motorModel(motor).kv_rad_per_v_s) / (2 * Math.PI * f0);
        // Within a third of the wanted ratio, the strongest train per gram; the flight model checks it.
        const band = trains.filter((t) => t.ratio >= 0.75 * Gwant && t.ratio <= 1.33 * Gwant);
        const train = (band.length ? band : trains).slice().sort((a, b) => b.crankRating / (b.mass + 10) - a.crankRating / (a.mass + 10))[0];
        if (!train) continue;
        const outerSpar = sparIn.specs.od_mm > 10 ? tubeBy(8) : sparIn;
        const params = {
          candidate, span_m: span, AR, taper, elbowFrac: 0.5, elbowGain: 0.5, amp_deg: amp, ampMax_deg: Math.min(1.6 * amp, 30), phiMean_deg: 5,
          gearRatio: train.ratio, crankRating: train.crankRating, staticMargin: 0.1, sparFrac: 0.35, crank_mm: 8, coupler_mm: 70,
          parts: {
            motor: motor.id, esc: esc.id, battery: battery.id, twistServo: tw.id, tailServo: L(servos).id, strokeServo: L(servos).id,
            ...avionics, sparInner: sparIn.id, sparOuter: outerSpar.id, leRod: rods.find((r) => r.specs.od_mm >= 2)?.id ?? rods[0].id,
            teRod: rods.find((r) => r.specs.od_mm >= 1.5)?.id ?? rods[0].id, ribMaterial: L(E.ribs).id, film: L(E.films).id, keel: tubeBy(8).id,
            gears: train.ids, bearing: L(E.bearings).id,
          },
        };
        const r = coarse(params, catalog);
        out.push({ params, ...summary(r) });
        if (++count >= limit) break;
      }
    }
  }
  out.sort((a, b) => b.minMargin - a.minMargin || a.mass_g - b.mass_g);
  return out;
}

function cellsOk(esc, batt) {
  const m = /(\d)\s*-\s*(\d)\s*S/i.exec(esc.specs.cells_text ?? '');
  if (!m) return false;
  const n = batt.specs.cells_series;
  return n >= +m[1] && n <= +m[2];
}

function summary(r) {
  return { mass_g: r.mass_g, Vs: r.Vs, V: r.V, P: r.P, endurance: r.endurance, ratio: r.ratio, f: r.f, Tcrank: r.Tcrank, Vneed: r.Vneed, margins: r.margins, minMargin: r.minMargin, ws: r.design.geometry.wing_loading_N_m2 };
}
