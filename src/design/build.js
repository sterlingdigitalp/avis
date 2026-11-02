// From parameters and a catalogue to a complete design: geometry, BOM, sized structure, CG, margin (M6).
import { byId, ofCategory, servoAt, motorModel, batteryModel, tubeAllowable, tubeSection, becOf } from './catalog.js';
import { makeFlapDrive, makeWing, wingArea, mac, wingSpan, linkageCycleStats } from '../physics/kinematics.js';
import { designAtmosphere, DESIGN_ALTITUDE_M, DESIGN_TEMPERATURE_C } from '../physics/atmosphere.js';
import { POLARS } from '../physics/polars.js';
import { podAndBoomLoft, loftVolume, loftWettedArea } from '../physics/geometry.js';
import { CLAIM } from './schema.js';

const DEG = Math.PI / 180;
export const MARGIN = 0.15; // M6: unlisted margin for wire, glue and fasteners

export const CANDIDATES = {
  larus: { name: 'larus-gl-2', label: 'Larus GL-2', reference: 'Gull, SmartBird-style active twist', spanRange: [1.5, 2.0] },
  falco: { name: 'falco-pf-2', label: 'Falco PF-2', reference: 'Falcon, high wing loading', spanRange: [1.0, 1.2] },
  columba: { name: 'columba-rp-2', label: 'Columba RP-2', reference: 'Pigeon, short broad wing', spanRange: [0.7, 0.9] },
};

/**
 * params: {
 *   candidate, span_m, AR, taper (tip/root chord), elbowFrac, elbowGain, sweepTip_m,
 *   airfoil, incidence_deg, washout_deg, twistStartFrac,
 *   amp_deg, phiMean_deg, tailVolume, tailArmChords, tailDihedral_deg, staticMargin,
 *   parts: { motor, esc, battery, twistServo, tailServo, strokeServo, fc, gps, airspeed, receiver,
 *            sparInner, sparOuter, leRod, teRod, ribMaterial, film, keel, gears:[p1,g1,p2,g2], bearing },
 *   gearRatio target, crank_mm
 * }
 */
export function buildDesign(params, catalog) {
  const p = params;
  const cat = CANDIDATES[p.candidate];
  const atm = designAtmosphere();
  const span = p.span_m;
  const S = (span * span) / p.AR;
  const hingeZ = p.hingeZ_m ?? 0.025;
  const semi = span / 2 - hingeZ;
  const elbow = semi * (p.elbowFrac ?? 0.5);
  const outer = semi - elbow;
  // Planform: inner segment nearly constant chord, outer tapers. Solve root chord from the area.
  const taper = p.taper ?? 0.6;
  const innerRatio = p.innerTaper ?? 0.95;
  // Area of one wing = elbow*(cr + cr*ir)/2 + outer*(cr*ir + cr*ir*taper)/2 = S/2
  const cr = S / 2 / (0.5 * elbow * (1 + innerRatio) + 0.5 * outer * innerRatio * (1 + taper));
  const ce = cr * innerRatio;
  const ct = ce * taper;
  // The spar sits at 30% of the root chord. Hinge at the same x.
  const sparFrac = p.sparFrac ?? 0.3;
  const xleRoot = sparFrac * cr;
  const xleElbow = xleRoot - (p.sweepElbow_m ?? 0) + (sparFrac * (ce - cr)) * 0 - 0;
  const xleTip = xleElbow - (p.sweepTip_m ?? 0.08 * outer / 0.42 * ce / 0.25);
  const polarName = p.airfoil ?? 'e387-film';
  const pol = POLARS[polarName];
  const wing = {
    hinge: [0, p.hingeY_m ?? 0.02, hingeZ],
    spar_x: 0,
    incidence: (p.incidence_deg ?? 3) * DEG,
    washout: (p.washout_deg ?? -2) * DEG,
    camber: pol.camber ?? 0.03,
    camber_pos: pol.camberPos ?? 0.4,
    phi_ref: (p.phiRef_deg ?? 0) * DEG,
    airfoil: polarName,
    twist_stiffness_Nm_rad: p.twistStiffness ?? 0.05,
    segments: [
      { length: elbow, chord_in: cr, chord_out: ce, xle_in: xleRoot, xle_out: xleElbow, gain: 0, twist_in: 0, twist_out: p.elbowTwist ?? 0.3 },
      { length: outer, chord_in: ce, chord_out: ct, xle_in: xleElbow, xle_out: xleTip, gain: p.elbowGain ?? 0.5, twist_in: p.elbowTwist ?? 0.3, twist_out: 1 },
    ],
  };
  const w = makeWing(wing);
  const macv = mac(w);
  const Sact = 2 * wingArea(w);
  const parts = p.parts;
  const bom = [];
  const add = (part, count, role, pos = null) => {
    bom.push({ id: part.id, name: part.name, role, count, unit_mass_g: part.mass_g, mass_g: part.mass_g * count, source_url: part.source_url, verified: part.verified, retrieved: part.retrieved, pos });
    return part.mass_g * count;
  };
  const structure = [];
  const addStruct = (name, material, mass_g, basis) => {
    structure.push({ name, material_id: material?.id ?? null, material: material?.name ?? null, mass_g, basis, source_url: material?.source_url ?? null });
    return mass_g;
  };
  // ---- Wing structure
  const inner = byId(catalog, parts.sparInner);
  const outerSpar = byId(catalog, parts.sparOuter);
  const leRod = byId(catalog, parts.leRod);
  const teRod = byId(catalog, parts.teRod);
  const rib = byId(catalog, parts.ribMaterial);
  const film = byId(catalog, parts.film);
  const innerMass = (inner.mass_g * elbow);
  const outerMass = (outerSpar.mass_g * outer);
  addStruct('inner spar, 2 off', inner, 2 * innerMass, `${inner.mass_g} g/m x ${elbow.toFixed(3)} m x 2`);
  addStruct('outer spar, 2 off', outerSpar, 2 * outerMass, `${outerSpar.mass_g} g/m x ${outer.toFixed(3)} m x 2`);
  // LE and TE members run the whole span of each wing.
  const leMass = leRod.mass_g * semi;
  const teMass = teRod.mass_g * semi;
  addStruct('leading edge rod, 2 off', leRod, 2 * leMass, `${leRod.mass_g} g/m x ${semi.toFixed(3)} m x 2`);
  addStruct('trailing edge rod, 2 off', teRod, 2 * teMass, `${teRod.mass_g} g/m x ${semi.toFixed(3)} m x 2`);
  // Film both faces plus a 5% allowance on curvature over the camber
  const filmArea = 2 * Sact * 1.03;
  const filmG = (film.specs.areal_density_g_m2 ?? film.mass_g) * filmArea;
  addStruct('wing film, both faces', film, filmG, `${film.specs.areal_density_g_m2 ?? film.mass_g} g/m2 x ${filmArea.toFixed(3)} m2`);
  // Ribs: section area ~ 0.07 c^2 of a 9% section, lightened to 40% solid; spacing about 0.12 m at mean chord
  const ribSpacing = p.ribSpacing_m ?? 0.1;
  const ribCount = 2 * Math.ceil(semi / ribSpacing);
  const tRib = p.ribThickness_m ?? 0.002;
  const meanc = Sact / (2 * semi);
  const ribArea = 0.06 * meanc * meanc * 0.45;
  const ribDensity = rib.specs.density_kg_m3 ?? (rib.specs.density_g_cm3 ?? 0) * 1000;
  const ribMassEach = ribArea * tRib * ribDensity * 1000;
  addStruct(`ribs, ${ribCount} off`, rib, ribCount * ribMassEach, `area ${ribArea.toExponential(2)} m2 x ${tRib} m x ${ribDensity} kg/m3 each`);
  const wingStructG = 2 * innerMass + 2 * outerMass + 2 * leMass + 2 * teMass + filmG + ribCount * ribMassEach;
  // ---- Tail: V surfaces. Horizontal volume coefficient VH = St_proj * lt / (S macv)
  const lt = (p.tailArmChords ?? 2.4) * cr;
  const dih = (p.tailDihedral_deg ?? 35) * DEG;
  const SH = ((p.tailVolume ?? 0.5) * Sact * macv) / lt;
  // Horizontal projection of total V area is S_total cos(dih)
  const Stail = SH / (Math.cos(dih) ** 2);
  const ARtail = p.tailAR ?? 3.5;
  const eachS = Stail / 2;
  const spanEach = Math.sqrt(ARtail * eachS);
  const chordT = (eachS / spanEach) * (1 / (0.5 * (1 + (p.tailTaper ?? 0.7))));
  const tailRod = byId(catalog, parts.teRod);
  const tailFilm = filmG / filmArea * 2 * Stail * 1.03;
  const tailSparG = tailRod.mass_g * spanEach * 2 * 2; // spar + edge per surface
  const tailRibG = 8 * ribMassEach * (chordT / meanc) ** 2;
  addStruct('tail surfaces, film, spar and ribs', film, tailFilm + tailSparG + tailRibG, `film ${(tailFilm).toFixed(1)} g + rods ${(tailSparG).toFixed(1)} g + ribs ${(tailRibG).toFixed(1)} g`);
  // ---- Fuselage: keel tube plus tail boom plus formers and film
  const keel = byId(catalog, parts.keel);
  const bodyLen = Math.max(0.28, 0.5 * lt + 0.1) + (p.bodyExtra_m ?? 0);
  const noseLen = 0.45 * bodyLen;
  const keelLen = lt + noseLen * 0.5;
  addStruct('keel and tail boom', keel, keel.mass_g * keelLen, `${keel.mass_g} g/m x ${keelLen.toFixed(3)} m`);
  const bodyW = p.bodyWidth_m ?? 0.06;
  const bodyH = p.bodyHeight_m ?? 0.07;
  const shell = (Math.PI * 0.5 * (bodyW + bodyH) * bodyLen * 0.8);
  addStruct('fuselage film shell', film, (film.specs.areal_density_g_m2 ?? film.mass_g) * shell, `${shell.toFixed(3)} m2`);
  addStruct('fuselage formers', rib, 5 * bodyW * bodyH * 0.5 * 0.002 * ribDensity * 1000, '5 off, 50% lightened, 2 mm');
  // ---- Drive
  const plate = byId(catalog, parts.drivePlate ?? 'plate-ec-2mm');
  const bellArea = 4 * (p.crank_mm ?? 12) * 25e-6 + 2 * 0.03 * 0.03;
  const crankPlateG = (plate.specs.areal_density_kg_m2 * 1000) * bellArea;
  addStruct('crank, rockers and gearbox plates', plate, crankPlateG + (p.gearboxPlate_g ?? 8), `${plate.specs.areal_density_kg_m2} kg/m2 x ${bellArea.toExponential(2)} m2, plus gearbox cheek allowance`);
  // Couplers: pultruded rod, two
  const couplerRod = byId(catalog, parts.coupler ?? 'rod-ec-3mm');
  const bMm = p.coupler_mm ?? 70;
  addStruct('couplers, 2 off', couplerRod, (2 * couplerRod.mass_g * bMm) / 1000, `${couplerRod.mass_g} g/m x ${bMm} mm x 2`);
  // ---- Parts
  const motorP = byId(catalog, parts.motor);
  const escP = byId(catalog, parts.esc);
  const batP = byId(catalog, parts.battery);
  const twistP = byId(catalog, parts.twistServo);
  const tailP = byId(catalog, parts.tailServo);
  const strokeP = byId(catalog, parts.strokeServo);
  const fcP = byId(catalog, parts.fc);
  const gpsP = byId(catalog, parts.gps);
  const aspdP = byId(catalog, parts.airspeed);
  const rxP = byId(catalog, parts.receiver);
  add(motorP, 1, 'flap motor');
  add(escP, 1, 'flap ESC, its BEC supplies the servos and avionics');
  add(batP, 1, 'battery');
  add(twistP, 2, 'wing twist servo');
  add(tailP, 2, 'tail servo');
  add(strokeP, 1, 'stroke amplitude servo');
  add(fcP, 1, 'flight controller with IMU and baro');
  add(gpsP, 1, 'GPS');
  add(aspdP, 1, 'airspeed sensor');
  add(rxP, 1, 'receiver');
  for (const g of parts.gears) add(byId(catalog, g), 1, 'gear');
  add(byId(catalog, parts.bearing), p.bearingCount ?? 8, 'bearing');
  const partsG = bom.reduce((a, b) => a + b.mass_g, 0);
  const structG = structure.reduce((a, b) => a + b.mass_g, 0);
  const marginG = MARGIN * (partsG + structG);
  const totalG = partsG + structG + marginG;

  // ---- Layout
  const tSpar = tubeSection(inner);
  const skinKgM2 = ((filmG + ribCount * ribMassEach) / 1000) / Sact;
  const sparKgPerM = (innerMass * elbow + outerMass * outer) / semi / semi / 1000 * 1; // length-weighted, kg/m
  const massWingOne = (innerMass + outerMass + leMass + teMass + filmG / 2 + (ribCount * ribMassEach) / 2) / 1000;
  const twistS = wing.segments[0].length + 0.65 * outer;
  // Distribution for the flight model: spar + edges along the span by length, skin by area. Margin share on top.
  const marginFrac = 1 + MARGIN;
  const wingMass = {
    spar_kg_per_m: (((innerMass + outerMass + leMass + teMass) / semi) / 1000) * marginFrac,
    skin_kg_per_m2: skinKgM2 * marginFrac,
    point: [{ name: 'twist servo', s_m: twistS, mass_kg: (twistP.mass_g / 1000) * marginFrac, d_m: 0 }],
    one_wing_kg: massWingOne * marginFrac + (twistP.mass_g / 1000) * marginFrac,
  };
  // Neutral point (x forward) and CG target. Wing AC near c/4 of MAC; tail adds a share by its volume.
  const xAcWing = xleRoot - 0.25 * macv - 0.05 * macv;
  const AR = span * span / Sact;
  const aw = (2 * Math.PI) / (1 + 2 / AR);
  const at = (2 * Math.PI) / (1 + 2 / ARtail);
  const VH = (SH * lt) / (Sact * macv);
  const dedA = 2 * aw / (Math.PI * AR);
  const npShift = VH * (at / aw) * (1 - dedA) * 0.7 * macv;
  const xNP = xAcWing - npShift;
  const xCGtarget = xNP + (p.staticMargin ?? 0.1) * macv;
  // Body lumps. Battery is placed last to set the CG.
  const lumps = [];
  const lump = (name, g, x, y = 0, z = 0, k = 0) => lumps.push({ name, mass_kg: (g / 1000) * marginFrac, pos: [x, y, z], gyration_m: k });
  const motorX = 0.04;
  lump('flap motor', motorP.mass_g, motorX, 0.0);
  lump('gears and bearings', bom.filter((b) => b.role === 'gear' || b.role === 'bearing').reduce((a, b) => a + b.mass_g, 0), 0.015, 0.0);
  lump('crank, rockers, plates', crankPlateG + (p.gearboxPlate_g ?? 8), 0.01, 0.01);
  lump('couplers', (2 * couplerRod.mass_g * bMm) / 1000, 0.0, 0.0);
  lump('ESC', escP.mass_g, 0.07, -0.01);
  lump('flight controller', fcP.mass_g, 0.0, -0.01);
  lump('receiver and airspeed', rxP.mass_g + aspdP.mass_g, 0.11, 0.0);
  lump('GPS', gpsP.mass_g, -0.03, 0.03);
  lump('tail servos', 2 * tailP.mass_g, -0.08, 0.0);
  lump('stroke servo', strokeP.mass_g, -0.03, 0.0);
  lump('keel and boom', keel.mass_g * keelLen, -lt / 2 + noseLen * 0.25, 0.0, 0, 0.29 * keelLen);
  lump('fuselage shell and formers', (film.specs.areal_density_g_m2 ?? film.mass_g) * shell + 5 * bodyW * bodyH * 0.5 * 0.002 * ribDensity * 1000, -0.05, 0.0, 0, 0.3 * bodyLen);
  lump('tail surfaces', tailFilm + tailSparG + tailRibG, -lt, 0.02);
  // Mean position of the wing mass, for the CG budget
  const wingOneKg = wingMass.one_wing_kg;
  const xWing = xleRoot - 0.42 * meanc; // skin centroid, spar at 0: weighted below
  const xWingAvg = (((filmG / 2 + (ribCount * ribMassEach) / 2) * xWing + (teMass) * (xleRoot - meanc) + (leMass) * (xleRoot)) / 1000 * marginFrac) / wingOneKg;
  let sumM = 2 * wingOneKg;
  let sumMx = 2 * wingOneKg * xWingAvg;
  for (const l of lumps) {
    sumM += l.mass_kg;
    sumMx += l.mass_kg * l.pos[0];
  }
  const battKg = (batP.mass_g / 1000) * marginFrac;
  const xBattery = (xCGtarget * (sumM + battKg) - sumMx) / battKg;
  lumps.push({ name: 'battery', mass_kg: battKg, pos: [xBattery, -0.005, 0], gyration_m: 0.02 });
  const total = sumM + battKg;
  // Crank-rocker sized from amplitude
  const ampNom = (p.amp_deg ?? 25) * DEG;
  const a = (p.crank_mm ?? 12) / 1000;
  const b0 = (p.coupler_mm ?? 70) / 1000;
  const d0 = p.groundLink_mm ? p.groundLink_mm / 1000 : Math.max(0.05, b0);
  const ampMax = (p.ampMax_deg ?? Math.min((p.amp_deg ?? 25) * 1.6, 60)) * DEG;
  const ampMin = ampNom * 0.6;
  const drive = { kind: 'crank-rocker', a, b: b0, d: d0, ampMin, ampMax, ampNominal: ampNom, phiMean: (p.phiMean_deg ?? 5) * DEG, gearRatio: p.gearRatio, gearEfficiency: p.gearEff ?? 0.85, crankFriction_Nm: 0.01, gears: parts.gears };
  const motor = motorModel(motorP);
  const batt = batteryModel(batP);
  // SPEC ruling after catalogue round 2: the design states its servo supply voltage and the BEC that provides it
  // (here the flap ESC's built-in BEC, in the BOM with the ESC), and no servo figure is extrapolated outside its rows.
  const bec = becOf(escP);
  const supplyV = p.servoSupply_V ?? bec.volts;
  if (!Number.isFinite(supplyV)) throw new Error(`ESC ${escP.id} publishes no BEC voltage: no servo supply`);
  const esc = { id: escP.id, I_cont: escP.specs.continuous_current_a, bec_A: bec.amps, bec_V: bec.volts, bec_kind: bec.kind, eta: 0.95, eta_assumed: true, regenerative: false, regenerative_basis: 'no regenerative operation stated in the ESC documentation read for the catalogue' };
  const servoSpec = (part, v) => {
    const s = servoAt(part, v);
    if (!s) throw new Error(`servo ${part.id} has no published row covering ${v} V (no extrapolation)`);
    if (!Number.isFinite(s.stall_A)) throw new Error(`servo ${part.id} has no published stall current (SPEC M8)`);
    return { id: part.id, stall_Nm: s.stall_Nm, speed_rad_s: s.speed_rad_s, stall_A: s.stall_A, tau_s: 0.02, idle_A: part.specs.idle_current_a ?? 0, volts: s.volts, mass_kg: part.mass_g / 1000 };
  };
  const servos = {
    twist: servoSpec(twistP, supplyV),
    tail: servoSpec(tailP, supplyV),
    stroke: { ...servoSpec(strokeP, supplyV), amp_rate_rad_s: 0.5 },
  };
  // Body geometry: pod from the nose (ahead of the battery and receiver) back bodyLen, then the keel tube as a boom to
  // the tail's trailing edge. The tunnels, page and film build from this loft (src/physics/geometry.js).
  const noseX = Math.max(0.45 * bodyLen, xBattery + 0.05, 0.14);
  const loft = podAndBoomLoft({ noseX_m: noseX, podLength_m: bodyLen, width_m: bodyW, height_m: bodyH, boomOd_m: keel.specs.od_mm / 1000, tailEndX_m: -lt - 0.75 * chordT, y_m: 0 });
  const body = {
    loft,
    loft_wetted_area_m2: loftWettedArea(loft),
    CdA_front_m2: (p.bodyCd ?? 0.25) * Math.PI / 4 * bodyW * bodyH,
    CdA_top_m2: 1.0 * bodyW * bodyLen * 0.8,
    CdA_side_m2: 1.0 * bodyH * bodyLen * 0.8,
    volume_m3: loftVolume(loft), // one source: the Munk moment uses the loft's volume
    kMunk: 0.8,
    cp: [0.05, 0, 0],
    length_m: bodyLen,
    width_m: bodyW,
    height_m: bodyH,
    bottom_y_m: -bodyH / 2,
  };
  // Spar check data; loads come from flight.
  const allow = tubeAllowable(inner);
  const allowOuter = tubeAllowable(outerSpar);
  const sparInner = { ...tubeSection(inner), id: inner.id, allowable_MPa: allow.sigma_MPa, allowable_basis: allow.basis, mass_g_per_m: inner.mass_g };
  const sparOuter = { ...tubeSection(outerSpar), id: outerSpar.id, allowable_MPa: allowOuter.sigma_MPa, allowable_basis: allowOuter.basis, mass_g_per_m: outerSpar.mass_g };
  const gearStages = stages(parts.gears, catalog);

  const d = {
    name: cat.name, label: cat.label, candidate: p.candidate, reference: cat.reference, claim: CLAIM,
    params: p,
    atmosphere: { altitude_m: DESIGN_ALTITUDE_M, temperature_C: DESIGN_TEMPERATURE_C, rho_kg_m3: atm.rho, mu_Pa_s: atm.mu },
    wing,
    geometry: { span_m: wingSpan(w), area_m2: Sact, AR: wingSpan(w) ** 2 / Sact, mac_m: macv, chord_root_m: cr, chord_elbow_m: ce, chord_tip_m: ct, tail_arm_m: lt, wing_loading_N_m2: (total * 9.80665) / Sact, x_np_m: xNP, x_cg_m: xCGtarget, static_margin: p.staticMargin ?? 0.1 },
    drive,
    tail: { x_m: -lt, y_m: 0.02, z_m: 0.005, span_each_m: spanEach, chord_root_m: chordT, taper: p.tailTaper ?? 0.7, dihedral_rad: dih, incidence_rad: (p.tailIncidence_deg ?? 0) * DEG, polar: 'thin-plate', AR_each: ARtail, lift_factor: ARtail / (ARtail + 2) * 1.05, kEps: 1.0, area_total_m2: Stail, deflection_max_rad: 30 * DEG },
    body,
    mass: {
      total_kg: total,
      total_g: total * 1000,
      parts_g: partsG,
      structure_g: structG,
      margin_g: marginG,
      margin_frac: MARGIN,
      lumps,
      wing: wingMass,
      x_battery_m: xBattery,
    },
    motor, esc, battery: batt, servos,
    structure: { items: structure, spar_inner: sparInner, spar_outer: sparOuter, gear_stages: gearStages, coupler: { id: couplerRod.id, od_m: couplerRod.specs.od_mm / 1000, length_m: bMm / 1000, E_GPa: couplerRod.specs.tensile_modulus_lengthways_gpa_range?.[0] ?? null, compressive_MPa: couplerRod.specs.compressive_strength_lengthways_mpa_range?.[0] ?? null } },
    bom: { parts: bom, structure, parts_g: partsG, structure_g: structG, margin_g: marginG, total_g: totalG },
    electrical: { avionics_A_5V: 0.35, avionics_note: 'FC, GPS, receiver at 5 V, assumed; catalogue lists no current draw', servo_supply_V: supplyV, bec: { part: escP.id, volts: bec.volts, amps: bec.amps, kind: bec.kind, published: bec.text, note: 'built into the flap ESC; its mass is in the BOM with the ESC' } },
    cruise: {},
    assumptions: [
      `Section polar ${polarName}: ${pol.assumed ?? 'measured table'}`,
      'Motor rotor inertia estimated from mass and diameter; not published.',
      'Battery internal resistance estimated from claimed C rate; not published.',
      'ESC efficiency 0.95 assumed.',
      'Servo current is the SPEC M8 estimate: stall current x (torque / stall torque), idle if published.',
      'Tail polar is an assumed thin plate.',
    ],
  };
  return d;
}

function parseBec(s) {
  const m = /(\d+(?:\.\d+)?)\s*A/.exec(String(s ?? ''));
  return m ? parseFloat(m[1]) : null;
}

/** Two-stage reduction: gears [p1,g1,p2,g2]. Returns the ratio and the torque ratings to check. */
export function stages(ids, catalog) {
  const parts = ids.map((i) => byId(catalog, i));
  const out = [];
  for (let k = 0; k + 1 < parts.length; k += 2) {
    const pin = parts[k], gear = parts[k + 1];
    out.push({ pinion: pin.id, gear: gear.id, ratio: gear.specs.teeth / pin.specs.teeth, pinion_allowable_Nm: minRating(pin), gear_allowable_Nm: minRating(gear) });
  }
  return out;
}
export function minRating(part) {
  const s = part.specs;
  const v = [s.allowable_torque_bending_nm, s.allowable_torque_surface_durability_nm].filter((x) => Number.isFinite(x));
  return v.length ? Math.min(...v) : null;
}
export function gearRatioOf(ids, catalog) {
  return stages(ids, catalog).reduce((r, s) => r * s.ratio, 1);
}
