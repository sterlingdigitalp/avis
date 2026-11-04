// From parameters and a catalogue to a complete design: geometry, BOM, sized structure, CG, margin (M6).
import { rateStages } from './gearing.js';
import { tailMechBom } from './tail-mech.js';
import { byId, ofCategory, servoAt, motorModel, batteryModel, tubeAllowable, tubeSection, tubeSpec, becOf, escCellsCover, servoVoltageRange, servoStallCurrentConservative } from './catalog.js';
import { batteryOcv } from '../physics/flight.js';
import { makeFlapDrive, makeWing, wingArea, mac, wingSpan, linkageCycleStats } from '../physics/kinematics.js';
import { designAtmosphere, DESIGN_ALTITUDE_M, DESIGN_TEMPERATURE_C } from '../physics/atmosphere.js';
import { POLARS } from '../physics/polars.js';
import { podAndBoomLoft, loftVolume, loftWettedArea } from '../physics/geometry.js';
import { CLAIM } from './schema.js';
import { driveJoints, driveLayout } from './joints.js';
import { driveMembers, jointHousings } from './linkage.js';

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
  // Round 6: with a linkage of sized metal parts (params.linkage, src/design/linkage.js), their masses come from the
  // drive's layout, which comes from the built design: a first pass without them, then the design with them.
  // The linkage's members ride on the joints' load path, so they need it (params.joints).
  if (params.linkage && params.joints && !params._linkageItems) {
    const d0 = buildDesign({ ...params, _linkageItems: [] }, catalog);
    const lm = driveMembers(d0, catalog, params.linkage, driveLayout(d0, catalog), d0.structure.joints);
    return buildDesign({ ...params, _linkageItems: lm.items }, catalog);
  }
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
  const addStruct = (name, material, mass_g, basis, extra = {}) => {
    structure.push({ name, material_id: material?.id ?? null, material: material?.name ?? null, mass_g, basis, source_url: material?.source_url ?? null, ...extra });
    return mass_g;
  };
  // ---- Wing structure
  const inner = byId(catalog, parts.sparInner);
  const outerSpar = byId(catalog, parts.sparOuter);
  const leRod = byId(catalog, parts.leRod);
  const teRod = byId(catalog, parts.teRod);
  const rib = byId(catalog, parts.ribMaterial);
  const film = byId(catalog, parts.film);
  // round 16 (p.wingStructure, SPEC completeness): the inner spar from its socket's base, the edge rods from the root rib,
  // as the assembly check draws them (src/design/wing-solids.js)
  const WS = p.wingStructure;
  const innerLen = WS ? elbow - (WS.spar_from_m ?? p.linkage.rocker.hub_od_mm / 2000) : elbow, rodLen = WS ? semi - WS.rib_root_m : semi;
  const innerMass = (inner.mass_g * innerLen);
  // round 16: the outer spar to where the swept leading edge still clears it (WS.spar_to_m)
  const outerLen = WS?.spar_to_m ? WS.spar_to_m - elbow : outer;
  // round 17: weighed at the heavier of the listing's and the spec sheet's figures (catalog.js tubeSpec)
  const specSheet = !!p.tubeSpecSheet, outerGm = tubeSpec(outerSpar, specSheet).g_per_m, outerMass = (outerGm * outerLen);
  addStruct('inner spar, 2 off', inner, 2 * innerMass, `${inner.mass_g} g/m x ${innerLen.toFixed(WS ? 4 : 3)} m x 2`);
  addStruct('outer spar, 2 off', outerSpar, 2 * outerMass, `${outerGm} g/m x ${outerLen.toFixed(WS ? 4 : 3)} m x 2`);
  // LE and TE members run the whole span of each wing.
  const leMass = leRod.mass_g * rodLen;
  const teMass = teRod.mass_g * rodLen;
  addStruct('leading edge rod, 2 off', leRod, 2 * leMass, `${leRod.mass_g} g/m x ${rodLen.toFixed(WS ? 4 : 3)} m x 2`);
  addStruct('trailing edge rod, 2 off', teRod, 2 * teMass, `${teRod.mass_g} g/m x ${rodLen.toFixed(WS ? 4 : 3)} m x 2`);
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
  // round 17 (p.tailMech: the all-moving tail's pivots and linkage, src/design/tail-mech.js): its lines from its geometry
  const tailMech = p.tailMech ? tailMechBom({ x_m: -lt, z_m: 0.005, y_m: p.tailY_m ?? 0.02, span_each_m: spanEach, chord_root_m: chordT, taper: p.tailTaper ?? 0.7, dihedral_rad: dih }, p.tailY_m ?? 0.02, byId(catalog, parts.keel).specs.od_mm / 2000, { ...p.tailMech, teRod: parts.teRod }, catalog) : null;
  const tailMechStructG = tailMech ? tailMech.structure.reduce((a, q) => a + q.mass_g, 0) : 0;
  if (tailMech) {
    addStruct('tail film, both faces', film, tailFilm, `${(film.specs.areal_density_g_m2 ?? film.mass_g)} g/m2 over both faces of both surfaces, with the wing's curvature allowance`);
    addStruct('tail ribs, 8 off', rib, tailRibG, `8 x the wing rib (area ${(ribArea).toExponential(2)} m2 at the mean chord) scaled to the tail's root chord squared, x ${tRib} m x ${ribDensity} kg/m3`, { count: 8 });
    for (const q of tailMech.structure) addStruct(q.name, q.material, q.mass_g, q.basis, { count: q.count });
  } else if (p.tailLines === 'split') {
    // round 16 (SPEC completeness): the tail's covering, rods and ribs as their own lines (the rods and ribs have solids)
    addStruct('tail film, both faces', film, tailFilm, `${(film.specs.areal_density_g_m2 ?? film.mass_g)} g/m2 over both faces of both surfaces, with the wing's curvature allowance`);
    addStruct('tail rods, 4 off', tailRod, tailSparG, `${tailRod.mass_g} g/m x ${spanEach.toFixed(4)} m x 4 (a spar and a trailing edge per surface)`, { count: 4 });
    addStruct('tail ribs, 8 off', rib, tailRibG, `8 x the wing rib (area ${(ribArea).toExponential(2)} m2 at the mean chord) scaled to the tail's root chord squared, x ${tRib} m x ${ribDensity} kg/m3`, { count: 8 });
  } else addStruct('tail surfaces, film, spar and ribs', film, tailFilm + tailSparG + tailRibG, `film ${(tailFilm).toFixed(1)} g + rods ${(tailSparG).toFixed(1)} g + ribs ${(tailRibG).toFixed(1)} g`);
  // ---- Fuselage: keel tube plus tail boom plus formers and film
  const keel = byId(catalog, parts.keel);
  const bodyLen = Math.max(0.28, 0.5 * lt + 0.1) + (p.bodyExtra_m ?? 0);
  const noseLen = 0.45 * bodyLen;
  const keelLen = lt + noseLen * 0.5;
  // round 16: the airframe states the boom's length from its solid (airframe.js af_boom), so its line comes with the linkage items
  const keelItem = (p._linkageItems ?? []).find((q) => q.name === 'keel and tail boom');
  if (!keelItem) addStruct('keel and tail boom', keel, keel.mass_g * keelLen, `${keel.mass_g} g/m x ${keelLen.toFixed(3)} m`);
  const bodyW = p.bodyWidth_m ?? 0.06;
  const bodyH = p.bodyHeight_m ?? 0.07;
  // Round 4: the pod's centre line may sit below the wing hinge so the crank and gear train fit inside it.
  const bodyY = p.bodyY_m ?? 0;
  const shell = (Math.PI * 0.5 * (bodyW + bodyH) * bodyLen * 0.8);
  addStruct('fuselage film shell', film, (film.specs.areal_density_g_m2 ?? film.mass_g) * shell, `${shell.toFixed(3)} m2`);
  // round 16: the airframe states the formers (airframe.js formerGeometry), so their line comes with the linkage items
  const formerItem = (p._linkageItems ?? []).find((q) => q.name === 'fuselage formers');
  const formersG = formerItem ? formerItem.mass_g : 5 * bodyW * bodyH * 0.5 * 0.002 * ribDensity * 1000;
  if (!formerItem) addStruct('fuselage formers', rib, formersG, '5 off, 50% lightened, 2 mm');
  // ---- Drive
  const plate = byId(catalog, parts.drivePlate ?? 'plate-ec-2mm');
  const bellArea = 4 * (p.crank_mm ?? 12) * 25e-6 + 2 * 0.03 * 0.03;
  const crankPlateG = (plate.specs.areal_density_kg_m2 * 1000) * bellArea;
  // Round 6: with a sized metal linkage the crank, rockers, gearbox plates and couplers are its parts instead.
  const linkItems = p.linkage && p.joints ? p._linkageItems ?? [] : null;
  if (!linkItems) addStruct('crank, rockers and gearbox plates', plate, crankPlateG + (p.gearboxPlate_g ?? 8), `${plate.specs.areal_density_kg_m2} kg/m2 x ${bellArea.toExponential(2)} m2, plus gearbox cheek allowance`);
  // Couplers: pultruded rod or carbon tube, two
  const couplerRod = byId(catalog, parts.coupler ?? 'rod-ec-3mm');
  const bMm = p.coupler_mm ?? 70;
  if (!linkItems) addStruct('couplers, 2 off', couplerRod, (2 * couplerRod.mass_g * bMm) / 1000, `${couplerRod.mass_g} g/m x ${bMm} mm x 2`);
  for (const it of linkItems ?? []) structure.push({ ...it });
  const linkG = (re) => (linkItems ?? []).filter((it) => re.test(it.name)).reduce((a, it) => a + it.mass_g, 0);
  const couplerG = linkItems ? linkG(/^coupler links/) : (2 * couplerRod.mass_g * bMm) / 1000;
  // round 16: the keel and the formers come with the linkage items but have lumps of their own (keel and boom; fuselage
  // shell and formers), so they are not also counted in the gearbox's lump
  const drivePlatesG = linkItems ? linkG(/^(?!coupler links|keel and tail boom$|fuselage formers$)/) : crankPlateG + (p.gearboxPlate_g ?? 8);
  // Round 4 derived parts (p.derivedParts), each from catalogue materials with its basis stated:
  //  - the stroke servo's linkage to the two slotted rockers: two 2 mm carbon pushrods and a 2 mm carbon-plate yoke;
  //  - a pitot-static probe for the airspeed board, whose catalogue mass excludes the probe: a titanium outer tube with
  //    a carbon inner (total-pressure) tube. Its silicone pressure lines are counted in the 15% unlisted margin with
  //    the wiring (no catalogue source for tubing).
  let strokeLinkG = 0, pitotG = 0;
  if (p.derivedParts) {
    const rod2 = byId(catalog, 'rod-ec-2mm'), plate2 = byId(catalog, 'plate-ec-2mm');
    const pushL = p.strokePushrod_mm ?? 60;
    if (!p.fixedStroke) {
      strokeLinkG = 2 * rod2.mass_g * pushL / 1000 + plate2.specs.areal_density_kg_m2 * 1000 * 0.04 * 0.02;
      addStruct('stroke servo linkage: 2 pushrods and a yoke', rod2, strokeLinkG, `2 x ${pushL} mm of ${rod2.id} (${rod2.mass_g} g/m) + yoke 40 x 20 mm of plate-ec-2mm (${plate2.specs.areal_density_kg_m2} kg/m2)`);
    }
    const ti = byId(catalog, 'mtube-tigertitanium-ti3al2.5v-6.35x5.54'), cst = byId(catalog, 'tube-cst-pultruded-5x4');
    const probeL = p.pitotLength_mm ?? 100;
    pitotG = (ti.mass_g + cst.mass_g) * probeL / 1000;
    addStruct('pitot-static probe', ti, pitotG, `${probeL} mm of ${ti.id} (${ti.mass_g} g/m, static annulus) around ${probeL} mm of ${cst.id} (${cst.mass_g} g/m, total pressure); pressure lines in the 15% margin`);
  }
  // Twist drive. Legacy (round 2): the servo sits on the hand wing at 65% of its length, no linkage counted. Round 3
  // option: the servo sits at the elbow on the arm wing and turns the hand wing through a torque rod running out to
  // the tip; the rod's mass is on the wing. A horn linkage ratio k (wing twist per servo turn) trades the servo's
  // published torque for speed: stall torque / k and no-load speed x k at the wing.
  const twistAtElbow = p.twistServoAt === 'elbow';
  const twistRod = twistAtElbow && parts.twistRod ? byId(catalog, parts.twistRod) : null;
  // round 16: in the outer spar's bore, from the running clearance past the inner spar's end to the spar's end
  const twistRodLen = twistRod ? (WS?.spar_to_m ? WS.spar_to_m - elbow - (p.layout?.running_mm ?? 0.5) / 1000 : semi - elbow) : 0;
  if (twistRod) addStruct('twist torque rods, 2 off', twistRod, 2 * twistRod.mass_g * twistRodLen, `${twistRod.mass_g} g/m x ${twistRodLen.toFixed(3)} m x 2`);
  // ---- Parts
  const motorP = byId(catalog, parts.motor);
  const escP = byId(catalog, parts.esc);
  const batP = byId(catalog, parts.battery);
  const twistP = byId(catalog, parts.twistServo);
  const tailP = byId(catalog, parts.tailServo);
  // Round 5: a fixed stroke (p.fixedStroke, a pinned rocker with no slot) has no stroke servo.
  const strokeP = parts.strokeServo && !p.fixedStroke ? byId(catalog, parts.strokeServo) : null;
  const fcP = byId(catalog, parts.fc);
  const gpsP = byId(catalog, parts.gps);
  const aspdP = byId(catalog, parts.airspeed);
  const rxP = byId(catalog, parts.receiver);
  add(motorP, 1, 'flap motor');
  const fcBec = p.becFrom === 'fc';
  add(escP, 1, fcBec ? 'flap ESC (no BEC; the flight controller\'s 5 V BEC supplies the avionics)' : p.servoSupply === 'battery' ? 'flap ESC, its BEC supplies the avionics (servos on the pack)' : 'flap ESC, its BEC supplies the servos and avionics');
  add(batP, 1, 'battery');
  add(twistP, 2, 'wing twist servo');
  add(tailP, 2, 'tail servo');
  for (const q of tailMech?.parts ?? []) add(byId(catalog, q.id), q.count, q.role);
  if (strokeP) add(strokeP, 1, 'stroke amplitude servo');
  add(fcP, 1, 'flight controller with IMU and baro');
  // Round 6: the hardware variant the design needs (the F405-WMO's corrected current-sense PDB), on its BOM line.
  if (p.currentSense?.hardware && p.currentSense.part === fcP.id) bom[bom.length - 1].hardware = p.currentSense.hardware;
  add(gpsP, 1, 'GPS');
  add(aspdP, 1, 'airspeed sensor');
  add(rxP, 1, 'receiver');
  // Round 5: the sensors and supplies the controller's own functions read or need (SPEC ruling "Control functions need
  // real hardware"; servo supply ruling), each in the BOM with its published mass.
  for (const [key, role] of [['crankEncoder', 'crank angle encoder (governor, clamp, twist sync)'], ['encoderMagnet', 'crank encoder magnet'], ['powerModule', 'power module (battery voltage and current)'], ['servoRegulator', 'servo regulator']]) if (parts[key]) add(byId(catalog, parts[key]), 1, role);
  for (const g of parts.gears) add(byId(catalog, g), 1, 'gear');
  const nStages = Math.floor(parts.gears.length / 2);
  const hinged = (p.elbowGain ?? 0.5) > 0;
  let bearingLayout = null;
  if (p.joints) {
    // Round 5: every joint of the drive with its own rated part (src/design/joints.js; load path filled in below).
    const J = p.joints;
    const shafts = Array.from({ length: nStages - 1 }, (_, j) => J.shafts[j] ?? J.shafts[J.shafts.length - 1]);
    const rows = [[J.shoulder, 4, 'shoulder pivots, two per side'], [J.crankShaft, 2, 'crank shaft'], ...shafts.map((id, j) => [id, 2, `gear shaft ${j + 1}`]), [J.couplerEyes, 4, 'coupler eyes, two per coupler'], ...(J.motorOutboard ? [[J.motorOutboard, 1, 'motor outboard, on the motor shaft in the forward plate']] : [])];
    const merged = new Map();
    for (const [id, count, where] of rows) {
      const m = merged.get(id) ?? { count: 0, where: [] };
      m.count += count;
      m.where.push(`${where} (${count})`);
      merged.set(id, m);
    }
    for (const [id, m] of merged) {
      const part = byId(catalog, id);
      add(part, m.count, `${part.category === 'bushing' ? 'bushing' : part.category === 'rod_end' ? 'rod end' : 'bearing'}: ${m.where.join(', ')}`);
    }
    bearingLayout = rows.map(([id, count, where]) => ({ where, count, id }));
  } else {
    // Bearings (round 4, p.bearingLayout 'auto'): two per shoulder pivot, two per gearbox shaft (one shaft per stage:
    // the motor's own bearings carry the first pinion), two per elbow hinge if the hand wing is hinged (elbowGain > 0).
    bearingLayout = p.bearingLayout === 'auto' ? [{ where: 'shoulder pivots', count: 4 }, { where: `gearbox shafts (${nStages} shafts, crank included)`, count: 2 * nStages }, { where: 'elbow hinges', count: hinged ? 4 : 0, ...(hinged ? {} : { note: 'rigid elbow: the hand wing is not hinged (elbowGain 0)' }) }].map((b) => ({ ...b, id: parts.bearing })) : null;
    const nBearings = bearingLayout ? bearingLayout.reduce((a, b) => a + b.count, 0) : p.bearingCount ?? 8;
    add(byId(catalog, parts.bearing), nBearings, 'bearing');
  }
  const partsG = bom.reduce((a, b) => a + b.mass_g, 0);
  const structG = structure.reduce((a, b) => a + b.mass_g, 0);
  const marginG = MARGIN * (partsG + structG);
  const totalG = partsG + structG + marginG;

  // ---- Layout
  const tSpar = tubeSection(inner);
  const skinKgM2 = ((filmG + ribCount * ribMassEach) / 1000) / Sact;
  const sparKgPerM = (innerMass * elbow + outerMass * outer) / semi / semi / 1000 * 1; // length-weighted, kg/m
  const massWingOne = (innerMass + outerMass + leMass + teMass + filmG / 2 + (ribCount * ribMassEach) / 2) / 1000;
  const twistS = twistAtElbow ? wing.segments[0].length : wing.segments[0].length + 0.65 * outer;
  // Distribution for the flight model: spar + edges along the span by length, skin by area. Margin share on top.
  const marginFrac = 1 + MARGIN;
  const rodKg = twistRod ? (twistRod.mass_g * twistRodLen) / 1000 : 0;
  const wingMass = {
    spar_kg_per_m: (((innerMass + outerMass + leMass + teMass) / semi) / 1000) * marginFrac,
    skin_kg_per_m2: skinKgM2 * marginFrac,
    point: [
      { name: 'twist servo', s_m: twistS, mass_kg: (twistP.mass_g / 1000) * marginFrac, d_m: 0 },
      // The torque rod as two halves at the quarter points of its run.
      ...(twistRod ? [0.25, 0.75].map((q) => ({ name: 'twist torque rod', s_m: elbow + q * twistRodLen, mass_kg: 0.5 * rodKg * marginFrac, d_m: 0 })) : []),
    ],
    one_wing_kg: massWingOne * marginFrac + (twistP.mass_g / 1000) * marginFrac + rodKg * marginFrac,
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
  // Round 4: p.lumpPos { name: [x, y, z] } places a lump where the packing check put the real part (sims/avis-shared
  // model.js layoutParts), so the CG and inertia follow the parts as fitted in the body.
  // Round 17 (SPEC "Function completeness": mass properties from the drawn solids): each lump names the BOM lines it
  // carries (lump.lines, src/design/massprops.js); with p.massFromSolids its position and inertia are those of its
  // lines' drawn solids (p.lumpPos, p.lumpInertia per kg from fit.js / round11 fittedBuild), and a lump with no drawn
  // position is an error, not a placeholder.
  const fromSolids = !!p.massFromSolids && !p._massPass;
  const lump = (name, g, x, y = 0, z = 0, k = 0, lines = []) => {
    if (fromSolids && !p.lumpPos?.[name]) throw new Error(`mass from solids: lump '${name}' has no drawn position (p.lumpPos)`);
    const J = fromSolids ? p.lumpInertia?.[name] : null, m = (g / 1000) * marginFrac;
    lumps.push({ name, mass_kg: m, pos: p.lumpPos?.[name] ?? [x, y, z], gyration_m: J ? 0 : k, ...(J ? { inertia_kg_m2: J.map((v) => v * m) } : {}), lines });
  };
  const motorX = 0.04;
  const linkNames = (re) => (linkItems ?? []).filter((it) => re.test(it.name)).map((it) => it.name);
  lump('flap motor', motorP.mass_g, motorX, 0.0, 0, 0, [motorP.id]);
  const gearRows = bom.filter((b) => b.role === 'gear' || /^(bearing|bushing|rod end)/.test(b.role));
  lump('gears and bearings', gearRows.reduce((a, b) => a + b.mass_g, 0), 0.015, 0.0, 0, 0, [...new Set(gearRows.map((b) => b.id))]);
  lump('crank, rockers, plates', drivePlatesG, 0.01, 0.01, 0, 0, linkItems ? linkNames(/^(?!coupler links|keel and tail boom$|fuselage formers$)/) : ['crank, rockers and gearbox plates']);
  lump('couplers', couplerG, 0.0, 0.0, 0, 0, linkItems ? linkNames(/^coupler links/) : ['couplers, 2 off']);
  lump('ESC', escP.mass_g, 0.07, -0.01, 0, 0, [escP.id]);
  lump('flight controller', fcP.mass_g, 0.0, -0.01, 0, 0, [fcP.id]);
  lump('receiver and airspeed', rxP.mass_g + aspdP.mass_g, 0.11, 0.0, 0, 0, [rxP.id, aspdP.id]);
  lump('GPS', gpsP.mass_g, -0.03, 0.03, 0, 0, [gpsP.id]);
  const tmPart = (re) => (tailMech?.parts ?? []).filter((q) => re.test(q.role));
  const tmG = (list) => list.reduce((a, q) => a + byId(catalog, q.id).mass_g * q.count, 0), tmS = (re) => (tailMech?.structure ?? []).filter((q) => re.test(q.name));
  lump('tail servos', 2 * tailP.mass_g + tmG(tmPart(/servo arms/)), -0.08, 0.0, 0, 0, [tailP.id, ...tmPart(/servo arms/).map((q) => q.id)]);
  // round 17: the tail's pushrods with their plugs, rod ends and screws, a lump of their own along the boom
  if (tailMech) lump('tail pushrods', tmS(/pushrod/).reduce((a, q) => a + q.mass_g, 0) + tmG(tmPart(/rod ends|linkage screws/)), -0.5, 0.0, 0, 0, [...tmS(/pushrod/).map((q) => q.name), ...tmPart(/rod ends|linkage screws/).map((q) => q.id)]);
  if (strokeP) lump('stroke servo', strokeP.mass_g + strokeLinkG, -0.03, 0.0, 0, 0, [strokeP.id]);
  // Round 5: the crank encoder sits on the crank axis with its magnet; a separate power module or servo regulator sits
  // with the flight controller.
  const sensorG = ['crankEncoder', 'encoderMagnet'].reduce((a, k) => a + (parts[k] ? byId(catalog, parts[k]).mass_g : 0), 0);
  if (sensorG > 0) lump('crank encoder', sensorG, 0.0, -0.05, 0, 0, ['crankEncoder', 'encoderMagnet'].filter((k) => parts[k]).map((k) => parts[k]));
  const supplyG = ['powerModule', 'servoRegulator'].reduce((a, k) => a + (parts[k] ? byId(catalog, parts[k]).mass_g : 0), 0);
  if (supplyG > 0) lump('power module and regulator', supplyG, -0.01, -0.03, 0, 0, ['powerModule', 'servoRegulator'].filter((k) => parts[k]).map((k) => parts[k]));
  if (pitotG) lump('pitot-static probe', pitotG, Math.max(0.45 * bodyLen, 0.14) + 0.04, 0.0, 0, 0, ['pitot-static probe']);
  if (keelItem) lump('keel and boom', keelItem.mass_g, keelItem.lump.pos[0], keelItem.lump.pos[1], keelItem.lump.pos[2], keelItem.lump.gyration_m, ['keel and tail boom']);
  else lump('keel and boom', keel.mass_g * keelLen, -lt / 2 + noseLen * 0.25, 0.0, 0, 0.29 * keelLen, ['keel and tail boom']);
  lump('fuselage shell and formers', (film.specs.areal_density_g_m2 ?? film.mass_g) * shell + formersG, -0.05, 0.0, 0, 0.3 * bodyLen, ['fuselage film shell', 'fuselage formers']);
  if (tailMech) lump('tail surfaces', tailFilm + tailRibG + tmS(/^(?!tail pushrod)/).reduce((a, q) => a + q.mass_g, 0) + tmG(tmPart(/bushings|washers/)), -lt, p.tailY_m ?? 0.02, 0, 0, ['tail film, both faces', 'tail ribs, 8 off', ...tmS(/^(?!tail pushrod)/).map((q) => q.name), ...tmPart(/bushings|washers/).map((q) => q.id)]);
  else lump('tail surfaces', tailFilm + tailSparG + tailRibG, -lt, p.tailY_m ?? 0.02, 0, 0, p.tailLines === 'split' ? ['tail film, both faces', 'tail rods, 4 off', 'tail ribs, 8 off'] : ['tail surfaces, film, spar and ribs']);
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
  // the battery placed for the CG target (rounds 1-16, before its drawn place is known); round 17 (p.massFromSolids): at
  // its drawn place, like every lump, so the CG is the drawn one and the target only states the design's intent
  const xBattery = (xCGtarget * (sumM + battKg) - sumMx) / battKg;
  if (fromSolids && !p.lumpPos?.battery) throw new Error('mass from solids: the battery has no drawn position (p.lumpPos)');
  const JB = fromSolids ? p.lumpInertia?.battery : null;
  lumps.push({ name: 'battery', mass_kg: battKg, pos: fromSolids ? p.lumpPos.battery : [xBattery, -0.005, 0], gyration_m: JB ? 0 : 0.02, ...(JB ? { inertia_kg_m2: JB.map((v) => v * battKg) } : {}), lines: [batP.id] });
  const total = sumM + battKg;
  // Crank-rocker sized from amplitude
  const ampNom = (p.amp_deg ?? 25) * DEG;
  const a = (p.crank_mm ?? 12) / 1000;
  const b0 = (p.coupler_mm ?? 70) / 1000;
  const d0 = p.groundLink_mm ? p.groundLink_mm / 1000 : Math.max(0.05, b0);
  const ampMax = p.fixedStroke ? ampNom : (p.ampMax_deg ?? Math.min((p.amp_deg ?? 25) * 1.6, 60)) * DEG;
  // Round 5: a fixed stroke (no slot, no stroke servo) has one amplitude, the nominal one.
  const ampMin = p.fixedStroke ? ampNom : ampNom * 0.6;
  const drive = { kind: 'crank-rocker', a, b: b0, d: d0, ampMin, ampMax, ampNominal: ampNom, phiMean: (p.phiMean_deg ?? 5) * DEG, gearRatio: p.gearRatio, gearEfficiency: p.gearEff ?? 0.85, crankFriction_Nm: 0.01, gears: parts.gears, ...(p.pinLayout ? { pinLayout: p.pinLayout } : {}), ...(bearingLayout ? { bearings: bearingLayout } : {}), ...(p.fixedStroke ? { fixedStroke: true } : {}), ...(p.governor ? { governor: p.governor } : {}), ...(Number.isFinite(p.escCurrentClamp_A) ? { escCurrentClamp_A: p.escCurrentClamp_A } : {}) };
  const motor = motorModel(motorP);
  // Braking (round 4). The flight model's default lets the drive brake at only half the motor's rating (an assumption
  // with no source), so on the upstroke the wing back-drove the crank to twice its commanded speed and the reversal at
  // the top of the stroke set the M7 root bending and twist torque peaks. An ESC whose own documentation states active
  // freewheeling (synchronous rectification) lets the motor current reverse whenever the back-EMF exceeds the applied
  // voltage; the drive firmware bounds it at the lower of the ESC's and motor's continuous ratings. The energy is still
  // never credited to the battery (SPEC no-regeneration ruling), only the braking torque is real.
  const af = p.escActiveFreewheeling;
  if (af && (af.esc === escP.id || (af.family && escP.id.startsWith(af.family)))) {
    motor.regenLimit_A = -Math.min(motor.I_cont ?? Infinity, escP.specs.continuous_current_a ?? Infinity);
    motor.regenLimit_basis = `ESC documentation: ${af.quote} (${af.source}); braking current bounded at the lower continuous rating`;
  }
  const batt = applyChargeLimit(batteryModel(batP), p.chargeLimit);
  // SPEC ruling after catalogue round 2: the design states its servo supply voltage and the BEC that provides it
  // (here the flap ESC's built-in BEC, in the BOM with the ESC), and no servo figure is extrapolated outside its rows.
  // Round 5 (p.becFrom 'fc'): the ESC has no BEC; the flight controller's own 5 V regulator feeds the avionics.
  const becPart = fcBec ? byId(catalog, parts.fc) : escP;
  const bec = becOf(becPart);
  const cellsOk = escCellsCover(escP, batP.specs.cells_series);
  if (cellsOk === false) throw new Error(`ESC ${escP.id} is not rated for a ${batP.specs.cells_series}S pack (${escP.specs.cells_text})`);
  // Servo supply. 'bec' (rounds 1-3): the ESC's BEC at its published voltage. 'battery' (round 4): HV servos straight
  // on the flight pack, as is usual for HV servos on 2S; no regulator, so the supply is the pack itself, in the BOM with
  // its mass. Its voltage moves with charge and load, so torque and speed are taken at the lowest terminal voltage in
  // use (20% charge, the M2 reserve, sagging at the pack's continuous rating) and stall current at full charge.
  // Round 12: servos behind a catalogue step-down regulator (p.servoSupply 'regulator', parts.servoRegulator), so the
  // pack's own voltage no longer has to sit inside the servo rows (3S packs become eligible).
  const regP = p.servoSupply === 'regulator' ? byId(catalog, parts.servoRegulator) : null;
  if (p.servoSupply === 'regulator' && !regP) throw new Error('servoSupply regulator needs parts.servoRegulator');
  const supply = regP ? regulatorServoSupply(regP, batt) : batteryServoSupply(p.servoSupply, batt);
  const supplyV = supply ? supply.performance_V : (p.servoSupply_V ?? bec.volts);
  if (!Number.isFinite(supplyV)) throw new Error(`ESC ${escP.id} publishes no BEC voltage: no servo supply`);
  const esc = { id: escP.id, I_cont: escP.specs.continuous_current_a, bec_A: bec.amps, bec_V: bec.volts, bec_kind: bec.kind, eta: 0.95, eta_assumed: true, regenerative: false, regenerative_basis: 'no regenerative operation stated in the ESC documentation read for the catalogue', cells_text: escP.specs.cells_text ?? null };
  const servoSpec = (part, v) => {
    const s = servoAt(part, v);
    if (!s) throw new Error(`servo ${part.id} has no published row covering ${v} V (no extrapolation)`);
    let stallA = s.stall_A;
    if (supply) {
      const range = servoVoltageRange(part);
      if (!range || supply.current_V > range[1] + 1e-9) throw new Error(`servo ${part.id}: published voltage range does not reach the full-charge ${supply.current_V} V`);
      stallA = servoStallCurrentConservative(part, supply.current_V);
    }
    if (!Number.isFinite(stallA)) throw new Error(`servo ${part.id} has no published stall current (SPEC M8)`);
    return { id: part.id, stall_Nm: s.stall_Nm, speed_rad_s: s.speed_rad_s, stall_A: stallA, tau_s: 0.02, idle_A: part.specs.idle_current_a ?? 0, volts: s.volts, ...(supply ? { current_volts: supply.current_V } : {}), mass_kg: part.mass_g / 1000 };
  };
  // Twist linkage ratio k: the wing sees the servo's published torque / k and its no-load speed x k. Current is from
  // the torque at the servo (wing torque x k), which stall_A x (wing torque / (stall / k)) gives unchanged.
  const kTw = p.twistLinkageRatio ?? 1;
  if (!(kTw > 0)) throw new Error('twistLinkageRatio must be positive');
  const twistSpec = servoSpec(twistP, supplyV);
  const twistAtWing = kTw === 1 ? twistSpec : { ...twistSpec, stall_Nm: twistSpec.stall_Nm / kTw, speed_rad_s: twistSpec.speed_rad_s * kTw, linkage_ratio: kTw, servo_stall_Nm: twistSpec.stall_Nm, servo_speed_rad_s: twistSpec.speed_rad_s, linkage_note: `horn linkage ${kTw}:1 (wing twist per servo turn); torque, speed and stall current at the wing, from the servo's published row` };
  const servos = {
    twist: twistAtWing,
    tail: servoSpec(tailP, supplyV),
    ...(strokeP ? { stroke: { ...servoSpec(strokeP, supplyV), amp_rate_rad_s: 0.5 } } : {}),
  };
  // Body geometry: pod from the nose (ahead of the battery and receiver) back bodyLen, then the keel tube as a boom to
  // the tail's trailing edge. The tunnels, page and film build from this loft (src/physics/geometry.js).
  // round 17: the nose where the design states it (p.noseX_m: its nose block, round 17), else from the CG-placed battery
  const noseX = p.noseX_m ?? Math.max(0.45 * bodyLen, xBattery + 0.05, 0.14);
  const loft = podAndBoomLoft({ noseX_m: noseX, podLength_m: bodyLen, width_m: bodyW, height_m: bodyH, boomOd_m: keel.specs.od_mm / 1000, tailEndX_m: -lt - 0.75 * chordT, y_m: bodyY, boomY_m: 0 });
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
    bottom_y_m: bodyY - bodyH / 2,
    centre_y_m: bodyY,
  };
  // Spar check data; loads come from flight.
  const allow = tubeAllowable(inner);
  const allowOuter = tubeAllowable(outerSpar);
  const sparInner = { ...tubeSection(inner), id: inner.id, allowable_MPa: allow.sigma_MPa, allowable_basis: allow.basis, mass_g_per_m: inner.mass_g };
  const sparOuter = { ...tubeSection(outerSpar, specSheet), id: outerSpar.id, allowable_MPa: allowOuter.sigma_MPa, allowable_basis: allowOuter.basis, mass_g_per_m: outerGm, section_basis: tubeSpec(outerSpar, specSheet).basis };
  const gearStages = stages(parts.gears, catalog);

  const d = {
    name: cat.name, label: cat.label, candidate: p.candidate, reference: cat.reference, claim: CLAIM,
    params: (({ _linkageItems, ...rest }) => rest)(p), // the linkage items are the structure list's, not a parameter
    atmosphere: { altitude_m: DESIGN_ALTITUDE_M, temperature_C: DESIGN_TEMPERATURE_C, rho_kg_m3: atm.rho, mu_Pa_s: atm.mu },
    wing,
    geometry: { span_m: wingSpan(w), area_m2: Sact, AR: wingSpan(w) ** 2 / Sact, mac_m: macv, chord_root_m: cr, chord_elbow_m: ce, chord_tip_m: ct, tail_arm_m: lt, wing_loading_N_m2: (total * 9.80665) / Sact, x_np_m: xNP, x_cg_m: xCGtarget, static_margin: p.staticMargin ?? 0.1 },
    drive,
    // round 16 (p.tailY_m): the V-tail's roots on the boom's sides, at its height
    tail: { x_m: -lt, y_m: p.tailY_m ?? 0.02, z_m: 0.005, span_each_m: spanEach, chord_root_m: chordT, taper: p.tailTaper ?? 0.7, dihedral_rad: dih, incidence_rad: (p.tailIncidence_deg ?? 0) * DEG, polar: 'thin-plate', AR_each: ARtail, lift_factor: ARtail / (ARtail + 2) * 1.05, kEps: 1.0, area_total_m2: Stail, deflection_max_rad: 30 * DEG },
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
    structure: { items: structure, spar_inner: sparInner, spar_outer: sparOuter, gear_stages: gearStages, coupler: couplerSpec(couplerRod, bMm), ...(twistRod ? { twist_rod: twistRodSpec(twistRod, twistRodLen) } : {}) },
    bom: { parts: bom, structure, parts_g: partsG, structure_g: structG, margin_g: marginG, total_g: totalG },
    electrical: { ...electricalOf({ supply, supplyV, batP, regP, becPart, bec, fcBec, extraA: p.avionicsExtra_A ?? 0, extraNote: p.avionicsExtraNote }), ...(p.currentSense ? { current_sense: p.currentSense } : {}) },
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
  if (p.joints) {
    // Round 5: the drive's load path, which bearing, bushing or rod end reacts which force (SPEC ruling).
    const jp = driveJoints(d, catalog, p.joints);
    d.structure.joints = jp.joints;
    d.structure.joints_unchecked = jp.unchecked;
    d.structure.joints_layout = jp.layout;
    // Round 9: the gear ratings under KHK's stated conditions (SPEC ruling after physics round 8), at the design's
    // stated running conditions (p.gearRating) and the stage planes of the layout.
    if (p.gearRating) {
      d.structure.gear_stages = rateStages(d.structure.gear_stages, catalog, jp.layout, p.gearRating);
      d.drive.gear_rating_conditions = p.gearRating;
    }
    if (p.linkage) {
      // Round 6: every loaded part of the drive (src/design/linkage.js), checked in M7 (src/physics/members.js).
      const lm = driveMembers(d, catalog, p.linkage, driveLayout(d, catalog), jp.joints);
      // Round 11: each support's housing length, so a bushing is judged on the length its seat actually holds.
      d.structure.joints = jointHousings(jp.joints, p.linkage, catalog);
      d.structure.members = lm.members;
      d.structure.members_unchecked = lm.unchecked;
      d.structure.coupler = lm.coupler;
      if (lm.airframe) d.structure.airframe = lm.airframe;
    }
  }
  return d;
}

/**
 * Servo supply straight from the flight pack (round 4), or null for the BEC. Performance voltage: the lowest terminal
 * voltage in use, the rested voltage at 20% charge (the M2 reserve) less the sag at the pack's continuous rating.
 * Current voltage: full charge, rested (4.2 V per cell plus the pack's published-nominal shift).
 */
export function batteryServoSupply(kind, batt) {
  if (kind !== 'battery') return null;
  const ocv20 = batteryOcv(batt, 0.2);
  const sag = (batt.I_cont ?? 0) * batt.R_int;
  const performance_V = +(ocv20 - sag).toFixed(3);
  // A fresh LiPo charge ends at 4.20 V per cell (the model's rested curve tops out at 4.18). Round 5: a pack charged to a
  // limited voltage (battery.charge_limit_V_per_cell) can present no more than that.
  const limit = batt.charge_limit_V_per_cell;
  const current_V = Number.isFinite(limit) ? +(batt.cells * limit).toFixed(3) : +Math.max(batteryOcv(batt, 1), batt.cells * (4.2 + (batt.cell_shift_V ?? 0))).toFixed(3);
  return { performance_V, current_V, rule: `torque and speed at ${performance_V} V: rested ${ocv20.toFixed(2)} V at 20% charge less ${sag.toFixed(2)} V sag at the pack's ${batt.I_cont} A continuous rating; stall current at the ${Number.isFinite(limit) ? `charge-limited ${current_V} V (${limit} V per cell)` : `full-charge ${current_V} V`}` };
}

/**
 * Round 12: servo supply through a catalogue step-down regulator (category 'regulator'). Its input is the pack, from
 * the lowest terminal voltage in use (as batteryServoSupply: 20% charge, sagging at the continuous rating) to full
 * charge; both must lie inside the regulator's published input range. Torque and speed are taken at the lowest output:
 * the published voltage less its accuracy, or the lowest input less the dropout read at the continuous current,
 * whichever is lower; stall current at the published voltage plus its accuracy. The battery sees the servos' power at
 * that highest output through the lower end of the published typical efficiency.
 */
export function regulatorServoSupply(reg, batt) {
  const s = reg.specs;
  const pack = batteryServoSupply('battery', batt);
  if (pack.performance_V < s.min_input_v) throw new Error(`regulator ${reg.id}: the pack's lowest terminal voltage ${pack.performance_V} V is below its published minimum input ${s.min_input_v} V`);
  if (pack.current_V > s.max_input_v) throw new Error(`regulator ${reg.id}: full charge ${pack.current_V} V is above its published maximum input ${s.max_input_v} V`);
  const lo = s.output_v * (1 - s.output_accuracy_frac), hi = s.output_v * (1 + s.output_accuracy_frac);
  // round 17: a regulator whose page gives no dropout figure is held to its published minimum input, which includes it
  const performance_V = +Math.min(lo, Number.isFinite(s.dropout_v_max) ? pack.performance_V - s.dropout_v_max : Infinity).toFixed(3);
  const current_V = +hi.toFixed(3);
  return { performance_V, current_V, output_V: s.output_v, I_cont_A: s.continuous_current_a, eta: s.efficiency_min_frac, input_V: [pack.performance_V, pack.current_V], rule: `regulated ${s.output_v} V +-${(100 * s.output_accuracy_frac).toFixed(0)}%: torque and speed at ${performance_V} V (${Number.isFinite(s.dropout_v_max) ? `the lower of ${lo.toFixed(3)} V and the lowest input ${pack.performance_V} V less ${s.dropout_v_max} V dropout` : `${lo.toFixed(3)} V; the lowest input ${pack.performance_V} V is above the published ${s.min_input_v} V minimum input`}), stall current at ${current_V} V; battery draw at ${current_V} V output through ${s.efficiency_min_frac} efficiency (lower end of the published typical range); continuous ${s.continuous_current_a} A (published)` };
}

/** The design's electrical block: servo supply and the 5 V regulator for the avionics (ESC's BEC or the FC's). */
function electricalOf({ supply, supplyV, batP, regP = null, becPart, bec, fcBec, extraA, extraNote }) {
  const where = fcBec ? 'built into the flight controller; its mass is in the BOM with the FC' : 'built into the flap ESC; its mass is in the BOM with the ESC';
  const avionics = { avionics_A_5V: +(0.35 + extraA).toFixed(4), avionics_note: `FC, GPS, receiver at 5 V, 0.35 A assumed (catalogue lists no current draw)${extraA ? ` plus ${extraA} A published (${extraNote ?? 'sensors'})` : ''}` };
  const becBlock = (carries) => ({ part: becPart.id, volts: bec.volts, amps: bec.amps, kind: bec.kind, published: bec.text, carries, note: where });
  if (regP) return { ...avionics, servo_source: 'regulator', servo_supply_V: supplyV, servo_supply: { part: regP.id, ...supply }, bec: becBlock('avionics only') };
  return supply
    ? { ...avionics, servo_source: 'battery', servo_supply_V: supplyV, servo_supply: { part: batP.id, ...supply }, bec: becBlock('avionics only') }
    : { ...avionics, servo_source: 'bec', servo_supply_V: supplyV, bec: becBlock('servos and avionics') };
}

/**
 * Round 5 (servo-voltage ruling): a pack charged only to lim.V_per_cell. Its capacity is derated to the lower of the
 * model's rested curve at that voltage (the state of charge where batteryOcv reaches it) and any published figure
 * (lim.published_fraction with its source), so M2 counts only what the pack holds.
 */
export function applyChargeLimit(batt, lim) {
  if (!lim) return batt;
  // The limit is the charger's terminal voltage per cell; the rested curve (shifted for Li-ion) is searched for it.
  const vPack = batt.cells * lim.V_per_cell;
  let lo = 0, hi = 1;
  for (let i = 0; i < 60; i++) {
    const m = 0.5 * (lo + hi);
    if (batteryOcv(batt, m) < vPack) lo = m; else hi = m;
  }
  const model = 0.5 * (lo + hi);
  const pub = Number.isFinite(lim.published_fraction) ? lim.published_fraction : null;
  const fraction = pub === null ? model : Math.min(model, pub);
  return { ...batt, charge_limit_V_per_cell: lim.V_per_cell, charge_fraction: +fraction.toFixed(4), charge_basis: `charged to ${lim.V_per_cell} V per cell (${vPack.toFixed(2)} V): capacity fraction ${fraction.toFixed(3)}, the lower of the model's rested curve (${model.toFixed(3)})${pub === null ? '' : ` and the published ${pub} (${lim.source})`}` };
}

/**
 * Coupler section for the M7 check. A rod (round 2) keeps its fields: modulus and compressive strength at the lower
 * of the published ranges. A tube adds its bore, and its published strength by the SPEC M7 rules (tubeAllowable).
 */
export function couplerSpec(part, lengthMm) {
  const s = part.specs;
  const base = { id: part.id, od_m: s.od_mm / 1000, length_m: lengthMm / 1000 };
  if (part.category === 'carbon_rod') return { ...base, E_GPa: s.tensile_modulus_lengthways_gpa_range?.[0] ?? null, compressive_MPa: s.compressive_strength_lengthways_mpa_range?.[0] ?? null };
  const a = tubeAllowable(part);
  return { ...base, id_m: (s.id_mm ?? 0) / 1000, E_GPa: s.tensile_modulus_lengthways_gpa ?? a.E_GPa ?? null, allowable_MPa: a.sigma_MPa, allowable_basis: a.basis };
}

/** Twist torque rod or tube for the M7 torsion check (structure.js torsionCheck). */
export function twistRodSpec(part, length_m) {
  const s = part.specs;
  const metal = part.category === 'metal_tube';
  return { id: part.id, od_m: s.od_mm / 1000, id_m: (s.id_mm ?? 0) / 1000, length_m, kind: metal ? 'metal' : 'composite', yield_MPa: metal ? s.yield_strength_mpa ?? null : null };
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
    const ps = pin.specs, gs = gear.specs;
    // Round 5: materials and mesh geometry for the mixed-material ruling (src/physics/structure.js mixedMeshCheck).
    out.push({ pinion: pin.id, gear: gear.id, ratio: gs.teeth / ps.teeth, pinion_allowable_Nm: minRating(pin), gear_allowable_Nm: minRating(gear), pinion_material: ps.material ?? null, gear_material: gs.material ?? null, pinion_rp_m: (ps.module_mm * ps.teeth) / 2000, gear_rp_m: (gs.module_mm * gs.teeth) / 2000, pinion_face_m: (ps.face_width_mm ?? 0) / 1000, gear_face_m: (gs.face_width_mm ?? 0) / 1000, alpha_rad: ((gs.pressure_angle_deg ?? 20) * Math.PI) / 180 });
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
