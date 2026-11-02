// SmartBird (Send et al., ICAS 2012) as a design file for V4: published geometry, mass and kinematics, with every
// unpublished quantity filled by a stated assumption. Shared by the V4 gate and its diagnostics.
import { buildDesign } from '../../design/build.js';

const DEG = Math.PI / 180;

export function smartbirdPublished(bench) {
  return {
    span: bench.geometry_and_mass.span_m.values[0].value, // 2 m
    mass: bench.geometry_and_mass.mass_kg.values[0].value, // 0.48 kg with battery
    area: bench.geometry_and_mass.wing_area_m2.value, // 0.5 m2
    freq: bench.kinematics.flap_frequency_hz.values[0].value, // 2 Hz
    target: bench.v4_power_targets.measured_free_flight, // 18 W at 4.7 m/s
    design5: bench.v4_power_targets.design_point, // 23 W at 5 m/s
    servoBias: bench.v4_power_targets.servo_bias_power_w.value, // 5 W, measured in passive flight
    netPower: bench.v4_power_targets.net_flapping_power_w.value, // 13 W
    plunge: bench.kinematics.stroke_2d_section.plunge_amplitude_m, // 0.3 m, read from a figure
    pitchDeg: bench.kinematics.torsion.pitch_amplitude_deg_typical, // 9 deg, read from a figure
    gear: parseFloat(String(bench.kinematics.transmission.gear_reduction).split(':')[1]), // 45
    rho: bench.aerodynamics_and_efficiency.air_density_used_kg_m3.value, // 1.2
  };
}

export const SMARTBIRD_ASSUMPTIONS = [
  'Kinematics imposed, not trimmed: 2 Hz, 0.3 m plunge and 9 deg pitch amplitude applied to one wing section, the 3/4-semispan station, as the paper\'s typical 2D section (both values read from its Fig. 2A).',
  'Planform: two segments with the elbow at half the semispan, taper 0.6, outer segment stroke gain 0.5 (the paper gives a 1:3 hinge transmission, not the outer stroke).',
  'Stroke law: sinusoidal; SmartBird\'s crank and hinge linkage geometry is not published.',
  'Twist: sinusoidal, plunge 90 deg ahead of pitch (the paper\'s stated convention); the paper\'s own schedule is partially linear and its design point uses 100 deg.',
  'Twist distribution: zero at the root, 0.3 of the command at the elbow, full at the tip, linear between; the 3/4-semispan station carries 0.65 of the command.',
  'Section: our E387 fit with the film drag factor 1.2 stands in for the NACA 7412 the paper names; no polar of either SmartBird section is published.',
  'Leading-edge suction efficiency as in the physics model (src/physics/strip.js SUCTION), from an independent source or an explicit assumption, never fitted to SmartBird.',
  'Structure: spars, ribs and film sized by our builder; body lumps scaled so the total is the published 0.48 kg.',
  'Body held at 4.7 m/s, level, pitch and tail solved for vertical force and pitching moment balance; the cycle-mean streamwise force is reported, not trimmed.',
  'Drive: E-flite Park 370 1080 class motor with the paper\'s 1:45 reduction at gear efficiency 0.85, ESC efficiency 0.95, 3S 1000 mAh; SmartBird\'s own motor, gearbox and battery are not published.',
  'No regeneration credit (SPEC ruling): the drive\'s battery power is the cycle mean of max(0, power).',
  'Servo bias: the paper\'s measured 5 W (passive flight) is added; twist servo power above idle comes from our servo model (stall current x torque / stall torque) on a catalogue servo standing in for SmartBird\'s.',
  'Avionics 0.35 A at 5 V through a BEC of efficiency 0.85, as in our designs; SmartBird\'s avionics draw is not published and may be inside the 5 W bias.',
  'Air density 1.2 kg/m3 as the paper used.',
];

/** Station on the semispan whose plunge and pitch are the paper's 2D section values. */
export const SECTION_FRAC = 0.75;

export function smartbirdDesign(catalog, bench, { sectionFrac = SECTION_FRAC } = {}) {
  const P = smartbirdPublished(bench);
  const params = {
    candidate: 'larus', span_m: P.span, AR: (P.span * P.span) / P.area, taper: 0.6, elbowFrac: 0.5, elbowGain: 0.5, amp_deg: 20, ampMax_deg: 34, phiMean_deg: 0,
    gearRatio: P.gear, staticMargin: 0.1, sparFrac: 0.35, crank_mm: 8, coupler_mm: 70,
    parts: {
      motor: 'motor-eflite-park370-1080', esc: 'esc-hobbywing-skywalker-20a-v2-mini', battery: 'batt-gensace-1000-3s-25c',
      twistServo: 'servo-kst-x06-v6', tailServo: 'servo-kst-x06-v6', strokeServo: 'servo-kst-x06-v6',
      fc: 'fc-ark-v6x', gps: 'gps-holybro-m10', airspeed: 'aspd-matek-4525', receiver: 'rx-matek-elrs-r24-p6',
      sparInner: 'tube-ec-woven-16x14', sparOuter: 'tube-ec-woven-8x6', leRod: 'rod-ec-2mm', teRod: 'rod-ec-1-5mm',
      ribMaterial: 'balsa-wood-database', film: 'film-oracover-air-light', keel: 'tube-ec-woven-8x6',
      gears: ['gear-khk-ds0.8-12', 'gear-khk-ds0.8-80', 'gear-khk-ds0.8-12', 'gear-khk-ds0.8-80'], bearing: 'bearing-skf-624-2z',
    },
  };
  const d = buildDesign(params, catalog);
  d.name = 'smartbird-v4';
  d.rho_override = P.rho;
  d.atmosphere.rho_kg_m3 = P.rho;
  d.drive.gearRatio = P.gear;
  // Scale body lumps so the total matches the published mass; wing mass stays as sized.
  const wingKg = 2 * d.mass.wing.one_wing_kg;
  const bodyKg = d.mass.lumps.reduce((a, l) => a + l.mass_kg, 0);
  const k = (P.mass - wingKg) / bodyKg;
  for (const l of d.mass.lumps) l.mass_kg *= k;
  d.mass.total_kg = P.mass;
  // Section station s on the semispan (from the hinge). Plunge y = L1 sin A + (s - L1) sin((1 + g) A), g = 0.5.
  const L1 = d.wing.segments[0].length;
  const L2 = d.wing.segments[1].length;
  const s = sectionFrac * (L1 + L2);
  const g = d.wing.segments[1].gain;
  let A = 0.2;
  for (let i = 0; i < 60; i++) {
    const y = L1 * Math.sin(A) + Math.max(0, s - L1) * Math.sin((1 + g) * A);
    A += (P.plunge - y) / (L1 * Math.cos(A) + (1 + g) * Math.max(0, s - L1) * Math.cos((1 + g) * A));
  }
  // Twist weight at that station: linear 0 -> twist_out on the inner segment, twist_in -> twist_out on the outer.
  const seg0 = d.wing.segments[0], seg1 = d.wing.segments[1];
  const wgt = s <= L1 ? seg0.twist_in + (seg0.twist_out - seg0.twist_in) * (s / L1) : seg1.twist_in + (seg1.twist_out - seg1.twist_in) * ((s - L1) / L2);
  const dyn = (P.pitchDeg * DEG) / wgt;
  d.drive = { ...d.drive, kind: 'sine', phiMean: 0, ampNominal: A, ampMin: A * 0.5, ampMax: Math.min(A * 1.6, 50 * DEG) };
  d.cruise = { V: P.target.speed_m_s, amp: A, dyn, sym: 0, lead: 0 };
  return { design: d, published: P, kinematics: { amp_rad: A, amp_deg: A / DEG, dyn_rad: dyn, dyn_deg: dyn / DEG, section_s_m: s, section_twist_weight: wgt, freq_hz: P.freq } };
}
