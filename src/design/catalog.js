// Reading the parts catalogue with no assumption it is complete. All accessors return null, never a guess.
// Pure: catalog is passed in, so this runs in node and in a browser.

export function byId(catalog, id) {
  const p = catalog.find((x) => x.id === id);
  if (!p) throw new Error(`catalogue has no part ${id}`);
  return p;
}
export function ofCategory(catalog, cat) {
  return catalog.filter((p) => p.category === cat);
}

/** SPEC: parts flagged sold out or end-of-life cannot go in a winning BOM. Looks at the stock notes only. */
export function isEligible(part) {
  const s = part.specs ?? {};
  const notes = [s.stock_note, s.availability, s.availability_note, s.status, s.eol_note].filter(Boolean).join(' ');
  return !/sold.?out|out of stock|end.?of.?life|discontinu|obsolete/i.test(notes);
}

/** kg per unit of the part's mass basis, read from specs.mass_basis as SPEC requires. */
export function massBasis(part) {
  return part.specs?.mass_basis ?? 'per_part';
}

/** Interpolate a servo's speed and stall-style torque at a supply voltage, from its published rows. */
export function servoAt(part, volts) {
  const rows = [...(part.specs.performance ?? [])].sort((a, b) => a.voltage_v - b.voltage_v);
  if (!rows.length) return null;
  const v = Math.max(rows[0].voltage_v, Math.min(rows[rows.length - 1].voltage_v, volts));
  let a = rows[0], b = rows[rows.length - 1];
  for (let i = 1; i < rows.length; i++) {
    if (v <= rows[i].voltage_v) {
      a = rows[i - 1];
      b = rows[i];
      break;
    }
  }
  const f = b.voltage_v === a.voltage_v ? 0 : (v - a.voltage_v) / (b.voltage_v - a.voltage_v);
  const torque = a.torque_nm + f * (b.torque_nm - a.torque_nm);
  const secPer60 = a.speed_s_per_60deg + f * (b.speed_s_per_60deg - a.speed_s_per_60deg);
  // Published stall current if there is one, else null. The caller must not invent it.
  const sc = part.specs.stall_current;
  let stallA = null;
  if (Array.isArray(sc) && sc.length) {
    stallA = sc.reduce((p, q) => (Math.abs(q.voltage_v - volts) < Math.abs(p.voltage_v - volts) ? q : p)).current_a;
  }
  return { stall_Nm: torque, speed_rad_s: (Math.PI / 3) / secPer60, stall_A: stallA, volts: v };
}

/**
 * Top of the published efficient current band, e.g. "4 - 7 A (>74%) |" gives 7, "8-12A" gives 12.
 * European decimals allowed, e.g. "2,5 - 7 A".
 */
export function efficientBandTop(text) {
  if (!text) return null;
  const m = /(\d+(?:[.,]\d+)?)\s*-\s*(\d+(?:[.,]\d+)?)\s*A/.exec(String(text));
  return m ? parseFloat(m[2].replace(',', '.')) : null;
}

/**
 * SPEC M2/M8 rule: a published continuous rating wins. Otherwise the lower of the top of the published efficient
 * band and 50% of the published burst current. A motor with neither is ineligible (null).
 */
export function motorContinuous(part) {
  const s = part.specs;
  if (Number.isFinite(s.continuous_current_a)) return { amps: s.continuous_current_a, rule: 'published continuous rating' };
  const band = efficientBandTop(s.max_efficiency_current_text);
  const burst = s.burst_current_a ?? s.current_capacity_a ?? null;
  if (band !== null && Number.isFinite(burst)) return { amps: Math.min(band, 0.5 * burst), rule: `no continuous rating: lower of efficient band top ${band} A and 50% of burst ${burst} A` };
  return { amps: null, rule: 'ineligible: no continuous rating, band or burst' };
}

export function motorModel(part) {
  const s = part.specs;
  const kv = (s.kv_rpm_per_v * 2 * Math.PI) / 60;
  const rated = motorContinuous(part);
  // Rotor inertia is not published. Estimated: 40% of mass as a bell at the outer radius.
  const dia = parseFloat(String(s.dimensions_text_mm ?? '28').replace(/[^0-9.,x ]/g, '').split(/[x ]/)[0].replace(',', '.')) / 1000 || 0.028;
  const J = 0.4 * (part.mass_g / 1000) * (0.45 * dia) ** 2;
  return { id: part.id, kv_rad_per_v_s: kv, R: s.resistance_ohm, I0: s.no_load_current_a, I_cont: rated.amps, I_cont_rule: rated.rule, I_burst: s.burst_current_a ?? s.current_capacity_a ?? null, burst_s: s.burst_duration_s ?? s.current_capacity_duration_s ?? null, J_kg_m2: J, mass_kg: part.mass_g / 1000, J_estimated: true };
}

export function batteryModel(part) {
  const s = part.specs;
  const Ah = (s.capacity_mah ?? 0) / 1000;
  const C = s.c_rating_continuous_claimed ?? null;
  const cells = s.cells_series;
  // Pack resistance is not in the catalogue. Estimated so that the claimed C rate sags about 10% of
  // nominal voltage, a common rule for hobby packs; flagged in the design file.
  const Rint = C && Ah ? (0.1 * cells * 3.7) / (C * Ah) : 0.1 * cells;
  return { id: part.id, cells, V_nom: s.nominal_voltage_v, capacity_Ah: Ah, I_cont: C ? C * Ah : null, R_int: Rint, R_estimated: true, mass_kg: part.mass_g / 1000 };
}

/** SPEC M7: only published strength counts. Woven tube 650 MPa, 64 GPa; rods the lower of their range; roll-wrapped none. */
export function tubeAllowable(part) {
  const s = part.specs;
  if (Number.isFinite(s.tensile_strength_lengthways_mpa)) return { sigma_MPa: s.tensile_strength_lengthways_mpa, E_GPa: s.tensile_modulus_lengthways_gpa ?? null, basis: 'published tensile strength' };
  if (s.tensile_strength_lengthways_mpa_range) return { sigma_MPa: s.tensile_strength_lengthways_mpa_range[0], E_GPa: s.tensile_modulus_lengthways_gpa_range?.[0] ?? null, basis: 'lower of the published range' };
  return { sigma_MPa: null, E_GPa: null, basis: 'no published strength: carries no primary load' };
}

export function tubeSection(part) {
  const od = part.specs.od_mm / 1000;
  const id = (part.specs.id_mm ?? 0) / 1000; // inner diameter, m
  return { od, id_m: id, I: (Math.PI * (od ** 4 - id ** 4)) / 64, c: od / 2, kgPerM: part.mass_g / 1000 };
}
