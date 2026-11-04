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
  return !/sold.?out|out of stock|end.?of.?life|discontinu|obsolete|no longer available/i.test(notes);
}

/** kg per unit of the part's mass basis, read from specs.mass_basis as SPEC requires. */
export function massBasis(part) {
  return part.specs?.mass_basis ?? 'per_part';
}

/**
 * A servo's speed and stall-style torque at a supply voltage, interpolated between its published rows. SPEC ruling:
 * no extrapolation outside the published voltage rows, so a supply outside them returns null (the servo cannot be
 * used on that supply). Stall current: a value published without a voltage is used as published at any supply;
 * values at stated voltages are interpolated, and outside them the nearest is used (the higher-voltage figure when
 * the supply is below every stated row, which is the conservative side).
 */
export function servoAt(part, volts) {
  const rows = [...(part.specs.performance ?? [])].filter((r) => Number.isFinite(r.voltage_v)).sort((a, b) => a.voltage_v - b.voltage_v);
  if (!rows.length) return null;
  if (volts < rows[0].voltage_v - 1e-9 || volts > rows[rows.length - 1].voltage_v + 1e-9) return null;
  let a = rows[0], b = rows[rows.length - 1];
  for (let i = 1; i < rows.length; i++) {
    if (volts <= rows[i].voltage_v) {
      a = rows[i - 1];
      b = rows[i];
      break;
    }
  }
  if (rows.length === 1) a = b = rows[0];
  const f = b.voltage_v === a.voltage_v ? 0 : (volts - a.voltage_v) / (b.voltage_v - a.voltage_v);
  const torque = a.torque_nm + f * (b.torque_nm - a.torque_nm);
  const secPer60 = a.speed_s_per_60deg + f * (b.speed_s_per_60deg - a.speed_s_per_60deg);
  return { stall_Nm: torque, speed_rad_s: (Math.PI / 3) / secPer60, stall_A: servoStallCurrent(part, volts), volts };
}

/** Published stall current at a supply voltage per the SPEC ruling, or null if none is published. */
export function servoStallCurrent(part, volts) {
  const sc = (part.specs.stall_current ?? []).filter((q) => Number.isFinite(q.current_a));
  if (!sc.length) return null;
  const free = sc.find((q) => q.voltage_v === null || q.voltage_v === undefined);
  if (free) return free.current_a;
  const r = sc.slice().sort((p, q) => p.voltage_v - q.voltage_v);
  if (volts <= r[0].voltage_v) return r[0].current_a;
  if (volts >= r[r.length - 1].voltage_v) return r[r.length - 1].current_a;
  for (let i = 1; i < r.length; i++) {
    if (volts <= r[i].voltage_v) {
      const f = (volts - r[i - 1].voltage_v) / (r[i].voltage_v - r[i - 1].voltage_v);
      return r[i - 1].current_a + f * (r[i].current_a - r[i - 1].current_a);
    }
  }
  return r[r.length - 1].current_a;
}

/** A servo's published operating voltage range [lo, hi] from text like "4.8V - 8.4V DC Volt" or "6.0V - 8.4V", or null. */
export function servoVoltageRange(part) {
  const m = /(\d+(?:\.\d+)?)\s*V?\s*[-~]\s*(?:\d+(?:\.\d+)?\s*V?\s*[-~]\s*)?(\d+(?:\.\d+)?)\s*V/i.exec(String(part.specs?.voltage_range_text ?? ''));
  return m ? [parseFloat(m[1]), parseFloat(m[2])] : null;
}

/**
 * Stall current at a supply voltage on the conservative side: the published figure (servoStallCurrent), and above the
 * highest stated row the top row scaled by the voltage ratio (a locked rotor draws V / R). Never below a published value.
 */
export function servoStallCurrentConservative(part, volts) {
  const base = servoStallCurrent(part, volts);
  if (!Number.isFinite(base)) return null;
  const stated = (part.specs.stall_current ?? []).filter((q) => Number.isFinite(q.current_a) && Number.isFinite(q.voltage_v));
  if (!stated.length) return base; // published without a voltage: as published at any supply (SPEC ruling)
  const top = stated.reduce((a, b) => (b.voltage_v > a.voltage_v ? b : a));
  return volts > top.voltage_v ? top.current_a * (volts / top.voltage_v) : base;
}

/** Whether an ESC's published cell range ("2-4S LiPo", "3-4S") covers a pack of `cells`; null if none is published. */
export function escCellsCover(esc, cells) {
  const m = /(\d+)\s*-\s*(\d+)\s*S/i.exec(String(esc.specs?.cells_text ?? ''));
  if (!m || !Number.isFinite(cells)) return null;
  return cells >= +m[1] && cells <= +m[2];
}

/** Output voltage and current of an ESC's built-in BEC from its published text, e.g. "Switch 5V 3A". */
export function becOf(esc) {
  const t = String(esc.specs?.bec ?? '');
  const v = /(\d+(?:\.\d+)?)\s*V/.exec(t);
  const a = /(\d+(?:\.\d+)?)\s*A/.exec(t);
  return { volts: v ? parseFloat(v[1]) : null, amps: a ? parseFloat(a[1]) : null, kind: /switch/i.test(t) ? 'switching' : /linear/i.test(t) ? 'linear' : null, text: t || null };
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
  // Rested voltage curve: the model's LiPo shape shifted to the published nominal cell voltage (Li-ion 3.6 V cells sit
  // about 0.1 V lower); an assumption, flagged, since no discharge curve is published for these packs.
  const shift = Number.isFinite(s.nominal_voltage_v) && cells ? s.nominal_voltage_v / cells - 3.7 : 0;
  return { id: part.id, chemistry: s.chemistry, cells, V_nom: s.nominal_voltage_v, capacity_Ah: Ah, I_cont: C ? C * Ah : null, R_int: Rint, R_estimated: true, cell_shift_V: shift, ocv_assumed: 'LiPo rested curve shifted to the published nominal cell voltage', mass_kg: part.mass_g / 1000 };
}

/**
 * SPEC M7: only published strength counts. Woven tube 650 MPa, 64 GPa; rods the lower of their range; roll-wrapped
 * none. Ruling after catalogue round 2: a strength the maker labels "reference only" or "design guide" counts at half
 * value (its modulus in full); a metal tube uses its published minimum yield. In bending the lower of the published
 * tensile and compressive strengths governs.
 */
export function tubeAllowable(part) {
  const s = part.specs;
  if (part.category === 'metal_tube') {
    if (Number.isFinite(s.yield_strength_mpa)) return { sigma_MPa: s.yield_strength_mpa, E_GPa: s.elastic_modulus_gpa ?? null, basis: 'published minimum yield (metal tube)' };
    return { sigma_MPa: null, E_GPa: null, basis: 'no published yield: carries no primary load' };
  }
  let sigma = null, E = null, basis = null;
  if (Number.isFinite(s.tensile_strength_lengthways_mpa)) {
    sigma = s.tensile_strength_lengthways_mpa; E = s.tensile_modulus_lengthways_gpa ?? null; basis = 'published tensile strength';
  } else if (s.tensile_strength_lengthways_mpa_range) {
    sigma = s.tensile_strength_lengthways_mpa_range[0]; E = s.tensile_modulus_lengthways_gpa_range?.[0] ?? null; basis = 'lower of the published range';
  } else return { sigma_MPa: null, E_GPa: null, basis: 'no published strength: carries no primary load' };
  const comp = Number.isFinite(s.compressive_strength_mpa) ? s.compressive_strength_mpa : s.compressive_strength_lengthways_mpa_range?.[0];
  if (Number.isFinite(comp) && comp < sigma) { sigma = comp; basis += ', compressive lower and governs in bending'; }
  if (/reference only|design guide/i.test(String(s.strength_note ?? ''))) { sigma *= 0.5; basis += '; disclaimed by the maker (reference only / design guide): half value per SPEC'; }
  return { sigma_MPa: sigma, E_GPa: E, basis };
}

/**
 * Round 17 (audit item 17): a tube whose listing and maker's spec sheet give different sections (CST: listing 8 x 7 mm,
 * spec sheet 7.8 x 6.8 mm, 18.1 g/m) is drawn on the envelope of both (the larger outside, the smaller bore), rated on
 * the weaker section and weighed at the heavier figure. Dimensions in mm, mass in g/m. A tube with one section is that.
 * specSheet false (designs before round 17, params.tubeSpecSheet unset) keeps the listing's section and mass, so their
 * records rebuild as they were.
 */
export function tubeSpec(part, specSheet = true) {
  const s = part.specs, od = s.od_mm, id = s.id_mm ?? 0, g = specSheet ? Math.max(part.mass_g, s.mass_g_per_m_spec_sheet ?? 0) : part.mass_g;
  if (!specSheet || !Number.isFinite(s.od_mm_spec_sheet)) return { od_draw: od, id_draw: id, od_rate: od, id_rate: id, g_per_m: g, basis: 'one published section' };
  const od2 = s.od_mm_spec_sheet, id2 = s.id_mm_spec_sheet ?? id, Z = (D, d) => (D ** 4 - d ** 4) / D;
  const spec = Z(od2, id2) < Z(od, id);
  return { od_draw: Math.max(od, od2), id_draw: Math.min(id, id2), od_rate: spec ? od2 : od, id_rate: spec ? id2 : id, g_per_m: g, basis: `two published sections (listing ${od} x ${id} mm, spec sheet ${od2} x ${id2} mm): drawn on their envelope (${Math.max(od, od2)} outside, ${Math.min(id, id2)} bore), rated on the weaker (${spec ? od2 : od} x ${spec ? id2 : id} mm), weighed at the heavier ${g} g/m` };
}

export function tubeSection(part, specSheet = true) {
  const t = tubeSpec(part, specSheet), od = t.od_rate / 1000;
  const id = t.id_rate / 1000; // inner diameter, m
  return { od, id_m: id, I: (Math.PI * (od ** 4 - id ** 4)) / 64, c: od / 2, kgPerM: t.g_per_m / 1000 };
}
