// Real numbers for panels and cards, every one read from the design file, the catalogue entries or a gate file.
// Nothing here is typed in: if the design changes, these change.
import { twistCommand } from '../../src/physics/kinematics.js';

const DEG = 180 / Math.PI;

export function numbers(model, data) {
  const D = model.design;
  const tr = model.train;
  const Vc = D.control.Vcruise;
  const row = D.control.trim.reduce((b, r) => (Math.abs(r.V - Vc) < Math.abs(b.V - Vc) ? r : b));
  const m2 = data.gates.find((x) => x.gate === 'M2')?.measured;
  const bat = D.battery;
  const sched = { sym: row.sym, dyn: row.dyn, lead: D.cruise.lead };
  return {
    name: D.label,
    reference: D.reference,
    span_m: model.span,
    area_m2: model.area,
    AR: D.geometry.AR,
    mass_g: D.mass.total_g,
    parts_g: D.bom.parts_g,
    structure_g: D.bom.structure_g,
    margin_g: D.bom.margin_g,
    wingLoading: D.geometry.wing_loading_N_m2,
    bomLines: D.bom.parts.length + D.bom.structure.length,
    partCount: D.bom.parts.reduce((p, b) => p + b.count, 0),
    cruiseV: Vc,
    flapHz: row.freq,
    ampDeg: row.amp * DEG,
    twistDynDeg: row.dyn * DEG,
    ratio: tr.ratio,
    stages: tr.stages.map((s) => `${s.wheel.z}/${s.pinion.z}`),
    crankRpm: row.freq * 60,
    motorRpm: row.freq * 60 * tr.ratio,
    kv: model.parts[D.motor.id]?.specs.kv_rpm_per_v,
    motorName: model.parts[D.motor.id]?.name,
    batteryName: model.parts[bat.id]?.name,
    batteryWh: bat.V_nom * bat.capacity_Ah,
    batteryCells: bat.cells,
    batteryMah: bat.capacity_Ah * 1000,
    cruisePowerW: m2 ? m2.cruise_power_w : null,
    enduranceMin: m2 ? m2.endurance_min : null,
    powerRatio: m2 ? m2.power_ratio : null,
    sched,
    twistAt: (phase, side = 1) => twistCommand(phase, sched, side),
    gates: data.gates,
    passCount: data.gates.filter((g) => g.passed).length,
    fit: data.fit,
  };
}

/** One line per gate for cards and panels, in the gate's own words and numbers. */
export function gateLine(g) {
  if (g.line) return g.line;
  const m = g.measured ?? {};
  const f = (v, d = 1) => (typeof v === 'number' ? v.toFixed(d) : '—');
  switch (g.gate) {
    case 'M1': return `Hand launch: ${f(m.height_lost_m, 2)} m lost${m.robustness_passed === false ? ', fails one robustness corner' : ''}`;
    case 'M2': return `Endurance ${f(m.endurance_min)} min at ${f(m.cruise_power_w)} W${m.robustness_passed === false ? '; fails at +5% mass' : ''}`;
    case 'M3': return `30° turn, height held to ${f(m.altitude_dev_m, 1)} m`;
    case 'M4': return `Rough air: ${m.seeds_passed ?? '—'} of ${m.seeds_run ?? 100} seeds (needs 98)`;
    case 'M5': return `Belly landing at ${f(m.touchdown_sink_m_s, 2)} m/s sink`;
    case 'M6': return `Mass ${f(m.total_g, 0)} g from the BOM`;
    case 'M7': {
      const s = m.spar_inner?.fos, c = m.coupler?.fos_buckling;
      return `Structure: inner spar FoS ${f(s, 2)}, coupler buckling ${f(c, 2)} (need 1.5)`;
    }
    case 'M8': {
      const pairs = [['motor', m.motor_rms_2s_max_A, m.motor_continuous_A], ['ESC', m.esc_rms_2s_max_A, m.esc_continuous_A], ['battery', m.battery_rms_2s_max_A, m.battery_continuous_A]].filter((p) => typeof p[1] === 'number' && p[2]);
      const w = pairs.sort((x, y) => y[1] / y[2] - x[1] / x[2])[0];
      return w ? `Currents: ${w[0]} ${f(w[1], 1)} A against ${f(w[2], 0)} A continuous` : 'Currents against ratings';
    }
    case 'V1': return 'UVLM vs Theodorsen';
    case 'V2': return `UVLM lift slope: ${f(m.max_error_pct, 1)}% off (limit 3%)`;
    case 'V3': return 'UVLM thrust vs Garrick';
    case 'V4': return 'SmartBird power check';
    case 'V5': return 'Strip model vs UVLM';
    case 'V6': return 'GPU lattice Boltzmann checks';
    case 'V7': return 'OpenFOAM vs wind tunnel';
    default: return g.gate;
  }
}
