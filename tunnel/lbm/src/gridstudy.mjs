// Grid convergence from successive resolutions: observed order and Richardson limit. Pure.

/**
 * Three or more values on grids refined by a constant ratio r. Order from the last two successive
 * differences, p = ln(d1/d2) / ln(r); NaN when the differences change sign (no asymptotic order).
 */
export function observedOrder(vals, ratio) {
  const d = [];
  for (let i = 1; i < vals.length; i++) d.push(vals[i] - vals[i - 1]);
  const n = d.length;
  const q = n >= 2 ? d[n - 2] / d[n - 1] : NaN;
  const p = q > 0 ? Math.log(q) / Math.log(ratio) : NaN;
  const extrap = Number.isFinite(p) ? vals[vals.length - 1] + d[n - 1] / (ratio ** p - 1) : NaN;
  return { values: vals, differences: d, difference_ratio: q, refinement_ratio: ratio, observed_order: p, richardson_limit: extrap, monotone: d.every((x) => Math.sign(x) === Math.sign(d[0])) };
}

/**
 * @param levels [{c, r}] in increasing c; r is a plate result ({cl:{amp,phaseDeg}, meanCd, ampErrPct, phaseErrDeg}) or missing
 * @param diverged (r) => boolean
 * The finest three stable levels give the headline order; every consecutive triple is reported too.
 */
export function gridStudy(levels, diverged) {
  const missing = levels.filter((x) => !x.r?.cl).map((x) => x.c);
  const div = levels.filter((x) => x.r?.cl && diverged(x.r)).map((x) => x.c);
  const ok = levels.filter((x) => x.r?.cl && !diverged(x.r));
  const use = ok.slice(-3);
  const out = { c_cells: levels.map((x) => x.c), missing, diverged: div, used_for_order: use.map((x) => x.c), complete: missing.length === 0 && use.length >= 3 };
  if (use.length < 3) return out;
  const ratio = (t) => Math.sqrt(t[2].c / t[0].c);
  const of = (t, f) => observedOrder(t.map((x) => f(x.r)), ratio(t));
  const triples = [];
  for (let i = 0; i + 3 <= ok.length; i++) {
    const t = ok.slice(i, i + 3);
    triples.push({ c_cells: t.map((x) => x.c), amp: of(t, (r) => r.cl.amp), phase_deg: of(t, (r) => r.cl.phaseDeg) });
  }
  return {
    ...out, triples,
    amp: of(use, (r) => r.cl.amp), phase_deg: of(use, (r) => r.cl.phaseDeg), mean_cd: of(use, (r) => r.meanCd),
    all_amp: ok.map((x) => [x.c, +x.r.cl.amp.toFixed(5)]),
    amp_err_pct_vs_theodorsen: ok.map((x) => [x.c, +x.r.ampErrPct.toFixed(2)]), phase_err_deg_vs_theodorsen: ok.map((x) => [x.c, +x.r.phaseErrDeg.toFixed(2)]),
  };
}
