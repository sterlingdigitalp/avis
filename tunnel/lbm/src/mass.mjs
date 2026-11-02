// Closed-box mass series: drift per cycle and the budget identity. Pure.

/**
 * @param pts [{deltaMass, cells, budget, fixApplied}] sampled once a cycle
 * The drift is a least-squares slope over all whole cycles: an endpoint difference is dominated by
 * the oscillation of the instantaneous mass within a cycle. With the correction on, the budget is
 * applied every step: the correction lags its budget by one sample, and swept-node refill makes
 * single-sample budgets swing by about 1 in total mass, so a ten-step lag alone shows up as noise.
 */
export function massSeries(pts) {
  const rel = pts.map((p) => p.deltaMass / p.cells);
  // Least squares about the middle cycle: the deviations sum to zero, so the mean of rel drops out.
  const N = rel.length, xm = (N - 1) / 2;
  let sxy = 0, sxx = 0;
  rel.forEach((v, i) => { sxy += (i - xm) * v; sxx += (i - xm) ** 2; });
  const first = pts[0], last = pts[N - 1];
  return {
    meanDrhoFirst: rel[0], meanDrhoEnd: rel[N - 1], slopePerCycle: sxy / sxx,
    // Change in total mass against the accumulated link and refill budget plus any correction applied.
    budgetCheck: N > 1 ? { massChange: last.deltaMass - first.deltaMass, budget: last.budget - first.budget, fixApplied: last.fixApplied - first.fixApplied } : null,
    series: rel.filter((_, i) => i % 5 === 0),
  };
}
