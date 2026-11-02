// Harmonic lift and mean drag from a plate run's force series. Pure; shared by runPlate and checks.
import { fitHarmonic } from './theodorsen.mjs';

/**
 * @param series {t, cl, cd}: samples at the centres of averaging windows of M steps
 * @param o {omega, period, M, cycles?}: fit over the last `cycles` periods (default 2), and the
 *          same span just before it so the caller can see whether the answer is still moving.
 * A window average attenuates a harmonic by sinc(omega M / 2); that is undone, so the amplitude is
 * of the instantaneous signal.
 */
export function liftHarmonic({ t, cl, cd }, { omega, period, M, cycles = 2 }) {
  const nFit = Math.round((cycles * period) / M);
  const fit = fitHarmonic(t.slice(-nFit), cl.slice(-nFit), omega);
  const prev = fitHarmonic(t.slice(-2 * nFit, -nFit), cl.slice(-2 * nFit, -nFit), omega);
  const atten = Math.sin((omega * M) / 2) / ((omega * M) / 2);
  return {
    cl: { amp: fit.amp / atten, phaseDeg: (fit.phaseRad * 180) / Math.PI, mean: fit.mean, prevAmp: prev.amp / atten, prevPhaseDeg: (prev.phaseRad * 180) / Math.PI },
    meanCd: cd.slice(-nFit).reduce((p, v) => p + v, 0) / nFit,
  };
}

/** The sample times runSeries produces: windows of M steps, the last one possibly shorter. */
export function sampleTimes(total, M) {
  const t = [];
  for (let n = 0; n < total; n += M) t.push(n + Math.min(M, total - n) / 2);
  return t;
}
