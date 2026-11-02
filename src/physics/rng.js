// Deterministic pseudo random numbers. Same seed, same sequence, in node and in a browser.
export function makeRng(seed) {
  let s = (seed >>> 0) || 0x9e3779b9;
  const next = () => {
    // mulberry32
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  let spare = null;
  return {
    uniform: next,
    range: (a, b) => a + (b - a) * next(),
    /** Standard normal by Box-Muller. */
    normal() {
      if (spare !== null) {
        const v = spare;
        spare = null;
        return v;
      }
      let u = 0, v = 0, r = 0;
      do {
        u = 2 * next() - 1;
        v = 2 * next() - 1;
        r = u * u + v * v;
      } while (r === 0 || r >= 1);
      const f = Math.sqrt((-2 * Math.log(r)) / r);
      spare = v * f;
      return u * f;
    },
  };
}
