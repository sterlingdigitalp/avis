// Mass budget identity on the CPU reference, for the closed-box case of V6 (jobs.massBoxCase), in
// f32 or f64. Same geometry frames and step order as the GPU job (src/run.mjs with relink 1). Each
// cycle it records the measured mass change, the fixed-point budget the GPU also keeps, and the
// exact audit (src/cpu.mjs massAudit): link, cell-change and collision terms in compensated f64.
//
//   node tools/mass-audit.mjs <out.json> '{"kind":"thin","bouzidi":true,"precision":"f64","scale":0.5}'
//
// scale shrinks the box, chord, heave and period together, keeping blockage, reduced frequency,
// peak body speed, tau and U the same as the full-size case.
import fs from 'node:fs';
import { CpuLbm } from '../src/cpu.mjs';
import { BoundaryEngine } from '../src/geometry.mjs';
import { massBoxCase } from '../src/jobs.mjs';

const [outPath, json] = process.argv.slice(2);
const o = JSON.parse(json);
const { kind = 'thin', bouzidi = true, precision = 'f64', scale = 1, cycles = 50, withBody = true, alphaMean = 0, h0 = 8 } = o;
const size = { nx: Math.round(160 * scale), ny: Math.round(160 * scale), nz: 4, c: Math.round(40 * scale), h0: h0 * scale, period: Math.round(1000 * scale), alphaMean, kind, withBody };
const { grid, cfg, surfaces } = massBoxCase(size);
const eng = new BoundaryEngine(grid, surfaces, { bouzidi });
const s = new CpuLbm({ ...cfg, precision, mask: eng.mask ?? undefined, massAudit: true });
s.setFrame(eng.init(0.5));
const total = cycles * size.period;
let fixed = 0;
const rows = [];
const t0 = Date.now();
for (let n = 0; n < total; n++) {
  s.setFrame(eng.frame(n + 0.5));
  s.step(1);
  if ((n + 1) % 10 === 0 || n + 1 === total) for (const p of s.takeForces()) fixed += p.dMass;
  if ((n + 1) % size.period === 0) {
    const m = s.massDelta();
    const a = s.audit;
    rows.push({ n: n + 1, deltaMass: m.deltaMass, cells: m.cells, budgetFixed: fixed, link: a.link.value, cell: a.cell.value, collide: a.collide.value });
  }
}
const first = rows[0], last = rows[rows.length - 1];
const d = (k) => last[k] - first[k];
const massChange = d('deltaMass'), exact = d('link') + d('cell');
const out = {
  case: { ...size, bouzidi, precision, cycles, tau0: cfg.tau0, U: -cfg.u0[0], cells: s.n }, seconds: (Date.now() - t0) / 1000,
  // Same window as the GPU check: from the end of the first cycle to the end of the last.
  massChange, budgetFixedPoint: d('budgetFixed'), budgetExact: exact, linkTerm: d('link'), cellTerm: d('cell'), collisionTerm: d('collide'),
  residualFixedPointRelative: Math.abs(massChange - d('budgetFixed')) / Math.abs(d('budgetFixed')),
  residualExactRelative: Math.abs(massChange - exact) / Math.abs(exact),
  // What is left once the collision's own non-conservation is counted too: the whole step is accounted for.
  residualAfterCollisionRelative: Math.abs(massChange - exact - d('collide')) / Math.abs(exact),
  fixedPointVsExactRelative: Math.abs(d('budgetFixed') - exact) / Math.abs(exact),
  rows,
};
fs.writeFileSync(outPath, JSON.stringify(out, null, 1));
console.log(JSON.stringify({ kind, bouzidi, precision, scale, seconds: out.seconds, massChange, budgetFixedPoint: out.budgetFixedPoint, budgetExact: exact, collisionTerm: out.collisionTerm, residualFixedPointRelative: out.residualFixedPointRelative, residualExactRelative: out.residualExactRelative, residualAfterCollisionRelative: out.residualAfterCollisionRelative }));
