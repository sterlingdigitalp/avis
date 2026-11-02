// Measured MLUPS and seconds per wingbeat for a real wing at several grid sizes.
//   node tools/wingbeat-cost.mjs <design.json> '<json array of {dx, half}>' <out.json>
import fs from 'node:fs';
import { openPage, cleanup, watchdog } from '../src/harness.mjs';
watchdog(560000);
const [designPath, list, outPath] = process.argv.slice(2);
const designFile = JSON.parse(fs.readFileSync(designPath, 'utf8'));
const { page } = await openPage();
await page.evaluate(async () => { window.jobs = await import('/tunnel/lbm/src/jobs.mjs'); });
const rows = fs.existsSync(outPath) ? JSON.parse(fs.readFileSync(outPath, 'utf8')) : [];
for (const c of JSON.parse(list)) {
  const r = await page.evaluate(({ designFile, c }) => window.jobs.runDesign({ designFile, name: `cost-${c.dx}-${c.half}`, wingbeats: c.wingbeats ?? 0.04, sampleEvery: 40, ...c, maxSeconds: 1e9 }), { designFile, c });
  const row = { dx_mm: c.dx * 1000, half: c.half ?? true, cells_M: +(r.grid.cells / 1e6).toFixed(1), grid: [r.grid.nx, r.grid.ny, r.grid.nz], gib_populations: +((r.grid.cells * 19 * 8) / 2 ** 30).toFixed(1), steps_measured: Math.round(r.cost.steps_per_wingbeat * (c.wingbeats ?? 0.04)), steps_per_wingbeat: r.cost.steps_per_wingbeat, mlups: Math.round(r.cost.mlups), seconds_per_wingbeat: Math.round(r.cost.seconds_per_wingbeat), minutes_per_wingbeat: +(r.cost.seconds_per_wingbeat / 60).toFixed(1), frame_ms: +r.cost.frame_ms_per_step.toFixed(1), links: r.cost.max_links, tau0: +r.params.tau0.toFixed(6), chord_cells: +r.params.chord_cells.toFixed(1), y_plus: Math.round(r.params.estimated_y_plus_first_cell), Re_mac: Math.round(r.params.Re_mac), errors: r.errors, lift_first_N: r.per_wingbeat?.[0]?.lift_N };
  rows.push(row);
  console.log(JSON.stringify(row));
  fs.writeFileSync(outPath, JSON.stringify(rows, null, 1));
}
cleanup();
setTimeout(() => process.exit(0), 1000);
