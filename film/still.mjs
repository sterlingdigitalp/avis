#!/usr/bin/env node
// Screenshot pages of the built site: one PNG per job. Run under tools/run-guarded.py.
//   node film/still.mjs --dir out/film/site --jobs jobs.json
//   jobs.json: [{"page":"avis-lab.html","query":{"s":{...}},"size":[1600,900],"dpr":1,"out":"out/film/review/x.png",
//               "wait":"window.READY","eval":"js run before the shot", "settle":500, "dark":true}]
import fs from 'node:fs';
import path from 'node:path';
import { serve, launch, done } from './lib/browser.mjs';

const arg = (k, d) => {
  const i = process.argv.indexOf(`--${k}`);
  return i > 0 ? process.argv[i + 1] : d;
};
const dir = path.resolve(arg('dir', 'out/film/site'));
const jobs = JSON.parse(fs.readFileSync(arg('jobs'), 'utf8'));
const srv = await serve(dir);
const browser = await launch({ selfTimeoutS: +arg('timeout', 500) });
let code = 0;
try {
  for (const j of jobs) {
    const [w, h] = j.size ?? [1600, 900];
    const ctx = await browser.newContext({ viewport: { width: w, height: h }, deviceScaleFactor: j.dpr ?? 1, colorScheme: j.dark === false ? 'light' : 'dark', reducedMotion: j.reducedMotion ? 'reduce' : 'no-preference' });
    const page = await ctx.newPage();
    const errs = [];
    page.on('pageerror', (e) => errs.push(String(e)));
    page.on('console', (m) => (m.type() === 'error' || m.type() === 'warning') && errs.push(`${m.type()}: ${m.text()}`));
    const qs = Object.entries(j.query ?? {}).map(([k, v]) => `${k}=${encodeURIComponent(typeof v === 'string' ? v : Buffer.from(JSON.stringify(v)).toString('base64'))}`).join('&');
    await page.goto(`${srv.url}/${j.page}${qs ? '?' + qs : ''}`);
    await page.waitForFunction(j.wait ?? 'window.READY === true', null, { timeout: j.timeoutMs ?? 60000 });
    if (j.eval) await page.evaluate(j.eval);
    if (j.probe) console.log('probe', JSON.stringify(await page.evaluate(j.probe)));
    if (j.settle) await page.waitForTimeout(j.settle);
    fs.mkdirSync(path.dirname(path.resolve(j.out)), { recursive: true });
    await page.screenshot({ path: j.out });
    const gpu = await page.evaluate(() => {
      const c = document.createElement('canvas').getContext('webgl2');
      const e = c?.getExtension('WEBGL_debug_renderer_info');
      return e ? c.getParameter(e.UNMASKED_RENDERER_WEBGL) : 'unknown';
    });
    console.log(`${j.out}  ${gpu}${errs.length ? '  ERRORS: ' + errs.slice(0, 4).join(' | ') : ''}`);
    await ctx.close();
  }
} catch (e) {
  console.error(e);
  code = 1;
} finally {
  srv.close();
  await done(code);
}
