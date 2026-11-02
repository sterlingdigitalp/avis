#!/usr/bin/env node
// Frame-accurate capture of the film page to video: serves the site folder, steps window.FILM.frame(t) and pipes
// each screenshot to ffmpeg. One call renders [from, to) of the timeline into one chunk; render.mjs splits the film
// into chunks so every GPU hold stays under 10 minutes and a killed job resumes at the next missing chunk.
//   node film/capture.mjs --site out/film/site --edit edit --from 0 --to 6 --fps 30 --out chunk.mp4
//   node film/capture.mjs ... --stills 1.5,3,7 --out sheet-prefix   (PNG per time, for review)
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { serve, launch, done, onCleanup } from './lib/browser.mjs';

const a = process.argv.slice(2);
const opt = (k, d) => (a.includes(`--${k}`) ? a[a.indexOf(`--${k}`) + 1] : d);
const site = path.resolve(opt('site', 'out/film/site'));
const edit = opt('edit', 'edit');
const fps = +opt('fps', 30);
const from = +opt('from', 0);
const to = +opt('to', NaN);
const out = path.resolve(opt('out'));
const stills = opt('stills', null);

const srv = await serve(site);
const browser = await launch({ selfTimeoutS: +opt('timeout', 560) });
let ff = null;
onCleanup(() => {
  if (ff && ff.exitCode == null) ff.kill('SIGKILL');
});
let code = 0;
try {
  const ctx = await browser.newContext({ viewport: { width: 1920, height: 1080 }, deviceScaleFactor: 1, colorScheme: 'dark' });
  const page = await ctx.newPage();
  const errs = [];
  page.on('pageerror', (e) => errs.push(String(e)));
  await page.goto(`${srv.url}/avis-film.html?edit=${edit}&t=${from}`);
  await page.waitForFunction('window.READY === true', null, { timeout: 120000 });
  if (errs.length) throw new Error(errs.join('\n'));
  const film = page.locator('#film');
  if (stills) {
    for (const t of stills.split(',').map(Number)) {
      await page.evaluate((tt) => window.FILM.frame(tt), t);
      await film.screenshot({ path: `${out}-${t.toFixed(2)}.png` });
    }
  } else {
    const end = Number.isFinite(to) ? to : await page.evaluate(() => window.FILM.duration);
    const n = Math.round((end - from) * fps);
    fs.mkdirSync(path.dirname(out), { recursive: true });
    const tmp = `${out}.part.mp4`;
    ff = spawn(process.env.FFMPEG || 'ffmpeg', ['-v', 'error', '-y', '-f', 'image2pipe', '-framerate', String(fps), '-i', '-', '-c:v', 'libx264', '-preset', 'slow', '-crf', '16', '-pix_fmt', 'yuv420p', '-r', String(fps), tmp], { stdio: ['pipe', 'inherit', 'inherit'] });
    const t0 = Date.now();
    for (let i = 0; i < n; i++) {
      const t = from + i / fps;
      await page.evaluate((tt) => window.FILM.frame(tt), t);
      const png = await film.screenshot({ type: 'png' });
      if (!ff.stdin.write(png)) await new Promise((r) => ff.stdin.once('drain', r));
      if (i % 60 === 0) console.log(`[capture] ${i}/${n} t=${t.toFixed(2)} ${((Date.now() - t0) / 1000).toFixed(0)}s`);
    }
    ff.stdin.end();
    await new Promise((r, j) => ff.on('exit', (c) => (c === 0 ? r() : j(new Error(`ffmpeg exit ${c}`)))));
    fs.renameSync(tmp, out);
    console.log(`[capture] ${out}: ${n} frames in ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  }
  if (errs.length) console.error('page errors:', errs.slice(0, 5).join(' | '));
} catch (e) {
  console.error(e);
  code = 1;
} finally {
  srv.close();
  await done(code);
}
