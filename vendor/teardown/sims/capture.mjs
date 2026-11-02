#!/usr/bin/env node
// Frame-accurate capture of a sim page to mp4: steps window.SIM.render(t) for
// every frame in headless Chromium, screenshots, pipes to ffmpeg, then muxes
// a music track if given.
//
//   node sims/capture.mjs <name> [--config post.json] [--out teardown-out/x.mp4]
//        [--size 1080x1350] [--dpr 1.3333] [--fps 60] [--from 0] [--to <duration>]
//        (--size is the layout in CSS px; --dpr multiplies the output: 1080x1350
//         at the default 4/3 gives a 1440x1800 video. Default 60 fps, crf 16.)
//        [--music music/track.mp3 [--music-start auto|SEC] [--no-sync] [--lufs -21]] [--crf 21]
//        [--still 12.5]   (--still: one PNG, no video)
//        [--stills 0,4,9,14 --cols 3]               (tiled review sheet, every tile labelled t=)
//        [--stills beats [--timeline f.json]]       (one tile per timeline beat, +0.3 s so motion settles)
//        [--stills every:0.2 --from 3 --to 5]       (scrub a transition)
//        [--self-timeout 600]  (seconds; hard ceiling so a hung page/encoder cannot run forever)
//
//   <name> is a sim built into teardown-out/dist, or a path to any .html
//   that exposes window.seek(seconds) and freezes its clock on ?render
//   (motion-effects, vox-explainer, milestone-reveal, model-showdown). An .html
//   needs --to <seconds> unless it sets window.DURATION.
//
// Leak safety: a killed capture must never leave Chromium or ffmpeg running. Every
// process this script spawns is tracked; SIGTERM/SIGINT/SIGHUP and normal completion
// (success or error) all run the same cleanup, with a SIGKILL fallback for anything
// that does not exit promptly, and a self-timeout in case nothing signals us at all.
// This is belt-and-suspenders: run under tools/run-guarded.py, which owns the whole
// process group and SIGKILLs it on timeout or on its own death (including its own
// un-catchable SIGKILL) regardless of what this script manages to do on its own.
import {spawn} from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath, pathToFileURL} from 'node:url';
import {CHROMIUM, CHROMIUM_ARGS, DIST, FFMPEG, OUT, PYTHON, playwright} from '../paths.mjs';

const {chromium} = playwright();

const HERE = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const name = argv[0];
const opt = (k, d) => {
  const i = argv.indexOf(`--${k}`);
  return i === -1 ? d : argv[i + 1];
};
if (!name) {
  console.error('usage: node sims/capture.mjs <name|file.html> [--config f.json] [--out f.mp4] [--music f.wav]');
  process.exit(1);
}
const SEEK = name.endsWith('.html');
const html = SEEK ? path.resolve(name) : path.join(DIST, `${name}.html`);
const label = SEEK ? path.basename(name, '.html') : name;
if (!fs.existsSync(html)) {
  console.error(SEEK ? `missing ${html}` : `missing ${html}; run node sims/build.mjs ${name}`);
  process.exit(1);
}
const [W, H] = opt('size', '1080x1350').split('x').map(Number);
const config = opt('config') ? JSON.parse(fs.readFileSync(opt('config'), 'utf8')) : {};
// --music <file> [--music-start auto|SECONDS]: analyse the track (sims/beats.py),
// cut the window so the drop lands ~40% in, and give the beat map to the director.
const music = opt('music');
let beats = null;
if (music) {
  const dur = Number(opt('to', config.duration ?? 30)) - Number(opt('from', 0));
  const ms = opt('music-start', 'auto');
  const cache = path.join(OUT, 'beats', `${path.basename(music).replace(/\.[^.]+$/, '')}-${ms}-${dur}.json`);
  fs.mkdirSync(path.dirname(cache), {recursive: true});
  const {execFileSync} = await import('node:child_process');
  if (!fs.existsSync(cache) || fs.statSync(cache).mtimeMs < fs.statSync(music).mtimeMs) {
    execFileSync(PYTHON, ['-I', path.join(HERE, 'beats.py'), music, '--duration', String(dur), '--start', ms, '-o', cache], {stdio: 'inherit'});
  }
  beats = JSON.parse(fs.readFileSync(cache, 'utf8'));
  if (!opt('no-sync')) config.beats = beats;
}
const still = opt('still');
const stamp = new Date().toISOString().slice(5, 16).replace(/[-:T]/g, '');
const out = path.resolve(opt('out', path.join(OUT, still ? `${label}-${still}.png` : `${label}-${stamp}.mp4`)));
fs.mkdirSync(path.dirname(out), {recursive: true});

// ---- cleanup infrastructure: tracks the browser and every ffmpeg child so any exit
// path (signal, self-timeout, error, or normal completion) can guarantee nothing survives.
let browser = null;
const children = new Set();
const trackedSpawn = (cmd, args, opts) => {
  const child = spawn(cmd, args, opts);
  children.add(child);
  const untrack = () => children.delete(child);
  child.once('close', untrack);
  child.once('error', untrack);
  return child;
};

// Last resort for process.on('exit'): exit handlers can only run synchronous code,
// so this is a synchronous SIGKILL sweep, not the graceful path.
//
// `browser.process` is guarded, not called directly, because current Playwright
// (1.58.2) does not expose it on the Browser returned by chromium.launch() — there is
// no public, synchronous way from here to reach Chromium's OS pid, so this is a no-op
// today and only fires if a future Playwright restores the method. The only reliable
// way to close Chromium from this script is the async `browser.close()` in cleanup()
// below. That also means a true SIGKILL straight to this node process (uncatchable by
// definition — nothing below runs) is NOT self-defended: Playwright always launches
// Chromium in its own detached process group on non-Windows (see processLauncher.js,
// `detached: process.platform !== 'win32'`), so tools/run-guarded.py's group kill
// (which targets this process's pgid) cannot reach it either — confirmed by hand.
// Running under run-guarded.py still fully protects the ffmpeg children (same pgid as
// this process), and in-process signals (SIGTERM/SIGINT/SIGHUP) still close Chromium
// cleanly via browser.close() below.
function hardKillSync() {
  for (const child of children) {
    if (child.pid && child.exitCode === null) {
      try {
        process.kill(child.pid, 'SIGKILL');
      } catch {}
    }
  }
  try {
    const proc = browser && browser.process && browser.process();
    if (proc && proc.pid && proc.exitCode === null) process.kill(proc.pid, 'SIGKILL');
  } catch {}
}

let cleaningUp = null;
// Graceful cleanup: SIGTERM each tracked child, give it 2 s, SIGKILL anything left,
// then close the browser with its own timeout/SIGKILL fallback.
async function cleanup(reason) {
  if (cleaningUp) return cleaningUp;
  cleaningUp = (async () => {
    if (reason) console.error(`\n[capture] ${reason}, cleaning up`);
    await Promise.all(
      [...children].map(
        (child) =>
          new Promise((resolve) => {
            if (child.exitCode !== null) return resolve();
            const t = setTimeout(() => {
              try {
                child.kill('SIGKILL');
              } catch {}
            }, 2000);
            child.once('close', () => {
              clearTimeout(t);
              resolve();
            });
            try {
              child.kill('SIGTERM');
            } catch {
              clearTimeout(t);
              resolve();
            }
          }),
      ),
    );
    if (browser) {
      const b = browser;
      browser = null;
      try {
        await Promise.race([
          b.close(),
          new Promise((_, reject) => setTimeout(() => reject(new Error('browser close timeout')), 3000)),
        ]);
      } catch {
        // b.process() is not available on this Playwright version (see hardKillSync's
        // comment); this stays as a forward-compatible no-op rather than a real fallback.
        try {
          const proc = b.process && b.process();
          if (proc && proc.pid) process.kill(proc.pid, 'SIGKILL');
        } catch {}
      }
    }
  })();
  return cleaningUp;
}

let exiting = false;
async function onSignal(sig) {
  if (exiting) return;
  exiting = true;
  clearTimeout(watchdog);
  await cleanup(`received ${sig}`);
  process.exit(sig === 'SELF_TIMEOUT' ? 124 : 128);
}
process.on('exit', hardKillSync);
process.on('SIGTERM', () => onSignal('SIGTERM'));
process.on('SIGINT', () => onSignal('SIGINT'));
process.on('SIGHUP', () => onSignal('SIGHUP'));

// Self-timeout: a hard ceiling so a hung page or encoder cannot run forever even when
// nothing external signals this script. unref() so it never keeps the process alive
// past a normal, successful exit.
const SELF_TIMEOUT_MS = Number(opt('self-timeout', 600)) * 1000;
let watchdog = setTimeout(() => onSignal('SELF_TIMEOUT'), SELF_TIMEOUT_MS);
watchdog.unref?.();

async function main() {
  // Browser and GPU flags come from ../paths.mjs.
  const exe = CHROMIUM;
  browser = await chromium.launch({
    executablePath: exe,
    args: CHROMIUM_ARGS,
  });
  const dpr = Number(opt('dpr', 4 / 3));
  const page = await browser.newPage({viewport: {width: W, height: H}, deviceScaleFactor: dpr});
  page.on('console', (m) => m.type() === 'error' && console.error('[page]', m.text()));
  page.on('pageerror', (e) => console.error('[page error]', e.message));
  await page.addInitScript((cfg) => {
    window.__CAPTURE = true;
    window.__SIM_CONFIG = cfg;
  }, config);
  await page.goto(pathToFileURL(html).href + (SEEK ? '?render' : ''));
  await page.waitForFunction(
    (seek) => (seek ? typeof window.seek === 'function' : window.SIM && window.SIM.ready),
    SEEK,
    {timeout: 60000},
  );
  await page.evaluate(() => document.fonts.ready);
  const info = await page.evaluate((seek) => {
    const gl = document.createElement('canvas').getContext('webgl2');
    const ext = gl && gl.getExtension('WEBGL_debug_renderer_info');
    return {
      duration: seek ? window.DURATION : window.SIM.duration,
      fps: seek ? window.FPS : window.SIM.fps,
      gpu: ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : 'unknown',
    };
  }, SEEK);
  console.log(`gpu: ${info.gpu}`);
  // One entry point for both contracts: a sim's SIM.render(t) or a page's seek(t).
  const draw = (t) => page.evaluate(([x, seek]) => (seek ? window.seek(x) : window.SIM.render(x)), [t, SEEK]);
  const end = Number(opt('to', info.duration));
  if (!Number.isFinite(end)) {
    throw new Error('no duration: pass --to <seconds> or set window.DURATION in the page');
  }

  // --stills: a list, `beats` (every number in the timeline, +0.3 s), or `every:<step>`.
  const stillTimes = (arg) => {
    if (arg === 'beats') {
      const file = opt('timeline', path.join(HERE, name, 'timeline.json'));
      const tl = JSON.parse(fs.readFileSync(file, 'utf8'));
      const ts = [];
      const walk = (v, key) => {
        if (key === 'fps' || key === 'duration') return;
        if (typeof v === 'number') ts.push(v + 0.3);
        else if (Array.isArray(v)) v.forEach((x) => walk(x));
        else if (v && typeof v === 'object') Object.entries(v).forEach(([k, x]) => walk(x, k));
      };
      walk(tl);
      return [0, ...ts];
    }
    if (arg.startsWith('every:')) {
      const step = Number(arg.slice(6));
      const ts = [];
      for (let t = Number(opt('from', 0)); t <= end + 1e-6; t += step) ts.push(t);
      return ts;
    }
    return arg.split(',').map(Number);
  };

  const stills = opt('stills');
  if (stills) return captureStillSheet(page, draw, stillTimes(stills), end);
  if (still !== undefined) return captureSingleStill(page, draw, Number(still));
  return captureVideo(page, draw, end);
}

// --stills: a tiled review sheet, one labelled frame per requested time.
async function captureStillSheet(page, draw, times, end) {
  const ts = [...new Set(times.map((t) => Math.min(Math.max(t, 0), end - 1 / 120).toFixed(2)))]
    .map(Number)
    .sort((a, b) => a - b);
  const dir = out.replace(/\.(png|mp4)$/, '') + '-frames';
  fs.mkdirSync(dir, {recursive: true});
  for (const [i, t] of ts.entries()) {
    await draw(t);
    // The t= label goes into the DOM for the screenshot only, so no ffmpeg font setup is needed.
    await page.evaluate((txt) => {
      const el = document.createElement('div');
      el.id = '__t';
      el.textContent = txt;
      el.style.cssText =
        'position:fixed;right:12px;top:12px;z-index:2147483647;font:600 28px monospace;color:#fff;background:rgba(0,0,0,.65);padding:4px 10px;border-radius:4px';
      document.body.appendChild(el);
    }, `t=${t.toFixed(2)}`);
    await page.screenshot({path: path.join(dir, `${String(i).padStart(3, '0')}.png`)});
    await page.evaluate(() => document.getElementById('__t')?.remove());
  }
  await browser.close();
  browser = null;
  const cols = Math.min(ts.length, Number(opt('cols', 3)));
  const sheet = out.endsWith('.png') ? out : out.replace(/\.mp4$/, '') + '-sheet.png';
  const tw = Math.round(W * 0.4);
  const th = Math.round(H * 0.4);
  await new Promise((r) =>
    trackedSpawn(FFMPEG, ['-loglevel', 'error', '-y', '-framerate', '1', '-i', path.join(dir, '%03d.png'),
      '-vf', `scale=${tw}:${th},tile=${cols}x${Math.ceil(ts.length / cols)}:padding=4:color=0x333333`, '-frames:v', '1', sheet],
      {stdio: 'inherit'}).on('close', r),
  );
  console.log(`→ ${sheet}  (full frames in ${dir})`);
  return 0;
}

// --still T: one PNG, no video.
async function captureSingleStill(page, draw, t) {
  await draw(t);
  await page.screenshot({path: out});
  await browser.close();
  browser = null;
  console.log(`→ ${out}`);
  return 0;
}

// The default mode: every frame piped to ffmpeg as it is drawn, then an optional music mux.
async function captureVideo(page, draw, end) {
  const fps = Number(opt('fps', 60));
  const from = Number(opt('from', 0));
  const to = end;
  const frames = Math.round((to - from) * fps);
  const silent = music ? out.replace(/\.mp4$/, '-silent.mp4') : out;

  const ff = trackedSpawn(
    FFMPEG,
    ['-loglevel', 'error', '-y', '-f', 'image2pipe', '-framerate', String(fps), '-i', '-',
      '-c:v', 'libx264', '-preset', 'slow', '-crf', String(opt('crf', 16)), '-pix_fmt', 'yuv420p', '-movflags', '+faststart', silent],
    {stdio: ['pipe', 'inherit', 'inherit']},
  );
  const t0 = Date.now();
  for (let i = 0; i < frames; i++) {
    await draw(from + i / fps);
    const buf = await page.screenshot({type: 'jpeg', quality: 97});
    if (!ff.stdin.write(buf)) await new Promise((r) => ff.stdin.once('drain', r));
    if (i % fps === 0) process.stdout.write(`\r${i}/${frames} frames  ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  }
  ff.stdin.end();
  await new Promise((r) => ff.on('close', r));
  await browser.close();
  browser = null;
  console.log(`\r${frames}/${frames} frames  ${((Date.now() - t0) / 1000).toFixed(0)} s`);

  if (music) {
    await new Promise((r, j) =>
      trackedSpawn(
        FFMPEG,
        ['-loglevel', 'error', '-y', '-i', silent, '-ss', String(beats.start), '-i', music, '-map', '0:v', '-map', '1:a',
          '-c:v', 'copy', '-c:a', 'aac', '-b:a', '256k', '-af', `loudnorm=I=${opt('lufs', -21)}:TP=-3,afade=t=in:d=0.04,afade=t=out:st=${(to - from - 0.6).toFixed(2)}:d=0.6`,
          '-shortest', '-movflags', '+faststart', out],
        {stdio: 'inherit'},
      ).on('close', (c) => (c === 0 ? r() : j(new Error('mux failed')))),
    );
    fs.rmSync(silent);
  }
  console.log(`→ ${out}`);
  return 0;
}

let code = 1;
try {
  code = await main();
} catch (err) {
  console.error('[capture] error:', (err && err.stack) || err);
  code = 1;
} finally {
  clearTimeout(watchdog);
  // Always await cleanup here, even if a signal handler already started it: cleanup()
  // is memoized to the same in-flight promise, so this never duplicates work, but it
  // closes a real race where this path's own process.exit(code) below could otherwise
  // fire before that handler's browser.close()/SIGKILL fallback has actually finished.
  await cleanup();
}
process.exit(code);
