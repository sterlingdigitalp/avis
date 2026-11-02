#!/usr/bin/env node
// The one command: renders the film from the current design and flight logs.
//
//   node film/render.mjs                    final cut, film/edit.json  -> out/film/avis.mp4
//   node film/render.mjs --edit preview     film/edit.preview.json     -> out/film/preview.mp4
//   options: --samples N (Blender samples, overrides the edit)  --only plates|capture|mix  --dry (plan and estimate)
//            --keep (keep frames on disk even if a signature changed)
//
// Resumable by shot and frame: Blender frames already on disk are kept unless the design, the log or the shot's
// settings changed (out/film/state.json); capture chunks likewise. Every GPU job runs through
// tools/run-guarded.py --gpu with a timeout under 10 minutes, so no lock hold is longer than that.
import { spawnSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.join(HERE, '..');
const OUT = path.join(ROOT, 'out', 'film');
const SITE = path.join(OUT, 'site');
const BLENDER = '/Applications/Blender.app/Contents/MacOS/Blender';
const HOLD_S = 540; // target GPU time per lock hold, under the 10 min rule
const a = process.argv.slice(2);
const opt = (k, d) => (a.includes(`--${k}`) ? a[a.indexOf(`--${k}`) + 1] : d);
const editName = opt('edit', 'edit');
const editFile = path.join(HERE, editName === 'edit' ? 'edit.json' : `edit.${editName}.json`);
const outName = editName === 'edit' ? 'avis' : editName;
const only = opt('only', null);
const dry = a.includes('--dry');
// --keep: keep Blender frames even if their inputs' signature changed (when the change is known not to touch them)
const keep = a.includes('--keep');

const sh = (cmd, args, { env = {}, quiet = false } = {}) => {
  const r = spawnSync(cmd, args, { cwd: ROOT, stdio: quiet ? 'pipe' : 'inherit', env: { ...process.env, ...env }, encoding: 'utf8' });
  if (r.status !== 0) throw new Error(`${cmd} ${args.slice(0, 6).join(' ')} exited ${r.status}\n${r.stderr ?? ''}`);
  return r.stdout;
};
const node = (script, args, env) => sh(process.execPath, ['--import', './film/lib/three-hooks.mjs', script, ...args], { env });
const guarded = (timeout, log, cmd, gpu = true) => sh('python3', ['tools/run-guarded.py', '--timeout', String(timeout), ...(gpu ? ['--gpu'] : []), '--log', log, '--', ...cmd]);
const hashOf = (...parts) => createHash('sha256').update(parts.map((p) => (typeof p === 'string' ? p : JSON.stringify(p))).join('|')).digest('hex').slice(0, 16);
const fileSig = (f) => (fs.existsSync(f) ? `${fs.statSync(f).size}:${fs.statSync(f).mtimeMs}` : 'missing');
const readJson = (f, d) => (fs.existsSync(f) ? JSON.parse(fs.readFileSync(f, 'utf8')) : d);
const log = (...m) => console.log('[render]', ...m);

// ---- 1. data, pages, bird ---------------------------------------------------------------------------------------
const edit = readJson(editFile);
const fps = edit.fps ?? 30;
const design = edit.design ?? 'larus-gl-2';
fs.mkdirSync(path.join(OUT, 'logs'), { recursive: true });
const state = readJson(path.join(OUT, 'state.json'), { plates: {}, chunks: {} });
const saveState = () => fs.writeFileSync(path.join(OUT, 'state.json'), JSON.stringify(state, null, 1));

node('film/gen_data.mjs', [design]);
const designSha = readJson(path.join(OUT, 'gen', 'model-data.json')).design_sha256;
buildPages();
// the bird is the design plus the code that builds it: a change to either re-exports it and re-renders its plates
const birdSig = hashOf(designSha, ...['bird.js', 'model.js'].map((f) => fs.readFileSync(path.join(ROOT, 'sims', 'avis-shared', f), 'utf8')));
const glb = path.join(OUT, 'bird', `${design}.glb`);
if (!fs.existsSync(glb) || readJson(path.join(OUT, 'bird', 'sig.json'), {}).sig !== birdSig) {
  node('film/export_glb.mjs', [design]);
  fs.writeFileSync(path.join(OUT, 'bird', 'sig.json'), JSON.stringify({ sig: birdSig }));
}
// plates depend on what Blender sees: the GLB's bytes and the bake, not on page-only code
const glbSha = createHash('sha256').update(fs.readFileSync(glb)).digest('hex').slice(0, 16);

// ---- 2. resolve the edit: start times, plate frame ranges, HUD samples ----------------------------------------
let t = 0;
const segs = edit.segments.map((s) => {
  const r = { ...s, start: +t.toFixed(4) };
  t += s.dur;
  if (s.kind === 'bl') {
    const slow = s.slow ?? 1;
    // flight shots come from the log only: the window must lie inside it
    const L = readJson(path.join(ROOT, 'out', 'flight', `${s.scenario}.json`));
    const span = L.columns.t.at(-1) - L.columns.t[0];
    if (s.start_s < 0 || s.start_s + s.dur / slow > span) throw new Error(`${s.id}: ${s.start_s}..${(s.start_s + s.dur / slow).toFixed(1)} s is outside ${s.scenario} (${span.toFixed(1)} s)`);
    const i0 = Math.round(s.start_s * fps * slow);
    const n = Math.round(s.dur * fps);
    r.plate = { dir: s.id, i0, n, fps, slow };
  }
  return r;
});
const duration = t;
const resolved = { ...edit, fps, duration, segments: segs };
fs.mkdirSync(path.join(SITE, 'data'), { recursive: true });
fs.writeFileSync(path.join(SITE, 'data', `${editName}.json`), JSON.stringify(resolved));
for (const s of segs.filter((x) => x.kind === 'bl')) writeSegData(s);
const blFrames = segs.filter((s) => s.kind === 'bl').reduce((p, s) => p + s.plate.n, 0);
const samples = +opt('samples', edit.blender?.samples ?? 64);
const res = edit.blender?.res ?? '1920x1080';
const timing = readJson(path.join(OUT, 'timing.json'), {});
const perFrame = timing[`${res}@${samples}`] ?? 30;
log(`${editName}: ${duration.toFixed(1)} s at ${fps} fps, ${blFrames} Blender frames at ${res}, ${samples} samples (~${perFrame.toFixed(1)} s/frame measured -> ~${((blFrames * perFrame) / 3600).toFixed(2)} GPU h)`);
if (dry) process.exit(0);

// ---- 3. trees: the environment's note, prebuild on the CPU if trees.py changed ---------------------------------
const treesSig = hashOf(fs.readFileSync(path.join(ROOT, 'env', 'trees.py'), 'utf8'));
if (state.trees !== treesSig && (!only || only === 'plates')) {
  log('env/trees.py changed since the last render: prebuilding trees on the CPU');
  guarded(3000, path.join(OUT, 'logs', 'prebuild_trees.log'), ['python3', 'env/prebuild_trees.py'], false);
  state.trees = treesSig;
  saveState();
}

// ---- 4. Blender plates, in chunks that fit one lock hold ----------------------------------------------------------
if (!only || only === 'plates') {
  for (const s of segs.filter((x) => x.kind === 'bl')) renderPlates(s);
}

// ---- 5. capture the page in chunks, then concat, score and mux ---------------------------------------------------
if (!only || only === 'capture' || only === 'mix') {
  const chunks = [];
  for (const s of segs) {
    // split long segments so a chunk never takes more than a few minutes
    for (let c0 = s.start; c0 < s.start + s.dur - 1e-6; c0 += 20) chunks.push([c0, Math.min(s.start + s.dur, c0 + 20), s]);
  }
  const files = [];
  const pageSig = hashOf(fileSig(path.join(SITE, 'avis-film.html')), designSha);
  for (const [c0, c1, s] of chunks) {
    const f = path.join(OUT, 'chunks', editName, `${c0.toFixed(3)}-${c1.toFixed(3)}.mp4`);
    const sig = hashOf(pageSig, s, s.kind === 'bl' ? state.plates[s.id] : '', c0, c1);
    files.push(f);
    if (only === 'mix') continue;
    if (fs.existsSync(f) && state.chunks[f] === sig) continue;
    fs.mkdirSync(path.dirname(f), { recursive: true });
    guarded(590, path.join(OUT, 'logs', `capture-${editName}.log`), [process.execPath, 'film/capture.mjs', '--site', SITE, '--edit', editName, '--from', String(c0), '--to', String(c1), '--fps', String(fps), '--out', f]);
    state.chunks[f] = sig;
    saveState();
  }
  const list = path.join(OUT, 'chunks', editName, 'list.txt');
  fs.writeFileSync(list, files.map((f) => `file '${f}'`).join('\n'));
  const silent = path.join(OUT, `${outName}-silent.mp4`);
  sh('ffmpeg', ['-v', 'error', '-y', '-f', 'concat', '-safe', '0', '-i', list, '-c', 'copy', silent]);
  // score on the same timeline
  const tl = path.join(OUT, `${outName}-timeline.json`);
  fs.writeFileSync(tl, JSON.stringify({ duration, fps, segments: segs.map(({ id, kind, seq, shot, start, dur, slow }) => ({ id, kind, seq, shot, start, dur, slow })) }));
  const wav = path.join(OUT, `${outName}-score.wav`);
  sh('python3', ['vendor/teardown/run_sfx.py', 'sims/avis-shared/score.py', '--timeline', tl, '--out', wav]);
  const final = path.join(OUT, `${outName}.mp4`);
  sh('ffmpeg', ['-v', 'error', '-y', '-i', silent, '-i', wav, '-map', '0:v', '-map', '1:a', '-c:v', 'copy', '-c:a', 'aac', '-b:a', '256k', '-shortest', '-movflags', '+faststart', final]);
  log(`wrote ${final}`);
}

// ---------------------------------------------------------------------------------------------------------------
function buildPages() {
  const env = { TEARDOWN_OUT: path.join(OUT, 'teardown') };
  for (const p of ['avis-film', 'avis-live', 'avis-lab']) {
    if (!fs.existsSync(path.join(ROOT, 'sims', p, 'main.js'))) continue;
    sh(process.execPath, ['vendor/teardown/sims/build.mjs', `./sims/${p}`], { env, quiet: true });
    fs.mkdirSync(SITE, { recursive: true });
    fs.copyFileSync(path.join(OUT, 'teardown', 'dist', `${p}.html`), path.join(SITE, `${p}.html`));
  }
  node('film/site_data.mjs', [design]);
}

function writeSegData(s) {
  const L = readJson(path.join(ROOT, 'out', 'flight', `${s.scenario}.json`));
  const c = L.columns;
  const step = 1 / (fps * s.plate.slow);
  const out = { t: [], airspeed: [], power_w: [], battery_pct: [] };
  // power swings through every wingbeat; the HUD shows its mean over the flap cycle around the frame (phase-based)
  const meanPower = (k) => {
    let a = k, b = k, unw = 0;
    while (a > 0 && unw < Math.PI) {
      let d = c.phase[a] - c.phase[a - 1];
      d -= 2 * Math.PI * Math.round(d / (2 * Math.PI));
      unw += Math.abs(d);
      a--;
    }
    unw = 0;
    while (b < c.t.length - 1 && unw < Math.PI) {
      let d = c.phase[b + 1] - c.phase[b];
      d -= 2 * Math.PI * Math.round(d / (2 * Math.PI));
      unw += Math.abs(d);
      b++;
    }
    let sum = 0;
    for (let q = a; q <= b; q++) sum += c.power_w[q];
    return sum / (b - a + 1);
  };
  let k = 0;
  for (let i = 0; i < s.plate.n; i++) {
    const tq = c.t[0] + (s.plate.i0 + i) * step;
    while (k < c.t.length - 2 && c.t[k + 1] < tq) k++;
    out.t.push(+tq.toFixed(3));
    out.airspeed.push(c.airspeed[k]);
    out.power_w.push(meanPower(k));
    out.battery_pct.push(c.battery_pct[k]);
  }
  fs.writeFileSync(path.join(SITE, 'data', `seg-${s.id}.json`), JSON.stringify(out));
}

function renderPlates(s) {
  const p = s.plate;
  const dir = path.join(OUT, 'shots', s.id);
  const logF = path.join(ROOT, 'out', 'flight', `${s.scenario}.json`);
  const bl = { ...edit.blender, samples, ...(s.blender ?? {}) };
  // the bake first (about a second): the frames depend on its bytes, the GLB's bytes and the shot, nothing else
  const bake = path.join(OUT, 'bake', `${s.id}`);
  fs.mkdirSync(path.dirname(bake), { recursive: true });
  node('film/bake_pose.mjs', [logF, '--fps', String(fps * p.slow), '--from', String(p.i0), '--to', String(p.i0 + p.n), '--out', bake]);
  const bakeSha = createHash('sha256').update(fs.readFileSync(`${bake}.bin`)).digest('hex').slice(0, 16);
  const sig = hashOf(glbSha, bakeSha, s.scenario, s.shot, p, bl, fileSig(path.join(HERE, 'blender_shot.py')), fileSig(path.join(HERE, 'blender_bird.py')), fileSig(path.join(ROOT, 'env', 'shots.py')));
  if (state.plates[s.id] !== sig && fs.existsSync(dir) && !keep) {
    log(`${s.id}: inputs changed, re-rendering its frames`);
    fs.rmSync(dir, { recursive: true, force: true });
  }
  fs.mkdirSync(dir, { recursive: true });
  fs.mkdirSync(path.join(SITE, 'plates'), { recursive: true });
  const link = path.join(SITE, 'plates', s.id);
  if (!fs.existsSync(link)) fs.symlinkSync(dir, link);
  let missing = 0;
  for (let i = p.i0; i < p.i0 + p.n; i++) if (!fs.existsSync(path.join(dir, `f_${String(i).padStart(5, '0')}.png`))) missing++;
  if (!missing) {
    state.plates[s.id] = sig;
    saveState();
    return;
  }
  const key = `${bl.res ?? res}@${bl.samples}`;
  const per = readJson(path.join(OUT, 'timing.json'), {})[key] ?? 30;
  const chunk = Math.max(1, Math.floor((HOLD_S - 75) / per)); // 75 s for scene setup per hold
  // Blender on Metal occasionally dies loading its kernels (SIGTRAP/SIGABRT inside MTLLibraryBuilder, not in our
  // code); frames on disk are kept, so a failed hold is simply run again, up to a limit.
  let failures = 0;
  for (;;) {
    const todo = [];
    for (let i = p.i0; i < p.i0 + p.n; i++) if (!fs.existsSync(path.join(dir, `f_${String(i).padStart(5, '0')}.png`))) todo.push(i);
    if (!todo.length) break;
    const lo = todo[0], hi = todo[Math.min(todo.length, chunk) - 1] + 1;
    log(`${s.id}: frames ${lo}..${hi - 1} (${hi - lo}) at ${key}`);
    const args = [`scenario=${s.scenario}`, `shot=${s.shot}`, `glb=${glb}`, `bake=${bake}`, `frames=${lo}:${hi}`, `span=${p.i0}:${p.i0 + p.n}`, `out=${dir}`, `fps=${fps}`, `slow=${p.slow}`,
      `res=${bl.res ?? res}`, `samples=${bl.samples}`, `sun=${bl.sun ?? 22}`, `world=${bl.world ?? 0.4}`, `haze=${bl.haze ?? 1.6e-5}`, `look=${bl.look ?? 'AgX - Medium High Contrast'}`, `exposure=${bl.exposure ?? 0}`];
    if (s.lens) args.push(`lens=${s.lens}`);
    try {
      guarded(595, path.join(OUT, 'logs', `blender-${s.id}.log`), [BLENDER, '--background', '--factory-startup', '--python-exit-code', '7', '--python', 'film/blender_shot.py', '--', ...args]);
    } catch (e) {
      failures++;
      log(`${s.id}: Blender failed (${failures}): ${String(e.message).split('\n')[0]}`);
      if (failures > 6) throw e;
    }
    // learn the real per-frame time for the next estimate
    const tf = readJson(path.join(dir, 'timing.json'), {});
    const v = Object.values(tf);
    if (v.length) {
      const all = readJson(path.join(OUT, 'timing.json'), {});
      all[key] = v.reduce((x, y) => x + y, 0) / v.length;
      fs.writeFileSync(path.join(OUT, 'timing.json'), JSON.stringify(all, null, 1));
    }
  }
  state.plates[s.id] = sig;
  saveState();
}
