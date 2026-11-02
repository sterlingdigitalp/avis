#!/usr/bin/env node
// Contact sheets from any rendered video: the review loop's eyes, and the way to
// read a clip before writing timecodes about it.
//
//   node sheet.mjs --video f.mp4 --timeline timeline.json   every beat, settled (+0.3s)
//   node sheet.mjs --video f.mp4 2.3 4.5 6.9                exact timestamps
//   node sheet.mjs --video f.mp4 --every 1                  one frame per second
//   node sheet.mjs --video f.mp4 --from 3 --to 5 --every 0.2   scrub a transition
//
// Options: --offset 0.3  --cols 4  --rows 2  --size 540  --name beats
// Sheets land next to the video in <video>_sheets/. Every tile is labelled t=.
//
// Leak safety: frame grabs run through ffmpeg one at a time. The in-flight one is
// tracked so a SIGTERM/SIGINT/SIGHUP (or this script's own exit) cannot leave it
// running; prefer tools/run-guarded.py, which also owns the whole process group.
import {execFileSync, spawn} from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import {FFMPEG, FFPROBE} from './paths.mjs';

const argv = process.argv.slice(2);
const opt = (name, fallback) => {
  const i = argv.indexOf(`--${name}`);
  if (i === -1) return fallback;
  const v = argv[i + 1];
  argv.splice(i, 2);
  return v;
};

const video = path.resolve(opt('video', ''));
const timelinePath = opt('timeline', null);
const offset = Number(opt('offset', '0.3'));
const every = opt('every', null);
const from = Number(opt('from', '0'));
const to = opt('to', null);
const cols = Number(opt('cols', '4'));
const rows = Number(opt('rows', '2'));
const size = Number(opt('size', '540'));
let name = opt('name', null);

if (!video || !fs.existsSync(video)) {
  console.error('pass --video <file.mp4>');
  process.exit(1);
}

// Metadata only (no decode): bounded to well under a second, so a plain sync call is fine.
const probe = (args) =>
  execFileSync(FFPROBE, ['-v', 'error', ...args, '-of', 'csv=p=0', video]).toString().trim();
const duration = Number(probe(['-show_entries', 'format=duration']));

const beats = () => {
  if (!timelinePath) {
    console.error('pass timestamps, --every, or --timeline <file.json>');
    process.exit(1);
  }
  const tl = JSON.parse(fs.readFileSync(timelinePath, 'utf8'));
  const out = [];
  const walk = (v, key) => {
    if (key === 'fps' || key === 'duration') return;
    if (typeof v === 'number') out.push(v);
    else if (Array.isArray(v)) v.forEach((x) => walk(x));
    else if (v && typeof v === 'object') Object.entries(v).forEach(([k, x]) => walk(x, k));
  };
  walk(tl);
  return out.map((t) => t + offset);
};

let times;
if (argv.length) {
  times = argv.map(Number);
  name ??= 'picked';
} else if (every) {
  const end = to === null ? duration : Number(to);
  times = [];
  for (let t = from; t <= end + 1e-6; t += Number(every)) times.push(Number(t.toFixed(3)));
  name ??= `every${every}`;
} else {
  times = beats();
  name ??= 'beats';
}

// ffmpeg returns no frame for a seek this close to the end of the file, so keep 50 ms off it
const eps = 0.05;
times = [...new Set(times.map((t) => Math.min(Math.max(t, 0), duration - eps).toFixed(2)))]
  .map(Number)
  .sort((a, b) => a - b);

const base = video.replace(/\.[^.]+$/, '') + '_sheets';
const dir = path.join(base, name);
fs.rmSync(dir, {recursive: true, force: true});
fs.mkdirSync(dir, {recursive: true});

// ---- leak-safe ffmpeg runner: one child at a time, tracked so a signal can kill it.
let current = null;
function hardKillSync() {
  if (current && current.pid && current.exitCode === null) {
    try {
      process.kill(current.pid, 'SIGKILL');
    } catch {}
  }
}
let exiting = false;
function onSignal(sig) {
  if (exiting) return;
  exiting = true;
  console.error(`\n[sheet] received ${sig}, cleaning up`);
  if (current && current.exitCode === null) {
    const t = setTimeout(() => {
      try {
        current.kill('SIGKILL');
      } catch {}
      process.exit(128);
    }, 2000);
    current.once('close', () => {
      clearTimeout(t);
      process.exit(128);
    });
    try {
      current.kill('SIGTERM');
    } catch {
      clearTimeout(t);
      process.exit(128);
    }
  } else {
    process.exit(128);
  }
}
process.on('exit', hardKillSync);
process.on('SIGTERM', () => onSignal('SIGTERM'));
process.on('SIGINT', () => onSignal('SIGINT'));
process.on('SIGHUP', () => onSignal('SIGHUP'));

function ffmpeg(args) {
  return new Promise((resolve, reject) => {
    const child = spawn(FFMPEG, ['-loglevel', 'error', '-y', ...args], {stdio: ['ignore', 'ignore', 'pipe']});
    current = child;
    let stderr = '';
    child.stderr.on('data', (d) => (stderr += d));
    child.on('error', reject);
    child.on('close', (code) => {
      if (current === child) current = null;
      code === 0 ? resolve() : reject(new Error(`ffmpeg exited ${code}\n${stderr}`));
    });
  });
}

let labels = true;
for (const [i, t] of times.entries()) {
  const file = path.join(dir, `${String(i).padStart(3, '0')}.png`);
  const scale = `scale=${size}:-2`;
  // Consolas ships with Windows; the colon in the drive letter is escaped for the filter parser.
  const label = `drawtext=fontfile='C\\:/Windows/Fonts/consola.ttf':text='t=${t.toFixed(2)}':x=10:y=10:fontsize=${Math.round(
    size / 24,
  )}:fontcolor=white:box=1:boxcolor=black@0.6:boxborderw=6`;
  const grab = (vf) => ffmpeg(['-ss', String(t), '-i', video, '-frames:v', '1', '-vf', vf, file]);
  if (labels) {
    try {
      await grab(`${scale},${label}`);
      continue;
    } catch {
      labels = false;
      console.warn('drawtext unavailable in this ffmpeg build; sheets will have no timestamps.');
    }
  }
  await grab(scale);
}

const perSheet = cols * rows;
const sheets = [];
for (let s = 0; s * perSheet < times.length; s++) {
  const out = path.join(base, `${name}-${s + 1}.png`);
  await ffmpeg([
    '-framerate', '1',
    '-start_number', String(s * perSheet),
    '-i', path.join(dir, '%03d.png'),
    '-vf', `tile=${cols}x${rows}:padding=6:color=0x333333`,
    '-frames:v', '1',
    out,
  ]);
  sheets.push(out);
}

console.log(`${times.length} frames from ${video} → ${sheets.length} sheet(s):`);
sheets.forEach((s) => console.log(`  ${s}`));
