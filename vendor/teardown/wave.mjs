#!/usr/bin/env node
// Sound check: waveform with every timeline beat drawn on top, plus loudness.
//
//   node wave.mjs teardown-out/effort.wav --timeline sfx/cues/effort_timeline.json
//   node wave.mjs final.mp4 --timeline timeline.json   measure what you will actually post
//   without --timeline: waveform and loudness only, no beat lines
//   → <input>_wave.png next to the input
//
// Every hit in the waveform should sit on (or a hair after) a beat line.
//
// Leak safety: the loudness pass (ebur128) decodes the whole file and can run for real
// seconds on a long clip. It is tracked so SIGTERM/SIGINT/SIGHUP (or this script's own
// exit) cannot leave it running; prefer tools/run-guarded.py, which also owns the whole
// process group.
import {execFileSync, spawn} from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import {FFMPEG, FFPROBE} from './paths.mjs';

const argv = process.argv.slice(2);
const ti = argv.indexOf('--timeline');
const timelinePath = ti === -1 ? null : path.resolve(argv.splice(ti, 2)[1]);
if (!argv[0] || !fs.existsSync(argv[0])) {
  console.error('usage: node wave.mjs <file.wav|file.mp4> [--timeline timeline.json]');
  process.exit(1);
}
const input = path.resolve(argv[0]);

const TARGET_LUFS = -14;
const MAX_TRUE_PEAK = -1;
const W = 1800;
const H = 280;

// ---- leak-safe ffmpeg runner: tracks the in-flight child so a signal can kill it.
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
  console.error(`\n[wave] received ${sig}, cleaning up`);
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

function run(cmd, args, opts = {}) {
  return new Promise((resolve, reject) => {
    const child = spawn(cmd, args, opts);
    current = child;
    let stdout = '';
    let stderr = '';
    if (child.stdout) child.stdout.on('data', (d) => (stdout += d));
    if (child.stderr) child.stderr.on('data', (d) => (stderr += d));
    child.on('error', reject);
    child.on('close', (code) => {
      if (current === child) current = null;
      resolve({code, stdout, stderr});
    });
  });
}

// Metadata only (no decode): bounded to well under a second, so a plain sync call is fine.
const probed = Number(execFileSync(FFPROBE, ['-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', input]).toString());
const tl = timelinePath ? JSON.parse(fs.readFileSync(timelinePath, 'utf8')) : {duration: probed};
tl.duration ??= probed;
const beats = [];
const walk = (v, key) => {
  if (key === 'fps' || key === 'duration') return;
  if (typeof v === 'number') beats.push(v);
  else if (Array.isArray(v)) v.forEach((x) => walk(x));
  else if (v && typeof v === 'object') Object.entries(v).forEach(([k, x]) => walk(x, k));
};
walk(tl);

const marks = [...new Set(beats)]
  .map((t) => `drawbox=x=${Math.round((t / tl.duration) * W)}:y=0:w=2:h=ih:color=0xD97757@0.9:t=fill`)
  .join(',');
const seconds = `drawgrid=w=${W / tl.duration}:h=${H}:color=white@0.12`;

const png = input.replace(/\.[^.]+$/, '') + '_wave.png';
const waveRes = await run(FFMPEG, [
  '-loglevel', 'error', '-y', '-i', input,
  '-filter_complex',
  `color=c=0x141413:s=${W}x${H}:d=1[bg];` +
    `[0:a]aformat=channel_layouts=mono,showwavespic=s=${W}x${H}:scale=sqrt:colors=0x8A877F[w];` +
    `[bg][w]overlay=format=auto,${seconds}${marks ? ',' + marks : ''}`,
  '-frames:v', '1', png,
]);
if (waveRes.code !== 0) throw new Error(`ffmpeg (waveform) exited ${waveRes.code}\n${waveRes.stderr}`);

const r = await run(FFMPEG, ['-hide_banner', '-nostats', '-i', input, '-af', 'ebur128=peak=true', '-f', 'null', '-']);
const summary = r.stderr.slice(r.stderr.lastIndexOf('Summary'));
const lufs = Number(summary.match(/I:\s+(-?[\d.]+) LUFS/)?.[1]);
const peak = Number(summary.match(/Peak:\s+(-?[\d.]+) dBFS/)?.[1]);

console.log(`waveform: ${png} (${beats.length} beats in coral, grid = 1s)`);
console.log(`loudness: ${lufs} LUFS integrated (target ${TARGET_LUFS} ±1)`);
console.log(`true peak: ${peak} dBFS (keep ≤ ${MAX_TRUE_PEAK})`);
if (Math.abs(lufs - TARGET_LUFS) > 1) console.warn('WARN loudness off target: re-run the cue sheet, check TARGET_LUFS');
if (peak > MAX_TRUE_PEAK) console.warn('WARN true peak too hot: AAC encoding may clip, lower the master ceiling');
