// WebGPU counterpart of src/cpu.mjs. Runs in a browser page. See wgsl.mjs for the shaders.
import { Q, AXIS, FLUID, MAX_PARTS, ACC_COMPS, SCALE_F, SCALE_M, LINK_WORDS, CELL_WORDS } from './lattice.mjs';
import { wgslMain, wgslLink1, wgslLink2, wgslCells, wgslFill, dirGroups, PARAM_BYTES } from './wgsl.mjs';

export async function requestDevice() {
  const adapter = await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' });
  if (!adapter) throw new Error('no WebGPU adapter');
  const info = adapter.info ?? {};
  const want = ['maxBufferSize', 'maxStorageBufferBindingSize', 'maxStorageBuffersPerShaderStage', 'maxComputeWorkgroupSizeX', 'maxComputeInvocationsPerWorkgroup', 'maxComputeWorkgroupsPerDimension'];
  const requiredLimits = {};
  for (const k of want) requiredLimits[k] = adapter.limits[k];
  const device = await adapter.requestDevice({ requiredLimits });
  const errors = [];
  device.addEventListener('uncapturederror', (e) => { errors.push(e.error.message); console.error('webgpu', e.error.message.slice(0, 500)); });
  return { device, errors, adapterInfo: { vendor: info.vendor, architecture: info.architecture, device: info.device, description: info.description, isFallbackAdapter: info.isFallbackAdapter }, limits: requiredLimits };
}

const STORAGE = 0x80 | 0x04 | 0x08; // STORAGE | COPY_SRC | COPY_DST

export class GpuLbm {
  constructor(device, cfg) {
    this.device = device;
    this.cfg = cfg;
    const { nx, ny, nz } = cfg;
    this.nx = nx; this.ny = ny; this.nz = nz;
    this.n = nx * ny * nz;
    this.g = {
      nx, ny, nz,
      xMode: cfg.xMode ?? AXIS.inflow, yLowMode: cfg.yLowMode ?? AXIS.freeslip, yHighMode: cfg.yHighMode ?? AXIS.freeslip,
      zLowMode: cfg.zLowMode ?? AXIS.freeslip, zHighMode: cfg.zHighMode ?? AXIS.freeslip,
    };
    // Four buffers, always. Besides lifting the 4 GiB per-buffer ceiling, this works around a real
    // fault measured on this machine (Chromium 131, Metal, M5 Max): an invocation making 19 unrolled
    // writes to one storage buffer intermittently loses the last two, in about half of all cells
    // (differently from run to run). Four buffers of at most five writes each match the CPU to
    // round-off. GPU-versus-CPU in v6.mjs would catch any return of this.
    this.nb = cfg.nb ?? 4;
    this.layout = cfg.layout ?? 'aos';
    this.wg = cfg.wg ?? 64;
    this.groups = dirGroups(this.nb);
    this.useMask = !!cfg.mask;
    this.hasSponge = !!(cfg.sponge && cfg.sponge.strength > 0);
    this.opt = { nb: this.nb, layout: this.layout, wg: this.wg, useMask: this.useMask, hasSponge: this.hasSponge };
    this.A = this.groups.map((list) => device.createBuffer({ size: this.groupBytes(list), usage: STORAGE }));
    this.B = this.groups.map((list) => device.createBuffer({ size: this.groupBytes(list), usage: STORAGE }));
    this.parity = 0;
    this.params = device.createBuffer({ size: PARAM_BYTES, usage: 0x40 | 0x08 });
    this.acc = device.createBuffer({ size: MAX_PARTS * ACC_COMPS * 8, usage: STORAGE });
    this.accBytes = MAX_PARTS * ACC_COMPS * 8;
    this.accStaging = [];
    this.linkCap = 0; this.cellCap = 0;
    this.linkCount = 0; this.cellCount = 0;
    this.steps = 0;
    this.drhoFix = 0;
    this.pipes = {};
    const compile = (name, code) => {
      const m = device.createShaderModule({ code });
      m.getCompilationInfo().then((info) => { for (const e of info.messages) if (e.type === 'error') console.error(`WGSL ${name}:${e.lineNum}:${e.linePos} ${e.message}`); });
      this.pipes[name] = device.createComputePipeline({ layout: 'auto', compute: { module: m, entryPoint: 'main' } });
    };
    compile('main', wgslMain(this.g, this.opt));
    compile('link1', wgslLink1(this.g, this.opt));
    compile('link2', wgslLink2(this.g, this.opt));
    compile('cells', wgslCells(this.g, this.opt));
    compile('fill', wgslFill(this.g, this.opt));
    if (this.useMask) {
      this.maskCpu = cfg.mask;
      const bytes = Math.ceil(this.n / 4) * 4;
      this.maskBuf = device.createBuffer({ size: bytes, usage: STORAGE });
      this.uploadWholeMask();
    }
    this.bgMain = [this.bindMain(this.A, this.B), this.bindMain(this.B, this.A)];
    this.ensureLinkBuffers(1024);
    this.ensureCellBuffer(256);
    this.writeParams();
    this.fillUniform(cfg.u0 ?? [0, 0, 0]);
  }

  groupBytes(list) { return this.n * list.length * 4; }

  bufferEntries(bufs, first = 0) { return bufs.map((buffer, k) => ({ binding: first + k, resource: { buffer } })); }

  bindMain(src, dst) {
    const nb = this.nb;
    const e = [...this.bufferEntries(src, 0), ...this.bufferEntries(dst, nb), { binding: 2 * nb, resource: { buffer: this.params } }];
    if (this.useMask) e.push({ binding: 2 * nb + 1, resource: { buffer: this.maskBuf } });
    return this.device.createBindGroup({ layout: this.pipes.main.getBindGroupLayout(0), entries: e });
  }

  ensureLinkBuffers(count) {
    if (count <= this.linkCap) return;
    const cap = Math.ceil(count * 1.5);
    this.linkRec?.destroy(); this.linkVal?.destroy();
    this.linkRec = this.device.createBuffer({ size: cap * LINK_WORDS * 4, usage: STORAGE });
    this.linkVal = this.device.createBuffer({ size: cap * 4, usage: STORAGE });
    this.linkCap = cap;
    const nb = this.nb;
    const mk = (pipe, bufs, extra) => this.device.createBindGroup({ layout: pipe.getBindGroupLayout(0), entries: [...this.bufferEntries(bufs, 0), ...extra] });
    const e1 = [{ binding: nb, resource: { buffer: this.linkRec } }, { binding: nb + 1, resource: { buffer: this.linkVal } }, { binding: nb + 2, resource: { buffer: this.params } }, { binding: nb + 3, resource: { buffer: this.acc } }];
    const e2 = [{ binding: nb, resource: { buffer: this.linkRec } }, { binding: nb + 1, resource: { buffer: this.linkVal } }, { binding: nb + 2, resource: { buffer: this.params } }];
    this.bgLink1 = [mk(this.pipes.link1, this.A, e1), mk(this.pipes.link1, this.B, e1)];
    this.bgLink2 = [mk(this.pipes.link2, this.A, e2), mk(this.pipes.link2, this.B, e2)];
  }

  ensureCellBuffer(count) {
    if (count <= this.cellCap) return;
    const cap = Math.ceil(count * 1.5);
    this.cellRec?.destroy();
    this.cellRec = this.device.createBuffer({ size: cap * CELL_WORDS * 4, usage: STORAGE });
    this.cellCap = cap;
    const nb = this.nb;
    const e = [{ binding: nb, resource: { buffer: this.cellRec } }, { binding: nb + 1, resource: { buffer: this.params } }, { binding: nb + 2, resource: { buffer: this.acc } }];
    const mk = (bufs) => this.device.createBindGroup({ layout: this.pipes.cells.getBindGroupLayout(0), entries: [...this.bufferEntries(bufs, 0), ...e] });
    this.bgCells = [mk(this.A), mk(this.B)];
  }

  writeParams() {
    const { inletU = [0, 0, 0], inletRampSteps = 0, sponge } = this.cfg;
    const frac = inletRampSteps > 0 ? Math.min(1, this.steps / inletRampSteps) : 1;
    const U = [Math.fround(inletU[0] * frac), Math.fround(inletU[1] * frac), Math.fround(inletU[2] * frac)];
    const fr = Math.fround;
    const buf = new ArrayBuffer(PARAM_BYTES);
    const f = new Float32Array(buf), u = new Uint32Array(buf);
    f[0] = this.cfg.tau0; f[1] = this.cfg.csSmag ?? 0; f[2] = U[0]; f[3] = U[1];
    f[4] = U[2]; f[5] = fr(fr(fr(U[0] * U[0]) + fr(U[1] * U[1])) + fr(U[2] * U[2])); f[6] = sponge?.strength ?? 0; f[7] = this.drhoFix;
    u[8] = this.linkCount; u[9] = this.cellCount; u[10] = (this.cfg.giTerm ?? true) ? 1 : 0; u[11] = (this.cfg.localRho ?? true) ? 1 : 0;
    f[12] = sponge?.xLow ?? 0; f[13] = sponge?.xHigh ?? 0; f[14] = sponge?.yLow ?? 0; f[15] = sponge?.yHigh ?? 0;
    f[16] = sponge?.zLow ?? 0; f[17] = sponge?.zHigh ?? 0;
    this.device.queue.writeBuffer(this.params, 0, buf);
  }

  fillUniform(u0) {
    const save = this.cfg.inletU, saveR = this.cfg.inletRampSteps;
    this.cfg.inletU = u0; this.cfg.inletRampSteps = 0;
    this.writeParams();
    this.cfg.inletU = save; this.cfg.inletRampSteps = saveR;
    const nb = this.nb;
    const bg = this.device.createBindGroup({ layout: this.pipes.fill.getBindGroupLayout(0), entries: [...this.bufferEntries(this.A, 0), { binding: nb, resource: { buffer: this.params } }] });
    const enc = this.device.createCommandEncoder();
    const pass = enc.beginComputePass();
    pass.setPipeline(this.pipes.fill);
    pass.setBindGroup(0, bg);
    pass.dispatchWorkgroups(Math.ceil(this.nx / this.wg), this.ny, this.nz);
    pass.end();
    this.device.queue.submit([enc.finish()]);
    this.writeParams();
  }

  uploadWholeMask() {
    const bytes = this.maskCpu;
    const CH = 1 << 26;
    for (let o = 0; o < bytes.length; o += CH) {
      let len = Math.min(CH, bytes.length - o);
      if (len % 4) {
        const pad = new Uint8Array(4 * Math.ceil(len / 4)).fill(255);
        pad.set(bytes.subarray(o, o + len));
        this.device.queue.writeBuffer(this.maskBuf, o, pad);
      } else this.device.queue.writeBuffer(this.maskBuf, o, bytes.buffer, bytes.byteOffset + o, len);
    }
  }

  /** Install a frame from BoundaryEngine. Cell changes run once on the next step. */
  setFrame(frame) {
    const q = this.device.queue;
    this.ensureLinkBuffers(Math.max(frame.linkCount, 1));
    if (frame.linkCount) q.writeBuffer(this.linkRec, 0, frame.linkBytes);
    this.linkCount = frame.linkCount;
    this.cellCount = frame.initial ? 0 : frame.cellCount;
    if (this.cellCount) {
      this.ensureCellBuffer(this.cellCount);
      q.writeBuffer(this.cellRec, 0, frame.cellBytes);
    }
    if (this.useMask) {
      if (frame.initial) this.uploadWholeMask();
      else this.patchMask(frame.maskEdits);
    }
    this.writeParams();
    this.pendingCells = this.cellCount > 0;
  }

  patchMask(cells) {
    if (!cells.length) return;
    const words = new Set();
    for (const c of cells) words.add(c >>> 2);
    const m = this.maskCpu;
    const sorted = [...words].sort((a, b) => a - b);
    let i = 0;
    while (i < sorted.length) {
      let j = i;
      while (j + 1 < sorted.length && sorted[j + 1] === sorted[j] + 1) j++;
      const start = sorted[i] * 4, end = (sorted[j] + 1) * 4;
      if (end <= m.length) this.device.queue.writeBuffer(this.maskBuf, start, m.buffer, m.byteOffset + start, end - start);
      else {
        const pad = new Uint8Array(end - start).fill(255);
        pad.set(m.subarray(start, Math.min(end, m.length)));
        this.device.queue.writeBuffer(this.maskBuf, start, pad);
      }
      i = j + 1;
    }
  }

  encodeStep(enc) {
    const p = this.parity;
    const pass = enc.beginComputePass();
    if (this.pendingCells) {
      pass.setPipeline(this.pipes.cells);
      pass.setBindGroup(0, this.bgCells[p]);
      pass.dispatchWorkgroups(Math.ceil(this.cellCount / 16));
    }
    if (this.linkCount) {
      const g = Math.ceil(this.linkCount / 64);
      pass.setPipeline(this.pipes.link1);
      pass.setBindGroup(0, this.bgLink1[p]);
      pass.dispatchWorkgroups(g);
      pass.setPipeline(this.pipes.link2);
      pass.setBindGroup(0, this.bgLink2[p]);
      pass.dispatchWorkgroups(g);
    }
    pass.setPipeline(this.pipes.main);
    pass.setBindGroup(0, this.bgMain[p]);
    pass.dispatchWorkgroups(Math.ceil(this.nx / this.wg), this.ny, this.nz);
    pass.end();
    this.parity = 1 - p;
    this.steps++;
  }

  /** Advance. Uniforms only change between submits, so a block needing per-step ramps submits one by one. */
  step(count = 1) {
    const ramp = this.cfg.inletRampSteps ?? 0;
    let left = count;
    while (left > 0) {
      const ramping = this.steps < ramp;
      const once = this.pendingCells || ramping || this.drhoFix !== 0;
      const k = once ? 1 : left;
      const needParams = ramping || this.pendingCells || this.drhoFix !== 0 || this.paramsDirty;
      if (needParams) { this.writeParams(); this.paramsDirty = false; }
      const enc = this.device.createCommandEncoder();
      for (let s = 0; s < k; s++) this.encodeStep(enc);
      this.device.queue.submit([enc.finish()]);
      if (this.pendingCells) { this.pendingCells = false; this.cellCount = 0; this.paramsDirty = true; }
      if (this.drhoFix !== 0) { this.drhoFix = 0; this.paramsDirty = true; }
      left -= k;
    }
  }

  async finish() { await this.device.queue.onSubmittedWorkDone(); }

  async takeForces() {
    const staging = this.accStaging.pop() ?? this.device.createBuffer({ size: this.accBytes, usage: 0x01 | 0x08 });
    const enc = this.device.createCommandEncoder();
    enc.copyBufferToBuffer(this.acc, 0, staging, 0, this.accBytes);
    enc.clearBuffer(this.acc);
    this.device.queue.submit([enc.finish()]);
    await staging.mapAsync(1);
    const u = new Uint32Array(staging.getMappedRange());
    const i32 = new Int32Array(u.buffer);
    const val = (k) => i32[k * 2 + 1] * 4294967296 + u[k * 2];
    const out = [];
    for (let p = 0; p < MAX_PARTS; p++) {
      const o = p * ACC_COMPS;
      out.push({ F: [val(o) / SCALE_F, val(o + 1) / SCALE_F, val(o + 2) / SCALE_F], M: [val(o + 3) / SCALE_M, val(o + 4) / SCALE_M, val(o + 5) / SCALE_M], dMass: val(o + 6) / SCALE_F });
    }
    staging.unmap();
    this.accStaging.push(staging);
    return out;
  }

  /** Copy one current-state buffer range to a CPU array. */
  async readBufferRange(buf, offset, size) {
    const staging = this.device.createBuffer({ size, usage: 0x01 | 0x08 });
    const enc = this.device.createCommandEncoder();
    enc.copyBufferToBuffer(buf, offset, staging, 0, size);
    this.device.queue.submit([enc.finish()]);
    await staging.mapAsync(1);
    const out = staging.getMappedRange().slice(0);
    staging.unmap();
    staging.destroy();
    return out;
  }

  current() { return this.parity === 0 ? this.A : this.B; }

  /** Whole post-collision state in canonical order (cell*19+dir). Small grids only. */
  async readAll() {
    const cur = this.current();
    const out = new Float32Array(this.n * Q);
    for (let k = 0; k < this.nb; k++) {
      const a = new Float32Array(await this.readBufferRange(cur[k], 0, cur[k].size));
      const list = this.groups[k];
      for (let c = 0; c < this.n; c++) for (let j = 0; j < list.length; j++) {
        const v = this.layout === 'soa' ? a[j * this.n + c] : a[c * list.length + j];
        out[c * Q + list[j]] = v;
      }
    }
    return out;
  }

  /** Mass and momentum by chunked read, safe at any grid size. Sums in f64. */
  async massMomentum() {
    if (this.layout === 'soa') throw new Error('massMomentum: aos only');
    const cur = this.current();
    let m = 0;
    const CHUNK_CELLS = 1 << 22;
    let cells = 0;
    for (let k = 0; k < this.nb; k++) {
      const gs = this.groups[k].length;
      for (let c0 = 0; c0 < this.n; c0 += CHUNK_CELLS) {
        const nc = Math.min(CHUNK_CELLS, this.n - c0);
        const a = new Float32Array(await this.readBufferRange(cur[k], c0 * gs * 4, nc * gs * 4));
        const mask = this.useMask ? this.maskCpu : null;
        for (let c = 0; c < nc; c++) {
          if (mask && mask[c0 + c] !== FLUID) continue;
          if (k === 0) cells++;
          for (let j = 0; j < gs; j++) m += a[c * gs + j];
        }
      }
    }
    return { deltaMass: m, cells };
  }

  /** Stream the state to the local server under out/lbm. Atomic: a partial file never looks valid. */
  async saveCheckpoint(path, meta) {
    const cur = this.current();
    const tmp = `${path}.tmp`;
    await fetch(`/${tmp}`, { method: 'DELETE' });
    // 16 MiB per request: Playwright's CDP session receives every request body as a base64 string in
    // node, and a 256 MiB body exceeds V8's maximum string length and kills the harness.
    const CHUNK = 1 << 24;
    for (let k = 0; k < this.nb; k++) {
      for (let o = 0; o < cur[k].size; o += CHUNK) {
        const size = Math.min(CHUNK, cur[k].size - o);
        const data = await this.readBufferRange(cur[k], o, size);
        const r = await fetch(`/${tmp}?append=1`, { method: 'PUT', body: data });
        if (!r.ok) throw new Error(`checkpoint write ${r.status}`);
      }
    }
    await fetch(`/${tmp}?to=/${path}`, { method: 'PATCH' });
    const m = { ...meta, steps: this.steps, nb: this.nb, layout: this.layout, nx: this.nx, ny: this.ny, nz: this.nz };
    await fetch(`/${path}.json`, { method: 'PUT', body: JSON.stringify(m) });
    return m;
  }

  async loadCheckpoint(path) {
    const r = await fetch(`/${path}.json`, { cache: 'no-store' });
    if (!r.ok) return null;
    const m = await r.json();
    if (m.nx !== this.nx || m.ny !== this.ny || m.nz !== this.nz || m.nb !== this.nb || m.layout !== this.layout) throw new Error('checkpoint grid mismatch');
    const cur = this.A;
    this.parity = 0;
    let fileOff = 0;
    const CHUNK = 1 << 28;
    for (let k = 0; k < this.nb; k++) {
      for (let o = 0; o < cur[k].size; o += CHUNK) {
        const size = Math.min(CHUNK, cur[k].size - o);
        const rr = await fetch(`/${path}`, { headers: { Range: `bytes=${fileOff}-${fileOff + size - 1}` }, cache: 'no-store' });
        if (rr.status !== 206) throw new Error(`checkpoint read ${rr.status}`);
        const ab = await rr.arrayBuffer();
        if (ab.byteLength !== size) throw new Error('short checkpoint read');
        this.device.queue.writeBuffer(cur[k], o, ab);
        await this.device.queue.onSubmittedWorkDone();
        fileOff += size;
      }
    }
    this.steps = m.steps;
    this.paramsDirty = true;
    return m;
  }

  destroy() {
    for (const b of [...this.A, ...this.B, this.params, this.acc, this.linkRec, this.linkVal, this.cellRec, this.maskBuf, ...this.accStaging]) b?.destroy();
  }
}
