// Reports the WebGPU adapter and the real buffer limits, then tries actual allocations, because
// advertised limits and what Metal will back with memory are different questions.
import { openPage, cleanup, watchdog } from '../src/harness.mjs';
watchdog(170000);
const execs = process.argv.slice(2);
const { page } = await openPage({ executablePath: execs[0] });
const r = await page.evaluate(async () => {
  const adapter = await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' });
  const info = adapter.info ?? {};
  const lim = {};
  for (const k in adapter.limits) lim[k] = adapter.limits[k];
  const device = await adapter.requestDevice({ requiredLimits: {
    maxBufferSize: adapter.limits.maxBufferSize,
    maxStorageBufferBindingSize: adapter.limits.maxStorageBufferBindingSize,
    maxStorageBuffersPerShaderStage: adapter.limits.maxStorageBuffersPerShaderStage,
    maxComputeWorkgroupSizeX: adapter.limits.maxComputeWorkgroupSizeX,
    maxComputeInvocationsPerWorkgroup: adapter.limits.maxComputeInvocationsPerWorkgroup,
  } });
  const tries = [];
  const held = [];
  for (const gb of [1, 2, 3.99, 4, 8, 16, 16, 16, 16]) {
    device.pushErrorScope('out-of-memory'); device.pushErrorScope('validation');
    const size = Math.floor(gb * 2 ** 30 / 4) * 4;
    let b = null, thrown = null;
    try { b = device.createBuffer({ size, usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_SRC | GPUBufferUsage.COPY_DST }); } catch (e) { thrown = String(e); }
    const v = await device.popErrorScope(); const o = await device.popErrorScope();
    tries.push({ gb, ok: !!b && !v && !o, validation: v?.message?.slice(0, 120), oom: o?.message?.slice(0, 120), thrown });
    if (b && !v && !o) held.push(b); else if (b) b.destroy();
  }
  const total = held.reduce((a, b) => a + b.size, 0) / 2 ** 30;
  held.forEach((b) => b.destroy());
  return { ua: navigator.userAgent, adapter: { vendor: info.vendor, architecture: info.architecture, device: info.device, description: info.description, isFallback: info.isFallbackAdapter }, limits: lim, tries, totalHeldGiB: total };
});
console.log(JSON.stringify({ ua: r.ua, adapter: r.adapter, tries: r.tries, totalHeldGiB: r.totalHeldGiB,
  limits: { maxBufferSize: r.limits.maxBufferSize, maxStorageBufferBindingSize: r.limits.maxStorageBufferBindingSize, maxStorageBuffersPerShaderStage: r.limits.maxStorageBuffersPerShaderStage, maxComputeWorkgroupSizeX: r.limits.maxComputeWorkgroupSizeX, maxComputeInvocationsPerWorkgroup: r.limits.maxComputeInvocationsPerWorkgroup, maxComputeWorkgroupsPerDimension: r.limits.maxComputeWorkgroupsPerDimension } }, null, 1));
cleanup();
setTimeout(() => process.exit(0), 1000);
