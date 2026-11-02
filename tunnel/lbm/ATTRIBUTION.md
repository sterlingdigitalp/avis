# Attribution

This directory is derived from the D3Q19 lattice Boltzmann solver in `~/tdf2/src/cfd/` (the AeroFit
tunnel project, same author, used with permission as the Avis SPEC directs). `~/tdf2` was not edited.

| Here | Derived from | What changed |
|---|---|---|
| `src/lattice.mjs` | `tdf2/src/cfd/lattice.ts` | Velocity set and ordering unchanged. Smagorinsky closure unchanged. Plain JS. |
| `src/cpu.mjs` | `tdf2/src/cfd/lbmCpu.ts` | Regularised BGK, local Smagorinsky and the sponge come from there. Rewritten for moving bodies, shifted populations, true specular mirrors. |
| `src/wgsl.mjs`, `src/gpu.mjs` | `tdf2/src/cfd/shaders.ts`, `lbm.ts` | Same collision model. New: merged pull-collide pass with unrolled directions, direction-grouped buffers beyond the 4 GiB limit, sparse link and cell-change passes, 64-bit fixed-point force accumulators, checkpointing. |
| `src/geometry.mjs` | `tdf2/src/cfd/voxelize.ts` | Ray-cast idea for closed meshes kept. Link cuts, thin surfaces, slabs, swept-node detection are new. |
| `src/harness.mjs` | `tdf2/e2e/lbm-common.mjs` | Same process-hygiene pattern (tracked browser, signal handlers, watchdog). |

Methods used, for the record:

- Latt & Chopard 2006, regularised BGK. Smagorinsky local closure per Krueger et al., *The Lattice Boltzmann Method*, ch. 9.
- Ladd 1994, moving-wall bounce-back. Bouzidi, Firdaouss & Lallemand 2001, interpolated bounce-back.
- Mei, Yu, Shyy & Luo 2002 momentum exchange. Wen, Zhang, Tu, Wang & Fang 2014, Galilean-invariant form.
- Aidun, Lu & Ding 1998, cover and uncover impulses. Caiazzo 2008, refill.
- Skordos 1993 style shifted populations (store f - w) for single-precision round-off.
