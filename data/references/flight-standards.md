# Turbulence and gust models for gate M4

SPEC M4 requires Dryden turbulence with `u_rms` 2 m/s plus 1-cosine gusts of 4 m/s, vertical and lateral.

## Sources actually read

- MathWorks, "Dryden Wind Turbulence Model (Continuous)", Aerospace Blockset documentation. Reproduces the MIL-F-8785C and MIL-HDBK-1797/1797B spectra, transfer functions, scale lengths and intensities. https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html (read 2026-11-02)
- MathWorks, "Discrete Wind Gust Model", citing U.S. Military Specification MIL-F-8785C, 5 Nov 1980. https://www.mathworks.com/help/aeroblks/discretewindgustmodel.html (read 2026-11-02)

**Not read:** the MIL-F-8785C and MIL-HDBK-1797 PDFs themselves, and Moorhouse & Woodcock AFWAL-TR-81-3109 (DTIC ADA119421). Direct downloads returned HTTP 403. The equations below are the MathWorks statement of those standards.

Symbols: `V` airspeed (m/s), `b` wingspan, `h` height above ground, `L` turbulence scale length, `sigma` RMS intensity, `omega` temporal frequency (rad/s), `s` Laplace variable. Scale-length formulas use **feet**: convert `h` to feet, evaluate, and convert `L` back.

## Velocity spectra and shaping filters (Dryden, MIL-F-8785C form)

Power spectra in temporal frequency:

```
Phi_u(w) = (2 sigma_u^2 L_u / (pi V)) * 1 / (1 + (L_u w / V)^2)
Phi_v(w) = (sigma_v^2 L_v / (pi V)) * (1 + 3 (L_v w / V)^2) / (1 + (L_v w / V)^2)^2
Phi_w(w) = (sigma_w^2 L_w / (pi V)) * (1 + 3 (L_w w / V)^2) / (1 + (L_w w / V)^2)^2
```

Transfer functions:

```
H_u(s) = sigma_u * sqrt(2 L_u / (pi V)) * 1 / (1 + (L_u/V) s)
H_v(s) = sigma_v * sqrt(L_v / (pi V)) * (1 + sqrt(3) (L_v/V) s) / (1 + (L_v/V) s)^2
H_w(s) = sigma_w * sqrt(L_w / (pi V)) * (1 + sqrt(3) (L_w/V) s) / (1 + (L_w/V) s)^2
H_p(s) = (sigma_w / (0.8 V)) * (pi / (4 b))^(1/6) / (L_w^(1/3) * (1 + (4 b / (pi V)) s))
H_q(s) = +/- (s / V) / (1 + (4 b / (pi V)) s) * H_w(s)
H_r(s) = -/+ (s / V) / (1 + (3 b / (pi V)) s) * H_v(s)
```

The upper signs of `H_q` and `H_r` are the block's convention for its stated axes. Check them against the body frame in SPEC (x forward, y up, z right), which differs from the aerospace z-down frame.

## MIL-HDBK-1797 / 1797B form

`H_u` is the same. For the lateral and vertical channels:

```
H_v(s) = sigma_v * sqrt(2 L_v / (pi V)) * (1 + 2 sqrt(3) (L_v/V) s) / (1 + 2 (L_v/V) s)^2
H_w(s) = sigma_w * sqrt(2 L_w / (pi V)) * (1 + 2 sqrt(3) (L_w/V) s) / (1 + 2 (L_w/V) s)^2
H_p(s) = (sigma_w / (0.8 V)) * (pi / (4 b))^(1/6) / ((2 L_w)^(1/3) * (1 + (4 b / (pi V)) s))
```

The handbook uses lateral and vertical scale lengths of half the 8785C value at low altitude (`2 L_v = L_u`, `2 L_w = h`). MathWorks states the military specifications give the same transfer function after the scale lengths are evaluated. Use one form, not a mix.

## Scale lengths and intensities

Low altitude, `h` below 1000 ft (305 m) above ground, axes aligned with the mean wind (`u` horizontal along the wind, `w` vertical):

```
L_w = h                                  (h in ft, L in ft)
L_u = L_v = h / (0.177 + 0.000823 h)^1.2
sigma_w = 0.1 * W20                      (W20 = wind speed at 20 ft)
sigma_u / sigma_w = sigma_v / sigma_w = 1 / (0.177 + 0.000823 h)^0.4
```

Medium and high altitude, above 2000 ft above ground, axes aligned with the body:

```
L_u = L_v = L_w = 1750 ft
sigma_u = sigma_v = sigma_w, from an altitude and exceedance table
```

Between 1000 and 2000 ft: interpolate linearly between the two. Keep `h` above a floor near the ground, because `L_w = h` goes to zero. MathWorks gives a table for the high-altitude case; it is not reproduced here.

## Worked values for M4

Computed here from the low-altitude formulas with the gate's `sigma_u = 2 m/s`. These are arithmetic, not published values.

| h above ground (m) | L_w (m) | L_u = L_v (m) | sigma_u / sigma_w | sigma_w (m/s) | Implied W20 (m/s) |
|---:|---:|---:|---:|---:|---:|
| 5 | 5.0 | 36.6 | 1.941 | 1.03 | 10.3 |
| 10 | 10.0 | 67.4 | 1.889 | 1.06 | 10.6 |
| 30 | 30.0 | 152.5 | 1.719 | 1.16 | 11.6 |
| 100 | 100.0 | 262.8 | 1.380 | 1.45 | 14.5 |
| 300 | 300.0 | 304.7 | 1.005 | 1.99 | 19.9 |

**Flag for physics.** `sigma_u = 2 m/s` implies `W20` of about 10-20 m/s through these formulas, more than the 6 m/s mean wind M4 also states. The gate fixes `u_rms` directly, so it is more severe than the standard's mapping from a 6 m/s mean wind would give. That is allowed and is the gate as written. Do not derive `sigma` from the 6 m/s mean wind.

## Time-domain implementation

This section is a derivation made here, not text from a standard. It gives output variance `sigma^2`.

Drive each filter with zero-mean Gaussian white noise of two-sided spectral density `pi`:

```
eta_k = sqrt(pi / dt) * N(0, 1)
```

That density follows from requiring output variance `sigma^2` from the transfer functions above: with `tau = L/V`, `(1/2pi) * integral |H|^2 S dw = sigma^2 S / pi` for both `H_u` and `H_w`, so `S = pi`.

First order, `H_u`. With `tau = L_u / V`, exact discretisation:

```
a = exp(-dt / tau)
u_{k+1} = a u_k + sigma_u sqrt(1 - a^2) N(0, 1)
```

Second order, `H_w` and `H_v`, as a state-space form with `tau = L / V`:

```
x1' = x2
x2' = -x1 / tau^2 - 2 x2 / tau + n(t)
y   = (sigma * sqrt(tau / pi) / tau^2) * (x1 + sqrt(3) tau x2)
```

Check before use: a long Monte Carlo run must return RMS `sigma` and the Phi spectrum to within sampling error. Check this against the spectrum above, not against this code. `dt` must be small against `tau`; with `V` near 10 m/s and `L` near 10 m, `tau` is about 1 s, so `dt = 1/120` s is ample.

Use `V` as true airspeed. For a slow bird in a wind of comparable speed, the frozen-field assumption behind Dryden (turbulence convected past the vehicle at `V`) is weaker than for an aircraft. State this limitation in any M4 report.

## Discrete 1-cosine gust

```
V_wind(x) = 0                                 for x < 0
V_wind(x) = (V_m / 2) * (1 - cos(pi x / d_m)) for 0 <= x <= d_m
V_wind(x) = V_m                               for x > d_m
```

`V_m` gust amplitude, `d_m` gust length, `x` distance flown since the gust started.

This form, as MathWorks states it, **holds at `V_m` after `x = d_m`**. The form commonly called a "1-cosine gust" in loads work returns to zero, with `V(x) = (V_m/2)(1 - cos(2 pi x / H))` over `0 <= x <= H`.

**Flag for orchestrator.** SPEC M4 gives amplitude 4 m/s vertical and lateral, but no gust length and no choice between the step-to-hold and return-to-zero shapes. The MathWorks defaults (`d_m` of 120 m, 120 m and 80 m on u, v and w) come from the block, not from a bird-scale requirement. A gust length tuned to the short-period mode is much harder than a long one. This must be fixed before M4 runs, so that it is declared before results.

## Density at the design condition

Computed here, not sourced: international standard atmosphere pressure at 1800 m (81,489 Pa) with SPEC's 10 C gives `rho = 1.003 kg/m3`, which is 0.818 of the sea-level standard 1.225 kg/m3. SPEC says about 18% loss, which agrees.
