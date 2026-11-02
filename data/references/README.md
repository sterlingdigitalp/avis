# data/parts and data/references

Owned by the `research` workstream. Read 2026-11-02. Rule followed: no value goes in unless it was read at a cited source. Arithmetic done here is labelled `computed`; values taken from a plot are labelled `read_from_figure`.

## Files

| File | Contents |
|---|---|
| `../parts/catalog.json` | Parts in SPEC format |
| `benchmarks.json` | SmartBird, bird morphology, other ornithopters |
| `airfoil-e387.json` | E387 coordinates; UIUC and NASA Cl, Cd tables; tunnel comparison |
| `low-re-airfoils.md`, `airfoils/` | Candidates, `.dat` files, UIUC AG24 and AG35-r polars |
| `flight-standards.md` | Dryden and gust equations, implementation |
| `rockies.md` | Bow Valley environment; elevation data URLs and licences |
| `credits.md` | Sources, obligations, licence decisions |

## Reading `catalog.json`

SPEC defines `mass_g` per part. For stock sold by size, `specs.mass_basis` says what it means: `per_metre` (tube, rod), `per_m2` (film, carbon sheet), `per_litre` (balsa). If absent, the mass is per item.

`price_usd` is set only when the page shows a dollar price in a US context, and is `null` otherwise, including AUD, GBP, EUR and CHF prices.

`verified` has three SPEC values. The leaflet and PDF catalogue pages are `datasheet`. `balsa-wood-database` is marked `retailer-page` for lack of a better value, though The Wood Database is a reference site, not a retailer.

## Gaps that affect gates

**V4 SmartBird**
- Two published power figures: 23 W at the 5 m/s design point and 18 W measured in free flight at 4.7 m/s. "The published figure" needs a declared choice, or both should be tested. See `benchmarks.json`.
- No full-wing stroke angle published. Only a typical 2D plunge amplitude, about 0.3 m, read from Fig. 2A, and a pitch of about 9 deg. The paper says free-flight torsion and bending "remain still unknown".
- Festo's own documents returned HTTP 403, so its 1.96 m and 450 g figures were not read. The Send et al. paper, with a Festo co-author, is the primary source used.
- SmartBird section coordinates not found. NACA 7412 is the paper's design-stage section.

**V7 E387**
- Data is complete for Re 100k and 200k, from both UIUC and NASA.
- The two tunnels disagree on Cd by up to 27% at Re 100k, near 3 deg (UIUC 0.0288, LTPT about 0.0226), and by at most 6.3% at Re 200k (alpha -1 to 9 deg). V7's 25% Cd limit against UIUC is therefore tighter than the tunnel-to-tunnel spread at Re 100k. Computed; see `inter_tunnel_comparison_computed`.
- The NASA tables were transcribed by eye from scans. UIUC tables were parsed from a text layer.

**M8 Electrical, M2 power margin**
- Only 3 of 14 motors, the E-flite ones, publish a continuous current. AXI publishes only a time-limited capacity (20-60 s) and an efficient current band. `continuous_current_a` is `null` for AXI. A rule for treating them needs declaring.
- Servos: one continuous-style rating (KST X06: 0.22 A at 6 V). Three publish stall current. None of the others publish current at the pages read, so M8 cannot be verified for servos such as the X08, HS-5065MG or HS-5070MH without an added assumption.
- ESC peak current has no published duration. Battery C ratings are retailer claims.

**M7 Structure**
- Roll-wrapped tubes, the lightest stiff spar stock here, have no published strength or modulus. The supplier says it has no torsional figures. Woven tubes list 650 MPa and 64 GPa lengthwise. Rods list 400-500 MPa and 28-40 GPa.
- Gear data is KHK's calculated allowable torque, with no fatigue life. No module below 0.5 was captured.
- Servo torque is a stall-style rating, not continuous.
- Balsa is a species average, not a grade.

**M4 Rough air**
- SPEC gives no gust length or shape (hold or return to zero). `sigma_u = 2 m/s` is more severe than the standard's mapping from 6 m/s of mean wind. See `flight-standards.md`.

## Other gaps

- **Military standards:** MIL-F-8785C and MIL-HDBK-1797 not read directly (403). Equations come from MathWorks' documentation of them.
- **Bird flap frequency:** No gull or peregrine value read. Pigeon has 6.7 Hz (via eLife supplement). Pigeon cruise speed and an unambiguous total wing area are missing. Pennycuick's papers returned 403.
- **Ornithopters:** Bat Bot, HIT-Hawk, HIT-Phoenix and the other Festo birds have no values here. Beihang and Songgolmae give span, mass and endurance; WindRider gives span, mass and flap frequency. None of the sources read publishes electrical power, so SmartBird is the only power benchmark.
- **Standalone IMU:** no board with a published mass was found. IMUs are recorded on flight controllers (`imu_onboard`), with their mass included in the board. Barometers likewise, plus one breakout.
- **Excluded for being discontinued:** Matek F405-WMN, H743-WLITE, H7A3-WING, M10-5883, M10Q-5883 and M10-L4-3100 all read "(EOL)". Tattu 3200 mAh 4S at readymaderc was discontinued.
- **Stock:** SpeedyBee F405 Wing Mini and Foxeer M10Q 180 showed sold out at the page read; both are flagged in `specs`. E-flite motors are from manuals, and current listing was not confirmed on a shop page.
- **Rate limits or 403:** KST DS125MG, Holybro Micro M10, MKS's own site, T-Motor, Icarex fabric (no datasheet found, excluded), Silvics of North America. Wikipedia was used for tree heights instead.
- **Banff HRDEM coverage:** the Alberta lidar box ends east of Banff. Check the `coverage` asset. See `rockies.md`.
- **Licences:** UIUC tables and coordinates need a decision before public push. See `credits.md`.

## Reproducing

Download and parse scripts are in `out/research/scripts/`, with downloads in `out/research/dl/` (gitignored). `build_catalog.py` regenerates `catalog.json`, and `build_e387.py` regenerates `airfoil-e387.json`.
