# data/parts and data/references

Owned by the `research` workstream. Read 2026-11-02. Rule followed: no value goes in unless it was read at a cited source. Arithmetic done here is labelled `computed`; values taken from a plot are labelled `read_from_figure`.

## Files

| File | Contents |
|---|---|
| `../parts/catalog.json` | Parts in SPEC format: 102 from round 1, 77 added in round 2 (2026-11-02) |
| `gear-materials.json` | Gear material grades: KHK allowable bending and contact stress, tensile, modulus, density, with sources and quotes |
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

Round 2 conventions:
- **New category `metal_tube`.** Fields are `alloy`, `temper`, `yield_strength_mpa`, `tensile_strength_mpa`, `elastic_modulus_gpa`, `density_g_cm3` and `alloy_property_source_url`. `tubeAllowable()` in `src/design/catalog.js` does not read it yet.
- **Tube prices.** For tube stock, `price_usd` is the price of the listed piece. `specs.price_note` gives the piece length, while `mass_g` is per metre. Tiger Titanium sells by the inch, so its `price_usd` is null and the per-inch price is in `price_note`.
- **Servo current.** For 10 of the 13 round-2 servos, the page gives a single stall current with no voltage. Those entries have `voltage_v: null` and a `stall_current_note`. Where a maker gives currents per voltage elsewhere (Savox SB-3262SG, Hitec MD89MW), the single figure matches the top of the voltage range, so it is conservative at 5-6 V. `idle_current_a` is set only where published.
- **Gears.** Each round-2 gear has `material_strength`, giving the grade's allowable bending stress and its source, plus `rating_basis` with KHK's calculation conditions.
- **Evidence.** Verbatim quotes for every round-2 number are in `out/research/round2/evidence.json`, keyed by part id. That file is local and gitignored.

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
- **Motors.** 13 of 24 motors publish a continuous current with no time limit: the 3 E-flite motors plus the 10 from round 2 (Scorpion SII, Cobra). The round-2 motors also publish continuous power, Kv, Rm and Io, but none publishes burst current or an efficiency band. AXI publishes only a time-limited capacity (20-60 s) and an efficient current band, so it falls under the SPEC fallback rule.
- **Motor resistance.** Cobra gives Rm both per phase and phase to phase; `resistance_ohm` is phase to phase. Scorpion's page does not say which it gives.
- **Motors left out.** T-Motor AT and Hacker publish only 180 s and 30 s ratings. E-flite Power 10/15 and Park 400/450 manuals were behind a Cloudflare check.
- **Servos.** 16 of 21 publish a stall current: 3 from round 1 (KST X06, Savox SH-0257MG, SH-0255MG) and all 13 from round 2. Round 1's KST X08, X08 Plus, Hitec HS-5065MG, HS-5070MH and MKS DS65K still publish none and remain ineligible under M8.
- **Servo current has two caveats:**
  - 10 of the round-2 stall currents have no stated voltage. See "Reading `catalog.json`".
  - Savox's "Rated Current" (120-250 mA) has no rated torque paired with it. It is stored as `rated_current_as_labelled`, not as a continuous rating.
- **Servos with no low-voltage row.** Six round-2 servos publish no row below 6.0 V: HS-7245MH, SG-1212MG, SG-1211MG, SV-1250MG+, SV-1261MG and SV-1254MG+. The Savox SB-3262SG's lowest row is also 6 V. `servoAt(part, 5)` clamps to the lowest row, so on a 5 V BEC it reports 6 V performance. A 6 V or higher BEC avoids this, or the design code should refuse to extrapolate.
- **ESC and battery ratings.** ESC peak current has no published duration. Battery C ratings are maker or retailer claims.
- **Li-ion C rating.** For the Nexus P30B Li-ion packs, the C rating is computed from the published 30 A continuous. `screen.js` currently admits only LiPo, so Li-ion packs are not screened.

**M7 Structure**
- **Carbon tubes with published strength.** Twelve were added in round 2, all with values for the product family rather than per size:
  - Easy Composites woven 7x5, 10x8, 10x9 and 12x11: 650 MPa and 64 GPa, "representative of a typical tube".
  - Rock West pultruded 5x3, 6x5, 8x6 and 10x8: 2372 MPa tensile, 1682 MPa compressive, 133.8 GPa. These come from classical lamination theory, "reference only", and the page says "We cannot supply test or strength data".
  - CST DPP pultruded 5x4, 6x4, 6x5 and 8x7: 2.50 GPa tensile, 1.60 GPa compressive, 140 GPa. These are from a 2011 comparison table, "a design guide only, not a certification of properties".
- **Ruling needed on the calculated figures.** The Rock West and CST figures are published, but they are calculated or for guidance only. The orchestrator should rule whether they count as "published strength" under M7. `tubeAllowable()` will use them as they stand.
- **No flexural values.** None of the makers read publishes flexural strength or flexural modulus for a tube. DragonPlate publishes only fibre modulus.
- **Metal tube.** No 7075-T6 round tube of 4-12 mm OD was found listed. Tiger Titanium Ti-3Al-2.5V CWSR tube is in, with alloy minimums from the Fine Tubes datasheet: 725 MPa yield, 860 MPa tensile, 107 GPa, 4.48 g/cm³. Tiger certifies to ASTM B338, not AMS 4944/4945. Its 9.52 x 0.635 mm item is "uncerted surplus stock". Mass per metre is computed from the cited density.
- **Gears.** All 20 round-2 gears are KHK steel or brass with published allowable torque for bending and for surface durability. The ratings assume a mate with the same tooth count, 10^7 cycles, uniform load, and a 2/3 bending limit for load in both directions.
  - Untreated S45C (SS, SSAY) is always limited by surface durability, which is very low.
  - Steel strong enough is heavy. The strongest 4-5:1 stage under 150 g (computed, the lower of the wheel rating and the pinion rating times the ratio) is SSG1-15 with the nylon PS1-60, at 2.41 N·m and 58 g.
  - With an SSY1-60H wheel it reaches 4.12 N·m at 206 g. MSGA1-18 with MSGA1-80 reaches 28 N·m at 490 g.
- **Gear yield strength not found.** No published yield was found for KHK's S45C, SCM415, SUS303 or C3604 in KHK's conditions. `gear-materials.json` gives KHK's allowable bending stress instead, plus AISI 1045 cold-rolled and C36000 yields as reference rows only.
- **Gear sources not searched.** SDP/SI, Misumi GEAKB and Reely brass gears were not researched. No gear below module 0.5 is captured.
- **Bearings.** Fourteen SKF bearings with bore 3-8 mm, with dynamic and static ratings and mass. No flanged versions.
- **Servo torque.** This is a stall-style rating, not continuous.
- **Balsa.** This is a species average, not a grade.

**M4 Rough air**
- SPEC gives no gust length or shape (hold or return to zero). `sigma_u = 2 m/s` is more severe than the standard's mapping from 6 m/s of mean wind. See `flight-standards.md`.

## Other gaps

- **Military standards:** MIL-F-8785C and MIL-HDBK-1797 not read directly (403). Equations come from MathWorks' documentation of them.
- **Bird flap frequency:** No gull or peregrine value read. Pigeon has 6.7 Hz (via eLife supplement). Pigeon cruise speed and an unambiguous total wing area are missing. Pennycuick's papers returned 403.
- **Ornithopters:** Bat Bot, HIT-Hawk, HIT-Phoenix and the other Festo birds have no values here. Beihang and Songgolmae give span, mass and endurance; WindRider gives span, mass and flap frequency. None of the sources read publishes electrical power, so SmartBird is the only power benchmark.
- **Standalone IMU:** no board with a published mass was found. IMUs are recorded on flight controllers (`imu_onboard`), with their mass included in the board. Barometers likewise, plus one breakout.
- **Excluded for being discontinued:** Matek F405-WMN, H743-WLITE, H7A3-WING, M10-5883, M10Q-5883 and M10-L4-3100 all read "(EOL)". Tattu 3200 mAh 4S at readymaderc was discontinued.
- **Stock:**
  - SpeedyBee F405 Wing Mini and Foxeer M10Q 180 showed sold out at the page read, and Scorpion SII-2205-1490 showed out of stock. All three are flagged in `specs`.
  - Scorpion SII-3008-1090 is on backorder, and the Nexus 2S1P P30B pack is on pre-order, shipping in 1-3 weeks. Both are noted but not flagged.
  - E-flite motors are from manuals, and current listing was not confirmed on a shop page.
- **Rate limits or 403:**
  - Round 1: KST DS125MG, Holybro Micro M10, MKS's own site, T-Motor, Icarex fabric (no datasheet found, excluded), Silvics of North America. Wikipedia was used for tree heights instead.
  - Round 2:
    - kstservos.com returned 429. KST A12/A15 pages list no current, so no KST servo was added.
    - mksservosusa.com returned 403, so MKS HV6160 is from the Lindinger retailer page.
    - savoxusa.com failed TLS, so Savox servos are from savox-servo.com.
    - The Horizon Hobby manual PDFs, TW Metals, Online Metals' titanium pages and Polyplastics' Duracon datasheet (login) could not be read.
  - Futaba, BlueBird and Torxis servos were not searched.
- **Banff HRDEM coverage:** the Alberta lidar box ends east of Banff. Check the `coverage` asset. See `rockies.md`.
- **Licences:** UIUC tables and coordinates need a decision before public push. See `credits.md`.

## Reproducing

Download and parse scripts are in `out/research/scripts/`, with downloads in `out/research/dl/` (gitignored). `build_catalog.py` regenerates `catalog.json`, and `build_e387.py` regenerates `airfoil-e387.json`.

Round 2 parts are rebuilt the same way:
- Lane fragments are in `out/research/round2/{servos,gears,motors,batteries,structure}.json`. Their build scripts are in `out/research/round2/scripts/` and their downloads in `out/research/dl/r2/`.
- `python3 -I out/research/scripts/add_round2.py` runs after `build_catalog.py`. It validates the fragments, applies and prints its fix-ups, appends the parts, and writes `evidence.json`.
- `--check` validates without writing.
