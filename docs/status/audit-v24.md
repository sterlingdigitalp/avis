# Audit v24 (Larus GL-2, commit a558baf): is every part real, adequate and present?

Independent, read-only audit, 2026-11-03. Nothing in ~/avis was edited and no git writes were made (`git show 323b0ab:...` was read-only).

**Who did what:**
- A (mass) by the lead auditor.
- B (interfaces) and C/D (build needs, anything else) by two forked auditors, whose work was cross-checked where it overlapped.
- Sections A, B and C/D follow this table in full.

**Scripts:**
- scratchpad `asmx.mjs` and `cd/*.mjs`: each runs `fullAssembly` from `src/design/clearance.js` on the shipped design.
- scratchpad `v23.json`: the v23 design.

Severity scale:
- (1) breaks "real parts, assembled"
- (2) could change a gate verdict
- (3) a disclosure or record gap
- (4) cosmetic

Unsourced means the figure was not read from a maker or stockist page in this audit.

## Ranked findings

| # | What | Evidence | Sev | Smallest fix |
|---|---|---|---|---|
| 1 | **The wing twist drive cannot twist anything.**<br>- The torque rod lies wholly inside the outer spar's bore, from 0.5 mm past the elbow to s 0.9383. It is joined to nothing at either end.<br>- The tip rib (s 0.974) is not on the rod.<br>- The hand-wing ribs are "on the spar", and the outer spar is bonded rigid to the inner spar.<br>- The servo case touches nothing: no frame, screws or ties, and it is not in the support ledger.<br>- The flown "horn linkage 1.6:1" has no horn, link or clevis in the BOM.<br>- M7 rates servo torque and rod torsion on a load path that does not exist. | wing-solids.js:89, 92-95 (rod from `elbow + run` to `sEnd`), 102-104, 123, 127 (`s <= sEnd`)<br>`design.servos.twist.linkage_note`<br>round 16 named only the elbow end | 1 | Bring the rod out of the spar at both ends. Needed parts:<br>- a servo frame and screws, a servo arm, a link with two clevises, and a lever on the rod;<br>- a rod bearing at the elbow, the rod fixed to the tip rib, and the ribs on bushings on the spar.<br>None of these is in data/parts, so all are unsourced. |
| 2 | **The V-tail is flown all-moving (±30°, pivot at 25% c), but it is drawn cable-tied fixed.**<br>- There is no pivot, bearing, horn or pushrod.<br>- The servos are at x -51..-22 mm and the tail at x -946 mm, so about 0.9 m of linkage per side is missing. | flight.js:454-457<br>`tail.deflection_max_rad` 0.5236<br>wing-solids.js:187-210 (fixed `S` solids, tied)<br>sequence steps 50-51 | 1 | Per surface:<br>- a pivot rod in a bushing (unsourced);<br>- a horn and clevises (unsourced);<br>- a pushrod of `rod-ec-2mm` (Easy Composites 2 mm pultruded rod, 4.5 g/m, about 4.3 g at 0.95 m) or `rod-ec-1-5mm` (2.7 g/m, about 2.6 g).<br>Draw them, sequence them and check them in M7. |
| 3 | **The elbow joint is two tube ends "butted, bonded".**<br>- The 8.0/7.0 mm outer spar meets the 9.982/7.976 mm inner spar end to end. The faces overlap only at r 3.988-4.000 mm, about 0.30 mm².<br>- The joint must carry 5.913 N m (M7 `spar_outer.moment_Nm`).<br>- Even crediting the whole annulus at Araldite 2011's 9.55 MPa gives FoS about 0.03.<br>- The outer spar's 8.000 mm OD cannot enter the 7.976 mm bore, and its own bore is taken by the torque rod.<br>- No member checks the joint. | wing-solids.js:89<br>catalogue tube dimensions<br>out/verification/M7-larus-gl-2.json | 1 | A stepped external joiner, machined `metal-al-7075-t6` (2.81 g/cm³), bores 10.0 and 8.0 mm x 20 mm, OD 12 mm. About 5.5 g per wing (computed; not a catalogue item), with bond and bearing checks.<br>If the hand wing must turn for twist, use a bearing joint designed together with row 1. |
| 4 | **The fuselage covering cannot exist on what is drawn, and the formers' -10.5 g shrank the BOM to an incomplete structure.**<br>- The structure is five flat balsa rings, 4 mm radial x 2 mm, at x -100, -30, 97, 165, 200 mm, with no stringers.<br>- The loft runs from x +257.8 to -315.1 mm. That leaves the nose (58 mm) and the tail cone (215 mm) with no structure. The cone ends at (x -315, y -27), where nothing is.<br>- A taut zero-pressure film cannot span rings 127 mm apart at R about 65 mm: a minimal surface between two coaxial rings exists only for 2h/R < 1.325, and here 2h/R is about 1.9.<br>- So the film necks onto the internals. The containment pass (battery 0.733 mm) rests on a skin nothing supports.<br>- Each ring hangs on a 2 x 2 mm balsa tab 23-41 mm tall.<br>- There is no former strength check. The stated reason, "they carry only the film", contradicts the tie check, which loads them with the pod's drag at 25 m/s and its inertia.<br>- The loft also carries a phantom 10 mm tube at y -27 out to x -1088.8 (0.0243 m² of wetted area) that encloses nothing; the real boom is at y +62. | params.linkage.airframe.pod<br>airframe.js:577-600, 636<br>body.loft.sections<br>`structure.members` (only "pod formers ties")<br>build.js:409 `podAndBoomLoft` | 1 | - Longerons: about 8 x 0.57 m of `rod-ec-1-5mm`, 12.3 g.<br>- Formers in `plate-ec-1mm` (Easy Composites 1 mm carbon plate, 1.24 kg/m²): 8.8 g for the drawn 7100 mm².<br>- A nose block and tail-cone formers (unsourced).<br>About +20 g, +23 g with margin. Put the covering in the sequence; drop or explain the phantom tube. |
| 5 | **There are no connectors in the BOM or in the catalogue (247 entries).**<br>- The pack ships with a Deans plug, and its mate is missing.<br>- The wings go on last (steps 82-83), so each twist servo needs a wing-root connector plus about 0.5 m of extension across two moving joints.<br>- The ASPD-4525's 20 cm JST-GH cable cannot cover the 213 mm straight line to the FC.<br>- Soldered motor/ESC/PDB/regulator joints count as wire and are exempt. | grep of bom/catalogue for connector, plug, bullet, XT, Deans, JST: nothing<br>Gens Ace page (SKU GEA223S35DGT): "Connector Type: Deans"<br>Matek package list | 1 | BOM lines for:<br>- a Deans Ultra Plug male (WSD Sales; part number and mass unsourced, about 1-2 g);<br>- two wing-root servo connector pairs;<br>- a longer JST-GH 4P cable;<br>- servo extensions.<br>All unsourced. |
| 6 | **The pack cannot be taken out to charge, and the bird cannot be armed, bound or set up, without cutting the covering.**<br>- The pack spans x 156.5-231.2 mm across two full-hoop formers (164-166, 199-201). The formers go on after the pack (steps 72-76 against 58).<br>- Its T40R ties are one-shot.<br>- No hatch or opening is declared.<br>- There is no switch or arming plug, and the only arming connection (the Deans) is sealed in.<br>- The RX bind button, the FC's USB-C and servo centring are sealed in too. | audit-CD.md C1<br>containment ledger openings (wing roots, horn slots, boom posts, pitot only) | 1 | - A top hatch over x about 150-235 that carries formers 4 and 5's top arcs, declared as an opening, with a stated fixing: 2 x ISO 7380 M3 into the riser flanges, or a magnet pair (mass unsourced).<br>- A releasable strap.<br>- An arming plug reachable from the hatch.<br>- Route the FC's included USB-C extender to the hatch. |
| 7 | **Nothing holds the coupler eyes on their pins.**<br>- The horn pins end flush with their eyes' free faces.<br>- The crank pin ends 1 mm past the left eye, with no groove. | `params.layout.horn_pin_past_mm` 0<br>drawn ends: right -0.80 / -0.80, left -1.30 / -1.30, crank pin -8.30 / -7.30 mm (audit-B.md)<br>confirms round 16 | 1 | Lengthen each pin by about 1.5 mm and fit DIN 6799 size 5 retaining washers, 3 off at 0.236 g each (fasteners.eu DIN 6799 table, read in B). Re-run clearance. |
| 8 | **The gear train is not located axially, apart from the crank shaft.**<br>- Gear shafts 1-3 are plain rods, plate to plate, with no shoulder, clip or collar.<br>- The keyed gears can slide on their keys: 4.5 mm to the aft bushing on shaft 1, 17 mm to the forward one.<br>- The 628/8 and 619/5 bearings are unretained, and shaft 3 has 3.3 mm of travel.<br>- M7 gives every bearing zero axial load. The last stage runs at 0.6% torque margin, so any loss of face overlap fails it. | clearance.js:369-378<br>catalogue `fastening`: keyways only<br>M7 gear margins | 1 | About 18 DIN 6799 size 5 washers, about 4.2 g (fasteners.eu), or 6061 spacer tubes. A DIN 471 ring or a seat shoulder for each bearing (unsourced). Re-check the shafts at the grooves. |
| 9 | **The CG the flight model flies is not the drawn CG.**<br>- The built bird is about 5.2 mm further forward (outside the ±5 mm CG corner) and 8.1 mm higher.<br>- The 507.6 g "crank, rockers, plates" lump (42 lines) is a point mass at (8.7, -56.0) mm, the crank axis. The drawn centroid is (30.2, -18.5).<br>- The battery's y is hard-coded at -5 mm against a drawn -30.1, and its drawn x is 8 mm aft of the trim.<br>- The couplers are at (0, 0, 0) against a drawn y of -28. The twist servos are put on the spar but drawn 22 mm aft of it.<br>- There is no y corner. Inertias carry the error (Ixx about 6%), and the packing residual (-0.65 mm) is never applied.<br>- The pack has no travel left to re-trim (CD C4). | audit-CD.md D3 (`cd/lumpcheck.mjs`)<br>build.js:340<br>flight.js:131-136<br>scenarios.js:491<br>`mass.lumps` | 2 | Take each lump's position and gyration from its drawn solids, re-solve the trim, re-lay the nose tray if the pack cannot reach, and re-fly. |
| 10 | **The spar and the twist servo stand out of the real E387's lower surface, and every rib's spar hole breaks through.**<br>- The spar is centred on the chord line, where the E387 lower surface at x/c 0.34-0.39 is -0.008 to -0.006 c, about -3.0 mm at the root.<br>- So the 10 mm inner spar protrudes about 2 mm, the outer about 1-1.6 mm, and the servo case 2.6-3.7 mm.<br>- The clearance model's rib envelope uses a NACA-form thickness on a parabolic camber line, with its lower surface at about -0.015 c, so it cannot see this.<br>- None of the protrusion's drag is counted. | data/references/airfoils/e387.dat (0.33968 -0.00804, 0.39252 -0.00605; confirmed)<br>wing-solids.js:48-64, 81 | 2 | Raise the spar axis about 12-14 mm toward the camber line. That moves the socket and the root opening, so re-run clearance and containment and re-fly. |
| 11 | **The tail-to-boom joint has no moment path.**<br>- Each 1.5 mm spar rod only butts against the boom.<br>- Two T18R ties forward of the spar are checked for force only (7.0 N against 2 x 80 N).<br>- Root moment is about 1.1 N m per surface against about 0.26 N m of rod capacity, so FoS about 0.2 (B's estimate).<br>- The boom (-1.8 g) was cut to 5 mm aft of the tail spar "because running it to the trailing edge moved the CG 45 mm aft", which leaves the aft 75% of each 191 mm root rib bare. | wing-solids.js:197-210<br>airframe.js:646-650<br>final-gates-larus.txt:66-68<br>`tail.chord_root_m` 0.1906 | 2 | A bonded root socket or a machined `metal-al-6061-t6` saddle on the boom (unsourced until drawn). A tail spar sized by the M7 rule: `tube-cst-pultruded-5x4` (CST, 11.1 g/m) is a candidate, adequacy unverified. Add the root moment to M7. |
| 12 | **The tip is held only by the edge rods.**<br>- The semispan from the shoulder axis is 0.975 m, not 1.0 m.<br>- The outer spar and rod end at s 0.9383, 35.7 mm short of rib 10 (s 0.974). At the tip the spar axis lies ahead of the leading edge (x/c -0.004).<br>- Rib 10 is joined "on the spar" to a spar that does not reach it. It sits on the 2 mm and 1.5 mm rods, cantilevered 102 mm from rib 9.<br>- My estimate: about 0.3-0.4 N m against 223 N mm of rod capacity at the catalogue's lower compressive 200 MPa, so FoS about 0.6.<br>- It is outside M7 (no rod, rib or tip member). | wing-solids.js:85, 120-127<br>params.wingStructure.spar_to_m<br>`wingRibStations` output 52.0 ... 871.6, 974.0 mm | 2 | Run the outer spar along about 12% chord to rib 10: +37 mm per side of `tube-cst-pultruded-8x7` (CST CS-529, 17.5 g/m), +1.3 g. Or put a rib at the spar's end and give the rod cantilever an M7 member.<br>The first changes the twist axis, a flight-model input. |
| 13 | **The split crank's web hub is a 2 mm titanium wall, 0.8 mm behind the 3 mm keyway, and nothing checks it.**<br>- The 14 mm capsule is bored 10 mm. The key's hub depth is 1.2 mm, and the key force is about 3.2 kN (16.14 N m / 5 mm).<br>- M7 checks only the arm and key crushing. | linkage.js:137-142<br>params.linkage.crank.web | 2 | A 20 mm hub boss on the web, about +2 g of `metal-ti-6al-4v-annealed-bar` (4.43 g/cm³; computed), plus a hub shear-out / hoop check. |
| 14 | **M4's FINAL standing rests on a BOM that lacks the parts above.**<br>- The sourced or computed fixes in row 2 (pushrods, 8.6 g), row 3 (11 g), row 4 (20 g), row 7 (0.7 g), row 8 (4.2 g), row 12 (1.3 g) and row 13 (2 g) add about 48 g, about 55 g with margin. Row 1, row 5, row 6 and row 11 add more, unsourced. That is about the whole 56 g that, by the record, turned M4 final.<br>- Row 9 also moves the CG.<br>- At uniform 0.6, v24's worst corners are exactly 98/100, the pass line. | final-gates-larus.txt:242-251, 194-203<br>SPEC "v24 result" | 2 | Fix rows 1-13, then re-fly the full set, including the 0.6 comparison, before calling any verdict final. |
| 15 | **The tail servo runs at 7.5 V, above the MD89MW's top published row (7.4 V).** Current is scaled to 7.8 V. That is conservative for M8, but it breaks the SPEC rule "no extrapolation outside a servo's published voltage rows", and it is not disclosed. | catalog.js:79-86<br>audit-CD.md D6 | 2 | Disclose it, or move both servos to the Pololu D36V50F6 (6 V output; mass unsourced) and re-fly with the HV6160's 6 V row. |
| 16 | **The ball bearings sit in seats narrower than they are, and their ratings are credited in full.** The 628/8 is seated over 1.2 of 5 mm and 2.0 of 5 mm; the 619/5 over 1.2 of 4 mm. Bushings are derated to their supported length, but ball bearings are not. The minimum bearing FoS is 3.41, so no verdict flips. | bearings.js:71-79 against 90-94 | 3 | Bosses as long as the bearing, as three seats already have. |
| 17 | **The outer spar is drawn and rated at the listing's 8 x 7 mm.** The catalogue's own mass_note quotes CST's spec sheet: 7.8 x 6.8 mm, 18.1 g/m. At that size Z falls 5.4% (FoS 2.81 to about 2.66), and the torque rod's clearance is 0.225 mm. | catalog.json `tube-cst-pultruded-8x7` mass_note | 3 | Rate on the smaller section and count mass on the larger figure (+0.5 g). |
| 18 | **The pitot's silicone tubing is put in the 15% margin.** It is neither glue nor wire, which are the only exemptions. Matek ships 40 cm with the board. The probe's seal and end fittings are not drawn. | build.js:187-188, 200<br>BOM pitot basis | 3 | A drawn tubing BOM line (mass unsourced). |
| 19 | **The tail ties are slid along the boom as pre-closed loops (steps 48-49) before the root ribs they bind arrive (50-51).** That sidesteps the rule that a tie is threaded after every part it binds. | round16.mjs:130<br>sequence.js:12-13<br>final-gates-larus.txt:47-48 | 3 | Mark them `threaded` and place them after steps 50-51 (an order that passes exists). |
| 20 | **The ESC and motor sit in a closed pod with no air path and no heat check.**<br>- M8 checks current only.<br>- The gears' 40 °C rating condition is stated, not verified.<br>- The igus temperature limit is not checked. | audit-CD.md C3<br>gears.js:259 | 3 | A declared inlet and outlet with their drag plus a heat balance, or mark the thermal condition unverified. |
| 21 | **The receiver antenna has no solid, route or mass.** TBS's package lists one. The ledger says "no catalogue dimensions", but the catalogue lists 18 x 11 mm. | audit-CD.md C5<br>ledger-v24.txt:10 | 3 | Draw and mass the antenna (mass unsourced). Compare the published size. |
| 22 | **Some interfaces are implied but unrated.**<br>- The spar-root bond is checked on axial force only (6.18 N); the spar's torsion passes through it unchecked.<br>- Ribs and their bonds have no check.<br>- The horn pins are "pressed" with no fit class.<br>- The shoulder pin is "clamped" by a hub with no clamp drawn.<br>- The igus G bushings run on bare 7075 / Ti, outside igus's tested shafts, with no pv or wear check.<br>- The motor's M3 x 8 goes about 6 mm into back-face threads of unknown depth.<br>- The tail-servo ties are checked for inertia only, without the servo's reaction.<br>- The crank web's LOCTITE 638 on titanium is unrated (already in the record). | audit-B.md items 8-15<br>linkage.js:221, 245-248<br>joints.js:96 | 3 | State and rate each one, or replace it with a rated part. |
| 23 | **The riser's 23 mm actuator hole comes within 1.09 mm of the tab's bend line.** The riser is checked only at its root; at the hole, Zo falls about 10%, so FoS stays near 4.3. | airframe.js:321-323, 334<br>riser y -79.6..-43.4, hole top -44.49 | 3 | Lower the hole or make it a slot, and check the section at the hole. |
| 24 | **The pack's mass is the maker's "Net Weight(±20g): 143".** It is inside the ±5% mass corner, but it is not disclosed beside M6. | genstattu.com SKU GEA223S35DGT, read 2026-11-03 | 3 | Disclose it. |
| 25 | **Record text is stale or wrong.**<br>- The housings' mounting is said to be "in the 15% margin".<br>- The former ties are "to the boom".<br>- clearance.js:691 says "screws ... not drawn".<br>- The pin is given as "43 mm" (it is 43.4).<br>- "1.0 m semispan" should be 0.975 m from the shoulder axis.<br>- The rib envelopes are called "full outlines" but are bounding rectangles.<br>- The "top panel" is the floor (y -81).<br>- The left root reads "polar -180..180".<br>- The shell-and-formers lump is at x -50, but the formers are drawn at x +52.<br>- Volume lines double count the bracket/ring overlap (about +3.7 g) and run up to 10% apart elsewhere (rockers +1.6 g each). Net drawn minus BOM is +2.75 g. | see sections A9, B16, CD D8 | 4 | Correct the text. Bring the volume lines to the drawn volume. |

## Checked and found sound (not silence)

**Mass (A):**
- **Lines that went down, judged sound:** shoulder brackets, aft seat bosses, M3 screw and nut tally, panels, inner spar, edge rods, forward tray doubler, plate bosses, riser strength.
  - Shoulder brackets: v23 double counted the ring and sleeve; M7 still rates the arm at the full 21.132 mm lever.
  - Aft seat bosses: replaced by the 2.0 mm plate and the lock flange, all rated.
  - Screws and nuts: 20 screws and 16 nuts reconcile joint by joint.
  - Panels: 59.8 mm between the inner faces, rated at a 61.4 mm span.
- **Lines that went up:** plates, clamp blocks and the crank's volumes.
- **Mass model = BOM:** 1514.27 g of lumps + 2 x 117.687 g of wing = 1749.64 g. Every lump is its lines x 1.15, and nothing is counted twice; the 53.5 g double count is fixed at build.js:183.
- **15% margin:** applied as SPEC says, on parts plus structure, to every lump and wing point mass.

**Interfaces (B):**
- **Shoulder pivots:** bushing FoS 2.90-4.47, flanges carrying axial load, rocker and collar captured.
- **Crank shaft:** located axially.
- **Coupler-eye bushings:** pressed in counterbored eyes.
- **Torque transmission:** keys rated, Duracon gear bonds within published figures.
- **Bolted joints:** every one has its BOM screws and nuts at a stated place, sized by the stated rule and checked as bolts and seats.
- **Bonded and tied joints:** the bracket, motor bracket, panel and clamp bonds and the packed-part, former and pitot ties are checked.
- **Encoder actuator:** fixed by the maker's set screws, on the non-ferrous shaft the maker requires.

**Build (C/D):**
- **Within rating:** the D36V50F7 input over the 3S range, the HV6160's 7.2 / 7.8 V rows, the Tekko32 / motor / FC cell ranges, motor current (23 of 26 A), and every tie loop.
- **Drawn sizes:** all 22 bought parts at catalogue dimensions to 0.05 mm.
- **Catalogue:** every BOM id exists, with none sold out or end-of-life (the motor is on backorder, the regulator rationed).
- **Flight-model geometry:** the tail position, wing hinge and semispan match the drawing.
- **GPS:** clear sky view.
- **Other lumps:** motor, gears, ESC, FC, GPS, tail servos, receiver and airspeed, encoder, regulator, pitot and boom are within about 1 mm of their drawn parts.
- **Airspeed board:** Matek publishes no dimensions for the ASPD-4525, confirmed.
- **Battery mass:** 143 g is the maker's own figure.

---

# A. Mass reconciliation (v23 -> v24, -56.0 g)

Method: `git show 323b0ab:data/designs/larus-gl-2.json` (v23, read-only) diffed line by line against
`data/designs/larus-gl-2.json` (v24); each line's v24 basis read in `src/design/build.js`, `airframe.js`,
`linkage.js`, `wing-solids.js`, `clearance.js`; M7 member list in `structure.members`; peaks in
`out/verification/M7-larus-gl-2.json`. Scratch copies: `scratchpad/v23.json`, `scratchpad/bom-structure.txt`.

Line diff (g, v23 -> v24), reproduced: shoulder brackets -5.94 x 2; aft seat bosses -5.24 x 2; formers 12.60 -> 2.13;
M3 screws 28.00 -> 16.00 + 4.92, nuts 9.60 -> 6.40; panels 22.55 -> 17.68; spars -0.62 / -1.28, LE/TE rods -0.47 / -0.28;
torque rods 33.15 -> 30.62; fwd tray doubler 4.98 -> 2.49; boom 46.21 -> 44.42; plate bosses 27.00 -> 25.88; riser
5.29 -> 4.39. Up: plates +5.70, clamp blocks +3.80, crank 40.51 -> 41.50, pins +0.10 x 2, rockers +0.15 x 2, forward
tray -0.23. Structure 733.87 -> 685.19 g; parts unchanged 836.24; margin 235.52 -> 228.21.

## Lines that went down, judged

| Line | Change | Verdict | Why |
|---|---|---|---|
| Shoulder brackets | -11.9 | **Sound** | v23 counted each arm over the full pin-to-rail distance (21.1 mm) on top of the ring and the sleeve. v24 draws the arm from the bushing flange's radius (6 mm) to the rail's surface: 21.132 - 6 - 6.5 = 8.6 mm (airframe.js:206). The free web between ring OD (9.75) and sleeve OD (8.5) is only 2.9 mm, so the bracket is a compact lug. M7 still checks the arm at the full 21.132 mm lever (`shoulder bracket ... arm` dims lever_mm 21.132, airframe.js:186), which is conservative. The arm bar still overlaps the ring and the sleeve by about 340 mm3 per bracket, and both lines count that volume, so the BOM is about 3.7 g high here (conservative). |
| Aft seat bosses | -10.5 | **Sound** | The bosses are gone. Their job moved to a 2.0 mm aft plate (+5.7 g on plates) and a 1.4 mm rail lock flange with 2 M3 screws per rail on the aft face. All of it is checked: plate seats 1.738 / 1.752, lock screws 3.380 / 3.619, screws in the plate 1.626 / 1.741 (M7 record). `aft_boss_mm: 0` in params.linkage.airframe.rail. |
| Formers | -10.5 | **Undersized and incomplete** | See finding A1. The 2.13 g is the right mass for what is drawn. What is drawn cannot hold the covering. |
| M3 screws and nuts | -10.3 | **Sound** | Tally from the sequence: 20 screws (aft tray 2, rail locks 4, fwd tray 2, riser 2, nose tray 2, boom clamps 4, motor 4) and 16 nuts (all but the 4 motor screws, which go into the motor's threads). This matches the BOM's 16 x M3x8 + 4 x M3x12 and 16 nuts. The rail locks went from 12 screws to 4 and nothing else was dropped. The per-part masses are computed upward-biased (max head + nominal shank: 1.00 g, 1.23 g, 0.40 g). |
| Panels | -4.9 | **Sound** | v23 strips were 61.6 mm plus two 24 mm flanges. v24 runs between the inner faces (59.8 mm), with 278 / 315 mm2 end flanges; the volume reproduces to 0.1%. M7 checks each panel over a 61.4 mm span (conservative) and its flange bond (2.018 / 2.173). Cosmetic: the "top panel" sits at y -81 mm and is the floor (member note "x-z plane at y -81 mm"; sequence step 3 "floor"). |
| Spars and edge rods | -2.6 | **Inner spar and rods sound. Outer spar incomplete** | Inner spar: from the socket's base (7.25 mm) to the elbow, 20 mm in the bonded socket, which M7 checks (spar root socket, spar root bond). Edge rods: root rib to tip, ending in the root rib. Outer spar: findings A2 and A3. |
| Torque rods | -2.5 | **Incomplete** | Finding A2: the rod now ends where the spar ends, and nothing is connected to either of its ends. |
| Fwd tray doubler | -2.5 | **Sound** | 24 -> 12 mm high, centred on the two flange screws (e = 6 mm = 2d). The plate check takes it as unbonded (t_eff 2.21 mm). |
| Keel and boom | -1.8 | **Shortened past what it must reach** | Finding A4. |
| Plate bosses | -1.1 | **Sound** | Each boss is "as long as the bushing, beyond the plate", so the 0.4 mm thicker aft plate shortens it. |
| Riser | -0.9 | **Sound for strength, marginal to make** | The -0.9 g is the BA100 hole: r = 11 + 0.5 mm, a 23 mm hole at the crank axis (y -56.0 mm). M7 checks the riser only at its root, on the full channel (`nose tray riser at the root`, channel 37.6 x 7 x 0.8). At the hole the moment is the same and Zo falls about 10% by my section calculation, so FoS stays near 4.3. The hole's top edge is 1.09 mm below the web's top edge, where the 0.8 mm tab is bent forward under the nose tray (riser y -79.6 to -43.4 mm; hole top y -44.49). That is less than the usual 2t + r between a hole and a bend (my rule of thumb; not sourced here), so the hole will distort when the tab is formed. Finding A8. |

## Lines that went up

- Gearbox plates +5.7, clamp blocks +3.8, rockers and pins +0.5: these follow the drawing and are checked.
- Crank +1.0 (split): the volumes reproduce exactly (shaft 7149.1 mm3, web and pin 2164.1). See finding A5 for the web's hub.

## The three confirmations

1. **Mass model = BOM sum, nothing counted twice: CONFIRMED.**
   - The lumps sum to 1514.27 g and the two wings to 2 x 117.687 g, which makes 1749.64 g, the BOM total.
   - Lump by lump, each one is its BOM lines x 1.15:
     - motor 95; gears, bushings and bearings 448.74; couplers 31.34; ESC 5.8; FC 9; RX + ASPD 4.0; GPS 8.3; tail servos 43; encoder + BA100 11.0; regulator 7; pitot 4.51; boom 44.42; film shell + formers 5.31; tail 14.91; battery 143;
     - each wing: spars, rods, ribs, film, servo, torque rod, 204.67 / 2;
     - "crank, rockers, plates": every remaining structure line (441.43 g x 1.15 = 507.63).
   - The 53.5 g double count from the first build is gone (build.js:183).
   - Residuals inside the 10% volume tolerance (sev 4): drawn minus BOM is +2.75 g net. The rockers are +1.60 each and the couplers +1.35 against the plates' -1.64; the bracket/ring overlap is double counted (about +3.7 g in the BOM). So "every line follows its drawn solids" holds to about ±3 g, not exactly.
2. **Nothing physical fell out of both: NOT CONFIRMED.** Every v23 line is accounted for. The bosses, the one-piece crank and the old screw counts were replaced, not dropped. But the BOM is short of parts that the function needs and that the brief's ruling does not exempt:
   - pitot pressure tubing is "in the 15% margin" (build.js:187-188, line basis). It is neither wire nor glue, so the completeness ruling does not exempt it;
   - no elbow joiner (A3);
   - nothing connects the torque rod (A2);
   - no pod stringers or nose/tail-cone support (A1).
   B and C/D add the rest (linkage hardware, connectors).
3. **15% margin as SPEC says: CONFIRMED.**
   - margin = 0.15 x (parts + structure) = 228.21 g, and every lump and wing point mass carries x1.15 (build.js:269, 279-289).
   - It still applies on top of fasteners that are now BOM lines (41.6 g of screws, nuts and ties), which is conservative.
   - The margin cannot stand in for connectors or tubing.

## Findings from A (ranked)

A1 (sev 1). Formers -10.5 g: the covering ("fuselage film shell", exempt as "the loft itself") cannot exist on what is drawn.
- **What is drawn:** five flat balsa rings, 4 mm radial x 2 mm thick, at x -100, -30, 97, 165, 200 mm (params.linkage.airframe.pod; airframe.js:577-600). Nothing runs between them.
- **The loft (body.loft.sections) runs from x +257.8 to -315.1 mm**, so the covering is unsupported over:
  - **the nose:** 58 mm forward of former 5, the pitot's skin hole included;
  - **the tail cone:** 215 mm aft of former 1, which tapers to a 10 mm point at (x -315, y -27). Nothing is at that point (the boom is at y +62);
  - **bays of 70 to 127 mm** between rings.
- **Why it cannot stand:** a taut film tube between two rings of radius R about 65 mm, 127 mm apart (2h/R about 1.9), has no equilibrium shape. A minimal surface spanning two coaxial rings exists only for 2h/R < 1.325. So the shrunk film necks inward onto the trays and the gearbox. That contradicts the containment clearance (battery 0.733 mm to the loft) and the drag shape.
- **The rings themselves:**
  - each hangs on a 2 x 2 mm balsa tab 23-41 mm tall (former 4: band at y -78.9, nose tray flange top at -37.8);
  - no member check: `structure.members` has only "pod formers ties";
  - the stated reason "they carry only the film" is contradicted by the tie check, which loads them with the pod's drag at 25 m/s (1.03 N) and its inertia.
- **v23's 12.6 g** was a nominal estimate, but the v24 drawing is a shrunk, incomplete structure rather than a corrected one.
- **Smallest fix:**
  - longerons from the catalogue rod `rod-ec-1-5mm` (Easy Composites 1.5 mm pultruded carbon rod, 2.7 g/m): about 8 x 0.57 m = 12.3 g;
  - a nose block and two tail-cone formers;
  - formers in `plate-ec-1mm` (Easy Composites 1 mm carbon plate, 1.24 kg/m2): 7100 mm2 gives 8.8 g, against 2.13 g of balsa;
  - in all about +20 g, +23 g with margin (computed from catalogue figures; the nose block is unsourced).
- That is 40% of the 56 g that made M4 final, so M4's FINAL standing must be re-flown after the fix.

A2 (sev 1). The twist torque rod (-2.5 g) and the outer spar (-1.3 g) stop 36.7 mm short of the tip rib, and nothing joins the rod at either end.
- **Where things end:**
  - semispan from the shoulder axis L = 0.975 m (not 1.0 m), rib 10 at s 0.9740 m;
  - outer spar and rod end at s 0.9383 (params.wingStructure.spar_to_m; wing-solids.js:85-95, build.js:106, 209);
  - the spar ends because the swept LE crosses its line: at s 0.974 the spar axis is ahead of the LE (x/c -0.004).
- **Rib 10 is declared "on the spar" (wing-solids.js:123) though the spar ends 35.7 mm inboard of it.** It is not joined to the rod at all (line 127, `s <= sEnd`). The tip rib therefore sits on the 2 mm and 1.5 mm edge rods alone, cantilevered 102.4 mm from rib 9 (s 0.8716).
- **Nothing joins the rod:**
  - the rod's outboard end is inside the spar's bore, joined to nothing;
  - its inboard end starts 0.5 mm past the inner spar's end, inside the outer spar's bore (line 93), out of reach of the servo at the elbow.
  - So the commanded twist (twist_out 1 at the tip) has no drawn path from the servo to any rib.
- **Tip load path, my estimate, not a gate run:**
  - tip bay rib 9 to tip is 0.0249 m2; at 250-350 N/m2 local peak loading half its load goes to rib 10, about 3-4 N at 102 mm, so 0.3-0.4 N m;
  - capacity of the two rods at the catalogue's lower compressive strength (200 MPa): 223 N mm;
  - so FoS is about 0.6, against a limit of 1.5.
  - It is outside M7, which has no rod, rib or tip member.
- **Smallest fix:**
  - run the outer spar along a constant-chord-fraction line (about 12% chord) to rib 10: +37 mm per side of `tube-cst-pultruded-8x7` (CST CS-529, 17.5 g/m), +1.3 g;
  - carry the rod to rib 10 and connect it there. The rod-to-rib arm is unsourced: no catalogue part.
  - This moves the twist axis, which is a flight-model input. Otherwise, put a rib at the spar's end and give the edge-rod cantilever an M7 member.

A3 (sev 1). The elbow joint is "butted to the inner spar at the elbow, bonded" (wing-solids.js:89) and is in no check.
- **What is drawn:** an 8/7 mm tube end-bonded to a 10/8 mm tube end. The outer spar's 8.000 mm OD cannot slide into the inner spar's 7.976 mm ID (catalogue). The outer spar's bore is taken by the torque rod, so no internal spigot fits either.
- **The load it must carry:** M7 gives the outer spar 5.913 N m at the elbow (out/verification/M7 spar_outer).
- **What a butt bond can carry:** even crediting the whole 8/7 annulus (Z = 20.8 mm3) at Araldite 2011's catalogue 9.55 MPa, about 0.20 N m, so FoS about 0.03. In fact the two end faces overlap only between r 3.988 and 4.000 mm, about 0.30 mm2 (B's figure, which I confirmed), so the joint has essentially no capacity.
- **Smallest fix:** a stepped external joiner sleeve, machined `metal-al-7075-t6` (2.81 g/cm3), bores 10.0 / 8.0 mm x 20 mm each, OD 12 mm. That is about 5.5 g per wing (computed; not a catalogue part), with its bond and bearing checks. Alternatively a hinge with bearings, if the outer panel is meant to rotate for twist: see B.

A4 (sev 2). Boom -1.8 g: it ends 5 mm aft of the tail's spar rod (airframe.js boom; BOM basis).
- **What that leaves bare:** the tail root chord is 0.1906 m, so the aft 75% of each root rib (about 140 mm) overhangs the boom's end. The two T18R ties sit forward of the spar (wing-solids.js:200-210).
- **Why the boom stops there:** the record says it ends there because "Running it to the trailing edge moved the CG 45 mm aft" (final-gates-larus.txt:67-68). A part was sized by its effect on trim rather than by its function.
- **What M7 checks:** each V surface's root moment and the tail's pitching moment rest on two ties through 2 mm balsa root ribs. M7 checks those ties for force only (`tail surfaces ties`, airframe.js:646-650: tail_F plus inertia, no moment couple). There is no rib-bearing or moment check.
- **Smallest fix:** a rated root mount, for example a machined 6061 saddle on the boom with a spar socket, from catalogue `metal-al-6061-t6`. Its mass is unsourced until drawn. Add the root moment to the tie / mount check.

A5 (sev 2). The split crank web's hub is a 2 mm wall, 0.8 mm at the keyway, and nothing checks it.
- **Geometry:**
  - the web is a 14 mm-wide capsule round the throw, bored 10 mm (linkage.js:137), so 2 mm of titanium surrounds the stub;
  - the DIN 6885 3 x 3 key's hub depth is h - t1 = 1.2 mm (params.linkage.crank.web.key), which leaves 0.8 mm behind the keyway.
- **What M7 checks:** the web as an arm (14 x 6 at a 12 mm lever, which is conservative) and key crushing on the web side (1.845). Nothing checks the hub ring's shear-out or hoop stress under the key's 3.2 kN (16.14 N m / 5 mm).
- **Smallest fix:** a 20 mm hub boss on the web, about +2 g of Ti-6Al-4V (catalogue `metal-ti-6al-4v-annealed-bar`, 4.43 g/cm3; computed), and a hub-wall check.

A6 (sev 3). The outer spar is drawn and rated on the listing's nominal 8 x 7 mm. The catalogue's own mass_note says CST's spec sheet lists this tube as 7.8 x 6.8 mm, 18.1 g/m.
- At 7.8 / 6.8, Z falls 5.4%, so the outer spar's FoS goes from 2.81 to about 2.66. The torque rod's radial clearance becomes 0.225 mm, not 0.325.
- The mass rises 0.6 g/m.
- **Fix:** rate on the smaller section and count mass on the heavier figure.

A7 (sev 3). The pitot's silicone pressure lines are carried in the 15% margin (build.js:187-188; BOM pitot basis). The completeness ruling exempts only glue, wire and the margin itself, and tubing is a physical object with a fixed run.
- **Fix:** a BOM line for the tubing. A catalogue part is needed; none is in data/parts (unsourced).

A8 (sev 3). The riser's actuator hole comes within 1.09 mm of the tab bend, and the riser is checked only at its root.
- **Fix:** move the hole down or make it a slot, and check the section at the hole.

A9 (sev 4). Stale record text.
- **Shoulder housings line:** "mounting to the frame not counted here: in the 15% margin with fasteners". The mounting is now the bracket line.
- **"pod formers ties" note:** "balsa formers to the boom and trays". They are tied to the trays only.
- **clearance.js:691 assumption:** "screws and their heads are not drawn". v24 draws them.
- **Shoulder pin basis:** "6 mm x 43 mm", where the drawn pin is 43.4 mm.
- **Record and brief:** both say "1.0 m semispan". The structure's semispan from the shoulder axis is 0.975 m.
- **Ledger:** calls the rib envelopes "full outlines". They are bounding rectangles in (x, n).

Checked and found sound in A: shoulder brackets, aft seat bosses, screw/nut tally, panels, inner spar, edge rods, doubler, plate bosses, riser strength, plates, clamp blocks, rockers and pins, the crank's volumes, the mass-model sum, the margin. Battery mass: Gens Ace's own page gives "Net Weight(±20g): 143" (genstattu.com, SKU GEA223S35DGT, read 2026-11-03), so the 143 g is the maker's figure, and its ±20 g lies inside the ±5% mass corner.


---

# Audit v24, question B: function audit of mechanical interfaces and fixings

Scope: v24 (`data/designs/larus-gl-2.json`, a558baf), built by `src/design/*`; M7 results from
`out/verification/M7-larus-gl-2.json`. This is read-only. Positions are body x in mm, taken from
`fullAssembly(d, catalog)` (`src/design/clearance.js:1074`) through the scratchpad script
`docs/status/audit-v24-asmx.mjs` (a copy of the auditor's scratch script).

Severity: (1) breaks "real parts, assembled"; (2) could change a gate verdict; (3) disclosure or record gap; (4) cosmetic.

## Interface table

| # | Interface | What carries the load (BOM line) | What holds it axially (BOM line) | M7 check | Flag |
|---|---|---|---|---|---|
| 1 | Shoulder pivot, each side (rocker on the shoulder pin in 2 bushings) | iglidur G GFM-0608-06 x2 (bushing-igus-GFM-0608-06) pressed in housing rings ("shoulder housings") on a bare 7075 pin ("shoulder pin") | Captured: the hub's aft face against the aft bushing flange, the pin's integral collar against the forward flange, 0.5 mm running clearance. The flanges take root_x | Bushings FoS 2.90 to 4.47, flange axial; pin 1.76; housings (lug) | Pin-to-hub fit not stated (F10); bare Al shaft (F11) |
| 2 | Shoulder pin in the rocker hub | Hub seat (rocker) | "clamped in the rocker hub" (`linkage.js:221`), but no clamp exists. Drawn as one solid of the rocker (`clearance.js:496`) | Rocker hub seat 3.79 | Fit implied, not stated (F10) |
| 3 | Crank shaft in the plates | GFM-1012-10 x2 in bossed seats (9 mm) | Captured within 0.5 mm: the web (aft) against the aft bushing flange (11.7 / 12.2), and the 15 mm wheel seat against the aft bushing's forward end (22.7 / 22.2). The axial channel is empty, and the record does not state this capture | FoS 1.86 (aft) | Sound |
| 4 | Crank web on the crank shaft stub | DIN 6885 3x3x6 Ti key ("key, crank web") | LOCTITE 638 on Ti: unrated | Key 2.663 / 1.845; axial "rc practice" | Confirmed open point (F13) |
| 5 | Coupler eyes on the crank pin (2 stacked) | GFM-0810-06 pressed in the eyes, flanges in counterbores | Nothing. The pin ends 1 mm past the L eye (pin -8.30, eye face -7.30). No groove, clip, washer or head | Bushing 2.36 / 2.55; pin 2.06 | MISSING (F5) |
| 6 | Coupler eyes on the horn pins | GFM-0810-06 | Nothing. `horn_pin_past_mm: 0`, so each pin ends flush with its eye's free face (R: pin -0.80, eye -0.80; L: pin -1.30, eye -1.30) | Bushing 2.36 / 2.55; pin 4.27 / 4.62 | MISSING (F5) |
| 7 | Horn pin in the horn boss | Boss seat (rocker) | "pressed in the horn boss" (`linkage.js:248`): press fit, no fit class, no rating | Boss 2.07 / 2.23 | Implied (F10) |
| 8 | Gear shafts 1 and 2 in the plates | GFM-0810-06 x2 each. Forward seats are a 1.2 mm plate with no boss (bushing body 71.2..76.2, plate 75.0..76.2) | Nothing. Plain 8 mm rods plate to plate (`clearance.js:375-378`), flanges on the plates' outer faces, no shoulder, clip or collar | Static on 1.2 mm supported length (honest), FoS 2.42 to 6.76 | MISSING (F4) |
| 9 | Gears on gear shafts 1 to 3 and the crank wheel | Keys (6 key lines), torque only | Nothing. Keys do not retain axially, and MSGA/MSGB/SSAG are keyway-only (catalogue `fastening`). E.g. on gear shaft 1 the stage-4 pinion is 4.5 mm from the aft bushing and the stage-3 wheel hub 17 mm from the forward one | Keys rated | MISSING (F4) |
| 10 | Gear shaft 3 in the plates | SKF 628/8-2Z (8x16x5) x2 | Nothing. Each bearing is seated over 2.0 mm (aft) and 1.2 mm (forward) of its 5 mm width, with no shoulder or ring. The shaft is free to slide (the DS1-60 can travel 3.3 mm aft onto the forward bearing) | ISO 76 C0, FoS 3.80 / 15.0, with no housing derate | MISSING axial (F4); seat (F6) |
| 11 | Motor shaft outboard | SKF 619/5-2Z (5x13x4) in the 1.2 mm forward plate (75.0..79.0, 1.2 of 4 mm seated) | Press fit only | FoS 3.41; the motor's own bearings are in `joints_unchecked` | F6 |
| 12 | DS1-20 on the motor shaft, DS1-60 on gear shaft 3 | LOCTITE 406 + SF 770 bond (RC practice) | Same bond | Within the published figure | Disclosed, sound |
| 13 | Coupler body | 6061 link | n/a | Buckling 2.00 / stress 2.89 | Sound |
| 14 | Inner spar in the rocker socket (wing to body) | Socket seat crush, coupler x c + root_mperp (2.64 / 2.83) | Araldite 2011 bond, root_x only (6.18 N, FoS 970) | Bond checked on axial force only | Torsion path unchecked (F7) |
| 15 | Elbow: outer spar to inner spar | "butted to the inner spar at the elbow, bonded" (`wing-solids.js:89`). Contact is a ring at r 3.988..4.000 mm, about 0.30 mm2 | The same butt | None (only spar_outer bending at the elbow, 5.913 N m) | INADEQUATE (F3) |
| 16 | Twist servo to the wing | Case "on the spar at the elbow" (`wing-solids.js:102-104`), 0.5 mm aft of the spar surface (case -38.49..-5.49, spar ±4.99): not touching | None: no frame, screws or ties. Not in the support ledger (its 12 rows have no twist servo) | Servo torque only (0.584 / 1.020 N m) | MISSING (F1) |
| 17 | Twist servo to the torque rod (the "horn linkage 1.6:1", `design.servos.twist.linkage_note`) | Nothing: no horn, link, clevis or rod lever in the BOM or the solids | n/a | n/a | MISSING (F1) |
| 18 | Torque rod in the outer spar | Ti tube in the CFRP bore, 0.325 mm radial (BORE exemption). No bearing, no declared plain pair | Nothing | Rod torsion 15.2 | MISSING (F1) |
| 19 | Torque rod to the hand wing | Nothing. The rod runs elbow+0.5 mm to s 0.9383 inside the spar's bore, so it can reach no rib. Ribs at s 0.5642, 0.6667, 0.7691, 0.8716 and 0.974 | n/a | n/a | MISSING (F1) |
| 20 | Hand-wing ribs on the outer spar (twist DOF) | "on the spar" (`wing-solids.js:123`), no bushing | n/a | No rib check | Contradiction with the flown twist (F1); rib bonds unchecked (F8) |
| 21 | V-tail surface to the boom | 1.5 mm spar rod "its root against the boom" (butt, `wing-solids.js:197`); balsa root rib "against the boom, tied" with 2 T18R (`:209`) | Ties | Ties on force only: 7.0 N vs 2 x 80 N, FoS 22.8 (`airframe.js:646-650`). No root moment, no tail spar check | INADEQUATE (F2b) |
| 22 | Tail servo to the V-tail surface (flown all-moving, ±30 deg, pivot at 25% c: `flight.js:454-457`, `build.js:440`) | Nothing: no pivot, bearing, horn or pushrod. The surfaces are tied fixed | n/a | Servo torque only (0.185 / 0.804) | MISSING (F2) |
| 23 | Tail servos on the aft tray | T40R x2 each | Ties | Inertia only (sf_origin, wdot, w2), not the output reaction | F14 |
| 24 | Rails in the plates and sleeves | Plate seats (lug), sleeve bonds | Lock flange on the aft plate's aft face, 2 x M3x8 + nut per rail | Seats 1.738 / 1.752; screws 3.38 / 3.62; screws in the plate 1.626 / 1.741 | Sound. The flange is implied integral (F16) |
| 25 | Shoulder brackets on the rails | Araldite 2011 Al sleeve bonds (T, F) | The same | Checked | Sound |
| 26 | Motor on its bracket; bracket on the rails | 4 x M3x8 into the motor's back-face threads; sleeve bonds | The same | Bolts, seat, plate arm 7.99, bond 15.76 | Thread depth not in the catalogue (F12) |
| 27 | Plates to panels | Flange bonds (lap) | The same | 2.018 / 2.173 | Sound |
| 28 | Trays, riser, nose tray, doubler | M3x8 + ISO 4032 nuts by `fixingLayout` / `pickScrew` | Nuts | Bolts, seats, plates | Sound |
| 29 | Boom clamps to the plate posts; boom in the clamps | 2 x M3x12 + nut each; Araldite 2011 in the 9.982 mm bores | Bond | Bolts, seats, posts; bond 11.0 | Sound |
| 30 | Encoder actuator on the crank stub | 3 x M3 DIN 913 (maker's, included); non-ferrous shaft (Ti), as the maker requires | Set screws | n/a (light) | Sound. The shaft's g6 is not stated (F17) |
| 31 | Readhead on the riser | 3 x M2x25 8.8 + 14 mm spacers + M2 nuts | Nuts | 1113 / 1641 | Sound |
| 32 | Packed parts, formers, pitot on the trays | T18R / T40R ties | Ties | Tie loop rating | Sound for inertia |
| 33 | Formers (pod frame) | 4 x 2 mm balsa bands; tab tied to the tray | Tie | No member check | Confirmed (F9) |

## Findings, ranked

**F1 (1). The wing-twist drive does not exist as hardware, at either end.**
The flight model flies commanded hand-wing twist through a "horn linkage 1.6:1"
(`design.servos.twist.linkage_note`; `build.js:202-205`: "turns the hand wing through a torque rod"). The drawing has
none of that mechanism:
- **Torque rod.** It sits wholly inside the outer spar's bore, from 0.5 mm past the inner spar's end to s 0.9383
  (`wing-solids.js:92-95`). Its inboard end is enclosed at the butt joint, and its outboard end stops inside the spar,
  short of the tip rib at s 0.974. No horn, lever or link can be fitted to it without cutting the spar, and no rib can
  be fixed to it. The rod is joined only by the BORE and RIB_HOLE clash exemptions, neither of which is a fixing.
- **Servo.** The servo case sits 0.5 mm aft of the inner spar (`x1 = xs - si.ro - run`, `:102`; case -38.49..-5.49
  against spar ±4.99), so it touches nothing. It has no frame, screws or ties, and it is outside the support ledger.
- **Hand wing.** The hand-wing ribs are "on the spar" (`:123`), and the outer spar is bonded rigidly to the inner one
  (`:89`). Even with a drive, nothing is free to twist, and no bushing lets a rib turn on the spar.

M7 checks only the servo's torque (0.584 of 1.020 N m) and the rod's torsion (FoS 15.2), on a load path that is not
there. This confirms round 16's open point and widens it.

Smallest fix: there is no catalogue-only fix, because the layout itself conflicts. The rod must leave the spar to take
a lever, and the elbow joint needs that same space (F3). A drawn mechanism needs:
- a servo frame and its screws;
- a servo arm;
- a link with two clevises or ball links;
- a lever on the rod, outside the spar;
- a rod bearing at the elbow;
- a rod-to-tip-rib fixing;
- rib bushings on the spar, if the ribs are to turn.

Every one of these parts is unsourced (the catalogue has none).

**F2 (1). The V-tail actuation does not exist.** `flight.js:454-457` flies all-moving surfaces with the pivot at 25%
chord, and `deflection_max_rad` is 30 deg (`build.js:440`). M7 checks the tail-servo torque. The drawing ties each
surface's root rib fixed to the boom with T18R ties (`wing-solids.js:197-209`), and has no pivot, bearing, horn or
pushrod. The servos are on the aft tray at x -51..-22 mm and the tail is at x -946 mm, so about 0.9 m of pushrod is
missing per side, with its exits through the pod. No BOM line contains "pushrod", "horn" or "clevis": grep finds only
the obsolete stroke-servo pushrods, `build.js:185-195`, which are not built since `fixedStroke` is true.

Smallest fix:
- two pushrods: catalogue `rod-ec-2mm` at 4.5 g/m, about 4.3 g each at 0.95 m (length estimated);
- a pivot tube with bearings at the boom, and a horn on each surface: unsourced;
- clevises: unsourced;
- the surfaces untied from the boom and pivoted on their spar rods.

**F2b (2). The tail-to-boom joint carries no moment, and the tail spars are outside M7.**
- **The joint.** Each surface's 1.5 mm spar rod only butts the boom. The two T18R ties are checked on the tail's
  force alone (7.0 N; `airframe.js:646-650`; FoS 22.8), not on its root bending.
- **My estimate** (derived, not from the record). On the area centroid of a 0.567 m, taper 0.7 surface (0.267 m),
  with about 0.6 x 7.0 N per surface, the root moment is about 1.1 N m.
- **Capacity.** A 1.5 mm rod at the SPEC rod strength of 400 MPa holds σπd³/32 = 0.13 N m, before the 1.5 factor of
  safety. The spar rod and the trailing-edge rod are side by side in the chord plane, so they add (0.26 N m) but form
  no couple. The FoS is about 0.2.

If M7 checked this member, it would fail.

Smallest fix: a bonded root socket on the boom, and a tail spar sized by the M7 spar rule. The catalogue's
`tube-cst-pultruded-5x4` (11.1 g/m) is a candidate. Its adequacy is unverified.

**F3 (1). The elbow joint cannot carry the wing.**
- **What is drawn.** The outer spar (8.0 / 7.0 mm, catalogue nominal) is "butted to the inner spar at the elbow,
  bonded" (`wing-solids.js:89`). The inner spar is 9.982 / 7.976 mm. Their end faces overlap only between r 3.988
  and 4.000 mm, about 0.30 mm2.
- **What it must carry.** M7's elbow moment is 5.913 N m (spar_outer; stress 284 MPa in the tube there).
- **The bond.** Araldite 2011 counts 9.55 MPa on CFRP, and that is lap shear. A butt in tension has no published
  figure at all.
- **Not checked.** No M7 member checks the joint.

Smallest fix: a spigot or sleeve joint, bonded over a length sized by the Araldite CFRP rule. Note the catalogue's
own spec-sheet dimensions for the CST tube, 7.8 / 6.8 mm (`od_mm_spec_sheet`). That OD would enter the 7.976 mm bore,
but it would also bury the torque rod's inboard end inside the inner spar (see F1). The joiner is unsourced.

**F4 (1). Nothing in the gear train is located axially, except the crank shaft.**
- **Shafts.** Gear shafts 1 to 3 are plain 8 mm rods plate to plate, with the bushing flanges on the outer faces and
  no shoulder, clip or collar (`clearance.js:375-378`).
- **Gears.** Every keyed gear can slide along its key. The catalogue `fastening` field gives only a keyway for
  MSGA1-18, MSGB1-45 and SSAG1-40J10.
- **Bearings.** The 628/8 and 619/5 have no shoulder or retaining ring.
- **Load.** All bearing Fa are 0 in M7, but landing and launch accelerations along x act on gears of 92 to 130 g.
- **Hardware.** The catalogue holds no retaining ring, collar, washer or set screw.

Smallest fix: DIN 6799 retaining washers, nominal size 5 (shafts 6 to 8 mm, s 0.7 mm, d3 11.3 mm, 0.236 kg per 1000
pieces, so 0.236 g each: fasteners.eu DIN 6799 table, read 2026-11-03).
- Two per shaft outside the flanges, and one on each keyed gear's free face (or 6061 spacer tubes, like the encoder
  spacers): about 18 off, about 4.2 g.
- The shafts need grooves, so M7 must check them at the groove.
- Bearing retention: a DIN 471 8 mm ring, or a seat shoulder. Unsourced.

**F5 (1). Nothing holds the coupler eyes on their pins** (round 16's open point, confirmed and sharpened).
- **Horn pins.** `params.layout.horn_pin_past_mm` is 0, so each pin ends flush with its eye's free face (R -0.80 /
  -0.80; L -1.30 / -1.30). There is no material to take a clip.
- **Crank pin.** It ends 1 mm past the L eye (-8.30 / -7.30) and has no groove.
- **The record.** The shoulder pins' collar is real (sound). The record says the collar retains them, and it does.

Smallest fix: lengthen each pin by about 1.5 mm and fit a DIN 6799 size 5 (0.236 g; sourced as F4) on the crank pin's
end and on each horn pin: 3 off, 0.71 g. Then re-run the clearance check.

**F6 (3). The ball bearings are seated over a fraction of their width, and C0 is credited in full.**
- 628/8-2Z: 1.2 of 5 mm (forward) and 2.0 of 5 mm (aft).
- 619/5-2Z: 1.2 of 4 mm.

The bushings are honestly derated to their supported length (`bearings.js:90-94`), but rolling bearings get no
equivalent (`bearings.js:71-79`). The bearing FoS minimum is 3.41, so a moderate derate would not flip M7. SKF's
housing-support condition is unsourced here.

Smallest fix: bosses as long as the bearing on those seats (as `plates.bossed` already does for three seats), or a
stated reason the rating holds.

**F7 (3). The spar root bond is checked on the axial force only.** It sees 6.18 N, FoS 970 (`linkage.js:246`). A
round spar in a round socket can carry torsion about its own axis only through that bond: the aerodynamic pitching
moment plus the twist servo's reaction. But M7 adds the "pitch" part of root_mperp to the socket's crush couple
(`linkage.js:245`; `joints.js:96`, "lead-lag and pitch"). The bond's torsional capacity is about 30 N m (9.55 MPa x
πd²L/2, my estimate), so it probably passes. It is still unchecked.

**F8 (3).** The ribs, and their bonds to the spars and rods, carry the wing's air load into the spar but have no M7
member (none appears in `structure.members`). The tail ribs are the same.

**F9 (3). Formers: confirmed, there is no member check.**
- **Load.** The record says they "carry only the film". But the pod formers ties member routes the pod's inertia plus
  1.03 N of drag at 25 m/s through them (`airframe.js:636`).
- **Strength.** The loads are tiny, so strength is probably fine.
- **Unquantified.** Shape-holding under the film's shrink tension, over spans of up to 127 mm (x -30 to 97) with no
  stringers, is not quantified anywhere.

**F10 (3). Fits that are implied but not rated.**
- The horn pins are "pressed in the horn boss" (`linkage.js:248`), with no fit class or holding figure.
- The shoulder pin is "clamped in the rocker hub" (`linkage.js:221`), with no clamp in the BOM; the pin is drawn as a
  solid of the rocker (`clearance.js:496`). Either it is an unrated press fit, or the hub turns on the pin, which is
  7075 on 7075 with no declared plain pair.

**F11 (3). Plain-bearing pairs: the shaft materials are outside igus's tested shafts.**
- **What the shafts are.** Every bushing runs on bare 7075-T6 (gear shafts, shoulder and horn pins) or Ti-6Al-4V
  (crank shaft, crank pin).
- **What igus says.** igus's iglidur G data tests "aluminium, hard anodised" and says the recommended shaft hardness
  rises with load. It does not mention titanium, and it recommends h9 shafts with Ra 0.8 µm. Source: the igus.co.uk
  iglidur G material data, read 2026-11-03.
- **No wear check.** M7 has no wear or pv check. Peak p x v reaches 43.1 MPa x 0.119 m/s, about 5.1 MPa·m/s, at the
  crank aft bushing. Those are peaks, not simultaneous, and igus's pv limit is unsourced here.
- **Speed limit.** The oscillating joints (shoulder, horn eyes) are judged against the rotating limit of 1 m/s. The
  igus oscillating permanent limit is 0.7 m/s. Every oscillating speed is 0.12 m/s or less, so no verdict is affected.

**F12 (3).** The motor screws are M3x8 through the 2 mm bracket, about 6 mm into the back-face threads. The
catalogue entry records no thread depth, so engagement and bottoming are unverified.

**F13 (3).** The crank web's LOCTITE 638 axial bond on titanium is unrated, as the record states. Confirmed.

**F14 (3).** The tail servos' ties are checked for inertia only, not the servo's output reaction (stall 0.80 N m).
That matters once F2's pushrods exist.

**F15 (4).** The shoulder housings line still says "(mounting to the frame not counted here: in the 15% margin with
fasteners)". v24 draws the housing, arm and sleeve as one part (`clearance.js:543-544`).

**F16 (4).** The rail's lock flange is counted in the rail line, so it is implied integral. No flange-to-rail joint is
stated.

**F17 (4).** The encoder actuator wants a g6 shaft (catalogue `fixing_text`). The crank shaft's tolerance is not
stated.

## Checked and found sound

- **Shoulder pivots.** Load, the flange's axial reaction, housings and speed are all checked. The rocker and collar
  are captured between the flanges.
- **Crank shaft axial capture.** The web and the wheel seat each sit 0.5 mm from the aft bushing. The record does not
  state it, but it holds.
- **Coupler eye bushings.** Pressed in, with flange counterbores. Radial FoS 2.36 to 2.55; lugs 5.4 to 5.8.
- **Torque path through the gears.** The keys are rated, and the DS gears' bonds sit within their published figures
  (RC practice).
- **Bolted joints.** Every one has BOM fixings placed and sized by `fixingLayout` / `pickScrew` and checked as bolts
  and seats: rail locks, trays, doubler, riser, nose tray, boom clamps, motor, readhead. The counts reconcile:
  16 M3x8 = 4 lock + 8 carrier + 4 motor; 16 nuts = 4 lock + 8 carrier + 4 clamp; 4 M3x12 for the clamps.
- **Bonds checked.** Bracket sleeves, motor bracket sleeves, panel flanges and boom clamps.
- **Ties.** Packed parts, formers and pitot, for inertia.
- **Encoder actuator.** Set screws included by the maker; non-ferrous shaft.
- **Partially housed bushings.** M7 derates them honestly to their supported length.

## Outside B, noted for the other sections

- **(D)** The CST outer spar is drawn at 8.0 / 7.0 mm. The catalogue's spec-sheet fields say 7.8 / 6.8 mm and
  18.1 g/m.
- **(A)** The tip rib at s 0.974 is joined "on the spar", but the spar ends at s 0.9383. It is held only by the LE and
  TE rods.
- **(D)** `wing.twist_stiffness_Nm_rad` 0.05 is tied to no drawn torsion member. If the 8 / 7 mm spar itself twists,
  its GJ/L is far higher: about 2 N m/rad at G of about 5 GPa over 0.44 m, my unsourced estimate.


---

# Audit v24, questions C and D (electrical, operational, other "real parts, assembled" breakers)

Scope: Larus GL-2 v24 (a558baf, data/designs/larus-gl-2.json). Read-only. Scripts used (all under this scratchpad, `cd/`):
`boxes.mjs` (per-part bounding boxes of the built assembly at the sequence pose), `section.mjs` (wing section vs spar and
twist servo), `lumpcheck.mjs` (each mass lump vs the drawn centroid of its BOM lines), `centroid.mjs`, `tags.mjs`.
Each imports `src/design/clearance.js fullAssembly` on the shipped design and catalogue; nothing is rebuilt or gated.
Severity: 1 breaks "real parts, assembled"; 2 could change a gate verdict; 3 disclosure/record gap; 4 cosmetic.
Masses marked "unsourced" were not read from a maker page in this audit.

## Findings, most serious first

### D1 (sev 1). The tail is flown as two all-moving surfaces; the drawn tail is cable-tied fixed, with no linkage at all
- Flight model: each V surface deflects up to `tail.deflection_max_rad` 0.5236 (30 deg), hinge moment "about the surface's
  quarter chord; pivot assumed at 25% c" (src/physics/flight.js:454-457), driven by the two MD89MW tail servos
  (M7: "tail 0.185 / 0.804" N m).
- Drawn: each surface is fixed solids (`S`, not moving `M`), "its root against the boom, through the root rib", held by
  two T18R ties round the boom (src/design/wing-solids.js:187-210). Sequence step 50/51: "its rods and ribs one piece,
  covered, along its own span onto the boom, its root rib through the ties".
- The servos sit on the aft tray at x -51.3..-22.3 mm (support ledger); the tail is at x -945.8 mm. Between them the BOM
  has no pivot shaft, pivot bearing or bushing, horn, servo arm, pushrod or pull-pull cable, guide, or clevis. The MD89MW
  catalogue mass explicitly excludes its horn ("Weight: 21.5g ... Excluding the weight of Horn").
- So pitch and yaw control, which M1-M5 all use, does not exist mechanically. (Overlaps question B's servo-to-linkage
  audit; it is here because it is also a gate input the drawn geometry contradicts.)
- Smallest fix: a pivot per surface (a rod through the root rib into a bushing on the boom, e.g. an igus iglidur flange
  bushing; part number and mass unsourced), a horn per surface, and two ~0.95 m pushrods. rod-ec-1-5mm is already in the
  catalogue at 2.7 g/m (about 2.6 g each). Clevises and servo arms are unsourced. Then draw them, add them to the
  sequence, and check them under M7.

### D2 (sev 1). The twist drive as drawn cannot twist the hand wing, at either end
- The torque rod lies wholly inside the outer spar's bore, from 0.5 mm past the elbow to s 0.9383 m
  (wing-solids.js:91-95; build.js:208-210). Clearance is 0.325 mm radial, exempt as BORE.
- The outer spar is "butted to the inner spar at the elbow, bonded" (wing-solids.js:89). Assumption 13 calls it a rigid
  elbow, "bonded into the inner one (spigot)", so the two descriptions also disagree.
- The hand-wing ribs are joined "on the spar" (wing-solids.js:123), that is, to the outer spar, which cannot turn.
- The twist servo case is outside the tube, aft of the spar (wing-solids.js:97-104).
- Nothing connects the servo to a rod sealed inside a closed tube. Nothing connects the rod to the ribs: the ribs ride on
  the spar, with the rod inside it.
- The flight model flies `twist_out 1` on segment 2 (wing.segments).
- Round 16 named only the elbow end. The outboard end is just as unconnected. (Overlaps B.)
- Smallest fix: the rod must leave the spar at both ends. Inboard, a horn on the rod through a slot, or the rod run
  beside the spar. Outboard, a tip rib or a set of ribs fixed to the rod, with the ribs free to turn on the spar
  (each then a plain-bearing pair).

### C1 (sev 1). The pack cannot be removed, charged or even plugged in without stripping the covering
- The drawn pack spans x 156.5..231.2 mm, y -42.8..-17.4 mm (boxes.mjs 'packed battery').
- It straddles former 4 (x 164..166) and former 5 (x 199..201). Both are full hoops on the skin (y up to +28.7 and +18.2),
  so lifting the pack hits their top arcs.
- Forward it is 0.733 mm from the nose skin (containment ledger). Aft it is 0.5 mm from the airspeed board (mount gap,
  params.layout.mounts.airspeed).
- Between former 3 (x 98) and former 4 (x 164) there are 66 mm, less than the pack's 74.7 mm length.
- Sequence: battery step 58 (down from above), formers steps 72-76 afterwards, then the film shell over all. The shell
  is exempt and is in no step.
- Declared openings: wing roots, horn slots, boom posts, pitot. There is no hatch, no hatch fixing, and no charge or
  arming access.
- The same applies to the receiver (bind; TBS's page does not say how it binds), to the FC's USB-C (the F405-WMO package
  includes a USB-C/beeper extender, which is neither drawn nor routed), and to every servo for centring.
- Smallest fix: a top hatch over x ~150..235 mm that carries formers 4 and 5's top arcs, with a stated fixing (for
  example 2 x ISO 7380 M3 into the riser flanges, or a magnet pair; mass unsourced), declared as an opening. Bring the
  USB extender to that hatch.

### C2 (sev 1). No connector of any kind is in the BOM or the catalogue
- The catalogue's 247 entries contain no connector, switch, plug or extension (category census).
- The pack, GEA22003S35D, ships with a Deans discharge plug on a 65 mm lead and a G-tech-4P balance plug (Gens Ace store
  page, part GA-B-35C-2200-3S1P-Deans-GT, read this audit). The mating Deans male on the PDB side is missing.
- Twist servos: the wing goes on last, along its span (sequence 82-83). Each servo's lead therefore needs a separable
  joint inside the pod at the wing root, plus about 0.5 m or more of extension from the elbow.
- Airspeed board: Matek's package has only a "JST-GH-4P to JST-GH-4P 20cm" cable (Matek page, read this audit). The board
  is at x 145 mm and the FC at x -68 mm, 213 mm apart in a straight line before routing round the gearbox, so the cable
  cannot reach.
- Wire and glue are exempt. Connectors are not.
- Solder joints are acceptable as wire: motor to ESC, ESC to PDB, regulator in and out.
- Smallest fix: BOM lines for one Deans Ultra Plug male (WSD Sales; part number and mass unsourced, about 1-2 g), two
  wing-root servo connector pairs (for example JR or MPX, unsourced), one longer JST-GH 4P cable (unsourced), and servo
  extensions (unsourced).

### D3 (sev 2). The flown CG is not the drawn CG: built about 5 mm forward and 8 mm higher than flown
- The flight model reads `design.mass.lumps` directly (flight.js:131-136). The packing's residual_cg_shift_m (-0.65 mm)
  is never applied.
- lumpcheck.mjs, drawn centroid of each lump's own BOM lines (volume-weighted over its solids at phase 0) against the
  lump position:

| lump | mass (g) | lump position (mm) | drawn centroid (mm) | CG error x, y (mm) |
|---|---|---|---|---|
| crank, rockers, plates (42 lines: rails, plates, bosses, trays, rockers, panels, fasteners, ties, ...) | 507.6 | 8.7, -56.0, 0 (params.lumpPos, an older round's crank-axis point), gyration 0 | 30.2, -18.5, -1.0 | +6.23, +10.89 |
| battery | 164.4 | 201.8, -5.0, 0 (y hard-coded, build.js:340) | 193.8, -30.1, 0 | -0.75, -2.36 |
| couplers | 36.0 | 0, 0, 0 | -1.1, -28.1, -0.6 | -0.02, -0.58 |
| twist servos (wing point, d_m 0) | 2 x 35.3 | on the spar | 22 mm aft of the spar (wing-solids.js:102-103) | about -0.9 in x |
| net | | | | about +5.2 mm x, +8.1 mm y |

- Every other lump matches its drawn parts within about 1 mm: motor, gears and bearings, ESC, FC, receiver and airspeed,
  GPS, tail servos, encoder, regulator, pitot, boom.
- The CG corners cover only x ±5 mm (src/physics/scenarios.js:491). There is no y corner.
- The record's "CG of the lumps 18.57, -32.99, -3.02 mm" and its inertias carry this error. The 441 g lump is a point
  mass with gyration 0, so Ixx, Iyy and Izz are also short of the drawn spread (x -110 to +228 mm).
- The 507.6 g lump's mass agrees with its 42 lines exactly (441.4 g x 1.15): A's mass check passes, and only position
  and inertia are wrong.
- Smallest fix: set each lump's position and gyration from its lines' drawn solids. The data is already in fullAssembly.
  Re-solve the battery x (see C4: it cannot move), then refly.

### D4 (sev 2). The spar and twist servo stand out of the wing's lower surface, and every rib's spar hole breaks out
- The spar is centred on the chord line (n = 0): wing-solids.js:81, with ribs and sections referenced to the same n.
- E387 lower surface at the spar's chord station (x/c 0.35-0.38): -0.0076 to -0.0064 c. Source:
  data/references/airfoils/e387.dat, local, read with python -I. That is -3.0 mm at the root and -2.4 mm at the elbow.
- So the 10 mm inner spar protrudes 2.0-2.6 mm below the skin. The 8 mm outer spar protrudes 1.6 mm at the elbow and
  about 1.0 mm at s 0.94.
- The twist servo case (n ±5 mm, x/c 0.38-0.47) protrudes 2.6-3.7 mm. section.mjs: case bottom 2.4 mm outside the
  lower surface even on the model's own NACA-form E387.
- The ribs are drawn as rectangles spanning the whole section, and the spar's hole in them is exempt (RIB_HOLE). The
  check therefore cannot see that each rib is cut through at the spar.
- The film cannot lie on the flown section there. The protrusion's frontal area, about 2 x 0.94 m x 2.5 mm =
  0.0047 m2, exceeds the whole body's CdA_front (0.0043 m2) before any Cd is applied. No drag is counted for it.
- Smallest fix: raise the spar axis about 12-14 mm, toward the camber line. That moves the rocker's socket and the
  wing-root opening, so the clearance and containment checks must be re-run.

### D5 (sev 2). The film shell cannot take the loft's shape: no nose block, no tail cone, no closing former
- Formers exist only at x -100, -30, 97, 165 and 200 mm.
- The loft runs from a nose point at x 257.8 mm to x -315 mm, a tail cone from 120 x 140 mm down to 10 x 10 mm over
  205 mm with no structure. It then continues as a 10 mm loft tube at y -27 mm to x -1088.8 mm, enclosing nothing. The
  real boom is at y +62 mm, outside the loft. That tube adds 0.0243 m2 of the 0.2004 m2 loft_wetted_area (body.loft).
- Taut film between former 5 and the nose point runs inside the elliptic loft. At the pack's forward corner (x 231 mm)
  a straight film would have a half-height of about 21 mm against the corner's elliptic radius about 1.1 (outside).
  The containment pass (pack 0.733 mm) therefore depends on a skin nothing holds.
- The wing-root slots (13 mm wide, sweeping about 82 deg of arc) also lie in a 127 mm unsupported panel between
  formers 2 and 3.
- Smallest fix: a nose block and a tail closing former in the BOM (balsa, the existing material, so the mass follows
  from its drawn volume), drawn and in the sequence. Drop or explain the 10 mm loft extension.

### C3 (sev 3). Nothing is cooled, and nothing checks heat
- The ESC is on the nose tray at x 117..134.3 mm. The motor is inside the gearbox. Both are in a closed film pod with no
  inlet or outlet; the declared openings are wing roots, horn slots, boom posts and pitot.
- M8 checks current only. The motor runs at 22.97 of 26 A continuous, and the maker states no cooling condition.
- The gear rating's 40 C condition is "(stated design limit)" (src/physics/gears.js:259; params.gearRating
  max_temperature_C 40). It is asserted, not computed.
- Rough in-pod heat at cruise (unsourced estimate): ESC 5% of 71 W, about 3.6 W; gears 15%, about 9 W; motor copper
  and iron about 8 W. That is about 20 W into a sealed pod for 18 min.
- Smallest fix: declare a cooling inlet and exit as openings (their drag counted). State a heat balance, or record the
  thermal condition as unverified beside M7 and M8.

### C4 (sev 3). The CG cannot be set
- The pack has zero travel: the nose skin is 0.733 mm ahead and the airspeed board 0.5 mm behind. The mount search
  already moved it 8 mm aft of its trim x to fit (packing.slides: "battery moved -8 mm").
- There is no ballast provision.
- With D3 corrected, the design CG would need the pack about 55 mm further aft, where the ESC, airspeed board and
  receiver are.
- Smallest fix: re-lay the nose tray (pack aft of the avionics), or state the as-built static margin.

### C5 (sev 3). Receiver: antenna not drawn or massed; "no catalogue dimensions" is wrong
- TBS's page (read this audit) lists "1x TBS Crossfire Micro Receiver Antenna" in the package. It gives the weight as
  0.5 g without saying whether the antenna is included, and the catalogue notes "antenna mass not stated". The antenna
  has no solid, no stated route and no mass.
- The catalogue holds width 18 mm and height 11 mm. The maker's page also prints "Length: 11 mm".
- sizeOf used the two catalogue dimensions and set a 1.36 mm thickness from 0.5 g at FR4 density
  (sims/avis-shared/model.js:707-716). The ledger says "no catalogue dimensions" (completeness.js:147).
- Smallest fix: compare the two published dimensions, draw the antenna and record its mass as unsourced.

### C6 (sev 3). Pitot pressure tubing and probe fittings are put in the margin, outside the exemption
- BOM: "pressure lines in the 15% margin" (build.js:200).
- SPEC Completeness exempts only glue, wire and the margin.
- Matek's ASPD-4525 package includes "Clear silicon tubing 40cm" and its own pitot tube (Matek page). The catalogue's
  3.5 g is "sensor board, pitot tube excluded".
- The probe's annulus seal, end plug and the total-pressure take-off at its aft end are not drawn.
- Smallest fix: a tubing BOM line (two lengths from the probe to the board), drawn, with mass unsourced.

### D6 (sev 3). The tail servo's current is taken above its top published voltage row
- The regulator gives 7.5 V ±4%, so current is taken at 7.8 V. The MD89MW rows stop at 7.4 V.
- stall_A 2.846 A = 2.7 x 7.8 / 7.4 (design.servos.tail; src/design/catalog.js:79-86). The code checks only the servo's
  3.5-8.4 V range text.
- This is conservative for M8. But the round 2 ruling says "No extrapolation outside a servo's published voltage rows",
  and the record does not disclose it.
- The HV6160 rows (4.8-8.2 V) do cover 7.2-7.8 V.
- Smallest fix: disclose it. Alternatively use the Pololu D36V50F6, 6 V, whose 5.76-6.24 V lies inside both servos'
  rows (mass unsourced; it is the same board family). That one needs a refly.

### D7 (sev 3). The pack's own mass tolerance is ±20 g
- Gens Ace store page: "Net Weight(±20g): 143". The catalogue's mass_note also says "verify before relying on it".
- The ±5% mass corners (±87 g) cover it. Disclose it beside M6.

### D8 (sev 4). Cosmetic and record items
- The containment ledger gives the left wing root as "polar -180..180 deg": an angle wrap-around, which reads as the
  whole circumference.
- The fuselage film shell, 0.187 m2, comes from a nominal formula π/2 (W + H) L x 0.8 (build.js:161), not the loft
  (0.176 m2 without the phantom tube). CdA_top and CdA_side are nominal too (build.js:414-415).
- The shell-and-formers lump sits at x -50 mm, while the formers' drawn centroid is at x +52 mm (-0.36 mm CG). These are
  small.

## Checked and found sound (so silence is not a pass)
- Ratings in range:
  - Pololu D36V50F7 input 8.3-50 V against the pack's 9.93-12.6 V, and 5 A continuous against a 2.50 A peak.
  - Tekko32 2-6S, Scorpion SII-3008 max 4S, and F405-WMO 5.4-30 V, all against 3S.
  - Pack 77 A against 15.6 A RMS.
  - FC BEC 2 A against 0.415 A.
  - HV6160 rows cover the supply.
  - Ties within bundle ranges: T18R 1.5-22 mm (loop up to 69 mm, largest used 65.72 mm); T40R 1.5-40 mm (up to
    126 mm, largest used 119 mm); smallest loop 25.1 mm.
- Drawn boxes equal catalogue dimensions for ESC, FC, GPS, pack, regulator and servos.
- Every BOM id exists in the catalogue. None is flagged sold out or end-of-life: the motor is on backorder and the
  regulator "Rationed (Active and Preferred)".
- Lumps for motor, gears and bearings, ESC, FC, GPS, tail servos, receiver and airspeed, encoder, regulator, pitot and
  boom match their drawn parts within about 1 mm.
- Flown tail position (x -0.9458 m, y 0.062 m, z 0.005 m) equals the drawn tail. Wing hinge [0, 0.02, 0.025] m equals
  the drawn shoulder axis (bushings centred y 20 mm, z 25 mm). Semispan 0.975 + 0.025 = 1.0 m equals the drawn tip rib.
- GPS view: only the 10 mm carbon boom, 133 mm above, about 4 deg; balsa formers and film. Clear.
- ESC location: the nose tray, stated and drawn.
- FC package includes the USB-C extender, beeper and pins. They exist, but are not drawn or routed (C1).
- Airspeed board: Matek publishes no dimensions (confirmed). Its 21 x 21 x 4.2 mm box from mass is the best available.


---

Provenance note: the audit read src/ and data/ at a558baf; `git status` shows them clean at the end. During the audit, four film-owned files (sims/avis-shared/bird.js, drive-solids.js, frame.js, solids.js) gained uncommitted edits, timestamped 15:59-16:04, two of them after both forked auditors had finished. They are not from this audit, and no audited file depends on them (round 16 record: nothing under src/ imports a film file).
