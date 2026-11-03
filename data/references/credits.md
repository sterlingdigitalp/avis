# Credits and licence obligations

Read 2026-11-02. This is a working record, not legal advice.

## Obligations that apply if the data ships

**Copernicus DEM GLO-30** (and MRDEM, which is partly built from it). Any public distribution of data, including rendered or derived terrain, needs the notice:

> produced using Copernicus WorldDEM-30 © DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved

Unmodified data uses the shorter source notice in the licence. The licence grants no right to Copernicus or provider logos. https://documentation.dataspace.copernicus.eu/APIs/SentinelHub/Data/DEM/resources/license/License-COPDEM-30.pdf

**HRDEM and MRDEM, Natural Resources Canada.** Open Government Licence - Canada 2.0. Acknowledge the source and link the licence where possible:

> Contains information licensed under the Open Government Licence – Canada.

https://open.canada.ca/en/open-government-licence-canada

**Alerstam et al. 2007.** CC BY, per the article's copyright statement. Credit: Alerstam T, Rosén M, Bäckman J, Ericson PGP, Hellgren O. *Flight Speeds among Bird Species: Allometric and Phylogenetic Effects.* PLoS Biology 5(8): e197.

## Needs a decision before the repository goes public

SPEC limits assets to CC0, CC-BY, public domain or open government data. These sources do not clearly fall in that list:

| Item in this repo | Source | Status |
|---|---|---|
| `airfoil-e387.json`, `uiuc_lsat_vol5` tables; `airfoils/uiuc-vol5-ag-polars.json` | UIUC Summary of Low-Speed Airfoil Data, Vol. 5 | The volume says "Copyright 2012 ... All rights reserved." The site offers the PDF free. These files hold numeric measurements only, with attribution, and no text or figures. Whether that is acceptable to publish is a call for the operator. |
| `airfoils/*.dat` | UIUC Airfoil Coordinates Database | No licence statement found on the database page. The AG24 file also carries Mark Drela's header comment. |

If either is unacceptable, the `uiuc_lsat_vol5` block and the `.dat` files can stay out of the public push. The NASA tables remain, and the CFD agent can read the UIUC PDF locally.

## Sources not redistributed, cited only

- **NASA TM-4062** (McGhee, Walker, Millard, 1988), a US government work. Tables transcribed. https://ntrs.nasa.gov/api/citations/19890001471/downloads/19890001471.pdf
- **Send et al., ICAS 2012 paper 053**: figures not copied, numbers cited. https://icas.org/ICAS_ARCHIVE/ICAS2012/PAPERS/053.PDF
- **Zhang, Wei, Cai, arXiv:2102.12687**, **Gerdes et al. 2014 (UMD)**, **Chang 2003**, eLife 89968 Supplementary File 3a, Hedenström et al. 1999, Tobalske & Dial 1996: numbers cited.
- **Parks Canada** ecosystem page; **Alberta Natural Regions Committee 2006**; **GeoConvention 2018 abstract**; **1977 Geographica Helvetica paper on Mount Rundle**; **Ben Gadd, Classic Canadian Rockies Geology, 2020**: facts cited. Do not copy Gadd's map into the project.
- **Wikipedia** species and Banff articles: facts cited; CC BY-SA applies if text is reused, so none is reused.
- **MathWorks** Aerospace Blockset documentation: equations cited as a statement of MIL-F-8785C and MIL-HDBK-1797. Military standards themselves were not read.
- **Parts:** manufacturer and retailer pages are listed per part in `data/parts/catalog.json` under `source_url`. Facts only; no images or text copied. Product names are the makers' trade marks, and their use here does not imply endorsement or that any physical bird exists.

Standards and technical references the design (v21) and SPEC rely on, cited only: no text, table or figure is reproduced. Titles and editions as each standards body publishes them, read 2026-11-03; where the repo names no edition, the current one is given and marked so.

- **KHK (Kohara Gear Industry Co., Ltd.), *Gear Technical Reference*** (khkgears.net, web, © 2021), chapters 9 Gear Materials, 10 Gear Strength and Gear Durability and 11 Design of Plastic Gears; and KHK's Japanese *歯車技術資料（web版）* pages khk416_1 (8.3 歯車の歯当たり, Table 8.4) and khk472 (平歯車及びはすば歯車の精度), whose data date from catalogue 3013 (2015). Used for gear rating corrections, the plastic-gear method and the mapping from new to old JIS accuracy grades. Facts cited. https://khkgears.net/new/gear_knowledge/gear_technical_reference/ , https://www.khkgears.co.jp/gear_technology/intermediate_guide/khk416_1/ , https://www.khkgears.co.jp/gear_technology/gear_reference/khk472/
- **JGMA 401-01:1974**, *Bending Strength Formulas for Spur Gears and Helical Gears*, and **JGMA 402-01:1975**, *Surface Durability Formulas for Spur Gears and Helical Gears* (Japan Gear Manufacturers Association): gear bending and surface-durability ratings and their life, speed, load-distribution and reversing-load factors, applied as KHK states them. The JGMA texts were not read.
- **ISO 898-1:2013**, *Mechanical properties of fasteners made of carbon steel and alloy steel — Part 1: Bolts, screws and studs with specified property classes — Coarse thread and fine pitch thread*: fastener proof loads. **ISO 898-2:2022**, *Fasteners — Mechanical properties of fasteners made of carbon steel and alloy steel — Part 2: Nuts with specified property classes*: nut class matched to the bolt. **ISO 898-5:2012**, *… Part 5: Set screws and similar threaded fasteners with specified hardness classes — Coarse thread and fine pitch thread*, and **ISO 3506-1:2020**, *Fasteners — Mechanical properties of corrosion-resistant stainless steel fasteners — Part 1: Bolts, screws and studs with specified grades and property classes*: SPEC's fastener rules. Current editions; the repo names none.
- **ISO 4762:2004**, *Hexagon socket head cap screws*, and **ISO 4032:2023**, *Fasteners — Hexagon regular nuts (style 1)*: M3 screw and nut dimensions for part masses.
- **ISO 76:2006** (with Amd 1:2017), *Rolling bearings — Static load ratings*: static equivalent load P0 and the C0/P0 check.
- **EN 754-2:2016**, *Aluminium and aluminium alloys - Cold drawn rod/bar and tube - Part 2: Mechanical properties*; **EN 755-2:2008**, *Aluminium and aluminium alloys - Extruded rod/bar, tube and profiles - Part 2: Mechanical properties*; **EN 485-2:2016+A1:2018**, *Aluminium and aluminium alloys - Sheet, strip and plate - Part 2: Mechanical properties* (CEN): specified-minimum strengths of 6061-T6 and 7075-T6 bar, extrusion and sheet. Values were read from supplier tables, not from the standards. The first two editions are the ones those tables name, and both have since been superseded (EN 754-2:2024, EN 755-2:2025). EN 485-2 is the current edition; the repo names none.
- **EN 1999-1-1:2007** (+A1:2009, +A2:2013), *Eurocode 9 - Design of aluminium structures - Part 1-1: General structural rules*, and **EN 1993-1-8:2005**, *Eurocode 3: Design of steel structures - Part 1-8: Design of joints* (CEN): bolt and pin bearing resistance k1·αb·fu·d·t/γM2, γM2 = 1.25, and minimum edge distance 1.2 d0. The repo names no edition, but its table numbers follow these editions. They have since been replaced by EN 1999-1-1:2023 and EN 1993-1-8:2024.
- **JIS G 4051:2016**, *機械構造用炭素鋼鋼材 (Carbon steels for machine structural use)*, and **JIS G 4053:2016**, *機械構造用合金鋼鋼材 (Low-alloyed steels for machine structural use)*: S45C and SCM415 gear steels. Neither standard specifies mechanical properties.
- **JIS B 1702-1:1998**, *円筒歯車－精度等級－第1部：歯車の歯面に関する誤差の定義及び許容値*, and the old **JIS B 1702:1976**, *平歯車及びはすば歯車の精度*: catalogue gear accuracy grades, and the old grades that index the JGMA speed factor.
- **JIS B 1301:1996**, *キー及びキー溝 (Keys and their corresponding keyways)*: shaft keyseat depths.
- **ISO 4587:2003**, *Adhesives — Determination of tensile lap-shear strength of rigid-to-rigid bonded assemblies*, and **ISO 13445:2003**, *Adhesives — Determination of shear strength of adhesive bonds between rigid substrates by the block-shear method*: the test methods behind the adhesive strengths quoted from makers' data sheets.
- **ASTM D341-20(2025)**, *Standard Practice for Viscosity-Temperature Equations and Charts for Liquid Petroleum or Hydrocarbon Products*: gear-oil viscosity at 50 °C from the published 40 and 100 °C values.
- **ASTM B348/B348M-25**, *Standard Specification for Titanium and Titanium Alloy Bars and Billets*, and **ASTM B338-17(2026)**, *Standard Specification for Seamless and Welded Titanium and Titanium Alloy Tubes for Condensers and Heat Exchangers*: the Ti-6Al-4V bar minimum yield, and the certification of the Ti-3Al-2.5V tube. Current editions; the repo names none.
- **14 CFR 25.125**, *Landing* (Part 25, Airworthiness Standards: Transport Category Airplanes; current eCFR text, last amended by Amdt. 25-140, 2014), a US government work: the VREF ≥ 1.23 VSR0 factor behind the landing approach speed. https://www.ecfr.gov/current/title-14/chapter-I/subchapter-C/part-25/subpart-B/subject-group-ECFR3c94b04a8a6b8a0/section-25.125
- **MIL-F-8785C**, *Military Specification: Flying Qualities of Piloted Airplanes*, 5 November 1980, and **MIL-HDBK-1797**, *Department of Defense Handbook: Flying Qualities of Piloted Aircraft*, 19 December 1997 (revision 1797B, 2012): the Dryden turbulence model. Both were read only through the MathWorks documentation above, not directly.
- **ISO 2533:1975**, *Standard Atmosphere*, and **ICAO Doc 7488**, *Manual of the ICAO Standard Atmosphere (extended to 80 kilometres (262 500 feet))*, 3rd edition, 1993: atmosphere constants.
- **Katz, J. and Plotkin, A., *Low-Speed Aerodynamics***, 2nd edition, Cambridge University Press, 2001: the vortex-ring method in the unsteady vortex-lattice solver.
- **Standards named only in catalogue entries**, quoted from makers' data sheets: ISO 8734:1997 (*Parallel pins, of hardened steel and martensitic stainless steel (Dowel pins)*), DIN 6325 (withdrawn; edition not stated in the catalogue), ISO 3104:2023, ISO 12185 (edition not stated in the catalogue), ISO 10123:2013 and ASTM B221-21.

## Claim that must travel with this data

"simulated flight, real parts". No physical bird exists. Catalogue parts were read from listings. None were bought, weighed or tested.

## Environment data (decided 2026-11-02)

The Copernicus licence (free, open, attribution required) is accepted under SPEC's rule, on the same footing as CC-BY. Full source list: `env/DATA_SOURCES.md`.

- **NRCan HRDEM 2 m lidar (AB-Upper_Bow_River) and MRDEM-30.** Open Government Licence – Canada: "Contains information licensed under the Open Government Licence – Canada."
- **Copernicus GLO-30**, bare rock outside the lidar only: the WorldDEM-30 notice above.
- **Sentinel-2 L2A, 2022-09-21 and 2023-09-21.** "Contains modified Copernicus Sentinel data 2022, 2023."
- **Poly Haven** textures and HDRI: CC0, credit given voluntarily.

The film's end card and the README carry all four.

**Fitted polar parameters** (`src/physics/polar-fit.js`): 18 fitted coefficients per airfoil derived from measured UIUC data. No measured table is reproduced; the source is credited here and in the file.
