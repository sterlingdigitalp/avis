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
