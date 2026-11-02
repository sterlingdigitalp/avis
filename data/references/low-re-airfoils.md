# Low-Reynolds airfoil candidates

For film-covered ribbed wings at Re 50k-200k. Coordinates are in `airfoils/<name>.dat`, unchanged from the UIUC Airfoil Coordinates Database (https://m-selig.ae.illinois.edu/ads/coord_database.html), retrieved 2026-11-02.

Thickness and camber were computed here from the coordinates and match UIUC's tables where UIUC lists them (Vol. 1 and Vol. 5 airfoil summary tables). "Lower surface hollow" is computed here too: the largest gap, in chords, between the lower surface and its convex hull. Film spanning between ribs cannot follow a hollow unless it is bonded down. A larger number means more shape error for a film skin. This is geometry, not a measured penalty.

| Airfoil | File | t/c | Camber | Lower-surface hollow (c) | Published polars |
|---|---|---:|---:|---:|---|
| E387 | `e387.dat` | 9.07% | 3.80% | 0.0076 at x/c 0.67 | UIUC Vol. 5 Appendix B, tabulated Re 60k-460k. NASA TM-4062. Full tables in `airfoil-e387.json` |
| SD7037 | `sd7037.dat` | 9.20% | 3.02% | 0.0058 at x/c 0.81 | UIUC Vol. 1 (SD7037 (A) Figs. 4.133-4.134; SD7037 (B) Figs. 4.137-4.138), Vol. 2 and Vol. 3 per the UIUC tested-airfoil list |
| SD7003 | `sd7003.dat` | 8.50% | 1.46% | 0.0038 at x/c 0.85 | UIUC Vol. 1, Figs. 4.121-4.122. Listed down to Re 60k |
| AG24 | `ag24.dat` | 8.41% | 2.22% | 0.0072 at x/c 0.79 | UIUC Vol. 5 Appendix B, tabulated Re 60k-400k. Numeric copy in `airfoils/uiuc-vol5-ag-polars.json` |
| S5010 | `s5010.dat` | 9.82% | 2.20% | 0.0001 (lower surface effectively convex) | UIUC Vol. 2 per the tested-airfoil list. **Polar pages not read** |
| MH32 | `mh32.dat` | 8.70% | 2.40% | 0.0029 at x/c 0.81 | UIUC Vol. 2 per the tested-airfoil list. **Polar pages not read** |

Sources: UIUC volumes at https://m-selig.ae.illinois.edu/uiuc_lsat.html and tested-airfoil list at https://m-selig.ae.illinois.edu/uiuc_lsat_airfoilsTested.html.

## Reading these for Avis

- **E387 is the validation anchor.** V7 is run on it, so it has the strongest evidence behind it. It also has the largest hollow here, and its Re 100k drag carries a strong laminar-bubble penalty (see `airfoil-e387.json`).
- **S5010 and MH32 suit film best by geometry** because their lower surfaces are nearly convex. Vol. 2 has not been read for them, so performance at Re 50k-100k is unconfirmed here.
- **AG24 has the only numeric polars besides E387 in this repository.** It is a discus-launch section; its numeric table is the easy one for the CFD agent to compare against.
- **SD7003** has the lowest camber here. UIUC's Vol. 1 discussion ties it to F3B sailplanes.
- All of these are measured on smooth rigid models. None was tested as film over ribs. No published polar was found for a film-over-rib version of any candidate. The rib-bay shape error and film sag are not in these data.

## SmartBird section, for V4

Send et al. (ICAS 2012, Fig. 12 and Fig. 11) compare SmartBird's wing section with NACA 7412, which they applied during design. The NACA 7412 was not on the UIUC database at the expected file name (`naca7412.dat` returned 404). NACA 4-digit coordinates can be generated from the standard equations by the physics or CFD workstream, and that file should not be presented as a UIUC or Festo file. The actual SmartBird section coordinates were not found.
