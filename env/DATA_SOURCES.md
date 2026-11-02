# Environment data sources and licences

The valley is the real Bow Valley at the north-east foot of Mount Rundle, Alberta: the Bow River reach west of Canmore,
just outside the east gate of Banff National Park. Corridor origin UTM 11N (EPSG:32611) E 610000, N 5665000, about 51.123 N, 115.43 W.

| Data | Use | Source | Licence / credit |
|---|---|---|---|
| NRCan HRDEM mosaic 2 m, DTM and DSM (source lidar "AB-Upper_Bow_River") | Near terrain at 2 m and 8 m, canopy height, real tree tops, corridor clearance | `https://canelevation-dem.s3.ca-central-1.amazonaws.com/hrdem-mosaic-2m/`, via STAC `https://datacube.services.geo.ca/stac/api/` | Open Government Licence - Canada. Contains information licensed under the Open Government Licence - Canada |
| NRCan MRDEM-30 DTM | Far terrain to 45 km | STAC collection `mrdem-30` as above | Open Government Licence - Canada |
| Copernicus DEM GLO-30 (DSM), tiles N50-N51 W115-W117 | Bare-rock terrain outside lidar, where it is sharper than MRDEM; blended out under vegetation | `copernicus-dem-30m.s3.amazonaws.com` | Copernicus DEM, free licence with attribution: "produced using Copernicus WorldDEM-30 (c) DLR e.V. 2010-2014 and (c) Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved". Please confirm this licence is acceptable under SPEC's licence rule |
| Sentinel-2 L2A, 2022-09-21 (S2A_11UPS_20220921_0_L2A), and 2023-09-21 (S2B_11UNS_20230921_0_L2A) west of E 600 km | Terrain colour, snow, vegetation and water masks | Element 84 Earth Search STAC, AWS `sentinel-cogs` | Copernicus Sentinel data: free and open. Credit: "Contains modified Copernicus Sentinel data 2022, 2023" |
| Poly Haven textures: `pine_bark`, `bark_brown_02`, `tree_bark_03`, `knotted_pine_bark`, `cliff_side`, `forest_ground_04`, `gravel_stones`, `aerial_grass_rock` | Bark, near rock, forest floor, gravel and boulders, meadow ground | `https://polyhaven.com`, API `api.polyhaven.com` | CC0 |
| Poly Haven HDRI `evening_road_01_puresky` | Sky dome, rotated so its sun matches the scene sun | as above | CC0 |

Solar position is computed with the NOAA low-precision formulae (`env/solar.py`).

## Honest list of what is not real

- Tree tops inside the lidar window: dominant stems are real local maxima of DSM minus DTM, at their measured height. Co-dominant stems are infilled at random on a jittered lattice where the lidar shows canopy, at a fraction of the measured canopy height. Species are assigned statistically from elevation, aspect and stand noise, not surveyed.
- Outside the lidar: stems are scattered from Sentinel-2 vegetation cover. Rock detail below 30 m there is synthetic.
- Canmore town, Highway 1 and the rail line are removed. Their pixels are replaced with neighbouring vegetation colour. Buildings are not modelled.
- The Sentinel 11UNS granule for 2022-09-21 has corrupt radiometry (values wrap at 1000 DN) and is not used.
- Colours for rock and foliage are artistic choices guided by the notes in `data/references/rockies.md`, apart from the terrain base colour, which is measured satellite reflectance.
