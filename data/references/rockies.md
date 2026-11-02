# Bow Valley, Banff: environment reference

For the `environment` workstream. Read 2026-11-02. "Computed" means calculated here and not published.

## Elevation and location

- Banff climate station: 51.183 N, 115.567 W, elevation 1,383.7 m. Read in the Environment Canada climate table reproduced at https://en.wikipedia.org/wiki/Banff,_Alberta (climate ID 3050520). Environment Canada's own normals page loaded, but its station value could not be confirmed in the text read.
- The design condition in SPEC, 1,800 m, is about 415 m above that valley-floor station (computed). That puts the bird in lower subalpine air, not at valley-floor height.

## Vegetation zones

From Parks Canada, Banff National Park, "Ecosystems and habitat": https://parks.canada.ca/pn-np/ab/banff/nature/environnement-environment/ecosystemes-ecosystems

| Zone | Share of park | Elevation stated | Vegetation stated |
|---|---:|---|---|
| Montane | 3% | 1350-1500 m on north-facing slopes, up to 1650 m on steep south-facing slopes | Douglas-fir, trembling aspen, lodgepole pine, with grassland patches on dry sites. White spruce, balsam poplar and shrub meadows on wetter sites |
| Lower subalpine | 27% | Up to about 2000 m | Dense lodgepole pine, Engelmann spruce, subalpine fir |
| Upper subalpine | 26% | From about 2000 m | Engelmann spruce and subalpine fir, with dwarf-shrub meadows and avalanche path communities |
| Alpine | 44% | Above treeline | About 6% of the park is alpine meadow and shrub. 38% is unvegetated rock, talus, moraine, snow, ice and water |

## Treeline

No single official treeline figure for the Bow Valley was found. Three sources bracket it:

- About 2,300 m for the subalpine to alpine transition, described as gradual. https://thecanadianrockies.com/?p=5836 (a guide site).
- Subalpine range 1300-2300 m and Alpine range 1900-3650 m, province-wide. Alberta Natural Regions Committee (Downing & Pettapiece, eds.), *Natural Regions and Subregions of Alberta*, 2006. https://albertaparks.ca/media/2942026/nrsrcomplete_may_06.pdf
- That report says the lower Alpine boundary falls about 0.5 m per km going north, so the province-wide minimum of 1900 m is a northern value.

Note that the report's Subalpine page prints "Average elevation: 2350 m (range 1300-2300 m)", and the average falls outside the range, so the average looks like a misprint. Treeline should be modelled as a gradual band around 2,300 m and varied by aspect, not as a hard line.

Near treeline, the same report describes open stands of Engelmann spruce with subalpine larch and whitebark pine, and krummholz islands at the upper limit.

## Tree form and height

Heights are from Wikipedia species articles (read). No forestry source was read: the USDA Silvics of North America site timed out. Treat these as general species values, not Banff stand measurements. Mountain stands will be shorter, and near treeline much shorter.

| Species | Height stated | Form |
|---|---|---|
| Engelmann spruce (*Picea engelmannii*) | 25-40 m, exceptionally 65 m | Medium-sized to large evergreen conifer. https://en.wikipedia.org/wiki/Picea_engelmannii |
| Subalpine fir (*Abies lasiocarpa*) | 20 m, exceptionally 40-50 m | Very narrow conic crown. Commonly at and just below treeline. https://en.wikipedia.org/wiki/Abies_lasiocarpa |
| Lodgepole pine (*Pinus contorta*) | Tall subspecies grow up to 40-50 m; a krummholz shrub form is 1-3 m | Thin and narrow-crowned. https://en.wikipedia.org/wiki/Pinus_contorta |
| Subalpine larch (*Larix lyallii*) | 10-25 m, shorter at higher elevation | Sparse conical crown, horizontal branches. https://en.wikipedia.org/wiki/Larix_lyallii |
| Whitebark pine (*Pinus albicaulis*) | Height not captured | Often marks treeline, often krummholz. https://en.wikipedia.org/wiki/Pinus_albicaulis |

Lodgepole form is partly a fire legacy: the Alberta report says the short fire return interval favours young to mid-seral lodgepole pine stands at low to middle elevations.

## Understorey

From the Alberta report's Subalpine section:

- South-west-facing slopes at low to middle elevations: lodgepole pine over bearberry and hairy wild rye.
- Mesic sites: lodgepole, Engelmann spruce and subalpine fir over false azalea, grouseberry, Canada buffaloberry, low bilberry, hairy wild rye, pine reed grass and feathermosses.
- Upper elevations in the south-central mountains: mature Engelmann spruce and subalpine fir over heather, mountain-heather, willow, grouseberry and feathermosses.
- Alpine dry sites: white mountain avens, with only lichens on the driest, coldest and most exposed rock.

## Rock type and colour

- Mount Rundle's stack, from the bottom: Palliser, Banff, Rundle (Fig. 1 of a 1977 paper in *Geographica Helvetica*: https://gh.copernicus.org/articles/32/219/1977/gh-32-219-1977.pdf, in German). That paper assigns older ages than later sources; age should not be taken from it.
- **Colour, from a 2018 GeoConvention abstract:** the Banff Formation weathers "beige to light brown" and forms relatively gentle slopes, compared with the underlying "dark grey, cliff-forming Palliser Formation" and the overlying "light grey Rundle". https://geoconvention.com/wp-content/uploads/abstracts/2018/219_GC2018_Rocks_Organics_Strength_Relief_Banff_Formation.pdf
- **Lithology, from Ben Gadd's map *Classic Canadian Rockies Geology* (2020):** https://www.bengadd.com/wp-content/uploads/2020/12/Classic-Canadian-Rockies-geology-map-and-section-2020-92-x-46.pdf (read for text only; the map must not be copied into the project without permission)
  - Livingstone Formation, Rundle Group: 310 m, limestone and dolostone.
  - Mount Head Formation, Rundle Group: 180 m, limestone and dolostone.
  - Etherington Formation, Rundle Group: 80 m, limestone, dolostone, shale and siltstone.
  - Sulphur Mountain Formation: 380 m, siltstone, mudstone and shale.

For rendering, this supports banded strata of dark grey cliffs, tan recessive slopes and light grey caps. No measured colour values were found, so any RGB is an artistic choice and must be described as such.

## Seasonal light

Computed from solar geometry at latitude 51.18 N, with solar declination of plus or minus 23.44 degrees and a -0.833 degree horizon. Local terrain blocks much of the low sun in a valley. timeanddate.com returned 403, so no published table was read for comparison.

| Date | Noon sun elevation | Day length, flat horizon |
|---|---:|---:|
| June solstice | 62.3 deg | 16.6 h |
| Equinox | 38.8 deg | 12.2 h |
| December solstice | 15.4 deg | 7.9 h |

The Alberta report gives about 470 mm of average annual precipitation at Banff and about 600 mm at Lake Louise.

## Elevation data

All three endpoints were checked live on 2026-11-02.

| Dataset | Resolution | Exact URL | Check | Licence |
|---|---|---|---|---|
| HRDEM, lidar project `AB-Upper_Bow_River-1m` | 1 m | https://canelevation-dem.s3.ca-central-1.amazonaws.com/hrdem-lidar/AB-Upper_Bow_River-1m-dtm.tif and `-dsm.tif` | STAC record read | OGL-Canada 2.0 |
| HRDEM, lidar project `BC-BC_East_Kootenay_UTM11_2015-1m` | 1 m | https://canelevation-dem.s3.ca-central-1.amazonaws.com/hrdem-lidar/BC-BC_East_Kootenay_UTM11_2015-1m-dtm.tif and `-dsm.tif` | STAC record read | OGL-Canada 2.0 |
| HRDEM mosaics | 1 m, 2 m | `s3://canelevation-dem/hrdem-mosaic-1m/`, `hrdem-mosaic-2m/` (anonymous listing works) | Listed | OGL-Canada 2.0 |
| MRDEM-30 DTM (replaces CDEM) | 30 m | https://canelevation-dem.s3.ca-central-1.amazonaws.com/mrdem-30/mrdem-30-dtm.tif | HTTP 200, 83.9 GB, cloud-optimised: read a window, do not download it whole | OGL-Canada 2.0, plus the Copernicus notice below |
| Copernicus GLO-30 tile N51 W116 | 30 m | https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_N51_00_W116_00_DEM/Copernicus_DSM_COG_10_N51_00_W116_00_DEM.tif | HTTP 200, 28.0 MB | Copernicus WorldDEM-30 licence |

Finding tiles: STAC API https://datacube.services.geo.ca/stac/api/ with collections `hrdem-lidar`, `hrdem-mosaic-1m`, `hrdem-mosaic-2m`, `mrdem-30`. The search used was `search?collections=hrdem-lidar&bbox=-115.7,51.1,-115.4,51.3`, which returned the two projects above.

**Coverage warning.** The `AB-Upper_Bow_River` bounding box runs from 115.469 W to 114.249 W, 51.021 N to 51.292 N. The Banff station at 115.567 W falls **west of that box**. The East Kootenay box does span it (117.568 W to 114.747 W), but that project is named for British Columbia and may have no data on the Alberta side of the divide. Each project has a `coverage` asset. Check it before relying on 1 m data at the corridor. GLO-30 and MRDEM cover everything.

Other pages:

- HRDEM: https://open.canada.ca/data/en/dataset/957782bf-847c-4644-a757-e383c0057995, with a direct download directory at https://download-telecharger.services.geo.ca/pub/elevation/dem_mne/highresolution_hauteresolution/dtm_mnt/
- MRDEM: https://open.canada.ca/data/en/dataset/18752265-bda3-498c-a4ba-9dfe68cb98da. It warns that the files are designed for streaming, not downloading.
- CDEM: https://open.canada.ca/data/en/dataset/7f245e4d-76c2-4caa-951a-45d1d2051333. Marked as a legacy product that is no longer supported. Files at https://ftp.maps.canada.ca/pub/nrcan_rncan/elevation/cdem_mnec/. Prefer MRDEM.
- AWS registry entries: https://registry.opendata.aws/canelevation-dem/ and https://registry.opendata.aws/copernicus-dem/

Vertical datums differ. HRDEM uses CGVD2013. GLO-30 uses EGM2008, per the MRDEM page. Do not mix them without a datum shift. GLO-30's file name marks it as a surface model (DSM), so forest canopy may be in the terrain. HRDEM and MRDEM publish a separate `dtm`.

## Licence confirmation

- **Open Government Licence - Canada 2.0**, read at https://open.canada.ca/en/open-government-licence-canada. Worldwide, royalty-free, perpetual, non-exclusive, including commercial use. You must acknowledge the source, using the provider's attribution statement or, failing that, "Contains information licensed under the Open Government Licence – Canada."
- **Copernicus WorldDEM-30 licence**, read at https://documentation.dataspace.copernicus.eu/APIs/SentinelHub/Data/DEM/resources/license/License-COPDEM-30.pdf. Free of charge. Distribution to the public requires this source notice: "© DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved". Adapted or modified data needs the "produced using Copernicus WorldDEM-30 ©..." notice. The licence grants no right to Copernicus or provider logos. This copy is the one hosted by the Copernicus Data Space documentation; the ESA spacedata page was not separately read.
- **MRDEM** is partly built from GLO-30 and carries the same Copernicus notice on its dataset page.
