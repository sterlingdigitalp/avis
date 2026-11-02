#!/usr/bin/env python3
"""Download the open source data for the Avis valley. Reproducible and resumable.

    fetch_data.py s2      # Sentinel-2 L2A bands, 2022-09-21, 10 m, whole AOI
    fetch_data.py mrdem   # NRCan MRDEM-30 DTM, whole AOI (30 m)
    fetch_data.py cop30   # Copernicus GLO-30 DSM, whole AOI (30 m), for bare rock
    fetch_data.py hrdem X0 Y0 X1 Y1 RES NAME  # NRCan HRDEM lidar DTM+DSM window in UTM 11N

All grids are EPSG:32611 north-up. Every output goes to its own directory under out/env/raw/.
Sources and licences are recorded in env/DATA_SOURCES.md.
"""
import json
import os
import sys
import urllib.request

os.environ.setdefault("GDAL_DISABLE_READDIR_ON_OPEN", "EMPTY_DIR")
os.environ.setdefault("GDAL_HTTP_MULTIRANGE", "YES")
os.environ.setdefault("GDAL_HTTP_MERGE_CONSECUTIVE_RANGES", "YES")
os.environ.setdefault("AWS_NO_SIGN_REQUEST", "YES")
os.environ.setdefault("VSI_CACHE", "TRUE")
os.environ.setdefault("GDAL_HTTP_MAX_RETRY", "6")
os.environ.setdefault("GDAL_HTTP_RETRY_DELAY", "3")

import numpy as np
import rasterio
import rasterio.transform
from rasterio.crs import CRS
from rasterio.enums import Resampling
from rasterio.vrt import WarpedVRT

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, "out/env/raw")
UTM = CRS.from_epsg(32611)
# Whole-AOI box: 90 km square around the Canmore / Lac des Arcs reach of the Bow Valley.
AOI = (573000, 5615000, 663000, 5705000)
S2_DATE = "2022-09-21"
S2_BANDS = {"blue": 10, "green": 10, "red": 10, "nir": 10, "swir16": 20, "scl": 20}
HRDEM = "https://canelevation-dem.s3.ca-central-1.amazonaws.com/hrdem-mosaic-{res}m/3_3-mosaic-{res}m-{kind}.tif"


def warp_read(url, box, res, nodata=None, resampling=Resampling.bilinear, dtype="float32", chunk=2048):
    """Read url into a UTM grid chunk by chunk."""
    x0, y0, x1, y1 = box
    W, H = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    out = np.full((H, W), np.nan if dtype == "float32" else 0, dtype=dtype)
    with rasterio.open(url) as ds:
        nd = nodata if nodata is not None else ds.nodata
        tr = rasterio.transform.from_origin(x0, y1, res, res)
        with WarpedVRT(ds, crs=UTM, transform=tr, width=W, height=H, resampling=resampling, nodata=nd,
                       warp_mem_limit=2048) as v:
            for r in range(0, H, chunk):
                for c in range(0, W, chunk):
                    win = rasterio.windows.Window(c, r, min(chunk, W - c), min(chunk, H - r))
                    a = v.read(1, window=win, masked=False)
                    if dtype == "float32":
                        a = a.astype("float32")
                        if nd is not None:
                            a[a == nd] = np.nan
                    out[r:r + win.height, c:c + win.width] = a
                print("row", r, "/", H, flush=True)
    return out


def stac(url, body):
    r = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(r, timeout=60))


def fetch_s2():
    d = os.path.join(RAW, "s2")
    os.makedirs(d, exist_ok=True)
    feats = stac("https://earth-search.aws.element84.com/v1/search", {
        "collections": ["sentinel-2-l2a"], "bbox": [-116.4, 50.65, -114.7, 51.5],
        "datetime": f"{S2_DATE}T00:00:00Z/{S2_DATE}T23:59:59Z", "limit": 20})["features"]
    meta = [{"id": f["id"], "cloud": f["properties"]["eo:cloud_cover"],
             "offset": f["properties"].get("earthsearch:boa_offset_applied"),
             "baseline": f["properties"].get("s2:processing_baseline"), "sun": [
                 f["properties"].get("view:sun_azimuth"), f["properties"].get("view:sun_elevation")],
             "datetime": f["properties"]["datetime"]} for f in feats]
    # The 2022 11UNS granule has a corrupted radiometric offset (values wrap at 1000 DN). Skip it; the area it
    # covers west of E 600 km is filled from the clean 2023-09-21 scene in s2_20230921/ at build time.
    feats = [f for f in feats if f["properties"].get("earthsearch:boa_offset_applied")]
    print(json.dumps(meta, indent=0), flush=True)
    json.dump(meta, open(os.path.join(d, "items.json"), "w"), indent=1)
    x0, y0, x1, y1 = AOI
    for band, res in S2_BANDS.items():
        p = os.path.join(d, f"{band}.npy")
        if os.path.exists(p):
            continue
        canvas = None
        for f in feats:
            nearest = band == "scl"
            a = warp_read(f["assets"][band]["href"], AOI, res, nodata=0,
                          resampling=Resampling.nearest if nearest else Resampling.bilinear, dtype="uint16", chunk=4096)
            # Sentinel-2 baseline >= 04.00 stores DN + 1000 unless the catalogue already removed it.
            off = f["properties"].get("earthsearch:boa_offset_applied")
            base = float(f["properties"].get("s2:processing_baseline", "0"))
            if band != "scl" and base >= 4.0 and not off:
                a = np.where(a > 1000, a - 1000, 0).astype("uint16")
            if canvas is None:
                canvas = a
            else:
                m = canvas == 0
                canvas[m] = a[m]
        np.save(p, canvas)
        print("saved", band, canvas.shape, flush=True)


def fetch_mrdem():
    d = os.path.join(RAW, "mrdem")
    os.makedirs(d, exist_ok=True)
    item = stac("https://datacube.services.geo.ca/stac/api/search", {
        "collections": ["mrdem-30"], "bbox": [-116, 51, -115, 51.2], "limit": 2})["features"][0]
    url = item["assets"]["dtm"]["href"]
    print(url, flush=True)
    a = warp_read(url, AOI, 30.0)
    np.save(os.path.join(d, "dtm30.npy"), a)
    json.dump({"url": url, "box": AOI, "res": 30}, open(os.path.join(d, "meta.json"), "w"))
    print("mrdem", a.shape, np.nanmin(a), np.nanmax(a))


def fetch_cop30():
    """Copernicus GLO-30 DSM: sharper than MRDEM on bare rock, used there only (it is a surface model, so not under forest)."""
    d = os.path.join(RAW, "cop30")
    os.makedirs(d, exist_ok=True)
    out = None
    for lat in (50, 51):
        for lon in (117, 116, 115):
            tile = f"Copernicus_DSM_COG_10_N{lat:02d}_00_W{lon:03d}_00_DEM"
            url = f"/vsicurl/https://copernicus-dem-30m.s3.amazonaws.com/{tile}/{tile}.tif"
            try:
                a = warp_read(url, AOI, 30.0, nodata=-32767.0, resampling=Resampling.cubic, chunk=4096)
            except Exception as e:  # noqa: BLE001 - a missing tile is reported and skipped
                print("skip", tile, e, flush=True)
                continue
            out = a if out is None else np.where(np.isfinite(out), out, a)
    np.save(os.path.join(d, "cop30.npy"), out)
    print("cop30", out.shape, "valid", np.isfinite(out).mean(), flush=True)


def fetch_hrdem(box, res, name):
    d = os.path.join(RAW, f"hrdem_{name}")
    os.makedirs(d, exist_ok=True)
    src = 1 if res < 2 else 2
    for kind in ("dtm", "dsm"):
        p = os.path.join(d, f"{kind}.npy")
        if os.path.exists(p):
            continue
        url = "/vsicurl/" + HRDEM.format(res=src, kind=kind)
        a = warp_read(url, box, res, nodata=-32767.0)
        np.save(p, a)
        print(kind, a.shape, np.nanmin(a), np.nanmax(a), "valid", np.isfinite(a).mean(), flush=True)
    json.dump({"box": box, "res": res, "src_res": src, "url": HRDEM}, open(os.path.join(d, "meta.json"), "w"))


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "s2":
        fetch_s2()
    elif cmd == "mrdem":
        fetch_mrdem()
    elif cmd == "cop30":
        fetch_cop30()
    elif cmd == "hrdem":
        box = tuple(float(v) for v in sys.argv[2:6])
        fetch_hrdem(box, float(sys.argv[6]), sys.argv[7])
