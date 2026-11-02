"""Low-res Sentinel-2 true-colour previews over the site for scene selection. Usage: s2_preview.py OUTDIR DATE [DATE...]"""
import json
import sys
import urllib.request

import numpy as np
import rasterio
import rasterio.transform
from PIL import Image, ImageDraw
from rasterio.crs import CRS
from rasterio.enums import Resampling
from rasterio.vrt import WarpedVRT

UTM = CRS.from_epsg(32611)
x0, x1, y0, y1 = 570000, 660000, 5620000, 5700000
res = 40.0
W, H = int((x1 - x0) / res), int((y1 - y0) / res)


def items(date):
    body = json.dumps({"collections": ["sentinel-2-l2a"], "bbox": [-115.9, 50.8, -114.9, 51.4],
                       "datetime": f"{date}T00:00:00Z/{date}T23:59:59Z", "limit": 20}).encode()
    r = urllib.request.Request("https://earth-search.aws.element84.com/v1/search", data=body, headers={"Content-Type": "application/json"})
    return json.load(urllib.request.urlopen(r, timeout=30))["features"]


for date in sys.argv[2:]:
    canvas = np.zeros((H, W, 3), np.uint8)
    feats = items(date)
    print(date, [f["id"] for f in feats])
    for f in feats:
        tr = rasterio.transform.from_origin(x0, y1, res, res)
        with rasterio.open(f["assets"]["visual"]["href"]) as ds:
            with WarpedVRT(ds, crs=UTM, transform=tr, width=W, height=H, resampling=Resampling.average, nodata=0) as v:
                a = v.read(masked=False)
        a = np.moveaxis(a, 0, 2)
        m = a.sum(axis=2) > 0
        canvas[m] = a[m]
    canvas = np.clip((canvas / 255.0) ** 0.8 * 255 * 1.4, 0, 255).astype(np.uint8)
    im = Image.fromarray(canvas)
    d = ImageDraw.Draw(im)
    for gx in range(x0, x1, 10000):
        d.line([((gx - x0) / res, 0), ((gx - x0) / res, 8)], fill=(255, 255, 0))
        d.text(((gx - x0) / res + 2, 8), str(gx // 1000), fill=(255, 255, 0))
    for gy in range(y0, y1, 10000):
        d.line([(0, (y1 - gy) / res), (8, (y1 - gy) / res)], fill=(255, 255, 0))
        d.text((10, (y1 - gy) / res), str(gy // 1000), fill=(255, 255, 0))
    im.save(f"{sys.argv[1]}/s2_preview_{date}.png")
