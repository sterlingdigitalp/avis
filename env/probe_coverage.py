"""Coverage + hillshade map of the HRDEM mosaic around the Bow Valley, and a canopy-height preview. Read-only probe."""
import sys

import numpy as np
import rasterio
import rasterio.transform
from PIL import Image, ImageDraw
from rasterio.crs import CRS
from rasterio.enums import Resampling
from rasterio.vrt import WarpedVRT
from rasterio.windows import from_bounds

URL = "/vsicurl/https://canelevation-dem.s3.ca-central-1.amazonaws.com/hrdem-mosaic-2m/3_3-mosaic-2m-{}.tif"
OUT = sys.argv[1]
UTM = CRS.from_epsg(32611)
# UTM 11N window around Canmore/Banff
x0, x1, y0, y1 = 590000, 650000, 5645000, 5690000
res = 50.0
w, h = int((x1 - x0) / res), int((y1 - y0) / res)


def read(kind):
    with rasterio.open(URL.format(kind)) as ds:
        tr = rasterio.transform.from_origin(x0, y1, res, res)
        with WarpedVRT(ds, crs=UTM, transform=tr, width=w, height=h, resampling=Resampling.bilinear, nodata=-32767) as v:
            return v.read(1, masked=False)


dtm = read("dtm").astype(float)
valid = dtm > 0
dx = np.gradient(np.where(valid, dtm, 1500), res, axis=1)
dy = np.gradient(np.where(valid, dtm, 1500), res, axis=0)
sh = np.clip((-dx * 0.5 + dy * 0.5 + 0.7) / np.sqrt(1 + dx * dx + dy * dy), 0, 1)
img = np.zeros((h, w, 3), np.uint8)
img[..., 0] = np.where(valid, sh * 255, 40)
img[..., 1] = np.where(valid, sh * 255, 0)
img[..., 2] = np.where(valid, sh * 255, 0)
im = Image.fromarray(img)
d = ImageDraw.Draw(im)
for gx in range(x0, x1, 5000):
    d.line([((gx - x0) / res, 0), ((gx - x0) / res, h)], fill=(0, 120, 255), width=1)
    d.text(((gx - x0) / res + 2, 2), str(gx // 1000), fill=(0, 255, 255))
for gy in range(y0, y1, 5000):
    d.line([(0, (y1 - gy) / res), (w, (y1 - gy) / res)], fill=(0, 120, 255), width=1)
    d.text((2, (y1 - gy) / res + 2), str(gy // 1000), fill=(0, 255, 255))
im.save(OUT)
print("valid frac", valid.mean(), "shape", dtm.shape)
