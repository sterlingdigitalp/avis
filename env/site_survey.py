"""Site survey sheet: Sentinel-2 true colour, lidar canopy height and hillshade over the lidar band. Usage: site_survey.py OUTDIR"""
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW = os.path.join(ROOT, "out/env/raw")
out = sys.argv[1]
AOI = (573000, 5615000, 663000, 5705000)
box = (604000, 5650000, 636000, 5674000)
# Sentinel crop at 10 m -> show at 10 m
x0, y0, x1, y1 = box
c0, r0 = (x0 - AOI[0]) // 10, (AOI[3] - y1) // 10
c1, r1 = (x1 - AOI[0]) // 10, (AOI[3] - y0) // 10
rgb = np.stack([np.load(f"{RAW}/s2/{b}.npy", mmap_mode="r")[r0:r1, c0:c1].astype(np.float32) for b in ("red", "green", "blue")], -1) / 10000.0
rgb8 = (np.clip(rgb * 3.2, 0, 1) ** (1 / 2.2) * 255).astype(np.uint8)
dtm = np.load(f"{RAW}/hrdem_survey/dtm.npy")
dsm = np.load(f"{RAW}/hrdem_survey/dsm.npy")
chm = dsm - dtm
d0 = np.nan_to_num(dtm, nan=1500)
gx, gy = np.gradient(d0, 8.0)
hs = np.clip((gy * 0.6 + gx * 0.6 + 0.8) / np.sqrt(1 + gx * gx + gy * gy), 0, 1)
c8 = np.zeros(dtm.shape + (3,), np.uint8)
c8[..., 0] = (hs * 180).astype(np.uint8)
c8[..., 1] = (np.clip(np.nan_to_num(chm) / 30, 0, 1) ** 0.6 * 255).astype(np.uint8)
c8[..., 2] = (hs * 120).astype(np.uint8)


def annotate(im, res):
    d = ImageDraw.Draw(im)
    for gx_ in range(x0, x1, 2000):
        d.line([((gx_ - x0) / res, 0), ((gx_ - x0) / res, 14)], fill=(255, 255, 0))
        d.text(((gx_ - x0) / res + 2, 14), str(gx_ // 1000), fill=(255, 255, 0))
    for gy_ in range(y0, y1, 2000):
        d.line([(0, (y1 - gy_) / res), (14, (y1 - gy_) / res)], fill=(255, 255, 0))
        d.text((16, (y1 - gy_) / res), str(gy_ // 1000), fill=(255, 255, 0))
    return im


annotate(Image.fromarray(rgb8), 10).resize((1600, 1200), Image.LANCZOS).save(f"{out}/survey_s2.png")
annotate(Image.fromarray(c8), 8).resize((1600, 1200), Image.LANCZOS).save(f"{out}/survey_chm.png")
print("ok", chm.shape, np.nanpercentile(chm, [50, 90, 99]))
