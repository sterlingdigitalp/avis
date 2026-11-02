"""Contact sheet of direct sunlight on the real terrain through a day. Usage: sun_survey.py OUT.png"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from PIL import Image, ImageDraw
from solar import solar_az_el

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AOI = (573000, 5615000, 663000, 5705000)
z = np.load(f"{ROOT}/out/env/raw/mrdem/dtm30.npy")
RES = 30.0
box = (604000, 5650000, 628000, 5672000)
c0, r0 = (box[0] - AOI[0]) // 30, (AOI[3] - box[3]) // 30
c1, r1 = (box[2] - AOI[0]) // 30, (AOI[3] - box[1]) // 30


def shadow(az, el):
    a = np.radians(az)
    dx, dy = np.sin(a), np.cos(a)
    tane = np.tan(np.radians(el))
    H, W = r1 - r0, c1 - c0
    yy, xx = np.mgrid[r0:r1, c0:c1]
    lit = np.ones((H, W), bool)
    base = z[r0:r1, c0:c1]
    for d in np.arange(45.0, 22000.0, 45.0):
        rr = (yy - dy * d / RES).astype(int).clip(0, z.shape[0] - 1)
        cc = (xx + dx * d / RES).astype(int).clip(0, z.shape[1] - 1)
        lit &= z[rr, cc] < base + d * tane + 2
    gy, gx = np.gradient(base, RES)
    n = np.dstack([-gx, gy, np.ones_like(gx)])  # x east, y north (row grows south)
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    s = np.array([np.cos(np.radians(el)) * dx, np.cos(np.radians(el)) * dy, np.sin(np.radians(el))])
    cosi = np.clip((n * s).sum(2), 0, 1)
    return lit * cosi


lat, lon = 51.09, -115.37
tiles = []
for mdt in (7.75, 8.25, 8.75, 9.5, 16.5, 17.25, 17.75, 18.25):
    utc = mdt + 6
    az, el = solar_az_el(lat, lon, 2022, 9, 21, utc)
    if el < 1:
        continue
    s = shadow(az, el)
    im = np.clip(s ** 0.6 * 255 + 25, 0, 255).astype(np.uint8)
    pil = Image.fromarray(im).resize((480, 440)).convert("RGB")
    d = ImageDraw.Draw(pil)
    d.text((4, 4), f"{mdt:.2f} MDT az {az:.0f} el {el:.1f}", fill=(255, 255, 0))
    for gx_ in range(606000, 628000, 4000):
        d.text(((gx_ - box[0]) / 50, 16), str(gx_ // 1000), fill=(0, 255, 255))
        d.line([((gx_ - box[0]) / 50, 26), ((gx_ - box[0]) / 50, 34)], fill=(0, 255, 255))
    for gy_ in range(5652000, 5672000, 4000):
        d.text((2, (box[3] - gy_) / 50), str(gy_ // 1000), fill=(0, 255, 255))
    tiles.append(pil)
W = 4
H = (len(tiles) + W - 1) // W
sheet = Image.new("RGB", (W * 480, H * 440))
for i, t in enumerate(tiles):
    sheet.paste(t, ((i % W) * 480, (i // W) * 440))
sheet.save(sys.argv[1])
print("ok")
