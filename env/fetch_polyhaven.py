#!/usr/bin/env python3
"""Fetch CC0 Poly Haven textures and HDRIs to out/env/raw/polyhaven/<id>/ (each asset its own directory).

    fetch_polyhaven.py [--list-skies]
"""
import json
import os
import sys
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "out/env/raw/polyhaven")
UA = {"User-Agent": "avis-env/1.0"}
TEX = {
    # id: resolution
    "pine_bark": "2k", "bark_brown_02": "2k", "tree_bark_03": "2k", "knotted_pine_bark": "2k",
    "cliff_side": "4k", "rock_face": "2k", "quarry_wall": "2k", "aerial_rocks_02": "4k", "rock_boulder_dry": "2k",
    "lichen_rock": "2k", "gray_rocks": "2k",
    "forest_ground_04": "2k", "rocky_gravel": "2k", "gravel_stones": "2k", "aerial_grass_rock": "4k", "snow_02": "2k", "snow_field_aerial": "4k",
    "brown_mud_leaves_01": "2k", "dry_riverbed_rock": "2k",
    # second pass: grey scaly spruce and pine bark, smooth blistered fir bark, grey limestone, river cobble, talus
    "bark_willow_02": "2k", "chinese_hackberry_bark": "2k", "marble_cliff_05": "4k", "ganges_river_pebbles": "2k",
    "pebble_ground_01": "2k", "rocks_ground_02": "2k", "dry_river_pebbles": "2k",
}
MAPS = {"Diffuse": "diff", "nor_gl": "nor", "Rough": "rough", "Displacement": "disp"}
HDRI = ["kloppenheim_06_puresky", "qwantani_dusk_2_puresky", "belfast_sunset_puresky", "kiara_1_dawn"]


def get(url):
    return json.load(urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=60))


def down(url, path):
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=300).read()
    with open(path + ".part", "wb") as f:
        f.write(data)
    os.replace(path + ".part", path)
    print(os.path.basename(path), len(data) // 1024, "KB", flush=True)


def main():
    if "--list-skies" in sys.argv:
        d = get("https://api.polyhaven.com/assets?t=hdris&categories=pure%20skies")
        print(" ".join(sorted(d)))
        return
    for tid, res in TEX.items():
        try:
            f = get(f"https://api.polyhaven.com/files/{tid}")
        except Exception as e:
            print("skip", tid, e)
            continue
        for key, short in MAPS.items():
            if key not in f or res not in f[key]:
                continue
            entry = f[key][res].get("jpg") or f[key][res].get("png")
            ext = "jpg" if "jpg" in f[key][res] else "png"
            down(entry["url"], os.path.join(OUT, tid, f"{short}.{ext}"))
    ids = sys.argv[1:] or HDRI
    for hid in ids:
        f = get(f"https://api.polyhaven.com/files/{hid}")
        e = f["hdri"]["4k"]["exr"]
        down(e["url"], os.path.join(OUT, hid, "sky.exr"))


main()
