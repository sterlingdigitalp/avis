#!/usr/bin/env python3
"""Before/after contact sheet of the film's flight shots.

    python3 env/review_sheet.py [before=out/env/review/pass3/before] [after=out/env/review/pass3/after]
                                [out=out/env/review/pass3/sheet.png] [w=760]

One row per film segment (matched on the segment id that prefixes each still, see env/review_shots.py), before on the
left, after on the right, labelled with the segment, the shot rig and the log frame of each still.
"""
import glob
import os
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ORDER = ["wide", "track", "lead", "under", "treetop", "topdown", "ridge", "river"]


def font(size):
    for f in ("/System/Library/Fonts/SFNS.ttf", "/System/Library/Fonts/Helvetica.ttc", "/Library/Fonts/Arial.ttf"):
        if os.path.exists(f):
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def first(folder, sid):
    hits = sorted(glob.glob(os.path.join(folder, sid + "_*.png")))
    return hits[0] if hits else None


def label(path):
    """'track_track_side_long_01566.png' -> 'track_side_long  frame 1566'."""
    stem = os.path.basename(path)[:-4]
    sid, rest = stem.split("_", 1)
    shot, frame = rest.rsplit("_", 1)
    return "%s   frame %d" % (shot, int(frame))


def main():
    kw = dict(before="out/env/review/pass3/before", after="out/env/review/pass3/after", out="out/env/review/pass3/sheet.png", w="760")
    kw.update(a.split("=", 1) for a in sys.argv[1:])
    w = int(kw["w"])
    h = round(w * 9 / 16)
    pad, head, cap = 16, 64, 30
    rows = [s for s in ORDER if first(os.path.join(ROOT, kw["before"]), s) or first(os.path.join(ROOT, kw["after"]), s)]
    sheet = Image.new("RGB", (2 * w + 3 * pad, head + len(rows) * (h + cap + pad) + pad), (14, 15, 17))
    d = ImageDraw.Draw(sheet)
    big, small = font(26), font(17)
    d.text((pad, 18), "Environment pass 2 (before)", font=big, fill=(200, 200, 200))
    d.text((2 * pad + w, 18), "Environment pass 3 (after)", font=big, fill=(200, 200, 200))
    y = head
    for sid in rows:
        for col, key in enumerate(("before", "after")):
            x = pad + col * (w + pad)
            p = first(os.path.join(ROOT, kw[key]), sid)
            if p:
                sheet.paste(Image.open(p).convert("RGB").resize((w, h), Image.LANCZOS), (x, y + cap))
                d.text((x, y + 5), "%s  ·  %s" % (sid, label(p)), font=small, fill=(170, 170, 170))
            else:
                d.text((x, y + cap + h // 2), "%s: no still" % sid, font=small, fill=(170, 90, 90))
        y += h + cap + pad
    out = os.path.join(ROOT, kw["out"])
    sheet.save(out, optimize=True)
    print(out, sheet.size)


if __name__ == "__main__":
    main()
