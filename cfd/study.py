#!/usr/bin/env python3
"""Run an explicit list of 2D points.  study.py <group> <coords> <level> <re,...> <alpha,...> [tu] """
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pipeline


def main():
    group, coords, level = sys.argv[1], sys.argv[2], float(sys.argv[3])
    res = [int(v) for v in sys.argv[4].split(",")]
    alphas = [float(v) for v in sys.argv[5].split(",")]
    tu = float(sys.argv[6]) if len(sys.argv) > 6 else 0.10
    stem = os.path.splitext(os.path.basename(coords))[0]
    mesh = pipeline.build_mesh(coords, f"{stem}-L{level:g}", level)
    pool = pipeline.Pool(2)
    for r in res:
        for a in alphas:
            pool.submit(group, f"{stem}-L{level:g}-Re{r}-a{a:g}-Tu{tu:g}", mesh, re_=r, alpha=a, tu=tu)
    pool.wait()


if __name__ == "__main__":
    main()
