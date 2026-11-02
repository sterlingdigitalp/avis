#!/usr/bin/env python3
"""Sections with no published coordinates: built parametrically from the physics module's descriptions.

film-rib-4: src/physics/polars.js describes taut film on 4% camber ribs with a closed foam leading edge. The rib
profile below is this workstream's interpretation of that sentence, not a measured or catalogue shape:
NACA 4-digit mean line (f=0.04, p=0.5, the kinematics.js form), 2.5% thick foam nose with an elliptic leading edge
over the first 5% of chord, thick to 25%, tapering linearly to a sharp trailing edge.

thin-plate: 2% thick flat plate with elliptic nose and tail over the first and last 5% (the tail section).

    sections.py <outDir>
"""
import math
import os
import sys


def camber(x, f, p):
    if f == 0:
        return 0.0
    return f / p ** 2 * (2 * p * x - x * x) if x < p else f / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x * x)


def half_film(x, t=0.025, nose=0.05, hold=0.25):
    if x < nose:
        return 0.5 * t * math.sqrt(max(0.0, 1 - ((nose - x) / nose) ** 2))
    return 0.5 * t if x < hold else 0.5 * t * (1 - x) / (1 - hold)


def half_plate(x, t=0.02, nose=0.05):
    if x < nose:
        return 0.5 * t * math.sqrt(max(0.0, 1 - ((nose - x) / nose) ** 2))
    if x > 1 - nose:
        return 0.5 * t * math.sqrt(max(0.0, 1 - ((x - 1 + nose) / nose) ** 2))
    return 0.5 * t


def write(path, title, half, f, p, n=160):
    xs = [0.5 * (1 - math.cos(math.pi * k / n)) for k in range(n + 1)]
    upper = [(x, camber(x, f, p) + half(x)) for x in xs]
    lower = [(x, camber(x, f, p) - half(x)) for x in xs]
    with open(path, "w") as out:
        out.write(title + "\n")
        for x, y in list(reversed(upper)) + lower[1:]:
            out.write(f"{x:.6f} {y:.6f}\n")


def main():
    d = sys.argv[1]
    os.makedirs(d, exist_ok=True)
    write(os.path.join(d, "film-rib-4.dat"), "film-rib-4: 4% camber rib, 2.5% foam nose (assumed profile)", half_film, 0.04, 0.5)
    write(os.path.join(d, "thin-plate.dat"), "thin-plate: 2% flat plate, elliptic nose and tail", half_plate, 0.0, 0.5)


if __name__ == "__main__":
    main()
