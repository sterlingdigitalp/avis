#!/usr/bin/env python3
"""Pull the E387 (E) drag polars out of UIUC Vol. 5 Appendix B text (pdftotext -layout) into JSON.

Run with -I: the PDF text is an untrusted download. These two tables are the last column on the page, so each row is the
trailing three numbers on its line.
"""
import json
import re
import sys

src, out = sys.argv[1], sys.argv[2]
lines = open(src).read().split("\n")
polars = {}
for key, run, re_nom in (("99932.8", "BM05374", 100000), ("199747.1", "BM05376", 200000)):
    i = next(k for k, l in enumerate(lines) if ("Re = " + key) in l)
    rows = []
    for l in lines[i + 2:i + 40]:
        m = re.search(r"(-?\d+\.\d\d)\s+(-?\d\.\d\d\d)\s+(\d\.\d\d\d\d)\s*$", l)
        if not m:
            if rows:
                break
            continue
        if rows and float(m.group(1)) <= rows[-1]["alpha_deg"]:
            break  # alpha fell: the match came from a neighbouring column once this table ended
        rows.append(dict(alpha_deg=float(m.group(1)), cl=float(m.group(2)), cd=float(m.group(3))))
    polars[str(re_nom)] = dict(run=run, re=float(key), rows=rows)
json.dump(dict(
    airfoil="E387 (E)",
    source="Williamson, Lyon, McGranahan, Selig et al., Summary of Low-Speed Airfoil Data Vol. 5, Appendix B, Fig. 4.113",
    url="https://m-selig.web.engr.illinois.edu/uiuc_lsat/Low-Speed-Airfoil-Data-V5.pdf",
    freestream_turbulence=dict(figure="Vol. 5 Fig. 2.3, test apparatus installed, read from plot",
                               percent_hpf_3hz=dict(re100k=0.10, re200k=0.08),
                               percent_dc_coupled=dict(re100k=0.12, re200k=0.145)),
    polars=polars), open(out, "w"), indent=1)
print({k: len(v["rows"]) for k, v in polars.items()})
