#!/usr/bin/env python3
"""Short report for the 3D finalist check, from out/cfd/<design>/<tag>/result.json. Reports only measured values.

    report3d.py <design> [tag]
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def fmt(v, n=3):
    return "n/a" if v is None else f"{v:.{n}f}" if isinstance(v, float) else str(v)


def main():
    design, tag = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "final"
    d = os.path.join(ROOT, "out/cfd", design, tag)
    r = json.load(open(os.path.join(d, "result.json")))
    s, p = r["spec"], r["pose"]
    lines = [
        f"# {design}: 3D frozen-glide check ({tag})",
        "",
        "Simulated, not measured: OpenFOAM 13 RANS (kOmegaSSTLM), half model with a symmetry plane, rigid glide pose.",
        "",
        f"- Pose: V = {fmt(p['V'], 2)} m/s, body angle of attack {fmt(p['aoa_deg'], 2)} deg (control.glide theta - gamma), "
        f"wings at the mean stroke angle {fmt(p['phi_deg'], 1)} deg with zero commanded twist, tail at incidence + glide "
        f"tail angle {fmt(p['glide'].get('tail'), 4)} rad.",
        f"- Air: rho = {fmt(r['atmosphere']['rho'], 4)} kg/m3, mu = {r['atmosphere']['mu']:.4g} Pa s (design file, 1800 m, 10 C). "
        f"Freestream turbulence {s['tu_pct']}% (assumed; matches the 2D polars).",
        f"- Mesh: {r['mesh'] and r['mesh']['cells']} cells, surface cell {s['surface_cell'] * 1000:.1f} mm, "
        f"{s['n_layers']} prism layers requested from {s['first_layer'] * 1e6:.0f} um at ratio {s['ratio']}; "
        f"checkMesh {'OK' if r['mesh'] and r['mesh']['mesh_ok'] else 'reported failed checks'} "
        f"(max non-orthogonality {r['mesh'] and r['mesh']['non_ortho_max']}, max skewness {r['mesh'] and r['mesh']['skewness_max']}).",
        "",
        "| Prism layers | faces | mean layers | thickness | % of target |",
        "|---|---|---|---|---|",
    ]
    for part, v in (r.get("layers") or {}).items():
        lines.append(f"| {part} | {v['faces']} | {v['layers_mean']} | {v['thickness_m'] * 1000:.2f} mm | {v['thickness_pct']} |")
    lines += ["", "| y+ | min | mean | max |", "|---|---|---|---|"]
    for part, v in (r.get("yplus") or {}).items():
        if part in ("body", "wing", "tail"):
            lines.append(f"| {part} | {fmt(v['min'], 2)} | {fmt(v['mean'], 2)} | {fmt(v['max'], 2)} |")
    lines += [
        "",
        f"Result after {fmt(r.get('iterations'), 0)} iterations ({'settled' if r.get('converged') else 'NOT settled'}: "
        "lift and drag means over consecutive 500-iteration windows within 0.5% and 1%):",
        "",
        f"- Lift {fmt(r.get('lift_n'))} N (weight {fmt(r['weight_n'])} N), drag {fmt(r.get('drag_n'), 4)} N, "
        f"L/D {fmt(r.get('L_over_D'), 2)}.",
        f"- CL {fmt(r.get('CL'))}, CD {fmt(r.get('CD'), 4)} on wing area {fmt(r.get('wing_area_m2'), 4)} m2; "
        f"viscous share of drag {fmt(r.get('viscous_drag_share'), 2)}.",
        f"- Range over the averaging window: lift {r.get('lift_range_n')}, drag {r.get('drag_range_n')}.",
        "",
        "Assumptions the design file leaves open are listed in cfd/stl_bird.py (section placement, all-moving V-tail, "
        "ellipsoid pod and 8 mm boom). No number here was adjusted.",
    ]
    out = os.path.join(ROOT, "out/cfd", design, "report.md")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(out)


if __name__ == "__main__":
    main()
