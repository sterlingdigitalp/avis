#!/usr/bin/env python3
"""Whole-bird frozen glide case: half model with a symmetry plane at z = 0, snappyHexMesh with prism layers.

A glide pose is left-right symmetric, so the half model gives the whole bird's lift and drag by doubling.
Body frame: x forward, y up, z right. The flow runs towards -x, with the angle of attack in the x-y plane.

    case3d.py <caseDir> <stlDir> --speed V --aoa-deg A --rho R --nu N --lref C --first-layer Y [--level 0..]
"""
import argparse
import json
import math
import os
import shutil


def header(cls, obj, loc):
    return (f'FoamFile\n{{\n    format      ascii;\n    class       {cls};\n'
            f'    location    "{loc}";\n    object      {obj};\n}}\n\n')


def layer_count(first, cell, ratio, fill=0.8):
    """Most layers whose whole stack, first * (ratio^n - 1) / (ratio - 1), fits in `fill` of a surface cell; at least 3.

    snappyHexMesh extrudes layers by shrinking the cells next to the wall, so a stack thicker than about one cell
    collapses (measured: a 7-cell stack on a 5 mm mesh gave 0.7% of the target thickness)."""
    n = math.floor(math.log(1 + fill * cell * (ratio - 1) / first) / math.log(ratio))
    return max(3, n)


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


def field(name, cls, dims, internal, far, parts):
    walls = "\n".join(f"    {p}\n    {{\n{parts}\n    }}" for p in ('"(body|wing|tail)"',))
    return (header(cls, name, "0") + f"dimensions {dims};\ninternalField uniform {internal};\nboundaryField\n{{\n"
            f'    #includeEtc "caseDicts/setConstraintTypes"\n'
            f"    farfield\n    {{\n{far}\n    }}\n{walls}\n}}\n")


def make_case(d, stl_dir, *, speed, aoa_deg, rho, nu, lref, area_half, first_layer, cpus, surface_cell, tu_pct=0.1,
              nu_ratio=5.0, ratio=1.2, end_time=3000, turbulence="kOmegaSSTLM", layer_feature_angle=180,
              medial_ratio=1.0):
    if os.path.exists(d):
        shutil.rmtree(d)
    tri = os.path.join(d, "constant", "triSurface")
    os.makedirs(tri)
    for part in ("body", "wing", "tail"):
        shutil.copy(os.path.join(stl_dir, part + ".stl"), tri)
    a = math.radians(aoa_deg)
    ux, uy = -speed * math.cos(a), speed * math.sin(a)  # flow towards -x, nose up at positive AoA sees flow from below
    span = 2.0 * max(json.load(open(os.path.join(stl_dir, "bounds.json")))["max"][2], 1e-3)
    half = max(8.0 * lref, 1.5 * span)
    x_min, x_max, y_h, z_max = -2.2 * half, 1.2 * half, 1.2 * half, 1.2 * half
    # Background cells of half a mean chord, so the wing cuts cell edges and gets refined; halvings to the surface cell.
    levels = max(1, round(math.log2(0.5 * lref / surface_cell)))
    base = surface_cell * 2 ** levels
    nx, ny, nz = (max(4, round((b - c) / base)) for b, c in ((x_max, x_min), (y_h, -y_h), (z_max, 0.0)))
    n_layers = layer_count(first_layer, surface_cell, ratio)
    write(os.path.join(d, "system/blockMeshDict"), header("dictionary", "blockMeshDict", "system") + f"""\
vertices (({x_min} {-y_h} 0) ({x_max} {-y_h} 0) ({x_max} {y_h} 0) ({x_min} {y_h} 0)
          ({x_min} {-y_h} {z_max}) ({x_max} {-y_h} {z_max}) ({x_max} {y_h} {z_max}) ({x_min} {y_h} {z_max}));
blocks (hex (0 1 2 3 4 5 6 7) ({nx} {ny} {nz}) simpleGrading (1 1 1));
boundary
(
    symmetryPlane {{ type symmetryPlane; faces ((0 3 2 1)); }}
    farfield {{ type patch; faces ((0 4 7 3) (1 2 6 5) (0 1 5 4) (3 7 6 2) (4 5 6 7)); }}
);
""")
    write(os.path.join(d, "system/snappyHexMeshDict"), header("dictionary", "snappyHexMeshDict", "system") + f"""\
castellatedMesh true;
snap true;
addLayers true;
geometry
{{
    body {{ type triSurface; file "body.stl"; }}
    wing {{ type triSurface; file "wing.stl"; }}
    tail {{ type triSurface; file "tail.stl"; }}
    near {{ type box; min ({-1.5 * lref} {-0.5 * lref} 0); max ({1.5 * lref} {0.5 * lref} {0.6 * span}); }}
    wake {{ type box; min ({-6 * lref} {-1.2 * lref} 0); max ({1.5 * lref} {1.2 * lref} {0.7 * span}); }}
}};
castellatedMeshControls
{{
    maxLocalCells 2000000;
    maxGlobalCells 30000000;
    minRefinementCells 10;
    maxLoadUnbalance 0.10;
    nCellsBetweenLevels 3;
    features ();
    refinementSurfaces
    {{
        wing {{ level ({levels} {levels + 1}); patchInfo {{ type wall; }} }}
        tail {{ level ({levels} {levels + 1}); patchInfo {{ type wall; }} }}
        body {{ level ({levels - 1} {levels}); patchInfo {{ type wall; }} }}
    }}
    resolveFeatureAngle 30;
    refinementRegions
    {{
        near {{ mode inside; level {max(levels - 3, 1)}; }}
        wake {{ mode inside; level {max(levels - 4, 1)}; }}
    }}
    insidePoint ({x_max * 0.9} {y_h * 0.9} {z_max * 0.9});
    allowFreeStandingZoneFaces false;
}}
snapControls {{ nSmoothPatch 3; tolerance 2.0; nSolveIter 50; nRelaxIter 5; nFeatureSnapIter 10; implicitFeatureSnap true; }}
addLayersControls
{{
    relativeSizes false;
    layers {{ "(body|wing|tail)" {{ nSurfaceLayers {n_layers}; }} }}
    expansionRatio {ratio};
    firstLayerThickness {first_layer};
    minThickness {first_layer * 0.3};
    nGrow 0;
    featureAngle {layer_feature_angle};
    slipFeatureAngle 30;
    nRelaxIter 5;
    nSmoothSurfaceNormals 1;
    nSmoothNormals 3;
    nSmoothThickness 10;
    maxFaceThicknessRatio 0.5;
    maxThicknessToMedialRatio {medial_ratio};
    minMedianAxisAngle 90;
    nBufferCellsNoExtrude 0;
    nLayerIter 50;
    nRelaxedIter 20;
}}
meshQualityControls
{{
    maxNonOrtho 70; maxBoundarySkewness 20; maxInternalSkewness 4; maxConcave 80; minFlatness 0.5; minVol -1e30;
    minTetQuality 1e-30; minTwist 0.02; minDeterminant 0.001; minFaceWeight 0.02; minVolRatio 0.01; nSmoothScale 4;
    errorReduction 0.75; minArea -1; relaxed {{ maxNonOrtho 75; }}
}}
writeFlags (layerSets);
mergeTolerance 1e-6;
""")
    write(os.path.join(d, "system/decomposeParDict"), header("dictionary", "decomposeParDict", "system")
          + f"numberOfSubdomains {cpus};\nmethod scotch;\n")
    # Lift perpendicular to the flow, drag along it, both in the x-y plane.
    write(os.path.join(d, "system/functions"), f"""\
forces1
{{
    type forces; libs ("libforces.so"); writeControl timeStep; writeInterval 5; log no;
    patches (body wing tail); rho rhoInf; rhoInf {rho}; CofR (0 0 0);
}}
byPart_body {{ type forces; libs ("libforces.so"); writeControl timeStep; writeInterval 20; log no; patches (body); rho rhoInf; rhoInf {rho}; CofR (0 0 0); }}
byPart_wing {{ type forces; libs ("libforces.so"); writeControl timeStep; writeInterval 20; log no; patches (wing); rho rhoInf; rhoInf {rho}; CofR (0 0 0); }}
byPart_tail {{ type forces; libs ("libforces.so"); writeControl timeStep; writeInterval 20; log no; patches (tail); rho rhoInf; rhoInf {rho}; CofR (0 0 0); }}
yPlus1 {{ type yPlus; libs ("libfieldFunctionObjects.so"); writeControl writeTime; log no; }}
stopFile {{ type stopAtFile; libs ("libutilityFunctionObjects.so"); file "$FOAM_CASE/stop"; action writeNow; }}
""")
    write(os.path.join(d, "system/controlDict"), header("dictionary", "controlDict", "system")
          + f"application foamRun;\nsolver incompressibleFluid;\nstartFrom latestTime;\nstartTime 0;\nstopAt endTime;\nendTime {end_time};\n"
            f"deltaT 1;\nwriteControl timeStep;\nwriteInterval {end_time};\npurgeWrite 1;\nwriteFormat binary;\nwritePrecision 8;\ntimePrecision 6;\n"
            'runTimeModifiable no;\nfunctions { #include "functions" }\n')
    write(os.path.join(d, "system/fvSchemes"), header("dictionary", "fvSchemes", "system") + """\
ddtSchemes { default steadyState; }
gradSchemes
{
    default Gauss linear;
    limited cellLimited Gauss linear 1;
    grad(U) $limited;
    grad(k) $limited;
    grad(omega) $limited;
}
divSchemes
{
    default none;
    div(phi,U) bounded Gauss linearUpwind grad(U);
    div(phi,k) bounded Gauss limitedLinear 1;
    div(phi,omega) bounded Gauss limitedLinear 1;
    div(phi,gammaInt) bounded Gauss limitedLinear 1;
    div(phi,ReThetat) bounded Gauss limitedLinear 1;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}
laplacianSchemes { default Gauss linear limited corrected 0.5; }
interpolationSchemes { default linear; }
snGradSchemes { default limited corrected 0.5; }
wallDist { method meshWave; }
""")
    write(os.path.join(d, "system/fvSolution"), header("dictionary", "fvSolution", "system") + """\
solvers
{
    p { solver GAMG; smoother GaussSeidel; tolerance 1e-8; relTol 0.05; }
    Phi { $p; }
    "(U|k|omega|gammaInt|ReThetat)" { solver smoothSolver; smoother symGaussSeidel; tolerance 1e-10; relTol 0.05; maxIter 10; }
}
potentialFlow { nNonOrthogonalCorrectors 3; }
SIMPLE { consistent no; nNonOrthogonalCorrectors 2; residualControl { p 1e-9; U 1e-9; } }
relaxationFactors { fields { p 0.3; } equations { U 0.7; ".*" 0.5; } }
""")
    write(os.path.join(d, "constant/physicalProperties"), header("dictionary", "physicalProperties", "constant")
          + f"viscosityModel constant;\nnu [0 2 -1 0 0 0 0] {nu:.8g};\n")
    write(os.path.join(d, "constant/momentumTransport"), header("dictionary", "momentumTransport", "constant")
          + f"simulationType RAS;\nRAS {{ model {turbulence}; turbulence on; printCoeffs on; }}\n")
    k = 1.5 * (tu_pct / 100.0 * speed) ** 2
    omega = k / (nu_ratio * nu)
    u = f"({ux:.8g} {uy:.8g} 0)"
    tu_tilde = max(tu_pct, 0.027)
    retheta = 1173.51 - 589.428 * tu_tilde + 0.2196 / tu_tilde ** 2
    io = lambda v: f"        type inletOutlet;\n        inletValue uniform {v};\n        value uniform {v};"
    fields = {
        "U": field("U", "volVectorField", "[0 1 -1 0 0 0 0]", u, f"        type freestreamVelocity;\n        freestreamValue uniform {u};\n        value uniform {u};", "        type noSlip;"),
        "p": field("p", "volScalarField", "[0 2 -2 0 0 0 0]", "0", "        type freestreamPressure;\n        freestreamValue uniform 0;\n        value uniform 0;", "        type zeroGradient;"),
        "k": field("k", "volScalarField", "[0 2 -2 0 0 0 0]", f"{k:.6g}", io(f"{k:.6g}"), "        type fixedValue;\n        value uniform 0;"),
        "omega": field("omega", "volScalarField", "[0 0 -1 0 0 0 0]", f"{omega:.6g}", io(f"{omega:.6g}"), f"        type omegaWallFunction;\n        value uniform {omega:.6g};"),
        "nut": field("nut", "volScalarField", "[0 2 -1 0 0 0 0]", "0", "        type calculated;\n        value uniform 0;", "        type nutLowReWallFunction;\n        value uniform 0;"),
        "gammaInt": field("gammaInt", "volScalarField", "[0 0 0 0 0 0 0]", "1", io("1"), "        type zeroGradient;"),
        "ReThetat": field("ReThetat", "volScalarField", "[0 0 0 0 0 0 0]", f"{retheta:.6g}", io(f"{retheta:.6g}"), "        type zeroGradient;"),
    }
    for name, text in fields.items():
        write(os.path.join(d, "0.orig", name), text)
    spec = dict(speed=speed, aoa_deg=aoa_deg, rho=rho, nu=nu, lref=lref, area_half=area_half, first_layer=first_layer,
                surface_cell=surface_cell, n_layers=n_layers, layer_feature_angle=layer_feature_angle,
                medial_ratio=medial_ratio, ratio=ratio, tu_pct=tu_pct, turbulence=turbulence,
                background=[nx, ny, nz], background_cell=base, levels=levels, domain=dict(x=[x_min, x_max], y=[-y_h, y_h], z=[0, z_max]), cpus=cpus,
                ux=ux, uy=uy)
    with open(os.path.join(d, "spec.json"), "w") as f:
        json.dump(spec, f, indent=1)
    return spec


def lift_drag(fx, fy, aoa_deg):
    """Half-model force resolved to whole-bird lift and drag. Flow runs towards -x, rotated up by aoa."""
    a = math.radians(aoa_deg)
    drag = -fx * math.cos(a) + fy * math.sin(a)
    lift = fx * math.sin(a) + fy * math.cos(a)
    return 2.0 * lift, 2.0 * drag
