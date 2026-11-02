#!/usr/bin/env python3
"""Write one 2D airfoil OpenFOAM case. Non-dimensional: chord 1, U 1, nu = 1/Re."""
import json
import math
import os
import shutil

BETA, BETA_STAR = 0.0828, 0.09
NU_RATIO = 5.0
DECAY_LENGTH = 40.0


def freestream_turbulence(tu_pct, nu, ratio=NU_RATIO, length=DECAY_LENGTH):
    """Far-field k and omega such that decay over `length` delivers tu_pct at the airfoil."""
    target = 1.5 * (tu_pct / 100.0) ** 2

    def delivered(k0):
        omega0 = k0 / (ratio * nu)
        return k0 * (1.0 + BETA * omega0 * length) ** (-BETA_STAR / BETA)
    lo, hi = target, target * 1e6
    if delivered(hi) < target:
        raise ValueError(f"Tu {tu_pct}% unreachable at nut/nu={ratio}, nu={nu}")
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if delivered(mid) < target:
            lo = mid
        else:
            hi = mid
    return hi, hi / (ratio * nu)


def retheta_correlation(tu_pct):
    """Langtry-Menter Re_theta,t at zero pressure gradient (Tu in percent, <= 1.3)."""
    tu = max(tu_pct, 0.027)
    return 1173.51 - 589.428 * tu + 0.2196 / tu ** 2


def header(cls, obj, loc):
    return (f'FoamFile\n{{\n    format      ascii;\n    class       {cls};\n'
            f'    location    "{loc}";\n    object      {obj};\n}}\n\n')


def field(name, cls, dims, internal, far, wall):
    return (header(cls, name, "0") + f"dimensions      {dims};\n\ninternalField   uniform {internal};\n\n"
            f"boundaryField\n{{\n    farfield\n    {{\n{far}\n    }}\n    airfoil\n    {{\n{wall}\n    }}\n"
            "    frontAndBack\n    {\n        type empty;\n    }\n}\n")


def inlet_outlet(v):
    return f"        type inletOutlet;\n        inletValue uniform {v};\n        value uniform {v};"


def write_fields(d, a, kf, omegaf, retheta):
    z = os.path.join(d, "0")
    os.makedirs(z, exist_ok=True)
    u = f"({math.cos(a):.10f} {math.sin(a):.10f} 0)"
    files = {
        "U": field("U", "volVectorField", "[0 1 -1 0 0 0 0]", u,
                   f"        type freestreamVelocity;\n        freestreamValue uniform {u};\n        value uniform {u};",
                   "        type noSlip;"),
        "p": field("p", "volScalarField", "[0 2 -2 0 0 0 0]", "0",
                   "        type freestreamPressure;\n        freestreamValue uniform 0;\n        value uniform 0;",
                   "        type zeroGradient;"),
        "k": field("k", "volScalarField", "[0 2 -2 0 0 0 0]", f"{kf:.8g}", inlet_outlet(f"{kf:.8g}"),
                   "        type fixedValue;\n        value uniform 0;"),
        "omega": field("omega", "volScalarField", "[0 0 -1 0 0 0 0]", f"{omegaf:.8g}", inlet_outlet(f"{omegaf:.8g}"),
                       f"        type omegaWallFunction;\n        value uniform {omegaf:.8g};"),
        "nut": field("nut", "volScalarField", "[0 2 -1 0 0 0 0]", "0",
                     "        type calculated;\n        value uniform 0;",
                     "        type nutLowReWallFunction;\n        value uniform 0;"),
        "gammaInt": field("gammaInt", "volScalarField", "[0 0 0 0 0 0 0]", "1", inlet_outlet("1"),
                          "        type zeroGradient;"),
        "ReThetat": field("ReThetat", "volScalarField", "[0 0 0 0 0 0 0]", f"{retheta:.6g}",
                          inlet_outlet(f"{retheta:.6g}"), "        type zeroGradient;"),
    }
    for name, text in files.items():
        with open(os.path.join(z, name), "w") as f:
            f.write(text)


FUNCTIONS = """\
forceCoeffs1
{{
    type            forceCoeffs;
    libs            ("libforces.so");
    writeControl    timeStep;
    writeInterval   {force_interval};
    log             no;
    patches         (airfoil);
    rho             rhoInf;
    rhoInf          1;
    magUInf         1;
    lRef            1;
    Aref            {span};
    CofR            (0.25 0 0);
    liftDir         ({lx:.10f} {ly:.10f} 0);
    dragDir         ({dx:.10f} {dy:.10f} 0);
    pitchAxis       (0 0 -1);
}}
yPlus1
{{
    type            yPlus;
    libs            ("libfieldFunctionObjects.so");
    writeControl    writeTime;
    log             no;
}}
wallShearStress1
{{
    type            wallShearStress;
    libs            ("libfieldFunctionObjects.so");
    writeControl    writeTime;
    log             no;
}}
upstream
{{
    type            probes;
    libs            ("libsampling.so");
    writeControl    writeTime;
    fields          (k omega);
    probeLocations  ((-1 0 0.05));
}}
stopFile
{{
    type            stopAtFile;
    libs            ("libutilityFunctionObjects.so");
    file            "$FOAM_CASE/stop";
    action          writeNow;
}}
"""

SCHEMES = """\
ddtSchemes {{ default {ddt}; }}
gradSchemes
{{
    default         Gauss linear;
    limited         cellLimited Gauss linear 1;
    grad(U)         $limited;
    grad(k)         $limited;
    grad(omega)     $limited;
}}
divSchemes
{{
    default             none;
    div(phi,U)          {bnd}Gauss linearUpwind grad(U);
    div(phi,k)          {bnd}Gauss limitedLinear 1;
    div(phi,omega)      {bnd}Gauss limitedLinear 1;
    div(phi,gammaInt)   {bnd}Gauss limitedLinear 1;
    div(phi,ReThetat)   {bnd}Gauss limitedLinear 1;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}}
laplacianSchemes {{ default Gauss linear limited corrected 0.5; }}
interpolationSchemes {{ default linear; }}
snGradSchemes {{ default limited corrected 0.5; }}
wallDist {{ method meshWave; }}
"""

SOLVERS = """\
solvers
{
    p
    {
        solver          GAMG;
        smoother        GaussSeidel;
        tolerance       1e-8;
        relTol          0.05;
    }
    pFinal { $p; smoother DICGaussSeidel; nCellsInCoarsestLevel 50; relTol 0; tolerance 1e-6; }
    Phi { $p; }
    "pcorr.*" { $p; tolerance 1e-6; relTol 0; }
    "(U|k|omega|gammaInt|ReThetat)"
    {
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-10;
        relTol          0.05;
        maxIter         10;
    }
    "(U|k|omega|gammaInt|ReThetat)Final" { solver smoothSolver; smoother symGaussSeidel; tolerance 1e-9; relTol 0.01; maxIter 20; }
}
potentialFlow { nNonOrthogonalCorrectors 2; }
"""

STEADY = """\
SIMPLE
{
    consistent      no;
    nNonOrthogonalCorrectors 1;
    residualControl { p 1e-9; U 1e-9; }
}
relaxationFactors
{
    fields { p 0.3; }
    equations
    {
        U 0.7;
        ".*" 0.5;
    }
}
"""

# Tested on Re 100k, alpha -2 from a converged steady field: under-relaxed PIMPLE diverged within 40 steps at every dt
# tried (0.002 to 0.01), and a loose final pressure (relTol 0.01) diverged after 2.3 c/U. Consistent PIMPLE without
# relaxation and a converged final pressure solve stayed stable.
TRANSIENT = """\
PIMPLE
{
    nOuterCorrectors {n_outer};
    nCorrectors 2;
    nNonOrthogonalCorrectors 0;
    consistent yes;
}
relaxationFactors { equations { ".*" 1; } }
"""


def write_system(d, *, mode, end_time, dt, span, a, ncpu, write_interval, n_outer=2):
    s = os.path.join(d, "system")
    os.makedirs(s, exist_ok=True)
    with open(os.path.join(s, "controlDict"), "w") as f:
        f.write(header("dictionary", "controlDict", "system"))
        f.write(f"application foamRun;\nsolver incompressibleFluid;\nstartFrom latestTime;\nstartTime 0;\n"
                f"stopAt endTime;\nendTime {end_time:g};\ndeltaT {dt:g};\n"
                f"writeControl {'timeStep' if mode == 'steady' else 'runTime'};\nwriteInterval {write_interval:g};\n"
                "purgeWrite 1;\nwriteFormat ascii;\nwritePrecision 10;\ntimePrecision 8;\nrunTimeModifiable no;\n"
                '#includeFunc none\nfunctions { #include "functions" }\n'.replace("#includeFunc none\n", ""))
    interval = 10 if mode == "steady" else 1
    with open(os.path.join(s, "functions"), "w") as f:
        f.write(FUNCTIONS.format(force_interval=interval, span=span, lx=-math.sin(a), ly=math.cos(a),
                                 dx=math.cos(a), dy=math.sin(a)))
    with open(os.path.join(s, "fvSchemes"), "w") as f:
        f.write(header("dictionary", "fvSchemes", "system"))
        f.write(SCHEMES.format(ddt="steadyState" if mode == "steady" else "backward",
                               bnd="bounded " if mode == "steady" else ""))
    with open(os.path.join(s, "fvSolution"), "w") as f:
        f.write(header("dictionary", "fvSolution", "system"))
        f.write(SOLVERS + (STEADY if mode == "steady" else TRANSIENT.replace("{n_outer}", str(n_outer))))
    with open(os.path.join(s, "decomposeParDict"), "w") as f:
        f.write(header("dictionary", "decomposeParDict", "system"))
        f.write(f"numberOfSubdomains {ncpu};\nmethod scotch;\n")


def make_case(d, mesh_dir, *, re, alpha_deg, tu_pct=0.10, mode="steady", end_time=20000, dt=1.0, ncpu=8,
              write_interval=None):
    if os.path.exists(d):
        shutil.rmtree(d)
    c = os.path.join(d, "constant")
    os.makedirs(c)
    shutil.copytree(os.path.join(mesh_dir, "polyMesh"), os.path.join(c, "polyMesh"))
    meta = json.load(open(os.path.join(mesh_dir, "mesh.json")))
    nu = 1.0 / re
    a = math.radians(alpha_deg)
    kf, omegaf = freestream_turbulence(tu_pct, nu)
    with open(os.path.join(c, "physicalProperties"), "w") as f:
        f.write(header("dictionary", "physicalProperties", "constant"))
        f.write(f"viscosityModel constant;\nnu [0 2 -1 0 0 0 0] {nu:.10g};\n")
    with open(os.path.join(c, "momentumTransport"), "w") as f:
        f.write(header("dictionary", "momentumTransport", "constant"))
        f.write("simulationType RAS;\nRAS\n{\n    model kOmegaSSTLM;\n    turbulence on;\n    printCoeffs on;\n}\n")
    write_fields(d, a, kf, omegaf, retheta_correlation(tu_pct))
    write_system(d, mode=mode, end_time=end_time, dt=dt, span=meta["span"], a=a, ncpu=ncpu,
                 write_interval=write_interval or (end_time if mode == "steady" else end_time))
    spec = dict(re=re, alpha_deg=alpha_deg, tu_pct=tu_pct, mode=mode, end_time=end_time, dt=dt, ncpu=ncpu,
                k_far=kf, omega_far=omegaf, nu_ratio=NU_RATIO, retheta_far=retheta_correlation(tu_pct),
                mesh=os.path.basename(mesh_dir), mesh_cells=meta["cells"], y1=meta["y1"], radius=meta["radius"])
    with open(os.path.join(d, "spec.json"), "w") as f:
        json.dump(spec, f, indent=1)
    return spec
