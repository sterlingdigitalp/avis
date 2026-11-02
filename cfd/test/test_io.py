"""Parsers, orchestration and scoring, on small fixture files. Docker is replaced by a fake that writes what a
solver run would leave behind, so these run anywhere.

    python3 -m unittest discover -s cfd/test
"""
import importlib.util
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import bird3d
import pipeline
import polars
import stl_bird


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


def load_gate():
    spec = importlib.util.spec_from_file_location("v7gate", os.path.join(os.path.dirname(HERE), "gates", "v7.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Bird3dParsers(unittest.TestCase):
    def test_forces_sum_pressure_and_viscous(self):
        with tempfile.TemporaryDirectory() as d:
            write(os.path.join(d, "postProcessing/forces1/0/forces.dat"),
                  "# Time forces moments\n"
                  "5\t((1.0 2.0 3.0) (0.5 -0.25 0)) ((0 0 0) (0 0 0))\n"
                  "10\t((1e-1 2e+0 0) (1e-2 0 0)) ((0 0 0) (0 0 0))\n")
            rows = bird3d.read_forces(d)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0][:4], (5.0, 1.5, 1.75, 3.0))
            self.assertAlmostEqual(rows[1][1], 0.11)

    def test_layer_table(self):
        with tempfile.TemporaryDirectory() as d:
            write(os.path.join(d, "log.snappyHexMesh"),
                  "noise\npatch faces    layers   overall thickness\n                       [m]       [%]\n"
                  "----- -----    ------   ---       ---\nbody 1631     1.8      0.00164   45.7\n"
                  "wing 34664    5.9      0.00296   82.1\ntail 10663    7.97     0.00305   84.6\n\nLayer mesh\n")
            got = bird3d.layer_coverage(d)
            self.assertEqual(got["wing"], dict(faces=34664, layers_mean=5.9, thickness_m=0.00296, thickness_pct=82.1))
            self.assertEqual(set(got), {"body", "wing", "tail"})

    def test_layer_table_absent(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(bird3d.layer_coverage(d))
            write(os.path.join(d, "log.snappyHexMesh"), "no table here\n")
            self.assertIsNone(bird3d.layer_coverage(d))

    def test_yplus_last_row_per_patch(self):
        with tempfile.TemporaryDirectory() as d:
            write(os.path.join(d, "postProcessing/yPlus1/0/yPlus.dat"),
                  "# Time patch min max average\n10 wing 0.1 3 0.8\n10 tail 0.2 2 0.5\n20 wing 0.1 1.5 0.6\n")
            got = bird3d.yplus(d)
            self.assertEqual(got["wing"], dict(min=0.1, max=1.5, mean=0.6))
            self.assertEqual(got["tail"]["max"], 2.0)

    def test_summary_whole_bird(self):
        a, rho, V, S = 4.0, 1.0, 10.0, 0.5
        ar = math.radians(a)
        # Half-model force giving whole-bird lift 20 N and drag 2 N; viscous part 0.5 N of the half drag.
        fx = -1.0 * math.cos(ar) + 10.0 * math.sin(ar)
        fy = 1.0 * math.sin(ar) + 10.0 * math.cos(ar)
        fv = (-0.5 * math.cos(ar), 0.5 * math.sin(ar), 0.0)
        rows = [(float(k), fx, fy, 0.0, None, fv) for k in range(1, 1001)]
        got = bird3d.summarise(rows, a, rho, V, S)
        self.assertAlmostEqual(got["lift_n"], 20.0)
        self.assertAlmostEqual(got["drag_n"], 2.0)
        self.assertAlmostEqual(got["CL"], 20.0 / (0.5 * rho * V * V * S))
        self.assertAlmostEqual(got["L_over_D"], 10.0)
        self.assertAlmostEqual(got["viscous_drag_share"], 0.5)


class PipelineParsers(unittest.TestCase):
    def test_force_coefficients_across_restarts(self):
        with tempfile.TemporaryDirectory() as d:
            write(os.path.join(d, "postProcessing/forceCoeffs1/0/forceCoeffs.dat"),
                  "# Time Cm Cd Cl\n10 -0.1 0.02 0.5\n20 -0.1 0.02 0.6\n")
            write(os.path.join(d, "postProcessing/forceCoeffs1/20/forceCoeffs.dat"), "# h\n20.5 -0.1 0.03 nan\n")
            rows = pipeline.read_forces(d)
            self.assertEqual([r[0] for r in rows], [10.0, 20.0, 20.5])
            self.assertTrue(pipeline.bad(rows))

    def test_probe_tu(self):
        with tempfile.TemporaryDirectory() as d:
            write(os.path.join(d, "postProcessing/upstream/0/k"), "# Probe 0 (-1 0 0.05)\n# Time 0\n100 1.5e-6\n")
            self.assertAlmostEqual(pipeline.probe_tu(d), 0.1)
            self.assertIsNone(pipeline.probe_tu(os.path.join(d, "missing")))

    def test_wall_extraction(self):
        with tempfile.TemporaryDirectory() as root:
            mesh, d = os.path.join(root, "mesh"), os.path.join(root, "case")
            # Lower TE -> LE -> upper TE, two faces per side, chord 1.
            wall = [(1, 0), (0.5, -0.05), (0, 0), (0.5, 0.05), (1, 0)]
            write(os.path.join(mesh, "mesh.json"), json.dumps(dict(wall=wall)))
            field = lambda kind, vals: ("boundaryField\n{\n    airfoil\n    {\n        type x;\n        value nonuniform "
                                        f"List<{kind}> {len(vals)}({' '.join(vals)});\n    }}\n}}\n")
            write(os.path.join(d, "800", "p"), field("scalar", ["0.1", "-0.2", "-0.4", "0.05"]))
            # Upper faces: forward shear (+x) on the first, reversed on the second.
            write(os.path.join(d, "800", "wallShearStress"),
                  field("vector", ["(-0.001 0 0)", "(-0.001 0 0)", "(0.002 0 0)", "(-0.001 0 0)"]))
            write(os.path.join(d, "800", "yPlus"), field("scalar", ["0.5", "0.6", "0.9", "1.2"]))
            w = pipeline.extract_wall(d, mesh)
            self.assertEqual(len(w["upper"]), 2)
            self.assertAlmostEqual(w["upper"][0]["cp"], -0.8)
            s = pipeline.wall_summary(w)
            self.assertAlmostEqual(s["yplus_max"], 1.2)
            self.assertAlmostEqual(s["yplus_over_1_frac"], 0.25)


class WallPressure(unittest.TestCase):
    def test_zero_gradient_wall_uses_adjacent_cells(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "p")
            write(path, "internalField   nonuniform List<scalar> \n6\n(\n9\n9\n0.1\n0.2\n9\n9\n)\n;\n"
                        "boundaryField\n{\n    airfoil\n    {\n        type zeroGradient;\n    }\n}\n")
            mesh = dict(n_wake=2, wall=[(1, 0), (0, 0), (1, 0.1)])
            self.assertEqual(pipeline.wall_pressure(path, mesh), [0.1, 0.2])

    def test_cell_numbering_matches_the_mesher(self):
        import cgrid
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "n.dat")
            with open(path, "w") as f:
                f.write("x\n")
                n = 30
                xs = [0.5 * (1 - math.cos(math.pi * k / n)) for k in range(n + 1)]
                ys = [0.6 * (0.2969 * math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4) for x in xs]
                f.writelines(f"{x} {y}\n" for x, y in list(zip(xs, ys))[::-1])
                f.writelines(f"{x} {-y}\n" for x, y in list(zip(xs, ys))[1:])
            curve, il, iu, meta = cgrid.wall_curve(path, 0.3, 10.0)
            grid, ni, nj, _, _ = cgrid.march(curve, il, iu, 0.3, 10.0, meta["thickness"])
            cgrid.write_polymesh(grid, ni, nj, il, iu, os.path.join(d, "polyMesh"))
            self.assertEqual(il, meta["n_wake"])
            text = open(os.path.join(d, "polyMesh", "boundary")).read()
            start = int(text.split("airfoil")[1].split("startFace")[1].split(";")[0])
            owners = open(os.path.join(d, "polyMesh", "owner")).read()
            owners = [int(v) for v in owners[owners.index("(\n") + 2:owners.rindex("\n)")].split()]
            wall_owners = owners[start:start + (iu - il)]
            self.assertEqual(wall_owners, list(range(meta["n_wake"], meta["n_wake"] + iu - il)))


class Theodorsen(unittest.TestCase):
    """Published table values of C(k) (Theodorsen 1935; Bisplinghoff, Ashley and Halfman, Table 5-1)."""

    def test_table(self):
        import suction
        for k, F, G in ((0.1, 0.8319, -0.1723), (0.2, 0.7276, -0.1886), (0.5, 0.5979, -0.1507), (1.0, 0.5394, -0.1003)):
            f, g = suction.theodorsen(k)
            self.assertAlmostEqual(f, F, delta=6e-4)
            self.assertAlmostEqual(g, G, delta=6e-4)

    def test_quasi_steady_limit(self):
        import suction
        # k -> 0: C -> 1, so ideal suction -> pi * alpha_amp^2 with alpha_amp = omega h0 / U = 2 k h0.
        k, h = 0.001, 0.5
        f, g = suction.theodorsen(k)
        self.assertAlmostEqual(suction.ideal_suction(k, h) / (math.pi * (2 * k * h) ** 2), f * f + g * g)
        self.assertGreater(f, 0.98)

    def test_strip_eta_recovers_a_known_efficiency(self):
        import suction
        # Synthetic history built from strip.js's own relation with eta = 0.4: cc = 0.4 CL sin a - Cd_f cos a.
        spec = dict(alpha_deg=4.0, omega=0.7, h_over_c=0.75, period=2 * math.pi / 0.7)
        a0, w, h0, span, cdf = math.radians(4.0), 0.7, 0.75, 0.1, 0.015
        raw = []
        for i in range(0, 4001):
            t = 3 * spec["period"] * i / 4000
            a = math.atan2(math.sin(a0) - h0 * w * math.cos(w * t), math.cos(a0))
            cl = 2 * math.pi * math.sin(a)
            cc = 0.4 * cl * math.sin(a) - cdf * math.cos(a)
            cx = -cc
            cy = (cl + cx * math.sin(a)) / math.cos(a)  # invert cl = cy cos a - cx sin a
            raw.append((t, cx * 0.5 * span, cy * 0.5 * span))
        got = suction.strip_eta(raw, spec, span, cdf)
        self.assertAlmostEqual(got["eta"], 0.4, places=4)

    def test_cycles_after_startup(self):
        import suction
        rows = [(0.0025 * i, 0.0, 0.0) for i in range(1, 1201)]  # 3.0 periods of 1.0, first output at dt
        self.assertEqual(suction.cycles_after_startup(rows, 1.0), 2)
        self.assertEqual(suction.cycles_after_startup(rows[:300], 1.0), 0)

    def test_strip_eta_uses_only_the_averaged_window(self):
        import suction
        spec = dict(alpha_deg=4.0, omega=0.7, h_over_c=0.75, period=2 * math.pi / 0.7)
        a0, w, h0, span, cdf = math.radians(4.0), 0.7, 0.75, 0.1, 0.015
        raw = []
        for i in range(0, 4001):
            t = 3 * spec["period"] * i / 4000
            eta = 0.9 if t < spec["period"] else 0.4  # start-up cycle different
            a = math.atan2(math.sin(a0) - h0 * w * math.cos(w * t), math.cos(a0))
            cl = 2 * math.pi * math.sin(a) + 0.3
            cx = -(eta * cl * math.sin(a) - cdf * math.cos(a))
            cy = (cl + cx * math.sin(a)) / math.cos(a)
            raw.append((t, cx * 0.5 * span, cy * 0.5 * span))
        got = suction.strip_eta(raw, spec, span, cdf, average=2)
        self.assertAlmostEqual(got["eta"], 0.4, places=3)
        mean = sum((2 * math.pi * math.sin(math.atan2(math.sin(a0) - h0 * w * math.cos(w * t), math.cos(a0))) + 0.3)
                   * math.sin(math.atan2(math.sin(a0) - h0 * w * math.cos(w * t), math.cos(a0)))
                   for t in [spec["period"] * (1 + 2 * j / 2000) for j in range(2000)]) / 2000
        self.assertAlmostEqual(got["mean_cl_sin_a"], mean, places=3)
        self.assertEqual(got["cd_f"], cdf)

    def test_cycle_mean(self):
        import suction
        rows = [(0.01 * i, 1.0 + math.sin(2 * math.pi * 0.01 * i), 0.0) for i in range(0, 501)]
        self.assertAlmostEqual(suction.cycle_mean(rows, 1.0, 2), 1.0, places=4)
        self.assertIsNone(suction.cycle_mean(rows, 1.0, 6))


class FakeSolver:
    """Stands in for pipeline.container: writes a force history and a time directory as a solver run would."""

    def __init__(self, cl):
        self.cl = cl
        self.calls = []

    def __call__(self, name, d, script, max_seconds, cpus, memory, on_poll):
        self.calls.append(script)
        rows = "".join(f"{k} -0.08 0.012 {self.cl(k)}\n" for k in range(10, 4010, 10))
        write(os.path.join(d, "postProcessing/forceCoeffs1/0/forceCoeffs.dat"), "# Time Cm Cd Cl\n" + rows)
        os.makedirs(os.path.join(d, "4000"), exist_ok=True)
        on_poll()
        return 0, False


class RunPoint(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = pipeline.ROOT
        pipeline.ROOT = self.tmp.name
        self.container = pipeline.container
        naca = os.path.join(self.tmp.name, "n.dat")
        with open(naca, "w") as f:
            f.write("flat\n")
            n = 40
            xs = [0.5 * (1 - math.cos(math.pi * k / n)) for k in range(n + 1)]
            ys = [0.6 * (0.2969 * math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4) for x in xs]
            f.writelines(f"{x} {y}\n" for x, y in list(zip(xs, ys))[::-1])
            f.writelines(f"{x} {-y}\n" for x, y in list(zip(xs, ys))[1:])
        self.mesh = os.path.join(self.tmp.name, "out/cfd/meshes/n")
        import cgrid
        curve, il, iu, meta = cgrid.wall_curve(naca, 0.3, 10.0)
        grid, ni, nj, y1, _ = cgrid.march(curve, il, iu, 0.3, 10.0, meta["thickness"])
        _, _, wall = cgrid.write_polymesh(grid, ni, nj, il, iu, os.path.join(self.mesh, "polyMesh"))
        write(os.path.join(self.mesh, "mesh.json"), json.dumps(dict(meta, wall=wall, span=0.1, cells=1, y1=y1, radius=10.0,
                                                                      coords=naca)))

    def tearDown(self):
        pipeline.ROOT = self.root
        pipeline.container = self.container
        self.tmp.cleanup()

    def test_flat_history_is_steady_and_cleaned(self):
        pipeline.container = FakeSolver(lambda k: 0.5)
        r = pipeline.run_point("g", "p1", self.mesh, re_=100000, alpha=2.0)
        self.assertEqual(r["status"], "steady")
        self.assertTrue(r["converged"])
        self.assertAlmostEqual(r["cl"], 0.5)
        d = os.path.join(self.tmp.name, "out/cfd/runs/g/p1")
        self.assertTrue(os.path.exists(os.path.join(d, "result.json")))
        self.assertFalse(os.path.exists(os.path.join(d, "constant")))
        # Resumable: a second call returns the stored result without running anything.
        pipeline.container = FakeSolver(lambda k: 9.9)
        self.assertAlmostEqual(pipeline.run_point("g", "p1", self.mesh, re_=100000, alpha=2.0)["cl"], 0.5)

    def test_oscillating_history_goes_transient_then_flags(self):
        fake = FakeSolver(lambda k: 0.5 + 0.2 * math.sin(k))
        pipeline.container = fake
        r = pipeline.run_point("g", "p2", self.mesh, re_=100000, alpha=2.0, steady_cfg=dict(max_iter=4000))
        self.assertEqual(fake.calls, ["steady.sh", "transient.sh"])
        self.assertFalse(r["converged"])
        self.assertIn(r["status"], ("unconverged-transient", "failed"))

    def test_sweep_runs_points_in_order(self):
        pipeline.container = FakeSolver(lambda k: 0.5)
        pool = pipeline.Pool(1)
        pool.submit_sweep("s", [("a0", 100000, 0.0), ("a2", 100000, 2.0)], self.mesh)
        res = pool.wait()
        self.assertEqual([r["name"] for r in res], ["a0", "a2"])
        self.assertTrue(all(r["status"] == "steady" for r in res))


class PolarFile(unittest.TestCase):
    def test_every_point_present_and_flagged(self):
        with tempfile.TemporaryDirectory() as root:
            old = pipeline.ROOT
            pipeline.ROOT = root
            try:
                write(os.path.join(root, "out/cfd/meshes/x-L1/mesh.json"),
                      json.dumps(dict(coords=os.path.join(root, "x.dat"), thickness=0.1, camber=0.02, te_gap_closed=0,
                                      cells=1, y1=1e-4)))
                for re_ in polars.RES:
                    for a in polars.ALPHAS[:3]:
                        write(os.path.join(root, "out/cfd/runs/polar-x", polars.point_name("x", 1.0, re_, a), "result.json"),
                              json.dumps(dict(status="steady", converged=True, cl=0.1 * a, cd=0.01, cm=-0.05)))
                polars.write("x", 1.0)
                out = json.load(open(os.path.join(root, "data/calibration/polars/x.json")))
            finally:
                pipeline.ROOT = old
            self.assertEqual(len(out["points"]), len(polars.RES) * len(polars.ALPHAS))
            missing = [p for p in out["points"] if p["status"] == "not-run"]
            self.assertEqual(len(missing), len(polars.RES) * (len(polars.ALPHAS) - 3))
            self.assertTrue(all(p["cl"] is None and not p["converged"] for p in missing))
            self.assertEqual(out["cl"][0][0], 0.1 * polars.ALPHAS[0])


class Gate(unittest.TestCase):
    def test_interp_and_nasa(self):
        g = load_gate()
        rows = [dict(alpha_deg=0.0, cl=0.2, cd=0.01), dict(alpha_deg=2.0, cl=0.4, cd=0.03)]
        self.assertAlmostEqual(g.interp(rows, 1.0, "cl"), 0.3)
        with self.assertRaises(ValueError):
            g.interp(rows, 3.0, "cl")
        table = [(0.0, 0.2, 0.01), (2.0, 0.4, None)]
        self.assertEqual(g.nasa_at(table, 1.0)[1], None)
        self.assertAlmostEqual(g.nasa_at(table, 1.0)[0], 0.3)
        self.assertEqual(g.nasa_at(table, 5.0), (None, None))

    def test_score_counts_unconverged_as_failed(self):
        g = load_gate()
        rows = [dict(alpha_deg=-3.0, cl=0.0, cd=0.02), dict(alpha_deg=11.0, cl=1.4, cd=0.02)]
        ref = dict(polars={str(r): dict(rows=rows) for r in g.RES}, nasa={str(r): [] for r in g.RES})
        results = {}
        for r in g.RES:
            for a in g.ALPHAS:
                cl = 0.1 * (a + 3)
                results[f"e387-L1-Re{r}-a{a:g}-Tu0.1"] = dict(status="steady", converged=True, cl=cl, cd=0.02)
        bad = "e387-L1-Re100000-a4-Tu0.1"
        results[bad] = dict(results[bad], status="unconverged-steady", converged=False)
        points, missing = g.score(results, 1.0, ref)
        self.assertEqual(missing, [])
        failed = [p for p in points if not p["passed"]]
        self.assertEqual([(p["re"], p["alpha_deg"]) for p in failed], [(100000, 4)])
        s = g.summary(points)
        self.assertEqual(s["points_passed"], len(points) - 1)
        self.assertEqual(s["points_unconverged"], 1)


class StlLoft(unittest.TestCase):
    def test_closed_and_outward(self):
        sec = ([0.0, 0.5, 1.0], [0.0, 0.05, 0.0], [0.0, -0.05, 0.0])
        rings = [stl_bird.loop(sec, [0, 0, z], [1, 0, 0], [0, 1, 0], 1.0) for z in (0.0, 0.5, 1.0)]
        tris = stl_bird.loft(rings)
        edges = {}
        for t in tris:
            for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
                key = tuple(sorted((tuple(a), tuple(b))))
                edges[key] = edges.get(key, 0) + 1
        self.assertTrue(all(v == 2 for v in edges.values()))
        # Diamond section area 0.05, span 1: |volume| 0.05.
        self.assertAlmostEqual(abs(stl_bird.signed_volume(tris)), 0.05)


if __name__ == "__main__":
    unittest.main()
