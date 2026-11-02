"""Pure-function tests. Expected values come from theory and the published correlation, not from this code.

    python3 -m unittest discover -s cfd/test
"""
import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import airfoil
import case
import cgrid
import pipeline
import polars


class Correlation(unittest.TestCase):
    def test_langtry_menter_value(self):
        # 1173.51 - 589.428*0.1 + 0.2196/0.01, by hand: 1136.5 (Langtry and Menter 2009, Tu <= 1.3%).
        self.assertAlmostEqual(case.retheta_correlation(0.1), 1136.53, places=1)

    def test_low_tu_clips_at_published_floor(self):
        self.assertEqual(case.retheta_correlation(0.001), case.retheta_correlation(0.027))


class FreestreamDecay(unittest.TestCase):
    def test_delivers_target_under_analytic_decay(self):
        nu = 1 / 2e5
        k0, w0 = case.freestream_turbulence(0.1, nu)
        # Wilcox homogeneous decay: k = k0 (1 + beta w0 t)^(-beta*/beta); beta=0.0828, beta*=0.09, t=40.
        k = k0 * (1 + 0.0828 * w0 * 40) ** (-0.09 / 0.0828)
        self.assertAlmostEqual(100 * math.sqrt(2 * k / 3), 0.1, places=6)

    def test_viscosity_ratio(self):
        nu = 1e-5
        k0, w0 = case.freestream_turbulence(0.1, nu)
        self.assertAlmostEqual(k0 / w0 / nu, 5.0, places=9)

    def test_unreachable_tu_is_refused(self):
        with self.assertRaises(ValueError):
            case.freestream_turbulence(5.0, 1e-7, ratio=0.1, length=100.0)


class Spline(unittest.TestCase):
    def test_exact_on_a_line(self):
        s = airfoil.Spline([0, 1, 2, 3], [0, 2, 4, 6])
        self.assertAlmostEqual(s(1.5), 3.0)
        self.assertAlmostEqual(s(2.75), 5.5)

    def test_knots_interpolated(self):
        t, y = [0, 0.5, 1.2, 3], [1, -1, 2, 0.5]
        s = airfoil.Spline(t, y)
        for a, b in zip(t, y):
            self.assertAlmostEqual(s(a), b)


class Stations(unittest.TestCase):
    def test_ends_count_and_clustering(self):
        st = airfoil.clustered_stations(1.0, 100, 0.001, 0.002)
        self.assertEqual(len(st), 101)
        self.assertEqual(st[0], 0.0)
        self.assertEqual(st[-1], 1.0)
        self.assertTrue(all(b > a for a, b in zip(st, st[1:])))
        self.assertLess(st[1] - st[0], st[50] - st[49])


def naca0012(path, n=80):
    t = 0.12
    with open(path, "w") as f:
        f.write("NACA 0012\n")
        xs = [0.5 * (1 - math.cos(math.pi * k / n)) for k in range(n + 1)]
        half = [5 * t * (0.2969 * math.sqrt(x) - 0.126 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4) for x in xs]
        for x, y in list(zip(xs, half))[::-1]:
            f.write(f"{x} {y}\n")
        for x, y in list(zip(xs, half))[1:]:
            f.write(f"{x} {-y}\n")


class Grid(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.path = os.path.join(cls.tmp.name, "n.dat")
        naca0012(cls.path)
        cls.curve, cls.il, cls.iu, cls.meta = cgrid.wall_curve(cls.path, 0.5, 40.0)
        cls.grid, cls.ni, cls.nj, cls.y1, _ = cgrid.march(cls.curve, cls.il, cls.iu, 0.5, 40.0, cls.meta["thickness"])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_thickness_recovered(self):
        self.assertAlmostEqual(self.meta["thickness"], 0.12, delta=1e-3)
        self.assertAlmostEqual(self.meta["camber"], 0.0, delta=1e-6)

    def test_no_folded_cells(self):
        g = self.grid
        signs = set()
        for j in range(self.nj - 1):
            for i in range(self.ni - 1):
                a, b, c, d = g[j][i], g[j][i + 1], g[j + 1][i + 1], g[j + 1][i]
                signs.add(((c[0] - a[0]) * (d[1] - b[1]) - (d[0] - b[0]) * (c[1] - a[1])) > 0)
        self.assertEqual(len(signs), 1)

    def test_first_wall_cell_height(self):
        g = self.grid
        i = (self.il + self.iu) // 2 + 60
        h = math.hypot(g[1][i][0] - g[0][i][0], g[1][i][1] - g[0][i][1])
        self.assertAlmostEqual(h, self.y1, delta=1e-3 * self.y1)

    def test_wake_cut_coincident(self):
        for i in range(self.il + 1):
            p, q = self.curve[i], self.curve[self.ni - 1 - i]
            self.assertAlmostEqual(p[0], q[0], places=12)
            self.assertAlmostEqual(p[1], q[1], places=12)

    def test_far_field_reach(self):
        le = min(self.grid[-1], key=lambda p: p[0])
        self.assertAlmostEqual(le[0], -40.0, delta=2.0)

    def test_polymesh_counts_and_upper_triangular_order(self):
        with tempfile.TemporaryDirectory() as d:
            ncell, _, wall = cgrid.write_polymesh(self.grid, self.ni, self.nj, self.il, self.iu, d)
            self.assertEqual(ncell, (self.ni - 1) * (self.nj - 1))
            self.assertEqual(len(wall) - 1, self.iu - self.il)

            def labels(name):
                with open(os.path.join(d, name)) as f:
                    text = f.read()
                body = text[text.index("(\n") + 2:text.rindex("\n)")]
                return [int(v) for v in body.split()]
            own, nei = labels("owner"), labels("neighbour")
            pairs = list(zip(own, nei))
            self.assertEqual(pairs, sorted(pairs))
            self.assertTrue(all(a < b for a, b in pairs))
            self.assertEqual(max(max(own), max(nei)), ncell - 1)


class Convergence(unittest.TestCase):
    cfg = dict(window=100, min_iter=100, cl_tol=0.005, cd_tol=0.01)

    def rows(self, n, cl, cd):
        return [(float(k), -0.1, cd(k), cl(k)) for k in range(n)]

    def test_flat_history_settles(self):
        self.assertTrue(pipeline.steady_settled(self.rows(300, lambda k: 0.5, lambda k: 0.02), self.cfg))

    def test_oscillating_lift_does_not_settle(self):
        rows = self.rows(300, lambda k: 0.5 + 0.01 * math.sin(k), lambda k: 0.02)
        self.assertFalse(pipeline.steady_settled(rows, self.cfg))

    def test_oscillating_drag_does_not_settle(self):
        rows = self.rows(300, lambda k: 0.5, lambda k: 0.02 + 0.001 * math.sin(k))
        self.assertFalse(pipeline.steady_settled(rows, self.cfg))

    def test_too_few_iterations_does_not_settle(self):
        self.assertFalse(pipeline.steady_settled(self.rows(50, lambda k: 0.5, lambda k: 0.02), self.cfg))

    def test_small_limit_cycle_accepted_by_mean_rule(self):
        cfg = dict(window=100, mean_after=200, mean_cl_tol=0.005, mean_cd_tol=0.01, amp_cl=0.04, amp_cd=0.15)
        rows = self.rows(400, lambda k: 0.5 + 0.004 * math.sin(k), lambda k: 0.02 + 0.0005 * math.sin(k))
        self.assertTrue(pipeline.steady_mean_settled(rows, cfg))

    def test_large_limit_cycle_refused_by_mean_rule(self):
        cfg = dict(window=100, mean_after=200, mean_cl_tol=0.005, mean_cd_tol=0.01, amp_cl=0.04, amp_cd=0.15)
        rows = self.rows(400, lambda k: 0.5 + 0.05 * math.sin(k), lambda k: 0.02)
        self.assertFalse(pipeline.steady_mean_settled(rows, cfg))

    def test_drifting_mean_refused_by_mean_rule(self):
        cfg = dict(window=100, mean_after=200, mean_cl_tol=0.005, mean_cd_tol=0.01, amp_cl=0.04, amp_cd=0.15)
        rows = self.rows(400, lambda k: 0.5 + 0.0005 * k, lambda k: 0.02)
        self.assertFalse(pipeline.steady_mean_settled(rows, cfg))

    def test_nan_is_bad(self):
        self.assertTrue(pipeline.bad([(1.0, float("nan"), 0.1, 0.2)]))
        self.assertTrue(pipeline.bad([(1.0, 0.1, 5e3, 0.2)]))
        self.assertFalse(pipeline.bad([(1.0, 0.1, 0.1, 0.2)]))

    def test_transient_windows_compared(self):
        cfg = dict(window=10.0, min_time=20.0, cl_tol=0.01, cd_tol=0.02)
        steady = [(0.1 * k, -0.1, 0.02, 0.5 + 0.1 * math.sin(2 * math.pi * 0.1 * k)) for k in range(300)]
        self.assertTrue(pipeline.transient_settled(steady, 0.0, cfg))
        drift = [(0.1 * k, -0.1, 0.02, 0.5 + 0.01 * 0.1 * k) for k in range(300)]
        self.assertFalse(pipeline.transient_settled(drift, 0.0, cfg))


class Classify(unittest.TestCase):
    sc = dict(window=100, min_iter=100, cl_tol=0.005, cd_tol=0.01, mean_after=200, mean_cl_tol=0.005, mean_cd_tol=0.01,
              amp_cl=0.04, amp_cd=0.15)
    tc = dict(window=10.0, min_time=20.0, cl_tol=0.01, cd_tol=0.02, dt=0.005)

    def rows(self, n, cl):
        return [(float(k), -0.1, 0.02, cl(k)) for k in range(n)]

    def test_flat_is_steady_with_values(self):
        r = pipeline.classify_steady(self.rows(300, lambda k: 0.5), False, self.sc)
        self.assertEqual(r["status"], "steady")
        self.assertTrue(r["converged"])
        self.assertAlmostEqual(r["cl"], 0.5)

    def test_small_cycle_is_steady_mean(self):
        r = pipeline.classify_steady(self.rows(300, lambda k: 0.5 + 0.004 * math.sin(k)), False, self.sc)
        self.assertEqual(r["status"], "steady-mean")
        self.assertTrue(r["converged"])
        self.assertTrue(r["steady"]["limit_cycle"])

    def test_big_cycle_keeps_values_but_is_flagged(self):
        r = pipeline.classify_steady(self.rows(300, lambda k: 0.5 + 0.1 * math.sin(k)), False, self.sc)
        self.assertEqual(r["status"], "unconverged-steady")
        self.assertFalse(r["converged"])
        self.assertIsNotNone(r["cl"])

    def test_diverged_and_empty_have_no_values(self):
        self.assertEqual(pipeline.classify_steady(self.rows(300, lambda k: 0.5), True, self.sc)["status"], "diverged")
        self.assertEqual(pipeline.classify_steady([], False, self.sc)["status"], "failed")
        self.assertIsNone(pipeline.classify_steady([], False, self.sc)["cl"])

    def test_transient_settled_and_not(self):
        flat = [(0.1 * k, -0.1, 0.02, 0.5) for k in range(300)]
        r = pipeline.classify_transient(flat, 0.0, False, self.tc)
        self.assertEqual(r["status"], "transient-mean")
        self.assertTrue(r["converged"])
        drift = [(0.1 * k, -0.1, 0.02, 0.5 + 0.01 * k) for k in range(300)]
        r = pipeline.classify_transient(drift, 0.0, False, self.tc)
        self.assertEqual(r["status"], "unconverged-transient")
        self.assertFalse(r["converged"])

    def test_transient_failures(self):
        self.assertEqual(pipeline.classify_transient([], 0.0, True, self.tc)["status"], "diverged")
        self.assertEqual(pipeline.classify_transient([], 0.0, False, self.tc)["status"], "failed")


class Layers(unittest.TestCase):
    def test_stack_fits_the_cell(self):
        import case3d
        n = case3d.layer_count(5e-5, 5e-3, 1.2)
        total = 5e-5 * (1.2 ** n - 1) / 0.2
        self.assertLessEqual(total, 0.8 * 5e-3)
        self.assertGreater(5e-5 * (1.2 ** (n + 1) - 1) / 0.2, 0.8 * 5e-3)

    def test_minimum_three(self):
        import case3d
        self.assertEqual(case3d.layer_count(1e-3, 1e-3, 1.2), 3)

    def test_force_resolution_round_trip(self):
        import case3d
        a = 5.0
        lift, drag = 2.0, 0.3  # whole bird, so the half model carries half
        ar = math.radians(a)
        fx = -0.5 * drag * math.cos(ar) + 0.5 * lift * math.sin(ar)
        fy = 0.5 * drag * math.sin(ar) + 0.5 * lift * math.cos(ar)
        got = case3d.lift_drag(fx, fy, a)
        self.assertAlmostEqual(got[0], lift)
        self.assertAlmostEqual(got[1], drag)


class WarmStart(unittest.TestCase):
    def test_internal_field_replaced_boundary_kept(self):
        with tempfile.TemporaryDirectory() as root:
            new, old = os.path.join(root, "new"), os.path.join(root, "old")
            os.makedirs(os.path.join(new, "0"))
            os.makedirs(os.path.join(old, "final"))
            for name in pipeline.FIELDS:
                with open(os.path.join(new, "0", name), "w") as f:
                    f.write("dimensions [];\ninternalField   uniform 1;\n\nboundaryField\n{\n farfield { type x; value uniform 7; }\n}\n")
                with open(os.path.join(old, "final", name), "w") as f:
                    f.write("dimensions [];\ninternalField   nonuniform List<scalar> 2(3 4);\nboundaryField\n{\n farfield { type y; value uniform 9; }\n}\n")
            pipeline.warm_start(new, old)
            text = open(os.path.join(new, "0", "U")).read()
            self.assertIn("nonuniform List<scalar> 2(3 4);", text)
            self.assertIn("farfield { type x; value uniform 7; }", text)
            self.assertNotIn("uniform 1;", text)


class Resume(unittest.TestCase):
    def make(self, root, done=dict(rc=0, diverged=False), mode="steady"):
        d = os.path.join(root, "p")
        os.makedirs(os.path.join(d, "processor0", "6000"))
        os.makedirs(os.path.join(d, "processor0", "6000.5"))
        os.makedirs(os.path.join(d, "6000"))
        os.makedirs(os.path.join(d, "postProcessing", "forceCoeffs1", "0"))
        os.makedirs(os.path.join(d, "postProcessing", "forceCoeffs1", "6000"))
        json.dump(dict(re=1e5, alpha_deg=2, tu_pct=0.1, mode=mode), open(os.path.join(d, "spec.json"), "w"))
        if done is not None:
            json.dump(done, open(os.path.join(d, "steady.done"), "w"))
        return d

    def test_resumes_and_removes_partial_transient(self):
        with tempfile.TemporaryDirectory() as root:
            d = self.make(root)
            self.assertIsNotNone(pipeline.resume_steady(d, 1e5, 2, 0.1))
            self.assertFalse(os.path.exists(os.path.join(d, "processor0", "6000.5")))
            self.assertTrue(os.path.exists(os.path.join(d, "processor0", "6000")))
            self.assertFalse(os.path.exists(os.path.join(d, "postProcessing", "forceCoeffs1", "6000")))
            self.assertTrue(os.path.exists(os.path.join(d, "postProcessing", "forceCoeffs1", "0")))

    def test_refuses_unfinished_failed_or_other_point(self):
        with tempfile.TemporaryDirectory() as root:
            self.assertIsNone(pipeline.resume_steady(self.make(root, done=None), 1e5, 2, 0.1))
        with tempfile.TemporaryDirectory() as root:
            self.assertIsNone(pipeline.resume_steady(self.make(root, done=dict(rc=13, diverged=False)), 1e5, 2, 0.1))
        with tempfile.TemporaryDirectory() as root:
            self.assertIsNone(pipeline.resume_steady(self.make(root, done=dict(rc=0, diverged=True)), 1e5, 2, 0.1))
        with tempfile.TemporaryDirectory() as root:
            self.assertIsNone(pipeline.resume_steady(self.make(root), 1e5, 4, 0.1))


class Separation(unittest.TestCase):
    def test_bubble_found(self):
        side = [dict(x=0.1 * k, cf=(-1 if 3 <= k <= 5 else 1)) for k in range(10)]
        self.assertEqual(pipeline.separated_regions(side, 1.0), [[0.3, 0.5]])

    def test_open_separation_marked(self):
        side = [dict(x=0.1 * k, cf=(-1 if k >= 7 else 1)) for k in range(10)]
        self.assertEqual(pipeline.separated_regions(side, 1.0), [[0.7, 0.9, "open"]])


class BoundaryParse(unittest.TestCase):
    def test_reads_one_patch(self):
        text = ("boundaryField\n{\n    farfield\n    {\n        type x;\n        value nonuniform List<scalar> 2(9 9);\n    }\n"
                "    airfoil\n    {\n        type y;\n        value nonuniform List<scalar> 3(1 2 3);\n    }\n}\n")
        with tempfile.NamedTemporaryFile("w", suffix=".f", delete=False) as f:
            f.write(text)
        try:
            self.assertEqual(pipeline.boundary_values(f.name, "airfoil"), [1.0, 2.0, 3.0])
        finally:
            os.unlink(f.name)


class StripFit(unittest.TestCase):
    """Recover known polar parameters from synthetic points built by the polars.js formulas."""

    def points(self, a=5.5, a0=math.radians(-3), cd0=0.016, exp=0.5, cl0=0.6, k=0.02, cm=-0.09):
        pts = []
        for re_ in (50000, 100000, 200000):
            for deg in range(-2, 7):  # three Re: the minimum for a drag fit
                cl = a * (math.radians(deg) - a0)
                cd = cd0 * (re_ / 1e5) ** -exp + k * (cl - cl0) ** 2
                pts.append(dict(re=re_, alpha_deg=deg, cl=cl, cd=cd, cm=cm, converged=True))
        return pts

    def test_recovers_parameters(self):
        f = polars.strip_params(self.points(), 0.03)
        self.assertAlmostEqual(f["a"], 5.5, places=6)
        self.assertAlmostEqual(f["alphaL0"], math.radians(-3), places=6)
        self.assertAlmostEqual(f["cd0"], 0.016, places=6)
        self.assertAlmostEqual(f["reExp"], 0.5, places=6)
        self.assertAlmostEqual(f["clDrag"], 0.6, places=6)
        self.assertAlmostEqual(f["kDrag"], 0.02, places=6)
        self.assertAlmostEqual(f["cm0"], -0.09, places=9)

    def test_unconverged_points_are_ignored(self):
        pts = self.points()
        pts.append(dict(re=100000, alpha_deg=2, cl=9.0, cd=9.0, cm=9.0, converged=False))
        self.assertAlmostEqual(polars.strip_params(pts, 0.03)["a"], 5.5, places=6)

    def test_no_converged_points_gives_no_parameters(self):
        pts = [dict(p, converged=False) for p in self.points()]
        self.assertIsNone(polars.strip_params(pts, 0.03))


if __name__ == "__main__":
    unittest.main()
