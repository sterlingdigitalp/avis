"""Tests for the suction, 3D report and loft code, on fixtures. Expected values come from constructions with a known
answer, not from these modules' own output."""
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import bird3d
import report3d
import stl_bird
import suction
import suction_kin


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


def forces_file(path, rows):
    write(path, "# Time forces\n" + "".join(f"{t:.6f}\t(({fx:.8e} {fy:.8e} 0) (0 0 0)) ((0 0 0) (0 0 0))\n"
                                            for t, fx, fy in rows))


class Loft(unittest.TestCase):
    def mesh(self, last_open):
        n = 8
        secs = [(0.3, 0.0), (0.2, 0.04), (0.0, 0.05), (-0.3, 0.02 if last_open else 0.0)]
        pts = [[x, 0.5 * s * math.cos(2 * math.pi * k / n), 0.5 * s * math.sin(2 * math.pi * k / n)]
               for x, s in secs for k in range(n)]
        return dict(rings=len(secs), nAround=n, points=pts)

    def closed(self, tris):
        edges = {}
        for t in tris:
            for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
                key = tuple(sorted((tuple(a), tuple(b))))
                edges[key] = edges.get(key, 0) + 1
        return all(v == 2 for v in edges.values())

    def test_apex_both_ends_closed(self):
        self.assertTrue(self.closed(stl_bird.body_from_loft(self.mesh(False))))

    def test_open_end_capped(self):
        self.assertTrue(self.closed(stl_bird.body_from_loft(self.mesh(True))))


class Poller(unittest.TestCase):
    def test_nan_kills_and_flat_stops(self):
        with tempfile.TemporaryDirectory() as d:
            poll, state = bird3d.poller(d, 3.0)
            self.assertIsNone(poll())
            forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"),
                        [(k, -0.1, 1.0) for k in range(5, 2005, 5)])
            self.assertEqual(poll(), "stop")
            forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"), [(5, float("nan"), 1.0)])
            self.assertEqual(poll(), "kill")
            self.assertTrue(state["diverged"])

    def test_drifting_lift_runs_on(self):
        with tempfile.TemporaryDirectory() as d:
            poll, _ = bird3d.poller(d, 3.0)
            forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"),
                        [(k, -0.1, 1.0 + 0.001 * k) for k in range(5, 2005, 5)])
            self.assertIsNone(poll())


class Report3d(unittest.TestCase):
    def test_report_written_from_result(self):
        with tempfile.TemporaryDirectory() as root:
            old = report3d.ROOT
            report3d.ROOT = root
            try:
                res = dict(spec=dict(tu_pct=0.1, surface_cell=0.004, n_layers=14, first_layer=5e-5, ratio=1.2),
                           pose=dict(V=9.5, aoa_deg=3.0, phi_deg=5.0, glide=dict(tail=0.01)),
                           atmosphere=dict(rho=1.0, mu=1.8e-5), mesh=dict(cells="100", mesh_ok=True, non_ortho_max="60",
                                                                          skewness_max="2"),
                           layers=dict(wing=dict(faces=10, layers_mean=5.0, thickness_m=0.002, thickness_pct=80.0)),
                           yplus=dict(wing=dict(min=0.1, mean=0.5, max=1.5)), iterations=1500.0, converged=True,
                           lift_n=4.2, drag_n=0.31, L_over_D=13.5, weight_n=4.0, CL=0.5, CD=0.04, wing_area_m2=0.46,
                           viscous_drag_share=0.4, lift_range_n=[4.1, 4.3], drag_range_n=[0.3, 0.32])
                write(os.path.join(root, "out/cfd/bird/final/result.json"), json.dumps(res))
                sys.argv = ["report3d.py", "bird", "final"]
                report3d.main()
                text = open(os.path.join(root, "out/cfd/bird/report.md")).read()
            finally:
                report3d.ROOT = old
            self.assertIn("Lift 4.200 N (weight 4.000 N)", text)
            self.assertIn("| wing | 10 | 5.0 | 2.00 mm | 80.0 |", text)
            self.assertIn("settled", text)


class KinematicsEta(unittest.TestCase):
    def motion(self):
        # One cycle of a heaving, pitching section with a known relative wind history.
        n, T = 120, 0.5
        samples = []
        for i in range(n):
            ph = 2 * math.pi * i / n
            u, w = 10.0, 1.5 * math.sin(ph)
            samples.append(dict(t=i * T / n, theta=0.1 * math.sin(ph + 0.5), u=u, w=w, alpha_eff=math.atan2(w, u)))
        return dict(samples=samples, period_s=T, chord_m=0.25, pitch_axis_x_over_c=0.25, re_chord=1.4e5)

    def test_eta_recovered_from_synthetic_forces(self):
        m = self.motion()
        tables = suction_kin.motion_tables(m)
        spec = dict(period=tables["period"])
        th_mean = sum(p["theta"] for p in m["samples"]) / len(m["samples"])
        eta, cdf, span = 0.3, 0.012, 0.1
        rows = []
        for i in range(0, 3 * 400 + 1):
            t = 3 * spec["period"] * i / 1200
            ph = (t / spec["period"]) % 1.0 * len(m["samples"])
            k = int(ph) % len(m["samples"])
            f = ph - int(ph)
            nx = m["samples"][(k + 1) % len(m["samples"])]
            lerp = lambda key: m["samples"][k][key] + f * (nx[key] - m["samples"][k][key])
            a = math.atan2(lerp("w"), lerp("u"))
            psi = -(lerp("theta") - th_mean)
            cl = 2 * math.pi * math.sin(a) + 0.4
            cc = eta * cl * math.sin(a) - cdf * math.cos(a)
            ax = -cc
            ay = (cl + ax * math.sin(a)) / math.cos(a)
            cx = ax * math.cos(psi) - ay * math.sin(psi)
            cy = ax * math.sin(psi) + ay * math.cos(psi)
            rows.append((t, cx * 0.5 * span, cy * 0.5 * span))
        with tempfile.TemporaryDirectory() as d:
            forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"), rows)
            got = suction_kin.evaluate(d, spec, m, cdf)
        self.assertTrue(got["complete"])
        self.assertEqual(got["cycles_averaged"], 2)
        self.assertAlmostEqual(got["eta_s_strip"], eta, places=3)

    def test_tables_close_and_freestream_is_the_mean(self):
        t = suction_kin.motion_tables(self.motion())
        self.assertAlmostEqual(t["disp"][-1][0], t["disp"][0][0], places=12)
        self.assertAlmostEqual(t["disp"][-1][1], t["disp"][0][1], places=12)
        self.assertAlmostEqual(t["psi"][-1], t["psi"][0], places=12)
        self.assertGreater(t["Umag"], 9.9)


class MotionConstruction(unittest.TestCase):
    """The tables must reproduce the section's relative wind exactly: R(psi) q + v = U at every sample."""

    def motion(self, w_amp=2.0, th_amp=0.0, u=10.0, n=200, T=0.4, c=0.25):
        s = []
        for i in range(n):
            ph = 2 * math.pi * i / n
            w = w_amp * math.sin(ph)
            s.append(dict(t=i * T / n, theta=0.05 + th_amp * math.sin(ph + 0.7), u=u, w=w, alpha_eff=math.atan2(w, u)))
        return dict(samples=s, period_s=T, chord_m=c)

    def test_pure_heave_closed_form(self):
        m = self.motion()
        tb = suction_kin.motion_tables(m)
        self.assertAlmostEqual(tb["Umag"], 10.0, places=9)
        self.assertAlmostEqual(tb["alpha_inf"], 0.0, places=9)
        self.assertTrue(all(abs(p) < 1e-12 for p in tb["psi"]))
        self.assertAlmostEqual(tb["period"], 0.4 * 10.0 / 0.25, places=9)
        # Mesh moves with -w, so y = -(1/c) * integral of w dt = -(A T / 2 pi c) (1 - cos(2 pi t / T)).
        T, A, c = 0.4, 2.0, 0.25
        for k in (50, 100, 150):
            t = k * T / 200
            expect = -(A * T / (2 * math.pi * c)) * (1 - math.cos(2 * math.pi * t / T))
            self.assertAlmostEqual(tb["disp"][k][1], expect, delta=2e-4)
            self.assertAlmostEqual(tb["t"][k], t * 10.0 / c, places=9)
        self.assertAlmostEqual(tb["mean_speed"], sum(math.hypot(10, p["w"]) for p in m["samples"]) / 200 / 10, places=9)
        self.assertEqual(len(tb["t"]), 201)
        self.assertEqual(len(tb["disp"]), 201)
        self.assertAlmostEqual(tb["t"][-1], tb["period"], places=9)

    def test_identity_with_pitch(self):
        m = self.motion(w_amp=1.5, th_amp=0.2)
        tb = suction_kin.motion_tables(m)
        n = len(m["samples"])
        th_mean = sum(p["theta"] for p in m["samples"]) / n
        U = tb["U"]
        c, T = m["chord_m"], m["period_s"]
        for k in range(0, n, 17):
            p = m["samples"][k]
            psi = tb["psi"][k]
            self.assertAlmostEqual(psi, -(p["theta"] - th_mean), places=12)  # nose up is clockwise here
            rx = p["u"] * math.cos(psi) - p["w"] * math.sin(psi)
            ry = p["u"] * math.sin(psi) + p["w"] * math.cos(psi)
            # mesh velocity from the displacement table, central difference, back to m/s
            dt = (tb["t"][k + 1] - tb["t"][k - 1]) if k > 0 else None
            if dt:
                vx = (tb["disp"][k + 1][0] - tb["disp"][k - 1][0]) / dt * tb["Umag"]
                vy = (tb["disp"][k + 1][1] - tb["disp"][k - 1][1]) / dt * tb["Umag"]
                self.assertAlmostEqual(rx + vx, U[0], delta=0.02)
                self.assertAlmostEqual(ry + vy, U[1], delta=0.02)
        mean_rel = (sum(p["u"] * math.cos(a) - p["w"] * math.sin(a) for p, a in zip(m["samples"], tb["psi"])) / n,
                    sum(p["u"] * math.sin(a) + p["w"] * math.cos(a) for p, a in zip(m["samples"], tb["psi"])) / n)
        self.assertAlmostEqual(U[0], mean_rel[0], places=9)
        self.assertAlmostEqual(U[1], mean_rel[1], places=9)
        self.assertAlmostEqual(tb["alpha_inf"], math.atan2(U[1], U[0]), places=12)


class EvaluatePaths(unittest.TestCase):
    def setUp(self):
        self.m = KinematicsEta().motion()
        self.spec = dict(period=suction_kin.motion_tables(self.m)["period"])

    def rows_const(self, cycles, cc=0.02, cl=0.6):
        # Section-frame forces held constant; psi rotates them into the CFD frame.
        th_mean = sum(p["theta"] for p in self.m["samples"]) / len(self.m["samples"])
        rows = []
        n = len(self.m["samples"])
        for i in range(1, int(cycles * 300) + 1):
            t = self.spec["period"] * i / 300
            ph = (t / self.spec["period"]) % 1.0 * n
            k = int(ph) % n
            f = ph - int(ph)
            nx = self.m["samples"][(k + 1) % n]
            lerp = lambda key: self.m["samples"][k][key] + f * (nx[key] - self.m["samples"][k][key])
            a = math.atan2(lerp("w"), lerp("u"))
            psi = -(lerp("theta") - th_mean)
            ax = -cc
            ay = (cl + ax * math.sin(a)) / math.cos(a)
            rows.append((t, (ax * math.cos(psi) - ay * math.sin(psi)) * 0.05, (ax * math.sin(psi) + ay * math.cos(psi)) * 0.05))
        return rows

    def test_means_and_window(self):
        with tempfile.TemporaryDirectory() as d:
            forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"), self.rows_const(3))
            got = suction_kin.evaluate(d, self.spec, self.m, 0.01)
        self.assertAlmostEqual(got["cc_mean"], 0.02, places=6)
        self.assertAlmostEqual(got["cl_mean"], 0.6, places=4)
        self.assertEqual(got["cycles_averaged"], 2)
        self.assertAlmostEqual(got["averaged_from_t"], self.spec["period"], delta=self.spec["period"] / 300 + 1e-9)
        self.assertFalse(got["partial"])

    def test_window_is_the_cycles_after_the_first(self):
        # cc 0.5 in the start-up cycle, 0.02 afterwards: only the later cycles may enter the mean.
        rows = self.rows_const(3)
        big = self.rows_const(3, cc=0.5)
        rows = [b if r[0] < self.spec["period"] - 1e-9 else r for r, b in zip(rows, big)]
        with tempfile.TemporaryDirectory() as d:
            forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"), rows)
            got = suction_kin.evaluate(d, self.spec, self.m, 0.01)
        self.assertAlmostEqual(got["cc_mean"], 0.02, places=4)
        self.assertAlmostEqual(got["cycles_simulated"], 3.0, places=3)
        self.assertAlmostEqual(got["averaged_to_t"], 3 * self.spec["period"], places=6)
        # Constant CL over a symmetric alpha history: the mean of CL sin(a) is zero.
        self.assertAlmostEqual(got["mean_cl_sin_a"], 0.0, places=5)
        self.assertEqual(len(got["eta_s_per_cycle"]), 2)

    def test_partial_run_is_flagged(self):
        with tempfile.TemporaryDirectory() as d:
            forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"), self.rows_const(1.5))
            got = suction_kin.evaluate(d, self.spec, self.m, 0.01)
        self.assertTrue(got["partial"])
        self.assertFalse(got["complete"])
        self.assertEqual(got["cycles_averaged"], 0)
        self.assertAlmostEqual(got["cc_mean"], 0.02, places=6)

    def test_no_forces(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(suction_kin.evaluate(d, self.spec, self.m, 0.01), dict(complete=False, cycles_simulated=0))


class History(unittest.TestCase):
    def test_decimation(self):
        rows = [(0.01 * i, float(i)) for i in range(1000)]  # 10 time units, period 2: 5 cycles
        h = suction.decimated_history(rows, 2.0, per_cycle=50)
        self.assertEqual(len(h["cc"]), len(h["t_over_period"]))
        self.assertTrue(240 <= len(h["cc"]) <= 260)
        self.assertEqual(h["t_over_period"][0], 0.0)
        self.assertEqual(h["cc"][1], 4.0)
        self.assertIsNone(suction.decimated_history([], 2.0))


class FrictionInterp(unittest.TestCase):
    def test_log_re_interpolation(self):
        with tempfile.TemporaryDirectory() as root:
            old = suction.ROOT
            suction.ROOT = root
            try:
                for re_, cd in ((100000, 0.016), (200000, 0.010)):
                    write(os.path.join(root, f"out/cfd/runs/v7/e387-L1-Re{re_}-a0-Tu0.1/result.json"),
                          json.dumps(dict(converged=True, cd=cd, cl=0.3)))
                mid, _ = suction.friction_drag_at(math.sqrt(2) * 1e5)
                low, _ = suction.friction_drag_at(5e4)
            finally:
                suction.ROOT = old
        self.assertAlmostEqual(mid, math.sqrt(0.016 * 0.010), places=9)
        self.assertAlmostEqual(low, 0.016, places=12)


class SuctionFile(unittest.TestCase):
    def test_write_keeps_every_point(self):
        with tempfile.TemporaryDirectory() as root:
            old = suction.ROOT
            suction.ROOT = root
            try:
                # Steady references and friction drag from two V7-like points.
                for name, cl, cd, conv in (("e387-L1-Re100000-a4-Tu0.1", 0.76, 0.023, True),
                                           ("e387-L1-Re100000-a0-Tu0.1", 0.36, 0.016, True),
                                           ("e387-L1-Re100000-a-2-Tu0.1", 0.16, 0.012, False)):
                    write(os.path.join(root, "out/cfd/runs/v7", name, "result.json"),
                          json.dumps(dict(status="steady" if conv else "unconverged-steady", converged=conv, cl=cl, cd=cd)))
                # The coarse grid's own steady reference comes from the grid-study group.
                write(os.path.join(root, "out/cfd/runs/v7-grid/e387-L0.667-Re100000-a4-Tu0.1/result.json"),
                      json.dumps(dict(status="steady", converged=True, cl=0.76, cd=0.023)))
                k, h = 0.35, 0.75
                period = math.pi / k
                for level, n_cycles in ((0.667, 3.0), (1.0, 0.5)):
                    name = f"e387-L{level:g}-Re100000-k{k:g}-h{h:g}-a4"
                    d = os.path.join(root, "out/cfd/runs/suction", name)
                    spec = dict(re=100000, alpha_deg=4.0, k=k, h_over_c=h, omega=2 * k, period=period)
                    rows = [(period * n_cycles * i / 600, -0.05 * 0.05, 0.0) for i in range(1, 601)]
                    forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"), rows)
                    if n_cycles >= 3:
                        write(os.path.join(d, "result.json"), json.dumps(dict(spec, diverged=False, complete=True,
                              ideal_suction=suction.ideal_suction(k, h), theodorsen=suction.theodorsen(k))))
                    else:
                        write(os.path.join(d, "spec.json"), json.dumps(spec))
                suction.write()
                out = json.load(open(os.path.join(root, "data/calibration/suction.json")))
            finally:
                suction.ROOT = old
        self.assertEqual(len(out["points"]), 2)
        full = next(p for p in out["points"] if p["level"] == 0.667)
        part = next(p for p in out["points"] if p["level"] == 1.0)
        self.assertTrue(full["complete"])
        self.assertAlmostEqual(full["cc_mean"], 0.05, places=6)
        # Primary eta: (0.05 - (0.76 sin 4 - 0.023 cos 4)) / Garrick ideal suction.
        a = math.radians(4)
        expect = (0.05 - (0.76 * math.sin(a) - 0.023 * math.cos(a))) / suction.ideal_suction(k, h)
        self.assertAlmostEqual(full["eta_s"], expect, places=6)
        self.assertEqual(full["strip_detail"]["cd_f"], 0.016)  # lowest converged Cd, the unconverged one ignored
        self.assertFalse(part["complete"])
        self.assertTrue(part["stopped_early"])
        self.assertEqual(len(out["grid_sensitivity"]), 1)
        g = out["grid_sensitivity"][0]
        self.assertAlmostEqual(g["difference_rel"], 0.0, places=6)
        self.assertEqual((g["coarse"], g["fine"]), (full["name"], part["name"]))
        self.assertAlmostEqual(g["window_cycles"], 0.5, places=2)
        self.assertIn("start-up", g["note"])
        self.assertAlmostEqual(full["alpha_eff_amplitude_deg"], math.degrees(math.atan(2 * k * h)), places=9)



if __name__ == "__main__":
    unittest.main()


class GateMain(unittest.TestCase):
    def test_writes_gate_and_local_report(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location("v7m", os.path.join(os.path.dirname(HERE), "gates", "v7.py"))
        g = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(g)
        with tempfile.TemporaryDirectory() as root:
            g.ROOT, g.RUNS = root, os.path.join(root, "out/cfd/runs")
            rows = [[-3.0, 0.0, 0.02], [11.0, 1.4, 0.02]]
            ref = {"uiuc_lsat_vol5": {"citation": "test", "re_100000": {"rows": rows}, "re_200000": {"rows": rows}},
                   "nasa_tm_4062": {"re_100000": {"columns": ["alpha_deg", "cl", "cd", "cm"], "rows": [[-3.0, 0.0, 0.02, 0], [11.0, 1.4, None, 0]]},
                                    "re_200000": {"columns": ["alpha_deg", "cl", "cd", "cm"], "rows": [[-3.0, 0.0, 0.02, 0], [11.0, 1.4, 0.02, 0]]}}}
            write(os.path.join(root, "data/references/airfoil-e387.json"), json.dumps(ref))
            own = {"polars": {r: {"rows": [dict(alpha_deg=x[0], cl=x[1], cd=x[2]) for x in rows]} for r in ("100000", "200000")},
                   "freestream_turbulence": {}}
            write(os.path.join(root, "out/cfd/ref/uiuc-e387e.json"), json.dumps(own))
            for re_ in g.RES:
                for a in g.ALPHAS:
                    write(os.path.join(root, f"out/cfd/runs/v7/e387-L1-Re{re_}-a{a}-Tu0.1/result.json"),
                          json.dumps(dict(status="steady", converged=True, cl=0.1 * (a + 3), cd=0.02, yplus_max=0.9,
                                          tu_measured_pct=0.1)))
            sys.argv = ["v7.py"]
            self.assertEqual(g.main(), 0)
            gate = json.load(open(os.path.join(root, "out/verification/V7.json")))
            report = json.load(open(os.path.join(root, "out/cfd/v7/report.json")))
        self.assertTrue(gate["passed"])
        self.assertEqual(gate["measured"]["points"], 26)
        self.assertNotIn("cl_ref", json.dumps(gate))  # reference values stay in the local report only
        self.assertIn("cl_ref", json.dumps(report))


class LastCycles(unittest.TestCase):
    def history(self, etas):
        k = KinematicsEta()
        m = k.motion()
        spec = dict(period=suction_kin.motion_tables(m)["period"])
        th_mean = sum(p["theta"] for p in m["samples"]) / len(m["samples"])
        n = len(m["samples"])
        rows = []
        for i in range(1, len(etas) * 300 + 1):
            t = spec["period"] * i / 300
            cyc = min(len(etas) - 1, int(t / spec["period"] - 1e-9))
            ph = (t / spec["period"]) % 1.0 * n
            j = int(ph) % n
            f = ph - int(ph)
            nx = m["samples"][(j + 1) % n]
            lerp = lambda key: m["samples"][j][key] + f * (nx[key] - m["samples"][j][key])
            a = math.atan2(lerp("w"), lerp("u"))
            psi = -(lerp("theta") - th_mean)
            cl = 2 * math.pi * math.sin(a) + 0.4
            ax = -(etas[cyc] * cl * math.sin(a) - 0.012 * math.cos(a))
            ay = (cl + ax * math.sin(a)) / math.cos(a)
            rows.append((t, (ax * math.cos(psi) - ay * math.sin(psi)) * 0.05, (ax * math.sin(psi) + ay * math.cos(psi)) * 0.05))
        return m, spec, rows

    def run_eval(self, etas):
        m, spec, rows = self.history(etas)
        with tempfile.TemporaryDirectory() as d:
            forces_file(os.path.join(d, "postProcessing/forces1/0/forces.dat"), rows)
            return suction_kin.evaluate_last(d, spec, m, 0.012)

    def test_settled_when_last_three_agree(self):
        r = self.run_eval([0.9, 0.1, 0.8, 0.50, 0.53, 0.47])
        self.assertEqual(r["averaged_cycles"], [4, 5, 6])
        self.assertEqual([round(p["eta_s"], 3) for p in r["per_cycle"]], [0.5, 0.53, 0.47])
        self.assertTrue(r["settled"])
        self.assertTrue(r["complete"])

    def test_unsettled_when_spread(self):
        r = self.run_eval([0.5, 0.5, 0.5, 0.40, 0.60, 0.50])
        self.assertFalse(r["settled"])

    def test_too_few_cycles_not_complete(self):
        r = self.run_eval([0.5, 0.5, 0.5, 0.5])
        self.assertFalse(r["complete"])
        self.assertEqual(r["averaged_cycles"], [2, 3, 4])


class Extendable(unittest.TestCase):
    def test_keep_restart_and_prepare_extend(self):
        import pipeline
        with tempfile.TemporaryDirectory() as d:
            for p in ("processor0", "processor1"):
                for t in ("0", "32.4", "48.6"):
                    os.makedirs(os.path.join(d, p, t))
                os.makedirs(os.path.join(d, p, "constant", "polyMesh"))
            os.makedirs(os.path.join(d, "48.6"))
            os.makedirs(os.path.join(d, "system"))
            write(os.path.join(d, "system", "controlDict"), "application foamRun;\\nendTime 48.6;\\nwriteInterval 48.6;\\npurgeWrite 1;\\n")
            write(os.path.join(d, "spec.json"), json.dumps(dict(period=16.2)))
            write(os.path.join(d, "stop"), "")
            pipeline.clean_keep_restart(d)
            self.assertEqual(sorted(os.listdir(os.path.join(d, "processor0"))), ["0", "48.6", "constant"])
            self.assertFalse(os.path.exists(os.path.join(d, "48.6")))
            t0, end = suction_kin.prepare_extend(d, 3, 3)
            ctl = open(os.path.join(d, "system", "controlDict")).read()
            self.assertEqual(t0, 48.6)
            self.assertAlmostEqual(end, 48.6 + 3 * 16.2)
            self.assertIn(f"endTime {end:.10g};", ctl)
            self.assertAlmostEqual((end / 16.2) % 1.0, 0.0, places=9)
            self.assertIn("writeInterval 16.2;", ctl)
            self.assertIn("purgeWrite 2;", ctl)
            self.assertIn("startFrom latestTime;", ctl)
            self.assertFalse(os.path.exists(os.path.join(d, "stop")))
            self.assertIn("foamRun -parallel", open(os.path.join(d, "resume.sh")).read())
            self.assertNotIn("decomposePar", open(os.path.join(d, "resume.sh")).read())
            self.assertEqual(json.load(open(os.path.join(d, "spec.json")))["extensions"][0]["from_t"], 48.6)

    def test_extend_refuses_without_restart(self):
        with tempfile.TemporaryDirectory() as d:
            write(os.path.join(d, "spec.json"), json.dumps(dict(period=1.0)))
            with self.assertRaises(RuntimeError):
                suction_kin.prepare_extend(d, 2, 3)

    def test_scripts_still_format(self):
        import pipeline
        self.assertIn("ckpt;", pipeline.STEADY_SCRIPT.format(n=3))
        self.assertIn("foamRun", suction_kin.RESUME_SCRIPT.format(n=3))
