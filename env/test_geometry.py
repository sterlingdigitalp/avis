"""Invariants of the procedural geometry, checked against hand-computed references. Pure numpy, no Blender.

    out/env/.venv/bin/python -I -m unittest discover -s env -p "test_geometry.py"
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np

import build_terrain as BT
import ground as G
import trees as T


class TipSegments(unittest.TestCase):
    # an L-shaped polyline: 3 m along x, then 2 m along y; tip at (3, 2, 0)
    P = np.array([[[0.0, 0, 0], [3.0, 0, 0], [3.0, 2.0, 0]]])

    def test_keeps_only_the_last_metres(self):
        base, d, ln, dist = T._tip_segments(self.P, 2.5)
        # 0.5 m of the first leg and all 2 m of the second
        np.testing.assert_allclose(ln, [0.5, 2.0])
        np.testing.assert_allclose(base, [[2.5, 0, 0], [3.0, 0, 0]])
        np.testing.assert_allclose(d, [[1, 0, 0], [0, 1, 0]])
        np.testing.assert_allclose(dist, [2.5, 2.0])

    def test_longer_than_the_line_keeps_it_all(self):
        base, d, ln, dist = T._tip_segments(self.P, 9.0)
        np.testing.assert_allclose(ln, [3.0, 2.0])
        np.testing.assert_allclose(dist, [5.0, 2.0])

    def test_inside_the_last_leg_drops_the_first(self):
        base, d, ln, dist = T._tip_segments(self.P, 1.5)
        np.testing.assert_allclose(ln, [1.5])
        np.testing.assert_allclose(base, [[3.0, 0.5, 0]])

    def test_keep_on_a_vertex_returns_no_zero_length_piece(self):
        base, d, ln, dist = T._tip_segments(self.P, 2.0)
        np.testing.assert_allclose(ln, [2.0])
        np.testing.assert_allclose(base, [[3.0, 0, 0]])

    def test_several_lines_of_several_legs(self):
        # two lines, three 1 m legs each along x; the second is shifted in y
        P = np.array([[[0.0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], [[0.0, 5, 0], [1, 5, 0], [2, 5, 0], [3, 5, 0]]])
        base, d, ln, dist = T._tip_segments(P, 1.5)
        np.testing.assert_allclose(ln, [0.5, 1.0, 0.5, 1.0])
        np.testing.assert_allclose(base[:, 0], [1.5, 2.0, 1.5, 2.0])
        np.testing.assert_allclose(base[:, 1], [0, 0, 5, 5])
        np.testing.assert_allclose(dist, [1.5, 1.0, 1.5, 1.0])

    def test_extra_distance_adds_to_path_distance(self):
        _, _, _, dist = T._tip_segments(self.P, 9.0, 0.4)
        np.testing.assert_allclose(dist, [5.4, 2.4])


class Needles(unittest.TestCase):
    def needles_on(self, species, length, lod=0):
        b = T.Builder()
        b.add(np.zeros((1, 3)), np.zeros((0, 3), int), 0)
        sp = T.SPECIES[species]
        T._needles(b, sp, np.zeros((1, 3)), np.array([[1.0, 0, 0]]), np.array([length]), np.array([length]),
                   np.random.default_rng(1), T.LOD[lod])
        return b.arrays(), sp

    def test_count_follows_spacing(self):
        arr, sp = self.needles_on("engelmann", 0.10)
        per_needle = 2  # LOD 0 needles are two-triangle blades
        n = (arr["mat"] == 1).sum() / per_needle
        self.assertAlmostEqual(n, 0.10 / sp["needle_space"], delta=1.0)

    def test_lod1_is_sparser_single_triangles(self):
        arr, sp = self.needles_on("engelmann", 0.10, lod=1)
        n = (arr["mat"] == 1).sum()
        self.assertAlmostEqual(n, 0.10 / sp["needle_space"] * T.LOD[1]["needle_mult"], delta=1.0)

    def test_one_needle_spacing_gives_one_needle(self):
        sp = T.SPECIES["engelmann"]
        b = T.Builder()
        T._needles(b, sp, np.zeros((1, 3)), np.array([[1.0, 0, 0]]), np.array([sp["needle_space"]]), np.array([0.5]),
                   np.random.default_rng(3), T.LOD[0])
        self.assertEqual((b.arrays()["mat"] == 1).sum(), 2)

    def test_lod1_blades_are_wider(self):
        sp = T.SPECIES["engelmann"]
        widths = []
        for lod in (0, 1):
            b = T.Builder()
            T._needles(b, sp, np.zeros((1, 3)), np.array([[1.0, 0, 0]]), np.array([0.3]), np.array([0.5]),
                       np.random.default_rng(4), T.LOD[lod])
            v = b.arrays()["verts"]
            widths.append(np.linalg.norm(v[0] - v[1]))  # base edge of the first blade
        expect = sp["needle_w"] * T.NEEDLE_GAIN * T.LOD[1]["needle_w"]
        self.assertAlmostEqual(widths[1], expect, delta=expect * 0.4)
        self.assertGreater(widths[1], 2.5 * widths[0])

    def test_larch_needles_come_in_tufts_on_six_spurs(self):
        sp = T.SPECIES["larch"]
        b = T.Builder()
        L = 0.2
        T._needles(b, sp, np.zeros((1, 3)), np.array([[1.0, 0, 0]]), np.array([L]), np.array([L]),
                   np.random.default_rng(5), T.LOD[0])
        a = b.arrays()
        n = (a["mat"] == 1).sum() // 2
        per = int(np.floor(L / sp["needle_space"]))
        self.assertIn(n, (per * 18, (per + 1) * 18))
        bases = a["verts"][::4, 0] / L  # first vertex of each blade sits at its base on the x axis (+- half width)
        spur = np.round(bases * 6) / 6
        self.assertLess(np.abs(bases - spur).max(), 0.02)
        self.assertLess(bases.max(), 1.0)   # tufts sit on the shoot, not past its end
        self.assertGreater(len(np.unique(spur)), 3)
        self.assertLessEqual(len(np.unique(spur)), 6)

    def test_needles_shorten_into_the_bud(self):
        sp = T.SPECIES["engelmann"]
        b = T.Builder()
        T._needles(b, sp, np.zeros((1, 3)), np.array([[1.0, 0, 0]]), np.array([0.5]), np.array([0.5]),
                   np.random.default_rng(6), T.LOD[0])
        v = b.arrays()["verts"].reshape(-1, 4, 3)
        base_x = 0.5 * (v[:, 0, 0] + v[:, 1, 0])
        length = np.linalg.norm(0.5 * (v[:, 2] + v[:, 3]) - 0.5 * (v[:, 0] + v[:, 1]), axis=1)
        at_bud = length[base_x > 0.495].mean()
        mature = length[base_x < 0.4].mean()
        self.assertAlmostEqual(at_bud / mature, 0.55 + 0.45 * 0.5 / 4, delta=0.12)
        self.assertAlmostEqual(mature, sp["needle_len"], delta=sp["needle_len"] * 0.1)

    def blades(self, species, lod=0, n=0.5):
        b = T.Builder()
        T._needles(b, T.SPECIES[species], np.zeros((1, 3)), np.array([[1.0, 0, 0]]), np.array([n]), np.array([2.0]),
                   np.random.default_rng(8), T.LOD[lod])
        v = b.arrays()["verts"].reshape(-1, 4 if lod == 0 else 3, 3)
        base = 0.5 * (v[:, 0] + v[:, 1])
        tip = 0.5 * (v[:, 2] + v[:, 3]) if lod == 0 else v[:, 2]
        d = tip - base
        return d / np.linalg.norm(d, axis=1, keepdims=True), np.linalg.norm(d, axis=1)

    def test_needles_point_forward_at_the_species_angle(self):
        for sp in ("engelmann", "lodgepole", "subalpine"):
            d, _ = self.blades(sp)
            ang = np.degrees(np.arccos(np.clip(d[:, 0], -1, 1)))
            self.assertAlmostEqual(np.median(ang), T.SPECIES[sp]["needle_theta"], delta=9, msg=sp)

    def test_spruce_brush_is_top_heavy_and_fir_spray_is_flat(self):
        # shoot along +x; _frames puts 'side' horizontal (y) and 'up' toward +z
        d, _ = self.blades("engelmann")
        self.assertGreater(d[:, 2].mean(), 0.25)                       # more needles on the upper side
        self.assertGreater(np.abs(d[:, 1]).mean(), 0.2)                 # but all round the shoot
        f, _ = self.blades("subalpine")
        self.assertGreater(np.abs(f[:, 1]).mean(), 1.6 * np.abs(f[:, 2]).mean())  # two-ranked, in the spray plane
        p, _ = self.blades("lodgepole")
        self.assertLess(abs(p[:, 2].mean()), 0.12)                      # pine needles radiate evenly

    def test_lod1_needles_are_longer(self):
        _, l0 = self.blades("engelmann", 0)
        _, l1 = self.blades("engelmann", 1)
        self.assertAlmostEqual(np.median(l1) / np.median(l0), T.LOD[1]["needle_l"], delta=0.12)

    def test_needles_stay_near_the_shoot(self):
        arr, sp = self.needles_on("lodgepole", 0.2)
        v = arr["verts"][1:]
        # every vertex lies within one needle length (plus blade width) of the shoot segment x in [0, 0.2]
        dx = np.clip(v[:, 0], 0, 0.2) - v[:, 0]
        off = np.hypot(dx, np.hypot(v[:, 1], v[:, 2]))
        self.assertLess(off.max(), sp["needle_len"] * 1.6 + 0.01)

    def test_age_runs_from_new_at_the_tip_to_old(self):
        b = T.Builder()
        sp = T.SPECIES["engelmann"]
        T._needles(b, sp, np.zeros((1, 3)), np.array([[1.0, 0, 0]]), np.array([0.6]), np.array([0.6]),
                   np.random.default_rng(2), T.LOD[0])
        a = b.arrays()
        x, age = a["verts"][:, 0], a["age"]
        self.assertTrue(x.max() < 0.6 + sp["needle_len"] * 1.3)  # needles sit along the 0.6 m piece, not past it
        self.assertLess(age[x > 0.55].mean(), 0.3)
        self.assertGreater(age[x < 0.05].mean(), 0.9)
        mid = (x > 0.33) & (x < 0.37)  # 0.25 m from the tip: half of AGE_M
        self.assertAlmostEqual(age[mid].mean(), 0.25 / T.AGE_M, delta=0.12)


class Trees(unittest.TestCase):
    def test_spruce_clothes_far_more_wood_than_pine(self):
        """Spruce keeps needles on every axis for years; lodgepole only in tufts at the branch ends."""
        a = T.build_tree("lodgepole", 5.0, 0.92, 35, 0)
        needles = a["mat"] == 1
        self.assertGreater(needles.sum(), 1000)
        spruce = T.build_tree("engelmann", 6.0, 1.0, 15, 0)
        # spruce clothes far more of its wood than pine for a similar tree
        self.assertGreater((spruce["mat"] == 1).sum(), 3 * needles.sum())

    def test_cache_key_isolates_species(self):
        k_fir = T.cache_key("subalpine", 0)
        k_eng = T.cache_key("engelmann", 0)
        old = T.SPECIES["engelmann"]["needle_len"]
        try:
            T.SPECIES["engelmann"]["needle_len"] = old * 1.1
            self.assertNotEqual(T.cache_key("engelmann", 0), k_eng)
            self.assertEqual(T.cache_key("subalpine", 0), k_fir)
        finally:
            T.SPECIES["engelmann"]["needle_len"] = old
        self.assertNotEqual(T.cache_key("engelmann", 1), k_eng)


class Ground(unittest.TestCase):
    def test_patches_stay_inside_their_tile(self):
        # blade tips arch up to ~0.6 m past the tile edge (that overlap is what hides tile seams); cobbles reach
        # at most one clast length
        for kind, seed, half, reach in (("meadow", 94, G.MEADOW_PATCH / 2, 0.62), ("cobbles", 97, G.COBBLE_PATCH / 2, 0.25)):
            v = G.build_under(kind, seed)["verts"]
            self.assertLess(np.abs(v[:, :2]).max(), half + reach, kind)
            self.assertGreater(v[:, 2].max(), 0.0, kind)

    def test_meadow_is_continuous_cover(self):
        v = G.build_under("meadow", 94)["verts"]
        # blade bases cover every 0.25 m cell of the tile
        cells = np.unique(np.floor((v[:, :2] + G.MEADOW_PATCH / 2) / 0.25).astype(int), axis=0)
        self.assertGreaterEqual(len(cells), (G.MEADOW_PATCH / 0.25) ** 2 * 0.95)

    def test_cobbles_are_flattened_and_half_buried(self):
        a = G.build_under("cobbles", 98)
        v = a["verts"]
        self.assertLess(v[:, 2].max(), 0.22)  # largest clast 0.22 m long, flattened to at most 0.65 of that
        self.assertLess(v[:, 2].min(), 0.0)   # partly bedded in the gravel

    def test_cobble_clasts_are_discs_with_one_lithology_each(self):
        a = G.build_under("cobbles", 99)
        per = 162  # icosphere, two subdivisions
        self.assertEqual(len(a["verts"]) % per, 0)
        n = len(a["verts"]) // per
        self.assertGreaterEqual(n, 120)
        self.assertLess(n, 170)
        stones = a["verts"].reshape(n, per, 3)
        ext = stones.max(1) - stones.min(1)
        flat = ext[:, 2] / np.maximum(ext[:, 0], ext[:, 1])
        self.assertLess(np.median(flat), 0.75)
        self.assertGreater(np.median(flat), 0.3)
        self.assertLess(np.median(np.maximum(ext[:, 0], ext[:, 1])), 0.12)  # mostly pebbles and small cobbles
        ages = a["age"].reshape(n, per)
        np.testing.assert_array_equal(ages.min(1), ages.max(1))
        self.assertGreater(ages[:, 0].std(), 0.2)
        self.assertGreater((stones[:, :, 2].min(1) < 0).mean(), 0.8)  # bedded in the gravel

    def test_sphere_is_unit_and_closed(self):
        v, f = G._sphere(*G.ICOSAHEDRON, 2)
        np.testing.assert_allclose(np.linalg.norm(v, axis=1), 1.0, atol=1e-12)
        self.assertEqual(len(v) - 3 * len(f) // 2 + len(f), 2)  # Euler characteristic of a sphere
        self.assertEqual(len(f), 20 * 4 ** 2)

    def test_every_kind_builds(self):
        for name, kind, seed in G.UNDER:
            self.assertGreater(len(G.build_under(kind, seed)["faces"]), 0, name)


class Terrain(unittest.TestCase):
    def test_bands_are_zero_mean_and_bounded(self):
        xs = np.arange(0, 4000.0, 8.0)
        xx, yy = np.meshgrid(xs, xs)
        z = 0.8 * xx  # a steep plane
        out = BT.rock_bands(xx, yy, z, np.ones_like(z), [(95.0, 30.0, 0.55)])
        d = out - z
        self.assertLess(abs(d.mean()), 1.5)
        self.assertLessEqual(np.abs(d).max(), 30.0 * 1.4 * 0.75 + 1e-6)
        self.assertGreater(d.std(), 3.0)

    def test_risers_take_their_share_of_each_bed(self):
        # Walk across the beds on a flat plane. The profile is smoothstep(t / r) - t, so it climbs where
        # 6u(1-u) / r > 1, i.e. over a fraction r * sqrt(1 - 2r/3) of every bed.
        xs = np.arange(0, 6000.0, 2.0)
        xx, yy = np.meshgrid(xs, xs[:3])
        z = np.zeros_like(xx)
        for riser in (0.3, 0.6):
            out = BT.rock_bands(xx, yy, z, np.ones_like(z), [(95.0, 30.0, riser)], dipdir=270.0)
            climbing = np.diff(out[1]) < 0  # dip to the west: the bedding coordinate falls as x grows
            self.assertAlmostEqual(climbing.mean(), riser * np.sqrt(1 - 2 * riser / 3), delta=0.04)

    def test_bands_strike_along_the_dip_normal(self):
        xs = np.arange(0, 3000.0, 4.0)
        xx, yy = np.meshgrid(xs, xs)  # rows run north here
        out = BT.rock_bands(xx, yy, np.zeros_like(xx), np.ones_like(xx), [(60.0, 20.0, 0.5)])
        # dip direction 225 deg: beds change along (1, 1) and run along strike (-1, 1)
        d_dip = np.abs(out[1:, 1:] - out[:-1, :-1]).mean()      # step +x, +y
        d_strike = np.abs(out[1:, :-1] - out[:-1, 1:]).mean()   # step -x, +y
        self.assertLess(d_strike, 0.35 * d_dip)

    def test_sharpen_is_identity_without_rock_and_lifts_only_short_waves(self):
        # a Gaussian of sigma s passes a sine of wavelength w with gain exp(-2 pi^2 s^2 / w^2), so the unsharp mask
        # multiplies a short wave by about (1 + gain) and leaves a long wave nearly untouched
        x = np.arange(0, 4096.0, 8.0)
        for w, expect in ((64.0, 1 + 1.0 * (1 - np.exp(-2 * np.pi ** 2 * 40 ** 2 / 64 ** 2))),
                          (2048.0, 1 + 1.0 * (1 - np.exp(-2 * np.pi ** 2 * 40 ** 2 / 2048 ** 2)))):
            z = np.tile(10 * np.sin(2 * np.pi * x / w), (8, 1))
            np.testing.assert_array_equal(BT.sharpen(z, 8.0, np.zeros_like(z), 40.0, 1.0), z)
            out = BT.sharpen(z, 8.0, np.ones_like(z), 40.0, 1.0)
            mid = slice(64, -64)
            self.assertAlmostEqual(np.abs(out[4, mid]).max() / 10.0, expect, delta=0.02)

    def test_flutes_run_down_the_fall_line(self):
        xs = np.arange(0, 1600.0, 8.0)
        xx, yy = np.meshgrid(xs, xs)
        z = 0.9 * xx  # falls toward -x
        np.testing.assert_array_equal(BT.fall_line_flutes(z, 8.0, np.zeros_like(z), 5.0, 110.0, 3), z)
        d = BT.fall_line_flutes(z, 8.0, np.ones_like(z), 5.0, 110.0, 3) - z
        self.assertAlmostEqual(d.std(), 5.0, delta=0.5)
        along = np.corrcoef(d[:, :-3].ravel(), d[:, 3:].ravel())[0, 1]   # 24 m down the fall line
        across = np.corrcoef(d[:-3].ravel(), d[3:].ravel())[0, 1]        # 24 m along the contour
        self.assertGreater(along, 0.8)
        self.assertLess(across, 0.3)

    def test_bands_respect_the_mask(self):
        xs = np.arange(0, 800.0, 8.0)
        xx, yy = np.meshgrid(xs, xs)
        z = 0.8 * xx
        out = BT.rock_bands(xx, yy, z, np.zeros_like(z), [(95.0, 30.0, 0.55)])
        np.testing.assert_array_equal(out, z)

    def test_shore_map_encodes_metres(self):
        w = np.zeros((40, 40), bool)
        w[:, :10] = True
        with tempfile.TemporaryDirectory() as d:
            old = BT.OUT
            BT.OUT = d
            try:
                BT.shore_map(w, np.ones_like(w), 2.0)
                from PIL import Image
                im = np.asarray(Image.open(os.path.join(d, "shore_l0.png")))
            finally:
                BT.OUT = old
        metres = im[..., 0].astype(float) / 255 * BT.C.SHORE_RANGE
        self.assertEqual(metres[5, 5], 0.0)                        # in the water
        self.assertAlmostEqual(metres[5, 14], 10.0, delta=0.15)   # 5 cells of 2 m from the waterline
        self.assertEqual(im[..., 1].min(), 255)


if __name__ == "__main__":
    unittest.main()
