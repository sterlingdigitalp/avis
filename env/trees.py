"""Procedural conifer generator for the Avis Rockies valley. Pure numpy, runs in Blender or plain Python.

Every tree is a function of (species, height, crown ratio, seed, lod) and returns triangle arrays:
    verts (V,3) float32, faces (F,3) uint32, mat (F) uint8 [0 bark, 1 needles, 2 dead wood],
    buv (V,2) float32 bark uv in metres, age (V) float32 [0 new growth .. 1 old inner needles]

Species parameters follow the forms in data/references/rockies.md and common field-guide proportions: spire crowns on subalpine fir
and Engelmann spruce, short high crowns on lodgepole pine, broader crowns on Douglas fir.

LOD 0 real needles for 5-40 m from camera; LOD 1 sparse enlarged needles; LOD 2 one spray per secondary
branch; LOD 3 one or two sprays per primary branch.
"""
import numpy as np

D2R = np.pi / 180.0

SPECIES = {
    # lmax: longest primary branch / H. crown: profile exponents. Lengths in metres.
    # fol_depth: only secondaries within this distance of the primary's tip carry foliage; the inner crown of a
    #   stand-grown conifer is shaded out and holds bare dead twigs.
    # ndl_prim / ndl_sec / ndl_shoot: metres back from each axis tip that still carry needles. Spruce and fir keep
    #   needles 7-10 years, so whole sprays are clothed; lodgepole keeps 3-5 years of short shoots, so its foliage
    #   is a tuft at every branch end with bare wood behind it.
    # needle_w is the real needle width; geometry widens it (NEEDLE_GAIN) because a flat blade at a random roll shows
    #   on average 2/pi of the width of a four-sided needle.
    "engelmann": dict(
        whorl=0.32, whorl_n=(4, 6), inter=2, lmax=0.105, p_exp=0.78, base_shrink=0.30,
        pitch_top=52, pitch_base=-18, droop=(8, 38), upturn=(6, 34), wobble=0.03,
        foliage_from=0.06, sec_space=0.05, sec_frac=0.44, sec_max=0.80, sec_spread=60, sec_droop=(2, 22), sec_updn=0.13,
        twig_space=0.024, twig_len=(0.04, 0.09), twig_spread=52, twig_droop=(0, 10),
        needle_len=0.021, needle_w=0.0013, needle_space=0.0017, needle_theta=56, needle_mode="brush",
        fol_depth=1.35, ndl_prim=1.0, ndl_sec=9.0, ndl_shoot=9.0,
        dbh=(0.0165, 1.0), dead_frac=0.85, dead_len=(0.25, 1.1), lean=0.012),
    "subalpine": dict(
        whorl=0.27, whorl_n=(4, 6), inter=4, lmax=0.082, p_exp=0.86, base_shrink=0.18,
        pitch_top=58, pitch_base=-22, droop=(6, 26), upturn=(4, 26), wobble=0.025,
        foliage_from=0.04, sec_space=0.046, sec_frac=0.42, sec_max=0.58, sec_spread=58, sec_droop=(0, 14), sec_updn=0.10,
        twig_space=0.024, twig_len=(0.04, 0.09), twig_spread=54, twig_droop=(0, 6),
        needle_len=0.024, needle_w=0.0016, needle_space=0.0016, needle_theta=66, needle_mode="flat",
        fol_depth=1.1, ndl_prim=0.8, ndl_sec=9.0, ndl_shoot=9.0,
        dbh=(0.0135, 1.0), dead_frac=0.75, dead_len=(0.2, 0.7), lean=0.008),
    "lodgepole": dict(
        whorl=0.30, whorl_n=(3, 5), inter=1, lmax=0.072, p_exp=0.52, base_shrink=0.42,
        pitch_top=62, pitch_base=4, droop=(0, 12), upturn=(10, 34), wobble=0.13,
        foliage_from=0.30, sec_space=0.07, sec_frac=0.50, sec_max=0.55, sec_spread=44, sec_droop=(0, 10), sec_updn=0.30,
        twig_space=0.085, twig_len=(0.07, 0.16), twig_spread=36, twig_droop=(-10, 6),
        needle_len=0.050, needle_w=0.0015, needle_space=0.0012, needle_theta=40, needle_mode="pair",
        fol_depth=1.0, ndl_prim=0.50, ndl_sec=0.40, ndl_shoot=0.30,
        dbh=(0.0115, 1.0), dead_frac=0.65, dead_len=(0.12, 0.55), lean=0.02),
    "douglas": dict(
        whorl=0.40, whorl_n=(4, 6), inter=5, lmax=0.15, p_exp=0.72, base_shrink=0.20,
        pitch_top=46, pitch_base=-10, droop=(6, 26), upturn=(4, 30), wobble=0.06,
        foliage_from=0.12, sec_space=0.07, sec_frac=0.48, sec_max=1.1, sec_spread=55, sec_droop=(10, 50), sec_updn=0.20,
        twig_space=0.034, twig_len=(0.06, 0.14), twig_spread=50, twig_droop=(0, 26),
        needle_len=0.026, needle_w=0.0015, needle_space=0.0022, needle_theta=60, needle_mode="brush",
        fol_depth=1.6, ndl_prim=0.8, ndl_sec=9.0, ndl_shoot=9.0,
        dbh=(0.020, 1.0), dead_frac=0.55, dead_len=(0.3, 1.4), lean=0.015),
    "aspen": dict(
        whorl=0.55, whorl_n=(2, 3), inter=2, lmax=0.17, p_exp=0.42, base_shrink=0.35,
        pitch_top=60, pitch_base=22, droop=(0, 18), upturn=(4, 20), wobble=0.16,
        foliage_from=0.25, sec_space=0.14, sec_frac=0.5, sec_max=1.0, sec_spread=46, sec_droop=(0, 20), sec_updn=0.28,
        twig_space=0.07, twig_len=(0.08, 0.2), twig_spread=44, twig_droop=(0, 25),
        needle_len=0.062, needle_w=0.052, needle_space=0.016, needle_theta=55, needle_mode="brush",
        fol_depth=9.0, ndl_prim=0.0, ndl_sec=0.0, ndl_shoot=9.0,
        dbh=(0.0135, 1.0), dead_frac=0.25, dead_len=(0.2, 0.6), lean=0.03),
    "larch": dict(
        whorl=0.40, whorl_n=(2, 4), inter=2, lmax=0.17, p_exp=0.70, base_shrink=0.25,
        pitch_top=50, pitch_base=-4, droop=(4, 18), upturn=(10, 30), wobble=0.13,
        foliage_from=0.15, sec_space=0.14, sec_frac=0.40, sec_max=0.8, sec_spread=48, sec_droop=(4, 30), sec_updn=0.28,
        twig_space=0.05, twig_len=(0.08, 0.2), twig_spread=44, twig_droop=(0, 30),
        needle_len=0.032, needle_w=0.0022, needle_space=0.018, needle_theta=48, needle_mode="tuft",
        fol_depth=9.0, ndl_prim=0.0, ndl_sec=0.0, ndl_shoot=9.0,
        dbh=(0.024, 1.0), dead_frac=0.4, dead_len=(0.2, 0.8), lean=0.03),
}
NEEDLE_GAIN = 1.5


# name, species, height m, crown ratio, seed. Size classes span the published ranges.
VARIANTS = [
    ("eng_a", "engelmann", 31.0, 0.72, 11), ("eng_b", "engelmann", 25.0, 0.82, 12),
    ("eng_c", "engelmann", 19.0, 0.90, 13), ("eng_d", "engelmann", 12.0, 0.96, 14),
    ("eng_e", "engelmann", 6.0, 1.00, 15),
    ("fir_a", "subalpine", 24.0, 0.78, 21), ("fir_b", "subalpine", 19.0, 0.88, 22),
    ("fir_c", "subalpine", 14.0, 0.95, 23), ("fir_d", "subalpine", 8.5, 1.00, 24),
    ("fir_e", "subalpine", 3.5, 1.00, 25),
    ("pin_a", "lodgepole", 26.0, 0.42, 31), ("pin_b", "lodgepole", 22.0, 0.50, 32),
    ("pin_c", "lodgepole", 17.0, 0.58, 33), ("pin_d", "lodgepole", 11.0, 0.72, 34),
    ("pin_e", "lodgepole", 5.0, 0.92, 35),
    ("dou_a", "douglas", 32.0, 0.62, 41), ("dou_b", "douglas", 22.0, 0.74, 42),
    ("dou_c", "douglas", 11.0, 0.92, 43),
    ("lar_a", "larch", 15.0, 0.78, 51), ("lar_b", "larch", 9.0, 0.9, 52),
    ("snag_a", "snag", 19.0, 0.0, 61), ("snag_b", "snag", 13.0, 0.0, 62), ("snag_c", "snag", 24.0, 0.0, 63),
    ("asp_a", "aspen", 18.0, 0.46, 71), ("asp_b", "aspen", 12.0, 0.55, 72), ("asp_c", "aspen", 6.5, 0.7, 73),
]

LOD = {
    # quad: LOD 0 needles are tapered blades (two triangles) that keep a needle's outline from 2 m; LOD 1 needles are
    # single enlarged triangles carrying the same foliage area at a sixth of the count.
    0: dict(trunk_k=14, prim_k=6, sec_k=3, twig_k=2, needle_mult=1.0, needle_w=1.0, needle_l=1.0, twig_keep=1.0, quad=True),
    1: dict(trunk_k=9, prim_k=4, sec_k=2, twig_k=0, needle_mult=0.16, needle_w=4.0, needle_l=1.35, twig_keep=0.6, quad=False),
}


def _norm(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)


def _frames(T):
    ref = np.zeros_like(T)
    ref[..., 2] = 1.0
    near = np.abs(T[..., 2]) > 0.96
    ref[near] = (1.0, 0.0, 0.0)
    S = _norm(np.cross(T, ref))
    return S, np.cross(S, T)


class Builder:
    def __init__(self):
        self.v, self.f, self.m, self.uv, self.age = [], [], [], [], []
        self.n = 0

    def add(self, verts, faces, mat, uv=None, age=None):
        verts = verts.reshape(-1, 3).astype(np.float32)
        faces = faces.reshape(-1, 3)
        self.v.append(verts)
        self.f.append((faces + self.n).astype(np.uint32))
        self.m.append(np.full(len(faces), mat, np.uint8))
        self.uv.append(np.zeros((len(verts), 2), np.float32) if uv is None else uv.reshape(-1, 2).astype(np.float32))
        if age is None:
            a = np.zeros(len(verts), np.float32)
        else:
            a = np.broadcast_to(np.asarray(age, np.float32).ravel(), (len(verts),)).astype(np.float32) if np.size(age) in (1, len(verts)) else np.asarray(age, np.float32).ravel()
        self.age.append(a)
        self.n += len(verts)

    def arrays(self):
        return dict(verts=np.concatenate(self.v), faces=np.concatenate(self.f), mat=np.concatenate(self.m),
                    buv=np.concatenate(self.uv), age=np.concatenate(self.age))


def tubes(b, P, R, k, mat, uv_scale=None, radial=None, age=None):
    """Add m tubes of n rings. P (m,n,3), R (m,n). k>=3 closed tube, k==2 flat ribbon."""
    m, n, _ = P.shape
    T = np.gradient(P, axis=1)
    T = _norm(T)
    S, U = _frames(T)
    seam = uv_scale is not None
    kk = k + 1 if seam else k
    ang = 2 * np.pi * np.arange(kk) / k
    c, s = np.cos(ang), np.sin(ang)
    if k == 2:
        c, s = np.array([1.0, -1.0]), np.array([0.0, 0.0])
    rr = R[:, :, None]
    if radial is not None:
        rr = rr * radial(ang, n, m)
    V = P[:, :, None, :] + rr[..., None] * (c[None, None, :, None] * S[:, :, None, :] + s[None, None, :, None] * U[:, :, None, :])
    idx = np.arange(m * n * kk).reshape(m, n, kk)
    if k == 2:
        a, bb, cc, d = idx[:, :-1, 0], idx[:, :-1, 1], idx[:, 1:, 1], idx[:, 1:, 0]
    elif seam:
        a, bb = idx[:, :-1, :-1], idx[:, :-1, 1:]
        cc, d = idx[:, 1:, 1:], idx[:, 1:, :-1]
    else:
        a, bb = idx[:, :-1, :], np.roll(idx[:, :-1, :], -1, axis=2)
        cc, d = np.roll(idx[:, 1:, :], -1, axis=2), idx[:, 1:, :]
    F = np.concatenate([np.stack([a, bb, cc], -1).reshape(-1, 3), np.stack([a, cc, d], -1).reshape(-1, 3)])
    uv = None
    if seam:
        seg = np.linalg.norm(np.diff(P, axis=1), axis=-1)
        sv = np.concatenate([np.zeros((m, 1)), np.cumsum(seg, axis=1)], axis=1)
        circ = 2 * np.pi * R.max(axis=1, keepdims=True)
        uu = (np.arange(kk) / k)[None, None, :] * np.round(np.maximum(circ, uv_scale))[:, :, None]
        uv = np.stack([np.broadcast_to(uu, (m, n, kk)), np.broadcast_to(sv[:, :, None], (m, n, kk))], -1) / uv_scale
    if age is not None and np.ndim(age) > 0:
        age = np.repeat(np.repeat(np.asarray(age)[:, None], n, 1)[:, :, None], kk, 2)
    b.add(V, F, mat, uv=uv, age=age)


def _lerp(a, b, t):
    return a + (b - a) * t


def _radius_profile(z, H, dbh):
    r = 0.5 * dbh * np.clip(1.0 - z / H, 0.0, 1.0) ** 0.82
    return r * (1.0 + 0.55 * np.exp(-z / 0.45)) + 0.0015


def trunk_line(H, lean, rng, n):
    z = np.linspace(-0.25, H, n)
    dx = np.cumsum(rng.normal(0, lean * H / n * 2.2, n))
    dy = np.cumsum(rng.normal(0, lean * H / n * 2.2, n))
    a = rng.uniform(0, 2 * np.pi)
    sweep = rng.uniform(0.0, lean) * z
    x = dx - dx[0] + sweep * np.cos(a)
    y = dy - dy[0] + sweep * np.sin(a)
    # pull tip back over the base so spires stay straight
    return np.stack([x * (1 - 0.6 * (z / H) ** 3), y * (1 - 0.6 * (z / H) ** 3), z], -1)


def branch_line(p0, az, pitch0, droop, upturn, L, n, rng, wob):
    t = np.linspace(0, 1, n + 1)
    pitch = pitch0 - droop * t ** 1.2 + upturn * t ** 3
    azs = az + np.cumsum(rng.normal(0, wob, n + 1))
    d = np.stack([np.cos(pitch) * np.cos(azs), np.cos(pitch) * np.sin(azs), np.sin(pitch)], -1)
    pts = p0 + np.vstack([np.zeros(3), np.cumsum(d[:-1] * L / n, axis=0)])
    return pts


def _at(line, zq):
    """Trunk point at height zq."""
    return np.array([np.interp(zq, line[:, 2], line[:, 0]), np.interp(zq, line[:, 2], line[:, 1]), zq])


def _leaf_geo(b, base, tdir, side, up, L, W, rng, age):
    """Rounded broadleaf: a hexagon, flexing on its petiole so a crown of them scatters light unevenly."""
    N = len(base)
    if N == 0:
        return
    tilt = rng.normal(0.0, 0.7, N)
    az = rng.uniform(0, 2 * np.pi, N)
    d = _norm(tdir * 0.5 + np.cos(az)[:, None] * side + np.sin(az)[:, None] * up + rng.normal(0, 0.3, (N, 3)) + np.array([0, 0, -0.7]))
    s = _norm(np.cross(d, up + rng.normal(0, 0.5, (N, 3))))
    pet = base + d * 0.018
    size = (np.ravel(L) * rng.uniform(0.7, 1.15, N))[:, None] if np.size(L) == N else (L * rng.uniform(0.7, 1.15, N))[:, None]
    o = np.array([0.0, 0.36, 0.86, 1.0, 0.86, 0.36])  # along
    w = np.array([0.0, 0.42, 0.40, 0.0, -0.40, -0.42])  # across
    V = pet[:, None, :] + size[:, None] * (o[None, :, None] * d[:, None, :] + w[None, :, None] * s[:, None, :])
    idx = np.arange(N)[:, None] * 6
    F = np.concatenate([idx + np.array(tri) for tri in ([0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5])])
    a = np.repeat(np.asarray(age, np.float32), 6) if np.size(age) == N else age
    b.add(V, F, 1, age=a)


def _needle_geo(b, base, tdir, side, up, L, W, th, mode, rng, age, quad=False):
    N = len(base)
    if N == 0:
        return
    if W > 0.02:
        _leaf_geo(b, base, tdir, side, up, L, W, rng, age)
        return
    t = th * (1 + rng.normal(0, 0.18, N))
    if mode == "flat":
        # fir: two-ranked with a strong upward sweep, the brush of the upper shoot
        phi = np.where(rng.random(N) < 0.5, 0.28, np.pi - 0.28) + rng.normal(0, 0.42, N)
    elif mode == "brush":
        # spruce: spiral all round the shoot, the lower ones twisting up to the sides, so the top is densest
        phi = np.pi / 2 + rng.normal(0, 1.35, N)
    else:
        phi = rng.uniform(0, 2 * np.pi, N)
    d = np.cos(t)[:, None] * tdir + np.sin(t)[:, None] * (np.cos(phi)[:, None] * side + np.sin(phi)[:, None] * up)
    d = _norm(d + np.array([0, 0, 0.18]) * (mode == "flat"))
    ll = L * (1 + rng.normal(0, 0.12, N))[:, None]
    r = _norm(rng.normal(size=(N, 3)))
    s = _norm(np.cross(d, r))
    w = W * NEEDLE_GAIN * (1 + rng.normal(0, 0.1, N))[:, None] * 0.5
    a = np.asarray(age, np.float32)
    if quad:
        # tapered blade: full width at the base, a third at the tip; stiff and straight
        tip = base + ll * d
        V = np.stack([base + w * s, base - w * s, tip - 0.33 * w * s, tip + 0.33 * w * s], 1)
        idx = np.arange(N, dtype=np.int64)[:, None] * 4
        F = np.concatenate([idx + np.array([0, 1, 2]), idx + np.array([0, 2, 3])])
        b.add(V, F, 1, age=np.repeat(a, 4) if np.size(a) == N else a)
        return
    V = np.stack([base + w * s, base - w * s, base + ll * d], 1)
    F = np.arange(N * 3, dtype=np.int64).reshape(N, 3)
    b.add(V, F, 1, age=np.repeat(a, 3) if np.size(a) == N else a)


def _diamonds(b, start, end, side, width, age, mat=1):
    """Leaf-like diamond from start to end, widest at 40 %, for sprays on the low LODs."""
    N = len(start)
    if N == 0:
        return
    mid = start + 0.42 * (end - start)
    V = np.stack([start, mid + side * width[:, None] * 0.5, end, mid - side * width[:, None] * 0.5], 1)
    idx = np.arange(N)[:, None] * 4
    F = np.concatenate([idx + np.array([0, 1, 2]), idx + np.array([0, 2, 3])])
    b.add(V, F, mat, age=np.repeat(np.asarray(age, np.float32), 4))


def _whorls(H, crown_base, sp, rng):
    """Heights of primary branches with their crown-relative depth."""
    zs = []
    z = H - rng.uniform(0.25, 0.45) * min(1.0, H / 8)
    step = sp["whorl"]
    while z > crown_base:
        n = rng.integers(*sp["whorl_n"], endpoint=True)
        zs.extend([z + rng.normal(0, 0.015)] * n)
        k = sp["inter"]
        if k:
            depth = (H - z) / max(H - crown_base, 1e-3)
            zs.extend((z - rng.uniform(0.15, 0.95, rng.poisson(k * min(1.0, depth * 2.5))) * step).tolist())
        z -= step * rng.uniform(0.75, 1.25) * _lerp(0.75, 1.0, min(1.0, (H - z) / (0.3 * H)))
    return np.array(zs)


def build_tree(species, H, crown_ratio, seed, lod):
    rng = np.random.default_rng(seed + 1000 * lod)
    if species == "snag":
        return build_snag(H, seed, lod)
    sp = SPECIES[species]
    b = Builder()
    L_ = LOD.get(lod)
    rngs = np.random.default_rng(seed)  # shared by all LODs so silhouettes match
    trunk_k = {0: 14, 1: 9, 2: 6, 3: 3}[lod]
    n_rings = {0: int(H / 0.28) + 8, 1: int(H / 0.6) + 5, 2: int(H / 1.5) + 4, 3: 4}[lod]
    line = trunk_line(H, sp["lean"], np.random.default_rng(seed + 7), max(n_rings, 4))
    dbh = sp["dbh"][0] * H
    R = _radius_profile(line[:, 2].clip(0), H, dbh)

    def bark_lobes(ang, n, m):
        zz = np.linspace(0, 1, n)[None, :, None]
        amp = 0.05 + 0.26 * np.exp(-zz * H / 0.8)
        return 1 + amp * (0.6 * np.sin(5 * ang + seed)[None, None, :] + 0.4 * np.sin(3 * ang + 2 * seed)[None, None, :])

    tubes(b, line[None], R[None], trunk_k, 0, uv_scale=0.9, radial=bark_lobes if lod < 2 else None, age=None)

    crown_len = H * crown_ratio
    crown_base = H - crown_len
    Lmax = sp["lmax"] * H * (1 + max(0.0, 11.0 - H) / 14.0)
    zs = _whorls(H, max(crown_base, 0.15), sp, rngs)
    rel = np.clip((H - zs) / max(crown_len, 1e-3), 0, 1)
    prof = rel ** sp["p_exp"] * (1 - sp["base_shrink"] * rel ** 3)
    Ls = Lmax * prof * rngs.normal(1.0, 0.13, len(zs)).clip(0.6, 1.45)
    Ls = np.maximum(Ls, 0.10)
    azs = rngs.uniform(0, 2 * np.pi, len(zs))
    if lod <= 1:
        # dead lower limbs are what make a forest-grown conifer read as real at close range
        z_dead = np.arange(0.5, max(crown_base, 0.5), sp["whorl"] * 0.9)
        keep = rngs.random(len(z_dead)) < sp["dead_frac"]
        z_dead = z_dead[keep]
        for z in z_dead:
            n = rngs.integers(1, 4)
            for _ in range(n):
                p0 = _at(line, z)
                L = rngs.uniform(*sp["dead_len"]) * (0.5 + 0.5 * z / max(crown_base, 1))
                pts = branch_line(p0, rngs.uniform(0, 6.28), rngs.uniform(-30, 10) * D2R, rngs.uniform(0, 30) * D2R, 0, L, 4, rngs, 0.12)
                rad = np.linspace(0.009, 0.0025, 5) * (0.6 + L)
                tubes(b, pts[None], rad[None], 3 if lod == 0 else 2, 2)
                if lod == 0:
                    _dead_twigs(b, pts, rngs)
    rel_all = rel
    sec_all, twig_store = [], []
    for i, (z, L, az, rl) in enumerate(zip(zs, Ls, azs, rel_all)):
        p0 = _at(line, z)
        rd = rngs.normal(1.0, 0.18)
        pitch0 = (_lerp(sp["pitch_top"], sp["pitch_base"], rl ** 0.55) + rngs.normal(0, 9)) * D2R * rd
        droop = _lerp(sp["droop"][0], sp["droop"][1], rl) * D2R * rngs.uniform(0.7, 1.25)
        up = _lerp(sp["upturn"][0], sp["upturn"][1], rl) * D2R * rngs.uniform(0.6, 1.3)
        nseg = {0: 8, 1: 6, 2: 4, 3: 2}[lod]
        pts = branch_line(p0, az, pitch0, droop, up, L, nseg, rngs, sp["wobble"])
        rad = np.linspace(1.0, 0.18, nseg + 1) ** 1.2 * (0.003 + 0.009 * L + 0.002 * L * L)
        if lod <= 2:
            tubes(b, pts[None], rad[None], {0: 6, 1: 4, 2: 3}[lod], 2, age=None)
        _foliage(b, sp, species, pts, L, rl, rngs, lod, line, z)
    if lod <= 1:
        _leader_shoots(b, sp, line, H, rngs, lod)
    return b.arrays()


def _dead_twigs(b, pts, rng):
    n = rng.integers(3, 9)
    t = rng.uniform(0.2, 0.98, n)
    idx = (t * (len(pts) - 1)).astype(int)
    base = pts[idx]
    T = _norm(pts[np.minimum(idx + 1, len(pts) - 1)] - pts[idx])
    S, U = _frames(T)
    a = rng.uniform(0, 6.28, n)[:, None]
    d = _norm(0.7 * T + np.cos(a) * S + np.sin(a) * U)
    L = rng.uniform(0.05, 0.25, n)[:, None]
    P = np.stack([base, base + d * L * 0.5, base + d * L], 1)
    R = np.array([0.0028, 0.0018, 0.0007])[None] * np.ones((n, 1))
    tubes(b, P, R, 2, 2)


def _foliage(b, sp, species, pts, L, rl, rng, lod, line, z):
    """Hang secondaries, shoots and needles (or sprays on the low LODs) on one primary branch."""
    if lod >= 3:
        _primary_sprays(b, sp, pts, L, rl)
        return
    sec = _secondaries(sp, pts, L, rl, rng, lod)
    if lod == 2:
        _secondary_sprays(b, sec, rl, rng)
        return
    _clothe(b, sp, pts, sec, rl, rng, lod)


def _primary_sprays(b, sp, pts, L, rl):
    """LOD 3: one or two big sprays on the primary, kept in sync with the silhouette."""
    n = len(pts) - 1
    p_a = pts[int(n * sp["foliage_from"])]
    side = _norm(np.cross(pts[-1] - pts[0], np.array([0, 0, 1.0])) + 1e-6)[None]
    w = np.array([L * sp["sec_frac"] * 1.45])
    _diamonds(b, p_a[None], pts[-1][None] + np.array([[0, 0, -0.06 * L]]), side, w, np.array([0.5 * rl]))
    up = np.cross(side, _norm(pts[-1] - pts[0])[None])
    _diamonds(b, p_a[None], pts[-1][None], up * 0.5 + side * 0.2, w * 0.55, np.array([0.5 * rl + 0.2]))


def _secondaries(sp, pts, L, rl, rng, lod):
    """Where the secondary branches leave the primary, their direction, length and droop."""
    n = len(pts) - 1
    seg = np.diff(pts, axis=0)
    cum = np.concatenate([[0], np.cumsum(np.linalg.norm(seg, axis=1))])
    total = cum[-1]
    start = sp["foliage_from"]
    ts = np.arange(start + rng.uniform(0, 0.05), 0.995, sp["sec_space"] / max(total, 1e-3))
    if len(ts) == 0:
        ts = np.array([0.9])
    if lod == 2:
        ts = ts[rng.integers(0, 3)::3]
        if len(ts) == 0:
            ts = np.array([0.9])
    s = ts * total
    P = np.stack([np.interp(s, cum, pts[:, i]) for i in range(3)], -1)
    T = _norm(seg[np.clip(np.searchsorted(cum, s) - 1, 0, n - 1)])
    S_ = _norm(np.cross(T, np.array([0, 0, 1.0])))
    U_ = np.cross(S_, T)
    sign = np.where(np.arange(len(ts)) % 2 == 0, 1.0, -1.0)
    spread = sp["sec_spread"] * D2R * rng.uniform(0.75, 1.2, len(ts))
    envelope = np.minimum(1.0, (1 - ts) / 0.35 + 0.28) * np.minimum(1.0, (ts - start) / 0.12 + 0.5)
    Ls = np.maximum(np.minimum(sp["sec_frac"] * L, sp["sec_max"]) * envelope * rng.uniform(0.75, 1.15, len(ts)), 0.05)
    # low branches let their secondaries hang (Engelmann curtains); upper ones hold flat
    drp = _lerp(sp["sec_droop"][0], sp["sec_droop"][1], rl) * D2R * rng.uniform(0.6, 1.3, len(ts))
    # some secondaries go over and under the branch to thicken the spray; spruce and fir sprays stay nearly planar,
    # which is what separates their branch tiers
    updn = rng.normal(0, sp["sec_updn"], len(ts))
    d0 = _norm(np.cos(spread)[:, None] * T + np.sin(spread)[:, None] * sign[:, None] * S_ + updn[:, None] * U_)
    return dict(P=P, U=U_, d0=d0, Ls=Ls, drp=drp, ts=ts, total=total)


def _secondary_sprays(b, sec, rl, rng):
    """LOD 2: one leaf-like spray per secondary."""
    P, d0, Ls, ts = sec["P"], sec["d0"], sec["Ls"], sec["ts"]
    end = P + d0 * Ls[:, None] * 1.25 + np.array([0, 0, -1.0]) * (np.sin(sec["drp"]) * Ls)[:, None] * 0.7
    side = _norm(np.cross(d0, sec["U"] + rng.normal(0, 0.25, (len(ts), 3))))
    _diamonds(b, P, end, side, Ls * 0.82, 0.6 * (1 - ts) * rl + rng.uniform(0, 0.2, len(ts)))


DOWN = np.array([0, 0, -1.0])


def _clothe(b, sp, pts, sec, rl, rng, lod):
    """LOD 0-1: secondary wood, dead inner twigs, shoots, and needles on every axis that still holds them."""
    P, d0, Ls, drp, ts = sec["P"], sec["d0"], sec["Ls"], sec["drp"], sec["ts"]
    t3 = np.linspace(0, 1, 4)[None, :, None]
    Sec = P[:, None, :] + d0[:, None, :] * Ls[:, None, None] * t3 + DOWN * (np.sin(drp) * Ls)[:, None, None] * t3 ** 2
    rs = (0.0009 + 0.0035 * Ls)[:, None] * np.linspace(1, 0.35, 4)[None, :]
    # the shaded inner crown: secondaries far from the branch tip have lost their foliage and hang on as bare twigs
    from_tip = (1 - ts) * sec["total"]
    live = from_tip < sp["fol_depth"] * rng.uniform(0.8, 1.2)
    if lod == 0 and (~live).any():
        stub = rng.uniform(0.25, 0.6, (~live).sum())[:, None, None]
        tubes(b, P[~live][:, None, :] + (Sec[~live] - P[~live][:, None, :]) * stub, rs[~live] * 0.8, 2, 2)
    L_ = LOD[lod]
    if not live.any():
        _needles(b, sp, *_tip_segments(pts[None], sp["ndl_prim"]), rng, L_)
        return
    live_sec = {k: (v[live] if np.ndim(v) else v) for k, v in sec.items()}
    Sec, from_tip = Sec[live], from_tip[live]
    tubes(b, Sec, rs[live], {0: 3, 1: 2}[lod], 2)
    base, tdir, Lt, sh_from = _shoots(sp, live_sec, Sec, rl, rng, L_)
    if L_["twig_k"] == 2:
        tp = np.stack([base, base + tdir * Lt[:, None] * 0.5, base + tdir * Lt[:, None]], 1)
        tubes(b, tp, np.array([0.0013, 0.001, 0.0005])[None] * np.ones((len(base), 1)), 2, 2)
    keep = np.minimum(Lt, sp["ndl_shoot"])
    segs = [
        _tip_segments(pts[None], sp["ndl_prim"]),
        _tip_segments(Sec, sp["ndl_sec"], from_tip[:, None] * 0.3),
        (base + tdir * (Lt - keep)[:, None], tdir, keep, keep + sh_from),
    ]
    _needles(b, sp, *(np.concatenate(c) for c in zip(*segs)), rng, L_)


def _shoots(sp, sec, Sec, rl, rng, L_):
    """Shoots along each secondary, both sides, plus its terminal growth. Returns base, direction, length and the
    path distance from each shoot's base to the secondary's growing tip, which sets needle age."""
    P, d0, Ls, drp, ts, U_ = sec["P"], sec["d0"], sec["Ls"], sec["drp"], sec["ts"], sec["U"]
    counts = np.maximum((Ls / sp["twig_space"]).astype(int), 1)
    if L_["twig_keep"] < 1:
        counts = np.maximum((counts * L_["twig_keep"]).astype(int), 1)
    sid = np.repeat(np.arange(len(ts)), counts)
    u = np.clip(np.concatenate([(np.arange(c) + rng.uniform(0.2, 0.8)) / c for c in counts]), 0.03, 1)
    base = P[sid] + d0[sid] * (Ls[sid] * u)[:, None] + DOWN * (np.sin(drp[sid]) * Ls[sid] * u ** 2)[:, None]
    tdir0 = _norm(d0[sid] + 2 * u[:, None] * DOWN * np.sin(drp[sid])[:, None])
    Sx = _norm(np.cross(tdir0, U_[sid]))
    m = len(sid)
    sgn = np.where(rng.random(m) < 0.5, 1.0, -1.0)
    twsp = sp["twig_spread"] * D2R * rng.uniform(0.7, 1.2, m)
    # shoot length tapers to the end of the secondary; last few run straight on
    Lt = rng.uniform(*sp["twig_len"], m) * np.where(u > 0.82, 1.2, 1.0) * np.minimum(1.0, 0.5 + (1 - u) * 1.5)
    tl_drp = _lerp(sp["twig_droop"][0], sp["twig_droop"][1], rl) * D2R * rng.uniform(0.5, 1.3, m)
    tdir = _norm(np.cos(twsp)[:, None] * tdir0 + np.sin(twsp)[:, None] * sgn[:, None] * Sx +
                 DOWN * np.sin(tl_drp)[:, None] + rng.normal(0, 0.10, (m, 3)))
    base = np.vstack([base, Sec[:, -1, :]])
    tdir = np.vstack([tdir, _norm(Sec[:, -1, :] - Sec[:, -2, :])])
    Lt = np.concatenate([Lt, rng.uniform(*sp["twig_len"], len(ts)) * 1.2])
    # path distance from the nearest growing tip: every branch end carries fresh light growth over an older interior
    sh_from = np.concatenate([(1 - u) * Ls[sid], np.zeros(len(ts))])
    return base, tdir, Lt, sh_from


AGE_M = 0.5  # metres of path back from a growing tip at which needles reach full old-needle colour


def _tip_segments(P, keep_m, extra=0.0):
    """Straight pieces of polylines P (m, k+1, 3) lying within keep_m metres of each line's end.
    Returns base (s,3), unit direction (s,3), length (s,), and path distance from the tip at each base (s,)."""
    seg = np.diff(P, axis=1)
    ln = np.linalg.norm(seg, axis=-1)
    d = seg / np.maximum(ln, 1e-9)[..., None]
    s0 = np.concatenate([np.zeros((len(P), 1)), np.cumsum(ln, 1)[:, :-1]], 1)
    tot = ln.sum(1, keepdims=True)
    a = np.maximum(s0, tot - keep_m)
    ok = s0 + ln > a + 1e-4
    base = P[:, :-1] + d * (a - s0)[..., None]
    dist = tot - a + np.broadcast_to(extra, ln.shape)
    return base[ok], d[ok], (s0 + ln - a)[ok], dist[ok]


def _needles(b, sp, base, tdir, length, dist, rng, L_):
    """Needles along straight shoot pieces, base to base + tdir * length; dist is the path distance from the
    growing tip at each base, which sets needle age."""
    per = np.maximum(length / sp["needle_space"] * L_["needle_mult"], 0.0)
    cnt = np.floor(per + rng.random(len(per))).astype(int)
    mode = sp["needle_mode"]
    if mode == "tuft":
        # larch: tufts of 15-25 on spur shoots
        cnt = cnt * max(1, int(18 * max(L_["needle_mult"], 0.25)))
    tid = np.repeat(np.arange(len(base)), cnt)
    if len(tid) == 0:
        return
    u = rng.random(len(tid))
    if mode == "tuft":
        spur = rng.integers(0, 6, len(tid)) / 6.0
        u = spur + rng.uniform(0, 0.012, len(tid))
    d = tdir[tid]
    S, U = _frames(d)
    age = np.clip((dist[tid] - u * length[tid]) / AGE_M + rng.normal(0, 0.08, len(tid)), 0, 1)
    # needles shorten into the bud at each growing tip
    tipd = np.clip(dist[tid] - u * length[tid], 0, None)
    ln = sp["needle_len"] * L_["needle_l"] * (0.55 + 0.45 * np.clip(tipd / 0.04, 0, 1))[:, None]
    _needle_geo(b, base[tid] + d * (u * length[tid])[:, None], d, S, U, ln, sp["needle_w"] * L_["needle_w"],
                sp["needle_theta"] * D2R, mode, rng, age, quad=L_.get("quad", False))


def _leader_shoots(b, sp, line, H, rng, lod):
    """Needles on the leader itself, the fine spike at the very top of the spire."""
    L_ = LOD[lod]
    m = 18 if lod == 0 else 5
    z = H - rng.uniform(0.0, min(0.9, 0.08 * H), m) - 0.05
    base = np.stack([_at(line, zz) for zz in z])
    a = rng.uniform(0, 6.28, m)
    tdir = _norm(np.stack([np.cos(a), np.sin(a), np.full(m, 1.6)], -1))
    Lt = rng.uniform(0.05, 0.13, m)
    _needles(b, sp, base, tdir, Lt, Lt, rng, L_)
    if lod == 0:
        z2 = np.linspace(H - 0.5, H, 24)
        base2 = np.stack([_at(line, zz) for zz in z2])
        _needles(b, sp, base2, np.tile([0, 0, 1.0], (24, 1)), np.full(24, 0.03), H - z2, rng, L_)


def build_snag(H, seed, lod):
    """Dead standing tree: grey wood, broken top, stubs. Mortality classes differ by seed."""
    rng = np.random.default_rng(seed)
    b = Builder()
    sp = SPECIES["subalpine"]
    n = {0: int(H / 0.3) + 6, 1: int(H / 0.8) + 4, 2: 8, 3: 4}[lod]
    line = trunk_line(H, 0.018, rng, n)
    dbh = 0.0165 * H * 1.15
    R = _radius_profile(line[:, 2].clip(0), H * 1.35, dbh)
    tubes(b, line[None], R[None], {0: 12, 1: 8, 2: 5, 3: 3}[lod], 2, uv_scale=0.9)
    if lod >= 3:
        return b.arrays()
    zs = np.arange(1.2, H - 0.3, 0.75 if lod == 2 else 0.42)
    for z in zs:
        if rng.random() > (0.55 if lod < 2 else 0.25):
            continue
        p0 = _at(line, z)
        L = rng.uniform(0.15, 1.5) * (1 - 0.5 * z / H) * rng.random() ** 0.6
        pts = branch_line(p0, rng.uniform(0, 6.28), rng.uniform(-20, 25) * D2R, rng.uniform(0, 30) * D2R, 0, L, 3, rng, 0.2)
        rad = np.linspace(0.016, 0.003, 4) * (0.6 + L)
        tubes(b, pts[None], rad[None], {0: 4, 1: 3, 2: 2}[lod], 2)
        if lod == 0:
            _dead_twigs(b, pts, rng)
    return b.arrays()


def info(arr):
    return dict(verts=len(arr["verts"]), tris=len(arr["faces"]))


def cache_key(species, lod):
    """Content key for a cached mesh: this file's code (without the species table) plus the one species entry and
    LOD entry it uses, so tuning one species does not throw away every other species' multi-GB hero meshes."""
    import hashlib
    import re
    with open(__file__) as fh:
        src = re.sub(r"\nSPECIES = \{.*?\n\}\n", "\n", fh.read(), flags=re.S)
    h = hashlib.sha1(src.encode())
    used = [species] if species in SPECIES else list(SPECIES)  # snags and the ground layer borrow other species
    for nm in used:
        h.update(repr(sorted(SPECIES[nm].items())).encode())
    h.update(repr(LOD.get(lod)).encode())
    return h.hexdigest()[:10]


if __name__ == "__main__":
    import time
    for name, spc, h, cr, seed in VARIANTS:
        if not name.endswith(("a", "e")):
            continue
        for lod in range(4):
            t0 = time.time()
            a = build_tree(spc, h, cr, seed, lod)
            print(name, "lod", lod, info(a), "%.1fs" % (time.time() - t0), flush=True)
