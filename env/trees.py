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
    # lmax: longest primary branch / H. crown: profile exponents. needle lengths in metres.
    "engelmann": dict(
        whorl=0.32, whorl_n=(4, 6), inter=5, lmax=0.105, p_exp=0.78, base_shrink=0.30,
        pitch_top=52, pitch_base=-14, droop=(8, 34), upturn=(6, 40), wobble=0.05,
        foliage_from=0.06, sec_space=0.058, sec_frac=0.46, sec_max=0.95, sec_spread=58, sec_droop=(4, 52),
        twig_space=0.030, twig_len=(0.06, 0.14), twig_spread=46, twig_droop=(0, 50),
        needle_len=0.024, needle_w=0.0033, needle_space=0.0019, needle_theta=52, needle_mode="brush",
        dbh=(0.0165, 1.0), dead_frac=0.85, dead_len=(0.25, 1.1), lean=0.012),
    "subalpine": dict(
        whorl=0.27, whorl_n=(4, 6), inter=4, lmax=0.082, p_exp=0.86, base_shrink=0.18,
        pitch_top=58, pitch_base=-22, droop=(6, 26), upturn=(4, 26), wobble=0.035,
        foliage_from=0.04, sec_space=0.052, sec_frac=0.42, sec_max=0.62, sec_spread=55, sec_droop=(0, 18),
        twig_space=0.028, twig_len=(0.05, 0.11), twig_spread=50, twig_droop=(0, 8),
        needle_len=0.026, needle_w=0.0036, needle_space=0.0019, needle_theta=68, needle_mode="flat",
        dbh=(0.0135, 1.0), dead_frac=0.75, dead_len=(0.2, 0.7), lean=0.008),
    "lodgepole": dict(
        whorl=0.30, whorl_n=(3, 5), inter=1, lmax=0.088, p_exp=0.52, base_shrink=0.42,
        pitch_top=62, pitch_base=4, droop=(0, 12), upturn=(10, 34), wobble=0.09,
        foliage_from=0.30, sec_space=0.07, sec_frac=0.42, sec_max=0.5, sec_spread=42, sec_droop=(0, 8),
        twig_space=0.05, twig_len=(0.10, 0.22), twig_spread=30, twig_droop=(-10, 6),
        needle_len=0.060, needle_w=0.0028, needle_space=0.0024, needle_theta=32, needle_mode="pair",
        dbh=(0.0115, 1.0), dead_frac=0.65, dead_len=(0.12, 0.55), lean=0.02),
    "douglas": dict(
        whorl=0.40, whorl_n=(4, 6), inter=5, lmax=0.15, p_exp=0.72, base_shrink=0.20,
        pitch_top=46, pitch_base=-10, droop=(6, 26), upturn=(4, 30), wobble=0.08,
        foliage_from=0.12, sec_space=0.07, sec_frac=0.48, sec_max=1.3, sec_spread=55, sec_droop=(10, 60),
        twig_space=0.034, twig_len=(0.07, 0.17), twig_spread=48, twig_droop=(0, 35),
        needle_len=0.028, needle_w=0.0027, needle_space=0.0028, needle_theta=58, needle_mode="brush",
        dbh=(0.020, 1.0), dead_frac=0.55, dead_len=(0.3, 1.4), lean=0.015),
    "aspen": dict(
        whorl=0.55, whorl_n=(2, 3), inter=2, lmax=0.17, p_exp=0.42, base_shrink=0.35,
        pitch_top=60, pitch_base=22, droop=(0, 18), upturn=(4, 20), wobble=0.16,
        foliage_from=0.25, sec_space=0.14, sec_frac=0.5, sec_max=1.0, sec_spread=46, sec_droop=(0, 20),
        twig_space=0.07, twig_len=(0.08, 0.2), twig_spread=44, twig_droop=(0, 25),
        needle_len=0.062, needle_w=0.052, needle_space=0.016, needle_theta=55, needle_mode="brush",
        dbh=(0.0135, 1.0), dead_frac=0.25, dead_len=(0.2, 0.6), lean=0.03),
    "larch": dict(
        whorl=0.40, whorl_n=(2, 4), inter=2, lmax=0.17, p_exp=0.70, base_shrink=0.25,
        pitch_top=50, pitch_base=-4, droop=(4, 18), upturn=(10, 30), wobble=0.13,
        foliage_from=0.15, sec_space=0.14, sec_frac=0.40, sec_max=0.8, sec_spread=48, sec_droop=(4, 30),
        twig_space=0.05, twig_len=(0.08, 0.2), twig_spread=44, twig_droop=(0, 30),
        needle_len=0.032, needle_w=0.0022, needle_space=0.018, needle_theta=48, needle_mode="tuft",
        dbh=(0.024, 1.0), dead_frac=0.4, dead_len=(0.2, 0.8), lean=0.03),
}


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

# Understorey, ground layer around the corridor only. name, kind, seed.
UNDER = [
    ("grass_a", "grass", 81), ("grass_b", "grass", 82), ("grass_c", "grass", 83),
    ("shrub_a", "shrub", 84), ("shrub_b", "shrub", 85), ("shrub_c", "shrub", 86),
    ("log_a", "log", 87), ("log_b", "log", 88), ("rock_a", "rock", 89), ("rock_b", "rock", 90), ("rock_c", "rock", 91),
    ("sap_a", "sapling", 92), ("sap_b", "sapling", 93),
]

LOD = {
    0: dict(trunk_k=14, prim_k=6, sec_k=3, twig_k=2, needle_mult=1.0, needle_w=1.0, needle_l=1.0, twig_keep=1.0),
    1: dict(trunk_k=9, prim_k=4, sec_k=2, twig_k=0, needle_mult=0.16, needle_w=2.6, needle_l=1.35, twig_keep=0.6),
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
    size = (L * rng.uniform(0.7, 1.15, N))[:, None]
    o = np.array([0.0, 0.36, 0.86, 1.0, 0.86, 0.36])  # along
    w = np.array([0.0, 0.42, 0.40, 0.0, -0.40, -0.42])  # across
    V = pet[:, None, :] + size[:, None] * (o[None, :, None] * d[:, None, :] + w[None, :, None] * s[:, None, :])
    idx = np.arange(N)[:, None] * 6
    F = np.concatenate([idx + np.array(tri) for tri in ([0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 5])])
    a = np.repeat(np.asarray(age, np.float32), 6) if np.size(age) == N else age
    b.add(V, F, 1, age=a)


def _needle_geo(b, base, tdir, side, up, L, W, th, mode, rng, age):
    N = len(base)
    if N == 0:
        return
    if W > 0.02:
        _leaf_geo(b, base, tdir, side, up, L, W, rng, age)
        return
    t = th * (1 + rng.normal(0, 0.22, N))
    if mode == "flat":
        # fir: two-ranked with a strong upward sweep, the brush of the upper shoot
        phi = np.where(rng.random(N) < 0.5, 0.28, np.pi - 0.28) + rng.normal(0, 0.42, N)
    elif mode == "brush":
        phi = np.pi / 2 + rng.normal(0, 1.15, N)
    else:
        phi = rng.uniform(0, 2 * np.pi, N)
    d = np.cos(t)[:, None] * tdir + np.sin(t)[:, None] * (np.cos(phi)[:, None] * side + np.sin(phi)[:, None] * up)
    d = _norm(d + np.array([0, 0, 0.18]) * (mode == "flat"))
    ll = L * (1 + rng.normal(0, 0.13, N))[:, None]
    r = _norm(rng.normal(size=(N, 3)))
    s = _norm(np.cross(d, r))
    w = W * (1 + rng.normal(0, 0.1, N))[:, None] * 0.5
    V = np.stack([base + w * s, base - w * s, base + ll * d], 1)
    F = np.arange(N * 3, dtype=np.int64).reshape(N, 3)
    a = np.repeat(np.asarray(age, np.float32), 3) if np.size(age) == N else age
    b.add(V, F, 1, age=a)


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
    n = len(pts) - 1
    seg = np.diff(pts, axis=0)
    seglen = np.linalg.norm(seg, axis=1)
    cum = np.concatenate([[0], np.cumsum(seglen)])
    total = cum[-1]

    def along(ts):
        s = ts * total
        p = np.stack([np.interp(s, cum, pts[:, i]) for i in range(3)], -1)
        idx = np.clip(np.searchsorted(cum, s) - 1, 0, n - 1)
        return p, _norm(seg[idx])

    start = sp["foliage_from"]
    spacing = sp["sec_space"]
    if lod >= 3:
        # one or two big sprays on the primary, kept in sync with the silhouette
        p_a = pts[int(n * start)]
        side = _norm(np.cross(pts[-1] - pts[0], np.array([0, 0, 1.0])) + 1e-6)[None]
        w = np.array([L * sp["sec_frac"] * 1.45])
        _diamonds(b, p_a[None], pts[-1][None] + np.array([[0, 0, -0.06 * L]]), side, w, np.array([0.5 * rl]))
        up = np.cross(side, _norm(pts[-1] - pts[0])[None])
        _diamonds(b, p_a[None], pts[-1][None], up * 0.5 + side * 0.2, w * 0.55, np.array([0.5 * rl + 0.2]))
        return
    ts = np.arange(start + rng.uniform(0, 0.05), 0.995, spacing / max(total, 1e-3))
    if len(ts) == 0:
        ts = np.array([0.9])
    if lod == 2:
        ts = ts[rng.integers(0, 3)::3]
        if len(ts) == 0:
            ts = np.array([0.9])
    P, T = along(ts)
    S_ = _norm(np.cross(T, np.array([0, 0, 1.0])))
    U_ = np.cross(S_, T)
    sign = np.where(np.arange(len(ts)) % 2 == 0, 1.0, -1.0)
    spread = sp["sec_spread"] * D2R * rng.uniform(0.75, 1.2, len(ts))
    envelope = np.minimum(1.0, (1 - ts) / 0.35 + 0.28) * np.minimum(1.0, (ts - start) / 0.12 + 0.5)
    Ls = np.minimum(sp["sec_frac"] * L, sp["sec_max"]) * envelope * rng.uniform(0.75, 1.15, len(ts))
    Ls = np.maximum(Ls, 0.05)
    # low branches let their secondaries hang (Engelmann curtains); upper ones hold flat
    drp = _lerp(sp["sec_droop"][0], sp["sec_droop"][1], rl) * D2R * rng.uniform(0.6, 1.3, len(ts))
    # some secondaries go over and under the branch to thicken the spray
    updn = rng.normal(0, 0.28, len(ts))
    d0 = _norm(np.cos(spread)[:, None] * T + np.sin(spread)[:, None] * sign[:, None] * S_ + updn[:, None] * U_)
    if lod == 2:
        end = P + d0 * Ls[:, None] * 1.25 + np.array([0, 0, -1.0]) * (np.sin(drp) * Ls)[:, None] * 0.7
        w = Ls * 0.82
        side = _norm(np.cross(d0, U_ + rng.normal(0, 0.25, (len(ts), 3))))
        _diamonds(b, P, end, side, w, 0.6 * (1 - ts) * rl + rng.uniform(0, 0.2, len(ts)))
        return
    k_seg = 3
    t3 = np.linspace(0, 1, k_seg + 1)[None, :, None]
    down = np.array([0, 0, -1.0])
    Sec = P[:, None, :] + d0[:, None, :] * Ls[:, None, None] * t3 + down * (np.sin(drp) * Ls)[:, None, None] * t3 ** 2
    rs = (0.0009 + 0.0035 * Ls)[:, None] * np.linspace(1, 0.35, k_seg + 1)[None, :]
    tubes(b, Sec, rs, {0: 3, 1: 2}[lod], 2)
    # shoots along each secondary, both sides, plus the terminal growth
    L_ = LOD[lod]
    tw = sp["twig_space"]
    counts = np.maximum((Ls / tw).astype(int), 1)
    if L_["twig_keep"] < 1:
        counts = np.maximum((counts * L_["twig_keep"]).astype(int), 1)
    sid = np.repeat(np.arange(len(ts)), counts)
    u = (np.concatenate([(np.arange(c) + rng.uniform(0.2, 0.8)) / c for c in counts]))
    u = np.clip(u, 0.03, 1)
    base = P[sid] + d0[sid] * (Ls[sid] * u)[:, None] + down * (np.sin(drp[sid]) * Ls[sid] * u ** 2)[:, None]
    tdir0 = _norm(d0[sid] + 2 * u[:, None] * down * np.sin(drp[sid])[:, None])
    Sx = _norm(np.cross(tdir0, U_[sid]))
    Ux = np.cross(Sx, tdir0)
    m = len(sid)
    sgn = np.where(rng.random(m) < 0.5, 1.0, -1.0)
    twsp = sp["twig_spread"] * D2R * rng.uniform(0.7, 1.2, m)
    # shoot length tapers to the end of the secondary; last few run straight on
    Lt = rng.uniform(*sp["twig_len"], m) * np.where(u > 0.82, 1.2, 1.0) * np.minimum(1.0, 0.5 + (1 - u) * 1.5)
    tl_drp = _lerp(sp["twig_droop"][0], sp["twig_droop"][1], rl) * D2R * rng.uniform(0.5, 1.3, m)
    tdir = _norm(np.cos(twsp)[:, None] * tdir0 + np.sin(twsp)[:, None] * sgn[:, None] * Sx +
                 down * np.sin(tl_drp)[:, None] + rng.normal(0, 0.15, (m, 3)))
    # terminal shoot on each secondary
    term = Sec[:, -1, :]
    tterm = _norm(Sec[:, -1, :] - Sec[:, -2, :])
    base = np.vstack([base, term])
    tdir = np.vstack([tdir, tterm])
    Lt = np.concatenate([Lt, rng.uniform(*sp["twig_len"], len(ts)) * 1.2])
    tprim = np.concatenate([ts[sid], ts])
    uu = np.concatenate([u, np.ones(len(ts))])
    age_tw = np.clip(1.0 - (0.55 * tprim + 0.45 * uu) + rng.normal(0, 0.1, len(base)), 0, 1)
    if L_["twig_k"] == 2:
        tp = np.stack([base, base + tdir * Lt[:, None] * 0.5, base + tdir * Lt[:, None]], 1)
        tr = np.array([0.0013, 0.001, 0.0005])[None] * np.ones((len(base), 1))
        tubes(b, tp, tr, 2, 2)
    _shoot_needles(b, sp, base, tdir, Lt, age_tw, rng, L_)


def _shoot_sprays(b, sp, base, tdir, Lt, age, rng):
    """LOD 1: each shoot is two crossed slim diamonds with the same foliage area as its needles."""
    S, U = _frames(tdir)
    nl = sp["needle_len"] * np.sin(sp["needle_theta"] * D2R)
    width = 2.0 * nl + 0.006
    a = rng.uniform(-0.5, 0.5, len(base))[:, None]
    side = np.cos(a) * S + np.sin(a) * U
    end = base + tdir * (1.3 * Lt + sp["needle_len"] * 0.7)[:, None]
    # three slim crossed blades read as a bottlebrush shoot from any side, not as a broad leaf
    w = np.full(len(base), width * 0.95)
    _diamonds(b, base, end, side, w, age)
    for ang in (2.09, 4.19):
        _diamonds(b, base, end, np.cos(a + ang) * S + np.sin(a + ang) * U, w, age)


def _shoot_needles(b, sp, base, tdir, Lt, age, rng, L_):
    per = np.maximum(Lt / sp["needle_space"] * L_["needle_mult"], 0.0)
    cnt = np.floor(per + rng.random(len(per))).astype(int)
    mode = sp["needle_mode"]
    if mode == "tuft":
        # larch: tufts of 15-25 on spur shoots
        cnt = cnt * max(1, int(18 * max(L_["needle_mult"], 0.25)))
    tid = np.repeat(np.arange(len(base)), cnt)
    if len(tid) == 0:
        return
    u = rng.random(len(tid))
    if mode == "pair":
        u = u ** 0.55  # pine foliage crowds toward the shoot tip
    if mode == "tuft":
        spur = rng.integers(0, 6, len(tid)) / 6.0
        u = spur + rng.uniform(0, 0.012, len(tid))
    d = tdir[tid]
    S, U = _frames(d)
    _needle_geo(b, base[tid] + d * (u * Lt[tid])[:, None], d, S, U,
                sp["needle_len"] * L_["needle_l"], sp["needle_w"] * L_["needle_w"],
                sp["needle_theta"] * D2R, mode, rng, np.clip(age[tid] + 0.1 * (1 - u) + rng.normal(0, 0.05, len(tid)), 0, 1))


def _leader_shoots(b, sp, line, H, rng, lod):
    """Needles on the leader itself, the fine spike at the very top of the spire."""
    L_ = LOD[lod]
    m = 18 if lod == 0 else 5
    z = H - rng.uniform(0.0, min(0.9, 0.08 * H), m) - 0.05
    base = np.stack([_at(line, zz) for zz in z])
    a = rng.uniform(0, 6.28, m)
    tdir = _norm(np.stack([np.cos(a), np.sin(a), np.full(m, 1.6)], -1))
    Lt = rng.uniform(0.05, 0.13, m)
    _shoot_needles(b, sp, base, tdir, Lt, np.zeros(m), rng, L_)
    if lod == 0:
        z2 = np.linspace(H - 0.5, H, 24)
        base2 = np.stack([_at(line, zz) for zz in z2])
        _shoot_needles(b, sp, base2, np.tile([0, 0, 1.0], (24, 1)), np.full(24, 0.03), np.zeros(24), rng, L_)


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


def build_under(kind, seed):
    """Ground-layer plants and debris, in the same array format as a tree. Single LOD."""
    rng = np.random.default_rng(seed)
    b = Builder()
    if kind == "grass":
        n = 70
        a = rng.uniform(0, 2 * np.pi, n)
        r0 = rng.uniform(0, 0.16, n)
        base = np.stack([r0 * np.cos(a), r0 * np.sin(a), np.zeros(n)], -1)
        lean = rng.uniform(0.25, 1.1, n)
        h = rng.uniform(0.22, 0.62, n)
        t = np.linspace(0, 1, 5)[None, :, None]
        d = np.stack([np.cos(a), np.sin(a), np.zeros(n)], -1)[:, None, :]
        P = base[:, None, :] + d * (lean * h)[:, None, None] * t ** 2 + np.array([0, 0, 1.0]) * h[:, None, None] * (t - 0.35 * lean[:, None, None] * t ** 3)
        R = 0.0035 * (1 - t[..., 0] ** 2) + 0.0006
        age = np.where(rng.random(n) < 0.72, 1.0, 0.0)
        # flat ribbon blade, shaded with the needle slot
        tubes(b, P, np.broadcast_to(R, (n, 5)), 2, 1, age=age)
    elif kind in ("shrub", "sapling"):
        sp = dict(SPECIES["aspen"])
        sp.update(needle_len=0.042, needle_w=0.034, needle_space=0.018) if kind == "shrub" else None
        if kind == "sapling":
            arr = build_tree("subalpine", rng.uniform(1.4, 2.6), 1.0, seed, 0)
            return arr
        stems = rng.integers(5, 9)
        for i in range(stems):
            az = rng.uniform(0, 2 * np.pi)
            H_ = rng.uniform(0.55, 1.25)
            pts = branch_line(np.array([rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15), 0.0]), az, rng.uniform(55, 80) * D2R, rng.uniform(5, 35) * D2R, 0, H_, 6, rng, 0.12)
            tubes(b, pts[None], (np.linspace(0.008, 0.002, 7))[None], 3, 2)
            m = rng.integers(7, 14)
            ts = rng.uniform(0.25, 1.0, m)
            idx = (ts * 6).astype(int)
            base = pts[idx]
            a2 = rng.uniform(0, 2 * np.pi, m)
            d = _norm(np.stack([np.cos(a2), np.sin(a2), rng.uniform(0.0, 0.9, m)], -1))
            L = rng.uniform(0.12, 0.34, m)
            P = np.stack([base, base + d * L[:, None] * 0.5, base + d * L[:, None]], 1)
            tubes(b, P, np.array([0.002, 0.0015, 0.0007])[None] * np.ones((m, 1)), 2, 2)
            nl = (L / sp["needle_space"]).astype(int)
            tid = np.repeat(np.arange(m), nl)
            u = rng.random(len(tid))
            S, U = _frames(d[tid])
            _needle_geo(b, base[tid] + d[tid] * (u * L[tid])[:, None], d[tid], S, U, sp["needle_len"], sp["needle_w"], 58 * D2R, "brush", rng, rng.random(len(tid)))
    elif kind == "log":
        L = rng.uniform(5.0, 11.0)
        rad = rng.uniform(0.09, 0.22)
        n = 10
        x = np.linspace(-L / 2, L / 2, n)
        P = np.stack([x, rng.normal(0, 0.03, n).cumsum(), rad * 0.6 + rng.normal(0, 0.01, n)], -1)[None]
        R = np.linspace(rad, rad * 0.45, n)[None]
        tubes(b, P, R, 9, 2)
        for _ in range(rng.integers(5, 12)):
            i = rng.integers(1, n - 1)
            pts = branch_line(P[0, i], rng.uniform(0, 6.28), rng.uniform(-30, 45) * D2R, 0, 0, rng.uniform(0.12, 0.6), 2, rng, 0.2)
            tubes(b, pts[None], np.array([0.016, 0.012, 0.004])[None], 3, 2)
    elif kind == "rock":
        # displaced subdivided octahedron; low relief so it shades correctly under the rock material
        v = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)
        f = np.array([[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]])
        for _ in range(4):
            mids = {}
            nf = []
            vv = list(v)

            def mid(a, c):
                k = (min(a, c), max(a, c))
                if k not in mids:
                    mids[k] = len(vv)
                    vv.append((vv[a] + vv[c]) * 0.5)
                return mids[k]

            for t in f:
                ab, bc, ca = mid(t[0], t[1]), mid(t[1], t[2]), mid(t[2], t[0])
                nf += [[t[0], ab, ca], [t[1], bc, ab], [t[2], ca, bc], [ab, bc, ca]]
            v, f = np.array(vv), np.array(nf)
        v = v / np.linalg.norm(v, axis=1, keepdims=True)
        rs = rng.normal(size=(9, 3))
        rs /= np.linalg.norm(rs, axis=1, keepdims=True)
        amp = 1.0 + 0.0
        disp = np.zeros(len(v))
        for i, q in enumerate(rs):
            disp += np.clip(v @ q - 0.25, 0, 1) * rng.uniform(-0.75, -0.2)  # flat fracture planes
        v = v * (1 + disp)[:, None] * np.array([1.0, rng.uniform(0.6, 1.0), rng.uniform(0.45, 0.8)])
        v[:, 2] -= 0.1
        b.add(v, f, 0, uv=v[:, :2] * 0.5)
    return b.arrays()


def info(arr):
    return dict(verts=len(arr["verts"]), tris=len(arr["faces"]))


if __name__ == "__main__":
    import time
    for name, spc, h, cr, seed in VARIANTS:
        if not name.endswith(("a", "e")):
            continue
        for lod in range(4):
            t0 = time.time()
            a = build_tree(spc, h, cr, seed, lod)
            print(name, "lod", lod, info(a), "%.1fs" % (time.time() - t0), flush=True)
