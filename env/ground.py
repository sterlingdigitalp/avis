"""Ground layer near the corridor: grass, meadow carpets, shrubs, logs, boulders, saplings, river cobble, driftwood.
Pure numpy, same array format as env/trees.py (which supplies the geometry primitives). Kept apart from the tree
generator so tuning the ground does not invalidate the multi-GB cached hero tree meshes.
"""
import hashlib
import os

import numpy as np

import trees
from trees import D2R, SPECIES, Builder, _frames, _leaf_geo, _needle_geo, _norm, branch_line, build_tree, tubes

# Understorey, ground layer around the corridor only. name, kind, seed.
UNDER = [
    ("grass_a", "grass", 81), ("grass_b", "grass", 82), ("grass_c", "grass", 83),
    ("shrub_a", "shrub", 84), ("shrub_b", "shrub", 85), ("shrub_c", "shrub", 86),
    ("log_a", "log", 87), ("log_b", "log", 88), ("rock_a", "rock", 89), ("rock_b", "rock", 90), ("rock_c", "rock", 91),
    ("sap_a", "sapling", 92), ("sap_b", "sapling", 93),
    # second pass: continuous meadow, river cobble, driftwood (appended so earlier indices keep their meaning)
    ("meadow_a", "meadow", 94), ("meadow_b", "meadow", 95), ("meadow_c", "meadow", 96),
    ("cobble_a", "cobbles", 97), ("cobble_b", "cobbles", 98), ("cobble_c", "cobbles", 99),
    ("drift_a", "drift", 100), ("drift_b", "drift", 101), ("drift_c", "drift", 102),
]
MEADOW_PATCH = 2.0  # m, side of one meadow carpet tile
COBBLE_PATCH = 1.6  # m, side of one cobble cluster

def _blades(b, base, az, h, lean, width, age, n_seg=4):
    """Arching ribbon grass blades from base points; lean is the tip's horizontal reach as a fraction of height."""
    n = len(base)
    t = np.linspace(0, 1, n_seg + 1)[None, :, None]
    d = np.stack([np.cos(az), np.sin(az), np.zeros(n)], -1)[:, None, :]
    # the blade rises, then its upper part bends over: tips nod, they do not stand like wire
    P = base[:, None, :] + d * (lean * h)[:, None, None] * t ** 2 + np.array([0, 0, 1.0]) * h[:, None, None] * (t - 0.35 * lean[:, None, None] * t ** 3)
    R = width[:, None] * (1 - 0.85 * t[..., 0] ** 1.5) + 0.0004
    tubes(b, P, R, 2, 1, age=age)


def _meadow(b, rng, seed):
    """One MEADOW_PATCH-square tile of late-September montane grassland: rough fescue bunches, finer grass between,
    seed stalks, and a few forb stems. Tiles overlap when scattered, so the carpet has no gaps. age: 1 cured, 0 green."""
    half = MEADOW_PATCH / 2
    lush = {94: 1.0, 95: 0.6, 96: 0.8}.get(seed, 0.8)
    # bunches
    nb = int(rng.integers(26, 40) * lush)
    cx, cy = rng.uniform(-half, half, nb), rng.uniform(-half, half, nb)
    per = rng.integers(25, 75, nb)
    bid = np.repeat(np.arange(nb), per)
    m = len(bid)
    a = rng.uniform(0, 2 * np.pi, m)
    r = rng.uniform(0, 1, m) ** 0.7 * rng.uniform(0.02, 0.07, nb)[bid]
    base = np.stack([cx[bid] + r * np.cos(a), cy[bid] + r * np.sin(a), np.full(m, -0.02)], -1)
    hb = rng.uniform(0.22, 0.5, nb)[bid] * rng.uniform(0.55, 1.1, m) * (0.75 + 0.25 * lush)
    lean = rng.uniform(0.15, 0.9, m) * (0.6 + r / 0.07 * 0.6)
    cured = rng.uniform(0.6, 1.0, nb)[bid]
    age = np.where(rng.random(m) < cured, rng.uniform(0.75, 1.0, m), rng.uniform(0.0, 0.4, m))
    _blades(b, base, a + rng.normal(0, 0.4, m), hb, lean, rng.uniform(0.0012, 0.0024, m), age)
    # fine grass and sedge filling between the bunches
    nf = int(1100 * lush)
    base = np.stack([rng.uniform(-half, half, nf), rng.uniform(-half, half, nf), np.full(nf, -0.02)], -1)
    _blades(b, base, rng.uniform(0, 2 * np.pi, nf), rng.uniform(0.05, 0.22, nf), rng.uniform(0.2, 1.2, nf),
            rng.uniform(0.0008, 0.0016, nf), np.where(rng.random(nf) < 0.8, rng.uniform(0.7, 1.0, nf), rng.uniform(0, 0.3, nf)))
    # seed stalks with a small nodding panicle
    ns = int(rng.integers(20, 45) * lush)
    sb = np.stack([rng.uniform(-half, half, ns), rng.uniform(-half, half, ns), np.full(ns, -0.02)], -1)
    sh = rng.uniform(0.4, 0.85, ns)
    _blades(b, sb, rng.uniform(0, 2 * np.pi, ns), sh, rng.uniform(0.05, 0.3, ns), np.full(ns, 0.0007), np.ones(ns), n_seg=3)
    tip = sb + np.stack([np.zeros(ns), np.zeros(ns), sh * 0.97], -1)
    k = 7
    pid = np.repeat(np.arange(ns), k)
    pa = rng.uniform(0, 2 * np.pi, ns * k)
    pd = _norm(np.stack([np.cos(pa), np.sin(pa), rng.uniform(-1.2, 0.4, ns * k)], -1))
    pl = rng.uniform(0.02, 0.06, ns * k)[:, None]
    P = np.stack([tip[pid], tip[pid] + pd * pl * 0.5, tip[pid] + pd * pl], 1)
    tubes(b, P, np.array([0.0016, 0.0014, 0.0006])[None] * np.ones((ns * k, 1)), 2, 1, age=np.ones(ns * k))
    # forbs: a stem with a few leaves, the asters and yarrow among the grass
    nfb = int(rng.integers(3, 9) * lush)
    for i in range(nfb):
        p0 = np.array([rng.uniform(-half, half), rng.uniform(-half, half), -0.02])
        hh = rng.uniform(0.15, 0.45)
        pts = branch_line(p0, rng.uniform(0, 6.28), rng.uniform(70, 88) * D2R, rng.uniform(0, 20) * D2R, 0, hh, 4, rng, 0.1)
        tubes(b, pts[None], np.linspace(0.0018, 0.0008, 5)[None], 2, 2)
        nl = rng.integers(5, 10)
        idx = rng.integers(1, 5, nl)
        la = rng.uniform(0, 2 * np.pi, nl)
        _leaf_geo(b, pts[idx], np.tile([0, 0, 1.0], (nl, 1)), np.stack([np.cos(la), np.sin(la), np.zeros(nl)], -1),
                  np.tile([0, 0, 1.0], (nl, 1)), 0.035, 0.03, rng, np.full(nl, 0.85))


def _cobbles(b, rng):
    """A COBBLE_PATCH-square cluster of rounded river stones, half bedded in gravel. Discoid and bladed, as fluvial
    clasts are. The per-stone value in `age` picks the lithology in the cobble material."""
    half = COBBLE_PATCH / 2
    n = int(rng.integers(120, 170))
    size = np.clip(np.exp(rng.normal(np.log(0.042), 0.5, n)), 0.012, 0.22)
    c = np.stack([rng.uniform(-half, half, n), rng.uniform(-half, half, n)], -1)
    v0, f0 = _sphere(*ICOSAHEDRON, 2)
    for i in range(n):
        ax = size[i] * np.array([1.0, rng.uniform(0.6, 0.95), rng.uniform(0.35, 0.65)])
        bump = 1 + 0.06 * np.sin(v0 @ rng.normal(size=3) * 3 + rng.uniform(0, 6)) + 0.04 * np.sin(v0 @ rng.normal(size=3) * 5)
        v = v0 * bump[:, None] * ax
        ang = rng.uniform(0, 2 * np.pi)
        ca, sa = np.cos(ang), np.sin(ang)
        tilt = rng.normal(0, 0.15)
        v = v @ np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]]).T
        v[:, 2] += v[:, 0] * tilt
        v = v + np.array([c[i, 0], c[i, 1], ax[2] * rng.uniform(-0.45, 0.05)])
        b.add(v, f0, 0, uv=v[:, :2], age=np.full(len(v), rng.random()))


ICOSAHEDRON = (np.array([[-1, 1.618, 0], [1, 1.618, 0], [-1, -1.618, 0], [1, -1.618, 0], [0, -1, 1.618], [0, 1, 1.618],
                         [0, -1, -1.618], [0, 1, -1.618], [1.618, 0, -1], [1.618, 0, 1], [-1.618, 0, -1], [-1.618, 0, 1]], float),
               [[0, 11, 5], [0, 5, 1], [0, 1, 7], [0, 7, 10], [0, 10, 11], [1, 5, 9], [5, 11, 4], [11, 10, 2], [10, 7, 6], [7, 1, 8],
                [3, 9, 4], [3, 4, 2], [3, 2, 6], [3, 6, 8], [3, 8, 9], [4, 9, 5], [2, 4, 11], [6, 2, 10], [8, 6, 7], [9, 8, 1]])


def _sphere(v0, f0, sub):
    """Unit sphere by repeated midpoint subdivision of a convex polyhedron."""
    v = list(v0 / np.linalg.norm(v0, axis=1, keepdims=True))
    f = f0
    for _ in range(sub):
        mids, nf = {}, []

        def mid(a, c):
            k = (min(a, c), max(a, c))
            if k not in mids:
                mids[k] = len(v)
                p = v[a] + v[c]
                v.append(p / np.linalg.norm(p))
            return mids[k]
        for a, bb, c in f:
            ab, bc, ca = mid(a, bb), mid(bb, c), mid(c, a)
            nf += [[a, ab, ca], [bb, bc, ab], [c, ca, bc], [ab, bc, ca]]
        f = nf
    return np.array(v), np.array(f)


def _driftwood(b, rng):
    """A bleached stranded log: tapered, gently bent, a root wad at the butt and snapped branch stubs."""
    L = rng.uniform(3.0, 9.0)
    rad = rng.uniform(0.11, 0.28)
    n = 14
    x = np.linspace(0, L, n)
    P = np.stack([x - L * 0.35, np.cumsum(rng.normal(0, 0.03, n)), rad * 0.55 + 0.04 * np.sin(x / L * 3) + rng.normal(0, 0.01, n)], -1)
    R = rad * np.linspace(1.0, 0.45, n) ** 1.1
    tubes(b, P[None], R[None], 12, 2, uv_scale=0.9)
    # root wad: thick roots splaying from the butt, curling back
    nr = rng.integers(8, 15)
    for _ in range(nr):
        az = rng.uniform(np.pi / 2, 3 * np.pi / 2) + rng.normal(0, 0.3)
        el = rng.uniform(-0.6, 0.9)
        pts = branch_line(P[0], az, el, rng.uniform(-0.3, 0.3), 0, rng.uniform(0.4, 1.4) * (0.5 + rad * 3), 5, rng, 0.25)
        tubes(b, pts[None], np.linspace(rad * 0.32, rad * 0.05, 6)[None], 6, 2)
    # snapped branch stubs along the stem
    for _ in range(rng.integers(3, 9)):
        i = rng.integers(2, n - 2)
        pts = branch_line(P[i], rng.uniform(0, 6.28), rng.uniform(-0.2, 0.9), 0, 0, rng.uniform(0.1, 0.7), 2, rng, 0.2)
        tubes(b, pts[None], np.array([0.035, 0.026, 0.012]) * (0.5 + rad * 2)[None] if False else np.array([[0.035, 0.026, 0.012]]) * (0.5 + rad * 2), 5, 2)


def _grass(b, rng):
    """A single bunch of cured and green blades: forest openings and meadow edges."""
    n = 70
    a = rng.uniform(0, 2 * np.pi, n)
    r0 = rng.uniform(0, 0.16, n)
    base = np.stack([r0 * np.cos(a), r0 * np.sin(a), np.zeros(n)], -1)
    lean, h = rng.uniform(0.25, 1.1, n), rng.uniform(0.22, 0.62, n)
    age = np.where(rng.random(n) < 0.72, 1.0, 0.0)
    _blades(b, base, a, h, lean, np.full(n, 0.0035), age)


def _shrub(b, rng):
    """Willow and buffaloberry: several leaning stems with small autumn leaves."""
    sp = dict(SPECIES["aspen"], needle_len=0.042, needle_w=0.034, needle_space=0.018)
    for _ in range(rng.integers(5, 9)):
        az = rng.uniform(0, 2 * np.pi)
        H_ = rng.uniform(0.55, 1.25)
        pts = branch_line(np.array([rng.uniform(-0.15, 0.15), rng.uniform(-0.15, 0.15), 0.0]), az, rng.uniform(55, 80) * D2R, rng.uniform(5, 35) * D2R, 0, H_, 6, rng, 0.12)
        tubes(b, pts[None], (np.linspace(0.008, 0.002, 7))[None], 3, 2)
        m = rng.integers(7, 14)
        base = pts[(rng.uniform(0.25, 1.0, m) * 6).astype(int)]
        a2 = rng.uniform(0, 2 * np.pi, m)
        d = _norm(np.stack([np.cos(a2), np.sin(a2), rng.uniform(0.0, 0.9, m)], -1))
        L = rng.uniform(0.12, 0.34, m)
        P = np.stack([base, base + d * L[:, None] * 0.5, base + d * L[:, None]], 1)
        tubes(b, P, np.array([0.002, 0.0015, 0.0007])[None] * np.ones((m, 1)), 2, 2)
        tid = np.repeat(np.arange(m), (L / sp["needle_space"]).astype(int))
        u = rng.random(len(tid))
        S, U = _frames(d[tid])
        _needle_geo(b, base[tid] + d[tid] * (u * L[tid])[:, None], d[tid], S, U, sp["needle_len"], sp["needle_w"], 58 * D2R, "brush", rng, rng.random(len(tid)))


def _log(b, rng):
    """A fallen log with snapped branch stubs, forest floor."""
    L = rng.uniform(5.0, 11.0)
    rad = rng.uniform(0.09, 0.22)
    n = 10
    x = np.linspace(-L / 2, L / 2, n)
    P = np.stack([x, rng.normal(0, 0.03, n).cumsum(), rad * 0.6 + rng.normal(0, 0.01, n)], -1)[None]
    tubes(b, P, np.linspace(rad, rad * 0.45, n)[None], 9, 2)
    for _ in range(rng.integers(5, 12)):
        i = rng.integers(1, n - 1)
        pts = branch_line(P[0, i], rng.uniform(0, 6.28), rng.uniform(-30, 45) * D2R, 0, 0, rng.uniform(0.12, 0.6), 2, rng, 0.2)
        tubes(b, pts[None], np.array([0.016, 0.012, 0.004])[None], 3, 2)


OCTAHEDRON = (np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float),
              [[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]])


def _boulder(b, rng):
    """Angular limestone boulder: a subdivided octahedron cut by flat fracture planes, low relief."""
    v, f = _sphere(*OCTAHEDRON, 4)
    rs = rng.normal(size=(9, 3))
    rs /= np.linalg.norm(rs, axis=1, keepdims=True)
    disp = np.zeros(len(v))
    for q in rs:
        disp += np.clip(v @ q - 0.25, 0, 1) * rng.uniform(-0.75, -0.2)
    v = v * (1 + disp)[:, None] * np.array([1.0, rng.uniform(0.6, 1.0), rng.uniform(0.45, 0.8)])
    v[:, 2] -= 0.1
    b.add(v, f, 0, uv=v[:, :2] * 0.5)


BUILDERS = {"grass": _grass, "shrub": _shrub, "log": _log, "rock": _boulder, "cobbles": _cobbles, "drift": _driftwood}


def build_under(kind, seed):
    """Ground-layer plants and debris, in the same array format as a tree. Single LOD."""
    rng = np.random.default_rng(seed)
    if kind == "sapling":
        return build_tree("subalpine", rng.uniform(1.4, 2.6), 1.0, seed, 0)
    b = Builder()
    if kind == "meadow":
        _meadow(b, rng, seed)
    else:
        BUILDERS[kind](b, rng)
    return b.arrays()


def cache_key(kind):
    """Content key for a cached ground mesh: this file and the tree primitives it borrows."""
    with open(os.path.abspath(__file__)) as fh:
        h = hashlib.sha1(fh.read().encode())
    h.update(trees.cache_key("under:" + kind, 0).encode())
    return h.hexdigest()[:10]
