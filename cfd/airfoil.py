"""Airfoil geometry: read Selig-format coordinates, spline them, resample with clustering.

Pure standard library. Coordinates run TE -> upper -> LE -> lower -> TE in Selig format.
"""
import math


def read_selig(path):
    pts = []
    with open(path) as f:
        for line in f:
            parts = line.replace(",", " ").split()
            if len(parts) < 2:
                continue
            try:
                x, y = float(parts[0]), float(parts[1])
            except ValueError:
                continue
            pts.append((x, y))
    if len(pts) < 10:
        raise ValueError(f"too few coordinates in {path}")
    if pts[0][0] < 0.5:
        raise ValueError(f"{path} is not Selig order (Lednicer needs conversion)")
    return pts


def normalise(pts):
    """Unit chord, LE at origin, TE on y=0; closes a finite trailing edge by shear. Returns (pts, gap)."""
    le = min(pts, key=lambda p: p[0])
    te = (0.5 * (pts[0][0] + pts[-1][0]), 0.5 * (pts[0][1] + pts[-1][1]))
    dx, dy = te[0] - le[0], te[1] - le[1]
    chord = math.hypot(dx, dy)
    c, s = dx / chord, dy / chord
    out = []
    for x, y in pts:
        xr, yr = x - le[0], y - le[1]
        out.append(((xr * c + yr * s) / chord, (-xr * s + yr * c) / chord))
    gap = out[0][1] - out[-1][1]
    closed = []
    n_up = out.index(min(out, key=lambda p: p[0]))
    for k, (x, y) in enumerate(out):
        sign = 1.0 if k <= n_up else -1.0
        closed.append((x, y - sign * 0.5 * gap * x))
    return closed, gap


class Spline:
    """Natural cubic spline of y(t)."""

    def __init__(self, t, y):
        n = len(t)
        self.t, self.y = t, y
        a, b, c, d = [0.0] * n, [1.0] * n, [0.0] * n, [0.0] * n
        for i in range(1, n - 1):
            h0, h1 = t[i] - t[i - 1], t[i + 1] - t[i]
            a[i], b[i], c[i] = h0, 2 * (h0 + h1), h1
            d[i] = 6 * ((y[i + 1] - y[i]) / h1 - (y[i] - y[i - 1]) / h0)
        for i in range(1, n):
            m = a[i] / b[i - 1]
            b[i] -= m * c[i - 1]
            d[i] -= m * d[i - 1]
        self.m = [0.0] * n
        self.m[-1] = d[-1] / b[-1]
        for i in range(n - 2, -1, -1):
            self.m[i] = (d[i] - c[i] * self.m[i + 1]) / b[i]
        self._k = 0

    def __call__(self, v):
        t = self.t
        k = self._k
        if not (t[k] <= v <= t[k + 1]):
            lo, hi = 0, len(t) - 2
            while lo < hi:
                mid = (lo + hi + 1) // 2
                if t[mid] <= v:
                    lo = mid
                else:
                    hi = mid - 1
            k = lo
            self._k = k
        h = t[k + 1] - t[k]
        A, B = (t[k + 1] - v) / h, (v - t[k]) / h
        return (A * self.y[k] + B * self.y[k + 1]
                + ((A ** 3 - A) * self.m[k] + (B ** 3 - B) * self.m[k + 1]) * h * h / 6)


class Surface:
    """Closed-at-TE airfoil curve, parameterised by arc length from the upper TE."""

    def __init__(self, pts, samples=20000):
        t = [0.0]
        for p, q in zip(pts, pts[1:]):
            t.append(t[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
        sx, sy = Spline(t, [p[0] for p in pts]), Spline(t, [p[1] for p in pts])
        self.dense = [(sx(t[-1] * k / samples), sy(t[-1] * k / samples)) for k in range(samples + 1)]
        self.s = [0.0]
        for p, q in zip(self.dense, self.dense[1:]):
            self.s.append(self.s[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
        self.le_index = min(range(len(self.dense)), key=lambda k: self.dense[k][0])

    def at(self, s):
        arr = self.s
        lo, hi = 0, len(arr) - 2
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if arr[mid] <= s:
                lo = mid
            else:
                hi = mid - 1
        f = (s - arr[lo]) / max(arr[lo + 1] - arr[lo], 1e-300)
        p, q = self.dense[lo], self.dense[lo + 1]
        return (p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1]))


def clustered_stations(length, n, h_start, h_end, growth=0.06):
    """n+1 stations on [0, length], spacing h(s)=min(h_start+g s, h_end+g (L-s), h_max), scaled to fit n."""
    def distribute(h_max):
        steps = 4000
        ds = length / steps
        cum = [0.0]
        for k in range(steps):
            s = (k + 0.5) * ds
            h = min(h_start + growth * s, h_end + growth * (length - s), h_max)
            cum.append(cum[-1] + ds / h)
        return cum
    lo, hi = 1e-6, length
    for _ in range(60):
        mid = math.sqrt(lo * hi)
        if distribute(mid)[-1] > n:
            lo = mid
        else:
            hi = mid
    cum = distribute(hi)
    scale = n / cum[-1]
    out, m = [0.0], 1
    ds = length / (len(cum) - 1)
    for k in range(1, n):
        target = k / scale
        while cum[m] < target:
            m += 1
        f = (target - cum[m - 1]) / (cum[m] - cum[m - 1])
        out.append((m - 1 + f) * ds)
    out.append(length)
    return out


def thickness_camber(pts):
    """Max thickness and camber over chord, from a normalised Selig curve."""
    le = min(range(len(pts)), key=lambda k: pts[k][0])
    up = sorted(pts[:le + 1])
    lo = sorted(pts[le:])

    def interp(arr, x):
        for p, q in zip(arr, arr[1:]):
            if p[0] <= x <= q[0] and q[0] > p[0]:
                return p[1] + (x - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
        return arr[-1][1]
    tmax, cmax = 0.0, 0.0
    for k in range(1, 200):
        x = k / 200
        yu, yl = interp(up, x), interp(lo, x)
        tmax, cmax = max(tmax, yu - yl), max(cmax, 0.5 * (yu + yl))
    return tmax, cmax
