"""NOAA solar position. Pure Python, shared by venv scripts and Blender scripts."""
import math


def solar_az_el(lat_deg, lon_deg, year, month, day, utc_hours):
    """Return (azimuth deg clockwise from north, elevation deg, no refraction). NOAA low-precision formulae."""
    a = (14 - month) // 12
    y = year + 4800 - a
    m = month + 12 * a - 3
    jdn = day + (153 * m + 2) // 5 + 365 * y + y // 4 - y // 100 + y // 400 - 32045
    jd = jdn - 0.5 + utc_hours / 24.0
    t = (jd - 2451545.0) / 36525.0
    l0 = (280.46646 + t * (36000.76983 + t * 0.0003032)) % 360
    mm = 357.52911 + t * (35999.05029 - 0.0001537 * t)
    e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t)
    mr = math.radians(mm)
    c = math.sin(mr) * (1.914602 - t * (0.004817 + 0.000014 * t)) + math.sin(2 * mr) * (0.019993 - 0.000101 * t) + math.sin(3 * mr) * 0.000289
    true_l = l0 + c
    omega = 125.04 - 1934.136 * t
    lam = true_l - 0.00569 - 0.00478 * math.sin(math.radians(omega))
    eps0 = 23 + (26 + (21.448 - t * (46.815 + t * (0.00059 - t * 0.001813))) / 60) / 60
    eps = eps0 + 0.00256 * math.cos(math.radians(omega))
    decl = math.degrees(math.asin(math.sin(math.radians(eps)) * math.sin(math.radians(lam))))
    yy = math.tan(math.radians(eps / 2)) ** 2
    l0r = math.radians(l0)
    eot = 4 * math.degrees(yy * math.sin(2 * l0r) - 2 * e * math.sin(mr) + 4 * e * yy * math.sin(mr) * math.cos(2 * l0r)
                           - 0.5 * yy * yy * math.sin(4 * l0r) - 1.25 * e * e * math.sin(2 * mr))
    tst = (utc_hours * 60 + eot + 4 * lon_deg) % 1440
    ha = tst / 4 - 180
    latr, dr, har = math.radians(lat_deg), math.radians(decl), math.radians(ha)
    cosz = math.sin(latr) * math.sin(dr) + math.cos(latr) * math.cos(dr) * math.cos(har)
    cosz = max(-1.0, min(1.0, cosz))
    z = math.acos(cosz)
    el = 90 - math.degrees(z)
    az = math.degrees(math.atan2(-math.sin(har) * math.cos(dr), math.cos(latr) * math.sin(dr) - math.sin(latr) * math.cos(dr) * math.cos(har)))
    return az % 360, el
