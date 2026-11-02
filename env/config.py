"""Site constants shared by every environment script. Pure Python.

World frame (SPEC): x east, y up, z south, metres, origin at ORIGIN_UTM and elevation ORIGIN_ELEV.
Blender frame: X east, Y north, Z up. (x, y, z)_world -> (x, -z, y)_blender.
"""
AOI = (573000, 5615000, 663000, 5705000)  # UTM 11N box of all downloaded data: west, south, east, north
ORIGIN_UTM = (610000.0, 5665000.0)
EPSG = 32611
HR_BOX = (604000, 5654000, 622000, 5672000)  # 2 m lidar window
HR8_BOX = (604000, 5650000, 636000, 5674000)  # 8 m lidar window
# Scene sun: real solar position at the site, 2022-09-21 (Sentinel-2 acquisition date), local morning.
SUN_DATE = (2022, 9, 21)
SUN_MDT_HOURS = 8.70
LAT_LON = (51.123, -115.43)
SHORE_RANGE = 32.0  # metres: shore_l0.png stores distance on land to the lidar waterline / SHORE_RANGE


def utm_to_blender(e, n):
    return e - ORIGIN_UTM[0], n - ORIGIN_UTM[1]


def world_to_blender(p):
    x, y, z = p
    return (x, -z, y)


def blender_to_world(p):
    x, y, z = p
    return (x, z, -y)
