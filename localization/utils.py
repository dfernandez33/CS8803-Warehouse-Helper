import math
import numpy as np
from shapely import geometry
import sys

def rotate_point(x, y, heading_deg):

    c = math.cos(math.radians(heading_deg))
    s = math.sin(math.radians(heading_deg))
    xr = x * c + y * -s
    yr = x * s + y * c
    return xr, yr


def marker_distance(marker_a: tuple, marker_b: tuple):
    x_a = marker_a[0]
    y_a = marker_a[1]

    x_b = marker_b[0]
    y_b = marker_b[1]

    return np.sqrt((x_a - x_b)**2 + (y_a - y_b)**2)


def marker_heading_diff(marker_a: int, marker_b: int) -> int:
    heading_a = marker_a
    heading_b = marker_b

    return (heading_a - heading_b) % 360


