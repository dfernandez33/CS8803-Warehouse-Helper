import math


def rotate_point(x, y, heading_deg):
    c = math.cos(math.radians(heading_deg))
    s = math.sin(math.radians(heading_deg))
    xr = x * c + y * -s
    yr = x * s + y * c
    return xr, yr


def get_visible_markers(particle, markers):
    pass


def marker_distance(marker_a, marker_b):
    pass


def marker_heeading_diff(marker_a, marker_b):
    pass


def sample_particles(existing_particles, random_particles):
    pass
