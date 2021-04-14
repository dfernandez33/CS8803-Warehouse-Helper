import math
import numpy as np
from shapely import geometry
from localization.particle_filter import Particle, World


def rotate_point(x: float, y: float, heading_deg: float):
    c = math.cos(math.radians(heading_deg))
    s = math.sin(math.radians(heading_deg))
    xr = x * c + y * -s
    yr = x * s + y * c
    return xr, yr


def get_visible_markers(particle: Particle, markers: list):
    # Return list of markers in the vision cone for particle
    marker_list = []
    x, y, h = particle.xyh
    for marker in markers:
        marker_x, marker_y, marker_heading = marker
        # rotate marker into robot frame
        marker_x, marker_y = rotate_point(marker_x - x, marker_y - y, -h)
        if math.fabs(math.degrees(math.atan2(marker_y, marker_x))) < 62 / 2.0:
            marker_heading = marker_heeading_diff(marker_heading, h)
            marker_list.append((marker_x, marker_y, marker_heading))
    return marker_list


def marker_distance(marker_a: tuple, marker_b: tuple):
    x_a = marker_a[0]
    y_a = marker_a[1]

    x_b = marker_b[0]
    y_b = marker_b[1]

    return np.sqrt((x_a - x_b)**2 + (y_a - y_b)**2)


def marker_heeading_diff(marker_a: tuple, marker_b: tuple):
    heading_a = marker_a[2]
    heading_b = marker_b[2]
    return (heading_a - heading_b) % 360


def sample_particles(world: World, existing_particles: list, random_particles: int, num_sample: int):
    new_particles = []
    minx, miny, maxx, maxy = world.world_polygon.bounds
    while len(new_particles) < random_particles:
        point = geometry.Point(np.randint(minx, maxx), np.randint(miny, maxy))
        if world.world_polygon.contains(point) and not world.obstacles.contains(point):
            new_particles.append(Particle(point, num_sample=random_particles))

    if existing_particles:
        new_particles = existing_particles + new_particles

    probs = []
    particles = []

    for particle in new_particles:
        probs.append(particle.weight)
        particles.append(particle)

    new_particles = np.random.choice(
        new_particles, size=num_sample, replace=False, p=probs
    )
    return new_particles
