import math
import numpy as np
from shapely import geometry 
from localization.particle_filter import Particle


def rotate_point(x, y, heading_deg):
    c = math.cos(math.radians(heading_deg))
    s = math.sin(math.radians(heading_deg))
    xr = x * c + y * -s
    yr = x * s + y * c
    return xr, yr


def get_visible_markers(particle, markers):
    # Return list of markers in the vision cone for particle
    x, y, h = particle.xyh

def marker_distance(marker_a, marker_b):
    coord_a = marker_a[0]
    coord_b = marker_b[0]
    return coord_a.distance(coord_b)


def marker_heeading_diff(marker_a, marker_b):
    heading_a = marker_a[1]
    heading_b = marker_b[1]
    return (heading_a - heading_b) % 360 


def sample_particles(world, existing_particles, random_particles):
    new_particles = []
    minx, miny, maxx, maxy = world.world_polygon.bounds
    while len(new_particles) < random_particles:
        point = geometry.Point(np.randint(minx, maxx), np.randint(miny, maxy))
        if world.world_polygon.contains(point) and not world.obstacles.contains(
            point
        ):
            new_particles.append(Particle(point))
    
    if existing_particles:
        return existing_particles + new_particles
    else:
        return new_particles
    

    
