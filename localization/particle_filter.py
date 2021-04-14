import numpy as np
import math
import itertools
from shapely import geometry
from localization.utils import (
    rotate_point,
    get_visible_markers,
    marker_distance,
    marker_heeading_diff,
    sample_particles,
)


MARKER_TRANS_SIGMA = 5  # translational err in inch (grid unit)
MARKER_ROT_SIGMA = 25  # rotational err in deg


class World:
    def __init__(self, boundary_points, obstacle_points, markers):
        self.world_polygon = geometry.Polygon([[p.x, p.y] for p in boundary_points])
        self.obstacles = geometry.Polygon([[p.x, p.y] for p in obstacle_points])
        self.markers = []
        for marker in markers:
            self.markers.append(marker)


class Particle:
    def __init__(self, point, num_sample=1, weight=1.0, heading=None):
        if heading is None:
            heading = np.randint(0, 360)

        self.shapely_point = point
        self.x = point.x
        self.y = point.y
        self.h = heading
        self.weight = weight / num_sample

    def __repr__(self):
        return "(x = %f, y = %f, heading = %f deg)" % (self.x, self.y, self.h)

    @property
    def xy(self):
        return self.x, self.y

    @property
    def xyh(self):
        return self.x, self.y, self.h


class ParticleFilter:
    def __init__(
        self,
        boundary_points,
        obstacle_points,
        markers,
        num_particles=1000,
        sigma_rotation=1.0,
        sigma_translation=1.0,
    ):
        self.world = World(boundary_points, obstacle_points, markers)
        self.particles = []
        self.num_particles = num_particles
        # could do average of top-k here instead
        self.top_particle = None

        minx, miny, maxx, maxy = self.world.world_polygon.bounds
        while len(self.particles) < self.num_particles:
            point = geometry.Point(np.randint(minx, maxx), np.randint(miny, maxy))
            if self.world.world_polygon.contains(
                point
            ) and not self.world.obstacles.contains(point):
                self.particles.append(Particle(point, num_sample=self.num_particles))

        self.sigma_rotation = sigma_rotation
        self.sigma_translation = sigma_translation

    def motion_update(self, odom_reading, sigma_translation=1.0, sigma_rotation=1.0):
        motion_particles = []

        for particle in self.particles:
            x, y, h = particle.xyh
            dx, dy, dh = odom_reading
            rot_x, rot_y = rotate_point(dx, dy, h)

            new_x = (x + rot_x) + np.random.normal(0.0, scale=self.sigma_translation)
            new_y = (y + rot_y) + np.random.normal(0.0, scale=self.sigma_translation)
            new_h = (h + dh) + np.random.normal(0.0, scale=self.sigma_rotation)

            new_particle = Particle(
                geometry.Point(new_x, new_y),
                weight=particle.weight,
                heading=new_h % 360,
            )
            motion_particles.append(new_particle)

        self.particles = motion_particles

    def measurement_update(self, measured_marker_list, grid):
        if len(measured_marker_list) > 0:
            for particle in self.particles:
                if self.world.world_polygon.contains(
                    particle.shapely_point
                ) and not self.world.obstacles(particle.shapely_point):
                    particle_markers = get_visible_markers(particle, self.world.markers)
                    gt_markers = measured_marker_list
                    marker_pairs = []

                    while len(particle_markers) > 0 and len(gt_markers) > 0:
                        pairs = itertools.product(particle_markers, gt_markers)
                        particle_marker, gt_marker = min(
                            pairs,
                            key=lambda marker_pair: marker_distance(
                                marker_pair[0], marker_pair[1]
                            ),
                        )
                        marker_pairs.append((particle_marker, gt_marker))

                        particle_markers.remove(particle_marker)
                        gt_markers.remove(gt_marker)

                    prob = 1.0
                    for particle_marker, gt_marker in marker_pairs:
                        d_xy = marker_distance(particle_marker, gt_marker)
                        d_h = marker_heeading_diff(particle_marker, gt_marker)

                        exp1 = (d_xy ** 2) / (2 * MARKER_TRANS_SIGMA ** 2)
                        exp2 = (d_h ** 2) / (2 * MARKER_ROT_SIGMA ** 2)

                        likelihood = math.exp(-(exp1 + exp2))
                        prob *= likelihood

                    particle.weight = prob

                else:
                    particle.weight = 0.0

        else:
            for particle in self.particles:
                particle.weight = 1.0 / len(self.particles)

        total_weight = 0.0
        for particle in self.particles:
            total_weight += particle.weight

        if total_weight != 0:
            for particle in self.particles:
                particle.weight /= total_weight
                self.particles = sample_particles(
                    self.world,
                    existing_particles=self.particles,
                    random_particles=50,
                    num_sample=self.num_particles // 2,
                )
        else:
            self.particles = sample_particles(
                self.world,
                existing_particles=None,
                random_particles=self.num_particles,
                num_sample=self.num_particles,
            )

        self.top_particle = max(self.particles, key=lambda p: p.weight)
