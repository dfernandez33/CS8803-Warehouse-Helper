import numpy as np
import math
import itertools
from shapely import geometry
from localization.utils import (
    rotate_point,
    marker_distance,
    marker_heading_diff,
)


MARKER_TRANS_SIGMA = 5  # translational err in inch (grid unit)
MARKER_ROT_SIGMA = 25  # rotational err in deg

ROBOT_CAMERA_FOV_DEG = 69.0

class World:

    def __init__(self, boundary_points: list, obstacle_points: list, markers: list):

        self.world_polygon = geometry.Polygon([[float(p[0]), float(p[1])] for p in boundary_points])
        self.obstacles = geometry.Polygon([[float(p[0]), float(p[1])] for p in obstacle_points])
        self.markers = []
        for marker in markers:
            self.markers.append(marker)


class Particle:
    def __init__(self, point: geometry.Point, num_sample=1, weight=1.0, heading=None):
        if heading is None:
            heading = np.random.randint(0, 360)

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

    def get_visible_markers(self, markers: list) -> list:
        # Return list of markers in the vision cone for particle
        marker_list = []
        x, y, h = self.xyh

        for marker in markers:
            marker_x, marker_y, marker_heading = marker
            # rotate marker into robot frame
            marker_x, marker_y = rotate_point(marker_x - x, marker_y - y, -h)
            if math.fabs(math.degrees(math.atan2(marker_y, marker_x))) < ROBOT_CAMERA_FOV_DEG / 2.0:
                marker_heading = marker_heading_diff(marker_heading, h)
                marker_list.append((marker_x, marker_y, marker_heading))

        return marker_list


class ParticleFilter:
    def __init__(
        self,
        boundary_points: list,
        obstacle_points: list,
        markers: list,
        num_particles=5000,
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
            point = geometry.Point(np.random.randint(minx, maxx), np.random.randint(miny, maxy))

            if self.world.world_polygon.contains(
                point
            ) and not self.world.obstacles.contains(point):
                self.particles.append(Particle(point, num_sample=self.num_particles))

        self.sigma_rotation = sigma_rotation
        self.sigma_translation = sigma_translation

    def motion_update(self, odom_reading: tuple, sigma_translation=1.0, sigma_rotation=1.0):
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


    def measurement_update(self, measured_marker_list: list):
        if measured_marker_list:
            for particle in self.particles:
                if self.world.world_polygon.contains(
                    particle.shapely_point
                ) and not self.world.obstacles.contains(particle.shapely_point):
                    particle_markers = particle.get_visible_markers(self.world.markers)
                    gt_markers = measured_marker_list.copy()
                    marker_pairs = []
                    while particle_markers and gt_markers:
                        pairs = itertools.product(particle_markers, gt_markers)

                        particle_marker, gt_marker = min(
                            list(pairs),
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
                        d_h = marker_heading_diff(particle_marker[2], gt_marker[2])

                        exp1 = (d_xy ** 2) / (2 * MARKER_TRANS_SIGMA ** 2)
                        exp2 = (d_h ** 2) / (2 * MARKER_ROT_SIGMA ** 2)

                        likelihood = math.exp(-(exp1 + exp2))
                        prob *= likelihood


                    particle.weight = prob

                else:
                    particle.weight = 0.0
        else:
            for p in self.particles:
                if p.get_visible_markers(self.world.markers):
                    p.weight = 0.0
        # else:
        #     for particle in self.particles:
        #         particle.weight = 1.0 / len(self.particles)

        total_weight = 0.0
        for particle in self.particles:
            total_weight += particle.weight


        if total_weight != 0:
            for particle in self.particles:
                particle.weight /= total_weight
            self.particles = self.sample_particles(
                existing_particles=self.particles,
                random_particles=50,
                num_sample=self.num_particles // 2,
            )
        else:
            self.particles = self.sample_particles(
                existing_particles=None,
                random_particles=self.num_particles,
                num_sample=self.num_particles,
            )

        self.top_particle = max(self.particles, key=lambda p: p.weight)
        return self.particles

    def sample_particles(self, existing_particles: list, random_particles: int, num_sample: int):
        new_particles = []
        minx, miny, maxx, maxy = self.world.world_polygon.bounds
        while len(new_particles) < random_particles:
            point = geometry.Point(np.random.randint(minx, maxx), np.random.randint(miny, maxy))
            if self.world.world_polygon.contains(point) and not self.world.obstacles.contains(point):
                new_particles.append(Particle(point, num_sample=random_particles))

        if existing_particles:
            new_particles = existing_particles + new_particles
            total_weight = 0.0
            for p in new_particles:
                total_weight += p.weight

            for p in new_particles:
                p.weight /= total_weight

        probs = []
        particles = []

        for particle in new_particles:
            probs.append(particle.weight)
            particles.append(particle)
        zipped = zip(probs,particles)
        sortedPart = sorted(zipped,key = lambda k: k[0], reverse = True)
        print("highest likely particles: ",[(x[0],x[1].x, x[1].y, x[1].h) for x in sortedPart[0:10]])
        new_particles = np.random.choice(new_particles, size=num_sample, replace=True, p=probs)
        return new_particles



