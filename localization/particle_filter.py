import numpy as np
import math
from shapely import geometry
from localization.utils import rotate_point

PARTICLE_COUNT = 5000
ROBOT_CAMERA_FOV_DEG = 69.0
DEPTH_RESOLUTION = (640, 480)


class World:
    def __init__(self, boundary_points: list, obstacle_points: list):

        self.world_polygon = geometry.Polygon(
            [[float(p[0]), float(p[1])] for p in boundary_points]
        )
        self.obstacles = geometry.Polygon(
            [[float(p[0]), float(p[1])] for p in obstacle_points]
        )


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

    def get_scan(self, world: geometry.Polygon, obstacle: geometry.Polygon) -> list:
        curr_h = self.h - ROBOT_CAMERA_FOV_DEG / 2
        delta_h = 10.0
        laser_scan = []

        for _ in range(int(ROBOT_CAMERA_FOV_DEG) // 10):
            infinity_point = geometry.Point(
                self.shapely_point.x + 5000 * math.cos(math.radians(curr_h)),
                self.shapely_point.y + 5000 * math.sin(math.radians(curr_h)),
            )

            laser = geometry.LineString([self.shapely_point, infinity_point])
            intersection_world = laser.intersection(world).bounds
            intersection_world_point = geometry.Point(
                intersection_world[2], intersection_world[3]
            )
            intersection_obs = laser.intersection(obstacle).bounds

            if intersection_obs:
                intersection_obs_point = geometry.Point(
                    intersection_obs[2], intersection_obs[3]
                )
                laser_scan.append(
                    min(
                        self.shapely_point.distance(intersection_world_point),
                        self.shapely_point.distance(intersection_obs_point),
                    )
                )
            else:
                laser_scan.append(self.shapely_point.distance(intersection_world_point))

            curr_h += delta_h

        return laser_scan


class ParticleFilter:
    def __init__(
        self,
        boundary_points: list,
        obstacle_points: list,
        num_particles=5000,
        sigma_rotation=1.0,
        sigma_translation=1.0,
    ):
        self.world = World(boundary_points, obstacle_points)
        self.particles = []
        self.num_particles = num_particles
        # could do average of top-k here instead
        self.top_particle = None

        minx, miny, maxx, maxy = self.world.world_polygon.bounds
        while len(self.particles) < self.num_particles:
            point = geometry.Point(
                np.random.randint(minx, maxx), np.random.randint(miny, maxy)
            )

            if self.world.world_polygon.contains(
                point
            ) and not self.world.obstacles.contains(point):
                self.particles.append(Particle(point, num_sample=self.num_particles))

        self.sigma_rotation = sigma_rotation
        self.sigma_translation = sigma_translation

    def motion_update(self, odom_reading: tuple):
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

    def measurement_update(self, robot_scan: list):
        for particle in self.particles:
            if self.world.world_polygon.contains(
                particle.shapely_point
            ) and not self.world.obstacles.contains(particle.shapely_point):
                particle_scan = particle.get_scan(
                    self.world.world_polygon, self.world.obstacles
                )
                norm = np.linalg.norm(
                    np.array(particle_scan) / 1000 - np.array(robot_scan) / 1000, ord=1
                ) / len(particle_scan)
                likelihood = math.exp(-norm)
                particle.weight = likelihood
            else:
                particle.weight = 0.0

        total_weight = 0.0
        for particle in self.particles:
            total_weight += particle.weight

        if total_weight > 0.0:
            for particle in self.particles:
                particle.weight /= total_weight

        if total_weight != 0:
            self.particles = self._sample_particles(
                existing_particles=self.particles,
                random_particles=self.num_particles // 20,
                num_sample=self.num_particles // 2,
            )
        else:
            self.particles = self._sample_particles(
                existing_particles=None,
                random_particles=self.num_particles,
                num_sample=self.num_particles,
            )

        self.top_particle = max(self.particles, key=lambda p: p.weight)
        return self.particles

    def _sample_particles(
        self, existing_particles: list, random_particles: int, num_sample: int
    ):
        new_particles = []
        minx, miny, maxx, maxy = self.world.world_polygon.bounds
        while len(new_particles) < random_particles:
            point = geometry.Point(
                np.random.randint(minx, maxx), np.random.randint(miny, maxy)
            )
            if self.world.world_polygon.contains(
                point
            ) and not self.world.obstacles.contains(point):
                min_prob = min(
                    [
                        particle.weight
                        for particle in self.particles
                        if particle.weight > 0.0
                    ]
                )
                new_particles.append(Particle(point, num_sample=int(1 / min_prob)))

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

        zipped = zip(probs, particles)
        sorted_part = sorted(zipped, key=lambda k: k[0], reverse=True)
        print(
            "highest likely particles: ",
            [(x[0], x[1].x, x[1].y, x[1].h) for x in sorted_part[0:10]],
        )
        new_particles = np.random.choice(
            new_particles, size=num_sample, replace=True, p=probs
        )
        return new_particles
