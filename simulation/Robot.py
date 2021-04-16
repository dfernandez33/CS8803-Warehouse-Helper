from localization.particle_filter import Particle
from setting import *
import random
random.seed(RANDOM_SEED)
from utils import *
from grid import *


""" Robot class
    A class for robot, contains same x, y, and heading information as particles
    but with some more utitilies for robot motion / collision checking
"""


class Robot(Particle):

    def __init__(self, x, y, h):
        point = geometry.Point(x, y)
        super(Robot, self).__init__(point, heading=h)

    def __repr__(self):
        return "(x = %f, y = %f, heading = %f deg)" % (self.x, self.y, self.h)

    # return a random robot heading angle
    @staticmethod
    def chose_random_heading():
        return random.uniform(0, 360)

    def read_markers(self, grid):
        """ Helper function to simulate markers measurements by robot's camera
            Only markers in robot's camera view (in FOV) will be in the list
            Arguments:
            grid -- map grid with marker information
            Return: robot detected marker list, each marker has format:
                    measured_marker_list[i] = (rx, ry, rh)
                    rx -- marker's relative X coordinate in robot's frame
                    ry -- marker's relative Y coordinate in robot's frame
                    rh -- marker's relative heading in robot's frame, in degree
        """
        detected_markers = []
        for marker in super(Robot, self).read_markers(grid):
            # Drop detected markers randomly according to the detection failure rate
            if random.random() >= DETECTION_FAILURE_RATE:
                detected_markers.append(marker)

            # Randomly add spurious markers
            if random.random() < SPURIOUS_DETECTION_RATE:
                fake_marker_x = random.random() * grid.width / 2.
                fake_marker_y = math.radians(ROBOT_CAMERA_FOV_DEG / 2.0) * (random.random()*2 - 1.) * fake_marker_x
                fake_marker = fake_marker_x, fake_marker_y, (random.random() - 0.5) * 360
                detected_markers.append(fake_marker)

        return detected_markers

    def move(self, odom):
        """ Move the robot with a steering angle and dist drive forward.
            Note that the robot *drive first, then turn head*.
            Arguments:
            odom -- odometry to move (dx, dy, dh) in *robot local frame*
            No return
        """

        dx, dy = rotate_point(odom[0], odom[1], self.h)
        self.x += dx
        self.y += dy
        self.h = self.h + odom[2]

    def check_collsion(self, odom, grid):
        """ Check whether moving the robot will cause collision.
            Note this function will *not* move the robot
            Arguments:
            odom -- odometry to move (dx, dy, dh) in robot local frame
            Return: True if will cause collision, False if will not be a collision
        """
        dx, dy = rotate_point(odom[0], odom[1], self.h)
        if grid.is_free(self.x + dx, self.y + dy):
            return False
        return True
