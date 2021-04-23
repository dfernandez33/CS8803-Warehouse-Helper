from functools import reduce
import numpy as np
from gtsam import Pose2
from gtsam import Point2


def vector3(x, y, z):
    """Create 3D double numpy array."""
    return np.array([x, y, z], dtype=np.float)


def compose(*poses):
    """Compose all Pose2 transforms given as arguments from left to right."""
    return reduce((lambda x, y: x.compose(y)), poses)


def vee(M):
    """Pose2 vee operator."""
    return vector3(M[0, 2], M[1, 2], M[1, 0])


def delta(g0, g1):
    """Difference between x,y,,theta components of SE(2) poses."""
    return vector3(g1.x() - g0.x(), g1.y() - g0.y(), g1.theta() - g0.theta())


def world2robot(robotPose: tuple, objectPose: tuple) -> tuple:
    # transforms objectPose from world frame to robot frame
    # objectPose: pose of object in robot pose
    # robotPose: pose of robot base in world frame

    rx, ry, rth = robotPose
    ox, oy, oth = objectPose
    rPose = Pose2(vector3(rx, ry, np.radians(rth)))
    oPose = Pose2(vector3(ox, oy, np.radians(oth)))
    
    newPose = rPose.transformTo(Point2(ox, oy))
    return newPose[0], newPose[1], (oth - rth) % 360


def robot2world(robotPose, objectPose):
    """
    :param robotPose: robot pose in worldframe
    :param objectPose: object pose in robotframe
    :return:
    """
    rx, ry, rth = robotPose

    ox, oy, oth = objectPose
    rPos = Pose2(vector3(rx, ry, 0))
    rAngle = Pose2(vector3(0, 0, np.radians(rth)))
    oPose = Pose2(vector3(ox, oy, np.radians(oth)))
    
    rPose = compose(rPos, rAngle)
    newPose = rPose.transformFrom(Point2(ox, oy))
    x, y = newPose.x(), newPose.y()
    return x, y, (rth - oth) % 360

def projectToRange(dh: float):
    """
    :param dh: angle in range [0, 360]
    :return: angle in range [-180,180]
    """
    while dh > 180:
        dh -= 360
    while dh <= -180:
        dh += 360
    return dh

def getAngleDiff(efAngle: tuple,objectName: tuple):
    """
    gets the angle difference
    :param pose: angle of end effector in worldframe
    :param objectName: string for either bottle, cup, or ball
    :return:
    """
    if objectName == 'bottle':
        angle = 180
    if objectName == 'cup':
        angle = 90
    if objectName == 'ball':
        angle = 270

    return projectToRange(angle - efAngle + 180)

def camera2EF(pose: tuple):
    """

    :param pose: pose (x,y,z) of object in camera frame
    :param robotPose: pose of robotBase (x,y,theta) in world frame
    :return: pose of object in end effector frame - x,y,z. This is essentially how much
    SPIKE's gripper  must shift (dx, dy, dz) to reach object cordinate
    """
    cx, cy, cz = pose
    return cz, -cy, cx



if __name__ == '__main__':
    cameraPose = (23,-24,18)
    robotPose = (4,4,30)

    efPose = camera2EF(cameraPose,robotPose)

