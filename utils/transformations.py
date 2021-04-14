
from functools import reduce
import numpy as np
import gtsam
import gtsam.utils.plot as gtsam_plot
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




def world2robot(robotPose, objectPose):
    #transforms objectPose from world frame to robot frame

    #objectPose: pose of object in robot pose
    #robotPose: pose of robot base in world frame

    rx, ry, rth = robotPose
    ox, oy, oth = objectPose
    rPos = Pose2(vector3(rx,ry,))
    rAngle = Pose2(vector3(0,0,np.radians(rth)))
    oPose = Pose2(vector3(ox,oy,np.radians(oth)))
    
    rPose = compose(rPos, rAngle)
    newPose = rPose.transformTo(Point2(ox,oy))
    x, y = newPose.x(), newPose.y()
    print(newPose)
    return x, y, (oth - rth)%360

    

def robot2world(robotPose, objectPose):
    worldPose = Pose2(vector3(0,0,0))
    rx, ry, rth = robotPose

    ox, oy, oth = objectPose
    rPos = Pose2(vector3(rx,ry,0))
    rAngle = Pose2(vector3(0,0,np.radians(rth)))
    oPose = Pose2(vector3(ox,oy,np.radians(oth)))
    
    rPose = compose(rPos, rAngle)
    newPose = rPose.transformFrom(Point2(ox,oy))
    x, y = newPose.x(), newPose.y()
    return x, y, (rth - oth)%360


