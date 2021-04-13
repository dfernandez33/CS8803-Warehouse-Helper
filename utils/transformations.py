
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




def world2robot(robotPose: tuple, objectPose: tuple) -> tuple:
    #transforms objectPose from world frame to robot frame

    #objectPose: pose of object in world frame
    #robotPose: pose of robot base in world frame

    worldPose = vector3(0,0,0)
    rx, ry, rth = robotPose
    ox, oy, oth = objectPose
    rPose = Pose2(vector3(rx,ry,np.radians(rth)))
    oPose = Pose2(vector3(ox,oy,np.radians(oth)))
    
    return compose(worldPose, rPose, oPose), rPose.transform_to(Point2(ox,oy))

#     w2r = getTransformationMatrix(robotPose)
#     xObject = 

    

# def getTransformationMatrix(pose: tuple(int,int,int)) -> tuple(int,int,int):
#     #get transformation matrix a given pose (x,y,theta)

#     R = np.identity(3)
#     cos = np.cosd(pose[2])
#     sin = np.sind(pose[2])
#     R[0,0] = cos
#     R[1,1] = cos
#     R[0,1] = -sin
#     R[1,0] = sin
#     R[2,1] = pose[1]
#     R[2,0] = pose[0]

#     return R

if __name__ == "__main__":
    robotsPose = (2,2,0)
    objectPose = (4,4,0)
    print(world2robot(robotsPose, objectPose))