from robot_manager import RobotManager
from robomaster import robot
from navigation.path_planner import PathPlanner
from utils.transformations import robot2world, world2robot
import time
import numpy as np


def calculate_angle(angle):
    new_angle = angle % 360
    new_angle = (new_angle + 360) % 360
    if new_angle > 180:
        new_angle -= 360

    return new_angle


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    manager = RobotManager(ep_robot)
    time.sleep(1)
    path_planner = PathPlanner()

    start_location = (1117, 203, 0)
    goal_location = (2616, 1117, 0)
    path = path_planner.get_optimal_path(start_location, goal_location)
    print(path)
    curr_global_pose = np.array(start_location, dtype=float)
    for global_waypoint in path:
        pre_movement_odom = manager.get_odometry()
        robot_waypoint = world2robot(tuple(curr_global_pose), global_waypoint)
        manager.move_base(round(robot_waypoint[0] / 1000, 3), round(-robot_waypoint[1] / 1000, 3), calculate_angle(robot_waypoint[2]))
        time.sleep(1)
        post_movement_odom = manager.get_odometry()
        odom_delta = post_movement_odom - pre_movement_odom
        curr_global_pose += np.array((odom_delta[0] * 1000, -odom_delta[1] * 1000, odom_delta[2]))

    manager.shutdown()
