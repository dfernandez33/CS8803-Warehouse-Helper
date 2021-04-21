from robomaster import robot
from enum import Enum
import numpy as np
import ctypes


class RobotManager:
    def __init__(self, ep_bot: robot):
        # robot control properties
        self.ep_robot = ep_bot
        self.battery_level = 100
        self.gripper_state = GripperState.CLOSED
        self.ee_offset = 76.2
        self.ee_body_pose = np.ones((1, 3))
        self.prev_chassis_attitude = np.ones((1, 3))
        self.prev_chassis_position = np.ones((1, 3))
        self.curr_chassis_attitude = np.ones((1, 3))
        self.curr_chassis_position = np.ones((1, 3))
        self.ep_robot.robotic_arm.sub_position(freq=20, callback=self.__arm_position_handler)
        self.ep_robot.chassis.sub_attitude(freq=20, callback=self.__chassis_attitude_handler)
        self.ep_robot.chassis.sub_position(freq=20, callback=self.__chassis_position_handler)
        self.ep_robot.battery.sub_battery_info(freq=1, callback=self.__battery_handler)

    def move_base(self, delta_x, delta_y, delta_theta):
        """
        Move the robot by the desired amount in each direction. All units are in mm and should be in robot coordinate
        frame.
        :param delta_x: Movement along robot's x-axis in mm.
        :param delta_y: Movement along robot's y-axis in mm.
        :param delta_theta: Rotation along robot's z-axis in degrees.
        :return:
        """
        self.ep_robot.chassis.move(delta_x, delta_y, delta_theta, xy_speed=0.3, z_speed=90).wait_for_completed()

    def move_end_effector(self):
        pass

    def open_end_effector(self):
        pass

    def close_end_effector(self):
        pass

    def get_odometry(self):
        delta_pose = self.curr_chassis_position - self.prev_chassis_position
        delta_attitude = self.curr_chassis_attitude - self.prev_chassis_attitude
        x, y, _ = self.curr_chassis_position[0][:]
        theta, _, _ = self.curr_chassis_attitude[0][:]
        dx, dy, _ = delta_pose[0][:]
        dtheta, _, _ = delta_attitude[0][:]

        return x, y, theta, dx, dy, dtheta

    def __battery_handler(self, battery_info):
        self.battery_level = battery_info

    def __arm_position_handler(self, sub_info):
        pos_x, pos_y = sub_info
        pos_y = ctypes.c_int32(pos_y).value
        self.ee_body_pose = np.array([[pos_x + self.ee_offset, pos_y, 0]])

    def __chassis_attitude_handler(self, sub_info):
        self.prev_chassis_attitude = self.curr_chassis_attitude
        yaw, pitch, roll = sub_info
        self.curr_chassis_attitude = np.array([[yaw, pitch, roll]])

    def __chassis_position_handler(self, pos_info):
        self.prev_chassis_position = self.curr_chassis_position
        x, y, z = pos_info
        self.curr_chassis_position = np.array([[x, y, z]])


class GripperState(Enum):
    OPEN = 1
    CLOSED = 2
