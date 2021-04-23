from robomaster import robot
from enum import Enum
import numpy as np
import ctypes
import time


class RobotManager:
    def __init__(self, ep_bot: robot):
        # robot control properties
        self.ep_robot = ep_bot
        self.battery_level = 100
        self.gripper_state = GripperState.CLOSED
        self.ee_offset = 76.2
        self.ee_body_pose = np.ones((1, 3))
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
        :param delta_x: Movement along robot's x-axis in m.
        :param delta_y: Movement along robot's y-axis in m.
        :param delta_theta: Rotation along robot's z-axis in degrees.
        :return:
        """
        self.ep_robot.chassis.move(delta_x, delta_y, 0, xy_speed=.6).wait_for_completed()
        self.ep_robot.chassis.move(0, 0, delta_theta, z_speed=90).wait_for_completed()

    def drive_base(self, x_speed, y_speed, rotation_speed, duration=10):
        self.ep_robot.chassis.drive_speed(x_speed, y_speed, rotation_speed, duration)

    def move_end_effector(self):
        pass

    def open_end_effector(self):
        self.ep_robot.gripper.open()
        time.sleep(2)
        self.ep_robot.gripper.pause()

    def close_end_effector(self, duration):
        self.ep_robot.gripper.close()
        time.sleep(duration)
        self.ep_robot.gripper.pause()

    def get_odometry(self):
        x, y, _ = self.curr_chassis_position[0][:]
        theta, _, _ = self.curr_chassis_attitude[0][:]

        return np.array((x, y, theta))

    def shutdown(self):
        self.ep_robot.robotic_arm.unsub_position()
        self.ep_robot.chassis.unsub_attitude()
        self.ep_robot.chassis.unsub_position()
        self.ep_robot.battery.unsub_battery_info()
        self.ep_robot.close()

    def __battery_handler(self, battery_info):
        self.battery_level = battery_info

    def __arm_position_handler(self, sub_info):
        pos_x, pos_y = sub_info
        pos_y = ctypes.c_int32(pos_y).value
        self.ee_body_pose = np.array([[pos_x + self.ee_offset, pos_y, 0]])

    def __chassis_attitude_handler(self, sub_info):
        yaw, pitch, roll = sub_info
        self.curr_chassis_attitude = np.array([[yaw, pitch, roll]])

    def __chassis_position_handler(self, pos_info):
        x, y, z = pos_info
        self.curr_chassis_position = np.array([[x, y, z]])


class GripperState(Enum):
    OPEN = 1
    CLOSED = 2
