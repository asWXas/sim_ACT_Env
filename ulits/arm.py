
import time
from .tools import LowPassFilter
import numpy as np
from Robotic_Arm.rm_robot_interface import *



GRIPPER=0
IP_ADDRESS = "192.168.110.119"
PORT = 8080
INITIAL_JOINTS = [0, 0, 0, 0, 0, 0]  # 初始关节角度
GRIPPER_OPEN = 0  # 假设 0 为松开夹爪
GRIPPER_CLOSE = 1  # 假设 1 为闭合夹爪
FILTER_ALPHA = 0.8  # 低通滤波 alpha (0-1, 越小越平滑)
ENABLE_FILTER = True  # 是否启用低通滤波 (True/False)

class RM65:
    def __init__(self):
        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        # 创建机械臂连接
        self.handle = self.arm.rm_create_robot_arm(IP_ADDRESS, PORT)
        print("连接 ID:", self.handle.id)

        # 初始化低通滤波器（如果启用）
        if ENABLE_FILTER:
            self.lpf = LowPassFilter(FILTER_ALPHA)
        else:
            self.lpf = None

        self.arm.rm_movej(INITIAL_JOINTS, 20, 0, 0, 1)  # 关节运动到初始

    def get_joint_angles(self):
        joint_angles = self.arm.rm_get_joint_degree()
        joint_angles = joint_angles[1]
        # 应用低通滤波（如果启用）
        if ENABLE_FILTER:
            filtered_angles = self.lpf.filter(joint_angles)
            filtered_angles = [np.deg2rad(angle) for angle in filtered_angles]
        else:
            filtered_angles = joint_angles
            filtered_angles = [np.deg2rad(angle) for angle in filtered_angles]
            
        return filtered_angles