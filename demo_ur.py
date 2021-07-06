import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import math
import matplotlib.pyplot as plt
from numpy import *
#from angle_change import rotationVectorToEulerAngles

import rtde_control
import rtde_receive
import time
import ikpy


ip = "192.168.1.100"
rtde_c = rtde_control.RTDEControlInterface(ip)
rtde_r = rtde_receive.RTDEReceiveInterface(ip)


# frome euler to Matrix, TCP
target_pose = np.array(position)
temp_euler = RR.from_euler('xyz', target_pose[3:6], degrees=False)
temp_matrix = temp_euler.as_matrix()
target_matrix = np.eye(4)
target_matrix[0:3, 0:3] = temp_matrix
target_matrix[0:3, 3] = target_pose[0:3]
# TCP to Flange
O_T_F = np.dot(target_matrix, np.linalg.pinv(self.F_Matrix))

robot_urdf_path = "~/PycharmProjects/yonh/ur10e.urdf"
my_chain = ikpy.chain.Chain.from_urdf_file(robot_urdf_path, active_links_mask=None)
# base_joint and gripper_joint, so there are 9 joint angles
current_joint = rtde_r.getActualQ()
init_joint = np.zeros(9)
init_joint = current_joint
ik_joint = my_chain.inverse_kinematics(O_T_F, init_joint)
target_joint = ik_joint




homejoint = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]

# Parameters
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300

init_q = rtde_r.getActualQ()
target_q = target_joint
joint_q = init_q




# Move to initial joint position with a regular moveJ
#rtde_c.moveJ(joint_q)

# Execute 500Hz control loop for 2 seconds, each cycle is 2ms
for i in range(1000):
    start = time.time()
    rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
    joint_q = init_q + sign(target_q - init_q) * 0.001
    end = time.time()
    duration = end - start
    if duration < dt:
        time.sleep(dt - duration)

rtde_c.servoStop()
rtde_c.stopScript()



