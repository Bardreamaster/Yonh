import dashboard_client
import rtde_control
import rtde_receive
import rtde_io
from dashboard_client import DashboardClient
import time
from pynput import mouse, keyboard
import pid

import cv2
import numpy as np
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

acceleration = 0.5
dt = 1.0 / 500  # 2ms
joint_home = [-1.15, -1.71, 1.95, -1.51, -1.47, 0.023]
joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ip_ur10 = "192.168.1.10"


def on_press(key):
    print(key);


def on_release(key):
    print(key);


def connect_UR():
    pass


def reconnect_UR():

    global controller, receiver, dashboard
    try:
        if not controller.isConnected():
            controller.reconnect()
        if not receiver.isConnected():
            receiver.reconnect()
        if not dashboard.isConnected():
            dashboard.connect()
    except:
        print('UR connection error!')
        exit()
    else:
        print("UR is online")


def getCurrentCartesian():
    print(receiver.getActualTCPPose())
    pose_now = R.from_rotvec(receiver.getActualTCPPose()[3:6])

    r2 = R.from_matrix([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    pose_now =  pose_now * r2
    pose_now = pose_now.as_euler('XYZ', degrees=False)
    # pose_now[0] = rotation_nojump(pose_now[0])
    print(pose_now)


def getCurrentQ():
    print(receiver.getActualQ())


def protectiveStop():
    controller.triggerProtectiveStop()


def unlockprotectiveStop():
    dashboard.unlockProtectiveStop()


def freedrive():
    controller.teachMode()


def endfreedrive():
    controller.endTeachMode()

def gripper_open():
    iocontroller.setToolDigitalOut(0,False)

def gripper_close():
    iocontroller.setToolDigitalOut(0, True)
    # while True:
    #     t1 = time.time()
    #     iocontroller.setToolDigitalOut(0,True)
    #     t2 = time.time()
    #     dt = t2-t1
    #     if dt<0.002:
    #         time.sleep(dt)
    #     t2 = time.time()
    #     iocontroller.setToolDigitalOut(0,False)
    #     t3 = time.time()
    #     dt = t3 - t2
    #     if dt < 0.0005:
    #         time.sleep(dt)

def rotation_nojump(theta):
    if theta >= 0:
        return (theta - np.pi)
    if theta < 0:
        return (theta + np.pi)

def trackpose(position, pose):
    tcppose = receiver.getActualTCPPose()
    fdbkx = tcppose[0]
    fdbky = tcppose[1]
    fdbkz = tcppose[2]

    pose_now = R.from_rotvec(receiver.getActualTCPPose()[3:6])
    r2 = R.from_matrix([[1,0,0],[0,-1,0],[0,0,-1]])
    pose_now = pose_now * r2
    pose_now = pose_now.as_euler('XYZ', degrees=False)
    fdbkr = pose_now[0]
    fdbkp = pose_now[1]
    fdbkf = pose_now[2]

    pidx.target = position[0]
    pidy.target = position[1]
    pidz.target = position[2]

    pidr.target = pose[0]
    pidp.target = pose[1]
    pidf.target = pose[2]



    pidx.update(fdbkx)
    pidy.update(fdbky)
    pidz.update(fdbkz)
    pidr.update(fdbkr)
    pidp.update(fdbkp)
    pidf.update(fdbkf)

    print('real pose: ', pose_now[0],',',pose_now[1],',',pose_now[2])
    print("target pose: ", pose[0],',',pose[1],',',pose[2])
    print('output ',pidr.output,',',pidp.output,',',pidf.output)
    controller.speedL([pidx.output, pidy.output, pidz.output, pidr.output, pidp.output, pidf.output], acceleration, dt)


def trackCartesian(position):
    tcppose = receiver.getActualTCPPose()
    fdbkx = tcppose[0]
    fdbky = tcppose[1]
    fdbkz = tcppose[2]
    pidx.target = position[0]
    pidy.target = position[1]
    pidz.target = position[2]

    pidx.update(fdbkx)
    pidy.update(fdbky)
    pidz.update(fdbkz)
    controller.speedL([pidx.output, pidy.output, pidz.output, 0, 0, 0], acceleration, dt)

def position_transform(tvector):
    ws_max = [-0.2955759874679728, 0.5219420169792065, 0.5]
    ws_min = [-0.8894115620774988, -0.16660725540676732, -0.04759156539705184]
    # ws_max = [0.3, 0.5, 0.4]
    # ws_min = [-0.3, -0.15, 0.0]
    #print('tvec: ' , tvector)
    rmatrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    # tvec = np.dot(np.array(tvector), rmatrix)
    tvec = np.dot( rmatrix,np.array(tvector))
    tvec = tvec + np.array([-0.6283958, 0.1094664, 0.6])
    position = np.zeros(3)
    if ws_min[0] > tvec[0]:
        position[0] = ws_min[0]
    elif ws_max[0] < tvec[0]:
        position[0] = ws_max[0]
    else:
        position[0] = tvec[0]
    if ws_min[1] > tvec[1]:
        position[1] = ws_min[1]
    elif ws_max[1] < tvec[1]:
        position[1] = ws_max[1]
    else:
        position[1] = tvec[1]
    if ws_min[2] > tvec[2]:
        position[2] = ws_min[2]
    elif ws_max[2] < tvec[2]:
        position[2] = ws_max[2]
    else:
        position[2] = tvec[2]
    return position

def pose_transform(rvec):
    pose = R.from_rotvec(rvec)
    # pose2 =  pose.as_euler("XYZ" , degrees=False)
    # print('xyz', pose2)
    return pose.as_euler("XYZ" , degrees=False)

def moveHome():
    controller.moveJ(joint_home)

def rotax():
    t2 = time.time()
    while 1:
        t1 = time.time()
        controller.speedL([0, 0, 0, 0.06, 0, 0])
        if (t1 - t2 > 3):
            controller.speedStop(5)
            break

def rotay():
    t2 = time.time()
    while 1:
        t1 = time.time()
        controller.speedL([0, 0, 0, 0, 0.06, 0])
        if (t1 - t2 > 3):
            controller.speedStop(5)
            break

def rotaz():
    t2 = time.time()
    while 1:
        t1 = time.time()
        controller.speedL([0, 0, 0, 0, 0, 0.06])
        if (t1 - t2 > 3):
            controller.speedStop(5)
            break

def move():
    t2 = time.time()
    while 1 :
        t1 = time.time()
        controller.speedL([0,0,0,0,0.07,0])
        if (t1-t2 > 3) :
            controller.speedStop(5)
            break

def track():
    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)
    cap.set(4, 720)
    cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(5, 60)
    calib_loc = '/home/changshanshi/Pictures/calibration/calib.yaml'
    cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode("camera_matrix").mat()
    dist = cv_file.getNode("dist_coeff").mat()

    finish = False
    counts = 0
    currentposition = receiver.getActualTCPPose()
    position = currentposition[0:3]
    pose = currentposition[3:6]



    while (not finish):
        time.sleep(0.2)
        start = time.time()

        counts = (counts + 1) % 9
        # if counts == 1:
        if 1:
            ret, frame = cap.read()
            #cv2.imshow('frame', frame)
            ids = 0
            if ret == True:
                blur = cv2.GaussianBlur(frame, (11, 11), 0)
                gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
                aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
                parameters = aruco.DetectorParameters_create()
                parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
                parameters.adaptiveThreshConstant = 10
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if np.all(ids == 4):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
                position = position_transform(tvec[0][0])
                pose = R.from_rotvec(rvec[0][0])
                pose = pose.as_euler("XYZ", degrees=False)
                pose[0] = rotation_nojump(pose[0])
                aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1)
                aruco.drawDetectedMarkers(frame, corners)
                # display the resulting frame

                cv2.waitKey(1)
                cv2.imshow('frame', frame)

                # print(position)
                # print('tvec: ', tvec[0][0])
                # print('pose: ', pose)
            else:
                pass
        # trackCartesian(position)

        trackpose(position,pose)

        end = time.time()
        duration = end - start
        if duration < dt:
            time.sleep(dt - duration)

    controller.speedStop(5)
    controller.stopScript()
    print('Tracking END')


def stopAll():
    receiver.getRobotMode()
    try:
        controller.speedStop(5)
        controller.stopL(5)
        controller.stopJ(5)
        controller.stopScript()
        print('Script Stopped')
    except:
        pass


if __name__ == "__main__":

    global controller, receiver, dashboard
    try:
        controller = rtde_control.RTDEControlInterface(ip_ur10)
        receiver = rtde_receive.RTDEReceiveInterface(ip_ur10)
        dashboard = dashboard_client.DashboardClient(ip_ur10)
        iocontroller = rtde_io.RTDEIOInterface(ip_ur10)
    except:
        print('UR connection error!')
        exit()
    else:
        print("UR is online")

    kp = 1
    kpr = 0.8
    ki = 0.0
    kd = 0.0
    pidx = pid.pid(kp, ki, kd)
    pidy = pid.pid(kp, ki, kd)
    pidz = pid.pid(kp, ki, kd)
    pidx.setSampleTime(dt)
    pidy.setSampleTime(dt)
    pidz.setSampleTime(dt)
    pidx.outputMax = 0.8
    pidy.outputMax = 0.8
    pidz.outputMax = 0.65

    pidr = pid.pid(kpr, ki, kd)
    pidp = pid.pid(kpr, ki, kd)
    pidf = pid.pid(kpr, ki, kd)

    pidr.setSampleTime(dt)
    pidp.setSampleTime(dt)
    pidf.setSampleTime(dt)
    pidr.outputMax = 0.3
    pidp.outputMax = 0.3
    pidf.outputMax = 0.6
    # plt.figure(0)
    # plt.grid(True)

    with keyboard.GlobalHotKeys({
        'h': moveHome,
        't': track,
        'c': getCurrentCartesian,
        'q': getCurrentQ,
        # 'p': protectiveStop,
        # 'u': unlockprotectiveStop,
        'r': reconnect_UR,
        'f': freedrive,
        's': stopAll,
        'm': move,
        '1': rotax,
        '2': rotay,
        '3': rotaz,
        '+': gripper_open,
        '-': gripper_close,
        '<ctrl>+f': endfreedrive,
        '<esc>': exit
    }) as h:
        h.join()