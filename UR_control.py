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
import cv2
import cv2.aruco as aruco
import glob
import os
import math
import matplotlib.pyplot as plt
from numpy import *

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
    print('tvec: ' , tvector)
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

def moveHome():
    controller.moveJ(joint_home)


def track():
    cap = cv2.VideoCapture(2)
    cap.set(3, 1280)
    cap.set(4, 720)
    cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(5, 60)
    calib_loc = '/home/changshanshi/Pictures/calibration/calib.yaml'
    cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode("camera_matrix").mat()
    dist = cv_file.getNode("dist_coeff").mat()

    #position = [-0.68845, 0.17467, 0.256]
    finish = False
    counts = 0
    currentposition = receiver.getActualTCPPose()
    position = currentposition[0:3]
    while (not finish):
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

                #aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1)
                aruco.drawDetectedMarkers(frame, corners)
                # display the resulting frame
                cv2.waitKey(1)
                cv2.imshow('frame', frame)
                print(position)
            else:
                pass
        trackCartesian(position)
        currentposition = receiver.getActualTCPPose()
        # if (currentposition[0] - position[0] < 0.01) and currentposition[1] - position[1] < 0.01 and currentposition[
        #    2] - position[2] < 0.01:
            # finish = True

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
    except:
        print('UR connection error!')
        exit()
    else:
        print("UR is online")

    kp = 1
    ki = 0.0
    kd = 0.0
    pidx = pid.pid(kp, ki, kd)
    pidy = pid.pid(kp, ki, kd)
    pidz = pid.pid(kp, ki, kd)
    pidx.setSampleTime(dt)
    pidy.setSampleTime(dt)
    pidz.setSampleTime(dt)

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
        '<ctrl>+f': endfreedrive,
        '<esc>': exit
    }) as h:
        h.join()