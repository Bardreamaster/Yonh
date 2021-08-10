import time
from pynput import mouse, keyboard
import cv2
import numpy as np
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation as R
from asystmachine.controller import run_controller
from asystmachine.joint import get_joints
import pid

address = '10.20.48.159:7000'
acceleration = 1000
dt = 1.0 / 200  # 20ms


def getposition():
    with run_controller(address) as ctl:
        position = [0.0, 0.0, 0.0]
        joints = get_joints()
        position[0] = joints[0].get_status().position
        position[1] = joints[2].get_status().position
        position[2] = joints[1].get_status().position
        print('real position:   ',format(position[0],'.2f'),',',format(position[1],'.2f'),',',format(position[2],'.2f'))
        return position


def rotation_nojump(theta):
    if theta >= 0:
        return (theta - np.pi)
    if theta < 0:
        return (theta + np.pi)

def move_speed(vvec):
    with run_controller(address) as ctl:
        joints = get_joints()
        joints[0].go_velocity(vvec[0])
        joints[2].go_velocity(vvec[1])
        joints[1].go_velocity(vvec[2])


def position_transform(tvector):
    # 393 325 468
    ws_min = [-190,-150,-200]
    ws_max = [190,150,200]
    #print('tvec: ' , tvector)
    rmatrix = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, 1]])
    tvec = np.dot( rmatrix,np.array(tvector))
    # tvec = tvec + np.array([-0.6283958, 0.1094664, 0.6])
    tvec = np.dot(400, tvec)
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


def trackCartesian(position):
    tvec = getposition()
    fdbkx = tvec[0]
    fdbky = tvec[1]
    fdbkz = tvec[2]
    pidx.target = position[0]
    # pidx.target = -100
    pidy.target = position[1]
    # pidy.target = -100
    pidz.target = position[2]

    pidx.update(fdbkx)
    pidy.update(fdbky)
    pidz.update(fdbkz)

    # move_speed([pidx.output,pidy.output,pidz.output])
    move_speed([pidx.output,pidy.output,0])


def track():
    cap = cv2.VideoCapture(2)
    cap.set(3, 1280)
    cap.set(4, 720)
    cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(5, 60)
    calib_loc = './config/calib60.yaml'
    cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode("camera_matrix").mat()
    dist = cv_file.getNode("dist_coeff").mat()

    finish = False
    counts = 0
    currentposition = getposition()
    position = currentposition[0:3]
    pose = currentposition[3:6]

    with run_controller(address) as ctl:
        joints = get_joints()
        for j in joints:
            j.enable()

    while (not finish):
        # time.sleep(0.2)
        start = time.time()

        # counts = (counts + 1) % 9
        # if counts == 1:
        if 1:
            ret, frame = cap.read()
            # cv2.imshow('frame', frame)
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
                # pose = R.from_rotvec(rvec[0][0])
                # pose = pose.as_euler("XYZ", degrees=False)
                # pose[0] = rotation_nojump(pose[0])
                # aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1)
                # aruco.drawDetectedMarkers(frame, corners)
                # display the resulting frame

                # cv2.waitKey(1)
                # cv2.imshow('frame', frame)

                print('target        :   ',format(position[0],'.2f'),',',format(position[1],'.2f'),',',format(position[2],'.2f'))
                # print('tvec: ', tvec[0][0])
                # print('pose: ', pose)
            else:
                pass

        trackCartesian(position)

        end = time.time()
        duration = end - start
        if duration < dt:
            time.sleep(dt - duration)

    print('Tracking END')


def stopAll():
    with run_controller(address) as ctl:
       joints = get_joints()
       for j in joints:
            j.stop()


def restart():
    joints = get_joints()
    #   disable all
    for j in joints:
        j.disable()
    #   enable all
    for j in joints:
        j.enable()

    print('joins enable')

if __name__ == '__main__':

    try:
        controller = run_controller(address)
    except:
        print('Connection error!')
        exit()
    else:
        print("Connected.")
        # joints = get_joints()
        # for j in joints:
        #     j.enable()
    kp = 2.0
    # kpr = 0.8
    ki = 0.0
    kd = 0.0
    pidx = pid.pid(kp, ki, kd)
    pidy = pid.pid(kp, ki, kd)
    pidz = pid.pid(kp, ki, kd)
    pidx.setSampleTime(dt)
    pidy.setSampleTime(dt)
    pidz.setSampleTime(dt)
    pidx.outputMax = 200
    pidy.outputMax = 200
    pidz.outputMax = 200

    # pidr = pid.pid(kpr, ki, kd)
    # pidp = pid.pid(kpr, ki, kd)
    # pidf = pid.pid(kpr, ki, kd)

    # pidr.setSampleTime(dt)
    # pidp.setSampleTime(dt)
    # pidf.setSampleTime(dt)
    # pidr.outputMax = 0.3
    # pidp.outputMax = 0.3
    # pidf.outputMax = 0.6

    with keyboard.GlobalHotKeys({
        # 'h': moveHome,
        't': track,
        'r': restart,
        # 'c': getCurrentCartesian,
        # 'q': getCurrentQ,
        # # 'p': protectiveStop,
        # # 'u': unlockprotectiveStop,
        # 'r': reconnect_UR,
        # 'f': freedrive,
        's': stopAll,
        # 'm': move,
        # '1': rotax,
        # '2': rotay,
        # '3': rotaz,
        # '+': gripper_open,
        # '-': gripper_close,
        # '<ctrl>+f': endfreedrive,
        '<esc>': exit
    }) as h:
        h.join()
