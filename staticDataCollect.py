import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import time
import math
import matplotlib.pyplot as plt
from numpy import *
#from angle_change import rotationVectorToEulerAngles



def rotationVectorToEulerAngles(rvecs):
    R = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(rvecs, R)
    sy = math.sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2])
    sz = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return [-x, -y, z]


case =4

cap = cv2.VideoCapture(2)
if case == 3:
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(5, 60)
if case == 1:
    cap.set(3, 1280)
    cap.set(4, 720)
    cap.set(5, 120)
if case ==2:
    cap.set(3, 640)
    cap.set(4, 360)
    cap.set(5, 330)
if case ==4:
    cap.set(3,1280)
    cap.set(4,720)
    cap.set(5,60)
    cap.set(6,cv2.VideoWriter.fourcc('M','J','P','G'))
# calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib_case1.yaml'
# calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib_case2.yaml'
# calib_loc = '/home/dong/PycharmProjects/testDemo/images/calib/calib_case3.yaml'
calib_loc = '/home/changshanshi/Pictures/calibration/calib.yaml'

cv_file = cv2.FileStorage(calib_loc, cv2.FILE_STORAGE_READ)

mtx = cv_file.getNode("camera_matrix").mat()
dist = cv_file.getNode("dist_coeff").mat()
font = cv2.FONT_HERSHEY_SIMPLEX

start_Time = time.perf_counter()
x = []
y = []
z = []
row = []
pitch = []
yaw = []
time_Now = []
timeFindConer = []
timeCaculation = []
totalFlash = 0
usefulFlash = 0

while (True):

    ret, frame = cap.read()
    totalFlash += 1
    if ret == True:


        time1 = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        parameters.adaptiveThreshConstant = 10

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        time2 = time.perf_counter()
        timeFindConer.append(time2-time1)



    if np.all(ids ==6):
        usefulFlash +=1

        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)
        #(rvec-tvec).any() # get rid of that nasty numpy value array error
        time3 = time.perf_counter()
        timeCaculation.append(time3-time2)
        time_Now.append(time.perf_counter()-start_Time)
        x.append(tvec[0][0][0])
        y.append(tvec[0][0][1])
        z.append(tvec[0][0][2])
        # print(tvec[0][0])
        r1,r2,r3 = rotationVectorToEulerAngles(tvec[0][0])

        row.append(r1)
        pitch.append(r2)
        yaw.append(r3)
        print(tvec)
    else:
        pass

    aruco.drawDetectedMarkers(frame, corners)
    # display the resulting frame
    cv2.imshow('frame',frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        end_Time = time.perf_counter()
        totalTime = end_Time - start_Time
        usefulFlashRate = str((usefulFlash/totalFlash)*100) + "%"
        # plt.figure(1)
        # plt.scatter(time_Now, x, label='x', color='r')
        # plt.xlabel('time/s')
        # plt.ylabel('X position/m')
        # plt.title('X virange')
        # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/x.png')
        mean_x = mean(x)
        max_x = max(x)
        min_x = min(x)
        if abs(max_x-mean_x)>= abs(mean_x-min_x):
            vari_x = abs(max_x-mean_x)
        else:
            vari_x = abs(mean_x-min_x)

        print("mean_X: "+str(mean_x))
        print("vari_X: "+str(vari_x))

        # plt.show()

        # plt.figure(2)
        # plt.scatter(time_Now, y, label='y', color='r')
        # plt.xlabel('time/s')
        # plt.ylabel('Y position/m')
        # plt.title('Y virange')
        # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/y.png')
        mean_y = mean(y)
        max_y = max(y)
        min_y = min(y)
        if abs(max_y - mean_y) >= abs(mean_y - min_y):
            vari_y = abs(max_y - mean_y)
        else:
            vari_y = abs(mean_y - min_y)

        print("mean_Y: " + str(mean_y))
        print("vari_Y: " + str(vari_y))
        # plt.show()

        # plt.figure(3)
        # plt.scatter(time_Now, z, label='z', color='r')
        # plt.xlabel('time/s')
        # plt.ylabel('Z position/m')
        # plt.title('Z virange')
        # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/z.png')
        mean_z = mean(z)
        max_z = max(z)
        min_z = min(z)
        if abs(max_z - mean_z) >= abs(mean_z - min_z):
            vari_z = abs(max_z - mean_z)
        else:
            vari_z = abs(mean_z - min_z)

        print("mean_Z: " + str(mean_z))
        print("vari_Z: " + str(vari_z))
        # plt.show()

        # plt.figure(4)
        # plt.scatter(time_Now, row, label='row', color='r')
        # plt.xlabel('time/s')
        # plt.ylabel('Row/radian')
        # plt.title('Row virange')
        # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/Row.png')
        mean_row = mean(row)
        max_row = max(row)
        min_row = min(row)
        if abs(max_row - mean_row) >= abs(mean_row - min_row):
            vari_row = abs(max_row - mean_row)
        else:
            vari_row = abs(mean_row - min_row)

        print("mean_Row: " + str(mean_row))
        print("vari_Row: " + str(vari_row))
        # plt.show()
        #
        # plt.figure(5)
        # plt.scatter(time_Now, pitch, label='pitch', color='r')
        # plt.xlabel('time/s')
        # plt.ylabel('Pitch/radian')
        # plt.title('Pitch virange')
        # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/Pitch.png')
        mean_pitch = mean(pitch)
        max_pitch = max(pitch)
        min_pitch = min(pitch)
        if abs(max_pitch - mean_pitch) >= abs(mean_pitch - min_pitch):
            vari_pitch = abs(max_pitch - mean_pitch)
        else:
            vari_pitch = abs(mean_pitch - min_pitch)

        print("mean_Pitch: " + str(mean_pitch))
        print("vari_Pitch: " + str(vari_pitch))
        # plt.show()
        #
        # plt.figure(6)
        # plt.scatter(time_Now, yaw, label='', color='r')
        # plt.xlabel('time/s')
        # plt.ylabel('yaw/radian')
        # plt.title('yaw virange')
        # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/Yaw.png')
        mean_yaw = mean(yaw)
        max_yaw = max(yaw)
        min_yaw = min(yaw)
        if abs(max_yaw - mean_yaw) >= abs(mean_yaw - min_yaw):
            vari_yaw = abs(max_yaw - mean_yaw)
        else:
            vari_yaw = abs(mean_yaw - min_yaw)

        print("mean_Yaw: " + str(mean_yaw))
        print("vari_Yaw: " + str(vari_yaw))
        # plt.show()
        #
        # plt.figure(7)
        # plt.scatter(time_Now, timeFindConer, label='timeFindConer', color='r')
        # plt.xlabel('flash/number')
        # plt.ylabel('time/s')
        # plt.title('time for find coner')
        # plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/timeFindConer.png')
        mean_timeFindConer = mean(timeFindConer)
        max_timeFindConer = max(timeFindConer)
        min_timeFindConer = min(timeFindConer)
        if abs(max_timeFindConer - mean_timeFindConer) >= abs(mean_timeFindConer - min_timeFindConer):
            vari_timeFindConer = abs(max_timeFindConer - mean_timeFindConer)
        else:
            vari_timeFindConer = abs(mean_timeFindConer - min_timeFindConer)

        print("mean_timeFindConer: " + str(mean_timeFindConer))
        print("vari_timeFindConer: " + str(vari_timeFindConer))
        # plt.show()


        plt.figure(8)
        plt.scatter(time_Now, timeCaculation, label='timeCaculation', color='r')
        plt.xlabel('flash/number')
        plt.ylabel('time/s')
        plt.title('timeCaculation')
        plt.savefig('/home/dong/PycharmProjects/testDemo/images/result/timeCal.png')
        # plt.show()
        mean_timeCaculation = mean(timeCaculation)
        max_timeCaculation = max(timeCaculation)
        min_timeCaculation = min(timeCaculation)
        if abs(max_timeCaculation - mean_timeCaculation) >= abs(mean_timeCaculation - min_timeCaculation):
            vari_timeCaculation = abs(max_timeCaculation - mean_timeCaculation)
        else:
            vari_timeCaculation = abs(mean_timeCaculation - min_timeCaculation)

        print("mean_timeCaculation: " + str(mean_timeCaculation))
        print("vari_timeCaculation: " + str(vari_timeCaculation))

        print("total_time:"+str(totalTime))
        print("usefulFlashRate: "+ str(usefulFlashRate))
        print("totalFlashNum: "+str(totalFlash))
        print("fail flash:"+str(totalFlash-usefulFlash))
        print('distance: ',str(sqrt(mean_x*mean_x+mean_y*mean_y+mean_z*mean_z)))
        break

# When everything done, release the capture
cap.release()

cv2.destroyAllWindows()

