import dashboard_client
import rtde_control
import rtde_receive
import rtde_io
from dashboard_client import DashboardClient
import time
from pynput import mouse, keyboard
import pid

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


def moveHome():
    controller.moveJ(joint_home)


def track():
    position = [-0.68845, 0.17467, 0.256]
    finish = False

    while (not finish):
        start = time.time()

        trackCartesian(position)
        currentposition = receiver.getActualTCPPose()
        if (currentposition[0] - position[0] < 0.01) and currentposition[1] - position[1] < 0.01 and currentposition[
            2] - position[2] < 0.01:
            finish = True

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
        '<ctrl>+f': endfreedrive,
        '<esc>': stopAll
    }) as h:
        h.join()

    # with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    #     listener.join()

    # while(True):
    #
    #     pass

    # controller.speedL([-0.05, 0, 0, 0, 0, 0], acceleration, dt)

    # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
    # for i in range(1000):
    #     start = time.time()
    #     #controller.speedL([-0.05,0,0,0,0,0], acceleration, dt)
    #
    #     end = time.time()
    #     duration = end - start
    #     if duration < dt:
    #         time.sleep(dt - duration)
    #
    # controller.speedStop(10)
    # controller.stopScript()
