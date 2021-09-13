from dynamixel_sdk import *  # Uses Dynamixel SDK library
import pid

class Gripper():
    def __init__(self):
        # Control table address
        self.ADDR_PRO_TORQUE_ENABLE = 64  # Control table address is different in Dynamixel model
        self.ADDR_PRO_GOAL_POSITION = 116
        self.ADDR_PRO_PRESENT_POSITION = 132
        self.ADDR_PRO_GOAL_CURRENT = 102
        self.ADDR_PRO_PRESENT_CURRENT = 126

        # Protocol version
        self.PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

        # Default setting
        self.DXL_ID = 1  # Dynamixel ID : 1
        self.BAUDRATE = 115200  # Dynamixel default baudrate : 57600
        self.DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE = 3299  # Dynamixel will rotate between this value
        self.DXL_MAXIMUM_POSITION_VALUE = 4116  # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        self.DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_TORQUE_ENABLE,
                                                                  self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    def close(self,force):

        force_target = force
        dt = 0.005
        kp = 1
        ki = 0.0
        kd = 0.0
        pidt = pid.pid(kp, ki, kd)
        pidt.setSampleTime(dt)
        pidt.outputMax = 100   # safe current = 100
        pidt.target = force_target

        while 1:
            # Read present position 4byte /current 2byte/force
            dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID,
                                                                                           self.ADDR_PRO_PRESENT_CURRENT)
            # update the parameter
            pidt.update(dxl_present_current)


            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            print("[ID:%03d] Goal:%03d  Pres:%03d" % (self.DXL_ID, pidt.target, dxl_present_current))

            # if not abs(self.dxl_goal_position - self.dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD:
            #     break

            # Write new goal position/current
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID,
                                                                           self.ADDR_PRO_GOAL_CURRENT,
                                                                           int(pidt.output))
    def open(self):
        self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID,
                                          self.ADDR_PRO_GOAL_POSITION,
                                          self.DXL_MINIMUM_POSITION_VALUE)

    def disable(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_TORQUE_ENABLE,
                                                                  self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()

    def reconnect(self):
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID,
                                                                       self.ADDR_PRO_TORQUE_ENABLE,
                                                                       self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")