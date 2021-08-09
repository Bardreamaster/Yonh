import grpc

import robot_pb2
import robot_pb2_grpc



if __name__ == '__main__':
    channel = grpc.insecure_channel('10.20.48.159:7000')
    stub = robot_pb2_grpc.MachineControllerStub(channel)

    stub.MoveJointPosition()
