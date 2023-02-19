#example of how to retrive obstacle data
import RobotCore
import numpy as np
import idl_data.IMU_data.IMU_build.IMU as IMU
import idl_data.Obstacle_data.Obstacle_build.Obstacle as Obstacle
import idl_data.Message_data.Message_build.Message as Message
import fastdds
import time


def ObstacleReader(data):
    obsX = list(data.obstaclesX())
    obsY = list(data.obstaclesY())
    obsDir = list(data.obstacleDir())
    obsXY = []
    obsXY = np.column_stack((obsX, obsY))
    print(f"obstacle info in 2d array: {obsXY}")
    print(f"Obstacle Direction: {obsDir}")

def MessageReader(data):
    print(f"message: {data.message()}")

def IMUReader(data):
    print(f"data: {data.message()}")

core = RobotCore.Core(0)

obstaclesub = core.createSubscriber(Obstacle, "obstacle_data", ObstacleReader, 10)

messagesub = core.createSubscriber(Message, "message1", MessageReader, 10)

# sub = core.createSubscriber(IMU, "IMU_data1", IMUReader, 10)

try:
    while(True):
        time.sleep(5)
        print("running")
except KeyboardInterrupt:
    print("Exiting")
