import math
import time
import logging

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.IMU_data.IMU_build import IMU
from idl_data.Frame_data.Frame_build import Frame

logging.basicConfig(level=logging.DEBUG)

class SimpleLocalizerNode(object):
    def __init__(self):
        self.core = RobotCore.Core()

        self.setState(0,0,0,0,0,0)

        self.xDotRead = 0.0
        self.yawRead = 0.0
        self.dYaw = 0.0

        self.stateSub = self.core.createSubscriber(RobotState, "robot_state", self.setReadState, 1)
        self.imuSub = self.core.createSubscriber(IMU, "imu_data", self.setReadIMU, 1)
        self.framePub = self.core.createPublisher(Frame, "robot_frame_simple", 1)

    def setReadState(self, data: RobotState.RobotState):
        self.xDotRead = data.velocity()
        self.yawRead = data.steer_angle() 
        #self.wRead = data.steer_angle()

    def setReadIMU(self, data: IMU.IMU):
        self.dYaw = data.angularVelZ()

    def setState(self, x, xDot, y, yDot, t, w):
        self.x = x
        self.xDot = xDot
        self.y = y
        self.yDot = yDot
        self.t = t # in radians
        self.w = w

    def localizeLoop(self, delay=0.02):

        lastTime = time.time()
        while(True):
            dt = time.time() - lastTime
            
            tTemp = self.yawRead + self.dYaw
            cT = math.cos(tTemp)
            sT = math.sin(tTemp)

            self.x += self.xDotRead * cT * dt
            self.y += self.xDotRead * sT * dt
            #self.t += self.w * dt
            self.t = self.yawRead

            self.xDot = self.xDotRead * cT
            self.yDot = self.xDotRead * sT
            self.w = self.dYaw

            lastTime = time.time()

            logging.info(f"Current State: {self.x}, {self.y}, {180*self.t/math.pi}, {self.xDot}, {self.yDot}, {180*self.w/math.pi}")

            sendFrame = Frame.Frame()
            sendFrame.x(self.x)
            sendFrame.y(self.y)
            sendFrame.yaw(self.t)
            self.framePub.publish(sendFrame)

            time.sleep(delay)

def main():
    localizer = SimpleLocalizerNode()
    try:
        localizer.localizeLoop()
    except KeyboardInterrupt:
        logging.info("Exiting")
    

if __name__ == "__main__":
    main()