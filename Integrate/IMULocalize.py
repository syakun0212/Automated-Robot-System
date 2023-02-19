import math
import time
import logging

import RobotCore
from idl_data.IMU_data.IMU_build import IMU
logging.basicConfig(level=logging.DEBUG)

class SimpleLocalizerNode(object):
    def __init__(self):
        self.core = RobotCore.Core()

        self.setState(0,0,0,0,0,0)

        self.xDDotRead = 0.0
        self.yDDotRead = 0.0
        self.wRead = 0.0

        self.stateSub = self.core.createSubscriber(IMU, "imu_data", self.setRead, 1)

    def setRead(self, data: IMU.IMU):
        self.xDDotRead = data.accelerationX()
        self.yDDotRead = data.accelerationY()
        self.wRead = data.angularVelZ()

    def setState(self, x, xDot, y, yDot, t, w):
        self.x = x
        self.xDot = xDot
        self.y = y
        self.yDot = yDot
        self.t = t # in radians
        self.w = w

    def localizeLoop(self, delay=0.015):

        lastTime = time.time()
        while(True):
            dt = time.time() - lastTime
            
            tTemp = self.t
            cT = math.cos(tTemp)
            sT = math.sin(tTemp)

            dt2 = dt*dt

            self.x += self.xDot * dt + 0.5 * dt2 * self.xDDotRead
            self.y += self.yDot * dt + 0.5 * dt2 * self.yDDotRead
            self.t += self.w * dt

            self.xDot += self.xDDotRead * cT * dt 
            self.yDot += self.xDDotRead * sT * dt
            self.w = self.wRead

            lastTime = time.time()

            logging.info(f"Current State: {self.x}, {self.y}, {180*self.t/math.pi}, {self.xDot}, {self.yDot}, {180*self.w/math.pi}")

            time.sleep(delay)

def main():
    localizer = SimpleLocalizerNode()
    try:
        localizer.localizeLoop()
    except KeyboardInterrupt:
        logging.info("Exiting")
    

if __name__ == "__main__":
    main()