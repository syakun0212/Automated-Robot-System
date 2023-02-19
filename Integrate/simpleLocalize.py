import math
import time
import logging

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState

logging.basicConfig(level=logging.DEBUG)

class SimpleLocalizerNode(object):
    def __init__(self):
        self.core = RobotCore.Core()

        self.setState(0,0,0,0,0,0)

        self.xDotRead = 0.0
        self.wRead = 0.0

        self.stateSub = self.core.createSubscriber(RobotState, "robot_state", self.setRead, 1)

    def setRead(self, data: RobotState.RobotState):
        self.xDotRead = data.velocity()
        self.wRead = data.steer_angle()

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
            
            tTemp = self.t
            cT = math.cos(tTemp)
            sT = math.sin(tTemp)

            self.x += self.xDot * dt
            self.y += self.yDot * dt
            self.t += self.w * dt

            self.xDot = self.xDotRead * cT
            self.yDot = self.xDotRead * sT
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