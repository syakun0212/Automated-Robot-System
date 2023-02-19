import math
import logging
import time

import RobotCore

from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.IMU_data.IMU_build import IMU
from idl_data.Frame_data.Frame_build import Frame

logging.basicConfig(level=logging.DEBUG)

class ParticleUpdaterNode(object):
    def __init__(self):

        self.core = RobotCore.Core()
        
        self.xDotRead = 0.0
        self.yawRead = 0.0

        self.dYaw = 0.0

        self.stateSub = self.core.createSubscriber(RobotState, "robot_state", self.setReadState, 1)
        self.imuSub = self.core.createSubscriber(IMU, "imu_data", self.setReadIMU, 1)
        self.framePub = self.core.createPublisher(Frame, "particle_frame_update", 1)
    
    def setReadState(self, data: RobotState.RobotState):
        self.xDotRead = data.velocity()
        self.yawRead = data.steer_angle() 

    def setReadIMU(self, data: IMU.IMU):
        self.dYaw = data.angularVelZ()

    def loop(self, rateSleep=0.02):

        PUBLISH_UPDATE_COUNT = 15
        count = 0

        while(not self.stateSub.matched()):
            logging.debug("Waiting for state sub")
            time.sleep(0.2)

        while(not self.imuSub.matched()):
            logging.debug("Waiting for imu sub")
            time.sleep(0.2)

        while(not self.framePub.matched()):
            logging.debug("Waiting for frame Pub")
            time.sleep(0.2)

        lastTime = time.time()

        oldAngle = 0.0
        x = 0
        y = 0
        t = 0

        while(True):
            dt = time.time() - lastTime

            if(count >= PUBLISH_UPDATE_COUNT):

                displace = Frame.Frame()
                displace.x(x)
                displace.y(y)
                displace.yaw(t - oldAngle)
                self.framePub.publish(displace)

                logging.info(f"Current State Change: {x}, {y}, {180*(t-oldAngle)/math.pi}")
                
                x = 0.0
                y = 0.0
                oldAngle = t

                count = 0

            #tTemp = self.yawRead + self.dYaw - oldAngle
            tTemp = self.yawRead - oldAngle
            cT = math.cos(tTemp)
            sT = math.sin(tTemp)

            x += self.xDotRead * cT * dt
            y += self.xDotRead * sT * dt
            t = self.yawRead
            
            count += 1
            lastTime = time.time()

            time.sleep(rateSleep)

def main():
    localizer = ParticleUpdaterNode()
    try:
        localizer.loop()
    except KeyboardInterrupt:
        logging.info("Exiting")
    

if __name__ == "__main__":
    main()