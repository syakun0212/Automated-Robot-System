import math
import time
import logging

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.Frame_data.Frame_build import Frame

logging.basicConfig(level=logging.DEBUG)

class Controller(object):
    def __init__(self):
        self.core = RobotCore.Core()

        self.targetVel = 0.0
        self.targetAngle = 0.0

        self.frameData = Frame.Frame()

        self.ctrlSub = self.core.createSubscriber(RobotState, "robot_ctrl", self.readCtrl, 1)
        self.frameSub = self.core.createSubscriber(Frame, "robot_frame", self.readFrame, 1)
        self.cmdPub = self.core.createPublisher(RobotState, "robot_cmd", 1)

    def readCtrl(self, data: RobotState.RobotState):
        self.targetVel = data.velocity()
        self.targetAngle = data.steer_angle()
    
    def readFrame(self, data: Frame.Frame):
        self.frameData = data

    def controlLoop(self, delay=0.05):
        curAngleCmd = 0.0
        angleGain = 0.2

        #sendCmd = RobotState.RobotState()
        #sendCmd.velocity(self.targetVel)
        #sendCmd.steer_angle(curAngleCmd)

        while(True):
            angleError = self.targetAngle - self.frameData.yaw()
            curAngleCmd += angleGain * angleError * self.targetVel

            print(f"Current Angle:{self.frameData.yaw():.03f} Target Angle: {self.targetAngle:.03f} Angle Cmd: {curAngleCmd:.03f} Velocity Cmd: {self.targetVel:.03f}")
            
            sendCmd = RobotState.RobotState()
            sendCmd.velocity(self.targetVel)
            sendCmd.steer_angle(curAngleCmd)
            logging.debug(self.cmdPub.publish(sendCmd))
            time.sleep(delay)
            

def main():
    controller = Controller()
    try:
        controller.controlLoop()
    except KeyboardInterrupt:
        logging.info("Exiting")
    

if __name__ == "__main__":
    main()