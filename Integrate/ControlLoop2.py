import math
import time
import logging
from Integrate.KalmanLocalize2 import MAX_ACC

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.Frame_data.Frame_build import Frame
from idl_data.Command_data.Command_build import Command

ROBOT_T = 0.16
ROBOT_L = 0.14
WHEEL_R = 0.0625/2

MAX_ACC = 1.5 * WHEEL_R * 2 * math.pi

CHECK_PERC = 0.2
LINE_DEV = 0.02 #Deviation to exit control distance in m
ARC_DEV = 20 * math.pi/180 #Deviation to exit control angle in rad

#Straight prioritize distance
#Arc prioritize angle

logging.basicConfig(level=logging.DEBUG)

class Controller(object):
    def __init__(self):
        self.core = RobotCore.Core()

        self.commandQueue = []

        self.frameData = Frame.Frame()

        self.ctrlSub = self.core.createSubscriber(Command, "robot_ctrl", self.readCommand, 1)
        self.frameSub = self.core.createSubscriber(Frame, "robot_frame_simple", self.readFrame, 1)
        self.cmdPub = self.core.createPublisher(RobotState, "robot_cmd", 1)

    def readCommand(self, data: Command.Command):
        self.commandQueue.append(data)
    
    def readFrame(self, data: Frame.Frame):
        #logging.debug(f"Received Frame Data: {data.x()}, {data.y()}, {data.yaw()}")
        self.frameData.x(data.x())
        self.frameData.y(data.y())
        self.frameData.yaw(data.yaw())

    def controlLoop(self, delay=0.05):
        #sendCmd = RobotState.RobotState()
        #sendCmd.velocity(self.targetVel)
        #sendCmd.steer_angle(curAngleCmd)

        currentCommand = None
        priorState = None

        while(True):

            if(currentCommand is None):
                if(len(self.commandQueue) > 0):
                    currentCommand = self.commandQueue[0]
                    priorState = self.frameData
                    self.commandQueue = self.commandQueue[1:]
            else:
                
                if(currentCommand.type() == 'line'):
                    logging.debug("Starting LINE motion")
                    priorState = self.frameData
                    startYaw = priorState.yaw()
                    xStraight = math.cos(startYaw)
                    yStraight = math.sin(startYaw)

                    v = currentCommand.velocity()
                    d = currentCommand.distance()

                    targetX = priorState.x() + xStraight * d
                    targetY = priorState.y() + yStraight * d

                    while(True):
                        
                        dx = targetX - self.frameData.x()
                        dy = targetY - self.frameData.y()

                        d = dx * xStraight + dy * yStraight

                        sendCmd = RobotState.RobotState()
                        if(abs(d) < LINE_DEV):
                            sendCmd.velocity(0.0)
                            sendCmd.steer_angle(0.0)
                            self.cmdPub.publish(sendCmd)
                            currentCommand = None
                            logging.debug("Finishing LINE motion")
                            break
                        else:
                            t = CHECK_PERC * abs(d/v)
                            logging.debug(f"Remaining Trajectory: [{t}] - {d}m @ {v} m/s Current: {self.frameData.x()}, {self.frameData.y()}, {self.frameData.yaw()}")
                            
                            sendCmd.velocity(v)
                            sendCmd.steer_angle(0.0)
                            time.sleep(t)
                            self.cmdPub.publish(sendCmd)

                elif(currentCommand.type() == 'arc'):
                    logging.debug("Starting ARC motion")
                    priorState = self.frameData
                    startYaw = priorState.yaw()
                    xStraight = math.cos(startYaw)
                    yStraight = math.sin(startYaw)

                    v = currentCommand.velocity()
                    r = currentCommand.turning_radius()

                    targetYaw = startYaw + currentCommand.desired_angle()

                    while(True):
                        
                        
                        dYaw = targetYaw - self.frameData.yaw()
                        if(dYaw < 0):
                            dYaw = -1*(abs(dYaw)%(2*math.pi))
                        else:
                            dYaw = (abs(dYaw)%(2*math.pi))

                        d = dYaw * r
                        logging.debug(f"Target: {targetYaw} Current: {self.frameData.yaw()} dYaw: {dYaw} d: {d} v: {v}")

                        sendCmd = RobotState.RobotState()
                        if(abs(dYaw) < ARC_DEV):
                            sendCmd.velocity(0.0)
                            sendCmd.steer_angle(r)
                            self.cmdPub.publish(sendCmd)
                            logging.debug("Finishing ARC motion")
                            currentCommand = None
                            break
                        else:
                            t = CHECK_PERC * abs(d/v)
                            sendCmd.velocity(v)
                            sendCmd.steer_angle(r)
                            time.sleep(t)
                            self.cmdPub.publish(sendCmd)
                else:
                    logging.info(f"Invalid command type: {currentCommand.type()}")
                    currentCommand = None

            time.sleep(delay)

def main():
    controller = Controller()
    try:
        controller.controlLoop()
    except KeyboardInterrupt:
        logging.info("Exiting")
    

if __name__ == "__main__":
    main()