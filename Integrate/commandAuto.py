import math
import logging
import time

import queue

import numpy as np
import RobotCore

from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.Command_data.Command_build import Command

logging.basicConfig(level=logging.DEBUG)

def linearTrapezoid(d,vMax,a):
    # Return keypoints in velocity profile
    accelTime = vMax/a
    
    accelD = accelTime*vMax
    dLeft = d - accelD
    
    if(dLeft >= 0):
        
        coastTime = dLeft / vMax
        return np.array([[0.0,0.0], [accelTime, vMax], [accelTime+coastTime, vMax], [accelTime+coastTime+accelTime,0.0]])
    else:
        
        accelTime = math.sqrt(d / a)
        vCoast = accelTime * a
        return np.array([[0.0,0.0], [accelTime, vCoast], [accelTime+accelTime, 0.0]])

def linearInterp(profile, t):
    # Check sorted by time
    if(np.all(np.diff(profile[:,0]) < 0)):
        # Decreasing time
        raise ValueError("Time not sorted")
    
    n = len(profile)
    # Get t section
    t = np.expand_dims(t,0)
    
    profileT = np.expand_dims(profile[:,0], -1)
    profileV = np.expand_dims(profile[:,1], -1)
    
    pre = t >= profileT
    post = t <= profileT
    
    v = np.zeros((t.shape[1]), dtype=np.float)
    
    # Check inside of range
    insideMask = np.logical_and(np.any(post, axis=0), np.any(pre, axis=0))
    
    preInd = n-np.argmax(pre[::-1], axis=0)-1
    postInd = np.argmax(post, axis=0)

    # Same
    sameMask = preInd == postInd
    
    mask = np.logical_and(sameMask, insideMask)
    v[mask] = profile[preInd[mask],1]
    
    # Interpolate Mask
    # Preind always less than postind in sorted list
    interpMask = preInd < postInd
    mask = np.logical_and(interpMask, insideMask)
    tPre = profile[preInd[mask],0]
    tPost = profile[postInd[mask],0]
    
    vPre = profile[preInd[mask],1]
    vPost = profile[postInd[mask],1]
    
    dt = np.divide(t[0][mask] - tPre, tPost - tPre)
    v[mask] = np.multiply(dt, vPost - vPre) + vPre
    
    return v

ROBOT_T = 0.16
ROBOT_L = 0.145
WHEEL_R = 0.065/2

MAX_ACC = 1.5 * WHEEL_R * 2 * math.pi

class CommanderNode(object):

    def __init__(self, dt, interCmdDelay):
        self.core = RobotCore.Core()
        self.cmdPub = self.core.createPublisher(RobotState, "robot_cmd", 1)
        self.pathSub = self.core.createSubscriber(Command, "robot_path", self.updatePathQueue, 10)

        self.dt = dt
        self.interCmdDelay = interCmdDelay
        self.waitSleepTime = 0.5

        self.pathQueue = queue.Queue(maxsize=300)

        while(not self.cmdPub.matched()):
            logging.info("Waiting for cmdPub match...")
            time.sleep(0.5)

        while(not self.pathSub.matched()):
            logging.info("Waiting for pathSub match...")
            time.sleep(0.5)

    def updatePathQueue(self, data: Command.Command):

        if(not self.pathQueue.full()):
            if(data.type() == 'line'):
                self.pathQueue.put(('line', data.distance(), data.velocity()))
            elif(data.type() == 'arc'):
                self.pathQueue.put(('arc', data.desired_angle(), data.turning_radius(), data.velocity()))
    
    def loop(self):

        dt = self.dt

        while True:

            if(not self.pathQueue.empty()):
                cmd = self.pathQueue.get()
                t = 0
                r = 0

                if(cmd[0] == 'line'):
                    d = cmd[1]
                    v = cmd[2]

                    # Correction Tuned
                    if(d > 0):
                        d = 1.007008*d + 0.016116
                    else:
                        d = 0.997737*d - 0.029177
                    

                elif(cmd[0] == 'arc'):
                    r = cmd[2]
                    v = cmd[3]

                    # Correction Tuned 
                    if(r > 0):
                        r = 1.007326*r + 0.022916
                    else:
                        r = 1.02624*r - 0.01276

                    dAngle = cmd[1]
                    d = r*dAngle

                steeringAngle = 0
                if(r != 0):
                    steeringAngle = math.atan(ROBOT_L/r)

                vProfile = linearTrapezoid(abs(d), abs(v), MAX_ACC)

                discreteProfile = linearInterp(vProfile, np.arange(0, vProfile[-1,0] + 0.5, dt))
                if(d < 0):
                    discreteProfile *= -1
                logging.debug(f"Profile:\n{discreteProfile}")
                dProfile = np.sum(discreteProfile) * dt
                logging.debug(f"Area Check:{dProfile}")
                if(cmd[0] == 'arc'):
                    logging.debug(f"Yaw Change Check:{(180/math.pi)*dProfile / r}")
                if(d != dProfile):
                    discreteProfile *= d/dProfile
                if(cmd[0] == 'arc'):
                    logging.debug(f"Yaw Change Check:{(180/math.pi)*np.sum(discreteProfile) * dt / r}")
                logging.debug(f"Area Check (Corrected):{np.sum(discreteProfile) * dt}")

                for c in discreteProfile:

                    data = RobotState.RobotState()
                    data.velocity(c)
                    data.steer_angle(steeringAngle)

                    logging.info(f"Publishing {c} - {steeringAngle}. Waiting {dt} seconds")
                    self.cmdPub.publish(data)

                    time.sleep(dt)
                
                time.sleep(self.interCmdDelay)
            else:
                time.sleep(self.waitSleepTime)

def main():
    dt = 0.4 
    INTER_CMD_DELAY = 1.0
    cmdNode = CommanderNode(dt, INTER_CMD_DELAY)

    try:
        cmdNode.loop()
    except KeyboardInterrupt:
        logging.info("Exiting")

if __name__ == "__main__":
    main()