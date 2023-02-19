import math
import logging
import time

import numpy as np

from RobotCore.Core import Core
from idl_data.RobotState_data.RobotState_build import RobotState

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
INTER_CMD_DELAY = 1

def main():
    core = Core()

    dt = 0.4

    cmdPub = core.createPublisher(RobotState, "robot_cmd", 1)

    cmdSet = [ # Type, data
        #('line', 1.2, 0.2), # ('line', d, v)
        #('arc', math.pi/2, 0.4, 0.2), # ('arc', change in yaw, r, v)
        #('line', 0.25, 0.2),
        #('arc', -math.pi, -0.4, 0.2),
        #('line', 0.25, 0.2)

        ('line', 0.1, 0.2),
        ('arc', math.pi/2, 0.4, 0.2),

        ('line', -0.05, -0.2),

        ('arc', -math.pi/2, -0.4, 0.2),
        ('line', 1.4, 0.2),
        ('arc', -math.pi/2, -0.4, 0.2),

        ('line', 0.87, 0.2), #0.8 + 0.07

        ('arc', -math.pi/2, -0.4, 0.2),
        ('line', 1.4, 0.2),
        ('arc', -math.pi/2, -0.4, 0.2),

        ('line', -0.05, -0.2),

        ('arc', math.pi/2, 0.4, 0.2),

    ]

    while(not cmdPub.matched()):
        logging.info("Waiting for match...")
        time.sleep(0.5)

    for cmd in cmdSet:

        t = 0
        r = 0

        if(cmd[0] == 'line'):
            d = cmd[1]
            v = cmd[2]

            if(d > 0):
                d = 1.007008*d + 0.016116
            else:
                d = 0.997737*d - 0.029177
            

        elif(cmd[0] == 'arc'):
            r = cmd[2]
            v = cmd[3]

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
            cmdPub.publish(data)

            time.sleep(dt)
        
        time.sleep(INTER_CMD_DELAY)
    

if __name__ == "__main__":
    main()