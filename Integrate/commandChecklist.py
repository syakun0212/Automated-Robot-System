import math
import logging
import time

from RobotCore.Core import Core
from idl_data.RobotState_data.RobotState_build import RobotState

logging.basicConfig(level=logging.DEBUG)

ROBOT_T = 0.16
ROBOT_L = 0.14
WHEEL_R = 0.0625/2

def main():
    core = Core()

    cmdPub = core.createPublisher(RobotState, "robot_cmd", 1)

    forwardVel = 0.2
    changeAngle = math.pi
    turningRadius = -0.3

    acc = 1.5 * WHEEL_R * 2 * math.pi
    accTime = forwardVel / acc
    slopeAngle = math.tan(acc)

    dist = abs(turningRadius * changeAngle)

    accDist = 2*accTime * abs(forwardVel)

    if(accDist > dist):
        topAngle = math.pi - 2*slopeAngle
        l2 = 2 * dist / math.sin(topAngle)
        l = math.sqrt(l2)
        accTime = math.cos(slopeAngle) * l
        forwardVel = math.sin(slopeAngle) * l
        timeUsed = accTime # Total profile - decelerate time = start to finish command
    else:
        remainingDist = dist - accDist
        fullSpeedTime = remainingDist / forwardVel
        timeUsed = fullSpeedTime + accTime
    
    cmdSet = [ # Delay after, vel, angle in radians
        (5, 0.0, 0.0),

        (timeUsed, forwardVel, math.atan(ROBOT_L/turningRadius)),
        (1, 0.0, math.atan(ROBOT_L/turningRadius)),

        (2, 0.0, 0.0)
    ]

    while(not cmdPub.matched()):
        logging.info("Waiting for match...")
        time.sleep(0.5)

    for delay, vel, ang in cmdSet:

        data = RobotState.RobotState()
        data.velocity(vel)
        data.steer_angle(ang)

        logging.info(f"Publishing {vel} - {ang}. Waiting {delay} seconds")
        cmdPub.publish(data)

        time.sleep(delay)
    

if __name__ == "__main__":
    main()