import math
import logging
import time

from RobotCore.Core import Core
from idl_data.RobotState_data.RobotState_build import RobotState

logging.basicConfig(level=logging.DEBUG)

def main():
    core = Core()

    cmdPub = core.createPublisher(RobotState, "robot_cmd", 1)

    cmdSet = [ # Delay after, vel, angle in radians
        (5, 0.0, 0.0),
        #(5, 0.2, 15*math.pi/180),
        #0.5, 0.0, 0.15),
        #(0.5, 0.0, -0.15),

        (5, 0.2, 0.0),

        #(5.6, 0.2, -21.25*math.pi/180),
        #(0.5, 0.0, -21.25*math.pi/180),

        #(4.75, 0.2, 13*math.pi/180),

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