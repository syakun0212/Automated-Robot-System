import math
import logging
import time

from RobotCore.Core import Core
from idl_data.Command_data.Command_build import Command

logging.basicConfig(level=logging.INFO)

delay = 0.1

def main():
    core = Core()

    cmdPub = core.createPublisher(Command, "robot_ctrl", 1)

    cmdSet = [ # Type, data
        #('line', 1.0, 0.2), # ('line', d, v)
        ('arc', -math.pi/2, -0.4, 0.2), # ('arc', change in yaw, r, v)
        ('line', 0.25, 0.2),
        ('arc', -math.pi, -0.4, 0.2),
        ('line', 0.25, 0.2)

    ]

    while(not cmdPub.matched()):
        logging.info("Waiting for match...")
        time.sleep(0.5)

    for cmd in cmdSet:

        data = Command.Command()
        data.type(cmd[0])
        if(cmd[0] == 'line'):
            data.distance(cmd[1])
            data.velocity(cmd[2])
        elif(cmd[0] == 'arc'):
            data.desired_angle(cmd[1])
            data.turning_radius(cmd[2])
            data.velocity(cmd[3])

        logging.info(f"Publishing {data}.")
        cmdPub.publish(data)

        time.sleep(delay)
    

if __name__ == "__main__":
    main()