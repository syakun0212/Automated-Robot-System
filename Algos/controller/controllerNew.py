# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:31:15 2022

@author: Yakun
"""

import sys
import time
import math

import logging

import numpy as np
from sympy import Ci 

from rrtMultDir import *
from utils import * 
from rrt_main import *
from rrt_skipped import rrt_skipped

import RobotCore
from idl_data.Obstacle_data.Obstacle_build import Obstacle
from idl_data.Message_data.Message_build import Message
from idl_data.Command_data.Command_build import Command
from idl_data.Frame_data.Frame_build import Frame 
from idl_data.RobotState_data.RobotState_build import RobotState
import time

logging.basicConfig(level=logging.DEBUG)

## PATH INTERPOLATION

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

#################### 1. [ANDROID]RECEIVE COORDINATES FROM ANDROID #################### 

obstacles_input = None
startstop = None
start = None 

def ObstacleReader(data):
    global obstacles_input

    obsX = list(data.obstaclesX())
    obsY = list(data.obstaclesY())
    obsDir = list(data.obstacleDir())
    # print(f"Obstacle Direction: {obsDir}")
    obstacles_input = []

    for x,y,d in zip(obsX, obsY, obsDir):
        obstacles_input.append([x*10,y*10,d])
    print(f"Full Obstacle Input: {obstacles_input}")



def MesssageReader(data):
    global startstop
    print(f"Message: {data.message()}")
    startstop = data.message()

def StartReader(data):
    global start_x, start_y, start_orientation, start
    print(f"Start Position: {data.x()}")
    start_x = data.x()
    start_y = data.y()
    start_orientation = data.yaw()
    start = [start_x, start_y, start_orientation]

def LocalizationReader(data):
    global curr_x, curr_xV, curr_xA, curr_y, curr_yV, curr_yA, curr_yaw, curr_w, curr_lozalized_pos
    #print(f"Current Localized Position: {data.x()}")
    curr_x = data.x()
    curr_xV = data.xV()
    curr_xA = data.xA()
    curr_y = data.y()
    curr_yV = data.yV()
    curr_yA = data.yA()
    curr_yaw = data.yaw()
    curr_w = data.w()
    curr_lozalized_pos = [curr_x, curr_xV, curr_xA, curr_y, curr_yV, curr_yA, curr_yaw, curr_w]

    x = int(curr_lozalized_pos[0]*10)
    y = int(curr_lozalized_pos[3]*10)
    yaw = curr_lozalized_pos[-2]
    facing = get_facing(yaw)
    msg = "ROBOT,"+str(x)+","+str(y)+","+str(facing)
    Message_data.message(msg)
    messagepub.publish(Message_data)
    

Obstacle.ObstaclePubSubType

core = RobotCore.Core(0)
obstaclesub = core.createSubscriber(Obstacle, "obstacle_data", ObstacleReader, 10)
messagesub = core.createSubscriber(Message, "startstop", MesssageReader, 10)
robot_start_sub = core.createSubscriber(Frame, "start_data", StartReader, 10 )

#create publisher
Message_data = Message.Message()
messagepub = core.createPublisher(Message, "send_message", 10)

localize_pos_sub = core.createSubscriber(Frame,"robot_est_pose",LocalizationReader, 10)

#commandPub = core.createPublisher(Command, "robot_path", 10)

# Control
cmdPub = core.createPublisher(RobotState, "robot_cmd", 1)
progressPub = core.createPublisher(Message, "path_progress", 1)
#self.pathSub = self.core.createSubscriber(Command, "robot_path", self.updatePathQueue, 10)

dt = 0.4
INTER_CMD_DELAY = 1
cur_progress = -1

#example of how to retrive obstacle data

while (startstop!="image"):
    print(startstop)
    time.sleep(1)
    print("waiting")

startTime = time.time()

# obstacles_input = np.array([
#     # [8, 6, "W"],
#     [80, 50, "N"],
#     [160, 120, "W"],
#     [110, 160, "S"],
#     [50, 100, "E"],
#     [160, 50, "W"]
# ])


#### Input 
# start = (15,15,90)


################################# 2.1 RRT Search Path (basic) #################################
print(f"Full Obstacle Input: {obstacles_input}")
robot_controls,obs_order_optimal,OBS_IDX_MAP, PATH_NO_TO_OBS_MAP = rrt_main(obstacles_input, start)
print("******RRT Path Searching Done!!!!")

print("\n Robot Controls: ", robot_controls)

timeLeft = (6 * 60) - (time.time() - startTime)

img_results=None

msg = "STATUS,RUNNING"
Message_data.message(msg)
messagepub.publish(Message_data)

for path_no in robot_controls:
    print("#################### EXECUTING PATH NO ",path_no," ####################\n")
    robot_commands_all = robot_controls[path_no]

    ## Send commands to the robot 
    cmd_count = 0
    print("Visit Order: ", obs_order_optimal)
    
    progress = 0
    for robot_command in robot_commands_all:

        timeLeft = (6 * 60) - (time.time() - startTime)

        '''
        if(timeLeft < 25):
            data = RobotState.RobotState()
            data.velocity(0)
            data.steer_angle(0)
            cmdPub.publish(data)
            print('Times up stopping')
            msg = Message.Message()
            msg.message("STATUS,Times up-stopping")
            messagepub.publish(msg)
            sys.exit(0)
        '''

        print(timeLeft)
        progress += 1

        if((progress / len(robot_commands_all)) > 0.7):
            cur_progress = OBS_IDX_MAP[PATH_NO_TO_OBS_MAP[path_no]] # Obstacle ID
        elif((progress / len(robot_commands_all)) > 0.3):
            cur_progress = -1

        msg = Message.Message()
        msg.message(f"{cur_progress}")
        progressPub.publish(msg)
        print(f"Publishing {cur_progress}")


        cmd = robot_command
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

    print("Obastacle ", path_no+1, " reached, continue to visit the next obstacle")
    
    # PLACEHOLDER: Publish message to STM, indicate one img is done 
    # PLACEHOLDER: Subscrib message from STM, to whether or not continue to the next loop 
    
    # msg_object = Message.Message

#Command_data = Command.Command()
#robot_command=["line",0,0.2]
#Command_data = send_command_helper(robot_command, Command_data)
#commandPub.publish(Command_data)


################################# 2.2 RRT Search Path (skipped obstacles) #################################

# print("\n\n==================================================================")
# print("\nThere are ", len(skipped_goals)," skipped obstacles.")
# print("planning path to visit the skipped obstacles now..........")
# print("Skipped obstacles are: ", skipped_goals)

# if skipped_goals is not None:
#     if len(skipped_goals) > 0 :

#         robot_controls_skipped = rrt_skipped(obstacles_input, start,skipped_goals)
#         print("******RRT Path Searching Done!!!!")
#         print("\n Robot Controls: ", robot_controls)
#         img_results=None

#         for path_no in robot_controls:
#             print("#################### EXECUTING PATH NO ",path_no," ####################\n")
#             robot_commands_all = robot_controls[path_no]
#             ## Send commands to the robot 
#             cmd_count = 0
#             print("Visit Order: ", obs_order_optimal)
#             progress = 0
#             for robot_command in robot_commands_all:
#                 progress += 1

#                 if(progress / len(robot_commands_all) > 0.3):
#                     cur_progress = -1
#                 elif(progress / len(robot_commands_all) > 0.7):
#                     cur_progress = OBS_IDX_MAP[skipped_goals[path_no]] # Obstacle ID
                
#                 msg = Message.Message()
#                 msg.message(f"{cur_progress}")
#                 progressPub.publish(msg)

#                 cmd = robot_command
#                 t = 0
#                 r = 0

#                 if(cmd[0] == 'line'):
#                     d = cmd[1]
#                     v = cmd[2]

#                     if(d > 0):
#                         d = 1.007008*d + 0.016116
#                     else:
#                         d = 0.997737*d - 0.029177
                    

#                 elif(cmd[0] == 'arc'):
#                     r = cmd[2]
#                     v = cmd[3]

#                     if(r > 0):
#                         r = 1.007326*r + 0.022916
#                     else:
#                         r = 1.02624*r - 0.01276

#                     dAngle = cmd[1]
#                     d = r*dAngle

#                 steeringAngle = 0
#                 if(r != 0):
#                     steeringAngle = math.atan(ROBOT_L/r)

#                 vProfile = linearTrapezoid(abs(d), abs(v), MAX_ACC)

#                 discreteProfile = linearInterp(vProfile, np.arange(0, vProfile[-1,0] + 0.5, dt))
#                 if(d < 0):
#                     discreteProfile *= -1
#                 logging.debug(f"Profile:\n{discreteProfile}")
#                 dProfile = np.sum(discreteProfile) * dt
#                 logging.debug(f"Area Check:{dProfile}")
#                 if(cmd[0] == 'arc'):
#                     logging.debug(f"Yaw Change Check:{(180/math.pi)*dProfile / r}")
#                 if(d != dProfile):
#                     discreteProfile *= d/dProfile
#                 if(cmd[0] == 'arc'):
#                     logging.debug(f"Yaw Change Check:{(180/math.pi)*np.sum(discreteProfile) * dt / r}")
#                 logging.debug(f"Area Check (Corrected):{np.sum(discreteProfile) * dt}")
                        

#                 for c in discreteProfile:

#                     data = RobotState.RobotState()
#                     data.velocity(c)
#                     data.steer_angle(steeringAngle)

#                     logging.info(f"Publishing {c} - {steeringAngle}. Waiting {dt} seconds")
#                     cmdPub.publish(data)

#                     time.sleep(dt)
                
#                 time.sleep(INTER_CMD_DELAY)
            

            

#             ### PlaceHolder: receive image results (from img rec) -- in terms of obstacle ID 

#         ### PlaceHolder: Receive localized position (from STM)
#         ### PlaceHolder: Send updated position (to android using BT sender/server)

#         ## Reduced Obs List 
#         # if len(img_results)>1:
#         #     obs_done_list = () # placeholder 
#         #     for obs_done in obs_done_list:
#         #         reduced_obs_list = reduce_obs_input(obstacles_input, obs_done)
#         #         ####Place Holder
        
#         # start = ()
#         # new_controls = rrt_main(reduced_obs_list, start)
# else:
#     print("No obstacle skipped, program ended...!!!!!!")

# ## Code to Pass updated robot position to BT



# ROBOT,1,15,N
# ROBOT,(x),(y),(Direction/N S E W)



    
data = RobotState.RobotState()
data.velocity(0)
data.steer_angle(0)
cmdPub.publish(data)

print('Done')
msg = Message.Message()
msg.message("STATUS,DONE")
messagepub.publish(msg)

time.sleep(1)
