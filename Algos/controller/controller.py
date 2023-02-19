# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:31:15 2022

@author: Yakun
"""

import sys
import time
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
import time


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
        obstacles_input.append([x*10+5,y*10+5,d])
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

    x = int((int(curr_lozalized_pos[0]))*10)
    y = int((int(curr_lozalized_pos[3]))*10)
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

commandPub = core.createPublisher(Command, "robot_path", 10)

#example of how to retrive obstacle data

while (startstop!="image"):
    print(startstop)
    time.sleep(1)
    print("waiting")

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
robot_controls,obs_order_optimal, skipped_goals, OBS_IDX_MAP, PATH_NO_TO_OBS_MAP= rrt_main(obstacles_input, start)
print("******RRT Path Searching Done!!!!")

print("\n Robot Controls: ", robot_controls)

img_results=None

for path_no in robot_controls:
    print("#################### EXECUTING PATH NO ",path_no," ####################\n")
    robot_commands_all = robot_controls[path_no]
    ## Send commands to the robot 
    cmd_count = 0
    print("Visit Order: ", obs_order_optimal)
    for robot_command in robot_commands_all:
        Command_data = Command.Command()
        Command_data = send_command_helper(robot_command, Command_data)
        commandPub.publish(Command_data)
        cmd_count +=1 
        print("\nCurrent Command: ", robot_command)

        if cmd_count%3 == 3:
            # Place Holder Read img results 
            # Pass the position to Android:  

            #curr_lozalized_pos
            #x = int((int(curr_lozalized_pos[0]))%10)
            #y = int((int(curr_lozalized_pos[3]))%10)
            #yaw = curr_lozalized_pos[-2]
            #facing = get_facing(yaw)
            #msg = "ROBOT,"+str(x)+","+str(y)+","+str(facing)
            #Message_data.message(msg)
            #messagepub.publish(Message_data)
            pass
        time.sleep(0.2)

    print("Obastacle ", path_no+1, " reached, continue to visit the next obstacle")
    curr_obs_visited = PATH_NO_TO_OBS_MAP[path_no]
    curr_obs_visited_idx = OBS_IDX_MAP[curr_obs_visited]
    print("Current obstacle visited: ", curr_obs_visited)
    print("Index of current obstacle visited:  ", curr_obs_visited_idx)
    # PLACEHOLDER: Publish message to STM, indicate one img is done 
    # PLACEHOLDER: Subscrib message from STM, to whether or not continue to the next loop 
    
    # msg_object = Message.Message

Command_data = Command.Command()
robot_command=["line",0,0.2]
Command_data = send_command_helper(robot_command, Command_data)
commandPub.publish(Command_data)


################################# 2.2 RRT Search Path (skipped obstacles) #################################

print("\n\n==================================================================")
print("\nThere are ", len(skipped_goals)," skipped obstacles.")
print("planning path to visit the skipped obstacles now..........")
print("Skipped obstacles are: ", skipped_goals)

if skipped_goals is not None:
    if len(skipped_goals) > 0 :

        robot_controls_skipped = rrt_skipped(obstacles_input, start,skipped_goals)
        print("******RRT Path Searching Done!!!!")
        print("\n Robot Controls: ", robot_controls)
        img_results=None

        for path_no in robot_controls:
            print("#################### EXECUTING PATH NO ",path_no," ####################\n")
            robot_commands_all = robot_controls[path_no]
            ## Send commands to the robot 
            cmd_count = 0
            print("Visit Order: ", obs_order_optimal)
            for robot_command in robot_commands_all:
                Command_data = Command.Command()
                Command_data = send_command_helper(robot_command, Command_data)
                commandPub.publish(Command_data)
                cmd_count +=1 
                print("\nCurrent Command: ", robot_command)

                if cmd_count%3 == 3:
                    # Place Holder Read img results 
                    # Pass the position to Android:  

                    #curr_lozalized_pos
                    #x = int((int(curr_lozalized_pos[0]))%10)
                    #y = int((int(curr_lozalized_pos[3]))%10)
                    #yaw = curr_lozalized_pos[-2]
                    #facing = get_facing(yaw)
                    #msg = "ROBOT,"+str(x)+","+str(y)+","+str(facing)
                    #Message_data.message(msg)
                    #messagepub.publish(Message_data)
                    pass
                time.sleep(0.2)

            print("Obastacle ", path_no+1, " reached, continue to visit the next obstacle")
            curr_skipped_obs_visited = skipped_goals[path_no]
            curr_skipped_obs_visited_idx = OBS_IDX_MAP[curr_skipped_obs_visited]
            print("Current Obstacle visited: ", curr_skipped_obs_visited)
            print("Index of Current obstacle visited: ", curr_skipped_obs_visited_idx)
        Command_data = Command.Command()
        robot_command=["line",0,0.2]
        Command_data = send_command_helper(robot_command, Command_data)
        commandPub.publish(Command_data)
            ### PlaceHolder: receive image results (from img rec) -- in terms of obstacle ID 

        ### PlaceHolder: Receive localized position (from STM)
        ### PlaceHolder: Send updated position (to android using BT sender/server)

        ## Reduced Obs List 
        # if len(img_results)>1:
        #     obs_done_list = () # placeholder 
        #     for obs_done in obs_done_list:
        #         reduced_obs_list = reduce_obs_input(obstacles_input, obs_done)
        #         ####Place Holder
        
        # start = ()
        # new_controls = rrt_main(reduced_obs_list, start)
else:
    print("No obstacle skipped, program ended...!!!!!!")

## Code to Pass updated robot position to BT



# ROBOT,1,15,N
# ROBOT,(x),(y),(Direction/N S E W)



    


