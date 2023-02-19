# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:31:15 2022

@author: Yakun
"""

import sys
import time
import numpy as np 
import math
# from controller.controller import PATH_NO_TO_OBS_MAP

from rrtMultDir import *
from utils import * 

def rrt_main(
    obstacles_input = None,
    start = None
    ):

    print("\n########################################################################################")
    print("############################### RRT Path-finding Started ###############################\n")
    # Decode obstacles inputs into obstacles & img positions 
    obstacles_list, img_pos_list, OBS_IMG_MAP = decode_input(obstacles_input)
    # Get original obstacle & index map 

    
    OBS_IDX_MAP = get_obs_idx_map(obstacles_list)

    # Get optimal visit order 
    obs_order_optimal = find_optimal_order(start, obstacles_list, img_pos_list)
    print("\n\n*****Info*****")
    print("\nOriginal Obs Order (Obstacle to index map): ", OBS_IDX_MAP)
    print("\nOptimal Obs Visit Order ", obs_order_optimal)
    print("\nObstacle to Image Position Map: ", OBS_IMG_MAP)
    print("\n*************\n\n")


    PATH_NO_TO_OBS_MAP = {}

    ####################################################################################
    ################################## RRT SEARCH ######################################

    print("\n============ [START] 3. Performing RRT Search now =============\n")
    robot_controls={}
    for path_no in range(0,len(obstacles_list)):
        print("\n------------- Path No ",path_no," -------------")
        goal_obs = list(obs_order_optimal.keys())[list(obs_order_optimal.values()).index(int(path_no)+1)]
        goal_obs_index = OBS_IDX_MAP[goal_obs]
        goal_img_pos = OBS_IMG_MAP[goal_obs]
        
        print("\nstart: ",start)
        print("goal: ",goal_obs,"\n")
        rrt_output = rrt_iter(
                                obstacles_list = obstacles_list,
                                start = start,
                                goal_obs_idx = goal_obs_index,
                                goal_img_pos = goal_img_pos,
                                )
        # if rrt_output["robot_command"] is not None:
        command_output = rrt_output["robot_command"]
        robot_final_pos = rrt_output["robot_final_pos"]

        robot_controls[path_no] = command_output
        print("\n\nCurrent Path with Commands: ",command_output)
        start = adj_final_pos(robot_final_pos)
        PATH_NO_TO_OBS_MAP[path_no] = goal_obs
    print("\n=================== [END] 3. RRT Search DONE ===================\n")

    ####################################################################################
    ####################################################################################


    print("\n############################### RRT Path-finding Done ################################")
    print("\n######################################################################################\n")

    return robot_controls,obs_order_optimal, OBS_IDX_MAP, PATH_NO_TO_OBS_MAP
    # return robot_controls,obs_order_optimal, OBS_IDX_MAP, PATH_NO_TO_OBS_MAP

        
        
# #%% Execution 
if __name__ == '__main__':

    obstacles_input = np.array([
    # [8, 6, "W"],
    [80, 50, "W"],
    [160, 120, "W"],
    [110, 160, "E"],
    [50, 100, "N"],
    [160, 50, "W"]
    ])

    start = (15,15,math.pi/2)
    rrt_main(obstacles_input, start)
         
        