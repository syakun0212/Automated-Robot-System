# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:31:15 2022

@author: Yakun
"""

import sys
import time
import numpy as np 
import math

from rrtMultDir import *
from utils import * 

def rrt_skipped(
    obstacles_input = None,
    start = None,
    skipped_obs_list = None
    ):

    print("\n########################################################################################")
    print("############################### RRT Path-finding Started ###############################\n")
    # Decode obstacles inputs into obstacles & img positions 
    obstacles_list, img_pos_list, OBS_IMG_MAP = decode_input(obstacles_input)
    # Get original obstacle & index map 

    
    OBS_IDX_MAP = get_obs_idx_map(obstacles_list)

    # # Get optimal visit order 
    # obs_order_optimal = find_optimal_order(start, obstacles_list, img_pos_list)
    # print("\n\n*****Info*****")
    # print("\nOriginal Obs Order (Obstacle to index map): ", OBS_IDX_MAP)
    # print("\nOptimal Obs Visit Order ", obs_order_optimal)
    # print("\nObstacle to Image Position Map: ", OBS_IMG_MAP)
    # print("\n*************\n\n")

    ####################################################################################
    ################################## RRT SEARCH ######################################

    print("\n============ [START] 3. Performing RRT Search now =============\n")
    robot_controls={}
    # skipped_goals = []
    nth_valid_obs = 1
    if skipped_obs_list is not None:
        for goal_obs in skipped_obs_list:
            print("\n------------- Path No ",nth_valid_obs," -------------")
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
            command_output = rrt_output["robot_command"]
            robot_final_pos = rrt_output["robot_final_pos"]
            robot_controls[nth_valid_obs] = command_output
            print("\n\nCurrent Path with Commands: ",command_output)
            start = adj_final_pos(robot_final_pos)
            nth_valid_obs += 1
    else:
        robot_controls=None


    print("\n=================== [END] 3. RRT Search DONE ===================\n")

    ####################################################################################
    ####################################################################################


    print("\n############################### RRT Path-finding Done ################################")
    print("\n######################################################################################\n")

    return robot_controls
        

        