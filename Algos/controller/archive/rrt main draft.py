# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:31:15 2022

@author: Yakun
"""

import sys
import time
import numpy as np 


sys.path.append(r'D:\\1. Academic\\5. Y3S1\\CZ3004\\@rrt')

from rrtMultDir import *
from utils import * 

def rrt_main(
    obstacles_input:np.array=None,
    start: tuple=()
    ):

    print("\n########################################################################################")
    print("############################### RRT Path-finding Started ###############################\n")
    # Decode obstacles inputs into obstacles & img positions 
    obstacles_list, img_pos_list, OBS_IMG_MAP = decode_input(obstacles_input)

    # Get original obstacle & index map 
    OBS_IDX_MAP = get_obs_idx_map(obstacles_list)

    goal_output = decode_goals(obstacles, img_pos)
    goal_sets=goal_output["goal_list"]
    obs_goal_dict=goal_output["goal_dictionary"]

    obs_order_optimal = find_optimal_order(start, obstacles_list, img_pos_list)
    # goal_sets_ordered = sort_goal_sets(goal_sets, obs_order_optimal, obs_goal_dict)
    print("\n\n*****Info*****")
    print("\nOrigiinal goal_sets Order", goal_sets)
    print("\nGoal to Obs Dict ", obs_goal_dict)
    print("\Optimal Obs Visit Order ", obs_order_optimal)
    print("\nOptimal goal_sets Order: ", goal_sets_ordered)
    print("\n*************\n\n")

    ####################################################################################
    ################################## RRT SEARCH ######################################

    print("\n============ [START] 3. Performing RRT Search now =============\n")
    robot_controls={}
    for path_no in range(0,len(goal_sets_ordered)):
        print("\n------------- Path No ",path_no," -------------")
        goal = goal_sets_ordered[path_no]
        print("\nstart: ",start)
        print("goal: ",goal,"\n")
        rrt_output = rrt_iter(
                                    obstacles,
                                    start,
                                    goal
                                 )
        command_output = rrt_output["robot_command"]
        robot_final_pos = rrt_output["robot_final_pos"]
        robot_controls[path_no] = command_output
        print("\n\nCurrent Path with Commands: ",command_output)
        start = robot_final_pos
    print("\n=================== [END] 3. RRT Search DONE ===================\n")

    ####################################################################################
    ####################################################################################


    print("\n############################### RRT Path-finding Done ################################")
    print("\n######################################################################################\n")

    return robot_controls
        
        
# #%% Execution 
# if __name__ == '__main__':

#     obstacles_input = np.array([
#     # [8, 6, "W"],
#     [80, 50, "N"],
#     [160, 120, "W"],
#     [110, 160, "S"],
#     [50, 100, "E"],
#     [160, 50, "W"]
#     ])

#     start = (15,15,90)
#     rrt_main(obstacles_input, start)
         
        