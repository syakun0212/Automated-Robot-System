# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:31:15 2022

@author: Yakun
"""

import sys
import time
import numpy as np 


sys.path.append(r'D:\\1. Academic\\5. Y3S1\\CZ3004\\@astar')

from map import MAP
from astar import ASTAR
from utils import decode_input,generate_robot_controls

def main():
    
    obstacles_input = np.array([
        # [8, 6, "W"],
        [16, 12, "W"],
        [8, 5, "N"],
        [11, 16, "S"],
        # [5, 10, "E"]
    ])
    
    # initialize 
    start = (1,1)
    start_orientation = "N"

    turning_rad = 4
    
    myMap = MAP()
    obstacles, img_pos = decode_input(obstacles_input)
    obs_map = myMap.get_obs_map(obstacles_list = obstacles)
    goals = myMap.get_goals(obstacles, img_pos)
    
    # Start Astar Search 
    path={}
    path_with_command={}
    path_with_orientation={}
    robot_controls={}
    for iteration in range(0,len(goals)):
        print("\n================== Path ",iteration," ==================")
        astar = ASTAR()
        astar.turning_rad = int(turning_rad)
        next_obs = obstacles[iteration]
        goal = goals[tuple(list(next_obs))]
        print("\nstart: ",start)
        print("goal: ",goal)
        print("starting orientation: ", start_orientation)
        
        print("\n... conducting astar search now ...")
        search_output = astar.aStarSearch(
                                           start = start,
                                           start_orientation = start_orientation,
                                           goal = goal,
                                           obs_map = obs_map 
                                          )
        
        curr_path = search_output["path"]
        curr_path_with_cmd = search_output["path_with_cmd"]
        curr_path_with_orientation = search_output["path_with_orientation"]
        curr_robot_control = generate_robot_controls(curr_path_with_cmd)
        
        path[iteration] = curr_path
        path_with_command[iteration] = curr_path_with_cmd
        path_with_orientation[iteration] = curr_path_with_cmd
        robot_controls[iteration] = curr_robot_control
        
        myMap.plot_grid(obstacles=obstacles,start=start,path=curr_path, title="Iteration #" + str(iteration))
        
        print("\nCurrent Path: ", curr_path)
        print("\nCurrent Path with Commands: ",curr_path_with_cmd)
        print("\nCurrent Path with Orientation: ", curr_path_with_orientation)
        print("\nCurrent Robot Controls: ", curr_robot_control)
        
        start = goal
        start_orientation = curr_path_with_orientation[-1][-1]
        
#%% Execution 
if __name__ == '__main__':
   main()
        
        
# %%
