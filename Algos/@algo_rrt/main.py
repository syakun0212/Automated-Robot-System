# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:31:15 2022

@author: Yakun
"""

import sys
import time
import numpy as np 


sys.path.append(r'D:\\1. Academic\\5. Y3S1\\CZ3004\\@rrt')

from rrtBiDirTest2 import *
from utils import * 

def main():
    
    # obstacles_input = np.array([
    #     # [8, 6, "W"],
    #     [80, 50, "N"],
    #     [160, 120, "W"],
    #     [110, 160, "S"],
    #     [50, 100, "E"],
    #     [160, 50, "W"]
    # ])

    obstacles_input = np.array([
        [40, 50, "N"]
        # [160, 50, "W"],
        # [90, 110, "S"],
        # [20, 160, "E"],
        # [150, 140, "W"],

    ])
    
    # initialize 
    start = (15,15,90)

    

    obstacles, img_pos = decode_input(obstacles_input)
    goal_sets = decode_goals(obstacles, img_pos)
    
    # Start Astar Search 
    robot_controls={}
    
    for iteration in range(0,len(goal_sets)):
        print("\n================== Path ",iteration," ==================")
        
        goal = goal_sets[iteration]
        print("\nstart: ",start)
        print("goal: ",goal)
        
        print("\n... conducting astar search now ...")
        command_output = rrt_iter(
                                    obstacles,
                                    start,
                                    goal
                                 )
    
        robot_controls[iteration] = command_output
        print("\nCurrent Path with Commands: ",command_output)
        start = goal
        
        
#%% Execution 
if __name__ == '__main__':
   main()
        
        