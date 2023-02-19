# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 11:11:18 2022

@author: Yakun
"""
import numpy as np
import math

def decode_input(obs_input):
    obstacles = []
    img_pos = []
    for i in obs_input:
        x = int(i[0])
        y = int(i[1])
        img = i[2]
        obstacles.append((x,y))
        img_pos.append(img)
    obstacles = np.array(obstacles)
    print("obstacles: ", obstacles,"\n")
    print("img_pos: ", img_pos, "\n")
    print("---------------------------")
    return obstacles, img_pos

def generate_robot_controls(path_with_commands, velocity: float=0.2,turning_rad=4):
    """
    Returns robot control commands:
        (‘line’, displacement, velocity)
        (‘arc’, yaw, turning radius, velocity)

    """
    mode=None
    displacement=None
    yaw=None

    turning_rad=turning_rad*10/100
    
    robot_controls=[]

    for i in range(0,len(path_with_commands)-1):
        curr_pos = path_with_commands[i]
        next_pos = path_with_commands[i+1]      
        
        # goal_x = next_pos[0]
        # goal_y = next_pos[1]
        movement = next_pos[2]
        
        if movement.upper() in ["F","B"]:
            mode="line"
            displacement=math.sqrt((curr_pos[0]-next_pos[0])**2+(curr_pos[1]-next_pos[1])**2)
            displacement=displacement*10/100
            control = (mode,displacement,velocity)
        else:
            mode="arc"
            if movement in ["R","L","RR","RL"]:
                yaw=math.pi/2
            else:
                yaw=math.pi/4
            if movement[-1] == "R":
                yaw=-yaw
                turning_rad=-turning_rad
            control = (mode,yaw,turning_rad,velocity)
        
        robot_controls.append(control)
    return robot_controls
           