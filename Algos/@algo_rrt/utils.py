# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 17:32:24 2022

@author: Yakun
"""
import numpy as np
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
    # print("obstacles: ", obstacles,"\n")
    # print("img_pos: ", img_pos, "\n")
    # print("---------------------------")
    return obstacles, img_pos

def get_goals(obstacles, img_pos):
    print()
    goals = {}
    for i in range(0, len(obstacles)):
        pos = img_pos[i]
        obs = tuple(list(obstacles[i]))
        obs_x, obs_y = obs
        if pos.upper()=="N":
            goal_coord=(obs_x,obs_y+40)
        elif pos.upper()=="S":
            goal_coord=(obs_x,obs_y-40)
        elif pos.upper()=="E":
            goal_coord=(obs_x+40,obs_y)
        elif pos.upper()=="W":
            goal_coord=(obs_x-40,obs_y)
        # print("goal_coord",goal_coord)
        # print("obstacle",obs,"\n")
        goals[obs]=goal_coord
    # print("---------------------------")
    return goals


def decode_goals(obstacles, img_pos):
    goal_coord_list = get_goals(obstacles,img_pos)
    print("goal_list",goal_coord_list)
    new_goal_list = []
    
    for i in range(0,len(obstacles)):
        obs = obstacles[i]
        img = img_pos[i]
        goal_coord = goal_coord_list[tuple(list(obs))]
        goal_x, goal_y = goal_coord
        if str(img) == "N":
            facing = -90
        elif str(img) == "S":
            facing = 90
        elif str(img) == "W":
            facing = 0
        elif str(img) == "E":
            facing = 180
        
        goal_sets = (goal_x,goal_y,facing)
        new_goal_list.append(goal_sets)
        # print("goal_list:",new_goal_list)
    return new_goal_list
        