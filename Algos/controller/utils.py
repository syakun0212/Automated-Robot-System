# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 17:32:24 2022

@author: Yakun
"""
from re import S
import numpy as np
import math 
import math
from tabnanny import check 
import numpy as np
import operator 
from idl_data.Command_data.Command_build import Command


def decode_input(obs_input):
    obstacles = []
    img_pos_list = []
    obs_img_map = {}
    for i in obs_input:
        x = int(i[0])
        y = int(i[1])
        img = i[2]
        obs = (x,y)
        obstacles.append(obs)
        img_pos_list.append(img)
        obs_img_map[obs] = img
    obstacles = np.array(obstacles)
    # print("obstacles: ", obstacles,"\n")
    # print("img_pos: ", img_pos, "\n")
    # print("---------------------------")
    return obstacles, img_pos_list, obs_img_map


def get_obs_idx_map(obstacles):
    obs_idx_map = {}
    for i in range(0,len(obstacles)):
        obs = obstacles[i]
        obs_idx_map[tuple(list(obs))] = i 
    return obs_idx_map 


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
    # print("goal_list",goal_coord_list)
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
    return {
        "goal_list": new_goal_list,
        "goal_dictionary": goal_coord_list
    }



def cal_total_dist(path:list=[]):
    dist = 0
    for i in range(len(path)-1):
        node_1 = path[i]
        node_2 = path[i+1]
        x1,y1 = node_1[0], node_1[1]
        x2,y2 = node_2[0], node_2[1]
        # print("\ndistance", i, ": ",math.sqrt((x1-x2)**2+ (y1-y2)**2))
        dist += math.sqrt((x1-x2)**2+ (y1-y2)**2)
        # print("total_dist: ", dist)
    return dist


def trial_order(visit_order):

    shortest_dist = cal_total_dist(list(visit_order.keys()))
    visit_order_shortest = visit_order.copy()

    for i in range(1,len(visit_order.keys())-1):
        visit_order_temp = visit_order.copy()

        ith_node = list(visit_order.keys())[i]
        neighbor_node = list(visit_order.keys())[i+1]

        visit_order_temp[ith_node]=i+1
        visit_order_temp[neighbor_node]=i
        visit_order_temp_sorted = sorted(visit_order_temp.items(),key = operator.itemgetter(1),reverse = False)
        visit_order_temp_sorted_list = [i[0] for i in visit_order_temp_sorted]

        print("\ntrial order: ",visit_order_temp_sorted_list)
        dist = cal_total_dist(visit_order_temp_sorted_list)
        # print("New Order: ", visit_order_temp_sorted_list)
        print("New Distance: ",dist)
        if dist<shortest_dist:
            print("Revised order: ",visit_order_temp)
            shortest_dist = dist
            visit_order_shortest = visit_order_temp

    return visit_order_shortest


def find_optimal_order(start, obstacles_list, img_pos_list):
    print("================ [START] 1. Determining optimal visit order now.... ================")
    middle_goal_output = decode_goals(obstacles_list, img_pos_list)
    goal_sets = middle_goal_output["goal_list"]
    obs_goal_dict = middle_goal_output["goal_dictionary"]
    
    start = start 
    unvisited_goals = obs_goal_dict.copy()
    visit_order = {}
    visit_order[tuple(start)] = 0
    start_x, start_y = start[0], start[1]

    for layer in range(1,len(obs_goal_dict.keys())+1):
        # start_x, start_y = start[0], start[1]
        nearest_dist = 10000
        nearest_neighbor = None

        print("\n---------- Obstacle Visit Order Tree - Layer",layer," ----------")
        for goal in unvisited_goals.keys():
            goal_x, goal_y = goal[0], goal[1]
            d = math.sqrt((goal_x-start_x)**2+(goal_y-start_y)**2)
            if d<nearest_dist:
                nearest_dist = d
                nearest_neighbor = goal
                start_x, start_y = goal_x, goal_y
            # print("Node ",goal," distance: ", d)

        unvisited_goals.pop(nearest_neighbor)
        visit_order[nearest_neighbor] = layer
        print("\nnearest neighbor appended: ", nearest_neighbor)
        # print("remaning neighbors: ",unvisited_goals)

    ## Do Minor Adjustments (switching neighbour nodes) 
    print("\n............Revising order now..............")
    visit_order_optimal = trial_order(visit_order)
    print("\n............Visit Order Revise Finished............\n")
    print("\n***Final Order: ", visit_order,"\n")
    print("=================== [DONE]1. Optimal Visit Order Found ===================")
    return visit_order_optimal

        
def sort_goal_sets(goal_sets: list=[],obs_order: dict={}, obs_goal_dict: dict={}):
    print("\n================ [START] 2. Sorting New Goal Sets Now ================\n")
    revised_goal_sets = []

    obs_order_copy = obs_order.copy()
    obs_order_sorted = sorted(obs_order_copy.items(),key = operator.itemgetter(1),reverse = False)

    print("\nObs order sorted: ", obs_order_sorted)
    print("\nOriginal goal sets order: ", goal_sets)
    for obs_coord in obs_order_sorted[1:]:
        goal_coord=obs_goal_dict[obs_coord[0]]
        for goal_set in goal_sets:
            if (int(goal_coord[0])==int(goal_set[0])) and (int(goal_coord[1])==int(goal_set[1])):
                # print("checking pair: ",goal_coord, goal_set)
                revised_goal_sets.append(goal_set)
    print("\nrevised goal_sets ordered: ", revised_goal_sets)

    print("\n================ [END] 2. Sorting New Goal Sets Finished ================")
    return revised_goal_sets


obstacles_input = np.array([
    # [8, 6, "W"],
    [80, 50, "N"],
    [160, 120, "W"],
    [110, 160, "S"],
    [50, 100, "E"],
    [160, 50, "W"]
])


def reduce_obs_input(original_obstacle_list, obs_done):
    obs_done_x, obs_done_y = obs_done[0], obs_done[1]
    obs_list_orig = list(original_obstacle_list,)

    obs_list_reduced = []
    for obs in obs_list_orig:
        obs_x, obs_y = obs[0], obs[1]
        if (obs_done_x==obs_x) and (obs_done_y==obs_y):
            continue
        else:
            obs_list_reduced.append(obs)
    return obs_list_reduced

        
def send_command_helper(robot_command, command_object: Command.Command):
    if robot_command[0].lower() == "line":
        command_object.type(str(robot_command[0]))
        command_object.distance(float(robot_command[1]))
        command_object.velocity(float(robot_command[2]))
    elif robot_command[0].lower() == "arc":
        command_object.type(str(robot_command[0]))
        command_object.desired_angle(float(robot_command[1]))
        command_object.turning_radius(float(robot_command[2]))
        command_object.velocity(float(robot_command[3]))

    return command_object

def adj_final_pos(final_pos):
    x, y, orientation = final_pos
    x = int(x/3)
    y = int(y/3)
    orientation = orientation

    return(x, y, orientation)


def get_facing(yaw):
    base_angle = float(yaw)
    while(base_angle < 0):
        base_angle += 2*math.pi

    base_angle = base_angle%(2*math.pi)
    if base_angle > (math.pi/4) and base_angle<(math.pi*3/4):
        return "N"
    #elif base_angle < -(math.pi/4) and base_angle > -(math.pi*3/4):
    elif base_angle > 5*math.pi/4 and base_angle < 7*math.pi*3/4:
        return "S"
    elif base_angle > 3*math.pi/4 and base_angle < 5*math.pi/4:
        return "W"
    else:
        return "E"
# obstacles_input = np.array([
#     # [8, 6, "W"],
#     [80, 50, "N"],
#     [160, 120, "W"],
#     [110, 160, "S"],
#     [50, 100, "E"],
#     [160, 50, "W"]
# ])