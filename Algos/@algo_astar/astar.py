# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 02:03:27 2022

@author: Yakun
"""

import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
import random
import copy
import math

class ASTAR(object):
    def __init__(self):
        self.turning_rad = 3
        
    def fixHeap(self, pq, posInPQ, fValue, h, k):
        if 2 * h > len(pq):
            try:
                pq[h - 1] = k
                posInPQ[pq[h - 1]] = h
            except:
                print("out of index",h-1)
        elif 2 * h == len(pq):
            if fValue[k] <= fValue[pq[2 * h - 1]]:
                pq[h - 1] = k
                posInPQ[pq[h - 1]] = h
            else:
                pq[h - 1] = pq[2 * h - 1]
                pq[2 * h - 1] = k
                posInPQ[pq[h - 1]] = h
                posInPQ[pq[2 * h - 1]] = 2 * h
        else:
            if (fValue[pq[2 * h]] < fValue[pq[2 * h - 1]]):
                smallerSubHeap = 2 * h + 1
            else:
                smallerSubHeap = 2 * h
            
            if (fValue[k] <= fValue[pq[smallerSubHeap - 1]]):
                pq[h - 1] = k
                posInPQ[pq[h - 1]] = h
            else:
                pq[h - 1] = pq[smallerSubHeap - 1]
                posInPQ[pq[h - 1]] = h
                self.fixHeap(pq, posInPQ, fValue, smallerSubHeap, k)
    
    def decreaseKey(self, pq, posInPQ, fValue , h):
        while (h > 1 and fValue[pq[h - 1]] < fValue[pq[int(h / 2) - 1]]):
            temp = pq[h - 1]
            pq[h - 1] = pq[int(h / 2) - 1]
            pq[int(h / 2) - 1] = temp
            posInPQ[pq[h - 1]] = h
            posInPQ[pq[int(h / 2) - 1]] = int(h / 2)
            h = int(h / 2)
    
    def deleteMin(self, pq, posInPQ, fValue):
        lastNode = pq[len(pq) - 1]
        pq.pop()
        self.fixHeap(pq, posInPQ, fValue, 1, lastNode)
    
    
    def heuristic(self,currentPosition, gValue, goal):
        #euclidean
        h = math.sqrt(abs(currentPosition[0] - currentPosition[0])**2 + abs(currentPosition[1] - currentPosition[1])**2)
        
        #manhattan
        h = abs(currentPosition[0] - currentPosition[0]) + abs(currentPosition[1] - currentPosition[1])
        return gValue + h 
    
    
    def generateNeighbors(self,currentPosition, currentOrientation):
        neighbors = []
        rad = self.turning_rad
        x_disp_half = abs(math.cos(math.pi)/4)*rad
        y_disp_half = abs(math.sin(math.pi)/4)*rad
        disp_slanted = math.sqrt(rad**2+rad**2)

        long_disp_slanted = disp_slanted/2
        short_disp_slanted = rad - (math.cos(math.pi/4)*rad)
        # x_disp_half = 1
        # y_disp_half = 1
        # long_disp_slanted = math.sqrt(rad**2+rad**2)
        # short_disp_slanted = math.sqrt(3**2+(long_disp_slanted/2)**2)
        
        if currentOrientation == 'N':
            neighbors = [
                         (currentPosition[0], currentPosition[1] + 1, 'N', 'F'),
                         (currentPosition[0] + rad, currentPosition[1] + rad, 'E', 'R'),
                         (currentPosition[0] - rad, currentPosition[1] + rad, 'W', 'L'),
                         (currentPosition[0] + x_disp_half, currentPosition[1] + y_disp_half, 'NE', 'FR'),
                         (currentPosition[0] - x_disp_half, currentPosition[1] + y_disp_half, 'NW', 'FL'),
                         (currentPosition[0], currentPosition[1] - 1, 'N', 'B'),
                         (currentPosition[0] + x_disp_half, currentPosition[1] - y_disp_half, 'NW', 'BR'),
                         (currentPosition[0] - x_disp_half, currentPosition[1] - y_disp_half, 'NE', 'BL'),
                         (currentPosition[0] + rad, currentPosition[1] - rad, 'W', 'RR'),
                         (currentPosition[0] - rad, currentPosition[1] - rad, 'E', 'RL')
                        ]
    
        elif currentOrientation == 'S':
            neighbors = [
                         (currentPosition[0], currentPosition[1] - 1, 'S', 'F'),
                         (currentPosition[0] - rad, currentPosition[1] - rad, 'W', 'R'),
                         (currentPosition[0] + rad, currentPosition[1] - rad, 'E', 'L'),
                         (currentPosition[0] - x_disp_half, currentPosition[1] - y_disp_half, 'SE', 'FR'),
                         (currentPosition[0] + x_disp_half, currentPosition[1] - y_disp_half, 'SW', 'FL'),
                         (currentPosition[0], currentPosition[1] + 1, 'S', 'B'),
                         (currentPosition[0] - x_disp_half, currentPosition[1] + y_disp_half, 'SE', 'BR'),
                         (currentPosition[0] + x_disp_half, currentPosition[1] + y_disp_half, 'NE', 'BL'),
                         (currentPosition[0] - rad, currentPosition[1] + rad, 'E', 'RR'),
                         (currentPosition[0] + rad, currentPosition[1] + rad, 'W', 'RL')
                        ]
        elif currentOrientation == 'E':
            neighbors = [
                         (currentPosition[0] + 1, currentPosition[1], 'E', 'F'),
                         (currentPosition[0] + rad, currentPosition[1] + rad, 'N', 'L'),
                         (currentPosition[0] + rad, currentPosition[1] - rad, 'S', 'R'),
                         (currentPosition[0] + x_disp_half, currentPosition[1] + y_disp_half, 'NE', 'FL'),
                         (currentPosition[0] + x_disp_half, currentPosition[1] - y_disp_half, 'SE', 'FR'),
                         (currentPosition[0] - 1, currentPosition[1], 'E', 'B'),
                         (currentPosition[0] - x_disp_half, currentPosition[1] - y_disp_half, 'NE', 'BR'),
                         (currentPosition[0] - x_disp_half, currentPosition[1] + y_disp_half, 'SE', 'BL'),
                         (currentPosition[0] - rad, currentPosition[1] - rad, 'N', 'RR'),
                         (currentPosition[0] - rad, currentPosition[1] + rad, 'S', 'RL')                    
                         ]
        elif currentOrientation == 'W':
            neighbors = [
                         (currentPosition[0] - 1, currentPosition[1], 'W', 'F'),
                         (currentPosition[0] - rad, currentPosition[1] + rad, 'N', 'R'),
                         (currentPosition[0] - rad, currentPosition[1] - rad, 'S', 'L'),
                         (currentPosition[0] - x_disp_half, currentPosition[1] + y_disp_half, 'NW', 'FR'),
                         (currentPosition[0] - x_disp_half, currentPosition[1] - y_disp_half, 'SW', 'FL'),
                         (currentPosition[0] + 1, currentPosition[1], 'W', 'B'),
                         (currentPosition[0] + x_disp_half, currentPosition[1] + 1, 'SW', 'BR'),
                         (currentPosition[0] + x_disp_half, currentPosition[1] - 1, 'NW', 'BL'),
                         (currentPosition[0] + rad, currentPosition[1] + rad, 'S', 'RR'),
                         (currentPosition[0] + rad, currentPosition[1] - rad, 'N', 'RL')    
                         ]
        elif currentOrientation == 'NE':
            neighbors = [
                         (currentPosition[0] + 1, currentPosition[1] + 1, 'NE', 'F'),
                         (currentPosition[0] + long_disp_slanted, currentPosition[1] + short_disp_slanted, 'SE', 'R'), 
                         (currentPosition[0] + short_disp_slanted, currentPosition[1] + long_disp_slanted, 'NW', 'L'),
                         (currentPosition[0] + disp_slanted, currentPosition[1], 'E', 'FR'),
                         (currentPosition[0], currentPosition[1] + disp_slanted, 'N', 'FL'),
                         (currentPosition[0] - 1, currentPosition[1] - 1, 'NE', 'B'),
                         (currentPosition[0] - short_disp_slanted, currentPosition[1] - long_disp_slanted, 'N', 'BR'),
                         (currentPosition[0] - long_disp_slanted, currentPosition[1] - short_disp_slanted, 'E', 'BL'),
                         (currentPosition[0], currentPosition[1] - disp_slanted, 'NW', 'RR'),
                         (currentPosition[0] - disp_slanted, currentPosition[1], 'SE', 'RL')
                         ]
        elif currentOrientation == 'NW':
            neighbors = [
                         (currentPosition[0] - 1, currentPosition[1] + 1, 'NW', 'F'),
                         (currentPosition[0], currentPosition[1] + disp_slanted, 'NE', 'R'),
                         (currentPosition[0] - disp_slanted, currentPosition[1], 'SW', 'L'),
                         (currentPosition[0] - short_disp_slanted, currentPosition[1] + long_disp_slanted, 'N', 'FR'),
                         (currentPosition[0] - long_disp_slanted, currentPosition[1] + short_disp_slanted, 'W', 'FL'),
                         (currentPosition[0] + 1, currentPosition[1] - 1, 'NW', 'B'),
                         (currentPosition[0] + long_disp_slanted, currentPosition[1] - short_disp_slanted, 'W', 'BR'),
                         (currentPosition[0] + short_disp_slanted, currentPosition[1] - long_disp_slanted, 'N', 'BL'),
                         (currentPosition[0] + disp_slanted, currentPosition[1], 'SW', 'RR'),
                         (currentPosition[0], currentPosition[1] - disp_slanted, 'NE', 'RL')
                         ]
        elif currentOrientation == 'SE':
            neighbors = [
                         (currentPosition[0] + 1, currentPosition[1] - 1, 'SE', 'F'),
                         (currentPosition[0] - 2, currentPosition[1] - disp_slanted, 'SW', 'R'),
                         (currentPosition[0] + disp_slanted, currentPosition[1], 'NE', 'L'),
                         (currentPosition[0] + short_disp_slanted, currentPosition[1] - long_disp_slanted, 'S', 'FR'),
                         (currentPosition[0] + long_disp_slanted, currentPosition[1] - short_disp_slanted, 'E', 'FL'),
                         (currentPosition[0] - 1, currentPosition[1] + 1, 'SE', 'B'),
                         (currentPosition[0] - long_disp_slanted, currentPosition[1] + short_disp_slanted, 'E', 'BR'),
                         (currentPosition[0] - short_disp_slanted, currentPosition[1] + long_disp_slanted, 'S', 'BL'),
                         (currentPosition[0] - disp_slanted, currentPosition[1], 'NE', 'RR'),
                         (currentPosition[0], currentPosition[1] + disp_slanted, 'SW', 'RL')
                         ]
        elif currentOrientation == 'SW':
            neighbors = [
                         (currentPosition[0] - 1, currentPosition[1] - 1, 'SW', 'F'),
                         (currentPosition[0] - disp_slanted, currentPosition[1], 'NW', 'R'),
                         (currentPosition[0], currentPosition[1] + disp_slanted, 'SE', 'L'),
                         (currentPosition[0] - long_disp_slanted, currentPosition[1] - short_disp_slanted, 'W', 'FR'),
                         (currentPosition[0] - short_disp_slanted, currentPosition[1] - long_disp_slanted, 'S', 'FL'),
                         (currentPosition[0] + 1, currentPosition[1] + 1, 'SW', 'B'),
                         (currentPosition[0] + short_disp_slanted, currentPosition[1] + long_disp_slanted, 'S', 'BR'),
                         (currentPosition[0] + long_disp_slanted, currentPosition[1] + short_disp_slanted, 'W', 'BL'),
                         (currentPosition[0], currentPosition[1] + disp_slanted, 'SE', 'RR'),
                         (currentPosition[0] + disp_slanted, currentPosition[1], 'NW', 'RL')     
                         ]
        for i in copy.deepcopy(neighbors):
            if i[0] < 0 or i[0] >= 20 or i[1] < 0 or i[1] >= 20:
                neighbors.remove(i)
        return neighbors
    
    
    def aStarSearch(self, start, start_orientation, goal, obs_map):
        v = {(i, j): False for i in range (0, 200) for j in range (0, 200)}        #Visited
        parent = {(i, j): None for i in range (0, 200) for j in range (0, 200)}     #Parent of the nodes to keep track of the path
        gValue = {(i, j): 2 ** 31 for i in range (0, 200) for j in range (0, 200)}     #Initialise to infinity
        fValue = {(i, j): 2 ** 31 for i in range (0, 200) for j in range (0, 200)}     #Initialise to infinity
        posInPQ = {(i, j): 0 for i in range (0, 200) for j in range (0, 200)}
        direction = {(i, j): None for i in range (0, 200) for j in range (0, 200)}
        prevCommand = {(i, j): None for i in range (0, 200) for j in range (0, 200)}
        
        gValue[start] = 0
        fValue[start] = self.heuristic(start, 0, goal)
    
        frontier = []       
        frontier.append(start)  #Initialise the frontier with the starting node
        posInPQ[start] = 1
        direction[start] = start_orientation
    
        pos = 2
        for x in range(0, 20):
            for y in range(0, 20):
                if (x, y) != start:
                    frontier.append((x, y))
                    posInPQ[(x, y)] = pos
                    pos += 1
        
        path = []
        path_with_command=[]
        path_with_orientation=[]
        while len(frontier) > 0:        #While the frontier is not empty
            u = frontier[0]
            if u == goal:
                break       #We break if we have found our goal
            self.deleteMin(frontier, posInPQ, fValue)
            v[u] = 1
            
            #generate neighbors wrt to current direction and position
            neighbors = self.generateNeighbors(u, direction[u])            
            for neighbor in neighbors:
                neighborCoord = (int(neighbor[0]), int(neighbor[1]))
                if (int(neighbor[0]), int(neighbor[1])) not in obs_map:
                    if neighbor[3] == 'F':
                        g = gValue[u] + 1
                    elif neighbor[3] in ["R","L","RR","RL"]:
                        # Check if cut through obstacles
                        x_diff = neighbor[0] - u[0]
                        y_diff = neighbor[1] - u[1]
                        for i in range(int(x_diff)):
                            for j in range(int(y_diff)):
                                x_inbetween = u[0]+i
                                y_inbetween = u[1]+j
                                coord_inbetween = (x_inbetween,y_inbetween)
                                if coord_inbetween in obs_map:
                                    continue
                        g = gValue[u] + (3*2* math.pi)/4     
                    elif neighbor[3] in ["FR", "FL","BR","BL"]:
                        x_diff = neighbor[0] - u[0]
                        y_diff = neighbor[1] - u[1]
                        for i in range(int(x_diff)):
                            for j in range(int(y_diff)):
                                x_inbetween = u[0]+i
                                y_inbetween = u[1]+j
                                coord_inbetween = (x_inbetween,y_inbetween)
                                if coord_inbetween in obs_map:
                                    continue
                        g = gValue[u] + math.pi/2
      
                    newF = self.heuristic(neighborCoord, g, goal)
                    if v[neighborCoord] != 1 and fValue[neighborCoord] > newF:
                        fValue[neighborCoord] = newF
                        gValue[neighborCoord] = g
                        parent[neighborCoord] = u
                        direction[neighborCoord] = neighbor[2]
                        prevCommand[neighborCoord] = neighbor[3]
                        self.decreaseKey(frontier, posInPQ, fValue, posInPQ[neighborCoord])
        ## Append path 
        tmp_goal = goal
        while tmp_goal is not None:
            path.append(tmp_goal)
            tmp_goal = parent[tmp_goal]
    
        # Append path with directions 
        tmp_command = goal
        command = prevCommand[tmp_command]
        tmp_command_copy = (*tmp_command,command)
        while tmp_command is not None:
            path_with_command.append(tmp_command_copy)
            tmp_command = parent[tmp_command]
            try:
                command = prevCommand[tmp_command]
                tmp_command_copy = (*tmp_command, command)
            except:
                continue
        path_with_command.reverse()
        
        # Append path wih orientation
        tmp_orientation = goal
        orientation = direction[tmp_orientation]
        tmp_orientation_copy = (*tmp_orientation,orientation)
        while tmp_orientation is not None:
            path_with_orientation.append(tmp_orientation_copy)
            tmp_orientation = parent[tmp_orientation]
            try:
                orientation = direction[tmp_orientation]
                tmp_orientation_copy = (*tmp_orientation, orientation)
            except:
                continue
        path_with_orientation.reverse()
        # print("\nPath with commands")
        # for i in path_with_command:
        #     print(i)
        # print("Path with directions:",path_with_direction)
        return {
            "path":path,
            "path_with_cmd": path_with_command,
            "path_with_orientation":path_with_orientation
            }
    