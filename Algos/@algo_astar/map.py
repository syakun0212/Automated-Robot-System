# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:04:26 2022

@author: Yakun
"""
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np


class MAP():
    
    def __init__(self):
    
        self.EMPTY_CELL = 0
        self.OBSTACLE_CELL = 1
        self.START_CELL = 2
        self.GOAL_CELL = 3
        self.MOVE_CELL = 4
 
    
        self.cmap = colors.ListedColormap(['white', 'black', 'green', 'red', 'blue'])
        self.bounds = [self.EMPTY_CELL, self.OBSTACLE_CELL, self.START_CELL, self.GOAL_CELL, self.MOVE_CELL]
        self.norm = colors.BoundaryNorm(self.bounds, self.cmap.N)
        self.rows = 20
        self.cols = 20
        
        self.start_orig = (1,1)
    
    # def plot_grid(self, obstacles=None, start: tuple=(1,1),path=None, title: str=""):
    def plot_grid(self, obstacles=None, start = (1,1),path=None, title=""):
        
        map_data = np.zeros(self.rows * self.cols).reshape(self.rows, self.cols)
        # map_data[start[0], start[1]] = self.START_CELL
        for obstacle in obstacles:
            obstacleY = obstacle[0]
            obstacleX = obstacle[1]
            for y in range(obstacleY - 1, obstacleY + 2):
                if y < 0 or y >= self.cols:
                    continue
                for x in range(obstacleX - 1, obstacleX + 2):
                    if x < 0 or x >= self.cols:
                        continue
                    map_data[x,y] = self.OBSTACLE_CELL
                    
        data = self.add_path(map_data, path) if path is not None else map_data
        
        fig, ax = plt.subplots(figsize=(10,10))
        ax.imshow(data, cmap=self.cmap, origin="lower",extent=(0,20,0,20))
        ax.grid(which='major', axis='both', linestyle='-', color='grey', linewidth=1)
        ax.set_xticks(np.arange(0, self.rows, 1))
        ax.set_yticks(np.arange(0, self.cols, 1))
        ax.tick_params(axis='both', labelsize=0, length = 0)
        ax.set_xlim(0,20)
        ax.set_ylim(0,20)
        ax.set_title(title)
        plt.show()
    
    def add_path(self, data, path):
        curr_start = path[-1]
        curr_goal = path[0]
        for i in path:
            if i == curr_start:
                data[i[1], i[0]] = self.START_CELL
            elif i == curr_goal:
                data[i[1], i[0]] = self.GOAL_CELL
            else:                
               data[i[1], i[0]] = self.MOVE_CELL
        return data 
    
        
                
    def get_obs_map(self,obstacles_list=None):
        obs_map = []
        for obs in obstacles_list:
            x = obs[0]
            y = obs[1]
            for i in [-1,0,1]:
                for j in [-1,0,1]:
                    coord = (x+i,y+j)
                    # print(coord)
                    obs_map.append(coord)
        return obs_map
    
    def get_goals(self,obstacles, img_pos):
        print()
        goals = {}
        for i in range(0, len(obstacles)):
            pos = img_pos[i]
            obs = tuple(list(obstacles[i]))
            obs_x, obs_y = obs
            if pos.upper()=="N":
                goal_coord=(obs_x,obs_y+2)
            elif pos.upper()=="S":
                goal_coord=(obs_x,obs_y-2)
            elif pos.upper()=="E":
                goal_coord=(obs_x+2,obs_y)
            elif pos.upper()=="W":
                goal_coord=(obs_x-2,obs_y)
            print("goal_coord",goal_coord)
            print("obstacle",obs,"\n")
            goals[obs]=goal_coord
        print("---------------------------")
        return goals


        
        
            
            
            
            
            