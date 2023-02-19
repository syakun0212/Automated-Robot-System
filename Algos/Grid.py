import numpy as np
import matplotlib.pyplot as plt
from Obstacle import *

class Grid:
    def __init__(self, obstacles):
        self.nodes = []
        self.neighbours = []

        for i in range(0, 201):
            a = [True for j in range(0,201)]
            self.nodes.append(a)

        #Account and block out the robot footprint(30 * 30)
        for x in range(0, 201):
            for y in range(0, 201):
                if x < 15 or x > 185 or y < 15 or y > 185:
                    self.nodes[x][y] = False

        #Account and block out turning radius(25)
        for x in range(0, 41):
            for y in range(0, 41):
                lhs = (x - 40) ** 2 + (y - 40) ** 2
                if (lhs >= 25 ** 2 and lhs <= 40 ** 2):
                    self.nodes[x][y] = False

        for x in range(0, 41):
            for y in range(160, 201):
                lhs = (x - 40) ** 2 + (y - 160) ** 2
                if (lhs >= 25 ** 2 and lhs <= 40 ** 2):
                    self.nodes[x][y] = False

        for x in range(160, 201):
            for y in range(160, 201):
                lhs = (x - 160) ** 2 + (y - 160) ** 2
                if (lhs >= 25 ** 2 and lhs <= 40 ** 2):
                    self.nodes[x][y] = False

        for x in range(160, 201):
            for y in range(0, 41):
                lhs = (x -160) ** 2 + (y - 40) ** 2
                if (lhs >= 25 ** 2 and lhs <= 40 ** 2):
                    self.nodes[x][y] = False

        #Account and block out obstacles(40 * 40)
        for obstacle in obstacles:
            obstacleX = obstacle.getX()
            obstacleY = obstacle.getY()

            for x in range(obstacleX - 20, obstacleX + 20 + 1):
                for y in range(obstacleY - 20, obstacleY + 20 + 1):
                    if x >= 0 and x <= 200 and y >= 0 and y <= 200:
                        self.nodes[x][y] = False

    def plot(self):
        x = []
        y = []
        xInvalid = []
        yInvalid = []

        for i in range(0, 201):
            for j in range(0, 201):
                if self.nodes[i][j] == True:
                    x.append(i)
                    y.append(j)
                else:
                    xInvalid.append(i)
                    yInvalid.append(j)
        plt.scatter(x, y, marker = ".", color='g', s = 1)
        plt.scatter(xInvalid, yInvalid, marker = ".", color='r', s = 4)
        plt.show()
