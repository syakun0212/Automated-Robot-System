import math
import random

import time
import logging

from dataclasses import dataclass

import numpy as np
import scipy.stats

# from rrtMultDir import *

def rotationMatrix2d(angles): #Homogeneous
    N = len(angles)
    cT = np.cos(angles)
    sT = np.sin(angles)
    rotM = np.transpose(np.stack([
        [cT, -sT, np.zeros(N)],
        [sT, cT, np.zeros(N)],
        [np.zeros(N), np.zeros(N), np.ones(N)]
    ]), [2,0,1])
    return rotM

@dataclass
class ObstacleData: 
    orientation: tuple # x, y, angle (angle of front face normal to axis)
    size: tuple # length , width
    faces: tuple # ID of face going counter clockwise from 'front' face

class Map(object):
    def __init__(self, obstacles):
        self.obstacles = obstacles
        
        self.facesId = []
        self.facesFrame = []
        self.facesAngle = []
        
        for o in obstacles:
            
            x,y,a = o.orientation
            l,w = o.size #length , width
            cA = math.cos(a)
            sA = math.sin(a)
            
            facesAngle = np.fmod(np.array([0, math.pi/2, math.pi, 3*math.pi/2]) + a,2*math.pi)
            facesAngle[facesAngle >= math.pi/2] = facesAngle[facesAngle >= math.pi/2] - 2*math.pi
            
            ids = np.array(list(o.faces))
            base = rotationMatrix2d(facesAngle)
            
            origins = np.zeros((4,3,1), dtype=float)
            origins[:,2] = 1.0
            
            origins = np.matmul(np.array([
                [[1, 0, l/2],
                 [0, 1, 0],
                 [0, 0, 1]],
                [[1, 0, 0],
                 [0, 1, w/2],
                 [0, 0, 1]],
                [[1, 0, -l/2],
                 [0, 1, 0],
                 [0, 0, 1]],
                [[1, 0, 0],
                 [0, 1, -w/2],
                 [0, 0, 1]],
            ]), origins)
            
            origins = np.matmul(np.array([
                [cA, -sA, x],
                [sA, cA, y],
                [0.0, 0.0, 1.0]
            ]), origins)

            base[:,:2,2] = origins[:,:2,0]
            self.facesId.append(ids)
            self.facesFrame.append(base)
            self.facesAngle.append(facesAngle)
        self.facesId = np.concatenate(self.facesId)
        self.facesFrame = np.concatenate(self.facesFrame)
        self.facesAngle = np.concatenate(self.facesAngle)
    
    def getRelativePoints(self, detectId, detectOrientation):
        # detectOrientation = [x,y,angle] #angle is angle of normal of sign saw by camera
        
        detectMask = self.facesId == detectId
        
        relPoints = np.matmul(self.facesFrame[detectMask], np.array([[[detectOrientation[0]], [detectOrientation[1]], [1.0]]]))
        facing = self.facesAngle[detectMask].reshape(-1,1,1) - detectOrientation[2] + math.pi
        facing[facing >= math.pi] = facing[facing >= math.pi] - 2*math.pi
        
        return np.concatenate([relPoints[:,:2], facing], axis=1)