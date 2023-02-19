import math
import random
import time

import numpy as np
import cv2 as cv

class RRT(object):

    def __init__(self):
        pass

    def reset(self):
        self.nodes = [] # [(x, y, ax, ay)]
        self.idTable = dict()
        self.nodeChild = dict()
        self.nodeParent = dict()
        self.nodeCost = []
        self.path = []
        self.reach = []
        self.samples = []
        self.maskCalcs = dict()
        
        self.map = None
        self.mShape = None
        
        self.start = None
        self.end = None
        
        self.found = False

    def setMap(self, mapImgG: np.ndarray):
        self.map = mapImgG
        self.mShape = np.array(self.map.shape)

    def setGoals(self, startPoint: tuple, endPoint: tuple):
        self.start = startPoint # (x, y, ax, ay)
        self.end = endPoint # (x, y, ax, ay)
        
        self.nodes.append(self.start)
        self.nodeCost.append(0.0)

        self.nodeChild[self.start] = []
        self.nodeParent[self.start] = None
        self.idTable[self.start] = 0

    def setActions(self):
        pass

    def roundClipToEdge(self, vertices: np.ndarray):
        #vertices is (Nx4) array
        v = vertices
        v[:,:2] = np.clip(np.round(v[:,:2]).astype(np.int32), [0,0], self.mShape[::-1]-1)
        return v

    def addNode(self, dQ, greedyBias=0.05, strict=False, neighbourRad = 0):
        # Sample
        while True:

            if(random.random() < (1-greedyBias)):
                rConf = self.roundClipToEdge(np.multiply(self.mShape[::-1], np.random.uniform(size=(2))))
            else:
                rConf = np.array(self.end)
            
            if(self.map[rConf[1], rConf[0]] == 0):
                break

        # Find nearest possible node
        nodeVec = np.array(self.nodes, dtype=np.float32)
        nodeDiff = nodeVec[:,:2]-rConf

        nodeDiffDot = np.dot(nodeVec[:,:2], nodeDiff)


        nodeDists = np.sum(np.multiply(nodeDiff, nodeDiff), axis=1)
        nodeDistsIndexSort = np.argsort(nodeDists)
        nodeDistsSort = nodeDists[nodeDistsIndexSort]
        
        extendInd = nodeDistsIndexSort[0]
        
        nearestNode = self.nodes[extendInd]
        nearestNodeNp = nodeVec[extendInd]
        
        path, maxDistMoved = self.steer(nearestNodeNp, rConf, dQ)
        
        if(len(path) == 0):
            #Blocked
            return self.found
        
        if(strict and not maxDistMoved):
            #Strictly max step no collision
            return self.found