import math
import random

import time
import logging

from dataclasses import dataclass

import numpy as np
import scipy.stats

import matplotlib.pyplot as plt

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.IMU_data.IMU_build import IMU
from idl_data.Frame_data.Frame_build import Frame

@dataclass
class Obstacle: 
    orientation: tuple # x, y, angle (angle of front face normal to axis)
    size: tuple # length , width
    faces: tuple # ID of face going counter clockwise from 'front' face

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

def forwardVector(angles):
    return np.transpose(np.array([[np.cos(angles)], [np.sin(angles)]]), [2,0,1])

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
    
    def plotKeypoints(self, xlim=(0, 2.0), ylim=(0, 2.0), quiverScale=20):
        fig = plt.figure()
        
        #Points
        points = self.facesFrame[:, :2, 2]
        pointsLabels = self.facesId[:]
        plt.scatter(points[:,0], points[:,1])
        
        # x
        plt.quiver(points[:,0], points[:,1], self.facesFrame[:,0,0], self.facesFrame[:,1,0], color=[(1,0,0,1)], scale=quiverScale)
        
        # y
        plt.quiver(points[:,0], points[:,1], self.facesFrame[:,0,1], self.facesFrame[:,1,1], color=[(0,1,0,1)], scale=quiverScale)
        
        for i,faceId in enumerate(self.facesId):
            plt.annotate(faceId, (points[i,0], points[i,1]))
            
        plt.axis('equal')
        plt.xlim(xlim)
        plt.ylim(ylim)
        plt.gca().set_autoscale_on(False)
        plt.grid()
        
        
        return fig
    
    def getRelativePoints(self, detectId, detectOrientation):
        # detectOrientation = [x,y,angle] #angle is angle of normal of sign saw by camera
        
        detectMask = self.facesId == detectId
        
        relPoints = np.matmul(self.facesFrame[detectMask], np.array([[[detectOrientation[0]], [detectOrientation[1]], [1.0]]]))
        facing = self.facesAngle[detectMask].reshape(-1,1,1) - detectOrientation[2] + math.pi
        facing[facing >= math.pi] = facing[facing >= math.pi] - 2*math.pi
        
        return np.concatenate([relPoints[:,:2], facing], axis=1)
        
def relPoseNormPdf(relPoses, relPosesStd, orientations, resolution=np.array([[[0.0001],[0.0001],[0.01*math.pi/180]]])):
    # K Keypoints
    # P particles
    # relPoses = (K,3,1)
    # relPosesStd = (K,3,1)
    # orientations = (1,3,P)
    
    #print(relPoses.shape)
    #print(relPosesStd.shape)
    #print(orientations.shape)
    
    posDist = scipy.stats.norm(loc=relPoses[:,:2], scale=relPosesStd[:,:2])
    angleDist = scipy.stats.norm(loc=np.expand_dims(relPoses[:,2],-1), scale=np.expand_dims(relPosesStd[:,2],-1))
    
    angles = orientations[0,2,:] + np.array([[[2*math.pi],[0.0],[-2*math.pi]]])
    
    # Using cdf
    posProb = posDist.cdf(orientations[0,:2,:]+resolution[0,:2]/2) - posDist.cdf(orientations[0,:2,:]-resolution[0,:2]/2)
    angleProb = np.max(np.abs(angleDist.cdf(angles+resolution[0,2,0]/2) - angleDist.cdf(angles-resolution[0,2,0]/2)), axis=1)
    
    # Using pdf (probably wrong)
    # posProb = posDist.pdf(orientations[0,:2,:])
    # angleProb = np.max(angleDist.pdf(angles), axis=1)
    
    prob = np.concatenate([posProb, np.expand_dims(angleProb,1)], axis=1)
    return prob

class ParticleFilter(object):

    def __init__(self, N, particles):
        self.N = N
        self.setParticles(particles)
        self.initWeight()
    
    def initWeight(self): 
        self.weights = np.ones(self.N)/N
    
    def setParticles(self, particles):
        self.particles = particles

    def resample(self): 
        index = np.random.choice(self.N, p=self.weights, size=self.N)
        self.particles = self.particles[:,index]
        self.initWeight()
        
    def neff(self):
        return 1.0/np.sum(np.power(self.weights,2))
    
    def chechAndResample(self, neffHigh):
        
        if(self.neff() < neffHigh):
            self.resample()
    
    def predict(self, movement, movementStd):
        # movement = [x, y, angle] relative to starting point of particle
        
        rotM = rotationMatrix2d(self.particles[2,:])
        self.particles += np.matmul(rotM, (np.random.normal(loc=movement, scale=movementStd)).ravel()).T
        
    def update(self, kpRelPoses, kpRelPosesStd, resolution=np.array([[[0.0001],[0.0001],[0.01*math.pi/180]]])):
        # len = number of symbols seen in image
        # kpRelPoses = [ relPoses0, relPoses1, ... ]
        # kpRelPosesStd = [ [xStd0, yStd0, angleStd0], [xStd1, yStd1, angleStd1] ]
        
        for i in range(len(kpRelPoses)):
            probs = relPoseNormPdf(kpRelPoses[i], kpRelPosesStd[i], np.expand_dims(self.particles,0), resolution=resolution)
            combinedProb = np.sum(probs, axis=0) / len(kpRelPoses[i])
            self.weights = np.multiply(self.weights, np.product(combinedProb, axis=0))
        
        self.weights = self.weights + 1e-300
        self.weights = self.weights / np.sum(self.weights)
        
    def getCurrentEstimate(self):
        return np.sum(np.multiply(self.weights, self.particles), axis=1)

def gaussianParticles(mean, std, N):
    return np.random.normal(mean, std, N)

def plotParticles(particles, xlim=[0,2], ylim=[0,2]):
    fig = plt.figure()
    plt.scatter(particles[0], particles[1])
    
    headingArrow = np.array([np.cos(particles[2]),np.sin(particles[2])])
    plt.quiver(particles[0], particles[1], headingArrow[0], headingArrow[1])
    
    plt.xlim(xlim)
    plt.ylim(ylim)
    
    return fig

class ParticleFilterNode(object):

    def __init__(self):
        # Create core

        # Create subscriber for obstacles

        # Create subscriber for imu and robot pose data

        # Create subscriber for image detection

        # Create publisher for estimated pose

        pass

    def setReadState(self, data: RobotState.RobotState):
        self.xDotRead = data.velocity()
        self.yawRead = data.steer_angle() 
        #self.wRead = data.steer_angle()

    def setReadIMU(self, data: IMU.IMU):
        self.dYaw = data.angularVelZ()

    def loop(self):

        SLEEP_TIME = 0.01
        PARTICLE_UPDATE_COUNT = 10

        lastTime = time.time()
        count = 0

        while (True):
            dt = time.time() - lastTime

            


            lastTime = time.time()
            time.sleep(SLEEP_TIME)




def main():
    obstacles = [
        Obstacle((0.5, 0.5, 0.0), (0.1, 0.1), (1,0,0,0)),
        Obstacle((0.5, 1.5, math.pi/2), (0.1, 0.1), (1,0,0,0)),
        Obstacle((1.0, 1.5, 5*math.pi/4), (0.2, 0.1), (1,0,0,0)),
        ]

    robotMap = Map(obstacles)

    while(True):

    
if __name__ == "__main__":
    main()