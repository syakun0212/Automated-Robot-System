import math
from pydoc import describe
import random

import time
import logging

import queue
import pickle
import cv2 as cv

from dataclasses import dataclass

import numpy as np
import scipy.stats

import matplotlib.pyplot as plt

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.IMU_data.IMU_build import IMU
from idl_data.Frame_data.Frame_build import Frame
from idl_data.Obstacle_data.Obstacle_build import Obstacle
from idl_data.Message_data.Message_build import Message
from idl_data.ImageLocalisation_data.ImageLocalisation_build import ImageLocalisation

logging.basicConfig(level=logging.DEBUG)

@dataclass
class ObstacleData: 
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
        
        self.facesVertex = []
        
        for o in obstacles:
            
            x,y,a = o.orientation
            l,w = o.size #length , width
            cA = math.cos(a)
            sA = math.sin(a)

            symW = 0.06
            symL = 0.06
            
            vertices = []
            vertices.append(np.array([[0,0,0,0], [-symW/2, symW/2, symW/2, -symW/2], [1.0, 1.0, 1.0, 1.0]]))
            vertices.append(np.array([[0,0,0,0], [-symL/2, symL/2, symL/2, -symL/2], [1.0, 1.0, 1.0, 1.0]]))
            vertices.append(np.array([[0,0,0,0], [-symW/2, symW/2, symW/2, -symW/2], [1.0, 1.0, 1.0, 1.0]]))
            vertices.append(np.array([[0,0,0,0], [-symL/2, symL/2, symL/2, -symL/2], [1.0, 1.0, 1.0, 1.0]]))
            vertices = np.stack(vertices, 0)
            
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
            
            vertTransformed = np.matmul(base, vertices)
            vertTransformed[:,2] = [0.15, 0.15, 0.09, 0.09] # Symbol top and bot
            
            self.facesId.append(ids)
            self.facesFrame.append(base)
            self.facesAngle.append(facesAngle)
            self.facesVertex.append(vertTransformed)
            
        self.facesId = np.concatenate(self.facesId)
        self.facesFrame = np.concatenate(self.facesFrame)
        self.facesAngle = np.concatenate(self.facesAngle)
        self.facesVertex = np.concatenate(self.facesVertex)
    
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
    
    def getProjection(self, pose, intrinsics, camHeight=0.12, epsilon=1e-6):
        # Project obstacle bounds 
        # pose = [1x3]
        direction = np.array([[math.cos(pose[0,2])], [math.sin(pose[0,2])]])
        r = self.facesFrame[:,:2,2] - pose[:,:2]
        dirCond = np.matmul(r, direction)
        dirCond[np.abs(dirCond) < epsilon] = 0
        normalCond = np.matmul(self.facesFrame[:,:2,0], direction)
        normalCond[np.abs(normalCond) < epsilon] = 0.0
        mask = np.logical_and(dirCond > 0, normalCond < 0)
        verticies = self.facesVertex[mask.ravel()]
        
        P = intrinsics
        
        up = np.array([0.0,0.0,1.0])
        front = np.array([direction[0,0], direction[1,0], 0.0])
        
        right = np.cross(front, up)#np.cross(up, front)#np.cross(front, up)
        camUp = np.cross(right, front)#np.cross(front, right)#np.cross(right, front)
        
        V = np.identity(4)
        V[0,:3] = right
        V[1,:3] = camUp
        V[2,:3] = front
        temp = np.identity(4)
        temp[:2, 3] = -pose[:,:2]
        temp[2, 3] = -camHeight
        V = np.matmul(V, temp)
        
        projectedVert = []
        worldV = []
        for i in range(len(verticies)):
            #print(verticies[i].shape)
            model = np.concatenate([verticies[i], [[1.0, 1.0, 1.0, 1.0]]], axis=0)
            world = np.matmul(V, model)
            worldV.append(world)
            projected = np.matmul(P, world)
            projectedVert.append(projected)
        return np.array(projectedVert), np.array(worldV), np.arange(len(mask))[mask.ravel()]
        
        
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
        self.weights = np.ones(self.N)/self.N
    
    def setParticles(self, particles):
        self.particles = particles

    def resample(self): 
        index = np.random.choice(self.N, p=self.weights, size=self.N)
        self.particles = self.particles[:,index]
        self.initWeight()
        
    def neff(self):
        return 1.0/np.sum(np.power(self.weights,2))
    
    def checkAndResample(self, neffHigh):
        
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
        pos = self.particles
        mean = np.sum(np.multiply(self.weights, self.particles), axis=1)
        var = np.sum(np.multiply(self.weights, np.power(pos - np.expand_dims(mean,-1),2)), axis=1)
        return mean, var

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

    def __init__(self, N, camMatrix):
        # Create core
        self.core = RobotCore.Core()

        # Create subscriber for obstacles
        self.obsSub = self.core.createSubscriber(Obstacle, "obstacle_data", self.setObstacles, 10)
        self.msgSub = self.core.createSubscriber(Message, "startstop", self.confirmObstacles, 10)
        self.startPosSub = self.core.createSubscriber(Frame, "start_data", self.setStartPose, 10 )

        # Create subscriber for imu and robot pose data
        self.frameSub = self.core.createSubscriber(Frame, "particle_frame_update", self.addFrameUpdate,1)

        # Create subscriber for image detection
        self.sequenceSub = self.core.createSubscriber(Message, "trigger", self.storeTriggerPose, 10)
        self.imgDetSub = self.core.createSubscriber(ImageLocalisation, "img_result", self.queueImageResult, 10)

        # Create publisher for estimated pose
        self.posePub = self.core.createPublisher(Frame, "robot_est_pose", 1)

        # Create publisher for image capture
        self.pathProgressSub = self.core.createSubscriber(Message, "path_progress", self.setPathProgress, 1)

        self.obsUpdatePub = self.core.createPublisher(Message, "send_message", 10)

        self.obstaclesInput = None
        self.start = False

        self.frameUpdateQueue = queue.Queue(maxsize=20)
        self.startPose = np.array([[0.15],[0.15],[90*math.pi/180]])

        self.N = N
        self.map = None
        self.pf = None

        self.poseDict = dict()
        self.detectionDict = dict() # [id, x, y, a, bbMinX, bbMaxX, bbMinY, bbMaxY]

        self.detectionProcessQueue = queue.Queue(maxsize=20) 

        self.projMatrix = np.concatenate([camMatrix, [[0],[0],[0]]], 1)

        self.detectionBank = None

        self.pathProgress = -1 # If current path is -1, else obstacle id

        pass

    def setObstacles(self, data: Obstacle.Obstacle, obstacleSize=(0.1, 0.1), obstacleCount=41):
        obsX = list(data.obstaclesX())
        obsY = list(data.obstaclesY())
        obsDir = list(data.obstacleDir())
        self.obstaclesInput = []
        self.detectionBank = []

        for x,y,d in zip(obsX, obsY, obsDir):
            if(d == "N"):
                ang = math.pi/2
            elif(d == "S"):
                ang = -math.pi/2
            elif(d == "E"):
                ang = 0
            elif(d == "W"):
                ang = -math.pi
            self.obstaclesInput.append(ObstacleData((x/100,y/100,ang), obstacleSize, (-1, 0, 0, 0))) # -1 place holder for ones to identify
            self.detectionBank.append([0 for i in range(obstacleCount)])
        
        logging.debug(f"Full Obstacle Input: {self.obstaclesInput}")
    
    def confirmObstacles(self, data: Message.Message):
        startMsg = data.message()
        if(startMsg == "image"):
            self.start = True

    def setStartPose(self, data: Frame.Frame):
        self.startPose = np.array([[data.x()/100],[data.y()/100],[data.yaw()]])
        print(self.startPose)

    def addFrameUpdate(self, data: Frame.Frame):
        #logging.debug(f"Frame update received")
        if(self.start):
            if(not self.frameUpdateQueue.full()):
                self.frameUpdateQueue.put((data.x(), data.y(), data.yaw()))
            else:
                logging.debug("[Error] Buffer full")

    def storeTriggerPose(self, data: Message.Message):
        '''
            Sequence number is data.message()
        '''
        if(self.pf is not None):
            logging.debug(f"Message [{data.message()}]: Received")
            self.poseDict[data.message()] = self.pf.getCurrentEstimate()
    
    def setPathProgress(self, data: Message.Message):
        print(f"Path Progress: {data.message()}\n\n")
        self.pathProgress = int(data.message())

    def queueImageResult(self, data: ImageLocalisation.ImageLocalisation):

        sequenceNum = data.sequenceNum()
        
        detections = data.imageID()
        if(len(detections) == 0):
            del self.poseDict[sequenceNum]
            return

        dets = []

        xScale = 640/416
        yScale = 480/416

        for id, x, y, a, bbMinX, bbMaxX, bbMinY, bbMaxY in zip(detections, data.x(), data.y(), data.angle(), data.xMin(), data.xMax(), data.yMin(), data.yMax()):
            dets.append([id, x, y, a, bbMinX*xScale, bbMaxX*xScale, bbMinY*yScale, bbMaxY*yScale])

        self.detectionDict[sequenceNum] = dets
        if(not self.detectionProcessQueue.full()):
            self.detectionProcessQueue.put(sequenceNum)


    def initPF(self, x0Std=np.array([[0.05],[0.05],[5 * math.pi/180]])):

        x0 = self.startPose

        self.pf = ParticleFilter(self.N, np.random.normal(x0, x0Std, (x0.shape[0], self.N)))

    def loop(self):

        SLEEP_TIME = 0.01
        POSE_UPDATE_COUNT = 100

        POSE_MATCH_SAMPLES = 10

        while(not self.obsSub.matched()):
            logging.debug("Waiting for Obs Match")
            time.sleep(0.2)

        while(not self.frameSub.matched()):
            logging.debug("Waiting for Frame Match")
            time.sleep(0.2)

        while(not self.pathProgressSub.matched()):
            logging.debug("Waiting for progress match")
            time.sleep(0.2)

        while(not self.start):
            logging.debug("Waiting for start")
            time.sleep(0.2)

        # Set map
        self.map = Map(self.obstaclesInput)
        self.initPF()

        lastTime = time.time()
        poseCount = 0

        movementStd = np.array([[0.02],[0.02],[2*math.pi/180]])

        while (True):
            dt = time.time() - lastTime

            if(not self.frameUpdateQueue.empty()):
                displacement = self.frameUpdateQueue.get()
                if(displacement[0] != 0 or displacement[1] != 0):
                    movement = np.array([[displacement[0]], [displacement[1]], [displacement[2]]])
                    # print(movement)
                    self.pf.predict(movement, movementStd)

            if(not self.detectionProcessQueue.empty()):
                # Process Image Data

                # Update particle
                curSeqNum = self.detectionProcessQueue.get()

                dets = self.detectionDict[curSeqNum] # Technically in the past but no pose graph optimization algo T.T
                detRelPoses = []
                detRelPosesStd = []

                curEst, curVar = self.pf.getCurrentEstimate() #self.poseDict[curSeqNum]
                curEst = np.expand_dims(curEst, -1)
                curVar = np.expand_dims(curVar, -1)
                estBBId = []
                estCenters = []

                samplePoses = np.random.normal(curEst, np.sqrt(curVar), (curEst.shape[0], POSE_MATCH_SAMPLES))

                for i in range(POSE_MATCH_SAMPLES):
                    projectedVert, viewVert, facesId = self.map.getProjection(np.expand_dims(samplePoses[:,i], 0), self.projMatrix)
                    if(len(projectedVert) != 0):
                        projectedVertN = np.divide(projectedVert, np.expand_dims(projectedVert[:,2,:],1))
                        # 640 x 480 img size
                        mask = np.all(np.logical_and(np.logical_and(projectedVertN[:,0,:] <= 640, projectedVertN[:,1,:] <= 480), np.logical_and(projectedVertN[:,0,:] > 0, projectedVertN[:,1,:] > 0)), 1)
                        #print(mask)
                        vertCenters = np.sum(projectedVertN[:,:2,:], axis=2) / projectedVertN.shape[2]
                        estBBId.append((projectedVert[mask], facesId[mask]))
                        estCenters.append(vertCenters[mask])

                detCenters = []
                detIds = []

                maxAreaId = -1
                maxArea = 0

                for id, x, y, a, bbMinX, bbMaxX, bbMinY, bbMaxY in dets:

                    if(id == 0):
                        # Bullseye
                        relPoses = self.map.getRelativePoints(id, [x,y,a])
                        rotM = rotationMatrix2d(relPoses[:,2,0])
                        camOffset = np.array([[-0.097],[0.0],[0.0]]) # Camera frame to robot frame displacement
                        relPoses += np.matmul(rotM, camOffset)
                        detRelPoses.append(relPoses)
                        detRelPosesStd.append(np.array([[[0.05], [0.05], [20*math.pi/180]]])) # Standard deviation
                    else:
                        area = (bbMaxX - bbMinX) * (bbMaxY - bbMinY)
                        if(area > maxArea):
                            maxArea = area
                            maxAreaId = id

                    detCenters.append([(bbMinX + bbMaxX)/2, (bbMinY + bbMaxY)/2])
                    detIds.append(id)



                detCenters = np.array(detCenters, dtype=np.float32)

                '''
                if(len(estCenters) != 0 and len(detCenters) != 0):
                    for i in range(len(estCenters)):
                        estNCenters = estCenters[i].astype(np.float32)
                        matcher = cv.BFMatcher.create(cv.NORM_L2, crossCheck=True)
                        matches = matcher.match(estNCenters, detCenters)
                        for dm in matches:
                            bb, obsId = estBBId[i]
                            symId = int(detIds[dm.trainIdx])
                            if(self.detectionBank[int(obsId[dm.queryIdx]) // 4][symId] <= 100):
                                self.detectionBank[int(obsId[dm.queryIdx]) // 4][symId] += 1 # 4 sides
                '''

                print(self.pathProgress, maxAreaId)
                if(self.pathProgress > -1 and maxAreaId > -1):
                    if(self.detectionBank[self.pathProgress][maxAreaId] < 100):
                        self.detectionBank[self.pathProgress][maxAreaId] += 1


                if(len(detRelPoses) > 0):
                    self.pf.update(detRelPoses, detRelPosesStd)
                    self.pf.checkAndResample(self.N/2)

                # Add to detection bank

                pass
            

            if(poseCount >= POSE_UPDATE_COUNT):
                curEst, curVar = self.pf.getCurrentEstimate()

                logging.debug(f"Cur Est: {curEst} {curVar}")

                robotPose = Frame.Frame()

                robotPose.x(curEst[0])
                robotPose.y(curEst[1])
                robotPose.yaw(curEst[2])

                self.posePub.publish(robotPose)

                poseCount = 0

                obsPair = []

                logMsg = ""
                logging.info(self.detectionBank)
                for i in range(len(self.detectionBank)):
                    if(np.sum(self.detectionBank[i]) != 0):
                        logMsg += f"[{i}]: {np.argmax(self.detectionBank[i])}, "
                        #logMsg += f"[{i}]: {self.detectionBank)}, "
                        obsPair.append((i+1, np.argmax(self.detectionBank[i])))
                    else:
                        logMsg += f"[{i}]: -1, "
                logging.info(logMsg)

                for o in obsPair:
                    obsId, imgId = o
                    obsMsg = Message.Message()
                    obsMsg.message(f"TARGET,{obsId},{imgId}")
                    self.obsUpdatePub.publish(obsMsg)
            
            poseCount += 1

            lastTime = time.time()
            time.sleep(SLEEP_TIME)




def main():
    calib_result_pickle = pickle.load(open("new_camera_calib_pickle.p", "rb" ))
    mtx = calib_result_pickle["mtx"]
    optimal_camera_matrix = calib_result_pickle["optimal_camera_matrix"]
    dist = calib_result_pickle["dist"]
    roi = calib_result_pickle["roi"]

    particlesCount = 1000
    pfNode = ParticleFilterNode(particlesCount, optimal_camera_matrix)

    try:
        pfNode.loop()
    except KeyboardInterrupt:
        logging.info("Exiting")

    
if __name__ == "__main__":
    main()