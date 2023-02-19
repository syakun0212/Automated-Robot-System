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
from idl_data.Command_data.Command_build import Command

logging.basicConfig(level=logging.DEBUG)

class StateMachine(object):
    def __init__(self):
        self.stateFuncs = [] # List of function pointers f(cur state) -> next state
    
    def setState(self, stateInd):
        self.index = stateInd

class ControllerNode(object):

    def __init__(self):
        self.core = RobotCore.Core()

        self.cmdPub = self.core.createPublisher(RobotState, "robot_cmd", 1) # Control manual
        self.pathPub = self.core.createPublisher(Command, "robot_path", 10) # Control via path

        self.resSub = self.core.createSubscriber(ImageLocalisation, "img_result", self.queueImageResult, 1)
        self.sequenceSub = self.core.createSubscriber(Message, "trigger", self.storeTriggerPose, 10)
        self.msgSub = self.core.createSubscriber(Message, "startstop", self.setStart, 1)

        # Create subscriber for imu and robot pose data
        self.frameSub = self.core.createSubscriber(Frame, "particle_frame_update", self.addFrameUpdate,1)

        self.frameUpdateQueue = queue.Queue(maxsize=20)

        self.detectionDict = dict() # [id, x, y, a, bbMinX, bbMaxX, bbMinY, bbMaxY]

        self.detectionProcessQueue = queue.Queue(maxsize=20) 

        self.poseDict = dict()

        self.start = False
        self.moveDistance = 0

    
    def queueImageResult(self, data: ImageLocalisation.ImageLocalisation):

        if(not self.start):
            return

        logging.debug("received")
        sequenceNum = data.sequenceNum()
        
        detections = data.imageID()

        dets = []

        xScale = 640/416
        yScale = 480/416

        for id, x, y, a, bbMinX, bbMaxX, bbMinY, bbMaxY in zip(detections, data.x(), data.y(), data.angle(), data.xMin(), data.xMax(), data.yMin(), data.yMax()):
            dets.append([id, x, y, a, bbMinX*xScale, bbMaxX*xScale, bbMinY*yScale, bbMaxY*yScale])

        self.detectionDict[sequenceNum] = dets
        if(not self.detectionProcessQueue.full()):
            self.detectionProcessQueue.put(sequenceNum)
    
    def storeTriggerPose(self, data: Message.Message):
        '''
            Sequence number is data.message()
        '''
        if(self.start):
            logging.debug(f"Message [{data.message()}]: Received")
            self.poseDict[data.message()] = self.moveDistance

    def addFrameUpdate(self, data: Frame.Frame):
        #logging.debug(f"Frame update received")
        if(self.start):
            if(not self.frameUpdateQueue.full()):
                self.frameUpdateQueue.put((data.x(), data.y(), data.yaw()))
            else:
                logging.debug("[Error] Buffer full")

    def setStart(self, data: Message.Message):
        self.start = True
        pass

    def loop(self, SLEEP_TIME=0.05):
        
        while(not self.cmdPub.matched()):
            logging.debug("Waiting for cmd pub match")
        
        while(not self.pathPub.matched()):
            logging.debug("Waiting for path pub")

        while(not self.frameSub.matched()):
            logging.debug("Waiting for frame sub")

        while(not self.resSub.matched()):
            logging.debug("Waiting for image sub")

        while(not self.sequenceSub.matched()):
            logging.debug("Waiting for sequence sub")

        while(not self.msgSub.matched()):
            logging.debug("Waiting for msgSub sub")

        while(not self.start):
            logging.info("Waiting for start")
            time.sleep(1.0)


        # while(True):
        #     logging.info("L")
        #     time.sleep(1)

        v1 = 0.25
        v2 = 0.25
        vArc = 0.22

        dirCmd = RobotState.RobotState()
        dirCmd.velocity(v1)
        dirCmd.steer_angle(0)
        self.cmdPub.publish(dirCmd)

        # Move straight until see obs

        self.moveDistance = 0
        obstacleDistance = 0

        while(True):
            detected = False

            ySum = 0
            count = 0

            xs = []

            xMin = 10000
            xMax = -10000

            if(not self.frameUpdateQueue.empty()):
                x,y,yaw = self.frameUpdateQueue.get()
                self.moveDistance += x

            if(not self.detectionProcessQueue.empty()):
                # Process image
                
                sequenceNum = self.detectionProcessQueue.get()
                for id, x, y, a, bbMinX, bbMaxX, bbMinY, bbMaxY in self.detectionDict[sequenceNum]:
                    if(id == 0):
                        detected = True
                        ySum += y
                        count += 1
                        xs.append(x)

                        if(x < xMin):
                            xMin = x
                        
                        if(x > xMax):
                            xMax = x 
                        
            if(count >= 2):
                break
        
            time.sleep(SLEEP_TIME)

        dirCmd = RobotState.RobotState()
        dirCmd.velocity(0)
        dirCmd.steer_angle(0)
        self.cmdPub.publish(dirCmd)

        startTime = time.time()

        while(time.time() - startTime < 1.3): # Wait 2 sec
            if(not self.frameUpdateQueue.empty()):
                x,y,yaw = self.frameUpdateQueue.get()
                self.moveDistance += x


        logging.info(f"Y Sum {ySum/count} - X {xs} - {xMin} {xMax}")

        obstacleDistance = self.poseDict[sequenceNum] + (ySum/count) + 0.097
        #obstacleDistance = self.moveDistance + (ySum/count) + 0.097
        #obstacleDistance = self.poseDict[sequenceNum] + ySum + 0.097


        logging.info(f"Obstacle detected at {obstacleDistance}. Current robot distance: {self.moveDistance} - {y}")

        # Move backward to get distance of 0.8

        desiredOffset = 0.6

        moveBackDistance = -(desiredOffset - (obstacleDistance - self.moveDistance))
        cmd = Command.Command()
        cmd.type('line')
        cmd.distance(moveBackDistance)
        if(moveBackDistance > 0):
            cmd.velocity(v2)
        else:
            cmd.velocity(-v2)
        
        logging.info(f"Moving {moveBackDistance}")
        self.pathPub.publish(cmd)

        backDist = 0.3 #0.15 #0.3 works good 
        moveDif =  -(xMin + 0.15 + 0.4) #-(xMin + 0.1 + 0.4) # -(xMin + 0.15 + 0.4) Works good
        moveDif2 = backDist - moveDif
        moveDif3 = -(obstacleDistance - desiredOffset)

        logging.info(f"Detected {xMin} - {xMax}. {moveDif} - {moveDif2} - {moveDif3}")

        cmdSet = [
            ('arc', math.pi/2, 0.4, vArc),
            ('line', moveDif, v2 if moveDif > 0 else -v2),
            # -0.4 current,  target = xMin + 0.15
            ('arc', -2*math.pi/2, -0.4, vArc), 
            ('line', backDist, v2), 
            ('arc', -2*math.pi/2, -0.4, vArc),
            ('line', moveDif2, v2 if moveDif2 > 0 else -v2),
            ('arc', -math.pi/2, 0.4, -vArc),
            ('line', moveDif3, v2 if moveDif3 > 0 else -v2)
        ]

        for cmd in cmdSet:
            cmdMsg = Command.Command()
            if(cmd[0] == 'line'):
                cmdMsg.type('line')
                cmdMsg.distance(cmd[1])
                cmdMsg.velocity(cmd[2])
            elif(cmd[0] == 'arc'):
                cmdMsg.type('arc')
                cmdMsg.desired_angle(cmd[1])
                cmdMsg.turning_radius(cmd[2])
                cmdMsg.velocity(cmd[3])
            
            self.pathPub.publish(cmdMsg)
            time.sleep(0.07)
        
        # Move worst case around obs

        # Move back


        # while (True):

        #     time.sleep(SLEEP_TIME)
        

def main():

    contNode = ControllerNode()

    try:
        contNode.loop()
    except KeyboardInterrupt:
        logging.info("Exiting")

    
if __name__ == "__main__":
    main()