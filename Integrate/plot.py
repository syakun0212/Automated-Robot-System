import time
import math
import logging

import numpy as np

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.IMU_data.IMU_build import IMU

import matplotlib.pyplot as plt

class StatePlotter(object):
    def __init__(self, maxDataLen, stateScale=[1.0, 0.1*math.pi/180], imuScale=[1.0, 1.0, 0.1*math.pi/180], yRange=None):

        self.maxDataLen = maxDataLen
        self.yRange = yRange # None means auto set
        self.stateScale = stateScale
        self.imuScale = imuScale

        self.core = RobotCore.Core()

        self.stateSub = self.core.createSubscriber(RobotState, "robot_state",  self.appendStateData, 1)
        self.imuSub = self.core.createSubscriber(IMU, "imu_data", self.appendIMUData, 1)

        self.figure = None

        self.xS = [0]
        self.yS = [[0.0, 0.0]]

        self.xI = [0]
        self.yI = [[0.0, 0.0, 0.0]]

    def appendStateData(self, data: RobotState.RobotState):
        
        self.xS.append(self.xS[-1] + 1)
        self.yS.append([ d*s for d,s in zip([data.velocity(), data.steer_angle()], self.stateScale) ])

        if(len(self.xS) > self.maxDataLen):
            self.xS = self.xS[1:]
            self.yS = self.yS[1:]
    
    def appendIMUData(self, data: IMU.IMU):
        
        self.xI.append(self.xI[-1] + 1)
        self.yI.append([ d*s for d,s in zip([data.accelerationX(), data.accelerationY(), data.angularVelZ()], self.imuScale) ])

        if(len(self.xI) > self.maxDataLen):
            self.xI = self.xI[1:]
            self.yI = self.yI[1:]

    def plotLoop(self, delay=0.10):
        self.figure, self.ax = plt.subplots(nrows=2)
        
        while(True):
            self.ax[0].clear()
            self.ax[0].plot(self.xS, self.yS)
            self.ax[0].legend(["Velocity", "Angle"])

            self.ax[1].clear()
            self.ax[1].plot(self.xI, self.yI)
            self.ax[1].legend(["X Acc", "Y Acc", "W"])
            
            plt.pause(delay)

def main():
    plotter = StatePlotter(100)
    try:
        plotter.plotLoop()
    except KeyboardInterrupt:
        logging.info("Exiting")
    


if __name__ == "__main__":
    main()