import math
import time
import logging

import numpy as np

import RobotCore
from idl_data.RobotState_data.RobotState_build import RobotState
from idl_data.IMU_data.IMU_build import IMU
from idl_data.Frame_data.Frame_build import Frame

logging.basicConfig(level=logging.DEBUG)

ROBOT_T = 0.1600
ROBOT_L = 0.1400
WHEEL_R = 0.0325

MAX_ACC = WHEEL_R * 1.5 * 2 * math.pi

def f(xK_1, dt):

    xP = xK_1[0,0]
    xV = xK_1[1,0]
    xA = xK_1[2,0]
    yP = xK_1[3,0]
    yV = xK_1[4,0]
    yA = xK_1[5,0]
    t = xK_1[6,0]
    w = xK_1[7,0]

    dt2 = dt*dt

    xK = np.array([
        [xP + dt * xV + 0.5 * dt2 * xA],
        [xV + dt * xA],
        [xA],
        [yP + dt * yV + 0.5 * dt2 * yA],
        [yV + dt * yA],
        [yA],
        [t + dt * w],
        [w]
    ])

    return xK

def F(xK_1, dt):
    xP = xK_1[0,0]
    xV = xK_1[1,0]
    xA = xK_1[2,0]
    yP = xK_1[3,0]
    yV = xK_1[4,0]
    yA = xK_1[5,0]
    t = xK_1[6,0]
    w = xK_1[7,0]

    dt2 = dt*dt

    matF = np.zeros((8,8), dtype=np.float)
    
    matF[0,0] = 1
    matF[0,1] = dt
    matF[0,2] = dt2/2
    matF[1,1] = 1
    matF[1,2] = dt
    matF[2,2] = 1

    matF[3,3] = 1
    matF[3,4] = dt
    matF[3,5] = dt2/2
    matF[4,4] = 1
    matF[4,5] = dt
    matF[5,5] = 1

    matF[6,6] = 1
    matF[6,7] = dt
    matF[7,7] = 1

    return matF

def Q(dt, xAccVar, yAccVar, wVar):
    dt2 = dt*dt
    dt3 = dt*dt2
    dt4 = dt*dt3

    matKin = np.array([
        [dt4/4, dt3/2, dt2/2],
        [dt3/2, dt2, dt],
        [dt2/2, dt, 1]
    ])

    matQ = np.zeros((8,8), dtype=np.float)
    matQ[0:3, 0:3] = matKin * xAccVar
    matQ[3:6, 3:6] = matKin * yAccVar
    matQ[6:8, 6:8] = matKin[1:, 1:] * wVar

    return matQ

def h(xK_1):
    xP = xK_1[0,0]
    xV = xK_1[1,0]
    xA = xK_1[2,0]
    yP = xK_1[3,0]
    yV = xK_1[4,0]
    yA = xK_1[5,0]
    t = xK_1[6,0]
    w = xK_1[7,0]

    cT = math.cos(t)
    sT = math.sin(t)

    zK = np.array([
        [xV * cT + yV * sT],
        [-xV * sT + yV * cT],
        [w],
        [xA * cT + yA * sT],
        [-xA * sT + yA * cT],
        [w],
        [xP],
        [yP],
        [t]
    ])

    return zK

def H(xK_1):
    xP = xK_1[0,0]
    xV = xK_1[1,0]
    xA = xK_1[2,0]
    yP = xK_1[3,0]
    yV = xK_1[4,0]
    yA = xK_1[5,0]
    t = xK_1[6,0]
    w = xK_1[7,0]

    cT = math.cos(t)
    sT = math.sin(t)

    matH = np.zeros((9,8), dtype=np.float)

    matH[0,1] = cT
    matH[0,4] = sT
    matH[0,6] = -xV*sT + yV*cT

    matH[1,1] = -sT
    matH[1,4] = cT
    matH[1,6] = -xV*cT - yV*sT

    matH[2,7] = 1

    matH[3,2] = cT
    matH[3,5] = sT
    matH[3,6] = -xA*sT + yA*cT

    matH[4,2] = -sT
    matH[4,5] = cT
    matH[4,6] = -xA*cT - yA*sT

    matH[5,7] = 1

    matH[6,0] = 1
    matH[7,3] = 1
    matH[8,6] = 1

    return matH

def R(measurementNoise):
    return np.diag(measurementNoise)

class KalmanLocalizerNode(object):
    def __init__(self):
        self.core = RobotCore.Core()

        # Initial States
        self.setState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.predictNoise = np.diag([0.5, 0.02, 0.02, 0.5, 0.02, 0.02, 0.2, 0.02])

        # State Read
        self.stateXDotRead = 0.0
        self.stateWRead = 0.0

        # IMU Read
        self.imuXAcc= 0.0
        self.imuYAcc= 0.0
        self.imuW = 0.0

        self.stateSub = self.core.createSubscriber(RobotState, "robot_state", self.setStateRead, 1)
        self.imuSub = self.core.createSubscriber(IMU, "imu_data", self.setIMURead, 1)
        self.framePub = self.core.createPublisher(Frame, "robot_frame", 1)

    def setStateRead(self, data: RobotState.RobotState):
        self.stateXDotRead = data.velocity()
        self.stateWRead = data.steer_angle()

    def setIMURead(self, data: IMU.IMU):
        self.imuXAcc = data.accelerationX()
        self.imuYAcc = data.accelerationY()
        self.imuW = data.angularVelZ() 

    def setState(self, x, xDot, xDDot, y, yDot, yDDot, t, w):
        self.x = np.array([[x], [xDot], [xDDot], [y], [yDot], [yDDot], [t], [w]])

    def getMeasurement(self):
        measurement = np.array([[self.stateXDotRead], [0.0], [self.stateWRead], [self.imuXAcc], [self.imuYAcc], [self.imuW], [0.0], [0.0], [0.0]])
        return measurement
    
    def getMeasurementNoise(self):
        self.measurementNoise = np.power(np.array([0.01, 0.01, 0.007, 0.2*9.81, 0.2*9.81, 0.015, 10000.0, 10000.0, 10000.0]),2)
        return R(self.measurementNoise)

    def localizeLoop(self, delay=0.02):

        lastTime = time.time()

        accXVar = math.pow(0.1*9.81,2)
        accYVar = math.pow(0.1*9.81,2)
        wVar = math.pow(0.2,2)

        #print(F(self.x, 0.02))
        #print(Q(0.02, accXStd, accYStd, wStd))
        #return

        

        while(True):
            dt = time.time() - lastTime
            
            xPredict = f(self.x, dt)
            matF = F(self.x, dt)

            self.predictNoise = matF @ self.predictNoise @ matF.T + Q(dt, accXVar, accYVar, wVar)

            predictMeasurement = h(xPredict)
            measurements = self.getMeasurement()
            innovation = measurements - predictMeasurement
            matH = H(xPredict)
            matS = matH @ self.predictNoise @ matH.T + self.getMeasurementNoise()

            # Update
            K = self.predictNoise @ matH.T @ np.linalg.inv(matS)
            self.x = xPredict + K @ innovation
            self.predictNoise = (np.identity(8) - K@matH) @ self.predictNoise

            lastTime = time.time()
            logging.info(f"Measurement:\n{measurements}")
            #logging.info(f"Gain:\n{K}")
            logging.info(f"State[{dt}]:\n{self.x}")
            #logging.info(f"Current State [{dt}]: {self.x[0]}, {self.x[3]}, {180*self.x[6]/math.pi}, {self.x[1]}, {self.x[4]}, {180*self.x[7]/math.pi}")
            sendFrame = Frame.Frame()
            sendFrame.x(self.x[0,0])
            sendFrame.xV(self.x[1,0])
            sendFrame.xA(self.x[2,0])

            sendFrame.y(self.x[3,0])
            sendFrame.yV(self.x[4,0])
            sendFrame.yA(self.x[5,0])

            sendFrame.yaw(self.x[6,0])
            sendFrame.w(self.x[7,0])
            self.framePub.publish(sendFrame)
            time.sleep(delay)

def main():
    localizer = KalmanLocalizerNode()
    try:
        localizer.localizeLoop()
    except KeyboardInterrupt:
        logging.info("Exiting")
    

if __name__ == "__main__":
    main()