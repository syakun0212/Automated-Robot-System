from types import FunctionType

import math
import random
import time

import numpy as np

ROBOT_T = 0.1600
ROBOT_L = 0.1400
WHEEL_R = 0.0325

MAX_ACC = WHEEL_R * 1.5 * 2 * math.pi

class EKF(object):

    def __init__(self, f: FunctionType, F: FunctionType, h: FunctionType, H: FunctionType):
        self.f = f
        self.F = F

        self.h = h
        self.H = H

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

    matH = np.zeros((8,8), dtype=np.float)

    matH[0,1] = cT
    matH[0,4] = sT
    matH[0,6] = -xV*sT + yV*cT

    matH[1,7] = 1

    matH[2,2] = cT
    matH[2,5] = sT
    matH[2,6] = -xA*sT + yA*cT

    matH[3,2] = -sT
    matH[3,5] = cT
    matH[3,6] = -xA*cT - yA*sT

    matH[4,7] = 1

    matH[5,0] = 1
    matH[6,3] = 1
    matH[7,6] = 1

    return matH

def R(dt):
    return np.diag([0.1, 0.1, 0.02, 0.02, 0.1, 10000.0, 10000.0, 10000.0])


def forwardKinematic(vl, vr):
    v = (vl + vr)/2
    w = math.atan2(2*ROBOT_L*(vr-vl), ROBOT_T*(vr+vl))
    return v,w

def inverseKinematic(vel, angle):
    tanAngle = math.tan(angle)
    c1 = ROBOT_T / (2 * ROBOT_L)

    leftCmd = -vel * (1 - tanAngle*c1) / (2*WHEEL_R*math.pi)
    rightCmd = vel * (1 + tanAngle*c1) / (2*WHEEL_R*math.pi)

    return leftCmd, rightCmd


def main():

    dt = 0.1

    # Initialization
    x = np.zeros((8,1), dtype=np.float)

    predictNoise = np.diag([0.5, 0.02, 0.02, 0.5, 0.02, 0.02, 0.2, 0.02])

    controlledInput = np.array([[0.0], [0.0]])
    measurements = np.expand_dims(np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]), -1)

    matQ = Q(dt, 0.01, 0.01, 0.02)

    maxTime = 10
    curTime = 0

    
    while(curTime <= maxTime):
        # Predict
        xPredict = f(x,dt)
        matF = F(x,dt)
        predictNoise = matF @ predictNoise @ matF.T + matQ

        predictMeasurement = h(xPredict)
        innovation = measurements - predictMeasurement
        matH = H(xPredict)
        matS = matH @ predictNoise @ matH.T + R(dt)

        # Update
        K = predictNoise @ matH.T @ np.linalg.inv(matS)
        x = xPredict + K @ innovation
        predictNoise = (np.identity(8) - K@matH) @ predictNoise

        #print(x)
        
        print(f"Predicted state\n{xPredict}\nEstimated State\n{x}\nPredict Measurement\n{predictMeasurement}\ninnovation\n{innovation}")

        #curTime += dt
    pass

if __name__ == "__main__":
    main()