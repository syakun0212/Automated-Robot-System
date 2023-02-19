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

def f(xK_1, uK1, dt):

    xP = xK_1[0,0]
    xV = xK_1[1,0]
    xA = xK_1[2,0]
    yP = xK_1[3,0]
    yV = xK_1[4,0]
    yA = xK_1[5,0]
    t = xK_1[6,0]
    w = xK_1[7,0]
    v = uK1[0,0]
    ts = uK1[1,0]

    cSteer = math.cos(t + ts)
    sSteer = math.sin(t + ts)

    xACmd = np.clip((v * cSteer - xV)/dt, -MAX_ACC, MAX_ACC)
    yACmd = np.clip((v * sSteer - yV)/dt, -MAX_ACC, MAX_ACC)

    dt2 = dt*dt

    xK = np.array([
        [xP + dt * xV + 0.5 * dt2 * (xA + xACmd)],
        [xV + dt* (xA + xACmd)],
        [xA + xACmd],
        [yP + dt * yV + 0.5 * dt2 * (yA + yACmd)],
        [yV + dt* (yA + yACmd)],
        [yA + yACmd],
        [t + dt * ts],
        [ts]
    ])

    return xK

def F(xK_1, uK1, dt):
    xP = xK_1[0,0]
    xV = xK_1[1,0]
    xA = xK_1[2,0]
    yP = xK_1[3,0]
    yV = xK_1[4,0]
    yA = xK_1[5,0]
    t = xK_1[6,0]
    w = xK_1[7,0]
    v = uK1[0,0]
    ts = uK1[1,0]

    cSteer = math.cos(t + ts)
    sSteer = math.sin(t + ts)

    xACmd = np.clip((v * cSteer - xV)/dt, -MAX_ACC, MAX_ACC)
    yACmd = np.clip((v * sSteer - yV)/dt, -MAX_ACC, MAX_ACC)

    dt2 = dt*dt

    matF = np.zeros((8,8), dtype=np.float)
    
    matF[0,0] = 1
    matF[0,1] = dt/2
    matF[0,2] = dt2/2
    matF[1,2] = dt
    matF[2,1] = -1/dt
    matF[2,2] = 1

    matF[3,3] = 1
    matF[3,4] = dt/2
    matF[3,5] = dt2/2
    matF[4,5] = dt
    matF[5,4] = -1/dt
    matF[5,5] = 1

    matF[1,6] = - v * sSteer
    matF[0,6] = matF[1,6] * dt / 2
    matF[2,6] = matF[1,6] / dt

    matF[4,6] = v * cSteer
    matF[3,6] = matF[4,6] * dt / 2
    matF[5,6] = matF[4,6] / dt

    matF[6,6] = 1

    if(abs(xACmd) > MAX_ACC):
        matF[0,0] = 1
        matF[0,1] = dt
        matF[0,2] = dt2/2
        matF[1,1] = 1
        matF[1,2] = dt
        matF[2,1] = 0
        matF[2,2] = 1
        
        matF[0:3, 6] = 0
    
    if(abs(yACmd) > MAX_ACC):
        matF[3,3] = 1
        matF[3,4] = dt
        matF[3,5] = dt2/2
        matF[4,4] = 1
        matF[4,5] = dt
        matF[5,4] = 0
        matF[5,5] = 1
        
        matF[3:7, 6] = 0


    return matF

def Q():
    matQ = np.zeros((8,8), dtype=np.float)

    matQ[1,1] = 0.05

    matQ[4,4] = 0.05

    matQ[7,7] = 0.05

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

def R(dt):
    return np.diag([0.1, 0.1, 0.1, 0.02, 0.02, 0.1, 10000.0, 10000.0, 10000.0])


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

    # Initialization
    x = np.zeros((8,1), dtype=np.float)

    predictNoise = np.diag([0.5, 0.02, 0.02, 0.5, 0.02, 0.02, 0.2, 0.02])

    controlledInput = np.array([[0.0], [0.0]])

    dt = 0.1
    maxTime = 1
    curTime = 0

    measurements = np.expand_dims(np.array([0.0, 0.0, 0.0, 0.02, 0.02, 0.0, 0.0, 0.0, 0.0]), -1)

    while(curTime <= maxTime):
        # Predict
        xPredict = f(x,controlledInput,dt)
        matF = F(x, controlledInput,dt)
        predictNoise = matF @ predictNoise @ matF.T + Q()

        innovation = measurements - h(xPredict)
        matH = H(xPredict)
        matS = matH @ predictNoise @ matH.T + R(dt)

        # Update
        K = predictNoise @ matH.T @ np.linalg.inv(matS)
        x = xPredict + K @ innovation
        predictNoise = (np.identity(8) - K@matH) @ predictNoise

        print(x)
        #print(K.shape)
        #print(innovation.shape)
    print
    pass

if __name__ == "__main__":
    main()