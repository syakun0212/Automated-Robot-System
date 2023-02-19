import math
import sys
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

ROBOT_T = 0.16
ROBOT_L = 0.14
WHEEL_R = 0.0325

def inverseKinematic(angle, velocity):
    
    #sina2 = np.sin(angle/2)
    tana = np.tan(angle)
    l_v = np.multiply(velocity, (1 - tana*ROBOT_T/(2*ROBOT_L))) / (2 * math.pi * WHEEL_R)
    r_v = np.multiply(velocity, (1 + tana*ROBOT_T/(2*ROBOT_L))) / (2 * math.pi * WHEEL_R)
    return l_v, r_v


if __name__ == "__main__":

    fileName = sys.argv[1]

    data = pd.read_csv(fileName)

    l_v, r_v = inverseKinematic(data["a"], data["v"])
    l_v *= -1

    yLabel = "Vel (rps)"

    try:
        if(sys.argv[2] == "m"):
            l_v *= 2 * math.pi * WHEEL_R
            r_v *= 2 * math.pi * WHEEL_R
            yLabel = "Vel (m/s)"
    except IndexError:
        pass
    

    fig, ax = plt.subplots(nrows=1, ncols=2)
    ax[0].plot(data["t(s)"], data["l"])
    ax[0].plot(data["t(s)"], l_v)
    ax[0].legend(["Actual", "Command"])
    ax[0].set_title("Left Wheel")
    ax[0].set_xlabel("t(s)")
    ax[0].set_ylabel(yLabel)

    ax[1].plot(data["t(s)"], data["r"])
    ax[1].plot(data["t(s)"], r_v)
    ax[1].legend(["Actual", "Command"])
    ax[1].set_title("Right Wheel")
    ax[1].set_xlabel("t(s)")
    ax[1].set_ylabel(yLabel)

    #ax[0].plot(data["t(s)"], data["l"])
    #ax[0].plot(data["t(s)"], l_v*np.ones_like(data["l"]))

    plt.show()