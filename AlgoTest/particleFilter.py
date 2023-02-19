import math
import random

import time
import logging

import numpy as np

def scaleDistribution(dist):
    return dist / np.sum(dist,0)

class ParticleFilter(object):

    def __init__(self):
        pass

    def reset(self):
        pass

    def sample(self, points=1000):
        pass