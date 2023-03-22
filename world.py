



import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random 
import constants as c

class WORLD:

    def __init__(self, solutionID):
        self.planeId = p.loadURDF("plane.urdf")

        p.loadSDF("world" + solutionID + ".sdf")

        
        

    