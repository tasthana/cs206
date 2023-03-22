


 
from world import WORLD
from robot import ROBOT
from pyrosim.neuralNetwork import NEURAL_NETWORK
import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random 
import constants as c

class SIMULATION:

    def __init__(self, directOrGUI, solutionID):
        self.directOrGUI = directOrGUI

        if (directOrGUI == "DIRECT"):
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        self.world = WORLD(solutionID)
        self.robot = ROBOT(solutionID)
        
        ROBOT.Prepare_To_Sense(self)

    def __del__(self):
            p.disconnect()

    def Runs(self):
        for x in range(c.length):
            p.stepSimulation()
            self.robot.Sense(x)
            self.robot.Think()
            self.robot.Act(x)
            if (self.directOrGUI == "GUI"):
                time.sleep(1/240)


    def Get_Fitness(self):
        self.robot.Get_Fitness()
    
