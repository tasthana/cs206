


 
from world import WORLD
from robot import ROBOT
import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random 
import constants as c

class SIMULATION:

    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT()

        self.world = p.loadSDF("world.sdf")
        ROBOT.Prepare_To_Sense(self)




    def Runs(self):

        for x in range(c.length):
            p.stepSimulation()
            self.robot.Sense(x)
            self.robot.Act(x)

            time.sleep(1/100)
            print(x)



    def __del__(self):
        p.disconnect()
