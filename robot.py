



from motor import MOTOR
from sensor import SENSOR
import pybullet as p
import pybullet_data
import generate
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random 
import constants as c


class ROBOT:

    def __init__(self):
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_to_Act()


    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)


    def Sense(self, t): 
        for i in self.sensors:
            self.sensors[i].Get_Value(t)

    def Prepare_to_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self,t):
        for i in self.motors:
            self.motors[i].Set_Value(t, self.robotId)

        
      

    