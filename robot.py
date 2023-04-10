



from motor import MOTOR
from sensor import SENSOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import pybullet as p
import pybullet_data
import generate
import time 
import pyrosim.pyrosim as pyrosim
import random as random
import numpy
import os 
import constants as c
import math


class ROBOT:

    def __init__(self, solutionID):
        self.solutionID = solutionID

        self.robotId = p.loadURDF("body" + self.solutionID + ".urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_to_Act()

        self.nn = NEURAL_NETWORK("mind" + self.solutionID + ".nndf")

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
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                for jointName in self.motors:
                    self.motors[jointName].Set_Value(desiredAngle * c.motorJointRange, self.robotId) 
            

    def Think(self):
        self.nn.Update()
        # self.nn.Print()

    def Get_Fitness(self):
        #tuple of states
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        # x,y,z coords of tuple[0]
        basePosition = basePositionAndOrientation[0]
        # x position in first tuple element
        xPosition = basePosition[0]

        tempFitness = open("tmp" + self.solutionID + ".txt", "w")
        tempFitness.write(str(xPosition))
        tempFitness.close()
        # Write to file
        # os.system("mv tmp" + self.solutionID + ".txt " "fitness" + self.solutionID + ".txt")
        os.rename("tmp"+str(self.solutionID)+".txt" , "fitness"+str(self.solutionID)+".txt")

       
        




        
      

    