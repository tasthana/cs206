

#solution python file
import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import os
import random as random
import math
import constants as c
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK

class SOLUTION:
    def __init__(self):
        self.weights = numpy.random.rand(3,2)
        self.weights = self.weights *2 - 1

    def Mutate(self):
        randomRow = random.randint(0, 2)
        randomColumn = random.randint(0, 1)
        self.weights[randomRow,randomColumn] = random.random() * (2 - 1)
        self.Generate_Mind(self)


    def Evaluate(self, view):
        file = "python3 simulate.py " + view
        os.system(file)
        f = open("fitness.txt","r")
        self.fitness = float(f.read())
        f.close() 

    def Create_World():
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[2,2,0.5] , size=[1,1,1])
        pyrosim.End()

    def Generate_Body(torso_coordinates, link_size):
        pyrosim.Start_URDF("body.urdf")
        joint_torso_backleg_coordinates = [torso_coordinates[0]-0.5, 0, torso_coordinates[2]-(0.5*link_size[2])]
        backLeg_coordinates = [link_size[0]*(-0.5),0,link_size[2]*(-0.5)]
        joint_torso_frontleg_coordinates = [torso_coordinates[0]+0.5, 0, torso_coordinates[2]-(0.5*link_size[2])]
        frontLeg_coordinates = [link_size[0]*0.5,0,link_size[2]*(-0.5)]
        pyrosim.Send_Cube(name="Torso", pos=torso_coordinates , size=link_size) # absolute
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = joint_torso_backleg_coordinates)
        pyrosim.Send_Cube(name="BackLeg", pos=backLeg_coordinates , size=link_size)
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = joint_torso_frontleg_coordinates)
        pyrosim.Send_Cube(name="FrontLeg", pos=frontLeg_coordinates , size=link_size)
        pyrosim.End()

    def Generate_Mind(link_size,self):
        pyrosim.Start_NeuralNetwork("brain.nndf")

        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

        for currentRow in range(0,3):
            for currentColumn in range(0,2):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+3, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()