

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
    def __init__(self, myID):
        self.myID = myID
        self.weights = numpy.random.rand(c.numSensorNeurons,c.numMotorNeurons)
        self.weights = self.weights * 2 - 1

    def Set_ID(self, myID):
        self.myID = myID
    
    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Mind()
        string = "python3 simulate.py " + directOrGUI + " " + str(self.myID) + " &" 
        os.system(string)


    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness" + str(self.myID) + ".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)
        time.sleep(0.01)
        fitness = open(fitnessFileName, "r")
        self.fitness = float(fitness.read())
        fitness.close()
        os.system("rm " + fitnessFileName)

    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons - 1)
        randomColumn = random.randint(0, c.numMotorNeurons -1 )
        self.weights[randomRow,randomColumn] = random.random() * (2 - 1)
        self.Generate_Mind()


    def Evaluate(self, view):
        file = "python3 simulate.py " + view
        os.system("python3 simulate.py " + view + "&")
        f = open("fitness.txt","r")
        self.fitness = float(f.read())
        f.close() 

        
    def Create_World(self):
        pyrosim.Start_SDF("world" + str(self.myID) + ".sdf")
        pyrosim.Send_Cube(name="Box", pos=[2,2,0.5] , size=[1,1,1])
        pyrosim.End()

    def Generate_Body( self):
        pyrosim.Start_URDF("body" + str(self.myID) + ".urdf")

        # joint_torso_backleg_coordinates = [torso_coordinates[0]-0.5, 0, torso_coordinates[2]-(0.5*link_size[2])]
        # backLeg_coordinates = [link_size[0]*(-0.5),0,link_size[2]*(-0.5)]
        # joint_torso_frontleg_coordinates = [torso_coordinates[0]+0.5, 0, torso_coordinates[2]-(0.5*link_size[2])]
        # frontLeg_coordinates = [link_size[0]*0.5,0,link_size[2]*(-0.5)]

        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1] , size=[1,1,1]) # absolute

        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0,-0.5,1], jointAxis = "3 0 1")
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5, 0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0,0.5,1], jointAxis = "3 0 1")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5, 0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5,0,1], jointAxis = "3 0 2")
        pyrosim.Send_Cube(name="LeftLeg", pos=[1.5,0, 0] , size=[1,.2,0.2])

        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [-0.5,0,1], jointAxis = "3 0 2")
        pyrosim.Send_Cube(name="RightLeg", pos=[-0.5,0, 0] , size=[1,.2,0.2])

        pyrosim.Send_Joint( name = "FrontLeg_LowerFrontLeg" , parent= "FrontLeg" , child = "LowerFrontLeg" , type = "revolute", position = [0,1,0], jointAxis = "3 0 1")
        pyrosim.Send_Cube(name="LowerFrontLeg", pos=[0,0, -0.5] , size=[0.2,.2,1])

        pyrosim.Send_Joint( name = "BackLeg_LowerBackLeg" , parent= "BackLeg" , child = "LowerBackLeg" , type = "revolute", position = [0,-1,0], jointAxis = "3 0 1")
        pyrosim.Send_Cube(name="LowerBackLeg", pos=[0,0, -0.5] , size=[0.2,.2,1])

        pyrosim.Send_Joint( name = "LeftLeg_LowerLeftLeg" , parent= "LeftLeg" , child = "LowerLeftLeg" , type = "revolute", position = [-1,0,0], jointAxis = "3 0 2")
        pyrosim.Send_Cube(name="LowerLeftLeg", pos=[0,0, -0.5] , size=[0.2,.2,1])

        pyrosim.Send_Joint( name = "RightLeg_LowerRightLeg" , parent= "RightLeg" , child = "LowerRightLeg" , type = "revolute", position = [2,0,0], jointAxis = "3 0 2")
        pyrosim.Send_Cube(name="LowerRightLeg", pos=[0,0, -0.5] , size=[0.2,.2,1])
        

        pyrosim.End()
        

    def Generate_Mind(self):
        pyrosim.Start_NeuralNetwork("mind" + str(self.myID) + ".nndf")

# sensor neurons 
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "LeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "RightLeg")

# lower leg sensor neurons 
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "LowerFrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "LowerBackLeg")
        pyrosim.Send_Sensor_Neuron(name = 7  , linkName = "LowerLeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 8  , linkName = "LowerRightLeg")


# motor neurons 
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 5 , jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron( name = 6 , jointName = "Torso_RightLeg")
        

# lower leg motor neurons
        pyrosim.Send_Motor_Neuron( name = 7 , jointName = "FrontLeg_LowerFrontLeg")
        pyrosim.Send_Motor_Neuron( name = 8 , jointName = "BackLeg_LowerBackLeg")
        pyrosim.Send_Motor_Neuron( name = 9 , jointName = "LeftLeg_LowerLeftLeg")
        pyrosim.Send_Motor_Neuron( name = 10 , jointName = "RightLeg_LowerRightLeg")

        for currentRow in range(0,c.numSensorNeurons):
            for currentColumn in range(0,c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+3, weight = self.weights[currentRow][currentColumn])
        pyrosim.End()