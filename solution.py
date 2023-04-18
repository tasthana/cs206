

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

        
#back legs
        pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0,-.5,1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5, 0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "BackLeg_LowerBackLeg" , parent= "BackLeg" , child = "LowerBackLeg" , type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LowerBackLeg", pos=[0,0, -0.5] , size=[0.2,.2,1])

        pyrosim.Send_Joint( name = "Torso_BackLeg2" , parent= "Torso" , child = "BackLeg2" , type = "revolute", position = [0,-.5,1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeg2", pos=[0.5,-0.5, 0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "BackLeg2_LowerBackLeg2" , parent= "BackLeg2" , child = "LowerBackLeg2" , type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="LowerBackLeg2", pos=[0.5,0, -0.5] , size=[0.2,.2,1])


#front legs
        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0,0.5,1], jointAxis = "0 0 1") 
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5, 0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "FrontLeg_LowerFrontLeg" , parent= "FrontLeg" , child = "LowerFrontLeg" , type = "revolute", position = [0,1,0], jointAxis = "0 0 1")
        pyrosim.Send_Cube(name="LowerFrontLeg", pos=[0,0, -0.5] , size=[0.2,.2,1])

        pyrosim.Send_Joint( name = "Torso_FrontLeg2" , parent= "Torso" , child = "FrontLeg2" , type = "revolute", position = [0,0.5,1], jointAxis = "0 0 1") 
        pyrosim.Send_Cube(name="FrontLeg2", pos=[0.5,0.5, 0] , size=[0.2,1,0.2])

        pyrosim.Send_Joint( name = "FrontLeg2_LowerFrontLeg" , parent= "FrontLeg2" , child = "LowerFrontLeg2" , type = "revolute", position = [0,1,0], jointAxis = "0 0 1")
        pyrosim.Send_Cube(name="LowerFrontLeg2", pos=[0.5,0, -0.5] , size=[0.2,.2,1])


#left legs
        pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5,0,1], jointAxis = "1 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0, 0] , size=[1,.2,0.2])

        pyrosim.Send_Joint( name = "LeftLeg_LowerLeftLeg" , parent= "LeftLeg" , child = "LowerLeftLeg" , type = "revolute", position = [-1,0,0], jointAxis = "1 1 0")
        pyrosim.Send_Cube(name="LowerLeftLeg", pos=[0,0, -0.5] , size=[0.2,.2,1])

        pyrosim.Send_Joint( name = "Torso_LeftLeg2" , parent= "Torso" , child = "LeftLeg2" , type = "revolute", position = [-.5,0,1], jointAxis = "1 1 0")
        pyrosim.Send_Cube(name="LeftLeg2", pos=[-.5,.55, 0] , size=[1,.2,0.2])

        pyrosim.Send_Joint( name = "LeftLeg2_LowerLeftLeg2" , parent= "LeftLeg2" , child = "LowerLeftLeg2" , type = "revolute", position = [1,0,0], jointAxis = "1 1 0")
        pyrosim.Send_Cube(name="LowerLeftLeg2", pos=[-2,.55, -0.5] , size=[0.2,.2,1])


#right legs 
        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [.5,0,1], jointAxis = "0 1 1")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0, 0] , size=[1,.2,0.2])

        pyrosim.Send_Joint( name = "RightLeg_LowerRightLeg" , parent= "RightLeg" , child = "LowerRightLeg" , type = "revolute", position = [1,0,0], jointAxis = "0 1 1")
        pyrosim.Send_Cube(name="LowerRightLeg", pos=[0 ,0, -.5] , size=[0.2,.2,1])

        pyrosim.Send_Joint( name = "Torso_RightLeg2" , parent= "Torso" , child = "RightLeg2" , type = "revolute", position = [-.5,0,1], jointAxis = "0 1 1")
        pyrosim.Send_Cube(name="RightLeg2", pos=[1.5,.5, 0] , size=[1,.2,0.2])

        pyrosim.Send_Joint( name = "RightLeg_LowerRightLeg2" , parent= "RightLeg2" , child = "LowerRightLeg2" , type = "revolute", position = [1,0,0], jointAxis = "0 1 1")
        pyrosim.Send_Cube(name="LowerRightLeg2", pos=[1, .5, -.5] , size=[0.2,.2,1])


        pyrosim.End()
        

    def Generate_Mind(self):
        pyrosim.Start_NeuralNetwork("mind" + str(self.myID) + ".nndf")


 
# lower leg sensor neurons 
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "LowerFrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "LowerBackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2  , linkName = "LowerLeftLeg")
        pyrosim.Send_Sensor_Neuron(name = 3  , linkName = "LowerRightLeg")

        pyrosim.Send_Sensor_Neuron(name = 4  , linkName = "LowerBackLeg2")
        pyrosim.Send_Sensor_Neuron(name = 5  , linkName = "LowerFrontLeg2")
        pyrosim.Send_Sensor_Neuron(name = 6  , linkName = "LowerLeftLeg2")
        pyrosim.Send_Sensor_Neuron(name = 6  , linkName = "LowerRightLeg2")

        

# motor neurons 
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "BackLeg_LowerBackLeg")

        pyrosim.Send_Motor_Neuron( name = 5 , jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 6 , jointName = "FrontLeg_LowerFrontLeg")
        
        pyrosim.Send_Motor_Neuron( name = 7 , jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron( name = 8 , jointName = "LeftLeg_LowerLeftLeg")

        pyrosim.Send_Motor_Neuron( name = 9 , jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron( name = 10 , jointName = "RightLeg_LowerRightLeg")

        pyrosim.Send_Motor_Neuron( name = 11 , jointName = "Torso_BackLeg2")
        pyrosim.Send_Motor_Neuron( name = 12 , jointName = "BackLeg2_LowerBackLeg2")

        pyrosim.Send_Motor_Neuron( name = 13 , jointName = "Torse_FrontLeg2")
        pyrosim.Send_Motor_Neuron( name = 14 , jointName = "FrontLeg2_LowerFrontLeg")

        pyrosim.Send_Motor_Neuron( name = 14 , jointName = "Torso_LeftLeg2")
        pyrosim.Send_Motor_Neuron( name = 15 , jointName = "LeftLeg2_LowerLeftLeg2")

        pyrosim.Send_Motor_Neuron( name = 16 , jointName = "Torso_RightLeg2")
        pyrosim.Send_Motor_Neuron( name = 17 , jointName = "RightLeg_LowerRightLeg2")



        for currentRow in range(0,c.numSensorNeurons):
            for currentColumn in range(0,c.numMotorNeurons):
                pyrosim.Send_Synapse( sourceNeuronName = currentRow , targetNeuronName = currentColumn+3, weight = self.weights[currentRow][currentColumn])
        pyrosim.End()

