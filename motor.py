 
 

import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random 
import constants as c


class MOTOR:

    def __init__(self, jointName):

        self.jointName = jointName
        self.Prepare_to_Act()

    def Prepare_to_Act(self):
        self.amplitude = c.amplitude
        self.offset = c.phaseOffset

        if("Back" in str(self.jointName)):
            self.frequency = c.frequency
            print("1 -------")

        else:
            print("2 -------")
            self.frequency = c.frequency * 2
        
        a = numpy.linspace(0, 2*numpy.pi, c.length)
        self.targetAngles = numpy.array(self.amplitude * numpy.sin(self.frequency * a * self.offset))


    def Set_Value(self, t, ID):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex= ID,
            jointName= self.jointName,
            controlMode= p.POSITION_CONTROL,
            targetPosition= self.targetAngles[t],
            maxForce= 25)

    def Save_Values(self):
        pass
        
        
