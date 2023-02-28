 
 

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
        

    def Set_Value(self, desiredAngle, ID):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex= ID,
            jointName= self.jointName,
            controlMode= p.POSITION_CONTROL,
            targetPosition= desiredAngle,
            maxForce= 25)

        
        
