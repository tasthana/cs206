

import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random as random
import constants as c



class SENSOR:

    def __init__(self, linkName):
        self.linkName = linkName
        self.values = numpy.zeros(c.length)


    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        # print(self.values)

    def Save_Values(self):
        pass
        # numpy.save(os.path.join("data", "backLegSensorValues.npy"), backLegSensorValues)
        # numpy.save(os.path.join("data", "frontLegSensorValues.npy"), frontLegSensorValues)
