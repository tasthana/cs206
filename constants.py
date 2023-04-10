

# file to hold all constants in simulate.py 


#  import statements 
import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random 

# constants 

a = -.91
b = .91
x = numpy.linspace(0, numpy.pi*2, 1000)


# set constants for motor movement 1
amplitude = numpy.pi/4
frequency = 10
phaseOffset = 1

# set constants for motor movement 2
amplitude1 = numpy.pi/4
frequency1 = 10
phaseOffset1 = 7

# set constants for sensor values array 
backLegSensorValues = numpy.zeros(1000)
frontLegSensorValues = numpy.zeros(1000)

# loop length
length = 1001

# hillclimber constant 
numberOfGenerations = 3

#parallel hillclimber constant 
populationSize = 5
numberOfGenerations = 5

#quadruped constants 
numSensorNeurons = 12
numMotorNeurons = 14

motorJointRange = .25







