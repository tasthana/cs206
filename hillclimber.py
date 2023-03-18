#hillclimber file
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
from solution import SOLUTION 
import copy

class HILL_CLIMBER:

    def __init__(self):
        self.parent = SOLUTION()

    def Evolve(self):
        self.parent.Evaluate("GUI")
        for currentGeneration in range(c.numberOfGenerations):
            # current generations = 2
            self.Evolve_For_One_Generation()

    def Show_Best(self): 
        self.parent.Evaluate("GUI")
        
    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate("DIRECT")
        self.Select()
        self.Print()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)
    
    def Mutate(self):
        self.child.Mutate()
        print(self.child.fitness)
        print(self.parent.fitness)

    def Select(self):
        if (self.parent.fitness < self.child.fitness):
            self.parent = self.child 

    def Print(self):
        print("\nparent fitness: " + str(self.parent.fitness) + "\nchild fitness: " + str(self.child.fitness))