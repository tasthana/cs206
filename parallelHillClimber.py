#parallelHC file
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

class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        os.system("rm tmp*.txt")
        os.system("rm body*.urdf")
        os.system("rm world*.sdf")
        self.parents = {}
        self.nextAvailableID = 0

        abmatrix = numpy.zeros((c.numberOfGenerations, c.populationSize))
        self.abmatrix = abmatrix

        for i in range(0, c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1


    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            # current generations = 2
            self.currentGeneration = currentGeneration
            self.Evolve_For_One_Generation()
     

    def Show_Best(self): 
        currentFitness = 10
        for key in self.parents.keys():
                if self.parents[key].fitness < currentFitness:
                    bestKey = key
                    currentFitness = self.parents[key].fitness
        print("This is the best fitness : ", self.parents[bestKey].fitness)            
        self.parents[bestKey].Start_Simulation("GUI")
        
        
    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.child)
        self.Select()
        self.Print()

    def Spawn(self):
        self.child ={}
        for key in range(len(self.parents)):
            self.child[key] =copy.deepcopy(self.parents[key])
            self.child[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for key in self.child:
            self.child[key].Mutate()

    def Evaluate(self, solutions):
        for key in solutions:
            solutions[key].Start_Simulation("DIRECT")
        for key in solutions:
            solutions[key].Wait_For_Simulation_To_End()

    def Select(self):
        for key in self.parents:
            if self.parents[key].fitness > self.child[key].fitness:
                self.parents[key] = self.child[key]

    def Print(self):
        for key in self.parents:
            self.abmatrix[key, self.currentGeneration] = self.parents[key].fitness
            # print("Parent:", self.parents[key].fitness, "Child:", self.child[key].fitness)


    def Matrix(self):
        print(self.abmatrix)
        numpy.savetxt("Fitness_Values_Octo", self.abmatrix, delimiter = ',' , fmt='%1.4e')
        numpy.save("Fitness_Values_Octo", self.abmatrix, delimiter = ',' , fmt='%1.4e')

    


