
import os 
import parallelHillClimber as PARALLEL_HILL_CLIMBER
import pyrosim.pyrosim as pyrosim




# os.system("python3 generate.py")
# os.system("python3 simulate.py")

phc = PARALLEL_HILL_CLIMBER.PARALLEL_HILL_CLIMBER()

phc.Evolve()
phc.Show_Best()