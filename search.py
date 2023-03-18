
import os 
import hillclimber as HILL_CLIMBER
import pyrosim.pyrosim as pyrosim




# os.system("python3 generate.py")
# os.system("python3 simulate.py")

hc = HILL_CLIMBER.HILL_CLIMBER()

hc.Evolve()
hc.Show_Best()