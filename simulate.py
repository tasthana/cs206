

import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

backLegSensorValues = numpy.zeros(100)
frontLegSensorValues = numpy.zeros(100)
for x in range(0, 100):
    time.sleep(.001)
    p.stepSimulation()
    backLegSensorValues[x] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[x] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    print(backLegSensorValues)
    print(frontLegSensorValues)
numpy.save(os.path.join("data", "BackLegSensorValues.npy"), backLegSensorValues)
numpy.save(os.path.join("data", "FrontLegSensorValues.npy"), frontLegSensorValues)
p.disconnect()


