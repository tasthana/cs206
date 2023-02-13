

import pybullet as p
import pybullet_data
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random 

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
a = -.91
b = .91

targetAngles = numpy.linspace(0, 2*numpy.pi, num= 1000)
numpy.save(os.path.join("data", "targetAngles.npy"), targetAngles)
exit()

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

backLegSensorValues = numpy.zeros(1000)
frontLegSensorValues = numpy.zeros(1000)
for x in range(0, 1000):
    time.sleep(.001)
    p.stepSimulation()
    backLegSensorValues[x] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[x] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    print(backLegSensorValues)
    print(frontLegSensorValues)
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = random.uniform(a,b), maxForce = 500)
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = random.uniform(a,b), maxForce = 500)

numpy.save(os.path.join("data", "BackLegSensorValues.npy"), backLegSensorValues)
numpy.save(os.path.join("data", "FrontLegSensorValues.npy"), frontLegSensorValues)

p.disconnect()


