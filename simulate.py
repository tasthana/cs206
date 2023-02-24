
# #  import statements 
# import pybullet as p
# import pybullet_data
# import time 
# import pyrosim.pyrosim as pyrosim
# import numpy
# import os 
# import random 
# import constants as c

# # connecting simulator, base, and gravity 
# physicsClient = p.connect(p.GUI)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0,0,-9.8)
# #a = -.91
# #b = .91
# #x = numpy.linspace(0, numpy.pi*2, 1000) 

# # set constants for motor movement 1
# #amplitude = numpy.pi/4
# #frequency = 10
# #phaseOffset = 1

# # set constants for motor movement 2 
# #amplitude1 = numpy.pi/4
# #frequency1 = 10
# #phaseOffset1 = 7

 
# #equate = amplitude * sin(frequency * x + phaseOffset)
# targetAngles = numpy.array(numpy.pi/4 * numpy.sin(x))

# targetAngles = c.amplitude1 * numpy.sin(c.frequency1 * c.x + c.phaseOffset1)
# targetAngles2 = c.amplitude * numpy.sin(c.frequency * c.x + c.phaseOffset)
# #numpy.save(os.path.join("data", "targetAngles.npy"), targetAngles)
# #numpy.save(os.path.join("data", "targetAngles2.npy"), targetAngles2)
# #exit()

# planeId = p.loadURDF("plane.urdf")
# robotId = p.loadURDF("body.urdf")
# p.loadSDF("world.sdf")

# pyrosim.Prepare_To_Simulate(robotId)

# #backLegSensorValues = numpy.zeros(1000)
# #frontLegSensorValues = numpy.zeros(1000)

# for x in range(c.length):
#     time.sleep(1/240)
#     p.stepSimulation()
#     c.backLegSensorValues[x] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
#     c.frontLegSensorValues[x] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
#     print(c.backLegSensorValues)
#     print(c.frontLegSensorValues)
#     pyrosim.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Torso_BackLeg', controlMode = p.POSITION_CONTROL, targetPosition = targetAngles[x], maxForce = 25)
#     pyrosim.Set_Motor_For_Joint(bodyIndex = robotId, jointName = b'Torso_FrontLeg', controlMode = p.POSITION_CONTROL, targetPosition = targetAngles2[x], maxForce = 25)

# numpy.save(os.path.join("data", "BackLegSensorValues.npy"), backLegSensorValues)
# numpy.save(os.path.join("data", "FrontLegSensorValues.npy"), frontLegSensorValues)

# p.disconnect()

from simulation import SIMULATION
from motor import MOTOR
from sensor import SENSOR
import pybullet as p
import pybullet_data
import generate
import time 
import pyrosim.pyrosim as pyrosim
import numpy
import os 
import random 
import constants as c


simulation = SIMULATION()
simulation.Runs()
