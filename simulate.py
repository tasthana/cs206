

import pybullet as p
import pybullet_data
import time 

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
#robotId = p.loadURDF("body.urdf")

p.loadSDF("boxes.sdf")

for x in range(0, 2001):
    time.sleep(.001)
    print(x)
p.disconnect()

