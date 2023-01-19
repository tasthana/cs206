

import pybullet as p
import time 

physicsClient = p.connect(p.GUI)

for x in range(0, 1001):
    time.sleep(.001)
    p.stepSimulation()
    print(x)
p.disconnect()

