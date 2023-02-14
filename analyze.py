



import numpy
import matplotlib.pyplot as plot
import os 

# load 
#backLegSensorValues = numpy.load(os.path.join("data", "BackLegSensorValues.npy"))
#frontLegSensorValues = numpy.load(os.path.join("data", "FrontLegSensorValues.npy"))


targetAngleValues = numpy.load(os.path.join("data", "targetAngles.npy"))
targetAngleValues2 = numpy.load(os.path.join("data", "targetAngles2.npy"))
# print values 
#print(backLegSensorValues)
#print(frontLegSensorValues)
#print(targetAngleValues)

# plotting 
#plot.plot(backLegSensorValues, label='Back Leg', linewidth=3.5)
#plot.plot(frontLegSensorValues,  label='Front Leg')
plot.plot(numpy.arange(len(targetAngleValues)), targetAngleValues)
plot.plot(numpy.arange(len(targetAngleValues2)), targetAngleValues2)

plot.legend()
plot.show()

