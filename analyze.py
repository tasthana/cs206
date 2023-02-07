



import numpy
import matplotlib.pyplot as plot
import os 

# load 
backLegSensorValues = numpy.load(os.path.join("data", "BackLegSensorValues.npy"))
frontLegSensorValues = numpy.load(os.path.join("data", "FrontLegSensorValues.npy"))

# print values 
print(backLegSensorValues)
print(frontLegSensorValues)

# plotting 
plot.plot(backLegSensorValues, label='Back Leg', linewidth=3.5)
plot.plot(frontLegSensorValues,  label='Front Leg')


plot.legend()
plot.show()


