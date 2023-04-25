
import numpy
import matplotlib.pyplot as plot
import os 

octo_fitness = []
hexa_fitness = []
hexa_fitness2 = []
octo_fitness2 = []

octo_fitness = numpy.load("Fitness_Values_Octo.npy")
octo_fitness2 = numpy.load("Fitness_Values_Octo2.npy")

hexa_fitness = numpy.load("Fitness_Values_Hexa.npy")
hexa_fitness2 = numpy.load("Fitness_Values_Hexa2.npy")

# print(octo_fitness)
# print(hexa_fitness)
print(hexa_fitness2)
print(octo_fitness2)

# plot.plot(numpy.arange(len(hexa_fitness)), hexa_fitness)

plot.plot(hexa_fitness[: ,0], label='Hexa Robot Fitness')
plot.plot(octo_fitness[: ,0], label='Octo Robot Fitness')
plot.xlabel("Number of Generations")
plot.ylabel("Fitness Range")
plot.title("Hexa Robot vs Octo Robot - 5 Num Generations")
plot.legend()
plot.show()



plot.plot(hexa_fitness2[: ,0], label='Hexa Robot Fitness')
plot.plot(octo_fitness2[: ,0], label='Octo Robot Fitness')
plot.xlabel("Number of Generations")
plot.ylabel("Fitness Range")
plot.title("Hexa Robot vs Octo Robot - 10 Num Generations")
plot.legend()
plot.show()