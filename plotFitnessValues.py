
import numpy
import matplotlib.pyplot as plot
import os 

### 5 generations 
quad_fitness_1 = []
quad_fitness_2 = []
quad_fitness_3 = []

## 10 generations 
hexa_fitness_1 = []
hexa_fitness_2 = []
hexa_fitness_3 = []

## 15 generations 
octo_fitness_1 = []
octo_fitness_2 = []
octo_fitness_3 = []

## 5 generations 
quad_fitness_1= numpy.load("Fitness_Values_Quad_1.npy") 
hexa_fitness_1 = numpy.load("Fitness_Values_Hexa_1.npy")
octo_fitness_1 = numpy.load("Fitness_Values_Octo_1.npy")

## 10 generations 
quad_fitness_2 = numpy.load("Fitness_Values_Quad_2.npy")
hexa_fitness_2 = numpy.load("Fitness_Values_Hexa_2.npy")
octo_fitness_3 = numpy.load("Fitness_Values_Octo_2.npy")

# 15 generations
quad_fitness_3 = numpy.load("Fitness_Values_Quad_3.npy")
hexa_fitness_3 = numpy.load("Fitness_Values_Hexa_3.npy")
octo_fitness_3= numpy.load("Fitness_Values_Octo_3.npy")

print(quad_fitness_1)
print(hexa_fitness_1)
print(octo_fitness_1)

print(quad_fitness_2)
print(hexa_fitness_2)
print(octo_fitness_2)

print(quad_fitness_3)
print(hexa_fitness_3)
print(octo_fitness_3)

# # plot.plot(numpy.arange(len(hexa_fitness)), hexa_fitness)

plot.plot(quad_fitness_1[: ,0], label='Quad Robot Fitness')
plot.plot(hexa_fitness_1[: ,0], label='Hexa Robot Fitness')
plot.plot(octo_fitness_1[: ,0], label='Octo Robot Fitness')
plot.xlabel("Number of Generations")
plot.ylabel("Fitness Range")
plot.title("Quad Robot vs Hexa Robot vs Octo Robot - 5 Num Generations")
plot.legend()
plot.show()

plot.plot(quad_fitness_2[: ,0], label='Quad Robot Fitness')
plot.plot(hexa_fitness_2[: ,0], label='Hexa Robot Fitness')
plot.plot(octo_fitness_3[: ,0], label='Octo Robot Fitness')
plot.xlabel("Number of Generations")
plot.ylabel("Fitness Range")
plot.title("Quad Robot vs Hexa Robot vs Octo Robot - 10 Num Generations")
plot.legend()
plot.show()

plot.plot(quad_fitness_3[: ,0], label='Quad Robot Fitness')
plot.plot(hexa_fitness_3[: ,0], label='Hexa Robot Fitness')
plot.plot(octo_fitness_3[: ,0], label='Octo Robot Fitness')
plot.xlabel("Number of Generations")
plot.ylabel("Fitness Range")
plot.title("Quad Robot vs Hexa Robot vs Octo Robot - 15 Num Generations")
plot.legend()
plot.show()

plot.plot(quad_fitness_1[: ,0], label='Quad Robot Fitness')
plot.plot(quad_fitness_2[: ,0], label='Quad Robot Fitness')
plot.plot(quad_fitness_3[: ,0], label='Quad Robot Fitness')
plot.xlabel("Number of Generations")
plot.ylabel("Fitness Range")
plot.title("Quad Robot Indiviual Comparisons")
plot.legend()
plot.show()


plot.plot(hexa_fitness_1[: ,0], label='Hexa Robot Fitness')
plot.plot(hexa_fitness_2[: ,0], label='Hexa Robot Fitness')
plot.plot(hexa_fitness_3[: ,0], label='Hexa Robot Fitness')
plot.xlabel("Number of Generations")
plot.ylabel("Fitness Range")
plot.title("Hexa Robot Indiviual Comparisons")
plot.legend()
plot.show()

plot.plot(hexa_fitness_1[: ,0], label='Octo Robot Fitness')
plot.plot(hexa_fitness_2[: ,0], label='Octo Robot Fitness')
plot.plot(hexa_fitness_3[: ,0], label='Octo Robot Fitness')
plot.xlabel("Number of Generations")
plot.ylabel("Fitness Range")
plot.title("Octo Robot Indiviual Comparisons")
plot.legend()
plot.show()