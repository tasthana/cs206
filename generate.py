
import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

length = 1
width = 1
height = 1

x = 0
y = 0
z = .25


for z in range(1,11):
  length = length *.9
  width = width *.9
  height = height *.9
  pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length, width, height])
  for x in range(1,6):
    pyrosim.Send_Cube(name="Box", pos=[x+1,y,z] , size=[length, width, height])
    for y in range(1,6):
      pyrosim.Send_Cube(name="Box", pos=[x,y+1,z] , size=[length, width, height])
pyrosim.End()


