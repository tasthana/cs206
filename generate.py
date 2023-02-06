import pyrosim.pyrosim as pyrosim
pyrosim.Start_SDF("boxes.sdf")
length = 1
width = 1
height = 1
verticals = 10
rows = 3
columns = 3
for i in range(rows):
	for j in range(columns):
		z = 0.5*height
		for k in range(0,verticals):
			l = pow(0.9, k)*length
			w = pow(0.9, k)*width
			h = pow(0.9, k)*height
			x = i*length
			y = j*width
			pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[l,w,h])
			z+=h
pyrosim.End()
