#A simple script to simulate a parallel delta robot

from visual import *
from math import cos, sin, pi, radians, sqrt

#for now this is a delta that is simple and equilateral
class delta:

	def __init__(self,radius,height,arm_length):
		self.radius = radius
		self.height = height
		self.arm    = arm
		self.sliders={x : 0, y : 0, z : 0}

	def draw_delta(self):



#some variables to do with the delta in question
pylon_height = 600  #height of the pylons
pylon_radius = 200  #radius that the pylons sit on
arm_length   = 250  #length of the delta arms

#handy aliases
origin = vector (0,0,0)

#set the scene, set the delta frame
scene.title="delta simulator 9000"
scene.center=(0,0,pylon_height/2.0)

#delta frame stuff
base = cylinder(pos = origin, axis=(0,0,-1), radius = pylon_radius, color = color.red, opacity = .2)
pylonA = cylinder(pos = origin, axis = (0,0,0), radius = 0, color = color.red)
pylonB = cylinder(pos = origin, axis = (0,0,0), radius = 0, color = color.red)
pylonC = cylinder(pos = origin, axis = (0,0,0), radius = 0, color = color.red)

#figure out the positions of the bases of the pylons.
#these are stored outside of the axis/origin because they will be useful later on for sim stuff

pylonA.x     = cos(radians(0))*pylon_radius
pylonA.y     = sin(radians(0))*pylon_radius
pylonA.point = vector (0,0,pylon_height)


pylonB.x     = cos(radians(120))*pylon_radius
pylonB.y     = sin(radians(120))*pylon_radius
pylonB.point = vector (0,0,pylon_height)

pylonC.x     = cos(radians(240))*pylon_radius
pylonC.y     = sin(radians(240))*pylon_radius
pylonC.point = vector (0,0,pylon_height)

#actually assign these to the pylons

pylonA.pos    = vector (pylonA.x,pylonA.y,0)
pylonA.axis   = pylonA.point
pylonA.radius = pylon_radius*.1

pylonB.pos    = vector (pylonB.x,pylonB.y,0)
pylonB.axis   = pylonB.point
pylonB.radius = pylon_radius*.1

pylonC.pos    = vector (pylonC.x,pylonC.y,0)
pylonC.axis   = pylonC.point
pylonC.radius = pylon_radius*.1

#set the location of the sliders
sliderA=vector(pylonA.x,pylonA.y,500)
sliderB=vector(pylonB.x,pylonB.y,500)
sliderC=vector(pylonC.x,pylonC.y,500)

#ok, sim stuff

def solve_sphere_intersect(P1, P2, P3, r):
	#step 1, create a coordinate system around S1 = (0,0,0), taking the vector from center S1->S2 to be one axis
	ehatx = ( P2 - P1 )/ mag(P2 - P1)
	
	i     = dot(ehatx, (P3-P1))

	ehaty = (P3 - P1 - i * ehatx)/mag(P3-P1-i*ehatx)
	
	ehatz = cross(ehatx, ehaty)
	
	d = mag(P2-P1)

	j = dot( ehaty, (P3-P1))

	#NB: the following x y z are only true iff the radius of all spheres is the same

	x = (d**2)/(2.0*d)

	y = ((i**2 + j**2) / (2.0 * j)) - (i * x / j)

	z = - sqrt ( r**2 - x**2 - y**2 )

	res = P1 + x*ehatx + y* ehaty + z*ehatz #there is another solution, where you use +z

	print(res)

	return res

fin=solve_sphere_intersect(sliderA,sliderB,sliderC,arm_length)

#draw the spheres for now
Asphere = sphere(pos = sliderA, radius= arm_length, opacity = .1)
Bsphere = sphere(pos = sliderB, radius= arm_length, opacity = .1)
Csphere = sphere(pos = sliderC, radius= arm_length, opacity = .1)

Dpoint  = sphere(pos = fin, radius = pylon_radius*.05, color = color.green)

pos=0

while pos<200:
	rate(20)
	pos=pos+1
	sliderA=sliderA-(0,0,pos*.3)
	sliderB=sliderB-(0,0,pos*.5)
	sliderC=sliderC-(0,0,pos)
	fin=solve_sphere_intersect(sliderA,sliderB,sliderC,arm_length)

	#draw the spheres for now
	Asphere.pos=sliderA
	Bsphere.pos=sliderB
	Csphere.pos=sliderC

	Dpoint  = sphere(pos = fin, radius = pylon_radius*.05, color = color.green)	

