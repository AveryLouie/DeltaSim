#A simple script to simulate a parallel delta robot

from visual import *
from math import cos, sin, pi, radians, sqrt, acos, ceil

#for now this is a delta that is simple and equilateral
class Delta:

	def __init__(self,radius,height,arm_length,scene):
		self.radius = radius
		self.height = height
		self.seg    = 1.0 #max segment length for interpolation
		self.plan_buf = [] #the buffer that holds xyz interp points
		self.motion_buf = [] #the buffer that holds abc interp points
		self.slider={'a' : 0, 'b' : 0, 'c' : 0} # the position of the slider along the pylon
		self.arm={'a': arm_length,'b': arm_length, 'c': arm_length}
		self.origin = vector (0,0,0)

		#
		scene.title="delta simulator 9000"
		scene.center=(0,0,self.height/2.0)

	#Trilateration based on https://en.wikipedia.org/wiki/Trilateration
	#step 1 change to coordinate system w spheres all in one plane
	#step 2 find solution to sphere intersect in new CS
	#step 3 use unit vectors from step 1 to transform to solution in original
	def solve_sphere_intersect(self, P1, P2, P3, r1, r2, r3 ):
		#step 1, create a coordinate system around S1 = (0,0,0), taking the vector from center S1->S2 to be one axis
		ehatx = ( P2 - P1 )/ mag(P2 - P1)
		i     = dot(ehatx, (P3-P1))
		ehaty = (P3 - P1 - i * ehatx)/mag(P3-P1-i*ehatx)
		ehatz = cross(ehatx, ehaty)
		d = mag(P2-P1)
		j = dot( ehaty, (P3-P1))
		#NB: the following x y z are only true iff the radius of all spheres is the same
		x = (r1**2 - r2**2 + d**2)/(2.0*d)
		y = ((r1**2 - r3**2 + i**2 + j**2) / (2.0 * j)) - (i * x / j)

		try:
			z = - sqrt ( r1**2 - x**2 - y**2 )
		except ValueError:
			print ("position not reachable! Spheres do not intersect.")
			return False

		res = P1 + x*ehatx + y* ehaty + z*ehatz #there is another solution, where you use +z
		#print(res)
		return res

	#draws the delta for the first time and sets things up
	def setup_delta(self):
		#delta frame stuff
		self.base = cylinder(pos = self.origin, axis=(0,0,-1), radius = self.radius, color = color.orange, opacity = .2)
		
		#we dont assign these just yet- it is neater to do it seperately, but handy to have the cylinder object to store stuff in 
		self.pylonA = cylinder(pos = self.origin, axis = (0,0,0), radius = 0, color = color.orange)
		self.pylonB = cylinder(pos = self.origin, axis = (0,0,0), radius = 0, color = color.orange)
		self.pylonC = cylinder(pos = self.origin, axis = (0,0,0), radius = 0, color = color.orange)

		#ignore where these are initially
		self.armA = cylinder(radius = self.radius * .01, color = color.orange)
		self.armB = cylinder(radius = self.radius * .01, color = color.orange)
		self.armC = cylinder(radius = self.radius * .01, color = color.orange)

		#should clean this up
		self.pylonA.x     = cos(radians(0))*self.radius
		self.pylonA.y     = sin(radians(0))*self.radius
		self.pylonA.point = vector (0,0,self.height)

		self.pylonB.x     = cos(radians(120))*self.radius
		self.pylonB.y     = sin(radians(120))*self.radius
		self.pylonB.point = vector (0,0,self.height)

		self.pylonC.x     = cos(radians(240))*self.radius
		self.pylonC.y     = sin(radians(240))*self.radius
		self.pylonC.point = vector (0,0,self.height)

		#actually assign these to the pylons

		self.pylonA.pos    = vector (self.pylonA.x,self.pylonA.y,0)
		self.pylonA.axis   = self.pylonA.point
		self.pylonA.radius = self.radius*.1

		self.pylonB.pos    = vector (self.pylonB.x,self.pylonB.y,0)
		self.pylonB.axis   = self.pylonB.point
		self.pylonB.radius = self.radius*.1

		self.pylonC.pos    = vector (self.pylonC.x,self.pylonC.y,0)
		self.pylonC.axis   = self.pylonC.point
		self.pylonC.radius = self.radius*.1


		#need to set make_trail true so that the object gets a trail interval object, otherwise it is broken later
		self.endpoint =  sphere(pos = self.origin, radius = self.radius*.05, color = color.green, make_trail = True)	
		self.endpoint.make_trail = False #make trail invisible

	#tries to update slider position, checks to make sure there is a solution
	#if there is no solution it returns False, else returns the solution based on solve_sphere_intersect
	def update_slider(self, a, b, c, mode):
		#get the position in space based on the position along the pylon.
		#if the pylon is not vertical, this is not neccecarily
		if (mode=='rel'):
			newa=self.slider['a']+a
			newb=self.slider['b']+b
			newc=self.slider['c']+c
		elif (mode=='abs'):
			newa=a
			newb=b
			newc=c
		else:
			print("valid modes for update_slider are 'rel' and 'abs'")

		#get the slider positions.  The movements are for the position of the slider /along the axis/ of the delta,
		#which may be at an angle or in a weird spot

		slider1 = self.pylonA.pos + newa * norm(self.pylonA.axis)
		slider2 = self.pylonB.pos + newb * norm(self.pylonB.axis)
		slider3 = self.pylonC.pos + newc * norm(self.pylonC.axis)

		res = self.solve_sphere_intersect(slider1, slider2, slider3, self.arm['a'], self.arm['b'], self.arm['c'])
		
		#if the move is actually valid, update the "official" slider positions
		if (res is not False):
			self.slider['a']=newa
			self.slider['b']=newb
			self.slider['c']=newc

			#update the endpoint
			self.endpoint.pos = res

			#update the arms too!
			self.armA.pos = slider1
			self.armB.pos = slider2
			self.armC.pos = slider3

			self.armA.axis = res-slider1
			self.armB.axis = res-slider2
			self.armC.axis = res-slider3

		else:
			print("the position you want to move to is not reachable.  holding still")
			return False

	#set the initial position of the sliders.  Possible to make an usafe setting!
	def slider_set(self, a, b, c):
		self.slider['a'] = a
		self.slider['b'] = b
		self.slider['c'] = c

	def arm_len_set(self, a, b, c):
		self.arm['a']=a
		self.arm['b']=b
		self.arm['c']=c

	#get desired slider-on-pylon position for x y z coords
	def rev_kinematic(self, R, r1, r2, r3):
		#R, the desired point
		s1 = self.solve_sphere_line_intersect(R,r1,self.pylonA.pos,self.pylonA.axis)
		s2 = self.solve_sphere_line_intersect(R,r2,self.pylonB.pos,self.pylonB.axis)
		s3 = self.solve_sphere_line_intersect(R,r3,self.pylonC.pos,self.pylonC.axis)

		s1 = self.point_line_scalar(s1, self.pylonA.pos, self.pylonA.axis)
		s2 = self.point_line_scalar(s2, self.pylonB.pos, self.pylonB.axis)
		s3 = self.point_line_scalar(s3, self.pylonC.pos, self.pylonC.axis)


		return [s1,s2,s3]

	#find intersection of circle and a line.
	#circle given in x,y,z,r
	#line given in point-vector form (but not neccecarily unit vector)


	#given a vector that is pointing at a point, a point on a line and the unit vector for the line,
	#what value t for line(t)=linepoint+t*norm(lineaxis) gives you the intersection?
	def point_line_scalar(self, point, linepoint, lineaxis):
		ret=mag(point-linepoint)#/mag(norm(lineaxis))==1
		return ret



	def solve_sphere_line_intersect(this,R,r,P,pvec):
		yhat=norm(pvec)
		xhat=norm(cross(cross(R-P,pvec),pvec))

		#line-point distance d
		d   = dot(P-R,xhat)/mag(xhat)

		#if d>r, there is no way the two intersect, and acos will return ValueError as range of acos is [1:-1]
		if(d>r):
			print("cannot solve for arm position")
			return False

		theta = acos(d/r)
		#note,there is more than one solution.  I only return one because I assume the arms are going "down"
		#this is the solution if you want a vector pointed at the place in space where the arm is
		solution = R+ r*cos(theta)*xhat +r*sin(theta)*yhat

		return solution

	def endpoint_trail(self):
		self.endpoint.make_trail = True
		self.endpoint.trail_type = "points"

	#you give plan_G0 B, as in from point A to B
	#puts these points in self.plan_buf
	def plan_G0(self,B):
		A=self.endpoint.pos
		Mhat=norm(A-B) #this is the motion vector

		#largest_seg is the largest segment length that evenly divides the total distance that is smaller than self.seg
		
		largest_seg = mag(A-B)/ceil(mag(A-B)/self.seg)
		print("seg len is %d" % (largest_seg))


		for i in range(int((mag(A-B)/largest_seg)+1)):
			self.plan_buf.append(A-i*largest_seg*Mhat)
		return 1
		
	def xyz_to_abc(self):
		for xyz in self.plan_buf:
			print self.rev_kinematic(xyz,self.arm['a'],self.arm['b'],self.arm['c'])
			self.motion_buf.append(self.rev_kinematic(xyz,self.arm['a'],self.arm['b'],self.arm['c']))
		self.plan_buf=[] #forget what was in this buffer

	def run_out_motion_buffer(self):
		for pos in self.motion_buf:
			rate(100)
			myDelta.update_slider(pos[0],pos[1],pos[2],'abs')
		self.motion_buf=[]




#draw the spheres for now
#Asphere = sphere(pos = sliderA, radius= arm_length, opacity = .1)
#Bsphere = sphere(pos = sliderB, radius= arm_length, opacity = .1)
#Csphere = sphere(pos = sliderC, radius= arm_length, opacity = .1)

myDelta = Delta(200, 600, 250, scene)
myDelta.setup_delta()
myDelta.arm_len_set(400, 400, 400)
myDelta.update_slider(500,500,500,'abs')
myDelta.endpoint_trail()

myDelta.plan_G0(vector(0,0,0))
myDelta.xyz_to_abc()
myDelta.run_out_motion_buffer()
myDelta.plan_G0(vector(-50,-50,0))
myDelta.xyz_to_abc()
myDelta.run_out_motion_buffer()
myDelta.plan_G0(vector(-50,50,0))
myDelta.xyz_to_abc()
myDelta.run_out_motion_buffer()
myDelta.plan_G0(vector(50,50,0))
myDelta.xyz_to_abc()
myDelta.run_out_motion_buffer()
myDelta.plan_G0(vector(50,-50,0))
myDelta.xyz_to_abc()
myDelta.run_out_motion_buffer()
