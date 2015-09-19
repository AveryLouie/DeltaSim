#A simple script to simulate a parallel delta robot

from visual import *
from math import cos, sin, pi, radians, sqrt, acos, ceil
import geometry as geo

#for now this is a delta that is simple and equilateral
class Delta:

	def __init__(self, scene, abase, bbase, cbase, adir, bdir, cdir, apos, bpos, cpos, alen, blen, clen, radius):
		self.scene = scene
		self.axes=['a','b','c']
		self.bases=   {'a':abase , 'b':bbase , 'c':cbase}
		self.dirs =   {'a':adir  , 'b':bdir  , 'c':cdir}
		self.tpos =   {'a':apos  , 'b':bpos  , 'c':cpos}
		self.armlen = {'a':alen  , 'b':blen  , 'c':clen}

		#creates self.towers, a dict that contains the tower objects
		self.tower_setup(self.bases, self.dirs, radius)
		#creates self.endpoint, which /is/ the endpoint object (not a dict)
		self.endpoint_setup(radius)
		#creates self.arm which is a dict that contains the arm objects
		self.arm_setup(radius)
		# self.visible()
		self.focus_delta()

		#plan_buf holds xyz positions, motion_buf holds abc positions
		self.plan_buf=[]
		self.motion_buf=[]

	#-----------------------------------SETUP FUNCTIONS------------------------------------------
	#sets up the delta.

	#creates an endpoint based on the tower positions.
	def endpoint_setup(self, rad):
		endpointpos = geo.get_sphere_intersect(self.vect_to_slider('a'), self.vect_to_slider('b'), self.vect_to_slider('c'),self.armlen['a'],self.armlen['b'],self.armlen['c'])
		self.endpoint = sphere (pos=endpointpos[0], radius = rad*.05)

	#cretes a dict of the tower cylinder objects
	def tower_setup(self, bases, dirs, rad):
		self.towers={}
		for axis in self.axes:
			self.towers[axis] = cylinder(pos=bases[axis], axis=dirs[axis], radius= rad*.1)
		return True
	
	#creates a dict of the arm cylinder objects
	def arm_setup(self, rad):
		self.arm={}
		for axis in self.axes:
			self.arm[axis] = cylinder(pos = self.vect_to_slider(axis), axis= self.endpoint.pos-self.vect_to_slider(axis), radius = rad*.01)

	#-----------------------------------KINEMATIC FUNCTIONS------------------------------------------
	#main delta kinematic functions are here

	#gives the vector that points from origin to the position of the slider along the tower
	#this uses the tpos tower positions- not any arbitrary positions
	def vect_to_slider(self, tower):
		return pos_to_vect(self, tower, self.tpos[tower])

	#changes from tower position to a vector the the tower position
	def pos_to_vect(self,tower,pos):
		return self.tower[tower].pos+pos*norm(self.towers[tower].axis)

	#rev kinematic takes a desired endpoint position and returns the tower positions that result in that endpoint 
	def rev_kinematic(self, R):
		#R, the desired point
		s={}
		for axis in axes:
			s[axis] = geo.get_sphere_line_intersect(R,self.armlen[axis],self.tower[axis].pos,self.tower[axis].axis)

		for axis in axes:
			s[axis] = geo.get_line_scalar(s[axis], self.tower[axis].pos, self.tower[axis].axis)

		# s1 = geo.solve_sphere_line_intersect(R,self.armlen,self.pylonA.pos,self.pylonA.axis)
		# s2 = geo.solve_sphere_line_intersect(R,r2,self.pylonB.pos,self.pylonB.axis)
		# s3 = geo.solve_sphere_line_intersect(R,r3,self.pylonC.pos,self.pylonC.axis)

		# s1 = self.point_line_scalar(s1, self.pylonA.pos, self.pylonA.axis)
		# s2 = self.point_line_scalar(s2, self.pylonB.pos, self.pylonB.axis)
		# s3 = self.point_line_scalar(s3, self.pylonC.pos, self.pylonC.axis)

		return s

		#takes s, a dict of delta positions s={'a':posa,'b':posb,'c':posc}
		#tries out the positions, then if they are valid, changes the self.tpos and self.arm values
		#to reflect upated positions
	def update_slider(self, s, mode):
		newpos={}

		#relative mode, add the new slider movement to the old slider position
		if mode=='rel':
			for axis in axes:
				newpos[axis]+=s[axis]+self.tpos[axis]

		#absolute mode, set the new slider position to whatever we got sent in s
		elif mode=='abs':
			for axis in axes:
				newpos[axis]=s[axis]
		
		else:
			print("valid modes for update_slider are 'rel' and 'abs'")

		#get the slider positions.  The movements are for the position of the slider /along the axis/ of the delta,
		#which may be at an angle or in a weird spot

		# slider1 = self.pylonA.pos + newa * norm(self.pylonA.axis)
		# slider2 = self.pylonB.pos + newb * norm(self.pylonB.axis)
		# slider3 = self.pylonC.pos + newc * norm(self.pylonC.axis)

		# res = geo.get_sphere_intersect(slider1, slider2, slider3, self.arm['a'], self.arm['b'], self.arm['c'])
		res = geo.get_shpere_intersect(self.pos_to_vect('a',s['a']),self.pos_to_vect('b',s['b']),self.pos_to_vect('c',s['c']),self.armlen['a'],self.armlen['b'],self.armlen['c'])
		
		#if the move is actually valid, update the "official" slider positions
		if (res is not False):
			self.endpoint.pos = res[0]

			for axis in axes:
				self.tpos[axis]=s[axis]
				self.arm[axis].pos=self.pos_to_vect(axis,s[axis])
				self.arm[axis].axis=res-self.arm[axis].pos

			# self.slider['a']=newa
			# self.slider['b']=newb
			# self.slider['c']=newc

			#update the endpoint
			# self.endpoint.pos = res

			# #update the arms too!
			# self.armA.pos = slider1
			# self.armB.pos = slider2
			# self.armC.pos = slider3

			# self.armA.axis = res-slider1
			# self.armB.axis = res-slider2
			# self.armC.axis = res-slider3

		else:
			print("the position you want to move to is not reachable.  holding still")

	#-----------------------------------UTILITY FUNCTIONS------------------------------------------
	#these are utility functions for doing things like making the delta visible, changing the scene,
	#and making the trail visible

	def endpoint_trail(self, state=True, col = color.orange):
		self.endpoint.make_trail = state
		self.endpoint.trail_object.color=col

		#TODO: move the scene to look at the delta- not sure the best way to do this yet.  hardcode for now
	def focus_delta(self):
		# scene.title="delta simulator 9000"
		# scene.center=(0,0,600*.5)
		# scene.forward=(-1,0,0)
		# scene.up     =(0,0,1)
		return True

	#TODO :need vpython ref to set all of the parts to invisible.
	def visible(self, state):
		pass

	#-----------------------------------SIMULATION FUNCTIONS------------------------------------------
	#these are functions for doing things like making moving the delta, and making paths

	



abase= vector(100*cos(radians(0)),100*sin(radians(0)),0)
bbase= vector(100*cos(radians(120)),100*sin(radians(120)),0)
cbase= vector(100*cos(radians(240)),100*sin(radians(240)),0)
adir = vector(0,0,300)
bdir = vector(0,0,300)
cdir = vector(0,0,300)
apos = 200
bpos = 200
cpos = 200
alen = 300
blen = 300
clen = 300
radius = 100


mydelta = Delta(scene, abase, bbase, cbase, adir, bdir, cdir, apos, bpos, cpos, alen, blen, clen, radius)

	# def xyz_to_abc(self):
	# 	for xyz in self.plan_buf:
	# 		print self.rev_kinematic(xyz,self.arm['a'],self.arm['b'],self.arm['c'])
	# 		self.motion_buf.append(self.rev_kinematic(xyz,self.arm['a'],self.arm['b'],self.arm['c']))
	# 	self.plan_buf=[] #forget what was in this buffer

	# def run_out_motion_buffer(self):
	# 	for pos in self.motion_buf:
	# 		rate(100)
	# 		self.update_slider(pos[0],pos[1],pos[2],'abs')
	# 	self.motion_buf=[]

	# 	#load a motion buffer into the machines motion buffer
	# def load_motion_buffer(self,buf):
	# 	self.motion_buf=buf
	# 	return True

	# def col_visible(self, state=True):
	# 	self.pylonA.visible=state
	# 	self.pylonB.visible=state
	# 	self.pylonC.visible=state