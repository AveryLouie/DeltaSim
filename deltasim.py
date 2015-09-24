#A simple script to simulate a parallel delta robot

from visual import *
from math import cos, sin, pi, radians, sqrt, acos, ceil
import geometry as geo

#for now this is a delta that is simple and equilateral
class Delta:

	def __init__(self, scene, abase, bbase, cbase, adir, bdir, cdir, apos, bpos, cpos, alen, blen, clen, radius):
		self.scene = scene
		self.seg = 1
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
		self.endpoint = sphere (pos=endpointpos[0], radius = rad*.05, make_trail = False , retain = 2000)

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
		return self.pos_to_vect(tower, self.tpos[tower])

	#changes from tower position to a vector the the tower position
	def pos_to_vect(self,tower,pos):
		return self.towers[tower].pos+pos*norm(self.towers[tower].axis)

	#rev kinematic takes a desired endpoint position and returns the tower positions that result in that endpoint 
	def rev_kinematic(self, R):
		#R, the desired point
		s={}
		for axis in self.axes:
			s[axis] = geo.get_sphere_line_intersect(R,self.armlen[axis],self.towers[axis].pos,self.towers[axis].axis)

		for axis in self.axes:
			s[axis] = geo.get_line_scalar(s[axis][0], self.towers[axis].pos, self.towers[axis].axis)

		return s

		#takes s, a dict of delta positions s={'a':posa,'b':posb,'c':posc}
		#tries out the positions, then if they are valid, changes the self.tpos and self.arm values
		#to reflect upated positions
	def update_slider(self, s, mode):
		newpos={}
		#relative mode, add the new slider movement to the old slider position
		if mode=='rel':
			for axis in self.axes:
				newpos[axis]+=s[axis]+self.tpos[axis]

		#absolute mode, set the new slider position to whatever we got sent in s
		elif mode=='abs':
			for axis in self.axes:
				newpos[axis]=s[axis]
		
		else:
			print("valid modes for update_slider are 'rel' and 'abs'")

		res = geo.get_sphere_intersect(self.pos_to_vect('a',s['a']),self.pos_to_vect('b',s['b']),self.pos_to_vect('c',s['c']),self.armlen['a'],self.armlen['b'],self.armlen['c'])
		
		#if the move is actually valid, update the "official" slider positions
		if (res is not False):
			self.endpoint.pos = res[0]

			for axis in self.axes:
				self.tpos[axis]=s[axis]
				self.arm[axis].pos=self.pos_to_vect(axis,s[axis])
				self.arm[axis].axis=res[0]-self.arm[axis].pos

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

	def xyz_to_abc(self, cartesian_buf, delta_buf):
		#the carteian buf is vectors
		for xyz in cartesian_buf:
			delta_buf.append(self.rev_kinematic(xyz))#,self.armlen['a'],self.armlen['b'],self.armlen['c']))
		return delta_buf

	def run_out_motion_buffer(self):
		for pos in self.motion_buf:
			rate(1000)
			self.update_slider(pos,'abs')
		self.motion_buf=[]

	#
	def G0_to_buf(self,B):
		#if there is nothing in the buffer, start fresh with the last tool pos
		#if there is something in the buffer, then we use the last point in the buffer
		
		if self.plan_buf==[]:
			to_add = geo.plan_G0(self.endpoint.pos,B,self.seg)
		else:
			to_add = geo.plan_G0(self.plan_buf[len(self.plan_buf)-1],B,self.seg)

		for item in to_add:
			self.plan_buf.append(item)


		return True

	def plan_to_motion(self):
		self.xyz_to_abc(self.plan_buf,self.motion_buf)

	def G_file_to_plan(self,filename):
		big_list=geo.import_G_code(filename)
		for item in big_list:
			if item[0]=='G1':
				self.G0_to_buf(item[1])




abase= vector(300*cos(radians(0)),300*sin(radians(0)),0)
bbase= vector(300*cos(radians(120)),300*sin(radians(120)),0)
cbase= vector(300*cos(radians(240)),300*sin(radians(240)),0)
adir = vector(0,0,600)
bdir = vector(0,0,600)
cdir = vector(0,0,600)
apos = 600
bpos = 600
cpos = 600
alen = 500
blen = 500
clen = 500
radius = 100


mydelta = Delta(scene, abase, bbase, cbase, adir, bdir, cdir, apos, bpos, cpos, alen, blen, clen, radius)

mydelta.endpoint_trail()
mydelta.G_file_to_plan('test.gcode')
mydelta.plan_to_motion()
mydelta.run_out_motion_buffer()
