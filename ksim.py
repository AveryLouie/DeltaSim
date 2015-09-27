#kinematics.py is a package for exploring kinematicts
#the goal is to be able to show what various things mean visually,
#and to provide a tool for visualizing solutions to kinematic problems
#vectors and matricies are great but it is hard to visualize what you are doing

from visual import *

class triad():
	def __init__(self,**kwargs):
		self.origin = vector(0,0,0)
		self.axes = ['x','y','z']

		self.local_unit = {'x':vector(1,0,0),'y':vector(0,1,0),'z':vector(0,0,1)}
		self.global_unit = {'x':vector(1,0,0),'y':vector(0,1,0),'z':vector(0,0,1)}

		self.arrows={}
		for axis in self.axes:
			self.arrows[axis]=arrow(pos=self.origin,axis = self.global_unit[axis])

	def rotate_global(self,alpha,beta,gamma):
		rot_angle={'x':alpha, 'y':beta, 'z':gamma}
		for key in self.arrows.keys():
			for axis in self.axes:
				self.arrows[key].rotate(angle = radians(rot_angle [axis]), axis = self.global_unit[axis])

	def rotate_local(self,axis,rotation):
		turn = self.arrows[axis].axis
		for key in self.arrows.keys():
			self.arrows[key].rotate(angle = radians(rotation), axis = turn)


	def rotate_euler(self, phi, theta, psi, animate=0):
		if animate==0:
			self.rotate_local('z',phi)
			self.rotate_local('x',theta)
			self.rotate_local('z',psi)
		else:
			for i in range(animate):
				rate(10)
				self.rotate_local('z',float(phi)/animate)
			for i in range(animate):
				rate(10)
				self.rotate_local('x',float(theta)/animate)
			for i in range(animate):
				rate(10)
				self.rotate_local('z',float(psi)/animate)

	def color_axes(self, col):
		for key in self.arrows.keys():
			self.arrows[key].color=col

	def scale_axes(self,size):
		for key in self.arrows.keys():
			self.arrows[key].shaftwidth=self.arrows[key].shaftwidth * .5
			self.arrows[key].headlength=self.arrows[key].headlength * .5
			self.arrows[key].headtwidth=self.arrows[key].headwidth * .5


G= triad()
G.color_axes(color.orange)
G.scale_axes(.5)
B= triad()

B.rotate_euler(90,-90,90,100)

# i=0
# while i<720:
# 	rate(50)
# 	B.rotate_local('x',1)
# 	i+=1