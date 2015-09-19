from visual import *
from math import cos, sin, pi, radians, sqrt, acos, ceil

#functions to do geometry things, like find the intersection of circles/lines etc.
#also used for interpolating G codes to XYZ coords
#basically the things that are not machine-dependent live here.
#to be used with visual python


#Trilateration based on https://en.wikipedia.org/wiki/Trilateration
#P1 P2 P3 are vectors the the centers of the speheres,
#r1, r2, r3 are the radius of the corresponding spheres
#returns two vectors to the intersections- if there is only 1 solution, it is duplicated 
def get_sphere_intersect(P1, P2, P3, r1, r2, r3 ):
	ehatx = ( P2 - P1 )/ mag(P2 - P1)
	i     = dot(ehatx, (P3-P1))
	ehaty = (P3 - P1 - i * ehatx)/mag(P3-P1-i*ehatx)
	ehatz = cross(ehatx, ehaty)
	d = mag(P2-P1)
	j = dot( ehaty, (P3-P1))
	x = (r1**2 - r2**2 + d**2)/(2.0*d)
	y = ((r1**2 - r3**2 + i**2 + j**2) / (2.0 * j)) - (i * x / j)

	try:
		z = - sqrt ( r1**2 - x**2 - y**2 )
	except ValueError:
		print ("get_sphere_intersect says: position not reachable! Spheres do not intersect.")
		return False

	res = [P1 + x*ehatx + y* ehaty + z*ehatz , P1 + x*ehatx + y* ehaty - z*ehatz]
	return res


#R is vector to center of sphere, r is radius of sphere
#P is vector to line, pvec is a vector paralell to the line (ie line=P+t*norm(pvec))
def solve_sphere_line_intersect(R,r,P,pvec):
	yhat=norm(pvec)
	xhat=norm(cross(cross(R-P,pvec),pvec))

	#line-point distance d
	d   = dot(P-R,xhat)/mag(xhat)

	#if d>r, there is no way the two intersect, and acos will return ValueError as range of acos is [1:-1]
	if(d>r):
		print("solve_sphere_line_intersect says: line and sphere do not intersect!")
		return False

	theta = acos(d/r)

	#NB: there are frequently two solutions to this problem
	solution = [R+ r*cos(theta)*xhat +r*sin(theta)*yhat, R+ r*cos(-theta)*xhat +r*sin(-theta)*yhat]

	return solution

#takes a line in point-vector format line(t)=linepoint+t||lineaxis||
#and a point on the line, point.  Returns t for that point 
def get_line_scalar(self, point, linepoint, lineaxis):
		ret=mag(point-linepoint)#/mag(norm(lineaxis))==1
		
		#check to make sure the point is on the line
		if linepoint+ret*norm(lineaxis) == point:
			return ret
		else:
			print ("get_line_scalar says: point not on line!")
			return False

#takes two vectors A and B, and scalar seg.  segment length of interpolated coords to be < seg
#returns a list of vectors pointing at the xyz coords 
def plan_G0(self,A,B,seg):
		ret=[]
		Mhat=norm(A-B) #this is the normal vector from A to B

		#largest_seg is the largest segment length that evenly divides the total distance that is smaller than self.seg
		largest_seg = mag(A-B)/ceil(mag(A-B)/self.seg)

		#return points along the line A+i*Mhat, starting at A ending at B
		
		for i in range(int((mag(A-B)/largest_seg)+1)):
			ret.append(A-i*largest_seg*Mhat)
		
		return ret