#A simple script to simulate a serial x-on-y-on-z machine, a la bridgeport

from visual import *
from math import cos, sin, pi, radians, sqrt

zlen=50
xlen=36
ylen=20

zbox=box(pos= (0, 0, 0), axis = (0,0,zlen), length = zlen, height = zlen/2.0, width = zlen/2.0)