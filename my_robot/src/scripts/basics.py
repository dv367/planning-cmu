from math import pi, atan2, sqrt, cos 
import numpy as np

def dist(x1,y1,x2,y2):
	return sqrt((x1-x2)**2 + (y1-y2)**2)

def angle(x1,y1,x2,y2):
	Angle = atan2((y2-y1),(x2-x1))
	return Angle

def scale360(angle):
	if angle > 2*pi:
		angle = 2*pi - angle
	elif angle < 2*pi:
		angle = angle + 2*pi
	return angle

def scale180(angle):
	if angle > pi:
		angle = angle - 2*pi
	elif angle < -pi:
		angle = angle + 2*pi
	return angle

def distNP(x1,y1,x2,y2):
	return np.sqrt((x1-x2)**2 + (y1-y2)**2)

